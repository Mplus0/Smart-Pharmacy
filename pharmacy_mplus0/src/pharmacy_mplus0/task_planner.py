# -*- coding: utf-8 -*-
"""Task selection and round-plan building for board1 detections.

任务规划模块只做"识别结果 -> 本轮任务计划"的纯逻辑转换。
它不订阅 ROS、不导航、不播报，也不关心 TCP。

这样做的好处是：
- 可以用假数据单独测试策略。
- 识别节点只负责识别，不参与比赛业务决策。
- 后续预留双车协作时，只需要在 select_best 里加入排除逻辑。
"""

from pharmacy_mplus0.constants import (
    DEFAULT_VISIT_ORDER,
    LAB_WINDOW_TO_SAMPLE_TYPE,
    VALID_QR_CODES,
)
from pharmacy_mplus0.models import Board1Detection, RoundPlan, make_board1_detection


class TaskPlanner(object):
    """Turn board1 detections into a single round delivery plan."""

    def __init__(self, visit_order=None, lab_window_to_sample_type=None):
        # visit_order 和 lab_window_to_sample_type 支持外部注入。
        # 后续从 YAML 读取配置后，可以不用改代码就替换策略表。
        self.visit_order = dict(visit_order or DEFAULT_VISIT_ORDER)
        self.lab_window_to_sample_type = dict(
            lab_window_to_sample_type or LAB_WINDOW_TO_SAMPLE_TYPE
        )

    def select_best(self, detections, excluded_box=None):
        """Select the highest-value detection, preferring more samples.

        当前策略优先配送样本数最多的二维码：
        ABC 优先，其次 AB/AC/BC，最后 A/B/C。
        如果样本数相同，选择方框索引更小的任务，使结果稳定可预测。

        excluded_box 用于未来双车方案：如果另一辆车正在处理某个方框，
        本车可以临时排除该方框。
        """
        normalized = []
        for detection in detections or []:
            item = self._coerce_detection(detection)
            if excluded_box is not None and item.box_index == int(excluded_box):
                continue
            normalized.append(item)
        if not normalized:
            return None
        return sorted(normalized, key=self._priority_key)[0]

    def build_round_plan(self, detection):
        """Build a RoundPlan from one Board1Detection.

        这里把识别结果扩展成主控可直接执行的计划：
        - code 决定要去哪些体检窗口。
        - lab_window 决定最终化验窗口。
        - lab_window 决定样本类型。
        - visit_order 决定体检窗口访问顺序。
        """
        item = self._coerce_detection(detection)
        if item.code not in VALID_QR_CODES:
            raise ValueError("unsupported QR code: %s" % item.code)
        if item.code not in self.visit_order:
            raise ValueError("missing visit order for QR code: %s" % item.code)
        if item.lab_window not in self.lab_window_to_sample_type:
            raise ValueError("unsupported lab window: %s" % item.lab_window)
        return RoundPlan(
            code=item.code,
            lab_window=item.lab_window,
            sample_type=self.lab_window_to_sample_type[item.lab_window],
            exam_windows=list(self.visit_order[item.code]),
            detection=item,
        )

    def plan_best_round(self, detections, excluded_box=None):
        """Select the best detection and build a RoundPlan for it.

        这是主控最常用的入口：给一批识别结果，直接得到一轮任务。
        没有可用识别结果时返回 None，由主控决定重试或容错。
        """
        best = self.select_best(detections, excluded_box=excluded_box)
        if best is None:
            return None
        return self.build_round_plan(best)

    def from_cam_return(self, data):
        """Convert old /cam_return data into a Board1Detection.

        旧格式: [is_C, is_A, is_B, count, box_idx, error_code]。
        注意旧脚本前三位顺序是 C、A、B，不是 A、B、C。
        box_idx 是 0 基索引，对应化验窗口 1-4。
        """
        if data is None or len(data) < 5:
            raise ValueError("cam_return requires at least 5 integers")
        is_c, is_a, is_b = [int(data[0]), int(data[1]), int(data[2])]
        box_index = int(data[4])
        code = ""
        if is_a:
            code += "A"
        if is_b:
            code += "B"
        if is_c:
            code += "C"
        detection = make_board1_detection(code, box_index)
        expected_count = int(data[3])
        if expected_count != detection.sample_count:
            raise ValueError(
                "cam_return count mismatch: got %d, expected %d"
                % (expected_count, detection.sample_count)
            )
        return detection

    def _coerce_detection(self, detection):
        # 为了方便测试和后续节点接入，这里接受三种输入：
        # 1. Board1Detection。
        # 2. dict，例如 {"code": "AB", "box_index": 1}。
        # 3. tuple/list，例如 ("AB", 1)。
        if isinstance(detection, Board1Detection):
            return detection
        if isinstance(detection, dict):
            if "box_index" in detection:
                return make_board1_detection(detection["code"], detection["box_index"])
            if "lab_window" in detection:
                return make_board1_detection(
                    detection["code"], int(detection["lab_window"]) - 1
                )
        if isinstance(detection, (list, tuple)) and len(detection) >= 2:
            return make_board1_detection(detection[0], detection[1])
        raise ValueError("unsupported detection: %r" % (detection,))

    def _priority_key(self, detection):
        # Python 排序默认从小到大，所以样本数取负数实现"越多越靠前"。
        # box_index 和 code 只是用于同分时稳定排序。
        return (-detection.sample_count, detection.box_index, detection.code)
