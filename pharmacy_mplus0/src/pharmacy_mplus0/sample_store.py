# -*- coding: utf-8 -*-
"""Track samples carried during one delivery round.

SampleStore 只记录"本轮车上带了什么样本"。
它不负责导航、不负责播报、不负责判断去哪个化验窗口。

比赛规则要求一轮只配送一种样本类型，所以这里会主动阻止混装。
这样主控写错流程时，逻辑测试阶段就能暴露问题。
"""

from pharmacy_mplus0.constants import EXAM_WINDOWS, LAB_WINDOW_TO_SAMPLE_TYPE


class SampleStore(object):
    """A one-round sample store that enforces a single sample type."""

    def __init__(self, lab_window_to_sample_type=None):
        # 映射表支持注入，后续可从 YAML 读取，便于现场改规则或调试。
        self.lab_window_to_sample_type = dict(
            lab_window_to_sample_type or LAB_WINDOW_TO_SAMPLE_TYPE
        )
        self.clear()

    def clear(self):
        # 每轮开始、投递完成后都清空携带状态。
        self.sample_type = None
        self.exam_windows = []

    def add_sample(self, exam_window, sample_type):
        # 到达体检窗口后调用。
        # 第一次取样会锁定本轮样本类型；后续取样必须同类型。
        window = str(exam_window).upper()
        sample_type = str(sample_type)
        if window not in EXAM_WINDOWS:
            raise ValueError("unsupported exam window: %s" % exam_window)
        if self.sample_type is None:
            self.sample_type = sample_type
        elif self.sample_type != sample_type:
            raise ValueError(
                "cannot mix sample types %s and %s"
                % (self.sample_type, sample_type)
            )
        if window not in self.exam_windows:
            # 同一个窗口重复调用时不重复计数，避免主控重试导致样本数膨胀。
            self.exam_windows.append(window)
        return len(self.exam_windows)

    def count_for_lab(self, lab_window):
        # 只有目标化验窗口需要的样本类型与车上样本类型一致时，才返回数量。
        # 如果主控误导航到错误化验窗口，这里会返回 0。
        lab_window = str(lab_window)
        expected = self.lab_window_to_sample_type.get(lab_window)
        if expected is None or self.sample_type != expected:
            return 0
        return len(self.exam_windows)

    def deliver_to_lab(self, lab_window):
        # 投递时先计算本窗口可投递数量，再清空车上样本，进入下一轮。
        count = self.count_for_lab(lab_window)
        self.clear()
        return count

    def carried_windows(self):
        # 返回副本，避免外部代码直接修改内部列表。
        return list(self.exam_windows)
