# -*- coding: utf-8 -*-
"""Small immutable data models used by the first-stage pure logic.

本文件定义模块之间传递的数据形状。

旧脚本里经常直接传 list 或 dict，例如 /cam_return 的
[is_C, is_A, is_B, count, box_idx]。这种方式很难看出每个位置的含义。
这里用 namedtuple 给字段命名，让主控、任务规划、识别模块之间的接口
更清楚，也保持 Python 2/ROS Melodic 兼容。
"""

from collections import namedtuple


# 识别板一的一个二维码结果。
# code: 二维码内容，如 A、AB、ABC。
# box_index: 识别板一方框索引，0-3。
# lab_window: 方框对应的化验窗口，1-4。
# sample_count: 本轮要取的样本数，也就是 code 的字母数量。
Board1Detection = namedtuple(
    "Board1Detection",
    ["code", "box_index", "lab_window", "sample_count"],
)

# 主控执行一轮任务所需的完整计划。
# exam_windows 已经按策略表排好顺序，主控只需要依次导航。
RoundPlan = namedtuple(
    "RoundPlan",
    ["code", "lab_window", "sample_type", "exam_windows", "detection"],
)

# 识别板二状态。
# wait_seconds 为 0 表示空闲，5-10 表示忙碌等待秒数。
Board2Status = namedtuple(
    "Board2Status",
    ["wait_seconds", "is_idle", "raw"],
)


def make_board1_detection(code, box_index):
    """Create a normalized Board1Detection from QR text and a zero-based box.

    识别节点只需要给出二维码文字和所在方框。
    这里统一完成：
    - 二维码文字标准化。
    - 方框合法性检查。
    - 方框 0-3 到化验窗口 1-4 的转换。
    """
    normalized_code = normalize_code(code)
    normalized_box = int(box_index)
    if normalized_box < 0 or normalized_box > 3:
        raise ValueError("box_index must be in [0, 3], got %s" % box_index)
    lab_window = str(normalized_box + 1)
    return Board1Detection(
        code=normalized_code,
        box_index=normalized_box,
        lab_window=lab_window,
        sample_count=len(normalized_code),
    )


def normalize_code(code):
    """Normalize QR text to sorted A/B/C letters, rejecting unknown values.

    视觉识别结果可能带大小写差异，甚至带额外字符。
    这里只保留 A/B/C，去重后按 A、B、C 顺序输出标准二维码。
    例如 "ca" 会标准化为 "AC"。
    """
    if code is None:
        raise ValueError("QR code cannot be None")
    letters = []
    for char in str(code).upper():
        if char in ("A", "B", "C") and char not in letters:
            letters.append(char)
    if not letters:
        raise ValueError("QR code has no valid A/B/C letters: %r" % code)
    order = {"A": 0, "B": 1, "C": 2}
    letters.sort(key=lambda item: order[item])
    return "".join(letters)


def board2_status_from_text(text):
    """Convert WAIT-0 / WAIT-5..WAIT-10 text into a Board2Status.

    识别板二节点后续会发布 WAIT-0 或 WAIT-5..WAIT-10。
    主控不直接解析字符串，而是先转成 Board2Status，便于判断空闲/忙碌。
    """
    raw = str(text).strip().upper()
    if raw in ("WAIT-0", "IDLE", "0"):
        return Board2Status(wait_seconds=0, is_idle=True, raw="WAIT-0")
    if raw.startswith("WAIT-"):
        wait_seconds = int(raw.split("-", 1)[1])
        return Board2Status(
            wait_seconds=wait_seconds,
            is_idle=(wait_seconds == 0),
            raw="WAIT-%d" % wait_seconds,
        )
    raise ValueError("unsupported board2 status: %r" % text)
