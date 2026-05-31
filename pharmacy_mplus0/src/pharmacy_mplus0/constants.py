# -*- coding: utf-8 -*-
"""Shared constants for the smart pharmacy competition flow.

本文件只放"不会随运行状态变化"的比赛常量：
- 窗口编号、样本类型、话题名。
- 主控状态机状态名。
- 默认体检窗口访问顺序。

把这些字符串集中在这里，可以避免后续主控、上报、导航等模块里
反复硬编码同一批值，降低拼写错误导致的现场问题。
"""

# 体检区窗口。二维码字母只允许由 A/B/C 组合而成。
EXAM_WINDOWS = ("A", "B", "C")

# 化验区窗口。识别板一的方框 0-3 会映射为化验窗口 1-4。
LAB_WINDOWS = ("1", "2", "3", "4")

# 化验窗口展示名称，后续播报和裁判可见信息会复用这里。
LAB_WINDOW_NAMES = {
    "1": "血常规窗口",
    "2": "体液窗口",
    "3": "免疫检测窗口",
    "4": "激素检验窗口",
}

# 样本类型名称。当前规则中，化验窗口和样本类型是一一对应的。
SAMPLE_TYPE_NAMES = {
    "1": "静脉血样本",
    "2": "唾液样本",
    "3": "组织样本",
    "4": "血浆样本",
}

# 化验窗口到样本类型的映射。
# 这里先保留字符串编号，便于 TCP/ROS 消息里保持简洁稳定。
LAB_WINDOW_TO_SAMPLE_TYPE = {
    "1": "1",
    "2": "2",
    "3": "3",
    "4": "4",
}

# 支持的二维码内容。normalize_code 会把乱序字母整理成这些标准值。
VALID_QR_CODES = ("A", "B", "C", "AB", "AC", "BC", "ABC")

# 固定最短访问顺序表，来自当前比赛策略。
# 例如 AC 不按字母顺序走 A->C，而是按场地经验走 C->A。
DEFAULT_VISIT_ORDER = {
    "A": ["A"],
    "B": ["B"],
    "C": ["C"],
    "AB": ["A", "B"],
    "AC": ["C", "A"],
    "BC": ["C", "B"],
    "ABC": ["C", "A", "B"],
}

# ROS 话题名集中定义，后续脚本中直接引用，避免散落字符串。
TOPIC_CAM_RETURN = "/cam_return"
TOPIC_ALL_QRCODES = "/all_qrcodes"
TOPIC_BOARD1_DETECTIONS = "/board1_detections"
TOPIC_CV1_RESULT = "/cv1_result"
TOPIC_BOARD2_STATUS = "/board2_status"
TOPIC_CV2_RESULT = "/cv2_result"
TOPIC_CURRENT_TASK = "/current_task"
TOPIC_CURRENT_QR_TASK = "/current_qr_task"
TOPIC_ANNOUNCE_REQUEST = "/announce_request"
TOPIC_RESET_DETECTION = "/reset_detection"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_ODOM = "/odom"

# 主控状态机状态名。第一阶段先定义，后续 main_controller.py 直接复用。
STATE_INIT = "INIT"
STATE_GOTO_BOARD1 = "GOTO_BOARD1"
STATE_AT_BOARD1 = "AT_BOARD1"
STATE_GOTO_EXAM = "GOTO_EXAM"
STATE_AT_EXAM = "AT_EXAM"
STATE_GOTO_BOARD2 = "GOTO_BOARD2"
STATE_AT_BOARD2 = "AT_BOARD2"
STATE_PASS_BOARD2 = "PASS_BOARD2"
STATE_GOTO_LAB = "GOTO_LAB"
STATE_AT_LAB = "AT_LAB"
STATE_DONE_ROUND = "DONE_ROUND"

# 裁判上报中的"路上/起点/识别区"通用任务状态。
TASK_ROAD = "R"

# navigation_client 使用的 move_base action 名称和 clear_costmaps 服务名。
NAV_ACTION_NAME = "move_base"
NAV_CLEAR_COSTMAPS_SRV = "/move_base/clear_costmaps"

# 默认导航超时。当 strategy.yaml 不可用或未提供 nav_default_seconds 时使用。
NAV_DEFAULT_TIMEOUT_SEC = 40.0

# 识别板二忙碌秒数范围。
BOARD2_BUSY_SECONDS_MIN = 5
BOARD2_BUSY_SECONDS_MAX = 10

# 化验窗口编号 → 音频 event_id 片段。
LAB_WINDOW_AUDIO_KEYS = {
    "1": "blood",
    "2": "bodyfluid",
    "3": "immunity",
    "4": "hormone",
}

# 样本类型编号 → 音频 event_id 片段。
SAMPLE_TYPE_AUDIO_KEYS = {
    "1": "venous_blood",
    "2": "saliva",
    "3": "tissue",
    "4": "plasma",
}
