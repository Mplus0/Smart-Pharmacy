# -*- coding: utf-8 -*-
from pharmacy_iscc.config.settings import COMMON, get_car_id, get_car_config

CAR_ID = get_car_id(default=1)
CAR_CONFIG = get_car_config(CAR_ID)
STATES = COMMON["states"]
BOARD1_CFG = COMMON["board1"]
BOARD2_CFG = COMMON["board2"]
DETECT_CFG = COMMON["detect"]

STATE_WAIT_TURN = STATES["WAIT_TURN"]
STATE_GO_TO_BOARD1 = STATES["GO_TO_BOARD1"]
STATE_BOARD1_RECOGNIZING = STATES["BOARD1_RECOGNIZING"]
STATE_GO_TO_PICKUP_WINDOWS = STATES["GO_TO_PICKUP_WINDOWS"]
STATE_GO_TO_BOARD2 = STATES["GO_TO_BOARD2"]
STATE_BOARD2_RECOGNIZING = STATES["BOARD2_RECOGNIZING"]
STATE_GO_TO_LAB_WINDOW = STATES["GO_TO_LAB_WINDOW"]
STATE_GO_BACK_HOME = STATES["GO_BACK_HOME"]

approxPolyDP_epslion = BOARD1_CFG["approx_poly_dp_epsilon"]
wh_rate = BOARD1_CFG["wh_rate"]
min_area = BOARD1_CFG["min_area"]
max_area = BOARD1_CFG["max_area"]
min_center_distance = BOARD1_CFG["min_center_distance"]
BOARD1_DINGWEIKUANG_MIN = BOARD1_CFG["locator_min"]
BOARD1_DINGWEIKUANG_MAX = BOARD1_CFG["locator_max"]
BOARD1_JIEGUO_MIN = BOARD1_CFG["result_min"]
BOARD1_STABLE_FRAMES = BOARD1_CFG["stable_frames"]

BOARD1_WINDOW_DETECT_ENABLE = BOARD1_CFG.get("window_detect_enable", True)
BOARD1_WINDOW_MIN_AREA = BOARD1_CFG.get("window_min_area", 2500)
BOARD1_WINDOW_MAX_AREA = BOARD1_CFG.get("window_max_area", 80000)
BOARD1_WINDOW_SQUARE_WH_RATE = BOARD1_CFG.get("window_square_wh_rate", 0.45)
BOARD1_WINDOW_DEDUP_DISTANCE = BOARD1_CFG.get("window_dedup_distance", 55)
BOARD1_WINDOW_WARP_SIZE = BOARD1_CFG.get("window_warp_size", 240)
BOARD1_WINDOW_INNER_MARGIN_RATE = BOARD1_CFG.get("window_inner_margin_rate", 0.16)
BOARD1_WINDOW_COMPLETE_FROM_THREE = BOARD1_CFG.get("window_complete_from_three", True)
BOARD1_REQUIRE_FOUR_WINDOWS = BOARD1_CFG.get("require_four_windows", True)
BOARD1_WINDOW_GRID_MAX_SCORE = float(BOARD1_CFG.get("window_grid_max_normalized_score", 0.75))
BOARD1_DIRECT_DECODE_ENABLE = BOARD1_CFG.get("direct_decode_enable", True)
BOARD1_DIRECT_MIN_QR_COUNT = BOARD1_CFG.get("direct_min_qr_count", 4)
BOARD1_DIRECT_DEDUP_DISTANCE = BOARD1_CFG.get("direct_dedup_distance", 45)
BOARD1_VALID_TEXT = set(["A", "B", "C", "AB", "AC", "BC", "ABC"])

fps = 0
detect_num = 0

CAMERA_URL = CAR_CONFIG["camera_url"]
FRAME_ROTATE_ANGLE = DETECT_CFG["frame_rotate_angle"]
DROP_FRAME_COUNT = DETECT_CFG["drop_frame_count"]
LIMIT_RATE_HZ = DETECT_CFG["limit_rate_hz"]
IDLE_GRAB_SLEEP_SEC = DETECT_CFG["idle_grab_sleep_sec"]
CAMERA_RECONNECT_SLEEP_SEC = DETECT_CFG["camera_reconnect_sleep_sec"]

# 图像输入源：默认直接订阅 ROS Image，HTTP 视频流只作为兜底。
IMAGE_SOURCE = DETECT_CFG.get("image_source", "ros_topic")
CAMERA_TOPIC = DETECT_CFG.get("camera_topic", "/camera/rgb/image_raw")
CAMERA_FALLBACK_TO_HTTP = DETECT_CFG.get("camera_fallback_to_http", True)
CAMERA_MAX_AGE_SEC = DETECT_CFG.get("camera_max_age_sec", 0.5)
ROS_IMAGE_BUFF_SIZE = DETECT_CFG.get("ros_image_buff_size", 2 ** 24)

BOARD2_TEMPLATE_DIR = COMMON["paths"]["board2_template_dir"]
BOARD2_MATCH_THRESHOLD = BOARD2_CFG["match_threshold"]
BOARD2_STABLE_FRAMES = BOARD2_CFG["stable_frames"]
BOARD2_SCORE_MARGIN = BOARD2_CFG["score_margin"]
BOARD2_PUBLISH_INTERVAL = BOARD2_CFG["publish_interval"]
BOARD2_ROI = BOARD2_CFG["roi"]
BOARD2_TEMPLATE_SCALES = BOARD2_CFG["template_scales"]
BOARD2_FORCE_LABEL_FOR_DEBUG = BOARD2_CFG.get("force_label_for_debug")
BOARD2_FORCE_SCORE_FOR_DEBUG = BOARD2_CFG.get("force_score_for_debug", 1.0)
BOARD2_RESULT_SOURCE = BOARD2_CFG.get("result_source", "vision")

