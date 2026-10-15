#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
F1_detect_code.py

视觉识别节点：
1. 识别板一：定位二维码板，读取四个区域二维码，选择最优方案并发布 /cam_return；
2. 识别板二：通过模板匹配识别空闲/忙碌和等待时间，发布 /board2_return。

备注：新增图像识别的判断算法，防止一直识别造成卡顿。订阅导航状态current_nav_state，只有在对应状态才进行图像处理，其他时间休眠等待。
    新增识别策略，减少摄像头的重复读取和图像处理，避免不必要的计算和卡顿。
    新增防止旧帧补偿策略，防止重复使用旧帧识别
    新增识别结果稳定性判断，连续多帧结果一致后才发布，避免单帧误识别导致的错误发布。且防止重复发布，减少系统负担和卡顿。

"""

import os
import time
import math
import random
import json
import threading
from enum import Enum
from collections import deque

import cv2
import cv_bridge
import numpy as np
import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32MultiArray, Int32, String

from dual_car_config import COMMON, get_car_config, get_car_id
from board1_selection import normalize_all_text, safe_to_str, select_board1_from_all_text

os.environ["LANG"] = "zh_CN.UTF-8"

# =========================
# 集中配置
# =========================

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

# =========================
# 全局运行变量
# =========================

pub_flag = None
pub_board2 = None
pub_board1_all_text = None
board2_templates = {}
board1_history = deque(maxlen=BOARD1_STABLE_FRAMES)
board2_history = deque(maxlen=BOARD2_STABLE_FRAMES)
board1_published = False
board2_published = False
board1_all_text_seq = 0
last_board2_publish_time = 0.0
last_board2_publish_label = None
previous_nav_state = None
current_nav_state = STATE_GO_TO_BOARD1

ros_image_bridge = None
ros_image_sub = None
latest_ros_frame = None
latest_ros_frame_stamp = None
latest_ros_frame_recv_time = None
latest_ros_frame_lock = threading.Lock()


# =========================
# 通用工具函数
# =========================

def distance(point1, point2):
    """计算两个点之间的欧氏距离。"""
    return math.sqrt((point1[0] - point2[0]) ** 2 +
                     (point1[1] - point2[1]) ** 2)


def find_contours(binary_img, mode, method):
    """兼容不同 OpenCV 版本的 findContours 返回值。"""
    result = cv2.findContours(binary_img, mode, method)
    if len(result) == 3:
        _, contours, _ = result
    else:
        contours, _ = result
    return contours


def rotate_frame(frame, angle):
    """按原代码角度旋转图像。"""
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    return cv2.warpAffine(frame, matrix, (width, height))


# =========================
# 识别板二：模板匹配
# =========================

def preprocess_board2_img(img):
    """
    识别板二模板匹配预处理：
    1. 灰度化；
    2. 高斯滤波；
    3. Otsu 二值化。
    """
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary


def load_board2_templates():
    """
    加载识别板二模板。

    模板命名：
    - free.png
    - busy_5.png ~ busy_10.png
    """
    templates = {}

    template_files = {
        "free": "free.png",
        "busy_5": "busy_5.png",
        "busy_6": "busy_6.png",
        "busy_7": "busy_7.png",
        "busy_8": "busy_8.png",
        "busy_9": "busy_9.png",
        "busy_10": "busy_10.png"
    }

    for label, filename in template_files.items():
        path = os.path.join(BOARD2_TEMPLATE_DIR, filename)
        img = cv2.imread(path)

        if img is None:
            rospy.logwarn("识别板二模板未找到: %s", path)
            continue

        templates[label] = preprocess_board2_img(img)

    rospy.logwarn("识别板二模板加载完成，数量: %d", len(templates))
    return templates


def crop_board2_roi(frame):
    """裁剪识别板二 ROI，当前默认全图。"""
    h, w = frame.shape[:2]
    x1_rate, y1_rate, x2_rate, y2_rate = BOARD2_ROI

    x1 = int(w * x1_rate)
    y1 = int(h * y1_rate)
    x2 = int(w * x2_rate)
    y2 = int(h * y2_rate)

    return frame[y1:y2, x1:x2]


def match_one_template(search_img, template_img):
    """单模板多尺度匹配，返回最高匹配分数。"""
    best_score = -1.0
    sh, sw = search_img.shape[:2]

    for scale in BOARD2_TEMPLATE_SCALES:
        tw = int(template_img.shape[1] * scale)
        th = int(template_img.shape[0] * scale)

        if tw <= 10 or th <= 10:
            continue
        if tw >= sw or th >= sh:
            continue

        resized_template = cv2.resize(template_img, (tw, th))
        result = cv2.matchTemplate(search_img, resized_template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, _ = cv2.minMaxLoc(result)

        if max_val > best_score:
            best_score = max_val

    return best_score


def detect_board2_status(frame):
    """检测识别板二状态，返回 label 和 score。"""
    global board2_templates

    if len(board2_templates) == 0:
        return None, 0.0

    roi = crop_board2_roi(frame)
    search_img = preprocess_board2_img(roi)

    scores = []
    for label, template_img in board2_templates.items():
        score = match_one_template(search_img, template_img)
        scores.append((label, score))

    if len(scores) == 0:
        return None, 0.0

    scores.sort(key=lambda x: x[1], reverse=True)
    best_label, best_score = scores[0]

    if len(scores) >= 2:
        second_label, second_score = scores[1]
    else:
        second_label, second_score = None, -1.0

    if (best_score >= BOARD2_MATCH_THRESHOLD and
            (best_score - second_score) >= BOARD2_SCORE_MARGIN):
        return best_label, best_score

    rospy.loginfo_throttle(
        1.0,
        "识别板二匹配不稳定: best=%s %.3f second=%s %.3f",
        best_label, best_score, second_label, second_score
    )
    return None, best_score


def board2_label_to_msg(label):
    """
    将模板标签转换为 /board2_return 消息。

    msg.data = [state, wait_time]
    state = 0 表示空闲；state = 1 表示忙碌。
    """
    msg = Int32MultiArray()

    if label == "free":
        msg.data = [0, 0]
        return msg

    if label.startswith("busy_"):
        try:
            wait_time = int(label.split("_")[1])
            msg.data = [1, wait_time]
            return msg
        except Exception:
            return None

    return None

def read_latest_frame(cap, drop_count=DROP_FRAME_COUNT):
    '''防止旧帧影响识别'''
    if cap is None:
        return False, None
    for _ in range(drop_count):
        cap.grab()
    return cap.read()


def ros_image_cb(msg):
    """ROS 原始图像回调：只缓存最新帧，不在回调里做识别。"""
    global latest_ros_frame
    global latest_ros_frame_stamp
    global latest_ros_frame_recv_time
    global ros_image_bridge

    if ros_image_bridge is None:
        return

    try:
        frame = ros_image_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logwarn_throttle(1.0, "ROS图像转换失败: %s", str(e))
        return

    try:
        stamp = msg.header.stamp
    except Exception:
        stamp = rospy.Time.now()

    with latest_ros_frame_lock:
        latest_ros_frame = frame.copy()
        latest_ros_frame_stamp = stamp
        latest_ros_frame_recv_time = time.time()


def ros_compressed_image_cb(msg):
    """ROS 压缩图像回调：支持 camera_topic 指向 /compressed 话题。"""
    global latest_ros_frame
    global latest_ros_frame_stamp
    global latest_ros_frame_recv_time

    try:
        data = np.fromstring(msg.data, np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception as e:
        rospy.logwarn_throttle(1.0, "ROS压缩图像转换失败: %s", str(e))
        return

    if frame is None:
        return

    try:
        stamp = msg.header.stamp
    except Exception:
        stamp = rospy.Time.now()

    with latest_ros_frame_lock:
        latest_ros_frame = frame.copy()
        latest_ros_frame_stamp = stamp
        latest_ros_frame_recv_time = time.time()


def init_ros_image_subscriber():
    """初始化 ROS 图像订阅。默认订阅 /camera/rgb/image_raw。"""
    global ros_image_bridge
    global ros_image_sub

    if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"]:
        rospy.logwarn("当前图像输入源为HTTP视频流: %s", CAMERA_URL)
        return

    if IMAGE_SOURCE == "ros_compressed" or CAMERA_TOPIC.endswith("/compressed"):
        ros_image_sub = rospy.Subscriber(
            CAMERA_TOPIC,
            CompressedImage,
            ros_compressed_image_cb,
            queue_size=1,
            buff_size=ROS_IMAGE_BUFF_SIZE
        )
        rospy.logwarn("图像输入源: ROS CompressedImage topic=%s, HTTP兜底=%s",
                      CAMERA_TOPIC, CAMERA_FALLBACK_TO_HTTP)
    else:
        ros_image_bridge = cv_bridge.CvBridge()
        ros_image_sub = rospy.Subscriber(
            CAMERA_TOPIC,
            Image,
            ros_image_cb,
            queue_size=1,
            buff_size=ROS_IMAGE_BUFF_SIZE
        )
        rospy.logwarn("图像输入源: ROS Image topic=%s, HTTP兜底=%s",
                      CAMERA_TOPIC, CAMERA_FALLBACK_TO_HTTP)


def get_latest_ros_frame():
    """从 ROS 图像缓存取最新帧。返回 ok, frame, reason。"""
    with latest_ros_frame_lock:
        if latest_ros_frame is None:
            return False, None, "empty"

        age = 0.0
        if latest_ros_frame_recv_time is not None:
            age = time.time() - latest_ros_frame_recv_time

        if CAMERA_MAX_AGE_SEC > 0 and age > CAMERA_MAX_AGE_SEC:
            return False, None, "stale %.3fs" % age

        return True, latest_ros_frame.copy(), "ok"


def open_http_capture():
    """打开 HTTP 视频流，用于 HTTP 模式或 ROS 模式下的兜底。"""
    cap = cv2.VideoCapture(CAMERA_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def read_frame_from_configured_source(cap):
    """按配置读取图像：优先 ROS topic，必要时回退到 HTTP 视频流。"""
    if IMAGE_SOURCE in ["ros_topic", "ros_image", "ros_compressed"]:
        ok, frame, reason = get_latest_ros_frame()
        if ok:
            return True, frame, "ros"

        if not CAMERA_FALLBACK_TO_HTTP:
            rospy.logwarn_throttle(1.0, "ROS图像暂不可用: %s", reason)
            return False, None, "ros:%s" % reason

        rospy.logwarn_throttle(2.0, "ROS图像暂不可用: %s，临时使用HTTP视频流兜底", reason)

    hx, frame = read_latest_frame(cap, drop_count=DROP_FRAME_COUNT)
    return hx, frame, "http"

def nav_state_cb(msg):
    """更新当前导航状态，供图像处理使用。"""
    global current_nav_state
    global previous_nav_state
    global board1_history
    global board2_history
    global board1_published
    global board2_published
    global last_board2_publish_time
    global last_board2_publish_label

    previous_nav_state = current_nav_state
    current_nav_state = msg.data

    if previous_nav_state != STATE_BOARD1_RECOGNIZING and current_nav_state == STATE_BOARD1_RECOGNIZING:
        board1_history.clear()
        board1_published = False
        rospy.loginfo("进入识别板一状态，清空历史缓存，允许发布一次识别结果")

    if previous_nav_state != STATE_BOARD2_RECOGNIZING and current_nav_state == STATE_BOARD2_RECOGNIZING:
        board2_history.clear()
        board2_published = False
        last_board2_publish_time = 0.0
        last_board2_publish_label = None
        rospy.loginfo("进入识别板二状态，清空历史缓存，允许发布一次识别结果")



# =========================
# 识别板一：二维码识别
# =========================

def filter_contours_by_area(contours):
    """筛选面积在指定范围内的轮廓。"""
    filtered = []
    for contour in contours:
        moment = cv2.moments(contour)
        if moment['m00'] < min_area or moment['m00'] > max_area:
            continue
        filtered.append(contour)
    return filtered


def filter_quadrangles(contours):
    """筛选四边形轮廓。"""
    quadrangles = []
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, approxPolyDP_epslion * peri, True)
        if len(approx) == 4:
            quadrangles.append(approx)
    return quadrangles


def filter_square_like_contours(quadrangles):
    """根据长宽比筛选近似正方形。"""
    squares = []
    for hull in quadrangles:
        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect
        if (w + h) > 0 and abs(w - h) / (w + h) < wh_rate:
            squares.append(hull)
    return squares


def find_locator_squares(squares):
    """根据正方形包含关系筛选二维码定位框。"""
    dingweikuang = []
    jieguo = []

    for i in range(len(squares) - 1):
        M1 = cv2.moments(squares[i])
        if M1['m00'] == 0:
            continue

        cx1 = int(M1['m10'] / M1['m00'])
        cy1 = int(M1['m01'] / M1['m00'])

        for j in range(i + 1, len(squares)):
            M2 = cv2.moments(squares[j])
            if M2['m00'] == 0:
                continue

            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])

            res1 = cv2.pointPolygonTest(squares[i], (cx2, cy2), False)
            res2 = cv2.pointPolygonTest(squares[j], (cx1, cy1), False)

            if res1 > 0 and res2 > 0:
                distance_ = distance((cx1, cy1), (cx2, cy2))
                if distance_ < min_center_distance:
                    dingweikuang += [squares[i], squares[j]]
                    jieguo.append(squares[i])

    return dingweikuang, jieguo


def get_board1_perspective(frame, jieguo):
    """根据定位框计算透视变换，返回 600x600 的二维码板图像。"""
    min_x = 1000
    max_x = 0
    min_y = 1000
    max_y = 0

    for i in range(len(jieguo)):
        for j in range(len(jieguo[i])):
            if jieguo[i][j][0][0] < min_x:
                min_x = jieguo[i][j][0][0]
                min_x_index = [i, j]
            if jieguo[i][j][0][0] > max_x:
                max_x = jieguo[i][j][0][0]
                max_x_index = [i, j]
            if jieguo[i][j][0][1] < min_y:
                min_y = jieguo[i][j][0][1]
                min_y_index = [i, j]
            if jieguo[i][j][0][1] > max_y:
                max_y = jieguo[i][j][0][1]
                max_y_index = [i, j]

    x0 = jieguo[min_x_index[0]][min_x_index[1]][0][0]
    y0 = jieguo[min_x_index[0]][min_x_index[1]][0][1]
    x1 = jieguo[max_x_index[0]][max_x_index[1]][0][0]
    y1 = jieguo[max_x_index[0]][max_x_index[1]][0][1]
    x2 = jieguo[min_y_index[0]][min_y_index[1]][0][0]
    y2 = jieguo[min_y_index[0]][min_y_index[1]][0][1]
    x3 = jieguo[max_y_index[0]][max_y_index[1]][0][0]
    y3 = jieguo[max_y_index[0]][max_y_index[1]][0][1]

    real_box_centers = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

    if min_x_index[0] < max_x_index[0]:
        goal_box_centers = [(50, 550), (550, 50), (50, 50), (550, 550)]
    else:
        goal_box_centers = [(50, 50), (550, 550), (550, 50), (50, 550)]

    dst_pts = np.array(goal_box_centers, dtype=np.float32)
    src_pts = np.array(real_box_centers, dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

    return cv2.warpPerspective(frame, matrix, (600, 600))


def decode_qr_text_light(img):
    """轻量二维码解码：保留 v2 体验，不走 v3 的多ROI/多倍率重增强。"""
    if img is None or img.size == 0:
        return ""

    candidates = []
    candidates.append(img)

    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img.copy()
    except Exception:
        return ""

    candidates.append(gray)

    try:
        _, binary_fixed = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        candidates.append(binary_fixed)
    except Exception:
        pass

    try:
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(binary_otsu)
    except Exception:
        pass

    # 轻量放大一版即可，避免 v3 那种多倍、多ROI导致速度和稳定性下降。
    try:
        h, w = gray.shape[:2]
        if h > 0 and w > 0:
            candidates.append(cv2.resize(gray, (w * 2, h * 2), interpolation=cv2.INTER_LINEAR))
    except Exception:
        pass

    for cand in candidates:
        try:
            code_result = decode(cand)
        except Exception:
            continue

        for qr in code_result:
            try:
                data = qr.data.decode("utf-8")
            except Exception:
                data = qr.data

            data = safe_to_str(data)
            if data in BOARD1_VALID_TEXT:
                return data

    return ""


def decode_four_qr_areas(warped_img):
    """裁剪四个二维码区域并解码。旧透视兜底也使用轻量增强解码。"""
    aera1_crop = warped_img[65:255, 65:235]
    aera2_crop = warped_img[65:255, 365:535]
    aera3_crop = warped_img[335:525, 65:235]
    aera4_crop = warped_img[335:525, 365:535]

    crops = [aera1_crop, aera2_crop, aera3_crop, aera4_crop]
    decoded_data = []

    for crop in crops:
        decoded_data.append(decode_qr_text_light(crop))

    return decoded_data


def calculate_qr_lengths(all_data):
    """按照原代码规则计算四个二维码字符串长度。"""
    img_len = [0, 0, 0, 0]

    if len(all_data[0]) < 4:
        img_len[0] = len(all_data[0])
    if len(all_data[1]) < 4:
        img_len[1] = len(all_data[1])
    if len(all_data[2]) < 4:
        img_len[2] = len(all_data[2])
    if len(all_data[3]) < 4:
        img_len[3] = len(all_data[3])

    return img_len

def normalize_qr_data(item):
    """统一二维码识别结果格式，避免 Python2 unicode 编码问题。"""
    return safe_to_str(item)



def detect_error_window(all_data):
    """检测错误二维码窗口编号。"""
    selection = select_board1_from_all_text(all_data)
    if selection is not None:
        return selection["selected_msg"][5]

    normal = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC', '']
    all_text = [normalize_qr_data(item) for item in all_data]
    for idx, text in enumerate(all_text):
        if text not in normal:
            rospy.logwarn("窗口 %d 出现错误信息: %s", idx + 1, text)
            return idx + 1

    return 0


def build_board1_msg(all_data):
    """
    根据四个二维码结果生成识别板一消息。
    注意：这里只生成 msg，不发布。
    """
    selection = select_board1_from_all_text(all_data)
    all_text = [normalize_qr_data(item) for item in all_data]
    rospy.logwarn("识别板一二维码结果: %s", all_text)

    if selection is None:
        rospy.logwarn("识别板一未识别到有效二维码组合，不进入稳定判断: %s", all_text)
        return None

    msg = Int32MultiArray()
    msg.data = selection["selected_msg"]

    rospy.loginfo("识别板一当前候选结果: %s -> %s",
                  selection["selected_text"], msg.data)
    return msg


def publish_board1_all_text(selection):
    """发布识别板一 all_text 共享 JSON。"""
    global board1_all_text_seq

    if not BOARD1_CFG.get("publish_all_text", True):
        return
    if pub_board1_all_text is None:
        return

    if selection is None:
        return

    board1_all_text_seq += 1
    payload = {
        "car_id": int(CAR_CONFIG["car_id"]),
        "seq": int(board1_all_text_seq),
        "all_text": selection["all_text"],
        "selected_index": int(selection["selected_index"]),
        "selected_text": selection["selected_text"],
        "selected_msg": list(selection["selected_msg"]),
    }

    text_msg = String()
    text_msg.data = json.dumps(payload, ensure_ascii=False)

    for _ in range(3):
        pub_board1_all_text.publish(text_msg)
        rospy.sleep(0.03)

    rospy.loginfo("[BOARD1-SHARE] 已发布 all_text: %s", text_msg.data)


def stable_publish_board1(all_data):
    """
    识别板一 all_text 连续多次完全一致后，才选择最优解并发布。
    v2修复点：['', '', '', ''] 这类无效结果不加入稳定历史，避免污染好结果。
    """
    global pub_flag
    global board1_history
    global board1_published

    if board1_published:
        return None

    all_text = normalize_all_text(all_data)
    rospy.logwarn("识别板一二维码结果: %s", all_text)

    selection = select_board1_from_all_text(all_text)
    if selection is None:
        rospy.logwarn("识别板一 all_text 无有效二维码组合，不加入稳定历史: %s", all_text)
        return None

    stable_key = tuple(all_text)
    board1_history.append(stable_key)

    if len(board1_history) < BOARD1_STABLE_FRAMES:
        rospy.loginfo("识别板一有效 all_text 稳定计数: %d/%d",
                      len(board1_history), BOARD1_STABLE_FRAMES)
        return None

    if len(set(board1_history)) != 1:
        rospy.logwarn("识别板一 all_text 连续结果不一致，继续等待稳定: %s",
                      list(board1_history))
        return None

    msg = Int32MultiArray()
    msg.data = selection["selected_msg"]

    pub_flag.publish(msg)
    board1_published = True
    publish_board1_all_text(selection)

    rospy.logwarn("识别板一有效 all_text 连续 %d 次稳定，正式发布: all_text=%s msg=%s",
                  BOARD1_STABLE_FRAMES, all_text, msg.data)

    return msg.data

def stable_publish_board2(label, score):
    """多帧稳定后发布识别板二结果，避免单帧误识别。"""
    global last_board2_publish_time
    global last_board2_publish_label
    global pub_board2
    global board2_published

    if board2_published:
        return None

    if label is None:
        board2_history.clear()
        return None

    board2_history.append(label)

    if len(board2_history) < BOARD2_STABLE_FRAMES:
        rospy.loginfo("识别板二稳定计数: %d/%d",
                      len(board2_history), BOARD2_STABLE_FRAMES)
        return None

    if len(set(board2_history)) != 1:
        rospy.logwarn("识别板二连续结果不一致，继续等待稳定: %s",
                      list(board2_history))
        return None

    now = time.time()
    if now - last_board2_publish_time < BOARD2_PUBLISH_INTERVAL:
        return None

    msg = board2_label_to_msg(label)
    if msg is None:
        board2_history.clear()
        return None

    pub_board2.publish(msg)
    board2_published = True

    last_board2_publish_time = now
    last_board2_publish_label = label

    rospy.logwarn("识别板二连续 %d 次稳定，正式发布: %s, score=%.3f, publish=%s",
                  BOARD2_STABLE_FRAMES, label, score, msg.data)

    return msg.data



def contour_center(contour):
    moment = cv2.moments(contour)
    if moment['m00'] == 0:
        return None
    return (float(moment['m10']) / moment['m00'], float(moment['m01']) / moment['m00'])


def order_quad_points(points):
    """将四边形点排序为 tl, tr, br, bl。"""
    pts = np.array(points, dtype=np.float32).reshape(4, 2)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(4)

    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(s)]
    ordered[2] = pts[np.argmax(s)]
    ordered[1] = pts[np.argmin(diff)]
    ordered[3] = pts[np.argmax(diff)]
    return ordered


def window_grid_order_score(items):
    """评估4个候选窗口是否像一个2x2网格，分数越小越好。"""
    if len(items) != 4:
        return 1e9

    ordered = sorted(items, key=lambda item: item["center"][1])
    top = sorted(ordered[:2], key=lambda item: item["center"][0])
    bottom = sorted(ordered[2:], key=lambda item: item["center"][0])
    grid = top + bottom

    tl, tr, bl, br = [np.array(item["center"], dtype=np.float32) for item in grid]
    top_vec = tr - tl
    bottom_vec = br - bl
    left_vec = bl - tl
    right_vec = br - tr

    row_parallel = np.linalg.norm(top_vec - bottom_vec)
    col_parallel = np.linalg.norm(left_vec - right_vec)
    row_y_gap = abs(tl[1] - tr[1]) + abs(bl[1] - br[1])
    col_x_gap = abs(tl[0] - bl[0]) + abs(tr[0] - br[0])

    areas = [item["area"] for item in grid]
    area_mean = max(sum(areas) / float(len(areas)), 1.0)
    area_var = sum([abs(a - area_mean) for a in areas]) / area_mean

    return row_parallel + col_parallel + 0.6 * row_y_gap + 0.6 * col_x_gap + 20.0 * area_var


def order_window_items(items):
    """按左上、右上、左下、右下排序窗口。"""
    ordered = sorted(items, key=lambda item: item["center"][1])
    top = sorted(ordered[:2], key=lambda item: item["center"][0])
    bottom = sorted(ordered[2:], key=lambda item: item["center"][0])
    return top + bottom


def dedup_window_candidates(candidates):
    """按中心点距离去重，保留面积较大的候选。"""
    candidates = sorted(candidates, key=lambda item: item["area"], reverse=True)
    unique = []
    for item in candidates:
        duplicate = False
        for kept in unique:
            if distance(item["center"], kept["center"]) < BOARD1_WINDOW_DEDUP_DISTANCE:
                duplicate = True
                break
        if not duplicate:
            unique.append(item)
    return unique


def complete_four_windows_from_three(items, frame_shape):
    """检测到3个窗口时，用平行四边形关系补第4个窗口。"""
    if len(items) != 3 or not BOARD1_WINDOW_COMPLETE_FROM_THREE:
        return None

    h, w = frame_shape[:2]
    best = None
    best_score = 1e9

    centers = [np.array(item["center"], dtype=np.float32) for item in items]
    avg_area = sum([item["area"] for item in items]) / float(len(items))
    avg_side = int(max(20.0, math.sqrt(max(avg_area, 1.0))))

    for anchor_idx in range(3):
        p_anchor = centers[anchor_idx]
        others = [centers[i] for i in range(3) if i != anchor_idx]
        inferred_center = others[0] + others[1] - p_anchor

        cx, cy = float(inferred_center[0]), float(inferred_center[1])
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            continue

        half = avg_side / 2.0
        box = np.array([
            [cx - half, cy - half],
            [cx + half, cy - half],
            [cx + half, cy + half],
            [cx - half, cy + half]
        ], dtype=np.float32)

        inferred = {
            "center": (cx, cy),
            "box": box,
            "area": avg_area,
            "inferred": True,
        }
        four = items + [inferred]
        score = window_grid_order_score(four)
        if score < best_score:
            best_score = score
            best = four

    if best is None:
        return None

    ordered = order_window_items(best)
    centers_log = []
    for item in ordered:
        mark = "*" if item.get("inferred", False) else ""
        centers_log.append("(%.1f,%.1f)%s" % (item["center"][0], item["center"][1], mark))
    rospy.loginfo("识别板一窗口框路径使用3点补全第4窗口: centers=%s", centers_log)
    return ordered


def find_board1_window_boxes(frame):
    """查找识别板一的4个大窗口框。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    candidates = []
    binaries = []

    try:
        edges = cv2.Canny(blur, 40, 140, apertureSize=3)
        binaries.append(edges)
    except Exception:
        pass

    try:
        _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binaries.append(otsu)
    except Exception:
        pass

    for binary in binaries:
        contours = find_contours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = abs(cv2.contourArea(contour))
            if area < BOARD1_WINDOW_MIN_AREA or area > BOARD1_WINDOW_MAX_AREA:
                continue

            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
            if len(approx) != 4:
                continue

            rect = cv2.minAreaRect(approx)
            (cx, cy), (rw, rh), angle = rect
            if rw <= 1 or rh <= 1:
                continue
            if abs(rw - rh) / (rw + rh) > BOARD1_WINDOW_SQUARE_WH_RATE:
                continue

            box = cv2.boxPoints(rect)
            candidates.append({
                "center": (float(cx), float(cy)),
                "box": np.array(box, dtype=np.float32),
                "area": float(area),
                "inferred": False,
            })

    unique = dedup_window_candidates(candidates)

    selected = None
    if len(unique) >= 4:
        # 候选多时，从面积靠前的若干个中挑最像2x2网格的四个。
        pool = sorted(unique, key=lambda item: item["area"], reverse=True)[:12]
        try:
            import itertools
            best_score = 1e9
            for combo in itertools.combinations(pool, 4):
                combo = list(combo)
                score = window_grid_order_score(combo)
                if score < best_score:
                    best_score = score
                    selected = combo
        except Exception:
            selected = pool[:4]

        selected = order_window_items(selected)

    elif len(unique) == 3:
        selected = complete_four_windows_from_three(unique, frame.shape)

    if selected is None or len(selected) != 4:
        rospy.loginfo_throttle(
            1.0,
            "识别板一窗口框路径未找全: candidates=%d unique=%d need=4",
            len(candidates),
            len(unique)
        )
        return None

    centers_log = ["(%.1f,%.1f)" % (item["center"][0], item["center"][1]) for item in selected]
    rospy.loginfo_throttle(1.0, "识别板一窗口框路径找到4个窗口: centers=%s", centers_log)
    return selected


def warp_window_from_item(frame, item):
    """把单个窗口透视拉正成正方形。"""
    size = int(BOARD1_WINDOW_WARP_SIZE)
    if size <= 0:
        size = 240

    src = order_quad_points(item["box"])
    dst = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, matrix, (size, size))


def crop_window_inner(warped):
    """裁掉窗口黑边，保留内部二维码区域。"""
    h, w = warped.shape[:2]
    margin = int(min(h, w) * BOARD1_WINDOW_INNER_MARGIN_RATE)
    if margin <= 0:
        return warped
    if margin * 2 >= min(h, w) - 5:
        return warped
    return warped[margin:h - margin, margin:w - margin]


def decode_board1_by_windows(frame):
    """主路径：先找4个大窗口，再逐窗口轻量解码。二维码数量允许为0~4。"""
    if not BOARD1_WINDOW_DETECT_ENABLE:
        return None

    windows = find_board1_window_boxes(frame)
    if windows is None:
        return None

    all_text = []
    for item in windows:
        try:
            warped = warp_window_from_item(frame, item)
            inner = crop_window_inner(warped)
            all_text.append(decode_qr_text_light(inner))
        except Exception as e:
            rospy.logwarn_throttle(1.0, "识别板一窗口解码异常: %s", str(e))
            all_text.append("")

    rospy.loginfo("识别板一窗口框路径得到 all_text=%s", all_text)
    return all_text


def unique_qr_results(results):
    unique = []
    for text, cx, cy in results:
        duplicate = False
        for _, ux, uy in unique:
            if abs(cx - ux) < BOARD1_DIRECT_DEDUP_DISTANCE and abs(cy - uy) < BOARD1_DIRECT_DEDUP_DISTANCE:
                duplicate = True
                break
        if not duplicate:
            unique.append((text, cx, cy))
    return unique


def decode_board1_direct_four_qr(frame):
    """兜底路径：只有直接扫到4个二维码时，才按几何关系排序。少于4个不猜窗口。"""
    if not BOARD1_DIRECT_DECODE_ENABLE:
        return None

    results = []
    candidates = [frame]
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        candidates.append(gray)
        _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(otsu)
    except Exception:
        pass

    for cand in candidates:
        try:
            decoded = decode(cand)
        except Exception:
            continue
        for qr in decoded:
            try:
                text = qr.data.decode("utf-8")
            except Exception:
                text = qr.data
            text = safe_to_str(text)
            if text not in BOARD1_VALID_TEXT:
                continue
            try:
                if hasattr(qr, "polygon") and qr.polygon:
                    xs = [p.x for p in qr.polygon]
                    ys = [p.y for p in qr.polygon]
                    cx = sum(xs) / float(len(xs))
                    cy = sum(ys) / float(len(ys))
                else:
                    cx = qr.rect.left + qr.rect.width / 2.0
                    cy = qr.rect.top + qr.rect.height / 2.0
            except Exception:
                continue
            results.append((text, cx, cy))

    unique = unique_qr_results(results)
    if len(unique) < BOARD1_DIRECT_MIN_QR_COUNT:
        rospy.loginfo_throttle(
            1.0,
            "识别板一直接二维码路径数量不足: unique_qr=%d/%d",
            len(unique),
            BOARD1_DIRECT_MIN_QR_COUNT
        )
        return None

    unique = sorted(unique, key=lambda x: x[2])[:4]
    top = sorted(unique[:2], key=lambda x: x[1])
    bottom = sorted(unique[2:4], key=lambda x: x[1])
    ordered = top + bottom
    all_text = [item[0] for item in ordered]
    rospy.loginfo("识别板一直接二维码路径得到 all_text=%s", all_text)
    return all_text



def detect_and_publish_board1(frame):
    """执行识别板一识别。顺序：v2窗口框主路径 -> 4二维码直接兜底 -> 旧透视兜底。"""
    # 1. v2主路径：识别4个大窗口框，二维码数量允许为0~4。
    window_all_text = decode_board1_by_windows(frame)
    if window_all_text is not None:
        result = stable_publish_board1(window_all_text)
        if result is not None:
            rospy.logwarn("识别板一通过窗口框路径发布成功")
            return result
        # 如果窗口路径已经读到了有效二维码但还未稳定，本轮不再混入其它路径，避免不同路径裁剪造成 all_text 抖动。
        if select_board1_from_all_text(window_all_text) is not None:
            return None

    # 2. 兜底：只有直接扫到4个二维码时才使用，少于4个不猜窗口。
    direct_all_text = decode_board1_direct_four_qr(frame)
    if direct_all_text is not None:
        result = stable_publish_board1(direct_all_text)
        if result is not None:
            rospy.logwarn("识别板一通过直接二维码路径发布成功")
            return result
        if select_board1_from_all_text(direct_all_text) is not None:
            return None

    # 3. 旧透视兜底：保留原二维码定位框逻辑，但失败时不清空历史。
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 50, 150, apertureSize=3)

    contours = find_contours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_by_area = filter_contours_by_area(contours)
    quadrangles = filter_quadrangles(contours_by_area)
    squares = filter_square_like_contours(quadrangles)
    dingweikuang, jieguo = find_locator_squares(squares)

    if (BOARD1_DINGWEIKUANG_MIN <= len(dingweikuang) <= BOARD1_DINGWEIKUANG_MAX and
            len(jieguo) >= BOARD1_JIEGUO_MIN):
        try:
            rospy.loginfo_throttle(
                1.0,
                "识别板一旧透视兜底定位数量满足范围: dingweikuang=%d jieguo=%d",
                len(dingweikuang),
                len(jieguo)
            )

            warped_img = get_board1_perspective(frame, jieguo)
            all_data = decode_four_qr_areas(warped_img)
            return stable_publish_board1(all_data)

        except Exception as e:
            rospy.logwarn("识别板一旧透视兜底处理异常，不清空历史: %s", str(e))
            return None

    rospy.loginfo_throttle(
        1.0,
        "识别板一定位框数量不满足要求，不清空历史: dingweikuang=%d jieguo=%d",
        len(dingweikuang),
        len(jieguo)
    )

    return None


# =========================
# 主循环
# =========================

def init_ros_node_and_publishers():
    """初始化 ROS 节点和发布器。"""
    global pub_flag
    global pub_board2
    global pub_board1_all_text

    rospy.init_node('detect_abc', anonymous=True)

    # 识别板一：[是否去C，是否去A，是否去B，样品数，windows_1234，错误样品窗口]
    pub_flag = rospy.Publisher('/cam_return', Int32MultiArray, queue_size=10)

    # 识别板二：[0：空闲 / 1：忙碌，等待秒数]
    pub_board2 = rospy.Publisher('/board2_return', Int32MultiArray, queue_size=10)

    pub_board1_all_text = rospy.Publisher(
        BOARD1_CFG["share_all_text_topic"],
        String,
        queue_size=10,
        latch=True
    )

    # 订阅导航状态，供图像处理使用
    rospy.Subscriber('/nav_state', Int32, nav_state_cb)

    # 订阅 ROS 图像；识别只在主循环中取最新帧，不在回调中做重活。
    init_ros_image_subscriber()


def main():
    """程序入口。"""
    global board2_templates

    init_ros_node_and_publishers()
    rate = rospy.Rate(LIMIT_RATE_HZ)
    board2_templates = load_board2_templates()

    cap = None
    if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"] or CAMERA_FALLBACK_TO_HTTP:
        cap = open_http_capture()
 ##########################################################################################
 #   rospy.sleep(IDLE_GRAB_SLEEP_SEC)   其中0.1可能偏高，表示在非识别状态也以10HZ去grab视频流，可能过高，但是也有好处，可以一直丢弃掉帧，保持视频流最新，
 #   避免在长时间非识别状态后突然进入识别状态时，处理到过旧的帧导致识别失败。可以根据实际情况调整这个值，
 #   如果发现CPU占用过高或者识别不及时，可以适当降低这个频率，比如改为0.2或者0.5，甚至更高。需要在实际环境中测试和调整，以找到最佳的平衡点。
 ##########################################################################################
    while not rospy.is_shutdown():
        # 只有在导航状态为识别板一或识别板二时才进行图像处理，其他时间休眠等待，避免不必要的计算和卡顿
        if current_nav_state not in [STATE_BOARD1_RECOGNIZING, STATE_BOARD2_RECOGNIZING]:
            if cap is not None and cap.isOpened():
                cap.grab()
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue
        # 进一步优化：如果已经发布过识别结果，且导航状态未改变，继续休眠等待，避免重复识别和发布，减少摄像头读取和图像处理的频率，防止 CPU 占用过高和系统卡顿
        if current_nav_state == STATE_BOARD1_RECOGNIZING and board1_published:
            if cap is not None and cap.isOpened():
                cap.grab()
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue
        # 进一步优化：如果已经发布过识别结果，且导航状态未改变，继续休眠等待，避免重复识别和发布，减少摄像头读取和图像处理的频率，防止 CPU 占用过高和系统卡顿
        if current_nav_state == STATE_BOARD2_RECOGNIZING and board2_published:
            if cap is not None and cap.isOpened():
                cap.grab()
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue


        hx, frame, frame_source = read_frame_from_configured_source(cap)

        if (not hx) or (frame is None):
            rospy.logwarn("摄像头读取失败，尝试重连或等待ROS图像...")
            if cap is not None:
                cap.release()
            rospy.sleep(CAMERA_RECONNECT_SLEEP_SEC)
            if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"] or CAMERA_FALLBACK_TO_HTTP:
                cap = open_http_capture()
            continue

        rospy.loginfo_throttle(2.0, "摄像头读取成功 source=%s", frame_source)
        # 识别版一和识别板二的处理逻辑分开，避免不必要的计算和卡顿

        if current_nav_state == STATE_BOARD1_RECOGNIZING:
            rospy.loginfo_throttle(1.0, "到达识别板一，开始识别")
            frame = rotate_frame(frame, FRAME_ROTATE_ANGLE)
            detect_and_publish_board1(frame)

        elif current_nav_state == STATE_BOARD2_RECOGNIZING:
            rospy.loginfo_throttle(1.0, "到达识别板二，开始识别")
            frame = rotate_frame(frame, FRAME_ROTATE_ANGLE)
            if BOARD2_FORCE_LABEL_FOR_DEBUG:
                rospy.logwarn_throttle(1.0, "识别板二使用强制调试结果: %s", BOARD2_FORCE_LABEL_FOR_DEBUG)
                board2_label = BOARD2_FORCE_LABEL_FOR_DEBUG
                board2_score = BOARD2_FORCE_SCORE_FOR_DEBUG
            else:
                board2_label, board2_score = detect_board2_status(frame)

            board2_publish_data = stable_publish_board2(board2_label, board2_score)

        else:
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue

        rate.sleep() # 限制主循环频率，避免过高导致 CPU 占用过大

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
