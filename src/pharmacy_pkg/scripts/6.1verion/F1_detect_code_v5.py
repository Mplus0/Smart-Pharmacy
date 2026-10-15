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

fps = 0
detect_num = 0

CAMERA_URL = CAR_CONFIG["camera_url"]
FRAME_ROTATE_ANGLE = DETECT_CFG["frame_rotate_angle"]
DROP_FRAME_COUNT = DETECT_CFG["drop_frame_count"]
LIMIT_RATE_HZ = DETECT_CFG["limit_rate_hz"]
IDLE_GRAB_SLEEP_SEC = DETECT_CFG["idle_grab_sleep_sec"]
CAMERA_RECONNECT_SLEEP_SEC = DETECT_CFG["camera_reconnect_sleep_sec"]

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
    for _ in range(drop_count):
        cap.grab()
    return cap.read()

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


def decode_four_qr_areas(warped_img):
    """裁剪四个二维码区域并解码。"""
    aera1_crop = warped_img[65:255, 65:235]
    aera2_crop = warped_img[65:255, 365:535]
    aera3_crop = warped_img[335:525, 65:235]
    aera4_crop = warped_img[335:525, 365:535]

    crops = [aera1_crop, aera2_crop, aera3_crop, aera4_crop]
    decoded_data = []

    for crop in crops:
        gray_img = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        ret, binary_img = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY)
        code_result = decode(binary_img)

        data = ""
        for QR in code_result:
            try:
                data = QR.data.decode("utf-8")
            except Exception:
                data = QR.data

            data = safe_to_str(data)

        decoded_data.append(data)

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
    """
    global pub_flag
    global board1_history
    global board1_published

    if board1_published:
        return None

    all_text = normalize_all_text(all_data)
    rospy.logwarn("识别板一二维码结果: %s", all_text)

    stable_key = tuple(all_text)
    board1_history.append(stable_key)

    if len(board1_history) < BOARD1_STABLE_FRAMES:
        rospy.loginfo("识别板一 all_text 稳定计数: %d/%d",
                      len(board1_history), BOARD1_STABLE_FRAMES)
        return None

    if len(set(board1_history)) != 1:
        rospy.logwarn("识别板一 all_text 连续结果不一致，继续等待稳定: %s",
                      list(board1_history))
        return None

    selection = select_board1_from_all_text(all_text)
    if selection is None:
        rospy.logwarn("识别板一 all_text 已稳定但无有效二维码组合，不发布: %s", all_text)
        board1_history.clear()
        return None

    msg = Int32MultiArray()
    msg.data = selection["selected_msg"]

    pub_flag.publish(msg)
    board1_published = True
    publish_board1_all_text(selection)

    rospy.logwarn("识别板一 all_text 连续 %d 次稳定，正式发布: all_text=%s msg=%s",
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




def detect_and_publish_board1(frame):
    """执行识别板一定位、透视变换、二维码解码和稳定发布。"""
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
                "识别板一定位数量满足范围: dingweikuang=%d jieguo=%d",
                len(dingweikuang),
                len(jieguo)
            )

            warped_img = get_board1_perspective(frame, jieguo)
            all_data = decode_four_qr_areas(warped_img)
            return stable_publish_board1(all_data)

        except Exception as e:
            rospy.logwarn("识别板一处理异常: %s", str(e))
            board1_history.clear()
            return None

    rospy.loginfo_throttle(
        1.0,
        "识别板一定位框数量不满足要求: dingweikuang=%d jieguo=%d",
        len(dingweikuang),
        len(jieguo)
    )

    board1_history.clear()
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


def main():
    """程序入口。"""
    global board2_templates

    init_ros_node_and_publishers()
    rate = rospy.Rate(LIMIT_RATE_HZ)
    board2_templates = load_board2_templates()

    cap = cv2.VideoCapture(CAMERA_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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


        hx, frame = read_latest_frame(cap, drop_count=DROP_FRAME_COUNT )

        if (not hx) or (frame is None):
            rospy.logwarn("摄像头读取失败，尝试重连...")
            cap.release()
            rospy.sleep(CAMERA_RECONNECT_SLEEP_SEC)
            cap = cv2.VideoCapture(CAMERA_URL)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            continue

        rospy.loginfo_throttle(2.0, "摄像头读取成功")
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

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
