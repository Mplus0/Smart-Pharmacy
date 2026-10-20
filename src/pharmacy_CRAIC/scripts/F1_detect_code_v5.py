#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
F1_detect_code.py

视觉识别节点：
1. 识别板一：定位二维码板，读取四个区域二维码，选择最优方案并发布 /cam_return；
2. 识别板二：按需加载 YOLO ONNX 模型，识别空闲/忙碌和等待时间，发布 /board2_return。

备注：
    针对识别板二进行优化：采用纯 onnxruntime 模型进行目标检测，为降低 CPU 占用并防 OOM，模型仅在到达识别板二时（按需/懒加载）才载入内存。
    符合完全自主控制赛规，彻底移除人为键盘TCP干扰。
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

# ===== 引入 ONNX Runtime 库，符合低算力硬件限制 =====
try:
    import onnxruntime as ort
except ImportError:
    rospy.logwarn("未找到 onnxruntime 库，请运行 'pip install onnxruntime' 以支持板二模型推理。")

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

IMAGE_SOURCE = DETECT_CFG.get("image_source", "ros_topic")
CAMERA_TOPIC = DETECT_CFG.get("camera_topic", "/camera/rgb/image_raw")
CAMERA_FALLBACK_TO_HTTP = DETECT_CFG.get("camera_fallback_to_http", True)
CAMERA_MAX_AGE_SEC = DETECT_CFG.get("camera_max_age_sec", 0.5)
ROS_IMAGE_BUFF_SIZE = DETECT_CFG.get("ros_image_buff_size", 2 ** 24)

# =========================
# 板二 ONNX 懒加载配置
# =========================
BOARD2_MATCH_THRESHOLD = BOARD2_CFG["match_threshold"]
BOARD2_STABLE_FRAMES = BOARD2_CFG["stable_frames"]
BOARD2_SCORE_MARGIN = BOARD2_CFG["score_margin"]
BOARD2_PUBLISH_INTERVAL = BOARD2_CFG["publish_interval"]
BOARD2_ROI = BOARD2_CFG["roi"]
BOARD2_FORCE_LABEL_FOR_DEBUG = BOARD2_CFG.get("force_label_for_debug")
BOARD2_FORCE_SCORE_FOR_DEBUG = BOARD2_CFG.get("force_score_for_debug", 1.0)

# ===== YOLO ONNX 模型文件路径 =====
ONNX_MODEL_PATH = "/home/EPRobot/robot_ws/src/pharmacy_pkg/models/number_best.onnx"

# 类别映射：须对应你训练 number_best.onnx 时的 labels 顺序
YOLO_CLASSES = {
    0: "free",
    1: "busy_5",
    2: "busy_6",
    3: "busy_7",
    4: "busy_8",
    5: "busy_9",
    6: "busy_10"
}

# =========================
# 全局运行变量
# =========================

pub_flag = None
pub_board2 = None
pub_board1_all_text = None

board2_ort_session = None  # ONNX Runtime 全局会话

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
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def find_contours(binary_img, mode, method):
    result = cv2.findContours(binary_img, mode, method)
    if len(result) == 3:
        _, contours, _ = result
    else:
        contours, _ = result
    return contours

def rotate_frame(frame, angle):
    if angle == 0:
        return frame
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    return cv2.warpAffine(frame, matrix, (width, height))


# =========================
# 识别板二：ONNX 目标检测
# =========================

def crop_board2_roi(frame):
    h, w = frame.shape[:2]
    x1_rate, y1_rate, x2_rate, y2_rate = BOARD2_ROI
    x1 = int(w * x1_rate)
    y1 = int(h * y1_rate)
    x2 = int(w * x2_rate)
    y2 = int(h * y2_rate)
    return frame[y1:y2, x1:x2]


def detect_board2_status_onnx(frame):
    """
    使用 onnxruntime 懒加载 ONNX 模型进行轻量化推理。
    返回预测置信度最高的 label 和 score。
    """
    global board2_ort_session

    # 1. 懒加载：按需载入 ONNX 模型 (仅使用 CPU，占用极低)
    if board2_ort_session is None:
        rospy.loginfo("正在加载轻量化 ONNX 模型: %s ...", ONNX_MODEL_PATH)
        try:
            board2_ort_session = ort.InferenceSession(ONNX_MODEL_PATH, providers=['CPUExecutionProvider'])
            rospy.loginfo("板二 ONNX 模型加载成功！")
        except Exception as e:
            rospy.logerr("ONNX 模型加载失败！错误: %s", str(e))
            return None, 0.0

    # 2. 图像预处理 (根据 YOLO 标准通常是 640x640)
    roi_img = crop_board2_roi(frame)
    if roi_img is None or roi_img.size == 0:
        return None, 0.0

    input_size = 640
    img_resized = cv2.resize(roi_img, (input_size, input_size))
    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
    img_tensor = np.transpose(img_rgb, (2, 0, 1)).astype(np.float32) / 255.0
    img_tensor = np.expand_dims(img_tensor, axis=0)

    # 3. 运行推理
    try:
        input_name = board2_ort_session.get_inputs()[0].name
        outputs = board2_ort_session.run(None, {input_name: img_tensor})
        predictions = outputs[0]
    except Exception as e:
        rospy.logerr_throttle(1.0, "ONNX 推理异常: %s", str(e))
        return None, 0.0

    # 4. YOLOv8 ONNX 简易后处理 (解析 1 x [4+classes] x anchors)
    best_label = None
    best_score = -1.0

    preds = np.squeeze(predictions)
    if len(preds.shape) == 2:
        preds = preds.T # 转换 shape 为 (anchors, 4 + classes)

        # 使用 numpy 加速寻找最高置信度的类别和边界框
        class_scores = preds[:, 4:]
        max_scores = np.max(class_scores, axis=1)
        best_box_idx = np.argmax(max_scores)
        best_score = max_scores[best_box_idx]

        if best_score > 0:
            class_id = np.argmax(class_scores[best_box_idx])
            best_label = YOLO_CLASSES.get(class_id, None)

    if best_label is not None and best_score >= BOARD2_MATCH_THRESHOLD:
        return best_label, float(best_score)

    rospy.loginfo_throttle(1.0, "ONNX 板二检测置信度不足: max_score=%.3f", best_score)
    return None, float(best_score)


def board2_label_to_msg(label):
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


# =========================
# 识别板一：二维码识别 (完整恢复)
# =========================

def filter_contours_by_area(contours):
    filtered = []
    for contour in contours:
        moment = cv2.moments(contour)
        if moment['m00'] < min_area or moment['m00'] > max_area:
            continue
        filtered.append(contour)
    return filtered

def filter_quadrangles(contours):
    quadrangles = []
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, approxPolyDP_epslion * peri, True)
        if len(approx) == 4:
            quadrangles.append(approx)
    return quadrangles

def filter_square_like_contours(quadrangles):
    squares = []
    for hull in quadrangles:
        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect
        if (w + h) > 0 and abs(w - h) / (w + h) < wh_rate:
            squares.append(hull)
    return squares

def find_locator_squares(squares):
    dingweikuang = []
    jieguo = []
    for i in range(len(squares) - 1):
        M1 = cv2.moments(squares[i])
        if M1['m00'] == 0: continue
        cx1 = int(M1['m10'] / M1['m00'])
        cy1 = int(M1['m01'] / M1['m00'])
        for j in range(i + 1, len(squares)):
            M2 = cv2.moments(squares[j])
            if M2['m00'] == 0: continue
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
    except Exception: pass
    try:
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(binary_otsu)
    except Exception: pass
    try:
        h, w = gray.shape[:2]
        if h > 0 and w > 0:
            candidates.append(cv2.resize(gray, (w * 2, h * 2), interpolation=cv2.INTER_LINEAR))
    except Exception: pass
    for cand in candidates:
        try:
            code_result = decode(cand)
        except Exception: continue
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
    img_len = [0, 0, 0, 0]
    if len(all_data[0]) < 4: img_len[0] = len(all_data[0])
    if len(all_data[1]) < 4: img_len[1] = len(all_data[1])
    if len(all_data[2]) < 4: img_len[2] = len(all_data[2])
    if len(all_data[3]) < 4: img_len[3] = len(all_data[3])
    return img_len

def normalize_qr_data(item):
    return safe_to_str(item)

def detect_error_window(all_data):
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
    selection = select_board1_from_all_text(all_data)
    all_text = [normalize_qr_data(item) for item in all_data]
    if selection is None: return None
    msg = Int32MultiArray()
    msg.data = selection["selected_msg"]
    return msg

def publish_board1_all_text(selection):
    global board1_all_text_seq
    if not BOARD1_CFG.get("publish_all_text", True): return
    if pub_board1_all_text is None: return
    if selection is None: return
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

def stable_publish_board1(all_data):
    global pub_flag, board1_history, board1_published
    if board1_published: return None
    all_text = normalize_all_text(all_data)
    selection = select_board1_from_all_text(all_text)
    if selection is None: return None
    stable_key = tuple(all_text)
    board1_history.append(stable_key)
    if len(board1_history) < BOARD1_STABLE_FRAMES: return None
    if len(set(board1_history)) != 1: return None

    msg = Int32MultiArray()
    msg.data = selection["selected_msg"]
    pub_flag.publish(msg)
    board1_published = True
    publish_board1_all_text(selection)
    return msg.data

def contour_center(contour):
    moment = cv2.moments(contour)
    if moment['m00'] == 0: return None
    return (float(moment['m10']) / moment['m00'], float(moment['m01']) / moment['m00'])

def order_quad_points(points):
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
    if len(items) != 4: return 1e9
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
    ordered = sorted(items, key=lambda item: item["center"][1])
    top = sorted(ordered[:2], key=lambda item: item["center"][0])
    bottom = sorted(ordered[2:], key=lambda item: item["center"][0])
    return top + bottom

def dedup_window_candidates(candidates):
    candidates = sorted(candidates, key=lambda item: item["area"], reverse=True)
    unique = []
    for item in candidates:
        duplicate = False
        for kept in unique:
            if distance(item["center"], kept["center"]) < BOARD1_WINDOW_DEDUP_DISTANCE:
                duplicate = True
                break
        if not duplicate: unique.append(item)
    return unique

def complete_four_windows_from_three(items, frame_shape):
    if len(items) != 3 or not BOARD1_WINDOW_COMPLETE_FROM_THREE: return None
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
        if cx < 0 or cy < 0 or cx >= w or cy >= h: continue
        half = avg_side / 2.0
        box = np.array([[cx - half, cy - half], [cx + half, cy - half], [cx + half, cy + half], [cx - half, cy + half]], dtype=np.float32)
        inferred = {"center": (cx, cy), "box": box, "area": avg_area, "inferred": True}
        four = items + [inferred]
        score = window_grid_order_score(four)
        if score < best_score:
            best_score = score
            best = four
    if best is None: return None
    ordered = order_window_items(best)
    return ordered

def find_board1_window_boxes(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    candidates = []
    binaries = []
    try:
        edges = cv2.Canny(blur, 40, 140, apertureSize=3)
        binaries.append(edges)
    except Exception: pass
    try:
        _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binaries.append(otsu)
    except Exception: pass
    for binary in binaries:
        contours = find_contours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = abs(cv2.contourArea(contour))
            if area < BOARD1_WINDOW_MIN_AREA or area > BOARD1_WINDOW_MAX_AREA: continue
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
            if len(approx) != 4: continue
            rect = cv2.minAreaRect(approx)
            (cx, cy), (rw, rh), angle = rect
            if rw <= 1 or rh <= 1: continue
            if abs(rw - rh) / (rw + rh) > BOARD1_WINDOW_SQUARE_WH_RATE: continue
            box = cv2.boxPoints(rect)
            candidates.append({"center": (float(cx), float(cy)), "box": np.array(box, dtype=np.float32), "area": float(area), "inferred": False})
    unique = dedup_window_candidates(candidates)
    selected = None
    if len(unique) >= 4:
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
    if selected is None or len(selected) != 4: return None
    return selected

def warp_window_from_item(frame, item):
    size = int(BOARD1_WINDOW_WARP_SIZE)
    if size <= 0: size = 240
    src = order_quad_points(item["box"])
    dst = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, matrix, (size, size))

def crop_window_inner(warped):
    h, w = warped.shape[:2]
    margin = int(min(h, w) * BOARD1_WINDOW_INNER_MARGIN_RATE)
    if margin <= 0: return warped
    if margin * 2 >= min(h, w) - 5: return warped
    return warped[margin:h - margin, margin:w - margin]

def decode_board1_by_windows(frame):
    if not BOARD1_WINDOW_DETECT_ENABLE: return None
    windows = find_board1_window_boxes(frame)
    if windows is None: return None
    all_text = []
    for item in windows:
        try:
            warped = warp_window_from_item(frame, item)
            inner = crop_window_inner(warped)
            all_text.append(decode_qr_text_light(inner))
        except Exception as e:
            all_text.append("")
    return all_text

def unique_qr_results(results):
    unique = []
    for text, cx, cy in results:
        duplicate = False
        for _, ux, uy in unique:
            if abs(cx - ux) < BOARD1_DIRECT_DEDUP_DISTANCE and abs(cy - uy) < BOARD1_DIRECT_DEDUP_DISTANCE:
                duplicate = True
                break
        if not duplicate: unique.append((text, cx, cy))
    return unique

def decode_board1_direct_four_qr(frame):
    if not BOARD1_DIRECT_DECODE_ENABLE: return None
    results = []
    candidates = [frame]
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        candidates.append(gray)
        _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(otsu)
    except Exception: pass
    for cand in candidates:
        try:
            decoded = decode(cand)
        except Exception: continue
        for qr in decoded:
            try: text = qr.data.decode("utf-8")
            except Exception: text = qr.data
            text = safe_to_str(text)
            if text not in BOARD1_VALID_TEXT: continue
            try:
                if hasattr(qr, "polygon") and qr.polygon:
                    xs = [p.x for p in qr.polygon]
                    ys = [p.y for p in qr.polygon]
                    cx = sum(xs) / float(len(xs))
                    cy = sum(ys) / float(len(ys))
                else:
                    cx = qr.rect.left + qr.rect.width / 2.0
                    cy = qr.rect.top + qr.rect.height / 2.0
            except Exception: continue
            results.append((text, cx, cy))
    unique = unique_qr_results(results)
    if len(unique) < BOARD1_DIRECT_MIN_QR_COUNT: return None
    unique = sorted(unique, key=lambda x: x[2])[:4]
    top = sorted(unique[:2], key=lambda x: x[1])
    bottom = sorted(unique[2:4], key=lambda x: x[1])
    ordered = top + bottom
    all_text = [item[0] for item in ordered]
    return all_text

def detect_and_publish_board1(frame):
    window_all_text = decode_board1_by_windows(frame)
    if window_all_text is not None:
        result = stable_publish_board1(window_all_text)
        if result is not None: return result
        if select_board1_from_all_text(window_all_text) is not None: return None

    direct_all_text = decode_board1_direct_four_qr(frame)
    if direct_all_text is not None:
        result = stable_publish_board1(direct_all_text)
        if result is not None: return result
        if select_board1_from_all_text(direct_all_text) is not None: return None

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 50, 150, apertureSize=3)
    contours = find_contours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_by_area = filter_contours_by_area(contours)
    quadrangles = filter_quadrangles(contours_by_area)
    squares = filter_square_like_contours(quadrangles)
    dingweikuang, jieguo = find_locator_squares(squares)

    if (BOARD1_DINGWEIKUANG_MIN <= len(dingweikuang) <= BOARD1_DINGWEIKUANG_MAX and len(jieguo) >= BOARD1_JIEGUO_MIN):
        try:
            warped_img = get_board1_perspective(frame, jieguo)
            all_data = decode_four_qr_areas(warped_img)
            return stable_publish_board1(all_data)
        except Exception as e: return None
    return None

# =========================
# 图像采集及基础通信回调
# =========================

def read_latest_frame(cap, drop_count=DROP_FRAME_COUNT):
    if cap is None: return False, None
    for _ in range(drop_count): cap.grab()
    return cap.read()

def ros_image_cb(msg):
    global latest_ros_frame, latest_ros_frame_stamp, latest_ros_frame_recv_time, ros_image_bridge
    if ros_image_bridge is None: return
    try: frame = ros_image_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception: return
    with latest_ros_frame_lock:
        latest_ros_frame = frame.copy()
        latest_ros_frame_recv_time = time.time()

def ros_compressed_image_cb(msg):
    global latest_ros_frame, latest_ros_frame_stamp, latest_ros_frame_recv_time
    try:
        data = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception: return
    if frame is None: return
    with latest_ros_frame_lock:
        latest_ros_frame = frame.copy()
        latest_ros_frame_recv_time = time.time()

def init_ros_image_subscriber():
    global ros_image_bridge, ros_image_sub
    if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"]: return
    if IMAGE_SOURCE == "ros_compressed" or CAMERA_TOPIC.endswith("/compressed"):
        ros_image_sub = rospy.Subscriber(CAMERA_TOPIC, CompressedImage, ros_compressed_image_cb, queue_size=1, buff_size=ROS_IMAGE_BUFF_SIZE)
    else:
        ros_image_bridge = cv_bridge.CvBridge()
        ros_image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, ros_image_cb, queue_size=1, buff_size=ROS_IMAGE_BUFF_SIZE)

def get_latest_ros_frame():
    with latest_ros_frame_lock:
        if latest_ros_frame is None: return False, None, "empty"
        age = time.time() - latest_ros_frame_recv_time if latest_ros_frame_recv_time else 0.0
        if CAMERA_MAX_AGE_SEC > 0 and age > CAMERA_MAX_AGE_SEC: return False, None, "stale %.3fs" % age
        return True, latest_ros_frame.copy(), "ok"

def open_http_capture():
    cap = cv2.VideoCapture(CAMERA_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap

def read_frame_from_configured_source(cap):
    if IMAGE_SOURCE in ["ros_topic", "ros_image", "ros_compressed"]:
        ok, frame, reason = get_latest_ros_frame()
        if ok: return True, frame, "ros"
        if not CAMERA_FALLBACK_TO_HTTP: return False, None, "ros:%s" % reason
    hx, frame = read_latest_frame(cap, drop_count=DROP_FRAME_COUNT)
    return hx, frame, "http"

def nav_state_cb(msg):
    global current_nav_state, previous_nav_state, board1_history, board2_history
    global board1_published, board2_published, last_board2_publish_time, last_board2_publish_label

    previous_nav_state = current_nav_state
    current_nav_state = msg.data

    if previous_nav_state != STATE_BOARD1_RECOGNIZING and current_nav_state == STATE_BOARD1_RECOGNIZING:
        board1_history.clear()
        board1_published = False

    if previous_nav_state != STATE_BOARD2_RECOGNIZING and current_nav_state == STATE_BOARD2_RECOGNIZING:
        board2_history.clear()
        board2_published = False
        last_board2_publish_time = 0.0
        last_board2_publish_label = None

def stable_publish_board2(label, score):
    global last_board2_publish_time, last_board2_publish_label, pub_board2, board2_published
    if board2_published: return None
    if label is None:
        board2_history.clear()
        return None

    board2_history.append(label)
    if len(board2_history) < BOARD2_STABLE_FRAMES: return None
    if len(set(board2_history)) != 1: return None

    now = time.time()
    if now - last_board2_publish_time < BOARD2_PUBLISH_INTERVAL: return None

    msg = board2_label_to_msg(label)
    if msg is None:
        board2_history.clear()
        return None

    pub_board2.publish(msg)
    board2_published = True
    last_board2_publish_time = now
    last_board2_publish_label = label
    rospy.logwarn("识别板二连续稳定，ONNX 发布: %s, score=%.3f", label, score)
    return msg.data


# =========================
# 主循环
# =========================

def init_ros_node_and_publishers():
    global pub_flag, pub_board2, pub_board1_all_text
    rospy.init_node('detect_abc', anonymous=True)
    pub_flag = rospy.Publisher('/cam_return', Int32MultiArray, queue_size=10)
    pub_board2 = rospy.Publisher('/board2_return', Int32MultiArray, queue_size=10)
    pub_board1_all_text = rospy.Publisher(BOARD1_CFG["share_all_text_topic"], String, queue_size=10, latch=True)
    rospy.Subscriber('/nav_state', Int32, nav_state_cb)
    init_ros_image_subscriber()

def main():
    init_ros_node_and_publishers()
    rate = rospy.Rate(LIMIT_RATE_HZ)

    rospy.loginfo("识别板二将采用 ONNX 极轻量化懒加载，符合比赛硬件要求。")

    cap = None
    if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"] or CAMERA_FALLBACK_TO_HTTP:
        cap = open_http_capture()

    while not rospy.is_shutdown():
        if current_nav_state not in [STATE_BOARD1_RECOGNIZING, STATE_BOARD2_RECOGNIZING]:
            if cap is not None and cap.isOpened(): cap.grab()
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue

        if current_nav_state == STATE_BOARD1_RECOGNIZING and board1_published:
            if cap is not None and cap.isOpened(): cap.grab()
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue

        if current_nav_state == STATE_BOARD2_RECOGNIZING and board2_published:
            if cap is not None and cap.isOpened(): cap.grab()
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue

        hx, frame, frame_source = read_frame_from_configured_source(cap)

        if (not hx) or (frame is None):
            if cap is not None: cap.release()
            rospy.sleep(CAMERA_RECONNECT_SLEEP_SEC)
            if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"] or CAMERA_FALLBACK_TO_HTTP:
                cap = open_http_capture()
            continue

        if current_nav_state == STATE_BOARD1_RECOGNIZING:
            frame = rotate_frame(frame, FRAME_ROTATE_ANGLE)
            # ===== 恢复板一视觉检测逻辑 =====
            detect_and_publish_board1(frame)

        elif current_nav_state == STATE_BOARD2_RECOGNIZING:
            frame = rotate_frame(frame, FRAME_ROTATE_ANGLE)
            if BOARD2_FORCE_LABEL_FOR_DEBUG:
                board2_label = BOARD2_FORCE_LABEL_FOR_DEBUG
                board2_score = BOARD2_FORCE_SCORE_FOR_DEBUG
            else:
                # ===== ONNX 推理 =====
                board2_label, board2_score = detect_board2_status_onnx(frame)

            stable_publish_board2(board2_label, board2_score)

        rate.sleep()

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()