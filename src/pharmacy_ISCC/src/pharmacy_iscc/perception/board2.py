# -*- coding: utf-8 -*-
import os
import cv2
import rospy
from std_msgs.msg import Int32MultiArray
from pharmacy_iscc.perception.settings import (
    BOARD2_MATCH_THRESHOLD,
    BOARD2_ROI,
    BOARD2_SCORE_MARGIN,
    BOARD2_TEMPLATE_DIR,
    BOARD2_TEMPLATE_SCALES,
)


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


def detect_board2_status(frame, board2_templates):
    """检测识别板二状态，返回 label 和 score。"""

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
