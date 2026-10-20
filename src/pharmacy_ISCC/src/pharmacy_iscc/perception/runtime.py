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
from pharmacy_iscc.navigation.audio import play_board1_error_audio
from pharmacy_iscc.communication.board2 import query_board2_tcp
import time
import json
from collections import deque

import cv2
import rospy
from std_msgs.msg import Int32MultiArray, Int32, String

from pharmacy_iscc.perception.selection import normalize_all_text, select_board1_from_all_text
from pharmacy_iscc.perception.selection import build_board1_task_queue
from pharmacy_iscc.recording.records import record_result

from pharmacy_iscc.perception.settings import (
    BOARD1_CFG,
    BOARD1_DINGWEIKUANG_MAX,
    BOARD1_DINGWEIKUANG_MIN,
    BOARD1_JIEGUO_MIN,
    BOARD1_REQUIRE_FOUR_WINDOWS,
    BOARD1_STABLE_FRAMES,
    BOARD1_VALID_TEXT,
    BOARD2_CFG,
    BOARD2_FORCE_LABEL_FOR_DEBUG,
    BOARD2_FORCE_SCORE_FOR_DEBUG,
    BOARD2_PUBLISH_INTERVAL,
    BOARD2_RESULT_SOURCE,
    BOARD2_STABLE_FRAMES,
    CAMERA_FALLBACK_TO_HTTP,
    CAMERA_RECONNECT_SLEEP_SEC,
    CAR_CONFIG,
    CAR_ID,
    FRAME_ROTATE_ANGLE,
    IDLE_GRAB_SLEEP_SEC,
    IMAGE_SOURCE,
    LIMIT_RATE_HZ,
    STATE_BOARD1_RECOGNIZING,
    STATE_BOARD2_RECOGNIZING,
    STATE_GO_TO_BOARD1,
)
from pharmacy_iscc.perception.image_utils import find_contours, rotate_frame
from pharmacy_iscc.perception.board1 import filter_contours_by_area, filter_quadrangles, filter_square_like_contours, find_locator_squares, get_board1_perspective, decode_four_qr_areas, normalize_qr_data, decode_board1_by_windows, decode_board1_direct_four_qr
from pharmacy_iscc.perception.board2 import load_board2_templates, detect_board2_status, board2_label_to_msg
from pharmacy_iscc.perception.image_input import init_ros_image_subscriber, open_http_capture, read_frame_from_configured_source

os.environ["LANG"] = "zh_CN.UTF-8"


# =========================
# 全局运行变量
# =========================

pub_flag = None
pub_board2 = None
pub_board1_all_text = None
board2_templates = {}
board1_history = deque(maxlen=BOARD1_STABLE_FRAMES)
board1_error_history = deque(maxlen=BOARD1_STABLE_FRAMES)
board1_error_announced = False
board2_history = deque(maxlen=BOARD2_STABLE_FRAMES)
board1_published = False
board2_published = False
board2_visit = 0
board1_all_text_seq = 0
last_board2_publish_time = 0.0
last_board2_publish_label = None
previous_nav_state = None
current_nav_state = STATE_GO_TO_BOARD1


# =========================
# 识别板二：模板匹配
# =========================


def publish_board2_tcp_result():
    """本次访问收到一个有效人工结果后发布一次，不重复进行图像稳定检测。"""
    global board2_published
    visit = board2_visit
    label = query_board2_tcp(
        lambda: rospy.is_shutdown() or current_nav_state != STATE_BOARD2_RECOGNIZING)
    if label is None:
        label = BOARD2_CFG.get("tcp_default_label", "free")
    if (label is None or rospy.is_shutdown() or board2_published
            or current_nav_state != STATE_BOARD2_RECOGNIZING or visit != board2_visit):
        return
    msg = board2_label_to_msg(label)
    pub_board2.publish(msg)
    board2_published = True


def nav_state_cb(msg):
    """更新当前导航状态，供图像处理使用。"""
    global current_nav_state
    global previous_nav_state
    global board1_history
    global board2_history
    global board1_published
    global board1_error_announced
    global board2_published
    global board2_visit
    global last_board2_publish_time
    global last_board2_publish_label

    previous_nav_state = current_nav_state
    current_nav_state = msg.data

    if previous_nav_state != STATE_BOARD1_RECOGNIZING and current_nav_state == STATE_BOARD1_RECOGNIZING:
        board1_history.clear()
        board1_error_history.clear()
        board1_error_announced = False
        board1_published = False
        rospy.loginfo("进入识别板一状态，清空历史缓存，允许发布一次识别结果")

    if previous_nav_state != STATE_BOARD2_RECOGNIZING and current_nav_state == STATE_BOARD2_RECOGNIZING:
        board2_visit += 1
        board2_history.clear()
        board2_published = False
        last_board2_publish_time = 0.0
        last_board2_publish_label = None
        rospy.loginfo("进入识别板二状态，清空历史缓存，允许发布一次识别结果")


# =========================
# 识别板一：二维码识别
# =========================


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


def announce_stable_board1_error(all_text):
    """One shared alert per board1 visit, even when no valid task is present."""
    global board1_error_announced
    if board1_error_announced:
        return
    if len(all_text) != 4 or not any(t and t not in BOARD1_VALID_TEXT for t in all_text):
        board1_error_history.clear()
        return
    board1_error_history.append(tuple(all_text))
    if len(board1_error_history) < BOARD1_STABLE_FRAMES or len(set(board1_error_history)) != 1:
        return
    board1_error_announced = True
    for text in all_text:
        if text and text not in BOARD1_VALID_TEXT:
            # 复用已归一化、连续稳定的异常内容；每次访问只经过此分支一次。
            if isinstance(text, bytes):
                text = text.decode('utf-8')
            record_result(CAR_ID, u'识别板一', u'干扰二维码', {u'干扰内容': text})
    rospy.logwarn("Board1 stable interference QR detected; skipping invalid QR task and playing error.wav")
    play_board1_error_audio()


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
    announce_stable_board1_error(all_text)
    if rospy.is_shutdown():
        return None
    rospy.logwarn("识别板一二维码结果: %s", all_text)

    selection = select_board1_from_all_text(all_text)
    if selection is None:
        board1_history.clear()
        rospy.logwarn("识别板一 all_text 无有效二维码组合，清空稳定历史: %s", all_text)
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
    # 首六项保持原选中任务格式，后续每六项为一个剩余合法任务。
    # 同一次消息完整传递整板，避免跨话题先后到达导致任务缓存不完整。
    tasks = build_board1_task_queue(all_text)
    msg.data = [value for task in tasks for value in task]

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
        # Four real windows were seen. Keep abnormal-only observations stable
        # for their alert, without falling through to the missing-border reset.
        if BOARD1_REQUIRE_FOUR_WINDOWS:
            return None

    # 2. 兜底：只有直接扫到4个二维码时才使用，少于4个不猜窗口。
    # Strict mode requires four observed window borders, including empty slots.
    if BOARD1_REQUIRE_FOUR_WINDOWS:
        board1_history.clear()
        board1_error_history.clear()
        return None

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
    if BOARD2_RESULT_SOURCE not in ('tcp', 'vision'):
        raise ValueError('board2.result_source must be tcp or vision')
    if BOARD2_RESULT_SOURCE == 'vision':
        board2_templates = load_board2_templates()

    cap = None
    if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"] or CAMERA_FALLBACK_TO_HTTP:
        cap = open_http_capture()
 ##########################################################################################
 #   rospy.sleep(IDLE_GRAB_SLEEP_SEC)   其中0.1可能偏高，表示在非识别状态也以10HZ去grab视频流，可能过高，但是也有好处，可以一直丢弃掉帧，保持视频流最新，
 #   避免在长时间非识别状态后突然进入识别状态时，处理到过旧的帧导致识别失败。可以根据实际情况调整这个值，
 #   如果发现CPU占用过高或者识别不及时，可以适当降低这个频率，比如改为0.2或者0.5，甚至更高。需要在实际环境中测试和调整，以找到最佳的平衡点。
 ##########################################################################################
    last_ros_frame_id = None
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


        # TCP 板二不依赖相机帧，也不使用强制 free 调试结果。
        if current_nav_state == STATE_BOARD2_RECOGNIZING and BOARD2_RESULT_SOURCE == 'tcp':
            publish_board2_tcp_result()
            rospy.sleep(float(BOARD2_CFG["tcp_retry_interval_sec"]))
            continue

        hx, frame, frame_info = read_frame_from_configured_source(cap, with_identity=True)
        frame_source, frame_id = frame_info

        if (not hx) or (frame is None):
            if current_nav_state == STATE_BOARD1_RECOGNIZING:
                board1_history.clear()
                board1_error_history.clear()
            rospy.logwarn("摄像头读取失败，尝试重连或等待ROS图像...")
            if cap is not None:
                cap.release()
            rospy.sleep(CAMERA_RECONNECT_SLEEP_SEC)
            if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"] or CAMERA_FALLBACK_TO_HTTP:
                cap = open_http_capture()
            continue

        if frame_source == "ros":
            # Count each received ROS frame once, even if the cache is read again.
            if frame_id == last_ros_frame_id:
                rospy.sleep(IDLE_GRAB_SLEEP_SEC)
                continue
            last_ros_frame_id = frame_id

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
                board2_label, board2_score = detect_board2_status(frame, board2_templates)

            board2_publish_data = stable_publish_board2(board2_label, board2_score)

        else:
            rospy.sleep(IDLE_GRAB_SLEEP_SEC)
            continue

        rate.sleep() # 限制主循环频率，避免过高导致 CPU 占用过大

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


