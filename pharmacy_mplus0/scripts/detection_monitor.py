#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""调试监视器 ROS 节点。

终端仪表盘：订阅所有关键话题，清屏刷新显示当前状态。
用于比赛现场快速定位"卡在哪个环节"。

显示内容：
  - 识别板一：/cam_return 旧格式、/board1_detections 新格式。
  - 识别板二：/cv1_result、/board2_status。
  - 任务状态：/current_task、/cv2_result、/current_qr_task。
  - 播报请求：/announce_request（最近一条）。

用法：
  rosrun pharmacy_mplus0 detection_monitor.py
"""

from __future__ import print_function

import os
import sys
import json
import time
import threading

import rospy
from std_msgs.msg import Int32MultiArray, String

from pharmacy_mplus0.constants import (
    TOPIC_CAM_RETURN,
    TOPIC_ALL_QRCODES,
    TOPIC_BOARD1_DETECTIONS,
    TOPIC_CV1_RESULT,
    TOPIC_BOARD2_STATUS,
    TOPIC_CV2_RESULT,
    TOPIC_CURRENT_TASK,
    TOPIC_CURRENT_QR_TASK,
    TOPIC_ANNOUNCE_REQUEST,
)


class DetectionMonitor(object):
    """收集所有话题的最新值并在终端刷新显示。"""

    def __init__(self):
        rospy.init_node("detection_monitor", anonymous=True)

        # ---- 数据缓存（线程安全）--------------------------------------
        self._lock = threading.Lock()
        self._cam_return = None
        self._all_qrcodes = None
        self._board1_detections = None
        self._cv1_result = None
        self._board2_status = None
        self._cv2_result = None
        self._current_task = None
        self._current_qr_task = None
        self._announce_request = None

        # 计时：每个话题的最近更新时间。
        self._times = {}

        # ---- 订阅所有关键话题 ----------------------------------------
        rospy.Subscriber(
            TOPIC_CAM_RETURN, Int32MultiArray,
            self._cb_cam_return, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_ALL_QRCODES, String,
            self._cb_all_qrcodes, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_BOARD1_DETECTIONS, String,
            self._cb_board1_detections, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_CV1_RESULT, String,
            self._cb_cv1_result, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_BOARD2_STATUS, String,
            self._cb_board2_status, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_CV2_RESULT, String,
            self._cb_cv2_result, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_CURRENT_TASK, String,
            self._cb_current_task, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_CURRENT_QR_TASK, String,
            self._cb_current_qr_task, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_ANNOUNCE_REQUEST, String,
            self._cb_announce_request, queue_size=5,
        )

        rospy.loginfo("[Monitor] 监视器初始化完成，已订阅 9 个话题")

    # ---- 回调 -------------------------------------------------------

    def _cb_cam_return(self, msg):
        with self._lock:
            self._cam_return = list(msg.data)
            self._times["cam_return"] = time.time()

    def _cb_all_qrcodes(self, msg):
        with self._lock:
            self._all_qrcodes = msg.data
            self._times["all_qrcodes"] = time.time()

    def _cb_board1_detections(self, msg):
        with self._lock:
            self._board1_detections = msg.data
            self._times["board1_detections"] = time.time()

    def _cb_cv1_result(self, msg):
        with self._lock:
            self._cv1_result = msg.data
            self._times["cv1_result"] = time.time()

    def _cb_board2_status(self, msg):
        with self._lock:
            self._board2_status = msg.data
            self._times["board2_status"] = time.time()

    def _cb_cv2_result(self, msg):
        with self._lock:
            self._cv2_result = msg.data
            self._times["cv2_result"] = time.time()

    def _cb_current_task(self, msg):
        with self._lock:
            self._current_task = msg.data
            self._times["current_task"] = time.time()

    def _cb_current_qr_task(self, msg):
        with self._lock:
            self._current_qr_task = msg.data
            self._times["current_qr_task"] = time.time()

    def _cb_announce_request(self, msg):
        with self._lock:
            self._announce_request = msg.data
            self._times["announce_request"] = time.time()

    # ---- 主循环 -----------------------------------------------------

    def run(self):
        """每秒清屏刷新一次状态。"""
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self._render()
            rate.sleep()

    def _render(self):
        """清屏并打印当前所有话题状态。"""
        # 清屏（跨平台）。
        if os.name == "nt":
            os.system("cls")
        else:
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.flush()

        with self._lock:
            cam_return = self._cam_return
            all_qrcodes = self._all_qrcodes
            board1_detections = self._board1_detections
            cv1_result = self._cv1_result
            board2_status = self._board2_status
            cv2_result = self._cv2_result
            current_task = self._current_task
            current_qr_task = self._current_qr_task
            announce_request = self._announce_request
            times = dict(self._times)

        now = time.time()

        def _age(key):
            """计算某话题距上次更新的秒数。"""
            t = times.get(key)
            if t is None:
                return "--"
            return "%.1fs" % (now - t)

        print("=" * 60)
        print("  detection_monitor — 智慧药房状态仪表盘")
        print("=" * 60)
        print()

        # 识别板一。
        print("── 识别板一 ──")
        if cam_return is not None:
            print("  /cam_return          [%s]  %s" % (
                _age("cam_return"), cam_return))
        else:
            print("  /cam_return          无数据")
        if board1_detections is not None:
            print("  /board1_detections   [%s]  %s" % (
                _age("board1_detections"),
                self._truncate(board1_detections, 80),
            ))
        else:
            print("  /board1_detections   无数据")
        if all_qrcodes is not None:
            print("  /all_qrcodes         [%s]  %s" % (
                _age("all_qrcodes"),
                self._truncate(all_qrcodes, 80),
            ))
        else:
            print("  /all_qrcodes         无数据")
        print()

        # 识别板二。
        print("── 识别板二 ──")
        print("  /cv1_result          [%s]  %s" % (
            _age("cv1_result"), cv1_result or "无数据"))
        if board2_status is not None:
            print("  /board2_status       [%s]  %s" % (
                _age("board2_status"),
                self._truncate(board2_status, 80),
            ))
        else:
            print("  /board2_status       无数据")
        print()

        # 任务状态。
        print("── 任务状态 ──")
        print("  /current_task        [%s]  %s" % (
            _age("current_task"), current_task or "无数据"))
        print("  /cv2_result          [%s]  %s" % (
            _age("cv2_result"), cv2_result or "无数据"))
        print("  /current_qr_task     [%s]  %s" % (
            _age("current_qr_task"), current_qr_task or "无数据"))
        print()

        # 播报。
        print("── 最近播报 ──")
        print("  /announce_request    [%s]  %s" % (
            _age("announce_request"),
            announce_request or "(无)",
        ))
        print()
        print("=" * 60)
        print("  按 Ctrl-C 退出")

    @staticmethod
    def _truncate(text, max_len):
        """截断过长字符串，避免终端换行混乱。"""
        if text is None:
            return "无数据"
        text = str(text)
        if len(text) <= max_len:
            return text
        return text[:max_len - 3] + "..."


# ---- 入口 ------------------------------------------------------------

def main():
    try:
        DetectionMonitor().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
