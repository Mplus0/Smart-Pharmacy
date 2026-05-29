#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""终端仪表盘（独立版本）。

订阅关键话题并清屏刷新显示，比 rostopic echo 更直观。
不依赖 pharmacy_mplus0 功能包（重复 detection_monitor.py 的功能，
但完全独立，可以在不编译 pharmacy_mplus0 时使用）。

用法:
  rosrun pharmacy_mplus0_debug topic_echo_dashboard.py
"""

from __future__ import print_function

import os
import sys
import time
import threading

import rospy
from std_msgs.msg import String, Int32MultiArray

# 直接写死话题名，避免跨包导入。
_TOPICS = {
    "/cam_return":          Int32MultiArray,
    "/cv1_result":          String,
    "/cv2_result":          String,
    "/current_task":        String,
    "/current_qr_task":     String,
    "/announce_request":    String,
    "/board1_detections":   String,
    "/board2_status":       String,
    "/all_qrcodes":         String,
}


class EchoDashboard(object):
    """独立终端仪表盘。"""

    def __init__(self):
        rospy.init_node("topic_echo_dashboard", anonymous=True)

        self._lock = threading.Lock()
        self._values = {}
        self._times = {}

        for topic, msg_type in _TOPICS.items():
            self._values[topic] = None
            self._times[topic] = None
            rospy.Subscriber(
                topic, msg_type,
                lambda msg, t=topic: self._cb(t, msg),
                queue_size=5,
            )

        rospy.loginfo(
            "[Dashboard] 已订阅 %d 个话题，按 Ctrl-C 退出", len(_TOPICS)
        )

    def _cb(self, topic, msg):
        with self._lock:
            if hasattr(msg, "data"):
                # 截断过长数据。
                val = str(msg.data)
                if len(val) > 80:
                    val = val[:77] + "..."
                self._values[topic] = val
            else:
                self._values[topic] = str(type(msg).__name__)
            self._times[topic] = time.time()

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self._render()
            rate.sleep()

    def _render(self):
        if os.name == "nt":
            os.system("cls")
        else:
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.flush()

        with self._lock:
            values = dict(self._values)
            times = dict(self._times)

        now = time.time()
        print("=" * 60)
        print("  终端仪表盘 — 智慧药房状态")
        print("=" * 60)

        for topic in sorted(values.keys()):
            val = values[topic]
            age = times[topic]
            if age is not None:
                age_str = "%.1fs" % (now - age)
            else:
                age_str = "--"
            display = val if val is not None else "(无数据)"
            print("  [%s] %-30s  %s" % (age_str, topic, display))

        print("=" * 60)


def main():
    try:
        EchoDashboard().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
