#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""TCP 状态上报与音频播报 ROS 节点。

本节点负责：
1. 采集并上报裁判所需状态——速度(/odom)、全局坐标(TF: map→base_footprint)、
   当前任务(/current_task)、CV1、CV2。
2. 以固定频率（默认 2 Hz）向裁判软件发送 JSON，每条末尾带换行。
3. 订阅 /announce_request，调用 AudioAnnouncer 播放 wav 音频。

TCP 连接失败不阻塞本节点，断线自动重连。
音频播放使用 aplay，通过 ~audio_player 参数可切换播放器。

启动方式:
  roslaunch pharmacy_mplus0 reporter.launch
  roslaunch pharmacy_mplus0 main.launch
"""

from __future__ import print_function

import sys
import os

# ---- 启动横幅 --------------------------------------------------------
sys.stdout.write(
    "[tcp_reporter] >>> SCRIPT START (Python %s)\n"
    % sys.version.split()[0]
)
sys.stdout.flush()

# ---- 依赖检查 --------------------------------------------------------
try:
    import rospy
except ImportError as exc:
    sys.stderr.write(
        "[tcp_reporter][FATAL] 无法 import rospy: %s\n"
        "  -> 请确认已经 source devel/setup.bash 并且 ROS 已安装。\n" % exc
    )
    sys.exit(2)

try:
    import tf2_ros
except ImportError as exc:
    sys.stderr.write(
        "[tcp_reporter][FATAL] 无法 import tf2_ros: %s\n"
        "  -> 请安装: sudo apt install ros-melodic-tf2-ros\n" % exc
    )
    sys.exit(2)

import json
import threading
import time
import socket

from nav_msgs.msg import Odometry
from std_msgs.msg import String

from pharmacy_mplus0.log_utils import (
    loginfo, logwarn, logerr, logdebug,
    loginfo_throttle, logwarn_throttle, set_node_name,
)
from pharmacy_mplus0.tcp_client import TcpClient
from pharmacy_mplus0.voice import AudioAnnouncer
from pharmacy_mplus0.constants import (
    TOPIC_ODOM,
    TOPIC_CURRENT_TASK,
    TOPIC_CV1_RESULT,
    TOPIC_CV2_RESULT,
    TOPIC_ANNOUNCE_REQUEST,
    TOPIC_RESET_DETECTION,
)

# 默认参数。
_DEFAULT_SERVER_IP = "192.168.124.2"
_DEFAULT_SERVER_PORT = 9999
_DEFAULT_SEND_HZ = 2.0
_DEFAULT_CAR_ID = "1"
_DEFAULT_AUDIO_DIR = ""
_DEFAULT_AUDIO_PLAYER = "aplay"
_DEFAULT_ALLOW_OVERLAP = False
# 单次 TF 查询的超时时间（秒），不宜过大以免阻塞主循环。
_TF_LOOKUP_TIMEOUT = 0.2


class StateCollector(object):
    """从 ROS 话题和 TF 中采集上报所需的状态数据。

    线程安全：所有回调通过 _lock 保护。
    TF 查询失败时不抛异常，使用上次缓存坐标。
    """

    def __init__(self, map_frame="map", robot_frame="base_footprint"):
        """初始化订阅和 TF 监听器。

        参数:
            map_frame:   全局地图坐标系名（从 waypoints.yaml 的 frames.map 读取）。
            robot_frame: 小车底盘坐标系名（从 waypoints.yaml 的 frames.robot 读取）。
        """
        self._lock = threading.Lock()
        self._speed = 0.0
        self._x = 0.0
        self._y = 0.0
        self._task = "R"
        self._cv1 = ""
        self._cv2 = ""

        self._map_frame = map_frame
        self._robot_frame = robot_frame

        # TF2 监听器，用于查询 map → base_footprint 的全局坐标。
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # 订阅状态话题。
        rospy.Subscriber(
            TOPIC_ODOM, Odometry, self._cb_odom, queue_size=5
        )
        rospy.Subscriber(
            TOPIC_CURRENT_TASK, String, self._cb_task, queue_size=5
        )
        rospy.Subscriber(
            TOPIC_CV1_RESULT, String, self._cb_cv1, queue_size=5
        )
        rospy.Subscriber(
            TOPIC_CV2_RESULT, String, self._cb_cv2, queue_size=5
        )

        loginfo("[State] 订阅初始化完成 (map=%s, robot=%s)",
                map_frame, robot_frame)

    # ---- 回调 -------------------------------------------------------

    def _cb_odom(self, msg):
        with self._lock:
            # 取线速度 x 分量作为当前速度近似值。
            self._speed = msg.twist.twist.linear.x

    def _cb_task(self, msg):
        with self._lock:
            self._task = msg.data or "R"

    def _cb_cv1(self, msg):
        with self._lock:
            self._cv1 = msg.data or ""

    def _cb_cv2(self, msg):
        with self._lock:
            self._cv2 = msg.data or ""

    # ---- TF 查询 ----------------------------------------------------

    def _query_map_pose(self):
        """查询 robot_frame 在 map_frame 下的坐标。

        失败时返回 None，调用方应继续使用上次缓存值。
        """
        try:
            trans = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rospy.Time(0),
                rospy.Duration(_TF_LOOKUP_TIMEOUT),
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            with self._lock:
                self._x = x
                self._y = y
            return x, y
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    # ---- 对外接口 ----------------------------------------------------

    def build_payload(self, car_id):
        """构建一帧待发送的 JSON 字典。

        参数:
            car_id: 小车编号字符串，如 "1"。

        返回:
            dict，包含 id/speed/odom/task/CV1/CV2 字段。
        """
        self._query_map_pose()

        with self._lock:
            speed = self._speed
            x = self._x
            y = self._y
            task = self._task
            cv1 = self._cv1
            cv2 = self._cv2

        return {
            "id": str(car_id),
            "speed": round(float(speed), 3),
            "odom": [round(float(x), 3), round(float(y), 3)],
            "task": str(task),
            "CV1": str(cv1),
            "CV2": str(cv2),
        }


class TcpReporterNode(object):
    """TCP 上报与语音播报主节点。"""

    def __init__(self):
        rospy.init_node("tcp_reporter", anonymous=False)
        set_node_name("tcp_reporter")

        # ---- 读取参数 ------------------------------------------------
        server_ip = rospy.get_param(
            "~server_ip", _DEFAULT_SERVER_IP
        )
        server_port = rospy.get_param(
            "~server_port", _DEFAULT_SERVER_PORT
        )
        send_hz = float(rospy.get_param(
            "~send_hz", _DEFAULT_SEND_HZ
        ))
        audio_dir = rospy.get_param(
            "~audio_dir", _DEFAULT_AUDIO_DIR
        )
        audio_player = rospy.get_param(
            "~audio_player", _DEFAULT_AUDIO_PLAYER
        )
        allow_overlap = rospy.get_param(
            "~allow_overlap", _DEFAULT_ALLOW_OVERLAP
        )
        car_id = str(rospy.get_param(
            "~car_id", _DEFAULT_CAR_ID
        ))

        # 从 tcp.yaml 读取的 connect_timeout / reconnect_seconds / hz。
        connect_timeout = float(rospy.get_param(
            "~connect_timeout_seconds", 1.0
        ))
        reconnect_interval = float(rospy.get_param(
            "~reconnect_seconds", 2.0
        ))

        self._car_id = car_id
        self._send_hz = float(send_hz)

        # ---- 打印配置信息 --------------------------------------------
        loginfo("=" * 60)
        loginfo("[Reporter] tcp_reporter 启动")
        loginfo("[Reporter]   裁判软件: %s:%d", server_ip, server_port)
        loginfo("[Reporter]   小车编号: %s", self._car_id)
        loginfo("[Reporter]   发送频率: %.1f Hz", self._send_hz)
        loginfo("[Reporter]   音频目录: %s", audio_dir)
        loginfo("[Reporter]   音频播放器: %s", audio_player)
        loginfo("[Reporter]   允许重叠: %s", allow_overlap)
        loginfo("=" * 60)

        # ---- 初始化子模块 --------------------------------------------
        # 状态采集器。
        self._collector = StateCollector()

        # TCP 客户端（后台线程自动连接）。
        self._tcp = TcpClient(
            server_ip=server_ip,
            server_port=server_port,
            connect_timeout=connect_timeout,
            reconnect_interval=reconnect_interval,
        )
        # 注入日志，使 TcpClient 的连接/发送事件可追踪。
        self._tcp._log_info = loginfo
        self._tcp._log_warn = logwarn
        self._tcp._log_error = logerr
        self._tcp._log_warn_throttle = logwarn_throttle

        # 音频播放器。
        self._announcer = AudioAnnouncer(
            audio_dir=audio_dir,
            player=audio_player,
            allow_overlap=allow_overlap,
        )
        self._announcer._log_info = loginfo
        self._announcer._log_warn = logwarn

        # ---- 订阅播报请求 --------------------------------------------
        rospy.Subscriber(
            TOPIC_ANNOUNCE_REQUEST, String,
            self._cb_announce, queue_size=10,
        )

        rospy.on_shutdown(self._on_shutdown)

    # ---- 回调 -------------------------------------------------------

    def _cb_announce(self, msg):
        """收到音频事件 ID，立即异步播放。"""
        if msg.data:
            self._announcer.play(str(msg.data))

    def _on_shutdown(self):
        """节点关闭时清理 TCP 连接。"""
        loginfo("[Reporter] 收到 shutdown，关闭 TCP 连接")
        self._tcp.shutdown()
        self._announcer.shutdown()

    # ---- 主循环 -----------------------------------------------------

    def run(self):
        """主循环：定时采集状态并上报到裁判软件。"""
        rate = rospy.Rate(self._send_hz)
        loginfo("[Reporter] 开始上报，频率 %.1f Hz", self._send_hz)

        sent_count = 0
        while not rospy.is_shutdown():
            # 采集并发送一帧。
            payload = self._collector.build_payload(self._car_id)
            was_connected = self._tcp.is_connected()
            self._tcp.send(payload)

            if was_connected:
                sent_count += 1
                if sent_count % 30 == 1:
                    # 每 30 帧（约 15 秒）打印一次进度，避免刷屏。
                    loginfo_throttle(
                        15.0,
                        "[Reporter] 已累计上报 %d 帧到裁判软件",
                        sent_count,
                    )

            rate.sleep()


# ---- 入口 ------------------------------------------------------------

def main():
    try:
        node = TcpReporterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        logerr("[Reporter] 主循环异常: %s", exc)
        raise


if __name__ == "__main__":
    main()
