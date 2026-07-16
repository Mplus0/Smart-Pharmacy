#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
referee_reporter.py

[2026 CRAIC 智慧药房赛项] 官方指定协议通信节点。

功能：
- 订阅小车速度、定位坐标、任务状态、CV1、CV2；
- 通过 TCP 按 2Hz 向裁判系统发送 JSON 数据；
- 连接断开后自动重连。
"""

import json
import socket
import threading

import rospy
import math
import tf

from nav_msgs.msg import Odometry
from std_msgs.msg import String

from pharmacy_mplus0.config import COMMON, get_car_config, get_car_id

CAR_ID = get_car_id(default=1)
CAR_CONFIG = get_car_config(CAR_ID)
TOPICS = COMMON["topics"]
REFEREE_CFG = COMMON["referee"]

SERVER_IP = REFEREE_CFG["server_ip"]
SERVER_PORT = REFEREE_CFG["server_port"]
SEND_RATE_HZ = REFEREE_CFG["send_rate_hz"]
SOCKET_TIMEOUT = REFEREE_CFG["socket_timeout_sec"]
RECONNECT_SLEEP = REFEREE_CFG["reconnect_sleep_sec"]

# 裁判 odom 坐标从 TF 读取：map -> base_footprint / base_link
TF_MAP_FRAME = "map"
TF_BASE_FRAMES = ["base_footprint", "base_link"]

# TF 位置更新频率。裁判发送频率是 2Hz，这里 10Hz 足够。
TF_UPDATE_RATE_HZ = 4.0



class RefereeClient(object):
    """裁判系统 TCP 客户端。"""

    def __init__(self):
        rospy.init_node('referee_client_node', anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # 比赛现场必须修改这里的 IP 和端口
        self.server_ip = SERVER_IP
        self.server_port = SERVER_PORT

        self.tcp_client = None
        self.is_connected = False
        self.stop_event = threading.Event()

        # payload 会被发送线程和 ROS 回调线程同时访问，因此加锁保护
        self.payload_lock = threading.Lock()
        self.payload = self.default_payload()

        # TF 监听器：从 /tf 查询机器人在 map 坐标系下的位置
        self.tf_listener = tf.TransformListener()

        # 记录当前成功使用的底盘坐标系，优先 base_footprint，失败再用 base_link
        self.active_base_frame = None

        self.init_subscribers()

        # 定时用 TF 更新 payload["odom"]
        self.tf_timer = rospy.Timer(
            rospy.Duration(1.0 / TF_UPDATE_RATE_HZ),
            self.tf_pose_timer_cb
        )

        self.start_send_thread()

    def default_payload(self):
        """创建裁判系统要求的默认 JSON 数据。"""
        return {
            "id": str(CAR_CONFIG["car_id"]),  # 小车编号来自集中配置
            "speed": 0.0,       # 速度 m/s
            "odom": [0.0, 0.0], # 地图二维坐标 [x, y]
            "task": "R",        # "1"~"4", "A"~"C", 或 "R"
            "CV1": "None",    # 识别板二结果
            "CV2": "None"           # 识别板一结果，未识别前不要使用类似 "AB-1" 的真实值
        }

    def init_subscribers(self):
        """订阅用于更新 payload 的 ROS 话题。

        speed 来自 /odometry/filtered.twist；
        odom 坐标来自 TF 的 map -> base_footprint/base_link。
        """
        rospy.Subscriber(TOPICS["odom"], Odometry, self.filtered_odom_cb, queue_size=10)
        rospy.Subscriber(TOPICS["referee_task"], String, self.task_cb, queue_size=10)
        rospy.Subscriber(TOPICS["referee_cv1"], String, self.cv1_cb, queue_size=10)
        rospy.Subscriber(TOPICS["referee_cv2"], String, self.cv2_cb, queue_size=10)

    def start_send_thread(self):
        """启动后台发送线程。"""
        self.thread = threading.Thread(target=self.send_loop)
        self.thread.daemon = True
        self.thread.start()

    def connect_server(self):
        """循环尝试连接裁判服务器，直到连接成功或节点退出。"""
        while (not rospy.is_shutdown()
               and not self.stop_event.is_set()
               and not self.is_connected):
            try:
                self.close_socket()

                self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_client.settimeout(SOCKET_TIMEOUT)
                self.tcp_client.connect((self.server_ip, self.server_port))

                self.is_connected = True
                rospy.loginfo("[裁判系统] TCP 连接成功！")

            except Exception as e:
                rospy.logwarn_throttle(
                    3,
                    "[裁判系统] 正在尝试连接裁判服务器: %s:%s，原因: %s",
                    self.server_ip,
                    self.server_port,
                    str(e)
                )
                rospy.sleep(RECONNECT_SLEEP)

    def send_loop(self):
        """后台持续发送数据。"""
        rate = rospy.Rate(SEND_RATE_HZ)

        while not rospy.is_shutdown() and not self.stop_event.is_set():
            if not self.is_connected:
                self.connect_server()
                if not self.safe_sleep(rate):
                    break
                continue

            try:
                # 复制一份 payload 再发送，避免发送过程中被回调修改
                with self.payload_lock:
                    payload_copy = dict(self.payload)

                json_str = json.dumps(payload_copy, ensure_ascii=False) + "\n"
                self.tcp_client.sendall(json_str.encode('utf-8'))

                rospy.loginfo_throttle(
                    0.5,
                    "正常发送裁判数据: %s",
                    json_str.strip()
                )

            except Exception as e:
                rospy.logerr("[裁判系统] TCP 发送失败，连接已断开: %s", str(e))
                self.is_connected = False
                self.close_socket()

            if not self.safe_sleep(rate):
                break

    def close_socket(self):
        """关闭当前 socket。"""
        if self.tcp_client:
            try:
                self.tcp_client.close()
            except Exception:
                pass
            self.tcp_client = None

    def safe_sleep(self, rate):
        """封装 rate.sleep，节点退出时返回 False。"""
        try:
            rate.sleep()
            return True
        except rospy.ROSInterruptException:
            return False

    def filtered_odom_cb(self, msg):
        """从 /odometry/filtered 只更新 speed；odom 坐标由 TF 更新。"""

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        speed = round(math.sqrt(vx * vx + vy * vy), 2)

        with self.payload_lock:
            self.payload["speed"] = speed

    def tf_pose_timer_cb(self, event):
        """从 TF 获取机器人在 map 坐标系下的位置，用于裁判 JSON 的 odom 字段。"""

        frames = []

        # 优先使用上一次成功的 base frame
        if self.active_base_frame:
            frames.append(self.active_base_frame)

        # 第一次优先 base_footprint，失败再 base_link
        for frame in TF_BASE_FRAMES:
            if frame not in frames:
                frames.append(frame)

        for base_frame in frames:
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    TF_MAP_FRAME,
                    base_frame,
                    rospy.Time(0)
                )

                x = round(float(trans[0]), 2)
                y = round(float(trans[1]), 2)

                with self.payload_lock:
                    self.payload["odom"] = [x, y]

                self.active_base_frame = base_frame
                return

            except Exception as e:
                rospy.logwarn_throttle(
                    2.0,
                    "[裁判系统] TF 查询失败: %s -> %s，原因: %s",
                    TF_MAP_FRAME,
                    base_frame,
                    str(e)
                )

    def task_cb(self, msg):
        """更新 task 字段。"""
        valid_tasks = REFEREE_CFG["valid_tasks"]

        if msg.data not in valid_tasks:
            rospy.logwarn("收到非法 task，已忽略: %s", msg.data)
            return

        with self.payload_lock:
            self.payload["task"] = msg.data

    def cv1_cb(self, msg):
        """更新 CV1 字段。"""
        with self.payload_lock:
            self.payload["CV1"] = msg.data

    def cv2_cb(self, msg):
        """更新 CV2 字段。"""
        with self.payload_lock:
            self.payload["CV2"] = msg.data

    def cleanup(self):
        """节点退出时清理资源。"""
        rospy.loginfo("shutting down referee client...")

        self.stop_event.set()
        self.is_connected = False

        if self.tcp_client:
            try:
                self.tcp_client.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self.close_socket()

        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=1.0)


if __name__ == '__main__':
    try:
        client = RefereeClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
