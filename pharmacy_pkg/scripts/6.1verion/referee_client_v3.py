#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
referee_client.py

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

from nav_msgs.msg import Odometry
from std_msgs.msg import String

from dual_car_config import COMMON, get_car_config, get_car_id

CAR_ID = get_car_id(default=1)
CAR_CONFIG = get_car_config(CAR_ID)
TOPICS = COMMON["topics"]
REFEREE_CFG = COMMON["referee"]

SERVER_IP = REFEREE_CFG["server_ip"]
SERVER_PORT = REFEREE_CFG["server_port"]
SEND_RATE_HZ = REFEREE_CFG["send_rate_hz"]
SOCKET_TIMEOUT = REFEREE_CFG["socket_timeout_sec"]
RECONNECT_SLEEP = REFEREE_CFG["reconnect_sleep_sec"]


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

        self.init_subscribers()
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
        """订阅用于更新 payload 的 ROS 话题。"""
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
        """从 /odometry/filtered 同时更新 speed 和 odom。"""

        # 位置：来自 /odometry/filtered 的 pose
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)

        # 速度：来自 /odometry/filtered 的 twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        speed = round(math.sqrt(vx * vx + vy * vy), 2)

        with self.payload_lock:
            self.payload["speed"] = speed
            self.payload["odom"] = [x, y]

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
