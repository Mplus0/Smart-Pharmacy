#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
dual_car_link.py

[DUAL-TCP] 双车直连 TCP 通信节点，独立于 referee_client_v2.py。

作用：
1. 订阅本车 ROS 话题 /dual_car/round_done
   消息格式：Int32MultiArray.data = [car_id, seq]

2. 通过 TCP 直接发给另一辆车

3. 监听另一辆车 TCP 发来的 done 消息

4. 收到对方 done 后，在本车 ROS 内发布 /dual_car/peer_done
   消息格式：Int32MultiArray.data = [peer_id, seq]

这样 referee_client_v2.py 可以恢复为只负责裁判系统通信，不再承担双车联动。
"""

import json
import socket
import threading
import time

import rospy
from std_msgs.msg import Int32MultiArray, String

from pharmacy_mplus0.config import COMMON, get_car_config, get_car_id

CAR_ID = get_car_id(default=1)
CAR_CONFIG = get_car_config(CAR_ID)
TOPICS = COMMON["topics"]
TCP_CFG = COMMON["dual_tcp"]
BOARD1_CFG = COMMON["board1"]

PEER_ID = CAR_CONFIG["peer_id"]
LOCAL_HOST = TCP_CFG["local_host"]
LOCAL_PORT = CAR_CONFIG["tcp"]["local_port"]
PEER_IP = CAR_CONFIG["tcp"]["peer_ip"]
PEER_PORT = CAR_CONFIG["tcp"]["peer_port"]
SOCKET_TIMEOUT = TCP_CFG["socket_timeout_sec"]
RETRY_INTERVAL = TCP_CFG["retry_interval_sec"]
SEND_DURATION = TCP_CFG["send_duration_sec"]
PEER_DONE_PUBLISH_REPEAT = TCP_CFG["peer_done_publish_repeat"]
PEER_DONE_PUBLISH_INTERVAL = TCP_CFG["peer_done_publish_interval_sec"]
SHARED_TOKEN = TCP_CFG.get("shared_token")
CHECK_PEER_IP = TCP_CFG.get("check_peer_ip", True)
MAX_LINE_CHARS = TCP_CFG["max_line_chars"]


class DualCarTcpLink(object):
    """双车直连 TCP 节点。"""

    def __init__(self):
        rospy.init_node("dual_car_tcp_link_car%s" % CAR_ID, anonymous=False)
        rospy.on_shutdown(self.cleanup)

        self.stop_event = threading.Event()

        # 待发送队列：每个元素为 {"car_id": int, "seq": int, "expire_time": float}
        self.pending_lock = threading.Lock()
        self.pending_msgs = []
        self.pending_board1_msgs = []

        # 去重：避免重复收到同一个 seq 后反复触发 yaofang
        self.last_peer_seq = 0
        self.last_local_seq = 0
        self.last_peer_board1_seq = 0

        self.pub_peer_done = rospy.Publisher(
            TOPICS["peer_done"],
            Int32MultiArray,
            queue_size=10
        )
        self.pub_peer_board1_all_text = rospy.Publisher(
            BOARD1_CFG["peer_all_text_topic"],
            String,
            queue_size=10,
            latch=True
        )

        self.sub_round_done = rospy.Subscriber(
            TOPICS["round_done"],
            Int32MultiArray,
            self.round_done_cb,
            queue_size=10
        )
        self.sub_board1_all_text = rospy.Subscriber(
            BOARD1_CFG["share_all_text_topic"],
            String,
            self.board1_all_text_cb,
            queue_size=10
        )

        self.server_thread = threading.Thread(target=self.server_loop)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.sender_thread = threading.Thread(target=self.sender_loop)
        self.sender_thread.daemon = True
        self.sender_thread.start()

        rospy.loginfo(
            "[DUAL-TCP] car%s 启动完成。本地监听 %s:%s，对方 car%s=%s:%s",
            CAR_ID,
            LOCAL_HOST,
            LOCAL_PORT,
            PEER_ID,
            PEER_IP,
            PEER_PORT
        )

    def board1_all_text_cb(self, msg):
        """收到本车 /board1_all_text 后，加入 TCP 发送队列。"""
        try:
            obj = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[BOARD1-SHARE] /board1_all_text JSON 解析失败，已忽略: %s", str(e))
            return

        seq = self.safe_int(obj.get("seq", None), None)
        sender_car_id = self.safe_int(obj.get("car_id", None), None)

        if sender_car_id != CAR_ID:
            rospy.logwarn("[BOARD1-SHARE] 收到非本车 all_text，已忽略: car_id=%s seq=%s", sender_car_id, seq)
            return
        if seq is None or seq <= 0:
            rospy.logwarn("[BOARD1-SHARE] all_text seq 非法，已忽略: %s", obj)
            return

        payload = {
            "type": "board1_all_text",
            "car_id": int(sender_car_id),
            "seq": int(seq),
            "all_text": obj.get("all_text", []),
            "selected_index": self.safe_int(obj.get("selected_index", -1), -1),
            "selected_text": obj.get("selected_text", ""),
            "selected_msg": obj.get("selected_msg", []),
        }

        item = {
            "seq": int(seq),
            "payload": payload,
            "expire_time": time.time() + SEND_DURATION
        }

        with self.pending_lock:
            self.pending_board1_msgs = [x for x in self.pending_board1_msgs if x["seq"] != seq]
            self.pending_board1_msgs.append(item)

        rospy.loginfo(
            "[BOARD1-SHARE] all_text 已加入 TCP 发送队列，seq=%s，将在 %.1f 秒内重复发送给 car%s",
            seq,
            SEND_DURATION,
            PEER_ID
        )

    def round_done_cb(self, msg):
        """收到 yaofang 的 /dual_car/round_done 后，加入 TCP 发送队列。"""
        data = list(msg.data)

        if len(data) < 2:
            rospy.logwarn("[DUAL-TCP] /dual_car/round_done 格式错误，应为 [car_id, seq]，实际为: %s", data)
            return

        try:
            done_car_id = int(data[0])
            seq = int(data[1])
        except Exception:
            rospy.logwarn("[DUAL-TCP] /dual_car/round_done 数据无法转为整数: %s", data)
            return

        if done_car_id != CAR_ID:
            rospy.logwarn("[DUAL-TCP] 收到非本车 round_done，已忽略: car_id=%s seq=%s", done_car_id, seq)
            return

        item = {
            "car_id": done_car_id,
            "seq": seq,
            "expire_time": time.time() + SEND_DURATION
        }

        with self.pending_lock:
            # yaofang 可能重复发布同一个 seq；这里先去重，避免短时间创建大量 TCP 连接。
            self.pending_msgs = [x for x in self.pending_msgs if x["seq"] != seq]
            self.pending_msgs.append(item)

        rospy.loginfo(
            "[DUAL-TCP] car%s 本轮 done 已加入发送队列，seq=%s，将在 %.1f 秒内重复发送给 car%s",
            CAR_ID,
            seq,
            SEND_DURATION,
            PEER_ID
        )

    def sender_loop(self):
        """后台发送线程：在 SEND_DURATION 内重复向对方发送消息。"""
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            now = time.time()

            with self.pending_lock:
                active_msgs = []
                still_pending = []
                active_board1_msgs = []
                still_pending_board1 = []

                for item in self.pending_msgs:
                    if item["expire_time"] >= now:
                        active_msgs.append(item)
                        still_pending.append(item)

                for item in self.pending_board1_msgs:
                    if item["expire_time"] >= now:
                        active_board1_msgs.append(item)
                        still_pending_board1.append(item)

                self.pending_msgs = still_pending
                self.pending_board1_msgs = still_pending_board1

            for item in active_msgs:
                self.send_done_once(item["car_id"], item["seq"])

            for item in active_board1_msgs:
                self.send_payload_once(item["payload"], "[BOARD1-SHARE] 已发送 all_text 给 car%s: %s")

            rospy.sleep(RETRY_INTERVAL)

    def send_done_once(self, car_id, seq):
        """单次 TCP 连接发送 JSON。失败没关系，sender_loop 会继续重试。"""
        payload = {
            "type": "round_done",
            "car_id": int(car_id),
            "seq": int(seq)
        }
        self.send_payload_once(payload, "[DUAL-TCP] 已发送 done 给 car%s: %s")

    def send_payload_once(self, payload, success_log_format):
        """单次 TCP 连接发送 JSON。失败没关系，sender_loop 会继续重试。"""
        if SHARED_TOKEN:
            payload["token"] = SHARED_TOKEN

        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(SOCKET_TIMEOUT)
            sock.connect((PEER_IP, PEER_PORT))

            text = json.dumps(payload, ensure_ascii=False) + "\n"
            sock.sendall(text.encode("utf-8"))
            sock.close()

            rospy.loginfo_throttle(
                0.5,
                success_log_format,
                PEER_ID,
                text.strip()
            )

        except Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "[DUAL-TCP] 发送给 car%s 失败，稍后重试: %s",
                PEER_ID,
                str(e)
            )
            try:
                if sock:
                    sock.close()
            except Exception:
                pass

    def server_loop(self):
        """TCP 服务端：监听对方车发来的 done。"""
        server_sock = None

        while not rospy.is_shutdown() and not self.stop_event.is_set():
            try:
                server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_sock.bind((LOCAL_HOST, LOCAL_PORT))
                server_sock.listen(5)
                server_sock.settimeout(TCP_CFG["server_accept_timeout_sec"])

                rospy.loginfo("[DUAL-TCP] car%s TCP 服务端已监听端口 %s", CAR_ID, LOCAL_PORT)

                while not rospy.is_shutdown() and not self.stop_event.is_set():
                    try:
                        conn, addr = server_sock.accept()
                    except socket.timeout:
                        continue

                    t = threading.Thread(target=self.handle_client, args=(conn, addr))
                    t.daemon = True
                    t.start()

            except Exception as e:
                rospy.logerr("[DUAL-TCP] TCP 服务端异常: %s，1 秒后重启监听", str(e))
                try:
                    if server_sock:
                        server_sock.close()
                except Exception:
                    pass
                rospy.sleep(1.0)

        try:
            if server_sock:
                server_sock.close()
        except Exception:
            pass

    def handle_client(self, conn, addr):
        """处理一个 TCP 连接。"""
        buffer_text = ""

        try:
            conn.settimeout(TCP_CFG["client_recv_timeout_sec"])

            while not rospy.is_shutdown() and not self.stop_event.is_set():
                data = conn.recv(4096)

                if not data:
                    break

                try:
                    text = data.decode("utf-8")
                except AttributeError:
                    text = data
                except Exception:
                    text = str(data)

                buffer_text += text
                if len(buffer_text) > MAX_LINE_CHARS:
                    rospy.logwarn("[DUAL-TCP] 收到超长数据，已关闭连接，addr=%s", addr)
                    break

                while "\n" in buffer_text:
                    line, buffer_text = buffer_text.split("\n", 1)
                    self.handle_line(line.strip(), addr)

        except Exception as e:
            rospy.logwarn_throttle(2.0, "[DUAL-TCP] 处理连接 %s 异常: %s", addr, str(e))

        try:
            conn.close()
        except Exception:
            pass

    def handle_line(self, line, addr):
        """解析一行 JSON。"""
        if not line:
            return

        try:
            obj = json.loads(line)
        except Exception:
            rospy.logwarn("[DUAL-TCP] 收到非 JSON 数据，已忽略，addr=%s, data=%s", addr, line)
            return

        if CHECK_PEER_IP and addr and addr[0] != PEER_IP:
            rospy.logwarn("[DUAL-TCP] 收到非配置对方 IP 的连接，已忽略: addr=%s expected=%s", addr, PEER_IP)
            return

        if SHARED_TOKEN and obj.get("token") != SHARED_TOKEN:
            rospy.logwarn("[DUAL-TCP] 收到 token 错误的消息，已忽略: addr=%s", addr)
            return

        msg_type = obj.get("type", "")
        sender_car_id = self.safe_int(obj.get("car_id", None), None)
        seq = self.safe_int(obj.get("seq", None), None)

        if msg_type == "round_done":
            self.handle_round_done_obj(obj, sender_car_id, seq)
            return

        if msg_type == "board1_all_text":
            self.handle_board1_all_text_obj(obj, sender_car_id, seq)
            return

        else:
            rospy.logwarn("[DUAL-TCP] 收到未知消息类型，已忽略: %s", obj)
            return

    def handle_round_done_obj(self, obj, sender_car_id, seq):
        """处理对车 round_done。"""

        if sender_car_id != PEER_ID:
            rospy.logwarn(
                "[DUAL-TCP] 收到非对方车辆 done，已忽略: sender=%s expected=%s obj=%s",
                sender_car_id,
                PEER_ID,
                obj
            )
            return

        if seq is None or seq <= 0:
            rospy.logwarn("[DUAL-TCP] 收到 seq 非法的 done，已忽略: %s", obj)
            return

        if seq <= self.last_peer_seq:
            rospy.loginfo_throttle(
                1.0,
                "[DUAL-TCP] 收到重复或旧的 car%s done，已忽略: seq=%s last=%s",
                sender_car_id,
                seq,
                self.last_peer_seq
            )
            return

        self.last_peer_seq = seq
        self.publish_peer_done(sender_car_id, seq)

    def handle_board1_all_text_obj(self, obj, sender_car_id, seq):
        """处理对车 board1_all_text。"""
        if sender_car_id != PEER_ID:
            rospy.logwarn(
                "[BOARD1-SHARE] 收到非对方车辆 all_text，已忽略: sender=%s expected=%s obj=%s",
                sender_car_id,
                PEER_ID,
                obj
            )
            return

        if seq is None or seq <= 0:
            rospy.logwarn("[BOARD1-SHARE] 收到 seq 非法的 all_text，已忽略: %s", obj)
            return

        if seq <= self.last_peer_board1_seq:
            rospy.loginfo_throttle(
                1.0,
                "[BOARD1-SHARE] 收到重复或旧的 car%s all_text，已忽略: seq=%s last=%s",
                sender_car_id,
                seq,
                self.last_peer_board1_seq
            )
            return

        all_text = obj.get("all_text", None)
        selected_msg = obj.get("selected_msg", None)
        selected_index = self.safe_int(obj.get("selected_index", None), None)
        if not isinstance(all_text, list) or not isinstance(selected_msg, list) or selected_index is None:
            rospy.logwarn("[BOARD1-SHARE] all_text 字段格式错误，已忽略: %s", obj)
            return

        self.last_peer_board1_seq = seq

        out_obj = dict(obj)
        if "token" in out_obj:
            del out_obj["token"]

        msg = String()
        msg.data = json.dumps(out_obj, ensure_ascii=False)

        for _ in range(3):
            self.pub_peer_board1_all_text.publish(msg)
            rospy.sleep(0.03)

        rospy.loginfo("[BOARD1-SHARE] 已发布对车 all_text 到 ROS: seq=%s", seq)

    def publish_peer_done(self, peer_id, seq):
        """收到对方 TCP done 后，发布本地 ROS /dual_car/peer_done。"""
        msg = Int32MultiArray()
        msg.data = [int(peer_id), int(seq)]

        for _ in range(PEER_DONE_PUBLISH_REPEAT):
            self.pub_peer_done.publish(msg)
            rospy.sleep(PEER_DONE_PUBLISH_INTERVAL)

        rospy.loginfo(
            "[DUAL-TCP] 已发布 /dual_car/peer_done: peer_id=%s seq=%s",
            peer_id,
            seq
        )

    def safe_int(self, value, default=None):
        try:
            return int(value)
        except Exception:
            return default

    def cleanup(self):
        rospy.loginfo("[DUAL-TCP] shutting down car%s tcp link...", CAR_ID)
        self.stop_event.set()

        if hasattr(self, "server_thread") and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)

        if hasattr(self, "sender_thread") and self.sender_thread.is_alive():
            self.sender_thread.join(timeout=1.0)


if __name__ == "__main__":
    try:
        node = DualCarTcpLink()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
