#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""模拟裁判 TCP 服务端。

在本机指定端口监听 TCP 连接，接收每行 JSON 并格式化打印。
用于在没有裁判软件时验证上报格式和频率。

用法:
  # 默认端口 8888
  rosrun pharmacy_mplus0_debug tcp_fake_server.py

  # 自定义端口
  rosrun pharmacy_mplus0_debug tcp_fake_server.py _port:=9999
"""

from __future__ import print_function

import json
import socket
import threading
import time

import rospy


class FakeTcpServer(object):
    """模拟裁判 TCP 服务端。"""

    def __init__(self):
        rospy.init_node("tcp_fake_server", anonymous=True)

        port = int(rospy.get_param("~port", 9999))
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
        )
        self._server.bind(("0.0.0.0", port))
        self._server.listen(1)
        self._server.settimeout(1.0)

        rospy.loginfo(
            "[FakeServer] 监听 0.0.0.0:%d，等待 tcp_reporter 连接...",
            port,
        )
        self._counter = 0

    def run(self):
        while not rospy.is_shutdown():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue

            rospy.loginfo("[FakeServer] 客户端已连接: %s:%d", *addr)
            buffer_data = b""

            try:
                while not rospy.is_shutdown():
                    data = conn.recv(4096)
                    if not data:
                        rospy.loginfo("[FakeServer] 客户端断开")
                        break
                    buffer_data += data

                    # 按换行解析 JSON。
                    while b"\n" in buffer_data:
                        line, buffer_data = buffer_data.split(
                            b"\n", 1
                        )
                        try:
                            obj = json.loads(
                                line.decode("utf-8")
                            )
                            self._counter += 1
                            self._print_payload(obj)
                        except (ValueError, UnicodeError) as exc:
                            rospy.logwarn(
                                "[FakeServer] JSON 解析失败: %s", exc
                            )
            except socket.error as exc:
                rospy.logwarn("[FakeServer] 连接异常: %s", exc)
            finally:
                conn.close()

    def _print_payload(self, obj):
        """格式化打印一条上报。"""
        rospy.loginfo(
            "#%-4d | car=%s  speed=%.3f  odom=(%.3f,%.3f)  "
            "task=%s  CV1=%s  CV2=%s",
            self._counter,
            obj.get("id", "?"),
            obj.get("speed", 0),
            obj.get("odom", [0, 0])[0],
            obj.get("odom", [0, 0])[1],
            obj.get("task", "?"),
            obj.get("CV1", "?"),
            obj.get("CV2", "?"),
        )


def main():
    try:
        FakeTcpServer().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
