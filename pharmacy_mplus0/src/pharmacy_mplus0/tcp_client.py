# -*- coding: utf-8 -*-
"""TCP 连接与发送封装。

管理与裁判软件之间的 TCP 长连接：
- 后台线程自动连接，init 不阻塞。
- send() 时若未连接则丢弃当前帧，不影响主控。
- 断线自动重连，发送失败时关闭 socket 等待重建。

本模块不依赖 ROS，仅依赖 Python 标准库 socket/threading/json。
"""

import json
import socket
import threading
import time


class TcpClient(object):
    """异步 TCP 客户端，用于向裁判软件上报 JSON 状态数据。

    用法:
        client = TcpClient("192.168.12.16", 8888, connect_timeout=1.0)
        client.send({"id": "1", "task": "A", ...})
        ...
        client.shutdown()
    """

    def __init__(self, server_ip, server_port,
                 connect_timeout=1.0, reconnect_interval=2.0):
        """初始化但不连接 —— 连接在后台线程中进行。

        参数:
            server_ip:          裁判电脑 IP。
            server_port:        裁判软件监听端口。
            connect_timeout:    单次 connect 超时秒数。
            reconnect_interval: 断线后重连间隔秒数。
        """
        self._server_ip = str(server_ip)
        self._server_port = int(server_port)
        self._connect_timeout = float(connect_timeout)
        self._reconnect_interval = float(reconnect_interval)

        self._sock = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # 启动后台连接线程。
        self._thread = threading.Thread(
            target=self._connect_loop, name="tcp_connect"
        )
        self._thread.daemon = True
        self._thread.start()

    # ---- 对外接口 ----------------------------------------------------

    def send(self, payload_dict):
        """将字典序列化为 JSON 并尝试发送。

        每条消息末尾追加换行符，便于裁判端按行解析（防粘包）。
        - 未连接时：静默丢弃本帧。
        - 发送失败时：关闭 socket，由后台线程重建连接。
        """
        try:
            json_str = json.dumps(payload_dict, ensure_ascii=False)
            data = (json_str + "\n").encode("utf-8")
        except (TypeError, ValueError, UnicodeError) as exc:
            # JSON 序列化失败 —— 不应发生，仅兜底。
            self._log_error("JSON 序列化失败: %s", exc)
            return

        with self._lock:
            sock = self._sock
        if sock is None:
            return  # 未连接，静默丢弃。

        try:
            sock.sendall(data)
        except (OSError, socket.error) as exc:
            self._log_warn("发送失败: %s，关闭 socket 等待重连", exc)
            with self._lock:
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except Exception:
                        pass
                    self._sock = None

    def is_connected(self):
        """查询当前是否有可用连接。"""
        with self._lock:
            return self._sock is not None

    def shutdown(self):
        """关闭连接和后台线程。"""
        self._stop_event.set()
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    # ---- 后台连接循环 ------------------------------------------------

    def _connect_loop(self):
        """后台线程：持续尝试连接直到 stop_event 被设置。"""
        attempt = 0
        while not self._stop_event.is_set():
            if self.is_connected():
                # 已连接时每 2 秒检查一次状态。
                self._stop_event.wait(2.0)
                continue

            attempt += 1
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(self._connect_timeout)
                s.connect((self._server_ip, self._server_port))
                s.settimeout(None)  # 恢复阻塞模式，sendall 自带超时语义。
                with self._lock:
                    self._sock = s
                self._log_info(
                    "已连接裁判软件 %s:%d (尝试 %d 次)",
                    self._server_ip, self._server_port, attempt,
                )
                attempt = 0
            except Exception as exc:
                self._log_warn_throttle(
                    5.0,
                    "连接 %s:%d 失败 (第 %d 次): %s",
                    self._server_ip, self._server_port, attempt, exc,
                )
                self._stop_event.wait(self._reconnect_interval)

    # ---- 日志占位（接入 log_utils 后可替换）----------------------------

    def _log_info(self, msg, *args):
        pass  # 后续由 tcp_reporter 节点统一管理日志。

    def _log_warn(self, msg, *args):
        pass

    def _log_error(self, msg, *args):
        pass

    def _log_warn_throttle(self, period, msg, *args):
        pass
