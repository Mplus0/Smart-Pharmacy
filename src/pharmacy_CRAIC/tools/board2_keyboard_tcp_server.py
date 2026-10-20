#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
识别板二键盘 TCP 服务端。

运行在另一台电脑上，通过键盘输入板二等待秒数：
    0       化验区空闲
    5~10    化验区忙碌，等待对应秒数
    show    查看当前值
    clear   清除当前值
    q       退出

车辆发送：
    GET\n

服务端返回：
    {"ok":true,"wait_time":7}
或：
    {"ok":false,"error":"no_value"}
"""

from __future__ import print_function

import argparse
import json
import socket
import threading
import time

try:
    input_func = raw_input
except NameError:
    input_func = input


class SharedWaitTime(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._wait_time = None
        self._updated_at = None

    def set(self, value):
        with self._lock:
            self._wait_time = int(value)
            self._updated_at = time.time()

    def clear(self):
        with self._lock:
            self._wait_time = None
            self._updated_at = None

    def snapshot(self):
        with self._lock:
            return self._wait_time, self._updated_at


def recv_line(conn, max_chars=64):
    chunks = []
    total = 0

    while total < max_chars:
        data = conn.recv(min(64, max_chars - total))
        if not data:
            break

        if not isinstance(data, str):
            data = data.decode("utf-8", "replace")

        newline_index = data.find("\n")
        if newline_index >= 0:
            chunks.append(data[:newline_index])
            break

        chunks.append(data)
        total += len(data)

    return "".join(chunks).strip()


def send_json_line(conn, payload):
    text = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
    if not isinstance(text, bytes):
        text = text.encode("utf-8")
    conn.sendall(text)


def handle_client(conn, address, shared):
    try:
        conn.settimeout(2.0)
        command = recv_line(conn).upper()

        if command != "GET":
            send_json_line(conn, {"ok": False, "error": "bad_command"})
            return

        wait_time, updated_at = shared.snapshot()
        if wait_time is None:
            send_json_line(conn, {"ok": False, "error": "no_value"})
            return

        send_json_line(
            conn,
            {
                "ok": True,
                "wait_time": wait_time,
                "updated_at": updated_at,
            }
        )
    except Exception as exc:
        print("[WARN] 处理客户端 %s:%s 失败: %s" %
              (address[0], address[1], exc))
    finally:
        try:
            conn.close()
        except Exception:
            pass


def tcp_server_loop(host, port, shared, stop_event, ready_event):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((host, port))
        server.listen(10)
        server.settimeout(0.5)
        ready_event.set()

        print("[TCP] 正在监听 %s:%d" % (host, port))

        while not stop_event.is_set():
            try:
                conn, address = server.accept()
            except socket.timeout:
                continue

            worker = threading.Thread(
                target=handle_client,
                args=(conn, address, shared)
            )
            worker.daemon = True
            worker.start()
    finally:
        try:
            server.close()
        except Exception:
            pass


def keyboard_loop(shared, stop_event):
    print("")
    print("请输入板二等待秒数：0 表示空闲；5~10 表示忙碌。")
    print("其他命令：show / clear / q")
    print("")

    while not stop_event.is_set():
        try:
            text = input_func("board2> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("")
            stop_event.set()
            break

        if text in ("q", "quit", "exit"):
            stop_event.set()
            break

        if text == "show":
            wait_time, updated_at = shared.snapshot()
            if wait_time is None:
                print("[当前] 尚未设置")
            else:
                print("[当前] %d 秒，更新时间 %.3f" %
                      (wait_time, updated_at))
            continue

        if text == "clear":
            shared.clear()
            print("[OK] 已清除。车辆到达板二后会等待新的键盘输入。")
            continue

        try:
            wait_time = int(text)
        except Exception:
            print("[ERROR] 请输入 0、5、6、7、8、9、10，或 show/clear/q。")
            continue

        if wait_time != 0 and not (5 <= wait_time <= 10):
            print("[ERROR] 只接受 0 或 5~10 秒。")
            continue

        shared.set(wait_time)
        if wait_time == 0:
            print("[OK] 已设置：化验区空闲，等待 0 秒。")
        else:
            print("[OK] 已设置：化验区忙碌，等待 %d 秒。" % wait_time)


def parse_args():
    parser = argparse.ArgumentParser(description="识别板二键盘 TCP 服务端")
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="监听地址，默认 0.0.0.0"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9100,
        help="监听端口，默认 9100"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    shared = SharedWaitTime()
    stop_event = threading.Event()
    ready_event = threading.Event()

    server_thread = threading.Thread(
        target=tcp_server_loop,
        args=(args.host, args.port, shared, stop_event, ready_event)
    )
    server_thread.daemon = True
    server_thread.start()

    if not ready_event.wait(3.0):
        raise RuntimeError("TCP 服务端未能正常启动，请检查端口是否被占用。")

    keyboard_loop(shared, stop_event)
    stop_event.set()
    server_thread.join(1.0)
    print("[TCP] 服务端已退出。")


if __name__ == "__main__":
    main()
