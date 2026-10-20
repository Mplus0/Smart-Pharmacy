# -*- coding: utf-8 -*-
"""Board2 GET protocol. Cancellation remains owned by the perception runtime."""
import json
import socket
import time
from pharmacy_iscc.config.settings import COMMON

BOARD2_CFG = COMMON["board2"]


def query_board2_tcp(cancelled):
    """一次 GET 查询；失败或服务端尚未设置值时不生成识别结果。"""
    conn = None
    try:
        deadline = time.time() + float(BOARD2_CFG["tcp_timeout_sec"])
        conn = socket.create_connection(
            (BOARD2_CFG["tcp_host"], int(BOARD2_CFG["tcp_port"])),
            float(BOARD2_CFG["tcp_timeout_sec"]))
        conn.sendall(b'GET\n')
        data = b''
        while b'\n' not in data:
            if cancelled():
                return None
            remaining = deadline - time.time()
            if remaining <= 0:
                raise ValueError('response timed out')
            if len(data) >= 4096:
                raise ValueError('response too long')
            conn.settimeout(remaining)
            chunk = conn.recv(min(1024, 4096 - len(data)))
            if not chunk:
                raise ValueError('connection closed before newline')
            data += chunk
        payload = json.loads(data.split(b'\n', 1)[0].decode('utf-8'))
        if not isinstance(payload, dict):
            raise ValueError('response must be a JSON object')
        if payload.get('ok') is not True:
            return None
        wait_time = payload.get('wait_time')
        if (isinstance(wait_time, bool) or not isinstance(wait_time, int)
                or (wait_time != 0 and not 5 <= wait_time <= 10)):
            raise ValueError('wait_time must be 0 or an integer from 5 to 10')
        return 'free' if wait_time == 0 else 'busy_%d' % wait_time
    except (socket.error, ValueError, TypeError):
        return None
    finally:
        if conn is not None:
            conn.close()
