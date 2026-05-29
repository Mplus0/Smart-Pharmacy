# -*- coding: utf-8 -*-
"""中文日志工具。

专门解决 Python 2 + ROS Melodic 环境中直接使用 rospy.log*
打印中文时抛出的 UnicodeEncodeError / UnicodeDecodeError。

设计思路：
  - 绝不调用 rospy.loginfo / logwarn / logerr / logdebug。
  - 所有输出直接走 sys.stdout.write + flush，自己处理 utf-8 编码。
  - roslaunch 的 output="screen" 会把 stdout 转发到终端，
    体验与 rospy.log* 一致。

用法（与 rospy.log* 完全一致）:
    from pharmacy_mplus0.log_utils import loginfo, logwarn, logerr
    loginfo("[Main] 当前任务=%s", task)
"""

from __future__ import print_function

import sys
import time
import threading


# ---- Py2/Py3 兼容 ----------------------------------------------------

PY2 = sys.version_info[0] == 2

if PY2:
    text_type = eval("unicode")  # noqa — Py2 only
    bytes_type = str
else:
    text_type = str
    bytes_type = bytes


# ---- 内部状态 --------------------------------------------------------

_print_lock = threading.Lock()
_node_name = "?"
_throttle_state = {}


def set_node_name(name):
    """节点初始化时调用，让日志带上节点标签。

    例如:
        set_node_name("main_controller")
        # 后续日志形如 [12:34:56] [INFO ] [main_controller] 消息内容
    """
    global _node_name
    try:
        _node_name = str(name)
    except Exception:
        _node_name = "?"


# ---- 内部工具 --------------------------------------------------------

def _to_text(obj):
    """安全地将任意对象转为 unicode(Py2) / str(Py3)，绝不抛异常。"""
    if obj is None:
        return text_type("")
    if isinstance(obj, text_type):
        return obj
    if PY2 and isinstance(obj, bytes_type):
        try:
            return obj.decode("utf-8")
        except Exception:
            return obj.decode("latin-1", errors="replace")
    try:
        if PY2:
            return text_type(obj)
        return str(obj)
    except Exception:
        try:
            return text_type(repr(obj))
        except Exception:
            return text_type("<unprintable>")


def _format(msg, args):
    """% 格式化，全程使用 unicode(Py2) / str(Py3)，避免编码错误。"""
    msg_t = _to_text(msg)
    if not args:
        return msg_t
    safe_args = []
    for a in args:
        if isinstance(a, (int, float, bool)) or a is None:
            safe_args.append(a)
        else:
            safe_args.append(_to_text(a))
    try:
        return msg_t % tuple(safe_args)
    except Exception:
        # % 格式化失败时退化为空格拼接，确保输出不丢失。
        try:
            parts = [msg_t]
            for a in safe_args:
                parts.append(_to_text(a))
            return text_type(" ").join(parts)
        except Exception:
            return msg_t


def _emit(level, msg, args):
    """直接输出到 stdout，绕过 rospy.log* 的编码陷阱。"""
    try:
        text = _format(msg, args)
        ts = time.strftime("%H:%M:%S")
        line = u"[%s] [%s] [%s] %s" % (ts, level, _node_name, text)

        with _print_lock:
            if PY2:
                # Py2 的 stdout 是 bytes 流，需要手动 encode。
                if isinstance(line, text_type):
                    payload = line.encode("utf-8") + b"\n"
                else:
                    payload = line + "\n"
                sys.stdout.write(payload)
            else:
                sys.stdout.write(line + "\n")
            sys.stdout.flush()
    except Exception:
        # 兜底：确保日志工具本身绝不崩溃业务逻辑。
        try:
            sys.stderr.write("[LOG-FAIL] %s\n" % repr([level, msg, args]))
            sys.stderr.flush()
        except Exception:
            pass


# ---- 对外接口 --------------------------------------------------------

def loginfo(msg, *args):
    """输出 INFO 级别日志。"""
    _emit("INFO ", msg, args)


def logwarn(msg, *args):
    """输出 WARN 级别日志。"""
    _emit("WARN ", msg, args)


def logerr(msg, *args):
    """输出 ERROR 级别日志。"""
    _emit("ERROR", msg, args)


def logdebug(msg, *args):
    """DEBUG 级别日志（默认关闭，减少屏幕输出）。"""
    pass  # 需要时改为 _emit("DEBUG", msg, args)


# ---- Throttle 版本 --------------------------------------------------

def _throttle(level, period, msg, args):
    """节流输出：相同 level+msg 的消息每 period 秒最多输出一次。

    用于 TCP 连接失败等反复触发的场景，防止刷屏。
    """
    key = (level, msg)
    now = time.time()
    last = _throttle_state.get(key, 0)
    if (now - last) >= period:
        _throttle_state[key] = now
        _emit(level, msg, args)


def loginfo_throttle(period, msg, *args):
    """INFO 级别节流日志。"""
    _throttle("INFO ", period, msg, args)


def logwarn_throttle(period, msg, *args):
    """WARN 级别节流日志。"""
    _throttle("WARN ", period, msg, args)
