# -*- coding: utf-8 -*-
"""TTS 语音播报封装。

支持三种后端：
  "espeak"  —— 调用系统 espeak-ng 命令，异步 subprocess，无需额外 ROS 节点。
  "topic"   —— 发布到指定 ROS 话题（对接讯飞等语音模块）。
  "silent"  —— 静默模式，仅记录日志不发声（调试时使用）。

speak() 调用后立即返回，不阻塞主循环。
"""

import os
import sys
import subprocess


def _which(cmd):
    """在 PATH 中查找可执行文件，兼容 Python 2（无 shutil.which）。"""
    for p in os.environ.get("PATH", "").split(os.pathsep):
        full = os.path.join(p, cmd)
        if os.path.isfile(full) and os.access(full, os.X_OK):
            return full
    return None


class VoiceAnnouncer(object):
    """异步语音播报器。

    用法:
        announcer = VoiceAnnouncer(method="espeak")
        announcer.speak("到达血常规窗口，样本数为 3")
    """

    # espeak-ng 中文语音参数。
    ESPEAK_ARGS = ["-vzh", "-s", "140", "-a", "80"]

    def __init__(self, method="espeak", tts_topic="/tts_text"):
        """初始化语音后端。

        参数:
            method:    "espeak" / "topic" / "silent"。
            tts_topic: method="topic" 时使用的 ROS 话题名。
        """
        self._method = str(method).lower()
        self._tts_topic = str(tts_topic)
        self._espeak_path = None
        self._espeak_warned = False
        self._topic_warned = False

        # 延迟导入 rospy —— 本模块可能被非 ROS 脚本复用。
        self._rospy = None
        self._publisher = None

        if self._method == "espeak":
            self._espeak_path = _which("espeak-ng") or _which("espeak")
            if self._espeak_path is None:
                self._log_warn(
                    "系统中未找到 espeak-ng / espeak，语音播报将被跳过。"
                    "安装: sudo apt install espeak-ng espeak-ng-data"
                )
        elif self._method == "topic":
            self._init_topic_publisher()
        elif self._method == "silent":
            self._log_info("静默模式，不发声")
        else:
            self._log_warn("未知 tts_method: %s，回退到 silent", method)
            self._method = "silent"

    def speak(self, text):
        """播报文本，立即返回（异步）。

        参数:
            text: 中文字符串，如 "化验区忙碌中,需等待 8 秒"。
        """
        self._log_info("播报: %s", text)

        # Python 2 兼容：subprocess 不接受 unicode，需编码为 utf-8 bytes。
        text_arg = self._to_bytes(text)

        if self._method == "espeak":
            self._speak_espeak(text_arg)
        elif self._method == "topic":
            self._speak_topic(text)
        # silent 模式仅日志记录，已在上方完成。

    def shutdown(self):
        """清理资源（topic 模式无需操作，espeak 无持久进程）。"""
        pass

    # ---- 内部 -------------------------------------------------------

    def _speak_espeak(self, text_bytes):
        """通过 subprocess.Popen 异步调用 espeak-ng。"""
        if self._espeak_path is None:
            return
        try:
            devnull = getattr(subprocess, "DEVNULL", None)
            if devnull is None:
                devnull = open(os.devnull, "wb")
            cmd = [self._espeak_path] + self.ESPEAK_ARGS + [text_bytes]
            subprocess.Popen(
                cmd, stdout=devnull, stderr=devnull
            )
        except OSError as exc:
            if not self._espeak_warned:
                self._log_warn("espeak-ng 调用失败: %s", exc)
                self._espeak_warned = True

    def _speak_topic(self, text):
        """发布到 ROS 话题。"""
        if not self._ensure_ros():
            return
        try:
            msg = self._rospy_String()
            msg.data = text
            self._publisher.publish(msg)
        except Exception as exc:
            if not self._topic_warned:
                self._log_warn("话题发布失败: %s", exc)
                self._topic_warned = True

    def _init_topic_publisher(self):
        """初始化 ROS 话题发布器。"""
        if not self._ensure_ros():
            return
        try:
            self._publisher = self._rospy.Publisher(
                self._tts_topic, self._rospy_String, queue_size=5
            )
            # 等待 Publisher 在 ROS master 注册完成。
            self._rospy.sleep(0.5)
            self._log_info(
                "话题模式已启用，发布到 %s", self._tts_topic
            )
        except Exception as exc:
            self._log_warn("无法创建话题发布器: %s", exc)

    def _ensure_ros(self):
        """延迟导入 rospy，允许本模块在无 ROS 环境下导入而不崩溃。"""
        if self._rospy is not None:
            return True
        try:
            import rospy as rp
            from std_msgs.msg import String as StdString
            self._rospy = rp
            self._rospy_String = StdString
            return True
        except ImportError:
            self._log_warn("rospy 不可用，topic 模式无法工作")
            return False

    @staticmethod
    def _to_bytes(text):
        """将文本转为 bytes，兼容 Python 2/3。"""
        if sys.version_info[0] == 2:
            if isinstance(text, unicode):  # noqa: F821 — Py2 only
                return text.encode("utf-8")
            return text
        return text.encode("utf-8")

    # ---- 日志占位 ---------------------------------------------------

    def _log_info(self, msg, *args):
        pass

    def _log_warn(self, msg, *args):
        pass
