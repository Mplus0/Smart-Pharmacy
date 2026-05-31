# -*- coding: utf-8 -*-
"""WAV 音频播放封装。

使用系统命令 aplay / paplay / ffplay 异步播放预录制好的 wav 文件。

play() 调用后立即返回，不阻塞主循环。
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


class AudioAnnouncer(object):
    """异步 WAV 音频播放器。

    用法:
        announcer = AudioAnnouncer(audio_dir="/path/to/audio", player="aplay")
        announcer.play("board2_idle")
    """

    # 允许的 event_id 字符：小写字母、数字、下划线。
    _EVENT_ID_ALLOWED = set("abcdefghijklmnopqrstuvwxyz0123456789_")

    def __init__(self, audio_dir=None, player="aplay", allow_overlap=False):
        """初始化音频播放器。

        参数:
            audio_dir:     wav 文件所在目录，默认为空（需通过参数设置）。
            player:        播放命令，默认 "aplay"，可选 "paplay" / "ffplay"。
            allow_overlap: True 允许多音频同时播放；False 会终止上一个。
        """
        self._audio_dir = audio_dir or ""
        self._player_cmd = str(player)
        self._allow_overlap = bool(allow_overlap)
        self._player_path = None
        self._player_warned = False
        self._current_process = None

        # 检查播放器是否可用。
        self._player_path = _which(self._player_cmd)
        if self._player_path is None:
            self._log_warn(
                "系统中未找到 %s，音频播放将被跳过。"
                "安装: sudo apt install alsa-utils" % self._player_cmd
            )

    def play(self, event_id):
        """播放指定事件的 wav 音频，立即返回（异步）。

        参数:
            event_id: 音频事件 ID，如 "board2_idle"。

        返回:
            True  播放已启动。
            False 播放失败（文件不存在 / 播放器不可用 / event_id 非法）。
        """
        if not self._validate_event_id(event_id):
            return False

        wav_path = self._resolve_audio_path(event_id)
        if wav_path is None:
            return False

        return self._play_wav_async(wav_path)

    def shutdown(self):
        """终止当前播放的子进程（如有）。"""
        self._kill_current()

    # ---- 内部 -------------------------------------------------------

    def _validate_event_id(self, event_id):
        """检查 event_id 不含路径穿越等危险字符。"""
        if not event_id or not isinstance(event_id, str):
            self._log_warn("event_id 为空或类型错误: %r", event_id)
            return False
        for ch in str(event_id):
            if ch not in self._EVENT_ID_ALLOWED:
                self._log_warn(
                    "event_id 包含非法字符 '%s': %s", ch, event_id
                )
                return False
        return True

    def _resolve_audio_path(self, event_id):
        """根据 event_id 查找 wav 文件。

        返回完整路径，文件不存在时返回 None 并打印错误日志。
        """
        if not self._audio_dir:
            self._log_warn("audio_dir 未设置，无法定位音频文件")
            return None
        filename = str(event_id) + ".wav"
        full_path = os.path.join(self._audio_dir, filename)
        if not os.path.isfile(full_path):
            self._log_warn("音频文件不存在: %s", full_path)
            return None
        return full_path

    def _play_wav_async(self, wav_path):
        """使用 subprocess.Popen 异步播放 wav 文件。"""
        if self._player_path is None:
            return False

        if not self._allow_overlap:
            self._kill_current()

        try:
            devnull = getattr(subprocess, "DEVNULL", None)
            if devnull is None:
                devnull = open(os.devnull, "wb")
            cmd = [self._player_path, wav_path]
            proc = subprocess.Popen(
                cmd, stdout=devnull, stderr=devnull
            )
            self._current_process = proc
            self._log_info("播放: %s", os.path.basename(wav_path))
            return True
        except OSError as exc:
            if not self._player_warned:
                self._log_warn("%s 调用失败: %s", self._player_cmd, exc)
                self._player_warned = True
            return False

    def _kill_current(self):
        """终止当前正在播放的子进程。"""
        if self._current_process is None:
            return
        try:
            proc = self._current_process
            self._current_process = None
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        except Exception:
            pass

    # ---- 日志 -------------------------------------------------------

    @staticmethod
    def _log_info(msg, *args):
        from pharmacy_mplus0.log_utils import loginfo
        loginfo("[Audio] " + msg, *args)

    @staticmethod
    def _log_warn(msg, *args):
        from pharmacy_mplus0.log_utils import logwarn
        logwarn("[Audio] " + msg, *args)
