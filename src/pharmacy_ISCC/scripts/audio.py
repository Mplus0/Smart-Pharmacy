# -*- coding: utf-8 -*-
"""Blocking task speech and bounded board-one interference alert."""
import os
import subprocess
import time
import rospy
from pharmacy_iscc.config import COMMON

AUDIO_DIR = COMMON["paths"]["audio_dir"]

def play_audio(filename):
    """等待播报结束后再继续状态机，避免下一段导航与播报重叠。"""
    if rospy.is_shutdown():
        raise rospy.ROSInterruptException("Audio interrupted by shutdown")
    path = os.path.join(AUDIO_DIR, filename)
    if not os.path.exists(path):
        rospy.logwarn("语音文件不存在: %s", path)
        return
    proc = None
    try:
        proc = subprocess.Popen(["play", path])
        rospy.loginfo("开始播报，等待播放完成: %s", filename)
        while proc.poll() is None:
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException("Audio interrupted by shutdown")
            # 使用墙上时间，避免 ROS 仿真时钟暂停时无法响应停止。
            time.sleep(0.05)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException("Audio interrupted by shutdown")
        if proc.returncode != 0:
            rospy.logwarn("语音播放失败: %s, exit=%s", path, proc.returncode)
        else:
            rospy.loginfo("播报完成: %s", filename)
    finally:
        # 停止任务时结束播放器，避免主任务退出后仍继续播报。
        if proc is not None and proc.poll() is None:
            proc.terminate()
            deadline = time.time() + 0.5
            while proc.poll() is None and time.time() < deadline:
                time.sleep(0.05)
            if proc.poll() is None:
                proc.kill()
            proc.wait()

def play_board1_error_audio():
    """Wait for the common alert before publishing a pickup task; bounded to 8s."""
    path = os.path.join(COMMON["paths"]["audio_dir"], "error.wav")
    if not os.path.isfile(path):
        rospy.logwarn("Board1 interference alert missing: %s", path)
        return False
    proc = None
    try:
        with open(os.devnull, 'wb') as output:
            proc = subprocess.Popen(['play', '-q', path], stdout=output, stderr=output)
            deadline = time.time() + 8.0
            while proc.poll() is None:
                if rospy.is_shutdown() or time.time() >= deadline:
                    rospy.logwarn("Board1 interference audio interrupted or timed out")
                    return False
                time.sleep(0.05)
            if proc.returncode != 0:
                rospy.logwarn("Board1 interference audio failed, exit=%s", proc.returncode)
                return False
        rospy.loginfo("Board1 interference audio completed: error.wav")
        return True
    except OSError as exc:
        rospy.logwarn("Board1 interference audio could not start: %s", exc)
        return False
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            deadline = time.time() + 0.5
            while proc.poll() is None and time.time() < deadline:
                time.sleep(0.05)
            if proc.poll() is None:
                proc.kill()
            proc.wait()


