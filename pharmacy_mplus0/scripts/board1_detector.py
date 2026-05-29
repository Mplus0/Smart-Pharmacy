#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板一 ROS 节点。

本节点负责：
1. 从 web_video_server 的 HTTP 视频流读取帧。
2. 调用 Board1Decoder 执行定位框筛选 + 透视矫正 + pyzbar 解码。
3. 发布识别结果到 /cam_return（兼容旧格式）、/all_qrcodes（JSON）、
   /board1_detections（新 JSON 格式）。
4. 识别结果连续稳定帧后锁定并持续发布，收到 /reset_detection 后解锁。

该节点不参与业务决策，只输出识别结果。
"""

import json
import os
import time

import cv2
import rospy
import rospkg
import yaml
from std_msgs.msg import Int32MultiArray, String

from pharmacy_mplus0.board1_decoder import Board1Decoder
from pharmacy_mplus0.constants import (
    TOPIC_CAM_RETURN,
    TOPIC_ALL_QRCODES,
    TOPIC_BOARD1_DETECTIONS,
    TOPIC_RESET_DETECTION,
)

# 默认配置文件名。
_VISION_YAML = "vision.yaml"


class Board1DetectorNode(object):
    """识别板一检测 ROS 节点。"""

    def __init__(self):
        rospy.init_node("board1_detector", anonymous=True)

        # ---- 加载视觉参数 -------------------------------------------
        stream_url = rospy.get_param(
            "~stream_url", None
        )
        vision_config = self._load_vision_config()
        board1_config = vision_config.get("board1", {})
        self._stable_frames = board1_config.get("stable_frames", 3)

        if stream_url is None:
            camera_config = vision_config.get("camera", {})
            stream_url = camera_config.get("stream_url", "")
        self._stream_url = stream_url

        # ---- 初始化解码器 -------------------------------------------
        self._decoder = Board1Decoder(config=board1_config)
        rospy.loginfo(
            "[Board1Detector] 解码器已初始化，视频流: %s", stream_url
        )

        # ---- ROS 发布器 ---------------------------------------------
        # /cam_return: Int32MultiArray，兼容旧主控格式。
        self._pub_cam = rospy.Publisher(
            TOPIC_CAM_RETURN, Int32MultiArray, queue_size=5
        )
        # /all_qrcodes: JSON 字符串，包含所有方框的识别结果。
        self._pub_all = rospy.Publisher(
            TOPIC_ALL_QRCODES, String, queue_size=5
        )
        # /board1_detections: 新 JSON 格式，结构更清晰。
        self._pub_detections = rospy.Publisher(
            TOPIC_BOARD1_DETECTIONS, String, queue_size=5
        )

        # ---- 订阅复位话题 -------------------------------------------
        self._locked_result = None
        self._lock_counter = 0
        rospy.Subscriber(
            TOPIC_RESET_DETECTION, String, self._cb_reset
        )

        # ---- 打开视频流 ---------------------------------------------
        self._cap = cv2.VideoCapture(self._stream_url)
        if not self._cap.isOpened():
            rospy.logerr(
                "[Board1Detector] 无法打开视频流: %s", self._stream_url
            )
            rospy.signal_shutdown("视频流打开失败")

        rospy.loginfo(
            "[Board1Detector] 节点启动完成，locking 阈值: %d 帧",
            self._stable_frames,
        )

    # ---- 主循环 ----------------------------------------------------

    def run(self):
        """主循环：读取帧 → 解码 → 发布（循环直到 ROS 关闭）。"""
        rate = rospy.Rate(10)  # 10 Hz 识别频率。
        while not rospy.is_shutdown():
            ret, frame = self._cap.read()
            if not ret:
                rospy.logwarn("[Board1Detector] 读取帧失败，重试中...")
                rate.sleep()
                continue

            # 调用解码器。
            detections = self._decoder.detect(frame)

            # 锁定逻辑：连续 stable_frames 帧结果一致后锁定。
            detections = self._apply_locking(detections)

            # 发布结果。
            if detections:
                self._publish_cam_return(detections)
                self._publish_all_qrcodes(detections)
                self._publish_board1_detections(detections)

            rate.sleep()

    def shutdown(self):
        """释放摄像头资源。"""
        if self._cap is not None:
            self._cap.release()

    # ---- 锁定逻辑 --------------------------------------------------

    def _apply_locking(self, detections):
        """连续 N 帧结果一致后锁定，锁定后持续返回锁定结果。

        锁定期间忽略新的识别结果变化，
        直到收到 /reset_detection 复位信号。
        """
        # 当前已锁定：直接返回锁定的结果。
        if self._locked_result is not None:
            return self._locked_result

        if not detections:
            # 没有识别结果，重置计数器。
            self._lock_counter = 0
            return []

        # 将检测结果转为可哈希的元组，用于比较。
        current_key = self._detections_key(detections)
        if not hasattr(self, "_last_key"):
            self._last_key = None
            self._lock_counter = 0

        if current_key == self._last_key:
            self._lock_counter += 1
        else:
            self._last_key = current_key
            self._lock_counter = 1

        if self._lock_counter >= self._stable_frames:
            self._locked_result = detections
            rospy.loginfo(
                "[Board1Detector] 识别结果锁定: %s",
                self._detections_summary(detections),
            )

        return detections

    def _cb_reset(self, msg):
        """接收到 /reset_detection 后清除锁定，允许更新识别结果。"""
        self._locked_result = None
        self._lock_counter = 0
        self._last_key = None
        rospy.loginfo("[Board1Detector] 锁定已复位")

    # ---- 发布方法 --------------------------------------------------

    def _publish_cam_return(self, detections):
        """发布 /cam_return 旧格式。

        旧格式 Int32MultiArray:
          [is_C, is_A, is_B, count, box_idx, error_code]

        从检测结果中选择样本数最多的二维码发布（与旧脚本行为一致）。
        """
        best = self._pick_best(detections)
        if best is None:
            return

        code = best.code
        is_a = 1 if "A" in code else 0
        is_b = 1 if "B" in code else 0
        is_c = 1 if "C" in code else 0

        msg = Int32MultiArray()
        msg.data = [
            is_c,
            is_a,
            is_b,
            best.sample_count,
            best.box_index,
            0,  # error_code: 正常为 0。
        ]
        self._pub_cam.publish(msg)

    def _publish_all_qrcodes(self, detections):
        """发布所有方框的二维码结果（JSON 字符串）。"""
        items = []
        for d in detections:
            items.append(
                {
                    "code": d.code,
                    "box": d.box_index,
                    "lab_window": d.lab_window,
                    "sample_count": d.sample_count,
                }
            )
        msg = String()
        msg.data = json.dumps(items, ensure_ascii=False)
        self._pub_all.publish(msg)

    def _publish_board1_detections(self, detections):
        """发布新格式的识别结果（含更多字段的 JSON）。

        格式:
          [{"code":"AB","box_index":0,"lab_window":"1","sample_count":2}, ...]
        """
        items = []
        for d in detections:
            items.append(
                {
                    "code": d.code,
                    "box_index": d.box_index,
                    "lab_window": d.lab_window,
                    "sample_count": d.sample_count,
                }
            )
        msg = String()
        msg.data = json.dumps(items, ensure_ascii=False)
        self._pub_detections.publish(msg)

    # ---- 工具方法 --------------------------------------------------

    @staticmethod
    def _pick_best(detections):
        """从检测结果中选样本数最多的（与旧脚本行为一致）。

        样本数相同时选方框索引较小的。
        """
        if not detections:
            return None
        return max(
            detections,
            key=lambda d: (d.sample_count, -d.box_index),
        )

    @staticmethod
    def _detections_key(detections):
        """将检测列表转为可哈希的元组，用于帧间比较。"""
        return tuple(
            (d.code, d.box_index, d.lab_window, d.sample_count)
            for d in sorted(detections, key=lambda x: x.box_index)
        )

    @staticmethod
    def _detections_summary(detections):
        """生成人类可读的结果摘要。"""
        parts = [
            "{0}(box{1})".format(d.code, d.box_index)
            for d in detections
        ]
        return ", ".join(parts) if parts else "[]"

    @staticmethod
    def _load_vision_config():
        """从 pharmacy_mplus0/config/vision.yaml 加载视觉参数。"""
        try:
            ros_pack = rospkg.RosPack()
            pkg_path = ros_pack.get_path("pharmacy_mplus0")
            config_path = os.path.join(pkg_path, "config", _VISION_YAML)
            with open(config_path, "r") as fh:
                data = yaml.safe_load(fh)
            return data if data else {}
        except IOError:
            rospy.logwarn(
                "[Board1Detector] 无法读取 %s，使用默认参数",
                _VISION_YAML,
            )
            return {}


# ---- 入口 ------------------------------------------------------------

def main():
    node = Board1DetectorNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
