#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板二 ROS 节点。

本节点负责：
1. 从 web_video_server 的 HTTP 视频流读取帧。
2. 调用 Board2Matcher 执行多尺度模板匹配。
3. 通过去抖队列锁定结果后，发布 /cv1_result 和 /board2_status。
4. 收到 /reset_detection 后解锁，允许更新识别。

识别板二展示的内容为：
  - "化验区空闲中，请快速通过"           → WAIT-0
  - "化验区忙碌中，需等待 n 秒" (n=5~10)  → WAIT-5~WAIT-10

该节点不参与业务决策，只输出识别结果。
"""

import os
import sys
from collections import deque

import cv2
import rospy
import rospkg
import yaml
from std_msgs.msg import String

from pharmacy_mplus0.board2_matcher import Board2Matcher
from pharmacy_mplus0.constants import (
    TOPIC_CV1_RESULT,
    TOPIC_BOARD2_STATUS,
    TOPIC_RESET_DETECTION,
)

# 默认配置文件名。
_VISION_YAML = "vision.yaml"

# 视频流默认 URL（现场通过 launch 参数覆盖）。
_DEFAULT_STREAM_URL = (
    "http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw"
)


class Board2DetectorNode(object):
    """识别板二检测 ROS 节点。"""

    def __init__(self):
        rospy.init_node("board2_detector", anonymous=True)

        # ---- 加载参数 -----------------------------------------------
        vision_config = self._load_vision_config()
        board2_config = vision_config.get("board2", {})

        # 模板目录：支持 ROS $(find pkg) 语法展开。
        raw_template_dir = board2_config.get(
            "template_dir",
            "$(find pharmacy_mplus0)/templates/board2",
        )
        self._template_dir = self._resolve_template_dir(raw_template_dir)

        self._stream_url = rospy.get_param(
            "~stream_url",
            vision_config.get("camera", {}).get(
                "stream_url", _DEFAULT_STREAM_URL
            ),
        )
        self._debounce_n = board2_config.get("debounce_frames", 3)
        self._publish_hz = board2_config.get("publish_hz", 5.0)

        # ---- 初始化匹配器 ------------------------------------------
        # 传递 config dict 与 vision.yaml board2 段保持一致。
        matcher_config = {
            "scales": board2_config.get("scales", None),
            "match_threshold": board2_config.get("match_threshold", None),
        }
        # 去除 None 值，让 matcher 使用自身默认值。
        matcher_config = {k: v for k, v in matcher_config.items() if v is not None}
        self._matcher = Board2Matcher(
            self._template_dir, config=matcher_config
        )

        if not self._matcher.is_loaded():
            rospy.logerr(
                "[Board2Detector] 模板目录 %s 中未找到有效模板 "
                "(需要 idle.png / wait5.png~wait10.png)",
                self._template_dir,
            )
        else:
            rospy.loginfo(
                "[Board2Detector] 已加载 %d 个模板，匹配阈值=%.2f，"
                "去抖帧数=%d",
                self._matcher.template_count,
                self._matcher.match_threshold,
                self._debounce_n,
            )

        # ---- ROS 发布器 / 订阅器 ------------------------------------
        # /cv1_result: String "WAIT-0" / "WAIT-5".."WAIT-10"
        self._pub_cv1 = rospy.Publisher(
            TOPIC_CV1_RESULT, String, queue_size=5
        )
        # /board2_status: 新 JSON 格式，含更多字段便于调试。
        self._pub_status = rospy.Publisher(
            TOPIC_BOARD2_STATUS, String, queue_size=5
        )
        # 接收 /reset_detection 复位信号。
        rospy.Subscriber(
            TOPIC_RESET_DETECTION, String, self._cb_reset, queue_size=5
        )

        # ---- 去抖状态 -----------------------------------------------
        # 最近 N 帧的匹配结果（存储 wait_seconds），用于去抖。
        self._recent = deque(maxlen=self._debounce_n)
        # 锁定后的结果，非 None 时持续发布，不再做新匹配。
        self._locked_wait = None

        # ---- 打开视频流 ---------------------------------------------
        self._cap = cv2.VideoCapture(self._stream_url)
        if not self._cap.isOpened():
            rospy.logerr(
                "[Board2Detector] 无法打开视频流: %s", self._stream_url
            )
            rospy.signal_shutdown("视频流打开失败")

        rospy.loginfo("[Board2Detector] 节点启动完成")

    # ---- 主循环 ----------------------------------------------------

    def run(self):
        """主循环：读帧 → 匹配 → 去抖 → 锁定 → 发布。"""
        rate = rospy.Rate(self._publish_hz)
        while not rospy.is_shutdown():
            # 已锁定时直接发布锁定结果，跳过匹配。
            if self._locked_wait is not None:
                self._publish(self._locked_wait)
                rate.sleep()
                continue

            if not self._matcher.is_loaded():
                rate.sleep()
                continue

            ok, frame = self._cap.read()
            if not ok or frame is None:
                rospy.logwarn_throttle(
                    5.0, "[Board2Detector] 读取视频帧失败"
                )
                rate.sleep()
                continue

            # 执行匹配。
            wait_sec, score = self._matcher.match(frame)

            # 分数不达标 → 清空去抖队列，防止噪声积累。
            if wait_sec is None or score < self._matcher.match_threshold:
                self._recent.clear()
                rate.sleep()
                continue

            # 追加到去抖队列。
            self._recent.append(wait_sec)

            # 队列满且全部一致 → 锁定结果。
            if len(self._recent) == self._debounce_n and all(
                r == self._recent[0] for r in self._recent
            ):
                self._locked_wait = self._recent[0]
                rospy.loginfo(
                    "[Board2Detector] 识别结果锁定: WAIT-%d (score=%.3f)",
                    self._locked_wait,
                    score,
                )
                self._publish(self._locked_wait)

            rate.sleep()

    def shutdown(self):
        """释放摄像头资源。"""
        if self._cap is not None:
            self._cap.release()

    # ---- 复位回调 --------------------------------------------------

    def _cb_reset(self, msg):
        """收到 /reset_detection 后清空锁定和去抖队列。"""
        self._locked_wait = None
        self._recent.clear()
        rospy.loginfo("[Board2Detector] 锁定已复位")

    # ---- 内部 -------------------------------------------------------

    def _publish(self, wait_seconds):
        """发布 /cv1_result 和 /board2_status。"""
        body = "WAIT-%d" % wait_seconds
        msg = String()
        msg.data = body
        self._pub_cv1.publish(msg)

        # 新格式：包含 is_idle 布尔字段，方便主控直接判断。
        status_msg = String()
        status_msg.data = (
            '{{"wait_seconds":{0},"is_idle":{1}}}'.format(
                wait_seconds,
                "true" if wait_seconds == 0 else "false",
            )
        )
        self._pub_status.publish(status_msg)

    @staticmethod
    def _resolve_template_dir(raw):
        """展开 $(find pkg) 这样的 ROS 路径占位符。

        如果已经是绝对路径则直接返回；否则用 rospkg 解析。
        """
        raw = str(raw).strip()
        if raw.startswith("$(find "):
            # 匹配 "$(find package_name)/rest"
            rest = raw[len("$(find "):]
            idx = rest.find(")")
            if idx < 0:
                return raw
            pkg_name = rest[:idx].strip()
            sub_path = rest[idx + 1:].lstrip("/")
            try:
                ros_pack = rospkg.RosPack()
                pkg_path = ros_pack.get_path(pkg_name)
            except rospkg.ResourceNotFound:
                rospy.logerr(
                    "[Board2Detector] 找不到 ROS 包: %s", pkg_name
                )
                return raw
            return os.path.join(pkg_path, sub_path) if sub_path else pkg_path
        return raw

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
                "[Board2Detector] 无法读取 %s，使用默认参数",
                _VISION_YAML,
            )
            return {}


# ---- 入口 ------------------------------------------------------------

def main():
    node = Board2DetectorNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
