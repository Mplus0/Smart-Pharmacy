#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板一初始策略调试工具。

从摄像头读取视频流，显示关键中间结果：
  - Canny 边缘图
  - 面积筛选后的轮廓
  - 四边形轮廓
  - 正方形轮廓
  - 嵌套定位框
  - 透视矫正结果
  - 四个裁剪区域

支持通过 ROS 参数调整检测参数，便于现场调参。

用法:
  rosrun pharmacy_mplus0_debug board1_initial_debugger.py
"""

import os
import sys
import time
import cv2
import numpy as np
import rospy
import rospkg
import yaml

from pharmacy_mplus0.board1_decoder import Board1Decoder

# 默认视频流。
_DEFAULT_STREAM_URL = (
    "http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw"
)


class Board1Debugger(object):
    """识别板一调试器，逐步骤显示中间结果。"""

    def __init__(self):
        rospy.init_node("board1_debugger", anonymous=True)

        # 加载视觉配置。
        vision_config = self._load_vision_config()
        board1_config = vision_config.get("board1", {})

        # 从 ROS 参数读取视频流（可被 launch 覆盖）。
        stream_url = rospy.get_param("~stream_url", None)
        if stream_url is None:
            stream_url = vision_config.get("camera", {}).get(
                "stream_url", _DEFAULT_STREAM_URL
            )
        self._stream_url = stream_url

        # 初始化 Decoder，配置外部可调。
        self._decoder = Board1Decoder(config=board1_config)

        # 窗口名称列表，用于创建 OpenCV 窗口。
        self._windows = [
            "step1_edges",
            "step2_all_contours",
            "step3_quadrilaterals",
            "step4_squares",
            "step5_positioning_boxes",
            "step6_warped",
        ]
        for name in self._windows:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)

        # 打开视频流。
        self._cap = cv2.VideoCapture(self._stream_url)
        if not self._cap.isOpened():
            rospy.logerr("无法打开视频流: %s", self._stream_url)
            sys.exit(1)

        rospy.loginfo("[Board1Debugger] 启动完成，按 q 退出")

    def run(self):
        """主循环：读帧 → 逐步显示中间结果。"""
        while not rospy.is_shutdown():
            ret, frame = self._cap.read()
            if not ret:
                rospy.logwarn_throttle(5.0, "读取帧失败")
                continue

            # 克隆原始帧用于绘制可视化。
            display_frame = frame.copy()

            # 步骤0: 旋转校正。
            angle = self._decoder._cfg["rotate_degrees"]
            if angle != 0.0:
                frame = self._decoder._rotate_frame(frame, angle)
                display_frame = frame.copy()

            # 步骤1: Canny 边缘。
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(
                gray,
                self._decoder._cfg["canny_low"],
                self._decoder._cfg["canny_high"],
                apertureSize=self._decoder._cfg["canny_aperture"],
            )
            cv2.imshow("step1_edges", edges)

            # 步骤2: 所有轮廓（面积筛选后）。
            # OpenCV 3.x 返回 3 值，4.x 返回 2 值，兼容处理。
            found = cv2.findContours(
                edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            contours = found[1] if len(found) == 3 else found[0]
            contours = self._decoder._filter_by_area(contours)
            c2_frame = display_frame.copy()
            cv2.drawContours(c2_frame, contours, -1, (0, 0, 255), 1)
            cv2.putText(
                c2_frame, "count:{}".format(len(contours)),
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )
            cv2.imshow("step2_all_contours", c2_frame)

            # 步骤3: 四边形。
            quads = self._decoder._filter_quadrilaterals(contours)
            c3_frame = display_frame.copy()
            cv2.drawContours(c3_frame, quads, -1, (0, 0, 255), 1)
            cv2.putText(
                c3_frame, "quads:{}".format(len(quads)),
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )
            cv2.imshow("step3_quadrilaterals", c3_frame)

            # 步骤4: 正方形。
            squares = self._decoder._filter_squares(quads)
            c4_frame = display_frame.copy()
            cv2.drawContours(c4_frame, squares, -1, (0, 0, 255), 1)
            cv2.putText(
                c4_frame, "squares:{}".format(len(squares)),
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )
            cv2.imshow("step4_squares", c4_frame)

            # 步骤5: 嵌套定位框。
            jieguo, dingweikuang = (
                self._decoder._find_nested_squares(squares)
            )
            c5_frame = display_frame.copy()
            cv2.drawContours(c5_frame, dingweikuang, -1, (0, 0, 255), 1)
            cv2.putText(
                c5_frame, "locating:{}".format(len(dingweikuang)),
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )

            # 定位框数量达标时尝试透视矫正。
            if len(dingweikuang) == self._decoder._cfg["locating_box_count"]:
                cv2.putText(
                    c5_frame, "FOUND!",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2,
                )
                src_pts, dst_pts = (
                    self._decoder._compute_perspective_corners(jieguo)
                )
                if src_pts is not None:
                    src_arr = np.array(src_pts, dtype=np.float32)
                    dst_arr = np.array(dst_pts, dtype=np.float32)
                    matrix = cv2.getPerspectiveTransform(
                        src_arr, dst_arr
                    )
                    warped = cv2.warpPerspective(
                        frame, matrix,
                        (self._decoder._cfg["warp_width"],
                         self._decoder._cfg["warp_height"]),
                    )
                    cv2.imshow("step6_warped", warped)

                    # 解码四区域并打印。
                    detections = self._decoder.detect(frame)
                    if detections:
                        summary = ", ".join(
                            "{0}(box{1})".format(d.code, d.box_index)
                            for d in detections
                        )
                        rospy.loginfo("识别结果: %s", summary)
            else:
                # 无结果时显示黑色画布。
                blank = np.zeros((600, 600, 3), dtype=np.uint8)
                cv2.imshow("step6_warped", blank)

            cv2.imshow("step5_positioning_boxes", c5_frame)

            # 按 q 退出。
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    def shutdown(self):
        cv2.destroyAllWindows()
        if self._cap is not None:
            self._cap.release()

    @staticmethod
    def _load_vision_config():
        """加载视觉参数，优先从 pharmacy_mplus0 包读取。"""
        try:
            ros_pack = rospkg.RosPack()
            pkg_path = ros_pack.get_path("pharmacy_mplus0")
            config_path = os.path.join(pkg_path, "config", "vision.yaml")
            with open(config_path, "r") as fh:
                return yaml.safe_load(fh) or {}
        except IOError:
            return {}


# ---- 入口 ------------------------------------------------------------

def main():
    debugger = Board1Debugger()
    rospy.on_shutdown(debugger.shutdown)
    try:
        debugger.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
