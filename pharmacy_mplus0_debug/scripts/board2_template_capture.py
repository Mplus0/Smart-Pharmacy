#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""识别板二模板采集工具。

从视频流截取当前帧，保存为模板图片，便于现场重新生成模板。

用法:
  rosrun pharmacy_mplus0_debug board2_template_capture.py _name:=idle
  rosrun pharmacy_mplus0_debug board2_template_capture.py _name:=wait5

命名约定:
  idle.png  → 化验区空闲
  wait5.png ~ wait10.png → 化验区忙碌 N 秒
"""

import os
import sys
import cv2
import rospy
import rospkg

_DEFAULT_STREAM_URL = (
    "http://192.168.124.3:8080/stream?topic=/camera/rgb/image_raw"
)


class Board2TemplateCapture(object):
    """识别板二模板采集器。"""

    def __init__(self):
        rospy.init_node("board2_template_capture", anonymous=True)

        self._template_name = rospy.get_param("~name", "idle")
        stream_url = rospy.get_param("~stream_url", _DEFAULT_STREAM_URL)

        # 输出到 pharmacy_mplus0 的模板目录。
        try:
            ros_pack = rospkg.RosPack()
            pkg_path = ros_pack.get_path("pharmacy_mplus0")
        except rospkg.ResourceNotFound:
            pkg_path = os.getcwd()
        self._output_dir = os.path.join(pkg_path, "templates", "board2")
        if not os.path.isdir(self._output_dir):
            os.makedirs(self._output_dir, exist_ok=True)

        self._cap = cv2.VideoCapture(stream_url)
        if not self._cap.isOpened():
            rospy.logerr("无法打开视频流: %s", stream_url)
            sys.exit(1)
        rospy.loginfo(
            "[Board2Capture] 准备采集模板 '%s'，按 s 保存，按 q 退出",
            self._template_name,
        )

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self._cap.read()
            if not ret:
                rospy.logwarn_throttle(5.0, "读取帧失败")
                continue

            cv2.imshow("capture", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                out_path = os.path.join(
                    self._output_dir,
                    self._template_name + ".png",
                )
                cv2.imwrite(out_path, frame)
                rospy.loginfo("[Board2Capture] 模板已保存: %s", out_path)
            elif key == ord("q"):
                break

    def shutdown(self):
        cv2.destroyAllWindows()
        if self._cap is not None:
            self._cap.release()


def main():
    cap = Board2TemplateCapture()
    rospy.on_shutdown(cap.shutdown)
    try:
        cap.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
