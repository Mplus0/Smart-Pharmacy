#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""模拟识别板一结果，发布 /cam_return。

用于不打开摄像头时调试主控后续流程（导航、播报、采样）。

用法:
  # 模拟 AB 二维码在方框 1（化验窗口 2）
  rosrun pharmacy_mplus0_debug send_fake_cam_return.py _code:=AB _box:=1

  # 模拟 ABC 二维码在方框 3（化验窗口 4）
  rosrun pharmacy_mplus0_debug send_fake_cam_return.py _code:=ABC _box:=3

旧格式: Int32MultiArray [is_C, is_A, is_B, count, box_idx, error_code]
"""

import sys
import rospy
from std_msgs.msg import Int32MultiArray

from pharmacy_mplus0.constants import TOPIC_CAM_RETURN


def main():
    rospy.init_node("send_fake_cam_return", anonymous=True)

    code = rospy.get_param("~code", "AB").upper()
    box_idx = int(rospy.get_param("~box", 0))
    # error_code: 正常为 0。
    error_code = int(rospy.get_param("~error", 0))

    is_a = 1 if "A" in code else 0
    is_b = 1 if "B" in code else 0
    is_c = 1 if "C" in code else 0
    count = is_a + is_b + is_c

    pub = rospy.Publisher(
        TOPIC_CAM_RETURN, Int32MultiArray, queue_size=5
    )
    rospy.sleep(0.5)

    msg = Int32MultiArray()
    msg.data = [is_c, is_a, is_b, count, box_idx, error_code]

    rospy.loginfo(
        "发布 /cam_return: code=%s box=%d data=%s",
        code, box_idx, msg.data,
    )
    pub.publish(msg)

    # 持续发布（latch 不生效时手动保持）。
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
