#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""模拟识别板二结果，发布 /cv1_result。

用于调试主控等待、快速通过和播报逻辑。

用法:
  # 空闲
  rosrun pharmacy_mplus0_debug send_fake_cv1.py _wait:=0

  # 忙碌等待 8 秒
  rosrun pharmacy_mplus0_debug send_fake_cv1.py _wait:=8
"""

import rospy
from std_msgs.msg import String

from pharmacy_mplus0.constants import TOPIC_CV1_RESULT


def main():
    rospy.init_node("send_fake_cv1", anonymous=True)

    wait_sec = int(rospy.get_param("~wait", 0))
    if wait_sec != 0 and not (5 <= wait_sec <= 10):
        rospy.logerr("wait 参数必须为 0 或 5~10")
        return

    body = "WAIT-%d" % wait_sec
    pub = rospy.Publisher(TOPIC_CV1_RESULT, String, queue_size=5)
    rospy.sleep(0.5)

    msg = String()
    msg.data = body
    rospy.loginfo("发布 /cv1_result: %s", body)
    pub.publish(msg)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
