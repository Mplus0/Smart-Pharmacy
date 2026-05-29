#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""模拟裁判可见任务状态，发布 /current_task、/cv2_result、/announce_request。

用于单独测试 tcp_reporter.py 和裁判软件显示。

用法:
  # 模拟正在体检窗口 A
  rosrun pharmacy_mplus0_debug send_fake_task.py _task:=A

  # 模拟停在化验窗口 1
  rosrun pharmacy_mplus0_debug send_fake_task.py _task:=1 _cv2:=AB-1
"""

import rospy
from std_msgs.msg import String

from pharmacy_mplus0.constants import (
    TOPIC_CURRENT_TASK,
    TOPIC_CV1_RESULT,
    TOPIC_CV2_RESULT,
    TOPIC_ANNOUNCE_REQUEST,
)
from pharmacy_mplus0.log_utils import loginfo


def main():
    rospy.init_node("send_fake_task", anonymous=True)

    task = rospy.get_param("~task", "R")
    cv1 = rospy.get_param("~cv1", "WAIT-0")
    cv2 = rospy.get_param("~cv2", "")
    announce = rospy.get_param("~announce", "")

    pub_task = rospy.Publisher(
        TOPIC_CURRENT_TASK, String, queue_size=5
    )
    pub_cv1 = rospy.Publisher(
        TOPIC_CV1_RESULT, String, queue_size=5
    )
    pub_cv2 = rospy.Publisher(
        TOPIC_CV2_RESULT, String, queue_size=5
    )
    pub_announce = rospy.Publisher(
        TOPIC_ANNOUNCE_REQUEST, String, queue_size=5
    )
    rospy.sleep(0.5)

    loginfo(
        "发布模拟状态: task=%s cv1=%s cv2=%s announce=%s",
        task, cv1, cv2, announce or "(无)",
    )

    pub_task.publish(String(data=task))
    pub_cv1.publish(String(data=cv1))
    if cv2:
        pub_cv2.publish(String(data=cv2))
    if announce:
        pub_announce.publish(String(data=announce))

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        pub_task.publish(String(data=task))
        pub_cv1.publish(String(data=cv1))
        if cv2:
            pub_cv2.publish(String(data=cv2))
        rate.sleep()


if __name__ == "__main__":
    main()
