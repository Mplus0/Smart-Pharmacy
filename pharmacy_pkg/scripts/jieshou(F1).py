#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String

def callback(msg):
    # 在这里，你可以根据data.data（即接收到的字符串）的内容做出不同的反应
    if '0' in msg.data or '1' in msg.data:
        data.data = 1
        rospy.loginfo("开始抓取")
    else:
        rospy.loginfo("停止抓取")

def listener():
    # 初始化节点，并指定节点名称（这里为'listener'）
    rospy.init_node('listener_node', anonymous=True)

    # 创建一个订阅者，订阅名为'txt.data'的主题，并指定消息类型为std_msgs/String，回调函数为callback
    rospy.Subscriber("/txt_data", String, callback,queue_size=10)

    # 保持节点运行，直到按下Ctrl+C
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
      
    except rospy.ROSInterruptException:
        pass