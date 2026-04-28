#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import hashlib
import time
import rospkg
def read_and_publish_file(file_path,topic_name):
    # 初始化文件内容的哈希值，用于检测变化
    old_hash = ""

    # ROS发布者
    rospy.init_node('file_msg', anonymous=True)
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    
    rate = rospy.Rate(1)  # 1hz
    msg = String()

    while not rospy.is_shutdown():
        try:
            with open(file_path, 'r') as file:
                msg.data = file.read()
                #current_hash = hashlib.sha256(msg.encode()).hexdigest()

                # 检查文件内容是否发生变化
                if msg.data != old_hash:
                    old_hash = msg.data
                    rospy.loginfo("%s",old_hash)
                    pub.publish(msg.data)

        except IOError:
            rospy.logerr("Error reading file: " + file_path)

        rate.sleep()

if __name__ == '__main__':
    # 假设你的txt文件位于ROS包的同一目录下，名为'data.txt'
    # 并且你想要发布到名为'txt_data'的话题上
    file_path = '/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/yolosign.txt'
    topic_name = 'txt_data'
    
    # 运行文件监视器
    read_and_publish_file(file_path, topic_name)

'''
if __name__ == '__main__':
    try:
        # 参数解析，从命令行获取文件路径和话题名称
        import sys

        if len(sys.argv) != 3:
            print("Usage: python script.py <file_path> <topic_name>")
            sys.exit(1)
        #file_path = '/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/yolosign.txt'
        file_path = sys.argv[1]
        topic_name = sys.argv[2]

        read_and_publish_file(file_path, topic_name)
    except rospy.ROSInterruptException:
        pass
'''
