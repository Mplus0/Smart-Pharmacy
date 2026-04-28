#! /usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

import os
import time

array_data = [0,3] # 保存视觉识别传回来的信息
num = 10 # 小车取药的轮数

def time_delay(s):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time<s):
            xyz=0

def my_shutdown():
    rospy.loginfo("Task over!")
    
def cam_callback(msg):
    # 接收到消息时的回调函数
    global array_data
    array_data = msg.data
    rospy.loginfo("Received array: %s", array_data)

def publisher():


    # 以下列表为每一轮小车在配药区和取药区停留的位置
    # 如第一轮：配药区停在C窗口（0），取药区停在3号窗口（4）
    #   第二轮：配药区停在A窗口（1），取药区停在4号窗口（3）
    #   以此类推
       

    time0 = 5.0    # 运行该程序后，停留多少时间（单位：秒），小车开始前进到识别区
    time1 = 15.0     # 小车到达识别区，等待time1（单位：秒）后，走向配药区
    time2 = 2.0     # 小车到达配药区，等待time2（单位：秒）后，走向答题区
    time3 = 10.0     # 小车到达答题区，等待time3（单位：秒）后，走向取药区
    time4 = 2.0     # 小车到达取药区，等待time4（单位：秒）后，走向起点
    time5 = 5.0     # 小车到达起点，等待time5（单位：秒）后，下一轮出发
    
    # time1 = rospy.get_param('~time1',1.0)
    # time2 = rospy.get_param('~time2',1.0)
    # time3 = rospy.get_param('~time3',1.0)
    # time4 = rospy.get_param('~time4',1.0)
    # num = rospy.get_param('~num',2)
    
    msg = Int32() # 定义要发布的int32数据
    
    pub_cmd = rospy.Publisher('cmd_send', Int32, queue_size=10)
    pub_dispense = rospy.Publisher('dispense_window', Int32, queue_size=10)
    pub_pick = rospy.Publisher('pick_up_num', Int32, queue_size=10)
    
    rospy.Subscriber('/cam_send', Int32MultiArray, cam_callback)
    
    
    rospy.init_node('my_publisher', anonymous=True)
    
    rospy.on_shutdown(my_shutdown)
    
    while not rospy.is_shutdown():
        for i in range(num):
        
            rate = rospy.Rate(1.0/time0) # 1/time0 Hz
            rate.sleep()
            
            rospy.loginfo('  Step [%d-1]',i+1)
            rospy.loginfo('    Move to screen ...')
            msg.data = 9
            pub_cmd.publish(msg.data)  # 发布数据

            rate = rospy.Rate(1.0/time1) # 1/time1 Hz
            rate.sleep()
            
            global array_data
            list_points = array_data  
            
            rospy.loginfo('  Step [%d-1]',i+1)
            rospy.loginfo('    Move to dispense_window :%d ...',list_points[0])
            msg.data = list_points[0]
            rate = rospy.Rate(1.0/time2) # 1/time1 Hz
            rate.sleep()
            pub_dispense.publish(msg.data)  # 发布数据
            
            msg.data = 1
            pub_cmd.publish(msg.data)  # 发布数据
            rate = rospy.Rate(1.0/time2) # 1/time2 Hz
            rate.sleep()
            
            rospy.loginfo('  Step [%d-2]',i+1)
            rospy.loginfo('    Move to question area ...')
            msg.data = 3
            pub_cmd.publish(msg.data)  # 发布数据
            rate = rospy.Rate(1.0/time3) # 1/time3 Hz
            rate.sleep()
            
            rospy.loginfo('  Step [%d-3]',i+1)
            rospy.loginfo('    Move to pick_up:%d ...',list_points[1])
            msg.data = list_points[1]
            pub_pick.publish(msg.data)  # 发布数据
            msg.data = 5
            pub_cmd.publish(msg.data)  # 发布数据
            rate = rospy.Rate(1.0/time4) # 1/time4 Hz
            rate.sleep()
            
            rospy.loginfo('  Step [%d-4]',i+1)
            rospy.loginfo('    Move to start point ...')
            msg.data = 7
            pub_cmd.publish(msg.data)  # 发布数据
            rate = rospy.Rate(1.0/time5) # 1/time5 Hz
            rate.sleep()
        

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
