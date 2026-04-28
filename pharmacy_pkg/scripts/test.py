#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray,String
from std_srvs.srv import Empty
import os
windows_1234 =0;
def init():
  windows_drug = rospy.Subscriber('/chatter',String, detect_drug,queue_size=10)
  rospy.init_node('nav_pharmacy', anonymous=False)
  rospy.spin()
def detect_drug(msg):   #只有识别的时候才识别，其他时间不接受识别，以防干扰
        windows_1234 = msg.data
        print(windows_1234)
        number = 2
        temp = '/home/EPRobot/robot_ws/src/pharmacy_pkg/'
        A4 = '/yuyingwenjian/D.wav'
        A3 = '/yuyingwenjian/E.wav'
        A2 = '/yuyingwenjian/F.wav'
        A1 = '/yuyingwenjian/G.wav'
        if(number):
          number -= 1
          if windows_1234 == '1':
              os.system('play '+temp+A4)
          elif windows_1234 == '2':
              os.system("play "+temp+A3)
          elif windows_1234 == '3':
              os.system("play "+temp+A2)
          elif windows_1234 == '4':
              os.system("play "+temp+A1)
        
if __name__ == '__main__':
  init()
  