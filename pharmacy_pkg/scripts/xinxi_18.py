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
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
import os
import random
class MoveBaseSquare():

    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)
     
        # Create a list to hold the target quaternions (orientations)
        # 创建一个列表，保存目标的角度数据
        quaternions = list()
         
        # First define the corner orientations as Euler angles
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)
        #euler_angles = (0,pi/2, pi/2,-pi/2, -pi/2, pi/2,-pi/2,0,pi/4)
        euler_angles = (pi/2,pi/2, pi/2,-pi/2, -pi/2, -pi/2,-pi/2,0,-pi,0)
        # Then convert the angles to quaternions
        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
         
        # Create a list to hold the waypoint poses
        # 创建一个列表存储导航点的位置
        waypoints = list()
        waypoints.append(Pose(Point(1.350, 1.990, 0), quaternions[0]))      # //C
        waypoints.append(Pose(Point(0.675, 2.393, 0), quaternions[1]))      #//A
        waypoints.append(Pose(Point(1.350, 2.899, 0), quaternions[2]))      # //B
        waypoints.append(Pose(Point(-0.830, 0.969, 0), quaternions[3]))     # //4
        waypoints.append(Pose(Point(-1.650, 1.232, 0), quaternions[4]))    # //3
        waypoints.append(Pose(Point(-0.895, 1.819, 0), quaternions[5]))    # //2
        waypoints.append(Pose(Point(-1.650, 2.275, 0), quaternions[6]))    # //1
        waypoints.append(Pose(Point(0.010,0.025,0), quaternions[7]))    #起点
        waypoints.append(Pose(Point(-0.704,3.803,0), quaternions[8]))  #答题区   
        waypoints.append(Pose(Point(0.723,0.146,0), quaternions[9]))  #识别板1           
        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人

        self.i = 0
        self.j = 0
        self.count = 9  #状态
        self.windows_ABC = 0
        self.windows_1234 = 2
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=11)
        self.cam_sub=rospy.Subscriber('/cam_send', Int32MultiArray, self.detect_result,queue_size=10)     #订阅检测的药品和窗口数组
        self.ram_result = [0,4]
        rospy.sleep(1)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("Starting navigation...")
        # 初始化一个计数器，记录到达的顶点号
        
        while(not rospy.is_shutdown()):#如果ros系统没有关机的话
            #有限状态机
            if(self.count == 9):#从起点到识别区
                rospy.loginfo("从起点到识别区")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal()  
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[9]     #self.i
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别区。。。。。。。。")
                    self.count = 10
                    rospy.sleep(5)    #给10秒钟识别时间
                    # 在识别区播报
                    temp ='/home/EPRobot/robot_ws/src/pharmacy_pkg/'
                    A="/yuyingwenjian/A.wav"
                    B="/yuyingwenjian/B.wav"
                    C="/yuyingwenjian/C.wav"
                    if self.windows_ABC == 0:
                        os.system("play "+temp+C)
                    elif self.windows_ABC == 1:
                        os.system("play "+temp+A)
                    elif self.windows_ABC == 2:
                        os.system("play "+temp+B)


            elif(self.count == 10):#从起点到配药区
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[self.windows_ABC]   
                if(self.move(goal) == True):
                    rospy.loginfo("到达字母区。。。。。。。。")
                    self.count = 11  
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    rospy.sleep(1)      #在ABC的框里等待3秒钟，后面可以加播报
                    temp ='/home/EPRobot/robot_ws/src/pharmacy_pkg/'
                    A="/yuyingwenjian/A.wav"
                    B="/yuyingwenjian/B.wav"
                    C="/yuyingwenjian/C.wav"
                    if self.windows_ABC == 0:
                        os.system("play "+temp+C)
                    elif self.windows_ABC == 1:
                        os.system("play "+temp+A)
                    elif self.windows_ABC == 2:
                        os.system("play "+temp+B)

    
            elif(self.count == 11):#从配药区到答题区
                rospy.loginfo("从配药区到答题区路上")
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[8]     #self.i
                if(self.move(goal) == True):
                    rospy.loginfo("到达答题区。。。。。。。。")
                    self.count = 12
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    rospy.sleep(2)     #在答题区等待3秒钟，后面可以加答题播报
                    temp = '/home/EPRobot/robot_ws/src/pharmacy_pkg/'
                    YX = "/yuyingwenjian/D.wav"
                    SS = "/yuyingwenjian/E.wav"
                    YGY = "/yuyingwenjian/F.wav"
                    DS = "/yuyingwenjian/G.wav"
                    TY = "/yuyingwenjian/H.wav"
                    yaop2 = [YX,DS,YX,YGY]
                    if self.i < 4:
                        os.system('play '+temp+yaop2[self.i])
                    else:
                        yaop = [YX,YGY,TY,SS]
                        yaop1 = random.choice(yaop)
                        os.system('play '+temp+yaop1)

            elif(self.count == 12):#从答题区到数字区
                rospy.loginfo("从答题区到数字区路上")
                goal = MoveBaseGoal() 
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[self.windows_1234]
                if(self.move(goal) == True):
                    rospy.loginfo("到达数字区。。。。。。。。")                    
                    self.count = 13
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    rospy.sleep(1)     #在数字区等待5秒钟，后面可以加播报送药完成
                    self.i = self.i + 1
                    temp = '/home/EPRobot/robot_ws/src/pharmacy_pkg/'
                    A4 = '/yuyingwenjian/4.wav'
                    A3 = '/yuyingwenjian/3.wav'
                    A2 = '/yuyingwenjian/2.wav'
                    A1 = '/yuyingwenjian/1.wav'
                    if self.windows_1234 == 3:
                        os.system('play '+temp+A4)
                    elif self.windows_1234 == 4:
                        os.system("play "+temp+A3)
                    elif self.windows_1234 == 5:
                        os.system("play "+temp+A2)
                    elif self.windows_1234 == 6:
                        os.system("play "+temp+A1)

            elif(self.count == 13):#从数字区到起点#9
                rospy.loginfo("从数字区到起点")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal()  
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[7]     #起点
                if(self.move(goal) == True):
                    rospy.loginfo("到达起点。。。。。。。。")
                    #发送 msg.data1 = 1
                   # 令  msg.data2 = 0
                    # wait for msg.data2 = 1
                    self.count = 9
                    
                    

    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            # 把目标位置发送给MoveBaseAction的服务器
            self.move_base.send_goal(goal)
            # Allow 1 minute to get there
            # 设定1分钟的时间限制
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
            # If we don't get there in time, abort the goal
            # 如果一分钟之内没有到达，放弃目标
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    return True
            return False        

    def detect_result(self,msg):
        if self.count == 11:   #只有识别的时候才识别，其他时间不接受识别，以防干扰
            self.ram_result = msg.data
            rospy.logwarn("self.ram_result: %s", self.ram_result)
            self.windows_ABC = self.ram_result[0]
            self.windows_1234 = self.ram_result[1]

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

  
if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
