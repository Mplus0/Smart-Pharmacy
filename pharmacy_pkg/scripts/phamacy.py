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
import os

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
        waypoints.append(Pose(Point(1.480, 1.990, 0), quaternions[0]))      # //C
        waypoints.append(Pose(Point(0.550, 2.320, 0), quaternions[1]))      #//A
        waypoints.append(Pose(Point(1.482, 2.790, 0), quaternions[2]))      # //B
        waypoints.append(Pose(Point(-0.875, 1.105, 0), quaternions[3]))     # //4
        waypoints.append(Pose(Point(-1.650, 1.232, 0), quaternions[4]))    # //3
        waypoints.append(Pose(Point(-0.875, 1.719, 0), quaternions[5]))    # //2
        waypoints.append(Pose(Point(-1.650, 2.275, 0), quaternions[6]))    # //1
        waypoints.append(Pose(Point(0.010,0.025,0), quaternions[7]))    #起点
        waypoints.append(Pose(Point(-0.480,3.700,0), quaternions[8]))  #答题区   
        waypoints.append(Pose(Point(0.510,0.025,0), quaternions[9]))  #识别板1           
        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人

        self.i = 0
        self.j = 0
        self.k = 0
        self.count = 0  #状态
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        
        #接受时间话题
        self.dispense_sub=rospy.Subscriber('/dispense_window', Int32, self.position,queue_size=10)
        self.pick_up_sub=rospy.Subscriber('/pick_up_num', Int32, self.position1,queue_size=10)
        self.cmd_sub=rospy.Subscriber('/cmd_send', Int32, self.cmd_get,queue_size=10)
        self.shibie_sub=rospy.Subscriber('/shibie_send', Int32, self.cam_callback,queue_size=10)
        # Subscribe to the move_base action server
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation...")
        # Initialize a counter to track waypoints
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
                # Start the robot moving toward the goal
                # 机器人移动
                if(self.move(goal) == True):
                    rospy.sleep(3)
                    rospy.loginfo("到达识别区。。。。。。。。")
                    self.count = 10
                    
            elif(self.count == 1):#从起点到配药区
                rospy.loginfo("从识别区到配药区")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal()  
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[self.i]     #self.i
                # Start the robot moving toward the goal
                # 机器人移动
                if(self.move(goal) == True):
                    rospy.sleep(3)
                    rospy.loginfo("到达配药区。。。。。。。。")
                    # temp = '/home/zlq/catkin_ws/src/robot_voice'
                    # A = '/yuyingwenjian/A.wav'
                    # B = '/yuyingwenjian/B.wav'
                    # C = '/yuyingwenjian/C.wav'
                    if self.i == 0:
                        os.system('rhythmBox '+temp+C)
                    elif self.i == 1:
                        os.system("rhythmBox "+temp+A)
                    elif self.i == 2:
                        os.system("rhythmBox "+temp+B)
                    self.count = 2


            elif(self.count == 3):#从配药区到屏幕识别点2
                rospy.loginfo("从配药区到屏幕识别点2")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal() 
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[8]
                # Start the robot moving toward the goal
                # 机器人移动
                if(self.move(goal) == True):
                    rospy.sleep(3)
                    rospy.loginfo("到达屏幕识别点2。。。。。。。。")
                    # temp = '/home/zlq/catkin_ws/src/robot_voice' 
                    # D = '/yuyingwenjian/D.wav'
                    # E = '/yuyingwenjian/E.wav'
                    # F = '/yuyingwenjian/F.wav'
                    # G = '/yuyingwenjian/G.wav'
                    # H = '/yuyingwenjian/H.wav'
                    if self.k == 1:
                        os.system('rhythmBox '+temp+D)
                    elif self.k == 2:
                        os.system("rhythmBox "+temp+E)
                    elif self.k == 3:
                        os.system("rhythmBox "+temp+F)
                    elif self.k == 4:
                        os.system("rhythmBox "+temp+G)
                    elif self.k == 5:
                        os.system("rhythmBox "+temp+H)
                    self.count = 4
                
            elif(self.count == 5):#从屏幕识别点到取药区
                rospy.loginfo("从屏幕识别点2到取药区")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal() 
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[self.j]     #self.j
                # Start the robot moving toward the goal
                # 机器人移动
                if(self.move(goal) == True):
                    rospy.sleep(3)
                    rospy.loginfo("到达取药区。。。。。。。。")
                    # temp = '/home/zlq/catkin_ws/src/robot_voice'
                    # A4 = '/yuyingwenjian/4.wav'
                    # A3 = '/yuyingwenjian/3.wav'
                    # A2 = '/yuyingwenjian/2.wav'
                    # A1 = '/yuyingwenjian/1.wav'
                    if self.j == 3:
                        os.system('rhythmBox '+temp+A4)
                    elif self.j == 4:
                        os.system("rhythmBox "+temp+A3)
                    elif self.j == 5:
                        os.system("rhythmBox "+temp+A2)
                    elif self.j == 6:
                        os.system("rhythmBox "+temp+A1)
                    self.count = 6

            elif(self.count == 7):#从取药区到起点
                rospy.loginfo("从取药区到起点")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal() 
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[7]
                # Start the robot moving toward the goal
                # 机器人移动
                if(self.move(goal) == True):
                    rospy.sleep(3)
                    rospy.loginfo("到达起点。。。。。。。。")
                    self.count = 8

         
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
    def cmd_get(self,msg):
        if msg.data > 0:
            self.count = msg.data
            rospy.loginfo("Get cmd:%d",self.count)
            
    def position(self,msg):
        self.i = msg.data
        rospy.loginfo("Get dispense_window:%d",self.i)

    def position1(self,msg1):
        self.j = msg1.data
        rospy.loginfo("Get pick_up_num:%d",self.j)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def cam_callback(self,msg):
    # 接收到消息时的回调函数
        self.k = msg
        rospy.loginfo("Received array: %s", self.k)

  
if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
