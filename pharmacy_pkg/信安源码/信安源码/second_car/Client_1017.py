#!/usr/bin/env python
# coding: utf-8

import socket
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
import os
import socket
import threading

# 全局变量用于socket 和 ros 通信
global rec_data     # 用于通信传输时接收信息
global enable_work  # 双车行驶开关，1为1号车行驶，2为2号车行驶
global go_to_ready  # 双车起点等待标志，1为1号车去起点等待，2为2号车在起点等待
global go_data      # 用于通信传输时发送信息
global ram_result  # 用于socket传输二维码数据的列表

class MoveBaseSquare():
    def __init__(self):
        # rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # 创建一个列表，保存目标的角度数据
        quaternions = list()
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)
        euler_angles = (pi / 2, pi / 2, pi / 2, -pi / 2, -pi / 2, -pi / 2, -pi / 2, 0, -pi, 0, 0)
        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # 创建一个列表存储导航点的位置
        waypoints = list()
        waypoints.append(Pose(Point(1.371, 1.995, 0), quaternions[0]))  # //C
        waypoints.append(Pose(Point(0.650, 2.578, 0), quaternions[1]))  # //A
        waypoints.append(Pose(Point(1.430, 2.990, 0), quaternions[2]))  # //B
        waypoints.append(Pose(Point(-0.762, 0.833, 0), quaternions[3]))  # //4
        waypoints.append(Pose(Point(-1.584, 1.451, 0), quaternions[4]))  # //3
        waypoints.append(Pose(Point(-0.820, 1.850, 0), quaternions[5]))  # //2
        waypoints.append(Pose(Point(-1.584, 2.340, 0), quaternions[6]))  # //1
        waypoints.append(Pose(Point(0.05, 0, 0), quaternions[7]))           # 起点
        waypoints.append(Pose(Point(-0.040, 3.899, 0), quaternions[8]))  # 识别板2
        waypoints.append(Pose(Point(0.75, 0.183, 0), quaternions[9]))    # 识别板1
        waypoints.append(Pose(Point(-0.85, 0.1, 0), quaternions[10]))           # 起点后方的候车区

        # 发布TWist消息控制机器人
        global go_data      # 用于通信传输时发送信息
        global go_to_ready  # 双车起点等待标志，1为1号车去起点等待，2为2号车在起点等待
        go_to_ready = 0
        global ram_result  # 用于socket传输二维码数据的列表
        self.count = 9           # 状态
        global enable_work       # 双车行驶开关，1为1号车行驶，2为2号车行驶
        enable_work = 1
        self.windows_ABC = 0
        self.windows_A = 1       # 字母区目标点是否包含A点，包含为1，不包含为0
        self.windows_B = 1       # 字母区目标点是否包含B点，包含为1，不包含为0
        self.windows_C = 1       # 字母区目标点是否包含C点，包含为1，不包含为0
        self.windows_1234 = 3    # 数字区的窗口数
        self.windows_count = 3   # 样本的数量
        self.windows_error = 0  # 错误的窗口

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.cam_sub = rospy.Subscriber('/cam_return', Int32MultiArray, self.detect_result, queue_size=10)  # 订阅检测的药品和窗口数组
        self.ram_result = [1, 1, 1, 3, 3, 0]  # 初始化二维码数据

        rospy.sleep(1)

        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("Starting navigation...")

        file = open('md5.txt',mode='w')
        file.write("this for test:测试\n")

        while not rospy.is_shutdown():  # 如果ros系统没有关机的话

            # 1号车行驶时，2号车停留在等候区，等待1号车行驶到识别版2时发来的消息，之后行驶到起点等待，并恢复状态到初始值，同时接收消息等待出发
            if enable_work == 1 and go_to_ready == 2:  # 1号车前往起点等待
                rospy.loginfo("前往起点等待......")
                self.windows_error = 0
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[7]  # 导航到起点
                if self.move(goal) == True:
                    rospy.loginfo("到达起点!")
                    self.count = 9  # 恢复到初始状态
                    go_to_ready = 0

            if enable_work == 2:  # 2号车可以行驶
                rospy.loginfo("2号车开始行驶o_o")

                # 有限状态机
                if self.count == 9:  # 从起点到识别区
                    rospy.loginfo("从起点到识别区")
                    # 初始化goal为MoveBaseGoal类型
                    goal = MoveBaseGoal()
                    # 使用map的frame定义goal的frame id
                    goal.target_pose.header.frame_id = 'map'
                    # 设置时间戳
                    goal.target_pose.header.stamp = rospy.Time.now()
                    # 导航到识别版1
                    goal.target_pose.pose = waypoints[9]
                    if self.move(goal) == True:
                        rospy.loginfo("到达识别区!")
                    
                        self.count = 10  # 进入下一个状态
                        rospy.sleep(5)  # 给5秒钟识别时间

                elif self.count == 10:  # 从识别版1到字母区
                    # 播报错误信息
                    if self.windows_error == 1:
                        os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/Error1.wav')
                    elif self.windows_error == 2:
                        os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/Error2.wav')
                    elif self.windows_error == 3:
                        os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/Error3.wav')
                    elif self.windows_error == 4:
                        os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/Error4.wav')

                    rospy.loginfo('前往字母区......')
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    
                    if self.windows_C == 1:  # 是否去c点
                        goal.target_pose.pose = waypoints[0]  # 导航到C点
                        if self.move(goal) == True:
                            rospy.loginfo("取到C窗口中的")
                            self.count = 11                # 进入下一个状态
                            self.clear_costmaps_service()  # 清除产生的偏移代价地图
                            # rospy.sleep(0.5)                 # 停留1s进行取药
                            if (self.windows_A == 0) and (self.windows_B == 0):  # 如果仅取C
                                if self.windows_1234 == 3:     # 4(激素检验窗口)
                                    rospy.loginfo("血浆样本")    # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangC.wav')
                                    rospy.logwarn("C的md5值为: 0d61f8370cad1d412f80b84d143e1257")
                                    file.write("C的md5值为: 0d61f8370cad1d412f80b84d143e1257 \n")
                                if self.windows_1234 == 2:     # 3(免疫检测窗口)
                                    rospy.loginfo("组织样本")    # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiC.wav')
                                    rospy.logwarn("C的md5值为: 0d61f8370cad1d412f80b84d143e1257")
                                    file.write("C的md5值为: 0d61f8370cad1d412f80b84d143e1257 \n")
                                if self.windows_1234 == 1:     # 2(体液窗口)
                                    rospy.loginfo("唾液样本")    # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeC.wav')
                                    rospy.logwarn("C的md5值为: 0d61f8370cad1d412f80b84d143e1257")
                                    file.write("C的md5值为: 0d61f8370cad1d412f80b84d143e1257 \n")
                                if self.windows_1234 == 0:      # 1(血常规窗口)
                                    rospy.loginfo("静脉血样本")   # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueC.wav')
                                    rospy.logwarn("C的md5值为: 0d61f8370cad1d412f80b84d143e1257")
                                    file.write("C的md5值为: 0d61f8370cad1d412f80b84d143e1257 \n")

                    if self.windows_A == 1:  # 是否去A点
                        goal.target_pose.pose = waypoints[1]  # 导航到A点
                        if self.move(goal) == True:
                            rospy.loginfo("取到A窗口中的")
                            self.count = 11  # 进入下一个状态
                            self.clear_costmaps_service()  # 清除产生的偏移代价地图
                            # rospy.sleep(0.5)  # 停留1s进行取药
                            if (self.windows_C == 0) and (self.windows_B == 0):  # 如果仅取A
                                if self.windows_1234 == 3:  # 4(激素检验窗口)
                                    rospy.loginfo("血浆样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangA.wav')
                                    rospy.logwarn("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29")
                                    file.write("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29 \n")
                                if self.windows_1234 == 2:  # 3(免疫检测窗口)
                                    rospy.loginfo("组织样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiA.wav')
                                    rospy.logwarn("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29")
                                    file.write("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29 \n")
                                if self.windows_1234 == 1:  # 2(体液窗口)
                                    rospy.loginfo("唾液样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeA.wav')
                                    rospy.logwarn("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29")
                                    file.write("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29 \n")
                                if self.windows_1234 == 0:  # 1(血常规窗口)
                                    rospy.loginfo("静脉血样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueA.wav')
                                    rospy.logwarn("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29")
                                    file.write("A的md5值为: 7fc56270e7a70fa81a5935b72eacbe29 \n")
                            elif (self.windows_C == 1) and (self.windows_B == 0):  # 如果取到AC
                                if self.windows_1234 == 3:  # 4(激素检验窗口)
                                    rospy.loginfo("血浆样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangAC.wav')
                                    rospy.logwarn("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc")
                                    file.write("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc \n")
                                if self.windows_1234 == 2:  # 3(免疫检测窗口)
                                    rospy.loginfo("组织样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiAC.wav')
                                    rospy.logwarn("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc")
                                    file.write("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc \n")
                                if self.windows_1234 == 1:  # 2(体液窗口)
                                    rospy.loginfo("唾液样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeAC.wav')
                                    rospy.logwarn("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc")
                                    file.write("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc \n")
                                if self.windows_1234 == 0:  # 1(血常规窗口)
                                    rospy.loginfo("静脉血样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueAC.wav')
                                    rospy.logwarn("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc")
                                    file.write("AC的md5值为: 4144e097d2fa7a491cec2a7a4322f2bc \n")
                            self.clear_costmaps_service()
                    if self.windows_B == 1:  # 是否去B
                        goal.target_pose.pose = waypoints[2]  # 导航到B点
                        if self.move(goal) == True:
                            rospy.loginfo("取到b窗口中的")
                            self.count = 11                # 进入下一个状态
                            self.clear_costmaps_service()  # 清除产生的偏移代价地图
                            # rospy.sleep(0.5)                 # 停留1s进行取药
                            if (self.windows_C == 1) and (self.windows_A == 1):  # 如果取ABC
                                if self.windows_1234 == 3:  # 4(激素检验窗口)
                                    rospy.loginfo("血浆样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangABC.wav')
                                    rospy.logwarn("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932")
                                    file.write("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932 \n")
                                if self.windows_1234 == 2:  # 3(免疫检测窗口)
                                    rospy.loginfo("组织样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiABC.wav')
                                    rospy.logwarn("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932")
                                    file.write("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932 \n")
                                if self.windows_1234 == 1:  # 2(体液窗口)
                                    rospy.loginfo("唾液样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeABC.wav')
                                    rospy.logwarn("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932")
                                    file.write("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932 \n")
                                if self.windows_1234 == 0:  # 1(血常规窗口)
                                    rospy.loginfo("静脉血样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueABC.wav')
                                    rospy.logwarn("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932")
                                    file.write("ABC的md5值为: 902fbdd2b1df0c4f70b4a5d23525e932 \n")
                            elif (self.windows_C == 0) and (self.windows_A == 1):  # 如果取到AB
                                if self.windows_1234 == 3:  # 4(激素检验窗口)
                                    rospy.loginfo("血浆样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangAB.wav')
                                    rospy.logwarn("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9")
                                    file.write("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9 \n")
                                if self.windows_1234 == 2:  # 3(免疫检测窗口)
                                    rospy.loginfo("组织样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiAB.wav')
                                    rospy.logwarn("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9")
                                    file.write("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9 \n")
                                if self.windows_1234 == 1:  # 2(体液窗口)
                                    rospy.loginfo("唾液样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeAB.wav')
                                    rospy.logwarn("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9")
                                    file.write("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9 \n")
                                if self.windows_1234 == 0:  # 1(血常规窗口)
                                    rospy.loginfo("静脉血样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueAB.wav')
                                    rospy.logwarn("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9")
                                    file.write("AB的md5值为: b86fc6b051f63d73de262d4c34e3a0a9 \n")
                            elif (self.windows_C == 1) and (self.windows_A == 0):  # 如果取到CB
                                if self.windows_1234 == 3:  # 4(激素检验窗口)
                                    rospy.loginfo("血浆样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangBC.wav')
                                    rospy.logwarn("BC的md5值为: f85b7b377112c272bc87f3e73f10508d")
                                    file.write("BC的md5值为: f85b7b377112c272bc87f3e73f10508d \n")
                                if self.windows_1234 == 2:  # 3(免疫检测窗口)
                                    rospy.loginfo("组织样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiBC.wav')
                                    rospy.logwarn("BC的md5值为: f85b7b377112c272bc87f3e73f10508d")
                                    file.write("BC的md5值为: f85b7b377112c272bc87f3e73f10508d \n")
                                if self.windows_1234 == 1:  # 2(体液窗口)
                                    rospy.loginfo("唾液样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeBC.wav')
                                    rospy.logwarn("BC的md5值为: f85b7b377112c272bc87f3e73f10508d")
                                    file.write("BC的md5值为: f85b7b377112c272bc87f3e73f10508d \n")
                                if self.windows_1234 == 0:  # 1(血常规窗口)
                                    rospy.loginfo("静脉血样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueBC.wav')
                                    rospy.logwarn("BC的md5值为: f85b7b377112c272bc87f3e73f10508d")
                                    file.write("BC的md5值为: f85b7b377112c272bc87f3e73f10508d \n")
                            elif (self.windows_C == 0) and (self.windows_A == 0):  # 如果仅取到B
                                if self.windows_1234 == 3:  # 4(激素检验窗口)
                                    rospy.loginfo("血浆样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangB.wav')
                                    rospy.logwarn("B的md5值为: 9d5ed678fe57bcca610140957afab571")
                                    file.write("B的md5值为: 9d5ed678fe57bcca610140957afab571 \n")
                                if self.windows_1234 == 2:  # 3(免疫检测窗口)
                                    rospy.loginfo("组织样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiB.wav')
                                    rospy.logwarn("B的md5值为: 9d5ed678fe57bcca610140957afab571")
                                    file.write("B的md5值为: 9d5ed678fe57bcca610140957afab571 \n")
                                if self.windows_1234 == 1:  # 2(体液窗口)
                                    rospy.loginfo("唾液样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeB.wav')
                                    rospy.logwarn("B的md5值为: 9d5ed678fe57bcca610140957afab571")
                                    file.write("B的md5值为: 9d5ed678fe57bcca610140957afab571 \n")
                                if self.windows_1234 == 0:  # 1(血常规窗口)
                                    rospy.loginfo("静脉血样本")  # 进行播报
                                    os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueB.wav')
                                    rospy.logwarn("B的md5值为: 9d5ed678fe57bcca610140957afab571")
                                    file.write("B的md5值为: 9d5ed678fe57bcca610140957afab571 \n")

                    self.count = 11     # 防止ABC都为零，使其也能进入下一个状态

                elif self.count == 11:  # 从字母区前往数字区
                    rospy.loginfo("前往识别版2....")
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose = waypoints[8]  # 导航到识别版二
                    if self.move(goal) == True:
                        rospy.loginfo("到达识别板2!")
                        os.system("play " + '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/spare.wav')
                        # 到达识别版2后应发送消息，让1号车走到起点位置
                        go_data = 2  # 发送2，表示1号车该前往起点准备
                        client_socket.send(str(go_data).encode('utf-8'))

                        self.count = 12                # 进入下一个状态
                        self.clear_costmaps_service()  # 清除产生的偏移代价地图

                elif self.count == 12:  # 从识别版2到数字区
                    rospy.loginfo("前往数字区......")
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    print('windows:',self.windows_1234)
                    goal.target_pose.pose = waypoints[(6 - self.windows_1234)]  # 导航到数字区的窗口
                    if self.move(goal) == True:
                        rospy.loginfo("到达数字区!")
                        self.count = 13                # 进入下一个状态
                        self.clear_costmaps_service()  # 清除产生的偏移代价地图
                        rospy.sleep(0.5)                 # 等待1s进行送药
                        if self.windows_1234 == 3:         # 窗口4
                            rospy.loginfo("到达激素检验窗口")
                            if self.windows_count == 3:    # 样本数为3
                                rospy.loginfo("样本数为3")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu3.wav')
                            elif self.windows_count == 2:  # 样本数为2
                                rospy.loginfo("样本数为2")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu2.wav')
                            elif self.windows_count == 1:  # 样本数为1
                                rospy.loginfo("样本数为1")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu1.wav')
                            # os.system('play '+temp+A4)
                        if self.windows_1234 == 2:         # 窗口3
                            rospy.loginfo("到达免疫检验窗口")
                            if self.windows_count == 3:    # 样本数为3
                                rospy.loginfo("样本数为3")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi3.wav')
                            elif self.windows_count == 2:  # 样本数为2
                                rospy.loginfo("样本数为2")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi2.wav')
                            elif self.windows_count == 1:  # 样本数为1
                                rospy.loginfo("样本数为1")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi1.wav')
                            # os.system("play "+temp+A3)
                        if self.windows_1234 == 1:         # 窗口2
                            rospy.loginfo("到达体液检验窗口")
                            if self.windows_count == 3:    # 样本数为3
                                rospy.loginfo("样本数为3")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye3.wav')
                            elif self.windows_count == 2:  # 样本数为2
                                rospy.loginfo("样本数为2")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye2.wav')
                            elif self.windows_count == 1:  # 样本数为1
                                rospy.loginfo("样本数为1")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye1.wav')
                            # os.system("play "+temp+A2)
                        if self.windows_1234 == 0:         # 窗口1
                            rospy.loginfo("到达血常规检验窗口")
                            if self.windows_count == 3:    # 样本数为3
                                rospy.loginfo("样本数为3")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui3.wav')
                            elif self.windows_count == 2:  # 样本数为2
                                rospy.loginfo("样本数为2")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui2.wav')
                            elif self.windows_count == 1:  # 样本数为1
                                rospy.loginfo("样本数为1")
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui1.wav')

                elif self.count == 13:  # 从数字区到候车区
                    rospy.loginfo("前往候车区")
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose = waypoints[10]  # 导航到候车区
                    if self.move(goal) == True:
                        # 在候车区发送消息让1号车出发
                        rospy.loginfo("到达候车区!")
                        enable_work = 1    # 2号车跑完一圈后，关闭小车行驶开关，轮到1号车行驶
                        go_data = 0        # 发送0，表示1号车该出发了
                        client_socket.send(str(go_data).encode('utf-8'))

    def move(self, goal):
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base.send_goal(goal)
        # 设定1分钟的时间限制
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(30))
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

    def detect_result(self, msg):
        if self.count == 10:   # 只有在识别区的时候才识别，其他时间不接受识别，以防干扰
            # 1号车数据处理
            self.ram_result = msg.data
            print(msg.data)
            # 数组应传入数据：0.是否去c，1.是否去a，2.是否去b，3.样品数 4.self.windows_1234
            self.windows_C = self.ram_result[0]      # 0=不去，1=去
            self.windows_A = self.ram_result[1]
            self.windows_B = self.ram_result[2]
            self.windows_count = self.ram_result[3]  # 样品数
            self.windows_1234 = self.ram_result[4]   # 现在要去的窗口,3=窗口4，2=窗口3，1=窗口2，0=窗口1
            self.windows_error = self.ram_result[5]  # 错误窗口

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


def socket_client():
    global enable_work
    global go_to_ready
    global rec_data
    global ram_result
    global client_socket
    global server_socket



    host = '192.168.166.248'   # 设置地址
    port = 7000              # 设置端口

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 以AF_INET为协议族，SOCK_STREAM为类型创建socket
    client_socket.connect((host, port))  # 连接上文的地址和端口

    while not rospy.is_shutdown():
        # 接收1号车传来的消息
        # 收到的消息为rec_data,内容为：0为1号车出发，1为2号车出发，2为1号车前往起点，3为2号车前往起点
        rec_data = client_socket.recv(1024).decode('utf-8')
        if rec_data != '' and rec_data != []:
            print('Received data: {}'.format(rec_data))
            if int(rec_data) == 1:
                enable_work = 2
                rospy.loginfo("2号车收到出发指令")
            elif int(rec_data) == 3:
                go_to_ready = 2
                rospy.loginfo("2号车收到前往起点指令")


def go_main():
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':
    # 创建ROS节点必须在主线程
    rospy.init_node('nav_pharmacy_client', anonymous=False)

    # 启动socket线程
    t1 = threading.Thread(target=socket_client)
    t1.daemon = True  # 设置为守护线程
    t1.start()

    # 启动ROS导航主循环
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
