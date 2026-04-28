#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from math import pi
import time

# 假设 QR 服务和 picture 服务的消息类型
from your_package.srv import QRRecognition, PictureRecognition

class MainTask:
    def __init__(self):
        rospy.init_node('main_task')
        rospy.on_shutdown(self.shutdown)

        # ---------- 航点定义（与 pharmacy.py 完全一致）----------
        # 索引: 0-C,1-A,2-B,3-4号,4-3号,5-2号,6-1号,7-起点,8-识别板二,9-识别板一
        raw_points = [
            [ 1.468,  1.972,  pi/2],  # 0: C
            [ 0.676,  2.543,  pi/2],  # 1: A
            [ 1.515,  2.757,  pi/2],  # 2: B
            [-0.895,  1.017, -pi/2],  # 3: 4号
            [-1.707,  1.468, -pi/2],  # 4: 3号
            [-0.821,  1.965, -pi/2],  # 5: 2号
            [-1.732,  2.414, -pi/2],  # 6: 1号
            [ 0.0,    0.0,    0.0 ],  # 7: 起点
            [-0.122,  3.873, -pi  ],  # 8: 识别板二
            [ 0.746,  0.144,  0.0  ]   # 9: 识别板一
        ]
        self.waypoints = []
        for p in raw_points:
            x, y, theta = p
            q = quaternion_from_euler(0, 0, theta)
            self.waypoints.append(Pose(Point(x, y, 0), Quaternion(*q)))

        # ---------- 导航客户端 ----------
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 服务器...")
        self.move_base.wait_for_server()
        rospy.loginfo("已连接 move_base")

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ---------- 播报话题 ----------
        self.speech_pub = rospy.Publisher('/speech_text', String, queue_size=10)

        # ---------- 服务客户端（等待服务上线）----------
        rospy.loginfo("等待 QR 识别服务...")
        rospy.wait_for_service('/qr_recognition')
        self.qr_service = rospy.ServiceProxy('/qr_recognition', QRRecognition)

        rospy.loginfo("等待 picture 识别服务...")
        rospy.wait_for_service('/picture_recognition')
        self.pic_service = rospy.ServiceProxy('/picture_recognition', PictureRecognition)

        # ---------- 样本到窗口索引的映射 ----------
        # 体检区：C->0, A->1, B->2
        self.sample_to_pickup_idx = {'A': 1, 'B': 2, 'C': 0}
        # 化验区：假设 A→1号窗口(6), B→2号窗口(5), C→3号窗口(4)
        self.sample_to_lab_idx = {'A': 6, 'B': 5, 'C': 4}
        # 窗口名称（用于播报）
        self.pickup_names = {'A': 'A窗口', 'B': 'B窗口', 'C': 'C窗口'}
        self.lab_names = {'A': '1号窗口', 'B': '2号窗口', 'C': '3号窗口'}

        # ---------- 轮次对应的二维码区域 ----------
        self.region_map = {
            1: 'left_top',
            2: 'right_top',
            3: 'left_bottom',
            4: 'right_bottom'
        }

        # 运行主流程
        self.run()

    def speak(self, text):
        """触发语音播报"""
        rospy.loginfo("【播报】%s", text)
        self.speech_pub.publish(String(text))

    def navigate_to(self, wp_index, name, timeout=60.0):
        """
        导航到指定航点，返回是否成功
        *** 这就是小车到达某个位置的代码入口 ***
        调用 self.navigate_to(索引, 名称) 即可让小车前往该点
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.waypoints[wp_index]

        rospy.loginfo("导航至: %s (索引 %d)", name, wp_index)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))

        if not finished:
            self.move_base.cancel_goal()
            rospy.logerr("%s 导航超时", name)
            return False

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("✓ 到达 %s", name)
            return True
        else:
            rospy.logerr("%s 导航失败，状态码: %d", name, state)
            return False

    def stay(self, seconds, reason=""):
        """原地等待"""
        rospy.loginfo("等待 %.1f 秒: %s", seconds, reason)
        rospy.sleep(seconds)

    def call_qr_recognition(self, region):
        """
        调用 QR.py 识别服务，返回 (windows, samples) 两个列表
        windows: 如 ['A','C']
        samples: 如 [1,3]
        """
        try:
            resp = self.qr_service(region)
            return resp.windows, resp.samples
        except rospy.ServiceException as e:
            rospy.logerr("QR 服务调用失败: %s", e)
            return [], []

    def call_picture_recognition(self):
        """调用 picture.py 识别服务，返回等待时间（秒）"""
        try:
            resp = self.pic_service()
            return resp.wait_time
        except rospy.ServiceException as e:
            rospy.logerr("Picture 服务调用失败: %s", e)
            return 5  # 默认等待5秒

    def run(self):
        rospy.loginfo("========== 任务开始 ==========")

        # 确保在起点
        if not self.navigate_to(7, "起点"):
            rospy.logerr("起点到达失败，任务终止")
            return
        self.stay(1.0)

        for round_num in range(1, 5):
            rospy.loginfo("========== 第 %d 轮 ==========", round_num)
            region = self.region_map[round_num]

            # ---------- 1. 去识别板一 ----------
            if not self.navigate_to(9, "识别板一"):
                rospy.logerr("无法到达识别板一，跳过本轮")
                continue

            # 调用 QR.py 识别
            windows, samples = self.call_qr_recognition(region)
            if not windows:
                rospy.logwarn("未识别到有效样本，返回起点")
                self.navigate_to(7, "起点")
                continue

            # 构建任务列表，按 C->A->B 排序
            tasks = list(zip(windows, samples))
            order = {'C': 0, 'A': 1, 'B': 2}
            tasks.sort(key=lambda x: order.get(x[0], 99))

            rospy.loginfo("本轮任务: %s", tasks)

            # ---------- 2. 体检区取样本 ----------
            for win, sam in tasks:
                idx = self.sample_to_pickup_idx[win]
                name = self.pickup_names[win]
                if not self.navigate_to(idx, name):
                    rospy.logerr("无法到达 %s", name)
                    continue
                self.stay(2.0, reason=f"取{win}窗口样本")
                # 播报1
                self.speak(f"取到{win}窗口{sam}号样本")

            # ---------- 3. 去识别板二 ----------
            if not self.navigate_to(8, "识别板二"):
                rospy.logerr("无法到达识别板二")
                continue

            wait_sec = self.call_picture_recognition()
            if wait_sec > 0:
                # 播报2：忙碌等待
                self.speak(f"化验区忙碌，请等待{wait_sec}秒")
                self.stay(wait_sec, reason="等待化验区空闲")
            else:
                # 播报2：空闲通过
                self.speak("化验区空闲，快速通过")
                # 立即通过，不等待

            # ---------- 4. 化验区送样本 ----------
            for win, sam in tasks:
                idx = self.sample_to_lab_idx[win]
                name = self.lab_names[win]
                if not self.navigate_to(idx, name):
                    rospy.logerr("无法到达 %s", name)
                    continue
                self.stay(2.0, reason=f"送{win}样本到{name}")
                # 播报3
                self.speak(f"到达{name}，样本数为1")

            # ---------- 5. 返回起点 ----------
            self.navigate_to(7, "起点")
            self.stay(1.0)

        rospy.loginfo("========== 全部任务完成 ==========")

    def shutdown(self):
        rospy.loginfo("节点关闭，停止小车")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)
        self.cmd_vel_pub.publish(Twist())


if __name__ == '__main__':
    try:
        MainTask()
    except rospy.ROSInterruptException:
        pass