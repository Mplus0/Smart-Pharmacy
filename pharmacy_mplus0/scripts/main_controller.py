#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
main_controller.py

主导航与任务状态机节点：
1. 起点 -> 识别板一
2. 根据识别板一结果前往 A/B/C 取样窗口
3. 前往识别板二，读取化验区等待状态
4. 前往 1/2/3/4 化验窗口
5. 通过 /referee_task、/referee_cv1、/referee_cv2 向裁判通信节点同步任务状态


备注：新增防卡顿：将状态机数据发布节点，记录所有状态机的变化,发布于 /nav_state 话题，类型为 Int32，发布时机为每次状态改变时。
     新增等待识别结果的机制：在前往识别板一和识别板二的状态中，增加一个循环等待机制，直到收到对应的识别结果消息才继续执行后续逻辑。通过设置标志位（如 board1_received 和 board2_received）来控制循环退出条件。
    新增超时保护
    新增取样窗口完成标志：在前往取样窗口的状态中，增加标志位（如 pickup_C_done、pickup_A_done、pickup_B_done）来记录每个窗口的完成状态。在导航失败重试时，检查这些标志位以决定是否需要重新前往已经完成的窗口，从而避免重复前往同一窗口导致的卡顿问题。
    新增回到原点后重新前往识别板一的逻辑：在送样完成后，先导航回原点，然后再前往识别板一，确保每轮任务都从起点开始，避免直接从化验窗口前往识别板一可能导致的导航问题。
    新增优化CV1以及CV2发布机制：在发布 CV1 和 CV2 的函数中，增加重复发布的逻辑（如发布三次），以降低 ROS 订阅漏收的概率，确保裁判系统能够稳定接收到这些关键状态信息。
    发布时机也有改变：CV1 的发布时机改为在识别板二结果接收后立即发布，而不是在前往化验窗口时发布；CV2 的发布时机改为在识别板一结果接收后立即发布，而不是在前往取样窗口时发布。这些调整可以让裁判系统更及时地获取状态信息，提升整体的响应速度和稳定性。
"""

import os
import random
import socket
import json
import threading

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String, Int32, Int32MultiArray, Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import subprocess
import time

from pharmacy_mplus0.config import COMMON, get_car_config, get_car_id
from pharmacy_mplus0.task_logic import select_board1_from_all_text

CAR_ID = get_car_id(default=1)
CAR_CONFIG = get_car_config(CAR_ID)
STATES = COMMON["states"]
TOPICS = COMMON["topics"]
NAV_CFG = COMMON["nav"]
BOARD1_CFG = COMMON["board1"]

AUDIO_DIR = COMMON["paths"]["audio_dir"]

# 集中配置：状态、化验窗口映射、窗口日志名称
STATE_WAIT_TURN = STATES["WAIT_TURN"]
STATE_GO_TO_BOARD1 = STATES["GO_TO_BOARD1"]
STATE_BOARD1_RECOGNIZING = STATES["BOARD1_RECOGNIZING"]
STATE_GO_TO_PICKUP_WINDOWS = STATES["GO_TO_PICKUP_WINDOWS"]
STATE_GO_TO_BOARD2 = STATES["GO_TO_BOARD2"]
STATE_BOARD2_RECOGNIZING = STATES["BOARD2_RECOGNIZING"]
STATE_GO_TO_LAB_WINDOW = STATES["GO_TO_LAB_WINDOW"]
STATE_GO_BACK_HOME = STATES["GO_BACK_HOME"]

LAB_INFO = COMMON["lab_info"]
WINDOW_LOG_NAME = COMMON["window_log_name"]


class MoveBaseSquare(object):
    """智慧药房主导航状态机。"""

    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.waypoints = self.build_waypoints()
        self.init_runtime_state()
        self.init_ros_interfaces()
        self.init_move_base_client()

        rospy.loginfo("准备完毕，开始导航！")
        self.run_state_machine()

    def build_waypoints(self):
        """从集中配置创建导航点列表，导航点顺序保持旧代码不变。"""
        quaternions = []
        for angle in NAV_CFG["euler_angles"]:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            quaternions.append(Quaternion(*q_angle))

        def pose_from_tuple(item):
            x, y, q_index = item
            return Pose(Point(float(x), float(y), 0), quaternions[int(q_index)])

        wp = COMMON["nav"]["waypoints"]
        waypoints = []
        waypoints.append(pose_from_tuple(wp["C"]))
        waypoints.append(pose_from_tuple(wp["A"]))
        waypoints.append(pose_from_tuple(wp["B"]))
        waypoints.append(pose_from_tuple(wp["lab4"]))
        waypoints.append(pose_from_tuple(wp["lab3"]))
        waypoints.append(pose_from_tuple(wp["lab2"]))
        waypoints.append(pose_from_tuple(wp["lab1"]))
        waypoints.append(pose_from_tuple(CAR_CONFIG["home_pose"]))
        waypoints.append(pose_from_tuple(wp["board2"]))
        waypoints.append(pose_from_tuple(wp["board1"]))
        return waypoints

    def init_runtime_state(self):
        """初始化状态机变量。"""
        # 车辆专用信息来自 dual_car_config.py。
        self.enable_dual_car = True
        self.car_id = CAR_CONFIG["car_id"]
        self.peer_id = CAR_CONFIG["peer_id"]
        self.start_active = CAR_CONFIG["start_active"]

        # [DUAL-ADD] 轮流令牌状态。
        self.have_turn = (not self.enable_dual_car) or bool(self.start_active)
        self.peer_done_pending = False
        self.dual_round_seq = 0
        self.last_peer_done_seq = 0
        self.turn_released_this_round = False

        # [DUAL-ADD] done 信号重复发布，降低 TCP/ROS 转发偶发漏收风险。
        self.dual_publish_repeat = int(rospy.get_param("~dual_publish_repeat", NAV_CFG["dual_publish_repeat"]))
        self.dual_publish_interval = float(rospy.get_param("~dual_publish_interval", NAV_CFG["dual_publish_interval_sec"]))

        # [DUAL-MOD] 原来默认直接 STATE_GO_TO_BOARD1；
        # 双车模式下，非先发车直接进入 STATE_WAIT_TURN。
        if self.have_turn:
            self.count = STATE_GO_TO_BOARD1
        else:
            self.count = STATE_WAIT_TURN

        self.windows_A = 1
        self.windows_B = 1
        self.windows_C = 1
        self.windows_1234 = 3
        self.windows_count = 3

        self.ram_result = [1, 1, 1, 3, 3]
        self.board2_result = [0, 0]

        self.board1_received = False
        self.board2_received = False

        # 取样窗口完成标志：用于失败重试时跳过已经去过的窗口
        self.pickup_C_done = False
        self.pickup_A_done = False
        self.pickup_B_done = False

        self.cv2_result = "None"
        self.use_peer_board1_result = bool(CAR_CONFIG.get("use_peer_board1_result", False))
        self.peer_board1_data = None
        self.last_received_peer_board1_seq = 0
        self.last_used_peer_board1_seq = 0

    def init_ros_interfaces(self):
        """初始化 ROS 发布器和订阅器。"""
        self.cmd_vel_pub = rospy.Publisher(TOPICS["cmd_vel"], Twist, queue_size=10)

        self.pub_referee_task = rospy.Publisher(TOPICS["referee_task"], String, queue_size=10)
        self.pub_referee_cv1 = rospy.Publisher(TOPICS["referee_cv1"], String, queue_size=10, latch=True)
        self.pub_referee_cv2 = rospy.Publisher(TOPICS["referee_cv2"], String, queue_size=10, latch=True)
        self.pub_nav_state = rospy.Publisher(TOPICS["nav_state"], Int32, queue_size=10, latch=True)
        self.pub_referee_active = rospy.Publisher(
            TOPICS["referee_active"], Bool, queue_size=10, latch=True
        )

        # [DUAL-ADD] 双车本地 ROS 话题。
        # /dual_car/round_done：本车进入化验区/到达化验窗口后，释放另一辆车出发。
        # /dual_car/peer_done：通信节点收到另一辆车 done 后，在本 ROS 内发布。
        # 消息格式：Int32MultiArray.data = [car_id, seq]
        self.pub_round_done = rospy.Publisher(TOPICS["round_done"], Int32MultiArray, queue_size=10)
        self.peer_done_sub = rospy.Subscriber(
            TOPICS["peer_done"],
            Int32MultiArray,
            self.peer_done_callback,
            queue_size=10
        )
        self.peer_board1_sub = rospy.Subscriber(
            BOARD1_CFG["peer_all_text_topic"],
            String,
            self.peer_board1_all_text_callback,
            queue_size=10
        )

        self.cam_sub = rospy.Subscriber(
            TOPICS["cam_return"], Int32MultiArray, self.detect_board1_result, queue_size=10
        )
        self.board2_sub = rospy.Subscriber(
            TOPICS["board2_return"], Int32MultiArray,
            self.detect_board2_result,
            queue_size=10
        )

        # latched 发布初始裁判上报权：
        # car1 默认拥有任务令牌，car2 默认等待。
        self.publish_referee_active(self.have_turn, "startup")
        rospy.sleep(0.5)

    def init_move_base_client(self):
        """连接 move_base，并准备清除代价地图服务。"""
        self.move_base = actionlib.SimpleActionClient(TOPICS["move_base_action"], MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(NAV_CFG["move_base_wait_server_sec"]))

        rospy.wait_for_service(TOPICS["clear_costmaps_service"])
        self.clear_costmaps_service = rospy.ServiceProxy(TOPICS["clear_costmaps_service"], Empty)

    def run_state_machine(self):
        """主状态机循环。"""
        rate = rospy.Rate(NAV_CFG["state_machine_rate_hz"])
        while not rospy.is_shutdown():
            # [DUAL-ADD] 等待另一辆车完成并释放本车。
            if self.count == STATE_WAIT_TURN:
                self.handle_wait_turn()
            elif self.count == STATE_GO_TO_BOARD1:
                self.handle_go_to_board1()
            elif self.count == STATE_GO_TO_PICKUP_WINDOWS:
                self.handle_go_to_pickup_windows()
            elif self.count == STATE_GO_TO_BOARD2:
                self.handle_go_to_board2()
            elif self.count == STATE_GO_TO_LAB_WINDOW:
                self.handle_go_to_lab_window()
            elif self.count == STATE_GO_BACK_HOME:
                self.handle_go_back_home()
            rate.sleep()

    def handle_go_to_board1(self):
        """状态 9：从起点前往识别板一。"""
        # [DUAL-ADD] 双车模式下，如果还没有令牌，不允许从起点出发。
        if self.enable_dual_car and not self.have_turn:
            rospy.loginfo_throttle(2.0, "[双车] car%s 当前没有出发令牌，继续等待 car%s done",
                                   self.car_id, self.peer_id)
            self.count = STATE_WAIT_TURN
            self.pub_nav_state.publish(self.count)
            self.publish_referee_active(False, "go_to_board1_without_turn")
            return

        # [DUAL-ADD] 新一轮开始，允许本轮在进入化验区后释放一次 done。
        self.turn_released_this_round = False

        self.pub_referee_task.publish("R")
        self.board1_received = False

        if self.try_use_peer_board1_result_before_board1_navigation():
            rospy.loginfo("[BOARD1-SHARE] 已使用对车 all_text，跳过识别板一停车点")
            self.count = STATE_GO_TO_PICKUP_WINDOWS
            self.pub_nav_state.publish(self.count)
            return

        rospy.loginfo("正在前往识别板一！")
        self.pub_nav_state.publish(9)

        goal = self.make_goal(self.waypoints[9])
        if self.move(goal) is True:
            rospy.loginfo("已经到达识别板一，正在识别！")
            self.board1_received = False #重置识别版一接受状态
            self.count = STATE_BOARD1_RECOGNIZING
            self.pub_nav_state.publish(self.count)
            self.clear_costmaps_after_arrival()
            # 等待识别板一真正识别成功
            if self.wait_for_board1_result():
                rospy.loginfo("识别板一识别成功，准备前往取样窗口")
                self.count = STATE_GO_TO_PICKUP_WINDOWS
                self.pub_nav_state.publish(self.count)
        else:
            rospy.logerr("前往识别版一失败，重新尝试")
            return

    def handle_go_to_pickup_windows(self):
        """状态 11：根据识别板一结果，依次前往 C/A/B 取样窗口。"""
        rospy.loginfo('识别成功，开始配送药品')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        if self.windows_C == 1 and not self.pickup_C_done:
            goal = self.make_goal(self.waypoints[0])
            if self.move(goal) is True:
                announce_code = "C" if self.windows_A == 0 and self.windows_B == 0 else None
                self.arrive_and_leave_pickup_window("C", announce_code)
            else:
                rospy.logerr("前往 C 窗口失败，重新尝试")
                return

        if self.windows_A == 1 and not self.pickup_A_done:
            goal = self.make_goal(self.waypoints[1])
            if self.move(goal) is True:
                announce_code = None
                if self.windows_C == 0 and self.windows_B == 0:
                    announce_code = "A"
                elif self.windows_C == 1 and self.windows_B == 0:
                    announce_code = "AC"
                self.arrive_and_leave_pickup_window("A", announce_code)
            else:
                rospy.logerr("前往 A 窗口失败，重新尝试")
                return

        if self.windows_B == 1 and not self.pickup_B_done:
            goal = self.make_goal(self.waypoints[2])
            if self.move(goal) is True:
                if self.windows_C == 1 and self.windows_A == 1:
                    announce_code = "ABC"
                elif self.windows_C == 0 and self.windows_A == 1:
                    announce_code = "AB"
                elif self.windows_C == 1 and self.windows_A == 0:
                    announce_code = "BC"
                else:
                    announce_code = "B"
                self.arrive_and_leave_pickup_window("B", announce_code)
            else:
                rospy.logerr("前往 B 窗口失败，重新尝试")
                return

        self.count = STATE_GO_TO_BOARD2
        self.pub_nav_state.publish(self.count)

    def handle_go_to_board2(self):
        """状态 12：前往识别板二，并根据识别板二结果决定是否等待。"""
        rospy.loginfo("正在前往识别板2！")
        self.board2_result = [0, 0]
        self.board2_received = False #重置识别板二接收状态

        goal = self.make_goal(self.waypoints[8])
        if self.move(goal) is True:
            rospy.loginfo("已经到达识别板2！正在识别！")
            self.count = STATE_BOARD2_RECOGNIZING
            self.pub_nav_state.publish(self.count)
            self.clear_costmaps_after_arrival()
            if self.wait_for_board2_result():
                rospy.loginfo("识别板二识别成功，结果为: %s", self.board2_result)

                if self.board2_result[0] == 1:
                    wait_time = self.board2_result[1]
                    rospy.loginfo("化验区忙碌，等待 %d 秒", wait_time)
                    self.play_audio("WAIT-%d.wav" % wait_time)
                    rospy.sleep(wait_time)
                else:
                    rospy.loginfo("化验区空闲，无需等待")
                    self.play_audio("WAIT-0.wav")

                self.count = STATE_GO_TO_LAB_WINDOW
                self.pub_nav_state.publish(self.count)
        else:
            rospy.logerr("前往识别版二失败，重新尝试")
            return


    def handle_go_to_lab_window(self):
        """状态 14：前往最终化验窗口。"""
        rospy.loginfo("正在前往化验室窗口！")

        goal = self.make_goal(self.waypoints[6 - self.windows_1234])
        if self.move(goal) is True:
            real_window_str = str(self.windows_1234 + 1)
            self.announce_delivery_result()
            self.pub_referee_task.publish(real_window_str)
            rospy.loginfo("已经到达化验室" + real_window_str + "窗口！")
            self.clear_costmaps_after_arrival()
            rospy.sleep(NAV_CFG["lab_task_hold_sec"])
            self.pub_referee_task.publish("R")

            # [DUAL-MOD-STATE15] 进入状态15的瞬间释放令牌。
            # 也就是本车完成化验窗口停留和播报，准备返回起点时，才允许另一辆车出发。
            self.count = STATE_GO_BACK_HOME
            self.pub_nav_state.publish(self.count)
            self.release_next_car_if_needed("enter_state_15_go_back_home")
        else:
            rospy.logerr("前往化验窗口失败，重新尝试")
            return

    def handle_go_back_home(self):
        """状态 15：送完样本后先回到原点，再前往识别板一。"""
        rospy.loginfo("送样完成，正在返回原点！")
        self.pub_referee_task.publish("R")
        self.pub_nav_state.publish(STATE_GO_BACK_HOME)

        goal = self.make_goal(self.waypoints[7])

        if self.move(goal) is True:
            rospy.loginfo("已经回到原点，准备前往识别板一！")
            self.clear_costmaps_after_arrival()
            rospy.sleep(NAV_CFG["home_arrive_sleep_sec"])

            # [DUAL-MOD] 原来这里直接进入下一轮 STATE_GO_TO_BOARD1。
            # 双车模式下，本车回到起点后等待另一辆车的 done；单车模式保持原逻辑。
            if self.enable_dual_car:
                if self.peer_done_pending:
                    rospy.loginfo("[双车] 回到起点时已收到 car%s done，立即开始下一轮", self.peer_id)
                    self.activate_turn_from_peer()
                else:
                    rospy.loginfo("[双车] 已回到起点，等待 car%s done 后再出发", self.peer_id)
                    self.have_turn = False
                    self.count = STATE_WAIT_TURN
                    self.pub_nav_state.publish(self.count)
            else:
                self.count = STATE_GO_TO_BOARD1
                self.pub_nav_state.publish(self.count)
        else:
            rospy.logerr("返回原点失败，重新尝试返回原点")
            return


    def publish_referee_active(self, active, reason=""):
        """发布本车是否拥有裁判 TCP 上报权。"""
        active = bool(active)
        self.pub_referee_active.publish(Bool(data=active))
        rospy.loginfo(
            "[裁判上报] car%s active=%s reason=%s",
            self.car_id,
            active,
            reason
        )

    def handle_wait_turn(self):
        """[DUAL-ADD] 状态 8：等待另一辆车释放令牌。"""
        self.pub_referee_task.publish("R")
        self.pub_nav_state.publish(STATE_WAIT_TURN)

        # 如果 peer_done 比本车回起点更早到达，则会先缓存；
        # 本车进入等待状态后立即消耗这个缓存，避免漏掉提前到达的 done。
        if self.peer_done_pending:
            rospy.loginfo("[双车] 检测到已缓存的 car%s done，准备出发", self.peer_id)
            self.activate_turn_from_peer()
            return

        rospy.loginfo_throttle(
            2.0,
            "[双车] car%s 正在等待 car%s done...",
            self.car_id,
            self.peer_id
        )

    def release_next_car_if_needed(self, reason=""):
        """[DUAL-ADD] 本车进入化验区后释放另一辆车。"""
        if not self.enable_dual_car:
            return

        if self.turn_released_this_round:
            rospy.loginfo_throttle(2.0, "[双车] 本轮 done 已发布过，忽略重复发布请求")
            return

        # 必须先停止本车裁判上报，再向下一辆车发送 done。
        # 对车收到 done 后才会开启上报，避免交接瞬间两车同时连接裁判软件。
        self.have_turn = False
        self.publish_referee_active(False, "release_turn:%s" % reason)

        self.dual_round_seq += 1
        msg = Int32MultiArray()
        msg.data = [self.car_id, self.dual_round_seq]

        rospy.loginfo(
            "[双车] car%s 发布 done，seq=%s，reason=%s，允许 car%s 出发",
            self.car_id,
            self.dual_round_seq,
            reason,
            self.peer_id
        )

        for _ in range(self.dual_publish_repeat):
            self.pub_round_done.publish(msg)
            rospy.sleep(self.dual_publish_interval)

        # 本车已经把本轮任务令牌交给对方，但仍继续返回起点。
        # 本车此时继续导航，但不再向裁判软件发送。
        self.turn_released_this_round = True

    def peer_done_callback(self, msg):
        """[DUAL-ADD] 收到通信节点转发的另一辆车 done。"""
        if not self.enable_dual_car:
            return

        data = list(msg.data)
        if len(data) < 2:
            rospy.logwarn("[双车] peer_done 数据格式错误，应为 [car_id, seq]，实际为: %s", data)
            return

        try:
            done_car_id = int(data[0])
            done_seq = int(data[1])
        except Exception:
            rospy.logwarn("[双车] peer_done 数据无法转为整数: %s", data)
            return

        if done_car_id != self.peer_id:
            rospy.logwarn_throttle(
                2.0,
                "[双车] 收到非对方车辆 done，已忽略: car_id=%s seq=%s",
                done_car_id,
                done_seq
            )
            return

        if done_seq <= self.last_peer_done_seq:
            rospy.loginfo_throttle(
                2.0,
                "[双车] 收到重复或旧的 car%s done，已忽略: seq=%s last=%s",
                done_car_id,
                done_seq,
                self.last_peer_done_seq
            )
            return

        self.last_peer_done_seq = done_seq
        self.peer_done_pending = True

        rospy.loginfo("[双车] 收到 car%s done，seq=%s", done_car_id, done_seq)

        if self.count == STATE_WAIT_TURN:
            self.activate_turn_from_peer()
        else:
            rospy.loginfo(
                "[双车] 当前状态=%s，暂不立即出发；done 已缓存，等回到起点/等待状态后再出发",
                self.count
            )

    def activate_turn_from_peer(self):
        """[DUAL-ADD] 消耗 peer_done，切换到本车下一轮。"""
        self.peer_done_pending = False
        self.have_turn = True
        self.turn_released_this_round = False

        # 获得新一轮任务令牌后，才允许本车连接并向裁判软件发送。
        self.publish_referee_active(True, "received_peer_done")

        self.count = STATE_GO_TO_BOARD1
        self.pub_nav_state.publish(self.count)

        rospy.loginfo("[双车] car%s 获得出发令牌，进入 STATE_GO_TO_BOARD1", self.car_id)

    def peer_board1_all_text_callback(self, msg):
        """缓存通信节点转发的对车识别板一 all_text。"""
        try:
            obj = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[BOARD1-SHARE] 对车 all_text JSON 解析失败，已忽略: %s", str(e))
            return

        sender_car_id = self.safe_int(obj.get("car_id", None), None)
        seq = self.safe_int(obj.get("seq", None), None)

        if sender_car_id != self.peer_id:
            rospy.logwarn("[BOARD1-SHARE] car_id 不是对方车，已忽略: car_id=%s expected=%s",
                          sender_car_id, self.peer_id)
            return

        if seq is None or seq <= 0:
            rospy.logwarn("[BOARD1-SHARE] 对车 all_text seq 非法，已忽略: %s", obj)
            return

        if seq <= self.last_received_peer_board1_seq:
            rospy.loginfo_throttle(
                1.0,
                "[BOARD1-SHARE] 对车 all_text 为旧数据或重复数据，已忽略: seq=%s last=%s",
                seq,
                self.last_received_peer_board1_seq
            )
            return

        self.peer_board1_data = obj
        self.last_received_peer_board1_seq = seq
        rospy.loginfo("[BOARD1-SHARE] 已缓存对车 all_text: seq=%s", seq)

    def try_use_peer_board1_result_before_board1_navigation(self):
        """car2 在去识别板一停车点前，优先尝试使用对车 all_text。"""
        if not self.use_peer_board1_result:
            return False

        if self.peer_board1_data is None:
            rospy.loginfo("[BOARD1-SHARE] 未收到对车 all_text，回退本车识别")
            return False

        obj = self.peer_board1_data
        sender_car_id = self.safe_int(obj.get("car_id", None), None)
        seq = self.safe_int(obj.get("seq", None), None)

        if sender_car_id != self.peer_id:
            rospy.logwarn("[BOARD1-SHARE] car_id 不是对方车，回退本车识别: car_id=%s expected=%s",
                          sender_car_id, self.peer_id)
            return False

        if seq is None or seq <= 0 or seq <= self.last_used_peer_board1_seq:
            rospy.loginfo("[BOARD1-SHARE] 对车 all_text 为旧数据或已使用，回退本车识别")
            return False

        all_text = obj.get("all_text", None)
        if not isinstance(all_text, list) or len(all_text) == 0:
            rospy.logwarn("[BOARD1-SHARE] 对车 all_text 为空或格式错误，回退本车识别")
            return False

        selected_index = self.safe_int(obj.get("selected_index", None), None)
        if selected_index is None or selected_index < 0 or selected_index >= len(all_text):
            rospy.logwarn("[BOARD1-SHARE] 对车 selected_index 不合法，回退本车识别: %s", selected_index)
            return False

        remaining_all_text = list(all_text)
        remaining_all_text[selected_index] = ""

        selection = select_board1_from_all_text(remaining_all_text)
        if selection is None:
            rospy.loginfo("[BOARD1-SHARE] 去掉对车已选项后无剩余有效二维码，回退本车识别")
            return False

        self.apply_board1_selected_msg(selection["selected_msg"])
        self.last_used_peer_board1_seq = seq

        rospy.loginfo(
            "[BOARD1-SHARE] 已使用对车 all_text，跳过识别板一停车点: seq=%s selected_index=%s selected_text=%s selected_msg=%s",
            seq,
            selection["selected_index"],
            selection["selected_text"],
            selection["selected_msg"]
        )
        return True

    def safe_int(self, value, default=None):
        try:
            return int(value)
        except Exception:
            return default

    def apply_board1_selected_msg(self, selected_msg):
        """把识别板一选择结果写入状态机变量，并发布 CV2。"""
        self.ram_result = list(selected_msg)
        self.windows_C = self.ram_result[0]
        self.windows_A = self.ram_result[1]
        self.windows_B = self.ram_result[2]
        self.windows_count = self.ram_result[3]
        self.windows_1234 = self.ram_result[4]

        self.pickup_C_done = False
        self.pickup_A_done = False
        self.pickup_B_done = False

        self.cv2_result = self.build_cv2_result()
        self.publish_cv2_repeated(self.cv2_result)
        self.board1_received = True

    def make_goal(self, pose):
        """根据 Pose 创建 MoveBaseGoal。"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return goal

    def wait_for_board1_result(self):
        """等待识别板一识别成功；配置为 0 时沿用旧逻辑无限等待。"""
        rospy.loginfo("等待识别板一识别结果...")
        timeout = float(NAV_CFG["board1_wait_timeout_sec"])
        start_time = time.time()
        rate = rospy.Rate(NAV_CFG["wait_result_rate_hz"])
        while not rospy.is_shutdown() and not self.board1_received:
            if timeout > 0 and (time.time() - start_time) > timeout:
                rospy.logerr("识别板一等待超时 %.1f 秒，返回失败", timeout)
                return False
            rospy.loginfo_throttle(1.0, "识别板一尚未识别成功，继续等待...")
            rate.sleep()
        return self.board1_received

    def wait_for_board2_result(self):
        """等待识别板二识别成功；配置为 0 时沿用旧逻辑无限等待。"""
        rospy.loginfo("等待识别板二识别结果...")
        timeout = float(NAV_CFG["board2_wait_timeout_sec"])
        start_time = time.time()
        rate = rospy.Rate(NAV_CFG["wait_result_rate_hz"])
        while not rospy.is_shutdown() and not self.board2_received:
            if timeout > 0 and (time.time() - start_time) > timeout:
                rospy.logerr("识别板二等待超时 %.1f 秒，返回失败", timeout)
                return False
            rospy.loginfo_throttle(1.0, "识别板二尚未识别成功，继续等待...")
            rate.sleep()
        return self.board2_received

    def move(self, goal):
        """发送导航目标，并等待 move_base 返回结果。"""
        self.move_base.send_goal(goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(NAV_CFG["move_timeout_sec"]))

        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.logwarn("Timed out achieving goal，清除代价地图后准备重试")
            try:
                self.clear_costmaps_service()
            except Exception as e:
                rospy.logwarn("清除代价地图失败: %s", e)
            rospy.sleep(NAV_CFG["post_clear_costmap_sleep_sec"])
            return False

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            return True

        rospy.logwarn("导航目标失败，move_base state=%s，清除代价地图后准备重试", state)
        try:
            self.clear_costmaps_service()
        except Exception as e:
            rospy.logwarn("清除代价地图失败: %s", e)

        rospy.sleep(NAV_CFG["post_clear_costmap_sleep_sec"])
        return False

    def clear_costmaps_after_arrival(self):
        """根据配置决定是否在成功到达目标点后清除代价地图。"""
        if NAV_CFG["clear_costmaps_on_arrival"]:
            self.clear_costmaps_service()

    def arrive_and_leave_pickup_window(self, window_name, announce_code=None):
        """到达取样窗口后立即播报，并在播报期间完成停留等待。"""

        # 语音通过 Popen 异步启动，会与后面的清图和 1.1 秒停留同时进行。
        if announce_code is not None:
            self.announce_pickup_result(announce_code)
        self.pub_referee_task.publish(window_name)

        self.clear_costmaps_after_arrival()
        rospy.sleep(NAV_CFG["pickup_task_hold_sec"])
        self.pub_referee_task.publish("R")

        # 标记该窗口已经完成，避免导航失败重试时重复前往
        if window_name == "C":
            self.pickup_C_done = True
        elif window_name == "A":
            self.pickup_A_done = True
        elif window_name == "B":
            self.pickup_B_done = True

        rospy.loginfo("%s 窗口取样流程已完成，已记录完成状态", window_name)

    def play_audio(self, filename):
        path = os.path.join(AUDIO_DIR, filename)
        if not os.path.exists(path):
            rospy.logwarn("语音文件不存在: %s", path)
            return
        subprocess.Popen(["play", path])

    def announce_pickup_result(self, window_code):
        """发布并播报从 A/B/C 取到的样本组合。"""
        lab_info = LAB_INFO[self.windows_1234]
        rospy.loginfo(
            "取到%s窗口中的%s样本",
            WINDOW_LOG_NAME[window_code],
            lab_info["sample_name"]
        )

        self.play_audio("%s%s.wav" % (lab_info["pickup_audio_prefix"], window_code))

    def announce_delivery_result(self):
        """到达最终化验窗口后，根据样本数量播报。"""
        lab_info = LAB_INFO[self.windows_1234]
        rospy.loginfo(
            "到达%s，样本数为%d",
            lab_info["lab_name"],
            self.windows_count
        )
        self.play_audio("%s%d.wav" % (lab_info["delivery_audio_prefix"], self.windows_count))

    def detect_board1_result(self, msg):
        """识别板一回调：只在状态 10 时更新任务参数。"""
        if self.count == STATE_BOARD1_RECOGNIZING:
            self.ram_result = list(msg.data)
            rospy.loginfo("识别到二维码并已经选择了最优方案：%s....正在更新参数", msg.data)
            rospy.logwarn("self.ram_result: %s", self.ram_result)

            self.apply_board1_selected_msg(self.ram_result)

    def detect_board2_result(self, msg):
        """识别板二回调：只在状态 13 时接收等待状态。"""
        if self.count == STATE_BOARD2_RECOGNIZING:
            data = list(msg.data)
            if len(data) < 2:
                rospy.logwarn("识别板2返回数据长度错误: %s", data)
                return

            state = int(data[0])
            wait_time = int(data[1])

            if state == 0:
                self.board2_state = 0
                self.board2_wait_time = 0
                self.board2_result = [0, 0]
                self.publish_cv1_repeated("WAIT-0")
                self.board2_received = True

            elif state == 1:
                if wait_time < 5:
                    rospy.logwarn("识别板2等待时间过小: %d，已修正为5秒", wait_time)
                    wait_time = 5
                elif wait_time > 10:
                    rospy.logwarn("识别板2等待时间过大: %d，已修正为10秒", wait_time)
                    wait_time = 10

                self.board2_state = 1
                self.board2_wait_time = wait_time
                self.board2_result = [1, wait_time]

                self.publish_cv1_repeated("WAIT-%d" % wait_time)

                self.board2_received = True

    def publish_cv1_repeated(self, value):
        """重复发布 CV1，降低 ROS 订阅漏收概率。"""
        for _ in range(NAV_CFG["cv_publish_repeat"]):
            self.pub_referee_cv1.publish(value)
            rospy.sleep(NAV_CFG["cv_publish_interval_sec"])

    def publish_cv2_repeated(self, value):
        """重复发布 CV2，降低 ROS 订阅漏收概率。"""
        for _ in range(NAV_CFG["cv_publish_repeat"]):
            self.pub_referee_cv2.publish(value)
            rospy.sleep(NAV_CFG["cv_publish_interval_sec"])

    def build_cv2_result(self):
        """根据识别板一结果生成 CV2，例如 A-2、AB-1、ABC-4。"""
        window_code = ""

        if self.windows_A == 1:
            window_code += "A"
        if self.windows_B == 1:
            window_code += "B"
        if self.windows_C == 1:
            window_code += "C"

        real_window = LAB_INFO[self.windows_1234]["real_window"]

        return "%s-%d" % (window_code, real_window)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
