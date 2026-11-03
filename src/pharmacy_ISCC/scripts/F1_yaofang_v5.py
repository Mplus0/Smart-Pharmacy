#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
F1_yaofang.py

主导航与任务状态机节点：
1. 起点 -> 识别板一
2. 根据识别板一结果前往 A/B/C 取样窗口
3. 前往识别板二，读取化验区等待状态
4. 前往 1/2/3/4 化验窗口


备注：新增防卡顿：将状态机数据发布节点，记录所有状态机的变化,发布于 /nav_state 话题，类型为 Int32，发布时机为每次状态改变时。
     新增等待识别结果的机制：在前往识别板一和识别板二的状态中，增加一个循环等待机制，直到收到对应的识别结果消息才继续执行后续逻辑。通过设置标志位（如 board1_received 和 board2_received）来控制循环退出条件。
    新增超时保护
    新增取样窗口完成标志：在前往取样窗口的状态中，增加标志位（如 pickup_C_done、pickup_A_done、pickup_B_done）来记录每个窗口的完成状态。在导航失败重试时，检查这些标志位以决定是否需要重新前往已经完成的窗口，从而避免重复前往同一窗口导致的卡顿问题。
    新增回到原点后重新前往识别板一的逻辑：在送样完成后，先导航回原点，然后再前往识别板一，确保每轮任务都从起点开始，避免直接从化验窗口前往识别板一可能导致的导航问题。
"""

import os
import sys
import random
import socket
import json
import threading
import argparse
import tf
from pharmacy_iscc.navigation_utils import NavigationGuard, target_pose, quaternion_yaw, validate_target

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String, Int32, Int32MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from std_srvs.srv import Empty, Trigger, TriggerResponse
import time

from pharmacy_iscc.config import COMMON, get_car_config, get_car_id
from pharmacy_iscc.recognition_records import RecognitionRecords
from pharmacy_iscc.audio import play_audio

CAR_ID = get_car_id(default=1)
CAR_CONFIG = get_car_config(CAR_ID)
STATES = COMMON["states"]
TOPICS = COMMON["topics"]
NAV_CFG = COMMON["nav"]
BOARD1_CFG = COMMON["board1"]


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
        self.recognition_records = RecognitionRecords(self.car_id)
        self.init_ros_interfaces()
        self.init_move_base_client()
        # Reuse only the map/localization checks before sending a goal.
        self.arrival_guard = NavigationGuard(argparse.Namespace())
        if rospy.get_param('~wait_for_startup_ready', False):
            if not self.wait_for_startup_ready():
                return

        if self.wait_for_manual_start():
            rospy.loginfo("已收到手动启动指令，开始任务！")
            self.run_state_machine()

    def wait_for_startup_ready(self):
        """快速合并启动时按视觉、RGB与定位实际就绪情况放行，无固定延时。"""
        from sensor_msgs.msg import Image, CompressedImage
        config = NAV_CFG['startup']
        topic = COMMON['detect'].get('camera_topic', '/camera/rgb/image_raw')
        source = COMMON['detect'].get('image_source', 'ros_topic')
        image_type = CompressedImage if source == 'ros_compressed' or topic.endswith('/compressed') else Image
        deadline = time.time() + float(config['ready_timeout_sec'])
        ready_param = config['vision_node'].rstrip('/') + '/startup_ready'
        while not rospy.is_shutdown() and time.time() < deadline:
            if rospy.get_param(ready_param, False):
                try:
                    image = rospy.wait_for_message(topic, image_type, timeout=min(
                        0.5, max(0.01, deadline - time.time())))
                    age = (rospy.Time.now() - image.header.stamp).to_sec()
                    if (not image.data or image.header.stamp.to_sec() <= 0
                            or not -0.1 <= age <= float(config['image_max_age_sec'])):
                        raise ValueError('RGB image is empty or stale')
                    grid, unused = self.arrival_guard.grid()
                    point = self.waypoints[9].position
                    validate_target(grid, target_pose(point.x, point.y, 0.0))
                    self.arrival_guard.pose()
                    rospy.loginfo('相机、识别、地图与定位已就绪，可以发车')
                    return True
                except (rospy.ROSException, ValueError) as exc:
                    rospy.loginfo_throttle(2.0, '等待启动就绪: %s', exc)
            else:
                rospy.loginfo_throttle(2.0, '等待相机与识别就绪...')
            rospy.sleep(1.0 / float(config['poll_rate_hz']))
        if not rospy.is_shutdown():
            rospy.logerr('启动就绪检查超时，未放行小车')
            rospy.signal_shutdown('Startup readiness timed out')
        return False

    def wait_for_manual_start(self):
        """初始化完成后提供启动服务，放行前不进入导航状态机。"""
        self.motion_start_event = threading.Event()
        self.motion_start_lock = threading.Lock()
        if rospy.is_shutdown():
            return False
        self.motion_start_service = rospy.Service('~start', Trigger, self.start_motion_callback)
        rospy.loginfo("初始化完成，等待手动启动：rosservice call %s/start '{}'",
                      rospy.get_name())
        while not rospy.is_shutdown() and not self.motion_start_event.is_set():
            self.motion_start_event.wait(0.1)
        return not rospy.is_shutdown()

    def start_motion_callback(self, request):
        """只放行一次，不在服务线程中执行导航或重置任务。"""
        with self.motion_start_lock:
            if rospy.is_shutdown():
                return TriggerResponse(success=False, message='Task is shutting down')
            if self.motion_start_event.is_set():
                return TriggerResponse(success=False, message='Task already started')
            self.motion_start_event.set()
            return TriggerResponse(success=True, message='Task start accepted')

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
        # 车辆专用信息由 config.py 从 vehicles.yaml 读取。
        self.enable_dual_car = bool(rospy.get_param("~enable_dual_car", False))
        self.max_rounds = int(rospy.get_param("~max_rounds", 1))
        if self.max_rounds < 0:
            raise ValueError("max_rounds must be >= 0 (0 means continuous)")
        self.completed_rounds = 0
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
        self.board2_result_lock = threading.Lock()
        self.board2_recognition_deadline = None

        self.board1_received = False
        self.board2_received = False
        self.pending_board1_tasks = []  # 单车本批次尚未开始的任务。
        self.deliveries_since_board1_recognition = 0

        # 取样窗口完成标志：用于失败重试时跳过已经去过的窗口
        self.pickup_C_done = False
        self.pickup_A_done = False
        self.pickup_B_done = False

        self.use_peer_board1_result = bool(CAR_CONFIG.get("use_peer_board1_result", False))
        self.peer_board1_data = None
        self.last_received_peer_board1_seq = 0
        self.last_used_peer_board1_seq = 0

    def init_ros_interfaces(self):
        """初始化 ROS 发布器和订阅器。"""
        self.cmd_vel_pub = rospy.Publisher(TOPICS["cmd_vel"], Twist, queue_size=10)

        self.pub_nav_state = rospy.Publisher(TOPICS["nav_state"], Int32, queue_size=10, latch=True)

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

    def init_move_base_client(self):
        """连接 move_base，并准备清除代价地图服务。"""
        self.move_base = actionlib.SimpleActionClient(TOPICS["move_base_action"], MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        if not self.move_base.wait_for_server(rospy.Duration(NAV_CFG["move_base_wait_server_sec"])):
            rospy.logerr("move_base 未就绪，不提供手动启动服务")
            rospy.signal_shutdown("move_base action server unavailable")
            raise rospy.ROSInterruptException("move_base action server unavailable")

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

    def move_via_points(self, points, board_name, transition=None):
        """依次到达引导点；中途保持原导航状态，不触发识别。"""
        for index, point in enumerate(points, 1):
            if rospy.is_shutdown():
                return False
            orientation = Quaternion(*quaternion_from_euler(
                0, 0, radians(float(point["yaw_deg"]))))
            pose = Pose(Point(float(point["x"]), float(point["y"]), 0),
                        orientation)
            rospy.loginfo("[%s-APPROACH] 前往引导点%s: x=%.3f y=%.3f yaw=%.1f",
                          board_name, index, point["x"], point["y"], point["yaw_deg"])
            if not self.move(self.make_goal(pose), transition=transition):
                return False
        return not rospy.is_shutdown()

    def handle_go_to_board1(self):
        """状态 9：前往识别板一，加载缓存任务或重新识别。"""
        # [DUAL-ADD] 双车模式下，如果还没有令牌，不允许从起点出发。
        if self.enable_dual_car and not self.have_turn:
            rospy.loginfo_throttle(2.0, "[双车] car%s 当前没有出发令牌，继续等待 car%s done",
                                   self.car_id, self.peer_id)
            self.count = STATE_WAIT_TURN
            self.pub_nav_state.publish(self.count)
            return

        # [DUAL-ADD] 新一轮开始，允许本轮在进入化验区后释放一次 done。
        self.turn_released_this_round = False

        self.board1_received = False

        if self.enable_dual_car and self.try_use_peer_board1_result_before_board1_navigation():
            rospy.loginfo("[BOARD1-SHARE] 已使用对车 all_text，跳过识别板一停车点")
            self.count = STATE_GO_TO_PICKUP_WINDOWS
            self.pub_nav_state.publish(self.count)
            return

        rospy.loginfo("正在前往识别板一！")
        self.pub_nav_state.publish(9)

        goal = self.make_goal(self.waypoints[9])
        if self.move(goal) is True:
            if not self.enable_dual_car and self.pending_board1_tasks:
                # 已有整板记忆：到达后加载下一项，不进入识别状态覆盖缓存。
                if rospy.is_shutdown():
                    return
                self.clear_costmaps_service()
                next_task = self.pending_board1_tasks.pop(0)
                self.apply_board1_selected_msg(next_task)
                self.count = STATE_GO_TO_PICKUP_WINDOWS
                self.pub_nav_state.publish(self.count)
                rospy.loginfo("已经过板一，执行缓存任务，送样窗口=%s，后续剩余=%s",
                              self.windows_1234 + 1, len(self.pending_board1_tasks))
                return
            rospy.loginfo("已经到达识别板一，正在识别！")
            self.board1_received = False #重置识别版一接受状态
            recognition_started = time.time()
            self.count = STATE_BOARD1_RECOGNIZING
            self.pub_nav_state.publish(self.count)
            # 等待识别板一真正识别成功
            if self.wait_for_board1_result(start_time=recognition_started):
                rospy.loginfo("识别板一识别成功，准备前往取样窗口")
                self.clear_costmaps_service()
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

        pickup_code = "".join(w for w in "ABC"
                              if getattr(self, "windows_" + w) == 1)
        last_window = "B" if self.windows_B == 1 else (
            "A" if self.windows_A == 1 else "C")

        if self.windows_C == 1 and not self.pickup_C_done:
            goal = self.make_goal(self.waypoints[0])
            if self.move(goal) is True:
                self.arrive_and_leave_pickup_window(
                    "C", pickup_code if last_window == "C" else None)
            else:
                rospy.logerr("前往 C 窗口失败，重新尝试")
                return

        if self.windows_A == 1 and not self.pickup_A_done:
            goal = self.make_goal(self.waypoints[1])
            if self.move(goal) is True:
                self.arrive_and_leave_pickup_window(
                    "A", pickup_code if last_window == "A" else None)
                self.clear_costmaps_service()
            else:
                rospy.logerr("前往 A 窗口失败，重新尝试")
                return

        if self.windows_B == 1 and not self.pickup_B_done:
            goal = self.make_goal(self.waypoints[2])
            if self.move(goal) is True:
                self.arrive_and_leave_pickup_window(
                    "B", pickup_code if last_window == "B" else None)
            else:
                rospy.logerr("前往 B 窗口失败，重新尝试")
                return

        self.count = STATE_GO_TO_BOARD2
        self.pub_nav_state.publish(self.count)

    def move_to_board2(self):
        """按原速度连续接近上方过渡点，切换前恢复正常容差和终速约束。"""
        route = NAV_CFG["board2_approach"]
        if not route["enabled"]:
            return self.move(self.make_goal(self.waypoints[8]))

        client = None
        original_settings = None
        reached = False
        failure = None
        try:
            timeout = float(route["config_timeout_sec"])
            client = self.arrival_guard.configuration_client(route["planner_namespace"], timeout)
            original = client.get_configuration(timeout=timeout)
            descriptions = client.get_parameter_descriptions(timeout=timeout)
            if original is None or descriptions is None:
                raise RuntimeError("TEB configuration is unavailable")
            names = ("xy_goal_tolerance", "yaw_goal_tolerance", "free_goal_vel")
            original_settings = dict((name, original[name]) for name in names)
            settings = {
                "xy_goal_tolerance": max(float(original_settings["xy_goal_tolerance"]),
                                         float(route["xy_goal_tolerance"])),
                "yaw_goal_tolerance": max(float(original_settings["yaw_goal_tolerance"]),
                                          radians(float(route["yaw_goal_tolerance_deg"]))),
                "free_goal_vel": bool(route["free_goal_vel"]),
            }
            applied = client.update_configuration(dict(settings))
            if any(abs(float(applied[name]) - float(settings[name])) > 1e-6 for name in names):
                raise RuntimeError("TEB transition settings were not applied")
            rospy.loginfo("[BOARD2-APPROACH] 不额外限速，过渡点临时设置: %s", settings)

            reached = self.move_via_points(route["via_points"], "BOARD2", transition=route)
        except Exception as exc:
            if not rospy.is_shutdown():
                rospy.logerr("[BOARD2-APPROACH] 上方进场失败: %s", exc)
                failure = "Board2 approach failed"
        finally:
            if client is not None:
                try:
                    if original_settings is not None:
                        restored = client.update_configuration(dict(original_settings))
                        if any(abs(float(restored[name]) - float(value)) > 1e-6
                               for name, value in original_settings.items()):
                            raise RuntimeError("TEB transition settings were not restored")
                        rospy.loginfo("[BOARD2-APPROACH] 已恢复原容差和终速约束")
                except Exception as exc:
                    rospy.logerr("[BOARD2-APPROACH] 恢复过渡设置失败: %s", exc)
                    failure = "Board2 approach configuration restore failed"
        if failure is not None and not rospy.is_shutdown():
            rospy.signal_shutdown(failure)
        if not reached or rospy.is_shutdown():
            return False
        return self.move(self.make_goal(self.waypoints[8]))

    def handle_go_to_board2(self):
        """状态 12：前往识别板二，并根据识别板二结果决定是否等待。"""
        rospy.loginfo("正在前往识别板2！")
        with self.board2_result_lock:
            self.board2_result = [0, 0]
            self.board2_received = False
            self.board2_recognition_deadline = None

        if self.move_to_board2() is True:
            timeout = float(NAV_CFG["board2_wait_timeout_sec"])
            with self.board2_result_lock:
                self.board2_recognition_deadline = time.time() + timeout if timeout > 0 else None
            rospy.loginfo("已经到达识别板2！正在识别！")
            self.count = STATE_BOARD2_RECOGNIZING
            self.pub_nav_state.publish(self.count)
            if self.wait_for_board2_result():
                rospy.loginfo("识别板二处理完成，采用结果: %s", self.board2_result)

                if self.board2_result[0] == 1:
                    wait_time = self.board2_result[1]
                    actual_wait = max(0, wait_time - 2)
                    wait_started = time.time()
                    rospy.loginfo("化验区忙碌，识别等待 %d 秒，实际等待 %d 秒（含播报）",
                                  wait_time, actual_wait)
                    play_audio("WAIT-%d.wav" % wait_time)
                    # 播报计入等待；若播报更长，仍坚持播报结束后才出发。
                    remaining = max(0.0, actual_wait - (time.time() - wait_started))
                    if remaining > 0 and not rospy.is_shutdown():
                        rospy.sleep(remaining)
                else:
                    rospy.loginfo("化验区空闲，无需等待")
                    play_audio("WAIT-0.wav")

                self.clear_costmaps_service()
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
            self.announce_delivery_result(
                during_playback=self.arrive_and_leave_lab_window)

            if rospy.is_shutdown():
                return
            if not self.enable_dual_car:
                self.deliveries_since_board1_recognition += 1
                if self.deliveries_since_board1_recognition >= 2 or not self.pending_board1_tasks:
                    # 每完成两项即作废旧缓存；不足两项时提前重新识别。
                    self.pending_board1_tasks = []
                    rospy.loginfo("本批已完成 %s 项配送，先返回原点，再前往板一重新识别",
                                  self.deliveries_since_board1_recognition)
                else:
                    rospy.loginfo("本项配送完成，先返回原点，再经过板一执行本批第二项任务")
                self.count = STATE_GO_BACK_HOME
                self.pub_nav_state.publish(self.count)
                return

            # [DUAL-MOD-STATE15] 进入状态15的瞬间释放令牌。
            # 也就是本车完成化验窗口停留和播报，准备返回起点时，才允许另一辆车出发。
            self.count = STATE_GO_BACK_HOME
            self.pub_nav_state.publish(self.count)
            self.release_next_car_if_needed("enter_state_15_go_back_home")
        else:
            rospy.logerr("前往化验窗口失败，重新尝试")
            return

    def stop_at_home(self, config):
        """原点到达后持续发零速度，等待新鲜里程计确认车辆已经停稳。"""
        from nav_msgs.msg import Odometry
        deadline = time.time() + float(config["stop_timeout_sec"])
        stationary_since = None
        last_stamp = 0.0
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(Twist())
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            try:
                odom = rospy.wait_for_message(config["odom_topic"], Odometry,
                                              timeout=min(0.5, remaining))
            except rospy.ROSException:
                stationary_since = None
                continue
            stamp = odom.header.stamp
            stamp_sec = stamp.to_sec()
            age = (rospy.Time.now() - stamp).to_sec()
            velocity = odom.twist.twist
            speed = (velocity.linear.x ** 2 + velocity.linear.y ** 2) ** 0.5
            if (stamp_sec > last_stamp and 0 <= age <= float(config["odom_max_age_sec"])
                    and speed <= float(config["stop_linear_speed"])
                    and abs(velocity.angular.z) <= float(config["stop_angular_speed"])):
                now = time.time()
                if stationary_since is None:
                    stationary_since = now
                if now - stationary_since >= float(config["stop_hold_sec"]):
                    rospy.loginfo("[HOME-ARRIVAL] 原点停车已确认")
                    return True
            else:
                stationary_since = None
            last_stamp = max(last_stamp, stamp_sec)
        self.cmd_vel_pub.publish(Twist())
        if not rospy.is_shutdown():
            rospy.logerr("[HOME-ARRIVAL] 未能在规定时间内确认停车，停止任务")
            rospy.signal_shutdown("Home stop confirmation failed")
        return False

    def move_to_home(self, pass_through=False):
        """回原点时临时放宽容差；可连续出发时提前交接，否则保持停车确认。"""
        config = NAV_CFG["home_arrival"]
        client = None
        original_tolerances = None
        reached = False
        failure = None
        try:
            timeout = float(config["config_timeout_sec"])
            client = self.arrival_guard.configuration_client(config["planner_namespace"], timeout)
            original = client.get_configuration(timeout=timeout)
            descriptions = client.get_parameter_descriptions(timeout=timeout)
            if original is None or descriptions is None:
                raise RuntimeError("TEB configuration is unavailable")
            names = ("xy_goal_tolerance", "yaw_goal_tolerance")
            if pass_through:
                names += ("free_goal_vel",)
            original_tolerances = dict((name, original[name]) for name in names)
            tolerances = {
                "xy_goal_tolerance": max(float(original_tolerances["xy_goal_tolerance"]),
                                         float(config["xy_goal_tolerance"])),
                "yaw_goal_tolerance": max(float(original_tolerances["yaw_goal_tolerance"]),
                                          radians(float(config["yaw_goal_tolerance_deg"]))),
            }
            if pass_through:
                tolerances["free_goal_vel"] = bool(config["free_goal_vel"])
            applied = client.update_configuration(dict(tolerances))
            if any(abs(float(applied[name]) - tolerances[name]) > 1e-6 for name in names):
                raise RuntimeError("Home goal tolerances were not applied")
            rospy.loginfo("[HOME-ARRIVAL] 原点临时设置: %s，经过模式=%s", tolerances, pass_through)
            if not rospy.is_shutdown():
                reached = self.move(self.make_goal(self.waypoints[7]),
                                    transition=config if pass_through else None)
                if reached and not pass_through:
                    reached = self.stop_at_home(config)
        except Exception as exc:
            if not rospy.is_shutdown():
                rospy.logerr("[HOME-ARRIVAL] 返回原点失败: %s", exc)
                failure = "Home arrival failed"
        finally:
            if client is not None:
                try:
                    if original_tolerances is not None:
                        restored = client.update_configuration(dict(original_tolerances))
                        if any(abs(float(restored[name]) - float(value)) > 1e-6
                               for name, value in original_tolerances.items()):
                            raise RuntimeError("Original goal tolerances were not restored")
                        rospy.loginfo("[HOME-ARRIVAL] 已恢复原到达容差: %s", original_tolerances)
                except Exception as exc:
                    rospy.logerr("[HOME-ARRIVAL] 恢复到达容差失败: %s", exc)
                    failure = "Home goal tolerance restore failed"
        if failure is not None and not rospy.is_shutdown():
            rospy.signal_shutdown(failure)
        return reached and not rospy.is_shutdown()

    def handle_go_back_home(self):
        """状态15：连续任务经过原点直接前往板一，等待令牌或结束时停车。"""
        rospy.loginfo("送样完成，正在返回原点！")
        self.pub_nav_state.publish(STATE_GO_BACK_HOME)

        finishing = (self.enable_dual_car and self.max_rounds
                     and self.completed_rounds + 1 >= self.max_rounds)
        can_continue = not self.enable_dual_car or (self.peer_done_pending and not finishing)
        pass_through = bool(NAV_CFG["home_arrival"]["pass_through"]) and can_continue

        if self.move_to_home(pass_through=pass_through) is True:
            rospy.loginfo("已经过原点，准备前往识别板一！")
            if not pass_through:
                self.clear_costmaps_service()
                rospy.sleep(NAV_CFG["home_arrive_sleep_sec"])

            # 双车保持原轮数结束判断；单车只增加回原点路线，保持连续配送。
            if self.enable_dual_car:
                self.completed_rounds += 1
                if self.max_rounds and self.completed_rounds >= self.max_rounds:
                    rospy.loginfo("FULL RUN COMPLETE: %s round(s), stopped at home", self.completed_rounds)
                    self.export_recognition_records()
                    rospy.signal_shutdown("Full run complete at home")
                    return

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
            if pass_through and self.count == STATE_GO_TO_BOARD1 and not rospy.is_shutdown():
                # 同一次状态处理内交接目标，避免主循环等待造成原点停顿。
                self.handle_go_to_board1()
        else:
            rospy.logerr("返回原点失败，重新尝试返回原点")
            return


    def handle_wait_turn(self):
        """[DUAL-ADD] 状态 8：等待另一辆车释放令牌。"""
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

        # 本车已经把本轮出发令牌交给对方，但仍继续完成当前配送和返回起点。
        self.turn_released_this_round = True
        self.have_turn = False

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

        from pharmacy_iscc.board1_detector import select_board1_from_all_text
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
        """把识别板一选择结果写入状态机变量并记录任务。"""
        self.ram_result = list(selected_msg)
        self.windows_C = self.ram_result[0]
        self.windows_A = self.ram_result[1]
        self.windows_B = self.ram_result[2]
        self.windows_count = self.ram_result[3]
        self.windows_1234 = self.ram_result[4]

        self.pickup_C_done = False
        self.pickup_A_done = False
        self.pickup_B_done = False

        if not self.board1_received:
            self.recognition_records.record(u'识别板一', u'执行任务', {
                u'取样窗口': [name for name, enabled in
                             [('A', self.windows_A), ('B', self.windows_B),
                              ('C', self.windows_C)] if enabled == 1],
                u'样本数量': int(self.windows_count),
                u'送样窗口': int(self.windows_1234) + 1,
            })
        self.board1_received = True

    def make_goal(self, pose):
        """根据 Pose 创建 MoveBaseGoal。"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return goal

    def wait_for_board1_result(self, start_time=None):
        """3秒无结果后退一次；再3秒无结果回识别点一次，仍保留总超时。"""
        rospy.loginfo("等待识别板一识别结果...")
        timeout = float(NAV_CFG["board1_wait_timeout_sec"])
        start_time = time.time() if start_time is None else start_time
        recovery = NAV_CFG["board1_recovery"]
        stage = 0
        stage_deadline = start_time + float(recovery["recognition_wait_sec"])
        rate = rospy.Rate(NAV_CFG["wait_result_rate_hz"])
        while not rospy.is_shutdown() and not self.board1_received:
            if timeout > 0 and (time.time() - start_time) > timeout:
                rospy.logerr("识别板一等待超时 %.1f 秒，返回失败", timeout)
                rospy.signal_shutdown("Board1 recognition timed out")
                return False
            if recovery["enabled"] and stage < 2 and time.time() >= stage_deadline:
                if self.board1_received:
                    break
                if stage == 0:
                    rospy.logwarn("[BOARD1-RECOVERY] %.1f秒无任务，沿当前车身方向后退%.0f厘米",
                                  recovery["recognition_wait_sec"], 100.0 * recovery["reverse_distance_m"])
                    if self.move_base.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE,
                                                     GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                        rospy.logerr("[BOARD1-RECOVERY] 导航目标仍活动，不叠加后退控制")
                        rospy.signal_shutdown("Board1 reverse requires inactive navigation")
                        return False
                    try:
                        distance = self.arrival_guard.short_reverse(
                            self.cmd_vel_pub, recovery,
                            force_reverse=recovery.get("force_reverse", False))
                    except Exception as exc:
                        rospy.logerr("[BOARD1-RECOVERY] 后退中止: %s", exc)
                        rospy.signal_shutdown("Board1 reverse failed")
                        return False
                    if distance is None:
                        rospy.logwarn("[BOARD1-RECOVERY] 本次未执行后退，原地再等待识别")
                    else:
                        rospy.loginfo("[BOARD1-RECOVERY] 后退结束，里程计距离=%.3f米，重新等待识别", distance)
                    # 保持状态10：视觉继续识别。运动期间收到任务，也必须等本次停车后再执行。
                    stage = 1
                    stage_deadline = time.time() + float(recovery["recognition_wait_sec"])
                else:
                    rospy.logwarn("[BOARD1-RECOVERY] 后退后仍无任务，重新导航到板一识别点")
                    self.count = STATE_GO_TO_BOARD1
                    self.pub_nav_state.publish(self.count)
                    if not self.move(self.make_goal(self.waypoints[9])):
                        return False
                    self.count = STATE_BOARD1_RECOGNIZING
                    self.pub_nav_state.publish(self.count)
                    # 状态9->10让视觉清空旧历史、重新允许发布；不丢弃已收到的有效任务。
                    stage = 2
                    rospy.loginfo("[BOARD1-RECOVERY] 已回板一识别点，使用剩余总等待时间重试")
                continue
            rospy.loginfo_throttle(1.0, "识别板一尚未识别成功，继续等待...")
            rate.sleep()
        return self.board1_received and not rospy.is_shutdown()

    def wait_for_board2_result(self):
        """从最终板二点到达起等待；超时按空闲继续，配置0时不设截止时间。"""
        rospy.loginfo("等待识别板二识别结果...")
        timeout = float(NAV_CFG["board2_wait_timeout_sec"])
        rate = rospy.Rate(NAV_CFG["wait_result_rate_hz"])
        while not rospy.is_shutdown() and not self.board2_received:
            deadline = self.board2_recognition_deadline
            if deadline is not None and time.time() >= deadline:
                if self.accept_board2_result(0, 0, timed_out=True):
                    rospy.logwarn("识别板二到点后 %.1f 秒无有效结果，按空闲处理", timeout)
                return self.board2_received and not rospy.is_shutdown()
            rospy.loginfo_throttle(1.0, "识别板二尚未识别成功，继续等待...")
            rate.sleep()
        return self.board2_received and not rospy.is_shutdown()

    def move(self, goal, transition=None):
        """普通目标等待完成；过渡目标接近后直接交接下一目标，不主动停车。"""
        requested = target_pose(goal.target_pose.pose.position.x,
                                goal.target_pose.pose.position.y,
                                quaternion_yaw([goal.target_pose.pose.orientation.x,
                                                goal.target_pose.pose.orientation.y,
                                                goal.target_pose.pose.orientation.z,
                                                goal.target_pose.pose.orientation.w])*180.0/pi)
        try:
            grid, unused_identity = self.arrival_guard.grid()
            validate_target(grid, requested)
            self.arrival_guard.pose()
        except Exception as exc:
            rospy.logerr("Navigation preflight rejected: %s", exc)
            rospy.signal_shutdown("Navigation preflight rejected")
            return False
        self.move_base.send_goal(goal)
        if transition is None:
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(NAV_CFG["move_timeout_sec"]))
        else:
            finished_within_time = False
            deadline = time.time() + float(NAV_CFG["move_timeout_sec"])
            while not rospy.is_shutdown() and time.time() < deadline:
                interval = min(1.0 / float(transition["poll_rate_hz"]), deadline - time.time())
                if interval <= 0:
                    break
                if self.move_base.wait_for_result(rospy.Duration(interval)):
                    finished_within_time = True
                    break
                try:
                    pose = self.arrival_guard.pose()
                except Exception as exc:
                    rospy.logerr("过渡点交接定位不可用: %s", exc)
                    self.move_base.cancel_goal()
                    rospy.signal_shutdown("Transition localization unavailable")
                    return False
                distance = ((pose["x"] - requested["x"]) ** 2 +
                            (pose["y"] - requested["y"]) ** 2) ** 0.5
                yaw_error = abs((pose["yaw_deg"] - requested["yaw_deg"] + 180.0) % 360.0 - 180.0)
                if (distance <= float(transition["switch_distance_m"])
                        and yaw_error <= float(transition["switch_yaw_tolerance_deg"])):
                    rospy.loginfo("[TRANSITION] 接近过渡点，直接交接下一目标: distance=%.3f yaw_error=%.1f",
                                  distance, yaw_error)
                    # 保持当前目标活动，由下一次 send_goal 接替；不 cancel、不发零速度。
                    return True

        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.signal_shutdown("Navigation timed out; full run stopped")
            rospy.logwarn("Timed out achieving goal，清除代价地图后准备重试")
            try:
                self.clear_costmaps_service()
            except Exception as e:
                rospy.logwarn("清除代价地图失败: %s", e)
            rospy.sleep(NAV_CFG["post_clear_costmap_sleep_sec"])
            return False

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("move_base goal succeeded; continuing to the next task step")
            return True

        rospy.signal_shutdown("Navigation failed; full run stopped")
        rospy.logwarn("导航目标失败，move_base state=%s，清除代价地图后准备重试", state)
        try:
            self.clear_costmaps_service()
        except Exception as e:
            rospy.logwarn("清除代价地图失败: %s", e)

        rospy.sleep(NAV_CFG["post_clear_costmap_sleep_sec"])
        return False

    def arrive_and_leave_pickup_window(self, window_name, window_code=None):
        """最后一个取样窗口先播报，再并行完成原到达/停留流程。"""
        if window_code is not None:
            self.announce_pickup_result(
                window_code,
                during_playback=lambda: self.arrive_and_leave_pickup_window(
                    window_name))
            return
        self.clear_costmaps_service()
        rospy.sleep(NAV_CFG["pickup_task_hold_sec"])

        # 标记该窗口已经完成，避免导航失败重试时重复前往
        if window_name == "C":
            self.pickup_C_done = True
        elif window_name == "A":
            self.pickup_A_done = True
        elif window_name == "B":
            self.pickup_B_done = True

        rospy.loginfo("%s 窗口取样流程已完成，已记录完成状态", window_name)

    def arrive_and_leave_lab_window(self):
        """播报已启动后，执行化验窗口清图和停留。"""
        real_window_str = str(self.windows_1234 + 1)
        rospy.loginfo("已经到达化验室" + real_window_str + "窗口！")
        self.clear_costmaps_service()
        rospy.sleep(NAV_CFG["lab_task_hold_sec"])

    def announce_pickup_result(self, window_code, during_playback=None):
        """先启动取样组合播报，再执行并行到达动作，等待两者结束。"""
        lab_info = LAB_INFO[self.windows_1234]
        message = u"取到%s窗口中的%s样本" % (
            WINDOW_LOG_NAME[window_code], lab_info["sample_name"])

        def on_started():
            rospy.loginfo(message.encode("utf-8") if sys.version_info[0] < 3 else message)
            if during_playback is not None:
                during_playback()

        play_audio("%s%s.wav" % (lab_info["pickup_audio_prefix"], window_code),
                   during_playback=on_started)

    def announce_delivery_result(self, during_playback=None):
        """先启动送达播报，再执行并行到达动作，等待两者结束。"""
        lab_info = LAB_INFO[self.windows_1234]
        message = u"到达%s，样本数为%d" % (
            lab_info["lab_name"], self.windows_count)

        def on_started():
            rospy.loginfo(message.encode("utf-8") if sys.version_info[0] < 3 else message)
            if during_playback is not None:
                during_playback()

        play_audio("%s%d.wav" % (lab_info["delivery_audio_prefix"], self.windows_count),
                   during_playback=on_started)

    def detect_board1_result(self, msg):
        """识别板一回调：只在状态 10 时更新任务参数。"""
        if self.count == STATE_BOARD1_RECOGNIZING and not self.board1_received:
            data = list(msg.data)
            if not data or len(data) % 6 != 0 or len(data) > 24:
                rospy.logwarn("板一任务消息格式错误，应为每任务六项: %s", data)
                return
            tasks = [data[index:index + 6] for index in range(0, len(data), 6)]
            destinations = set()
            for task in tasks:
                flags = task[:3]
                if (any(flag not in (0, 1) for flag in flags)
                        or not 1 <= task[3] <= 3 or sum(flags) != task[3]
                        or not 0 <= task[4] <= 3 or task[4] in destinations):
                    rospy.logwarn("板一任务参数或窗口编号非法: %s", data)
                    return
                destinations.add(task[4])
            # 双车仍只执行首项，原有 all_text 共享与对车选择逻辑不变。
            self.pending_board1_tasks = [] if self.enable_dual_car else tasks[1:]
            self.deliveries_since_board1_recognition = 0
            self.ram_result = tasks[0]
            rospy.loginfo("识别到二维码并已经选择了最优方案：%s....正在更新参数", msg.data)
            rospy.logwarn("self.ram_result: %s", self.ram_result)

            self.apply_board1_selected_msg(self.ram_result)

    def accept_board2_result(self, state, wait_time, timed_out=False):
        """每次访问只提交一次；真实结果与超时默认结果互斥，迟到结果不覆盖。"""
        with self.board2_result_lock:
            if self.count != STATE_BOARD2_RECOGNIZING or self.board2_received:
                return False
            if (not timed_out and self.board2_recognition_deadline is not None
                    and time.time() >= self.board2_recognition_deadline):
                return False
            self.board2_state = int(state)
            self.board2_wait_time = int(wait_time)
            self.board2_result = [int(state), int(wait_time)]
            self.board2_received = True
        content = {u'状态': u'空闲' if state == 0 else u'忙碌', u'等待秒数': int(wait_time)}
        kind = u'识别结果'
        if timed_out:
            kind = u'超时默认'
            content[u'原因'] = u'到点后未识别到板二'
            content[u'超时秒数'] = float(NAV_CFG["board2_wait_timeout_sec"])
        self.recognition_records.record(u'识别板二', kind, content)
        return True

    def detect_board2_result(self, msg):
        """识别板二回调：状态13内、截止时间前只接收首个有效结果。"""
        if self.count != STATE_BOARD2_RECOGNIZING or self.board2_received:
            return
        data = list(msg.data)
        if len(data) < 2:
            rospy.logwarn("识别板2返回数据长度错误: %s", data)
            return
        state, wait_time = int(data[0]), int(data[1])
        if state == 0:
            self.accept_board2_result(0, 0)
        elif state == 1:
            if wait_time < 5:
                rospy.logwarn("识别板2等待时间过小: %d，已修正为5秒", wait_time)
                wait_time = 5
            elif wait_time > 10:
                rospy.logwarn("识别板2等待时间过大: %d，已修正为10秒", wait_time)
                wait_time = 10
            self.accept_board2_result(1, wait_time)

    def export_recognition_records(self):
        if not hasattr(self, 'recognition_records'):
            return
        try:
            path = self.recognition_records.export()
            rospy.loginfo('Recognition Excel exported: %s', path)
        except Exception as exc:
            rospy.logerr('Recognition Excel export failed: %s', exc)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        try:
            if (hasattr(self, "move_base")
                    and self.move_base.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE,
                                                       GoalStatus.PREEMPTING, GoalStatus.RECALLING)):
                self.move_base.cancel_goal()
                self.move_base.wait_for_result(rospy.Duration(3.0))
            if hasattr(self, "cmd_vel_pub"):
                self.cmd_vel_pub.publish(Twist())
        finally:
            if hasattr(self, 'arrival_guard'):
                self.arrival_guard.close_configuration_clients()
            self.export_recognition_records()


if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
