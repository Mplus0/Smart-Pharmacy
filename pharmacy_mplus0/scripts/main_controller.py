#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""主控状态机 ROS 节点。

本节点是比赛业务的核心调度器，负责串联所有模块：

  识别板一 → 选最优二维码 → 依次取体检窗口样本
  → 识别板二播报/等待 → 化验窗口投递 → 回起点 → 下一轮

状态机（11 个状态）:
  INIT         上电初始化
  GOTO_BOARD1  前往识别板一
  AT_BOARD1    等待二维码识别结果
  GOTO_EXAM    前往下一个体检窗口
  AT_EXAM      停在体检窗口、取样、播报
  GOTO_BOARD2  前往识别板二
  AT_BOARD2    等待化验区状态识别
  PASS_BOARD2  空闲快速通过 / 忙碌等待
  GOTO_LAB     前往化验窗口
  AT_LAB       停在化验窗口、播报、投递
  DONE_ROUND   本轮结束 → 回起点 → 下一轮

架构原则:
  - 主控只做状态调度，不直接处理图像、TCP、播报文本拼接。
  - 导航走 NavigationClient，状态发布走 CompetitionIO，
    样本记录走 SampleStore，任务选择走 TaskPlanner。
  - 所有可调参数从 strategy.yaml 读取，现场只改 YAML。
"""

from __future__ import print_function

import json
import os
import time

import rospy
import rospkg
import yaml
from std_msgs.msg import Int32MultiArray, String, Empty
from geometry_msgs.msg import Twist

from pharmacy_mplus0.log_utils import (
    loginfo, logwarn, logerr, set_node_name,
)
from pharmacy_mplus0.constants import (
    STATE_INIT,
    STATE_GOTO_BOARD1,
    STATE_AT_BOARD1,
    STATE_GOTO_EXAM,
    STATE_AT_EXAM,
    STATE_GOTO_BOARD2,
    STATE_AT_BOARD2,
    STATE_PASS_BOARD2,
    STATE_GOTO_LAB,
    STATE_AT_LAB,
    STATE_DONE_ROUND,
    TOPIC_CAM_RETURN,
    TOPIC_BOARD1_DETECTIONS,
    TOPIC_CV1_RESULT,
    TOPIC_BOARD2_STATUS,
    TOPIC_RESET_DETECTION,
    TOPIC_CMD_VEL,
    TASK_ROAD,
)
from pharmacy_mplus0.navigation_client import NavigationClient
from pharmacy_mplus0.competition_io import CompetitionIO
from pharmacy_mplus0.sample_store import SampleStore
from pharmacy_mplus0.task_planner import TaskPlanner
from pharmacy_mplus0.dual_car_tcp import DualCarTcpBridge
from pharmacy_mplus0.models import (
    Board1Detection,
    Board2Status,
    make_board1_detection,
    board2_status_from_text,
)

# 默认配置文件名。
_STRATEGY_YAML = "strategy.yaml"


class _TcpLoggerAdapter(object):
    """将 DualCarTcpBridge 的日志调用转发到 log_utils，确保 TCP 层日志可见。"""
    def info(self, msg, *args):
        loginfo(msg, *args)

    def warn(self, msg, *args):
        logwarn(msg, *args)

    def warning(self, msg, *args):
        logwarn(msg, *args)

    def error(self, msg, *args):
        logerr(msg, *args)


class MainController(object):
    """主控状态机。"""

    def __init__(self):
        rospy.init_node("main_controller", anonymous=False)
        set_node_name("main_controller")
        rospy.on_shutdown(self.shutdown)

        # ---- 加载策略配置 --------------------------------------------
        strategy = self._load_strategy_config()

        # 停留时间。
        dwell = strategy.get("dwell", {})
        self._exam_dwell = dwell.get("exam_seconds", 1.5)
        self._lab_dwell = dwell.get("lab_seconds", 1.5)
        self._board1_settle = dwell.get("board1_settle_seconds", 1.0)

        # 超时与重试。
        timeouts = strategy.get("timeouts", {})
        self._board1_timeout = timeouts.get("board1_wait_seconds", 15.0)
        self._board1_max_retry = timeouts.get("board1_max_retry", 3)
        self._board2_timeout = timeouts.get("board2_wait_seconds", 10.0)
        self._nav_timeout = timeouts.get("nav_default_seconds", 40.0)
        self._max_consec_fails = timeouts.get(
            "max_consecutive_fail_rounds", 5
        )

        # 轮次策略。
        rounds = strategy.get("rounds", {})
        self._return_to_start = rounds.get("round_return_to_start", True)

        # ---- 双车协作配置 --------------------------------------------
        self._dual_car_enabled = rospy.get_param(
            "~dual_car_enabled",
            strategy.get("dual_car_enabled", False),
        )
        self._car_id = str(rospy.get_param("~car_id", "1"))
        start_first_id = str(
            strategy.get("dual_car_start_first_car_id", "1")
        )

        self._dual_waiting_at_start = False
        self._dual_start_allowed = True
        self._dual_allow_sent_this_round = False

        if self._dual_car_enabled:
            if self._car_id == start_first_id:
                self._dual_start_allowed = True
                self._dual_waiting_at_start = False
            else:
                self._dual_start_allowed = False
                self._dual_waiting_at_start = True

        # ---- 远程任务共享（默认关闭，不影响单车和基础双车流程）-------
        self._dual_remote_task_enabled = rospy.get_param(
            "~dual_car_remote_task_enabled", None
        )
        if self._dual_remote_task_enabled is None:
            self._dual_remote_task_enabled = strategy.get(
                "dual_car_remote_task_enabled", False
            )
        self._dual_assigned_remote_task = None
        self._dual_remote_task_sent_this_round = False

        # ---- 双车 TCP 通信配置 ----------------------------------------
        self._dual_car_peer_ip = rospy.get_param(
            "~dual_car_peer_ip", ""
        )
        self._dual_car_listen_ip = rospy.get_param(
            "~dual_car_listen_ip", "0.0.0.0"
        )
        self._dual_car_listen_port = int(rospy.get_param(
            "~dual_car_listen_port", 9001
        ))
        self._dual_car_peer_port = int(rospy.get_param(
            "~dual_car_peer_port", 9001
        ))

        # ---- 双车 TCP 通信桥接 ----------------------------------------
        self._dual_tcp_bridge = None
        if self._dual_car_enabled:
            if self._dual_car_peer_ip:
                self._dual_tcp_bridge = DualCarTcpBridge(
                    car_id=self._car_id,
                    peer_id=self._dual_peer_id(),
                    listen_ip=self._dual_car_listen_ip,
                    listen_port=self._dual_car_listen_port,
                    peer_ip=self._dual_car_peer_ip,
                    peer_port=self._dual_car_peer_port,
                    on_message=self._dual_handle_tcp_message,
                    logger=_TcpLoggerAdapter(),
                )
                self._dual_tcp_bridge.start()
            else:
                logwarn(
                    "[Main] dual_car_peer_ip 为空，"
                    "双车 TCP bridge 不启动，无法与对车通信"
                )

        # 访问顺序和任务优先级传给 TaskPlanner。
        visit_order = strategy.get("visit_order", None)
        prefer_more = strategy.get("task_priority", {}).get(
            "prefer_more_samples", True
        )

        # ---- 干跑模式 ------------------------------------------------
        self._dry_run = rospy.get_param("~dry_run", False)
        if self._dry_run:
            loginfo("[Main] 干跑模式已启用：跳过所有真实导航，模拟导航成功")

        # ---- 初始化子模块 --------------------------------------------
        self._nav = NavigationClient(dry_run=self._dry_run)
        self._io = CompetitionIO(car_id=self._car_id)
        self._store = SampleStore()
        self._planner = TaskPlanner(visit_order=visit_order)

        # ---- ROS 发布器 ----------------------------------------------
        # 通知识别节点复位。
        self._reset_pub = rospy.Publisher(
            TOPIC_RESET_DETECTION, String, queue_size=5
        )
        # 紧急停车。
        self._cmd_vel_pub = rospy.Publisher(
            TOPIC_CMD_VEL, Twist, queue_size=5
        )

        # ---- 订阅识别结果 --------------------------------------------
        # 兼容旧格式 /cam_return。
        rospy.Subscriber(
            TOPIC_CAM_RETURN, Int32MultiArray,
            self._cb_cam_return, queue_size=5,
        )
        # 新格式 /board1_detections (JSON)。
        rospy.Subscriber(
            TOPIC_BOARD1_DETECTIONS, String,
            self._cb_board1_detections, queue_size=5,
        )
        # 识别板二。
        rospy.Subscriber(
            TOPIC_CV1_RESULT, String,
            self._cb_cv1_result, queue_size=5,
        )
        rospy.Subscriber(
            TOPIC_BOARD2_STATUS, String,
            self._cb_board2_status, queue_size=5,
        )

        # ---- 运行状态 ------------------------------------------------
        self._state = STATE_INIT
        self._round_index = 0
        self._consecutive_fails = 0

        # 识别板一结果缓存（仅在 AT_BOARD1 时接收有效）。
        self._board1_detections = []

        # 识别板二结果缓存（仅在 AT_BOARD2 时接收有效）。
        self._board2_wait = None

        # 本轮任务上下文。
        self._round_plan = None
        self._exam_visit_idx = 0

        # 双车协作：对车占用的方框索引，None 表示无占用。
        self._peer_occupied_box = None

        self._print_banner()
        loginfo("[Main] 主控初始化完成")

    # ---- 回调 -------------------------------------------------------

    def _cb_cam_return(self, msg):
        """接收 /cam_return（旧格式），只在 AT_BOARD1 时缓存。"""
        if self._state != STATE_AT_BOARD1:
            return
        try:
            detection = self._planner.from_cam_return(msg.data)
            self._board1_detections.append(detection)
            loginfo("[Main] 收到旧格式 /cam_return: code=%s box=%d",
                    detection.code, detection.box_index)
        except ValueError as exc:
            logwarn("[Main] /cam_return 解析失败: %s", exc)

    def _cb_board1_detections(self, msg):
        """接收 /board1_detections (新 JSON 格式)，只在 AT_BOARD1 时缓存。"""
        if self._state != STATE_AT_BOARD1:
            return
        try:
            items = json.loads(msg.data)
            for item in items:
                detection = make_board1_detection(
                    item["code"], item["box_index"]
                )
                self._board1_detections.append(detection)
            loginfo("[Main] 收到新格式 /board1_detections: %d 个结果",
                    len(items))
        except (ValueError, KeyError, TypeError) as exc:
            logwarn("[Main] /board1_detections 解析失败: %s", exc)

    def _cb_cv1_result(self, msg):
        """接收 /cv1_result (String "WAIT-N")，只在 AT_BOARD2 时缓存。"""
        if self._state != STATE_AT_BOARD2:
            return
        try:
            status = board2_status_from_text(msg.data)
            self._board2_wait = status.wait_seconds
        except ValueError:
            pass

    def _cb_board2_status(self, msg):
        """接收 /board2_status (JSON)，只在 AT_BOARD2 时缓存。"""
        if self._state != STATE_AT_BOARD2:
            return
        try:
            data = json.loads(msg.data)
            wait_sec = int(data.get("wait_seconds", 0))
            if wait_sec == 0 or 5 <= wait_sec <= 10:
                self._board2_wait = wait_sec
        except (ValueError, KeyError, TypeError):
            pass

    # ---- 双车协作回调 -----------------------------------------------

    def _dual_peer_id(self):
        """返回对车编号：1→2, 2→1。"""
        return "2" if self._car_id == "1" else "1"

    def _dual_handle_allow_start_signal(self, target_id, from_car=None):
        """处理 ALLOW_START 放行信号（由 TCP 回调触发）。"""
        if target_id != self._car_id:
            loginfo(
                "[DualCar] ALLOW_START for car %s ignored (本车是 car %s)",
                target_id, self._car_id,
            )
            return
        if not self._dual_car_enabled:
            return
        if not self._dual_waiting_at_start:
            loginfo(
                "[DualCar] ALLOW_START ignored — 本车未在等待状态"
            )
            return
        self._dual_start_allowed = True
        self._dual_waiting_at_start = False
        src_info = " from car %s" % from_car if from_car else ""
        loginfo(
            "[DualCar] ALLOW_START accepted for car %s%s, "
            "将从等待状态切换到 GOTO_BOARD1",
            target_id, src_info,
        )

    def _dual_handle_remote_task_data(self, data):
        """处理远程任务数据（由 TCP 回调触发）。

        当收到合法远程任务时：
        - 缓存任务到 _dual_assigned_remote_task
        - 同时执行放行逻辑
        """
        if not self._dual_car_enabled:
            return

        msg_type = data.get("type", "")
        target_car = str(data.get("target_car", ""))
        from_car = str(data.get("from_car", ""))

        # 不是发给自己的，忽略。
        if target_car != self._car_id:
            return
        # 自己发出的，忽略。
        if from_car == self._car_id:
            return

        if msg_type != "remote_task":
            logwarn("[DualCar] unknown message type: %s", msg_type)
            return

        # 提取远程任务。
        task = data.get("task")
        if (
            task
            and self._dual_remote_task_enabled
            and isinstance(task, dict)
        ):
            code = task.get("code", "")
            box = task.get("box")
            lab_window = task.get("lab_window", "")
            sample_count = task.get("sample_count")

            # 基本字段校验（不再校验 stamp 是否过期）。
            code_ok = bool(code and str(code).strip())
            box_ok = box is not None and isinstance(box, int)
            lab_ok = bool(lab_window and str(lab_window).strip() in ("1", "2", "3", "4"))

            if not (code_ok and box_ok and lab_ok):
                logwarn(
                    "[DualCar] remote task fields invalid (code=%s box=%s lab=%s), "
                    "fallback to normal board1 flow",
                    code, box, lab_window,
                )
                self._dual_assigned_remote_task = None
            else:
                # 如果已经有远程任务且本轮已开始执行，不再覆盖。
                if (self._dual_assigned_remote_task is not None
                        and self._state not in (STATE_INIT, STATE_GOTO_BOARD1)):
                    loginfo(
                        "[DualCar] already executing a task, ignoring new remote task"
                    )
                    return

                stamp = data.get("stamp", 0)
                self._dual_assigned_remote_task = {
                    "from_car": from_car,
                    "code": str(code).strip(),
                    "box": int(box),
                    "lab_window": str(lab_window).strip(),
                    "sample_count": (int(sample_count)
                                     if sample_count is not None
                                     else len(str(code).strip())),
                    "stamp": stamp,
                }
                loginfo(
                    "[DualCar] received remote task from car %s: "
                    "code=%s box=%d lab_window=%s sample_count=%s, skip board1",
                    from_car, code, int(box), lab_window,
                    self._dual_assigned_remote_task["sample_count"],
                )
        else:
            # 远程任务功能未启用或无 task 字段，仅当作普通放行信号。
            if self._dual_remote_task_enabled:
                logwarn("[DualCar] JSON missing valid task, treated as plain ALLOW_START")
            self._dual_assigned_remote_task = None

        # 无论是否有远程任务，都执行放行逻辑。
        self._dual_start_allowed = True
        self._dual_waiting_at_start = False
        loginfo("[DualCar] received ALLOW_START from car %s via %s", from_car, msg_type)

    def _dual_handle_tcp_message(self, payload):
        """TCP 消息分发：将收到的 JSON 路由到对应处理函数。"""
        msg_type = payload.get("type", "")
        from_car = str(payload.get("from_car", "?"))
        loginfo(
            "[DualCar] TCP recv: type=%s from_car=%s",
            msg_type, from_car,
        )
        if msg_type == "allow_start":
            target_car = str(payload.get("target_car", ""))
            self._dual_handle_allow_start_signal(target_car, from_car)
        elif msg_type == "remote_task":
            self._dual_handle_remote_task_data(payload)
        elif msg_type == "ack":
            loginfo(
                "[DualCar] received ACK: ack_type=%s status=%s from car %s",
                payload.get("ack_type", ""),
                payload.get("status", ""),
                payload.get("from_car", ""),
            )
        elif msg_type == "qr_task":
            self._dual_handle_peer_qr_task_payload(payload)
        elif msg_type == "qr_task_clear":
            self._dual_handle_peer_qr_task_clear(payload)
        else:
            logwarn("[DualCar] unknown TCP message type: %s", msg_type)

    def _dual_handle_peer_qr_task_payload(self, payload):
        """处理 TCP qr_task 消息：更新对车占用的 box_index。"""
        from_car = str(payload.get("from_car", ""))
        if from_car == self._car_id:
            return
        code = str(payload.get("code", ""))
        box = payload.get("box")
        if isinstance(box, int) and 0 <= box <= 3:
            self._peer_occupied_box = box
            loginfo("[Main] 对车占用(TCP): box=%d (任务 %s from car %s)",
                    box, code, from_car)
        else:
            logwarn("[DualCar] qr_task has invalid box: %s", box)

    def _dual_handle_peer_qr_task_clear(self, payload):
        """处理 TCP qr_task_clear 消息：清空对车占用。"""
        from_car = str(payload.get("from_car", ""))
        if from_car == self._car_id:
            return
        self._peer_occupied_box = None

    def _dual_publish_allow_peer_start(self):
        """完成配送后放行对车：通过 TCP 发送 ALLOW_START。

        仅在双车模式启用、本轮尚未发送过时生效。
        发送后本车 _dual_start_allowed 置 False，下一轮需等待对车放行。
        """
        if not self._dual_car_enabled:
            return
        if self._dual_allow_sent_this_round:
            return
        peer_id = self._dual_peer_id()

        sent_ok = False
        if self._dual_tcp_bridge is not None:
            payload = {
                "type": "allow_start",
                "from_car": self._car_id,
                "target_car": peer_id,
                "stamp": time.time(),
            }
            sent_ok = self._dual_tcp_bridge.send(payload)
        else:
            logwarn(
                "[DualCar] 无法发送 ALLOW_START:%s — TCP bridge 未启动 "
                "(dual_car_peer_ip 未配置或为空)",
                peer_id,
            )

        if sent_ok:
            loginfo(
                "[DualCar] send ALLOW_START:%s success via TCP, "
                "peer can start while this car returns",
                peer_id,
            )
        elif self._dual_tcp_bridge is not None:
            logwarn(
                "[DualCar] send ALLOW_START:%s failed — TCP 未连接, "
                "消息已丢弃",
                peer_id,
            )

        self._dual_allow_sent_this_round = True
        self._dual_start_allowed = False
        self._dual_waiting_at_start = False

    def _dual_publish_remote_task_for_peer(self, task_detection):
        """将剩余任务通过 TCP 发送给对车。

        参数:
            task_detection: Board1Detection，要发送给对车的任务。
        发送失败不影响本车继续执行自己的配送任务。
        """
        if not self._dual_car_enabled:
            return
        if not self._dual_remote_task_enabled:
            return
        peer_id = self._dual_peer_id()

        task_data = {
            "code": task_detection.code,
            "box": task_detection.box_index,
            "lab_window": task_detection.lab_window,
            "sample_count": task_detection.sample_count,
        }

        sent_ok = False
        if self._dual_tcp_bridge is not None:
            payload = {
                "type": "remote_task",
                "from_car": self._car_id,
                "target_car": peer_id,
                "stamp": time.time(),
                "task": task_data,
            }
            sent_ok = self._dual_tcp_bridge.send(payload)
        else:
            logwarn(
                "[DualCar] 无法发送远程任务 — TCP bridge 未启动"
            )

        if sent_ok:
            loginfo(
                "[DualCar] send remote task via TCP to car %s success: "
                "code=%s box=%d lab_window=%s",
                peer_id,
                task_detection.code,
                task_detection.box_index,
                task_detection.lab_window,
            )
        elif self._dual_tcp_bridge is not None:
            logwarn(
                "[DualCar] send remote task to car %s failed — TCP 未连接",
                peer_id,
            )

    def _dual_publish_qr_task(self, code, lab_window, box_index=None):
        """发布本车任务占用或清空占用，通过 TCP 发送。

        参数:
            code:       二维码内容，空字符串表示清空。
            lab_window: 化验窗口，空字符串表示清空。
            box_index:  方框索引 0-3，TCP 模式发送时使用。
        """
        if not self._dual_car_enabled:
            return

        if self._dual_tcp_bridge is not None:
            if code and lab_window:
                payload = {
                    "type": "qr_task",
                    "from_car": self._car_id,
                    "code": code,
                    "lab_window": lab_window,
                    "box": box_index if box_index is not None else 0,
                    "stamp": time.time(),
                }
            else:
                payload = {
                    "type": "qr_task_clear",
                    "from_car": self._car_id,
                    "stamp": time.time(),
                }
            self._dual_tcp_bridge.send(payload)

    def _dual_apply_remote_task_if_available(self):
        """如果存在有效的远程任务，直接转换为当前任务并跳过识别板一。

        职责:
        1. 检查远程任务是否存在且功能已启用。
        2. 将远程任务转换为 Board1Detection，构建 RoundPlan。
        3. 发布 /current_task、/cv2_result，通过 TCP 发送任务占用。
        4. 清空远程任务标记已消费。
        5. 切换到 GOTO_EXAM 状态。

        返回:
            True  表示已接管流程，调用方应 return 跳过正常识别板一流程。
            False 表示无远程任务或转换失败，继续正常流程。
        """
        if not self._dual_car_enabled:
            return False
        if not self._dual_remote_task_enabled:
            return False
        if self._dual_assigned_remote_task is None:
            return False

        remote = self._dual_assigned_remote_task
        loginfo(
            "[DualCar] using assigned remote task, skip board1: "
            "code=%s lab_window=%s",
            remote["code"], remote["lab_window"],
        )

        try:
            # 将远程任务转换为 Board1Detection。
            from pharmacy_mplus0.models import make_board1_detection
            detection = make_board1_detection(remote["code"], remote["box"])
        except (ValueError, KeyError) as exc:
            logwarn(
                "[DualCar] failed to convert remote task to detection: %s, "
                "fallback to normal board1 flow",
                exc,
            )
            self._dual_assigned_remote_task = None
            return False

        try:
            # 构建本轮执行计划。
            self._round_plan = self._planner.build_round_plan(detection)
        except (ValueError, KeyError) as exc:
            logwarn(
                "[DualCar] failed to build round plan from remote task: %s, "
                "fallback to normal board1 flow",
                exc,
            )
            self._dual_assigned_remote_task = None
            return False

        self._exam_visit_idx = 0

        loginfo(
            "[Main] 本轮任务（远程）: 二维码=%s 方框=%d 化验窗口=%s "
            "样本类型=%s 体检顺序=%s",
            self._round_plan.code,
            detection.box_index,
            self._round_plan.lab_window,
            self._round_plan.sample_type,
            self._round_plan.exam_windows,
        )

        # 发布 CV2 和任务占用（与正常识别板一流程一致）。
        self._io.publish_cv2(
            self._round_plan.code, self._round_plan.lab_window
        )
        self._dual_publish_qr_task(
            self._round_plan.code, self._round_plan.lab_window,
            box_index=detection.box_index,
        )

        # 清空远程任务，标记已消费。
        self._dual_assigned_remote_task = None

        # 直接跳到体检区配送流程，跳过识别板一。
        self._state = STATE_GOTO_EXAM
        return True

    # ---- 状态机调度 -------------------------------------------------

    def run(self):
        """主循环：根据当前状态分发到对应处理函数。"""
        dispatch = {
            STATE_INIT:        self._do_init,
            STATE_GOTO_BOARD1: self._do_goto_board1,
            STATE_AT_BOARD1:   self._do_at_board1,
            STATE_GOTO_EXAM:   self._do_goto_exam,
            STATE_AT_EXAM:     self._do_at_exam,
            STATE_GOTO_BOARD2: self._do_goto_board2,
            STATE_AT_BOARD2:   self._do_at_board2,
            STATE_PASS_BOARD2: self._do_pass_board2,
            STATE_GOTO_LAB:    self._do_goto_lab,
            STATE_AT_LAB:      self._do_at_lab,
            STATE_DONE_ROUND:  self._do_done_round,
        }
        loginfo("[Main] 主循环开始")
        while not rospy.is_shutdown():
            handler = dispatch.get(self._state)
            if handler is None:
                logerr("[Main] 未知状态: %s", self._state)
                break
            handler()

    def shutdown(self):
        """节点关闭时停车并取消导航。"""
        loginfo("[Main] 关闭中，停车...")
        try:
            self._nav.cancel()
        except Exception:
            pass
        # 停止双车 TCP 桥接。
        if self._dual_tcp_bridge is not None:
            self._dual_tcp_bridge.stop()
        # 发送零速度确保小车停止。
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)

    # ---- 状态处理函数 -----------------------------------------------

    def _do_init(self):
        """初始化：清空状态，进入前往识别板一。"""
        loginfo("[Main] === INIT === 初始化")
        self._io.set_task_road()
        self._store.clear()
        self._board1_detections = []
        self._board2_wait = None
        self._state = STATE_GOTO_BOARD1

    def _do_goto_board1(self):
        """前往识别板一。导航失败时保持当前状态自动重试。"""
        # 双车协作：检查是否允许出发，未允许则保持等待。
        if self._dual_car_enabled and not self._dual_start_allowed:
            if not self._dual_waiting_at_start:
                loginfo(
                    "[Main] 车 %s 在起点等待对车放行 (ALLOW_START) ...",
                    self._car_id,
                )
            self._dual_waiting_at_start = True
            self._cmd_vel_pub.publish(Twist())
            rospy.sleep(0.1)
            return

        # 远程任务共享：如果已缓存远程任务，直接跳过识别板一。
        if self._dual_apply_remote_task_if_available():
            return

        # 新一轮开始，重置本轮放行和远程任务发送标记。
        self._dual_allow_sent_this_round = False
        self._dual_remote_task_sent_this_round = False

        loginfo("[Main] === GOTO_BOARD1 (即将进入第 %d 轮) ===",
                self._round_index + 1)
        self._io.set_task_road()
        # 新一轮：清空缓存，通知识别节点解锁。
        self._board1_detections = []
        self._board2_wait = None
        self._store.clear()
        self._reset_pub.publish(String())
        self._dual_publish_qr_task("", "")

        ok = self._nav.go_to("board1", timeout_sec=self._nav_timeout)
        self._nav.clear_costmaps()
        if ok:
            self._round_index += 1
            loginfo("[Main] 第 %d 轮开始", self._round_index)
            self._state = STATE_AT_BOARD1
        else:
            logwarn("[Main] 前往识别板一失败，保持状态自动重试")

    def _do_at_board1(self):
        """等待二维码识别结果，选择最优任务，规划本轮配送。

        等待策略:
          - 到达后先 settle 1 秒，让画面稳定。
          - 单次等待最多 board1_wait_seconds 秒。
          - 最多重试 board1_max_retry 次。
          - 全部失败则结束本轮，累计连续失败。
          - 连续失败达 max_consecutive_fail_rounds 后原地停车。
        """
        loginfo("[Main] === AT_BOARD1 === 等待二维码识别结果")
        rospy.sleep(self._board1_settle)

        retry = 0
        self._board1_detections = []

        while not rospy.is_shutdown():
            t0 = rospy.Time.now()
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - t0).to_sec()
                if elapsed > self._board1_timeout:
                    break
                if self._board1_detections:
                    break
                rospy.sleep(0.1)

            if self._board1_detections:
                self._consecutive_fails = 0
                break

            # 超时，重试。
            retry += 1
            if retry > self._board1_max_retry:
                self._consecutive_fails += 1
                logwarn(
                    "[Main] 识别板一 %d 次重试均超时，本轮放弃；"
                    "连续失败计数 %d/%d",
                    self._board1_max_retry,
                    self._consecutive_fails,
                    self._max_consec_fails,
                )
                if self._consecutive_fails >= self._max_consec_fails:
                    logerr(
                        "[Main] 连续 %d 轮识别失败！原地停车，"
                        "请人工检查摄像头/识别板/光线",
                        self._consecutive_fails,
                    )
                    while not rospy.is_shutdown():
                        self._cmd_vel_pub.publish(Twist())
                        rospy.sleep(1.0)
                    return
                self._state = STATE_DONE_ROUND
                return

            logwarn("[Main] 识别板一 %.0fs 内未识别，第 %d/%d 次重试",
                    self._board1_timeout, retry, self._board1_max_retry)
            self._reset_pub.publish(String())
            self._board1_detections = []

        # 使用 TaskPlanner 从所有检测结果中选出最优任务。
        best = self._planner.select_best(
            self._board1_detections,
            excluded_box=self._peer_occupied_box,
        )
        if best is None:
            logwarn("[Main] 无有效二维码，结束本轮")
            self._consecutive_fails += 1
            self._state = STATE_DONE_ROUND
            return

        # 构建本轮执行计划。
        self._round_plan = self._planner.build_round_plan(best)
        self._exam_visit_idx = 0

        lab_name = self._nav.get_waypoint(
            "lab_" + self._round_plan.lab_window
        )
        loginfo(
            "[Main] 本轮任务: 二维码=%s 方框=%d 化验窗口=%s "
            "样本类型=%s 体检顺序=%s",
            self._round_plan.code,
            best.box_index,
            self._round_plan.lab_window,
            self._round_plan.sample_type,
            self._round_plan.exam_windows,
        )

        # 发布 CV2 和双车协作信息。
        self._io.publish_cv2(
            self._round_plan.code, self._round_plan.lab_window
        )
        self._dual_publish_qr_task(
            self._round_plan.code, self._round_plan.lab_window,
            box_index=best.box_index,
        )

        # ---- 远程任务共享：将剩余任务发送给对车 -------------------
        if (self._dual_car_enabled
                and self._dual_remote_task_enabled
                and not self._dual_remote_task_sent_this_round
                and self._car_id == "1"):
            # 排除本车已选任务，从剩余检测中选出最适合对车的任务。
            remaining = [
                d for d in self._board1_detections
                if d.box_index != best.box_index
            ]
            if remaining:
                peer_task = self._planner.select_best(remaining)
                if peer_task is not None:
                    self._dual_publish_remote_task_for_peer(peer_task)
                    self._dual_remote_task_sent_this_round = True
        # -----------------------------------------------------------

        self._state = STATE_GOTO_EXAM

    def _do_goto_exam(self):
        """前往下一个体检窗口。

        如果所有窗口已取完 → GOTO_BOARD2。
        导航失败则跳过当前窗口，尝试下一个。
        """
        if self._exam_visit_idx >= len(
            self._round_plan.exam_windows
        ):
            self._state = STATE_GOTO_BOARD2
            return

        win = self._round_plan.exam_windows[self._exam_visit_idx]
        wp_name = "exam_" + win
        loginfo("[Main] === GOTO_EXAM === 前往体检窗口 %s", win)
        self._io.set_task_road()

        ok = self._nav.go_to(wp_name, timeout_sec=self._nav_timeout)
        self._nav.clear_costmaps()
        if ok:
            self._state = STATE_AT_EXAM
        else:
            logwarn("[Main] 前往体检窗口 %s 失败，跳过", win)
            self._exam_visit_idx += 1
            # 保持 GOTO_EXAM，下次循环尝试下一个窗口。

    def _do_at_exam(self):
        """停在体检窗口：停 task → 记录样本 → 播报 → 停留 → 下一个。"""
        win = self._round_plan.exam_windows[self._exam_visit_idx]
        loginfo("[Main] === AT_EXAM === 停在体检窗口 %s", win)

        # 1) 更新 task 为当前窗口（裁判可见）。
        self._io.set_task(win)
        # 2) 记录取样。
        self._store.add_sample(win, self._round_plan.sample_type)

        self._exam_visit_idx += 1

        # 3) 最后一个窗口？先播报再停留，让音频播放和 dwell 重叠。
        if self._exam_visit_idx >= len(
            self._round_plan.exam_windows
        ):
            self._io.announce_exam_samples(
                self._store.carried_windows(), self._store.sample_type
            )
            # 停留，满足"明显停留"规则；期间音频异步播放。
            rospy.sleep(self._exam_dwell)
            self._io.set_task_road()
            self._state = STATE_GOTO_BOARD2
        else:
            # 停留，满足"明显停留"规则。
            rospy.sleep(self._exam_dwell)
            self._io.set_task_road()
            self._state = STATE_GOTO_EXAM

    def _do_goto_board2(self):
        """前往识别板二。导航失败时按空闲处理。"""
        loginfo("[Main] === GOTO_BOARD2 ===")
        self._io.set_task_road()
        self._board2_wait = None

        ok = self._nav.go_to("board2", timeout_sec=self._nav_timeout)
        self._nav.clear_costmaps()
        if ok:
            self._state = STATE_AT_BOARD2
        else:
            logwarn("[Main] 前往识别板二失败，按空闲处理")
            self._board2_wait = 0
            self._state = STATE_PASS_BOARD2

    def _do_at_board2(self):
        """等待识别板二结果。超时按空闲处理。"""
        loginfo("[Main] === AT_BOARD2 === 等待 /cv1_result")

        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._board2_wait is not None:
                break
            if (rospy.Time.now() - t0).to_sec() > self._board2_timeout:
                logwarn(
                    "[Main] 识别板二 %.0fs 内未识别，按空闲处理",
                    self._board2_timeout,
                )
                self._board2_wait = 0
                break
            rospy.sleep(0.1)

        # 播报并更新 CV1。
        self._io.publish_cv1(self._board2_wait)
        self._io.announce_board2(self._board2_wait)
        self._state = STATE_PASS_BOARD2

    def _do_pass_board2(self):
        """执行识别板二动作：
          空闲 (0) → 直接进入去化验窗口。
          忙碌 (5~10) → 等待指定秒数后进入去化验窗口。
        """
        wait_sec = int(self._board2_wait or 0)
        if wait_sec == 0:
            loginfo("[Main] 化验区空闲，快速通过")
        else:
            loginfo("[Main] 化验区忙碌，等待 %d 秒", wait_sec)
            rospy.sleep(wait_sec)
        self._state = STATE_GOTO_LAB

    def _do_goto_lab(self):
        """前往目标化验窗口。导航失败则结束本轮。"""
        lab_win = self._round_plan.lab_window
        wp_name = "lab_" + lab_win
        loginfo("[Main] === GOTO_LAB === 前往化验窗口 %s", lab_win)
        self._io.set_task_road()

        ok = self._nav.go_to(wp_name, timeout_sec=self._nav_timeout)
        self._nav.clear_costmaps()
        if ok:
            self._state = STATE_AT_LAB
        else:
            logwarn("[Main] 前往化验窗口 %s 失败，结束本轮", lab_win)
            self._state = STATE_DONE_ROUND

    def _do_at_lab(self):
        """停在化验窗口：停 task → 投递 → 播报+放行 → 停留 → 结束本轮。"""
        lab_win = self._round_plan.lab_window
        loginfo("[Main] === AT_LAB === 停在化验窗口 %s", lab_win)

        # 1) 更新 task。
        self._io.set_task(lab_win)
        # 2) 投递。
        self._store.deliver_to_lab(lab_win)
        # 3) 先播报、放行对车，再停留，让音频播放和 dwell 重叠。
        count = self._store.count_for_lab(lab_win)
        self._io.announce_lab_arrival(lab_win, count)
        self._dual_publish_allow_peer_start()
        # 4) 停留，满足"明显停留"规则；期间音频异步播放。
        rospy.sleep(self._lab_dwell)
        # 5) 离开。
        self._io.set_task_road()
        self._state = STATE_DONE_ROUND

    def _do_done_round(self):
        """本轮结束。根据策略回起点或不回起点，然后进入下一轮。"""
        loginfo("[Main] === DONE_ROUND === 本轮结束")
        self._dual_publish_qr_task("", "")

        # 清理远程任务缓存，避免下一轮误用。
        self._dual_assigned_remote_task = None

        # 双车放行信号已在 _do_at_lab 中发送，这里不再重复。

        if self._return_to_start:
            loginfo("[Main] 返回起点...")
            self._nav.go_to("start", timeout_sec=self._nav_timeout)
            self._nav.clear_costmaps()

        self._state = STATE_GOTO_BOARD1

    # ---- 配置加载 ---------------------------------------------------

    @staticmethod
    def _load_strategy_config():
        """从 pharmacy_mplus0/config/strategy.yaml 加载策略参数。"""
        try:
            ros_pack = rospkg.RosPack()
            pkg_path = ros_pack.get_path("pharmacy_mplus0")
            config_path = os.path.join(pkg_path, "config", _STRATEGY_YAML)
            with open(config_path, "r") as fh:
                data = yaml.safe_load(fh)
            return data if data else {}
        except IOError:
            logwarn("[Main] 无法读取 strategy.yaml，使用默认参数")
            return {}

    # ---- 启动横幅 ---------------------------------------------------

    def _print_banner(self):
        """打印当前关键配置，方便现场对照调试。"""
        loginfo("=" * 60)
        loginfo("main_controller 启动")
        loginfo("  识别板一: 超时=%.0fs  最大重试=%d  稳定等待=%.1fs",
                self._board1_timeout, self._board1_max_retry,
                self._board1_settle)
        loginfo("  识别板二: 超时=%.0fs", self._board2_timeout)
        loginfo("  导航: 默认超时=%.0fs", self._nav_timeout)
        loginfo("  停留: 体检=%.1fs  化验=%.1fs",
                self._exam_dwell, self._lab_dwell)
        loginfo("  连续失败保护: %d 轮",
                self._max_consec_fails)
        loginfo("  每轮回起点: %s",
                "是" if self._return_to_start else "否")
        loginfo("  双车协作: %s",
                "启用 (car_id=%s)"
                % self._car_id if self._dual_car_enabled else "关闭")
        if self._dual_car_enabled:
            loginfo("  双车通信: TCP 模式 本车监听 %s:%d 对车 %s:%d",
                    self._dual_car_listen_ip, self._dual_car_listen_port,
                    self._dual_car_peer_ip if self._dual_car_peer_ip else "(未配置)",
                    self._dual_car_peer_port)
            if self._dual_start_allowed:
                loginfo("  双车状态: car %s 首发，允许立即出发", self._car_id)
            else:
                loginfo("  双车状态: car %s 等待对车 ALLOW_START 放行", self._car_id)
            if not self._dual_car_peer_ip:
                logwarn("  双车 TCP 模式已启用但 dual_car_peer_ip 为空，"
                        "TCP 连接将无法建立")
        loginfo("=" * 60)


# ---- 入口 ------------------------------------------------------------

def main():
    try:
        MainController().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
