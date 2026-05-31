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
from pharmacy_mplus0.models import (
    Board1Detection,
    Board2Status,
    make_board1_detection,
    board2_status_from_text,
)

# 默认配置文件名。
_STRATEGY_YAML = "strategy.yaml"


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
        self._io = CompetitionIO()
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
        loginfo("[Main] === GOTO_BOARD1 (即将进入第 %d 轮) ===",
                self._round_index + 1)
        self._io.set_task_road()
        # 新一轮：清空缓存，通知识别节点解锁。
        self._board1_detections = []
        self._board2_wait = None
        self._store.clear()
        self._reset_pub.publish(String())
        self._io.publish_qr_task("", "")

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
        best = self._planner.select_best(self._board1_detections)
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
        self._io.publish_qr_task(
            self._round_plan.code, self._round_plan.lab_window
        )

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
        """停在体检窗口：停 task → 停留 → 记录样本 → 播报 → 下一个。"""
        win = self._round_plan.exam_windows[self._exam_visit_idx]
        loginfo("[Main] === AT_EXAM === 停在体检窗口 %s", win)

        # 1) 更新 task 为当前窗口（裁判可见）。
        self._io.set_task(win)
        # 2) 停留，满足"明显停留"规则。
        rospy.sleep(self._exam_dwell)
        # 3) 记录取样。
        self._store.add_sample(win, self._round_plan.sample_type)

        self._exam_visit_idx += 1

        # 4) 最后一个窗口？播报并前往识别板二。
        if self._exam_visit_idx >= len(
            self._round_plan.exam_windows
        ):
            # 播报取到的样本窗口和样本类型。
            self._io.announce_exam_samples(
                self._store.carried_windows(), self._store.sample_type
            )
            self._io.set_task_road()
            self._state = STATE_GOTO_BOARD2
        else:
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
        """停在化验窗口：停 task → 停留 → 播报 → 投递 → 结束本轮。"""
        lab_win = self._round_plan.lab_window
        loginfo("[Main] === AT_LAB === 停在化验窗口 %s", lab_win)

        # 1) 更新 task。
        self._io.set_task(lab_win)
        # 2) 停留。
        rospy.sleep(self._lab_dwell)
        # 3) 播报到达化验窗口及样本数量。
        count = self._store.count_for_lab(lab_win)
        self._io.announce_lab_arrival(lab_win, count)
        # 4) 投递。
        self._store.deliver_to_lab(lab_win)
        # 5) 离开。
        self._io.set_task_road()
        self._state = STATE_DONE_ROUND

    def _do_done_round(self):
        """本轮结束。根据策略回起点或不回起点，然后进入下一轮。"""
        loginfo("[Main] === DONE_ROUND === 本轮结束")
        self._io.publish_qr_task("", "")

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
        loginfo("=" * 60)


# ---- 入口 ------------------------------------------------------------

def main():
    try:
        MainController().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
