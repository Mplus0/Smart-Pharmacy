# -*- coding: utf-8 -*-
"""裁判可见状态与播报请求统一发布。

本模块是主控与 ROS 话题之间的唯一出口：
- /current_task    当前任务状态（A/B/C/1/2/3/4/R）
- /cv1_result      识别板二结果（WAIT-0 / WAIT-5..WAIT-10）
- /cv2_result      识别板一任务结果（如 AB-1）
- /announce_request  语音播报请求文本
- /current_qr_task  当前二维码任务信息（双车预留）

主控只调用语义化方法（如 arrive_exam("A")），
不需要自己拼接播报文本、状态字符串或 JSON。

本模块不负责实际 TTS 合成，只把文本发布到
/announce_request，由 tcp_reporter.py 中的 Voice 模块消费。
"""

import rospy
from std_msgs.msg import String

from pharmacy_mplus0.constants import (
    TOPIC_CURRENT_TASK,
    TOPIC_CV1_RESULT,
    TOPIC_CV2_RESULT,
    TOPIC_ANNOUNCE_REQUEST,
    TOPIC_CURRENT_QR_TASK,
    LAB_WINDOW_NAMES,
    TASK_ROAD,
    ANNOUNCE_EXAM_SAMPLES,
    ANNOUNCE_BOARD2_IDLE,
    ANNOUNCE_BOARD2_BUSY,
    ANNOUNCE_LAB_ARRIVAL,
)


class CompetitionIO(object):
    """统一发布比赛裁判可见状态和播报请求。"""

    def __init__(self):
        # ---- 裁判评分可见话题 ----
        # task: 当前小车所在位置或执行的动作类型。
        self._pub_task = rospy.Publisher(
            TOPIC_CURRENT_TASK, String, queue_size=5
        )
        # CV1: 识别板二的状态（WAIT-0 或 WAIT-5..WAIT-10）。
        self._pub_cv1 = rospy.Publisher(
            TOPIC_CV1_RESULT, String, queue_size=5
        )
        # CV2: 识别板一本轮选择的任务（如 AB-1 表示取 A、B 窗口样本送 1 号化验窗）。
        self._pub_cv2 = rospy.Publisher(
            TOPIC_CV2_RESULT, String, queue_size=5
        )
        # announce_request: 语音播报请求，由 tcp_reporter 中的 voice 模块消费。
        self._pub_announce = rospy.Publisher(
            TOPIC_ANNOUNCE_REQUEST, String, queue_size=5
        )
        # current_qr_task: 当前正在执行的二维码任务（双车协作预留）。
        self._pub_qr_task = rospy.Publisher(
            TOPIC_CURRENT_QR_TASK, String, queue_size=5
        )

        rospy.loginfo("[CompetitionIO] 裁判状态发布器已初始化")

    # ---- 当前任务状态 -----------------------------------------------

    def set_task(self, task):
        """发布当前任务状态。

        参数:
            task: "A"/"B"/"C" 表示停在体检窗口，
                  "1"/"2"/"3"/"4" 表示停在化验窗口，
                  TASK_ROAD ("R") 表示路上/起点/识别区。
        """
        msg = String()
        msg.data = str(task)
        self._pub_task.publish(msg)

    def set_task_road(self):
        """快捷方法：将当前任务置为"路上/起点/识别区"。"""
        self.set_task(TASK_ROAD)

    # ---- 识别板一结果 (CV2) -----------------------------------------

    def publish_cv2(self, code, lab_window):
        """发布识别板一任务结果。

        参数:
            code:       二维码内容（如 AB / ABC / C）。
            lab_window: 目标化验窗口编号（字符串 1-4）。
        裁判端会看到类似 "AB-1" 的结果。
        """
        body = "{0}-{1}".format(code, lab_window)
        msg = String()
        msg.data = body
        self._pub_cv2.publish(msg)
        rospy.loginfo("[CompetitionIO] CV2 发布: %s", body)

    # ---- 识别板二结果 (CV1) -----------------------------------------

    def publish_cv1(self, wait_seconds):
        """发布识别板二状态。

        参数:
            wait_seconds: 等待秒数，0 表示空闲，5-10 表示忙碌。
        """
        body = "WAIT-{0}".format(wait_seconds)
        msg = String()
        msg.data = body
        self._pub_cv1.publish(msg)
        rospy.loginfo("[CompetitionIO] CV1 发布: %s", body)

    # ---- 语音播报请求 -----------------------------------------------

    def announce_exam_samples(self, windows):
        """播报体检区取样完成。

        参数:
            windows: 已取样的窗口列表，如 ["A", "B"]。
        播报示例: "取到 A、B 窗口的样本"
        """
        joined = "、".join([str(w) for w in windows])
        text = ANNOUNCE_EXAM_SAMPLES.format(joined)
        self._publish_announce(text)

    def announce_board2(self, wait_seconds):
        """播报识别板二状态。

        参数:
            wait_seconds: 等待秒数，0 播报空闲，5-10 播报忙碌等待。
        """
        if wait_seconds == 0:
            text = ANNOUNCE_BOARD2_IDLE
        else:
            text = ANNOUNCE_BOARD2_BUSY.format(wait_seconds)
        self._publish_announce(text)

    def announce_lab_arrival(self, lab_window, sample_count):
        """播报到化验窗口并报告样本数量。

        参数:
            lab_window:   化验窗口编号（字符串 1-4）。
            sample_count: 车上携带的样本数量。
        播报示例: "到达血常规窗口，样本数为 3"
        """
        window_name = LAB_WINDOW_NAMES.get(
            str(lab_window), "{0}号窗口".format(lab_window)
        )
        text = ANNOUNCE_LAB_ARRIVAL.format(window_name, sample_count)
        self._publish_announce(text)

    # ---- 双车协作预留 -----------------------------------------------

    def publish_qr_task(self, code, lab_window):
        """发布当前正在执行的二维码任务，供同伴小车避让。

        参数:
            code:       二维码内容。
            lab_window: 目标化验窗口。
        """
        body = "{0}-{1}".format(code, lab_window)
        msg = String()
        msg.data = body
        self._pub_qr_task.publish(msg)

    # ---- 内部 -------------------------------------------------------

    def _publish_announce(self, text):
        """发布播报请求到 /announce_request 话题。"""
        msg = String()
        msg.data = text
        self._pub_announce.publish(msg)
        rospy.loginfo("[CompetitionIO] 播报请求: %s", text)
