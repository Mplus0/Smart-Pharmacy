# -*- coding: utf-8 -*-
"""裁判可见状态与播报请求统一发布。

本模块是主控与 ROS 话题之间的唯一出口：
- /current_task    当前任务状态（A/B/C/1/2/3/4/R）
- /cv1_result      识别板二结果（WAIT-0 / WAIT-5..WAIT-10）
- /cv2_result      识别板一任务结果（如 AB-1）
- /announce_request  音频事件 ID（如 board2_idle）
- /current_qr_task  当前二维码任务信息（双车预留）

主控只调用语义化方法（如 arrive_exam("A")），
不需要自己拼接 event_id、状态字符串或 JSON。

本模块不负责实际音频播放，只把 event_id 发布到
/announce_request，由 tcp_reporter.py 中的 AudioAnnouncer 消费。
"""

import rospy
from std_msgs.msg import String

from pharmacy_mplus0.constants import (
    TOPIC_CURRENT_TASK,
    TOPIC_CV1_RESULT,
    TOPIC_CV2_RESULT,
    TOPIC_ANNOUNCE_REQUEST,
    TOPIC_CURRENT_QR_TASK,
    TOPIC_DUAL_CAR_SIGNAL,
    LAB_WINDOW_AUDIO_KEYS,
    SAMPLE_TYPE_AUDIO_KEYS,
    BOARD2_BUSY_SECONDS_MIN,
    BOARD2_BUSY_SECONDS_MAX,
    TASK_ROAD,
)


class CompetitionIO(object):
    """统一发布比赛裁判可见状态和播报请求。"""

    def __init__(self, car_id="1"):
        self._car_id = str(car_id)

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
        # announce_request: 音频事件 ID，由 tcp_reporter 中的 AudioAnnouncer 消费。
        self._pub_announce = rospy.Publisher(
            TOPIC_ANNOUNCE_REQUEST, String, queue_size=5
        )
        # current_qr_task: 当前正在执行的二维码任务（双车协作预留）。
        self._pub_qr_task = rospy.Publisher(
            TOPIC_CURRENT_QR_TASK, String, queue_size=5
        )
        # dual_car_signal: 双车轮流出发信号发布器。
        self._pub_dual_signal = rospy.Publisher(
            TOPIC_DUAL_CAR_SIGNAL, String, queue_size=5
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

    # ---- 音频播报请求 -----------------------------------------------

    def announce_exam_samples(self, windows, sample_type="1"):
        """播报体检区取样完成。

        参数:
            windows:     已取样的窗口列表，如 ["A", "B"]。
            sample_type: 样本类型编号，如 "1"（静脉血）。
        示例 event_id: exam_sample_ab_venous_blood
        """
        win_key = "".join(sorted([str(w).lower() for w in windows]))
        sample_key = SAMPLE_TYPE_AUDIO_KEYS.get(
            str(sample_type), "unknown"
        )
        event_id = "exam_sample_{0}_{1}".format(win_key, sample_key)
        self._publish_announce(event_id)

    def announce_board2(self, wait_seconds):
        """播报识别板二状态。

        参数:
            wait_seconds: 等待秒数，0 表示空闲，5-10 表示忙碌。
        示例 event_id: board2_idle / board2_busy_8
        """
        wait_sec = int(wait_seconds)
        if wait_sec == 0:
            event_id = "board2_idle"
        elif BOARD2_BUSY_SECONDS_MIN <= wait_sec <= BOARD2_BUSY_SECONDS_MAX:
            event_id = "board2_busy_{0}".format(wait_sec)
        else:
            rospy.logwarn(
                "[CompetitionIO] board2 wait_seconds 异常: %d，按空闲处理",
                wait_sec,
            )
            event_id = "board2_idle"
        self._publish_announce(event_id)

    def announce_lab_arrival(self, lab_window, sample_count):
        """播报到化验窗口并报告样本数量。

        参数:
            lab_window:   化验窗口编号（字符串 1-4）。
            sample_count: 车上携带的样本数量，限制在 1~3。
        示例 event_id: lab_blood_3
        """
        lab_key = LAB_WINDOW_AUDIO_KEYS.get(str(lab_window), "unknown")
        count = max(1, min(int(sample_count), 3))
        event_id = "lab_{0}_{1}".format(lab_key, count)
        self._publish_announce(event_id)

    # ---- 双车协作预留 -----------------------------------------------

    def publish_dual_signal(self, text):
        """发布双车协作信号（如 ALLOW_START:1 / ALLOW_START:2）。

        参数:
            text: 信号内容，如 "ALLOW_START:2"。
        """
        msg = String()
        msg.data = str(text)
        self._pub_dual_signal.publish(msg)

    def publish_qr_task(self, code, lab_window):
        """发布当前正在执行的二维码任务，供同伴小车避让。

        格式: CAR<id>:<code>-<lab_window>，如 CAR1:AB-1。
        清空时 code="" 且 lab_window=""，发布 CAR<id>:。

        参数:
            code:       二维码内容。
            lab_window: 目标化验窗口。
        """
        if code or lab_window:
            body = "CAR{0}:{1}-{2}".format(self._car_id, code, lab_window)
        else:
            body = "CAR{0}:".format(self._car_id)
        msg = String()
        msg.data = body
        self._pub_qr_task.publish(msg)

    # ---- 内部 -------------------------------------------------------

    def _publish_announce(self, event_id):
        """发布音频事件 ID 到 /announce_request 话题。"""
        msg = String()
        msg.data = event_id
        self._pub_announce.publish(msg)
        rospy.loginfo("[CompetitionIO] 播报事件: %s", event_id)
