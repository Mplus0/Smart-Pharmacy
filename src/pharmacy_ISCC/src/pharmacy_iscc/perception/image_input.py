# -*- coding: utf-8 -*-
import time
import threading
import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage
from pharmacy_iscc.perception.settings import (
    CAMERA_FALLBACK_TO_HTTP,
    CAMERA_MAX_AGE_SEC,
    CAMERA_TOPIC,
    CAMERA_URL,
    DROP_FRAME_COUNT,
    IMAGE_SOURCE,
    ROS_IMAGE_BUFF_SIZE,
)

ros_image_bridge = None
ros_image_sub = None
latest_ros_frame = None
latest_ros_frame_stamp = None
latest_ros_frame_recv_time = None
latest_ros_frame_lock = threading.Lock()


def read_latest_frame(cap, drop_count=DROP_FRAME_COUNT):
    '''防止旧帧影响识别'''
    if cap is None:
        return False, None
    for _ in range(drop_count):
        cap.grab()
    return cap.read()


def ros_image_cb(msg):
    """ROS 原始图像回调：只缓存最新帧，不在回调里做识别。"""
    global latest_ros_frame
    global latest_ros_frame_stamp
    global latest_ros_frame_recv_time
    global ros_image_bridge

    if ros_image_bridge is None:
        return

    try:
        frame = ros_image_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logwarn_throttle(1.0, "ROS图像转换失败: %s", str(e))
        return

    try:
        stamp = msg.header.stamp
    except Exception:
        stamp = rospy.Time.now()

    with latest_ros_frame_lock:
        latest_ros_frame = frame.copy()
        latest_ros_frame_stamp = stamp
        latest_ros_frame_recv_time = time.time()


def ros_compressed_image_cb(msg):
    """ROS 压缩图像回调：支持 camera_topic 指向 /compressed 话题。"""
    global latest_ros_frame
    global latest_ros_frame_stamp
    global latest_ros_frame_recv_time

    try:
        data = np.fromstring(msg.data, np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception as e:
        rospy.logwarn_throttle(1.0, "ROS压缩图像转换失败: %s", str(e))
        return

    if frame is None:
        return

    try:
        stamp = msg.header.stamp
    except Exception:
        stamp = rospy.Time.now()

    with latest_ros_frame_lock:
        latest_ros_frame = frame.copy()
        latest_ros_frame_stamp = stamp
        latest_ros_frame_recv_time = time.time()


def init_ros_image_subscriber():
    """初始化 ROS 图像订阅。默认订阅 /camera/rgb/image_raw。"""
    global ros_image_bridge
    global ros_image_sub

    if IMAGE_SOURCE not in ["ros_topic", "ros_image", "ros_compressed"]:
        rospy.logwarn("当前图像输入源为HTTP视频流: %s", CAMERA_URL)
        return

    if IMAGE_SOURCE == "ros_compressed" or CAMERA_TOPIC.endswith("/compressed"):
        ros_image_sub = rospy.Subscriber(
            CAMERA_TOPIC,
            CompressedImage,
            ros_compressed_image_cb,
            queue_size=1,
            buff_size=ROS_IMAGE_BUFF_SIZE
        )
        rospy.logwarn("图像输入源: ROS CompressedImage topic=%s, HTTP兜底=%s",
                      CAMERA_TOPIC, CAMERA_FALLBACK_TO_HTTP)
    else:
        ros_image_bridge = cv_bridge.CvBridge()
        ros_image_sub = rospy.Subscriber(
            CAMERA_TOPIC,
            Image,
            ros_image_cb,
            queue_size=1,
            buff_size=ROS_IMAGE_BUFF_SIZE
        )
        rospy.logwarn("图像输入源: ROS Image topic=%s, HTTP兜底=%s",
                      CAMERA_TOPIC, CAMERA_FALLBACK_TO_HTTP)


def get_latest_ros_frame(with_identity=False):
    """从 ROS 图像缓存取最新帧。返回 ok, frame, reason。"""
    with latest_ros_frame_lock:
        if latest_ros_frame is None:
            return False, None, "empty"

        age = 0.0
        if latest_ros_frame_recv_time is not None:
            age = time.time() - latest_ros_frame_recv_time

        if CAMERA_MAX_AGE_SEC > 0 and age > CAMERA_MAX_AGE_SEC:
            return False, None, "stale %.3fs" % age

        if with_identity:
            return True, latest_ros_frame.copy(), ("ok", latest_ros_frame_recv_time)
        return True, latest_ros_frame.copy(), "ok"


def open_http_capture():
    """打开 HTTP 视频流，用于 HTTP 模式或 ROS 模式下的兜底。"""
    cap = cv2.VideoCapture(CAMERA_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def read_frame_from_configured_source(cap, with_identity=False):
    """按配置读取图像：优先 ROS topic，必要时回退到 HTTP 视频流。"""
    if IMAGE_SOURCE in ["ros_topic", "ros_image", "ros_compressed"]:
        ok, frame, reason = get_latest_ros_frame(with_identity=with_identity)
        if ok:
            source = ("ros", reason[1]) if with_identity else "ros"
            return True, frame, source

        if not CAMERA_FALLBACK_TO_HTTP:
            rospy.logwarn_throttle(1.0, "ROS图像暂不可用: %s", reason)
            source = ("ros", None) if with_identity else "ros:%s" % reason
            return False, None, source

        rospy.logwarn_throttle(2.0, "ROS图像暂不可用: %s，临时使用HTTP视频流兜底", reason)

    hx, frame = read_latest_frame(cap, drop_count=DROP_FRAME_COUNT)
    source = ("http", None) if with_identity else "http"
    return hx, frame, source
