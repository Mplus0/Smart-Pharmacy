#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only ROS observer; uses the board's ROS Python interpreter."""
from __future__ import print_function
import json
import sys
import threading
import time
import rospy
from std_msgs.msg import Int32, Int32MultiArray

lock = threading.Lock()


def emit(kind, value):
    with lock:
        sys.stdout.write('CONSOLE_JSON ' + json.dumps(
            {'kind': kind, 'value': value, 'time': time.time()}) + '\n')
        sys.stdout.flush()


rospy.init_node('pharmacy_web_observer', anonymous=True, disable_signals=True)
for topic, msg_type, key in [('/nav_state', Int32, 'nav'),
                             ('/cam_return', Int32MultiArray, 'board1'),
                             ('/board2_return', Int32MultiArray, 'board2')]:
    rospy.Subscriber(topic, msg_type, lambda msg, k=key: emit(k, msg.data), queue_size=1)

while not rospy.is_shutdown():
    try:
        code, message, state = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError(message)
        nodes = set(node for group in state for name, providers in group for node in providers)
        services = set(name for name, providers in state[2])
        emit('ros', {'ready': '/nav_pharmacy/start' in services,
                     'nav_node': '/nav_pharmacy' in nodes})
    except Exception as exc:
        emit('ros_error', str(exc))
    time.sleep(2)
