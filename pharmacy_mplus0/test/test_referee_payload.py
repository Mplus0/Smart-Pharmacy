#!/usr/bin/env python
# -*- coding: utf-8 -*-

import imp
import os
import threading
import unittest

import rospkg
from nav_msgs.msg import Odometry


def load_referee_module():
    package_path = rospkg.RosPack().get_path("pharmacy_mplus0")
    script_path = os.path.join(package_path, "scripts", "referee_reporter.py")
    return imp.load_source("referee_reporter_under_test", script_path)


class RefereePayloadTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.module = load_referee_module()

    def new_client_without_ros_node(self):
        client = self.module.RefereeClient.__new__(self.module.RefereeClient)
        client.payload_lock = threading.Lock()
        client.payload = client.default_payload()
        return client

    def test_default_payload(self):
        client = self.new_client_without_ros_node()
        self.assertEqual({
            "id": str(self.module.CAR_CONFIG["car_id"]),
            "speed": 0.0,
            "odom": [0.0, 0.0],
            "task": "R",
            "CV1": "None",
            "CV2": "None",
        }, client.payload)

    def test_speed_uses_filtered_odom_twist(self):
        client = self.new_client_without_ros_node()
        msg = Odometry()
        msg.pose.pose.position.x = 99.0
        msg.pose.pose.position.y = 88.0
        msg.twist.twist.linear.x = 3.0
        msg.twist.twist.linear.y = 4.0

        client.filtered_odom_cb(msg)

        self.assertEqual(5.0, client.payload["speed"])
        self.assertEqual([0.0, 0.0], client.payload["odom"])


if __name__ == "__main__":
    unittest.main()
