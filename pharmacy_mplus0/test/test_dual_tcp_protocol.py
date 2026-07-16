#!/usr/bin/env python
# -*- coding: utf-8 -*-

import imp
import json
import os
import unittest

import rospkg


def load_dual_link_module():
    package_path = rospkg.RosPack().get_path("pharmacy_mplus0")
    script_path = os.path.join(package_path, "scripts", "dual_car_link.py")
    return imp.load_source("dual_car_link_under_test", script_path)


class DummyPublisher(object):

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class DualTcpProtocolTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.module = load_dual_link_module()

    def setUp(self):
        self.node = self.module.DualCarTcpLink.__new__(self.module.DualCarTcpLink)
        self.node.last_peer_seq = 0
        self.node.last_peer_board1_seq = 0
        self.node.pub_peer_board1_all_text = DummyPublisher()
        self.done_events = []
        self.node.publish_peer_done = self.record_done

        self.original_sleep = self.module.rospy.sleep
        self.original_token = self.module.SHARED_TOKEN
        self.original_check_peer_ip = self.module.CHECK_PEER_IP
        self.module.rospy.sleep = lambda seconds: None

    def tearDown(self):
        self.module.rospy.sleep = self.original_sleep
        self.module.SHARED_TOKEN = self.original_token
        self.module.CHECK_PEER_IP = self.original_check_peer_ip

    def record_done(self, peer_id, seq):
        self.done_events.append((peer_id, seq))

    def peer_addr(self):
        return (self.module.PEER_IP, 12345)

    def test_round_done_sequence_deduplication(self):
        payload = {
            "type": "round_done",
            "car_id": self.module.PEER_ID,
            "seq": 1,
        }
        line = json.dumps(payload)

        self.node.handle_line(line, self.peer_addr())
        self.node.handle_line(line, self.peer_addr())

        self.assertEqual([(self.module.PEER_ID, 1)], self.done_events)

    def test_wrong_car_id_is_ignored(self):
        payload = {
            "type": "round_done",
            "car_id": self.module.CAR_ID,
            "seq": 1,
        }
        self.node.handle_line(json.dumps(payload), self.peer_addr())
        self.assertEqual([], self.done_events)

    def test_invalid_json_and_unknown_type_are_ignored(self):
        self.node.handle_line("not-json", self.peer_addr())
        self.node.handle_line(json.dumps({
            "type": "unknown",
            "car_id": self.module.PEER_ID,
            "seq": 1,
        }), self.peer_addr())
        self.assertEqual([], self.done_events)
        self.assertEqual([], self.node.pub_peer_board1_all_text.messages)

    def test_wrong_source_ip_is_ignored(self):
        self.module.CHECK_PEER_IP = True
        payload = {
            "type": "round_done",
            "car_id": self.module.PEER_ID,
            "seq": 1,
        }
        self.node.handle_line(json.dumps(payload), ("127.0.0.1", 12345))
        self.assertEqual([], self.done_events)

    def test_optional_token_validation(self):
        self.module.SHARED_TOKEN = "test-token"
        payload = {
            "type": "round_done",
            "car_id": self.module.PEER_ID,
            "seq": 1,
            "token": "wrong-token",
        }
        self.node.handle_line(json.dumps(payload), self.peer_addr())
        self.assertEqual([], self.done_events)

        payload["token"] = "test-token"
        self.node.handle_line(json.dumps(payload), self.peer_addr())
        self.assertEqual([(self.module.PEER_ID, 1)], self.done_events)

    def test_board1_payload_and_sequence_deduplication(self):
        payload = {
            "type": "board1_all_text",
            "car_id": self.module.PEER_ID,
            "seq": 1,
            "all_text": ["ABC", "AB", "A", ""],
            "selected_index": 0,
            "selected_text": "ABC",
            "selected_msg": [1, 1, 1, 3, 0, 0],
        }
        line = json.dumps(payload)

        self.node.handle_line(line, self.peer_addr())
        self.node.handle_line(line, self.peer_addr())

        self.assertEqual(3, len(self.node.pub_peer_board1_all_text.messages))
        published = json.loads(self.node.pub_peer_board1_all_text.messages[0])
        self.assertEqual(payload, published)


if __name__ == "__main__":
    unittest.main()
