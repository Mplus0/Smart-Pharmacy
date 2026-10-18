#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS Melodic / Python 2 regression checks; no running ROS master is needed."""
from __future__ import print_function
import imp
import os
import sys
import time
import unittest
from contextlib import contextmanager

import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(PACKAGE, 'scripts')
if not os.path.isdir(SCRIPTS):
    SCRIPTS = PACKAGE  # Isolated candidate layout.
sys.path.insert(0, SCRIPTS)
sys.path.append('/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts')
FIXTURE = os.path.join(os.path.dirname(__file__), 'fixtures', 'board1_D_blank_ABC_BC.png')
EXPECTED_TEXT = ['D', '', 'ABC', 'BC']
EXPECTED_MSG = [1, 1, 1, 3, 2, 1]


@contextmanager
def attrs(obj, changes):
    before = dict((key, getattr(obj, key)) for key in changes)
    try:
        for key, value in changes.items():
            setattr(obj, key, value)
        yield
    finally:
        for key, value in before.items():
            setattr(obj, key, value)


class Recorder(object):
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data if hasattr(msg, 'data') else msg)


class Board1Tests(unittest.TestCase):
    def setUp(self):
        imp.load_source('dual_car_config', os.path.join(SCRIPTS, 'dual_car_config.py'))
        self.m = imp.load_source('board1_regression_vision', os.path.join(SCRIPTS, 'F1_detect_code_v5.py'))
        self.frame = cv2.imread(FIXTURE)
        self.assertIsNotNone(self.frame)
        self.m.pub_flag = Recorder()
        self.m.pub_board1_all_text = Recorder()
        self.error_plays = []
        self.actual_error_audio = self.m.play_board1_error_audio
        self.m.play_board1_error_audio = lambda: self.error_plays.append('error.wav')
        self.ros_patch = attrs(rospy, dict((name, lambda *a, **k: None) for name in [
            'loginfo', 'logwarn', 'loginfo_throttle', 'logwarn_throttle', 'sleep']))
        self.ros_patch.__enter__()

    def tearDown(self):
        self.ros_patch.__exit__(None, None, None)

    def test_error_plays_once_after_three_stable_observations(self):
        self.good(2)
        self.assertEqual(self.error_plays, [])
        self.assertEqual(self.m.pub_flag.messages, [])
        self.good(3)
        self.assertEqual(self.error_plays, ['error.wav'])
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG])

    def test_clean_and_blank_frames_never_alert(self):
        for text in [['A', 'AB', 'ABC', ''], ['', '', '', '']]:
            for _ in range(5): self.m.announce_stable_board1_error(text)
        self.assertEqual(self.error_plays, [])

    def test_abnormal_only_board_alerts_without_creating_pickup_task(self):
        for _ in range(4): self.m.stable_publish_board1(['D', '', '', ''])
        self.assertEqual(self.error_plays, ['error.wav'])
        self.assertEqual(self.m.pub_flag.messages, [])

    def test_abnormal_only_real_window_path_keeps_stability(self):
        with attrs(self.m, {'decode_board1_by_windows': lambda frame: ['D', '', '', '']}):
            for _ in range(3): self.m.detect_and_publish_board1(self.frame)
        self.assertEqual(self.error_plays, ['error.wav'])
        self.assertEqual(self.m.pub_flag.messages, [])

    def test_error_position_independent_and_next_visit_resets(self):
        for index in range(4):
            self.m.nav_state_cb(Int32(self.m.STATE_GO_TO_BOARD1))
            self.m.nav_state_cb(Int32(self.m.STATE_BOARD1_RECOGNIZING))
            text=['', '', '', ''];text[index]='ACD'
            for _ in range(3):self.m.announce_stable_board1_error(text)
        self.assertEqual(self.error_plays, ['error.wav']*4)

    def test_error_stability_resets_on_missing_border(self):
        self.good(2)
        with attrs(self.m, {'decode_board1_by_windows': lambda frame: None}):
            self.m.detect_and_publish_board1(self.frame)
        self.good(1)
        self.assertEqual(self.error_plays, [])
        self.good(2)
        self.assertEqual(self.error_plays, ['error.wav'])

    def test_missing_audio_does_not_spawn_or_crash(self):
        def forbidden(*args, **kwargs):raise AssertionError('Must not play missing file')
        with attrs(self.m.os.path, {'isfile': lambda p: False}):
            with attrs(self.m.subprocess, {'Popen': forbidden}):
                self.assertFalse(self.actual_error_audio())

    def test_audio_is_finished_before_task_publication(self):
        order=[]
        def publish(msg):order.append('task')
        with attrs(self.m.pub_flag, {'publish': publish}):
            with attrs(self.m, {'play_board1_error_audio': lambda:order.append('audio_completed')}):
                self.good(3)
        self.assertEqual(order, ['audio_completed','task'])

    def masked(self, slot):
        items = self.m.find_board1_window_boxes(self.frame)
        self.assertEqual(len(items), 4)
        box = items[slot]['box']
        x1, y1 = np.floor(box.min(axis=0) - 6).astype(int)
        x2, y2 = np.ceil(box.max(axis=0) + 6).astype(int)
        result = self.frame.copy()
        result[max(0, y1):y2 + 1, max(0, x1):x2 + 1] = 255
        return result

    def good(self, n):
        for _ in range(n):
            self.m.stable_publish_board1(EXPECTED_TEXT)

    def test_four_observed_borders_and_order(self):
        items = self.m.find_board1_window_boxes(self.frame)
        self.assertEqual(len(items), 4)
        self.assertFalse(any(item['inferred'] for item in items))
        centers = [item['center'] for item in items]
        self.assertLess(centers[0][0], centers[1][0])
        self.assertLess(centers[2][0], centers[3][0])
        self.assertLess(max(centers[0][1], centers[1][1]), min(centers[2][1], centers[3][1]))

    def test_four_slot_contents(self):
        self.assertEqual(self.m.decode_board1_by_windows(self.frame), EXPECTED_TEXT)

    def test_large_interference_qr_crop_recovery(self):
        items = self.m.find_board1_window_boxes(self.frame)
        warped = self.m.warp_window_from_item(self.frame, items[0])
        self.assertEqual(self.m.decode_qr_text_light(self.m.crop_window_inner(warped)), '')
        self.assertEqual(self.m.decode_board1_window(warped), 'D')

    def test_interference_is_reported_without_becoming_a_pickup_task(self):
        selection = self.m.select_board1_from_all_text(EXPECTED_TEXT)
        self.assertEqual(selection['selected_msg'], EXPECTED_MSG)
        self.assertEqual(selection['selected_text'], 'ABC')
        self.assertIsNone(self.m.select_board1_from_all_text(['D', '', '', '']))

    def test_missing_each_real_border_never_publishes(self):
        for slot in range(4):
            frame = self.masked(slot)
            self.assertIsNone(self.m.decode_board1_by_windows(frame), slot)
            for _ in range(3):
                self.m.detect_and_publish_board1(frame)
            self.assertEqual(self.m.pub_flag.messages, [], slot)

    def test_blank_frame_never_publishes(self):
        for _ in range(4):
            self.m.detect_and_publish_board1(np.full(self.frame.shape, 255, dtype=np.uint8))
        self.assertEqual(self.m.pub_flag.messages, [])

    def test_inferred_border_is_rejected(self):
        items = self.m.find_board1_window_boxes(self.frame)
        items[0]['inferred'] = True
        with attrs(self.m, {'find_board1_window_boxes': lambda frame: items}):
            self.assertIsNone(self.m.decode_board1_by_windows(self.frame))

    def test_three_observations_publish_expected_message(self):
        self.good(2)
        self.assertEqual(self.m.pub_flag.messages, [])
        self.good(1)
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG])

    def test_all_blank_result_interrupts_stability(self):
        self.good(2)
        self.m.stable_publish_board1(['', '', '', ''])
        self.good(2)
        self.assertEqual(self.m.pub_flag.messages, [])
        self.good(1)
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG])

    def test_missing_border_interrupts_stability(self):
        self.good(2)
        self.m.detect_and_publish_board1(self.masked(0))
        self.good(2)
        self.assertEqual(self.m.pub_flag.messages, [])
        self.good(1)
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG])

    def test_disagreement_requires_three_new_matching_results(self):
        self.good(2)
        self.m.stable_publish_board1(['A', '', '', ''])
        self.good(2)
        self.assertEqual(self.m.pub_flag.messages, [])
        self.good(1)
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG])

    def test_one_publication_per_visit_and_reset(self):
        self.good(6)
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG])
        self.m.nav_state_cb(Int32(data=9))
        self.m.nav_state_cb(Int32(data=10))
        self.good(3)
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG, EXPECTED_MSG])

    def test_camera_reader_keeps_default_api(self):
        self.m.latest_ros_frame = self.frame
        self.m.latest_ros_frame_recv_time = time.time()
        ok, frame, source = self.m.read_frame_from_configured_source(None)
        self.assertTrue(ok)
        self.assertEqual(source, 'ros')
        self.assertEqual(frame.shape, self.frame.shape)

    def test_cached_frame_identity_is_stable_until_new_receipt(self):
        self.m.latest_ros_frame = self.frame
        self.m.latest_ros_frame_recv_time = time.time()
        first = self.m.read_frame_from_configured_source(None, with_identity=True)[2]
        second = self.m.read_frame_from_configured_source(None, with_identity=True)[2]
        self.assertEqual(first, second)
        self.m.latest_ros_frame_recv_time += 0.01
        third = self.m.read_frame_from_configured_source(None, with_identity=True)[2]
        self.assertNotEqual(first, third)

    def run_main_fixture(self, fresh):
        m = self.m
        m.current_nav_state = m.STATE_BOARD1_RECOGNIZING
        m.latest_ros_frame = self.frame
        m.latest_ros_frame_recv_time = time.time()
        checks, reads = [0], [0]
        original_read = m.read_frame_from_configured_source

        def shutdown():
            checks[0] += 1
            return checks[0] > 7

        def read(cap, with_identity=False):
            reads[0] += 1
            if fresh:
                m.latest_ros_frame_recv_time = time.time() + reads[0] * 0.001
            return original_read(cap, with_identity=with_identity)

        class Rate(object):
            def sleep(self):
                pass

        with attrs(m, {'init_ros_node_and_publishers': lambda: None,
                       'load_board2_templates': lambda: {},
                       'CAMERA_FALLBACK_TO_HTTP': False,
                       'read_frame_from_configured_source': read}), \
                attrs(rospy, {'is_shutdown': shutdown, 'Rate': lambda hz: Rate()}):
            m.main()

    def test_main_does_not_count_repeated_cached_frame(self):
        self.run_main_fixture(False)
        self.assertEqual(self.m.pub_flag.messages, [])
        self.assertEqual(len(self.m.board1_history), 1)

    def test_main_counts_new_receipts_and_publishes(self):
        self.run_main_fixture(True)
        self.assertEqual(self.m.pub_flag.messages, [EXPECTED_MSG])


if __name__ == '__main__':
    unittest.main(verbosity=2)
