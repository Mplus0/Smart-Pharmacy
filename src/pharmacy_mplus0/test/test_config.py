#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import unittest

from pharmacy_mplus0.config import CARS, COMMON, get_car_config


class ConfigTest(unittest.TestCase):

    def test_state_numbers(self):
        self.assertEqual({
            "WAIT_TURN": 8,
            "GO_TO_BOARD1": 9,
            "BOARD1_RECOGNIZING": 10,
            "GO_TO_PICKUP_WINDOWS": 11,
            "GO_TO_BOARD2": 12,
            "BOARD2_RECOGNIZING": 13,
            "GO_TO_LAB_WINDOW": 14,
            "GO_BACK_HOME": 15,
        }, COMMON["states"])

    def test_behavior_sensitive_defaults(self):
        self.assertEqual("free", COMMON["board2"]["force_label_for_debug"])
        self.assertEqual(0, COMMON["nav"]["board1_wait_timeout_sec"])
        self.assertEqual(0, COMMON["nav"]["board2_wait_timeout_sec"])
        self.assertEqual(3, COMMON["board1"]["stable_frames"])
        self.assertEqual(2, COMMON["board2"]["stable_frames"])
        self.assertEqual(False, COMMON["board1"]["direct_decode_enable"])
        self.assertEqual(True, COMMON["dual_tcp"]["check_peer_ip"])
        self.assertEqual(None, COMMON["dual_tcp"]["shared_token"])

    def test_waypoints(self):
        self.assertEqual({
            "C": (1.410, 1.828, 0),
            "A": (0.685, 2.450, 8),
            "B": (1.385, 2.852, 2),
            "lab4": (-1.000, 0.800, 3),
            "lab3": (-1.800, 1.240, 4),
            "lab2": (-1.100, 1.600, 7),
            "lab1": (-1.800, 2.300, 6),
            "board2": (-0.454, 3.791, 8),
            "board1": (0.591, -0.269, 9),
        }, COMMON["nav"]["waypoints"])

    def test_car_identity_and_home(self):
        self.assertEqual(True, CARS[1]["start_active"])
        self.assertEqual(False, CARS[2]["start_active"])
        self.assertEqual(False, CARS[1]["use_peer_board1_result"])
        self.assertEqual(True, CARS[2]["use_peer_board1_result"])
        self.assertEqual((0.0, 0.0, 7), get_car_config(1)["home_pose"])
        self.assertEqual((-0.069, -0.269, 7), get_car_config(2)["home_pose"])

    def test_package_resource_paths(self):
        audio_parts = os.path.normpath(COMMON["paths"]["audio_dir"]).split(os.sep)
        board2_parts = os.path.normpath(COMMON["paths"]["board2_template_dir"]).split(os.sep)
        self.assertEqual(["resources", "audio"], audio_parts[-2:])
        self.assertEqual(["resources", "board2"], board2_parts[-2:])


if __name__ == "__main__":
    unittest.main()
