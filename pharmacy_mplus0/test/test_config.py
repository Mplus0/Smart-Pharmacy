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
        self.assertEqual(0, COMMON["nav"]["board1_wait_timeout_sec"])
        self.assertEqual(0, COMMON["nav"]["board2_wait_timeout_sec"])
        self.assertEqual(3, COMMON["board1"]["stable_frames"])
        self.assertEqual(5, COMMON["board2"]["smoothing_window"])
        self.assertEqual(224, COMMON["board2"]["image_size"])
        self.assertEqual((229, 91, 108, 59), COMMON["board2"]["status_roi"])
        self.assertEqual((258, 150, 57, 52), COMMON["board2"]["number_roi"])
        self.assertEqual(["busy", "idle"], COMMON["board2"]["status_classes"])
        self.assertEqual(["10", "5", "6", "7", "8", "9"], COMMON["board2"]["number_classes"])
        self.assertEqual(False, COMMON["board1"]["direct_decode_enable"])
        self.assertEqual(True, COMMON["dual_tcp"]["check_peer_ip"])
        self.assertEqual(None, COMMON["dual_tcp"]["shared_token"])
        self.assertEqual(False, COMMON["nav"]["clear_costmaps_on_arrival"])
        self.assertEqual(True, COMMON["referee"]["send_only_when_active"])
        self.assertEqual(True, COMMON["referee"]["reset_task_cv_on_activate"])
        self.assertEqual("192.168.124.6", COMMON["referee"]["server_ip"])
        self.assertEqual("/referee_active", COMMON["topics"]["referee_active"])

    def test_waypoints(self):
        self.assertEqual({
            "C": (1.420, 2.060, 0),
            "A": (0.700, 2.550, 1),
            "B": (1.501, 3.050, 2),
            "lab4": (-0.807, 0.980, 3),
            "lab3": (-1.615, 1.580, 4),
            "lab2": (-0.830, 1.820, 7),
            "lab1": (-1.584, 2.530, 6),
            "board2": (-0.454, 3.791, 8),
            "board1": (0.750, 0.000, 9),
        }, COMMON["nav"]["waypoints"])

    def test_car_identity_and_home(self):
        self.assertEqual(True, CARS[1]["start_active"])
        self.assertEqual(False, CARS[2]["start_active"])
        self.assertEqual(False, CARS[1]["use_peer_board1_result"])
        self.assertEqual(True, CARS[2]["use_peer_board1_result"])
        self.assertEqual((0.0, 0.0, 7), get_car_config(1)["home_pose"])
        self.assertEqual((0.0, 0.0, 7), get_car_config(2)["home_pose"])

    def test_package_resource_paths(self):
        audio_parts = os.path.normpath(COMMON["paths"]["audio_dir"]).split(os.sep)
        board2_parts = os.path.normpath(COMMON["paths"]["board2_model_dir"]).split(os.sep)
        status_parts = os.path.normpath(COMMON["board2"]["status_model_path"]).split(os.sep)
        number_parts = os.path.normpath(COMMON["board2"]["number_model_path"]).split(os.sep)
        self.assertEqual(["resources", "audio"], audio_parts[-2:])
        self.assertEqual(["models", "board2"], board2_parts[-2:])
        self.assertEqual(["board2", "status_best.onnx"], status_parts[-2:])
        self.assertEqual(["board2", "number_best.onnx"], number_parts[-2:])


if __name__ == "__main__":
    unittest.main()
