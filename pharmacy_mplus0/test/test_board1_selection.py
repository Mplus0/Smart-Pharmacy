#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import unittest

from pharmacy_mplus0.task_logic import select_board1_from_all_text


class Board1SelectionTest(unittest.TestCase):

    def test_fixture_cases(self):
        fixture_path = os.path.join(
            os.path.dirname(__file__),
            "fixtures",
            "board1_selection_cases.json"
        )
        with open(fixture_path, "r") as stream:
            cases = json.load(stream)

        for case in cases:
            actual = select_board1_from_all_text(case["all_text"])
            self.assertEqual(case["expected"], actual, case["name"])


if __name__ == "__main__":
    unittest.main()
