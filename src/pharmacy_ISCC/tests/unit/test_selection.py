"""Offline regressions for the wire format and task priority."""
import itertools
import unittest

from pharmacy_iscc.perception.selection import build_board1_task_queue, select_board1_from_all_text


class SelectionTests(unittest.TestCase):
    def test_all_four_window_combinations(self):
        legal = ["A", "B", "C", "AB", "AC", "BC", "ABC"]
        for values in itertools.product([""] + legal + ["D", "ACD"], repeat=4):
            expected = sorted(((-len(text), index, text) for index, text in enumerate(values)
                               if text in legal))
            tasks = build_board1_task_queue(values)
            self.assertEqual(len(tasks), len(expected))
            error = next((i + 1 for i, text in enumerate(values) if text and text not in legal), 0)
            for task, (_, index, text) in zip(tasks, expected):
                self.assertEqual(task, [int("C" in text), int("A" in text), int("B" in text),
                                        len(text), index, error])
            selected = select_board1_from_all_text(values)
            self.assertEqual(selected["selected_msg"] if selected else None, tasks[0] if tasks else None)

    def test_invalid_and_empty_input(self):
        for value in [None, "", 1, {}, [], ["D", "", "", ""]]:
            self.assertIsNone(select_board1_from_all_text(value))
            self.assertEqual(build_board1_task_queue(value), [])

    def test_input_not_mutated_and_ties_keep_window_order(self):
        values = ["AB", "AC", "BC", "ABC"]
        original = values[:]
        self.assertEqual([task[4] for task in build_board1_task_queue(values)], [3, 0, 1, 2])
        self.assertEqual(values, original)
