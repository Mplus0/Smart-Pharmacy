#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Verify first-stage pure logic with fake board1 data.

这个脚本是第一阶段的"离线自检"：
- 不启动 ROS。
- 不读摄像头。
- 不连接 move_base 或 TCP。

它只验证任务规划和样本记录两件事，确保后续接入真实 ROS 节点前，
核心策略已经能用普通 Python 假数据跑通。
"""

from __future__ import print_function

import os
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    # 允许直接运行 python pharmacy_mplus0/scripts/verify_logic.py。
    # 在 catkin 环境中安装后通常不需要这一步，但源码调试时很方便。
    sys.path.insert(0, SRC)

from pharmacy_mplus0.models import make_board1_detection
from pharmacy_mplus0.sample_store import SampleStore
from pharmacy_mplus0.task_planner import TaskPlanner


def assert_equal(actual, expected, label):
    # 简单断言工具。写成函数是为了在失败时输出更明确的字段名。
    if actual != expected:
        raise AssertionError("%s: expected %r, got %r" % (label, expected, actual))


def verify_task_planner():
    # 构造同一帧中可能识别到的多个二维码。
    # 按策略应该优先选择 ABC，因为它一次配送三个样本，收益最高。
    planner = TaskPlanner()
    detections = [
        make_board1_detection("A", 0),
        make_board1_detection("AB", 1),
        make_board1_detection("ABC", 3),
    ]

    best = planner.select_best(detections)
    assert_equal(best.code, "ABC", "best QR code")
    assert_equal(best.lab_window, "4", "best lab window")

    plan = planner.build_round_plan(best)
    # ABC 的固定访问顺序应为 C -> A -> B，而不是字母顺序 A -> B -> C。
    assert_equal(plan.code, "ABC", "round code")
    assert_equal(plan.lab_window, "4", "round lab window")
    assert_equal(plan.sample_type, "4", "round sample type")
    assert_equal(plan.exam_windows, ["C", "A", "B"], "ABC visit order")

    from_cam = planner.from_cam_return([1, 1, 0, 2, 2, 0])
    # 旧 /cam_return 前三位是 C、A、B。
    # [1, 1, 0, 2, 2, 0] 表示 AC，方框 2，对应化验窗口 3。
    assert_equal(from_cam.code, "AC", "cam_return QR code")
    assert_equal(from_cam.lab_window, "3", "cam_return lab window")


def verify_sample_store():
    # 模拟一轮 ABC -> 化验窗口 4 的取样和投递过程。
    store = SampleStore()
    assert_equal(store.add_sample("C", "4"), 1, "add C")
    assert_equal(store.add_sample("A", "4"), 2, "add A")
    assert_equal(store.add_sample("B", "4"), 3, "add B")
    assert_equal(store.count_for_lab("4"), 3, "count lab 4")
    assert_equal(store.count_for_lab("1"), 0, "count wrong lab")
    assert_equal(store.deliver_to_lab("4"), 3, "deliver lab 4")
    assert_equal(store.carried_windows(), [], "store cleared")

    store.add_sample("A", "1")
    try:
        # 比赛规则要求一轮只带一种样本类型。
        # 已经取了样本类型 1 后，再取类型 2 应该报错。
        store.add_sample("B", "2")
    except ValueError:
        pass
    else:
        raise AssertionError("mixing sample types should fail")


def main():
    # 两类纯逻辑都通过后，说明第一阶段核心策略可以继续往 ROS 封装推进。
    verify_task_planner()
    verify_sample_store()
    print("first-stage logic verification passed")


if __name__ == "__main__":
    main()
