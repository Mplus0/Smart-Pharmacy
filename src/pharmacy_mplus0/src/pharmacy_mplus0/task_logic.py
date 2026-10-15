#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
task_logic.py

识别板一 all_text -> /cam_return 结果的公共选择逻辑。
"""


VALID_COMBO_COUNT = {
    "": 0,
    "A": 1,
    "B": 1,
    "C": 1,
    "AB": 2,
    "AC": 2,
    "BC": 2,
    "ABC": 3,
}

NORMAL_TEXT = set(VALID_COMBO_COUNT.keys())


def safe_to_str(item):
    """兼容 Python2/3，把二维码结果统一转成普通字符串。"""
    if item is None or item == []:
        return ""

    try:
        if isinstance(item, unicode):
            return item.encode("utf-8").strip()
    except NameError:
        pass

    if isinstance(item, str):
        return item.strip()

    return str(item).strip()


def normalize_all_text(all_data):
    return [safe_to_str(item) for item in all_data]


def detect_error_window(all_text):
    for idx, item in enumerate(all_text):
        if item not in NORMAL_TEXT:
            return idx + 1
    return 0


def build_msg_from_selected_text(selected_text, selected_index, error_count):
    if selected_text == "ABC":
        return [1, 1, 1, 3, selected_index, error_count]
    if selected_text == "AB":
        return [0, 1, 1, 2, selected_index, error_count]
    if selected_text == "AC":
        return [1, 1, 0, 2, selected_index, error_count]
    if selected_text == "BC":
        return [1, 0, 1, 2, selected_index, error_count]
    if selected_text == "A":
        return [0, 1, 0, 1, selected_index, error_count]
    if selected_text == "B":
        return [0, 0, 1, 1, selected_index, error_count]
    if selected_text == "C":
        return [1, 0, 0, 1, selected_index, error_count]
    return None


def select_board1_from_all_text(all_data):
    """
    返回 dict:
      all_text, selected_index, selected_text, selected_msg
    无有效二维码时返回 None。
    """
    if not isinstance(all_data, (list, tuple)):
        return None

    all_text = normalize_all_text(all_data)
    if len(all_text) == 0:
        return None

    scores = [VALID_COMBO_COUNT.get(item, 0) for item in all_text]
    max_score = max(scores)
    if max_score == 0:
        return None

    selected_index = scores.index(max_score)
    selected_text = all_text[selected_index]
    error_count = detect_error_window(all_text)
    selected_msg = build_msg_from_selected_text(
        selected_text,
        selected_index,
        error_count
    )

    if selected_msg is None:
        return None

    return {
        "all_text": all_text,
        "selected_index": selected_index,
        "selected_text": selected_text,
        "selected_msg": selected_msg,
    }
