#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Load the package YAML files while preserving the legacy config interface."""

import os

import rospkg
import yaml


STATES = {
    "WAIT_TURN": 8,
    "GO_TO_BOARD1": 9,
    "BOARD1_RECOGNIZING": 10,
    "GO_TO_PICKUP_WINDOWS": 11,
    "GO_TO_BOARD2": 12,
    "BOARD2_RECOGNIZING": 13,
    "GO_TO_LAB_WINDOW": 14,
    "GO_BACK_HOME": 15,
}


def _package_path():
    return rospkg.RosPack().get_path("pharmacy_mplus0")


def _normalize_yaml_value(value):
    """Keep Python 2 string behavior equivalent to the original source literals."""
    try:
        unicode_type = unicode
    except NameError:
        unicode_type = None

    if unicode_type is not None and isinstance(value, unicode_type):
        return value.encode("utf-8")
    if isinstance(value, dict):
        return dict(
            (_normalize_yaml_value(key), _normalize_yaml_value(item))
            for key, item in value.items()
        )
    if isinstance(value, list):
        return [_normalize_yaml_value(item) for item in value]
    return value


def _load_yaml(filename):
    path = os.path.join(_package_path(), "config", filename)
    with open(path, "r") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("配置文件必须包含字典: %s" % path)
    return _normalize_yaml_value(data)


def _as_tuple(value, field_name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("%s 必须是三个元素的序列" % field_name)
    return tuple(value)


def _build_config():
    package_path = _package_path()
    strategy = _load_yaml("strategy.yaml")
    waypoint_data = _load_yaml("waypoints.yaml")
    vision = _load_yaml("vision.yaml")
    communication = _load_yaml("communication.yaml")

    nav = dict(strategy["nav"])
    nav["euler_angles"] = list(waypoint_data["euler_angles"])
    nav["waypoints"] = dict(
        (name, _as_tuple(value, "waypoints.%s" % name))
        for name, value in waypoint_data["waypoints"].items()
    )

    board2 = dict(vision["board2"])
    board2["roi"] = tuple(board2["roi"])

    common = {
        "paths": {
            "audio_dir": os.path.join(package_path, "resources", "audio"),
            "board2_template_dir": os.path.join(package_path, "resources", "board2"),
        },
        "topics": dict(communication["topics"]),
        "states": dict(STATES),
        "board1": dict(vision["board1"]),
        "board2": board2,
        "detect": dict(vision["detect"]),
        "nav": nav,
        "referee": dict(communication["referee"]),
        "dual_tcp": dict(communication["dual_tcp"]),
        "lab_info": dict(strategy["lab_info"]),
        "window_log_name": dict(strategy["window_log_name"]),
    }

    cars = {}
    for car_id in sorted(strategy["cars"].keys()):
        car = dict(strategy["cars"][car_id])
        car["camera_url"] = vision["camera_url"][car_id]
        car["home_pose"] = _as_tuple(
            waypoint_data["home_pose"][car_id],
            "home_pose.%s" % car_id
        )
        car["tcp"] = dict(communication["cars"][car_id])
        cars[int(car_id)] = car

    return common, cars


COMMON, CARS = _build_config()


def get_car_id(default=1):
    """Read CAR_ID from the environment, preserving the legacy validation."""
    value = os.environ.get("CAR_ID", str(default))
    try:
        car_id = int(value)
    except Exception:
        raise ValueError("CAR_ID 必须是整数，当前为: %r" % value)
    if car_id not in CARS:
        raise ValueError("CAR_ID 只支持 %s，当前为: %s" % (sorted(CARS.keys()), car_id))
    return car_id


def get_car_config(car_id=None, default=1):
    """Return the selected car configuration."""
    if car_id is None:
        car_id = get_car_id(default=default)
    return CARS[int(car_id)]
