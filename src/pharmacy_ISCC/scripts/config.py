# -*- coding: utf-8 -*-
"""Load the three YAML sources and resolve resources and writable records."""
import os

import rospkg
import yaml


def package_path(*parts):
    return os.path.join(rospkg.RosPack().get_path("pharmacy_ISCC"), *parts)


def records_path():
    return os.path.abspath(os.path.expanduser(os.environ.get(
        "PHARMACY_RECORD_DIR",
        os.path.join(os.environ.get("ROS_HOME", "~/.ros"),
                     "pharmacy_ISCC", "result_md5"))))


def get_car_id(default=1):
    """从环境变量 CAR_ID 获取车号；没有设置时使用 default。"""
    value = os.environ.get("CAR_ID", str(default))
    try:
        car_id = int(value)
    except Exception:
        raise ValueError("CAR_ID 必须是整数，当前为: %r" % value)
    if car_id not in CARS:
        raise ValueError("CAR_ID 只支持 %s，当前为: %s" % (sorted(CARS.keys()), car_id))
    return car_id


def get_car_config(car_id=None, default=1):
    """获取指定车号配置。"""
    if car_id is None:
        car_id = get_car_id(default=default)
    return CARS[int(car_id)]


def load_config(filename):
    with open(package_path("config", filename), "rb") as stream:
        return yaml.safe_load(stream)


COMMON = load_config("competition.yaml")
COMMON.update(load_config("vision.yaml"))
COMMON["paths"] = {
    "audio_dir": package_path("resources", "audio"),
    "board2_template_dir": package_path("resources", "board2_templates"),
}
CARS = load_config("vehicles.yaml")

# Restore the tuple contracts of the original Python configuration.
COMMON["board2"]["roi"] = tuple(COMMON["board2"]["roi"])
for name, point in COMMON["nav"]["waypoints"].items():
    COMMON["nav"]["waypoints"][name] = tuple(point)
for car in CARS.values():
    car["home_pose"] = tuple(car["home_pose"])
