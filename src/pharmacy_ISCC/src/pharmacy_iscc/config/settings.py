# -*- coding: utf-8 -*-
"""Single configuration loader; values match the current pharmacy_pkg baseline."""
import os
import yaml
from pharmacy_iscc.config.paths import package_path


def _load(filename):
    with open(package_path("config", filename), "rb") as stream:
        return yaml.safe_load(stream)


COMMON = _load("competition.yaml")
COMMON.update(_load("vision.yaml"))
COMMON["lab_info"] = {int(key): value for key, value in COMMON["lab_info"].items()}
COMMON["board2"]["roi"] = tuple(COMMON["board2"]["roi"])
COMMON["nav"]["waypoints"] = {key: tuple(value) for key, value in COMMON["nav"]["waypoints"].items()}
COMMON["paths"] = {
    "audio_dir": package_path("resources", "audio"),
    "board2_template_dir": package_path("resources", "board2_templates"),
}
CARS = {int(key): value for key, value in _load("vehicles.yaml").items()}
for _car in CARS.values():
    _car["home_pose"] = tuple(_car["home_pose"])

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
