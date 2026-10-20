# -*- coding: utf-8 -*-
"""Resolve installed ROS resources separately from writable runtime records."""
import os

import rospkg


def package_path(*parts):
    return os.path.join(rospkg.RosPack().get_path("pharmacy_ISCC"), *parts)


def records_path():
    return os.path.abspath(os.path.expanduser(os.environ.get(
        "PHARMACY_RECORD_DIR",
        os.path.join(os.environ.get("ROS_HOME", "~/.ros"),
                     "pharmacy_ISCC", "result_md5"))))
