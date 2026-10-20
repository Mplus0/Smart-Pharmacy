#!/usr/bin/env python
# -*- coding: utf-8 -*-
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Includes detectors, audio, config and navigation_utils alongside node modules.
# Keep the installed namespace independent of the ROS package's mixed-case name.
setup(**generate_distutils_setup(
    packages=["pharmacy_iscc"],
    package_dir={"pharmacy_iscc": "scripts"},
))
