#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['pharmacy_iscc', 'pharmacy_iscc.common', 'pharmacy_iscc.config', 'pharmacy_iscc.perception', 'pharmacy_iscc.navigation', 'pharmacy_iscc.communication', 'pharmacy_iscc.recording'],
    package_dir={"": "src"},
))
