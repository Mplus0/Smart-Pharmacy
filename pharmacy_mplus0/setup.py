#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# catkin_python_setup 会读取这个 setup.py。
# package_dir 指向 src，表示 pharmacy_mplus0 Python 包位于 src/pharmacy_mplus0。
setup_args = generate_distutils_setup(
    packages=["pharmacy_mplus0"],
    package_dir={"": "src"},
)

setup(**setup_args)
