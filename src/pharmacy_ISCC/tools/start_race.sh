#!/usr/bin/env bash
# 环境初始化完成、确认定位后，手动运行本脚本开始比赛。
set -e
source ~/robot_ws/devel/setup.bash
rosservice call /nav_pharmacy/start '{}'
