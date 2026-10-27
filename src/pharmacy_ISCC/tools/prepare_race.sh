#!/usr/bin/env bash
# 启动基础环境和单车业务，等待比赛放行。
set -e
source ~/robot_ws/devel/setup.bash

roslaunch pharmacy_ISCC base_system.launch use_race_init:=false open_rviz:=false &
base_pid=$!
roslaunch pharmacy_ISCC single_car.launch &
business_pid=$!

# 退出准备终端时，让 roslaunch 清理其启动的节点。
trap 'kill -TERM "$business_pid" "$base_pid" 2>/dev/null || true; wait' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait
