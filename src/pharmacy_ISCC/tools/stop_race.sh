#!/usr/bin/env bash
# 停止单车合并入口或原基础/业务分开入口，由 roslaunch 清理子节点。
# 使用 TERM：准备脚本在后台启动的进程可能继承忽略 SIGINT 的行为。
pkill -TERM -u "$(id -u)" -f '[r]oslaunch( --skip-log-check)?( --log)? pharmacy_ISCC single_car[.]launch' || true
pkill -TERM -u "$(id -u)" -f '[r]oslaunch( --skip-log-check)?( --log)? pharmacy_ISCC base_system[.]launch' || true
