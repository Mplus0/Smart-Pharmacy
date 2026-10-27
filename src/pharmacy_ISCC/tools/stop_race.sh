#!/usr/bin/env bash
# 停止当前用户启动的单车业务及基础环境，由 roslaunch 清理子节点。
# 使用 TERM：准备脚本在后台启动的进程可能继承忽略 SIGINT 的行为。
pkill -TERM -u "$(id -u)" -f '[r]oslaunch pharmacy_ISCC single_car[.]launch' || true
pkill -TERM -u "$(id -u)" -f '[r]oslaunch pharmacy_ISCC base_system[.]launch' || true
