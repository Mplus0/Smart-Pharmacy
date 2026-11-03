#!/usr/bin/env bash
# 终端二：启动即等待真正就绪并自动放行一次，不再要求手动重试命令。
set -e
source "${ISCC_WORKSPACE:-$HOME/robot_ws}/devel/setup.bash"
echo '开始按钮已按下，等待系统就绪后自动发车。'
exec /usr/bin/python -u - <<'PY'
# -*- coding: utf-8 -*-
from __future__ import print_function
import sys
import time
import rosgraph
import rospy
from std_srvs.srv import Trigger

try:
    master = rosgraph.Master('/race_start_button')
    while True:
        try:
            master.getPid()
            break
        except Exception:
            time.sleep(0.1)
    rospy.init_node('race_start_button', anonymous=True, disable_signals=True, disable_rosout=True)
    rospy.wait_for_service('/nav_pharmacy/start')
    response = rospy.ServiceProxy('/nav_pharmacy/start', Trigger)()
    if response.success:
        print('已放行，小车开始比赛。')
    else:
        print('未重复发车：' + response.message)
        sys.exit(1)
except (rospy.ROSException, rospy.ServiceException) as exc:
    print('开始请求失败：' + str(exc))
    sys.exit(1)
except KeyboardInterrupt:
    print('已取消开始请求。')
    sys.exit(130)
PY
