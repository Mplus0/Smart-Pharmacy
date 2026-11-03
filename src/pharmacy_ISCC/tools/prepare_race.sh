#!/usr/bin/env bash
# 终端一：一个 roslaunch 并行启动全部单车功能，只显示比赛任务信息。
set -e
source "${ISCC_WORKSPACE:-$HOME/robot_ws}/devel/setup.bash"

package_dir="$(rospack find pharmacy_ISCC)"
log_dir="${ROS_HOME:-$HOME/.ros}/pharmacy_ISCC/startup_logs"
mkdir -p "$log_dir"
launch_log="$log_dir/prepare-$(date +%Y%m%d-%H%M%S)-$$.log"

launch_pid=""
status_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$status_pid" ]]; then kill -TERM "$status_pid" 2>/dev/null || true; fi
  if [[ -n "$launch_pid" ]]; then kill -TERM "$launch_pid" 2>/dev/null || true; fi
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo '正在启动比赛系统，请等待“系统已就绪”提示。'
roslaunch --skip-log-check --log pharmacy_ISCC single_car.launch \
  start_base:=true fast_start:=true output:=log max_rounds:=1 "$@" \
  >"$launch_log" 2>&1 &
launch_pid=$!

/usr/bin/python -u - <<'PY' 2>>"$launch_log" &
# -*- coding: utf-8 -*-
from __future__ import print_function
import signal
import sys
import threading
import time
import rosgraph
import rospy
from rosgraph_msgs.msg import Log

def write(text):
    if sys.version_info[0] < 3 and isinstance(text, unicode):
        text = text.encode('utf-8')
    print(text)

def stop(signum, frame):
    if rospy.core.is_initialized():
        rospy.signal_shutdown('Race terminal closed')
    else:
        sys.exit(0)

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
master = rosgraph.Master('/race_terminal')
while True:
    try:
        master.getPid()
        break
    except Exception:
        time.sleep(0.1)

rospy.init_node('race_terminal', anonymous=True, disable_signals=True, disable_rosout=True)

def announce_ready():
    try:
        # 只探测服务可连接，不调用Trigger，不触发发车；不依赖可能漏收的单条日志。
        rospy.wait_for_service('/nav_pharmacy/start')
    except rospy.ROSException:
        return
    if not rospy.is_shutdown():
        write(u'【系统已就绪】相机、识别与导航初始化完成。')
        write(u'请保持终端一打开，在终端二执行：')
        write(u'bash ~/robot_ws/src/pharmacy_ISCC/tools/start_race.sh')

ready_thread = threading.Thread(target=announce_ready)
ready_thread.daemon = True
ready_thread.start()
markers = (u'已收到手动启动', u'正在前往', u'已经到达', u'已经过', u'送样完成',
           u'识别到二维码并已经选择', u'识别板二处理完成', u'化验区',
           u'本批已完成', u'本项配送', u'窗口取样流程', u'识别板一识别成功',
           u'识别板二到点后', u'[BOARD1-RECOVERY]', u'FULL RUN COMPLETE')

def status(msg):
    if msg.name not in ('/nav_pharmacy', '/detect_abc'):
        return
    text = msg.msg
    if isinstance(text, bytes):
        text = text.decode('utf-8', 'replace')
    if msg.level >= Log.ERROR:
        write(u'任务异常：' + text)
    elif msg.name == '/nav_pharmacy':
        if any(marker in text for marker in markers):
            write(text)

rospy.Subscriber('/rosout_agg', Log, status, queue_size=100)
rospy.spin()
PY
status_pid=$!

if wait "$launch_pid"; then
  echo '比赛系统已停止。'
else
  result=$?
  echo "比赛系统启动或运行失败，详细日志：$launch_log" >&2
  exit "$result"
fi
