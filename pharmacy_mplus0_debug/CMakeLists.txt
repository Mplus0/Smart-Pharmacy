cmake_minimum_required(VERSION 3.0.2)
project(pharmacy_mplus0_debug)

# 调试包只放现场调试脚本，不参与正式比赛主流程。
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# 安装所有调试脚本和 launch 文件。
catkin_install_python(PROGRAMS
  scripts/board1_initial_debugger.py
  scripts/board2_template_capture.py
  scripts/send_fake_cam_return.py
  scripts/send_fake_cv1.py
  scripts/send_fake_task.py
  scripts/waypoint_tester.py
  scripts/topic_echo_dashboard.py
  scripts/tcp_fake_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
