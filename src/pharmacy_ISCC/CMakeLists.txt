cmake_minimum_required(VERSION 3.0.2)
project(pharmacy_iscc)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  rosgraph
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  actionlib_msgs
  tf
  visualization_msgs
  cv_bridge
)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/F1_detect_code_v5.py
  scripts/F1_yaofang_v5.py
  scripts/tcp_link_v2.py
  scripts/referee_client_v3.py
  scripts/record_waypoint.py
  scripts/waypoint_console.py
  scripts/test_move_to_waypoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config resources docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
