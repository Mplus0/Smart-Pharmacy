cmake_minimum_required(VERSION 3.0.2)
project(pharmacy_mplus0)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  geometry_msgs
  move_base_msgs
  nav_msgs
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dual_car_link.py
  scripts/main_controller.py
  scripts/referee_reporter.py
  scripts/vision_node.py
  scripts/waypoint_tester.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch models resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md STRUCTURE_REFACTOR_PLAN.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
