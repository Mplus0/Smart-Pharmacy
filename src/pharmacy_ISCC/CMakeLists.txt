cmake_minimum_required(VERSION 3.0.2)
project(pharmacy_ISCC)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

# catkin creates executable devel relays and installs executable node scripts.
catkin_install_python(PROGRAMS
  scripts/F1_detect_code_v5.py
  scripts/F1_yaofang_v5.py
  scripts/referee_client_v3.py
  scripts/tcp_link_v2.py
  tools/record_waypoint.py
  tools/waypoint_console.py
  tools/test_move_to_waypoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md AGENTS.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
