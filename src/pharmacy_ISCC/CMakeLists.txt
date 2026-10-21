cmake_minimum_required(VERSION 3.0.2)
project(pharmacy_ISCC)

find_package(catkin REQUIRED)
catkin_package()

# Melodic requires package_dir basenames to match Python package names.
# Keep the flat scripts layout and expose pharmacy_iscc in devel/install.
set(ISCC_PYTHON_DESTINATION "${CATKIN_GLOBAL_PYTHON_DESTINATION}/pharmacy_iscc")
file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${ISCC_PYTHON_DESTINATION}")
file(WRITE "${CATKIN_DEVEL_PREFIX}/${ISCC_PYTHON_DESTINATION}/__init__.py"
  "__path__ = [r'${CMAKE_CURRENT_SOURCE_DIR}/scripts']\n")
install(DIRECTORY scripts/
  DESTINATION "${ISCC_PYTHON_DESTINATION}"
  FILES_MATCHING PATTERN "*.py"
)

# Only ROS node/tool entry points are executable programs.
# The CMake rules above install the flat scripts/ package as pharmacy_iscc.
# Melodic uses executable source scripts in devel; catkin installs node scripts.
catkin_install_python(PROGRAMS
  scripts/F1_detect_code_v5.py
  scripts/F1_yaofang_v5.py
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
