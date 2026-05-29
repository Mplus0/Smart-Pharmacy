cmake_minimum_required(VERSION 3.0.2)
project(pharmacy_mplus0)

# 主包当前阶段只需要 rospy/std_msgs。
# 后续导航、TF、actionlib 相关依赖主要在 package.xml 中声明运行依赖。
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# 启用 setup.py，使 src/pharmacy_mplus0 可以作为 Python 包被 ROS 脚本导入。
catkin_python_setup()

catkin_package()

# 第一阶段的离线验证脚本。它不依赖 ROS master，方便在开发机上直接跑。
catkin_install_python(PROGRAMS
  scripts/verify_logic.py
  scripts/board1_detector.py
  scripts/board2_detector.py
  scripts/tcp_reporter.py
  scripts/main_controller.py
  scripts/detection_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装配置、launch 和模板资源。
install(DIRECTORY config launch templates
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
