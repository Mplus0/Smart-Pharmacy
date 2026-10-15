# Smart-Pharmacy

ROS1 Melodic 双车智慧药房运行源码。主功能包为 `src/pharmacy_pkg`。

## 启动

在两辆车上分别运行（车 2 将 `car_id:=1` 改为 `car_id:=2`）：

```bash
source /opt/ros/melodic/setup.bash
cd ~/robot_ws
catkin_make
source devel/setup.bash
# 按实际硬件设置 EPRobotV2.2 或 EPRobotV2.3；不要按车号推断硬件型号。
export ROBOT_TYPE=EPRobotV2.3
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1
# 另开终端，source 相同环境：
roslaunch pharmacy_pkg referee_only.launch car_id:=1
```

默认导航走 `robot_race_init.launch` 和 talos 定位；`use_race_init:=false` 走 AMCL。
两条路径均使用 EKF、move_base、TEB、底盘驱动和按 ROBOT_TYPE 选择的雷达。

## 运行依赖

四个业务节点为 `tcp_link_v2.py`、`F1_detect_code_v5.py`、`F1_yaofang_v5.py`、`referee_client_v3.py`。
共享模块为 `dual_car_config.py`、`board1_selection.py`。car1 主动开始，car2 等待并复用板一结果。
语音由 SoX `play` 播放，不依赖语音识别功能包。板二使用 PNG 模板，不依赖训练数据集。

ROS 系统依赖还包括 web_video_server、rgbd_launch、nodelet、cv_bridge、消息包、TF、serial、costmap_converter、libg2o 等；完整声明见各包 package.xml。
Python 环境需要 OpenCV、NumPy、pyzbar（及 zbar 系统库）、pyserial；Python 2 还需要 enum34。
保持目标车原有 Melodic/Python 环境，不在本次清理中迁移 Python 版本。

## 保留配置和部署约束

业务参数在 `src/pharmacy_pkg/scripts/dual_car_config.py`。资源路径仍为 `/home/EPRobot/robot_ws/src/pharmacy_pkg`，需按现有方式部署。
47 个 WAV、7 个板二 PNG、两车 IP/端口、状态机和通信协议保持原样。
`force_label_for_debug` 原值为 `free`，本次清理未修改，部署者需知晓这一原有行为。

## 原有阻塞

- `robot_navigation/maps/map.yaml` 指向缺失的 `compitation..pgm`。现有 `map.pgm` 不能未经确认就视作同一地图。
- `talos_laser_loc` 缺失 `src/talos_laser_loc.cpp` 和 `src/talos_costmap_cleaner.cpp`；默认入口无法仅靠本仓库重建定位节点。
- `src/CMakeLists.txt` 在 Git 中是指向 Melodic catkin 的符号链接。Windows 将它物化为文本；应在原生 Linux 正确检出符号链接。
- 保留上游导航/相机/滤波器的测试、动态插件和硬件库；外部 rgbd_launch 的节点加载需在目标机核实。

未补齐原有缺件前，不能宣称完整 catkin 编译或实车运行通过。

清理范围、删除/保留包清单和验证结果见 [清理报告](cleanup-report/清理报告.md)。
