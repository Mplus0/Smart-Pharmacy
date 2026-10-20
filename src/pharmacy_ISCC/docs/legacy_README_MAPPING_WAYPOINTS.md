# 建图、取点与单点导航操作

适用：EPRobot2，IP 192.168.124.6；逻辑 car_id=1；Ubuntu 18.04 / ROS Melodic / Python 2。通过 MobaXterm SSH + X11 查看 RViz。以下命令全部在登录小车后的 Linux 终端执行。

`test_move_to_waypoint.launch` 用于发送已配置的导航目标；本次新增 `record_waypoint.py` 读取当前位置和朝向。取点脚本没有速度或导航目标发布功能。

## 1. 进入小车并准备终端

MobaXterm 的 X server 应在运行，SSH 会话启用 X11 forwarding。每个新小车终端先执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/robot_ws/devel/setup.bash
```

需要显示 RViz 的终端检查：

```bash
echo "$DISPLAY"
```

正常应有 `localhost:10.0` 一类的值；空值表示该会话没有 X11 转发。当前小车 ROS 网络已配置为 .6，不要再使用旧的 .3 地址。

## 2. 建图

把车放在场地起点，车头对准计划出发方向。本次地图使用新文件 `~/maps/pharmacy_20261016.yaml`，保留功能包中的旧地图。若此文件已存在，重建时请换一个地图名，并同步替换后续命令中的路径。

终端 1：

```bash
roslaunch robot_navigation robot_slam_laser.launch \
  slam_methods:=gmapping open_rviz:=true
```

该文件原默认 Cartographer，这里明确选择车上已有的 Gmapping。默认不启动 move_base；此阶段不要同时运行 AMCL 导航或药房状态机。

终端 2：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  _speed:=0.08 _turn:=0.15 _repeat_rate:=10.0 _key_timeout:=0.6
```

保持焦点在遥控终端：`i` 前进，`,` 后退，`u/o` 向前弧线转弯，`m/.` 倒车转弯，`k` 停车。当前是车式底盘，转弯走弧线；不要用 `j/l` 期待原地转圈。先慢速绕外围，再补取样窗口、识别板、化验窗口附近，最后返回起点附近。RViz 中看地图和激光能否稳定重合；明显双墙、跳变时先停止采图检查定位链路。

查看传感器时可另开终端：

```bash
rostopic hz /scan_filtered
rostopic hz /odometry/filtered
```

## 3. 保存地图，再结束 SLAM

按 `k` 停稳，保持建图终端运行。终端 3：

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/pharmacy_20261016
ls -lh ~/maps/pharmacy_20261016.yaml ~/maps/pharmacy_20261016.pgm
```

必须确认两个文件都生成，且地图墙线、通道完整。之后在建图终端按 Ctrl+C 结束 SLAM，再启动下一步定位。地图使用 YAML 元数据和 PGM 图像共同加载；不要只移动或修改其中一个。

## 4. 加载新地图，初始化定位

终端 1：

```bash
roslaunch robot_navigation robot_navigation.launch \
  map_file:=$HOME/maps/pharmacy_20261016.yaml open_rviz:=true
```

在 RViz 使用 **2D Pose Estimate**，在车实际位置按下并拖出箭头，箭头方向沿真实车头。缓慢走一小段再停，确认激光与地图墙线重合。此处定位与取点阶段先不用 **2D Nav Goal**。

建图终端退出后原遥控终端若也退出了，可重新运行第 2 步的遥控命令。

## 5. 遥控到位后取点

每个目标都按实际任务停车姿态对准：识别板要在相机里完整且可读，窗口要按实际取/放样的位置和车头方向停靠。按 `k` 停稳约 1 秒，在另一个终端执行记录。例如记录 A：

```bash
rosrun pharmacy_pkg record_waypoint.py A \
  --map-file ~/maps/pharmacy_20261016.yaml \
  --output ~/waypoints/pharmacy_20261016/points.json
```

只替换命令中的 `A`，依次记录：

```text
home  board1  A  B  C  board2  lab1  lab2  lab3  lab4
```

脚本从 `map -> base_footprint` 获取 x、y、yaw 和四元数；检查新鲜里程计和停车状态，记录地图文件指纹与实际 `/map` 内容指纹。拒绝缺失/过期 TF、地图或车辆混用，以及没有明确确认的同名重录。地图在初次采集时还会核对分辨率与原点；因此必须用第 4 步命令明确加载对应地图。

出现 `SAVED A: x=... y=... yaw=...` 才表示保存成功。`NOT SAVED` 时按提示检查，不会留下成功点位。

查看已采和缺失点：

```bash
rosrun pharmacy_pkg record_waypoint.py --list \
  --output ~/waypoints/pharmacy_20261016/points.json
```

重新记录某点时加 `--replace`，例如：

```bash
rosrun pharmacy_pkg record_waypoint.py A \
  --map-file ~/maps/pharmacy_20261016.yaml \
  --output ~/waypoints/pharmacy_20261016/points.json --replace
```

保存前会保留旧 JSON 的 `.bak.时间戳` 副本。地图重建后使用新输出目录，不把两张地图的坐标合并。

## 6. 写入配置后再试走

采齐后核对 `points.json`，再更新 `scripts/dual_car_config.py` 中的坐标与朝向。

当前配置为 `(x, y, 朝向索引)`，第三项不是 yaw 弧度；实际角度在 `COMMON['nav']['euler_angles']`。旧配置的 lab2 与 home 使用同一角度索引 7，写入新点时应处理为各自明确的角度，避免相互影响。取点脚本不会自动改导航配置。

完成新地图点位配置后，仍保持基础导航运行，先检查目标，再实际试走：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch \
  car_id:=1 target:=A dry_run:=true

roslaunch pharmacy_pkg test_move_to_waypoint.launch \
  car_id:=1 target:=A timeout_sec:=60
```

测试自动导航时先退出键盘遥控，避免它持续发布零速度或与 move_base 同时控制底盘。逐个测试其他目标，确认位置、方向以及相机视野。未写入新点位前不要发送旧配置的实际导航目标。

## 准备与验证状态

小车已具备 Gmapping、map_server、teleop_twist_keyboard。建图与导航 launch 已解析节点清单；没有为本次检查启动底盘。取点脚本在小车 Python 2 环境通过 5 项保存/重录/地图隔离检查，并验证无 `/map` 时拒绝保存。真实地图、真实点位和试走需要现场操作后验证。

参考：ROS Melodic 官方源码 [map_saver](https://github.com/ros-planning/navigation/blob/melodic-devel/map_server/src/map_saver.cpp)、[TF listener](https://github.com/ros/geometry/blob/melodic-devel/tf/src/tf/listener.py)。启动命令以本车实际安装的 launch 为准。
