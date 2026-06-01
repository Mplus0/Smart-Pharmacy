# pharmacy_mplus0 导航定位调试说明

本文档用于记录 `pharmacy_mplus0` 功能包中导航定位系统的结构、调试方法、可修改参数位置以及常见问题排查方法，方便后续优化智慧药房小车的导航方案。

> 说明：本文档根据当前已整理出的导航功能包关系编写。由于未直接读取完整仓库源码，部分参数路径和节点名称需要结合实际工程进一步核对。文中标注“需要实机验证”的内容，建议在小车上通过 ROS 命令确认。

---

## 1. 导航定位系统总体结构

`pharmacy_mplus0` 的导航定位系统可以分为三层：

1. 上层任务控制层
2. ROS 导航执行层
3. 底层定位与传感器数据层

整体执行逻辑如下：

```text
waypoints.yaml
    ↓
navigation_client.py
    ↓
move_base action
    ↓
global_planner / teb_local_planner
    ↓
/cmd_vel
    ↓
底盘运动
```

其中：

- `waypoints.yaml` 保存比赛巡检航点。
- `navigation_client.py` 读取航点并向 `move_base` 发送目标点。
- `move_base` 负责全局路径规划和局部路径规划。
- `global_planner` 负责生成全局路径。
- `teb_local_planner` 默认作为局部规划器，负责局部避障和速度控制。
- `dwa_local_planner` 作为备选局部规划器，是否启用取决于 launch 参数或 move_base 配置。
- `costmap_2d` 维护全局和局部代价地图。
- `/cmd_vel` 输出底盘控制速度。
- `/odom` 或 `/odometry/filtered` 提供里程计信息。
- `/tf` 提供坐标变换关系。
- `/scan` 或 `/scan_filtered` 提供激光雷达数据。

当前系统存在两种定位模式：

| 模式 | 定位方案 | 说明 |
|---|---|---|
| 常规定位模式 | `amcl + robot_localization` | 使用 AMCL 基于激光雷达和静态地图进行定位，同时由 EKF 融合里程计、IMU 等信息 |
| 比赛定位模式 | `talos_laser_loc + robot_localization` | 当启用 `use_race_init:=true` 时，使用比赛专用激光定位方案替代 AMCL |

核心结论：

```text
定位：amcl 或 talos_laser_loc + robot_localization
导航：move_base + global_planner + teb_local_planner + costmap_2d
上层控制：navigation_client.py + waypoints.yaml
状态上报：tcp_reporter.py + /odom + TF 查询
```

---

## 2. 直接依赖的 ROS 功能包

以下功能包通常在 `pharmacy_mplus0/package.xml` 中声明，属于上层导航控制代码的直接依赖。

| 功能包 | 使用位置 | 作用 |
|---|---|---|
| `nav_msgs` | `tcp_reporter.py` | 订阅 `/odom` 里程计消息，用于获取小车运动状态或上报定位信息 |
| `move_base_msgs` | `navigation_client.py` | 构造 `MoveBaseGoal`，向 `move_base` 发送导航目标点 |
| `actionlib` | `navigation_client.py` | 连接 `move_base` action server，发送目标、等待结果、取消目标 |
| `actionlib_msgs` | `navigation_client.py` | 检查 `move_base` 返回状态，例如成功、失败、取消、超时等 |
| `tf` | `navigation_client.py` | 将航点中的 yaw 偏航角转换为四元数，用于填充导航目标姿态 |
| `tf2_ros` | `tcp_reporter.py` | 查询 `map → base_footprint` 等全局坐标变换，用于定位上报 |
| `geometry_msgs` | 导航控制脚本 | 发布 `/cmd_vel`，用于紧急停车或底盘速度控制 |
| `std_srvs` | `navigation_client.py` | 调用 `/move_base/clear_costmaps` 服务，清除代价地图 |

这些依赖主要服务于 `pharmacy_mplus0` 的上层导航控制逻辑。完整的导航定位栈由 launch 文件间接启动。

---

## 3. launch 文件启动关系

根据当前整理结果，`pharmacy_mplus0` 的导航启动关系大致如下：

```text
race_bringup.launch
├── base_camera_nav.launch
│   └── robot_navigation 相关 launch
│       ├── map_server
│       ├── amcl 或 talos_laser_loc
│       ├── robot_localization / ekf_localization_node
│       ├── move_base
│       ├── global_planner
│       ├── teb_local_planner 或 dwa_local_planner
│       └── costmap_2d
│
└── pharmacy_mplus0 自身任务节点
    ├── navigation_client.py
    ├── tcp_reporter.py
    └── 其他业务节点
```

常见 launch 参数说明如下：

| 参数 | 可能位置 | 作用 |
|---|---|---|
| `use_race_init` | `race_bringup.launch` 或相关导航 launch | 是否启用比赛专用定位 `talos_laser_loc` |
| `planner` | `base_camera_nav.launch` 或 `robot_navigation` launch | 选择局部规划器，例如 `teb` 或 `dwa` |
| `simulation` | `base_camera_nav.launch` 或 `robot_navigation` launch | 是否启用仿真模式 |
| `open_rviz` | 导航 launch | 是否自动打开 RViz |
| `map_file` | `robot_navigation` launch | 指定静态地图 yaml 文件路径 |
| `lidar_mode` | 激光雷达相关 launch | 设置激光雷达运行模式，具体含义需要结合驱动验证 |

建议在仓库中重点检查以下文件：

```text
pharmacy_mplus0/launch/race_bringup.launch
pharmacy_mplus0/launch/base_camera_nav.launch
pharmacy_mplus0/launch/robot_race_init.launch
robot_navigation/launch/
robot_navigation/param/
robot_navigation/maps/
```

---

## 4. 核心节点说明

| 节点名 | 所属功能包/文件 | 输入话题/服务/action | 输出话题/服务/action | 作用 |
|---|---|---|---|---|
| `move_base` | `navigation-melodic` / `robot_navigation` | `/move_base/goal`、`/map`、`/tf`、`/scan` 或 `/scan_filtered` | `/cmd_vel`、`/move_base/status`、`/move_base/result` | ROS 导航核心节点，负责全局规划、局部规划和速度输出 |
| `map_server` | `map_server` | 地图 yaml 文件 | `/map` | 加载静态地图 |
| `amcl` | `amcl` | `/map`、`/scan` 或 `/scan_filtered`、`/tf`、`/initialpose` | `/amcl_pose`、`map → odom` TF | 常规激光定位节点 |
| `talos_laser_loc` | `talos_laser_loc` | 激光数据、地图或定位初始化信息 | 定位结果或 TF | 比赛专用激光定位，替代 AMCL，具体接口需要实机验证 |
| `ekf_localization_node` | `robot_localization` | `/odom`、IMU 等 | `/odometry/filtered`、可能发布 TF | EKF 传感器融合，输出滤波后的里程计 |
| `teb_local_planner` | `teb_local_planner-melodic` | move_base 内部调用 | 局部速度规划结果 | 默认局部规划器，适合局部避障和轨迹优化 |
| `dwa_local_planner` | `navigation-melodic` | move_base 内部调用 | 局部速度规划结果 | 备选局部规划器 |
| `costmap_2d` | `navigation-melodic` | `/map`、`/scan`、`/tf` | global/local costmap | 维护全局和局部代价地图 |
| `navigation_client.py` | `pharmacy_mplus0/scripts/` | `waypoints.yaml`、`move_base` action | `/move_base/goal`、`/move_base/cancel`、`/move_base/clear_costmaps` | 上层航点导航控制 |
| `tcp_reporter.py` | `pharmacy_mplus0/scripts/` | `/odom`、TF | TCP 上报数据 | 获取并上报小车状态和定位信息 |
| `laser_filter` | 激光过滤相关包 | `/scan` | `/scan_filtered` | 对激光数据进行过滤后供 AMCL 或 move_base 使用 |

---

## 5. 坐标系与 TF 关系

常见坐标系如下：

| 坐标系 | 作用 |
|---|---|
| `map` | 全局地图坐标系，导航目标点通常在该坐标系下表示 |
| `odom` | 里程计坐标系，短时间连续但会随运动产生累计漂移 |
| `base_footprint` | 机器人底盘在地面投影的坐标系，常用于导航 |
| `base_link` | 机器人机体坐标系 |
| `base_laser_link` | 激光雷达坐标系 |
| `IMU_link` | IMU 坐标系 |
| `camera_link` | 摄像头坐标系 |

正常导航时，应该具备类似以下 TF 链路：

```text
map → odom → base_footprint → base_link → base_laser_link
```

如果工程中使用 `base_link` 代替 `base_footprint`，应以实际 launch 和参数文件中的 `base_frame_id`、`robot_base_frame` 为准。

常用检查命令：

```bash
rosrun tf view_frames
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo odom base_footprint
rostopic echo /tf
rostopic echo /tf_static
```

如果 `map → base_footprint` 查询失败，常见原因包括：

- `amcl` 或 `talos_laser_loc` 没有正常输出定位。
- `map_server` 没有正确加载地图。
- `/odom` 没有发布。
- `map`、`odom`、`base_footprint`、`base_link` 的 frame_id 不一致。
- `/use_sim_time` 设置错误。
- 激光数据没有输入定位节点。
- 同时启动了多个定位节点，导致 TF 冲突。

---

## 6. 重要话题、服务与 action

| 名称 | 类型 | 作用 | 检查命令 |
|---|---|---|---|
| `/map` | Topic | 静态地图 | `rostopic echo -n 1 /map` |
| `/odom` | Topic | 原始里程计 | `rostopic echo -n 1 /odom` |
| `/odometry/filtered` | Topic | EKF 融合后的里程计 | `rostopic echo -n 1 /odometry/filtered` |
| `/scan` | Topic | 原始激光雷达数据 | `rostopic hz /scan` |
| `/scan_filtered` | Topic | 过滤后的激光雷达数据 | `rostopic info /scan_filtered` |
| `/tf` | Topic | 动态坐标变换 | `rostopic echo /tf` |
| `/tf_static` | Topic | 静态坐标变换 | `rostopic echo /tf_static` |
| `/cmd_vel` | Topic | 底盘速度控制 | `rostopic echo /cmd_vel` |
| `/move_base/goal` | Action Topic | move_base 目标点 | `rostopic echo /move_base/goal` |
| `/move_base/result` | Action Topic | move_base 导航结果 | `rostopic echo /move_base/result` |
| `/move_base/status` | Action Topic | move_base 当前状态 | `rostopic echo /move_base/status` |
| `/move_base/cancel` | Action Topic | 取消当前导航目标 | `rostopic pub /move_base/cancel actionlib_msgs/GoalID '{}'` |
| `/move_base/clear_costmaps` | Service | 清除全局和局部代价地图 | `rosservice call /move_base/clear_costmaps` |

常用命令：

```bash
rostopic list
rosnode list
rostopic info /scan
rostopic echo -n 1 /odom
rostopic echo -n 1 /odometry/filtered
rostopic hz /scan
rostopic hz /odom
rostopic echo /move_base/status
rostopic echo /move_base/result
rosservice call /move_base/clear_costmaps
```

---

## 7. 航点配置说明

航点通常保存在：

```text
pharmacy_mplus0/config/waypoints.yaml
```

或：

```text
pharmacy_mplus0/params/waypoints.yaml
```

具体路径需要以 `navigation_client.py` 中读取的实际路径为准。

航点一般包含以下字段：

| 字段 | 含义 |
|---|---|
| `x` | 航点在 `map` 坐标系下的 x 坐标 |
| `y` | 航点在 `map` 坐标系下的 y 坐标 |
| `yaw` | 小车到达该点后的目标朝向 |

示例：

```yaml
point_1:
  x: 1.23
  y: 0.45
  yaw: 90
```

需要重点确认：

- `yaw` 单位是角度还是弧度。
- `navigation_client.py` 是否对 yaw 做了角度转弧度处理。
- 航点是否按照 yaml 中的顺序执行。
- 是否存在固定的 11 个航点编号。
- 是否存在跳过某个航点或动态选择航点的逻辑。

修改方法：

- 修改 `x`、`y` 可以改变目标点位置。
- 修改 `yaw` 可以改变到点后的朝向。
- 新增航点时，需要确认 `navigation_client.py` 是否自动遍历所有航点，还是只读取固定名称。
- 删除航点时，需要确认代码中是否硬编码了航点数量或航点名称。

修改 `.yaml` 文件后通常不需要重新编译，但需要重新启动相关 launch 文件，使新参数重新加载。

---

## 8. 可修改参数位置总表

### 8.1 航点相关参数

| 调试目标 | 参数位置 | 参数名 | 作用 | 调大/调小的影响 |
|---|---|---|---|---|
| 修改巡检点位置 | `pharmacy_mplus0/config/waypoints.yaml` | `x`、`y` | 控制导航目标位置 | 靠近墙体可能导致路径规划失败，远离墙体通常更稳定 |
| 修改到点朝向 | `pharmacy_mplus0/config/waypoints.yaml` | `yaw` | 控制目标朝向 | 朝向不准会影响识别板拍摄角度 |
| 修改航点顺序 | `waypoints.yaml` 或 `navigation_client.py` | 航点顺序 | 控制巡检路线 | 顺序不同会影响总路径长度和稳定性 |
| 跳过航点 | `navigation_client.py` | 需要检查实际变量 | 控制是否执行某个点 | 适合临时调试单点导航 |

### 8.2 `navigation_client.py` 中的参数

建议在以下文件中搜索关键字：

```text
pharmacy_mplus0/scripts/navigation_client.py
```

建议重点搜索：

```text
timeout
wait
retry
clear_costmaps
MoveBaseGoal
send_goal
wait_for_result
cancel_goal
quaternion_from_euler
cmd_vel
```

可能存在的可调参数：

| 调试目标 | 参数名/关键词 | 作用 | 调整影响 |
|---|---|---|---|
| 控制单个目标最大执行时间 | `timeout` | 防止导航长时间卡死 | 太短会误判失败，太长会拖慢比赛节奏 |
| 到点后等待 | `wait` / `sleep` | 稳定车身和识别画面 | 增大可提高拍摄稳定性，但会增加耗时 |
| 导航失败重试 | `retry` | 失败后重新发送目标 | 增大可提高容错，但可能造成反复卡死 |
| 清除代价地图 | `clear_costmaps` | 清除错误障碍物 | 频繁调用可能造成短暂规划波动 |
| yaw 转四元数 | `quaternion_from_euler` | 将航点 yaw 转为目标姿态 | 需要确认 yaw 单位是否正确 |
| 紧急停车 | `/cmd_vel` | 发布 0 速度 | 用于取消导航或异常保护 |

### 8.3 move_base 参数

常见位置：

```text
robot_navigation/param/move_base_params.yaml
robot_navigation/param/move_base.yaml
robot_navigation/launch/*.launch
```

| 参数 | 作用 | 调大/调小影响 |
|---|---|---|
| `controller_frequency` | 局部控制频率 | 调大响应更快但 CPU 占用更高 |
| `planner_frequency` | 全局规划频率 | 调大更频繁重规划，可能更耗资源 |
| `oscillation_timeout` | 判断震荡的超时时间 | 太小容易误触发恢复行为 |
| `oscillation_distance` | 判断震荡的距离阈值 | 太小容易认为机器人没有移动 |
| `recovery_behavior_enabled` | 是否启用恢复行为 | 关闭后失败更直接，开启可自动恢复 |
| `clearing_rotation_allowed` | 是否允许清障旋转 | 对差速小车有用，但狭窄区域可能原地转圈 |

### 8.4 TEB 局部规划器参数

常见位置：

```text
robot_navigation/param/teb_local_planner_params.yaml
robot_navigation/param/teb_local_planner.yaml
teb_local_planner-melodic/*
```

| 参数 | 作用 | 调大/调小影响 |
|---|---|---|
| `max_vel_x` | 最大前进速度 | 调大更快但更容易冲过目标点 |
| `max_vel_x_backwards` | 最大后退速度 | 调大后退能力增强，但比赛中可能不稳定 |
| `max_vel_theta` | 最大角速度 | 调大转向更快，过大容易抖动 |
| `acc_lim_x` | 线加速度限制 | 调大启动更快，过大易打滑 |
| `acc_lim_theta` | 角加速度限制 | 调大转向响应快，过大易震荡 |
| `min_turning_radius` | 最小转弯半径 | 差速小车通常可设为 0，具体需看配置 |
| `xy_goal_tolerance` | 位置到点容差 | 调小更精准但更难到点 |
| `yaw_goal_tolerance` | 朝向到点容差 | 调小朝向更准但可能原地调整更久 |
| `min_obstacle_dist` | 与障碍物最小距离 | 调大更安全但可能绕路或失败 |
| `inflation_dist` | 障碍物膨胀影响距离 | 调大更保守，调小更贴近障碍物 |
| `weight_obstacle` | 避障权重 | 调大更远离障碍物 |
| `weight_kinematics_nh` | 非完整约束权重 | 影响差速运动约束 |
| `weight_kinematics_forward_drive` | 前进运动偏好 | 调大更不愿意倒车 |
| `dt_ref` | 轨迹时间分辨率 | 影响轨迹优化精度和计算量 |
| `dt_hysteresis` | 时间分辨率滞回 | 影响轨迹调整稳定性 |

比赛建议：优先保证稳定，不要一开始把速度调得过高。若小车到点后拍摄识别板角度不稳定，应重点调整 `yaw_goal_tolerance`、速度限制和到点等待时间。

### 8.5 DWA 局部规划器参数

常见位置：

```text
robot_navigation/param/dwa_local_planner_params.yaml
robot_navigation/param/dwa_local_planner.yaml
```

| 参数 | 作用 | 调整影响 |
|---|---|---|
| `max_vel_x` | 最大前进速度 | 影响直线速度 |
| `min_vel_x` | 最小前进速度 | 过大可能导致低速调整困难 |
| `max_vel_theta` | 最大角速度 | 影响转向速度 |
| `acc_lim_x` | 线加速度限制 | 影响启动和刹车 |
| `acc_lim_theta` | 角加速度限制 | 影响转向响应 |
| `xy_goal_tolerance` | 位置容差 | 影响到点精度 |
| `yaw_goal_tolerance` | 角度容差 | 影响最终朝向 |
| `path_distance_bias` | 贴近全局路径权重 | 调大更贴路径 |
| `goal_distance_bias` | 靠近目标点权重 | 调大更重视目标 |
| `occdist_scale` | 避障权重 | 调大更远离障碍物 |

当前默认局部规划器为 `teb_local_planner`，DWA 是否实际启用需要检查 `planner` 参数和 move_base 配置。

### 8.6 costmap 参数

常见位置：

```text
robot_navigation/param/costmap_common_params.yaml
robot_navigation/param/global_costmap_params.yaml
robot_navigation/param/local_costmap_params.yaml
```

| 参数 | 作用 | 调大/调小影响 |
|---|---|---|
| `global_frame` | 代价地图所在全局坐标系 | 通常 global_costmap 使用 `map`，local_costmap 使用 `odom` |
| `robot_base_frame` | 机器人底盘坐标系 | 必须与 TF 中的底盘 frame 一致 |
| `update_frequency` | 更新频率 | 调大响应快但占用 CPU |
| `publish_frequency` | 发布频率 | 影响 RViz 显示更新速度 |
| `transform_tolerance` | TF 容忍时间 | 太小容易 TF 超时报错 |
| `resolution` | 地图分辨率 | 越小越精细但计算量越大 |
| `inflation_radius` | 障碍物膨胀半径 | 调大更安全但容易绕路 |
| `cost_scaling_factor` | 膨胀代价衰减 | 影响靠近障碍物的代价变化 |
| `obstacle_range` | 障碍物加入范围 | 过大可能引入远处噪声 |
| `raytrace_range` | 清除障碍物范围 | 影响动态障碍物清除 |
| `footprint` / `robot_radius` | 机器人外形 | 配置不准会导致贴墙或规划失败 |

### 8.7 AMCL 参数

常见位置：

```text
robot_navigation/param/amcl_params.yaml
robot_navigation/launch/amcl.launch
```

| 参数 | 作用 | 调整影响 |
|---|---|---|
| `min_particles` | 最小粒子数 | 调大更稳定但更耗 CPU |
| `max_particles` | 最大粒子数 | 调大可增强重定位能力 |
| `update_min_d` | 平移更新阈值 | 调小更新更频繁 |
| `update_min_a` | 旋转更新阈值 | 调小旋转时更新更频繁 |
| `laser_min_range` | 激光最小有效距离 | 过滤近距离异常点 |
| `laser_max_range` | 激光最大有效距离 | 限制参与匹配的远距离激光 |
| `odom_frame_id` | 里程计坐标系 | 通常为 `odom` |
| `base_frame_id` | 机器人底盘坐标系 | 通常为 `base_footprint` 或 `base_link` |
| `global_frame_id` | 全局坐标系 | 通常为 `map` |
| `transform_tolerance` | TF 容忍时间 | 太小容易 TF 报错，太大可能造成延迟 |

### 8.8 robot_localization EKF 参数

常见位置：

```text
robot_navigation/param/ekf.yaml
robot_navigation/param/robot_localization.yaml
robot_localization/params/
```

| 参数 | 作用 |
|---|---|
| `frequency` | EKF 输出频率 |
| `sensor_timeout` | 传感器超时时间 |
| `two_d_mode` | 是否启用二维模式，移动机器人通常设为 true |
| `map_frame` | map 坐标系名称 |
| `odom_frame` | odom 坐标系名称 |
| `base_link_frame` | base 坐标系名称 |
| `world_frame` | EKF 使用的世界坐标系 |
| `odom0` | 第一个里程计输入 |
| `imu0` | 第一个 IMU 输入 |
| `odom0_config` | 控制 odom 中哪些变量参与融合 |
| `imu0_config` | 控制 IMU 中哪些变量参与融合 |

EKF 的典型作用：

```text
输入：/odom、IMU 等
输出：/odometry/filtered
```

需要注意：不要重复融合同一来源的信息，否则可能导致定位漂移或估计不稳定。

### 8.9 talos_laser_loc 参数

比赛专用定位包可能位于：

```text
talos_laser_loc/
```

建议重点检查：

```text
talos_laser_loc/launch/
talos_laser_loc/config/
talos_laser_loc/params/
```

需要确认以下内容：

| 检查项 | 说明 |
|---|---|
| 输入激光话题 | 是 `/scan` 还是 `/scan_filtered` |
| 输出定位话题 | 是否发布定位 Pose |
| 输出 TF | 是否发布 `map → odom` 或其他 TF |
| 地图输入 | 是否依赖 `map_server` 的 `/map` |
| frame_id | 是否与 move_base、costmap、EKF 一致 |
| 初始化方式 | 是否需要比赛专用初始化节点 |

若参数含义不明确，应在 README 中标注“需要实机验证”，不要强行修改。

---

## 9. 常见问题与排查方法

### 9.1 RViz 中小车模型不动

| 现象 | 可能原因 | 检查命令 | 解决办法 |
|---|---|---|---|
| 小车实际运动，但 RViz 模型不动 | `/odom` 没有变化 | `rostopic echo -n 1 /odom` | 检查底盘里程计节点 |
| RViz Fixed Frame 报错 | Fixed Frame 设置错误 | RViz 左侧 Global Options | 设置为 `map` 或存在的 frame |
| 模型没有全局位姿 | `map → odom` 缺失 | `rosrun tf tf_echo map base_footprint` | 检查 AMCL 或 talos_laser_loc |
| 坐标系断开 | frame_id 不一致 | `rosrun tf view_frames` | 统一 base frame 名称 |

### 9.2 2D Pose Estimate 无效

| 现象 | 可能原因 | 检查命令 | 解决办法 |
|---|---|---|---|
| RViz 中点击 2D Pose Estimate 后无反应 | AMCL 没启动 | `rosnode list | grep amcl` | 启动 AMCL 或检查比赛定位模式 |
| `/initialpose` 无订阅者 | 定位节点未订阅 | `rostopic info /initialpose` | 检查定位节点配置 |
| 激光与地图不重合 | 激光数据未进入 AMCL | `rostopic info /scan_filtered` | 检查激光话题配置 |
| 位姿估计后仍没有 map TF | AMCL 没有发布 TF | `rosrun tf tf_echo map odom` | 检查 AMCL 参数 |

### 9.3 move_base 一直失败

| 现象 | 可能原因 | 检查命令 | 解决办法 |
|---|---|---|---|
| 目标点发送后立刻失败 | 目标点在障碍物或未知区域 | RViz 查看 goal 位置 | 修改航点位置 |
| 规划失败 | costmap 膨胀过大 | 查看 local/global costmap | 调整 `inflation_radius` |
| TF 超时 | TF 链不稳定 | `rostopic echo /move_base/status` | 检查 `transform_tolerance` |
| 小车不动 | 局部规划器速度限制异常 | 查看 TEB/DWA 参数 | 检查速度和加速度限制 |

### 9.4 小车到点后角度不准

| 可能原因 | 解决建议 |
|---|---|
| `yaw_goal_tolerance` 太大 | 适当减小角度容差 |
| 航点 yaw 设置不准 | 重新在 RViz 中标定目标朝向 |
| 轮子打滑 | 降低角速度和角加速度 |
| NavigationClient 过早判断成功 | 检查是否只依赖 move_base result |
| 局部规划器不重视最终朝向 | 调整 TEB 相关权重和容差 |

### 9.5 小车绕路或贴墙

| 可能原因 | 解决建议 |
|---|---|
| `inflation_radius` 不合理 | 贴墙则调大，绕路严重则适当调小 |
| `min_obstacle_dist` 不合理 | 根据小车实际宽度调整 |
| 地图边缘噪声 | 重新修图或重新建图 |
| 航点太靠近墙体 | 将航点向通道中心移动 |
| 激光雷达噪声 | 检查 `/scan_filtered` 过滤效果 |

### 9.6 小车原地转圈或震荡

| 可能原因 | 解决建议 |
|---|---|
| TEB 参数不适合差速小车 | 检查 `min_turning_radius`、速度限制和非完整约束权重 |
| `max_vel_theta` 过大 | 降低最大角速度 |
| `acc_lim_theta` 过大 | 降低角加速度 |
| 局部代价地图过于保守 | 检查 `inflation_radius` 和 `min_obstacle_dist` |
| recovery behavior 反复触发 | 查看 `/move_base/status` 和日志 |

### 9.7 定位突然跳变

| 可能原因 | 解决建议 |
|---|---|
| AMCL 粒子数不足 | 适当增加 `min_particles` 和 `max_particles` |
| 地图与真实环境不一致 | 重新建图或修正地图 |
| 激光匹配失败 | 检查激光雷达安装和 `/scan_filtered` |
| odom 漂移严重 | 检查底盘编码器和 EKF 参数 |
| EKF 输入重复融合 | 检查 `odom0_config` 和 `imu0_config` |
| AMCL 与 talos_laser_loc 冲突 | 确认同一时间只启用一个全局定位源 |

---

## 10. 推荐调试流程

### Step 1：确认底盘和里程计

```bash
rostopic echo -n 1 /odom
rostopic hz /odom
```

如果小车运动时 `/odom` 不变化，优先检查底盘驱动和编码器。

### Step 2：确认激光雷达

```bash
rostopic echo -n 1 /scan
rostopic hz /scan
rostopic info /scan_filtered
```

如果 AMCL 使用 `/scan_filtered`，必须确认 `/scan_filtered` 正常发布。

### Step 3：确认地图

```bash
rostopic echo -n 1 /map
```

如果没有 `/map`，检查 `map_server` 是否启动，以及地图 yaml 路径是否正确。

### Step 4：确认 TF

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map base_footprint
rosrun tf view_frames
```

如果 `odom → base_footprint` 存在，但 `map → base_footprint` 不存在，通常是全局定位节点没有正常工作。

### Step 5：确认定位

在 RViz 中：

1. Fixed Frame 设置为 `map`。
2. 添加 `Map`、`LaserScan`、`RobotModel`、`TF`。
3. 使用 `2D Pose Estimate` 初始化位姿。
4. 观察激光点云是否与地图边缘重合。

常用命令：

```bash
rostopic echo -n 1 /amcl_pose
rosrun tf tf_echo map odom
```

如果使用 `talos_laser_loc`，需要根据该节点实际输出话题替换检查命令。

### Step 6：确认 move_base

```bash
rostopic echo /move_base/status
rostopic echo /move_base/result
rosservice call /move_base/clear_costmaps
```

如果 move_base 反复失败，应先在 RViz 中手动发送单个目标点测试。

### Step 7：单点导航测试

建议先只测试一个航点，不要直接运行完整比赛流程。

可选方法：

1. 临时修改 `waypoints.yaml`，只保留一个航点。
2. 在 `navigation_client.py` 中增加调试参数，只执行指定航点。
3. 使用 RViz 的 `2D Nav Goal` 手动发送目标。

单点测试通过后，再逐步增加航点数量。

### Step 8：完整任务测试

完整启动示例：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch
```

比赛定位模式示例：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch use_race_init:=true
```

运行时重点观察：

```bash
rostopic echo /move_base/status
rostopic echo /move_base/result
rostopic echo -n 1 /odom
rosrun tf tf_echo map base_footprint
```

---

## 11. 导航参数优化建议

智慧药房比赛场景特点：

- 场地相对固定。
- 航点数量有限。
- 小车停靠点固定。
- 比赛更重视稳定性，而不是极限速度。
- 识别板拍摄需要稳定朝向。
- 轮子打滑可能导致车头朝向误差。

### 11.1 提高到点稳定性

建议：

- 适当降低 `max_vel_x` 和 `max_vel_theta`。
- 适当降低 `acc_lim_x` 和 `acc_lim_theta`，减少打滑。
- 根据识别需求调整 `xy_goal_tolerance` 和 `yaw_goal_tolerance`。
- 到点后增加 0.5 到 1 秒停车等待，使摄像头画面稳定。
- 如果识别板角度要求高，可以在到点后增加二次角度修正逻辑。

### 11.2 减少贴墙和绕路

建议：

- 航点尽量放在通道中心，不要贴近墙体。
- 适当调整 `inflation_radius`。
- 如果小车贴墙，增大 `inflation_radius` 或 `min_obstacle_dist`。
- 如果小车绕路严重，适当减小 `inflation_radius`。
- 检查地图边界是否有噪声，必要时修图。

### 11.3 减少定位跳变

建议：

- 确保激光雷达扫描方向和地图一致。
- 确保 `/scan` 或 `/scan_filtered` 的 frame_id 与 TF 一致。
- 调整 AMCL 粒子数，提高定位稳定性。
- 检查 EKF 是否重复融合相同来源的里程计信息。
- 避免同时启动 AMCL 和 `talos_laser_loc` 发布冲突 TF。

### 11.4 适配比赛专用定位

建议：

- 使用 `use_race_init:=true` 时，重点检查 `talos_laser_loc` 是否正常输出定位。
- 对比 AMCL 和 `talos_laser_loc` 在同一场地中的稳定性。
- 确认 `talos_laser_loc` 输出的 frame 是否能被 move_base 使用。
- 如果定位输出 frame 与 move_base 配置不一致，需要统一 `map`、`odom`、`base_footprint` 等名称。

---

## 12. 文件修改后是否需要重新编译

| 修改内容 | 是否需要重新编译 | 是否需要重新启动 launch | 说明 |
|---|---|---|---|
| `.py` 脚本 | 通常不需要 | 需要 | Python 脚本重新运行即可 |
| `.yaml` 参数文件 | 不需要 | 需要 | 参数在 launch 启动时加载 |
| `.launch` 文件 | 不需要 | 需要 | 修改后重新 roslaunch |
| `package.xml` | 建议重新编译 | 需要 | 依赖变化后需要重新构建环境 |
| `CMakeLists.txt` | 需要 | 需要 | 构建规则变化后需要重新编译 |
| 新增 Python 节点 | 通常不需要，但要检查权限 | 需要 | 需要 `chmod +x` |

常用命令：

```bash
cd ~/robot_ws
catkin_make
source devel/setup.bash
chmod +x src/pharmacy_mplus0/scripts/xxx.py
```

如果只是修改 `waypoints.yaml` 或 move_base 参数文件，一般执行：

```bash
# 关闭原 launch 后重新启动
roslaunch pharmacy_mplus0 race_bringup.launch
```

---

## 13. 调试命令速查表

### 启动相关

```bash
roslaunch pharmacy_mplus0 race_bringup.launch
roslaunch pharmacy_mplus0 race_bringup.launch use_race_init:=true
```

### 节点和话题

```bash
rostopic list
rosnode list
rostopic info /scan
rostopic info /scan_filtered
rostopic info /odom
rostopic info /odometry/filtered
```

### 传感器数据

```bash
rostopic hz /scan
rostopic hz /odom
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rostopic echo -n 1 /odometry/filtered
```

### 地图和定位

```bash
rostopic echo -n 1 /map
rostopic echo -n 1 /amcl_pose
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo odom base_footprint
rosrun tf view_frames
```

### move_base 状态

```bash
rostopic echo /move_base/status
rostopic echo /move_base/result
rostopic echo /move_base/goal
rosservice call /move_base/clear_costmaps
```

### 紧急停止

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

---

## 14. 当前工程中需要重点检查的问题

后续调试时，建议优先检查以下问题：

| 检查项 | 可能影响 |
|---|---|
| launch 文件是否正确包含 `robot_navigation` | 导航栈未启动会导致 move_base、map、amcl 不存在 |
| `use_race_init:=true` 时 AMCL 是否被正确替换 | 同时启动 AMCL 和 talos_laser_loc 可能导致 TF 冲突 |
| `/scan` 与 `/scan_filtered` 是否使用一致 | 定位节点订阅错误会导致无定位输出 |
| `base_link` 与 `base_footprint` 是否混用 | TF 或 costmap frame 不一致会导致 move_base 报错 |
| `map_file` 路径是否正确 | map_server 无法加载地图会导致全局导航失败 |
| move_base 参数文件是否实际加载 | 修改参数后如果未生效，可能是 launch 没有引用该 yaml |
| Python 脚本是否有执行权限 | 没有权限会导致节点启动失败 |
| `waypoints.yaml` 中 yaw 单位是否正确 | 单位错误会导致目标朝向异常 |
| EKF 是否重复融合同一来源数据 | 可能导致里程计输出不稳定 |
| `talos_laser_loc` 输出 frame 是否与 move_base 一致 | 比赛定位模式下尤其重要 |

---

## 15. 建议后续补充内容

当完整读取仓库后，建议继续补充以下内容：

1. 实际 launch 文件树状结构。
2. `waypoints.yaml` 中 11 个航点的完整表格。
3. `navigation_client.py` 中所有可调变量的行号和默认值。
4. move_base、TEB、costmap、AMCL、EKF 参数文件的真实路径。
5. 常规定位模式和比赛定位模式的启动日志对比。
6. RViz 推荐显示配置截图。
7. 单点导航测试步骤和完整比赛流程测试步骤。

---

## 16. 推荐给 Codex 的后续完善要求

如果后续使用 Codex 基于完整仓库继续完善本文档，可以发送以下要求：

```text
请你基于当前仓库真实代码继续完善 README_NAVIGATION.md。
要求：
1. 不要写通用 ROS 教程。
2. 必须补充真实文件路径、真实 launch 包含关系、真实参数名和真实默认值。
3. 给每个关键参数补充所在文件和行号。
4. 如果某个节点或参数在仓库中不存在，不要写成已经存在，只能写成建议检查。
5. 对 navigation_client.py、tcp_reporter.py、waypoints.yaml 进行重点分析。
6. 补充当前工程中最可能影响导航稳定性的 5 个问题。
```
