# pharmacy_mplus0 导航定位系统文档

> 本文档基于仓库真实代码梳理，所有路径、参数名和默认值均来自实际文件。
> 标注「需要实机验证」的内容建议在小车上通过 ROS 命令确认。

---

## 1. 导航定位系统总体结构

`pharmacy_mplus0` 的导航定位分为三层：

1. **上层任务控制层** — `navigation_client.py` + `waypoints.yaml` + `strategy.yaml`
2. **ROS 导航执行层** — `move_base` + `global_planner` + `teb_local_planner` + `costmap_2d`
3. **底层定位与传感器数据层** — AMCL / talos_laser_loc + `robot_localization` (EKF)

整体执行逻辑：

```text
waypoints.yaml  (config/waypoints.yaml, 12个航点)
    ↓  NavigationClient 读取航点，按逻辑名称查找坐标
navigation_client.py  (src/pharmacy_mplus0/navigation_client.py)
    ↓  send_goal → MoveBaseGoal
move_base  (robot_navigation/launch/move_base.launch)
    ↓  全局规划 (global_planner) + 局部规划 (teb_local_planner)
/cmd_vel
    ↓
底盘运动
```

两种定位模式：

| 模式 | 定位方案 | 启用方式 |
|------|----------|----------|
| 常规定位 | AMCL + robot_localization (EKF) | `robot_navigation.launch`（默认） |
| 比赛定位 | talos_laser_loc + robot_localization (EKF) | `robot_race_init.launch`（`use_race_init:=true`） |

核心结论：

```text
定位：amcl（或 talos_laser_loc）+ ekf_localization_node (robot_localization)
导航：move_base + global_planner + teb_local_planner + costmap_2d
上层控制：NavigationClient + waypoints.yaml + strategy.yaml
状态上报：tcp_reporter.py (StateCollector: /odom + TF)
```

---

## 2. 直接依赖的 ROS 功能包

以下依赖在 `package.xml` 中声明：

[package.xml](package.xml)

| 功能包 | 使用位置 | 作用 |
|--------|----------|------|
| `rospy` | 全部脚本 | ROS Python 客户端 |
| `actionlib` | [navigation_client.py:18](src/pharmacy_mplus0/navigation_client.py#L18) | move_base action 客户端 |
| `actionlib_msgs` | [navigation_client.py:19](src/pharmacy_mplus0/navigation_client.py#L19) | GoalStatus 状态判断 |
| `move_base_msgs` | [navigation_client.py:20](src/pharmacy_mplus0/navigation_client.py#L20) | MoveBaseAction / MoveBaseGoal |
| `nav_msgs` | [tcp_reporter.py:55](scripts/tcp_reporter.py#L55) | Odometry 消息订阅 |
| `tf` | [navigation_client.py:22](src/pharmacy_mplus0/navigation_client.py#L22) | yaw → 四元数转换 |
| `tf2_ros` | [tcp_reporter.py:42](scripts/tcp_reporter.py#L42) | map→base_footprint TF 查询 |
| `geometry_msgs` | move_base 内部 | /cmd_vel 速度控制 |
| `std_srvs` | [navigation_client.py:21](src/pharmacy_mplus0/navigation_client.py#L21) | /move_base/clear_costmaps 服务 |
| `std_msgs` | [tcp_reporter.py:56](scripts/tcp_reporter.py#L56) | String 消息类型 |
| `rospkg` | [navigation_client.py:17](src/pharmacy_mplus0/navigation_client.py#L17) | 定位 config 目录 |

---

## 3. launch 文件真实启动关系

### 3.1 启动链

```
race_bringup.launch
│   参数: start_base, start_navigation, start_camera, start_video_server,
│          start_pharmacy, planner, server_ip, server_port, car_id, audio_dir
│
├── base_camera_nav.launch
│   │   参数: start_navigation, start_camera, start_video_server, use_race_init
│   │
│   ├── [use_race_init=false, 默认] → robot_navigation.launch
│   │   ├── 静态 TF: base_footprint→base_link, base_link→IMU_link,
│   │   │           base_link→base_laser_link, base_link→camera_link
│   │   ├── robot_lidar.launch
│   │   ├── map_server (map.yaml → /map)
│   │   ├── amcl (scan←scan_filtered, /odom←/odometry/filtered)
│   │   ├── move_base.launch (planner=teb, odom_topic=/odometry/filtered)
│   │   └── ekf_localization_node (ekf_params.yaml)
│   │
│   ├── [use_race_init=true] → robot_race_init.launch
│   │   ├── 静态 TF: (同上)
│   │   ├── robot_lidar.launch
│   │   ├── map_server
│   │   ├── talos_laser_loc + talos_costmap_cleaner ★替代AMCL
│   │   ├── move_base.launch (同上)
│   │   └── ekf_localization_node
│   │
│   ├── astra.launch (摄像头)
│   └── web_video_server
│
└── main.launch
    ├── board1_detector.py
    ├── board2_detector.py
    ├── tcp_reporter.py
    └── main_controller.py
```

### 3.2 所有 launch 文件及参数

| launch 文件 | 路径 | 关键参数 |
|-------------|------|----------|
| `race_bringup.launch` | [launch/race_bringup.launch](launch/race_bringup.launch) | `start_base`(true), `start_navigation`(true), `start_camera`(true), `start_video_server`(true), `start_pharmacy`(true), `planner`(teb), `server_ip`(192.168.124.2), `server_port`(8888), `car_id`(1), `audio_dir`(包内audio) |
| `base_camera_nav.launch` | [launch/base_camera_nav.launch](launch/base_camera_nav.launch) | `start_navigation`(true), `start_camera`(true), `start_video_server`(true), `use_race_init`(false) |
| `main.launch` | [launch/main.launch](launch/main.launch) | `stream_url`, `server_ip`(192.168.124.2), `server_port`(8888), `car_id`(1), `audio_dir`(包内audio) |
| `main_single.launch` | [launch/main_single.launch](launch/main_single.launch) | 同 main.launch，`audio_dir` 默认为空（调试入口，禁用语音） |
| `reporter.launch` | [launch/reporter.launch](launch/reporter.launch) | `server_ip`(192.168.124.2), `server_port`(8888), `car_id`(1) |
| `detectors.launch` | [launch/detectors.launch](launch/detectors.launch) | `stream_url` |

### 3.3 robot_navigation 包 launch 文件

| launch 文件 | 路径 | 关键参数 |
|-------------|------|----------|
| `robot_navigation.launch` | [robot_navigation/launch/robot_navigation.launch](robot_navigation/launch/robot_navigation.launch) | `map_file`(map.yaml), `simulation`(false), `planner`(teb), `open_rviz`(false), `lidar_mode`(Boost) |
| `robot_race_init.launch` | [robot_navigation/launch/robot_race_init.launch](robot_navigation/launch/robot_race_init.launch) | 同上，额外参数 `is_send_anger`(false) |
| `move_base.launch` | [robot_navigation/launch/move_base.launch](robot_navigation/launch/move_base.launch) | `cmd_vel_topic`(cmd_vel), `odom_topic`(odom), `planner`(teb) |

---

## 4. 核心节点说明

| 节点名 | 所属包 / 文件 | 输入 | 输出 | 作用 |
|--------|--------------|------|------|------|
| `move_base` | navigation-melodic | `/move_base/goal`, `/map`, `/tf`, `scan_filtered` | `/cmd_vel`, `/move_base/status` | ROS 导航核心 |
| `map_server` | map_server | 地图 yaml 文件 | `/map` | 加载静态地图 |
| `amcl` | amcl | `/map`, `scan_filtered`, `/tf`, `/initialpose` | `/amcl_pose`, `map→odom` TF | 常规激光定位 |
| `talos_laser_loc` | talos_laser_loc | `scan_filtered`, 定位初始化 | 定位结果/TF | 比赛专用定位 |
| `talos_costmap_cleaner` | talos_laser_loc | — | — | 代价地图清理 |
| `ekf_localization_node` | robot_localization | `/odom`, `/imu_data` | `/odometry/filtered` | EKF 传感器融合 |
| `teb_local_planner` | teb_local_planner-melodic | move_base 内部 | 局部速度规划 | 默认局部规划器 |
| `global_planner` | navigation-melodic | move_base 内部 | 全局路径 | 全局路径规划器 |
| `tcp_reporter` | [scripts/tcp_reporter.py](scripts/tcp_reporter.py) | `/odom`, `/current_task`, `/cv1_result`, `/cv2_result`, TF | TCP JSON 上报 | 状态采集与上报 |
| `main_controller` | [scripts/main_controller.py](scripts/main_controller.py) | 各业务话题 | move_base goal 等 | 主控状态机 |

**注意**: `navigation_client.py` 不是一个独立 ROS 节点，而是作为 `NavigationClient` 类被 `main_controller.py` 导入使用。

---

## 5. 坐标系与 TF 关系

### 5.1 坐标系定义

| 坐标系 | 定义来源 | 说明 |
|--------|----------|------|
| `map` | [waypoints.yaml:6](config/waypoints.yaml#L6), [amcl_params.yaml:5](robot_navigation/param/EPRobot/amcl_params.yaml#L5) | 全局地图坐标系 |
| `odom` | [amcl_params.yaml:3](robot_navigation/param/EPRobot/amcl_params.yaml#L3), [ekf_params.yaml:20](robot_navigation/param/EPRobot/ekf_params.yaml#L20) | 里程计坐标系 |
| `base_footprint` | [waypoints.yaml:8](config/waypoints.yaml#L8), [amcl_params.yaml:4](robot_navigation/param/EPRobot/amcl_params.yaml#L4) | 机器人底盘投影 |
| `base_link` | [robot_navigation.launch:16](robot_navigation/launch/robot_navigation.launch#L16) | 机器人机体 |
| `base_laser_link` | [robot_navigation.launch:18](robot_navigation/launch/robot_navigation.launch#L18) | 激光雷达 |
| `IMU_link` | [robot_navigation.launch:17](robot_navigation/launch/robot_navigation.launch#L17) | IMU |
| `camera_link` | [robot_navigation.launch:19](robot_navigation/launch/robot_navigation.launch#L19) | 摄像头 |

### 5.2 静态 TF（在 robot_navigation.launch 中定义）

| 变换 | offset (x, y, z) | 说明 |
|------|------------------|------|
| `base_footprint → base_link` | (0, 0, 0) | 底盘→机体 |
| `base_link → IMU_link` | (0.08, 0, 0) | 机体→IMU |
| `base_link → base_laser_link` | (0.14, 0, 0.11) | 机体→激光雷达 |
| `base_link → camera_link` | (0.14, 0, 0.11) | 机体→摄像头 |

### 5.3 TF 链路

```text
map → odom → base_footprint → base_link → base_laser_link
                                        ├── IMU_link
                                        └── camera_link
```

- `map → odom`: 由 AMCL 或 talos_laser_loc 动态发布
- `odom → base_footprint`: 由 EKF（/odometry/filtered）或底盘里程计提供
- `base_footprint → base_link` 及以下: 静态 TF

### 5.4 常用检查命令

```bash
rosrun tf view_frames
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo odom base_footprint
rostopic echo /tf
rostopic echo /tf_static
```

---

## 6. 重要话题、服务与 action

| 名称 | 类型 | 作用 | 定义位置 |
|------|------|------|----------|
| `/map` | Topic | 静态地图 | map_server |
| `/odom` | Topic | 原始里程计 | 底盘驱动 |
| `/odometry/filtered` | Topic | EKF 融合里程计 | [ekf_params.yaml](robot_navigation/param/EPRobot/ekf_params.yaml) |
| `/scan` | Topic | 原始激光雷达 | 激光驱动 |
| `/scan_filtered` | Topic | 过滤后激光 | laser_filters 包 |
| `/imu_data` | Topic | IMU 数据 | 底盘驱动 |
| `/tf` | Topic | 动态 TF | amcl / talos_laser_loc |
| `/tf_static` | Topic | 静态 TF | [robot_navigation.launch](robot_navigation/launch/robot_navigation.launch) |
| `/cmd_vel` | Topic | 底盘速度控制 | [constants.py:70](src/pharmacy_mplus0/constants.py#L70) |
| `/move_base/goal` | Action | 导航目标 | move_base |
| `/move_base/result` | Action | 导航结果 | move_base |
| `/move_base/status` | Action | 导航状态 | move_base |
| `/move_base/cancel` | Action | 取消导航 | move_base |
| `/move_base/clear_costmaps` | Service | 清除代价地图 | [constants.py:91](src/pharmacy_mplus0/constants.py#L91) |
| `/current_task` | Topic | 当前任务状态 | [constants.py:66](src/pharmacy_mplus0/constants.py#L66) |
| `/cv1_result` | Topic | 识别板一结果 | [constants.py:64](src/pharmacy_mplus0/constants.py#L64) |
| `/cv2_result` | Topic | 识别板二结果 | [constants.py:65](src/pharmacy_mplus0/constants.py#L65) |

---

## 7. 航点配置

**文件位置**: [config/waypoints.yaml](config/waypoints.yaml)（39行）

### 7.1 坐标系配置

```yaml
frames:
  map: map               # 全局地图坐标系
  robot: base_footprint  # 小车底盘坐标系，TF 查询和位置上报使用
```

### 7.2 全部航点（yaw 单位为弧度）

| 航点名 | x | y | yaw (rad) | yaw (度) | 类别 | 用途 |
|--------|---|---|-----------|----------|------|------|
| `start` | 0.0 | 0.0 | 0.0 | 0° | 起点 | 每轮开始/结束位置 |
| `board1` | 0.711 | 0.0 | 0.0 | 0° | 识别板一 | 获取二维码 |
| `board2` | -0.107 | 3.913 | 3.1416 | 180° | 识别板二 | 读取状态 |
| `exam_A` | 0.785 | 2.616 | 3.1414 | 180° | 体检窗口 | 二维码 A 对应窗口 |
| `exam_B` | 1.543 | 2.950 | 1.5708 | 90° | 体检窗口 | 二维码 B 对应窗口 |
| `exam_C` | 1.530 | 2.085 | 1.5708 | 90° | 体检窗口 | 二维码 C 对应窗口 |
| `lab_1` | -1.639 | 2.521 | -1.5707 | -90° | 化验窗口 | 血常规窗口 |
| `lab_2` | -0.846 | 1.921 | 0.0 | 0° | 化验窗口 | 体液窗口 |
| `lab_3` | -1.669 | 1.538 | -1.5707 | -90° | 化验窗口 | 免疫检测窗口 |
| `lab_4` | -0.835 | 0.950 | -1.5708 | -90° | 化验窗口 | 激素检验窗口 |

**共10个航点**（非原推测的11个）。

### 7.3 名称映射表

```yaml
exam_waypoints:          # 体检窗口字母 → 航点名
  A: exam_A
  B: exam_B
  C: exam_C

lab_waypoints:           # 化验窗口编号 → 航点名
  "1": lab_1
  "2": lab_2
  "3": lab_3
  "4": lab_4
```

### 7.4 航点修改注意事项

- yaw 是**弧度**，由 `quaternion_from_euler(0, 0, yaw)` 直接使用——[navigation_client.py:217](src/pharmacy_mplus0/navigation_client.py#L217)
- 新增航点需同时更新 `waypoints` 和对应的 `exam_waypoints` / `lab_waypoints` 映射表
- `main_controller.py` 通过 `exam_waypoints` / `lab_waypoints` 查找航点，不硬编码航点数量
- 修改 `.yaml` 后无需编译，重启 launch 即可

---

## 8. 策略配置

**文件位置**: [config/strategy.yaml](config/strategy.yaml)（41行）

### 8.1 导航相关参数

| 参数路径 | 值 | 说明 | 代码引用 |
|----------|-----|------|----------|
| `timeouts.nav_default_seconds` | 40.0 | 单段导航默认超时 (秒) | [navigation_client.py:60-63](src/pharmacy_mplus0/navigation_client.py#L60-L63) |
| `dwell.exam_seconds` | 1.5 | 体检窗口到达后停留 (秒) | main_controller.py |
| `dwell.lab_seconds` | 1.5 | 化验窗口到达后停留 (秒) | main_controller.py |
| `dwell.board1_settle_seconds` | 1.0 | 识别板一稳定等待 (秒) | main_controller.py |

### 8.2 容错参数

| 参数路径 | 值 | 说明 |
|----------|-----|------|
| `timeouts.board1_wait_seconds` | 15.0 | 识板一等待超时 |
| `timeouts.board1_max_retry` | 3 | 识板一最大重试次数 |
| `timeouts.board2_wait_seconds` | 10.0 | 识板二等待超时 |
| `timeouts.max_consecutive_fail_rounds` | 5 | 最大连续失败轮数 |

### 8.3 策略参数

| 参数路径 | 值 | 说明 |
|----------|-----|------|
| `task_priority.prefer_more_samples` | true | 优先样本数多的二维码 |
| `rounds.round_return_to_start` | true | 每轮结束回起点 |
| `visit_order.AC` | [C, A] | AC 二维码访问顺序（非字母序） |
| `visit_order.ABC` | [C, A, B] | ABC 二维码访问顺序（非字母序） |

---

## 9. navigation_client.py 详细分析

**文件**: [src/pharmacy_mplus0/navigation_client.py](src/pharmacy_mplus0/navigation_client.py)（247行）

### 9.1 类与方法

```python
class NavigationClient(object):
    def __init__(self, waypoints_path=None, strategy_path=None, dry_run=False)
    def go_to(self, waypoint_name, timeout_sec=None) -> bool
    def clear_costmaps(self)
    def cancel(self)
    def get_waypoint(self, name) -> (x, y, yaw) or None
    def _ensure_server(self)           # 等待 move_base action server
    def _build_goal(self, waypoint_name) -> MoveBaseGoal or None
    @staticmethod _resolve_config_path(filename) -> str
    @staticmethod _load_yaml(path) -> dict
```

### 9.2 关键参数及默认值

| 参数/变量 | 位置 | 默认值 | 说明 |
|-----------|------|--------|------|
| `NAV_ACTION_NAME` | [constants.py:90](src/pharmacy_mplus0/constants.py#L90) | `"move_base"` | move_base action 名称 |
| `NAV_CLEAR_COSTMAPS_SRV` | [constants.py:91](src/pharmacy_mplus0/constants.py#L91) | `"/move_base/clear_costmaps"` | 清除代价地图服务名 |
| `NAV_DEFAULT_TIMEOUT_SEC` | [constants.py:94](src/pharmacy_mplus0/constants.py#L94) | `40.0` | strategy.yaml 不可用时的兜底超时 |
| `DEFAULT_WAYPOINTS_FILE` | [line 31](src/pharmacy_mplus0/navigation_client.py#L31) | `"waypoints.yaml"` | 航点文件名 |
| `DEFAULT_STRATEGY_FILE` | [line 32](src/pharmacy_mplus0/navigation_client.py#L32) | `"strategy.yaml"` | 策略文件名 |
| `self._map_frame` | [line 52-53](src/pharmacy_mplus0/navigation_client.py#L52-L53) | `"map"` (取自 waypoints.yaml frames.map) | 目标位姿坐标系 |
| `self._nav_timeout` | [line 60-63](src/pharmacy_mplus0/navigation_client.py#L60-L63) | strategy.yaml 或 `40.0` | 默认导航超时 |
| `self._dry_run` | [line 67](src/pharmacy_mplus0/navigation_client.py#L67) | `False` | 干跑模式 |
| `_ensure_server` 等待 | [line 192](src/pharmacy_mplus0/navigation_client.py#L192) | `5.0` 秒 | move_base server 等待超时 |
| `clear_costmaps` 等待 | [line 158](src/pharmacy_mplus0/navigation_client.py#L158) | `2.0` 秒 | clear_costmaps 服务等待超时 |

### 9.3 导航执行流程

```text
go_to("exam_A", timeout_sec=40.0)
  ├── _build_goal("exam_A")
  │   ├── get_waypoint("exam_A") → (0.685, 2.516, 1.5708)
  │   ├── 构造 MoveBaseGoal (frame_id="map")
  │   └── quaternion_from_euler(0, 0, 1.5708) → 四元数
  ├── _ensure_server() — 等待 move_base, 最多 5 秒, 超时抛异常
  ├── send_goal(goal)
  ├── wait_for_result(timeout=40s)
  │   ├── 超时 → cancel_goal() → return False
  │   └── 完成 → get_state()
  │       ├── SUCCEEDED → return True
  │       └── 其他 → return False
  └── (线程锁 release)
```

### 9.4 可调行为

| 调试点 | 说明 | 建议 |
|--------|------|------|
| `timeout_sec` | 调用 `go_to` 时传入，覆盖 strategy.yaml | 单点测试时可调短（如 20s），比赛用默认 40s |
| `dry_run=True` | 跳过硬件的导航测试模式 | 调试主控逻辑时使用 |
| `clear_costmaps()` | 每段导航结束后清理代价地图 | 若雷达噪声多，可提高调用频率 |
| `_ensure_server` 5秒等待 | move_base 未就绪则抛异常 | 启动顺序有问题时需检查 |

---

## 10. tcp_reporter.py 详细分析

**文件**: [scripts/tcp_reporter.py](scripts/tcp_reporter.py)（349行）

### 10.1 架构

```text
TcpReporterNode (ROS 节点 "tcp_reporter")
├── StateCollector      # 状态采集器
│   ├── /odom           → speed (线速度 x 分量)
│   ├── /current_task   → task
│   ├── /cv1_result     → CV1
│   ├── /cv2_result     → CV2
│   └── TF: map→base_footprint → odom (x, y)
├── TcpClient           # TCP 连接管理（后台线程自动重连）
└── AudioAnnouncer      # 音频播报
```

### 10.2 StateCollector 参数

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| `map_frame` | [line 92](scripts/tcp_reporter.py#L92) | `"map"` | 从 waypoints.yaml frames.map |
| `robot_frame` | [line 92](scripts/tcp_reporter.py#L92) | `"base_footprint"` | 从 waypoints.yaml frames.robot |
| `_TF_LOOKUP_TIMEOUT` | [line 82](scripts/tcp_reporter.py#L82) | `0.2` 秒 | 单次 TF 查询超时 |

### 10.3 订阅的话题

| 话题 | 类型 | 提取字段 | 代码位置 |
|------|------|----------|----------|
| `/odom` | `Odometry` | `twist.twist.linear.x` | [line 136](scripts/tcp_reporter.py#L136) |
| `/current_task` | `String` | `data` | [line 139](scripts/tcp_reporter.py#L139) |
| `/cv1_result` | `String` | `data` | [line 144](scripts/tcp_reporter.py#L144) |
| `/cv2_result` | `String` | `data` | [line 148](scripts/tcp_reporter.py#L148) |

### 10.4 上报 JSON 格式

```json
{
  "id": "1",
  "speed": 0.123,
  "odom": [0.685, 2.516],
  "task": "R",
  "CV1": "",
  "CV2": ""
}
```

### 10.5 ROS 参数（私有命名空间 `~`）

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| `~server_ip` | [line 216-217](scripts/tcp_reporter.py#L216-L217) | `"192.168.124.2"` | 裁判电脑 IP |
| `~server_port` | [line 219-220](scripts/tcp_reporter.py#L219-L220) | `8888` | 裁判软件端口 |
| `~send_hz` | [line 222-223](scripts/tcp_reporter.py#L222-L223) | `2.0` | 上报频率 |
| `~car_id` | [line 234-235](scripts/tcp_reporter.py#L234-L235) | `"1"` | 小车编号 |
| `~audio_dir` | [line 225-226](scripts/tcp_reporter.py#L225-L226) | `""` | wav 目录 |
| `~audio_player` | [line 228-229](scripts/tcp_reporter.py#L228-L229) | `"aplay"` | 播放器 |
| `~allow_overlap` | [line 231-232](scripts/tcp_reporter.py#L231-L232) | `False` | 音频重叠 |
| `~connect_timeout_seconds` | [line 239-240](scripts/tcp_reporter.py#L239-L240) | `1.0` | TCP 连接超时 |
| `~reconnect_seconds` | [line 242-243](scripts/tcp_reporter.py#L242-L243) | `2.0` | TCP 重连间隔 |

### 10.6 关键行为

- TF 查询使用 `rospy.Time(0)` 获取最新可用变换，失败时保持上次缓存坐标——[line 158-162](scripts/tcp_reporter.py#L158-L162)
- 所有回调通过 `threading.Lock()` 保护——[line 99](scripts/tcp_reporter.py#L99)
- 主循环以 `send_hz` 频率运行，每 30 帧（约 15 秒）打印一次进度——[line 323-329](scripts/tcp_reporter.py#L323-L329)
- 节点关闭时调用 `_on_shutdown` 断开 TCP——[line 301-305](scripts/tcp_reporter.py#L301-L305)

---

## 11. 可修改参数总表（含真实文件路径、行号和通俗解释）

> **阅读提示**: 每个参数都附带了「通俗解释」，即使不熟悉 ROS 导航也能理解参数的大致作用。
> 「调大/调小」列中的建议基于智慧药房比赛场景（场地固定、航点有限、重视稳定性）。

---

### 11.1 move_base 参数

**文件**: [robot_navigation/param/EPRobot/move_base_params.yaml](robot_navigation/param/EPRobot/move_base_params.yaml)

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `controller_frequency` | 8 | 局部规划器每秒计算几次速度指令。值=8 表示每秒发 8 次速度给底盘 | 调大：小车反应更快，但 CPU 占用更高。调小：CPU 省了，但遇到障碍可能刹车不及时 |
| `planner_frequency` | 0.2 | 全局规划器每隔几秒重新算一条全局路径。值=0.2 表示每 5 秒重算一次 | 调大：路径更新更勤快，但费 CPU。比赛场地固定，0.2 通常够用 |
| `controller_patience` | 3 | 局部规划器允许自己在原地"挣扎"几秒，超过后才认输并触发恢复行为 | 调大：给小车更多时间自己脱困。调小：更快触发清障恢复，但可能打断正常的精细调整 |
| `oscillation_timeout` | 5 | 判断小车是否在"原地震荡"的时间窗口（秒）。如果在这段时间内来回晃动，就认定震荡了 | 调小：更容易触发震荡判定，可能导致正常微调被误判。调大：真正的震荡发现得慢 |
| `oscillation_distance` | 0.2 | 判断震荡的距离阈值（米）。小车移动距离小于这个值，就被认为"没怎么动" | 调小：更容易判定震荡。调大：只有明显不动才算震荡 |
| `recovery_behavior_enabled` | true | 是否启用自动恢复行为。true=卡住时自动尝试清障、旋转等方式脱困 | 建议保持 true。关闭后导航失败更干脆，但失去自动恢复能力 |
| `clearing_rotation_allowed` | false | 恢复行为中是否允许小车原地旋转来清除代价地图 | 差速小车原地旋转容易漂移。false=只用直行清障，更安全 |
| `conservative_reset.reset_distance` | 1 | 保守清障时，清除小车周围多少米范围内的障碍物（米） | 值=1 表示清除半径 1 米的障碍记录。调大清除范围更大，但也可能把真实障碍清掉 |
| `aggressive_reset.reset_distance` | 3 | 激进清障时，清除小车周围多少米范围内的障碍物（米） | 保守清障失败后才会用这个。值=3 表示大范围清除。谨慎调大 |

---

### 11.2 TEB 局部规划器参数

**文件**: [robot_navigation/param/EPRobot/teb_local_planner_params.yaml](robot_navigation/param/EPRobot/teb_local_planner_params.yaml)

#### 速度与加速度限制

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `max_vel_x` | 1.0 | 小车直线前进的最大速度（米/秒）。值=1.9 约等于人快步走的速度 | **比赛中最重要的参数之一。** 调大=更快但容易冲过目标点或刹不住。调小=更稳但耗时长。建议从 1.0 开始调试，稳定后再逐步提高 |
| `max_vel_x_backwards` | 0.5 | 小车倒车的最大速度（米/秒） | 比赛场景通常不需要倒车，保持默认或调小到 0.5 |
| `max_vel_theta` | 2.3 | 小车原地旋转的最大角速度（弧度/秒）。值=2.3 约等于每秒转 132° | 调大：转向快但容易抖动。调小：转向平缓但耗时。如果小车到点后原地转圈，优先调小这个值 |
| `acc_lim_x` | 1.0 | 小车前进时的最大加速度（米/秒²）。值越大，从静止加速到全速越猛 | 调大：起步快但容易打滑。调小：起步柔和，减少轮子打滑。比赛建议不大于 1.0 |
| `acc_lim_theta` | 0.5 | 小车旋转时的最大角加速度（弧度/秒²） | 调大：转向启动快但容易震荡。调小：转向平缓。出现原地转圈问题时优先调小 |

#### 底盘特性

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `min_turning_radius` | 0.35 | 小车的最小转弯半径（米）。差速小车理论上可以原地转（半径=0），阿克曼小车有最小值 | 差速小车（左右轮独立驱动）可以设为 0。当前值 0.35 偏保守，如果小车是差速底盘，建议改为 0 |

#### 到点精度

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `xy_goal_tolerance` | 0.05 | 小车距离目标点多近才算"到达"（米）。值=0.15 表示距离目标点 15 厘米以内就算成功 | 调小：停得更准但更难到达（可能长时间微调）。调大：容易到达但停得不准。识别板场景建议 0.05-0.10 |
| `yaw_goal_tolerance` | 0.10 | 小车朝向与目标朝向差多少算"对准"（弧度）。值=0.10 约 5.7°，即车头偏差在 ±5.7° 内就算对准 | **识别板场景的关键参数。** 已从 0.55 收紧到 0.10，如果到点后长时间原地微调，可适当放宽到 0.15-0.20 |

#### 避障相关

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `min_obstacle_dist` | 0.115 | 小车与障碍物之间至少保持多少米（11.5 厘米） | 调大：更安全但通道窄时可能过不去。调小：贴墙更近。比赛建议 0.10-0.15 |
| `inflation_dist` | 0.15 | 障碍物周围额外膨胀多少米作为安全缓冲区 | 调大：更早绕开障碍。调小：更贴近障碍物。配合 costmap 的 inflation_radius 一起调整 |
| `obstacle_poses_affected` | 15 | 轨迹上考虑多少个位姿点与障碍物的距离 | 值越大避障越保守。一般保持默认 |

#### 轨迹优化权重

> 权重决定了规划器在多个目标之间如何取舍。**权重越大，该因素在决策中越重要。**

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `weight_optimaltime` | 15 | "走最短时间"的权重。值越大，小车越倾向于走快、走近道 | **非常关键。** 调大：小车更大胆，贴内弯、拉满速度，但也更容易撞。调小：更保守。建议从 10 开始调试 |
| `weight_obstacle` | 90 | "远离障碍物"的权重。值越大，小车离障碍物越远 | 如果小车频繁贴墙，调大这个值。如果绕路太远，适当调小 |
| `weight_shortest_path` | 2.0 | "路径最短"的权重。值越大，越倾向于选短路径 | 配合 weight_optimaltime 一起使用。一般不需要大改 |
| `weight_kinematics_nh` | 1000 | "遵守差速运动学"的权重。值越大，生成的轨迹越符合差速小车的运动特性 | 保持默认即可。如果轨迹出现侧移等不合理的运动，可以继续调大 |
| `weight_kinematics_forward_drive` | 700 | "尽量往前走、不倒车"的权重 | 保持默认即可 |
| `weight_acc_lim_x` | 0.8 | "遵守加速度限制"的权重 | 一般保持默认 |
| `weight_acc_lim_theta` | 0.5 | "遵守角加速度限制"的权重 | 一般保持默认 |
| `weight_max_vel_x` | 2 | "遵守最大速度限制"的权重 | 一般保持默认 |
| `weight_max_vel_theta` | 0.8 | "遵守最大角速度限制"的权重 | 一般保持默认 |
| `weight_inflation` | 2.0 | "考虑膨胀层"的权重 | 一般保持默认 |
| `weight_viapoint` | 25.0 | "经过全局路径关键点"的权重 | 一般保持默认 |

#### 轨迹时域参数

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `dt_ref` | 0.25 | 轨迹上相邻两个位姿点之间的时间间隔（秒）。值=0.25 表示每隔 0.25 秒放一个轨迹点 | 调小：轨迹更精细但计算量更大。调大：计算快但轨迹粗糙。0.2-0.3 通常合适 |
| `dt_hysteresis` | 0.1 | 允许 dt_ref 上下浮动的范围（秒） | 保持默认 |
| `min_samples` | 5 | 轨迹最少包含几个位姿点 | 保持默认 |
| `max_samples` | 200 | 轨迹最多包含几个位姿点 | 保持默认 |
| `max_global_plan_lookahead_dist` | 2.0 | 向前看全局路径多长的距离来优化（米） | 调大：看得更远，路径更平滑，但避障和转弯效果变差。调小：只看眼前，转弯灵敏。2.0 是个平衡值 |

#### 震荡恢复

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `oscillation_recovery` | true | 检测到小车震荡时是否自动尝试恢复 | 建议保持 true。关闭后震荡可能无法自动解除 |
| `oscillation_recovery_min_duration` | 10 | 震荡恢复最少持续多久（秒） | 保持默认 |
| `oscillation_filter_duration` | 10 | 检测震荡的时间窗口（秒） | 保持默认 |

---

### 11.3 DWA 局部规划器参数

**文件**: [robot_navigation/param/EPRobot/dwa_local_planner_params.yaml](robot_navigation/param/EPRobot/dwa_local_planner_params.yaml)

> **注意**: 当前默认使用 TEB 规划器（`planner=teb`），DWA 参数仅在切换到 `planner=dwa` 时生效。

#### 速度限制

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `max_vel_x` | 1.2 | 最大前进速度（米/秒） | 同 TEB，比赛建议从 1.0 开始调试 |
| `min_vel_x` | -1.2 | 最小前进速度（米/秒）。负值表示允许倒车，-1.2 表示允许最快 1.2m/s 倒车 | 比赛通常不需要倒车，可设为 0 禁止倒车 |
| `max_rot_vel` | 1.0 | 最大旋转速度（弧度/秒） | 调大：转向快。调小：转向平缓 |
| `min_rot_vel` | 0.5 | 最小旋转速度（弧度/秒）。旋转速度不会低于这个值 | 调小：允许更慢的精细旋转调整 |
| `acc_lim_x` | 2.5 | 前进方向最大加速度（米/秒²） | 调小减少打滑。比赛建议 1.0-1.5 |
| `acc_lim_theta` | 3.2 | 旋转方向最大角加速度（弧度/秒²） | 调小减少转向抖动 |

#### 到点精度

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `xy_goal_tolerance` | 0.05 | 到达目标点的位置容差（米）。5 厘米内就算到 | 比 TEB 默认 0.15 更严格。识别板场景合适的值 |
| `yaw_goal_tolerance` | 0.2 | 到达目标点的角度容差（弧度≈11.5°） | 比 TEB 默认 0.55 更严格 |

#### 路径评分权重

> DWA 通过给每条候选轨迹打分来选择最优轨迹。以下权重决定各因素在评分中的重要性。

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `path_distance_bias` | 32.0 | "贴近全局路径"在评分中的权重。值越大，小车越贴着全局路径走 | 如果全局路径合理但小车总偏航，调大这个值 |
| `goal_distance_bias` | 20.0 | "靠近目标点"在评分中的权重。值越大，小车越积极向目标靠拢 | 如果小车不敢靠近目标，调大这个值 |
| `occdist_scale` | 0.02 | "远离障碍物"在评分中的权重。值越大，小车越害怕障碍 | 贴墙时调大，绕路时调小 |

#### 模拟参数

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `sim_time` | 2.5 | 向前模拟多少秒来评估轨迹（秒）。值越大看得越远 | 调大：更有远见但计算量大。调小：反应快但短视 |

---

### 11.4 costmap 参数

#### 全局代价地图 (global_costmap)

**文件**: [robot_navigation/param/EPRobot/global_costmap_params.yaml](robot_navigation/param/EPRobot/global_costmap_params.yaml)

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `global_frame` | map | 全局代价地图在哪个坐标系中画 | 固定为 map，不要改动 |
| `robot_base_frame` | base_footprint | 机器人底盘在代价地图中用哪个坐标系表示 | 必须与 TF 树中的底盘 frame 完全一致 |
| `update_frequency` | 5.0 | 每秒更新几次代价地图（Hz） | 调大：地图更新快但吃 CPU。5Hz 已足够 |
| `publish_frequency` | 0.5 | 每秒向 RViz 发布几次可视化数据（Hz） | 仅影响 RViz 显示刷新速度，不影响导航。不需要调大 |
| `resolution` | 0.05 | 每个栅格代表实际多少米。0.05=每个格子 5 厘米 | 调小：地图更精细但计算量大。0.05 是常用值 |
| `inflation_radius` | 0.5 | 障碍物周围膨胀多少米。膨胀范围内的栅格会被标记为"危险" | **关键参数。** 值=0.5 表示障碍物周围 50 厘米都是危险区。全局 costmap 用较大的膨胀可以让全局路径更安全、远离墙壁。如果小车绕路太远，适当调小 |
| `cost_scaling_factor` | 17 | 膨胀代价的衰减速度。值越大，离障碍物越近时代价增长越慢 | 调大：代价衰减慢，小车更愿意靠近障碍物。调小：代价衰减快，小车更早避开 |
| `transform_tolerance` | 1.0 | 允许 TF 数据最多延迟多久（秒）。超过这个时间就报错 | 如果频繁出现 TF 超时警告，适当调大。但不建议超过 2.0 |

#### 局部代价地图 (local_costmap)

**文件**: [robot_navigation/param/EPRobot/local_costmap_params.yaml](robot_navigation/param/EPRobot/local_costmap_params.yaml)

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `global_frame` | map | 局部代价地图在哪个坐标系中画 | 当前设为 map（与全局相同）。部分项目会设为 odom。需要实机确认 |
| `robot_base_frame` | base_footprint | 底盘坐标系 | 必须与 TF 一致 |
| `update_frequency` | 8.0 | 局部代价地图更新频率（Hz） | 比全局更频繁，因为需要实时避障。8Hz 合理 |
| `publish_frequency` | 1.0 | RViz 可视化发布频率（Hz） | 仅影响显示 |
| `rolling_window` | true | 是否以小车为中心滚动窗口 | true=代价地图跟着小车移动，适合局部规划 |
| `width` × `height` | 2.5 × 2.5 | 局部代价地图覆盖范围（米），小车居中 | 2.5m×2.5m 表示小车前后左右各约 1.25 米的范围。增大可看得更远但计算量大 |
| `resolution` | 0.05 | 栅格分辨率（米） | 与全局保持一致 |
| `inflation_radius` | 0.15 | 障碍物膨胀半径（米） | **注意: 局部仅 0.15m，全局为 0.5m，差异较大。** 局部用较小值是合理的——局部规划需要更精确地贴着路径走。但如果小车频繁刮墙，可适当调大 |
| `cost_scaling_factor` | 6 | 膨胀代价衰减速度 | 值=6 比全局的 17 小很多，意味着局部代价地图中障碍物"威力"更大。如果局部避障过于保守，可调大 |

#### 通用代价地图层 (costmap_common)

**文件**: [robot_navigation/param/EPRobot/costmap_common_params.yaml](robot_navigation/param/EPRobot/costmap_common_params.yaml)

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `footprint` | [[-0.145,-0.105], [0.145,-0.105], [0.145,0.105], [-0.145,0.105]] | 小车的外轮廓顶点坐标（米）。定义了一个 0.29m×0.21m 的矩形 | 必须与实际小车尺寸一致。尺寸不对会导致贴墙刮蹭或通道过不去 |
| `obstacle_range` | 5.0 | 多远的激光点被认为是障碍物（米）。超过 5 米的激光数据被忽略 | 调大：考虑更远的障碍，但噪声多。调小：只关心近处障碍 |
| `raytrace_range` | 5.0 | 多远的激光点可以用来"清除"障碍标记（米） | 与 obstacle_range 保持一致即可 |
| `inflation_radius` (common) | 0.15 | 通用膨胀半径（米）。会被全局/局部的同名字段覆盖 | 实际生效的是全局/局部 yaml 中的值，这里只是默认值 |
| `cost_scaling_factor` (common) | 7 | 通用代价衰减。同样会被覆盖 | 实际生效的是全局/局部中的值 |

---

### 11.5 AMCL 参数

**文件**: [robot_navigation/param/EPRobot/amcl_params.yaml](robot_navigation/param/EPRobot/amcl_params.yaml)

#### 粒子滤波器

AMCL 用"粒子"来表示"小车可能在哪里"。粒子越多，定位越稳定，但 CPU 占用越高。

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `min_particles` | 500 | 最少用多少个粒子来估计位置 | 调大：定位更稳定但更耗 CPU。500 是常用值 |
| `max_particles` | 2000 | 最多用多少个粒子来估计位置。定位不确定时会自动增加粒子数，最多到这个值 | 调大：重定位能力更强。如果小车被"绑架"（搬动后）恢复定位慢，可调大 |
| `kld_err` | 0.05 | 粒子数自适应调整的误差阈值。值越小，需要的粒子越多 | 保持默认 |
| `kld_z` | 0.99 | 粒子数自适应的置信度。0.99=99% 确信粒子数够用 | 保持默认 |

#### 更新触发条件

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `update_min_d` | 0.15 | 小车平移超过多少米才触发一次定位更新 | 调小：更新更频繁，定位更及时但耗 CPU。0.1-0.2 合适 |
| `update_min_a` | 0.1 | 小车旋转超过多少弧度（约 5.7°）才触发一次定位更新 | 调小：转弯时定位更新更频繁。0.05-0.1 合适 |

#### 激光模型

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `laser_max_range` | 12.0 | 参与定位的激光数据最大距离（米）。超过 12m 的激光点被忽略 | 根据实际激光雷达性能设置。一般 8-15m |
| `laser_model_type` | "likelihood_field" | 激光匹配算法类型。likelihood_field = 似然场模型，比 beam 模型更平滑鲁棒 | 保持默认。likelihood_field 是主流选择 |
| `laser_max_beams` | 60 | 每次定位更新使用多少条激光束 | 调大：定位更准但计算量大。60 是平衡值 |
| `laser_likelihood_max_dist` | 2.0 | 似然场模型中，激光点与地图障碍物的最大匹配距离（米） | 保持默认 |

#### 里程计噪声模型

AMCL 需要知道里程计的误差特性。以下参数告诉 AMCL "里程计有多不准"。

| 参数 | 值 | 通俗解释 |
|------|-----|----------|
| `odom_model_type` | "diff" | 里程计模型类型。diff=差速模型 |
| `odom_alpha1` | 0.2 | 旋转带来的旋转噪声 |
| `odom_alpha2` | 0.2 | 平移带来的旋转噪声 |
| `odom_alpha3` | 0.2 | 平移带来的平移噪声 |
| `odom_alpha4` | 0.2 | 旋转带来的平移噪声 |

> 噪声参数值越大，AMCL 越"不相信"里程计。如果小车定位容易漂移，可适当调大 alpha3 和 alpha4。

#### 坐标系与恢复

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `odom_frame_id` | "odom" | 里程计坐标系名称 | 必须与 TF 树一致 |
| `base_frame_id` | "base_footprint" | 底盘坐标系名称 | 必须与 TF 树一致 |
| `global_frame_id` | "map" | 全局坐标系名称 | 必须与 TF 树一致 |
| `transform_tolerance` | 1.0 | 允许 TF 数据最多延迟多久（秒） | 如果出现 TF 超时警告，适当调大 |
| `recovery_alpha_slow` | 0.001 | 慢恢复时向均匀分布添加随机粒子的速率 | 值越小越慢。定位完全丢失时起作用 |
| `recovery_alpha_fast` | 0.1 | 快恢复时向均匀分布添加随机粒子的速率 | 值越大恢复越快。定位完全丢失时起作用 |

---

### 11.6 EKF (robot_localization) 参数

**文件**: [robot_navigation/param/EPRobot/ekf_params.yaml](robot_navigation/param/EPRobot/ekf_params.yaml)

#### 什么是 EKF？

EKF (扩展卡尔曼滤波器) 的作用是把多个传感器（里程计、IMU 等）的数据融合在一起，输出一个比单独传感器更准确、更平滑的位姿估计。在本项目中，输入是 `/odom`（里程计）和 `/imu_data`（IMU），输出是 `/odometry/filtered`。

#### 基础配置

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `frequency` | 30 | EKF 每秒输出几次融合结果（Hz） | 30Hz 已足够。不需要改 |
| `two_d_mode` | true | 是否只做二维融合。true=忽略 Z/roll/pitch，只估计 x/y/yaw | 移动机器人通常设为 true |
| `sensor_timeout` | 0.025 | 传感器数据超过多久没来就认为失效（秒） | 保持默认 |
| `publish_tf` | true | 是否由 EKF 发布 odom→base_footprint 的 TF | true 表示 EKF 负责发布这段 TF |
| `publish_acceleration` | true | 是否发布加速度估计 | 保持默认 |

#### 坐标系配置

| 参数 | 值 | 通俗解释 |
|------|-----|----------|
| `map_frame` | /map | 地图坐标系名称 |
| `odom_frame` | /odom | 里程计坐标系名称 |
| `base_link_frame` | /base_footprint | 底盘坐标系名称 |
| `world_frame` | /odom | EKF 以哪个坐标系作为"世界"。此处设为 /odom，表示 EKF 在 odom 系中进行估计 |

#### 传感器输入配置

| 参数 | 值 | 通俗解释 |
|------|-----|----------|
| `odom0` | /odom | 第一个里程计输入的话题名 |
| `odom0_config` | 仅 vx, vy = true | 告诉 EKF："从里程计只取前进速度和横向速度，不要取位置"。这样位置由速度积分得到 |
| `odom0_differential` | true | 将里程计的绝对位姿转为速度差分量再融合 |
| `imu0` | /imu_data | 第一个 IMU 输入的话题名 |
| `imu0_config` | 仅 yaw, vyaw = true | 告诉 EKF："从 IMU 只取航向角和角速度" |
| `imu0_relative` | true | IMU 数据从启动时的值开始累积 |
| `imu0_remove_gravitational_acceleration` | true | 让 EKF 帮你去掉重力加速度（9.8m/s²）的影响 |

---

### 11.7 global_planner 参数

**文件**: [robot_navigation/param/EPRobot/global_planner_params.yaml](robot_navigation/param/EPRobot/global_planner_params.yaml)

| 参数 | 值 | 通俗解释 | 调大/调小建议 |
|------|-----|----------|--------------|
| `use_dijkstra` | true | 是否用 Dijkstra 算法（true）而不是 A*（false） | Dijkstra 保证找到最短路径但较慢；A* 更快但不保证最短。场地不大时 Dijkstra 更好 |
| `allow_unknown` | false | 是否允许路径穿过未知区域（地图上灰色的地方） | 比赛建议保持 false。true 可能导致小车往地图外面走 |
| `default_tolerance` | 0.2 | 路径规划目标点的位置容差（米）。规划时离目标点 0.2m 内就算到 | 保持默认 |
| `use_quadratic` | true | 是否使用二次曲线平滑路径 | true=路径更圆滑。保持默认 |
| `use_grid_path` | false | 是否直接输出栅格路径（不优化） | false=路径经过平滑处理。保持默认 |
| `cost_factor` | 0.6 | 路径代价的缩放系数 | 保持默认 |
| `lethal_cost` | 253 | 判定为"致命障碍"的代价值阈值 | 保持默认 |
| `neutral_cost` | 60 | "中性区域"的代价值 | 保持默认 |

---

## 12. 地图文件

**位置**: [robot_navigation/maps/](robot_navigation/maps/)

| 文件 | 说明 |
|------|------|
| `map.yaml` + `map.pgm` | 默认地图（robot_navigation.launch / robot_race_init.launch 使用） |
| `f1_map.yaml` + `f1_map.pgm` | 备用地图 |

---

## 13. 当前工程中影响导航稳定性的 5 个重点问题

### 问题 1: TEB `yaw_goal_tolerance` 偏大（已修复 ✅）

**位置**: [teb_local_planner_params.yaml:54](robot_navigation/param/EPRobot/teb_local_planner_params.yaml#L54)

**原问题**: 识别板拍摄需要稳定朝向，31° 的角度容差过大，可能导致小车到达航点后摄像头无法正对识别板。

**已修复**: 从 0.55 收紧到 0.10 rad（约 5.7°）。如果到点后长时间原地微调无法到达，可适当放宽到 0.15-0.20。

### 问题 2: 全局和局部 costmap 参数分歧

**位置**:
- [global_costmap_params.yaml:12-13](robot_navigation/param/EPRobot/global_costmap_params.yaml#L12-L13): `inflation_radius=0.5`, `cost_scaling_factor=17`
- [local_costmap_params.yaml:16-17](robot_navigation/param/EPRobot/local_costmap_params.yaml#L16-L17): `inflation_radius=0.15`, `cost_scaling_factor=6`

**影响**: 全局规划路径可能在局部执行时被局部代价地图认为不可行，导致频繁重规划。两个 costmap 的膨胀参数差异较大（全局膨胀 0.5m，局部仅 0.15m）。

**建议**: 将全局 `inflation_radius` 与局部对齐（0.15-0.2），或在确认全局路径确实需要更大安全边距的前提下保留现状（但需明确原因）。

### 问题 3: `controller_frequency` (8Hz) 与 `planner_frequency` (0.2Hz) 差距过大

**位置**: [move_base_params.yaml:2-4](robot_navigation/param/EPRobot/move_base_params.yaml#L2-L4)

**影响**: 全局规划每 5 秒才更新一次，在动态环境或局部路径受阻时，小车可能在错误路径上持续执行较长时间。如果 TEB 局部规划也无法避障，就会触发 recovery。

**建议**: 在比赛场地相对固定的前提下，0.2Hz 可能已足够；若出现频繁的全局重规划延迟，可考虑调大到 1-2Hz。

### 问题 4: AMCL 与 talos_laser_loc 切换机制依赖 launch 选择

**位置**:
- [base_camera_nav.launch:26-39](launch/base_camera_nav.launch#L26-L39)
- [robot_navigation.launch:32-39](robot_navigation/launch/robot_navigation.launch#L32-L39)（AMCL 活跃）
- [robot_race_init.launch:43-50](robot_navigation/launch/robot_race_init.launch#L43-L50)（talos_laser_loc 活跃）

**影响**: 两个 launch 文件中硬编码了不同的定位节点（AMCL 在 robot_navigation.launch 中注释掉了 talos_laser_loc，反之亦然）。如果错误地同时启动了包含 AMCL 和 talos_laser_loc 的 launch，将导致 map→odom TF 冲突。

**建议**: 确保 `base_camera_nav.launch` 中 `use_race_init` 参数正确传递，且同一时间只有其中一个 launch 被包含。

### 问题 5: EKF `odom0_config` 仅融合 vx/vy，位置完全信任里程计

**位置**: [ekf_params.yaml:38-42](robot_navigation/param/EPRobot/ekf_params.yaml#L38-L42)

**影响**: `odom0_config` 中仅有 `vx` 和 `vy` 设为 true，即 EKF 只从 `/odom` 获取速度信息进行融合，位置完全信任里程计的积分结果。如果底盘编码器打滑或累计误差较大，EKF 无法通过其他传感器（如 IMU 的位置信息）进行修正。

**建议**: 这是合理的配置（EKF 定位模式），但需确保：
- `/odom` 发布频率稳定
- AMCL 的 `map→odom` 变换能正常发布以修正漂移
- `imu0_config` 中 yaw 的融合能正常对航向角进行补偿

---

## 14. 推荐调试流程

### Step 1: 确认底盘和里程计

```bash
rostopic echo -n 1 /odom
rostopic hz /odom
```

### Step 2: 确认激光雷达

```bash
rostopic echo -n 1 /scan
rostopic hz /scan
rostopic info /scan_filtered
```

### Step 3: 确认地图

```bash
rostopic echo -n 1 /map
```

### Step 4: 确认 TF

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map base_footprint
rosrun tf view_frames
```

### Step 5: 确认定位

在 RViz 中设置 Fixed Frame 为 `map`，添加 `Map`、`LaserScan`、`RobotModel`、`TF`，使用 `2D Pose Estimate` 初始化位姿，观察激光与地图重合度。

```bash
rostopic echo -n 1 /amcl_pose
rosrun tf tf_echo map odom
```

### Step 6: 确认 move_base

```bash
rostopic echo /move_base/status
rostopic echo /move_base/result
rosservice call /move_base/clear_costmaps
```

### Step 7: 单点导航测试

临时修改 `waypoints.yaml` 只保留一个航点，或使用 RViz `2D Nav Goal` 手动发送目标。

### Step 8: 完整任务测试

```bash
# 常规模式
roslaunch pharmacy_mplus0 race_bringup.launch

# 比赛定位模式
roslaunch pharmacy_mplus0 race_bringup.launch use_race_init:=true
```

---

## 15. 文件修改后是否需要重新编译

| 修改内容 | 是否需编译 | 是否需重启 | 说明 |
|----------|-----------|-----------|------|
| `.py` 脚本 | 不需要 | 需要 | Python 解释执行 |
| `.yaml` 参数 | 不需要 | 需要 | launch 启动时加载 |
| `.launch` 文件 | 不需要 | 需要 | 修改后重新 roslaunch |
| `package.xml` | 建议编译 | 需要 | 依赖变化需重新构建 |
| `CMakeLists.txt` | 需要 | 需要 | 构建规则变化 |

---

## 16. 调试命令速查表

### 启动

```bash
roslaunch pharmacy_mplus0 race_bringup.launch
roslaunch pharmacy_mplus0 race_bringup.launch use_race_init:=true
```

### 节点和话题

```bash
rostopic list
rosnode list
rostopic info /scan /scan_filtered /odom /odometry/filtered
```

### 传感器

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
