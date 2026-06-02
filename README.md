# Smart-Pharmacy

参加 ARAIC 中国机器人及人工智能大赛智慧药房赛项比赛源码。

本仓库是基于 ROS Melodic 的智慧药房比赛项目，包含比赛主控流程、视觉识别、导航调度、裁判通信、双车协作及调试工具。

> 本文档面向新队友快速上手。各功能包的详细说明见包内 `README.md`。

---

## 1. 比赛相关功能包

以下是比赛中实际使用的功能包：

| 功能包 | 来源 | 主要作用 |
|---|---|---|
| [pharmacy_mplus0](pharmacy_mplus0/) | **自研** | 比赛主控状态机、视觉识别、TCP上报、语音播报、双车协作 — **正式比赛入口** |
| [robot_navigation](robot_navigation/) | **自研配置** | 封装导航的 launch、参数、地图，不包含算法代码 |
| [pharmacy_mplus0_debug](pharmacy_mplus0_debug/) | **自研** | 各子系统独立调试工具（假数据、单航点测试、仪表盘、干跑）— **不参与正式比赛** |
| [eprobot_start](eprobot_start/) | 厂商 | 底盘驱动启动（`art_racecar.py`） |
| [base_control](base_control/) | 厂商 | 底盘控制底层，发布 `/odom` 和 `/cmd_vel` |
| [eprobot_description](eprobot_description/) | 厂商 | 机器人 URDF 模型，提供静态 TF |
| [ros_astra_camera](ros_astra_camera/) + [ros_astra_launch](ros_astra_launch/) | 厂商 | Astra RGB-D 摄像头驱动 |
| [navigation-melodic/](navigation-melodic/) | ROS 标准 | move_base、amcl、global_planner、costmap_2d、map_server |
| [teb_local_planner-melodic](teb_local_planner-melodic/) | 开源 | TEB 局部路径规划器（默认） |
| [robot_localization](robot_localization/) | 开源 | EKF 多传感器融合（/odom + /imu_data → /odometry/filtered） |
| [laser_filters](laser_filters/) | 开源 | 激光雷达滤波（/scan → /scan_filtered） |
| [lidar/](lidar/) | 厂商 | 激光雷达驱动（多型号，由 `robot_lidar.launch` 按实车型号选择） |

> 仓库中还有旧比赛包 `pharmacy_pkg` 和 `smart_pharmacy`，仅作历史参考，**不被 `pharmacy_mplus0` 依赖或导入**。
>
> 其余未列出的包（如语音识别、键盘遥控、雷达备选驱动等）为厂商附带或工具包，不参与比赛主流程。

---

## 2. 系统总体架构

### 2.1 Launch 启动层级

```text
race_bringup.launch（正式比赛一键启动）
├── base_camera_nav.launch（基础系统）
│   ├── EPRobot_start.launch          → 底盘驱动（art_racecar.py）
│   ├── robot_navigation.launch       → AMCL + move_base + EKF + 雷达
│   │   ├── robot_lidar.launch        → 激光雷达 + base_control
│   │   ├── map_server                → 静态地图 /map
│   │   ├── amcl                      → 激光定位
│   │   ├── move_base.launch          → 全局规划 + TEB/DWA 局部规划
│   │   └── ekf_localization_node     → EKF 融合 /odometry/filtered
│   ├── astra.launch                  → Astra 摄像头
│   └── web_video_server              → HTTP 视频流
│
└── main.launch（比赛业务层）
    ├── board1_detector.py            → 识别板一：二维码解码 → /board1_detections
    ├── board2_detector.py            → 识别板二：模板匹配 → /cv1_result
    ├── tcp_reporter.py               → TCP JSON 上报 + 语音播报
    └── main_controller.py            → 主控状态机（核心调度器）
```

### 2.2 坐标系 TF 链路

```text
map ──(AMCL)── odom ──(EKF)── base_footprint ── base_link
                                               ├── base_laser_link
                                               ├── IMU_link
                                               └── camera_link
```

---

## 3. 主流程说明

### 3.1 单车比赛流程

```text
起点
  ↓
前往识别板一 → 识别二维码 → 确定任务（体检窗口 + 化验窗口）
  ↓
前往体检窗口取样
  ↓
前往识别板二 → 判断化验区状态 → 等待或通行
  ↓
前往化验窗口投递
  ↓
返回起点 → 进入下一轮
```

### 3.2 双车循环流程

```text
车 1 先出发执行配送 → 车 2 在起点等待
  ↓
车 1 配送完成，返回途中发 ALLOW_START:2
  ↓
车 2 收到信号出发 → 车 1 回到起点等待
  ↓
车 2 配送完成，返回途中发 ALLOW_START:1
  ↓
车 1 收到信号出发 → 循环
```

---

## 4. 主要话题

### 4.1 比赛业务话题

| 话题 | 类型 | 发布者 | 订阅者 | 作用 |
|---|---|---|---|---|
| `/board1_detections` | String(JSON) | board1_detector | main_controller | 识别板一二维码结果 |
| `/cam_return` | Int32MultiArray | board1_detector | main_controller | 识别板一旧格式（兼容） |
| `/cv1_result` | String | board2_detector | main_controller, tcp_reporter | 识别板二状态（WAIT-0~WAIT-10） |
| `/current_task` | String | main_controller | tcp_reporter | 当前任务（A/B/C/1/2/3/4/R） |
| `/cv2_result` | String | main_controller | tcp_reporter | 识别板一任务结果（如"AB-1"） |
| `/announce_request` | String | main_controller | tcp_reporter | 语音播报事件 ID |
| `/reset_detection` | String | main_controller | board1/board2_detector | 解锁识别节点 |

### 4.2 双车协作话题

| 话题 | 类型 | 发布者 | 订阅者 | 作用 |
|---|---|---|---|---|
| `/dual_car_signal` | String | main_controller | main_controller（对车） | 轮流放行（`ALLOW_START:1` / `ALLOW_START:2`） |
| `/current_qr_task` | String | main_controller | main_controller（对车） | 任务占用广播（`CAR1:AB-1`） |

### 4.3 基础系统话题

| 话题 | 类型 | 发布者 | 订阅者 | 作用 |
|---|---|---|---|---|
| `/odom` | Odometry | base_control | tcp_reporter, EKF | 原始里程计 |
| `/odometry/filtered` | Odometry | EKF | move_base, amcl | EKF 融合里程计 |
| `/scan_filtered` | LaserScan | laser_filters | move_base, amcl | 滤波后激光数据 |
| `/cmd_vel` | Twist | move_base | base_control | 底盘速度指令 |
| `/map` | OccupancyGrid | map_server | move_base, amcl | 静态地图 |
| `/camera/rgb/image_raw` | Image | astra_camera | web_video_server | 摄像头图像 |

---

## 5. 主要 launch 文件与启动命令

### 5.1 正式比赛

```bash
# 单车
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1

# 双车 — 车 1（先出发）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true

# 双车 — 车 2（等待放行）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true
```

### 5.2 分步启动（赛前验证）

```bash
# 先启动基础系统
roslaunch pharmacy_mplus0 base_camera_nav.launch

# 确认定位正常后启动业务
roslaunch pharmacy_mplus0 main.launch car_id:=1
```

### 5.3 其他参数

```bash
# 修改裁判 IP
roslaunch pharmacy_mplus0 race_bringup.launch server_ip:=192.168.12.100

# DWA 规划器（默认 TEB）
roslaunch pharmacy_mplus0 base_camera_nav.launch planner:=dwa

# 仅启动识别节点（安全，不动）
roslaunch pharmacy_mplus0 detectors.launch
```

### 5.4 调试工具

```bash
# 单航点导航测试
roslaunch pharmacy_mplus0_debug test_navigation.launch target:=exam_A

# 主控干跑（无摄像头、无导航）
roslaunch pharmacy_mplus0_debug test_full_dryrun.launch

# 终端仪表盘
rosrun pharmacy_mplus0_debug topic_echo_dashboard.py

# TCP 上报测试
roslaunch pharmacy_mplus0_debug test_reporter.launch
```

详细调试命令见 [pharmacy_mplus0_debug/README.md](pharmacy_mplus0_debug/README.md)。

---

## 6. 配置文件一览

| 配置文件 | 所属包 | 作用 |
|---|---|---|
| [config/strategy.yaml](pharmacy_mplus0/config/strategy.yaml) | pharmacy_mplus0 | 比赛策略（超时/停留/访问顺序/双车开关） |
| [config/waypoints.yaml](pharmacy_mplus0/config/waypoints.yaml) | pharmacy_mplus0 | 所有航点坐标（10个） |
| [config/tcp.yaml](pharmacy_mplus0/config/tcp.yaml) | pharmacy_mplus0 | TCP 上报（裁判IP/端口/车号/频率） |
| [config/vision.yaml](pharmacy_mplus0/config/vision.yaml) | pharmacy_mplus0 | 视觉识别参数（视频流/二维码/模板匹配） |
| [param/EPRobot/teb_local_planner_params.yaml](robot_navigation/param/EPRobot/teb_local_planner_params.yaml) | robot_navigation | TEB 规划器（速度/精度/避障权重） |
| [param/EPRobot/move_base_params.yaml](robot_navigation/param/EPRobot/move_base_params.yaml) | robot_navigation | move_base 全局参数 |
| [param/EPRobot/amcl_params.yaml](robot_navigation/param/EPRobot/amcl_params.yaml) | robot_navigation | AMCL 定位参数 |
| [param/EPRobot/ekf_params.yaml](robot_navigation/param/EPRobot/ekf_params.yaml) | robot_navigation | EKF 融合参数 |
| [param/EPRobot/*costmap*.yaml](robot_navigation/param/EPRobot/) | robot_navigation | 代价地图参数（全局/局部/通用） |
| [maps/map.yaml](robot_navigation/maps/map.yaml) | robot_navigation | 比赛场地地图 |

---

## 7. 双车协作

- **默认关闭**：`dual_car_enabled` 默认为 `false`，单车模式不受影响。
- **轮流出发**：车 1 先出发，配送完成返回途中通过 `/dual_car_signal` 放行车 2，两车间隔运行。
- **任务占用排除**：每车选定二维码后通过 `/current_qr_task` 广播占用，对车排除已被占用的方框。
- **远程任务共享**预留接口但默认关闭（`dual_car_remote_task_enabled: false`）。

详细说明见 [pharmacy_mplus0/README_DUAL_CAR.md](pharmacy_mplus0/README_DUAL_CAR.md)。

---

## 8. 编译与部署

```bash
cd ~/robot_ws

# 忽略有编译问题的厂商包
touch src/yujin_ocs/CATKIN_IGNORE
touch src/xf_mic_asr_offline_circle/CATKIN_IGNORE
touch src/talos_laser_loc/CATKIN_IGNORE

# 编译
catkin_make
source devel/setup.bash
```

修改 Python 脚本、YAML 配置、launch 文件**不需要重新编译**，重启节点即生效。

同步到小车：

```bash
scp -r ~/Smart-Pharmacy/pharmacy_mplus0/ EPRobot@<小车IP>:~/robot_ws/src/pharmacy_mplus0/
```

---

## 9. 常见问题与排查

### Q1：roslaunch 找不到功能包

```bash
source ~/robot_ws/devel/setup.bash
rospack find pharmacy_mplus0
```

### Q2：小车无法导航

```bash
rostopic echo /map              # 地图是否加载
rostopic echo /scan_filtered    # 雷达是否正常
rosrun tf view_frames           # TF 树是否正确
```

### Q3：识别板一无结果

1. 浏览器打开 `http://<小车IP>:8080/stream?topic=/camera/rgb/image_raw` 确认视频流
2. 确认 web_video_server 在运行
3. 检查 `vision.yaml` 中 `rotate_degrees` 是否匹配摄像头角度

### Q4：识别板二不锁定

1. 确认 `templates/board2/` 下有 `idle.png` ~ `wait10.png`
2. 降低 `vision.yaml` 中 `match_threshold`（如 0.72 → 0.55）
3. 用 `board2_template_capture.py` 现场重新采集模板

### Q5：主控卡住不动

```bash
rosrun pharmacy_mplus0_debug topic_echo_dashboard.py  # 仪表盘
rostopic echo /current_task                            # 确认当前状态
```

### Q6：双车模式车 2 不出发

```bash
rostopic echo /dual_car_signal
rostopic pub /dual_car_signal std_msgs/String "data: 'ALLOW_START:2'" -1  # 手动放行
```

### Q7：TCP 连接失败

不影响比赛 —— 连接失败只丢弃上报数据，不阻塞主控。用假服务端测试：

```bash
rosrun pharmacy_mplus0_debug tcp_fake_server.py _port:=8888
```

---

## 10. 开发与维护建议

- 视觉节点只输出识别结果，不参与业务决策。
- 任务规划是纯逻辑，修改后先跑 `verify_logic.py` 验证。
- 所有现场可调参数放入 YAML，不写进 Python 代码。
- 不要在正式比赛主流程中依赖 `pharmacy_mplus0_debug`。
- 修改航点坐标后不需要编译，重启 launch 即生效。
- 赛前检查：确认分支为 `main`、无未提交修改、Python 脚本有执行权限。
