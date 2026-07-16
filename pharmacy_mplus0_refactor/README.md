# pharmacy_mplus0

智慧药房双车比赛业务 ROS 包。本目录是从 `dual_car_refactor` 等价迁移得到的规范化版本，重构仅调整 Catkin 包结构、配置组织、资源定位、脚本名称和 launch 入口，不修改识别算法、导航状态机、双车策略或通信协议。

## 1. 部署名称

当前开发目录名是 `pharmacy_mplus0_refactor`。复制到正式 Ubuntu 18.04 小车时，应放置为：

```text
~/robot_ws/src/pharmacy_mplus0/
```

以下三个名称均固定为 `pharmacy_mplus0`：

- `package.xml` 中的 ROS 包名；
- Python 包名；
- `roslaunch` 和 `rosrun` 使用的包名。

同一 Catkin 工作空间内不能同时存在另一个名为 `pharmacy_mplus0` 的 ROS 包。切换新包前，需要先由部署人员处理旧同名包。

## 2. 目录结构

```text
pharmacy_mplus0/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── STRUCTURE_REFACTOR_PLAN.md
├── config/
│   ├── strategy.yaml
│   ├── waypoints.yaml
│   ├── vision.yaml
│   └── communication.yaml
├── launch/
│   ├── race_bringup.launch
│   ├── base_camera_nav.launch
│   ├── main.launch
│   ├── vision.launch
│   └── test_navigation.launch
├── scripts/
│   ├── vision_node.py
│   ├── main_controller.py
│   ├── dual_car_link.py
│   ├── referee_reporter.py
│   └── waypoint_tester.py
├── src/pharmacy_mplus0/
│   ├── __init__.py
│   ├── config.py
│   └── task_logic.py
├── resources/
│   ├── audio/
│   └── board2/
├── docs/
└── test/
```

## 3. 运行依赖

正式环境为 ROS1 Melodic / Ubuntu 18.04 / Python 2.7。ROS 依赖以 `package.xml` 为准，主要包括：

- `rospy`、`roslib`、`std_msgs`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`std_srvs`；
- `actionlib`、`actionlib_msgs`、`move_base_msgs`、`tf`、`visualization_msgs`、`cv_bridge`；
- 同级功能包 `robot_navigation`、`astra_camera`、`web_video_server`。

视觉和音频还使用系统中已经安装的 OpenCV、NumPy、pyzbar/zbar、PyYAML、rospkg 和 SoX `play` 命令。本次重构不安装或升级依赖。

## 4. 资源文件

仓库当前没有正式小车上的板二模板和语音文件，两个资源目录仅含 `.gitkeep`。部署前必须从小车现有资源复制真实文件，不得生成替代文件或修改文件内容。

板二模板放置于 `resources/board2/`，代码使用的文件名为：

```text
free.png
busy_5.png
busy_6.png
busy_7.png
busy_8.png
busy_9.png
busy_10.png
```

语音放置于 `resources/audio/`。实际文件名由主控现有拼接规则决定，包括：

- `WAIT-0.wav`、`WAIT-5.wav`～`WAIT-10.wav`；
- 取样语音前缀 `xuejiang`、`zuzhi`、`tuoye`、`jingmaixue` 加窗口组合；
- 送样语音前缀 `jisu`、`mianyi`、`tiye`、`xuechanggui` 加样本数。

资源通过 `rospkg` 定位当前包，不依赖用户名、工作空间绝对路径或其他智慧药房业务包。

## 5. 配置

| 文件 | 内容 |
|---|---|
| `strategy.yaml` | 状态机时序、双车身份和行为、化验窗口与语音映射 |
| `waypoints.yaml` | 航点、朝向索引和两车起点 |
| `vision.yaml` | 板一、板二、图像输入和两车相机 URL |
| `communication.yaml` | ROS 话题、裁判 TCP 和双车 TCP |

同一套代码通过 `CAR_ID=1` 或 `CAR_ID=2` 选择车辆配置。launch 的 `car_id` 参数会自动设置该环境变量。

当前默认值全部按迁移来源保留，其中包括：

- 板二 `force_label_for_debug: "free"`；
- 板一、板二等待超时均为 0，即无限等待；
- 双车模式在主控中固定开启；
- 双车 token 为 null，来源 IP 检查开启。

这些值可能属于现场调试设置，但本次结构重构没有擅自修正。正式比赛前应由负责人按实际需求单独确认。

## 6. 构建

以下命令只应在正式 Ubuntu 18.04 ROS 工作空间执行：

```bash
cd ~/robot_ws
catkin_make
source devel/setup.bash
rospack find pharmacy_mplus0
```

本次重构所在开发机未执行上述命令。

## 7. 启动入口

### 7.1 完整比赛入口

车 1：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1
```

车 2：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=2
```

完整入口先 include 基础系统，再 include 全部业务节点。

### 7.2 仅基础系统

```bash
roslaunch pharmacy_mplus0 base_camera_nav.launch
```

该入口只启动导航、Astra 摄像头和 web_video_server。

### 7.3 业务节点

```bash
roslaunch pharmacy_mplus0 main.launch car_id:=1
```

`main.launch` 默认启动双车 TCP、裁判通信、视觉和主控。可以通过以下开关单独关闭节点：

```text
start_dual_tcp
start_referee
start_detect
start_nav_pharmacy
```

例如只启动裁判通信：

```bash
roslaunch pharmacy_mplus0 main.launch \
  car_id:=1 \
  start_dual_tcp:=false \
  start_detect:=false \
  start_nav_pharmacy:=false \
  start_referee:=true
```

### 7.4 仅视觉节点

```bash
roslaunch pharmacy_mplus0 vision.launch car_id:=1
```

该入口 include `main.launch`，只开启视觉节点。默认保留 3 秒启动延时和自动 respawn。

### 7.5 单航点测试

首次使用建议只做 dry-run：

```bash
roslaunch pharmacy_mplus0 test_navigation.launch \
  car_id:=1 target:=A dry_run:=true
```

确认坐标后再由现场人员进行实际导航：

```bash
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=A
```

支持目标：`A`、`B`、`C`、`lab1`～`lab4`、`board1`、`board2`、`home`。

## 8. 兼容接口

结构迁移期间以下 ROS 接口保持不变：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/nav_state` | `std_msgs/Int32` | 状态 8～15 |
| `/cam_return` | `std_msgs/Int32MultiArray` | `[C,A,B,count,selected_index,error_window]` |
| `/board2_return` | `std_msgs/Int32MultiArray` | `[state,wait_time]` |
| `/board1_all_text` | `std_msgs/String` | 本车板一完整 JSON |
| `/dual_car/round_done` | `std_msgs/Int32MultiArray` | `[car_id,seq]` |
| `/dual_car/peer_done` | `std_msgs/Int32MultiArray` | `[peer_id,seq]` |
| `/dual_car/peer_board1_all_text` | `std_msgs/String` | 对车板一 JSON |
| `/referee_task` | `std_msgs/String` | `R/A/B/C/1/2/3/4` |
| `/referee_cv1` | `std_msgs/String` | `WAIT-0` 或 `WAIT-5`～`WAIT-10` |
| `/referee_cv2` | `std_msgs/String` | 如 `AB-1` |

双车 TCP 和裁判 TCP 的 JSON 字段、换行分隔、序列号、去重、重试与断线恢复保持不变。详细基线见 `docs/BEHAVIOR_BASELINE.md`。

## 9. 比赛流程

- 车 1先发，车 2等待。
- 取样顺序固定为 `C → A → B`。
- 已完成取样窗口在导航重试时不会重复访问。
- 本车完成化验窗口任务并进入返程状态 15时释放对车。
- 提前收到的令牌缓存到本车回到起点后使用。
- 车 2优先使用车 1板一完整结果并排除已选任务；不可用时回退本车识别。

主控专项静态核对见 `docs/MAIN_CONTROLLER_EQUIVALENCE.md`。

## 10. 当前验证范围

已执行的检查仅限静态结构：

- 新旧节点允许变换后的全文比较；
- launch XML 解析和参数映射；
- 配置键、默认值和迁移路径核对；
- Python 2.7 禁用语法扫描；
- Catkin 元数据、安装节点和目录结构核对。

未执行：

- Python/ROS 节点运行；
- `catkin_make`、`rospack find`、`roslaunch`；
- 摄像头、模板、音频、TF、move_base；
- 双车网络、裁判服务器和实车完整流程。

上述运行验证必须在资源齐全的 Ubuntu 18.04 正式环境完成，不能把本机静态检查视为实车验证通过。

## 11. 已知待办

- 从小车复制真实板二模板和语音文件。
- 确认 `package.xml` 中当前占位许可证 `TODO`。
- 在正式工作空间确认不存在另一个同名 `pharmacy_mplus0` 包。
- 按 `docs/BEHAVIOR_BASELINE.md` 的顺序完成正式环境和双车实车回归。
