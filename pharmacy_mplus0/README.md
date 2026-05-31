# pharmacy_mplus0

智慧药房比赛主功能包 —— 负责完整比赛流程的调度与执行。

> **重要约束：本包不依赖、不导入、不 include 旧比赛包 `pharmacy_v5`、`pharmacy_pkg`、`smart_pharmacy`。旧包仅作为历史只读参考。**

---

## 1. 功能简介

`pharmacy_mplus0` 是 2026 CRAIC 智慧药房比赛的应用层控制包，完成以下任务：

- **视觉识别**：识别板一（二维码解码）和识别板二（化验区状态模板匹配）。
- **任务规划**：从多二维码结果中选择最优任务（优先最大化样本数），规划体检窗口访问顺序。
- **导航调度**：通过 `move_base` 依次前往体检窗口 → 识别板二 → 化验窗口 → 起点。
- **裁判状态上报**：通过 TCP 向裁判软件上报速度、坐标、任务状态、CV1、CV2。
- **语音播报**：在体检区取样、识别板二、化验区投递等节点按规则播报。
- **样本追踪**：记录每轮携带的样本类型和窗口，投递时校验收发一致性。
- **双车协作预留**：通过 `/current_qr_task` 广播当前任务，单车模式不影响。

适用场景：正式比赛（`race_bringup.launch`）、业务层调试（`main.launch`）、识别节点单独验证（`detectors.launch`）。

---

## 2. 功能包目录结构

```
pharmacy_mplus0/
├── CMakeLists.txt                # catkin 构建文件
├── package.xml                   # ROS 包依赖声明
├── setup.py                      # catkin_python_setup 入口
├── README.md                     # 本文件
├── config/
│   ├── waypoints.yaml            # 航点坐标和窗口映射
│   ├── strategy.yaml             # 比赛策略参数（超时/停留/访问顺序）
│   ├── tcp.yaml                  # TCP 上报参数（裁判 IP/端口/频率）
│   └── vision.yaml               # 视觉识别参数（Canny/透视/模板匹配）
├── launch/
│   ├── race_bringup.launch       # 【正式比赛入口】一键全量启动
│   ├── base_camera_nav.launch    # 仅启动底盘+导航+摄像头+视频流
│   ├── main.launch               # 业务层启动（4 个比赛节点）
│   ├── main_single.launch        # 静默语音模式
│   ├── detectors.launch          # 仅两个识别节点
│   └── reporter.launch           # 仅 TCP 上报节点
├── scripts/                      # ROS 节点入口（可执行脚本）
│   ├── main_controller.py        # 主控状态机
│   ├── board1_detector.py        # 识别板一 ROS 节点
│   ├── board2_detector.py        # 识别板二 ROS 节点
│   ├── tcp_reporter.py           # TCP 上报 + 语音播报节点
│   ├── detection_monitor.py      # 终端状态仪表盘
│   └── verify_logic.py           # 纯 Python 离线逻辑自检
├── src/pharmacy_mplus0/          # Python 库模块（纯逻辑，可单独测试）
│   ├── __init__.py
│   ├── constants.py              # 比赛常量（状态名/话题名/窗口映射）
│   ├── models.py                 # 数据结构（Board1Detection 等）
│   ├── task_planner.py           # 任务选择与轮次规划
│   ├── sample_store.py           # 样本携带与投递记录
│   ├── navigation_client.py      # move_base 导航封装
│   ├── competition_io.py         # 裁判可见状态发布封装
│   ├── board1_decoder.py         # 识别板一解码核心算法
│   ├── board2_matcher.py         # 识别板二模板匹配核心
│   ├── tcp_client.py             # TCP 异步连接与发送
│   ├── voice.py                  # WAV 音频播放封装
│   └── log_utils.py              # 中文日志工具
└── templates/board2/             # 识别板二模板图片目录
    └── .gitkeep                  # 占位，模板需从小车复制
```

---

## 3. 主要文件说明

### 3.1 Launch 文件

| 文件路径 | 作用说明 | 备注 |
| --- | --- | --- |
| `launch/race_bringup.launch` | 正式比赛一键全量启动：底盘→导航→摄像头→视频流→4个业务节点 | 通过 `start_*` 参数可跳过已启动的子系统 |
| `launch/base_camera_nav.launch` | 仅启动基础系统（底盘/导航/摄像头/视频流），不启动业务 | 赛前验证定位、导航、视频流 |
| `launch/main.launch` | 启动 4 个比赛业务节点：board1_detector + board2_detector + tcp_reporter + main_controller | 不启动底盘/导航/摄像头 |
| `launch/main_single.launch` | 同 `main.launch`，但 `audio_dir=""` 禁用语音 | 单车调试或双车跟车 |
| `launch/detectors.launch` | 仅启动 board1_detector + board2_detector | 安全，小车不会运动 |
| `launch/reporter.launch` | 仅启动 tcp_reporter | 单独调试 TCP 和播报 |

**Launch 文件层级关系**：
```
race_bringup.launch
  ├── base_camera_nav.launch  (可开关)
  │     ├── eprobot_start/EPRobot_start.launch     (底盘)
  │     ├── robot_navigation/robot_navigation.launch (导航)
  │     ├── astra_camera/astra.launch               (摄像头)
  │     └── web_video_server                         (HTTP 视频流)
  └── main.launch  (可开关)
        ├── board1_detector.py
        ├── board2_detector.py
        ├── tcp_reporter.py
        └── main_controller.py

main_single.launch → main.launch (audio_dir="")
```

### 3.2 脚本（ROS 节点入口）

| 文件路径 | 作用说明 | 备注 |
| --- | --- | --- |
| `scripts/main_controller.py` | 主控状态机（11 个状态）：串联导航、识别、样本、播报的全部流程 | 核心调度器，需要 move_base 可用 |
| `scripts/board1_detector.py` | 识别板一 ROS 节点：读取 HTTP 视频流 → 解码二维码 → 发布 `/cam_return` 等 | 发布后锁定，收到 `/reset_detection` 解锁 |
| `scripts/board2_detector.py` | 识别板二 ROS 节点：读取视频流 → 模板匹配 → 发布 `/cv1_result` | 连续 N 帧一致后锁定 |
| `scripts/tcp_reporter.py` | TCP 状态上报 + 语音播报 ROS 节点：订阅 odom/TF/任务话题 → 定时 JSON 上报 | TCP 连接失败不阻塞 |
| `scripts/detection_monitor.py` | 终端仪表盘：清屏刷新显示 9 个关键话题的最新值 | 用于现场快速定位问题 |
| `scripts/verify_logic.py` | 离线逻辑自检：用变量构造假数据测试 task_planner + sample_store | 不依赖 ROS，可在开发机直接跑 |

### 3.3 库模块（src/pharmacy_mplus0/）

| 文件路径 | 作用说明 | 备注 |
| --- | --- | --- |
| `constants.py` | 所有比赛常量：状态名、话题名、窗口映射、播报模板、默认访问顺序 | 集中管理，避免各模块散落字符串 |
| `models.py` | 数据结构：`Board1Detection`、`RoundPlan`、`Board2Status` 及 `make_board1_detection` 等工厂函数 | 使用 namedtuple |
| `task_planner.py` | 从识别板一结果中选择最优任务（按样本数最大化），构建本轮配送计划 | 支持 `from_cam_return` 解析旧格式 |
| `sample_store.py` | 记录本轮携带的样本类型和窗口，阻止混装不同样本类型 | 每轮开始/投递完成后清空 |
| `navigation_client.py` | 封装 `move_base` SimpleActionClient：`go_to("exam_A")`、`clear_costmaps()` | 坐标全部从 waypoints.yaml 读取 |
| `competition_io.py` | 统一发布 5 个裁判话题 + 语音播报请求 | 提供 `set_task("A")` 等语义化接口 |
| `board1_decoder.py` | 识别板一核心算法：直接 pyzbar 整帧解码 → 方框位置推断 → 稳定性过滤（旧方案代码保留在文件末尾供参考） | 纯算法，可离线用图片测试 |
| `board2_matcher.py` | 识别板二核心算法：加载模板 → 多尺度 TM_CCOEFF_NORMED 匹配 | 纯算法，不依赖 ROS |
| `tcp_client.py` | TCP 异步客户端：后台线程连接、非阻塞发送、断线自动重连 | 不依赖 ROS |
| `voice.py` | WAV 音频播放：根据 event_id 播放预录制 wav 文件，aplay 异步执行 | 支持 aplay / paplay / ffplay，文件缺失不崩溃 |
| `log_utils.py` | 中文日志工具：直接写 stdout 绕过 rospy.log* 的 Python 2 编码陷阱 | 提供 `loginfo_throttle` / `logwarn_throttle` |

### 3.4 配置文件

| 文件路径 | 作用说明 | 关键字段 |
| --- | --- | --- |
| `config/waypoints.yaml` | 所有航点坐标及窗口映射 | `frames.map`, `frames.robot`, `waypoints.exam_A~C`, `waypoints.lab_1~4`, `waypoints.start/board1/board2`, `exam_waypoints`, `lab_waypoints` |
| `config/strategy.yaml` | 比赛策略和超时参数 | `dwell.exam_seconds`(1.5), `timeouts.board1_wait_seconds`(15), `visit_order`, `rounds.round_return_to_start` |
| `config/tcp.yaml` | TCP 上报参数 | `server.ip`(192.168.12.16), `server.port`(8888), `report.car_id`, `report.hz`(2.0) |
| `config/vision.yaml` | 视觉识别参数 | `camera.stream_url`, `board1.rotate_degrees/stable_frames/pyzbar_symbols`（旧 Canny/轮廓参数已弃用保留）, `board2.match_threshold`(0.72) |

---

## 4. 编译方法

本包位于 `Smart-Pharmacy/`（对应小车上的 `~/robot_ws/src/`）。

```bash
cd ~/robot_ws

# 注意：src 下部分厂家包（非比赛所需）存在 C++ 源码缺失或 cmake 配置错误，
# 会阻断 catkin_make 的全量扫描。编译前需要先忽略它们：
touch src/yujin_ocs/CATKIN_IGNORE
touch src/xf_mic_asr_offline_circle/CATKIN_IGNORE
touch src/talos_laser_loc/CATKIN_IGNORE

# 编译
catkin_make
source devel/setup.bash
```

**被忽略的厂家包说明**：

| 包名 | 忽略原因 | 是否影响比赛 |
|------|---------|:---:|
| `yujin_ocs` | 出厂残留元包，cmake 配置错误 | 否 |
| `xf_mic_asr_offline_circle` | 与 `xf_mic_asr_offline` 重复，同名 C++ target | 否 |
| `talos_laser_loc` | 源码缺失（`.cpp` 文件不存在） | 否 |

以上三个包不涉及底盘、导航、摄像头、雷达、TF 等核心功能，忽略后不影响比赛。

**编译注意事项**：

| 操作 | 是否需要重新编译 |
| --- | --- |
| 修改 `src/pharmacy_mplus0/*.py` 中的库模块 | 不需要（Python 是解释执行） |
| 修改 `scripts/*.py` | 不需要，但需确保有 `chmod +x` 权限 |
| 修改 `config/*.yaml` | 不需要 |
| 修改 `launch/*.launch` | 不需要 |
| 修改 `CMakeLists.txt` 或 `package.xml` | **需要**重新 `catkin build` |

**验证包是否正确安装**：

```bash
rospack find pharmacy_mplus0
# 应输出: /home/EPRobot/robot_ws/src/pharmacy_mplus0

roscd pharmacy_mplus0
# 应切换到上述目录

roslaunch pharmacy_mplus0 detectors.launch
# 验证 launch 文件可被找到（节点会因无视频流报错，但至少说明包可被识别）
```

---

## 5. 运行方法

### 5.1 推荐运行顺序

```bash
# 第一步：启动基础系统（底盘+导航+摄像头+视频流）
roslaunch pharmacy_mplus0 base_camera_nav.launch

# 确认定位和导航正常后，第二步：启动比赛业务
roslaunch pharmacy_mplus0 main.launch
```

### 5.2 各启动场景

| 场景 | 命令 |
| --- | --- |
| 正式比赛一键启动 | `roslaunch pharmacy_mplus0 race_bringup.launch` |
| 赛前基础系统验证 | `roslaunch pharmacy_mplus0 base_camera_nav.launch` |
| 业务层调试/重启 | `roslaunch pharmacy_mplus0 main.launch` |
| 单车静默调试 | `roslaunch pharmacy_mplus0 main_single.launch` |
| 单独调识别 | `roslaunch pharmacy_mplus0 detectors.launch` |
| 单独调上报 | `roslaunch pharmacy_mplus0 reporter.launch` |

### 5.3 带参数覆盖的运行示例

```bash
# 修改裁判 IP
roslaunch pharmacy_mplus0 race_bringup.launch server_ip:=192.168.12.100

# 跳过已启动的底盘和导航
roslaunch pharmacy_mplus0 race_bringup.launch \
  start_base:=false start_navigation:=false \
  start_camera:=false start_video_server:=false

# 使用 DWA 规划器（默认 teb）
roslaunch pharmacy_mplus0 base_camera_nav.launch planner:=dwa

# 2 号车静默模式
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=2 audio_dir:=""
```

### 5.4 离线逻辑自检（无需 ROS）

```bash
cd ~/robot_ws/src/pharmacy_mplus0
python scripts/verify_logic.py
# 预期输出: first-stage logic verification passed
```

该脚本用变量构造假数据代替真实 ROS 消息，在开发机上验证 task_planner 和 sample_store 的核心逻辑是否正确。

---

## 6. 参数调试说明

### 6.1 常用调试参数（strategy.yaml）

| 参数名 | 所在文件 | 默认值 | 作用 | 调大/调小的影响 |
| --- | --- | --- | --- | --- |
| `dwell.exam_seconds` | strategy.yaml | 1.5 | 体检窗口停留时间(秒) | 太小可能不满足"明显停留"规则加分；太大浪费时间 |
| `dwell.lab_seconds` | strategy.yaml | 1.5 | 化验窗口停留时间(秒) | 同上 |
| `dwell.board1_settle_seconds` | strategy.yaml | 1.0 | 到识别板一后等待画面稳定 | 视觉晃动时可适当增大 |
| `timeouts.board1_wait_seconds` | strategy.yaml | 15.0 | 单次等待二维码超时 | 二维码更新慢可增大，但会拖慢失败重试节奏 |
| `timeouts.board1_max_retry` | strategy.yaml | 3 | 识别板一最大重试次数 | 增大可容忍偶发失败但拖长总时间 |
| `timeouts.board2_wait_seconds` | strategy.yaml | 10.0 | 识别板二最大等待时间 | 超时后按"空闲"处理，不影响任务继续 |
| `timeouts.nav_default_seconds` | strategy.yaml | 40.0 | 单段导航默认超时 | 场地大时可增大；太小会导致频繁导航超时 |
| `timeouts.max_consecutive_fail_rounds` | strategy.yaml | 5 | 连续失败 N 轮后原地停车 | 设为 0 可禁用自动停车保护 |
| `task_priority.prefer_more_samples` | strategy.yaml | true | 是否优先选样本最多的二维码 | false 时按方框顺序选 |
| `rounds.round_return_to_start` | strategy.yaml | true | 每轮结束是否回起点 | false 直接去识别板一（节省时间） |

### 6.2 视觉识别参数（vision.yaml — board1 段）

新方案只需 3 个参数，简洁易调：

| 参数名 | 默认值 | 作用 | 调大/调小的影响 |
| --- | --- | --- | --- |
| `rotate_degrees` | 0 | 画面旋转校正角度 | 摄像头安装角度偏差时调整（如 ±3 度） |
| `stable_frames` | 3 | 连续稳定帧数（解码器内部稳定性过滤 + ROS 节点锁定） | 增大减少误检但增加延迟；设为 1 跳过稳定性过滤 |
| `pyzbar_symbols` | ["QRCODE"] | pyzbar 扫描的条码类型 | 通常只扫 QR 码即可；可选 CODE128、EAN13 等 |

> **旧方案参数**（`canny_low/high`、`contour_min_area/max_area`、`square_wh_rate`、`center_distance_threshold`、`crop_regions` 等）已在 `vision.yaml` 中标记为"已弃用，保留供参考"，新方案不再使用。

### 6.3 视觉识别参数（vision.yaml — board2 段）

| 参数名 | 默认值 | 作用 | 调大/调小的影响 |
| --- | --- | --- | --- |
| `match_threshold` | 0.72 | 模板匹配置信度阈值 | 降低容忍误匹配；增大减少漏检 |
| `debounce_frames` | 3 | 锁定去抖帧数 | 同 stable_frames |
| `scales` | [0.7,0.8,...,1.2] | 多尺度匹配比例 | 缩小范围提升速度；扩大范围适应更远/更近距离 |

### 6.4 Launch 可覆盖参数

所有 launch 文件中以 `<arg>` 声明的参数都可在命令行覆盖，常用：

| 参数 | 可覆盖的 launch | 默认值 | 作用 |
| --- | --- | --- | --- |
| `server_ip` | main, race_bringup, reporter | 192.168.12.16 | 裁判电脑 IP |
| `server_port` | 同上 | 8888 | 裁判软件端口 |
| `car_id` | 同上 | "1" | 小车编号 |
| `audio_dir` | main, race_bringup, reporter | $(find pharmacy_mplus0)/audio | wav 音频文件目录，设为空字符串可禁用语音 |
| `audio_player` | 同上 | aplay | 音频播放器：aplay/paplay/ffplay |
| `allow_overlap` | 同上 | false | 是否允许音频重叠播放 |
| `stream_url` | main, detectors, race_bringup | http://192.168.12.1:8080/stream?... | 摄像头 HTTP 视频流 |
| `planner` | base_camera_nav, race_bringup | teb | 路径规划器：dwa/teb |
| `start_base/navigation/camera/video_server` | race_bringup | true | 各子系统开关 |
| `dry_run` | main_controller (debug launch) | false | 为 true 时跳过 move_base，所有导航直接模拟成功 |

---

## 7. ROS 节点、话题和坐标系关系

### 7.1 节点列表

| 节点名 | 来自文件 | 主要功能 |
| --- | --- | --- |
| `/main_controller` | `scripts/main_controller.py` | 主控状态机，调度全部比赛流程 |
| `/board1_detector` | `scripts/board1_detector.py` | 识别板一：HTTP视频流→二维码解码→发布结果 |
| `/board2_detector` | `scripts/board2_detector.py` | 识别板二：HTTP视频流→模板匹配→发布结果 |
| `/tcp_reporter` | `scripts/tcp_reporter.py` | TCP上报(odom+TF+CV话题)+语音播报 |
| `/detection_monitor` | `scripts/detection_monitor.py` | 终端仪表盘（调试用，不参与比赛流程） |

### 7.2 话题列表

#### 识别板一相关

| 话题名 | 消息类型 | 发布者 | 订阅者 | 作用 |
| --- | --- | --- | --- | --- |
| `/cam_return` | `Int32MultiArray` | board1_detector | main_controller | 旧格式：[is_C,is_A,is_B,count,box_idx,error] |
| `/board1_detections` | `String`(JSON) | board1_detector | main_controller | 新格式JSON：[{code,box_index,lab_window,...}] |
| `/all_qrcodes` | `String`(JSON) | board1_detector | — | 所有方框解码结果（调试/双车预留） |

#### 识别板二相关

| 话题名 | 消息类型 | 发布者 | 订阅者 | 作用 |
| --- | --- | --- | --- | --- |
| `/cv1_result` | `String` | board2_detector | main_controller, tcp_reporter | "WAIT-0" / "WAIT-5"~"WAIT-10" |
| `/board2_status` | `String`(JSON) | board2_detector | — | 新格式：{wait_seconds, is_idle} |

#### 裁判可见状态（由 main_controller 通过 CompetitionIO 发布）

| 话题名 | 消息类型 | 发布者 | 订阅者 | 作用 |
| --- | --- | --- | --- | --- |
| `/current_task` | `String` | main_controller | tcp_reporter | 当前任务："A"/"B"/"C"/"1"/"2"/"3"/"4"/"R" |
| `/cv2_result` | `String` | main_controller | tcp_reporter | 识别板一任务结果："AB-1" |
| `/current_qr_task` | `String` | main_controller | —（双车预留） | 当前执行的二维码任务 |
| `/announce_request` | `String` | main_controller | tcp_reporter | 音频事件 ID（如 board2_idle、lab_blood_3） |
| `/reset_detection` | `String` | main_controller | board1_detector, board2_detector | 通知识别节点解锁，重新识别 |

#### 基础系统话题（由厂家包提供）

| 话题名 | 消息类型 | 发布者 | 订阅者 | 作用 |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | base_control | tcp_reporter | 里程计（取速度） |
| `/cmd_vel` | `geometry_msgs/Twist` | move_base | base_control | 底盘速度指令 |
| `/scan` 或 `/scan_filtered` | `sensor_msgs/LaserScan` | lidar 节点 | move_base, amcl | 激光雷达数据 |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | astra_camera | web_video_server | 摄像头原始图像 |
| `/move_base/clear_costmaps` | `std_srvs/Empty` | —(服务) | main_controller | 清理代价地图 |

### 7.3 坐标系关系

```text
map ──(AMCL)── odom ──(EKF/里程计)── base_footprint ── base_link
                                                   ├── base_laser_link
                                                   └── camera_link
```

- `map`：全局地图坐标系（AMCL 定位结果）。
- `odom` 或 `/odometry/filtered`：里程计累计坐标系（经 EKF 融合）。
- `base_footprint`：底盘投影到地面（小车坐标系原点，TF 查询目标）。
- `base_link`：底盘本体。
- `base_laser_link`：激光雷达位置。
- `camera_link`：摄像头位置。

**检查命令**：
```bash
rosrun tf view_frames              # 生成 frames.pdf
rosrun rqt_tf_tree rqt_tf_tree     # GUI 查看 TF 树
rosrun tf tf_echo map base_footprint  # 实时查看小车在地图中的位置
```

---

## 8. 调试与排错方法

### 8.1 基础排查命令

```bash
# 查看所有节点和话题
rosnode list
rostopic list

# 查看某个话题的详细信息
rostopic info /cam_return
rostopic info /current_task

# 实时查看话题数据
rostopic echo /cam_return
rostopic echo /cv1_result
rostopic echo /announce_request

# 查看话题发布频率
rostopic hz /odom
rostopic hz /current_task

# 查看参数
rosparam list
rosparam get /main_controller/timeouts

# 节点/话题关系图
rosrun rqt_graph rqt_graph
```

### 8.2 离线逻辑自检

```bash
cd ~/robot_ws/src/pharmacy_mplus0
python scripts/verify_logic.py
# 通过则说明 task_planner + sample_store 核心逻辑无问题
```

### 8.3 状态机卡死排查

如果小车停在某个状态不动：
1. 运行 `rosrun pharmacy_mplus0_debug topic_echo_dashboard.py` 查看各话题当前值。
2. 检查 `rostopic echo /current_task` — 如果始终是 "R"，说明在导航中；如果卡在某个状态没变，说明该状态的超时逻辑未触发。
3. 检查 `rostopic echo /cam_return` — 如果始终无数据，问题在识别板一。

### 8.4 RViz 调试

- **Fixed Frame** 应选择 `map`。
- 如果 RViz 没有地图：检查 `rostopic list | grep map` 确认 `/map` 话题存在。
- 如果机器人不移动：检查 `/tf` 和 `/odom`。
- 如果雷达无显示：检查 `/scan_filtered` 话题。

### 8.5 摄像头与视觉调试

```bash
# 确认摄像头话题
rostopic list | grep camera

# 用 image_view 查看画面
rosrun image_view image_view image:=/camera/rgb/image_raw

# 查看识别板一结果
rostopic echo /cam_return
rostopic echo /board1_detections

# 查看识别板二结果
rostopic echo /cv1_result
```

### 8.6 TCP 上报调试

```bash
# 启动假服务端
rosrun pharmacy_mplus0_debug tcp_fake_server.py

# 启动上报节点（指向本机）
rosrun pharmacy_mplus0 tcp_reporter.py _server_ip:=127.0.0.1 _audio_dir:=""

# 用假数据填充状态
rosrun pharmacy_mplus0_debug send_fake_task.py _task:=A _cv2:=AB-1
```

---

## 9. 常见问题 FAQ

### Q1：roslaunch 找不到 pharmacy_mplus0

原因：未 source 工作空间或包未编译。

解决：
```bash
cd ~/robot_ws
catkin build pharmacy_mplus0
source devel/setup.bash
rospack find pharmacy_mplus0  # 验证
```

### Q2：启动 main_controller 后报"move_base action 服务器未就绪"

原因：move_base 未启动或未就绪。

解决：
1. 正式比赛 — 先启动导航（`robot_navigation.launch`），确认 `rostopic list | grep move_base` 有相关话题后再启主控。
2. 干跑调试 — 使用 `pharmacy_mplus0_debug/test_full_dryrun.launch`，该 launch 已默认启用 `dry_run=true`，自动跳过所有导航请求。

### Q3：识别板一始终无结果

排查步骤：
1. 确认摄像头视频流可用：浏览器打开 `http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw`
2. 确认 `web_video_server` 在运行。
3. 确认二维码在画面中清晰可见、未被遮挡。
4. 检查 `vision.yaml` 中 `board1.rotate_degrees` 是否与实际摄像头角度匹配。
5. 新方案使用直接 pyzbar 解码，不依赖 Canny/轮廓参数，旧参数已弃用。

### Q4：识别板二始终不锁定

原因：模板匹配分数不够或模板图片缺失。

解决：
1. 确认 `pharmacy_mplus0/templates/board2/` 下有 `idle.png` ~ `wait10.png`。
2. 降低 `vision.yaml` 中 `board2.match_threshold`（如 0.72 → 0.55）。
3. 用 `pharmacy_mplus0_debug/board2_template_capture.py` 重新采集模板。

### Q5：Python 文件没有执行权限

```bash
chmod +x ~/robot_ws/src/pharmacy_mplus0/scripts/*.py
```

### Q6：修改 strategy.yaml 后参数未生效

修改 YAML 后直接重启节点即可生效（Python 每次启动会重新读取），不需要重新编译。

### Q7：主控连续失败后原地停车不再动

原因：`max_consecutive_fail_rounds` (默认 5) 达到，触发了死循环保护。

解决：重新启动 `main_controller` 节点即可重置计数。

### Q8：TCP 连接失败会影响比赛吗

不会。`tcp_client.py` 的连接和发送失败只丢弃数据、输出警告日志，**不阻塞主控状态机**。比赛期间即便裁判软件未连接，小车仍能正常完成配送任务。

---

## 10. 二次开发说明

| 修改目标 | 应关注的文件 |
| --- | --- |
| 调整比赛策略（超时/停留/访问顺序） | `config/strategy.yaml` |
| 修改航点坐标 | `config/waypoints.yaml`，修改后无需重新编译 |
| 调整视觉参数 | `config/vision.yaml` |
| 修改 TCP 上报格式/频率 | `config/tcp.yaml` + `scripts/tcp_reporter.py` |
| 修改状态机流程 | `scripts/main_controller.py` |
| 修改任务选择逻辑 | `src/pharmacy_mplus0/task_planner.py` |
| 修改导航行为 | `src/pharmacy_mplus0/navigation_client.py` |
| 修改播报音频映射 | `src/pharmacy_mplus0/constants.py` 的 `LAB_WINDOW_AUDIO_KEYS` / `SAMPLE_TYPE_AUDIO_KEYS` |
| 修改识别板一算法 | `src/pharmacy_mplus0/board1_decoder.py` |
| 修改识别板二算法 | `src/pharmacy_mplus0/board2_matcher.py` |
| 替换语音后端 | `src/pharmacy_mplus0/voice.py` |
| 新增 launch 参数 | 对应 `launch/*.launch` 中的 `<arg>` 块 |
| 新增 ROS 话题 | 先在 `constants.py` 加 `TOPIC_*`，再在对应模块发布/订阅 |

**开发原则**：
- 视觉节点只输出识别结果，不参与业务决策。
- 任务规划是纯逻辑，修改后先跑 `verify_logic.py` 验证。
- 所有现场可调参数放入 YAML，不要写进 Python 代码。
- 不要在正式比赛主流程中依赖 `pharmacy_mplus0_debug`。

---

## 11. Git 协作指南

> 仓库地址：**[https://github.com/Mplus0/Smart-Pharmacy](https://github.com/Mplus0/Smart-Pharmacy)**

### 11.1 首次克隆

```bash
git clone https://github.com/Mplus0/Smart-Pharmacy.git ~/Smart-Pharmacy
```

> 小车上的工作空间在 `~/robot_ws/src/`，开发机 clone 到任意路径即可。
> 小车上通常不直接 clone，而是通过 U 盘或 `scp` 同步 `Smart-Pharmacy/` 目录到
> `~/robot_ws/src/` 下。

### 11.2 分支策略

| 分支 | 用途 | 说明 |
|------|------|------|
| `main` | 稳定主线 | 已通过干跑验证、可在小车上运行的代码 |
| `dev` | 开发分支 | 新功能、实验性修改的集成分支 |
| `fix/<描述>` | 问题修复 | 从 `main` 拉出，修完合回 `main` |
| `feat/<描述>` | 新功能 | 从 `dev` 拉出，做完合回 `dev` |

**日常开发流程**：

```bash
# 1. 同步最新代码
git checkout main
git pull origin main

# 2. 从 main 拉修复分支（或从 dev 拉功能分支）
git checkout -b fix/board1-timeout

# 3. 修改代码，多次小步提交
git add pharmacy_mplus0/config/strategy.yaml
git commit -m "fix: 识别板一超时从 15s 调整为 20s"

# 4. 推送到远程
git push origin fix/board1-timeout

# 5. 在 GitHub 上创建 Pull Request，合入 main
```

### 11.3 Commit 信息规范

采用 **`<type>: <简短描述>`** 格式，中文或英文均可，保持一致性即可。

| type | 用途 | 示例 |
|------|------|------|
| `feat` | 新功能 | `feat: 增加干跑模式 dry_run 参数` |
| `fix` | 问题修复 | `fix: 修复 odom TF 双重发布冲突` |
| `refactor` | 代码重构 | `refactor: 将导航超时抽取到 strategy.yaml` |
| `docs` | 文档更新 | `docs: 补充 Git 协作指南` |
| `chore` | 杂项（配置、构建等） | `chore: 统一 TCP 默认端口为 9999` |

**原则**：
- 一个 commit 只做一件事，便于事后 `git bisect` 定位问题。
- 不要提交调试用的临时文件（`*.pyc`、`__pycache__/`、`.vscode/` 等），已通过 `.gitignore` 排除。
- **不要提交凭据或 IP 地址等敏感信息**。

### 11.4 .gitignore 补充

以下模式已在仓库根目录 `.gitignore` 中配置，如需追加：

```gitignore
# Python
__pycache__/
*.py[cod]
*.egg-info/

# ROS
build/
devel/
logs/

# IDE
.vscode/
.idea/

# 音频和模板（较大，通过其他方式同步）
*.wav
templates/
```

### 11.5 小车 ↔ 开发机同步

比赛现场网络环境不确定，建议用 U 盘或 `scp` 同步：

```bash
# 从开发机推到小车（在开发机上执行）
scp -r ~/Smart-Pharmacy/pharmacy_mplus0/ EPRobot@<小车IP>:~/robot_ws/src/pharmacy_mplus0/

# 从小车拉回开发机（在开发机上执行）
scp -r EPRobot@<小车IP>:~/robot_ws/src/pharmacy_mplus0/ ~/Smart-Pharmacy/pharmacy_mplus0/
```

> `scp` 之前建议在源端先 `git add -A && git commit` 保存当前状态，避免覆盖未保存的工作。

### 11.6 回滚错误修改

```bash
# 查看最近提交记录
git log --oneline -10

# 回滚某个文件到上次提交的状态
git checkout -- <文件路径>

# 回滚到某个历史提交（保留工作区修改）
git revert <commit-hash>

# 查看两个提交之间的差异
git diff main..dev -- pharmacy_mplus0/
```

### 11.7 赛前检查清单

每次在小车上部署前执行：

```bash
cd ~/robot_ws/src/Smart-Pharmacy

# 1. 确认在正确分支
git branch
# 应显示 * main

# 2. 确认无未提交修改
git status

# 3. 查看最近提交，确认版本
git log --oneline -5

# 4. 确保 Python 脚本有执行权限（Windows 下提交后权限可能丢失）
chmod +x pharmacy_mplus0/scripts/*.py
chmod +x pharmacy_mplus0_debug/scripts/*.py
```
