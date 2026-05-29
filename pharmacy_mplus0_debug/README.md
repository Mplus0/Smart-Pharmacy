# pharmacy_mplus0_debug

智慧药房调试工具包 —— 提供阶段性调试脚本，分模块验证视觉、导航、TCP 上报、语音播报和主控状态机。

> **本包不参与正式比赛主流程。`pharmacy_mplus0` 的任何 launch 文件不会 include 或依赖本包。**

---

## 1. 功能简介

`pharmacy_mplus0_debug` 为 `pharmacy_mplus0` 的每个子系统提供**独立的调试入口**：

- **识别板一逐步骤可视化**：显示 Canny 边缘、轮廓筛选、定位框、透视矫正的中间结果，用于现场调参。
- **识别板二模板采集**：从视频流截取帧保存为 `idle.png` / `wait5.png` 等模板。
- **假数据发布器**：模拟 `/cam_return`、`/cv1_result`、`/current_task` 等话题，用于不接摄像头时调试后续流程。
- **单航点导航测试**：从 `waypoints.yaml` 读取一个航点，发一次 `move_base` 请求，验证停靠精度。
- **终端仪表盘**：订阅所有关键话题，清屏刷新，替代多个 `rostopic echo` 窗口。
- **TCP 假服务端**：在本地监听，接收并格式化打印 TCP 上报 JSON，无需裁判软件即可验证上报。
- **主控干跑**：用假识别数据 + 静默语音启动主控，验证状态机流转。

---

## 2. 功能包目录结构

```
pharmacy_mplus0_debug/
├── CMakeLists.txt                # catkin 构建文件
├── package.xml                   # ROS 包依赖（含 pharmacy_mplus0 依赖）
├── README.md                     # 本文件
├── launch/
│   ├── test_board1.launch        # 启动识别板一调试器
│   ├── test_board2.launch        # 启动识别板二检测节点
│   ├── test_navigation.launch    # 启动单航点导航测试
│   ├── test_reporter.launch      # 启动 TCP 假服务端 + 上报节点
│   ├── test_voice.launch         # 启动语音播报测试
│   └── test_full_dryrun.launch   # 启动主控干跑（无摄像头/导航）
└── scripts/
    ├── board1_initial_debugger.py    # 识别板一逐步骤可视化
    ├── board2_template_capture.py    # 识别板二模板采集
    ├── send_fake_cam_return.py       # 模拟 /cam_return
    ├── send_fake_cv1.py             # 模拟 /cv1_result
    ├── send_fake_task.py            # 模拟全套裁判可见状态
    ├── waypoint_tester.py           # 单航点导航测试
    ├── topic_echo_dashboard.py      # 独立终端仪表盘
    └── tcp_fake_server.py           # 模拟裁判 TCP 服务端
```

---

## 3. 主要文件说明

### 3.1 Launch 文件

| 文件路径 | 启动内容 | 前置条件 |
| --- | --- | --- |
| `launch/test_board1.launch` | `board1_initial_debugger.py` | 摄像头 + web_video_server 已运行 |
| `launch/test_board2.launch` | `board2_detector.py`（来自 pharmacy_mplus0） | 摄像头 + web_video_server 已运行 |
| `launch/test_navigation.launch` | `waypoint_tester.py`，参数 `target` 指定目标航点 | 底盘 + 导航 已启动 |
| `launch/test_reporter.launch` | `tcp_fake_server.py` + `tcp_reporter.py`（来自 pharmacy_mplus0） | roscore 已启动 |
| `launch/test_voice.launch` | `tcp_reporter.py` + `send_fake_task.py` 发送测试文本 | roscore 已启动 |
| `launch/test_full_dryrun.launch` | `tcp_reporter.py` + `main_controller.py`（均来自 pharmacy_mplus0） | roscore 已启动；需另外手动运行假识别脚本 |

### 3.2 调试脚本

| 文件路径 | 作用说明 | 何时用 |
| --- | --- | --- |
| `scripts/board1_initial_debugger.py` | 显示识别板一处理流水线的中间步骤窗口 + pyzbar 解码结果。**注意：该调试器仍在调用旧方案的中间步骤（Canny/轮廓/正方形/定位框/透视矫正），这些步骤仅用于可视化参考，实际 `detect()` 已改用直接 pyzbar 方案** | 理解识别板一检测过程时；后续需跟进更新为新流水线的可视化 |
| `scripts/board2_template_capture.py` | 从视频流截取当前帧，按 s 保存为模板（如 idle.png） | 现场重新制作识别板二模板时 |
| `scripts/send_fake_cam_return.py` | 以 1 Hz 持续发布旧格式 `/cam_return`，参数 `_code` 和 `_box` | 不接摄像头时调试主控 |
| `scripts/send_fake_cv1.py` | 以 1 Hz 持续发布 `/cv1_result`，参数 `_wait`(0~10) | 不接摄像头时调试主控等待/快速通过 |
| `scripts/send_fake_task.py` | 发布 `/current_task`、`/cv1_result`、`/cv2_result`、`/announce_request` | 单独测试 tcp_reporter 的 JSON 格式和频率 |
| `scripts/waypoint_tester.py` | 发送单次 move_base 导航请求到参数 `_target` 指定的航点 | 逐个验证 waypoints.yaml 中每个航点的停靠精度 |
| `scripts/topic_echo_dashboard.py` | 每秒清屏，打印 9 个关键话题的最新值和距上次更新的秒数 | 替代多个 rostopic echo，快速判断卡在哪个环节 |
| `scripts/tcp_fake_server.py` | 监听 TCP 端口，接收每行 JSON 并格式化打印序号/车牌/速度/坐标/任务 | 没有裁判软件时验证上报格式、频率和内容 |

---

## 4. 编译方法

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

**注意**：本包通过 `package.xml` 声明了 `pharmacy_mplus0` 为 `<exec_depend>`，编译时会检查 `pharmacy_mplus0` 是否存在。确保先编译 `pharmacy_mplus0`。

与 `pharmacy_mplus0` 一样，修改 Python 脚本和 launch 文件不需要重新编译。

---

## 5. 运行方法

### 5.1 调试识别板一

```bash
# 前置：基础系统已启动（摄像头+视频流可用）
roslaunch pharmacy_mplus0 base_camera_nav.launch start_base:=false start_navigation:=false
roslaunch pharmacy_mplus0_debug test_board1.launch
```

> **注意**：`board1_initial_debugger.py` 使用的 `Board1Decoder` 已更新为直接 pyzbar 方案。调试器窗口中的 Canny/轮廓/正方形/定位框/透视矫正步骤仍会显示（调用旧方案保留的辅助方法），但实际解码流程不再经过这些步骤。

识别板一的调参现在只需关注：
- 终端日志中 pyzbar 解码结果是否与二维码内容一致
- 画面偏转角度是否合适（`vision.yaml` 中 `board1.rotate_degrees`）
- 锁定稳定性（`board1.stable_frames`）

开启识别节点直接验证：

```bash
roslaunch pharmacy_mplus0 detectors.launch
# 另开终端查看结果
rostopic echo /board1_detections
```

### 5.2 调试识别板二

```bash
# 单独特启识别板二检测节点
roslaunch pharmacy_mplus0_debug test_board2.launch

# 另开终端查看结果
rostopic echo /cv1_result
```

如果模板匹配分数不够，用模板采集工具重新截取：

```bash
rosrun pharmacy_mplus0_debug board2_template_capture.py _name:=idle
# 将摄像头对准空闲状态的识别板，按 s 保存
# 重复获取 wait5 ~ wait10
```

### 5.3 调试单航点导航

```bash
# 前置：底盘+导航已启动
roslaunch pharmacy_mplus0_debug test_navigation.launch target:=exam_A
roslaunch pharmacy_mplus0_debug test_navigation.launch target:=lab_1
roslaunch pharmacy_mplus0_debug test_navigation.launch target:=board2
```

### 5.4 调试 TCP 上报

```bash
# 终端 1：启动假服务端 + 上报节点
roslaunch pharmacy_mplus0_debug test_reporter.launch

# 终端 2：发送假状态数据
rosrun pharmacy_mplus0_debug send_fake_task.py _task:=A _cv2:=AB-1

# 终端 3（可选）：查看仪表盘
rosrun pharmacy_mplus0_debug topic_echo_dashboard.py
```

假服务端会按行打印每条 JSON：
```
#1    | car=1  speed=0.000  odom=(0.000,0.000)  task=A  CV1=WAIT-0  CV2=AB-1
#2    | car=1  speed=0.000  odom=(0.000,0.000)  task=A  CV1=WAIT-0  CV2=AB-1
...
```

### 5.5 调试语音播报

```bash
roslaunch pharmacy_mplus0_debug test_voice.launch tts_method:=espeak

# 或使用 ROS 话题模式
roslaunch pharmacy_mplus0_debug test_voice.launch tts_method:=topic
```

### 5.6 主控干跑（无摄像头、无导航）

```bash
# 终端 1：启动主控 + TCP 上报
roslaunch pharmacy_mplus0_debug test_full_dryrun.launch

# 终端 2：在主控进入 AT_BOARD1 状态后，发送假识别结果
rosrun pharmacy_mplus0_debug send_fake_cam_return.py _code:=AB _box:=1

# 终端 3：在主控进入 AT_BOARD2 状态后，发送假 CV1 结果
rosrun pharmacy_mplus0_debug send_fake_cv1.py _wait:=0
```

> 干跑模式下因为没有真实 move_base 服务，导航 state 会一直卡在等待中。此时主要验证：假数据接收 → 任务规划 → 状态切换 → 播报文本 → TCP JSON 格式等逻辑。

---

## 6. 参数调试说明

### 6.1 test_navigation.launch 参数

| 参数名 | 默认值 | 作用 |
| --- | --- | --- |
| `target` | board1 | 目标航点名称：`start` / `board1` / `board2` / `exam_A`~`exam_C` / `lab_1`~`lab_4` |
| `clear` | true | 导航前是否清理代价地图 |

### 6.2 假数据脚本参数

| 脚本 | 参数 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `send_fake_cam_return.py` | `_code` | AB | 模拟的二维码内容（A/B/C/AB/AC/BC/ABC） |
| | `_box` | 0 | 方框索引 0-3 |
| `send_fake_cv1.py` | `_wait` | 0 | 等待秒数（0=空闲,5~10=忙碌） |
| `send_fake_task.py` | `_task` | R | 当前任务（A/B/C/1/2/3/4/R） |
| | `_cv1` | WAIT-0 | 识别板二结果 |
| | `_cv2` | (空) | 识别板一结果（如 AB-1） |
| | `_announce` | (空) | 播报文本 |
| `tcp_fake_server.py` | `_port` | 9999 | 监听端口 |
| `board2_template_capture.py` | `_name` | idle | 保存的模板名（idle / wait5~wait10） |

---

## 7. ROS 节点和话题关系

### 7.1 本包启动的节点

| 节点名 | 来自文件 | 主要功能 |
| --- | --- | --- |
| `/board1_debugger` | `scripts/board1_initial_debugger.py` | 逐步骤显示识别板一中间结果 |
| `/waypoint_tester` | `scripts/waypoint_tester.py` | 单次导航测试 |
| `/tcp_fake_server` | `scripts/tcp_fake_server.py` | 假裁判服务端 |
| `/send_fake_cam_return` | `scripts/send_fake_cam_return.py` | 模拟识别板一结果 |
| `/send_fake_cv1` | `scripts/send_fake_cv1.py` | 模拟识别板二结果 |
| `/send_fake_task` | `scripts/send_fake_task.py` | 模拟裁判状态 |
| `/topic_echo_dashboard` | `scripts/topic_echo_dashboard.py` | 终端仪表盘 |

### 7.2 假数据发布者的话题

| 脚本 | 发布的话题 |
| --- | --- |
| `send_fake_cam_return.py` | `/cam_return` (Int32MultiArray) |
| `send_fake_cv1.py` | `/cv1_result` (String) |
| `send_fake_task.py` | `/current_task`, `/cv1_result`, `/cv2_result`, `/announce_request` (均为 String) |

### 7.3 仪表盘订阅的话题

`topic_echo_dashboard.py` 订阅以下 9 个话题：
`/cam_return`, `/cv1_result`, `/cv2_result`, `/current_task`, `/current_qr_task`, `/announce_request`, `/board1_detections`, `/board2_status`, `/all_qrcodes`

---

## 8. 调试与排错方法

### 8.1 识别板一调参流程

新方案参数极少，调参步骤大幅简化：

1. 启动识别节点验证：
   ```bash
   roslaunch pharmacy_mplus0 detectors.launch
   rostopic echo /board1_detections
   ```
2. 如果解析结果为空或不正确 — 检查摄像头视频流是否清晰、二维码是否完整可见。
3. 画面有倾斜 — 调整 `vision.yaml` 中 `board1.rotate_degrees`（如 3、-3）。
4. 结果频繁跳变 — 增大 `board1.stable_frames`（如 3→5）。
5. 直接 pyzbar 解码对画面质量要求低于旧方案，通常不需要额外调参。

> 如需可视化调试，仍可启动 `board1_initial_debugger.py`，但其显示的是旧方案辅助方法的结果，不代表实际解码管线。

### 8.2 导航停靠精度测试

```bash
roslaunch pharmacy_mplus0_debug test_navigation.launch target:=exam_A
# 小车到达后，在 RViz 中观察 base_footprint 与目标点的距离
# 偏差过大时调整 waypoints.yaml 中对应航点的 x/y/yaw
```

### 8.3 TCP JSON 格式验证

假服务端每收到一条会打印一行。**检查要点**：
- `car=` 是否与期望的车号一致
- `task=` 是否随主控状态变化（R→A→R→1→R...）
- `CV1=` 是否在 WAIT-0 和 WAIT-5~10 之间正确切换
- `CV2=` 格式是否为 "AB-1" 等
- 频率是否约 2 Hz（每秒约 2 条）

---

## 9. 常见问题 FAQ

### Q1：board1_initial_debugger 窗口不显示

原因：OpenCV `imshow` 需要图形界面（X11 转发或本地显示器）。

解决：确保通过 SSH -X 连接到小车，或在本地运行。

### Q2：board2_template_capture 保存模板后识别板二仍不工作

原因：模板图片放入的目录不对。

解决：确认保存路径为 `pharmacy_mplus0/templates/board2/`，文件名严格为 `idle.png`、`wait5.png` ~ `wait10.png`。

### Q3：waypoint_tester 卡在"等待 move_base action server"

原因：导航未启动。

解决：
```bash
# 先启动导航
roslaunch pharmacy_mplus0 base_camera_nav.launch start_camera:=false start_video_server:=false
# 再试
roslaunch pharmacy_mplus0_debug test_navigation.launch target:=board1
```

### Q4：tcp_fake_server 启动报"Address already in use"

原因：端口已被占用（默认 9999，也可能是其他服务或残留进程）。

解决：
```bash
rosrun pharmacy_mplus0_debug tcp_fake_server.py _port:=9998
# 同时修改上报节点的端口
rosrun pharmacy_mplus0 tcp_reporter.py _server_port:=9998
```

### Q5：send_fake_*.py 发布后主控无反应

原因：
- 主控不在对应的状态（如 `send_fake_cam_return.py` 只在 `AT_BOARD1` 状态有效）。
- 话题名拼写不一致。

排查：
```bash
rostopic list | grep -E "cam_return|cv1_result|board1"
rostopic echo /current_task  # 确认主控当前状态
```

---

## 10. 二次开发说明

| 需求 | 应修改的文件 |
| --- | --- |
| 新增一个假数据发布器 | 参考 `send_fake_cam_return.py`，发布对应话题即可 |
| 新增一个调试 launch | 参考 `test_board1.launch`，放在 `launch/` 目录 |
| 注册新脚本 | 在 `CMakeLists.txt` 的 `catkin_install_python` 中添加 |
| 向仪表盘增加显示的话题 | 修改 `topic_echo_dashboard.py` 的 `_TOPICS` 字典 |

**原则**：
- 调试脚本应尽量独立，不需要 `pharmacy_mplus0` 的全部模块即可运行。
- 每个调试工具只做一件事，便于现场快速定位问题。
- 不要在调试脚本中写死话题名，从 `pharmacy_mplus0.constants` 引用。
