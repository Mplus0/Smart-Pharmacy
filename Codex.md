# Codex 重构提示词：智慧药房 `pharmacy_mplus0`

请你作为代码重构助手，基于当前仓库中的 `Smart-Pharmacy.md` 理解比赛规则、现有策略和 ROS 代码结构，然后协助完成智慧药房比赛代码重构。当前阶段的目标是重构代码结构、提高可调试性和可维护性，不要求兼容旧代码接口，也不要求保留旧脚本命名，但需要保留当前比赛策略。

## 一、重构目标

主比赛功能包仍命名为 `pharmacy_mplus0`。请将现有主控、识别、导航、播报、TCP 上报、样本记录等逻辑拆分为职责清晰的模块。

核心比赛策略保持不变：

1. 小车循环执行配送任务，在每轮结束后回起点再进入下一轮。（为双车方案预留）
2. 先到识别板一识别二维码任务。
3. 优先选择样本数最多的二维码：`ABC` 优先，其次双窗口组合，最后单窗口组合。
4. 一个二维码对应一轮任务，一轮只配送一种样本类型。
5. 二维码所在方框决定化验窗口和样本类型，二维码字母决定体检窗口集合。
6. 体检窗口按固定最短顺序访问：`A`、`B`、`C`、`AB`、`AC`、`BC`、`ABC` 分别使用策略表。
7. 到达体检窗口和化验窗口时应明显停留，默认 1.5 秒。
8. 到识别板二后根据空闲/忙碌状态播报并执行对应动作。
9. 所有裁判可见状态统一上报：`task`、`CV1`、`CV2`、速度、位置、里程计。
10. TCP 连接失败不应阻塞主控。

## 二、建议生成的主功能包结构

目标目录结构建议如下：

```text
pharmacy_mplus0/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── waypoints.yaml
│   ├── strategy.yaml
│   ├── tcp.yaml
│   └── vision.yaml
├── launch/
│   ├── race_bringup.launch
│   ├── main.launch
│   ├── main_single.launch
│   ├── detectors.launch
│   ├── reporter.launch
│   └── base_camera_nav.launch
├── scripts/
│   ├── main_controller.py
│   ├── board1_detector.py
│   ├── board2_detector.py
│   ├── tcp_reporter.py
│   └── detection_monitor.py
├── src/
│   └── pharmacy_mplus0/
│       ├── __init__.py
│       ├── constants.py
│       ├── models.py
│       ├── task_planner.py
│       ├── navigation_client.py
│       ├── sample_store.py
│       ├── competition_io.py
│       ├── board1_decoder.py
│       ├── board2_matcher.py
│       ├── tcp_client.py
│       ├── voice.py
│       └── log_utils.py
└── templates/
    └── board2/
        ├── idle.png
        ├── wait5.png
        ├── wait6.png
        ├── wait7.png
        ├── wait8.png
        ├── wait9.png
        └── wait10.png
```

## 三、主功能包文件职责与实现思路

### `config/waypoints.yaml`

功能：保存所有比赛航点和窗口映射。

实现思路：

- 用 YAML 描述体检窗口、化验窗口、识别板、起点坐标。
- 每个航点包含 `x`、`y`、`yaw`。
- 提供逻辑名称，例如 `exam_A`、`lab_1`、`board1`、`board2`。
- 避免在主控代码中硬编码坐标。

建议内容：

```yaml
frames:
  map: map
  robot: base_footprint

waypoints:
  exam_A: {x: 0.685, y: 2.516, yaw: 1.5708}
  exam_B: {x: 1.463, y: 2.980, yaw: 1.5708}
  exam_C: {x: 1.530, y: 1.985, yaw: 1.5708}
  lab_1: {x: -1.639, y: 2.471, yaw: -1.5708}
  lab_2: {x: -0.846, y: 1.771, yaw: -1.5708}
  lab_3: {x: -1.669, y: 1.238, yaw: -1.5708}
  lab_4: {x: -0.885, y: 0.780, yaw: -1.5708}
  start: {x: 0.0, y: 0.0, yaw: 0.0}
  board1: {x: 0.861, y: 0.0, yaw: 0.0}
  board2: {x: -0.147, y: 3.813, yaw: 3.1416}
```

### `config/strategy.yaml`

功能：保存比赛策略参数。

实现思路：

- 保存体检窗口访问顺序表。
- 保存停留时间、识别超时、重试次数、导航超时等参数。
- 保存任务选择权重，默认按样本数最大化。

建议内容：

```yaml
dwell:
  exam_seconds: 1.5
  lab_seconds: 1.5
  board1_settle_seconds: 1.0

timeouts:
  board1_wait_seconds: 15.0
  board1_max_retry: 3
  board2_wait_seconds: 10.0
  nav_default_seconds: 40.0
  max_consecutive_fail_rounds: 5

visit_order:
  A: [A]
  B: [B]
  C: [C]
  AB: [A, B]
  AC: [C, A]
  BC: [C, B]
  ABC: [C, A, B]

task_priority:
  prefer_more_samples: true
```

### `config/tcp.yaml`

功能：保存 TCP 上报参数。

实现思路：

- 保存裁判电脑 IP、端口、小车编号、发送频率。
- 可由 launch 参数覆盖。

### `config/vision.yaml`

功能：保存视觉识别参数。

实现思路：

- 保存摄像头视频流 URL。
- 保存识别板一初始检测参数，例如旋转角度、Canny 阈值、轮廓面积范围、四边形近似系数、正方形长宽比阈值、定位框中心距离阈值。
- 保存识别板二模板匹配阈值、去抖帧数、发布频率。

### `src/pharmacy_mplus0/constants.py`

功能：定义比赛常量。

实现思路：

- 定义化验窗口名称、样本类型名称、化验窗口与样本类型映射。
- 定义 ROS 话题名，避免脚本中重复写字符串。
- 定义状态机状态枚举或字符串常量。

核心常量：

```python
LAB_WINDOW_NAMES = {
    "1": "血常规窗口",
    "2": "体液窗口",
    "3": "免疫检测窗口",
    "4": "激素检验窗口",
}

LAB_WINDOW_TO_SAMPLE_TYPE = {
    "1": "1",
    "2": "2",
    "3": "3",
    "4": "4",
}
```

### `src/pharmacy_mplus0/models.py`

功能：定义内部数据结构。

实现思路：

- 使用简单类或 `namedtuple` 表示二维码识别结果、任务计划、识别板二状态。
- 避免主控直接传递散乱的 list 或 dict。

建议模型：

- `Board1Detection`: `code`、`box_index`、`lab_window`、`sample_count`。
- `RoundPlan`: `code`、`lab_window`、`sample_type`、`exam_windows`。
- `Board2Status`: `wait_seconds`、`is_idle`。

### `src/pharmacy_mplus0/task_planner.py`

功能：将识别结果转换为本轮配送计划。

实现思路：

- 输入识别板一识别到的二维码列表。
- 根据样本数选择最优任务。
- 根据方框号计算化验窗口。
- 根据化验窗口计算样本类型。
- 根据二维码字母和访问顺序表计算体检窗口顺序。
- 后续可扩展双车避让：排除对方正在执行的方框或二维码任务。

关键接口建议：

```python
class TaskPlanner(object):
    def select_best(self, detections, excluded_box=None):
        pass

    def build_round_plan(self, detection):
        pass
```

### `src/pharmacy_mplus0/navigation_client.py`

功能：封装 `move_base` 导航。

实现思路：

- 初始化 `SimpleActionClient("move_base", MoveBaseAction)`。
- 从 `waypoints.yaml` 加载目标点。
- 提供 `go_to(name, timeout)` 方法。
- 内部构建 `MoveBaseGoal`。
- 每次导航结束后可调用 `clear_costmaps()`。
- 失败时返回 `False`，主控决定重试、跳过或结束本轮。

关键接口建议：

```python
class NavigationClient(object):
    def go_to(self, waypoint_name, timeout_sec=None):
        pass

    def clear_costmaps(self):
        pass

    def stop(self):
        pass
```

### `src/pharmacy_mplus0/sample_store.py`

功能：记录本轮携带的样本。

实现思路：

- 一轮任务只允许一种样本类型。
- 记录已经取过样的体检窗口列表。
- 到达化验窗口后根据目标窗口计算样本数量。
- 投递完成后清空。

关键接口建议：

```python
class SampleStore(object):
    def clear(self):
        pass

    def add_sample(self, exam_window, sample_type):
        pass

    def count_for_lab(self, lab_window):
        pass

    def deliver_to_lab(self, lab_window):
        pass
```

### `src/pharmacy_mplus0/competition_io.py`

功能：统一发布裁判可见状态和播报请求。

实现思路：

- 发布 `/current_task`、`/cv1_result`、`/cv2_result`、`/announce_request`、`/current_qr_task`。
- 提供语义化接口，例如 `arrive_exam("A")`、`arrive_lab("1")`、`publish_cv2("AB", 1)`。
- 不负责实际 TTS，只发布播报请求。

关键接口建议：

```python
class CompetitionIO(object):
    def set_task(self, task):
        pass

    def publish_cv1(self, wait_seconds):
        pass

    def publish_cv2(self, code, lab_window):
        pass

    def announce_exam_samples(self, windows):
        pass

    def announce_board2(self, wait_seconds):
        pass

    def announce_lab_arrival(self, lab_window, sample_count):
        pass
```

### `src/pharmacy_mplus0/board1_decoder.py`

功能：实现识别板一二维码解码算法。

实现思路：

- 参考 `pharmacy_pkg/scripts/detect_code_for_EPRobot.py` 的初始识别策略。
- 不使用固定 ROI 映射，也不实现 ROI 优先、多层融合策略。
- 对摄像头帧做必要旋转校正，默认沿用旧策略中的小角度旋转。
- 灰度化后使用 Canny 边缘检测。
- 通过轮廓面积筛选、四边形筛选、近似正方形筛选、嵌套定位框中心重合筛选，找到二维码定位框集合。
- 当定位框数量满足旧策略条件时，根据定位框极值点计算识别板四角。
- 使用 `cv2.getPerspectiveTransform` 和 `cv2.warpPerspective` 将识别板矫正为固定尺寸画面。
- 按固定四区域裁剪方式取出方框 1-4 的二维码区域。
- 对每个裁剪区域灰度化、二值化，再用 `pyzbar.decode` 解码。
- 将四个区域的二维码内容和方框编号转换为标准化的 `Board1Detection` 列表。

建议初始策略流程：

1. 轮廓筛选定位二维码定位框。
2. 根据定位框推算识别板透视变换。
3. 透视矫正到固定画布。
4. 四区域裁剪。
5. `pyzbar` 解码。
6. 输出识别结果。

暂时不要加入 ROI 策略。如果后续现场效果不好，再单独新增优化版本。

### `src/pharmacy_mplus0/board2_matcher.py`

功能：实现识别板二模板匹配。

实现思路：

- 加载 `templates/board2` 下的模板。
- 根据文件名解析 `WAIT-0`、`WAIT-5` 到 `WAIT-10`。
- 对视频帧进行灰度化、多尺度模板匹配。
- 用阈值和去抖队列锁定结果。
- 输出 `Board2Status`。

### `src/pharmacy_mplus0/tcp_client.py`

功能：封装 TCP 连接和发送。

实现思路：

- 后台线程自动连接裁判软件。
- 连接失败时定时重试。
- 发送失败时关闭 socket，等待重连。
- 每条 JSON 后加 `\n`，防止粘包。
- 不阻塞主控和上报循环。

### `src/pharmacy_mplus0/voice.py`

功能：封装 TTS 播报。

实现思路：

- 支持 `espeak`、`topic`、`silent` 三种模式。
- `espeak` 模式异步调用系统命令。
- `topic` 模式发布到指定语音模块话题。
- `silent` 模式仅记录日志，适合调试。

### `src/pharmacy_mplus0/log_utils.py`

功能：中文日志封装。

实现思路：

- 继续沿用当前 `log_utils.py` 思路，避免 Python2/ROS 中文编码问题。
- 提供 `loginfo`、`logwarn`、`logerr`、`logdebug`、throttle 版本。

### `scripts/main_controller.py`

功能：主控 ROS 节点入口。

实现思路：

- 只负责状态机调度。
- 初始化 `NavigationClient`、`CompetitionIO`、`SampleStore`、`TaskPlanner`。
- 订阅 `/cam_return` 或新的 `/board1_detections`。
- 订阅 `/cv1_result` 或新的 `/board2_status`。
- 根据状态调用各模块，不直接处理图像、不直接发送 TCP、不直接拼接 JSON。

状态机保持：

```text
INIT
GOTO_BOARD1
AT_BOARD1
GOTO_EXAM
AT_EXAM
GOTO_BOARD2
AT_BOARD2
PASS_BOARD2
GOTO_LAB
AT_LAB
DONE_ROUND
```

### `scripts/board1_detector.py`

功能：识别板一 ROS 节点入口。

实现思路：

- 从 `vision.yaml` 或 ROS 参数读取视频流 URL、旋转角度、轮廓筛选阈值、透视矫正尺寸、四区域裁剪参数。
- 调用 `Board1Decoder` 处理帧。
- 发布：
  - `/cam_return`：兼容当前主控格式时可保留。
  - `/all_qrcodes`：JSON，供调试和双车协作。
  - 可选 `/board1_detections`：更清晰的新 JSON 格式。
- 接收 `/reset_detection`，清除锁定结果。
- 识别结果连续稳定后锁定并持续发布。

### `scripts/board2_detector.py`

功能：识别板二 ROS 节点入口。

实现思路：

- 从参数读取视频流 URL、模板目录、匹配阈值、去抖帧数。
- 调用 `Board2Matcher`。
- 发布 `/cv1_result`。
- 接收 `/reset_detection`，清除锁定结果。

### `scripts/tcp_reporter.py`

功能：TCP 上报与语音播报节点入口。

实现思路：

- 使用 `TcpClient` 管理连接。
- 使用 `VoiceAnnouncer` 处理播报。
- 订阅 `/odom`、`/current_task`、`/cv1_result`、`/cv2_result`、`/announce_request`。
- 查询 TF 获取 `map -> base_footprint`。
- 按频率发送 JSON。

### `scripts/detection_monitor.py`

功能：调试监视器。

实现思路：

- 订阅 `/cam_return`、`/all_qrcodes`、`/cv1_result`、`/cv2_result`、`/current_task`、`/current_qr_task`、`/announce_request`。
- 在终端显示当前识别、任务、播报和上报状态。
- 用于比赛现场快速定位问题。

### `launch/main.launch`

功能：比赛业务层启动文件。

实现思路：

- 只启动 `pharmacy_mplus0` 自身的比赛业务节点。
- 启动识别板一节点。
- 启动识别板二节点。
- 启动 TCP 上报节点。
- 启动主控节点。
- 提供参数覆盖：`stream_url`、`server_ip`、`server_port`、`car_id`、`tts_method`。
- 不负责启动底盘、导航、摄像头和 `web_video_server`。
- 适合在基础系统已经启动后使用，也适合调试时反复重启业务逻辑。

### `launch/race_bringup.launch`

功能：正式比赛一键全量启动文件。

实现思路：

- 负责启动完整比赛运行环境。
- include 底盘启动 launch，例如 `eprobot_start/EPRobot_start.launch`。
- include 导航启动 launch，例如实际仓库中的 `robot_navigation` 导航 launch。
- include 摄像头启动 launch，例如 `astra_camera/astra.launch`。
- 启动 `web_video_server`，为识别节点提供 HTTP 视频流。
- include `pharmacy_mplus0/main.launch`，启动识别、TCP 上报和主控。
- 所有外部 launch 名称建议通过 arg 配置，避免不同车上包名不同导致写死。
- 提供开关参数：`start_base`、`start_navigation`、`start_camera`、`start_video_server`、`start_pharmacy`，便于现场跳过已启动的基础节点。

建议参数：

```xml
<arg name="start_base" default="true"/>
<arg name="start_navigation" default="true"/>
<arg name="start_camera" default="true"/>
<arg name="start_video_server" default="true"/>
<arg name="start_pharmacy" default="true"/>
<arg name="server_ip" default="192.168.12.16"/>
<arg name="server_port" default="8888"/>
<arg name="car_id" default="1"/>
<arg name="tts_method" default="espeak"/>
```

注意：

- `race_bringup.launch` 是正式比赛入口。
- `main.launch` 是业务层入口。
- 不要把底盘、导航、摄像头启动逻辑直接塞进主控 Python 代码。

### `launch/base_camera_nav.launch`

功能：基础系统启动文件。

实现思路：

- 只启动底盘、导航、摄像头和 `web_video_server`。
- 不启动智慧药房业务节点。
- 适合赛前先启动基础系统并确认定位、导航、视频流正常。
- `race_bringup.launch` 可以 include 本文件，然后再 include `main.launch`。

### `launch/main_single.launch`

功能：单车静默业务层调试/比赛模式。

实现思路：

- 与 `main.launch` 基本一致。
- 默认 `tts_method=silent`，保留 TCP 上报但不发声。
- 同样不启动底盘、导航、摄像头和 `web_video_server`。

### `launch/detectors.launch`

功能：只启动两个识别节点。

实现思路：

- 用于验证摄像头和识别算法。
- 不启动主控，小车不会运动。

### `launch/reporter.launch`

功能：只启动 TCP 上报和语音播报节点。

实现思路：

- 用于单独调试裁判软件连接和播报后端。

## 四、建议新增独立调试功能包

可以单独构建一个调试包，建议命名为 `pharmacy_mplus0_debug`。该包只用于阶段性调试，不参与正式比赛主流程。

建议目录：

```text
pharmacy_mplus0_debug/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── test_board1.launch
│   ├── test_board2.launch
│   ├── test_navigation.launch
│   ├── test_reporter.launch
│   ├── test_voice.launch
│   └── test_full_dryrun.launch
└── scripts/
    ├── board1_initial_debugger.py
    ├── board2_template_capture.py
    ├── send_fake_cam_return.py
    ├── send_fake_cv1.py
    ├── send_fake_task.py
    ├── waypoint_tester.py
    ├── topic_echo_dashboard.py
    └── tcp_fake_server.py
```

### `pharmacy_mplus0_debug/scripts/board1_initial_debugger.py`

功能：识别板一初始策略调试工具。

实现思路：

- 读取摄像头视频流。
- 显示或保存关键中间结果：边缘图、面积筛选轮廓、四边形轮廓、正方形轮廓、嵌套定位框、透视矫正结果、四个裁剪区域。
- 打印定位框数量、透视四角、四区域解码结果。
- 支持通过 ROS 参数调整 Canny 阈值、面积范围、四边形近似系数、正方形长宽比阈值、中心距离阈值。
- 用于现场调试 `board1_decoder.py` 的初始透视矫正策略。

### `pharmacy_mplus0_debug/scripts/board2_template_capture.py`

功能：识别板二模板采集工具。

实现思路：

- 从视频流截取当前帧。
- 根据参数保存为 `idle.png`、`wait5.png` 等模板。
- 可选裁剪目标文字区域后保存。
- 用于现场重新生成模板。

### `pharmacy_mplus0_debug/scripts/send_fake_cam_return.py`

功能：模拟识别板一结果。

实现思路：

- 发布 `/cam_return`。
- 参数包含 `code` 和 `box_idx`。
- 例如 `code=AB`、`box_idx=0` 发布 `[0,1,1,2,0,0]`。
- 用于不打开摄像头时调试主控后续流程。

### `pharmacy_mplus0_debug/scripts/send_fake_cv1.py`

功能：模拟识别板二结果。

实现思路：

- 发布 `/cv1_result`。
- 参数 `wait_seconds` 支持 `0` 或 `5-10`。
- 用于调试主控等待、快速通过和播报逻辑。

### `pharmacy_mplus0_debug/scripts/send_fake_task.py`

功能：模拟裁判可见任务状态。

实现思路：

- 发布 `/current_task`、`/cv1_result`、`/cv2_result`、`/announce_request`。
- 用于单独测试 `tcp_reporter.py` 和裁判软件显示。

### `pharmacy_mplus0_debug/scripts/waypoint_tester.py`

功能：单航点导航测试工具。

实现思路：

- 从 `waypoints.yaml` 读取航点。
- 参数 `target` 指定目标，例如 `board1`、`exam_A`、`lab_1`。
- 调用 `move_base` 前往目标。
- 支持是否清理 costmap。
- 用于逐个调试停靠点。

### `pharmacy_mplus0_debug/scripts/topic_echo_dashboard.py`

功能：终端仪表盘。

实现思路：

- 订阅关键话题并清屏刷新显示。
- 显示二维码结果、识别板二状态、当前任务、播报文本、TCP JSON 关键字段。
- 比 `rostopic echo` 更适合现场观察。

### `pharmacy_mplus0_debug/scripts/tcp_fake_server.py`

功能：模拟裁判 TCP 服务端。

实现思路：

- 在本机指定端口监听 TCP。
- 接收每行 JSON 并格式化打印。
- 用于没有裁判软件时验证上报格式和频率。

### `pharmacy_mplus0_debug/launch/test_board1.launch`

功能：只启动识别板一初始策略调试与监视。

### `pharmacy_mplus0_debug/launch/test_board2.launch`

功能：只启动识别板二模板匹配。

### `pharmacy_mplus0_debug/launch/test_navigation.launch`

功能：启动单航点导航测试。

### `pharmacy_mplus0_debug/launch/test_reporter.launch`

功能：启动 TCP 假服务端和 `tcp_reporter.py`，验证上报。

### `pharmacy_mplus0_debug/launch/test_voice.launch`

功能：测试播报后端，向 `/announce_request` 发布固定文本。

### `pharmacy_mplus0_debug/launch/test_full_dryrun.launch`

功能：无摄像头干跑主控。

实现思路：

- 启动主控、TCP 上报、假识别发布器。
- 可选择不启动真实导航，或使用仿真/假导航接口。
- 用于验证状态机逻辑。

## 五、实施顺序建议

请按以下顺序从零新建功能包，避免把旧比赛包继续“修修补补”：

1. 先梳理厂家/基础源码提供的运行能力，不修改这些包，只确认可调用入口：
   - 底盘启动：如 `eprobot_start`、`base_control` 等。
   - 导航与定位：如 `robot_navigation`、`eprobot_start` 中的导航 launch、`move_base`、地图与参数文件。
   - 摄像头与视频流：如 `astra_camera` / `ros_astra_camera`、`web_video_server`。
   - 机器人模型、雷达、里程计、TF：如 `eprobot_description`、`lidar`、`lslidar`、`ydlidar`、`robot_pose_ekf`、`robot_localization` 等。
2. 新建主比赛包 `pharmacy_mplus0`，新建调试包 `pharmacy_mplus0_debug`。不要在 `pharmacy_v5`、`pharmacy_pkg`、`smart_pharmacy` 等旧比赛包中继续改代码。
3. 在 `pharmacy_mplus0` 中先创建包骨架：`package.xml`、`CMakeLists.txt`、`config/`、`launch/`、`scripts/`、`src/pharmacy_mplus0/`、`templates/board2/`。
4. 新建配置文件：`waypoints.yaml`、`strategy.yaml`、`tcp.yaml`、`vision.yaml`。配置文件从现场实测值和策略文档填写，不直接读取旧比赛包配置。
5. 编写纯逻辑模块：`models.py`、`constants.py`、`task_planner.py`、`sample_store.py`。先用普通 Python 假数据验证任务选择、样本记录和投递数量。
6. 编写 ROS 封装模块：`navigation_client.py`、`competition_io.py`。这些模块只对接厂家基础系统暴露的 ROS 接口，例如 `move_base`、`/cmd_vel`、`/odom`、TF、`/move_base/clear_costmaps`。
7. 编写识别板一解码核心：`board1_decoder.py`，采用本文档指定的初始策略：定位框轮廓筛选、透视矫正、四区域裁剪、`pyzbar` 解码。
8. 编写识别板一 ROS 节点：`scripts/board1_detector.py`。
9. 编写识别板二模板匹配核心：`board2_matcher.py`，再写 `scripts/board2_detector.py`。
10. 编写 TCP 与语音模块：`tcp_client.py`、`voice.py`，再写 `scripts/tcp_reporter.py`。
11. 编写主控入口：`scripts/main_controller.py`。此时再接入导航、识别、样本、上报、播报模块。
12. 编写 launch 文件：先完成 `main.launch`、`detectors.launch`、`reporter.launch`，再完成 `base_camera_nav.launch` 和 `race_bringup.launch`。
13. 编写 `pharmacy_mplus0_debug` 调试工具包，用假识别、假 CV1、假 TCP 服务端、单航点导航测试分阶段验证。
14. 验证顺序：纯逻辑假数据 -> 识别节点单测 -> TCP/语音单测 -> 单航点导航 -> 主控干跑 -> 连接厂家基础系统实车运行。
15. 最后整理 README、启动命令、现场参数修改说明。

## 六、验收标准

重构完成后应满足：

- `pharmacy_mplus0` 是新建功能包，不依赖、不导入、不 include `pharmacy_v5` 的任何代码或 launch。
- `pharmacy_mplus0_debug` 是新建调试包，不参与正式比赛主流程。
- 厂家/基础包只作为底盘、导航、摄像头、雷达、TF、地图等运行依赖使用，不在其中写入比赛业务代码。
- `roslaunch pharmacy_mplus0 race_bringup.launch` 能一键启动正式比赛完整运行环境。
- `roslaunch pharmacy_mplus0 base_camera_nav.launch` 能只启动底盘、导航、摄像头和视频流。
- `roslaunch pharmacy_mplus0 main.launch` 能在基础系统已启动时启动比赛业务主流程。
- `roslaunch pharmacy_mplus0 main_single.launch` 能在基础系统已启动时以静默语音模式运行。
- `roslaunch pharmacy_mplus0 detectors.launch` 能单独验证识别节点。
- `pharmacy_mplus0_debug` 能分别调试识别板一、识别板二、单航点导航、TCP 上报、播报。
- 主控代码中不再硬编码大段坐标、播报文本和 TCP JSON 细节。
- 任务规划逻辑可以通过假识别数据独立测试。
- TCP 连接失败不影响主控状态机运行。
- 识别失败、导航失败、识别板二超时等情况都有明确容错路径。
- 删除 `pharmacy_v5` 后，新的 `pharmacy_mplus0` 仍应能够编译和启动自身节点。(后续会删除pharmacy_v5，所有不要对pharmacy_v5功能包有任何依赖项，如果两个功能包之间存在兼容性问题可以不考虑，在运行时我会剔除掉pharmacy_v5文件夹)

## 七、重要约束

- 本次不是在 `pharmacy_v5` 上重构，也不是迁移旧包文件；本次是在厂家源代码基础上新建比赛功能包。
- 不要修改 `pharmacy_v5`，也不要从 `pharmacy_v5` 导入模块、复制 launch 作为正式实现，或让新包运行依赖 `pharmacy_v5`。
- 不要把 `pharmacy_pkg`、`smart_pharmacy` 等旧比赛包作为运行依赖；它们最多只能作为理解历史策略的只读参考。
- 不要为了兼容旧代码保留混乱结构。
- 不要把视觉识别、导航、播报、TCP 上报全部写进主控脚本。
- 不要让 TCP 连接阻塞主控。
- 不要让识别节点参与业务决策，识别节点只输出识别结果。
- 不要让主控直接拼接 TCP JSON。
- 不要在正式比赛主流程中依赖调试包。
- 所有现场可调参数尽量放入 YAML 或 launch 参数。
- 不要把比赛业务代码写进厂家基础包；厂家包只通过 ROS 话题、service、action、TF、launch include 被调用。
- 对厂家 launch 名称不确定时，先读取当前 `src` 中对应包的 launch 文件确认，再在 `race_bringup.launch` 中 include。

## 八、给后续 Codex 的起始任务

请先阅读：

1. `Smart-Pharmacy.md`
2. `Codex.md`
3. 当前 `src` 根目录下的厂家/基础功能包目录结构，重点看 `eprobot_start`、`robot_navigation`、`base_control`、`eprobot_description`、`ros_astra_camera` / `astra_camera`、雷达相关包、定位相关包。
4. 厂家/基础包中的 launch 文件，确认底盘、导航、摄像头、地图、`move_base`、TF、`/odom`、`/cmd_vel`、`/move_base/clear_costmaps` 的启动方式。
5. `pharmacy_pkg/scripts/detect_code_for_EPRobot.py`，只用于理解识别板一初始算法，不要直接把旧脚本作为新包实现。
6. `pharmacy_v5` 只可作为策略背景参考；不要把它作为本次重构的代码基础。

阅读完成后，不要立刻大规模写代码。请先输出：

1. 新建包 `pharmacy_mplus0` 与 `pharmacy_mplus0_debug` 的具体文件清单。
2. 将要 include 的厂家基础 launch 文件清单，以及不确定项。
3. 第一阶段要生成的文件和验证方式。

第一阶段建议只完成：

- 新建两个包的骨架。
- 新建 `config/` 配置文件草案。
- 新建 `constants.py`、`models.py`、`task_planner.py`、`sample_store.py`。
- 用假数据验证任务规划和样本记录。

第一阶段不要接入真实导航、摄像头或 TCP；等纯逻辑通过后，再进入 ROS 节点和实车接口重构。
