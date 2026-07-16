# pharmacy_Mplus0 独立功能包重构方案

## 1. 文档目的

本文档用于规划把当前暂存在 `dual_car_refactor` 中、仍依附 `pharmacy_pkg` 运行的代码，迁移为独立目录 `pharmacy_Mplus0`。目标结构参考改名前原方案的分层方式，但新目录不导入、不 include、也不依赖 `pharmacy_pkg` 或旧业务包的代码。

本方案只讨论目录、模块职责、配置组织、命名和验证方法，不在本阶段修改任何代码。

后续实施必须遵守以下原则：

1. 不改变比赛流程、任务选择规则、导航顺序、识别算法和双车协作时机。
2. 不改变现有 ROS 节点、话题、消息类型和消息内容。
3. 不改变裁判 TCP 与双车 TCP 的协议格式。
4. 先完成等价搬迁，再进行命名整理；不在同一步中加入功能优化。
5. 每个阶段都应能够独立验证和回退。

### 1.1 目标目录名与 ROS 包名

旧功能包会由用户另行改名，因此本方案不考虑与旧名称并存的问题。统一采用以下命名：

| 项目 | 确定值 | 说明 |
|---|---|---|
| 目标目录 | `pharmacy_Mplus0/` | 独立保存本方案全部内容 |
| `package.xml` 中的 ROS 包名 | `pharmacy_mplus0` | ROS Catkin 使用全小写包名 |
| Python 包名 | `pharmacy_mplus0` | 与 ROS 包名一致，便于 import |
| roslaunch 用法 | `roslaunch pharmacy_mplus0 ...` | launch 统一使用新包身份 |

后续所有 `package.xml`、`CMakeLists.txt`、`setup.py`、launch、Python import、资源定位和部署命令都按上述名称编写，不再引入临时的 `*_refactor` 包名。

---

## 2. 当前目录状态

当前目录结构：

```text
dual_car_refactor/
├── README.md
├── STRUCTURE_REFACTOR_PLAN.md
├── README_board1_v2_ros_topic.md
├── README_waypoint_test.md
├── README_启动文件说明.md
├── launch/
│   ├── base_camera_nav.launch
│   ├── pharmacy_main_no_referee.launch
│   ├── referee_only.launch
│   └── test_move_to_waypoint.launch
└── scripts/
    ├── board1_selection.py
    ├── dual_car_config.py
    ├── F1_detect_code_v5.py
    ├── F1_yaofang_v5.py
    ├── referee_client_v3.py
    ├── tcp_link_v2.py
    └── test_move_to_waypoint.py
```

目录中另外已有三份专项说明：板一 ROS 图像输入说明、启动文件说明和航点测试说明。它们记录了当前方案的使用方法，但与正式入口 README 仍存在内容重复。

它目前更像一组准备复制进 `pharmacy_pkg` 的替换文件，还不是一个完整的 ROS Catkin 功能包：

- 缺少 `package.xml`、`CMakeLists.txt` 和 `setup.py`；
- launch 中的 `pkg` 仍为 `pharmacy_pkg`；
- 模板与语音路径硬编码为 `~/robot_ws/src/pharmacy_pkg/...`；
- 可执行节点和可复用逻辑全部放在 `scripts/`；
- 文件名保留版本号和旧工程命名，无法直接体现职责；
- 配置集中在一个 Python 大字典中，代码、现场参数和路径配置耦合；
- 视觉、导航、双车通信和裁判通信之间的接口主要依赖约定，缺少统一的数据模型与接口说明。

---

## 3. 当前已有功能

### 3.1 集中配置与双车身份

`dual_car_config.py` 负责：

- 通过环境变量 `CAR_ID` 区分 1 号车与 2 号车；
- 保存 ROS 话题名和状态编号；
- 保存识别板一、识别板二参数；
- 保存导航航点、等待时间和 move_base 参数；
- 保存裁判服务器地址；
- 保存两车直连 TCP 的 IP、端口、超时、重试及安全参数；
- 保存每辆车的起点、首发状态和板一共享策略；
- 保存语音文件和化验窗口映射。

### 3.2 识别板一任务选择

`board1_selection.py` 是当前最接近纯逻辑模块的文件，负责：

- 规范化二维码文本；
- 识别非法二维码所在窗口；
- 按样本数量选择最优二维码组合；
- 生成旧格式 `/cam_return` 数组：

```text
[是否去C, 是否去A, 是否去B, 样本数, 选择窗口下标, 错误窗口编号]
```

### 3.3 合并视觉识别

`F1_detect_code_v5.py` 同时负责两块识别板：

- 订阅 `/nav_state`，只在识别状态处理图像；
- 默认读取 `/camera/rgb/image_raw` 的最新 ROS 图像；
- ROS 图像不可用时可回退 HTTP 视频流；
- 识别板一查找四个大窗口框并逐窗口解码二维码；
- 提供整图二维码和旧透视算法兜底；
- 连续多帧稳定后发布 `/cam_return`；
- 发布 `/board1_all_text`，供两车共享完整板一结果；
- 识别板二通过多尺度模板匹配判断 `free` 或 `busy_5`～`busy_10`；
- 发布 `/board2_return`；
- 识别结果发布后停止该阶段的重复处理。

### 3.4 导航与比赛状态机

`F1_yaofang_v5.py` 负责：

- 建立 move_base 客户端并发送导航目标；
- 清除代价地图和处理导航超时；
- 依次执行板一、取样窗口、板二、化验窗口、返回起点；
- 保存已经完成的取样窗口，避免导航重试时重复取样；
- 发布 `/nav_state` 控制视觉节点是否工作；
- 接收板一和板二结果；
- 生成并发布裁判所需的 task、CV1 和 CV2；
- 播放取样、等待和送样语音；
- 管理双车出发令牌；
- 在收到对车板一结果时，为车 2排除车 1已选择的任务；
- 在完成化验窗口任务、开始返程时释放对车。

当前状态流程：

```text
WAIT_TURN(8)
  → GO_TO_BOARD1(9)
  → BOARD1_RECOGNIZING(10)
  → GO_TO_PICKUP_WINDOWS(11)
  → GO_TO_BOARD2(12)
  → BOARD2_RECOGNIZING(13)
  → GO_TO_LAB_WINDOW(14)
  → GO_BACK_HOME(15)
  → WAIT_TURN(8)
```

### 3.5 双车直连 TCP

`tcp_link_v2.py` 负责：

- 订阅本车 `/dual_car/round_done`；
- 将完成轮次消息通过 TCP 重复发送给对车；
- 将 `/board1_all_text` 通过 TCP 共享给对车；
- 同时运行 TCP 服务端接收对车消息；
- 校验来源车号、来源 IP、序列号及可选共享口令；
- 对重复消息进行去重；
- 在本车 ROS 中发布 `/dual_car/peer_done`；
- 在本车 ROS 中发布 `/dual_car/peer_board1_all_text`。

### 3.6 裁判通信

`referee_client_v3.py` 负责：

- 订阅 `/odometry/filtered` 并仅从 twist 计算平面速度；
- 使用 `tf.TransformListener` 查询机器人地图坐标；
- 优先查询 `map → base_footprint`，失败时回退 `map → base_link`；
- 以 4 Hz 的 ROS Timer 更新裁判 payload 中的 `odom`；
- 订阅 `/referee_task`、`/referee_cv1`、`/referee_cv2`；
- 按固定频率向裁判服务器发送 JSON；
- 断线后自动重连；
- 使用线程锁保护发送数据。

### 3.7 单航点导航测试

`test_move_to_waypoint.py` 和对应 launch 负责：

- 从集中配置读取 A/B/C、板一、板二、lab1～lab4 航点；
- 按 `CAR_ID` 读取各自的 `home_pose`；
- 把 `(x, y, 朝向索引)` 转换为 map 坐标系下的导航目标；
- 可选清除 move_base 代价地图；
- 支持 move_base 服务等待和单次导航超时；
- 支持 `dry_run`，只检查目标值而不发送导航目标；
- 使用退出码区分未知目标、服务不可用、超时和导航失败。

它补足了现场逐点校准能力，但仍属于人工集成测试工具，不等于自动化单元测试。

### 3.8 启动管理

四个 launch 文件分别负责：

- `base_camera_nav.launch`：导航、Astra 摄像头、web_video_server；
- `pharmacy_main_no_referee.launch`：基础系统、双车 TCP、视觉和导航状态机；
- `referee_only.launch`：单独启动裁判通信节点；
- `test_move_to_waypoint.launch`：配置并启动单航点测试节点。

---

## 4. 当前结构上的主要问题

### 4.1 不是独立 Catkin 包

当前代码必须复制到 `pharmacy_pkg` 才能按现有 launch 运行。目录名、launch 中的包名和资源路径不一致，容易导致部署遗漏或启动错误。

### 4.2 ROS 节点入口与业务实现混合

例如 `F1_detect_code_v5.py` 同时包含：

- ROS 节点初始化；
- 图像输入管理；
- 板一算法；
- 板二算法；
- 稳定帧管理；
- 全局运行状态；
- 主循环。

`F1_yaofang_v5.py` 同时包含：

- ROS 接口；
- 状态机；
- move_base 封装；
- 双车令牌；
- 板一共享任务选择；
- 语音播放；
- 裁判话题发布。

这种结构可以运行，但后续定位问题、编写测试和复用逻辑较困难。

### 4.3 文件命名无法稳定表达职责

`F1_*_v5.py`、`*_v2.py`、`*_v3.py` 依赖版本号区分文件。继续迭代时容易出现多个版本并存，但无法知道哪个是正式入口。

### 4.4 配置组织过于集中

所有配置都位于 `dual_car_config.py`，包括：

- 现场经常修改的 IP 和航点；
- 视觉算法参数；
- 状态编号和话题名；
- 资源路径；
- 程序常量。

这些内容的修改频率和性质不同，放在同一字典中容易误改。

### 4.5 资源路径依赖绝对目录

音频和模板路径写死为 `pharmacy_pkg` 的绝对路径，不利于换工作空间、换用户或直接使用新包名。

### 4.6 缺少结构化验证

当前已经有单航点现场测试节点，但仍缺少自动化、可重复的结构化测试，例如：

- 板一选择逻辑单元测试；
- 配置完整性检查；
- TCP 消息解析测试；
- 状态机状态转换测试；
- 视觉图片回放入口；
- 新旧结构输出一致性测试。

结构搬迁本身因此存在引入行为差异的风险。

---

## 5. 目标架构

建议把 `dual_car_refactor` 作为迁移来源，最终内容落到独立目录 `pharmacy_Mplus0`。以下结构按 ROS 包名 `pharmacy_mplus0` 编写，并参考原方案的“launch + config + scripts + src + resources”分层：

```text
pharmacy_Mplus0/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── STRUCTURE_REFACTOR_PLAN.md
│
├── config/
│   ├── strategy.yaml             # 两车身份、状态机、停留和轮流策略
│   ├── waypoints.yaml            # 航点、朝向和两车起点
│   ├── vision.yaml               # 板一、板二及图像输入参数
│   └── communication.yaml        # ROS 话题、裁判通信和双车 TCP 参数
│
├── launch/
│   ├── race_bringup.launch       # 完整入口：基础系统 + 全部比赛业务
│   ├── base_camera_nav.launch    # 仅基础系统
│   ├── main.launch               # 全部业务节点，可用参数控制子节点
│   ├── vision.launch             # 仅视觉节点
│   └── test_navigation.launch    # 单航点导航测试
│
├── scripts/
│   ├── vision_node.py            # 完整视觉节点：图像输入、板一/板二识别、稳定发布
│   ├── main_controller.py        # 完整主控：状态机、导航、双车令牌、裁判话题、语音
│   ├── dual_car_link.py          # 完整双车 TCP 收发节点
│   ├── referee_reporter.py       # 完整裁判状态采集与 TCP 上报节点
│   └── waypoint_tester.py        # 完整单航点测试节点
│
├── src/pharmacy_mplus0/
│   ├── __init__.py
│   ├── config.py                 # YAML 加载、默认值、常量和配置校验
│   └── task_logic.py             # 板一选择及两车共享任务选择纯逻辑
│
├── resources/
│   ├── board2/                   # free、busy_5～busy_10 模板
│   └── audio/                    # 比赛语音
│
└── test/
    ├── test_board1_selection.py
    ├── test_config.py
    ├── test_dual_tcp_protocol.py
    ├── test_referee_payload.py
    └── fixtures/                 # 固定 JSON、消息及测试图片
```

目标结构并不要求一次性全部建立。应按第 8 节的阶段逐步迁移。

### 5.1 独立包边界

迁移完成后，`pharmacy_Mplus0` 自己拥有比赛业务所需的代码、配置、launch、模板和音频，不再从 `pharmacy_pkg` 查找或导入任何内容：

| 类型 | 迁移后的归属 |
|---|---|
| 视觉、状态机、双车 TCP、裁判 TCP、航点测试代码 | `pharmacy_Mplus0/scripts` 与 `src/pharmacy_mplus0` |
| 航点、车辆 IP、视觉和通信参数 | `pharmacy_Mplus0/config` |
| 板二模板 | `pharmacy_Mplus0/resources/board2` |
| 语音文件 | `pharmacy_Mplus0/resources/audio` |
| 业务 launch | `pharmacy_Mplus0/launch` |
| 使用与维护文档 | `pharmacy_Mplus0/README.md` 与 `docs/` |

导航、底盘、摄像头、move_base、TF 和 ROS 消息包仍是正常的外部系统依赖，可以通过 launch include 或 Catkin 依赖使用；“独立”指不再依赖另一个智慧药房业务包，而不是把所有 ROS 官方或硬件驱动包复制进新目录。

---

## 6. 模块职责边界

### 6.1 `scripts/` 按运行节点整合

为了避免过度拆分，每个 ROS 运行节点对应一个完整脚本。脚本内部可以用类和函数分区，但不再为每个小职责单独建立 Python 文件：

| 脚本 | 保留在同一文件中的内容 |
|---|---|
| `vision_node.py` | ROS/HTTP 图像输入、板一解码、板二模板匹配、稳定帧和结果发布 |
| `main_controller.py` | 状态 8～15、move_base、裁判话题、语音、双车令牌和对车板一结果使用 |
| `dual_car_link.py` | ROS 桥接、TCP 服务端、TCP 发送、消息校验、重试和去重 |
| `referee_reporter.py` | 里程计速度、TF 坐标、裁判 payload、TCP 连接和重连 |
| `waypoint_tester.py` | 航点读取、目标构造、清代价地图、dry-run 和结果退出码 |

这样仍然能按节点独立启动和排查，同时避免把当前代码拆成十几个相互跳转的小模块。

### 6.2 `src/pharmacy_mplus0/` 只保存真正共享的逻辑

实现库只保留两个跨节点复用且适合独立测试的模块：

- `config.py`：统一读取四份 YAML，提供当前 `COMMON`、`CARS`、状态编号和资源路径对应的数据；
- `task_logic.py`：承接现有 `board1_selection.py`，供视觉节点和主控使用对车板一结果时共同调用。

其他函数继续留在所属节点脚本中。例如 `decode_qr_text_light()` 与 `match_one_template()` 都留在 `vision_node.py`，`move()` 与语音方法都留在 `main_controller.py`，TCP 解析方法留在各自通信节点。整理时只调整文件名、配置来源和内部区域顺序，不重新设计调用关系。

### 6.3 `config/` 只保存现场或部署参数

配置只拆成四份，避免现场参数分散：

| 配置文件 | 内容 | 典型修改时机 |
|---|---|---|
| `strategy.yaml` | 车号、首发策略、状态机超时、停留和双车行为 | 比赛策略变化 |
| `waypoints.yaml` | A/B/C、板一、板二、化验窗口、朝向和两车起点 | 换地图或校准点位 |
| `vision.yaml` | 阈值、ROI、帧率、图像源 | 相机和光照变化 |
| `communication.yaml` | ROS 话题、两车 IP/端口、双车重试、裁判 IP/端口/频率 | 网络或 ROS 集成变化 |

状态编号、窗口含义和协议字段等程序不变量由 `config.py` 集中提供，不再单独创建 `constants.py`。

### 6.4 `resources/` 保存包内资源

模板和音频应通过 `rospkg` 或 `$(find pharmacy_mplus0)` 定位，不再依赖用户名、工作空间路径、`pharmacy_pkg` 或旧业务包。

资源路径变化只能改变“如何找到同一个文件”，不能在结构整理阶段替换模板或语音内容。

---

## 7. 当前文件到目标模块的映射

| 当前文件 | 目标位置 | 处理方式 |
|---|---|---|
| `dual_car_config.py` | `config/*.yaml`、`src/pharmacy_mplus0/config.py` | 配置值进入四份 YAML，加载和常量集中在一个 Python 模块 |
| `board1_selection.py` | `src/pharmacy_mplus0/task_logic.py` | 整体搬迁并保留现有函数接口 |
| `F1_detect_code_v5.py` | `scripts/vision_node.py` | 整体保留为一个视觉节点，只整理内部区域和配置读取方式 |
| `F1_yaofang_v5.py` | `scripts/main_controller.py` | 整体保留为一个主控节点，不拆分状态机、导航、语音和双车令牌 |
| `tcp_link_v2.py` | `scripts/dual_car_link.py` | 整体保留双车 ROS/TCP 桥接逻辑 |
| `referee_client_v3.py` | `scripts/referee_reporter.py` | 整体保留数据采集、TF 和裁判 TCP 逻辑 |
| `test_move_to_waypoint.py` | `scripts/waypoint_tester.py` | 整体保留测试节点和全部参数 |
| `base_camera_nav.launch` | `base_camera_nav.launch` | 保留基础系统独立启动能力 |
| `pharmacy_main_no_referee.launch` | `main.launch` | 统一承载视觉、主控、双车通信和裁判通信，并提供子节点开关 |
| `referee_only.launch` | 合并进 `main.launch` | 不再保留裁判通信专用 launch；需要时通过 main 参数只启裁判节点 |
| `test_move_to_waypoint.launch` | `test_navigation.launch` | 保留全部现有参数及 dry-run 行为 |
| 当前完整启动逻辑 | `race_bringup.launch` | 只负责 include `base_camera_nav.launch` 和 `main.launch` |
| 当前 main 中的视觉节点配置 | `vision.launch` | 提取为唯一的视觉专用调试入口 |
| 三份专项说明 | `docs/`、`docs/history/` 或合并进 README | 航点测试说明保留为当前使用文档；历史变更记录与正式入口说明分开 |

---

## 8. 分阶段实施方案

### 阶段 0：建立行为基线

在移动任何文件前记录当前行为：

- 四种板一输入对应的 `/cam_return` 输出；
- 板二 `free`、`busy_5`～`busy_10` 的输出；
- 状态 8～15 的转换顺序；
- 车 1和车 2的首发状态；
- `round_done` 与 `peer_done` 的消息格式；
- 板一共享 JSON 格式；
- 裁判 JSON 的键、类型和默认值；
- 裁判 `speed` 来自 `/odometry/filtered.twist`；
- 裁判 `odom` 来自 TF `map → base_footprint/base_link`，包括 4 Hz 更新和回退顺序；
- 单航点测试支持的目标、参数、退出码和 move_base 目标内容；
- 当前所有参数默认值。

建议把这些结果保存为测试夹具。该阶段不改目录。

验收条件：能够用固定输入重复得到相同输出。

### 阶段 1：补齐 Catkin 包骨架

新增：

- `package.xml`；
- `CMakeLists.txt`；
- `setup.py`；
- `src/pharmacy_mplus0/__init__.py`。

在 `pharmacy_Mplus0` 中建立包骨架，暂时不拆大文件。launch 的 `pkg` 改为 `pharmacy_mplus0`。如果资源尚未迁入，本阶段只能作为构建过渡态，不能作为最终部署版本；进入运行验证前必须完成阶段 2，彻底去除 `pharmacy_pkg` 路径。

验收条件：

- `catkin_make` 成功；
- `rospack find pharmacy_mplus0` 成功，并返回 `pharmacy_Mplus0` 的实际路径；
- 四个业务节点均可启动；
- ROS 节点、话题和消息与原版本一致。

### 阶段 2：规范配置与资源路径

先实现配置加载器，再把 `dual_car_config.py` 中的值等价迁移到 YAML。要求：

- 每个旧配置键都有明确的新位置；
- 新默认值与旧值逐项一致；
- `CAR_ID` 的行为保持一致；
- 将当前实际使用的板二模板和语音文件复制进 `pharmacy_Mplus0/resources`；
- 模板与音频改为 `pharmacy_mplus0` 包相对路径；
- 所有 `$(find pharmacy_pkg)`、`pkg="pharmacy_pkg"` 和 `/home/.../pharmacy_pkg/...` 引用均替换为新包；
- 新包不得 import `pharmacy_pkg` 或旧业务包的 Python 模块；
- 仍允许 launch 或 ROS 参数覆盖必要字段。

当前存在风险的默认值，例如 `force_label_for_debug="free"`、识别等待超时为 `0`、双车固定开启，不应在本阶段顺手修改。它们属于行为调整，应另开变更任务。

验收条件：自动比较旧配置字典和新配置对象，确认关键值完全一致。

### 阶段 3：建立两个共享模块

只提取真正跨节点使用的内容：

1. 将配置加载、默认值、状态编号和固定映射集中到 `config.py`；
2. 将 `board1_selection.py` 整体迁移为 `task_logic.py`。

板一/板二识别函数、状态机、导航、TCP 和裁判 payload 不继续拆分。本阶段保持函数参数、返回值、异常处理和日志时机不变。

验收条件：相同测试输入在拆分前后得到完全相同的 Python 值和 ROS 消息。

### 阶段 4：按节点整理并改名

将五个当前可执行脚本按目标名称迁移：

- `F1_detect_code_v5.py` → `vision_node.py`；
- `F1_yaofang_v5.py` → `main_controller.py`；
- `tcp_link_v2.py` → `dual_car_link.py`；
- `referee_client_v3.py` → `referee_reporter.py`；
- `test_move_to_waypoint.py` → `waypoint_tester.py`。

每个文件仍然是一个完整节点。可以整理 import、注释、常量区域、类和主入口的排列，但不把图像源、导航、语音或 TCP 再拆成单独文件。

验收条件：节点数量、话题、TCP 端口、重试频率、导航超时和语音触发时机不变。

### 阶段 5：校验主控内部结构

最后重点校验风险最高的 `main_controller.py`，但不再提取新的状态机或双车协调器文件：

- 保留状态编号 8～15；
- 保留每个状态的进入和退出条件；
- 保留取样窗口顺序 C → A → B；
- 保留完成窗口标志；
- 保留释放对车的准确时机；
- 保留提前到达的 `peer_done` 缓存行为；
- 保留车 2使用板一共享结果的条件和回退路径。

`handle_*` 方法、move_base 调用、裁判话题发布、语音和双车令牌继续保留在同一个主控类中。只允许通过注释分区和方法排序提高可读性，不在此阶段改成新的状态机框架、异步模型或多层封装。

验收条件：完整一轮和双车两轮的状态序列与原代码一致。

### 阶段 6：规范 launch 与文档

只保留五个 launch 入口：

- `race_bringup.launch`：完整入口，依次 include 基础系统和 main；
- `base_camera_nav.launch`：只启动导航、底盘、摄像头和视频服务；
- `main.launch`：启动视觉、主控、双车通信和裁判通信，保留 `start_*` 参数按需关闭某个业务节点；
- `vision.launch`：只启动合并视觉节点，用于板一和板二调试；
- `test_navigation.launch`：单航点和 dry-run 调试。

不再建立 `dual_car_link.launch` 和 `referee.launch`。若需要单独调试双车通信或裁判通信，使用 `main.launch` 的节点开关，只开启目标节点，不增加新的 launch 文件。

README 应以新入口为准，历史说明移到 `docs/history/`。

验收条件：README 中的每条启动命令均与实际文件和参数一致。

### 阶段 7：清理旧入口和迁移来源

只有在 `pharmacy_Mplus0` 通过上车验证后，才处理旧入口和 `dual_car_refactor` 迁移来源：

- 保留一段兼容期；
- `dual_car_refactor` 中的旧入口可变为说明文件或调用新模块的薄包装；
- 明确标注弃用；
- 确认部署脚本和比赛启动命令均已切换到新 `pharmacy_mplus0` 后再删除旧名文件；
- 用户另行完成旧功能包改名，保证新 `pharmacy_mplus0` 是工作空间中的唯一同名 ROS 包。

验收条件：不存在同时维护两份算法实现的情况。

---

## 9. 兼容性约束

结构整理期间以下接口必须保持不变。

### 9.1 ROS 接口

| 接口 | 必须保持的内容 |
|---|---|
| `/nav_state` | `std_msgs/Int32`，状态编号 8～15 |
| `/cam_return` | `std_msgs/Int32MultiArray`，六个字段顺序不变 |
| `/board2_return` | `std_msgs/Int32MultiArray`，`[state, wait_time]` |
| `/board1_all_text` | `std_msgs/String`，JSON 字段不变 |
| `/dual_car/round_done` | `[car_id, seq]` |
| `/dual_car/peer_done` | `[peer_id, seq]` |
| `/dual_car/peer_board1_all_text` | 对车板一 JSON |
| `/referee_task` | `R/A/B/C/1/2/3/4` |
| `/referee_cv1` | `WAIT-0` 或 `WAIT-5`～`WAIT-10` |
| `/referee_cv2` | 如 `AB-1` |

### 9.2 双车 TCP 协议

以下 JSON 字段和含义不得变化：

```json
{"type":"round_done","car_id":1,"seq":1}
```

```json
{
  "type":"board1_all_text",
  "car_id":1,
  "seq":1,
  "all_text":["ABC","AB","A",""],
  "selected_index":0,
  "selected_text":"ABC",
  "selected_msg":[1,1,1,3,0,0]
}
```

共享口令启用时的 `token` 字段也必须保持兼容。

### 9.3 裁判 TCP 协议

键名、类型和换行分隔保持不变：

```json
{"id":"1","speed":0.0,"odom":[0.0,0.0],"task":"R","CV1":"None","CV2":"None"}
```

数据来源也属于兼容行为：`speed` 必须继续来自 `/odometry/filtered.twist`，`odom` 必须继续来自 TF 的 `map → base_footprint`，查询失败时按当前顺序回退 `base_link`，不能在纯结构整理中改回里程计 pose。

### 9.4 比赛行为

- 车 1先发，车 2等待；
- 本车完成化验窗口并进入返程时释放对车；
- 本车必须回到起点后才使用缓存令牌出发；
- 车 2优先复用车 1板一结果；
- 共享结果不可用时车 2自行前往板一；
- 取样顺序、播报时机、等待时长和导航超时不变。

---

## 10. 验证方案

### 10.1 静态检查

- Python 文件可通过语法检查；
- launch XML 可解析；
- `package.xml` 与代码 import 的 ROS 依赖一致；
- 所有安装的节点均在 `catkin_install_python` 中声明；
- 所有 YAML 均可加载并通过字段校验；
- `package.xml` 的包名为 `pharmacy_mplus0`；
- `rospack find pharmacy_mplus0` 唯一指向 `pharmacy_Mplus0`；
- 不再存在指向 `pharmacy_pkg` 或旧业务包的业务 import、launch `pkg` 或资源硬编码路径。

### 10.2 纯逻辑测试

重点测试 `board1_selection`：

| 输入 | 预期选择 |
|---|---|
| `["A","AB","ABC",""]` | `ABC` |
| `["AB","AC","B",""]` | 第一个双字母组合 `AB` |
| `["","","",""]` | 无结果 |
| 包含非法文本 | 正确记录错误窗口 |
| 对车已选窗口被置空 | 从剩余项重新选择 |

测试必须比较完整 `selected_msg`，不能只比较文本。

### 10.3 ROS 接口回归

通过固定消息依次触发状态，记录：

- 话题名；
- 消息类型；
- latch 行为；
- 重复发布次数；
- 发布时间顺序；
- 状态转换序列。

### 10.4 TCP 协议回归

- 同一个 seq 重复发送只触发一次；
- 旧 seq 被忽略；
- 错误车号被忽略；
- 错误 token 被忽略；
- 非对车 IP 在检查开启时被忽略；
- 超长行被关闭；
- 断线后继续重试；
- 裁判服务器重启后自动恢复发送。

### 10.5 视觉回归

使用同一批现场图片或录制视频，对比拆分前后：

- 四窗口排序；
- 每窗二维码文本；
- 最终 `/cam_return`；
- 板二最佳标签和得分；
- 稳定帧数量；
- 单阶段正式发布次数。

允许执行时间出现小范围波动，但识别路径和结果不能因结构搬迁发生变化。

### 10.6 小车全流程回归

按以下顺序进行：

1. 单独检查基础导航和摄像头；
2. 使用现有 `test_move_to_waypoint.py` 先 dry run，再逐点检查两车起点和全部比赛航点；
3. 单独检查视觉节点；
4. 单独检查两车 TCP；
5. 单独检查裁判 TCP，并验证 TF 坐标与 RViz/map 中位置一致；
6. 车 1完成一轮；
7. 车 2接收令牌并完成一轮；
8. 检查车 2板一共享成功路径；
9. 主动断开共享网络，检查车 2本车识别回退路径；
10. 双车连续运行多轮，确认无重复令牌和旧结果复用。

---

## 11. 不应与结构整理混在一起的改动

下列事项值得后续处理，但会改变现有行为，不属于本次结构规范化：

- 将板二 `force_label_for_debug` 默认值从 `"free"` 改为 `None`；
- 为板一、板二设置非零等待超时；
- 增加单车/双车模式开关；
- 改变车 1和车 2的任务分配策略；
- 调整航点、导航超时和停留时长；
- 更换二维码或模板匹配算法；
- 改变图像处理频率；
- 将同步状态机改为异步或多线程；
- 改变语音播放器或增加音频队列；
- 修改 TCP 重试周期、持续时间和消息格式；
- 改变裁判坐标来源；
- 调整日志频率和自动 respawn 策略。

这些内容应在结构重构完成、行为基线稳定后，分别建立独立改动和独立验证记录。

---

## 12. 推荐实施顺序总结

```text
记录当前行为
  ↓
在 pharmacy_Mplus0 中补齐 Catkin 包骨架
  ↓
等价迁移配置和资源路径
  ↓
建立 config.py 和 task_logic.py 两个共享模块
  ↓
将五个现有节点整体改名并整理内部区域
  ↓
校验主控内部结构和行为一致性
  ↓
规范 launch 和 README
  ↓
小车双车回归验证
  ↓
弃用旧入口
```

建议每完成一个阶段就单独提交一次，并在提交说明中记录：移动了哪些文件、哪些接口保持不变、执行了哪些验证。这样出现问题时可以准确定位，而不需要整体回退。

---

## 13. 完成标准

结构规范化完成时，应满足：

- 目标目录为 `pharmacy_Mplus0`，且自身包含完整 Catkin 元数据、业务代码、配置、资源、launch 和文档；
- ROS 包 `pharmacy_mplus0` 可以从 `pharmacy_Mplus0` 独立编译和启动；
- launch、脚本、Python 库、配置、资源和测试分层清晰；
- 正式运行代码保持“五个节点脚本 + 两个共享模块”的适度粒度，不为单个辅助职责额外拆文件；
- 正式入口不再带 `v2/v3/v5` 版本号；
- 不再依赖 `pharmacy_pkg` 或旧业务包的代码与资源；
- 新包可以从小车工作空间单独复制、编译和部署，不需要同时复制另一个智慧药房业务包；
- 所有当前功能均有明确归属模块；
- 所有现有 ROS 与 TCP 接口保持兼容；
- 新旧实现对同一输入产生相同输出；
- 两辆小车的轮流出发、板一共享、导航和裁判上报行为不变；
- README 能够独立指导部署、配置、启动和故障排查；
- 旧入口只保留兼容包装或已在验证后安全移除。

本方案的核心不是重新设计比赛逻辑，而是先把现有可运行逻辑放入稳定、清晰、可验证的工程结构中，为后续单独进行性能优化和功能修正打好基础。
