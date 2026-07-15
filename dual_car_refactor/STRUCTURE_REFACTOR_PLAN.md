# dual_car_refactor 结构规范化方案

## 1. 文档目的

本文档用于规划 `dual_car_refactor` 的后续结构整理，使其逐步具备与 `pharmacy_mplus0` 类似的清晰分层、Catkin 包规范和可维护性。

本方案只讨论目录、模块职责、配置组织、命名和验证方法，不在本阶段修改任何代码。

后续实施必须遵守以下原则：

1. 不改变比赛流程、任务选择规则、导航顺序、识别算法和双车协作时机。
2. 不改变现有 ROS 节点、话题、消息类型和消息内容。
3. 不改变裁判 TCP 与双车 TCP 的协议格式。
4. 先完成等价搬迁，再进行命名整理；不在同一步中加入功能优化。
5. 每个阶段都应能够独立验证和回退。

---

## 2. 当前目录状态

当前目录结构：

```text
dual_car_refactor/
├── README.md
├── README_board1_v2_ros_topic.md
├── README_启动文件说明.md
├── launch/
│   ├── base_camera_nav.launch
│   ├── pharmacy_main_no_referee.launch
│   └── referee_only.launch
└── scripts/
    ├── board1_selection.py
    ├── dual_car_config.py
    ├── F1_detect_code_v5.py
    ├── F1_yaofang_v5.py
    ├── referee_client_v3.py
    └── tcp_link_v2.py
```

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

- 订阅 `/odometry/filtered`；
- 订阅 `/referee_task`、`/referee_cv1`、`/referee_cv2`；
- 计算平面速度并获取二维位置；
- 按固定频率向裁判服务器发送 JSON；
- 断线后自动重连；
- 使用线程锁保护发送数据。

### 3.7 启动管理

三个 launch 文件分别负责：

- `base_camera_nav.launch`：导航、Astra 摄像头、web_video_server；
- `pharmacy_main_no_referee.launch`：基础系统、双车 TCP、视觉和导航状态机；
- `referee_only.launch`：单独启动裁判通信节点。

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

当前没有：

- 板一选择逻辑单元测试；
- 配置完整性检查；
- TCP 消息解析测试；
- 状态机状态转换测试；
- 视觉图片回放入口；
- 新旧结构输出一致性测试。

结构搬迁本身因此存在引入行为差异的风险。

---

## 5. 目标架构

建议将其整理为一个真正独立的 Catkin 包，包名暂定为 `dual_car_refactor`。目标结构参考 `pharmacy_mplus0` 的“launch + config + scripts + src + resources”分层：

```text
dual_car_refactor/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── STRUCTURE_REFACTOR_PLAN.md
│
├── config/
│   ├── cars.yaml                 # 两车身份、IP、端口、起点和首发策略
│   ├── topics.yaml               # ROS 话题与服务名
│   ├── navigation.yaml           # 航点、角度、超时和停留时间
│   ├── vision.yaml               # 板一、板二及图像输入参数
│   ├── referee.yaml              # 裁判服务器与发送参数
│   └── dual_tcp.yaml             # 双车 TCP 重试、校验和消息限制
│
├── launch/
│   ├── race_bringup.launch       # 可选的一键完整启动入口
│   ├── base_camera_nav.launch    # 基础系统
│   ├── main.launch               # 双车通信、视觉、状态机
│   ├── detectors.launch          # 仅视觉节点
│   ├── dual_car_link.launch      # 仅双车通信
│   └── referee.launch            # 仅裁判通信
│
├── scripts/
│   ├── vision_node.py            # 仅保留 ROS 节点入口
│   ├── main_controller.py        # 仅保留状态机节点入口
│   ├── dual_car_link_node.py     # 仅保留双车 TCP 节点入口
│   └── referee_reporter.py       # 仅保留裁判节点入口
│
├── src/dual_car_refactor/
│   ├── __init__.py
│   ├── constants.py              # 状态编号、固定映射等不应现场修改的常量
│   ├── config_loader.py          # YAML 加载、默认值和配置校验
│   ├── models.py                 # 板一共享、轮次完成等内部数据结构
│   ├── board1_selection.py       # 现有纯任务选择逻辑
│   ├── board1_decoder.py         # 从现有视觉脚本等价拆出的板一算法
│   ├── board2_matcher.py         # 从现有视觉脚本等价拆出的板二算法
│   ├── image_source.py           # ROS Image/CompressedImage/HTTP 最新帧管理
│   ├── detection_stability.py    # 连续帧稳定及单阶段只发布一次
│   ├── navigation_client.py      # move_base 与清代价地图封装
│   ├── competition_io.py         # task、CV1、CV2、nav_state 发布封装
│   ├── round_state_machine.py    # 现有 8～15 状态逻辑
│   ├── turn_coordinator.py       # 双车令牌和板一共享选择
│   ├── dual_tcp.py               # TCP 收发、校验、去重
│   ├── referee_client.py         # 裁判 JSON 和 TCP 重连
│   └── audio.py                  # 语音路径与播放封装
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

---

## 6. 模块职责边界

### 6.1 `scripts/` 只作为 ROS 入口

每个脚本只做以下工作：

1. 初始化 ROS 节点；
2. 加载配置；
3. 创建对应实现类；
4. 调用 `run()` 或 `rospy.spin()`；
5. 捕获退出异常。

算法、协议和状态机不继续堆放在入口脚本中。

### 6.2 `src/dual_car_refactor/` 保存可复用实现

实现层按职责拆分，但第一轮拆分只能移动现有函数和类，不得重写算法。

例如：

- 原 `decode_qr_text_light()` 原样移入 `board1_decoder.py`；
- 原 `match_one_template()` 原样移入 `board2_matcher.py`；
- 原 `move()` 原样移入 `navigation_client.py`；
- 原 `handle_round_done_obj()` 原样移入 `dual_tcp.py`；
- 原裁判 payload 字段和发送频率保持不变。

### 6.3 `config/` 只保存现场或部署参数

配置建议按变化原因拆分：

| 配置文件 | 内容 | 典型修改时机 |
|---|---|---|
| `cars.yaml` | 车号、对车 IP、端口、起点、首发车 | 更换车辆或网络 |
| `navigation.yaml` | 航点、角度、超时、停留时间 | 换地图或现场调参 |
| `vision.yaml` | 阈值、ROI、帧率、图像源 | 相机和光照变化 |
| `referee.yaml` | 裁判 IP、端口、频率 | 比赛网络变化 |
| `dual_tcp.yaml` | 重试、口令、IP 校验 | 双车网络调试 |
| `topics.yaml` | 话题和服务名 | ROS 系统集成变化 |

状态编号、窗口含义和协议字段等程序不变量应放入 `constants.py`，避免被当成现场参数随意修改。

### 6.4 `resources/` 保存包内资源

模板和音频应通过 `rospkg` 或 `$(find dual_car_refactor)` 定位，不再依赖用户名、工作空间路径和旧包名。

资源路径变化只能改变“如何找到同一个文件”，不能在结构整理阶段替换模板或语音内容。

---

## 7. 当前文件到目标模块的映射

| 当前文件 | 目标位置 | 处理方式 |
|---|---|---|
| `dual_car_config.py` | `constants.py`、`config/*.yaml`、`config_loader.py` | 按性质拆分，保留全部当前默认值 |
| `board1_selection.py` | `src/dual_car_refactor/board1_selection.py` | 第一阶段可原文件搬迁 |
| `F1_detect_code_v5.py` | `vision_node.py`、`image_source.py`、`board1_decoder.py`、`board2_matcher.py`、`detection_stability.py` | 按函数职责机械拆分，不改函数内容和调用顺序 |
| `F1_yaofang_v5.py` | `main_controller.py`、`round_state_machine.py`、`navigation_client.py`、`competition_io.py`、`turn_coordinator.py`、`audio.py` | 先封装依赖，再逐块搬迁 |
| `tcp_link_v2.py` | `dual_car_link_node.py`、`dual_tcp.py`、`models.py` | ROS 接口和 TCP 实现分开 |
| `referee_client_v3.py` | `referee_reporter.py`、`referee_client.py` | ROS 数据采集和 TCP 客户端分开 |
| `pharmacy_main_no_referee.launch` | `main.launch` | 节点与参数保持等价，去掉版本化名称 |
| `referee_only.launch` | `referee.launch` | 仅规范命名和包名 |
| 两份原始说明 | `docs/history/` 或合并进 README | 保留历史背景，不与当前入口说明混放 |

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
- 当前所有参数默认值。

建议把这些结果保存为测试夹具。该阶段不改目录。

验收条件：能够用固定输入重复得到相同输出。

### 阶段 1：补齐 Catkin 包骨架

新增：

- `package.xml`；
- `CMakeLists.txt`；
- `setup.py`；
- `src/dual_car_refactor/__init__.py`。

暂时不拆大文件，只让当前脚本能够以 `dual_car_refactor` 包运行。launch 中改用新包名，资源仍可临时保持原路径以降低一次变更范围。

验收条件：

- `catkin_make` 成功；
- `rospack find dual_car_refactor` 成功；
- 四个业务节点均可启动；
- ROS 节点、话题和消息与原版本一致。

### 阶段 2：规范配置与资源路径

先实现配置加载器，再把 `dual_car_config.py` 中的值等价迁移到 YAML。要求：

- 每个旧配置键都有明确的新位置；
- 新默认值与旧值逐项一致；
- `CAR_ID` 的行为保持一致；
- 模板与音频改为包相对路径；
- 仍允许 launch 或 ROS 参数覆盖必要字段。

当前存在风险的默认值，例如 `force_label_for_debug="free"`、识别等待超时为 `0`、双车固定开启，不应在本阶段顺手修改。它们属于行为调整，应另开变更任务。

验收条件：自动比较旧配置字典和新配置对象，确认关键值完全一致。

### 阶段 3：拆分纯逻辑模块

优先搬迁依赖最少的代码：

1. `board1_selection.py`；
2. 板一解码函数；
3. 板二模板匹配函数；
4. TCP JSON 解析与校验；
5. 裁判 payload 构造。

本阶段保持函数参数、返回值、异常处理和日志时机尽量不变。

验收条件：相同测试输入在拆分前后得到完全相同的 Python 值和 ROS 消息。

### 阶段 4：拆分外部接口封装

提取：

- ROS/HTTP 图像源；
- move_base 导航客户端；
- 裁判话题发布；
- 音频播放；
- 双车 TCP 收发；
- 裁判 TCP 发送。

状态机只调用清晰接口，不直接处理 socket、图像缓存或资源路径。

验收条件：节点数量、话题、TCP 端口、重试频率、导航超时和语音触发时机不变。

### 阶段 5：提取状态机与双车协调器

最后处理风险最高的 `F1_yaofang_v5.py`：

- 保留状态编号 8～15；
- 保留每个状态的进入和退出条件；
- 保留取样窗口顺序 C → A → B；
- 保留完成窗口标志；
- 保留释放对车的准确时机；
- 保留提前到达的 `peer_done` 缓存行为；
- 保留车 2使用板一共享结果的条件和回退路径。

可以把每个 `handle_*` 方法移动到状态机模块，但不在此阶段改成新的状态机框架或异步模型。

验收条件：完整一轮和双车两轮的状态序列与原代码一致。

### 阶段 6：规范 launch 与文档

建立清晰入口：

- `race_bringup.launch`：完整启动；
- `main.launch`：业务层；
- `detectors.launch`：视觉调试；
- `dual_car_link.launch`：双车通信调试；
- `referee.launch`：裁判通信；
- `base_camera_nav.launch`：基础系统。

README 应以新入口为准，历史说明移到 `docs/history/`。

验收条件：README 中的每条启动命令均与实际文件和参数一致。

### 阶段 7：清理旧入口

只有在新结构通过上车验证后，才处理旧文件：

- 保留一段兼容期；
- 旧入口可变为调用新模块的薄包装；
- 明确标注弃用；
- 确认部署脚本和比赛启动命令均已切换后再删除旧名文件。

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
- 不再存在指向 `pharmacy_pkg` 的非兼容硬编码路径。

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
2. 单独检查视觉节点；
3. 单独检查两车 TCP；
4. 单独检查裁判 TCP；
5. 车 1完成一轮；
6. 车 2接收令牌并完成一轮；
7. 检查车 2板一共享成功路径；
8. 主动断开共享网络，检查车 2本车识别回退路径；
9. 双车连续运行多轮，确认无重复令牌和旧结果复用。

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
补齐 Catkin 包骨架
  ↓
等价迁移配置和资源路径
  ↓
拆分板一/板二/TCP 等纯逻辑
  ↓
拆分图像、导航、音频和 ROS 接口
  ↓
提取状态机与双车协调器
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

- `dual_car_refactor` 可以作为独立 Catkin 包编译和启动；
- launch、脚本、Python 库、配置、资源和测试分层清晰；
- 正式入口不再带 `v2/v3/v5` 版本号；
- 不再依赖 `pharmacy_pkg` 的绝对路径；
- 所有当前功能均有明确归属模块；
- 所有现有 ROS 与 TCP 接口保持兼容；
- 新旧实现对同一输入产生相同输出；
- 两辆小车的轮流出发、板一共享、导航和裁判上报行为不变；
- README 能够独立指导部署、配置、启动和故障排查；
- 旧入口只保留兼容包装或已在验证后安全移除。

本方案的核心不是重新设计比赛逻辑，而是先把现有可运行逻辑放入稳定、清晰、可验证的工程结构中，为后续单独进行性能优化和功能修正打好基础。
