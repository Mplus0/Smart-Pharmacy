# pharmacy_mplus0

`pharmacy_mplus0` 是智慧药房双车协同比赛的 ROS1 主功能包，负责比赛任务状态机、视觉识别、导航调度、双车协同、裁判系统通信和语音提示。同一套代码通过车辆编号区分车 1 与车 2，并使用 ROS 话题和 TCP 通信完成两车之间的任务交接。

## 1. 系统组成

功能包由五个运行节点和三个共享 Python 模块组成。

| 组件 | 主要职责 |
|---|---|
| `main_controller.py` | 管理比赛状态机、目标导航、取送样流程、语音提示和双车任务令牌 |
| `vision_node.py` | 识别板一二维码信息和板二空闲/忙碌状态 |
| `dual_car_link.py` | 在 ROS 与双车 TCP 连接之间转发任务完成消息和板一完整识别结果 |
| `referee_reporter.py` | 采集任务状态、视觉结果、速度和位置，并向裁判服务器发送数据 |
| `waypoint_tester.py` | 读取统一航点配置并构造单目标导航任务 |
| `config.py` | 加载 YAML 配置、车辆参数、状态编号和包内资源路径 |
| `task_logic.py` | 处理板一结果标准化、任务选择和双车共享结果复用 |
| `board2_yolo.py` | 完成板二定位、图像对齐、ROI 处理、ONNX 分类和概率平滑 |

## 2. 总体工作流程

车 1 默认先执行任务，车 2 等待车 1 释放任务令牌。单车一轮任务的流程如下：

1. 前往识别板一并读取四个窗口的二维码信息。
2. 根据二维码组合确定取样窗口、样本数量和异常窗口。
3. 按 `C → A → B` 的固定顺序前往需要访问的取样窗口。
4. 前往识别板二，判断化验窗口是否空闲；忙碌时按照识别结果等待 5～10 秒。
5. 前往选定的化验窗口完成送样。
6. 返回本车起点，并通过双车通信释放下一轮任务令牌。

车 2 优先复用车 1 发布的板一完整结果，在排除车 1 已选择任务后重新选择本车任务；共享结果不可用时，车 2 使用本车视觉识别结果。

## 3. 状态机

主控使用固定编号描述比赛阶段。

| 状态 | 编号 | 含义 |
|---|---:|---|
| `WAIT_TURN` | 8 | 等待另一辆车完成当前轮次 |
| `GO_TO_BOARD1` | 9 | 前往识别板一 |
| `BOARD1_RECOGNIZING` | 10 | 等待板一视觉结果 |
| `GO_TO_PICKUP_WINDOWS` | 11 | 按顺序执行取样窗口任务 |
| `GO_TO_BOARD2` | 12 | 前往识别板二 |
| `BOARD2_RECOGNIZING` | 13 | 等待板二视觉结果 |
| `GO_TO_LAB_WINDOW` | 14 | 前往化验窗口并完成送样 |
| `GO_BACK_HOME` | 15 | 返回本车起点并完成任务交接 |

主控通过 `/nav_state` 发布当前状态。板一和板二视觉节点仅在对应识别状态处理图像，正式结果发布后停止该阶段的重复识别。

## 4. 视觉识别

### 4.1 识别板一

识别板一用于读取四个窗口中的二维码内容。二维码有效文本包括空字符串、`A`、`B`、`C`、`AB`、`AC`、`BC` 和 `ABC`。

视觉节点依次使用以下识别路径：

1. 查找四个大窗口框，分别完成透视变换和二维码解码。
2. 在满足配置条件时使用整图四二维码直接识别路径。
3. 使用旧透视定位路径作为兜底。

四个窗口的完整文本结果经过标准化后，根据包含的样本数量选择任务；分数相同时选择位置更靠前的窗口。非法二维码文本会记录为异常窗口。只有连续多帧得到完全一致的有效结果时，节点才发布正式识别消息。

板一输出包括：

- `/cam_return`：主控使用的任务选择结果；
- `/board1_all_text`：包含四窗口文本、选中窗口和消息内容的完整 JSON，用于双车共享。

### 4.2 识别板二

识别板二完全采用双模型 YOLO 分类策略，模型以 ONNX 格式随功能包保存，不依赖外部训练目录。

处理流程如下：

1. 根据灰度阈值查找识别板外围黑框。
2. 将黑框区域透视变换为粗定位图像。
3. 查找内部白色区域并进行第二次透视对齐，得到 `480 × 320` 的标准板面。
4. 从标准板面裁剪状态 ROI `[229, 91, 108, 59]` 和数字 ROI `[258, 150, 57, 52]`。
5. 使用白色背景将两个 ROI 补为正方形，并缩放到 `224 × 224`。
6. 通过 OpenCV DNN 分别运行状态模型和数字模型。
7. 对连续 5 个有效对齐帧的分类概率进行滑动平均，并发布最终结果。

状态模型类别顺序为 `busy、idle`，数字模型类别顺序为 `10、5、6、7、8、9`。状态和数字模型在每个有效帧中都会执行；当状态为 `idle` 时忽略数字分类结果。

板二结果映射为：

| 识别结果 | `/board2_return` |
|---|---|
| 空闲 | `[0, 0]` |
| 忙碌 5～10 秒 | `[1, wait_time]` |

黑框或内部白色区域定位失败时，概率缓存会被清空，节点继续等待新的有效画面。

## 5. 导航与航点

主控通过 `move_base` 发送导航目标。所有航点、两车起点和朝向索引统一保存在 `config/waypoints.yaml`，支持以下目标名称：

- 取样窗口：`A`、`B`、`C`；
- 化验窗口：`lab1`、`lab2`、`lab3`、`lab4`；
- 识别位置：`board1`、`board2`；
- 两车起点：`home_pose.1`、`home_pose.2`。

航点数据由平面坐标和朝向索引组成，朝向索引对应 `euler_angles` 中的角度。导航失败时主控调用代价地图清理服务并按照状态机逻辑重试；成功到达目标点后的清图行为由配置控制。

## 6. 双车协同

双车协同由任务令牌和板一完整结果两部分组成。

### 6.1 任务令牌

本车完成化验窗口任务并进入返程阶段时发布轮次完成消息。`dual_car_link.py` 将消息通过 TCP 重复发送给另一辆车，接收端完成来源、字段和序列号检查后发布对车完成话题。

提前收到的对车令牌会被缓存，只有本车处于等待状态或已经回到起点时才会启用。序列号用于避免网络重试造成同一轮任务被重复处理。

### 6.2 板一结果共享

车 1 的 `/board1_all_text` 经双车 TCP 发送给车 2，并在车 2 发布为 `/dual_car/peer_board1_all_text`。共享内容包含：

- 车辆编号和消息序列号；
- 四个窗口的完整二维码文本；
- 已选择的窗口及对应任务消息。

车 2 使用共享结果时会清除车 1 的选择标记，再根据剩余内容重新执行任务选择。

## 7. 裁判系统通信

`referee_reporter.py` 汇总以下信息并通过 TCP 向裁判服务器发送 JSON 数据：

- 车辆编号；
- 当前平面速度；
- `map` 坐标系中的车辆位置；
- 当前任务阶段；
- 板二识别结果 `CV1`；
- 板一识别结果 `CV2`。

速度来自 `/odometry/filtered`，位置优先来自 `map → base_footprint` 的 TF 变换，失败时回退到 `map → base_link`。TCP 连接支持断线重连。

只有当前持有任务令牌的车辆拥有裁判上报权。主控通过带 latch 的 `/referee_active` 发布该状态；失去上报权时裁判节点关闭现有连接，重新获得上报权时按配置重置任务与视觉字段。

## 8. 语音提示

比赛语音位于 `resources/audio/`，主控根据当前任务动态组合文件名并异步调用系统音频命令播放。

语音内容包括：

- 板二空闲或等待秒数提示；
- 取样窗口和样本类型提示；
- 化验窗口、项目类型和样本数量提示。

取样与送样语音的播放时机由状态机控制，不阻塞主控的 ROS 回调处理。

## 9. ROS 接口

### 9.1 主要业务话题

| 话题 | 类型 | 数据含义 |
|---|---|---|
| `/nav_state` | `std_msgs/Int32` | 当前比赛状态 8～15 |
| `/cam_return` | `std_msgs/Int32MultiArray` | `[C,A,B,count,selected_index,error_window]` |
| `/board2_return` | `std_msgs/Int32MultiArray` | `[state,wait_time]` |
| `/board1_all_text` | `std_msgs/String` | 本车板一完整 JSON |
| `/dual_car/round_done` | `std_msgs/Int32MultiArray` | `[car_id,seq]` |
| `/dual_car/peer_done` | `std_msgs/Int32MultiArray` | `[peer_id,seq]` |
| `/dual_car/peer_board1_all_text` | `std_msgs/String` | 对车板一完整 JSON |
| `/referee_task` | `std_msgs/String` | 当前裁判任务标识 |
| `/referee_cv1` | `std_msgs/String` | 板二结果 `WAIT-0` 或 `WAIT-5`～`WAIT-10` |
| `/referee_cv2` | `std_msgs/String` | 板一结果，例如 `AB-1` |
| `/referee_active` | `std_msgs/Bool` | 本车是否拥有裁判上报权 |

### 9.2 图像与导航接口

视觉节点默认订阅 `/camera/rgb/image_raw`，在配置允许时可回退到 HTTP 视频流。导航部分使用 `move_base` action、TF 坐标变换和代价地图清理服务。

## 10. 配置结构

| 配置文件 | 内容 |
|---|---|
| `config/strategy.yaml` | 状态机时序、车辆角色、化验窗口映射和语音映射 |
| `config/waypoints.yaml` | 航点、朝向索引和两车起点 |
| `config/vision.yaml` | 板一参数、板二 YOLO 参数、图像输入和相机地址 |
| `config/communication.yaml` | ROS 话题、裁判 TCP、双车 TCP 和两车网络参数 |

同一套程序通过 `CAR_ID` 选择车辆配置。车辆编号决定本车身份、对车身份、初始任务状态、相机地址、TCP 端口和板一共享策略。

## 11. 模型与资源

板二模型位于：

```text
models/board2/status_best.onnx
models/board2/number_best.onnx
```

语音资源位于：

```text
resources/audio/
```

模型与资源路径由 `rospkg` 根据当前 ROS 包位置解析，不依赖固定用户名或工作空间绝对路径。

## 12. 功能包目录

```text
pharmacy_mplus0/
├── config/                     # 比赛参数配置
├── docs/                       # 行为和接口说明
├── launch/                     # ROS 启动文件
├── models/board2/              # 板二 ONNX 模型
├── resources/audio/            # 比赛语音
├── scripts/                    # ROS 运行节点
├── src/pharmacy_mplus0/        # 共享 Python 模块
├── test/                       # 配置和纯逻辑测试
├── CMakeLists.txt              # Catkin 构建与安装规则
├── package.xml                 # 功能包元数据与依赖
└── setup.py                    # Python 包安装配置
```

## 13. 运行基础

功能包面向 ROS1 Melodic、Ubuntu 18.04 和 Python 2.7。主要使用以下组件：

- ROS：`rospy`、`actionlib`、`move_base_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`std_msgs`、`std_srvs`、`tf`、`cv_bridge`；
- 视觉：OpenCV、OpenCV DNN、NumPy、pyzbar/zbar；
- 配置与资源：PyYAML、rospkg；
- 外部 ROS 功能包：`robot_navigation`、`astra_camera`、`web_video_server`；
- 音频：SoX `play`。

板二 ONNX 推理需要 OpenCV 提供 `cv2.dnn.readNetFromONNX` 接口。
