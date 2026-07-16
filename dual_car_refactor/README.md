# dual_car_refactor

智慧药房双车主程序的轻量化重构代码。本目录针对小车运行 `pharmacy_mplus0` / `pharmacy_mplus0_debug` 时负担较大的问题，将比赛业务重新组织为“导航状态机、按状态工作的视觉节点、双车直连通信、裁判通信”四部分。

> **先说明目录性质：**本目录目前不是一个可独立编译、独立 `roslaunch` 的 Catkin 功能包，因为没有 `package.xml` 和 `CMakeLists.txt`。现有 launch 中的 `pkg` 也是 `pharmacy_pkg`，脚本内的资源绝对路径同样指向 `~/robot_ws/src/pharmacy_pkg/`。因此它更接近一套待部署到 `pharmacy_pkg` 的替换文件，而不是名为 `dual_car_refactor` 的 ROS 包。

---

## 1. 它与原有两个包的关系

| 目录 | 定位 | 实现特点 |
|---|---|---|
| `pharmacy_mplus0` | 自研正式比赛包 | 模块划分完整：两个视觉节点、主控、裁判上报/语音节点以及独立 Python 库；支持单车和可选双车模式 |
| `pharmacy_mplus0_debug` | 自研调试包 | 提供假识别数据、假裁判服务器、单航点导航、模板采集和终端仪表盘，不参与正式流程 |
| `dual_car_refactor` | 新导入的双车轻量化方案 | 基于较早的 `pharmacy_pkg` 风格代码重构；视觉合并为一个节点，双车固定轮流运行，车 2 可复用车 1 的板一完整识别结果 |

新方案没有导入 `pharmacy_mplus0` 的 Python 模块，也没有使用其 YAML 配置、话题命名和调试工具。两套主程序不能仅替换某一个脚本后混合运行；部署时应把本目录列出的配套文件作为一组理解和验证。

---

## 2. 为什么它对小车更轻

主要减负点位于 `F1_detect_code_v5.py`：

- 识别板一和识别板二合并到同一个视觉节点，不再常驻两个独立识别进程。
- 视觉节点订阅 `/nav_state`，只在状态 `10`（板一识别）和 `13`（板二识别）处理图像；导航和等待期间只休眠并维持视频流最新。
- 默认直接订阅 `/camera/rgb/image_raw`，回调只保留最新帧，避免 HTTP 缓冲造成连续处理旧帧；HTTP 仅作兜底。
- 主循环限制为 6 Hz，识别结果稳定后每个识别阶段只正式发布一次。
- 板一主路径先寻找四个大窗口框，再逐窗口做轻量二维码解码，默认关闭高开销的整图二维码兜底。
- 裁判 TCP 上报拆到单独节点和终端，不阻塞导航状态机，也避免高频日志淹没主业务日志。

需要注意：`pharmacy_main_no_referee.launch` 默认仍启动 `web_video_server`，因为配置允许 ROS 图像失效时回退 HTTP。如果已确认 ROS 图像话题稳定，并希望进一步减负，可以关闭 HTTP 服务和回退，见第 8 节。

---

## 3. 目录与文件说明

```text
dual_car_refactor/
├── README.md                         # 本说明
├── STRUCTURE_REFACTOR_PLAN.md        # 后续结构规范化方案
├── README_board1_v2_ros_topic.md     # 板一 ROS 图像输入的简要变更记录
├── README_waypoint_test.md           # 单航点测试详细说明
├── README_启动文件说明.md             # 原始启动说明
├── launch/
│   ├── base_camera_nav.launch        # 仅基础导航、摄像头和视频流
│   ├── pharmacy_main_no_referee.launch # 主业务，不含裁判通信
│   ├── referee_only.launch           # 仅裁判通信
│   └── test_move_to_waypoint.launch  # 单航点导航测试
└── scripts/
    ├── dual_car_config.py            # 两车共用的集中配置
    ├── board1_selection.py           # 板一任务选择纯逻辑
    ├── F1_detect_code_v5.py          # 合并后的板一/板二视觉节点
    ├── F1_yaofang_v5.py              # 导航与双车轮流状态机
    ├── tcp_link_v2.py                # 两车之间的 TCP 桥接节点
    ├── referee_client_v3.py          # 向裁判系统上报状态
    └── test_move_to_waypoint.py      # 独立航点测试节点
```

### 3.1 `dual_car_config.py`

所有现场参数集中在这里：

- `COMMON`：话题、状态编号、视觉、导航、航点、裁判服务器、双车 TCP、语音和化验窗口映射。
- `CARS[1]` / `CARS[2]`：首发顺序、本车起点、摄像头 URL、对车 IP 和端口，以及是否复用对车板一结果。
- `CAR_ID` 环境变量：让同一套代码区分 1 号车与 2 号车；launch 会自动设置。

### 3.2 `board1_selection.py`

把四个窗口的二维码文本转换为旧协议 `/cam_return`：

```text
[是否去 C, 是否去 A, 是否去 B, 样本数, 选中的窗口下标, 错误窗口编号]
```

选择规则是优先二维码组合中包含样本最多的一项，即 `ABC` 优先于双字母组合，双字母组合优先于单字母；同分时选择靠前窗口。

### 3.3 `F1_detect_code_v5.py`

合并处理两块识别板：

- 板一：四个大窗口框 → 单窗透视矫正 → pyzbar 轻量解码 → 连续 3 帧稳定 → `/cam_return`。
- 板一共享：同时把四窗完整文本和本车选择结果编码为 JSON，发布 `/board1_all_text`。
- 板二：灰度、模糊、Otsu 二值化后，对 `free.png`、`busy_5.png` 至 `busy_10.png` 做多尺度模板匹配 → `/board2_return`。
- 图像输入：默认 ROS `sensor_msgs/Image`，也支持压缩图像和 HTTP 视频流。

### 3.4 `F1_yaofang_v5.py`

负责 move_base 导航、取样/送样、语音和双车令牌：

```text
等待轮次(8)
  → 前往板一(9) → 板一识别(10)
  → 前往 C/A/B 取样(11)
  → 前往板二(12) → 板二识别(13)
  → 前往化验窗口(14)
  → 返回起点(15) → 等待下一轮
```

导航失败时保留已经完成的取样窗口标志，重试不会重复访问成功窗口。到达化验窗口并准备返回起点时，本车发布 `round_done`，允许对车开始下一轮。

当前代码中 `self.enable_dual_car = True` 是固定值，没有单车模式 launch 开关。

### 3.5 `tcp_link_v2.py`

把本车 ROS 消息通过两车直连 TCP 转发给对车：

- `round_done`：通知对车获得下一轮出发令牌。
- `board1_all_text`：共享车 1 看到的四个板一二维码。

发送端会在配置的持续时间内重复建立短连接发送；接收端按来源车号、序列号、可选口令和可选来源 IP 校验、去重，再发布到本车 ROS。

### 3.6 `referee_client_v3.py`

独立订阅里程计及任务话题，默认以 2 Hz 向裁判服务器发送一行一个 JSON：

```json
{"id":"1","speed":0.0,"odom":[0.0,0.0],"task":"R","CV1":"None","CV2":"None"}
```

断线后自动重连。当前版本把速度和位置拆开取值：

- `speed` 来自 `/odometry/filtered.twist` 的平面速度；
- `odom` 由定时器以 4 Hz 查询 TF `map → base_footprint`，失败时回退到 `map → base_link`；
- 成功使用某个底盘坐标系后会优先继续查询该坐标系；
- TF 暂时不可用时保留上一次成功位置，并以节流日志提示。

因此裁判 JSON 中的 `odom` 是地图坐标，而不是直接使用里程计消息中的 pose。

### 3.7 `test_move_to_waypoint.py`

用于脱离完整比赛状态机，单独验证集中配置里的导航点：

```text
A、B、C、lab1～lab4、board1、board2、home
```

节点从 `dual_car_config.py` 读取航点和朝向，支持：

- 按 `CAR_ID` 使用对应车辆的 `home_pose`；
- 等待 move_base action server；
- 出发前可选清除代价地图；
- 导航超时后取消目标并返回不同退出码；
- `dry_run` 只打印目标坐标和朝向，不发送导航目标。

配套文件为 `launch/test_move_to_waypoint.launch`，详细操作见 `README_waypoint_test.md`。

---

## 4. 双车协作逻辑

### 4.1 轮流出发

默认配置：

- 车 1：`start_active=True`，上电后先出发。
- 车 2：`start_active=False`，停在起点等待车 1 的 `done`。
- 一辆车到达化验窗口、完成停留和播报、进入返程状态时释放另一辆车。
- 若 `done` 在本车回到起点前到达，状态机会缓存它，回到起点后再出发。
- 序列号用于忽略重复或旧消息。

### 4.2 板一结果共享

车 1 默认自己识别四个窗口并通过 TCP 共享完整 `all_text`。车 2 默认 `use_peer_board1_result=True`：

1. 接收并缓存车 1 的板一结果；
2. 删除车 1 已选择的窗口；
3. 从剩余二维码中重新选择样本数最多的一项；
4. 若仍有有效任务，车 2 跳过板一导航点，直接去取样；
5. 数据缺失、过期、格式错误或没有剩余任务时，车 2 回退为本车前往板一识别。

该设计既避免两车选择同一个二维码窗口，也让车 2 在共享成功时少走一个导航点、少做一次视觉识别。

---

## 5. ROS 节点与话题

| 节点 | 脚本 | 主要订阅 | 主要发布 |
|---|---|---|---|
| `/detect_abc_*` | `F1_detect_code_v5.py` | `/nav_state`、摄像头图像 | `/cam_return`、`/board2_return`、`/board1_all_text` |
| `/nav_pharmacy` | `F1_yaofang_v5.py` | `/cam_return`、`/board2_return`、`/dual_car/peer_done`、`/dual_car/peer_board1_all_text` | `/nav_state`、`/referee_task`、`/referee_cv1`、`/referee_cv2`、`/dual_car/round_done` |
| `/dual_car_tcp_link_carN` | `tcp_link_v2.py` | `/dual_car/round_done`、`/board1_all_text` | `/dual_car/peer_done`、`/dual_car/peer_board1_all_text` |
| `/referee_client_node_*` | `referee_client_v3.py` | `/odometry/filtered`、`/tf`、三个 `/referee_*` 话题 | 无 ROS 输出；通过 TCP 上报 |
| `/test_move_to_waypoint` | `test_move_to_waypoint.py` | move_base action 状态 | 向 move_base 发送单个测试目标（`dry_run` 时不发送） |

重要消息格式：

| 话题 | 类型 | 数据 |
|---|---|---|
| `/nav_state` | `Int32` | 状态编号 8～15 |
| `/cam_return` | `Int32MultiArray` | `[C,A,B,count,selected_index,error_window]` |
| `/board2_return` | `Int32MultiArray` | 空闲 `[0,0]`；忙碌 `[1,5..10]` |
| `/board1_all_text` | `String` JSON | 本车四窗文本、选择项、车号和序列号 |
| `/dual_car/round_done` | `Int32MultiArray` | `[本车车号, seq]` |
| `/dual_car/peer_done` | `Int32MultiArray` | `[对车车号, seq]` |
| `/referee_task` | `String` | `R/A/B/C/1/2/3/4` |
| `/referee_cv1` | `String` | `WAIT-0` 或 `WAIT-5`～`WAIT-10` |
| `/referee_cv2` | `String` | 如 `AB-1`、`ABC-4` |

---

## 6. 部署前必须准备的资源与依赖

现有代码预期最终文件位于：

```text
~/robot_ws/src/pharmacy_pkg/
├── launch/       # 四个 launch 文件
├── scripts/      # 七个 Python 文件
├── pictures/board_2/
│   ├── free.png
│   └── busy_5.png ... busy_10.png
└── yuyingwenjian/ # WAIT、取样和送样 wav 文件
```

ROS/Python 运行依赖至少包括：

- ROS Melodic、`rospy`、`actionlib`、`move_base_msgs`、`geometry_msgs`、`std_msgs`、`sensor_msgs`、`nav_msgs`、`std_srvs`、`tf`；
- `robot_navigation`、`astra_camera`、`web_video_server`（仅 HTTP 模式或兜底需要）；
- Python OpenCV、NumPy、`cv_bridge`、`pyzbar` 及系统 `zbar`；
- 系统音频命令 `play`，通常由 SoX 提供。

脚本需要执行权限：

```bash
cd ~/robot_ws/src/pharmacy_pkg/scripts
chmod +x F1_detect_code_v5.py F1_yaofang_v5.py tcp_link_v2.py referee_client_v3.py test_move_to_waypoint.py
```

`dual_car_config.py` 和 `board1_selection.py` 必须与可执行脚本处于同一 Python 搜索目录。

---

## 7. 上车前必须修改或确认

### 7.1 必改配置

编辑 `scripts/dual_car_config.py`：

1. `CARS[1/2]["tcp"]["peer_ip"]`：分别填写对车真实 IP。
2. `CARS[1/2]["camera_url"]`：若保留 HTTP 兜底，填写每辆车自己的视频流地址。
3. `COMMON["referee"]["server_ip"]` 和 `server_port`：填写裁判服务器地址。
4. `COMMON["nav"]["waypoints"]` 与两车 `home_pose`：核对现场地图坐标。
5. `COMMON["paths"]`：核对模板和音频绝对路径。

### 7.2 当前代码中的重要风险

> **板二目前被强制判为空闲。**配置是：
>
> ```python
> COMMON["board2"]["force_label_for_debug"] = "free"
> ```
>
> 正式使用真实模板识别前必须改为 `None`，否则无论相机画面是什么都会发布 `[0, 0]` / `WAIT-0`。

还需确认：

- 两个板一/板二等待超时当前均为 `0`，代表无限等待；识别节点异常会使状态机永久停住。调试时建议先设为 20～60 秒。
- `shared_token` 当前为 `None`。可信隔离网络可用；若要防止误连接，两车必须设置相同非空字符串。
- `check_peer_ip=True`，现场 DHCP 导致 IP 变化时会拒收对车消息。
- `max_line_chars=2048`，若将板一 JSON 扩展得很大，需要同步提高限制。
- 双车模式硬编码开启；如果只跑一辆车，非首发车会一直等待，首发车完成一轮后也会等待不存在的对车。
- `robot_race_init.launch` 是否接受 `planner` 和 `open_rviz` 参数取决于本车的 `robot_navigation` 包；若报告 `unused args`，应核对并移除不被支持的传参。

---

## 8. 推荐启动方式

以下命令以文件已经部署到 `pharmacy_pkg` 为前提。

### 8.1 每辆车启动主业务

车 1：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1
```

车 2：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=2
```

主业务 launch 默认启动导航、摄像头、视频服务器、双车 TCP、视觉和导航状态机；视觉延时 3 秒、状态机延时 6 秒启动。

### 8.2 每辆车另开终端启动裁判通信

```bash
# 车 1
roslaunch pharmacy_pkg referee_only.launch car_id:=1

# 车 2
roslaunch pharmacy_pkg referee_only.launch car_id:=2
```

### 8.3 基础系统已经启动

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1 start_base:=false
```

### 8.4 只使用 ROS 图像并关闭 HTTP 开销

先在 `dual_car_config.py` 设置：

```python
COMMON["detect"]["image_source"] = "ros_topic"
COMMON["detect"]["camera_fallback_to_http"] = False
```

再启动：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch \
  car_id:=1 start_video_server:=false
```

### 8.5 分阶段检查基础系统

```bash
roslaunch pharmacy_pkg base_camera_nav.launch
```

该 launch 只启动导航、Astra 摄像头和 web_video_server，不启动智慧药房业务。

### 8.6 单独测试航点

航点测试时不要启动完整药房状态机。可以先只启动基础导航：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch \
  car_id:=1 \
  start_dual_tcp:=false \
  start_detect:=false \
  start_nav_pharmacy:=false \
  start_camera:=false \
  start_video_server:=false
```

第一次建议先 dry run：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch \
  car_id:=1 target:=A dry_run:=true
```

确认打印的坐标和朝向无误后再正式发送：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=A
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=board1
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab1
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=home
```

支持的目标为 `A`、`B`、`C`、`lab1`～`lab4`、`board1`、`board2` 和 `home`。完整安全检查、点位记录方法和急停命令见 `README_waypoint_test.md`。

---

## 9. 主业务 launch 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `car_id` | `1` | 车辆编号 1 或 2 |
| `start_base` | `true` | 是否启动基础系统分组 |
| `start_navigation` | `true` | 是否启动导航 |
| `start_camera` | `true` | 是否启动 Astra |
| `start_video_server` | `true` | 是否启动 HTTP 视频服务器 |
| `use_race_init` | `true` | 使用 `robot_race_init.launch`；false 使用 `robot_navigation.launch` |
| `start_dual_tcp` | `true` | 启动双车 TCP 桥 |
| `start_detect` | `true` | 启动合并视觉节点 |
| `start_nav_pharmacy` | `true` | 启动导航状态机 |
| `detect_start_delay` | `3` | 视觉延时启动秒数 |
| `nav_start_delay` | `6` | 状态机延时启动秒数 |
| `dual_publish_repeat` | `3` | 状态机向本地 TCP 节点发布 done 的次数 |
| `dual_publish_interval` | `0.10` | 本地重复发布间隔 |

`referee_only.launch` 另提供 `output` 和 `respawn_referee`；主业务节点也分别有 respawn 参数。导航状态机默认不自动重启，视觉和双车通信默认自动重启。

航点测试 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `car_id` | `1` | 决定使用哪辆车的 `home_pose` |
| `target` | `A` | 测试目标名称 |
| `timeout_sec` | `60` | 等待导航完成的最大时间 |
| `wait_server_sec` | `30` | 等待 move_base 服务的时间 |
| `clear_costmaps` | `true` | 发送目标前是否清除代价地图 |
| `dry_run` | `false` | 只显示目标，不让小车运动 |

---

## 10. 建议的上车验证顺序

1. **静态检查**：确认两车 `CAR_ID`、IP、端口、航点、资源路径和板二强制调试值。
2. **基础系统**：检查 `/odometry/filtered`、`/tf`、`/camera/rgb/image_raw`、move_base 和清代价地图服务。
3. **航点独测**：使用 `test_move_to_waypoint.launch` 先 dry run，再逐个验证 A/B/C、两块识别板、四个化验窗口和两车起点。
4. **视觉独测**：暂不启动导航状态机，手动发布 `/nav_state` 为 10 或 13，观察识别结果。
5. **双车通信独测**：两车只启动 `tcp_link_v2.py`，发布测试 `round_done`，确认对车收到 `peer_done`。
6. **裁判通信独测**：单独启动 `referee_only.launch`，确认速度来自里程计、坐标来自 `map` TF，并检查 JSON 中 id、任务和 CV 字段。
7. **低速全流程**：先验证车 1 一轮，再验证车 2收到令牌与共享板一结果，最后双车循环。

常用观察命令：

```bash
rostopic echo /nav_state
rostopic echo /cam_return
rostopic echo /board2_return
rostopic echo /board1_all_text
rostopic echo /dual_car/round_done
rostopic echo /dual_car/peer_done
rostopic echo /dual_car/peer_board1_all_text
rostopic echo /referee_task
rostopic echo /referee_cv1
rostopic echo /referee_cv2
```

手动测试对车放行：

```bash
rostopic pub /dual_car/round_done std_msgs/Int32MultiArray \
  "data: [1, 1]" -1
```

---

## 11. 已知边界

- 本目录尚未提供 Catkin 元数据和自动化测试，部署依赖已有 `pharmacy_pkg`；目前已有单航点测试工具，但还不是独立调试功能包。
- 配置集中在 Python 字典，修改后要重启相关节点；不像 `pharmacy_mplus0` 那样按视觉、导航、TCP 分为多个 YAML。
- 状态机在构造函数中直接进入长期循环，主要通过 ROS 日志现场诊断。
- 板一共享只配置为车 2使用车 1结果，不是双向对等任务分配。
- 板一的窗口框、透视和模板阈值依赖现场相机视角，需要用真实画面校准。
- 音频通过外部 `play` 进程启动，没有统一的排队或重叠控制。
- 裁判通信、双车通信是两个独立 TCP 通道；裁判服务器不可用不会替代或承担双车放行。

理解和修改本方案时，建议从 `dual_car_config.py` 开始，再按“`F1_detect_code_v5.py` 产生结果 → `F1_yaofang_v5.py` 执行业务 → `tcp_link_v2.py` 协调两车 → `referee_client_v3.py` 对外上报”的顺序阅读。
