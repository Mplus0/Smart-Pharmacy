# 通信系统说明

本文档描述 `pharmacy_mplus0` 的两套通信系统：小车→裁判系统的 TCP 上报，以及两车之间的 ROS 话题通信。

---

## 1. 通信系统总览

```text
┌──────────────────────────────────────────────────────────────────┐
│                         裁判电脑                                   │
│                    TCP Server :8888                               │
│                  IP: 192.168.124.2                                │
│              接收两车 JSON 状态，用于评分                            │
└──────────┬──────────────────────────────────┬────────────────────┘
           │ TCP (JSON)                        │ TCP (JSON)
           │ 192.168.124.2:8888                │
    ┌──────┴──────────┐                 ┌──────┴──────────┐
    │    车 1          │                 │    车 2          │
    │  IP: 192.168.124.3│                │  IP: 192.168.124.9│
    │  tcp_reporter    │   ROS 话题       │  tcp_reporter    │
    │  main_controller │◄═══════════════►│  main_controller │
    │                  │ /dual_car_signal │                  │
    │                  │ /current_qr_task │                  │
    └──────────────────┘                 └──────────────────┘
```

**各设备 IP 一览**：

| 设备 | IP | 说明 |
|---|---|---|
| 裁判电脑 | `192.168.124.2` | TCP 服务端，监听端口 `8888` |
| 车一 | `192.168.124.3` | 摄像头视频流 + ROS 节点 |
| 车二 | `192.168.124.9` | 摄像头视频流 + ROS 节点 |

- **小车 → 裁判**：TCP 长连接，每车独立上报。上报失败不阻塞比赛流程。
- **车 1 ↔ 车 2**：ROS 话题，在同一个 ROS Master 下通过话题通信实现轮流出发和任务占用排除。

---

## 2. 小车 → 裁判系统（TCP 上报）

### 2.1 架构

```text
main_controller.py
  │
  ├── 发布 /current_task    ──┐
  ├── 发布 /cv2_result       ├── tcp_reporter.py
  ├── 发布 /announce_request ─┤      │
  │                           │  StateCollector 订阅以上话题
  │                           │  StateCollector 查询 TF: map→base_footprint
  │                           │  StateCollector 订阅 /odom 取速度
  │                           │      │
  │                           │  build_payload() → JSON dict
  │                           │      │
  │                           │  TcpClient.send() → 裁判电脑
  │                           │
  └── (board2_detector 发布 /cv1_result，由 tcp_reporter 订阅)
```

关键代码文件：

| 文件 | 作用 |
|---|---|
| [scripts/tcp_reporter.py](scripts/tcp_reporter.py) | TCP 上报 ROS 节点：采集状态 + 定时发送 + 语音播报 |
| [src/pharmacy_mplus0/tcp_client.py](src/pharmacy_mplus0/tcp_client.py) | TCP 异步客户端：后台线程连接、非阻塞发送、断线自动重连 |
| [src/pharmacy_mplus0/competition_io.py](src/pharmacy_mplus0/competition_io.py) | 发布 `/current_task`、`/cv2_result`、`/announce_request` 等话题 |
| [config/tcp.yaml](config/tcp.yaml) | TCP 上报配置文件 |

### 2.2 上报 JSON 格式

每 0.5 秒（2 Hz）发送一条，每条末尾带换行符：

```json
{
  "id": "1",
  "speed": 0.123,
  "odom": [1.234, -0.567],
  "task": "A",
  "CV1": "WAIT-0",
  "CV2": "AB-1"
}
```

| 字段 | 类型 | 值来源 | 含义 | 可能的值 |
|---|---|---|---|---|
| `id` | String | 参数 `~car_id` | 小车编号 | `"1"` / `"2"` |
| `speed` | Float | `/odom` 话题 `twist.twist.linear.x` | 当前线速度（m/s），保留 3 位小数 | 如 `0.123` |
| `odom` | [Float, Float] | TF 查询 `map → base_footprint` | 小车在全局地图中的 x, y 坐标（保留 3 位小数）。**注意：字段名叫 `odom`，实际是 map 系坐标** | 如 `[1.234, -0.567]` |
| `task` | String | `/current_task` 话题 | 当前任务/位置 | `"A"` `"B"` `"C"`（体检窗口）<br>`"1"` `"2"` `"3"` `"4"`（化验窗口）<br>`"R"`（路上/起点/识别区） |
| `CV1` | String | `/cv1_result` 话题 | 识别板二状态 | `"WAIT-0"`（空闲）<br>`"WAIT-5"` ~ `"WAIT-10"`（需等待的秒数） |
| `CV2` | String | `/cv2_result` 话题 | 识别板一本轮任务 | `"AB-1"`（二维码AB → 化验窗口1）<br>`"C-3"`（二维码C → 化验窗口3） |

### 2.3 坐标数据说明

**坐标来自 `map` 系，不是 `odom` 系**。虽然 JSON 字段名叫 `"odom"`（裁判系统约定的字段名），但实际取值逻辑在 `StateCollector._query_map_pose()`：

```python
trans = self._tf_buffer.lookup_transform(
    "map",              # ← 全局地图坐标系
    "base_footprint",   # ← 小车底盘投影坐标系
    rospy.Time(0),
    rospy.Duration(0.2),
)
```

其中 `"map"` 和 `"base_footprint"` 分别来自 [waypoints.yaml](config/waypoints.yaml) 的 `frames.map` 和 `frames.robot`。

TF 查询失败时**不会报错**，保持上次缓存坐标继续上报。

### 2.4 TCP 连接特性

| 特性 | 行为 |
|---|---|
| 连接时机 | 初始化时在后台线程中异步连接，不阻塞节点启动 |
| 连接失败 | 每 `reconnect_seconds` 秒重试一次，持续直到成功 |
| 发送失败 | 关闭当前 socket，静默丢弃本帧，不抛异常 |
| 对比赛的影响 | **不阻塞主控状态机**，TCP 异常完全不影响配送流程 |
| 多车支持 | 每车独立 TCP 连接，通过 `car_id` 区分上报来源 |

### 2.5 相关配置参数

[config/tcp.yaml](config/tcp.yaml)：

```yaml
server:
  ip: 192.168.124.2       # 裁判电脑 IP（可通过 launch 参数 server_ip 覆盖）
  port: 8888               # 裁判软件监听端口
  connect_timeout_seconds: 1.0
  reconnect_seconds: 2.0
report:
  car_id: "1"              # 小车编号（可通过 launch 参数 car_id 覆盖）
  hz: 2.0                  # 上报频率
  newline_delimited_json: true
```

Launch 参数（在 `main.launch` / `race_bringup.launch` / `reporter.launch` 中）：

| 参数 | 默认值 | 来源文件 |
|---|---|---|
| `server_ip` | `192.168.124.2` | 各 launch 的 `<arg>` 定义 |
| `server_port` | `8888` | 各 launch 的 `<arg>` 定义 |
| `car_id` | `"1"` | 各 launch 的 `<arg>` 定义 |
| `audio_dir` | `$(find pharmacy_mplus0)/audio` | launch `<arg>`，设 `""` 禁用语音 |
| `audio_player` | `aplay` | launch `<arg>`（aplay / paplay / ffplay） |

---

## 3. 车 1 ↔ 车 2（ROS 话题通信）

### 3.1 架构

两车通过 ROS 话题通信，不使用 TCP：

```text
车 1  main_controller                  车 2  main_controller
  │                                        │
  ├── 发布 /current_qr_task ──────────────►├── 订阅（解析对车占用的方框）
  ├── 订阅 /current_qr_task ◄──────────────├── 发布（广播自己的占用）
  │                                        │
  ├── 发布 /dual_car_signal ──────────────►├── 订阅（收到 ALLOW_START:2 后出发）
  ├── 订阅 /dual_car_signal ◄──────────────├── 发布（完成配送后放行对车）
```

关键代码文件：

| 文件 | 作用 |
|---|---|
| [scripts/main_controller.py](scripts/main_controller.py) | 双车核心调度：订阅/发布双车话题，控制出发和等待 |
| [src/pharmacy_mplus0/competition_io.py](src/pharmacy_mplus0/competition_io.py) | 提供 `publish_dual_signal()` 和 `publish_qr_task()` 接口 |
| [src/pharmacy_mplus0/task_planner.py](src/pharmacy_mplus0/task_planner.py) | `select_best(excluded_box=...)` 排除对车占用的方框 |
| [config/strategy.yaml](config/strategy.yaml) | 双车开关和首发车配置 |

### 3.2 双车话题

| 话题 | 类型 | 格式 | 作用 |
|---|---|---|---|
| `/dual_car_signal` | `std_msgs/String` | `ALLOW_START:1` 或 `ALLOW_START:2` | 轮流出发控制信号 |
| `/current_qr_task` | `std_msgs/String` | `CAR1:AB-1`（占用）<br>`CAR1:`（清空） | 任务占用广播，排除对车已选的二维码方框 |

### 3.3 `/dual_car_signal` — 轮流出发信号

**发布时机**：主控在 `DONE_ROUND` 状态开头，本轮配送完成后立即发送（不等本车回到起点）。

**接收处理**：

- 仅处理发给自己的信号（`target_id == self._car_id`）
- 仅在起点等待状态时（`_dual_waiting_at_start == True`）响应
- 配送途中收到的信号会被忽略
- 格式校验：非 `ALLOW_START:<id>` 格式的信号被忽略

**信号流**：

```text
车 1 完成配送 → 发布 ALLOW_START:2
  → 车 2 收到，_dual_start_allowed = True，开始出发
  → 车 2 完成配送 → 发布 ALLOW_START:1
    → 车 1 收到，_dual_start_allowed = True，开始出发
      → 循环
```

### 3.4 `/current_qr_task` — 任务占用广播

**发布时机**：

- 车选定识别板一二维码后，发布 `CAR<id>:<code>-<lab_window>`（如 `CAR1:AB-1`）
- 本轮配送完成后，发布 `CAR<id>:`（清空）

**接收处理**：

- 解析对车的消息，提取 `box_index` 作为 `_peer_occupied_box`
- 忽略自己的消息（`sender_id == self._car_id`）
- 忽略格式不匹配的消息（双车模式下无 `CAR` 前缀的消息被丢弃）
- 对车发布空占用时，`_peer_occupied_box` 清为 `None`

**在任务选择时的作用**（[task_planner.py](src/pharmacy_mplus0/task_planner.py)）：

```python
best = planner.select_best(detections, excluded_box=peer_occupied_box)
```

排除被对车占用的方框后，在剩余方框中按样本数最大化原则选择。

### 3.5 相关配置参数

[strategy.yaml](config/strategy.yaml)：

```yaml
dual_car_enabled: false              # 双车协作开关，默认关闭
dual_car_start_first_car_id: "1"     # 开局允许先出发的车号
dual_car_signal_topic: "/dual_car_signal"  # 信号话题名

# 远程任务共享（预留，默认关闭）
dual_car_remote_task_enabled: false  # 车1将第二个二维码分配给车2
dual_car_remote_task_max_age_sec: 45.0
```

Launch 参数（在 `main.launch` / `race_bringup.launch` 中）：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `car_id` | `"1"` | 车号（1 或 2），决定本车身份 |
| `dual_car_enabled` | `false` | 是否启用双车协作 |

---

## 4. 参数修改速查

### 4.1 裁判通信相关

| 修改目标 | 修改方式 | 涉及文件 |
|---|---|---|
| 裁判电脑 IP | launch 参数 `server_ip:=192.168.x.x` 或修改各 launch 文件中的 default | [race_bringup.launch](launch/race_bringup.launch) [main.launch](launch/main.launch) [tcp.yaml](config/tcp.yaml) [tcp_reporter.py](scripts/tcp_reporter.py) |
| 裁判端口 | launch 参数 `server_port:=xxxx` | 同上 |
| 车号 | launch 参数 `car_id:=1` | 同上 |
| 上报频率 | 修改 `tcp.yaml` 中 `report.hz` | [tcp.yaml](config/tcp.yaml) |
| TCP 超时/重连 | 修改 `tcp.yaml` 中 `server.connect_timeout_seconds` / `server.reconnect_seconds` | [tcp.yaml](config/tcp.yaml) |

### 4.2 双车通信相关

| 修改目标 | 修改方式 | 涉及文件 |
|---|---|---|
| 启用双车 | launch 参数 `dual_car_enabled:=true` | [race_bringup.launch](launch/race_bringup.launch) |
| 首发车号 | 修改 `strategy.yaml` 中 `dual_car_start_first_car_id` 或 launch 覆盖 | [strategy.yaml](config/strategy.yaml) |
| 信号话题名 | 修改 `strategy.yaml` 中 `dual_car_signal_topic` | [strategy.yaml](config/strategy.yaml) |

---

## 5. 调试方法

### 5.1 查看 TCP 上报内容

```bash
# 启动假服务端（本地监听，模拟裁判软件）
rosrun pharmacy_mplus0_debug tcp_fake_server.py _port:=8888

# 另开终端启动上报节点（指向本机）
rosrun pharmacy_mplus0 tcp_reporter.py _server_ip:=127.0.0.1 _audio_dir:=""

# 假服务端会格式化打印每条 JSON
```

输出示例：

```
#1    | car=1  speed=0.000  odom=(0.000,0.000)  task=R  CV1=WAIT-0  CV2=
#2    | car=1  speed=0.150  odom=(0.500,0.200)  task=R  CV1=WAIT-0  CV2=
```

### 5.2 查看双车信号

```bash
# 实时查看放行信号
rostopic echo /dual_car_signal

# 实时查看任务占用
rostopic echo /current_qr_task
```

### 5.3 手动模拟双车信号

```bash
# 手动放行车 2
rostopic pub /dual_car_signal std_msgs/String "data: 'ALLOW_START:2'" -1

# 模拟车 1 占用 box=0
rostopic pub /current_qr_task std_msgs/String "data: 'CAR1:AB-1'" -r 2

# 清空占用
rostopic pub /current_qr_task std_msgs/String "data: 'CAR1:'" -r 2
```

### 5.4 终端仪表盘监控

```bash
rosrun pharmacy_mplus0_debug topic_echo_dashboard.py
```

仪表盘同时显示 `/current_qr_task`、`/dual_car_signal`、`/current_task`、`/cv1_result`、`/cv2_result` 等关键话题。

---

## 6. 注意事项

- **TCP 上报的 `odom` 字段实际是 map 系坐标**，通过 TF `map → base_footprint` 查询。命名与 ROS `/odom` 话题无关，是裁判系统约定的字段名。
- **TCP 连接失败不影响比赛**，主控状态机不依赖 TCP 上报。
- **双车协作默认关闭**，单车模式不受任何影响。不传 `dual_car_enabled:=true` 时，`/dual_car_signal` 和 `/current_qr_task` 话题仍会被发布但无对车订阅。
- **`/current_qr_task` 是任务占用广播，不是出发控制信号**。出发控制由 `/dual_car_signal` 负责。
- **双车模式下两车必须连到同一个 ROS Master**，否则话题无法互通。
- 语音播报与 TCP 上报在同一个节点（`tcp_reporter`）中，禁用语音（`audio_dir:=""`）不会影响 TCP 上报。
- 远程任务共享（车 1 将第二个二维码分配给车 2）**默认关闭**，当前车 2 收到放行信号后正常前往识别板一自行识别。
