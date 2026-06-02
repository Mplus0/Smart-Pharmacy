# 通信系统说明

本文档描述 `pharmacy_mplus0` 的两套通信系统：小车→裁判系统的 TCP 上报，以及两车之间的 TCP 通信。

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
    │  ROS Master 独立  │  TCP :9001      │  ROS Master 独立  │
    │  tcp_reporter    │◄═══════════════►│  tcp_reporter    │
    │  main_controller │ dual_car TCP     │  main_controller │
    │                  │ (JSON line)      │                  │
    └──────────────────┘                 └──────────────────┘
```

**各设备 IP 一览**：

| 设备 | IP | 说明 |
|---|---|---|
| 裁判电脑 | `192.168.124.2` | TCP 服务端，监听端口 `8888` |
| 车一 | `192.168.124.3` | 摄像头视频流 + ROS 节点 |
| 车二 | `192.168.124.9` | 摄像头视频流 + ROS 节点 |

- **小车 → 裁判**：TCP 长连接，每车独立上报。上报失败不阻塞比赛流程。
- **车 1 ↔ 车 2**：TCP 通信，两车使用独立 ROS Master，通过 TCP :9001 传递 JSON 消息。

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

## 3. 车 1 ↔ 车 2（TCP 通信，默认）

### 3.1 架构

两车通过 TCP :9001 直接通信，各自使用独立 ROS Master。

```text
车 1  main_controller                  车 2  main_controller
  │                                        │
  │  DualCarTcpBridge                      │  DualCarTcpBridge
  │    ├── TCP Server :9001 (接收)          │    ├── TCP Server :9001 (接收)
  │    └── TCP Client → 车2:9001 (发送)     │    └── TCP Client → 车1:9001 (发送)
  │                                        │
  │  TCP 消息:                              │   TCP 消息:
  │    allow_start / remote_task            │    allow_start / remote_task
  │    qr_task / qr_task_clear              │    qr_task / qr_task_clear
```

关键代码文件：

| 文件 | 作用 |
|---|---|
| [scripts/main_controller.py](scripts/main_controller.py) | 双车核心调度：TCP 或 ROS 话题，控制出发和等待 |
| [src/pharmacy_mplus0/dual_car_tcp.py](src/pharmacy_mplus0/dual_car_tcp.py) | **新增**：双车 TCP 通信模块（`DualCarTcpBridge`） |
| [src/pharmacy_mplus0/competition_io.py](src/pharmacy_mplus0/competition_io.py) | 提供 `publish_dual_signal()` 和 `publish_qr_task()` 接口（ROS topic 模式） |
| [src/pharmacy_mplus0/task_planner.py](src/pharmacy_mplus0/task_planner.py) | `select_best(excluded_box=...)` 排除对车占用的方框 |
| [config/strategy.yaml](config/strategy.yaml) | 双车开关、通信模式和首发车配置 |

### 3.2 通信协议

每辆车同时运行 TCP Server（监听 :9001）和 TCP Client（连接对车 :9001）。
协议为 JSON + 换行符 `\n` 分隔。

| 消息类型 | JSON 格式 | 作用 |
|---|---|---|
| `allow_start` | `{"type":"allow_start","from_car":"1","target_car":"2","stamp":...}` | 放行对车出发 |
| `remote_task` | `{"type":"remote_task","from_car":"1","target_car":"2","stamp":...,"task":{...}}` | 远程任务共享 |
| `qr_task` | `{"type":"qr_task","from_car":"1","code":"AB","lab_window":"1","box":0,"stamp":...}` | 任务占用广播 |
| `qr_task_clear` | `{"type":"qr_task_clear","from_car":"1","stamp":...}` | 清空任务占用 |
| `ack` | `{"type":"ack","from_car":"2","target_car":"1","ack_type":"...","status":"...","stamp":...}` | ACK（仅用于日志） |

### 3.3 相关配置参数

[strategy.yaml](config/strategy.yaml)：

```yaml
# 双车协作开关，默认关闭
dual_car_enabled: false
# 开局允许先出发的车号
dual_car_start_first_car_id: "1"
# 远程任务共享（默认关闭）
dual_car_remote_task_enabled: false

# 双车通信固定使用 TCP（独立 ROS Master），不再支持 ros_topic 跨车通信。
# TCP 通信参数
dual_car_peer_ip: ""            # 对车 IP，车1填192.168.124.9，车2填192.168.124.3
dual_car_listen_ip: "0.0.0.0"
dual_car_listen_port: 9001
dual_car_peer_port: 9001
dual_car_tcp_connect_timeout_sec: 1.0
dual_car_tcp_reconnect_sec: 2.0
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
| 双车 TCP 通信 | launch 参数 `dual_car_peer_ip:=192.168.124.x` | [race_bringup.launch](launch/race_bringup.launch) |
| 对车 IP | launch 参数 `dual_car_peer_ip:=192.168.124.x` | [race_bringup.launch](launch/race_bringup.launch) |
| TCP 端口 | launch 参数 `dual_car_listen_port:=9001` / `dual_car_peer_port:=9001` | [race_bringup.launch](launch/race_bringup.launch) |
| 首发车号 | 修改 `strategy.yaml` 中 `dual_car_start_first_car_id` 或 launch 覆盖 | [strategy.yaml](config/strategy.yaml) |
| 远程任务共享 | launch 参数 `dual_car_remote_task_enabled:=true` | [race_bringup.launch](launch/race_bringup.launch) |
| 信号话题名（ROS topic 模式） | 修改 `strategy.yaml` 中 `dual_car_signal_topic` | [strategy.yaml](config/strategy.yaml) |

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

### 5.2 查看双车通信

```bash
# 检查 TCP 端口监听状态
ss -tlnp | grep 9001

# 查看主控日志中的双车通信信息
# 日志前缀: [DualCar] / [DualCarTcp]

# 用 nc 监听本车端口，查看收到的 TCP 消息
nc -l 9001

# 用 nc 手动向对车发送放行信号
echo '{"type":"allow_start","from_car":"1","target_car":"2","stamp":1}' | nc 192.168.124.9 9001

# 模拟任务占用
echo '{"type":"qr_task","from_car":"2","code":"AB","lab_window":"1","box":0,"stamp":1}' | nc 192.168.124.9 9001

# 清空占用
echo '{"type":"qr_task_clear","from_car":"2","stamp":1}' | nc 192.168.124.9 9001
```

### 5.3 终端仪表盘监控

```bash
rosrun pharmacy_mplus0_debug topic_echo_dashboard.py
```

仪表盘同时显示 `/current_task`、`/cv1_result`、`/cv2_result` 等本车话题。双车通信已通过 TCP 传输，不再通过 ROS 话题显示。

---

## 6. 注意事项

- **TCP 上报的 `odom` 字段实际是 map 系坐标**，通过 TF `map → base_footprint` 查询。命名与 ROS `/odom` 话题无关，是裁判系统约定的字段名。
- **TCP 连接失败不影响比赛**，主控状态机不依赖 TCP 上报。
- **双车协作默认关闭**，单车模式不受任何影响。不传 `dual_car_enabled:=true` 时，双车 TCP 模块不会启动。
- **双车 TCP 通信**：两车使用独立 ROS Master，通过 TCP :9001 通信。**不会出现节点名冲突**（`/move_base`、`/amcl` 等各车独立）。不要使用同一个 ROS_MASTER_URI。
- **双车 TCP 端口与裁判 TCP 端口不同**：双车 TCP 默认 :9001，裁判上报 TCP :8888，互不干扰。
- **任务占用广播和出发控制均通过 TCP 消息完成**。`qr_task`/`qr_task_clear` 负责任务占用，`allow_start` 负责出发控制。
- 语音播报与 TCP 上报在同一个节点（`tcp_reporter`）中，禁用语音（`audio_dir:=""`）不会影响 TCP 上报。
- 远程任务共享（车 1 将第二个二维码分配给车 2）**默认关闭**，当前车 2 收到放行信号后正常前往识别板一自行识别。
- **双车 TCP 连接失败不阻塞启动**：对车未启动时本车 TCP 客户端持续重连，不影响主控流程。
