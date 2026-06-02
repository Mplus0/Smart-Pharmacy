# Codex 提示词：彻底移除双车 ROS topic 通信方案，仅保留 TCP 双车通信

## 一、任务背景

当前项目为 `pharmacy_mplus0`，用于智慧药房双车协作比赛。

当前 Codex 已经在原有“双车 ROS topic 通信”基础上新增了 TCP 双车通信方案。根据当前 README，代码中仍然存在两套双车通信方式：

1. `dual_car_comm_mode:=tcp`：两车独立 ROS Master，通过 TCP :9001 通信。
2. `dual_car_comm_mode:=ros_topic`：两车共用同一个 ROS Master，通过 `/dual_car_signal` 和 `/current_qr_task` 通信。

现在我已经确定最终比赛方案为：

```text
车 1：独立 ROS Master
车 2：独立 ROS Master
车 1 ↔ 车 2：只通过 TCP 通信
```

因此，本次任务不是继续兼容两种通信方案，而是要**安全地去除双车 ROS topic 通信方案**，让当前代码中双车协作只通过 TCP 完成。

注意：这里的“去除 ROS Master 通信方案”指的是去除“两车之间通过同一个 ROS Master 进行 ROS topic 通信”的方案。不是删除本车内部 ROS 机制。每辆车内部仍然正常使用 ROS：

```text
/move_base
/amcl
/map_server
/cmd_vel
/scan
/odom
/camera/rgb/image_raw
/current_task
/cv1_result
/cv2_result
/announce_request
```

两辆车之间不能再通过 `/dual_car_signal` 或 `/current_qr_task` 进行跨车通信。

---

## 二、最终目标

请将当前代码整理为：

```text
双车协作 = 独立 ROS Master + TCP 通信
```

最终应满足：

1. 两辆车必须各自使用独立 ROS Master。
2. 双车之间只通过 TCP JSON line 协议通信。
3. 删除或禁用 `dual_car_comm_mode:=ros_topic` 兼容逻辑。
4. 不再要求两车连接到同一个 ROS Master。
5. 不再通过 `/dual_car_signal` 跨车通信。
6. 不再通过 `/current_qr_task` 跨车通信。
7. 保留本车内部 ROS topic，例如 `/current_task`、`/cv1_result`、`/cv2_result`、`/announce_request`，因为这些是本车内部模块和裁判上报使用的。
8. 保留主控状态机、导航、识别、配送、裁判上报的主要逻辑，不要重写业务流程。
9. 保留单车模式：`dual_car_enabled:=false` 时，双车 TCP 不启动，原有单车流程不受影响。
10. 保留远程任务共享：车 1 扫描到多个二维码后，自己选择一个任务，将剩余任务通过 TCP 发送给车 2；车 2 收到远程任务后跳过识别板一。

---

## 三、必须先阅读和梳理的文件

请先完整阅读并梳理代码，不要立刻修改。

重点文件：

```text
README_DUAL_CAR.md
README_COMMUNICATION.md
README_CAR2_MIGRATION.md

pharmacy_mplus0/
├── config/
│   ├── strategy.yaml
│   └── tcp.yaml
├── launch/
│   ├── race_bringup.launch
│   ├── main.launch
│   ├── main_single.launch
│   └── reporter.launch
├── scripts/
│   ├── main_controller.py
│   ├── tcp_reporter.py
│   ├── board1_detector.py
│   └── board2_detector.py
└── src/pharmacy_mplus0/
    ├── dual_car_tcp.py
    ├── competition_io.py
    ├── constants.py
    ├── task_planner.py
    ├── models.py
    ├── tcp_client.py
    └── log_utils.py
```

重点检查：

1. `dual_car_comm_mode` 当前在哪里读取。
2. `dual_car_comm_mode:=ros_topic` 当前在哪里分支。
3. `/dual_car_signal` 的 publisher/subscriber 当前在哪里创建。
4. `/current_qr_task` 的 publisher/subscriber 当前在哪里创建。
5. `competition_io.py` 中 `publish_dual_signal()` 和 `publish_qr_task()` 当前是否仍被主控双车逻辑调用。
6. `main_controller.py` 中哪些函数已经抽象为 TCP/ROS 共用处理函数。
7. `dual_car_tcp.py` 当前是否已经具备 TCP server、TCP client、JSON line 接收、断线重连、非阻塞发送等能力。
8. 当前 README 是否仍然提到 `ros_topic` 兼容模式。

完成梳理后，先输出“现状分析”，不要直接修改。

---

## 四、本次修改范围

允许修改：

```text
config/strategy.yaml
launch/race_bringup.launch
launch/main.launch
launch/main_single.launch
scripts/main_controller.py
src/pharmacy_mplus0/dual_car_tcp.py
src/pharmacy_mplus0/competition_io.py
src/pharmacy_mplus0/constants.py
README_DUAL_CAR.md
README_COMMUNICATION.md
README_CAR2_MIGRATION.md
```

谨慎修改：

```text
scripts/tcp_reporter.py
src/pharmacy_mplus0/tcp_client.py
```

原则上不要修改，除非发现与双车 TCP 清理直接相关的问题。

禁止修改：

```text
导航航点逻辑
move_base action 名称
AMCL / map_server / EKF / 雷达 / 相机启动逻辑
识别板一识别算法
识别板二识别算法
体检取样逻辑
化验投递逻辑
语音播报资源
```

---

## 五、核心修改要求

### 5.1 移除 `ros_topic` 双车通信模式

请安全移除或彻底禁用以下逻辑：

```text
dual_car_comm_mode == "ros_topic"
```

最终不应该再需要通过：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true dual_car_comm_mode:=ros_topic
```

启动双车协作。

建议处理方式：

1. 删除 `dual_car_comm_mode` 参数，或保留但强制只允许 `tcp`。
2. 如果保留该参数，遇到非 `tcp` 值时应打印错误并自动回退到 `tcp`，或者明确拒绝启动双车模式。
3. 不要继续保留“兼容 ROS topic 模式”的 README、launch 示例和调试命令。

推荐最终配置：

```yaml
# 双车协作开关
dual_car_enabled: false

# 双车通信固定使用 TCP，不再支持 ros_topic 跨车通信
dual_car_peer_ip: ""
dual_car_listen_ip: "0.0.0.0"
dual_car_listen_port: 9001
dual_car_peer_port: 9001
dual_car_tcp_connect_timeout_sec: 1.0
dual_car_tcp_reconnect_sec: 2.0
```

可以删除：

```yaml
dual_car_comm_mode: "tcp"
dual_car_signal_topic: "/dual_car_signal"
```

如果考虑短期兼容启动参数，也可以暂时保留 `dual_car_comm_mode` 参数，但必须满足：

```text
无论用户传什么，双车跨车通信都只能走 TCP。
```

---

### 5.2 移除跨车 `/dual_car_signal`

原用途：

```text
ALLOW_START:1
ALLOW_START:2
ALLOW_START_WITH_TASK JSON
```

新用途：全部改为 TCP 消息。

请确保：

1. `main_controller.py` 中不再订阅 `/dual_car_signal` 用于跨车通信。
2. `main_controller.py` 中不再发布 `/dual_car_signal` 用于跨车通信。
3. 原来的 `_cb_dual_car_signal()` 如果只用于旧方案，应删除。
4. 如果该函数中有可复用业务逻辑，应保留为协议无关函数，例如：

```python
def _dual_handle_allow_start_signal(self, target_id, from_car=None):
    pass

def _dual_handle_remote_task_data(self, data):
    pass
```

5. TCP 收到 `allow_start` 时调用 `_dual_handle_allow_start_signal()`。
6. TCP 收到 `remote_task` 时调用 `_dual_handle_remote_task_data()`。

---

### 5.3 移除跨车 `/current_qr_task`

原用途：

```text
CAR1:AB-1
CAR1:
CAR2:ABC-3
CAR2:
```

新用途：全部改为 TCP 消息。

请确保：

1. `main_controller.py` 中不再订阅 `/current_qr_task` 用于获取对车占用。
2. `main_controller.py` 中不再发布 `/current_qr_task` 用于告诉对车占用。
3. 原来的 `_cb_peer_qr_task()` 如果只用于旧方案，应删除。
4. 如果其中有解析业务逻辑，应改为 TCP payload 处理函数，例如：

```python
def _dual_handle_peer_qr_task_payload(self, payload):
    pass

def _dual_handle_peer_qr_task_clear(self, payload):
    pass
```

5. 本车选定任务时，通过 TCP 发送：

```json
{
  "type": "qr_task",
  "from_car": "1",
  "code": "AB",
  "lab_window": "1",
  "box": 0,
  "stamp": 1760000000.123
}
```

6. 本轮结束或清空占用时，通过 TCP 发送：

```json
{
  "type": "qr_task_clear",
  "from_car": "1",
  "stamp": 1760000000.123
}
```

7. 收到 `qr_task` 后更新：

```python
self._peer_occupied_box = box
```

8. 收到 `qr_task_clear` 后更新：

```python
self._peer_occupied_box = None
```

---

### 5.4 保留 TCP 消息类型

最终双车 TCP 至少支持以下消息：

#### 放行消息

```json
{
  "type": "allow_start",
  "from_car": "1",
  "target_car": "2",
  "stamp": 1760000000.123
}
```

#### 远程任务消息

```json
{
  "type": "remote_task",
  "from_car": "1",
  "target_car": "2",
  "stamp": 1760000000.123,
  "task": {
    "code": "C",
    "box": 2,
    "lab_window": "3",
    "sample_count": 1
  }
}
```

#### 任务占用消息

```json
{
  "type": "qr_task",
  "from_car": "1",
  "code": "AB",
  "lab_window": "1",
  "box": 0,
  "stamp": 1760000000.123
}
```

#### 清空占用消息

```json
{
  "type": "qr_task_clear",
  "from_car": "1",
  "stamp": 1760000000.123
}
```

#### ACK，可选

```json
{
  "type": "ack",
  "from_car": "2",
  "target_car": "1",
  "ack_type": "remote_task",
  "status": "accepted",
  "stamp": 1760000000.456
}
```

ACK 只能用于日志，不能让主流程等待 ACK。

---

## 六、分阶段执行要求

请严格分阶段进行，不要一次性改完所有文件。

---

### 阶段 0：现状梳理，不修改代码

请输出：

1. 当前代码中 TCP 双车通信已经实现了哪些内容。
2. 当前代码中 ROS topic 双车通信还残留在哪些文件、哪些函数、哪些参数。
3. 哪些内容可以删除。
4. 哪些内容需要保留但改名或改为本车内部用途。
5. 预计修改文件清单。
6. 风险点。

如果发现 README 与代码不一致，先说明，不要盲目修改。

---

### 阶段 1：固定双车通信模式为 TCP

目标：让代码层面不再存在可切换到 `ros_topic` 的双车通信路径。

要求：

1. 修改 `strategy.yaml`，删除或弃用 `dual_car_comm_mode`、`dual_car_signal_topic`。
2. 修改 `launch/main.launch` 和 `launch/race_bringup.launch`，删除或弃用 `dual_car_comm_mode` 参数。
3. `main_controller.py` 中不要再根据 `dual_car_comm_mode` 分支选择 ROS topic 或 TCP。
4. `dual_car_enabled=true` 时，只尝试启动 `DualCarTcpBridge`。
5. 如果 `dual_car_peer_ip` 为空：
   - 打印明确 warning。
   - 不启动 TCP bridge。
   - 不让程序崩溃。
   - 但要说明双车协作无法真正与对车通信。

---

### 阶段 2：清理 `/dual_car_signal` 跨车逻辑

目标：放行和远程任务共享只通过 TCP。

要求：

1. 删除或停用 `/dual_car_signal` 的 subscriber。
2. 删除或停用 `/dual_car_signal` 的 publisher。
3. 保留放行处理业务逻辑，但由 TCP 回调触发。
4. 保留远程任务解析业务逻辑，但由 TCP `remote_task` 触发。
5. 删除 README 中关于 `rostopic pub /dual_car_signal` 的双车调试命令。

---

### 阶段 3：清理 `/current_qr_task` 跨车逻辑

目标：任务占用排除只通过 TCP。

要求：

1. 删除或停用 `/current_qr_task` 的双车占用 subscriber。
2. 删除或停用 `/current_qr_task` 的双车占用 publisher。
3. 确认本车选定任务时，TCP 发送 `qr_task`，且包含 `box`。
4. 确认本轮结束时，TCP 发送 `qr_task_clear`。
5. 收到对车 `qr_task` 后更新 `_peer_occupied_box`。
6. 收到对车 `qr_task_clear` 后清空 `_peer_occupied_box`。
7. 如果 `competition_io.publish_qr_task()` 不再被使用，删除或标注废弃。
8. 不要影响裁判上报需要的 `/current_task`、`/cv1_result`、`/cv2_result`。

---

### 阶段 4：清理 constants 和 competition_io

目标：去除不再使用的双车 ROS topic 常量和接口。

要求：

1. 检查 `constants.py` 中：
   - `TOPIC_DUAL_CAR_SIGNAL`
   - `TOPIC_CURRENT_QR_TASK`

   如果只用于旧双车跨车通信，则删除。

2. 检查 `competition_io.py` 中：
   - `publish_dual_signal()`
   - `publish_qr_task()`

   如果只用于旧双车跨车通信，则删除或标注废弃。

3. 不要删除本车内部仍需发布给 `tcp_reporter.py` 的接口，例如：
   - `/current_task`
   - `/cv2_result`
   - `/announce_request`

---

### 阶段 5：更新 README 和启动命令

目标：文档中只保留最终方案。

需要更新：

```text
README_DUAL_CAR.md
README_COMMUNICATION.md
README_CAR2_MIGRATION.md
```

要求：

1. 删除或改写“ROS topic 模式兼容保留”的描述。
2. 明确最终方案：

```text
两车独立 ROS Master + TCP :9001 双车通信
```

3. 删除以下旧命令：

```bash
rostopic echo /dual_car_signal
rostopic echo /current_qr_task
rostopic pub /dual_car_signal ...
rostopic pub /current_qr_task ...
roslaunch ... dual_car_comm_mode:=ros_topic
```

4. 保留 TCP 调试命令，例如：

```bash
ss -tlnp | grep 9001
nc -l 9001
echo '{"type":"allow_start","from_car":"1","target_car":"2","stamp":1}' | nc 192.168.124.9 9001
```

5. 给出车 1 启动命令：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.124.3
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_peer_ip:=192.168.124.9 \
  stream_url:=http://192.168.124.3:8080/stream?topic=/camera/rgb/image_raw
```

6. 给出车 2 启动命令：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.124.9
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_peer_ip:=192.168.124.3 \
  stream_url:=http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw
```

7. 文档中必须明确：

```text
不要让两辆车使用同一个 ROS_MASTER_URI。
不要再用 ROS topic 做双车跨车通信。
```

---

## 七、验收标准

### 7.1 代码搜索验收

完成后，请执行并检查：

```bash
grep -R "dual_car_comm_mode" -n .
grep -R "ros_topic" -n .
grep -R "dual_car_signal" -n .
grep -R "current_qr_task" -n .
```

要求：

1. `dual_car_comm_mode` 不应再作为运行时通信模式分支。
2. `ros_topic` 不应再作为双车通信方案出现。
3. `/dual_car_signal` 不应再用于跨车通信。
4. `/current_qr_task` 不应再用于跨车任务占用。
5. 如果某些字符串只出现在历史说明或明确废弃说明中，需要说明原因。

### 7.2 单车模式验收

```bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1 dual_car_enabled:=false
```

要求：

```text
主控正常启动
不要求 dual_car_peer_ip
不启动双车 TCP bridge
导航、识别、裁判上报不受影响
```

### 7.3 双车 TCP 模式验收

车 1：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.124.3
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_peer_ip:=192.168.124.9
```

车 2：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.124.9
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_peer_ip:=192.168.124.3
```

要求：

```text
两车 ROS_MASTER_URI 都指向本机
不会出现 new node registered with same name
两车各自启动 /move_base /amcl /map_server 不冲突
车 1 能通过 TCP allow_start 放行车 2
车 2 能通过 TCP allow_start 放行车 1
车 1 能通过 TCP remote_task 给车 2 分配远程任务
车 2 收到 remote_task 后能跳过识别板一
TCP 断开或对车未启动不会导致 main_controller 崩溃
```

---

## 八、禁止事项

1. 不要保留“双车 ROS topic 通信”作为可运行方案。
2. 不要要求两辆车共用同一个 ROS Master。
3. 不要进行多机器人 namespace 改造。
4. 不要修改 `/move_base`、`/amcl`、`/map_server` 等本车内部节点名。
5. 不要修改 TF frame。
6. 不要改导航航点。
7. 不要重写主控状态机。
8. 不要把双车 TCP 和裁判 TCP 混用同一个端口。
9. 不要让 TCP 连接失败导致主控退出。
10. 不要一次性修改大量文件，每阶段完成后先说明修改内容和测试方式。

---

## 九、每阶段输出格式

每完成一个阶段，请按以下格式输出：

```text
阶段 X 完成情况

1. 已阅读/已修改的文件
- ...

2. 本阶段修改内容
- ...

3. 删除或禁用的旧 ROS topic 通信内容
- ...

4. 保留的本车内部 ROS 内容
- ...

5. 如何测试
- ...

6. 风险和需要我确认的问题
- ...
```

如果发现当前代码与 README 不一致，或者发现某个 ROS topic 可能仍被本车内部模块依赖，请不要直接删除，先说明依赖关系并向我确认。
