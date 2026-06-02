# Codex 修改任务：将双车通信从 ROS 话题改为 TCP 通信，保留主控流程不变

## 一、任务背景

当前项目为 `pharmacy_mplus0`，用于智慧药房双车协作比赛。

当前双车方案原本要求两辆车连接到同一个 ROS Master，通过 ROS 话题完成双车通信：

- `/dual_car_signal`：用于轮流出发信号，例如 `ALLOW_START:2`
- `/current_qr_task`：用于任务占用广播，例如 `CAR1:AB-1`
- `/dual_car_signal` 同时也承载远程任务共享 JSON，例如 `ALLOW_START_WITH_TASK`

但是实测中，两辆车共用同一个 ROS Master 会导致大量全局节点名冲突，例如 `/move_base`、`/amcl`、`/map_server`、`/web_video_server`、`/main_controller` 等节点互相抢占，最终导致导航 action server 掉线、主控退出。

因此现在决定改为：

```text
车 1：独立 ROS Master
车 2：独立 ROS Master
车 1 ↔ 车 2：只通过 TCP/Socket 通信
```

注意：本次不是重构导航系统，不是多机器人 namespace 改造，也不是修改主控状态机整体逻辑。目标只是把原来跨车使用 ROS 话题传递的双车通信数据，改成 TCP 传输。

---

## 二、总体目标

请在阅读并梳理当前代码结构后，分阶段完成以下目标：

1. 保持两辆车使用同一套代码。
2. 两辆车各自使用独立 ROS Master。
3. 不再依赖跨车 ROS 话题通信。
4. 使用 TCP 在两辆车之间传递原 `/dual_car_signal` 和 `/current_qr_task` 的等价信息。
5. 保留当前主控状态机、导航流程、识别流程、配送流程，不要重写主要业务逻辑。
6. 保留单车模式完全可用，`dual_car_enabled=false` 时行为不变。
7. 保留当前远程任务共享逻辑：车 1 扫描到多个二维码后，自己选择一个任务，将剩余任务发送给车 2；车 2 收到远程任务后跳过识别板一，直接进入配送流程。
8. 不要一次性大规模修改文件。必须先梳理代码，再分阶段、小步修改、每一步都说明修改点和验证方法。
9. 如果发现当前代码结构与 README 描述不一致，先停止并向我提问，不要盲目猜测修改。

---

## 三、必须先阅读的文件

请先完整阅读并梳理以下文件，不要立刻修改：

```text
README_COMMUNICATION.md
README_DUAL_CAR.md

pharmacy_mplus0/
├── launch/
│   ├── race_bringup.launch
│   ├── main.launch
│   └── reporter.launch
├── config/
│   ├── strategy.yaml
│   └── tcp.yaml
├── scripts/
│   ├── main_controller.py
│   ├── tcp_reporter.py
│   ├── board1_detector.py
│   └── board2_detector.py
└── src/pharmacy_mplus0/
    ├── competition_io.py
    ├── tcp_client.py
    ├── task_planner.py
    ├── models.py
    ├── constants.py
    └── log_utils.py
```

重点梳理：

1. `main_controller.py` 中所有双车相关变量和方法：
   - `_dual_car_enabled`
   - `_car_id`
   - `_dual_waiting_at_start`
   - `_dual_start_allowed`
   - `_dual_allow_sent_this_round`
   - `_peer_occupied_box`
   - `_dual_remote_task_enabled`
   - `_dual_assigned_remote_task`
   - `_dual_remote_task_sent_this_round`
   - `_cb_dual_car_signal`
   - `_cb_peer_qr_task`
   - `_dual_publish_allow_peer_start`
   - `_dual_publish_remote_task_for_peer`
   - `_dual_try_parse_remote_task`
   - `_dual_apply_remote_task_if_available`

2. `competition_io.py` 中：
   - `publish_dual_signal()`
   - `publish_qr_task()`

3. `tcp_client.py` 中已有 TCP 客户端是否可以复用，还是应该新增一个独立的双车 TCP 通信模块。

4. `launch/main.launch` 和 `launch/race_bringup.launch` 中当前双车参数如何传递。

5. `strategy.yaml` 中当前双车配置项。

---

## 四、修改原则

### 4.1 只修改通信部分，不重写主流程

本次只允许修改或新增与“双车通信”有关的代码。

不要改动以下核心业务逻辑，除非为了接入 TCP 通信做极小适配：

```text
导航航点逻辑
move_base action 调用逻辑
识别板一检测算法
识别板二检测算法
体检区取样逻辑
化验窗口投递逻辑
语音播报逻辑
裁判 tcp_reporter 上报逻辑
```

### 4.2 不要进行 ROS namespace 多机器人改造

本方案已经决定使用独立 ROS Master，因此不要把节点改成：

```text
/car1/move_base
/car2/move_base
```

不要改 TF frame，不要改 `/move_base` action 名称，不要改 `/cmd_vel`、`/scan`、`/map` 等本机内部话题。

每辆车在自己的 ROS Master 内部仍然可以使用：

```text
/move_base
/amcl
/map_server
/cmd_vel
/scan
/odom
/camera/rgb/image_raw
```

### 4.3 保留单车模式

当：

```bash
dual_car_enabled:=false
```

时，所有双车 TCP 模块都不应影响主控流程。

单车模式下不能因为 peer_ip 未配置、TCP 连接失败、端口被占用而导致主控退出。

### 4.4 TCP 异常不能阻塞比赛流程

双车 TCP 通信必须遵守：

```text
连接失败不阻塞启动
发送失败不抛异常导致主控退出
接收失败自动重连或等待
网络断开时本车主流程不能崩溃
```

---

## 五、推荐设计方案

### 5.1 新增双车 TCP 通信模块

建议新增文件：

```text
src/pharmacy_mplus0/dual_car_tcp.py
```

该模块只负责双车 TCP 通信，不要和裁判上报 `tcp_reporter.py` 混在一起。

建议实现一个类，例如：

```python
class DualCarTcpBridge(object):
    def __init__(
        self,
        car_id,
        peer_id,
        listen_ip,
        listen_port,
        peer_ip,
        peer_port,
        on_message,
        logger=None,
    ):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def send(self, payload):
        pass
```

基本要求：

1. Python 2.7 兼容，因为 ROS Melodic 默认是 Python 2.7。
2. 使用标准库 `socket`、`threading`、`json`、`time` 即可，不要引入新的第三方依赖。
3. TCP server 在后台线程监听。
4. TCP client 发送失败时自动丢弃本次消息或稍后重连，不允许阻塞主控状态机。
5. 每条消息使用 JSON + 换行符 `\n` 分隔，方便调试。
6. 接收到完整一行 JSON 后回调 `on_message(dict)`。
7. 解析失败时记录 warning，不要抛异常退出。
8. 支持重复连接、断线重连、对方未启动时持续等待。
9. socket 线程应支持 `stop()` 退出，避免 roslaunch 关闭时报错。

### 5.2 TCP 消息格式

请尽量复用当前 `/dual_car_signal` 和 `/current_qr_task` 的语义，但用统一 JSON 包装。

#### 5.2.1 放行消息

原 ROS 话题格式：

```text
ALLOW_START:2
```

改为 TCP JSON：

```json
{
  "type": "allow_start",
  "from_car": "1",
  "target_car": "2",
  "stamp": 1760000000.123
}
```

车 2 收到后，应触发与原 `_cb_dual_car_signal()` 收到 `ALLOW_START:2` 一样的效果：

```text
_dual_start_allowed = True
```

但仍需保留原有条件判断：

```text
只处理 target_car == self._car_id
配送途中收到放行信号不能打断当前任务
```

#### 5.2.2 任务占用消息

原 ROS 话题格式：

```text
CAR1:AB-1
CAR1:
```

改为 TCP JSON：

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

清空占用：

```json
{
  "type": "qr_task_clear",
  "from_car": "1",
  "stamp": 1760000000.123
}
```

车 2 收到后，应触发与原 `_cb_peer_qr_task()` 一样的效果：

```text
_peer_occupied_box = 对车占用的 box
或清空为 None
```

如果当前原有 `publish_qr_task(code, lab_window)` 没有传递 box，请先梳理现有代码如何从二维码任务得到 box_index。不要盲目硬编码。如果无法确定，请先向我提问。

#### 5.2.3 远程任务共享消息

原 ROS JSON：

```json
{
  "type": "ALLOW_START_WITH_TASK",
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

TCP 中可以继续使用等价结构，但建议统一小写类型：

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

车 2 收到后，应复用原有远程任务校验和应用逻辑：

```text
校验字段合法性
target_car 必须等于 self._car_id
from_car 不能等于 self._car_id
不因为 stamp 过旧丢弃
缓存到 _dual_assigned_remote_task
允许跳过识别板一
```

如果为了兼容旧逻辑，也可以同时支持旧类型：

```text
ALLOW_START_WITH_TASK
```

但不要让两套逻辑重复执行。

#### 5.2.4 ACK 消息，建议实现但不要影响主流程

建议添加 ACK 方便调试：

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

ACK 只用于日志，不要让车 1 阻塞等待 ACK。

---

## 六、参数设计

请新增或完善以下参数，要求可以通过 launch 覆盖。

### 6.1 strategy.yaml 建议新增

```yaml
# 双车 TCP 通信配置
dual_car_comm_mode: "tcp"        # "ros_topic" 或 "tcp"，默认建议 "tcp"
dual_car_tcp_enabled: true       # 双车 TCP 通信开关，配合 dual_car_enabled 使用

dual_car_peer_ip: "192.168.124.9"
dual_car_listen_ip: "0.0.0.0"
dual_car_listen_port: 9001
dual_car_peer_port: 9001

dual_car_tcp_connect_timeout_sec: 1.0
dual_car_tcp_reconnect_sec: 2.0
```

说明：

1. `dual_car_enabled=false` 时，不启动双车 TCP。
2. `dual_car_enabled=true` 且 `dual_car_comm_mode=tcp` 时，使用 TCP 进行双车通信。
3. 可以保留 `ros_topic` 模式作为兼容，但不要让默认配置继续要求两辆车共用 ROS Master。
4. 如果保留 `ros_topic` 模式，请保证旧逻辑还能工作，但本次重点是 TCP 模式。

### 6.2 launch 参数建议新增

在 `launch/main.launch` 和 `launch/race_bringup.launch` 中新增或透传：

```xml
<arg name="dual_car_comm_mode" default="tcp" />
<arg name="dual_car_peer_ip" default="" />
<arg name="dual_car_listen_ip" default="0.0.0.0" />
<arg name="dual_car_listen_port" default="9001" />
<arg name="dual_car_peer_port" default="9001" />
```

启动示例：

车 1：

```bash
export ROS_MASTER_URI=http://192.168.124.3:11311
export ROS_IP=192.168.124.3
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_comm_mode:=tcp \
  dual_car_peer_ip:=192.168.124.9 \
  dual_car_listen_port:=9001 \
  dual_car_peer_port:=9001
```

车 2：

```bash
export ROS_MASTER_URI=http://192.168.124.9:11311
export ROS_IP=192.168.124.9
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_comm_mode:=tcp \
  dual_car_peer_ip:=192.168.124.3 \
  dual_car_listen_port:=9001 \
  dual_car_peer_port:=9001
```

注意：两辆车端口可以相同，因为它们运行在不同 IP 上。

---

## 七、分阶段执行要求

请严格分阶段进行，不要一次性改完所有文件。

---

### 阶段 0：代码结构梳理，不修改代码

请先输出以下内容：

1. 当前双车通信相关文件清单。
2. 当前 `/dual_car_signal` 的发布位置、订阅位置、消息格式、处理函数。
3. 当前 `/current_qr_task` 的发布位置、订阅位置、消息格式、处理函数。
4. 当前远程任务共享 JSON 的生成位置、解析位置、应用位置。
5. 当前 `competition_io.py` 和 `main_controller.py` 的职责边界。
6. 当前是否已有可复用的 TCP 客户端代码，例如 `tcp_client.py`。
7. 你计划新增或修改哪些文件，每个文件的修改目的是什么。

在完成阶段 0 之前，不要修改任何代码。

---

### 阶段 1：新增 TCP 通信模块，不接入主流程

目标：只新增 `dual_car_tcp.py`，不改主控状态机。

要求：

1. 新增 `src/pharmacy_mplus0/dual_car_tcp.py`。
2. 实现 TCP server + client 发送。
3. 实现 JSON line 协议。
4. 实现接收回调。
5. 实现异常保护。
6. 实现最小自测方法或说明如何用 Python 命令测试。
7. 不修改 `main_controller.py` 的业务流程。

完成后请输出：

```text
修改文件：
- xxx

验证方法：
- xxx

风险：
- xxx
```

---

### 阶段 2：添加参数和 launch 透传

目标：让主控能读取 TCP 通信参数，但还不替换原 ROS 话题逻辑。

修改范围：

```text
config/strategy.yaml
launch/main.launch
launch/race_bringup.launch
scripts/main_controller.py
```

要求：

1. 新增 `dual_car_comm_mode` 参数。
2. 新增 `dual_car_peer_ip`、`dual_car_listen_ip`、`dual_car_listen_port`、`dual_car_peer_port`。
3. `main_controller.py` 启动时打印当前双车通信模式。
4. 如果 `dual_car_enabled=true` 但 `dual_car_comm_mode=tcp` 且 `dual_car_peer_ip` 为空，应打印明确 warning，但不要直接崩溃。
5. 不要删除原 ROS 话题逻辑。

---

### 阶段 3：将 `/dual_car_signal` 等价替换为 TCP

目标：先只替换“放行信号”和“远程任务共享”，暂时不处理任务占用排除。

修改范围：

```text
scripts/main_controller.py
src/pharmacy_mplus0/dual_car_tcp.py
```

要求：

1. 当 `dual_car_comm_mode=tcp` 时：
   - `_dual_publish_allow_peer_start()` 通过 TCP 发送 `allow_start`
   - `_dual_publish_remote_task_for_peer()` 通过 TCP 发送 `remote_task`

2. 收到 TCP `allow_start` 时：
   - 复用原 `_cb_dual_car_signal()` 的判断逻辑
   - 或抽出公共函数，例如 `_dual_handle_allow_start(target_car, from_car, raw_msg)`

3. 收到 TCP `remote_task` 时：
   - 复用原 `_dual_try_parse_remote_task()` 或抽出公共解析函数
   - 不要复制粘贴两套相似逻辑

4. `dual_car_comm_mode=ros_topic` 时，原 ROS 话题方式仍可用。

5. 不要修改导航、识别、配送状态机。

完成后，请给出测试方法：

- 不启动车二时，车一不会崩溃。
- 启动车二后，车一完成配送能通过 TCP 放行车二。
- 车一发送 remote_task 后，车二能缓存远程任务并跳过识别板一。

---

### 阶段 4：将 `/current_qr_task` 等价替换为 TCP

目标：替换任务占用排除信息。

要求：

1. 当 `dual_car_comm_mode=tcp` 时：
   - 本车选定二维码任务后，通过 TCP 发送 `qr_task`
   - 本轮结束或清空任务时，通过 TCP 发送 `qr_task_clear`

2. 收到 `qr_task` 后：
   - 更新 `_peer_occupied_box`
   - 行为应与原 `_cb_peer_qr_task()` 一致

3. 收到 `qr_task_clear` 后：
   - `_peer_occupied_box = None`

4. 如果当前发布占用时拿不到 box_index：
   - 不要硬编码
   - 不要乱猜
   - 先说明需要在哪个对象或函数中传入 box_index
   - 等我确认后再改

---

### 阶段 5：清理兼容逻辑和 README

目标：确认 TCP 模式可用后，再更新文档。

要求：

1. 更新 `README_COMMUNICATION.md`：
   - 说明双车通信已从 ROS 话题改为 TCP。
   - 说明两辆车使用独立 ROS Master。
   - 保留裁判 TCP 上报说明，并明确裁判上报 TCP 与双车 TCP 不是同一个端口。
   - 更新启动命令。
   - 更新调试方法。

2. 更新 `README_DUAL_CAR.md`：
   - 说明当前双车方案为“独立 ROS Master + TCP 通信”。
   - 更新放行、任务占用、远程任务共享的消息格式。
   - 删除或标注旧的 `rostopic echo /dual_car_signal` 调试方法只适用于 `ros_topic` 兼容模式。
   - 增加 TCP 调试命令，例如 `nc`、日志查看、端口检查。

3. 不要删除旧文档内容，除非确认旧方案完全不再支持。可以标注为“旧 ROS topic 模式，仅兼容保留”。

---

## 八、接入 main_controller.py 的建议方式

为了避免破坏主控状态机，请优先将原回调逻辑拆成“协议无关处理函数”。

例如，当前可能有：

```python
def _cb_dual_car_signal(self, msg):
    body = msg.data.strip()
    ...
```

建议改为：

```python
def _cb_dual_car_signal(self, msg):
    self._dual_handle_signal_text(msg.data)

def _dual_handle_signal_text(self, body):
    # 兼容原 ROS topic 文本格式：
    # ALLOW_START:2
    # JSON ALLOW_START_WITH_TASK
    pass

def _dual_handle_tcp_message(self, payload):
    msg_type = payload.get("type")
    if msg_type == "allow_start":
        self._dual_handle_allow_start(payload)
    elif msg_type == "remote_task":
        self._dual_handle_remote_task_payload(payload)
    elif msg_type == "qr_task":
        self._dual_handle_peer_qr_task_payload(payload)
    elif msg_type == "qr_task_clear":
        self._dual_handle_peer_qr_task_clear(payload)
```

不要把 TCP socket 细节写进状态机各个状态函数里。

---

## 九、验收标准

完成后必须满足：

### 9.1 单车模式

```bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1 dual_car_enabled:=false
```

要求：

```text
主控正常启动
不要求 peer_ip
不启动或不依赖双车 TCP
导航和识别流程不受影响
```

### 9.2 双车 TCP 模式

车 1：

```bash
export ROS_MASTER_URI=http://192.168.124.3:11311
export ROS_IP=192.168.124.3
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_comm_mode:=tcp \
  dual_car_peer_ip:=192.168.124.9
```

车 2：

```bash
export ROS_MASTER_URI=http://192.168.124.9:11311
export ROS_IP=192.168.124.9
unset ROS_HOSTNAME

roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  dual_car_remote_task_enabled:=true \
  dual_car_comm_mode:=tcp \
  dual_car_peer_ip:=192.168.124.3
```

要求：

```text
两车不用同一个 ROS_MASTER_URI
不会出现 new node registered with same name
车 1 能通过 TCP 放行车 2
车 2 能通过 TCP 放行车 1
车 1 能通过 TCP 发送 remote_task 给车 2
车 2 收到 remote_task 后能跳过识别板一
TCP 断开不会导致 main_controller 崩溃
```

### 9.3 兼容性

```bash
dual_car_comm_mode:=ros_topic
```

如果保留兼容模式，旧 ROS 话题方式仍然可用。

如果你认为保留兼容模式会导致代码过于复杂，请先说明原因并向我确认，不要直接删除旧逻辑。

---

## 十、禁止事项

1. 不要把两车改成 ROS namespace 多机器人架构。
2. 不要修改 `/move_base`、`/amcl`、`/map_server` 的节点名。
3. 不要修改 TF frame。
4. 不要改导航航点。
5. 不要重写 `main_controller.py` 状态机。
6. 不要把裁判 TCP 上报和双车 TCP 通信混用同一个端口。
7. 不要让 TCP 连接失败导致主控退出。
8. 不要一次性修改大量文件。
9. 不要删除现有双车逻辑，除非已经确认 TCP 模式完全替代并经过测试。
10. 遇到代码与 README 不一致、box_index 来源不明确、远程任务结构不明确时，必须先提问。

---

## 十一、每个阶段完成后的输出格式

每完成一个阶段，请按以下格式输出：

```text
阶段 X 完成情况

1. 已阅读/已修改的文件
- ...

2. 本阶段修改内容
- ...

3. 未修改的内容
- ...

4. 如何测试
- ...

5. 风险和需要我确认的问题
- ...
```

如果某阶段发现问题，请不要继续下一阶段，先说明问题并等待确认。
