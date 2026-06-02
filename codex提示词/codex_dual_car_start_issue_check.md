# Codex 检查提示词：双车协作中车一发送启动信息后车二未启动运行的问题

## 1. 背景说明

当前项目为 ROS1 Melodic 智慧药房双车协作程序，功能包为 `pharmacy_mplus0`。

本次分别启动两辆车：

```bash
# 车 1（主车，先出发）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true

# 车 2（初始等待，收到放行信号后出发）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true
```

预期流程：

1. 车 1 先前往识别板一，识别到多个二维码任务。
2. 车 1 自己选择其中一个任务执行。
3. 车 1 在合适时机向车 2 发送启动或任务分配信息。
4. 车 2 收到车 1 的放行信息后，不再一直停留等待，而是开始执行自己的配送流程。

实际问题：

车 1 日志中出现了发送启动信息的记录：

```text
[01:40:21] [INFO ] [main_controller] [DualCar] sent ALLOW_START:2, peer can start while this car returns
```

但是车 2 没有进入后续导航或配送流程，只停留在初始化后的等待状态，直到手动 Ctrl+C 关闭。

请你基于代码和日志，检查为什么车 2 没有在收到车 1 的启动信息后开始运行。

---

## 2. 从终端日志得到的初步判断

### 2.1 最可疑问题：`dual_car_peer_ip` 没有配置

车 1 启动参数中显示：

```text
/main_controller/dual_car_peer_ip:
/main_controller/dual_car_peer_port: 9001
```

并且车 1 明确警告：

```text
[01:39:04] [WARN ] [main_controller] [Main] dual_car_peer_ip 为空，双车 TCP bridge 不启动，无法与对车通信
[01:39:06] [WARN ] [main_controller]   双车 TCP 模式已启用但 dual_car_peer_ip 为空，TCP 连接将无法建立
```

车 2 也有相同警告：

```text
[01:38:30] [WARN ] [main_controller] [Main] dual_car_peer_ip 为空，双车 TCP bridge 不启动，无法与对车通信
[01:38:32] [WARN ] [main_controller]   双车 TCP 模式已启用但 dual_car_peer_ip 为空，TCP 连接将无法建立
```

因此，虽然车 1 后面打印了：

```text
[DualCar] sent ALLOW_START:2
```

但这条日志很可能只是代码层面调用了发送函数，或者写入了本地状态，并不代表 TCP 消息真的已经成功发送到车 2。

请重点检查：

- `dual_car_peer_ip` 为空时，`DualCar` / TCP bridge 是否根本没有启动；
- `sent ALLOW_START` 日志是否在没有 socket 连接成功的情况下也会打印；
- 发送函数是否没有判断实际发送结果；
- launch 文件是否没有根据 `car_id` 自动填入对车 IP；
- 是否需要在启动命令中显式传入：

```bash
# 车 1 上配置车 2 的 IP
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true dual_car_peer_ip:=192.168.124.9

# 车 2 上配置车 1 的 IP
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true dual_car_peer_ip:=192.168.124.3
```

以上 IP 仅根据日志初步推测：车 1 日志中 `started roslaunch server http://192.168.124.3:39497/`，车 2 视频流使用 `http://192.168.124.9:8080/...`。请结合现场 `ip addr` 实际确认。

---

### 2.2 车 2 主控可能只进入 INIT，没有进入等待放行后的运行分支

车 2 日志中可以看到：

```text
[01:38:32] [INFO ] [main_controller] [Main] === INIT === 初始化
```

但在后续日志中没有看到类似：

```text
GOTO_BOARD1
AT_BOARD1
received ALLOW_START
remote task received
start by peer
```

也就是说，车 2 可能处于以下状态之一：

1. 车 2 的主循环在 `car_id == 2` 时主动等待远程放行；
2. 由于 TCP bridge 没有启动，所以永远收不到 `ALLOW_START`；
3. 即使收到了 `ALLOW_START`，代码中也没有把状态机从 `INIT / WAIT_REMOTE_START` 切换到 `GOTO_BOARD1` 或远程任务执行状态；
4. `ALLOW_START:2` 只表示“允许车 2 启动”，但车 2 代码实际监听的是另一种消息格式，例如 `REMOTE_TASK`、`START`、`START_TASK` 等。

请重点检查 `main_controller.py` 中：

- `car_id == 2` 的初始化流程；
- 车 2 是否默认等待远程信号；
- 远程信号回调函数是否正确注册；
- 收到 `ALLOW_START` 后是否修改了状态变量；
- 状态机是否真的会继续执行下一步；
- 是否存在变量名不一致，例如 `allow_start`、`remote_start_allowed`、`dual_car_remote_task_enabled`、`waiting_for_peer` 等。

---

### 2.3 当前启动命令没有传入 `dual_car_remote_task_enabled`

车 1 参数中显示：

```text
/main_controller/dual_car_remote_task_enabled: False
```

请检查该参数的含义。

可能存在的问题：

- 如果 `dual_car_remote_task_enabled` 控制“是否允许接收远程任务”，那么车 2 没有设置为 `true` 可能导致它忽略车 1 的任务或启动信息；
- 如果该参数只用于备用方案，可以不用改；
- 如果该参数影响车 2 是否跳过识别板一，则需要明确在 README 和 launch 参数中说明。

请检查：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true dual_car_remote_task_enabled:=true
```

是否才是车 2 等待并执行远程任务的正确启动方式。

---

### 2.4 车 2 的识别节点存在视频流短暂失败，但这可能不是“不启动”的主因

车 2 日志中出现：

```text
[ERROR] [1780421910.085542]: [Board1Detector] 无法打开视频流: http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw
[ERROR] [1780421910.587413]: [Board2Detector] 无法打开视频流: http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw
```

但后面节点又重启并显示：

```text
[INFO] [1780421916.252453]: [Board1Detector] 节点启动完成，locking 阈值: 3 帧
[INFO] [1780421916.255474]: [Board2Detector] 节点启动完成
```

因此视频流问题可能会影响识别，但它不太像是车 2 完全不启动导航的根本原因。根因仍然更像是双车 TCP 通信没有建立，或者车 2 状态机没有处理启动消息。

---

### 2.5 识别板二模板缺失不是本问题主因，但需要记录

车 1 和车 2 都出现：

```text
[Board2Detector] 模板目录 /home/EPRobot/robot_ws/src/pharmacy_mplus0/templates/board2 中未找到有效模板 (需要 idle.png / wait5.png~wait10.png)
```

该问题会导致识别板二无法正常识别，车 1 本次也在识别板二等待超时后按空闲处理：

```text
[Main] 识别板二 10s 内未识别，按空闲处理
```

但这不是车 2 没有响应 `ALLOW_START` 的主要原因。请作为附带问题修复或在 README 中标注依赖模板文件。

---

## 3. 请 Codex 重点检查的文件

请优先检查以下文件，具体路径以仓库实际结构为准：

```text
pharmacy_mplus0/launch/race_bringup.launch
pharmacy_mplus0/scripts/main_controller.py
pharmacy_mplus0/scripts/dual_car_bridge.py
pharmacy_mplus0/scripts/tcp_reporter.py
pharmacy_mplus0/README_DUAL_CAR.md
pharmacy_mplus0/README_COMMUNICATION.md
```

如果没有 `dual_car_bridge.py`，请在 `main_controller.py` 或其他通信相关文件中搜索以下关键词：

```text
dual_car
peer_ip
peer_port
listen_port
ALLOW_START
REMOTE_TASK
remote_task
start_allowed
socket
TCP
send
recv
callback
```

---

## 4. 请 Codex 执行的检查任务

### 4.1 检查 TCP bridge 是否真的启动

请确认：

1. `dual_car_enabled == true` 但 `dual_car_peer_ip == ""` 时，程序是否直接不启动通信线程；
2. 本车监听端口 `0.0.0.0:9001` 是否仍然启动；
3. 是否需要即使 peer_ip 为空，也先启动 server 监听，等待对车连接；
4. 当前代码是双端主动连接、双端监听，还是只允许一端连接另一端；
5. 两车同时监听 `9001` 是否会冲突。正常情况下不同机器同端口不冲突，但如果在同一 ROS master 或同一主机测试，需要注意。

### 4.2 检查 `sent ALLOW_START` 日志是否可信

请确认：

1. 这条日志是在 `socket.send()` 成功后打印，还是只要调用发送函数就打印；
2. 如果 TCP bridge 没启动或 socket 未连接，是否仍然会打印 `sent ALLOW_START`；
3. 发送失败时是否有异常被吞掉；
4. 是否应该将日志改成：
   - `try send ALLOW_START`
   - `send ALLOW_START success`
   - `send ALLOW_START failed: reason`

建议修改为只有实际发送成功才打印 `sent`。

### 4.3 检查车 2 收到消息后的状态机转换

请确认车 2 收到 `ALLOW_START:2` 后是否会：

1. 确认消息目标是本车 `car_id == 2`；
2. 设置允许启动标志；
3. 从等待状态进入下一阶段；
4. 如果没有远程任务内容，是否默认进入 `GOTO_BOARD1`；
5. 如果已经收到剩余二维码任务，是否跳过识别板一并直接进入配送流程。

建议增加明确日志：

```text
[DualCar] received raw message: ALLOW_START:2
[DualCar] ALLOW_START accepted for car_id=2
[Main] remote start allowed, transition INIT/WAIT_REMOTE_START -> GOTO_BOARD1
```

### 4.4 检查消息格式是否一致

请检查发送端和接收端是否使用同一种格式。

车 1 当前发送日志为：

```text
ALLOW_START:2
```

请确认接收端是否真的解析这种格式，而不是等待：

```text
START
START:2
ALLOW_START
ALLOW_START car_id=2
REMOTE_TASK:{...}
TASK_ASSIGN:{...}
```

如果格式不一致，请统一协议，并在 README 中写清楚。

### 4.5 检查启动参数和 README 是否完整

请检查 `race_bringup.launch` 是否声明并传递了以下参数：

```xml
<arg name="dual_car_enabled" default="false" />
<arg name="dual_car_listen_ip" default="0.0.0.0" />
<arg name="dual_car_listen_port" default="9001" />
<arg name="dual_car_peer_ip" default="" />
<arg name="dual_car_peer_port" default="9001" />
<arg name="dual_car_remote_task_enabled" default="false" />
```

并确认这些参数是否正确传入 `main_controller.py`。

请同步修改 README，写明双车启动命令，例如：

```bash
# 车 1：对车 IP 填车 2 的 wlan0/eth0 地址
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_peer_ip:=192.168.124.9 \
  dual_car_peer_port:=9001

# 车 2：对车 IP 填车 1 的 wlan0/eth0 地址
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  dual_car_peer_ip:=192.168.124.3 \
  dual_car_peer_port:=9001 \
  dual_car_remote_task_enabled:=true
```

如果不需要 `dual_car_remote_task_enabled:=true`，请在 README 中明确说明该参数的真实用途。

---

## 5. 建议的修复方向

请优先采用“最小侵入式修复”，不要大规模重构状态机。

### 5.1 参数层修复

至少保证两车都能配置对车 IP：

- 车 1 的 `dual_car_peer_ip` 指向车 2；
- 车 2 的 `dual_car_peer_ip` 指向车 1；
- README 中给出 `ip addr`、`ping`、`nc` 或 `telnet` 的检查方法。

### 5.2 日志层修复

增加更明确的通信日志：

```text
[DualCar] listen on 0.0.0.0:9001
[DualCar] connecting to peer 192.168.124.x:9001
[DualCar] peer connected
[DualCar] send success: ALLOW_START:2
[DualCar] received: ALLOW_START:2
[DualCar] car_id matched, start allowed
```

如果连接失败，必须输出原因：

```text
[DualCar] connect failed: Connection refused
[DualCar] send failed: no active socket
```

### 5.3 状态机层修复

车 2 在 `car_id == 2` 且 `dual_car_enabled == true` 时，可以采用如下逻辑：

1. 启动后进入 `WAIT_REMOTE_START`；
2. 如果收到 `ALLOW_START:2`，进入 `GOTO_BOARD1`；
3. 如果收到完整远程任务，例如 `REMOTE_TASK`，则跳过识别板一，直接进入对应配送流程；
4. 如果长时间没有收到消息，可以保持等待，不要误启动；
5. 所有状态切换都必须有日志。

### 5.4 协议层修复

建议把消息格式封装成统一函数，避免字符串散落：

```python
def make_allow_start_msg(target_car_id):
    return "ALLOW_START:{}".format(target_car_id)


def parse_dual_car_msg(raw):
    # 返回 msg_type, target_car_id, payload
    pass
```

如果当前已有协议类，请只修复不一致的地方，不要重复造轮子。

---

## 6. 请 Codex 输出的结果

请完成检查后输出：

1. 根因判断：车 2 不启动的直接原因和间接原因；
2. 涉及文件列表；
3. 需要修改的代码点；
4. 修改后的完整代码文件；
5. 修改后的 README 相关章节；
6. 重新测试命令；
7. 预期正确日志。

---

## 7. 推荐重新测试步骤

修改后请按以下顺序测试。

### 7.1 网络连通性测试

在车 1：

```bash
ping 192.168.124.9
nc -vz 192.168.124.9 9001
```

在车 2：

```bash
ping 192.168.124.3
nc -vz 192.168.124.3 9001
```

如果没有 `nc`，可安装或用 Python socket 简单测试。

### 7.2 ROS 参数检查

车 1：

```bash
rosparam get /main_controller/dual_car_peer_ip
rosparam get /main_controller/dual_car_peer_port
```

车 2：

```bash
rosparam get /main_controller/dual_car_peer_ip
rosparam get /main_controller/dual_car_peer_port
rosparam get /main_controller/dual_car_remote_task_enabled
```

### 7.3 正常启动命令

```bash
# 车 1
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_peer_ip:=192.168.124.9

# 车 2
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  dual_car_peer_ip:=192.168.124.3 \
  dual_car_remote_task_enabled:=true
```

### 7.4 预期日志

车 1 应看到：

```text
[DualCar] connecting to peer 192.168.124.9:9001
[DualCar] peer connected
[DualCar] send success: ALLOW_START:2
```

车 2 应看到：

```text
[DualCar] listen on 0.0.0.0:9001
[DualCar] peer connected from 192.168.124.3
[DualCar] received raw message: ALLOW_START:2
[DualCar] ALLOW_START accepted for car_id=2
[Main] remote start allowed, transition WAIT_REMOTE_START -> GOTO_BOARD1
```

如果车 2 后续设计为跳过识别板一，则应看到：

```text
[Main] received remote task, skip board1
[Main] === GOTO_EXAM === ...
```

---

## 8. 当前初步结论

从本次日志看，最可能的根因是：

1. 两车都没有配置 `dual_car_peer_ip`；
2. 日志明确提示 `双车 TCP bridge 不启动，无法与对车通信`；
3. 车 1 虽然打印了 `sent ALLOW_START:2`，但该日志很可能没有严格绑定真实 TCP 发送成功；
4. 车 2 日志中完全没有收到 `ALLOW_START` 的记录，也没有进入 `GOTO_BOARD1` 或远程任务执行状态；
5. 因此应优先检查并修复双车 TCP 参数传递、连接建立、发送成功判断、接收回调和状态机切换。

请 Codex 按以上方向检查代码并给出最小可用修复。
