# Codex 提示词：简化 `main_controller.py` 中的双车协作逻辑

## 任务背景

你现在需要阅读并修改 ROS1 Python 节点 `main_controller.py`。这是智慧药房小车的主控状态机代码，目前代码中已经实现了单车任务流程和双车协作流程，但双车逻辑较复杂，包含了 `ALLOW_START` 交替放行、`remote_task` 远程任务分配、`qr_task` 占用同步等多套机制。

我希望你在不破坏单车主流程的前提下，将双车协作逻辑简化为当前方案：

```text
车1负责识别板一识别与任务分配；
车2只等待车1发送的 remote_task，然后直接执行配送流程。
```

---

## 一、目标双车方案

双车模式下只保留一种核心协作逻辑：

```text
车1：
    1. 正常启动
    2. 前往识别板一
    3. 识别多个二维码任务
    4. 选择一个任务给自己执行
    5. 如果还有剩余有效任务，则选择一个任务发送给车2
    6. 自己进入 GOTO_EXAM，继续完成体检、识别板二、化验投递流程

车2：
    1. 启动后停在起点等待
    2. 不主动前往识别板一
    3. 等待车1通过 TCP 发送 remote_task
    4. 收到 remote_task 后，直接构建 RoundPlan
    5. 跳过 GOTO_BOARD1 和 AT_BOARD1
    6. 直接进入 GOTO_EXAM，执行体检、识别板二、化验投递流程
```

两车配送部分共用原来的状态机：

```text
GOTO_EXAM
→ AT_EXAM
→ GOTO_BOARD2
→ AT_BOARD2
→ PASS_BOARD2
→ GOTO_LAB
→ AT_LAB
→ DONE_ROUND
```

---

## 二、需要保留的逻辑

请保留以下内容：

### 1. 单车模式完整逻辑

当 `dual_car_enabled = false` 时，代码行为应与原来保持一致。

### 2. 导航逻辑

继续使用：

```python
NavigationClient.go_to()
NavigationClient.clear_costmaps()
NavigationClient.cancel()
```

### 3. 识别板一接收逻辑

继续支持以下话题：

```text
/cam_return
/board1_detections
```

对应回调逻辑可以保留：

```python
_cb_cam_return()
_cb_board1_detections()
```

### 4. 识别板二逻辑

继续支持以下话题：

```text
/cv1_result
/board2_status
```

对应回调逻辑可以保留：

```python
_cb_cv1_result()
_cb_board2_status()
```

### 5. 任务规划逻辑

继续使用：

```python
TaskPlanner.select_best()
TaskPlanner.build_round_plan()
```

### 6. 比赛状态发布逻辑

继续使用 `CompetitionIO`，包括但不限于：

```python
set_task()
set_task_road()
publish_cv1()
publish_cv2()
announce_exam_samples()
announce_board2()
announce_lab_arrival()
```

### 7. 样本记录逻辑

继续使用：

```python
SampleStore
```

### 8. TCP 通信桥接

继续使用：

```python
DualCarTcpBridge
```

---

## 三、需要简化或删除的逻辑

请重点简化以下内容。

---

### 1. 删除或弱化 `ALLOW_START` 交替放行逻辑

当前代码中存在：

```python
_dual_publish_allow_peer_start()
_dual_handle_allow_start_signal()
```

以及 `allow_start` 类型 TCP 消息。

在新的方案中，车2的启动不再依赖普通 `ALLOW_START`，而是依赖 `remote_task`。

因此：

- 可以删除 `allow_start` 消息处理逻辑；
- 可以不再在 `_do_at_lab()` 中调用 `_dual_publish_allow_peer_start()`；
- 可以删除或停用 `_dual_allow_sent_this_round`；
- 可以删除或停用 `_dual_waiting_at_start` 中与 `ALLOW_START` 相关的复杂判断；
- 保留“车2等待 `remote_task` 时发布零速度停车”的安全逻辑。

---

### 2. 删除二维码占用同步逻辑

当前代码中有：

```python
_dual_publish_qr_task()
_dual_handle_peer_qr_task_payload()
_dual_handle_peer_qr_task_clear()
_peer_occupied_box
```

新方案中不需要对车之间同步二维码占用，因为车1统一识别和分配任务。

因此：

- 可以删除 `qr_task` 和 `qr_task_clear` 消息处理逻辑；
- 可以删除 `_peer_occupied_box`；
- `TaskPlanner.select_best()` 不再需要传入 `excluded_box=self._peer_occupied_box`；
- 车1选择自己的任务后，再从剩余检测结果中选择任务发给车2。

---

### 3. 保留并强化 `remote_task` 逻辑

当前代码中已经有：

```python
_dual_publish_remote_task_for_peer()
_dual_handle_remote_task_data()
_dual_apply_remote_task_if_available()
```

请以这三个函数为核心重构双车逻辑。

要求：

- 车1在 `AT_BOARD1` 识别到多个二维码后：
  - 先选择自己的 `best`；
  - 再从剩余任务中选择一个 `peer_task`；
  - 通过 TCP 发送给车2。

- 车2收到 `remote_task` 后：
  - 检查 `target_car` 是否为自己；
  - 检查 `from_car` 是否不是自己；
  - 检查 `task` 字段是否合法；
  - 缓存到 `_dual_assigned_remote_task`；
  - 设置允许执行标志；
  - 在状态机中直接进入 `GOTO_EXAM`。

---

## 四、期望最终状态逻辑

---

### 1. `_do_goto_board1()` 期望逻辑

请将 `_do_goto_board1()` 简化为类似下面的逻辑：

```python
def _do_goto_board1(self):
    # 双车模式下，车2不去识别板一，只等待车1分配 remote_task
    if self._dual_car_enabled and self._car_id == "2":
        if self._dual_apply_remote_task_if_available():
            return

        loginfo("[Main] 车2等待车1分配 remote_task ...")
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(0.1)
        return

    # 单车模式或车1模式：正常前往识别板一
    self._dual_remote_task_sent_this_round = False

    loginfo("[Main] === GOTO_BOARD1 (即将进入第 %d 轮) ===",
            self._round_index + 1)
    self._io.set_task_road()
    self._board1_detections = []
    self._board2_wait = None
    self._store.clear()
    self._reset_pub.publish(String())

    ok = self._nav.go_to("board1", timeout_sec=self._nav_timeout)
    self._nav.clear_costmaps()
    if ok:
        self._round_index += 1
        loginfo("[Main] 第 %d 轮开始", self._round_index)
        self._state = STATE_AT_BOARD1
    else:
        logwarn("[Main] 前往识别板一失败，保持状态自动重试")
```

最终行为应为：

```text
车1：正常去识别板一
车2：不去识别板一，只等待 remote_task
单车：正常去识别板一
```

---

### 2. `_do_at_board1()` 期望逻辑

车1或单车到达识别板一后：

```text
等待二维码识别
→ 选择 best 作为本车任务
→ 构建本车 RoundPlan
→ 发布 cv2
→ 如果双车启用且本车是车1：
       从剩余任务中选择 peer_task
       发送 remote_task 给车2
→ 进入 GOTO_EXAM
```

注意：

- 单车模式不发送 `remote_task`；
- 车2不应该进入 `AT_BOARD1`；
- 如果只有一个有效二维码，则车1自己执行，不给车2发任务；
- `remote_task` 发送失败不能影响车1继续执行自己的任务。

---

### 3. `_dual_handle_tcp_message()` 期望逻辑

只需要处理：

```python
remote_task
ack  # 可选保留，仅日志
```

可以删除：

```python
allow_start
qr_task
qr_task_clear
```

示例结构：

```python
def _dual_handle_tcp_message(self, payload):
    msg_type = payload.get("type", "")
    from_car = str(payload.get("from_car", "?"))
    loginfo("[DualCar] TCP recv: type=%s from_car=%s", msg_type, from_car)

    if msg_type == "remote_task":
        self._dual_handle_remote_task_data(payload)
    elif msg_type == "ack":
        loginfo(
            "[DualCar] received ACK: ack_type=%s status=%s from car %s",
            payload.get("ack_type", ""),
            payload.get("status", ""),
            payload.get("from_car", ""),
        )
    else:
        logwarn("[DualCar] unknown TCP message type: %s", msg_type)
```

---

### 4. `_do_at_lab()` 期望逻辑

不再放行对车。

原逻辑中：

```python
self._dual_publish_allow_peer_start()
```

请删除或注释掉。

化验窗口逻辑应变为：

```text
set_task(lab_win)
deliver_to_lab(lab_win)
announce_lab_arrival(lab_win, count)
sleep(lab_dwell)
set_task_road()
state = DONE_ROUND
```

---

### 5. `_do_done_round()` 期望逻辑

不再发布二维码占用清空信息。

原逻辑中：

```python
self._dual_publish_qr_task("", "")
```

请删除或注释掉。

车1完成一轮后，可以继续回到 `GOTO_BOARD1` 开始下一轮。

车2完成一轮后，回到 `GOTO_BOARD1`，但由于车2在 `_do_goto_board1()` 中会等待 `remote_task`，所以它会停在起点等待下一次车1分配任务。

---

## 五、配置参数建议

请尽量减少双车配置参数。

建议保留：

```yaml
dual_car_enabled: true
dual_car_remote_task_enabled: true
```

其中：

```text
car_id = "1" 的小车负责识别和分配任务；
car_id = "2" 的小车负责等待并执行远程任务。
```

请检查代码中是否还残留无用配置，例如：

```python
_dual_allow_sent_this_round
_dual_start_allowed
_dual_waiting_at_start
_peer_occupied_box
```

如果不再需要，请删除。

---

## 六、代码修改要求

1. 请直接修改 `main_controller.py`。
2. 保持原有代码风格。
3. 保持 Python 2 / ROS Melodic 兼容性。
4. 不要引入新的第三方库。
5. 不要重构无关模块。
6. 不要修改 `NavigationClient`、`CompetitionIO`、`SampleStore`、`TaskPlanner` 的外部接口，除非确实必要。
7. 修改后请输出完整的 `main_controller.py` 文件内容，方便我直接替换。
8. 修改完成后，请额外给出一份简短说明，说明：
   - 删除了哪些旧逻辑；
   - 保留了哪些核心双车逻辑；
   - 车1启动后的流程；
   - 车2启动后的流程；
   - 需要在 launch 或 yaml 中确认的参数。

---

## 七、特别注意

请重点检查以下边界情况：

1. 单车模式不能受影响。
2. 车2没有收到 `remote_task` 时必须原地停车等待。
3. 车2收到非法 `remote_task` 时不能进入错误状态。
4. 车1发送 `remote_task` 失败时，车1仍然继续执行自己的任务。
5. 如果识别板一只识别到一个二维码，车1自己执行即可，不需要发给车2。
6. `DONE_ROUND` 后：
   - 车1进入下一轮识别；
   - 车2回到等待 `remote_task` 状态。
7. `shutdown` 时仍然要停车、取消导航、停止 TCP bridge。

---

## 八、建议修改后的双车运行逻辑说明

修改完成后，双车整体逻辑应清晰变为：

```text
启动双车
│
├── 车1 car_id=1
│   │
│   ├── 去识别板一
│   ├── 识别二维码列表
│   ├── 选择自己的任务
│   ├── 如果还有剩余任务：
│   │       发送 remote_task 给车2
│   └── 自己进入 GOTO_EXAM
│
└── 车2 car_id=2
    │
    ├── 起点等待 remote_task
    ├── 收到 remote_task
    ├── 构建 round_plan
    └── 直接进入 GOTO_EXAM
```

两车后续配送流程一致：

```text
GOTO_EXAM
→ AT_EXAM
→ GOTO_BOARD2
→ AT_BOARD2
→ PASS_BOARD2
→ GOTO_LAB
→ AT_LAB
→ DONE_ROUND
```
