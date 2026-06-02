# 双车协作 — 架构说明与使用指南

> 本文档涵盖双车循环协作的设计思路、已实现的接口、启用方法、运行命令和调试方法。

---

## 1. 设计思路

比赛场地有两辆小车同时运行。双车协作采用 **"轮流出发 + 任务占用排除 + 远程任务共享"** 机制：

- **轮流出发**：比赛开始时限一辆车出发，另一辆在起点等待。完成配送的车在返回起点途中放行对车。
- **任务占用排除**：每辆车选定识别板一二维码后立即广播占用，对车选择任务时排除已被占用的方框。
- **远程任务共享**：车 1 在识别板一扫描到多个二维码时，自己选择一个任务配送，将剩余任务通过 JSON 发送给车 2。车 2 收到远程任务后直接跳过识别板一流程，从体检区开始配送。**已移除时间戳过期限制**，远程任务不再因 `stamp` 过旧而被丢弃。

核心原则：**单车模式不受任何影响**，所有双车功能默认关闭，仅传入 `dual_car_enabled:=true` 时才启用。远程任务共享需额外传入 `dual_car_remote_task_enabled:=true`。

---

## 2. 双车话题拓扑

```
                      ┌──────────────────────────────────┐
                      │           识别板一                │
                      │   box0    box1    box2    box3    │
                      └────┬───────┬───────┬───────┬──────┘
                           │                       │
                           ▼                       ▼
                  车1 board1_detector      车2 board1_detector
                           │                       │
                      ┌────┴────┐             ┌────┴────┐
                      │ /cam_return │           │ /cam_return │
                      │ /all_qrcodes│           │ /all_qrcodes│
                      │ /board1_   │           │ /board1_   │
                      │  detections│           │  detections│
                      └───────────┘             └───────────┘
                           │                       │
                           ▼                       ▼
                  车1 main_controller      车2 main_controller
                           │                       │
                      ┌────┴────┐             ┌────┴────┐
                      │ /current_task│          │ /current_task│
                      │ /cv2_result │          │ /cv2_result │
                      │ /current_qr_task │     │ /current_qr_task │
                      │ /dual_car_signal │    │ /dual_car_signal │
                      └───────────┘             └───────────┘
                           │                       │
                           └──────────┬────────────┘
                                      │
                         互相订阅 /current_qr_task（任务占用）
                         互相订阅 /dual_car_signal（轮流出发）
```

| 话题 | 发布者 | 内容示例 | 双车用途 |
|------|--------|---------|---------|
| `/dual_car_signal` | main_controller | `ALLOW_START:2` 或 JSON | 放行对车出发 / 发送远程任务 |
| `/current_qr_task` | main_controller | `CAR1:AB-1` 或 `CAR1:` | 广播自己的任务占用，订阅对车的占用 |
| `/all_qrcodes` | board1_detector | `[{"code":"AB","box":0,...}]` | 共享全部二维码结果（调试用） |

`/dual_car_signal` 支持两种格式：
- **简单放行**：`ALLOW_START:<id>` — 仅放行对车出发
- **远程任务共享（JSON）**：
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
  `stamp` 仅用于日志/调试，**不再作为任务丢弃条件**。两种格式完全兼容，回调会自动识别。

---

## 3. 双车循环流程

```
比赛开始：
  car_id=1 允许先出发（dual_car_start_first_car_id）
  car_id=2 必须在起点等待

第 1 轮：
  车 1 执行配送任务（识别板一 → 体检取样 → 识别板二 → 化验投递）
  车 2 在起点等待，持续发布零速度
  车 1 完成配送，进入 DONE_ROUND 时发布 ALLOW_START:2
  车 1 继续返回起点
  车 2 收到 ALLOW_START:2 → 开始前往识别板一

第 2 轮：
  车 2 执行配送任务
  车 1 回到起点后等待
  车 2 完成配送，进入 DONE_ROUND 时发布 ALLOW_START:1
  车 1 收到 ALLOW_START:1 → 开始第 3 轮

之后循环：车 1 → 车 2 → 车 1 → 车 2 → ...

**远程任务共享流程（启用时）：**

```
第 1 轮：
  车 1 前往识别板一，扫描到多个二维码（如 ABC 在 box2、C 在 box1）
  车 1 选择最优任务（ABC box2）自己执行
  车 1 将剩余任务（C box1）打包为 JSON 发送到 /dual_car_signal
  车 1 继续正常配送流程（体检取样 → 识别板二 → 化验投递）
  车 2 收到 JSON 远程任务 → 缓存任务 → 跳过识别板一
  车 2 直接从远程任务进入体检区配送流程
  车 1 完成配送后发布 ALLOW_START:2（与基础双车逻辑一致）
```
```

关键时序：
- **放行信号发送时机**：本车完成配送（`_do_at_lab` 投递完成）后，在 `_do_done_round` 开头立即发送，**不等本车回到起点**。
- **本车放行后行为**：`_dual_start_allowed` 置 `False`，本车下一轮回到起点后会进入等待。
- **对车收到放行后**：可立即出发前往识别板一，与本车返回起点同步进行。

---

## 4. 已实现的接口清单

### 4.1 配置项（`config/strategy.yaml`）

```yaml
# 双车循环协作配置（默认关闭，不影响单车模式）。
dual_car_enabled: false              # 是否启用双车循环协作
dual_car_start_first_car_id: "1"     # 开局允许先出发的车号
dual_car_signal_topic: "/dual_car_signal"  # 放行信号话题名
dual_car_publish_allow_when_returning: true  # 完成后是否放行对车

# 远程任务共享（默认关闭，需 dual_car_enabled:=true 配合使用）。
dual_car_remote_task_enabled: false  # 是否启用远程任务共享
# 注意：已删除 dual_car_remote_task_max_age_sec 配置项。
# 远程任务不再因时间戳过期而被丢弃，有效性仅由字段合法性判断。
```

### 4.2 常量（`src/pharmacy_mplus0/constants.py`）

| 常量 | 值 | 说明 |
|------|-----|------|
| `TOPIC_DUAL_CAR_SIGNAL` | `"/dual_car_signal"` | 双车轮流出发信号话题 |
| `TOPIC_CURRENT_QR_TASK` | `"/current_qr_task"` | 任务占用广播话题 |

### 4.3 CompetitionIO（`src/pharmacy_mplus0/competition_io.py`）

| 方法 | 说明 |
|------|------|
| `__init__(car_id="1")` | 构造函数接收车号 |
| `publish_dual_signal(text)` | 发布双车信号，如 `ALLOW_START:2` |
| `publish_qr_task(code, lab_window)` | 发布任务占用，格式 `CAR<id>:<code>-<lab_window>` |

### 4.4 TaskPlanner（`src/pharmacy_mplus0/task_planner.py`）

| 方法 | 说明 |
|------|------|
| `select_best(detections, excluded_box=None)` | 选择最优任务，可排除被对车占用的方框 |

### 4.5 MainController（`scripts/main_controller.py`）

| 状态变量 | 说明 |
|------|------|
| `_dual_car_enabled` | 是否启用双车模式 |
| `_car_id` | 当前车编号（来自 launch 参数） |
| `_dual_waiting_at_start` | 是否在起点等待对车放行 |
| `_dual_start_allowed` | 是否已获得本轮出发令牌 |
| `_dual_allow_sent_this_round` | 本轮是否已向对车发送放行信号 |
| `_peer_occupied_box` | 对车占用的方框索引，None 表示无占用 |
| `_dual_remote_task_enabled` | 是否启用远程任务共享 |
| `_dual_assigned_remote_task` | 缓存的对车分配远程任务（dict 或 None） |
| `_dual_remote_task_sent_this_round` | 本轮是否已向对车发送远程任务 |

| 方法 | 说明 |
|------|------|
| `_dual_peer_id()` | 返回对车编号：1→2, 2→1 |
| `_cb_dual_car_signal(msg)` | 接收并解析 `/dual_car_signal`，自动识别简单格式和 JSON |
| `_dual_try_parse_remote_task(body)` | 安全解析 JSON 远程任务，校验字段合法性（不校验 stamp 过期） |
| `_dual_publish_allow_peer_start()` | 完成配送后发布 `ALLOW_START:<peer_id>` |
| `_dual_publish_remote_task_for_peer(task)` | 将剩余任务打包为 JSON 通过 `/dual_car_signal` 发送 |
| `_dual_apply_remote_task_if_available()` | 检查并应用缓存的远程任务，跳过识别板一，返回 True/False |
| `_cb_peer_qr_task(msg)` | 解析 `/current_qr_task`，提取对车占用的 box_index |

### 4.6 Launch 文件

| 文件 | 新增参数 | 说明 |
|------|---------|------|
| `launch/main.launch` | `dual_car_enabled` | 传递给 `main_controller` |
| `launch/race_bringup.launch` | `dual_car_enabled` | 透传到 `main.launch` |
| `launch/main.launch` | `car_id` → `main_controller` param | 主控节点获得车号 |

---

## 5. 启用方法

### 5.1 双车模式

```bash
# 车 1（主车，先出发）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true

# 车 2（初始等待，收到放行信号后出发，各自播报）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true
```

### 5.2 双车模式 + 远程任务共享

```bash
# 车 1（主车，先出发，扫描到多个二维码时发送剩余任务给车 2）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true dual_car_remote_task_enabled:=true

# 车 2（收到远程任务后跳过识别板一，直接进入体检区配送）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true dual_car_remote_task_enabled:=true
```

### 5.3 单车模式（默认，不受影响）

```bash
# 不传 dual_car_enabled 或传 false
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1
```

### 5.4 覆盖开局首发车

```bash
# 让车 2 首发
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true dual_car_start_first_car_id:=2
```

---

## 6. 调试方法

### 6.1 监控双车信号

```bash
# 实时查看放行信号
rostopic echo /dual_car_signal

# 预期输出节奏：
# 车 1 完成配送 → data: "ALLOW_START:2"
# 车 2 完成配送 → data: "ALLOW_START:1"
```

### 6.2 监控任务占用

```bash
# 查看任务占用广播（新格式 CAR<id>:...）
rostopic echo /current_qr_task

# 预期：
# 车 1 到达识别板一 → data: "CAR1:AB-1"
# 车 1 本轮结束     → data: "CAR1:"
# 车 2 到达识别板一 → data: "CAR2:ABC-2"
```

### 6.3 手动模拟放行（调试用）

```bash
# 手动放行车 2
rostopic pub /dual_car_signal std_msgs/String "data: 'ALLOW_START:2'" -1

# 手动放行车 1
rostopic pub /dual_car_signal std_msgs/String "data: 'ALLOW_START:1'" -1
```

### 6.4 模拟对车任务占用（调试用）

```bash
# 模拟车 1 占用 box=0
rostopic pub /current_qr_task std_msgs/String "data: 'CAR1:AB-1'" -r 2

# 模拟清空
rostopic pub /current_qr_task std_msgs/String "data: 'CAR1:'" -r 2
```

### 6.5 手动模拟远程任务（调试用）

```bash
# 模拟车 1 向车 2 发送远程任务（车 2 收到后应跳过识别板一）
rostopic pub /dual_car_signal std_msgs/String "data: '{\"type\":\"ALLOW_START_WITH_TASK\",\"from_car\":\"1\",\"target_car\":\"2\",\"stamp\":1760000000.123,\"task\":{\"code\":\"C\",\"box\":2,\"lab_window\":\"3\",\"sample_count\":1}}'" -1

# 模拟车 2 向车 1 发送远程任务
rostopic pub /dual_car_signal std_msgs/String "data: '{\"type\":\"ALLOW_START_WITH_TASK\",\"from_car\":\"2\",\"target_car\":\"1\",\"stamp\":1760000000.456,\"task\":{\"code\":\"AB\",\"box\":0,\"lab_window\":\"1\",\"sample_count\":2}}'" -1
```

### 6.6 监控关键话题

```bash
# 实时查看双车信号（含 JSON 远程任务）
rostopic echo /dual_car_signal

# 查看任务占用广播
rostopic echo /current_qr_task

# 查看当前任务状态
rostopic echo /current_task
```

### 6.8 终端仪表盘

```bash
rosrun pharmacy_mplus0_debug topic_echo_dashboard.py
```

仪表盘实时显示 `/current_qr_task`、`/dual_car_signal`、`/current_task`、`/cv2_result` 等关键话题。

### 6.9 预期双车循环日志

```
# 车 1 启动
[Main] 双车协作: 启用 (car_id=1)
[DualCar] car_id=1 start allowed at boot

# 车 2 启动
[Main] 双车协作: 启用 (car_id=2)
[DualCar] car_id=2 waiting at start

# 车 1 完成第一轮
[Main] === DONE_ROUND === 本轮结束
[DualCar] sent ALLOW_START:2, peer can start while this car returns

# 车 2 收到信号
[DualCar] received ALLOW_START:2, this car can start next round

# 车 2 完成第一轮
[Main] === DONE_ROUND === 本轮结束
[DualCar] sent ALLOW_START:1, peer can start while this car returns

# 车 1 收到信号
[DualCar] received ALLOW_START:1, this car can start next round

# ... 循环 ...
```

### 6.10 离线验证排除逻辑

```bash
cd ~/robot_ws/src/pharmacy_mplus0
python -c "
from pharmacy_mplus0.task_planner import TaskPlanner
from pharmacy_mplus0.models import make_board1_detection

p = TaskPlanner()
dets = [
    make_board1_detection('AB', 0),   # box=0 → lab_window=1
    make_board1_detection('C', 1),    # box=1 → lab_window=2
    make_board1_detection('ABC', 2),  # box=2 → lab_window=3
]
# 无排除：ABC(box2) 样本最多优先
best = p.select_best(dets)
print('无排除:', best.code, 'box', best.box_index)  # ABC box 2

# 排除 box=2：剩下 AB(box0) 和 C(box1)，AB 样本更多
best = p.select_best(dets, excluded_box=2)
print('排除box2:', best.code, 'box', best.box_index)  # AB box 0
"
```

预期输出：
```
无排除: ABC box 2
排除box2: AB box 0
```

---

## 7. 边界情况与注意事项

| 场景 | 预期行为 |
|------|---------|
| 单车运行（dual_car_enabled=false） | 所有双车逻辑跳过，原有流程不受影响 |
| 对车未启动 | 本车 `/current_qr_task` 只有自己发布，`_peer_occupied_box` 始终为 None |
| 对车未到识别板一（发布 `CAR2:`） | `_peer_occupied_box` 被清为 None，本车正常选择 |
| 两车同时到识别板一 | 各自锁定不同方框后自然错开 |
| 所有方框都被对车占用 | `select_best` 返回 None → 进入 DONE_ROUND → 返回起点再试 |
| 对车崩溃 / 不再发布消息 | `_peer_occupied_box` 保持最后值，下一轮自动被本车清空任务时清除 |
| 收到发给对车的 ALLOW_START | 回调中 `target_id != self._car_id`，直接忽略 |
| 配送途中收到 ALLOW_START | `_dual_waiting_at_start` 为 False，回调忽略 |
| 收到旧格式 `/current_qr_task`（无 CAR 前缀） | 双车模式下忽略（无法区分来源），单车模式下正常处理 |
| 语音播报冲突 | 两车轮流出发，配送时段不重叠，各自播报即可；如需禁用语音仍可通过 `audio_dir:=""` 关闭 |
| `round_return_to_start: false` | 双车模式依赖回起点来保证节奏，建议保持 `true` |
| 远程任务无 `stamp` | 仍可接受，只要字段合法 |
| 远程任务 `stamp` 很旧 | 仍可接受，不再超时丢弃 |
| 远程任务字段缺失 | 忽略并回退正常识别板一流程 |
| 车 2 收到远程任务但未被放行 | 以远程任务 JSON 为准，可视为同时放行 |
| 车 2 使用远程任务 | 跳过识别板一，直接进入体检区配送流程 |
| 本轮重复收到远程任务 | 优先使用第一个合法任务；若已开始执行则忽略新任务 |
| `dual_car_remote_task_enabled=false` | 走原有双车方案（轮流出发 + 任务占用排除），车 2 正常去识别板一 |

---

## 8. 相关文件索引

```
pharmacy_mplus0/
├── README_DUAL_CAR.md              ← 本文件
├── config/
│   └── strategy.yaml               ← dual_car_enabled 等配置
├── launch/
│   ├── main.launch                 ← car_id / dual_car_enabled 参数
│   ├── main_single.launch          ← 可选静默入口（调试用）
│   └── race_bringup.launch         ← 全量启动，暴露 dual_car_enabled
├── scripts/
│   ├── main_controller.py          ← ★ 双车核心调度逻辑
│   └── board1_detector.py          ← /all_qrcodes 发布
└── src/pharmacy_mplus0/
    ├── constants.py                ← 话题名定义
    ├── models.py                   ← Board1Detection 数据结构
    ├── task_planner.py             ← select_best(excluded_box=...) 已实现
    ├── competition_io.py           ← publish_qr_task / publish_dual_signal
    └── log_utils.py                ← 中文安全日志封装

pharmacy_mplus0_debug/
└── scripts/
    ├── topic_echo_dashboard.py     ← 监控 /current_qr_task + /dual_car_signal
    └── tcp_fake_server.py          ← 按 car= 区分两车上报
```

---

## 9. 远程任务共享 — 已实现

### 9.1 功能概述

识别板一每次可能出现 2 个二维码。启用远程任务共享后：

1. **车 1** 在识别板一扫描到多个二维码 → 用 `TaskPlanner.select_best()` 选择本车任务 → 将剩余任务打包为 JSON 发送到 `/dual_car_signal`
2. **车 2** 收到 JSON 远程任务 → 缓存并解析 → 跳过识别板一导航和识别流程 → 直接进入体检区配送

### 9.2 配置

在 `config/strategy.yaml` 中：

```yaml
# 远程任务共享（默认关闭）
dual_car_remote_task_enabled: false
```

也可通过 launch 参数覆盖：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true dual_car_remote_task_enabled:=true
```

**注意**：`dual_car_remote_task_max_age_sec` 配置项已删除。远程任务不再因时间戳过期被丢弃。

### 9.3 关键实现

在 `scripts/main_controller.py` 中：

| 变量/方法 | 说明 |
|------|------|
| `_dual_remote_task_enabled` | 配置开关，默认 `False` |
| `_dual_assigned_remote_task` | 缓存的对车分配任务（dict），消费后清空 |
| `_dual_remote_task_sent_this_round` | 本轮是否已发送远程任务，避免重复发送 |
| `_dual_try_parse_remote_task(body)` | JSON 信号安全解析，校验字段合法性，不校验 stamp 过期 |
| `_dual_publish_remote_task_for_peer(task)` | 车 1 将剩余任务打包为 JSON 发送 |
| `_dual_apply_remote_task_if_available()` | 车 2 检查并应用缓存的远程任务，跳过识别板一 |

### 9.4 信号格式

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

`stamp` 仅用于日志/调试，不再作为任务丢弃条件。

### 9.5 安全机制

- 字段完整性校验（code / box / lab_window 必须合法）
- `target_car` 必须匹配本车，`from_car` 不能是本车
- 解析失败或字段不合法 → 回退到正常识别板一流程
- 本轮已发送过远程任务 → 不再重复发送
- 车 2 已开始执行任务 → 忽略后续远程任务
- `dual_car_remote_task_enabled=false` → 完全走原有双车流程
