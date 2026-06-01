# 双车协作 — 架构说明与使用指南

> 本文档涵盖双车循环协作的设计思路、已实现的接口、启用方法、运行命令和调试方法。

---

## 1. 设计思路

比赛场地有两辆小车同时运行。双车协作采用 **"轮流出发 + 任务占用排除"** 机制：

- **轮流出发**：比赛开始时限一辆车出发，另一辆在起点等待。完成配送的车在返回起点途中放行对车。
- **任务占用排除**：每辆车选定识别板一二维码后立即广播占用，对车选择任务时排除已被占用的方框。

核心原则：**单车模式不受任何影响**，所有双车功能默认关闭，仅传入 `dual_car_enabled:=true` 时才启用。

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
| `/dual_car_signal` | main_controller | `ALLOW_START:2` | 放行对车出发 |
| `/current_qr_task` | main_controller | `CAR1:AB-1` 或 `CAR1:` | 广播自己的任务占用，订阅对车的占用 |
| `/all_qrcodes` | board1_detector | `[{"code":"AB","box":0,...}]` | 共享全部二维码结果（调试用） |

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

| 方法 | 说明 |
|------|------|
| `_dual_peer_id()` | 返回对车编号：1→2, 2→1 |
| `_cb_dual_car_signal(msg)` | 接收并解析 `/dual_car_signal`，仅处理发给自己的信号 |
| `_dual_publish_allow_peer_start()` | 完成配送后发布 `ALLOW_START:<peer_id>` |
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
# 车 1（主车，先出发，完整功能）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true

# 车 2（跟车，初始等待，静默模式避免双车同时播报）
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true audio_dir:=""
```

### 5.2 单车模式（默认，不受影响）

```bash
# 不传 dual_car_enabled 或传 false
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1
```

### 5.3 覆盖开局首发车

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

### 6.5 终端仪表盘

```bash
rosrun pharmacy_mplus0_debug topic_echo_dashboard.py
```

仪表盘实时显示 `/current_qr_task`、`/dual_car_signal`、`/current_task`、`/cv2_result` 等关键话题。

### 6.6 预期双车循环日志

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

### 6.7 离线验证排除逻辑

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
| 语音播报冲突 | 跟车用 `audio_dir:=""` 禁用语音，主车正常播报 |
| `round_return_to_start: false` | 双车模式依赖回起点来保证节奏，建议保持 `true` |

---

## 8. 相关文件索引

```
pharmacy_mplus0/
├── README_DUAL_CAR.md              ← 本文件
├── config/
│   └── strategy.yaml               ← dual_car_enabled 等配置
├── launch/
│   ├── main.launch                 ← car_id / dual_car_enabled 参数
│   ├── main_single.launch          ← 双车跟车入口（静默模式）
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

## 9. 后续优化：远程任务共享（暂不启用）

### 9.1 优化目标

识别板一每次可能出现 2 个二维码。车 1 在识别板一同时识别到两个二维码后，可把另一个分配给车 2，车 2 收到后可直接跳过识别板一前往体检区。

### 9.2 当前默认行为

**车 2 收到 `ALLOW_START` 后正常前往识别板一自行识别。** 远程任务共享功能已预留接口但默认关闭。

### 9.3 预留配置

在 `config/strategy.yaml` 中：

```yaml
# 远程任务共享开关（默认 false）
dual_car_remote_task_enabled: false
# 远程任务最大有效时间（秒）
dual_car_remote_task_max_age_sec: 45.0
```

### 9.4 预留代码

在 `scripts/main_controller.py` 中：

| 变量 | 说明 |
|------|------|
| `_dual_remote_task_enabled` | 配置开关，默认 `False` |
| `_dual_remote_task_max_age_sec` | 任务超时，默认 45 秒 |
| `_dual_assigned_remote_task` | 缓存的对车分配任务，默认 `None` |
| `_dual_try_parse_remote_task(body)` | JSON 信号安全解析函数 |

### 9.5 信号格式

对车可以在 `/dual_car_signal` 中发送 JSON 格式消息（兼容现有 `ALLOW_START:<id>` 简单格式）：

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

### 9.6 后续启用步骤

要启用远程任务共享，需要修改以下入口：

1. **`config/strategy.yaml`**：设置 `dual_car_remote_task_enabled: true`
2. **`_do_at_board1()`**：车 1 识别到 2 个二维码后，将第二个打包为 JSON 通过 `/dual_car_signal` 发送
3. **`_do_goto_board1()`**：车 2 出发前检查 `_dual_assigned_remote_task` 是否有效，若有效且配置允许则跳过识别板一导航，直接进入体检区
4. **注意**：必须确认小车配送速度能赶上识别板一二维码刷新周期后再启用
5. **兜底**：保持 `_dual_assigned_remote_task` 超时和字段校验，任何异常都回退到正常识别板一流程

### 9.7 当前保障

- 默认 `dual_car_remote_task_enabled: false`，车 2 不会跳过识别板一
- JSON 解析失败不影响现有简单格式信号处理
- 远程任务超时、字段缺失、target_car 不匹配均自动忽略
- 所有新增代码不改变原有主流程

```
