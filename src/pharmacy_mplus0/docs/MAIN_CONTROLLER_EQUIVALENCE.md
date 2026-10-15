# 主控等价迁移专项核对

本文档记录 `main_controller.py` 相对迁移来源 `F1_yaofang_v5.py` 的静态核对结果。
本阶段没有调整主控代码、状态机或参数。

## 1. 源码等价性

对旧文件只应用以下机械变换后，新旧文件全文一致：

1. 模块说明中的 `F1_yaofang.py` 改为 `main_controller.py`；
2. `dual_car_config` import 改为 `pharmacy_mplus0.config`；
3. `board1_selection` import 改为 `pharmacy_mplus0.task_logic`。

除上述三项外，没有修改函数顺序、函数体、条件、发布语句、日志、休眠、异常处理或线程模型。

## 2. 状态编号和调度方式

状态编号由 `pharmacy_mplus0.config.STATES` 提供，保持为 8～15：

| 状态 | 编号 | 主控处理位置 |
|---|---:|---|
| `WAIT_TURN` | 8 | `handle_wait_turn()` |
| `GO_TO_BOARD1` | 9 | `handle_go_to_board1()` |
| `BOARD1_RECOGNIZING` | 10 | 在状态 9处理函数内阻塞等待板一结果 |
| `GO_TO_PICKUP_WINDOWS` | 11 | `handle_go_to_pickup_windows()` |
| `GO_TO_BOARD2` | 12 | `handle_go_to_board2()` |
| `BOARD2_RECOGNIZING` | 13 | 在状态 12处理函数内阻塞等待板二结果 |
| `GO_TO_LAB_WINDOW` | 14 | `handle_go_to_lab_window()` |
| `GO_BACK_HOME` | 15 | `handle_go_back_home()` |

状态 10 和 13没有单独的主循环分支，而是分别在状态 9和 12的处理函数中发布后同步等待结果。这是现有行为，未改成新的异步状态机。

## 3. 正常转换路径

车 1初始持有令牌，从状态 9开始：

```text
9 → 10 → 11 → 12 → 13 → 14 → 15 → 8/9
```

车 2初始没有令牌，从状态 8开始。收到有效对车令牌后进入状态 9。

车 2成功使用对车板一完整结果时，保留现有跳过路径：

```text
9 → 11 → 12 → 13 → 14 → 15 → 8/9
```

所有转换均继续在赋值 `self.count` 后按现有位置发布 `/nav_state`。

## 4. 导航失败与识别等待

- 板一、C、A、B、板二、化验窗口和返程导航失败时，处理函数直接返回，`self.count` 不前进，下一次状态机循环重试当前阶段。
- `move()` 超时或失败后继续取消目标、尝试清除代价地图并按原配置休眠。
- 板一和板二等待超时默认均为 0，即无限等待。
- 若以后人为设置非零识别等待超时，等待函数返回失败后状态仍停留在 10或 13；主循环没有对应处理分支。这是迁移来源的既有行为，本次不修正。

## 5. 取样顺序和完成标志

取样判断顺序保持为：

```text
C → A → B
```

- 每个窗口只在导航成功并完成裁判任务停留后设置对应 `pickup_*_done=True`。
- 后续窗口导航失败并重新进入状态 11时，已经完成的窗口会被跳过。
- 每次应用新的板一任务结果时，三个完成标志都恢复为 `False`。
- 取样裁判任务仍按 `窗口字母 → 停留 → R` 发布，语音触发位置不变。

## 6. 对车令牌

- 车 1配置为 `start_active=True`，车 2为 `False`。
- 新一轮进入状态 9时，`turn_released_this_round` 恢复为 `False`。
- 本车完成化验窗口停留和送样播报后，先进入并发布状态 15，再调用 `release_next_car_if_needed()`。
- 每轮只增加一次 `dual_round_seq` 并发布一次逻辑令牌；ROS 重复发布次数和间隔继续由原参数控制。
- 发布后 `turn_released_this_round=True`、`have_turn=False`，但本车继续执行返程。
- `peer_done_callback()` 继续校验对车车号和递增序列号。
- 非状态 8收到的新令牌只设置 `peer_done_pending=True`；本车回到起点后才消耗。
- 状态 8收到令牌时立即调用 `activate_turn_from_peer()` 进入状态 9。

## 7. 对车板一结果

- 只有配置了 `use_peer_board1_result=True` 的车 2尝试复用对车结果。
- 接收回调继续校验 `car_id`、正序列号和新旧序列号。
- 使用前继续校验 `all_text`、`selected_index` 和是否已经使用过该 seq。
- 对车已选下标仍被置为 `""`，再调用同一个 `select_board1_from_all_text()` 从剩余项选择。
- 成功后继续更新 C/A/B、样本数、化验窗口下标，重置取样完成标志并重复发布 CV2。
- 数据缺失、格式错误、旧数据或无剩余有效任务时返回 `False`，本车继续导航至板一识别。

## 8. 裁判和语音时机

- 状态 9开始发布任务 `R`。
- 到达取样窗口发布对应 `C/A/B`，停留后发布 `R`。
- 板一结果接收后立即重复发布 CV2。
- 板二结果接收后立即重复发布 CV1。
- 板二忙碌时播放 `WAIT-N.wav` 后等待 N 秒；空闲时播放 `WAIT-0.wav` 且不增加等待。
- 到达化验窗口发布 `1/2/3/4`，停留后发布 `R`，然后播放送样语音。
- 音频仍通过 `subprocess.Popen(["play", path])` 启动，没有增加队列或同步等待。

## 9. 本阶段未执行项

本机未运行 ROS Master、move_base、TF、摄像头、音频、双车 TCP、裁判 TCP 或状态机。以下内容仍需在 Ubuntu 18.04 正式环境和实车上验证：

- 完整单车一轮的实际 `/nav_state` 发布时间序列；
- 双车连续轮流出发和提前令牌缓存；
- 车 2板一共享成功与本车识别回退；
- 导航失败后的实车重试及已完成窗口跳过；
- 裁判话题和语音的实际时序。
