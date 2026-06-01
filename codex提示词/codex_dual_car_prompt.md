# 给 Codex 的提示词：补充智慧药房双车循环协作逻辑

> 使用方式：请把本 Markdown 作为任务说明发给 Codex。不要让 Codex 一次性修改完整项目，建议严格按照“阶段 0 → 阶段 1 → 阶段 2 → 阶段 3 → 阶段 4”的顺序执行。每个阶段完成后先阅读 diff、运行静态检查或小规模仿真测试，再继续下一阶段。本文新增“后续优化预留”部分，仅用于提前设计接口，默认不要启用车 2 跳过识别板一。

---

## 0. 项目背景

这是一个基于 ROS1 的智慧药房配送小车项目，当前单车逻辑已经能够完成：

1. 从起点出发；
2. 前往识别板一；
3. 识别二维码任务；
4. 前往体检区取样；
5. 前往识别板二判断化验区状态；
6. 前往化验区窗口配送；
7. 完成一轮后返回起点。

现在需要在现有代码基础上补充“双车协作”。请优先保证：

- 单车模式不受影响；
- 双车模式逻辑尽量轻量化，不增加大量计算负担；
- 不要一次性大改所有文件；
- 不要重构无关代码；
- 不要凭空编造不存在的类、函数、launch 文件或话题；
- 修改前必须先阅读相关文件，确认真实代码结构后再给出修改方案。

---

## 1. 必须先阅读的文件

请先只阅读并总结，不要立刻修改：

```text
README_DUAL_CAR.md
scripts/main_controller.py
scripts/board1_detector.py
src/pharmacy_mplus0/constants.py
src/pharmacy_mplus0/competition_io.py
src/pharmacy_mplus0/task_planner.py
src/pharmacy_mplus0/models.py
config/strategy.yaml
config/tcp.yaml
launch/main.launch
launch/main_single.launch
launch/race_bringup.launch
```

阅读后请先输出以下内容：

1. 当前主控状态机的大致流程；
2. 哪些函数负责“到识别板一”“取样”“配送”“返回起点”；
3. 现有 `/current_qr_task` 的发布与清空位置；
4. 现有 `TaskPlanner.select_best(..., excluded_box=...)` 是否已经支持排除对车任务；
5. 当前代码是否已经存在“轮流出发 / 等待对车完成 / 允许对车出发”的逻辑；
6. 你建议修改哪些文件，不要超过必要范围。

在没有完成以上阅读总结前，不要写代码。

---

## 2. 比赛规则与正确双车逻辑

根据比赛规则，双车可以轮流完成药品配送，并且双车交接区域只允许在化验区与起点线之间的区域，过程必须顺畅及时；如果双车在起点共同停留时间超过 10 秒，下一轮配送成绩不计。规则还说明，使用双车轮流完成配送时，总分基础上有 1.5 倍加成。

因此，本项目的双车协作不是“两辆车同时抢任务”，而是“轮流出发 + 返回途中放行对车”。

正确逻辑如下：

```text
比赛开始：
  car_id=1 允许先出发
  car_id=2 必须在起点等待

第 1 轮：
  车 1 执行配送任务
  车 2 在起点等待
  车 1 完成配送，开始返回起点时，立即通知车 2：你可以出发
  车 1 继续返回起点
  车 2 收到允许出发信号后，从起点出发前往识别板一

第 2 轮：
  车 2 执行配送任务
  车 1 回到起点后等待
  车 2 完成配送，开始返回起点时，立即通知车 1：你可以出发
  车 2 继续返回起点
  车 1 收到允许出发信号后，从起点出发前往识别板一

之后循环：
  车 1 → 车 2 → 车 1 → 车 2 → ...
```

关键要求：

- “允许对车出发”的信号发送时机不是本车完全回到起点之后，而是本车完成配送并开始返回起点时。
- 正在配送的车继续返回起点。
- 另一台车收到允许出发信号后，可以同时前往识别板一识别。
- 起点等待的车必须保持静止，不能提前出发。
- 收到不是发给自己的允许出发信号必须忽略。
- 如果自己不是“起点等待状态”，即使收到允许出发信号也不能重复触发。
- `/current_qr_task` 只用于二维码任务占用与方框排除，不要混用为出发控制信号。

---

## 3. 当前 README 中已有但不完整的逻辑

当前 `README_DUAL_CAR.md` 已经预留了以下能力：

1. `/current_qr_task`：广播当前车占用的二维码任务，例如 `"AB-1"` 或空字符串；
2. `/all_qrcodes`：广播识别板一全部二维码识别结果；
3. `TaskPlanner.select_best(detections, excluded_box=None)`：可排除对车占用的方框；
4. `CompetitionIO.publish_qr_task(code, lab_window)`：发布当前任务占用；
5. `round_return_to_start: true`：每轮结束后返回起点；
6. `car_id`：launch、tcp.yaml 中已有车号配置。

但是这些内容主要解决的是“不要抢同一个二维码任务”，没有实现真正的“双车循环轮流出发”。

请补充新的轻量化双车调度逻辑。

---

## 4. 推荐方案：新增轻量级出发令牌话题

为了避免把任务占用话题和出发调度话题混在一起，建议新增一个极轻量的话题：

```text
/dual_car_signal
```

消息类型使用：

```text
std_msgs/String
```

不需要自定义 msg，不增加编译复杂度。

推荐消息格式：

```text
ALLOW_START:1
ALLOW_START:2
```

含义：

```text
ALLOW_START:1  表示允许 car_id=1 出发
ALLOW_START:2  表示允许 car_id=2 出发
```

也可以根据实际代码风格使用 JSON 字符串，但优先推荐简单字符串，因为负载更低、调试更方便：

```bash
rostopic pub /dual_car_signal std_msgs/String "data: 'ALLOW_START:2'" -1
```

---

## 5. 推荐新增配置

请在不破坏单车模式的前提下，尽量新增配置项，而不是写死逻辑。

建议在 `config/strategy.yaml` 或更合适的位置新增：

```yaml
dual_car:
  enabled: false              # 默认 false，保持单车模式不受影响
  start_first_car_id: "1"     # 比赛开始时默认 1 号车先出发
  signal_topic: "/dual_car_signal"
  wait_start_timeout: 0.0     # 0 表示一直等待；如需要可后续扩展
  publish_allow_when_returning: true
```

如果项目当前读取 YAML 的方式不支持嵌套字典，可以改为扁平字段：

```yaml
dual_car_enabled: false
dual_car_start_first_car_id: "1"
dual_car_signal_topic: "/dual_car_signal"
dual_car_publish_allow_when_returning: true
```

请先检查现有配置读取方式，再决定采用嵌套还是扁平字段。不要强行引入复杂配置解析。

---

## 6. 状态机设计要求

请在 `scripts/main_controller.py` 中增加尽量少量的状态变量。

推荐状态变量：

```python
self._dual_car_enabled = False
self._car_id = "1"
self._dual_waiting_at_start = False
self._dual_start_allowed = True
self._dual_allow_sent_this_round = False
```

含义：

```text
_dual_car_enabled:
  是否启用双车循环调度。

_car_id:
  当前车编号，来自 launch 参数或 tcp.yaml 中已有 car_id 配置。

_dual_waiting_at_start:
  当前车是否处于“起点等待对车允许出发”的状态。

_dual_start_allowed:
  当前车是否已经获得本轮出发令牌。

_dual_allow_sent_this_round:
  本轮是否已经向对车发送过允许出发信号，防止重复发送。
```

初始化逻辑：

```text
如果 dual_car_enabled=false：
  保持原单车逻辑，不等待任何信号。

如果 dual_car_enabled=true：
  如果 self.car_id == start_first_car_id：
    self._dual_start_allowed = True
    self._dual_waiting_at_start = False
  否则：
    self._dual_start_allowed = False
    self._dual_waiting_at_start = True
    小车保持起点等待，不进入前往识别板一的流程。
```

---

## 7. 出发等待逻辑

请在每一轮开始、准备从起点前往识别板一之前加入判断：

```text
如果 dual_car_enabled=false：
  直接按原逻辑出发。

如果 dual_car_enabled=true 且 self._dual_start_allowed=True：
  清除本轮状态，开始前往识别板一。

如果 dual_car_enabled=true 且 self._dual_start_allowed=False：
  保持停车，进入 WAIT_AT_START 状态或等价逻辑。
  循环等待 /dual_car_signal 中发给自己的 ALLOW_START。
```

等待时必须发布零速度，避免小车因为上一轮速度命令残留继续移动。

如果现有状态机已经有 `WAIT`、`IDLE`、`DONE_ROUND`、`RETURN_TO_START` 等状态，请优先复用现有状态，不要强行新建复杂状态机。目标是轻量修改。

---

## 8. 允许对车出发的发送时机

请找到“完成配送，准备返回起点”的代码位置。

在本车已经完成一次配送、即将进入返回起点流程时，发送：

```text
如果 self.car_id == "1"：发布 ALLOW_START:2
如果 self.car_id == "2"：发布 ALLOW_START:1
```

注意：

- 只发送一次；
- 不要等到自己完全回到起点后再发送；
- 发送后本车继续执行返回起点；
- 发送后将 `self._dual_start_allowed = False`，表示自己下一轮需要等待对车放行；
- 发送后将 `self._dual_allow_sent_this_round = True`，防止重复发送；
- 单车模式下不要发送该信号。

推荐封装函数：

```python
def _dual_peer_id(self):
    return "2" if str(self._car_id) == "1" else "1"


def _dual_publish_allow_peer_start(self):
    if not self._dual_car_enabled:
        return
    if self._dual_allow_sent_this_round:
        return
    peer_id = self._dual_peer_id()
    msg = "ALLOW_START:%s" % peer_id
    self._dual_signal_pub.publish(String(data=msg))
    self._dual_allow_sent_this_round = True
    self._dual_start_allowed = False
    self._dual_waiting_at_start = False
    loginfo("[DualCar] sent %s, peer can start while this car returns", msg)
```

请根据项目实际的日志封装方式使用 `rospy.loginfo` 或已有 `log_utils.loginfo`。

---

## 9. 接收允许出发信号

新增订阅：

```python
rospy.Subscriber(
    self._dual_signal_topic,
    String,
    self._cb_dual_car_signal,
    queue_size=5,
)
```

推荐回调：

```python
def _cb_dual_car_signal(self, msg):
    body = (msg.data or "").strip()
    prefix = "ALLOW_START:"
    if not body.startswith(prefix):
        return

    target_id = body[len(prefix):].strip()
    if target_id != str(self._car_id):
        return

    if not self._dual_car_enabled:
        return

    if not self._dual_waiting_at_start:
        loginfo("[DualCar] ignored %s because current car is not waiting at start", body)
        return

    self._dual_start_allowed = True
    self._dual_waiting_at_start = False
    loginfo("[DualCar] received %s, this car can start next round", body)
```

关键点：

- 只处理发给自己的消息；
- 非双车模式忽略；
- 当前车不在起点等待时忽略，防止配送途中误触发；
- 收到允许后，只改变状态，不要在回调里直接调用长耗时导航函数，避免 ROS 回调线程阻塞。

---

## 10. 与 `/current_qr_task` 的关系

`/current_qr_task` 继续只做“任务占用广播”。

不要把它改成出发信号。

保留或完善以下逻辑：

```text
本车到识别板一并选定二维码任务后：
  publish_qr_task(code, lab_window)

本车本轮结束、任务不再占用时：
  publish_qr_task("", "")
```

如果现有代码中“每轮开始前往识别板一时清空任务广播”的行为会导致对车无法正确排除任务，请重新评估清空时机。更合理的清空时机通常是：

```text
本车已完成该二维码对应配送任务后，再清空当前任务占用。
```

请检查现有逻辑是否存在提前清空导致任务冲突的问题。如果存在，请在单独阶段提出修改建议，不要和出发调度一次性混改。

---

## 11. 可选优化：删除或弱化无用逻辑

如果你发现当前 README 或代码中的 `/all_qrcodes` 共享逻辑对本项目实际双车循环帮助不大，可以不要优先实现它。

原因：

- 两车轮流出发时，后一台车收到允许信号后会前往识别板一自行识别；
- `/all_qrcodes` 会增加话题传输和解析逻辑；
- 对树莓派小车来说，优先保证主控稳定，避免额外 JSON 解析负担；
- `/all_qrcodes` 可以保留为调试或后续增强，不作为第一阶段必须实现内容。

请优先实现：

```text
1. /dual_car_signal 轮流出发
2. /current_qr_task 任务占用排除
3. 必要日志和调试命令
```

不要优先实现复杂的全局任务池、抢占式调度、心跳检测、超时恢复、多车路径避障等高级功能。


---

## 12. 后续优化预留：车 1 共享识别板一任务给车 2（暂不启用）

> 重要说明：本节是“后续优化预留接口”，不是当前阶段必须立即实施的功能。当前优先目标仍然是完成 `/dual_car_signal` 的双车循环轮流出发，以及 `/current_qr_task` 的任务占用排除。除非我明确要求启用，否则不要让车 2 直接跳过识别板一执行远程任务。

### 12.1 优化思路

识别板一每次可能出现 2 个二维码。当前视觉方案中，如果车 1 在识别板一能够稳定同时识别到两个二维码，则可以考虑：

```text
车 1 到识别板一；
车 1 同时识别到两个二维码任务；
车 1 选择其中一个任务自己执行；
车 1 将另一个任务缓存为“可分配给车 2 的候选远程任务”；
车 1 完成配送并开始返回起点时，除了发送 ALLOW_START:2，也可以附带发送分配给车 2 的二维码任务；
车 2 收到远程任务后，如果该功能已启用且任务未过期，则可以跳过识别板一，直接前往对应体检区窗口取样并配送。
```

这样可以减少车 2 的识别板一停车识别时间，降低车 2 视觉计算负载，提高双车轮流效率。

### 12.2 当前阶段不要立即启用

请注意：这个方案依赖小车整体配送速度和二维码刷新周期。如果车 2 收到的任务在真正执行前已经过期，反而可能导致配送错误。因此当前只要求预留轻量接口，不要求改变默认流程。

默认行为必须仍然是：

```text
车 2 收到 ALLOW_START 后，正常前往识别板一自行识别。
```

只有在后续确认小车配送速度能够赶上识别板一二维码刷新速度后，才启用：

```text
车 2 收到有效远程任务后，跳过识别板一。
```

### 12.3 建议预留配置

可以在配置中预留开关，但默认必须关闭：

```yaml
dual_car_remote_task_enabled: false       # 默认关闭，不改变现有流程
dual_car_remote_task_max_age_sec: 45.0    # 远程任务最大有效时间，超过则丢弃
dual_car_remote_task_skip_board1: false   # 默认不跳过识别板一
dual_car_remote_task_prefer_peer: false   # 默认不优先使用对车任务
```

如果项目配置不适合新增这么多字段，可以只预留最小字段：

```yaml
dual_car_remote_task_enabled: false
dual_car_remote_task_max_age_sec: 45.0
```

### 12.4 建议预留数据结构

可以在 `main_controller.py` 中预留缓存变量，但不要立即接入主流程：

```python
self._dual_remote_task_enabled = False
self._dual_remote_task_max_age_sec = 45.0
self._dual_assigned_remote_task = None
```

远程任务建议至少包含：

```text
from_car:       发送任务的小车编号
target_car:     接收任务的小车编号
code:           体检区窗口组合，例如 A、B、C、AB、AC、BC、ABC
box:            识别板一方框编号，用于调试和去重
lab_window:     目标化验窗口，例如 1、2、3、4
sample_count:   样本数量
stamp:          识别或发送时间戳，用于判断是否过期
```

### 12.5 建议消息格式

为了轻量化，可以继续使用 `std_msgs/String`，消息内容使用 JSON 字符串。不要新增自定义 msg。

示例：

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

也可以保留原来的简单信号：

```text
ALLOW_START:2
```

要求：

```text
当前必须兼容 ALLOW_START:<car_id> 简单格式；
后续如果收到 ALLOW_START_WITH_TASK，但 remote_task_enabled=false，则只把它当作普通 ALLOW_START 使用，不跳过识别板一；
只有 remote_task_enabled=true 且任务未过期，才缓存 task；
只有 remote_task_skip_board1=true 时，才允许跳过识别板一；
任务过期、字段缺失、target_car 不是自己、当前不在 WAIT_AT_START 时，必须忽略远程任务。
```

### 12.6 建议任务分配策略

如果后续启用该优化，车 1 在识别到两个二维码后可以采用最简单策略：

```text
车 1 使用 TaskPlanner 选择最优任务给自己；
从剩余二维码中选择一个任务分配给车 2；
不要让车 2 再对远程任务重新 select_best，避免再次产生冲突；
车 2 只负责校验远程任务是否合法、是否过期、是否发给自己。
```

如果车 1 只识别到一个二维码，则不要给车 2 分配远程任务，车 2 仍按原流程去识别板一。

### 12.7 车 2 的兜底逻辑

后续启用远程任务时，车 2 必须保留兜底路径：

```text
收到有效远程任务：
  如果 remote_task_skip_board1=true：跳过识别板一，直接执行远程任务；
  如果 remote_task_skip_board1=false：仍然去识别板一，但可把远程任务作为调试日志或候选信息；

没有收到远程任务：
  正常去识别板一识别；

远程任务超时或字段非法：
  丢弃远程任务，正常去识别板一识别；

通信异常：
  不影响原双车轮流出发流程。
```

### 12.8 当前阶段只需要 ai 做什么

当前阶段请你最多只做以下轻量预留，不要改主流程：

```text
1. 在配置文件中预留 remote_task 相关开关，默认 false；
2. 在 main_controller.py 中预留远程任务缓存变量；
3. 在 /dual_car_signal 回调中预留对 JSON 消息的安全解析函数，但默认不启用跳过识别板一；
4. 在阶段分析报告中指出：后续要启用该方案，需要修改哪几个主控流程入口；
5. 不要让车 2 现在就跳过识别板一。
```

如果 Codex 认为当前代码结构不适合立即预留 JSON 解析，也可以只在文档和注释中保留设计，不做代码改动。

---

## 13. 分阶段修改要求

### 阶段 0：只分析，不修改

请先输出分析结果：

```text
- 当前主控循环在哪里；
- 当前每轮开始的位置在哪里；
- 当前配送完成的位置在哪里；
- 当前返回起点的位置在哪里；
- 应该在哪里等待出发；
- 应该在哪里发布 ALLOW_START；
- 哪些文件需要修改；
- 是否存在当前代码与 README 不一致的地方。
```

完成后暂停，等待我确认。

---

### 阶段 1：只补充常量、配置、IO 接口

目标：只增加轻量级通信接口，不改变主控流程。

建议修改：

```text
src/pharmacy_mplus0/constants.py
src/pharmacy_mplus0/competition_io.py
config/strategy.yaml 或其他实际配置文件
```

需要实现：

```text
TOPIC_DUAL_CAR_SIGNAL = "/dual_car_signal"
CompetitionIO 中增加 dual signal publisher，可选增加 publish_dual_signal(text)
读取 dual_car_enabled、start_first_car_id、signal_topic 等配置
```

阶段 1 完成后请输出：

```text
- 修改文件列表；
- 每个文件具体改了什么；
- 如何用 rostopic echo /dual_car_signal 验证；
- 不要继续改 main_controller.py。
```

---

### 阶段 2：只实现起点等待和信号接收

目标：实现“车 2 开局等待”“收到 ALLOW_START 后才允许出发”。

建议修改：

```text
scripts/main_controller.py
```

需要实现：

```text
- 初始化双车状态变量；
- 订阅 /dual_car_signal；
- 解析 ALLOW_START:<car_id>；
- 在每轮开始前检查是否允许出发；
- 未允许出发时发布零速度并保持等待；
- 单车模式完全不受影响。
```

阶段 2 完成后请提供最小测试方法：

```bash
# 模拟允许 2 号车出发
rostopic pub /dual_car_signal std_msgs/String "data: 'ALLOW_START:2'" -1
```

请说明预期日志。

---

### 阶段 3：只实现配送完成后放行对车

目标：实现“本车完成配送并开始返回起点时，通知对车出发”。

建议修改：

```text
scripts/main_controller.py
```

需要实现：

```text
- 找到完成配送后进入返回起点的代码位置；
- 在该位置调用 _dual_publish_allow_peer_start()；
- 确保每轮只发送一次；
- 发送后本车继续返回起点；
- 发送后本车下一轮需要等待对车允许。
```

阶段 3 完成后请输出：

```text
- ALLOW_START 的发送位置；
- 为什么选择这个位置；
- 如何避免重复发送；
- 如何保证发送后本车继续返回起点。
```

---

### 阶段 4：补充任务占用排除逻辑

目标：在不影响轮流出发的前提下，补齐 README 中原本预留的任务避让逻辑。

建议修改：

```text
scripts/main_controller.py
```

需要实现：

```text
- 订阅 /current_qr_task；
- 解析对车占用任务；
- 注意忽略自己发布的消息，避免把自己的任务当作对车任务；
- 在 TaskPlanner.select_best() 中传入 excluded_box；
- 保留单车模式兼容。
```

特别注意：

当前 `/current_qr_task` 的格式只有 `"AB-1"`，没有包含 car_id，因此两台车如果在同一个 ROS Master 下互相订阅，可能会收到自己发布的消息。

请优先考虑将发布格式升级为带车号的轻量格式，例如：

```text
CAR1:AB-1
CAR2:ABC-2
CAR1:
CAR2:
```

或者：

```text
1|AB|1
2|ABC|2
1||
2||
```

要求：

- 必须兼容旧格式，避免单车调试崩溃；
- 新格式中能够识别 sender_car_id；
- 如果 sender_car_id == self.car_id，则忽略；
- 如果收到旧格式且无法判断来源，请只在 dual_car_enabled=false 或明确需要兼容时处理，避免误排除自己的任务。

如果你认为升级 `/current_qr_task` 格式牵涉较多，请先提出方案和影响范围，不要直接大改。

---

## 14. 调试与验证要求

请为每个阶段提供测试命令。

### 13.1 单车模式回归测试

```bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1
```

预期：

```text
- 不等待 /dual_car_signal；
- 原有单车流程正常；
- 不因为缺少对车而卡死。
```

### 13.2 双车开局等待测试

```bash
# 车 1
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1 dual_car_enabled:=true

# 车 2
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=2 dual_car_enabled:=true audio_dir:=""
```

预期：

```text
car_id=1 直接开始第一轮；
car_id=2 在起点等待，不移动；
```

### 13.3 模拟放行测试

```bash
rostopic pub /dual_car_signal std_msgs/String "data: 'ALLOW_START:2'" -1
```

预期：

```text
car_id=2 收到信号后从 WAIT_AT_START 进入出发流程；
car_id=1 忽略该信号。
```

### 13.4 自动循环测试

预期日志类似：

```text
[DualCar] car_id=1 start allowed at boot
[DualCar] car_id=2 waiting at start
[DualCar] car_id=1 delivery done, sent ALLOW_START:2
[DualCar] car_id=2 received ALLOW_START:2, start next round
[DualCar] car_id=2 delivery done, sent ALLOW_START:1
[DualCar] car_id=1 received ALLOW_START:1, start next round
```

### 13.5 任务避让测试

```bash
rostopic pub /current_qr_task std_msgs/String "data: 'CAR1:AB-1'" -r 2
```

预期：

```text
car_id=2 解析到对车占用 box=0；
TaskPlanner.select_best(..., excluded_box=0) 不选择 box=0；
car_id=1 忽略自己发布的 CAR1:AB-1。
```

---

## 15. 代码风格要求

请遵守以下要求：

1. 保持 Python 2 / ROS Melodic 兼容，如果项目当前是 Python 2，不要使用 f-string；
2. 中文日志输出使用已有的日志封装`log_utils`，避免 ROS Melodic + Python 2 编码问题；
3. 不要引入新第三方依赖；
4. 不要引入 actionlib、service、自定义 msg，除非现有项目已经使用且确有必要；
5. 不要一次性重写 `main_controller.py`；
6. 每次只输出当前阶段的完整修改代码或精确 diff；
7. 修改后说明如何测试；
8. 如果发现 README 与真实代码不一致，以真实代码为准，并指出不一致点。

---

## 16. 最终目标

最终实现后，双车流程应满足：

```text
car_id=1 第一轮出发；
car_id=2 起点等待；
car_id=1 完成配送并开始返回起点时，发布 ALLOW_START:2；
car_id=2 收到后开始第二轮；
car_id=1 回到起点后等待；
car_id=2 完成配送并开始返回起点时，发布 ALLOW_START:1；
car_id=1 收到后开始第三轮；
两车持续循环，直到比赛结束或任务无法继续。
```

同时保留：

```text
/current_qr_task 用于任务占用；
TaskPlanner.select_best(excluded_box=...) 用于避开对车已选二维码方框；
/all_qrcodes 暂时可保留为调试/后续增强，不作为第一优先级；
单车模式默认不启用双车等待逻辑。
```
```text
car1与car2代码能够兼容共有，做到只修改部分内容（如car_id、IP地址等）便可部署运行。
```

---

## 17. 请先执行的第一步

请现在只做“阶段 0：只分析，不修改”。

输出格式：

```markdown
# 阶段 0 分析结果

## 1. 当前主控流程
...

## 2. 每轮开始位置
...

## 3. 配送完成与返回起点位置
...

## 4. 当前双车预留接口
...

## 5. 缺失的循环协作逻辑
...

## 6. 建议修改文件
...

## 7. 风险点
...

## 8. 下一阶段计划
...
```

分析完成后停止，等待我确认再修改代码。当所有流程补充实现完整后，在README_DUAL_CAR.md中修改补充当前的双车协作逻辑、启用方法、运行命令、debug方法等。
