# 语音播报实现方案

智慧药房比赛语音播报系统的完整技术文档。

---

## 1. 功能包关系

语音播报横跨两个项目功能包，通过 ROS 话题解耦：

```
pharmacy_mplus0 (主功能包)            pharmacy_mplus0_debug (调试包)
┌──────────────────────────┐         ┌───────────────────────────┐
│                          │         │                           │
│  main_controller.py      │         │  test_voice.launch         │
│  (播报触发)               │         │  (独立语音测试入口)         │
│        │                 │         │        │                  │
│        ▼                 │         │        ▼                  │
│  competition_io.py       │         │  send_fake_task.py        │
│  (发布播报请求)           │         │  (模拟播报文本)             │
│        │                 │         │        │                  │
│        │  /announce_request         │        │                  │
│        ▼                 │         │        ▼                  │
│  tcp_reporter.py         │         │  tcp_reporter.py          │
│  (消费并执行播报)         │◄────────│  (复用主包的节点)          │
│        │                 │         │                           │
│        ▼                 │         │                           │
│  voice.py                │         │                           │
│  (TTS 引擎封装)           │         │                           │
│                          │         │                           │
│  constants.py            │         │                           │
│  (播报模板常量)           │         │                           │
│                          │         │                           │
│  launch/                 │         │                           │
│  ├─ main.launch          │         │                           │
│  ├─ main_single.launch   │         │                           │
│  └─ reporter.launch      │         │                           │
└──────────────────────────┘         └───────────────────────────┘
```

### 各包职责

| 功能包 | 角色 | 说明 |
| --- | --- | --- |
| `pharmacy_mplus0` | 语音播报的完整业务实现 | 从触发、文本拼接、发布到 TTS 执行的全链路 |
| `pharmacy_mplus0_debug` | 语音的独立测试入口 | 不参与正式比赛，仅用于开发阶段验证语音链路 |

---

## 2. 数据流向

```
main_controller.py              competition_io.py             tcp_reporter.py            voice.py
(主控状态机)                     (裁判状态出口)                 (TCP上报+语音节点)           (TTS引擎)
     │                                │                            │                        │
     │ ① announce_exam_samples()      │                            │                        │
     │───────────────────────────────►│                            │                        │
     │ ① announce_board2()            │                            │                        │
     │───────────────────────────────►│                            │                        │
     │ ① announce_lab_arrival()       │ ② _publish_announce()     │                        │
     │───────────────────────────────►│──────────────────────────►│                        │
     │                                │   /announce_request        │ ③ speak(text)          │
     │                                │   (std_msgs/String)        │───────────────────────►│
     │                                │                            │                        │
     │                                │                            │              ┌─────────┴──────────┐
     │                                │                            │              │ espeak → subprocess │
     │                                │                            │              │ topic  → ROS话题    │
     │                                │                            │              │ silent → 日志      │
     │                                │                            │              └────────────────────┘
```

| 步骤 | 所在文件 | 操作 |
| --- | --- | --- |
| ① 触发 | `main_controller.py` | 状态机在 `AT_EXAM`、`AT_BOARD2`、`AT_LAB` 调用 `CompetitionIO` 的对应方法 |
| ② 发布 | `competition_io.py` | 拼接文本，发布到 ROS 话题 `/announce_request` |
| ③ 执行 | `tcp_reporter.py` → `voice.py` | 订阅话题回调中调用 `VoiceAnnouncer.speak()` |

---

## 3. 三个播报触发点

全部在 `main_controller.py` 的状态机中触发：

### 3.1 体检区取样完成

- **触发状态**：`AT_EXAM` — 所有体检窗口取完后
- **触发代码**：[main_controller.py:427-433](pharmacy_mplus0/scripts/main_controller.py#L427-L433)
- **播报模板**（[constants.py:97](pharmacy_mplus0/src/pharmacy_mplus0/constants.py#L97)）：`"取到{0}窗口的样本"`
- **播出示例**：`"取到A、B窗口的样本"`

### 3.2 识别板二状态播报

- **触发状态**：`AT_BOARD2` — 收到 `/cv1_result` 后
- **触发代码**：[main_controller.py:473-474](pharmacy_mplus0/scripts/main_controller.py#L473-L474)
- **播报模板**（[constants.py:98-99](pharmacy_mplus0/src/pharmacy_mplus0/constants.py#L98-L99)）：
  - 空闲时：`"化验区空闲中,请快速通过"`
  - 忙碌时：`"化验区忙碌中,需等待{0}秒"`（{0} 为 5~10）
- **播出示例**：`"化验区忙碌中,需等待8秒"`

### 3.3 化验窗口到达

- **触发状态**：`AT_LAB` — 停在化验窗口后
- **触发代码**：[main_controller.py:515-516](pharmacy_mplus0/scripts/main_controller.py#L515-L516)
- **播报模板**（[constants.py:100](pharmacy_mplus0/src/pharmacy_mplus0/constants.py#L100)）：`"到达{0}，样本数为{1}"`
- **播出示例**：`"到达血常规窗口，样本数为3"`
- **窗口中文名映射**（[constants.py:20-25](pharmacy_mplus0/src/pharmacy_mplus0/constants.py#L20-L25)）：
  | 编号 | 名称 |
  | --- | --- |
  | 1 | 血常规窗口 |
  | 2 | 体液窗口 |
  | 3 | 免疫检测窗口 |
  | 4 | 激素检验窗口 |

---

## 4. 三种 TTS 后端

全部实现在 [voice.py](pharmacy_mplus0/src/pharmacy_mplus0/voice.py) 中，通过 `tts_method` 参数切换：

### 4.1 espeak（默认）

- **机制**：`subprocess.Popen` 异步调用系统 `espeak-ng` 命令
- **参数**：`-vzh -s 140 -a 80`（中文语音、语速 140 wpm、音量 80%）
- **特点**：调用后立即返回，不阻塞主循环；无需额外 ROS 节点；无持久后台进程
- **适用**：正式比赛，小车自带音箱

### 4.2 topic

- **机制**：发布到 ROS 话题 `/tts_text`（`std_msgs/String`）
- **延迟导入**：`voice.py` 在构造时才 `import rospy`，非 ROS 环境可安全加载
- **适用**：对接外部语音模块（如讯飞离线 TTS）

### 4.3 silent

- **机制**：仅通过 `log_utils.loginfo` 记录日志，不发声
- **适用**：调试、双车跟车模式、无音箱环境
- **入口**：[main_single.launch](pharmacy_mplus0/launch/main_single.launch) 默认 `tts_method=silent`

### 后端对比

| 维度 | espeak | topic | silent |
| --- | --- | --- | --- |
| 依赖 | 系统 espeak-ng | ROS + 外部 TTS 节点 | 无 |
| 延迟 | 低（子进程） | 取决于外部节点 | 无 |
| 可脱离 ROS | 否（subprocess） | 否 | 是 |
| 配置参数 | 无 | `~tts_topic` (默认 /tts_text) | 无 |
| 实际发声 | 是 | 是 | 否 |

---

## 5. 涉及的 ROS 话题

| 话题名 | 消息类型 | 方向 | 作用 |
| --- | --- | --- | --- |
| `/announce_request` | `std_msgs/String` | main_controller → tcp_reporter | 携带播报文本 |
| `/tts_text` | `std_msgs/String` | voice.py → 外部 TTS 节点 | 仅 `topic` 模式使用 |

---

## 6. 涉及文件清单

### pharmacy_mplus0

| 文件 | 作用 |
| --- | --- |
| [scripts/main_controller.py](pharmacy_mplus0/scripts/main_controller.py) | 主控状态机，在 `AT_EXAM` / `AT_BOARD2` / `AT_LAB` 触发播报 |
| [scripts/tcp_reporter.py](pharmacy_mplus0/scripts/tcp_reporter.py) | TCP 上报节点，订阅 `/announce_request` 并调用 `VoiceAnnouncer` |
| [src/pharmacy_mplus0/voice.py](pharmacy_mplus0/src/pharmacy_mplus0/voice.py) | TTS 引擎封装，支持 espeak / topic / silent |
| [src/pharmacy_mplus0/constants.py](pharmacy_mplus0/src/pharmacy_mplus0/constants.py) | 播报文本模板、窗口中文名映射 |
| [src/pharmacy_mplus0/competition_io.py](pharmacy_mplus0/src/pharmacy_mplus0/competition_io.py) | 封装语义化播报方法，发布 `/announce_request` |
| [src/pharmacy_mplus0/log_utils.py](pharmacy_mplus0/src/pharmacy_mplus0/log_utils.py) | 中文日志工具，`voice.py` 通过它输出播报日志 |
| [launch/main.launch](pharmacy_mplus0/launch/main.launch) | 比赛业务启动，通过 `tts_method` 参数选择后端 |
| [launch/main_single.launch](pharmacy_mplus0/launch/main_single.launch) | 静默模式，`tts_method=silent` |
| [launch/reporter.launch](pharmacy_mplus0/launch/reporter.launch) | 单独启动 `tcp_reporter` |

### pharmacy_mplus0_debug

| 文件 | 作用 |
| --- | --- |
| [launch/test_voice.launch](pharmacy_mplus0_debug/launch/test_voice.launch) | 启动 `tcp_reporter` + `send_fake_task`，独立测试语音链路 |
| [scripts/send_fake_task.py](pharmacy_mplus0_debug/scripts/send_fake_task.py) | 向 `/announce_request` 发布自定义文本，模拟主控触发 |

---

## 7. 运行方式

### 正常比赛

```bash
roslaunch pharmacy_mplus0 main.launch tts_method:=espeak
```

### 静默调试

```bash
roslaunch pharmacy_mplus0 main_single.launch
```

### 单独测试语音

```bash
# espeak 后端
roslaunch pharmacy_mplus0_debug test_voice.launch tts_method:=espeak

# topic 后端（需外部 TTS 节点运行）
roslaunch pharmacy_mplus0_debug test_voice.launch tts_method:=topic

# silent（查看日志）
roslaunch pharmacy_mplus0_debug test_voice.launch tts_method:=silent
```

### 仅启动语音节点（配合假数据）

```bash
# 终端 1：启动语音节点
roslaunch pharmacy_mplus0 reporter.launch tts_method:=espeak

# 终端 2：发送测试文本
rosrun pharmacy_mplus0_debug send_fake_task.py _announce:="化验区忙碌中,需等待 8 秒"
```

---

## 8. 播报文本与变量来源

| 播报 | 模板 | 变量来源 |
| --- | --- | --- |
| `ANNOUNCE_EXAM_SAMPLES` | `"取到{0}窗口的样本"` | `SampleStore.carried_windows()` 返回已取样窗口列表 |
| `ANNOUNCE_BOARD2_IDLE` | `"化验区空闲中,请快速通过"` | 无变量，固定文本 |
| `ANNOUNCE_BOARD2_BUSY` | `"化验区忙碌中,需等待{0}秒"` | `board2_detector` 识别的 wait_seconds（5~10） |
| `ANNOUNCE_LAB_ARRIVAL` | `"到达{0}，样本数为{1}"` | {0}：`LAB_WINDOW_NAMES[lab_window]`，{1}：`SampleStore.count_for_lab()` |

---

## 9. 架构约束

- **主控与播报解耦**：[main_controller.py](pharmacy_mplus0/scripts/main_controller.py) 只调用 `CompetitionIO` 的方法，不直接接触 `voice.py` 或 `/announce_request`
- **播报不阻塞导航**：`espeak` 使用 `subprocess.Popen`（异步），`speak()` 调用后立即返回
- **TCP 连接失败不影响播报**：`tcp_reporter.py` 的 TCP 和语音是两个独立通道，彼此不阻塞
- **不依赖调试包**：正式比赛不加载 `pharmacy_mplus0_debug`
