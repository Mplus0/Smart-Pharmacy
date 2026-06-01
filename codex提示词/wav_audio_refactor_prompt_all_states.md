# 智慧药房小车语音播报系统重构提示词

你现在需要帮我重构智慧药房小车项目中的语音播报系统。

我当前的语音播报策略是：

- `main_controller.py` 在状态机中触发播报；
- `competition_io.py` 根据业务状态拼接中文播报文本；
- `competition_io.py` 将文本发布到 ROS 话题 `/announce_request`；
- `tcp_reporter.py` 订阅 `/announce_request`；
- `tcp_reporter.py` 调用 `voice.py` 中的 `VoiceAnnouncer.speak(text)`；
- `voice.py` 当前通过 `espeak / topic / silent` 三种方式执行 TTS 播报。

现在我想完全摒弃原来的“文本 TTS 播报策略”，不再使用 `espeak`、`topic TTS`、`silent` 文本模式，也不希望运行时再拼接中文句子进行播报。

请你将语音系统重构为“播放预先录制好的 wav 音频文件”的方案。

---

## 一、重构目标

请完成以下目标：

1. 去掉原来基于文本的 TTS 播报逻辑。
2. 改为根据业务事件播放固定的 wav 音频文件。
3. 主控状态机仍然只负责触发语音事件，不直接播放音频。
4. 仍然保留 ROS 话题解耦结构，避免主控和音频播放强耦合。
5. 音频播放不能阻塞导航主循环，播放函数应尽量异步执行。
6. 比赛运行时应稳定、简单、依赖少。
7. 调试时可以单独测试每一个 wav 音频是否能正常播放。

---

## 二、当前涉及文件

请重点检查并修改以下文件：

- `pharmacy_mplus0/scripts/main_controller.py`
- `pharmacy_mplus0/scripts/tcp_reporter.py`
- `pharmacy_mplus0/src/pharmacy_mplus0/competition_io.py`
- `pharmacy_mplus0/src/pharmacy_mplus0/voice.py`
- `pharmacy_mplus0/src/pharmacy_mplus0/constants.py`
- `pharmacy_mplus0/launch/main.launch`
- `pharmacy_mplus0/launch/main_single.launch`
- `pharmacy_mplus0/launch/reporter.launch`
- `pharmacy_mplus0_debug/launch/test_voice.launch`
- `pharmacy_mplus0_debug/scripts/send_fake_task.py`

如果项目中还有其他与 `/announce_request`、`VoiceAnnouncer`、`tts_method`、`espeak`、`/tts_text`、`silent` 相关的文件，也请一起检查并修改。

---

## 三、希望的新架构

请将语音播报系统重构为下面这种结构：

```text
main_controller.py
    |
    | 调用 CompetitionIO 的语音事件方法
    v
competition_io.py
    |
    | 发布“音频事件 ID”到 /announce_request
    | 例如：exam_sample_ab、board2_idle、board2_busy_8、lab_blood_3
    v
tcp_reporter.py
    |
    | 订阅 /announce_request
    | 收到音频事件 ID 后调用 AudioAnnouncer.play(event_id)
    v
voice.py
    |
    | 根据 event_id 查找对应 wav 文件
    | 使用 aplay / paplay / pygame / simpleaudio 等方式播放 wav
    v
audio/*.wav
```

注意：

- `/announce_request` 可以继续使用 `std_msgs/String`，但里面不再传中文句子，而是传音频事件 ID。
- `voice.py` 不再叫 TTS 引擎，而是音频播放封装。
- 可以保留 `VoiceAnnouncer` 类名以减少其他文件修改量，也可以重命名为 `AudioAnnouncer`，但如果重命名，请同步修改所有引用。

---

## 四、音频文件组织方式

请在功能包中新增音频目录，例如：

```text
pharmacy_mplus0/
├── audio/
│   ├── exam_sample_a.wav
│   ├── exam_sample_b.wav
│   ├── exam_sample_ab.wav
│   ├── board2_idle.wav
│   ├── board2_busy_5.wav
│   ├── board2_busy_6.wav
│   ├── board2_busy_7.wav
│   ├── board2_busy_8.wav
│   ├── board2_busy_9.wav
│   ├── board2_busy_10.wav
│   ├── lab_blood_1.wav
│   ├── lab_blood_2.wav
│   ├── lab_blood_3.wav
│   ├── lab_bodyfluid_1.wav
│   ├── lab_bodyfluid_2.wav
│   ├── lab_bodyfluid_3.wav
│   ├── lab_immunity_1.wav
│   ├── lab_immunity_2.wav
│   ├── lab_immunity_3.wav
│   ├── lab_hormone_1.wav
│   ├── lab_hormone_2.wav
│   └── lab_hormone_3.wav
```

补充：根据比赛规则中的语音播报细则，音频状态需要覆盖以下完整状态。

#### 1. 体检区取样播报状态

体检区存在 `A`、`B`、`C` 三个管口，每轮可能出现以下 7 种管口组合状态：

| 管口组合 | 组合含义 | event_id 片段 |
| --- | --- | --- |
| A | A 管口含有样本 | `a` |
| B | B 管口含有样本 | `b` |
| C | C 管口含有样本 | `c` |
| A+B | A、B 管口含有样本 | `ab` |
| A+C | A、C 管口含有样本 | `ac` |
| B+C | B、C 管口含有样本 | `bc` |
| A+B+C | A、B、C 管口均含有样本 | `abc` |

样本类型共有 4 种，并且对应化验区 1、2、3、4 四个目标点位：

| 样本类型 | 对应目标点位 | 对应化验窗口 | event_id 片段 |
| --- | --- | --- | --- |
| 静脉血样本 | 1 | 血常规窗口 | `venous_blood` |
| 唾液样本 | 2 | 体液窗口 | `saliva` |
| 组织样本 | 3 | 免疫检测窗口 | `tissue` |
| 血浆样本 | 4 | 激素检验窗口 | `plasma` |

因此，体检区取样播报应覆盖 `7 × 4 = 28` 种完整状态：

| 播报内容含义 | event_id | wav 文件 |
| --- | --- | --- |
| 取到 A 管口中的静脉血样本 | `exam_sample_a_venous_blood` | `exam_sample_a_venous_blood.wav` |
| 取到 B 管口中的静脉血样本 | `exam_sample_b_venous_blood` | `exam_sample_b_venous_blood.wav` |
| 取到 C 管口中的静脉血样本 | `exam_sample_c_venous_blood` | `exam_sample_c_venous_blood.wav` |
| 取到 A、B 管口中的静脉血样本 | `exam_sample_ab_venous_blood` | `exam_sample_ab_venous_blood.wav` |
| 取到 A、C 管口中的静脉血样本 | `exam_sample_ac_venous_blood` | `exam_sample_ac_venous_blood.wav` |
| 取到 B、C 管口中的静脉血样本 | `exam_sample_bc_venous_blood` | `exam_sample_bc_venous_blood.wav` |
| 取到 A、B、C 管口中的静脉血样本 | `exam_sample_abc_venous_blood` | `exam_sample_abc_venous_blood.wav` |
| 取到 A 管口中的唾液样本 | `exam_sample_a_saliva` | `exam_sample_a_saliva.wav` |
| 取到 B 管口中的唾液样本 | `exam_sample_b_saliva` | `exam_sample_b_saliva.wav` |
| 取到 C 管口中的唾液样本 | `exam_sample_c_saliva` | `exam_sample_c_saliva.wav` |
| 取到 A、B 管口中的唾液样本 | `exam_sample_ab_saliva` | `exam_sample_ab_saliva.wav` |
| 取到 A、C 管口中的唾液样本 | `exam_sample_ac_saliva` | `exam_sample_ac_saliva.wav` |
| 取到 B、C 管口中的唾液样本 | `exam_sample_bc_saliva` | `exam_sample_bc_saliva.wav` |
| 取到 A、B、C 管口中的唾液样本 | `exam_sample_abc_saliva` | `exam_sample_abc_saliva.wav` |
| 取到 A 管口中的组织样本 | `exam_sample_a_tissue` | `exam_sample_a_tissue.wav` |
| 取到 B 管口中的组织样本 | `exam_sample_b_tissue` | `exam_sample_b_tissue.wav` |
| 取到 C 管口中的组织样本 | `exam_sample_c_tissue` | `exam_sample_c_tissue.wav` |
| 取到 A、B 管口中的组织样本 | `exam_sample_ab_tissue` | `exam_sample_ab_tissue.wav` |
| 取到 A、C 管口中的组织样本 | `exam_sample_ac_tissue` | `exam_sample_ac_tissue.wav` |
| 取到 B、C 管口中的组织样本 | `exam_sample_bc_tissue` | `exam_sample_bc_tissue.wav` |
| 取到 A、B、C 管口中的组织样本 | `exam_sample_abc_tissue` | `exam_sample_abc_tissue.wav` |
| 取到 A 管口中的血浆样本 | `exam_sample_a_plasma` | `exam_sample_a_plasma.wav` |
| 取到 B 管口中的血浆样本 | `exam_sample_b_plasma` | `exam_sample_b_plasma.wav` |
| 取到 C 管口中的血浆样本 | `exam_sample_c_plasma` | `exam_sample_c_plasma.wav` |
| 取到 A、B 管口中的血浆样本 | `exam_sample_ab_plasma` | `exam_sample_ab_plasma.wav` |
| 取到 A、C 管口中的血浆样本 | `exam_sample_ac_plasma` | `exam_sample_ac_plasma.wav` |
| 取到 B、C 管口中的血浆样本 | `exam_sample_bc_plasma` | `exam_sample_bc_plasma.wav` |
| 取到 A、B、C 管口中的血浆样本 | `exam_sample_abc_plasma` | `exam_sample_abc_plasma.wav` |

#### 2. 识别板二状态播报状态

识别板二状态播报应覆盖以下完整状态：

| 播报内容含义 | event_id | wav 文件 |
| --- | --- | --- |
| 化验区空闲中，请快速通过 | `board2_idle` | `board2_idle.wav` |
| 化验区忙碌中，需等待 5 秒 | `board2_busy_5` | `board2_busy_5.wav` |
| 化验区忙碌中，需等待 6 秒 | `board2_busy_6` | `board2_busy_6.wav` |
| 化验区忙碌中，需等待 7 秒 | `board2_busy_7` | `board2_busy_7.wav` |
| 化验区忙碌中，需等待 8 秒 | `board2_busy_8` | `board2_busy_8.wav` |
| 化验区忙碌中，需等待 9 秒 | `board2_busy_9` | `board2_busy_9.wav` |
| 化验区忙碌中，需等待 10 秒 | `board2_busy_10` | `board2_busy_10.wav` |

#### 3. 化验区目标点位到达播报状态

化验区存在 1、2、3、4 四个目标点位，分别对应 4 个化验窗口：

| 目标点位 | 化验窗口 | event_id 片段 |
| --- | --- | --- |
| 1 | 血常规窗口 | `blood` |
| 2 | 体液窗口 | `bodyfluid` |
| 3 | 免疫检测窗口 | `immunity` |
| 4 | 激素检验窗口 | `hormone` |

化验区到达播报应覆盖以下完整状态：

| 播报内容含义 | event_id | wav 文件 |
| --- | --- | --- |
| 到达血常规窗口，样本数为 1 | `lab_blood_1` | `lab_blood_1.wav` |
| 到达血常规窗口，样本数为 2 | `lab_blood_2` | `lab_blood_2.wav` |
| 到达血常规窗口，样本数为 3 | `lab_blood_3` | `lab_blood_3.wav` |
| 到达体液窗口，样本数为 1 | `lab_bodyfluid_1` | `lab_bodyfluid_1.wav` |
| 到达体液窗口，样本数为 2 | `lab_bodyfluid_2` | `lab_bodyfluid_2.wav` |
| 到达体液窗口，样本数为 3 | `lab_bodyfluid_3` | `lab_bodyfluid_3.wav` |
| 到达免疫检测窗口，样本数为 1 | `lab_immunity_1` | `lab_immunity_1.wav` |
| 到达免疫检测窗口，样本数为 2 | `lab_immunity_2` | `lab_immunity_2.wav` |
| 到达免疫检测窗口，样本数为 3 | `lab_immunity_3` | `lab_immunity_3.wav` |
| 到达激素检验窗口，样本数为 1 | `lab_hormone_1` | `lab_hormone_1.wav` |
| 到达激素检验窗口，样本数为 2 | `lab_hormone_2` | `lab_hormone_2.wav` |
| 到达激素检验窗口，样本数为 3 | `lab_hormone_3` | `lab_hormone_3.wav` |

请在代码中确保以上所有 event_id 都能映射到对应的 wav 文件。真实比赛中无需一次性播放所有音频，只需要根据识别结果、样本类型、取样管口组合、目标点位和样本数量动态选择对应 event_id。


如果你认为还有更合理的命名方式，可以优化，但必须做到：

- 文件名语义清晰；
- event_id 和 wav 文件一一对应；
- 缺失文件时要打印明确错误日志；
- 不要因为某个 wav 文件缺失导致整个主控程序崩溃。

---

## 五、原三个播报触发点的改造要求

当前有三个主要播报触发点，请分别改造。

### 1. 体检区取样完成

原逻辑：

- 拼接中文文本，例如：`取到A、B窗口的样本`。

新逻辑：

- 根据取样窗口生成音频事件 ID。

示例：

| 场景 | event_id | wav 文件 |
| --- | --- | --- |
| 只取 A | `exam_sample_a` | `exam_sample_a.wav` |
| 只取 B | `exam_sample_b` | `exam_sample_b.wav` |
| A 和 B 都取到 | `exam_sample_ab` | `exam_sample_ab.wav` |

请检查 `SampleStore.carried_windows()` 的返回格式，并根据实际代码生成稳定的 event_id。

### 2. 识别板二状态播报

原逻辑：

- 空闲：`化验区空闲中,请快速通过`
- 忙碌：`化验区忙碌中,需等待{0}秒`

新逻辑：

| 场景 | event_id | wav 文件 |
| --- | --- | --- |
| 化验区空闲 | `board2_idle` | `board2_idle.wav` |
| 等待 5 秒 | `board2_busy_5` | `board2_busy_5.wav` |
| 等待 6 秒 | `board2_busy_6` | `board2_busy_6.wav` |
| 等待 7 秒 | `board2_busy_7` | `board2_busy_7.wav` |
| 等待 8 秒 | `board2_busy_8` | `board2_busy_8.wav` |
| 等待 9 秒 | `board2_busy_9` | `board2_busy_9.wav` |
| 等待 10 秒 | `board2_busy_10` | `board2_busy_10.wav` |

请确保 `wait_seconds` 被限制在 `5~10` 范围内。如果识别结果异常，请使用默认值或打印警告，不要崩溃。

### 3. 化验窗口到达

原逻辑：

- 拼接中文文本，例如：`到达血常规窗口，样本数为3`。

新逻辑：

- 根据化验窗口编号和样本数量生成音频事件 ID。

示例：

| 场景 | event_id | wav 文件 |
| --- | --- | --- |
| 血常规窗口，样本数 1 | `lab_blood_1` | `lab_blood_1.wav` |
| 血常规窗口，样本数 2 | `lab_blood_2` | `lab_blood_2.wav` |
| 血常规窗口，样本数 3 | `lab_blood_3` | `lab_blood_3.wav` |
| 体液窗口，样本数 1 | `lab_bodyfluid_1` | `lab_bodyfluid_1.wav` |
| 体液窗口，样本数 2 | `lab_bodyfluid_2` | `lab_bodyfluid_2.wav` |
| 体液窗口，样本数 3 | `lab_bodyfluid_3` | `lab_bodyfluid_3.wav` |
| 免疫检测窗口，样本数 1 | `lab_immunity_1` | `lab_immunity_1.wav` |
| 免疫检测窗口，样本数 2 | `lab_immunity_2` | `lab_immunity_2.wav` |
| 免疫检测窗口，样本数 3 | `lab_immunity_3` | `lab_immunity_3.wav` |
| 激素检验窗口，样本数 1 | `lab_hormone_1` | `lab_hormone_1.wav` |
| 激素检验窗口，样本数 2 | `lab_hormone_2` | `lab_hormone_2.wav` |
| 激素检验窗口，样本数 3 | `lab_hormone_3` | `lab_hormone_3.wav` |

请检查 `constants.py` 中原来的 `LAB_WINDOW_NAMES` 或窗口编号映射，并新增适合 event_id 使用的英文映射，例如：

```python
LAB_WINDOW_AUDIO_KEYS = {
    1: "blood",
    2: "bodyfluid",
    3: "immunity",
    4: "hormone",
}
```


- 比赛规则样本数最多为 3，限制为 `1~3`；

---

## 六、voice.py 的重构要求

请重写 `voice.py`，使其负责播放 wav 文件，而不是调用 TTS。

建议功能：

```python
class AudioAnnouncer(object):
    def __init__(self, audio_dir=None, player="aplay", allow_overlap=False):
        pass

    def play(self, event_id):
        pass

    def _resolve_audio_path(self, event_id):
        pass

    def _play_wav_async(self, wav_path):
        pass
```

要求：

1. 默认 `audio_dir` 指向 `pharmacy_mplus0/audio`。
2. 可以通过 ROS 参数配置 `audio_dir`。
3. 可以通过 ROS 参数配置 `player`，默认使用 `aplay`。
4. 播放时使用 `subprocess.Popen` 异步执行，避免阻塞。
5. 如果 `allow_overlap=False`，则新音频播放前可以终止上一个还在播放的音频，避免多段声音重叠。
6. 如果 `allow_overlap=True`，则允许多个音频同时播放。
7. `event_id` 必须做基本安全检查，避免路径穿越，例如禁止包含 `/`、`..`、空格等危险字符。
8. 如果 wav 文件不存在，要打印错误日志并返回 `False`。
9. 播放成功返回 `True`，失败返回 `False`。
10. 日志输出需要兼容 ROS Melodic / Python2 环境，如果项目已有 `log_utils`，请继续使用它。

---

## 七、competition_io.py 的重构要求

请修改 `competition_io.py`：

1. 原来发布中文文本的方法，改为发布 event_id。
2. 保留语义化方法名，例如：
   - `announce_exam_samples(...)`
   - `announce_board2(...)`
   - `announce_lab_arrival(...)`
3. 这些方法内部不再拼接中文句子，而是生成 event_id。
4. 统一通过 `_publish_announce(event_id)` 发布到 `/announce_request`。
5. `_publish_announce` 中打印日志时应显示 event_id，方便调试。
6. 删除或废弃原来的中文播报模板依赖。

---

## 八、constants.py 的重构要求

请检查 `constants.py`：

1. 删除或不再使用原来的 `ANNOUNCE_EXAM_SAMPLES`、`ANNOUNCE_BOARD2_IDLE`、`ANNOUNCE_BOARD2_BUSY`、`ANNOUNCE_LAB_ARRIVAL` 等中文文本模板。
2. 新增音频事件 ID 相关常量或映射。
3. 保留必要的窗口中文名映射，如果其他业务仍然需要。
4. 新增 `LAB_WINDOW_AUDIO_KEYS` 等用于生成 wav 文件名的映射。
5. 如果有必要，新增 `BOARD2_BUSY_SECONDS_MIN` / `BOARD2_BUSY_SECONDS_MAX` 等常量。

---

## 九、tcp_reporter.py 的重构要求

请修改 `tcp_reporter.py`：

1. 原来订阅 `/announce_request` 后调用 `VoiceAnnouncer.speak(text)`。
2. 现在改为调用 `AudioAnnouncer.play(event_id)` 或 `VoiceAnnouncer.play(event_id)`。
3. 不再读取 `tts_method` 参数。
4. 新增参数：
   - `~audio_dir`
   - `~audio_player`
   - `~allow_overlap`
5. TCP 上报逻辑不要破坏。
6. TCP 连接失败不应该影响音频播放。
7. 音频播放失败不应该导致 `tcp_reporter` 节点崩溃。

---

## 十、launch 文件修改要求

请修改 launch 文件。

### 1. main.launch

删除：

```xml
<arg name="tts_method" ... />
```

新增：

```xml
<arg name="audio_dir" default="$(find pharmacy_mplus0)/audio"/>
<arg name="audio_player" default="aplay"/>
<arg name="allow_overlap" default="false"/>
```

并传给 `tcp_reporter` 节点。

### 2. main_single.launch

如果原来默认 `tts_method=silent`，请改成以下其中一种方案：

方案 A：

- 仍然启动音频播放，但 `audio_dir` 指向真实 `audio` 目录。

方案 B：

- 增加 `enable_voice` 参数，调试时可以关闭语音。

请根据代码结构选择更合理的方式。

### 3. reporter.launch

同样删除 `tts_method`，新增 `audio_dir`、`audio_player`、`allow_overlap` 参数。

### 4. test_voice.launch

改为测试 wav 音频播放：

- 启动 `tcp_reporter`；
- 启动 `send_fake_task.py`；
- `send_fake_task.py` 发布 event_id，而不是中文文本。

---

## 十一、send_fake_task.py 的重构要求

请修改 `pharmacy_mplus0_debug/scripts/send_fake_task.py`：

1. 原来的 `_announce` 参数如果是中文文本，现在改为 event_id。
2. 默认测试 event_id 可以设为 `board2_idle`。
3. 示例运行方式：

```bash
rosrun pharmacy_mplus0_debug send_fake_task.py _announce:=board2_idle
rosrun pharmacy_mplus0_debug send_fake_task.py _announce:=board2_busy_8
rosrun pharmacy_mplus0_debug send_fake_task.py _announce:=lab_blood_3
```

---

## 十二、依赖与播放方式

请优先使用系统命令 `aplay` 播放 wav：

```bash
aplay xxx.wav
```

原因：

- 简单；
- Ubuntu / ROS 环境常见；
- 不需要复杂 Python 音频库；
- 适合比赛小车。

请在代码中允许通过参数切换播放器，例如：

- `aplay`
- `paplay`
- `ffplay`

但默认使用 `aplay`。

如果系统没有 `aplay`，请在 README 或注释中提示安装：

```bash
sudo apt-get install alsa-utils
```

---

## 十三、测试要求

请给出以下测试步骤。

### 1. 检查音频文件是否存在

```bash
ls pharmacy_mplus0/audio
```

### 2. 直接测试 wav 文件

```bash
aplay pharmacy_mplus0/audio/board2_idle.wav
```

### 3. 单独启动语音节点测试

终端 1：

```bash
roslaunch pharmacy_mplus0 reporter.launch
```

终端 2：

```bash
rostopic pub /announce_request std_msgs/String "data: 'board2_idle'"
```

### 4. 使用 debug 包测试

```bash
roslaunch pharmacy_mplus0_debug test_voice.launch announce:=board2_busy_8
```

### 5. 正式主程序测试

```bash
roslaunch pharmacy_mplus0 main.launch
```

---

## 十四、请你输出的内容

请你完成代码重构，并输出以下内容：

1. 修改了哪些文件。
2. 每个文件修改的核心逻辑。
3. 新增的 `audio` 目录结构建议。
4. 最终完整代码。
5. 最终 launch 文件内容。
6. 测试命令。
7. 如果某些地方需要我补充真实 wav 文件，请明确告诉我文件名列表。

---

## 十五、重要限制

1. 不要再使用 `espeak`。
2. 不要再使用 `/tts_text`。
3. 不要再拼接中文句子给 TTS。
4. 不要破坏 `main_controller.py` 的主控状态机逻辑。
5. 不要让音频播放阻塞导航。
6. 不要让音频播放失败导致主程序崩溃。
7. 尽量少改主控代码，优先改 `competition_io.py`、`voice.py`、`tcp_reporter.py` 和 launch 文件。
8. 如果必须改 `main_controller.py`，请说明原因。
9. 请保证代码兼容 ROS1、Ubuntu 18.04、Python2.7 环境。
10. 如果你修改代码，请直接输出修改后的完整代码，方便我直接替换。

---

## 十六、修改前请先做全局检查

请先不要直接大范围改动主控逻辑，先全局搜索以下关键词，确认依赖关系后再逐文件修改：

```text
VoiceAnnouncer
speak
tts_method
announce_request
ANNOUNCE_
/tts_text
espeak
silent
```

确认完依赖关系后，再开始重构。将上述修改方案拆分为多个步骤，逐步骤进行修改，不要一次进行大范围的改动
