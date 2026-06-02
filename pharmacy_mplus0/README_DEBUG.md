# README_DEBUG — 修改日志

本文档记录对 Smart-Pharmacy 项目的调试修改，便于现场排查和赛后复盘。

---

## 修改记录

### #1 修复 `odom → base_footprint` TF 双重发布冲突

| 项目 | 内容 |
|------|------|
| **日期** | 2026-05-31 |
| **文件** | `robot_navigation/launch/robot_lidar.launch` |
| **修改类型** |  bug修复 |
| **严重程度** | P0 - 严重影响定位与导航 |

#### 问题描述

`art_racecar.py` 底盘节点和 `ekf_se`（robot_localization EKF）节点同时发布
`odom → base_footprint` 的 TF 变换，导致 TF 树中同一父子关系有两个发布者：

| 发布者 | 发布内容 | 来源 |
|--------|---------|------|
| `art_racecar.py` (base_control) | 原始里程计 `odom → base_footprint` | `robot_lidar.launch` 未设置 `is_pub_odom_tf` |
| `ekf_se` (robot_localization) | EKF 融合后 `odom → base_footprint` | `ekf_params.yaml` 中 `publish_tf: true` |

`art_racecar.py` 中 `is_pub_odom_tf` 的默认值为 `'false'`（字符串），代码逻辑为：

```python
# art_racecar.py 第 485-486 行
if self.is_pub_odom_tf == 'false':
    self.tf_broadcaster.sendTransform(...)   # 当 is_pub_odom_tf == 'false' 时发布 TF
```

即参数为 `'false'` 时会发布 TF，参数为 `'true'` 时**不发布**。

#### 后果

`base_footprint` 在 `odom` 坐标系下的位姿在原始里程计值和 EKF 融合值之间跳动，
导致 AMCL 定位抖动、move_base 规划的路径不稳定。

#### 修改内容

在 `robot_lidar.launch` 的 `base_control` 节点配置中添加：

```xml
<param name="is_pub_odom_tf" type="string" value="true"/>
```

| 参数 | 原值 | 新值 |
|------|------|------|
| `is_pub_odom_tf` | `'false'`（默认值，未显式设置） | `'true'`（显式设置） |

#### 修改后行为

- `art_racecar.py` 不再发布 `odom → base_footprint` 的 TF
- 该 TF 由 `ekf_se`（robot_localization）独立发布，确保融合后的里程计数据一致

#### 参考

`EPRobot_start.launch` 第 34 行已采用相同配置：
```xml
<param name="is_pub_odom_tf" type="string" value="true"/>
```

#### 验证方法

1. 启动完整系统后，在终端执行：
   ```bash
   rosrun tf tf_monitor
   ```
2. 检查 `odom → base_footprint` 是否只有一个发布者（应为 `ekf_se`）
3. 或者用 `rostopic echo /tf | grep base_footprint` 观察

---

### #7 统一 TCP 上报端口默认值

| 项目 | 内容 |
|------|------|
| **日期** | 2026-05-31 |
| **文件** | `pharmacy_mplus0/scripts/tcp_reporter.py` |
| **修改类型** | 参数修正 |
| **严重程度** | P3 - 防止默认使用场景下的连接失败 |

#### 问题描述

`tcp_reporter.py` 代码中 `_DEFAULT_SERVER_PORT` 硬编码为 `9999`，而所有 launch 文件
(`main.launch`、`reporter.launch`、`race_bringup.launch`) 和 `tcp.yaml` 配置文件中
默认端口均为 `8888`。

如果用户直接通过 `rosrun pharmacy_mplus0 tcp_reporter.py` 启动节点
（不经过 launch 文件传入 `~server_port`），节点会使用 `9999` 作为目标端口，
而裁判软件实际监听的是 `8888`，导致 TCP 连接失败。

#### 修改内容

| 文件 | 行号 | 参数 | 原值 | 新值 |
|------|------|------|------|------|
| `tcp_reporter.py` | 75 | `_DEFAULT_SERVER_PORT` | `9999` | `8888` |

```python
# 修改前
_DEFAULT_SERVER_PORT = 9999

# 修改后
_DEFAULT_SERVER_PORT = 8888
```

#### 受影响文件（端口 8888 一致性已验证）

| 文件 | 参数/字段 | 值 |
|------|-----------|-----|
| `pharmacy_mplus0/launch/main.launch` | `server_port` | `"8888"` |
| `pharmacy_mplus0/launch/reporter.launch` | `server_port` | `"8888"` |
| `pharmacy_mplus0/launch/race_bringup.launch` | `server_port` | `"8888"` |
| `pharmacy_mplus0/config/tcp.yaml` | `server.port` | `8888` |
| `pharmacy_mplus0/scripts/tcp_reporter.py` | `_DEFAULT_SERVER_PORT` | `8888` (已修正) |

---

### #8 移除 strategy.yaml 中未使用的 `dual_car_publish_allow_when_returning` 配置

| 项目 | 内容 |
|------|------|
| **日期** | 2026-06-01 |
| **文件** | `pharmacy_mplus0/config/strategy.yaml` |
| **修改类型** | 清理死配置 |
| **严重程度** | P3 - 配置与代码行为不一致 |

#### 问题描述

`strategy.yaml` 第 49 行定义了 `dual_car_publish_allow_when_returning: true`，但 `main_controller.py` 的 `__init__` 从未读取该配置。`_dual_publish_allow_peer_start()` 仅检查 `_dual_car_enabled` 和 `_dual_allow_sent_this_round`，不会因为这个 YAML 值为 `false` 而跳过发送。

此外，该配置本身是冗余的——用户启用 `dual_car_enabled: true` 即表示需要双车循环协作，"不放行对车"的使用场景不存在。

#### 后果

- 如果用户在 YAML 中将该项改为 `false`，期望阻止放行，但实际放行照常发生，配置与实际行为不一致
- 增加后续维护者的理解成本

#### 修改内容

从 `strategy.yaml` 中直接删除该行（第 49 行）：

```yaml
# 删除前
dual_car_signal_topic: "/dual_car_signal"
# 完成配送后是否发布允许对车出发信号。
dual_car_publish_allow_when_returning: true

# 删除后
dual_car_signal_topic: "/dual_car_signal"
```

#### 受影响检查

| 检查项 | 结论 |
|--------|------|
| `main_controller.py` 读取 `strategy.get("dual_car_publish_allow_when_returning", ...)` | 不读取，无影响 |
| `_dual_publish_allow_peer_start()` 行为 | 不变，始终在配送完成后放行对车 |
| 双车循环逻辑 | 无任何影响 |

#### 排查提示

如果后续现场调试时发现双车模式下仍能正常放行对车但 YAML 中找不到 `dual_car_publish_allow_when_returning`，说明该配置已从 YAML 中移除，放行行为由 `dual_car_enabled` 控制，属于正常现象。

---

### #9 修复 `race_bringup.launch` → `base_camera_nav.launch` 参数传递不匹配

| 项目 | 内容 |
|------|------|
| **日期** | 2026-06-02 |
| **文件** | `pharmacy_mplus0/launch/race_bringup.launch`、`pharmacy_mplus0/launch/base_camera_nav.launch` |
| **修改类型** | bug修复 |
| **严重程度** | P1 - 导致双车模式无法启动 |

#### 问题描述

`race_bringup.launch` 在 include `base_camera_nav.launch` 时传入了 5 个参数：

| 传入参数 | base_camera_nav.launch 是否声明 |
|----------|-------------------------------|
| `start_base` | 未声明 |
| `start_navigation` | 已声明 |
| `start_camera` | 已声明 |
| `start_video_server` | 已声明 |
| `planner` | 未声明 |

ROS launch 机制要求 `<include>` 传入的所有 `<arg>` 必须在被 include 的文件中有对应的 `<arg>` 声明，否则报错：

```
RLException: unused args [planner, start_base] for include of [base_camera_nav.launch]
```

#### 后果

双车调试时执行 `roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1 dual_car_enabled:=true` 直接启动失败。

#### 修改内容

**文件 1：`base_camera_nav.launch`**

| 修改项 | 说明 |
|--------|------|
| 新增 `<arg name="planner" default="teb">` | 声明 `planner` 参数（第 27 行） |
| 两处 `value="teb"` → `value="$(arg planner)"` | 将硬编码的 planner 值改为引用 arg 变量，使参数可透传 |

**文件 2：`race_bringup.launch`**

| 修改项 | 说明 |
|--------|------|
| 移除 `<arg name="start_base" value="$(arg start_base)"/>` | `base_camera_nav.launch` 不接收也不使用该参数（底盘已由 `robot_navigation.launch` 一并启动） |

> 注意：`race_bringup.launch` 中 `start_base` 的 `<arg>` 定义仍保留，但它已成为 dead code，不再参与 include 传参。

---

### #10 移除双车 ROS topic 通信方案，统一使用 TCP

| 项目 | 内容 |
|------|------|
| **日期** | 2026-06-03 |
| **文件** | `config/strategy.yaml`、`launch/race_bringup.launch`、`launch/main.launch`、`scripts/main_controller.py`、`src/pharmacy_mplus0/competition_io.py`、`src/pharmacy_mplus0/constants.py`、`README_DUAL_CAR.md`、`README_COMMUNICATION.md`、`README_CAR2_MIGRATION.md` |
| **修改类型** | 架构清理 |
| **严重程度** | P0 - 双车通信方案变更 |

#### 问题描述

项目此前存在两套双车通信方案：
- **TCP 模式**（`dual_car_comm_mode:=tcp`）：两车独立 ROS Master，通过 TCP :9001 通信
- **ROS topic 模式**（`dual_car_comm_mode:=ros_topic`）：两车共用同一个 ROS Master，通过 `/dual_car_signal` 和 `/current_qr_task` 通信

最终比赛方案确定为两车独立 ROS Master + TCP 通信，ROS topic 方案需彻底移除。

#### 修改内容

分五个阶段执行，以下为各阶段摘要：

**阶段 1：固定双车通信模式为 TCP**

| 文件 | 修改 |
|------|------|
| `config/strategy.yaml` | 删除 `dual_car_comm_mode`、`dual_car_signal_topic` 配置项 |
| `launch/race_bringup.launch` | 删除 `dual_car_comm_mode` arg 及传递 |
| `launch/main.launch` | 删除 `dual_car_comm_mode` arg 及 param |
| `scripts/main_controller.py` | 删除 `_dual_car_comm_mode` 变量及所有分支判断；`dual_car_enabled=true` + `peer_ip` 非空时启动 `DualCarTcpBridge`；`peer_ip` 为空时打印 warning 不崩溃 |

**阶段 2：清理 `/dual_car_signal` 跨车逻辑**

| 文件 | 修改 |
|------|------|
| `scripts/main_controller.py` | 删除 `/dual_car_signal` 订阅者、`_cb_dual_car_signal()`、`_dual_try_parse_remote_task()`；清理 `ALLOW_START_WITH_TASK` 类型兼容 |
| `src/pharmacy_mplus0/competition_io.py` | 删除 `_pub_dual_signal` publisher 和 `publish_dual_signal()` 方法 |
| `src/pharmacy_mplus0/constants.py` | 删除 `TOPIC_DUAL_CAR_SIGNAL` 常量 |

**阶段 3：清理 `/current_qr_task` 跨车逻辑**

| 文件 | 修改 |
|------|------|
| `scripts/main_controller.py` | 删除 `/current_qr_task` 订阅者、`_cb_peer_qr_task()` |
| `src/pharmacy_mplus0/competition_io.py` | 删除 `_pub_qr_task` publisher 和 `publish_qr_task()` 方法 |

**阶段 4：最终审查**

`constants.py` 中 `TOPIC_CURRENT_QR_TASK` 常量保留（`detection_monitor.py` 仍引用），但已无发布者，monitor 中该字段将显示"无数据"。

**阶段 5：README 更新**

三份 README 中所有 `ros_topic` 模式说明、`rostopic pub/echo` 调试命令、`dual_car_comm_mode` 启动参数均已删除。

#### 删除的方法和接口

| 已删除 | 原用途 | 替代方案 |
|--------|--------|---------|
| `CompetitionIO.publish_dual_signal()` | ROS topic 发布 `/dual_car_signal` | `DualCarTcpBridge.send({"type":"allow_start",...})` |
| `CompetitionIO.publish_qr_task()` | ROS topic 发布 `/current_qr_task` | `DualCarTcpBridge.send({"type":"qr_task",...})` |
| `_cb_dual_car_signal()` | 订阅 `/dual_car_signal` | `_dual_handle_allow_start_signal()` 由 TCP `allow_start` 触发 |
| `_cb_peer_qr_task()` | 订阅 `/current_qr_task` | `_dual_handle_peer_qr_task_payload/clear()` 由 TCP `qr_task`/`qr_task_clear` 触发 |
| `_dual_try_parse_remote_task()` | 解析 ROS topic JSON 远程任务 | `_dual_handle_remote_task_data()` 由 TCP `remote_task` 触发 |

#### 调试影响

| 旧调试方式 | 状态 |
|------------|------|
| `rostopic echo /dual_car_signal` | 不再有效（publisher 和 subscriber 已删除） |
| `rostopic pub /dual_car_signal ...` | 不再有效 |
| `rostopic echo /current_qr_task` | 不再有效（publisher 已删除，仅 `detection_monitor` 仍在订阅但无数据） |
| `roslaunch ... dual_car_comm_mode:=ros_topic` | 参数已删除，传入将被忽略 |
| `nc -l 9001` 监听 TCP 消息 | 推荐替代，直接查看 JSON 消息 |
| `echo '{"type":"allow_start",...}' \| nc IP 9001` | 推荐替代，手动模拟 TCP 消息 |

#### 保留的内部话题

以下本车内部 ROS topic 不受影响，裁判上报和语音播报正常：

```
/current_task    — 当前任务状态
/cv1_result      — 识别板二结果
/cv2_result      — 识别板一结果
/announce_request — 语音播报事件
```

---

## 待修复问题清单

以下为检查报告中发现但尚未修复的问题：

| 编号 | 优先级 | 简述 | 涉及文件 |
|------|--------|------|---------|
| #2 | P0 | IMU 帧名不一致：`IMU_link` vs `base_imu_Link` | `ekf_params.yaml`, `art_racecar.py`, 静态TF |
| #3 | P1 | 底盘参数 `base_kv` 不一致 (1.089 vs 1.0) | `robot_lidar.launch`, `EPRobot_start.launch` |
| #4 | P1 | `/cv1_result` 话题有双重发布者 | `board2_detector.py`, `competition_io.py` |
| #6 | P2 | `robot_navigation/package.xml` 缺少所有运行时依赖声明 | `robot_navigation/package.xml` |
