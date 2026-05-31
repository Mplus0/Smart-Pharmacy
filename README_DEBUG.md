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

`tcp_reporter.py` 代码中 `_DEFAULT_SERVER_PORT` 硬编码为 `8888`，而所有 launch 文件
(`main.launch`、`reporter.launch`、`race_bringup.launch`) 和 `tcp.yaml` 配置文件中
默认端口均为 `9999`。

如果用户直接通过 `rosrun pharmacy_mplus0 tcp_reporter.py` 启动节点
（不经过 launch 文件传入 `~server_port`），节点会使用 `8888` 作为目标端口，
而裁判软件实际监听的是 `9999`，导致 TCP 连接失败。

#### 修改内容

| 文件 | 行号 | 参数 | 原值 | 新值 |
|------|------|------|------|------|
| `tcp_reporter.py` | 75 | `_DEFAULT_SERVER_PORT` | `8888` | `9999` |

```python
# 修改前
_DEFAULT_SERVER_PORT = 8888

# 修改后
_DEFAULT_SERVER_PORT = 9999
```

#### 受影响文件（端口 9999 一致性已验证）

| 文件 | 参数/字段 | 值 |
|------|-----------|-----|
| `pharmacy_mplus0/launch/main.launch` | `server_port` | `"9999"` |
| `pharmacy_mplus0/launch/reporter.launch` | `server_port` | `"9999"` |
| `pharmacy_mplus0/launch/race_bringup.launch` | `server_port` | `"9999"` |
| `pharmacy_mplus0/config/tcp.yaml` | `server.port` | `9999` |
| `pharmacy_mplus0/scripts/tcp_reporter.py` | `_DEFAULT_SERVER_PORT` | `9999` (已修正) |

---

## 待修复问题清单

以下为检查报告中发现但尚未修复的问题：

| 编号 | 优先级 | 简述 | 涉及文件 |
|------|--------|------|---------|
| #2 | P0 | IMU 帧名不一致：`IMU_link` vs `base_imu_Link` | `ekf_params.yaml`, `art_racecar.py`, 静态TF |
| #3 | P1 | 底盘参数 `base_kv` 不一致 (1.089 vs 1.0) | `robot_lidar.launch`, `EPRobot_start.launch` |
| #4 | P1 | `/cv1_result` 话题有双重发布者 | `board2_detector.py`, `competition_io.py` |
| #5 | P2 | `base_camera_nav.launch` 中 `<arg>` 传递语法不规范 | `base_camera_nav.launch` |
| #6 | P2 | `robot_navigation/package.xml` 缺少所有运行时依赖声明 | `robot_navigation/package.xml` |
