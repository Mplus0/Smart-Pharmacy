# 车二迁移指南

将 `pharmacy_mplus0` 代码从车一复制到车二上运行时，需要修改以下配置。

> 车二的 TF 卡由车一镜像烧录而来，网络由 DHCP 自动分配 IP。以下分"系统层"和"代码层"两类说明。

---

## 1. 系统层修改（镜像烧录后必须执行）

以下文件不在 git 仓库内，需要在小车上直接操作。

### 1.1 修改主机名 — `/etc/hostname`

两台车主机名相同会导致网络冲突。车一主机名默认为 `EPRobot`，车二需改为不同值：

```bash
# 在车二上执行
sudo hostnamectl set-hostname EPRobot-2
```

同时检查 `/etc/hosts` 中是否有 `127.0.1.1 EPRobot` 条目，同步更新为新主机名：

```bash
sudo vim /etc/hosts
# 将 127.0.1.1 EPRobot 改为 127.0.1.1 EPRobot-2
```

### 1.2 配置 ROS Master — `~/.bashrc`

双车模式下两车需连接同一个 ROS Master。镜像烧录后两车的 `~/.bashrc` 中 ROS Master 均为 `http://127.0.0.1:11311`（各自本地），需要指定一台车作为 Master。

**方案：车一作为 ROS Master，车二指向车一。**

在车二的 `~/.bashrc` 末尾追加：

```bash
export ROS_MASTER_URI=http://192.168.124.3:11311
```

如果 `~/.bashrc` 中存在写死 IP 的 `ROS_IP` 或 `ROS_HOSTNAME`（如 `export ROS_IP=192.168.124.3`），车二上需要删除或改为：

```bash
export ROS_HOSTNAME=$(hostname)
```

### 1.3 建议检查项

| 文件 | 检查内容 | 说明 |
|---|---|---|
| `~/.bashrc` | `export ROBOT_TYPE=...` | 两车硬件型号相同则不用改 |
| `~/.bashrc` | `export LIDAR_TYPE=...` | 雷达型号相同则不用改 |
| `/etc/udev/rules.d/` | 设备串口规则 | 硬件相同不用改（如 `/dev/EPRobot_base`） |
| `/etc/network/interfaces` | 网络配置 | DHCP 自动分配，无需改动 |

---

## 2. 航点坐标（必须修改，无法通过命令行覆盖）

**文件**: [config/waypoints.yaml](config/waypoints.yaml)

车二在场地中的起点位置与车一不同（双车各占一边），航点坐标需要重新实地标定：

```yaml
waypoints:
  exam_A:  {x: 0.785, y: 2.616, yaw: 3.1414}   
  exam_B:  {x: 1.543, y: 2.950, yaw: 1.5708}   
  exam_C:  {x: 1.530, y: 2.085, yaw: 1.5708}   
  lab_1:   {x: -1.639, y: 2.521, yaw: -1.5707} 
  lab_2:   {x: -0.846, y: 1.921, yaw: 0.0}     
  lab_3:   {x: -1.669, y: 1.538, yaw: -1.5707} 
  lab_4:   {x: -0.835, y: 0.950, yaw: -1.5708} 
  start:   {x: 0.0, y: 0.0, yaw: 0.0}          # ← 需重新标定
  board1:  {x: 0.711, y: 0.0, yaw: 0.0}        
  board2:  {x: -0.107, y: 3.913, yaw: 3.1416}  
```

---

## 3. 摄像头视频流 IP（车二的自身 IP）

车一的摄像头 IP 为 `192.168.124.3`，车二的自身 IP 为 `192.168.124.9`。

以下 6 处均可通过命令行 `stream_url:=http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw` 覆盖，也可直接修改文件：

| 文件 | 行号 | 参数 |
|------|------|------|
| [config/vision.yaml](config/vision.yaml#L5) | 5 | `stream_url: "http://192.168.124.3:8080/stream?..."` |
| [launch/race_bringup.launch](launch/race_bringup.launch#L37) | 37 | `<arg name="stream_url" default="http://192.168.124.3:8080/stream?..."` |
| [launch/main.launch](launch/main.launch#L27) | 27 | `<arg name="stream_url" default="http://192.168.124.3:8080/stream?..."` |
| [launch/main_single.launch](launch/main_single.launch#L13) | 13 | `<arg name="stream_url" default="http://192.168.124.3:8080/stream?..."` |
| [launch/detectors.launch](launch/detectors.launch#L16) | 16 | `<arg name="stream_url" default="http://192.168.124.3:8080/stream?..."` |
| [scripts/board2_detector.py](scripts/board2_detector.py#L39) | 39 | `_DEFAULT_STREAM_URL = "http://192.168.124.3:8080/stream?..."` |

---

## 4. car_id（车号）

所有位置默认值为 `"1"`，车二需改为 `"2"`。

以下 6 处均可通过命令行 `car_id:=2` 覆盖，也可直接修改文件：

| 文件 | 行号 | 参数 |
|------|------|------|
| [config/tcp.yaml](config/tcp.yaml#L15) | 15 | `car_id: "1"` |
| [launch/race_bringup.launch](launch/race_bringup.launch#L41) | 41 | `<arg name="car_id" default="1"` |
| [launch/main.launch](launch/main.launch#L31) | 31 | `<arg name="car_id" default="1"` |
| [launch/main_single.launch](launch/main_single.launch#L16) | 16 | `<arg name="car_id" default="1"` |
| [launch/reporter.launch](launch/reporter.launch#L17) | 17 | `<arg name="car_id" default="1"` |
| [scripts/tcp_reporter.py](scripts/tcp_reporter.py#L77) | 77 | `_DEFAULT_CAR_ID = "1"` |

---

## 5. 裁判电脑 IP（通常不需要改）

裁判电脑 IP 为 `192.168.124.2`，两台车通常指向同一台裁判电脑。

如果裁判 IP 发生变化，以下 6 处可通过命令行 `server_ip:=<新IP>` 覆盖，也可直接修改文件：

| 文件 | 行号 | 参数 |
|------|------|------|
| [config/tcp.yaml](config/tcp.yaml#L5) | 5 | `ip: 192.168.124.2` |
| [launch/race_bringup.launch](launch/race_bringup.launch#L39) | 39 | `<arg name="server_ip" default="192.168.124.2"` |
| [launch/main.launch](launch/main.launch#L29) | 29 | `<arg name="server_ip" default="192.168.124.2"` |
| [launch/main_single.launch](launch/main_single.launch#L14) | 14 | `<arg name="server_ip" default="192.168.124.2"` |
| [launch/reporter.launch](launch/reporter.launch#L15) | 15 | `<arg name="server_ip" default="192.168.124.2"` |
| [scripts/tcp_reporter.py](scripts/tcp_reporter.py#L74) | 74 | `_DEFAULT_SERVER_IP = "192.168.124.2"` |

---

## 6. 音频播报

两车为轮流出发，配送时段不重叠，各自播报互不干扰。车二正常启动即可，无需特殊处理。

- **双车模式**：车一和车二各播各的，启用车二的 wav 音频文件即可。
- **单车模式**：同上，正常播报。
- **如需禁用语音**：通过 `audio_dir:=""` 关闭，适用于调试或现场有特殊要求的情况。

---

## 7. 车二启动命令

### 单车模式（正常播报）

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  stream_url:=http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw
```

### 双车模式（轮流出发，各自播报）

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  stream_url:=http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw
```

### 让车二首发（覆盖默认首发车）

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_start_first_car_id:=2
```

---

## 8. 修改优先级总结

| 优先级 | 修改项 | 所在位置 | 类型 |
|--------|--------|------|------|
| P0 | 主机名 | `/etc/hostname` + `/etc/hosts` | 系统层 — 镜像烧录后必须改 |
| P0 | ROS Master 指向 | `~/.bashrc` | 系统层 — 双车模式必须改 |
| P0 | 航点坐标 | `config/waypoints.yaml` | 代码层 — 需重新标定 |
| P0 | 摄像头 stream_url | 6 个文件（可命令行覆盖） | 代码层 |
| P1 | car_id | 6 个文件（可命令行覆盖） | 代码层 |
| P2 | 裁判 IP | 6 个文件（可命令行覆盖） | 代码层 |
| P3 | 语音播报 | 无需额外修改 | — |

---

## 9. 代码同步方式

从车一同步代码到车二（在开发机上执行）：

```bash
scp -r ~/Smart-Pharmacy/pharmacy_mplus0/ EPRobot@192.168.124.9:~/robot_ws/src/pharmacy_mplus0/
```

同步后在车二上确保 Python 脚本有执行权限：

```bash
chmod +x ~/robot_ws/src/pharmacy_mplus0/scripts/*.py
```
