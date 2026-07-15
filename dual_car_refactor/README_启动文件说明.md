# 智慧药房启动文件说明

本文档说明当前推荐的启动方式：**主业务节点和裁判通信节点分开启动**。

这样做的目的：

- 主业务终端主要查看导航、识别、双车 TCP、状态机日志；
- 裁判通信节点单独开终端，避免高频发送日志刷屏；
- 双车系统仍然通过 `CAR_ID` 区分一号车和二号车；
- 同一套代码、同一套 launch 文件可以同时适配两辆车。

---

## 1. 启动文件列表

当前推荐使用两个 launch 文件：

```text
pharmacy_main_no_referee.launch
referee_only.launch
```

### 1.1 pharmacy_main_no_referee.launch

主业务启动文件，不启动裁判通信节点。

主要启动内容：

```text
底盘 / 导航 / 摄像头驱动 / web_video_server
双车 TCP 通信节点 tcp_link_v2.py
视觉识别节点 F1_detect_code_v5.py
药房导航状态机 F1_yaofang_v5.py
```

启动顺序设计为：

```text
先启动基础系统和双车 TCP
再延时启动视觉识别
最后延时启动药房状态机
```

这样可以减少药房状态机过早启动，而通信、识别节点尚未准备好的风险。

### 1.2 referee_only.launch

裁判通信单独启动文件。

只启动：

```text
referee_client_v3.py
```

建议单独开一个终端运行，避免裁判通信高频打印影响主业务日志阅读。

---

## 2. 文件放置位置

将两个 launch 文件放到：

```bash
~/robot_ws/src/pharmacy_pkg/launch/
```

推荐目录结构：

```text
pharmacy_pkg/
├── launch/
│   ├── pharmacy_main_no_referee.launch
│   └── referee_only.launch
├── scripts/
│   ├── F1_detect_code_v5.py
│   ├── F1_yaofang_v5.py
│   ├── referee_client_v3.py
│   ├── tcp_link_v2.py
│   ├── dual_car_config.py
│   └── board1_selection.py
├── pictures/
│   └── board_2/
└── yuyingwenjian/
```

---

## 3. 启动前检查

### 3.1 确认脚本有执行权限

```bash
cd ~/robot_ws/src/pharmacy_pkg/scripts
chmod +x F1_detect_code_v5.py
chmod +x F1_yaofang_v5.py
chmod +x referee_client_v3.py
chmod +x tcp_link_v2.py
```

### 3.2 确认车号配置

系统通过环境变量 `CAR_ID` 区分车辆。

启动一号车时：

```bash
car_id:=1
```

启动二号车时：

```bash
car_id:=2
```

不需要手动 `export CAR_ID`，launch 文件会自动传入：

```xml
<env name="CAR_ID" value="$(arg car_id)"/>
```

### 3.3 确认现场参数

启动前必须检查 `dual_car_config.py`：

```python
CARS[1]["tcp"]["peer_ip"]
CARS[2]["tcp"]["peer_ip"]
```

两辆车的对方 IP 必须填写正确，否则双车 done 消息和识别板一共享结果无法正常传输。

还要确认：

```python
COMMON["board2"]["force_label_for_debug"] = None
```

正式比赛时必须为 `None`，否则识别板二会使用调试强制结果。

调试阶段建议将下面两个等待超时改为 20~60 秒：

```python
COMMON["nav"]["board1_wait_timeout_sec"]
COMMON["nav"]["board2_wait_timeout_sec"]
```

如果保持为 `0`，表示沿用旧逻辑：一直等待识别结果。

---

## 4. 推荐启动方式

每辆车建议开两个终端。

---

## 5. 一号车启动

### 终端 1：启动主业务

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1
```

### 终端 2：启动裁判通信

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg referee_only.launch car_id:=1
```

---

## 6. 二号车启动

### 终端 1：启动主业务

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=2
```

### 终端 2：启动裁判通信

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg referee_only.launch car_id:=2
```

---

## 7. 常用启动参数

### 7.1 只启动主业务节点，不启动底盘、导航、摄像头

如果底盘、导航、摄像头已经由其他 launch 启动，可以使用：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1 start_base:=false
```

二号车：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=2 start_base:=false
```

### 7.2 不启动摄像头驱动

如果摄像头已经启动：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1 start_camera:=false
```

### 7.3 不启动 web_video_server

如果 `web_video_server` 已经启动，或者不需要网页视频流：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1 start_video_server:=false
```

### 7.4 调整识别节点和状态机延时启动时间

默认：

```xml
<arg name="detect_start_delay" default="3"/>
<arg name="nav_start_delay" default="6"/>
```

含义：

```text
视觉识别节点延时 3 秒启动
药房状态机延时 6 秒启动
```

如果现场电脑启动较慢，可以加大延时：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1 detect_start_delay:=5 nav_start_delay:=10
```

### 7.5 修改双车 done 重复发布次数

默认：

```xml
<arg name="dual_publish_repeat" default="3"/>
<arg name="dual_publish_interval" default="0.10"/>
```

含义：

```text
每次释放另一辆车时，重复发布 3 次 done 消息
每次间隔 0.10 秒
```

如果现场网络不稳定，可以适当增加：

```bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1 dual_publish_repeat:=5
```

---

## 8. 主业务 launch 参数说明

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `car_id` | `1` | 车辆编号，支持 `1` 或 `2` |
| `output` | `screen` | 日志输出到终端，或改为 `log` |
| `start_base` | `true` | 是否启动基础系统 |
| `start_navigation` | `true` | 是否启动导航 |
| `start_camera` | `true` | 是否启动 Astra 摄像头 |
| `start_video_server` | `true` | 是否启动 web_video_server |
| `use_race_init` | `true` | 是否使用 `robot_race_init.launch` |
| `start_dual_tcp` | `true` | 是否启动双车 TCP 节点 |
| `start_detect` | `true` | 是否启动视觉识别节点 |
| `start_nav_pharmacy` | `true` | 是否启动药房状态机 |
| `detect_start_delay` | `3` | 视觉识别节点延时启动秒数 |
| `nav_start_delay` | `6` | 药房状态机延时启动秒数 |
| `dual_publish_repeat` | `3` | 双车 done 重复发布次数 |
| `dual_publish_interval` | `0.10` | 双车 done 重复发布间隔 |

---

## 9. 裁判通信 launch 参数说明

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `car_id` | `1` | 车辆编号，支持 `1` 或 `2` |
| `output` | `screen` | 裁判通信日志输出位置 |
| `respawn_referee` | `true` | 裁判通信节点异常退出后是否自动重启 |

---

## 10. 日志查看建议

主业务终端重点观察：

```text
nav_pharmacy
detect_abc
dual_car_tcp_link
move_base
```

裁判通信终端重点观察：

```text
referee_client_node
```

如果裁判通信日志太多，但节点运行正常，可以将裁判通信输出改为 `log`：

```bash
roslaunch pharmacy_pkg referee_only.launch car_id:=1 output:=log
```

---

## 11. 常见问题

### 11.1 提示找不到 launch 文件

确认文件是否放在：

```bash
~/robot_ws/src/pharmacy_pkg/launch/
```

然后重新编译并加载环境：

```bash
cd ~/robot_ws
catkin_make
source devel/setup.bash
```

### 11.2 提示节点没有执行权限

执行：

```bash
cd ~/robot_ws/src/pharmacy_pkg/scripts
chmod +x F1_detect_code_v5.py
chmod +x F1_yaofang_v5.py
chmod +x referee_client_v3.py
chmod +x tcp_link_v2.py
```

### 11.3 提示找不到 Python 模块

确认以下文件在同一个 Python 可搜索目录中，推荐放在 `pharmacy_pkg/scripts/`：

```text
dual_car_config.py
board1_selection.py
F1_detect_code_v5.py
F1_yaofang_v5.py
referee_client_v3.py
tcp_link_v2.py
```

如果仍然找不到，可以临时在终端执行：

```bash
export PYTHONPATH=$PYTHONPATH:~/robot_ws/src/pharmacy_pkg/scripts
```

### 11.4 双车无法轮流启动

优先检查：

```python
CARS[1]["tcp"]["peer_ip"]
CARS[2]["tcp"]["peer_ip"]
```

并确认两车网络互通：

```bash
ping 对方车IP
```

还要确认两车的端口配置没有写反，且没有被防火墙拦截。

### 11.5 裁判系统连接不上

检查 `dual_car_config.py`：

```python
COMMON["referee"]["server_ip"]
COMMON["referee"]["server_port"]
```

并确认裁判服务器已经启动，且本车能 ping 通裁判服务器。

### 11.6 include robot_race_init.launch 报参数错误

如果报错类似：

```text
unused args [planner, open_rviz]
```

说明 `robot_race_init.launch` 不接受这两个参数。

可以在 `pharmacy_main_no_referee.launch` 中删除对应 include 里的：

```xml
<arg name="planner" value="teb"/>
<arg name="open_rviz" value="false"/>
```

---

## 12. 推荐现场启动流程

一号车：

```bash
# 终端 1
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=1

# 终端 2
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg referee_only.launch car_id:=1
```

二号车：

```bash
# 终端 1
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg pharmacy_main_no_referee.launch car_id:=2

# 终端 2
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_pkg referee_only.launch car_id:=2
```

---

## 13. 备注

当前方案不依赖 launch 自动弹出新终端，而是手动开两个终端启动。

原因是不同机器人系统环境可能没有图形桌面，或者没有 `gnome-terminal`。手动分终端最稳定，也最适合比赛现场调试。
