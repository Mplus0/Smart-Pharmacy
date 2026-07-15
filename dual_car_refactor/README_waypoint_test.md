# Waypoint Test README（小车点位测试说明）

本文档用于指导你**单独测试小车前往 A/B/C、识别板、化验窗口、起点等点位**，用于验证 `dual_car_config.py` 中点位坐标和朝向是否准确。

> 建议文件名使用英文，例如 `README_waypoint_test.md`。  
> 内容可以写中文，但 ROS package 内部尽量不要出现中文文件名或中文目录名，避免 ROS Melodic / Python2 在扫描 package 时出现 ASCII 编码问题。

---

## 1. 测试目的

点位测试的目标不是跑完整比赛流程，而是逐个验证：

```text
A / B / C 取样窗口
board1 识别板一
board2 识别板二
lab1 / lab2 / lab3 / lab4 化验窗口
home 起点
```

每个点主要检查三件事：

1. 小车能否成功到达；
2. 停车位置是否对准窗口或识别板；
3. 停车朝向是否正确，摄像头是否能看到目标区域。

---

## 2. 测试原则

点位测试时，**不要启动完整药房状态机**。

因为完整状态机会自动执行：

```text
起点 -> 识别板一 -> A/B/C -> 识别板二 -> 化验窗口 -> 回起点
```

如果只是想测某一个点，应该只启动基础导航系统，然后用测试节点单独给 `move_base` 发送目标点。

推荐结构：

```text
终端 1：启动基础导航
终端 2：发送单个测试点位
终端 3：可选，观察话题 / 急停
```

---

## 3. 测试文件

推荐使用下面两个测试文件：

```text
scripts/test_move_to_waypoint.py
launch/test_move_to_waypoint.launch
```

建议放置位置：

```bash
~/robot_ws/src/pharmacy_pkg/scripts/test_move_to_waypoint.py
~/robot_ws/src/pharmacy_pkg/launch/test_move_to_waypoint.launch
```

给脚本执行权限：

```bash
chmod +x ~/robot_ws/src/pharmacy_pkg/scripts/test_move_to_waypoint.py
```

重新编译并加载环境：

```bash
cd ~/robot_ws
catkin_make
source devel/setup.bash
```

---

## 4. 点位来源

测试节点读取的点位来自：

```bash
~/robot_ws/src/pharmacy_pkg/scripts/dual_car_config.py
```

重点看：

```python
COMMON["nav"]["waypoints"]
COMMON["nav"]["euler_angles"]
CARS[1]["home_pose"]
CARS[2]["home_pose"]
```

当前点位格式一般是：

```python
"点位名": (x, y, 朝向索引)
```

例如：

```python
"waypoints": {
    "C": (1.382, 1.828, 0),
    "A": (0.585, 2.394, 1),
    "B": (1.434, 2.929, 2),
    "lab4": (-0.906, 0.921, 3),
    "lab3": (-1.871, 1.427, 4),
    "lab2": (-0.870, 1.734, 7),
    "lab1": (-1.794, 2.271, 6),
    "board2": (-0.454, 3.791, 8),
    "board1": (0.591, -0.269, 9),
}
```

第三个数字不是角度本身，而是朝向索引，对应：

```python
COMMON["nav"]["euler_angles"]
```

如果位置不准，改 `x, y`。  
如果车头方向不准，改第三个数字，或者调整 `euler_angles` 中对应角度。

---

## 5. 启动基础导航

### 5.1 推荐启动方式

开终端 1，只启动底盘、雷达、定位、地图、`move_base`，不要启动识别、药房状态机、双车 TCP。

一号车：

```bash
source ~/robot_ws/devel/setup.bash

roslaunch pharmacy_pkg pharmacy_main_no_referee.launch \
  car_id:=1 \
  start_dual_tcp:=false \
  start_detect:=false \
  start_nav_pharmacy:=false \
  start_camera:=false \
  start_video_server:=false
```

二号车：

```bash
source ~/robot_ws/devel/setup.bash

roslaunch pharmacy_pkg pharmacy_main_no_referee.launch \
  car_id:=2 \
  start_dual_tcp:=false \
  start_detect:=false \
  start_nav_pharmacy:=false \
  start_camera:=false \
  start_video_server:=false
```

这条命令的含义：

```text
启动基础导航
不启动双车 TCP
不启动视觉识别
不启动药房状态机
不启动摄像头
不启动 web_video_server
```

---

### 5.2 如果你有单独的基础系统 launch

如果你已经有基础系统启动文件，例如：

```text
base_camera_nav.launch
```

也可以这样启动：

```bash
source ~/robot_ws/devel/setup.bash

roslaunch pharmacy_pkg base_camera_nav.launch \
  start_camera:=false \
  start_video_server:=false
```

点位测试阶段，如果不需要看摄像头，建议先关闭摄像头和视频服务，减少终端日志干扰。

---

## 6. 测试前准备

### 6.1 确认 ROS 环境

```bash
echo $ROS_MASTER_URI
echo $ROS_IP
hostname -I
```

单车本地测试时，建议每辆车使用自己的 ROS master。

例如一号车：

```bash
export ROS_MASTER_URI=http://192.168.124.3:11311
export ROS_IP=192.168.124.3
```

二号车：

```bash
export ROS_MASTER_URI=http://192.168.124.9:11311
export ROS_IP=192.168.124.9
```

实际 IP 以现场为准。

---

### 6.2 确认 move_base 正常

启动基础导航后检查：

```bash
rostopic list | grep move_base
```

应该能看到类似：

```text
/move_base/goal
/move_base/status
/move_base/result
/move_base/cancel
```

也可以检查 action server 是否存在：

```bash
rostopic echo /move_base/status
```

---

### 6.3 在 RViz 里设置初始位姿

点位测试前必须确认定位是准的。

在 RViz 里使用：

```text
2D Pose Estimate
```

把小车当前位置和朝向标到地图上。

如果定位没有初始化，直接发目标点会导致小车跑偏。

---

## 7. 先 dry run，只打印点位不运动

第一次测试建议先 dry run，确认目标点坐标和朝向正确。

测试 A 点：

```bash
source ~/robot_ws/devel/setup.bash

roslaunch pharmacy_pkg test_move_to_waypoint.launch \
  car_id:=1 \
  target:=A \
  dry_run:=true
```

二号车：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch \
  car_id:=2 \
  target:=A \
  dry_run:=true
```

dry run 应该只打印目标点信息，不会真正发送导航目标。

---

## 8. 单独测试某个点

开终端 2，发送目标点。

### 8.1 测试 A / B / C 取样窗口

测试 A 点：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=A
```

测试 B 点：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=B
```

测试 C 点：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=C
```

二号车测试时，把 `car_id:=1` 改成：

```bash
car_id:=2
```

---

### 8.2 测试识别板点位

测试识别板一：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=board1
```

测试识别板二：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=board2
```

测试识别板点位时，重点观察：

```text
车停下后，摄像头是否正对识别板
识别板是否在画面中间
二维码或模板区域是否完整
车头朝向是否合适
```

如果位置到了但画面偏，优先调整朝向索引或对应角度。

---

### 8.3 测试化验窗口

测试 1 号化验窗口：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab1
```

测试 2 号化验窗口：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab2
```

测试 3 号化验窗口：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab3
```

测试 4 号化验窗口：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab4
```

注意：`lab1~lab4` 是程序里的点位名，实际窗口含义要和 `dual_car_config.py` 里的 `lab_info` 对照确认。

---

### 8.4 测试回起点

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=home
```

`home` 通常来自：

```python
CARS[1]["home_pose"]
CARS[2]["home_pose"]
```

所以一号车和二号车的 home 可以不一样。

---

## 9. 推荐测试顺序

建议按安全程度从近到远测试。

一号车建议顺序：

```text
home
A
B
C
board1
board2
lab1
lab2
lab3
lab4
home
```

如果场地狭窄，先测最近、最安全的点，不要一开始就测远点或贴边点。

每测试一个点，先确认小车能安全停住，再测试下一个点。

---

## 10. 如何判断点位是否准确

每个点位都建议记录以下内容：

| 点位 | 是否到达 | 位置是否准确 | 朝向是否准确 | 需要修改 |
|---|---|---|---|---|
| A | 是/否 | 偏左/偏右/偏前/偏后 | 正确/偏转 | x/y/yaw |
| B | 是/否 | 偏左/偏右/偏前/偏后 | 正确/偏转 | x/y/yaw |
| C | 是/否 | 偏左/偏右/偏前/偏后 | 正确/偏转 | x/y/yaw |
| board1 | 是/否 | 画面是否居中 | 摄像头是否正对 | x/y/yaw |
| board2 | 是/否 | 画面是否居中 | 摄像头是否正对 | x/y/yaw |
| lab1 | 是/否 | 是否对准窗口 | 正确/偏转 | x/y/yaw |
| lab2 | 是/否 | 是否对准窗口 | 正确/偏转 | x/y/yaw |
| lab3 | 是/否 | 是否对准窗口 | 正确/偏转 | x/y/yaw |
| lab4 | 是/否 | 是否对准窗口 | 正确/偏转 | x/y/yaw |
| home | 是/否 | 是否回到起点 | 正确/偏转 | x/y/yaw |

---

## 11. 点位不准时怎么改

打开：

```bash
gedit ~/robot_ws/src/pharmacy_pkg/scripts/dual_car_config.py
```

或者用命令行编辑：

```bash
nano ~/robot_ws/src/pharmacy_pkg/scripts/dual_car_config.py
```

找到：

```python
COMMON["nav"]["waypoints"]
```

例如 A 点：

```python
"A": (0.585, 2.394, 1)
```

含义是：

```text
x = 0.585
y = 2.394
yaw_index = 1
```

### 11.1 位置偏差

如果小车停得太靠前、靠后、靠左、靠右，就微调 `x, y`。

建议每次小幅修改：

```text
0.03 m ~ 0.10 m
```

不要一次改太大。

### 11.2 朝向偏差

如果位置对，但车头方向不对，修改第三个数字：

```python
"A": (0.585, 2.394, 1)
```

把 `1` 改成其他朝向索引，例如：

```python
"A": (0.585, 2.394, 0)
```

或者修改：

```python
COMMON["nav"]["euler_angles"]
```

里面对应索引的角度。

常见角度参考：

```text
0.0                  朝向地图 x 正方向
1.57079632679         约 +90°
-1.57079632679        约 -90°
3.14159265359         约 180°
-3.14159265359        约 -180°
```

---

## 12. 修改后是否需要重新编译

如果只修改 `dual_car_config.py` 这种 Python 配置文件，一般不需要重新 `catkin_make`，但建议重新启动测试节点，让它重新读取配置。

推荐流程：

```bash
# 修改 dual_car_config.py 后

# 重新运行点位测试
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=A
```

如果你不确定环境是否更新，可以执行：

```bash
cd ~/robot_ws
source devel/setup.bash
```

---

## 13. 急停方法

如果小车跑偏，先在当前 roslaunch 终端按：

```text
Ctrl + C
```

然后发布零速度：

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "{}" -1
```

如果需要持续急停：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{}"
```

确认小车停稳后，再按：

```text
Ctrl + C
```

结束急停命令。

---

## 14. 常见问题

### 14.1 提示 move_base 连接不上

检查基础导航是否启动：

```bash
rostopic list | grep move_base
```

如果没有 `/move_base` 相关话题，说明基础导航没有起来。

---

### 14.2 小车不动

检查：

```bash
rostopic echo /move_base/status
rostopic echo /cmd_vel
```

可能原因：

```text
定位没有初始化
目标点在障碍物里
move_base 没启动
底盘控制节点没启动
急停还在发布 0 速度
```

---

### 14.3 小车规划失败

可以尝试清除代价地图：

```bash
rosservice call /move_base/clear_costmaps "{}"
```

如果仍失败，检查目标点是否太靠墙、太靠障碍物、或不在地图可通行区域。

---

### 14.4 到点后位置对，但姿态不对

优先改点位第三个值，也就是朝向索引：

```python
"A": (0.585, 2.394, 1)
```

如果几个点都朝向类似偏差，再考虑修改 `euler_angles` 中的角度值。

---

### 14.5 一启动就报 run_id 不一致

说明旧 ROS 进程没关干净。

执行：

```bash
pkill -f roslaunch
pkill -f roscore
pkill -f rosmaster
pkill -f rosout
sleep 2
```

然后重新启动基础导航。

---

### 14.6 出现 ASCII 编码报错

ROS package 内部不要放中文文件名或中文目录名。

检查：

```bash
find ~/robot_ws/src/pharmacy_pkg -print | LC_ALL=C grep -n '[^ -~]'
```

如果输出了中文文件名，建议改成英文文件名，或者移动到 package 外部。

例如：

```bash
mv README_启动文件说明.md README_launch_usage.md
mv 识别板一优化建议.md README_board1_optimization.md
```

---

## 15. 推荐完整测试流程

下面是一套推荐流程。

### 第一步：清理旧 ROS 进程

```bash
pkill -f roslaunch
pkill -f roscore
pkill -f rosmaster
pkill -f rosout
sleep 2
```

### 第二步：启动基础导航

```bash
source ~/robot_ws/devel/setup.bash

roslaunch pharmacy_pkg pharmacy_main_no_referee.launch \
  car_id:=1 \
  start_dual_tcp:=false \
  start_detect:=false \
  start_nav_pharmacy:=false \
  start_camera:=false \
  start_video_server:=false
```

### 第三步：RViz 设置初始位姿

使用：

```text
2D Pose Estimate
```

确认小车在地图上的位置和方向正确。

### 第四步：dry run 检查 A 点

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=A dry_run:=true
```

### 第五步：正式前往 A 点

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=A
```

### 第六步：记录偏差并修改配置

修改：

```bash
~/robot_ws/src/pharmacy_pkg/scripts/dual_car_config.py
```

### 第七步：重复测试

继续测试：

```bash
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=B
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=C
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=board1
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=board2
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab1
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab2
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab3
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=lab4
roslaunch pharmacy_pkg test_move_to_waypoint.launch car_id:=1 target:=home
```

---

## 16. 最终确认标准

所有点位确认通过后，至少满足：

```text
A/B/C：能稳定到达，取样窗口对准
board1：车停下后识别板一完整出现在画面中，二维码区域清晰
board2：车停下后识别板二完整出现在画面中，模板区域清晰
lab1~lab4：能稳定到达对应化验窗口，位置和车头方向合适
home：能回到起点，下一轮出发不会卡住
```

如果每个点连续测试 3 次都能稳定到达，说明点位基本可用。
