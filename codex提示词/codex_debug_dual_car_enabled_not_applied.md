# Codex 调试提示词：`dual_car_enabled:=true` 未生效，车二直接进入识别板一导航

## 一、问题背景

当前项目为 `pharmacy_mplus0`，用于智慧药房双车协作。现在目标是：

```text
两辆车使用各自独立的 ROS Master
双车之间只通过 TCP 通信
不再依赖同一个 ROS Master 下的 ROS topic 通信
```

我在两辆车上分别使用以下命令启动：

### 车 1

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 dual_car_enabled:=true
```

### 车 2

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 dual_car_enabled:=true
```

按预期，车 1 应该作为主车先出发，车 2 应该在起点等待，直到收到车 1 的 TCP 放行信号后再启动。

但是实际运行结果是：**车 1 和车 2 都显示双车协作关闭，并且车 2 直接进入识别板一导航。**

---

## 二、日志现象

### 车 1 日志关键现象

车 1 的 `tcp_reporter` 正确显示小车编号为 1：

```text
[Reporter]   小车编号: 1
```

但是 `main_controller` 显示：

```text
[main_controller]   双车协作: 关闭
[main_controller] [Main] === INIT === 初始化
[main_controller] [Main] === GOTO_BOARD1 (即将进入第 1 轮) ===
[NavigationClient] 正在前往 board1，超时 40.0 秒
```

说明车 1 虽然命令行传入了 `dual_car_enabled:=true`，但是 `main_controller.py` 实际没有启用双车逻辑。

### 车 2 日志关键现象

车 2 的 `tcp_reporter` 正确显示小车编号为 2：

```text
[Reporter]   小车编号: 2
```

但是 `main_controller` 同样显示：

```text
[main_controller]   双车协作: 关闭
[main_controller] [Main] === INIT === 初始化
[main_controller] [Main] === GOTO_BOARD1 (即将进入第 1 轮) ===
[NavigationClient] 正在前往 board1，超时 40.0 秒
```

随后车 2 甚至成功到达识别板一：

```text
[NavigationClient] 成功到达 board1
[Main] 第 1 轮开始
[Main] === AT_BOARD1 === 等待二维码识别结果
```

这说明车 2 没有进入“起点等待放行”状态，而是仍然按单车模式执行。

---

## 三、初步判断

当前最可能的问题是：

```text
命令行传入的 dual_car_enabled:=true 没有成功传递到 main_controller.py，
或者 main_controller.py 读取参数的命名空间/参数名不正确。
```

也就是说，虽然启动命令中写了：

```bash
dual_car_enabled:=true
```

但是 `main_controller.py` 实际读取到的仍然是：

```text
dual_car_enabled = false
```

所以两辆车都进入了单车流程，直接导航到 `board1`。

---

## 四、请你先做阶段 0：只检查，不修改

请先不要直接修改代码。请先完整检查以下文件，并输出分析结果：

```text
pharmacy_mplus0/
├── launch/
│   ├── race_bringup.launch
│   ├── main.launch
│   ├── main_single.launch
│   └── reporter.launch
├── config/
│   ├── strategy.yaml
│   └── tcp.yaml
├── scripts/
│   ├── main_controller.py
│   └── tcp_reporter.py
└── src/pharmacy_mplus0/
    ├── competition_io.py
    ├── dual_car_tcp.py
    └── constants.py
```

重点检查：

1. `race_bringup.launch` 是否定义了 `dual_car_enabled` 参数。
2. `race_bringup.launch` 是否把 `dual_car_enabled` 正确透传给 `main.launch`。
3. `main.launch` 是否定义了 `dual_car_enabled` 参数。
4. `main.launch` 是否把 `dual_car_enabled` 写入到了 `main_controller` 节点的私有参数中。
5. `main_controller.py` 是否使用正确方式读取该参数，例如：

```python
rospy.get_param("~dual_car_enabled", False)
```

6. 是否存在读取成以下错误形式的情况：

```python
rospy.get_param("dual_car_enabled", False)
rospy.get_param("/dual_car_enabled", False)
```

7. 是否存在 `strategy.yaml` 中的默认值覆盖了 launch 命令行传参。
8. 是否存在 `main_controller.py` 先从 YAML 读取配置，后又覆盖了 launch 私有参数。
9. `car_id` 为什么能够被 `tcp_reporter` 正确读取为 1/2，但 `main_controller` 的双车开关没有生效。请对比 `tcp_reporter.py` 和 `main_controller.py` 的参数读取路径。
10. 当前 TCP 模式相关参数是否也正确传入：

```text
dual_car_comm_mode
dual_car_peer_ip
dual_car_listen_ip
dual_car_listen_port
dual_car_peer_port
```

---

## 五、期望的正确参数结构

启动命令中的：

```bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=2 dual_car_enabled:=true
```

最终应该让 `main_controller` 节点下出现：

```text
/main_controller/car_id = 2
/main_controller/dual_car_enabled = true
/main_controller/dual_car_comm_mode = tcp
```

如果使用 TCP 双车通信，还应能读取：

```text
/main_controller/dual_car_peer_ip
/main_controller/dual_car_listen_ip
/main_controller/dual_car_listen_port
/main_controller/dual_car_peer_port
```

请给出实际代码中这些参数最终落在哪个命名空间。

---

## 六、请提供可执行的检查命令

请你在分析后给出我可以在车上直接执行的命令，用于验证参数是否正确传入。例如：

```bash
rosparam get /main_controller/dual_car_enabled
rosparam get /main_controller/car_id
rosparam get /main_controller/dual_car_comm_mode
rosparam get /main_controller/dual_car_peer_ip
```

也请给出用于搜索参数位置的命令，例如：

```bash
rosparam list | grep dual_car
rosparam list | grep car_id
```

如果你认为参数可能被放在全局命名空间，也请给出检查命令：

```bash
rosparam get /dual_car_enabled
rosparam get /car_id
```

---

## 七、阶段 1：确认原因后再小步修改

在完成阶段 0 的检查后，如果确认是 launch 参数没有透传，请只修改 launch 文件，不要动主控逻辑。

优先检查并修复：

```text
launch/race_bringup.launch
launch/main.launch
```

期望结构如下。

### race_bringup.launch 中应该有

```xml
<arg name="car_id" default="1"/>
<arg name="dual_car_enabled" default="false"/>
<arg name="dual_car_remote_task_enabled" default="false"/>
<arg name="dual_car_comm_mode" default="tcp"/>
<arg name="dual_car_peer_ip" default=""/>
<arg name="dual_car_listen_ip" default="0.0.0.0"/>
<arg name="dual_car_listen_port" default="9001"/>
<arg name="dual_car_peer_port" default="9001"/>
```

如果 `race_bringup.launch` include 了 `main.launch`，应透传：

```xml
<include file="$(find pharmacy_mplus0)/launch/main.launch">
    <arg name="car_id" value="$(arg car_id)"/>
    <arg name="dual_car_enabled" value="$(arg dual_car_enabled)"/>
    <arg name="dual_car_remote_task_enabled" value="$(arg dual_car_remote_task_enabled)"/>
    <arg name="dual_car_comm_mode" value="$(arg dual_car_comm_mode)"/>
    <arg name="dual_car_peer_ip" value="$(arg dual_car_peer_ip)"/>
    <arg name="dual_car_listen_ip" value="$(arg dual_car_listen_ip)"/>
    <arg name="dual_car_listen_port" value="$(arg dual_car_listen_port)"/>
    <arg name="dual_car_peer_port" value="$(arg dual_car_peer_port)"/>
</include>
```

### main.launch 中应该有

```xml
<arg name="car_id" default="1"/>
<arg name="dual_car_enabled" default="false"/>
<arg name="dual_car_remote_task_enabled" default="false"/>
<arg name="dual_car_comm_mode" default="tcp"/>
<arg name="dual_car_peer_ip" default=""/>
<arg name="dual_car_listen_ip" default="0.0.0.0"/>
<arg name="dual_car_listen_port" default="9001"/>
<arg name="dual_car_peer_port" default="9001"/>
```

并且 `main_controller` 节点内应该写入私有参数：

```xml
<node pkg="pharmacy_mplus0" type="main_controller.py" name="main_controller" output="screen" required="true">
    <param name="car_id" value="$(arg car_id)"/>
    <param name="dual_car_enabled" value="$(arg dual_car_enabled)"/>
    <param name="dual_car_remote_task_enabled" value="$(arg dual_car_remote_task_enabled)"/>
    <param name="dual_car_comm_mode" value="$(arg dual_car_comm_mode)"/>
    <param name="dual_car_peer_ip" value="$(arg dual_car_peer_ip)"/>
    <param name="dual_car_listen_ip" value="$(arg dual_car_listen_ip)"/>
    <param name="dual_car_listen_port" value="$(arg dual_car_listen_port)"/>
    <param name="dual_car_peer_port" value="$(arg dual_car_peer_port)"/>
</node>
```

---

## 八、阶段 2：如果是 main_controller.py 读取错误，再小步修改

如果 launch 参数已经正确出现在 `/main_controller/dual_car_enabled`，但日志仍然显示“双车协作: 关闭”，请检查 `main_controller.py` 的读取方式。

应该使用节点私有参数读取：

```python
self._dual_car_enabled = rospy.get_param("~dual_car_enabled", False)
self._car_id = str(rospy.get_param("~car_id", "1"))
self._dual_car_comm_mode = str(rospy.get_param("~dual_car_comm_mode", "tcp"))
self._dual_peer_ip = str(rospy.get_param("~dual_car_peer_ip", ""))
```

如果当前代码从 `strategy.yaml` 中读取了默认值，请确保优先级为：

```text
命令行 launch 参数 > ROS 私有参数 > YAML 默认配置
```

不要让 YAML 中的：

```yaml
dual_car_enabled: false
```

覆盖命令行传入的：

```bash
dual_car_enabled:=true
```

---

## 九、阶段 3：修复后验收标准

修复后重新启动：

### 车 1

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=1 \
  dual_car_enabled:=true \
  dual_car_comm_mode:=tcp \
  dual_car_peer_ip:=192.168.124.9
```

### 车 2

```bash
roslaunch pharmacy_mplus0 race_bringup.launch \
  car_id:=2 \
  dual_car_enabled:=true \
  dual_car_comm_mode:=tcp \
  dual_car_peer_ip:=192.168.124.3
```

预期日志：

### 车 1

```text
双车协作: 启用
双车通信: TCP
car_id=1 start allowed at boot
[Main] === GOTO_BOARD1 ...
```

车 1 可以先出发。

### 车 2

```text
双车协作: 启用
双车通信: TCP
car_id=2 waiting at start
```

车 2 不应该直接进入：

```text
[Main] === GOTO_BOARD1
```

除非已经收到车 1 的 TCP `allow_start` 或 `remote_task` 消息。

---

## 十、注意：这不是本次主因，但需要记录的问题

这次日志中还有以下独立问题，但它们不是“车二直接进入识别板一导航”的主因：

### 1. Board2 模板缺失

两车日志中都有：

```text
[Board2Detector] 模板目录 /home/EPRobot/robot_ws/src/pharmacy_mplus0/templates/board2 中未找到有效模板 (需要 idle.png / wait5.png~wait10.png)
```

请后续检查：

```bash
ls -lh /home/EPRobot/robot_ws/src/pharmacy_mplus0/templates/board2
```

该目录至少应包含：

```text
idle.png
wait5.png
wait6.png
wait7.png
wait8.png
wait9.png
wait10.png
```

### 2. 车二视频流曾短暂打不开

车二日志中曾出现：

```text
[Board1Detector] 无法打开视频流: http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw
[Board2Detector] 无法打开视频流: http://192.168.124.9:8080/stream?topic=/camera/rgb/image_raw
```

但后续节点重启后又能继续运行，所以这不是当前“双车协作未启用”的主因。

### 3. 退出时底盘/雷达线程报错

日志中还有 `timerBatteryCB`、`lslidar_driver_node` 关闭时报错。这些多发生在 Ctrl+C 关闭阶段，暂时不要把它们当作本次问题主因。

---

## 十一、禁止事项

1. 不要重写主控状态机。
2. 不要修改导航、识别、配送业务流程。
3. 不要把两车改成共用 ROS Master。
4. 不要恢复 ROS topic 双车通信方案。
5. 不要进行 namespace 多机器人改造。
6. 不要一次性大规模修改多个文件。
7. 必须先确认参数为什么没有生效，再做小范围修复。
8. 每一步修改后都要给出对应的验证命令。

---

## 十二、请按以下格式输出结果

```text
阶段 0 检查结果：
1. race_bringup.launch 参数定义与透传情况
2. main.launch 参数定义与 main_controller 私有参数写入情况
3. main_controller.py 参数读取方式
4. strategy.yaml 是否覆盖命令行参数
5. 问题根因判断
6. 建议修改文件

阶段 1 修改内容：
1. 修改了哪些文件
2. 修改了哪些行/参数
3. 为什么这样修改
4. 如何验证

阶段 2 验收：
1. 启动命令
2. rosparam 检查结果
3. 预期日志
4. 若仍失败，下一步排查方向
```
