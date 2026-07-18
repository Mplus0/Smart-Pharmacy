# pharmacy_mplus0 上机测试命令

适用环境：Ubuntu 18.04、ROS Melodic、Python 2.7。

本文假定新功能包已经放置为：

```text
~/robot_ws/src/pharmacy_mplus0/
```

所有命令均在小车终端执行。车 1使用 `car_id:=1`，车 2使用 `car_id:=2`。

当前版本使用包内 ONNX 模型识别板二，并启用裁判单车上报。单独启动裁判节点时需手动发布 `/referee_active=True`。

## 1. 部署前检查

### 1.1 确认目录名称

```bash
cd ~/robot_ws/src
ls -ld pharmacy_mplus0
```

### 1.2 确认工作空间中只有一个同名 ROS 包

```bash
grep -R -l '<name>pharmacy_mplus0</name>' ~/robot_ws/src/*/package.xml
```

输出应只有：

```text
~/robot_ws/src/pharmacy_mplus0/package.xml
```

### 1.3 设置节点脚本执行权限

```bash
chmod +x ~/robot_ws/src/pharmacy_mplus0/scripts/*.py
```

### 1.4 检查资源目录

```bash
ls -lh ~/robot_ws/src/pharmacy_mplus0/models/board2
ls -la ~/robot_ws/src/pharmacy_mplus0/resources/audio
```

板二模型应包含：

```text
status_best.onnx
number_best.onnx
```

音频至少需要包含本次测试会触发的 `WAIT-*.wav`、取样和送样语音。

### 1.5 检查关键配置

```bash
grep -n 'model_file\|smoothing_window\|status_roi\|number_roi' ~/robot_ws/src/pharmacy_mplus0/config/vision.yaml
grep -n 'server_ip\|server_port' ~/robot_ws/src/pharmacy_mplus0/config/communication.yaml
grep -n 'peer_ip\|peer_port\|local_port' ~/robot_ws/src/pharmacy_mplus0/config/communication.yaml
grep -n 'camera_url' -A 2 ~/robot_ws/src/pharmacy_mplus0/config/vision.yaml
```

确认目标机 OpenCV 支持 ONNX DNN：

```bash
python -c 'import cv2; print(cv2.__version__); print(hasattr(cv2.dnn, "readNetFromONNX"))'
```

最后一项必须输出 `True`；此检查只读取当前环境，不安装或升级依赖。

## 2. 编译与环境加载

```bash
cd ~/robot_ws
catkin_make
source ~/robot_ws/devel/setup.bash
rospack profile
rospack find pharmacy_mplus0
```

`rospack find pharmacy_mplus0` 应输出：

```text
/home/EPRobot/robot_ws/src/pharmacy_mplus0
```

如果小车用户名或工作空间位置不同，以实际路径为准。

每打开一个新终端，都先执行：

```bash
source ~/robot_ws/devel/setup.bash
```

## 3. 第一阶段：只测试基础系统

启动底盘、导航、摄像头和视频服务：

```bash
roslaunch pharmacy_mplus0 base_camera_nav.launch
```

另开终端检查基础话题：

```bash
source ~/robot_ws/devel/setup.bash
rostopic list
rostopic hz /odometry/filtered
rostopic hz /camera/rgb/image_raw
rosservice list | grep clear_costmaps
rostopic echo /move_base/status
```

基础系统正常后按 `Ctrl+C` 停止，再进行下一阶段。

## 4. 第二阶段：单航点测试

航点测试时不要启动完整主控。

### 4.1 先启动基础导航

如果不需要摄像头：

```bash
roslaunch pharmacy_mplus0 base_camera_nav.launch \
  start_camera:=false \
  start_video_server:=false
```

### 4.2 首先 dry-run

另开终端：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 test_navigation.launch \
  car_id:=1 \
  target:=A \
  dry_run:=true
```

车 2检查起点：

```bash
roslaunch pharmacy_mplus0 test_navigation.launch \
  car_id:=2 \
  target:=home \
  dry_run:=true
```

### 4.3 确认坐标后实际导航

```bash
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=A
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=B
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=C
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=board1
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=board2
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=lab1
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=lab2
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=lab3
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=lab4
roslaunch pharmacy_mplus0 test_navigation.launch car_id:=1 target:=home
```

车 2测试时将 `car_id:=1` 改为 `car_id:=2`。

## 5. 第三阶段：单独测试视觉节点

### 5.1 启动摄像头，不启动导航

终端 1：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 base_camera_nav.launch \
  start_navigation:=false
```

### 5.2 启动视觉节点

终端 2：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 vision.launch car_id:=1
```

### 5.3 测试板一

终端 3发布状态 10：

```bash
source ~/robot_ws/devel/setup.bash
rostopic pub /nav_state std_msgs/Int32 'data: 10' -1
```

观察输出：

```bash
rostopic echo /cam_return
rostopic echo /board1_all_text
```

视觉节点每次进入状态 10只正式发布一次。需要重新识别时，先切换到其他状态，再重新发布 10：

```bash
rostopic pub /nav_state std_msgs/Int32 'data: 9' -1
rostopic pub /nav_state std_msgs/Int32 'data: 10' -1
```

### 5.4 测试板二

发布状态 13：

```bash
rostopic pub /nav_state std_msgs/Int32 'data: 13' -1
```

观察输出：

```bash
rostopic echo /board2_return
```

板面为空闲时预期结果为：

```text
data: [0, 0]
```

板面忙碌时预期结果为 `[1, 5]`～`[1, 10]`。模型会在黑框与内白区对齐成功后累计 5 个有效帧再发布一次。

## 6. 第四阶段：单独测试双车 TCP

先确认两车 `communication.yaml` 中的 IP 和端口互相对应，并确认两车网络可达。

### 6.1 车 1只启动双车 TCP

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 main.launch \
  car_id:=1 \
  start_dual_tcp:=true \
  start_referee:=false \
  start_detect:=false \
  start_nav_pharmacy:=false
```

### 6.2 车 2只启动双车 TCP

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 main.launch \
  car_id:=2 \
  start_dual_tcp:=true \
  start_referee:=false \
  start_detect:=false \
  start_nav_pharmacy:=false
```

### 6.3 从车 1发布测试完成消息

车 1另开终端：

```bash
source ~/robot_ws/devel/setup.bash
rostopic pub /dual_car/round_done std_msgs/Int32MultiArray 'data: [1, 1]' -1
```

车 2观察：

```bash
rostopic echo /dual_car/peer_done
```

预期收到：

```text
data: [1, 1]
```

反向测试时，从车 2发布 `[2, 1]`，在车 1观察 `/dual_car/peer_done`。

## 7. 第五阶段：单独测试裁判通信

先启动基础系统，使 `/odometry/filtered` 和 TF 可用，然后只启动裁判节点。

车 1：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 main.launch \
  car_id:=1 \
  start_dual_tcp:=false \
  start_referee:=true \
  start_detect:=false \
  start_nav_pharmacy:=false
```

车 2将 `car_id:=1` 改为 `car_id:=2`。

手动发布裁判字段进行检查：

```bash
rostopic pub /referee_task std_msgs/String 'data: "A"' -1
rostopic pub /referee_cv1 std_msgs/String 'data: "WAIT-5"' -1
rostopic pub /referee_cv2 std_msgs/String 'data: "AB-1"' -1
```

裁判服务器应收到以换行分隔的 JSON。重点确认：

- `id` 与车号一致；
- `speed` 来自 `/odometry/filtered.twist`；
- `odom` 来自 TF `map → base_footprint`，失败时回退 `base_link`；
- `task`、`CV1`、`CV2` 与手动发布值一致。

## 8. 第六阶段：完整单车流程

完整入口默认同时启动基础系统、双车 TCP、裁判、视觉和主控。

车 1：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=1
```

车 2：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_mplus0 race_bringup.launch car_id:=2
```

注意：双车模式在主控中固定开启。车 1完成一轮后会等待车 2，车 2初始会等待车 1令牌，因此不能把单车连续循环视为当前支持的运行方式。

如果底盘、导航和摄像头已经由其他终端启动，只启动业务节点：

```bash
roslaunch pharmacy_mplus0 main.launch car_id:=1
```

如果暂时不连接裁判服务器：

```bash
roslaunch pharmacy_mplus0 main.launch \
  car_id:=1 \
  start_referee:=false
```

## 9. 第七阶段：完整双车测试

1. 两辆车分别启动完整入口。
2. 确认车 1进入状态 9，车 2保持状态 8。
3. 车 1完成化验窗口并进入状态 15时，确认车 2获得令牌。
4. 车 2确认收到车 1的 `/dual_car/peer_board1_all_text`。
5. 车 2共享成功时应跳过板一停车点；共享不可用时应回退本车板一识别。
6. 两车连续运行多轮，确认没有重复令牌或旧板一结果复用。

两车都建议开启以下观察终端：

```bash
source ~/robot_ws/devel/setup.bash
rostopic echo /nav_state
```

其他观察命令：

```bash
rostopic echo /cam_return
rostopic echo /board2_return
rostopic echo /board1_all_text
rostopic echo /dual_car/round_done
rostopic echo /dual_car/peer_done
rostopic echo /dual_car/peer_board1_all_text
rostopic echo /referee_task
rostopic echo /referee_cv1
rostopic echo /referee_cv2
```

## 10. 停止与故障排查

前台 roslaunch 使用：

```text
Ctrl+C
```

检查节点：

```bash
rosnode list
rosnode info /nav_pharmacy
```

检查话题连接：

```bash
rostopic info /nav_state
rostopic info /cam_return
rostopic info /board2_return
```

检查 TF：

```bash
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo map base_link
```

检查双车端口：

```bash
ss -lntp | grep 9001
ss -lntp | grep 9002
```

检查配置是否被当前包读取：

```bash
rospack find pharmacy_mplus0
```

如果修改过 YAML、模板或语音文件，需要停止并重新启动对应节点。
