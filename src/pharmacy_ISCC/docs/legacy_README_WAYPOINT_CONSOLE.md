# 输入坐标导航、到达后命名取点

当前车辆：192.168.124.20；Ubuntu 18.04 / ROS Melodic / Python 2.7。脚本独立于药房业务逻辑，不读取旧的 dual_car_config 点位，也不会自动改业务配置。

## 操作

MobaXterm 的一个终端运行基础导航，保持窗口运行：

```bash
source ~/.bashrc
roslaunch robot_navigation robot_navigation.launch
```

通过 RViz 的 2D Pose Estimate 初始化定位，确认激光与地图墙线稳定重合。自动导航时退出键盘遥控，避免遥控节点与 move_base 同时发布速度。

另一个小车 SSH 终端执行：

```bash
source ~/.bashrc
rosrun pharmacy_pkg waypoint_console.py
```

出现 `waypoint>` 后输入三个数字，以空格分隔：

```text
x y 朝向角度
```

坐标是当前 `/map` 坐标系下的米，不是相对小车的前进距离。朝向单位是度：0 沿地图 +X，90 沿 +Y，180 沿 -X，-90 沿 -Y；不是“相对当前车头转多少度”。RViz 的坐标轴或 `pose` 可帮助确认方向；不要直接照抄旧地图或示例坐标。

输入三个有效数字并回车就会发送一次导航目标。导航期间按 Ctrl+C 取消本脚本的目标，返回输入提示；输入 `quit` 退出。

本车到达判定现为位置误差不超过 **5 cm**、朝向误差不超过 **3°**。规划器的容差也已收紧，并使用慢速取点参数（前进上限 0.25 m/s、后退 0.12 m/s）。导航重新启动后生效。

脚本收到 move_base 成功后，还会用实际 TF 位姿及里程计复核：误差满足上述范围、线速度与角速度均不超过 0.01 m/s、0.01 rad/s，并连续保持 **1 秒**，才提示到达及命名：

```text
Point name (A/board1/lab1/home; Enter to skip):
```

此时输入 `A` 等点名保存，也可直接回车跳过。保存的是停稳后的实际定位，文件同时保留本次请求的目标坐标及误差验收结果。命名后保存前会再检查一次，避免等待输入期间位姿变化。若导航返回成功但实际误差超标，显示 `Arrival rejected`，不会提示成功取点或自动保存。

若到达后希望微调位置，可先跳过命名，遥控微调并退出遥控、停稳后输入 `save A`；本次有目标坐标时仍需满足误差范围。重录已有 A 点时，命名提示先按回车跳过，再输入 `replace A`，旧文件自动备份。以前按宽容差保存的点应重新验证；本次不自动覆盖旧点。

其他命令：

```text
pose            查看当前位置与朝向
list            查看已保存点
save A          将当前停稳位置保存为 A
replace A       明确覆盖 A，旧文件自动备份
quit            退出
```

默认保存到小车：

```text
/home/EPRobot/waypoints/car20_20261016/points.json
```

推荐点名：`home board1 A B C board2 lab1 lab2 lab3 lab4`。不要求一次取齐。地图或车辆标识不同时拒绝混入同一个文件；新一轮可换输出路径：

```bash
rosrun pharmacy_pkg waypoint_console.py --output ~/waypoints/car20_round2/points.json
```

只预览坐标、不发送导航目标：

```bash
rosrun pharmacy_pkg waypoint_console.py --dry-run
```

默认单次导航超时 90 秒，可用 `--timeout 120` 修改；成功后的独立误差/停稳复核最多等待 8 秒。导航失败、超时、取消或实际误差超限不会作为成功到达点保存。脚本检查地图边界、目标中心是否为可通行栅格、新鲜定位及已有导航任务；具体路径、车身占地和动态避障由当前 move_base 配置处理。

这是车式底盘，靠近目标后需要通过行驶轨迹对齐，不能依靠原地转动摆正。取点时应留出进站空间。收紧容差使超差被明确拒绝，并不代表当前底盘、定位和场地已实测达到此精度。

## 文件格式及交付边界

这是 schema=2 点位文件，绑定实际加载的 `/map` 内容及元数据，不依赖手工选择 map.yaml。与之前 record_waypoint.py 的 schema=1 文件分开保存。`points.NAME.pose` 为实际停车位姿，`requested` 为请求目标，`arrival_check` 为本次验收误差和稳定时长；手动无目标采集时该字段为 null。方向同时保存 yaw_rad、yaw_deg。文件更新采用原子替换，重录自动生成 `.bak.时间戳`。

此次只部署工具，没有选择实际运动目标，也没有自动写入业务点位。采齐后再核对点位并同步业务配置。
