# pharmacy_ISCC

基于 `pharmacy_pkg` 重构的 ROS1 智慧药房功能包。保持原有业务逻辑，规范代码结构、资源路径及启动入口。

## 目录结构

```text
pharmacy_ISCC/
├── .gitattributes
├── README.md
├── AGENTS.md
├── CMakeLists.txt
├── package.xml
├── setup.py
├── scripts/                         # 正式比赛代码
│   ├── __init__.py
│   ├── F1_detect_code_v5.py         # 视觉识别与结果发布
│   ├── F1_yaofang_v5.py             # 主状态机、导航与任务调度
│   ├── board1_selection.py          # 板一任务选择与队列生成
│   ├── dual_car_config.py           # 公共参数与车辆配置
│   ├── navigation_guard.py          # 地图、定位与目标校验
│   ├── package_paths.py             # 配置、资源及记录路径管理
│   ├── recognition_records.py       # MD5、SQLite 与 Excel 记录
│   ├── referee_client_v3.py         # 裁判通信与状态上报
│   └── tcp_link_v2.py               # 双车 TCP 通信与任务共享
├── tools/                           # 调试与维护工具
│   ├── record_waypoint.py           # 航点记录
│   ├── test_move_to_waypoint.py     # 单点导航测试
│   └── waypoint_console.py          # 交互式航点控制台
├── launch/                          # ROS 启动配置
│   ├── base_camera_nav.launch      # 基础导航与相机
│   ├── single_car_base.launch      # 单车基础系统
│   ├── single_car_task.launch      # 单车比赛业务
│   ├── dual_car_task.launch        # 双车比赛业务
│   ├── pharmacy_main_no_referee.launch # 通用业务入口
│   ├── full_run_debug.launch       # 单车完整任务调试
│   ├── detect.launch               # 独立视觉调试
│   ├── referee_only.launch         # 独立裁判通信
│   ├── test_move_to_waypoint.launch # 单点导航测试
│   └── QRcode.launch               # 可选 AR 标签检测
├── config/
│   └── vehicles.yaml               # 双车编号、网络参数与起点配置
└── resources/
    ├── audio/                      # 语音播报 WAV 文件
    └── board2_templates/           # 板二识别模板图片
```

## 运行方式

完成 catkin 构建并 source 工作空间后执行以下命令。

### 1. 单车比赛

先启动基础系统：

```bash
roslaunch pharmacy_ISCC single_car_base.launch
```

另开终端启动比赛业务：

```bash
roslaunch pharmacy_ISCC single_car_task.launch
```

手动放行任务：

```bash
rosservice call /nav_pharmacy/start '{}'
```

默认执行一轮整板任务。设置 `max_rounds:=0` 可连续执行：

```bash
roslaunch pharmacy_ISCC single_car_task.launch max_rounds:=0
```

如需同时启动裁判通信：

```bash
roslaunch pharmacy_ISCC single_car_task.launch start_referee:=true
```

也可以独立启动裁判通信：

```bash
roslaunch pharmacy_ISCC referee_only.launch car_id:=1
```

### 2. 双车比赛

两辆车分别在自己的 ROS Master 下启动基础系统：

```bash
roslaunch pharmacy_ISCC base_camera_nav.launch
```

启动业务。车 1：

```bash
roslaunch pharmacy_ISCC dual_car_task.launch car_id:=1
```

车 2：

```bash
roslaunch pharmacy_ISCC dual_car_task.launch car_id:=2
```

分别手动放行：

```bash
rosservice call /nav_pharmacy/start '{}'
```

两车通过 TCP 通信协作，车 1 先发起任务，车 2 等待令牌。默认每车执行一轮；连续执行时传入 `max_rounds:=0`。

### 3. 视觉调试

基础系统应提前启动，并确保相机图像源可用。

测试板一：

```bash
roslaunch pharmacy_ISCC detect.launch state:=10
rostopic echo /pharmacy_vision_debug/cam_return
```

检查板二：

```bash
roslaunch pharmacy_ISCC detect.launch state:=13
```

板二默认使用 TCP 查询。如需测试本地模板识别，将 `scripts/dual_car_config.py` 中板二的 `result_source` 设置为 `vision`。

### 4. 导航与航点工具

航点控制台：

```bash
rosrun pharmacy_ISCC waypoint_console.py --dry-run
```

记录航点：

```bash
rosrun pharmacy_ISCC record_waypoint.py --list --output ~/waypoints/pharmacy_points.json
```

测试前往航点 A：

```bash
roslaunch pharmacy_ISCC test_move_to_waypoint.launch car_id:=1 target:=A dry_run:=true
```

### 5. 其他入口

- `pharmacy_main_no_referee.launch`：通用业务入口，默认不启动基础系统和裁判通信。
- `full_run_debug.launch`：单车完整任务调试入口。
- `QRcode.launch`：独立 AR 标签检测，不属于默认比赛流程。

## 目录职责

- `scripts/`：正式比赛节点与业务实现。
- `tools/`：航点记录、导航测试及其他维护工具。
- `launch/`：基础系统、比赛业务及调试入口。
- `config/`：车辆参数配置。
- `resources/`：语音和识别模板资源。

基础地图及导航参数由外部 `robot_navigation` 管理。双车通过 TCP 协作，各自运行于独立的 ROS Master。
