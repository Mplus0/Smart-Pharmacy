# pharmacy_ISCC

基于当前已迁移的 pharmacy_ISCC 整理，以 pharmacy_pkg 为业务基线。ROS 包名保持
`pharmacy_ISCC`，Python 命名空间保持 `pharmacy_iscc`。不修改原包、不恢复历史实现。

## 最终结构与职责

```text
pharmacy_ISCC/
├── .gitattributes
├── README.md
├── AGENTS.md
├── CMakeLists.txt
├── package.xml
├── setup.py
├── scripts/
│   ├── __init__.py
│   ├── F1_detect_code_v5.py
│   ├── board1_detector.py
│   ├── board2_detector.py
│   ├── board2_tcp.py
│   ├── F1_yaofang_v5.py
│   ├── audio.py
│   ├── tcp_link_v2.py
│   ├── config.py
│   ├── recognition_records.py
│   └── navigation_utils.py
├── tools/
│   ├── prepare_race.sh
│   ├── start_race.sh
│   ├── stop_race.sh
│   ├── web_console/
│   │   ├── server.py         # 独立 HTTP 控制后台
│   │   ├── ros_observer.py   # 只读 ROS 订阅进程
│   │   ├── index.html        # 单车控制台
│   │   └── README.md
│   ├── waypoint_console.py
│   ├── record_waypoint.py
│   └── test_move_to_waypoint.py
├── launch/
│   ├── base_system.launch
│   ├── pharmacy_main.launch
│   ├── single_car.launch
│   ├── dual_car.launch
│   ├── detect.launch
│   ├── test_move_to_waypoint.launch
│   └── ar_marker_detection.launch
├── config/
│   ├── competition.yaml
│   ├── vehicles.yaml
│   └── vision.yaml
└── resources/
    ├── audio/                 # 48 个 WAV
    └── board2_templates/      # 7 个 PNG
```

| 文件 | 职责 |
| --- | --- |
| F1_detect_code_v5.py | 唯一视觉 ROS 节点；图像输入、状态缓存、稳定判定、识别调度及消息发布 |
| board1_detector.py | 板一几何定位、窗口与二维码解析、样本数量优先选择及整板队列；识别流程调用节点传入的稳定发布函数并使用同一历史队列 |
| board2_detector.py | 板二模板加载、多尺度匹配；接收已有图像和模板，不打开相机 |
| board2_tcp.py | GET 查询客户端，接收节点提供的状态检查函数；不单独启动 ROS 节点 |
| F1_yaofang_v5.py | 实际就绪检查、服务放行、导航状态机、单车缓存、双车轮转及任务/记录调用 |
| audio.py | 取样/送达先启动播放器，清图和原 0.5 秒停留与播报并行；等待两者结束后才导航。板一异常提示仍有时限 |
| tcp_link_v2.py | 双车 TCP 重发、序号去重、IP/token 检查及本地话题转发 |
| config.py | 三个 YAML 的统一读取、元组恢复、车辆选择、资源及记录路径；复用 RosPack 缓存 |
| recognition_records.py | MD5、SQLite 活动任务和 Excel 导出 |
| navigation_utils.py | 地图、姿态、目标合法性、新鲜定位检查，供正式节点和工具复用 |
| tools/ | 赛前准备、手动放行、交互取点、停车记录、单点导航；比赛代码不导入 tools |

## 配置与原行为

三个 YAML 是业务配置的唯一来源，使用 PyYAML safe_load；competition/vision 使用
带注释的 YAML 流式映射语法，保持原参数键、数值和分组，便于与原字典逐项对照。

- `competition.yaml`：topics、states、nav、dual_tcp、lab_info、window_log_name。
- `vehicles.yaml`：原 CARS，两辆车的 ID、地址、端口、起点及先发/共享结果差异。
- `vision.yaml`：board1、board2、detect，包括图像输入、模板阈值、板二 TCP。

`config.py` 继续提供 COMMON、CARS、get_car_id、get_car_config、package_path、
records_path。ROI、各航点和 home_pose 恢复为 tuple；车辆和 lab_info 的键保持整数。
原 `2 ** 24` 以等值整数 16777216 保存。资源路径由包位置生成，不重复维护 YAML 路径。
不加载历史 compitation_para.yaml，不改导航、识别和通信的原有默认值。

比赛状态仍为 8 等令牌、9 去板一、10 板一识别、11 取样、12 去板二、13 板二识别、
14 送样、15 回家。导航仍在 move_base 及服务就绪后等待 `/nav_pharmacy/start`。
板一保留四个真实窗口、三帧稳定、样本数量优先及同分原窗口顺序；每项任务仍为
`[C标记,A标记,B标记,样本数,目标窗口零基索引,错误窗口编号]`，整板最多四项。
板一到点后3秒没有有效任务时，沿当前车身方向后退约30 cm，再给3秒识别时间；
仍无结果则导航回现有板一停车点，再次进入状态10清空视觉历史并重试。每次访问
最多后退一次、回点一次，最后一次识别仍受原 `board1_wait_timeout_sec` 总等待
超时保护（导航本身沿用原导航超时）；不无限后退。参数在 `nav.board1_recovery`。
后退速度上限 `0.15 m/s`，用新鲜 `odom -> base_footprint` 距离反馈控制，接近
30 cm时减速并发送零速度停车；直退不带转向。后退动作超时上限改为5秒，
仍按里程计距离停车，不固定倒车5秒。`nav.board1_recovery.force_reverse`
默认为 `true`，仅这一次板一保底后退跳过后方激光的可见性、障碍及数据检查；
其他导航与避障设置不变。关闭该项时恢复原后方激光检查。里程计新鲜度、方向、
偏航/横移、超时与停车限制始终保留；尚未起步时条件不足仍给第二次3秒识别机会，
起步后违反限制则中止任务。视觉在后退时继续识别，运动期间
收到任务也等本次停车后才执行；后退不会改变任务选择规则、三帧稳定或错误播报。
此功能只在真的重新识别时启用，已有缓存任务经过板一的分支不触发。清图放到
本次识别成功之后，避免阻塞首个3秒计时。本次仅静态核对，未在实车执行后退；
强制后退分支不依赖后方激光，后退距离和制动误差尚未实车验证。
单车缓存剩余任务，每项送样后先回原点，再前往板一；保留每完成两项重新识别及连续配送逻辑。
双车只执行首项，car2 可从对车 all_text 中排除已选项再选择，否则回退本车识别。

板一引导点已删除，连续任务经过原点后直接前往现有板一停车点 `(0.55, 0.00)`。
状态15通过 `nav.home_arrival.pass_through: true` 将原点作为经过点：临时将位置
容差放宽到至少 `0.25 m`、朝向容差放宽到至少 `30°`，并设置 `free_goal_vel: true`。
与板二引导点共用交接逻辑，以20 Hz检查定位，进入原点 `0.35 m` 范围且朝向误差
不超过 `45°` 即可交接，不等精确到点，不主动取消目标或发送零速度；不额外限速。
恢复进入本段前的实际位置/朝向容差和 `free_goal_vel` 后，在同一次状态处理中
立即进入板一导航，省去经过原点时的清图、等待和停车确认。若导航先报告成功，
也直接继续。避障、重规划或目标完成仍可能造成减速，未承诺全程不停。
双车需要等待令牌或达到结束轮数时仍沿用原停车确认：新鲜里程计确认线速度
不超过 `0.02 m/s`、角速度不超过 `0.05 rad/s` 并保持 `0.3 s`，最多等待 `5 s`。
关闭 `pass_through` 可恢复原点停车逻辑。所有返回分支均尝试恢复临时参数，失败
不继续出发。原点/板一坐标、单车缓存、双车令牌与两项后重新识别逻辑不变。
本次只做静态核对，未做实车经过或停车验证；需重启主业务节点加载新配置。

原点与板二共用按命名空间缓存的动态参数客户端，等待描述回调完成参数类型表后
才修改设置，避免Melodic客户端偶发报 `don't know parameter: free_goal_vel`。
客户端在任务节点退出时统一关闭，不在每个经过点重复创建/关闭；出现修改失败时，
先尝试恢复原设置，再执行任务退出。退出时仅取消仍活动的本节点导航目标。

前往板二的状态12增加上方进场：先到 `(0.85, 3.70)`、朝西 `180°` 的引导点，
再到原板二停车点 `(-1.70, 3.70)`；到达最终停车点后才进入状态13识别。
配置位于 `competition.yaml` 的 `nav.board2_approach`；引导点第三项使用显式
`yaw_deg`，原 `waypoints` 的索引和各窗口坐标不变。过渡点前后均不额外修改速度
上限。过渡段临时将位置/朝向容差放宽到至少 `0.20 m / 30°`，并设置
`free_goal_vel: true`，不要求规划以零速度结束此段。以20 Hz检查新鲜定位，进入
过渡点 `0.35 m` 范围且朝向误差不超过 `45°` 时即可交接，不等精确到点，不主动
取消目标或发送停车指令；下一导航目标直接接替活动目标。若先收到过渡点成功
结果，则按宽容差完成该段，减少停留微调。
发往最终板二点前恢复进入本段前的实际位置/朝向容差和 `free_goal_vel`，保证最终
识别停车点仍使用原要求；失败时同样尝试恢复。恢复失败则停止任务。普通目标、
各识别点的停车和识别触发逻辑不变。实际仍可能因避障、重规划或目标已完成而减速停车，
不能保证恒速无停顿。此流程按ROS新目标接替活动目标的机制实现，未改底层规划器。
设置 `enabled: false` 可恢复原先直达板二的逻辑。保留现有规划虚拟墙与定位地图。
本次仅按地图静态核对引导点及上方通道，未运行 ROS、导航或实车验证；需重启主业务
节点加载新配置。首次实车应核对各取样窗口进场的转弯空间、过渡点交接和最终停车。

到达最终板二识别停车点后开始5秒识别计时（`nav.board2_wait_timeout_sec`），
过渡点不计时。截止前收到有效结果则按原忙闲及等待规则处理；到时仍无结果，
主状态机提交 `[0, 0]` 默认空闲结果，播放现有 `WAIT-0.wav` 后继续送样，不再因
板二超时退出。晚到结果不覆盖已采用的结果。超时默认单独标为“超时默认”写入记录，
不标作真实识别。清图服务移至结果播报之后，避免阻塞5秒识别计时和空闲播报。
参数为0时仍可无限等待；板一超时、视觉算法、过渡路线及其他任务流程不变。
本次仅静态核对，需重启主业务节点后生效，未做实车验证。

板二默认 TCP，发送 `GET\n`（换行结尾），读取一行 JSON；ok 必须为 true，
wait_time 必须为 0 或 5～10 的整数，超时/无有效结果仍使用默认 free。
vision 模式使用原模板算法，复用视觉节点的相机帧。忙碌实际等待
`max(0, wait_time - 2)` 秒，播报计时且必须等播报结束。
完成送样停留和播报、进入状态 15 时释放双车令牌。双车 JSON 字段、重发、去重不变。

记录继续使用原 JSON 序列化和 UTF-8 MD5 输入、SQLite 表及活动标记、记录调用时机、
Excel 列/样式与覆盖策略。目录默认 `${ROS_HOME:-~/.ros}/pharmacy_ISCC/result_md5`，
可通过 PHARMACY_RECORD_DIR 设置绝对路径，视觉和任务进程必须使用同一设置。
文件仍为 car1.sqlite3/car2.sqlite3 和 recognition.xlsx。两车使用各自机器和 ROS
Master；如同机运行需各自记录目录，避免固定 Excel 名覆盖。不迁移旧数据库或报表。

## 安装与依赖

setup.py 将平铺 scripts/ 安装为 pharmacy_iscc；全部内部导入使用该命名空间。
CMake 安装三个正式节点、三个 Python 工具和三个 Bash 脚本，并将 scripts/ 安装为 pharmacy_iscc 包。
ROS Melodic 的 catkin_python_setup 要求包名与源目录名一致；为保留平铺的 scripts/，
CMake 在 devel 生成包路径入口，在 install 安装同一命名空间下的 Python 模块。
launch/config/resources 安装至包 share，rospkg 定位资源，不依赖当前工作目录。
不支持未构建时直接从 scripts 目录执行脚本；车辆端先完成 catkin 构建并 source 工作空间。

依赖包括 ROS1 rospy、actionlib、tf、cv_bridge、消息包、rosgraph、rostopic、rospkg、
PyYAML、NumPy、OpenCV、openpyxl、pyzbar、libzbar 和 SoX 的 play。
pyzbar 需另行安装到 ROS 使用的 Python 解释器。package.xml 按 ROS_PYTHON_VERSION
区分 Python 2/3 依赖；移除了无使用点的 enum 导入及 enum34 依赖。
基础系统依赖车辆上的 robot_navigation、astra_camera、web_video_server；AR 入口还需
ar_track_alvar。地图和底盘参数仍由外部 robot_navigation 管理。

catkin_install_python 的 PROGRAMS 负责安装可执行入口。此车辆的 ROS Melodic
在 devel 使用源目录脚本，三个正式节点和三个工具必须有 Linux 执行权限。
文本由 .gitattributes 固定 LF，避免 shebang 的 CRLF 问题。

## 车辆端运行步骤

单车比赛使用两个终端，工作空间默认 `~/robot_ws`。停止原三条命令启动的旧进程后，
使用以下入口，不要与旧入口重复启动同名节点。

```bash
# 终端一：并行启动底盘、雷达、定位、规划、原始RGB、识别与任务，保持终端打开。
bash ~/robot_ws/src/pharmacy_ISCC/tools/prepare_race.sh

# 终端二：开始按钮；运行即请求开始，可提前运行，就绪后自动发车一次。
bash ~/robot_ws/src/pharmacy_ISCC/tools/start_race.sh

# 停止单车业务及基础环境。
bash ~/robot_ws/src/pharmacy_ISCC/tools/stop_race.sh
```

两个脚本自动 source 工作空间，可用 `ISCC_WORKSPACE` 指定其他工作空间。
终端一开始显示“正在启动比赛系统，请等待‘系统已就绪’提示”；主业务完成相机、
识别与导航初始化、开始服务实际可连接后，只显示一次“【系统已就绪】”，并打印
终端二的开始命令。此提示不调用开始服务；即使显示进程晚启动、漏收初始化日志，
也能通过服务就绪检测补上提示。终端二运行 `start_race.sh`，不要再次运行准备脚本。
准备脚本只运行一个合并 `roslaunch`，保留原 `use_race_init=true` 的导航入口，
不调用开始服务；开始脚本等待ROS Master、初始化ROS节点及时间，再等待
`/nav_pharmacy/start` 可用后调用一次，无二次确认，避免冷启动重试时ROS时间未初始化。
快速入口移除视觉3秒、导航6秒固定延迟，节点与底层同时初始化。任务主节点保留
move_base/清图服务等待，并检查视觉初始化及YOLO预热完成、新鲜RGB、地图和新鲜
map→base_footprint定位后才提供开始服务；就绪检查最多60秒，参数在 `nav.startup`。
这不保证设备立即完成初始化，也不改变初始位姿假设；仍需按赛道原点摆放小车。

比赛终端只显示任务状态、采用的识别结果、板一保底与任务/视觉错误。底盘、雷达、
TEB、相机与逐帧识别输出不显示在终端，详细日志仍由ROS保存；roslaunch启动输出
在 `~/.ros/pharmacy_ISCC/startup_logs/`（设置ROS_HOME时跟随该目录）。不删除历史日志。
`--skip-log-check` 只跳过启动前历史日志目录大小扫描；`--log` 强制节点输出到日志。
显示过滤器先将ROS消息的UTF-8字节串解码为Unicode，再匹配中文状态和错误，
避免Python 2的ASCII隐式解码异常遮住真正任务错误。

关闭准备终端会请求终止该合并launch并清理状态显示进程；停止脚本兼容新合并入口
及旧基础/业务分开入口，由roslaunch清理其子节点。使用 `bash` 调用无需设置执行权限。

本次快速入口省去或推迟的内容：

- 视觉3秒、导航6秒及接口初始化0.5秒固定等待，改用实际依赖就绪检查。
- 基础与业务两次roslaunch分开启动，改为一次合并启动，不重复打开RViz或视频服务。
- 额外RGB处理、深度图处理、深度配准处理和硬件配准；保留原始RGB驱动、话题和TF。
- 主任务单车启动时不再导入双车共享选项才需要的板一视觉模块，双车用到时再导入。
- 重复创建RosPack并扫描包路径，改为每个进程复用缓存；本机只读微测量中6次查找
  从约0.97秒降至0.16秒，此值不代表整车启动耗时。
- 终端中的设备/规划/逐帧调试刷屏，改存详细日志；未删除这些节点的功能。

静态解析确认快速入口由31个节点减为22个，少启动的9个相机处理节点为：
`depth_metric`、`depth_metric_rect`、`depth_points`、`depth_rectify_depth`、
`depth_registered_hw_metric_rect`、`depth_registered_metric`、
`depth_registered_rectify_depth`、`points_xyzrgb_hw_registered`、`rgb_rectify_color`
（以上均在 `/camera/` 命名空间）。未删除节点源码、地图、模型或历史日志文件。
上述计数仅为launch内节点；准备脚本另用一个只读ROS日志显示节点过滤任务信息。

保留底盘、激光滤波、AMCL、EKF、TEB、定位地图与规划虚拟墙、YOLO模型预热、
语音、MD5/SQLite/Excel记录、识别算法和全部既有任务/保底/原点经过逻辑。
底层导航参数及硬件驱动文件未修改。未实车启动或测量优化后的总耗时，不能承诺
从40秒降到某个固定秒数。本次仅做静态解析与配置对照，不启动或放行小车。

合并launch的底层等效命令如下（直接执行会输出roslaunch信息，比赛推荐上面的脚本）：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch --skip-log-check --log pharmacy_ISCC single_car.launch \
  start_base:=true fast_start:=true output:=log max_rounds:=1
# 另一终端：
bash ~/robot_ws/src/pharmacy_ISCC/tools/start_race.sh
```

single_car固定car1、关闭双车TCP和双车轮转，视觉不respawn；保留现有单车连续配送，
`max_rounds` 当前只约束双车结束轮数，传 `max_rounds:=1` 不会让单车一轮后停止。
本次优化不改变该既有行为。单独调用single_car默认仍复用已启动的底层，
只有 `start_base:=true` 才包含底层；只有 `fast_start:=true` 才使用零延迟与就绪门控。

双车分别在各自机器的 ROS Master 上操作；基础层沿用赛道初始化、不打开 RViz：

```bash
roslaunch pharmacy_ISCC base_system.launch use_race_init:=true open_rviz:=false
roslaunch pharmacy_ISCC dual_car.launch car_id:=1
rosservice call /nav_pharmacy/start '{}'
```

car2 业务传 `car_id:=2`，两车均需手动放行，car1 先发、car2 等令牌。
默认每车一轮，连续运行传 `max_rounds:=0`。
检查 vehicles.yaml、competition.yaml 和 vision.yaml 中的现场地址。

base_system保留start_navigation、start_camera、start_video_server、use_race_init、
open_rviz和output，增加camera_processing（默认true）；快速单车入口设为false。
默认use_race_init=true、open_rviz=false，规划器仍为TEB。single_car新增start_base、
fast_start和output，独立双车入口不改变。

pharmacy_main 仅启动业务，保留 car_id、output、max_rounds、enable_dual_car、
start_dual_tcp、start_detect、start_nav_pharmacy、respawn_comm、respawn_detect、
respawn_nav_pharmacy、detect_start_delay、nav_start_delay、dual_publish_repeat、
dual_publish_interval，增加wait_for_startup_ready。普通入口默认仍视觉延时3秒、
导航延时6秒；快速单车入口覆盖为0并启用实际就绪检查。重启策略保持原值。

独立视觉调试使用运行中的相机，保留话题与记录目录隔离，不启动任务导航：

```bash
roslaunch pharmacy_ISCC detect.launch car_id:=1 state:=10
rostopic echo /pharmacy_vision_debug/cam_return
```

state=10 为板一，state=13 为板二。板二默认 TCP；模板调试先修改 vision.yaml 的
board2.result_source 为 vision。状态/结果在 /pharmacy_vision_debug 下，每次访问只
发布一次；重启调试入口可重新识别。调试入口不创建活动任务数据库。

航点记录与控制台：

```bash
rosrun pharmacy_ISCC waypoint_console.py --dry-run
rosrun pharmacy_ISCC waypoint_console.py --output ~/waypoints/points.json
rosrun pharmacy_ISCC record_waypoint.py A --map-file /absolute/path/map.yaml --output ~/waypoints/pharmacy_points.json --car-id 1
rosrun pharmacy_ISCC record_waypoint.py --list --output ~/waypoints/pharmacy_points.json
```

控制台输入地图 x、y、角度（度）后导航，停稳验收后命名保存；命令有 pose、list、
save NAME、replace NAME、quit。record_waypoint 只记录停车定位，不发导航目标。
两者保持原格式和检查，不自动写入业务配置；审核后更新 competition.yaml 的
nav.waypoints/euler_angles 或 vehicles.yaml 的 home_pose。坐标第三项仍是角度索引。

单点测试先预览，再在车辆环境显式发送目标：

```bash
roslaunch pharmacy_ISCC test_move_to_waypoint.launch car_id:=1 target:=A dry_run:=true
roslaunch pharmacy_ISCC test_move_to_waypoint.launch car_id:=1 target:=A
```

AR 功能独立保留，比赛节点无 AR 订阅，不自动纳入基础/比赛启动：

```bash
roslaunch pharmacy_ISCC ar_marker_detection.launch
```

原 marker_size、误差阈值、点云/相机话题和 world 到相机 TF 均保持不变。

## 本次迁移清单

| 原文件/内容 | 最终归属 |
| --- | --- |
| 单车赛前启动与手动放行命令 | tools/prepare_race.sh、tools/start_race.sh、tools/stop_race.sh；环境启动、比赛放行与全部停止 |
| F1_detect_code_v5.py 中板一算法 + board1_selection.py | board1_detector.py；节点保留输入、状态与发布 |
| 视觉中的板二模板算法 | board2_detector.py，模板由节点持有并传入 |
| 视觉中的 query_board2_tcp | board2_tcp.py，实时状态检查通过参数传入 |
| 状态机 play_audio、视觉异常音频 | audio.py；取样/送达在导航成功后优先启动播报，与原到达动作并行；下一段导航仍等待播报结束 |
| dual_car_config.py + package_paths.py | config.py + competition/vehicles/vision.yaml |
| navigation_guard.py | navigation_utils.py |
| base_camera_nav.launch + single_car_base.launch | base_system.launch，以参数保留差异 |
| single_car_task.launch + full_run_debug.launch | single_car.launch |
| dual_car_task.launch | dual_car.launch |
| QRcode.launch | ar_marker_detection.launch |
| 其余三个 Launch、三个工具、记录和通信代码 | 保留，修正导入 |
| 48 个 WAV、7 个 PNG | 保留原资源 |

被替代的四个旧 Python 模块和七个旧 Launch 已删除。视觉脚本中无调用的
calculate_qr_lengths、normalize_qr_data、build_board1_msg 及重复 detect_error_window
已去除；实际运行的错误窗口检测统一保留任务选择模块的原实现。
不增加模块、嵌套 src、docs、tests 或历史配置，不提交 Git。

## 静态验收与边界

开发机无 ROS 运行环境，遵照本次会话“不运行代码”的限制，仅编辑和静态检查。
按 pharmacy_pkg 对照算法函数、选择函数、导航主体和语音函数，允许的差异仅为模块
位置、导入、参数传递和资源/配置读取；核对配置逐项迁移、元组恢复、Launch XML、
本包 include 参数、模块引用、安装清单和资源 SHA256。

本次静态核对结果：47 个视觉/算法/TCP 函数、6 个任务选择函数、导航状态机和任务
播报主体与基线一致（仅还原接口迁移差异后比较）；200 个配置标量的值和类型完成
对照。8 个 Launch 的 XML、参数及本包引用无缺失，20 条内部导入的模块和符号可在
文本中定位，55 个资源文件 SHA256 一致。git diff --check 无空白错误。

未执行 Python 语法编译、模块导入、单元测试、catkin 构建、ROS 启动或实车运行。
因此运行等价尚未实测，devel/install 导入、Linux 权限、依赖安装、硬件与网络连接及
完整比赛链路仍需车辆环境验证。原包 license 为 TODO，本次保留。

## 单车网页控制台（第一版）

独立工具位于 `tools/web_console`，不改变原业务逻辑和启动架构。
车辆端执行 `python3 ~/robot_ws/src/pharmacy_ISCC/tools/web_console/server.py`，
在同一局域网访问 `http://小车IP:8090`。提供环境启动、比赛放行、全部停止、
只读状态及识别结果、按需视频与日志。使用源码路径运行，不依赖 catkin 安装。
详见 [控制台说明](tools/web_console/README.md)。第一版仅完成静态检查，尚未板端验证。


## Board2 native YOLO (2026-10-29)

`config/vision.yaml` now selects `board2.result_source: yolo`. The existing `tcp` and `vision` (template) modes remain selectable. `board2_tcp.py`, the GET/newline JSON protocol, all tcp_* keys, vehicles.yaml and dual-car TCP code are unchanged. No automatic TCP fallback is used for an unrecognized YOLO image.

The Python2 ROS vision node keeps its existing /camera/rgb/image_raw input, visit cancellation and /board2_return Int32MultiArray output. It sends JPEG frames through private subprocess pipes to Python3.6 ONNX Runtime; the worker never opens a camera, network port or ROS publisher. Models are under resources/board2_yolo. The contract pins ROI, class order, RGB/white-pad/224/NCHW preprocessing, one-frame averaging and conservative confidence/margin rejection. Each visit resets history. free -> [0,0]; busy_5..busy_10 -> [1,5..10]. The existing navigation business already selects WAIT-0.wav or WAIT-5..10.wav; its motion and timing logic is unchanged.

Private runtime: `/home/EPRobot/.local/share/pharmacy-board2/runtime-20261029_2323/bin/python`. No system Python/ROS replacement.

The live poster uses different text spacing from the dataset. `text_row_gray_v1` normalizes the complete lower text row to the dataset's median center x=236, top y=160 and height=42, preserving aspect ratio, and converts the number input to grayscale. Geometry is determined without a digit label. The original number ROI, confidence 0.90 and margin 0.15 remain fixed. Missing or incomplete text geometry abstains. Status classification keeps the original color input.

The original-distance digit 9 required supervised domain adaptation. The active `number_best.onnx` SHA256 is `9bedc149ec8b1ec90fca9ca761f8f8a3ed5b9b4e5cf602eb3fc4c5ae89a21651`: only the final linear layer's weight/bias changed; the backbone and original local PT files remain intact. Training used the original training split plus confirmed field examples and shared label-preserving augmentations. Four original test images already seen in base training and one within-test duplicate were excluded, leaving 55 unique test images; all passed along with 60 validation and 29 separate field validation images. Cache fingerprints pin data, base model, worker/locator and training code. PT/ONNX maximum error was 1.19e-7, IR7/opset12. Dataset manifest, exclusions and metrics are in the external rollback anchor's `domain_adaptation` directory. Fresh live acceptance is recorded separately; dataset accuracy alone does not prove field success.

Stationary recognition and speech only (camera must already provide the ROS image; do not start navigation):

```bash
source /home/EPRobot/robot_ws/devel/setup.bash
roslaunch pharmacy_ISCC detect.launch state:=13 board2_result_source:=yolo speak_board2:=true camera_fallback_to_http:=false
rosservice call /pharmacy_vision_debug/status
# After changing the actual board and when speech has finished:
rosservice call /pharmacy_vision_debug/rearm
```

This launch isolates nav_state and board2_return under /pharmacy_vision_debug and never starts F1_yaofang_v5, a chassis driver or an action client. One recognized result speaks once per explicitly armed visit. Unknown/no board produces no result and no speech. The regular business launch continues to use its original audio logic.

To select the preserved TCP mode in the stationary tool, pass `board2_result_source:=tcp`; the configured original peer must be online. Simulated loopback protocol regression is not a real-peer acceptance.

File rollback anchor and scripts: `/home/EPRobot/robot_ws_diagnostics/20261029_2323_board2_deploy`. Run `python3 .../rollback_robot.py` to stop only this deployment's stationary processes, restore the enumerated original files from the verified archive, and quarantine additions/private runtime. The original startup state was all task processes stopped. Rollback does not execute Git commands. The initial deployment used file backups only; a later explicitly authorized Git publish preserves the pre-YOLO source checkpoint and the deployment as separate commits. The file archive also preserves permissions and files outside the tracked Git snapshot.

Private Python3.6 dependency versions: numpy 1.19.5, opencv-python-headless 4.5.5, onnxruntime 1.10.0, dataclasses 0.8, protobuf 3.19.6 and flatbuffers 1.12. Offline wheels, pinned package output and the original environment snapshot remain in the external rollback anchor and its local mirror.

Live acceptance was paused by the operator before Git publication. The final number model passed fresh original-position 5, 9, 10 and idle image/result/single-play-call tests; original-position 9, 10 and idle sound was confirmed on site. Final-model 6, 7 and 8 live regression and final-model 5 hearing confirmation remain pending (the earlier model's 5-8 sound was confirmed). The old idle result was observed clearing to null in 22ms before the new [1,5] result, with no speech during clearing. Physical no-board testing was skipped following operator steering. Real TCP peer testing remains deferred; source/config and simulated protocol regression are preserved. No navigation or actuator task was run.


## Board2 NCNN backend (2026-11-02)

Only the Board2 private Python3 worker changes. `board2.result_source` remains
`yolo`; `yolo_backend` selects `ncnn` or `onnx`, and `yolo_threads` accepts 1 or 2.
The installed candidate uses NCNN FP32 CPU, 1 inference thread and 1 OpenCV thread
inside the Board2 worker. Winograd/SGEMM transformed convolution buffers are
disabled to reduce memory. Board1, TCP, thresholds, smoothing, timeouts and audio
maps remain unchanged. Original ONNX models and the adapted number head remain.

Python3.6 runtime: `~/.local/share/pharmacy-board2/runtime-ncnn-20261101/bin/python`,
with ncnn 1.0.20250503 and the original NumPy/OpenCV/ORT packages reused through
a .pth file. Keep `runtime-20261029_2323` intact. Model resources add two NCNN
.param/.bin pairs and ncnn_manifest.json under resources/board2_yolo.

Switch only `config/vision.yaml` Board2 `yolo_backend` to `onnx` and restart the
visual node to use ORT; set threads=2 and restore the previous yolo_python path
to reproduce the archived original runtime exactly. Do not overwrite navigation
or competition.yaml. Full backup and hashes are stored outside the repository:
`~/robot_ws_diagnostics/20261101_board2_ncnn`.

The existing isolated stationary entry point is unchanged:
`roslaunch pharmacy_ISCC detect.launch state:=13 board2_result_source:=yolo speak_board2:=true camera_fallback_to_http:=false`.
Use the existing ROS master/camera and confirm the vehicle is stationary first.
This entry point remaps recognition topics and starts no navigation/base/arm node.

## Startup warmup and ROS-only images (2026-11-02)

Starting `single_car.launch` now creates the private Board2 worker when the
vision node starts, before any Board2 visit. Both classifiers run once on a
dummy tensor before `Board2 YOLO startup warmup complete` is printed. The
warmup publishes no result, plays no audio and does not fill visit history.
The same worker remains resident and waits for requests; the existing one-frame
history and one-result confirmation still apply to each real visit.

`base_system.launch` defaults `start_video_server` to false. Production and
standalone vision default to ROS images with HTTP fallback disabled; Board1 and
Board2 continue to use `/camera/rgb/image_raw`. The preparation script explicitly
disables the HTTP video server. ROS camera failure produces no recognition result.
No navigation/localization/chassis parameters, launch delays or manual start
service behavior change. Wait for the warmup-complete log before manually starting
the task. The private parameter `/detect_abc/board2_yolo_ready` reports startup readiness.

Rollback files and verification are outside the package:
`~/robot_ws_diagnostics/20261102_board2_startup_no_http`.
