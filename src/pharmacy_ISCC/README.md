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
| F1_yaofang_v5.py | 手动放行、导航状态机、单车缓存、双车轮转及任务/记录调用 |
| audio.py | 取样/送达先启动播放器，清图和原 0.5 秒停留与播报并行；等待两者结束后才导航。板一异常提示仍有时限 |
| tcp_link_v2.py | 双车 TCP 重发、序号去重、IP/token 检查及本地话题转发 |
| config.py | 三个 YAML 的统一读取、元组恢复、车辆选择、资源及记录路径 |
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
单车缓存剩余任务，每项送样后经过板一继续下一项，整板完成才回家计轮。
双车只执行首项，car2 可从对车 all_text 中排除已选项再选择，否则回退本车识别。

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

## 车辆端运行步骤（本次未执行）

始终先启动基础系统，确认定位，再启动业务，最后手动放行。

单车使用三个 Bash 脚本，工作空间默认 `~/robot_ws`：

```bash
# 终端一：启动基础环境及单车业务，保持终端打开。
bash ~/robot_ws/src/pharmacy_ISCC/tools/prepare_race.sh

# 终端二：等待业务初始化完成，确认定位后开始比赛。
bash ~/robot_ws/src/pharmacy_ISCC/tools/start_race.sh

# 停止单车业务及基础环境。
bash ~/robot_ws/src/pharmacy_ISCC/tools/stop_race.sh
```

启动脚本直接 source 工作空间，不做额外检查或交互确认。
准备脚本并行启动两个 launch，不调用比赛放行服务；业务内部原有启动延时不变。
比赛开始脚本直接调用 `/nav_pharmacy/start`，应在日志提示等待手动启动后执行。
停止脚本向当前用户的上述两个 launch 发送 TERM，由 roslaunch 退出并清理子节点；
不停止其他 ROS 项目，也不轮询确认退出。准备终端 Ctrl+C 同样请求关闭两个 launch。
这属于软件进程关闭，不是独立硬件急停。使用 `bash` 调用无需设置执行权限。

等效手动命令如下，两个 roslaunch 应分别在独立终端执行，均先 source 工作空间：

```bash
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_ISCC base_system.launch use_race_init:=false open_rviz:=false
# 另一终端，确认定位后：
source ~/robot_ws/devel/setup.bash
roslaunch pharmacy_ISCC single_car.launch
# 再开一个终端，等待业务初始化完成后：
source ~/robot_ws/devel/setup.bash
rosservice call /nav_pharmacy/start '{}'
```

single_car 固定 car1、关闭双车 TCP 和双车轮转，视觉不 respawn，默认一轮整板任务。
`max_rounds:=0` 连续运行。

双车分别在各自机器的 ROS Master 上操作；基础层沿用赛道初始化、不打开 RViz：

```bash
roslaunch pharmacy_ISCC base_system.launch use_race_init:=true open_rviz:=false
roslaunch pharmacy_ISCC dual_car.launch car_id:=1
rosservice call /nav_pharmacy/start '{}'
```

car2 业务传 `car_id:=2`，两车均需手动放行，car1 先发、car2 等令牌。
默认每车一轮，连续运行传 `max_rounds:=0`。
检查 vehicles.yaml、competition.yaml 和 vision.yaml 中的现场地址。

base_system 保留 start_navigation、start_camera、start_video_server、use_race_init，
并接收 open_rviz、output。默认 use_race_init=true、open_rviz=false，规划器保持 teb。
原主入口的基础参数现在传给 base_system；不再提供混合启动用的 start_base。

pharmacy_main 仅启动业务，保留 car_id、output、max_rounds、enable_dual_car、
start_dual_tcp、start_detect、start_nav_pharmacy、respawn_comm、respawn_detect、
respawn_nav_pharmacy、detect_start_delay、nav_start_delay、dual_publish_repeat、
dual_publish_interval。默认双车模式 false、TCP true；专用单/双车入口传入相应值。
视觉延时仍 3 秒，导航延时仍 6 秒；启动顺序和重启策略保持原值。

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

The Python2 ROS vision node keeps its existing /camera/rgb/image_raw input, visit cancellation and /board2_return Int32MultiArray output. It sends JPEG frames through private subprocess pipes to Python3.6 ONNX Runtime; the worker never opens a camera, network port or ROS publisher. Models are under resources/board2_yolo. The contract pins ROI, class order, RGB/white-pad/224/NCHW preprocessing, five-frame averaging and conservative confidence/margin rejection. Each visit resets history. free -> [0,0]; busy_5..busy_10 -> [1,5..10]. The existing navigation business already selects WAIT-0.wav or WAIT-5..10.wav; its motion and timing logic is unchanged.

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
