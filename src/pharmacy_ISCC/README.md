# 智慧药房代码重构包

本目录是现有 `Smart-Pharmacy` catkin 工作空间中的功能包，**不是独立工作空间**。
按用户最后指令，保存路径为 `Smart-Pharmacy/src/pharmacy_ISCC/`。
文件夹名为 `pharmacy_ISCC`，ROS 包名及 Python 包名均为 `pharmacy_iscc`；
因此使用 `roslaunch pharmacy_iscc ...`，不要用文件夹名作 ROS 包名。

功能基线为相邻 `pharmacy_pkg/scripts/` 的当前版本；原包不修改。
重构只调整模块、导入、资源定位、配置存储及失效启动引用，保持业务流程。
本次开发机没有 ROS/catkin 和机器人环境，构建、真实 ROS 通信、图像识别及实车运行均待验证。

## 目录

```text
Smart-Pharmacy/
├── src/CMakeLists.txt              # 现有工作空间顶层文件，不修改
└── src/pharmacy_ISCC/
    ├── CMakeLists.txt              # 功能包构建/安装规则
    ├── package.xml
    ├── setup.py
    ├── launch/
    ├── config/                    # competition.yaml / vehicles.yaml / vision.yaml
    ├── scripts/                   # 7 个薄入口，保留当前脚本文件名
    ├── src/pharmacy_iscc/
    │   ├── common/                # 共用数值工具
    │   ├── config/                # 配置加载、资源定位
    │   ├── perception/            # 图像输入、板一、板二、稳定发布
    │   ├── navigation/            # 状态机、语音、航点工具
    │   ├── communication/         # 双车、裁判、板二 GET
    │   └── recording/             # SQLite、MD5、Excel
    ├── resources/                 # audio/、board2_templates/
    ├── tests/                     # unit/、integration/、fixtures/
    └── docs/                      # 架构、接口、迁移、验收及历史资料
```

## 参数与默认行为

- `config/competition.yaml`：话题、状态号、导航、双车 TCP、裁判配置及播报映射。
- `config/vehicles.yaml`：车辆编号、对车地址、初始令牌、原点、结果共享开关。
- `config/vision.yaml`：板一/板二识别与图像输入参数。
- `src/pharmacy_iscc/config/settings.py` 统一加载配置，恢复原来的整数键及元组类型。
  原 `compitation_para.yaml` 原样保留；当前主业务原本不加载它，本次也不将其接入。
- 板二仍默认 TCP，查询无有效结果时按当前基线回退 `free`；
  `force_label_for_debug` 仍为 `None`。图像输入默认 ROS 原始图像，并允许 HTTP 兜底。
- 板一需四个真实窗口、连续三帧内容一致；单车通过一条消息传递整板任务；
  双车仍只执行首项、分享整板，由对车从剩余项选择。
- 主 Launch 的 `enable_dual_car` **原值就是 false**，保留不变。
  TCP 节点的启动开关默认 true，与是否启用双车任务调度是两个不同参数。
  `dual_car_task.launch` 是显式设置双车调度的新入口。
- `max_rounds` 默认 1；0 表示持续运行。车 1 默认先发，车 2 等待令牌。
  车辆到达化验窗口、停留且播报结束，进入状态 15 时释放令牌。

## 机器人侧依赖与构建（本次未执行）

沿用车辆现有 ROS 发行版及其 Python 版本（原代码兼容 Melodic/Python 2）。
不要把开发机 Python 3 的依赖覆盖到车辆 ROS Python 环境。

所需 ROS 依赖已列入 `package.xml`，包括 rospy、消息包、actionlib、tf、
cv_bridge、robot_navigation、astra_camera、web_video_server、ar_track_alvar。
`robot_navigation` 等硬件包使用现有工作空间版本，不在本包内重新实现。
Python 依赖包括 NumPy、OpenCV、PyYAML、rospkg、openpyxl、pyzbar；
系统还需 zbar 共享库和 SoX 的 `play` 命令。

在已安装 ROS 的 Linux 车辆/构建机：

```bash
cd /实际路径/Smart-Pharmacy
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src/pharmacy_ISCC --ignore-src -r -y
# 用当前 ROS 对应的 Python/pip 安装 pyzbar；安装系统 zbar 和播放器
sudo apt-get install libzbar0 sox
# Python 3 ROS：
python3 -m pip install pyzbar
# Python 2 ROS 使用兼容 Python 2 的版本：
# python2 -m pip install 'pyzbar==0.1.8'
catkin_make --pkg pharmacy_iscc
source devel/setup.bash
rospack find pharmacy_iscc
python -c 'from pharmacy_iscc.config.settings import COMMON; print(COMMON["paths"])'
```

安装空间也应验证 `catkin_make install` 后的资源定位及节点导入。
构建命令是交接步骤，**不代表本次已构建通过**。
包的维护者和 `TODO` 许可证字段沿用原项目，未擅自声明许可证。

## 启动

每个终端先 source 现有工作空间的 `devel/setup.bash`。每辆车使用自己的 ROS Master。

单车：

```bash
roslaunch pharmacy_iscc single_car_base.launch
# 在 RViz 完成定位检查后，另开终端：
roslaunch pharmacy_iscc single_car_task.launch
# 节点初始化且 move_base 就绪后，手动放行：
rosservice call /nav_pharmacy/start '{}'
```

双车：分别在两辆车启动基础系统并完成定位；下面车号按车辆改为 1 或 2。

```bash
roslaunch pharmacy_iscc base_camera_nav.launch
roslaunch pharmacy_iscc dual_car_task.launch car_id:=1
roslaunch pharmacy_iscc referee_only.launch car_id:=1
rosservice call /nav_pharmacy/start '{}'
```

两车都需要手动放行；车 2 放行后仍等待令牌。需要持续运行时传 `max_rounds:=0`。
也可使用原统一入口：

```bash
roslaunch pharmacy_iscc pharmacy_main_no_referee.launch car_id:=1 enable_dual_car:=true
```

此入口默认启动基础系统，不要和已经运行的基础 Launch 重复启动。
car2 分层入口仍保留：`pharmacy_base_car2.launch`、
`pharmacy_app_car2.launch`、`pharmacy_task_car2.launch`；
需要双车调度时给 task 层显式传 `enable_dual_car:=true`。
该旧任务入口节点名保持 `F1_yaofang_v4_car2`，其放行服务为
`/F1_yaofang_v4_car2/start`。

独立工具：

```bash
rosrun pharmacy_iscc waypoint_console.py --help
rosrun pharmacy_iscc record_waypoint.py --help
roslaunch pharmacy_iscc test_move_to_waypoint.launch target:=A dry_run:=true
```

## 记录与测试

记录默认写到新包的 `result_md5/`，每车 `car1.sqlite3` / `car2.sqlite3`；
每次任务启动清空该车旧记录，结束时覆盖 `recognition.xlsx`。
视觉与导航必须使用相同记录目录。在只读安装空间或自定义部署中，
在所有相关终端统一设置 `PHARMACY_RECORD_DIR` 为同一个可写的绝对路径。
原结果文件仅存于 `tests/fixtures/recording_baseline/`，不会作为新任务的活动数据库。

开发机离线检查（Python 3，PyYAML、openpyxl）：

```bash
cd Smart-Pharmacy/src/pharmacy_ISCC
python tests/run_offline.py
```

`tests/integration/test_offline_contracts.py` 使用假的 ROS、图像库和 socket，
只验证消息/状态/协议/记录边界，不启动节点、不连接外部服务、不控制车辆。
历史脚本和识别数据保存在 `tests/fixtures/legacy/`；旧脚本含旧路径和过时断言，
不纳入本次测试发现，也不将旧报告的通过状态作为本次验证。

详见 [架构与接口](docs/architecture.md)、[迁移清单](docs/migration.md)、
[验收记录](docs/validation.md)。
