# pharmacy_ISCC 维护约束

以当前 pharmacy_ISCC 为修改起点、相邻 pharmacy_pkg 为业务逻辑基线。保留有效迁移，
不恢复历史旧版实现，不修改 pharmacy_pkg 或其他功能包，不提交 Git。

## 固定文件结构

根目录仅保留 .gitattributes、README.md、AGENTS.md、CMakeLists.txt、package.xml、
setup.py 及以下目录；不额外增加模块。

- scripts：__init__.py、F1_detect_code_v5.py、board1_detector.py、board2_detector.py、
  board2_tcp.py、F1_yaofang_v5.py、audio.py、tcp_link_v2.py、referee_client_v3.py、
  config.py、recognition_records.py、navigation_utils.py。
- tools：waypoint_console.py、record_waypoint.py、test_move_to_waypoint.py。
- launch：base_system.launch、pharmacy_main.launch、single_car.launch、dual_car.launch、
  referee_only.launch、detect.launch、test_move_to_waypoint.launch、ar_marker_detection.launch。
- config：competition.yaml、vehicles.yaml、vision.yaml。
- resources：audio/ 中的 WAV、board2_templates/ 中的 PNG。

不保留嵌套 src、docs、tests、compitation_para.yaml、旧模块或旧 Launch 别名。
完整目录树和迁移映射见 README.md。

## 模块边界

视觉节点拥有唯一图像输入、识别状态和发布器；板一/板二算法复用输入帧，不能再开相机。
board1_detector 合并原选择函数，保留稳定发布回调和历史队列的调用顺序。
board2_tcp 是查询客户端，不是 ROS 节点；保留实时取消检查和节点侧访问序号检查。
主状态机保留导航和任务调度；audio 负责原播放机制；navigation_utils 供正式代码和
工具共同使用。正式代码不得反向导入 tools，不重复实现同一功能。

config.py 统一加载三个 YAML，提供 COMMON、CARS、车辆选择及路径接口；业务参数
仅在对应 YAML 维护，必要 tuple 类型由读取端恢复。ROS 包名 pharmacy_ISCC、Python
命名空间 pharmacy_iscc 不变，通过 setup.py 把 scripts 映射为包，无嵌套源码目录。

## 行为约束

不得调整算法、状态编号、航点顺序、阈值、超时、等待、语音映射、协议或消息格式。
保留板一样本数量优先、同分原窗口顺序和整板队列；单车缓存和双车令牌逻辑不变。
板二默认 TCP，GET 加换行、一行 JSON、无有效结果默认 free；vision 模式保留模板。
保留双车 TCP 字段、重发、去重、IP/token 检查；保留裁判字段、发送频率和重连。
保留 JSON/UTF-8 MD5、SQLite 表结构和记录时机、Excel 格式与覆盖策略。
全部 WAV/PNG 保留，不迁移历史数据库/报表。

基础系统与业务分开启动：基础启动、定位确认、业务启动、/nav_pharmacy/start 放行。
保留视觉 3 秒/导航 6 秒启动延时、各入口重启策略及双车开关语义；基础差异通过
base_system 参数表达。独立视觉保留话题隔离；AR 参数、话题和 TF 保持原值。

## 检查与交付

先检查引用再迁移或删除。静态对照 pharmacy_pkg，核对导入、配置键/类型、资源、
Launch XML/传参及 catkin 安装规则。开发机无运行环境，本次只编辑和静态检查，不
运行项目、测试、语法编译或构建。后续仅在用户授权且环境具备时进行运行验证。
如实报告未验证内容，不将文本一致性表述为实车功能验收。更新 README 的实际目录、
迁移清单及新启动命令，保持 LF。
