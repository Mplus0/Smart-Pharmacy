# AGENTS.md

## 适用范围

本文件适用于 `dual_car_refactor/` 及其后续重构产物。

- 本次只修改 `dual_car_refactor/` 内的代码、配置、launch、资源、测试和文档。
- `pharmacy_mplus0/` 仅用于参考 Catkin 包骨架、目录分层、launch 组织和资源安装方式；其业务代码、算法、接口和状态机均不具备参考价值，禁止复制、导入或基于其逻辑改写。
- 不得修改 `pharmacy_mplus0/`、`pharmacy_mplus0_debug/`、`pharmacy_pkg/`，也不得修改底盘、导航、摄像头及其他厂家功能包。
- 未经用户明确确认，不得覆盖现有 `pharmacy_mplus0`，也不得在同一工作空间制造两个同名 ROS 包。

## 重构依据与优先级

发生冲突时按以下顺序处理：

1. 用户当前指令；
2. `STRUCTURE_REFACTOR_PLAN.md`；
3. `dual_car_refactor/` 当前可运行代码及其实际行为；
4. 《2026CRAIC 智慧药房比赛规则》；
5. 现有 README 和历史说明。

本次目标是结构规范化和等价迁移，不是重新设计比赛方案。先建立行为基线，再分阶段重构；每个阶段都应可验证、可回退。

## 目标结构

按方案建立独立 Catkin 包能力，并保持适度粒度：

- 5 个完整 ROS 节点脚本：`vision_node.py`、`main_controller.py`、`dual_car_link.py`、`referee_reporter.py`、`waypoint_tester.py`；
- 2 个共享模块：`config.py`、`task_logic.py`；
- 配置拆分为 `strategy.yaml`、`waypoints.yaml`、`vision.yaml`、`communication.yaml`；
- 模板和音频放入包内资源目录，使用 `rospkg` 或 `$(find ...)` 定位；
- 不为单个辅助函数继续拆分文件，不引入新的状态机框架、异步架构或多层封装。

目标名称遵循重构方案；但旧包改名、目录迁移和最终覆盖由用户另行确认。在确认前，所有实施内容保持在本文件适用范围内。

## 必须保持的行为

以下内容属于兼容接口，不得在本次重构中改变：

- ROS 状态编号 `8～15`、状态转换条件和发布时序；
- 取样访问顺序 `C → A → B`，已完成窗口在导航重试时不得重复访问；
- 车 1 先发、车 2 等待；本车完成化验窗口任务并进入返程状态时释放对车；提前收到的令牌必须缓存到回到起点后使用；
- 车 2 优先使用车 1 的板一完整结果，排除已选任务；共享数据不可用时回退本车识别；
- `/nav_state`、`/cam_return`、`/board2_return`、`/board1_all_text`、`/dual_car/round_done`、`/dual_car/peer_done`、`/dual_car/peer_board1_all_text` 和三个 `/referee_*` 话题的名称、类型、字段顺序、latch 与重复发布行为；
- 双车 TCP 的 `round_done`、`board1_all_text` JSON 字段、序列号、去重、来源校验、可选 token、重试和断线恢复；
- 裁判 JSON 的键、类型和换行分隔；`speed` 继续来自 `/odometry/filtered.twist`，`odom` 继续来自 TF `map → base_footprint`，失败后回退 `base_link`；
- 板一选择规则、板二 `free/busy_5～busy_10` 识别结果、稳定帧数、正式发布次数；
- 到达取样和送样区域后的停车、停留、语音播报及等待时机；识别板二空闲时尽快通过，忙碌时按识别秒数等待；
- 所有当前默认参数，包括看似不合理的调试默认值。行为修正必须另开任务。

不得因比赛规则中出现额外描述而擅自扩展裁判协议、改变任务分配策略或增加新功能。

## 实现要求

- ROS1 Melodic / Ubuntu 18.04 环境优先，保持 Python 2.7 兼容；禁止引入 f-string、dataclass、仅 Python 3 可用的类型标注或标准库接口。
- 先等价迁移，再整理命名；禁止同一提交同时做结构迁移、算法优化和参数调优。
- 不得导入 `pharmacy_pkg`、`pharmacy_mplus0` 或其他旧智慧药房业务包的 Python 模块。
- 清除 `pkg="pharmacy_pkg"`、`$(find pharmacy_pkg)` 和绝对工作空间路径；正常 ROS/硬件依赖仍可保留。
- 所有配置键迁移时必须有旧值到新字段的明确映射，默认值逐项一致。
- 正式入口文件不保留 `v2/v3/v5` 版本号；旧入口仅在验证完成后改为兼容包装或删除。
- 优先做最小改动，保留函数输入输出、异常处理、日志时机和线程模型。
- 不提交模板替换、识别算法更换、导航点调整、TCP 频率修改、语音系统重写或单车模式新增。
- 不修改比赛之外的仓库文件，不进行无关格式化或全仓库重命名。

## 验证要求

每个阶段至少完成适用的检查：

- Python 语法检查，且不得使用 Python 3 专属语法；
- launch XML 和 YAML 可解析；
- `package.xml`、`CMakeLists.txt`、`setup.py` 与实际依赖和安装节点一致；
- 板一纯逻辑测试比较完整 `selected_msg`；
- 配置迁移测试比较关键旧值与新值；
- 双车 TCP、裁判 payload、序列号去重和错误输入测试；
- 固定图片或消息下，新旧 ROS 输出、状态序列和协议内容一致；
- 条件允许时执行 `catkin_make`、`rospack find` 和 launch 启动检查；
- 无法执行硬件、ROS Master、摄像头、TF、move_base 或双车实车测试时，明确列出未验证项，不得声称已通过。

每次提交只完成一个阶段。完成后报告：修改文件、保持不变的接口、执行的测试、未执行的实车验证和已知风险。
