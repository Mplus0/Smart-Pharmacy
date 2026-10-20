# 迁移清单

本次源：`../pharmacy_pkg/` 当前文件；未从历史 6.1verion 恢复任何实现。
目标为现有工作空间内的 `src/pharmacy_ISCC/` 功能包；
用户最新目录要求覆盖原 AGENTS.md 的独立工作空间要求。

| 原文件/目录 | 新位置 |
|---|---|
| scripts/dual_car_config.py | config/{competition,vision,vehicles}.yaml + src/pharmacy_iscc/config/{settings,paths}.py |
| scripts/board1_selection.py | src/pharmacy_iscc/perception/selection.py |
| scripts/recognition_records.py | src/pharmacy_iscc/recording/records.py |
| scripts/F1_detect_code_v5.py | 同名薄入口 + perception/{runtime,settings,board1,board2,image_input,image_utils}.py |
| 板二 TCP 查询 | communication/board2.py（退出条件由调用者传入） |
| 板一干扰语音 | navigation/audio.py 的 play_board1_error_audio |
| scripts/F1_yaofang_v5.py | 同名薄入口 + navigation/state_machine.py + navigation/audio.py |
| scripts/tcp_link_v2.py | 同名薄入口 + communication/dual_car.py |
| scripts/referee_client_v3.py | 同名薄入口 + communication/referee.py |
| scripts/record_waypoint.py | 同名薄入口 + navigation/record_waypoint.py |
| scripts/waypoint_console.py | 同名薄入口 + navigation/waypoint_console.py |
| scripts/test_move_to_waypoint.py | 同名薄入口 + navigation/test_move_to_waypoint.py |
| 重复 finite / safe_int | common/numeric.py |
| launch/*.launch | launch/，仅包引用、失效入口按下文调整 |
| config/compitation_para.yaml | config/compitation_para.yaml，原样保留、仍不自动加载 |
| yuyingwenjian/ | resources/audio/ |
| pictures/board_2/ | resources/board2_templates/ |
| tests/ | tests/fixtures/legacy/ 原样留档；适配的航点测试位于 tests/unit/ |
| result_md5/ | tests/fixtures/recording_baseline/，仅留档 |
| 原说明文档、VERSION_CONTROL.md | docs/legacy_*，仅历史参考 |

## 启动引用修复

现有源码中以下 Launch 引用了不存在的脚本：

- detect.launch：detect_abc_for_EPRobot.py → F1_detect_code_v5.py。
- pharmacy_app_car2.launch：tcp_link_car2.py / referee_client_v2_car2.py /
  F1_detect_code_v4_car2.py → 当前 tcp_link_v2.py / referee_client_v3.py /
  F1_detect_code_v5.py，并为每个节点显式设置 CAR_ID=2。
- pharmacy_task_car2.launch：F1_yaofang_v4_car2.py → 当前 F1_yaofang_v5.py，
  显式设置 CAR_ID=2；调度参数遵循当前实现默认 false / 1。

原分层节点名保留，避免改变私有服务命名；主业务 Launch 原参数不变。
新增 dual_car_task.launch 仅组合现有主入口并显式启用双车，不修改状态机。
无法声称失效的旧 v4 启动路径已做运行等价验证；功能唯一基线仍为当前 scripts。

## 审计

`baseline_sha256.json` 保存开始重构时原包内所有现存文件的 SHA-256。
离线检查核对文件集合、文件内容及全部业务配置（资源路径除外）。
`test_business_function_bodies_preserved` 对比 AST，覆盖保留函数及方法；
需要参数注入、main 入口调整的函数由接口测试及人工核对覆盖。
历史测试部分仍断言板二强制 free 或只发六项，已落后于当前实现；
留档不删除，也不把这些断言反向施加到当前业务代码。
