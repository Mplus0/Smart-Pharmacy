# 单车主任务导航完成判定（2026-10-17）

按用户要求，F1_yaofang_v5.py 的 move() 在 move_base 返回 SUCCEEDED 后直接返回 True，继续识别、取样、送样或返程流程。

删除主任务到达后的 stopped_pose(requested) 调用，不再计算定位与目标的到达误差，不再等待额外 1 秒连续停稳，不会因为独立坐标验收超时而停止主任务。

发送目标前仍检查目标是否在地图空闲格以及定位是否新鲜。move_base 的到达容差和避障继续生效；导航失败或超时仍停止任务。窗口停留与播报、一轮结束后停止的逻辑保留。

独立取点工具 waypoint_console.py 保留原来的误差和停稳验收。

启动命令不变：终端一 `roslaunch pharmacy_pkg single_car_base.launch`，定位确认后终端二 `roslaunch pharmacy_pkg single_car_task.launch`。修改在下次启动任务节点时加载。
