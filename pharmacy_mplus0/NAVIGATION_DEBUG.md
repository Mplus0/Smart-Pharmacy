# 导航参数调试日志

> 本文件记录每次导航参数的修改，用于追踪调试过程和方便回退。
> 每次修改包含：日期、修改参数的文件路径、参数名、修改前的值、修改后的值、修改原因。

---

## 修改记录

| # | 日期 | 文件 | 参数 | 旧值 | 新值 | 原因 |
|---|------|------|------|------|------|------|
| 1 | 2026-06-01 | teb_local_planner_params.yaml | max_vel_x | 1.9 | 1.0 | 降速调试地图点位精准度 |
| 2 | 2026-06-01 | teb_local_planner_params.yaml | max_vel_x_backwards | 1 | 0.5 | 降速调试地图点位精准度 |
| 3 | 2026-06-01 | teb_local_planner_params.yaml | acc_lim_x | 1.6 | 1.0 | 降速调试地图点位精准度 |
| 4 | 2026-06-01 | teb_local_planner_params.yaml | xy_goal_tolerance | 0.15 | 0.10 | 收紧停靠精度 |
| 5 | 2026-06-01 | teb_local_planner_params.yaml | yaw_goal_tolerance | 0.55 | 0.30 | 收紧朝向精度 |
| 6 | 2026-06-01 | teb_local_planner_params.yaml | yaw_goal_tolerance | 0.30 | 0.10 | 进一步提高定位精度 |
| 7 | 2026-06-01 | teb_local_planner_params.yaml | yaw_goal_tolerance | 0.10 | 0.03 | 再次提高朝向精度 |
| 8 | 2026-06-01 | teb_local_planner_params.yaml | yaw_goal_tolerance | 0.03 | 0.5 | 提高角度容忍度，避免长时间原地微调 |
| 9 | 2026-06-01 | teb_local_planner_params.yaml | acc_lim_theta | 1 | 0.5 | 降低转向加速度，减少转向抖动 |
| 10 | 2026-06-01 | teb_local_planner_params.yaml | xy_goal_tolerance | 0.10 | 0.05 | 进一步收紧停靠精度，5cm 判定到达 |
| 11 | 2026-06-01 | waypoints.yaml | board1 | (0.861, 0.0, 0.0) | (0.711, 0.0, 0.0) | 实测修正 |
| 12 | 2026-06-01 | waypoints.yaml | board2 | (-0.147, 3.813, 3.1416) | (-0.107, 3.913, 3.1416) | 实测修正 |
| 13 | 2026-06-01 | waypoints.yaml | exam_A | (0.685, 2.516, 1.5708) | (0.785, 2.616, 3.1414) | 实测修正 |
| 14 | 2026-06-01 | waypoints.yaml | exam_B | (1.463, 2.980, 1.5708) | (1.543, 2.950, 1.5708) | 实测修正 |
| 15 | 2026-06-01 | waypoints.yaml | exam_C | (1.530, 1.985, 1.5708) | (1.530, 2.085, 1.5708) | 实测修正 |
| 16 | 2026-06-01 | waypoints.yaml | lab_1 | (-1.639, 2.471, -1.5708) | (-1.639, 2.521, -1.5707) | 实测修正 |
| 17 | 2026-06-01 | waypoints.yaml | lab_2 | (-0.846, 1.771, -1.5708) | (-0.846, 1.921, 0.0) | 实测修正 |
| 18 | 2026-06-01 | waypoints.yaml | lab_3 | (-1.669, 1.238, -1.5708) | (-1.669, 1.538, -1.5707) | 实测修正 |
| 19 | 2026-06-01 | waypoints.yaml | lab_4 | (-0.885, 0.780, -1.5708) | (-0.835, 0.950, -1.5708) | 实测修正 |
| 20 | 2026-06-01 | teb_local_planner_params.yaml | yaw_goal_tolerance | 0.5 | 0.10 | 收紧朝向精度，恢复小角度定位 |

---

## 回退指南

回退时，根据上表中的记录找到对应的文件和参数，恢复为「旧值」即可。

### 当前参数状态快照（截至 2026-06-01）

**teb_local_planner_params.yaml**:
- `max_vel_x`: 1.0（原 1.9）
- `max_vel_x_backwards`: 0.5（原 1）
- `acc_lim_x`: 1.0（原 1.6）
- `xy_goal_tolerance`: 0.05（原 0.15）
- `acc_lim_theta`: 0.5（原 1）
- `yaw_goal_tolerance`: 0.10（原 0.55）

**waypoints.yaml**:
| 航点 | x | y | yaw |
|------|---|---|-----|
| start | 0.0 | 0.0 | 0.0 |
| board1 | 0.711 | 0.0 | 0.0 |
| board2 | -0.107 | 3.913 | 3.1416 |
| exam_A | 0.785 | 2.616 | 3.1414 |
| exam_B | 1.543 | 2.950 | 1.5708 |
| exam_C | 1.530 | 2.085 | 1.5708 |
| lab_1 | -1.639 | 2.521 | -1.5707 |
| lab_2 | -0.846 | 1.921 | 0.0 |
| lab_3 | -1.669 | 1.538 | -1.5707 |
| lab_4 | -0.835 | 0.950 | -1.5708 |
