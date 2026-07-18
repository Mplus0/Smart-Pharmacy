# 现有行为基线

本文档记录同步三项优化后的 `dual_car_refactor` 实现中必须保持不变的行为。
它只作为等价迁移的静态核对依据，不定义新功能，也不修正现有行为。

## 1. 迁移边界

- 迁移来源：`dual_car_refactor/`，只读。
- 重构产物：`pharmacy_mplus0_refactor/`。
- 正式部署时，目录会以 `robot_ws/src/pharmacy_mplus0/` 放置，ROS 包名和 Python 包名均为 `pharmacy_mplus0`。
- 只移除对旧智慧药房业务包 `pharmacy_pkg` 的代码、资源和 launch 身份依赖。
- `robot_navigation`、`astra_camera`、`web_video_server` 等同级 ROS 或硬件功能包仍是正常外部依赖，不因本次重构改名或迁移。

## 2. 节点职责

| 当前脚本 | 当前节点名 | 必须保持的职责 |
|---|---|---|
| `F1_detect_code_v5.py` | `detect_abc`，anonymous | 板一、板二识别及结果发布 |
| `F1_yaofang_v5.py` | `nav_pharmacy` | 状态机、导航、语音、裁判话题和双车令牌 |
| `tcp_link_v2.py` | `dual_car_tcp_link_carN` | 双车 ROS/TCP 桥接 |
| `referee_client_v3.py` | `referee_client_node`，anonymous | 裁判状态采集和 TCP 上报 |
| `test_move_to_waypoint.py` | `test_move_to_waypoint` | 单航点导航测试 |

## 3. 状态机

状态编号保持固定：

| 状态 | 编号 |
|---|---:|
| `WAIT_TURN` | 8 |
| `GO_TO_BOARD1` | 9 |
| `BOARD1_RECOGNIZING` | 10 |
| `GO_TO_PICKUP_WINDOWS` | 11 |
| `GO_TO_BOARD2` | 12 |
| `BOARD2_RECOGNIZING` | 13 |
| `GO_TO_LAB_WINDOW` | 14 |
| `GO_BACK_HOME` | 15 |

正常流程为 `8/9 → 10 → 11 → 12 → 13 → 14 → 15 → 8/9`：车 1 初始进入 9，车 2 初始进入 8。

- 取样访问顺序固定为 `C → A → B`。
- `pickup_C_done`、`pickup_A_done`、`pickup_B_done` 在导航重试时阻止重复取样。
- 本车完成化验窗口停留和播报、进入状态 15 时发布 `round_done`。
- 发布 `round_done` 前先将本车 `/referee_active` 置为 `False`；收到有效对车令牌后置为 `True`。
- 提前收到的 `peer_done` 只缓存；本车回到起点或处于状态 8 后才使用。
- 车 2优先使用车 1的完整板一结果，清空车 1的 `selected_index` 后重新选择；不可用时回退本车板一识别。

## 4. ROS 接口

| 话题 | 类型 | 数据与发布行为 |
|---|---|---|
| `/nav_state` | `std_msgs/Int32` | 状态 8～15，latch |
| `/cam_return` | `std_msgs/Int32MultiArray` | `[C,A,B,count,selected_index,error_window]` |
| `/board2_return` | `std_msgs/Int32MultiArray` | 空闲 `[0,0]`；忙碌 `[1,5..10]` |
| `/board1_all_text` | `std_msgs/String` | JSON，latch，正式结果连续发布 3 次，间隔 0.03 秒 |
| `/dual_car/round_done` | `std_msgs/Int32MultiArray` | `[car_id,seq]` |
| `/dual_car/peer_done` | `std_msgs/Int32MultiArray` | `[peer_id,seq]`，接收后发布 3 次，间隔 0.05 秒 |
| `/dual_car/peer_board1_all_text` | `std_msgs/String` | 对车 JSON，latch，发布 3 次，间隔 0.03 秒 |
| `/referee_task` | `std_msgs/String` | `R/A/B/C/1/2/3/4` |
| `/referee_cv1` | `std_msgs/String` | `WAIT-0` 或 `WAIT-5`～`WAIT-10`，latch，发布 3 次 |
| `/referee_cv2` | `std_msgs/String` | 如 `AB-1`，latch，发布 3 次 |
| `/referee_active` | `std_msgs/Bool` | 当前任务令牌车为 `True`，latch |

`/referee_cv1` 和 `/referee_cv2` 的重复发布间隔均为 0.05 秒。

## 5. 板一选择与视觉发布

- 有效文本为 `""`、`A`、`B`、`C`、`AB`、`AC`、`BC`、`ABC`。
- 按包含样本数选择，分数相同时选择列表中靠前项。
- `error_window` 是第一个非法文本的一基窗口编号，无非法文本时为 0。
- 板一完整 `all_text` 连续 3 帧完全一致后，只正式发布一次。
- 无有效组合的帧不加入板一稳定历史。
- 板一识别路径顺序为：四个大窗口框主路径、四二维码直接兜底、旧透视兜底。
- 整图直接二维码兜底当前默认关闭。
- 板二使用黑框粗定位、白色内区二次对齐和固定 ROI，不再保留模板、强制预设或键盘 TCP 策略。
- 状态模型分类 `busy/idle`，数字模型分类 `10/5/6/7/8/9`；连续 5 个有效对齐帧按类别概率滑动平均后，只正式发布一次。
- 板二内部标签保持 `free`、`busy_5`～`busy_10`，ROS 输出仍为 `[0,0]` 或 `[1,5..10]`。
- 黑框或内白区定位失败时清空概率缓存，并保持状态 13 继续等待。

## 6. 双车 TCP

`round_done` 格式：

```json
{"type":"round_done","car_id":1,"seq":1}
```

`board1_all_text` 必须保持字段：

```json
{"type":"board1_all_text","car_id":1,"seq":1,"all_text":["ABC","AB","A",""],"selected_index":0,"selected_text":"ABC","selected_msg":[1,1,1,3,0,0]}
```

- JSON 使用换行分隔。
- 可选共享口令字段名为 `token`；当前默认值为 `None`。
- 默认检查来源 IP。
- `round_done` 与板一结果分别按序列号去重，旧序列号不触发 ROS 发布。
- 发送持续时间 3.0 秒，重试间隔 0.20 秒，socket 超时 2.0 秒，最大行长度 2048 字符。

## 7. 裁判 TCP

默认 payload：

```json
{"id":"1","speed":0.0,"odom":[0.0,0.0],"task":"R","CV1":"None","CV2":"None"}
```

- JSON 使用换行分隔，以 2 Hz 发送。
- `speed` 只来自 `/odometry/filtered.twist` 的平面速度，并保留两位小数。
- `odom` 来自 TF `map → base_footprint`，失败后回退 `map → base_link`，并保留两位小数。
- TF 更新频率为 4 Hz；查询失败时保留上一次成功位置。
- TCP 断线后自动重连，socket 超时 3.0 秒，重连等待 0.5 秒。
- 裁判服务端为 `192.168.124.6:8888`。
- 默认仅 `/referee_active=True` 的任务令牌车连接和发送；失去令牌时关闭已有连接。
- 重新获得上报权时默认将 `task/CV1/CV2` 重置为 `R/None/None`。

## 8. 关键默认行为

- 双车模式在主控中固定开启。
- 车 1 `start_active=True`，车 2 `start_active=False`。
- 车 2 `use_peer_board1_result=True`，车 1为 `False`。
- 板一和板二等待超时均为 0，即无限等待。
- 状态机频率 5 Hz，识别结果等待频率 10 Hz，导航超时 30 秒。
- 取样与化验窗口停留均为 1.1 秒。
- 到达最后一个取样窗口和化验窗口后先异步启动对应语音，再执行后续清图/停留；成功到达后的清图开关默认关闭，导航失败后的清图逻辑不变。
- 视觉主循环限制为 6 Hz，默认订阅 `/camera/rgb/image_raw`，允许回退 HTTP。
- 板二模型、定位阈值和 ROI 来自 `smart-pharmacy-board2` 训练测试留档；航点、IP、端口、音频名和其他默认配置仍按对应迁移来源核对。

## 9. 本机不执行的验证

本机不是正式运行环境，因此不执行 ROS Master、move_base、TF、摄像头、HTTP 视频、音频、裁判 TCP、双车 TCP、`roslaunch`、`rospack`、`catkin_make` 或实车测试。这里只执行不会启动业务代码的静态结构检查。
