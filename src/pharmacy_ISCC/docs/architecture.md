# 架构与接口

## 依赖方向

入口 → 各功能模块 → 集中配置/公共工具。配置不导入节点；
记录模块不导入视觉或导航；通信模块不导入状态机。
板二 TCP 查询通过 `cancelled` 回调读取原来的退出条件，不反向导入视觉运行状态。

视觉运行状态只在 `perception/runtime.py` 内保存；
ROS 图像缓存及锁只在 `perception/image_input.py` 内保存。
板二模板字典由运行模块传入识别函数。板一几何/解码算法独立于发布状态。
导航状态机继承 `AudioAnnouncements`，播报方法保持原来的阻塞、停止和异常处理。
两个航点工具的相同 `finite`、导航与双车通信的相同 `safe_int` 提取为公共函数。

## 节点与话题

| 节点/入口 | 输入 | 输出 |
|---|---|---|
| F1_detect_code_v5.py | `/nav_state` Int32、相机 Image/CompressedImage 或 HTTP | `/cam_return` Int32MultiArray、`/board2_return` Int32MultiArray、`/board1_all_text` String |
| F1_yaofang_v5.py | 识别结果、`/dual_car/peer_done`、`/dual_car/peer_board1_all_text`、map/TF | move_base action、cmd_vel、nav_state、裁判话题、round_done |
| tcp_link_v2.py | `/dual_car/round_done`、`/board1_all_text`、对车 TCP | `/dual_car/peer_done` Int32MultiArray、`/dual_car/peer_board1_all_text` String |
| referee_client_v3.py | `/odometry/filtered` Odometry、map→base TF、三个裁判 String 话题 | 裁判 TCP JSON |

主导航服务 `~start` 使用 `std_srvs/Trigger`，只放行一次。
move_base action 名为 `move_base`，清图服务为 `/move_base/clear_costmaps`。
裁判话题为 `/referee_task`、`/referee_cv1`、`/referee_cv2`。
原有 latch、queue_size、重复发布次数与时间间隔均保留。
配置中的 `/cmd_vel`、相机 topic 和服务器地址亦保持基线原值。

## 消息约定

- `/cam_return`：每个任务六项 `[C, A, B, count, window_index, error_window]`，
  一条消息可有 1～4 个任务（6～24 项）。`window_index` 为 0～3，
  `error_window` 为首个干扰窗口的 1～4 编号，无干扰为 0。
  优先选样本数最多者，同数量按原窗口顺序。单车按全部合法任务配送；
  双车只执行首项，原 all_text 分享字段仍只描述首项。
- `/board2_return`：`[0, 0]` 空闲，`[1, wait_time]` 忙碌。
  导航将忙碌等待时间限制到 5～10 秒。
- `/dual_car/round_done`、`/dual_car/peer_done`：`[car_id, seq]`。
- all_text JSON：`car_id, seq, all_text, selected_index, selected_text, selected_msg`。
- 裁判 JSON：`id` 为字符串，`speed` 为平面速度，`odom` 为 map 下二维位置，
  `task` 为 R/A/B/C/1/2/3/4。**CV1 对应板二 WAIT-n，CV2 对应板一组合-窗口**。

## TCP

- 双车默认各监听 9001 / 9002；UTF-8 JSON 加换行。
  `type` 为 `round_done` 或 `board1_all_text`。保留 seq 去重、对车 IP 校验、
  可选 token、重复发送窗口及接收长度上限。两车地址来自 vehicles.yaml。
- 裁判默认 `192.168.124.11:8888`；UTF-8 JSON 加换行，2 Hz。
  speed 来自 filtered odometry；odom 来自 map→base_footprint/base_link TF，
  TF 更新频率保留 4 Hz。
- 板二默认 `192.168.124.24:9100`；请求 `GET\n`，读取首行 JSON。
  仅接受 `ok: true` 且 wait_time 为整数 0 或 5～10（拒绝 bool）。
  查询失败返回 None；调用者按当前基线默认 free 发布一次。
  导航退出板二或访问编号变化时不发布迟到结果。

## 状态与任务流程

8 等待令牌 → 9 前往板一 → 10 板一识别 → 11 按 C/A/B 取样 →
12 前往板二 → 13 板二识别 → 14 化验窗口 → 15 返回起点。

单车缓存整板剩余任务：完成一个化验窗口后，如尚有任务，先经过板一停车点，
再执行下一项，不重新识别、不提前回家。整板完成并回家才累计一轮。
双车仍按一个任务计轮；进入状态 15 时释放令牌。
车 2 可跳过板一导航，使用共享整板去掉对车已选项后的最优结果。

导航航点次序为 C/A/B/lab4/lab3/lab2/lab1/home/board2/board1；
坐标、朝向、超时、成功判断、错误退出、等待和语音顺序均保留。
导航失败会停止整场，不在重构中根据旧注释改为新的重试策略。

## 记录

SQLite `active_run` 与 `records` 表结构保留。记录发生在原确认点：
干扰二维码稳定确认、执行任务应用、板二结果首次接受。
MD5 输入为 `json.dumps(..., ensure_ascii=False, sort_keys=True, separators=(',', ':'))`
得到的 UTF-8 文本，与 Excel “具体内容”列一致。
导出先关闭 active_run，按 id 顺序写表，重入导出不重复写；迟到结果不入库。
记录错误按原逻辑记录日志，不改动识别/导航分支。
