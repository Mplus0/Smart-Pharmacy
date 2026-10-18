# 板二键盘 TCP 输入

已有视觉节点 `F1_detect_code_v5.py` 内置客户端，连接提供的 `board2_keyboard_tcp_server.py`。不增加 ROS 节点，不改变主任务的 `/board2_return` 接口。

配置在 `scripts/dual_car_config.py` 的 `board2` 中：

- `result_source: "tcp"`：当前默认使用 TCP；改为 `"vision"` 才恢复图像模板识别。
- `tcp_host: "192.168.124.11"`：部署时填写运行键盘服务端的电脑地址，这里暂用现有配置中的电脑地址；与裁判通信独立。
- `tcp_port: 9100`：与键盘脚本默认端口一致。
- `tcp_timeout_sec: 1.0`、`tcp_retry_interval_sec: 0.2`：查询超时和重试间隔。
- `force_label_for_debug: None`：关闭原来的固定 free；TCP 模式不使用该字段。

车辆到达板二进入识别状态后，建立 TCP 连接并发送 `GET\n`，读取一行 UTF-8 JSON，然后关闭连接。只有 `ok` 为 true 且 `wait_time` 是 0 或 5～10 的整数才接受：

- 0 → `/board2_return` 发布 `[0, 0]`。
- 5～10 → 发布 `[1, wait_time]`。

每次访问只发布一次有效结果。TCP 结果直接复用原有标签转换，不进行多帧图像确认，也不依赖相机取帧。主任务继续执行 MD5 记录、裁判字段发布、同步播报和等待，再进行下一次导航。忙碌时先播报完，再等待指定秒数。

连接失败、无输入（`no_value`）、非法响应均不发布空闲结果，而是继续查询。主任务原有 `board2_wait_timeout_sec=5` 保持不变：到达后 5 秒内未收到有效结果将停止任务，因此应提前在服务端输入值。

服务端输入值不会在 GET 后自动清空，也不会按轮次过期。下一轮若不修改输入，仍会返回上一次的值。`clear` 后需重新输入；车辆已接受结果后再修改，只影响下一次访问。

客户端仅使用标准库 socket/json，无新增 Python 依赖。本次仅静态检查，未运行服务端、车端代码或测试；部署时需验证 0、5～10、clear、连接失败和重复轮次场景。
