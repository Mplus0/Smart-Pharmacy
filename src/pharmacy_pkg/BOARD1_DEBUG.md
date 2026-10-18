# 板一静态识别调试，2026-10-16

本次修复基于实车 `349ca1c`，没有使用未通过等价审查的 `pharmacy_pkg(1)` 替换整个包。

当前测试板按左上、右上、左下、右下排列为 `D / 空 / ABC / BC`。预期选择第三格 `ABC`，报告第一格干扰，`/cam_return` 为 `[1,1,1,3,2,1]`。第五字段是零基槽索引，末字段是从 1 开始的干扰窗口号，0 表示没有干扰。

本次行为：

- 仅在四个实际外框满足 2×2 排列后识别和发布，关闭三框推算和严格模式下的旧定位兜底。
- 用 3×3 闭运算修复细边框的小断口，拒绝非凸、矩形覆盖率不足或网格偏差过大的背景候选。
- 保留解码得到的干扰内容；默认内裁剪失败时回退到较宽裁剪，恢复贴近边框的大二维码。
- 缺框、无有效任务或相机读取失败时清空稳定计数；同一张 ROS 缓存图不重复计为新帧。帧身份使用接收时间，不依赖相机自身不稳定的时间戳。
- 当前逻辑 car1 的 HTTP 视频备用地址更新为 `192.168.124.6`，实际主输入仍为 `/camera/rgb/image_raw`。

配置位于 `scripts/dual_car_config.py`：`require_four_windows=True`、`window_complete_from_three=False`、`window_grid_max_normalized_score=0.75`。归一化网格分数越小表示四框排列越一致；拍摄角度或场地改变后需要重新验证，不应仅为通过识别放宽门槛。

## 运行

静态调试使用独立视觉节点，导航主程序应保持停止：

```bash
rosrun pharmacy_pkg F1_detect_code_v5.py
rostopic echo /board1_all_text
rostopic echo /cam_return
```

在视觉节点订阅就绪后发布 `/nav_state=10` 才会进入板一识别；每次成功只发布一轮。再次测试需先发状态 9，再发 10。

```bash
rostopic pub -1 /nav_state std_msgs/Int32 'data: 9'
rostopic pub -1 /nav_state std_msgs/Int32 'data: 10'
```

## 验证

现有 Python 2.7、ROS Melodic、OpenCV 3.2 和 pyzbar 环境，无新增依赖：

```bash
PYTHONDONTWRITEBYTECODE=1 python tests/test_board1_recognition.py
PYTHONDONTWRITEBYTECODE=1 python tests/verify_board1_live.py --output /tmp/board1_live_verification/results.json
```

第一条包含真实原图回归、干扰码、大二维码裁剪、四种缺框、空白画面、稳定计数重置、单轮发布与缓存帧测试。无需 ROS master，不会启动业务节点。

第二条需要相机已发布图像，专用于当前 `D / 空 / ABC / BC` 布局。它读取实时图像并在诊断进程内模拟结果接收器，不向导航、裁判或速度话题发布。三轮均须得到预期消息，结果及原图写入指定临时目录。

本次只验证了当前板一画面和静止视角；导航、送样、双车交接、其他板面组合的实拍验证另行进行。
