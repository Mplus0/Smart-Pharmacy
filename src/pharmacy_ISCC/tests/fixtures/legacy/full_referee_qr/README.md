# 裁判原码全集检查

包含从裁判程序提取的 17 张二维码原图：7 张带字合法任务、7 张纯二维码、3 张干扰码；另有 1 张空白窗口图。原图指纹与真实编码见 `manifest.json`。

- 干扰 1：编码 `ACD`。
- 干扰 2：编码 `D`。
- 干扰 3：中央标注 BD，实际编码 `https://www.hlcode.cn`。它必须作为干扰内容处理，不能按画面文字转换成合法任务。

识别程序正常运行只需要摄像头图像，不需要这些模板参与匹配。本目录用于复测、投屏和审计。

## 小车复测

在现有 ROS Melodic / Python 2 环境执行：

```bash
python /home/EPRobot/robot_ws/src/pharmacy_pkg/tests/full_referee_qr/verify_full_suite.py
```

检查原图解码、每张码分别放在四格中的 68 张合成板面（降至 640×360），以及 14,641 种四格原文组合的选格/错误格逻辑。此脚本不向 ROS 业务话题发布消息。

`verify_live.py` 用于独立相机调试；要求 `/nav_state` 唯一订阅者为 `/board1_debug_detector`，会发送 9→10，再恢复 9。只有停止导航后才能使用。参数为 `manifest.json` 中的场景名，默认三轮。例如：

```bash
python /home/EPRobot/robot_ws/src/pharmacy_pkg/tests/full_referee_qr/verify_live.py invalid_url
```

脚本会保存相机原图并核对正式节点的新序号、三份共享 JSON 和单次 `/cam_return`。全部干扰场景要求在观察期内无任务发布。

## 投屏

在显示器电脑打开 `board1_all_referee_qr.html`，按 F11 全屏，按 1～8 切换：带字合法码两组、纯码两组、ACD 干扰、D 干扰、网址干扰、全部干扰。四格顺序为左上、右上、左下、右下。

`build_full_suite.py` 仅用于本机重新生成板面，使用本机既有 Python/Pillow；小车无需安装新环境，也无需运行生成器。

