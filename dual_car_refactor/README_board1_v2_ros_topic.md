# board1_v2_ros_topic_input

这版以 v2 的识别体验为基底：

1. 默认使用 ROS Image topic 作为图像输入：`/camera/rgb/image_raw`。
2. HTTP 视频流保留为 fallback。
3. 板一识别仍采用 v2 思路：先找 4 个大窗口框，再逐窗口轻量解码。
4. 不使用 v3 的多 ROI、多倍率、OpenCV QRCodeDetector 重增强默认路径。
5. 无效 all_text（例如 `["", "", "", ""]`）不会进入稳定历史。

替换到：

```bash
~/robot_ws/src/pharmacy_pkg/scripts/
chmod +x F1_detect_code_v5.py
```

如果 ROS 图像话题暂时不可用，会自动使用 `camera_url` 的 HTTP 视频流兜底。

需要切回纯 HTTP 时，在 `dual_car_config.py` 里改：

```python
"image_source": "http"
```

如果想试矫正图像，改：

```python
"camera_topic": "/camera/rgb/image_rect_color"
```
