# -*- coding: utf-8 -*-
"""识别板二模板匹配核心算法。

识别板二展示化验区状态，共 7 种：
  1. "化验区空闲中，请快速通过"       → 模板 idle.png  → WAIT-0
  2. "化验区忙碌中，需等待 n 秒"       → 模板 wait5.png ~ wait10.png → WAIT-5..10

本模块是纯算法实现，不依赖 ROS：
- 加载模板目录下的 idle.png / wait5.png..wait10.png。
- 对输入帧做多尺度模板匹配（TM_CCOEFF_NORMED）。
- 返回最佳匹配的等待秒数和匹配分数。

ROS 节点 board2_detector.py 负责视频读取、去抖锁定和话题发布。
"""

import os
import re
import glob

import cv2
import numpy as np

from pharmacy_mplus0.models import Board2Status, board2_status_from_text


# ---- 默认参数（与 vision.yaml board2 段保持一致）----------------------

_DEFAULT_CONFIG = {
    "scales": [0.7, 0.8, 0.9, 1.0, 1.1, 1.2],
    "match_threshold": 0.72,
}


def parse_template_filename(fname):
    """从模板文件名解析等待秒数。

      idle.png    → 0
      wait5.png   → 5
      wait10.jpg  → 10

    无法解析时返回 None。
    """
    name = os.path.splitext(os.path.basename(fname))[0].lower()
    if name == "idle":
        return 0
    m = re.match(r"wait(\d+)", name)
    if m:
        return int(m.group(1))
    return None


class Board2Matcher(object):
    """识别板二模板匹配器。

    用法:
        matcher = Board2Matcher(template_dir, config)
        wait_seconds, score = matcher.match(frame)
        if wait_seconds is not None and score >= matcher.match_threshold:
            # 匹配成功，wait_seconds ∈ {0, 5, 6, 7, 8, 9, 10}
    """

    def __init__(self, template_dir, config=None):
        """初始化匹配器。

        参数:
            template_dir: 模板图片目录的绝对路径。
            config:       可选 dict，覆盖默认匹配参数（scales, match_threshold）。
        """
        cfg = dict(_DEFAULT_CONFIG)
        if config:
            cfg.update(config)

        self._scales = cfg.get("scales", _DEFAULT_CONFIG["scales"])
        self.match_threshold = cfg.get(
            "match_threshold", _DEFAULT_CONFIG["match_threshold"]
        )

        # 加载模板: [(wait_seconds, template_gray), ...]
        self._templates = self._load_templates(template_dir)
        self.template_count = len(self._templates)

    def match(self, frame):
        """对一帧图像做多尺度模板匹配。

        参数:
            frame: BGR 格式的 numpy 数组。

        返回:
            (wait_seconds, best_score)
            - wait_seconds: int ∈ {0, 5, 6, 7, 8, 9, 10} 或 None（匹配失败）。
            - best_score:   float，最高匹配置信度。
        """
        if frame is None or not self._templates:
            return None, 0.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_h, frame_w = gray.shape[:2]

        best_wait = None
        best_score = -1.0

        # 遍历所有模板和缩放比例，取最高匹配分数。
        for wait_sec, tmpl in self._templates:
            th, tw = tmpl.shape[:2]
            for scale in self._scales:
                sw, sh = int(tw * scale), int(th * scale)
                # 缩放过小或超过帧尺寸时跳过。
                if sw < 20 or sh < 20:
                    continue
                if sw > frame_w or sh > frame_h:
                    continue
                try:
                    resized = cv2.resize(tmpl, (sw, sh))
                    result = cv2.matchTemplate(
                        gray, resized, cv2.TM_CCOEFF_NORMED
                    )
                    _, max_val, _, _ = cv2.minMaxLoc(result)
                except cv2.error:
                    continue
                if max_val > best_score:
                    best_score = max_val
                    best_wait = wait_sec

        return best_wait, best_score

    def match_to_status(self, frame):
        """匹配帧并直接返回 Board2Status。

        这是 match() 的便捷封装：如果匹配分数低于阈值，
        返回的 Board2Status 中 wait_seconds=None, is_idle=False。
        调用方可根据 is_idle 或 wait_seconds 做后续判断。
        """
        wait_seconds, score = self.match(frame)
        if wait_seconds is None or score < self.match_threshold:
            return Board2Status(
                wait_seconds=None,
                is_idle=False,
                raw="NONE",
            )
        raw = "WAIT-%d" % wait_seconds
        return board2_status_from_text(raw)

    def is_loaded(self):
        """模板是否已成功加载。"""
        return len(self._templates) > 0

    # ---- 模板加载 ----------------------------------------------------

    def _load_templates(self, dir_path):
        """加载模板目录下所有图片，返回按 wait_seconds 排序的列表。

        每项为 (wait_seconds, template_gray_image)。
        """
        templates = []
        if not os.path.isdir(dir_path):
            return templates

        for ext in ("png", "jpg", "jpeg", "bmp"):
            for f in glob.glob(os.path.join(dir_path, "*.%s" % ext)):
                w = parse_template_filename(f)
                if w is None:
                    continue
                img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue
                templates.append((w, img))

        templates.sort(key=lambda item: item[0])
        return templates
