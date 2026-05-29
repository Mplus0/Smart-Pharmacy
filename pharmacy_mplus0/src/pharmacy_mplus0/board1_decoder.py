# -*- coding: utf-8 -*-
"""识别板一二维码解码核心算法。

实现定位框轮廓筛选 → 透视矫正 → 四区域裁剪 → pyzbar 解码的完整流水线。

策略来源：当前比赛初始策略，参考 pharmacy_pkg/scripts/detect_code_for_EPRobot.py。
仅实现定位框筛选 + 透视矫正 + pyzbar，不包含 ROI 优先或多层融合。

本模块是纯算法实现，不依赖 ROS，方便：
- 用静态图片离线测试。
- 通过构造参数注入参数，替代硬编码常量。
- 后续可替换为其他解码方案而不影响 ROS 节点。
"""

import math
import cv2
import numpy as np
from pyzbar.pyzbar import decode as pyzbar_decode

from pharmacy_mplus0.models import Board1Detection, make_board1_detection


# ---- 默认参数（与 vision.yaml board1 段保持一致）----------------------

_DEFAULT_CONFIG = {
    "rotate_degrees": 5.0,
    "canny_low": 50,
    "canny_high": 150,
    "canny_aperture": 3,
    "contour_min_area": 500,
    "contour_max_area": 20000,
    # 多边形近似系数，越小越贴合原始轮廓。
    "approx_poly_epsilon": 0.02,
    # 正方形长宽比系数，归一化后 (|w-h|/(w+h)) < this 即认为是正方形。
    "square_wh_rate": 0.4,
    # 嵌套定位框中心距离阈值（像素），小于此值视为同一组嵌套方框。
    "center_distance_threshold": 20,
    # 定位框总数满足该数量后开始透视矫正（旧策略写死 48）。
    "locating_box_count": 48,
    # 透视矫正后固定画布尺寸。
    "warp_width": 600,
    "warp_height": 600,
    # 裁剪区域二值化阈值。
    "binary_threshold": 127,
    # 四个二维码区域在矫正画布中的裁剪范围 (y1, y2, x1, x2)。
    "crop_regions": {
        "box_1": {"y1": 65, "y2": 255, "x1": 65, "x2": 235},
        "box_2": {"y1": 65, "y2": 255, "x1": 365, "x2": 535},
        "box_3": {"y1": 335, "y2": 525, "x1": 65, "x2": 235},
        "box_4": {"y1": 335, "y2": 525, "x1": 365, "x2": 535},
    },
}


def _distance(p1, p2):
    """两点欧氏距离。"""
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2
    )


def _merge_close_points(points, threshold):
    """将距离小于 threshold 的点合并为它们的质心。

    参考代码中 filter_points 的逻辑：
    遍历点列表，如果当前点与已有聚类代表点的距离 < 阈值，
    则归入该聚类；否则新建一个聚类。最后每个聚类输出其质心。

    返回合并后的点列表。
    """
    if not points:
        return []
    clusters = []  # 每个元素是 [代表点, 成员列表]
    for pt in points:
        matched = False
        for cluster in clusters:
            rep = cluster[0]
            if _distance(pt, rep) < threshold:
                cluster[1].append(pt)
                # 更新代表点为聚类质心
                xs = [p[0] for p in cluster[1]]
                ys = [p[1] for p in cluster[1]]
                cluster[0] = (
                    int(sum(xs) / len(xs)),
                    int(sum(ys) / len(ys)),
                )
                matched = True
                break
        if not matched:
            clusters.append([pt, [pt]])
    return [c[0] for c in clusters]


class Board1Decoder(object):
    """识别板一初始策略解码器。

    用法:
        decoder = Board1Decoder(config_dict)
        detections = decoder.detect(frame)
        # detections 是 [Board1Detection, ...] 或空列表。
    """

    def __init__(self, config=None):
        """从 dict 读取参数，未提供的键使用默认值。"""
        cfg = dict(_DEFAULT_CONFIG)
        if config:
            cfg.update(config)
        self._cfg = cfg

        # 预计算透视矫正后的裁剪坐标，避免每帧重复构造。
        self._crops = [
            self._cfg["crop_regions"]["box_1"],
            self._cfg["crop_regions"]["box_2"],
            self._cfg["crop_regions"]["box_3"],
            self._cfg["crop_regions"]["box_4"],
        ]
        self._warp_size = (
            self._cfg["warp_width"],
            self._cfg["warp_height"],
        )

    def detect(self, frame):
        """处理一帧图像，返回识别到的二维码结果列表。

        参数:
            frame: BGR 格式的 numpy 数组。

        返回:
            [Board1Detection, ...] — 按方框顺序排列的识别结果。
            未检测到完整识别板时返回空列表。
        """
        if frame is None:
            return []

        # 步骤 0: 旋转校正。
        angle = self._cfg["rotate_degrees"]
        if angle != 0.0:
            frame = self._rotate_frame(frame, angle)

        # 步骤 1: 灰度化 + Canny 边缘检测。
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(
            gray,
            self._cfg["canny_low"],
            self._cfg["canny_high"],
            apertureSize=self._cfg["canny_aperture"],
        )

        # 步骤 2: 查找轮廓，按面积筛选。
        # OpenCV 3.x 返回 (image, contours, hierarchy)，4.x 返回 (contours, hierarchy)。
        found = cv2.findContours(
            edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = found[1] if len(found) == 3 else found[0]
        contours = self._filter_by_area(contours)

        # 步骤 3: 四边形近似筛选。
        quadrilaterals = self._filter_quadrilaterals(contours)

        # 步骤 4: 正方形长宽比筛选。
        squares = self._filter_squares(quadrilaterals)

        # 步骤 5: 嵌套定位框筛选（中心重合 + 距离阈值）。
        jieguo, dingweikuang = self._find_nested_squares(squares)

        # 步骤 6: 定位框数量不足 → 判定未识别到完整板面。
        if len(dingweikuang) != self._cfg["locating_box_count"]:
            return []

        # 步骤 7: 根据定位框极值点计算识别板四角。
        src_pts, dst_pts = self._compute_perspective_corners(jieguo)
        if src_pts is None or dst_pts is None:
            return []

        # 步骤 8: 透视变换 → 600×600 矫正画布。
        src_pts = np.array(src_pts, dtype=np.float32)
        dst_pts = np.array(dst_pts, dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(frame, matrix, self._warp_size)

        # 步骤 9: 四区域裁剪 → 二值化 → pyzbar 解码。
        detections = []
        for box_idx, crop_cfg in enumerate(self._crops):
            region = warped[
                crop_cfg["y1"]:crop_cfg["y2"],
                crop_cfg["x1"]:crop_cfg["x2"],
            ]
            code_text = self._decode_region(region)
            if code_text:
                try:
                    detection = make_board1_detection(
                        code_text, box_idx
                    )
                    detections.append(detection)
                except ValueError:
                    # 二维码内容无法标准化（不是合法 A/B/C 组合），跳过。
                    continue

        return detections

    # ---- 筛选步骤 ----------------------------------------------------

    def _filter_by_area(self, contours):
        """保留面积在 [min_area, max_area] 范围内的轮廓。"""
        min_a = self._cfg["contour_min_area"]
        max_a = self._cfg["contour_max_area"]
        result = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_a <= area <= max_a:
                result.append(cnt)
        return result

    def _filter_quadrilaterals(self, contours):
        """用 approxPolyDP 逼近，保留四边形。"""
        epsilon = self._cfg["approx_poly_epsilon"]
        result = []
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon * peri, True)
            if len(approx) == 4:
                result.append(approx)
        return result

    def _filter_squares(self, quadrilaterals):
        """用最小外接矩形的长宽比筛选近似正方形。

        归一化差值 |w-h|/(w+h) 小于 square_wh_rate 则保留。
        """
        rate = self._cfg["square_wh_rate"]
        result = []
        for hull in quadrilaterals:
            rect = cv2.minAreaRect(hull)
            (_, _), (w, h), _ = rect
            if w + h == 0:
                continue
            if abs(w - h) / (w + h) < rate:
                result.append(hull)
        return result

    def _find_nested_squares(self, squares):
        """在正方形列表中寻找嵌套定位框。

        对任意两个正方形：
        - 如果它们的中心互相包含在对方的轮廓内 → 互为嵌套。
        - 如果它们的中心距离 < center_distance_threshold → 视作同一组。
        满足以上条件的两个正方形，其外轮廓构成一个定位框。

        返回:
            jieguo:      所有外轮廓正方形的列表。
            dingweikuang: 所有定位框（外轮廓 + 内轮廓）的扁平列表。
        """
        dist_th = self._cfg["center_distance_threshold"]
        squares = list(squares)

        # 计算每个正方形的中心。
        centers = []
        for sq in squares:
            M = cv2.moments(sq)
            if M["m00"] == 0:
                centers.append(None)
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append((cx, cy))

        jieguo = []
        dingweikuang = []

        n = len(squares)
        for i in range(n - 1):
            ci = centers[i]
            if ci is None:
                continue
            for j in range(i + 1, n):
                cj = centers[j]
                if cj is None:
                    continue

                # 判断两个正方形中心是否互相包含。
                inside_ij = (
                    cv2.pointPolygonTest(squares[i], cj, False) > 0
                )
                inside_ji = (
                    cv2.pointPolygonTest(squares[j], ci, False) > 0
                )
                if not (inside_ij and inside_ji):
                    continue

                # 中心距离检查。
                if _distance(ci, cj) >= dist_th:
                    continue

                # 二者互为嵌套且中心足够近 → 保留外轮廓（面积更大的）。
                area_i = cv2.contourArea(squares[i])
                area_j = cv2.contourArea(squares[j])
                if area_i >= area_j:
                    outer = squares[i]
                else:
                    outer = squares[j]

                if not any(
                    np.array_equal(outer, existing)
                    for existing in jieguo
                ):
                    jieguo.append(outer)
                dingweikuang.extend([squares[i], squares[j]])

        return jieguo, dingweikuang

    # ---- 透视变换 ----------------------------------------------------

    def _compute_perspective_corners(self, jieguo):
        """从外轮廓集合中提取最小/最大 x/y 点，计算源四角坐标。

        源四角取自定位框外轮廓的极值点：
        - min_x: x 坐标最小的点。
        - max_x: x 坐标最大的点。
        - min_y: y 坐标最小的点。
        - max_y: y 坐标最大的点。

        目标四角根据 min_x 和 max_x 在 jieguo 中的位置关系决定方向，
        最终映射到 600×600 画布上。
        """
        if not jieguo:
            return None, None

        warp_w = self._cfg["warp_width"]
        warp_h = self._cfg["warp_height"]

        # 遍历所有外轮廓的所有顶点，找极值点和它们所属的轮廓索引。
        min_x = float("inf")
        max_x = -float("inf")
        min_y = float("inf")
        max_y = -float("inf")
        min_x_idx = None
        max_x_idx = None
        min_y_idx = None
        max_y_idx = None

        for i, contour in enumerate(jieguo):
            for pt in contour:
                px, py = pt[0][0], pt[0][1]
                if px < min_x:
                    min_x = px
                    min_x_idx = i
                if px > max_x:
                    max_x = px
                    max_x_idx = i
                if py < min_y:
                    min_y = py
                    min_y_idx = i
                if py > max_y:
                    max_y = py
                    max_y_idx = i

        if (
            min_x_idx is None
            or min_y_idx is None
            or max_x_idx is None
            or max_y_idx is None
        ):
            return None, None

        # 源四角：来自 min_x 轮廓的 min_x 点、max_x 轮廓的 max_x 点、
        #         min_y 轮廓的 min_y 点、max_y 轮廓的 max_y 点。
        x0, y0 = int(min_x), self._get_y_for_x(jieguo[min_x_idx], min_x)
        x1, y1 = int(max_x), self._get_y_for_x(jieguo[max_x_idx], max_x)
        x2, y2 = self._get_x_for_y(jieguo[min_y_idx], min_y), int(min_y)
        x3, y3 = self._get_x_for_y(jieguo[max_y_idx], max_y), int(max_y)

        real_box_centers = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

        # 目标四角：根据 min_x 轮廓索引和 max_x 轮廓索引的相对位置决定朝向。
        margin = 50
        if min_x_idx < max_x_idx:
            goal_box_centers = [
                (margin, warp_h - margin),
                (warp_w - margin, margin),
                (margin, margin),
                (warp_w - margin, warp_h - margin),
            ]
        else:
            goal_box_centers = [
                (margin, margin),
                (warp_w - margin, warp_h - margin),
                (warp_w - margin, margin),
                (margin, warp_h - margin),
            ]

        return real_box_centers, goal_box_centers

    @staticmethod
    def _get_y_for_x(contour, target_x):
        """在轮廓顶点中搜索 x 坐标最接近 target_x 的点，返回其 y 值。"""
        best_y = 0
        best_dist = float("inf")
        for pt in contour:
            dist = abs(pt[0][0] - target_x)
            if dist < best_dist:
                best_dist = dist
                best_y = pt[0][1]
        return int(best_y)

    @staticmethod
    def _get_x_for_y(contour, target_y):
        """在轮廓顶点中搜索 y 坐标最接近 target_y 的点，返回其 x 值。"""
        best_x = 0
        best_dist = float("inf")
        for pt in contour:
            dist = abs(pt[0][1] - target_y)
            if dist < best_dist:
                best_dist = dist
                best_x = pt[0][0]
        return int(best_x)

    # ---- 解码 --------------------------------------------------------

    def _decode_region(self, region):
        """对单个裁剪区域灰度化 → 二值化 → pyzbar 解码。

        返回解码文本字符串，失败时返回 None。
        """
        if region.size == 0:
            return None
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(
            gray,
            self._cfg["binary_threshold"],
            255,
            cv2.THRESH_BINARY,
        )
        decoded = pyzbar_decode(binary)
        if decoded:
            return decoded[0].data.decode("utf-8")
        return None

    # ---- 工具 --------------------------------------------------------

    @staticmethod
    def _rotate_frame(frame, angle_deg):
        """绕图像中心旋转指定角度。"""
        h, w = frame.shape[:2]
        center = (w // 2, h // 2)
        matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
        return cv2.warpAffine(frame, matrix, (w, h))
