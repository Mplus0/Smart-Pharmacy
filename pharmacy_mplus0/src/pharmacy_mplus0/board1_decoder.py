# -*- coding: utf-8 -*-
"""识别板一二维码解码核心算法。

新方案（当前）：直接 pyzbar 整帧解码 → 方框位置推断 → 稳定性过滤。
不依赖 Canny / 轮廓 / 透视矫正，参数少，现场调参简单。

旧方案（参考）：定位框轮廓筛选 → 透视矫正 → 四区域裁剪 → pyzbar 解码。
相关辅助方法保留在文件末尾，供历史参考。
"""

import math
from collections import Counter, deque

import cv2
import numpy as np
from pyzbar.pyzbar import ZBarSymbol, decode as pyzbar_decode

from pharmacy_mplus0.models import (
    Board1Detection,
    make_board1_detection,
    normalize_code,
)


# ---- 符号名 → ZBarSymbol 映射 -----------------------------------------

_SYMBOL_NAME_MAP = {
    "QRCODE": ZBarSymbol.QRCODE,
    "CODE128": ZBarSymbol.CODE128,
    "CODE39": ZBarSymbol.CODE39,
    "EAN13": ZBarSymbol.EAN13,
    "EAN8": ZBarSymbol.EAN8,
    "I25": ZBarSymbol.I25,
    "CODABAR": ZBarSymbol.CODABAR,
    "CODE93": ZBarSymbol.CODE93,
    "UPCA": ZBarSymbol.UPCA,
    "UPCE": ZBarSymbol.UPCE,
    "ISBN10": ZBarSymbol.ISBN10,
    "ISBN13": ZBarSymbol.ISBN13,
    "COMPOSITE": ZBarSymbol.COMPOSITE,
}


# ---- 默认参数 ---------------------------------------------------------

_DEFAULT_CONFIG = {
    # === 新方案参数 ===
    "rotate_degrees": 0.0,
    "stable_frames": 3,
    "pyzbar_symbols": ["QRCODE"],

    # === 以下为旧方案（Canny/轮廓/透视）参数，已弃用，保留供参考 ===
    "canny_low": 50,
    "canny_high": 150,
    "canny_aperture": 3,
    "contour_min_area": 500,
    "contour_max_area": 20000,
    "approx_poly_epsilon": 0.02,
    "square_wh_rate": 0.4,
    "center_distance_threshold": 20,
    "locating_box_count": 48,
    "min_outer_boxes": 8,
    "warp_width": 600,
    "warp_height": 600,
    "binary_threshold": 127,
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

    旧方案辅助函数，新方案未使用。
    """
    if not points:
        return []
    clusters = []
    for pt in points:
        matched = False
        for cluster in clusters:
            rep = cluster[0]
            if _distance(pt, rep) < threshold:
                cluster[1].append(pt)
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
    """识别板一解码器（直接 pyzbar 方案）。

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

        # 解析 pyzbar 符号过滤。
        symbol_names = self._cfg.get("pyzbar_symbols", ["QRCODE"])
        self._qr_symbols = [
            _SYMBOL_NAME_MAP.get(name.upper(), ZBarSymbol.QRCODE)
            for name in symbol_names
        ]

        # 稳定性过滤滑动窗口：{box_index: deque(maxlen=stable_frames)}
        self._history = {}
        self._stable_frames = int(self._cfg.get("stable_frames", 3))

    # ---- 新方案 detect -------------------------------------------------

    def detect(self, frame):
        """处理一帧图像，返回识别到的二维码结果列表。

        流水线:
            1. 旋转校正
            2. pyzbar 整帧解码（仅 QR 码）
            3. 筛选有效 A/B/C 内容，提取位置
            4. 中位数网格分栏 → 方框编号
            5. 滑动窗口多数投票 → 稳定性过滤
            6. 返回 [Board1Detection, ...]
        """
        if frame is None:
            return []

        # 步骤 1: 旋转校正。
        angle = self._cfg["rotate_degrees"]
        if angle != 0.0:
            frame = self._rotate_frame(frame, angle)

        # 步骤 2: pyzbar 整帧解码，仅扫描 QR 码。
        decoded = pyzbar_decode(frame, symbols=self._qr_symbols)

        # 步骤 3: 筛选有效 A/B/C 码，提取中心坐标。
        raw_results = []
        for d in decoded:
            try:
                text = d.data.decode("utf-8")
                code = normalize_code(text)
            except (ValueError, UnicodeDecodeError):
                continue
            cx = d.rect.left + d.rect.width / 2.0
            cy = d.rect.top + d.rect.height / 2.0
            raw_results.append((code, cx, cy))

        if not raw_results:
            self._history.clear()
            return []

        # 步骤 4: 中位数网格分栏 → 方框编号。
        box_assignments = self._assign_boxes(raw_results)

        # 步骤 5: 滑动窗口多数投票。
        detections = self._apply_stability(box_assignments)

        return detections

    # ---- 新方案辅助方法 -------------------------------------------------

    @staticmethod
    def _assign_boxes(raw_results):
        """按二维码位置推断方框编号 0-3。

        x 坐标中位数分左右列，y 坐标中位数分上下行:
            左上 = box_0, 右上 = box_1
            左下 = box_2, 右下 = box_3

        同一方框有多个 QR 时只保留第一个。
        """
        if not raw_results:
            return {}

        xs = [r[1] for r in raw_results]
        ys = [r[2] for r in raw_results]

        sorted_xs = sorted(xs)
        sorted_ys = sorted(ys)
        med_x = sorted_xs[len(sorted_xs) // 2]
        med_y = sorted_ys[len(sorted_ys) // 2]

        assignments = {}
        for code, cx, cy in raw_results:
            col = 0 if cx < med_x else 1
            row = 0 if cy < med_y else 1
            box_idx = row * 2 + col
            if box_idx not in assignments:
                assignments[box_idx] = code
        return assignments

    def _apply_stability(self, box_assignments):
        """滑动窗口多数投票。

        - 检测到的方框：追加编码到历史 deque。
        - 未检测到的方框：清除历史（QR 已消失）。
        - 窗口满且多数派（> stable_frames/2）的方框才输出为 Board1Detection。
        """
        # 更新检测到的方框。
        for box_idx, code in box_assignments.items():
            if box_idx not in self._history:
                self._history[box_idx] = deque(maxlen=self._stable_frames)
            self._history[box_idx].append(code)

        # 清除本帧未检测到的方框。
        missing = set(self._history.keys()) - set(box_assignments.keys())
        for box_idx in missing:
            del self._history[box_idx]

        # 输出满足稳定性条件的方框。
        detections = []
        threshold = self._stable_frames / 2.0
        for box_idx, hist in list(self._history.items()):
            if len(hist) < self._stable_frames:
                continue
            counts = Counter(hist)
            most_common_code, count = counts.most_common(1)[0]
            if count > threshold:
                try:
                    d = make_board1_detection(most_common_code, box_idx)
                    detections.append(d)
                except ValueError:
                    continue

        return detections

    # ---- 工具 --------------------------------------------------------

    @staticmethod
    def _rotate_frame(frame, angle_deg):
        """绕图像中心旋转指定角度。"""
        h, w = frame.shape[:2]
        center = (w // 2, h // 2)
        matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
        return cv2.warpAffine(frame, matrix, (w, h))

    # ================================================================
    # 以下为旧方案（Canny/轮廓/定位框/透视矫正/pyzbar）辅助方法。
    # 新方案 detect 不再调用，保留供历史参考。
    # ================================================================

    def _legacy_detect(self, frame):
        """旧方案 detect（保留供参考，新方案不调用）。"""
        if frame is None:
            return []

        angle = self._cfg["rotate_degrees"]
        if angle != 0.0:
            frame = self._rotate_frame(frame, angle)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(
            gray,
            self._cfg["canny_low"],
            self._cfg["canny_high"],
            apertureSize=self._cfg["canny_aperture"],
        )

        found = cv2.findContours(
            edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = found[1] if len(found) == 3 else found[0]
        contours = self._filter_by_area(contours)

        quadrilaterals = self._filter_quadrilaterals(contours)
        squares = self._filter_squares(quadrilaterals)
        jieguo, dingweikuang = self._find_nested_squares(squares)

        min_outer = self._cfg.get("min_outer_boxes", 8)
        if len(jieguo) < min_outer:
            return []

        src_pts, dst_pts = self._compute_perspective_corners(jieguo)
        if src_pts is None or dst_pts is None:
            return []

        src_pts = np.array(src_pts, dtype=np.float32)
        dst_pts = np.array(dst_pts, dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(
            frame, matrix,
            (self._cfg["warp_width"], self._cfg["warp_height"]),
        )

        detections = []
        for box_idx in range(4):
            key = "box_{0}".format(box_idx + 1)
            crop_cfg = self._cfg["crop_regions"][key]
            region = warped[
                crop_cfg["y1"]:crop_cfg["y2"],
                crop_cfg["x1"]:crop_cfg["x2"],
            ]
            code_text = self._decode_region(region)
            if code_text:
                try:
                    detection = make_board1_detection(code_text, box_idx)
                    detections.append(detection)
                except ValueError:
                    continue

        return detections

    def _filter_by_area(self, contours):
        min_a = self._cfg["contour_min_area"]
        max_a = self._cfg["contour_max_area"]
        result = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_a <= area <= max_a:
                result.append(cnt)
        return result

    def _filter_quadrilaterals(self, contours):
        epsilon = self._cfg["approx_poly_epsilon"]
        result = []
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon * peri, True)
            if len(approx) == 4:
                result.append(approx)
        return result

    def _filter_squares(self, quadrilaterals):
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
        dist_th = self._cfg["center_distance_threshold"]
        squares = list(squares)

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

                inside_ij = (
                    cv2.pointPolygonTest(squares[i], cj, False) > 0
                )
                inside_ji = (
                    cv2.pointPolygonTest(squares[j], ci, False) > 0
                )
                if not (inside_ij and inside_ji):
                    continue

                if _distance(ci, cj) >= dist_th:
                    continue

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

    def _compute_perspective_corners(self, jieguo):
        if not jieguo:
            return None, None

        warp_w = self._cfg["warp_width"]
        warp_h = self._cfg["warp_height"]

        centers = []
        for contour in jieguo:
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append([cx, cy])

        if len(centers) < 4:
            return None, None

        centers = np.array(centers, dtype=np.float32)

        hull = cv2.convexHull(centers.astype(np.int32))

        peri = cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, 0.02 * peri, True)

        if len(approx) != 4:
            rect = cv2.minAreaRect(centers)
            approx = cv2.boxPoints(rect)
            approx = np.int0(approx)

        if len(approx) != 4:
            return None, None

        src_corners = self._order_corners(approx)

        margin = 0
        dst_corners = np.array([
            [margin, margin],
            [warp_w - margin, margin],
            [margin, warp_h - margin],
            [warp_w - margin, warp_h - margin],
        ], dtype=np.float32)

        src_list = [(int(p[0]), int(p[1])) for p in src_corners]
        dst_list = [(int(p[0]), int(p[1])) for p in dst_corners]

        return src_list, dst_list

    @staticmethod
    def _order_corners(pts):
        pts = pts.reshape(4, 2)
        sorted_y = pts[np.argsort(pts[:, 1])]
        top_two = sorted_y[:2]
        bottom_two = sorted_y[2:]
        top_two = top_two[np.argsort(top_two[:, 0])]
        bottom_two = bottom_two[np.argsort(bottom_two[:, 0])]
        return np.array([
            top_two[0],
            top_two[1],
            bottom_two[0],
            bottom_two[1],
        ], dtype=np.float32)

    def _decode_region(self, region):
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


