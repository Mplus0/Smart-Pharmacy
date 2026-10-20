# -*- coding: utf-8 -*-
"""Board-one geometry, QR decoding and task selection; no camera ownership."""
import math
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
from pharmacy_iscc.config import COMMON

BOARD1_CFG = COMMON["board1"]
approxPolyDP_epslion = BOARD1_CFG["approx_poly_dp_epsilon"]
wh_rate = BOARD1_CFG["wh_rate"]
min_area = BOARD1_CFG["min_area"]
max_area = BOARD1_CFG["max_area"]
min_center_distance = BOARD1_CFG["min_center_distance"]
BOARD1_DINGWEIKUANG_MIN = BOARD1_CFG["locator_min"]
BOARD1_DINGWEIKUANG_MAX = BOARD1_CFG["locator_max"]
BOARD1_JIEGUO_MIN = BOARD1_CFG["result_min"]
BOARD1_STABLE_FRAMES = BOARD1_CFG["stable_frames"]

BOARD1_WINDOW_DETECT_ENABLE = BOARD1_CFG.get("window_detect_enable", True)
BOARD1_WINDOW_MIN_AREA = BOARD1_CFG.get("window_min_area", 2500)
BOARD1_WINDOW_MAX_AREA = BOARD1_CFG.get("window_max_area", 80000)
BOARD1_WINDOW_SQUARE_WH_RATE = BOARD1_CFG.get("window_square_wh_rate", 0.45)
BOARD1_WINDOW_DEDUP_DISTANCE = BOARD1_CFG.get("window_dedup_distance", 55)
BOARD1_WINDOW_WARP_SIZE = BOARD1_CFG.get("window_warp_size", 240)
BOARD1_WINDOW_INNER_MARGIN_RATE = BOARD1_CFG.get("window_inner_margin_rate", 0.16)
BOARD1_WINDOW_COMPLETE_FROM_THREE = BOARD1_CFG.get("window_complete_from_three", True)
BOARD1_REQUIRE_FOUR_WINDOWS = BOARD1_CFG.get("require_four_windows", True)
BOARD1_WINDOW_GRID_MAX_SCORE = float(BOARD1_CFG.get("window_grid_max_normalized_score", 0.75))
BOARD1_DIRECT_DECODE_ENABLE = BOARD1_CFG.get("direct_decode_enable", True)
BOARD1_DIRECT_MIN_QR_COUNT = BOARD1_CFG.get("direct_min_qr_count", 4)
BOARD1_DIRECT_DEDUP_DISTANCE = BOARD1_CFG.get("direct_dedup_distance", 45)
BOARD1_VALID_TEXT = set(["A", "B", "C", "AB", "AC", "BC", "ABC"])

VALID_COMBO_COUNT = {
    "": 0,
    "A": 1,
    "B": 1,
    "C": 1,
    "AB": 2,
    "AC": 2,
    "BC": 2,
    "ABC": 3,
}

NORMAL_TEXT = set(VALID_COMBO_COUNT.keys())


def safe_to_str(item):
    """兼容 Python2/3，把二维码结果统一转成普通字符串。"""
    if item is None or item == []:
        return ""

    try:
        if isinstance(item, unicode):
            return item.encode("utf-8").strip()
    except NameError:
        pass

    if isinstance(item, str):
        return item.strip()

    return str(item).strip()


def normalize_all_text(all_data):
    return [safe_to_str(item) for item in all_data]


def detect_error_window(all_text):
    for idx, item in enumerate(all_text):
        if item not in NORMAL_TEXT:
            return idx + 1
    return 0


def build_msg_from_selected_text(selected_text, selected_index, error_count):
    if selected_text == "ABC":
        return [1, 1, 1, 3, selected_index, error_count]
    if selected_text == "AB":
        return [0, 1, 1, 2, selected_index, error_count]
    if selected_text == "AC":
        return [1, 1, 0, 2, selected_index, error_count]
    if selected_text == "BC":
        return [1, 0, 1, 2, selected_index, error_count]
    if selected_text == "A":
        return [0, 1, 0, 1, selected_index, error_count]
    if selected_text == "B":
        return [0, 0, 1, 1, selected_index, error_count]
    if selected_text == "C":
        return [1, 0, 0, 1, selected_index, error_count]
    return None


def select_board1_from_all_text(all_data):
    """
    返回 dict:
      all_text, selected_index, selected_text, selected_msg
    无有效二维码时返回 None。
    """
    if not isinstance(all_data, (list, tuple)):
        return None

    all_text = normalize_all_text(all_data)
    if len(all_text) == 0:
        return None

    scores = [VALID_COMBO_COUNT.get(item, 0) for item in all_text]
    max_score = max(scores)
    if max_score == 0:
        return None

    selected_index = scores.index(max_score)
    selected_text = all_text[selected_index]
    error_count = detect_error_window(all_text)
    selected_msg = build_msg_from_selected_text(
        selected_text,
        selected_index,
        error_count
    )

    if selected_msg is None:
        return None

    return {
        "all_text": all_text,
        "selected_index": selected_index,
        "selected_text": selected_text,
        "selected_msg": selected_msg,
    }


def build_board1_task_queue(all_data):
    """按原选择优先级构造整板任务，保留格子索引作为送样窗口。"""
    if not isinstance(all_data, (list, tuple)):
        return []
    remaining = normalize_all_text(all_data)
    tasks = []
    while True:
        selection = select_board1_from_all_text(remaining)
        if selection is None:
            return tasks
        tasks.append(list(selection["selected_msg"]))
        # 只清空已选格，不删除元素，避免剩余任务窗口编号变化。
        remaining[selection["selected_index"]] = ""


def distance(point1, point2):
    """计算两个点之间的欧氏距离。"""
    return math.sqrt((point1[0] - point2[0]) ** 2 +
                     (point1[1] - point2[1]) ** 2)


def find_contours(binary_img, mode, method):
    """兼容不同 OpenCV 版本的 findContours 返回值。"""
    result = cv2.findContours(binary_img, mode, method)
    if len(result) == 3:
        _, contours, _ = result
    else:
        contours, _ = result
    return contours


def filter_contours_by_area(contours):
    """筛选面积在指定范围内的轮廓。"""
    filtered = []
    for contour in contours:
        moment = cv2.moments(contour)
        if moment['m00'] < min_area or moment['m00'] > max_area:
            continue
        filtered.append(contour)
    return filtered


def filter_quadrangles(contours):
    """筛选四边形轮廓。"""
    quadrangles = []
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, approxPolyDP_epslion * peri, True)
        if len(approx) == 4:
            quadrangles.append(approx)
    return quadrangles


def filter_square_like_contours(quadrangles):
    """根据长宽比筛选近似正方形。"""
    squares = []
    for hull in quadrangles:
        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect
        if (w + h) > 0 and abs(w - h) / (w + h) < wh_rate:
            squares.append(hull)
    return squares


def find_locator_squares(squares):
    """根据正方形包含关系筛选二维码定位框。"""
    dingweikuang = []
    jieguo = []

    for i in range(len(squares) - 1):
        M1 = cv2.moments(squares[i])
        if M1['m00'] == 0:
            continue

        cx1 = int(M1['m10'] / M1['m00'])
        cy1 = int(M1['m01'] / M1['m00'])

        for j in range(i + 1, len(squares)):
            M2 = cv2.moments(squares[j])
            if M2['m00'] == 0:
                continue

            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])

            res1 = cv2.pointPolygonTest(squares[i], (cx2, cy2), False)
            res2 = cv2.pointPolygonTest(squares[j], (cx1, cy1), False)

            if res1 > 0 and res2 > 0:
                distance_ = distance((cx1, cy1), (cx2, cy2))
                if distance_ < min_center_distance:
                    dingweikuang += [squares[i], squares[j]]
                    jieguo.append(squares[i])

    return dingweikuang, jieguo


def get_board1_perspective(frame, jieguo):
    """根据定位框计算透视变换，返回 600x600 的二维码板图像。"""
    min_x = 1000
    max_x = 0
    min_y = 1000
    max_y = 0

    for i in range(len(jieguo)):
        for j in range(len(jieguo[i])):
            if jieguo[i][j][0][0] < min_x:
                min_x = jieguo[i][j][0][0]
                min_x_index = [i, j]
            if jieguo[i][j][0][0] > max_x:
                max_x = jieguo[i][j][0][0]
                max_x_index = [i, j]
            if jieguo[i][j][0][1] < min_y:
                min_y = jieguo[i][j][0][1]
                min_y_index = [i, j]
            if jieguo[i][j][0][1] > max_y:
                max_y = jieguo[i][j][0][1]
                max_y_index = [i, j]

    x0 = jieguo[min_x_index[0]][min_x_index[1]][0][0]
    y0 = jieguo[min_x_index[0]][min_x_index[1]][0][1]
    x1 = jieguo[max_x_index[0]][max_x_index[1]][0][0]
    y1 = jieguo[max_x_index[0]][max_x_index[1]][0][1]
    x2 = jieguo[min_y_index[0]][min_y_index[1]][0][0]
    y2 = jieguo[min_y_index[0]][min_y_index[1]][0][1]
    x3 = jieguo[max_y_index[0]][max_y_index[1]][0][0]
    y3 = jieguo[max_y_index[0]][max_y_index[1]][0][1]

    real_box_centers = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

    if min_x_index[0] < max_x_index[0]:
        goal_box_centers = [(50, 550), (550, 50), (50, 50), (550, 550)]
    else:
        goal_box_centers = [(50, 50), (550, 550), (550, 50), (50, 550)]

    dst_pts = np.array(goal_box_centers, dtype=np.float32)
    src_pts = np.array(real_box_centers, dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

    return cv2.warpPerspective(frame, matrix, (600, 600))


def decode_qr_text_light(img):
    """轻量二维码解码：保留 v2 体验，不走 v3 的多ROI/多倍率重增强。"""
    if img is None or img.size == 0:
        return ""

    candidates = []
    candidates.append(img)

    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img.copy()
    except Exception:
        return ""

    candidates.append(gray)

    try:
        _, binary_fixed = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        candidates.append(binary_fixed)
    except Exception:
        pass

    try:
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(binary_otsu)
    except Exception:
        pass

    # 轻量放大一版即可，避免 v3 那种多倍、多ROI导致速度和稳定性下降。
    try:
        h, w = gray.shape[:2]
        if h > 0 and w > 0:
            candidates.append(cv2.resize(gray, (w * 2, h * 2), interpolation=cv2.INTER_LINEAR))
    except Exception:
        pass

    for cand in candidates:
        try:
            code_result = decode(cand)
        except Exception:
            continue

        for qr in code_result:
            try:
                data = qr.data.decode("utf-8")
            except Exception:
                data = qr.data

            data = safe_to_str(data)
            # Preserve decoded interference payloads for detect_error_window.
            if data:
                return data

    return ""


def decode_four_qr_areas(warped_img):
    """裁剪四个二维码区域并解码。旧透视兜底也使用轻量增强解码。"""
    aera1_crop = warped_img[65:255, 65:235]
    aera2_crop = warped_img[65:255, 365:535]
    aera3_crop = warped_img[335:525, 65:235]
    aera4_crop = warped_img[335:525, 365:535]

    crops = [aera1_crop, aera2_crop, aera3_crop, aera4_crop]
    decoded_data = []

    for crop in crops:
        decoded_data.append(decode_qr_text_light(crop))

    return decoded_data


def contour_center(contour):
    moment = cv2.moments(contour)
    if moment['m00'] == 0:
        return None
    return (float(moment['m10']) / moment['m00'], float(moment['m01']) / moment['m00'])


def order_quad_points(points):
    """将四边形点排序为 tl, tr, br, bl。"""
    pts = np.array(points, dtype=np.float32).reshape(4, 2)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(4)

    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(s)]
    ordered[2] = pts[np.argmax(s)]
    ordered[1] = pts[np.argmin(diff)]
    ordered[3] = pts[np.argmax(diff)]
    return ordered


def window_grid_order_score(items):
    """评估4个候选窗口是否像一个2x2网格，分数越小越好。"""
    if len(items) != 4:
        return 1e9

    ordered = sorted(items, key=lambda item: item["center"][1])
    top = sorted(ordered[:2], key=lambda item: item["center"][0])
    bottom = sorted(ordered[2:], key=lambda item: item["center"][0])
    grid = top + bottom

    tl, tr, bl, br = [np.array(item["center"], dtype=np.float32) for item in grid]
    top_vec = tr - tl
    bottom_vec = br - bl
    left_vec = bl - tl
    right_vec = br - tr

    row_parallel = np.linalg.norm(top_vec - bottom_vec)
    col_parallel = np.linalg.norm(left_vec - right_vec)
    row_y_gap = abs(tl[1] - tr[1]) + abs(bl[1] - br[1])
    col_x_gap = abs(tl[0] - bl[0]) + abs(tr[0] - br[0])

    areas = [item["area"] for item in grid]
    area_mean = max(sum(areas) / float(len(areas)), 1.0)
    area_var = sum([abs(a - area_mean) for a in areas]) / area_mean

    return row_parallel + col_parallel + 0.6 * row_y_gap + 0.6 * col_x_gap + 20.0 * area_var


def order_window_items(items):
    """按左上、右上、左下、右下排序窗口。"""
    ordered = sorted(items, key=lambda item: item["center"][1])
    top = sorted(ordered[:2], key=lambda item: item["center"][0])
    bottom = sorted(ordered[2:], key=lambda item: item["center"][0])
    return top + bottom


def dedup_window_candidates(candidates):
    """按中心点距离去重，保留面积较大的候选。"""
    candidates = sorted(candidates, key=lambda item: item["area"], reverse=True)
    unique = []
    for item in candidates:
        duplicate = False
        for kept in unique:
            if distance(item["center"], kept["center"]) < BOARD1_WINDOW_DEDUP_DISTANCE:
                duplicate = True
                break
        if not duplicate:
            unique.append(item)
    return unique


def complete_four_windows_from_three(items, frame_shape):
    """检测到3个窗口时，用平行四边形关系补第4个窗口。"""
    if len(items) != 3 or not BOARD1_WINDOW_COMPLETE_FROM_THREE:
        return None

    h, w = frame_shape[:2]
    best = None
    best_score = 1e9

    centers = [np.array(item["center"], dtype=np.float32) for item in items]
    avg_area = sum([item["area"] for item in items]) / float(len(items))
    avg_side = int(max(20.0, math.sqrt(max(avg_area, 1.0))))

    for anchor_idx in range(3):
        p_anchor = centers[anchor_idx]
        others = [centers[i] for i in range(3) if i != anchor_idx]
        inferred_center = others[0] + others[1] - p_anchor

        cx, cy = float(inferred_center[0]), float(inferred_center[1])
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            continue

        half = avg_side / 2.0
        box = np.array([
            [cx - half, cy - half],
            [cx + half, cy - half],
            [cx + half, cy + half],
            [cx - half, cy + half]
        ], dtype=np.float32)

        inferred = {
            "center": (cx, cy),
            "box": box,
            "area": avg_area,
            "inferred": True,
        }
        four = items + [inferred]
        score = window_grid_order_score(four)
        if score < best_score:
            best_score = score
            best = four

    if best is None:
        return None

    ordered = order_window_items(best)
    centers_log = []
    for item in ordered:
        mark = "*" if item.get("inferred", False) else ""
        centers_log.append("(%.1f,%.1f)%s" % (item["center"][0], item["center"][1], mark))
    rospy.loginfo("识别板一窗口框路径使用3点补全第4窗口: centers=%s", centers_log)
    return ordered


def find_board1_window_boxes(frame):
    """查找识别板一的4个大窗口框。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    candidates = []
    binaries = []

    try:
        edges = cv2.Canny(blur, 40, 140, apertureSize=3)
        binaries.append(edges)
        # Join small gaps in observed borders; do not infer a missing box.
        binaries.append(cv2.morphologyEx(
            edges, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8)))
    except Exception:
        pass

    try:
        _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binaries.append(otsu)
    except Exception:
        pass

    for binary in binaries:
        contours = find_contours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = abs(cv2.contourArea(contour))
            if area < BOARD1_WINDOW_MIN_AREA or area > BOARD1_WINDOW_MAX_AREA:
                continue

            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue

            rect = cv2.minAreaRect(approx)
            (cx, cy), (rw, rh), angle = rect
            if rw <= 1 or rh <= 1:
                continue
            if abs(rw - rh) / (rw + rh) > BOARD1_WINDOW_SQUARE_WH_RATE:
                continue
            # Open/L-shaped background contours are not closed window frames.
            if area / float(rw * rh) < 0.65:
                continue

            box = cv2.boxPoints(rect)
            candidates.append({
                "center": (float(cx), float(cy)),
                "box": np.array(box, dtype=np.float32),
                "area": float(area),
                "inferred": False,
            })

    unique = dedup_window_candidates(candidates)

    selected = None
    if len(unique) >= 4:
        # 候选多时，从面积靠前的若干个中挑最像2x2网格的四个。
        pool = sorted(unique, key=lambda item: item["area"], reverse=True)[:12]
        try:
            import itertools
            best_score = 1e9
            for combo in itertools.combinations(pool, 4):
                combo = list(combo)
                score = window_grid_order_score(combo)
                if score < best_score:
                    best_score = score
                    selected = combo
        except Exception:
            selected = pool[:4]

        selected = order_window_items(selected)

    elif len(unique) == 3:
        selected = complete_four_windows_from_three(unique, frame.shape)

    if selected is None or len(selected) != 4:
        rospy.loginfo_throttle(
            1.0,
            "识别板一窗口框路径未找全: candidates=%d unique=%d need=4",
            len(candidates),
            len(unique)
        )
        return None

    if BOARD1_REQUIRE_FOUR_WINDOWS:
        mean_side = sum(math.sqrt(max(item["area"], 1.0)) for item in selected) / 4.0
        normalized_score = window_grid_order_score(selected) / mean_side
        if normalized_score > BOARD1_WINDOW_GRID_MAX_SCORE:
            rospy.logwarn_throttle(1.0, "板一候选外框不满足2x2排列: score=%.3f", normalized_score)
            return None

    centers_log = ["(%.1f,%.1f)" % (item["center"][0], item["center"][1]) for item in selected]
    rospy.loginfo_throttle(1.0, "识别板一窗口框路径找到4个窗口: centers=%s", centers_log)
    return selected


def warp_window_from_item(frame, item):
    """把单个窗口透视拉正成正方形。"""
    size = int(BOARD1_WINDOW_WARP_SIZE)
    if size <= 0:
        size = 240

    src = order_quad_points(item["box"])
    dst = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, matrix, (size, size))


def crop_window_inner(warped):
    """裁掉窗口黑边，保留内部二维码区域。"""
    h, w = warped.shape[:2]
    margin = int(min(h, w) * BOARD1_WINDOW_INNER_MARGIN_RATE)
    if margin <= 0:
        return warped
    if margin * 2 >= min(h, w) - 5:
        return warped
    return warped[margin:h - margin, margin:w - margin]


def decode_board1_window(warped):
    """Try the configured inner crop, then wider crops if QR decoding fails."""
    text = decode_qr_text_light(crop_window_inner(warped))
    if text:
        return text

    h, w = warped.shape[:2]
    previous_shapes = set([crop_window_inner(warped).shape[:2]])
    for rate in [min(float(BOARD1_WINDOW_INNER_MARGIN_RATE), 0.08), 0.0]:
        margin = max(0, int(min(h, w) * rate))
        if margin * 2 >= min(h, w) - 5:
            continue
        crop = warped[margin:h - margin, margin:w - margin] if margin else warped
        if crop.shape[:2] in previous_shapes:
            continue
        previous_shapes.add(crop.shape[:2])
        text = decode_qr_text_light(crop)
        if text:
            return text
    return ""


def decode_board1_by_windows(frame):
    """主路径：先找4个大窗口，再逐窗口轻量解码。二维码数量允许为0~4。"""
    if not BOARD1_WINDOW_DETECT_ENABLE:
        return None

    windows = find_board1_window_boxes(frame)
    if windows is None:
        return None
    if BOARD1_REQUIRE_FOUR_WINDOWS and any(item.get("inferred", False) for item in windows):
        return None

    all_text = []
    for item in windows:
        try:
            warped = warp_window_from_item(frame, item)
            all_text.append(decode_board1_window(warped))
        except Exception as e:
            rospy.logwarn_throttle(1.0, "识别板一窗口解码异常: %s", str(e))
            all_text.append("")

    rospy.loginfo("识别板一窗口框路径得到 all_text=%s", all_text)
    return all_text


def unique_qr_results(results):
    unique = []
    for text, cx, cy in results:
        duplicate = False
        for _, ux, uy in unique:
            if abs(cx - ux) < BOARD1_DIRECT_DEDUP_DISTANCE and abs(cy - uy) < BOARD1_DIRECT_DEDUP_DISTANCE:
                duplicate = True
                break
        if not duplicate:
            unique.append((text, cx, cy))
    return unique


def decode_board1_direct_four_qr(frame):
    """兜底路径：只有直接扫到4个二维码时，才按几何关系排序。少于4个不猜窗口。"""
    if not BOARD1_DIRECT_DECODE_ENABLE:
        return None

    results = []
    candidates = [frame]
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        candidates.append(gray)
        _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(otsu)
    except Exception:
        pass

    for cand in candidates:
        try:
            decoded = decode(cand)
        except Exception:
            continue
        for qr in decoded:
            try:
                text = qr.data.decode("utf-8")
            except Exception:
                text = qr.data
            text = safe_to_str(text)
            if text not in BOARD1_VALID_TEXT:
                continue
            try:
                if hasattr(qr, "polygon") and qr.polygon:
                    xs = [p.x for p in qr.polygon]
                    ys = [p.y for p in qr.polygon]
                    cx = sum(xs) / float(len(xs))
                    cy = sum(ys) / float(len(ys))
                else:
                    cx = qr.rect.left + qr.rect.width / 2.0
                    cy = qr.rect.top + qr.rect.height / 2.0
            except Exception:
                continue
            results.append((text, cx, cy))

    unique = unique_qr_results(results)
    if len(unique) < BOARD1_DIRECT_MIN_QR_COUNT:
        rospy.loginfo_throttle(
            1.0,
            "识别板一直接二维码路径数量不足: unique_qr=%d/%d",
            len(unique),
            BOARD1_DIRECT_MIN_QR_COUNT
        )
        return None

    unique = sorted(unique, key=lambda x: x[2])[:4]
    top = sorted(unique[:2], key=lambda x: x[1])
    bottom = sorted(unique[2:4], key=lambda x: x[1])
    ordered = top + bottom
    all_text = [item[0] for item in ordered]
    rospy.loginfo("识别板一直接二维码路径得到 all_text=%s", all_text)
    return all_text



def detect_and_publish_board1(frame, stable_publish_board1, board1_history, board1_error_history):
    """执行识别板一识别。顺序：v2窗口框主路径 -> 4二维码直接兜底 -> 旧透视兜底。"""
    # 1. v2主路径：识别4个大窗口框，二维码数量允许为0~4。
    window_all_text = decode_board1_by_windows(frame)
    if window_all_text is not None:
        result = stable_publish_board1(window_all_text)
        if result is not None:
            rospy.logwarn("识别板一通过窗口框路径发布成功")
            return result
        # 如果窗口路径已经读到了有效二维码但还未稳定，本轮不再混入其它路径，避免不同路径裁剪造成 all_text 抖动。
        if select_board1_from_all_text(window_all_text) is not None:
            return None
        # Four real windows were seen. Keep abnormal-only observations stable
        # for their alert, without falling through to the missing-border reset.
        if BOARD1_REQUIRE_FOUR_WINDOWS:
            return None

    # 2. 兜底：只有直接扫到4个二维码时才使用，少于4个不猜窗口。
    # Strict mode requires four observed window borders, including empty slots.
    if BOARD1_REQUIRE_FOUR_WINDOWS:
        board1_history.clear()
        board1_error_history.clear()
        return None

    direct_all_text = decode_board1_direct_four_qr(frame)
    if direct_all_text is not None:
        result = stable_publish_board1(direct_all_text)
        if result is not None:
            rospy.logwarn("识别板一通过直接二维码路径发布成功")
            return result
        if select_board1_from_all_text(direct_all_text) is not None:
            return None

    # 3. 旧透视兜底：保留原二维码定位框逻辑，但失败时不清空历史。
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 50, 150, apertureSize=3)

    contours = find_contours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_by_area = filter_contours_by_area(contours)
    quadrangles = filter_quadrangles(contours_by_area)
    squares = filter_square_like_contours(quadrangles)
    dingweikuang, jieguo = find_locator_squares(squares)

    if (BOARD1_DINGWEIKUANG_MIN <= len(dingweikuang) <= BOARD1_DINGWEIKUANG_MAX and
            len(jieguo) >= BOARD1_JIEGUO_MIN):
        try:
            rospy.loginfo_throttle(
                1.0,
                "识别板一旧透视兜底定位数量满足范围: dingweikuang=%d jieguo=%d",
                len(dingweikuang),
                len(jieguo)
            )

            warped_img = get_board1_perspective(frame, jieguo)
            all_data = decode_four_qr_areas(warped_img)
            return stable_publish_board1(all_data)

        except Exception as e:
            rospy.logwarn("识别板一旧透视兜底处理异常，不清空历史: %s", str(e))
            return None

    rospy.loginfo_throttle(
        1.0,
        "识别板一定位框数量不满足要求，不清空历史: dingweikuang=%d jieguo=%d",
        len(dingweikuang),
        len(jieguo)
    )

    return None


# =========================
# 主循环
# =========================

