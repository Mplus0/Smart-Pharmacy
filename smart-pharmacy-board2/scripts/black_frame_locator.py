from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


DETECT_WIDTH = 640
WARP_WIDTH = 480
WARP_HEIGHT = 320
DEFAULT_DARK_THRESHOLD = 110

MIN_AREA_RATIO = 0.020
MAX_AREA_RATIO = 0.600
MIN_ASPECT_RATIO = 1.10
MAX_ASPECT_RATIO = 2.00
MIN_RECTANGULARITY = 0.65
MIN_CHILD_AREA_RATIO = 0.40
MAX_CHILD_AREA_RATIO = 0.95
MIN_SIDE_CONTINUITY = 0.58
MIN_SIDE_DARK_RATIO = 0.025
MIN_INNER_LIGHT_RATIO = 0.48
MIN_INNER_DARK_RATIO = 0.004
MAX_INNER_DARK_RATIO = 0.38
MIN_INNER_EDGE_DENSITY = 0.008
BLACK_VALIDATION_WIDTH = 420
BLACK_VALIDATION_HEIGHT = 280

COARSE_BOARD_WIDTH = 600
COARSE_BOARD_HEIGHT = 400
WHITE_MAX_SATURATION = 150
WHITE_MIN_VALUE = 85
WHITE_MIN_AREA_RATIO = 0.35
WHITE_MAX_AREA_RATIO = 0.92
WHITE_MIN_ASPECT_RATIO = 1.10
WHITE_MAX_ASPECT_RATIO = 2.05
WHITE_MIN_RECTANGULARITY = 0.58
WHITE_MIN_CENTER_SCORE = 0.35
TRAINING_WHITE_EXPAND_FACTOR = 1.16


@dataclass
class BorderFeatures:
    top_continuity: float
    bottom_continuity: float
    left_continuity: float
    right_continuity: float
    top_dark_ratio: float
    bottom_dark_ratio: float
    left_dark_ratio: float
    right_dark_ratio: float
    inner_light_ratio: float
    inner_dark_ratio: float
    inner_edge_density: float

    @property
    def minimum_continuity(self) -> float:
        return min(
            self.top_continuity,
            self.bottom_continuity,
            self.left_continuity,
            self.right_continuity,
        )

    @property
    def minimum_side_dark_ratio(self) -> float:
        return min(
            self.top_dark_ratio,
            self.bottom_dark_ratio,
            self.left_dark_ratio,
            self.right_dark_ratio,
        )


@dataclass
class BoardCandidate:
    points: np.ndarray
    score: float
    area_ratio: float
    aspect_ratio: float
    rectangularity: float
    child_area_ratio: float
    border_features: BorderFeatures


@dataclass
class AlignmentResult:
    coarse_board: np.ndarray
    aligned_board: np.ndarray
    inner_white_quad: np.ndarray
    white_mask: np.ndarray
    score: float


def order_points(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    point_sum = points.sum(axis=1)
    point_difference = np.diff(points, axis=1).reshape(-1)
    return np.array(
        [
            points[np.argmin(point_sum)],
            points[np.argmin(point_difference)],
            points[np.argmax(point_sum)],
            points[np.argmax(point_difference)],
        ],
        dtype=np.float32,
    )


def expand_quad(
    points: np.ndarray,
    factor: float,
    image_width: int,
    image_height: int,
) -> np.ndarray:
    center = points.mean(axis=0)
    expanded = center + (points - center) * factor
    expanded[:, 0] = np.clip(expanded[:, 0], 0, image_width - 1)
    expanded[:, 1] = np.clip(expanded[:, 1], 0, image_height - 1)
    return expanded.astype(np.float32)


def warp_quad(
    image: np.ndarray,
    points: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    destination = np.array(
        [
            [0, 0],
            [width - 1, 0],
            [width - 1, height - 1],
            [0, height - 1],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(
        order_points(points),
        destination,
    )
    return cv2.warpPerspective(image, matrix, (width, height))


def calculate_quad_geometry(
    points: np.ndarray,
) -> tuple[float, float, float]:
    points = order_points(points)
    width_top = np.linalg.norm(points[1] - points[0])
    width_bottom = np.linalg.norm(points[2] - points[3])
    height_left = np.linalg.norm(points[3] - points[0])
    height_right = np.linalg.norm(points[2] - points[1])
    mean_width = (width_top + width_bottom) / 2.0
    mean_height = (height_left + height_right) / 2.0
    if mean_height <= 1e-6:
        return mean_width, mean_height, 0.0
    return mean_width, mean_height, float(mean_width / mean_height)


def contour_to_quad(contour: np.ndarray) -> np.ndarray:
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    for epsilon_ratio in (
        0.012,
        0.016,
        0.020,
        0.025,
        0.030,
        0.040,
        0.050,
    ):
        approximate = cv2.approxPolyDP(
            hull,
            epsilon_ratio * perimeter,
            True,
        )
        if len(approximate) == 4 and cv2.isContourConvex(approximate):
            return order_points(approximate.reshape(4, 2))
    return order_points(cv2.boxPoints(cv2.minAreaRect(contour)))


def create_dark_mask(
    image: np.ndarray,
    dark_threshold: int,
) -> tuple[np.ndarray, float]:
    original_height, original_width = image.shape[:2]
    scale = min(1.0, DETECT_WIDTH / original_width)
    detect_width = int(round(original_width * scale))
    detect_height = int(round(original_height * scale))
    if scale < 1.0:
        detect_image = cv2.resize(
            image,
            (detect_width, detect_height),
            interpolation=cv2.INTER_AREA,
        )
    else:
        detect_image = image.copy()

    gray = cv2.cvtColor(detect_image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    dark_mask = np.where(
        gray <= dark_threshold,
        255,
        0,
    ).astype(np.uint8)
    dark_mask = cv2.morphologyEx(
        dark_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        iterations=2,
    )
    dark_mask = cv2.morphologyEx(
        dark_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        iterations=1,
    )
    return dark_mask, scale


def get_largest_child_area(
    contours: list[np.ndarray] | tuple[np.ndarray, ...],
    hierarchy: np.ndarray,
    contour_index: int,
) -> float:
    child_index = int(hierarchy[contour_index][2])
    largest_area = 0.0
    while child_index >= 0:
        largest_area = max(
            largest_area,
            abs(cv2.contourArea(contours[child_index])),
        )
        child_index = int(hierarchy[child_index][0])
    return largest_area


def calculate_side_continuity(
    binary_band: np.ndarray,
    horizontal: bool,
) -> float:
    if binary_band.size == 0:
        return 0.0
    positions_with_dark = np.any(
        binary_band,
        axis=0 if horizontal else 1,
    )
    return float(np.mean(positions_with_dark))


def calculate_border_features(
    image: np.ndarray,
    points: np.ndarray,
    dark_threshold: int,
) -> BorderFeatures:
    patch = warp_quad(
        image,
        points,
        BLACK_VALIDATION_WIDTH,
        BLACK_VALIDATION_HEIGHT,
    )
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    dark = gray <= dark_threshold
    height, width = gray.shape
    horizontal_band_height = max(10, int(round(height * 0.16)))
    vertical_band_width = max(10, int(round(width * 0.13)))
    top_band = dark[:horizontal_band_height, :]
    bottom_band = dark[height - horizontal_band_height :, :]
    left_band = dark[:, :vertical_band_width]
    right_band = dark[:, width - vertical_band_width :]
    inner_x1 = int(round(width * 0.18))
    inner_x2 = int(round(width * 0.82))
    inner_y1 = int(round(height * 0.18))
    inner_y2 = int(round(height * 0.82))
    inner_gray = gray[inner_y1:inner_y2, inner_x1:inner_x2]
    inner_edges = cv2.Canny(inner_gray, 45, 130)
    return BorderFeatures(
        top_continuity=calculate_side_continuity(top_band, True),
        bottom_continuity=calculate_side_continuity(bottom_band, True),
        left_continuity=calculate_side_continuity(left_band, False),
        right_continuity=calculate_side_continuity(right_band, False),
        top_dark_ratio=float(np.mean(top_band)),
        bottom_dark_ratio=float(np.mean(bottom_band)),
        left_dark_ratio=float(np.mean(left_band)),
        right_dark_ratio=float(np.mean(right_band)),
        inner_light_ratio=float(np.mean(inner_gray >= 125)),
        inner_dark_ratio=float(np.mean(inner_gray <= dark_threshold)),
        inner_edge_density=float(np.mean(inner_edges > 0)),
    )


def score_black_candidate(
    area_ratio: float,
    aspect_ratio: float,
    rectangularity: float,
    child_area_ratio: float,
    features: BorderFeatures,
) -> float:
    aspect_score = max(
        0.0,
        1.0 - abs(aspect_ratio - 1.48) / 0.70,
    )
    area_score = min(area_ratio / 0.15, 1.2)
    child_score = max(
        0.0,
        1.0 - abs(child_area_ratio - 0.75) / 0.40,
    )
    continuity_score = (
        features.top_continuity
        + features.bottom_continuity
        + features.left_continuity
        + features.right_continuity
    ) / 4.0
    side_dark_score = min(
        (
            features.top_dark_ratio
            + features.bottom_dark_ratio
            + features.left_dark_ratio
            + features.right_dark_ratio
        )
        / 0.50,
        1.2,
    )
    text_score = min(features.inner_edge_density / 0.06, 1.2)
    return (
        area_score * 1.5
        + aspect_score * 2.0
        + rectangularity * 1.2
        + child_score * 1.5
        + continuity_score * 3.0
        + side_dark_score * 1.5
        + features.inner_light_ratio * 1.2
        + text_score * 1.5
    )


def find_black_frame(
    image: np.ndarray,
    dark_threshold: int = DEFAULT_DARK_THRESHOLD,
) -> tuple[BoardCandidate | None, int, np.ndarray]:
    if image is None or image.size == 0:
        raise ValueError("输入图像为空")
    if not 1 <= dark_threshold <= 254:
        raise ValueError("dark-threshold 必须位于 1 到 254 之间")
    dark_mask, scale = create_dark_mask(image, dark_threshold)
    contours, hierarchy = cv2.findContours(
        dark_mask,
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if hierarchy is None:
        return None, 0, dark_mask

    hierarchy = hierarchy[0]
    mask_height, mask_width = dark_mask.shape[:2]
    mask_area = float(mask_width * mask_height)
    candidates: list[BoardCandidate] = []
    for contour_index, contour in enumerate(contours):
        if int(hierarchy[contour_index][3]) >= 0:
            continue
        outer_area = abs(cv2.contourArea(contour))
        if outer_area <= 1.0:
            continue
        area_ratio = outer_area / mask_area
        if not MIN_AREA_RATIO <= area_ratio <= MAX_AREA_RATIO:
            continue

        child_area = get_largest_child_area(
            contours,
            hierarchy,
            contour_index,
        )
        if child_area <= 1.0:
            continue
        child_area_ratio = child_area / outer_area
        if not MIN_CHILD_AREA_RATIO <= child_area_ratio <= MAX_CHILD_AREA_RATIO:
            continue

        quad = contour_to_quad(contour)
        mean_width, mean_height, aspect_ratio = calculate_quad_geometry(quad)
        if mean_width < 70 or mean_height < 45:
            continue
        if not MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO:
            continue
        quad_area = abs(cv2.contourArea(quad.astype(np.float32)))
        if quad_area <= 1.0:
            continue
        rectangularity = min(outer_area / quad_area, 1.0)
        if rectangularity < MIN_RECTANGULARITY:
            continue

        original_quad = quad / scale
        features = calculate_border_features(
            image,
            original_quad,
            dark_threshold,
        )
        if features.minimum_continuity < MIN_SIDE_CONTINUITY:
            continue
        if features.minimum_side_dark_ratio < MIN_SIDE_DARK_RATIO:
            continue
        if features.inner_light_ratio < MIN_INNER_LIGHT_RATIO:
            continue
        if not (
            MIN_INNER_DARK_RATIO
            <= features.inner_dark_ratio
            <= MAX_INNER_DARK_RATIO
        ):
            continue
        if features.inner_edge_density < MIN_INNER_EDGE_DENSITY:
            continue

        candidates.append(
            BoardCandidate(
                points=original_quad,
                score=score_black_candidate(
                    area_ratio,
                    aspect_ratio,
                    rectangularity,
                    child_area_ratio,
                    features,
                ),
                area_ratio=area_ratio,
                aspect_ratio=aspect_ratio,
                rectangularity=rectangularity,
                child_area_ratio=child_area_ratio,
                border_features=features,
            )
        )

    if not candidates:
        return None, 0, dark_mask
    candidates.sort(key=lambda candidate: candidate.score, reverse=True)
    return candidates[0], len(candidates), dark_mask


def create_white_mask(coarse_board: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(coarse_board, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    white_mask = np.where(
        (saturation <= WHITE_MAX_SATURATION)
        & (value >= WHITE_MIN_VALUE),
        255,
        0,
    ).astype(np.uint8)
    height, width = white_mask.shape
    margin_x = int(round(width * 0.035))
    margin_y = int(round(height * 0.035))
    white_mask[:margin_y, :] = 0
    white_mask[height - margin_y :, :] = 0
    white_mask[:, :margin_x] = 0
    white_mask[:, width - margin_x :] = 0
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        iterations=1,
    )
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)),
        iterations=2,
    )
    return white_mask


def find_inner_white_quad(
    coarse_board: np.ndarray,
) -> tuple[np.ndarray | None, float, np.ndarray]:
    white_mask = create_white_mask(coarse_board)
    contours, _ = cv2.findContours(
        white_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    height, width = white_mask.shape
    image_area = float(width * height)
    best_quad: np.ndarray | None = None
    best_score = -1.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= 1.0:
            continue
        area_ratio = area / image_area
        if not WHITE_MIN_AREA_RATIO <= area_ratio <= WHITE_MAX_AREA_RATIO:
            continue
        quad = contour_to_quad(contour)
        _, mean_height, aspect_ratio = calculate_quad_geometry(quad)
        if mean_height <= 1.0:
            continue
        if not WHITE_MIN_ASPECT_RATIO <= aspect_ratio <= WHITE_MAX_ASPECT_RATIO:
            continue
        quad_area = abs(cv2.contourArea(quad.astype(np.float32)))
        if quad_area <= 1.0:
            continue
        rectangularity = min(area / quad_area, 1.0)
        if rectangularity < WHITE_MIN_RECTANGULARITY:
            continue

        center_x_ratio = float(quad[:, 0].mean() / width)
        center_y_ratio = float(quad[:, 1].mean() / height)
        center_score = max(
            0.0,
            1.0
            - abs(center_x_ratio - 0.5) / 0.45
            - abs(center_y_ratio - 0.5) / 0.45,
        )
        if center_score < WHITE_MIN_CENTER_SCORE:
            continue
        aspect_score = max(
            0.0,
            1.0 - abs(aspect_ratio - 1.48) / 0.70,
        )
        score = (
            area_ratio * 2.0
            + rectangularity * 1.5
            + aspect_score * 1.5
            + center_score * 2.0
        )
        if score > best_score:
            best_score = score
            best_quad = quad.astype(np.float32)
    return best_quad, best_score, white_mask


def align_board_to_training_coordinates(
    frame: np.ndarray,
    black_border_quad: np.ndarray,
    warp_width: int = WARP_WIDTH,
    warp_height: int = WARP_HEIGHT,
) -> AlignmentResult | None:
    coarse_board = warp_quad(
        frame,
        black_border_quad,
        COARSE_BOARD_WIDTH,
        COARSE_BOARD_HEIGHT,
    )
    inner_white_quad, score, white_mask = find_inner_white_quad(coarse_board)
    if inner_white_quad is None:
        return None
    expanded_white_quad = expand_quad(
        inner_white_quad,
        TRAINING_WHITE_EXPAND_FACTOR,
        COARSE_BOARD_WIDTH,
        COARSE_BOARD_HEIGHT,
    )
    aligned_board = warp_quad(
        coarse_board,
        expanded_white_quad,
        warp_width,
        warp_height,
    )
    return AlignmentResult(
        coarse_board=coarse_board,
        aligned_board=aligned_board,
        inner_white_quad=inner_white_quad,
        white_mask=white_mask,
        score=score,
    )
