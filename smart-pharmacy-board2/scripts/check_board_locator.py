from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
DATASET_ROOT = PROJECT_ROOT / "dataset"
OUTPUT_ROOT = PROJECT_ROOT / "results" / "board_locator_check"

CLASSES = [
    "free",
    "busy_5",
    "busy_6",
    "busy_7",
    "busy_8",
    "busy_9",
    "busy_10",
]

VALID_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}

# 检测时缩放到该宽度。原图为 640×480 时不会放大。
DETECT_WIDTH = 640

# 透视矫正后的预览尺寸。
WARP_WIDTH = 480
WARP_HEIGHT = 320

# 白色区域的 HSV 初始阈值。
WHITE_MAX_SATURATION = 100
WHITE_MIN_VALUE = 120

# 候选白色区域面积占整幅图像的范围。
MIN_AREA_RATIO = 0.004
MAX_AREA_RATIO = 0.15

# 识别板大致为横向矩形。
MIN_ASPECT_RATIO = 1.10
MAX_ASPECT_RATIO = 2.10

# 候选区域不能过于破碎。
MIN_RECTANGULARITY = 0.60

# 白色内框向外扩张，尽量包含黑色边框。
QUAD_EXPAND_FACTOR = 1.16


@dataclass
class BoardCandidate:
    points: np.ndarray
    score: float
    area_ratio: float
    aspect_ratio: float
    rectangularity: float
    white_ratio: float
    dark_ring_ratio: float


def order_points(points: np.ndarray) -> np.ndarray:
    """将四个角点排序为左上、右上、右下、左下。"""
    points = np.asarray(points, dtype=np.float32)

    point_sum = points.sum(axis=1)
    point_diff = np.diff(points, axis=1).reshape(-1)

    top_left = points[np.argmin(point_sum)]
    bottom_right = points[np.argmax(point_sum)]
    top_right = points[np.argmin(point_diff)]
    bottom_left = points[np.argmax(point_diff)]

    return np.array(
        [top_left, top_right, bottom_right, bottom_left],
        dtype=np.float32,
    )


def expand_quad(
    points: np.ndarray,
    factor: float,
    image_width: int,
    image_height: int,
) -> np.ndarray:
    """以四边形中心为基准向外扩张。"""
    center = points.mean(axis=0)
    expanded = center + (points - center) * factor

    expanded[:, 0] = np.clip(
        expanded[:, 0],
        0,
        image_width - 1,
    )
    expanded[:, 1] = np.clip(
        expanded[:, 1],
        0,
        image_height - 1,
    )

    return expanded.astype(np.float32)


def warp_quad(
    image: np.ndarray,
    points: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    source = order_points(points)

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
        source,
        destination,
    )

    return cv2.warpPerspective(
        image,
        matrix,
        (width, height),
    )


def calculate_side_geometry(
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

    return (
        mean_width,
        mean_height,
        mean_width / mean_height,
    )


def calculate_patch_features(
    image: np.ndarray,
    inner_points: np.ndarray,
) -> tuple[float, float]:
    """
    计算白色内部比例和扩张区域边缘的深色比例。

    white_ratio 高说明内部大部分是白色；
    dark_ring_ratio 高说明白色区域外围存在黑色边框。
    """
    inner_patch = warp_quad(
        image,
        inner_points,
        320,
        220,
    )

    inner_gray = cv2.cvtColor(
        inner_patch,
        cv2.COLOR_BGR2GRAY,
    )

    white_ratio = float(
        np.mean(inner_gray > 145)
    )

    height, width = image.shape[:2]

    expanded_points = expand_quad(
        inner_points,
        QUAD_EXPAND_FACTOR,
        width,
        height,
    )

    expanded_patch = warp_quad(
        image,
        expanded_points,
        320,
        220,
    )

    expanded_gray = cv2.cvtColor(
        expanded_patch,
        cv2.COLOR_BGR2GRAY,
    )

    ring_mask = np.zeros(
        expanded_gray.shape,
        dtype=np.uint8,
    )

    border_x = 30
    border_y = 22

    ring_mask[:, :] = 255
    ring_mask[
        border_y:-border_y,
        border_x:-border_x,
    ] = 0

    ring_pixels = expanded_gray[ring_mask == 255]

    if ring_pixels.size == 0:
        dark_ring_ratio = 0.0
    else:
        dark_ring_ratio = float(
            np.mean(ring_pixels < 105)
        )

    return white_ratio, dark_ring_ratio


def score_candidate(
    area_ratio: float,
    aspect_ratio: float,
    rectangularity: float,
    white_ratio: float,
    dark_ring_ratio: float,
    center_y_ratio: float,
) -> float:
    # 识别板比例约为 1.4～1.6，但允许一定透视误差。
    aspect_score = max(
        0.0,
        1.0 - abs(aspect_ratio - 1.48) / 0.70,
    )

    # 主识别板通常比背景中的小标牌大。
    area_score = min(
        area_ratio / 0.035,
        1.5,
    )

    # 主识别板通常位于画面中下部。
    vertical_score = min(
        max((center_y_ratio - 0.20) / 0.45, 0.0),
        1.0,
    )

    return (
        area_score * 3.0
        + aspect_score * 2.0
        + rectangularity * 1.5
        + white_ratio * 1.5
        + dark_ring_ratio * 2.0
        + vertical_score
    )


def find_board(
    image: np.ndarray,
) -> tuple[BoardCandidate | None, int]:
    original_height, original_width = image.shape[:2]

    scale = min(
        1.0,
        DETECT_WIDTH / original_width,
    )

    detect_width = int(round(original_width * scale))
    detect_height = int(round(original_height * scale))

    if scale < 1.0:
        small = cv2.resize(
            image,
            (detect_width, detect_height),
            interpolation=cv2.INTER_AREA,
        )
    else:
        small = image.copy()

    hsv = cv2.cvtColor(
        small,
        cv2.COLOR_BGR2HSV,
    )

    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]

    white_mask = np.where(
        (saturation <= WHITE_MAX_SATURATION)
        & (value >= WHITE_MIN_VALUE),
        255,
        0,
    ).astype(np.uint8)

    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (3, 3),
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (7, 7),
    )

    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=1,
    )

    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=2,
    )

    contours, _ = cv2.findContours(
        white_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    image_area = float(
        detect_width * detect_height
    )

    candidates: list[BoardCandidate] = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area <= 1:
            continue

        area_ratio = area / image_area

        if not (
            MIN_AREA_RATIO
            <= area_ratio
            <= MAX_AREA_RATIO
        ):
            continue

        perimeter = cv2.arcLength(
            contour,
            True,
        )

        approximate = cv2.approxPolyDP(
            contour,
            0.025 * perimeter,
            True,
        )

        if (
            len(approximate) == 4
            and cv2.isContourConvex(approximate)
        ):
            points = approximate.reshape(4, 2)
        else:
            rectangle = cv2.minAreaRect(contour)
            points = cv2.boxPoints(rectangle)

        points = order_points(points)

        _, mean_height, aspect_ratio = (
            calculate_side_geometry(points)
        )

        if mean_height <= 1:
            continue

        if not (
            MIN_ASPECT_RATIO
            <= aspect_ratio
            <= MAX_ASPECT_RATIO
        ):
            continue

        minimum_rectangle = cv2.minAreaRect(
            contour
        )

        rectangle_area = (
            minimum_rectangle[1][0]
            * minimum_rectangle[1][1]
        )

        if rectangle_area <= 1:
            continue

        rectangularity = min(
            area / rectangle_area,
            1.0,
        )

        if rectangularity < MIN_RECTANGULARITY:
            continue

        original_points = points / scale

        white_ratio, dark_ring_ratio = (
            calculate_patch_features(
                image,
                original_points,
            )
        )

        center_y_ratio = float(
            original_points[:, 1].mean()
            / original_height
        )

        score = score_candidate(
            area_ratio=area_ratio,
            aspect_ratio=aspect_ratio,
            rectangularity=rectangularity,
            white_ratio=white_ratio,
            dark_ring_ratio=dark_ring_ratio,
            center_y_ratio=center_y_ratio,
        )

        candidates.append(
            BoardCandidate(
                points=original_points,
                score=score,
                area_ratio=area_ratio,
                aspect_ratio=aspect_ratio,
                rectangularity=rectangularity,
                white_ratio=white_ratio,
                dark_ring_ratio=dark_ring_ratio,
            )
        )

    if not candidates:
        return None, 0

    candidates.sort(
        key=lambda candidate: candidate.score,
        reverse=True,
    )

    return candidates[0], len(candidates)


def draw_result(
    image: np.ndarray,
    candidate: BoardCandidate | None,
    candidate_count: int,
) -> np.ndarray:
    annotated = image.copy()

    if candidate is None:
        cv2.putText(
            annotated,
            "BOARD NOT FOUND",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    height, width = image.shape[:2]

    expanded_points = expand_quad(
        candidate.points,
        QUAD_EXPAND_FACTOR,
        width,
        height,
    )

    polygon = np.round(
        expanded_points
    ).astype(np.int32)

    cv2.polylines(
        annotated,
        [polygon],
        True,
        (0, 255, 0),
        3,
        cv2.LINE_AA,
    )

    label_lines = [
        f"score={candidate.score:.2f}",
        f"candidates={candidate_count}",
        f"area={candidate.area_ratio:.4f}",
        f"aspect={candidate.aspect_ratio:.3f}",
        f"white={candidate.white_ratio:.3f}",
        f"dark_ring={candidate.dark_ring_ratio:.3f}",
    ]

    for index, text in enumerate(label_lines):
        cv2.putText(
            annotated,
            text,
            (12, 25 + index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return annotated


def get_middle_frame(
    stop_directory: Path,
) -> Path:
    images = sorted(
        path
        for path in stop_directory.iterdir()
        if path.is_file()
        and path.suffix.lower() in VALID_SUFFIXES
    )

    if not images:
        raise RuntimeError(
            f"停车目录没有图片：{stop_directory}"
        )

    return images[len(images) // 2]


def main() -> None:
    annotated_root = OUTPUT_ROOT / "annotated"
    warped_root = OUTPUT_ROOT / "warped"
    failed_root = OUTPUT_ROOT / "failed"

    annotated_root.mkdir(
        parents=True,
        exist_ok=True,
    )
    warped_root.mkdir(
        parents=True,
        exist_ok=True,
    )
    failed_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    report_rows: list[dict[str, object]] = []

    total_count = 0
    success_count = 0

    for class_name in CLASSES:
        class_directory = DATASET_ROOT / class_name

        stop_directories = sorted(
            path
            for path in class_directory.iterdir()
            if path.is_dir()
            and path.name.startswith("stop_")
        )

        for stop_directory in stop_directories:
            total_count += 1

            image_path = get_middle_frame(
                stop_directory
            )

            image = cv2.imread(
                str(image_path),
                cv2.IMREAD_COLOR,
            )

            if image is None or image.size == 0:
                raise RuntimeError(
                    f"无法读取图片：{image_path}"
                )

            candidate, candidate_count = find_board(
                image
            )

            annotated = draw_result(
                image,
                candidate,
                candidate_count,
            )

            relative_name = (
                f"{class_name}_{stop_directory.name}.jpg"
            )

            annotated_path = (
                annotated_root / relative_name
            )

            cv2.imwrite(
                str(annotated_path),
                annotated,
            )

            if candidate is None:
                success = False

                cv2.imwrite(
                    str(failed_root / relative_name),
                    image,
                )

                report_rows.append(
                    {
                        "class": class_name,
                        "stop_id": stop_directory.name,
                        "image": str(
                            image_path.relative_to(
                                DATASET_ROOT
                            )
                        ),
                        "success": False,
                        "candidate_count": candidate_count,
                        "score": "",
                        "area_ratio": "",
                        "aspect_ratio": "",
                        "rectangularity": "",
                        "white_ratio": "",
                        "dark_ring_ratio": "",
                    }
                )

                print(
                    f"[FAIL] {class_name}/"
                    f"{stop_directory.name}"
                )
                continue

            success = True
            success_count += 1

            image_height, image_width = image.shape[:2]

            expanded_points = expand_quad(
                candidate.points,
                QUAD_EXPAND_FACTOR,
                image_width,
                image_height,
            )

            warped = warp_quad(
                image,
                expanded_points,
                WARP_WIDTH,
                WARP_HEIGHT,
            )

            cv2.imwrite(
                str(warped_root / relative_name),
                warped,
            )

            report_rows.append(
                {
                    "class": class_name,
                    "stop_id": stop_directory.name,
                    "image": str(
                        image_path.relative_to(
                            DATASET_ROOT
                        )
                    ),
                    "success": success,
                    "candidate_count": candidate_count,
                    "score": f"{candidate.score:.6f}",
                    "area_ratio": (
                        f"{candidate.area_ratio:.6f}"
                    ),
                    "aspect_ratio": (
                        f"{candidate.aspect_ratio:.6f}"
                    ),
                    "rectangularity": (
                        f"{candidate.rectangularity:.6f}"
                    ),
                    "white_ratio": (
                        f"{candidate.white_ratio:.6f}"
                    ),
                    "dark_ring_ratio": (
                        f"{candidate.dark_ring_ratio:.6f}"
                    ),
                }
            )

            print(
                f"[OK] {class_name}/"
                f"{stop_directory.name} "
                f"score={candidate.score:.2f}"
            )

    report_path = (
        OUTPUT_ROOT / "board_locator_report.csv"
    )

    with report_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as file:
        fieldnames = [
            "class",
            "stop_id",
            "image",
            "success",
            "candidate_count",
            "score",
            "area_ratio",
            "aspect_ratio",
            "rectangularity",
            "white_ratio",
            "dark_ring_ratio",
        ]

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )
        writer.writeheader()
        writer.writerows(report_rows)

    success_rate = (
        success_count / total_count
        if total_count > 0
        else 0.0
    )

    print("\n" + "=" * 50)
    print("识别板定位检查完成")
    print("=" * 50)
    print(f"代表图片数量：{total_count}")
    print(f"定位成功数量：{success_count}")
    print(
        f"初步定位成功率："
        f"{success_rate * 100:.2f}%"
    )
    print(f"报告文件：{report_path}")
    print(f"标注图片：{annotated_root}")
    print(f"矫正图片：{warped_root}")
    print(f"失败图片：{failed_root}")


if __name__ == "__main__":
    main()
