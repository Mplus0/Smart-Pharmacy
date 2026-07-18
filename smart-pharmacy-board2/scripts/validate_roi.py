from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
RECTIFIED_ROOT = PROJECT_ROOT / "results" / "rectified_dataset" / "images"
CONFIG_PATH = PROJECT_ROOT / "config" / "board2_roi.yaml"
OUTPUT_ROOT = PROJECT_ROOT / "results" / "roi_validation"

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

GRID_COLUMNS = 4
GRID_ROWS = 3

ANNOTATED_TILE_SIZE = (480, 320)
STATUS_TILE_SIZE = (200, 100)
NUMBER_TILE_SIZE = (160, 100)


def load_roi_config() -> tuple[
    tuple[int, int, int, int],
    tuple[int, int, int, int],
]:
    if not CONFIG_PATH.is_file():
        raise FileNotFoundError(
            f"找不到 ROI 配置文件：{CONFIG_PATH}"
        )

    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    try:
        status_roi = tuple(
            int(value)
            for value in config["roi"]["status"]["pixel_xywh"]
        )

        number_roi = tuple(
            int(value)
            for value in config["roi"]["number"]["pixel_xywh"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "board2_roi.yaml 格式错误，"
            "缺少 roi.status.pixel_xywh "
            "或 roi.number.pixel_xywh"
        ) from error

    if len(status_roi) != 4 or len(number_roi) != 4:
        raise ValueError("ROI 必须包含 x、y、width、height")

    return status_roi, number_roi


def get_images(directory: Path) -> list[Path]:
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file()
        and path.suffix.lower() in VALID_SUFFIXES
    )


def validate_roi_bounds(
    roi: tuple[int, int, int, int],
    image_width: int,
    image_height: int,
    name: str,
) -> None:
    x, y, width, height = roi

    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise ValueError(f"{name} 坐标无效：{roi}")

    if x + width > image_width:
        raise ValueError(
            f"{name} 超出右边界：{roi}，"
            f"图像宽度为 {image_width}"
        )

    if y + height > image_height:
        raise ValueError(
            f"{name} 超出下边界：{roi}，"
            f"图像高度为 {image_height}"
        )


def crop_roi(
    image: np.ndarray,
    roi: tuple[int, int, int, int],
) -> np.ndarray:
    x, y, width, height = roi
    return image[y:y + height, x:x + width]


def draw_roi(
    image: np.ndarray,
    roi: tuple[int, int, int, int],
    label: str,
    color: tuple[int, int, int],
) -> None:
    x, y, width, height = roi

    cv2.rectangle(
        image,
        (x, y),
        (x + width, y + height),
        color,
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        image,
        label,
        (x, max(y - 7, 18)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
        cv2.LINE_AA,
    )


def add_label(
    image: np.ndarray,
    label: str,
) -> np.ndarray:
    output = image.copy()

    overlay = output.copy()
    cv2.rectangle(
        overlay,
        (0, 0),
        (output.shape[1], 30),
        (0, 0, 0),
        -1,
    )

    output = cv2.addWeighted(
        overlay,
        0.55,
        output,
        0.45,
        0,
    )

    cv2.putText(
        output,
        label,
        (6, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    return output


def make_grid(
    tiles: list[np.ndarray],
    columns: int,
    rows: int,
) -> np.ndarray:
    expected_count = columns * rows

    if len(tiles) != expected_count:
        raise ValueError(
            f"网格需要 {expected_count} 张图片，"
            f"实际为 {len(tiles)} 张"
        )

    grid_rows = []

    for row_index in range(rows):
        start = row_index * columns
        end = start + columns
        grid_rows.append(np.hstack(tiles[start:end]))

    return np.vstack(grid_rows)


def save_class_overviews(
    class_name: str,
    status_roi: tuple[int, int, int, int],
    number_roi: tuple[int, int, int, int],
) -> None:
    class_directory = RECTIFIED_ROOT / class_name

    if not class_directory.is_dir():
        raise FileNotFoundError(
            f"缺少矫正图片类别目录：{class_directory}"
        )

    stop_directories = sorted(
        path
        for path in class_directory.iterdir()
        if path.is_dir() and path.name.startswith("stop_")
    )

    if len(stop_directories) != 12:
        raise RuntimeError(
            f"{class_name} 有 {len(stop_directories)} 个停车批次，"
            "预期为 12"
        )

    annotated_tiles: list[np.ndarray] = []
    status_tiles: list[np.ndarray] = []
    number_tiles: list[np.ndarray] = []

    for stop_directory in stop_directories:
        images = get_images(stop_directory)

        if len(images) != 5:
            raise RuntimeError(
                f"{class_name}/{stop_directory.name} "
                f"有 {len(images)} 张图片，预期为 5"
            )

        # 每次停车选择第 3 张作为代表图
        image_path = images[len(images) // 2]

        image = cv2.imread(
            str(image_path),
            cv2.IMREAD_COLOR,
        )

        if image is None or image.size == 0:
            raise RuntimeError(f"无法读取：{image_path}")

        image_height, image_width = image.shape[:2]

        validate_roi_bounds(
            status_roi,
            image_width,
            image_height,
            "status_roi",
        )

        validate_roi_bounds(
            number_roi,
            image_width,
            image_height,
            "number_roi",
        )

        annotated = image.copy()

        draw_roi(
            annotated,
            status_roi,
            "status",
            (0, 255, 0),
        )

        draw_roi(
            annotated,
            number_roi,
            "number",
            (0, 0, 255),
        )

        annotated = cv2.resize(
            annotated,
            ANNOTATED_TILE_SIZE,
            interpolation=cv2.INTER_AREA,
        )

        annotated = add_label(
            annotated,
            stop_directory.name,
        )

        status_crop = crop_roi(image, status_roi)

        status_crop = cv2.resize(
            status_crop,
            STATUS_TILE_SIZE,
            interpolation=cv2.INTER_CUBIC,
        )

        status_crop = add_label(
            status_crop,
            stop_directory.name,
        )

        number_crop = crop_roi(image, number_roi)

        number_crop = cv2.resize(
            number_crop,
            NUMBER_TILE_SIZE,
            interpolation=cv2.INTER_CUBIC,
        )

        number_crop = add_label(
            number_crop,
            stop_directory.name,
        )

        annotated_tiles.append(annotated)
        status_tiles.append(status_crop)
        number_tiles.append(number_crop)

    annotated_grid = make_grid(
        annotated_tiles,
        GRID_COLUMNS,
        GRID_ROWS,
    )

    status_grid = make_grid(
        status_tiles,
        GRID_COLUMNS,
        GRID_ROWS,
    )

    number_grid = make_grid(
        number_tiles,
        GRID_COLUMNS,
        GRID_ROWS,
    )

    annotated_directory = OUTPUT_ROOT / "annotated"
    status_directory = OUTPUT_ROOT / "status"
    number_directory = OUTPUT_ROOT / "number"

    annotated_directory.mkdir(parents=True, exist_ok=True)
    status_directory.mkdir(parents=True, exist_ok=True)
    number_directory.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(
        str(annotated_directory / f"{class_name}.jpg"),
        annotated_grid,
    )

    cv2.imwrite(
        str(status_directory / f"{class_name}.jpg"),
        status_grid,
    )

    cv2.imwrite(
        str(number_directory / f"{class_name}.jpg"),
        number_grid,
    )


def check_all_images(
    status_roi: tuple[int, int, int, int],
    number_roi: tuple[int, int, int, int],
) -> None:
    total_images = 0
    invalid_status = 0
    invalid_number = 0

    for class_name in CLASSES:
        class_directory = RECTIFIED_ROOT / class_name

        for image_path in sorted(class_directory.rglob("*")):
            if (
                not image_path.is_file()
                or image_path.suffix.lower() not in VALID_SUFFIXES
            ):
                continue

            total_images += 1

            image = cv2.imread(
                str(image_path),
                cv2.IMREAD_COLOR,
            )

            if image is None or image.size == 0:
                raise RuntimeError(f"无法读取：{image_path}")

            image_height, image_width = image.shape[:2]

            validate_roi_bounds(
                status_roi,
                image_width,
                image_height,
                "status_roi",
            )

            validate_roi_bounds(
                number_roi,
                image_width,
                image_height,
                "number_roi",
            )

            status_crop = crop_roi(image, status_roi)
            number_crop = crop_roi(image, number_roi)

            if status_crop.size == 0:
                invalid_status += 1

            if number_crop.size == 0:
                invalid_number += 1

    print(f"检查图片数量：{total_images}")
    print(f"无效状态 ROI：{invalid_status}")
    print(f"无效数字 ROI：{invalid_number}")


def main() -> None:
    status_roi, number_roi = load_roi_config()

    print(f"status_roi：{status_roi}")
    print(f"number_roi：{number_roi}")
    print()

    check_all_images(
        status_roi,
        number_roi,
    )

    for class_name in CLASSES:
        save_class_overviews(
            class_name,
            status_roi,
            number_roi,
        )

        print(f"[OK] {class_name}")

    print()
    print("ROI 验证图生成完成")
    print(f"输出目录：{OUTPUT_ROOT}")


if __name__ == "__main__":
    main()
