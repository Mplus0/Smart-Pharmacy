from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

VALID_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp"}

TASK_LABELS = {
    "status": ["idle", "busy"],
    "number": ["5", "6", "7", "8", "9", "10"],
}

SPLITS = ["train", "val", "test"]

COLUMNS = 5
ROWS = 5
IMAGES_PER_PAGE = COLUMNS * ROWS

TILE_WIDTH = 220
TILE_HEIGHT = 130
LABEL_HEIGHT = 38


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成分类 ROI 数据集总览图"
    )

    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("results/classification_dataset"),
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/classification_overview"),
    )

    return parser.parse_args()


def resolve_project_path(path: Path) -> Path:
    if path.is_absolute():
        return path.resolve()
    return (PROJECT_ROOT / path).resolve()


def get_images(directory: Path) -> list[Path]:
    if not directory.is_dir():
        return []

    return sorted(
        path
        for path in directory.rglob("*")
        if path.is_file()
        and path.suffix.lower() in VALID_SUFFIXES
    )


def make_tile(
    image_path: Path,
    label_root: Path,
) -> np.ndarray:
    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR,
    )

    if image is None or image.size == 0:
        raise RuntimeError(
            f"无法读取图片：{image_path}"
        )

    source_height, source_width = image.shape[:2]

    available_height = TILE_HEIGHT - LABEL_HEIGHT

    scale = min(
        TILE_WIDTH / source_width,
        available_height / source_height,
    )

    resized_width = max(
        1,
        int(round(source_width * scale)),
    )
    resized_height = max(
        1,
        int(round(source_height * scale)),
    )

    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_NEAREST,
    )

    tile = np.full(
        (TILE_HEIGHT, TILE_WIDTH, 3),
        255,
        dtype=np.uint8,
    )

    x_offset = (TILE_WIDTH - resized_width) // 2
    y_offset = (
        LABEL_HEIGHT
        + (available_height - resized_height) // 2
    )

    tile[
        y_offset:y_offset + resized_height,
        x_offset:x_offset + resized_width,
    ] = resized

    relative_path = image_path.relative_to(
        label_root
    )

    parts = relative_path.parts

    if len(parts) >= 3:
        source_class = parts[-3]
        stop_id = parts[-2]
        filename = parts[-1]
        text = f"{source_class}/{stop_id}/{filename}"
    else:
        text = str(relative_path)

    cv2.putText(
        tile,
        text[:34],
        (5, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )

    cv2.rectangle(
        tile,
        (0, 0),
        (TILE_WIDTH - 1, TILE_HEIGHT - 1),
        (150, 150, 150),
        1,
    )

    return tile


def make_blank_tile() -> np.ndarray:
    return np.full(
        (TILE_HEIGHT, TILE_WIDTH, 3),
        255,
        dtype=np.uint8,
    )


def save_pages(
    images: list[Path],
    label_root: Path,
    output_directory: Path,
) -> None:
    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    page_count = math.ceil(
        len(images) / IMAGES_PER_PAGE
    )

    for page_index in range(page_count):
        start = page_index * IMAGES_PER_PAGE
        end = start + IMAGES_PER_PAGE

        page_images = images[start:end]

        tiles = [
            make_tile(path, label_root)
            for path in page_images
        ]

        while len(tiles) < IMAGES_PER_PAGE:
            tiles.append(make_blank_tile())

        rows = []

        for row_index in range(ROWS):
            row_start = row_index * COLUMNS
            row_end = row_start + COLUMNS

            rows.append(
                np.hstack(
                    tiles[row_start:row_end]
                )
            )

        overview = np.vstack(rows)

        output_path = (
            output_directory
            / f"page_{page_index + 1:02d}.jpg"
        )

        if not cv2.imwrite(
            str(output_path),
            overview,
        ):
            raise RuntimeError(
                f"无法保存：{output_path}"
            )


def main() -> None:
    args = parse_args()

    dataset_root = resolve_project_path(args.dataset)
    output_root = resolve_project_path(args.output)

    if not dataset_root.is_dir():
        raise FileNotFoundError(
            f"找不到数据集：{dataset_root}"
        )

    total_images = 0

    for task, labels in TASK_LABELS.items():
        for split in SPLITS:
            for label in labels:
                label_root = (
                    dataset_root
                    / task
                    / split
                    / label
                )

                images = get_images(label_root)

                if not images:
                    raise RuntimeError(
                        f"没有找到图片：{label_root}"
                    )

                output_directory = (
                    output_root
                    / task
                    / split
                    / label
                )

                save_pages(
                    images,
                    label_root,
                    output_directory,
                )

                total_images += len(images)

                print(
                    f"[OK] {task}/{split}/{label}: "
                    f"{len(images)} 张"
                )

    print()
    print(f"检查图片总数：{total_images}")
    print(f"总览图目录：{output_root}")


if __name__ == "__main__":
    main()
