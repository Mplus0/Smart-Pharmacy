from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
DATASET_ROOT = PROJECT_ROOT / "dataset"
OUTPUT_ROOT = PROJECT_ROOT / "results" / "stop_overview"

CLASSES = [
    "free",
    "busy_5",
    "busy_6",
    "busy_7",
    "busy_8",
    "busy_9",
    "busy_10",
]

VALID_SUFFIXES = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
}

TILE_WIDTH = 320
TILE_HEIGHT = 240

GRID_COLUMNS = 4
GRID_ROWS = 3


def read_middle_frame(stop_dir: Path) -> tuple[np.ndarray, str]:
    images = sorted(
        path
        for path in stop_dir.iterdir()
        if path.is_file()
        and path.suffix.lower() in VALID_SUFFIXES
    )

    if not images:
        raise RuntimeError(f"停车批次中没有图片：{stop_dir}")

    # 每次停车有 5 张时，选择第 3 张作为代表
    middle_path = images[len(images) // 2]

    image = cv2.imread(
        str(middle_path),
        cv2.IMREAD_COLOR,
    )

    if image is None or image.size == 0:
        raise RuntimeError(f"无法读取图片：{middle_path}")

    return image, middle_path.name


def make_tile(
    image: np.ndarray,
    stop_name: str,
    filename: str,
) -> np.ndarray:
    tile = cv2.resize(
        image,
        (TILE_WIDTH, TILE_HEIGHT),
        interpolation=cv2.INTER_AREA,
    )

    # 添加半透明黑色标签背景
    overlay = tile.copy()

    cv2.rectangle(
        overlay,
        (0, 0),
        (TILE_WIDTH, 42),
        (0, 0, 0),
        thickness=-1,
    )

    tile = cv2.addWeighted(
        overlay,
        0.55,
        tile,
        0.45,
        0,
    )

    label = f"{stop_name}  {filename}"

    cv2.putText(
        tile,
        label,
        (8, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        thickness=2,
        lineType=cv2.LINE_AA,
    )

    return tile


def make_class_overview(class_name: str) -> None:
    class_dir = DATASET_ROOT / class_name

    if not class_dir.is_dir():
        raise FileNotFoundError(
            f"缺少类别文件夹：{class_dir}"
        )

    stop_dirs = sorted(
        path
        for path in class_dir.iterdir()
        if path.is_dir()
        and path.name.startswith("stop_")
    )

    if len(stop_dirs) != GRID_COLUMNS * GRID_ROWS:
        raise RuntimeError(
            f"{class_name} 有 {len(stop_dirs)} 个停车批次，"
            f"预期为 {GRID_COLUMNS * GRID_ROWS}"
        )

    tiles: list[np.ndarray] = []

    for stop_dir in stop_dirs:
        image, filename = read_middle_frame(stop_dir)

        tile = make_tile(
            image=image,
            stop_name=stop_dir.name,
            filename=filename,
        )

        tiles.append(tile)

    rows: list[np.ndarray] = []

    for row_index in range(GRID_ROWS):
        start = row_index * GRID_COLUMNS
        end = start + GRID_COLUMNS

        row = np.hstack(tiles[start:end])
        rows.append(row)

    overview = np.vstack(rows)

    OUTPUT_ROOT.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = OUTPUT_ROOT / f"{class_name}_overview.jpg"

    success = cv2.imwrite(
        str(output_path),
        overview,
    )

    if not success:
        raise RuntimeError(
            f"无法保存总览图：{output_path}"
        )

    print(f"[OK] {class_name}: {output_path}")


def main() -> None:
    for class_name in CLASSES:
        make_class_overview(class_name)

    print("\n总览图生成完成")
    print(f"输出目录：{OUTPUT_ROOT}")


if __name__ == "__main__":
    main()
