from __future__ import annotations

import csv
from pathlib import Path

import cv2


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
DATASET_ROOT = PROJECT_ROOT / "dataset"
MANIFEST_PATH = PROJECT_ROOT / "results" / "dataset_manifest.csv"

CLASSES = [
    "free",
    "busy_5",
    "busy_6",
    "busy_7",
    "busy_8",
    "busy_9",
    "busy_10",
]

EXPECTED_STOPS_PER_CLASS = 12
EXPECTED_FRAMES_PER_STOP = 5

VALID_SUFFIXES = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
}


def get_image_files(directory: Path) -> list[Path]:
    """读取一个停车批次目录中的所有图像。"""
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file()
        and path.suffix.lower() in VALID_SUFFIXES
    )


def main() -> None:
    errors: list[str] = []
    manifest_rows: list[dict[str, object]] = []

    reference_resolution: tuple[int, int] | None = None

    total_valid_images = 0
    total_valid_stops = 0

    print(f"数据集根目录：{DATASET_ROOT}")
    print("开始检查数据集\n")

    for class_name in CLASSES:
        class_dir = DATASET_ROOT / class_name

        if not class_dir.is_dir():
            errors.append(f"缺少类别文件夹：{class_dir}")
            continue

        stop_dirs = sorted(
            path
            for path in class_dir.iterdir()
            if path.is_dir()
            and path.name.startswith("stop_")
        )

        print(
            f"{class_name}: "
            f"{len(stop_dirs)} 个停车批次"
        )

        if len(stop_dirs) != EXPECTED_STOPS_PER_CLASS:
            errors.append(
                f"{class_name} 有 {len(stop_dirs)} 个停车批次，"
                f"预期为 {EXPECTED_STOPS_PER_CLASS}"
            )

        class_image_count = 0

        for stop_dir in stop_dirs:
            images = get_image_files(stop_dir)

            print(
                f"  {stop_dir.name}: "
                f"{len(images)} 张"
            )

            if len(images) != EXPECTED_FRAMES_PER_STOP:
                errors.append(
                    f"{class_name}/{stop_dir.name} "
                    f"有 {len(images)} 张图片，"
                    f"预期为 {EXPECTED_FRAMES_PER_STOP}"
                )

            valid_images_in_stop = 0

            for frame_index, image_path in enumerate(
                images,
                start=1,
            ):
                image = cv2.imread(
                    str(image_path),
                    cv2.IMREAD_COLOR,
                )

                if image is None or image.size == 0:
                    errors.append(
                        f"图片损坏或无法读取：{image_path}"
                    )
                    continue

                height, width = image.shape[:2]
                resolution = (width, height)

                if reference_resolution is None:
                    reference_resolution = resolution
                elif resolution != reference_resolution:
                    errors.append(
                        f"分辨率不一致：{image_path} 为 "
                        f"{width}×{height}，参考分辨率为 "
                        f"{reference_resolution[0]}×"
                        f"{reference_resolution[1]}"
                    )

                manifest_rows.append(
                    {
                        "class": class_name,
                        "stop_id": stop_dir.name,
                        "frame_in_stop": frame_index,
                        "filename": image_path.name,
                        "relative_path": str(
                            image_path.relative_to(
                                DATASET_ROOT
                            )
                        ),
                        "width": width,
                        "height": height,
                    }
                )

                valid_images_in_stop += 1
                class_image_count += 1
                total_valid_images += 1

            if (
                valid_images_in_stop
                == EXPECTED_FRAMES_PER_STOP
            ):
                total_valid_stops += 1

        expected_class_images = (
            EXPECTED_STOPS_PER_CLASS
            * EXPECTED_FRAMES_PER_STOP
        )

        if class_image_count != expected_class_images:
            errors.append(
                f"{class_name} 有效图片数量为 "
                f"{class_image_count}，"
                f"预期为 {expected_class_images}"
            )

        print(
            f"  合计：{class_image_count} 张有效图片\n"
        )

    manifest_path = MANIFEST_PATH

    manifest_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with manifest_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as file:
        fieldnames = [
            "class",
            "stop_id",
            "frame_in_stop",
            "filename",
            "relative_path",
            "width",
            "height",
        ]

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(manifest_rows)

    print("=" * 50)
    print("检查结果")
    print("=" * 50)

    if reference_resolution is not None:
        print(
            "参考分辨率："
            f"{reference_resolution[0]}×"
            f"{reference_resolution[1]}"
        )

    print(f"有效停车批次：{total_valid_stops}")
    print(f"成功读取图片：{total_valid_images}")
    print(f"清单文件：{manifest_path}")

    if errors:
        print(f"\n发现 {len(errors)} 个问题：")

        for error in errors:
            print(f"[ERROR] {error}")

        raise SystemExit(1)

    print("\n数据集检查通过：")
    print("- 7 个类别")
    print("- 每类 12 次停车")
    print("- 每次停车 5 张图片")
    print("- 总计 84 次停车")
    print("- 总计 420 张有效图片")


if __name__ == "__main__":
    main()
