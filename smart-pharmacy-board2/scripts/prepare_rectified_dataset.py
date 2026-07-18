from __future__ import annotations

import csv
from pathlib import Path

import cv2

from check_board_locator import (
    CLASSES,
    QUAD_EXPAND_FACTOR,
    VALID_SUFFIXES,
    WARP_HEIGHT,
    WARP_WIDTH,
    expand_quad,
    find_board,
    warp_quad,
)


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
DATASET_ROOT = PROJECT_ROOT / "dataset"

OUTPUT_ROOT = PROJECT_ROOT / "results" / "rectified_dataset"
RECTIFIED_ROOT = OUTPUT_ROOT / "images"
FAILED_ROOT = OUTPUT_ROOT / "failed"
REPORT_PATH = OUTPUT_ROOT / "rectification_report.csv"


def get_images(stop_directory: Path) -> list[Path]:
    return sorted(
        path
        for path in stop_directory.iterdir()
        if path.is_file()
        and path.suffix.lower() in VALID_SUFFIXES
    )


def main() -> None:
    RECTIFIED_ROOT.mkdir(parents=True, exist_ok=True)
    FAILED_ROOT.mkdir(parents=True, exist_ok=True)

    report_rows: list[dict[str, object]] = []

    total_count = 0
    success_count = 0
    failed_count = 0

    for class_name in CLASSES:
        class_directory = DATASET_ROOT / class_name

        if not class_directory.is_dir():
            raise FileNotFoundError(
                f"缺少类别目录：{class_directory}"
            )

        stop_directories = sorted(
            path
            for path in class_directory.iterdir()
            if path.is_dir()
            and path.name.startswith("stop_")
        )

        for stop_directory in stop_directories:
            image_paths = get_images(stop_directory)

            output_stop_directory = (
                RECTIFIED_ROOT
                / class_name
                / stop_directory.name
            )

            output_stop_directory.mkdir(
                parents=True,
                exist_ok=True,
            )

            failed_stop_directory = (
                FAILED_ROOT
                / class_name
                / stop_directory.name
            )

            for image_path in image_paths:
                total_count += 1

                image = cv2.imread(
                    str(image_path),
                    cv2.IMREAD_COLOR,
                )

                if image is None or image.size == 0:
                    failed_count += 1

                    report_rows.append(
                        {
                            "class": class_name,
                            "stop_id": stop_directory.name,
                            "filename": image_path.name,
                            "success": False,
                            "reason": "image_read_failed",
                            "candidate_count": 0,
                            "score": "",
                            "area_ratio": "",
                            "aspect_ratio": "",
                            "rectangularity": "",
                            "white_ratio": "",
                            "dark_ring_ratio": "",
                            "output_path": "",
                        }
                    )

                    print(f"[FAIL] 无法读取：{image_path}")
                    continue

                candidate, candidate_count = find_board(image)

                if candidate is None:
                    failed_count += 1

                    failed_stop_directory.mkdir(
                        parents=True,
                        exist_ok=True,
                    )

                    failed_path = (
                        failed_stop_directory
                        / image_path.name
                    )

                    cv2.imwrite(
                        str(failed_path),
                        image,
                    )

                    report_rows.append(
                        {
                            "class": class_name,
                            "stop_id": stop_directory.name,
                            "filename": image_path.name,
                            "success": False,
                            "reason": "board_not_found",
                            "candidate_count": candidate_count,
                            "score": "",
                            "area_ratio": "",
                            "aspect_ratio": "",
                            "rectangularity": "",
                            "white_ratio": "",
                            "dark_ring_ratio": "",
                            "output_path": "",
                        }
                    )

                    print(
                        f"[FAIL] 未找到识别板："
                        f"{class_name}/"
                        f"{stop_directory.name}/"
                        f"{image_path.name}"
                    )
                    continue

                image_height, image_width = image.shape[:2]

                expanded_points = expand_quad(
                    candidate.points,
                    QUAD_EXPAND_FACTOR,
                    image_width,
                    image_height,
                )

                rectified = warp_quad(
                    image,
                    expanded_points,
                    WARP_WIDTH,
                    WARP_HEIGHT,
                )

                output_path = (
                    output_stop_directory
                    / image_path.name
                )

                write_success = cv2.imwrite(
                    str(output_path),
                    rectified,
                )

                if not write_success:
                    failed_count += 1

                    report_rows.append(
                        {
                            "class": class_name,
                            "stop_id": stop_directory.name,
                            "filename": image_path.name,
                            "success": False,
                            "reason": "image_write_failed",
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
                            "output_path": "",
                        }
                    )

                    print(f"[FAIL] 保存失败：{output_path}")
                    continue

                success_count += 1

                report_rows.append(
                    {
                        "class": class_name,
                        "stop_id": stop_directory.name,
                        "filename": image_path.name,
                        "success": True,
                        "reason": "",
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
                            "output_path": str(
                                output_path.relative_to(
                                    PROJECT_ROOT
                                )
                            ),
                    }
                )

                print(
                    f"[OK] {class_name}/"
                    f"{stop_directory.name}/"
                    f"{image_path.name}"
                )

    OUTPUT_ROOT.mkdir(
        parents=True,
        exist_ok=True,
    )

    with REPORT_PATH.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as file:
        fieldnames = [
            "class",
            "stop_id",
            "filename",
            "success",
            "reason",
            "candidate_count",
            "score",
            "area_ratio",
            "aspect_ratio",
            "rectangularity",
            "white_ratio",
            "dark_ring_ratio",
            "output_path",
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
    print("全部图片透视矫正完成")
    print("=" * 50)
    print(f"总图片数：{total_count}")
    print(f"成功数量：{success_count}")
    print(f"失败数量：{failed_count}")
    print(f"成功率：{success_rate * 100:.2f}%")
    print(f"矫正图片目录：{RECTIFIED_ROOT}")
    print(f"失败图片目录：{FAILED_ROOT}")
    print(f"报告文件：{REPORT_PATH}")


if __name__ == "__main__":
    main()
