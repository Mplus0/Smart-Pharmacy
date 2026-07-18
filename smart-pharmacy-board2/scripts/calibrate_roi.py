from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent


def resolve_project_path(path: Path) -> Path:
    if path.is_absolute():
        return path.resolve()
    return (PROJECT_ROOT / path).resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="标定识别板状态 ROI 和数字 ROI"
    )

    parser.add_argument(
        "--image",
        type=Path,
        required=True,
        help="一张已经透视矫正的 busy_10 图片",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("config/board2_roi.yaml"),
        help="ROI 配置输出路径",
    )

    parser.add_argument(
        "--preview",
        type=Path,
        default=Path("results/board2_roi_preview.jpg"),
        help="ROI 预览图片输出路径",
    )

    return parser.parse_args()


def roi_to_normalized(
    roi: tuple[int, int, int, int],
    image_width: int,
    image_height: int,
) -> list[float]:
    x, y, width, height = roi

    return [
        round(x / image_width, 6),
        round(y / image_height, 6),
        round((x + width) / image_width, 6),
        round((y + height) / image_height, 6),
    ]


def validate_roi(
    roi: tuple[int, int, int, int],
    name: str,
) -> None:
    _, _, width, height = roi

    if width <= 0 or height <= 0:
        raise ValueError(
            f"{name} 无效，必须框选一个非空区域"
        )


def draw_roi(
    image,
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
        (x, max(y - 8, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def main() -> None:
    args = parse_args()

    image_path = resolve_project_path(args.image)
    output_path = resolve_project_path(args.output)
    preview_path = resolve_project_path(args.preview)

    if not image_path.is_file():
        raise FileNotFoundError(
            f"找不到标定图片：{image_path}"
        )

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR,
    )

    if image is None or image.size == 0:
        raise RuntimeError(
            f"无法读取标定图片：{image_path}"
        )

    image_height, image_width = image.shape[:2]

    print(f"标定图片：{image_path}")
    print(f"图片尺寸：{image_width}×{image_height}")
    print()
    print("第一次框选：只框住“忙碌”两个字")
    print("完成后按 Enter 或 Space 确认，按 C 取消")

    status_roi = cv2.selectROI(
        "Select status ROI",
        image,
        showCrosshair=True,
        fromCenter=False,
    )

    status_roi = tuple(
        int(value) for value in status_roi
    )

    validate_roi(status_roi, "status_roi")

    print()
    print("第二次框选：框住完整数字“10”")
    print("数字四周保留少量空白")
    print("完成后按 Enter 或 Space 确认，按 C 取消")

    number_roi = cv2.selectROI(
        "Select number ROI",
        image,
        showCrosshair=True,
        fromCenter=False,
    )

    number_roi = tuple(
        int(value) for value in number_roi
    )

    validate_roi(number_roi, "number_roi")

    cv2.destroyAllWindows()

    config = {
        "board": {
            "warp_width": image_width,
            "warp_height": image_height,
        },
        "roi": {
            "status": {
                "pixel_xywh": list(status_roi),
                "normalized_xyxy": roi_to_normalized(
                    status_roi,
                    image_width,
                    image_height,
                ),
            },
            "number": {
                "pixel_xywh": list(number_roi),
                "normalized_xyxy": roi_to_normalized(
                    number_roi,
                    image_width,
                    image_height,
                ),
            },
        },
    }

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with output_path.open(
        "w",
        encoding="utf-8",
    ) as file:
        yaml.safe_dump(
            config,
            file,
            allow_unicode=True,
            sort_keys=False,
        )

    preview = image.copy()

    draw_roi(
        preview,
        status_roi,
        "status_roi",
        (0, 255, 0),
    )

    draw_roi(
        preview,
        number_roi,
        "number_roi",
        (0, 0, 255),
    )

    preview_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    if not cv2.imwrite(
        str(preview_path),
        preview,
    ):
        raise RuntimeError(
            f"无法保存预览图片：{preview_path}"
        )

    print()
    print("ROI 标定完成")
    print(f"配置文件：{output_path}")
    print(f"预览图片：{preview_path}")
    print(f"status_roi 像素坐标：{status_roi}")
    print(f"number_roi 像素坐标：{number_roi}")


if __name__ == "__main__":
    main()
