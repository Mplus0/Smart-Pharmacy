from __future__ import annotations

import argparse
import csv
import random
import shutil
from collections import Counter
from pathlib import Path

import cv2
import yaml


CLASSES = [
    "free",
    "busy_5",
    "busy_6",
    "busy_7",
    "busy_8",
    "busy_9",
    "busy_10",
]

SPLITS = ["train", "val", "test"]

VALID_SUFFIXES = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
}


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent


def resolve_project_path(path: Path) -> Path:
    if path.is_absolute():
        return path.resolve()
    return (PROJECT_ROOT / path).resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="根据 ROI 和停车批次划分生成状态、数字分类数据集"
    )

    parser.add_argument(
        "--rectified-root",
        type=Path,
        default=Path("results/rectified_dataset/images"),
        help="透视矫正图片根目录",
    )

    parser.add_argument(
        "--roi-config",
        type=Path,
        default=Path("config/board2_roi_final.yaml"),
        help="最终 ROI 配置文件",
    )

    parser.add_argument(
        "--split-config",
        type=Path,
        default=Path("config/dataset_split.yaml"),
        help="停车批次划分配置",
    )

    parser.add_argument(
        "--status-train-config",
        type=Path,
        default=Path("config/status_train.yaml"),
        help="状态分类训练配置，用于生成平衡训练子集",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/classification_dataset"),
        help="输出目录",
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="删除并重新生成已经存在的输出目录",
    )

    return parser.parse_args()


def load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"找不到配置文件：{path.resolve()}")

    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    if not isinstance(data, dict):
        raise ValueError(f"YAML 内容无效：{path}")

    return data


def load_roi(
    config_path: Path,
) -> tuple[
    tuple[int, int, int, int],
    tuple[int, int, int, int],
]:
    config = load_yaml(config_path)

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
            "ROI 配置缺少："
            "roi.status.pixel_xywh 或 "
            "roi.number.pixel_xywh"
        ) from error

    if len(status_roi) != 4:
        raise ValueError(
            f"status ROI 应包含 4 个数值，实际为：{status_roi}"
        )

    if len(number_roi) != 4:
        raise ValueError(
            f"number ROI 应包含 4 个数值，实际为：{number_roi}"
        )

    return status_roi, number_roi


def validate_roi(
    roi: tuple[int, int, int, int],
    image_width: int,
    image_height: int,
    name: str,
) -> None:
    x, y, width, height = roi

    if x < 0 or y < 0:
        raise ValueError(f"{name} 起点不能小于 0：{roi}")

    if width <= 0 or height <= 0:
        raise ValueError(f"{name} 宽高必须大于 0：{roi}")

    if x + width > image_width:
        raise ValueError(
            f"{name} 超出图像右边界：{roi}，"
            f"图像宽度为 {image_width}"
        )

    if y + height > image_height:
        raise ValueError(
            f"{name} 超出图像下边界：{roi}，"
            f"图像高度为 {image_height}"
        )


def crop_roi(
    image,
    roi: tuple[int, int, int, int],
):
    x, y, width, height = roi
    return image[y:y + height, x:x + width]


def get_images(stop_directory: Path) -> list[Path]:
    return sorted(
        path
        for path in stop_directory.iterdir()
        if path.is_file()
        and path.suffix.lower() in VALID_SUFFIXES
    )


def normalize_stop_name(value: object) -> str:
    """
    同时兼容 stop_001 和 1 两种 YAML 写法。
    """
    text = str(value)

    if text.startswith("stop_"):
        return text

    try:
        stop_number = int(text)
    except ValueError as error:
        raise ValueError(
            f"无法识别停车批次名称：{value}"
        ) from error

    return f"stop_{stop_number:03d}"


def validate_split_config(split_config: dict) -> None:
    for class_name in CLASSES:
        if class_name not in split_config:
            raise ValueError(
                f"dataset_split.yaml 缺少类别：{class_name}"
            )

        split_sets: dict[str, set[str]] = {}

        for split_name in SPLITS:
            if split_name not in split_config[class_name]:
                raise ValueError(
                    f"{class_name} 缺少划分：{split_name}"
                )

            values = {
                normalize_stop_name(value)
                for value in split_config[class_name][split_name]
            }

            if not values:
                raise ValueError(
                    f"{class_name}/{split_name} 不能为空"
                )

            split_sets[split_name] = values

        train_val_overlap = (
            split_sets["train"]
            & split_sets["val"]
        )

        train_test_overlap = (
            split_sets["train"]
            & split_sets["test"]
        )

        val_test_overlap = (
            split_sets["val"]
            & split_sets["test"]
        )

        if train_val_overlap:
            raise ValueError(
                f"{class_name} 的 train 和 val 重复："
                f"{sorted(train_val_overlap)}"
            )

        if train_test_overlap:
            raise ValueError(
                f"{class_name} 的 train 和 test 重复："
                f"{sorted(train_test_overlap)}"
            )

        if val_test_overlap:
            raise ValueError(
                f"{class_name} 的 val 和 test 重复："
                f"{sorted(val_test_overlap)}"
            )

        all_stops = (
            split_sets["train"]
            | split_sets["val"]
            | split_sets["test"]
        )

        if len(all_stops) != 12:
            raise ValueError(
                f"{class_name} 总停车批次数为 "
                f"{len(all_stops)}，预期为 12。"
                f"实际批次：{sorted(all_stops)}"
            )


def build_status_train_selection(
    split_config: dict,
    status_train_config: dict,
) -> tuple[dict[str, set[str]], dict]:
    subset_config = status_train_config.get("data", {}).get(
        "train_subset",
        {},
    )
    subset_enabled = bool(subset_config.get("enabled", False))
    subset_seed = int(
        subset_config.get("seed", status_train_config["seed"])
    )
    random_generator = random.Random(subset_seed)

    available_stops = {
        class_name: sorted(
            normalize_stop_name(value)
            for value in split_config[class_name]["train"]
        )
        for class_name in CLASSES
    }

    if not subset_enabled:
        selected_stops = {
            class_name: set(stop_names)
            for class_name, stop_names in available_stops.items()
        }
    else:
        selected_stops: dict[str, set[str]] = {}

        keep_all_idle = bool(
            subset_config.get("keep_all_idle", True)
        )
        idle_candidates = available_stops["free"]

        if keep_all_idle:
            selected_stops["free"] = set(idle_candidates)
        else:
            idle_limit = int(subset_config["idle_stops_limit"])
            if idle_limit <= 0 or idle_limit > len(idle_candidates):
                raise ValueError(
                    "data.train_subset.idle_stops_limit 超出可选范围"
                )
            selected_stops["free"] = set(
                random_generator.sample(idle_candidates, idle_limit)
            )

        busy_source_classes = [
            str(value)
            for value in subset_config.get(
                "busy_source_classes",
                [],
            )
        ]
        expected_busy_classes = set(CLASSES) - {"free"}

        if set(busy_source_classes) != expected_busy_classes:
            raise ValueError(
                "data.train_subset.busy_source_classes 必须完整包含 "
                "busy_5～busy_10，且不能包含其他类别"
            )

        stops_per_class = int(
            subset_config.get("busy_stops_per_source_class", 2)
        )
        if stops_per_class <= 0:
            raise ValueError(
                "data.train_subset.busy_stops_per_source_class 必须大于 0"
            )

        for class_name in busy_source_classes:
            candidates = available_stops[class_name]
            if stops_per_class > len(candidates):
                raise ValueError(
                    f"{class_name} 只有 {len(candidates)} 个训练停车批次，"
                    f"无法选择 {stops_per_class} 个"
                )
            selected_stops[class_name] = set(
                random_generator.sample(candidates, stops_per_class)
            )

    selection_summary = {
        "enabled": subset_enabled,
        "seed": subset_seed,
        "selected_train_stops": {
            class_name: sorted(selected_stops[class_name])
            for class_name in CLASSES
        },
    }
    return selected_stops, selection_summary


def prepare_output_directory(
    output_root: Path,
    overwrite: bool,
) -> None:
    if output_root.exists():
        has_content = any(output_root.iterdir())

        if has_content and not overwrite:
            raise FileExistsError(
                f"输出目录非空：{output_root.resolve()}\n"
                "确认需要重新生成时添加 --overwrite"
            )

        if overwrite:
            shutil.rmtree(output_root)

    output_root.mkdir(parents=True, exist_ok=True)


def write_crop(
    crop,
    output_path: Path,
) -> None:
    if crop is None or crop.size == 0:
        raise ValueError(
            f"裁剪结果为空，无法保存：{output_path}"
        )

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    # 使用 PNG，避免再次进行 JPEG 有损压缩。
    output_path = output_path.with_suffix(".png")

    if not cv2.imwrite(str(output_path), crop):
        raise RuntimeError(
            f"无法保存图片：{output_path}"
        )


def main() -> None:
    args = parse_args()

    rectified_root = resolve_project_path(args.rectified_root)
    roi_config_path = resolve_project_path(args.roi_config)
    split_config_path = resolve_project_path(args.split_config)
    status_train_config_path = resolve_project_path(
        args.status_train_config
    )
    output_root = resolve_project_path(args.output)

    if not rectified_root.is_dir():
        raise FileNotFoundError(
            f"找不到矫正图片目录：{rectified_root}"
        )

    status_roi, number_roi = load_roi(
        roi_config_path
    )

    split_config = load_yaml(
        split_config_path
    )

    validate_split_config(split_config)

    status_train_config = load_yaml(status_train_config_path)
    status_train_stops, status_selection_summary = (
        build_status_train_selection(
            split_config,
            status_train_config,
        )
    )

    prepare_output_directory(
        output_root,
        args.overwrite,
    )

    manifest_rows: list[dict[str, object]] = []
    counts: Counter[tuple[str, str, str]] = Counter()

    for class_name in CLASSES:
        class_directory = rectified_root / class_name

        if not class_directory.is_dir():
            raise FileNotFoundError(
                f"缺少矫正图片类别目录：{class_directory}"
            )

        for split_name in SPLITS:
            stop_names = [
                normalize_stop_name(value)
                for value in split_config[class_name][split_name]
            ]

            for stop_name in stop_names:
                stop_directory = (
                    class_directory / stop_name
                )

                if not stop_directory.is_dir():
                    raise FileNotFoundError(
                        f"找不到停车批次目录："
                        f"{stop_directory}"
                    )

                images = get_images(stop_directory)

                if len(images) != 5:
                    raise RuntimeError(
                        f"{class_name}/{stop_name} "
                        f"有 {len(images)} 张图片，"
                        "预期为 5"
                    )

                for image_path in images:
                    image = cv2.imread(
                        str(image_path),
                        cv2.IMREAD_COLOR,
                    )

                    if image is None or image.size == 0:
                        raise RuntimeError(
                            f"无法读取图片：{image_path}"
                        )

                    image_height, image_width = (
                        image.shape[:2]
                    )

                    validate_roi(
                        status_roi,
                        image_width,
                        image_height,
                        "status_roi",
                    )

                    validate_roi(
                        number_roi,
                        image_width,
                        image_height,
                        "number_roi",
                    )

                    include_status = (
                        split_name != "train"
                        or stop_name in status_train_stops[class_name]
                    )

                    if include_status:
                        status_crop = crop_roi(
                            image,
                            status_roi,
                        )

                        status_label = (
                            "idle"
                            if class_name == "free"
                            else "busy"
                        )
                        status_filename = (
                            f"{class_name}__{stop_name}__"
                            f"{image_path.name}"
                        )
                        status_output = (
                            output_root
                            / "status"
                            / split_name
                            / status_label
                            / status_filename
                        )

                        write_crop(
                            status_crop,
                            status_output,
                        )

                        final_status_output = (
                            status_output.with_suffix(".png")
                        )

                        manifest_rows.append(
                            {
                                "task": "status",
                                "split": split_name,
                                "label": status_label,
                                "source_class": class_name,
                                "stop_id": stop_name,
                                "source_path": str(
                                    image_path.relative_to(
                                        rectified_root
                                    )
                                ),
                                "output_path": str(
                                    final_status_output.relative_to(
                                        output_root
                                    )
                                ),
                                "roi_x": status_roi[0],
                                "roi_y": status_roi[1],
                                "roi_width": status_roi[2],
                                "roi_height": status_roi[3],
                            }
                        )

                        counts[
                            (
                                "status",
                                split_name,
                                status_label,
                            )
                        ] += 1

                    # 空闲状态没有等待数字，不生成数字样本。
                    if class_name == "free":
                        continue

                    number_label = class_name.replace(
                        "busy_",
                        "",
                        1,
                    )

                    number_crop = crop_roi(
                        image,
                        number_roi,
                    )

                    number_filename = (
                        f"{class_name}__{stop_name}__"
                        f"{image_path.name}"
                    )
                    number_output = (
                        output_root
                        / "number"
                        / split_name
                        / number_label
                        / number_filename
                    )

                    write_crop(
                        number_crop,
                        number_output,
                    )

                    final_number_output = (
                        number_output.with_suffix(".png")
                    )

                    manifest_rows.append(
                        {
                            "task": "number",
                            "split": split_name,
                            "label": number_label,
                            "source_class": class_name,
                            "stop_id": stop_name,
                            "source_path": str(
                                image_path.relative_to(
                                    rectified_root
                                )
                            ),
                            "output_path": str(
                                final_number_output.relative_to(
                                    output_root
                                )
                            ),
                            "roi_x": number_roi[0],
                            "roi_y": number_roi[1],
                            "roi_width": number_roi[2],
                            "roi_height": number_roi[3],
                        }
                    )

                    counts[
                        (
                            "number",
                            split_name,
                            number_label,
                        )
                    ] += 1

    manifest_path = (
        output_root / "roi_dataset_manifest.csv"
    )

    with manifest_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as file:
        fieldnames = [
            "task",
            "split",
            "label",
            "source_class",
            "stop_id",
            "source_path",
            "output_path",
            "roi_x",
            "roi_y",
            "roi_width",
            "roi_height",
        ]

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(manifest_rows)

    status_selection_path = (
        output_root / "status_train_subset.yaml"
    )
    with status_selection_path.open(
        "w",
        encoding="utf-8",
    ) as file:
        yaml.safe_dump(
            status_selection_summary,
            file,
            allow_unicode=True,
            sort_keys=False,
        )

    print("\n状态数据集")
    for split_name in SPLITS:
        idle_count = counts[
            ("status", split_name, "idle")
        ]

        busy_count = counts[
            ("status", split_name, "busy")
        ]

        print(
            f"{split_name:5s}: "
            f"idle={idle_count:3d}, "
            f"busy={busy_count:3d}, "
            f"total={idle_count + busy_count:3d}"
        )

    print("\n数字数据集")
    for split_name in SPLITS:
        class_counts = []

        for label in ["5", "6", "7", "8", "9", "10"]:
            count = counts[
                ("number", split_name, label)
            ]

            class_counts.append(
                f"{label}={count}"
            )

        print(
            f"{split_name:5s}: "
            + ", ".join(class_counts)
        )

    status_total = sum(
        count
        for (task, _, _), count in counts.items()
        if task == "status"
    )

    number_total = sum(
        count
        for (task, _, _), count in counts.items()
        if task == "number"
    )

    print("\n生成完成")
    print(f"状态样本：{status_total}")
    print(f"数字样本：{number_total}")
    print(f"总裁剪样本：{status_total + number_total}")
    print(f"输出目录：{output_root}")
    print(f"清单文件：{manifest_path}")
    print(f"状态训练子集：{status_selection_path}")


if __name__ == "__main__":
    main()
