from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from torchvision import transforms
from torchvision.transforms import InterpolationMode
from torchvision.transforms import functional as transform_functional
from ultralytics import YOLO
from ultralytics.data.dataset import ClassificationDataset
from ultralytics.models.yolo.classify import (
    ClassificationTrainer,
    ClassificationValidator,
)


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
VALID_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp"}
SPLITS = ("train", "val", "test")


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"找不到配置文件：{path}")
    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"YAML 内容无效：{path}")
    return data


def resolve_project_path(value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path.resolve()
    return (PROJECT_ROOT / path).resolve()


def resolve_model_weights(value: str | Path) -> str:
    path = Path(value)
    if path.is_absolute():
        return str(path)

    project_path = PROJECT_ROOT / path
    if project_path.is_file():
        return str(project_path.resolve())

    # yolo11n-cls.pt 这类官方模型名交给 Ultralytics 查找或下载。
    return str(value)


def get_direct_images(directory: Path) -> list[Path]:
    if not directory.is_dir():
        return []
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file()
        and path.suffix.lower() in VALID_IMAGE_SUFFIXES
    )


def validate_dataset(
    dataset_root: Path,
    class_names: list[str],
    expected_counts: dict[str, dict[str, int]],
) -> dict[str, dict[str, int]]:
    if not dataset_root.is_dir():
        raise FileNotFoundError(
            f"找不到 YOLO 分类数据集：{dataset_root}\n"
            "请先运行 scripts/prepare_roi_dataset.py --overwrite"
        )

    actual_counts: dict[str, dict[str, int]] = {}
    expected_class_set = set(class_names)

    for split_name in SPLITS:
        split_directory = dataset_root / split_name
        if not split_directory.is_dir():
            raise FileNotFoundError(
                f"缺少数据集划分目录：{split_directory}"
            )

        actual_class_set = {
            path.name
            for path in split_directory.iterdir()
            if path.is_dir()
        }
        if actual_class_set != expected_class_set:
            raise ValueError(
                f"{split_name} 类别目录应为 {sorted(expected_class_set)}，"
                f"实际为 {sorted(actual_class_set)}"
            )

        split_counts: dict[str, int] = {}
        for class_name in class_names:
            class_directory = split_directory / class_name
            nested_directories = [
                path
                for path in class_directory.iterdir()
                if path.is_dir()
            ]
            if nested_directories:
                raise ValueError(
                    f"YOLO 分类类别目录中不应再包含子目录："
                    f"{nested_directories[0]}\n"
                    "请重新运行 scripts/prepare_roi_dataset.py --overwrite"
                )

            image_count = len(get_direct_images(class_directory))
            expected_count = expected_counts[split_name][class_name]
            if image_count != expected_count:
                raise ValueError(
                    f"{split_name}/{class_name} 有 {image_count} 张图片，"
                    f"预期为 {expected_count} 张"
                )
            split_counts[class_name] = image_count

        actual_counts[split_name] = split_counts

    return actual_counts


class SquarePad:
    """使用白色补边将 ROI 变为正方形，避免裁掉文字或数字。"""

    def __call__(self, image):
        width, height = image.size
        side = max(width, height)
        horizontal_padding = side - width
        vertical_padding = side - height
        padding = (
            horizontal_padding // 2,
            vertical_padding // 2,
            horizontal_padding - horizontal_padding // 2,
            vertical_padding - vertical_padding // 2,
        )
        return transform_functional.pad(image, padding, fill=255)


class PreserveContentClassificationDataset(ClassificationDataset):
    """YOLO 分类数据集：保留完整 ROI，不使用随机裁剪。"""

    def __init__(
        self,
        root: str,
        args,
        augment: bool = False,
        prefix: str = "",
    ) -> None:
        super().__init__(root, args, augment, prefix)

        image_size = int(args.imgsz)
        transform_list: list[Any] = [
            SquarePad(),
            transforms.Resize(
                (image_size, image_size),
                interpolation=InterpolationMode.BILINEAR,
                antialias=True,
            ),
        ]

        if augment:
            scale_gain = float(args.scale)
            transform_list.extend(
                [
                    transforms.RandomAffine(
                        degrees=float(args.degrees),
                        translate=(
                            float(args.translate),
                            float(args.translate),
                        ),
                        scale=(1.0 - scale_gain, 1.0 + scale_gain),
                        interpolation=InterpolationMode.BILINEAR,
                        fill=255,
                    ),
                    transforms.ColorJitter(
                        brightness=float(args.hsv_v),
                        saturation=float(args.hsv_s),
                    ),
                ]
            )

        transform_list.append(transforms.ToTensor())
        self.torch_transforms = transforms.Compose(transform_list)


class PreserveContentTrainer(ClassificationTrainer):
    def build_dataset(
        self,
        img_path: str,
        mode: str = "train",
        batch=None,
    ):
        return PreserveContentClassificationDataset(
            root=img_path,
            args=self.args,
            augment=mode == "train",
            prefix=mode,
        )


class PreserveContentValidator(ClassificationValidator):
    def build_dataset(self, img_path: str):
        return PreserveContentClassificationDataset(
            root=img_path,
            args=self.args,
            augment=False,
            prefix=self.args.split,
        )


def resolve_device(value: object):
    text = str(value).strip().lower()
    if text in {"", "auto", "none"}:
        return None
    return value


def load_roi_from_config(
    config_path: Path,
    roi_name: str,
) -> tuple[int, int, int, int]:
    config = load_yaml(config_path)
    try:
        roi = tuple(
            int(value)
            for value in config["roi"][roi_name]["pixel_xywh"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"ROI 配置缺少 roi.{roi_name}.pixel_xywh"
        ) from error
    if len(roi) != 4:
        raise ValueError(f"{roi_name} ROI 必须包含 x、y、width、height")
    return roi


def crop_configured_roi(
    image: np.ndarray,
    roi: tuple[int, int, int, int],
) -> np.ndarray:
    x, y, width, height = roi
    image_height, image_width = image.shape[:2]
    if (
        x < 0
        or y < 0
        or width <= 0
        or height <= 0
        or x + width > image_width
        or y + height > image_height
    ):
        raise ValueError(
            f"ROI {roi} 超出图片范围 {image_width}×{image_height}"
        )
    crop = image[y:y + height, x:x + width]
    if crop.size == 0:
        raise ValueError("ROI 裁剪结果为空")
    return crop


def pad_and_resize_roi(
    roi_image: np.ndarray,
    image_size: int,
) -> np.ndarray:
    height, width = roi_image.shape[:2]
    side = max(width, height)
    square = np.full((side, side, 3), 255, dtype=np.uint8)
    x_offset = (side - width) // 2
    y_offset = (side - height) // 2
    square[
        y_offset:y_offset + height,
        x_offset:x_offset + width,
    ] = roi_image
    return cv2.resize(
        square,
        (image_size, image_size),
        interpolation=cv2.INTER_LINEAR,
    )


def ensure_output_available(
    output_directory: Path,
    overwrite: bool,
) -> None:
    if not output_directory.exists():
        return
    if any(output_directory.iterdir()) and not overwrite:
        raise FileExistsError(
            f"输出目录已存在且非空：{output_directory}\n"
            "请修改 output.name，或将 output.overwrite 设置为 true"
        )


def evaluate_test_predictions(
    model: YOLO,
    dataset_root: Path,
    class_names: list[str],
    output_directory: Path,
    image_size: int,
    batch_size: int,
    device,
) -> dict[str, Any]:
    test_samples: list[tuple[Path, str]] = []
    for class_name in class_names:
        test_samples.extend(
            (image_path, class_name)
            for image_path in get_direct_images(
                dataset_root / "test" / class_name
            )
        )

    prediction_results = model.predict(
        source=[str(path) for path, _ in test_samples],
        imgsz=image_size,
        batch=batch_size,
        device=device,
        verbose=False,
    )
    if len(prediction_results) != len(test_samples):
        raise RuntimeError(
            "测试集预测结果数量与输入图片数量不一致"
        )

    confusion_counts = {
        actual_name: {
            predicted_name: 0
            for predicted_name in class_names
        }
        for actual_name in class_names
    }
    prediction_rows: list[dict[str, object]] = []

    for (image_path, actual_name), result in zip(
        test_samples,
        prediction_results,
    ):
        if result.probs is None:
            raise RuntimeError(f"分类预测没有概率结果：{image_path}")

        predicted_index = int(result.probs.top1)
        predicted_name = str(result.names[predicted_index])
        confidence = float(result.probs.top1conf)
        if predicted_name not in confusion_counts[actual_name]:
            raise ValueError(
                f"模型返回了未配置的类别：{predicted_name}"
            )

        confusion_counts[actual_name][predicted_name] += 1
        prediction_rows.append(
            {
                "image": str(image_path.relative_to(dataset_root)),
                "actual": actual_name,
                "predicted": predicted_name,
                "confidence": f"{confidence:.8f}",
                "correct": actual_name == predicted_name,
            }
        )

    recalls: dict[str, float] = {}
    correct_count = 0
    for class_name in class_names:
        class_total = sum(confusion_counts[class_name].values())
        class_correct = confusion_counts[class_name][class_name]
        recalls[class_name] = (
            class_correct / class_total if class_total else 0.0
        )
        correct_count += class_correct

    metrics = {
        "accuracy": correct_count / len(test_samples),
        "balanced_accuracy": sum(recalls.values()) / len(recalls),
        "recall_per_class": recalls,
        "confusion_counts": confusion_counts,
    }

    predictions_path = output_directory / "test_predictions.csv"
    with predictions_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "image",
                "actual",
                "predicted",
                "confidence",
                "correct",
            ],
        )
        writer.writeheader()
        writer.writerows(prediction_rows)

    metrics_path = output_directory / "test_metrics.json"
    with metrics_path.open("w", encoding="utf-8") as file:
        json.dump(metrics, file, ensure_ascii=False, indent=2)

    return metrics


def run_training(
    config_path: Path,
    expected_counts: dict[str, dict[str, int]],
) -> None:
    config_path = config_path.resolve()
    config = load_yaml(config_path)

    seed = int(config["seed"])
    class_names = [str(name) for name in config["data"]["class_names"]]
    dataset_root = resolve_project_path(config["data"]["root"])
    actual_counts = validate_dataset(
        dataset_root,
        class_names,
        expected_counts,
    )

    model_config = config["model"]
    training_config = config["training"]
    augmentation_config = config["augmentation"]
    output_config = config["output"]

    output_project = resolve_project_path(output_config["project"])
    run_name = str(output_config["name"])
    overwrite = bool(output_config.get("overwrite", False))
    output_directory = output_project / run_name
    ensure_output_available(output_directory, overwrite)

    device = resolve_device(training_config.get("device", "auto"))
    image_size = int(model_config["image_size"])
    batch_size = int(training_config["batch_size"])
    number_of_workers = int(training_config["num_workers"])

    model = YOLO(resolve_model_weights(model_config["weights"]))
    model.train(
        data=str(dataset_root),
        trainer=PreserveContentTrainer,
        epochs=int(training_config["epochs"]),
        patience=int(training_config["patience"]),
        batch=batch_size,
        imgsz=image_size,
        workers=number_of_workers,
        device=device,
        optimizer=str(training_config["optimizer"]),
        lr0=float(training_config["learning_rate"]),
        lrf=float(training_config["final_learning_rate_fraction"]),
        weight_decay=float(training_config["weight_decay"]),
        dropout=float(model_config["dropout"]),
        amp=bool(training_config["use_amp"]),
        seed=seed,
        deterministic=bool(training_config["deterministic"]),
        degrees=float(augmentation_config["rotation_degrees"]),
        translate=float(augmentation_config["translate"]),
        scale=float(augmentation_config["scale"]),
        hsv_h=0.0,
        hsv_s=float(augmentation_config["saturation"]),
        hsv_v=float(augmentation_config["brightness"]),
        fliplr=0.0,
        flipud=0.0,
        project=str(output_project),
        name=run_name,
        exist_ok=overwrite,
        plots=True,
        verbose=True,
    )

    best_model_path = Path(model.trainer.best)
    if not best_model_path.is_file():
        raise RuntimeError(f"训练结束后未找到最佳模型：{best_model_path}")

    best_model = YOLO(str(best_model_path))
    test_metrics = best_model.val(
        data=str(dataset_root),
        split="test",
        validator=PreserveContentValidator,
        imgsz=image_size,
        batch=batch_size,
        workers=number_of_workers,
        device=device,
        project=str(output_directory),
        name="test",
        exist_ok=True,
        plots=True,
    )

    detailed_test_metrics = evaluate_test_predictions(
        model=best_model,
        dataset_root=dataset_root,
        class_names=class_names,
        output_directory=output_directory,
        image_size=image_size,
        batch_size=batch_size,
        device=device,
    )

    summary = {
        "model": str(model_config["weights"]),
        "dataset": str(config["data"]["root"]),
        "class_names": class_names,
        "seed": seed,
        "image_size": image_size,
        "sample_counts": actual_counts,
        "best_model": str(best_model_path),
        "test_top1_accuracy": float(test_metrics.top1),
        "test_top5_accuracy": float(test_metrics.top5),
        "test_balanced_accuracy": detailed_test_metrics[
            "balanced_accuracy"
        ],
        "test_recall_per_class": detailed_test_metrics[
            "recall_per_class"
        ],
    }
    summary_path = output_directory / "test_summary.json"
    with summary_path.open("w", encoding="utf-8") as file:
        json.dump(summary, file, ensure_ascii=False, indent=2)

    print(f"最佳模型：{best_model_path}")
    print(f"测试集 Top-1 准确率：{test_metrics.top1:.4f}")
    print(
        "测试集平衡准确率："
        f"{detailed_test_metrics['balanced_accuracy']:.4f}"
    )
    for class_name in class_names:
        print(
            f"测试集 {class_name} recall："
            f"{detailed_test_metrics['recall_per_class'][class_name]:.4f}"
        )
    print(f"测试结果摘要：{summary_path}")
