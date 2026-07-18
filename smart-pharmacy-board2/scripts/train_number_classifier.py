from __future__ import annotations

import argparse
from pathlib import Path

from yolo_classification import PROJECT_ROOT, run_training


CLASS_NAMES = ["5", "6", "7", "8", "9", "10"]
EXPECTED_COUNTS = {
    "train": {class_name: 40 for class_name in CLASS_NAMES},
    "val": {class_name: 10 for class_name in CLASS_NAMES},
    "test": {class_name: 10 for class_name in CLASS_NAMES},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="使用 YOLO11n-cls 训练等待数字六分类模型"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "number_train.yaml",
        help="YOLO11 数字分类训练配置",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = (
        args.config
        if args.config.is_absolute()
        else PROJECT_ROOT / args.config
    )
    run_training(config_path, EXPECTED_COUNTS)


if __name__ == "__main__":
    main()
