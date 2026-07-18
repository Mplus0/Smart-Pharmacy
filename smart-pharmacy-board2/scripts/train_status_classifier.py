from __future__ import annotations

import argparse
from pathlib import Path

from yolo_classification import PROJECT_ROOT, run_training


EXPECTED_COUNTS = {
    "train": {
        "idle": 40,
        "busy": 60,
    },
    "val": {
        "idle": 10,
        "busy": 60,
    },
    "test": {
        "idle": 10,
        "busy": 60,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="使用 YOLO11n-cls 训练空闲/忙碌状态分类模型"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "status_train.yaml",
        help="YOLO11 状态分类训练配置",
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
