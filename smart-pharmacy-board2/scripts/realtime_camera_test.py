from __future__ import annotations

import argparse
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ultralytics import YOLO

from black_frame_locator import (
    AlignmentResult,
    WARP_HEIGHT,
    WARP_WIDTH,
    align_board_to_training_coordinates,
    find_black_frame,
)
from yolo_classification import (
    PROJECT_ROOT,
    crop_configured_roi,
    load_roi_from_config,
    load_yaml,
    pad_and_resize_roi,
    resolve_device,
    resolve_project_path,
)


WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 900
WINDOW_NAME = "Smart Pharmacy Board 2 - Realtime Debug"

COLOR_BACKGROUND = (24, 27, 32)
COLOR_PANEL = (36, 40, 47)
COLOR_PANEL_BORDER = (75, 82, 92)
COLOR_TEXT = (235, 238, 242)
COLOR_MUTED = (155, 164, 176)
COLOR_GREEN = (72, 196, 118)
COLOR_ORANGE = (40, 158, 235)
COLOR_RED = (70, 70, 230)
COLOR_YELLOW = (65, 220, 235)


class ProbabilitySmoother:
    def __init__(self, window_size: int) -> None:
        if window_size <= 0:
            raise ValueError("smoothing-window 必须大于 0")
        self.history: deque[dict[str, float]] = deque(
            maxlen=window_size
        )

    def update(
        self,
        probabilities: dict[str, float],
    ) -> dict[str, float]:
        self.history.append(probabilities)
        class_names = probabilities.keys()
        return {
            class_name: sum(
                values[class_name] for values in self.history
            )
            / len(self.history)
            for class_name in class_names
        }

    def reset(self) -> None:
        self.history.clear()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "使用摄像头和黑框实时定位识别板，并运行状态和数字 YOLO11 分类模型"
        )
    )
    parser.add_argument(
        "--status-model",
        type=Path,
        required=True,
        help="状态分类模型 best.pt 路径",
    )
    parser.add_argument(
        "--number-model",
        type=Path,
        required=True,
        help="数字分类模型 best.pt 路径",
    )
    parser.add_argument(
        "--camera",
        default="0",
        help="摄像头序号、视频文件或 RTSP 地址；默认 0",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "dshow", "msmf", "v4l2"],
        default="auto",
        help="OpenCV 摄像头后端；Windows 可尝试 dshow 或 msmf",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument(
        "--device",
        default="auto",
        help="推理设备，例如 auto、cpu、0",
    )
    parser.add_argument(
        "--smoothing-window",
        type=int,
        default=5,
        help="分类概率滑动平均帧数；默认 5",
    )
    parser.add_argument(
        "--black-threshold",
        type=int,
        default=110,
        help="黑框灰度上限（1~254）；光线较暗时可适当增大，默认 110",
    )
    parser.add_argument(
        "--mirror",
        action="store_true",
        help="水平镜像摄像头画面",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/realtime_debug"),
        help="按 S 保存调试素材的根目录",
    )
    parser.add_argument(
        "--auto-save-interval",
        type=float,
        default=0.0,
        help="自动保存截图的秒数间隔；0 表示关闭",
    )
    parser.add_argument(
        "--roi-config",
        type=Path,
        default=PROJECT_ROOT / "config" / "board2_roi_final.yaml",
    )
    parser.add_argument(
        "--status-config",
        type=Path,
        default=PROJECT_ROOT / "config" / "status_train.yaml",
    )
    parser.add_argument(
        "--number-config",
        type=Path,
        default=PROJECT_ROOT / "config" / "number_train.yaml",
    )
    return parser.parse_args()


def parse_camera_source(value: str) -> int | str:
    text = str(value).strip()
    if text.lstrip("-").isdigit():
        return int(text)
    return text


def open_camera(
    source: int | str,
    backend_name: str,
) -> cv2.VideoCapture:
    backends = {
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
        "v4l2": cv2.CAP_V4L2,
    }
    if backend_name == "auto":
        capture = cv2.VideoCapture(source)
    else:
        capture = cv2.VideoCapture(source, backends[backend_name])
    return capture


def model_class_names(model: YOLO) -> set[str]:
    names = model.names
    if isinstance(names, dict):
        return {str(value) for value in names.values()}
    return {str(value) for value in names}


def validate_model_classes(
    model: YOLO,
    expected_classes: set[str],
    model_name: str,
) -> None:
    actual_classes = model_class_names(model)
    if actual_classes != expected_classes:
        raise ValueError(
            f"{model_name} 模型类别为 {sorted(actual_classes)}，"
            f"预期为 {sorted(expected_classes)}"
        )


def predict_probabilities(
    model: YOLO,
    prepared_roi: np.ndarray,
    image_size: int,
    device,
) -> dict[str, float]:
    result = model.predict(
        source=prepared_roi,
        imgsz=image_size,
        device=device,
        verbose=False,
    )[0]
    if result.probs is None:
        raise RuntimeError("模型没有返回分类概率")

    probability_values = result.probs.data.detach().cpu().numpy()
    return {
        str(result.names[index]): float(probability_values[index])
        for index in range(len(probability_values))
    }


def top_prediction(
    probabilities: dict[str, float],
) -> tuple[str, float]:
    return max(probabilities.items(), key=lambda item: item[1])


def fit_image(
    image: np.ndarray | None,
    width: int,
    height: int,
    background: tuple[int, int, int] = COLOR_BACKGROUND,
) -> np.ndarray:
    canvas = np.full((height, width, 3), background, dtype=np.uint8)
    if image is None or image.size == 0:
        return canvas

    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, height / source_height)
    resized_width = max(1, int(round(source_width * scale)))
    resized_height = max(1, int(round(source_height * scale)))
    interpolation = (
        cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    )
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=interpolation,
    )
    x_offset = (width - resized_width) // 2
    y_offset = (height - resized_height) // 2
    canvas[
        y_offset:y_offset + resized_height,
        x_offset:x_offset + resized_width,
    ] = resized
    return canvas


def draw_text(
    image: np.ndarray,
    text: str,
    position: tuple[int, int],
    scale: float = 0.55,
    color: tuple[int, int, int] = COLOR_TEXT,
    thickness: int = 1,
) -> None:
    cv2.putText(
        image,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_panel(
    canvas: np.ndarray,
    x: int,
    y: int,
    width: int,
    height: int,
    title: str,
) -> tuple[int, int, int, int]:
    cv2.rectangle(
        canvas,
        (x, y),
        (x + width, y + height),
        COLOR_PANEL,
        thickness=-1,
    )
    cv2.rectangle(
        canvas,
        (x, y),
        (x + width, y + height),
        COLOR_PANEL_BORDER,
        thickness=1,
    )
    draw_text(
        canvas,
        title,
        (x + 14, y + 27),
        scale=0.58,
        color=COLOR_TEXT,
        thickness=2,
    )
    return x + 12, y + 40, width - 24, height - 52


def draw_confidence_bar(
    image: np.ndarray,
    x: int,
    y: int,
    width: int,
    label: str,
    value: float,
    color: tuple[int, int, int],
) -> None:
    draw_text(image, label, (x, y + 15), 0.43, COLOR_MUTED)
    bar_x = x + 94
    bar_width = max(20, width - 160)
    cv2.rectangle(
        image,
        (bar_x, y + 2),
        (bar_x + bar_width, y + 17),
        (65, 69, 77),
        thickness=-1,
    )
    cv2.rectangle(
        image,
        (bar_x, y + 2),
        (bar_x + int(round(bar_width * value)), y + 17),
        color,
        thickness=-1,
    )
    draw_text(
        image,
        f"{value:.3f}",
        (bar_x + bar_width + 8, y + 15),
        0.43,
        COLOR_TEXT,
    )


def annotate_camera_frame(
    frame: np.ndarray,
    candidate,
    candidate_count: int,
    alignment: AlignmentResult | None,
) -> np.ndarray:
    annotated = frame.copy()
    if candidate is None:
        draw_text(
            annotated,
            f"BLACK FRAME NOT FOUND | candidates={candidate_count}",
            (20, 36),
            0.78,
            COLOR_RED,
            2,
        )
        return annotated

    frame_color = COLOR_GREEN if alignment is not None else COLOR_ORANGE
    cv2.polylines(
        annotated,
        [np.round(candidate.points).astype(np.int32)],
        True,
        frame_color,
        3,
        cv2.LINE_AA,
    )
    if alignment is None:
        lock_text = "BLACK FRAME FOUND | INNER ALIGNMENT FAILED"
    else:
        lock_text = (
            "BOARD ALIGNED | "
            f"black_score={candidate.score:.3f} "
            f"| white_score={alignment.score:.3f}"
        )
    draw_text(
        annotated,
        lock_text,
        (20, 36),
        0.72,
        frame_color,
        2,
    )
    draw_text(
        annotated,
        (
            f"BLACK CANDIDATES={candidate_count} | "
            f"SIDE CONTINUITY={candidate.border_features.minimum_continuity:.3f} "
            f"| CHILD RATIO={candidate.child_area_ratio:.3f}"
        ),
        (20, 66),
        0.50,
        COLOR_TEXT,
        1,
    )
    return annotated


def annotate_rectified_board(
    rectified: np.ndarray,
    status_roi: tuple[int, int, int, int],
    number_roi: tuple[int, int, int, int],
) -> np.ndarray:
    annotated = rectified.copy()
    for roi, label, color in (
        (status_roi, "STATUS ROI", COLOR_GREEN),
        (number_roi, "NUMBER ROI", COLOR_ORANGE),
    ):
        x, y, width, height = roi
        cv2.rectangle(
            annotated,
            (x, y),
            (x + width, y + height),
            color,
            2,
            cv2.LINE_AA,
        )
        draw_text(
            annotated,
            label,
            (x, max(18, y - 7)),
            0.42,
            color,
            1,
        )
    return annotated


def build_visualization(
    annotated_frame: np.ndarray,
    rectified_annotated: np.ndarray | None,
    status_roi_image: np.ndarray | None,
    number_roi_image: np.ndarray | None,
    state: dict[str, Any],
) -> np.ndarray:
    canvas = np.full(
        (WINDOW_HEIGHT, WINDOW_WIDTH, 3),
        COLOR_BACKGROUND,
        dtype=np.uint8,
    )

    draw_text(
        canvas,
        "SMART PHARMACY BOARD 2 | TWO-STAGE ALIGNMENT DEBUG",
        (22, 39),
        0.88,
        COLOR_TEXT,
        2,
    )
    draw_text(
        canvas,
        state["timestamp"],
        (1240, 37),
        0.55,
        COLOR_MUTED,
    )

    camera_box = draw_panel(
        canvas, 20, 66, 1000, 774, "A. LIVE CAMERA / TWO-STAGE LOCALIZATION"
    )
    camera_view = fit_image(annotated_frame, camera_box[2], camera_box[3])
    x, y, width, height = camera_box
    canvas[y:y + height, x:x + width] = camera_view

    board_box = draw_panel(
        canvas, 1040, 66, 540, 360, "B. RECTIFIED BOARD / ROI POSITION"
    )
    board_view = fit_image(
        rectified_annotated,
        board_box[2],
        board_box[3],
    )
    x, y, width, height = board_box
    canvas[y:y + height, x:x + width] = board_view

    status_box = draw_panel(
        canvas, 1040, 442, 260, 196, "C1. STATUS ROI"
    )
    status_view = fit_image(
        status_roi_image,
        status_box[2],
        status_box[3] - 32,
        background=(245, 245, 245),
    )
    x, y, width, _ = status_box
    canvas[y:y + status_view.shape[0], x:x + width] = status_view

    number_box = draw_panel(
        canvas, 1320, 442, 260, 196, "C2. NUMBER ROI"
    )
    number_view = fit_image(
        number_roi_image,
        number_box[2],
        number_box[3] - 32,
        background=(245, 245, 245),
    )
    x, y, width, _ = number_box
    canvas[y:y + number_view.shape[0], x:x + width] = number_view

    result_box = draw_panel(
        canvas, 1040, 654, 540, 186, "D. REALTIME INFERENCE"
    )
    x, y, width, _ = result_box

    board_color = COLOR_GREEN if state["board_found"] else COLOR_RED
    board_text = "ALIGNED" if state["board_found"] else "SEARCHING"
    draw_text(
        canvas,
        f"BOARD: {board_text}",
        (x, y + 18),
        0.58,
        board_color,
        2,
    )
    draw_text(
        canvas,
        f"FPS: {state['fps']:.1f}   FRAME: {state['frame_id']}",
        (x + 250, y + 18),
        0.50,
        COLOR_MUTED,
    )

    if state["status_name"] is None:
        draw_text(
            canvas,
            "STATUS: --",
            (x, y + 52),
            0.72,
            COLOR_MUTED,
            2,
        )
        draw_text(
            canvas,
            "NUMBER: --",
            (x + 260, y + 52),
            0.72,
            COLOR_MUTED,
            2,
        )
        if state.get("error"):
            draw_text(
                canvas,
                str(state["error"])[:72],
                (x, y + 90),
                0.45,
                COLOR_RED,
            )
    else:
        status_name = str(state["status_name"]).upper()
        status_confidence = float(state["status_confidence"])
        status_color = (
            COLOR_ORANGE if status_name == "BUSY" else COLOR_GREEN
        )
        draw_text(
            canvas,
            f"STATUS: {status_name}  {status_confidence:.3f}",
            (x, y + 49),
            0.68,
            status_color,
            2,
        )

        if status_name == "BUSY":
            number_text = (
                f"NUMBER: {state['number_name']}  "
                f"{state['number_confidence']:.3f}"
            )
            number_color = COLOR_ORANGE
        else:
            number_text = "NUMBER: N/A (IDLE)"
            number_color = COLOR_MUTED
        draw_text(
            canvas,
            number_text,
            (x + 260, y + 49),
            0.62,
            number_color,
            2,
        )

        status_probabilities = state["status_probabilities"]
        draw_confidence_bar(
            canvas,
            x,
            y + 66,
            width,
            "idle",
            float(status_probabilities.get("idle", 0.0)),
            COLOR_GREEN,
        )
        draw_confidence_bar(
            canvas,
            x,
            y + 90,
            width,
            "busy",
            float(status_probabilities.get("busy", 0.0)),
            COLOR_ORANGE,
        )

        number_probabilities = sorted(
            state["number_probabilities"].items(),
            key=lambda item: item[1],
            reverse=True,
        )[:3]
        top_number_text = "  ".join(
            f"{name}:{probability:.3f}"
            for name, probability in number_probabilities
        )
        draw_text(
            canvas,
            f"NUMBER TOP-3: {top_number_text}",
            (x, y + 137),
            0.46,
            COLOR_TEXT,
        )

    draw_text(
        canvas,
        "S: SAVE DEBUG SNAPSHOT   SPACE: PAUSE / RESUME   Q or ESC: QUIT",
        (24, 878),
        0.58,
        COLOR_MUTED,
        1,
    )
    return canvas


def write_debug_image(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"无法保存调试图片：{path}")


def save_debug_snapshot(
    output_root: Path,
    visualization: np.ndarray,
    raw_frame: np.ndarray,
    rectified: np.ndarray | None,
    status_roi_image: np.ndarray | None,
    number_roi_image: np.ndarray | None,
    black_mask: np.ndarray | None,
    alignment: AlignmentResult | None,
    state: dict[str, Any],
) -> Path:
    timestamp = datetime.now()
    base_folder_name = timestamp.strftime("%Y%m%d_%H%M%S_%f")[:-3]
    snapshot_directory = output_root / "snapshots" / base_folder_name
    duplicate_index = 2
    while snapshot_directory.exists():
        snapshot_directory = (
            output_root
            / "snapshots"
            / f"{base_folder_name}_{duplicate_index:02d}"
        )
        duplicate_index += 1
    snapshot_directory.mkdir(parents=True, exist_ok=False)

    write_debug_image(
        snapshot_directory / "01_realtime_overview.jpg",
        visualization,
    )
    write_debug_image(
        snapshot_directory / "02_raw_camera.jpg",
        raw_frame,
    )
    if black_mask is not None:
        write_debug_image(
            snapshot_directory / "03_black_frame_mask.jpg",
            black_mask,
        )
    if alignment is not None:
        coarse_debug = alignment.coarse_board.copy()
        cv2.polylines(
            coarse_debug,
            [np.round(alignment.inner_white_quad).astype(np.int32)],
            True,
            COLOR_GREEN,
            3,
            cv2.LINE_AA,
        )
        write_debug_image(
            snapshot_directory / "04_coarse_board_and_inner_white.jpg",
            coarse_debug,
        )
        write_debug_image(
            snapshot_directory / "05_inner_white_mask.jpg",
            alignment.white_mask,
        )
    if rectified is not None:
        write_debug_image(
            snapshot_directory / "06_training_aligned_board.jpg",
            rectified,
        )
    if status_roi_image is not None:
        write_debug_image(
            snapshot_directory / "07_status_roi.jpg",
            status_roi_image,
        )
    if number_roi_image is not None:
        write_debug_image(
            snapshot_directory / "08_number_roi.jpg",
            number_roi_image,
        )

    metadata = {
        key: value
        for key, value in state.items()
        if key not in {"timestamp"}
    }
    metadata["captured_at"] = timestamp.isoformat(timespec="milliseconds")
    with (snapshot_directory / "metadata.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(metadata, file, ensure_ascii=False, indent=2)

    return snapshot_directory


def main() -> None:
    args = parse_args()

    if not 1 <= args.black_threshold <= 254:
        raise ValueError("--black-threshold 必须位于 1 到 254 之间")

    status_model_path = resolve_project_path(args.status_model)
    number_model_path = resolve_project_path(args.number_model)
    if not status_model_path.is_file():
        raise FileNotFoundError(f"找不到状态模型：{status_model_path}")
    if not number_model_path.is_file():
        raise FileNotFoundError(f"找不到数字模型：{number_model_path}")

    roi_config_path = resolve_project_path(args.roi_config)
    status_config = load_yaml(resolve_project_path(args.status_config))
    number_config = load_yaml(resolve_project_path(args.number_config))
    status_image_size = int(status_config["model"]["image_size"])
    number_image_size = int(number_config["model"]["image_size"])
    status_roi = load_roi_from_config(roi_config_path, "status")
    number_roi = load_roi_from_config(roi_config_path, "number")
    output_root = resolve_project_path(args.output)
    output_root.mkdir(parents=True, exist_ok=True)

    device = resolve_device(args.device)
    status_model = YOLO(str(status_model_path))
    number_model = YOLO(str(number_model_path))
    validate_model_classes(
        status_model,
        {"idle", "busy"},
        "状态",
    )
    validate_model_classes(
        number_model,
        {"5", "6", "7", "8", "9", "10"},
        "数字",
    )

    camera_source = parse_camera_source(args.camera)
    capture = open_camera(camera_source, args.backend)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
    if not capture.isOpened():
        raise RuntimeError(
            f"无法打开摄像头或视频源：{camera_source}"
        )

    status_smoother = ProbabilitySmoother(args.smoothing_window)
    number_smoother = ProbabilitySmoother(args.smoothing_window)
    frame_id = 0
    smoothed_fps = 0.0
    paused = False
    last_auto_save_time = time.monotonic()
    consecutive_read_failures = 0
    latest_bundle: dict[str, Any] | None = None

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

    print(f"状态模型：{status_model_path}")
    print(f"数字模型：{number_model_path}")
    print(f"摄像头源：{camera_source}")
    print(
        "定位方式：黑框粗定位 + 白色内区二次对齐 "
        f"(dark_threshold={args.black_threshold})"
    )
    print(f"调试素材目录：{output_root}")
    print("按 S 保存截图，按 Space 暂停/继续，按 Q 或 Esc 退出。")

    try:
        while True:
            if not paused:
                frame_start_time = time.perf_counter()
                read_success, frame = capture.read()
                if not read_success or frame is None or frame.size == 0:
                    consecutive_read_failures += 1
                    if consecutive_read_failures >= 30:
                        raise RuntimeError("连续 30 帧无法读取摄像头画面")
                    continue
                consecutive_read_failures = 0
                frame_id += 1

                if args.mirror:
                    frame = cv2.flip(frame, 1)

                candidate, candidate_count, black_mask = find_black_frame(
                    frame,
                    dark_threshold=args.black_threshold,
                )
                alignment = None
                rectified = None
                rectified_annotated = None
                status_roi_image = None
                number_roi_image = None
                status_probabilities: dict[str, float] = {}
                number_probabilities: dict[str, float] = {}
                status_name = None
                status_confidence = None
                number_name = None
                number_confidence = None
                error_message = ""

                if candidate is not None:
                    alignment = align_board_to_training_coordinates(
                        frame,
                        candidate.points,
                        WARP_WIDTH,
                        WARP_HEIGHT,
                    )

                if alignment is not None:
                    rectified = alignment.aligned_board
                    rectified_annotated = annotate_rectified_board(
                        rectified,
                        status_roi,
                        number_roi,
                    )

                    try:
                        status_roi_image = crop_configured_roi(
                            rectified,
                            status_roi,
                        )
                        number_roi_image = crop_configured_roi(
                            rectified,
                            number_roi,
                        )
                        prepared_status = pad_and_resize_roi(
                            status_roi_image,
                            status_image_size,
                        )
                        prepared_number = pad_and_resize_roi(
                            number_roi_image,
                            number_image_size,
                        )

                        raw_status_probabilities = predict_probabilities(
                            status_model,
                            prepared_status,
                            status_image_size,
                            device,
                        )
                        raw_number_probabilities = predict_probabilities(
                            number_model,
                            prepared_number,
                            number_image_size,
                            device,
                        )
                        status_probabilities = status_smoother.update(
                            raw_status_probabilities
                        )
                        number_probabilities = number_smoother.update(
                            raw_number_probabilities
                        )
                        status_name, status_confidence = top_prediction(
                            status_probabilities
                        )
                        number_name, number_confidence = top_prediction(
                            number_probabilities
                        )
                    except Exception as error:
                        error_message = f"INFERENCE ERROR: {error}"
                        status_smoother.reset()
                        number_smoother.reset()
                else:
                    if candidate is not None:
                        error_message = "INNER WHITE ALIGNMENT FAILED"
                    status_smoother.reset()
                    number_smoother.reset()

                annotated_frame = annotate_camera_frame(
                    frame,
                    candidate,
                    candidate_count,
                    alignment,
                )

                frame_elapsed = max(
                    time.perf_counter() - frame_start_time,
                    1e-9,
                )
                instant_fps = 1.0 / frame_elapsed
                smoothed_fps = (
                    instant_fps
                    if smoothed_fps <= 0.0
                    else 0.90 * smoothed_fps + 0.10 * instant_fps
                )

                state = {
                    "timestamp": datetime.now().strftime(
                        "%Y-%m-%d %H:%M:%S.%f"
                    )[:-3],
                    "frame_id": frame_id,
                    "fps": round(smoothed_fps, 3),
                    "board_found": alignment is not None,
                    "board_score": (
                        round(float(candidate.score), 6)
                        if candidate is not None
                        else None
                    ),
                    "candidate_count": candidate_count,
                    "locator": "black_frame_then_inner_white",
                    "black_threshold": args.black_threshold,
                    "frame_area_ratio": (
                        round(float(candidate.area_ratio), 6)
                        if candidate is not None
                        else None
                    ),
                    "frame_aspect_ratio": (
                        round(float(candidate.aspect_ratio), 6)
                        if candidate is not None
                        else None
                    ),
                    "frame_child_area_ratio": (
                        round(float(candidate.child_area_ratio), 6)
                        if candidate is not None
                        else None
                    ),
                    "frame_minimum_side_continuity": (
                        round(
                            float(
                                candidate.border_features.minimum_continuity
                            ),
                            6,
                        )
                        if candidate is not None
                        else None
                    ),
                    "inner_alignment_score": (
                        round(float(alignment.score), 6)
                        if alignment is not None
                        else None
                    ),
                    "status_name": status_name,
                    "status_confidence": status_confidence,
                    "status_probabilities": status_probabilities,
                    "number_name": number_name,
                    "number_confidence": number_confidence,
                    "number_probabilities": number_probabilities,
                    "display_number": (
                        number_name if status_name == "busy" else None
                    ),
                    "error": error_message,
                }
                visualization = build_visualization(
                    annotated_frame,
                    rectified_annotated,
                    status_roi_image,
                    number_roi_image,
                    state,
                )
                latest_bundle = {
                    "visualization": visualization,
                    "raw_frame": frame.copy(),
                    "rectified": (
                        rectified.copy() if rectified is not None else None
                    ),
                    "status_roi_image": (
                        status_roi_image.copy()
                        if status_roi_image is not None
                        else None
                    ),
                    "number_roi_image": (
                        number_roi_image.copy()
                        if number_roi_image is not None
                        else None
                    ),
                    "black_mask": black_mask.copy(),
                    "alignment": alignment,
                    "state": state,
                }

                if (
                    args.auto_save_interval > 0.0
                    and time.monotonic() - last_auto_save_time
                    >= args.auto_save_interval
                ):
                    snapshot_directory = save_debug_snapshot(
                        output_root,
                        **latest_bundle,
                    )
                    last_auto_save_time = time.monotonic()
                    print(f"自动保存调试素材：{snapshot_directory}")

            if latest_bundle is None:
                continue

            display = latest_bundle["visualization"].copy()
            if paused:
                cv2.rectangle(
                    display,
                    (665, 18),
                    (930, 60),
                    (15, 15, 15),
                    thickness=-1,
                )
                draw_text(
                    display,
                    "PAUSED",
                    (735, 50),
                    0.95,
                    COLOR_YELLOW,
                    2,
                )

            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(30 if paused else 1) & 0xFF

            if key in {ord("q"), ord("Q"), 27}:
                break
            if key == 32:
                paused = not paused
                print("已暂停" if paused else "继续实时检测")
            elif key in {ord("s"), ord("S")}:
                snapshot_directory = save_debug_snapshot(
                    output_root,
                    visualization=display,
                    raw_frame=latest_bundle["raw_frame"],
                    rectified=latest_bundle["rectified"],
                    status_roi_image=latest_bundle["status_roi_image"],
                    number_roi_image=latest_bundle["number_roi_image"],
                    black_mask=latest_bundle["black_mask"],
                    alignment=latest_bundle["alignment"],
                    state=latest_bundle["state"],
                )
                print(f"已保存调试素材：{snapshot_directory}")
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
