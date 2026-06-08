"""Smoke-test YOLO weights with an Intel RealSense color stream.

Run from the repository root:

    python src/macgyvbot_resources/weights/test_yolo_realsense.py

By default this script looks for:

    src/macgyvbot_resources/weights/yolov11_best.pt

The RealSense profile defaults match the project README camera profile:
640x480 color at 30 FPS.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "macgyvbot_config"))

from macgyvbot_config.models import YOLO_MODEL_NAME

DEFAULT_MODEL_NAME = YOLO_MODEL_NAME
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLO inference on a live RealSense RGB stream."
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL_NAME,
        help=(
            "YOLO weight path or filename. Relative filenames are resolved "
            "from this weights directory, the current directory, and the "
            "installed macgyvbot_resources share directory."
        ),
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="YOLO confidence threshold.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="YOLO inference image size.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after this many frames. 0 means run until q or Ctrl+C.",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Do not open an OpenCV preview window.",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=30,
        help="Log detection summaries every N frames.",
    )
    return parser.parse_args()


def package_share(package_name: str) -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        return None

    try:
        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


def resolve_model_path(model_name: str) -> Path:
    requested = Path(model_name).expanduser()
    weights_dir = Path(__file__).resolve().parent
    cwd = Path.cwd()
    share = package_share("macgyvbot_resources")

    if requested.is_absolute() or requested.exists():
        return requested

    candidates = [
        weights_dir / requested,
        cwd / requested,
        cwd / "weights" / requested,
    ]

    if share is not None:
        candidates.extend([share / requested, share / "weights" / requested])

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return weights_dir / requested


def summarize_detections(result, names: dict[int, str]) -> str:
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return "no detections"

    labels = []
    for box in boxes:
        cls_id = int(box.cls[0])
        confidence = float(box.conf[0])
        label = names.get(cls_id, str(cls_id))
        labels.append(f"{label}:{confidence:.2f}")
    return ", ".join(labels)


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(message)s",
    )
    logger = logging.getLogger("test_yolo_realsense")

    model_path = resolve_model_path(args.model)
    if not model_path.exists():
        logger.error("YOLO weight file not found: %s", model_path)
        logger.error(
            "Place weights at src/macgyvbot_resources/weights/%s "
            "or pass --model /path/to/model.pt.",
            DEFAULT_MODEL_NAME,
        )
        return 2

    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        from ultralytics import YOLO
    except ImportError as exc:
        logger.error("Missing dependency: %s", exc)
        logger.error(
            "Install project requirements with: "
            "python -m pip install -r requirements.txt"
        )
        return 2

    logger.info("Loading YOLO weights: %s", model_path)
    model = YOLO(str(model_path))

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.bgr8,
        args.fps,
    )

    try:
        logger.info(
            "Starting RealSense color stream: %dx%d@%d",
            args.width,
            args.height,
            args.fps,
        )
        pipeline.start(config)
    except Exception as exc:
        logger.error("Failed to start RealSense camera: %s", exc)
        logger.error(
            "Check USB connection and close other RealSense users before retrying."
        )
        return 2

    frame_count = 0
    started_at = time.monotonic()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                logger.warning("RealSense frame received without color data.")
                continue

            color_image = np.asanyarray(color_frame.get_data())
            results = model.predict(
                color_image,
                conf=args.conf,
                imgsz=args.imgsz,
                verbose=False,
            )
            result = results[0]
            frame_count += 1

            if args.log_every > 0 and frame_count % args.log_every == 0:
                elapsed = max(time.monotonic() - started_at, 1e-6)
                fps = frame_count / elapsed
                logger.info(
                    "frame=%d fps=%.1f detections=%s",
                    frame_count,
                    fps,
                    summarize_detections(result, model.names),
                )

            if not args.no_display:
                annotated = result.plot()
                cv2.imshow("MacGyvBot YOLO RealSense Test", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Stopped by user.")
                    break

            if args.max_frames > 0 and frame_count >= args.max_frames:
                logger.info("Reached --max-frames=%d.", args.max_frames)
                break
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    finally:
        pipeline.stop()
        if not args.no_display:
            cv2.destroyAllWindows()

    logger.info("Finished after %d frame(s).", frame_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
