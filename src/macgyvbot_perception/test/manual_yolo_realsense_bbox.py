#!/usr/bin/env python3
"""Smoke-test YOLO bbox output on a live Intel RealSense color stream.

Run from the repository root:

    python src/macgyvbot_perception/test/manual_yolo_realsense_bbox.py \
        --model yolov11_best.pt

This is a manual diagnostic helper, not a pytest test. It opens the RealSense
color stream directly and displays YOLO annotations. Press q in the preview
window to quit.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path


DEFAULT_MODEL_NAME = "yolov11_best.pt"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30


def add_workspace_packages_to_path() -> None:
    workspace_src = Path(__file__).resolve().parents[2]
    for package_dir in workspace_src.iterdir():
        if package_dir.is_dir() and str(package_dir) not in sys.path:
            sys.path.insert(0, str(package_dir))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLO bbox inference on a live RealSense RGB stream."
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL_NAME,
        help=(
            "YOLO weight path or filename. Relative filenames are resolved "
            "through macgyvbot_resources weight lookup."
        ),
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
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
        help="Log bbox summaries every N frames.",
    )
    return parser.parse_args()


def detection_records(result, names: dict[int, str]) -> list[dict]:
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []

    records = []
    for index, box in enumerate(boxes):
        cls_id = int(box.cls[0])
        confidence = float(box.conf[0])
        label = names.get(cls_id, str(cls_id))
        x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].cpu().numpy()]
        records.append(
            {
                "index": index,
                "label": label,
                "class_id": cls_id,
                "confidence": confidence,
                "bbox_xyxy": (x1, y1, x2, y2),
            }
        )
    return records


def summarize_detections(records: list[dict]) -> str:
    if not records:
        return "no detections"

    parts = []
    for record in records:
        x1, y1, x2, y2 = record["bbox_xyxy"]
        parts.append(
            f"{record['label']}:{record['confidence']:.2f} "
            f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})"
        )
    return "; ".join(parts)


def main() -> int:
    add_workspace_packages_to_path()
    args = parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("manual_yolo_realsense_bbox")

    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        from ultralytics import YOLO

        from macgyvbot_perception.yolo_detector import resolve_model_path
    except ImportError as exc:
        logger.error("Missing dependency: %s", exc)
        logger.error("Install requirements with: python -m pip install -r requirements.txt")
        return 2

    model_path = Path(resolve_model_path(args.model)).expanduser()
    if not model_path.exists():
        logger.error("YOLO weight file not found: %s", model_path)
        logger.error(
            "Place weights under src/macgyvbot_resources/weights "
            "or pass --model /path/to/model.pt."
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
        logger.error("Check USB connection and close other RealSense users.")
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
            records = detection_records(result, model.names)
            frame_count += 1

            elapsed = max(time.monotonic() - started_at, 1e-6)
            fps = frame_count / elapsed
            if args.log_every > 0 and frame_count % args.log_every == 0:
                logger.info(
                    "frame=%d fps=%.1f detections=%s",
                    frame_count,
                    fps,
                    summarize_detections(records),
                )

            if not args.no_display:
                annotated = result.plot()
                cv2.putText(
                    annotated,
                    f"frame={frame_count} fps={fps:.1f} detections={len(records)}",
                    (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    annotated,
                    "q: quit",
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.imshow("MacGyvBot YOLO RealSense BBox Test", annotated)
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
    raise SystemExit(main())
