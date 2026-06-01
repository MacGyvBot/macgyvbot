"""Collect RealSense + YOLO frames for VLM grasp tuning.

Run from the repository root:

    python src/macgyvbot_perception/data/collect_vlm_grasp_dataset.py

Controls:
    s  save the current frame, annotation, detections, and bbox crops
    q  quit

Saved samples go under:

    src/macgyvbot_perception/data/grasp_dataset/<session_id>/
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path


DEFAULT_MODEL_NAME = "yolov11_best.pt"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30
DEFAULT_SAM_CHECKPOINT = "mobile_sam_FT.pt"
DATA_ROOT = Path(__file__).resolve().parent / "grasp_dataset"


def add_workspace_packages_to_path() -> None:
    workspace_src = Path(__file__).resolve().parents[2]
    for package_dir in workspace_src.iterdir():
        if package_dir.is_dir() and str(package_dir) not in sys.path:
            sys.path.insert(0, str(package_dir))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect RealSense frames and YOLO crops for VLM grasp tuning."
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL_NAME,
        help=(
            "YOLO weight path or filename. Relative filenames are resolved "
            "from macgyvbot_resources/weights and common workspace locations."
        ),
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--sam-checkpoint",
        default=DEFAULT_SAM_CHECKPOINT,
        help="SAM checkpoint path or weight filename.",
    )
    parser.add_argument(
        "--sam-backend",
        default="mobile_sam",
        choices=("mobile_sam", "sam"),
        help="SAM implementation used to create overlay images.",
    )
    parser.add_argument(
        "--sam-model-type",
        default="vit_t",
        help="SAM model type, e.g. vit_t for mobile_sam or vit_b for SAM.",
    )
    parser.add_argument(
        "--sam-device",
        default="cuda",
        help="Device for SAM inference.",
    )
    parser.add_argument(
        "--no-sam",
        action="store_true",
        help="Disable SAM overlay generation.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DATA_ROOT,
        help="Dataset root directory.",
    )
    parser.add_argument(
        "--session",
        default="",
        help="Optional session id. Defaults to a timestamp.",
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
    cwd = Path.cwd()
    repo_root = Path(__file__).resolve().parents[3]
    resources_weights = repo_root / "macgyvbot_resources" / "weights"
    share = package_share("macgyvbot_resources")

    if requested.is_absolute() or requested.exists():
        return requested

    candidates = [
        resources_weights / requested,
        cwd / requested,
        cwd / "weights" / requested,
        cwd / "src" / "macgyvbot_resources" / "weights" / requested,
    ]

    if share is not None:
        candidates.extend([share / requested, share / "weights" / requested])

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return resources_weights / requested


def detection_records(result, names: dict[int, str]) -> list[dict]:
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []

    records = []
    for index, box in enumerate(boxes):
        cls_id = int(box.cls[0])
        confidence = float(box.conf[0])
        label = names.get(cls_id, str(cls_id))
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].cpu().numpy()]
        records.append(
            {
                "index": index,
                "label": label,
                "class_id": cls_id,
                "confidence": confidence,
                "bbox_xyxy": [x1, y1, x2, y2],
            }
        )
    return records


def clamp_bbox(record: dict, width: int, height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = record["bbox_xyxy"]
    x1 = max(0, min(width - 1, int(round(x1))))
    y1 = max(0, min(height - 1, int(round(y1))))
    x2 = max(0, min(width, int(round(x2))))
    y2 = max(0, min(height, int(round(y2))))
    return x1, y1, x2, y2


def make_sam_segmenter(args, logger):
    if args.no_sam:
        return None

    try:
        from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import (
            BBoxPromptSegmenter,
        )
    except ImportError as exc:
        logger.warning("SAM import failed. Overlay images disabled: %s", exc)
        return None

    try:
        return BBoxPromptSegmenter(
            backend=args.sam_backend,
            checkpoint_path=args.sam_checkpoint,
            model_type=args.sam_model_type,
            device=args.sam_device,
        )
    except Exception as exc:
        logger.warning("SAM init failed. Overlay images disabled: %s", exc)
        return None


def overlay_mask(cv2, frame, mask):
    green = frame.copy()
    green[:, :] = (0, 255, 0)
    mask = mask.astype(bool)
    frame[mask] = cv2.addWeighted(frame, 0.65, green, 0.35, 0)[mask]


def save_sam_overlay(
    *,
    cv2,
    segmenter,
    color_image,
    record: dict,
    sample_dir: Path,
    crop_dir: Path,
    width: int,
    height: int,
) -> dict:
    overlay_record = {
        "sam_available": segmenter is not None,
        "sam_source": None,
        "sam_overlay_path": None,
        "sam_crop_path": None,
        "sam_error": "",
    }
    if segmenter is None:
        return overlay_record

    roi = clamp_bbox(record, width, height)
    try:
        mask = segmenter.segment(color_image, roi)
    except Exception as exc:
        overlay_record["sam_error"] = str(exc)
        return overlay_record

    if mask is None or int(mask.sum()) <= 0:
        overlay_record["sam_error"] = "empty_mask"
        return overlay_record

    overlay = color_image.copy()
    overlay_mask(cv2, overlay, mask)

    overlay_name = (
        f"{record['index']:02d}_{record['label']}_"
        f"{record['confidence']:.2f}_sam_overlay.png"
    )
    overlay_path = sample_dir / overlay_name
    cv2.imwrite(str(overlay_path), overlay)

    x1, y1, x2, y2 = roi
    sam_crop = overlay[y1:y2, x1:x2]
    sam_crop_name = (
        f"{record['index']:02d}_{record['label']}_"
        f"{record['confidence']:.2f}_sam_crop.png"
    )
    sam_crop_path = crop_dir / sam_crop_name
    cv2.imwrite(str(sam_crop_path), sam_crop)

    overlay_record.update(
        {
            "sam_source": "SAM_BBOX_PROMPT",
            "sam_overlay_path": str(overlay_path.relative_to(sample_dir)),
            "sam_crop_path": str(sam_crop_path.relative_to(sample_dir)),
        }
    )
    return overlay_record


def save_sample(
    *,
    cv2,
    segmenter,
    color_image,
    annotated_image,
    detections: list[dict],
    model_path: Path,
    output_dir: Path,
    frame_count: int,
) -> Path:
    sample_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    sample_dir = output_dir / sample_id
    crop_dir = sample_dir / "crops"
    crop_dir.mkdir(parents=True, exist_ok=True)

    image_path = sample_dir / "image_bgr.png"
    annotated_path = sample_dir / "annotated.png"
    metadata_path = sample_dir / "metadata.json"

    cv2.imwrite(str(image_path), color_image)
    cv2.imwrite(str(annotated_path), annotated_image)

    height, width = color_image.shape[:2]
    saved_crops = []
    for record in detections:
        x1, y1, x2, y2 = clamp_bbox(record, width, height)
        if x2 <= x1 or y2 <= y1:
            continue

        crop = color_image[y1:y2, x1:x2]
        crop_name = (
            f"{record['index']:02d}_{record['label']}_"
            f"{record['confidence']:.2f}.png"
        )
        crop_path = crop_dir / crop_name
        cv2.imwrite(str(crop_path), crop)
        crop_record = dict(record)
        crop_record["crop_path"] = str(crop_path.relative_to(sample_dir))
        crop_record.update(
            save_sam_overlay(
                cv2=cv2,
                segmenter=segmenter,
                color_image=color_image,
                record=record,
                sample_dir=sample_dir,
                crop_dir=crop_dir,
                width=width,
                height=height,
            )
        )
        saved_crops.append(crop_record)

    metadata = {
        "sample_id": sample_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "frame_count": frame_count,
        "model_path": str(model_path),
        "image_path": str(image_path.relative_to(sample_dir)),
        "annotated_path": str(annotated_path.relative_to(sample_dir)),
        "detections": detections,
        "saved_crops": saved_crops,
        "qwen_grasp_output": {
            "x_px": None,
            "y_px": None,
            "yaw_deg": None,
            "confidence": None,
            "reason": "",
        },
    }
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    index_path = output_dir / "metadata.jsonl"
    with index_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(metadata, ensure_ascii=False) + "\n")

    return sample_dir


def main() -> int:
    add_workspace_packages_to_path()
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("collect_vlm_grasp_dataset")

    model_path = resolve_model_path(args.model)
    if not model_path.exists():
        logger.error("YOLO weight file not found: %s", model_path)
        return 2

    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        from ultralytics import YOLO
    except ImportError as exc:
        logger.error("Missing dependency: %s", exc)
        logger.error("Install requirements with: python -m pip install -r requirements.txt")
        return 2

    session_id = args.session or datetime.now().strftime("session_%Y%m%d_%H%M%S")
    output_dir = args.output_dir.expanduser().resolve() / session_id
    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("Loading YOLO weights: %s", model_path)
    model = YOLO(str(model_path))
    segmenter = make_sam_segmenter(args, logger)
    if segmenter is None:
        logger.info("SAM overlay generation is disabled.")
    else:
        logger.info("SAM overlay generation is enabled.")
    logger.info("Saving dataset samples under: %s", output_dir)
    logger.info("Press 's' in the preview window to save, 'q' to quit.")

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
            annotated = result.plot()
            detections = detection_records(result, model.names)
            frame_count += 1

            elapsed = max(time.monotonic() - started_at, 1e-6)
            fps = frame_count / elapsed
            cv2.putText(
                annotated,
                "s: save  q: quit",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                f"frame={frame_count} fps={fps:.1f} detections={len(detections)}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow("MacGyvBot VLM Dataset Collector", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logger.info("Stopped by user.")
                break
            if key == ord("s"):
                sample_dir = save_sample(
                    cv2=cv2,
                    segmenter=segmenter,
                    color_image=color_image,
                    annotated_image=annotated,
                    detections=detections,
                    model_path=model_path,
                    output_dir=output_dir,
                    frame_count=frame_count,
                )
                logger.info(
                    "Saved sample: %s (%d detection(s))",
                    sample_dir,
                    len(detections),
                )
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    logger.info("Finished after %d frame(s).", frame_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
