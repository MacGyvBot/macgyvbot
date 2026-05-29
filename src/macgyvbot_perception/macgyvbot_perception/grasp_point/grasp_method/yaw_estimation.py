"""Shared SAM/PCA yaw estimation helpers for grasp selectors."""

from __future__ import annotations

import logging
import math
from copy import deepcopy
from typing import Any

import cv2
import numpy as np

from macgyvbot_config.vlm import PCA_YAW_SAM_DEFAULT_CONFIG
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import BBoxPromptSegmenter


def default_pca_yaw_config() -> dict[str, Any]:
    """Return a mutable copy of the default PCA-yaw configuration."""
    return deepcopy(PCA_YAW_SAM_DEFAULT_CONFIG)


def normalize_parallel_gripper_yaw(angle_deg: float) -> float:
    """Normalize a relative yaw angle to the closest gripper-equivalent value."""
    angle = ((float(angle_deg) + 180.0) % 360.0) - 180.0
    normalized = ((angle + 90.0) % 180.0) - 90.0
    if math.isclose(normalized, -90.0, abs_tol=1e-6) and angle > 0.0:
        normalized = 90.0
    if math.isclose(abs(normalized), 0.0, abs_tol=1e-6):
        normalized = 0.0
    return float(normalized)


def estimate_yaw_from_mask(mask: np.ndarray) -> tuple[float, dict[str, Any]]:
    """Estimate tool yaw from a binary mask using PCA on object pixels."""
    debug_info: dict[str, Any] = {
        "success": False,
        "method": "pca_mask",
        "num_pixels": 0,
        "raw_angle_deg": None,
        "normalized_yaw_deg": None,
        "eigenvector": None,
        "reason": "",
    }

    if mask is None:
        debug_info["reason"] = "mask_is_none"
        return 0.0, debug_info
    if not isinstance(mask, np.ndarray) or mask.ndim != 2:
        debug_info["reason"] = "mask_must_be_2d_array"
        return 0.0, debug_info

    ys, xs = np.where(mask > 0)
    num_pixels = int(xs.size)
    debug_info["num_pixels"] = num_pixels
    if num_pixels < int(PCA_YAW_SAM_DEFAULT_CONFIG["min_pca_pixels"]):
        debug_info["reason"] = "mask_has_too_few_pixels"
        return 0.0, debug_info

    coords = np.column_stack((xs, ys)).astype(np.float32)
    centered = coords - coords.mean(axis=0)
    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    principal = eigenvectors[:, int(np.argmax(eigenvalues))]
    raw_angle_deg = math.degrees(
        math.atan2(-float(principal[0]), float(principal[1]))
    )
    normalized_yaw_deg = normalize_parallel_gripper_yaw(raw_angle_deg)

    debug_info.update(
        {
            "success": True,
            "raw_angle_deg": float(raw_angle_deg),
            "normalized_yaw_deg": float(normalized_yaw_deg),
            "eigenvector": [float(principal[0]), float(principal[1])],
            "reason": "ok",
        }
    )
    return float(normalized_yaw_deg), debug_info


def estimate_sam_mask_for_crop(
    crop_bgr: np.ndarray,
    sam_predictor=None,
    bbox_prompt=None,
    grasp_point=None,
    config: dict | None = None,
) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Run SAM on one crop image and select the best tool mask candidate."""
    cfg = _merge_config(config)
    debug_info: dict[str, Any] = {
        "success": False,
        "method": "sam_single_crop",
        "sam_num_masks": 0,
        "selected_mask_area": 0,
        "selected_mask_score": None,
        "mask_area_ratio": 0.0,
        "grasp_point_inside_mask": False,
        "reason": "",
    }

    if crop_bgr is None or not isinstance(crop_bgr, np.ndarray) or crop_bgr.ndim < 2:
        debug_info["reason"] = "invalid_crop_image"
        _log(cfg, "warning", f"SAM mask selection failed: {debug_info['reason']}")
        return None, debug_info
    predictor = _resolve_predictor(sam_predictor)
    if predictor is None:
        debug_info["reason"] = "sam_predictor_unavailable"
        _log(cfg, "warning", f"SAM mask selection failed: {debug_info['reason']}")
        return None, debug_info

    height, width = crop_bgr.shape[:2]
    if height <= 0 or width <= 0:
        debug_info["reason"] = "empty_crop_image"
        _log(cfg, "warning", f"SAM mask selection failed: {debug_info['reason']}")
        return None, debug_info

    box = _normalize_bbox_prompt(bbox_prompt, width, height)
    point_coords, point_labels = _build_point_prompt(grasp_point, width, height)

    try:
        predictor.set_image(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
        masks, scores, _ = predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            box=np.asarray(box, dtype=np.float32),
            multimask_output=True,
        )
    except Exception as exc:
        debug_info["reason"] = f"sam_predict_failed:{exc}"
        _log(cfg, "warning", f"SAM mask selection failed: {debug_info['reason']}")
        return None, debug_info

    if masks is None or len(masks) == 0:
        debug_info["reason"] = "sam_returned_no_masks"
        _log(cfg, "warning", f"SAM mask selection failed: {debug_info['reason']}")
        return None, debug_info

    crop_area = float(max(1, width * height))
    debug_info["sam_num_masks"] = int(len(masks))
    min_ratio = float(cfg["min_mask_area_ratio"])
    max_ratio = float(cfg["max_mask_area_ratio"])
    grasp_point_xy = _coerce_point(grasp_point)
    crop_center = np.array([width / 2.0, height / 2.0], dtype=np.float32)

    best_mask = None
    best_candidate = None
    for index, mask in enumerate(masks):
        candidate_mask = _to_binary_mask(mask, (height, width))
        if candidate_mask is None:
            continue
        candidate_mask = _keep_largest_component(candidate_mask)
        area = int(candidate_mask.sum())
        ratio = area / crop_area
        if ratio < min_ratio or ratio > max_ratio:
            continue

        mask_score = float(scores[index]) if scores is not None and index < len(scores) else 0.0
        inside = _point_inside_mask(candidate_mask, grasp_point_xy)
        centroid = _mask_centroid(candidate_mask)
        center_dist = _normalized_distance(centroid, crop_center, width, height)
        selection_score = mask_score + (0.2 if inside else 0.0) - (0.05 * center_dist)

        candidate = {
            "mask": candidate_mask,
            "area": area,
            "ratio": ratio,
            "score": mask_score,
            "inside": inside,
            "selection_score": selection_score,
        }
        if best_candidate is None or candidate["selection_score"] > best_candidate["selection_score"]:
            best_candidate = candidate
            best_mask = candidate_mask

    if best_candidate is None or best_mask is None:
        debug_info["reason"] = "no_mask_passed_selection"
        _log(cfg, "warning", f"SAM mask selection failed: {debug_info['reason']}")
        return None, debug_info

    debug_info.update(
        {
            "success": True,
            "selected_mask_area": int(best_candidate["area"]),
            "selected_mask_score": float(best_candidate["score"]),
            "mask_area_ratio": float(best_candidate["ratio"]),
            "grasp_point_inside_mask": bool(best_candidate["inside"]),
            "reason": "ok",
        }
    )
    _log(
        cfg,
        "info",
        "SAM mask selected: "
        f"success=True area={debug_info['selected_mask_area']} "
        f"score={debug_info['selected_mask_score']:.3f} "
        f"ratio={debug_info['mask_area_ratio']:.3f} "
        f"point_inside={debug_info['grasp_point_inside_mask']}",
    )
    return best_mask.astype(np.uint8), debug_info


def aggregate_masks_across_frames(
    masks: list[np.ndarray],
    config: dict | None = None,
) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Aggregate per-frame masks into one stable mask."""
    cfg = _merge_config(config)
    debug_info: dict[str, Any] = {
        "success": False,
        "method": "sam_mask_aggregation",
        "num_input_masks": len(masks or []),
        "num_valid_masks": 0,
        "aggregation_mode": str(cfg["aggregation_mode"]),
        "vote_threshold": float(cfg["mask_vote_threshold"]),
        "final_mask_area": 0,
        "final_mask_area_ratio": 0.0,
        "reason": "",
    }

    if not masks:
        debug_info["reason"] = "no_input_masks"
        _log(cfg, "warning", f"Mask aggregation failed: {debug_info['reason']}")
        return None, debug_info

    first_shape = None
    valid_masks: list[np.ndarray] = []
    for mask in masks:
        binary_mask = _to_binary_mask(mask)
        if binary_mask is None or int(binary_mask.sum()) <= 0:
            continue
        if first_shape is None:
            first_shape = binary_mask.shape
        if binary_mask.shape != first_shape:
            binary_mask = cv2.resize(
                binary_mask.astype(np.uint8),
                (first_shape[1], first_shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        valid_masks.append(binary_mask.astype(bool))

    debug_info["num_valid_masks"] = len(valid_masks)
    if not valid_masks:
        debug_info["reason"] = "no_valid_masks"
        _log(cfg, "warning", f"Mask aggregation failed: {debug_info['reason']}")
        return None, debug_info
    if len(valid_masks) < int(cfg["min_valid_masks"]):
        debug_info["reason"] = "insufficient_valid_masks"
        _log(cfg, "warning", f"Mask aggregation failed: {debug_info['reason']}")
        return None, debug_info

    stack = np.stack(valid_masks, axis=0)
    mode = str(cfg["aggregation_mode"]).strip().lower()
    if mode == "union":
        aggregated = np.any(stack, axis=0)
    elif mode == "intersection":
        aggregated = np.all(stack, axis=0)
    else:
        mode = "majority"
        threshold = float(cfg["mask_vote_threshold"])
        aggregated = stack.mean(axis=0) >= threshold
        debug_info["vote_threshold"] = threshold
    debug_info["aggregation_mode"] = mode

    aggregated = _apply_morphology(aggregated.astype(np.uint8), cfg).astype(bool)
    aggregated = _keep_largest_component(aggregated)
    final_area = int(aggregated.sum())
    total_area = float(max(1, aggregated.shape[0] * aggregated.shape[1]))
    final_ratio = final_area / total_area
    debug_info["final_mask_area"] = final_area
    debug_info["final_mask_area_ratio"] = float(final_ratio)

    if final_area <= 0:
        debug_info["reason"] = "aggregated_mask_empty"
        _log(cfg, "warning", f"Mask aggregation failed: {debug_info['reason']}")
        return None, debug_info
    if final_ratio < float(cfg["min_mask_area_ratio"]) or final_ratio > float(cfg["max_mask_area_ratio"]):
        debug_info["reason"] = "aggregated_mask_area_ratio_out_of_range"
        _log(cfg, "warning", f"Mask aggregation failed: {debug_info['reason']}")
        return None, debug_info

    debug_info["success"] = True
    debug_info["reason"] = "ok"
    _log(
        cfg,
        "info",
        "Mask aggregation result: "
        f"success=True valid_masks={debug_info['num_valid_masks']} "
        f"mode={mode} area_ratio={final_ratio:.3f}",
    )
    return aggregated.astype(np.uint8), debug_info


def estimate_yaw_from_multi_frame_sam(
    crop_frames_bgr: list[np.ndarray] | np.ndarray,
    sam_predictor=None,
    grasp_point=None,
    config: dict | None = None,
) -> tuple[float, dict[str, Any]]:
    """Estimate yaw from multiple crop frames using SAM aggregation plus PCA."""
    cfg = _merge_config(config)
    frames = _coerce_frames(crop_frames_bgr)
    debug_info: dict[str, Any] = {
        "success": False,
        "method": "multi_frame_sam_pca_yaw",
        "num_frames": len(frames),
        "per_frame_sam_debug": [],
        "aggregation_debug": {},
        "pca_debug": {},
        "normalized_yaw_deg": 0.0,
        "reason": "",
    }

    if not frames:
        debug_info["reason"] = "no_frames"
        _log(cfg, "warning", f"Multi-frame PCA yaw failed: {debug_info['reason']}")
        return 0.0, debug_info

    masks: list[np.ndarray | None] = []
    for index, frame in enumerate(frames):
        mask, frame_debug = estimate_sam_mask_for_crop(
            frame,
            sam_predictor=sam_predictor,
            grasp_point=grasp_point,
            config=cfg,
        )
        masks.append(mask)
        debug_info["per_frame_sam_debug"].append(frame_debug)
        _log(
            cfg,
            "info",
            f"SAM frame[{index}] success={frame_debug['success']} reason={frame_debug['reason']}",
        )

    aggregated_mask, aggregation_debug = aggregate_masks_across_frames(
        masks,
        config=cfg,
    )
    debug_info["aggregation_debug"] = aggregation_debug
    _log(
        cfg,
        "info",
        "SAM aggregation summary: "
        f"success={aggregation_debug.get('success')} "
        f"area_ratio={aggregation_debug.get('final_mask_area_ratio')} "
        f"reason={aggregation_debug.get('reason')}",
    )
    if aggregated_mask is None or not aggregation_debug.get("success"):
        debug_info["reason"] = aggregation_debug.get("reason", "aggregation_failed")
        return 0.0, debug_info

    yaw_deg, pca_debug = estimate_yaw_from_mask(aggregated_mask)
    debug_info["pca_debug"] = pca_debug
    _log(
        cfg,
        "info",
        "PCA yaw summary: "
        f"success={pca_debug.get('success')} "
        f"raw_angle={pca_debug.get('raw_angle_deg')} "
        f"normalized_angle={pca_debug.get('normalized_yaw_deg')}",
    )
    if not pca_debug.get("success"):
        debug_info["reason"] = pca_debug.get("reason", "pca_failed")
        return 0.0, debug_info

    debug_info["success"] = True
    debug_info["normalized_yaw_deg"] = float(yaw_deg)
    debug_info["reason"] = "ok"
    _log(cfg, "info", f"Multi-frame PCA yaw={yaw_deg:.2f}deg")
    return float(yaw_deg), debug_info


def apply_pca_yaw_to_grasp_result(
    grasp_result: dict,
    crop_frames_bgr: list[np.ndarray] | np.ndarray,
    sam_predictor=None,
    config: dict | None = None,
) -> tuple[dict, dict[str, Any]]:
    """Replace input yaw with SAM/PCA yaw while preserving response fields."""
    cfg = _merge_config(config)
    final_result = dict(grasp_result or {})
    frames = _coerce_frames(crop_frames_bgr)
    fallback_yaw = _safe_float(final_result.get("yaw_deg"))
    grasp_point = _coerce_point((final_result.get("x_px"), final_result.get("y_px")))

    debug_info: dict[str, Any] = {
        "success": True,
        "method": "apply_pca_yaw_to_grasp_result",
        "vlm_yaw_deg": fallback_yaw,
        "multi_frame_debug": {},
        "single_frame_debug": [],
        "pca_yaw_deg": None,
        "final_yaw_deg": None,
        "sam_success": False,
        "aggregation_success": False,
        "pca_success": False,
        "fallback_used": False,
        "fallback_source": None,
        "fallback_reason": "",
        "invert_yaw_sign_applied": bool(cfg["invert_yaw_sign"]),
    }

    _log(cfg, "info", f"VLM yaw before PCA override: {fallback_yaw}")

    selected_yaw = None
    multi_yaw, multi_debug = estimate_yaw_from_multi_frame_sam(
        frames,
        sam_predictor=sam_predictor,
        grasp_point=grasp_point,
        config=cfg,
    )
    debug_info["multi_frame_debug"] = multi_debug
    debug_info["sam_success"] = any(
        bool(item.get("success")) for item in multi_debug.get("per_frame_sam_debug", [])
    )
    debug_info["aggregation_success"] = bool(
        multi_debug.get("aggregation_debug", {}).get("success")
    )
    debug_info["pca_success"] = bool(multi_debug.get("pca_debug", {}).get("success"))

    if multi_debug.get("success"):
        selected_yaw = float(multi_yaw)
        debug_info["pca_yaw_deg"] = selected_yaw
    else:
        for index, frame in enumerate(frames):
            mask, sam_debug = estimate_sam_mask_for_crop(
                frame,
                sam_predictor=sam_predictor,
                grasp_point=grasp_point,
                config=cfg,
            )
            frame_debug: dict[str, Any] = {"sam_debug": sam_debug, "pca_debug": {}}
            if mask is not None and sam_debug.get("success"):
                single_yaw, pca_debug = estimate_yaw_from_mask(mask)
                frame_debug["pca_debug"] = pca_debug
                if pca_debug.get("success"):
                    selected_yaw = float(single_yaw)
                    debug_info["pca_yaw_deg"] = selected_yaw
                    debug_info["pca_success"] = True
                    debug_info["sam_success"] = True
                    debug_info["fallback_used"] = True
                    debug_info["fallback_source"] = "single_frame_pca"
                    debug_info["fallback_reason"] = multi_debug.get(
                        "reason",
                        "multi_frame_failed",
                    )
                    debug_info["single_frame_debug"].append(frame_debug)
                    _log(
                        cfg,
                        "info",
                        f"Single-frame PCA fallback succeeded on frame[{index}]: yaw={single_yaw:.2f}deg",
                    )
                    break
            debug_info["single_frame_debug"].append(frame_debug)

    if selected_yaw is None:
        if bool(cfg["fallback_to_vlm_yaw"]) and fallback_yaw is not None:
            selected_yaw = float(fallback_yaw)
            debug_info["fallback_used"] = True
            debug_info["fallback_source"] = "input_yaw"
            debug_info["fallback_reason"] = _first_non_empty_reason(
                multi_debug.get("reason"),
                "pca_unavailable",
            )
        else:
            selected_yaw = 0.0
            debug_info["fallback_used"] = True
            debug_info["fallback_source"] = "default_zero"
            debug_info["fallback_reason"] = _first_non_empty_reason(
                multi_debug.get("reason"),
                "no_valid_input_yaw",
            )

    if bool(cfg["invert_yaw_sign"]):
        selected_yaw = normalize_parallel_gripper_yaw(-float(selected_yaw))
    else:
        selected_yaw = normalize_parallel_gripper_yaw(float(selected_yaw))

    final_result["yaw_deg"] = float(selected_yaw)
    final_result["x_px"] = final_result.get("x_px")
    final_result["y_px"] = final_result.get("y_px")
    final_result["confidence"] = final_result.get("confidence")
    final_result["reason"] = final_result.get("reason")

    debug_info["final_yaw_deg"] = float(selected_yaw)
    _log(
        cfg,
        "info",
        "Final PCA yaw application: "
        f"multi_frame_pca={debug_info['pca_yaw_deg']} "
        f"final_yaw={selected_yaw:.2f} "
        f"fallback_used={debug_info['fallback_used']} "
        f"fallback_source={debug_info['fallback_source']} "
        f"invert_yaw_sign={cfg['invert_yaw_sign']}",
    )
    return final_result, debug_info


class SamPcaYawRefiner:
    """Lazy SAM-backed yaw refiner shared across grasp selectors."""

    def __init__(
        self,
        logger,
        *,
        sam_enabled: bool = True,
        sam_checkpoint: str = "",
        sam_backend: str = "mobile_sam",
        sam_model_type: str = "vit_t",
        sam_device: str = "cuda",
        config: dict[str, Any] | None = None,
    ) -> None:
        self.logger = logger
        self.sam_enabled = sam_enabled
        self.sam_checkpoint = sam_checkpoint
        self.sam_backend = sam_backend
        self.sam_model_type = sam_model_type
        self.sam_device = sam_device
        self.config = default_pca_yaw_config()
        if config:
            self.config.update(config)
        self._sam_segmenter = None
        self._sam_init_attempted = False

    def refine(
        self,
        grasp_result: dict[str, Any],
        crop_frames_bgr: list[np.ndarray] | np.ndarray,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Apply PCA yaw override with a lazily loaded SAM predictor."""
        config = {**self.config, "logger": self.logger}
        return apply_pca_yaw_to_grasp_result(
            grasp_result,
            crop_frames_bgr,
            sam_predictor=self.sam_predictor(),
            config=config,
        )

    def sam_predictor(self):
        """Return the lazily created SAM predictor, or None if unavailable."""
        if not self.sam_enabled:
            return None
        if self._sam_segmenter is not None:
            return self._sam_segmenter.predictor
        if self._sam_init_attempted:
            return None

        self._sam_init_attempted = True
        try:
            self._sam_segmenter = BBoxPromptSegmenter(
                backend=self.sam_backend,
                checkpoint_path=self.sam_checkpoint,
                model_type=self.sam_model_type,
                device=self.sam_device,
            )
            _log(
                {"logger": self.logger},
                "info",
                "SAM predictor loaded for PCA yaw refinement: "
                f"backend={self.sam_backend}, model_type={self.sam_model_type}",
            )
        except Exception as exc:
            _log(
                {"logger": self.logger},
                "warning",
                f"SAM predictor init failed for PCA yaw refinement: {exc}",
            )
            self._sam_segmenter = None
        return self._sam_segmenter.predictor if self._sam_segmenter is not None else None


def _merge_config(config: dict[str, Any] | None) -> dict[str, Any]:
    merged = default_pca_yaw_config()
    if config:
        merged.update(config)
    return merged


def _coerce_frames(crop_frames_bgr: list[np.ndarray] | np.ndarray | None) -> list[np.ndarray]:
    if crop_frames_bgr is None:
        return []
    if isinstance(crop_frames_bgr, np.ndarray):
        return [crop_frames_bgr]
    return [frame for frame in crop_frames_bgr if isinstance(frame, np.ndarray)]


def _resolve_predictor(sam_predictor):
    if sam_predictor is None:
        return None
    predictor = getattr(sam_predictor, "predictor", sam_predictor)
    if not hasattr(predictor, "set_image") or not hasattr(predictor, "predict"):
        return None
    return predictor


def _normalize_bbox_prompt(bbox_prompt, width: int, height: int) -> list[float]:
    if bbox_prompt is None:
        return [0.0, 0.0, float(width - 1), float(height - 1)]
    if isinstance(bbox_prompt, (list, tuple, np.ndarray)) and len(bbox_prompt) >= 4:
        x1 = float(np.clip(bbox_prompt[0], 0, max(width - 1, 0)))
        y1 = float(np.clip(bbox_prompt[1], 0, max(height - 1, 0)))
        x2 = float(np.clip(bbox_prompt[2], 0, max(width - 1, 0)))
        y2 = float(np.clip(bbox_prompt[3], 0, max(height - 1, 0)))
        return [x1, y1, x2, y2]
    return [0.0, 0.0, float(width - 1), float(height - 1)]


def _build_point_prompt(grasp_point, width: int, height: int):
    point = _coerce_point(grasp_point)
    if point is None:
        return None, None
    x = float(np.clip(point[0], 0, max(width - 1, 0)))
    y = float(np.clip(point[1], 0, max(height - 1, 0)))
    return np.asarray([[x, y]], dtype=np.float32), np.asarray([1], dtype=np.int32)


def _coerce_point(grasp_point) -> tuple[float, float] | None:
    if grasp_point is None:
        return None
    if isinstance(grasp_point, dict):
        x_value = grasp_point.get("x_px", grasp_point.get("x"))
        y_value = grasp_point.get("y_px", grasp_point.get("y"))
    else:
        try:
            x_value, y_value = grasp_point[:2]
        except Exception:
            return None
    x = _safe_float(x_value)
    y = _safe_float(y_value)
    if x is None or y is None:
        return None
    return float(x), float(y)


def _to_binary_mask(mask: np.ndarray | None, shape: tuple[int, int] | None = None):
    if mask is None or not isinstance(mask, np.ndarray) or mask.ndim != 2:
        return None
    binary = mask > 0
    if shape is not None and binary.shape != shape:
        binary = cv2.resize(
            binary.astype(np.uint8),
            (shape[1], shape[0]),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
    return binary


def _keep_largest_component(mask: np.ndarray) -> np.ndarray:
    binary = mask.astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    if count <= 1:
        return binary.astype(bool)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == largest


def _point_inside_mask(mask: np.ndarray, point: tuple[float, float] | None) -> bool:
    if point is None:
        return False
    x = int(round(point[0]))
    y = int(round(point[1]))
    height, width = mask.shape[:2]
    return 0 <= x < width and 0 <= y < height and bool(mask[y, x])


def _mask_centroid(mask: np.ndarray) -> np.ndarray:
    ys, xs = np.where(mask > 0)
    if xs.size == 0:
        return np.asarray([0.0, 0.0], dtype=np.float32)
    return np.asarray([float(xs.mean()), float(ys.mean())], dtype=np.float32)


def _normalized_distance(point_a, point_b, width: int, height: int) -> float:
    if point_a is None or point_b is None:
        return 1.0
    diagonal = math.hypot(width, height)
    if diagonal <= 0.0:
        return 0.0
    return float(np.linalg.norm(np.asarray(point_a) - np.asarray(point_b)) / diagonal)


def _apply_morphology(mask: np.ndarray, cfg: dict[str, Any]) -> np.ndarray:
    kernel_size = max(1, int(cfg["morph_kernel_size"]))
    if kernel_size <= 1:
        return mask.astype(np.uint8)
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    opened = cv2.morphologyEx(
        mask.astype(np.uint8),
        cv2.MORPH_OPEN,
        kernel,
        iterations=max(0, int(cfg["morph_open_iterations"])),
    )
    return cv2.morphologyEx(
        opened,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=max(0, int(cfg["morph_close_iterations"])),
    )


def _safe_float(value) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _first_non_empty_reason(*values: Any) -> str:
    for value in values:
        text = str(value or "").strip()
        if text:
            return text
    return ""


def _log(cfg: dict[str, Any], level: str, message: str) -> None:
    logger = cfg.get("logger")
    if logger is None:
        logging.getLogger(__name__).log(
            getattr(logging, level.upper(), logging.INFO),
            message,
        )
        return
    methods = [level]
    if level == "warning":
        methods.append("warn")
    for method_name in methods:
        method = getattr(logger, method_name, None)
        if callable(method):
            method(message)
            return
    logging.getLogger(__name__).info(message)
