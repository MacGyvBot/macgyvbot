"""VLM-based grasp point selection helpers.

The default model is intentionally modest so it can run on a consumer GPU
class machine, while still being useful for target/candidate selection.
"""

from __future__ import annotations

import json
import importlib.util
import math
import re
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
from transformers import AutoProcessor
from ament_index_python.packages import get_package_share_directory

from macgyvbot.config.config import (
    GRASP_POINT_MODE_VLM,
    VLM_GRASP_GRID_SIZES,
)

try:
    # Available in newer transformers.
    from transformers import AutoModelForImageTextToText as _AutoVLMModel
except ImportError:
    # Backward-compatible fallback for older transformers (e.g., 4.40.x).
    from transformers import AutoModelForVision2Seq as _AutoVLMModel


DEFAULT_VLM_MODEL = "HuggingFaceTB/SmolVLM2-2.2B-Instruct"
DEFAULT_MAX_IMAGE_SIZE = 640
DEFAULT_GRID_SIZES = ((3, 3), (4, 4), (5, 5))


def _default_local_model_root():
    # In ROS install space, data files live under share/<package>/...
    # Fallback to source-tree relative path for editable/dev execution.
    try:
        share_dir = Path(get_package_share_directory("macgyvbot"))
        return share_dir / "weights" / "vlm"
    except Exception:
        return Path(__file__).resolve().parents[2] / "weights" / "vlm"


DEFAULT_LOCAL_MODEL_ROOT = _default_local_model_root()


@dataclass(frozen=True)
class VLMResult:
    """Structured VLM output for downstream selection logic."""

    text: str
    data: Any | None = None


@dataclass(frozen=True)
class GridChoice:
    """One VLM-selected grid cell."""

    rows: int
    cols: int
    row: int
    col: int
    cell: int
    bbox: tuple[int, int, int, int]
    center: tuple[float, float]
    confidence: float | None
    reason: str
    raw_text: str


@dataclass(frozen=True)
class GraspRegionResult:
    """ORACLE-Grasp-style 2D grasp region prediction."""

    point: tuple[float, float]
    angle_rad: float
    angle_deg: float
    orientation_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    choices: list[GridChoice] = field(default_factory=list)
    context: str = ""


@dataclass(frozen=True)
class DepthRefinementResult:
    """Depth-refined grasp pixel and metric depth."""

    point: tuple[int, int]
    depth_m: float
    radius_px: int


class VLMGraspMechanism:
    """Select grasp pixels using a VLM and optional depth refinement."""

    def __init__(self, logger):
        self.logger = logger
        self.model = None

    def select_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        x1, y1, x2, y2 = self.clamp_bbox_to_image(bbox, color_image)

        if x2 <= x1 or y2 <= y1:
            self.logger.warn("VLM crop bbox가 비어 있습니다.")
            return None

        self._ensure_model_loaded()

        crop_bgr = color_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = Image.fromarray(crop_rgb)

        try:
            result = self.model.select_grasp_region(
                crop_image,
                object_label=label,
                user_request=target_label,
                grid_sizes=VLM_GRASP_GRID_SIZES,
            )
        except Exception as exc:
            self.logger.warn(f"VLM grasp point 추론 실패: {exc}")
            return None

        u = x1 + int(round(result.point[0]))
        v = y1 + int(round(result.point[1]))
        source = GRASP_POINT_MODE_VLM

        refined = VLMModel.refine_grasp_point_with_depth(
            depth_image,
            (u, v),
            focal_px=(intrinsics["fx"] + intrinsics["fy"]) / 2.0,
        )
        if refined is not None:
            u, v = refined.point
            source = f"{GRASP_POINT_MODE_VLM}+depth"

        self.logger.info(
            f"VLM grasp point 선택: pixel=({u}, {v}), "
            f"angle={result.angle_deg:.1f}deg, "
            f"rpy_deg={result.orientation_rpy_deg}, source={source}"
        )

        return u, v, source, result.orientation_rpy_deg

    def _ensure_model_loaded(self):
        if self.model is not None:
            return

        self.logger.info("VLM grasp 모델을 lazy load 준비합니다.")
        self.model = VLMModel()
        runtime = self.model.get_runtime_info()
        self.logger.info(
            "VLM runtime: "
            f"device={runtime['device']}, "
            f"dtype={runtime['dtype']}, "
            f"local_weights={runtime['using_local_weights']}, "
            f"source={runtime['model_source']}"
        )
        if runtime["device"] == "cuda":
            self.logger.info("VLM은 CUDA를 사용합니다.")
        else:
            self.logger.warn("VLM이 CUDA를 사용하지 않습니다. (CPU 실행)")
        self.logger.info("VLM 가중치 로드 시작...")
        self.model.load()
        self.logger.info("VLM 가중치 로드 완료.")

    @staticmethod
    def clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2


class VLMModel:
    """Lazy-loading VLM helper.

    This class does not depend on ROS, MoveIt, YOLO, or the gripper. Keep it as
    a lightweight adapter so the pick/place node can choose whether and when to
    call the VLM.
    """

    def __init__(
        self,
        model_id: str = DEFAULT_VLM_MODEL,
        max_image_size: int = DEFAULT_MAX_IMAGE_SIZE,
        max_new_tokens: int = 96,
        use_4bit: bool = True,
    ):
        self.model_id = model_id
        self.max_image_size = max_image_size
        self.max_new_tokens = max_new_tokens
        self.use_4bit = use_4bit
        self.local_model_root = DEFAULT_LOCAL_MODEL_ROOT

        self.processor = None
        self.model = None
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = self._select_dtype()

    def _select_dtype(self):
        if self.device != "cuda":
            return torch.float32

        if torch.cuda.is_bf16_supported():
            return torch.bfloat16

        return torch.float16

    def load(self):
        """Load the processor/model once, only when VLM inference is needed."""
        if self.model is not None and self.processor is not None:
            return

        model_source = self._resolve_model_source()
        local_dir = self.local_model_root / self.model_id.replace("/", "__")
        print(
            "[VLMModel.load] "
            f"model_id={self.model_id}, "
            f"resolved_source={model_source}, "
            f"local_expected={local_dir}, "
            f"local_exists={local_dir.exists()}"
        )

        try:
            self.processor = AutoProcessor.from_pretrained(model_source)
        except Exception as exc:
            raise RuntimeError(
                "VLM processor 로드 실패: "
                f"model_source={model_source}, "
                f"local_expected={local_dir}, "
                f"local_exists={local_dir.exists()}, "
                f"error={exc}"
            ) from exc

        has_accelerate = importlib.util.find_spec("accelerate") is not None
        has_bitsandbytes = importlib.util.find_spec("bitsandbytes") is not None
        device_map = "auto" if self.device == "cuda" and has_accelerate else None
        quant_config = self._make_quantization_config(has_bitsandbytes)

        model_kwargs = {
            "dtype": self.torch_dtype,
            "device_map": device_map,
            "low_cpu_mem_usage": True,
            "attn_implementation": "sdpa",
        }
        if quant_config is not None:
            model_kwargs["quantization_config"] = quant_config

        try:
            self.model = _AutoVLMModel.from_pretrained(
                model_source,
                **model_kwargs,
            )
        except Exception as exc:
            raise RuntimeError(
                "VLM model 로드 실패: "
                f"model_source={model_source}, "
                f"local_expected={local_dir}, "
                f"local_exists={local_dir.exists()}, "
                f"kwargs_keys={list(model_kwargs.keys())}, "
                f"error={exc}"
            ) from exc

        if device_map is None and quant_config is None:
            self.model.to(self.device)

        self.model.eval()

    def _resolve_model_source(self):
        # Prefer pre-downloaded local weights under weights/vlm/<org>__<model>.
        local_dir = self.local_model_root / self.model_id.replace("/", "__")
        if local_dir.exists():
            return str(local_dir)
        return self.model_id

    def get_runtime_info(self):
        model_source = self._resolve_model_source()
        return {
            "device": self.device,
            "dtype": str(self.torch_dtype).replace("torch.", ""),
            "model_source": model_source,
            "using_local_weights": str(model_source).startswith(str(self.local_model_root)),
        }

    def unload(self):
        """Release VRAM/RAM when the VLM is no longer needed."""
        self.processor = None
        self.model = None

        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    def ask(self, image: Image.Image, prompt: str) -> VLMResult:
        """Run one VLM request and return plain text plus parsed JSON if present."""
        self.load()

        image = self._prepare_image(image)
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": prompt},
                ],
            }
        ]

        inputs = self.processor.apply_chat_template(
            messages,
            add_generation_prompt=True,
            tokenize=True,
            processor_kwargs={
                "return_dict": True,
                "return_tensors": "pt",
            },
        )
        inputs = self._move_inputs_to_device(inputs)

        with torch.inference_mode():
            if isinstance(inputs, dict):
                generated_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=self.max_new_tokens,
                    do_sample=False,
                )
            else:
                generated_ids = self.model.generate(
                    inputs,
                    max_new_tokens=self.max_new_tokens,
                    do_sample=False,
                )

        if isinstance(inputs, dict) and "input_ids" in inputs:
            prompt_len = inputs["input_ids"].shape[-1]
        else:
            prompt_len = inputs.shape[-1]
        generated_ids = generated_ids[:, prompt_len:]
        text = self.processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
        )[0].strip()

        return VLMResult(text=text, data=self._try_parse_json(text))

    def select_candidate(
        self,
        image: Image.Image,
        user_request: str,
        candidates: list[dict[str, Any]],
    ) -> VLMResult:
        """Ask the VLM to choose one candidate from detector results.

        Candidates should be small dictionaries from the detector layer, for
        example: {"index": 0, "label": "cup", "bbox": [x1, y1, x2, y2]}.
        """
        prompt = (
            "You are helping a robot choose one object candidate to pick.\n"
            "Choose only from the provided candidates. Return compact JSON only "
            "with keys: index, label, confidence, reason.\n\n"
            f"User request: {user_request}\n"
            f"Candidates: {json.dumps(candidates, ensure_ascii=False)}"
        )
        return self.ask(image, prompt)

    def select_grasp_region(
        self,
        image: Image.Image,
        object_label: str = "the object",
        user_request: str | None = None,
        grid_sizes: tuple[tuple[int, int], ...] = DEFAULT_GRID_SIZES,
        early_stop_radius: float = 0.18,
        max_retries_per_grid: int = 5,
    ) -> GraspRegionResult:
        """Select an affordance-aligned grasp region using grid reasoning.

        The VLM only chooses numbered grid cells. The final grasp point and
        orientation are computed locally from those selected cells, keeping
        metric execution outside the model.
        """
        original_width, original_height = image.size
        work_image = self._prepare_image(image)
        scale_x = original_width / float(work_image.size[0])
        scale_y = original_height / float(work_image.size[1])

        context = self.describe_grasp_context(
            work_image,
            object_label=object_label,
            user_request=user_request,
        )

        choices = []
        retry_limit = max(1, int(max_retries_per_grid))
        for rows, cols in grid_sizes:
            overlay = self._draw_numbered_grid(work_image, rows, cols)
            choice = None
            for attempt in range(1, retry_limit + 1):
                result = self._choose_grid_cell(
                    overlay,
                    rows=rows,
                    cols=cols,
                    context=context,
                    object_label=object_label,
                    user_request=user_request,
                )
                choice = self._grid_choice_from_result(
                    result=result,
                    rows=rows,
                    cols=cols,
                    image_size=work_image.size,
                )
                if choice is not None:
                    break
                print(
                    "[VLMModel.select_grasp_region] "
                    f"invalid grid choice (rows={rows}, cols={cols}) "
                    f"retry {attempt}/{retry_limit}, "
                    f"raw='{result.text[:200]}'"
                )

            if choice is None:
                continue

            choices.append(choice)

            if self._has_converged(choices, work_image.size, early_stop_radius):
                break

        if not choices:
            raise RuntimeError("VLM did not return a valid grasp grid cell.")

        coarse_point, angle_rad = self._estimate_grasp_pose(choices)
        coarse_yaw_deg = math.degrees(angle_rad)

        precise = self._estimate_precise_point_and_yaw(
            work_image,
            object_label=object_label,
            user_request=user_request,
            context=context,
            coarse_point=coarse_point,
            coarse_yaw_deg=coarse_yaw_deg,
            max_retries=max_retries_per_grid,
        )
        if precise is not None:
            point_work, yaw_deg = precise
            point = (point_work[0] * scale_x, point_work[1] * scale_y)
            angle_rad = math.radians(yaw_deg)
        else:
            point = (coarse_point[0] * scale_x, coarse_point[1] * scale_y)

        orientation_rpy_deg = self._estimate_grasp_orientation_rpy(
            work_image,
            object_label=object_label,
            user_request=user_request,
            context=context,
            yaw_hint_deg=math.degrees(angle_rad),
        )

        return GraspRegionResult(
            point=point,
            angle_rad=angle_rad,
            angle_deg=math.degrees(angle_rad),
            orientation_rpy_deg=orientation_rpy_deg,
            choices=choices,
            context=context,
        )

    def _estimate_grasp_orientation_rpy(
        self,
        image: Image.Image,
        object_label: str,
        user_request: str | None,
        context: str,
        yaw_hint_deg: float | None = None,
    ) -> tuple[float, float, float]:
        request_text = user_request or "Pick up the object in a natural way."
        hint = ""
        if yaw_hint_deg is not None and math.isfinite(yaw_hint_deg):
            hint = f"\nPrevious coarse yaw hint (deg): {float(yaw_hint_deg):.2f}"
        prompt = (
            "Estimate grasp orientation as strict JSON only for a robot end-effector.\n"
            "Return keys: roll_deg, pitch_deg, yaw_deg, confidence, reason.\n"
            "Use degrees. Keep values in [-180, 180].\n"
            "If uncertain, keep roll_deg=0 and pitch_deg=0, but still output yaw_deg.\n"
            "Do not return markdown or code fences.\n"
            f"Object: {object_label}\n"
            f"Robot task: {request_text}\n"
            f"Context: {context}\n"
            f"{hint}\n"
        )
        result = self.ask(image, prompt)
        data = self._as_mapping(result.data)
        roll = self._as_float(data.get("roll_deg"))
        pitch = self._as_float(data.get("pitch_deg"))
        yaw = self._as_float(data.get("yaw_deg"))

        # Fallback for variant key names returned by some model outputs.
        if roll is None:
            roll = self._as_float(data.get("roll"))
        if pitch is None:
            pitch = self._as_float(data.get("pitch"))
        if yaw is None:
            yaw = self._as_float(data.get("yaw"))

        # Last resort: extract numbers from free-form text.
        if roll is None or pitch is None or yaw is None:
            extracted = self._extract_orientation_from_text(result.text)
            if extracted is not None:
                roll, pitch, yaw = extracted

        if yaw is None:
            yaw = yaw_hint_deg
        if roll is None:
            roll = 0.0
        if pitch is None:
            pitch = 0.0
        if yaw is None:
            return (0.0, 0.0, 0.0)
        return (float(roll), float(pitch), float(yaw))

    def describe_grasp_context(
        self,
        image: Image.Image,
        object_label: str = "the object",
        user_request: str | None = None,
    ) -> str:
        """Extract concise affordance context before grid selection."""
        request_text = user_request or "Pick up the object in a natural way."
        prompt = (
            "Briefly describe the most suitable part of the object for a "
            "two-finger parallel gripper to grasp. Prefer functional and safe "
            "parts such as handles or sturdy bodies, and avoid blades, tips, "
            "openings, fragile parts, or task-critical surfaces.\n"
            f"Object: {object_label}\n"
            f"Robot task: {request_text}\n"
            "Answer in one short sentence."
        )
        return self.ask(image, prompt).text

    @staticmethod
    def refine_grasp_point_with_depth(
        depth_image: np.ndarray,
        point: tuple[float, float],
        focal_px: float,
        gripper_clearance_m: float = 0.025,
        samples: int = 6,
        depth_scale: float = 0.001,
    ) -> DepthRefinementResult | None:
        """Move a VLM-selected pixel to the closest valid local depth surface.

        This follows the ORACLE-Grasp idea: if the selected pixel lands on a
        hole or background, search circular regions sized by expected gripper
        clearance and choose the nearest valid surface.
        """
        if depth_image is None or depth_image.size == 0:
            return None

        height, width = depth_image.shape[:2]
        u = int(round(point[0]))
        v = int(round(point[1]))

        if not (0 <= u < width and 0 <= v < height):
            return None

        depth_m = VLMModel._depth_at(depth_image, u, v, depth_scale)
        if depth_m is None:
            depth_m = VLMModel._nearest_valid_depth(
                depth_image,
                u,
                v,
                radius_px=12,
                depth_scale=depth_scale,
            )

        if depth_m is None:
            return None

        best = None
        for idx in range(1, samples + 1):
            sampled_depth = max((idx / float(samples)) * depth_m, depth_scale)
            radius_px = max(
                2,
                int(round((focal_px * gripper_clearance_m) / sampled_depth)),
            )
            candidate = VLMModel._closest_valid_in_circle(
                depth_image,
                u,
                v,
                radius_px=radius_px,
                depth_scale=depth_scale,
            )

            if candidate is None:
                continue

            cu, cv, candidate_depth = candidate
            if best is None or candidate_depth < best.depth_m:
                best = DepthRefinementResult(
                    point=(cu, cv),
                    depth_m=candidate_depth,
                    radius_px=radius_px,
                )

        return best

    def _prepare_image(self, image: Image.Image) -> Image.Image:
        image = image.convert("RGB")
        width, height = image.size
        longest = max(width, height)

        if longest <= self.max_image_size:
            return image

        scale = self.max_image_size / float(longest)
        new_size = (int(width * scale), int(height * scale))
        return image.resize(new_size, Image.Resampling.LANCZOS)

    def _make_quantization_config(self, has_bitsandbytes: bool):
        if not self.use_4bit or self.device != "cuda" or not has_bitsandbytes:
            return None

        from transformers import BitsAndBytesConfig

        return BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=self.torch_dtype,
            bnb_4bit_use_double_quant=True,
        )

    def _choose_grid_cell(
        self,
        image: Image.Image,
        rows: int,
        cols: int,
        context: str,
        object_label: str,
        user_request: str | None,
    ) -> VLMResult:
        request_text = user_request or "Pick up the object in a natural way."
        prompt = (
            "The image has a numbered grid overlay. Choose the single best grid "
            "cell for a two-finger robot gripper to grasp the object. Use the "
            "context to prefer affordance-aligned, sturdy, safe regions.\n"
            "Return JSON only with keys: row, col, cell, confidence, reason.\n"
            "Rows and columns are 1-indexed.\n\n"
            "Output must be a single JSON object. No extra text.\n"
            f"Object: {object_label}\n"
            f"Robot task: {request_text}\n"
            f"Context: {context}\n"
            f"Grid: {rows} rows x {cols} columns"
        )
        return self.ask(image, prompt)

    def _estimate_precise_point_and_yaw(
        self,
        image: Image.Image,
        object_label: str,
        user_request: str | None,
        context: str,
        coarse_point: tuple[float, float],
        coarse_yaw_deg: float,
        max_retries: int = 5,
    ) -> tuple[tuple[float, float], float] | None:
        request_text = user_request or "Pick up the object in a natural way."
        width, height = image.size
        retry_limit = max(1, int(max_retries))
        coarse_x = int(round(coarse_point[0]))
        coarse_y = int(round(coarse_point[1]))

        prompt = (
            "Choose a precise grasp center and in-plane grasp yaw for a two-finger gripper.\n"
            "Return strict JSON only with keys: x_px, y_px, yaw_deg, confidence, reason.\n"
            f"Image size: width={width}, height={height}.\n"
            "Constraints:\n"
            f"- x_px must be integer in [0, {width - 1}]\n"
            f"- y_px must be integer in [0, {height - 1}]\n"
            "- yaw_deg must be in [-180, 180]\n"
            "- choose a sturdy graspable region, avoid edges/holes/slippery tips\n"
            "- no markdown, no code fence, no explanation outside JSON\n\n"
            f"Object: {object_label}\n"
            f"Robot task: {request_text}\n"
            f"Context: {context}\n"
            f"Coarse point hint: x={coarse_x}, y={coarse_y}\n"
            f"Coarse yaw hint(deg): {coarse_yaw_deg:.2f}\n"
        )

        for attempt in range(1, retry_limit + 1):
            result = self.ask(image, prompt)
            data = self._as_mapping(result.data)
            parsed_point = self._extract_point_px(data, result.text, width, height)
            yaw = self._extract_yaw_deg(data, result.text)

            if parsed_point is not None and yaw is not None:
                return parsed_point, self._normalize_angle_deg(yaw)

            print(
                "[VLMModel._estimate_precise_point_and_yaw] "
                f"invalid response retry {attempt}/{retry_limit}, "
                f"raw='{result.text[:220]}'"
            )

        return None

    def _draw_numbered_grid(
        self,
        image: Image.Image,
        rows: int,
        cols: int,
    ) -> Image.Image:
        overlay = image.convert("RGB").copy()
        draw = ImageDraw.Draw(overlay, "RGBA")
        width, height = overlay.size
        cell_w = width / float(cols)
        cell_h = height / float(rows)
        font = ImageFont.load_default()

        for row in range(rows):
            for col in range(cols):
                x0 = int(round(col * cell_w))
                y0 = int(round(row * cell_h))
                x1 = int(round((col + 1) * cell_w))
                y1 = int(round((row + 1) * cell_h))
                cell_id = row * cols + col + 1

                draw.rectangle((x0, y0, x1, y1), outline=(255, 0, 0, 220), width=2)
                draw.rectangle((x0 + 3, y0 + 3, x0 + 34, y0 + 23), fill=(0, 0, 0, 145))
                draw.text(
                    (x0 + 8, y0 + 7),
                    str(cell_id),
                    fill=(255, 255, 255, 255),
                    font=font,
                )

        return overlay

    def _grid_choice_from_result(
        self,
        result: VLMResult,
        rows: int,
        cols: int,
        image_size: tuple[int, int],
    ) -> GridChoice | None:
        data = self._as_mapping(result.data)
        row = self._as_int(data.get("row"))
        col = self._as_int(data.get("col"))
        cell = self._as_int(data.get("cell"))

        if cell is None:
            cell = self._as_int(data.get("cell_id"))
        if cell is None:
            cell = self._as_int(data.get("grid_cell"))

        if (row is None or col is None) and data:
            rc = self._extract_row_col_from_text(json.dumps(data, ensure_ascii=False))
            if rc is not None:
                row, col = rc
            elif cell is None:
                cell = self._extract_first_int(json.dumps(data, ensure_ascii=False))

        if row is None or col is None or cell is None:
            text_row_col = self._extract_row_col_from_text(result.text)
            if text_row_col is not None:
                row, col = text_row_col
            elif cell is None:
                cell = self._extract_first_int(result.text)

        if (row is None or col is None) and cell is not None:
            row = (cell - 1) // cols + 1
            col = (cell - 1) % cols + 1

        if row is None or col is None:
            return None

        if row < 1 or row > rows or col < 1 or col > cols:
            return None

        cell = (row - 1) * cols + col
        width, height = image_size
        x0 = int(round((col - 1) * width / float(cols)))
        y0 = int(round((row - 1) * height / float(rows)))
        x1 = int(round(col * width / float(cols)))
        y1 = int(round(row * height / float(rows)))
        center = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)

        return GridChoice(
            rows=rows,
            cols=cols,
            row=row,
            col=col,
            cell=cell,
            bbox=(x0, y0, x1, y1),
            center=center,
            confidence=self._as_float(data.get("confidence")),
            reason=str(data.get("reason", "")),
            raw_text=result.text,
        )

    def _has_converged(
        self,
        choices: list[GridChoice],
        image_size: tuple[int, int],
        radius_factor: float,
    ) -> bool:
        if len(choices) < 2:
            return False

        centers = [choice.center for choice in choices]
        cx = sum(point[0] for point in centers) / len(centers)
        cy = sum(point[1] for point in centers) / len(centers)
        max_dist = max(math.hypot(x - cx, y - cy) for x, y in centers)
        diagonal = math.hypot(image_size[0], image_size[1])

        return max_dist < radius_factor * diagonal

    def _estimate_grasp_pose(
        self,
        choices: list[GridChoice],
    ) -> tuple[tuple[float, float], float]:
        centers = [choice.center for choice in choices]
        point = (
            sum(center[0] for center in centers) / len(centers),
            sum(center[1] for center in centers) / len(centers),
        )

        if len(centers) < 2:
            return point, 0.0

        mean_x, mean_y = point
        xx = sum((x - mean_x) * (x - mean_x) for x, _ in centers) / len(centers)
        yy = sum((y - mean_y) * (y - mean_y) for _, y in centers) / len(centers)
        xy = sum((x - mean_x) * (y - mean_y) for x, y in centers) / len(centers)

        angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
        return point, angle

    @staticmethod
    def _depth_at(
        depth_image: np.ndarray,
        u: int,
        v: int,
        depth_scale: float,
    ) -> float | None:
        value = float(depth_image[v, u]) * depth_scale

        if not math.isfinite(value) or value <= 0.0:
            return None

        return value

    @staticmethod
    def _nearest_valid_depth(
        depth_image: np.ndarray,
        u: int,
        v: int,
        radius_px: int,
        depth_scale: float,
    ) -> float | None:
        candidate = VLMModel._closest_valid_in_circle(
            depth_image,
            u,
            v,
            radius_px=radius_px,
            depth_scale=depth_scale,
        )

        if candidate is None:
            return None

        return candidate[2]

    @staticmethod
    def _closest_valid_in_circle(
        depth_image: np.ndarray,
        u: int,
        v: int,
        radius_px: int,
        depth_scale: float,
    ) -> tuple[int, int, float] | None:
        height, width = depth_image.shape[:2]
        x0 = max(0, u - radius_px)
        y0 = max(0, v - radius_px)
        x1 = min(width, u + radius_px + 1)
        y1 = min(height, v + radius_px + 1)

        if x0 >= x1 or y0 >= y1:
            return None

        patch = depth_image[y0:y1, x0:x1].astype(np.float32) * depth_scale
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (xx - u) ** 2 + (yy - v) ** 2 <= radius_px**2
        valid = mask & np.isfinite(patch) & (patch > 0.0)

        if not np.any(valid):
            return None

        valid_depths = np.where(valid, patch, np.inf)
        flat_index = int(np.argmin(valid_depths))
        local_v, local_u = np.unravel_index(flat_index, valid_depths.shape)
        depth_m = float(valid_depths[local_v, local_u])

        return x0 + int(local_u), y0 + int(local_v), depth_m

    @staticmethod
    def _as_int(value) -> int | None:
        if value is None:
            return None

        try:
            return int(value)
        except (TypeError, ValueError):
            try:
                return int(float(value))
            except (TypeError, ValueError):
                return None

    @staticmethod
    def _as_float(value) -> float | None:
        if value is None:
            return None

        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _move_inputs_to_device(self, inputs):
        target_device = self._model_input_device()

        if hasattr(inputs, "to") and not hasattr(inputs, "items"):
            if torch.is_floating_point(inputs):
                return inputs.to(target_device, dtype=self.torch_dtype)
            return inputs.to(target_device)

        if not hasattr(inputs, "items"):
            return inputs

        moved = {}
        for key, value in inputs.items():
            if not hasattr(value, "to"):
                moved[key] = value
                continue

            if torch.is_floating_point(value):
                moved[key] = value.to(target_device, dtype=self.torch_dtype)
            else:
                moved[key] = value.to(target_device)

        return moved

    def _model_input_device(self):
        if hasattr(self.model, "hf_device_map"):
            for device in self.model.hf_device_map.values():
                if isinstance(device, int):
                    return torch.device(f"cuda:{device}")
                if isinstance(device, str) and device not in ("cpu", "disk"):
                    return torch.device(device)

        try:
            return next(self.model.parameters()).device
        except StopIteration:
            return torch.device(self.device)

    @staticmethod
    def _try_parse_json(text: str) -> Any | None:
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        start = text.find("{")
        end = text.rfind("}")

        if start == -1 or end == -1 or end <= start:
            return None

        try:
            return json.loads(text[start : end + 1])
        except json.JSONDecodeError:
            return None

    @staticmethod
    def _as_mapping(value: Any) -> dict[str, Any]:
        if isinstance(value, dict):
            return value
        return {}

    @staticmethod
    def _extract_first_int(text: str) -> int | None:
        if not text:
            return None

        match = re.search(r"-?\d+", text)
        if not match:
            return None

        try:
            return int(match.group(0))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _extract_row_col_from_text(text: str) -> tuple[int, int] | None:
        if not text:
            return None

        row_match = re.search(r"row\s*[:=]?\s*(-?\d+)", text, re.IGNORECASE)
        col_match = re.search(r"col(?:umn)?\s*[:=]?\s*(-?\d+)", text, re.IGNORECASE)
        if row_match and col_match:
            try:
                return int(row_match.group(1)), int(col_match.group(1))
            except (TypeError, ValueError):
                return None

        # Fallback for short forms like "(2,3)".
        tuple_match = re.search(r"\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\)", text)
        if tuple_match:
            try:
                return int(tuple_match.group(1)), int(tuple_match.group(2))
            except (TypeError, ValueError):
                return None

        return None

    @staticmethod
    def _extract_orientation_from_text(text: str) -> tuple[float, float, float] | None:
        if not text:
            return None

        roll_match = re.search(r"roll(?:_deg)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
        pitch_match = re.search(r"pitch(?:_deg)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
        yaw_match = re.search(r"yaw(?:_deg)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)

        if roll_match and pitch_match and yaw_match:
            try:
                return (
                    float(roll_match.group(1)),
                    float(pitch_match.group(1)),
                    float(yaw_match.group(1)),
                )
            except (TypeError, ValueError):
                return None

        return None

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        return ((float(angle_deg) + 180.0) % 360.0) - 180.0

    def _extract_yaw_deg(self, data: dict[str, Any], raw_text: str) -> float | None:
        for key in ("yaw_deg", "yaw", "angle_deg", "grasp_angle_deg"):
            value = self._as_float(data.get(key))
            if value is not None and math.isfinite(value):
                return value

        yaw_match = re.search(r"yaw(?:_deg)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)", raw_text, re.IGNORECASE)
        if yaw_match:
            try:
                return float(yaw_match.group(1))
            except (TypeError, ValueError):
                return None
        return None

    def _extract_point_px(
        self,
        data: dict[str, Any],
        raw_text: str,
        width: int,
        height: int,
    ) -> tuple[float, float] | None:
        direct_key_pairs = (
            ("x_px", "y_px"),
            ("x", "y"),
            ("u", "v"),
            ("pixel_x", "pixel_y"),
        )
        for x_key, y_key in direct_key_pairs:
            x = self._as_int(data.get(x_key))
            y = self._as_int(data.get(y_key))
            if x is not None and y is not None:
                return self._clamp_point_to_image(float(x), float(y), width, height)

        x_norm = self._as_float(data.get("x_norm"))
        y_norm = self._as_float(data.get("y_norm"))
        if x_norm is not None and y_norm is not None:
            x = x_norm * (width - 1)
            y = y_norm * (height - 1)
            return self._clamp_point_to_image(x, y, width, height)

        x_match = re.search(r"x(?:_px)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)", raw_text, re.IGNORECASE)
        y_match = re.search(r"y(?:_px)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)", raw_text, re.IGNORECASE)
        if x_match and y_match:
            try:
                x = float(x_match.group(1))
                y = float(y_match.group(1))
                return self._clamp_point_to_image(x, y, width, height)
            except (TypeError, ValueError):
                return None

        tuple_match = re.search(r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)", raw_text)
        if tuple_match:
            try:
                x = float(tuple_match.group(1))
                y = float(tuple_match.group(2))
                return self._clamp_point_to_image(x, y, width, height)
            except (TypeError, ValueError):
                return None

        return None

    @staticmethod
    def _clamp_point_to_image(x: float, y: float, width: int, height: int) -> tuple[float, float]:
        cx = min(max(float(x), 0.0), float(max(width - 1, 0)))
        cy = min(max(float(y), 0.0), float(max(height - 1, 0)))
        return (cx, cy)
