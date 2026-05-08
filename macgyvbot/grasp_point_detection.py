"""Small VLM wrapper kept separate from the robot control loop.

The default model is intentionally modest so it can run on a consumer GPU
class machine, while still being useful for target/candidate selection.
"""

from __future__ import annotations

import json
import importlib.util
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
from transformers import AutoProcessor
try:
    from transformers import AutoModelForImageTextToText as AutoModelForVLM
except ImportError:
    from transformers import AutoModelForVision2Seq as AutoModelForVLM


DEFAULT_VLM_MODEL = "HuggingFaceTB/SmolVLM2-2.2B-Instruct"
DEFAULT_MAX_IMAGE_SIZE = 640
DEFAULT_GRID_SIZES = ((3, 3), (4, 4), (5, 5))


@dataclass(frozen=True)
class VLMResult:
    """Structured VLM output for downstream selection logic."""

    text: str
    data: dict[str, Any] | None = None


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
    choices: list[GridChoice] = field(default_factory=list)
    context: str = ""


@dataclass(frozen=True)
class DepthRefinementResult:
    """Depth-refined grasp pixel and metric depth."""

    point: tuple[int, int]
    depth_m: float
    radius_px: int


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

        self.processor = AutoProcessor.from_pretrained(self.model_id)
        has_accelerate = importlib.util.find_spec("accelerate") is not None
        has_bitsandbytes = importlib.util.find_spec("bitsandbytes") is not None
        quant_config = self._make_quantization_config(has_bitsandbytes)
        device_map = (
            "auto"
            if self.device == "cuda" and has_accelerate and quant_config is None
            else None
        )

        model_kwargs = {
            "torch_dtype": self.torch_dtype,
            "device_map": device_map,
            "low_cpu_mem_usage": True,
            "attn_implementation": "sdpa",
        }
        if quant_config is not None:
            model_kwargs["quantization_config"] = quant_config

        self.model = AutoModelForVLM.from_pretrained(
            self.model_id,
            **model_kwargs,
        )

        if device_map is None and quant_config is None:
            self.model.to(self.device)

        self.model.eval()

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
            return_dict=True,
            return_tensors="pt",
        )
        inputs = self._move_inputs_to_device(inputs)

        with torch.inference_mode():
            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=self.max_new_tokens,
                do_sample=False,
            )

        prompt_len = inputs["input_ids"].shape[-1]
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
        for rows, cols in grid_sizes:
            overlay = self._draw_numbered_grid(work_image, rows, cols)
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

            if choice is None:
                continue

            choices.append(choice)

            if self._has_converged(choices, work_image.size, early_stop_radius):
                break

        if not choices:
            raise RuntimeError("VLM did not return a valid grasp grid cell.")

        point, angle_rad = self._estimate_grasp_pose(choices)
        point = (point[0] * scale_x, point[1] * scale_y)

        return GraspRegionResult(
            point=point,
            angle_rad=angle_rad,
            angle_deg=math.degrees(angle_rad),
            choices=choices,
            context=context,
        )

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
            f"Object: {object_label}\n"
            f"Robot task: {request_text}\n"
            f"Context: {context}\n"
            f"Grid: {rows} rows x {cols} columns"
        )
        return self.ask(image, prompt)

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
        data = result.data or {}
        row = self._as_int(data.get("row"))
        col = self._as_int(data.get("col"))
        cell = self._as_int(data.get("cell"))

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
    def _try_parse_json(text: str) -> dict[str, Any] | None:
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
