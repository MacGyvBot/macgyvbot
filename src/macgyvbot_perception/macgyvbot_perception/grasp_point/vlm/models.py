"""Local VLM model wrappers for grasp point inference."""

from __future__ import annotations

import importlib.util
import json
import math
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from PIL import Image

try:
    import torch
except ImportError:
    torch = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from macgyvbot_config.vlm import VLM_MODEL_SMOL


DEFAULT_VLM_MODEL = VLM_MODEL_SMOL
DEFAULT_MAX_IMAGE_SIZE = 640


def _default_local_model_root():
    try:
        if get_package_share_directory is None:
            raise RuntimeError("ament_index_python is unavailable")
        share_dir = Path(get_package_share_directory("macgyvbot_resources"))
        return share_dir / "weights" / "vlm"
    except Exception:
        workspace_src = Path(__file__).resolve().parents[4]
        return workspace_src / "macgyvbot_resources" / "weights" / "vlm"


DEFAULT_LOCAL_MODEL_ROOT = _default_local_model_root()


def _load_transformers_classes():
    try:
        from transformers import AutoProcessor
    except ImportError as exc:
        raise RuntimeError(
            "VLM grasp mode requires the optional 'transformers' package."
        ) from exc

    try:
        from transformers import AutoModelForImageTextToText as AutoVLMModel
    except ImportError:
        from transformers import AutoModelForVision2Seq as AutoVLMModel

    return AutoProcessor, AutoVLMModel


def _load_stopping_criteria_classes():
    try:
        from transformers import StoppingCriteria, StoppingCriteriaList
    except ImportError:
        return None, None
    return StoppingCriteria, StoppingCriteriaList


@dataclass(frozen=True)
class VLMResult:
    """Plain-text VLM output plus parsed JSON when available."""

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
    """Grid VLM grasp result in crop-image coordinates."""

    point: tuple[float, float]
    angle_rad: float
    angle_deg: float
    orientation_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    choices: list[GridChoice] = field(default_factory=list)
    context: str = ""
    raw_text: str = ""


class VLMProgressLogger:
    """Log token-generation progress without stopping generation."""

    def __init__(
        self,
        logger,
        prompt_len: int,
        max_new_tokens: int,
        step_percent: int = 10,
    ):
        self.logger = logger
        self.prompt_len = max(0, int(prompt_len))
        self.max_new_tokens = max(1, int(max_new_tokens))
        self.step_percent = max(1, int(step_percent))
        self.next_percent = self.step_percent

    def __call__(self, input_ids, scores, **kwargs):
        if self.logger is None or not hasattr(input_ids, "shape"):
            return False

        generated = max(0, int(input_ids.shape[-1]) - self.prompt_len)
        percent = min(99, int(generated * 100 / self.max_new_tokens))
        if percent >= self.next_percent:
            self.logger.info(
                "VLM inference progress: "
                f"{percent}% ({generated}/{self.max_new_tokens} tokens)"
            )
            while self.next_percent <= percent:
                self.next_percent += self.step_percent
        return False


class _BaseVLM:
    """Shared local VLM loading and request execution."""

    def __init__(
        self,
        model_id: str = DEFAULT_VLM_MODEL,
        max_image_size: int = DEFAULT_MAX_IMAGE_SIZE,
        max_new_tokens: int = 96,
        use_4bit: bool = True,
        logger=None,
    ):
        self.model_id = model_id
        self.max_image_size = max_image_size
        self.max_new_tokens = max_new_tokens
        self.use_4bit = use_4bit
        self.logger = logger
        self.local_model_root = DEFAULT_LOCAL_MODEL_ROOT
        self.processor = None
        self.model = None
        self.device = "cuda" if torch is not None and torch.cuda.is_available() else "cpu"
        self.torch_dtype = self._select_dtype()

    def load(self):
        """Load processor/model once, only when inference is needed."""
        if self.model is not None and self.processor is not None:
            return

        if torch is None:
            raise RuntimeError("VLM grasp mode requires the optional 'torch' package.")

        model_source = self._resolve_model_source()
        local_dir = self.local_model_root / self.model_id.replace("/", "__")
        AutoProcessor, AutoVLMModel = _load_transformers_classes()

        try:
            self.processor = AutoProcessor.from_pretrained(model_source)
        except Exception as exc:
            raise RuntimeError(
                "VLM processor load failed: "
                f"model_source={model_source}, "
                f"local_expected={local_dir}, "
                f"local_exists={local_dir.exists()}, error={exc}"
            ) from exc

        has_accelerate = importlib.util.find_spec("accelerate") is not None
        has_bitsandbytes = importlib.util.find_spec("bitsandbytes") is not None
        device_map = "auto" if self.device == "cuda" and has_accelerate else None
        quant_config = self._make_quantization_config(has_bitsandbytes)
        if quant_config is not None:
            self._suppress_known_bitsandbytes_future_warning()
        model_kwargs = {
            "dtype": self.torch_dtype,
            "device_map": device_map,
            "low_cpu_mem_usage": True,
            "attn_implementation": "sdpa",
        }
        if quant_config is not None:
            model_kwargs["quantization_config"] = quant_config

        try:
            self.model = AutoVLMModel.from_pretrained(model_source, **model_kwargs)
        except Exception as exc:
            raise RuntimeError(
                "VLM model load failed: "
                f"model_source={model_source}, "
                f"local_expected={local_dir}, "
                f"local_exists={local_dir.exists()}, "
                f"kwargs_keys={list(model_kwargs.keys())}, error={exc}"
            ) from exc

        if device_map is None and quant_config is None:
            self.model.to(self.device)
        self.model.eval()

    def unload(self):
        """Release VRAM/RAM when the VLM is no longer needed."""
        self.processor = None
        self.model = None
        if torch is not None and torch.cuda.is_available():
            torch.cuda.empty_cache()

    def ask(self, image: Image.Image, prompt: str) -> VLMResult:
        """Run one VLM request."""
        self.load()
        image = self._prepare_image(image)
        inputs = self._build_inputs(image, prompt)
        inputs = self._move_inputs_to_device(inputs)
        self._log_inference_device(inputs)
        prompt_len = self._input_prompt_len(inputs)
        progress_criteria = self._build_progress_criteria(prompt_len)

        with torch.inference_mode():
            if isinstance(inputs, dict):
                generated_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=self.max_new_tokens,
                    do_sample=False,
                    **progress_criteria,
                )
            else:
                generated_ids = self.model.generate(
                    inputs,
                    max_new_tokens=self.max_new_tokens,
                    do_sample=False,
                    **progress_criteria,
                )

        generated_ids = generated_ids[:, prompt_len:]
        self._log_inference_complete(generated_ids)
        text = self.processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
        )[0].strip()
        return VLMResult(text=text, data=self._try_parse_json(text))

    def _build_inputs(self, image: Image.Image, prompt: str):
        """Build generation inputs with an attention mask when possible."""
        messages = self._build_messages(image, prompt)
        errors = []

        try:
            text = self.processor.apply_chat_template(
                messages,
                add_generation_prompt=True,
                tokenize=False,
            )
            return self._processor_inputs_from_text_and_image(text, image)
        except Exception as exc:
            errors.append(f"processor call path: {exc}")

        try:
            inputs = self.processor.apply_chat_template(
                messages,
                add_generation_prompt=True,
                tokenize=True,
                processor_kwargs={"return_tensors": "pt"},
            )
            return self._ensure_attention_mask(inputs)
        except Exception as exc:
            errors.append(f"chat template tokenization path: {exc}")

        raise RuntimeError("VLM input preparation failed: " + "; ".join(errors))

    @staticmethod
    def _build_messages(image: Image.Image, prompt: str):
        return [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": prompt},
                ],
            }
        ]

    def _processor_inputs_from_text_and_image(self, text, image: Image.Image):
        text_values = [text]
        if isinstance(text, str):
            text_values.insert(0, [text])

        image_values = ([image], [[image]], image)
        errors = []
        for text_value in text_values:
            for image_value in image_values:
                try:
                    inputs = self.processor(
                        text=text_value,
                        images=image_value,
                        return_tensors="pt",
                        padding=True,
                    )
                except Exception as exc:
                    errors.append(str(exc))
                    continue
                if self._input_ids(inputs) is not None:
                    return self._ensure_attention_mask(inputs)

        detail = errors[-1] if errors else "processor did not return input_ids"
        raise RuntimeError(detail)

    def _ensure_attention_mask(self, inputs):
        input_ids = self._input_ids(inputs)
        if (
            torch is not None
            and input_ids is not None
            and hasattr(inputs, "items")
            and "attention_mask" not in inputs
        ):
            inputs["attention_mask"] = torch.ones_like(input_ids)
        if torch is not None and input_ids is not None and not hasattr(inputs, "items"):
            return {
                "input_ids": input_ids,
                "attention_mask": torch.ones_like(input_ids),
            }
        return inputs

    @staticmethod
    def _input_ids(inputs):
        if hasattr(inputs, "items"):
            return inputs.get("input_ids")
        if hasattr(inputs, "shape"):
            return inputs
        return None

    def _input_prompt_len(self, inputs):
        input_ids = self._input_ids(inputs)
        if input_ids is None:
            raise RuntimeError("VLM inputs do not include input_ids.")
        return input_ids.shape[-1]

    def get_runtime_info(self):
        model_source = self._resolve_model_source()
        return {
            "model_id": self.model_id,
            "device": self.device,
            "dtype": str(self.torch_dtype).replace("torch.", ""),
            "model_source": model_source,
            "using_local_weights": str(model_source).startswith(str(self.local_model_root)),
        }

    def _resolve_model_source(self):
        local_dir = self.local_model_root / self.model_id.replace("/", "__")
        if local_dir.exists():
            return str(local_dir)
        return self.model_id

    def _select_dtype(self):
        if torch is None:
            return None
        if self.device != "cuda":
            return torch.float32
        if torch.cuda.is_bf16_supported():
            return torch.bfloat16
        return torch.float16

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
        self._suppress_known_bitsandbytes_future_warning()
        from transformers import BitsAndBytesConfig

        return BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=self.torch_dtype,
            bnb_4bit_use_double_quant=True,
        )

    @staticmethod
    def _suppress_known_bitsandbytes_future_warning():
        warnings.filterwarnings(
            "ignore",
            message=r"_check_is_size will be removed in a future PyTorch release.*",
            category=FutureWarning,
            module=r"bitsandbytes\.backends\.cuda\.ops",
        )

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

    def _log_inference_device(self, inputs):
        if self.logger is None:
            return

        cuda_available = torch is not None and torch.cuda.is_available()
        input_ids = self._input_ids(inputs)
        input_device = getattr(input_ids, "device", "unknown")
        model_device = self._model_input_device()
        placement = self._model_placement_summary()
        self.logger.info(
            "VLM inference device check: "
            f"cuda_available={cuda_available}, "
            f"configured_device={self.device}, "
            f"model_input_device={model_device}, "
            f"input_ids_device={input_device}, "
            f"model_placement={placement}"
        )

    def _build_progress_criteria(self, prompt_len):
        if self.logger is None:
            return {}

        StoppingCriteria, StoppingCriteriaList = _load_stopping_criteria_classes()
        if StoppingCriteria is None or StoppingCriteriaList is None:
            return {}

        class _ProgressStoppingCriteria(StoppingCriteria):
            def __init__(self, progress):
                self.progress = progress

            def __call__(self, input_ids, scores, **kwargs):
                return self.progress(input_ids, scores, **kwargs)

        return {
            "stopping_criteria": StoppingCriteriaList(
                [
                    _ProgressStoppingCriteria(
                        VLMProgressLogger(
                            self.logger,
                            prompt_len,
                            self.max_new_tokens,
                        )
                    )
                ]
            )
        }

    def _log_inference_complete(self, generated_ids):
        if self.logger is None:
            return
        generated_tokens = (
            int(generated_ids.shape[-1]) if hasattr(generated_ids, "shape") else 0
        )
        self.logger.info(
            "VLM inference complete: "
            f"generated_tokens={generated_tokens}/{self.max_new_tokens}"
        )

    def _model_placement_summary(self):
        device_map = getattr(self.model, "hf_device_map", None)
        if not device_map:
            try:
                return f"single_device:{next(self.model.parameters()).device}"
            except StopIteration:
                return f"single_device:{self.device}"

        counts = {}
        for device in device_map.values():
            device_name = self._device_name(device)
            counts[device_name] = counts.get(device_name, 0) + 1

        parts = [f"{device}:{count}" for device, count in sorted(counts.items())]
        offload = any(device in ("cpu", "disk") for device in counts)
        return f"hf_device_map({', '.join(parts)}; offload={offload})"

    @staticmethod
    def _device_name(device):
        if isinstance(device, int):
            return f"cuda:{device}"
        return str(device)

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


class VLMOnly(_BaseVLM):
    """Single-call local VLM model."""

    def inference(self, image: Image.Image, prompt: str) -> VLMResult:
        """Run the single-call grasp inference prompt."""
        return self.ask(image, prompt)


class VLM(_BaseVLM):
    """Grid-based local VLM model."""

    def inference(
        self,
        image: Image.Image,
        prompts,
        parser,
        grid_policy,
        grid_sizes: tuple[tuple[int, int], ...],
        object_label: str,
        user_request: str | None = None,
        early_stop_radius: float = 0.18,
        max_retries_per_grid: int = 5,
    ) -> GraspRegionResult:
        """Run context, grid, precise point, and orientation VLM calls."""
        original_width, original_height = image.size
        work_image = self._prepare_image(image)
        scale_x = original_width / float(work_image.size[0])
        scale_y = original_height / float(work_image.size[1])

        context_result = self.ask(
            work_image,
            prompts.build_context_prompt(object_label, user_request),
        )
        context = context_result.text
        choices = []
        retry_limit = max(1, int(max_retries_per_grid))

        for rows, cols in grid_sizes:
            overlay = grid_policy.draw_numbered_grid(work_image, rows, cols)
            choice = None
            prompt = prompts.build_grid_prompt(
                rows=rows,
                cols=cols,
                context=context,
                object_label=object_label,
                user_request=user_request,
            )
            for _attempt in range(1, retry_limit + 1):
                result = self.ask(overlay, prompt)
                choice = parser.parse_vlm_grid_result(
                    result,
                    rows=rows,
                    cols=cols,
                    image_size=work_image.size,
                )
                if choice is not None:
                    break
            if choice is None:
                continue
            choices.append(choice)
            if grid_policy.has_converged(choices, work_image.size, early_stop_radius):
                break

        if not choices:
            raise RuntimeError("VLM did not return a valid grasp grid cell.")

        coarse_point, angle_rad = grid_policy.estimate_grasp_pose(choices)
        coarse_yaw_deg = math.degrees(angle_rad)
        precise_result = None
        precise_prompt = prompts.build_precise_prompt(
            image_size=work_image.size,
            object_label=object_label,
            user_request=user_request,
            context=context,
            coarse_point=coarse_point,
            coarse_yaw_deg=coarse_yaw_deg,
        )
        for _attempt in range(1, retry_limit + 1):
            result = self.ask(work_image, precise_prompt)
            precise_result = parser.parse_vlm_precise_result(
                result,
                width=work_image.size[0],
                height=work_image.size[1],
            )
            if precise_result is not None:
                break

        if precise_result is not None:
            point_work, yaw_deg = precise_result
            point = (point_work[0] * scale_x, point_work[1] * scale_y)
            angle_rad = math.radians(yaw_deg)
            raw_text = result.text
        else:
            point = (coarse_point[0] * scale_x, coarse_point[1] * scale_y)
            raw_text = context_result.text

        orientation_result = self.ask(
            work_image,
            prompts.build_orientation_prompt(
                object_label=object_label,
                user_request=user_request,
                context=context,
                yaw_hint_deg=math.degrees(angle_rad),
            ),
        )
        orientation_rpy_deg = parser.parse_vlm_orientation_result(
            orientation_result,
            yaw_fallback_deg=math.degrees(angle_rad),
        )

        return GraspRegionResult(
            point=point,
            angle_rad=angle_rad,
            angle_deg=math.degrees(angle_rad),
            orientation_rpy_deg=orientation_rpy_deg,
            choices=choices,
            context=context,
            raw_text=raw_text,
        )
