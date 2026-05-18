"""Gemini API-backed VLM grasp point selection."""

from __future__ import annotations

import base64
import io
import json
import os
from pathlib import Path
from urllib import error, request

import cv2
from PIL import Image

from macgyvbot_config.vlm import (
    GRASP_POINT_MODE_API,
    VLM_GRASP_GRID_SIZES,
)
from macgyvbot_perception.grasp_point.vlm_grasp_point_selector import (
    DEFAULT_MAX_IMAGE_SIZE,
    VLMGraspPointSelector,
    VLMModel,
    VLMResult,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


DEFAULT_GEMINI_MODEL = "gemini-3-flash-preview"
DEFAULT_GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta"
DEFAULT_ENV_FILENAME = ".env"
GEMINI_API_KEY_NAME = "GEMINI_API_KEY"


class APIVLMGraspPointSelector:
    """Select grasp pixels using the Gemini vision API."""

    def __init__(
        self,
        logger,
        model: str | None = None,
        env_file: str | None = None,
        base_url: str | None = None,
        timeout_sec: float = 30.0,
    ):
        self.logger = logger
        self.model = GeminiVLMModel(
            model_id=model,
            env_file=env_file,
            base_url=base_url,
            timeout_sec=timeout_sec,
        )

    def select_grasp_pixel(
        self,
        bbox,
        label,
        color_image,
        depth_image,
        intrinsics,
        target_label,
    ):
        x1, y1, x2, y2 = VLMGraspPointSelector.clamp_bbox_to_image(
            bbox,
            color_image,
        )

        if x2 <= x1 or y2 <= y1:
            self.logger.warn("Gemini VLM crop bbox is empty.")
            return None

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
            self.logger.warn(f"Gemini VLM grasp point inference failed: {exc}")
            return None

        u = x1 + int(round(result.point[0]))
        v = y1 + int(round(result.point[1]))
        source = GRASP_POINT_MODE_API

        refined = VLMModel.refine_grasp_point_with_depth(
            depth_image,
            (u, v),
            focal_px=(intrinsics["fx"] + intrinsics["fy"]) / 2.0,
        )
        if refined is not None:
            u, v = refined.point
            source = f"{GRASP_POINT_MODE_API}+depth"

        self.logger.info(
            f"Gemini VLM grasp point selected: pixel=({u}, {v}), "
            f"angle={result.angle_deg:.1f}deg, "
            f"rpy_deg={result.orientation_rpy_deg}, source={source}"
        )

        return u, v, source, result.orientation_rpy_deg


class GeminiVLMModel(VLMModel):
    """Gemini implementation of the VLMModel ask() interface."""

    def __init__(
        self,
        model_id: str | None = None,
        max_image_size: int = DEFAULT_MAX_IMAGE_SIZE,
        max_new_tokens: int = 256,
        timeout_sec: float = 30.0,
        env_file: str | None = None,
        base_url: str | None = None,
        temperature: float = 0.0,
    ):
        self.model_id = model_id or DEFAULT_GEMINI_MODEL
        self.max_image_size = max_image_size
        self.max_new_tokens = max_new_tokens
        self.timeout_sec = float(timeout_sec)
        self.env_file = env_file or self._default_env_file()
        self.base_url = base_url or DEFAULT_GEMINI_BASE_URL
        self.temperature = float(temperature)

    def load(self):
        if not self._api_key():
            raise RuntimeError(
                "Gemini API key is not available. "
                f"Set {GEMINI_API_KEY_NAME} in {self.env_file}."
            )

    def get_runtime_info(self):
        return {
            "provider": "gemini",
            "model": self.model_id,
            "base_url": self.base_url,
            "env_file": self.env_file,
        }

    def unload(self):
        return None

    def ask(self, image: Image.Image, prompt: str) -> VLMResult:
        self.load()
        image = self._prepare_image(image)
        image_b64 = self._encode_jpeg_base64(image)
        text = self._ask_gemini(image_b64, prompt)
        return VLMResult(text=text, data=self._try_parse_json(text))

    def _ask_gemini(self, image_b64: str, prompt: str) -> str:
        base_url = self.base_url.rstrip("/")
        url = f"{base_url}/models/{self.model_id}:generateContent"
        payload = {
            "contents": [
                {
                    "parts": [
                        {
                            "inline_data": {
                                "mime_type": "image/jpeg",
                                "data": image_b64,
                            }
                        },
                        {"text": prompt},
                    ]
                }
            ],
            "generationConfig": {
                "maxOutputTokens": self.max_new_tokens,
                "temperature": self.temperature,
            },
        }
        body = self._post_json(
            url,
            payload,
            {
                "x-goog-api-key": self._api_key(),
                "Content-Type": "application/json",
            },
        )
        return self._extract_gemini_text(json.loads(body))

    def _post_json(self, url: str, payload: dict, headers: dict[str, str]) -> str:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = request.Request(url, data=data, headers=headers, method="POST")
        try:
            with request.urlopen(req, timeout=self.timeout_sec) as response:
                return response.read().decode("utf-8")
        except error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                "Gemini API request failed: "
                f"status={exc.code}, detail={detail[:500]}"
            ) from exc
        except error.URLError as exc:
            raise RuntimeError(f"Gemini API request failed: {exc}") from exc
        except TimeoutError as exc:
            raise RuntimeError("Gemini API request timed out") from exc

    def _api_key(self) -> str:
        key = os.environ.get(GEMINI_API_KEY_NAME, "").strip()
        if key:
            return key

        self._load_env_file()
        return os.environ.get(GEMINI_API_KEY_NAME, "").strip()

    def _load_env_file(self):
        path = Path(self.env_file).expanduser()
        if not path.exists() or not path.is_file():
            return

        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError:
            return

        for raw_line in lines:
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue

            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if not key or not value:
                continue

            os.environ.setdefault(key, value)

    @staticmethod
    def _default_env_file() -> str:
        candidates = []

        if get_package_share_directory is not None:
            try:
                share_dir = Path(get_package_share_directory("macgyvbot_resources"))
                candidates.append(share_dir / DEFAULT_ENV_FILENAME)
            except Exception:
                pass

        cwd = Path.cwd()
        candidates.append(cwd / "src" / "macgyvbot_resources" / DEFAULT_ENV_FILENAME)
        candidates.append(cwd / "macgyvbot_resources" / DEFAULT_ENV_FILENAME)

        current = Path(__file__).resolve()
        for parent in current.parents:
            candidates.append(
                parent
                / "src"
                / "macgyvbot_resources"
                / DEFAULT_ENV_FILENAME
            )
            candidates.append(
                parent
                / "macgyvbot_resources"
                / DEFAULT_ENV_FILENAME
            )

        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return str(candidate)

        for candidate in candidates:
            if candidate.parent.exists():
                return str(candidate)

        return str(current.parents[0] / DEFAULT_ENV_FILENAME)

    @staticmethod
    def _encode_jpeg_base64(image: Image.Image) -> str:
        buffer = io.BytesIO()
        image.convert("RGB").save(buffer, format="JPEG", quality=90)
        return base64.b64encode(buffer.getvalue()).decode("ascii")

    @staticmethod
    def _extract_gemini_text(data: dict) -> str:
        parts = []
        for candidate in data.get("candidates", []):
            content = candidate.get("content", {})
            for part in content.get("parts", []):
                text = part.get("text")
                if isinstance(text, str):
                    parts.append(text)
        text = "\n".join(parts).strip()
        if not text:
            raise RuntimeError(f"Gemini response did not contain text: {data}")
        return text
