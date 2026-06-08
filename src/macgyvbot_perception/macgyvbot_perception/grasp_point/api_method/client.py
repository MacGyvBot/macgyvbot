"""Gemini API client for grasp point selection."""

from __future__ import annotations

import base64
import io
import json
import os
from dataclasses import dataclass
from pathlib import Path
from urllib import error, request

from PIL import Image

from macgyvbot_config.vlm import (
    GRASP_POINT_API_BASE_URL,
    GRASP_POINT_API_ENV_FILENAME,
    GRASP_POINT_API_KEY_NAME,
    GRASP_POINT_API_MAX_IMAGE_SIZE,
    GRASP_POINT_API_MAX_OUTPUT_TOKENS,
    GRASP_POINT_API_MODEL,
    GRASP_POINT_API_TEMPERATURE,
    GRASP_POINT_API_THINKING_BUDGET,
    GRASP_POINT_API_TIMEOUT_SEC,
)
from macgyvbot_perception.grasp_point.api_method.prompts import build_grasp_prompt
from macgyvbot_perception.grasp_point.vlm.parser import Parser
from macgyvbot_resources.resources import resolve_env_file


DEFAULT_GEMINI_MODEL = GRASP_POINT_API_MODEL
DEFAULT_GEMINI_BASE_URL = GRASP_POINT_API_BASE_URL
DEFAULT_ENV_FILENAME = GRASP_POINT_API_ENV_FILENAME
DEFAULT_MAX_IMAGE_SIZE = GRASP_POINT_API_MAX_IMAGE_SIZE
GEMINI_API_KEY_NAME = GRASP_POINT_API_KEY_NAME


@dataclass(frozen=True)
class APIGraspResult:
    """A single API-selected grasp pose in crop-image coordinates."""

    point: tuple[float, float]
    orientation_rpy_deg: tuple[float, float, float]
    confidence: float | None
    raw_text: str = ""


class GeminiGraspAPIClient:
    """Small Gemini client dedicated to one-shot grasp pose prediction."""

    def __init__(
        self,
        model_id: str | None = None,
        max_image_size: int = DEFAULT_MAX_IMAGE_SIZE,
        max_output_tokens: int = GRASP_POINT_API_MAX_OUTPUT_TOKENS,
        timeout_sec: float = GRASP_POINT_API_TIMEOUT_SEC,
        env_file: str | None = None,
        base_url: str | None = None,
        temperature: float = GRASP_POINT_API_TEMPERATURE,
        thinking_budget: int = GRASP_POINT_API_THINKING_BUDGET,
    ):
        self.model_id = model_id or DEFAULT_GEMINI_MODEL
        self.max_image_size = int(max_image_size)
        self.max_output_tokens = int(max_output_tokens)
        self.timeout_sec = float(timeout_sec)
        self.env_file = env_file or self._default_env_file()
        self.base_url = base_url or DEFAULT_GEMINI_BASE_URL
        self.temperature = float(temperature)
        self.thinking_budget = int(thinking_budget)
        self.parser = Parser()

    def select_grasp_pose(
        self,
        image: Image.Image,
        object_label: str,
        user_request: str | None = None,
        bbox_xyxy: tuple[int, int, int, int] | None = None,
    ) -> APIGraspResult:
        original_width, original_height = image.size
        work_image = self._prepare_image(image)
        work_width, work_height = work_image.size
        prompt = build_grasp_prompt(
            object_label=object_label,
            user_request=user_request,
            image_size=work_image.size,
            bbox_xyxy=bbox_xyxy,
        )
        text = self._ask_gemini(work_image, prompt)
        data = self.parser.as_mapping(self.parser.parse_json(text))
        x_px = self.parser.as_float(data.get("x_px"))
        y_px = self.parser.as_float(data.get("y_px"))
        roll = self.parser.as_float(data.get("roll_deg")) or 0.0
        pitch = self.parser.as_float(data.get("pitch_deg")) or 0.0
        yaw = self.parser.as_float(data.get("yaw_deg")) or 0.0

        if x_px is None or y_px is None:
            raise RuntimeError(f"API response has no grasp pixel: {text}")

        point, yaw = self.parser.correct_vlm_output(
            (x_px, y_px),
            yaw,
            work_width,
            work_height,
        )
        scale_x = original_width / float(work_width)
        scale_y = original_height / float(work_height)
        return APIGraspResult(
            point=(point[0] * scale_x, point[1] * scale_y),
            orientation_rpy_deg=(
                self.parser.normalize_angle_deg(roll),
                self.parser.normalize_angle_deg(pitch),
                self._clamp_yaw_delta_deg(yaw),
            ),
            confidence=self.parser.as_float(data.get("confidence")),
            raw_text=text,
        )

    def _ask_gemini(self, image: Image.Image, prompt: str) -> str:
        api_key = self._api_key()
        if not api_key:
            raise RuntimeError(
                "Gemini API key not found. "
                f"Set {GEMINI_API_KEY_NAME} in {self.env_file}."
            )

        image_b64 = self._encode_jpeg_base64(image)
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
                "maxOutputTokens": self.max_output_tokens,
                "temperature": self.temperature,
                "thinkingConfig": {"thinkingBudget": self.thinking_budget},
            },
        }
        body = self._post_json(
            url,
            payload,
            {
                "x-goog-api-key": api_key,
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
                f"Gemini API request failed: status={exc.code}, detail={detail[:500]}"
            ) from exc
        except error.URLError as exc:
            raise RuntimeError(f"Gemini API request failed: {exc}") from exc
        except TimeoutError as exc:
            raise RuntimeError("Gemini API request timeout") from exc

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
            if key and value:
                os.environ.setdefault(key, value)

    @staticmethod
    def _default_env_file() -> str:
        return str(resolve_env_file(DEFAULT_ENV_FILENAME))

    def _prepare_image(self, image: Image.Image) -> Image.Image:
        image = image.convert("RGB")
        width, height = image.size
        longest = max(width, height)
        if longest <= self.max_image_size:
            return image
        scale = self.max_image_size / float(longest)
        new_size = (int(width * scale), int(height * scale))
        return image.resize(new_size, Image.Resampling.LANCZOS)

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
            raw_response = json.dumps(data, ensure_ascii=False, indent=2)
            raise RuntimeError(f"Gemini response has no text:\n{raw_response}")
        return text

    @staticmethod
    def _clamp_yaw_delta_deg(value: float) -> float:
        yaw = ((float(value) + 90.0) % 180.0) - 90.0
        return 90.0 if yaw == -90.0 and float(value) > 0.0 else yaw
