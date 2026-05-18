"""Gemini API-backed VLM grasp point selection."""

from __future__ import annotations

import base64
import io
import json
import os
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


DEFAULT_GEMINI_MODEL = "gemini-2.5-flash"
DEFAULT_GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta"
DEFAULT_GEMINI_API_KEY_ENV = "GEMINI_API_KEY"


class APIVLMGraspPointSelector:
    """Select grasp pixels using the Gemini vision API."""

    def __init__(
        self,
        logger,
        model: str | None = None,
        api_key_env: str | None = None,
        base_url: str | None = None,
        timeout_sec: float = 30.0,
    ):
        self.logger = logger
        self.model = GeminiVLMModel(
            model_id=model,
            api_key_env=api_key_env,
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
        api_key_env: str | None = None,
        base_url: str | None = None,
        temperature: float = 0.0,
    ):
        self.model_id = model_id or DEFAULT_GEMINI_MODEL
        self.max_image_size = max_image_size
        self.max_new_tokens = max_new_tokens
        self.timeout_sec = float(timeout_sec)
        self.api_key_env = api_key_env or DEFAULT_GEMINI_API_KEY_ENV
        self.base_url = base_url or DEFAULT_GEMINI_BASE_URL
        self.temperature = float(temperature)

    def load(self):
        if not self._api_key():
            raise RuntimeError(
                f"{self.api_key_env} is not set for Gemini API VLM mode."
            )

    def get_runtime_info(self):
        return {
            "provider": "gemini",
            "model": self.model_id,
            "base_url": self.base_url,
            "api_key_env": self.api_key_env,
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
        return os.environ.get(self.api_key_env, "").strip()

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
