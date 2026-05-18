"""Gemini API-based grasp point selection."""

from __future__ import annotations

import base64
import io
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib import error, request

import cv2
from PIL import Image

from macgyvbot_config.vlm import GRASP_POINT_MODE_API
from macgyvbot_perception.grasp_point.vlm_grasp_point_selector import VLMModel

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


DEFAULT_GEMINI_MODEL = "gemini-3-flash-preview"
DEFAULT_GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta"
DEFAULT_ENV_FILENAME = ".env"
DEFAULT_MAX_IMAGE_SIZE = 640
DEFAULT_MAX_RETRIES = 3
GEMINI_API_KEY_NAME = "GEMINI_API_KEY"


@dataclass(frozen=True)
class APIGraspResult:
    """A single API-selected grasp pose in crop-image coordinates."""

    point: tuple[float, float]
    orientation_rpy_deg: tuple[float, float, float]
    confidence: float | None
    reason: str
    raw_text: str


class APIGraspPointSelector:
    """Select grasp pixels using one Gemini API prompt."""

    def __init__(
        self,
        logger,
        model: str | None = None,
        env_file: str | None = None,
        base_url: str | None = None,
        timeout_sec: float = 30.0,
    ):
        self.logger = logger
        self.client = GeminiGraspAPIClient(
            logger=logger,
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
        x1, y1, x2, y2 = self._clamp_bbox_to_image(bbox, color_image)

        if x2 <= x1 or y2 <= y1:
            self.logger.warn("API grasp crop bbox가 비어 있습니다.")
            return None

        self.logger.info(
            f"API grasp crop 준비: bbox=({x1}, {y1}, {x2}, {y2}), "
            f"label={label}, target={target_label}"
        )

        crop_bgr = color_image[y1:y2, x1:x2]
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        crop_image = Image.fromarray(crop_rgb)

        try:
            result = self.client.select_grasp_pose(
                crop_image,
                object_label=label,
                user_request=target_label,
            )
        except Exception as exc:
            self.logger.warn(f"API grasp point 추론 실패: {exc}")
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
                f"API grasp depth 보정 적용: pixel=({u}, {v}), "
                f"depth={refined.depth_m:.4f}m, radius={refined.radius_px}px"
            )
        else:
            self.logger.warn("API grasp depth 보정 결과가 없어 원래 pixel을 사용합니다.")

        self.logger.info(
            f"API grasp point 선택: pixel=({u}, {v}), "
            f"rpy_deg={result.orientation_rpy_deg}, "
            f"confidence={result.confidence}, source={source}, "
            f"reason={result.reason}"
        )

        return u, v, source, result.orientation_rpy_deg

    @staticmethod
    def _clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(math.floor(bbox[0]))))
        y1 = max(0, min(height - 1, int(math.floor(bbox[1]))))
        x2 = max(0, min(width, int(math.ceil(bbox[2]))))
        y2 = max(0, min(height, int(math.ceil(bbox[3]))))
        return x1, y1, x2, y2


class GeminiGraspAPIClient:
    """Small Gemini client dedicated to one-shot grasp pose prediction."""

    def __init__(
        self,
        logger=None,
        model_id: str | None = None,
        max_image_size: int = DEFAULT_MAX_IMAGE_SIZE,
        max_output_tokens: int = 256,
        timeout_sec: float = 30.0,
        env_file: str | None = None,
        base_url: str | None = None,
        temperature: float = 0.0,
        max_retries: int = DEFAULT_MAX_RETRIES,
    ):
        self.logger = logger
        self.model_id = model_id or DEFAULT_GEMINI_MODEL
        self.max_image_size = int(max_image_size)
        self.max_output_tokens = int(max_output_tokens)
        self.timeout_sec = float(timeout_sec)
        self.env_file = env_file or self._default_env_file()
        self.base_url = base_url or DEFAULT_GEMINI_BASE_URL
        self.temperature = float(temperature)
        self.max_retries = max(1, int(max_retries))

    def select_grasp_pose(
        self,
        image: Image.Image,
        object_label: str,
        user_request: str | None = None,
    ) -> APIGraspResult:
        original_width, original_height = image.size
        work_image = self._prepare_image(image)
        work_width, work_height = work_image.size
        prompt = self._build_grasp_prompt(
            object_label=object_label,
            user_request=user_request,
            image_size=work_image.size,
        )

        failures = []
        for attempt in range(1, self.max_retries + 1):
            try:
                self._log_info(
                    f"Gemini API grasp 요청 시도 {attempt}/{self.max_retries}: "
                    f"model={self.model_id}, object={object_label}, "
                    f"image={work_width}x{work_height}"
                )
                text = self._ask_gemini(work_image, prompt)
                self._log_info(f"Gemini API raw response:\n{text}")
                self._log_info(
                    "Gemini API 응답 수신: "
                    f"{self._shorten_text(text, max_len=300)}"
                )
                data = self._extract_json(text)
                result = self._result_from_data(
                    data=data,
                    text=text,
                    original_size=(original_width, original_height),
                    work_size=(work_width, work_height),
                )
                self._log_info(
                    "Gemini API 응답 파싱 성공: "
                    f"point={result.point}, "
                    f"rpy={result.orientation_rpy_deg}, "
                    f"confidence={result.confidence}, reason={result.reason}"
                )
                return result
            except Exception as exc:
                reason = str(exc)
                failures.append(f"{attempt}차 실패: {reason}")
                self._log_warn(
                    f"Gemini API grasp 시도 {attempt}/{self.max_retries} 실패: "
                    f"{reason}"
                )

        raise RuntimeError(
            f"Gemini API grasp point 추론이 {self.max_retries}회 모두 실패했습니다. "
            f"실패 이유: {' | '.join(failures)}"
        )

    def _result_from_data(
        self,
        data: dict[str, Any],
        text: str,
        original_size: tuple[int, int],
        work_size: tuple[int, int],
    ) -> APIGraspResult:
        original_width, original_height = original_size
        work_width, work_height = work_size
        x_px = self._as_float(data.get("x_px"))
        y_px = self._as_float(data.get("y_px"))
        roll = self._as_float(data.get("roll_deg"), default=0.0)
        pitch = self._as_float(data.get("pitch_deg"), default=0.0)
        yaw = self._as_float(data.get("yaw_deg"), default=0.0)

        if x_px is None or y_px is None:
            raise RuntimeError(f"API 응답에 grasp pixel이 없습니다: {text}")

        x_px = min(max(float(x_px), 0.0), float(work_width - 1))
        y_px = min(max(float(y_px), 0.0), float(work_height - 1))
        scale_x = original_width / float(work_width)
        scale_y = original_height / float(work_height)

        return APIGraspResult(
            point=(x_px * scale_x, y_px * scale_y),
            orientation_rpy_deg=(
                self._normalize_angle_deg(roll),
                self._normalize_angle_deg(pitch),
                self._normalize_angle_deg(yaw),
            ),
            confidence=self._as_float(data.get("confidence")),
            reason=str(data.get("reason", "")),
            raw_text=text,
        )

    @staticmethod
    def _build_grasp_prompt(
        object_label: str,
        user_request: str | None,
        image_size: tuple[int, int],
    ) -> str:
        width, height = image_size
        request_text = user_request or "Pick up the object in a natural way."
        return (
            "You are selecting a grasp pose for a robot with a two-finger "
            "parallel gripper.\n"
            "The image is a crop of one detected object. Choose one precise "
            "pixel inside the image as the grasp center and estimate the "
            "end-effector orientation.\n\n"
            "Return strict JSON only. Do not return markdown or code fences.\n"
            "Required keys: x_px, y_px, roll_deg, pitch_deg, yaw_deg, "
            "confidence, reason.\n"
            f"Image size: width={width}, height={height}.\n"
            "Constraints:\n"
            f"- x_px must be a number in [0, {width - 1}]\n"
            f"- y_px must be a number in [0, {height - 1}]\n"
            "- roll_deg, pitch_deg, yaw_deg must be degrees in [-180, 180]\n"
            "- Prefer sturdy, safe, functional grasp regions such as handles "
            "or thick bodies\n"
            "- Avoid blades, sharp tips, holes, fragile parts, slippery ends, "
            "and task-critical surfaces\n"
            "- If roll or pitch is uncertain, use 0.0, but still estimate "
            "yaw_deg\n\n"
            "Example output:\n"
            "{"
            "\"x_px\": 320, "
            "\"y_px\": 180, "
            "\"roll_deg\": 0.0, "
            "\"pitch_deg\": 0.0, "
            "\"yaw_deg\": 25.0, "
            "\"confidence\": 0.82, "
            "\"reason\": \"sturdy handle region suitable for a two-finger grasp\""
            "}\n\n"
            f"Object: {object_label}\n"
            f"Robot task: {request_text}\n"
        )

    def _ask_gemini(self, image: Image.Image, prompt: str) -> str:
        api_key = self._api_key()
        if not api_key:
            raise RuntimeError(
                "Gemini API key를 찾을 수 없습니다. "
                f"{self.env_file} 파일에 {GEMINI_API_KEY_NAME}를 설정하세요."
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
                "Gemini API 요청 실패: "
                f"status={exc.code}, detail={detail[:500]}"
            ) from exc
        except error.URLError as exc:
            raise RuntimeError(f"Gemini API 요청 실패: {exc}") from exc
        except TimeoutError as exc:
            raise RuntimeError("Gemini API 요청 timeout") from exc

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
            raise RuntimeError(f"Gemini 응답에 text가 없습니다: {data}")
        return text

    @staticmethod
    def _extract_json(text: str) -> dict[str, Any]:
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            start = text.find("{")
            end = text.rfind("}")
            if start == -1 or end == -1 or end <= start:
                raise RuntimeError(f"API 응답 JSON 파싱 실패: {text}") from None
            try:
                data = json.loads(text[start : end + 1])
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"API 응답 JSON 파싱 실패: {text}") from exc

        if not isinstance(data, dict):
            raise RuntimeError(f"API 응답 JSON object가 아닙니다: {text}")
        return data

    @staticmethod
    def _as_float(value, default=None) -> float | None:
        if value is None:
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _normalize_angle_deg(value: float) -> float:
        if value is None or not math.isfinite(value):
            return 0.0
        while value > 180.0:
            value -= 360.0
        while value < -180.0:
            value += 360.0
        return float(value)

    def _log_info(self, message: str):
        if self.logger is not None:
            self.logger.info(message)

    def _log_warn(self, message: str):
        if self.logger is not None:
            self.logger.warn(message)

    @staticmethod
    def _shorten_text(text: str, max_len: int) -> str:
        text = str(text).replace("\n", " ").strip()
        if len(text) <= max_len:
            return text
        return text[: max_len - 3] + "..."
