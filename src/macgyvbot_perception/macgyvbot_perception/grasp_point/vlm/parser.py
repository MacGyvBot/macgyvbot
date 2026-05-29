"""Parsing helpers for local VLM grasp outputs."""

from __future__ import annotations

import json
import math
import re
from typing import Any

from macgyvbot_perception.grasp_point.vlm.models import GridChoice, VLMResult


class Parser:
    """Parse VLM-only and grid VLM outputs with shared helpers."""

    def parse_json(self, text: str) -> Any | None:
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

    def parse_vlm_only_result(
        self,
        result: VLMResult,
        width: int,
        height: int,
    ) -> tuple[tuple[float, float], float] | None:
        data = self.as_mapping(result.data)
        point = self.extract_point_px(data, result.text, width, height)
        yaw = self.extract_yaw_deg(data, result.text)
        if point is None or yaw is None:
            return None
        return self.correct_vlm_output(point, yaw, width, height)

    def parse_vlm_grid_result(
        self,
        result: VLMResult,
        rows: int,
        cols: int,
        image_size: tuple[int, int],
    ) -> GridChoice | None:
        data = self.as_mapping(result.data)
        row = self.as_int(data.get("row"))
        col = self.as_int(data.get("col"))
        cell = self.as_int(data.get("cell"))

        if cell is None:
            cell = self.as_int(data.get("cell_id"))
        if cell is None:
            cell = self.as_int(data.get("grid_cell"))

        if (row is None or col is None) and data:
            encoded = json.dumps(data, ensure_ascii=False)
            rc = self.extract_row_col_from_text(encoded)
            if rc is not None:
                row, col = rc
            elif cell is None:
                cell = self.extract_first_int(encoded)

        if row is None or col is None or cell is None:
            text_row_col = self.extract_row_col_from_text(result.text)
            if text_row_col is not None:
                row, col = text_row_col
            elif cell is None:
                cell = self.extract_first_int(result.text)

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
            confidence=self.as_float(data.get("confidence")),
            reason=str(data.get("reason", "")),
            raw_text=result.text,
        )

    def parse_vlm_precise_result(
        self,
        result: VLMResult,
        width: int,
        height: int,
    ) -> tuple[tuple[float, float], float] | None:
        data = self.as_mapping(result.data)
        point = self.extract_point_px(data, result.text, width, height)
        yaw = self.extract_yaw_deg(data, result.text)
        if point is None or yaw is None:
            return None
        return self.correct_vlm_output(point, yaw, width, height)

    def parse_vlm_orientation_result(
        self,
        result: VLMResult,
        yaw_fallback_deg: float | None = None,
    ) -> tuple[float, float, float]:
        data = self.as_mapping(result.data)
        roll = self.as_float(data.get("roll_deg"))
        pitch = self.as_float(data.get("pitch_deg"))
        yaw = self.as_float(data.get("yaw_deg"))

        if roll is None:
            roll = self.as_float(data.get("roll"))
        if pitch is None:
            pitch = self.as_float(data.get("pitch"))
        if yaw is None:
            yaw = self.as_float(data.get("yaw"))

        if roll is None or pitch is None or yaw is None:
            parsed = self.extract_orientation_from_text(result.text)
            if parsed is not None:
                roll, pitch, yaw = parsed

        if roll is None:
            roll = 0.0
        if pitch is None:
            pitch = 0.0
        if yaw is None:
            yaw = yaw_fallback_deg if yaw_fallback_deg is not None else 0.0

        return (
            self.normalize_angle_deg(roll),
            self.normalize_angle_deg(pitch),
            self.normalize_angle_deg(yaw),
        )

    def correct_vlm_output(
        self,
        point: tuple[float, float],
        yaw_deg: float,
        width: int,
        height: int,
    ) -> tuple[tuple[float, float], float]:
        return (
            self.clamp_point_to_image(point[0], point[1], width, height),
            self.normalize_angle_deg(yaw_deg),
        )

    @staticmethod
    def as_mapping(value: Any) -> dict[str, Any]:
        if isinstance(value, dict):
            return value
        return {}

    @staticmethod
    def as_int(value) -> int | None:
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
    def as_float(value) -> float | None:
        if value is None:
            return None
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(result):
            return None
        return result

    @staticmethod
    def extract_first_int(text: str) -> int | None:
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
    def extract_row_col_from_text(text: str) -> tuple[int, int] | None:
        if not text:
            return None
        row_match = re.search(r"row\s*[:=]?\s*(-?\d+)", text, re.IGNORECASE)
        col_match = re.search(r"col(?:umn)?\s*[:=]?\s*(-?\d+)", text, re.IGNORECASE)
        if row_match and col_match:
            try:
                return int(row_match.group(1)), int(col_match.group(1))
            except (TypeError, ValueError):
                return None

        tuple_match = re.search(r"\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\)", text)
        if tuple_match:
            try:
                return int(tuple_match.group(1)), int(tuple_match.group(2))
            except (TypeError, ValueError):
                return None
        return None

    @staticmethod
    def extract_orientation_from_text(text: str) -> tuple[float, float, float] | None:
        if not text:
            return None
        roll_match = re.search(
            r"roll(?:_deg)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)",
            text,
            re.IGNORECASE,
        )
        pitch_match = re.search(
            r"pitch(?:_deg)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)",
            text,
            re.IGNORECASE,
        )
        yaw_match = re.search(
            r"yaw(?:_deg)?\s*[:=]?\s*(-?\d+(?:\.\d+)?)",
            text,
            re.IGNORECASE,
        )
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

    def extract_yaw_deg(self, data: dict[str, Any], raw_text: str) -> float | None:
        for key in ("yaw_deg", "yaw", "angle_deg", "grasp_angle_deg"):
            value = self.as_float(data.get(key))
            if value is not None:
                return value
        yaw_match = re.search(
            r'"?yaw(?:_deg)?"?\s*[:=]\s*(-?\d+(?:\.\d+)?)',
            raw_text,
            re.IGNORECASE,
        )
        if yaw_match:
            try:
                return float(yaw_match.group(1))
            except (TypeError, ValueError):
                return None
        return None

    def extract_point_px(
        self,
        data: dict[str, Any],
        raw_text: str,
        width: int,
        height: int,
    ) -> tuple[float, float] | None:
        for x_key, y_key in (
            ("x_px", "y_px"),
            ("x", "y"),
            ("u", "v"),
            ("pixel_x", "pixel_y"),
        ):
            x = self.as_int(data.get(x_key))
            y = self.as_int(data.get(y_key))
            if x is not None and y is not None:
                return self.clamp_point_to_image(float(x), float(y), width, height)

        x_norm = self.as_float(data.get("x_norm"))
        y_norm = self.as_float(data.get("y_norm"))
        if x_norm is not None and y_norm is not None:
            x = x_norm * (width - 1)
            y = y_norm * (height - 1)
            return self.clamp_point_to_image(x, y, width, height)

        x_match = re.search(
            r'"?x(?:_px)?"?\s*[:=]\s*(-?\d+(?:\.\d+)?)',
            raw_text,
            re.IGNORECASE,
        )
        y_match = re.search(
            r'"?y(?:_px)?"?\s*[:=]\s*(-?\d+(?:\.\d+)?)',
            raw_text,
            re.IGNORECASE,
        )
        if x_match and y_match:
            try:
                return self.clamp_point_to_image(
                    float(x_match.group(1)),
                    float(y_match.group(1)),
                    width,
                    height,
                )
            except (TypeError, ValueError):
                return None

        tuple_match = re.search(
            r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)",
            raw_text,
        )
        if tuple_match:
            try:
                return self.clamp_point_to_image(
                    float(tuple_match.group(1)),
                    float(tuple_match.group(2)),
                    width,
                    height,
                )
            except (TypeError, ValueError):
                return None
        return None

    @staticmethod
    def normalize_angle_deg(angle_deg: float) -> float:
        return ((float(angle_deg) + 180.0) % 360.0) - 180.0

    @staticmethod
    def clamp_point_to_image(
        x: float,
        y: float,
        width: int,
        height: int,
    ) -> tuple[float, float]:
        cx = min(max(float(x), 0.0), float(max(width - 1, 0)))
        cy = min(max(float(y), 0.0), float(max(height - 1, 0)))
        return (cx, cy)

