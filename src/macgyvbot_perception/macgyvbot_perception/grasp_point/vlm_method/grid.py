"""Grid policy helpers for the grid-based VLM grasp method."""

from __future__ import annotations

import math

from PIL import Image, ImageDraw, ImageFont

from macgyvbot_perception.grasp_point.vlm.models import GridChoice


class GridPolicy:
    """Draw numbered grids and compute local grasp geometry."""

    def draw_numbered_grid(
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

    def has_converged(
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

    def estimate_grasp_pose(
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

