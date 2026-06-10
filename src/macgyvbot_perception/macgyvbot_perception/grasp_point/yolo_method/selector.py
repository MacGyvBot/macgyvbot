"""Select a grasp point from a YOLO-detected grasp-point bbox."""

from __future__ import annotations

from macgyvbot_config.vlm import GRASP_POINT_MODE_YOLO


DEFAULT_GRASP_POINT_LABELS = (
    "grasp_point",
    "grasp-point",
    "grasp point",
    "grasp",
)


class YoloGraspPointSelector:
    """Find a grasp-point detection and return the center of its bbox."""

    def __init__(self, logger, grasp_point_labels=DEFAULT_GRASP_POINT_LABELS):
        self.logger = logger
        self.grasp_point_labels = {
            self._normalize_label(label) for label in grasp_point_labels
        }

    def select_grasp_pixel(
        self,
        boxes,
        names,
        target_bbox,
    ):
        """Return the center pixel of the best grasp-point bbox, or None."""
        if boxes is None:
            return None

        best_box = None
        best_score = -1.0
        for box in boxes:
            label = self._box_label(box, names)
            if label not in self.grasp_point_labels:
                continue

            bbox = self._box_bbox(box)
            if not self._bbox_center_inside(bbox, target_bbox):
                continue

            score = self._box_confidence(box)
            if score > best_score:
                best_box = bbox
                best_score = score

        if best_box is None:
            return None

        x1, y1, x2, y2 = best_box
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        return u, v, GRASP_POINT_MODE_YOLO, None

    @classmethod
    def _box_label(cls, box, names) -> str:
        try:
            class_id = int(box.cls[0])
        except (TypeError, IndexError):
            class_id = int(box.cls)
        if hasattr(names, "get"):
            return cls._normalize_label(names.get(class_id, class_id))
        return cls._normalize_label(names[class_id])

    @staticmethod
    def _box_bbox(box):
        return [float(value) for value in box.xyxy[0].cpu().numpy()]

    @staticmethod
    def _box_confidence(box) -> float:
        try:
            return float(box.conf[0])
        except (TypeError, IndexError):
            return float(box.conf)

    @staticmethod
    def _bbox_center_inside(bbox, container_bbox) -> bool:
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        tx1, ty1, tx2, ty2 = [float(value) for value in container_bbox]
        return tx1 <= cx <= tx2 and ty1 <= cy <= ty2

    @staticmethod
    def _normalize_label(label) -> str:
        return str(label or "").strip().lower().replace(" ", "_").replace("-", "_")
