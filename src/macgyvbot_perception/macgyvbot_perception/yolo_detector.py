"""YOLO detector construction and model path resolution."""

from macgyvbot_perception.model_paths import resolve_weight_path

DEFAULT_MODEL_PATH = "yolov11_best.pt"


def resolve_model_path(model_name):
    return str(resolve_weight_path(model_name, default_model_name=DEFAULT_MODEL_PATH))


class YoloDetector:
    """Small wrapper around Ultralytics YOLO to keep node wiring thin."""

    def __init__(self, model_name, logger=None):
        from ultralytics import YOLO

        self.logger = logger
        self.model_path = resolve_model_path(model_name)
        if self.logger is not None:
            self.logger.info(
                "model_load",
                "start",
                pipe="yolo",
                model=self.model_path,
                msg="YOLO model loading",
            )
        self.model = YOLO(self.model_path)
        if self.logger is not None:
            self.logger.info(
                "model_load",
                "done",
                pipe="yolo",
                model=self.model_path,
                msg="YOLO model loaded",
            )

    @property
    def names(self):
        return self.model.names

    def detect(self, image):
        if self.logger is not None:
            shape = getattr(image, "shape", None)
            self.logger.info(
                "detect",
                "start",
                pipe="yolo",
                image_shape=shape,
                msg="YOLO detection started",
            )
        results = self.model(image, verbose=False)
        if self.logger is not None:
            self.logger.info(
                "detect",
                "done",
                pipe="yolo",
                result_count=len(results) if results is not None else 0,
                msg="YOLO detection completed",
            )
        return results
