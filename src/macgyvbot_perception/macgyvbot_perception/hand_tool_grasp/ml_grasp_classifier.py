from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
from math import sqrt
from pathlib import Path
from typing import Optional

from macgyvbot_perception.model_paths import resolve_weight_path

ACTIVE_GRASP_STATES = {"grasp"}
ACTIVE_MODEL_STATES = {"open", "grasp"}
EXPECTED_LANDMARK_COUNT = 21
FEATURE_COUNT = EXPECTED_LANDMARK_COUNT * 3
MIN_FEATURE_SCALE = 1e-6
WRIST = 0
MIDDLE_MCP = 9


@dataclass(frozen=True)
class MLGraspResult:
    raw_state: str
    stable_state: str
    confidence: Optional[float]
    is_grasp: bool
    enabled: bool = True


class MLHandGraspClassifier:
    """Load a trained .pkl model and smooth raw hand open/grasp states."""

    def __init__(
        self,
        model_path: str,
        stable_window_size: int = 5,
        stable_min_count: int = 3,
    ) -> None:
        try:
            import joblib
        except ImportError as exc:
            raise RuntimeError("joblib is required to load the grasp classifier") from exc

        path = resolve_model_path(model_path)
        if not path.exists():
            raise RuntimeError(f"grasp classifier not found: {path}")

        self.model = joblib.load(path)
        self.path = path
        self.buffer: deque[str] = deque(maxlen=stable_window_size)
        self.stable_min_count = stable_min_count

    def reset(self) -> None:
        self.buffer.clear()

    def update(self, hand_info: Optional[dict]) -> MLGraspResult:
        if hand_info is None:
            self.reset()
            return MLGraspResult(
                raw_state="no_hand",
                stable_state="no_hand",
                confidence=None,
                is_grasp=False,
            )

        features = extract_ml_features(hand_info)
        raw_state = str(self.model.predict([features])[0])
        if raw_state not in ACTIVE_MODEL_STATES:
            raw_state = "unstable"

        confidence = None
        if hasattr(self.model, "predict_proba"):
            probabilities = self.model.predict_proba([features])[0]
            confidence = float(max(probabilities))

        self.buffer.append(raw_state)
        stable_state = compute_stable_state(self.buffer, self.stable_min_count)
        is_grasp = (
            raw_state in ACTIVE_GRASP_STATES
            and stable_state in ACTIVE_GRASP_STATES
        )
        return MLGraspResult(
            raw_state=raw_state,
            stable_state=stable_state,
            confidence=confidence,
            is_grasp=is_grasp,
        )


def disabled_ml_result(reason: str = "disabled") -> MLGraspResult:
    return MLGraspResult(
        raw_state=reason,
        stable_state=reason,
        confidence=None,
        is_grasp=False,
        enabled=False,
    )


def extract_ml_features(hand_info: dict) -> list[float]:
    if "ml_features" in hand_info:
        return list(hand_info["ml_features"])

    landmarks = hand_info["landmarks"]
    if len(landmarks) != EXPECTED_LANDMARK_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_LANDMARK_COUNT} hand landmarks, got {len(landmarks)}."
        )

    wrist = landmarks[WRIST]
    middle_mcp = landmarks[MIDDLE_MCP]
    scale = sqrt(
        (middle_mcp[0] - wrist[0]) ** 2
        + (middle_mcp[1] - wrist[1]) ** 2
    )
    if scale < MIN_FEATURE_SCALE:
        scale = 1.0

    features: list[float] = []
    for index in range(EXPECTED_LANDMARK_COUNT):
        point = landmarks[index]
        features.append(float((point[0] - wrist[0]) / scale))
        features.append(float((point[1] - wrist[1]) / scale))
        features.append(0.0)

    if len(features) != FEATURE_COUNT:
        raise ValueError(f"Expected {FEATURE_COUNT} features, got {len(features)}.")
    return features


def extract_mediapipe_features(hand_landmarks) -> list[float]:
    landmarks = hand_landmarks.landmark
    if len(landmarks) != EXPECTED_LANDMARK_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_LANDMARK_COUNT} hand landmarks, got {len(landmarks)}."
        )

    wrist = landmarks[WRIST]
    middle_mcp = landmarks[MIDDLE_MCP]
    scale = sqrt(
        (middle_mcp.x - wrist.x) ** 2
        + (middle_mcp.y - wrist.y) ** 2
        + (middle_mcp.z - wrist.z) ** 2
    )
    if scale < MIN_FEATURE_SCALE:
        scale = 1.0

    features: list[float] = []
    for point in landmarks:
        features.append(float((point.x - wrist.x) / scale))
        features.append(float((point.y - wrist.y) / scale))
        features.append(float((point.z - wrist.z) / scale))
    return features


def compute_stable_state(buffer: deque[str], stable_min_count: int) -> str:
    if not buffer:
        return "unstable"

    state, count = Counter(buffer).most_common(1)[0]
    if count >= stable_min_count:
        return state
    return "unstable"


def resolve_model_path(model_path: str) -> Path:
    return Path(resolve_weight_path(model_path))
