import numpy as np

from macgyvbot_task.application.adapters.hand_grasp_result_adapter import (
    HandGraspResultAdapter,
)
from macgyvbot_task.application.state.runtime_state import TaskRuntimeState


class FakeDepthProjector:
    def camera_to_base(self, camera_point):
        x, y, z = camera_point
        return x + 1.0, y + 2.0, z + 3.0


class FakeLogger:
    def warn(self, message):
        pass


def test_attach_base_position_from_hand_pixel_and_depth():
    state = TaskRuntimeState(
        logger_provider=lambda: FakeLogger(),
        publish_robot_status=lambda *args, **kwargs: None,
        publish_status_payload=lambda payload: None,
    )
    state.depth_image = np.full((20, 20), 500.0, dtype=np.float32)
    state.intrinsics = {
        "fx": 100.0,
        "fy": 100.0,
        "ppx": 10.0,
        "ppy": 10.0,
    }
    result = {
        "hand_pixel": {
            "u": 10,
            "v": 10,
        },
        "_received_monotonic_sec": 123.0,
    }

    adapter = HandGraspResultAdapter(state, FakeDepthProjector(), FakeLogger())
    adapter.attach_base_position(result)

    assert result["position"] == {
        "x": 1.0,
        "y": 2.0,
        "z": 3.5,
        "frame_id": "base_link",
    }
    assert result["position_observed_monotonic_sec"] == 123.0
