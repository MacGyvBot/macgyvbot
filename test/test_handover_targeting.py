import sys
import threading
import time
import types
import unittest
from importlib import import_module


class DummyRobotState:
    pass


class DummyPoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


moveit_module = types.ModuleType("moveit")
moveit_core_module = types.ModuleType("moveit.core")
moveit_robot_state_module = types.ModuleType("moveit.core.robot_state")
moveit_robot_state_module.RobotState = DummyRobotState
moveit_module.core = moveit_core_module
moveit_core_module.robot_state = moveit_robot_state_module
sys.modules.setdefault("moveit", moveit_module)
sys.modules.setdefault("moveit.core", moveit_core_module)
sys.modules.setdefault("moveit.core.robot_state", moveit_robot_state_module)

geometry_module = types.ModuleType("geometry_msgs")
geometry_msg_module = types.ModuleType("geometry_msgs.msg")
geometry_msg_module.PoseStamped = DummyPoseStamped
geometry_module.msg = geometry_msg_module
sys.modules.setdefault("geometry_msgs", geometry_module)
sys.modules.setdefault("geometry_msgs.msg", geometry_msg_module)

numpy_module = types.ModuleType("numpy")
numpy_module.asarray = lambda value, dtype=None: value
sys.modules.setdefault("numpy", numpy_module)

scipy_module = types.ModuleType("scipy")
scipy_spatial_module = types.ModuleType("scipy.spatial")
scipy_transform_module = types.ModuleType("scipy.spatial.transform")
scipy_transform_module.Rotation = types.SimpleNamespace(
    from_matrix=lambda matrix: types.SimpleNamespace(
        as_quat=lambda: (0.0, 0.0, 0.0, 1.0),
    ),
)
scipy_module.spatial = scipy_spatial_module
scipy_spatial_module.transform = scipy_transform_module
sys.modules.setdefault("scipy", scipy_module)
sys.modules.setdefault("scipy.spatial", scipy_spatial_module)
sys.modules.setdefault("scipy.spatial.transform", scipy_transform_module)

handover_targeting = import_module(
    "macgyvbot.util.macgyvbot_main.model_control.handover_targeting"
)
TargetCandidate = handover_targeting.TargetCandidate
_observe_stable_candidate = handover_targeting._observe_stable_candidate
move_to_candidate_with_offset = handover_targeting.move_to_candidate_with_offset


class FakeLogger:
    def info(self, message):
        pass

    def warn(self, message):
        pass

    def warning(self, message):
        pass


class FakeState:
    def __init__(self):
        self.last_grasp_result = None


class FakeMotion:
    def __init__(self, results):
        self.results = list(results)
        self.targets = []

    def plan_and_execute(self, logger, pose_goal=None, state_goal=None):
        self.targets.append(
            (
                pose_goal.pose.position.x,
                pose_goal.pose.position.y,
                pose_goal.pose.position.z,
            )
        )
        return self.results.pop(0)


class TestHandoverTargeting(unittest.TestCase):
    def test_stale_single_frame_does_not_become_stable(self):
        state = FakeState()
        state.last_grasp_result = {
            "position": {
                "x": 0.4,
                "y": 0.2,
                "z": 0.3,
                "frame_id": "base_link",
            },
            "position_observed_monotonic_sec": time.monotonic(),
        }

        candidate = _observe_stable_candidate(
            state,
            FakeLogger(),
            timeout_sec=0.08,
            poll_sec=0.01,
            stable_duration_sec=0.03,
            stable_tolerance_m=0.03,
        )

        self.assertFalse(candidate.found)

    def test_fresh_stable_frames_become_candidate(self):
        state = FakeState()
        start = time.monotonic()

        def feed_frames():
            for index in range(8):
                state.last_grasp_result = {
                    "position": {
                        "x": 0.4 + index * 0.001,
                        "y": 0.2,
                        "z": 0.3,
                        "frame_id": "base_link",
                    },
                    "position_observed_monotonic_sec": start + index * 0.02,
                }
                time.sleep(0.01)

        feeder = threading.Thread(target=feed_frames)
        feeder.start()
        try:
            candidate = _observe_stable_candidate(
                state,
                FakeLogger(),
                timeout_sec=0.5,
                poll_sec=0.005,
                stable_duration_sec=0.06,
                stable_tolerance_m=0.03,
            )
        finally:
            feeder.join()

        self.assertTrue(candidate.found)
        self.assertAlmostEqual(candidate.x, 0.403, places=2)

    def test_failed_handoff_planning_retries_with_safer_targets(self):
        motion = FakeMotion([False, False, True])
        candidate = TargetCandidate(
            found=True,
            x=0.5,
            y=0.18,
            z=0.25,
            frame_id="base_link",
            source="test",
        )

        ok, final_pose, reason = move_to_candidate_with_offset(
            motion,
            candidate,
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            FakeLogger(),
            x_offset_m=0.0,
            z_offset_m=0.08,
        )

        self.assertTrue(ok)
        self.assertEqual(reason, "")
        self.assertEqual(len(motion.targets), 3)
        self.assertAlmostEqual(motion.targets[0][2], 0.39)
        self.assertLess(motion.targets[1][0], motion.targets[0][0])
        self.assertLess(abs(motion.targets[1][1]), abs(motion.targets[0][1]))
        self.assertAlmostEqual(motion.targets[1][2], 0.39)
        self.assertEqual(final_pose.x, motion.targets[2][0])


if __name__ == "__main__":
    unittest.main()
