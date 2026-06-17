"""Reusable drawer open, observe, and close motions."""

from __future__ import annotations

import math

import numpy as np
from moveit.core.robot_state import RobotState

from macgyvbot_config.drawer import (
    DRAWER_CLOSE_LIFT_OFFSET_M,
    DRAWER_CLOSE_PREPARE_WRIST_J6_DEG,
    DRAWER_GRIPPER_SETTLE_SEC,
    DRAWER_HANDLE_PREAPPROACH_X_OFFSET_M,
    DRAWER_HANDLE_JOINT_DEGREES,
    DRAWER_JOINT_NAMES,
    DRAWER_OBSERVE_OFFSET_XYZ_M,
    DRAWER_OPEN_OFFSET_XYZ_M,
    TOOL_DRAWER_IDS,
)
from macgyvbot_config.robot import EE_LINK
from macgyvbot_config.structured_logging import format_structured_log
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
    make_safe_pose,
    orientation_from_transform,
)


class DrawerMotionFlow:
    """Move to drawer handles and execute drawer open/observe/close steps."""

    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        wait_fn,
        observation_orientation_provider=None,
        dry_run=False,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.wait_fn = wait_fn
        self.observation_orientation_provider = observation_orientation_provider
        self.dry_run = dry_run
        self._opened_drawers = {}
        self._open_handle_targets = {}
        self._active_open_handles = {}

    @staticmethod
    def drawer_id_for_tool(tool_name):
        return TOOL_DRAWER_IDS.get(str(tool_name or "").strip())

    @staticmethod
    def supported_tool_labels():
        return set(TOOL_DRAWER_IDS)

    def open_drawer(self, drawer_id, logger):
        """Move to a drawer handle, grip it, pull it open, then release."""
        if not self.prepare_open_handle_target(drawer_id, logger):
            return False

        if not self.move_to_open_handle_preapproach(drawer_id, logger):
            return False

        if not self.move_to_open_handle_pose(drawer_id, logger):
            return False

        if not self.grip_open_handle(drawer_id, logger):
            return False

        if not self.pull_open_drawer(drawer_id, logger):
            return False

        return self.release_open_handle(drawer_id, logger)

    def prepare_open_handle_target(self, drawer_id, logger):
        """Compute and cache the handle target used by the open-drawer steps."""
        joint_positions = self._joint_positions(drawer_id)
        if joint_positions is None:
            _log_drawer_motion_failed(
                logger,
                stage="handle_joint_config",
                drawer_id=drawer_id,
                reason="unsupported_drawer_id",
            )
            return False

        logger.info(
            f"drawer {drawer_id} handle joint pose 이동: "
            + ", ".join(
                f"{name}={value:.3f}rad"
                for name, value in joint_positions.items()
            )
        )
        if self.dry_run:
            self._open_handle_targets[drawer_id] = {
                "state_goal": None,
                "xyz": [0.0, 0.0, 0.0],
                "ori": None,
            }
            return True

        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = joint_positions
        state_goal.update()
        handle_transform = np.asarray(
            state_goal.get_global_link_transform(EE_LINK),
            dtype=float,
        )
        self._open_handle_targets[drawer_id] = {
            "state_goal": state_goal,
            "xyz": self._xyz_from_pose(handle_transform),
            "ori": orientation_from_transform(handle_transform),
        }
        return True

    def move_to_open_handle_preapproach(self, drawer_id, logger):
        """Move near the drawer handle before the final handle-pose motion."""
        target = self._open_handle_target(drawer_id, logger)
        if target is None:
            return False
        return self._move_to_handle_preapproach(
            target["xyz"],
            target["ori"],
            f"drawer {drawer_id} handle preapproach",
            logger,
            collision_scene_key="drawer/handle_preapproach",
            drawer_id=drawer_id,
        )

    def move_to_open_handle_pose(self, drawer_id, logger):
        """Move to the drawer handle joint pose and cache the actual handle pose."""
        target = self._open_handle_target(drawer_id, logger)
        if target is None:
            return False

        if self.dry_run:
            self._active_open_handles[drawer_id] = {
                "xyz": target["xyz"],
                "ori": target["ori"],
            }
            return True

        ok = self.motion.plan_and_execute(
            logger,
            state_goal=target["state_goal"],
            collision_scene_key="drawer/handle_pose",
        )
        if not ok:
            _log_drawer_motion_failed(
                logger,
                stage="handle_joint_pose",
                drawer_id=drawer_id,
                reason="state_goal_plan_failed",
                target_xyz=target["xyz"],
                collision_scene_key="drawer/handle_pose",
            )
            return False

        handle_pose = get_ee_matrix(self.robot)
        handle_xyz = self._xyz_from_pose(handle_pose)
        handle_ori = current_ee_orientation(self.robot)
        logger.info(
            f"drawer {drawer_id} handle 위치 xyz={self._format_xyz(handle_xyz)}"
        )
        self._active_open_handles[drawer_id] = {
            "xyz": handle_xyz,
            "ori": handle_ori,
        }
        return True

    def grip_open_handle(self, drawer_id, logger):
        """Close the gripper on the drawer handle."""
        return self._close_gripper("drawer/open/grip_handle", logger)

    def pull_open_drawer(self, drawer_id, logger):
        """Pull the drawer open from the cached handle pose."""
        handle = self._active_open_handles.get(drawer_id)
        if handle is None:
            _log_drawer_motion_failed(
                logger,
                stage="open_handle_pose",
                drawer_id=drawer_id,
                reason="handle_pose_missing",
            )
            return False

        if not self._move_by_offset(
            handle["xyz"],
            handle["ori"],
            DRAWER_OPEN_OFFSET_XYZ_M,
            f"drawer {drawer_id} open",
            logger,
            collision_scene_key="drawer/open_pull",
            drawer_id=drawer_id,
        ):
            return False

        if self.dry_run:
            opened_xyz = handle["xyz"]
        else:
            opened_pose = get_ee_matrix(self.robot)
            opened_xyz = self._xyz_from_pose(opened_pose)
        self._opened_drawers[drawer_id] = {
            "xyz": opened_xyz,
            "ori": handle["ori"],
        }
        return True

    def release_open_handle(self, drawer_id, logger):
        """Open the gripper after the drawer has been pulled open."""
        return self._open_gripper("drawer/open/release_handle", logger)

    def observe_drawer(self, drawer_id, logger):
        """Move from the opened handle pose to the drawer observation pose."""
        opened = self._opened_drawers.get(drawer_id)
        if opened is None:
            _log_drawer_motion_failed(
                logger,
                stage="observe_opened_pose",
                drawer_id=drawer_id,
                reason="opened_pose_missing",
            )
            return False

        observe_ori = self._observation_orientation(opened["ori"], logger)
        return self._move_by_offset(
            opened["xyz"],
            observe_ori,
            DRAWER_OBSERVE_OFFSET_XYZ_M,
            f"drawer {drawer_id} observe",
            logger,
            collision_scene_key="drawer/observe",
            drawer_id=drawer_id,
        )

    def close_drawer(self, drawer_id, logger, prepare_wrist=False):
        """Move to the opened handle pose, grip it, push closed, then release."""
        opened = self._opened_drawers.get(drawer_id)
        if opened is None:
            _log_drawer_motion_failed(
                logger,
                stage="close_opened_pose",
                drawer_id=drawer_id,
                reason="opened_pose_missing",
            )
            return False

        open_handle_xyz = self._open_handle_target_xyz(drawer_id, opened["xyz"])
        if prepare_wrist and not self._prepare_wrist_for_close(drawer_id, logger):
            return False

        if not self._move_to_handle_pose_with_preapproach(
            open_handle_xyz,
            opened["ori"],
            f"drawer {drawer_id} return_to_open_handle",
            logger,
            collision_scene_key="drawer/approach_to_close",
            drawer_id=drawer_id,
        ):
            return False

        if not self._close_gripper("drawer/close/grip_handle", logger):
            return False

        for label, offset in self._close_offsets(drawer_id):
            if not self._move_by_offset(
                open_handle_xyz,
                opened["ori"],
                offset,
                f"drawer {drawer_id} {label}",
                logger,
                collision_scene_key="drawer/close_push",
                drawer_id=drawer_id,
            ):
                return False

        if not self._open_gripper("drawer/close/release_handle", logger):
            return False

        self._opened_drawers.pop(drawer_id, None)
        return True

    def open_and_observe(self, drawer_id, logger):
        return self.open_drawer(drawer_id, logger) and self.observe_drawer(
            drawer_id,
            logger,
        )

    def open_and_close(self, drawer_id, logger):
        return self.open_drawer(drawer_id, logger) and self.close_drawer(
            drawer_id,
            logger,
        )

    def _prepare_wrist_for_close(self, drawer_id, logger):
        logger.info(
            f"drawer {drawer_id} close prepare: "
            f"J6를 {DRAWER_CLOSE_PREPARE_WRIST_J6_DEG:.1f}도로 이동합니다."
        )
        if self.dry_run:
            return True

        move_wrist = getattr(self.motion, "move_wrist_to_joint_rad", None)
        if move_wrist is None:
            _log_drawer_motion_failed(
                logger,
                stage="close_prepare_wrist",
                drawer_id=drawer_id,
                reason="wrist_joint_target_api_unavailable",
            )
            return False

        ok = move_wrist(
            math.radians(DRAWER_CLOSE_PREPARE_WRIST_J6_DEG),
            logger,
            collision_scene_key="drawer/close_prepare_wrist",
        )
        if not ok:
            _log_drawer_motion_failed(
                logger,
                stage="close_prepare_wrist",
                drawer_id=drawer_id,
                reason="wrist_joint_target_failed",
                target_j6_deg=DRAWER_CLOSE_PREPARE_WRIST_J6_DEG,
                collision_scene_key="drawer/close_prepare_wrist",
            )
        return ok

    def _open_handle_target(self, drawer_id, logger):
        target = self._open_handle_targets.get(drawer_id)
        if target is not None:
            return target
        if not self.prepare_open_handle_target(drawer_id, logger):
            return None
        return self._open_handle_targets.get(drawer_id)

    def _move_to_handle_joints(self, drawer_id, logger):
        joint_positions = self._joint_positions(drawer_id)
        if joint_positions is None:
            _log_drawer_motion_failed(
                logger,
                stage="handle_joint_config",
                drawer_id=drawer_id,
                reason="unsupported_drawer_id",
            )
            return False

        logger.info(
            f"drawer {drawer_id} handle joint pose 이동: "
            + ", ".join(
                f"{name}={value:.3f}rad"
                for name, value in joint_positions.items()
            )
        )
        if self.dry_run:
            return True

        state_goal = RobotState(self.robot.get_robot_model())
        state_goal.joint_positions = joint_positions
        state_goal.update()
        handle_transform = np.asarray(
            state_goal.get_global_link_transform(EE_LINK),
            dtype=float,
        )
        handle_xyz = self._xyz_from_pose(handle_transform)
        handle_ori = orientation_from_transform(handle_transform)
        if not self._move_to_handle_preapproach(
            handle_xyz,
            handle_ori,
            f"drawer {drawer_id} handle preapproach",
            logger,
            collision_scene_key="drawer/handle_preapproach",
            drawer_id=drawer_id,
        ):
            return False
        ok = self.motion.plan_and_execute(
            logger,
            state_goal=state_goal,
            collision_scene_key="drawer/handle_pose",
        )
        if not ok:
            _log_drawer_motion_failed(
                logger,
                stage="handle_joint_pose",
                drawer_id=drawer_id,
                reason="state_goal_plan_failed",
                target_xyz=handle_xyz,
                collision_scene_key="drawer/handle_pose",
            )
        return ok

    def _move_to_handle_pose_with_preapproach(
        self,
        target_xyz,
        ori,
        label,
        logger,
        collision_scene_key=None,
        drawer_id=None,
    ):
        if not self._move_to_handle_preapproach(
            target_xyz,
            ori,
            f"{label} preapproach",
            logger,
            collision_scene_key=collision_scene_key,
            drawer_id=drawer_id,
        ):
            return False
        return self._move_by_offset(
            target_xyz,
            ori,
            [0.0, 0.0, 0.0],
            label,
            logger,
            collision_scene_key=collision_scene_key,
            drawer_id=drawer_id,
        )

    def _move_to_handle_preapproach(
        self,
        target_xyz,
        ori,
        label,
        logger,
        collision_scene_key=None,
        drawer_id=None,
    ):
        return self._move_by_offset(
            target_xyz,
            ori,
            [DRAWER_HANDLE_PREAPPROACH_X_OFFSET_M, 0.0, 0.0],
            label,
            logger,
            collision_scene_key=collision_scene_key,
            drawer_id=drawer_id,
        )

    def _move_by_offset(
        self,
        base_xyz,
        ori,
        offset_xyz,
        label,
        logger,
        collision_scene_key=None,
        drawer_id=None,
    ):
        target_xyz = [
            float(base_xyz[0]) + float(offset_xyz[0]),
            float(base_xyz[1]) + float(offset_xyz[1]),
            float(base_xyz[2]) + float(offset_xyz[2]),
        ]
        logger.info(
            f"{label}: 기준={self._format_xyz(base_xyz)}, "
            f"offset={self._format_xyz(offset_xyz)}, "
            f"목표={self._format_xyz(target_xyz)}"
        )
        if self.dry_run:
            return True

        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                target_xyz[0],
                target_xyz[1],
                target_xyz[2],
                ori,
                logger,
            ),
            collision_scene_key=collision_scene_key,
        )
        if not ok:
            _log_drawer_motion_failed(
                logger,
                stage=label,
                drawer_id=drawer_id,
                reason="pose_goal_plan_failed",
                base_xyz=base_xyz,
                offset_xyz=offset_xyz,
                target_xyz=target_xyz,
                collision_scene_key=collision_scene_key,
            )
        return ok

    def _observation_orientation(self, fallback_ori, logger):
        if self.observation_orientation_provider is None:
            return fallback_ori

        observe_ori = self.observation_orientation_provider()
        if observe_ori is None:
            logger.warn(
                "drawer observe orientation이 없어 "
                "handle orientation을 사용합니다."
            )
            return fallback_ori

        logger.info("drawer observe orientation: home/approach orientation 사용")
        return observe_ori

    def _close_gripper(self, label, logger):
        logger.info(f"{label}: 그리퍼 닫기")
        if self.dry_run:
            return True
        try:
            self.gripper.close_gripper()
            self.wait_fn(DRAWER_GRIPPER_SETTLE_SEC)
        except Exception as exc:
            _log_drawer_motion_failed(
                logger,
                stage=label,
                reason="gripper_close_failed",
                error=str(exc) or type(exc).__name__,
            )
            return False
        return True

    def _open_gripper(self, label, logger):
        logger.info(f"{label}: 그리퍼 열기")
        if self.dry_run:
            return True
        try:
            self.gripper.open_gripper()
            self.wait_fn(DRAWER_GRIPPER_SETTLE_SEC)
        except Exception as exc:
            _log_drawer_motion_failed(
                logger,
                stage=label,
                reason="gripper_open_failed",
                error=str(exc) or type(exc).__name__,
            )
            return False
        return True

    @staticmethod
    def _joint_positions(drawer_id):
        degrees = DRAWER_HANDLE_JOINT_DEGREES.get(drawer_id)
        if degrees is None or len(degrees) != len(DRAWER_JOINT_NAMES):
            return None
        return {
            name: math.radians(float(value))
            for name, value in zip(DRAWER_JOINT_NAMES, degrees)
        }

    @staticmethod
    def _close_offsets(drawer_id):
        close_offset = [-value for value in DRAWER_OPEN_OFFSET_XYZ_M]
        lifted_offset = [0.0, 0.0, DRAWER_CLOSE_LIFT_OFFSET_M]
        lifted_close_offset = [
            close_offset[0],
            close_offset[1],
            DRAWER_CLOSE_LIFT_OFFSET_M,
        ]
        return [
            ("lift_before_close", lifted_offset),
            ("close_lifted", lifted_close_offset),
        ]

    @staticmethod
    def _open_handle_target_xyz(drawer_id, opened_xyz):
        target_xyz = [float(value) for value in opened_xyz]
        target_xyz[2] -= DRAWER_CLOSE_LIFT_OFFSET_M
        return target_xyz

    @staticmethod
    def _xyz_from_pose(pose):
        return [
            float(pose[0, 3]),
            float(pose[1, 3]),
            float(pose[2, 3]),
        ]

    @staticmethod
    def _format_xyz(values):
        return (
            f"({float(values[0]):.3f}, "
            f"{float(values[1]):.3f}, "
            f"{float(values[2]):.3f})"
        )


def _log_drawer_motion_failed(
    logger,
    *,
    stage,
    reason,
    drawer_id=None,
    base_xyz=None,
    offset_xyz=None,
    target_xyz=None,
    collision_scene_key=None,
    error=None,
):
    fields = {
        "step": "drawer_motion",
        "event": "fail",
        "stage": stage,
        "drawer_id": drawer_id,
        "reason": reason,
        "base_xyz": _format_optional_xyz(base_xyz),
        "offset_xyz": _format_optional_xyz(offset_xyz),
        "target_xyz": _format_optional_xyz(target_xyz),
        "collision_scene_key": collision_scene_key,
        "error": error,
    }
    try:
        logger.error("drawer motion failed", **fields)
    except TypeError:
        logger.error(
            format_structured_log(
                svc="manipulation",
                pipe="drawer",
                msg="drawer motion failed",
                **fields,
            )
        )


def _format_optional_xyz(values):
    if values is None:
        return None
    try:
        return (
            f"({float(values[0]):.3f},"
            f"{float(values[1]):.3f},"
            f"{float(values[2]):.3f})"
        )
    except (TypeError, ValueError, IndexError):
        return str(values)
