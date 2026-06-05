"""Reusable drawer open, observe, and close motions."""

from __future__ import annotations

import math

import numpy as np
from moveit.core.robot_state import RobotState

from macgyvbot_config.drawer import (
    DRAWER_CLOSE_LIFT_OFFSET_M,
    DRAWER_GRIPPER_SETTLE_SEC,
    DRAWER_HANDLE_PREAPPROACH_X_OFFSET_M,
    DRAWER_HANDLE_JOINT_DEGREES,
    DRAWER_JOINT_NAMES,
    DRAWER_OBSERVE_OFFSET_XYZ_M,
    DRAWER_OPEN_OFFSET_XYZ_M,
    TOOL_DRAWER_IDS,
)
from macgyvbot_config.robot import EE_LINK
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

    @staticmethod
    def drawer_id_for_tool(tool_name):
        return TOOL_DRAWER_IDS.get(str(tool_name or "").strip())

    @staticmethod
    def supported_tool_labels():
        return set(TOOL_DRAWER_IDS)

    def open_drawer(self, drawer_id, logger):
        """Move to a drawer handle, grip it, pull it open, then release."""
        if not self._move_to_handle_joints(drawer_id, logger):
            return False

        handle_pose = get_ee_matrix(self.robot)
        handle_xyz = self._xyz_from_pose(handle_pose)
        handle_ori = current_ee_orientation(self.robot)
        logger.info(f"drawer {drawer_id} handle xyz={self._format_xyz(handle_xyz)}")

        if not self._close_gripper("drawer/open/grip_handle", logger):
            return False

        if not self._move_by_offset(
            handle_xyz,
            handle_ori,
            DRAWER_OPEN_OFFSET_XYZ_M,
            f"drawer {drawer_id} open",
            logger,
        ):
            return False

        opened_pose = get_ee_matrix(self.robot)
        opened_xyz = self._xyz_from_pose(opened_pose)
        self._opened_drawers[drawer_id] = {
            "xyz": opened_xyz,
            "ori": handle_ori,
        }

        return self._open_gripper("drawer/open/release_handle", logger)

    def observe_drawer(self, drawer_id, logger):
        """Move from the opened handle pose to the drawer observation pose."""
        opened = self._opened_drawers.get(drawer_id)
        if opened is None:
            logger.error(f"drawer {drawer_id} opened pose가 없어 관찰할 수 없습니다.")
            return False

        observe_ori = self._observation_orientation(opened["ori"], logger)
        return self._move_by_offset(
            opened["xyz"],
            observe_ori,
            DRAWER_OBSERVE_OFFSET_XYZ_M,
            f"drawer {drawer_id} observe",
            logger,
        )

    def close_drawer(self, drawer_id, logger):
        """Move to the opened handle pose, grip it, push closed, then release."""
        opened = self._opened_drawers.get(drawer_id)
        if opened is None:
            logger.error(f"drawer {drawer_id} opened pose가 없어 닫을 수 없습니다.")
            return False

        open_handle_xyz = self._open_handle_target_xyz(drawer_id, opened["xyz"])
        if not self._move_to_handle_pose_with_preapproach(
            open_handle_xyz,
            opened["ori"],
            f"drawer {drawer_id} return_to_open_handle",
            logger,
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

    def _move_to_handle_joints(self, drawer_id, logger):
        joint_positions = self._joint_positions(drawer_id)
        if joint_positions is None:
            logger.error(f"지원하지 않는 drawer id입니다: {drawer_id}")
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
        ):
            return False
        return self.motion.plan_and_execute(logger, state_goal=state_goal)

    def _move_to_handle_pose_with_preapproach(self, target_xyz, ori, label, logger):
        if not self._move_to_handle_preapproach(
            target_xyz,
            ori,
            f"{label} preapproach",
            logger,
        ):
            return False
        return self._move_by_offset(
            target_xyz,
            ori,
            [0.0, 0.0, 0.0],
            label,
            logger,
        )

    def _move_to_handle_preapproach(self, target_xyz, ori, label, logger):
        return self._move_by_offset(
            target_xyz,
            ori,
            [DRAWER_HANDLE_PREAPPROACH_X_OFFSET_M, 0.0, 0.0],
            label,
            logger,
        )

    def _move_by_offset(self, base_xyz, ori, offset_xyz, label, logger):
        target_xyz = [
            float(base_xyz[0]) + float(offset_xyz[0]),
            float(base_xyz[1]) + float(offset_xyz[1]),
            float(base_xyz[2]) + float(offset_xyz[2]),
        ]
        logger.info(
            f"{label}: base={self._format_xyz(base_xyz)}, "
            f"offset={self._format_xyz(offset_xyz)}, "
            f"target={self._format_xyz(target_xyz)}"
        )
        if self.dry_run:
            return True

        return self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                target_xyz[0],
                target_xyz[1],
                target_xyz[2],
                ori,
                logger,
            ),
        )

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
            logger.error(f"그리퍼 닫기 실패: {exc}")
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
            logger.error(f"그리퍼 열기 실패: {exc}")
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
        if drawer_id == 0:
            return [("close", close_offset)]

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
        if drawer_id != 0:
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
