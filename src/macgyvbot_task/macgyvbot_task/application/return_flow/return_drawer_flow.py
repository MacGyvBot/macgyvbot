"""Store a returned tool in the hardcoded drawer."""
from __future__ import annotations

from macgyvbot_config.drawer import get_tool_drawer_id
from macgyvbot_config.timing import GRIPPER_OPEN_WAIT_SEC
from macgyvbot_domain import DetectedTarget
from macgyvbot_manipulation.force_detection import ForceReactionDetector
from macgyvbot_manipulation.grasp_verifier import GraspVerifier
from macgyvbot_manipulation.handover_targeting import move_to_observation_pose
from macgyvbot_manipulation.robot_pose import get_ee_matrix, make_safe_pose
from macgyvbot_task.application.drawer_flow.drawer_sequence import DrawerInteraction


class ReturnDrawerFlow:
    """Open drawer → pick tool from home → place in drawer → close drawer → home."""

    def __init__(self, robot, motion_controller, gripper, state, reporter, wait_fn, tool_hold_monitor=None):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.reporter = reporter
        self.wait_fn = wait_fn
        self.tool_hold_monitor = tool_hold_monitor
        self.force_detector = ForceReactionDetector(motion_controller, state, wait_fn)
        self.grasp_verifier = GraspVerifier(gripper, wait_fn)
        self.drawer = DrawerInteraction(
            robot=robot,
            motion_controller=motion_controller,
            gripper=gripper,
            state=state,
            detector=None,
            drawer_detector=None,
            depth_projector=None,
            wait_fn=wait_fn,
        )

    def run(self, tool_name, command, logger):
        picked_from_home = False
        placed_in_drawer = False
        motion = None

        try:
            try:
                drawer_id = get_tool_drawer_id(tool_name)
            except ValueError as exc:
                self.reporter.fail(
                    tool_name,
                    f"서랍 ID 설정 오류: {exc}",
                    "invalid_drawer_floor_config",
                    command,
                    logger,
                )
                return False
            ok, _ = move_to_observation_pose(self.motion, self.robot, logger)
            if not ok:
                self.reporter.fail(
                    tool_name,
                    "서랍 반납 전 관찰 자세 이동 실패",
                    "drawer_return_observation_failed",
                    command,
                    logger,
                )
                return False

            self.reporter.publish(
                "opening_drawer", tool_name, "반납 서랍을 여는 중입니다.", command,
            )
            motion = self.drawer.build_motion(logger, drawer_id=drawer_id)
            if motion is None:
                self.reporter.fail(
                    tool_name,
                    "반납 서랍 손잡이 joint 이동 실패",
                    "drawer_return_joint_move_failed",
                    command,
                    logger,
                )
                return False

            if self.drawer.open_drawer_from_handle(motion, logger) is None:
                self.reporter.fail(
                    tool_name,
                    "반납 서랍 열기 실패",
                    "drawer_return_open_failed",
                    command,
                    logger,
                )
                motion = None
                return False

            self.reporter.publish(
                "picking_from_home",
                tool_name,
                "홈 위치에서 반납 공구를 집는 중입니다.",
                command,
            )
            if not self._pick_from_home(tool_name, command, logger):
                return False
            picked_from_home = True

            self.reporter.publish(
                "placing_in_drawer",
                tool_name,
                "열린 서랍에 반납 공구를 넣는 중입니다.",
                command,
            )
            drawer_target = DetectedTarget(
                label="drawer",
                x=motion.closed_x,
                y=motion.closed_y,
                z=motion.grasp_z,
                depth_m=0.0,
                source="hardcoded_joint",
            )
            logger.info(
                f"서랍 반납 선택: tool={tool_name}, drawer_id={drawer_id}, "
                f"target_z={drawer_target.z:.3f}"
            )
            if not self.drawer.place_tool_in_open_drawer(
                drawer_target,
                logger,
                pre_release_cb=self._stop_monitor_before_release,
            ):
                self.reporter.fail(
                    tool_name,
                    "서랍에 반납 공구 배치 실패",
                    "drawer_return_place_failed",
                    command,
                    logger,
                )
                return False
            placed_in_drawer = True

            return True

        finally:
            # Mirror pick_sequence: only close drawer if gripper is empty (safe).
            # picked_from_home=True but placed_in_drawer=False means gripper still holds the tool.
            gripper_safe = not picked_from_home or placed_in_drawer
            if motion is not None:
                if gripper_safe:
                    ok, _ = move_to_observation_pose(self.motion, self.robot, logger)
                    if ok:
                        self.reporter.publish(
                            "closing_drawer", tool_name, "반납 서랍을 닫는 중입니다.", command,
                        )
                        if self.motion.move_to_drawer_handle_joints(logger, motion.drawer_id):
                            if not self.drawer.close_drawer(motion, logger):
                                logger.error("반납 서랍 닫기 실패")
                        else:
                            logger.error("서랍 닫기 전 손잡이 joint 복귀 실패")
                    else:
                        logger.error("서랍 닫기 전 관찰 자세 이동 실패")
                else:
                    logger.error("그리퍼가 공구를 잡고 있어 서랍을 닫지 않습니다.")

            self.motion.move_to_home_joints(logger)

    def _stop_monitor_before_release(self):
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("drawer_return_place")

    def _pick_from_home(self, tool_name, command, logger):
        logger.info("반납 서랍 - 홈 joint pose 이동 후 공구 파지")
        if not self.motion.move_to_home_joints(logger):
            self.reporter.fail(
                tool_name,
                "홈 joint 이동 실패",
                "drawer_return_home_move_failed",
                command,
                logger,
            )
            return False

        ori = self.state.home_ori
        current_pose = get_ee_matrix(self.robot)
        target_x = float(current_pose[0, 3])
        target_y = float(current_pose[1, 3])
        approach_z = float(current_pose[2, 3])

        self.gripper.open_gripper()
        self.wait_fn(GRIPPER_OPEN_WAIT_SEC)

        stop_z = self.force_detector.descend_until_z_reaction(
            target_x, target_y, approach_z, ori, logger,
        )
        if stop_z is None:
            self.reporter.fail(
                tool_name,
                "홈에서 공구 탐색 하강 실패",
                "drawer_return_home_descend_failed",
                command,
                logger,
            )
            return False

        if not self.grasp_verifier.try_grasp(logger):
            self.reporter.fail(
                tool_name,
                "홈에서 공구 grasp 실패",
                "drawer_return_home_grasp_failed",
                command,
                logger,
            )
            return False

        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.start(tool_name, "return_drawer", command)

        if not self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, approach_z, ori, logger),
        ):
            self.reporter.fail(
                tool_name,
                "홈에서 공구 집기 후 리프트 실패",
                "drawer_return_home_lift_failed",
                command,
                logger,
            )
            return False

        return True
