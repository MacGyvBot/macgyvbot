"""Pick sequence orchestration."""

import time

import rclpy

from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    clamp_to_safe_workspace,
)

from macgyvbot.config.config import (
    APPROACH_Z_OFFSET,
    COLLISION_MARGIN,
    GRASP_Z_OFFSET,
    HAND_GRASP_TIMEOUT_SEC,
    MAX_DESCENT_FROM_APPROACH,
    MIN_GRASP_CLEARANCE,
    MIN_PICK_Z,
    MIN_TRAVEL_Z,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot.util.macgyvbot_main.task_pipeline.grasp_verifier import (
    GraspVerifier,
)
from macgyvbot.util.macgyvbot_main.task_pipeline.handoff_motion import (
    HandoffMotion,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
    make_safe_pose,
)


class PickSequenceRunner:
    def __init__(self, robot, motion_controller, gripper, state):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.grasp_verifier = GraspVerifier(gripper, self.cooperative_wait)
        self.handoff_motion = HandoffMotion(motion_controller, state)

    def run(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None

        log = self.state.logger()
        ori = self.state.home_ori

        safe_grasp_offset = max(
            GRASP_Z_OFFSET,
            z_m * 0.35 + MIN_GRASP_CLEARANCE,
        )
        safe_grasp_offset += COLLISION_MARGIN

        approach_z = bz + APPROACH_Z_OFFSET + COLLISION_MARGIN
        grasp_z = bz + safe_grasp_offset
        grasp_z = max(grasp_z, MIN_TRAVEL_Z, MIN_PICK_Z)
        if grasp_z > approach_z:
            grasp_z = approach_z - 0.01

        min_safe_grasp_z = approach_z - MAX_DESCENT_FROM_APPROACH
        if grasp_z < min_safe_grasp_z:
            log.warn(
                f"grasp_z({grasp_z:.3f})가 과도하게 낮아 "
                f"하강 제한 적용: {min_safe_grasp_z:.3f}"
            )
            grasp_z = min_safe_grasp_z

        current_pose = get_ee_matrix(self.robot)
        current_x = current_pose[0, 3]
        current_y = current_pose[1, 3]

        target_x, target_y, _ = clamp_to_safe_workspace(bx, by, SAFE_Z, log)
        _, _, travel_z = clamp_to_safe_workspace(
            target_x,
            target_y,
            SAFE_Z,
            log,
        )
        _, _, approach_z = clamp_to_safe_workspace(
            target_x,
            target_y,
            approach_z,
            log,
        )
        _, _, grasp_z = clamp_to_safe_workspace(
            target_x,
            target_y,
            grasp_z,
            log,
        )

        if grasp_z > approach_z:
            log.warn(
                f"안전영역 적용 후 grasp_z({grasp_z:.3f})가 "
                f"approach_z({approach_z:.3f})보다 높아 approach_z로 맞춥니다."
            )
            grasp_z = approach_z

        should_descend_to_grasp = abs(approach_z - grasp_z) > 0.005

        try:
            log.info(
                f"시퀀스 시작: Target({target_x:.3f}, {target_y:.3f}), "
                f"depth={z_m:.3f}, travel_z={travel_z:.3f}, "
                f"approach_z={approach_z:.3f}, grasp_z={grasp_z:.3f}"
            )

            self.gripper.open_gripper()
            self.cooperative_wait(0.5)

            log.info("1단계: 안전 이동 높이 확보")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    current_x,
                    current_y,
                    travel_z,
                    ori,
                    log,
                ),
            )
            if not ok:
                log.error("안전 이동 높이 확보 실패. Pick 시퀀스 중단")
                self.state._publish_robot_status(
                    "failed",
                    message="안전 이동 높이 확보 실패",
                    reason="travel_z_plan_failed",
                    command=self.state.current_command,
                )
                return

            log.info("2단계: 안전 높이에서 XY 수평 이동")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    target_x,
                    target_y,
                    travel_z,
                    ori,
                    log,
                ),
            )
            if not ok:
                log.error("XY 이동 실패. Pick 시퀀스 중단")
                self.state._publish_robot_status(
                    "failed",
                    message="XY 이동 실패",
                    reason="xy_move_failed",
                    command=self.state.current_command,
                )
                return

            if vlm_yaw_deg is not None:
                ok = self.motion.rotate_wrist_by_yaw_deg(vlm_yaw_deg, log)
                if not ok:
                    log.error("J6 회전 실패. Pick 시퀀스 중단")
                    self.state._publish_robot_status(
                        "failed",
                        message="J6 회전 실패",
                        reason="wrist_rotation_failed",
                        command=self.state.current_command,
                    )
                    return
                ori = current_ee_orientation(self.robot)

            log.info("3단계: 타겟 상단 접근")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    target_x,
                    target_y,
                    approach_z,
                    ori,
                    log,
                ),
            )
            if not ok:
                log.error("상단 접근 실패. Pick 시퀀스 중단")
                self.state._publish_robot_status(
                    "failed",
                    message="상단 접근 실패",
                    reason="approach_failed",
                    command=self.state.current_command,
                )
                return

            if should_descend_to_grasp:
                log.info("4단계: 파지 높이 하강")
                ok = self.motion.plan_and_execute(
                    log,
                    pose_goal=make_safe_pose(
                        target_x,
                        target_y,
                        grasp_z,
                        ori,
                        log,
                    ),
                )
                if not ok:
                    log.error("파지 높이 하강 실패. Pick 시퀀스 중단")
                    self.state._publish_robot_status(
                        "failed",
                        message="파지 높이 하강 실패",
                        reason="grasp_descent_failed",
                        command=self.state.current_command,
                    )
                    return
            else:
                log.info("4단계: approach_z와 grasp_z가 같아 추가 하강 생략")

            log.info("5단계: 공구 grasp 시도")
            if not self.try_robot_grasp(log):
                log.error("공구 grasp 실패. Pick 시퀀스 중단")
                self.state._publish_robot_status(
                    "failed",
                    message="공구 grasp에 실패했습니다.",
                    reason="robot_grasp_failed",
                    command=self.state.current_command,
                )
                return

            self.state._publish_robot_status(
                "grasp_success",
                message="공구 grasp에 성공했습니다.",
                command=self.state.current_command,
            )

            log.info("6단계: 안전 높이 복귀")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    target_x,
                    target_y,
                    travel_z,
                    ori,
                    log,
                ),
            )
            if not ok:
                log.error("안전 높이 복귀 실패")
                self.state._publish_robot_status(
                    "failed",
                    message="안전 높이 복귀 실패",
                    reason="lift_failed",
                    command=self.state.current_command,
                )
                return

            handoff_pose = self.handoff_motion.move_to_handoff_pose(
                travel_z,
                ori,
                log,
            )
            if handoff_pose[0] is None:
                return

            log.info("8단계: 사용자 잡기 인식 대기")
            self.state._publish_robot_status(
                "waiting_handoff",
                message="사용자 잡기 인식을 기다립니다.",
                command=self.state.current_command,
            )
            if not self.wait_for_human_grasp(log):
                log.error("사용자 잡기 인식 실패. 원래 공구 위치로 반환합니다.")
                returned = self.return_tool_to_original_position(
                    target_x,
                    target_y,
                    travel_z,
                    grasp_z,
                    ori,
                    log,
                )
                status = "returned" if returned else "failed"
                self.state._publish_robot_status(
                    status,
                    message=(
                        "사용자 잡기 인식 실패로 공구를 원래 위치에 반환했습니다."
                        if returned
                        else "사용자 잡기 인식 실패 후 원위치 반환에도 실패했습니다."
                    ),
                    reason="handoff_timeout",
                    command=self.state.current_command,
                )
                return

            log.info("9단계: 사용자 잡기 확인 후 그리퍼 오픈(놓기)")
            self.gripper.open_gripper()
            self.cooperative_wait(0.8)

            log.info("10단계: 전달 후 Home 위치로 복귀")
            ok = self.handoff_motion.move_home_after_handoff(travel_z, ori, log)
            if not ok:
                return

            log.info("Pick 시퀀스 완료")
            self.state._publish_robot_status(
                "done",
                message="공구 전달 후 Home 복귀까지 완료되었습니다.",
                command=self.state.current_command,
            )

        finally:
            self.state.picking = False
            self.state.target_label = None
            self.state.human_grasped_tool = False
            self.state.current_command = None

    def return_tool_to_original_position(
        self,
        target_x,
        target_y,
        travel_z,
        grasp_z,
        ori,
        logger,
    ):
        logger.info("반환 1단계: 원래 공구 위치 상단으로 이동")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                target_x,
                target_y,
                travel_z,
                ori,
                logger,
            ),
        )
        if not ok:
            logger.error("원래 공구 위치 상단 이동 실패. 공구를 잡은 상태로 중단합니다.")
            return False

        logger.info("반환 2단계: 원래 공구 위치로 하강")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(target_x, target_y, grasp_z, ori, logger),
        )
        if not ok:
            logger.error("원래 공구 위치 하강 실패. 공구를 잡은 상태로 중단합니다.")
            return False

        logger.info("반환 3단계: 원래 위치에 공구 놓기")
        self.gripper.open_gripper()
        self.cooperative_wait(0.8)

        logger.info("반환 4단계: 공구를 놓은 뒤 안전 높이로 복귀")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                target_x,
                target_y,
                travel_z,
                ori,
                logger,
            ),
        )
        if not ok:
            logger.error("공구를 놓은 뒤 안전 높이 복귀 실패")
            return False

        logger.info("반환 5단계: Home 위치로 복귀")
        ok = self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                self.state.home_xyz[0],
                self.state.home_xyz[1],
                travel_z,
                ori,
                logger,
            ),
        )
        if not ok:
            logger.error("공구 반환 후 Home 복귀 실패")
            return False

        return True

    def wait_for_human_grasp(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            if self.state.human_grasped_tool:
                logger.info("사용자가 공구를 잡은 것으로 확인됨")
                return True

            if time.monotonic() - start_time >= HAND_GRASP_TIMEOUT_SEC:
                logger.warn(
                    f"{HAND_GRASP_TIMEOUT_SEC:.1f}초 동안 사용자 잡기 인식이 없어 대기 종료"
                )
                return False

            self.cooperative_wait(0.1)

        return False

    def try_robot_grasp(self, logger):
        def publish_attempt(attempt, retry_limit):
            self.state._publish_robot_status(
                "grasping",
                message=f"공구 grasp 시도 {attempt}/{retry_limit}",
                command=self.state.current_command,
            )

        return self.grasp_verifier.try_grasp(
            logger,
            publish_attempt=publish_attempt,
            failure_prefix="그리퍼 grasp",
        )

    @staticmethod
    def cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
