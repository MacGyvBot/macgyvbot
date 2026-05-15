"""Pick sequence orchestration."""

import time

import rclpy

from macgyvbot.util.macgyvbot_main.model_control.robot_safezone import (
    SAFE_Z_MIN,
    clamp_to_safe_workspace,
)

from macgyvbot.config.config import (
    APPROACH_Z_OFFSET,
    BASE_FRAME,
    DRAWER_HANDLE_LABEL,
    DRAWER_LABEL,
    GRASP_Z_OFFSET,
    HAND_GRASP_MASK_LOCK_TIMEOUT_SEC,
    HANDOVER_HAND_X_OFFSET_M,
    HANDOVER_HAND_Z_OFFSET_M,
    OBJECT_Z_HEIGHT_BIAS_M,
    OBSERVATION_TIMEOUT_SEC,
    HAND_GRASP_TIMEOUT_SEC,
    SAFE_Z,
    SEQUENCE_WAIT_POLL_SEC,
    USE_DRAWER_HANDLE_OFFSET_FALLBACK,
)
from macgyvbot.util.macgyvbot_main.model_control.grasp_verifier import (
    GraspVerifier,
)
from macgyvbot.util.macgyvbot_main.model_control.handover_targeting import (
    move_to_observation_pose,
    move_to_candidate_with_offset,
    start_async_observation_search,
)
from macgyvbot.util.macgyvbot_main.model_control.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
    make_safe_pose,
)
from macgyvbot.util.macgyvbot_main.task_pipeline.drawer_sequence import (
    DrawerInteraction,
)


class PickSequenceRunner:
    def __init__(self, robot, motion_controller, gripper, state):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.grasp_verifier = GraspVerifier(gripper, self.cooperative_wait)
        self.drawer = DrawerInteraction(
            robot,
            motion_controller,
            gripper,
            state,
            self.cooperative_wait,
        )

    def run_drawer_pick(self, requested_tool):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None

        log = self.state.logger()

        try:
            self.state._publish_robot_status(
                "moving_to_observation",
                tool_name=requested_tool,
                action="bring",
                message="서랍 탐지를 위해 관찰 자세로 이동 중입니다.",
                command=self.state.current_command,
            )
            ok, start_pose = move_to_observation_pose(self.motion, self.robot, log)
            log.info(
                "서랍 탐지 관찰 자세 이동 "
                f"pose=({start_pose.x:.3f},{start_pose.y:.3f},{start_pose.z:.3f})"
            )
            if not ok:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="서랍 탐지 관찰 자세로 이동하지 못했습니다.",
                    reason="drawer_observation_pose_failed",
                    command=self.state.current_command,
                )
                return

            self.state._publish_robot_status(
                "searching_drawer",
                tool_name=requested_tool,
                action="bring",
                message=f"{requested_tool}를 꺼낼 공구함 찾기 중입니다.",
                command=self.state.current_command,
            )
            drawer_target = self.drawer.wait_for_target(DRAWER_LABEL, log)
            if drawer_target is None:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="관찰 자세에서 공구함을 찾지 못했습니다.",
                    reason="drawer_not_found",
                    command=self.state.current_command,
                )
                return

            self.state._publish_robot_status(
                "moving_to_drawer",
                tool_name=requested_tool,
                action="bring",
                message="공구함으로 이동 중입니다.",
                command=self.state.current_command,
            )
            if not self.drawer.move_to_drawer_view(drawer_target, log):
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="공구함으로 이동하지 못했습니다.",
                    reason="drawer_move_failed",
                    command=self.state.current_command,
                )
                return

            self.state._publish_robot_status(
                "searching_drawer_handle",
                tool_name=requested_tool,
                action="bring",
                message="서랍 손잡이를 찾는 중입니다.",
                command=self.state.current_command,
            )
            if USE_DRAWER_HANDLE_OFFSET_FALLBACK:
                handle_target = self.drawer.handle_target_from_drawer_offset(
                    drawer_target,
                    log,
                )
            else:
                handle_target = self.drawer.wait_for_target(DRAWER_HANDLE_LABEL, log)
            if handle_target is None:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="서랍 손잡이를 찾지 못했습니다.",
                    reason="drawer_handle_not_found",
                    command=self.state.current_command,
                )
                return

            self.state._publish_robot_status(
                "opening_drawer",
                tool_name=requested_tool,
                action="bring",
                message="서랍 손잡이를 당겨 여는 중입니다.",
                command=self.state.current_command,
            )
            if self.drawer.open_drawer(handle_target, log) is None:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="서랍을 열지 못했습니다.",
                    reason="drawer_open_failed",
                    command=self.state.current_command,
                )
                return

            self.state._publish_robot_status(
                "searching",
                tool_name=requested_tool,
                action="bring",
                message=f"열린 서랍 안에서 {requested_tool} 탐색 중입니다.",
                command=self.state.current_command,
            )
            tool_target = self.drawer.wait_for_target(
                requested_tool,
                log,
                use_grasp_selector=True,
            )
            if tool_target is None:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message=f"열린 서랍 안에서 {requested_tool}를 찾지 못했습니다.",
                    reason="tool_not_found_in_drawer",
                    command=self.state.current_command,
                )
                return

            self.run(
                tool_target.x,
                tool_target.y,
                tool_target.z,
                tool_target.depth_m,
                tool_target.yaw_deg,
            )
        finally:
            if self.state.picking and self.state.target_label == requested_tool:
                self.state.picking = False
                self.state.target_label = None
                self.state.human_grasped_tool = False
                self.state.current_command = None

    def run(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None

        log = self.state.logger()
        ori = self.state.home_ori

        corrected_bz = bz - OBJECT_Z_HEIGHT_BIAS_M
        grasp_z = SAFE_Z_MIN + corrected_bz - GRASP_Z_OFFSET
        approach_z = grasp_z - GRASP_Z_OFFSET

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
                f"depth={z_m:.3f}, raw_bz={bz:.3f}, "
                f"corrected_bz={corrected_bz:.3f}, travel_z={travel_z:.3f}, "
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

            log.info("6단계: 공구 mask lock 완료 대기")
            if not self.wait_for_tool_mask_lock(log):
                log.error("공구 mask lock 실패. Lift 전에 pick 시퀀스를 중단합니다.")
                self.state._publish_robot_status(
                    "failed",
                    message="공구 mask lock에 실패했습니다.",
                    reason="tool_mask_lock_failed",
                    command=self.state.current_command,
                )
                return

            log.info("7단계: 안전 높이 복귀")
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

            handoff_pose = self.move_to_handoff_pose(
                travel_z,
                ori,
                log,
            )
            if handoff_pose[0] is None:
                log.error("사용자 손 위치 확인 실패. 원래 공구 위치로 반환합니다.")
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
                        "사용자 손 위치 확인 실패로 공구를 원래 위치에 반환했습니다."
                        if returned
                        else "사용자 손 위치 확인 실패 후 원위치 반환에도 실패했습니다."
                    ),
                    reason="handoff_pose_unavailable",
                    command=self.state.current_command,
                )
                return

            log.info("9단계: 사용자 잡기 인식 대기")
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

            log.info("10단계: 사용자 잡기 확인 후 그리퍼 오픈(놓기)")
            self.gripper.open_gripper()
            self.cooperative_wait(0.8)

            log.info("10단계: 전달 후 Home 위치로 복귀")
            ok = self.move_home_after_handoff(travel_z, ori, log)
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

        logger.info("반환 5단계: Home joint pose로 복귀")
        ok = self.motion.move_to_home_joints(logger)
        if not ok:
            logger.error("공구 반환 후 Home 복귀 실패")
            return False

        return True

    def move_to_handoff_pose(self, travel_z, ori, logger):
        ok, start_pose = move_to_observation_pose(self.motion, self.robot, logger)
        logger.info(
            "7단계: 사용자 전달 관찰 자세 이동 "
            f"pose=({start_pose.x:.3f},{start_pose.y:.3f},{start_pose.z:.3f})"
        )
        if not ok:
            logger.error("사용자 전달 위치 이동 실패. Pick 시퀀스 중단")
            self.state._publish_robot_status(
                "failed",
                message="사용자 전달 위치 이동에 실패했습니다.",
                reason="handoff_pose_move_failed",
                command=self.state.current_command,
            )
            return None, None, None

        future = start_async_observation_search(
            self.state,
            logger,
            timeout_sec=OBSERVATION_TIMEOUT_SEC,
        )
        candidate = future.result()

        if not candidate.found:
            logger.error("사용자 손 위치를 찾지 못했습니다.")
            self.state._publish_robot_status(
                "failed",
                message="사용자 손 위치를 찾지 못했습니다.",
                reason="handoff_search_failed",
                command=self.state.current_command,
            )
            return None, None, None

        if candidate.frame_id not in ("world", BASE_FRAME):
            logger.error(
                "사용자 손 위치 frame을 planning에 사용할 수 없습니다: "
                f"frame={candidate.frame_id}, source={candidate.source}"
            )
            self.state._publish_robot_status(
                "failed",
                message="사용자 손 위치 frame이 planning frame이 아닙니다.",
                reason="handoff_unsupported_frame",
                command=self.state.current_command,
            )
            return None, None, None

        ok, final_pose, reason = move_to_candidate_with_offset(
            self.motion,
            candidate,
            ori,
            logger,
            x_offset_m=HANDOVER_HAND_X_OFFSET_M,
            z_offset_m=HANDOVER_HAND_Z_OFFSET_M,
        )
        logger.info(
            "사용자 손 위치로 전달 이동: "
            f"source={candidate.source}, frame={candidate.frame_id}, "
            f"raw=({candidate.x:.3f},{candidate.y:.3f},{candidate.z:.3f}), "
            f"offset=({HANDOVER_HAND_X_OFFSET_M:.3f},0.000,{HANDOVER_HAND_Z_OFFSET_M:.3f}), "
            f"safe=({final_pose.x:.3f},{final_pose.y:.3f},{final_pose.z:.3f})"
        )
        if not ok:
            logger.error("사용자 손 위치로 전달 이동 실패")
            self.state._publish_robot_status(
                "failed",
                message="사용자 손 위치로 이동하지 못했습니다.",
                reason="handoff_hand_pose_move_failed" if reason == "target_move_failed" else reason,
                command=self.state.current_command,
            )
            return None, None, None

        return final_pose.x, final_pose.y, final_pose.z

    def _recover_to_home(self, travel_z, ori, logger, reason):
        logger.warn("handoff 실패 후 Home 복귀를 시도합니다.")
        ok = self.move_home_after_handoff(
            travel_z,
            ori,
            logger,
            publish_on_failure=False,
        )
        status = "returned" if ok else "failed"
        self.state._publish_robot_status(
            status,
            message=(
                "handoff 실패 후 Home으로 복귀했습니다."
                if ok
                else "handoff 실패 후 Home 복귀에도 실패했습니다."
            ),
            reason=reason if ok else f"{reason}_recovery_failed",
            command=self.state.current_command,
        )

    def move_home_after_handoff(self, travel_z, ori, logger, publish_on_failure=True):
        ok = self.motion.move_to_home_joints(logger)
        if not ok:
            logger.error("전달 후 Home 복귀 실패")
            if publish_on_failure:
                self.state._publish_robot_status(
                    "failed",
                    message="공구 전달 후 Home 복귀에 실패했습니다.",
                    reason="home_after_handoff_failed",
                    command=self.state.current_command,
                )
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

    def wait_for_tool_mask_lock(self, logger):
        start_time = time.monotonic()

        while rclpy.ok():
            result = self.state.last_tool_mask_lock_result
            if self.state.tool_mask_locked and result is not None:
                logger.info(
                    "공구 mask lock 확인: "
                    f"source={result.get('mask_source')}, roi={result.get('tool_roi')}"
                )
                return True

            if result is not None and result.get("locked") is False:
                logger.warn(
                    "공구 mask lock 실패 응답: "
                    f"reason={result.get('reason', 'unknown')}"
                )
                return False

            if time.monotonic() - start_time >= HAND_GRASP_MASK_LOCK_TIMEOUT_SEC:
                logger.warn(
                    f"{HAND_GRASP_MASK_LOCK_TIMEOUT_SEC:.1f}초 동안 "
                    "공구 mask lock 응답이 없어 대기 종료"
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
