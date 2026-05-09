"""Pick, handoff, and return sequence orchestration."""

import time

import rclpy
from scipy.spatial.transform import Rotation

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
    GRASP_POINT_MODE_VLA,
    VLA_SWITCH_Z_OFFSET,
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

    def run(
        self,
        bx,
        by,
        bz,
        z_m,
        vlm_yaw_deg=None,
        label=None,
        bbox=None,
        color_image=None,
        task_instruction=None,
    ):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None

        log = self.state.logger()
        ori = self.state.home_ori
        final_target_x = bx
        final_target_y = by
        final_grasp_z = bz
        final_ori = ori
        switch_z = None
        moved_to_vla_switch_pose = False

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

        if self.state.grasp_point_mode == GRASP_POINT_MODE_VLA:
            switch_z = min(approach_z, grasp_z + VLA_SWITCH_Z_OFFSET)

        should_descend_to_grasp = abs(approach_z - grasp_z) > 0.005

        try:
            log.info(
                f"시퀀스 시작: Target({target_x:.3f}, {target_y:.3f}), "
                f"depth={z_m:.3f}, travel_z={travel_z:.3f}, "
                f"approach_z={approach_z:.3f}, grasp_z={grasp_z:.3f}"
            )

            self.gripper.open_gripper()
            time.sleep(0.5)

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

            log.info("5단계: 그리퍼 닫기")
            if self.state.grasp_point_mode == GRASP_POINT_MODE_VLA and label is not None:
                if switch_z is not None:
                    log.info(f"VLA switch pose로 상승: z={switch_z:.3f}")
                    ok = self.motion.plan_and_execute(
                        log,
                        pose_goal=make_safe_pose(
                            target_x,
                            target_y,
                            switch_z,
                            ori,
                            log,
                        ),
                    )
                    if not ok:
                        log.warn(
                            "VLA switch pose 이동 실패. 기존 grasp pose에서 계속 진행합니다."
                        )
                        switch_z = grasp_z
                    else:
                        moved_to_vla_switch_pose = abs(switch_z - grasp_z) > 0.005

                vla_result = self.refine_grasp_pose_with_vla(
                    target_x=target_x,
                    target_y=target_y,
                    switch_z=(switch_z if switch_z is not None else grasp_z),
                    label=label,
                    bbox=bbox,
                    object_xyz=(bx, by, bz),
                    color_image=color_image,
                    task_instruction=task_instruction,
                )
                if vla_result is not None:
                    final_target_x = vla_result["x"]
                    final_target_y = vla_result["y"]
                    final_grasp_z = vla_result["z"]
                    final_ori = vla_result["ori"]
                else:
                    final_target_x = target_x
                    final_target_y = target_y
                    final_grasp_z = grasp_z
                    final_ori = ori
                    if moved_to_vla_switch_pose:
                        log.info(
                            "VLA 보정 실패 또는 생략. 기존 grasp pose로 다시 하강합니다."
                        )
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
                            log.error("기존 grasp pose 재하강 실패. Pick 시퀀스 중단")
                            return
            else:
                final_target_x = target_x
                final_target_y = target_y
                final_grasp_z = grasp_z
                final_ori = ori

            self.gripper.close_gripper()
            time.sleep(1.0)

            log.info("6단계: 안전 높이 복귀")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    final_target_x,
                    final_target_y,
                    travel_z,
                    final_ori,
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

            log.info("7단계: Home XY 복귀")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    self.state.home_xyz[0],
                    self.state.home_xyz[1],
                    travel_z,
                    ori,
                    log,
                ),
            )
            if not ok:
                log.error("Home 복귀 실패")
                self.state._publish_robot_status(
                    "failed",
                    message="Home 복귀 실패",
                    reason="home_return_failed",
                    command=self.state.current_command,
                )
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
                    final_grasp_z,
                    final_ori,
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
            time.sleep(0.8)

            log.info("Pick 시퀀스 완료")
            self.state._publish_robot_status(
                "done",
                message="공구 전달 동작이 완료되었습니다.",
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
        time.sleep(0.8)

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

    def refine_grasp_pose_with_vla(
        self,
        target_x,
        target_y,
        switch_z,
        label,
        bbox,
        object_xyz,
        color_image,
        task_instruction=None,
    ):
        log = self.state.logger()

        if color_image is None:
            log.warn("VLA grasp 생략: color image가 없습니다.")
            return None

        try:
            from macgyvbot.grasp_point_vla import (
                DetectedObjectContext,
                Pose3D,
                RobotArmState,
            )
        except ImportError as exc:
            log.warn(f"VLA grasp 모듈 import 실패: {exc}")
            return None

        vla_state_model = self.state.ensure_vla_state_model_loaded()
        if vla_state_model is None:
            return None

        ee_matrix = get_ee_matrix(self.robot)
        ee_quat = Rotation.from_matrix(ee_matrix[:3, :3]).as_quat()
        current_state = RobotArmState(
            ee_pose=Pose3D(
                position_xyz=(
                    float(ee_matrix[0, 3]),
                    float(ee_matrix[1, 3]),
                    float(ee_matrix[2, 3]),
                ),
                quaternion_xyzw=tuple(float(v) for v in ee_quat),
            ),
        )

        bbox_xyxy = None
        if bbox is not None:
            bbox_xyxy = self.clamp_bbox_to_image(bbox, color_image)

        object_context = DetectedObjectContext(
            label=label,
            base_position_xyz=(
                float(object_xyz[0]),
                float(object_xyz[1]),
                float(object_xyz[2]),
            ),
            bbox_xyxy=bbox_xyxy,
            switch_offset_z_m=max(switch_z - float(object_xyz[2]), 0.0),
            task_instruction=task_instruction,
        )

        try:
            proposal = vla_state_model.propose_grasp_state(
                color_image,
                current_state=current_state,
                object_context=object_context,
            )
        except Exception as exc:
            log.warn(f"VLA grasp 상태 추론 실패: {exc}")
            return None

        final_pose = proposal.recommended_state.ee_pose
        final_xyz = final_pose.position_xyz
        final_ori = self.quat_to_ori_dict(final_pose.quaternion_xyzw)

        log.info(
            f"VLA final grasp pose: x={final_xyz[0]:.3f}, "
            f"y={final_xyz[1]:.3f}, z={final_xyz[2]:.3f}, "
            f"gripper={proposal.action.gripper}"
        )
        log.info(
            "VLA action delta: "
            f"xyz={proposal.action.delta_xyz}, "
            f"rpy={proposal.action.delta_rpy}, "
            f"full={proposal.action.full_action}"
        )
        if proposal.notes:
            log.info(f"VLA notes: {'; '.join(proposal.notes)}")

        ok = self.motion.plan_and_execute(
            log,
            pose_goal=make_safe_pose(
                final_xyz[0],
                final_xyz[1],
                final_xyz[2],
                final_ori,
                log,
            ),
        )
        if not ok:
            log.warn("VLA final grasp pose 이동 실패. 기존 grasp pose를 사용합니다.")
            return None

        return {
            "x": float(final_xyz[0]),
            "y": float(final_xyz[1]),
            "z": float(final_xyz[2]),
            "ori": final_ori,
            "proposal": proposal,
        }

    @staticmethod
    def clamp_bbox_to_image(bbox, image):
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(bbox[0])))
        y1 = max(0, min(height - 1, int(bbox[1])))
        x2 = max(0, min(width, int(bbox[2])))
        y2 = max(0, min(height, int(bbox[3])))
        return x1, y1, x2, y2

    @staticmethod
    def quat_to_ori_dict(quat_xyzw):
        return {
            "x": float(quat_xyzw[0]),
            "y": float(quat_xyzw[1]),
            "z": float(quat_xyzw[2]),
            "w": float(quat_xyzw[3]),
        }

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

            time.sleep(0.1)

        return False
