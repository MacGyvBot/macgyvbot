"""Pick sequence orchestration."""

from macgyvbot_config.drawer import DRAWER_LABEL
from macgyvbot_config.timing import GRIPPER_OPEN_WAIT_SEC
from macgyvbot_domain import DetectedTarget
from macgyvbot_manipulation.grasp_verifier import cooperative_wait
from macgyvbot_manipulation.handover_targeting import move_to_observation_pose
from macgyvbot_task.application.drawer_flow.drawer_sequence import DrawerInteraction
from macgyvbot_task.application.pick_flow.pick_grasp_flow import PickGraspFlow
from macgyvbot_task.application.pick_flow.pick_handoff_flow import PickHandoffFlow
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    make_safe_pose,
)


class PickSequenceRunner:
    def __init__(
        self,
        robot,
        motion_controller,
        gripper,
        state,
        detector=None,
        drawer_detector=None,
        depth_projector=None,
        grasp_point_selector=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.detector = detector
        self.drawer_detector = drawer_detector
        self.depth_projector = depth_projector
        self.grasp_point_selector = grasp_point_selector
        self.target_planner = PickTargetPlanner(robot)
        self.handoff = PickHandoffFlow(
            robot,
            motion_controller,
            gripper,
            state,
            cooperative_wait,
        )
        self.grasp = PickGraspFlow(
            gripper,
            state,
            cooperative_wait,
        )

    def run_drawer_pick(self, requested_tool):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None

        log = self.state.get_logger()
        drawer = DrawerInteraction(
            robot=self.robot,
            motion_controller=self.motion,
            gripper=self.gripper,
            state=self.state,
            detector=self.detector,
            drawer_detector=self.drawer_detector,
            depth_projector=self.depth_projector,
            grasp_point_selector=self.grasp_point_selector,
            wait_fn=cooperative_wait,
        )

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

            # --- YOLO 서랍 탐지 (하드코딩 이동 시 불필요, 추후 활성화 가능) ---
            # self.state._publish_robot_status(
            #     "searching_drawer",
            #     tool_name=requested_tool,
            #     action="bring",
            #     message=f"{requested_tool}를 꺼낼 공구함 찾기 중입니다.",
            #     command=self.state.current_command,
            # )
            # drawer_target = drawer.wait_for_target(DRAWER_LABEL, log)
            # if drawer_target is None:
            #     self.state._publish_robot_status(
            #         "failed",
            #         tool_name=requested_tool,
            #         action="bring",
            #         message="관찰 자세에서 공구함을 찾지 못했습니다.",
            #         reason="drawer_not_found",
            #         command=self.state.current_command,
            #     )
            #     return
            # drawer_target = DetectedTarget(
            #     label=DRAWER_LABEL,
            #     x=DRAWER_FIXED_X,
            #     y=DRAWER_FIXED_Y,
            #     z=DRAWER_FIXED_Z,
            #     depth_m=drawer_target.depth_m,
            # )
            # -----------------------------------------------------------

            self.state._publish_robot_status(
                "moving_to_drawer",
                tool_name=requested_tool,
                action="bring",
                message="서랍 손잡이 joint 위치로 이동 중입니다.",
                command=self.state.current_command,
            )
            motion = drawer.build_motion_from_joints(log)
            if motion is None:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="서랍 손잡이 joint 이동에 실패했습니다.",
                    reason="drawer_joint_move_failed",
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
            if drawer.open_drawer(motion, log) is None:
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
            tool_target = drawer.wait_for_target(
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

    def run_pick_from_open_drawer(self, requested_tool):
        """서랍이 이미 열려 있는 상태에서 공구 탐지 → pick + handoff → 서랍 닫기."""
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None

        log = self.state.get_logger()
        drawer = DrawerInteraction(
            robot=self.robot,
            motion_controller=self.motion,
            gripper=self.gripper,
            state=self.state,
            detector=self.detector,
            drawer_detector=self.drawer_detector,
            depth_projector=self.depth_projector,
            grasp_point_selector=self.grasp_point_selector,
            wait_fn=cooperative_wait,
        )
        close_motion = self.state.drawer_handle_motion
        if close_motion is None:
            log.error("서랍 손잡이 위치 정보 없음. startup에서 서랍이 열리지 않았습니다.")
            return

        try:
            self.state._publish_robot_status(
                "searching",
                tool_name=requested_tool,
                action="bring",
                message=f"열린 서랍 안에서 {requested_tool} 탐색 중입니다.",
                command=self.state.current_command,
            )
            tool_target = drawer.wait_for_target(
                requested_tool, log, use_grasp_selector=True
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

            self.state._publish_robot_status(
                "closing_drawer",
                tool_name=requested_tool,
                action="bring",
                message="공구 전달 완료. 서랍을 닫는 중입니다.",
                command=self.state.current_command,
            )
            ok, _ = move_to_observation_pose(self.motion, self.robot, log)
            if not ok:
                log.error("서랍 닫기 전 관찰 자세 이동 실패")
                return
            if not drawer.close_drawer(close_motion, log):
                log.error("서랍 닫기 실패")
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

        log = self.state.get_logger()
        ori = self.state.home_ori

        plan = self.target_planner.plan(bx, by, bz, log)

        try:
            log.info(
                f"시퀀스 시작: Target({plan.target_x:.3f}, {plan.target_y:.3f}), "
                f"depth={z_m:.3f}, raw_bz={bz:.3f}, "
                f"corrected_bz={plan.corrected_bz:.3f}, "
                f"travel_z={plan.travel_z:.3f}, "
                f"approach_z={plan.approach_z:.3f}, "
                f"grasp_z={plan.grasp_z:.3f}"
            )

            self.gripper.open_gripper()
            cooperative_wait(GRIPPER_OPEN_WAIT_SEC)

            log.info("1단계: 안전 이동 높이 확보")
            ok = self.motion.plan_and_execute(
                log,
                pose_goal=make_safe_pose(
                    plan.current_x,
                    plan.current_y,
                    plan.travel_z,
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
                    plan.target_x,
                    plan.target_y,
                    plan.travel_z,
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
                    plan.target_x,
                    plan.target_y,
                    plan.approach_z,
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

            if plan.should_descend_to_grasp:
                log.info("4단계: 파지 높이 하강")
                ok = self.motion.plan_and_execute(
                    log,
                    pose_goal=make_safe_pose(
                        plan.target_x,
                        plan.target_y,
                        plan.grasp_z,
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
            if not self.grasp.try_robot_grasp(log):
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
            if not self.handoff.wait_for_tool_mask_lock(log):
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
                    plan.target_x,
                    plan.target_y,
                    plan.travel_z,
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

            handoff_pose = self.handoff.move_to_handoff_pose(ori, log)
            if handoff_pose[0] is None:
                log.error("사용자 손 위치 확인 실패. 원래 공구 위치로 반환합니다.")
                returned = self.handoff.return_tool_to_original_position(
                    plan.target_x,
                    plan.target_y,
                    plan.travel_z,
                    plan.grasp_z,
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
            if not self.handoff.wait_for_human_grasp(log):
                log.error("사용자 잡기 인식 실패. 원래 공구 위치로 반환합니다.")
                returned = self.handoff.return_tool_to_original_position(
                    plan.target_x,
                    plan.target_y,
                    plan.travel_z,
                    plan.grasp_z,
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
            cooperative_wait(0.8)

            log.info("10단계: 전달 후 Home 위치로 복귀")
            ok = self.handoff.move_home_after_handoff(log)
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
