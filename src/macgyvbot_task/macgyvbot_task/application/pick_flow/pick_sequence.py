"""Pick sequence orchestration."""

import math
import time

import rclpy

from macgyvbot_config.drawer import (
    DRAWER_ENTRY_GRIPPER_WIDTH_M,
    get_tool_drawer_floor,
)
from macgyvbot_config.timing import (
    GRIPPER_GRASP_WAIT_SEC,
    GRIPPER_OPEN_WAIT_SEC,
    SEQUENCE_WAIT_POLL_SEC,
)
from macgyvbot_manipulation.grasp_verifier import cooperative_wait
from macgyvbot_manipulation.handover_targeting import move_to_observation_pose
from macgyvbot_task.application.drawer_flow.drawer_sequence import DrawerInteraction
from macgyvbot_task.application.pick_flow.pick_grasp_flow import PickGraspFlow
from macgyvbot_task.application.pick_flow.pick_handoff_flow import PickHandoffFlow
from macgyvbot_task.application.pick_flow.pick_target_planner import PickTargetPlanner
from macgyvbot_manipulation.robot_pose import (
    current_ee_orientation,
    get_ee_matrix,
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
        tool_hold_monitor=None,
    ):
        self.robot = robot
        self.motion = motion_controller
        self.gripper = gripper
        self.state = state
        self.detector = detector
        self.drawer_detector = drawer_detector
        self.depth_projector = depth_projector
        self.grasp_point_selector = grasp_point_selector
        self.tool_hold_monitor = tool_hold_monitor
        self.target_planner = PickTargetPlanner(robot)
        self.handoff = PickHandoffFlow(
            robot,
            motion_controller,
            gripper,
            state,
            self.cooperative_wait,
            tool_hold_monitor,
        )
        self.grasp = PickGraspFlow(
            gripper,
            state,
            self.cooperative_wait,
        )

    def run_drawer_pick(self, requested_tool):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None
        self.state.robot_grasp_succeeded = False

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

        motion = None
        pick_success = False
        try:
            try:
                floor = get_tool_drawer_floor(requested_tool)
            except ValueError as exc:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message=f"서랍 층수 설정 오류: {exc}",
                    reason="invalid_drawer_floor_config",
                    command=self.state.current_command,
                )
                return
            self.state._publish_robot_status(
                "moving_to_observation",
                tool_name=requested_tool,
                action="bring",
                message="서랍 접근을 위해 관찰 자세로 이동 중입니다.",
                command=self.state.current_command,
            )
            ok, _ = move_to_observation_pose(self.motion, self.robot, log)
            if not ok:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="서랍 관찰 자세로 이동하지 못했습니다.",
                    reason="drawer_observation_pose_failed",
                    command=self.state.current_command,
                )
                return

            self.state._publish_robot_status(
                "moving_to_drawer",
                tool_name=requested_tool,
                action="bring",
                message="서랍 손잡이 joint 위치로 이동 중입니다.",
                command=self.state.current_command,
            )
            self.gripper.open_gripper()
            self.cooperative_wait(GRIPPER_OPEN_WAIT_SEC)
            motion = drawer.build_motion_from_joints(log, floor=floor)
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
            if drawer.open_drawer_from_handle(motion, log) is None:
                self.state._publish_robot_status(
                    "failed",
                    tool_name=requested_tool,
                    action="bring",
                    message="서랍을 열지 못했습니다.",
                    reason="drawer_open_failed",
                    command=self.state.current_command,
                )
                motion = None
                return

            log.info(f"서랍 픽업 층 선택: tool={requested_tool}, floor={floor}")
            self.state._publish_robot_status(
                "searching",
                tool_name=requested_tool,
                action="bring",
                message=f"열린 서랍 {floor}층에서 {requested_tool} 탐색 중입니다.",
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

            pick_success = self._drawer_grasp_and_handoff(tool_target, motion.handle_fk_z, floor, log)
        finally:
            # robot_grasp_succeeded is set True only after try_robot_grasp succeeds.
            # If False, gripper never closed on the tool → safe to close drawer.
            gripper_safe = pick_success or not self.state.robot_grasp_succeeded
            drawer_ok = False

            if motion is not None:
                self.state._publish_robot_status(
                    "closing_drawer",
                    tool_name=requested_tool,
                    action="bring",
                    message="서랍을 닫는 중입니다.",
                    command=self.state.current_command,
                )
                if gripper_safe:
                    ok, _ = move_to_observation_pose(self.motion, self.robot, log)
                    if ok:
                        drawer_ok = drawer.close_drawer(motion, log)
                        if not drawer_ok:
                            log.error("서랍 닫기 실패")
                    else:
                        log.error("서랍 닫기 전 관찰 자세 이동 실패")
                else:
                    log.error("그리퍼가 공구를 잡고 있어 서랍을 닫지 않습니다.")

            if pick_success:
                log.info("서랍 pick 완료: Home 복귀")
                home_ok = self.handoff.move_home_after_handoff(log)
                if home_ok and drawer_ok:
                    self.state._publish_robot_status(
                        "done",
                        tool_name=requested_tool,
                        action="bring",
                        message="공구 전달 후 서랍 닫기 및 Home 복귀까지 완료되었습니다.",
                        command=self.state.current_command,
                    )
                else:
                    self.state._publish_robot_status(
                        "failed",
                        tool_name=requested_tool,
                        action="bring",
                        message="공구 전달은 완료됐으나 서랍 닫기 또는 Home 복귀에 실패했습니다.",
                        reason="post_pick_cleanup_failed",
                        command=self.state.current_command,
                    )

            self.state.picking = False
            self.state.target_label = None
            self.state.human_grasped_tool = False
            self.state.current_command = None

    def _drawer_grasp_and_handoff(self, tool_target, handle_fk_z, floor, log):
        """서랍 전용 pick: 관찰 자세에서 approach_z로 직접 이동 → grasp_z 하강 → 파지 → 수직 리프트 → handoff.

        XY+Z를 approach_z까지 한 번에 이동해 lift_z 기준 XY 보정보다
        base 거리를 줄여 planning 실패를 피한다.
        """
        plan = self.target_planner.plan_drawer(
            tool_target.x, tool_target.y, handle_fk_z, floor, log
        )
        current_pose = get_ee_matrix(self.robot)
        current_x = float(current_pose[0, 3])
        current_y = float(current_pose[1, 3])
        lift_z = float(current_pose[2, 3])
        ori = current_ee_orientation(self.robot)

        base_dist_approach = math.sqrt(
            plan.target_x ** 2 + plan.target_y ** 2 + plan.approach_z ** 2
        )
        log.info(
            f"[서랍pick] 시작 — "
            f"EE현재=({current_x:.3f}, {current_y:.3f}, {lift_z:.3f}) "
            f"tool_raw=({tool_target.x:.3f}, {tool_target.y:.3f}, {tool_target.z:.3f}) "
            f"depth_m={tool_target.depth_m:.3f} "
            f"plan.target=({plan.target_x:.3f}, {plan.target_y:.3f}) "
            f"grasp_z={plan.grasp_z:.3f} approach_z={plan.approach_z:.3f} "
            f"corrected_bz={plan.corrected_bz:.3f} "
            f"base_dist_approach={base_dist_approach:.3f}m"
        )

        # 1. Gripper pre-open → narrow to drawer entry width
        self.gripper.open_gripper()
        self.cooperative_wait(GRIPPER_OPEN_WAIT_SEC)
        self.gripper.move_gripper(int(DRAWER_ENTRY_GRIPPER_WIDTH_M * 10000))
        self.cooperative_wait(GRIPPER_OPEN_WAIT_SEC)

        # 2. XY+Z 동시 이동 → approach_z (lift_z 중간 경유 없음 — base 거리 단축)
        xy_offset = math.hypot(plan.target_x - current_x, plan.target_y - current_y)
        log.info(
            f"[서랍pick] 2단계 approach 이동 계획 — "
            f"from=({current_x:.3f}, {current_y:.3f}, {lift_z:.3f}) "
            f"to=({plan.target_x:.3f}, {plan.target_y:.3f}, {plan.approach_z:.3f}) "
            f"XY거리={xy_offset:.3f}m base_dist={base_dist_approach:.3f}m"
        )
        ok = self.motion.plan_and_execute(
            log,
            pose_goal=make_safe_pose(plan.target_x, plan.target_y, plan.approach_z, ori, log),
        )
        if not ok:
            log.error(
                f"[서랍pick] 2단계 approach 이동 실패 — "
                f"target=({plan.target_x:.3f}, {plan.target_y:.3f}, {plan.approach_z:.3f}) "
                f"base_dist={base_dist_approach:.3f}m"
            )
            self.state._publish_robot_status(
                "failed",
                message="서랍 pick approach 이동 실패",
                reason="drawer_pick_approach_failed",
                command=self.state.current_command,
            )
            return False
        log.info(f"[서랍pick] 2단계 approach 이동 완료 — approach_z={plan.approach_z:.3f}")

        # 3. 수직 하강 to grasp_z
        descent = plan.approach_z - plan.grasp_z
        log.info(
            f"[서랍pick] 3단계 하강 계획 — "
            f"from_z={plan.approach_z:.3f} to_z={plan.grasp_z:.3f} "
            f"하강거리={descent:.3f}m "
            f"{'⚠ grasp_z<0.24 → clamp됨' if plan.grasp_z < 0.24 else ''}"
        )
        ok = self.motion.plan_and_execute(
            log,
            pose_goal=make_safe_pose(plan.target_x, plan.target_y, plan.grasp_z, ori, log),
        )
        if not ok:
            log.error(
                f"[서랍pick] 3단계 하강 실패 — "
                f"approach_z={plan.approach_z:.3f} grasp_z={plan.grasp_z:.3f} descent={descent:.3f}m"
            )
            self.state._publish_robot_status(
                "failed",
                message="서랍 pick 하강 실패",
                reason="drawer_pick_descend_failed",
                command=self.state.current_command,
            )
            return False
        log.info(f"[서랍pick] 3단계 하강 완료 — grasp_z={plan.grasp_z:.3f}")

        # 4. Grasp
        if not self.grasp.try_robot_grasp(log):
            log.error("[서랍pick] 4단계 grasp 실패")
            self.state._publish_robot_status(
                "failed",
                message="서랍 pick grasp 실패",
                reason="drawer_pick_grasp_failed",
                command=self.state.current_command,
            )
            return False
        self.state.robot_grasp_succeeded = True
        log.info("[서랍pick] 4단계 grasp 성공")
        self.state._publish_robot_status(
            "grasp_success",
            message="서랍 공구 grasp 성공",
            command=self.state.current_command,
        )
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.start(
                self.state.target_label,
                "bring",
                self.state.current_command,
            )

        # 5. Mask lock (handoff 인식에 필요)
        if not self.handoff.wait_for_tool_mask_lock(log):
            log.error("[서랍pick] 5단계 mask lock 실패")
            self.state._publish_robot_status(
                "failed",
                message="서랍 pick mask lock 실패",
                reason="drawer_pick_mask_lock_failed",
                command=self.state.current_command,
            )
            return False
        log.info("[서랍pick] 5단계 mask lock 완료")

        # 6. 수직 리프트만 (XY 이동 없음)
        post_grasp_pose = get_ee_matrix(self.robot)
        actual_z_after_grasp = float(post_grasp_pose[2, 3])
        lift_dist = lift_z - actual_z_after_grasp
        log.info(
            f"[서랍pick] 6단계 수직리프트 계획 — "
            f"현재z={actual_z_after_grasp:.3f} → lift_z={lift_z:.3f} "
            f"리프트거리={lift_dist:.3f}m"
        )
        ok = self.motion.plan_and_execute(
            log,
            pose_goal=make_safe_pose(plan.target_x, plan.target_y, lift_z, ori, log),
        )
        if not ok:
            log.error(
                f"[서랍pick] 6단계 수직리프트 실패 — "
                f"현재z={actual_z_after_grasp:.3f} lift_z={lift_z:.3f} dist={lift_dist:.3f}m"
            )
            self.state._publish_robot_status(
                "failed",
                message="서랍 pick 수직 리프트 실패",
                reason="drawer_pick_lift_failed",
                command=self.state.current_command,
            )
            return False
        log.info(f"[서랍pick] 6단계 수직리프트 완료 — lift_z={lift_z:.3f}")

        # 7. Handoff 자세로 이동
        handoff_pose = self.handoff.move_to_handoff_pose(ori, log)
        if handoff_pose[0] is None:
            log.error("[서랍pick] 7단계 handoff 자세 이동 실패 — 사용자 손 위치 미확인")
            self.state._publish_robot_status(
                "failed",
                message="사용자 손 위치 확인 실패",
                reason="handoff_pose_unavailable",
                command=self.state.current_command,
            )
            return False
        log.info("[서랍pick] 7단계 handoff 자세 이동 완료")

        # 8. 사용자 잡기 대기
        self.state._publish_robot_status(
            "waiting_handoff",
            message="사용자 잡기 인식을 기다립니다.",
            command=self.state.current_command,
        )
        if not self.handoff.wait_for_human_grasp(log):
            log.error("[서랍pick] 8단계 사용자 잡기 인식 실패 (timeout)")
            self.state._publish_robot_status(
                "failed",
                message="사용자 잡기 인식 실패",
                reason="handoff_timeout",
                command=self.state.current_command,
            )
            return False
        log.info("[서랍pick] 8단계 사용자 잡기 인식 완료")

        # 9. 그리퍼 해제
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("handoff_release")
        self.gripper.open_gripper()
        self.cooperative_wait(GRIPPER_GRASP_WAIT_SEC)
        self.state.robot_grasp_succeeded = False
        log.info("[서랍pick] 9단계 그리퍼 해제 완료 — 전체 시퀀스 성공")
        return True

    def run(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        self.state.human_grasped_tool = False
        self.state.last_grasp_result = None
        self.state.tool_mask_locked = False
        self.state.last_tool_mask_lock_result = None

        try:
            if not self._pick_and_handoff(bx, by, bz, z_m, vlm_yaw_deg):
                return

            log = self.state.logger()

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
            if self.tool_hold_monitor is not None:
                self.tool_hold_monitor.stop("pick_sequence_finished")
            self.state.picking = False
            self.state.target_label = None
            self.state.human_grasped_tool = False
            self.state.current_command = None

    def _pick_and_handoff(self, bx, by, bz, z_m, vlm_yaw_deg=None):
        """공구를 파지하고 사용자에게 전달한다. 그리퍼 해제 성공 시 True 반환.

        Home 복귀 및 done 상태 publish는 호출자가 담당한다.
        상태 초기화(picking, current_command 등)도 호출자의 finally에서 수행한다.
        """
        log = self.state.logger()
        ori = self.state.home_ori

        plan = self.target_planner.plan(bx, by, bz, log)

        log.info(
            f"시퀀스 시작: Target({plan.target_x:.3f}, {plan.target_y:.3f}), "
            f"depth={z_m:.3f}, raw_bz={bz:.3f}, "
            f"corrected_bz={plan.corrected_bz:.3f}, "
            f"travel_z={plan.travel_z:.3f}, "
            f"approach_z={plan.approach_z:.3f}, "
            f"grasp_z={plan.grasp_z:.3f}"
        )

        self.gripper.open_gripper()
        self.cooperative_wait(GRIPPER_OPEN_WAIT_SEC)

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
            return False

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
            return False

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
                return False
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
            return False

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
                return False
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
            return False

        self.state._publish_robot_status(
            "grasp_success",
            message="공구 grasp에 성공했습니다.",
            command=self.state.current_command,
        )
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.start(
                self.state.target_label,
                "bring",
                self.state.current_command,
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
            return False

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
            return False

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
            return False

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
            return False

        log.info("10단계: 사용자 잡기 확인 후 그리퍼 오픈(놓기)")
        if self.tool_hold_monitor is not None:
            self.tool_hold_monitor.stop("handoff_release")
        self.gripper.open_gripper()
        self.cooperative_wait(GRIPPER_GRASP_WAIT_SEC)
        return True

    @staticmethod
    def cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))
