#!/usr/bin/env python3
"""ROS node that owns task queue execution and task-control handling."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import threading
import time
import traceback

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from macgyvbot_interfaces.msg import (
    HumanGraspResult,
    RobotTaskControl,
    RobotTaskStatus,
    ToolDropEvent,
    ToolMaskLock,
)

from macgyvbot_config.drawer import DRAWER_OBSERVATION_J6_DEG
from macgyvbot_config.models import YOLO_MODEL_NAME
from macgyvbot_config.robot import GROUP_NAME, HOME_JOINTS, WRIST_JOINT_NAME
from macgyvbot_config.timing import SEQUENCE_WAIT_POLL_SEC
from macgyvbot_config.topics import (
    CAMERA_COLOR_TOPIC,
    CAMERA_DEPTH_TOPIC,
    CAMERA_INFO_TOPIC,
    FORCE_TORQUE_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_MASK_LOCK_TOPIC,
    HAND_GRASP_TOPIC,
    ROBOT_STATUS_TOPIC,
    ROBOT_TASK_CONTROL_TOPIC,
    TASK_REQUEST_TOPIC,
    TOOL_DROP_TOPIC,
)
from macgyvbot_config.vlm import DEFAULT_GRASP_POINT_MODE
from macgyvbot_manipulation.drawer_motion import DrawerMotionFlow
from macgyvbot_manipulation.moveit_controller import MoveItController
from macgyvbot_manipulation.onrobot_gripper import RG
from macgyvbot_manipulation.robot_pose import (
    get_ee_matrix,
    orientation_from_joint_positions,
)
from macgyvbot_perception.depth_projection import DepthProjector
from macgyvbot_perception.grasp_point.grasp_point_selector import (
    GraspPointSelector,
    normalize_grasp_point_mode,
)
from macgyvbot_perception.pick_target_resolver import PickTargetResolver
from macgyvbot_perception.yolo_detector import YoloDetector
from macgyvbot_task.application import (
    RobotStatusPublisher,
    TaskRuntimeState,
    ToolDropStatusReporter,
)
from macgyvbot_task.application.adapters.hand_grasp_result_adapter import (
    HandGraspResultAdapter,
)
from macgyvbot_task.application.display.debug_display import DebugDisplay
from macgyvbot_task.application.pick_flow.pick_frame_processor import (
    PickFrameProcessor,
)
from macgyvbot_task.application.pick_flow.pick_sequence import PickSequenceRunner
from macgyvbot_task.application.return_flow.return_sequence import ReturnSequenceRunner
from macgyvbot_task.application.robot.robot_home_initializer import (
    RobotHomeInitializer,
)
from macgyvbot_task.application.task_control.task_step import TaskStep


class VLMStatusLogger:
    """Forward VLM logs to ROS logger and robot status topic."""

    def __init__(self, logger, publish_status_payload):
        self._logger = logger
        self._publish_status_payload = publish_status_payload

    def info(self, message):
        self._logger.info(message)
        self._publish_if_vlm_status("info", message)

    def warn(self, message):
        self._logger.warn(message)
        self._publish_if_vlm_status("warn", message)

    def error(self, message):
        self._logger.error(message)
        self._publish_if_vlm_status("error", message)

    def _publish_if_vlm_status(self, level, message):
        text = str(message or "")
        if "VLM" not in text:
            return

        if "로드 시작" in text:
            status = "vlm_loading"
        elif "로드 완료" in text:
            status = "vlm_ready"
        elif level == "error" or "실패" in text:
            status = "vlm_error"
        elif level == "warn" or "CPU 실행" in text:
            status = "vlm_warning"
        else:
            return

        self._publish_status_payload(
            {
                "status": status,
                "tool_name": "unknown",
                "action": "system",
                "message": text,
            }
        )


class TaskCoordinatorNode(Node):
    """Own pick/return workflow queues and task-control requests."""

    def __init__(self):
        super().__init__("task_coordinator_node")

        self.declare_parameter("display_debug_windows", False)
        self.bridge = CvBridge()
        self.display_debug_windows = self._read_bool_parameter(
            "display_debug_windows",
            False,
        )
        self.display = DebugDisplay(enabled=self.display_debug_windows)
        self.exit_req = threading.Event()
        self.pause_req = threading.Event()
        self.resume_req = threading.Event()

        self._queue = deque()
        self._queue_lock = threading.RLock()
        self._step_thread = None
        self._current_step = None
        self._current_task_name = None
        self._suspended_step = None
        self._suspended_task_name = None
        self._exit_home_thread = None

        self.grasp_point_mode = self._read_grasp_point_mode()
        self.yolo_model = self._read_yolo_model()
        self.force_torque_topic = self._read_force_torque_topic()
        self.robot_status_pub = self.create_publisher(
            RobotTaskStatus,
            ROBOT_STATUS_TOPIC,
            10,
        )
        self.tool_drop_pub = self.create_publisher(
            ToolDropEvent,
            TOOL_DROP_TOPIC,
            10,
        )

        self.state = TaskRuntimeState(
            logger_provider=self.get_logger,
            publish_robot_status=self._publish_robot_status,
            publish_status_payload=self._publish_status_payload,
        )
        self.detector = YoloDetector(self.yolo_model)
        self.grasp_point_logger = VLMStatusLogger(
            self.get_logger(),
            self._publish_status_payload,
        )
        self.grasp_point_selector = GraspPointSelector(
            self.grasp_point_mode,
            self.grasp_point_logger,
            **self._read_grasp_point_api_config(),
            **self._read_vlm_sam_config(),
        )
        self.grasp_point_selector.preload_vlm_if_needed()

        calib_file = self._resolve_calibration_file("T_gripper2camera.npy")
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0

        self.gripper = RG("rg2", "192.168.1.1", 502)
        self.robot = MoveItPy(node_name="task_coordinator_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.depth_projector = DepthProjector(self._base_to_camera_matrix)
        self.hand_grasp_adapter = HandGraspResultAdapter(
            self.state,
            self.depth_projector,
            self.get_logger(),
        )
        self.pick_target_resolver = PickTargetResolver(
            self.detector,
            self.grasp_point_selector,
            self.depth_projector,
            self.get_logger(),
        )

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.2

        self.motion = MoveItController(
            self.robot,
            self.arm,
            self.pilz_params,
            should_interrupt=self._motion_interrupted,
            node=self,
        )
        self.drawer_flow = DrawerMotionFlow(
            self.robot,
            self.motion,
            self.gripper,
            self._cooperative_wait,
            observation_orientation_provider=self._drawer_observation_orientation,
        )
        self.home_initializer = RobotHomeInitializer(
            self.robot,
            self.motion,
            self.state,
        )
        self.status_publisher = RobotStatusPublisher(
            self._publish_status_payload,
            target_label_provider=lambda: self.state.target_label,
        )
        self.tool_hold_monitor = ToolDropStatusReporter(
            self.gripper,
            self._publish_tool_drop_payload,
            self._publish_status_payload,
        )

        control_events = {
            "exit": self.exit_req,
            "pause": self.pause_req,
            "resume": self.resume_req,
        }
        self.pick_runner = PickSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self.state,
            self.tool_hold_monitor,
            refine_pick_target=self._refine_pick_target_after_centering,
            control_events=control_events,
            drawer_flow=self.drawer_flow,
        )
        self.return_runner = ReturnSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self.state,
            self.tool_hold_monitor,
            control_events=control_events,
            drawer_flow=self.drawer_flow,
            detect_home_tool_label=self._detect_home_tool_label,
        )
        self.frame_processor = PickFrameProcessor(
            self.state,
            self.detector,
            self.pick_target_resolver,
            self.start_pick_sequence,
            self._publish_robot_status,
            self.get_logger(),
            drawer_ready_for_target=self._drawer_ready_for_target,
            prepare_drawer_for_target=self._prepare_drawer_for_target,
        )

        self._create_subscriptions()
        self._log_startup()

    def _base_to_camera_matrix(self):
        return get_ee_matrix(self.robot) @ self.gripper2cam

    def _drawer_observation_orientation(self):
        joint_positions = dict(HOME_JOINTS)
        joint_positions[WRIST_JOINT_NAME] = math.radians(
            DRAWER_OBSERVATION_J6_DEG
        )
        return orientation_from_joint_positions(self.robot, joint_positions)

    @staticmethod
    def _resolve_calibration_file(filename):
        try:
            package_share = Path(
                get_package_share_directory("macgyvbot_resources")
            )
            candidate = package_share / "calibration" / filename
            if candidate.exists():
                return candidate
        except Exception:
            pass

        workspace_src = Path(__file__).resolve().parents[2]
        return workspace_src / "macgyvbot_resources" / "calibration" / filename

    def _read_grasp_point_mode(self):
        self.declare_parameter("grasp_point_mode", DEFAULT_GRASP_POINT_MODE)
        grasp_point_mode = (
            self.get_parameter("grasp_point_mode")
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        return normalize_grasp_point_mode(grasp_point_mode, self.get_logger())

    def _read_yolo_model(self):
        self.declare_parameter("yolo_model", YOLO_MODEL_NAME)
        return (
            self.get_parameter("yolo_model")
            .get_parameter_value()
            .string_value
            .strip()
        ) or YOLO_MODEL_NAME

    def _read_grasp_point_api_config(self):
        self.declare_parameter("grasp_point_api_model", "gemini-2.5-flash")
        self.declare_parameter("grasp_point_api_env_file", "")
        self.declare_parameter("grasp_point_api_base_url", "")
        self.declare_parameter("grasp_point_api_timeout_sec", 30.0)
        return {
            "api_model": str(
                self.get_parameter("grasp_point_api_model").value
            ).strip(),
            "api_env_file": str(
                self.get_parameter("grasp_point_api_env_file").value
            ).strip(),
            "api_base_url": str(
                self.get_parameter("grasp_point_api_base_url").value
            ).strip(),
            "api_timeout_sec": float(
                self.get_parameter("grasp_point_api_timeout_sec").value
            ),
        }

    def _read_vlm_sam_config(self):
        self.declare_parameter("sam_enabled", True)
        self.declare_parameter("sam_checkpoint", str(Path("weights") / "mobile_sam.pt"))
        self.declare_parameter("sam_backend", "mobile_sam")
        self.declare_parameter("sam_model_type", "vit_t")
        self.declare_parameter("sam_device", "cuda")
        return {
            "sam_enabled": self._read_bool_parameter("sam_enabled", True),
            "sam_checkpoint": str(self.get_parameter("sam_checkpoint").value).strip(),
            "sam_backend": str(self.get_parameter("sam_backend").value).strip(),
            "sam_model_type": str(self.get_parameter("sam_model_type").value).strip(),
            "sam_device": str(self.get_parameter("sam_device").value).strip(),
        }

    def _read_force_torque_topic(self):
        self.declare_parameter("force_torque_topic", FORCE_TORQUE_TOPIC)
        return (
            self.get_parameter("force_torque_topic")
            .get_parameter_value()
            .string_value
            .strip()
        ) or FORCE_TORQUE_TOPIC

    def _read_bool_parameter(self, name, default_value=False):
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _create_subscriptions(self):
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._cam_info_cb, 10)
        self.create_subscription(Image, CAMERA_COLOR_TOPIC, self._color_cb, 10)
        self.create_subscription(Image, CAMERA_DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(String, TASK_REQUEST_TOPIC, self._task_request_cb, 10)
        self.create_subscription(
            RobotTaskControl,
            ROBOT_TASK_CONTROL_TOPIC,
            self._task_control_cb,
            10,
        )
        self.create_subscription(
            HumanGraspResult,
            HAND_GRASP_TOPIC,
            self._hand_grasp_cb,
            10,
        )
        self.create_subscription(
            Image,
            HAND_GRASP_IMAGE_TOPIC,
            self._hand_grasp_image_cb,
            10,
        )
        self.create_subscription(
            ToolMaskLock,
            HAND_GRASP_MASK_LOCK_TOPIC,
            self._tool_mask_lock_cb,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            self.force_torque_topic,
            self._wrench_cb,
            10,
        )
        self.create_subscription(ToolDropEvent, TOOL_DROP_TOPIC, self._tool_drop_cb, 10)

    def _log_startup(self):
        self.get_logger().info("task coordinator node 초기화 완료")
        self.get_logger().info(f"task request 토픽: {TASK_REQUEST_TOPIC}")
        self.get_logger().info(f"task control 토픽: {ROBOT_TASK_CONTROL_TOPIC}")
        self.get_logger().info(f"robot status 토픽: {ROBOT_STATUS_TOPIC}")
        self.get_logger().info(f"YOLO model: {self.yolo_model}")
        self.get_logger().info(f"grasp point mode: {self.grasp_point_mode}")

    def _task_request_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"{TASK_REQUEST_TOPIC} JSON 파싱 실패: {msg.data}"
            )
            self._publish_robot_status(
                "rejected",
                message="task request JSON을 해석하지 못했습니다.",
                reason="invalid_task_request_json",
            )
            return

        task = str(payload.get("task", "")).strip().lower()
        if task in ("bring", "pick"):
            self._handle_bring_request(payload)
            return
        if task == "return":
            command = payload.get("command") or {}
            self.start_return_sequence(command)
            return
        if task == "release":
            self._handle_release_request(payload)
            return
        if task == "home":
            self._handle_home_request(payload)
            return

        self.get_logger().warn(f"지원하지 않는 task request: {task}")
        self._publish_robot_status(
            "rejected",
            message="지원하지 않는 task request입니다.",
            reason="unsupported_task_request",
            command=payload.get("command"),
        )

    def _handle_bring_request(self, payload):
        command = payload.get("command") or {}
        tool_name = (
            payload.get("tool_name")
            or command.get("tool_name")
            or "unknown"
        )
        if tool_name == "unknown":
            self._publish_robot_status(
                "rejected",
                tool_name=tool_name,
                action="bring",
                message="대상 공구가 unknown이라 pick 요청을 시작하지 않습니다.",
                reason="unknown_tool",
                command=command,
            )
            return

        if self.is_running() or self.state.picking:
            self._publish_robot_status(
                "busy",
                tool_name=tool_name,
                action="bring",
                message="이미 작업 큐가 실행 중입니다.",
                reason="task_queue_busy",
                command=command,
            )
            return

        if all(key in payload for key in ("bx", "by", "bz")):
            self.state.target_label = tool_name
            self.state.current_command = command
            self.start_pick_sequence(
                float(payload["bx"]),
                float(payload["by"]),
                float(payload["bz"]),
                payload.get("vlm_yaw_deg"),
            )
            return

        self.state.current_command = command
        self.state.drawer_prepared_tool = None
        self.state.drawer_preparing_tool = None
        self.state._last_search_status_target = None
        self._publish_robot_status(
            "accepted",
            tool_name=tool_name,
            action="bring",
            message=f"{tool_name} 탐색 요청을 task coordinator가 수신했습니다.",
            command=command,
        )
        if not self._prepare_drawer_for_target(tool_name):
            self.state.target_label = tool_name
            self._publish_robot_status(
                "searching",
                tool_name=tool_name,
                action="bring",
                message=f"{tool_name} 탐색을 시작합니다.",
                command=command,
            )

    def _handle_release_request(self, payload):
        command = payload.get("command") or {}
        tool_name = command.get("tool_name", payload.get("tool_name", "unknown"))
        if self.is_running() or self.state.picking:
            self._publish_robot_status(
                "busy",
                tool_name=tool_name,
                action="release",
                message="로봇이 작업 중이라 release 요청은 즉시 실행하지 않습니다.",
                reason="already_picking",
                command=command,
            )
            return

        self.get_logger().info("release task request 수신: 그리퍼를 엽니다.")
        self.tool_hold_monitor.release_gripper()
        self._publish_robot_status(
            "done",
            tool_name=tool_name,
            action="release",
            message="그리퍼를 열었습니다.",
            command=command,
        )

    def _handle_home_request(self, payload):
        command = payload.get("command") or {}
        tool_name = command.get("tool_name", payload.get("tool_name", "unknown"))
        if self.is_running() or self.state.picking:
            self._publish_robot_status(
                "busy",
                tool_name=tool_name,
                action="home",
                message="로봇이 작업 중입니다. 먼저 정지한 뒤 Home 복귀를 요청해주세요.",
                reason="already_picking",
                command=command,
            )
            return

        self._publish_robot_status(
            "returning_home",
            tool_name=tool_name,
            action="home",
            message="Home 위치로 복귀하는 중입니다.",
            command=command,
        )
        ok = self.home_initializer.initialize()
        if not ok:
            self._publish_robot_status(
                "failed",
                tool_name=tool_name,
                action="home",
                message="Home 위치 복귀에 실패했습니다.",
                reason="home_motion_failed",
                command=command,
            )
            return
        self.tool_hold_monitor.release_gripper(reason="home_return")
        self._publish_robot_status(
            "done",
            tool_name=tool_name,
            action="home",
            message="Home 위치로 복귀하고 그리퍼를 열었습니다.",
            command=command,
        )

    def _task_control_cb(self, msg):
        action = str(msg.action or "").strip().lower()
        reason = str(msg.reason or "").strip()
        if not action:
            return
        if action == "pause":
            self._handle_pause(reason)
            return
        if action == "resume":
            self._handle_resume(reason)
            return
        if action == "cancel":
            self._handle_cancel(reason)
            return
        if action == "exit":
            self._handle_exit(reason)
            return
        self.get_logger().warn(f"지원하지 않는 task control action: {action}")

    def _handle_pause(self, reason):
        if self.exit_req.is_set():
            return False
        self.get_logger().warn(f"task queue pause 요청: reason={reason}")
        self.pause_req.set()
        self.resume_req.clear()
        with self._queue_lock:
            if self._current_step is not None:
                self._suspended_step = self._current_step
                self._suspended_task_name = self._current_task_name
        self.motion.cancel_current_goal(self.get_logger(), reason=reason or "pause")
        self._publish_robot_status(
            "paused",
            message="사용자 요청으로 작업을 일시정지합니다.",
            reason=reason or "pause_requested",
            command=self.state.current_command,
        )
        return True

    def _handle_resume(self, reason):
        self.get_logger().info(f"task queue resume 요청: reason={reason}")
        with self._queue_lock:
            if self._suspended_step is not None:
                self._queue.appendleft(
                    (self._suspended_task_name, self._suspended_step)
                )
                self._suspended_step = None
                self._suspended_task_name = None
            self.pause_req.clear()
            self.resume_req.clear()

        self._publish_robot_status(
            "resumed",
            message="작업을 재개합니다.",
            reason=reason or "resume_requested",
            command=self.state.current_command,
        )
        self._dispatch_next()
        return True

    def _handle_cancel(self, reason, publish_status=True):
        self.get_logger().warn(f"task queue cancel 요청: reason={reason}")
        task_running = self.is_running() or self.state.picking
        self.exit_req.set()
        self.pause_req.clear()
        self.resume_req.clear()
        self.motion.cancel_current_goal(self.get_logger(), reason=reason or "cancel")
        with self._queue_lock:
            self._queue.clear()
            self._suspended_step = None
            self._suspended_task_name = None

        if not task_running:
            self.exit_req.clear()
            self._clear_task_state()

        if publish_status:
            self._publish_robot_status(
                "cancelled",
                action="cancel",
                message="현재 작업을 취소했습니다. 다음 명령을 기다립니다.",
                reason=reason or "cancel_requested",
                command=self.state.current_command,
            )
        return True

    def _handle_exit(self, reason, publish_status=True):
        self.get_logger().warn(f"task queue exit 요청: reason={reason}")
        self.exit_req.set()
        self.pause_req.clear()
        self.resume_req.clear()
        self.motion.cancel_current_goal(self.get_logger(), reason=reason or "exit")
        with self._queue_lock:
            self._queue.clear()
            self._suspended_step = None
            self._suspended_task_name = None

        if publish_status:
            self._publish_robot_status(
                "cancelled",
                action="exit",
                message="사용자 요청으로 작업을 중단합니다.",
                reason=reason or "exit_requested",
                command=self.state.current_command,
            )

        self._start_exit_home_recovery(reason)
        return True

    def _start_exit_home_recovery(self, reason):
        if self._exit_home_thread is not None and self._exit_home_thread.is_alive():
            return
        self._exit_home_thread = threading.Thread(
            target=self._move_home_after_exit,
            args=(reason,),
            name="task_exit_home_recovery",
            daemon=True,
        )
        self._exit_home_thread.start()

    def _move_home_after_exit(self, reason):
        log = self.get_logger()
        command = self.state.current_command
        if not self._wait_for_task_queue_to_stop(log):
            self._publish_robot_status(
                "failed",
                action="exit",
                message="종료 요청 후 작업 큐가 멈추지 않아 Home 복귀를 시작하지 못했습니다.",
                reason="exit_queue_shutdown_timeout",
                command=command,
            )
            return

        self.exit_req.clear()
        log.info("종료 요청 후 Home 위치로 복귀합니다.")
        self._publish_robot_status(
            "returning_home",
            action="exit",
            message="종료 요청 후 Home 위치로 복귀합니다.",
            reason=reason or "exit_requested",
            command=command,
        )

        ok = self.motion.move_to_home_joints(log)
        if not ok:
            self._publish_robot_status(
                "failed",
                action="exit",
                message="종료 요청 후 Home 위치 복귀에 실패했습니다.",
                reason="exit_home_failed",
                command=command,
            )
            return

        try:
            self.tool_hold_monitor.release_gripper("exit_home_completed")
        except Exception as exc:
            log.warn(f"exit Home 복귀 후 그리퍼 열기 실패: {exc}")
            self._publish_robot_status(
                "failed",
                action="exit",
                message="종료 요청 후 Home 복귀는 완료했지만 그리퍼 열기에 실패했습니다.",
                reason="exit_gripper_release_failed",
                command=command,
            )
            return

        self._publish_robot_status(
            "done",
            action="exit",
            message="종료 요청 후 Home 위치로 복귀하고 그리퍼를 열었습니다.",
            reason=reason or "exit_home_completed",
            command=command,
        )

    def _wait_for_task_queue_to_stop(self, logger, timeout_sec=3.0):
        deadline = time.monotonic() + timeout_sec
        while self.is_running() and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.is_running():
            logger.warn("exit 요청 후 task queue 종료 대기 시간이 초과되었습니다.")
            return False
        return True

    def _tool_drop_cb(self, msg):
        payload = self._tool_drop_payload(msg)
        if payload.get("event") != "tool_dropped":
            return
        reason = payload.get("reason") or "tool_dropped"
        self.get_logger().warn(
            "공구 drop 감지를 task exit으로 처리합니다: "
            f"reason={reason}, tool={payload.get('tool_name', 'unknown')}"
        )
        self._handle_exit(reason, publish_status=False)

    def start_pick_sequence(self, bx, by, bz, vlm_yaw_deg=None):
        if self.is_running() or self.state.picking:
            self.get_logger().warn("이미 pick 동작 중이라 새 pick 요청을 무시합니다.")
            self._publish_robot_status(
                "busy",
                tool_name=self.state.target_label,
                action="bring",
                message="이미 작업 큐가 실행 중입니다.",
                reason="task_queue_busy",
                command=self.state.current_command,
            )
            return False

        self.state.picking = True
        self._publish_robot_status(
            "picking",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label} pick 동작을 시작합니다.",
            command=self.state.current_command,
        )
        steps = self.pick_runner.build_steps(bx, by, bz, vlm_yaw_deg)
        return self._load_queue("pick", steps)

    def start_return_sequence(self, command):
        tool_name = command.get("tool_name", "unknown")
        if self.is_running() or self.state.picking:
            self._publish_robot_status(
                "busy",
                tool_name=tool_name,
                action="return",
                message="이미 작업 큐가 실행 중입니다.",
                reason="task_queue_busy",
                command=command,
            )
            return False

        self.get_logger().info(f"반납 시퀀스 시작: tool={tool_name}")
        self.state.picking = True
        self.state.target_label = None
        self.state.current_command = command
        steps = self.return_runner.build_steps(command)
        return self._load_queue("return", steps)

    def _load_queue(self, task_name, steps):
        with self._queue_lock:
            if self._current_step is not None or self._step_thread_alive():
                self.get_logger().warn(
                    f"이미 task coordinator가 실행 중이라 {task_name} 요청을 무시합니다."
                )
                return False
            self.exit_req.clear()
            self.resume_req.clear()
            self._queue.clear()
            self._queue.extend((task_name, step) for step in steps)
            self.get_logger().info(
                f"{task_name} task queue 로딩 완료: {len(steps)} steps"
            )
        self._dispatch_next()
        return True

    def _dispatch_next(self):
        with self._queue_lock:
            if self.exit_req.is_set() or self.pause_req.is_set():
                return
            if self._current_step is not None or self._step_thread_alive():
                return
            if not self._queue:
                self._complete_queue_locked()
                return

            task_name, step = self._queue.popleft()
            self._current_task_name = task_name
            self._current_step = step
            self._step_thread = threading.Thread(
                target=self._execute_step,
                args=(task_name, step),
                name=f"task_step:{step.name}",
                daemon=True,
            )
            self.get_logger().info(f"task step 시작: {step.name}")
            self._step_thread.start()

    def _execute_step(self, task_name, step):
        ok = False
        exc = None
        try:
            ok = bool(step.execute())
        except Exception as error:
            exc = error
        self._finish_step(task_name, step, ok, exc)

    def _finish_step(self, task_name, step: TaskStep, ok, exc):
        if exc is not None:
            self._log_step_exception(self.get_logger(), task_name, step.name, exc)
            self._publish_task_exception_status(
                task_name,
                step.name,
                exc,
                reason="task_step_exception",
            )
            self._fail_queue(task_name)
            return

        with self._queue_lock:
            self._current_step = None
            self._current_task_name = None
            self._step_thread = None

            if ok and not self.pause_req.is_set():
                self.get_logger().info(f"task step 완료: {step.name}")
            elif self.pause_req.is_set() and step.retry_on_pause:
                self.get_logger().info(f"task step 일시정지 보관: {step.name}")
                if self._suspended_step is None:
                    self._suspended_step = step
                    self._suspended_task_name = task_name
                return
            elif self.exit_req.is_set():
                self.get_logger().info(f"{task_name} task queue 종료 요청으로 중단")
                self._run_cleanup_callbacks()
                self._clear_task_state()
                return
            elif not ok:
                self.get_logger().error(f"task step 실패: {step.name}")
                self._run_cleanup_callbacks()
                self._clear_task_state()
                return

        self._dispatch_next()

    def _fail_queue(self, task_name):
        with self._queue_lock:
            self._queue.clear()
            self._current_step = None
            self._current_task_name = None
            self._step_thread = None
        self.get_logger().info(f"{task_name} task queue 실패로 종료")
        self._run_cleanup_callbacks()
        self._clear_task_state()

    def _complete_queue_locked(self):
        self.get_logger().info("task queue 완료")
        self._step_thread = None
        self._run_cleanup_callbacks()
        self._clear_task_state()
        self.exit_req.clear()
        self.resume_req.clear()

    def clear_queue(self):
        with self._queue_lock:
            self._queue.clear()

    def is_running(self):
        with self._queue_lock:
            return (
                self._current_step is not None
                or self._step_thread_alive()
                or bool(self._queue)
            )

    def _step_thread_alive(self):
        return self._step_thread is not None and self._step_thread.is_alive()

    def _clear_task_state(self):
        self.state.picking = False
        self.state.target_label = None
        self.state.human_grasped_tool = False
        self.state.current_command = None
        self.state.drawer_prepared_tool = None
        self.state.drawer_preparing_tool = None

    def _run_cleanup_callbacks(self):
        try:
            self.tool_hold_monitor.stop("task_queue_finished")
        except Exception as exc:
            self.get_logger().warn(f"task cleanup callback 실패: {exc}")

    def _publish_task_exception_status(self, task_name, step_name, exc, reason):
        action = "bring" if task_name == "pick" else task_name
        exc_type = type(exc).__name__
        self._publish_robot_status(
            "failed",
            tool_name=self.state.target_label,
            action=action,
            message=(
                f"{task_name} task step 예외로 작업을 중단합니다: "
                f"{step_name} ({exc_type})"
            ),
            reason=reason,
            command=self.state.current_command,
        )

    @staticmethod
    def _log_step_exception(log, task_name, step_name, exc):
        log.error(
            f"{task_name} task step 예외 발생: "
            f"step={step_name}, error={type(exc).__name__}: {exc}"
        )
        log.error(traceback.format_exc())

    def _drawer_ready_for_target(self, target_label):
        drawer_id = self.drawer_flow.drawer_id_for_tool(target_label)
        if drawer_id is None:
            return True
        return self.state.drawer_prepared_tool == target_label

    def _prepare_drawer_for_target(self, target_label):
        drawer_id = self.drawer_flow.drawer_id_for_tool(target_label)
        if drawer_id is None:
            return False
        if self.state.drawer_preparing_tool == target_label:
            return True
        if self.state.picking:
            return True

        self.state.picking = True
        self.state.drawer_preparing_tool = target_label
        self._publish_robot_status(
            "opening_drawer",
            tool_name=target_label,
            action="bring",
            message=f"{target_label}가 들어있는 서랍을 엽니다.",
            command=self.state.current_command,
        )
        worker = threading.Thread(
            target=self._prepare_drawer_worker,
            args=(target_label, drawer_id),
            daemon=True,
        )
        worker.start()
        return True

    def _prepare_drawer_worker(self, target_label, drawer_id):
        try:
            log = self.get_logger()
            log.info(
                f"{target_label} 탐색 전 drawer {drawer_id}를 열고 관찰 위치로 이동합니다."
            )
            ok = self.drawer_flow.open_drawer(drawer_id, log)
            if ok:
                self._publish_robot_status(
                    "observing_drawer",
                    tool_name=target_label,
                    action="bring",
                    message=f"{target_label} 탐색을 위해 서랍 내부를 관찰합니다.",
                    command=self.state.current_command,
                )
                ok = self.drawer_flow.observe_drawer(drawer_id, log)

            if ok:
                self.state.target_label = target_label
                self.state.drawer_prepared_tool = target_label
                self._publish_robot_status(
                    "searching",
                    tool_name=target_label,
                    action="bring",
                    message=f"{target_label} 탐색을 시작합니다.",
                    command=self.state.current_command,
                )
            else:
                self._publish_robot_status(
                    "failed",
                    tool_name=target_label,
                    action="bring",
                    message=f"{target_label} 탐색 전 서랍 준비에 실패했습니다.",
                    reason="drawer_prepare_failed",
                    command=self.state.current_command,
                )
                self.state.target_label = None
                self.state.current_command = None
        finally:
            self.state.drawer_preparing_tool = None
            self.state.picking = False

    def _refine_pick_target_after_centering(self, target_label):
        if not self.pick_target_resolver.should_defer_vlm_until_top_view():
            return None
        if not self.frame_processor.has_camera_state():
            self.get_logger().warn("상단 view VLM 갱신을 위한 camera state가 없습니다.")
            return None

        color_image = self.state.color_image.copy()
        depth_image = self.state.depth_image.copy()
        intrinsics = dict(self.state.intrinsics)
        results = self.detector.detect(color_image)
        return self.pick_target_resolver.target_from_boxes(
            results[0].boxes,
            target_label,
            color_image,
            depth_image,
            intrinsics,
        )

    def _detect_home_tool_label(self):
        if self.state.color_image is None:
            return None
        results = self.detector.detect(self.state.color_image)
        boxes = results[0].boxes if results else None
        if boxes is None:
            return None
        supported = self.drawer_flow.supported_tool_labels()
        for box in boxes:
            label = self._box_label(box)
            if label in supported:
                return label
        return None

    def _box_label(self, box):
        try:
            class_id = int(box.cls[0])
        except (TypeError, IndexError):
            class_id = int(box.cls)
        return str(self.detector.names[class_id])

    def _motion_interrupted(self):
        return self.exit_req.is_set() or self.pause_req.is_set()

    @staticmethod
    def _cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(SEQUENCE_WAIT_POLL_SEC, max(0.0, remaining)))

    def _hand_grasp_cb(self, msg):
        result = self._human_grasp_payload(msg)
        result["_received_monotonic_sec"] = time.monotonic()
        self.hand_grasp_adapter.attach_base_position(result)
        self.state.last_grasp_result = result
        self.state.human_grasped_tool = bool(result.get("human_grasped_tool", False))

    def _tool_mask_lock_cb(self, msg):
        result = self._tool_mask_payload(msg)
        self.state.last_tool_mask_lock_result = result
        self.state.tool_mask_locked = bool(result.get("locked", False))

    def _hand_grasp_image_cb(self, msg):
        try:
            self.state.hand_grasp_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warn(f"잡기 인식 화면 변환 실패: {exc}")

    def _cam_info_cb(self, msg):
        self.state.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "ppx": msg.k[2],
            "ppy": msg.k[5],
        }

    def _color_cb(self, msg):
        self.state.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg):
        self.state.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def _wrench_cb(self, msg):
        self.state.latest_wrench = msg.wrench

    def run(self):
        self.home_initializer.initialize()
        self._process_frames()

    def _process_frames(self):
        if self.state.home_xyz is None or self.state.home_ori is None:
            return
        self.get_logger().info("task coordinator camera loop 대기 중...")

        while rclpy.ok():
            if not self.frame_processor.has_camera_state():
                time.sleep(0.01)
                continue

            if self.state.picking:
                self.display.show_robot(self.state.color_image)
                self.display.show_hand_grasp(self.state.hand_grasp_image)
                if self.display.wait_key() == 27:
                    break
                continue

            annotated_frame = self.frame_processor.process_current_frame()
            self.display.show_robot(annotated_frame)
            self.display.show_hand_grasp(self.state.hand_grasp_image)
            if self.display.wait_key() == 27:
                break

        self.display.close()

    def _publish_robot_status(
        self,
        status,
        tool_name=None,
        action=None,
        message="",
        reason="",
        command=None,
    ):
        self.status_publisher.publish(
            status,
            tool_name=tool_name,
            action=action,
            message=message,
            reason=reason,
            command=command,
        )

    def _publish_status_payload(self, payload):
        msg = RobotTaskStatus()
        msg.status = str(payload.get("status", "unknown"))
        msg.task = str(payload.get("task", ""))
        msg.tool_name = str(payload.get("tool_name", "unknown"))
        msg.action = str(payload.get("action", "unknown"))
        msg.message = str(payload.get("message", ""))
        msg.reason = str(payload.get("reason", ""))
        command = payload.get("command")
        msg.command_json = (
            json.dumps(command, ensure_ascii=False) if command is not None else ""
        )
        msg.payload_json = json.dumps(payload, ensure_ascii=False)
        self.robot_status_pub.publish(msg)

    def _publish_tool_drop_payload(self, payload):
        msg = ToolDropEvent()
        msg.event = str(payload.get("event", ""))
        msg.tool_name = str(payload.get("tool_name", "unknown"))
        msg.action = str(payload.get("action", "unknown"))
        msg.reason = str(payload.get("reason", ""))
        width_mm = payload.get("width_mm")
        msg.has_width_mm = width_mm is not None
        msg.width_mm = float(width_mm) if width_mm is not None else 0.0
        status = payload.get("status")
        msg.gripper_status_json = (
            json.dumps(status, ensure_ascii=False) if status is not None else ""
        )
        msg.error = str(payload.get("error", ""))
        command = payload.get("command")
        msg.command_json = (
            json.dumps(command, ensure_ascii=False) if command is not None else ""
        )
        msg.payload_json = json.dumps(payload, ensure_ascii=False)
        self.tool_drop_pub.publish(msg)

    @staticmethod
    def _tool_drop_payload(msg):
        payload = json.loads(msg.payload_json) if msg.payload_json else {}
        payload.update(
            {
                "event": msg.event,
                "tool_name": msg.tool_name,
                "action": msg.action,
                "reason": msg.reason,
            }
        )
        if msg.has_width_mm:
            payload["width_mm"] = msg.width_mm
        if msg.gripper_status_json:
            payload["status"] = json.loads(msg.gripper_status_json)
        if msg.error:
            payload["error"] = msg.error
        if msg.command_json:
            payload["command"] = json.loads(msg.command_json)
        return payload

    @staticmethod
    def _human_grasp_payload(msg):
        payload = json.loads(msg.payload_json) if msg.payload_json else {}
        payload.update(
            {
                "state": msg.state,
                "human_grasped_tool": msg.human_grasped_tool,
                "grasp_counter": msg.grasp_counter,
                "grasp_score": msg.grasp_score,
                "ml_raw_state": msg.ml_raw_state,
                "ml_stable_state": msg.ml_stable_state,
                "ml_grasp_confirmed": msg.ml_grasp_confirmed,
                "depth_grasp_confirmed": msg.depth_grasp_confirmed,
                "mask_locked": msg.mask_locked,
                "mask_source": msg.mask_source,
            }
        )
        if msg.has_tool_label:
            payload["tool_label"] = msg.tool_label
        if msg.has_tool_roi:
            payload["tool_roi"] = list(msg.tool_roi)
        if msg.has_hand_pixel:
            payload["hand_pixel"] = {
                "u": msg.hand_u,
                "v": msg.hand_v,
                "source": msg.hand_pixel_source,
            }
        return payload

    @staticmethod
    def _tool_mask_payload(msg):
        payload = json.loads(msg.payload_json) if msg.payload_json else {}
        payload.update(
            {
                "locked": msg.locked,
                "mask_source": msg.mask_source,
                "tool_roi": list(msg.tool_roi) if msg.has_tool_roi else None,
            }
        )
        if msg.has_reason:
            payload["reason"] = msg.reason
        return payload


def main():
    rclpy.init()
    node = TaskCoordinatorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="task_coordinator_executor",
        daemon=True,
    )
    executor_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
