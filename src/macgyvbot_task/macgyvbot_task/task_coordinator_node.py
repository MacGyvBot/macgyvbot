#!/usr/bin/env python3
"""ROS node that owns task queue execution and task-control handling."""

from __future__ import annotations

from collections import deque
from dataclasses import replace
import math
from pathlib import Path
import re
import threading
import time
import traceback

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from macgyvbot_interfaces.msg import (
    HumanGraspResult,
    RobotTaskControl,
    RobotTaskStatus,
    TaskRequest,
    ToolCommand,
    ToolDropEvent,
    ToolMaskLock,
)
from macgyvbot_interfaces.srv import SetGripper

from macgyvbot_config.drawer import DRAWER_OBSERVATION_J6_DEG
from macgyvbot_config.models import HAND_GRASP_SAM_CHECKPOINT_NAME, YOLO_MODEL_NAME
from macgyvbot_config.pick import PICK_GRASP_YAW_OFFSET_DEG
from macgyvbot_config.robot import GROUP_NAME, HOME_JOINTS, WRIST_JOINT_NAME
from macgyvbot_config.timing import (
    CAMERA_LOOP_IDLE_SLEEP_SEC,
    TASK_QUEUE_STOP_POLL_SEC,
)
from macgyvbot_config.topics import (
    CAMERA_COLOR_TOPIC,
    CAMERA_DEPTH_TOPIC,
    CAMERA_INFO_TOPIC,
    FORCE_TORQUE_TOPIC,
    HAND_GRASP_IMAGE_TOPIC,
    HAND_GRASP_MASK_LOCK_TOPIC,
    HAND_GRASP_TOPIC,
    MANUAL_GRIPPER_SERVICE,
    ROBOT_STATUS_TOPIC,
    TASK_CONTROL_TOPIC,
    TASK_REQUEST_TOPIC,
    TOOL_DROP_TOPIC,
)
from macgyvbot_config.vlm import (
    DEFAULT_GRASP_POINT_MODE,
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_VLM,
    VLM_GRASP_SERVICE_NAME,
    VLM_ONLY_MODES,
)
from macgyvbot_domain.target_models import PickTarget
from macgyvbot_manipulation.drawer_motion import DrawerMotionFlow
from macgyvbot_manipulation.moveit_controller import MoveItController
from macgyvbot_manipulation.onrobot_gripper import RG
from macgyvbot_manipulation.robot_pose import (
    get_ee_matrix,
    orientation_from_joint_positions,
)
from macgyvbot_manipulation.timing import cooperative_wait
from macgyvbot_perception.depth_projection import DepthProjector
from macgyvbot_perception.grasp_point.grasp_point_selector import (
    GraspPointSelector,
    normalize_grasp_point_mode,
)
from macgyvbot_perception.grasp_point.grasp_method import (
    estimate_yaw_from_binary_crop,
)
from macgyvbot_perception.grasp_point.mask_image_for_grasp_detection import (
    generate_sam_depth_mask_image_for_grasp_detection,
)
from macgyvbot_perception.hand_tool_grasp.calculations import (
    depth_to_mm,
)
from macgyvbot_perception.hand_tool_grasp.sam_tool_mask import (
    BBoxPromptSegmenter,
)
from macgyvbot_perception.pick_target_resolver import PickTargetResolver
from macgyvbot_perception.yolo_detector import YoloDetector
from macgyvbot_resources.calibration import resolve_calibration_file
from macgyvbot_task.application import (
    RobotStatusPublisher,
    TaskRuntimeState,
    ToolDropStatusReporter,
)
from macgyvbot_task.application.adapters.hand_grasp_result_adapter import (
    HandGraspResultAdapter,
)
from macgyvbot_task.application.adapters.vlm_grasp_service_client import (
    VLMGraspServiceClient,
)
from macgyvbot_task.application.display.debug_display import DebugDisplay
from macgyvbot_task.application.pick_flow.pick_frame_processor import (
    PickFrameProcessor,
)
from macgyvbot_task.application.return_flow.return_perception_adapter import (
    ReturnPerceptionAdapter,
)
from macgyvbot_task.application.pick_flow.pick_sequence import PickSequenceRunner
from macgyvbot_task.application.return_flow.return_sequence import ReturnSequenceRunner
from macgyvbot_task.application.robot.robot_home_initializer import (
    RobotHomeInitializer,
)
from macgyvbot_task.application.task_control.task_step import TaskStep


class VLMStatusLogger:
    """Forward VLM logs to ROS logger and robot status topic."""

    _MODEL_ID_RE = re.compile(r"\bmodel_id=([^,\s)]+)")
    _SOURCE_RE = re.compile(r"\bsource=([^,\s)]+)")

    def __init__(self, logger, publish_status_payload):
        self._logger = logger
        self._publish_status_payload = publish_status_payload
        self._vlm_inference_active = False

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

        if "로드 시작" in text or "weights loading" in text:
            status = "vlm_loading"
        elif "로드 완료" in text or "weights loaded" in text:
            status = "vlm_ready"
            self._vlm_inference_active = False
        elif "inference progress" in text:
            if self._vlm_inference_active:
                return
            self._vlm_inference_active = True
            status = "vlm_inferencing"
        elif "inference complete" in text:
            self._vlm_inference_active = False
            return
        elif level == "error" or "실패" in text:
            status = "vlm_error"
            self._vlm_inference_active = False
        elif level == "warn" or "CPU 실행" in text or "not using CUDA" in text:
            status = "vlm_warning"
        else:
            return

        self._publish_status_payload(
            {
                "status": status,
                "tool_name": "unknown",
                "action": "system",
                "message": self._chat_message(status, text),
            }
        )

    @classmethod
    def _chat_message(cls, status, text):
        model_label = cls._extract_model_label(text)
        model_prefix = f"({model_label}) " if model_label else ""

        if status == "vlm_loading":
            return f"{model_prefix}VLM 모델을 로드하는 중입니다."
        if status == "vlm_ready":
            return f"{model_prefix}VLM 모델 로드가 완료되었습니다."
        if status == "vlm_inferencing":
            return f"{model_prefix}VLM으로 공구 파지점을 탐색하는 중입니다."
        if status == "vlm_warning":
            return f"{model_prefix}VLM 모델 상태를 확인해야 합니다."
        if status == "vlm_error":
            return f"{model_prefix}VLM 모델 처리 중 오류가 발생했습니다."
        return text

    @classmethod
    def _extract_model_label(cls, text):
        match = cls._MODEL_ID_RE.search(text)
        if match is None:
            match = cls._SOURCE_RE.search(text)
        if match is None:
            return ""

        model = match.group(1).strip()
        if not model:
            return ""

        model = model.rstrip(".,)")
        if "/" in model:
            model = model.rsplit("/", 1)[-1]
        if "__" in model:
            model = model.rsplit("__", 1)[-1]
        return model


class TaskCoordinatorNode(Node):
    """Own pick/return workflow queues and task-control requests."""

    def __init__(self):
        super().__init__("task_coordinator_node")

        self.declare_parameter("display_debug_windows", False)
        self.declare_parameter("manual_gripper_service", MANUAL_GRIPPER_SERVICE)
        self.bridge = CvBridge()
        self.display_debug_windows = self._read_bool_parameter(
            "display_debug_windows",
            False,
        )
        self.display = DebugDisplay(enabled=self.display_debug_windows)
        self.exit_req = threading.Event()
        self.pause_req = threading.Event()
        self.resume_req = threading.Event()
        self.handoff_pending_req = threading.Event()
        self.handoff_retry_req = threading.Event()
        self.handoff_fallback_req = threading.Event()

        self._queue = deque()
        self._queue_lock = threading.RLock()
        self._step_thread = None
        self._current_step = None
        self._current_task_name = None
        self._suspended_step = None
        self._suspended_task_name = None
        self._exit_home_thread = None
        self._vlm_preload_timer = None
        self._manual_gripper_lock = threading.Lock()

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
        self.grasp_detection_sam_config = self._read_grasp_detection_sam_config()
        self.grasp_detection_sam_segmenter = None
        self.grasp_detection_sam_init_attempted = False
        self.vlm_service_client = VLMGraspServiceClient(
            self,
            self.bridge,
            **self._read_vlm_service_config(),
        )
        if self.grasp_point_mode not in (GRASP_POINT_MODE_VLM, *VLM_ONLY_MODES):
            self.grasp_point_selector.preload_vlm_if_needed()

        calib_file = resolve_calibration_file("T_gripper2camera.npy")
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
            cooperative_wait,
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
            "handoff_pending": self.handoff_pending_req,
            "handoff_retry": self.handoff_retry_req,
            "handoff_fallback": self.handoff_fallback_req,
        }
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
        self.return_perception = ReturnPerceptionAdapter(
            self.state,
            self.detector,
            self.drawer_flow,
            self.frame_processor,
            self.pick_target_resolver,
            self.depth_projector,
            self.get_logger(),
        )
        self.pick_runner = PickSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self.state,
            self.tool_hold_monitor,
            refine_pick_target=self._refine_pick_target_after_centering,
            generate_grasp_detection_mask_images=(
                self._generate_grasp_detection_mask_images_after_vlm_observe
            ),
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
            detect_store_tool_label=(
                self.return_perception.detect_store_tool_label
            ),
            resolve_store_tool_target=(
                self.return_perception.resolve_store_tool_target
            ),
            resolve_drawer_marker_target=(
                self.return_perception.resolve_drawer_marker_target
            ),
        )

        self._create_subscriptions()
        self._create_services()
        self._log_startup()
        self._vlm_preload_timer = self.create_timer(
            3.0,
            self._preload_vlm_after_startup,
        )

    def _base_to_camera_matrix(self):
        return get_ee_matrix(self.robot) @ self.gripper2cam

    def _preload_vlm_after_startup(self):
        if self._vlm_preload_timer is not None:
            self._vlm_preload_timer.cancel()

        self.grasp_point_selector.preload_vlm_if_needed()

    def _drawer_observation_orientation(self):
        joint_positions = dict(HOME_JOINTS)
        joint_positions[WRIST_JOINT_NAME] = math.radians(
            DRAWER_OBSERVATION_J6_DEG
        )
        return orientation_from_joint_positions(self.robot, joint_positions)

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
        self.declare_parameter(
            "sam_checkpoint",
            str(Path("weights") / HAND_GRASP_SAM_CHECKPOINT_NAME),
        )
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

    def _read_grasp_detection_sam_config(self):
        self.declare_parameter("sam_depth_tolerance_mm", 30.0)
        self.declare_parameter("sam_depth_min_valid_ratio", 0.03)
        self.declare_parameter("sam_depth_expand_iterations", 1)
        return {
            "sam_enabled": self._read_bool_parameter("sam_enabled", True),
            "sam_checkpoint": str(self.get_parameter("sam_checkpoint").value).strip(),
            "sam_backend": str(self.get_parameter("sam_backend").value).strip(),
            "sam_model_type": str(self.get_parameter("sam_model_type").value).strip(),
            "sam_device": str(self.get_parameter("sam_device").value).strip(),
            "sam_depth_tolerance_mm": float(
                self.get_parameter("sam_depth_tolerance_mm").value
            ),
            "sam_depth_min_valid_ratio": float(
                self.get_parameter("sam_depth_min_valid_ratio").value
            ),
            "sam_depth_expand_iterations": int(
                self.get_parameter("sam_depth_expand_iterations").value
            ),
        }

    def _create_grasp_detection_sam_segmenter(self, config):
        if not bool(config["sam_enabled"]):
            self.get_logger().info(
                "VLM 관찰 위치 grasp detection SAM mask 저장 비활성화"
            )
            return None

        try:
            return BBoxPromptSegmenter(
                backend=config["sam_backend"],
                checkpoint_path=config["sam_checkpoint"],
                model_type=config["sam_model_type"],
                device=config["sam_device"],
            )
        except Exception as exc:
            self.get_logger().warn(
                f"VLM 관찰 위치 grasp detection SAM 초기화 실패: {exc}"
            )
            return None

    def _grasp_detection_sam(self):
        if self.grasp_detection_sam_segmenter is not None:
            return self.grasp_detection_sam_segmenter
        if self.grasp_detection_sam_init_attempted:
            return None

        self.grasp_detection_sam_init_attempted = True
        self.grasp_detection_sam_segmenter = (
            self._create_grasp_detection_sam_segmenter(
                self.grasp_detection_sam_config,
            )
        )
        return self.grasp_detection_sam_segmenter

    def _read_vlm_service_config(self):
        self.declare_parameter("vlm_service_name", VLM_GRASP_SERVICE_NAME)
        self.declare_parameter("vlm_service_wait_timeout_sec", 2.0)
        self.declare_parameter("vlm_service_response_timeout_sec", 30.0)
        return {
            "service_name": str(self.get_parameter("vlm_service_name").value).strip(),
            "wait_timeout_sec": float(
                self.get_parameter("vlm_service_wait_timeout_sec").value
            ),
            "response_timeout_sec": float(
                self.get_parameter("vlm_service_response_timeout_sec").value
            ),
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
        self.create_subscription(
            TaskRequest,
            TASK_REQUEST_TOPIC,
            self._task_request_cb,
            10,
        )
        self.create_subscription(
            RobotTaskControl,
            TASK_CONTROL_TOPIC,
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

    def _create_services(self):
        self.manual_gripper_service_name = (
            str(self.get_parameter("manual_gripper_service").value).strip()
            or MANUAL_GRIPPER_SERVICE
        )
        self.create_service(
            SetGripper,
            self.manual_gripper_service_name,
            self._manual_gripper_cb,
        )

    def _log_startup(self):
        self.get_logger().info("task coordinator node 초기화 완료")
        self.get_logger().info(f"task request 토픽: {TASK_REQUEST_TOPIC}")
        self.get_logger().info(f"task control 토픽: {TASK_CONTROL_TOPIC}")
        self.get_logger().info(f"robot status 토픽: {ROBOT_STATUS_TOPIC}")
        self.get_logger().info(
            f"manual gripper service: {self.manual_gripper_service_name}"
        )
        self.get_logger().info(f"YOLO model: {self.yolo_model}")
        self.get_logger().info(f"grasp point mode: {self.grasp_point_mode}")

    def _manual_gripper_cb(self, request, response):
        requested_width_mm = float(getattr(request, "width_mm", 0.0))
        source = str(getattr(request, "source", "") or "unknown").strip()
        response.requested_width_mm = requested_width_mm
        response.applied_width_mm = 0.0

        allowed, reason = self._manual_gripper_is_allowed()
        if not allowed:
            return self._reject_manual_gripper(
                response,
                reason,
                "작업 실행 중에는 수동 그리퍼 조작을 사용할 수 없습니다.",
                requested_width_mm,
                source,
            )

        max_width_mm = self._manual_gripper_max_width_mm()
        if requested_width_mm < 0.0 or requested_width_mm > max_width_mm:
            return self._reject_manual_gripper(
                response,
                "width_out_of_range",
                f"그리퍼 폭은 0-{max_width_mm:.0f} mm 범위에서 요청해야 합니다.",
                requested_width_mm,
                source,
            )

        status = self._safe_gripper_status()
        if status is None:
            return self._reject_manual_gripper(
                response,
                "gripper_status_unavailable",
                "그리퍼 상태를 읽지 못해 수동 조작을 중단합니다.",
                requested_width_mm,
                source,
            )
        if status[0]:
            return self._reject_manual_gripper(
                response,
                "gripper_busy",
                "그리퍼가 이미 움직이는 중이라 새 명령을 보낼 수 없습니다.",
                requested_width_mm,
                source,
            )
        if any(status[2:]):
            return self._reject_manual_gripper(
                response,
                "gripper_safety_error",
                "그리퍼 안전 상태가 정상적이지 않아 수동 조작을 중단합니다.",
                requested_width_mm,
                source,
            )

        width_raw = int(round(requested_width_mm * 10.0))
        with self._manual_gripper_lock:
            try:
                self.gripper.move_gripper(width_raw)
            except Exception as exc:
                self.get_logger().error(
                    "manual gripper command failed: "
                    f"source={source}, width_mm={requested_width_mm:.1f}, "
                    f"error={type(exc).__name__}: {exc}"
                )
                return self._reject_manual_gripper(
                    response,
                    "gripper_command_failed",
                    "그리퍼 명령 전송에 실패했습니다.",
                    requested_width_mm,
                    source,
                )

        response.success = True
        response.status = "accepted"
        response.message = f"그리퍼를 {requested_width_mm:.0f} mm로 적용합니다."
        response.applied_width_mm = requested_width_mm
        self.get_logger().info(
            "manual gripper command accepted: "
            f"source={source}, width_mm={requested_width_mm:.1f}, raw={width_raw}"
        )
        return response

    def _manual_gripper_is_allowed(self):
        with self._queue_lock:
            active_step = self._current_step is not None or self._step_thread_alive()
            queued = bool(self._queue)
            paused = self.pause_req.is_set()

        exit_home_running = (
            self._exit_home_thread is not None
            and self._exit_home_thread.is_alive()
        )
        if exit_home_running:
            return False, "exit_home_running"
        if active_step:
            return False, "task_step_active"
        if queued and not paused:
            return False, "task_queue_active"
        if self.state.picking and not paused:
            return False, "task_active"
        return True, "safe"

    def _manual_gripper_max_width_mm(self):
        return float(getattr(self.gripper, "max_width", 0) or 0) / 10.0

    def _safe_gripper_status(self):
        try:
            return self.gripper.get_status()
        except Exception as exc:
            self.get_logger().warn(
                f"manual gripper status read failed: {type(exc).__name__}: {exc}"
            )
            return None

    def _reject_manual_gripper(
        self,
        response,
        status,
        message,
        requested_width_mm,
        source,
    ):
        response.success = False
        response.status = status
        response.message = message
        response.applied_width_mm = 0.0
        self.get_logger().warn(
            "manual gripper command rejected: "
            f"status={status}, source={source}, width_mm={requested_width_mm:.1f}, "
            f"message={message}"
        )
        return response

    def _task_request_cb(self, msg):
        task = str(msg.task or "").strip().lower()
        if task in ("bring", "pick"):
            self._handle_bring_request(msg)
            return
        if task == "return":
            self.start_return_sequence(self._task_request_command(msg))
            return
        if task == "release":
            self._handle_release_request(msg)
            return
        if task == "home":
            self._handle_home_request(msg)
            return

        self.get_logger().warn(f"지원하지 않는 task request: {task}")
        self._publish_robot_status(
            "rejected",
            message="지원하지 않는 task request입니다.",
            reason="unsupported_task_request",
            command=self._task_request_command(msg),
        )

    def _handle_bring_request(self, request):
        command = self._task_request_command(request)
        tool_name = (
            request.tool_name
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

        if request.has_base_target:
            self.state.target_label = tool_name
            self.state.current_command = command
            self.start_pick_sequence(
                request.bx,
                request.by,
                request.bz,
                request.vlm_yaw_deg if request.has_vlm_yaw_deg else None,
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

    def _handle_release_request(self, request):
        command = self._task_request_command(request)
        tool_name = command.get("tool_name", request.tool_name or "unknown")
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

    def _handle_home_request(self, request):
        command = self._task_request_command(request)
        tool_name = command.get("tool_name", request.tool_name or "unknown")
        if self.handoff_pending_req.is_set():
            self._handle_handoff_fallback("home_requested_during_handoff_inspection")
            return

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
        if action == "handoff_retry":
            self._handle_handoff_retry(reason)
            return
        if action == "handoff_fallback":
            self._handle_handoff_fallback(reason)
            return
        if action == "pause":
            self._handle_pause(reason)
            return
        if action == "resume":
            self._handle_resume(reason)
            return
        if action == "cancel":
            if self.handoff_pending_req.is_set():
                self._handle_handoff_fallback(reason or "cancel_during_handoff_inspection")
                return
            self._handle_cancel(reason)
            return
        if action == "exit":
            self._handle_exit(reason)
            return
        self.get_logger().warn(f"지원하지 않는 task control action: {action}")

    def _handle_handoff_retry(self, reason):
        if not self.handoff_pending_req.is_set():
            self.get_logger().warn(
                f"handoff retry 요청을 무시합니다: pending=false, reason={reason}"
            )
            self._publish_robot_status(
                "rejected",
                action="handoff_retry",
                message="현재 재시도할 handoff 인식 대기가 없습니다.",
                reason="handoff_retry_not_pending",
                command=self.state.current_command,
            )
            return False

        self.get_logger().info(f"handoff inspection retry 요청: reason={reason}")
        self.handoff_fallback_req.clear()
        self.handoff_retry_req.set()
        self._publish_robot_status(
            "searching_hand",
            action="bring",
            message="사용자 손 인식을 다시 시도합니다.",
            reason=reason or "handoff_retry_requested",
            command=self.state.current_command,
        )
        return True

    def _handle_handoff_fallback(self, reason):
        if not self.handoff_pending_req.is_set():
            self.get_logger().warn(
                f"handoff fallback 요청을 무시합니다: pending=false, reason={reason}"
            )
            return False

        self.get_logger().warn(f"handoff inspection fallback 요청: reason={reason}")
        self.handoff_retry_req.clear()
        self.handoff_fallback_req.set()
        self._publish_robot_status(
            "returning_home",
            action="bring",
            message="사용자 손 인식을 중단하고 공구를 원래 위치로 복귀합니다.",
            reason=reason or "handoff_fallback_requested",
            command=self.state.current_command,
        )
        return True

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
            time.sleep(TASK_QUEUE_STOP_POLL_SEC)
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
        self.handoff_pending_req.clear()
        self.handoff_retry_req.clear()
        self.handoff_fallback_req.clear()
        self.state.picking = False
        self.state.target_label = None
        self.state.human_grasped_tool = False
        self.state.current_command = None
        self.state.drawer_prepared_tool = None
        self.state.drawer_preparing_tool = None
        self.state.grasp_detection_mask_images = None
        self.state.grasp_detection_mask_target = None

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
        if self.grasp_point_mode not in (GRASP_POINT_MODE_VLM, *VLM_ONLY_MODES):
            target = self.pick_target_resolver.target_from_boxes(
                results[0].boxes,
                target_label,
                color_image,
                depth_image,
                intrinsics,
            )
            if self.grasp_point_mode != GRASP_POINT_MODE_CENTER:
                return target
            return self._target_with_mask_pca_yaw(target, target_label)

        matched_box = self.pick_target_resolver.matching_box(
            results[0].boxes,
            target_label,
        )
        if matched_box is None:
            return PickTarget(
                found=False,
                label=target_label,
                pixel=None,
                base_xyz=None,
                depth_m=None,
                yaw_deg=None,
                reason="target_not_found",
            )

        box, label = matched_box
        bbox_xyxy = box.xyxy[0].cpu().numpy().tolist()
        response = self.vlm_service_client.infer_grasp(
            color_image=color_image,
            bbox_xyxy=bbox_xyxy,
            label=label,
            target_label=target_label,
            mode=self.grasp_point_mode,
            interrupted=self._motion_interrupted,
        )
        if response is None:
            self.get_logger().warn(
                "Top-view VLM service refine failed. Keeping existing pick plan."
            )
            return PickTarget(
                found=False,
                label=target_label,
                pixel=None,
                base_xyz=None,
                depth_m=None,
                yaw_deg=None,
                reason="vlm_service_failed",
            )

        orientation_rpy_deg = list(response.orientation_rpy_deg) or None
        pca_yaw_deg = self._mask_pca_yaw_for_target(target_label)
        if pca_yaw_deg is not None:
            orientation_rpy_deg = [0.0, 0.0, float(pca_yaw_deg)]

        return self.pick_target_resolver.target_from_selected_grasp(
            label,
            target_label,
            (
                int(response.pixel_u),
                int(response.pixel_v),
                str(response.source or self.grasp_point_mode),
                orientation_rpy_deg,
            ),
            depth_image,
            intrinsics,
        )

    def _target_with_mask_pca_yaw(self, target, target_label):
        if target is None or not target.found:
            return target
        pca_yaw_deg = self._mask_pca_yaw_for_target(target_label)
        if pca_yaw_deg is None:
            return target
        return replace(target, yaw_deg=float(pca_yaw_deg))

    def _mask_pca_yaw_for_target(self, target_label):
        if self.state.grasp_detection_mask_target != target_label:
            return None
        images = self.state.grasp_detection_mask_images or []
        if not images:
            return None

        yaw_deg, debug = estimate_yaw_from_binary_crop(images[0])
        if yaw_deg is None:
            self.get_logger().warn(
                "grasp detection binary crop PCA yaw failed: "
                f"reason={debug.get('reason')}, pixels={debug.get('num_pixels')}"
            )
            return None

        adjusted_yaw_deg = float(yaw_deg) + PICK_GRASP_YAW_OFFSET_DEG
        self.get_logger().info(
            "grasp detection binary crop PCA yaw applied: "
            f"target={target_label}, yaw={yaw_deg:.1f}deg, "
            f"offset={PICK_GRASP_YAW_OFFSET_DEG:.1f}deg, "
            f"adjusted_yaw={adjusted_yaw_deg:.1f}deg, "
            f"pixels={debug.get('num_pixels')}"
        )
        return adjusted_yaw_deg

    def _generate_grasp_detection_mask_images_after_vlm_observe(self, target_label):
        if not self.frame_processor.has_camera_state():
            self.get_logger().warn(
                "grasp detection mask image 생성을 위한 camera state가 없습니다."
            )
            return None

        color_image = self.state.color_image.copy()
        results = self.detector.detect(color_image)
        matched_box = self.pick_target_resolver.matching_box(
            results[0].boxes,
            target_label,
        )
        if matched_box is None:
            self.get_logger().warn(
                f"{target_label} grasp detection mask image 생성을 위한 YOLO bbox가 없습니다."
            )
            return None

        box, _label = matched_box
        bbox_xyxy = box.xyxy[0].cpu().numpy().tolist()
        depth_mm = depth_to_mm(self.state.depth_image.copy(), "passthrough")
        images = generate_sam_depth_mask_image_for_grasp_detection(
            color_image=color_image,
            depth_mm=depth_mm,
            bbox_xyxy=bbox_xyxy,
            sam_segmenter=self._grasp_detection_sam(),
            data_root=Path("src/macgyvbot_perception/data"),
            filename_prefix=f"pick_vlm_observe_{target_label}",
            sam_depth_tolerance_mm=(
                self.grasp_detection_sam_config["sam_depth_tolerance_mm"]
            ),
            sam_depth_min_valid_ratio=(
                self.grasp_detection_sam_config["sam_depth_min_valid_ratio"]
            ),
            sam_depth_expand_iterations=(
                self.grasp_detection_sam_config["sam_depth_expand_iterations"]
            ),
        )
        if images is None:
            self.get_logger().warn(
                f"{target_label} VLM 관찰 위치 SAM+Depth mask 생성 실패"
            )
            self.state.grasp_detection_mask_images = None
            self.state.grasp_detection_mask_target = None
            return None

        self.state.grasp_detection_mask_images = images
        self.state.grasp_detection_mask_target = target_label
        self.get_logger().info(
            f"{target_label} VLM 관찰 위치 SAM+Depth grasp detection mask image 저장 완료: "
            "src/macgyvbot_perception/data/yaw_pca"
        )
        return images

    def _motion_interrupted(self):
        return self.exit_req.is_set() or self.pause_req.is_set()

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
                time.sleep(CAMERA_LOOP_IDLE_SLEEP_SEC)
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
        msg.command = self._tool_command_message(payload.get("command") or {})
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
        msg.error = str(payload.get("error", ""))
        msg.command = self._tool_command_message(payload.get("command") or {})
        self.tool_drop_pub.publish(msg)

    @staticmethod
    def _tool_command_message(command):
        msg = ToolCommand()
        msg.action = str(command.get("action", "unknown"))
        msg.tool_name = str(command.get("tool_name", "unknown"))
        msg.target_mode = str(command.get("target_mode", "unknown"))
        msg.raw_text = str(command.get("raw_text", ""))
        msg.match_method = str(command.get("match_method", "unknown"))
        msg.confidence = float(command.get("confidence", 0.0))
        return msg

    @staticmethod
    def _task_request_command(msg):
        command = msg.command
        return {
            "action": command.action,
            "tool_name": command.tool_name,
            "target_mode": command.target_mode,
            "raw_text": command.raw_text,
            "match_method": command.match_method,
            "confidence": command.confidence,
        }

    @staticmethod
    def _tool_drop_payload(msg):
        payload = {
            "event": msg.event,
            "tool_name": msg.tool_name,
            "action": msg.action,
            "reason": msg.reason,
            "command": TaskCoordinatorNode._tool_command_payload(msg.command),
        }
        if msg.has_width_mm:
            payload["width_mm"] = msg.width_mm
        if msg.error:
            payload["error"] = msg.error
        return payload

    @staticmethod
    def _human_grasp_payload(msg):
        payload = {
            "state": msg.state,
            "hand_present": msg.hand_present,
            "human_grasped_tool": msg.human_grasped_tool,
            "grasp_counter": msg.grasp_counter,
            "grasp_score": msg.grasp_score,
            "ml_required": msg.ml_required,
            "ml_grasp_confirmed": msg.ml_grasp_confirmed,
            "ml_confidence_ok": msg.ml_confidence_ok,
            "ml_raw_state": msg.ml_raw_state,
            "ml_stable_state": msg.ml_stable_state,
            "depth_required": msg.depth_required,
            "depth_available": msg.depth_available,
            "depth_grasp_ok": msg.depth_grasp_ok,
            "depth_grasp_confirmed": msg.depth_grasp_confirmed,
            "depth_contact_count": msg.depth_contact_count,
            "mask_locked": msg.mask_locked,
            "locked_mask_grasp_ok": msg.locked_mask_grasp_ok,
            "mask_contact_confirmed": msg.mask_contact_confirmed,
            "mask_proximity_ok": msg.mask_proximity_ok,
            "mask_near_or_contact": msg.mask_near_or_contact,
            "mask_source": msg.mask_source,
            "mask_contact_count": msg.mask_contact_count,
        }
        payload["ml_confidence"] = (
            msg.ml_confidence if msg.has_ml_confidence else None
        )
        payload["tool_depth_mm"] = (
            msg.tool_depth_mm if msg.has_tool_depth_mm else None
        )
        payload["min_hand_tool_depth_diff_mm"] = (
            msg.min_hand_tool_depth_diff_mm
            if msg.has_min_hand_tool_depth_diff_mm
            else None
        )
        payload["min_landmark_to_tool_distance"] = (
            msg.min_landmark_to_tool_distance
            if msg.has_min_landmark_to_tool_distance
            else None
        )
        if msg.has_tool_label:
            payload["tool_label"] = msg.tool_label
        if msg.has_tool_confidence:
            payload["tool_confidence"] = msg.tool_confidence
        if msg.has_tool_roi:
            payload["tool_roi"] = list(msg.tool_roi)
        if msg.has_hand_pixel:
            payload["hand_pixel"] = {
                "u": msg.hand_u,
                "v": msg.hand_v,
                "source": msg.hand_pixel_source,
            }
        if msg.has_active_hand_index:
            payload["active_hand_index"] = msg.active_hand_index
        if msg.has_active_handedness:
            payload["active_handedness"] = msg.active_handedness
        return payload

    @staticmethod
    def _tool_mask_payload(msg):
        payload = {
            "locked": msg.locked,
            "mask_source": msg.mask_source,
            "tool_roi": list(msg.tool_roi) if msg.has_tool_roi else None,
        }
        if msg.has_reason:
            payload["reason"] = msg.reason
        return payload

    @staticmethod
    def _tool_command_payload(msg):
        return {
            "action": msg.action,
            "tool_name": msg.tool_name,
            "target_mode": msg.target_mode,
            "raw_text": msg.raw_text,
            "match_method": msg.match_method,
            "confidence": msg.confidence,
        }


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
