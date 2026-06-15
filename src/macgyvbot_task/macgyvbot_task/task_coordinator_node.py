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

from macgyvbot_config.drawer import (
    DRAWER_OBSERVATION_J6_DEG,
    ENABLE_DRAWER_COLLISION_SCENE_DEFAULT,
    TOOL_OBSERVE_X_BACKOFF_M,
)
from macgyvbot_config.gripper import ENABLE_GRIPPER_SELF_COLLISION_ACM_DEFAULT
from macgyvbot_config.joint_velocity import MOTION_VELOCITY_SCALING_FACTOR
from macgyvbot_config.structured_logging import (
    format_structured_log,
)
from macgyvbot_config.models import (
    HAND_GRASP_SAM_CHECKPOINT_NAME,
    YOLO_CONFIDENCE_THRESHOLD,
    YOLO_MODEL_NAME,
)
from macgyvbot_config.pick import PICK_GRASP_YAW_OFFSET_DEG
from macgyvbot_config.robot import GROUP_NAME, HOME_JOINTS, WRIST_JOINT_NAME
from macgyvbot_config.timing import (
    CAMERA_LOOP_IDLE_SLEEP_SEC,
    TASK_QUEUE_STOP_POLL_SEC,
    TASK_QUEUE_STOP_TIMEOUT_SEC,
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
    GRASP_POINT_API_MODEL,
    GRASP_POINT_API_TIMEOUT_SEC,
    GRASP_POINT_MODE_CENTER,
    GRASP_POINT_MODE_YOLO,
    GRASP_POINT_MODE_VLM,
    SAM_BACKEND_DEFAULT,
    SAM_DEVICE_DEFAULT,
    SAM_ENABLED_DEFAULT,
    SAM_MODEL_TYPE_DEFAULT,
    SAM_YAW_SERVICE_NAME,
    SAM_YAW_SERVICE_RESPONSE_TIMEOUT_SEC,
    SAM_YAW_SERVICE_WAIT_TIMEOUT_SEC,
    VLM_GRASP_SERVICE_NAME,
    VLM_ONLY_MODES,
    VLM_SERVICE_RESPONSE_TIMEOUT_SEC,
    VLM_SERVICE_WAIT_TIMEOUT_SEC,
)
from macgyvbot_config.ui import DISPLAY_DEBUG_WINDOWS_DEFAULT
from macgyvbot_domain.target_models import PickTarget
from macgyvbot_manipulation.drawer_collision_scene import (
    DrawerCollisionSceneManager,
)
from macgyvbot_manipulation.drawer_motion import DrawerMotionFlow
from macgyvbot_manipulation.gripper_collision_scene import (
    GripperSelfCollisionManager,
)
from macgyvbot_manipulation.moveit_controller import MoveItController
from macgyvbot_manipulation.onrobot_gripper import RG
from macgyvbot_manipulation.robot_pose import (
    get_ee_matrix,
    make_safe_pose,
    normalize_angle_deg,
    orientation_from_joint_positions,
)
from macgyvbot_manipulation.robot_safezone import SAFE_Z_MIN, safe_z_min_for_drawer
from macgyvbot_manipulation.timing import cooperative_wait
from macgyvbot_perception.depth_projection import DepthProjector
from macgyvbot_perception.grasp_point.grasp_point_selector import (
    GraspPointSelector,
    normalize_grasp_point_mode,
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
from macgyvbot_task.application.adapters.sam_yaw_service_client import (
    SAMYawServiceClient,
)
from macgyvbot_task.application.adapters.vlm_grasp_service_client import (
    VLMGraspServiceClient,
)
from macgyvbot_task.application.display.debug_display import DebugDisplay
from macgyvbot_task.application.pick_flow.bring_sequence import BringSequenceRunner
from macgyvbot_task.application.pick_flow.pick_frame_processor import (
    PickFrameProcessor,
)
from macgyvbot_task.application.return_flow.return_perception_adapter import (
    ReturnPerceptionAdapter,
)
from macgyvbot_task.application.pick_flow.pick_sequence import PickSequenceRunner
from macgyvbot_task.application.return_flow.return_sequence import ReturnSequenceRunner
from macgyvbot_task.application.recovery import (
    build_drop_recovery_steps,
)
from macgyvbot_task.application.recovery.recovery_utils import RecoveryConfig
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
        if self._publish_if_vlm_status("info", message):
            self._logger.info(message)
            return
        self._logger.debug(message)

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
            return False

        self._publish_status_payload(
            {
                "status": status,
                "tool_name": "unknown",
                "action": "system",
                "message": self._chat_message(status, text),
            }
        )
        return True

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


class PipelineLogger:
    """Format pipeline logs consistently and optionally quiet routine info."""

    def __init__(self, logger, svc="task", pipe="system", quiet_info=False):
        self._logger = logger
        self._svc = svc
        self._pipe = pipe
        self._quiet_info = bool(quiet_info)

    def bind(self, pipe=None, *, quiet_info=None):
        return PipelineLogger(
            self._logger,
            svc=self._svc,
            pipe=pipe or self._pipe,
            quiet_info=self._quiet_info if quiet_info is None else quiet_info,
        )

    def debug(self, message, **fields):
        self._emit("debug", message, **fields)

    def info(self, message, **fields):
        level = "debug" if self._quiet_info else "info"
        self._emit(level, message, **fields)

    def warn(self, message, **fields):
        self._emit("warn", message, **fields)

    def warning(self, message, **fields):
        self.warn(message, **fields)

    def error(self, message, **fields):
        self._emit("error", message, **fields)

    def _emit(self, level, message, **fields):
        text = _format_pipeline_log(
            svc=fields.pop("svc", self._svc),
            pipe=fields.pop("pipe", self._pipe),
            step=fields.pop("step", "log"),
            event=fields.pop("event", "status"),
            msg=message,
            **fields,
        )
        if level == "debug":
            self._logger.debug(text)
        elif level == "error":
            self._logger.error(text)
        elif level == "warn":
            self._logger.warn(text)
        else:
            self._logger.info(text)


def _format_pipeline_log(*, svc, pipe, step, event, msg="", **fields):
    return format_structured_log(
        svc=svc,
        pipe=pipe,
        step=step,
        event=event,
        msg=msg,
        **fields,
    )


class TaskCoordinatorNode(Node):
    """Own pick/return workflow queues and task-control requests."""

    def __init__(self):
        super().__init__("task_coordinator_node")

        self.declare_parameter(
            "display_debug_windows",
            DISPLAY_DEBUG_WINDOWS_DEFAULT,
        )
        self.declare_parameter("manual_gripper_service", MANUAL_GRIPPER_SERVICE)
        self.declare_parameter(
            "enable_drawer_collision_scene",
            ENABLE_DRAWER_COLLISION_SCENE_DEFAULT,
        )
        self.declare_parameter(
            "enable_gripper_self_collision_acm",
            ENABLE_GRIPPER_SELF_COLLISION_ACM_DEFAULT,
        )
        self.bridge = CvBridge()
        self.display_debug_windows = self._read_bool_parameter(
            "display_debug_windows",
            DISPLAY_DEBUG_WINDOWS_DEFAULT,
        )
        self.enable_drawer_collision_scene = self._read_bool_parameter(
            "enable_drawer_collision_scene",
            ENABLE_DRAWER_COLLISION_SCENE_DEFAULT,
        )
        self.enable_gripper_self_collision_acm = self._read_bool_parameter(
            "enable_gripper_self_collision_acm",
            ENABLE_GRIPPER_SELF_COLLISION_ACM_DEFAULT,
        )
        self.display = DebugDisplay(enabled=self.display_debug_windows)
        self.exit_req = threading.Event()
        self.pause_req = threading.Event()
        self.drop_req = threading.Event()
        self.resume_req = threading.Event()
        self.handoff_retry_req = threading.Event()
        self.handoff_fallback_req = threading.Event()
        self.handoff_decision_pending = threading.Event()

        self._queue = deque()
        self._queue_lock = threading.RLock()
        self._step_thread = None
        self._current_step = None
        self._current_task_name = None
        self._suspended_step = None
        self._suspended_task_name = None
        self._exit_home_thread = None
        self._drop_recovery_thread = None
        self._pending_drop_recovery_payload = None
        self._active_drop_recovery_snapshot = None
        self._drop_recovery_resume_step = None
        self._drop_recovery_resume_task_name = None
        self._drop_recovery_resume_queue = None
        self._vlm_preload_timer = None
        self._manual_gripper_lock = threading.Lock()

        self.grasp_point_mode = self._read_grasp_point_mode()
        self.yolo_model = self._read_yolo_model()
        self.yolo_conf = self._read_yolo_conf()
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
            logger_provider=lambda: self._task_log("task", quiet_info=True),
            publish_robot_status=self._publish_robot_status,
            publish_status_payload=self._publish_status_payload,
        )
        self.detector = YoloDetector(
            self.yolo_model,
            confidence_threshold=self.yolo_conf,
        )
        self.grasp_point_logger = VLMStatusLogger(
            self._task_log("vlm"),
            self._publish_status_payload,
        )
        self.grasp_point_selector = GraspPointSelector(
            self.grasp_point_mode,
            self.grasp_point_logger,
            **self._read_grasp_point_api_config(),
            **self._read_vlm_sam_config(),
        )
        self.vlm_service_client = VLMGraspServiceClient(
            self,
            self.bridge,
            **self._read_vlm_service_config(),
        )
        self.sam_yaw_service_client = SAMYawServiceClient(
            self,
            self.bridge,
            **self._read_sam_yaw_service_config(),
        )
        if self.grasp_point_mode not in (GRASP_POINT_MODE_VLM, *VLM_ONLY_MODES):
            self.grasp_point_selector.preload_vlm_if_needed()

        calib_file = resolve_calibration_file("T_gripper2camera.npy")
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0

        self.gripper = RG("rg2", "192.168.1.1", 502)
        self.robot = MoveItPy(node_name="task_coordinator_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.gripper_self_collision_scene = None
        if self.enable_gripper_self_collision_acm:
            self.gripper_self_collision_scene = GripperSelfCollisionManager(
                self,
                moveit_robot=self.robot,
            )
        self.drawer_collision_scene = DrawerCollisionSceneManager(
            self,
            moveit_robot=self.robot,
        )
        if self.enable_drawer_collision_scene:
            self._apply_drawer_collision_scene()
        self.depth_projector = DepthProjector(self._base_to_camera_matrix)
        self.hand_grasp_adapter = HandGraspResultAdapter(
            self.state,
            self.depth_projector,
            self._task_log("adapter", quiet_info=True),
        )
        self.pick_target_resolver = PickTargetResolver(
            self.detector,
            self.grasp_point_selector,
            self.depth_projector,
            self._task_log("perception", quiet_info=True),
        )

        self.planning_params = PlanRequestParameters(self.robot)
        self.planning_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.planning_params.planner_id = "PTP"
        self.planning_params.max_velocity_scaling_factor = (
            MOTION_VELOCITY_SCALING_FACTOR
        )

        self.motion = MoveItController(
            self.robot,
            self.arm,
            self.planning_params,
            should_interrupt=self._motion_interrupted,
            node=self,
            drawer_collision_scene=self.drawer_collision_scene,
            gripper_self_collision_scene=self.gripper_self_collision_scene,
            enable_drawer_collision_scene=self.enable_drawer_collision_scene,
            enable_gripper_self_collision_acm=(
                self.enable_gripper_self_collision_acm
            ),
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
            "drop": self.drop_req,
            "resume": self.resume_req,
            "handoff_retry": self.handoff_retry_req,
            "handoff_fallback": self.handoff_fallback_req,
            "handoff_decision_pending": self.handoff_decision_pending,
        }
        self.frame_processor = PickFrameProcessor(
            self.state,
            self.detector,
            self.pick_target_resolver,
            self.start_pick_sequence,
            self._publish_robot_status,
            self._task_log("pick", quiet_info=True),
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
            self._task_log("return", quiet_info=True),
            wait_fn=cooperative_wait,
            refine_store_tool_target=self._target_with_mask_pca_result,
        )
        self.pick_runner = PickSequenceRunner(
            self.robot,
            self.motion,
            self.gripper,
            self.state,
            self.tool_hold_monitor,
            refine_pick_target=self._refine_pick_target_after_centering,
            estimate_grasp_yaw=self._mask_pca_yaw_for_target,
            generate_grasp_detection_mask_images=(
                self._generate_grasp_detection_mask_images_after_vlm_observe
            ),
            control_events=control_events,
            drawer_flow=self.drawer_flow,
        )
        self.bring_runner = BringSequenceRunner(
            state=self.state,
            drawer_flow=self.drawer_flow,
            return_perception=self.return_perception,
            frame_processor=self.frame_processor,
            detector=self.detector,
            pick_target_resolver=self.pick_target_resolver,
            pick_runner=self.pick_runner,
            publish_robot_status=self._publish_robot_status,
            task_log=self._task_log,
            interrupted=self._motion_interrupted,
            append_task_steps=self._append_task_steps,
            has_queued_task_steps=self._has_queued_task_steps,
            recover_after_drawer_validation_failure=(
                self._recover_after_pick_drawer_validation_failure
            ),
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
            detect_drawer_tool_labels=(
                self.return_perception.detect_drawer_tool_labels
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

    def _apply_drawer_collision_scene(self):
        return self.drawer_collision_scene.apply(self.get_logger())

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
        return normalize_grasp_point_mode(grasp_point_mode, self._task_log("config"))

    def _read_yolo_model(self):
        self.declare_parameter("yolo_model", YOLO_MODEL_NAME)
        return (
            self.get_parameter("yolo_model")
            .get_parameter_value()
            .string_value
            .strip()
        ) or YOLO_MODEL_NAME

    def _read_yolo_conf(self):
        self.declare_parameter("yolo_conf", YOLO_CONFIDENCE_THRESHOLD)
        return float(self.get_parameter("yolo_conf").value)

    def _read_grasp_point_api_config(self):
        self.declare_parameter("grasp_point_api_model", GRASP_POINT_API_MODEL)
        self.declare_parameter("grasp_point_api_env_file", "")
        self.declare_parameter("grasp_point_api_base_url", "")
        self.declare_parameter(
            "grasp_point_api_timeout_sec",
            GRASP_POINT_API_TIMEOUT_SEC,
        )
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
        self.declare_parameter("sam_enabled", SAM_ENABLED_DEFAULT)
        self.declare_parameter(
            "sam_checkpoint",
            str(Path("weights") / HAND_GRASP_SAM_CHECKPOINT_NAME),
        )
        self.declare_parameter("sam_backend", SAM_BACKEND_DEFAULT)
        self.declare_parameter("sam_model_type", SAM_MODEL_TYPE_DEFAULT)
        self.declare_parameter("sam_device", SAM_DEVICE_DEFAULT)
        return {
            "sam_enabled": self._read_bool_parameter("sam_enabled", True),
            "sam_checkpoint": str(self.get_parameter("sam_checkpoint").value).strip(),
            "sam_backend": str(self.get_parameter("sam_backend").value).strip(),
            "sam_model_type": str(self.get_parameter("sam_model_type").value).strip(),
            "sam_device": str(self.get_parameter("sam_device").value).strip(),
        }

    def _read_vlm_service_config(self):
        self.declare_parameter("vlm_service_name", VLM_GRASP_SERVICE_NAME)
        self.declare_parameter(
            "vlm_service_wait_timeout_sec",
            VLM_SERVICE_WAIT_TIMEOUT_SEC,
        )
        self.declare_parameter(
            "vlm_service_response_timeout_sec",
            VLM_SERVICE_RESPONSE_TIMEOUT_SEC,
        )
        return {
            "service_name": str(self.get_parameter("vlm_service_name").value).strip(),
            "wait_timeout_sec": float(
                self.get_parameter("vlm_service_wait_timeout_sec").value
            ),
            "response_timeout_sec": float(
                self.get_parameter("vlm_service_response_timeout_sec").value
            ),
        }

    def _read_sam_yaw_service_config(self):
        self.declare_parameter("sam_yaw_service_name", SAM_YAW_SERVICE_NAME)
        self.declare_parameter(
            "sam_yaw_service_wait_timeout_sec",
            SAM_YAW_SERVICE_WAIT_TIMEOUT_SEC,
        )
        self.declare_parameter(
            "sam_yaw_service_response_timeout_sec",
            SAM_YAW_SERVICE_RESPONSE_TIMEOUT_SEC,
        )
        return {
            "service_name": str(
                self.get_parameter("sam_yaw_service_name").value
            ).strip(),
            "wait_timeout_sec": float(
                self.get_parameter("sam_yaw_service_wait_timeout_sec").value
            ),
            "response_timeout_sec": float(
                self.get_parameter("sam_yaw_service_response_timeout_sec").value
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
        log = self._task_log("startup")
        log.info("task coordinator initialized", step="node", event="done")
        log.info("topic ready", step="topic", event="status", name="task_request", topic=TASK_REQUEST_TOPIC)
        log.info("topic ready", step="topic", event="status", name="task_control", topic=TASK_CONTROL_TOPIC)
        log.info("topic ready", step="topic", event="status", name="robot_status", topic=ROBOT_STATUS_TOPIC)
        log.info(
            "manual gripper service ready",
            step="service",
            event="status",
            name="manual_gripper",
            service=self.manual_gripper_service_name,
        )
        log.info(
            "YOLO model ready",
            step="model",
            event="status",
            name="yolo",
            weight=Path(getattr(self.detector, "model_path", self.yolo_model)).name,
            conf=self.yolo_conf,
        )
        log.info("grasp point mode ready", step="grasp_point", event="status", mode=self.grasp_point_mode)

    def _task_log(self, pipe="system", *, quiet_info=False):
        return PipelineLogger(
            self.get_logger(),
            svc="task",
            pipe=pipe,
            quiet_info=quiet_info,
        )

    def _motion_log(self, *, quiet_info=True):
        return PipelineLogger(
            self.get_logger(),
            svc="manipulation",
            pipe="moveit",
            quiet_info=quiet_info,
        )

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
                self._task_log("gripper").error(
                    "manual gripper command failed",
                    step="manual_gripper",
                    event="fail",
                    source=source,
                    width_mm=f"{requested_width_mm:.1f}",
                    reason=f"{type(exc).__name__}: {exc}",
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
        self._task_log("gripper").info(
            "manual gripper command accepted",
            step="manual_gripper",
            event="accepted",
            source=source,
            width_mm=f"{requested_width_mm:.1f}",
            raw=width_raw,
        )
        return response

    def _manual_gripper_is_allowed(self):
        with self._queue_lock:
            active_step = self._current_step is not None or self._step_thread_alive()
            queued = bool(self._queue)
            paused = self.pause_req.is_set() or self.drop_req.is_set()

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
            self._task_log("gripper").warn(
                "manual gripper status read failed",
                step="manual_gripper",
                event="fail",
                reason=f"{type(exc).__name__}: {exc}",
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
        self._task_log("gripper").warn(
            "manual gripper command rejected",
            step="manual_gripper",
            event="rejected",
            status=status,
            source=source,
            width_mm=f"{requested_width_mm:.1f}",
            reason=message,
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

        self._task_log("request").warn(
            "unsupported task request",
            step="task_request",
            event="rejected",
            task=task or "unknown",
            reason="unsupported_task_request",
        )
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
            self.state.target_tool = tool_name
            self.state.current_command = command
            self.start_pick_sequence(
                request.bx,
                request.by,
                request.bz,
                request.vlm_yaw_deg if request.has_vlm_yaw_deg else None,
            )
            return

        self.state.current_command = command
        self.state.target_tool = tool_name
        self.state.target_label = tool_name
        self.state.picking = True
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
        if not self._load_queue("bring", self.bring_runner.build_steps(tool_name)):
            self._clear_task_state()

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

        self._task_log("gripper", quiet_info=True).info(
            "release task request received",
            step="release",
            event="received",
        )
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
        if self.handoff_decision_pending.is_set():
            self._handle_handoff_fallback("home_requested_during_handoff_inspection")
            return

        self._run_home_request(command, tool_name)

    def _handle_home_control(self, reason):
        command = dict(self.state.current_command or {})
        command.setdefault("action", "home")
        command.setdefault("raw_text", reason or "home")
        tool_name = command.get("tool_name", "unknown")
        self._run_home_request(command, tool_name)

    def _run_home_request(self, command, tool_name):
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

        if not self.motion.ensure_gripper_self_collision_acm(
            self.get_logger(),
            attempts=3,
            retry_delay_sec=0.3,
        ):
            self._publish_robot_status(
                "failed",
                tool_name=tool_name,
                action="home",
                message="그리퍼 self-collision 허용 설정 적용에 실패했습니다.",
                reason="gripper_self_collision_acm_failed",
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
        if self.state.recovery_mode and action not in {
            "pause",
            "resume",
            "cancel",
            "exit",
        }:
            self._task_log("control").warn(
                "task control ignored during drop recovery",
                step="task_control",
                event="rejected",
                action=action,
                reason="drop_recovery_active",
            )
            self._publish_robot_status(
                "busy",
                action=action,
                message="drop recovery가 진행 중입니다. 복구가 끝나면 작업을 자동으로 재개합니다.",
                reason="drop_recovery_active",
                command=self.state.current_command,
            )
            return
        if action == "pause":
            self._handle_pause(reason)
            return
        if action == "resume":
            self._handle_resume(reason)
            return
        if action == "retry":
            self._handle_retry(reason)
            return
        if action == "cancel":
            self._handle_cancel(reason)
            return
        if action == "home":
            self._handle_home_control(reason)
            return
        if action == "exit":
            self._handle_exit(reason)
            return
        self._task_log("control").warn(
            "unsupported task control action",
            step="task_control",
            event="rejected",
            action=action,
            reason="unsupported_task_control",
        )

    def _handle_retry(self, reason):
        if not self.handoff_decision_pending.is_set():
            self.get_logger().warn(
                f"재시도 요청을 처리할 handoff decision 상태가 아닙니다: reason={reason}"
            )
            self._publish_robot_status(
                "rejected",
                action="retry",
                message="현재 재시도할 hand inspection 작업이 없습니다.",
                reason="retry_not_available",
                command=self.state.current_command,
            )
            return False

        self.get_logger().info(f"handoff inspection retry 요청: reason={reason}")
        self.handoff_fallback_req.clear()
        self.handoff_retry_req.set()
        self._publish_robot_status(
            "resumed",
            action="retry",
            message="사용자 손 인식을 다시 시도합니다.",
            reason=reason or "handoff_retry_requested",
            command=self.state.current_command,
        )
        return True

    def _handle_handoff_fallback(self, reason):
        if not self.handoff_decision_pending.is_set():
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
        self._task_log("control").warn(
            "task queue pause requested",
            step="task_control",
            event="pause",
            reason=reason or "pause_requested",
        )
        self.pause_req.set()
        self.resume_req.clear()
        with self._queue_lock:
            if self._current_step is not None:
                self._suspended_step = self._current_step
                self._suspended_task_name = self._current_task_name
        self.motion.cancel_current_goal(self._motion_log(), reason=reason or "pause")
        self._publish_robot_status(
            "paused",
            message="사용자 요청으로 작업을 일시정지합니다.",
            reason=reason or "pause_requested",
            command=self.state.current_command,
        )
        return True

    def _handle_resume(self, reason):
        self._task_log("control").info(
            "task queue resume requested",
            step="task_control",
            event="resume",
            reason=reason or "resume_requested",
        )
        if self.state.recovery_mode:
            should_dispatch = False
            with self._queue_lock:
                self.resume_req.set()
                if self._current_step is None and not self._step_thread_alive():
                    should_dispatch = self._resume_suspended_step_locked()
            self._publish_robot_status(
                "resumed",
                message="drop recovery를 재개합니다.",
                reason=reason or "resume_requested",
                command=self.state.current_command,
            )
            if should_dispatch:
                self._dispatch_next()
            return True

        should_dispatch = False
        resume_available = False
        with self._queue_lock:
            if (
                not self.pause_req.is_set()
                and not self.drop_req.is_set()
                and self._suspended_step is None
            ):
                self.resume_req.clear()
            else:
                resume_available = True
                self.resume_req.set()
                if self._current_step is None and not self._step_thread_alive():
                    should_dispatch = self._resume_suspended_step_locked()

        if not resume_available:
            self._task_log("control").info(
                "task queue resume ignored because no paused task exists",
                step="task_control",
                event="resume",
                reason=reason or "resume_without_paused_task",
            )
            self._publish_robot_status(
                "rejected",
                action="resume",
                message="재개할 작업이 없습니다. 다음 명령을 기다리겠습니다.",
                reason="resume_without_paused_task",
                command=self.state.current_command,
            )
            return False

        self._publish_robot_status(
            "resumed",
            message="작업을 재개합니다.",
            reason=reason or "resume_requested",
            command=self.state.current_command,
        )
        if should_dispatch:
            self._dispatch_next()
        return True

    def _handle_cancel(self, reason, publish_status=True):
        if self.handoff_decision_pending.is_set():
            return self._handle_handoff_fallback(
                reason or "cancel_during_handoff_inspection"
            )

        self._task_log("control").warn(
            "task queue cancel requested",
            step="task_control",
            event="cancel",
            reason=reason or "cancel_requested",
        )
        self.exit_req.set()
        self.pause_req.clear()
        self.drop_req.clear()
        self.resume_req.clear()
        self.motion.cancel_current_goal(self._motion_log(), reason=reason or "cancel")
        with self._queue_lock:
            active_step = self._current_step is not None or self._step_thread_alive()
            self._queue.clear()
            self._clear_suspended_task_state_locked()
            self._clear_drop_recovery_state_locked()

        if not active_step:
            self._run_cleanup_callbacks()
            self.exit_req.clear()
            self._clear_task_state()

        if publish_status:
            self._publish_robot_status(
                "cancelled",
                action="cancel",
                message="작업을 취소했습니다. 다음 명령을 기다립니다.",
                reason=reason or "cancel_requested",
                command=self.state.current_command,
            )
        return True

    def _handle_exit(self, reason, publish_status=True):
        self._task_log("control").warn(
            "task queue exit requested",
            step="task_control",
            event="exit",
            reason=reason or "exit_requested",
        )
        self.exit_req.set()
        self.pause_req.clear()
        self.drop_req.clear()
        self.resume_req.clear()
        self.motion.cancel_current_goal(self._motion_log(), reason=reason or "exit")
        with self._queue_lock:
            self._queue.clear()
            self._clear_suspended_task_state_locked()
            self._clear_drop_recovery_state_locked()

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
        log = self._task_log("exit")
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
        log.info(
            "moving home after exit request",
            step="exit_home",
            event="start",
            reason=reason or "exit_requested",
        )
        self._publish_robot_status(
            "returning_home",
            action="exit",
            message="종료 요청 후 Home 위치로 복귀합니다.",
            reason=reason or "exit_requested",
            command=command,
        )

        if not self.motion.ensure_gripper_self_collision_acm(
            log,
            attempts=3,
            retry_delay_sec=0.3,
        ):
            self._publish_robot_status(
                "failed",
                action="exit",
                message="그리퍼 self-collision 허용 설정 적용에 실패했습니다.",
                reason="gripper_self_collision_acm_failed",
                command=command,
            )
            return

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
            log.warn(
                "gripper release failed after exit home",
                step="exit_home",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )
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

    def _wait_for_task_queue_to_stop(
        self,
        logger,
        timeout_sec=TASK_QUEUE_STOP_TIMEOUT_SEC,
    ):
        deadline = time.monotonic() + timeout_sec
        while self.is_running() and time.monotonic() < deadline:
            time.sleep(TASK_QUEUE_STOP_POLL_SEC)
        if self.is_running():
            logger.warn(
                "task queue shutdown wait timed out",
                step="queue",
                event="timeout",
                reason="exit_queue_shutdown_timeout",
                timeout_sec=timeout_sec,
            )
            return False
        return True

    def _tool_drop_cb(self, msg):
        payload = self._tool_drop_payload(msg)
        if payload.get("event") != "tool_dropped":
            return
        reason = payload.get("reason") or "tool_dropped"
        if self.state.recovery_mode:
            self._task_log("safety").warn(
                "tool drop queued while recovery is active",
                step="tool_drop",
                event="queued",
                reason=reason,
                tool=payload.get("tool_name", "unknown"),
            )
            self._pending_drop_recovery_payload = payload
            self.drop_req.set()
            self.resume_req.clear()
            self.motion.cancel_current_goal(
                self._motion_log(),
                reason=f"{reason}_during_recovery",
            )
            return
        self._task_log("safety").warn(
            "tool drop recovery requested",
            step="tool_drop",
            event="detected",
            reason=reason,
            tool=payload.get("tool_name", "unknown"),
        )
        self._start_drop_recovery(payload)

    def _start_drop_recovery(self, payload):
        if (
            self._drop_recovery_thread is not None
            and self._drop_recovery_thread.is_alive()
        ):
            self._pending_drop_recovery_payload = payload
            return

        snapshot = self._drop_recovery_snapshot(payload)
        self.drop_req.set()
        self.resume_req.clear()
        self.motion.cancel_current_goal(
            self._motion_log(),
            reason=snapshot["reason"],
        )
        with self._queue_lock:
            if self._drop_recovery_resume_queue is None:
                self._drop_recovery_resume_queue = list(self._queue)
            self._queue.clear()
            if self._current_step is not None:
                self._drop_recovery_resume_step = self._current_step
                self._drop_recovery_resume_task_name = self._current_task_name

        self._drop_recovery_thread = threading.Thread(
            target=self._load_drop_recovery_queue,
            args=(snapshot,),
            name="task_drop_recovery",
            daemon=True,
        )
        self._drop_recovery_thread.start()

    def _drop_recovery_snapshot(self, payload):
        command = payload.get("command") or self.state.current_command or {}
        task_name = self._current_task_name or str(payload.get("action") or "")
        action = str(payload.get("action") or command.get("action") or task_name)
        tool_name = (
            payload.get("tool_name")
            or self.state.held_tool
            or self.state.target_tool
            or self.state.target_label
            or command.get("tool_name")
            or "unknown"
        )
        return {
            "task_name": task_name,
            "action": action,
            "tool_name": tool_name,
            "command": command,
            "reason": payload.get("reason") or "tool_dropped",
            "resume_state": {
                "picking": self.state.picking,
                "target_label": self.state.target_label,
                "target_tool": self.state.target_tool,
                "current_command": self.state.current_command,
            },
        }

    def _load_drop_recovery_queue(self, snapshot):
        log = self._task_log("recovery")
        if not self._wait_for_active_task_step_to_stop(log):
            self._publish_robot_status(
                "failed",
                action=snapshot["action"],
                tool_name=snapshot["tool_name"],
                message="drop recovery를 시작하기 전에 기존 작업 큐가 멈추지 않았습니다.",
                reason="drop_recovery_queue_shutdown_timeout",
                command=snapshot["command"],
            )
            self._active_drop_recovery_snapshot = None
            self._drop_recovery_thread = None
            return

        self.drop_req.clear()
        self.resume_req.clear()
        self.state.picking = True
        self.state.current_command = snapshot["command"]
        self.state.target_tool = snapshot["tool_name"]
        if snapshot["action"] in ("bring", "pick"):
            self.state.target_label = snapshot["tool_name"]
        elif snapshot["action"] == "return" and not self.state.held_tool:
            self.state.held_tool = snapshot["tool_name"]

        recovery_config = self._build_recovery_config(snapshot)
        is_pick_drop_recovery = (
            snapshot["action"] in ("bring", "pick")
            or snapshot["task_name"] == "pick"
        )
        steps = build_drop_recovery_steps(
            self.state,
            self.motion,
            self.gripper,
            recovery_config,
            log,
            task_type="pick" if is_pick_drop_recovery else "return",
        )

        with self._queue_lock:
            self._active_drop_recovery_snapshot = snapshot
            self._queue.extend(("recovery", step) for step in steps)
            self._drop_recovery_thread = None
        self._dispatch_next()

    def _finish_drop_recovery_queue_locked(self, ok):
        snapshot = self._active_drop_recovery_snapshot
        if snapshot is None:
            return False

        is_pick_drop_recovery = (
            snapshot["action"] in ("bring", "pick")
            or snapshot["task_name"] == "pick"
        )
        action = "bring" if is_pick_drop_recovery else "return"

        if self._pending_drop_recovery_payload is not None and not ok:
            self._publish_robot_status(
                "recovering",
                tool_name=snapshot["tool_name"],
                action=action,
                message="drop recovery 중 다시 drop이 감지되어 새 recovery를 시작합니다.",
                reason="drop_recovery_restart_requested",
                command=snapshot["command"],
            )
            self._restore_drop_recovery_resume_state(snapshot)
            self._active_drop_recovery_snapshot = None
            self.drop_req.set()
            self.resume_req.clear()
            self._restart_pending_drop_recovery()
            return False

        self._publish_robot_status(
            "returned" if ok else "failed",
            tool_name=snapshot["tool_name"],
            action=action,
            message=(
                "drop recovery가 완료되었습니다."
                if ok
                else "drop recovery가 실패했습니다. 로봇은 가능한 안전 정리 절차를 수행했습니다."
            ),
            reason="drop_recovery_succeeded" if ok else "drop_recovery_failed",
            command=snapshot["command"],
        )
        self._restore_drop_recovery_resume_state(snapshot)
        pending_drop_recovery = self._pending_drop_recovery_payload is not None
        self._active_drop_recovery_snapshot = None
        if ok and not pending_drop_recovery:
            if self._drop_recovery_resume_step is not None:
                self._queue.append(
                    (
                        self._drop_recovery_resume_task_name,
                        self._drop_recovery_resume_step,
                    )
                )
                self._drop_recovery_resume_step = None
                self._drop_recovery_resume_task_name = None
            if self._drop_recovery_resume_queue is not None:
                self._queue.extend(self._drop_recovery_resume_queue)
                self._drop_recovery_resume_queue = None
            should_dispatch = self._resume_suspended_step_locked()
            self._publish_robot_status(
                "resumed",
                tool_name=snapshot["tool_name"],
                action=action,
                message="drop recovery가 완료되어 중단된 작업을 자동으로 다시 실행합니다.",
                reason="drop_recovery_auto_resumed",
                command=snapshot["command"],
            )
            return should_dispatch
        elif not ok and not pending_drop_recovery:
            self._clear_failed_drop_recovery_queue()
            self.pause_req.clear()
            self.drop_req.clear()
            self.resume_req.clear()
            self._run_cleanup_callbacks()
            self._clear_task_state()
        else:
            self.drop_req.set()
            self.resume_req.clear()
        self._restart_pending_drop_recovery()
        return False

    def _wait_for_active_task_step_to_stop(
        self,
        logger,
        timeout_sec=TASK_QUEUE_STOP_TIMEOUT_SEC,
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._queue_lock:
                active = self._current_step is not None or self._step_thread_alive()
            if not active:
                return True
            time.sleep(TASK_QUEUE_STOP_POLL_SEC)
        logger.warn(
            "active task step shutdown wait timed out",
            step="queue",
            event="timeout",
            reason="drop_recovery_active_step_shutdown_timeout",
            timeout_sec=timeout_sec,
        )
        return False

    def _restore_drop_recovery_resume_state(self, snapshot):
        resume_state = snapshot.get("resume_state") or {}
        self.state.picking = bool(resume_state.get("picking", self.state.picking))
        self.state.target_label = resume_state.get("target_label")
        self.state.target_tool = resume_state.get("target_tool")
        self.state.current_command = resume_state.get("current_command")
        self.state.recovery_mode = False

    def _restart_pending_drop_recovery(self):
        payload = self._pending_drop_recovery_payload
        self._pending_drop_recovery_payload = None
        if payload is None:
            return False

        self._task_log("recovery").warn(
            "restarting drop recovery from queued drop event",
            step="tool_drop",
            event="restart",
            reason=payload.get("reason") or "tool_dropped",
            tool=payload.get("tool_name", "unknown"),
        )
        self._drop_recovery_thread = None
        self._start_drop_recovery(payload)
        return True

    def _clear_failed_drop_recovery_queue(self):
        with self._queue_lock:
            self._queue.clear()
            self._clear_suspended_task_state_locked()
            self._clear_drop_recovery_state_locked()
            self._current_step = None
            self._current_task_name = None
            self._step_thread = None

    def _clear_suspended_task_state_locked(self):
        self._suspended_step = None
        self._suspended_task_name = None

    def _clear_drop_recovery_state_locked(self):
        self._pending_drop_recovery_payload = None
        self._active_drop_recovery_snapshot = None
        self._drop_recovery_resume_step = None
        self._drop_recovery_resume_task_name = None
        self._drop_recovery_resume_queue = None

    def _build_recovery_config(self, snapshot):
        command = snapshot["command"]
        return RecoveryConfig(
            robot=self.robot,
            state=self.state,
            command=command,
            task_type=snapshot["action"],
            tool_hold_monitor=self.tool_hold_monitor,
            initial_detect_target_fn=lambda target_tool: self._resolve_recovery_initial_target(
                target_tool,
                allow_supported_fallback=not (
                    snapshot["action"] in ("bring", "pick")
                    or snapshot["task_name"] == "pick"
                ),
            ),
            target_observe_fn=self._move_to_drop_recovery_target_observe_pose,
            observed_tool_label_fn=self._detect_recovery_observed_tool_label,
            close_open_drawer_fn=self._close_open_drawer_for_recovery_failure,
            detect_target_fn=lambda target_tool: self._resolve_recovery_target(
                target_tool,
                apply_pca_yaw=True,
            ),
            pause_event=self.pause_req,
            resume_event=self.resume_req,
            drop_event=self.drop_req,
            exit_event=self.exit_req,
        )

    def _close_open_drawer_for_recovery_failure(self, logger):
        drawer_id = getattr(self.state, "opened_drawer_id", None)
        if not getattr(self.state, "drawer_open", False) or drawer_id is None:
            return True

        if not self.drawer_flow.close_drawer(drawer_id, logger):
            return False

        self.state.drawer_open = False
        self.state.opened_drawer_id = None
        return True

    def _resolve_recovery_initial_target(
        self,
        target_tool,
        allow_supported_fallback=False,
    ):
        target = self._resolve_recovery_target(target_tool, apply_pca_yaw=False)
        if target is not None and target.found:
            return target
        if not allow_supported_fallback:
            return target

        observed_tool = self._detect_recovery_observed_tool_label()
        if not observed_tool or observed_tool == "unknown":
            return target
        return self._resolve_recovery_target(observed_tool, apply_pca_yaw=False)

    def _resolve_recovery_target(self, target_tool, apply_pca_yaw=False):
        if not self.frame_processor.has_camera_state():
            self._task_log("recovery").warn(
                "recovery target camera state unavailable",
                step="recovery_detect",
                event="fail",
                target=target_tool,
                reason="camera_state_unavailable",
            )
            return None

        color_image = self.state.color_image.copy()
        depth_image = self.state.depth_image.copy()
        intrinsics = dict(self.state.intrinsics)
        results = self.detector.detect(color_image)
        boxes = results[0].boxes if results else None
        target = self._resolve_recovery_grasp_point_target(
            boxes,
            target_tool,
            color_image,
            depth_image,
            intrinsics,
        )
        if apply_pca_yaw:
            target = self._target_with_mask_pca_result(target, target_tool)
            self._set_recovery_tool_mask_lock_state(target, target_tool)
            return target
        return target

    def _resolve_recovery_grasp_point_target(
        self,
        boxes,
        target_tool,
        color_image,
        depth_image,
        intrinsics,
    ):
        if self.grasp_point_mode == GRASP_POINT_MODE_YOLO:
            target = self._resolve_recovery_yolo_grasp_point_target(
                boxes,
                target_tool,
                depth_image,
                intrinsics,
            )
            if target is not None and target.found:
                return target

            reason = (
                getattr(target, "reason", "unknown")
                if target is not None
                else "unknown"
            )
            self._task_log("recovery").warn(
                "recovery YOLO grasp point unavailable; using bbox center",
                step="recovery_detect",
                event="fallback",
                target=target_tool,
                reason=reason,
            )
            return self._resolve_recovery_center_target(
                boxes,
                target_tool,
                color_image,
                depth_image,
                intrinsics,
            )

        return self.pick_target_resolver.target_from_boxes(
            boxes,
            target_tool,
            color_image,
            depth_image,
            intrinsics,
            use_bbox_center=False,
        )

    def _resolve_recovery_yolo_grasp_point_target(
        self,
        boxes,
        target_tool,
        depth_image,
        intrinsics,
    ):
        matched_box = self.pick_target_resolver.matching_box(boxes, target_tool)
        if matched_box is None:
            return PickTarget(
                found=False,
                label=target_tool,
                pixel=None,
                base_xyz=None,
                depth_m=None,
                yaw_deg=None,
                reason="target_not_found",
            )

        box, label = matched_box
        selected = self.grasp_point_selector.select_yolo_grasp_point(
            boxes,
            self.detector.names,
            box,
        )
        if selected is None:
            return PickTarget(
                found=False,
                label=target_tool,
                pixel=None,
                base_xyz=None,
                depth_m=None,
                yaw_deg=None,
                reason="grasp_selection_failed",
            )

        return self.pick_target_resolver.target_from_selected_grasp(
            label,
            target_tool,
            selected,
            depth_image,
            intrinsics,
        )

    def _resolve_recovery_center_target(
        self,
        boxes,
        target_tool,
        color_image,
        depth_image,
        intrinsics,
    ):
        return self.pick_target_resolver.target_from_boxes(
            boxes,
            target_tool,
            color_image,
            depth_image,
            intrinsics,
            use_bbox_center=True,
        )

    def _set_recovery_tool_mask_lock_state(self, target, target_tool):
        if target is None or not getattr(target, "found", False):
            self.state.tool_mask_locked = False
            self.state.last_tool_mask_lock_result = None
            return

        yaw_target = getattr(self.state, "grasp_detection_yaw_target", None)
        yaw_deg = getattr(self.state, "grasp_detection_yaw_deg", None)
        if yaw_target != target_tool or yaw_deg is None:
            self.state.tool_mask_locked = False
            self.state.last_tool_mask_lock_result = None
            return

        tool_roi = None
        pixel = getattr(target, "pixel", None)
        if pixel is not None:
            tool_roi = {
                "center_u": int(pixel[0]),
                "center_v": int(pixel[1]),
            }

        self.state.tool_mask_locked = True
        self.state.last_tool_mask_lock_result = {
            "locked": True,
            "mask_source": "SAM_DEPTH_RECOVERY",
            "tool_roi": tool_roi,
            "reason": "drop_recovery_sam_depth_locked",
            "target_tool": target_tool,
        }

    def _move_to_drop_recovery_target_observe_pose(self, detection, target_tool, logger):
        base_xyz = getattr(detection, "base_xyz", None)
        if base_xyz is None:
            return False

        plan = self.pick_runner.target_planner.plan(
            *base_xyz,
            logger,
            safe_z_min=SAFE_Z_MIN,
        )
        observe_x = plan.target_x - TOOL_OBSERVE_X_BACKOFF_M
        observe_y = plan.target_y
        observe_z = plan.drawer_wall_clearance_z
        logger.info(
            "pick recovery target observe pose",
            step="recovery_observe",
            event="start",
            target=target_tool,
            observe_x=f"{observe_x:.3f}",
            observe_y=f"{observe_y:.3f}",
            observe_z=f"{observe_z:.3f}",
        )
        return self.motion.plan_and_execute(
            logger,
            pose_goal=make_safe_pose(
                observe_x,
                observe_y,
                observe_z,
                self.state.home_ori,
                logger,
            ),
            collision_scene_key="recovery/pick_observe_offset",
        )

    def _detect_recovery_observed_tool_label(self):
        if not self.frame_processor.has_camera_state():
            return None

        results = self.detector.detect(self.state.color_image.copy())
        boxes = results[0].boxes if results else None
        if boxes is None:
            return None

        best_label = None
        best_conf = -1.0
        confidence_threshold = float(
            getattr(self.detector, "confidence_threshold", 0.0)
        )
        for box in boxes:
            label = self._box_label(box)
            conf = self._box_confidence(box)
            if conf < confidence_threshold:
                continue
            if conf > best_conf:
                best_conf = conf
                best_label = label
        return best_label

    def _box_label(self, box):
        try:
            class_id = int(box.cls[0])
        except (TypeError, IndexError):
            class_id = int(box.cls)
        return str(self.detector.names[class_id])

    @staticmethod
    def _box_confidence(box):
        try:
            return float(box.conf[0])
        except (AttributeError, TypeError, IndexError):
            return 0.0

    def start_pick_sequence(self, bx, by, bz, vlm_yaw_deg=None):
        if self.is_running() or self.state.picking:
            self._task_log("pick").warn(
                "pick request ignored while task is running",
                step="queue",
                event="skipped",
                reason="task_queue_busy",
            )
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
        self.state.target_tool = self.state.target_label
        target_label = self.state.target_label
        steps = self.pick_runner.build_steps(bx, by, bz, vlm_yaw_deg)
        self._publish_robot_status(
            "picking",
            tool_name=target_label,
            action="bring",
            message=f"{target_label} pick 동작을 시작합니다.",
            command=self.state.current_command,
        )
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

        self.state.picking = True
        self.state.target_label = None
        self.state.target_tool = tool_name
        self.state.current_command = command
        steps = self.return_runner.build_steps(command)
        return self._load_queue("return", steps)

    def _load_queue(self, task_name, steps):
        with self._queue_lock:
            if self._current_step is not None or self._step_thread_alive():
                self._task_log(task_name).warn(
                    "task request ignored while queue is running",
                    step="queue",
                    event="skipped",
                    reason="task_queue_busy",
                )
                return False
            self.exit_req.clear()
            self.drop_req.clear()
            self.resume_req.clear()
            self._queue.clear()
            self._queue.extend((task_name, step) for step in steps)
            self._task_log(task_name, quiet_info=True).info(
                "task queue loaded",
                step="queue",
                event="loaded",
                step_count=len(steps),
            )
        self._dispatch_next()
        return True

    def _append_task_steps(self, task_name, steps):
        with self._queue_lock:
            self._queue.extend((task_name, step) for step in steps)
        return True

    def _has_queued_task_steps(self, task_name):
        with self._queue_lock:
            return any(name == task_name for name, _step in self._queue)

    def _dispatch_next(self):
        should_dispatch_after_completion = False
        with self._queue_lock:
            if (
                self.exit_req.is_set()
                or self.pause_req.is_set()
                or self.drop_req.is_set()
            ):
                return
            if self._current_step is not None or self._step_thread_alive():
                return
            if not self._queue:
                should_dispatch_after_completion = self._complete_queue_locked()
            else:
                task_name, step = self._queue.popleft()
                self._current_task_name = task_name
                self._current_step = step
                self._step_thread = threading.Thread(
                    target=self._execute_step,
                    args=(task_name, step),
                    name=f"task_step:{step.name}",
                    daemon=True,
                )
                self._task_log(task_name).info(
                    "task step started",
                    step="step",
                    event="start",
                    step_name=step.name,
                    target=self.state.target_label or "unknown",
                )
                self._step_thread.start()

        if should_dispatch_after_completion:
            self._dispatch_next()

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
            self._log_step_exception(
                self._task_log(task_name),
                task_name,
                step.name,
                exc,
            )
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

            if self.exit_req.is_set():
                self._task_log(task_name, quiet_info=True).info(
                    "task queue stopped by exit request",
                    step="queue",
                    event="cancel",
                    reason="exit_requested",
                )
                self._run_cleanup_callbacks()
                self._clear_task_state()
                return
            if task_name == "recovery" and self.drop_req.is_set():
                self._queue.clear()
                self._suspended_step = None
                self._suspended_task_name = None
                self._finish_drop_recovery_queue_locked(False)
                return
            if (
                task_name == "recovery"
                and not ok
                and not self.pause_req.is_set()
                and not self.resume_req.is_set()
            ):
                self._queue.clear()
                self._suspended_step = None
                self._suspended_task_name = None
                self._finish_drop_recovery_queue_locked(False)
                return
            if (
                task_name != "recovery"
                and self.drop_req.is_set()
                and self._drop_recovery_resume_step is not None
            ):
                self._task_log(task_name, quiet_info=True).info(
                    "task step suspended for drop recovery",
                    step="step",
                    event="drop",
                    step_name=step.name,
                    target=self.state.target_label or "unknown",
                )
                return

            if ok and not self.pause_req.is_set() and not self.drop_req.is_set():
                self._task_log(task_name, quiet_info=True).info(
                    "task step completed",
                    step="step",
                    event="done",
                    step_name=step.name,
                    target=self.state.target_label or "unknown",
                )
            elif (
                self.pause_req.is_set()
                or self.drop_req.is_set()
                or self.resume_req.is_set()
            ) and step.retry_on_pause:
                if task_name == "recovery":
                    self.state.recovery_mode = True
                self._task_log(task_name, quiet_info=True).info(
                    "task step suspended",
                    step="step",
                    event="drop" if self.drop_req.is_set() else "pause",
                    step_name=step.name,
                    target=self.state.target_label or "unknown",
                )
                if self._suspended_step is None:
                    self._suspended_step = step
                    self._suspended_task_name = task_name
                if self.resume_req.is_set():
                    self._resume_suspended_step_locked()
                else:
                    return
            elif not ok:
                self._task_log(task_name).error(
                    "task step failed",
                    step="step",
                    event="fail",
                    step_name=step.name,
                    target=self.state.target_label or "unknown",
                    reason="task_step_failed",
                )
                self._queue.clear()
                self._suspended_step = None
                self._suspended_task_name = None
                self._run_cleanup_callbacks()
                self._clear_task_state()
                return

        self._dispatch_next()

    def _resume_suspended_step_locked(self):
        if self._suspended_step is not None:
            self._queue.appendleft(
                (self._suspended_task_name, self._suspended_step)
            )
            self._suspended_step = None
            self._suspended_task_name = None
        self.pause_req.clear()
        self.drop_req.clear()
        self.resume_req.clear()
        return True

    def _fail_queue(self, task_name):
        with self._queue_lock:
            self._queue.clear()
            self._current_step = None
            self._current_task_name = None
            self._step_thread = None
        self._task_log(task_name, quiet_info=True).info(
            "task queue failed",
            step="queue",
            event="fail",
            reason="task_step_exception",
        )
        self._run_cleanup_callbacks()
        self._clear_task_state()

    def _complete_queue_locked(self):
        if self._active_drop_recovery_snapshot is not None:
            return self._finish_drop_recovery_queue_locked(True)

        self._task_log(self._current_task_name or "task", quiet_info=True).info(
            "task queue completed",
            step="queue",
            event="done",
        )
        self._step_thread = None
        self._run_cleanup_callbacks()
        self._clear_task_state()
        self.exit_req.clear()
        self.drop_req.clear()
        self.resume_req.clear()
        return False

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
        self.state.target_tool = None
        self.state.recovery_mode = False
        self.state.human_grasped_tool = False
        self.state.current_command = None
        self.state.drawer_prepared_tool = None
        self.state.drawer_preparing_tool = None
        self.state.grasp_detection_mask_images = None
        self.state.grasp_detection_mask_target = None
        self.state.grasp_detection_yaw_deg = None
        self.state.grasp_detection_yaw_target = None
        self.state.grasp_detection_width_mm = None
        self.state.grasp_detection_width_target = None
        self.handoff_retry_req.clear()
        self.handoff_fallback_req.clear()
        self.handoff_decision_pending.clear()

    def _run_cleanup_callbacks(self):
        try:
            self.tool_hold_monitor.stop("task_queue_finished")
        except Exception as exc:
            self._task_log("cleanup").warn(
                "task cleanup callback failed",
                step="cleanup",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )

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
        if self.state.picking:
            return True

        self.state.picking = True
        self.state.target_label = target_label
        self.state.target_tool = target_label
        self.state.drawer_prepared_tool = None
        self.state.drawer_preparing_tool = None
        self.state._last_search_status_target = None
        if self.state.current_command is None:
            self.state.current_command = {
                "action": "bring",
                "tool_name": target_label,
            }
        if not self._load_queue("bring", self.bring_runner.build_steps(target_label)):
            self._clear_task_state()
            return False
        return True

    def _recover_after_pick_drawer_validation_failure(
        self,
        target_label,
        drawer_id,
        log,
        reason,
    ):
        self._publish_robot_status(
            "closing_drawer",
            tool_name=target_label,
            action="bring",
            message="서랍 검증 실패 후 서랍을 닫습니다.",
            reason=reason,
            command=self.state.current_command,
        )
        if not self.drawer_flow.close_drawer(drawer_id, log):
            self._publish_robot_status(
                "failed",
                tool_name=target_label,
                action="bring",
                message="서랍 검증 실패 후 서랍 닫기에 실패했습니다.",
                reason=f"{reason}_drawer_close_failed",
                command=self.state.current_command,
            )
            self.state.target_label = None
            self.state.current_command = None
            return False

        self.state.drawer_open = False
        self.state.opened_drawer_id = None
        self._publish_robot_status(
            "returning_home",
            tool_name=target_label,
            action="bring",
            message="서랍 검증 실패 후 Home 위치로 복귀합니다.",
            reason=reason,
            command=self.state.current_command,
        )
        if not self.motion.move_to_home_joints(log):
            self._publish_robot_status(
                "failed",
                tool_name=target_label,
                action="bring",
                message="서랍 검증 실패 후 Home 위치 복귀에 실패했습니다.",
                reason=f"{reason}_home_failed",
                command=self.state.current_command,
            )
            self.state.target_label = None
            self.state.current_command = None
            return False

        self._publish_robot_status(
            "returned",
            tool_name=target_label,
            action="bring",
            message="서랍 검증 실패 후 서랍을 닫고 Home으로 복귀했습니다.",
            reason="drawer_validation_recovered",
            command=self.state.current_command,
        )
        self.state.target_label = None
        self.state.current_command = None
        return True

    def _refine_pick_target_after_centering(self, target_label):
        if not self.pick_target_resolver.should_refine_grasp_point_at_top_view():
            return None
        if not self.frame_processor.has_camera_state():
            self._task_log("perception").warn(
                "camera state unavailable for top-view grasp point refine",
                step="pick_refine",
                event="unavailable",
                reason="camera_state_unavailable",
            )
            return None

        color_image = self.state.color_image.copy()
        depth_image = self.state.depth_image.copy()
        intrinsics = dict(self.state.intrinsics)
        if self.grasp_point_mode == GRASP_POINT_MODE_YOLO:
            return self._refine_yolo_pick_target_after_centering(target_label)

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
            target = self._target_with_mask_pca_result(target, target_label)
            self._log_grasp_point_refine_success(
                target,
                GRASP_POINT_MODE_CENTER,
                target_label,
            )
            return target

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
            self._task_log("vlm").warn(
                "top-view VLM service refine failed; keeping existing pick plan",
                step="pick_refine",
                event="fail",
                reason="vlm_service_failed",
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

        target = self.pick_target_resolver.target_from_selected_grasp(
            label,
            target_label,
            (
                int(response.pixel_u),
                int(response.pixel_v),
                str(response.source or self.grasp_point_mode),
                list(response.orientation_rpy_deg) or None,
            ),
            depth_image,
            intrinsics,
        )
        return self._target_with_mask_pca_result(target, target_label)

    def _refine_yolo_pick_target_after_centering(
        self,
        target_label,
        timeout_sec=2.0,
    ):
        deadline = time.monotonic() + timeout_sec

        while not self._motion_interrupted():
            if not self.frame_processor.has_camera_state():
                return None

            color_image = self.state.color_image.copy()
            depth_image = self.state.depth_image.copy()
            intrinsics = dict(self.state.intrinsics)
            results = self.detector.detect(color_image)

            matched_box = self.pick_target_resolver.matching_box(
                results[0].boxes,
                target_label,
            )
            if matched_box is not None:
                box, label = matched_box
                selected = self.grasp_point_selector.select_yolo_grasp_point(
                    results[0].boxes,
                    self.detector.names,
                    box,
                )
                if selected is not None:
                    target = self._target_with_mask_pca_result(
                        self.pick_target_resolver.target_from_selected_grasp(
                            label,
                            target_label,
                            selected,
                            depth_image,
                            intrinsics,
                        ),
                        target_label,
                    )
                    self._log_grasp_point_refine_success(
                        target,
                        GRASP_POINT_MODE_YOLO,
                        target_label,
                    )
                    return target

            if time.monotonic() >= deadline:
                self._task_log("perception").warn(
                    "YOLO grasp point fallback to center",
                    step="pick_refine",
                    event="fallback",
                    reason="grasp_point_bbox_timeout",
                    timeout_sec=timeout_sec,
                )
                target = self._target_with_mask_pca_result(
                    self.pick_target_resolver.target_from_boxes(
                        results[0].boxes,
                        target_label,
                        color_image,
                        depth_image,
                        intrinsics,
                        use_bbox_center=True,
                    ),
                    target_label,
                )
                self._log_grasp_point_refine_success(
                    target,
                    GRASP_POINT_MODE_CENTER,
                    target_label,
                )
                return target

            time.sleep(0.05)

        return None

    def _log_grasp_point_refine_success(self, target, mode, target_label):
        if target is None or not target.found:
            return

        pixel_u, pixel_v = target.pixel
        base_x, base_y, base_z = target.base_xyz
        self._task_log("perception").info(
            f"{mode} grasp point calculation succeeded",
            step="pick_refine",
            event="done",
            mode=mode,
            target=target_label,
            source=target.source,
            pixel=f"({pixel_u},{pixel_v})",
            base=f"({base_x:.3f},{base_y:.3f},{base_z:.3f})",
            yaw_deg=(
                f"{float(target.yaw_deg):.1f}"
                if target.yaw_deg is not None
                else "none"
            ),
        )

    def _target_with_mask_pca_yaw(self, target, target_label):
        if target is None or not target.found:
            return target
        pca_yaw_deg = self._mask_pca_yaw_for_target(target_label)
        if pca_yaw_deg is None:
            return target
        return replace(target, yaw_deg=float(pca_yaw_deg))

    def _target_with_mask_pca_result(self, target, target_label):
        if target is None or not target.found:
            return target
        self._generate_grasp_detection_mask_images_after_vlm_observe(
            target_label,
            grasp_point_xy=target.pixel,
        )
        return self._target_with_mask_pca_yaw(target, target_label)

    def _mask_pca_yaw_for_target(self, target_label):
        if self.state.grasp_detection_yaw_target != target_label:
            return None
        yaw_deg = self.state.grasp_detection_yaw_deg
        if yaw_deg is None:
            return None

        adjusted_yaw_deg = float(yaw_deg) + PICK_GRASP_YAW_OFFSET_DEG
        final_grasp_yaw_deg = normalize_angle_deg(adjusted_yaw_deg)
        self._task_log("perception", quiet_info=True).info(
            "grasp detection binary crop PCA yaw applied",
            step="pca_yaw",
            event="done",
            target=target_label,
            yaw_deg=f"{yaw_deg:.1f}",
            offset_deg=f"{PICK_GRASP_YAW_OFFSET_DEG:.1f}",
            adjusted_yaw_deg=f"{adjusted_yaw_deg:.1f}",
        )
        return final_grasp_yaw_deg

    def _generate_grasp_detection_mask_images_after_vlm_observe(
        self,
        target_label,
        grasp_point_xy=None,
    ):
        if not self.frame_processor.has_camera_state():
            self._task_log("perception").warn(
                "camera state unavailable for grasp detection mask image",
                step="grasp_mask",
                event="unavailable",
                reason="camera_state_unavailable",
            )
            return None

        color_image = self.state.color_image.copy()
        results = self.detector.detect(color_image)
        matched_box = self.pick_target_resolver.matching_box(
            results[0].boxes,
            target_label,
        )
        if matched_box is None:
            self._task_log("perception").warn(
                "YOLO bbox unavailable for grasp detection mask image",
                step="grasp_mask",
                event="unavailable",
                target=target_label,
                reason="target_bbox_unavailable",
            )
            return None

        box, _label = matched_box
        bbox_xyxy = box.xyxy[0].cpu().numpy().tolist()
        response = self.sam_yaw_service_client.estimate_yaw(
            color_image=color_image,
            depth_image=self.state.depth_image.copy(),
            bbox_xyxy=bbox_xyxy,
            target_label=target_label,
            grasp_point_xy=grasp_point_xy,
            intrinsics=self.state.intrinsics,
            interrupted=self._motion_interrupted,
        )
        if response is None:
            self._task_log("perception").warn(
                "SAM yaw service failed at grasp observe pose",
                step="grasp_mask",
                event="fail",
                target=target_label,
                reason="sam_yaw_service_failed",
            )
            self.state.grasp_detection_mask_images = None
            self.state.grasp_detection_mask_target = None
            self.state.grasp_detection_yaw_deg = None
            self.state.grasp_detection_yaw_target = None
            self.state.grasp_detection_width_mm = None
            self.state.grasp_detection_width_target = None
            return None

        self.state.grasp_detection_mask_images = None
        self.state.grasp_detection_mask_target = None
        self.state.grasp_detection_yaw_deg = float(response.yaw_deg)
        self.state.grasp_detection_yaw_target = target_label
        if getattr(response, "has_grasp_width_mm", False):
            self.state.grasp_detection_width_mm = float(response.grasp_width_mm)
            self.state.grasp_detection_width_target = target_label
        else:
            self.state.grasp_detection_width_mm = None
            self.state.grasp_detection_width_target = None
        self._task_log("perception", quiet_info=True).info(
            "SAM yaw service result saved",
            step="grasp_mask",
            event="done",
            target=target_label,
            yaw_deg=f"{response.yaw_deg:.1f}",
            width_mm=(
                f"{float(response.grasp_width_mm):.1f}"
                if getattr(response, "has_grasp_width_mm", False)
                else "none"
            ),
            path=response.debug_image_path,
        )
        return response

    def _motion_interrupted(self):
        return (
            self.exit_req.is_set()
            or self.pause_req.is_set()
            or self.drop_req.is_set()
        )

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
            self._task_log("perception").warn(
                "hand grasp image conversion failed",
                step="image_convert",
                event="fail",
                reason=str(exc) or type(exc).__name__,
            )

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
        self.state.depth_encoding = msg.encoding

    def _wrench_cb(self, msg):
        self.state.latest_wrench = msg.wrench

    def run(self):
        if not self.motion.ensure_gripper_self_collision_acm(
            self.get_logger(),
            attempts=5,
            retry_delay_sec=0.5,
        ):
            return
        self.home_initializer.initialize()
        self._process_frames()

    def _process_frames(self):
        if self.state.home_xyz is None or self.state.home_ori is None:
            return
        self._task_log("camera", quiet_info=True).info(
            "task coordinator camera loop waiting",
            step="camera_loop",
            event="idle",
        )

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
