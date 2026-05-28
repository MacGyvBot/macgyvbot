#!/usr/bin/env python3
"""Main ROS wiring node for the MacGyvBot pick pipeline."""

import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from macgyvbot_config.drawer import DRAWER_OBSERVATION_J6_DEG
from macgyvbot_config.models import YOLO_MODEL_NAME
from macgyvbot_config.robot import GROUP_NAME, HOME_JOINTS, WRIST_JOINT_NAME
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
    TOOL_DROP_TOPIC,
    TOOL_COMMAND_TOPIC,
)
from macgyvbot_config.vlm import DEFAULT_GRASP_POINT_MODE
from macgyvbot_manipulation.moveit_controller import (
    MoveItController,
)
from macgyvbot_manipulation.drawer_motion import DrawerMotionFlow
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
from macgyvbot_perception.yolo_detector import YoloDetector
from macgyvbot_perception.pick_target_resolver import (
    PickTargetResolver,
)
from macgyvbot_task.application import (
    RobotStatusPublisher,
    TaskRuntimeState,
    ToolCommandController,
    ToolDropStatusReporter,
)
from macgyvbot_task.application.adapters.hand_grasp_result_adapter import (
    HandGraspResultAdapter,
)
from macgyvbot_task.application.display.debug_display import DebugDisplay
from macgyvbot_task.application.pick_flow.pick_frame_processor import (
    PickFrameProcessor,
)
from macgyvbot_task.application.pick_flow.pick_sequence import (
    PickSequenceRunner,
)
from macgyvbot_task.application.return_flow.return_sequence import (
    ReturnSequenceRunner,
)
from macgyvbot_task.application.return_flow.return_perception_adapter import (
    ReturnPerceptionAdapter,
)
from macgyvbot_task.application.robot.robot_home_initializer import (
    RobotHomeInitializer,
)
from macgyvbot_task.application.task_control.task_control_coordinator import (
    TaskControlCoordinator,
)
from macgyvbot_task.application.task_control.exit_home_recovery import (
    ExitHomeRecovery,
)
from macgyvbot_task.application.task_control.task_management import TaskManagement


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


class MacGyvBotNode(Node):
    def __init__(self):
        super().__init__("macgyvbot_main_node")

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
        self.control_callback_group = MutuallyExclusiveCallbackGroup()

        self.grasp_point_mode = self._read_grasp_point_mode()
        self.yolo_model = self._read_yolo_model()
        self.force_torque_topic = self._read_force_torque_topic()
        self.robot_status_pub = self.create_publisher(
            String,
            ROBOT_STATUS_TOPIC,
            10,
        )
        self.tool_drop_pub = self.create_publisher(
            String,
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
        self.robot = MoveItPy(node_name="yolo_pick_moveit_py")
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.depth_projector = DepthProjector(self._base_to_camera_matrix)
        self.hand_grasp_adapter = HandGraspResultAdapter(
            self.state,
            self.depth_projector,
            self.get_logger(),
        )

        if self.display_debug_windows:
            self.get_logger().info("OpenCV debug display windows are enabled.")
        else:
            self.get_logger().info(
                "OpenCV debug display windows are disabled. "
                "Use the command GUI detector panel instead."
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
        self.task_controller = ToolCommandController(
            self.get_logger(),
            self.status_publisher,
            is_busy=lambda: self.state.picking,
            set_target=self._set_active_target,
            clear_target=self._clear_pending_target,
            reset_search_status=self._reset_search_status,
            start_return=self.start_return_sequence,
            release_gripper=self.tool_hold_monitor.release_gripper,
            move_home=self._move_home,
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
        self.return_perception = ReturnPerceptionAdapter(
            self.state,
            self.detector,
            self.drawer_flow,
            self.frame_processor,
            self.pick_target_resolver,
            self.depth_projector,
            self.get_logger(),
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
            detect_store_tool_label=self.return_perception.detect_store_tool_label,
            resolve_store_tool_target=(
                self.return_perception.resolve_store_tool_target
            ),
            resolve_drawer_marker_target=(
                self.return_perception.resolve_drawer_marker_target
            ),
        )
        self.task_coordinator = TaskControlCoordinator(
            self.pick_runner,
            self.return_runner,
            self.state,
            self.exit_req,
            self.pause_req,
            self.resume_req,
            self.get_logger,
            cleanup_callbacks=[
                lambda: self.tool_hold_monitor.stop("task_queue_finished"),
            ],
        )
        self.task_management = TaskManagement(
            self.state,
            self.task_coordinator,
            self.exit_req,
            self.pause_req,
            self.resume_req,
            self.get_logger,
        )
        self.exit_home_recovery = ExitHomeRecovery(
            self.motion,
            self.exit_req,
            self.task_coordinator,
            self._publish_robot_status,
            self.get_logger,
            lambda: self.state.current_command,
            release_gripper=self.tool_hold_monitor.release_gripper,
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

        workspace_src = Path(__file__).resolve().parents[3]
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
        self.declare_parameter(
            "sam_checkpoint",
            str(Path("weights") / "mobile_sam.pt"),
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
        self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self._cam_info_cb,
            10,
        )
        self.create_subscription(
            Image,
            CAMERA_COLOR_TOPIC,
            self._color_cb,
            10,
        )
        self.create_subscription(
            Image,
            CAMERA_DEPTH_TOPIC,
            self._depth_cb,
            10,
        )
        self.create_subscription(
            String,
            "/target_label",
            self._target_label_cb,
            10,
        )
        self.create_subscription(
            String,
            TOOL_COMMAND_TOPIC,
            self._tool_command_cb,
            10,
        )
        self.create_subscription(
            String,
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
            String,
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
        self.create_subscription(
            String,
            ROBOT_TASK_CONTROL_TOPIC,
            self._task_control_cb,
            10,
            callback_group=self.control_callback_group,
        )
        self.create_subscription(
            String,
            TOOL_DROP_TOPIC,
            self._tool_drop_cb,
            10,
            callback_group=self.control_callback_group,
        )

    def _log_startup(self):
        self.get_logger().info("노드 초기화 완료")
        self.get_logger().info(f"YOLO model: {self.yolo_model}")
        self.get_logger().info(f"grasp point mode: {self.grasp_point_mode}")
        self.get_logger().info(
            "객체 입력 예시: ros2 topic pub --once /target_label "
            "std_msgs/msg/String \"{data: cup}\""
        )
        self.get_logger().info(f"공구 명령 토픽: {TOOL_COMMAND_TOPIC}")
        self.get_logger().info(f"로봇 상태 토픽: {ROBOT_STATUS_TOPIC}")
        self.get_logger().info(f"공구 drop 감지 토픽: {TOOL_DROP_TOPIC}")
        self.get_logger().info(f"작업 제어 토픽: {ROBOT_TASK_CONTROL_TOPIC}")
        self.get_logger().info(f"잡기 인식 결과 토픽: {HAND_GRASP_TOPIC}")
        self.get_logger().info(f"잡기 인식 화면 토픽: {HAND_GRASP_IMAGE_TOPIC}")
        self.get_logger().info(f"공구 mask lock 토픽: {HAND_GRASP_MASK_LOCK_TOPIC}")
        self.get_logger().info(f"RGB 카메라 토픽: {CAMERA_COLOR_TOPIC}")
        self.get_logger().info(f"Depth 카메라 토픽: {CAMERA_DEPTH_TOPIC}")
        self.get_logger().info(f"CameraInfo 토픽: {CAMERA_INFO_TOPIC}")
        self.get_logger().info(f"힘/토크 입력 토픽: {self.force_torque_topic}")

    def _target_label_cb(self, msg):
        val = msg.data.strip()

        if not val:
            return

        self.task_controller.handle_target_label(val, source="/target_label")

    def _task_control_cb(self, msg):
        action, reason = self._parse_task_control_payload(msg.data)
        if action is None:
            return

        if action not in ("pause", "resume", "exit"):
            self.get_logger().warn(f"지원하지 않는 task control action: {action}")
            return

        self.get_logger().info(
            f"task control 수신: action={action}, reason={reason}"
        )

        handled = self.task_management.handle_control(action, reason=reason)
        if not handled:
            self.get_logger().warn(f"task control 처리 실패: action={action}")
            return

        if action in ("pause", "exit"):
            self.motion.cancel_current_goal(
                self.get_logger(),
                reason=reason or action,
            )
        if action == "exit":
            self.exit_home_recovery.move_home_after_exit(reason)

    def _motion_interrupted(self):
        return self.exit_req.is_set() or self.pause_req.is_set()

    def _parse_task_control_payload(self, payload):
        raw = (payload or "").strip()
        if not raw:
            return None, ""

        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return raw.lower(), raw.lower()

        if not isinstance(data, dict):
            return None, ""

        action = str(data.get("action", "")).strip().lower()
        reason = str(data.get("reason", "")).strip()
        return action or None, reason

    def _tool_drop_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"공구 drop 이벤트 JSON 파싱 실패: {msg.data}")
            return

        if payload.get("event") != "tool_dropped":
            return

        self._handle_tool_drop_exit(payload)

    def _handle_tool_drop_exit(self, payload):
        reason = payload.get("reason") or "tool_dropped"
        self.get_logger().warn(
            "공구 drop 감지를 task exit으로 처리합니다: "
            f"reason={reason}, tool={payload.get('tool_name', 'unknown')}"
        )
        self.motion.cancel_current_goal(
            self.get_logger(),
            reason=reason,
        )
        self.task_management.handle_control(
            "exit",
            reason=reason,
            publish_status=False,
        )

    def _tool_command_cb(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"/tool_command JSON 파싱 실패: {msg.data}")
            self._publish_robot_status(
                "rejected",
                message="명령 JSON을 해석하지 못했습니다.",
                reason="invalid_json",
            )
            return

        self.task_controller.handle_command(command)

    def _wrench_cb(self, msg):
        self.state.latest_wrench = msg.wrench

    def _set_active_target(self, tool_name, command=None):
        self.state.target_label = tool_name
        self.state.current_command = command
        self.state.drawer_prepared_tool = None
        self.state.drawer_preparing_tool = None
        self._prepare_drawer_for_target(tool_name)

    def _clear_pending_target(self):
        self.state.target_label = None
        self.state.current_command = None
        self.state.drawer_prepared_tool = None
        self.state.drawer_preparing_tool = None

    def _reset_search_status(self):
        self.state._last_search_status_target = None

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

    @staticmethod
    def _cooperative_wait(duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            time.sleep(min(0.02, max(0.0, remaining)))

    def _hand_grasp_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"잡기 인식 결과 JSON 파싱 실패: {msg.data}"
            )
            return

        result["_received_monotonic_sec"] = time.monotonic()
        self.hand_grasp_adapter.attach_base_position(result)
        self.state.last_grasp_result = result
        self.state.human_grasped_tool = bool(result.get("human_grasped_tool", False))

    def _tool_mask_lock_cb(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"공구 mask lock 결과 JSON 파싱 실패: {msg.data}")
            return

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

    def start_pick_sequence(self, bx, by, bz, vlm_yaw_deg=None):
        if self.state.picking:
            self.get_logger().warn("이미 pick 동작 중이라 새 pick 요청을 무시합니다.")
            return

        self.state.picking = True
        self._publish_robot_status(
            "picking",
            tool_name=self.state.target_label,
            action="bring",
            message=f"{self.state.target_label} pick 동작을 시작합니다.",
            command=self.state.current_command,
        )

        started = self.task_coordinator.start_pick(bx, by, bz, vlm_yaw_deg)
        if not started:
            self.state.picking = False
            self._publish_robot_status(
                "busy",
                tool_name=self.state.target_label,
                action="bring",
                message="이미 작업 큐가 실행 중입니다.",
                reason="task_queue_busy",
                command=self.state.current_command,
            )

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

    def start_return_sequence(self, command):
        if self.state.picking:
            tool_name = command.get("tool_name", "unknown")
            self.get_logger().warn(
                "이미 로봇 동작 중이라 반납 요청을 무시합니다."
            )
            self._publish_robot_status(
                "busy",
                tool_name=tool_name,
                action="return",
                message="이미 로봇 동작 중이라 반납 요청을 무시합니다.",
                reason="already_picking",
                command=command,
            )
            return

        tool_name = command.get("tool_name", "unknown")
        self.get_logger().info(f"반납 시퀀스 시작: tool={tool_name}")
        self.state.picking = True
        self.state.target_label = None
        self.state.current_command = command

        started = self.task_coordinator.start_return(command)
        if not started:
            self.state.picking = False
            self._publish_robot_status(
                "busy",
                tool_name=tool_name,
                action="return",
                message="이미 작업 큐가 실행 중입니다.",
                reason="task_queue_busy",
                command=command,
            )

    def run(self):
        self.home_initializer.initialize()
        self._process_frames()

    def _move_home(self):
        return self.home_initializer.initialize()

    def _process_frames(self):
        if self.state.home_xyz is None or self.state.home_ori is None:
            return

        self.get_logger().info("카메라 영상 대기 중...")
        self.get_logger().info(
            "다른 터미널에서 객체명을 publish 하세요. 예: "
            "ros2 topic pub --once /target_label "
            "std_msgs/msg/String \"{data: cup}\""
        )

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
        self.robot_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_tool_drop_payload(self, payload):
        self.tool_drop_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )


def main():
    rclpy.init()
    node = MacGyvBotNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="macgyvbot_main_executor",
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
