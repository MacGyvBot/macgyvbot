"""ROS topic names shared across MacGyvBot nodes."""

CAMERA_COLOR_TOPIC = "/camera/camera/color/image_raw"
CAMERA_DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

TOOL_COMMAND_TOPIC = "/tool_command"
ROBOT_STATUS_TOPIC = "/robot_task_status"
TOOL_DROP_TOPIC = "/tool_drop_detected"
TASK_REQUEST_TOPIC = "/task_request"
TASK_CONTROL_TOPIC = "/task_control"
ROBOT_TASK_CONTROL_TOPIC = TASK_CONTROL_TOPIC
DRAWER_COMMAND_TOPIC = "/drawer_command"

HAND_GRASP_TOPIC = "/human_grasped_tool"
HAND_GRASP_IMAGE_TOPIC = "/hand_grasp_detection/annotated_image"
HAND_GRASP_MASK_LOCK_TOPIC = "/hand_grasp_detection/tool_mask_lock"

FORCE_TORQUE_TOPIC = "/force_torque_sensor_broadcaster/wrench"
