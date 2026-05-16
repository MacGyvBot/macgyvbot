"""ROS topic names shared across MacGyvBot nodes."""

TOOL_COMMAND_TOPIC = "/tool_command"
ROBOT_STATUS_TOPIC = "/robot_task_status"

HAND_GRASP_TOPIC = "/human_grasped_tool"
HAND_GRASP_IMAGE_TOPIC = "/hand_grasp_detection/annotated_image"
HAND_GRASP_MASK_LOCK_TOPIC = "/hand_grasp_detection/tool_mask_lock"

FORCE_TORQUE_TOPIC = "/force_torque_sensor_broadcaster/wrench"
