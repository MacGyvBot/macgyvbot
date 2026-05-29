# ROS Topic Contracts

This document records package-level topic ownership. It is the contract between
runtime packages, especially after command and UI are split into separate nodes.

Current runtime uses `macgyvbot_interfaces/msg/*` types for structured
package-boundary contracts. Plain recognized text and manual target labels
remain `std_msgs/String`; image and sensor streams keep their standard ROS
types.

## Command And UI

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/stt_text` | `std_msgs/String` | `macgyvbot_ui` operator UI, optional STT source | `macgyvbot_command` | Plain recognized or typed text |
| `/tool_command` | `macgyvbot_interfaces/msg/ToolCommand` | `macgyvbot_command` | `macgyvbot_task`, `macgyvbot_ui` | Tool command including `bring`, `return`, `release`, and idle-only `home` |
| `/command_feedback` | `macgyvbot_interfaces/msg/CommandFeedback` | `macgyvbot_command` | `macgyvbot_command` TTS, `macgyvbot_ui` | Command interpretation feedback |
| `/task_control` | `macgyvbot_interfaces/msg/RobotTaskControl` | `macgyvbot_command`, operator/manual tools | `macgyvbot_task` | `cancel` clears current work/queue and remains idle, while `exit` also returns Home and reports terminal status for UI shutdown |
| `/command_shutdown` | `macgyvbot_interfaces/msg/CommandShutdown` | `macgyvbot_ui` | `macgyvbot_command` | UI lifecycle signal only; closes the headless command node without issuing robot task control |

## Task Status And Safety Events

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/robot_task_status` | `macgyvbot_interfaces/msg/RobotTaskStatus` | `macgyvbot_task` | `macgyvbot_command` if context is needed, `macgyvbot_ui`, `macgyvbot_perception` | Typed task status; nested/extended compatibility details remain in `payload_json` |
| `/tool_drop_detected` | `macgyvbot_interfaces/msg/ToolDropEvent` | `macgyvbot_task` tool hold monitor | `macgyvbot_task` | Typed drop/monitor event converted into robot status for UI consumers |
| `/target_label` | `std_msgs/String` | manual tools | `macgyvbot_task` | Plain tool label for compatibility/manual pick |

## Perception

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/human_grasped_tool` | `macgyvbot_interfaces/msg/HumanGraspResult` | `macgyvbot_perception` | `macgyvbot_task` | Typed hand-tool grasp result |
| `/hand_grasp_detection/annotated_image` | `sensor_msgs/Image` | `macgyvbot_perception` | `macgyvbot_ui`, optional debug consumers | Annotated BGR/RGB image |
| `/hand_grasp_detection/tool_mask_lock` | `macgyvbot_interfaces/msg/ToolMaskLock` | `macgyvbot_perception` | `macgyvbot_task` | Typed mask lock result |

## Sensors

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense camera | `macgyvbot_task`, `macgyvbot_perception`, `macgyvbot_ui` | Color image; UI uses reception for connection state |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense camera | `macgyvbot_task`, `macgyvbot_perception` | Depth image aligned to color |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RealSense camera | `macgyvbot_task` | Camera intrinsics |
| `/force_torque_sensor_broadcaster/wrench` | `geometry_msgs/WrenchStamped` | Force/torque sensor or controller | `macgyvbot_task` | Force/torque sample |

## Contract Rules

- Topic names should be defined in `macgyvbot_config.topics` when used by more
  than one package.
- Command/status payload schemas must remain backward compatible unless a
  dedicated migration issue changes them.
- UI code should render topic payloads; it must not import command parser, STT,
  or TTS internals.
- Command code should publish feedback/status payloads; it must not import GUI
  widgets or presenters.
- `/command_shutdown` is a process-lifecycle signal for closing the UI window;
  it must not be treated as robot `exit` or trigger robot motion.
- If a launch remap changes any topic above, update this document in the same
  PR.
