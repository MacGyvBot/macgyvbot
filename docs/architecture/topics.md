# ROS Topic Contracts

This document records package-level topic ownership. It is the contract between
runtime packages, especially after command and UI are split into separate nodes.

Current runtime uses `std_msgs/String` with JSON payloads for command/status
contracts unless noted otherwise. Typed message migration belongs in a separate
issue.

## Command And UI

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/stt_text` | `std_msgs/String` | `macgyvbot_ui` operator UI, optional STT source | `macgyvbot_command` | Plain recognized or typed text |
| `/tool_command` | `std_msgs/String` | `macgyvbot_command` | `macgyvbot_task`, `macgyvbot_ui` | JSON tool command payload, including `bring`, `return`, `release`, and idle-only `home` |
| `/command_feedback` | `std_msgs/String` | `macgyvbot_command` | `macgyvbot_ui` | JSON command interpretation feedback |
| `/robot_task_control` | `std_msgs/String` | `macgyvbot_command`, operator/manual tools | `macgyvbot_task` | JSON task control action; `exit` cancels work, returns Home, and reports terminal status for UI shutdown |

## Task Status And Safety Events

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/robot_task_status` | `std_msgs/String` | `macgyvbot_task` | `macgyvbot_command` if context is needed, `macgyvbot_ui`, `macgyvbot_perception` | JSON task status payload |
| `/tool_drop_detected` | `std_msgs/String` | `macgyvbot_task` tool hold monitor | `macgyvbot_task`, UI/status consumers | JSON drop event payload |
| `/target_label` | `std_msgs/String` | manual tools | `macgyvbot_task` | Plain tool label for compatibility/manual pick |

## Perception

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/human_grasped_tool` | `std_msgs/String` | `macgyvbot_perception` | `macgyvbot_task` | JSON hand-tool grasp result |
| `/hand_grasp_detection/annotated_image` | `sensor_msgs/Image` | `macgyvbot_perception` | `macgyvbot_ui`, optional debug consumers | Annotated BGR/RGB image |
| `/hand_grasp_detection/tool_mask_lock` | `std_msgs/String` | `macgyvbot_perception` | `macgyvbot_task` | JSON mask lock result |

## Sensors

| Topic | Type | Publisher | Subscriber | Payload |
| --- | --- | --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense camera | `macgyvbot_task`, `macgyvbot_perception` | Color image |
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
- If a launch remap changes any topic above, update this document in the same
  PR.
