# macgyvbot_ui

Operator-facing user interface package.

## Scope

- `operator_ui_node`
- command/status/chat GUI widgets
- detector image panel
- robot/camera/detector connection panels
- Task Log presentation

This package does not import command parser, STT, or TTS internals.  It connects
to the rest of MacGyvBot through ROS topics:

```text
operator_ui_node -> /stt_text -> command_input_node
command_input_node -> /tool_command -> operator_ui_node
command_input_node -> /command_feedback -> operator_ui_node
macgyvbot_task -> /robot_task_status -> operator_ui_node
perception -> /hand_grasp_detection/annotated_image -> operator_ui_node
```
