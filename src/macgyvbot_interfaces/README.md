# macgyvbot_interfaces

Shared ROS message, service, and action contracts for MacGyvBot packages.

This package should contain only interface definitions. It should not depend on
runtime packages such as perception, manipulation, command, or task.

The current migrated runtime still publishes compatibility JSON over
`std_msgs/String`. The messages in `msg/` are typed migration targets for those
JSON payloads, not a runtime behavior change. Publishers and subscribers should
move to these types only when both ends of a topic are migrated together.

For migration compatibility, typed messages include `payload_json` where the
legacy payload has extra fields, nested objects, or fields that are still being
stabilized. Consumers should prefer typed fields when present and use
`payload_json` as the compatibility escape hatch. Nested command objects are
carried as `command_json` on command feedback and robot task status messages.

Nullable JSON values are represented by `has_*` flags plus a concrete typed
value. Fixed image ROIs use `[x1, y1, x2, y2]` pixel arrays with a separate
`has_tool_roi` flag so an absent ROI is not confused with `[0, 0, 0, 0]`.

## Planned Interfaces

```text
msg/
  ToolCommand.msg
  CommandFeedback.msg
  RobotTaskStatus.msg
  HumanGraspResult.msg
  ToolMaskLock.msg

future msg/
  DetectedObject.msg
  DetectedObjectArray.msg
  PickTarget.msg
  GripperState.msg
  ForceContactState.msg

future srv/
  SetGripper.srv
  MoveToPose.srv
  GetRobotPose.srv

future action/
  PickTool.action
  ReturnTool.action
  MoveToHome.action
  HandoffTool.action
```

Only data that crosses package boundaries should become an interface. Internal
workflow objects such as `PickMotionPlan` should remain Python domain models.
