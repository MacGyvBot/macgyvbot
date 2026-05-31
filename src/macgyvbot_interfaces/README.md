# macgyvbot_interfaces

Shared ROS message, service, and action contracts for MacGyvBot packages.

This package should contain only interface definitions. It should not depend on
runtime packages such as perception, manipulation, command, or task.

Runtime package-boundary topics use the message types in this package. ROS
sensor/image topics keep their native ROS types.

Command text, task request, status, feedback, safety-event, and perception
topics use typed fields only. JSON payload fields are not part of the
package-boundary runtime contracts.

Nullable values are represented by `has_*` flags plus a concrete typed value.
Fixed image ROIs use `[x1, y1, x2, y2]` pixel arrays with a separate
`has_tool_roi` flag so an absent ROI is not confused with `[0, 0, 0, 0]`.

## Planned Interfaces

```text
msg/
  CommandText.msg
  ToolCommand.msg
  TaskRequest.msg
  CommandFeedback.msg
  RobotTaskStatus.msg
  RobotTaskControl.msg
  CommandShutdown.msg
  ToolDropEvent.msg
  HumanGraspResult.msg
  ToolMaskLock.msg
srv/
  SetGripper.srv
  VLMGrasp.srv

future msg/
  DetectedObject.msg
  DetectedObjectArray.msg
  PickTarget.msg
  GripperState.msg
  ForceContactState.msg

future srv/
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
