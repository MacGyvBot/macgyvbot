cmake_minimum_required(VERSION 3.8)
project(macgyvbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ToolCommand.msg"
  "msg/CommandFeedback.msg"
  "msg/RobotTaskStatus.msg"
  "msg/TaskRequest.msg"
  "msg/RobotTaskControl.msg"
  "msg/CommandShutdown.msg"
  "msg/ToolDropEvent.msg"
  "msg/HumanGraspResult.msg"
  "msg/ToolMaskLock.msg"
  "srv/VLMGrasp.srv"
  DEPENDENCIES sensor_msgs
)

ament_package()
