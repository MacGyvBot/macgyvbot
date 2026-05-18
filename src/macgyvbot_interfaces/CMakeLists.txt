cmake_minimum_required(VERSION 3.8)
project(macgyvbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ToolCommand.msg"
  "msg/CommandFeedback.msg"
  "msg/RobotTaskStatus.msg"
  "msg/HumanGraspResult.msg"
  "msg/ToolMaskLock.msg"
)

ament_package()
