cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionStatus.msg"
  "msg/Battery.msg"
  "msg/Fault.msg"
  "srv/Dock.srv"
  "srv/ChargeUntil.srv"
  "srv/DetectAnomaly.srv"
  "srv/FaceIdentify.srv"
  "srv/SetConfig.srv"
  "action/Navigate.action"
  "action/LLMInteraction.action"
  "action/CallNurse.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
