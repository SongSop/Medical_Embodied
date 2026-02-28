cmake_minimum_required(VERSION 3.0.2)
project(interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  ActionStatus.msg
  Battery.msg
  Fault.msg
)

add_service_files(
  FILES
  Dock.srv
  ChargeUntil.srv
  DetectAnomaly.srv
  FaceIdentify.srv
)

add_action_files(
  FILES
  Navigate.action
  LLMInteraction.action
  CallNurse.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)
