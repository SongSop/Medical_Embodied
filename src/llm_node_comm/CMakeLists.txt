cmake_minimum_required(VERSION 3.8)
project(llm_node_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DialogSessionFinished.msg"
  "msg/LlmDialogContent.msg"
  "msg/LlmDialogHistoryMsg.msg"
  "msg/LlmQuestion.msg"
  "msg/QuestionQueueEmpty.msg"
  "srv/EndSession.srv"
  "srv/HciDialog.srv"
  "srv/LlmDialogHistorysSrv.srv"
  "srv/NurseAlert.srv"
  "srv/TtsOneshot.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
