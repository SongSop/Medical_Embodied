cmake_minimum_required(VERSION 3.8)
project(xjrobot_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(xjrobot_bridge_node
  src/xjrobot_bridge_node.cpp
)

target_include_directories(xjrobot_bridge_node
  PRIVATE
    include
)

ament_target_dependencies(xjrobot_bridge_node
  geometry_msgs
  interfaces
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
)

target_compile_features(xjrobot_bridge_node PUBLIC cxx_std_17)

install(
  TARGETS xjrobot_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
