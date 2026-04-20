cmake_minimum_required(VERSION 3.8)
project(xjrobot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanFrame.msg"
  "msg/RealCMD.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_library(wheel_motor_driver STATIC
  src/driver/wheel_motor_driver.cpp
)
target_compile_features(wheel_motor_driver PUBLIC cxx_std_17)
target_include_directories(wheel_motor_driver PUBLIC include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(base_controller_node src/driver/base_controller_node.cpp)
target_compile_features(base_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(base_controller_node
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(base_controller_node "${cpp_typesupport_target}")

add_executable(can_hardware_node src/driver/can_hardware_node.cpp)
target_compile_features(can_hardware_node PUBLIC cxx_std_17)
ament_target_dependencies(can_hardware_node
  rclcpp
)
target_link_libraries(can_hardware_node
  wheel_motor_driver
  "${cpp_typesupport_target}"
  controlcan
)
set_target_properties(can_hardware_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

add_executable(wheel_odom_fusion_node src/wheel_odom_fusion_node.cpp)
target_compile_features(wheel_odom_fusion_node PUBLIC cxx_std_17)
ament_target_dependencies(wheel_odom_fusion_node
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(wheel_odom_fusion_node "${cpp_typesupport_target}")

add_executable(mid360_rotation_node src/mid360_rotation_node.cpp)
target_compile_features(mid360_rotation_node PUBLIC cxx_std_17)
ament_target_dependencies(mid360_rotation_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
  tf2
)

target_link_directories(can_hardware_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)

install(TARGETS
  wheel_motor_driver
  ARCHIVE DESTINATION lib
)

install(TARGETS
  base_controller_node
  can_hardware_node
  mid360_rotation_node
  wheel_odom_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    lib/libcontrolcan.so
    lib/librockmong.so
    lib/libusb-1.0.so
  DESTINATION lib
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
