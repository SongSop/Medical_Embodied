cmake_minimum_required(VERSION 3.8)
project(xjrobot_localization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(PythonLibs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(map_odom_tf_publisher src/map_odom_tf_publisher.cpp)
ament_target_dependencies(
  map_odom_tf_publisher
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(fastlio_mapping
  src/fastlio_localization/laserMapping.cpp
  src/fastlio_localization/preprocess.cpp
  include/fastlio_localization/ikd-Tree/ikd_Tree.cpp
)
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/fastlio_localization
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fastlio_localization
  ${PCL_INCLUDE_DIRS}
)
target_include_directories(fastlio_mapping PRIVATE
  ${PYTHON_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  Eigen3::Eigen
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fastlio_mapping "${cpp_typesupport_target}")
ament_target_dependencies(
  fastlio_mapping
  geometry_msgs
  livox_ros_driver2
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(fastlio_relative_odom_node
  src/fastlio_relative_odom_node.cpp
)
target_compile_features(fastlio_relative_odom_node PUBLIC cxx_std_17)
ament_target_dependencies(fastlio_relative_odom_node
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

install(
  DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fastlio_localization/global_localization.py
    scripts/manual_initial_pose_publisher.py
    scripts/fastlio_localization/transform_fusion.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    map_odom_tf_publisher
    fastlio_mapping
    fastlio_relative_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
