cmake_minimum_required(VERSION 3.8)
project(bt_runtime_probe)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(bt_runtime_probe src/main.cpp)
ament_target_dependencies(bt_runtime_probe behaviortree_cpp)

install(TARGETS bt_runtime_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
