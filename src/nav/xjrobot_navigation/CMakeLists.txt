cmake_minimum_required(VERSION 3.5)
project(xjrobot_navigation)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
