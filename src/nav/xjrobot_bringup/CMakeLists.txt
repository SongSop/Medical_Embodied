cmake_minimum_required(VERSION 3.5)
project(xjrobot_bringup)

find_package(ament_cmake REQUIRED)

install(
  FILES launch/robot.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
