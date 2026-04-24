cmake_minimum_required(VERSION 3.8)
project(charge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(battery_status_get src/battery_status_get.cpp)
ament_target_dependencies(battery_status_get rclcpp std_msgs interfaces)
if (TARGET serial::serial)
  target_link_libraries(battery_status_get serial::serial)
else ()
  target_include_directories(battery_status_get PRIVATE ${serial_INCLUDE_DIRS})
  target_link_libraries(battery_status_get ${serial_LIBRARIES})
endif ()

install(TARGETS battery_status_get
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY cfg
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
