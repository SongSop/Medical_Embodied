cmake_minimum_required(VERSION 3.8)
project(llm_node_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(llm_node_comm REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
pkg_check_modules(SPEEXDSP REQUIRED speexdsp)

add_executable(ros_node_keyword_asr
  src/ros_node_keyword_asr.cpp
  src/asr_stream.cpp
  src/asr_punctuation.cpp
  src/keywords.cpp
)

target_compile_features(ros_node_keyword_asr PRIVATE cxx_std_20)

target_include_directories(ros_node_keyword_asr PRIVATE
  include
  ${PORTAUDIO_INCLUDE_DIRS}
  ${SPEEXDSP_INCLUDE_DIRS}
  /opt/sherpa-onnx/aarch64-install-static-lib/include
)

target_link_directories(ros_node_keyword_asr PRIVATE
  /opt/sherpa-onnx/aarch64-install-static-lib/lib
)

ament_target_dependencies(ros_node_keyword_asr
  llm_node_comm
  rclcpp
  std_msgs
)

target_link_libraries(ros_node_keyword_asr
  ${PORTAUDIO_LIBRARIES}
  ${SPEEXDSP_LIBRARIES}
  sherpa-onnx-c-api
)

set_target_properties(ros_node_keyword_asr PROPERTIES
  BUILD_RPATH "/opt/sherpa-onnx/aarch64-install-static-lib/lib"
  INSTALL_RPATH "/opt/sherpa-onnx/aarch64-install-static-lib/lib"
)

install(TARGETS
  ros_node_keyword_asr
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
