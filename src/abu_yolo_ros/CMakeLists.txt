cmake_minimum_required(VERSION 3.8)
project(abu_yolo_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)

# ONNX Runtime
if(DEFINED ENV{ONNXRUNTIME_DIR})
  set(ONNXRUNTIME_DIR $ENV{ONNXRUNTIME_DIR})
else()
  message(FATAL_ERROR "ONNXRUNTIME_DIR environment variable is not set")
endif()

message(STATUS "Using ONNXRUNTIME_DIR: ${ONNXRUNTIME_DIR}")

set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_DIR}/include)
set(ONNXRUNTIME_LIBRARY ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

link_directories(${ONNXRUNTIME_DIR}/lib)

# Image Passthrough Node
add_executable(image_passthrough_node
  src/image_passthrough_node.cpp
)

ament_target_dependencies(
  image_passthrough_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  image_passthrough_node
  DESTINATION lib/${PROJECT_NAME}
)

# Image BGR Debug Node
add_executable(image_bgr_debug_node
  src/image_bgr_debug_node.cpp
)

ament_target_dependencies(
  image_bgr_debug_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  image_bgr_debug_node
  ${OpenCV_LIBS}
)

install(TARGETS
  image_bgr_debug_node
  DESTINATION lib/${PROJECT_NAME}
)

# YOLO Detection Node
add_executable(yolo_detection_node
  src/yolo_detection_node.cpp
  src/yolo_detector.cpp
)

ament_target_dependencies(
  yolo_detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  vision_msgs
)

target_link_libraries(
  yolo_detection_node
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARY}
)

install(TARGETS
  yolo_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()