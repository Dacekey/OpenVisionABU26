cmake_minimum_required(VERSION 3.8)
project(abu_yolo_ros)

include(CTest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KfsInstance.msg"
  "msg/KfsInstanceArray.msg"
  "msg/LocalizedKfsInstance.msg"
  "msg/LocalizedKfsInstanceArray.msg"
  DEPENDENCIES std_msgs vision_msgs geometry_msgs
)

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
  src/team_color_filter.cpp
  src/decision_engine.cpp
  src/kfs_instance_aggregator.cpp
)

ament_target_dependencies(
  yolo_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  vision_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(yolo_detection_node ${cpp_typesupport_target})

target_link_libraries(
  yolo_detection_node
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARY}
)

install(TARGETS
  yolo_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# KFS 3D Localizer Node
add_executable(kfs_3d_localizer_node
  src/kfs_3d_localizer_node.cpp
)

ament_target_dependencies(
  kfs_3d_localizer_node
  rclcpp
  std_msgs
  geometry_msgs
  vision_msgs
)

target_link_libraries(
  kfs_3d_localizer_node
  ${OpenCV_LIBS}
  ${cpp_typesupport_target}
)

install(TARGETS
  kfs_3d_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(decision_engine_test
    test/decision_engine_test.cpp
    src/decision_engine.cpp
  )

  target_include_directories(decision_engine_test PRIVATE include)

  add_test(
    NAME decision_engine_test
    COMMAND decision_engine_test
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
