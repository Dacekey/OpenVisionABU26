cmake_minimum_required(VERSION 3.8)
project(abu_yolo_ros)

include(CTest)
option(ENABLE_TENSORRT "Enable TensorRT backend when libraries are available" ON)

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

set(ABU_YOLO_ROS_TENSORRT_ENABLED FALSE)
if(ENABLE_TENSORRT)
  find_path(TENSORRT_INCLUDE_DIR
    NvInfer.h
    PATHS
      /usr/include
      /usr/include/x86_64-linux-gnu
      /usr/local/include
      /usr/local/TensorRT/include
  )
  find_library(TENSORRT_NVINFER_LIBRARY
    nvinfer
    PATHS
      /usr/lib
      /usr/lib/x86_64-linux-gnu
      /usr/local/lib
      /usr/local/TensorRT/lib
  )
  find_library(TENSORRT_NVINFER_PLUGIN_LIBRARY
    nvinfer_plugin
    PATHS
      /usr/lib
      /usr/lib/x86_64-linux-gnu
      /usr/local/lib
      /usr/local/TensorRT/lib
  )
  find_library(CUDART_LIBRARY
    cudart
    PATHS
      /usr/lib
      /usr/lib/x86_64-linux-gnu
      /usr/local/cuda/lib64
      /usr/local/lib
  )

  if(TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER_LIBRARY AND TENSORRT_NVINFER_PLUGIN_LIBRARY AND CUDART_LIBRARY)
    set(ABU_YOLO_ROS_TENSORRT_ENABLED TRUE)
    message(STATUS "TensorRT backend enabled")
  else()
    message(WARNING "TensorRT requested but not found; building ONNX Runtime backend only")
  endif()
endif()

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
  src/onnx_runtime_backend.cpp
  src/tensorrt_backend.cpp
  src/yolo_inference_utils.cpp
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

target_include_directories(yolo_detection_node PRIVATE include)

if(ABU_YOLO_ROS_TENSORRT_ENABLED)
  target_compile_definitions(yolo_detection_node PRIVATE ABU_YOLO_ROS_HAS_TENSORRT=1)
  target_include_directories(yolo_detection_node PRIVATE ${TENSORRT_INCLUDE_DIR})
endif()

target_link_libraries(
  yolo_detection_node
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARY}
)

if(ABU_YOLO_ROS_TENSORRT_ENABLED)
  target_link_libraries(
    yolo_detection_node
    ${TENSORRT_NVINFER_LIBRARY}
    ${TENSORRT_NVINFER_PLUGIN_LIBRARY}
    ${CUDART_LIBRARY}
  )
endif()

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

# KFS Localization Stabilizer Node
add_executable(kfs_localization_stabilizer_node
  src/kfs_localization_stabilizer_node.cpp
)

ament_target_dependencies(
  kfs_localization_stabilizer_node
  rclcpp
  std_msgs
  geometry_msgs
  vision_msgs
)

target_link_libraries(
  kfs_localization_stabilizer_node
  ${OpenCV_LIBS}
  ${cpp_typesupport_target}
)

install(TARGETS
  kfs_localization_stabilizer_node
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
