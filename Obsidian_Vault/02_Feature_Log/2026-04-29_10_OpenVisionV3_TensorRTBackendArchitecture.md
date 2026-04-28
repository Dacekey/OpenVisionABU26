# OpenVision-v3 – Configurable TensorRT Backend Architecture and ONNX Runtime Fallback

## 1. Overview

This milestone adds a config-driven inference backend architecture to OpenVision-v3.

The main goal is to allow runtime backend selection by configuration between:

- `onnxruntime`
- `tensorrt`

Key points of this milestone:

- ONNX Runtime is preserved as the default stable backend.
- TensorRT Runtime is added as an optional backend.
- TensorRT is not hard-required for building the package.
- If TensorRT is unavailable, the system can fall back to ONNX Runtime.
- ROS message contracts and topic names remain unchanged.

This is a backend architecture milestone, not a verified TensorRT performance milestone yet.

## 2. Why This Was Needed

OpenVision-v3 already has an ONNX Runtime baseline benchmark. Before converting fully to TensorRT, the runtime needed a backend abstraction layer.

Why this abstraction matters:

- it allows fair ONNX Runtime vs TensorRT comparison under the same node runtime structure
- it keeps ONNX Runtime available as a safe fallback
- it avoids hard-replacing the current working ONNX pipeline
- it reduces risk while TensorRT support is still being integrated and validated

Target long-term flow:

ONNX Runtime baseline  
-> TensorRT engine build  
-> TensorRT runtime execution  
-> ONNX vs TensorRT benchmark comparison  
-> decide if TensorRT gives enough performance benefit

## 3. Files Added

New backend-related files added:

- `src/abu_yolo_ros/include/abu_yolo_ros/inference_backend.hpp`
- `src/abu_yolo_ros/include/abu_yolo_ros/onnx_runtime_backend.hpp`
- `src/abu_yolo_ros/include/abu_yolo_ros/tensorrt_backend.hpp`
- `src/abu_yolo_ros/include/abu_yolo_ros/yolo_inference_utils.hpp`
- `src/abu_yolo_ros/src/onnx_runtime_backend.cpp`
- `src/abu_yolo_ros/src/tensorrt_backend.cpp`
- `src/abu_yolo_ros/src/yolo_inference_utils.cpp`

Files modified for backend integration:

- `src/abu_yolo_ros/include/abu_yolo_ros/yolo_detector.hpp`
- `src/abu_yolo_ros/src/yolo_detector.cpp`
- `src/abu_yolo_ros/src/yolo_detection_node.cpp`
- `src/abu_yolo_ros/CMakeLists.txt`
- `src/abu_yolo_ros/config/yolo_detection.yaml`
- `src/abu_yolo_ros/launch/yolo.launch.py`
- `README.md`

## 4. Backend Abstraction Design

The new architecture is organized as:

`YOLODetector`  
-> selects backend based on config  
-> uses an `InferenceBackend` interface  
-> active backend can be `onnxruntime` or `tensorrt`

Core design roles:

- `InferenceBackend`
  common interface for all inference backends
- `ONNXRuntimeBackend`
  preserves the existing stable backend behavior
- `TensorRTBackend`
  optional backend for serialized TensorRT engine loading
- `YOLODetector`
  wrapper/facade that selects and owns the active backend

Design consistency goals:

- detection output format remains aligned with the previous runtime
- preprocessing and postprocessing semantics are kept consistent with the ONNX path
- class mapping remains unchanged
- KFS downstream modules continue to receive the same detection semantics

This means the backend can change internally without requiring downstream ROS interface changes.

## 5. ONNX Runtime Preservation

ONNX Runtime remains the default backend after the refactor.

The existing ONNX model path is still supported:

`src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx`

Preservation outcomes for this milestone:

- the ONNX path remains the default active runtime path
- the ONNX path was validated after the backend refactor
- default runtime launch with ONNX backend passed
- benchmark summaries appeared with `backend=onnxruntime`

This preserves a safe and working fallback while TensorRT support is developed and tested later.

## 6. TensorRT Backend Support

TensorRT backend support was added as an optional runtime engine loader.

Current intended behavior:

- it loads an existing serialized `.engine` file
- TensorRT is only needed when TensorRT is actually selected and available
- normal package build and ONNX Runtime use do not require TensorRT
- TensorRT engine generation is expected to happen externally, for example with `trtexec`

Current limitation:

TensorRT live inference was not performed in the current environment because TensorRT is not installed.

Environment checks observed:

```bash
which trtexec
# trtexec not found

ldconfig -p | grep nvinfer
# no output

find /usr -name NvInfer.h 2>/dev/null | head
# no output
```

Meaning of those checks:

- no `trtexec`
- no `nvinfer` runtime library
- no `NvInfer.h` development header
- therefore, TensorRT backend cannot run live on this machine yet

This milestone implemented the architecture and fallback path, not live TensorRT execution verification.

## 7. Config Changes

The runtime now includes a backend-selection inference configuration section.

Conceptual config form:

```yaml
inference:
  backend: "onnxruntime"  # onnxruntime | tensorrt
  fallback_backend: "onnxruntime"
  allow_fallback: true

  onnx:
    model_path: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx"
    use_gpu: true

  tensorrt:
    engine_path: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.engine"
    input_width: 640
    input_height: 640
    fp16: true
    workspace_size_mb: 1024
    allow_engine_rebuild: false
    onnx_model_path_for_build: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx"
```

The actual ROS parameter YAML in this workspace uses flattened keys. Equivalent current parameters include:

```yaml
inference.backend: "onnxruntime"
inference.fallback_backend: "onnxruntime"
inference.allow_fallback: true

inference.onnx.model_path: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx"
inference.onnx.use_gpu: true

inference.tensorrt.engine_path: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.engine"
inference.tensorrt.input_name: ""
inference.tensorrt.output_name: ""
inference.tensorrt.input_width: 640
inference.tensorrt.input_height: 640
inference.tensorrt.fp16: true
inference.tensorrt.workspace_size_mb: 1024
inference.tensorrt.allow_engine_rebuild: false
inference.tensorrt.onnx_model_path_for_build: "/home/dacekey/openvision_ros2_ws_v2/src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx"
```

Parameter meaning:

- `inference.backend` chooses the requested backend
- `inference.allow_fallback` enables fallback when the requested backend is unavailable
- `inference.fallback_backend` is usually `onnxruntime`
- `inference.tensorrt.engine_path` points to the `.engine` file
- `inference.onnx.model_path` keeps the ONNX fallback path available

## 8. CMake and Optional TensorRT Build

`CMakeLists.txt` now supports optional TensorRT integration via `ENABLE_TENSORRT`.

Behavior:

- if TensorRT headers and libraries are found, TensorRT backend can be compiled with real support
- if TensorRT is not found, the package still builds using ONNX Runtime only
- this prevents TensorRT from becoming a hard dependency during normal development

Current implementation details:

- `ENABLE_TENSORRT` exists as a CMake option
- TensorRT support is gated by discovery of `NvInfer.h`, `nvinfer`, `nvinfer_plugin`, and `cudart`
- the compile definition `ABU_YOLO_ROS_HAS_TENSORRT=1` is only added when TensorRT dependencies are actually found

`package.xml` did not need changes.

This is important because the current laptop environment does not have TensorRT installed.

## 9. Runtime Behavior

### Default ONNX Runtime

Command:

```bash
ros2 launch abu_yolo_ros yolo.launch.py
```

Expected behavior:

- requested backend: `onnxruntime`
- active backend: `onnxruntime`
- normal topic publishing
- benchmark logs include `backend=onnxruntime`

### TensorRT Requested Without TensorRT Environment

Command:

```bash
ros2 launch abu_yolo_ros yolo.launch.py inference_backend:=tensorrt
```

Observed or expected behavior in the current environment:

- requested backend: `tensorrt`
- TensorRT unavailable
- fallback enabled
- active backend: `onnxruntime`

Example runtime state:

```text
requested=tensorrt active=onnxruntime fallback=true
```

This confirms backend selection and fallback behavior, but it does not confirm TensorRT runtime performance.

## 10. Runtime Safety Compatibility

Runtime safety behavior remains compatible with both backends.

The existing runtime safety layer still applies:

- inference mutex
- busy-frame drop
- circuit breaker
- inference timeout detection
- skip publish on failed inference frame

Expected behavior for future TensorRT failure cases:

- record circuit breaker failure
- skip publish for that frame
- do not publish misleading empty detections

This keeps failure semantics aligned with the hardened runtime behavior already added to OpenVision-v3.

## 11. Benchmark Compatibility

Existing runtime benchmark instrumentation now includes backend name.

Benchmark logs can distinguish:

- `backend=onnxruntime`
- `backend=tensorrt`

This allows future ONNX Runtime vs TensorRT comparison using the same measured runtime stages.

Measured stages remain:

- `image_callback_total_ms`
- `preprocess_ms`
- `inference_ms`
- `postprocess_ms`
- `team_color_filter_ms`
- `kfs_instance_aggregation_ms`
- `publish_ms`
- `total_ms`

TensorRT benchmark data is not available yet because TensorRT live runtime has not been tested in this environment.

## 12. TensorRT Engine Build Plan

Planned future engine build command:

```bash
trtexec \
  --onnx=src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx \
  --saveEngine=src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.engine \
  --fp16 \
  --workspace=1024
```

Important notes:

- the `.engine` file is required to run the TensorRT backend live
- the engine must match the model
- the engine must match the input resolution
- the engine must match the TensorRT version
- the engine must match the CUDA version
- the engine must match the target GPU architecture or Jetson target
- engine files are often environment-specific and may not be portable across devices

## 13. Validation

Validation performed for this milestone:

- `colcon build --packages-select abu_yolo_ros` passed
- default ONNX launch passed
- topics published normally
- benchmark summary appeared with `backend=onnxruntime`
- TensorRT request path failed gracefully to ONNX fallback:
  `requested=tensorrt active=onnxruntime fallback=true`

Scope limitation of validation:

TensorRT live runtime was not tested because the current environment lacks:

- TensorRT headers
- TensorRT runtime libraries
- `trtexec`
- TensorRT `.engine`

## 14. Current Status

Current milestone status:

- backend abstraction is implemented
- ONNX Runtime remains stable and default
- TensorRT backend exists as an optional backend
- TensorRT is not available in the current environment
- TensorRT live inference and benchmark are not verified yet
- fallback behavior is verified
- runtime safety and benchmark integration remain compatible
- no ROS message or topic contract changes were introduced

## 15. Future Work

Next steps:

- prepare a TensorRT-capable environment
- install or use a Jetson environment with TensorRT
- verify `trtexec`
- verify `NvInfer.h`
- verify `nvinfer` libraries
- build a `.engine` from the ONNX model
- rebuild the package with TensorRT enabled
- launch with `inference_backend:=tensorrt`
- verify the actual active backend is `tensorrt`
- run benchmark with TensorRT
- compare ONNX Runtime vs TensorRT using these metrics:
- FPS
- total mean latency
- inference mean latency
- P95 latency
- P99 latency
- timeout count
- busy drop count
- circuit breaker state
- decide whether TensorRT provides enough performance gain
- consider NVMM / zero-copy only if benchmark shows camera or copy bottleneck

## 16. Suggested Commit Message

```bash
git commit -m "Add configurable TensorRT inference backend"
```

This commit message matches the scope of the milestone: the configurable TensorRT backend architecture and fallback path were added, but live TensorRT runtime validation and performance verification remain future work.
