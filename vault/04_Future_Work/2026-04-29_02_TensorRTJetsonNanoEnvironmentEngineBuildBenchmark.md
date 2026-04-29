# TensorRT on Jetson Nano – Environment Setup, Engine Build, and Live Benchmark Plan

## 1. Purpose

This note documents future work for deployment validation on Jetson Nano.

Main goal:

- prepare the Jetson Nano environment
- build a TensorRT `.engine` from the ONNX model
- run OpenVision-v3 with the TensorRT backend
- benchmark TensorRT against the existing ONNX Runtime baseline

This task is separate from the already completed TensorRT backend architecture milestone.

That earlier milestone already delivered:

- default backend: ONNX Runtime
- optional backend: TensorRT Runtime
- config-driven backend selection
- ONNX fallback when TensorRT is unavailable
- runtime benchmark logs that include backend name

This note is about environment setup, engine generation, live runtime validation, and target-hardware benchmarking.

## 2. Why This Must Be Done on Jetson Nano

TensorRT engine files are often device-specific or environment-specific.

They can depend on:

- GPU architecture
- TensorRT version
- CUDA version
- cuDNN version
- input shape
- FP32 / FP16 / INT8 precision mode
- JetPack version

Because of that, the engine should ideally be built and tested on the actual target hardware: Jetson Nano.

Important practical note:

- building a `.engine` on a laptop and copying it to Jetson may fail
- even if it loads, results may be unreliable or not representative of the real deployment environment

For deployment-quality validation, engine generation and runtime benchmarking should happen directly on the Jetson Nano / JetPack environment that will run the robot.

## 3. Current Status

Completed:

- ONNX Runtime baseline exists
- backend abstraction exists
- TensorRT backend loader exists
- ONNX fallback works
- benchmark instrumentation exists

Not completed yet:

- TensorRT environment validation on Jetson Nano
- `.engine` build
- `ENABLE_TENSORRT=ON` build on Jetson
- actual active backend = `tensorrt`
- TensorRT benchmark
- ONNX vs TensorRT comparison on target hardware

Current laptop limitation:

- `which trtexec` -> `trtexec not found`
- `ldconfig -p | grep nvinfer` -> no output
- `find /usr -name NvInfer.h 2>/dev/null | head` -> no output

Therefore, the current laptop can validate ONNX Runtime behavior and fallback behavior only. Real TensorRT validation should move to Jetson Nano.

## 4. Jetson Nano Environment Checklist

Run these commands directly on Jetson Nano:

```bash
which trtexec
ldconfig -p | grep nvinfer
find /usr -name NvInfer.h 2>/dev/null | head
dpkg -l | grep tensorrt
cat /etc/nv_tegra_release
nvcc --version
```

Expected:

- `trtexec` exists
- `nvinfer` libraries exist
- `NvInfer.h` header exists
- TensorRT package appears in `dpkg`
- JetPack / L4T version is visible

If any of these are missing, TensorRT runtime packages, TensorRT development packages, CUDA, or the JetPack environment may need to be checked or installed first.

## 5. ONNX Model Input

Current ONNX model:

`src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx`

The TensorRT engine should be generated from this ONNX file unless a newer model is selected later.

The generated engine path should stay aligned with the current runtime configuration:

`inference.tensorrt.engine_path`

Current configured path in `yolo_detection.yaml`:

`src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.engine`

## 6. Build TensorRT Engine

Planned command:

```bash
cd ~/openvision_ros2_ws_v2

trtexec \
  --onnx=src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.onnx \
  --saveEngine=src/abu_yolo_ros/models/fptu_abu26_detect_yolo_v1.engine \
  --fp16 \
  --workspace=1024
```

Important notes:

- verify `trtexec --help` first because TensorRT versions may use slightly different flags
- if FP16 is unsupported or unstable, test FP32 as fallback
- engine path should match `inference.tensorrt.engine_path` in `yolo_detection.yaml`
- engine generation should be treated as target-device preparation, not as a laptop-side artifact build

## 7. Build ROS Package with TensorRT Enabled

Planned commands:

```bash
cd ~/openvision_ros2_ws_v2
source /opt/ros/jazzy/setup.zsh

colcon build --packages-select abu_yolo_ros --cmake-args -DENABLE_TENSORRT=ON

source install/setup.zsh
```

Expected:

- CMake finds TensorRT headers and libraries
- `ABU_YOLO_ROS_HAS_TENSORRT` or equivalent compile support is enabled
- package builds successfully

If CMake still reports TensorRT missing, environment discovery paths or TensorRT dev/runtime installation must be fixed before live validation.

## 8. Run OpenVision-v3 with TensorRT Backend

Planned command:

```bash
ros2 launch abu_yolo_ros yolo.launch.py inference_backend:=tensorrt
```

Expected runtime log:

```text
requested=tensorrt active=tensorrt fallback=false
```

If runtime instead shows:

```text
requested=tensorrt active=onnxruntime fallback=true
```

then TensorRT is still not active and the system is falling back to ONNX Runtime.

That should be treated as incomplete TensorRT activation, not a successful TensorRT validation.

## 9. Runtime Topic Checks

Planned commands:

```bash
ros2 topic list | grep yolo
ros2 topic hz /yolo/kfs_instances
ros2 topic hz /yolo/kfs_instances_localized
ros2 topic hz /yolo/kfs_instances_stabilized
ros2 topic echo /yolo/kfs_instances --once
```

Expected:

- topics publish normally
- no message contract changes
- no runtime crash
- `/yolo/kfs_instances` remains compatible with the downstream KFS pipeline

This check matters because backend replacement must not break downstream interfaces or semantic expectations.

## 10. Benchmark Comparison Plan

Use the existing runtime benchmark instrumentation already added to OpenVision-v3.

ONNX Runtime and TensorRT Runtime should be compared under the same conditions:

- same camera
- same resolution
- same model
- same input scene
- same config except backend
- same hardware

Metrics to compare:

- FPS
- total mean latency
- inference mean latency
- P50 latency
- P95 latency
- P99 latency
- timeout count
- busy drop count
- circuit breaker state
- stability of `/yolo/kfs_instances`
- KFS detection consistency

Comparison template:

| Metric | ONNX Runtime Baseline | TensorRT Runtime | Improvement |
| --- | --- | --- | --- |
| FPS | TBD | TBD | TBD |
| total mean ms | TBD | TBD | TBD |
| inference mean ms | TBD | TBD | TBD |
| P95 latency | TBD | TBD | TBD |
| P99 latency | TBD | TBD | TBD |
| timeout count | TBD | TBD | TBD |
| busy drop count | TBD | TBD | TBD |
| active backend | `onnxruntime` | `tensorrt` | - |

Benchmark interpretation rule:

TensorRT should only be considered beneficial if it improves real runtime behavior, not only theoretical model execution time.

## 11. Risks and Notes

Known risks:

- TensorRT engine may fail to build if the ONNX graph contains unsupported operations
- engine files may not be portable across devices
- FP16 behavior may differ from FP32
- TensorRT output tensor layout must match current decoder assumptions
- detection output should be compared against ONNX output to avoid silent wrong detections
- Jetson Nano memory may be limited
- build time may be long on Jetson Nano

Validation note:

TensorRT activation is not enough by itself. Output quality and semantic consistency must also be checked against the ONNX path.

## 12. Success Criteria

This future work is considered successful when:

- Jetson has TensorRT environment available
- `.engine` file is generated successfully
- `colcon build` with `ENABLE_TENSORRT=ON` passes
- runtime log shows `active=tensorrt`
- `/yolo/kfs_instances` publishes normally
- runtime benchmark summary appears with `backend=tensorrt`
- TensorRT output is visually and semantically consistent with ONNX output
- benchmark comparison shows whether TensorRT provides real benefit

## 13. Future Follow-up

After TensorRT benchmark is complete, decide whether to:

- keep ONNX as default
- switch TensorRT to default on Jetson
- tune FP16 vs FP32 behavior
- try INT8 only if calibration data and accuracy validation are available
- consider NVMM / zero-copy only if benchmark shows a camera or copy bottleneck

This note is a future work plan only. It does not claim that TensorRT is already working, that a `.engine` file already exists, or that TensorRT already improves performance.
