# OpenVisionABU26 Version 3 Technical Report

## 1. Executive Summary

OpenVision-v3 is the current ROS 2 perception stack for ABU Robocon 2026 in this workspace.

It:

- detects KFS symbols using YOLO
- aggregates symbol detections into KFS-level instances
- classifies KFS legality using instance-level team color and decision logic
- localizes KFS instances into robot frame using a monocular ray-plane method
- stabilizes localized positions using lightweight temporal filtering
- includes runtime safety hardening and benchmark instrumentation
- supports ONNX Runtime by default and includes an optional TensorRT backend architecture with ONNX fallback

## 2. System Overview

Current OpenVision-v3 runtime pipeline:

```text
Camera / Image Stream
-> YOLO Symbol Detection
-> Symbol Filtering
   - confidence
   - ROI
   - range / bbox-size filtering
-> KFSInstanceAggregator
   - square-normalized symbol geometry
   - HSV body mask / contour support
   - conservative clustering / merging
   - ambiguous cluster dropping
-> Instance-level TeamColorFilter
-> Instance-level DecisionEngine
   - legal / illegal / unknown
-> /yolo/kfs_instances
-> KFS 3D Localizer
   - bbox projection point
   - undistort pixel
   - ray_camera
   - ray_robot
   - plane_z_height intersection
-> /yolo/kfs_instances_localized
-> KFS Localization Stabilizer
   - lightweight Kalman filter
   - gating
-> /yolo/kfs_instances_stabilized
```

```mermaid
flowchart TD
    A[Camera / image_raw] --> B[yolo_detection_node]
    B --> C[/yolo/detections]
    B --> D[/yolo/image_annotated]
    B --> E[/yolo/kfs_instances]
    B --> F[/yolo/kfs_instances/image_annotated]
    E --> G[kfs_3d_localizer_node]
    G --> H[/yolo/kfs_instances_localized]
    H --> I[kfs_localization_stabilizer_node]
    I --> J[/yolo/kfs_instances_stabilized]
```

## 3. ROS Graph Snapshot

![OpenVision-v3 ROS graph](../05_Node_Images/rqt_graph_2.png)

This image was captured from `rqt_graph`. It visualizes the current ROS 2 topic and node flow and is useful for confirming the active perception graph structure.

## 4. Main Runtime Nodes

| Node / Executable | File | Role | Default Runtime? |
| --- | --- | --- | --- |
| `yolo_detection_node` | `src/abu_yolo_ros/src/yolo_detection_node.cpp` | Main perception runtime: backend inference, symbol output, KFS aggregation, instance-level color and decision, debug images, runtime safety, benchmark | Yes |
| `kfs_3d_localizer_node` | `src/abu_yolo_ros/src/kfs_3d_localizer_node.cpp` | Converts KFS 2D instances into raw robot-frame 3D estimates | Yes |
| `kfs_localization_stabilizer_node` | `src/abu_yolo_ros/src/kfs_localization_stabilizer_node.cpp` | Smooths raw localized KFS positions with Kalman filtering and gating | Yes |
| `image_bgr_debug_node` | `src/abu_yolo_ros/src/image_bgr_debug_node.cpp` | Converts raw `/image_raw` YUY2 feed into a BGR debug image topic | No |
| `image_passthrough_node` | `src/abu_yolo_ros/src/image_passthrough_node.cpp` | Republishes `/image_raw` unchanged for topic-flow validation | No |

`image_bgr_debug_node` and `image_passthrough_node` are developer utilities, not part of the main production perception pipeline. They are useful for camera and topic validation.

## 5. Module Relationship Map

| File / Component | Used By | Purpose |
| --- | --- | --- |
| `yolo_detection_node.cpp` | main runtime node | Owns detector, TeamColorFilter, DecisionEngine, KFS aggregation, publishing, runtime safety, benchmark |
| `yolo_detector.cpp/.hpp` | `yolo_detection_node` | Wrapper/facade that selects and owns the active inference backend |
| `inference_backend.hpp` | detector and backends | Common inference backend interface |
| `onnx_runtime_backend.cpp/.hpp` | `YOLODetector` | ONNX Runtime backend implementation |
| `tensorrt_backend.cpp/.hpp` | `YOLODetector` | Optional TensorRT serialized-engine backend |
| `yolo_inference_utils.cpp/.hpp` | ONNX/TensorRT backends | Shared preprocessing and YOLO output decoding |
| `team_color_filter.cpp/.hpp` | `yolo_detection_node` | HSV-based team-color evaluation for symbol and instance crops |
| `decision_engine.cpp/.hpp` | `yolo_detection_node` | Legality decision logic: `legal / illegal / unknown` |
| `kfs_instance_aggregator.cpp/.hpp` | `yolo_detection_node` | Symbol-to-KFS aggregation, filtering, clustering, merge safety, contour refinement |
| `kfs_3d_localizer_node.cpp` | standalone runtime node | Raw 3D localization from KFS 2D instances |
| `kfs_localization_stabilizer_node.cpp` | standalone runtime node | Stabilized 3D output via CV Kalman filter and gating |
| `image_bgr_debug_node.cpp` | developer utility | BGR debug conversion from `/image_raw` |
| `image_passthrough_node.cpp` | developer utility | Raw image passthrough debug node |
| `block_localizer.hpp` | future hook | Placeholder future module for block localization; not part of the active runtime |
| `temporal_tracker.hpp` | future hook | Placeholder future module for generic detection tracking; not part of the active runtime |

## 6. Topics and Message Contracts

| Topic | Message Type | Publisher | Consumer | Purpose |
| --- | --- | --- | --- | --- |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | `yolo_detection_node` | debug / compatibility consumers | Symbol-level YOLO detections |
| `/yolo/image_annotated` | `sensor_msgs/msg/Image` | `yolo_detection_node` | image viewers | Symbol-level annotated image |
| `/yolo/kfs_instances` | `abu_yolo_ros/msg/KfsInstanceArray` | `yolo_detection_node` | `kfs_3d_localizer_node` and future planning modules | KFS-level interpreted runtime output |
| `/yolo/kfs_instances/image_annotated` | `sensor_msgs/msg/Image` | `yolo_detection_node` | image viewers | KFS-level debug visualization |
| `/yolo/kfs_instances_localized` | `abu_yolo_ros/msg/LocalizedKfsInstanceArray` | `kfs_3d_localizer_node` | `kfs_localization_stabilizer_node` | Raw localized KFS output |
| `/yolo/kfs_instances_stabilized` | `abu_yolo_ros/msg/LocalizedKfsInstanceArray` | `kfs_localization_stabilizer_node` | future block-map / planner consumers | Stabilized localized KFS output |

### Custom Messages

**`KfsInstance.msg`**

High-level fields:

- `cluster_id`
- `group_type`
- `decision`
- `confidence`
- `bbox`
- `bbox_quality`
- `symbol_indices`
- `class_names`
- `team_color`, `team_color_match`, `color_confidence`, `color_mask_coverage`
- `ambiguous`, `ambiguous_reason`

**`KfsInstanceArray.msg`**

- `header`
- `team_color`
- `instances[]`

**`LocalizedKfsInstance.msg`**

High-level fields:

- `source_instance`
- `localized`, `localization_status`, `localization_quality`, `failure_reason`
- `position_robot_mm`
- `distance_mm`, `bearing_deg`
- `projection_point_mode`, `projection_u`, `projection_v`
- `plane_z_height_mm`, `plane_height_source`
- `ray_camera_*`
- `ray_robot_*`

**`LocalizedKfsInstanceArray.msg`**

- `header`
- `team_color`
- `instances[]`

## 7. Techniques Used

### 7.1 YOLO Symbol Detection

- Detects symbol-level classes from the input image stream.
- Runs through a configurable inference backend.
- Publishes `vision_msgs/msg/Detection2DArray` for backward compatibility and debug visibility.

### 7.2 Backend Abstraction

- `InferenceBackend` allows switching between ONNX Runtime and TensorRT.
- ONNX Runtime is the default backend.
- TensorRT backend is optional and expects an existing serialized `.engine`.
- ONNX fallback remains available when TensorRT is requested but unavailable.
- ONNX Runtime can attempt CUDA execution and warns/falls back if CUDA provider enablement fails.

### 7.3 KFS Instance Aggregation

- Converts symbol detections into KFS-level instances.
- Uses ROI filtering, symbol size filtering, square-normalized geometry, conservative clustering, second-stage merge guards, HSV body-mask support, and contour refinement.
- Drops ambiguous clusters when configured to do so.
- Produces `/yolo/kfs_instances`.

### 7.4 TeamColorFilter

- HSV-based color evaluation.
- Used at both symbol level and instance level.
- Instance-level evaluation uses `refined_bbox` first and `expanded_bbox` as fallback.
- Provides team-color evidence for legality interpretation.

### 7.5 DecisionEngine

- Current labels are `legal`, `illegal`, and `unknown`.
- It no longer represents direct robot actions like `collect` or `avoid`.
- `FAKE` and `R1` are illegal.
- `REAL` with strong team-color match can be legal.
- `REAL` without strong evidence can be unknown.
- Final collection strategy belongs to downstream planning, not the vision node.

### 7.6 KFS 3D Localizer

- Converts a 2D KFS bbox projection point into a robot-frame ray.
- Supports distortion modes: `none`, `pinhole`, and `fisheye`.
- Uses camera intrinsics and configured camera pose.
- Intersects the robot-frame ray with a selected plane height.
- Current active stable plane mode is `fixed`.
- `block_map` plane height exists as a future hook.

### 7.7 KFS Localization Stabilizer

- Smooths localized KFS positions.
- Uses a lightweight constant-velocity Kalman filter plus gating.
- Associates measurements conservatively using distance and compatibility checks.
- Does not perform target selection or strategy logic.
- Publishes `/yolo/kfs_instances_stabilized`.

### 7.8 Runtime Safety Hardening

- Uses `SensorDataQoS` / `BEST_EFFORT` on realtime perception streams.
- Protects inference-critical shared state with a mutex when enabled.
- Drops incoming frames when inference is already busy.
- Uses a circuit breaker with `CLOSED / OPEN / HALF_OPEN` behavior.
- Skips publish on failed inference frames rather than publishing misleading empty output.
- Dedicated health/status topic remains future work.

### 7.9 Runtime Benchmark

- Adds ONNX Runtime baseline benchmark instrumentation.
- Measures stages including:
  - preprocess
  - inference
  - postprocess
  - team color
  - KFS aggregation
  - publish
  - total
- Logs FPS and latency summary statistics such as mean, p50, p95, and p99.
- Intended to provide the comparison baseline for future TensorRT evaluation.

### 7.10 Developer Tools

- `kfs_instance_prototype.py`: offline prototype for symbol-to-KFS aggregation experiments.
- `calibrate_camera.py`: offline camera calibration tool for localizer intrinsics and distortion.
- `hsv_calibration_viewer.py`: interactive HSV threshold tuning utility.
- `tune_decision_thresholds.py`: offline threshold sweep tool for DecisionEngine tuning.

## 8. Configuration Overview

| Config Group | Purpose |
| --- | --- |
| inference backend config | Select ONNX Runtime or TensorRT, fallback behavior, engine/model paths |
| model paths | ONNX model path and class-names path |
| team color / HSV config | TeamColorFilter thresholds and matching behavior |
| KFS instance aggregation config | ROI, size filtering, clustering, merge, contour, ambiguity handling |
| KFS debug image config | KFS debug-topic enablement and ROI overlay |
| KFS 3D localizer config | intrinsics, distortion, camera pose, projection strategy, plane mode, validation |
| KFS localization stabilizer config | filter model, process noise, gating, track lifecycle, output behavior |
| runtime safety config | QoS, inference mutex, busy-frame drop, circuit breaker |
| runtime benchmark config | summary interval, warmup, per-frame logging, publish-time inclusion |

Actual parameter values live in `src/abu_yolo_ros/config/yolo_detection.yaml`.

## 9. Current Demo / Test Commands

### Build

```bash
cd ~/openvision_ros2_ws_v2
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select abu_yolo_ros
source install/setup.zsh
```

### Run Main Launch

```bash
ros2 launch abu_yolo_ros yolo.launch.py
```

### Run with TensorRT Requested

```bash
ros2 launch abu_yolo_ros yolo.launch.py inference_backend:=tensorrt
```

On the current laptop, TensorRT may fall back to ONNX Runtime if TensorRT is unavailable. Confirm the active backend from startup logs.

### Run Individual Nodes if Needed

```bash
ros2 run abu_yolo_ros kfs_3d_localizer_node
ros2 run abu_yolo_ros kfs_localization_stabilizer_node
ros2 run abu_yolo_ros image_bgr_debug_node
ros2 run abu_yolo_ros image_passthrough_node
```

## 10. Useful Observation / Debug Commands

### Topic List

```bash
ros2 topic list | grep yolo
ros2 topic list | grep kfs
```

### Topic Echo

```bash
ros2 topic echo /yolo/kfs_instances --once
ros2 topic echo /yolo/kfs_instances_localized --once
ros2 topic echo /yolo/kfs_instances_stabilized --once
```

### Topic Rate

```bash
ros2 topic hz /yolo/kfs_instances
ros2 topic hz /yolo/kfs_instances_localized
ros2 topic hz /yolo/kfs_instances_stabilized
```

### QoS Info

```bash
ros2 topic info /yolo/kfs_instances --verbose
ros2 topic info /yolo/detections --verbose
```

### Parameters

```bash
ros2 param list /yolo_detection_node | grep runtime_safety
ros2 param list /yolo_detection_node | grep runtime_benchmark
```

### Interface Inspection

```bash
ros2 interface show abu_yolo_ros/msg/KfsInstance
ros2 interface show abu_yolo_ros/msg/KfsInstanceArray
ros2 interface show abu_yolo_ros/msg/LocalizedKfsInstance
ros2 interface show abu_yolo_ros/msg/LocalizedKfsInstanceArray
```

### ROS Graph

```bash
rqt_graph
```

### Image Viewer

```bash
ros2 run rqt_image_view rqt_image_view
```

Useful image topics:

- `/yolo/image_annotated`
- `/yolo/kfs_instances/image_annotated`

## 11. Current Validation Snapshot

Based on the current repository logs and documentation:

- the runtime pipeline can launch
- KFS topics exist and are wired into the localizer and stabilizer chain
- ONNX baseline benchmark instrumentation can log runtime summaries
- runtime safety parameters are loaded and logged
- realtime perception topics use `BEST_EFFORT` QoS
- TensorRT backend architecture exists, but live TensorRT requires a proper target environment

Observed benchmark examples already documented elsewhere show ONNX baseline behavior around the 30 FPS class with total mean latency in the low-30 ms range. These are observed examples, not a final production benchmark claim.

## 12. Current Limitations

- Camera calibration is not completed.
- Camera calibration file may be missing on the current machine.
- Some camera controls may not exist on the current device or driver.
- The 3D localizer still depends on placeholder intrinsics and extrinsics until calibration is completed.
- `intersection_behind_camera` can occur if camera pose or ray convention is not calibrated correctly.
- `plane.mode=fixed` is the current active stable mode.
- `block_map` plane-height integration is future work.
- TensorRT live runtime is not verified on the current laptop.
- TensorRT `.engine` should be built on the target environment, preferably Jetson Nano.
- GStreamer, NVMM, DMA, and zero-copy optimization are future work only.
- A dedicated health/status topic is future work.
- KFS Target Priority Tracking, Closest KFS Selection, and Visual Servoing are future work.
- `BlockLocalizer` / Meihua block-map integration is future work.

## 13. Future Work Summary

Main future directions:

- camera calibration
- extrinsics calibration
- block map and block height integration
- `BlockLocalizer` integration
- TensorRT live benchmark on Jetson Nano
- NVMM / zero-copy only if benchmark proves a bottleneck
- strategy-level target logic
- visual servoing
- runtime health diagnostics

Reference:

`../04_Future_Work/2026-04-29_00_OpenVisionV3_GeneralFutureWork.md`

## 14. Conclusion

OpenVision-v3 now provides a KFS-level perception pipeline for ABU Robocon 2026.

It outputs:

- symbol detections
- KFS instances
- localized KFS
- stabilized KFS

It also includes runtime safety hardening, ONNX benchmark instrumentation, and a backend abstraction layer with ONNX Runtime default and optional TensorRT support.

The stack is ready for calibration, block-map integration, and target-hardware validation.
