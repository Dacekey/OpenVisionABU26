# OpenVision-v3 – General Future Work Index

## 1. Purpose

This file is a consolidated Future Work index for OpenVision-v3.

- Individual feature logs may contain local Future Work notes.
- Separate files under `04_Future_Work` may describe specific future tasks in more detail.
- This file collects those items into one central roadmap.
- This is not a claim that these items are implemented.
- This file is intended to help prioritize future development.

## 2. Current OpenVision-v3 Baseline

Current implemented baseline:

- KFS-level runtime perception
- `KFSInstanceAggregator`
- instance-level `TeamColorFilter`
- instance-level `DecisionEngine`
- decision labels: `legal / illegal / unknown`
- `/yolo/kfs_instances`
- KFS 3D Localizer
- `/yolo/kfs_instances_localized`
- KFS Localization Stabilizer
- `/yolo/kfs_instances_stabilized`
- runtime safety hardening
- ONNX Runtime baseline benchmark
- configurable TensorRT backend architecture with ONNX fallback

## 3. Future Work by Area

### 3.1 Camera Calibration and Camera Model

- Perform real camera calibration for the IMX219-160. (Calibration Tool just ouput intrinsics, not including extrinsic)
- Capture calibration images for the actual runtime mode.
- Decide and validate `fisheye` vs `pinhole` model for the deployed camera.
- Update `fx`, `fy`, `cx`, `cy`.
- Update distortion coefficients.
- Ensure runtime resolution matches calibration resolution.
- Use `src/abu_yolo_ros/tools/calibrate_camera.py` as the calibration-tool reference.
- Optionally save calibration to `~/.ros/camera_info/default_cam.yaml` for `camera_info` use when appropriate.

Source-backed notes:

- current docs explicitly say calibration is incomplete
- current 3D localizer values are placeholders
- runtime camera mode and localizer config must match

### 3.2 Camera Pose / Extrinsics Calibration

- Measure and update camera offset `x / y / z` relative to the robot frame.
- Calibrate `roll / pitch / yaw`.
- Validate camera frame convention.
- Debug `intersection_behind_camera`.
- Verify ray direction, sign conventions, and ray-plane intersection behavior.

### 3.3 Plane Height and Block Map

- `plane.mode=fixed` is effectively the active stable mode today.
- Treat `block_map` as the preferred future plane-height source.
- Integrate future block ID / block height lookup into plane selection.
- Keep `instance_hint` postponed until there is a better strategy-backed use case.
- Connect board-map information to `BlockLocalizer` and height selection when ready.

No explicit source-backed item was found for a finalized documented list of valid Meihua heights such as `[0, 200, 400, 600]`, so that exact numeric set is not asserted here.

### 3.4 BlockLocalizer / Meihua Block Map Integration

- Convert stabilized KFS positions into Meihua block IDs.
- Build a legal KFS map from stabilized/localized perception output.
- Feed downstream path planning.
- Support the current strategy of finding a safe path through Meihua before collection.
- Bridge perception output to the planner when block-map design is ready.

### 3.5 Strategy-Level KFS Target Logic

- Add KFS Target Priority Tracking as a strategy-level selected-target mechanism.
- Add Closest KFS Selection policy only when strategy allows it.
- Define collection-priority logic.
- Define legal and reachable KFS policy.
- Keep this explicitly separate from perception-level temporal smoothing.
- Revisit only after strategy and planning are clearer.

### 3.6 Visual Servoing

- Add close-range alignment using visual feedback.
- Use selected-target feedback rather than raw perception alone.
- Consider bbox center error, bottom-center projection, bearing error, and 3D localized position.
- Integrate with arm or gripper approach behavior.
- Revisit only after target selection, arm or gripper integration, and calibration are reliable.

### 3.7 TensorRT Runtime on Jetson Nano

- Validate TensorRT environment on Jetson Nano.
- Check `trtexec`, `nvinfer`, and `NvInfer.h`.
- Build the `.engine` from ONNX on the Jetson Nano target.
- Rebuild the package with `ENABLE_TENSORRT=ON`.
- Verify runtime log shows `active=tensorrt`.
- Benchmark TensorRT against the ONNX baseline on the same hardware.
- Keep engine-portability constraints in mind because `.engine` files are environment-specific.

### 3.8 GStreamer / NVMM / Zero-Copy Optimization

- Keep `nvv4l2camerasrc`, NVMM, DMA, and zero-copy work postponed for now.
- Revisit only if Jetson TensorRT benchmarking proves camera, preprocess, or copy bottlenecks.
- Treat GStreamer/NVMM as worthwhile only if capture or copy becomes a major latency component.
- Be careful because CPU-side HSV, contour, aggregation, and debug processing may reduce zero-copy benefit.

### 3.9 Runtime Health / Diagnostics

- Add `/yolo/vision_status` or an equivalent diagnostics topic.
- Integrate vision-health state with higher-level runtime or state machine logic.
- Distinguish “no KFS detected” from “vision unhealthy”.
- Expose circuit-breaker health in a cleaner downstream-readable form.
- Keep this separate from the existing current behavior where failed frames simply skip publish.

### 3.10 Failure Recovery Extensions

- Add camera dirty detection.
- Add sudden lighting detection.
- Add adaptive lighting correction.
- Add stronger occlusion handling.
- Revisit only after more real hardware and field testing.
- Avoid overfitting before competition-environment validation.

### 3.11 Benchmarking and Performance Evaluation

- Run longer ONNX Runtime baseline benchmark sessions.
- Run TensorRT benchmark comparison once TensorRT is active on Jetson Nano.
- Add optional CSV or JSON benchmark export if useful.
- Compare FPS, mean latency, P95, P99, timeout count, busy-drop count, and circuit state.
- Base optimization decisions on measured bottlenecks rather than assumptions.

### 3.12 Documentation / Technical Report

- Update the OpenVisionABU26 Version 3 technical report to reflect the latest architecture.
- Summarize runtime architecture, message contracts, topics, and roadmap.
- Keep terminology aligned with `legal / illegal / unknown`.
- Clearly document what is implemented vs what remains future work.

No explicit future-work section was found in the current technical report file, but the report is an obvious documentation surface that should eventually be synchronized with the implemented architecture and roadmap.

## 4. Prioritization Proposal

Practical priority order based on the documented current state:

### P0 – Required before reliable 3D use

- camera calibration
- camera pose and extrinsics calibration
- validate ray direction and fix or understand `intersection_behind_camera`

### P1 – Required for Meihua strategy

- `BlockLocalizer` or block-map integration
- legal KFS map
- `block_map` plane-height source

### P2 – Performance validation

- TensorRT runtime on Jetson Nano
- TensorRT benchmark vs ONNX baseline

### P3 – Optimization only if benchmark proves bottleneck

- GStreamer, NVMM, DMA, and zero-copy optimization

### P4 – Strategy and control future

- KFS Target Priority Tracking
- Closest KFS Selection
- Visual Servoing

### P5 – Robustness extensions

- health or diagnostics topic
- camera dirty detection
- lighting and occlusion recovery extensions

## 5. Source Reference Map

- `2026-04-24_02_TeamColorFilter.md`
  - HSV calibration tooling
  - symbol-level and instance-level color interpretation context
- `2026-04-25_04_KFSInstancePrototype.md`
  - ROI and range filtering
  - contour and mask tuning background
  - debug and diagnostics history
- `2026-04-26_05_OpenVisionV3_KFSInstanceRuntime.md`
  - `BlockLocalizer` and planning bridge references
  - instance-level runtime interpretation context
- `2026-04-27_06_OpenVisionV3_KFS3DLocalizer.md`
  - camera calibration
  - fisheye vs pinhole model
  - extrinsics calibration
  - `intersection_behind_camera`
  - `block_map` plane-height future work
  - calibration tool reference
- `2026-04-27_07_OpenVisionV3_KFSLocalizationStabilizer.md`
  - `BlockLocalizer` integration
  - strategy-level target logic as future work
  - Visual Servoing as future work
- `2026-04-28_08_OpenVisionV3_RuntimeSafetyHardening.md`
  - `/yolo/vision_status` or diagnostics topic
  - vision-health/state-machine integration
  - lighting and occlusion follow-up
  - calibration resolution alignment reminder
  - future TensorRT and NVMM references
- `2026-04-29_09_OpenVisionV3_ONNXRuntimeBaselineBenchmark.md`
  - longer benchmark use
  - TensorRT comparison
  - optional CSV or JSON export
  - bottleneck-driven optimization rule
- `2026-04-29_10_OpenVisionV3_TensorRTBackendArchitecture.md`
  - TensorRT backend future validation
  - engine build
  - Jetson runtime activation
  - ONNX fallback
  - benchmark comparison metrics
- `2026-04-29_01_LogWarning.md`
  - camera calibration file follow-up
  - runtime resolution must match calibration resolution
- `2026-04-29_02_TensorRTJetsonNanoEnvironmentEngineBuildBenchmark.md`
  - Jetson Nano environment validation
  - `.engine` build on target
  - `ENABLE_TENSORRT=ON` build
  - TensorRT live benchmark
  - engine portability notes
- `2026-04-29_03_GStreamerNVMMZeroCopyOptimization.md`
  - GStreamer/NVMM/zero-copy postponement
  - benchmark-driven optimization conditions
- `2026-04-29_04_KFSTargetPriority_ClosestSelection_VisualServoing.md`
  - KFS Target Priority Tracking
  - Closest KFS Selection
  - Visual Servoing
  - strategy/planning separation

## 6. Non-Goals / Not Implemented Yet

- This file does not implement anything.
- TensorRT live runtime is not verified yet.
- GStreamer, NVMM, DMA, and zero-copy are not implemented in the main runtime.
- Visual Servoing is not implemented.
- strategy-level KFS target tracking is not implemented.
- closest-KFS selection is not implemented as an active runtime rule.
- `/yolo/vision_status` or equivalent health topic is not implemented.
- block-map integration is not implemented yet.

## 7. Maintenance Note

This file should be updated whenever a Future Work item is completed, moved into implementation, or removed from the roadmap.
