# OpenVision-v3 – yolo_detection.yaml Configuration Guide

## 1. Purpose

`src/abu_yolo_ros/config/yolo_detection.yaml` is the main runtime configuration file for OpenVision-v3.

It controls:

- camera/image input assumptions
- YOLO backend selection
- detection thresholds
- team color filtering
- KFS instance aggregation
- KFS instance output and debug image behavior
- KFS 3D localization
- KFS localization stabilizer
- runtime safety
- runtime benchmark
- TensorRT / ONNX backend selection

This file is not a calibration result by itself. Some values currently act as placeholders until real camera calibration, camera pose calibration, and target-hardware validation are completed.

## 2. Quick Configuration Map

| Config Group | Main Purpose | Tuning / Calibration Status | Related Tool |
|---|---|---|---|
| `model_path`, `class_names_path` | Select YOLO model and class-label order | Update only when model/export changes | validation dataset, runtime inspection |
| `inference.*` | Select ONNX vs TensorRT backend and backend-specific model/engine settings | Runtime selection is tunable; TensorRT depends on target environment | `trtexec`, runtime benchmark logs |
| `conf_threshold`, `inference.nms_threshold` | Control symbol detection acceptance and duplicate suppression | Tunable with validation images | runtime logs, test image dataset |
| `team_color_*`, `hsv_profile` | HSV-based team-color evidence for symbol and instance evaluation | Tunable in real lighting | `hsv_calibration_viewer.py` |
| `r1_conf_threshold`, `real_conf_threshold`, `fake_conf_threshold`, decision weights | KFS legality decision thresholds | Tunable after confidence distribution is understood | `tune_decision_thresholds.py` |
| `kfs_instance_aggregation.*` | Convert symbol detections into KFS-level instances | Sensitive tuning area; test with representative images | `kfs_instance_prototype.py`, debug image topic |
| KFS instance publish/debug params | Publish `/yolo/kfs_instances` and debug overlay image | Safe to toggle for debugging | `rqt_image_view`, `rqt_graph` |
| `kfs_3d_localizer_node.*` | Monocular 2D-to-3D localization | Requires calibration and robot measurement | `calibrate_camera.py`, manual measurement |
| `kfs_localization_stabilizer_node.*` | Temporal smoothing of localized KFS positions | Tune after localization is reliable | runtime logs, topic echo/hz |
| `runtime_safety.*` | Prevent backlog and repeated failure loops | Tunable after runtime behavior is observed | ROS logs, `ros2 topic hz` |
| `runtime_benchmark.*` | Runtime timing summaries and benchmark logging | Safe to tune for benchmark sessions | runtime logs |

## 3. Inference Backend Configuration

Important parameters:

- `inference.backend`
- `inference.fallback_backend`
- `inference.allow_fallback`
- `inference.onnx.model_path`
- `inference.onnx.use_gpu`
- `inference.tensorrt.engine_path`
- `inference.tensorrt.input_name`
- `inference.tensorrt.output_name`
- `inference.tensorrt.input_width`
- `inference.tensorrt.input_height`
- `inference.tensorrt.fp16`
- `inference.tensorrt.workspace_size_mb`
- `inference.tensorrt.allow_engine_rebuild`
- `inference.tensorrt.onnx_model_path_for_build`
- `inference.nms_threshold`

What this group does:

- `ONNX Runtime` is the default stable backend.
- `TensorRT` is an optional backend for loading a serialized `.engine`.
- The detector selects the requested backend and can fall back to `onnxruntime` if enabled.
- Backend switching is runtime-configurable through `inference.backend` or the launch override.

Important behavior:

- TensorRT requires a TensorRT-capable build and runtime environment.
- `allow_engine_rebuild=true` is only a future hook right now. The current TensorRT backend warns that rebuild is not implemented in-process.
- `input_width` and `input_height` must match the actual model or engine input shape.
- `input_name` and `output_name` matter for TensorRT engine bindings.

Tuning / usage guidance:

- `backend`: safe runtime selection parameter.
- `fallback_backend`: normally leave as `onnxruntime`.
- `model_path`: change only when switching to a different ONNX model.
- `engine_path`: must point to a valid `.engine` built externally.
- `fp16`: depends on target hardware support and engine build.
- `workspace_size_mb`: TensorRT performance/build parameter; relevant only on TRT-capable systems.
- `inference.nms_threshold`: tunable, but it changes duplicate suppression behavior and therefore affects detection grouping downstream.

Related tools:

- `trtexec` for `.engine` generation
- runtime benchmark logs for ONNX vs TensorRT comparison

## 4. Detection Thresholds and Class Names

Important parameters:

- `conf_threshold`
- `inference.nms_threshold`
- `class_names_path`
- model input shape parameters under `inference.onnx.*` and `inference.tensorrt.*`

What they mean:

- `conf_threshold` controls which YOLO symbol detections are accepted before later logic.
- `inference.nms_threshold` controls duplicate bounding-box suppression.
- `class_names_path` must match the exported model class order exactly.
- Backend input dimensions must match the model export and, for TensorRT, the generated engine.

Tuning guidance:

- `conf_threshold` is a normal tuning parameter.
- `nms_threshold` should be changed carefully because it influences how many overlapping symbol detections survive into KFS aggregation.
- `class_names_path` and class order should not be edited casually.

Related tools:

- validation image set
- runtime logs
- `tune_decision_thresholds.py` for downstream decision analysis, not raw YOLO training

## 5. TeamColorFilter Configuration

Important parameters:

- `team_color`
- `team_color_red_h1_low`, `team_color_red_h1_high`
- `team_color_red_h2_low`, `team_color_red_h2_high`
- `team_color_red_s_low`, `team_color_red_v_low`
- `team_color_blue_h_low`, `team_color_blue_h_high`
- `team_color_blue_s_low`, `team_color_blue_v_low`
- `team_color_min_coverage_ratio`
- `team_color_confidence_scale`
- `team_color_min_match_confidence`
- `kfs_instance_aggregation.hsv_profile`
- `kfs_instance_aggregation.dark_blue_*`

What this group does:

- The TeamColorFilter produces HSV-based team-color evidence.
- It is used at symbol level and at instance level.
- In the KFS runtime path, instance-level TeamColorFilter evaluates the refined bbox first, then a fallback bbox if needed.
- It supports legality classification, but it does not decide final robot action.

Practical notes:

- `team_color` selects the team the robot is trying to interpret.
- The red and blue HSV thresholds define color masks.
- `min_coverage_ratio`, `confidence_scale`, and `min_match_confidence` affect how strict the color match is.
- `hsv_profile` inside aggregation affects the body-mask / contour side of KFS grouping. The current config uses `competition_blue`.
- `dark_blue_*` is present as an auxiliary profile/debug hook and should be treated carefully to avoid overfitting.

Tuning guidance:

- HSV thresholds are expected tuning parameters.
- Tune under realistic lighting and camera exposure.
- Avoid tuning from a single frame or a single scene.
- Recheck after camera exposure, white balance, or venue lighting changes.

Related tool:

- `src/abu_yolo_ros/tools/hsv_calibration_viewer.py`

## 6. DecisionEngine Configuration

Important parameters:

- `r1_conf_threshold`
- `real_conf_threshold`
- `fake_conf_threshold`
- `collect_min_confidence`
- `yolo_confidence_weight`
- `color_confidence_weight`
- `require_team_color_match`
- `unknown_on_low_confidence`

What this group does:

- The DecisionEngine outputs legality-oriented labels:
  - `legal`
  - `illegal`
  - `unknown`
- It no longer represents direct robot actions like `collect` or `avoid`.
- Collection strategy remains downstream.

Current behavior summary:

- `FAKE` and `R1` resolve to `illegal`.
- `REAL` requires strong enough confidence and team-color evidence to become `legal`.
- Weak, missing, or uncertain evidence tends to remain `unknown`.
- At symbol level, the runtime is stricter in some cases and may mark certain unsupported/no-color situations as `illegal`.

Tuning guidance:

- Thresholds can be tuned, but only after confidence distributions are understood from a dataset or runtime logs.
- `yolo_confidence_weight` and `color_confidence_weight` affect how visual evidence is combined.
- `require_team_color_match` is semantically important and should not be changed casually.
- `unknown_on_low_confidence` changes how borderline cases are exposed downstream.

Related tools:

- `src/abu_yolo_ros/tools/tune_decision_thresholds.py`
- runtime logs
- validation dataset

## 7. KFS Instance Aggregation Configuration

Important parameters:

- `kfs_instance_aggregation.enabled`
- `enable_roi_filter`
- `roi_x_min_norm`, `roi_x_max_norm`, `roi_y_min_norm`, `roi_y_max_norm`
- `min_symbol_height_px`, `min_symbol_area_px`, `min_confidence`
- `use_square_symbol_bbox`
- `square_symbol_scale`
- `max_square_side_ratio_of_image`
- `expand_scale_x`, `expand_scale_y`
- `max_expanded_area_ratio`
- `cluster_distance_scale`
- `min_cluster_distance_px`, `max_cluster_distance_px`
- `min_height_similarity_for_grouping`
- `min_area_similarity_for_grouping`
- `max_bottom_y_diff_ratio`
- `enable_basic_merge`
- `merge_same_group_only`
- `max_symbols_after_merge`
- `min_expanded_iou`
- `min_expanded_intersection_over_min_area`
- `max_expanded_gap_px`
- `min_color_mask_coverage_for_merge`
- `enable_low_mask_adjacent_same_group_fallback`
- `low_mask_*`
- `max_symbols_per_instance`
- `drop_ambiguous_clusters`
- contour/body-mask params such as `contour_enabled`, `min_contour_area_px`, `morphology_kernel_size`, `contour_selection`
- neighbor protection params such as `neighbor_aware_clamp`, `protect_other_cluster_symbols`, `neighbor_protection_bbox_source`

What this group does:

- The aggregator converts symbol detections into KFS-level instances.
- It uses geometry, clustering, merge rules, body-mask/contour support, and conservative ambiguity handling.
- It is one of the most sensitive configuration areas in the stack.

Important behavior:

- ROI and bbox-size filtering reject implausible symbol detections before grouping.
- Square-normalized symbol bbox logic helps make grouping less sensitive to non-square raw symbol detections.
- Body-mask and contour refinement help estimate a more realistic KFS region.
- Merge logic can combine nearby compatible symbols into one KFS instance.

Ambiguous handling:

- Ambiguous detection and ambiguous dropping are different.
- `max_symbols_per_instance` controls the “too many symbols” ambiguity threshold.
- If a cluster is ambiguous and `drop_ambiguous_clusters=false`, the instance can still be published with:
  - `ambiguous=true`
  - `ambiguous_reason`
  - `decision=unknown`
- If `drop_ambiguous_clusters=true`, ambiguous clusters are removed from final output.

Current caveat:

- Mixed semantic groups are not fully promoted to `AMBIGUOUS` by a dedicated semantic-mixing rule. Current logic is more conservative and rule-based than fully semantic-aware.

Tuning guidance:

- ROI and symbol-size filters are normal tuning parameters for a specific camera setup.
- Clustering and merge thresholds are sensitive and should be tested with representative images, not guessed.
- `drop_ambiguous_clusters` can be adjusted depending on whether the team prefers strict production output or richer debugging visibility.

Related tools:

- `src/abu_yolo_ros/tools/kfs_instance_prototype.py`
- `src/abu_yolo_ros/tools/config/kfs_instance_prototype.yaml`
- `/yolo/kfs_instances/image_annotated`

## 8. KFS Instance Output and Debug Image Configuration

Important parameters:

- `kfs_instance_aggregation.publish_instances`
- `kfs_instance_aggregation.instances_topic`
- `kfs_instance_aggregation.debug_instances`
- `kfs_instance_aggregation.publish_debug_image`
- `kfs_instance_aggregation.debug_image_topic`
- `kfs_instance_aggregation.draw_roi`
- root-level `publish_detections`
- root-level `debug_detections`

What this group does:

- `/yolo/kfs_instances` is the main KFS-level 2D output contract.
- `/yolo/kfs_instances/image_annotated` is the debug visualization path.
- Detection-level annotated output remains useful for compatibility and debugging.

Tuning guidance:

- Safe to enable or disable debug publishing depending on runtime load.
- Debug image drawing adds overhead and should be disabled in strict benchmark runs if not needed.
- Topic names should not be changed casually because downstream tooling may depend on them.

Related tools:

- `rqt_image_view`
- `rqt_graph`
- `ros2 topic echo`
- `ros2 topic hz`

## 9. KFS 3D Localizer Configuration

Important parameters:

- `kfs_3d_localizer_node.enabled`
- `input_topic`, `output_topic`
- `robot_frame`, `camera_frame`
- `image.width`, `image.height`
- `camera_matrix.fx`, `camera_matrix.fy`, `camera_matrix.cx`, `camera_matrix.cy`
- `distortion.model`
- `distortion.pinhole_coeffs`
- `distortion.fisheye_coeffs`
- `camera_pose_robot.x_mm`, `y_mm`, `z_mm`
- `camera_pose_robot.roll_deg`, `pitch_deg`, `yaw_deg`
- `projection.primary_point`
- `projection.fallback_point`
- `projection.bottom_center_y_offset_ratio`
- `plane.mode`
- `plane.default_z_height_mm`
- `plane.valid_z_heights_mm`
- `plane.block_map.*`
- `plane.instance_hint.*`
- `validation.min_distance_mm`
- `validation.max_distance_mm`
- `validation.max_abs_y_mm`
- `validation.reject_if_ray_parallel`

What this group does:

- This node converts a 2D KFS bbox projection into a robot-frame 3D estimate.
- The runtime selects a projection pixel, undistorts it, builds `ray_camera`, transforms it into `ray_robot`, and intersects it with the configured plane height.

Important behavior:

- Supported distortion modes are `none`, `pinhole`, and `fisheye`.
- `projection.primary_point` and `fallback_point` determine which image point represents the KFS for the ray-plane intersection.
- The currently active practical plane mode is `fixed`.
- `block_map` is the preferred future hook for plane-height selection.
- `instance_hint` exists in config but is explicitly postponed in the current implementation.

Calibration guidance:

- `image.width` and `image.height` must match the actual runtime camera resolution.
- `fx`, `fy`, `cx`, `cy` must come from camera calibration.
- Distortion coefficients must come from camera calibration.
- `camera_pose_robot.*` must come from robot measurement and camera mounting calibration.
- Plane height strategy should later align with the Meihua block map.

Related tools:

- `src/abu_yolo_ros/tools/calibrate_camera.py`
- `src/abu_yolo_ros/tools/config/camera_calibration_example.yaml`
- manual robot/camera mounting measurement

## 10. KFS Localization Stabilizer Configuration

Important parameters:

- `kfs_localization_stabilizer_node.enabled`
- `input_topic`, `output_topic`
- `filter.model`
- `filter.process_noise_pos`
- `filter.process_noise_vel`
- `filter.measurement_noise_pos`
- `filter.initial_position_variance`
- `filter.initial_velocity_variance`
- `filter.max_dt_sec`
- `filter.min_dt_sec`
- `gating.max_association_distance_mm`
- `gating.max_jump_distance_mm`
- `gating.require_group_type_match`
- `gating.require_decision_compatibility`
- `gating.min_class_name_overlap`
- `lifecycle.max_missed_frames`
- `lifecycle.max_track_age_sec_without_update`
- `lifecycle.min_hits_for_stable`
- `output.publish_unlocalized_instances`
- `output.pass_through_failed_instances`
- `output.append_quality_suffix`

What this group does:

- The stabilizer smooths localized KFS positions over time.
- The current implementation uses a lightweight constant-velocity Kalman-style model, exposed as `kalman_cv_2d`.
- Gating prevents unlikely updates from being fused into an existing track.

Important clarification:

- This is perception-level temporal smoothing only.
- It is not closest-KFS selection.
- It is not strategy-level KFS Target Priority Tracking.

Tuning guidance:

- Tune process noise, measurement noise, and gating only after the 3D localizer is reasonably calibrated.
- `max_association_distance_mm` and `max_jump_distance_mm` should reflect expected measurement jitter and robot motion.
- Output policy flags are safe to adjust depending on whether downstream prefers conservative or passthrough behavior.

Related tools:

- no dedicated standalone tuning tool was found
- runtime logs
- `ros2 topic echo`
- `ros2 topic hz`

## 11. Runtime Safety Configuration

Important parameters:

- `runtime_safety.qos.use_sensor_data_qos`
- `runtime_safety.qos.queue_depth`
- `runtime_safety.threading.protect_inference_with_mutex`
- `runtime_safety.threading.drop_frame_when_busy`
- `runtime_safety.threading.busy_log_throttle_sec`
- `runtime_safety.circuit_breaker.enabled`
- `runtime_safety.circuit_breaker.failure_threshold`
- `runtime_safety.circuit_breaker.success_threshold`
- `runtime_safety.circuit_breaker.reset_timeout_sec`
- `runtime_safety.circuit_breaker.inference_timeout_ms`
- `runtime_safety.circuit_breaker.log_throttle_sec`

What this group does:

- It prevents latency buildup and repeated backend failure loops.
- `SensorDataQoS` / `BEST_EFFORT` is used for realtime perception behavior when enabled.
- Frame-drop-on-busy prevents backlog from growing.
- The circuit breaker protects runtime stability after repeated failures or timeouts.
- Failed inference frames are skipped instead of publishing misleading empty results.

Tuning guidance:

- `queue_depth=1` is appropriate for latest-frame behavior and should not be changed casually.
- `inference_timeout_ms` should be tuned only after measuring actual latency.
- `failure_threshold`, `success_threshold`, and `reset_timeout_sec` control robustness vs sensitivity.

Related tools:

- runtime benchmark logs
- ROS logs
- `ros2 topic hz`

## 12. Runtime Benchmark Configuration

Important parameters:

- `runtime_benchmark.enabled`
- `runtime_benchmark.summary_interval_frames`
- `runtime_benchmark.warmup_frames`
- `runtime_benchmark.log_per_frame`
- `runtime_benchmark.reset_after_summary`
- `runtime_benchmark.include_publish_time`

What this group does:

- It measures runtime pipeline timing and prints summary statistics.
- It is intended for ONNX baseline measurement now and later TensorRT comparison.
- Backend name is included in benchmark summaries.

Measured stages are intended to cover:

- preprocess
- inference
- postprocess
- team-color evaluation
- KFS aggregation
- publish
- total

Tuning guidance:

- Increase `summary_interval_frames` for more stable summaries.
- Keep `log_per_frame=false` for normal use to avoid log spam.
- `warmup_frames` should be large enough to avoid startup bias.

Related tools:

- runtime logs
- future CSV/JSON export was not found as implemented yet

## 13. Tools and Calibration/Tuning Matrix

| Tool | What It Tunes / Calibrates | Related Config |
|---|---|---|
| `hsv_calibration_viewer.py` | HSV color ranges and coverage intuition | `team_color_*`, aggregation HSV/body-mask thresholds |
| `calibrate_camera.py` | camera intrinsics and distortion coefficients | `camera_matrix.*`, `distortion.*` |
| `kfs_instance_prototype.py` | aggregation geometry, range filtering, clustering, contour refinement, merge behavior | `kfs_instance_aggregation.*` |
| `tune_decision_thresholds.py` | legality threshold recommendations from data | DecisionEngine thresholds and weights |
| `trtexec` | TensorRT engine build | `inference.tensorrt.engine_path`, engine compatibility |
| `ros2 topic hz` | runtime topic rate observation | runtime performance and output rate |
| `rqt_image_view` | visual debug of annotated images | debug image topics |
| `rqt_graph` | graph inspection | node/topic integration and launch validation |

## 14. Parameters That Should Not Be Changed Casually

The following should be treated as high-risk or calibration-sensitive:

- class names order
- model input shape
- TensorRT engine path and input size without rebuilding the engine
- camera intrinsics
- distortion coefficients
- camera pose / extrinsics
- semantic legality mapping
- topic names used by downstream consumers

Why:

- Class-order mismatch breaks semantic interpretation immediately.
- Model-shape mismatch can break inference or silently degrade output quality.
- TensorRT engines are shape- and environment-specific.
- Intrinsics, distortion, and extrinsics directly control 3D localization correctness.
- Semantic legality mapping affects downstream planning meaning.
- Topic-name changes break contracts across nodes, launch files, and debug workflows.

## 15. Recommended Tuning Order

1. Verify actual runtime camera resolution.
2. Tune HSV ranges only if needed and under realistic lighting.
3. Tune YOLO confidence / NMS only with validation images.
4. Validate KFS aggregation with the prototype tool and annotated debug image.
5. Run camera calibration for intrinsics and distortion.
6. Measure or calibrate camera extrinsics on the robot.
7. Validate ray-plane behavior in the 3D localizer.
8. Tune the stabilizer only after localization is reasonably reliable.
9. Benchmark ONNX Runtime baseline.
10. Benchmark TensorRT on Jetson later.
11. Consider NVMM / zero-copy only if benchmark evidence shows a real bottleneck.

## 16. Current Known Placeholders / Future Hooks

Current non-final areas include:

- camera calibration values may still be placeholders
- camera pose / extrinsics may still be placeholders
- `plane.mode=fixed` is the active practical mode
- `plane.block_map.*` is a future integration path
- `plane.instance_hint.*` is present but postponed
- TensorRT live runtime depends on Jetson / TensorRT environment validation
- runtime health/status topic is future work
- GStreamer / NVMM / zero-copy is future work only

## 17. Useful Commands

```bash
# Build
cd ~/openvision_ros2_ws_v2
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select abu_yolo_ros
source install/setup.zsh

# Run
ros2 launch abu_yolo_ros yolo.launch.py

# Check config params
ros2 param list /yolo_detection_node | grep runtime_safety
ros2 param list /yolo_detection_node | grep runtime_benchmark

# Check topics
ros2 topic list | grep yolo
ros2 topic echo /yolo/kfs_instances --once
ros2 topic echo /yolo/kfs_instances_localized --once
ros2 topic echo /yolo/kfs_instances_stabilized --once
ros2 topic hz /yolo/kfs_instances
```

## 18. Maintenance Note

Update this guide whenever `src/abu_yolo_ros/config/yolo_detection.yaml` changes significantly, especially when:

- a new runtime node is added
- parameter names or semantics change
- camera calibration becomes finalized
- block-map integration becomes real
- TensorRT target-hardware validation is completed

