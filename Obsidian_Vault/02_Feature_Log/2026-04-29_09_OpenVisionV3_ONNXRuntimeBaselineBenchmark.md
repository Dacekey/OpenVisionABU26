# OpenVision-v3 – ONNX Runtime Baseline Benchmark

## 1. Overview

This milestone adds ONNX Runtime baseline benchmark instrumentation to `yolo_detection_node`.

OpenVision-v3 currently follows this runtime perception pipeline:

Camera/Image  
→ YOLO detection node  
→ KFSInstanceAggregator  
→ `/yolo/kfs_instances`  
→ `kfs_3d_localizer_node`  
→ `/yolo/kfs_instances_localized`  
→ `kfs_localization_stabilizer_node`  
→ `/yolo/kfs_instances_stabilized`

This work is focused on runtime performance instrumentation only.

**Important scope clarification:**
- This is not a perception algorithm change.
- This is not TensorRT conversion.
- This is not NVMM / zero-copy optimization.
- This is runtime performance instrumentation only.
- Existing ROS message/topic contracts remain unchanged.

**Why this baseline matters:**
- The current backend is ONNX Runtime.
- Before converting to TensorRT, a reliable ONNX baseline is required.
- Without a baseline, it is not possible to judge whether TensorRT actually improves runtime performance.
- Benchmark data helps identify whether the main bottleneck is inference, preprocessing, postprocessing, KFS aggregation, publishing, or another runtime stage.

## 2. Benchmark Configuration

The new benchmark configuration lives under:

`runtime_benchmark.*`

Current parameters:

```yaml
runtime_benchmark:
  enabled: true
  summary_interval_frames: 100
  warmup_frames: 20
  log_per_frame: false
  reset_after_summary: true
  include_publish_time: true
```

**Parameter meaning:**
- `enabled`
  enables or disables runtime benchmark instrumentation
- `summary_interval_frames`
  prints a benchmark summary every N measured frames
- `warmup_frames`
  skips the first N frames to reduce startup and model warmup noise
- `log_per_frame`
  if `true`, logs every frame timing; default is `false` to avoid runtime log spam
- `reset_after_summary`
  clears collected samples after each summary window
- `include_publish_time`
  includes publish timing in the benchmark summary

## 3. Measured Pipeline Stages

The benchmark currently measures these runtime stages:

- `image_callback_total_ms`
- `preprocess_ms`
- `inference_ms`
- `postprocess_ms`
- `team_color_filter_ms`
- `kfs_instance_aggregation_ms`
- `publish_ms`
- `total_ms`

**Stage meaning:**

`image_callback_total_ms`

Total time spent inside the image callback path.

`preprocess_ms`

Time spent preparing the image before inference, such as image conversion or input preparation.

`inference_ms`

Time spent inside the ONNX Runtime inference call.

`postprocess_ms`

Node-side symbol post-inference work after `detector_->infer()`.

`team_color_filter_ms`

Time spent applying symbol-level TeamColorFilter and related decision support preparation.

`kfs_instance_aggregation_ms`

Time spent running KFS instance aggregation and the current instance-color evaluation stage.

`publish_ms`

Time spent publishing detection, KFS, and debug outputs when `include_publish_time=true`.

`total_ms`

End-to-end measured runtime pipeline time for the frame.

**Important note:**
- Some boundaries are practical runtime boundaries based on the current implementation, not theoretical model-only timings.

## 4. Metrics Computed

Each measured stage computes:

- `mean_ms`
- `min_ms`
- `max_ms`
- `p50_ms`
- `p95_ms`
- `p99_ms`
- `sample_count`

The runtime benchmark also reports additional counters:

- `estimated_fps`
- `benchmark_frames_seen`
- `benchmark_frames_measured`
- `benchmark_inference_failures`
- `benchmark_timeout_count`
- `benchmark_busy_drop_count`
- current circuit breaker state

**Why P95 and P99 matter:**
- Average latency can look acceptable while occasional spikes still break realtime behavior.
- P95 and P99 expose tail latency and jitter.
- Robotics runtime needs stable latency, not only high average FPS.

## 5. Runtime Safety Interaction

This benchmark is designed to work together with the existing runtime safety layer.

Runtime safety already includes:

- `BEST_EFFORT` / `SensorDataQoS`
- inference mutex
- busy-frame drop
- circuit breaker
- skip publish on failed inference frame

The benchmark tracks:

- inference failures
- timeouts
- busy drops
- current circuit breaker state

**Behavior note:**
- Failed or skipped frames should not be treated as normal successful latency samples unless clearly marked.
- This keeps the benchmark data meaningful and avoids hiding runtime instability behind average numbers.

## 6. Observed Baseline Results

Observed runtime benchmark summaries during validation:

**Example 1**

```text
ONNX benchmark frames=100 fps=30.5 total mean=32.8ms p95=34.4ms p99=34.9ms ... dropped=0 timeout=1 failures=1 warmup_skipped=20 circuit=CLOSED
```

**Example 2**

```text
ONNX benchmark frames=100 fps=30.8 total mean=32.4ms p95=33.6ms p99=34.3ms ... dropped=0 timeout=1 failures=1 warmup_skipped=20 circuit=CLOSED
```

**Interpretation:**
- FPS was around `30.5–30.8 FPS`
- total mean latency was around `32.4–32.8 ms`
- P95 latency was around `33.6–34.4 ms`
- P99 latency was around `34.3–34.9 ms`
- dropped frames were `0`
- timeout/failure count was `1`
- circuit breaker stayed `CLOSED`
- the current runtime appears to meet the approximately `30 FPS` baseline target

The single timeout/failure should continue to be monitored, but it was not enough to open the circuit breaker in the observed validation run.

## 7. What Was Not Changed

This milestone did not change:

- YOLO inference behavior
- model outputs
- TeamColorFilter logic
- DecisionEngine logic
- KFSInstanceAggregator logic
- 3D Localizer logic
- Localization Stabilizer logic
- ROS message definitions
- topic names
- TensorRT backend
- NVMM / zero-copy pipeline

This is instrumentation only.

## 8. Validation

Validation commands:

```bash
cd ~/openvision_ros2_ws_v2
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select abu_yolo_ros
source install/setup.zsh
ros2 launch abu_yolo_ros yolo.launch.py
```

**Validation result:**
- `colcon build --packages-select abu_yolo_ros` passed
- `ros2 launch abu_yolo_ros yolo.launch.py` started normally
- benchmark config was logged at startup
- benchmark summaries appeared during runtime
- existing topics continued to publish
- no message/topic contract changes occurred

## 9. Why This Baseline Is Important Before TensorRT

This ONNX Runtime benchmark is the reference point for future TensorRT work.

Future comparison should use the same:

- model
- image resolution
- input source
- hardware
- runtime config
- test conditions
- benchmark stages

Future TensorRT evaluation should compare:

| Metric | ONNX Runtime Baseline | TensorRT Runtime | Improvement |
| --- | --- | --- | --- |
| FPS | TBD | TBD | TBD |
| total mean ms | TBD | TBD | TBD |
| inference mean ms | TBD | TBD | TBD |
| P95 latency | TBD | TBD | TBD |
| P99 latency | TBD | TBD | TBD |
| timeout count | TBD | TBD | TBD |
| busy drop count | TBD | TBD | TBD |

**Important interpretation rule:**
- TensorRT conversion should only be considered meaningful if it improves actual runtime pipeline latency, not only model-only inference timing.

## Current OpenVision-v3 Status

This log is the current ONNX Runtime baseline reference for later backend comparison.

It does not claim TensorRT performance, and it should be read as the benchmark baseline for the current default backend only.

## 10. Future Work

- Convert backend from ONNX Runtime to TensorRT Runtime
- Add TensorRT benchmark using the same benchmark structure
- Compare ONNX vs TensorRT using the same metrics: FPS, mean latency, P95/P99 latency, inference latency, total pipeline latency, and timeout/drop count
- Add optional CSV/JSON benchmark export if needed
- Consider NVMM / zero-copy optimization only if benchmark shows camera/copy bottleneck
- Continue monitoring timeout/failure count during longer tests

## Relationship to Later Milestones

- This benchmark baseline directly feeds `2026-04-29_10_OpenVisionV3_TensorRTBackendArchitecture.md`
- live TensorRT benchmark comparison remains future work on Jetson Nano

## 11. Suggested Commit Message

```bash
git commit -m "Add ONNX Runtime baseline benchmark instrumentation"
```
