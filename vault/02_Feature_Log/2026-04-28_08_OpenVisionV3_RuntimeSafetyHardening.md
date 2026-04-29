# OpenVision-v3 – Runtime Safety Hardening

## 1. Overview

This milestone is a runtime safety hardening update for the OpenVision-v3 perception pipeline. It is not a new perception algorithm, and it does not change the semantic interpretation logic of the existing vision stack. The purpose of this work is to make the high-rate ROS 2 perception runtime more robust under load, under inference delays, and under backend failure conditions.

**Current Perception Pipeline:**
Camera/Image  
→ YOLO detection node  
→ KFSInstanceAggregator  
→ `/yolo/kfs_instances`  
→ `kfs_3d_localizer_node`  
→ `/yolo/kfs_instances_localized`  
→ `kfs_localization_stabilizer_node`  
→ `/yolo/kfs_instances_stabilized`

**Main Additions in This Milestone:**
- BEST_EFFORT / `SensorDataQoS` for realtime streams
- Inference mutex / thread lock
- Drop incoming frame when inference is busy
- Circuit breaker for inference timeout/failure
- Skip publish on failed inference frame
- Skip inference/publish when the circuit breaker is `OPEN`

**Important Scope Boundaries:**
- Existing message contracts were not changed.
- Existing topic names were not changed.
- YOLO, KFS aggregation, DecisionEngine, 3D Localizer, and Stabilizer logic were not changed.
- `FPTU_ABU_R2` state machine and older common packages were not modified.

The work is therefore best understood as runtime robustness engineering around `yolo_detection_node`, not as a perception model upgrade.

## 2. Why Runtime Safety Is Needed

The camera/image stream operates at around 30 FPS. In a realtime perception system, that means a new frame arrives roughly every 33 ms. If inference takes longer than the incoming frame period, then new frames can arrive before the previous frame has finished processing.

Without runtime safeguards, several failure modes become likely:
- Frames accumulate faster than they are consumed.
- The perception node starts processing old frames instead of the newest frame.
- End-to-end latency increases even if the node is still technically publishing.
- The robot can act on stale world state rather than current scene state.
- Inference backend stalls or exceptions can degrade the node into unstable or misleading behavior.
- Multithreaded callback execution can trigger race conditions if shared backend state is accessed concurrently.

For robot perception, the design priority is usually freshness, not perfect delivery of every image frame. A robot should prefer the newest available frame over an old queued frame that no longer reflects the current field state.

**Design Goals of This Hardening Pass:**
- Keep realtime behavior under load
- Avoid backlog and frame accumulation
- Avoid race conditions around inference/backend access
- Avoid publishing misleading empty results on inference failure
- Allow controlled recovery after repeated inference failures

## 3. QoS Policy

`yolo_detection_node` now builds its realtime stream QoS from the following parameters:

- `runtime_safety.qos.use_sensor_data_qos`
- `runtime_safety.qos.queue_depth`

When enabled, the node uses a sensor-style ROS 2 QoS profile appropriate for high-rate image and perception streams.

**Why `SensorDataQoS` / `BEST_EFFORT` is Used Here:**
- Camera and perception streams are high-frequency realtime data.
- The newest frame is more valuable than guaranteed delivery of every past frame.
- `BEST_EFFORT` helps avoid building queues of stale frames.
- Lower queue depth reduces latency and keeps the runtime closer to the current scene state.

**Observed Current QoS:**
- `/yolo/kfs_instances` uses `BEST_EFFORT`
- `/yolo/detections` uses `BEST_EFFORT`
- Durability is `VOLATILE`

This is consistent with the goal of keeping the OpenVision-v3 perception pipeline low-latency.

**Important Limitation:**
`BEST_EFFORT` should not be applied blindly to every ROS 2 topic. State, command, safety, emergency stop, health/status, and event topics typically need more reliable delivery semantics. This milestone only changes the OpenVision-v3 realtime perception streams where low latency and newest-frame preference are the main requirements.

## 4. Thread Lock / Inference Mutex

The node now includes runtime controls for inference-thread protection:

- `runtime_safety.threading.protect_inference_with_mutex`
- `runtime_safety.threading.drop_frame_when_busy`
- `runtime_safety.threading.busy_log_throttle_sec`

The key behavior is that the inference backend can be protected so only one inference call runs at a time. This reduces the risk of unsafe concurrent backend access in multithreaded ROS 2 callback execution.

**Behavior:**
- If `protect_inference_with_mutex=true`, inference-critical shared state is guarded by a mutex.
- Only one inference path is allowed to execute at a time.
- If a new frame arrives while inference is still running and `drop_frame_when_busy=true`, the incoming frame is skipped.

This is intentional. In realtime robot vision, dropping a busy frame is safer than allowing a queue to grow. A dropped frame loses one sample, but a backlog can shift the entire perception output into stale time.

**Logging Behavior:**
- Busy/drop logs are throttled with `runtime_safety.threading.busy_log_throttle_sec`.
- This prevents log spam at camera rate, especially around 30 FPS operation.

## 5. Circuit Breaker Design

The runtime hardening also adds a circuit breaker around inference execution:

- `runtime_safety.circuit_breaker.enabled`
- `runtime_safety.circuit_breaker.failure_threshold`
- `runtime_safety.circuit_breaker.success_threshold`
- `runtime_safety.circuit_breaker.reset_timeout_sec`
- `runtime_safety.circuit_breaker.inference_timeout_ms`
- `runtime_safety.circuit_breaker.log_throttle_sec`

The circuit breaker is designed to prevent repeated backend failures or stalls from continuously destabilizing the node.

### CLOSED

- Normal operating state
- Inference is allowed
- Failures are counted

### OPEN

- Repeated failures exceeded the configured threshold
- Inference is temporarily suspended
- Output publishing is skipped for those frames
- No empty results are published merely because the circuit is open

### HALF_OPEN

- After the reset timeout expires, controlled recovery attempts are allowed
- Enough successful attempts close the circuit again
- A new failure sends the circuit back to `OPEN`

**Failure/Success Rules:**
- An inference timeout or exception records a failure.
- A successful inference completed within the configured threshold records a success.

**Current Configured Thresholds:**
- `failure_threshold=5`
- `success_threshold=3`
- `reset_timeout_sec=2.0`
- `inference_timeout_ms=100.0`

The current timeout threshold is 100 ms. This is intentionally higher than the nominal 33 ms frame period of a 30 FPS stream. The goal is not to reject every temporary spike, but to catch materially unhealthy backend stalls while still tolerating short runtime variation.

## 6. Important Behavior Change: Skip Publish on Failed Inference Frame

This milestone also corrects an important runtime behavior detail.

Previously, an inference timeout could record a circuit breaker failure while the circuit was still in the `CLOSED` state, but the frame could still end up publishing empty detections or empty KFS outputs. That behavior is unsafe from a systems interpretation standpoint.

**New Desired Behavior:**
- If inference for the current frame throws an exception or exceeds the timeout, the node records a circuit breaker failure.
- That failed frame skips publish.
- The node does not publish empty `/yolo/detections`.
- The node does not publish empty `/yolo/kfs_instances`.

**Why This Matters:**
- An empty output can be interpreted downstream as "no KFS exists in this frame"
- But the actual meaning is "inference failed or timed out"
- Those meanings are not equivalent
- Until a dedicated health/status topic exists, skipping publish is safer than publishing misleading empty perception output

Also in this milestone:
- When the circuit breaker is `OPEN`, both inference and publish are skipped
- No dedicated health/status topic was added yet

This is an important semantic correction even though no ROS message definition changed.

## 7. Runtime Logs and Observations

Observed startup logs should now reflect the runtime safety configuration:

- Runtime safety QoS: `sensor_data/best_effort depth=1`
- Runtime safety threading: `protect_inference_with_mutex=true`
- Runtime safety threading: `drop_frame_when_busy=true`
- Runtime safety circuit breaker: `enabled`
- Runtime safety circuit breaker: `failure_threshold=5`
- Runtime safety circuit breaker: `success_threshold=3`
- Runtime safety circuit breaker: `reset_timeout_sec=2.00`
- Runtime safety circuit breaker: `inference_timeout_ms=100.00`

**Observed Example During Test:**
- One inference timeout/failure was observed with latency around 2499 ms
- This exceeded the configured 100 ms threshold by a large margin
- With the updated behavior, that failed frame should skip publish instead of emitting misleading empty output

**Observed Topic Behavior:**
- `/yolo/kfs_instances` topic info showed `BEST_EFFORT`
- `/yolo/detections` topic info showed `BEST_EFFORT`
- Topic rate in normal operation remained around 29.6 Hz

This suggests the safety hardening preserved nominal realtime throughput while adding defensive runtime behavior for overload and failure cases.

## 8. Camera Warnings Observed During Test

Several camera-related warnings were observed during testing, but they are not caused by the runtime safety hardening itself.

**Warnings Seen:**
- `Camera calibration file not found`
- `unknown control 'white_balance_temperature_auto'`
- `unknown control 'exposure_auto'`
- `unknown control 'focus_auto'`

**Interpretation:**
- `Camera calibration file not found` means `usb_cam` attempted to load `~/.ros/camera_info/default_cam.yaml`, but no calibration file exists yet.
- This is expected at the current stage because formal camera calibration has not been completed.
- The 3D Localizer currently uses its own configured values under `kfs_3d_localizer.camera_matrix.*` and `kfs_3d_localizer.distortion.*`.

- The unknown camera controls indicate that the current camera/driver combination does not expose those V4L2 controls.
- The camera still started successfully at `640x480` `YUYV` `30 FPS`.

**Important Calibration Note:**
- Runtime camera resolution must match the calibration/config expected by the 3D Localizer.
- If the runtime camera remains at `640x480`, then the 3D Localizer image size and calibration should eventually match `640x480`.
- If the runtime camera mode is changed back to `1280x720`, then the 3D Localizer configuration should match that runtime mode instead.

These warnings should be tracked separately as camera configuration/calibration follow-up, not as a regression caused by the runtime safety changes.

## 9. Validation Commands

```bash
cd ~/openvision_ros2_ws_v2
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select abu_yolo_ros
source install/setup.zsh

ros2 launch abu_yolo_ros yolo.launch.py

# Check runtime safety parameters
ros2 param list /yolo_detection_node | grep runtime_safety

# Check topic QoS
ros2 topic info /yolo/kfs_instances --verbose
ros2 topic info /yolo/detections --verbose

# Check rates
ros2 topic hz /yolo/kfs_instances
ros2 topic hz /yolo/kfs_instances_localized
ros2 topic hz /yolo/kfs_instances_stabilized
```

**Expected:**
- Build passes
- Runtime safety parameters exist
- `/yolo/kfs_instances` and `/yolo/detections` use `BEST_EFFORT`
- Topic rates remain near realtime
- Failed inference frames do not publish misleading empty outputs

## 10. Current Status

- Runtime safety hardening is implemented in `yolo_detection_node`.
- QoS configuration is active.
- Inference mutex and busy-frame drop are configured.
- Circuit breaker is configured and logs startup state.
- Circuit `OPEN` skips inference and publish.
- Failed inference frames should skip publish instead of publishing empty outputs.
- No message contract changes were made.
- No health/status topic exists yet.
- No `FPTU_ABU_R2` modification was made.

This milestone improves runtime robustness without changing the higher-level OpenVision-v3 perception contract.

## Relationship to Later Milestones

- This hardening work is the runtime foundation for the ONNX baseline benchmark milestone
- it also protects the later configurable backend architecture by defining failure and timeout behavior around inference execution

## 11. Future Work

The following items are identified as future work and are not implemented in this milestone:

- Dedicated `/yolo/vision_status` or diagnostics topic
- State machine integration with vision health
- Camera dirty detection
- Sudden lighting detection
- Adaptive lighting correction
- Occlusion handling
- Benchmark ONNX Runtime baseline
- TensorRT runtime benchmark after TensorRT conversion
- NVMM/zero-copy optimization only if benchmark data shows camera/copy bottlenecks

These items extend the runtime health and performance story, but they are intentionally deferred. This milestone does not claim complete failure recovery or complete runtime observability.

## 12. Suggested Commit Message

```text
Add runtime safety hardening for vision pipeline
```
