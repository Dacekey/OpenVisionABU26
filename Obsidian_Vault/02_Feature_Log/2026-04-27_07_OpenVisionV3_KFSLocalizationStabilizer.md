# 2026-04-27_07_OpenVisionV3_KFSLocalizationStabilizer

# OpenVision-v3 – KFS Localization Stabilizer, Lightweight Kalman Filtering, and Realtime Output Validation

## 1. Overview

This milestone adds a new KFS localization stabilizer layer to the OpenVision-v3 perception stack. Positioned immediately after the 3D Localizer, this layer provides perception-level temporal smoothing to reduce noise and jitter in the estimated 3D positions of KFS objects.

**Final Topic Flow:**
`/yolo/kfs_instances` (2D Perception)  
→ `/yolo/kfs_instances_localized` (Raw 3D Localization)  
→ **`/yolo/kfs_instances_stabilized` (Stabilized 3D Localization)**

**Key Roles:**
- **`/yolo/kfs_instances`**: Provides KFS-level 2D perception (symbol grouping, type classification).
- **`/yolo/kfs_instances_localized`**: Performs raw 3D localization via monocular ray-plane intersection.
- **`/yolo/kfs_instances_stabilized`**: Smoothes the 3D position estimates using temporal filtering.
- **Goal**: Reduce spatial jitter in the robot frame before downstream modules (BlockLocalizer, Path Planning) consume the data.

## 2. Why the Stabilizer Was Added

Monocular 3D localization from a single camera projection is inherently sensitive to small variations in 2D bounding box placement. Even minor jitter in the pixel-space detection can translate to significant jumps in the 3D world, especially as distance from the camera increases.

**Motivation:**
- **Noise Reduction**: Smoothens the frame-to-frame jitter of KFS positions.
- **Stability**: Prevents large 3D position jumps caused by small 2D bbox changes.
- **Downstream Reliability**: Provides stable coordinates for path finding through the Meihua forest and KFS localization within MH blocks.

**Clarification on Scope:**
- **Perception-Level Smoothing**: This is strictly a temporal filter for perception data.
- **No Strategy**: It does *not* decide which KFS to collect or perform closest-KFS selection.
- **No Long-Term Memory**: It does *not* remember a strategic target across long periods if it leaves the FOV.
- **Strategic Tracking**: High-level target persistence and collection priority logic are reserved for future work.

## 3. New Runtime Node

The stabilizer is implemented as a dedicated runtime node:

- **Source**: `src/abu_yolo_ros/src/kfs_localization_stabilizer_node.cpp`
- **Executable**: `ros2 run abu_yolo_ros kfs_localization_stabilizer_node`
- **Node Name**: `kfs_localization_stabilizer_node`

**Runtime Contract:**
- **Input**: `/yolo/kfs_instances_localized` (`abu_yolo_ros/msg/LocalizedKfsInstanceArray`)
- **Output**: `/yolo/kfs_instances_stabilized` (`abu_yolo_ros/msg/LocalizedKfsInstanceArray`)

**Integration:**
- Added to the main vision launch flow.
- Reuses the `LocalizedKfsInstanceArray` message type, allowing downstream consumers to seamlessly switch between raw and stabilized data.

## 4. Technique Used

The node employs a **lightweight Constant-Velocity Kalman Filter (CV-KF)** for each tracked KFS instance.

- **Kalman State**: `[x, y, vx, vy]` (Position and Velocity in robot-frame millimeters).
- **Measurement**: `[x, y]` (Raw raw localized X, Y).
- **Dynamics**: Constant velocity model ensures smoothness and provides a prediction step to handle brief occlusion or missed detections.
- **Z-Height**: The Z value is currently kept from the raw measurement (as it is typically fixed by the plane height strategy).
- **Secondary Metrics**: `distance_mm` and `bearing_deg` are automatically recomputed from the smoothed XY state.

## 5. Track Association and Gating

Since KFS IDs are not globally guaranteed, the stabilizer uses conservative association logic to link measurements to filter tracks:

- **Greedy Nearest-Neighbor**: Matches measurements to tracks based on the shortest distance.
- **Association Gate**: Controlled by `max_association_distance_mm` (default: 350mm) to prevent matching unrelated objects.
- **Jump Gate**: Controlled by `max_jump_distance_mm` (default: 600mm) to reject outliers that would corrupt the filter state.
- **Compatibility Checks**: Requires `group_type` match (e.g., REAL only matches REAL) and optional class-name overlap.

**Lifecycle Management:**
- **New Tracks**: Created for measurements that don't match existing tracks.
- **Cleanup**: Tracks are removed after `max_missed_frames` (default: 10) or if they exceed a specific age without an update.
- **Stability**: Tracks are marked as stable only after `min_hits_for_stable` (default: 2) detections.

## 6. Runtime Config

Configuration is managed under the `kfs_localization_stabilizer` namespace in `yolo_detection.yaml`:

### filter
- `process_noise_pos`: 25.0
- `process_noise_vel`: 100.0
- `measurement_noise_pos`: 100.0
- `max_dt_sec`: 0.2 (Prevents large filter jumps after long pauses)

### gating
- `max_association_distance_mm`: 350.0
- `max_jump_distance_mm`: 600.0
- `require_group_type_match`: true

### track lifecycle
- `max_missed_frames`: 10
- `min_hits_for_stable`: 2

### output behavior
- `publish_unlocalized_instances`: true (Ensures detections are still visible even if 3D localization fails)
- `pass_through_failed_instances`: true
- `append_quality_suffix`: true

## 7. Output Behavior

The stabilized output maintains the structure of the localized message but updates specific fields:

- **Smoothed Fields**: `position_robot_mm.x`, `position_robot_mm.y`, `distance_mm`, and `bearing_deg` reflect the Kalman-filtered state.
- **Pass-Through**: Failed or unlocalized instances are passed through unchanged to maintain a complete view of the detections.
- **Quality Annotation**: The `localization_quality` string is updated to indicate the filter status:
    - `stabilizing_kalman`: Filter is warming up.
    - `stabilized_kalman`: Filter is stable.
    - `raw_gating_rejected`: Measurement rejected by jump gating, using raw value.

## 8. What Was Intentionally Not Implemented

To keep the perception layer focused and modular, the following were **not** implemented in this node:
- **Closest-KFS Selection**: Choosing a primary target is a strategy-level task.
- **Collect-Priority Tracking**: High-level logic for sorting and prioritizing targets.
- **Strategy-Level Memory**: Persistent object memory across camera blind spots.
- **Visual Servoing**: Close-range alignment logic for the collection arm.

*Note: Visual servoing is identified as future work, while PnP, marker, and depth-based refinement are not part of the current OpenVision-v3 roadmap.*

## 9. Runtime Validation

Final test observations confirm the topic chain is healthy:
- Topics `/yolo/kfs_instances`, `/yolo/kfs_instances_localized`, and `/yolo/kfs_instances_stabilized` are all publishing.
- **Gating Check**: The node successfully handles cases where localization fails (e.g., `intersection_behind_camera`) by passing them through with the appropriate quality and failure reasons.
- **Stability**: Observation of the stabilized output shows significantly reduced jitter compared to the raw localized stream.

## 10. Realtime Rate Check

- **Average Rate**: ~28 Hz.
- **Consistency**: Interval standard deviation remains low, confirming the lightweight Kalman filter does not introduce significant latency or processing overhead.
- **Suitability**: The rate is well-matched to the camera and YOLO pipeline frequency.

## 11. Validation Commands

```bash
# Build and source
cd ~/openvision_ros2_ws_v2
colcon build --packages-select abu_yolo_ros
source install/setup.zsh

# Launch the full vision stack
ros2 launch abu_yolo_ros yolo.launch.py

# Verify topics
ros2 topic list | grep kfs
ros2 topic hz /yolo/kfs_instances_stabilized
ros2 topic echo /yolo/kfs_instances_stabilized --once

# Standalone check (disabled mode)
ros2 run abu_yolo_ros kfs_localization_stabilizer_node --ros-args \
  -p kfs_localization_stabilizer.enabled:=false
```

## 12. Current Status

- **Implemented**: KFS localization stabilizer node is fully operational.
- **Contract**: Reuses `LocalizedKfsInstanceArray` for easy downstream consumption.
- **Performance**: High-rate, low-latency Kalman filtering.
- **Reliability**: Robust gating and lifecycle management.
- **Readiness**: Ready for higher-level integration once 3D Localizer calibration is finalized.

## 13. Future Work

- **Camera Calibration**: Finalize intrinsics using `tools/calibrate_camera.py`.
- **Extrinsics Measurement**: Calibrate precise camera mounting pose on the robot.
- **Frame Convention**: Resolve `intersection_behind_camera` by validating camera-to-robot frame rotation and pitch sign.
- **Integration**: Connect `/yolo/kfs_instances_stabilized` to the `BlockLocalizer` and board map.
- **Strategy**: Implement high-level collection priority logic.
- **Alignment**: Consider visual servoing for precise close-range collection.

## 14. Suggested Commit Message

```text
Add KFS localization stabilizer runtime node
```
