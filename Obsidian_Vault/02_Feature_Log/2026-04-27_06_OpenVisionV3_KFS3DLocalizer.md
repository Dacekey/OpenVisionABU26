# OpenVision-v3 – KFS 3D Localizer Runtime Node

## 1. Overview

This milestone adds a dedicated 3D localization layer to the OpenVision-v3 perception stack. Building upon the KFS-level 2D perception established in the previous phase, the system can now project 2D KFS instances into 3D robot-frame coordinates using a ray-plane intersection strategy.

**Perception Pipeline:**
Camera/Image  
→ YOLO symbol detection  
→ KFSInstanceAggregator  
→ Instance-level TeamColorFilter  
→ Instance-level DecisionEngine  
→ `/yolo/kfs_instances` (2D)  
→ **kfs_3d_localizer_node** (3D Projection)  
→ `/yolo/kfs_instances_localized` (3D)

**Key Points:**
- `/yolo/kfs_instances` remains the primary 2D perception topic.
- `/yolo/kfs_instances_localized` is the new topic for downstream modules requiring spatial coordinates.
- Separation into a standalone node keeps geometry and calibration logic modular and decoupled from the high-frequency YOLO detection node.

## 2. New Runtime Node

A new standalone node has been implemented to handle geometric transformations and localization:

- **Source:** `src/abu_yolo_ros/src/kfs_3d_localizer_node.cpp`
- **Executable:** `ros2 run abu_yolo_ros kfs_3d_localizer_node`

**Runtime Contract:**
- **Input:** `/yolo/kfs_instances` (`abu_yolo_ros/msg/KfsInstanceArray`)
- **Output:** `/yolo/kfs_instances_localized` (`abu_yolo_ros/msg/LocalizedKfsInstanceArray`)

The node is integrated into `yolo.launch.py` to ensure it starts automatically with the vision stack. This modular approach allows for easier debugging and independent calibration of the 3D projection logic without impacting the core detector.

## Current OpenVision-v3 Status

This node currently provides the raw 3D localization stage in the runtime:

- input: `/yolo/kfs_instances`
- output: `/yolo/kfs_instances_localized`

It is integrated and publishing, but accurate metric behavior still depends on real camera calibration and reliable extrinsics.

## 3. New Custom Messages

Two new custom messages provide the interface for localized KFS data:

- `src/abu_yolo_ros/msg/LocalizedKfsInstance.msg`
- `src/abu_yolo_ros/msg/LocalizedKfsInstanceArray.msg`

### `LocalizedKfsInstance.msg` Fields:
- `source_instance`: The original 2D `KfsInstance` metadata.
- `localized`: Boolean indicating if 3D projection was successful.
- `localization_status`: `ok` or `failed`.
- `localization_quality`: Contextual status (e.g., `estimated_fixed_plane`, `out_of_range`).
- `failure_reason`: Detailed error string for debugging.
- `position_robot_mm`: Estimated (x, y, z) in robot frame (millimeters).
- `distance_mm`: Planar (2D) distance from robot origin.
- `bearing_deg`: Horizontal bearing relative to robot forward axis.
- `projection_point_mode`: The 2D point selected (e.g., `bottom_center`).
- `projection_u`, `projection_v`: Raw pixel coordinates used for the ray.
- `plane_z_height_mm`: The Z-height used for intersection.
- `plane_height_source`: Origin of the Z-height (`fixed`, `fixed_fallback_from_block_map`, etc.).
- `ray_camera_x/y/z`: Normalized ray in the camera's optical frame.
- `ray_robot_x/y/z`: Normalized ray transformed into the robot frame.

### `LocalizedKfsInstanceArray.msg`:
- `std_msgs/Header header`: Uses the robot frame (e.g., `base_link`).
- `string team_color`: Copied from the input array.
- `LocalizedKfsInstance[] instances`: List of localized objects.

## 4. 3D Localization Flow

The node executes the following geometric pipeline for every KFS instance:

1. **Receive `KfsInstanceArray`**: Triggers the calculation.
2. **Validate BBox**: Ensures the 2D box has finite, positive dimensions.
3. **Select Projection Point**:
   - Primary: `bottom_center` (base of the KFS).
   - Fallback: `center`.
4. **Clamp Projection Point**: Ensures the point stays within valid image bounds if configured.
5. **Undistort Pixel**: Efficiently undistorts only the selected pixel (not the whole image) to find the true normalized coordinates.
6. **Generate Camera Ray**: Converts the undistorted pixel into a 3D unit vector in the camera frame.
7. **Transform Ray**: Rotates the ray into the robot frame using current camera extrinsics.
8. **Select Plane Height**: Selects the target Z-height (currently defaults to `fixed`).
9. **Ray-Plane Intersection**: Solves for `t` where the ray hits the Z-plane.
   - Equation: `t = (plane_z_height_mm - origin_robot.z) / ray_robot.z`
   - Point: `point_robot = origin_robot + t * ray_robot`
10. **Validate Intersection**: 
    - Fails if `t <= 0` (`intersection_behind_camera`).
    - Fails if the point is outside configured distance/lateral bounds.
11. **Publish Result**: Emits the structured `LocalizedKfsInstanceArray`.

## 5. Distortion Model Support

The localizer supports multiple camera models to accommodate different lens types:

- **`none`**: No undistortion; useful for quick prototyping.
- **`pinhole`**: Standard OpenCV model using `k1, k2, p1, p2, k3` coefficients.
- **`fisheye`**: Optimized for wide-angle lenses like the IMX219-160, using `k1, k2, k3, k4`.

Currently, the IMX219-160 likely requires the `fisheye` model due to its high field of view. All coefficients are currently placeholders until real calibration is performed.

## 6. Camera Parameters and Configuration

The `kfs_3d_localizer` parameters are defined in `yolo_detection.yaml`:

- **Frames**: `robot_frame` (e.g., `base_link`), `camera_frame`.
- **Intrinsics**: `fx, fy, cx, cy` and distortion coefficients.
- **Extrinsics**: `x, y, z` (mm) and `roll, pitch, yaw` (deg) relative to robot origin.
- **Projection**: `primary_point`, `fallback_point`, and clamping behavior.
- **Validation**: `min_distance_mm`, `max_distance_mm`, and `max_abs_y_mm`.

**Note:** The camera pose parameters are critical for accurate localization. Placeholder values are currently used, which may result in calculation failures like `intersection_behind_camera` if the ray points away from the ground plane.

## 7. Projection Point Strategy

- **Primary: `bottom_center`**: This point represents the physical base of the KFS object where it meets the field surface. It is the most accurate point for monocular 3D projection on a flat plane.
- **Fallback: `center`**: Used if the bottom point is clipped or invalid. While less accurate for ground distance, it provides a best-effort localization.

## 8. Plane Height Strategy

The system is designed to support multiple height determination modes:

### fixed (Currently Active)
- Uses a `default_z_height_mm` (usually 0.0 for ground).
- Most stable mode for early testing.

### instance_hint (Postponed)
- Intended to guess height based on KFS type (e.g., stacked vs. single).
- Currently falls back to `fixed` with a warning to avoid instability from lighting-sensitive visual clues.

### block_map (Future Implementation)
- Will use the `BlockLocalizer` board map to look up the actual height of a known block position.
- Currently falls back to `fixed` until the map provider is implemented.

## 9. Current Runtime Observation

Tests on the skeleton implementation confirm:
- `/yolo/kfs_instances_localized` publishes at the camera rate (~29.6 Hz).
- The message contract is fully functional.
- Empty arrays are published when no KFS instances are detected.
- Some `intersection_behind_camera` failures were observed, which is expected given the placeholder camera pitch and pose. This verifies that the validation logic is working as intended.

## 10. Camera Calibration Reference Tool

A new utility `src/abu_yolo_ros/tools/calibrate_camera.py` is available to generate real intrinsics. It supports both pinhole and fisheye models and generates YAML output directly compatible with the `kfs_3d_localizer` configuration.

**Limitation**: This tool does not calibrate extrinsics.

## 11. Validation Commands

```bash
# Build and source
cd ~/openvision_ros2_ws_v2
colcon build --packages-select abu_yolo_ros
source install/setup.zsh

# Check interfaces
ros2 interface show abu_yolo_ros/msg/LocalizedKfsInstanceArray

# Run and inspect
ros2 launch abu_yolo_ros yolo.launch.py
ros2 topic hz /yolo/kfs_instances_localized
ros2 topic echo /yolo/kfs_instances_localized --once
```

## 12. Current Limitations

- **Intrinsics/Extrinsics**: All camera matrix, distortion, and pose values are placeholders.
- **Metric Accuracy**: Current output validates flow and logic, not physical accuracy.
- **Plane Height**: Only `fixed` mode is fully active.
- **Advanced Logic**: `block_map` and `instance_hint` are runtime hooks only.

## 13. Future Work

- Perform real camera calibration with the IMX219-160 at 1280x720.
- Measure and update precise camera mounting extrinsics (height, pitch, offsets).
- Debug `intersection_behind_camera` by aligning camera/robot frame conventions.
- Implement `block_map` logic once the board map module is ready.
- Connect localized output to tracking and planning modules.

## Relationship to Later Milestones

- This localized output is stabilized in `2026-04-27_07_OpenVisionV3_KFSLocalizationStabilizer.md`
- future block-map integration depends on camera calibration and board-map work that is not complete yet

## 14. Suggested Commit Message

```text
Add KFS 3D localizer with fixed-plane height fallback
```
