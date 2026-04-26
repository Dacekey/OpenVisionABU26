# 2026-04-24_05_OpenVisionV3_KFSInstanceRuntime

# OpenVision-v3 – KFS Instance Runtime Pipeline, Custom Message Output, and Instance-Level Decisions

## 1. Overview

OpenVision-v3 represents a fundamental shift in the perception stack, moving from symbol-level YOLO output toward structured KFS-level runtime perception. While YOLO remains the primary detection engine, the system now interprets those detections as components of physical KFS objects.

**Previous OpenVision-v2 behavior:**
- YOLO detected individual KFS symbols (REAL, FAKE, R1, R2, R3).
- DecisionEngine and TeamColorFilter operated exclusively on symbol-level detections.
- `/yolo/detections` was the main output for downstream consumers.
- KFS instance grouping was limited to offline analysis in `tools/kfs_instance_prototype.py`.

**New OpenVision-v3 behavior:**
- YOLO continues to detect individual symbols as raw input.
- **KFSInstanceAggregator** groups these symbols into KFS-level instances in real-time.
- **Instance-level TeamColorFilter** evaluates the larger KFS body crop for more reliable color matching.
- **Instance-level DecisionEngine** issues final `collect` / `avoid` / `unknown` decisions for the entire KFS object.
- Runtime publishes structured KFS instances on `/yolo/kfs_instances`.
- Specialized debug visualization is available on `/yolo/kfs_instances/image_annotated`.

## 2. Runtime Pipeline

The final runtime pipeline is designed for high-fidelity interpretation while maintaining backward compatibility:

Camera/Image  
→ YOLO symbol detection  
→ Symbol-level TeamColorFilter / DecisionEngine (kept for debug/compatibility)  
→ **KFSInstanceAggregator** (Groups symbols into clusters)  
→ **Instance-level TeamColorFilter** (Evaluates body crop)  
→ **Instance-level DecisionEngine** (Final interpretation)  
→ Publish:
  - `/yolo/detections` (Symbol-level)
  - `/yolo/image_annotated` (Symbol-level visualization)
  - `/yolo/kfs_instances` (KFS-level structured data)
  - `/yolo/kfs_instances/image_annotated` (KFS-level visualization)

**Clarification:**
- `/yolo/detections` remains unchanged as a symbol-level stream.
- `/yolo/kfs_instances` serves as the new recommended runtime contract for downstream modules (e.g., Planning, Localizer).
- Existing ROS outputs were preserved to prevent breaking legacy debug tools.

## 3. KFSInstanceAggregator Runtime Port

The offline prototype logic was ported into a dedicated C++ runtime module:
- `src/abu_yolo_ros/include/abu_yolo_ros/kfs_instance_aggregator.hpp`
- `src/abu_yolo_ros/src/kfs_instance_aggregator.cpp`

**Stable Baseline Logic Ported:**
- **Semantic Grouping:** Class-name/family-based association.
- **ROI Filtering:** Hard-coded or normalized ROI bounds to drop noisy edge detections.
- **Geometry Normalization:** Square-normalized symbol geometry for consistent grouping.
- **Clustering:** Basic spatial clustering with conservative same-group merges.
- **Low-Mask Fallback:** Ported the stable "Case 2" checkpoint logic for adjacent same-group fallback when HSV coverage is low.
- **Body Refinement:** HSV body-mask contour refinement to tighten KFS body bounding boxes.
- **Ambiguity Handling:** Dropping ambiguous clusters (e.g., overlapping different types) before they reach the decision stage.

**Intentionally Omitted Features:**
- Late Case-6 experimental ranking heuristics (too aggressive).
- Vertical-stack priors and foreground/occlusion penalties (increased complexity without significant reliability gains in representative tests).
- These were left out of the stable runtime path to prioritize deterministic behavior over overfitting noisy frames.

## 4. Custom ROS Messages

To provide a clean long-term contract, custom ROS messages were implemented:
- `src/abu_yolo_ros/msg/KfsInstance.msg`
- `src/abu_yolo_ros/msg/KfsInstanceArray.msg`

### `KfsInstance.msg` (Single KFS Object)
Represents a fully interpreted KFS:
- `cluster_id`: Unique ID for the instance in the current frame.
- `group_type`: Semantic type (REAL, FAKE, R1, etc.).
- `decision`: Final action (COLLECT, AVOID, UNKNOWN).
- `confidence`: Instance-level confidence proxy.
- `bbox`: `vision_msgs/BoundingBox2D` of the KFS body.
- `bbox_quality`: Metadata (normal, clipped, etc.).
- `symbol_indices`: Indices of symbols belonging to this cluster.
- `team_color`: Detected color (RED, BLUE, UNKNOWN).
- `team_color_match`: Boolean relative to `my_team`.
- `color_confidence` & `color_mask_coverage`: Reliability metrics.
- `ambiguous` & `ambiguous_reason`: For debugging dropped candidates.

### `KfsInstanceArray.msg` (Frame-level)
- `header`: Standard ROS 2 header.
- `team_color`: The robot's assigned team color.
- `instances[]`: List of detected KFS instances.

**Rationale:** Reusing `vision_msgs/Detection2DArray` was considered but rejected because KFS-level metadata (indices of constituent symbols, color coverage, group types) does not fit cleanly into standard messages. Custom messages provide a better contract for downstream planning logic.

## 5. New Runtime Topic

**Topic:** `/yolo/kfs_instances`  
**Type:** `abu_yolo_ros/msg/KfsInstanceArray`

**Purpose:**
- Provides a structured, "object-level" view of the world.
- Decouples raw symbol detection from high-level interpretation.
- Future-proofs the system for `BlockLocalizer` and planning modules.

## 6. Debug Visualization Topic

**Topic:** `/yolo/kfs_instances/image_annotated`  
**Type:** `sensor_msgs/msg/Image`

**Purpose:**
- Visualizes final KFS instance boxes (e.g., `[[C0]] REAL`, `[[C1]] FAKE`).
- Displays the ROI overlay to verify filtering.
- Simplifies monitoring in `rqt_image_view` by hiding raw YOLO symbol boxes.

**Configuration:**
- `kfs_instance_aggregation.publish_debug_image: true`
- `kfs_instance_aggregation.draw_roi: true`

## 7. Instance-Level TeamColorFilter

The `TeamColorFilter` was extended to support arbitrary `cv::Rect` crops. In the KFS pipeline, it evaluates the **KFS body crop** rather than just the symbol crop.

**Strategy:**
- **Primary Crop:** `refined_bbox` (tightly fit to the HSV body mask).
- **Fallback Crop:** `expanded_bbox` or `union_bbox` (used if the refined bbox is invalid or too small).
- **Rationale:** Symbol bounding boxes are often too small and dominated by sticker/symbol pixels. Evaluating the whole KFS body provides much higher color matching confidence.

**Key Metrics:**
- `team_color_match` is now the primary driver for `COLLECT` decisions.
- `color_mask_coverage` is used to filter out noise and background reflections.

## 8. Instance-Level DecisionEngine

The `DecisionEngine` now processes `KFSInstance` objects. It uses a combination of semantic group type and color matching results.

**Instance Decision Logic:**
- `FAKE` → `AVOID`
- `R1` → `AVOID`
- `AMBIGUOUS` → `UNKNOWN` (or dropped)
- `REAL` + `team_color_match` + `conf > threshold` → `COLLECT`
- `REAL` without color match → `UNKNOWN`
- Unsupported groups → `UNKNOWN`

## 9. Runtime Logging Improvements

Logging was standardized to distinguish between symbol-level and instance-level activities:
- **Symbol Logs:** Use `symbol_decision`, `symbol_final_conf`, and `symbol_reason`.
- **Instance Logs:** Use `instance_decision`, `instance_final_conf`, and `instance_reason`.
- **Labels:** Instance logs are prefixed with `[[C0]]`, `[[C1]]`, etc., to make them visually distinct from detection indices `[0]`, `[1]`.
- **Summary:** Clear counts are printed for both symbol detections and KFS instances.

## 10. Configuration Summary

Key settings in `src/abu_yolo_ros/config/yolo_detection.yaml`:

```yaml
kfs_instance_aggregation:
  enabled: true
  debug_instances: true
  publish_instances: true
  instances_topic: "/yolo/kfs_instances"
  publish_debug_image: true
  draw_roi: true
  enable_instance_team_color_filter: true
  instance_color_primary_bbox: "refined_bbox"
  instance_color_fallback_bbox: "expanded_bbox"
  hsv_profile: "competition_blue" # Default profile for ABU26
```

**Note:** `competition_blue` is the default profile. HSV fine-tuning is deferred to competition day using `tools/hsv_calibration_viewer.py`.

## 11. Validation

```bash
# Build the package with new messages
cd ~/openvision_ros2_ws_v2
colcon build --packages-select abu_yolo_ros

# Verify message definitions
source install/setup.zsh
ros2 interface show abu_yolo_ros/msg/KfsInstance

# Launch and verify topics
ros2 launch abu_yolo_ros yolo.launch.py
ros2 topic list | grep kfs
ros2 topic echo /yolo/kfs_instances --once
```