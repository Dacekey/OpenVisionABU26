# OpenVision – V2 Refactor and Workspace Migration

**Date:** 2026-04-24  
**Status:** Completed  
**Workspace:** `~/openvision_ros2_ws_v2`

## 1. Historical Objective

This log originally recorded the migration of the latest `abu_yolo_ros` v2 refactor work from an older experimental workspace into a clean Git-cloned workspace.

At that time, the goal was to preserve and continue development of:

- YOLO symbol detection
- early TeamColorFilter groundwork
- early DecisionEngine groundwork
- a cleaner ROS 2 package layout for continued iteration

## 2. Migration Scope at the Time

Original workspace movement:

- **Source:** `openvision_ros2_ws/src/abu_yolo_ros`
- **Target:** `openvision_ros2_ws_v2/src/abu_yolo_ros`

Key files and modules moved during that refactor period included:

- `include/abu_yolo_ros/detection_types.hpp`
- `include/abu_yolo_ros/team_color_filter.hpp`
- `include/abu_yolo_ros/decision_engine.hpp`
- `include/abu_yolo_ros/block_localizer.hpp`
- `include/abu_yolo_ros/temporal_tracker.hpp`
- `CMakeLists.txt`
- `src/yolo_detection_node.cpp`
- `src/yolo_detector.cpp`
- `include/abu_yolo_ros/yolo_detector.hpp`
- `config/yolo_detection.yaml`

## 3. Why This Migration Still Matters

Although this milestone was recorded during the older v2 phase, it became the starting point for what later evolved into OpenVision-v3.

That refactor created the workspace and package base that later enabled:

- KFS-level aggregation
- custom KFS message contracts
- 3D localization
- localization stabilization
- runtime safety hardening
- benchmark instrumentation
- configurable backend architecture with ONNX Runtime default and TensorRT fallback

## 4. Current OpenVision-v3 Pipeline

The project has since evolved far beyond the original v2 symbol-only flow.

Current OpenVision-v3 runtime pipeline:

Camera/Image  
-> YOLO symbol detection  
-> symbol filtering  
-> KFSInstanceAggregator  
-> instance-level TeamColorFilter  
-> instance-level DecisionEngine  
-> `/yolo/kfs_instances`  
-> KFS 3D Localizer  
-> `/yolo/kfs_instances_localized`  
-> KFS Localization Stabilizer  
-> `/yolo/kfs_instances_stabilized`

Later milestones also added:

- runtime safety hardening
- ONNX Runtime baseline benchmark instrumentation
- configurable ONNX Runtime / TensorRT backend architecture

## 5. Terminology Update

The terminology used during the original v2 phase has since been clarified.

Older action-oriented wording:

- `collect`
- `avoid`
- `unknown`

Current OpenVision-v3 vision-level wording:

- `legal`
- `illegal`
- `unknown`

Important clarification:

- `legal / illegal / unknown` are perception-level interpretation labels
- direct robot action such as collection, skipping, routing, or grasp behavior is now considered downstream strategy or control logic

## 6. Current OpenVision-v3 Status

This migration log should now be read as the foundation milestone, not as a description of the final runtime architecture.

Current role of this milestone in the full sequence:

- it established the clean workspace used by later OpenVision-v3 work
- it preserved the early symbol-level modules that later evolved into KFS-level runtime interpretation
- it is historical groundwork rather than a complete description of the current system

## 7. Relationship to Later Milestones

This migration milestone later evolved into the following OpenVision-v3 sequence:

- `2026-04-25_04_KFSInstancePrototype.md`
- `2026-04-26_05_OpenVisionV3_KFSInstanceRuntime.md`
- `2026-04-27_06_OpenVisionV3_KFS3DLocalizer.md`
- `2026-04-27_07_OpenVisionV3_KFSLocalizationStabilizer.md`
- `2026-04-28_08_OpenVisionV3_RuntimeSafetyHardening.md`
- `2026-04-29_09_OpenVisionV3_ONNXRuntimeBaselineBenchmark.md`
- `2026-04-29_10_OpenVisionV3_TensorRTBackendArchitecture.md`

## 8. Validation at the Time

- `colcon build --packages-select abu_yolo_ros` passed in the migrated workspace
- the refactor preserved buildability and allowed continued incremental development

## 9. Final Interpretation

This file preserves the historical record of the workspace migration, but the current OpenVision-v3 architecture should be understood through the later milestones rather than through this log alone.
