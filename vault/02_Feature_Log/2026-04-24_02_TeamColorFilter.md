# OpenVision – TeamColorFilter Groundwork

**Date:** 2026-04-24  
**Status:** Stable groundwork later extended into OpenVision-v3  
**Workspace:** `~/openvision_ros2_ws_v2`  
**Package:** `abu_yolo_ros`

## 1. Historical Objective

This milestone originally introduced a real `TeamColorFilter` module during the v2 phase.

Initial goals at that time:

- classify detections as red, blue, or unknown using HSV filtering
- make thresholds configurable in YAML
- support runtime debugging and calibration
- avoid changing existing ROS outputs while the logic was still being validated

## 2. What Was Implemented Initially

Initial module and tooling work:

- `include/abu_yolo_ros/team_color_filter.hpp`
- `src/team_color_filter.cpp`
- YAML-configurable HSV thresholds in `config/yolo_detection.yaml`
- calibration helper tooling in `tools/hsv_calibration_viewer.py`
- debug-oriented diagnostics for coverage and color confidence

The original v2 implementation operated primarily on symbol-level YOLO crops.

## 3. How TeamColorFilter Evolved in OpenVision-v3

In OpenVision-v3, TeamColorFilter is no longer only a symbol-level helper.

It now participates in two distinct roles:

- **symbol-level evaluation**
  used for compatibility, debugging, and low-level diagnostics around raw YOLO symbol detections
- **instance-level evaluation**
  used after `KFSInstanceAggregator` groups symbols into physical KFS candidates

The current OpenVision-v3 runtime relies more heavily on instance-level color evaluation because KFS body color is more reliable than tiny symbol crops.

## 4. Current OpenVision-v3 TeamColorFilter Role

Current TeamColorFilter usage in the runtime:

- YOLO symbol detections are first interpreted at symbol level
- `KFSInstanceAggregator` groups symbol detections into KFS instances
- TeamColorFilter then evaluates the KFS instance crop
- the result feeds instance-level legality evaluation

Current instance-level crop strategy:

- **primary crop:** `refined_bbox`
- **fallback crop:** `expanded_bbox` if the refined crop is invalid, too small, or unreliable

This makes TeamColorFilter a key part of the current legal / illegal / unknown decision flow, but it is still not the final strategy layer.

## 5. Terminology Update

Older v2 development often described the next step as producing:

- `collect`
- `avoid`
- `unknown`

Current OpenVision-v3 terminology is:

- `legal`
- `illegal`
- `unknown`

Important clarification:

- TeamColorFilter helps estimate whether a detected KFS matches the robot team color
- it supports legality interpretation
- it does **not** decide final robot action such as collection, path choice, or arm behavior

Those action decisions are deferred to downstream planning and control.

## 6. Current Technical Behavior

Current TeamColorFilter design characteristics:

- HSV thresholds remain YAML-configurable
- symbol-level and instance-level crops are both supported
- color confidence and mask coverage remain useful diagnostics
- instance-level color evaluation is designed to support KFS legality interpretation

Known runtime usage patterns:

- symbol-level color is still useful for debug visibility and compatibility
- instance-level color is preferred for final KFS interpretation
- color matching is combined with semantic KFS type and confidence thresholds in the DecisionEngine

## 7. HSV Profiles and Calibration Notes

Current OpenVision-v3 documentation and config convention include HSV profiles such as:

- `competition_blue`
- debug-only variants such as `dark_blue`

These profiles are tuning aids, not separate perception algorithms.

Calibration principles that still remain valid:

- keep thresholds configurable in YAML
- use debug tools for tuning
- avoid auto-overwriting known-good config during field iteration

## 8. Current OpenVision-v3 Status

TeamColorFilter is now part of a larger KFS-level runtime rather than a standalone symbol-only feature.

Its place in the current pipeline:

Camera/Image  
-> YOLO symbol detection  
-> KFSInstanceAggregator  
-> instance-level TeamColorFilter  
-> instance-level DecisionEngine  
-> `/yolo/kfs_instances`

This means the feature should now be understood as groundwork that later matured into KFS-level color evaluation.

## 9. Relationship to Later Milestones

This groundwork later evolved into:

- `2026-04-24_03_DecisionEngine.md`
- `2026-04-25_04_KFSInstancePrototype.md`
- `2026-04-26_05_OpenVisionV3_KFSInstanceRuntime.md`

The later OpenVision-v3 runtime milestone is where TeamColorFilter became part of:

- instance-level KFS interpretation
- `refined_bbox` and `expanded_bbox` crop selection
- legal / illegal / unknown runtime output semantics

## 10. Validation History

Validation from the original phase still remains relevant:

- `colcon build --packages-select abu_yolo_ros` passed
- red and blue threshold tuning was validated in the development environment
- diagnostics and calibration tooling improved repeatability

## 11. Final Interpretation

This log preserves the v2-origin history of TeamColorFilter, but its current meaning inside OpenVision-v3 is broader:

- not just symbol-level color classification
- not direct collection strategy
- now a supporting feature for instance-level KFS legality interpretation
