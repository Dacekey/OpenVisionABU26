# Feature Log: DecisionEngine Implementation

**Date:** 2026-04-24
**Status:** Stable / Logic Verified
**Workspace:** ~/openvision_ros2_ws_v2
**Package:** abu_yolo_ros

---

## 1. Objective
- Add a DecisionEngine module to OpenVisionABU26 v2.
- Convert YOLO detections and TeamColorFilter results into internal decisions: `COLLECT`, `AVOID`, or `UNKNOWN`.
- Maintain existing ROS output message contracts (Detection2DArray) for stability.
- Utilize YAML-configurable thresholds for risk-based tuning without recompilation.

## 2. Implemented Changes
- **Module Implementation:**
  - `include/abu_yolo_ros/decision_engine.hpp`
  - `src/decision_engine.cpp`
- **Data Structures:**
  - Added `KFSDecision` enum: `COLLECT`, `AVOID`, `UNKNOWN`.
  - Added `DecisionEngineConfig` for per-class thresholds and confidence weighting.
  - Added `DecisionResult` fields: `decision`, `reason`, `final_confidence`.
- **Node Integration:**
  - Wired into `yolo_detection_node.cpp` following `TeamColorFilter` execution.
  - Implemented rich debug logging: `class_id`, `class_name`, color result, decision, and reason.
- **Configuration & Tests:**
  - Added parameters and explanatory comments to `config/yolo_detection.yaml`.
  - Added unit tests for key logic cases in `test/decision_engine_test.cpp`.

## 3. Important Bug Found and Fixed
- **The Issue:** Initial implementation used hard-coded `class_id` ranges (e.g., 0=R1, 1-15=REAL) for classification.
- **The Risk:** Actual model label indices did not match these assumptions (e.g., `REAL_3` was mapped to 24, incorrectly falling into the `FAKE` range).
- **The Fix:** Refactored `DecisionEngine` to use resolved semantic `class_name` as the source of truth for all logic. `class_id` is now preserved strictly for technical traceability.

## 4. Current Decision Logic
- **R1:** Always `AVOID` (Not the target for R2).
- **FAKE_***: Always `AVOID`.
- **REAL_***:
  - `COLLECT` if team color matches AND `final_confidence` >= `collect_min_confidence`.
  - `AVOID` if team color is wrong/unknown.
- **Confidence Handling:** `UNKNOWN` if `unknown_on_low_confidence` is enabled and thresholds aren't met.
- **Safety Default:** Invalid or unknown `class_name` defaults to `AVOID`.
- **Confidence Calculation:** 
  `final_confidence = (yolo_weight * yolo_conf) + (color_weight * color_conf)`

## 5. Current Risk-Based Default Config
- `r1_conf_threshold`: 0.55
- `real_conf_threshold`: 0.60 (Higher required for collection)
- `fake_conf_threshold`: 0.45 (Lower to trigger `AVOID` early for safety)
- `collect_min_confidence`: 0.60
- `yolo_confidence_weight`: 0.60
- `color_confidence_weight`: 0.40
- `require_team_color_match`: true
- `unknown_on_low_confidence`: true

## 6. Runtime Behavior
- Runs internally after inference and color filtering.
- **Encapsulation:** No changes to ROS topic names, message types, or annotated image output.
- **Validation Tool:** Currently serves as a high-signal logic validator through terminal logs before being exposed to custom messages.

## 7. Rulebook / Phase Scope
- **Current Scope:** Meihua Forest KFS collection phase (R1/R2 retrieval vs. Fakes).
- **Future Scope:** Arena Tic-Tac-Toe strategy (slot selection, board state) to be handled by separate modules (`ArenaDecisionEngine`, `TicTacToePlanner`, etc.).

## 8. Offline Threshold Tuning Tool
- **Tool:** `tools/tune_decision_thresholds.py`
- **Purpose:** Automate the calculation of optimal `r1/real/fake` thresholds from validation datasets.
- **Status:** Skeleton implemented; uses IoU matching and precision/recall sweeping to provide YAML recommendations. Requires full live-testing with a complete dataset.

## 9. Validation
- `colcon build` successful.
- Verified that `class_name` semantics correctly drive decisions even when `class_id` ordering is non-linear.
- **Example:** `REAL_3` now correctly triggers `COLLECT` (if team matches) instead of `AVOID`.

## 10. Safety / Design Notes
- Design prioritizes semantic labels (`class_name`) over model-specific indices (`class_id`).
- Decision results remain internal to the node to prevent breaking downstream consumers during the development/tuning phase.

## 11. Next Step
- Proceed to `BlockLocalizer` for 3D localization.
- Evaluate the upgrade of the ROS message contract to expose `DecisionResult` to other nodes.
