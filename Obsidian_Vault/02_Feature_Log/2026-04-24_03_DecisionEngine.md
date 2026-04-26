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

## 12. KFS Instance Aggregation & Decision Tree (Offline Prototype)

### A. Purpose
The Decision Tree module is utilized post-YOLO detection to group disparate symbol observations into high-level KFS-level instances. The goal is to aggregate evidence to determine if a physical object is `REAL`, `FAKE`, `R1`, or `AMBIGUOUS`.

### B. High-Level Pipeline
1. **Symbol Detection:** YOLO identifies individual symbols (observations).
2. **Preprocessing:** Symbols are filtered by ROI and normalized (square-normalized expansion) to stabilize geometry.
3. **Region Generation:** Candidate KFS footprints are generated using geometry and expanded bounding boxes, supported by color-contour analysis.
4. **Clustering/Merging:** Nearby and compatible symbols are merged based on spatial and semantic rules.
5. **Aggregation:** Final clusters are evaluated and converted into `final_instances`.
6. **Safety Filter:** Uncertain or conflicting clusters are classified as `AMBIGUOUS` and dropped from the actionable output if configured.

### C. Decision Tree Logic
The following logic is applied to every candidate cluster (containing one or more symbols):

#### Step 1 — Validate Candidate Symbols
- Map `class_id` to semantic `class_family` (REAL, FAKE, R1).
- Preserve all metadata (confidence, source geometry, color diagnostics) for evidence weighting.

#### Step 2 — Merge Decision (Compatibility Check)
- **Spatial:** Check overlap, proximity, and gaps.
- **Geometric:** Verify scale compatibility and face alignment.
- **Hard Safety Gates:** Apply strict exclusion rules (e.g., maximum distance, incompatible scale).
- **Ranking:** Use a scoring layer to select the best match among candidates that pass all hard gates.

#### Step 3 — Cluster Classification
- **IF** cluster only contains REAL symbols → **REAL**
- **IF** cluster only contains FAKE symbols → **FAKE**
- **IF** cluster contains R1 symbols → **R1**
- **IF** cluster contains a mix of REAL and FAKE evidence → **AMBIGUOUS**

#### Step 4 — Safety Handling
- **IF** `drop_ambiguous_clusters` is enabled:
  - Remove `AMBIGUOUS` clusters from `final_instances`.
  - Log to `dropped_clusters` with reason `ambiguous_cluster`.
- **Note:** This "Safety-First" approach prevents the robot from acting on uncertain data, prioritizing a better future viewpoint over a current high-risk decision.

### D. Key Technical Improvements
- **Semantic Classification:** Moved away from rigid `class_id` ranges to robust class-family rules.
- **Geometry Stabilization:** Implemented square-normalized symbol bboxes before expansion to mitigate "bad expansion" from tall/thin detection boxes.
- **Conflict Avoidance:** Added neighbor-clamped expansion to prevent expanded search areas from "swallowing" independent nearby KFS.
- **Edge Case Robustness:** Added partial-visibility handling for frame borders and same-label neighbor merge support.
- **Rich Diagnostics:** Separated output into `clusters`, `final_instances`, and `dropped_clusters` with detailed ranking score components.

### E. Prototype vs. Runtime Status
- **Validation Path:** This logic is currently implemented and validated within the offline prototype:
  - `src/abu_yolo_ros/tools/kfs_instance_prototype.py`
  - `src/abu_yolo_ros/tools/config/kfs_instance_prototype.yaml`
- **Current State:** ROS runtime remains decoupled and unchanged for stability.
- **Porting Plan:** The logic is considered mature (12/13 test cases passed) and is ready for C++ porting into the primary ROS node.

### F. Validation Summary
- **Dataset:** 13 representative competition-like images.
- **Performance:** 12/13 cases successfully merged and classified.
- **Edge Case:** Case 6 remains unmerged due to extreme depth/perspective ambiguity; dropping it is preferred over forcing a potentially incorrect merge.
- **Commands:**
  - `python3 -m py_compile src/abu_yolo_ros/tools/kfs_instance_prototype.py`
  - `colcon build --packages-select abu_yolo_ros`
