# OpenVision-v3 – KFS Instance Prototype and Aggregation Design

**Date:** 2026-04-25
**Status:** Prototype Stable (12/13 Test Cases)
**Workspace:** ~/openvision_ros2_ws_v2
**Tool Path:** src/abu_yolo_ros/tools/kfs_instance_prototype.py

---

## 1. Overview
The `kfs_instance_prototype` was created to bridge the gap between low-level object detection and high-level robot decision making. While YOLO is excellent at identifying individual symbols, the robot requires a holistic understanding of "KFS instances" (the physical boxes containing these symbols). This offline prototype serves as a sandbox to validate complex grouping and classification logic before it is ported into the production C++/ROS 2 environment.

## Version Context

This log belongs to the OpenVision-v3 development sequence. It builds on the earlier V2 refactor, TeamColorFilter, and DecisionEngine groundwork, and moves the project toward a KFS-level perception runtime for ABU Robocon 2026.

This prototype is the bridge between earlier symbol-level logic and the later OpenVision-v3 KFS runtime.

## Terminology Update

Current OpenVision-v3 treats `legal / illegal / unknown` as the vision-level interpretation labels.

Any action-oriented wording from earlier development should be read as historical discussion, not as the current runtime contract.

## 2. Problem Statement
- **Granularity Mismatch:** YOLO detects individual symbols, but decisions must be made at the KFS box level.
- **Observation Variability:** A single KFS may show multiple symbols or only one, depending on distance, perspective, and lighting.
- **Environmental Noise:** Dark images, motion blur, and partial occlusions make simple geometric grouping unreliable.
- **Risk of Over-Merging:** Incorrectly merging two nearby KFS boxes can lead to unsafe legality interpretation and unstable downstream planning input.

## 3. Input and Output
### Inputs
- **Source Image:** Raw camera frame.
- **YOLO Symbol Detections:** Class IDs, names, and bounding boxes.
- **ROI Filtering:** Definable working area to exclude background noise.
- **Color Information:** Mask/contour data for KFS-colored surfaces.

### Outputs
- **Debug Visualization:** Multi-panel images showing YOLO symbols, intermediate clustering steps, color masks, and final instances.
- **`kfs_instance_summary.json`:** Comprehensive diagnostics including:
  - `clusters` and `final_instances`
  - `dropped_clusters` (ambiguous results)
  - `merge_steps` and `merge_candidate_rankings`
  - `config_used` for reproducibility.

## 4. Main Techniques Used

### A. Range and ROI Filtering
Detections outside the relevant working area are pruned immediately. A yellow debug rectangle represents the ROI, ensuring the algorithm focuses only on valid field regions.

### B. Symbol-Level Preprocessing
Symbols are grouped by semantic "families" (e.g., REAL, FAKE, R1). Each observation carries its own geometry and confidence score.

### C. Square-Normalized Bbox Expansion
To stabilize the area estimation, symbol bounding boxes can be normalized into a square around their center before expansion. This prevents thin or skewed YOLO boxes from creating distorted search areas.

### D. Geometry vs. Expanded Bboxes
- **Geometry Bbox:** Represents the tight, symbol-level observation.
- **Expanded Bbox:** Estimates the potential footprint of the physical KFS box, used to check for overlaps with neighboring symbols.

### E. Color Contour and Mask Support
The prototype utilizes color masking to verify if a candidate region contains sufficient KFS-colored surface. The `color_mask` ratio acts as a confidence multiplier, though it is tuned to handle lighting-induced degradation.

### F. Hard-Gate Merge Logic
Merge candidates must pass a series of "Hard Gates" (safety checks) before being considered. These gates prevent merges based on conflicting classes, excessive distance, or geometric inconsistencies.

### G. Ranking-Only Candidate Selection
A second-stage ranking layer evaluates valid merge candidates based on overlap, center distance, scale similarity, and alignment. Ranking reorders candidates but never overrides the safety gates.

### H. Ambiguous Cluster Dropping
If a cluster’s final classification is `AMBIGUOUS`, it is dropped from the final output. This "Safety First" approach prefers missing a detection over acting on uncertain data.

### I. Neighbor Clamping
When KFS boxes are partially visible or near the image boundary, clamping logic prevents the algorithm from over-expanding into neighboring boxes.

### J. Score Path Diagnostics
The JSON output includes explicit tracking of the score evolution: `base_score` -> individual ranking deltas -> `final_score`, allowing for surgical tuning of weights.

## 5. Important Design Principles
- **Safety First:** Avoid false-positive KFS instances at all costs.
- **Explainability:** Every merge or drop must be traceable through diagnostics.
- **Config-Driven:** All thresholds are stored in YAML to allow for rapid environment-specific tuning.
- **No Overfitting:** If a single frame is exceptionally difficult, the algorithm should drop it rather than breaking the logic for representative cases.

## 6. Testing Summary
Tested against **13 representative images**.
- **Current Result:** 12 / 13 cases passed.
- **Case 2:** Improved by handling partial visibility and neighbor-clamped merging.
- **Case 6:** Remained unstable due to extreme perspective overlap; intentionally left unmerged to avoid overfitting the global logic.

## 7. Case Notes
- **Case 12 (Near-Distance):** Initially oversized. Fixed via square-normalized expansion and color-contour support.
- **Case 13 (Medium-Distance):** Remained stable throughout geometric changes.
- **Case 2 (Dark/Partial):** Improved by the neighbor-clamping merge path despite weak color mask quality.
- **Case 6 (Ambiguous Depth):** Experimental ranking rules (vertical-stack prior, cross-depth penalty) were tested but did not yield a universal fix without risking regressions.

## 8. Git / Branching Note
- **Stable Checkpoint:** Preserved at commit `e10a78e`.
- **Experimental Work:** Case 6 experiments (ranking-only reordering) were moved to a backup branch (e.g., `backup/case6-experiments`) to keep the main branch focused on proven robustness.

## 9. Validation
- **Compilation:** `python3 -m py_compile src/abu_yolo_ros/tools/kfs_instance_prototype.py`
- **Build:** `colcon build --packages-select abu_yolo_ros`
- **Scope:** Changes are strictly confined to offline prototype files; the ROS 2 runtime remains unchanged.

## 10. Final Status
The offline KFS instance prototype is now fully usable for analyzing competition-style images. With a 12/13 pass rate, the current logic is considered a high-quality baseline. Future work will involve evaluating more datasets before initiating the port to the ROS 2 C++ runtime.

## Current OpenVision-v3 Status

This prototype is no longer the active runtime path, but it remains the design checkpoint that validated:

- ROI and range filtering
- square-normalized symbol geometry
- HSV body mask and contour support
- conservative KFS clustering and merging
- ambiguous cluster dropping

## Relationship to Later Milestones

- The stable logic from this prototype was ported into `2026-04-26_05_OpenVisionV3_KFSInstanceRuntime.md`
- later milestones then extended the runtime with localization, stabilization, runtime safety, benchmarking, and backend architecture work
