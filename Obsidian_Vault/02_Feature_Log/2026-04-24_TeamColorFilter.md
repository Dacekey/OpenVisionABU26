# Feature Log: TeamColorFilter Implementation & Calibration

**Date:** 2026-04-24
**Status:** Stable / Calibration Complete
**Workspace:** ~/openvision_ros2_ws_v2
**Package:** abu_yolo_ros

---

## 1. Objective
- Add a real TeamColorFilter module to OpenVisionABU26 v2.
- Detect whether each YOLO bbox belongs to red team, blue team, or unknown using HSV color filtering.
- Keep ROS output contracts unchanged for now (no message type or topic modifications).

## 2. Implemented Changes
- **Module Implementation:**
  - `include/abu_yolo_ros/team_color_filter.hpp`
  - `src/team_color_filter.cpp`
- **Data Structures:**
  - Added `TeamColor` enum: `RED`, `BLUE`, `UNKNOWN`.
  - Added `TeamColorResult` fields for coverage, mean HSV, confidence, and dominant color analysis.
  - Added `TeamColorFilterConfig` to move HSV thresholds from hard-coded constants to ROS parameters.
- **Node Integration:**
  - Wired `TeamColorFilter` into `yolo_detection_node.cpp`.
  - Added and handled parameters: `enable_team_color_filter`, `team_color`, `debug_detections`, and specific HSV range/tuning values.
- **Configuration:**
  - Added default tuning values to `config/yolo_detection.yaml`.
- **Diagnostics & Tooling:**
  - Added controlled detection debug logging and HSV diagnostics for calibration.
  - Added standalone HSV calibration tool: `tools/hsv_calibration_viewer.py` and `tools/README.md`.

## 3. Runtime Behavior
- TeamColorFilter runs internally immediately after YOLO inference.
- **Isolation:** It does not modify `Detection2DArray` output or draw on the annotated image yet.
- **Decoupling:** It does not implement `DecisionEngine` yet; it serves as a pure classification and diagnostic provider.

## 4. Current HSV Tuning Result
- **Red:** Initial thresholds worked well immediately.
- **Blue:** Initially detected as `UNKNOWN` due to overly strict saturation (`s_low`) thresholds.
- **Relaxation:** Diagnostics showed blue samples had lower saturation than anticipated in the environment.
- **Result:** Relaxed Blue YAML thresholds (`H: 90–140`, `S >= 40`, `V >= 80`) successfully resolved the detection issue.

## 5. Calibration Tool
- Added `hsv_calibration_viewer.py` as a standalone Python/OpenCV tool.
- **Features:** Supports `--camera`, `--color`, `--width/height`, and `--yaml` inputs.
- **Feedback:** Displays original image, mask, and overlay with real-time coverage stats.
- **Workflow:** Pressing "s" prints a YAML snippet to the terminal for manual update to `yolo_detection.yaml` (no auto-saving for safety).

## 6. Validation
- `colcon build --packages-select abu_yolo_ros` passed.
- **Live Testing:** Confirmed Red and Blue detections are accurate with new thresholds.
- **Integrity:** Existing ROS outputs and annotated images remain unchanged.

## 7. Safety / Design Notes
- Thresholds are now YAML-configurable to avoid recompiling C++ code for different lighting environments.
- **Operating Modes:** Use `debug_detections` for tuning; disable in competition mode to minimize overhead.
- **Manual Gate:** The calibration tool intentionally does not auto-save to prevent accidental corruption of known-good configurations.

## 8. Future Option: Threshold Validation Runner
- **Purpose:** An optional tool to verify tuned thresholds against a dataset of sample images (`red/`, `blue/`, `background/`).
- **Function:** Provides confidence checking (e.g., "Red samples detected as RED: 48/50") before deployment.
- **Timeline:** To be built after `DecisionEngine` or before final competition deployment.

## 9. Next Step
- Proceed to `DecisionEngine` implementation to consume `TeamColorResult` and produce game-state actions (`COLLECT`, `AVOID`, `UNKNOWN`).
