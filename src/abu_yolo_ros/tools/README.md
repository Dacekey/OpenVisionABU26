# HSV Calibration Viewer

This tool is a standalone OpenCV utility for tuning the red and blue HSV thresholds used by `abu_yolo_ros` before a match. It is intended for calibration only and is not part of the real-time ROS inference path.

## Run

From the `abu_yolo_ros` package directory:

```bash
python3 tools/hsv_calibration_viewer.py --camera 0 --color blue
python3 tools/hsv_calibration_viewer.py --camera 0 --color red
```

To load initial values from the package config:

```bash
python3 tools/hsv_calibration_viewer.py --camera 0 --color blue --yaml ../config/yolo_detection.yaml
```

Optional arguments:

```bash
python3 tools/hsv_calibration_viewer.py --camera 0 --color blue --width 640 --height 480
```

## What It Shows

- `original`: raw camera frame
- `mask`: binary HSV mask
- `overlay`: mask blended on top of the camera frame

The overlay also shows the current HSV range and mask coverage ratio.

## Controls

- sliders adjust thresholds
- `s` prints a YAML snippet you can copy into `config/yolo_detection.yaml`
- `h` prints help
- `q` exits

## Recommended Workflow

1. Run blue calibration.
2. Copy the printed blue values into `config/yolo_detection.yaml`.
3. Run red calibration.
4. Copy the printed red values into `config/yolo_detection.yaml`.
5. Relaunch `yolo_detection_node`.
6. Check `debug_detections` logs to compare mask behavior against runtime detection results.
7. Disable `debug_detections` for competition mode.

## Notes

- If `--yaml` is provided, the tool will try to load initial values with PyYAML.
- If PyYAML is missing or YAML parsing fails, the tool prints a warning and falls back to built-in defaults.
- The tool never edits `yolo_detection.yaml` automatically. It only prints copyable snippets.

## Decision Threshold Tuning Tool

This standalone offline analysis tool tunes `r1_conf_threshold`, `real_conf_threshold`, and `fake_conf_threshold` using a YOLO-format validation dataset. It is intended to recommend DecisionEngine thresholds from validation data before deployment.

Example usage:

```bash
python3 tools/tune_decision_thresholds.py \
  --model models/fptu_abu26_detect_yolo_v1.onnx \
  --dataset /path/to/valid \
  --output-csv threshold_sweep.csv \
  --save-json threshold_summary.json
```

Class groups:

- `R1` = class `0`
- `REAL` = classes `1–15`
- `FAKE` = classes `16–30`

Beta choices:

- `REAL` beta = `0.75`
- `FAKE` beta = `2.0`
- `R1` beta = `1.0`

The rationale is:

- `REAL` is slightly precision-oriented to reduce unsafe collection.
- `FAKE` is recall-oriented because missing fake objects is risky.
- `R1` uses balanced `F1`.

The tool runs model inference, matches predictions to ground truth with greedy IoU matching, sweeps thresholds, reports per-group precision/recall/F-beta, and prints a YAML-ready snippet. It only prints or saves recommendations and does not auto-edit `config/yolo_detection.yaml`.

`collect_min_confidence` is not fully tuned by this first tool because it also depends on TeamColorFilter confidence.
