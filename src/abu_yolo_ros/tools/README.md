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
