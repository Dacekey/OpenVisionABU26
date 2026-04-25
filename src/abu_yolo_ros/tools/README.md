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

## KFS Instance Prototype Tool

This standalone single-image prototype explores how to convert YOLO symbol detections into coarse KFS instance candidates for Meihua Forest. It is offline-only, experimental, and does not affect the ROS runtime path.

Example usage:

```bash
python3 tools/kfs_instance_prototype.py \
  --model models/fptu_abu26_detect_yolo_v1.onnx \
  --image /path/to/test.jpg \
  --team-color red \
  --max-distance-m 1.0
```

With a separate prototype config:

```bash
python3 tools/kfs_instance_prototype.py \
  --model models/fptu_abu26_detect_yolo_v1.onnx \
  --image /path/to/test.jpg \
  --config tools/config/kfs_instance_prototype.yaml \
  --team-color red
```

Prototype v2 runs:

- YOLO symbol detection
- bbox-size range filtering by default
- optional coarse projection-distance filtering for comparison
- stricter symbol clustering
- class-name-based safety voting (`R1`, `REAL_*`, `FAKE_*`)
- union and expanded KFS bbox generation
- HSV body-color masking on the expanded crop
- optional contour-based bbox refinement from the color mask only
- debug image and JSON summary export

Range filtering:

- Default mode is `bbox_size`.
- Symbols are dropped if confidence is too low, bbox height is too small, or bbox area is too small.
- `projection` mode remains available for coarse comparison but is not the default.

Clustering:

- Clustering still uses center distance with a dynamic threshold.
- Grouping is stricter because it also requires height similarity, area similarity, and bottom-y consistency.
- If a cluster contains more than `max_symbols_per_instance`, it is marked `AMBIGUOUS` and treated as unsafe.

ROI and bbox defaults:

- The default ROI is centered on the front working area.
- Expanded bbox defaults are smaller so nearby KFS or background are less likely to be swallowed.
- If the bbox is still too large or too small, tune `bbox.expand_scale` or `bbox.expand_scale_x/y`.

HSV mask modes:

- `team`
- `red`
- `blue`
- `red_or_blue`
- `dark_blue`
- `blue_or_dark_blue`
- `red_or_blue_or_dark_blue`

KFS body extraction:

- KFS body extraction uses the HSV color mask only.
- White sticker/symbol regions are not used to compute the KFS body shape.
- The optional white mask is debug-only and never affects contour selection, refined bbox, or final instance geometry.

Contour refinement:

- Default contour selection is `closest_to_symbol_union_center`.
- This is safer than `largest` when a crop contains a bigger unrelated color region.

Second-stage cluster merge:

- Close KFS can show symbols on different faces that are far apart in 2D because of perspective.
- The prototype now keeps the first-stage symbol clustering, then runs a second-stage merge over cluster candidates.
- Medium-distance KFS can false-merge if the second stage is too permissive, so the current merge also applies a compactness guard to the proposed merged symbol union.
- Merge requires semantic compatibility, enough HSV color-mask support, a strong spatial condition, and compact merged geometry.
- Center distance is debug-only by default and does not accept a merge by itself unless explicitly enabled in config.
- This is offline experimental logic only and is not part of the runtime pipeline.

Recommended first tuning attempt after v2:

- use a stricter `bbox_size` range filter
- relax area similarity slightly for perspective-distorted symbols
- set `hsv_mask.mode = red_or_blue_or_dark_blue`

`dark_blue` is useful for navy or low-value blue KFS bodies under real lighting, where the normal blue mask may miss the cube body.

Outputs are written under `tools/kfs_instance_debug` by default. The separate experimental config lives at `tools/config/kfs_instance_prototype.yaml`. The tool does not modify ROS topics, message types, runtime config, TeamColorFilter, DecisionEngine, or launch files.
