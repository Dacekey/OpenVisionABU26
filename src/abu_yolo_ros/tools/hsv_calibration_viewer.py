#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


BLUE_DEFAULTS = {
    "h_low": 90,
    "h_high": 140,
    "s_low": 40,
    "s_high": 255,
    "v_low": 80,
    "v_high": 255,
}

RED_DEFAULTS = {
    "h1_low": 0,
    "h1_high": 12,
    "h2_low": 168,
    "h2_high": 180,
    "s_low": 130,
    "s_high": 255,
    "v_low": 80,
    "v_high": 255,
}

WINDOW_ORIGINAL = "original"
WINDOW_MASK = "mask"
WINDOW_OVERLAY = "overlay"
WINDOW_CONTROLS = "controls"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Standalone HSV calibration viewer for abu_yolo_ros."
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera index.")
    parser.add_argument(
        "--color",
        choices=("red", "blue"),
        default="blue",
        help="Color profile to calibrate.",
    )
    parser.add_argument("--width", type=int, default=640, help="Capture width.")
    parser.add_argument("--height", type=int, default=480, help="Capture height.")
    parser.add_argument(
        "--yaml",
        type=str,
        default=None,
        help="Optional path to yolo_detection.yaml for initial values.",
    )
    return parser.parse_args()


def print_help():
    print("Controls:")
    print("  sliders: adjust HSV thresholds")
    print("  s: print YAML snippet for current thresholds")
    print("  h: print this help")
    print("  q or ESC: quit")


def load_yaml_thresholds(yaml_path, color):
    defaults = dict(BLUE_DEFAULTS if color == "blue" else RED_DEFAULTS)
    if not yaml_path:
        return defaults

    try:
        import yaml  # type: ignore
    except ImportError:
        print(
            "[WARN] PyYAML is not installed. Using built-in defaults instead.",
            file=sys.stderr,
        )
        return defaults

    path = Path(yaml_path)
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception as exc:
        print(
            f"[WARN] Failed to load YAML from {path}: {exc}. "
            "Using built-in defaults instead.",
            file=sys.stderr,
        )
        return defaults

    params = data.get("yolo_detection_node", {}).get("ros__parameters", {})
    try:
        if color == "blue":
            defaults["h_low"] = int(params.get("blue_h_low", defaults["h_low"]))
            defaults["h_high"] = int(params.get("blue_h_high", defaults["h_high"]))
            defaults["s_low"] = int(params.get("blue_s_low", defaults["s_low"]))
            defaults["v_low"] = int(params.get("blue_v_low", defaults["v_low"]))
        else:
            defaults["h1_low"] = int(params.get("red_h_low_1", defaults["h1_low"]))
            defaults["h1_high"] = int(params.get("red_h_high_1", defaults["h1_high"]))
            defaults["h2_low"] = int(params.get("red_h_low_2", defaults["h2_low"]))
            defaults["h2_high"] = int(params.get("red_h_high_2", defaults["h2_high"]))
            defaults["s_low"] = int(params.get("red_s_low", defaults["s_low"]))
            defaults["v_low"] = int(params.get("red_v_low", defaults["v_low"]))
    except (TypeError, ValueError) as exc:
        print(
            f"[WARN] YAML parsing error: {exc}. Using built-in defaults instead.",
            file=sys.stderr,
        )
        return BLUE_DEFAULTS if color == "blue" else RED_DEFAULTS

    return defaults


def noop(_value):
    return None


def create_trackbars(color, initial_values):
    cv2.namedWindow(WINDOW_CONTROLS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_CONTROLS, 420, 320)

    if color == "blue":
        trackbars = (
            ("H low", 179, initial_values["h_low"]),
            ("H high", 179, initial_values["h_high"]),
            ("S low", 255, initial_values["s_low"]),
            ("S high", 255, initial_values["s_high"]),
            ("V low", 255, initial_values["v_low"]),
            ("V high", 255, initial_values["v_high"]),
        )
    else:
        trackbars = (
            ("H1 low", 179, initial_values["h1_low"]),
            ("H1 high", 179, initial_values["h1_high"]),
            ("H2 low", 179, initial_values["h2_low"]),
            ("H2 high", 180, initial_values["h2_high"]),
            ("S low", 255, initial_values["s_low"]),
            ("S high", 255, initial_values["s_high"]),
            ("V low", 255, initial_values["v_low"]),
            ("V high", 255, initial_values["v_high"]),
        )

    for name, max_value, initial in trackbars:
        cv2.createTrackbar(name, WINDOW_CONTROLS, initial, max_value, noop)


def get_trackbar_values(color):
    if color == "blue":
        values = {
            "h_low": cv2.getTrackbarPos("H low", WINDOW_CONTROLS),
            "h_high": cv2.getTrackbarPos("H high", WINDOW_CONTROLS),
            "s_low": cv2.getTrackbarPos("S low", WINDOW_CONTROLS),
            "s_high": cv2.getTrackbarPos("S high", WINDOW_CONTROLS),
            "v_low": cv2.getTrackbarPos("V low", WINDOW_CONTROLS),
            "v_high": cv2.getTrackbarPos("V high", WINDOW_CONTROLS),
        }
    else:
        values = {
            "h1_low": cv2.getTrackbarPos("H1 low", WINDOW_CONTROLS),
            "h1_high": cv2.getTrackbarPos("H1 high", WINDOW_CONTROLS),
            "h2_low": cv2.getTrackbarPos("H2 low", WINDOW_CONTROLS),
            "h2_high": cv2.getTrackbarPos("H2 high", WINDOW_CONTROLS),
            "s_low": cv2.getTrackbarPos("S low", WINDOW_CONTROLS),
            "s_high": cv2.getTrackbarPos("S high", WINDOW_CONTROLS),
            "v_low": cv2.getTrackbarPos("V low", WINDOW_CONTROLS),
            "v_high": cv2.getTrackbarPos("V high", WINDOW_CONTROLS),
        }

    return sanitize_values(color, values)


def sanitize_values(color, values):
    if color == "blue":
        values["h_low"] = min(values["h_low"], values["h_high"])
        values["s_low"] = min(values["s_low"], values["s_high"])
        values["v_low"] = min(values["v_low"], values["v_high"])
    else:
        values["h1_low"] = min(values["h1_low"], values["h1_high"])
        values["h2_low"] = min(values["h2_low"], values["h2_high"])
        values["s_low"] = min(values["s_low"], values["s_high"])
        values["v_low"] = min(values["v_low"], values["v_high"])

    return values


def build_mask(hsv_frame, color, values):
    if color == "blue":
        lower = np.array(
            [values["h_low"], values["s_low"], values["v_low"]],
            dtype=np.uint8,
        )
        upper = np.array(
            [values["h_high"], values["s_high"], values["v_high"]],
            dtype=np.uint8,
        )
        mask = cv2.inRange(hsv_frame, lower, upper)
    else:
        lower_1 = np.array(
            [values["h1_low"], values["s_low"], values["v_low"]],
            dtype=np.uint8,
        )
        upper_1 = np.array(
            [values["h1_high"], values["s_high"], values["v_high"]],
            dtype=np.uint8,
        )
        lower_2 = np.array(
            [values["h2_low"], values["s_low"], values["v_low"]],
            dtype=np.uint8,
        )
        upper_2 = np.array(
            [values["h2_high"], values["s_high"], values["v_high"]],
            dtype=np.uint8,
        )
        mask_1 = cv2.inRange(hsv_frame, lower_1, upper_1)
        mask_2 = cv2.inRange(hsv_frame, lower_2, upper_2)
        mask = cv2.bitwise_or(mask_1, mask_2)

    return mask


def make_overlay(frame, mask, color):
    overlay = frame.copy()
    color_mask = np.zeros_like(frame)
    if color == "blue":
        color_mask[:, :] = (255, 0, 0)
    else:
        color_mask[:, :] = (0, 0, 255)

    masked_color = cv2.bitwise_and(color_mask, color_mask, mask=mask)
    return cv2.addWeighted(overlay, 0.7, masked_color, 0.6, 0.0)


def describe_range(color, values):
    if color == "blue":
        return (
            f"blue H[{values['h_low']},{values['h_high']}] "
            f"S[{values['s_low']},{values['s_high']}] "
            f"V[{values['v_low']},{values['v_high']}]"
        )

    return (
        f"red H1[{values['h1_low']},{values['h1_high']}] "
        f"H2[{values['h2_low']},{values['h2_high']}] "
        f"S[{values['s_low']},{values['s_high']}] "
        f"V[{values['v_low']},{values['v_high']}]"
    )


def draw_overlay_text(image, lines):
    y = 24
    for line in lines:
        cv2.putText(
            image,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        y += 24


def print_yaml_snippet(color, values):
    print("")
    print("# Copy into config/yolo_detection.yaml -> yolo_detection_node.ros__parameters")
    if color == "blue":
        print(f"blue_h_low: {values['h_low']}")
        print(f"blue_h_high: {values['h_high']}")
        print(f"blue_s_low: {values['s_low']}")
        print(f"blue_v_low: {values['v_low']}")
        print(f"# blue_s_high: {values['s_high']}")
        print(f"# blue_v_high: {values['v_high']}")
    else:
        print(f"red_h_low_1: {values['h1_low']}")
        print(f"red_h_high_1: {values['h1_high']}")
        print(f"red_h_low_2: {values['h2_low']}")
        print(f"red_h_high_2: {values['h2_high']}")
        print(f"red_s_low: {values['s_low']}")
        print(f"red_v_low: {values['v_low']}")
        print(f"# red_s_high: {values['s_high']}")
        print(f"# red_v_high: {values['v_high']}")
    print("")


def main():
    args = parse_args()
    initial_values = load_yaml_thresholds(args.yaml, args.color)

    print_help()
    print(f"Starting HSV calibration viewer in {args.color} mode.")
    if args.yaml:
        print(f"YAML path: {args.yaml}")

    capture = cv2.VideoCapture(args.camera)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not capture.isOpened():
        print(
            f"[ERROR] Failed to open camera {args.camera}. "
            "Check the camera index and device permissions.",
            file=sys.stderr,
        )
        return 1

    cv2.namedWindow(WINDOW_ORIGINAL, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_OVERLAY, cv2.WINDOW_NORMAL)
    create_trackbars(args.color, initial_values)

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("[WARN] Camera frame read failed. Exiting.", file=sys.stderr)
                break

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            values = get_trackbar_values(args.color)
            mask = build_mask(hsv_frame, args.color, values)
            overlay = make_overlay(frame, mask, args.color)

            coverage_ratio = float(cv2.countNonZero(mask)) / float(mask.size)
            range_description = describe_range(args.color, values)
            coverage_description = f"mask coverage={coverage_ratio:.4f}"

            draw_overlay_text(
                overlay,
                (
                    f"mode={args.color}",
                    range_description,
                    coverage_description,
                    "Keys: s=print yaml, h=help, q/ESC=quit",
                ),
            )

            cv2.imshow(WINDOW_ORIGINAL, frame)
            cv2.imshow(WINDOW_MASK, mask)
            cv2.imshow(WINDOW_OVERLAY, overlay)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                print_yaml_snippet(args.color, values)
            if key == ord("h"):
                print_help()
                print(range_description)
                print(coverage_description)

    finally:
        capture.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
