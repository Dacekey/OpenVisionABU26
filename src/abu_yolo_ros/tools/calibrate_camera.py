#!/usr/bin/env python3
"""Offline camera calibration tool for OpenVision-v3 localizer inputs.

This tool calibrates a chessboard dataset using either the OpenCV fisheye
model or the standard pinhole model. It writes a standalone YAML file plus a
copyable `kfs_3d_localizer` config snippet.
"""

from __future__ import annotations

import argparse
import glob
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np


@dataclass
class ImageObservation:
    path: Path
    image_size: tuple[int, int]
    object_points: np.ndarray
    image_points: np.ndarray


@dataclass
class CalibrationResult:
    model: str
    image_width: int
    image_height: int
    runtime_width: int
    runtime_height: int
    board_cols: int
    board_rows: int
    square_size_mm: float
    valid_images: int
    total_images: int
    reprojection_error: float
    camera_matrix: np.ndarray
    runtime_camera_matrix: np.ndarray
    distortion_coeffs: list[float]
    pinhole_coeffs: list[float]
    fisheye_coeffs: list[float]
    resize_warning: str
    skipped_images: list[dict[str, str]]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Offline chessboard calibration for OpenVision-v3. "
            "Supports fisheye and pinhole models and exports a "
            "kfs_3d_localizer-compatible YAML snippet."
        )
    )
    parser.add_argument(
        "--images",
        required=True,
        help="Glob pattern for calibration images, for example 'data/*.jpg'.",
    )
    parser.add_argument(
        "--model",
        required=True,
        choices=("fisheye", "pinhole"),
        help="Calibration model to estimate.",
    )
    parser.add_argument(
        "--board-cols",
        required=True,
        type=int,
        help="Number of inner chessboard corners horizontally.",
    )
    parser.add_argument(
        "--board-rows",
        required=True,
        type=int,
        help="Number of inner chessboard corners vertically.",
    )
    parser.add_argument(
        "--square-size-mm",
        required=True,
        type=float,
        help="Physical chessboard square size in millimeters.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output YAML file path.",
    )
    parser.add_argument(
        "--preview-dir",
        help="Optional directory to save detected corners and undistort previews.",
    )
    parser.add_argument(
        "--resize-width",
        type=int,
        help="Optional runtime width for scaled intrinsics export.",
    )
    parser.add_argument(
        "--resize-height",
        type=int,
        help="Optional runtime height for scaled intrinsics export.",
    )
    parser.add_argument(
        "--min-valid-images",
        type=int,
        default=10,
        help="Minimum valid calibration images required to continue.",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        help="Optional cap for the number of images loaded from the glob.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show OpenCV preview windows while processing, if display is available.",
    )
    parser.add_argument(
        "--export-localizer-snippet",
        action="store_true",
        help="Print a ready-to-copy kfs_3d_localizer YAML snippet to stdout.",
    )
    return parser


def load_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "OpenCV Python bindings are required for calibration. "
            "Install python3-opencv or an equivalent package."
        ) from exc
    return cv2


def make_object_points(board_cols: int, board_rows: int, square_size_mm: float) -> np.ndarray:
    objp = np.zeros((board_rows * board_cols, 3), np.float32)
    grid = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    objp[:, :2] = grid
    objp *= square_size_mm
    return objp


def ensure_same_resolution(
    image_size: tuple[int, int] | None,
    new_size: tuple[int, int],
    path: Path,
) -> tuple[int, int]:
    if image_size is None:
        return new_size
    if image_size != new_size:
        raise SystemExit(
            f"Calibration images have mixed resolutions. "
            f"Expected {image_size[0]}x{image_size[1]}, got {new_size[0]}x{new_size[1]} "
            f"for {path}."
        )
    return image_size


def detect_observations(
    cv2: Any,
    image_paths: list[Path],
    board_cols: int,
    board_rows: int,
    square_size_mm: float,
    preview_dir: Path | None,
    show: bool,
) -> tuple[list[ImageObservation], list[dict[str, str]], tuple[int, int]]:
    object_template = make_object_points(board_cols, board_rows, square_size_mm)
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    pattern_size = (board_cols, board_rows)

    observations: list[ImageObservation] = []
    skipped: list[dict[str, str]] = []
    image_size: tuple[int, int] | None = None

    if preview_dir is not None:
        preview_dir.mkdir(parents=True, exist_ok=True)

    for index, path in enumerate(image_paths):
        image = cv2.imread(str(path))
        if image is None:
            skipped.append({"image": str(path), "reason": "failed_to_read"})
            continue

        current_size = (image.shape[1], image.shape[0])
        image_size = ensure_same_resolution(image_size, current_size, path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=(
                cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK
            ),
        )

        preview = image.copy()
        if not found or corners is None:
            skipped.append({"image": str(path), "reason": "corners_not_found"})
            if preview_dir is not None:
                cv2.putText(
                    preview,
                    "corners_not_found",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imwrite(str(preview_dir / f"{index:03d}_skip_{path.name}"), preview)
            if show:
                cv2.imshow("calibration_detection", preview)
                cv2.waitKey(50)
            continue

        refined_corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria,
        )
        cv2.drawChessboardCorners(preview, pattern_size, refined_corners, found)
        if preview_dir is not None:
            cv2.imwrite(str(preview_dir / f"{index:03d}_corners_{path.name}"), preview)
        if show:
            cv2.imshow("calibration_detection", preview)
            cv2.waitKey(50)

        observations.append(
            ImageObservation(
                path=path,
                image_size=current_size,
                object_points=object_template.copy(),
                image_points=refined_corners.copy(),
            )
        )

    if show:
        cv2.destroyAllWindows()

    if image_size is None:
        raise SystemExit("No readable images were found in the provided glob.")

    return observations, skipped, image_size


def calibrate_pinhole(cv2: Any, observations: list[ImageObservation], image_size: tuple[int, int]) -> tuple[np.ndarray, list[float], list[np.ndarray], list[np.ndarray]]:
    object_points = [obs.object_points.astype(np.float32) for obs in observations]
    image_points = [obs.image_points.astype(np.float32) for obs in observations]
    success, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    if not success:
        raise SystemExit("OpenCV pinhole calibration failed.")
    coeffs = [float(value) for value in dist_coeffs.reshape(-1)[:5]]
    if len(coeffs) < 5:
        coeffs.extend([0.0] * (5 - len(coeffs)))
    return camera_matrix, coeffs, rvecs, tvecs


def calibrate_fisheye(cv2: Any, observations: list[ImageObservation], image_size: tuple[int, int]) -> tuple[np.ndarray, list[float], list[np.ndarray], list[np.ndarray]]:
    if not hasattr(cv2, "fisheye"):
        raise SystemExit("This OpenCV build does not include fisheye calibration support.")

    object_points = [
        obs.object_points.reshape(1, -1, 3).astype(np.float64) for obs in observations
    ]
    image_points = [
        obs.image_points.reshape(-1, 1, 2).astype(np.float64) for obs in observations
    ]

    camera_matrix = np.eye(3, dtype=np.float64)
    dist_coeffs = np.zeros((4, 1), dtype=np.float64)
    rvecs = [np.zeros((3, 1), dtype=np.float64) for _ in observations]
    tvecs = [np.zeros((3, 1), dtype=np.float64) for _ in observations]
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_CHECK_COND
        | cv2.fisheye.CALIB_FIX_SKEW
    )
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-6,
    )

    try:
        cv2.fisheye.calibrate(
            object_points,
            image_points,
            image_size,
            camera_matrix,
            dist_coeffs,
            rvecs,
            tvecs,
            flags,
            criteria,
        )
    except cv2.error as exc:
        raise SystemExit(f"OpenCV fisheye calibration failed: {exc}") from exc

    coeffs = [float(value) for value in dist_coeffs.reshape(-1)[:4]]
    if len(coeffs) < 4:
        coeffs.extend([0.0] * (4 - len(coeffs)))
    return camera_matrix, coeffs, rvecs, tvecs


def compute_reprojection_error(
    cv2: Any,
    model: str,
    observations: list[ImageObservation],
    camera_matrix: np.ndarray,
    distortion_coeffs: list[float],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
) -> float:
    total_error = 0.0
    total_points = 0

    for observation, rvec, tvec in zip(observations, rvecs, tvecs):
        if model == "fisheye":
            dist = np.array(distortion_coeffs, dtype=np.float64).reshape(4, 1)
            projected, _ = cv2.fisheye.projectPoints(
                observation.object_points.reshape(1, -1, 3).astype(np.float64),
                rvec,
                tvec,
                camera_matrix,
                dist,
            )
            reference = observation.image_points.reshape(-1, 1, 2).astype(np.float64)
        else:
            dist = np.array(distortion_coeffs, dtype=np.float64).reshape(-1, 1)
            projected, _ = cv2.projectPoints(
                observation.object_points.astype(np.float64),
                rvec,
                tvec,
                camera_matrix,
                dist,
            )
            reference = observation.image_points.astype(np.float64)

        error = cv2.norm(reference, projected, cv2.NORM_L2)
        total_error += error * error
        total_points += len(observation.object_points)

    if total_points <= 0:
        return 0.0
    return math.sqrt(total_error / total_points)


def scale_camera_matrix(
    camera_matrix: np.ndarray,
    original_size: tuple[int, int],
    runtime_size: tuple[int, int],
) -> np.ndarray:
    scaled = camera_matrix.copy().astype(np.float64)
    scale_x = runtime_size[0] / original_size[0]
    scale_y = runtime_size[1] / original_size[1]
    scaled[0, 0] *= scale_x
    scaled[1, 1] *= scale_y
    scaled[0, 2] *= scale_x
    scaled[1, 2] *= scale_y
    return scaled


def save_undistort_previews(
    cv2: Any,
    preview_dir: Path,
    observations: list[ImageObservation],
    model: str,
    camera_matrix: np.ndarray,
    distortion_coeffs: list[float],
) -> None:
    sample_observations = observations[: min(5, len(observations))]
    for index, observation in enumerate(sample_observations):
        image = cv2.imread(str(observation.path))
        if image is None:
            continue
        if model == "fisheye":
            undistorted = cv2.fisheye.undistortImage(
                image,
                camera_matrix,
                np.array(distortion_coeffs, dtype=np.float64).reshape(4, 1),
                Knew=camera_matrix,
            )
        else:
            undistorted = cv2.undistort(
                image,
                camera_matrix,
                np.array(distortion_coeffs, dtype=np.float64).reshape(-1, 1),
            )
        cv2.imwrite(str(preview_dir / f"{index:03d}_undistort_{observation.path.name}"), undistorted)


def format_float(value: float) -> str:
    return f"{value:.8f}".rstrip("0").rstrip(".") if "." in f"{value:.8f}" else f"{value:.8f}"


def yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        return format_float(float(value))
    if value is None:
        return "null"
    text = str(value)
    if text == "" or any(ch in text for ch in [":", "#", "[", "]", "{", "}", '"', "'"]) or text.strip() != text:
        return f'"{text}"'
    return text


def dump_yaml(data: Any, indent: int = 0) -> str:
    prefix = " " * indent
    if isinstance(data, dict):
        lines: list[str] = []
        for key, value in data.items():
            if isinstance(value, (dict, list)):
                lines.append(f"{prefix}{key}:")
                lines.append(dump_yaml(value, indent + 2))
            else:
                lines.append(f"{prefix}{key}: {yaml_scalar(value)}")
        return "\n".join(lines)
    if isinstance(data, list):
        lines = []
        for item in data:
            if isinstance(item, (dict, list)):
                lines.append(f"{prefix}-")
                lines.append(dump_yaml(item, indent + 2))
            else:
                lines.append(f"{prefix}- {yaml_scalar(item)}")
        return "\n".join(lines)
    return f"{prefix}{yaml_scalar(data)}"


def build_output_data(result: CalibrationResult) -> dict[str, Any]:
    return {
        "camera_calibration": {
            "model": result.model,
            "image_width": result.image_width,
            "image_height": result.image_height,
            "runtime_width": result.runtime_width,
            "runtime_height": result.runtime_height,
            "board_cols": result.board_cols,
            "board_rows": result.board_rows,
            "square_size_mm": result.square_size_mm,
            "valid_images": result.valid_images,
            "total_images": result.total_images,
            "reprojection_error": result.reprojection_error,
            "resize_warning": result.resize_warning,
            "camera_matrix": {
                "fx": float(result.camera_matrix[0, 0]),
                "fy": float(result.camera_matrix[1, 1]),
                "cx": float(result.camera_matrix[0, 2]),
                "cy": float(result.camera_matrix[1, 2]),
            },
            "runtime_camera_matrix": {
                "fx": float(result.runtime_camera_matrix[0, 0]),
                "fy": float(result.runtime_camera_matrix[1, 1]),
                "cx": float(result.runtime_camera_matrix[0, 2]),
                "cy": float(result.runtime_camera_matrix[1, 2]),
            },
            "distortion": {
                "model": result.model,
                "fisheye_coeffs": result.fisheye_coeffs,
                "pinhole_coeffs": result.pinhole_coeffs,
            },
            "skipped_images": result.skipped_images,
        },
        "kfs_3d_localizer": {
            "image": {
                "width": result.runtime_width,
                "height": result.runtime_height,
            },
            "camera_matrix": {
                "fx": float(result.runtime_camera_matrix[0, 0]),
                "fy": float(result.runtime_camera_matrix[1, 1]),
                "cx": float(result.runtime_camera_matrix[0, 2]),
                "cy": float(result.runtime_camera_matrix[1, 2]),
            },
            "distortion": {
                "model": result.model,
                "pinhole_coeffs": result.pinhole_coeffs,
                "fisheye_coeffs": result.fisheye_coeffs,
            },
        },
    }


def print_summary(result: CalibrationResult, output_path: Path) -> None:
    print("Calibration completed.")
    print(f"  model: {result.model}")
    print(f"  images: {result.valid_images}/{result.total_images} valid")
    print(f"  reprojection_error: {result.reprojection_error:.6f}")
    print(
        "  calibration_size: "
        f"{result.image_width}x{result.image_height}"
    )
    print(
        "  runtime_size: "
        f"{result.runtime_width}x{result.runtime_height}"
    )
    print(f"  output_yaml: {output_path}")
    print(
        "  note: runtime camera extrinsics are not calibrated by this tool. "
        "Measure or calibrate camera_pose_robot.x/y/z_mm and roll/pitch/yaw separately."
    )
    if result.resize_warning:
        print(f"  warning: {result.resize_warning}")


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    image_matches = sorted(glob.glob(args.images))
    if not image_matches:
        parser.error(f"No calibration images matched: {args.images}")

    if args.max_images is not None and args.max_images <= 0:
        parser.error("--max-images must be > 0 when provided")
    if args.min_valid_images <= 0:
        parser.error("--min-valid-images must be > 0")
    if args.board_cols <= 0 or args.board_rows <= 0:
        parser.error("--board-cols and --board-rows must be > 0")
    if args.square_size_mm <= 0.0:
        parser.error("--square-size-mm must be > 0")
    if (args.resize_width is None) ^ (args.resize_height is None):
        parser.error("--resize-width and --resize-height must be provided together")

    image_paths = [Path(path) for path in image_matches]
    if args.max_images is not None:
        image_paths = image_paths[: args.max_images]

    preview_dir = Path(args.preview_dir) if args.preview_dir else None
    output_path = Path(args.output)
    if output_path.parent != Path(""):
        output_path.parent.mkdir(parents=True, exist_ok=True)

    cv2 = load_cv2()
    observations, skipped_images, image_size = detect_observations(
        cv2=cv2,
        image_paths=image_paths,
        board_cols=args.board_cols,
        board_rows=args.board_rows,
        square_size_mm=args.square_size_mm,
        preview_dir=preview_dir,
        show=args.show,
    )

    if len(observations) < args.min_valid_images:
        raise SystemExit(
            f"Not enough valid calibration images: "
            f"{len(observations)} valid, need at least {args.min_valid_images}."
        )

    if args.model == "fisheye":
        camera_matrix, distortion_coeffs, rvecs, tvecs = calibrate_fisheye(
            cv2, observations, image_size
        )
        fisheye_coeffs = distortion_coeffs
        pinhole_coeffs = [0.0] * 5
    else:
        camera_matrix, distortion_coeffs, rvecs, tvecs = calibrate_pinhole(
            cv2, observations, image_size
        )
        pinhole_coeffs = distortion_coeffs
        fisheye_coeffs = [0.0] * 4

    reprojection_error = compute_reprojection_error(
        cv2=cv2,
        model=args.model,
        observations=observations,
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
        rvecs=rvecs,
        tvecs=tvecs,
    )

    runtime_size = image_size
    resize_warning = ""
    runtime_camera_matrix = camera_matrix.copy()
    if args.resize_width is not None and args.resize_height is not None:
        runtime_size = (args.resize_width, args.resize_height)
        runtime_camera_matrix = scale_camera_matrix(camera_matrix, image_size, runtime_size)
        if runtime_size != image_size:
            resize_warning = (
                "Calibration should ideally use the same resolution and camera mode "
                "as runtime. Intrinsics were scaled for the requested runtime size."
            )

    if preview_dir is not None:
        save_undistort_previews(
            cv2=cv2,
            preview_dir=preview_dir,
            observations=observations,
            model=args.model,
            camera_matrix=camera_matrix,
            distortion_coeffs=distortion_coeffs,
        )

    result = CalibrationResult(
        model=args.model,
        image_width=image_size[0],
        image_height=image_size[1],
        runtime_width=runtime_size[0],
        runtime_height=runtime_size[1],
        board_cols=args.board_cols,
        board_rows=args.board_rows,
        square_size_mm=args.square_size_mm,
        valid_images=len(observations),
        total_images=len(image_paths),
        reprojection_error=reprojection_error,
        camera_matrix=camera_matrix,
        runtime_camera_matrix=runtime_camera_matrix,
        distortion_coeffs=distortion_coeffs,
        pinhole_coeffs=pinhole_coeffs,
        fisheye_coeffs=fisheye_coeffs,
        resize_warning=resize_warning,
        skipped_images=skipped_images,
    )

    output_data = build_output_data(result)
    output_path.write_text(dump_yaml(output_data) + "\n", encoding="utf-8")
    print_summary(result, output_path)

    if args.export_localizer_snippet:
        print("\n# kfs_3d_localizer snippet")
        print(dump_yaml({"kfs_3d_localizer": output_data["kfs_3d_localizer"]}))

    return 0


if __name__ == "__main__":
    sys.exit(main())
