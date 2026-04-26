#!/usr/bin/env python3

import argparse
import copy
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "kfs_instance_debug"
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent / "config" / "kfs_instance_prototype.yaml"
)

DEFAULT_CONFIG = {
    "range_filter": {
        "mode": "bbox_size",
        "min_symbol_height_px": 60,
        "min_symbol_area_px": 4000,
        "min_confidence": 0.25,
        "max_distance_m": 1.0,
        "min_distance_m": 0.2,
    },
    "clustering": {
        "cluster_distance_scale": 2.5,
        "min_cluster_distance_px": 35,
        "max_cluster_distance_px": 180,
        "max_depth_diff_m": 0.25,
        "max_symbols_per_instance": 3,
        "min_height_similarity_for_grouping": 0.60,
        "min_area_similarity_for_grouping": 0.30,
        "max_bottom_y_diff_ratio": 1.2,
    },
    "cluster_merge": {
        "enabled": True,
        "merge_same_group_only": True,
        "max_symbols_after_merge": 3,
        "min_expanded_iou": 0.03,
        "min_expanded_intersection_over_min_area": 0.12,
        "max_expanded_gap_px": 40,
        "max_union_center_distance_px": 550,
        "allow_center_distance_only_merge": False,
        "min_color_mask_coverage_for_merge": 0.15,
        "use_distance_check": False,
        "max_distance_diff_m": 0.35,
        "max_merged_union_width_scale": 2.2,
        "max_merged_union_height_scale": 2.6,
    },
    "merge": {
        "low_mask_adjacent_same_group_fallback": {
            "enabled": True,
            "require_same_group_type": True,
            "reject_ambiguous": True,
            "reject_real_fake_mix": True,
            "max_symbols_after_merge": 3,
            "require_height_similarity": True,
            "min_height_similarity": 0.55,
            "require_bottom_y_consistency": True,
            "max_bottom_y_diff_ratio": 1.35,
            "require_small_gap": True,
            "max_gap_px": 45,
            "max_gap_ratio_to_symbol_height": 0.55,
            "max_merged_aspect_ratio": 2.2,
            "max_merged_width_scale": 2.2,
            "max_merged_height_scale": 2.6,
            "log_rejected_candidates": True,
        },
    },
    "bbox": {
        "use_square_symbol_bbox": True,
        "square_symbol_scale": 1.0,
        "max_square_side_ratio_of_image": 0.35,
        "clustering_center_source": "raw",
        "expand_scale": 1.6,
        "expand_scale_x": 1.6,
        "expand_scale_y": 1.8,
        "max_expanded_area_ratio": 0.25,
    },
    "roi_filter": {
        "enabled": True,
        "preset": "center_working_area",
        "mode": "normalized_box",
        "x_min_norm": 0.10,
        "x_max_norm": 0.78,
        "y_min_norm": 0.22,
        "y_max_norm": 1.0,
        "polygon": [],
    },
    "hsv_mask": {
        "active_profile": "competition_blue",
        "profiles": {
            "competition_blue": {
                "description": (
                    "Default competition blue KFS body mask. Placeholder values; "
                    "tune with hsv_calibration_viewer.py before competition."
                ),
                "mode": "blue",
                "blue_h_low": 90,
                "blue_h_high": 140,
                "blue_s_low": 40,
                "blue_v_low": 80,
            },
            "red": {
                "description": "Red-team / red-body fallback profile.",
                "mode": "red",
                "red_h_low_1": 0,
                "red_h_high_1": 12,
                "red_h_low_2": 168,
                "red_h_high_2": 180,
                "red_s_low": 130,
                "red_v_low": 80,
            },
            "dark_blue_debug": {
                "enabled": False,
                "debug_only": True,
                "description": (
                    "Debug-only dark-blue fallback for underexposed/offline images. "
                    "Do not use as default competition profile unless explicitly needed."
                ),
                "mode": "dark_blue",
                "dark_blue_h_low": 90,
                "dark_blue_h_high": 140,
                "dark_blue_s_low": 20,
                "dark_blue_v_low": 15,
                "dark_blue_v_high": 160,
            },
        },
    },
    "contour": {
        "enabled": True,
        "min_contour_area_px": 300,
        "morphology_kernel_size": 5,
        "morphology_iterations": 1,
        "selection": "closest_to_symbol_union_center",
        "max_center_offset_ratio": 1.0,
        "min_refined_to_union_area_ratio": 0.5,
    },
    "refinement": {
        "neighbor_aware_clamp": True,
        "neighbor_overlap_margin_px": 10,
        "protect_other_cluster_symbols": True,
        "neighbor_protection_bbox_source": "geometry",
        "edge_clip_margin_px": 3,
        "drop_edge_clipped": False,
    },
    "aggregation": {
        "drop_ambiguous_clusters": True,
    },
    "debug_white_mask": {
        "enabled": True,
        "s_high": 80,
        "v_low": 150,
        "debug_only": True,
    },
}


@dataclass
class SymbolRecord:
    index: int
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: Tuple[int, int, int, int]
    raw_bbox: Tuple[int, int, int, int]
    geometry_bbox: Tuple[int, int, int, int]
    geometry_bbox_source: str
    center_xy: Tuple[float, float]
    raw_center_xy: Tuple[float, float]
    geometry_center_xy: Tuple[float, float]
    width: float
    height: float
    area: float
    geometry_width: float
    geometry_height: float
    geometry_area: float
    bottom_y: float
    estimated_distance_m: Optional[float]
    keep: bool = True
    drop_reason: str = ""
    keep_reason: str = "kept"
    roi_inside: Optional[bool] = None


@dataclass
class ClusterRecord:
    cluster_id: int
    symbol_indices: List[int]
    class_names: List[str]
    group_type: str
    ambiguous: bool
    ambiguous_reason: str
    union_bbox: Tuple[int, int, int, int]
    union_bbox_source: str
    expanded_bbox: Tuple[int, int, int, int]
    expanded_bbox_note: str
    hsv_mask_mode: str
    color_mask_coverage: float
    debug_white_mask_coverage: Optional[float]
    refined_bbox: Tuple[int, int, int, int]
    refined_bbox_source: str
    refined_bbox_fallback_reason: str
    refined_bbox_before_neighbor_clamp: Tuple[int, int, int, int]
    refined_bbox_neighbor_clamped: bool
    neighbor_clamp_reason: str
    neighbor_clamp_against_cluster_id: Optional[int]
    edge_clipped: bool
    edge_clipped_sides: List[str]
    bbox_quality: str
    mean_distance_m: Optional[float]
    merged_from: List[int]
    is_merged_cluster: bool


@dataclass
class MergeStep:
    before_cluster_ids: List[int]
    after_cluster_id: int
    symbol_indices: List[int]
    reason: str
    metrics: Dict[str, Any]


@dataclass
class MergeDiagnostic:
    cluster_ids: List[int]
    accepted: bool
    merge_method: str
    reason: str
    metrics: Dict[str, Any]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline single-image KFS instance prototype v2."
    )
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--config", default=None)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--team-color", choices=("red", "blue"), default="red")
    parser.add_argument("--max-distance-m", type=float, default=1.0)
    parser.add_argument("--min-distance-m", type=float, default=0.2)
    parser.add_argument("--camera-fx", type=float, default=800.0)
    parser.add_argument("--camera-fy", type=float, default=800.0)
    parser.add_argument("--camera-cx", type=float, default=320.0)
    parser.add_argument("--camera-cy", type=float, default=320.0)
    parser.add_argument("--camera-height-m", type=float, default=1.1)
    parser.add_argument("--camera-tilt-deg", type=float, default=45.0)
    parser.add_argument("--cluster-distance-scale", type=float, default=2.5)
    parser.add_argument("--min-cluster-distance-px", type=float, default=40.0)
    parser.add_argument("--max-cluster-distance-px", type=float, default=180.0)
    parser.add_argument("--max-depth-diff-m", type=float, default=0.25)
    parser.add_argument("--max-symbols-per-instance", type=int, default=3)
    parser.add_argument("--bbox-expand-scale", type=float, default=2.0)
    return parser.parse_args()


def fail(message: str) -> int:
    print(f"[ERROR] {message}", file=sys.stderr)
    return 1


def load_model(model_path: Path):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "Ultralytics is required for this prototype. "
            "Install it with `pip install ultralytics`."
        ) from exc

    try:
        return YOLO(str(model_path))
    except Exception as exc:
        raise RuntimeError(f"Failed to load model {model_path}: {exc}") from exc


def deep_merge(base: dict, override: dict) -> dict:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if (
            key in merged and
            isinstance(merged[key], dict) and
            isinstance(value, dict)
        ):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_yaml_config(config_path: Optional[str]) -> Tuple[dict, Optional[str]]:
    resolved_path = Path(config_path).expanduser().resolve() if config_path else None
    config = copy.deepcopy(DEFAULT_CONFIG)

    if resolved_path is None:
        return config, None

    try:
        import yaml  # type: ignore
    except ImportError:
        print(
            "[WARN] PyYAML is not installed. Using built-in prototype defaults.",
            file=sys.stderr,
        )
        return config, str(resolved_path)

    try:
        with resolved_path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}
    except Exception as exc:
        print(
            f"[WARN] Failed to load prototype config {resolved_path}: {exc}. "
            "Using built-in defaults.",
            file=sys.stderr,
        )
        return config, str(resolved_path)

    if not isinstance(loaded, dict):
        print(
            f"[WARN] Prototype config {resolved_path} is not a mapping. "
            "Using built-in defaults.",
            file=sys.stderr,
        )
        return config, str(resolved_path)

    return deep_merge(config, loaded), str(resolved_path)


def resolve_hsv_mask_config(config: dict):
    section = config.get("hsv_mask", {})
    if not isinstance(section, dict):
        raise ValueError("hsv_mask config must be a mapping")

    profiles = section.get("profiles")
    active_profile = section.get("active_profile")
    if isinstance(profiles, dict) and active_profile is not None:
        if active_profile not in profiles:
            raise ValueError(
                f"hsv_mask.active_profile '{active_profile}' not found in hsv_mask.profiles"
            )
        profile = profiles[active_profile]
        if not isinstance(profile, dict):
            raise ValueError(
                f"hsv_mask.profiles['{active_profile}'] must be a mapping"
            )
        if profile.get("debug_only", False) and not profile.get("enabled", True):
            raise ValueError(
                f"hsv_mask profile '{active_profile}' is debug_only and disabled"
            )

        resolved = copy.deepcopy(section)
        resolved.update(profile)
        resolved["resolved_profile"] = active_profile
        resolved["resolved_from_profiles"] = True
        config["hsv_mask"] = resolved
        return

    if "mode" in section:
        section["resolved_profile"] = section.get("resolved_profile", "legacy_flat")
        section["resolved_from_profiles"] = False
        return

    raise ValueError(
        "hsv_mask config must define either active_profile/profiles or legacy flat mode fields"
    )


def apply_roi_preset(config: dict):
    roi = config["roi_filter"]
    preset = roi.get("preset", "none")
    presets = {
        "center_working_area": {
            "x_min_norm": 0.10,
            "x_max_norm": 0.78,
            "y_min_norm": 0.22,
            "y_max_norm": 1.00,
        },
        "left_working_area": {
            "x_min_norm": 0.00,
            "x_max_norm": 0.60,
            "y_min_norm": 0.25,
            "y_max_norm": 1.00,
        },
        "right_working_area": {
            "x_min_norm": 0.40,
            "x_max_norm": 1.00,
            "y_min_norm": 0.25,
            "y_max_norm": 1.00,
        },
    }
    if preset in presets:
        roi.update(presets[preset])


def apply_config_to_args(args, config: dict):
    parser_defaults = {
        "max_distance_m": 1.0,
        "min_distance_m": 0.2,
        "cluster_distance_scale": 2.5,
        "min_cluster_distance_px": 40.0,
        "max_cluster_distance_px": 180.0,
        "max_depth_diff_m": 0.25,
        "max_symbols_per_instance": 3,
        "bbox_expand_scale": 2.0,
        "conf": 0.25,
    }

    mappings = {
        "max_distance_m": ("range_filter", "max_distance_m"),
        "min_distance_m": ("range_filter", "min_distance_m"),
        "cluster_distance_scale": ("clustering", "cluster_distance_scale"),
        "min_cluster_distance_px": ("clustering", "min_cluster_distance_px"),
        "max_cluster_distance_px": ("clustering", "max_cluster_distance_px"),
        "max_depth_diff_m": ("clustering", "max_depth_diff_m"),
        "max_symbols_per_instance": ("clustering", "max_symbols_per_instance"),
        "bbox_expand_scale": ("bbox", "expand_scale"),
        "conf": ("range_filter", "min_confidence"),
    }

    for arg_name, (section, key) in mappings.items():
        current = getattr(args, arg_name)
        if current == parser_defaults[arg_name]:
            setattr(args, arg_name, config[section][key])


def resolve_class_name(model, class_id: int) -> str:
    names = getattr(model, "names", None)
    if isinstance(names, dict):
        return str(names.get(class_id, "unknown"))
    if isinstance(names, list) and 0 <= class_id < len(names):
        return str(names[class_id])
    return "unknown"


def clamp_bbox(
    bbox: Tuple[float, float, float, float],
    image_width: int,
    image_height: int,
) -> Tuple[int, int, int, int]:
    x1 = max(0, min(image_width - 1, int(round(bbox[0]))))
    y1 = max(0, min(image_height - 1, int(round(bbox[1]))))
    x2 = max(0, min(image_width - 1, int(round(bbox[2]))))
    y2 = max(0, min(image_height - 1, int(round(bbox[3]))))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return (x1, y1, x2, y2)


def bbox_center(bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
    return (0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3]))


def bbox_width_height_area(bbox: Tuple[int, int, int, int]) -> Tuple[float, float, float]:
    width = max(1.0, float(bbox[2] - bbox[0]))
    height = max(1.0, float(bbox[3] - bbox[1]))
    return width, height, width * height


def make_square_bbox(
    bbox: Tuple[int, int, int, int],
    image_width: int,
    image_height: int,
    scale: float,
    max_square_side_ratio_of_image: float,
) -> Tuple[int, int, int, int]:
    cx, cy = bbox_center(bbox)
    width, height, _ = bbox_width_height_area(bbox)
    side = max(width, height) * max(1e-6, scale)
    max_side = max(1.0, max_square_side_ratio_of_image * float(max(image_width, image_height)))
    side = min(side, max_side)
    half_side = 0.5 * side
    return clamp_bbox(
        (cx - half_side, cy - half_side, cx + half_side, cy + half_side),
        image_width,
        image_height,
    )


def estimate_symbol_distance(
    symbol: SymbolRecord,
    args,
) -> Optional[float]:
    bottom_center_v = symbol.bbox_xyxy[3]
    if args.camera_fy <= 0.0 or args.camera_height_m <= 0.0:
        return None

    tilt_rad = math.radians(args.camera_tilt_deg)
    pixel_angle = math.atan2(bottom_center_v - args.camera_cy, args.camera_fy)
    total_downward_angle = tilt_rad + pixel_angle
    if total_downward_angle <= 1e-6:
        return None

    tangent = math.tan(total_downward_angle)
    if tangent <= 1e-6:
        return None

    distance = args.camera_height_m / tangent
    if not math.isfinite(distance) or distance <= 0.0:
        return None

    return distance


def infer_symbols(model, image: np.ndarray, args, config: dict) -> List[SymbolRecord]:
    try:
        results = model.predict(
            source=image,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
            verbose=False,
        )
    except Exception as exc:
        raise RuntimeError(f"Model inference failed: {exc}") from exc

    if not results:
        return []

    result = results[0]
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []

    image_height, image_width = image.shape[:2]
    xyxy_values = boxes.xyxy.cpu().tolist() if hasattr(boxes, "xyxy") else []
    conf_values = boxes.conf.cpu().tolist() if hasattr(boxes, "conf") else []
    class_values = boxes.cls.cpu().tolist() if hasattr(boxes, "cls") else []

    symbols: List[SymbolRecord] = []
    for index, (xyxy, conf, cls_value) in enumerate(
        zip(xyxy_values, conf_values, class_values)
    ):
        class_id = int(cls_value)
        class_name = resolve_class_name(model, class_id)
        raw_bbox = clamp_bbox(tuple(float(v) for v in xyxy), image_width, image_height)
        bbox_cfg = config["bbox"]
        if bbox_cfg.get("use_square_symbol_bbox", False):
            geometry_bbox = make_square_bbox(
                raw_bbox,
                image_width,
                image_height,
                float(bbox_cfg.get("square_symbol_scale", 1.0)),
                float(bbox_cfg.get("max_square_side_ratio_of_image", 0.35)),
            )
            geometry_bbox_source = "square_symbol_bbox"
        else:
            geometry_bbox = raw_bbox
            geometry_bbox_source = "raw_yolo_bbox"

        x1, y1, x2, y2 = raw_bbox
        width, height, area = bbox_width_height_area(raw_bbox)
        geometry_width, geometry_height, geometry_area = bbox_width_height_area(geometry_bbox)
        symbol = SymbolRecord(
            index=index,
            class_id=class_id,
            class_name=class_name,
            confidence=float(conf),
            bbox_xyxy=raw_bbox,
            raw_bbox=raw_bbox,
            geometry_bbox=geometry_bbox,
            geometry_bbox_source=geometry_bbox_source,
            center_xy=bbox_center(raw_bbox),
            raw_center_xy=bbox_center(raw_bbox),
            geometry_center_xy=bbox_center(geometry_bbox),
            width=width,
            height=height,
            area=area,
            geometry_width=geometry_width,
            geometry_height=geometry_height,
            geometry_area=geometry_area,
            bottom_y=float(y2),
            estimated_distance_m=None,
        )
        symbol.estimated_distance_m = estimate_symbol_distance(symbol, args)
        symbols.append(symbol)

    return symbols


def format_distance(distance_m: Optional[float]) -> str:
    if distance_m is None:
        return "unknown"
    return f"{distance_m:.3f}"


def apply_range_filter(symbols: Sequence[SymbolRecord], config: dict) -> List[SymbolRecord]:
    filtered: List[SymbolRecord] = []
    section = config["range_filter"]
    mode = section["mode"]

    for symbol in symbols:
        reasons: List[str] = []

        if symbol.confidence < section["min_confidence"]:
            reasons.append(
                f"confidence {symbol.confidence:.2f} < min_confidence {section['min_confidence']:.2f}"
            )

        if mode in ("bbox_size", "projection"):
            if symbol.height < section["min_symbol_height_px"]:
                reasons.append(
                    f"bbox_height {symbol.height:.1f} < min_symbol_height_px {section['min_symbol_height_px']}"
                )
            if symbol.area < section["min_symbol_area_px"]:
                reasons.append(
                    f"bbox_area {symbol.area:.1f} < min_symbol_area_px {section['min_symbol_area_px']}"
                )

        if mode == "projection":
            if symbol.estimated_distance_m is not None:
                if symbol.estimated_distance_m > section["max_distance_m"]:
                    reasons.append(
                        f"distance {symbol.estimated_distance_m:.3f}m > max_distance_m {section['max_distance_m']:.3f}m"
                    )
                if symbol.estimated_distance_m < section["min_distance_m"]:
                    reasons.append(
                        f"distance {symbol.estimated_distance_m:.3f}m < min_distance_m {section['min_distance_m']:.3f}m"
                    )

        symbol.keep = len(reasons) == 0
        if symbol.keep:
            if mode == "none":
                symbol.keep_reason = "keep conf>=min_confidence"
            elif mode == "bbox_size":
                symbol.keep_reason = "keep bbox_size_constraints_pass"
            else:
                if symbol.estimated_distance_m is None:
                    symbol.keep_reason = (
                        "keep projection_distance_unknown_bbox_constraints_pass"
                    )
                else:
                    symbol.keep_reason = "keep projection_and_bbox_constraints_pass"
            symbol.drop_reason = ""
            filtered.append(symbol)
        else:
            symbol.drop_reason = "; ".join(reasons)
            symbol.keep_reason = f"drop {symbol.drop_reason}"

    return filtered


def normalized_point_to_pixel(
    point: Sequence[float],
    image_width: int,
    image_height: int,
) -> Tuple[int, int]:
    return (
        int(round(float(point[0]) * max(0, image_width - 1))),
        int(round(float(point[1]) * max(0, image_height - 1))),
    )


def point_inside_roi(
    center_xy: Tuple[float, float],
    image_width: int,
    image_height: int,
    config: dict,
) -> bool:
    roi = config["roi_filter"]
    if not roi["enabled"]:
        return True

    x_norm = center_xy[0] / float(max(1, image_width))
    y_norm = center_xy[1] / float(max(1, image_height))
    mode = roi["mode"]

    if mode == "normalized_polygon":
        polygon = roi.get("polygon", [])
        if len(polygon) < 3:
            return True
        polygon_points = np.array(
            [normalized_point_to_pixel(point, image_width, image_height) for point in polygon],
            dtype=np.int32,
        )
        return cv2.pointPolygonTest(
            polygon_points,
            (float(center_xy[0]), float(center_xy[1])),
            False,
        ) >= 0

    return (
        roi["x_min_norm"] <= x_norm <= roi["x_max_norm"] and
        roi["y_min_norm"] <= y_norm <= roi["y_max_norm"]
    )


def apply_roi_filter(
    symbols: Sequence[SymbolRecord],
    image_width: int,
    image_height: int,
    config: dict,
) -> List[SymbolRecord]:
    roi = config["roi_filter"]
    if not roi["enabled"]:
        for symbol in symbols:
            symbol.roi_inside = True
        return list(symbols)

    filtered: List[SymbolRecord] = []
    for symbol in symbols:
        inside = point_inside_roi(symbol.center_xy, image_width, image_height, config)
        symbol.roi_inside = inside
        if inside:
            filtered.append(symbol)
            continue

        symbol.keep = False
        if symbol.drop_reason:
            symbol.drop_reason = f"{symbol.drop_reason}; outside_roi"
        else:
            symbol.drop_reason = "outside_roi"
        symbol.keep_reason = f"drop {symbol.drop_reason}"

    return filtered


def clustering_center(symbol: SymbolRecord, config: dict) -> Tuple[float, float]:
    source = config["bbox"].get("clustering_center_source", "raw")
    if source == "geometry":
        return symbol.geometry_center_xy
    return symbol.raw_center_xy


def symbol_size_metric(symbol: SymbolRecord) -> float:
    return math.sqrt(max(1.0, symbol.area))


def similarity_ratio(first: float, second: float) -> float:
    high = max(first, second)
    low = min(first, second)
    if high <= 1e-6:
        return 0.0
    return low / high


def pairwise_cluster_debug(
    first: SymbolRecord,
    second: SymbolRecord,
    center_distance: float,
    threshold_px: float,
    height_similarity: float,
    area_similarity: float,
    bottom_diff: float,
) -> str:
    return (
        f"pair({first.index},{second.index}) "
        f"center_dist={center_distance:.1f} "
        f"threshold_px={threshold_px:.1f} "
        f"height_similarity={height_similarity:.2f} "
        f"area_similarity={area_similarity:.2f} "
        f"bottom_y_diff={bottom_diff:.1f}"
    )


def should_link_symbols(
    first: SymbolRecord,
    second: SymbolRecord,
    config: dict,
) -> Tuple[bool, str]:
    section = config["clustering"]
    first_center = clustering_center(first, config)
    second_center = clustering_center(second, config)
    center_distance = math.hypot(
        first_center[0] - second_center[0],
        first_center[1] - second_center[1],
    )
    average_symbol_size = 0.5 * (
        symbol_size_metric(first) + symbol_size_metric(second)
    )
    threshold_px = section["cluster_distance_scale"] * average_symbol_size
    threshold_px = min(
        section["max_cluster_distance_px"],
        max(section["min_cluster_distance_px"], threshold_px),
    )

    height_similarity = similarity_ratio(first.height, second.height)
    area_similarity = similarity_ratio(first.area, second.area)
    avg_height = 0.5 * (first.height + second.height)
    bottom_diff = abs(first.bottom_y - second.bottom_y)

    allowed = center_distance <= threshold_px
    allowed &= height_similarity >= section["min_height_similarity_for_grouping"]
    allowed &= area_similarity >= section["min_area_similarity_for_grouping"]
    allowed &= bottom_diff <= section["max_bottom_y_diff_ratio"] * avg_height

    if (
        first.estimated_distance_m is not None and
        second.estimated_distance_m is not None
    ):
        allowed &= (
            abs(first.estimated_distance_m - second.estimated_distance_m) <=
            section["max_depth_diff_m"]
        )

    return allowed, pairwise_cluster_debug(
        first,
        second,
        center_distance,
        threshold_px,
        height_similarity,
        area_similarity,
        bottom_diff,
    )


class UnionFind:
    def __init__(self, size: int):
        self.parent = list(range(size))

    def find(self, value: int) -> int:
        while self.parent[value] != value:
            self.parent[value] = self.parent[self.parent[value]]
            value = self.parent[value]
        return value

    def union(self, first: int, second: int):
        root_a = self.find(first)
        root_b = self.find(second)
        if root_a != root_b:
            self.parent[root_b] = root_a


def cluster_symbols(symbols: Sequence[SymbolRecord], config: dict) -> List[List[SymbolRecord]]:
    if not symbols:
        return []

    union_find = UnionFind(len(symbols))
    for i in range(len(symbols)):
        for j in range(i + 1, len(symbols)):
            allowed, debug_text = should_link_symbols(symbols[i], symbols[j], config)
            print(f"  {debug_text} link={str(allowed).lower()}")
            if allowed:
                union_find.union(i, j)

    grouped: Dict[int, List[SymbolRecord]] = {}
    for index, symbol in enumerate(symbols):
        root = union_find.find(index)
        grouped.setdefault(root, []).append(symbol)

    return list(grouped.values())


def semantic_group_type(
    class_names: Sequence[str],
    ambiguous: bool,
) -> Tuple[str, str]:
    if ambiguous:
        return "AMBIGUOUS", "too_many_symbols_possible_merged_kfs"

    if any(name == "R1" for name in class_names):
        return "R1", ""
    if any(name.startswith("FAKE_") for name in class_names):
        return "FAKE", ""
    if class_names and all(name.startswith("REAL_") for name in class_names):
        return "REAL", ""
    return "UNKNOWN", ""


def union_bbox(symbols: Sequence[SymbolRecord]) -> Tuple[int, int, int, int]:
    return (
        min(symbol.geometry_bbox[0] for symbol in symbols),
        min(symbol.geometry_bbox[1] for symbol in symbols),
        max(symbol.geometry_bbox[2] for symbol in symbols),
        max(symbol.geometry_bbox[3] for symbol in symbols),
    )


def bbox_iou(
    first: Tuple[int, int, int, int],
    second: Tuple[int, int, int, int],
) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    inter_w = max(0.0, float(x2 - x1))
    inter_h = max(0.0, float(y2 - y1))
    intersection = inter_w * inter_h
    if intersection <= 0.0:
        return 0.0
    area_a = max(1.0, float(first[2] - first[0]) * float(first[3] - first[1]))
    area_b = max(1.0, float(second[2] - second[0]) * float(second[3] - second[1]))
    union = area_a + area_b - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union


def bbox_intersection_over_min_area(
    first: Tuple[int, int, int, int],
    second: Tuple[int, int, int, int],
) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    inter_w = max(0.0, float(x2 - x1))
    inter_h = max(0.0, float(y2 - y1))
    intersection = inter_w * inter_h
    if intersection <= 0.0:
        return 0.0
    area_a = max(1.0, float(first[2] - first[0]) * float(first[3] - first[1]))
    area_b = max(1.0, float(second[2] - second[0]) * float(second[3] - second[1]))
    return intersection / min(area_a, area_b)


def bbox_gap_px(
    first: Tuple[int, int, int, int],
    second: Tuple[int, int, int, int],
) -> float:
    dx = max(float(second[0] - first[2]), float(first[0] - second[2]), 0.0)
    dy = max(float(second[1] - first[3]), float(first[1] - second[3]), 0.0)
    return math.hypot(dx, dy)


def bbox_center_distance(
    first: Tuple[int, int, int, int],
    second: Tuple[int, int, int, int],
) -> float:
    first_cx = 0.5 * (first[0] + first[2])
    first_cy = 0.5 * (first[1] + first[3])
    second_cx = 0.5 * (second[0] + second[2])
    second_cy = 0.5 * (second[1] + second[3])
    return math.hypot(first_cx - second_cx, first_cy - second_cy)


def semantic_groups_compatible(
    group_a: str,
    group_b: str,
    merge_same_group_only: bool,
) -> bool:
    if group_a == "AMBIGUOUS" or group_b == "AMBIGUOUS":
        return False
    if merge_same_group_only:
        if group_a == "UNKNOWN" or group_b == "UNKNOWN":
            return group_a == "UNKNOWN" and group_b == "UNKNOWN"
        return group_a == group_b
    if group_a == group_b:
        return True
    if group_a == "UNKNOWN" or group_b == "UNKNOWN":
        return False
    return False


def expand_bbox_with_limit(
    bbox: Tuple[int, int, int, int],
    image_width: int,
    image_height: int,
    config: dict,
) -> Tuple[Tuple[int, int, int, int], str]:
    scale = config["bbox"]["expand_scale"]
    scale_x = config["bbox"].get("expand_scale_x", scale)
    scale_y = config["bbox"].get("expand_scale_y", scale)
    max_ratio = config["bbox"]["max_expanded_area_ratio"]
    x1, y1, x2, y2 = bbox
    width = max(1.0, float(x2 - x1))
    height = max(1.0, float(y2 - y1))
    center_x = 0.5 * (x1 + x2)
    center_y = 0.5 * (y1 + y2)

    adjusted_scale_x = scale_x
    adjusted_scale_y = scale_y
    note = "expanded_nominal"
    image_area = float(image_width * image_height)

    for _ in range(3):
        half_width = 0.5 * width * adjusted_scale_x
        half_height = 0.5 * height * adjusted_scale_y
        expanded = clamp_bbox(
            (
                center_x - half_width,
                center_y - half_height,
                center_x + half_width,
                center_y + half_height,
            ),
            image_width,
            image_height,
        )
        area = max(1.0, float(expanded[2] - expanded[0]) * float(expanded[3] - expanded[1]))
        if area / image_area <= max_ratio:
            return expanded, note

        scale_factor = math.sqrt(max_ratio / max(area / image_area, 1e-6))
        adjusted_scale_x = max(1.0, adjusted_scale_x * scale_factor)
        adjusted_scale_y = max(1.0, adjusted_scale_y * scale_factor)
        note = (
            f"expanded_clamped_large_area ratio={area / image_area:.3f} "
            f"> max_expanded_area_ratio {max_ratio:.3f}"
        )

    return expanded, note


def select_hsv_mask(crop_bgr: np.ndarray, team_color: str, config: dict) -> np.ndarray:
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    section = config["hsv_mask"]
    mode = section["mode"]
    if mode == "team":
        mode = team_color

    masks: List[np.ndarray] = []

    if mode in ("red", "red_or_blue", "red_or_blue_or_dark_blue"):
        red_1 = cv2.inRange(
            hsv,
            np.array([section["red_h_low_1"], section["red_s_low"], section["red_v_low"]], dtype=np.uint8),
            np.array([section["red_h_high_1"], 255, 255], dtype=np.uint8),
        )
        red_2 = cv2.inRange(
            hsv,
            np.array([section["red_h_low_2"], section["red_s_low"], section["red_v_low"]], dtype=np.uint8),
            np.array([section["red_h_high_2"], 255, 255], dtype=np.uint8),
        )
        masks.append(cv2.bitwise_or(red_1, red_2))

    if mode in ("blue", "red_or_blue", "blue_or_dark_blue", "red_or_blue_or_dark_blue"):
        masks.append(
            cv2.inRange(
                hsv,
                np.array([section["blue_h_low"], section["blue_s_low"], section["blue_v_low"]], dtype=np.uint8),
                np.array([section["blue_h_high"], 255, 255], dtype=np.uint8),
            )
        )

    if mode in ("dark_blue", "blue_or_dark_blue", "red_or_blue_or_dark_blue"):
        masks.append(
            cv2.inRange(
                hsv,
                np.array([section["dark_blue_h_low"], section["dark_blue_s_low"], section["dark_blue_v_low"]], dtype=np.uint8),
                np.array([section["dark_blue_h_high"], 255, section["dark_blue_v_high"]], dtype=np.uint8),
            )
        )

    if not masks:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    mask = masks[0]
    for extra in masks[1:]:
        mask = cv2.bitwise_or(mask, extra)
    return mask


def compute_debug_white_mask(crop_bgr: np.ndarray, config: dict) -> Optional[np.ndarray]:
    section = config["debug_white_mask"]
    if not section["enabled"]:
        return None
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(
        hsv,
        np.array([0, 0, section["v_low"]], dtype=np.uint8),
        np.array([180, section["s_high"], 255], dtype=np.uint8),
    )


def apply_morphology(mask: np.ndarray, config: dict) -> np.ndarray:
    section = config["contour"]
    kernel_size = max(1, int(section["morphology_kernel_size"]))
    iterations = max(1, int(section["morphology_iterations"]))
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    output = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel, iterations=iterations)
    return output


def contour_centroid(contour: np.ndarray) -> Tuple[float, float]:
    moments = cv2.moments(contour)
    if moments["m00"] <= 1e-6:
        x, y, w, h = cv2.boundingRect(contour)
        return (x + 0.5 * w, y + 0.5 * h)
    return (
        moments["m10"] / moments["m00"],
        moments["m01"] / moments["m00"],
    )


def refine_cluster_bbox(
    image: np.ndarray,
    cluster: ClusterRecord,
    team_color: str,
    config: dict,
) -> Tuple[float, Optional[float], Tuple[int, int, int, int], str, str, np.ndarray, Optional[np.ndarray], np.ndarray]:
    x1, y1, x2, y2 = cluster.expanded_bbox
    crop = image[y1:y2, x1:x2]
    if crop.size == 0:
        empty_mask = np.zeros((max(1, y2 - y1), max(1, x2 - x1)), dtype=np.uint8)
        return 0.0, None, cluster.expanded_bbox, "expanded_bbox_fallback", "empty_crop", empty_mask, None, crop

    color_mask = select_hsv_mask(crop, team_color, config)
    color_mask = apply_morphology(color_mask, config)
    color_mask_coverage = float(cv2.countNonZero(color_mask)) / float(max(1, color_mask.size))

    debug_white_mask = compute_debug_white_mask(crop, config)
    debug_white_mask_coverage = None
    if debug_white_mask is not None:
        debug_white_mask_coverage = float(cv2.countNonZero(debug_white_mask)) / float(max(1, debug_white_mask.size))

    contour_config = config["contour"]
    if not contour_config["enabled"]:
        return (
            color_mask_coverage,
            debug_white_mask_coverage,
            cluster.expanded_bbox,
            "expanded_bbox_fallback",
            "contour_disabled",
            color_mask,
            debug_white_mask,
            crop,
        )

    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [
        contour for contour in contours
        if cv2.contourArea(contour) >= contour_config["min_contour_area_px"]
    ]

    if not valid_contours:
        return (
            color_mask_coverage,
            debug_white_mask_coverage,
            cluster.expanded_bbox,
            "expanded_bbox_fallback",
            "no_valid_color_contour",
            color_mask,
            debug_white_mask,
            crop,
        )

    selected = valid_contours[0]
    if contour_config["selection"] == "largest":
        selected = max(valid_contours, key=cv2.contourArea)
    else:
        union_cx = 0.5 * (cluster.union_bbox[0] + cluster.union_bbox[2]) - x1
        union_cy = 0.5 * (cluster.union_bbox[1] + cluster.union_bbox[3]) - y1
        selected = min(
            valid_contours,
            key=lambda contour: math.hypot(
                contour_centroid(contour)[0] - union_cx,
                contour_centroid(contour)[1] - union_cy,
            ),
        )

    rx, ry, rw, rh = cv2.boundingRect(selected)
    refined = (x1 + rx, y1 + ry, x1 + rx + rw, y1 + ry + rh)
    union_w = max(1.0, float(cluster.union_bbox[2] - cluster.union_bbox[0]))
    union_h = max(1.0, float(cluster.union_bbox[3] - cluster.union_bbox[1]))
    union_area = union_w * union_h
    union_diag = math.hypot(union_w, union_h)
    union_cx = 0.5 * (cluster.union_bbox[0] + cluster.union_bbox[2])
    union_cy = 0.5 * (cluster.union_bbox[1] + cluster.union_bbox[3])
    refined_cx = 0.5 * (refined[0] + refined[2])
    refined_cy = 0.5 * (refined[1] + refined[3])
    center_offset = math.hypot(refined_cx - union_cx, refined_cy - union_cy)
    refined_area = max(1.0, float((refined[2] - refined[0]) * (refined[3] - refined[1])))
    if center_offset > contour_config["max_center_offset_ratio"] * union_diag:
        return (
            color_mask_coverage,
            debug_white_mask_coverage,
            cluster.expanded_bbox,
            "expanded_bbox_fallback",
            "contour_center_too_far_from_union_center",
            color_mask,
            debug_white_mask,
            crop,
        )
    if refined_area < contour_config["min_refined_to_union_area_ratio"] * union_area:
        return (
            color_mask_coverage,
            debug_white_mask_coverage,
            cluster.expanded_bbox,
            "expanded_bbox_fallback",
            "refined_bbox_too_small_vs_union_bbox",
            color_mask,
            debug_white_mask,
            crop,
        )
    return (
        color_mask_coverage,
        debug_white_mask_coverage,
        refined,
        "color_contour",
        "",
        color_mask,
        debug_white_mask,
        crop,
    )


def mean_distance(symbols: Sequence[SymbolRecord]) -> Optional[float]:
    values = [
        symbol.estimated_distance_m
        for symbol in symbols
        if symbol.estimated_distance_m is not None
    ]
    if not values:
        return None
    return float(sum(values) / len(values))


def build_cluster_record(
    cluster_id: int,
    group: Sequence[SymbolRecord],
    image: np.ndarray,
    args,
    config: dict,
    merged_from: Optional[List[int]] = None,
) -> Tuple[ClusterRecord, Tuple[int, np.ndarray, np.ndarray, Optional[np.ndarray], Tuple[int, int, int, int], str]]:
    image_height, image_width = image.shape[:2]
    _ = image_height
    ambiguous = len(group) > config["clustering"]["max_symbols_per_instance"]
    group_type, ambiguous_reason = semantic_group_type(
        [symbol.class_name for symbol in group],
        ambiguous,
    )
    if ambiguous:
        print(
            f"Cluster[{cluster_id}] AMBIGUOUS: size={len(group)} > "
            f"max_symbols_per_instance={config['clustering']['max_symbols_per_instance']}"
        )

    union = union_bbox(group)
    expanded, expanded_note = expand_bbox_with_limit(
        union,
        image_width,
        image.shape[0],
        config,
    )
    if expanded_note != "expanded_nominal":
        print(f"Cluster[{cluster_id}] expanded bbox adjusted: {expanded_note}")

    merged_from_ids = sorted(merged_from or [cluster_id])
    record = ClusterRecord(
        cluster_id=cluster_id,
        symbol_indices=sorted(symbol.index for symbol in group),
        class_names=[symbol.class_name for symbol in group],
        group_type=group_type,
        ambiguous=ambiguous,
        ambiguous_reason=ambiguous_reason,
        union_bbox=union,
        union_bbox_source=(
            "geometry_bbox" if config["bbox"].get("use_square_symbol_bbox", False)
            else "raw_yolo_bbox"
        ),
        expanded_bbox=expanded,
        expanded_bbox_note=expanded_note,
        hsv_mask_mode=config["hsv_mask"]["mode"],
        color_mask_coverage=0.0,
        debug_white_mask_coverage=None,
        refined_bbox=expanded,
        refined_bbox_source="expanded_bbox_fallback",
        refined_bbox_fallback_reason="",
        refined_bbox_before_neighbor_clamp=expanded,
        refined_bbox_neighbor_clamped=False,
        neighbor_clamp_reason="",
        neighbor_clamp_against_cluster_id=None,
        edge_clipped=False,
        edge_clipped_sides=[],
        bbox_quality="normal",
        mean_distance_m=mean_distance(group),
        merged_from=merged_from_ids,
        is_merged_cluster=len(merged_from_ids) > 1,
    )

    (
        color_mask_coverage,
        debug_white_mask_coverage,
        refined_bbox,
        refined_source,
        fallback_reason,
        color_mask,
        debug_white_mask,
        crop,
    ) = refine_cluster_bbox(image, record, args.team_color, config)

    record.color_mask_coverage = color_mask_coverage
    record.debug_white_mask_coverage = debug_white_mask_coverage
    record.refined_bbox = refined_bbox
    record.refined_bbox_before_neighbor_clamp = refined_bbox
    record.refined_bbox_source = refined_source
    record.refined_bbox_fallback_reason = fallback_reason

    return record, (
        cluster_id,
        crop,
        color_mask,
        debug_white_mask,
        refined_bbox,
        refined_source,
    )


def merge_metrics(
    first: ClusterRecord,
    second: ClusterRecord,
    merged_symbols: Sequence[SymbolRecord],
    config: dict,
) -> Dict[str, Any]:
    distance_diff = None
    if first.mean_distance_m is not None and second.mean_distance_m is not None:
        distance_diff = abs(first.mean_distance_m - second.mean_distance_m)
    merged_union = union_bbox(merged_symbols)
    merged_union_width = max(1.0, float(merged_union[2] - merged_union[0]))
    merged_union_height = max(1.0, float(merged_union[3] - merged_union[1]))
    largest_symbol_width = max(max(1.0, symbol.geometry_width) for symbol in merged_symbols)
    largest_symbol_height = max(max(1.0, symbol.geometry_height) for symbol in merged_symbols)
    width_scale = merged_union_width / largest_symbol_width
    height_scale = merged_union_height / largest_symbol_height
    compactness_pass = (
        width_scale <= config["cluster_merge"]["max_merged_union_width_scale"] and
        height_scale <= config["cluster_merge"]["max_merged_union_height_scale"]
    )
    strong_spatial_pass = (
        bbox_iou(first.expanded_bbox, second.expanded_bbox) >= config["cluster_merge"]["min_expanded_iou"] or
        bbox_intersection_over_min_area(first.expanded_bbox, second.expanded_bbox) >=
        config["cluster_merge"]["min_expanded_intersection_over_min_area"] or
        bbox_gap_px(first.expanded_bbox, second.expanded_bbox) <= config["cluster_merge"]["max_expanded_gap_px"]
    )
    center_distance_only_used = (
        not strong_spatial_pass and
        bbox_center_distance(first.union_bbox, second.union_bbox) <=
        config["cluster_merge"]["max_union_center_distance_px"]
    )
    return {
        "iou": bbox_iou(first.expanded_bbox, second.expanded_bbox),
        "intersection_over_min_area": bbox_intersection_over_min_area(
            first.expanded_bbox,
            second.expanded_bbox,
        ),
        "gap_px": bbox_gap_px(first.expanded_bbox, second.expanded_bbox),
        "center_distance_px": bbox_center_distance(first.union_bbox, second.union_bbox),
        "distance_diff_m": distance_diff,
        "width_scale": width_scale,
        "height_scale": height_scale,
        "compactness_pass": compactness_pass,
        "strong_spatial_pass": strong_spatial_pass,
        "center_distance_only_used": center_distance_only_used,
        "merge_method": "normal",
        "fallback_reason": "",
        "fallback_checks": {},
        "accepted": False,
    }


def bbox_height(bbox: Tuple[int, int, int, int]) -> float:
    return max(1.0, float(bbox[3] - bbox[1]))


def bbox_bottom_y(bbox: Tuple[int, int, int, int]) -> float:
    return float(bbox[3])


def low_mask_fallback_candidate_allowed(
    first: ClusterRecord,
    second: ClusterRecord,
    merged_symbols: Sequence[SymbolRecord],
    config: dict,
    normal_reason: str,
    metrics: Dict[str, Any],
) -> Tuple[bool, str, Dict[str, Any]]:
    fallback_cfg = config.get("merge", {}).get("low_mask_adjacent_same_group_fallback", {})
    fallback_metrics = dict(metrics)
    fallback_metrics["merge_method"] = "low_mask_adjacent_same_group_fallback"

    if not fallback_cfg.get("enabled", False):
        fallback_metrics["fallback_reason"] = "fallback_disabled"
        return False, "fallback_disabled", fallback_metrics

    if normal_reason not in ("first_cluster_color_mask_too_low", "second_cluster_color_mask_too_low"):
        fallback_metrics["fallback_reason"] = "normal_rejection_not_weak_color_mask"
        return False, "normal_rejection_not_weak_color_mask", fallback_metrics

    total_symbols = len({symbol.index for symbol in merged_symbols})
    if fallback_cfg.get("reject_ambiguous", True) and (first.ambiguous or second.ambiguous):
        fallback_metrics["fallback_reason"] = "ambiguous_cluster_not_mergeable"
        return False, "ambiguous_cluster_not_mergeable", fallback_metrics
    if total_symbols > fallback_cfg.get("max_symbols_after_merge", 3):
        fallback_metrics["fallback_reason"] = "too_many_symbols_after_merge"
        return False, "too_many_symbols_after_merge", fallback_metrics
    if fallback_cfg.get("require_same_group_type", True) and first.group_type != second.group_type:
        fallback_metrics["fallback_reason"] = "fallback_requires_same_group_type"
        return False, "fallback_requires_same_group_type", fallback_metrics
    if fallback_cfg.get("reject_real_fake_mix", True):
        pair = {first.group_type, second.group_type}
        if "REAL" in pair and "FAKE" in pair:
            fallback_metrics["fallback_reason"] = "real_fake_mix_rejected"
            return False, "real_fake_mix_rejected", fallback_metrics

    first_height = bbox_height(first.union_bbox)
    second_height = bbox_height(second.union_bbox)
    height_similarity = similarity_ratio(first_height, second_height)
    avg_height = 0.5 * (first_height + second_height)
    bottom_y_diff = abs(bbox_bottom_y(first.union_bbox) - bbox_bottom_y(second.union_bbox))
    gap_limit = min(
        float(fallback_cfg.get("max_gap_px", 45)),
        float(fallback_cfg.get("max_gap_ratio_to_symbol_height", 0.55)) * avg_height,
    )
    merged_union = union_bbox(merged_symbols)
    merged_union_width = max(1.0, float(merged_union[2] - merged_union[0]))
    merged_union_height = max(1.0, float(merged_union[3] - merged_union[1]))
    merged_aspect_ratio = max(
        merged_union_width / merged_union_height,
        merged_union_height / merged_union_width,
    )

    checks = {
        "height_similarity": height_similarity,
        "height_similarity_pass": (
            (not fallback_cfg.get("require_height_similarity", True)) or
            height_similarity >= float(fallback_cfg.get("min_height_similarity", 0.55))
        ),
        "bottom_y_diff": bottom_y_diff,
        "bottom_y_consistency_pass": (
            (not fallback_cfg.get("require_bottom_y_consistency", True)) or
            bottom_y_diff <= float(fallback_cfg.get("max_bottom_y_diff_ratio", 1.35)) * avg_height
        ),
        "gap_px": metrics["gap_px"],
        "gap_limit_px": gap_limit,
        "small_gap_pass": (
            (not fallback_cfg.get("require_small_gap", True)) or
            metrics["gap_px"] <= gap_limit
        ),
        "merged_aspect_ratio": merged_aspect_ratio,
        "merged_aspect_ratio_pass": merged_aspect_ratio <= float(
            fallback_cfg.get("max_merged_aspect_ratio", 2.2)
        ),
        "width_scale": metrics["width_scale"],
        "width_scale_pass": metrics["width_scale"] <= float(
            fallback_cfg.get("max_merged_width_scale", 2.2)
        ),
        "height_scale": metrics["height_scale"],
        "height_scale_pass": metrics["height_scale"] <= float(
            fallback_cfg.get("max_merged_height_scale", 2.6)
        ),
    }
    fallback_metrics["fallback_checks"] = checks

    failed_checks = [
        name for name, passed in checks.items()
        if name.endswith("_pass") and not bool(passed)
    ]
    if failed_checks:
        fallback_metrics["fallback_reason"] = ",".join(failed_checks)
        return False, f"fallback_geometry_failed:{','.join(failed_checks)}", fallback_metrics

    fallback_metrics["fallback_reason"] = "weak_color_mask_but_strong_geometry"
    return (
        True,
        (
            f"fallback_same_group {first.group_type}, weak_color_mask, "
            f"gap={metrics['gap_px']:.1f}px, hsim={height_similarity:.2f}"
        ),
        fallback_metrics,
    )


def merge_candidate_allowed(
    first: ClusterRecord,
    second: ClusterRecord,
    first_symbols: Sequence[SymbolRecord],
    second_symbols: Sequence[SymbolRecord],
    config: dict,
) -> Tuple[bool, str, Dict[str, Any]]:
    merge_cfg = config["cluster_merge"]
    merged_symbols = list(first_symbols) + list(second_symbols)
    metrics = merge_metrics(first, second, merged_symbols, config)
    total_symbols = len(set(first.symbol_indices + second.symbol_indices))
    if not merge_cfg["enabled"]:
        return False, "cluster_merge_disabled", metrics
    if first.ambiguous or second.ambiguous:
        return False, "ambiguous_cluster_not_mergeable", metrics
    if total_symbols > merge_cfg["max_symbols_after_merge"]:
        return False, "too_many_symbols_after_merge", metrics
    if not semantic_groups_compatible(
        first.group_type,
        second.group_type,
        merge_cfg["merge_same_group_only"],
    ):
        return False, "semantic_groups_incompatible", metrics
    if first.color_mask_coverage < merge_cfg["min_color_mask_coverage_for_merge"]:
        return False, "first_cluster_color_mask_too_low", metrics
    if second.color_mask_coverage < merge_cfg["min_color_mask_coverage_for_merge"]:
        return False, "second_cluster_color_mask_too_low", metrics
    if not metrics["compactness_pass"]:
        if metrics["width_scale"] > merge_cfg["max_merged_union_width_scale"]:
            return False, "merged_union_too_wide_vs_largest_symbol", metrics
        return False, "merged_union_too_tall_vs_largest_symbol", metrics
    if (
        merge_cfg.get("use_distance_check", False) and
        metrics["distance_diff_m"] is not None and
        metrics["distance_diff_m"] > merge_cfg["max_distance_diff_m"]
    ):
        return False, "distance_difference_too_large", metrics

    spatial_ok = bool(metrics["strong_spatial_pass"])
    if (
        not spatial_ok and
        merge_cfg.get("allow_center_distance_only_merge", False) and
        metrics["center_distance_px"] <= merge_cfg["max_union_center_distance_px"]
    ):
        spatial_ok = True
    if not spatial_ok:
        return False, "spatial_conditions_not_met", metrics

    metrics["merge_method"] = "normal"
    metrics["accepted"] = True
    return (
        True,
        (
            f"same_group {first.group_type}, gap={metrics['gap_px']:.1f}px, "
            f"compact w={metrics['width_scale']:.2f} h={metrics['height_scale']:.2f}, color ok"
        ),
        metrics,
    )


def merge_score(metrics: Dict[str, Optional[float]]) -> Tuple[float, float, float]:
    return (
        metrics["iou"] + metrics["intersection_over_min_area"],
        -metrics["gap_px"],
        -metrics["center_distance_px"],
    )


def merge_clusters_second_stage(
    clusters: Sequence[ClusterRecord],
    cluster_groupings: Dict[int, List[SymbolRecord]],
    image: np.ndarray,
    args,
    config: dict,
) -> Tuple[
    List[ClusterRecord],
    Dict[int, List[SymbolRecord]],
    List[Tuple[int, np.ndarray, np.ndarray, Optional[np.ndarray], Tuple[int, int, int, int], str]],
    List[MergeStep],
    List[MergeDiagnostic],
]:
    active_clusters = {cluster.cluster_id: cluster for cluster in clusters}
    active_groupings = {cluster_id: list(group) for cluster_id, group in cluster_groupings.items()}
    merge_steps: List[MergeStep] = []
    merge_diagnostics: List[MergeDiagnostic] = []

    if not config["cluster_merge"]["enabled"]:
        mask_debug: List[Tuple[int, np.ndarray, np.ndarray, Optional[np.ndarray], Tuple[int, int, int, int], str]] = []
        for cluster in clusters:
            _, debug = build_cluster_record(
                cluster.cluster_id,
                active_groupings[cluster.cluster_id],
                image,
                args,
                config,
                merged_from=cluster.merged_from,
            )
            mask_debug.append(debug)
        return list(clusters), active_groupings, mask_debug, merge_steps, merge_diagnostics

    while True:
        best_pair = None
        best_reason = ""
        best_metrics: Dict[str, Any] = {}
        best_score = None

        cluster_ids = sorted(active_clusters.keys())
        for i in range(len(cluster_ids)):
            for j in range(i + 1, len(cluster_ids)):
                left_id = cluster_ids[i]
                right_id = cluster_ids[j]
                allowed, reason, metrics = merge_candidate_allowed(
                    active_clusters[left_id],
                    active_clusters[right_id],
                    active_groupings[left_id],
                    active_groupings[right_id],
                    config,
                )
                merged_symbols = active_groupings[left_id] + active_groupings[right_id]
                final_allowed = allowed
                final_reason = reason
                final_metrics = metrics
                if not allowed:
                    fallback_allowed, fallback_reason, fallback_metrics = (
                        low_mask_fallback_candidate_allowed(
                            active_clusters[left_id],
                            active_clusters[right_id],
                            merged_symbols,
                            config,
                            reason,
                            metrics,
                        )
                    )
                    if fallback_allowed:
                        final_allowed = True
                        final_reason = fallback_reason
                        final_metrics = fallback_metrics
                        final_metrics["accepted"] = True
                    else:
                        final_metrics = fallback_metrics
                        final_metrics["accepted"] = False
                        log_rejected = (
                            config.get("merge", {})
                            .get("low_mask_adjacent_same_group_fallback", {})
                            .get("log_rejected_candidates", True)
                        )
                        if log_rejected and final_metrics.get("merge_method") == "low_mask_adjacent_same_group_fallback":
                            print(
                                f"Fallback merge rejected C{left_id}+C{right_id} "
                                f"reason=\"{fallback_reason}\" checks={final_metrics.get('fallback_checks', {})}"
                            )
                print(
                    f"Merge candidate C{left_id}+C{right_id} "
                    f"accepted={str(final_allowed).lower()} "
                    f"method={final_metrics.get('merge_method', 'normal')} "
                    f"reason=\"{final_reason}\" "
                    f"iou={final_metrics['iou']:.3f} "
                    f"intersection_over_min_area={final_metrics['intersection_over_min_area']:.3f} "
                    f"gap_px={final_metrics['gap_px']:.1f} "
                    f"center_distance_px={final_metrics['center_distance_px']:.1f} "
                    f"distance_diff_m={final_metrics['distance_diff_m']} "
                    f"width_scale={final_metrics['width_scale']:.2f} "
                    f"height_scale={final_metrics['height_scale']:.2f} "
                    f"compactness_pass={str(bool(final_metrics['compactness_pass'])).lower()} "
                    f"strong_spatial_pass={str(bool(final_metrics['strong_spatial_pass'])).lower()} "
                    f"center_distance_only_used={str(bool(final_metrics['center_distance_only_used'])).lower()} "
                    f"fallback_reason={final_metrics.get('fallback_reason', '')}"
                )
                merge_diagnostics.append(
                    MergeDiagnostic(
                        cluster_ids=[left_id, right_id],
                        accepted=final_allowed,
                        merge_method=final_metrics.get("merge_method", "normal"),
                        reason=final_reason,
                        metrics=final_metrics,
                    )
                )
                if not final_allowed:
                    continue

                score = merge_score(final_metrics)
                if best_score is None or score > best_score:
                    best_score = score
                    best_pair = (left_id, right_id)
                    best_reason = final_reason
                    best_metrics = final_metrics

        if best_pair is None:
            break

        left_id, right_id = best_pair
        merged_symbols = active_groupings[left_id] + active_groupings[right_id]
        merged_from = sorted(
            set(active_clusters[left_id].merged_from + active_clusters[right_id].merged_from)
        )
        merged_cluster, _ = build_cluster_record(
            left_id,
            merged_symbols,
            image,
            args,
            config,
            merged_from=merged_from,
        )
        active_clusters[left_id] = merged_cluster
        active_groupings[left_id] = merged_symbols
        del active_clusters[right_id]
        del active_groupings[right_id]

        merge_steps.append(
            MergeStep(
                before_cluster_ids=[left_id, right_id],
                after_cluster_id=left_id,
                symbol_indices=merged_cluster.symbol_indices,
                reason=best_reason,
                metrics=best_metrics,
            )
        )
        print(
            f"Merged clusters C{left_id} + C{right_id} -> C{left_id} "
            f"symbols={merged_cluster.symbol_indices} reason=\"{best_reason}\""
        )

    final_clusters = [active_clusters[cluster_id] for cluster_id in sorted(active_clusters.keys())]
    final_mask_debug: List[Tuple[int, np.ndarray, np.ndarray, Optional[np.ndarray], Tuple[int, int, int, int], str]] = []
    for cluster in final_clusters:
        _, debug = build_cluster_record(
            cluster.cluster_id,
            active_groupings[cluster.cluster_id],
            image,
            args,
            config,
            merged_from=cluster.merged_from,
        )
        final_mask_debug.append(debug)

    return final_clusters, active_groupings, final_mask_debug, merge_steps, merge_diagnostics


def symbol_union_bbox_from_source(
    symbols: Sequence[SymbolRecord],
    source: str,
) -> Tuple[int, int, int, int]:
    if source == "raw":
        return (
            min(symbol.raw_bbox[0] for symbol in symbols),
            min(symbol.raw_bbox[1] for symbol in symbols),
            max(symbol.raw_bbox[2] for symbol in symbols),
            max(symbol.raw_bbox[3] for symbol in symbols),
        )
    return (
        min(symbol.geometry_bbox[0] for symbol in symbols),
        min(symbol.geometry_bbox[1] for symbol in symbols),
        max(symbol.geometry_bbox[2] for symbol in symbols),
        max(symbol.geometry_bbox[3] for symbol in symbols),
    )


def bboxes_overlap(
    first: Tuple[int, int, int, int],
    second: Tuple[int, int, int, int],
) -> bool:
    return (
        min(first[2], second[2]) > max(first[0], second[0]) and
        min(first[3], second[3]) > max(first[1], second[1])
    )


def bbox_intersection_area(
    first: Tuple[int, int, int, int],
    second: Tuple[int, int, int, int],
) -> int:
    inter_w = min(first[2], second[2]) - max(first[0], second[0])
    inter_h = min(first[3], second[3]) - max(first[1], second[1])
    if inter_w <= 0 or inter_h <= 0:
        return 0
    return int(inter_w * inter_h)


def bbox_contains_point(
    bbox: Tuple[int, int, int, int],
    point: Tuple[float, float],
) -> bool:
    return bbox[0] <= point[0] <= bbox[2] and bbox[1] <= point[1] <= bbox[3]


def valid_neighbor_clamp_bbox(
    bbox: Tuple[int, int, int, int],
    own_center: Tuple[float, float],
) -> bool:
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    return width >= 5 and height >= 5 and bbox_contains_point(bbox, own_center)


def apply_neighbor_aware_refined_bbox_clamp(
    clusters: Sequence[ClusterRecord],
    cluster_groupings: Dict[int, List[SymbolRecord]],
    config: dict,
):
    refinement_cfg = config.get("refinement", {})
    if not refinement_cfg.get("neighbor_aware_clamp", False):
        return
    if not refinement_cfg.get("protect_other_cluster_symbols", True):
        return

    margin = int(refinement_cfg.get("neighbor_overlap_margin_px", 10))
    source = refinement_cfg.get("neighbor_protection_bbox_source", "geometry")

    for cluster in clusters:
        original = cluster.refined_bbox
        current = original
        own_center = bbox_center(cluster.union_bbox)
        clamp_reasons: List[str] = []
        clamped_against: Optional[int] = None

        for other in clusters:
            if other.cluster_id == cluster.cluster_id:
                continue

            other_symbols = cluster_groupings.get(other.cluster_id, [])
            if not other_symbols:
                continue

            protected_bbox = symbol_union_bbox_from_source(other_symbols, source)
            intersection_area = bbox_intersection_area(current, protected_bbox)
            if intersection_area <= 0:
                continue

            ax, ay = bbox_center(cluster.union_bbox)
            bx, by = bbox_center(other.union_bbox)
            candidate = list(current)
            axis = "x" if abs(ax - bx) >= abs(ay - by) else "y"
            if axis == "x":
                if ax < bx:
                    candidate[2] = min(candidate[2], protected_bbox[0] - margin)
                else:
                    candidate[0] = max(candidate[0], protected_bbox[2] + margin)
            else:
                if ay < by:
                    candidate[3] = min(candidate[3], protected_bbox[1] - margin)
                else:
                    candidate[1] = max(candidate[1], protected_bbox[3] + margin)

            candidate_bbox = tuple(int(v) for v in candidate)
            print(
                f"Neighbor clamp candidate A=C{cluster.cluster_id} B=C{other.cluster_id} "
                f"a_refined={current} b_protected={protected_bbox} "
                f"intersection_area={intersection_area} axis={axis} "
                f"candidate={candidate_bbox}"
            )
            if candidate_bbox == current:
                print(
                    f"Neighbor clamp rejected A=C{cluster.cluster_id} B=C{other.cluster_id} "
                    "reason=no_effect_after_axis_clamp"
                )
                continue
            if not valid_neighbor_clamp_bbox(candidate_bbox, own_center):
                print(
                    f"Neighbor clamp rejected A=C{cluster.cluster_id} B=C{other.cluster_id} "
                    f"reason=invalid_or_excludes_own_center candidate={candidate_bbox}"
                )
                continue

            current = candidate_bbox
            clamped_against = other.cluster_id
            clamp_reasons.append(
                f"old_bbox={original} protected_{source}_bbox={protected_bbox} "
                f"axis={axis} new_bbox={candidate_bbox} against C{other.cluster_id}"
            )
            print(
                f"Neighbor clamp accepted A=C{cluster.cluster_id} B=C{other.cluster_id} "
                f"axis={axis} new_bbox={candidate_bbox}"
            )

        if current != original:
            cluster.refined_bbox_before_neighbor_clamp = original
            cluster.refined_bbox = current
            cluster.refined_bbox_neighbor_clamped = True
            cluster.neighbor_clamp_reason = "; ".join(clamp_reasons)
            cluster.neighbor_clamp_against_cluster_id = clamped_against
            print(
                f"Neighbor clamp C{cluster.cluster_id}: "
                f"before={original} after={current} "
                f"reason={cluster.neighbor_clamp_reason}"
            )


def detect_edge_clipped_sides(
    bbox: Tuple[int, int, int, int],
    image_width: int,
    image_height: int,
    margin: int,
) -> List[str]:
    sides: List[str] = []
    if bbox[0] <= margin:
        sides.append("left")
    if bbox[1] <= margin:
        sides.append("top")
    if bbox[2] >= image_width - 1 - margin:
        sides.append("right")
    if bbox[3] >= image_height - 1 - margin:
        sides.append("bottom")
    return sides


def apply_edge_visibility_metadata(
    clusters: Sequence[ClusterRecord],
    image_width: int,
    image_height: int,
    config: dict,
) -> List[ClusterRecord]:
    refinement_cfg = config.get("refinement", {})
    margin = int(refinement_cfg.get("edge_clip_margin_px", 3))
    keep_clusters: List[ClusterRecord] = []

    for cluster in clusters:
        union_sides = detect_edge_clipped_sides(
            cluster.union_bbox,
            image_width,
            image_height,
            margin,
        )
        refined_sides = detect_edge_clipped_sides(
            cluster.refined_bbox,
            image_width,
            image_height,
            margin,
        )
        combined_sides = []
        for side in union_sides + refined_sides:
            if side not in combined_sides:
                combined_sides.append(side)

        cluster.edge_clipped = len(combined_sides) > 0
        cluster.edge_clipped_sides = combined_sides
        cluster.bbox_quality = "partial_visible" if cluster.edge_clipped else "normal"

        if cluster.edge_clipped:
            print(
                f"Edge-clipped cluster C{cluster.cluster_id}: "
                f"sides={cluster.edge_clipped_sides} "
                f"union_bbox={cluster.union_bbox} refined_bbox={cluster.refined_bbox}"
            )

        if refinement_cfg.get("drop_edge_clipped", False) and cluster.edge_clipped:
            print(f"Dropping edge-clipped cluster C{cluster.cluster_id} due to config")
            continue

        keep_clusters.append(cluster)

    return keep_clusters


def build_clusters(
    image: np.ndarray,
    symbols: Sequence[SymbolRecord],
    args,
    config: dict,
) -> Tuple[
    List[ClusterRecord],
    Dict[int, List[SymbolRecord]],
    List[Tuple[int, np.ndarray, np.ndarray, Optional[np.ndarray], Tuple[int, int, int, int], str]],
]:
    _image_height, _image_width = image.shape[:2]
    grouped_symbols = cluster_symbols(symbols, config)

    clusters: List[ClusterRecord] = []
    cluster_groupings: Dict[int, List[SymbolRecord]] = {}
    mask_debug: List[Tuple[int, np.ndarray, np.ndarray, Optional[np.ndarray], Tuple[int, int, int, int], str]] = []

    for cluster_id, group in enumerate(grouped_symbols):
        record, debug = build_cluster_record(
            cluster_id,
            group,
            image,
            args,
            config,
        )
        clusters.append(record)
        cluster_groupings[cluster_id] = list(group)
        mask_debug.append(debug)

    return clusters, cluster_groupings, mask_debug


def draw_symbols(
    image: np.ndarray,
    symbols: Sequence[SymbolRecord],
    title: str,
    config: Optional[dict] = None,
    show_geometry_bbox: bool = False,
) -> np.ndarray:
    canvas = image.copy()
    for symbol in symbols:
        x1, y1, x2, y2 = symbol.raw_bbox
        color = (0, 255, 0) if symbol.keep else (0, 0, 255)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        if show_geometry_bbox and symbol.geometry_bbox_source == "square_symbol_bbox":
            gx1, gy1, gx2, gy2 = symbol.geometry_bbox
            cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), (255, 255, 0), 1)
            cv2.putText(
                canvas,
                "geom",
                (gx1, min(image.shape[0] - 6, gy2 + 14)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )
        label = (
            f"{symbol.index}:{symbol.class_name} "
            f"{symbol.confidence:.2f} "
            f"h={symbol.height:.0f} a={symbol.area:.0f}"
        )
        cv2.putText(
            canvas,
            label,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        canvas,
        title,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if config is not None:
        draw_roi_boundary(canvas, config)
    return canvas


def draw_roi_boundary(image: np.ndarray, config: dict):
    roi = config["roi_filter"]
    if not roi["enabled"]:
        return

    image_height, image_width = image.shape[:2]
    color = (0, 255, 255)
    if roi["mode"] == "normalized_polygon" and len(roi.get("polygon", [])) >= 3:
        polygon_points = np.array(
            [normalized_point_to_pixel(point, image_width, image_height) for point in roi["polygon"]],
            dtype=np.int32,
        )
        cv2.polylines(image, [polygon_points], True, color, 2)
    else:
        x1 = int(round(roi["x_min_norm"] * max(0, image_width - 1)))
        x2 = int(round(roi["x_max_norm"] * max(0, image_width - 1)))
        y1 = int(round(roi["y_min_norm"] * max(0, image_height - 1)))
        y2 = int(round(roi["y_max_norm"] * max(0, image_height - 1)))
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

    cv2.putText(
        image,
        "ROI",
        (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


def cluster_color(cluster_id: int) -> Tuple[int, int, int]:
    palette = [
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
        (0, 165, 255),
    ]
    return palette[cluster_id % len(palette)]


def draw_clusters(
    image: np.ndarray,
    symbols: Sequence[SymbolRecord],
    clusters: Sequence[ClusterRecord],
) -> np.ndarray:
    canvas = image.copy()
    symbol_lookup = {symbol.index: symbol for symbol in symbols}
    for cluster in clusters:
        color = cluster_color(cluster.cluster_id)
        for symbol_index in cluster.symbol_indices:
            symbol = symbol_lookup[symbol_index]
            x1, y1, x2, y2 = symbol.raw_bbox
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

        ux1, uy1, ux2, uy2 = cluster.union_bbox
        cv2.rectangle(canvas, (ux1, uy1), (ux2, uy2), color, 2)
        label = (
            f"C{cluster.cluster_id} AMBIGUOUS size={len(cluster.symbol_indices)}"
            if cluster.ambiguous else
            (
                f"C{cluster.cluster_id} {cluster.group_type} merged symbols={len(cluster.symbol_indices)}"
                if cluster.is_merged_cluster else
                f"C{cluster.cluster_id} {cluster.group_type}"
            )
        )
        cv2.putText(
            canvas,
            label,
            (ux1, max(18, uy1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return canvas


def draw_expanded_bboxes(
    image: np.ndarray,
    symbols: Sequence[SymbolRecord],
    clusters: Sequence[ClusterRecord],
) -> np.ndarray:
    canvas = image.copy()
    symbol_lookup = {symbol.index: symbol for symbol in symbols}
    for cluster in clusters:
        color = cluster_color(cluster.cluster_id)
        for symbol_index in cluster.symbol_indices:
            symbol = symbol_lookup[symbol_index]
            rx1, ry1, rx2, ry2 = symbol.raw_bbox
            cv2.rectangle(canvas, (rx1, ry1), (rx2, ry2), color, 1)
            if symbol.geometry_bbox_source == "square_symbol_bbox":
                gx1, gy1, gx2, gy2 = symbol.geometry_bbox
                cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), (255, 255, 0), 1)
                cv2.putText(
                    canvas,
                    "geom",
                    (gx1, min(image.shape[0] - 6, gy2 + 14)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
        ux1, uy1, ux2, uy2 = cluster.union_bbox
        ex1, ey1, ex2, ey2 = cluster.expanded_bbox
        cv2.rectangle(canvas, (ux1, uy1), (ux2, uy2), color, 2)
        cv2.rectangle(canvas, (ex1, ey1), (ex2, ey2), color, 1)
        cv2.putText(
            canvas,
            f"C{cluster.cluster_id} union/thick expanded/thin {cluster.union_bbox_source}",
            (ex1, max(18, ey1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )
    return canvas


def make_mask_debug_board(
    mask_debug: Sequence[Tuple[int, np.ndarray, np.ndarray, Optional[np.ndarray], Tuple[int, int, int, int], str]],
    clusters: Sequence[ClusterRecord],
) -> np.ndarray:
    if not mask_debug:
        board = np.zeros((320, 960, 3), dtype=np.uint8)
        cv2.putText(
            board,
            "No clusters available",
            (60, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return board

    cluster_lookup = {cluster.cluster_id: cluster for cluster in clusters}
    tiles: List[np.ndarray] = []
    for cluster_id, crop, color_mask, debug_white_mask, refined_bbox, refined_source in mask_debug:
        if crop.size == 0:
            continue

        cluster = cluster_lookup[cluster_id]
        crop_h, crop_w = crop.shape[:2]
        color_mask_bgr = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)
        contour_overlay = crop.copy()
        rx1, ry1, rx2, ry2 = refined_bbox
        ex1, ey1, _, _ = cluster.expanded_bbox
        cv2.rectangle(
            contour_overlay,
            (rx1 - ex1, ry1 - ey1),
            (rx2 - ex1, ry2 - ey1),
            (0, 255, 255),
            2,
        )

        if debug_white_mask is not None:
            debug_white_bgr = cv2.cvtColor(debug_white_mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(
                debug_white_bgr,
                "debug only",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            debug_white_bgr = np.zeros_like(color_mask_bgr)

        tile_width = 260
        tile_height = 220
        row = np.hstack([
            cv2.resize(crop, (tile_width, tile_height)),
            cv2.resize(color_mask_bgr, (tile_width, tile_height)),
            cv2.resize(contour_overlay, (tile_width, tile_height)),
            cv2.resize(debug_white_bgr, (tile_width, tile_height)),
        ])
        cv2.putText(
            row,
            (
                f"C{cluster_id} {cluster.group_type} "
                f"color_mask={cluster.color_mask_coverage:.3f} "
                f"source={refined_source}"
            ),
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        tiles.append(row)

    return np.vstack(tiles)


def draw_final_instances(
    image: np.ndarray,
    clusters: Sequence[ClusterRecord],
) -> np.ndarray:
    canvas = image.copy()
    for cluster in clusters:
        color = cluster_color(cluster.cluster_id)
        ex1, ey1, ex2, ey2 = cluster.expanded_bbox
        cv2.rectangle(canvas, (ex1, ey1), (ex2, ey2), color, 1)

        fx1, fy1, fx2, fy2 = cluster.refined_bbox
        thickness = 3 if cluster.refined_bbox_source == "color_contour" else 2
        cv2.rectangle(canvas, (fx1, fy1), (fx2, fy2), color, thickness)
        label = (
            f"C{cluster.cluster_id} AMBIGUOUS size={len(cluster.symbol_indices)}"
            if cluster.ambiguous else
            (
                (
                    f"C{cluster.cluster_id} {cluster.group_type} "
                    f"merged symbols={len(cluster.symbol_indices)} neighbor_clamped"
                    if cluster.is_merged_cluster and cluster.refined_bbox_neighbor_clamped else
                    f"C{cluster.cluster_id} {cluster.group_type} merged symbols={len(cluster.symbol_indices)}"
                    if cluster.is_merged_cluster else
                    (
                        f"C{cluster.cluster_id} {cluster.group_type} "
                        f"source={cluster.refined_bbox_source} neighbor_clamped"
                        if cluster.refined_bbox_neighbor_clamped else
                        f"C{cluster.cluster_id} {cluster.group_type} source={cluster.refined_bbox_source}"
                    )
                )
            )
        )
        if cluster.edge_clipped:
            label = (
                f"{label} {cluster.bbox_quality}:{','.join(cluster.edge_clipped_sides)}"
            )
        cv2.putText(
            canvas,
            label,
            (fx1, max(18, fy1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )
    return canvas


def print_symbol_logs(symbols: Sequence[SymbolRecord]):
    print("Raw symbols:")
    for symbol in symbols:
        print(
            "  "
            f"idx={symbol.index} "
            f"class_id={symbol.class_id} "
            f"class_name={symbol.class_name} "
            f"conf={symbol.confidence:.2f} "
            f"bbox={symbol.raw_bbox} "
            f"geometry_bbox={symbol.geometry_bbox} "
            f"geometry_bbox_source={symbol.geometry_bbox_source} "
            f"bbox_height={symbol.height:.1f} "
            f"bbox_area={symbol.area:.1f} "
            f"geometry_height={symbol.geometry_height:.1f} "
            f"geometry_area={symbol.geometry_area:.1f} "
            f"estimated_distance_m={format_distance(symbol.estimated_distance_m)} "
            f"keep={str(symbol.keep).lower()} "
            f"reason={symbol.keep_reason if symbol.keep else symbol.drop_reason}"
        )


def print_cluster_logs(clusters: Sequence[ClusterRecord]):
    print("Clusters:")
    for cluster in clusters:
        print(
            "  "
            f"cluster_id={cluster.cluster_id} "
            f"symbol_indices={cluster.symbol_indices} "
            f"class_names={cluster.class_names} "
            f"group_type={cluster.group_type} "
            f"ambiguous={str(cluster.ambiguous).lower()} "
            f"ambiguous_reason={cluster.ambiguous_reason} "
            f"union_bbox={cluster.union_bbox} "
            f"union_bbox_source={cluster.union_bbox_source} "
            f"expanded_bbox={cluster.expanded_bbox} "
            f"color_mask_coverage={cluster.color_mask_coverage:.3f} "
            f"debug_white_mask_coverage={cluster.debug_white_mask_coverage} "
            f"refined_bbox={cluster.refined_bbox} "
            f"refined_bbox_before_neighbor_clamp={cluster.refined_bbox_before_neighbor_clamp} "
            f"refined_bbox_neighbor_clamped={str(cluster.refined_bbox_neighbor_clamped).lower()} "
            f"refined_bbox_source={cluster.refined_bbox_source} "
            f"refined_bbox_fallback_reason={cluster.refined_bbox_fallback_reason} "
            f"neighbor_clamp_reason={cluster.neighbor_clamp_reason} "
            f"neighbor_clamp_against_cluster_id={cluster.neighbor_clamp_against_cluster_id} "
            f"edge_clipped={str(cluster.edge_clipped).lower()} "
            f"edge_clipped_sides={cluster.edge_clipped_sides} "
            f"bbox_quality={cluster.bbox_quality} "
            f"merged_from={cluster.merged_from}"
        )


def save_image(path: Path, image: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image)


def filter_final_candidate_clusters(
    clusters: Sequence[ClusterRecord],
    config: dict,
) -> Tuple[List[ClusterRecord], List[dict]]:
    aggregation_cfg = config.get("aggregation", {})
    valid_clusters: List[ClusterRecord] = []
    dropped_clusters: List[dict] = []

    for cluster in clusters:
        if (
            aggregation_cfg.get("drop_ambiguous_clusters", True) and
            cluster.group_type == "AMBIGUOUS"
        ):
            dropped_clusters.append(
                {
                    "cluster_id": cluster.cluster_id,
                    "symbol_indices": cluster.symbol_indices,
                    "class_names": cluster.class_names,
                    "drop_reason": "ambiguous_cluster",
                }
            )
            print(
                f"Dropping final candidate cluster C{cluster.cluster_id} "
                f"reason=ambiguous_cluster symbol_indices={cluster.symbol_indices}"
            )
            continue
        valid_clusters.append(cluster)

    return valid_clusters, dropped_clusters


def summary_payload(
    args,
    config: dict,
    config_path: Optional[str],
    image_path: Path,
    symbols: Sequence[SymbolRecord],
    clusters: Sequence[ClusterRecord],
    final_instances: Sequence[ClusterRecord],
    dropped_clusters: Sequence[dict],
    merge_steps: Sequence[MergeStep],
    merge_diagnostics: Sequence[MergeDiagnostic],
) -> dict:
    return {
        "image": str(image_path),
        "model": str(Path(args.model).expanduser()),
        "team_color": args.team_color,
        "config_path": config_path,
        "config_used": config,
        "range_mode": config["range_filter"]["mode"],
        "roi_filter": config["roi_filter"],
        "cluster_merge": config["cluster_merge"],
        "merge": config.get("merge", {}),
        "aggregation": config.get("aggregation", {}),
        "bbox_expand": {
            "use_square_symbol_bbox": config["bbox"].get("use_square_symbol_bbox"),
            "square_symbol_scale": config["bbox"].get("square_symbol_scale"),
            "max_square_side_ratio_of_image": config["bbox"].get("max_square_side_ratio_of_image"),
            "clustering_center_source": config["bbox"].get("clustering_center_source"),
            "expand_scale": config["bbox"].get("expand_scale"),
            "expand_scale_x": config["bbox"].get("expand_scale_x"),
            "expand_scale_y": config["bbox"].get("expand_scale_y"),
        },
        "contour_selection": config["contour"]["selection"],
        "refinement": config.get("refinement", {}),
        "symbols": [
            {
                "index": symbol.index,
                "class_id": symbol.class_id,
                "class_name": symbol.class_name,
                "confidence": symbol.confidence,
                "bbox": symbol.raw_bbox,
                "raw_bbox": symbol.raw_bbox,
                "geometry_bbox": symbol.geometry_bbox,
                "geometry_bbox_source": symbol.geometry_bbox_source,
                "square_symbol_scale": config["bbox"].get("square_symbol_scale"),
                "bbox_width": symbol.width,
                "bbox_height": symbol.height,
                "bbox_area": symbol.area,
                "geometry_width": symbol.geometry_width,
                "geometry_height": symbol.geometry_height,
                "geometry_area": symbol.geometry_area,
                "estimated_distance_m": symbol.estimated_distance_m,
                "keep": symbol.keep,
                "drop_reason": symbol.drop_reason,
                "roi_inside": symbol.roi_inside,
            }
            for symbol in symbols
        ],
        "clusters": [
            {
                "cluster_id": cluster.cluster_id,
                "symbol_indices": cluster.symbol_indices,
                "class_names": cluster.class_names,
                "group_type": cluster.group_type,
                "ambiguous": cluster.ambiguous,
                "ambiguous_reason": cluster.ambiguous_reason,
                "union_bbox": cluster.union_bbox,
                "union_bbox_source": cluster.union_bbox_source,
                "expanded_bbox": cluster.expanded_bbox,
                "hsv_mask_mode": cluster.hsv_mask_mode,
                "color_mask_coverage": cluster.color_mask_coverage,
                "debug_white_mask_coverage": cluster.debug_white_mask_coverage,
                "refined_bbox": cluster.refined_bbox,
                "refined_bbox_before_neighbor_clamp": cluster.refined_bbox_before_neighbor_clamp,
                "refined_bbox_neighbor_clamped": cluster.refined_bbox_neighbor_clamped,
                "refined_bbox_source": cluster.refined_bbox_source,
                "refined_bbox_fallback_reason": cluster.refined_bbox_fallback_reason,
                "neighbor_clamp_reason": cluster.neighbor_clamp_reason,
                "neighbor_clamp_against_cluster_id": cluster.neighbor_clamp_against_cluster_id,
                "edge_clipped": cluster.edge_clipped,
                "edge_clipped_sides": cluster.edge_clipped_sides,
                "bbox_quality": cluster.bbox_quality,
                "merged_from": cluster.merged_from,
                "is_merged_cluster": cluster.is_merged_cluster,
            }
            for cluster in clusters
        ],
        "final_instances": [
            {
                "cluster_id": cluster.cluster_id,
                "symbol_indices": cluster.symbol_indices,
                "class_names": cluster.class_names,
                "group_type": cluster.group_type,
                "refined_bbox": cluster.refined_bbox,
                "bbox_quality": cluster.bbox_quality,
            }
            for cluster in final_instances
        ],
        "dropped_clusters": list(dropped_clusters),
        "merge_steps": [
            {
                "before_cluster_ids": step.before_cluster_ids,
                "after_cluster_id": step.after_cluster_id,
                "symbol_indices": step.symbol_indices,
                "reason": step.reason,
                "metrics": step.metrics,
            }
            for step in merge_steps
        ],
        "merge_diagnostics": [
            {
                "cluster_ids": diagnostic.cluster_ids,
                "accepted": diagnostic.accepted,
                "merge_method": diagnostic.merge_method,
                "reason": diagnostic.reason,
                "metrics": diagnostic.metrics,
            }
            for diagnostic in merge_diagnostics
        ],
    }


def main() -> int:
    args = parse_args()
    model_path = Path(args.model).expanduser().resolve()
    image_path = Path(args.image).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser()

    if not model_path.exists():
        return fail(f"Model path does not exist: {model_path}")
    if not image_path.exists():
        return fail(f"Image path does not exist: {image_path}")

    config_arg = args.config if args.config else str(DEFAULT_CONFIG_PATH)
    config, config_path = load_yaml_config(config_arg if Path(config_arg).exists() else args.config)
    if args.config and not Path(args.config).expanduser().exists():
        print(
            f"[WARN] Prototype config path does not exist: {args.config}. "
            "Using built-in defaults.",
            file=sys.stderr,
        )
        config_path = str(Path(args.config).expanduser())

    try:
        resolve_hsv_mask_config(config)
    except ValueError as exc:
        return fail(str(exc))

    apply_roi_preset(config)
    apply_config_to_args(args, config)

    image = cv2.imread(str(image_path))
    if image is None:
        return fail(f"Failed to load image: {image_path}")

    try:
        model = load_model(model_path)
        symbols = infer_symbols(model, image, args, config)
    except Exception as exc:
        return fail(str(exc))

    print(f"Using prototype config: {config_path or 'built-in defaults'}")
    print(f"Effective ROI filter: {config['roi_filter']}")
    print(
        "Effective bbox expand: "
        f"scale={config['bbox'].get('expand_scale')} "
        f"scale_x={config['bbox'].get('expand_scale_x')} "
        f"scale_y={config['bbox'].get('expand_scale_y')}"
    )
    print(f"Contour selection mode: {config['contour']['selection']}")
    kept_symbols = apply_range_filter(symbols, config)
    kept_symbols = apply_roi_filter(
        kept_symbols,
        image.shape[1],
        image.shape[0],
        config,
    )
    first_stage_clusters, cluster_groupings, first_stage_mask_debug = build_clusters(
        image,
        kept_symbols,
        args,
        config,
    )
    clusters, cluster_groupings, mask_debug, merge_steps, merge_diagnostics = merge_clusters_second_stage(
        first_stage_clusters,
        cluster_groupings,
        image,
        args,
        config,
    )
    apply_neighbor_aware_refined_bbox_clamp(clusters, cluster_groupings, config)
    clusters = apply_edge_visibility_metadata(
        clusters,
        image.shape[1],
        image.shape[0],
        config,
    )
    final_instances, dropped_clusters = filter_final_candidate_clusters(clusters, config)

    print(
        "Selected HSV mask profile: "
        f"{config['hsv_mask'].get('resolved_profile', 'legacy_flat')} "
        f"(mode={config['hsv_mask']['mode']})"
    )
    print_symbol_logs(symbols)
    print("First-stage clusters:")
    print_cluster_logs(first_stage_clusters)
    print("Final clusters:")
    print_cluster_logs(clusters)
    if dropped_clusters:
        print(f"Dropped final candidate clusters: {dropped_clusters}")

    output_dir.mkdir(parents=True, exist_ok=True)
    save_image(output_dir / "01_yolo_symbols.jpg", draw_symbols(image, symbols, "YOLO symbols"))
    save_image(
        output_dir / "02_range_filtered.jpg",
        draw_symbols(
            image,
            kept_symbols,
            "Range + ROI filtered symbols",
            config,
            show_geometry_bbox=True,
        ),
    )
    save_image(output_dir / "03_clusters.jpg", draw_clusters(image, kept_symbols, first_stage_clusters))
    save_image(output_dir / "03b_merged_clusters.jpg", draw_clusters(image, kept_symbols, clusters))
    save_image(output_dir / "04_expanded_bbox.jpg", draw_expanded_bboxes(image, kept_symbols, clusters))
    save_image(output_dir / "05_kfs_mask.jpg", make_mask_debug_board(mask_debug, clusters))
    save_image(output_dir / "06_final_instances.jpg", draw_final_instances(image, final_instances))

    summary = summary_payload(
        args,
        config,
        config_path,
        image_path,
        symbols,
        clusters,
        final_instances,
        dropped_clusters,
        merge_steps,
        merge_diagnostics,
    )
    with (output_dir / "kfs_instance_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    print(f"Saved debug outputs to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
