#!/usr/bin/env python3

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
GROUPS = ("R1", "REAL", "FAKE")


@dataclass
class BoxRecord:
    class_id: int
    group: str
    xyxy: Tuple[float, float, float, float]


@dataclass
class PredictionRecord:
    image_id: str
    class_id: int
    group: str
    confidence: float
    xyxy: Tuple[float, float, float, float]


@dataclass
class SweepRow:
    group: str
    threshold: float
    beta: float
    tp: int
    fp: int
    fn: int
    precision: float
    recall: float
    f_beta: float


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline DecisionEngine threshold tuning from YOLO validation data."
    )
    parser.add_argument("--model", required=True, help="Path to YOLO model.")
    parser.add_argument("--dataset", required=True, help="Path to validation dataset root.")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size.")
    parser.add_argument("--conf-min", type=float, default=0.05, help="Minimum prediction confidence to keep before sweeping.")
    parser.add_argument("--iou", type=float, default=0.45, help="IoU threshold for greedy matching.")
    parser.add_argument("--threshold-step", type=float, default=0.01, help="Threshold sweep step size.")
    parser.add_argument("--output-csv", default=None, help="Optional CSV path for full sweep results.")
    parser.add_argument("--save-json", default=None, help="Optional JSON path for summary output.")
    parser.add_argument("--beta-r1", type=float, default=1.0, help="F-beta value for R1.")
    parser.add_argument("--beta-real", type=float, default=0.75, help="F-beta value for REAL.")
    parser.add_argument("--beta-fake", type=float, default=2.0, help="F-beta value for FAKE.")
    parser.add_argument(
        "--print-yaml",
        dest="print_yaml",
        action="store_true",
        help="Print YAML-ready threshold recommendations.",
    )
    parser.add_argument(
        "--no-print-yaml",
        dest="print_yaml",
        action="store_false",
        help="Disable YAML snippet output.",
    )
    parser.set_defaults(print_yaml=True)
    return parser.parse_args()


def fail(message: str) -> int:
    print(f"[ERROR] {message}", file=sys.stderr)
    return 1


def group_for_class_id(class_id: int) -> Optional[str]:
    if class_id == 0:
        return "R1"
    if 1 <= class_id <= 15:
        return "REAL"
    if 16 <= class_id <= 30:
        return "FAKE"
    return None


def xywhn_to_xyxy(
    cx: float,
    cy: float,
    width: float,
    height: float,
    image_width: int,
    image_height: int,
) -> Tuple[float, float, float, float]:
    box_width = width * image_width
    box_height = height * image_height
    center_x = cx * image_width
    center_y = cy * image_height

    x1 = max(0.0, center_x - box_width / 2.0)
    y1 = max(0.0, center_y - box_height / 2.0)
    x2 = min(float(image_width), center_x + box_width / 2.0)
    y2 = min(float(image_height), center_y + box_height / 2.0)
    return (x1, y1, x2, y2)


def compute_iou(
    box_a: Tuple[float, float, float, float],
    box_b: Tuple[float, float, float, float],
) -> float:
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])

    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    intersection = inter_w * inter_h
    if intersection <= 0.0:
        return 0.0

    area_a = max(0.0, box_a[2] - box_a[0]) * max(0.0, box_a[3] - box_a[1])
    area_b = max(0.0, box_b[2] - box_b[0]) * max(0.0, box_b[3] - box_b[1])
    union = area_a + area_b - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union


def safe_divide(numerator: float, denominator: float) -> float:
    if denominator == 0.0:
        return 0.0
    return numerator / denominator


def compute_f_beta(precision: float, recall: float, beta: float) -> float:
    beta_sq = beta * beta
    denominator = beta_sq * precision + recall
    if denominator <= 0.0:
        return 0.0
    return (1.0 + beta_sq) * precision * recall / denominator


def list_dataset_images(dataset_root: Path) -> List[Path]:
    images_dir = dataset_root / "images"
    if not images_dir.is_dir():
        raise FileNotFoundError(f"Missing images directory: {images_dir}")

    images = sorted(
        path for path in images_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )
    if not images:
        raise FileNotFoundError(f"No images found in {images_dir}")
    return images


def load_ground_truth_for_image(image_path: Path) -> List[BoxRecord]:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required for reading image dimensions. Install opencv-python."
        ) from exc

    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {image_path}")
    image_height, image_width = image.shape[:2]

    label_path = image_path.parent.parent / "labels" / f"{image_path.stem}.txt"
    if not label_path.exists():
        return []

    records: List[BoxRecord] = []
    with label_path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                continue

            parts = line.split()
            if len(parts) != 5:
                print(
                    f"[WARN] Skipping malformed label line {line_number} in {label_path}",
                    file=sys.stderr,
                )
                continue

            class_id = int(float(parts[0]))
            group = group_for_class_id(class_id)
            if group is None:
                continue

            cx, cy, width, height = (float(value) for value in parts[1:])
            xyxy = xywhn_to_xyxy(cx, cy, width, height, image_width, image_height)
            records.append(BoxRecord(class_id=class_id, group=group, xyxy=xyxy))

    return records


def load_all_ground_truth(dataset_root: Path) -> Tuple[List[Path], Dict[str, List[BoxRecord]]]:
    image_paths = list_dataset_images(dataset_root)
    ground_truth: Dict[str, List[BoxRecord]] = {}
    for image_path in image_paths:
        ground_truth[str(image_path)] = load_ground_truth_for_image(image_path)
    return image_paths, ground_truth


def load_model(model_path: Path):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "Ultralytics is required for offline tuning. Install it with "
            "`pip install ultralytics` in your analysis environment."
        ) from exc

    try:
        return YOLO(str(model_path))
    except Exception as exc:
        raise RuntimeError(f"Failed to load model {model_path}: {exc}") from exc


def run_inference(
    model,
    image_paths: Sequence[Path],
    imgsz: int,
    conf_min: float,
) -> Dict[str, List[PredictionRecord]]:
    predictions: Dict[str, List[PredictionRecord]] = {}

    try:
        results = model.predict(
            source=[str(path) for path in image_paths],
            imgsz=imgsz,
            conf=conf_min,
            verbose=False,
        )
    except Exception as exc:
        raise RuntimeError(f"Model inference failed: {exc}") from exc

    for image_path, result in zip(image_paths, results):
        image_id = str(image_path)
        image_predictions: List[PredictionRecord] = []
        boxes = getattr(result, "boxes", None)

        if boxes is not None:
            xyxy_values = boxes.xyxy.cpu().tolist() if hasattr(boxes, "xyxy") else []
            conf_values = boxes.conf.cpu().tolist() if hasattr(boxes, "conf") else []
            class_values = boxes.cls.cpu().tolist() if hasattr(boxes, "cls") else []

            for xyxy, conf, cls_value in zip(xyxy_values, conf_values, class_values):
                class_id = int(cls_value)
                group = group_for_class_id(class_id)
                if group is None:
                    continue

                image_predictions.append(
                    PredictionRecord(
                        image_id=image_id,
                        class_id=class_id,
                        group=group,
                        confidence=float(conf),
                        xyxy=tuple(float(value) for value in xyxy),
                    )
                )

        predictions[image_id] = sorted(
            image_predictions,
            key=lambda item: item.confidence,
            reverse=True,
        )

    return predictions


def filter_predictions_by_group_and_threshold(
    predictions: Dict[str, List[PredictionRecord]],
    group: str,
    threshold: float,
) -> Dict[str, List[PredictionRecord]]:
    filtered: Dict[str, List[PredictionRecord]] = {}
    for image_id, items in predictions.items():
        filtered[image_id] = [
            item for item in items
            if item.group == group and item.confidence >= threshold
        ]
    return filtered


def evaluate_group(
    group: str,
    threshold: float,
    iou_threshold: float,
    predictions: Dict[str, List[PredictionRecord]],
    ground_truth: Dict[str, List[BoxRecord]],
    beta: float,
) -> SweepRow:
    tp = 0
    fp = 0
    fn = 0

    filtered_predictions = filter_predictions_by_group_and_threshold(
        predictions,
        group,
        threshold,
    )

    for image_id, image_predictions in filtered_predictions.items():
        group_ground_truth = [
            item for item in ground_truth.get(image_id, [])
            if item.group == group
        ]
        matched_gt_indices = set()

        for prediction in image_predictions:
            best_match_index = None
            best_iou = 0.0

            for gt_index, gt_box in enumerate(group_ground_truth):
                if gt_index in matched_gt_indices:
                    continue

                iou = compute_iou(prediction.xyxy, gt_box.xyxy)
                if iou >= iou_threshold and iou > best_iou:
                    best_iou = iou
                    best_match_index = gt_index

            if best_match_index is not None:
                matched_gt_indices.add(best_match_index)
                tp += 1
            else:
                fp += 1

        fn += len(group_ground_truth) - len(matched_gt_indices)

    precision = safe_divide(tp, tp + fp)
    recall = safe_divide(tp, tp + fn)
    f_beta = compute_f_beta(precision, recall, beta)

    return SweepRow(
        group=group,
        threshold=threshold,
        beta=beta,
        tp=tp,
        fp=fp,
        fn=fn,
        precision=precision,
        recall=recall,
        f_beta=f_beta,
    )


def build_thresholds(step: float) -> List[float]:
    if step <= 0.0 or step > 1.0:
        raise ValueError("--threshold-step must be > 0 and <= 1")

    thresholds: List[float] = []
    current = 0.0
    while current < 1.0:
        thresholds.append(round(current, 6))
        current += step
    thresholds.append(1.0)
    return thresholds


def choose_best_row(rows: Sequence[SweepRow], group: str) -> SweepRow:
    if not rows:
        raise ValueError(f"No sweep rows available for group {group}")

    best = rows[0]
    for row in rows[1:]:
        if row.f_beta > best.f_beta + 1e-12:
            best = row
            continue
        if math.isclose(row.f_beta, best.f_beta, rel_tol=1e-12, abs_tol=1e-12):
            if group in ("R1", "REAL") and row.threshold > best.threshold:
                best = row
            elif group == "FAKE" and row.threshold < best.threshold:
                best = row
    return best


def count_boxes_by_group(ground_truth: Dict[str, List[BoxRecord]]) -> Dict[str, int]:
    counts = {group: 0 for group in GROUPS}
    for boxes in ground_truth.values():
        for box in boxes:
            counts[box.group] += 1
    return counts


def count_predictions_by_group(predictions: Dict[str, List[PredictionRecord]]) -> Dict[str, int]:
    counts = {group: 0 for group in GROUPS}
    for boxes in predictions.values():
        for box in boxes:
            counts[box.group] += 1
    return counts


def write_csv(rows: Sequence[SweepRow], output_path: Path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "group",
            "threshold",
            "beta",
            "tp",
            "fp",
            "fn",
            "precision",
            "recall",
            "f_beta",
        ])
        for row in rows:
            writer.writerow([
                row.group,
                f"{row.threshold:.4f}",
                f"{row.beta:.4f}",
                row.tp,
                row.fp,
                row.fn,
                f"{row.precision:.6f}",
                f"{row.recall:.6f}",
                f"{row.f_beta:.6f}",
            ])


def write_json(summary: dict, output_path: Path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)


def print_summary(
    dataset_path: Path,
    model_path: Path,
    image_count: int,
    gt_counts: Dict[str, int],
    pred_counts: Dict[str, int],
    best_rows: Dict[str, SweepRow],
):
    print("")
    print("Decision Threshold Tuning Summary")
    print(f"dataset: {dataset_path}")
    print(f"model: {model_path}")
    print(f"images: {image_count}")
    print("ground_truth_boxes:")
    for group in GROUPS:
        print(f"  {group}: {gt_counts[group]}")
    print("predictions:")
    for group in GROUPS:
        print(f"  {group}: {pred_counts[group]}")
    print("best_thresholds:")
    for group in GROUPS:
        row = best_rows[group]
        print(
            f"  {group}: threshold={row.threshold:.2f} "
            f"precision={row.precision:.4f} "
            f"recall={row.recall:.4f} "
            f"f_beta={row.f_beta:.4f}"
        )


def print_yaml_snippet(best_rows: Dict[str, SweepRow]):
    print("")
    print("# DecisionEngine thresholds tuned from validation data")
    print(f"r1_conf_threshold: {best_rows['R1'].threshold:.2f}")
    print(f"real_conf_threshold: {best_rows['REAL'].threshold:.2f}")
    print(f"fake_conf_threshold: {best_rows['FAKE'].threshold:.2f}")
    print("")
    print("# Keep collect_min_confidence risk-based unless separately tuned")
    print("collect_min_confidence: 0.60")
    print("")


def main() -> int:
    args = parse_args()

    dataset_path = Path(args.dataset).expanduser().resolve()
    model_path = Path(args.model).expanduser().resolve()

    if not model_path.exists():
        return fail(f"Model path does not exist: {model_path}")
    if not dataset_path.exists():
        return fail(f"Dataset path does not exist: {dataset_path}")

    try:
        image_paths, ground_truth = load_all_ground_truth(dataset_path)
        model = load_model(model_path)
        predictions = run_inference(
            model=model,
            image_paths=image_paths,
            imgsz=args.imgsz,
            conf_min=args.conf_min,
        )
        thresholds = build_thresholds(args.threshold_step)
    except Exception as exc:
        return fail(str(exc))

    betas = {
        "R1": args.beta_r1,
        "REAL": args.beta_real,
        "FAKE": args.beta_fake,
    }

    sweep_rows: List[SweepRow] = []
    best_rows: Dict[str, SweepRow] = {}

    for group in GROUPS:
        group_rows = [
            evaluate_group(
                group=group,
                threshold=threshold,
                iou_threshold=args.iou,
                predictions=predictions,
                ground_truth=ground_truth,
                beta=betas[group],
            )
            for threshold in thresholds
        ]
        sweep_rows.extend(group_rows)
        best_rows[group] = choose_best_row(group_rows, group)

    gt_counts = count_boxes_by_group(ground_truth)
    pred_counts = count_predictions_by_group(predictions)

    print_summary(
        dataset_path=dataset_path,
        model_path=model_path,
        image_count=len(image_paths),
        gt_counts=gt_counts,
        pred_counts=pred_counts,
        best_rows=best_rows,
    )

    if args.print_yaml:
        print_yaml_snippet(best_rows)

    if args.output_csv:
        write_csv(sweep_rows, Path(args.output_csv).expanduser())
        print(f"Saved CSV sweep results to {args.output_csv}")

    if args.save_json:
        summary = {
            "dataset": str(dataset_path),
            "model": str(model_path),
            "image_count": len(image_paths),
            "ground_truth_boxes": gt_counts,
            "predictions": pred_counts,
            "betas": betas,
            "recommended_thresholds": {
                "r1_conf_threshold": round(best_rows["R1"].threshold, 4),
                "real_conf_threshold": round(best_rows["REAL"].threshold, 4),
                "fake_conf_threshold": round(best_rows["FAKE"].threshold, 4),
                "collect_min_confidence": 0.60,
            },
            "best_rows": {
                group: asdict(row)
                for group, row in best_rows.items()
            },
        }
        write_json(summary, Path(args.save_json).expanduser())
        print(f"Saved JSON summary to {args.save_json}")

    print("This tool only recommends thresholds. Update yolo_detection.yaml manually.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
