#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline ROI color evaluation tool.

Evaluates the current LAB color range configuration against labeled images.
Generates JSON metrics, Markdown report, and per-image debug overlays.
"""

import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2

from roi_color_detection_utils import (
    build_debug_overlay,
    estimate_color,
    load_lab_ranges,
    parse_filename_label,
)

ROOT = Path(__file__).resolve().parent.parent
IMAGES_DIR = ROOT / "inputs" / "images"
CONFIG_PATH = ROOT / "configs" / "lab_config.yaml"
OUTPUT_DIR = ROOT / "outputs"
REPORTS_DIR = OUTPUT_DIR / "reports"
DEBUG_DIR = OUTPUT_DIR / "debug_images"


def main():
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    lab_ranges, color_keys = load_lab_ranges(str(CONFIG_PATH))
    print(f"Loaded {len(color_keys)} color keys: {color_keys}")

    image_files = sorted(IMAGES_DIR.glob("*.jpg"))
    print(f"Found {len(image_files)} images")

    results = []
    for img_path in image_files:
        try:
            expected_color, expected_class = parse_filename_label(img_path.name)
        except ValueError as e:
            print(f"  SKIP {img_path.name}: {e}")
            continue

        bgr = cv2.imread(str(img_path))
        if bgr is None:
            print(f"  SKIP {img_path.name}: cannot load image")
            continue

        pred_color, confidence, hits, contour = estimate_color(bgr, lab_ranges)

        correct = (pred_color == expected_color)
        results.append({
            "filename": img_path.name,
            "expected_color": expected_color,
            "expected_class": expected_class,
            "pred_color": pred_color,
            "confidence": confidence,
            "correct": correct,
            "all_hits": hits,
        })

        status = "OK" if correct else "WRONG"
        print(f"  {img_path.name:28s}  GT={expected_color:6s}  PRED={pred_color:8s}  "
              f"conf={confidence:.3f}  {status}")

        debug_img = build_debug_overlay(
            bgr, contour, expected_color, expected_class,
            pred_color, confidence, hits,
        )
        out_name = img_path.stem + "_debug.png"
        cv2.imwrite(str(DEBUG_DIR / out_name), debug_img)

    if not results:
        print("No results generated.")
        sys.exit(1)

    total = len(results)
    correct_count = sum(1 for r in results if r["correct"])
    wrong_results = [r for r in results if not r["correct"]]
    correct_results = [r for r in results if r["correct"]]

    conf_correct = [r["confidence"] for r in correct_results]
    conf_wrong = [r["confidence"] for r in wrong_results]
    correct_mean_conf = round(sum(conf_correct) / len(conf_correct), 4) if conf_correct else 0.0
    wrong_mean_conf = round(sum(conf_wrong) / len(conf_wrong), 4) if conf_wrong else 0.0

    per_color = defaultdict(lambda: {"total": 0, "correct": 0})
    for r in results:
        c = r["expected_color"]
        per_color[c]["total"] += 1
        if r["correct"]:
            per_color[c]["correct"] += 1
    per_color_accuracy = {
        c: round(v["correct"] / v["total"], 4) if v["total"] else 0.0
        for c, v in sorted(per_color.items())
    }

    confusion = defaultdict(Counter)
    for r in results:
        confusion[r["expected_color"]][r["pred_color"]] += 1

    all_confidences = [r["confidence"] for r in results]
    conf_min = min(all_confidences)
    conf_max = max(all_confidences)
    bins = [0.0, 0.2, 0.4, 0.6, 0.8, 1.01]
    hist_labels = ["0.0-0.2", "0.2-0.4", "0.4-0.6", "0.6-0.8", "0.8-1.0"]
    histogram = {}
    for i in range(len(bins) - 1):
        lo, hi = bins[i], bins[i + 1]
        histogram[hist_labels[i]] = sum(1 for c in all_confidences if lo <= c < hi)

    summary = {
        "total_images": total,
        "correct": correct_count,
        "wrong": total - correct_count,
        "accuracy": round(correct_count / total, 4),
        "per_color_accuracy": per_color_accuracy,
        "correct_mean_conf": correct_mean_conf,
        "wrong_mean_conf": wrong_mean_conf,
        "confidence_min": conf_min,
        "confidence_max": conf_max,
        "confidence_histogram": histogram,
    }

    result_json = {
        "summary": summary,
        "confusion_matrix": {k: dict(v) for k, v in sorted(confusion.items())},
        "per_image": results,
    }

    json_path = REPORTS_DIR / "roi_eval_result.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result_json, f, indent=2, ensure_ascii=False)
    print(f"\n✅ {json_path}")

    md_path = REPORTS_DIR / "roi_eval_report.md"
    write_report(md_path, summary, confusion, results, histogram)
    print(f"✅ {md_path}")
    print(f"✅ {DEBUG_DIR}/  ({total} debug images)")

    print(f"\n{'='*50}")
    print(f"  Accuracy:  {correct_count}/{total} = {summary['accuracy']:.2%}")
    print(f"  Correct mean confidence: {correct_mean_conf:.4f}")
    print(f"  Wrong   mean confidence: {wrong_mean_conf:.4f}")
    print(f"  Per-color: {per_color_accuracy}")
    print(f"  Confidence histogram: {histogram}")
    print(f"{'='*50}")


def write_report(md_path, summary, confusion, results, histogram):
    lines = []
    lines.append("# ROI Color Evaluation Report\n")
    lines.append(f"**Images**: {summary['total_images']}  |  ")
    lines.append(f"**Accuracy**: {summary['correct']}/{summary['total_images']} = {summary['accuracy']:.2%}  |  ")
    lines.append(f"**Correct mean conf**: {summary['correct_mean_conf']:.4f}  |  ")
    lines.append(f"**Wrong mean conf**: {summary['wrong_mean_conf']:.4f}\n")

    lines.append("## Per-Color Accuracy\n")
    lines.append("| Color | Correct | Total | Accuracy |")
    lines.append("|-------|---------|-------|----------|")
    for c, acc in sorted(summary["per_color_accuracy"].items()):
        total_c = sum(1 for r in results if r["expected_color"] == c)
        correct_c = sum(1 for r in results if r["expected_color"] == c and r["correct"])
        lines.append(f"| {c} | {correct_c} | {total_c} | {acc:.2%} |")
    lines.append("")

    lines.append("## Confusion Matrix (expected \\ predicted)\n")
    all_keys = sorted(set().union(*[v.keys() for v in confusion.values()], confusion.keys()))
    header = "| Expected | " + " | ".join(all_keys) + " |"
    sep = "|" + "|".join(["---"] * (len(all_keys) + 1)) + "|"
    lines.append(header)
    lines.append(sep)
    for exp in sorted(confusion.keys()):
        row = f"| {exp} "
        for pred in all_keys:
            row += f"| {confusion[exp].get(pred, 0)} "
        row += "|"
        lines.append(row)
    lines.append("")

    lines.append("## Confidence Distribution\n")
    lines.append("| Bin | Count |")
    lines.append("|-----|-------|")
    for bin_label, count in histogram.items():
        bar = "█" * max(1, int(count / max(1, max(histogram.values())) * 20))
        lines.append(f"| {bin_label} | {count} {bar} |")
    lines.append("")

    wrong = [r for r in results if not r["correct"]]
    if wrong:
        lines.append("## Failures\n")
        lines.append(f"**{len(wrong)}/{summary['total_images']} images misclassified**\n")
        lines.append("| Image | GT | Predicted | Confidence | Top Hits |")
        lines.append("|-------|----|-----------|------------|----------|")
        for r in wrong:
            top_hits = sorted(r["all_hits"].items(), key=lambda x: -x[1])[:3]
            hits_str = ", ".join(f"{k}:{v}" for k, v in top_hits)
            lines.append(
                f"| {r['filename']} | {r['expected_color']} | "
                f"{r['pred_color']} | {r['confidence']:.3f} | {hits_str} |"
            )
        lines.append("")

    lines.append("## Tuning Recommendations\n")
    lines.append("Based on confusion matrix and per-color accuracy:\n")

    for color, acc in sorted(summary["per_color_accuracy"].items()):
        if acc < 0.80:
            misclass = [(r["pred_color"], r["filename"]) for r in wrong if r["expected_color"] == color]
            top_wrong = Counter(c for c, _ in misclass).most_common(2)
            lines.append(f"- **{color}** (accuracy={acc:.2%}): most confused with "
                         f"{', '.join(f'{c}({n}x)' for c, n in top_wrong)}. "
                         f"Consider narrowing {color} LAB range or adjusting thresholds.")

    lines.append(
        f"- Confidence calibration: correct_mean={summary['correct_mean_conf']:.4f}, "
        f"wrong_mean={summary['wrong_mean_conf']:.4f}. "
        f"{'Good separation' if summary['correct_mean_conf'] > summary['wrong_mean_conf'] + 0.1 else 'Poor separation — confidence not predictive of correctness'}."
    )

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("".join(lines))


if __name__ == "__main__":
    main()
