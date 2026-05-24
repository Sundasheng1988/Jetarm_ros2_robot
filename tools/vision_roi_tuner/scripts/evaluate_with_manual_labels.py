#!/usr/bin/env python3
"""Evaluate ROI color accuracy using human-provided manual labels."""

import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2

from roi_color_detection_utils import (
    estimate_color_with_otsu_chroma,
    load_lab_ranges,
)

ROOT = Path(__file__).resolve().parent.parent
IMAGES_DIR = ROOT / "inputs" / "images"
CONFIG_PATH = ROOT / "configs" / "lab_config.yaml"
LABELS_PATH = ROOT / "inputs" / "labels" / "manual_labels.json"
OUTPUT_DIR = ROOT / "outputs" / "reports_v2"

# Best strategy from experiment loop (Iter 4)
BEST_PARAMS = {
    "use_otsu": True,
    "use_chroma": True,
    "chroma_threshold": 5.0,
    "chromatic_min_confidence": 0.10,
    "lab_range_overrides": {
        "blue": {"B": (59, 143)},
        "red": {"A": (124, 183), "B": (125, 189)},
    },
}


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    lab_ranges, _ = load_lab_ranges(str(CONFIG_PATH))

    with open(LABELS_PATH) as f:
        labels = json.load(f)

    if not labels:
        print("No manual labels found.")
        sys.exit(1)

    print(f"Loaded {len(labels)} manual labels")

    results = []
    correct = 0
    wrong_high_conf = 0
    per_color = defaultdict(lambda: {"total": 0, "correct": 0})
    per_class = defaultdict(lambda: {"total": 0, "correct": 0})
    per_mask = defaultdict(lambda: {"total": 0, "correct": 0})
    confusion = defaultdict(Counter)
    bad_mask_images = []

    for entry in labels:
        fname = entry["image"]
        true_color = entry["true_color"]
        true_class = entry["true_class"]
        mask_quality = entry.get("mask_quality", "good")
        note = entry.get("note", "")

        img_path = IMAGES_DIR / fname
        if not img_path.exists():
            print(f"  SKIP {fname}: image not found")
            continue

        bgr = cv2.imread(str(img_path))
        pred_color, confidence, hits, contour = estimate_color_with_otsu_chroma(
            bgr, lab_ranges, **BEST_PARAMS)

        ok = (pred_color == true_color)
        if ok:
            correct += 1

        per_color[true_color]["total"] += 1
        per_class[true_class]["total"] += 1
        per_mask[mask_quality]["total"] += 1
        if ok:
            per_color[true_color]["correct"] += 1
            per_class[true_class]["correct"] += 1
            per_mask[mask_quality]["correct"] += 1
        elif confidence >= 0.75:
            wrong_high_conf += 1

        confusion[true_color][pred_color] += 1

        if mask_quality == "bad":
            bad_mask_images.append({
                "image": fname,
                "true_color": true_color,
                "predicted": pred_color,
                "confidence": confidence,
                "note": note,
            })

        results.append({
            "image": fname,
            "true_color": true_color,
            "true_class": true_class,
            "predicted": pred_color,
            "confidence": confidence,
            "correct": ok,
            "mask_quality": mask_quality,
            "note": note,
        })

        status = "OK" if ok else "WRONG"
        print(f"  {fname:28s}  GT={true_color:5s}/{true_class:5s}  "
              f"PRED={pred_color:8s}  conf={confidence:.3f}  "
              f"mask={mask_quality:6s}  {status}")

    total = len(results)
    acc = round(correct / total, 4) if total else 0.0

    per_color_acc = {
        c: round(v["correct"] / v["total"], 4) if v["total"] else 0.0
        for c, v in sorted(per_color.items())
    }
    per_class_acc = {
        c: round(v["correct"] / v["total"], 4) if v["total"] else 0.0
        for c, v in sorted(per_class.items())
    }
    per_mask_acc = {
        m: round(v["correct"] / v["total"], 4) if v["total"] else 0.0
        for m, v in sorted(per_mask.items())
    }

    summary = {
        "total_images": total,
        "correct": correct,
        "accuracy": acc,
        "wrong_high_confidence": wrong_high_conf,
        "per_color_accuracy": per_color_acc,
        "per_class_accuracy": per_class_acc,
        "per_mask_accuracy": per_mask_acc,
        "confusion_matrix": {k: dict(v) for k, v in sorted(confusion.items())},
        "bad_mask_images": bad_mask_images,
        "per_image": results,
    }

    json_path = OUTPUT_DIR / "human_label_eval.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"\n✅ {json_path}")

    md = []
    md.append("# Human-Labeled ROI Color Evaluation\n")
    md.append(f"**Images**: {total} (human-labeled)\n")
    md.append(f"**Accuracy**: {correct}/{total} = {acc:.2%}\n")
    md.append(f"**Wrong high-confidence**: {wrong_high_conf}\n")

    md.append("## Per-Color Accuracy\n")
    md.append("| Color | Correct | Total | Accuracy |")
    md.append("|-------|---------|-------|----------|")
    for c, a in sorted(per_color_acc.items()):
        md.append(f"| {c} | {per_color[c]['correct']} | {per_color[c]['total']} | {a:.2%} |")
    md.append("")

    md.append("## Per-Class Accuracy\n")
    md.append("| Class | Correct | Total | Accuracy |")
    md.append("|-------|---------|-------|----------|")
    for c, a in sorted(per_class_acc.items()):
        md.append(f"| {c} | {per_class[c]['correct']} | {per_class[c]['total']} | {a:.2%} |")
    md.append("")

    md.append("## Per-Mask-Quality Accuracy\n")
    md.append("| Mask Quality | Correct | Total | Accuracy |")
    md.append("|-------------|---------|-------|----------|")
    for m, a in sorted(per_mask_acc.items()):
        md.append(f"| {m} | {per_mask[m]['correct']} | {per_mask[m]['total']} | {a:.2%} |")
    md.append("")

    md.append("## Confusion Matrix (true \\ predicted)\n")
    all_keys = sorted(set().union(*[v.keys() for v in confusion.values()], confusion.keys()))
    header = "| True | " + " | ".join(all_keys) + " |"
    sep = "|" + "|".join(["---"] * (len(all_keys) + 1)) + "|"
    md.append(header)
    md.append(sep)
    for exp in sorted(confusion.keys()):
        row = f"| {exp} " + " ".join(f"| {confusion[exp].get(pred, 0)} " for pred in all_keys) + "|"
        md.append(row)
    md.append("")

    md.append("## High-Confidence Wrong Cases\n")
    high_conf_wrong = [r for r in results if not r["correct"] and r["confidence"] >= 0.75]
    if high_conf_wrong:
        md.append(f"**{len(high_conf_wrong)}/{total} images**\n")
        md.append("| Image | True | Predicted | Confidence | Mask |")
        md.append("|-------|------|-----------|------------|------|")
        for r in high_conf_wrong:
            md.append(f"| {r['image']} | {r['true_color']} | {r['predicted']} | {r['confidence']:.3f} | {r['mask_quality']} |")
    else:
        md.append("None\n")
    md.append("")

    md.append("## Images with Bad Mask Quality\n")
    if bad_mask_images:
        md.append(f"**{len(bad_mask_images)} images**\n")
        md.append("| Image | True Color | Predicted | Confidence | Note |")
        md.append("|-------|-----------|-----------|------------|------|")
        for b in bad_mask_images:
            md.append(f"| {b['image']} | {b['true_color']} | {b['predicted']} | {b['confidence']:.3f} | {b['note']} |")
    else:
        md.append("None\n")
    md.append("")

    md.append("## Per-Image Detail\n")
    md.append("| Image | True | Predicted | Confidence | Mask | Correct |")
    md.append("|-------|------|-----------|------------|------|---------|")
    for r in results:
        s = "✅" if r["correct"] else "❌"
        md.append(f"| {r['image']} | {r['true_color']}/{r['true_class']} | {r['predicted']} | {r['confidence']:.3f} | {r['mask_quality']} | {s} |")

    md_path = OUTPUT_DIR / "human_label_eval.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md))
    print(f"✅ {md_path}")

    print(f"\n{'='*60}")
    print(f"  Accuracy: {correct}/{total} = {acc:.2%}")
    print(f"  Per-color: {per_color_acc}")
    print(f"  Per-mask: {per_mask_acc}")
    print(f"  Bad mask images: {len(bad_mask_images)}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
