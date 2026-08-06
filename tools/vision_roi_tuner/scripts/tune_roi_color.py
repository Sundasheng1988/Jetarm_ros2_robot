#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auto-tuning loop for ROI color estimation parameters.
Grid-searches over erode, luminance thresholds, confidence, and chromatic priority.
Outputs tuning_results.json, tuning_report.md, and best_config.json.
"""

import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2

from roi_color_detection_utils import (
    estimate_color_with_thresholds,
    load_lab_ranges,
    parse_filename_label,
)

ROOT = Path(__file__).resolve().parent.parent
IMAGES_DIR = ROOT / "inputs" / "images"
CONFIG_PATH = ROOT / "configs" / "lab_config.yaml"
REPORTS_DIR = ROOT / "outputs" / "reports"

ERODE_SIZES = [5]
ERODE_ITERS = [1]
MIN_CONFIDENCE_VALS = [0.0]
CHROMATIC_MIN_CONF_VALS = [0.10, 0.15, 0.20, 0.25, 0.30]
IGNORE_DARK = [None, 30, 50, 70, 90]
IGNORE_BRIGHT = [None, 200, 220, 240]


def evaluate_config(
    images, lab_ranges, params
):
    correct = 0
    wrong_high_conf = 0
    per_color = defaultdict(lambda: {"total": 0, "correct": 0})
    confusion = defaultdict(Counter)
    detail = []

    for img_path, expected_color, expected_class in images:
        bgr = cv2.imread(str(img_path))

        pred_color, confidence, hits, _ = estimate_color_with_thresholds(
            bgr, lab_ranges,
            erode_size=params["erode_size"],
            erode_iterations=params["erode_iterations"],
            ignore_l_min=params["ignore_dark"],
            ignore_l_max=params["ignore_bright"],
            min_confidence=params["min_confidence"],
            chromatic_min_confidence=params["chromatic_min"],
        )

        ok = (pred_color == expected_color)

        per_color[expected_color]["total"] += 1
        if ok:
            correct += 1
            per_color[expected_color]["correct"] += 1
        elif confidence >= 0.75:
            wrong_high_conf += 1

        confusion[expected_color][pred_color] += 1
        detail.append({
            "filename": img_path.name,
            "expected": expected_color,
            "predicted": pred_color,
            "confidence": confidence,
            "correct": ok,
        })

    total = len(images)
    acc = round(correct / total, 4) if total else 0.0
    per_color_acc = {
        c: round(v["correct"] / v["total"], 4) if v["total"] else 0.0
        for c, v in sorted(per_color.items())
    }

    return {
        "accuracy": acc,
        "correct": correct,
        "total": total,
        "per_color_accuracy": per_color_acc,
        "wrong_high_confidence": wrong_high_conf,
        "confusion": {k: dict(v) for k, v in sorted(confusion.items())},
        "detail": detail,
    }


def main():
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)

    lab_ranges, color_keys = load_lab_ranges(str(CONFIG_PATH))
    print(f"Colors: {color_keys}")

    image_files = sorted(IMAGES_DIR.glob("*.jpg"))
    images = []
    for p in image_files:
        try:
            color, cls = parse_filename_label(p.name)
            images.append((p, color, cls))
        except ValueError:
            pass
    print(f"Images: {len(images)}")

    all_results = []
    total_combos = (
        len(ERODE_SIZES) * len(ERODE_ITERS)
        * len(CHROMATIC_MIN_CONF_VALS)
        * len(MIN_CONFIDENCE_VALS)
        * len(IGNORE_DARK) * len(IGNORE_BRIGHT)
    )
    print(f"Grid size: {total_combos} combinations")
    print()

    count = 0
    top_acc = -1.0
    top_config = None

    for es in ERODE_SIZES:
        for ei in ERODE_ITERS:
            for cmc in CHROMATIC_MIN_CONF_VALS:
                for mc in MIN_CONFIDENCE_VALS:
                    for idark in IGNORE_DARK:
                        for ibright in IGNORE_BRIGHT:
                            count += 1

                            params = {
                                "erode_size": es,
                                "erode_iterations": ei,
                                "min_confidence": mc,
                                "chromatic_min": cmc,
                                "ignore_dark": idark,
                                "ignore_bright": ibright,
                            }

                            result = evaluate_config(images, lab_ranges, params)
                            result["params"] = params
                            all_results.append(result)

                            if result["accuracy"] > top_acc:
                                top_acc = result["accuracy"]
                                top_config = result

                            if count % 200 == 0:
                                print(f"  [{count}/{total_combos}] best={top_acc:.2%}  "
                                      f"erode={es}/{ei} chromatic_min={cmc} min_conf={mc} "
                                      f"dark={idark} bright={ibright}")

    print(f"\n  Done: {count} combinations evaluated")

    all_results.sort(key=lambda r: (
        -r["accuracy"],
        -min(r["per_color_accuracy"].values()) if r["per_color_accuracy"] else 0,
        r["wrong_high_confidence"],
    ))

    tuning_path = REPORTS_DIR / "tuning_results.json"
    with open(tuning_path, "w", encoding="utf-8") as f:
        json.dump(all_results[:50], f, indent=2, ensure_ascii=False)
    print(f"\n✅ {tuning_path}  (top 50 configs)")

    best_path = REPORTS_DIR / "best_config.json"
    with open(best_path, "w", encoding="utf-8") as f:
        json.dump(top_config, f, indent=2, ensure_ascii=False)
    print(f"✅ {best_path}")

    report_path = REPORTS_DIR / "tuning_report.md"
    write_report(report_path, all_results[:10], top_config, lab_ranges)
    print(f"✅ {report_path}")

    print(f"\n{'='*60}")
    print(f"  Best accuracy: {top_config['accuracy']:.2%}  ({top_config['correct']}/{top_config['total']})")
    print(f"  Per-color: {top_config['per_color_accuracy']}")
    print(f"  Wrong high-conf: {top_config['wrong_high_confidence']}")
    print(f"  Params: {top_config['params']}")
    print(f"{'='*60}")


def write_report(report_path, top10, best, lab_ranges):
    lines = []
    lines.append("# ROI Color Tuning Report\n")
    lines.append(f"**Grid size**: {len(ERODE_SIZES)}×{len(ERODE_ITERS)}×{len(CHROMATIC_MIN_CONF_VALS)}×"
                 f"{len(MIN_CONFIDENCE_VALS)}×{len(IGNORE_DARK)}×{len(IGNORE_BRIGHT)} = "
                 f"{len(ERODE_SIZES)*len(ERODE_ITERS)*len(CHROMATIC_MIN_CONF_VALS)*len(MIN_CONFIDENCE_VALS)*len(IGNORE_DARK)*len(IGNORE_BRIGHT)}\n")

    lines.append("## Best Configuration\n")
    lines.append("| Parameter | Value |")
    lines.append("|-----------|-------|")
    for k, v in best["params"].items():
        lines.append(f"| {k} | {v} |")
    lines.append(f"\n**Accuracy**: {best['accuracy']:.2%} ({best['correct']}/{best['total']})")
    lines.append(f"**Per-color**: {best['per_color_accuracy']}")
    lines.append(f"**Wrong high-confidence**: {best['wrong_high_confidence']}\n")

    lines.append("## Top 10 Configurations\n")
    lines.append("| # | Accuracy | Per-Color | Wrong HiConf | Params |")
    lines.append("|---|----------|-----------|-------------|--------|")
    for i, r in enumerate(top10):
        params_str = (f"e={r['params']['erode_size']}/{r['params']['erode_iterations']} "
                      f"cmc={r['params']['chromatic_min']} "
                      f"mc={r['params']['min_confidence']} "
                      f"dark={r['params']['ignore_dark']} bright={r['params']['ignore_bright']}")
        lines.append(f"| {i+1} | {r['accuracy']:.2%} | {r['per_color_accuracy']} | {r['wrong_high_confidence']} | {params_str} |")
    lines.append("")

    lines.append("## Tuning Recommendations\n")
    lines.append(f"- **Chromatic priority**: best `chromatic_min_confidence={best['params']['chromatic_min']}`. "
                 "Chromatic colors (red/blue/green/yellow/purple/tennis) take priority over neutral (black/white) when their confidence exceeds this threshold.")
    lines.append(f"- **Luminance thresholds**: dark={best['params']['ignore_dark']}, bright={best['params']['ignore_bright']}. "
                 "Pixels outside these L bounds are excluded from color voting.")
    lines.append(f"- **Erode**: kernel={best['params']['erode_size']}, iterations={best['params']['erode_iterations']}. "
                 "Removes boundary/shadow pixels before color sampling.")
    lines.append(f"- **Min confidence**: {best['params']['min_confidence']}. "
                 "Predictions below this threshold are marked 'unknown'.")
    if best["accuracy"] < 0.80:
        lines.append("\n⚠ Accuracy below 80% — LAB ranges in lab_config.yaml likely need manual adjustment for this lighting condition.")
    lines.append("\n## Next Steps\n")
    lines.append("1. Human review `best_config.json`")
    lines.append("2. Do NOT auto-apply to lab_config.yaml or roi_color_detector_node.py")
    lines.append("3. Consider narrowing the 'black' LAB a-range (currently 30-187) to reduce chromatic overlap")
    lines.append("4. Re-run evaluation with adjusted LAB ranges once approved")

    with open(report_path, "w", encoding="utf-8") as f:
        f.write("".join(lines))


if __name__ == "__main__":
    main()
