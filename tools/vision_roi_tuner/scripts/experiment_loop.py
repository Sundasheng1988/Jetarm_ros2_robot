#!/usr/bin/env python3
"""Autonomous experiment loop for ROI color accuracy improvement."""

import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2

from roi_color_detection_utils import (
    estimate_color_with_otsu_chroma,
    load_lab_ranges,
    parse_filename_label,
)

ROOT = Path(__file__).resolve().parent.parent
IMAGES_DIR = ROOT / "inputs" / "images"
CONFIG_PATH = ROOT / "configs" / "lab_config.yaml"
REPORTS_DIR = ROOT / "outputs" / "reports"

MAX_ITERATIONS = 5


def evaluate(method_params, lab_ranges):
    images = []
    for p in sorted(IMAGES_DIR.glob("*.jpg")):
        try:
            color, cls = parse_filename_label(p.name)
            images.append((p, color, cls))
        except ValueError:
            pass

    correct = 0
    wrong_high_conf = 0
    per_color = defaultdict(lambda: {"total": 0, "correct": 0})
    confusion = defaultdict(Counter)
    detail = []

    for img_path, expected_color, _ in images:
        bgr = cv2.imread(str(img_path))

        pred_color, confidence, _, _ = estimate_color_with_otsu_chroma(
            bgr, lab_ranges, **method_params)

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

    experiments = [
        {
            "name": "Baseline (chromatic priority only)",
            "params": {"use_otsu": False, "use_chroma": False},
            "reason": "Establish baseline with chromatic priority voting, no mask refinement",
        },
        {
            "name": "Otsu body extraction",
            "params": {"use_otsu": True, "use_chroma": False},
            "reason": "Internal Otsu removes shadow/background leaks from mask before color voting",
        },
        {
            "name": "Otsu + Chroma filtering (C>5)",
            "params": {"use_otsu": True, "use_chroma": True, "chroma_threshold": 5.0},
            "reason": "Otsu removes shadow, Chroma removes residual neutral pixels. Combined filtering.",
        },
        {
            "name": "Otsu + Chroma + adjusted LAB ranges",
            "params": {
                "use_otsu": True, "use_chroma": True, "chroma_threshold": 5.0,
                "lab_range_overrides": {
                    "blue": {"B": (59, 143)},
                    "red": {"A": (124, 183), "B": (125, 189)},
                },
            },
            "reason": "Expand blue B-max (119→143) and red A-min (142→124) to match observed LAB distributions from v0.3 analysis",
        },
        {
            "name": "Otsu + Chroma + ranges + confidence tuning",
            "params": {
                "use_otsu": True, "use_chroma": True, "chroma_threshold": 5.0,
                "chromatic_min_confidence": 0.15,
                "min_confidence": 0.45,
                "lab_range_overrides": {
                    "blue": {"B": (59, 143)},
                    "red": {"A": (124, 183), "B": (125, 189)},
                },
            },
            "reason": "Raise chromatic_min to 0.15 and add min_confidence=0.45 to suppress weak predictions",
        },
    ]

    log_lines = []
    log_lines.append("# Experiment Log\n")
    log_lines.append(f"**Date**: 2026-05-24\n")
    log_lines.append(f"**Images**: 20 (14 blue, 6 red)\n")
    log_lines.append(f"**Max iterations**: {MAX_ITERATIONS}\n")

    best_result = None
    best_params = None
    best_name = None
    previous_acc = -1.0

    for i, exp in enumerate(experiments):
        if i >= MAX_ITERATIONS:
            break

        name = exp["name"]
        params = exp["params"]
        reason = exp["reason"]

        print(f"\n{'='*60}")
        print(f"  Iteration {i+1}/{MAX_ITERATIONS}: {name}")
        print(f"  Reason: {reason}")
        print(f"  Params: {params}")
        print(f"{'='*60}")

        result = evaluate(params, lab_ranges)

        acc = result["accuracy"]
        red_acc = result["per_color_accuracy"].get("red", 0.0)
        blue_acc = result["per_color_accuracy"].get("blue", 0.0)
        wrong_high = result["wrong_high_confidence"]
        false_black = sum(
            1 for d in result["detail"] if not d["correct"] and d["predicted"] == "black"
        )

        improved = acc > previous_acc or (acc == previous_acc and red_acc > 0 and best_result is None)
        verdict = "KEEP" if improved else "ROLLBACK"

        log_lines.append(f"### Iter {i+1}: {name}\n")
        log_lines.append(f"| Metric | Value |\n")
        log_lines.append(f"|--------|-------|\n")
        log_lines.append(f"| Accuracy | {acc:.2%} ({result['correct']}/{result['total']}) |\n")
        log_lines.append(f"| Blue | {blue_acc:.2%} |\n")
        log_lines.append(f"| Red | {red_acc:.2%} |\n")
        log_lines.append(f"| Wrong high-conf | {wrong_high} |\n")
        log_lines.append(f"| False black | {false_black} |\n")
        log_lines.append(f"| Verdict | **{verdict}** |\n")
        log_lines.append(f"| Reason | {reason} |\n")
        log_lines.append(f"| Params | `{json.dumps(params)}` |\n")
        log_lines.append("")

        print(f"  Accuracy: {acc:.2%}  Blue: {blue_acc:.2%}  Red: {red_acc:.2%}  WrongHiConf: {wrong_high}  FalseBlack: {false_black}")
        print(f"  Verdict: {verdict}")

        if improved:
            best_result = result
            best_params = params
            best_name = name
            previous_acc = acc

        # Show confusion
        for exp_c, preds in sorted(result["confusion"].items()):
            print(f"    {exp_c} → {preds}")

    # Write experiment log
    log_path = REPORTS_DIR / "experiment_log.md"
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("".join(log_lines))
    print(f"\n✅ {log_path}")

    # Write best strategy
    if best_result:
        best_strategy = {
            "name": best_name,
            "params": best_params,
            "result": {
                "accuracy": best_result["accuracy"],
                "correct": best_result["correct"],
                "total": best_result["total"],
                "per_color_accuracy": best_result["per_color_accuracy"],
                "wrong_high_confidence": best_result["wrong_high_confidence"],
                "confusion": best_result["confusion"],
            },
        }
        best_path = REPORTS_DIR / "best_strategy.json"
        with open(best_path, "w", encoding="utf-8") as f:
            json.dump(best_strategy, f, indent=2, ensure_ascii=False)
        print(f"✅ {best_path}")

        report_lines = []
        report_lines.append("# ROI Color Best Strategy Report\n")
        report_lines.append(f"**Best iteration**: {best_name}\n")
        report_lines.append(f"**Accuracy**: {best_result['accuracy']:.2%} ({best_result['correct']}/{best_result['total']})\n")
        report_lines.append(f"**Per-color**: {best_result['per_color_accuracy']}\n")
        report_lines.append(f"**Params**: `{json.dumps(best_params)}`\n")
        report_lines.append("\n## Confusion Matrix\n")
        for exp_c, preds in sorted(best_result["confusion"].items()):
            report_lines.append(f"- **{exp_c}**: {preds}\n")
        report_lines.append("\n## Details\n")
        for d in best_result["detail"]:
            s = "OK" if d["correct"] else "WRONG"
            report_lines.append(f"- {d['filename']:30s} {d['expected']:6s} → {d['predicted']:8s} ({d['confidence']:.3f}) {s}\n")

        br_path = REPORTS_DIR / "best_report.md"
        with open(br_path, "w", encoding="utf-8") as f:
            f.write("".join(report_lines))
        print(f"✅ {br_path}")

    print(f"\n{'='*60}")
    print(f"  Best: {best_name}")
    print(f"  Accuracy: {best_result['accuracy']:.2%}")
    print(f"  Per-color: {best_result['per_color_accuracy']}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
