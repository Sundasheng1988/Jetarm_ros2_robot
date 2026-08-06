#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LAB color distribution analyzer.
Extracts LAB pixels from object contours and compares against current config ranges.
"""

import json
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from roi_color_detection_utils import (
    detect_object_contour,
    erode_contour_mask,
    load_lab_ranges,
    parse_filename_label,
)

ROOT = Path(__file__).resolve().parent.parent
IMAGES_DIR = ROOT / "inputs" / "images"
CONFIG_PATH = ROOT / "configs" / "lab_config.yaml"
REPORTS_DIR = ROOT / "outputs" / "reports"
DEBUG_DIR = ROOT / "outputs" / "debug_images" / "lab"


def extract_lab_pixels(bgr_img, contour, erode_size=5):
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    if contour is None:
        contour = detect_object_contour(gray)
        if contour is None:
            return None

    mask = erode_contour_mask(gray.shape, contour, kernel_size=erode_size)
    lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
    indices = mask > 0
    L = lab[:, :, 0][indices]
    A = lab[:, :, 1][indices]
    B = lab[:, :, 2][indices]
    return L.astype(np.float64), A.astype(np.float64), B.astype(np.float64)


def compute_stats(values):
    if len(values) == 0:
        return {"mean": 0, "std": 0, "min": 0, "max": 0, "count": 0,
                "p5": 0, "p25": 0, "p50": 0, "p75": 0, "p95": 0}
    return {
        "mean": round(float(np.mean(values)), 2),
        "std": round(float(np.std(values)), 2),
        "min": int(np.min(values)),
        "max": int(np.max(values)),
        "count": int(len(values)),
        "p5": int(np.percentile(values, 5)),
        "p25": int(np.percentile(values, 25)),
        "p50": int(np.percentile(values, 50)),
        "p75": int(np.percentile(values, 75)),
        "p95": int(np.percentile(values, 95)),
    }


def compare_range(config_min, config_max, observed_stats):
    obs_min = observed_stats["p5"]
    obs_max = observed_stats["p95"]
    config_span = max(config_max - config_min, 1)

    overlap_min = max(config_min, obs_min)
    overlap_max = min(config_max, obs_max)
    overlap_width = max(0, overlap_max - overlap_min)
    overlap_pct = round(overlap_width / config_span * 100, 1) if config_span > 0 else 0.0
    obs_span = max(obs_max - obs_min, 1)
    coverage = round(overlap_width / obs_span * 100, 1)

    obs_mean = observed_stats["mean"]
    obs_std = observed_stats["std"]
    suggested_min = int(max(0, obs_mean - 2.0 * obs_std))
    suggested_max = int(min(255, obs_mean + 2.0 * obs_std))

    return {
        "config_range": [int(config_min), int(config_max)],
        "observed_p5_p95": [obs_min, obs_max],
        "observed_mean_2std": [suggested_min, suggested_max],
        "overlap_pct": overlap_pct,
        "coverage_pct": coverage,
    }


def save_histogram_plot(L, A, B, filename, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    plot_w, plot_h = 900, 700
    img = np.ones((plot_h, plot_w, 3), dtype=np.uint8) * 30

    colors = [(200, 80, 80), (80, 200, 80), (200, 200, 80)]
    labels = ["L (luminance)", "A (green-red)", "B (blue-yellow)"]
    data = [L, A, B]
    xlims = [(0, 255), (0, 255), (0, 255)]

    for ch_idx in range(3):
        y0 = 30 + ch_idx * 210
        h = 180
        vals = data[ch_idx]
        hist, bins = np.histogram(vals, bins=64, range=(0, 255))
        hist = hist / max(hist.max(), 1)

        x0 = 60
        w = plot_w - 100

        for i in range(len(hist)):
            bx = int(x0 + i * w / len(hist))
            bw = max(1, int(w / len(hist)) - 1)
            by = int(y0 + h - hist[i] * h)
            cv2.rectangle(img, (bx, by), (bx + bw, y0 + h), colors[ch_idx], -1)

        cv2.putText(img, labels[ch_idx], (x0, y0 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[ch_idx], 1)
        cv2.putText(img, f"mean={vals.mean():.1f} std={vals.std():.1f} n={len(vals)}",
                    (x0 + w - 280, y0 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    cv2.imwrite(str(out_dir / filename), img)


def main():
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    lab_ranges, color_keys = load_lab_ranges(str(CONFIG_PATH))
    print(f"Config colors: {color_keys}")

    image_files = sorted(IMAGES_DIR.glob("*.jpg"))
    per_color_pixels = defaultdict(lambda: {"L": [], "A": [], "B": []})
    per_image = {}

    for img_path in image_files:
        try:
            expected_color, expected_class = parse_filename_label(img_path.name)
        except ValueError:
            continue

        bgr = cv2.imread(str(img_path))
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        contour = detect_object_contour(gray)
        if contour is None:
            print(f"  SKIP {img_path.name}: no contour")
            continue

        result = extract_lab_pixels(bgr, contour)
        if result is None:
            continue
        L, A, B = result

        stats_L = compute_stats(L)
        stats_A = compute_stats(A)
        stats_B = compute_stats(B)

        per_image[img_path.name] = {
            "expected_color": expected_color,
            "expected_class": expected_class,
            "pixel_count": int(len(L)),
            "L": stats_L,
            "A": stats_A,
            "B": stats_B,
        }

        per_color_pixels[expected_color]["L"].extend(L.tolist())
        per_color_pixels[expected_color]["A"].extend(A.tolist())
        per_color_pixels[expected_color]["B"].extend(B.tolist())

        save_histogram_plot(L, A, B, img_path.stem + "_hist.png", DEBUG_DIR)
        print(f"  {img_path.name:28s}  L={stats_L['mean']:.0f}±{stats_L['std']:.0f}  "
              f"A={stats_A['mean']:.0f}±{stats_A['std']:.0f}  "
              f"B={stats_B['mean']:.0f}±{stats_B['std']:.0f}  n={len(L)}")

    per_color_stats = {}
    for color in sorted(per_color_pixels.keys()):
        px = per_color_pixels[color]
        stats = {
            "total_pixels": len(px["L"]),
            "L": compute_stats(np.array(px["L"])),
            "A": compute_stats(np.array(px["A"])),
            "B": compute_stats(np.array(px["B"])),
        }
        per_color_stats[color] = stats

        print(f"\n=== {color} ({stats['total_pixels']} pixels across images) ===")
        for ch in ["L", "A", "B"]:
            s = stats[ch]
            idx = {"L": 0, "A": 1, "B": 2}[ch]
            if color in lab_ranges:
                cmin = lab_ranges[color][0][idx]
                cmax = lab_ranges[color][1][idx]
                cfg_str = f"[{cmin}, {cmax}]"
            else:
                cfg_str = "N/A"
            print(f"  {ch}: mean={s['mean']:.1f} std={s['std']:.1f}  "
                  f"[{s['p5']}, {s['p95']}] (90% range)  "
                  f"config={cfg_str}")

    range_comparison = {}
    for color in per_color_stats:
        if color not in lab_ranges:
            continue
        config_min, config_max = lab_ranges[color]
        obs = per_color_stats[color]
        range_comparison[color] = {
            "L": compare_range(config_min[0], config_max[0], obs["L"]),
            "A": compare_range(config_min[1], config_max[1], obs["A"]),
            "B": compare_range(config_min[2], config_max[2], obs["B"]),
        }

    result = {
        "per_color_stats": per_color_stats,
        "per_image": per_image,
        "range_comparison": range_comparison,
    }

    json_path = REPORTS_DIR / "lab_distribution.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"\n✅ {json_path}")

    md_path = REPORTS_DIR / "lab_distribution.md"
    write_report(md_path, per_color_stats, range_comparison, lab_ranges, per_image)
    print(f"✅ {md_path}")
    print(f"✅ {DEBUG_DIR}/  ({len(per_image)} histograms)")


def write_report(md_path, per_color_stats, range_comparison, lab_ranges, per_image):
    lines = []
    lines.append("# LAB Color Distribution Analysis\n")

    for color in sorted(per_color_stats.keys()):
        stats = per_color_stats[color]
        lines.append(f"## {color} ({stats['total_pixels']} pixels)\n")

        lines.append("| Channel | Mean | Std | P5 | P25 | P50 | P75 | P95 | Config Range | Observed (P5-P95) | Overlap % |")
        lines.append("|---------|------|-----|----|-----|-----|-----|-----|-------------|-------------------|-----------|")

        for ch_idx, ch in enumerate(["L", "A", "B"]):
            s = stats[ch]
            config_str = "N/A"
            obs_str = f"[{s['p5']}, {s['p95']}]"
            overlap_str = "N/A"

            if color in range_comparison:
                rc = range_comparison[color][ch]
                config_str = f"[{rc['config_range'][0]}, {rc['config_range'][1]}]"
                overlap_str = f"{rc['overlap_pct']}%"

            lines.append(
                f"| {ch} | {s['mean']:.1f} | {s['std']:.1f} | "
                f"{s['p5']} | {s['p25']} | {s['p50']} | {s['p75']} | {s['p95']} | "
                f"{config_str} | {obs_str} | {overlap_str} |"
            )
        lines.append("")

        if color in range_comparison:
            lines.append(f"### Suggested Range for {color}\n")
            lines.append("| Channel | Current Config | Observed (mean±2σ) | Suggested | Overlap % |")
            lines.append("|---------|---------------|---------------------|-----------|-----------|")
            for ch_idx, ch in enumerate(["L", "A", "B"]):
                rc = range_comparison[color][ch]
                suggested = rc["observed_mean_2std"]
                lines.append(
                    f"| {ch} | {rc['config_range']} | {suggested} | "
                    f"{'keep' if rc['overlap_pct'] > 70 else 'review'}"
                    f" | {rc['overlap_pct']}% |"
                )
            lines.append("")

    lines.append("## Diagnosis: Why Red Collapses\n")
    red = per_color_stats.get("red", {})
    blue = per_color_stats.get("blue", {})

    if red and blue:
        lines.append(f"- **Blue**: A={blue['A']['mean']:.0f}±{blue['A']['std']:.0f}, "
                     f"B={blue['B']['mean']:.0f}±{blue['B']['std']:.0f}")
        lines.append(f"- **Red**: A={red['A']['mean']:.0f}±{red['A']['std']:.0f}, "
                     f"B={red['B']['mean']:.0f}±{red['B']['std']:.0f}")
        lines.append("")

        if "red" in range_comparison:
            rc = range_comparison["red"]
            for ch in ["L", "A", "B"]:
                r = rc[ch]
                if r["overlap_pct"] < 50:
                    lines.append(
                        f"- **{ch} channel mismatch**: config [{r['config_range'][0]}, {r['config_range'][1]}] "
                        f"vs observed P5-P95 [{r['observed_p5_p95'][0]}, {r['observed_p5_p95'][1]}]. "
                        f"Only {r['overlap_pct']}% overlap."
                    )

    lines.append("\n## Next Steps\n")
    lines.append("1. Compare observed L/A/B ranges per color against config")
    lines.append("2. For channels with <50% overlap, adjust LAB config ranges")
    lines.append("3. Re-run evaluate_roi_color.py to verify improvement")
    lines.append("4. Do NOT auto-modify lab_config.yaml")

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("".join(lines))


if __name__ == "__main__":
    main()
