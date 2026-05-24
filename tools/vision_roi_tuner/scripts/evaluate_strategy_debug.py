#!/usr/bin/env python3
"""Visual debug evaluator using the best strategy from the experiment loop."""

import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

from roi_color_detection_utils import (
    build_debug_overlay,
    detect_object_contour,
    dominant_color_key_with_priority,
    erode_contour_mask,
    load_lab_ranges,
)

ROOT = Path(__file__).resolve().parent.parent
IMAGES_DIR = ROOT / "inputs" / "images"
CONFIG_PATH = ROOT / "configs" / "lab_config.yaml"
LABELS_PATH = ROOT / "inputs" / "labels" / "manual_labels.json"
OUTPUT_DIR = ROOT / "outputs" / "reports_v2"
DEBUG_DIR = ROOT / "outputs" / "debug_images_v2"
STRATEGY_PATH = ROOT / "outputs" / "reports" / "best_strategy.json"

FALLBACK_STRATEGY = {
    "name": "Otsu + Chroma + adjusted LAB ranges",
    "params": {
        "use_otsu": True,
        "use_chroma": True,
        "chroma_threshold": 5.0,
        "chromatic_min_confidence": 0.10,
        "min_confidence": 0.2,
        "lab_range_overrides": {
            "blue": {"B": (59, 143)},
            "red": {"A": (124, 183), "B": (125, 189)},
        },
    },
}


def estimate_with_strategy(bgr_img, lab_ranges, params):
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    contour = detect_object_contour(gray, min_area=500)
    if contour is None:
        return "unknown", 0.0, {}, None, None

    mask = erode_contour_mask(gray.shape, contour, kernel_size=params.get("erode_size", 5))
    lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
    mask_full = mask.copy()

    if params.get("use_otsu"):
        vals = gray[mask > 0].astype(np.uint8)
        if len(vals) > 0:
            otsu_val, _ = cv2.threshold(vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            mask[(mask > 0) & (gray < otsu_val)] = 0
    mask_otsu = mask.copy() if params.get("use_otsu") else mask_full

    if params.get("use_chroma") and (mask > 0).sum() > 0:
        Lab = lab[:, :, 1].astype(float)
        Bab = lab[:, :, 2].astype(float)
        C = np.sqrt((Lab - 128.0) ** 2 + (Bab - 128.0) ** 2)
        ct = params.get("chroma_threshold", 5.0)
        mask[(mask > 0) & (C < ct)] = 0
    mask_chroma = mask.copy() if params.get("use_chroma") else mask_otsu

    ranges = lab_ranges
    overrides = params.get("lab_range_overrides")
    if overrides:
        ranges = dict(lab_ranges)
        for color, ch_overrides in overrides.items():
            if color in ranges:
                om, ox = ranges[color]
                nm, nx = list(om), list(ox)
                for ch, (lo, hi) in ch_overrides.items():
                    idx = {"L": 0, "A": 1, "B": 2}[ch]
                    nm[idx], nx[idx] = lo, hi
                ranges[color] = (tuple(nm), tuple(nx))

    cmc = params.get("chromatic_min_confidence", 0.10)
    pred_color, confidence, hits = dominant_color_key_with_priority(
        lab, mask, ranges, cmc)

    raw_pred = pred_color
    min_conf = params.get("min_confidence", 0.0)
    filtered = False
    if min_conf > 0 and confidence < min_conf:
        pred_color = "unknown"
        filtered = True

    mask_info = {
        "full": mask_full,
        "otsu": mask_otsu,
        "chroma": mask_chroma,
        "final": mask,
    }
    return pred_color, confidence, raw_pred, filtered, hits, contour, mask_info


def build_debug_v2(img, contour, masks, expected, pred_color, conf, raw_pred, filtered, hits, strategy_name):
    h, w = img.shape[:2]
    panel = np.ones((h, w + 320, 3), dtype=np.uint8) * 30
    panel[:h, :w] = img.copy()

    x0 = w + 8

    # Contour on main image
    if contour is not None:
        cv2.drawContours(panel[:h, :w], [contour], -1, (0, 255, 0), 2)

    correct = (pred_color == expected) if expected else None
    status = "OK" if correct else ("WRONG" if correct is False else "?")
    tc = (0, 255, 0) if correct else ((0, 0, 255) if correct is False else (200, 200, 0))
    cv2.putText(panel, f"GT:  {expected or 'N/A'}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    if filtered:
        cv2.putText(panel, f"PRED: {pred_color} (raw={raw_pred}, conf={conf:.3f}) {status}", (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, tc, 2)
    else:
        cv2.putText(panel, f"PRED: {pred_color} ({conf:.3f}) {status}", (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, tc, 2)

    y = 16
    cv2.putText(panel, f"Strategy: {strategy_name[:30]}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y += 18

    mask_names = [("Full Mask", masks["full"]), ("Otsu Body", masks["otsu"]), ("Chroma", masks["chroma"]), ("Final", masks["final"])]
    panel_h = min(80, (h - y - 100) // max(1, len(mask_names)))
    thumb_w = 180
    for name, m in mask_names:
        thumb = np.zeros((panel_h, thumb_w, 3), dtype=np.uint8)
        m_small = cv2.resize(m[:h, :w].astype(np.uint8), (thumb_w, panel_h), interpolation=cv2.INTER_NEAREST)
        thumb[m_small > 0] = (0, 255, 0)
        pct = (m > 0).sum() / max(1, (masks["full"] > 0).sum()) * 100
        cv2.putText(thumb, f"{name} ({pct:.0f}%)", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        panel[y:y+panel_h, x0:x0+thumb_w] = thumb
        y += panel_h + 6

    # Bar chart
    if hits:
        y += 4
        bar_max_h = 60
        max_hits = max(hits.values()) if hits else 1
        keys_sorted = sorted(hits.keys())
        bar_w = max(6, 150 // max(1, len(keys_sorted)) - 2)
        for i, k in enumerate(keys_sorted):
            bh = int((hits[k] / max_hits) * bar_max_h) if max_hits > 0 else 0
            bx = x0 + i * (bar_w + 2)
            by = y + bar_max_h - bh
            bcol = (100, 200, 100) if k == pred_color else (100, 100, 100)
            cv2.rectangle(panel, (bx, by), (bx + bar_w, y + bar_max_h), bcol, -1)
            cv2.putText(panel, f"{k[:4]} {hits[k]}", (bx, y + bar_max_h + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (180, 180, 180), 1)

    return panel


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    lab_ranges, _ = load_lab_ranges(str(CONFIG_PATH))

    if STRATEGY_PATH.exists():
        with open(STRATEGY_PATH) as f:
            strategy = json.load(f)
    else:
        strategy = FALLBACK_STRATEGY

    params = strategy["params"]
    strategy_name = strategy.get("name", "best_strategy")
    print(f"Strategy: {strategy_name}")
    print(f"Params: {params}")

    manual_labels = {}
    if LABELS_PATH.exists():
        with open(LABELS_PATH) as f:
            for entry in json.load(f):
                manual_labels[entry["image"]] = entry

    image_files = sorted(IMAGES_DIR.glob("*.jpg"))
    results = []
    correct = 0
    confusion = defaultdict(Counter)

    for img_path in image_files:
        manual = manual_labels.get(img_path.name, {})
        expected = manual.get("true_color")

        bgr = cv2.imread(str(img_path))
        pred_color, confidence, raw_pred, filtered, hits, contour, masks = estimate_with_strategy(
            bgr, lab_ranges, params)

        ok = (pred_color == expected) if expected else None
        if ok:
            correct += 1
        if expected:
            confusion[expected][pred_color] += 1

        results.append({
            "image": img_path.name,
            "expected": expected,
            "raw_predicted": raw_pred,
            "predicted": pred_color,
            "confidence": confidence,
            "filtered_by_confidence": filtered,
            "correct": ok,
        })

        status = "OK" if ok else ("WRONG" if ok is False else "?")
        filt_tag = " [FILT]" if filtered else ""
        print(f"  {img_path.name:28s}  GT={str(expected):5s}  PRED={pred_color:8s}{filt_tag}  conf={confidence:.3f}  {status}")

        dbg = build_debug_v2(bgr, contour, masks, expected, pred_color,
                             confidence, raw_pred, filtered, hits, strategy_name)
        cv2.imwrite(str(DEBUG_DIR / (img_path.stem + "_v2.png")), dbg)

    total_labeled = sum(1 for r in results if r["expected"] is not None)
    labeled_correct = sum(1 for r in results if r["correct"] is True)

    summary = {
        "strategy": strategy_name,
        "params": params,
        "total_images": len(results),
        "labeled_images": total_labeled,
        "labeled_correct": labeled_correct,
        "labeled_accuracy": round(labeled_correct / total_labeled, 4) if total_labeled else 0.0,
        "per_color_accuracy": {},
        "confusion": {k: dict(v) for k, v in sorted(confusion.items())},
        "per_image": results,
    }

    json_path = OUTPUT_DIR / "strategy_debug_eval.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"\n✅ {json_path}")

    md = []
    md.append("# Strategy Debug Evaluation\n")
    md.append(f"**Strategy**: {strategy_name}\n")
    md.append(f"**Params**: `{json.dumps(params)}`\n")
    md.append(f"**Labeled images**: {total_labeled}\n")
    md.append(f"**Labeled accuracy**: {labeled_correct}/{total_labeled} = {summary['labeled_accuracy']:.2%}\n")

    md.append("## Confusion Matrix\n")
    all_keys = sorted(set().union(*[v.keys() for v in confusion.values()], confusion.keys()))
    if all_keys:
        header = "| True | " + " | ".join(all_keys) + " |"
        sep = "|" + "|".join(["---"] * (len(all_keys) + 1)) + "|"
        md.append(header)
        md.append(sep)
        for exp in sorted(confusion.keys()):
            row = f"| {exp} " + " ".join(f"| {confusion[exp].get(pred, 0)} " for pred in all_keys) + "|"
            md.append(row)
    md.append("")

    md.append("## Per-Image Detail\n")
    for r in results:
        s = "✅" if r["correct"] else ("❌" if r["correct"] is False else "⬜")
        f_tag = " [FILT]" if r.get("filtered_by_confidence") else ""
        md.append(f"| {r['image']} | {r['expected'] or 'N/A'} | {r['raw_predicted']} → {r['predicted']} | {r['confidence']:.3f}{f_tag} | {s} |")

    md_path = OUTPUT_DIR / "strategy_debug_eval.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md))
    print(f"✅ {md_path}")
    print(f"✅ {DEBUG_DIR}/ ({len(results)} debug images)")


if __name__ == "__main__":
    main()
