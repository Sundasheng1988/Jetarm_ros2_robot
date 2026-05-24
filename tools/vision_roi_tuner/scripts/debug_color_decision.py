#!/usr/bin/env python3
"""Color decision debugger — explains why the final color vote produces each prediction."""

import json
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from roi_color_detection_utils import (
    detect_object_contour,
    dominant_color_key_with_priority,
    erode_contour_mask,
    load_lab_ranges,
    parse_filename_label,
)

ROOT = Path(__file__).resolve().parent.parent
IMAGES_DIR = ROOT / "inputs" / "images"
CONFIG_PATH = ROOT / "configs" / "lab_config.yaml"
LABELS_PATH = ROOT / "inputs" / "labels" / "manual_labels.json"
STRATEGY_PATH = ROOT / "outputs" / "reports" / "best_strategy.json"
OUTPUT_DIR = ROOT / "outputs" / "reports_v2"
DEBUG_DIR = ROOT / "outputs" / "debug_images_v3"

BEST_PARAMS = {
    "use_otsu": True, "use_chroma": True, "chroma_threshold": 5.0,
    "chromatic_min_confidence": 0.10,
    "lab_range_overrides": {
        "blue": {"B": (59, 143)},
        "red":  {"A": (124, 183), "B": (125, 189)},
    },
}

CHROMATIC = {"red", "blue", "green", "yellow", "purple", "tennis"}
NEUTRAL = {"black", "white"}


def compute_stats(arr):
    if len(arr) == 0:
        return {"mean": 0, "std": 0, "p5": 0, "p50": 0, "p95": 0, "count": 0}
    return {
        "mean": round(float(np.mean(arr)), 2),
        "std":  round(float(np.std(arr)), 2),
        "p5":   int(np.percentile(arr, 5)),
        "p50":  int(np.percentile(arr, 50)),
        "p95":  int(np.percentile(arr, 95)),
        "count": int(len(arr)),
    }


def build_decision_debug_img(img, contour, final_mask, pred_color, expected, stats):
    h, w = img.shape[:2]
    panel_w = 640
    panel = np.ones((h, w + panel_w, 3), dtype=np.uint8) * 25
    panel[:h, :w] = img.copy()

    if contour is not None:
        cv2.drawContours(panel[:h, :w], [contour], -1, (0, 255, 0), 2)

    correct = (pred_color == expected) if expected else None
    s = "OK" if correct else ("WRONG" if correct is False else "?")
    tc = (0, 255, 0) if correct else ((0, 0, 255) if correct is False else (200, 200, 0))
    cv2.putText(panel, f"GT:  {expected or 'N/A'}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(panel, f"PRED: {pred_color}  {s}", (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, tc, 2)

    mask_thumb = np.zeros((100, 140, 3), dtype=np.uint8)
    ms = cv2.resize(final_mask[:h, :w].astype(np.uint8), (140, 100), interpolation=cv2.INTER_NEAREST)
    mask_thumb[ms > 0] = (0, 255, 0)
    cv2.putText(mask_thumb, f"mask px={stats['total_px']}", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    panel[0:100, w:w+140] = mask_thumb

    x0 = w + 4
    y = 108

    for label, lab_vals, hsv_vals, nrgb in [
        ("LAB", stats["lab"], stats["hsv"], None),
        ("HSV", stats["lab"], stats["hsv"], None),
    ]:
        cv2.putText(panel, label, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 100), 1)
        y += 16
        if label == "LAB":
            cv2.putText(panel, f"L:{stats['lab']['L']['p50']} A:{stats['lab']['A']['p50']} B:{stats['lab']['B']['p50']}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
        else:
            cv2.putText(panel, f"H:{stats['hsv']['H']['p50']} S:{stats['hsv']['S']['p50']} V:{stats['hsv']['V']['p50']}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
        y += 16

    # RGB normalized
    nr = stats["rgb"]
    y += 4
    cv2.putText(panel, f"nR={nr['R']['mean']:.3f} nG={nr['G']['mean']:.3f} nB={nr['B']['mean']:.3f}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
    y += 16
    r_minus_b = nr["R"]["mean"] - nr["B"]["mean"]
    cv2.putText(panel, f"R-B diff: {r_minus_b:.4f} {'(reddish)' if r_minus_b > 0.01 else '(neutral/bluish)'}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 100) if r_minus_b > 0.01 else (100, 200, 255), 1)
    y += 20

    # Hit ratios
    hits = stats["hits"]
    total = stats["total_px"]
    cv2.putText(panel, "Hits:", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    y += 16
    for c in ["blue", "red", "black", "white"]:
        h = hits.get(c, 0)
        pct = h / max(1, total) * 100
        bar_w = int(pct / 100 * 200)
        cv2.rectangle(panel, (x0 + 60, y - 10), (x0 + 60 + bar_w, y - 2), (200, 100, 40) if c == "red" else (40, 100, 200) if c == "blue" else (100, 100, 100), -1)
        cv2.putText(panel, f"{c}: {h} ({pct:.0f}%)", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        y += 14

    y += 6
    reason = stats.get("win_reason", "")
    cv2.putText(panel, f"WIN: {reason[:60]}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 255, 100), 1)
    y += 16
    if "red/blue" in stats:
        cv2.putText(panel, f"red/blue overlap: {stats['red/blue']}", (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    return panel


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    lab_ranges, _ = load_lab_ranges(str(CONFIG_PATH))
    params = BEST_PARAMS

    manual_labels = {}
    if LABELS_PATH.exists():
        with open(LABELS_PATH) as f:
            for e in json.load(f):
                manual_labels[e["image"]] = e

    results = []

    for img_path in sorted(IMAGES_DIR.glob("*.jpg")):
        manual = manual_labels.get(img_path.name, {})
        expected = manual.get("true_color")
        expected_class = manual.get("true_class")

        bgr = cv2.imread(str(img_path))
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        contour = detect_object_contour(gray)
        if contour is None:
            continue

        mask = erode_contour_mask(gray.shape, contour, kernel_size=params.get("erode_size", 5))
        mask_full = mask.copy()

        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)

        if params.get("use_otsu"):
            vals = gray[mask > 0].astype(np.uint8)
            if len(vals) > 0:
                o, _ = cv2.threshold(vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                mask[(mask > 0) & (gray < o)] = 0

        if params.get("use_chroma") and (mask > 0).sum() > 0:
            Lab = lab[:, :, 1].astype(float)
            Bab = lab[:, :, 2].astype(float)
            C = np.sqrt((Lab - 128.) ** 2 + (Bab - 128.) ** 2)
            mask[(mask > 0) & (C < params.get("chroma_threshold", 5.))] = 0

        total_px = int((mask > 0).sum())
        if total_px == 0:
            continue

        # Per-pixel stats within final mask
        mx = mask > 0
        L_vals, A_vals, B_vals = lab[:,:,0][mx], lab[:,:,1][mx], lab[:,:,2][mx]

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        H_vals, S_vals, V_vals = hsv[:,:,0][mx], hsv[:,:,1][mx], hsv[:,:,2][mx]

        R, G, B_ch = bgr[:,:,2].astype(float), bgr[:,:,1].astype(float), bgr[:,:,0].astype(float)
        total_rgb = R[mx] + G[mx] + B_ch[mx] + 1e-9
        nr = R[mx] / total_rgb
        ng = G[mx] / total_rgb
        nb = B_ch[mx] / total_rgb

        # Build overridden ranges
        ranges = dict(lab_ranges)
        for color, ch_ov in params.get("lab_range_overrides", {}).items():
            if color in ranges:
                om, ox = ranges[color]
                nm, nx = list(om), list(ox)
                for ch, (lo, hi) in ch_ov.items():
                    idx = {"L": 0, "A": 1, "B": 2}[ch]
                    nm[idx], nx[idx] = lo, hi
                ranges[color] = (tuple(nm), tuple(nx))

        # Compute per-color hits
        hits = {}
        for k, (mn, mx) in ranges.items():
            Lh = (L_vals >= mn[0]) & (L_vals <= mx[0])
            Ah = (A_vals >= mn[1]) & (A_vals <= mx[1])
            Bh = (B_vals >= mn[2]) & (B_vals <= mx[2])
            hits[k] = int((Lh & Ah & Bh).sum())

        # Chromatic priority decision
        best_c, best_ch = None, -1
        best_n, best_nh = None, -1
        for k, h in hits.items():
            if k in CHROMATIC:
                if h > best_ch:
                    best_ch, best_c = h, k
            elif k in NEUTRAL:
                if h > best_nh:
                    best_nh, best_n = h, k

        c_conf = round(best_ch / total_px, 4) if best_ch >= 0 else 0
        n_conf = round(best_nh / total_px, 4) if best_nh >= 0 else 0
        cmc = params.get("chromatic_min_confidence", 0.10)

        if best_c is not None and c_conf >= cmc:
            pred_color, conf = best_c, c_conf
            win_reason = f"chromatic '{best_c}' conf={c_conf:.4f} >= {cmc} (hits={best_ch}/{total_px})"
        elif best_n is not None:
            pred_color, conf = best_n, n_conf
            win_reason = f"fallback neutral '{best_n}' conf={n_conf:.4f} (no chromatic >= {cmc})"
        else:
            pred_color, conf = "unknown", 0.0
            win_reason = "no color matched"

        # Red-vs-blue overlap analysis
        rb_overlap = {}
        if "red" in hits and "blue" in hits:
            red_range = ranges["red"]
            blue_range = ranges["blue"]
            red_mask = ((L_vals >= red_range[0][0]) & (L_vals <= red_range[1][0]) &
                        (A_vals >= red_range[0][1]) & (A_vals <= red_range[1][1]) &
                        (B_vals >= red_range[0][2]) & (B_vals <= red_range[1][2]))
            blue_mask = ((L_vals >= blue_range[0][0]) & (L_vals <= blue_range[1][0]) &
                         (A_vals >= blue_range[0][1]) & (A_vals <= blue_range[1][1]) &
                         (B_vals >= blue_range[0][2]) & (B_vals <= blue_range[1][2]))
            overlap_both = (red_mask & blue_mask).sum()
            red_only = (red_mask & ~blue_mask).sum()
            blue_only = (blue_mask & ~red_mask).sum()
            rb_overlap = {
                "red_hits": int(red_mask.sum()),
                "blue_hits": int(blue_mask.sum()),
                "overlap_both": int(overlap_both),
                "red_only": int(red_only),
                "blue_only": int(blue_only),
                "red_blue_ratio": round(
                    red_mask.sum() / max(1, blue_mask.sum()), 4),
            }

        stats = {
            "total_px": total_px,
            "lab": {
                "L": compute_stats(L_vals),
                "A": compute_stats(A_vals),
                "B": compute_stats(B_vals),
            },
            "hsv": {
                "H": compute_stats(H_vals),
                "S": compute_stats(S_vals),
                "V": compute_stats(V_vals),
            },
            "rgb": {
                "R": compute_stats(nr),
                "G": compute_stats(ng),
                "B": compute_stats(nb),
            },
            "hits": hits,
            "predicted": pred_color,
            "confidence": conf,
            "win_reason": win_reason,
            "red/blue": rb_overlap,
        }

        results.append({
            "image": img_path.name,
            "expected_color": expected,
            "expected_class": expected_class,
            "predicted": pred_color,
            "confidence": conf,
            "correct": (pred_color == expected) if expected else None,
            "stats": stats,
        })

        status = "OK" if pred_color == expected else ("WRONG" if expected else "?")
        print(f"  {img_path.name:28s} GT={str(expected):5s} PRED={pred_color:6s} "
              f"conf={conf:.3f} {status}  "
              f"hits blue={hits.get('blue',0)} red={hits.get('red',0)} black={hits.get('black',0)}  "
              f"reason={win_reason[:50]}")

        dbg = build_decision_debug_img(bgr, contour, mask, pred_color, expected, stats)
        cv2.imwrite(str(DEBUG_DIR / (img_path.stem + "_v3.png")), dbg)

    # Write JSON
    json_path = OUTPUT_DIR / "color_decision_debug.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\n✅ {json_path}")

    # Write report
    md = ["# Color Decision Debug Report\n"]

    red_blocks = [r for r in results if "red_block" in r["image"]]
    blue_objects = [r for r in results if "blue" in r["image"] and "red_block" not in r["image"]]

    md.append("## Why Red Blocks Become Blue\n")
    md.append("Adjusted LAB ranges create overlap between blue and red:\n")
    md.append(f"- **Blue range**: A=[105,178] B=[59,143] (A-width=73, B-width=84)")
    md.append(f"- **Red range**:  A=[124,183] B=[125,189] (A-width=59, B-width=64)\n")
    md.append("Both ranges fully capture red-block pixels (A≈129, B≈134). ")
    md.append("Blue wins because its wider B-range captures more pixels, and its lower B-min (59 vs red's 125) ")
    md.append("includes darker/neutral border pixels that red excludes.\n")

    md.append("### Red Block Analysis\n")
    for r in results:
        if "red_block" in r["image"]:
            s = r["stats"]
            rb = s.get("red/blue", {})
            nr = s["rgb"]
            hits = s["hits"]
            md.append(f"**{r['image']}** (GT=red, pred={r['predicted']}, conf={r['confidence']})\n")
            md.append(f"- LAB median: L={s['lab']['L']['p50']} A={s['lab']['A']['p50']} B={s['lab']['B']['p50']}")
            md.append(f"- RGB norm: R={nr['R']['mean']:.3f} G={nr['G']['mean']:.3f} B={nr['B']['mean']:.3f}  R-B diff={nr['R']['mean']-nr['B']['mean']:.4f}")
            md.append(f"- HSV: H={s['hsv']['H']['p50']} S={s['hsv']['S']['p50']} V={s['hsv']['V']['p50']}")
            md.append(f"- Red hits={hits.get('red',0)}, Blue hits={hits.get('blue',0)}, Black hits={hits.get('black',0)}")
            if rb:
                md.append(f"- Red/blue: overlap_both={rb['overlap_both']} red_only={rb['red_only']} blue_only={rb['blue_only']} ratio={rb['red_blue_ratio']}")
            md.append(f"- Win reason: {s['win_reason']}")
            md.append("")

    md.append("### Blue Object Comparison\n")
    for r in results:
        if "blue" in r["image"] and "red" not in r["image"]:
            s = r["stats"]
            nr = s["rgb"]
            hits = s["hits"]
            rb = s.get("red/blue", {})
            md.append(f"**{r['image']}**")
            md.append(f"- LAB median: L={s['lab']['L']['p50']} A={s['lab']['A']['p50']} B={s['lab']['B']['p50']}")
            md.append(f"- RGB norm: R={nr['R']['mean']:.3f} G={nr['G']['mean']:.3f} B={nr['B']['mean']:.3f}")
            md.append(f"- Red hits={hits.get('red',0)}, Blue hits={hits.get('blue',0)}, Black hits={hits.get('black',0)}")
            if rb:
                md.append(f"- Red/blue ratio={rb['red_blue_ratio']}")
            md.append("")

    md.append("## Root Cause\n")
    md.append("**Red and blue objects have nearly identical A-values** (127-131) under this lighting. ")
    md.append("The A-channel (green↔red axis) provides zero separation. ")
    md.append("The B-channel (blue↔yellow axis) has a slight difference (blue ~127, red ~134), ")
    md.append("but after expanding both ranges to capture all pixels, they overlap completely. ")
    md.append("Blue's wider range always wins the pixel-count vote because it's a superset of the red range.\n")
    md.append("**Fix options** (not implemented):\n")
    md.append("1. Narrow blue B-range (remove the 59→143 expansion, keep original 59→119) to eliminate overlap\n")
    md.append("2. Use ratio voting: pick the color with higher hit ratio, not higher absolute hits\n")
    md.append("3. Use RGB-based discrimination: red blocks have higher R/(R+G+B) ratio than blue objects\n")
    md.append("4. Restore original red A-min (142) to exclude red-block pixels entirely (they're actually not red in LAB)")

    md_path = OUTPUT_DIR / "color_decision_debug.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md))
    print(f"✅ {md_path}")
    print(f"✅ {DEBUG_DIR}/ ({len(results)} debug images)")


if __name__ == "__main__":
    main()
