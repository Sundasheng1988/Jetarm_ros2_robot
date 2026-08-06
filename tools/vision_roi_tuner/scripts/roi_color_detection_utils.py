#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline ROI color detection helpers.
Mirrors logic in src/app/app/roi_color_detector_node.py (zero ROS2 dependency).
"""

import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

CHROMATIC_COLORS = {"red", "blue", "green", "yellow", "purple", "tennis"}
NEUTRAL_COLORS = {"black", "white"}


def load_lab_ranges(lab_config_path: str) -> Tuple[Dict[str, Tuple], List[str]]:
    p = Path(lab_config_path)
    data = yaml.safe_load(p.read_text())
    cl = data["/**"]["ros__parameters"]["color_range_list"]
    rng = {}
    for name in cl:
        rng[name] = (tuple(cl[name]["min"]), tuple(cl[name]["max"]))
    color_keys = list(cl.keys())
    return rng, color_keys


def dominant_color_key(lab_img, contour, lab_ranges):
    mask = np.zeros(lab_img.shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
    total_pixels = int((mask > 0).sum())
    if total_pixels == 0:
        return "unknown", 0.0, {}
    hits = {}
    best_key, best_hits = None, -1
    for k, (mn, mx) in lab_ranges.items():
        cm = cv2.inRange(lab_img, np.array(mn, np.uint8), np.array(mx, np.uint8))
        h = int((cm & mask).sum() // 255)
        hits[k] = h
        if h > best_hits:
            best_hits, best_key = h, k
    pred_color = best_key or "unknown"
    confidence = round(best_hits / total_pixels, 4) if total_pixels > 0 else 0.0
    return pred_color, confidence, hits


def color_center_score(lab_img, mask, lab_ranges):
    total_px = int((mask > 0).sum())
    if total_px == 0:
        return {}
    L = lab_img[:, :, 0].astype(np.float64)
    A = lab_img[:, :, 1].astype(np.float64)
    B = lab_img[:, :, 2].astype(np.float64)
    scores = {}
    for k, (mn, mx) in lab_ranges.items():
        in_range = ((L >= mn[0]) & (L <= mx[0]) & (A >= mn[1]) & (A <= mx[1]) &
                    (B >= mn[2]) & (B <= mx[2]) & (mask > 0))
        hit_count = int(in_range.sum())
        hit_ratio = hit_count / total_px if total_px > 0 else 0.0
        if hit_count == 0:
            scores[k] = 0.0
            continue
        Lc, Ac, Bc = (mn[0]+mx[0])/2.0, (mn[1]+mx[1])/2.0, (mn[2]+mx[2])/2.0
        Lw, Aw, Bw = max((mx[0]-mn[0])/2.0, 1.0), max((mx[1]-mn[1])/2.0, 1.0), max((mx[2]-mn[2])/2.0, 1.0)
        dL, dA, dB = (L[in_range]-Lc)/Lw, (A[in_range]-Ac)/Aw, (B[in_range]-Bc)/Bw
        d = np.sqrt(dL**2 + dA**2 + dB**2)
        pixel_scores = np.maximum(0.0, 1.0-d)
        center_score = float(pixel_scores.mean())
        scores[k] = round(hit_ratio*0.4 + center_score*0.6, 6)
    return scores


def dominant_color_key_with_priority(lab_img, mask, lab_ranges,
                                     chromatic_min_confidence=0.10,
                                     ambiguity_margin=0.15):
    total_pixels = int((mask > 0).sum())
    if total_pixels == 0:
        return "unknown", 0.0, {}
    hits = {}
    for k, (mn, mx) in lab_ranges.items():
        cm = cv2.inRange(lab_img, np.array(mn, np.uint8), np.array(mx, np.uint8))
        h = int((cm & mask).sum() // 255)
        hits[k] = h
    color_scores = color_center_score(lab_img, mask, lab_ranges)
    chromatic_candidates = []
    for k in CHROMATIC_COLORS:
        if k in color_scores and color_scores[k] > 0:
            chromatic_candidates.append((k, hits.get(k, 0), color_scores[k]))
    chromatic_candidates.sort(key=lambda x: x[2], reverse=True)
    if chromatic_candidates:
        best_color, best_hits, best_score = chromatic_candidates[0]
        second_score = chromatic_candidates[1][2] if len(chromatic_candidates) >= 2 else 0.0
        margin = best_score - second_score
        if best_score >= chromatic_min_confidence:
            if margin < ambiguity_margin:
                confidence = round(max(margin, 0.01), 4)
            else:
                confidence = round(best_score, 4)
            return best_color, confidence, hits
    neutral_candidates = []
    for k in NEUTRAL_COLORS:
        if k in color_scores and color_scores[k] > 0:
            neutral_candidates.append((k, hits.get(k, 0), color_scores[k]))
    neutral_candidates.sort(key=lambda x: x[2], reverse=True)
    if neutral_candidates:
        best_color, _, best_score = neutral_candidates[0]
        second_score = neutral_candidates[1][2] if len(neutral_candidates) >= 2 else 0.0
        confidence = round(max(best_score - second_score, 0.01), 4)
        return best_color, confidence, hits
    return "unknown", 0.0, hits


def detect_object_contour(gray_img, min_area=500):
    blurred = cv2.GaussianBlur(gray_img, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    largest = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    return largest


def erode_contour_mask(img_shape, contour, kernel_size=5):
    mask = np.zeros(img_shape, np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    return cv2.erode(mask, kernel, iterations=1)


def erode_contour_mask_v2(img_shape, contour, kernel_size=5, iterations=1):
    mask = np.zeros(img_shape, np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    return cv2.erode(mask, kernel, iterations=iterations)


def apply_luminance_thresholds(mask, lab_img, ignore_l_min=None, ignore_l_max=None):
    L = lab_img[:, :, 0]
    mask = mask.copy()
    if ignore_l_min is not None:
        mask[L < ignore_l_min] = 0
    if ignore_l_max is not None:
        mask[L > ignore_l_max] = 0
    return mask


def estimate_color(bgr_img, lab_ranges, min_area=500, erode_size=5):
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    contour = detect_object_contour(gray, min_area=min_area)
    if contour is None:
        return "unknown", 0.0, {}, None
    lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
    pred_color, confidence, hits = dominant_color_key(lab, contour, lab_ranges)
    return pred_color, confidence, hits, contour


def estimate_color_with_thresholds(bgr_img, lab_ranges, min_area=500, erode_size=5,
                                   erode_iterations=1, ignore_l_min=None,
                                   ignore_l_max=None, min_confidence=0.0,
                                   chromatic_min_confidence=0.10):
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    contour = detect_object_contour(gray, min_area=min_area)
    if contour is None:
        return "unknown", 0.0, {}, None
    mask = erode_contour_mask_v2(gray.shape, contour, erode_size, erode_iterations)
    lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
    mask = apply_luminance_thresholds(mask, lab, ignore_l_min, ignore_l_max)
    pred_color, confidence, hits = dominant_color_key_with_priority(
        lab, mask, lab_ranges, chromatic_min_confidence)
    if confidence < min_confidence:
        return "unknown", confidence, hits, contour
    return pred_color, confidence, hits, contour


def estimate_color_with_otsu_chroma(bgr_img, lab_ranges, min_area=500, erode_size=5,
                                    use_otsu=False, use_chroma=False, chroma_threshold=5.0,
                                    chromatic_min_confidence=0.10, min_confidence=0.0,
                                    lab_range_overrides=None):
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    contour = detect_object_contour(gray, min_area=min_area)
    if contour is None:
        return "unknown", 0.0, {}, None
    mask = erode_contour_mask(gray.shape, contour, kernel_size=erode_size)
    lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
    if use_otsu:
        vals = gray[mask > 0].astype(np.uint8)
        if len(vals) > 0:
            otsu_val, _ = cv2.threshold(vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            mask[(mask > 0) & (gray < otsu_val)] = 0
    if use_chroma and (mask > 0).sum() > 0:
        Lab = lab[:, :, 1].astype(float)
        Bab = lab[:, :, 2].astype(float)
        C = np.sqrt((Lab - 128.0) ** 2 + (Bab - 128.0) ** 2)
        mask[(mask > 0) & (C < chroma_threshold)] = 0
    ranges = lab_ranges
    if lab_range_overrides:
        ranges = dict(lab_ranges)
        for color, ch_overrides in lab_range_overrides.items():
            if color in ranges:
                om, ox = ranges[color]
                nm, nx = list(om), list(ox)
                for ch, (lo, hi) in ch_overrides.items():
                    idx = {"L": 0, "A": 1, "B": 2}[ch]
                    nm[idx], nx[idx] = lo, hi
                ranges[color] = (tuple(nm), tuple(nx))
    pred_color, confidence, hits = dominant_color_key_with_priority(
        lab, mask, ranges, chromatic_min_confidence)
    if confidence < min_confidence:
        return "unknown", confidence, hits, contour
    return pred_color, confidence, hits, contour


def parse_filename_label(filename):
    stem = Path(filename).stem
    m = re.match(r"^(?P<color>[a-z]+)_(?P<class>[a-z]+)_\d+$", stem)
    if not m:
        raise ValueError(f"cannot parse label from filename: {filename}")
    return m.group("color"), m.group("class")


def build_debug_overlay(img, contour, expected_color, expected_class, pred_color, confidence, all_hits):
    vis = img.copy()
    if contour is not None:
        cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)
    correct = (pred_color == expected_color)
    status = "OK" if correct else "WRONG"
    text_color = (0, 255, 0) if correct else (0, 0, 255)
    cv2.putText(vis, f"GT:  {expected_color} {expected_class}", (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(vis, f"PRED: {pred_color} ({confidence:.2f})  {status}", (12, 54),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, text_color, 2)
    bar_base_y = vis.shape[0] - 6
    bar_max_h = 80
    max_hits = max(all_hits.values()) if all_hits else 1
    bar_w, gap = 28, 4
    keys_sorted = sorted(all_hits.keys())
    x0 = vis.shape[1] - (bar_w + gap) * len(keys_sorted) - 4
    for i, k in enumerate(keys_sorted):
        h = int((all_hits[k] / max_hits) * bar_max_h) if max_hits > 0 else 0
        x = x0 + i * (bar_w + gap)
        y1 = bar_base_y - h
        bar_color = (100, 200, 100) if k == pred_color else (100, 100, 100)
        cv2.rectangle(vis, (x, y1), (x + bar_w, bar_base_y), bar_color, -1)
        cv2.putText(vis, k[:4], (x, bar_base_y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    return vis
