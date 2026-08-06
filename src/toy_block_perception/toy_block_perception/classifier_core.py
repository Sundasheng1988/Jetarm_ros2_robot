#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure OpenCV core for the ROS 2 toy-block detector.

Current validated scope:
- one colored toy block in the fixed work area
- 640x480 RGB image
- diffuse lighting
- known closed set: yellow, green, cyan, blue, purple

This module contains no ROS imports and performs no file I/O.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    expected_width: int = 640
    expected_height: int = 480
    gate_margin_px: int = 8

    locate_x_min_frac: float = 0.10
    locate_x_max_frac: float = 0.90
    locate_y_min_frac: float = 0.12
    locate_y_max_frac: float = 0.70
    locate_center_x_frac: float = 0.50
    locate_center_y_frac: float = 0.37

    locate_min_saturation: int = 45
    locate_min_chroma: float = 8.0
    locate_min_area: int = 300
    locate_max_area: int = 30000
    locate_min_side: int = 20
    locate_max_side: int = 250

    core_edge_inset_frac: float = 0.15
    core_reject_l_le: int = 60
    core_highlight_l_ge: int = 230
    core_highlight_chroma_max: float = 14.0
    core_chroma_floor: float = 12.0
    core_local_window: int = 5
    core_local_std_max: float = 18.0
    core_min_pixels: int = 60

    hsv_h_band_degrees: int = 25
    hsv_s_floor: float = 20.0
    hsv_s_p10_factor: float = 0.65

    outer_margin_ratio: float = 0.10
    outer_margin_extra_px: int = 6


COLOR_PROFILES: dict[str, dict[str, float]] = {
    "yellow": {"h": 21.289, "a": 135.0, "b": 185.2},
    "green": {"h": 68.509, "a": 99.6, "b": 147.0},
    "cyan": {"h": 98.132, "a": 116.0, "b": 107.3},
    "blue": {"h": 105.712, "a": 129.2, "b": 101.2},
    "purple": {"h": 138.289, "a": 150.2, "b": 105.4},
}

COLOR_NAMES_CN = {
    "yellow": "黄色",
    "green": "绿色",
    "cyan": "青色",
    "blue": "蓝色",
    "purple": "紫色",
    "unknown": "未知",
}

CLASSIFY_MAX_HUE_DISTANCE = 6.0
CLASSIFY_MAX_LAB_DISTANCE = 22.0
CLASSIFY_MIN_HUE_MARGIN = 1.5
CYAN_BLUE_A_THRESHOLD = 123.5
CYAN_BLUE_B_THRESHOLD = 104.5


@dataclass(frozen=True)
class LocateResult:
    bbox_xywh: tuple[int, int, int, int]
    candidate_mask: np.ndarray
    selected_component_mask: np.ndarray
    diagnostics: dict[str, Any]


def validate_image(image: np.ndarray, config: DetectorConfig) -> None:
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected a BGR image with shape HxWx3")

    height, width = image.shape[:2]
    if (width, height) != (config.expected_width, config.expected_height):
        raise ValueError(
            f"Expected {config.expected_width}x{config.expected_height}, "
            f"got {width}x{height}"
        )


def clip_xyxy(
    shape: tuple[int, ...],
    xyxy: tuple[int, int, int, int],
) -> tuple[int, int, int, int]:
    height, width = shape[:2]
    x0, y0, x1, y1 = map(int, xyxy)

    x0 = max(0, min(width, x0))
    x1 = max(0, min(width, x1))
    y0 = max(0, min(height, y0))
    y1 = max(0, min(height, y1))

    if x1 <= x0 or y1 <= y0:
        raise ValueError(f"Invalid clipped rectangle: {(x0, y0, x1, y1)}")

    return x0, y0, x1, y1


def rect_mask_xyxy(
    shape: tuple[int, ...],
    xyxy: tuple[int, int, int, int],
) -> np.ndarray:
    x0, y0, x1, y1 = clip_xyxy(shape, xyxy)
    mask = np.zeros(shape[:2], dtype=np.uint8)
    mask[y0:y1, x0:x1] = 255
    return mask


def expand_bbox_xywh(
    shape: tuple[int, ...],
    bbox_xywh: tuple[int, int, int, int],
    margin: int,
) -> tuple[int, int, int, int]:
    x, y, width, height = map(int, bbox_xywh)
    return clip_xyxy(
        shape,
        (
            x - margin,
            y - margin,
            x + width + margin,
            y + height + margin,
        ),
    )


def local_lab_std(lab: np.ndarray, window: int) -> np.ndarray:
    if window <= 0 or window % 2 == 0:
        raise ValueError("local LAB std window must be positive and odd")

    variance_sum = np.zeros(lab.shape[:2], dtype=np.float64)
    for channel_index in range(3):
        channel = lab[:, :, channel_index].astype(np.float64)
        mean = cv2.blur(channel, (window, window))
        mean_square = cv2.blur(channel * channel, (window, window))
        variance_sum += np.maximum(mean_square - mean * mean, 0.0)

    return np.sqrt(variance_sum)


def automatic_locate(
    image: np.ndarray,
    config: DetectorConfig,
) -> LocateResult:
    """Locate the most plausible colored component near the work-area center."""
    height, width = image.shape[:2]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    saturation = hsv[:, :, 1]
    a_channel = lab[:, :, 1].astype(np.float64)
    b_channel = lab[:, :, 2].astype(np.float64)
    chroma = np.sqrt(
        (a_channel - 128.0) ** 2
        + (b_channel - 128.0) ** 2
    )

    search_x0 = int(round(width * config.locate_x_min_frac))
    search_x1 = int(round(width * config.locate_x_max_frac))
    search_y0 = int(round(height * config.locate_y_min_frac))
    search_y1 = int(round(height * config.locate_y_max_frac))

    search_region = np.zeros((height, width), dtype=np.uint8)
    search_region[search_y0:search_y1, search_x0:search_x1] = 255

    region_saturation = saturation[search_region > 0]
    if region_saturation.size == 0:
        raise RuntimeError("Automatic-locate search region is empty")

    otsu_threshold, _ = cv2.threshold(
        region_saturation.reshape(-1, 1),
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU,
    )
    saturation_threshold = max(
        config.locate_min_saturation,
        int(round(float(otsu_threshold))),
    )

    raw_candidate = (
        (saturation >= saturation_threshold)
        & (chroma >= config.locate_min_chroma)
        & (search_region > 0)
    ).astype(np.uint8) * 255

    candidate = cv2.morphologyEx(
        raw_candidate,
        cv2.MORPH_CLOSE,
        np.ones((5, 5), dtype=np.uint8),
        iterations=1,
    )
    candidate = cv2.morphologyEx(
        candidate,
        cv2.MORPH_OPEN,
        np.ones((3, 3), dtype=np.uint8),
        iterations=1,
    )

    component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        (candidate > 0).astype(np.uint8),
        connectivity=8,
    )

    expected_cx = width * config.locate_center_x_frac
    expected_cy = height * config.locate_center_y_frac
    ranked: list[dict[str, Any]] = []

    for label_id in range(1, component_count):
        x, y, box_width, box_height, area = map(int, stats[label_id])
        centroid_x, centroid_y = map(float, centroids[label_id])

        if area < config.locate_min_area or area > config.locate_max_area:
            continue
        if box_width < config.locate_min_side or box_height < config.locate_min_side:
            continue
        if box_width > config.locate_max_side or box_height > config.locate_max_side:
            continue

        fill_ratio = area / float(max(1, box_width * box_height))
        aspect_ratio = max(
            box_width / float(max(1, box_height)),
            box_height / float(max(1, box_width)),
        )
        normalized_distance = math.hypot(
            (centroid_x - expected_cx) / (width * 0.40),
            (centroid_y - expected_cy) / (height * 0.30),
        )
        touches_search_boundary = bool(
            x <= search_x0 + 2
            or y <= search_y0 + 2
            or x + box_width >= search_x1 - 2
            or y + box_height >= search_y1 - 2
        )

        center_score = 3.5 * (1.0 - min(normalized_distance, 1.5) / 1.5)
        area_score = 1.2 * min(area / 8000.0, 1.5)
        fill_score = 0.7 * fill_ratio
        aspect_penalty = max(0.0, aspect_ratio - 2.0)
        boundary_penalty = 2.0 if touches_search_boundary else 0.0

        score = (
            center_score
            + area_score
            + fill_score
            - aspect_penalty
            - boundary_penalty
        )

        ranked.append(
            {
                "label_id": label_id,
                "score": float(score),
                "bbox_xywh": [x, y, box_width, box_height],
                "area": area,
                "centroid": [centroid_x, centroid_y],
                "fill_ratio": float(fill_ratio),
                "aspect_ratio": float(aspect_ratio),
                "normalized_distance": float(normalized_distance),
                "touches_search_boundary": touches_search_boundary,
            }
        )

    if not ranked:
        raise RuntimeError("Automatic locate found no plausible colored component")

    ranked.sort(key=lambda item: item["score"], reverse=True)
    best = ranked[0]
    second_score = ranked[1]["score"] if len(ranked) > 1 else None
    best_label = int(best["label_id"])

    return LocateResult(
        bbox_xywh=tuple(map(int, best["bbox_xywh"])),
        candidate_mask=candidate,
        selected_component_mask=(labels == best_label).astype(np.uint8) * 255,
        diagnostics={
            "mode": "auto",
            "saturation_otsu": float(otsu_threshold),
            "saturation_threshold": saturation_threshold,
            "chroma_threshold": config.locate_min_chroma,
            "search_region_xyxy_exclusive": [
                search_x0, search_y0, search_x1, search_y1
            ],
            "expected_center": [round(expected_cx, 2), round(expected_cy, 2)],
            "plausible_component_count": len(ranked),
            "best_score": round(float(best["score"]), 4),
            "second_score": (
                round(float(second_score), 4)
                if second_score is not None else None
            ),
            "score_margin": (
                round(float(best["score"] - second_score), 4)
                if second_score is not None else None
            ),
            "selected_area": int(best["area"]),
            "selected_fill_ratio": round(float(best["fill_ratio"]), 4),
            "selected_aspect_ratio": round(float(best["aspect_ratio"]), 4),
            "selected_centroid": [
                round(float(best["centroid"][0]), 2),
                round(float(best["centroid"][1]), 2),
            ],
            "selected_normalized_distance": round(
                float(best["normalized_distance"]), 4
            ),
            "touches_search_boundary": bool(best["touches_search_boundary"]),
        },
    )


def derive_core_seed(
    image: np.ndarray,
    bbox_xywh: tuple[int, int, int, int],
    config: DetectorConfig,
) -> tuple[np.ndarray, dict[str, Any]]:
    height, width = image.shape[:2]
    x, y, box_width, box_height = map(int, bbox_xywh)

    x0 = max(0, min(width, x))
    y0 = max(0, min(height, y))
    x1 = max(0, min(width, x + box_width))
    y1 = max(0, min(height, y + box_height))

    inset_x = max(1, int(round((x1 - x0) * config.core_edge_inset_frac)))
    inset_y = max(1, int(round((y1 - y0) * config.core_edge_inset_frac)))

    if x0 + inset_x >= x1 - inset_x or y0 + inset_y >= y1 - inset_y:
        raise RuntimeError("Core-seed inset removed the entire bbox")

    inner = np.zeros((height, width), dtype=bool)
    inner[y0 + inset_y:y1 - inset_y, x0 + inset_x:x1 - inset_x] = True

    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    lightness = lab[:, :, 0].astype(np.float64)
    a_channel = lab[:, :, 1].astype(np.float64)
    b_channel = lab[:, :, 2].astype(np.float64)
    chroma = np.sqrt(
        (a_channel - 128.0) ** 2
        + (b_channel - 128.0) ** 2
    )
    local_instability = local_lab_std(lab, config.core_local_window)

    valid = (
        inner
        & ~(lightness <= config.core_reject_l_le)
        & ~(
            (lightness >= config.core_highlight_l_ge)
            & (chroma <= config.core_highlight_chroma_max)
        )
        & ~(chroma < config.core_chroma_floor)
        & ~(local_instability > config.core_local_std_max)
    )

    raw_seed = valid.astype(np.uint8) * 255
    opened_seed = cv2.morphologyEx(
        raw_seed,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )

    raw_count = int(np.count_nonzero(raw_seed))
    opened_count = int(np.count_nonzero(opened_seed))

    if opened_count >= config.core_min_pixels:
        seed = opened_seed
        mode = "morph_open"
    elif raw_count >= config.core_min_pixels:
        seed = raw_seed
        mode = "raw_valid_pixels"
    else:
        raise RuntimeError(
            f"Core seed too small: opened={opened_count}, raw={raw_count}"
        )

    return seed, {
        "seed_mode": mode,
        "seed_pixel_count": int(np.count_nonzero(seed)),
        "raw_seed_pixel_count": raw_count,
        "opened_seed_pixel_count": opened_count,
        "bbox_inset_px": [inset_x, inset_y],
    }


def circular_hue_center(hue_values: np.ndarray) -> float:
    values = hue_values.astype(np.float64)
    theta = values * (math.pi / 90.0)
    angle = math.atan2(
        float(np.mean(np.sin(theta))),
        float(np.mean(np.cos(theta))),
    )
    if angle < 0:
        angle += 2.0 * math.pi
    return angle * (90.0 / math.pi)


def circular_hue_distance(value: float, center: float) -> float:
    return abs(((float(value) - float(center) + 90.0) % 180.0) - 90.0)


def percentile_description(values: np.ndarray) -> dict[str, float]:
    p10, median, p90 = np.percentile(values.astype(np.float64), [10, 50, 90])
    return {
        "p10": round(float(p10), 3),
        "median": round(float(median), 3),
        "p90": round(float(p90), 3),
    }


def describe_core_colors(
    image: np.ndarray,
    core_seed: np.ndarray,
) -> dict[str, Any]:
    seed = core_seed > 0
    if int(np.count_nonzero(seed)) == 0:
        raise RuntimeError("Cannot describe an empty core seed")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    return {
        "pixel_count": int(np.count_nonzero(seed)),
        "hue_circular_center_opencv": round(
            circular_hue_center(hsv[:, :, 0][seed]), 3
        ),
        "H_opencv": percentile_description(hsv[:, :, 0][seed]),
        "S": percentile_description(hsv[:, :, 1][seed]),
        "V": percentile_description(hsv[:, :, 2][seed]),
        "L_opencv": percentile_description(lab[:, :, 0][seed]),
        "a_opencv": percentile_description(lab[:, :, 1][seed]),
        "b_opencv": percentile_description(lab[:, :, 2][seed]),
    }


def classify_block_color(color_statistics: dict[str, Any]) -> dict[str, Any]:
    hue = float(color_statistics["hue_circular_center_opencv"])
    a_median = float(color_statistics["a_opencv"]["median"])
    b_median = float(color_statistics["b_opencv"]["median"])

    distances: list[dict[str, Any]] = []
    for color, profile in COLOR_PROFILES.items():
        hue_distance = circular_hue_distance(hue, profile["h"])
        lab_distance = math.hypot(
            a_median - profile["a"],
            b_median - profile["b"],
        )
        combined_score = math.sqrt(
            (hue_distance / 4.0) ** 2
            + (lab_distance / 14.0) ** 2
        )
        distances.append({
            "color": color,
            "hue_distance": round(hue_distance, 4),
            "lab_ab_distance": round(lab_distance, 4),
            "combined_score": round(combined_score, 4),
        })

    hue_ranked = sorted(distances, key=lambda item: item["hue_distance"])
    nearest, runner_up = hue_ranked[0], hue_ranked[1]
    predicted = str(nearest["color"])
    hue_margin = (
        float(runner_up["hue_distance"])
        - float(nearest["hue_distance"])
    )
    nearest_lab_distance = float(nearest["lab_ab_distance"])
    reasons: list[str] = []

    if float(nearest["hue_distance"]) > CLASSIFY_MAX_HUE_DISTANCE:
        reasons.append("nearest_hue_distance")
    if nearest_lab_distance > CLASSIFY_MAX_LAB_DISTANCE:
        reasons.append("nearest_lab_ab_distance")
    if hue_margin < CLASSIFY_MIN_HUE_MARGIN:
        reasons.append("hue_margin")

    if predicted in {"cyan", "blue"}:
        if (
            a_median < CYAN_BLUE_A_THRESHOLD
            and b_median >= CYAN_BLUE_B_THRESHOLD
        ):
            lab_label = "cyan"
        elif (
            a_median >= CYAN_BLUE_A_THRESHOLD
            and b_median < CYAN_BLUE_B_THRESHOLD
        ):
            lab_label = "blue"
        else:
            lab_label = "unknown"
            reasons.append("cyan_blue_lab_ambiguous")

        if lab_label not in {"unknown", predicted}:
            reasons.append("cyan_blue_hue_lab_disagree")
    else:
        lab_label = "not_applicable"

    final_label = predicted if not reasons else "unknown"
    status = "CLASSIFIED" if not reasons else "UNKNOWN"

    hue_confidence = max(
        0.0,
        1.0 - float(nearest["hue_distance"]) / CLASSIFY_MAX_HUE_DISTANCE,
    )
    lab_confidence = max(
        0.0,
        1.0 - nearest_lab_distance / CLASSIFY_MAX_LAB_DISTANCE,
    )
    margin_confidence = min(1.0, max(0.0, hue_margin / 8.0))
    confidence = (
        0.50 * hue_confidence
        + 0.25 * lab_confidence
        + 0.25 * margin_confidence
    )

    return {
        "status": status,
        "predicted_color": final_label,
        "predicted_color_cn": COLOR_NAMES_CN[final_label],
        "hue_candidate": predicted,
        "confidence": round(confidence, 4),
        "features": {
            "hue_circular_center_opencv": round(hue, 3),
            "a_median_opencv": round(a_median, 3),
            "b_median_opencv": round(b_median, 3),
        },
        "nearest_hue_distance": round(float(nearest["hue_distance"]), 4),
        "second_hue_color": str(runner_up["color"]),
        "second_hue_distance": round(float(runner_up["hue_distance"]), 4),
        "hue_margin": round(hue_margin, 4),
        "nearest_lab_ab_distance": round(nearest_lab_distance, 4),
        "cyan_blue_lab_label": lab_label,
        "reasons": reasons,
    }


def select_max_core_overlap(
    candidate: np.ndarray,
    core_seed: np.ndarray,
    allowed_region_mask: np.ndarray,
) -> tuple[np.ndarray | None, int]:
    restricted = (
        (candidate > 0)
        & (allowed_region_mask > 0)
    ).astype(np.uint8)

    component_count, labels = cv2.connectedComponents(
        restricted,
        connectivity=8,
    )

    seed = core_seed > 0
    best_label: int | None = None
    best_overlap = 0

    for label_id in range(1, component_count):
        overlap = int(np.count_nonzero((labels == label_id) & seed))
        if overlap > best_overlap:
            best_overlap = overlap
            best_label = label_id

    if best_label is None or best_overlap <= 0:
        return None, 0

    return (labels == best_label).astype(np.uint8) * 255, best_overlap


def fill_holes_after_selection(mask: np.ndarray) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        binary,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    filled = np.zeros_like(binary)
    if contours:
        cv2.drawContours(filled, contours, -1, 255, thickness=-1)
    return filled


def segment_hsv(
    image: np.ndarray,
    core_seed: np.ndarray,
    outer_roi_mask: np.ndarray,
    object_gate_mask: np.ndarray,
    config: DetectorConfig,
) -> tuple[np.ndarray, dict[str, Any]]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0].astype(np.int16)
    saturation = hsv[:, :, 1].astype(np.float64)
    seed = core_seed > 0

    hue_center = circular_hue_center(hue[seed])
    hue_distance = np.abs(
        ((hue.astype(np.float64) - hue_center + 90.0) % 180.0) - 90.0
    )
    seed_saturation_p10 = float(np.percentile(saturation[seed], 10))
    saturation_min = max(
        config.hsv_s_floor,
        seed_saturation_p10 * config.hsv_s_p10_factor,
    )
    h_band = int(math.ceil(config.hsv_h_band_degrees / 2.0))

    allowed_region = (
        (outer_roi_mask > 0)
        & (object_gate_mask > 0)
    )

    candidate = (
        (hue_distance <= h_band)
        & (saturation >= saturation_min)
        & allowed_region
    ).astype(np.uint8) * 255

    selected, core_overlap_px = select_max_core_overlap(
        candidate,
        core_seed,
        allowed_region.astype(np.uint8) * 255,
    )
    if selected is None:
        raise RuntimeError("HSV candidate has no core-seed-overlapping component")

    return fill_holes_after_selection(selected), {
        "status": "ok",
        "hue_center_opencv": round(hue_center, 3),
        "hue_band_opencv": h_band,
        "seed_saturation_p10": round(seed_saturation_p10, 3),
        "saturation_min": round(saturation_min, 3),
        "core_overlap_px_before_fill": core_overlap_px,
    }


def object_gate_edge_sides(
    mask: np.ndarray,
    object_gate_mask: np.ndarray,
) -> list[str]:
    binary = mask > 0
    ys, xs = np.where(object_gate_mask > 0)
    if xs.size == 0 or ys.size == 0:
        return []

    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    sides: list[str] = []

    if np.any(binary[y0:y1 + 1, x0]):
        sides.append("left")
    if np.any(binary[y0:y1 + 1, x1]):
        sides.append("right")
    if np.any(binary[y0, x0:x1 + 1]):
        sides.append("top")
    if np.any(binary[y1, x0:x1 + 1]):
        sides.append("bottom")

    return sides


def compute_metrics(
    mask: np.ndarray,
    core_seed: np.ndarray,
    outer_roi_mask: np.ndarray,
    object_gate_mask: np.ndarray,
) -> dict[str, Any]:
    binary = (mask > 0).astype(np.uint8)
    pixel_count = int(np.count_nonzero(binary))
    if pixel_count == 0:
        raise RuntimeError("Final mask is empty")

    contours, _ = cv2.findContours(
        binary * 255,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        raise RuntimeError("Final mask has no contour")

    all_points = np.vstack(contours)
    contour_area = float(sum(cv2.contourArea(c) for c in contours))
    hull = cv2.convexHull(all_points)
    hull_area = float(cv2.contourArea(hull))
    x, y, width, height = cv2.boundingRect(all_points)
    bbox_area = float(width * height)

    moments = cv2.moments(binary)
    centroid_x = float(moments["m10"] / moments["m00"])
    centroid_y = float(moments["m01"] / moments["m00"])

    core_count = int(np.count_nonzero(core_seed))
    core_overlap_px = int(
        np.count_nonzero((binary > 0) & (core_seed > 0))
    )
    gate_sides = object_gate_edge_sides(binary, object_gate_mask)

    rect = cv2.minAreaRect(all_points.astype(np.float32))
    (_, _), (rect_width, rect_height), raw_angle = rect
    angle_deg = float(raw_angle)
    if rect_width < rect_height:
        angle_deg += 90.0
    while angle_deg >= 90.0:
        angle_deg -= 180.0
    while angle_deg < -90.0:
        angle_deg += 180.0

    return {
        "status": "ok",
        "pixel_count": pixel_count,
        "bbox_xywh": [int(x), int(y), int(width), int(height)],
        "centroid": [round(centroid_x, 2), round(centroid_y, 2)],
        "angle_deg": round(angle_deg, 2),
        "core_overlap_fraction": round(
            core_overlap_px / max(1, core_count), 4
        ),
        "solidity": round(
            contour_area / hull_area if hull_area > 0 else 0.0, 4
        ),
        "extent": round(
            contour_area / bbox_area if bbox_area > 0 else 0.0, 4
        ),
        "touches_image_edge": bool(
            x <= 2
            or y <= 2
            or x + width >= binary.shape[1] - 2
            or y + height >= binary.shape[0] - 2
        ),
        "touches_object_gate_edge": bool(gate_sides),
        "object_gate_edge_sides": gate_sides,
    }


def automatic_review_status(
    locate_diagnostics: dict[str, Any],
    metrics: dict[str, Any],
    classification: dict[str, Any],
) -> tuple[str, list[str]]:
    reasons: list[str] = []

    if metrics["core_overlap_fraction"] < 0.95:
        reasons.append("core_overlap_fraction")
    if metrics["solidity"] < 0.85:
        reasons.append("solidity")
    if metrics["pixel_count"] < 300:
        reasons.append("mask_pixel_count")
    if metrics["touches_image_edge"]:
        reasons.append("touches_image_edge")
    if metrics["touches_object_gate_edge"]:
        reasons.append(
            "touches_object_gate_edge:"
            + ",".join(metrics["object_gate_edge_sides"])
        )
    if locate_diagnostics["touches_search_boundary"]:
        reasons.append("locate_touches_search_boundary")
    if float(locate_diagnostics["best_score"]) < 3.0:
        reasons.append("locate_best_score")
    if classification["status"] != "CLASSIFIED":
        reasons.append(
            "classification_unknown:"
            + "|".join(classification["reasons"])
        )

    return ("REVIEW", reasons) if reasons else ("AUTO_OK", [])


def tint_mask(
    image: np.ndarray,
    mask: np.ndarray,
    color_bgr: tuple[int, int, int],
) -> np.ndarray:
    tint = np.full_like(image, color_bgr)
    blended = cv2.addWeighted(image, 0.60, tint, 0.40, 0)
    return np.where(mask[:, :, None] > 0, blended, image.copy())


def draw_overlay(
    image: np.ndarray,
    mask: np.ndarray,
    object_gate_xyxy: tuple[int, int, int, int],
    result: dict[str, Any],
    stable: bool = False,
    stable_count: int = 0,
) -> np.ndarray:
    classification = result["classification"]
    review_status = result["review_status"]
    metrics = result["metrics"]

    class_colors_bgr = {
        "yellow": (0, 220, 220),
        "green": (0, 180, 0),
        "cyan": (220, 220, 0),
        "blue": (220, 0, 0),
        "purple": (200, 0, 200),
        "unknown": (128, 128, 128),
    }
    predicted = str(classification["predicted_color"])
    class_color = class_colors_bgr.get(predicted, (128, 128, 128))

    overlay = tint_mask(image, mask, class_color)
    contours, _ = cv2.findContours(
        (mask > 0).astype(np.uint8) * 255,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    cv2.drawContours(overlay, contours, -1, class_color, 2)

    x, y, width, height = metrics["bbox_xywh"]
    cv2.rectangle(
        overlay,
        (x, y),
        (x + width - 1, y + height - 1),
        (0, 255, 255),
        2,
    )
    cx, cy = map(int, map(round, metrics["centroid"]))
    cv2.drawMarker(
        overlay,
        (cx, cy),
        (0, 0, 255),
        cv2.MARKER_CROSS,
        16,
        2,
    )

    gx0, gy0, gx1, gy1 = object_gate_xyxy
    cv2.rectangle(
        overlay,
        (gx0, gy0),
        (gx1 - 1, gy1 - 1),
        (255, 255, 0),
        1,
    )

    cv2.rectangle(
        overlay,
        (0, 0),
        (overlay.shape[1] - 1, 62),
        (0, 0, 0),
        thickness=-1,
    )

    confidence = float(classification["confidence"])
    features = classification["features"]
    stable_text = f"STABLE {stable_count}" if stable else f"SEEN {stable_count}"

    cv2.putText(
        overlay,
        (
            f"{predicted.upper()} | {classification['status']} | "
            f"{review_status} | {stable_text}"
        ),
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        class_color if predicted != "unknown" else (220, 220, 220),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        overlay,
        (
            f"conf={confidence:.2f} H={features['hue_circular_center_opencv']} "
            f"a={features['a_median_opencv']} b={features['b_median_opencv']} "
            f"angle={metrics['angle_deg']:.1f}"
        ),
        (8, 49),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    return overlay


def draw_error_overlay(image: np.ndarray, message: str) -> np.ndarray:
    overlay = image.copy()
    cv2.rectangle(
        overlay,
        (0, 0),
        (overlay.shape[1] - 1, 52),
        (0, 0, 0),
        thickness=-1,
    )
    cv2.putText(
        overlay,
        "NO VALID BLOCK",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        overlay,
        message[:85],
        (8, 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return overlay


def detect_block(
    image: np.ndarray,
    config: DetectorConfig,
) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    """Return result dictionary, uint8 mask and BGR overlay."""
    validate_image(image, config)

    locate = automatic_locate(image, config)
    detected_bbox = locate.bbox_xywh
    _, _, box_width, box_height = detected_bbox

    outer_margin = (
        int(round(config.outer_margin_ratio * min(box_width, box_height)))
        + config.outer_margin_extra_px
    )
    outer_roi_xyxy = expand_bbox_xywh(
        image.shape, detected_bbox, outer_margin
    )
    object_gate_xyxy = expand_bbox_xywh(
        image.shape, detected_bbox, config.gate_margin_px
    )

    outer_roi_mask = rect_mask_xyxy(image.shape, outer_roi_xyxy)
    object_gate_mask = rect_mask_xyxy(image.shape, object_gate_xyxy)

    core_seed, core_seed_diagnostics = derive_core_seed(
        image, detected_bbox, config
    )
    color_statistics = describe_core_colors(image, core_seed)
    classification = classify_block_color(color_statistics)

    hsv_mask, hsv_diagnostics = segment_hsv(
        image,
        core_seed,
        outer_roi_mask,
        object_gate_mask,
        config,
    )
    metrics = compute_metrics(
        hsv_mask,
        core_seed,
        outer_roi_mask,
        object_gate_mask,
    )
    review_status, review_reasons = automatic_review_status(
        locate.diagnostics,
        metrics,
        classification,
    )

    result = {
        "detected_bbox_xywh": list(detected_bbox),
        "outer_roi_xyxy_exclusive": list(outer_roi_xyxy),
        "object_gate_xyxy_exclusive": list(object_gate_xyxy),
        "locate": locate.diagnostics,
        "core_seed": core_seed_diagnostics,
        "core_color_statistics": color_statistics,
        "classification": classification,
        "hsv": hsv_diagnostics,
        "metrics": metrics,
        "review_status": review_status,
        "review_reasons": review_reasons,
    }

    return result, hsv_mask, draw_overlay(
        image,
        hsv_mask,
        object_gate_xyxy,
        result,
    )
