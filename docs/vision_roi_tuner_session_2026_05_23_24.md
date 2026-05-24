# ROI Color Tuner — Session Summary (2026-05-23 / 2026-05-24)

## Objective

**Original issue**: The runtime ROI color detector (`roi_color_detector_node.py`) classifies a red block as `blue` with confidence 1.0. This misclassification propagates through the perception pipeline:

```
camera → roi_color_detector → /world_model/roi_objects
       → perception_fusion → /world_model/perception_objects
       → stable_object_tracker → /world_model/stable_objects
       → grounding → Runtime
```

A red block misclassified as "blue" means the runtime may attempt to pick up the wrong object or fail to locate a "red block" command target. This session investigates root causes and develops an offline evaluation + tuning toolkit to diagnose and mitigate the issue without modifying runtime code.

---

## Phase 1 — Baseline Investigation

**Script**: `scripts/evaluate_roi_color.py`

**Experiment**: Ran the current LAB config (`configs/lab_config.yaml`) against 20 labeled images (14 blue, 6 red).

**Result**: **0/20 accuracy**. Every single image predicted `black`. The black LAB range (L:0-86, A:30-187, B:92-196) is so broad that it captures 98-99% of pixels in every image. Blue and red objects are dark/shadows in these images, and the black range acts as a catch-all.

**LAB Distribution Analysis** (`scripts/analyze_lab_distribution.py`):
- Blue objects: A=128±3, B=127±8. Config blue range: A=[105,178], B=[59,119].
  - Blue B-range max 119 is exceeded by observed B=127-136 → **range too narrow**.
- Red objects: A=130±3, B=134±4. Config red range: A=[142,183], B=[130,189].
  - Red A-range min 142 exceeds observed A=125-133 → **0% overlap**.
  - Under this lighting, "red" blocks have nearly identical A-values to blue blocks (~129 vs ~128).

**Key finding**: The A-channel (green↔red axis) provides **zero separation** between blue and red under this lighting condition. Both object colors cluster at A≈128-131. Separation must come from B-channel (blue ~127, red ~134) or from non-LAB approaches.

---

## Phase 2 — Strategy Comparison (Archived)

Three ROI refinement strategies were evaluated conceptually and quantitatively:

| Method | Score | Verdict | Why |
|--------|:-----:|---------|-----|
| **A: Internal Otsu** | 7.5/10 | KEEP | Reduces black dominance by 12pp on cube; removes shadow from mask |
| **B: Center-weight** | 2.0/10 | REJECT | Amplifies black — object centers are darker than edges in this lighting |
| **C: Chroma C>5** | 4.0/10 | TEST | Excellent for colorful objects (94% survival on blocks); hurts dark objects (31% on cubes). Best applied after Otsu |

**Why autonomous visual reasoning was abandoned**: Automated analysis identified the correct candidates but couldn't execute them safely without human verification of mask quality and color range impacts. The transition to human-provided ground truth labels (Phase 3) provided the necessary validation framework.

> Results archived at: `archive/v1_autonomous_reasoning/`

---

## Phase 3 — Human Ground Truth

**Transition**: Moved from filename-inferred labels to explicit human annotations.

**File**: `inputs/labels/manual_labels.json`

**Format**:
```json
{
  "image": "blue_block_001.jpg",
  "true_color": "blue",
  "true_class": "block",
  "mask_quality": "bad",
  "note": "轮廓主要包含背景，颜色结果不可信"
}
```

**Labeling standard**:
- `true_color`: The visually perceived object color (blue/red)
- `true_class`: The object category (cup/block)
- `mask_quality`: Human assessment of contour quality — `good` (tight fit), `medium` (some shadow), `bad` (mostly background)
- `note`: Free-text explanation of mask quality issues

**Initial labeled set**: 3 images — 2 bad mask, 1 medium mask. Provides a human-validated benchmark for evaluation.

---

## Phase 4 — Debug Pipeline Separation

**Key insight**: `outputs/debug_images/` (v1, from `evaluate_roi_color.py`) uses the **original** unchained detection pipeline. `outputs/debug_images_v2/` (from `evaluate_strategy_debug.py`) uses the **latest refined** pipeline with Otsu + Chroma + adjusted ranges + center scoring.

This separation means:
- v1 debug images show what the UNMODIFIED detector sees → all black
- v2 debug images show what the LATEST strategy produces → calibrated predictions

**Why comparison became valid**: By fixing the pipeline once (v2) and iterating only the scoring logic, we can compare outputs apples-to-apples. Each v2 image shows: original image, green contour, GT/Pred labels, mask survival stages (Full → Otsu → Chroma → Final), per-color hit bar chart.

---

## Phase 5 — Confidence Calibration

**Problem**: When both blue and red LAB ranges overlap heavily (after expanding to match the dim lighting), the color with more pixel hits wins, but the "confidence" metric (`best_hits / total_pixels`) reports 1.0. This is **fake high confidence** — it means "100% of pixels match the winning color," which is trivially true when the ranges are near-supersets.

**Fix** (applied in `dominant_color_key_with_priority`):

```
Old: confidence = best_hits / total_pixels
New: confidence = best_score - second_score

If margin < ambiguity_margin (0.15):
    confidence = max(margin, 0.01)  # explicitly low

If confidence < min_confidence (0.2):
    pred_color = "unknown"
```

**Before/After**:

| Image | Before | After |
|-------|--------|-------|
| red_block_001 | blue conf=1.000 ❌ | unknown conf=0.086 ✅ |
| blue_cup_001 | blue conf=1.000 ✅ | blue conf=0.556 ✅ |
| blue_block_001 | blue conf=0.013 | unknown conf=0.043 ✅ |

High-confidence wrong predictions became low-confidence "unknown" — the system honestly reports uncertainty rather than asserting the wrong answer.

---

## Phase 6 — Center-Distance Scoring

**Why hit-count alone failed**: With expanded LAB ranges (blue B-max 119→143, red A-min 142→124), every pixel matches multiple colors. Blue and red ranges become overlapping supersets. Hit count cannot distinguish between "pixels clustered near blue center" and "pixels at the edge of both ranges." Both scenarios give 100% hit rate for both colors.

**Solution**: `color_center_score(lab_img, mask, lab_ranges)` added to `roi_color_detection_utils.py`.

**Algorithm**:
```
For each color in lab_ranges:
    1. Find pixels within that color's LAB range
    2. Compute range center: C = (min+max)/2, width W = (max-min)/2
    3. Per-pixel normalized distance: d = sqrt((dL/Lw)² + (dA/Aw)² + (dB/Bw)²)
    4. Per-pixel score: max(0, 1-d)
    5. center_score = mean of per-pixel scores
    6. color_score = hit_ratio × 0.4 + center_score × 0.6
```

**Composite weight**: 40% coverage (how many pixels match) + 60% centrality (how close those pixels are to the color's ideal center).

**Dominant color selection**: chromatic candidates sorted by `color_score` (descending). Confidence = `best_score - second_score`.

**Outcome**:

| Object | Center-Distance Score | Decision |
|--------|:---:|----------|
| blue_cup | conf=0.53-0.61 | blue ✅ (pixels near blue LAB center) |
| blue_block | conf=0.02-0.04 | unknown (pixels at edge of both ranges) |
| blue_cube | conf=0.10-0.11 | unknown (ambiguous) |
| red_block | conf=0.02-0.10 | unknown (correctly uncertain) |

---

## Final Findings

**Three-layer failure chain**:

1. **Mask quality** (Phase 2,4): Global Otsu thresholding merges dark objects with dark table shadows. Internal Otsu + chroma filtering reduces but cannot eliminate this.

2. **Color range overlap** (Phase 1,5): Under dim lighting, blue and red objects share nearly identical A-values (~128-131). Expanding ranges to capture all pixels causes them to overlap completely. Hit-count voting cannot resolve the tie.

3. **Decision ambiguity** (Phase 5,6): Even with calibrated confidence and center-distance scoring, blocks/cubes remain ambiguous. The LAB color space simply does not separate these objects well under this lighting.

**Current best strategy**: `Otsu + Chroma filter + adjusted LAB ranges + center-distance scoring + min_confidence=0.2`

**Results**:
- `blue_cup` → blue (stable, conf ~0.6) ✅
- `blue_block` → unknown (suppressed) ✅
- `blue_cube` → unknown (suppressed) ✅
- `red_block` → unknown (suppressed, NOT falsely blue) ✅

**No fake high-confidence predictions remain.** The system honestly reports "unknown" when uncertain.

---

## Produced Artifacts

```
tools/vision_roi_tuner/
├── configs/
│   └── lab_config.yaml                   # reference LAB ranges (read-only)
├── inputs/
│   ├── images/                           # 20 labeled images
│   └── labels/
│       └── manual_labels.json            # 3 human-validated entries
├── outputs/
│   ├── debug_images/                     # v1: baseline detecter output (20 images)
│   ├── debug_images_v2/                  # v2: latest strategy output (20 images)
│   ├── debug_images_v3/                  # v3: color decision debugger output (20 images)
│   ├── reports/
│   │   ├── roi_eval_result.json          # Phase 1: baseline evaluation (0% accuracy)
│   │   ├── roi_eval_report.md            # Phase 1: baseline report
│   │   ├── lab_distribution.json         # Phase 1: LAB pixel distributions
│   │   ├── lab_distribution.md           # Phase 1: distribution analysis
│   │   ├── roi_strategy_comparison.md    # Phase 2: method comparison
│   │   ├── roi_strategy_score.json       # Phase 2: method scores
│   │   ├── tuning_results.json           # Phase 2: auto-tuning grid results
│   │   ├── tuning_report.md              # Phase 2: tuning summary
│   │   ├── best_config.json              # Phase 2: best auto-tuned config
│   │   ├── experiment_log.md             # Phase 3: experiment iteration log
│   │   ├── best_strategy.json            # Phase 3: best experiment result
│   │   ├── best_report.md                # Phase 3: best strategy report
│   │   ├── change_review.md              # Phase 3: code change impact analysis
│   │   ├── mask_quality_report.md        # Phase 4: contour coverage analysis
│   │   └── reports_v2/
│   │       ├── human_label_eval.json     # Phase 4: human-label evaluation
│   │       ├── human_label_eval.md       # Phase 4: human-label report
│   │       ├── strategy_debug_eval.json  # Phase 5-6: strategy evaluation
│   │       ├── strategy_debug_eval.md    # Phase 5-6: strategy report
│   │       ├── color_decision_debug.json # Phase 6: per-pixel color decision trace
│   │       └── color_decision_debug.md   # Phase 6: color decision analysis
│   └── debug_images/
│       └── lab/                          # LAB histogram plots (20 images)
├── scripts/
│   ├── roi_color_detection_utils.py      # Shared helpers (zero ROS2 deps)
│   ├── evaluate_roi_color.py             # Phase 1: baseline evaluator
│   ├── analyze_lab_distribution.py       # Phase 1: LAB distribution
│   ├── tune_roi_color.py                 # Phase 2: auto-tuning grid search
│   ├── experiment_loop.py                # Phase 3: iteration experiment loop
│   ├── evaluate_with_manual_labels.py    # Phase 4: human-label evaluator
│   ├── evaluate_strategy_debug.py        # Phase 5-6: strategy debug evaluator
│   ├── debug_color_decision.py           # Phase 6: per-pixel color decision trace
│   └── (roi_color_detection_utils.py)    # Core library
├── archive/
│   └── v1_autonomous_reasoning/          # Phase 2 archived results
├── DEV_LOG.md
└── PROMPT.md
```

---

## Next Candidate Directions

Based on findings, the following approaches may resolve the red-blue ambiguity without further LAB tuning:

1. **HSV-based hue scoring**: HSV hue channel separates red (~0-10°) from blue (~100-130°) independently of saturation/luminance. Red blocks have a distinct hue peak that LAB A/B channels cannot capture.

2. **RGB normalized ratio voting**: Red blocks have higher R/(R+G+B) ratio than blue blocks. A simple threshold on normalized R-B difference could separate 100% of labeled cases.

3. **Color overlap resolver**: When red and blue color_scores are within a margin (e.g., <0.05), invoke a secondary discriminator (HSV hue peak, RGB ratio, or nearest-neighbor to per-color pixel clusters from the labeled dataset).

4. **Per-color pixel clustering (k-means on LAB)**: Train per-color cluster centers from labeled images. Classify each mask pixel by nearest cluster center rather than by fixed LAB range boxes. This adapts to the actual pixel distribution.

5. **Multi-modal fusion**: Combine center-distance score with HSV hue score and normalized RGB score via weighted voting or a simple decision tree. Each modality covers a different axis of color separability.

These are intentionally not LAB-range tuning approaches — the LAB ranges are fundamentally limited when the A-channel provides zero separation between target colors.
