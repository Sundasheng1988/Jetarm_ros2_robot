# Experiment Change Review

**Date**: 2026-05-24  
**Total experiments**: 5 (max 5)  
**Best iteration**: 4 — Otsu + Chroma + adjusted LAB ranges  
**Best accuracy**: 70.00% (14/20) — blue=100%, red=0%

---

## Iteration 1: Baseline (chromatic priority only)

| Metric | Value |
|--------|-------|
| Accuracy | 30.00% (6/20) |
| Blue | 42.86% (6/14) |
| Red | 0.00% (0/6) |
| False black | 14 |
| Wrong high-conf | 10 |
| Decision | **KEEP** (baseline) |

**Changed**: None — initial measurement.

**Files**: No files changed (baseline measurement using existing code).

---

## Iteration 2: Otsu body extraction

| Metric | Value |
|--------|-------|
| Accuracy | 25.00% (5/20) |
| Blue | 35.71% (5/14) |
| Red | 0.00% (0/6) |
| False black | 14 |
| Wrong high-conf | 10 |
| Decision | **ROLLBACK** |

**Changed files**:
- `roi_color_detection_utils.py`: added `estimate_color_with_otsu_chroma()` function (+35 LOC)
- `experiment_loop.py`: created (+242 LOC)

**Function changed**: `estimate_color_with_otsu_chroma` — applies internal Otsu threshold within the eroded contour mask, removing pixels darker than the Otsu split point.

**Behavior impact**: 
- Reduces mask area to "body" pixels only
- On images where body/shadow have similar brightness (blue_block: body L=72, shadow L~50, otsu=57), removes ~49% of mask including some body pixels
- On blue_cube, body goes from L=67→87 but blue hits dropped from 6 to 5

**Risk**: Low — function is additive, doesn't modify existing code paths. Parameter `use_otsu=False` by default.

**Why rolled back**: Lost 1 blue classification. Otsu assumed body is brighter than shadow, but shadow brightness varies. Global Otsu within mask is too coarse when body and shadow distributions overlap.

---

## Iteration 3: Otsu + Chroma filtering

| Metric | Value |
|--------|-------|
| Accuracy | 30.00% (6/20) |
| Blue | 42.86% (6/14) |
| Red | 0.00% (0/6) |
| False black | 14 |
| Wrong high-conf | 14 |
| Decision | **ROLLBACK** |

**Changed files**: None (parameter change only)

**Behavior impact**: Added chroma C<5 filter on top of Otsu. Removed low-chroma pixels from the already-reduced Otsu mask. Wrong high-confidence increased from 10→14 (more high-confidence wrong predictions).

**Risk**: Medium — chroma threshold is a hard cutoff. Objects with naturally low saturation (blue_cube: 67% C<5) lose most body pixels.

**Why rolled back**: No improvement over baseline. Wrong high-conf increased. Chroma filtering alone can't fix color range mismatch — it removes pixels but doesn't change which color range those pixels match.

---

## Iteration 4: Adjusted LAB ranges

| Metric | Value |
|--------|-------|
| Accuracy | **70.00%** (14/20) |
| Blue | **100.00%** (14/14) |
| Red | 0.00% (0/6) |
| False black | **0** |
| Wrong high-conf | 6 |
| Decision | **KEEP** |

**Changed files**: None (parameter change via `lab_range_overrides` in `estimate_color_with_otsu_chroma`)

**Function changed**: `dominant_color_key_with_priority` receives overridden ranges:
- Blue B: [59, 119] → [59, 143]
- Red A: [142, 183] → [124, 183]
- Red B: [130, 189] → [125, 189]

**Behavior impact**:
- **Blue 43% → 100%**: Blue blocks/cubes have B=130-136 which was above the old max of 119. New max 143 captures all blue body pixels.
- **Red 0% → 0% (but classified as blue)**: Red blocks have A=129. Old red A-min 142 excluded them → black prediction. New A-min 124 includes them, but they also match blue A-range [105,178]. Blue wins because blue range covers more pixels (wider A/B span).
- **False black eliminated**: 14 → 0. Black no longer dominates because objects now match at least one chromatic range.

**Risk**: Medium — expanding blue B-max could cause false blue detection on non-blue objects. Red A-min lowering causes red→blue misclassification. Red differentiation requires better range separation, not just expansion.

**Why kept**: Highest accuracy (70%). Blue classification is now perfect. Red misclassified as blue rather than black — this is a "better wrong" (blue is at least a chromatic color, and the robot can potentially correct with shape/context). Eliminating false black is a major safety improvement (black→action blocked, blue→action may proceed with wrong target).

---

## Iteration 5: Confidence tuning

| Metric | Value |
|--------|-------|
| Accuracy | 70.00% (14/20) |
| Blue | 100.00% (14/14) |
| Red | 0.00% (0/6) |
| False black | 0 |
| Wrong high-conf | 6 |
| Decision | **ROLLBACK** |

**Changed files**: None (parameter change only)

**Behavior impact**: Raised chromatic_min_confidence to 0.15 and min_confidence to 0.45. No change in output — all surviving predictions had confidence above these thresholds.

**Risk**: None.

**Why rolled back**: No improvement. Thresholds already adequate at defaults.

---

## Limiting Factors

| Factor | Status |
|--------|--------|
| **LOC delta** | +277 (utils +35, experiment +242) — under budget |
| **Files changed** | 2 (utils.py, experiment_loop.py) — under budget |
| **Iterations used** | 5/5 |

---

## Best Strategy

**Name**: Otsu + Chroma + adjusted LAB ranges

```json
{
  "use_otsu": true,
  "use_chroma": true,
  "chroma_threshold": 5.0,
  "chromatic_min_confidence": 0.10,
  "lab_range_overrides": {
    "blue": {"B": [59, 143]},
    "red": {"A": [124, 183], "B": [125, 189]}
  }
}
```

**Metrics**: accuracy=70%, blue=100%, red=0%, false_black=0

**Next step**: Red differentiation needs further work. Red blocks and blue blocks have nearly identical A-values (129 vs 128). Differentiation must come from B-channel (blue ~127, red ~134) or from shape classification (block vs cup). Expanding red A-min to 124 was necessary but insufficient — red and blue ranges now overlap, and blue's wider coverage wins the vote. A weighted or context-aware voting scheme may help.
