# Experiment Log
**Date**: 2026-05-24
**Images**: 20 (14 blue, 6 red)
**Max iterations**: 5
### Iter 1: Baseline (chromatic priority only)
| Metric | Value |
|--------|-------|
| Accuracy | 30.00% (6/20) |
| Blue | 42.86% |
| Red | 0.00% |
| Wrong high-conf | 10 |
| False black | 14 |
| Verdict | **KEEP** |
| Reason | Establish baseline with chromatic priority voting, no mask refinement |
| Params | `{"use_otsu": false, "use_chroma": false}` |
### Iter 2: Otsu body extraction
| Metric | Value |
|--------|-------|
| Accuracy | 25.00% (5/20) |
| Blue | 35.71% |
| Red | 0.00% |
| Wrong high-conf | 10 |
| False black | 14 |
| Verdict | **ROLLBACK** |
| Reason | Internal Otsu removes shadow/background leaks from mask before color voting |
| Params | `{"use_otsu": true, "use_chroma": false}` |
### Iter 3: Otsu + Chroma filtering (C>5)
| Metric | Value |
|--------|-------|
| Accuracy | 30.00% (6/20) |
| Blue | 42.86% |
| Red | 0.00% |
| Wrong high-conf | 14 |
| False black | 14 |
| Verdict | **ROLLBACK** |
| Reason | Otsu removes shadow, Chroma removes residual neutral pixels. Combined filtering. |
| Params | `{"use_otsu": true, "use_chroma": true, "chroma_threshold": 5.0}` |
### Iter 4: Otsu + Chroma + adjusted LAB ranges
| Metric | Value |
|--------|-------|
| Accuracy | 70.00% (14/20) |
| Blue | 100.00% |
| Red | 0.00% |
| Wrong high-conf | 6 |
| False black | 0 |
| Verdict | **KEEP** |
| Reason | Expand blue B-max (119→143) and red A-min (142→124) to match observed LAB distributions from v0.3 analysis |
| Params | `{"use_otsu": true, "use_chroma": true, "chroma_threshold": 5.0, "lab_range_overrides": {"blue": {"B": [59, 143]}, "red": {"A": [124, 183], "B": [125, 189]}}}` |
### Iter 5: Otsu + Chroma + ranges + confidence tuning
| Metric | Value |
|--------|-------|
| Accuracy | 70.00% (14/20) |
| Blue | 100.00% |
| Red | 0.00% |
| Wrong high-conf | 6 |
| False black | 0 |
| Verdict | **ROLLBACK** |
| Reason | Raise chromatic_min to 0.15 and add min_confidence=0.45 to suppress weak predictions |
| Params | `{"use_otsu": true, "use_chroma": true, "chroma_threshold": 5.0, "chromatic_min_confidence": 0.15, "min_confidence": 0.45, "lab_range_overrides": {"blue": {"B": [59, 143]}, "red": {"A": [124, 183], "B": [125, 189]}}}` |
