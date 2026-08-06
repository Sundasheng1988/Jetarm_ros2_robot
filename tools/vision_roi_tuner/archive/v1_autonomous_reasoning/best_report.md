# ROI Color Best Strategy Report
**Best iteration**: Otsu + Chroma + adjusted LAB ranges
**Accuracy**: 70.00% (14/20)
**Per-color**: {'blue': 1.0, 'red': 0.0}
**Params**: `{"use_otsu": true, "use_chroma": true, "chroma_threshold": 5.0, "lab_range_overrides": {"blue": {"B": [59, 143]}, "red": {"A": [124, 183], "B": [125, 189]}}}`

## Confusion Matrix
- **blue**: {'blue': 14}
- **red**: {'blue': 6}

## Details
- blue_block_001.jpg             blue   → blue     (1.000) OK
- blue_block_002.jpg             blue   → blue     (1.000) OK
- blue_block_003.jpg             blue   → blue     (1.000) OK
- blue_block_004.jpg             blue   → blue     (1.000) OK
- blue_cube_001.jpg              blue   → blue     (1.000) OK
- blue_cube_002.jpg              blue   → blue     (1.000) OK
- blue_cube_003.jpg              blue   → blue     (1.000) OK
- blue_cube_004.jpg              blue   → blue     (1.000) OK
- blue_cup_001.jpg               blue   → blue     (1.000) OK
- blue_cup_002.jpg               blue   → blue     (1.000) OK
- blue_cup_003.jpg               blue   → blue     (1.000) OK
- blue_cup_004.jpg               blue   → blue     (0.974) OK
- blue_cup_005.jpg               blue   → blue     (1.000) OK
- blue_cup_006.jpg               blue   → blue     (1.000) OK
- red_block_001.jpg              red    → blue     (1.000) WRONG
- red_block_002.jpg              red    → blue     (1.000) WRONG
- red_block_003.jpg              red    → blue     (1.000) WRONG
- red_block_004.jpg              red    → blue     (1.000) WRONG
- red_block_005.jpg              red    → blue     (1.000) WRONG
- red_block_006.jpg              red    → blue     (1.000) WRONG
