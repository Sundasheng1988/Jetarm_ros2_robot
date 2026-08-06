# Strategy Debug Evaluation

**Strategy**: Otsu + Chroma + adjusted LAB ranges

**Params**: `{"use_otsu": true, "use_chroma": true, "chroma_threshold": 5.0, "chromatic_min_confidence": 0.1, "min_confidence": 0.2, "lab_range_overrides": {"blue": {"B": [59, 143]}, "red": {"A": [124, 183], "B": [125, 189]}}}`

**Labeled images**: 3

**Labeled accuracy**: 1/3 = 33.33%

## Confusion Matrix

| True | blue | red | unknown |
|---|---|---|---|
| blue | 1  | 0  | 1 |
| red | 0  | 0  | 1 |

## Per-Image Detail

| blue_block_001.jpg | blue | blue → unknown | 0.043 [FILT] | ❌ |
| blue_block_002.jpg | N/A | blue → unknown | 0.042 [FILT] | ⬜ |
| blue_block_003.jpg | N/A | blue → unknown | 0.040 [FILT] | ⬜ |
| blue_block_004.jpg | N/A | blue → unknown | 0.022 [FILT] | ⬜ |
| blue_cube_001.jpg | N/A | blue → unknown | 0.100 [FILT] | ⬜ |
| blue_cube_002.jpg | N/A | blue → unknown | 0.107 [FILT] | ⬜ |
| blue_cube_003.jpg | N/A | blue → unknown | 0.104 [FILT] | ⬜ |
| blue_cube_004.jpg | N/A | blue → unknown | 0.101 [FILT] | ⬜ |
| blue_cup_001.jpg | blue | blue → blue | 0.556 | ✅ |
| blue_cup_002.jpg | N/A | blue → blue | 0.608 | ⬜ |
| blue_cup_003.jpg | N/A | blue → blue | 0.582 | ⬜ |
| blue_cup_004.jpg | N/A | blue → blue | 0.589 | ⬜ |
| blue_cup_005.jpg | N/A | blue → blue | 0.594 | ⬜ |
| blue_cup_006.jpg | N/A | blue → blue | 0.527 | ⬜ |
| red_block_001.jpg | red | blue → unknown | 0.079 [FILT] | ❌ |
| red_block_002.jpg | N/A | blue → unknown | 0.098 [FILT] | ⬜ |
| red_block_003.jpg | N/A | blue → unknown | 0.020 [FILT] | ⬜ |
| red_block_004.jpg | N/A | blue → unknown | 0.037 [FILT] | ⬜ |
| red_block_005.jpg | N/A | blue → unknown | 0.022 [FILT] | ⬜ |
| red_block_006.jpg | N/A | blue → unknown | 0.031 [FILT] | ⬜ |