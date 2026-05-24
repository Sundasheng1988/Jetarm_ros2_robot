# Color Decision Debug Report

## Why Red Blocks Become Blue

Adjusted LAB ranges create overlap between blue and red:

- **Blue range**: A=[105,178] B=[59,143] (A-width=73, B-width=84)
- **Red range**:  A=[124,183] B=[125,189] (A-width=59, B-width=64)

Both ranges fully capture red-block pixels (A≈129, B≈134). 
Blue wins because its wider B-range captures more pixels, and its lower B-min (59 vs red's 125) 
includes darker/neutral border pixels that red excludes.

### Red Block Analysis

**red_block_001.jpg** (GT=red, pred=blue, conf=1.0)

- LAB median: L=69 A=131 B=136
- RGB norm: R=0.390 G=0.330 B=0.280  R-B diff=0.1100
- HSV: H=14 S=77 V=73
- Red hits=18559, Blue hits=20305, Black hits=19826
- Red/blue: overlap_both=18559 red_only=0 blue_only=1746 ratio=0.914
- Win reason: chromatic 'blue' conf=1.0000 >= 0.1 (hits=20305/20305)

**red_block_002.jpg** (GT=red, pred=blue, conf=0.9996)

- LAB median: L=72 A=131 B=136
- RGB norm: R=0.380 G=0.330 B=0.290  R-B diff=0.0900
- HSV: H=15 S=74 V=75
- Red hits=17244, Blue hits=19527, Black hits=18695
- Red/blue: overlap_both=17236 red_only=8 blue_only=2291 ratio=0.8831
- Win reason: chromatic 'blue' conf=0.9996 >= 0.1 (hits=19527/19535)

**red_block_003.jpg** (GT=red, pred=blue, conf=1.0)

- LAB median: L=63 A=131 B=136
- RGB norm: R=0.400 G=0.330 B=0.270  R-B diff=0.1300
- HSV: H=14 S=84 V=68
- Red hits=20030, Blue hits=20119, Black hits=20111
- Red/blue: overlap_both=20030 red_only=0 blue_only=89 ratio=0.9956
- Win reason: chromatic 'blue' conf=1.0000 >= 0.1 (hits=20119/20119)

**red_block_004.jpg** (GT=red, pred=blue, conf=0.9999)

- LAB median: L=63 A=131 B=136
- RGB norm: R=0.400 G=0.330 B=0.270  R-B diff=0.1300
- HSV: H=14 S=80 V=69
- Red hits=21369, Blue hits=21733, Black hits=21578
- Red/blue: overlap_both=21367 red_only=2 blue_only=366 ratio=0.9833
- Win reason: chromatic 'blue' conf=0.9999 >= 0.1 (hits=21733/21735)

**red_block_005.jpg** (GT=red, pred=blue, conf=1.0)

- LAB median: L=63 A=131 B=136
- RGB norm: R=0.400 G=0.330 B=0.270  R-B diff=0.1300
- HSV: H=15 S=82 V=67
- Red hits=21128, Blue hits=21246, Black hits=21220
- Red/blue: overlap_both=21127 red_only=1 blue_only=119 ratio=0.9944
- Win reason: chromatic 'blue' conf=1.0000 >= 0.1 (hits=21246/21247)

**red_block_006.jpg** (GT=red, pred=blue, conf=1.0)

- LAB median: L=63 A=131 B=136
- RGB norm: R=0.400 G=0.330 B=0.270  R-B diff=0.1300
- HSV: H=14 S=81 V=69
- Red hits=20781, Blue hits=20953, Black hits=20909
- Red/blue: overlap_both=20780 red_only=1 blue_only=173 ratio=0.9918
- Win reason: chromatic 'blue' conf=1.0000 >= 0.1 (hits=20953/20954)

### Blue Object Comparison

**blue_block_001.jpg**
- LAB median: L=69 A=131 B=136
- RGB norm: R=0.390 G=0.330 B=0.270
- Red hits=13151, Blue hits=13331, Black hits=13255
- Red/blue ratio=0.9865

**blue_block_002.jpg**
- LAB median: L=69 A=132 B=136
- RGB norm: R=0.400 G=0.330 B=0.270
- Red hits=13441, Blue hits=13648, Black hits=13573
- Red/blue ratio=0.9848

**blue_block_003.jpg**
- LAB median: L=68 A=132 B=136
- RGB norm: R=0.400 G=0.330 B=0.270
- Red hits=14262, Blue hits=14408, Black hits=14357
- Red/blue ratio=0.9899

**blue_block_004.jpg**
- LAB median: L=63 A=131 B=136
- RGB norm: R=0.400 G=0.330 B=0.270
- Red hits=20035, Blue hits=20119, Black hits=20107
- Red/blue ratio=0.9958

**blue_cube_001.jpg**
- LAB median: L=75 A=131 B=135
- RGB norm: R=0.380 G=0.330 B=0.290
- Red hits=24896, Blue hits=27282, Black hits=22628
- Red/blue ratio=0.9125

**blue_cube_002.jpg**
- LAB median: L=76 A=130 B=135
- RGB norm: R=0.370 G=0.340 B=0.290
- Red hits=26870, Blue hits=30367, Black hits=23785
- Red/blue ratio=0.8848

**blue_cube_003.jpg**
- LAB median: L=75 A=131 B=135
- RGB norm: R=0.380 G=0.330 B=0.290
- Red hits=25180, Blue hits=27793, Black hits=22619
- Red/blue ratio=0.906

**blue_cube_004.jpg**
- LAB median: L=76 A=130 B=135
- RGB norm: R=0.370 G=0.330 B=0.290
- Red hits=26088, Blue hits=29001, Black hits=23449
- Red/blue ratio=0.8996

**blue_cup_001.jpg**
- LAB median: L=42 A=130 B=106
- RGB norm: R=0.120 G=0.330 B=0.540
- Red hits=1839, Blue hits=18399, Black hits=18275
- Red/blue ratio=0.1

**blue_cup_002.jpg**
- LAB median: L=50 A=130 B=104
- RGB norm: R=0.120 G=0.330 B=0.550
- Red hits=442, Blue hits=9650, Black hits=9507
- Red/blue ratio=0.0458

**blue_cup_003.jpg**
- LAB median: L=55 A=128 B=106
- RGB norm: R=0.180 G=0.340 B=0.480
- Red hits=1487, Blue hits=6047, Black hits=5344
- Red/blue ratio=0.2459

**blue_cup_004.jpg**
- LAB median: L=84 A=125 B=113
- RGB norm: R=0.250 G=0.340 B=0.410
- Red hits=461, Blue hits=1556, Black hits=866
- Red/blue ratio=0.2963

**blue_cup_005.jpg**
- LAB median: L=46 A=130 B=105
- RGB norm: R=0.110 G=0.330 B=0.560
- Red hits=154, Blue hits=9618, Black hits=9535
- Red/blue ratio=0.016

**blue_cup_006.jpg**
- LAB median: L=91 A=124 B=131
- RGB norm: R=0.290 G=0.350 B=0.360
- Red hits=1617, Blue hits=2510, Black hits=1007
- Red/blue ratio=0.6442

## Root Cause

**Red and blue objects have nearly identical A-values** (127-131) under this lighting. 
The A-channel (green↔red axis) provides zero separation. 
The B-channel (blue↔yellow axis) has a slight difference (blue ~127, red ~134), 
but after expanding both ranges to capture all pixels, they overlap completely. 
Blue's wider range always wins the pixel-count vote because it's a superset of the red range.

**Fix options** (not implemented):

1. Narrow blue B-range (remove the 59→143 expansion, keep original 59→119) to eliminate overlap

2. Use ratio voting: pick the color with higher hit ratio, not higher absolute hits

3. Use RGB-based discrimination: red blocks have higher R/(R+G+B) ratio than blue objects

4. Restore original red A-min (142) to exclude red-block pixels entirely (they're actually not red in LAB)