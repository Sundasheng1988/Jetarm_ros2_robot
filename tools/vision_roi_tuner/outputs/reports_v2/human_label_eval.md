# Human-Labeled ROI Color Evaluation

**Images**: 3 (human-labeled)

**Accuracy**: 2/3 = 66.67%

**Wrong high-confidence**: 1

## Per-Color Accuracy

| Color | Correct | Total | Accuracy |
|-------|---------|-------|----------|
| blue | 2 | 2 | 100.00% |
| red | 0 | 1 | 0.00% |

## Per-Class Accuracy

| Class | Correct | Total | Accuracy |
|-------|---------|-------|----------|
| block | 1 | 2 | 50.00% |
| cup | 1 | 1 | 100.00% |

## Per-Mask-Quality Accuracy

| Mask Quality | Correct | Total | Accuracy |
|-------------|---------|-------|----------|
| bad | 1 | 2 | 50.00% |
| medium | 1 | 1 | 100.00% |

## Confusion Matrix (true \ predicted)

| True | blue | red |
|---|---|---|
| blue | 2  | 0 |
| red | 1  | 0 |

## High-Confidence Wrong Cases

**1/3 images**

| Image | True | Predicted | Confidence | Mask |
|-------|------|-----------|------------|------|
| red_block_001.jpg | red | blue | 1.000 | bad |

## Images with Bad Mask Quality

**2 images**

| Image | True Color | Predicted | Confidence | Note |
|-------|-----------|-----------|------------|------|
| blue_block_001.jpg | blue | blue | 1.000 | 轮廓主要包含背景，颜色结果不可信 |
| red_block_001.jpg | red | blue | 1.000 | 轮廓包含背景，颜色结果不可信 |

## Per-Image Detail

| Image | True | Predicted | Confidence | Mask | Correct |
|-------|------|-----------|------------|------|---------|
| blue_block_001.jpg | blue/block | blue | 1.000 | bad | ✅ |
| blue_cup_001.jpg | blue/cup | blue | 1.000 | medium | ✅ |
| red_block_001.jpg | red/block | blue | 1.000 | bad | ❌ |