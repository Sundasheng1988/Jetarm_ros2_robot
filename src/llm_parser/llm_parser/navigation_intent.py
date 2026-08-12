#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航意图解析（命名地点导航 + 取消导航）。

本模块独立于机械臂抓取解析，用于在 ``parse()`` 的最前面识别两类语音导航
意图，并把它们转换成与现有 ``/parsed_command`` 协议兼容的 JSON：

- ``navigate_to_place``：去/前往/导航到/移动到/到达/到 <命名地点>
- ``cancel_navigation``：停止移动 / 停止导航 / 取消导航 / 别走了 ...

设计约束（与 Rebecca 语音栈、place_manager 保持一致）：

1. 不在本模块硬编码地点名单；地点是否存在由 ``place_manager`` 判断。
2. 使用严格的整句匹配，避免把普通聊天或机械臂指令误判为导航：
   - ``把蓝色杯子移动到右边`` → 必须仍是机械臂任务（句首是“把”，不匹配）。
   - ``介绍一下移动机器人技术`` → 必须仍是普通聊天（句首是“介绍”）。
   - ``去`` → 不得产生空地点导航。
3. ``停止说话`` 与 ``停止移动`` 必须严格分离：本模块只识别“停止移动/导航”，
   不识别“停止说话/安静”等只中止 LLM/TTS 的指令。
"""

from __future__ import annotations

import re
from typing import Optional, Dict


# 去掉中文与中文之间的空白（Vosk/ASR 常见分词空格）以及常见标点。
_SPACE_BETWEEN_CN = re.compile(r'(?<=[一-鿿])\s+(?=[一-鿿])')
_PUNCT = re.compile(r'[，。！!？?；;：:、\s]+')


def _normalize(text: str) -> str:
    """归一化：合并中文间空白、去掉首尾标点与空白。"""
    if not text:
        return ''
    t = _SPACE_BETWEEN_CN.sub('', text.strip())
    return t.strip()


# ── 导航到命名地点 ────────────────────────────────────────────────────────
# 句首动词，按“更长更具体”的顺序排列，避免 `到` 误吃 `导航到/到达`。
# 注意：`移动到` 必须成对出现，因此“移动蓝色杯子到右边”不会匹配（移动后不是到）。
_NAV_VERB_RE = re.compile(r'^(导航到|移动到|到达|前往|去|到)(.+)$', re.UNICODE)

# 这些占位词不是真实地点名，避免“去那边/去这里”被当作有效地点。
_PLACE_FILLER = {'那边', '这边', '那里', '这里', '哪儿', '哪里', '地方', '一下', '一下儿'}


def parse_navigation(text: str) -> Optional[Dict]:
    """识别“导航到命名地点”意图。

    返回 ``navigate_to_place`` 的命令 dict；若不是导航意图（或地点为空）返回 ``None``。
    """
    norm = _normalize(text)
    if not norm:
        return None

    m = _NAV_VERB_RE.match(norm)
    if not m:
        return None

    place = m.group(2).strip()
    # 去掉地点名末尾常见的方位助词（“客厅点1旁边”这类仍保留主名）。
    place = re.sub(r'[。！！.]+$', '', place).strip()
    if not place or place in _PLACE_FILLER:
        # “去”单独出现、或只有占位词 → 不产生空地点导航。
        return None

    return {
        'action': 'navigate_to_place',
        'place_name': place,
        'source': 'voice',
        'raw_text': text.strip() if text else norm,
        'raw': text.strip() if text else norm,
    }


# ── 取消导航 / 停止移动 ───────────────────────────────────────────────────
# 只匹配“停止/取消 + 移动/导航”或“别走/不要走”，绝不含“说话/安静”。
# 使用整句匹配：归一化后与下列短语集合精确比对，最大化可控性。
_STOP_MOVE_PHRASES = (
    '停止移动',
    '停止导航',
    '取消导航',
    '取消移动',
    '别走了',
    '别走啦',
    '别走',
    '不要走了',
    '不要走',
    '停下移动',
    '停下导航',
)

# 仅用于把“停止移动吧/停止移动。”这类带语气词的句子归一到核心短语。
_TAIL_NOISE = re.compile(r'[吧呢啊呀。！!？?，,]+$')


def is_cancel_navigation(text: str) -> bool:
    """是否为“停止移动 / 取消导航”意图（不需要二次确认）。"""
    norm = _normalize(text)
    if not norm:
        return False
    norm = _TAIL_NOISE.sub('', norm).strip()
    return norm in _STOP_MOVE_PHRASES


def parse_cancel_navigation(text: str) -> Optional[Dict]:
    """识别“停止移动 / 取消导航”意图，返回命令 dict 或 ``None``。"""
    if not is_cancel_navigation(text):
        return None
    return {
        'action': 'cancel_navigation',
        'source': 'voice',
        'raw_text': text.strip() if text else '',
        'raw': text.strip() if text else '',
    }
