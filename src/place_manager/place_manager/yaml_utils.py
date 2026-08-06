"""YAML 安全读写工具。"""

import os
from typing import Any, Dict

import yaml


def load_yaml(file_path: str) -> Dict[str, Any]:
    """读取 YAML；文件不存在时返回空 dict。"""
    if not os.path.exists(file_path):
        return {}

    try:
        with open(file_path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        return data if isinstance(data, dict) else {}
    except yaml.YAMLError as exc:
        raise RuntimeError(f'YAML 格式错误: {file_path}\n  {exc}') from exc
    except OSError as exc:
        raise RuntimeError(f'无法读取 YAML 文件: {file_path}\n  {exc}') from exc


def dump_yaml(file_path: str, data: Dict[str, Any]) -> None:
    """将 dict 原子写入 YAML，支持相对路径。"""
    absolute_path = os.path.abspath(file_path)
    parent_dir = os.path.dirname(absolute_path)
    tmp_path = absolute_path + '.tmp'

    try:
        os.makedirs(parent_dir, exist_ok=True)
        with open(tmp_path, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(
                data,
                stream,
                allow_unicode=True,
                default_flow_style=False,
                sort_keys=False,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_path, absolute_path)
    except OSError as exc:
        raise RuntimeError(f'无法写入 YAML 文件: {file_path}\n  {exc}') from exc
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass
