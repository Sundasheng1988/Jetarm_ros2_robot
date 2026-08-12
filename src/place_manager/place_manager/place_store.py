"""命名地点的 YAML 数据管理。"""

import math
import os
import threading
from typing import Dict, List, Optional, Tuple

from place_manager.config import DEFAULT_PLACES_FILE
from place_manager.yaml_utils import dump_yaml, load_yaml


class PlaceStore:
    """命名地点的增删查改；读取接口会自动感知外部文件更新。"""

    def __init__(self, file_path: str = DEFAULT_PLACES_FILE):
        self._file_path = os.path.abspath(os.path.expanduser(file_path))
        self._places: Dict[str, Dict[str, float]] = {}
        self._aliases_by_name: Dict[str, List[str]] = {}
        self._alias_to_name: Dict[str, str] = {}
        self._signature: Optional[Tuple[int, int]] = None
        self._lock = threading.RLock()
        self._load()

    @property
    def file_path(self) -> str:
        return self._file_path

    def _file_signature(self) -> Optional[Tuple[int, int]]:
        try:
            stat = os.stat(self._file_path)
            return (stat.st_mtime_ns, stat.st_size)
        except FileNotFoundError:
            return None
        except OSError as exc:
            raise RuntimeError(
                f'无法读取地点文件状态: {self._file_path}\n  {exc}'
            ) from exc

    @staticmethod
    def _validated_place(data: object) -> Optional[Dict[str, float]]:
        if not isinstance(data, dict):
            return None
        if not all(key in data for key in ('x', 'y', 'yaw')):
            return None
        try:
            place = {
                'x': float(data['x']),
                'y': float(data['y']),
                'yaw': float(data['yaw']),
            }
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in place.values()):
            return None
        return place

    @staticmethod
    def _validated_aliases(name: str, data: object) -> List[str]:
        if not isinstance(data, dict) or 'aliases' not in data:
            return []
        raw_aliases = data['aliases']
        if not isinstance(raw_aliases, list):
            raise RuntimeError(f'地点 {name!r} 的 aliases 必须是列表')

        aliases: List[str] = []
        seen = set()
        for raw_alias in raw_aliases:
            if not isinstance(raw_alias, str) or not raw_alias.strip():
                raise RuntimeError(f'地点 {name!r} 包含无效别名')
            alias = raw_alias.strip()
            if alias in seen:
                raise RuntimeError(f'地点 {name!r} 重复声明别名 {alias!r}')
            seen.add(alias)
            aliases.append(alias)
        return aliases

    def _load(self) -> None:
        with self._lock:
            raw = load_yaml(self._file_path)
            raw_places = raw.get('places', {})
            if not isinstance(raw_places, dict):
                raw_places = {}

            loaded: Dict[str, Dict[str, float]] = {}
            aliases_by_name: Dict[str, List[str]] = {}
            for raw_name, raw_data in raw_places.items():
                name = str(raw_name).strip()
                place = self._validated_place(raw_data)
                if name and place is not None:
                    loaded[name] = place
                    aliases_by_name[name] = self._validated_aliases(
                        name, raw_data
                    )

            alias_to_name: Dict[str, str] = {}
            standard_names = set(loaded)
            for name, aliases in aliases_by_name.items():
                for alias in aliases:
                    if alias in standard_names:
                        raise RuntimeError(
                            f'地点别名 {alias!r} 与标准地点名冲突'
                        )
                    previous = alias_to_name.get(alias)
                    if previous is not None:
                        raise RuntimeError(
                            f'地点别名 {alias!r} 同时指向 '
                            f'{previous!r} 和 {name!r}'
                        )
                    alias_to_name[alias] = name

            self._places = loaded
            self._aliases_by_name = aliases_by_name
            self._alias_to_name = alias_to_name
            self._signature = self._file_signature()

    def _reload_if_changed(self) -> None:
        with self._lock:
            if self._file_signature() != self._signature:
                self._load()

    def _serialized_places(self) -> Dict[str, Dict[str, object]]:
        serialized: Dict[str, Dict[str, object]] = {}
        for name, coordinates in self._places.items():
            entry: Dict[str, object] = dict(coordinates)
            aliases = self._aliases_by_name.get(name, [])
            if aliases:
                entry['aliases'] = list(aliases)
            serialized[name] = entry
        return serialized

    def _save(self) -> None:
        with self._lock:
            dump_yaml(
                self._file_path,
                {'places': self._serialized_places()},
            )
            self._signature = self._file_signature()

    def load(self) -> None:
        self._load()

    def add(self, name: str, x: float, y: float, yaw: float) -> None:
        name = name.strip()
        if not name:
            raise ValueError('地点名称不能为空')

        values = (float(x), float(y), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('地点坐标必须是有限数值')

        with self._lock:
            # 保留其他进程刚写入的地点。
            self._reload_if_changed()
            if name in self._alias_to_name:
                raise ValueError(f'地点名称 {name!r} 已被用作别名')
            self._places[name] = {
                'x': values[0],
                'y': values[1],
                'yaw': values[2],
            }
            self._aliases_by_name.setdefault(name, [])
            self._save()

    def delete(self, name: str) -> bool:
        name = name.strip()
        with self._lock:
            self._reload_if_changed()
            if name not in self._places:
                return False
            del self._places[name]
            for alias in self._aliases_by_name.pop(name, []):
                self._alias_to_name.pop(alias, None)
            self._save()
            return True

    def exists(self, name: str) -> bool:
        return self.get(name) is not None

    def get(self, name: str) -> Optional[Dict[str, float]]:
        with self._lock:
            self._reload_if_changed()
            lookup_name = name.strip()
            canonical_name = self._alias_to_name.get(
                lookup_name, lookup_name
            )
            data = self._places.get(canonical_name)
            return None if data is None else dict(data)

    def list_names(self) -> List[str]:
        with self._lock:
            self._reload_if_changed()
            return sorted(self._places.keys())
