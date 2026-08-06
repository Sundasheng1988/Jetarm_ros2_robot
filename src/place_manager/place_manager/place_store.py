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

    def _load(self) -> None:
        with self._lock:
            raw = load_yaml(self._file_path)
            raw_places = raw.get('places', {})
            if not isinstance(raw_places, dict):
                raw_places = {}

            loaded: Dict[str, Dict[str, float]] = {}
            for raw_name, raw_data in raw_places.items():
                name = str(raw_name).strip()
                place = self._validated_place(raw_data)
                if name and place is not None:
                    loaded[name] = place

            self._places = loaded
            self._signature = self._file_signature()

    def _reload_if_changed(self) -> None:
        with self._lock:
            if self._file_signature() != self._signature:
                self._load()

    def _save(self) -> None:
        with self._lock:
            dump_yaml(self._file_path, {'places': self._places})
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
            self._places[name] = {
                'x': values[0],
                'y': values[1],
                'yaw': values[2],
            }
            self._save()

    def delete(self, name: str) -> bool:
        name = name.strip()
        with self._lock:
            self._reload_if_changed()
            if name not in self._places:
                return False
            del self._places[name]
            self._save()
            return True

    def exists(self, name: str) -> bool:
        return self.get(name) is not None

    def get(self, name: str) -> Optional[Dict[str, float]]:
        with self._lock:
            self._reload_if_changed()
            data = self._places.get(name.strip())
            return None if data is None else dict(data)

    def list_names(self) -> List[str]:
        with self._lock:
            self._reload_if_changed()
            return sorted(self._places.keys())
