#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""PlaceStore 地点别名的离线回归测试。"""

import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from place_manager.place_store import PlaceStore  # noqa: E402
from place_manager.yaml_utils import dump_yaml, load_yaml  # noqa: E402


def place(x, y, yaw, aliases=None):
    data = {'x': x, 'y': y, 'yaw': yaw}
    if aliases is not None:
        data['aliases'] = aliases
    return data


class PlaceStoreAliasTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.path = Path(self.temp_dir.name) / 'places.yaml'
        self._write({
            'bedroom_sunnysun': place(
                -7.0, 1.5, 0.16,
                ['卧室', '房间', 'bedroom'],
            ),
            'kitchen': place(3.5, 1.5, 0.04, ['厨房']),
        })
        self.store = PlaceStore(str(self.path))

    def tearDown(self):
        self.temp_dir.cleanup()

    def _write(self, places):
        dump_yaml(str(self.path), {'places': places})

    def test_standard_name_and_aliases_resolve_to_same_coordinates(self):
        expected = self.store.get('bedroom_sunnysun')
        self.assertIsNotNone(expected)
        for name in ('卧室', '房间', 'bedroom'):
            with self.subTest(name=name):
                self.assertEqual(self.store.get(name), expected)

    def test_unknown_name_fails_and_list_contains_only_standard_names(self):
        self.assertIsNone(self.store.get('不存在的地点'))
        self.assertEqual(
            self.store.list_names(),
            ['bedroom_sunnysun', 'kitchen'],
        )

    def test_add_preserves_existing_aliases_in_yaml(self):
        self.store.add('living_room', -5.7, -2.9, 3.08)

        raw = load_yaml(str(self.path))
        self.assertEqual(
            raw['places']['bedroom_sunnysun']['aliases'],
            ['卧室', '房间', 'bedroom'],
        )
        reloaded = PlaceStore(str(self.path))
        self.assertEqual(
            reloaded.get('卧室'),
            reloaded.get('bedroom_sunnysun'),
        )

    def test_add_rejects_name_already_used_as_alias(self):
        with self.assertRaises(ValueError):
            self.store.add('卧室', 0.0, 0.0, 0.0)

    def test_delete_accepts_only_standard_name_and_removes_aliases(self):
        self.assertFalse(self.store.delete('卧室'))
        self.assertIsNotNone(self.store.get('卧室'))

        self.assertTrue(self.store.delete('bedroom_sunnysun'))
        self.assertIsNone(self.store.get('bedroom_sunnysun'))
        self.assertIsNone(self.store.get('卧室'))

    def test_duplicate_or_shared_alias_is_rejected(self):
        invalid_cases = (
            {
                'one': place(0, 0, 0, ['重复', '重复']),
            },
            {
                'one': place(0, 0, 0, ['共享']),
                'two': place(1, 1, 1, ['共享']),
            },
        )
        for places in invalid_cases:
            with self.subTest(places=places):
                self._write(places)
                with self.assertRaises(RuntimeError):
                    PlaceStore(str(self.path))

    def test_alias_conflicting_with_standard_name_is_rejected(self):
        self._write({
            'bedroom': place(0, 0, 0),
            'other': place(1, 1, 1, ['bedroom']),
        })
        with self.assertRaises(RuntimeError):
            PlaceStore(str(self.path))

    def test_hot_reload_replaces_alias_index(self):
        self.assertIsNotNone(self.store.get('卧室'))
        self._write({
            'bedroom_sunnysun': place(
                -7.0, 1.5, 0.16,
                ['睡觉的房间'],
            ),
            'kitchen': place(3.5, 1.5, 0.04, ['厨房']),
        })

        self.assertIsNone(self.store.get('卧室'))
        self.assertEqual(
            self.store.get('睡觉的房间'),
            self.store.get('bedroom_sunnysun'),
        )


if __name__ == '__main__':
    unittest.main()
