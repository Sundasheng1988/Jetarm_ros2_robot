#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import py_compile
import stat
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from llm_voice_agent import rebecca_start


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ENV_NAME = "ZHIPUAI_API_KEY"


class _FakeResp:
    def __init__(self, body):
        self._body = body.encode("utf-8") if isinstance(body, str) else body

    def read(self):
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _urlopen_returning(body):
    return lambda url, timeout=None: _FakeResp(body)


class CosyvoicePythonTests(unittest.TestCase):
    def test_missing_interpreter_raises_without_system_fallback(self):
        # 指向不存在的解释器必须报错退出，绝不静默回退系统 python3。
        with mock.patch.dict(os.environ, {"COSYVOICE_PYTHON": "/no/such/python_xyz"}, clear=False):
            with self.assertRaises(SystemExit) as caught:
                rebecca_start.find_cosyvoice_python()
        self.assertIn("cosyvoice", str(caught.exception))

    def test_explicit_valid_interpreter_is_used(self):
        with tempfile.TemporaryDirectory() as tmp:
            py = os.path.join(tmp, "python3")
            Path(py).write_text("#!/bin/sh\n")
            os.chmod(py, 0o755)
            with mock.patch.dict(os.environ, {"COSYVOICE_PYTHON": py}, clear=False):
                self.assertEqual(rebecca_start.find_cosyvoice_python(), py)

    def test_default_interpreter_is_not_system_python3(self):
        # 即便用默认值，返回的也不能是系统 python3。
        with mock.patch.dict(os.environ, {"COSYVOICE_PYTHON": ""}, clear=True):
            try:
                resolved = rebecca_start.find_cosyvoice_python()
            except SystemExit:
                self.skipTest("本机未安装默认 cosyvoice 解释器，跳过默认路径断言")
            self.assertNotEqual(resolved, "/usr/bin/python3")
            self.assertNotEqual(resolved, "python3")
            self.assertTrue(os.path.isfile(resolved))

    def test_cosyvoice_child_isolated_from_ros_and_user_python_paths(self):
        with mock.patch.dict(
            os.environ,
            {
                "PYTHONPATH": "/tmp/ros-or-user-python-path",
                "PYTHONHOME": "/tmp/wrong-python-home",
                "CUDA_VISIBLE_DEVICES": "0",
            },
            clear=False,
        ), mock.patch.object(rebecca_start.subprocess, "Popen") as popen:
            rebecca_start.start_cosyvoice(
                "/conda/cosyvoice/bin/python3",
                "/share/cosyvoice_fp16_server.py",
                "/models/cosyvoice",
                50000,
                "127.0.0.1",
            )

        cmd = popen.call_args.args[0]
        kwargs = popen.call_args.kwargs
        self.assertEqual(
            cmd[:4],
            [
                "/conda/cosyvoice/bin/python3",
                "-E",
                "-s",
                "/share/cosyvoice_fp16_server.py",
            ],
        )
        self.assertTrue(kwargs["start_new_session"])
        self.assertEqual(kwargs["env"]["PYTHONNOUSERSITE"], "1")
        self.assertNotIn("PYTHONPATH", kwargs["env"])
        self.assertNotIn("PYTHONHOME", kwargs["env"])
        self.assertEqual(kwargs["env"]["CUDA_VISIBLE_DEVICES"], "0")


class ProbeCosyvoiceTests(unittest.TestCase):
    def test_openapi_with_zero_shot_is_cosyvoice(self):
        body = json.dumps({"paths": {"/inference_zero_shot": {}}})
        with mock.patch.object(rebecca_start.urllib.request, "urlopen", _urlopen_returning(body)):
            state, _ = rebecca_start.probe_cosyvoice()
        self.assertEqual(state, "cosyvoice")

    def test_foreign_service_is_other(self):
        body = json.dumps({"paths": {"/something_else": {}}})
        with mock.patch.object(rebecca_start.urllib.request, "urlopen", _urlopen_returning(body)):
            state, _ = rebecca_start.probe_cosyvoice()
        self.assertEqual(state, "other")

    def test_connection_refused_is_down(self):
        def boom(url, timeout=None):
            raise rebecca_start.urllib.error.URLError("refused")
        with mock.patch.object(
            rebecca_start.urllib.request, "urlopen", boom
        ), mock.patch.object(
            rebecca_start, "_tcp_port_open", return_value=False
        ):
            state, _ = rebecca_start.probe_cosyvoice()
        self.assertEqual(state, "down")

    def test_raw_tcp_listener_is_foreign_service_not_down(self):
        def boom(url, timeout=None):
            raise rebecca_start.urllib.error.URLError("invalid http")

        with mock.patch.object(
            rebecca_start.urllib.request, "urlopen", boom
        ), mock.patch.object(
            rebecca_start, "_tcp_port_open", return_value=True
        ):
            state, _ = rebecca_start.probe_cosyvoice()
        self.assertEqual(state, "other")

    def test_non_loopback_host_is_rejected(self):
        with self.assertRaises(SystemExit):
            rebecca_start.normalize_local_host("0.0.0.0")
        with self.assertRaises(SystemExit):
            rebecca_start.normalize_local_host("192.168.1.10")
        self.assertEqual(
            rebecca_start.normalize_local_host("localhost"), "127.0.0.1"
        )

    def test_health_wait_fails_immediately_if_owned_process_exits(self):
        class _Exited:
            def poll(self):
                return 1

        with mock.patch.object(rebecca_start, "probe_cosyvoice") as probe:
            healthy = rebecca_start.wait_health(
                "127.0.0.1",
                50000,
                timeout=240,
                process=_Exited(),
            )

        self.assertFalse(healthy)
        probe.assert_not_called()


class PlanAndCleanupTests(unittest.TestCase):
    def test_existing_cosyvoice_is_adopted_not_started(self):
        # 已存在的 CosyVoice → adopt；该分支 cosy_proc=None，清理时不被停止。
        self.assertEqual(rebecca_start.plan_cosyvoice_action("cosyvoice"), "adopt")
        self.assertEqual(rebecca_start.plan_cosyvoice_action("down"), "start")
        self.assertEqual(rebecca_start.plan_cosyvoice_action("other"), "error")

    def test_terminate_of_none_or_dead_is_noop(self):
        # 复用进程 cosy_proc=None：清理必须是 no-op，不触碰任何进程。
        self.assertFalse(rebecca_start.terminate_process_group(None, "CosyVoice"))

        class _Dead:
            def poll(self):
                return 0

        self.assertFalse(rebecca_start.terminate_process_group(_Dead(), "CosyVoice"))

    def test_run_reuses_existing_cosyvoice_without_stopping_it(self):
        class _Launch:
            pid = 2468

            def wait(self):
                return 0

        launch = _Launch()
        with mock.patch.object(
            rebecca_start, "ensure_api_key", return_value=("env", None)
        ), mock.patch.object(
            rebecca_start, "probe_cosyvoice", return_value=("cosyvoice", {})
        ), mock.patch.object(
            rebecca_start.subprocess, "Popen", return_value=launch
        ), mock.patch.object(
            rebecca_start, "terminate_process_group", return_value=False
        ) as terminate:
            rebecca_start.run([])

        terminate.assert_called_once_with(launch, "voice_stack", grace=10)

    def test_owned_cosyvoice_is_cleaned_if_voice_stack_cannot_start(self):
        class _Cosy:
            pid = 1357

        cosy = _Cosy()
        with mock.patch.object(
            rebecca_start, "ensure_api_key", return_value=("env", None)
        ), mock.patch.object(
            rebecca_start, "probe_cosyvoice", return_value=("down", "refused")
        ), mock.patch.object(
            rebecca_start, "find_cosyvoice_python", return_value="/fake/python"
        ), mock.patch.object(
            rebecca_start, "cosyvoice_share_script", return_value="/fake/server.py"
        ), mock.patch.object(
            rebecca_start, "start_cosyvoice", return_value=cosy
        ), mock.patch.object(
            rebecca_start, "wait_health", return_value=True
        ), mock.patch.object(
            rebecca_start.subprocess, "Popen", side_effect=OSError("ros2 missing")
        ), mock.patch.object(
            rebecca_start, "terminate_process_group", return_value=True
        ) as terminate:
            with self.assertRaises(OSError):
                rebecca_start.run([])

        terminate.assert_called_once_with(cosy, "CosyVoice", grace=10)


class SecretsTests(unittest.TestCase):
    def setUp(self):
        # 每个用例都从干净环境开始
        os.environ.pop(ENV_NAME, None)

    def test_env_present_is_used_without_file(self):
        with mock.patch.dict(os.environ, {ENV_NAME: "from-env"}, clear=False):
            source, spath = rebecca_start.ensure_api_key()
        self.assertEqual(source, "env")
        self.assertIsNone(spath)

    def test_file_is_loaded_when_env_absent(self):
        with tempfile.TemporaryDirectory() as tmp:
            secrets = os.path.join(tmp, "secrets.env")
            with open(secrets, "w", encoding="utf-8") as fh:
                fh.write(f"{ENV_NAME}=from-file\n")
            os.chmod(secrets, 0o600)
            source, spath = rebecca_start.ensure_api_key(secrets_path=secrets)
        self.assertEqual(source, "file")
        self.assertEqual(spath, secrets)
        self.assertEqual(os.environ.get(ENV_NAME), "from-file")

    def test_existing_file_permissions_are_corrected_to_600(self):
        with tempfile.TemporaryDirectory() as tmp:
            secrets = os.path.join(tmp, "secrets.env")
            with open(secrets, "w", encoding="utf-8") as fh:
                fh.write(f"{ENV_NAME}=from-file\n")
            os.chmod(secrets, 0o644)
            rebecca_start.ensure_api_key(secrets_path=secrets)
            self.assertEqual(stat.S_IMODE(os.stat(secrets).st_mode), 0o600)

    def test_first_run_prompts_and_writes_mode_600(self):
        with tempfile.TemporaryDirectory() as tmp:
            secrets = os.path.join(tmp, "nested", "secrets.env")
            source, spath = rebecca_start.ensure_api_key(
                secrets_path=secrets,
                prompt=lambda p: "super-secret-key",
            )
            self.assertEqual(source, "prompted")
            self.assertEqual(spath, secrets)
            # 权限 600
            mode = stat.S_IMODE(os.stat(secrets).st_mode)
            self.assertEqual(mode, 0o600)
            # 内容写入且不回显到别处
            with open(secrets, "r", encoding="utf-8") as fh:
                self.assertEqual(fh.read().strip(), f"{ENV_NAME}=super-secret-key")
            self.assertEqual(os.environ.get(ENV_NAME), "super-secret-key")

    def test_empty_prompt_exits(self):
        with tempfile.TemporaryDirectory() as tmp:
            secrets = os.path.join(tmp, "secrets.env")
            with self.assertRaises(SystemExit):
                rebecca_start.ensure_api_key(secrets_path=secrets, prompt=lambda p: "")

    def test_symlink_secrets_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = os.path.join(tmp, "target.env")
            link = os.path.join(tmp, "secrets.env")
            Path(target).write_text(f"{ENV_NAME}=from-link\n")
            os.symlink(target, link)
            with self.assertRaises(SystemExit):
                rebecca_start.ensure_api_key(secrets_path=link)


class StaticCompileTests(unittest.TestCase):
    def test_wrapper_and_launcher_compile(self):
        # “shell/static 检查”：入口脚本与 FP16 启动器均可静态编译。
        py_compile.compile(
            str(PACKAGE_ROOT / "llm_voice_agent" / "rebecca_start.py"), doraise=True
        )
        py_compile.compile(
            str(PACKAGE_ROOT / "scripts" / "cosyvoice_fp16_server.py"), doraise=True
        )


if __name__ == "__main__":
    unittest.main()
