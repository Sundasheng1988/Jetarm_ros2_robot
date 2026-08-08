#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FP16 CosyVoice3 FastAPI 启动器，供 rebecca_voice_start 调用。

本脚本**不含任何 ROS 依赖**，必须由 ``cosyvoice`` conda 环境的 Python 解释器
执行（绝不是 ROS/系统 python3）。它复用官方 CosyVoice FastAPI 的 ``app``（含
``inference_zero_shot`` 等路由以及 server.py 里已打过的 zero-shot 路径补丁），
但用实测可用的 FP16 配置初始化模型：

    AutoModel(model_dir=..., load_trt=False, load_vllm=False, fp16=True)

默认值（均可被环境变量 / 命令行覆盖）：
    COSYVOICE_ROOT = /home/sundasheng/tools/CosyVoice
    model_dir      = $COSYVOICE_ROOT/pretrained_models/Fun-CosyVoice3-0.5B
    host / port    = 127.0.0.1 / 50000
"""
import argparse
import os
import site
import sys


DEFAULT_ROOT = "/home/sundasheng/tools/CosyVoice"
DEFAULT_MODEL_DIR = os.path.join(DEFAULT_ROOT, "pretrained_models", "Fun-CosyVoice3-0.5B")


def _remove_user_site_from_sys_path():
    """Defensively remove ~/.local site-packages before third-party imports.

    ``rebecca_voice_start`` already launches this script with ``-s`` and an
    isolated environment.  This second boundary also protects a direct manual
    invocation of the script.
    """
    user_sites = site.getusersitepackages()
    if isinstance(user_sites, str):
        user_sites = [user_sites]
    blocked = {os.path.realpath(path) for path in user_sites if path}
    sys.path[:] = [
        entry for entry in sys.path
        if os.path.realpath(entry or os.curdir) not in blocked
    ]


def _resolve_root(explicit=None):
    root = explicit or os.environ.get("COSYVOICE_ROOT", DEFAULT_ROOT)
    root = os.path.expanduser(root)
    if not os.path.isdir(root):
        raise SystemExit(f"[cosyvoice_fp16_server] CosyVoice 根目录不存在：{root}")
    return root


def main():
    parser = argparse.ArgumentParser(description="FP16 CosyVoice3 FastAPI 启动器")
    parser.add_argument("--model_dir", default=None)
    parser.add_argument("--port", type=int, default=50000)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--cosyvoice_root", default=None)
    args = parser.parse_args()

    if args.host.strip().lower() not in {"127.0.0.1", "localhost", "::1"}:
        raise SystemExit(
            "[cosyvoice_fp16_server] --host 仅允许 127.0.0.1/localhost；"
            "拒绝绑定外部网络。"
        )
    bind_host = "127.0.0.1"

    _remove_user_site_from_sys_path()

    root = _resolve_root(args.cosyvoice_root)

    # 让 cosyvoice 包与第三方 Matcha-TTS 可被 import（与官方 server.py 一致）。
    sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, "third_party", "Matcha-TTS"))

    # 复用官方 fastapi/server.py 的 app（含 inference_zero_shot 等路由）。
    fastapi_dir = os.path.join(root, "runtime", "python", "fastapi")
    sys.path.insert(0, fastapi_dir)
    try:
        import typing_extensions  # noqa: E402
        from typing_extensions import NoExtraItems  # noqa: E402, F401
    except ImportError as exc:
        loaded_module = sys.modules.get("typing_extensions")
        loaded_from = getattr(loaded_module, "__file__", "not importable")
        raise SystemExit(
            "[cosyvoice_fp16_server] cosyvoice 环境中的 typing_extensions "
            "缺少 NoExtraItems（需要 >= 4.13）。\n"
            f"当前加载：{loaded_from}\n"
            f"请执行：{sys.executable} "
            "-m pip install -U 'typing_extensions>=4.13'"
        ) from exc
    import server as cv_server  # noqa: E402  官方 server.py

    model_dir = (
        os.path.expanduser(args.model_dir)
        if args.model_dir
        else os.path.join(root, "pretrained_models", "Fun-CosyVoice3-0.5B")
    )
    if not os.path.isdir(model_dir):
        raise SystemExit(f"[cosyvoice_fp16_server] 模型目录不存在：{model_dir}")

    # 实测可用的 FP16 初始化（不要回退到未启用 FP16 的官方默认初始化）。
    # 复用 server 模块已 import 的 AutoModel，避免重复 import 路径差异。
    cv_server.cosyvoice = cv_server.AutoModel(
        model_dir=model_dir,
        load_trt=False,
        load_vllm=False,
        fp16=True,
    )

    import uvicorn  # noqa: E402

    print(
        f"[cosyvoice_fp16_server] FP16 模型已加载，启动 uvicorn "
        f"{bind_host}:{args.port}（model_dir={model_dir}）",
        flush=True,
    )
    uvicorn.run(cv_server.app, host=bind_host, port=args.port)


if __name__ == "__main__":
    main()
