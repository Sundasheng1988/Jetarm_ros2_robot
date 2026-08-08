#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动 Rebecca 语音栈：CosyVoice(FP16) + ros2 launch voice_stack。

入口（构建安装后）::

    ros2 run llm_voice_agent rebecca_voice_start [ros2 launch 参数...]

行为：
  * 首次运行若没有 ZHIPUAI_API_KEY，隐藏输入并写入
    ``~/.config/jetarm_voice/secrets.env``（权限 600），后续自动加载；
    密钥绝不写入源码/launch/yaml/日志/命令行参数。
  * 探测 127.0.0.1:50000：已是 CosyVoice 则复用（不拥有、Ctrl+C 不停止）；
    端口被其它服务占用则报错退出；未启动则用 cosyvoice 环境的 Python 以 FP16
    启动并记录 PID，等健康检查通过再启动 voice_stack。
  * Ctrl+C：只停止本入口启动的 ROS Launch 与本入口自己启动的 CosyVoice，
    绝不误杀已存在的 CosyVoice 或其它 Python 进程。

环境变量（均有默认值）：
    COSYVOICE_PYTHON          cosyvoice conda env 的 python 绝对路径
    COSYVOICE_MODEL_DIR       Fun-CosyVoice3 模型目录
    COSYVOICE_ROOT            CosyVoice 仓库根目录
    COSYVOICE_HOST/PORT       host 仅允许 localhost；端口默认 50000
    COSYVOICE_START_TIMEOUT   等待模型加载秒数，默认 240
"""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

from llm_voice_agent.runtime_control import (
    ProcessStopError,
    stop_owned_process_group,
)


SECRETS_PATH = os.path.expanduser("~/.config/jetarm_voice/secrets.env")
DEFAULT_COSYVOICE_PYTHON = "/home/sundasheng/miniforge3/envs/cosyvoice/bin/python3"
DEFAULT_MODEL_DIR = "/home/sundasheng/tools/CosyVoice/pretrained_models/Fun-CosyVoice3-0.5B"
DEFAULT_ROOT = "/home/sundasheng/tools/CosyVoice"


# ============================ secrets ============================

def _parse_env_file(path):
    result = {}
    with open(path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip().strip('"').strip("'")
            if key:
                result[key] = val
    return result


def ensure_api_key(env_name="ZHIPUAI_API_KEY", secrets_path=None, prompt=None):
    """保证 os.environ 中存在 API key。

    返回 (来源, 路径_or_None)：来源为 'env' / 'file' / 'prompted'。
    密钥值绝不打印、不进命令行参数。首次写入文件权限强制 600。
    """
    secrets_path = os.path.expanduser(secrets_path or SECRETS_PATH)
    secrets_dir = os.path.dirname(secrets_path) or "."

    if os.environ.get(env_name, "").strip():
        return ("env", None)

    if os.path.islink(secrets_path):
        raise SystemExit("[rebecca_start] secrets.env 不能是符号链接，退出。")

    if os.path.isfile(secrets_path):
        # Correct overly broad permissions left by a manual file creation.
        os.chmod(secrets_path, 0o600)
        data = _parse_env_file(secrets_path)
        for key, val in data.items():
            os.environ.setdefault(key, val)
        if os.environ.get(env_name, "").strip():
            return ("file", secrets_path)

    # 首次：隐藏输入
    import getpass
    if prompt is None:
        prompt_func = lambda p: getpass.getpass(p)
    else:
        prompt_func = prompt
    val = prompt_func(f"未发现 {env_name}，请输入（不回显）: ").strip()
    if not val:
        raise SystemExit(f"[rebecca_start] 未输入 {env_name}，退出。")

    os.makedirs(secrets_dir, mode=0o700, exist_ok=True)
    os.chmod(secrets_dir, 0o700)
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(secrets_path, flags, 0o600)
    except OSError as exc:
        raise SystemExit(
            f"[rebecca_start] 无法安全写入 secrets.env：{type(exc).__name__}"
        ) from exc
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(f"{env_name}={val}\n")
    os.chmod(secrets_path, 0o600)
    os.environ[env_name] = val
    return ("prompted", secrets_path)


# ============================ cosyvoice ============================

def normalize_local_host(host=None):
    """Accept localhost spellings but always bind/probe IPv4 loopback."""
    value = (host or "127.0.0.1").strip().lower()
    if value not in {"127.0.0.1", "localhost", "::1"}:
        raise SystemExit(
            "[rebecca_start] COSYVOICE_HOST 仅允许 127.0.0.1/localhost；"
            "拒绝将语音服务暴露到外部网络。"
        )
    return "127.0.0.1"


def _tcp_port_open(host, port, timeout):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def find_cosyvoice_python():
    """返回 cosyvoice conda env 的 Python 绝对路径；找不到则报错退出。

    永不静默回退到系统 python3。
    """
    py = os.environ.get("COSYVOICE_PYTHON", "").strip()
    if not py:
        py = DEFAULT_COSYVOICE_PYTHON
    py = os.path.expanduser(py)
    if not (os.path.isfile(py) and os.access(py, os.X_OK)):
        raise SystemExit(
            f"[rebecca_start] 未找到 cosyvoice 环境的 Python 解释器：{py}\n"
            "请用环境变量 COSYVOICE_PYTHON 指向 conda env 的 python；"
            "不允许回退到系统 python3。"
        )
    return py


def cosyvoice_share_script():
    """定位安装到 share 的 cosyvoice_fp16_server.py。"""
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception as exc:  # 未 source ROS 环境
        raise SystemExit(f"[rebecca_start] 无法定位包共享目录（未 source ROS 环境？）：{exc}")
    share = get_package_share_directory("llm_voice_agent")
    script = os.path.join(share, "scripts", "cosyvoice_fp16_server.py")
    if not os.path.isfile(script):
        raise SystemExit(
            f"[rebecca_start] 未找到 CosyVoice 启动脚本：{script}\n"
            "请重新 colcon build --packages-select llm_voice_agent。"
        )
    return script


def probe_cosyvoice(host="127.0.0.1", port=50000, timeout=2.0):
    """探测端口上的服务。

    返回 (state, info)：
      'cosyvoice' — 200 且含 inference_zero_shot 路由；
      'other'     — 有响应但不是 CosyVoice（端口被占）；
      'down'      — 连不上（未启动）。
    """
    host = normalize_local_host(host)
    url = f"http://{host}:{port}/openapi.json"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", "replace")
    except urllib.error.URLError as exc:
        if _tcp_port_open(host, port, timeout):
            return ("other", f"TCP port open but OpenAPI probe failed: {type(exc).__name__}")
        return ("down", str(exc.reason))
    except Exception as exc:  # noqa: BLE001
        if _tcp_port_open(host, port, timeout):
            return ("other", f"TCP port open but OpenAPI probe failed: {type(exc).__name__}")
        return ("down", f"{type(exc).__name__}: {exc}")
    try:
        data = json.loads(body)
    except ValueError:
        return ("other", "non-json response")
    paths = (data.get("paths") or {}) if isinstance(data, dict) else {}
    if "/inference_zero_shot" in paths or "inference_zero_shot" in body:
        return ("cosyvoice", (data.get("info") if isinstance(data, dict) else {}))
    return ("other", "no inference_zero_shot endpoint")


def plan_cosyvoice_action(state):
    """纯函数：把探测结果映射为 adopt / start / error，便于测试。"""
    if state == "cosyvoice":
        return "adopt"
    if state == "down":
        return "start"
    return "error"


def start_cosyvoice(python, script, model_dir, port, host):
    host = normalize_local_host(host)
    root = os.environ.get("COSYVOICE_ROOT", DEFAULT_ROOT)
    # CosyVoice must not inherit ROS/user Python import paths.  In particular,
    # a package under ~/.local/lib/pythonX.Y/site-packages can otherwise shadow
    # the version installed in the dedicated cosyvoice conda environment.
    # Keep non-Python variables (CUDA, proxy settings, model paths, etc.).
    child_env = os.environ.copy()
    child_env["PYTHONNOUSERSITE"] = "1"
    child_env.pop("PYTHONPATH", None)
    child_env.pop("PYTHONHOME", None)
    cmd = [
        python, "-E", "-s", script,
        "--model_dir", model_dir,
        "--port", str(port),
        "--host", host,
        "--cosyvoice_root", root,
    ]
    return subprocess.Popen(cmd, start_new_session=True, env=child_env)


def wait_health(
    host,
    port,
    timeout=240,
    interval=2.0,
    sleep=time.sleep,
    clock=time.monotonic,
    process=None,
):
    host = normalize_local_host(host)
    deadline = clock() + timeout
    last = None
    while clock() < deadline:
        if process is not None:
            rc = process.poll()
            if rc is not None:
                print(
                    f"[rebecca_start] CosyVoice 进程提前退出（rc={rc}）。",
                    flush=True,
                )
                return False
        state, info = probe_cosyvoice(host, port, timeout=2.0)
        if state == "cosyvoice":
            return True
        if state == "other":
            print(f"[rebecca_start] 端口已被非 CosyVoice 服务占用：{info}", flush=True)
            return False
        last = info
        sleep(interval)
    print(f"[rebecca_start] CosyVoice 健康检查超时（last={last}）。", flush=True)
    return False


# ============================ process cleanup ============================

def terminate_process_group(proc, name, grace=10):
    """优雅→强制停止一个本入口拥有的子进程（及其进程组）。

    proc 为 None（复用/未拥有）时直接返回，绝不触碰它。
    """
    try:
        result = stop_owned_process_group(
            proc,
            term_timeout=grace,
            kill_timeout=5,
        )
    except ProcessStopError as exc:
        print(f"[rebecca_start] {name} 停止失败：{exc}", flush=True)
        return False
    for warning in result.warnings:
        print(f"[rebecca_start] {name} 停止降级：{warning}", flush=True)
    if not result.stopped:
        return False
    suffix = "已强制结束" if result.forced else "已停止"
    print(f"[rebecca_start] {name} {suffix}。", flush=True)
    return True


# ============================ main ============================

def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="rebecca_voice_start",
        description="一键启动 CosyVoice(FP16) + voice_stack",
    )
    parser.add_argument(
        "launch_args",
        nargs=argparse.REMAINDER,
        help="透传给 ros2 launch llm_voice_agent voice_stack.launch.py 的参数",
    )
    return parser.parse_args(argv)


def _install_term_handler():
    def _handler(signum, frame):  # noqa: ARG001
        raise KeyboardInterrupt
    try:
        signal.signal(signal.SIGTERM, _handler)
    except (ValueError, OSError):
        pass  # 非主线程


def run(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    _install_term_handler()

    # 1) API key
    source, _ = ensure_api_key()
    print(f"[rebecca_start] ZHIPUAI_API_KEY 已就绪（来源：{source}）。", flush=True)

    host = normalize_local_host(os.environ.get("COSYVOICE_HOST", "127.0.0.1"))
    port = int(os.environ.get("COSYVOICE_PORT", "50000"))
    if not 1 <= port <= 65535:
        raise SystemExit("[rebecca_start] COSYVOICE_PORT 必须在 1..65535。")

    cosy_proc = None  # None 代表“不拥有”（复用），Ctrl+C 不停止
    reused_cosyvoice = False
    launch_proc = None
    try:
        # 2) CosyVoice：复用 / 启动 / 报错
        state, info = probe_cosyvoice(host, port)
        action = plan_cosyvoice_action(state)
        if action == "adopt":
            reused_cosyvoice = True
            print(
                f"[rebecca_start] 检测到已运行的 CosyVoice（{host}:{port}），"
                "复用，不拥有、Ctrl+C 不会停止它。",
                flush=True,
            )
        elif action == "error":
            raise SystemExit(
                f"[rebecca_start] {host}:{port} 已被非 CosyVoice 服务占用（{info}），退出。"
            )
        else:  # start
            python = find_cosyvoice_python()
            script = cosyvoice_share_script()
            model_dir = os.environ.get("COSYVOICE_MODEL_DIR", DEFAULT_MODEL_DIR)
            cosy_proc = start_cosyvoice(python, script, model_dir, port, host)
            print(
                f"[rebecca_start] 启动 CosyVoice（PID={cosy_proc.pid}，FP16），等待模型加载…",
                flush=True,
            )
            timeout = int(os.environ.get("COSYVOICE_START_TIMEOUT", "240"))
            if not wait_health(host, port, timeout=timeout, process=cosy_proc):
                raise SystemExit("[rebecca_start] CosyVoice 健康检查失败，退出。")

        # 3) voice_stack
        launch_cmd = ["ros2", "launch", "llm_voice_agent", "voice_stack.launch.py"]
        launch_cmd += [a for a in args.launch_args if a != "--"]
        launch_proc = subprocess.Popen(launch_cmd, start_new_session=True)
        print(f"[rebecca_start] 启动 voice_stack（PID={launch_proc.pid}）。", flush=True)

        # 4) 等待 / 信号 → 清理
        rc = launch_proc.wait()
        print(f"[rebecca_start] voice_stack 已退出，rc={rc}。", flush=True)
    except KeyboardInterrupt:
        print("\n[rebecca_start] 收到中断信号，开始清理…", flush=True)
    finally:
        if launch_proc is not None:
            terminate_process_group(launch_proc, "voice_stack", grace=10)
        if cosy_proc is not None:
            terminate_process_group(cosy_proc, "CosyVoice", grace=10)
        elif reused_cosyvoice:
            print("[rebecca_start] CosyVoice 为复用进程，不停止。", flush=True)


def main():
    run()


if __name__ == "__main__":
    main()
