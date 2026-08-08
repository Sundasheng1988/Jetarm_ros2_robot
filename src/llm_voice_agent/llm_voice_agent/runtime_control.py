#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Thread-safe runtime controls shared by the voice nodes.

This module intentionally has no ROS dependency.  The cancellation, queue and
process-shutdown behaviour can therefore be exercised without starting a ROS
graph, an audio player, CosyVoice, or any robot component.
"""

from __future__ import annotations

import os
import queue
import signal
import subprocess
import threading
from dataclasses import dataclass
from typing import Callable, Generic, Optional, Tuple, TypeVar


T = TypeVar("T")


class CancellationTokenSlot:
    """Own the current LLM cancellation token without ever reusing an Event."""

    def __init__(self):
        self._lock = threading.Lock()
        self._current: Optional[threading.Event] = None

    def begin(self) -> threading.Event:
        """Install a fresh token and permanently cancel any older live token."""
        token = threading.Event()
        with self._lock:
            previous = self._current
            if previous is not None:
                previous.set()
            self._current = token
        return token

    def cancel_current(self) -> bool:
        """Cancel the current token, returning whether one existed."""
        with self._lock:
            token = self._current
            if token is not None:
                token.set()
        return token is not None

    def release(self, token: threading.Event) -> bool:
        """Clear *token* only when it is still the current request's token."""
        with self._lock:
            if self._current is not token:
                return False
            self._current = None
            return True

    @property
    def current(self) -> Optional[threading.Event]:
        with self._lock:
            return self._current


class GenerationQueue(Generic[T]):
    """Queue items with the playback generation in which they were created."""

    def __init__(self, maxsize: int):
        self._queue: queue.Queue[Tuple[int, T]] = queue.Queue(maxsize=maxsize)
        self._lock = threading.Lock()
        self._generation = 0

    def put_nowait(self, item: T) -> int:
        """Atomically tag and enqueue *item* with the current generation."""
        with self._lock:
            generation = self._generation
            self._queue.put_nowait((generation, item))
            return generation

    def put_nowait_if_current(self, generation: int, item: T) -> bool:
        """Enqueue only if an interrupt has not invalidated the producer."""
        with self._lock:
            if generation != self._generation:
                return False
            self._queue.put_nowait((generation, item))
            return True

    def get(self, timeout: Optional[float] = None) -> Tuple[int, T]:
        return self._queue.get(timeout=timeout)

    def task_done(self) -> None:
        self._queue.task_done()

    def empty(self) -> bool:
        return self._queue.empty()

    def is_current(self, generation: int) -> bool:
        with self._lock:
            return generation == self._generation

    @property
    def generation(self) -> int:
        with self._lock:
            return self._generation

    def interrupt(self) -> Tuple[int, int]:
        """Advance generation and remove every queued item from older answers."""
        with self._lock:
            self._generation += 1
            generation = self._generation
            dropped = 0
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
                else:
                    self._queue.task_done()
                    dropped += 1
            return generation, dropped


@dataclass(frozen=True)
class ProcessStopResult:
    """Outcome of stopping one explicitly owned process group."""

    stopped: bool
    forced: bool = False
    warnings: Tuple[str, ...] = ()


class ProcessStopError(RuntimeError):
    """An owned child process did not stop after TERM and KILL."""


def stop_owned_process_group(
    proc,
    *,
    term_timeout: float = 0.5,
    kill_timeout: float = 0.5,
    getpgid: Callable[[int], int] = os.getpgid,
    killpg: Callable[[int, int], None] = os.killpg,
) -> ProcessStopResult:
    """Stop only ``proc`` and the process group created for it.

    The caller must pass a child it owns and started in a new session.  SIGTERM
    is followed by a bounded wait; SIGKILL and a second bounded wait are used
    only when required.  Process-group lookup/signal failures are reported in
    the result and fall back to ``proc.terminate`` / ``proc.kill``.
    """
    if proc is None or proc.poll() is not None:
        return ProcessStopResult(stopped=False)

    warnings = []
    pgid = None
    try:
        pgid = getpgid(proc.pid)
    except (OSError, AttributeError) as exc:
        warnings.append(f"process-group lookup failed: {type(exc).__name__}")

    def send(sig: int) -> None:
        nonlocal pgid
        if pgid is not None:
            try:
                killpg(pgid, sig)
                return
            except ProcessLookupError:
                return
            except OSError as exc:
                warnings.append(
                    f"process-group signal {sig} failed: {type(exc).__name__}"
                )
                pgid = None
        try:
            if sig == signal.SIGTERM:
                proc.terminate()
            else:
                proc.kill()
        except ProcessLookupError:
            return
        except (OSError, AttributeError) as exc:
            warnings.append(f"process signal {sig} failed: {type(exc).__name__}")

    send(signal.SIGTERM)
    try:
        proc.wait(timeout=term_timeout)
        return ProcessStopResult(stopped=True, warnings=tuple(warnings))
    except subprocess.TimeoutExpired:
        send(signal.SIGKILL)

    try:
        proc.wait(timeout=kill_timeout)
    except subprocess.TimeoutExpired as exc:
        raise ProcessStopError("owned process did not exit after SIGKILL") from exc
    return ProcessStopResult(stopped=True, forced=True, warnings=tuple(warnings))
