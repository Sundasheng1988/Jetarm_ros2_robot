#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import queue
import signal
import subprocess
import unittest

from llm_voice_agent.runtime_control import (
    CancellationTokenSlot,
    GenerationQueue,
    ProcessStopError,
    stop_owned_process_group,
)


class CancellationTokenSlotTests(unittest.TestCase):
    def test_cancel_and_release_current_token(self):
        slot = CancellationTokenSlot()
        token = slot.begin()

        self.assertTrue(slot.cancel_current())
        self.assertTrue(token.is_set())
        self.assertTrue(slot.release(token))
        self.assertIsNone(slot.current)
        self.assertFalse(slot.cancel_current())

    def test_new_request_never_clears_or_restores_old_token(self):
        slot = CancellationTokenSlot()
        old = slot.begin()
        old.set()

        new = slot.begin()

        self.assertTrue(old.is_set())
        self.assertFalse(new.is_set())
        self.assertIs(slot.current, new)

    def test_old_finally_cannot_clear_newer_token(self):
        slot = CancellationTokenSlot()
        old = slot.begin()
        new = slot.begin()

        self.assertTrue(old.is_set())
        self.assertFalse(slot.release(old))
        self.assertIs(slot.current, new)
        self.assertTrue(slot.release(new))
        self.assertIsNone(slot.current)


class GenerationQueueTests(unittest.TestCase):
    def test_interrupt_invalidates_dequeued_item_and_clears_old_queue(self):
        items = GenerationQueue[str](maxsize=4)
        old_generation = items.put_nowait("playing")
        items.put_nowait("queued")
        dequeued_generation, dequeued = items.get(timeout=0.01)
        self.assertEqual(dequeued, "playing")

        new_generation, dropped = items.interrupt()

        self.assertEqual(dropped, 1)
        self.assertNotEqual(new_generation, old_generation)
        self.assertFalse(items.is_current(dequeued_generation))
        self.assertTrue(items.empty())
        items.task_done()

    def test_post_interrupt_item_uses_only_new_generation(self):
        items = GenerationQueue[str](maxsize=2)
        old_generation = items.put_nowait("old")
        items.interrupt()
        new_generation = items.put_nowait("new")

        self.assertFalse(items.is_current(old_generation))
        self.assertTrue(items.is_current(new_generation))
        generation, value = items.get(timeout=0.01)
        self.assertEqual((generation, value), (new_generation, "new"))
        items.task_done()
        with self.assertRaises(queue.Empty):
            items.get(timeout=0.01)

    def test_old_reply_cannot_enqueue_more_segments_after_interrupt(self):
        items = GenerationQueue[str](maxsize=3)
        old_generation = items.generation
        self.assertTrue(
            items.put_nowait_if_current(old_generation, "first old segment")
        )
        items.interrupt()

        self.assertFalse(
            items.put_nowait_if_current(old_generation, "late old segment")
        )
        self.assertTrue(items.empty())


class _FakeProcess:
    def __init__(self, waits, pid=1234, poll_result=None):
        self.pid = pid
        self._waits = list(waits)
        self._poll_result = poll_result
        self.wait_timeouts = []
        self.terminate_calls = 0
        self.kill_calls = 0

    def poll(self):
        return self._poll_result

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        result = self._waits.pop(0)
        if isinstance(result, BaseException):
            raise result
        self._poll_result = result
        return result

    def terminate(self):
        self.terminate_calls += 1

    def kill(self):
        self.kill_calls += 1


class OwnedProcessStopTests(unittest.TestCase):
    def test_term_then_wait_exits_without_kill(self):
        proc = _FakeProcess([0])
        signals = []
        result = stop_owned_process_group(
            proc,
            term_timeout=0.2,
            kill_timeout=0.3,
            getpgid=lambda pid: 777 if pid == proc.pid else None,
            killpg=lambda pgid, sig: signals.append((pgid, sig)),
        )

        self.assertTrue(result.stopped)
        self.assertFalse(result.forced)
        self.assertEqual(signals, [(777, signal.SIGTERM)])
        self.assertEqual(proc.wait_timeouts, [0.2])

    def test_term_timeout_uses_kill_then_waits_again(self):
        proc = _FakeProcess([
            subprocess.TimeoutExpired("player", 0.2),
            0,
        ])
        signals = []
        result = stop_owned_process_group(
            proc,
            term_timeout=0.2,
            kill_timeout=0.3,
            getpgid=lambda pid: 888,
            killpg=lambda pgid, sig: signals.append((pgid, sig)),
        )

        self.assertTrue(result.stopped)
        self.assertTrue(result.forced)
        self.assertEqual(
            signals,
            [(888, signal.SIGTERM), (888, signal.SIGKILL)],
        )
        self.assertEqual(proc.wait_timeouts, [0.2, 0.3])

    def test_group_failure_falls_back_to_exact_child(self):
        proc = _FakeProcess([0], pid=4321)

        def fail_group(pgid, sig):
            raise PermissionError("denied")

        result = stop_owned_process_group(
            proc,
            getpgid=lambda pid: 999,
            killpg=fail_group,
        )

        self.assertTrue(result.stopped)
        self.assertEqual(proc.terminate_calls, 1)
        self.assertEqual(proc.kill_calls, 0)
        self.assertTrue(result.warnings)

    def test_none_and_dead_processes_are_noops(self):
        self.assertFalse(stop_owned_process_group(None).stopped)
        dead = _FakeProcess([], poll_result=0)
        self.assertFalse(stop_owned_process_group(dead).stopped)

    def test_process_still_alive_after_kill_raises(self):
        proc = _FakeProcess([
            subprocess.TimeoutExpired("player", 0.1),
            subprocess.TimeoutExpired("player", 0.1),
        ])
        with self.assertRaises(ProcessStopError):
            stop_owned_process_group(
                proc,
                term_timeout=0.1,
                kill_timeout=0.1,
                getpgid=lambda pid: 555,
                killpg=lambda pgid, sig: None,
            )


if __name__ == "__main__":
    unittest.main()
