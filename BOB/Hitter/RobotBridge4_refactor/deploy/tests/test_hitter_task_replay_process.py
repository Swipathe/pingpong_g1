from __future__ import annotations

import json
import multiprocessing
import os
from pathlib import Path
import queue
import signal
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

from diagnostics.hitter_task_replay import (
    ReplayJobRef,
    ReplayProcessController,
)


class _Recorder:
    def __init__(self):
        self.job_statuses = []
        self.analyses = []
        self.writer_pids = []
        self.persistence_confirmations = []
        self.incomplete_calls = []

    def offer_replay_job_status(self, value):
        self.writer_pids.append(os.getpid())
        self.job_statuses.append(dict(value))
        return True

    def offer_replay_analysis(self, value):
        self.writer_pids.append(os.getpid())
        self.analyses.append(dict(value))
        return True

    def confirm_replay_persistence(self, timeout_s):
        self.persistence_confirmations.append(float(timeout_s))
        return SimpleNamespace(
            healthy=True,
            recording_complete=True,
            last_error=None,
        )

    def mark_incomplete(self, attempt_id, reason):
        self.incomplete_calls.append((attempt_id, reason))


class _EventSink:
    def __init__(self):
        self.drafts = []
        self.writer_pids = []
        self.drain_calls = []

    def publish(self, draft):
        self.writer_pids.append(os.getpid())
        self.drafts.append(draft)
        return draft

    def drain(self, *, timeout_s):
        self.drain_calls.append(float(timeout_s))
        return True


class _GateWorker:
    def __init__(self, starts, release):
        self.starts = starts
        self.release = release
        self.run = 0

    def __call__(self, job, checkpoint):
        self.run += 1
        self.starts.put((job.attempt_id, self.run))
        while not self.release.wait(0.01):
            checkpoint()
        checkpoint()
        return {
            "attempt_id": job.attempt_id,
            "run": self.run,
            "source": json.loads(job.input_path.read_text())["source"],
        }


class _UncooperativeWorker:
    def __init__(self, starts, release_reader):
        self.starts = starts
        self.release_reader = release_reader

    def __call__(self, job, checkpoint):
        self.starts.put((job.attempt_id, os.getpid()))
        self.release_reader.recv()
        checkpoint()
        return {
            "attempt_id": job.attempt_id,
            "source": "restarted-from-disk",
        }


class _DelayedTerminateWorker:
    def __init__(self, started):
        self.started = started

    def __call__(self, job, checkpoint):
        del job, checkpoint

        def delayed_exit(_signum, _frame):
            time.sleep(0.1)
            os._exit(0)

        signal.signal(signal.SIGTERM, delayed_exit)
        self.started.set()
        while True:
            time.sleep(1.0)


class _IgnoreTerminateWorker:
    def __init__(self, started):
        self.started = started

    def __call__(self, job, checkpoint):
        del job, checkpoint
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        self.started.set()
        while True:
            time.sleep(1.0)


def _failing_worker(job, checkpoint):
    checkpoint()
    raise RuntimeError("scripted child failure {}".format(job.attempt_id))


def _successful_worker(job, checkpoint):
    checkpoint()
    return {"attempt_id": job.attempt_id, "status": "ok"}


def _exit_23_then_succeed_worker(job, checkpoint):
    checkpoint()
    if job.attempt_id == 23:
        os._exit(23)
    return {"attempt_id": job.attempt_id, "status": "restarted"}


class ReplayProcessControllerTest(unittest.TestCase):
    def _drain_until(self, controller, terminal_status, timeout_s=3.0):
        replies = []
        remaining = timeout_s
        while remaining > 0.0:
            controller.wait_for_reply(timeout_s=min(remaining, 0.25))
            batch = controller.poll()
            replies.extend(batch)
            if any(reply.status == terminal_status for reply in replies):
                return tuple(replies)
            remaining -= 0.25
        self.fail(
            "did not receive {!r}; got {}".format(
                terminal_status,
                [reply.status for reply in replies],
            )
        )

    def test_spawn_pauses_busy_replay_and_restarts_same_disk_job(self):
        context = multiprocessing.get_context("spawn")
        starts = context.Queue()
        release = context.Event()
        recorder = _Recorder()
        events = _EventSink()
        parent_pid = os.getpid()
        with tempfile.TemporaryDirectory() as temp_dir:
            input_path = Path(temp_dir) / "attempt-7.json"
            input_path.write_text('{"source":"disk"}\n')
            controller = ReplayProcessController(
                worker_fn=_GateWorker(starts, release),
                recorder=recorder,
                event_sink=events,
                mp_context=context,
            )
            try:
                controller.set_realtime_busy(
                    attempt_active=True,
                    reacquire_grace_active=False,
                )
                controller.submit_disk_job(ReplayJobRef("session", 7, input_path))
                with self.assertRaises(queue.Empty):
                    starts.get(timeout=0.1)

                controller.set_realtime_busy(
                    attempt_active=False,
                    reacquire_grace_active=False,
                )
                self.assertEqual(starts.get(timeout=2.0), (7, 1))
                controller.set_realtime_busy(
                    attempt_active=True,
                    reacquire_grace_active=False,
                )
                paused = controller.poll()
                self.assertTrue(
                    any(reply.status == "PAUSED" for reply in paused)
                    or any(item["status"] == "PAUSED" for item in recorder.job_statuses)
                )

                release.set()
                controller.set_realtime_busy(
                    attempt_active=False,
                    reacquire_grace_active=False,
                )
                self.assertEqual(starts.get(timeout=2.0), (7, 2))
                replies = self._drain_until(controller, "COMPLETED")
                completed = next(reply for reply in replies if reply.status == "COMPLETED")
                self.assertEqual(completed.result["run"], 2)
                self.assertEqual(completed.result["source"], "disk")
                self.assertEqual(
                    [item["status"] for item in recorder.job_statuses if item["attempt_id"] == 7],
                    [
                        "QUEUED",
                        "RUNNING",
                        "PAUSED",
                        "RUNNING",
                        "COMPLETED",
                    ],
                )
                self.assertEqual(len(recorder.analyses), 1)
                self.assertTrue(all(pid == parent_pid for pid in recorder.writer_pids))
                self.assertTrue(all(pid == parent_pid for pid in events.writer_pids))
                self.assertEqual(
                    len(events.drain_calls),
                    len(events.drafts),
                )
            finally:
                self.assertTrue(controller.close(timeout_s=5.0))

    def test_reacquire_keeps_job_queued_and_child_failure_is_a_reply(self):
        context = multiprocessing.get_context("spawn")
        recorder = _Recorder()
        with tempfile.TemporaryDirectory() as temp_dir:
            input_path = Path(temp_dir) / "attempt-9.json"
            input_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_failing_worker,
                recorder=recorder,
                mp_context=context,
            )
            try:
                controller.set_realtime_busy(
                    attempt_active=False,
                    reacquire_grace_active=True,
                )
                controller.submit_disk_job(ReplayJobRef("session", 9, input_path))
                self.assertFalse(controller.wait_for_reply(timeout_s=0.1))
                controller.set_realtime_busy(
                    attempt_active=False,
                    reacquire_grace_active=False,
                )
                replies = self._drain_until(controller, "FAILED")
                failure = next(reply for reply in replies if reply.status == "FAILED")
                self.assertIn("scripted child failure 9", failure.error)
                self.assertIsNone(failure.result)
                self.assertEqual(recorder.analyses, [])
            finally:
                self.assertTrue(controller.close(timeout_s=5.0))

    def test_pending_disk_window_is_bounded_before_queueing(self):
        context = multiprocessing.get_context("spawn")
        recorder = _Recorder()
        with tempfile.TemporaryDirectory() as temp_dir:
            first_path = Path(temp_dir) / "attempt-1.json"
            second_path = Path(temp_dir) / "attempt-2.json"
            first_path.write_text("{}\n")
            second_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_failing_worker,
                recorder=recorder,
                mp_context=context,
                pending_capacity=1,
            )
            try:
                controller.set_realtime_busy(
                    attempt_active=True,
                    reacquire_grace_active=False,
                )
                controller.submit_disk_job(ReplayJobRef("session", 1, first_path))
                with self.assertRaisesRegex(
                    OverflowError,
                    "pending window",
                ):
                    controller.submit_disk_job(ReplayJobRef("session", 2, second_path))
                self.assertEqual(
                    [item["attempt_id"] for item in recorder.job_statuses if item["status"] == "QUEUED"],
                    [1],
                )
            finally:
                self.assertTrue(controller.close(timeout_s=5.0))

    def test_pause_timeout_keeps_pause_and_restarts_job_next_idle(self):
        context = multiprocessing.get_context("spawn")
        starts = context.Queue()
        release_reader, release_writer = context.Pipe(duplex=False)
        recorder = _Recorder()
        with tempfile.TemporaryDirectory() as temp_dir:
            input_path = Path(temp_dir) / "attempt-11.json"
            input_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_UncooperativeWorker(starts, release_reader),
                recorder=recorder,
                mp_context=context,
                pause_timeout_s=0.05,
            )
            try:
                controller.submit_disk_job(ReplayJobRef("session", 11, input_path))
                first_attempt, _first_pid = starts.get(timeout=2.0)
                self.assertEqual(first_attempt, 11)

                controller.set_realtime_busy(
                    attempt_active=True,
                    reacquire_grace_active=False,
                )
                self.assertEqual(recorder.analyses, [])
                self.assertFalse(any(item["status"] == "COMPLETED" for item in recorder.job_statuses))

                controller.set_realtime_busy(
                    attempt_active=False,
                    reacquire_grace_active=False,
                )
                release_writer.send(True)
                self._drain_until(controller, "PAUSED")
                second_attempt, _second_pid = starts.get(timeout=2.0)
                self.assertEqual(second_attempt, 11)
                release_writer.send(True)
                self._drain_until(controller, "COMPLETED")
                self.assertEqual(len(recorder.analyses), 1)
                self.assertEqual(
                    [item["status"] for item in recorder.job_statuses if item["attempt_id"] == 11],
                    [
                        "QUEUED",
                        "RUNNING",
                        "PAUSED",
                        "RUNNING",
                        "COMPLETED",
                    ],
                )
            finally:
                self.assertTrue(controller.close(timeout_s=5.0))
                release_reader.close()
                release_writer.close()

    def test_completed_status_rejection_fails_closed_and_runs_next_job(self):
        class RejectFirstCompletedRecorder(_Recorder):
            def offer_replay_job_status(self, value):
                if value["attempt_id"] == 1 and value["status"] == "COMPLETED":
                    return False
                return super().offer_replay_job_status(value)

        context = multiprocessing.get_context("spawn")
        recorder = RejectFirstCompletedRecorder()
        with tempfile.TemporaryDirectory() as temp_dir:
            first_path = Path(temp_dir) / "attempt-1.json"
            second_path = Path(temp_dir) / "attempt-2.json"
            first_path.write_text("{}\n")
            second_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_successful_worker,
                recorder=recorder,
                mp_context=context,
            )
            try:
                controller.submit_disk_job(ReplayJobRef("session", 1, first_path))
                controller.submit_disk_job(ReplayJobRef("session", 2, second_path))
                replies = []
                remaining = 4.0
                while remaining > 0.0:
                    controller.wait_for_reply(timeout_s=0.1)
                    replies.extend(controller.poll())
                    if any(reply.attempt_id == 2 and reply.status == "COMPLETED" for reply in replies):
                        break
                    remaining -= 0.1

                first_terminal = [
                    reply for reply in replies if reply.attempt_id == 1 and reply.status in ("COMPLETED", "FAILED")
                ]
                self.assertTrue(first_terminal)
                self.assertEqual(first_terminal[-1].status, "FAILED")
                self.assertIn("recorder rejected", first_terminal[-1].error)
                self.assertTrue(any(reply.attempt_id == 2 and reply.status == "COMPLETED" for reply in replies))
            finally:
                self.assertTrue(controller.close(timeout_s=5.0))

    def test_completed_reply_waits_for_durable_recorder_confirmation(self):
        class AnalysisWriteFailureRecorder(_Recorder):
            def confirm_replay_persistence(self, timeout_s):
                self.persistence_confirmations.append(float(timeout_s))
                failed = bool(self.analyses)
                return SimpleNamespace(
                    healthy=not failed,
                    recording_complete=not failed,
                    last_error=("OSError: synthetic replay analysis disk failure" if failed else None),
                )

        context = multiprocessing.get_context("spawn")
        recorder = AnalysisWriteFailureRecorder()
        with tempfile.TemporaryDirectory() as temp_dir:
            input_path = Path(temp_dir) / "attempt-17.json"
            input_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_successful_worker,
                recorder=recorder,
                mp_context=context,
                persistence_timeout_s=0.5,
            )
            try:
                controller.submit_disk_job(ReplayJobRef("session", 17, input_path))
                replies = self._drain_until(controller, "FAILED")
                terminal = [
                    reply for reply in replies if reply.attempt_id == 17 and reply.status in ("COMPLETED", "FAILED")
                ]
                self.assertEqual(terminal[-1].status, "FAILED")
                self.assertIn(
                    "REPLAY_PERSISTENCE_ERROR",
                    terminal[-1].error,
                )
                self.assertNotIn(
                    "COMPLETED",
                    [reply.status for reply in terminal],
                )
                self.assertTrue(recorder.analyses)
                self.assertIn(
                    (17, "REPLAY_PERSISTENCE_ERROR"),
                    recorder.incomplete_calls,
                )
                self.assertGreaterEqual(
                    len(recorder.persistence_confirmations),
                    3,
                )
            finally:
                self.assertTrue(controller.close(timeout_s=5.0))

    def test_unexpected_child_exit_fails_current_and_restarts_pending(self):
        context = multiprocessing.get_context("spawn")
        recorder = _Recorder()
        with tempfile.TemporaryDirectory() as temp_dir:
            crash_path = Path(temp_dir) / "attempt-23.json"
            next_path = Path(temp_dir) / "attempt-24.json"
            crash_path.write_text("{}\n")
            next_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_exit_23_then_succeed_worker,
                recorder=recorder,
                mp_context=context,
            )
            try:
                controller.submit_disk_job(ReplayJobRef("session", 23, crash_path))
                controller.submit_disk_job(ReplayJobRef("session", 24, next_path))
                replies = []
                remaining = 5.0
                while remaining > 0.0:
                    controller.wait_for_reply(timeout_s=0.1)
                    replies.extend(controller.poll())
                    if any(reply.attempt_id == 24 and reply.status == "COMPLETED" for reply in replies):
                        break
                    remaining -= 0.1

                crash = [reply for reply in replies if reply.attempt_id == 23 and reply.status == "FAILED"]
                self.assertTrue(crash)
                self.assertIn("exitcode 23", crash[-1].error)
                self.assertTrue(any(reply.attempt_id == 24 and reply.status == "COMPLETED" for reply in replies))
            finally:
                self.assertTrue(controller.close(timeout_s=5.0))

    def test_close_releases_retained_process_and_queue_descriptors(self):
        descriptor_root = Path("/proc/self/fd")
        if not descriptor_root.is_dir():
            self.skipTest("Linux fd canary requires /proc/self/fd")
        context = multiprocessing.get_context("spawn")
        retained = []

        warm = ReplayProcessController(
            worker_fn=_failing_worker,
            recorder=_Recorder(),
            mp_context=context,
        )
        self.assertTrue(warm.close(timeout_s=5.0))
        retained.append(warm)
        baseline = len(tuple(descriptor_root.iterdir()))

        for _index in range(8):
            controller = ReplayProcessController(
                worker_fn=_failing_worker,
                recorder=_Recorder(),
                mp_context=context,
            )
            self.assertTrue(controller.close(timeout_s=5.0))
            self.assertTrue(controller.close(timeout_s=0.0))
            retained.append(controller)

        after = len(tuple(descriptor_root.iterdir()))
        self.assertLessEqual(after, baseline + 2)

    def test_zero_budget_close_can_reap_child_on_later_close(self):
        descriptor_root = Path("/proc/self/fd")
        if not descriptor_root.is_dir():
            self.skipTest("Linux fd canary requires /proc/self/fd")
        context = multiprocessing.get_context("spawn")

        warm = ReplayProcessController(
            worker_fn=_failing_worker,
            recorder=_Recorder(),
            mp_context=context,
        )
        self.assertTrue(warm.close(timeout_s=5.0))
        baseline = len(tuple(descriptor_root.iterdir()))

        started = context.Event()
        with tempfile.TemporaryDirectory() as temp_dir:
            input_path = Path(temp_dir) / "attempt-31.json"
            input_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_DelayedTerminateWorker(started),
                recorder=_Recorder(),
                mp_context=context,
            )
            controller.submit_disk_job(ReplayJobRef("session", 31, input_path))
            self.assertTrue(started.wait(2.0))

            self.assertFalse(controller.close(timeout_s=0.0))
            self.assertIsNotNone(controller._process)
            time.sleep(0.2)
            self.assertTrue(controller.close(timeout_s=0.0))
            self.assertIsNone(controller._process)
            self.assertIsNone(controller._request_queue)
            self.assertIsNone(controller._reply_queue)

        after = len(tuple(descriptor_root.iterdir()))
        self.assertLessEqual(after, baseline + 2)

    def test_zero_budget_close_kills_and_reaps_without_second_close(self):
        context = multiprocessing.get_context("spawn")
        started = context.Event()
        with tempfile.TemporaryDirectory() as temp_dir:
            input_path = Path(temp_dir) / "attempt-41.json"
            input_path.write_text("{}\n")
            controller = ReplayProcessController(
                worker_fn=_IgnoreTerminateWorker(started),
                recorder=_Recorder(),
                mp_context=context,
            )
            try:
                controller.submit_disk_job(ReplayJobRef("session", 41, input_path))
                self.assertTrue(started.wait(2.0))

                started_close = time.monotonic()
                controller.close(timeout_s=0.0)
                self.assertLess(time.monotonic() - started_close, 0.05)

                deadline = time.monotonic() + 1.0
                while controller._process is not None and time.monotonic() < deadline:
                    time.sleep(0.005)
                self.assertIsNone(controller._process)
                reaper = controller._reaper_thread
                if reaper is not None:
                    reaper.join(0.1)
                    self.assertFalse(reaper.is_alive())
            finally:
                process = controller._process
                if process is not None and process.is_alive():
                    process.kill()
                    process.join(1.0)
                controller.close(timeout_s=0.0)

    def test_process_reap_uses_one_total_timeout_budget(self):
        class BudgetProcess:
            def __init__(self):
                self.alive = True
                self.kill_calls = 0
                self.closed = False

            def join(self, timeout=None):
                if timeout is None:
                    self.alive = False
                elif timeout > 0.0:
                    time.sleep(timeout)

            def is_alive(self):
                return self.alive

            def terminate(self):
                return None

            def kill(self):
                self.kill_calls += 1
                self.alive = False

            def close(self):
                self.closed = True

        process = BudgetProcess()
        controller = ReplayProcessController.__new__(ReplayProcessController)
        controller._process = process
        controller._process_guard = threading.RLock()
        controller._reaper_thread = None

        started = time.monotonic()
        self.assertTrue(controller._reap_process(timeout_s=0.05, terminate=True))
        elapsed = time.monotonic() - started

        self.assertLess(elapsed, 0.08)
        self.assertEqual(process.kill_calls, 1)
        self.assertTrue(process.closed)
        self.assertIsNone(controller._process)


if __name__ == "__main__":
    unittest.main()
