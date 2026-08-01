from pathlib import Path
from queue import Queue
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from PIL import Image

if __package__:
    from .bounded_temp import BoundedTemporaryDirectory
else:
    from bounded_temp import BoundedTemporaryDirectory

from ARrobots.HMI.joint_motion import MotionInputError
from ARrobots.HMI.vision_io import (
    CameraPreviewFrame,
    CameraPreviewLifecycleState,
    CameraPreviewWorker,
    fit_vision_preview_square,
    load_bounded_vision_image,
    normalize_camera_exception_detail,
    prepare_camera_preview_frame,
)


class QueuedCapture:
    def __init__(self):
        self.responses = Queue()
        self.read_threads = []
        self.release_threads = []
        self.release_count = 0
        self.opened = True

    def isOpened(self):
        return self.opened

    def read(self):
        self.read_threads.append(threading.get_ident())
        return self.responses.get(timeout=2)

    def release(self):
        self.release_threads.append(threading.get_ident())
        self.release_count += 1
        self.opened = False


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


class VisionIoTests(unittest.TestCase):
    def test_camera_preview_worker_validates_constructor_contract(self):
        valid_factory = lambda camera_index: None
        constructors = (
            (
                "capture factory",
                lambda: CameraPreviewWorker(None),
                "capture factory",
            ),
            (
                "frame transform",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    frame_transform=None,
                ),
                "frame transform",
            ),
            (
                "warmup",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    warmup_frames=-1,
                ),
                "warmup frame count",
            ),
            (
                "warmup upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    warmup_frames=121,
                ),
                "warmup frame count exceeds",
            ),
            (
                "read limit",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    read_failure_limit=0,
                ),
                "read failure limit must be positive",
            ),
            (
                "read upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    read_failure_limit=121,
                ),
                "read failure count exceeds",
            ),
            (
                "retry",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    retry_seconds=float("inf"),
                ),
                "retry interval",
            ),
            (
                "retry upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    retry_seconds=5.1,
                ),
                "retry interval",
            ),
            (
                "release attempts",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    release_attempts=0,
                ),
                "release attempt count must be positive",
            ),
            (
                "release upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    release_attempts=11,
                ),
                "release attempt count exceeds",
            ),
            (
                "thread factory",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    thread_factory=None,
                ),
                "thread factory",
            ),
        )
        for name, constructor, expected_detail in constructors:
            with self.subTest(name=name):
                with self.assertRaisesRegex(
                    MotionInputError,
                    expected_detail,
                ):
                    constructor()

    def test_camera_preview_request_and_diagnostic_boundaries(self):
        worker = CameraPreviewWorker(
            lambda camera_source: self.fail(
                "invalid camera source must not reach the factory"
            )
        )
        for source in (
            True,
            -1,
            "",
            " /dev/video0",
            "/dev/video0\n",
            "camera\x00source",
            "x" * 513,
        ):
            with self.subTest(source=source):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "camera source",
                ):
                    worker.request_start(source)
        with self.assertRaisesRegex(
            MotionInputError,
            "replacement option",
        ):
            worker.request_start(0, replace="yes")

        detail = normalize_camera_exception_detail(
            RuntimeError("a" * 511 + " " + "tail")
        )
        self.assertEqual(detail, "a" * 511)
        self.assertEqual(detail, detail.strip())

    def test_camera_capture_once_uses_owned_worker_lifecycle(self):
        capture = QueuedCapture()
        factory_calls = []

        def factory(camera_source):
            factory_calls.append(camera_source)
            return capture

        worker = CameraPreviewWorker(
            factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0.05,
        )
        expected = np.full((2, 3, 3), 19, dtype=np.uint8)
        capture.responses.put((True, expected))
        capture.responses.put((False, None))
        captured = worker.capture_once("/dev/video6", 1)

        np.testing.assert_array_equal(captured, expected)
        self.assertIsNot(captured, expected)
        self.assertEqual(factory_calls, ["/dev/video6"])
        self.assertEqual(capture.release_count, 1)
        self.assertFalse(worker.active)
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["starting", "started", "stopping", "stopped"],
        )

        cancelled = threading.Event()
        cancelled.set()
        with self.assertRaisesRegex(MotionInputError, "was cancelled"):
            worker.capture_once(0, 1, cancelled)
        self.assertEqual(factory_calls, ["/dev/video6"])

    def test_camera_readiness_wait_honors_cancellation(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_source: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        cancelled = threading.Event()
        cancelled.set()

        self.assertFalse(worker.wait_ready(request_id, 1, cancelled))
        self.assertTrue(worker.request_stop(request_id))
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_request_stopped(request_id, 1))

    def test_camera_stop_before_worker_entry_has_terminal_lifecycle(self):
        worker_entry = threading.Event()

        def thread_factory(target, name, daemon):
            def delayed_target():
                worker_entry.wait(1)
                target()

            return threading.Thread(
                target=delayed_target,
                name=name,
                daemon=daemon,
            )

        worker = CameraPreviewWorker(
            lambda camera_source: self.fail(
                "cancelled request must not open a capture"
            ),
            thread_factory=thread_factory,
        )
        request_id = worker.request_start(0)
        self.assertTrue(worker.request_stop(request_id))
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["starting", "stopping", "stopped"],
        )
        worker_entry.set()
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_replacement_retains_undrained_stop_event(self):
        worker_entry = threading.Event()
        capture = QueuedCapture()
        factory_sources = []

        def thread_factory(target, name, daemon):
            def delayed_target():
                worker_entry.wait(1)
                target()

            return threading.Thread(
                target=delayed_target,
                name=name,
                daemon=daemon,
            )

        def capture_factory(camera_source):
            factory_sources.append(camera_source)
            return capture

        worker = CameraPreviewWorker(
            capture_factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
            thread_factory=thread_factory,
        )
        stopped_request = worker.request_start(0)
        self.assertTrue(worker.request_stop(stopped_request))
        replacement_request = worker.request_start(1)
        self.assertEqual(
            [
                (event.kind, event.request_id)
                for event in worker.drain_events()
            ],
            [
                ("stopped", stopped_request),
                ("starting", replacement_request),
            ],
        )

        worker_entry.set()
        self.assertTrue(
            wait_for(
                lambda: (
                    worker.active_request_id == replacement_request
                    and factory_sources == [1]
                )
            )
        )
        self.assertEqual(factory_sources, [1])
        self.assertTrue(worker.request_stop(replacement_request))
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_worker_moves_io_off_caller_and_coalesces_frames(self):
        capture = QueuedCapture()
        factory_threads = []

        def factory(camera_index):
            factory_threads.append((camera_index, threading.get_ident()))
            return capture

        worker = CameraPreviewWorker(
            factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        self.assertFalse(worker.closed)
        caller_thread = threading.get_ident()
        request_id = worker.request_start(2)
        for value in (1, 2, 3):
            capture.responses.put(
                (
                    True,
                    np.full((3, 4, 3), value, dtype=np.uint8),
                )
            )

        def latest_raw_is_three():
            frame = worker.snapshot_raw_frame(request_id)
            return frame is not None and np.all(frame == 3)

        self.assertTrue(wait_for(latest_raw_is_three))
        self.assertEqual(worker.active_request_id, request_id)
        self.assertIsNone(worker.take_latest_frame(request_id + 1))
        preview = worker.take_latest_frame(request_id)
        self.assertIsNotNone(preview)
        self.assertTrue(np.all(preview.image == 3))
        self.assertFalse(preview.image.flags.writeable)
        self.assertEqual(factory_threads[0][0], 2)
        self.assertNotEqual(factory_threads[0][1], caller_thread)
        self.assertTrue(capture.read_threads)
        self.assertNotIn(caller_thread, capture.read_threads)

        events = list(worker.drain_events())
        self.assertEqual(
            [event.kind for event in events],
            ["starting", "started"],
        )
        self.assertEqual(
            [event.sequence for event in events],
            sorted(event.sequence for event in events),
        )

        self.assertTrue(worker.request_stop())
        capture.responses.put((False, None))
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertEqual(capture.release_count, 1)
        self.assertNotIn(caller_thread, capture.release_threads)
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["stopping", "stopped"],
        )

    def test_camera_preview_worker_uses_default_warmup_and_transform(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        for value in range(5):
            capture.responses.put(
                (
                    True,
                    np.full((2, 3, 3), value, dtype=np.uint8),
                )
            )
        final_frame = np.zeros((2, 3, 3), dtype=np.uint8)
        final_frame[:, :] = (10, 20, 30)
        capture.responses.put((True, final_frame))

        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(request_id) is not None
            )
        )
        raw = worker.snapshot_raw_frame(request_id)
        np.testing.assert_array_equal(raw, final_frame)
        preview = worker.take_latest_frame(request_id)
        self.assertEqual(preview.image.shape, (320, 480, 3))
        np.testing.assert_array_equal(preview.image[0, 0], (30, 20, 10))
        self.assertFalse(preview.image.flags.writeable)

        self.assertTrue(worker.request_stop())
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_restart_cannot_be_stranded_during_worker_exit(self):
        first = QueuedCapture()
        second = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: (first, second)[camera_index],
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )

        class ExitBoundaryLock:
            def __init__(self, owner):
                self._lock = threading.Lock()
                self.owner = owner
                self.candidate_releases = 0
                self.boundary = threading.Event()
                self.resume = threading.Event()

            def acquire(self, *args, **kwargs):
                return self._lock.acquire(*args, **kwargs)

            def release(self):
                current = threading.current_thread()
                exit_candidate = (
                    current.name == "ar4-camera-preview"
                    and self.owner._desired is None
                    and self.owner._active_request_id is None
                )
                if exit_candidate:
                    self.candidate_releases += 1
                self._lock.release()
                if exit_candidate and self.candidate_releases == 2:
                    self.boundary.set()
                    if not self.resume.wait(2):
                        raise RuntimeError("exit-boundary test was not resumed")

            def __enter__(self):
                self.acquire()
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                self.release()

        boundary_lock = ExitBoundaryLock(worker)
        worker._lock = boundary_lock
        worker._state_changed = threading.Condition(boundary_lock)
        first_request = worker.request_start(0)
        first.responses.put(
            (True, np.full((2, 2, 3), 1, dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(first_request) is not None
            )
        )
        old_thread = worker._worker

        self.assertTrue(worker.request_stop())
        first.responses.put((False, None))
        self.assertTrue(boundary_lock.boundary.wait(2))
        second_request = worker.request_start(1)
        self.assertIsNot(worker._worker, old_thread)
        self.assertEqual(worker.desired_request_id, second_request)
        boundary_lock.resume.set()
        old_thread.join(1)
        self.assertFalse(old_thread.is_alive())

        second.responses.put(
            (True, np.full((2, 2, 3), 2, dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(second_request) is not None
            )
        )
        self.assertEqual(worker.active_request_id, second_request)
        self.assertTrue(worker.request_stop())
        second.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_close_reports_and_waits_for_live_retirement(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        capture.responses.put(
            (True, np.ones((2, 2, 3), dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(request_id) is not None
            )
        )

        close_state = worker.close_state()
        self.assertIsInstance(close_state, CameraPreviewLifecycleState)
        self.assertTrue(close_state.active)
        self.assertFalse(close_state.stopped)
        self.assertTrue(close_state.closed)
        self.assertIsNone(close_state.fault_reason)
        self.assertFalse(close_state.clean)
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))
        self.assertFalse(worker.active)
        retired_state = worker.close_state()
        self.assertFalse(retired_state.active)
        self.assertTrue(retired_state.stopped)
        self.assertTrue(retired_state.clean)
        self.assertTrue(worker.close())
        with self.assertRaisesRegex(MotionInputError, "worker is closed"):
            worker.request_start(1)
        with self.assertRaisesRegex(MotionInputError, "wait timeout"):
            worker.wait_stopped(float("nan"))
        with self.assertRaisesRegex(MotionInputError, "wait timeout"):
            worker.wait_stopped(threading.TIMEOUT_MAX + 1)

    def test_camera_preview_worker_replaces_active_request_without_tk_wait(self):
        first = QueuedCapture()
        second = QueuedCapture()
        captures = {4: first, 7: second}
        factory_calls = []

        def factory(camera_index):
            factory_calls.append(camera_index)
            return captures[camera_index]

        worker = CameraPreviewWorker(
            factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        first_request = worker.request_start(4)
        first.responses.put(
            (True, np.full((2, 2, 3), 4, dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(first_request) is not None
            )
        )

        second_request = worker.request_start(7)
        self.assertNotEqual(first_request, second_request)
        self.assertEqual(worker.desired_request_id, second_request)
        with self.assertRaisesRegex(
            MotionInputError,
            "ownership changed before idle",
        ):
            worker.wait_request_stopped(
                first_request,
                0,
                require_idle=True,
            )
        first.responses.put((False, None))
        second.responses.put(
            (True, np.full((2, 2, 3), 7, dtype=np.uint8))
        )

        def replacement_is_live():
            frame = worker.snapshot_raw_frame(second_request)
            return frame is not None and np.all(frame == 7)

        self.assertTrue(wait_for(replacement_is_live))
        self.assertEqual(factory_calls, [4, 7])
        self.assertEqual(first.release_count, 1)
        self.assertIsNone(worker.snapshot_raw_frame(first_request))

        self.assertTrue(worker.request_stop())
        second.responses.put((False, None))
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertEqual(second.release_count, 1)
        events = worker.drain_events()
        first_stopped = next(
            index
            for index, event in enumerate(events)
            if event.kind == "stopped"
            and event.request_id == first_request
        )
        second_started = next(
            index
            for index, event in enumerate(events)
            if event.kind == "started"
            and event.request_id == second_request
        )
        self.assertLess(first_stopped, second_started)

    def test_camera_preview_replacement_preserves_undelivered_failure(self):
        class ClosedCapture(QueuedCapture):
            def __init__(self):
                super().__init__()
                self.opened = False

        first = ClosedCapture()
        second = QueuedCapture()
        captures = iter((first, second))
        worker = CameraPreviewWorker(
            lambda camera_index: next(captures),
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        first_request = worker.request_start(0)
        self.assertTrue(worker.wait_stopped(1))

        second_request = worker.request_start(0)
        second.responses.put(
            (True, np.ones((2, 2, 3), dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(second_request) is not None
            )
        )
        events = worker.drain_events()
        retained_failure = next(
            event
            for event in events
            if event.kind == "failed"
            and event.request_id == first_request
        )
        self.assertIn("did not open", retained_failure.detail)
        self.assertIn(
            ("started", second_request),
            tuple((event.kind, event.request_id) for event in events),
        )

        self.assertTrue(worker.request_stop())
        second.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_worker_rejects_invalid_frames_without_poisoning(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        capture.responses.put(
            (True, np.zeros((2, 2), dtype=np.uint8))
        )
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertIsNone(worker.fault_reason)
        self.assertIsNone(worker.snapshot_raw_frame(request_id))
        self.assertEqual(capture.release_count, 1)
        events = worker.drain_events()
        failures = [event for event in events if event.kind == "failed"]
        self.assertEqual(len(failures), 1)
        self.assertIn("three 8-bit channels", failures[0].detail)
        self.assertEqual(events[-1].kind, "stopped")

    def test_camera_preview_worker_exhausts_bounded_read_failures(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        worker.request_start(0)
        for _ in range(3):
            capture.responses.put((False, None))

        self.assertTrue(worker.wait_stopped(1))
        self.assertEqual(capture.release_count, 1)
        failure = next(
            event
            for event in worker.drain_events()
            if event.kind == "failed"
        )
        self.assertIn("consecutive read-failure limit", failure.detail)

    def test_camera_preview_worker_rejects_open_and_read_contract_failures(self):
        class ResultCapture:
            def __init__(self, result, opened=True):
                self.result = result
                self.opened = opened
                self.read_count = 0
                self.release_count = 0

            def isOpened(self):
                return self.opened

            def read(self):
                self.read_count += 1
                return self.result

            def release(self):
                self.release_count += 1
                self.opened = False

        cases = (
            ("closed", ResultCapture((True, None), opened=False), "did not open"),
            ("shape", ResultCapture("invalid"), "read result is invalid"),
            ("status", ResultCapture((1, None)), "read status is invalid"),
            (
                "failed-data",
                ResultCapture(
                    (False, np.zeros((2, 2, 3), dtype=np.uint8))
                ),
                "unexpected frame data",
            ),
        )
        for name, capture, expected_detail in cases:
            with self.subTest(name=name):
                worker = CameraPreviewWorker(
                    lambda camera_index, selected=capture: selected,
                    frame_transform=lambda frame: frame,
                    warmup_frames=0,
                    retry_seconds=0,
                )
                worker.request_start(0)
                self.assertTrue(worker.wait_stopped(1))
                failure = next(
                    event
                    for event in worker.drain_events()
                    if event.kind == "failed"
                )
                self.assertIn(expected_detail, failure.detail)
                self.assertEqual(capture.release_count, 1)
                self.assertEqual(
                    capture.read_count,
                    0 if name == "closed" else 1,
                )

    def test_camera_preview_worker_rejects_missing_capture_operations(self):
        class MissingRead:
            @staticmethod
            def isOpened():
                return True

            @staticmethod
            def release():
                pass

        class MissingRelease:
            @staticmethod
            def isOpened():
                return True

            @staticmethod
            def read():
                return False, None

        class InvalidOpenState:
            @staticmethod
            def isOpened():
                return 1

            @staticmethod
            def read():
                return False, None

            @staticmethod
            def release():
                pass

        cases = (
            (MissingRead(), "no read operation"),
            (MissingRelease(), "no release operation"),
            (InvalidOpenState(), "open state is invalid"),
        )
        for capture, expected_detail in cases:
            with self.subTest(expected_detail=expected_detail):
                worker = CameraPreviewWorker(
                    lambda camera_index, selected=capture: selected,
                    retry_seconds=0,
                )
                worker.request_start(0)
                self.assertTrue(worker.wait_stopped(1))
                failure = next(
                    event
                    for event in worker.drain_events()
                    if event.kind == "failed"
                    and expected_detail in event.detail
                )
                self.assertIn(expected_detail, failure.detail)

    def test_camera_preview_error_details_are_fully_bounded_and_normalized(self):
        prefix = "camera preview failed: "
        long_detail = "x" * (512 - len(prefix) - 1) + " " + "tail"
        worker = CameraPreviewWorker(
            lambda camera_index: (_ for _ in ()).throw(
                RuntimeError(long_detail)
            ),
            retry_seconds=0,
        )
        worker.request_start(0)

        self.assertTrue(worker.wait_stopped(1))
        failure = next(
            event
            for event in worker.drain_events()
            if event.kind == "failed"
        )
        self.assertLessEqual(len(failure.detail), 512)
        self.assertEqual(len(failure.detail), 511)
        self.assertEqual(failure.detail, failure.detail.strip())
        self.assertIsNone(worker.fault_reason)

    def test_camera_preview_worker_latches_unreleased_capture(self):
        class UnreleasedCapture:
            @staticmethod
            def isOpened():
                return True

            @staticmethod
            def read():
                return True, np.zeros((2, 2), dtype=np.uint8)

            @staticmethod
            def release():
                raise RuntimeError("device remains owned")

        worker = CameraPreviewWorker(
            lambda camera_index: UnreleasedCapture(),
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        worker.request_start(0)
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertIn("camera release failed", worker.fault_reason)
        with self.assertRaisesRegex(
            MotionInputError,
            "requires an application restart",
        ):
            worker.request_start(1)
        self.assertFalse(worker.close())
        failures = [
            event
            for event in worker.drain_events()
            if event.kind == "failed"
        ]
        self.assertEqual(len(failures), 2)
        self.assertIn("device remains owned", failures[-1].detail)

    def test_camera_preview_release_retries_and_latches_persistent_open_state(self):
        class DelayedReleaseCapture:
            def __init__(self, closes_after):
                self.closes_after = closes_after
                self.release_count = 0

            def isOpened(self):
                return self.release_count < self.closes_after

            @staticmethod
            def read():
                return True, np.zeros((2, 2), dtype=np.uint8)

            def release(self):
                self.release_count += 1

        recovering_capture = DelayedReleaseCapture(2)
        recovering_worker = CameraPreviewWorker(
            lambda camera_index: recovering_capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0.25,
        )
        recovering_sleeps = []
        with patch(
            "ARrobots.HMI.vision_io.time.sleep",
            side_effect=recovering_sleeps.append,
        ):
            recovering_worker.request_start(0)
            self.assertTrue(recovering_worker.wait_stopped(1))
        self.assertEqual(recovering_capture.release_count, 2)
        self.assertEqual(recovering_sleeps, [0.25])
        self.assertIsNone(recovering_worker.fault_reason)

        retained_capture = DelayedReleaseCapture(4)
        retained_worker = CameraPreviewWorker(
            lambda camera_index: retained_capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0.25,
        )
        retained_sleeps = []
        with patch(
            "ARrobots.HMI.vision_io.time.sleep",
            side_effect=retained_sleeps.append,
        ):
            retained_worker.request_start(0)
            self.assertTrue(retained_worker.wait_stopped(1))
        self.assertEqual(retained_capture.release_count, 3)
        self.assertEqual(retained_sleeps, [0.25, 0.25])
        self.assertIn("device remained open", retained_worker.fault_reason)

    def test_camera_preview_worker_rolls_back_thread_start_failure(self):
        def failing_thread_factory(**kwargs):
            raise RuntimeError("thread construction unavailable")

        creation_worker = CameraPreviewWorker(
            lambda camera_index: self.fail("capture factory must not run"),
            thread_factory=failing_thread_factory,
        )
        with self.assertRaisesRegex(
            RuntimeError,
            "thread construction unavailable",
        ):
            creation_worker.request_start(0)
        self.assertFalse(creation_worker.active)
        self.assertIsNone(creation_worker.desired_request_id)
        self.assertTrue(creation_worker.wait_stopped(0))
        self.assertEqual(
            [event.kind for event in creation_worker.drain_events()],
            ["starting", "failed"],
        )

        invalid_worker = CameraPreviewWorker(
            lambda camera_index: self.fail("capture factory must not run"),
            thread_factory=lambda **kwargs: object(),
        )
        with self.assertRaisesRegex(MotionInputError, "invalid worker"):
            invalid_worker.request_start(0)
        self.assertFalse(invalid_worker.active)
        self.assertIsNone(invalid_worker.desired_request_id)
        self.assertTrue(invalid_worker.wait_stopped(0))

        class FailingThread(threading.Thread):
            def start(self):
                raise RuntimeError("thread unavailable")

        worker = CameraPreviewWorker(
            lambda camera_index: self.fail("capture factory must not run"),
            thread_factory=FailingThread,
        )
        with self.assertRaisesRegex(RuntimeError, "thread unavailable"):
            worker.request_start(0)
        self.assertFalse(worker.active)
        self.assertIsNone(worker.desired_request_id)
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["starting", "failed"],
        )
        self.assertTrue(worker.close())

    def test_camera_preview_frame_conversion_validates_and_converts_bgr(self):
        source = np.zeros((2, 3, 3), dtype=np.uint8)
        source[:, :] = (1, 2, 3)
        preview = prepare_camera_preview_frame(source, 3, 2)

        np.testing.assert_array_equal(preview[0, 0], (3, 2, 1))
        np.testing.assert_array_equal(source[0, 0], (1, 2, 3))
        with self.assertRaisesRegex(MotionInputError, "three 8-bit channels"):
            prepare_camera_preview_frame(
                np.zeros((2, 3), dtype=np.uint8),
                3,
                2,
            )
        with patch(
            "ARrobots.HMI.vision_io.MAX_CAMERA_FRAME_PIXELS",
            3,
        ):
            with self.assertRaisesRegex(MotionInputError, "pixel count"):
                prepare_camera_preview_frame(source, 3, 2)
        first = CameraPreviewFrame(0, 0, preview)
        second = CameraPreviewFrame(0, 0, preview)
        self.assertIsNot(first, second)
        self.assertNotEqual(first, second)

    def test_bounded_loader_decodes_valid_image_from_admitted_bytes(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "template.png"
            source = np.zeros((7, 11, 3), dtype=np.uint8)
            source[2, 3] = (10, 20, 30)
            self.assertTrue(cv2.imwrite(str(image_path), source))

            loaded = load_bounded_vision_image(
                image_path,
                cv2.IMREAD_COLOR,
                "vision template",
            )

        self.assertEqual(loaded.shape, source.shape)
        np.testing.assert_array_equal(loaded[2, 3], source[2, 3])

    def test_bounded_loader_uses_stored_geometry_for_exif_rotated_jpeg(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "rotated.jpg"
            exif = Image.Exif()
            exif[274] = 6
            Image.new("RGB", (3, 5), color=(10, 20, 30)).save(
                image_path,
                exif=exif,
            )

            loaded = load_bounded_vision_image(
                image_path,
                cv2.IMREAD_COLOR,
                "vision template",
            )

        self.assertEqual(loaded.shape, (5, 3, 3))

    def test_bounded_loader_normalizes_pillow_decompression_bomb_errors(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "template.jpg"
            image_path.write_bytes(b"bounded")
            with patch(
                "ARrobots.HMI.vision_io.Image.open",
                side_effect=Image.DecompressionBombError("declared image too large"),
            ):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "could not be decoded",
                ):
                    load_bounded_vision_image(
                        image_path,
                        cv2.IMREAD_COLOR,
                        "vision template",
                    )

    def test_bounded_loader_rejects_unsupported_decode_modes(self):
        with self.assertRaisesRegex(MotionInputError, "decode mode"):
            load_bounded_vision_image(
                "unused.jpg",
                cv2.IMREAD_ANYDEPTH,
                "vision template",
            )

    def test_bounded_loader_normalizes_opencv_decode_errors(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "template.png"
            Image.new("RGB", (3, 5), color=(10, 20, 30)).save(image_path)
            with patch(
                "ARrobots.HMI.vision_io.cv2.imdecode",
                side_effect=cv2.error("decode failed"),
            ):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "could not be decoded",
                ):
                    load_bounded_vision_image(
                        image_path,
                        cv2.IMREAD_COLOR,
                        "vision template",
                    )

    def test_bounded_loader_rejects_directory_corruption_and_size_overflow(self):
        with BoundedTemporaryDirectory() as directory:
            root = Path(directory)
            corrupt_path = root / "corrupt.jpg"
            corrupt_path.write_bytes(b"not an image")
            with self.assertRaisesRegex(MotionInputError, "could not be decoded"):
                load_bounded_vision_image(
                    corrupt_path,
                    cv2.IMREAD_COLOR,
                    "vision template",
                )
            with self.assertRaisesRegex(
                MotionInputError,
                "regular file|could not be read",
            ):
                load_bounded_vision_image(
                    root,
                    cv2.IMREAD_COLOR,
                    "vision template",
                )

            oversized_path = root / "oversized.jpg"
            oversized_path.write_bytes(b"12345")
            with patch(
                "ARrobots.HMI.vision_io.MAX_VISION_IMAGE_BYTES",
                4,
            ):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "file-size limit",
                ):
                    load_bounded_vision_image(
                        oversized_path,
                        cv2.IMREAD_COLOR,
                        "vision template",
                    )

    def test_preview_square_preserves_extreme_aspects_with_positive_sides(self):
        wide = np.zeros((1, 8192, 3), dtype=np.uint8)
        tall = np.zeros((8192, 1, 3), dtype=np.uint8)

        wide_preview = fit_vision_preview_square(wide, 150)
        tall_preview = fit_vision_preview_square(tall, 150)

        self.assertEqual(wide_preview.shape, (150, 150, 3))
        self.assertEqual(tall_preview.shape, (150, 150, 3))

    def test_preview_square_rejects_invalid_target_and_image_shape(self):
        with self.assertRaisesRegex(MotionInputError, "preview size"):
            fit_vision_preview_square(np.zeros((1, 1, 3), dtype=np.uint8), 0)
        with self.assertRaisesRegex(MotionInputError, "three 8-bit channels"):
            fit_vision_preview_square(np.zeros((1, 1), dtype=np.uint8), 150)
        with self.assertRaisesRegex(MotionInputError, "8-bit channels"):
            fit_vision_preview_square(
                np.zeros((1, 1, 3), dtype=np.float32),
                150,
            )


if __name__ == "__main__":
    unittest.main()
