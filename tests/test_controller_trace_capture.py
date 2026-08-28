import threading
import unittest
from unittest.mock import patch
from pathlib import Path

try:
    from .bounded_temp import BoundedTemporaryDirectory
except ImportError:
    from bounded_temp import BoundedTemporaryDirectory

from ARrobots.controller_trace import (
    ControllerMotionProfile,
    ControllerTraceMetadata,
    decode_controller_trace,
    encode_controller_trace,
)
from ARrobots.controller_trace_capture import (
    ControllerTraceCapture,
    ControllerTraceCaptureError,
    ControllerTraceStore,
    controller_configuration_fingerprint,
)


def trace_metadata(target_positions=None):
    return ControllerTraceMetadata(
        controller_hardware_id="A1B2C3",
        firmware_version="6.7.1-ar4hmi.5",
        configuration_fingerprint="sha256:" + "a" * 64,
        start_positions=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        target_positions=(
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
            if target_positions is None
            else target_positions
        ),
        motion_profile=ControllerMotionProfile(
            speed_mode="p",
            speed_value=50.0,
            acceleration_percent=10.0,
            deceleration_percent=10.0,
            ramp_percent=20.0,
        ),
    )


class SequenceClock:
    def __init__(self, *values):
        self.values = list(values)

    def __call__(self):
        if not self.values:
            raise RuntimeError("clock exhausted")
        return self.values.pop(0)


def completed_capture(clock=None, target_positions=None):
    capture = ControllerTraceCapture(
        trace_metadata(target_positions),
        clock=clock or SequenceClock(10.0, 10.1, 10.2),
    )
    if not capture.set():
        raise AssertionError("test capture write marker failed")
    if not capture.record_telemetry((0.1, 0.2, 0.3, 0.4, 0.5, 0.6)):
        raise AssertionError("test capture telemetry failed")
    if not capture.complete((1.0, 2.0, 3.0, 4.0, 5.0, 6.0)):
        raise AssertionError("test capture terminal failed")
    return capture


class ControllerTraceCaptureTests(unittest.TestCase):
    def test_configuration_fingerprint_hashes_exact_canonical_up_bytes(self):
        self.assertEqual(
            controller_configuration_fingerprint("UPA1\n"),
            "sha256:f03dc9c105916c84989fa79084454e02f0c6a9a36fa06714a209c655ba2e7f5d",
        )
        for invalid in (
            b"UPA1\n",
            "CEA1\n",
            "UPA1",
            "UPA1\r\n",
            "UPA1\nUPB2\n",
            "UP\N{SNOWMAN}\n",
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ControllerTraceCaptureError):
                    controller_configuration_fingerprint(invalid)

    def test_complete_capture_uses_write_marker_as_time_origin(self):
        capture = completed_capture()

        trace = capture.freeze()

        self.assertEqual(capture.sample_count, 1)
        self.assertAlmostEqual(trace.samples[0].elapsed_seconds, 0.1)
        self.assertAlmostEqual(trace.terminal.elapsed_seconds, 0.2)
        self.assertEqual(trace.terminal.outcome, "completed")
        self.assertEqual(
            trace.terminal.reported_positions,
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        )

    def test_write_marker_failure_is_nonthrowing_and_discards_capture(self):
        def failed_clock():
            raise RuntimeError("monotonic unavailable")

        capture = ControllerTraceCapture(trace_metadata(), clock=failed_clock)

        self.assertFalse(capture.set())
        self.assertTrue(capture.finalized)
        self.assertIn("capture clock failed", capture.discard_reason)
        with self.assertRaisesRegex(
            ControllerTraceCaptureError,
            "capture clock failed",
        ):
            capture.freeze()

    def test_callable_write_admission_never_blocks_motion(self):
        capture = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(1.0),
        )

        self.assertIs(capture(), True)
        self.assertFalse(capture.finalized)

        def failed_clock():
            raise RuntimeError("monotonic unavailable")

        failed = ControllerTraceCapture(
            trace_metadata(),
            clock=failed_clock,
        )
        self.assertIs(failed(), True)
        self.assertTrue(failed.finalized)
        self.assertIn("capture clock failed", failed.discard_reason)

    def test_invalid_capture_sequence_discards_without_late_recovery(self):
        early_sample = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(1.0),
        )
        self.assertFalse(early_sample.record_telemetry((0.0,) * 6))
        self.assertIn("before the serial write marker", early_sample.discard_reason)
        self.assertFalse(early_sample.set())

        duplicate_marker = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(1.0, 1.1),
        )
        self.assertTrue(duplicate_marker.set())
        self.assertFalse(duplicate_marker.set())
        self.assertIn("more than once", duplicate_marker.discard_reason)

        nonadvancing = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(2.0, 2.1, 2.1),
        )
        self.assertTrue(nonadvancing.set())
        self.assertTrue(nonadvancing.record_telemetry((0.0,) * 6))
        self.assertFalse(nonadvancing.record_telemetry((0.0,) * 6))
        self.assertIn("did not advance", nonadvancing.discard_reason)

    def test_sample_limit_and_invalid_encoder_values_discard_capture(self):
        limited = ControllerTraceCapture(
            trace_metadata(),
            maximum_samples=1,
            clock=SequenceClock(1.0, 1.1, 1.2),
        )
        self.assertTrue(limited.set())
        self.assertTrue(limited.record_telemetry((0.0,) * 6))
        self.assertFalse(limited.record_telemetry((0.1,) * 6))
        self.assertEqual(limited.sample_count, 0)
        self.assertIn("sample limit", limited.discard_reason)

        invalid = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(1.0, 1.1),
        )
        self.assertTrue(invalid.set())
        self.assertFalse(invalid.record_telemetry((0.0001,) * 6))
        self.assertIn("millidegree resolution", invalid.discard_reason)

    def test_failed_and_stopped_capture_preserve_bounded_ascii_detail(self):
        failed = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(1.0, 1.1),
        )
        self.assertTrue(failed.set())
        self.assertTrue(failed.fail("bad\nresponse \N{SNOWMAN}"))
        failed_trace = failed.freeze()
        self.assertEqual(failed_trace.terminal.outcome, "failed")
        self.assertEqual(failed_trace.terminal.detail, "bad response ?")

        stopped = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(2.0, 2.1),
        )
        self.assertTrue(stopped.set())
        self.assertTrue(stopped.stop((0.0,) * 6, "physical E-stop"))
        stopped_trace = stopped.freeze()
        self.assertEqual(stopped_trace.terminal.outcome, "stopped")
        self.assertEqual(stopped_trace.terminal.reported_positions, (0.0,) * 6)

    def test_constructor_and_terminal_boundaries_reject_invalid_input(self):
        with self.assertRaisesRegex(ControllerTraceCaptureError, "metadata"):
            ControllerTraceCapture(object())
        with self.assertRaisesRegex(ControllerTraceCaptureError, "positive integer"):
            ControllerTraceCapture(trace_metadata(), maximum_samples=True)

        capture = ControllerTraceCapture(
            trace_metadata(),
            clock=SequenceClock(1.0, 1.1),
        )
        self.assertTrue(capture.set())
        self.assertFalse(capture.complete(None))
        self.assertIn("requires reported_positions", capture.discard_reason)

    def test_late_clock_failure_cannot_invalidate_finalized_trace(self):
        capture = completed_capture()
        expected = capture.freeze()

        self.assertFalse(capture.record_telemetry((0.0,) * 6))
        self.assertFalse(capture.fail("late failure"))

        self.assertIsNone(capture.discard_reason)
        self.assertEqual(capture.freeze(), expected)


class BlockingControllerTraceStore(ControllerTraceStore):
    def __init__(self, directory, entered, release):
        super().__init__(directory, maximum_pending=1)
        self.entered = entered
        self.release = release

    def _write_payload(self, payload):
        self.entered.set()
        if not self.release.wait(5.0):
            raise RuntimeError("test writer release timed out")
        return super()._write_payload(payload)


class ControllerTraceStoreTests(unittest.TestCase):
    def test_store_failure_diagnostic_is_bounded_and_nonthrowing(self):
        store = ControllerTraceStore()

        class Unprintable:
            def __str__(self):
                raise RuntimeError("format unavailable")

        self.assertTrue(store.report_failure("capture unavailable", "bad\nstate"))
        self.assertTrue(store.report_failure(Unprintable(), Unprintable()))

        events = store.drain_events()
        self.assertEqual(len(events), 2)
        self.assertEqual(events[0].kind, "failed")
        self.assertEqual(events[0].detail, "capture unavailable: bad state")
        self.assertEqual(events[1].kind, "failed")
        self.assertEqual(events[1].detail, "Unprintable: Unprintable")
        self.assertTrue(store.close())

    def test_store_reports_diagnostic_event_overflow(self):
        store = ControllerTraceStore(maximum_events=1)
        self.assertTrue(store.report_failure("first", "failure"))
        self.assertTrue(store.report_failure("second", "failure"))

        events = store.drain_events()

        self.assertEqual(len(events), 2)
        self.assertIn("event buffer was full: 1", events[0].detail)
        self.assertEqual(events[1].detail, "second: failure")
        self.assertTrue(store.close())

    def test_store_persists_decodable_trace_and_runs_analysis(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            store = ControllerTraceStore(directory)

            self.assertTrue(store.submit(completed_capture()))
            self.assertTrue(store.close(5.0))

            events = store.drain_events()
            self.assertEqual(len(events), 1)
            self.assertEqual(events[0].kind, "saved")
            self.assertIsNotNone(events[0].analysis)
            self.assertFalse(events[0].analysis.profile_analysis_eligible)
            self.assertEqual(events[0].path.parent, Path(directory).resolve())
            trace = decode_controller_trace(events[0].path.read_bytes())
            self.assertEqual(trace, completed_capture().freeze())
            self.assertFalse(any(Path(directory).glob("*.tmp")))

    def test_store_queue_saturation_drops_without_blocking_submission(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            entered = threading.Event()
            release = threading.Event()
            store = BlockingControllerTraceStore(directory, entered, release)
            self.assertTrue(store.submit(completed_capture()))
            self.assertTrue(entered.wait(2.0))
            self.assertTrue(store.submit(completed_capture()))

            self.assertFalse(store.submit(completed_capture()))

            release.set()
            self.assertTrue(store.close(5.0))
            events = store.drain_events()
            self.assertEqual(
                [event.kind for event in events].count("saved"),
                2,
            )
            dropped = [event for event in events if event.kind == "dropped"]
            self.assertEqual(len(dropped), 1)
            self.assertIn("queue is full", dropped[0].detail)

    def test_store_preserves_trace_when_analysis_fails(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            store = ControllerTraceStore(directory)
            with patch(
                "ARrobots.controller_trace_capture.analyze_controller_trace",
                side_effect=RuntimeError("analysis unavailable"),
            ):
                self.assertTrue(store.submit(completed_capture()))
                self.assertTrue(store.close(5.0))

            events = store.drain_events()
            self.assertEqual(len(events), 1)
            self.assertEqual(events[0].kind, "saved")
            self.assertIsNone(events[0].analysis)
            self.assertIn("analysis failed", events[0].detail)
            self.assertIsNotNone(events[0].path)
            self.assertTrue(events[0].path.is_file())
            decode_controller_trace(events[0].path.read_bytes())

    def test_store_reports_retention_failure_after_persisting_trace(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            store = ControllerTraceStore(directory)
            with patch.object(
                store,
                "_prune_retention",
                side_effect=OSError("retention unavailable"),
            ):
                self.assertTrue(store.submit(completed_capture()))
                self.assertTrue(store.close(5.0))

            events = store.drain_events()
            self.assertEqual(
                [event.kind for event in events],
                ["saved", "failed"],
            )
            self.assertIsNotNone(events[0].analysis)
            self.assertEqual(events[1].path, events[0].path)
            self.assertIn("retention failed", events[1].detail)
            self.assertTrue(events[0].path.is_file())
            decode_controller_trace(events[0].path.read_bytes())

    def test_store_retention_prunes_only_owned_oldest_trace_files(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            unrelated = Path(directory) / "keep.txt"
            unrelated.write_text("keep", encoding="utf-8")
            store = ControllerTraceStore(
                directory,
                maximum_files=2,
                maximum_total_bytes=1024 * 1024,
            )
            for axis in range(3):
                target = tuple(float(axis + joint) for joint in range(6))
                self.assertTrue(store.submit(completed_capture(target_positions=target)))

            self.assertTrue(store.close(5.0))

            saved = list(Path(directory).glob("trace-*.jsonl"))
            self.assertEqual(len(saved), 2)
            self.assertEqual(unrelated.read_text(encoding="utf-8"), "keep")
            self.assertEqual(
                [event.kind for event in store.drain_events()],
                ["saved", "saved", "saved"],
            )

    def test_store_retention_prunes_by_total_encoded_bytes(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            capture = completed_capture()
            payload_size = len(encode_controller_trace(capture.freeze()))
            store = ControllerTraceStore(
                directory,
                maximum_files=10,
                maximum_total_bytes=(payload_size * 2) - 1,
            )

            self.assertTrue(store.submit(capture))
            self.assertTrue(store.submit(completed_capture()))
            self.assertTrue(store.close(5.0))

            self.assertEqual(len(list(Path(directory).glob("trace-*.jsonl"))), 1)
            self.assertEqual(
                [event.kind for event in store.drain_events()],
                ["saved", "saved"],
            )

    def test_store_reports_invalid_destination_and_capture_drop(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            destination = Path(directory) / "not-a-directory"
            destination.write_text("occupied", encoding="utf-8")
            store = ControllerTraceStore(destination)
            self.assertTrue(store.submit(completed_capture()))
            self.assertTrue(store.close(5.0))
            events = store.drain_events()
            self.assertEqual(len(events), 1)
            self.assertEqual(events[0].kind, "failed")
            self.assertIn("persistence failed", events[0].detail)

            discarded = ControllerTraceCapture(
                trace_metadata(),
                clock=SequenceClock(1.0),
            )
            self.assertFalse(discarded.record_telemetry((0.0,) * 6))
            drop_store = ControllerTraceStore(Path(directory) / "unused")
            self.assertFalse(drop_store.submit(discarded))
            self.assertTrue(drop_store.close())
            drop_events = drop_store.drain_events()
            self.assertEqual(drop_events[0].kind, "dropped")

    def test_store_reports_write_and_temporary_cleanup_failures(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            store = ControllerTraceStore(directory)
            with (
                patch(
                    "ARrobots.controller_trace_capture.os.replace",
                    side_effect=OSError("replace unavailable"),
                ),
                patch.object(
                    Path,
                    "unlink",
                    side_effect=OSError("cleanup unavailable"),
                ),
            ):
                self.assertTrue(store.submit(completed_capture()))
                self.assertTrue(store.close(5.0))

            events = store.drain_events()
            self.assertEqual(len(events), 1)
            self.assertEqual(events[0].kind, "failed")
            self.assertIn("write failed", events[0].detail)
            self.assertIn("temporary cleanup failed", events[0].detail)

    def test_store_rejects_unfinalized_wrong_type_and_invalid_limits(self):
        with BoundedTemporaryDirectory(prefix="ar4-controller-trace-") as directory:
            with self.assertRaisesRegex(ControllerTraceCaptureError, "positive"):
                ControllerTraceStore(directory, maximum_pending=0)
            with self.assertRaisesRegex(ControllerTraceCaptureError, "positive"):
                ControllerTraceStore(directory, maximum_total_bytes=True)

            store = ControllerTraceStore(directory)
            with self.assertRaisesRegex(ControllerTraceCaptureError, "requires"):
                store.submit(object())
            with self.assertRaisesRegex(ControllerTraceCaptureError, "finalized"):
                store.submit(ControllerTraceCapture(trace_metadata()))
            self.assertTrue(store.close())
            self.assertFalse(store.submit(completed_capture()))
            self.assertEqual(store.drain_events()[0].kind, "dropped")
            with self.assertRaisesRegex(ControllerTraceCaptureError, "event limit"):
                store.drain_events(True)


if __name__ == "__main__":
    unittest.main()
