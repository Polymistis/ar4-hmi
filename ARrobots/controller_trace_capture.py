"""Bounded, non-blocking capture and persistence for controller traces."""

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import math
from numbers import Real
import os
from pathlib import Path
from queue import Empty, Full, Queue
import secrets
import threading
import time
from typing import Optional

from ARrobots.controller_trace import (
    CONTROLLER_TRACE_MAXIMUM_BYTES,
    CONTROLLER_TRACE_MAXIMUM_DETAIL_LENGTH,
    CONTROLLER_TRACE_MAXIMUM_SAMPLES,
    ControllerTrace,
    ControllerTraceAnalysis,
    ControllerTraceMetadata,
    ControllerTraceSample,
    ControllerTraceTerminal,
    analyze_controller_trace,
    encode_controller_trace,
)


CONTROLLER_TRACE_CAPTURE_DIRECTORY = "controller-traces"
CONTROLLER_TRACE_CAPTURE_MAXIMUM_PENDING = 8
CONTROLLER_TRACE_CAPTURE_MAXIMUM_FILES = 64
CONTROLLER_TRACE_CAPTURE_MAXIMUM_TOTAL_BYTES = 64 * 1024 * 1024
CONTROLLER_TRACE_CAPTURE_MAXIMUM_EVENTS = 128


class ControllerTraceCaptureError(ValueError):
    """Capture or persistence input violates the bounded trace contract."""


def controller_configuration_fingerprint(command):
    """Hash the exact validated canonical UP command sent to the controller."""

    if not isinstance(command, str):
        raise ControllerTraceCaptureError(
            "controller configuration command must be text"
        )
    if (
        not command.startswith("UP")
        or not command.endswith("\n")
        or "\n" in command[:-1]
        or "\r" in command
    ):
        raise ControllerTraceCaptureError(
            "controller configuration command must be one UP line"
        )
    try:
        payload = command.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ControllerTraceCaptureError(
            "controller configuration command must be ASCII"
        ) from exc
    if len(payload) > CONTROLLER_TRACE_MAXIMUM_BYTES:
        raise ControllerTraceCaptureError(
            "controller configuration command exceeds the capture byte limit"
        )
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _positive_integer(value, field_name, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ControllerTraceCaptureError(f"{field_name} must be a positive integer")
    if maximum is not None and value > maximum:
        raise ControllerTraceCaptureError(
            f"{field_name} must not exceed {maximum}"
        )
    return value


def _nonnegative_finite(value, field_name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ControllerTraceCaptureError(
            f"{field_name} must be a non-negative finite number"
        )
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ControllerTraceCaptureError(
            f"{field_name} must be a non-negative finite number"
        ) from exc
    if not math.isfinite(result) or result < 0:
        raise ControllerTraceCaptureError(
            f"{field_name} must be a non-negative finite number"
        )
    return result


def _bounded_detail(value):
    try:
        raw_detail = str(value)
    except Exception:
        raw_detail = type(value).__name__
    detail = " ".join(
        raw_detail[:CONTROLLER_TRACE_MAXIMUM_DETAIL_LENGTH * 4].split()
    )
    detail = "".join(
        character if 32 <= ord(character) < 127 else "?"
        for character in detail
    )
    detail = detail[:CONTROLLER_TRACE_MAXIMUM_DETAIL_LENGTH]
    return detail or "controller trace capture failed without details"


def _capture_clock(clock):
    if not callable(clock):
        raise ControllerTraceCaptureError("capture clock must be callable")
    try:
        timestamp = clock()
    except Exception as exc:
        raise ControllerTraceCaptureError("capture clock failed") from exc
    return _nonnegative_finite(timestamp, "capture clock result")


@dataclass(frozen=True)
class ControllerTracePersistenceEvent:
    """Bounded completion record emitted by the background trace store."""

    kind: str
    detail: str
    path: Optional[Path] = None
    analysis: Optional[ControllerTraceAnalysis] = None

    def __post_init__(self):
        if self.kind not in ("saved", "dropped", "failed"):
            raise ControllerTraceCaptureError(
                "persistence event kind must be saved, dropped, or failed"
            )
        detail = _bounded_detail(self.detail)
        if self.path is not None and not isinstance(self.path, Path):
            raise ControllerTraceCaptureError(
                "persistence event path must be pathlib.Path or None"
            )
        if (
            self.analysis is not None
            and not isinstance(self.analysis, ControllerTraceAnalysis)
        ):
            raise ControllerTraceCaptureError(
                "persistence event analysis has an invalid type"
            )
        object.__setattr__(self, "detail", detail)


class ControllerTraceCapture:
    """Collect one RJ exchange without performing file or analysis work."""

    def __init__(
        self,
        metadata,
        maximum_samples=CONTROLLER_TRACE_MAXIMUM_SAMPLES,
        clock=time.monotonic,
    ):
        if not isinstance(metadata, ControllerTraceMetadata):
            raise ControllerTraceCaptureError(
                "capture metadata must be ControllerTraceMetadata"
            )
        self._maximum_samples = _positive_integer(
            maximum_samples,
            "maximum_samples",
            CONTROLLER_TRACE_MAXIMUM_SAMPLES,
        )
        if not callable(clock):
            raise ControllerTraceCaptureError("capture clock must be callable")
        self._metadata = metadata
        self._clock = clock
        self._lock = threading.Lock()
        self._origin_seconds = None
        self._samples = []
        self._terminal = None
        self._discard_reason = None

    @property
    def metadata(self):
        return self._metadata

    @property
    def finalized(self):
        with self._lock:
            return self._terminal is not None or self._discard_reason is not None

    @property
    def discard_reason(self):
        with self._lock:
            return self._discard_reason

    @property
    def sample_count(self):
        with self._lock:
            return len(self._samples)

    def set(self):
        """Satisfy the serial write-marker contract without raising into motion."""

        try:
            timestamp = _capture_clock(self._clock)
        except Exception as exc:
            self._discard(_bounded_detail(exc))
            return False
        with self._lock:
            if self._discard_reason is not None or self._terminal is not None:
                return False
            if self._origin_seconds is not None:
                self._discard_locked("serial write marker was set more than once")
                return False
            self._origin_seconds = timestamp
        return True

    def __call__(self):
        """Admit the controller write even when diagnostic capture is lost."""

        try:
            self.set()
        except Exception as exc:
            try:
                self._discard(_bounded_detail(exc))
            except Exception:
                pass
        return True

    def record_telemetry(self, encoder_positions):
        try:
            timestamp = _capture_clock(self._clock)
        except Exception as exc:
            self._discard(_bounded_detail(exc))
            return False
        with self._lock:
            if self._discard_reason is not None or self._terminal is not None:
                return False
            if self._origin_seconds is None:
                self._discard_locked(
                    "telemetry arrived before the serial write marker"
                )
                return False
            if len(self._samples) >= self._maximum_samples:
                self._discard_locked("controller trace exceeded the sample limit")
                return False
            elapsed = timestamp - self._origin_seconds
            try:
                sample = ControllerTraceSample(elapsed, encoder_positions)
            except Exception as exc:
                self._discard_locked(_bounded_detail(exc))
                return False
            if self._samples and elapsed <= self._samples[-1].elapsed_seconds:
                self._discard_locked(
                    "controller trace sample timestamps did not advance"
                )
                return False
            self._samples.append(sample)
        return True

    def complete(self, reported_positions):
        return self._finish("completed", reported_positions, None)

    def stop(self, reported_positions, detail):
        return self._finish("stopped", reported_positions, detail)

    def fail(self, detail, reported_positions=None):
        return self._finish("failed", reported_positions, detail)

    def freeze(self):
        """Build the validated immutable trace after queue ownership transfer."""

        with self._lock:
            if self._discard_reason is not None:
                raise ControllerTraceCaptureError(self._discard_reason)
            if self._terminal is None:
                raise ControllerTraceCaptureError(
                    "controller trace capture is not finalized"
                )
            return ControllerTrace(
                self._metadata,
                tuple(self._samples),
                self._terminal,
            )

    def _finish(self, outcome, reported_positions, detail):
        try:
            timestamp = _capture_clock(self._clock)
        except Exception as exc:
            self._discard(_bounded_detail(exc))
            return False
        with self._lock:
            if self._discard_reason is not None or self._terminal is not None:
                return False
            if self._origin_seconds is None:
                self._discard_locked(
                    "controller trace ended before the serial write marker"
                )
                return False
            elapsed = timestamp - self._origin_seconds
            if self._samples and elapsed < self._samples[-1].elapsed_seconds:
                self._discard_locked(
                    "controller trace terminal timestamp preceded telemetry"
                )
                return False
            try:
                self._terminal = ControllerTraceTerminal(
                    elapsed,
                    outcome,
                    reported_positions,
                    None if outcome == "completed" else _bounded_detail(detail),
                )
            except Exception as exc:
                self._discard_locked(_bounded_detail(exc))
                return False
        return True

    def _discard(self, reason):
        with self._lock:
            self._discard_locked(reason)

    def _discard_locked(self, reason):
        if self._terminal is None and self._discard_reason is None:
            self._discard_reason = _bounded_detail(reason)
            self._samples.clear()


class ControllerTraceStore:
    """Persist and analyze finalized captures on a bounded daemon worker."""

    def __init__(
        self,
        directory=CONTROLLER_TRACE_CAPTURE_DIRECTORY,
        maximum_pending=CONTROLLER_TRACE_CAPTURE_MAXIMUM_PENDING,
        maximum_files=CONTROLLER_TRACE_CAPTURE_MAXIMUM_FILES,
        maximum_total_bytes=CONTROLLER_TRACE_CAPTURE_MAXIMUM_TOTAL_BYTES,
        maximum_events=CONTROLLER_TRACE_CAPTURE_MAXIMUM_EVENTS,
    ):
        if isinstance(directory, bytes):
            raise ControllerTraceCaptureError(
                "trace directory must be a text or path-like value"
            )
        try:
            normalized_directory = Path(directory).expanduser().resolve()
        except (OSError, TypeError, ValueError) as exc:
            raise ControllerTraceCaptureError("trace directory is invalid") from exc
        self._maximum_pending = _positive_integer(
            maximum_pending,
            "maximum_pending",
        )
        self._maximum_files = _positive_integer(
            maximum_files,
            "maximum_files",
        )
        self._maximum_total_bytes = _positive_integer(
            maximum_total_bytes,
            "maximum_total_bytes",
        )
        self._maximum_events = _positive_integer(
            maximum_events,
            "maximum_events",
        )
        self._directory = normalized_directory
        self._queue = Queue(maxsize=self._maximum_pending)
        self._events = deque()
        self._discarded_event_count = 0
        self._events_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop = threading.Event()
        self._worker = None
        self._closed = False

    def submit(self, capture):
        if not isinstance(capture, ControllerTraceCapture):
            raise ControllerTraceCaptureError(
                "trace store requires ControllerTraceCapture input"
            )
        if not capture.finalized:
            raise ControllerTraceCaptureError(
                "trace capture must be finalized before persistence"
            )
        discard_reason = capture.discard_reason
        if discard_reason is not None:
            self._publish_event("dropped", discard_reason)
            return False

        with self._lifecycle_lock:
            if self._closed:
                self._publish_event("dropped", "controller trace store is closed")
                return False
            if self._worker is None:
                worker = threading.Thread(
                    target=self._run,
                    name="ar4-controller-trace-store",
                    daemon=True,
                )
                try:
                    worker.start()
                except Exception as exc:
                    self._publish_event(
                        "failed",
                        "controller trace worker could not start: "
                        f"{_bounded_detail(exc)}",
                    )
                    return False
                self._worker = worker
            try:
                self._queue.put_nowait(capture)
            except Full:
                self._publish_event(
                    "dropped",
                    "controller trace persistence queue is full",
                )
                return False
        return True

    def drain_events(self, limit=None):
        if limit is not None:
            if isinstance(limit, bool) or not isinstance(limit, int) or limit < 0:
                raise ControllerTraceCaptureError(
                    "event limit must be a non-negative integer or None"
                )
        drained = []
        with self._events_lock:
            if self._discarded_event_count and limit != 0:
                discarded = self._discarded_event_count
                self._discarded_event_count = 0
                drained.append(
                    ControllerTracePersistenceEvent(
                        "failed",
                        "controller trace persistence diagnostics were "
                        f"discarded because the event buffer was full: {discarded}",
                    )
                )
            while self._events and (limit is None or len(drained) < limit):
                drained.append(self._events.popleft())
        return drained

    def report_failure(self, context, error):
        """Queue a bounded diagnostic without raising into a motion worker."""

        try:
            context_detail = _bounded_detail(context)
            error_detail = _bounded_detail(error)
            self._publish_event(
                "failed",
                f"{context_detail}: {error_detail}",
            )
        except Exception:
            return False
        return True

    def close(self, wait_seconds=0.0):
        timeout = _nonnegative_finite(wait_seconds, "wait_seconds")
        with self._lifecycle_lock:
            self._closed = True
            self._stop.set()
            worker = self._worker
        if worker is None:
            return True
        worker.join(timeout)
        return not worker.is_alive()

    def _run(self):
        while not self._stop.is_set() or not self._queue.empty():
            try:
                capture = self._queue.get(timeout=0.1)
            except Empty:
                continue
            try:
                trace = capture.freeze()
                payload = encode_controller_trace(trace)
                path = self._write_payload(payload)
            except Exception as exc:
                self._publish_event(
                    "failed",
                    "controller trace persistence failed: "
                    f"{_bounded_detail(exc)}",
                )
            else:
                retention_error = None
                try:
                    self._prune_retention()
                except Exception as exc:
                    retention_error = exc
                try:
                    analysis = analyze_controller_trace(trace)
                except Exception as exc:
                    self._publish_event(
                        "saved",
                        "controller trace saved; profile analysis failed: "
                        f"{_bounded_detail(exc)}",
                        path=path,
                    )
                else:
                    eligibility = (
                        "eligible"
                        if analysis.profile_analysis_eligible
                        else "ineligible"
                    )
                    self._publish_event(
                        "saved",
                        f"controller trace saved; profile analysis {eligibility}",
                        path=path,
                        analysis=analysis,
                    )
                if retention_error is not None:
                    self._publish_event(
                        "failed",
                        "controller trace retention failed after persistence: "
                        f"{_bounded_detail(retention_error)}",
                        path=path,
                    )
            finally:
                self._queue.task_done()

    def _write_payload(self, payload):
        if not isinstance(payload, bytes) or not payload:
            raise ControllerTraceCaptureError(
                "encoded controller trace must be non-empty bytes"
            )
        if len(payload) > CONTROLLER_TRACE_MAXIMUM_BYTES:
            raise ControllerTraceCaptureError(
                "encoded controller trace exceeds the byte limit"
            )
        if len(payload) > self._maximum_total_bytes:
            raise ControllerTraceCaptureError(
                "encoded controller trace exceeds the retention byte limit"
            )
        self._directory.mkdir(parents=True, exist_ok=True)
        if not self._directory.is_dir():
            raise ControllerTraceCaptureError(
                "controller trace destination is not a directory"
            )
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        token = secrets.token_hex(6)
        final_path = self._directory / f"trace-{timestamp}-{token}.jsonl"
        temporary_path = self._directory / f".{final_path.name}.tmp"
        try:
            with temporary_path.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_path, final_path)
        except Exception as operation_error:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError as cleanup_error:
                raise ControllerTraceCaptureError(
                    "controller trace write failed: "
                    f"{_bounded_detail(operation_error)}; temporary cleanup "
                    f"failed: {_bounded_detail(cleanup_error)}"
                ) from operation_error
            raise
        return final_path

    def _prune_retention(self):
        candidates = []
        for path in self._directory.glob("trace-*.jsonl"):
            try:
                if path.is_symlink():
                    continue
                if not path.is_file():
                    raise ControllerTraceCaptureError(
                        f"owned controller trace path is not a file: {path.name}"
                    )
                stat_result = path.stat()
            except OSError as exc:
                raise ControllerTraceCaptureError(
                    f"owned controller trace could not be inspected: {path.name}"
                ) from exc
            candidates.append((stat_result.st_mtime_ns, path.name, path, stat_result.st_size))
        candidates.sort()
        total_bytes = sum(candidate[3] for candidate in candidates)
        while (
            len(candidates) > self._maximum_files
            or total_bytes > self._maximum_total_bytes
        ):
            _, _, path, size = candidates.pop(0)
            path.unlink()
            total_bytes -= size

    def _publish_event(self, kind, detail, path=None, analysis=None):
        event = ControllerTracePersistenceEvent(
            kind,
            detail,
            path=path,
            analysis=analysis,
        )
        with self._events_lock:
            if len(self._events) >= self._maximum_events:
                self._events.popleft()
                self._discarded_event_count += 1
            self._events.append(event)
        return event
