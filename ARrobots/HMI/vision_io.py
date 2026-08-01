"""Bounded image loading and camera-preview support for the AR4 HMI."""

from collections import deque
from dataclasses import dataclass
from io import BytesIO
import os
import stat
import threading
import time

import cv2
import numpy as np
from PIL import Image, UnidentifiedImageError

from ARrobots.HMI.joint_motion import MotionInputError


MAX_VISION_IMAGE_BYTES = 16 * 1024 * 1024
MAX_VISION_IMAGE_DIMENSION = 8192
MAX_VISION_IMAGE_PIXELS = 16 * 1024 * 1024
MAX_CAMERA_FRAME_DIMENSION = 8192
MAX_CAMERA_FRAME_PIXELS = 32 * 1024 * 1024
CAMERA_PREVIEW_WARMUP_FRAMES = 5
MAX_CAMERA_PREVIEW_WARMUP_FRAMES = 120
CAMERA_PREVIEW_READ_FAILURE_LIMIT = 3
MAX_CAMERA_PREVIEW_READ_FAILURE_LIMIT = 120
CAMERA_PREVIEW_RETRY_SECONDS = 0.05
MAX_CAMERA_PREVIEW_RETRY_SECONDS = 5.0
CAMERA_PREVIEW_RELEASE_ATTEMPTS = 3
MAX_CAMERA_PREVIEW_RELEASE_ATTEMPTS = 10
MAX_CAMERA_PREVIEW_EVENT_DETAIL = 512
MAX_CAMERA_PREVIEW_EVENTS = 64
MAX_RETAINED_CAMERA_TERMINAL_EVENTS = 32
MAX_CAMERA_SOURCE_TEXT = 512
CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS = 0.05
CAMERA_PREVIEW_EVENT_KINDS = frozenset(
    ("starting", "started", "stopping", "stopped", "failed")
)


@dataclass(frozen=True)
class CameraPreviewEvent:
    sequence: int
    kind: str
    request_id: int
    detail: str | None = None

    def __post_init__(self):
        _validate_nonnegative_integer(self.sequence, "camera event sequence")
        _validate_nonnegative_integer(self.request_id, "camera request id")
        if self.kind not in CAMERA_PREVIEW_EVENT_KINDS:
            raise MotionInputError("camera event kind is invalid")
        if self.detail is not None and (
            not isinstance(self.detail, str)
            or not self.detail
            or " ".join(self.detail.split()) != self.detail
            or self.detail != self.detail.strip()
            or "\r" in self.detail
            or "\n" in self.detail
            or len(self.detail) > MAX_CAMERA_PREVIEW_EVENT_DETAIL
        ):
            raise MotionInputError("camera event detail must be normalized text")


@dataclass(frozen=True, eq=False)
class CameraPreviewFrame:
    sequence: int
    request_id: int
    image: np.ndarray

    def __post_init__(self):
        _validate_nonnegative_integer(self.sequence, "camera frame sequence")
        _validate_nonnegative_integer(self.request_id, "camera request id")
        _validated_camera_frame(self.image, "camera preview frame")


@dataclass(frozen=True)
class CameraPreviewLifecycleState:
    active: bool
    stopped: bool
    closed: bool
    fault_reason: str | None

    def __post_init__(self):
        if not all(
            isinstance(value, bool)
            for value in (self.active, self.stopped, self.closed)
        ):
            raise MotionInputError("camera lifecycle state is invalid")
        if self.active == self.stopped:
            raise MotionInputError("camera lifecycle activity state is invalid")
        if self.fault_reason is not None and (
            not isinstance(self.fault_reason, str)
            or not self.fault_reason
            or self.fault_reason != self.fault_reason.strip()
            or " ".join(self.fault_reason.split()) != self.fault_reason
            or "\r" in self.fault_reason
            or "\n" in self.fault_reason
            or len(self.fault_reason) > MAX_CAMERA_PREVIEW_EVENT_DETAIL
        ):
            raise MotionInputError("camera lifecycle fault state is invalid")

    @property
    def clean(self):
        return self.stopped and self.fault_reason is None


@dataclass(frozen=True)
class _CameraPreviewRequest:
    request_id: int
    camera_source: int | str

    def __post_init__(self):
        _validate_nonnegative_integer(self.request_id, "camera request id")
        _validate_camera_source(self.camera_source)


def _validate_nonnegative_integer(value, field_name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise MotionInputError(f"{field_name} must be a non-negative integer")
    return value


def _validate_camera_source(value):
    if isinstance(value, int) and not isinstance(value, bool):
        if value < 0:
            raise MotionInputError(
                "camera source index must be a non-negative integer"
            )
        return value
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or "\r" in value
        or "\n" in value
        or "\x00" in value
        or len(value) > MAX_CAMERA_SOURCE_TEXT
    ):
        raise MotionInputError(
            "camera source must be a normalized device index or name"
        )
    return value


def _validate_camera_wait_timeout(value):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or value < 0
        or value > threading.TIMEOUT_MAX
    ):
        raise MotionInputError(
            "camera wait timeout must be a finite non-negative number"
        )
    return float(value)


def _camera_cancellation_requested(cancellation_event):
    if cancellation_event is None:
        return False
    is_set = getattr(cancellation_event, "is_set", None)
    if not callable(is_set):
        raise MotionInputError(
            "camera cancellation event does not satisfy the event contract"
        )
    cancelled = is_set()
    if not isinstance(cancelled, bool):
        raise MotionInputError("camera cancellation state must be boolean")
    return cancelled


def normalize_camera_exception_detail(error, prefix=""):
    """Return bounded, single-line diagnostic text for camera failures."""

    if (
        not isinstance(prefix, str)
        or prefix != prefix.lstrip()
        or "\r" in prefix
        or "\n" in prefix
    ):
        raise TypeError("camera error prefix must be normalized text")
    try:
        detail = " ".join(str(error).split())
    except Exception:
        detail = type(error).__name__
    detail = detail or type(error).__name__
    bounded = (prefix + detail)[:MAX_CAMERA_PREVIEW_EVENT_DETAIL].rstrip()
    return bounded or type(error).__name__[:MAX_CAMERA_PREVIEW_EVENT_DETAIL]


def _validated_camera_frame(image, field_name):
    if (
        not isinstance(image, np.ndarray)
        or image.ndim != 3
        or image.shape[2] != 3
        or image.dtype != np.uint8
    ):
        raise MotionInputError(
            f"{field_name} must contain three 8-bit channels"
        )
    height, width = image.shape[:2]
    if width <= 0 or height <= 0:
        raise MotionInputError(f"{field_name} dimensions are invalid")
    if (
        width > MAX_CAMERA_FRAME_DIMENSION
        or height > MAX_CAMERA_FRAME_DIMENSION
    ):
        raise MotionInputError(f"{field_name} dimensions exceed the limit")
    if width * height > MAX_CAMERA_FRAME_PIXELS:
        raise MotionInputError(f"{field_name} pixel count exceeds the limit")
    return image


def copy_camera_capture_frame(image):
    """Validate camera output and detach storage owned by the backend."""

    source = _validated_camera_frame(image, "camera capture frame")
    return np.array(source, dtype=np.uint8, copy=True, order="C")


def prepare_camera_preview_frame(image, width=480, height=320):
    """Convert an untrusted BGR capture into an owned RGB preview frame."""

    _validate_nonnegative_integer(width, "camera preview width")
    _validate_nonnegative_integer(height, "camera preview height")
    if width == 0 or height == 0:
        raise MotionInputError("camera preview dimensions must be positive")
    _validated_image_dimensions(width, height, "camera preview")
    source = _validated_camera_frame(image, "camera capture frame")
    try:
        resized = cv2.resize(
            source,
            (width, height),
            interpolation=(
                cv2.INTER_AREA
                if source.shape[1] >= width and source.shape[0] >= height
                else cv2.INTER_LINEAR
            ),
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    except cv2.error as exc:
        raise MotionInputError("camera preview conversion failed") from exc
    return np.array(
        _validated_camera_frame(rgb, "camera preview frame"),
        dtype=np.uint8,
        copy=True,
        order="C",
    )


class CameraPreviewWorker:
    """Own camera acquisition off Tk and retain only the latest frame."""

    def __init__(
        self,
        capture_factory,
        frame_transform=prepare_camera_preview_frame,
        *,
        warmup_frames=CAMERA_PREVIEW_WARMUP_FRAMES,
        read_failure_limit=CAMERA_PREVIEW_READ_FAILURE_LIMIT,
        retry_seconds=CAMERA_PREVIEW_RETRY_SECONDS,
        release_attempts=CAMERA_PREVIEW_RELEASE_ATTEMPTS,
        thread_factory=threading.Thread,
    ):
        if not callable(capture_factory):
            raise MotionInputError("camera capture factory must be callable")
        if not callable(frame_transform):
            raise MotionInputError("camera frame transform must be callable")
        _validate_nonnegative_integer(warmup_frames, "camera warmup frame count")
        if warmup_frames > MAX_CAMERA_PREVIEW_WARMUP_FRAMES:
            raise MotionInputError("camera warmup frame count exceeds the limit")
        _validate_nonnegative_integer(
            read_failure_limit,
            "camera read failure limit",
        )
        if read_failure_limit == 0:
            raise MotionInputError("camera read failure limit must be positive")
        if read_failure_limit > MAX_CAMERA_PREVIEW_READ_FAILURE_LIMIT:
            raise MotionInputError("camera read failure count exceeds the limit")
        if (
            isinstance(retry_seconds, bool)
            or not isinstance(retry_seconds, (int, float))
            or not np.isfinite(retry_seconds)
            or retry_seconds < 0
            or retry_seconds > MAX_CAMERA_PREVIEW_RETRY_SECONDS
        ):
            raise MotionInputError(
                "camera retry interval must be a finite number between zero "
                f"and {MAX_CAMERA_PREVIEW_RETRY_SECONDS} seconds"
            )
        _validate_nonnegative_integer(
            release_attempts,
            "camera release attempt count",
        )
        if release_attempts == 0:
            raise MotionInputError("camera release attempt count must be positive")
        if release_attempts > MAX_CAMERA_PREVIEW_RELEASE_ATTEMPTS:
            raise MotionInputError("camera release attempt count exceeds the limit")
        if not callable(thread_factory):
            raise MotionInputError("camera thread factory must be callable")

        self._capture_factory = capture_factory
        self._frame_transform = frame_transform
        self._warmup_frames = warmup_frames
        self._read_failure_limit = read_failure_limit
        self._retry_seconds = float(retry_seconds)
        self._release_attempts = release_attempts
        self._thread_factory = thread_factory
        self._lock = threading.Lock()
        self._state_changed = threading.Condition(self._lock)
        self._request_changed = threading.Event()
        self._worker_stopped = threading.Event()
        self._worker_stopped.set()
        self._events = deque(maxlen=MAX_CAMERA_PREVIEW_EVENTS)
        self._next_event_sequence = 0
        self._next_frame_sequence = 0
        self._next_request_id = 0
        self._desired = None
        self._active_request_id = None
        self._latest_preview = None
        self._latest_raw_frame = None
        self._worker = None
        self._closed = False
        self._fault_reason = None

    @property
    def active(self):
        with self._lock:
            return self._worker is not None

    @property
    def closed(self):
        with self._lock:
            return self._closed

    @property
    def fault_reason(self):
        with self._lock:
            return self._fault_reason

    @property
    def desired_request_id(self):
        with self._lock:
            return None if self._desired is None else self._desired.request_id

    @property
    def active_request_id(self):
        with self._lock:
            return self._active_request_id

    def request_start(self, camera_source, *, replace=True):
        camera_source = _validate_camera_source(camera_source)
        if not isinstance(replace, bool):
            raise MotionInputError("camera replacement option must be boolean")
        with self._lock:
            if self._closed:
                raise MotionInputError("camera preview worker is closed")
            if self._fault_reason is not None:
                raise MotionInputError(
                    "camera preview worker requires an application restart: "
                    + self._fault_reason
                )
            if not replace and self._worker is not None:
                raise MotionInputError("camera capture worker is busy")
            self._next_request_id += 1
            request = _CameraPreviewRequest(
                request_id=self._next_request_id,
                camera_source=camera_source,
            )
            self._desired = request
            self._latest_preview = None
            self._latest_raw_frame = None
            self._retain_terminal_events_locked()
            self._append_event_locked("starting", request)
            self._request_changed.set()
            self._state_changed.notify_all()
            if self._worker is not None:
                return request.request_id

            self._worker_stopped.clear()
            try:
                worker = self._thread_factory(
                    target=self._run,
                    name="ar4-camera-preview",
                    daemon=True,
                )
            except Exception as exc:
                self._desired = None
                self._worker_stopped.set()
                detail = normalize_camera_exception_detail(
                    exc,
                    "camera thread creation failed: ",
                )
                self._append_event_locked("failed", request, detail)
                self._state_changed.notify_all()
                raise RuntimeError(detail) from exc
            if not isinstance(worker, threading.Thread):
                self._desired = None
                self._worker_stopped.set()
                detail = "camera thread factory returned an invalid worker"
                self._append_event_locked("failed", request, detail)
                self._state_changed.notify_all()
                raise MotionInputError(detail)
            self._worker = worker
            try:
                worker.start()
            except Exception as exc:
                self._worker = None
                self._desired = None
                self._worker_stopped.set()
                detail = normalize_camera_exception_detail(
                    exc,
                    "camera preview worker startup failed: ",
                )
                self._append_event_locked("failed", request, detail)
                self._state_changed.notify_all()
                raise RuntimeError(detail) from exc
            return request.request_id

    def request_stop(self, request_id=None):
        if request_id is not None:
            _validate_nonnegative_integer(request_id, "camera request id")
        with self._lock:
            request = self._desired
            if (
                request is None
                or (
                    request_id is not None
                    and request.request_id != request_id
                )
            ):
                return False
            self._desired = None
            self._latest_preview = None
            self._latest_raw_frame = None
            self._append_event_locked("stopping", request)
            if self._active_request_id != request.request_id:
                self._append_event_locked("stopped", request)
            self._request_changed.set()
            self._state_changed.notify_all()
            return True

    def _close_locked(self):
        self._closed = True
        request = self._desired
        self._desired = None
        self._latest_preview = None
        self._latest_raw_frame = None
        if request is not None:
            self._append_event_locked("stopping", request)
            if self._active_request_id != request.request_id:
                self._append_event_locked("stopped", request)
        self._request_changed.set()
        self._state_changed.notify_all()

    def _lifecycle_state_locked(self):
        return CameraPreviewLifecycleState(
            active=self._worker is not None,
            stopped=self._worker_stopped.is_set(),
            closed=self._closed,
            fault_reason=self._fault_reason,
        )

    def close_state(self):
        """Close acquisition and return one atomic lifecycle snapshot."""

        with self._lock:
            self._close_locked()
            return self._lifecycle_state_locked()

    def close(self):
        return self.close_state().clean

    def wait_stopped(self, timeout=None):
        """Wait for worker retirement from a non-Tk lifecycle owner."""

        if timeout is not None:
            timeout = _validate_camera_wait_timeout(timeout)
        return self._worker_stopped.wait(timeout)

    def wait_ready(self, request_id, timeout, cancellation_event=None):
        """Wait off Tk until a request owns a validated raw frame."""

        _validate_nonnegative_integer(request_id, "camera request id")
        timeout = _validate_camera_wait_timeout(timeout)
        deadline = time.monotonic() + timeout
        with self._state_changed:
            while True:
                if _camera_cancellation_requested(cancellation_event):
                    return False
                if (
                    self._latest_raw_frame is not None
                    and self._latest_raw_frame[0] == request_id
                ):
                    return True
                if self._fault_reason is not None:
                    raise MotionInputError(
                        "camera preview cleanup requires an application "
                        "restart: " + self._fault_reason
                    )
                desired_request_id = (
                    None
                    if self._desired is None
                    else self._desired.request_id
                )
                if (
                    desired_request_id != request_id
                    and self._active_request_id != request_id
                ):
                    raise MotionInputError(
                        "camera preview request ended before readiness"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                if cancellation_event is not None:
                    remaining = min(
                        remaining,
                        CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS,
                    )
                self._state_changed.wait(remaining)

    def wait_request_stopped(
        self,
        request_id,
        timeout,
        cancellation_event=None,
        *,
        require_idle=False,
    ):
        """Wait off Tk for request cleanup without joining the worker."""

        _validate_nonnegative_integer(request_id, "camera request id")
        timeout = _validate_camera_wait_timeout(timeout)
        if not isinstance(require_idle, bool):
            raise MotionInputError("camera idle requirement must be boolean")
        deadline = time.monotonic() + timeout
        with self._state_changed:
            while True:
                if _camera_cancellation_requested(cancellation_event):
                    return False
                if self._fault_reason is not None:
                    raise MotionInputError(
                        "camera preview cleanup requires an application "
                        "restart: " + self._fault_reason
                    )
                desired_request_id = (
                    None
                    if self._desired is None
                    else self._desired.request_id
                )
                if (
                    require_idle
                    and desired_request_id is not None
                    and desired_request_id != request_id
                ):
                    raise MotionInputError(
                        "camera request ownership changed before idle"
                    )
                if (
                    desired_request_id != request_id
                    and self._active_request_id != request_id
                ):
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                if cancellation_event is not None:
                    remaining = min(
                        remaining,
                        CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS,
                    )
                self._state_changed.wait(remaining)

    def capture_once(self, camera_source, timeout, cancellation_event=None):
        """Acquire one validated frame through the owned worker lifecycle."""

        timeout = _validate_camera_wait_timeout(timeout)
        if _camera_cancellation_requested(cancellation_event):
            raise MotionInputError("camera capture was cancelled")
        request_id = self.request_start(camera_source, replace=False)
        try:
            ready = self.wait_ready(
                request_id,
                timeout,
                cancellation_event,
            )
            if not isinstance(ready, bool):
                raise RuntimeError(
                    "camera worker returned an invalid readiness result"
                )
            if not ready:
                if _camera_cancellation_requested(cancellation_event):
                    raise MotionInputError("camera capture was cancelled")
                raise MotionInputError(
                    "camera capture did not become ready before timeout"
                )
            frame = self.snapshot_raw_frame(request_id)
            if frame is None:
                raise MotionInputError(
                    "camera capture lost request ownership before snapshot"
                )
            return frame
        finally:
            cleanup_error = None
            try:
                self.request_stop(request_id)
                settled = self.wait_request_stopped(request_id, timeout)
                if not isinstance(settled, bool):
                    raise RuntimeError(
                        "camera worker returned an invalid cleanup result"
                    )
                if not settled:
                    cleanup_error = "camera capture cleanup timed out"
            except Exception as exc:
                cleanup_error = normalize_camera_exception_detail(
                    exc,
                    "camera capture cleanup failed: ",
                )
            if cleanup_error is not None:
                self.close()
                raise MotionInputError(cleanup_error)

    def drain_events(self):
        with self._lock:
            events = tuple(self._events)
            self._events.clear()
            return events

    def take_latest_frame(self, request_id):
        _validate_nonnegative_integer(request_id, "camera request id")
        with self._lock:
            frame = self._latest_preview
            if frame is None or frame.request_id != request_id:
                return None
            self._latest_preview = None
            return frame

    def snapshot_raw_frame(self, request_id):
        _validate_nonnegative_integer(request_id, "camera request id")
        with self._lock:
            if (
                self._latest_raw_frame is None
                or self._latest_raw_frame[0] != request_id
            ):
                return None
            frame = self._latest_raw_frame[1]
        return np.array(frame, dtype=np.uint8, copy=True, order="C")

    def _retain_terminal_events_locked(self):
        retained = tuple(
            event
            for event in self._events
            if event.kind in ("failed", "stopped")
        )[-MAX_RETAINED_CAMERA_TERMINAL_EVENTS:]
        self._events.clear()
        self._events.extend(retained)

    def _append_event_locked(self, kind, request, detail=None):
        event = CameraPreviewEvent(
            sequence=self._next_event_sequence,
            kind=kind,
            request_id=request.request_id,
            detail=detail,
        )
        self._next_event_sequence += 1
        self._events.append(event)
        return event

    def _request_is_current(self, request):
        with self._lock:
            return self._desired == request and not self._closed

    def _wait_for_change(self, request):
        self._request_changed.clear()
        if not self._request_is_current(request):
            return False
        return self._request_changed.wait(self._retry_seconds)

    @staticmethod
    def _validated_capture(capture):
        if not callable(getattr(capture, "read", None)):
            raise MotionInputError("camera capture has no read operation")
        if not callable(getattr(capture, "release", None)):
            raise MotionInputError("camera capture has no release operation")
        opened = getattr(capture, "isOpened", None)
        if callable(opened):
            open_state = opened()
            if not isinstance(open_state, (bool, np.bool_)):
                raise MotionInputError("camera open state is invalid")
            if not bool(open_state):
                raise MotionInputError("camera device did not open")
        return capture

    @staticmethod
    def _read_frame(capture):
        result = capture.read()
        if not isinstance(result, (tuple, list)) or len(result) != 2:
            raise MotionInputError("camera read result is invalid")
        succeeded, frame = result
        if not isinstance(succeeded, (bool, np.bool_)):
            raise MotionInputError("camera read status is invalid")
        if not bool(succeeded):
            if frame is not None:
                raise MotionInputError(
                    "failed camera read returned unexpected frame data"
                )
            return None
        return copy_camera_capture_frame(frame)

    def _release_capture(self, capture):
        last_error = None
        for attempt in range(self._release_attempts):
            try:
                capture.release()
                opened = getattr(capture, "isOpened", None)
                if callable(opened):
                    open_state = opened()
                    if not isinstance(open_state, (bool, np.bool_)):
                        raise MotionInputError("camera open state is invalid")
                    if bool(open_state):
                        raise MotionInputError("camera device remained open")
                return None
            except Exception as exc:
                last_error = exc
                if attempt + 1 < self._release_attempts:
                    time.sleep(self._retry_seconds)
        return normalize_camera_exception_detail(
            last_error,
            "camera release failed: ",
        )

    def _publish_frame(self, request, raw_frame):
        raw_frame.setflags(write=False)
        preview = self._frame_transform(raw_frame)
        preview = np.array(
            _validated_camera_frame(preview, "camera preview frame"),
            dtype=np.uint8,
            copy=True,
            order="C",
        )
        preview.setflags(write=False)
        with self._lock:
            if self._desired != request or self._closed:
                return False
            frame = CameraPreviewFrame(
                sequence=self._next_frame_sequence,
                request_id=request.request_id,
                image=preview,
            )
            self._next_frame_sequence += 1
            first_frame = self._latest_raw_frame is None
            self._latest_raw_frame = (request.request_id, raw_frame)
            self._latest_preview = frame
            if first_frame:
                self._append_event_locked("started", request)
            self._state_changed.notify_all()
            return True

    def _publish_request_failure(self, request, error):
        detail = normalize_camera_exception_detail(
            error,
            "camera preview failed: ",
        )
        with self._lock:
            if self._desired == request:
                self._desired = None
                self._latest_preview = None
                self._latest_raw_frame = None
            self._append_event_locked("failed", request, detail)
            self._request_changed.set()
            self._state_changed.notify_all()
        return detail

    def _run_session(self, request):
        capture = None
        try:
            capture = self._capture_factory(request.camera_source)
            self._validated_capture(capture)
            warmup_remaining = self._warmup_frames
            consecutive_failures = 0
            while self._request_is_current(request):
                frame = self._read_frame(capture)
                if frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= self._read_failure_limit:
                        raise MotionInputError(
                            "camera exceeded the consecutive read-failure limit"
                        )
                    self._wait_for_change(request)
                    continue
                consecutive_failures = 0
                if warmup_remaining:
                    warmup_remaining -= 1
                    continue
                self._publish_frame(request, frame)
        except Exception as exc:
            self._publish_request_failure(request, exc)
        finally:
            release_error = None
            if capture is not None:
                release_error = self._release_capture(capture)
            with self._lock:
                if release_error is not None:
                    affected_request = self._desired or request
                    self._fault_reason = release_error
                    self._desired = None
                    self._latest_preview = None
                    self._latest_raw_frame = None
                    self._append_event_locked(
                        "failed",
                        affected_request,
                        release_error,
                    )
                self._append_event_locked("stopped", request)
                if self._active_request_id == request.request_id:
                    self._active_request_id = None
                self._request_changed.set()
                self._state_changed.notify_all()

    def _run(self):
        current_thread = threading.current_thread()
        try:
            while True:
                with self._lock:
                    request = self._desired
                    if request is None or self._closed:
                        if self._worker is current_thread:
                            self._worker = None
                            self._worker_stopped.set()
                            self._active_request_id = None
                        self._request_changed.set()
                        self._state_changed.notify_all()
                        break
                    self._active_request_id = request.request_id
                    self._state_changed.notify_all()
                self._run_session(request)
        except BaseException as exc:
            detail = normalize_camera_exception_detail(
                exc,
                "camera preview worker terminated: ",
            )
            with self._lock:
                if self._worker is current_thread:
                    request = self._desired
                    self._desired = None
                    self._active_request_id = None
                    self._latest_preview = None
                    self._latest_raw_frame = None
                    self._fault_reason = detail
                    if request is not None:
                        self._append_event_locked("failed", request, detail)
                    self._state_changed.notify_all()
        finally:
            with self._lock:
                if self._worker is current_thread:
                    self._worker = None
                    self._worker_stopped.set()
                    self._active_request_id = None
                self._request_changed.set()
                self._state_changed.notify_all()


def _validated_image_dimensions(width, height, field_name):
    if (
        isinstance(width, bool)
        or isinstance(height, bool)
        or not isinstance(width, int)
        or not isinstance(height, int)
        or width <= 0
        or height <= 0
    ):
        raise MotionInputError(f"{field_name} dimensions are invalid")
    if width > MAX_VISION_IMAGE_DIMENSION or height > MAX_VISION_IMAGE_DIMENSION:
        raise MotionInputError(f"{field_name} dimensions exceed the limit")
    if width * height > MAX_VISION_IMAGE_PIXELS:
        raise MotionInputError(f"{field_name} pixel count exceeds the limit")
    return width, height


def _read_regular_image_bytes(filename, field_name):
    if not isinstance(field_name, str) or not field_name.strip():
        raise TypeError("image field name must be nonempty text")
    try:
        candidate = os.fspath(filename)
    except TypeError as exc:
        raise MotionInputError(f"{field_name} path is invalid") from exc
    if isinstance(candidate, bytes):
        candidate = os.fsdecode(candidate)
    if not isinstance(candidate, str) or not candidate:
        raise MotionInputError(f"{field_name} path is invalid")

    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0)
    no_follow = getattr(os, "O_NOFOLLOW", 0)
    if isinstance(no_follow, int):
        flags |= no_follow
    descriptor = None
    try:
        path_state = os.stat(candidate, follow_symlinks=False)
        if not stat.S_ISREG(path_state.st_mode):
            raise MotionInputError(f"{field_name} must be a regular file")
        if path_state.st_size <= 0:
            raise MotionInputError(f"{field_name} is empty")
        if path_state.st_size > MAX_VISION_IMAGE_BYTES:
            raise MotionInputError(f"{field_name} exceeds the file-size limit")
        descriptor = os.open(candidate, flags)
        file_state = os.fstat(descriptor)
        if not stat.S_ISREG(file_state.st_mode):
            raise MotionInputError(f"{field_name} must be a regular file")
        if (
            file_state.st_dev != path_state.st_dev
            or file_state.st_ino != path_state.st_ino
            or file_state.st_size != path_state.st_size
        ):
            raise MotionInputError(f"{field_name} changed before being read")
        if file_state.st_size <= 0:
            raise MotionInputError(f"{field_name} is empty")
        if file_state.st_size > MAX_VISION_IMAGE_BYTES:
            raise MotionInputError(f"{field_name} exceeds the file-size limit")
        with os.fdopen(descriptor, "rb", closefd=True) as source:
            descriptor = None
            payload = source.read(MAX_VISION_IMAGE_BYTES + 1)
            final_state = os.fstat(source.fileno())
        if len(payload) > MAX_VISION_IMAGE_BYTES:
            raise MotionInputError(f"{field_name} exceeds the file-size limit")
        if (
            not stat.S_ISREG(final_state.st_mode)
            or final_state.st_dev != file_state.st_dev
            or final_state.st_ino != file_state.st_ino
            or final_state.st_size != file_state.st_size
            or len(payload) != file_state.st_size
        ):
            raise MotionInputError(f"{field_name} changed while being read")
        return payload
    except MotionInputError:
        raise
    except OSError as exc:
        raise MotionInputError(f"{field_name} could not be read") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def load_bounded_vision_image(filename, decode_mode, field_name):
    if isinstance(decode_mode, bool) or not isinstance(decode_mode, int):
        raise TypeError("image decode mode must be an integer")
    if decode_mode not in (
        cv2.IMREAD_COLOR,
        cv2.IMREAD_GRAYSCALE,
        cv2.IMREAD_UNCHANGED,
    ):
        raise MotionInputError("image decode mode is unsupported")
    payload = _read_regular_image_bytes(filename, field_name)
    try:
        with Image.open(BytesIO(payload)) as header:
            header_width, header_height = _validated_image_dimensions(
                header.width,
                header.height,
                field_name,
            )
            header.verify()
    except MotionInputError:
        raise
    except (
        Image.DecompressionBombError,
        Image.DecompressionBombWarning,
        OSError,
        SyntaxError,
        UnidentifiedImageError,
    ) as exc:
        raise MotionInputError(f"{field_name} could not be decoded") from exc

    encoded = np.frombuffer(payload, dtype=np.uint8)
    decode_flags = decode_mode
    if decode_mode != cv2.IMREAD_UNCHANGED:
        decode_flags |= cv2.IMREAD_IGNORE_ORIENTATION
    try:
        image = cv2.imdecode(encoded, decode_flags)
    except cv2.error as exc:
        raise MotionInputError(f"{field_name} could not be decoded") from exc
    if image is None:
        raise MotionInputError(f"{field_name} could not be decoded")
    if not isinstance(image, np.ndarray) or image.ndim not in (2, 3):
        raise MotionInputError(f"{field_name} decoded shape is invalid")
    decoded_height, decoded_width = image.shape[:2]
    _validated_image_dimensions(decoded_width, decoded_height, field_name)
    if (decoded_width, decoded_height) != (header_width, header_height):
        raise MotionInputError(f"{field_name} decoded dimensions changed")
    return image


def fit_vision_preview_square(image, target_size):
    if (
        isinstance(target_size, bool)
        or not isinstance(target_size, int)
        or target_size <= 0
        or target_size > MAX_VISION_IMAGE_DIMENSION
    ):
        raise MotionInputError("vision preview size is invalid")
    if (
        not isinstance(image, np.ndarray)
        or image.ndim != 3
        or image.shape[2] != 3
        or image.dtype != np.uint8
    ):
        raise MotionInputError(
            "vision preview image must contain three 8-bit channels"
        )
    height, width = image.shape[:2]
    _validated_image_dimensions(width, height, "vision preview image")

    if width >= height:
        resized_width = target_size
        resized_height = max(1, round(height * target_size / width))
    else:
        resized_height = target_size
        resized_width = max(1, round(width * target_size / height))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA,
    )
    square = np.zeros((target_size, target_size, 3), dtype=np.uint8)
    x_offset = (target_size - resized_width) // 2
    y_offset = (target_size - resized_height) // 2
    square[
        y_offset:y_offset + resized_height,
        x_offset:x_offset + resized_width,
    ] = resized
    return square
