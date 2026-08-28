"""Bounded serial byte transport for correlated JSON sessions.

The transport owns framing and serial timeout mutation only. No reader thread
starts here; one active caller-owned worker polls complete frames, durably
acknowledges each caller handoff, and passes each frame to
:class:`CorrelatedJsonSession`. Sequential worker ownership can transfer only
through an explicit complete-frame-boundary release.
"""

import math
import threading
import time

from .messages import (
    JSON_PROTOCOL_MAXIMUM_FRAME_BYTES,
    JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
    JsonProtocolError,
    Request,
    decode_message,
)


# A maximum protocol frame at the configured 9600 baud needs about 4.27 seconds.
JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS = 5.0
JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS = 5.0
JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS = 0.05
JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES = 256
JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS = 0.001
_JSON_SERIAL_MAXIMUM_CLOCK_STALL_SECONDS = 1.0


class JsonSerialTransportError(RuntimeError):
    """Base error for the bounded JSON serial transport."""


class JsonSerialTransportAdmissionError(JsonSerialTransportError):
    """Transport construction or caller input is invalid."""


class JsonSerialTransportDeadlineError(JsonSerialTransportError):
    """An outbound deadline expired before a complete write."""


class JsonSerialTransportQuarantinedError(JsonSerialTransportError):
    """Serial framing or I/O ownership can no longer be trusted."""


class JsonSerialTransportFrameTimeoutError(
    JsonSerialTransportQuarantinedError
):
    """A started inbound frame missed the bounded assembly deadline."""


class JsonSerialTransportReadDeferredError(JsonSerialTransportError):
    """Inbound polling yielded to bounded serial I/O ownership."""

    def __init__(self, deferred_for):
        super().__init__(
            "serial frame read deferred by active serial I/O ownership"
        )
        self.deferred_for = deferred_for


class _JsonSerialReadReservation:
    __slots__ = ()


class _JsonSerialReaderReleaseToken:
    __slots__ = ("owner",)

    def __init__(self, owner):
        self.owner = owner


class _JsonSerialFrameHandoff:
    __slots__ = ("frame",)

    def __init__(self, frame):
        self.frame = frame


def _positive_finite(value, field_name):
    if type(value) not in (int, float):
        raise JsonSerialTransportAdmissionError(
            f"{field_name} must be a finite number"
        )
    try:
        normalized = float(value)
    except (OverflowError, ValueError) as exc:
        raise JsonSerialTransportAdmissionError(
            f"{field_name} must be a finite number"
        ) from exc
    if not math.isfinite(normalized) or normalized <= 0:
        raise JsonSerialTransportAdmissionError(
            f"{field_name} must be positive and finite"
        )
    return normalized


def _clock_value(value):
    if type(value) not in (int, float):
        raise ValueError("transport clock must return a finite number")
    try:
        normalized = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError("transport clock must return a finite number") from exc
    if not math.isfinite(normalized):
        raise ValueError("transport clock must return a finite number")
    return normalized


def _validated_outbound_frame(frame):
    if type(frame) is not bytes:
        raise JsonSerialTransportAdmissionError(
            "outbound protocol frame must use exact bytes"
        )
    if (
        not frame
        or len(frame) > JSON_PROTOCOL_MAXIMUM_FRAME_BYTES
        or not frame.endswith(b"\n")
        or b"\n" in frame[:-1]
    ):
        raise JsonSerialTransportAdmissionError(
            "outbound protocol frame has invalid framing"
        )
    payload = frame[:-1]
    if (
        not payload
        or len(payload) > JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES
        or any(byte < 0x20 or byte > 0x7E for byte in payload)
    ):
        raise JsonSerialTransportAdmissionError(
            "outbound protocol payload violates the JSON byte boundary"
        )
    try:
        message = decode_message(frame)
    except JsonProtocolError as exc:
        raise JsonSerialTransportAdmissionError(
            "outbound protocol frame is not a valid JSON request"
        ) from exc
    if type(message) is not Request:
        raise JsonSerialTransportAdmissionError(
            "outbound protocol frame must contain a request envelope"
        )
    return frame


def _validate_inbound_frame(frame):
    payload = frame[:-1]
    if payload.endswith(b"\r"):
        payload = payload[:-1]
    if (
        not payload
        or len(payload) > JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES
        or any(byte < 0x20 or byte > 0x7E for byte in payload)
    ):
        raise ValueError("inbound protocol payload is invalid")


class JsonSerialTransport:
    """Adapt one open pyserial-compatible handle to JSON frame operations.

    ``write_frame`` matches the callback required by
    :class:`CorrelatedJsonSession`. ``poll_frame`` performs at most one serial
    read and returns either one complete frame or ``None``.  Split frames stay
    bounded and must complete before the configured assembly deadline.
    Unresolved I/O contention raises
    :class:`JsonSerialTransportReadDeferredError` after one bounded poll
    interval instead of reporting an idle line. A bound session and transport
    must use the same monotonic clock source because ``write_frame`` receives
    the session's absolute deadline.
    """

    def __init__(
        self,
        serial_port,
        *,
        clock=time.monotonic,
        sleeper=time.sleep,
        poll_interval=JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS,
        frame_timeout=JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS,
        write_timeout=JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS,
        read_chunk_bytes=JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES,
        drain_poll_interval=JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS,
    ):
        if not callable(clock):
            raise JsonSerialTransportAdmissionError(
                "transport clock must be callable"
            )
        if not callable(sleeper):
            raise JsonSerialTransportAdmissionError(
                "transport sleeper must be callable"
            )
        normalized_poll_interval = _positive_finite(
            poll_interval,
            "serial poll interval",
        )
        normalized_frame_timeout = _positive_finite(
            frame_timeout,
            "serial frame timeout",
        )
        normalized_write_timeout = _positive_finite(
            write_timeout,
            "serial write timeout",
        )
        normalized_drain_poll_interval = _positive_finite(
            drain_poll_interval,
            "serial drain poll interval",
        )
        if type(read_chunk_bytes) is not int or not (
            1 <= read_chunk_bytes <= JSON_PROTOCOL_MAXIMUM_FRAME_BYTES
        ):
            raise JsonSerialTransportAdmissionError(
                "serial read chunk size is invalid"
            )
        try:
            read = getattr(serial_port, "read")
            write = getattr(serial_port, "write")
            open_state = getattr(serial_port, "is_open")
            input_pending = getattr(serial_port, "in_waiting")
            output_pending = getattr(serial_port, "out_waiting")
            getattr(serial_port, "timeout")
            getattr(serial_port, "write_timeout")
        except Exception as exc:
            raise JsonSerialTransportAdmissionError(
                "serial connection does not satisfy the JSON transport contract"
            ) from exc
        if not callable(read) or not callable(write):
            raise JsonSerialTransportAdmissionError(
                "serial connection does not satisfy the JSON transport contract"
            )
        if type(open_state) is not bool or not open_state:
            raise JsonSerialTransportAdmissionError(
                "serial connection must be open"
            )
        if type(input_pending) is not int or input_pending < 0:
            raise JsonSerialTransportAdmissionError(
                "serial input queue state is invalid"
            )
        if type(output_pending) is not int or output_pending < 0:
            raise JsonSerialTransportAdmissionError(
                "serial output queue state is invalid"
            )
        try:
            initial_clock = _clock_value(clock())
        except Exception as exc:
            raise JsonSerialTransportAdmissionError(
                "transport clock failed during construction"
            ) from exc

        self._serial_port = serial_port
        self._read = read
        self._write = write
        self._clock = clock
        self._sleeper = sleeper
        self._poll_interval = normalized_poll_interval
        self._frame_timeout = normalized_frame_timeout
        self._write_timeout = normalized_write_timeout
        self._read_chunk_bytes = read_chunk_bytes
        self._drain_poll_interval = normalized_drain_poll_interval
        self._last_clock = initial_clock
        self._buffer = bytearray()
        self._partial_deadline = None
        self._frame_handoff = None
        self._last_acknowledged_frame = None
        self._reader_binding = None
        self._quarantine_reason = None
        self._state_lock = threading.RLock()
        self._clock_lock = threading.Lock()
        self._read_reservation = None
        self._write_lock = threading.Lock()
        self._serial_io_lock = threading.Lock()

    @property
    def quarantined(self):
        with self._state_lock:
            return self._quarantine_reason is not None

    @property
    def quarantine_reason(self):
        with self._state_lock:
            return self._quarantine_reason

    @property
    def buffered_input_bytes(self):
        """Return bytes retained by framing or caller-handoff ownership."""
        with self._state_lock:
            handoff_bytes = (
                len(self._frame_handoff.frame)
                if type(self._frame_handoff) is _JsonSerialFrameHandoff
                else 0
            )
            return len(self._buffer) + handoff_bytes

    @property
    def frame_handoff_pending(self):
        with self._state_lock:
            return type(self._frame_handoff) is _JsonSerialFrameHandoff

    def frame_acknowledgement_complete(self, frame):
        """Confirm exact completion of the latest frame acknowledgement."""
        with self._state_lock:
            return (
                type(frame) is bytes
                and self._frame_handoff is None
                and self._last_acknowledged_frame is frame
            )

    @property
    def reader_owner(self):
        with self._state_lock:
            if type(self._reader_binding) is _JsonSerialReaderReleaseToken:
                return self._reader_binding.owner
            return self._reader_binding

    def _bind_reader_locked(self):
        reader_thread = threading.current_thread()
        if self._reader_binding is None:
            self._reader_binding = reader_thread
        elif type(self._reader_binding) is _JsonSerialReaderReleaseToken:
            self._mark_quarantined_locked(
                "serial reader release remained unsettled"
            )
            self._raise_retained_quarantine_locked()
        elif self._reader_binding is not reader_thread:
            self._mark_quarantined_locked(
                "serial frame reader ownership changed threads"
            )
            self._raise_retained_quarantine_locked()

    def _begin_read_reservation(self, token, *, contention_quarantines):
        if (
            type(token) is not _JsonSerialReadReservation
            or type(contention_quarantines) is not bool
        ):
            raise JsonSerialTransportAdmissionError(
                "serial read reservation is invalid"
            )
        with self._state_lock:
            self._raise_retained_quarantine_locked()
            owner = self._read_reservation
            if owner is not None:
                if type(owner) is not _JsonSerialReadReservation:
                    self._mark_quarantined_locked(
                        "serial read reservation owner is invalid"
                    )
                    self._raise_retained_quarantine_locked()
                if contention_quarantines:
                    self._mark_quarantined_locked(
                        "serial frame reads became concurrent"
                    )
                    self._raise_retained_quarantine_locked()
                raise JsonSerialTransportAdmissionError(
                    "serial reader lifecycle operation overlaps an active "
                    "reservation"
                )
            self._read_reservation = token
            if contention_quarantines:
                self._bind_reader_locked()
        return token

    def _force_end_read_reservation(self, token):
        if type(token) is not _JsonSerialReadReservation:
            raise JsonSerialTransportAdmissionError(
                "serial read reservation is invalid"
            )
        with self._state_lock:
            if self._read_reservation is token:
                self._read_reservation = None

    def _run_read_reservation_operation(self, reservation, operation, *args):
        try:
            try:
                return operation(reservation, *args)
            finally:
                self._force_end_read_reservation(reservation)
        finally:
            self._force_end_read_reservation(reservation)

    def _validate_reader_release_reserved(self, reservation):
        release_valid = False
        self._begin_read_reservation(
            reservation,
            contention_quarantines=False,
        )
        self._ensure_open()
        with self._state_lock:
            self._raise_retained_quarantine_locked()
            reader_binding = self._reader_binding
            if type(reader_binding) is _JsonSerialReaderReleaseToken:
                self._mark_quarantined_locked(
                    "serial reader release remained unsettled"
                )
                self._raise_retained_quarantine_locked()
            elif reader_binding is None:
                release_valid = False
            else:
                if reader_binding is not threading.current_thread():
                    raise JsonSerialTransportAdmissionError(
                        "serial reader ownership can only be released by "
                        "its current reader"
                    )
                if self._buffer or self._partial_deadline is not None:
                    raise JsonSerialTransportAdmissionError(
                        "serial reader ownership requires a complete "
                        "frame boundary"
                    )
                if self._frame_handoff is not None:
                    raise JsonSerialTransportAdmissionError(
                        "serial reader ownership requires acknowledged "
                        "frame handoff"
                    )
                release_valid = True
        return release_valid

    def validate_reader_release(self):
        """Validate release at a complete JSON frame boundary."""
        reservation = _JsonSerialReadReservation()
        return self._run_read_reservation_operation(
            reservation,
            self._validate_reader_release_reserved,
        )

    def _settle_reader_release(self, token):
        with self._state_lock:
            if self._reader_binding is not token:
                raise JsonSerialTransportAdmissionError(
                    "serial reader release ownership changed before settlement"
                )
            self._reader_binding = None
            self._last_acknowledged_frame = None
        return True

    def _release_reader_reserved(self, reservation, release_token):
        mutation_started = False
        released = False
        try:
            self._begin_read_reservation(
                reservation,
                contention_quarantines=False,
            )
            self._ensure_open()
            with self._state_lock:
                self._raise_retained_quarantine_locked()
                reader_binding = self._reader_binding
                if type(reader_binding) is _JsonSerialReaderReleaseToken:
                    self._mark_quarantined_locked(
                        "serial reader release remained unsettled"
                    )
                    self._raise_retained_quarantine_locked()
                elif reader_binding is None:
                    released = False
                else:
                    if reader_binding is not threading.current_thread():
                        raise JsonSerialTransportAdmissionError(
                            "serial reader ownership can only be released "
                            "by its current reader"
                        )
                    if self._buffer or self._partial_deadline is not None:
                        raise JsonSerialTransportAdmissionError(
                            "serial reader ownership requires a complete "
                            "frame boundary"
                        )
                    if self._frame_handoff is not None:
                        raise JsonSerialTransportAdmissionError(
                            "serial reader ownership requires acknowledged "
                            "frame handoff"
                        )
                    mutation_started = True
                    self._reader_binding = release_token
            if mutation_started:
                released = self._settle_reader_release(release_token)
        except BaseException:
            with self._state_lock:
                if (
                    mutation_started
                    and self._reader_binding is release_token
                ):
                    self._mark_quarantined_locked(
                        "serial reader release did not return a disposition"
                    )
            raise
        return released

    def release_reader(self):
        """Release the current reader without consuming serial input."""
        reservation = _JsonSerialReadReservation()
        release_token = _JsonSerialReaderReleaseToken(
            threading.current_thread()
        )
        return self._run_read_reservation_operation(
            reservation,
            self._release_reader_reserved,
            release_token,
        )

    def _acknowledge_frame_reserved(self, reservation, frame):
        acknowledged = False
        self._begin_read_reservation(
            reservation,
            contention_quarantines=False,
        )
        self._ensure_open()
        with self._state_lock:
            self._raise_retained_quarantine_locked()
            handoff = self._frame_handoff
            if type(handoff) is not _JsonSerialFrameHandoff:
                raise JsonSerialTransportAdmissionError(
                    "serial frame handoff is not pending"
                )
            if type(frame) is not bytes or frame is not handoff.frame:
                raise JsonSerialTransportAdmissionError(
                    "serial frame acknowledgement does not own handoff"
                )
            self._last_acknowledged_frame = frame
            self._frame_handoff = None
            acknowledged = True
        return acknowledged

    def acknowledge_frame(self, frame):
        """Acknowledge durable caller ownership of one returned frame."""
        reservation = _JsonSerialReadReservation()
        return self._run_read_reservation_operation(
            reservation,
            self._acknowledge_frame_reserved,
            frame,
        )

    @property
    def pending_input_bytes(self):
        """Return bytes waiting in the owned serial driver's input queue."""
        with self._serial_io_lock:
            self._ensure_open()
            return self._sample_pending_input_bytes()

    def _mark_quarantined_locked(self, reason):
        if self._quarantine_reason is None:
            self._quarantine_reason = reason
        if type(self._reader_binding) is _JsonSerialReaderReleaseToken:
            self._reader_binding = None
        self._frame_handoff = None
        self._last_acknowledged_frame = None

    def _raise_quarantined(
        self,
        reason,
        cause=None,
        error_type=JsonSerialTransportQuarantinedError,
    ):
        with self._state_lock:
            self._mark_quarantined_locked(reason)
            retained_reason = self._quarantine_reason
        error = error_type(
            f"JSON serial transport quarantined: {retained_reason}"
        )
        if cause is not None:
            raise error from cause
        raise error

    def _raise_retained_quarantine_locked(self):
        if self._quarantine_reason is not None:
            raise JsonSerialTransportQuarantinedError(
                "JSON serial transport quarantined: "
                f"{self._quarantine_reason}"
            )

    def quarantine(self, reason):
        if (
            type(reason) is not str
            or not reason
            or len(reason) > 512
            or reason != reason.strip()
            or not reason.isascii()
            or any(
                ord(character) < 0x20 or ord(character) > 0x7E
                for character in reason
            )
        ):
            raise JsonSerialTransportAdmissionError(
                "transport quarantine reason is invalid"
            )
        with self._state_lock:
            self._mark_quarantined_locked(reason)

    def _sample_clock(self):
        with self._clock_lock:
            with self._state_lock:
                self._raise_retained_quarantine_locked()
            try:
                current = _clock_value(self._clock())
            except BaseException as exc:
                if isinstance(exc, Exception):
                    self._raise_quarantined("transport clock failed", exc)
                with self._state_lock:
                    self._mark_quarantined_locked(
                        "transport clock read was interrupted"
                    )
                raise
            with self._state_lock:
                self._raise_retained_quarantine_locked()
                if current < self._last_clock:
                    self._mark_quarantined_locked(
                        "transport clock moved backward"
                    )
                    self._raise_retained_quarantine_locked()
                self._last_clock = current
            return current

    def _ensure_open(self):
        with self._state_lock:
            self._raise_retained_quarantine_locked()
        try:
            open_state = self._serial_port.is_open
        except Exception as exc:
            self._raise_quarantined(
                "serial open state could not be read",
                exc,
            )
        if type(open_state) is not bool or not open_state:
            self._raise_quarantined("serial connection is closed")

    def _restore_timeout(self, attribute, value):
        try:
            setattr(self._serial_port, attribute, value)
        except BaseException as exc:
            with self._state_lock:
                self._mark_quarantined_locked(
                    f"serial {attribute} restoration failed"
                )
            raise exc

    def _drain_output(self, deadline):
        stagnant_delay = 0.0
        while True:
            self._ensure_open()
            current = self._sample_clock()
            if current >= deadline:
                self._raise_quarantined(
                    "outbound serial drain exceeded the request deadline"
                )
            try:
                output_pending = self._serial_port.out_waiting
            except BaseException as exc:
                if isinstance(exc, Exception):
                    self._raise_quarantined(
                        "serial output queue state could not be read",
                        exc,
                    )
                with self._state_lock:
                    self._mark_quarantined_locked(
                        "serial output queue state read was interrupted"
                    )
                raise
            if type(output_pending) is not int or output_pending < 0:
                self._raise_quarantined(
                    "serial output queue state is invalid"
                )
            if output_pending == 0:
                return
            delay = min(self._drain_poll_interval, deadline - current)
            try:
                self._sleeper(delay)
            except BaseException as exc:
                if isinstance(exc, Exception):
                    self._raise_quarantined(
                        "serial output drain wait failed",
                        exc,
                    )
                with self._state_lock:
                    self._mark_quarantined_locked(
                        "serial output drain wait was interrupted"
                    )
                raise
            advanced = self._sample_clock()
            if advanced > current:
                stagnant_delay = 0.0
            else:
                stagnant_delay += delay
            if stagnant_delay >= min(
                deadline - current,
                _JSON_SERIAL_MAXIMUM_CLOCK_STALL_SECONDS,
            ):
                self._raise_quarantined(
                    "transport clock did not advance before the output-drain "
                    "deadline"
                )

    def write_frame(self, frame, deadline):
        """Write one complete frame before an absolute monotonic deadline."""
        with self._state_lock:
            self._raise_retained_quarantine_locked()
        outbound = _validated_outbound_frame(frame)
        if type(deadline) not in (int, float):
            raise JsonSerialTransportAdmissionError(
                "serial write deadline must be a finite number"
            )
        try:
            normalized_deadline = float(deadline)
        except (OverflowError, ValueError) as exc:
            raise JsonSerialTransportAdmissionError(
                "serial write deadline must be a finite number"
            ) from exc
        if not math.isfinite(normalized_deadline):
            raise JsonSerialTransportAdmissionError(
                "serial write deadline must be a finite number"
            )

        current = self._sample_clock()
        transport_limit = current + self._write_timeout
        if not math.isfinite(transport_limit) or transport_limit <= current:
            raise JsonSerialTransportAdmissionError(
                "serial write timeout range is invalid"
            )
        transport_deadline = min(
            normalized_deadline,
            transport_limit,
        )
        remaining = transport_deadline - current
        if not math.isfinite(remaining):
            raise JsonSerialTransportAdmissionError(
                "serial write deadline range is invalid"
            )
        if remaining <= 0:
            raise JsonSerialTransportDeadlineError(
                "serial write deadline expired before write ownership"
            )
        acquired = self._write_lock.acquire(
            timeout=min(remaining, threading.TIMEOUT_MAX)
        )
        if not acquired:
            raise JsonSerialTransportDeadlineError(
                "serial write deadline expired while waiting for ownership"
            )

        write_called = False
        try:
            self._ensure_open()
            current = self._sample_clock()
            remaining = transport_deadline - current
            if not math.isfinite(remaining):
                raise JsonSerialTransportAdmissionError(
                    "serial write deadline range is invalid"
                )
            if remaining <= 0:
                raise JsonSerialTransportDeadlineError(
                    "serial write deadline expired before transmission"
                )

            serial_io_acquired = self._serial_io_lock.acquire(
                timeout=min(remaining, threading.TIMEOUT_MAX)
            )
            if not serial_io_acquired:
                raise JsonSerialTransportDeadlineError(
                    "serial write deadline expired while waiting for I/O "
                    "ownership"
                )

            original_timeout = None
            timeout_captured = False
            failure = None
            written = None
            try:
                current = self._sample_clock()
                remaining = transport_deadline - current
                if not math.isfinite(remaining):
                    raise JsonSerialTransportAdmissionError(
                        "serial write deadline range is invalid"
                    )
                if remaining <= 0:
                    raise JsonSerialTransportDeadlineError(
                        "serial write deadline expired before serial I/O"
                    )
                try:
                    original_timeout = self._serial_port.write_timeout
                    timeout_captured = True
                    self._serial_port.write_timeout = remaining
                    write_called = True
                    written = self._write(outbound)
                    if type(written) is not int or written != len(outbound):
                        raise OSError("outbound serial write was incomplete")
                except BaseException as exc:
                    failure = exc
                finally:
                    if timeout_captured:
                        try:
                            self._restore_timeout(
                                "write_timeout",
                                original_timeout,
                            )
                        except BaseException as restore_exc:
                            if failure is None:
                                failure = restore_exc
            finally:
                self._serial_io_lock.release()

            if failure is not None:
                reason = (
                    "outbound serial write failed after transmission admission"
                    if write_called
                    else "serial write-timeout setup failed"
                )
                if isinstance(
                    failure,
                    JsonSerialTransportQuarantinedError,
                ):
                    raise failure
                if isinstance(failure, Exception):
                    self._raise_quarantined(reason, failure)
                with self._state_lock:
                    self._mark_quarantined_locked(reason)
                raise failure
            self._drain_output(transport_deadline)
            completed_at = self._sample_clock()
            if completed_at >= transport_deadline:
                self._raise_quarantined(
                    "outbound serial write exceeded the transport deadline"
                )
            self._ensure_open()
            return written
        except BaseException:
            if write_called:
                with self._state_lock:
                    self._mark_quarantined_locked(
                        "outbound serial write did not return a disposition"
                    )
            raise
        finally:
            self._write_lock.release()

    def _new_partial_deadline(self, current):
        deadline = current + self._frame_timeout
        if not math.isfinite(deadline) or deadline <= current:
            self._raise_quarantined(
                "serial frame deadline is not representable"
            )
        return deadline

    def _extract_frame(self, current):
        newline_index = self._buffer.find(b"\n")
        if newline_index < 0:
            return None
        frame_length = newline_index + 1
        frame = bytes(self._buffer[:frame_length])
        try:
            _validate_inbound_frame(frame)
        except ValueError as exc:
            self._raise_quarantined(
                "inbound serial frame violates the JSON byte boundary",
                exc,
            )
        next_deadline = (
            self._new_partial_deadline(current)
            if len(self._buffer) > frame_length
            else None
        )
        with self._state_lock:
            if self._frame_handoff is not None:
                self._mark_quarantined_locked(
                    "inbound serial frame handoff ownership changed"
                )
                self._raise_retained_quarantine_locked()
            self._frame_handoff = _JsonSerialFrameHandoff(frame)
        del self._buffer[:frame_length]
        self._partial_deadline = next_deadline
        return frame

    def _sample_pending_input_bytes(self):
        try:
            input_pending = self._serial_port.in_waiting
        except BaseException as exc:
            if isinstance(exc, Exception):
                self._raise_quarantined(
                    "serial input queue state could not be read",
                    exc,
                )
            with self._state_lock:
                self._mark_quarantined_locked(
                    "serial input queue state read was interrupted"
                )
            raise
        if type(input_pending) is not int or input_pending < 0:
            self._raise_quarantined("serial input queue state is invalid")
        return input_pending

    def _pending_read_size(self, maximum_size):
        input_pending = self._sample_pending_input_bytes()
        return min(maximum_size, max(1, input_pending))

    def _defer_partial_deadline(self, delay):
        if self._partial_deadline is None:
            return
        deferred_deadline = self._partial_deadline + delay
        if (
            not math.isfinite(deferred_deadline)
            or deferred_deadline <= self._partial_deadline
        ):
            self._raise_quarantined(
                "serial frame deadline deferral is not representable"
            )
        self._partial_deadline = deferred_deadline

    def _read_chunk(self, read_size, read_timeout):
        serial_io_acquired = self._serial_io_lock.acquire(blocking=False)
        if not serial_io_acquired:
            serial_io_acquired = self._serial_io_lock.acquire(
                timeout=min(read_timeout, threading.TIMEOUT_MAX)
            )
            try:
                self._defer_partial_deadline(read_timeout)
            except BaseException:
                if serial_io_acquired:
                    self._serial_io_lock.release()
                raise
            if not serial_io_acquired:
                with self._state_lock:
                    self._raise_retained_quarantine_locked()
                raise JsonSerialTransportReadDeferredError(read_timeout)
        original_timeout = None
        timeout_captured = False
        failure = None
        chunk = None
        read_called = False
        try:
            self._ensure_open()
            read_size = self._pending_read_size(read_size)
            original_timeout = self._serial_port.timeout
            timeout_captured = True
            self._serial_port.timeout = read_timeout
            with self._state_lock:
                self._raise_retained_quarantine_locked()
            read_called = True
            chunk = self._read(read_size)
        except BaseException as exc:
            failure = exc
        finally:
            if timeout_captured:
                try:
                    self._restore_timeout("timeout", original_timeout)
                except BaseException as restore_exc:
                    if failure is None:
                        failure = restore_exc
            self._serial_io_lock.release()

        if failure is not None:
            reason = (
                "inbound serial read failed after read admission"
                if read_called
                else "serial read-timeout setup failed"
            )
            if isinstance(
                failure,
                JsonSerialTransportQuarantinedError,
            ):
                raise failure
            if isinstance(failure, Exception):
                self._raise_quarantined(reason, failure)
            with self._state_lock:
                self._mark_quarantined_locked(reason)
            raise failure
        if type(chunk) not in (bytes, bytearray):
            self._raise_quarantined(
                "serial read returned a non-bytes result"
            )
        normalized = bytes(chunk)
        if len(normalized) > read_size:
            self._raise_quarantined(
                "serial read exceeded the requested byte count"
            )
        return normalized

    def _poll_frame_reserved(self):
        self._ensure_open()
        current = self._sample_clock()
        complete = self._extract_frame(current)
        if complete is not None:
            return complete

        if (
            self._partial_deadline is not None
            and current >= self._partial_deadline
        ):
            self._raise_quarantined(
                "inbound serial frame assembly timed out",
                error_type=JsonSerialTransportFrameTimeoutError,
            )
        if len(self._buffer) >= JSON_PROTOCOL_MAXIMUM_FRAME_BYTES:
            self._raise_quarantined("inbound serial frame is oversized")

        read_timeout = self._poll_interval
        if self._partial_deadline is not None:
            read_timeout = min(
                read_timeout,
                self._partial_deadline - current,
            )
        read_size = min(
            self._read_chunk_bytes,
            JSON_PROTOCOL_MAXIMUM_FRAME_BYTES - len(self._buffer),
        )
        chunk = self._read_chunk(read_size, read_timeout)
        completed_at = self._sample_clock()
        self._ensure_open()
        if (
            self._partial_deadline is not None
            and completed_at >= self._partial_deadline
        ):
            self._raise_quarantined(
                "inbound serial frame assembly timed out",
                error_type=JsonSerialTransportFrameTimeoutError,
            )
        if not chunk:
            return None

        had_partial = bool(self._buffer)
        self._buffer.extend(chunk)
        if not had_partial:
            self._partial_deadline = self._new_partial_deadline(completed_at)
        complete = self._extract_frame(completed_at)
        if complete is not None:
            return complete
        if len(self._buffer) >= JSON_PROTOCOL_MAXIMUM_FRAME_BYTES:
            self._raise_quarantined("inbound serial frame is oversized")
        return None

    def _poll_frame_owned(self, reservation):
        frame = None
        try:
            self._begin_read_reservation(
                reservation,
                contention_quarantines=True,
            )
            with self._state_lock:
                self._last_acknowledged_frame = None
                if self._frame_handoff is not None:
                    self._mark_quarantined_locked(
                        "inbound serial frame handoff remained unacknowledged"
                    )
                    self._raise_retained_quarantine_locked()
            frame = self._poll_frame_reserved()
            return frame
        except JsonSerialTransportReadDeferredError:
            raise
        except BaseException:
            with self._state_lock:
                self._mark_quarantined_locked(
                    "inbound serial poll did not return a disposition"
                )
            raise

    def poll_frame(self):
        """Return at most one frame retained until exact acknowledgement."""
        reservation = _JsonSerialReadReservation()
        return self._run_read_reservation_operation(
            reservation,
            self._poll_frame_owned,
        )


__all__ = (
    "JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS",
    "JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS",
    "JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS",
    "JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES",
    "JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS",
    "JsonSerialTransport",
    "JsonSerialTransportAdmissionError",
    "JsonSerialTransportDeadlineError",
    "JsonSerialTransportError",
    "JsonSerialTransportFrameTimeoutError",
    "JsonSerialTransportQuarantinedError",
    "JsonSerialTransportReadDeferredError",
)
