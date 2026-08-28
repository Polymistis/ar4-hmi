"""Thread-safe correlation state for one bound JSON controller session."""

from collections.abc import Mapping
from contextlib import contextmanager
from dataclasses import dataclass, field
import math
import re
import threading
import time
import weakref
from typing import Callable, Optional, Union

from .catalog import (
    AUXILIARY_CONTROLLER,
    MAIN_CONTROLLER,
    commands_for_device,
)
from .messages import (
    JSON_PROTOCOL_MAXIMUM_FRAME_BYTES,
    JSON_PROTOCOL_MAXIMUM_IDENTIFIER,
    JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
    Event,
    JsonProtocolDecodeError,
    JsonProtocolError,
    ProtocolErrorFrame,
    Request,
    Response,
    Telemetry,
    decode_message,
    encode_message,
    freeze_json_object,
)


JSON_SESSION_MAXIMUM_PENDING_REQUESTS = 128
JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS = 128

_COMMAND_NAME_PATTERN = re.compile(r"[a-z][a-z0-9_]{0,63}\Z")
_MAXIMUM_QUARANTINE_REASON_LENGTH = 512
_SESSION_ACTIVE = "active"
_SESSION_CLOSED = "closed"
_SESSION_QUARANTINED = "quarantined"
_DEADLINE_HANDSHAKE_POLL_SECONDS = 0.01
_DEADLINE_CANCELLATION_DEFERRED = object()


class JsonSessionError(RuntimeError):
    """Base error for JSON session admission, state, or correlation failure."""


class JsonSessionAdmissionError(JsonSessionError):
    """A request cannot enter the bound session."""


class JsonSessionClosedError(JsonSessionError):
    """An operation requires a session that has not been closed."""


class JsonSessionQuarantinedError(JsonSessionError):
    """Session framing or state is untrusted and requires reconnection."""


class JsonSessionProtocolError(JsonSessionQuarantinedError):
    """An inbound frame violated correlation or sequencing."""


class JsonSessionTimeoutError(JsonSessionQuarantinedError):
    """An admitted request missed the absolute response deadline."""


class JsonSessionTransportError(JsonSessionQuarantinedError):
    """An outbound frame did not cross the complete write boundary."""


class JsonSessionClockError(JsonSessionQuarantinedError):
    """The monotonic deadline source became unavailable or moved backward."""


class JsonSessionDeadlineError(JsonSessionQuarantinedError):
    """The deadline owner violated scheduling or cancellation semantics."""


@dataclass(frozen=True)
class JsonCommandContract:
    """Explicit activation and schema callbacks for one canonical command."""

    name: str
    request_validator: Callable[[Mapping], None]
    response_validator: Callable[[Response], None]
    acceptance_required_for_terminal: bool = False
    deadline_suspended_after_acceptance: bool = False

    def __post_init__(self):
        if (
            type(self.name) is not str
            or _COMMAND_NAME_PATTERN.fullmatch(self.name) is None
        ):
            raise JsonSessionAdmissionError(
                "command contract name is invalid"
            )
        if not callable(self.request_validator):
            raise JsonSessionAdmissionError(
                "command request validator must be callable"
            )
        if not callable(self.response_validator):
            raise JsonSessionAdmissionError(
                "command response validator must be callable"
            )
        if type(self.acceptance_required_for_terminal) is not bool:
            raise JsonSessionAdmissionError(
                "command response sequencing policy must be boolean"
            )
        if type(self.deadline_suspended_after_acceptance) is not bool:
            raise JsonSessionAdmissionError(
                "command deadline-suspension policy must be boolean"
            )
        if (
            self.deadline_suspended_after_acceptance
            and not self.acceptance_required_for_terminal
        ):
            raise JsonSessionAdmissionError(
                "command deadline suspension requires accepted-response "
                "sequencing"
            )


@dataclass(eq=False)
class _TerminalAcknowledgement:
    _session_key: object = field(repr=False)
    committed: bool = False
    ticket_ref: Optional[weakref.ReferenceType] = field(
        default=None,
        repr=False,
    )


@dataclass(eq=False)
class _PublicReturnAcknowledgement:
    _session_key: object = field(repr=False)
    operation_token: object = field(repr=False)
    committed: bool = False


@dataclass(frozen=True, eq=False)
class JsonRequestTicket:
    """Opaque admission handle retained through terminal acknowledgement."""

    request_id: int
    command: str
    deadline: float
    _session_key: object = field(repr=False, compare=False)
    _terminal_acknowledgement: Optional[_TerminalAcknowledgement] = field(
        default=None,
        repr=False,
        compare=False,
    )
    _return_acknowledgement: Optional[_PublicReturnAcknowledgement] = field(
        default=None,
        repr=False,
        compare=False,
    )


@dataclass(frozen=True)
class JsonRequestSnapshot:
    ticket: JsonRequestTicket
    accepted: Optional[Response]
    terminal: Optional[Response]
    terminal_sequence: Optional[int] = None


@dataclass(frozen=True)
class JsonResponseDelivery:
    ticket: JsonRequestTicket
    response: Response
    _return_acknowledgement: Optional[_PublicReturnAcknowledgement] = field(
        default=None,
        repr=False,
        compare=False,
    )


@dataclass(frozen=True)
class JsonEventDelivery:
    event: Event
    sequence_contiguous: Optional[bool]
    _return_acknowledgement: Optional[_PublicReturnAcknowledgement] = field(
        default=None,
        repr=False,
        compare=False,
    )


@dataclass(frozen=True)
class JsonTelemetryDelivery:
    telemetry: Telemetry
    sequence_contiguous: Optional[bool]
    _return_acknowledgement: Optional[_PublicReturnAcknowledgement] = field(
        default=None,
        repr=False,
        compare=False,
    )


@dataclass(eq=False)
class _DeadlineRegistration:
    owner: object
    cancel: Callable[[], None]
    callback_failure_generation: int = 0
    cancelled: bool = False
    cancellation_token: Optional[object] = field(
        default=None,
        repr=False,
    )
    settlement_pending: Optional[object] = None

    @property
    def cancellation_in_progress(self):
        return self.cancellation_token is not None


@dataclass(frozen=True)
class _ActiveDeadlineCallback:
    registration: _DeadlineRegistration
    thread: threading.Thread = field(repr=False)


@dataclass(eq=False)
class _DeadlineRearmTransfer:
    source_registration: Optional[_DeadlineRegistration]


@dataclass
class _PendingRequest:
    ticket: JsonRequestTicket
    contract: JsonCommandContract
    deadline_registration: Optional[
        Union[_DeadlineRegistration, _DeadlineRearmTransfer]
    ]
    write_admitted: bool = False
    accepted: Optional[Response] = None
    terminal: Optional[Response] = None
    terminal_sequence: Optional[int] = None


class _OperationOwner:
    __slots__ = ()


@dataclass(eq=False)
class _OperationReservation:
    kind: str
    admitted: bool = False
    committed: bool = False
    receive_token: Optional[object] = None
    owner_context: Optional[_OperationOwner] = None
    public_result: Optional[object] = None
    public_acknowledgement: Optional[_PublicReturnAcknowledgement] = None


class _ReaderReleaseToken:
    __slots__ = ("owner",)

    def __init__(self, owner):
        self.owner = owner


@dataclass(frozen=True)
class _SessionDisposition:
    state: str
    quarantine_reason: Optional[str] = None


class _DeadlineRegistrationTransfer:
    def __init__(self, publication_handler, violation_handler):
        self._publication_handler = publication_handler
        self._violation_handler = violation_handler
        self._lock = threading.Lock()
        self._registration = None
        self._failure = None
        self._sealed = False

    @property
    def registration(self):
        with self._lock:
            return self._registration

    def publish(self, owner, cancel):
        registration = (
            _DeadlineRegistration(owner, cancel)
            if callable(cancel)
            else None
        )
        reason = None
        accepted_registration = None
        publication_exception = None
        accepted = False
        with self._lock:
            accepted_registration = self._registration
            if self._sealed:
                reason = "deadline scheduler published ownership after completion"
            elif self._failure is not None:
                reason = "deadline scheduler published ownership after a violation"
            elif self._registration is not None:
                reason = "deadline scheduler published ownership more than once"
            elif registration is None:
                reason = "deadline cancellation owner must be callable"
            elif not _valid_deadline_owner(owner):
                reason = "deadline scheduler did not publish a valid owner"
            else:
                try:
                    reason = self._publication_handler(registration)
                    if reason is None:
                        self._registration = registration
                        accepted = True
                except BaseException as exc:
                    publication_exception = exc
                    accepted_registration = self._registration
                    reason = (
                        "deadline scheduler owner publication was interrupted"
                    )
            if not accepted and self._failure is None:
                self._failure = reason
        if accepted:
            return
        # The compact call keeps a latched violation inside the exception table
        # before interruption can bypass session-owned cleanup.
        try: self._violation_handler(accepted_registration, registration, reason)
        except BaseException:
            self._violation_handler(
                accepted_registration,
                registration,
                reason,
            )
            raise
        if publication_exception is not None:
            raise publication_exception
        raise JsonSessionAdmissionError(reason)

    def seal(self):
        with self._lock:
            self._sealed = True
            return self._registration, self._failure

    def force_seal(self):
        with self._lock:
            self._sealed = True


class _DeadlineHandshake:
    def __init__(self):
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._scheduling = True
        self._resolution = None
        self._callback_invoked = False
        self._invoked_during_schedule = False

    def complete_schedule(self):
        with self._lock:
            self._scheduling = False
            return self._invoked_during_schedule

    def _publish_resolution_locked(self, resolution):
        self._resolution = resolution
        self._event.set()

    def callback_action(self):
        with self._lock:
            if self._callback_invoked:
                if self._resolution is None:
                    self._publish_resolution_locked(False)
                else:
                    self._event.set()
                return "duplicate"
            self._callback_invoked = True
            if self._scheduling:
                self._invoked_during_schedule = True
                self._publish_resolution_locked(False)
                return "early"
        while True:
            self._event.wait(_DEADLINE_HANDSHAKE_POLL_SECONDS)
            with self._lock:
                if self._resolution is not None:
                    return "drive" if self._resolution is True else "aborted"
                self._event.clear()

    def activate(self):
        with self._lock:
            if self._scheduling or self._resolution is not None:
                if self._resolution is not None:
                    self._event.set()
                return False
            self._publish_resolution_locked(True)
            return True

    def abort(self):
        with self._lock:
            if self._resolution is not None:
                self._event.set()
                return False
            self._publish_resolution_locked(False)
            return True


class _RequestIdentifierAllocator:
    def __init__(self, next_identifier=1):
        if (
            type(next_identifier) is not int
            or next_identifier < 1
            or next_identifier > JSON_PROTOCOL_MAXIMUM_IDENTIFIER
        ):
            raise JsonSessionAdmissionError(
                "next request identifier is outside the protocol range"
            )
        self._next_identifier = next_identifier

    def next_available(self, retained):
        candidate = self._next_identifier
        for _ in range(len(retained) + 1):
            if candidate not in retained:
                return candidate
            candidate = (
                1
                if candidate == JSON_PROTOCOL_MAXIMUM_IDENTIFIER
                else candidate + 1
            )
        raise JsonSessionAdmissionError(
            "no request identifier is available within the retained window"
        )

    def commit(self, identifier):
        self._next_identifier = (
            1
            if identifier == JSON_PROTOCOL_MAXIMUM_IDENTIFIER
            else identifier + 1
        )


def _clock_value(value, field_name):
    if type(value) not in (int, float):
        raise JsonSessionAdmissionError(f"{field_name} must be a finite number")
    try:
        normalized = float(value)
    except (OverflowError, ValueError) as exc:
        raise JsonSessionAdmissionError(
            f"{field_name} must be a finite number"
        ) from exc
    if not math.isfinite(normalized):
        raise JsonSessionAdmissionError(f"{field_name} must be a finite number")
    return normalized


def _normalize_inbound_frame(frame):
    if type(frame) is bytes:
        return frame
    if type(frame) is not bytearray:
        raise JsonProtocolDecodeError(
            "session frame must use exact bytes or bytearray"
        )
    try:
        with memoryview(frame) as view:
            if (
                not view.nbytes
                or view.nbytes > JSON_PROTOCOL_MAXIMUM_FRAME_BYTES
            ):
                raise JsonProtocolDecodeError(
                    "protocol frame exceeds the size limit"
                )
            return view.tobytes()
    except JsonProtocolDecodeError:
        raise
    except Exception as exc:
        raise JsonProtocolDecodeError(
            "protocol frame is inaccessible"
        ) from exc


def _positive_timeout(value):
    timeout = _clock_value(value, "request timeout")
    if timeout <= 0:
        raise JsonSessionAdmissionError("request timeout must be positive")
    return timeout


def _valid_deadline_owner(owner):
    return owner is not None and not callable(owner)


def _deadline_registrations_share_owner(left, right):
    if left is right:
        return True
    return (
        type(left) is _DeadlineRegistration
        and type(right) is _DeadlineRegistration
        and _valid_deadline_owner(left.owner)
        and _valid_deadline_owner(right.owner)
        and left.owner is right.owner
    )


def _bounded_limit(value, field_name, maximum):
    if type(value) is not int or value < 1 or value > maximum:
        raise JsonSessionAdmissionError(
            f"{field_name} must be an integer between 1 and {maximum}"
        )
    return value


def _normalized_reason(value):
    if (
        type(value) is not str
        or not value
        or value != value.strip()
        or len(value) > _MAXIMUM_QUARANTINE_REASON_LENGTH
        or not value.isascii()
        or any(ord(character) < 32 or ord(character) == 127 for character in value)
    ):
        raise JsonSessionAdmissionError(
            "session quarantine reason must be normalized printable ASCII"
        )
    return value


def _schedule_deadline_timer(
    deadline,
    callback,
    publish_owner,
    *,
    clock=time.monotonic,
    clock_resolution=0.0,
):
    normalized_deadline = _clock_value(
        deadline,
        "request deadline",
    )
    normalized_resolution = _clock_value(
        clock_resolution,
        "deadline scheduler clock resolution",
    )
    if normalized_resolution < 0:
        raise JsonSessionAdmissionError(
            "deadline scheduler clock resolution must be non-negative"
        )
    timer = threading.Timer(0.0, callback)
    timer.daemon = True
    publish_owner(timer, timer.cancel)
    current = _clock_value(clock(), "deadline scheduler clock")
    remaining = max(normalized_deadline - current, 0.0)
    timer.interval = min(
        remaining + normalized_resolution if remaining > 0 else 0.0,
        threading.TIMEOUT_MAX,
    )
    timer.start()


class CorrelatedJsonSession:
    """Own request correlation for a validated device command surface.

    One active caller thread supplies complete inbound frames. Sequential
    reader ownership can transfer only through an explicit settled-boundary
    release. Outbound writes are mutually serialized while inbound settlement
    remains full-duplex, and command contracts validate both directions before
    data crosses the session boundary. The default scheduler uses the host
    monotonic-clock resolution unless an explicit resolution is supplied. A
    custom clock with coarser observable ticks must provide matching resolution
    metadata.
    """

    def __init__(
        self,
        device,
        contracts,
        write_frame,
        *,
        clock=time.monotonic,
        clock_resolution=None,
        deadline_scheduler=_schedule_deadline_timer,
        maximum_pending_requests=JSON_SESSION_MAXIMUM_PENDING_REQUESTS,
        maximum_telemetry_streams=JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS,
    ):
        if type(device) is not str or device not in (
            MAIN_CONTROLLER,
            AUXILIARY_CONTROLLER,
        ):
            raise JsonSessionAdmissionError(
                "session device is invalid"
            )
        normalized_device = (
            MAIN_CONTROLLER
            if device == MAIN_CONTROLLER
            else AUXILIARY_CONTROLLER
        )
        try:
            device_commands = commands_for_device(normalized_device)
        except ValueError as exc:
            raise JsonSessionAdmissionError(
                "session device is invalid"
            ) from exc
        if type(contracts) is not tuple or not contracts:
            raise JsonSessionAdmissionError(
                "session command contracts must be a non-empty tuple"
            )
        if not callable(write_frame):
            raise JsonSessionAdmissionError(
                "session frame writer must be callable"
            )
        if not callable(clock):
            raise JsonSessionAdmissionError("session clock must be callable")
        if not callable(deadline_scheduler):
            raise JsonSessionAdmissionError(
                "session deadline scheduler must be callable"
            )
        if clock_resolution is None:
            normalized_clock_resolution = _clock_value(
                time.get_clock_info("monotonic").resolution,
                "session clock resolution",
            )
        else:
            normalized_clock_resolution = _clock_value(
                clock_resolution,
                "session clock resolution",
            )
            if normalized_clock_resolution < 0:
                raise JsonSessionAdmissionError(
                    "session clock resolution must be non-negative"
                )
        canonical_names = frozenset(command.name for command in device_commands)
        contract_by_name = {}
        for contract in contracts:
            if type(contract) is not JsonCommandContract:
                raise JsonSessionAdmissionError(
                    "session command contract entry is invalid"
                )
            if contract.name not in canonical_names:
                raise JsonSessionAdmissionError(
                    "session command contract is not canonical for the device"
                )
            if contract.name in contract_by_name:
                raise JsonSessionAdmissionError(
                    "session command contracts contain a duplicate command"
                )
            contract_by_name[contract.name] = contract

        normalized_pending_limit = _bounded_limit(
            maximum_pending_requests,
            "maximum pending requests",
            JSON_SESSION_MAXIMUM_PENDING_REQUESTS,
        )
        normalized_telemetry_limit = _bounded_limit(
            maximum_telemetry_streams,
            "maximum telemetry streams",
            JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS,
        )

        try:
            initial_clock = _clock_value(clock(), "session clock value")
        except JsonSessionAdmissionError:
            raise
        except Exception as exc:
            raise JsonSessionAdmissionError("session clock failed") from exc

        self._device = normalized_device
        self._contracts = contract_by_name
        self._write_frame = write_frame
        self._clock = clock
        self._clock_resolution = normalized_clock_resolution
        self._deadline_scheduler = (
            self._schedule_default_deadline
            if deadline_scheduler is _schedule_deadline_timer
            else deadline_scheduler
        )
        self._last_clock = initial_clock
        self._identifier_allocator = _RequestIdentifierAllocator()
        self._maximum_pending_requests = normalized_pending_limit
        self._maximum_telemetry_streams = normalized_telemetry_limit
        self._session_key = object()
        self._disposition = _SessionDisposition(_SESSION_ACTIVE)
        self._pending = {}
        self._issued_terminal_tickets = weakref.WeakKeyDictionary()
        self._reader_binding = None
        self._last_event_sequence = None
        self._telemetry_sequences = {}
        self._terminal_sequence = 0
        self._receive_token = None
        self._reserved_deadline_registrations = []
        self._retained_deadline_registrations = []
        self._active_deadline_callbacks = {}
        self._callback_context = threading.local()
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._clock_condition = threading.Condition()
        self._clock_owner = None
        self._deadline_cleanup_lock = threading.RLock()
        self._dependency_lock = threading.Lock()
        self._dependency_tokens = {}
        self._operation_tokens = {}

    def _schedule_default_deadline(
        self,
        deadline,
        callback,
        publish_owner,
    ):
        return _schedule_deadline_timer(
            deadline,
            callback,
            publish_owner,
            clock=self._sample_clock,
            clock_resolution=self._clock_resolution,
        )

    @property
    def device(self):
        return self._device

    @property
    def active_commands(self):
        return tuple(self._contracts)

    @property
    def reader_owner(self):
        with self._lock:
            if type(self._reader_binding) is _ReaderReleaseToken:
                return self._reader_binding.owner
            return self._reader_binding

    def validate_reader_release(self):
        """Validate a quiet inbound-frame ownership handoff.

        Sequence state remains session-owned. Only the current reader can
        release the thread binding, and no receive operation or unacknowledged
        receive return may cross the handoff boundary.
        """
        current_thread = threading.current_thread()
        with self._lock:
            self._ensure_active_locked()
            reader_binding = self._reader_binding
            if type(reader_binding) is _ReaderReleaseToken:
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    "inbound reader release remained unsettled",
                )
            if reader_binding is None:
                return False
            if reader_binding is not current_thread:
                raise JsonSessionAdmissionError(
                    "inbound frame ownership can only be released by its "
                    "current reader"
                )
            if self._receive_token is not None:
                raise JsonSessionAdmissionError(
                    "inbound frame ownership cannot be released during "
                    "frame processing"
                )
            with self._dependency_lock:
                receive_operations = tuple(
                    kind
                    for kind in self._operation_tokens.values()
                    if kind in ("receive", "receive_finalization")
                )
            if receive_operations:
                raise JsonSessionAdmissionError(
                    "inbound frame ownership cannot be released before "
                    "receive settlement"
                )
        return True

    def _settle_reader_release(self, token):
        with self._lock:
            if self._reader_binding is not token:
                raise JsonSessionAdmissionError(
                    "inbound reader release ownership changed before "
                    "settlement"
                )
            self._reader_binding = None
        return True

    def release_reader(self):
        """Release the current reader at a validated frame boundary."""
        if self.validate_reader_release() is not True:
            return False
        release_token = _ReaderReleaseToken(threading.current_thread())
        mutation_started = False
        released = False
        try:
            with self._lock:
                self._ensure_active_locked()
                reader_binding = self._reader_binding
                if type(reader_binding) is _ReaderReleaseToken:
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "inbound reader release remained unsettled",
                    )
                if reader_binding is not threading.current_thread():
                    raise JsonSessionAdmissionError(
                        "inbound frame ownership changed before release"
                    )
                if self._receive_token is not None:
                    raise JsonSessionAdmissionError(
                        "inbound frame processing began before reader release"
                    )
                with self._dependency_lock:
                    if any(
                        kind in ("receive", "receive_finalization")
                        for kind in self._operation_tokens.values()
                    ):
                        raise JsonSessionAdmissionError(
                            "receive settlement began before reader release"
                        )
                mutation_started = True
                self._reader_binding = release_token
            released = self._settle_reader_release(release_token)
        except BaseException:
            try:
                with self._lock:
                    if (
                        mutation_started
                        and self._reader_binding is release_token
                        and self._disposition.state == _SESSION_ACTIVE
                    ):
                        self._mark_quarantined_locked(
                            "inbound reader release did not return a "
                            "disposition"
                        )
            finally:
                with self._lock:
                    quarantined = (
                        self._disposition.state == _SESSION_QUARANTINED
                    )
                if quarantined:
                    self._drain_retained_deadline_cancellations()
            raise
        return released

    @property
    def quarantined(self):
        with self._lock:
            return self._disposition.state == _SESSION_QUARANTINED

    @property
    def closed(self):
        with self._lock:
            return self._disposition.state == _SESSION_CLOSED

    @property
    def quarantine_reason(self):
        with self._lock:
            return self._disposition.quarantine_reason

    @property
    def pending_count(self):
        with self._lock:
            return len(self._retained_request_identifiers_locked())

    @property
    def deadline_cleanup_count(self):
        """Return deadline owners retained for cleanup or retry."""
        with self._lock:
            return len(self._retained_deadline_registrations)

    def _ensure_active_locked(self):
        if self._disposition.state == _SESSION_QUARANTINED:
            raise JsonSessionQuarantinedError(
                "JSON session is quarantined: "
                f"{self._disposition.quarantine_reason}"
            )
        if self._disposition.state == _SESSION_CLOSED:
            raise JsonSessionClosedError("JSON session is closed")

    def _mark_quarantined_locked(self, reason):
        if self._disposition.state == _SESSION_CLOSED:
            raise JsonSessionClosedError("JSON session is closed")
        try:
            if self._disposition.state != _SESSION_QUARANTINED:
                self._disposition = _SessionDisposition(
                    _SESSION_QUARANTINED,
                    reason,
                )
        finally:
            if self._disposition.state == _SESSION_QUARANTINED:
                if type(self._reader_binding) is _ReaderReleaseToken:
                    self._reader_binding = None
                self._queue_pending_deadline_registrations_locked()
                self._queue_reserved_deadline_registrations_locked()
        return self._disposition.quarantine_reason

    def _queue_pending_deadline_registrations_locked(self):
        for pending in self._pending.values():
            if type(pending.deadline_registration) is _DeadlineRegistration:
                registration = pending.deadline_registration
                self._retain_deadline_registration_locked(registration)
                pending.deadline_registration = None
            elif type(
                pending.deadline_registration
            ) is _DeadlineRearmTransfer:
                source_registration = (
                    pending.deadline_registration.source_registration
                )
                if source_registration is not None:
                    self._retain_deadline_registration_locked(
                        source_registration
                    )
                pending.deadline_registration = None

    def _queue_reserved_deadline_registrations_locked(self):
        registrations = tuple(self._reserved_deadline_registrations)
        for registration in registrations:
            self._retain_deadline_registration_locked(registration)

    def _raise_quarantined_locked(self, error_type, reason, cause=None):
        retained_reason = self._mark_quarantined_locked(reason)
        error = error_type(f"JSON session quarantined: {retained_reason}")
        if cause is not None:
            raise error from cause
        raise error

    def _begin_dependency(self, token, kind):
        try:
            with self._dependency_lock:
                self._dependency_tokens[token] = kind
            local_tokens = getattr(
                self._callback_context,
                "dependency_tokens",
                (),
            )
            self._callback_context.dependency_tokens = local_tokens + (token,)
        except BaseException:
            try: self._end_dependency(token)
            finally: self._force_end_dependency(token)
            raise

    def _begin_active_dependency(self, token, kind):
        try:
            with self._lock:
                self._ensure_active_locked()
                self._begin_dependency(token, kind)
        except BaseException:
            try: self._end_dependency(token)
            finally: self._force_end_dependency(token)
            raise

    def _end_dependency(self, token):
        local_tokens = getattr(
            self._callback_context,
            "dependency_tokens",
            (),
        )
        try:
            self._callback_context.dependency_tokens = tuple(
                retained for retained in local_tokens if retained is not token
            )
        finally:
            with self._dependency_lock:
                self._dependency_tokens.pop(token, None)

    def _force_end_dependency(self, token):
        local_tokens = getattr(
            self._callback_context,
            "dependency_tokens",
            (),
        )
        retained_tokens = tuple(
            retained for retained in local_tokens if retained is not token
        )
        with self._dependency_lock:
            try: self._callback_context.dependency_tokens = retained_tokens
            finally: self._dependency_tokens.pop(token, None)

    @contextmanager
    def _dependency_reservation(self, kind, *, require_active=False):
        token = object()
        try:
            if require_active:
                self._begin_active_dependency(token, kind)
            else:
                self._begin_dependency(token, kind)
            yield
        finally:
            # Compact suites place cleanup calls inside the protected
            # bytecode range before a traced interruption can be delivered.
            try: self._end_dependency(token)
            finally: self._force_end_dependency(token)

    def _force_end_local_dependencies(self, retained_tokens=()):
        local_tokens = getattr(
            self._callback_context,
            "dependency_tokens",
            (),
        )
        with self._dependency_lock:
            retained_local_tokens = tuple(
                token
                for token in local_tokens
                if any(token is retained for retained in retained_tokens)
            )
            try:
                self._callback_context.dependency_tokens = (
                    retained_local_tokens
                )
            finally:
                for token in local_tokens:
                    if not any(
                        token is retained for retained in retained_tokens
                    ):
                        self._dependency_tokens.pop(token, None)

    def _dependency_kinds(self):
        with self._dependency_lock:
            return tuple(self._dependency_tokens.values())

    def _local_dependency_kinds(self):
        local_tokens = getattr(
            self._callback_context,
            "dependency_tokens",
            (),
        )
        with self._dependency_lock:
            return tuple(
                self._dependency_tokens[token]
                for token in local_tokens
                if token in self._dependency_tokens
            )

    def _local_dependency_tokens(self):
        local_tokens = getattr(
            self._callback_context,
            "dependency_tokens",
            (),
        )
        with self._dependency_lock:
            return tuple(
                token
                for token in local_tokens
                if token in self._dependency_tokens
            )

    def _local_operation_tokens(self):
        local_tokens = getattr(
            self._callback_context,
            "operation_tokens",
            (),
        )
        with self._dependency_lock:
            retained_tokens = tuple(
                token
                for token in local_tokens
                if token in self._operation_tokens
            )
        self._callback_context.operation_tokens = retained_tokens
        return retained_tokens

    def _local_operation_owner(self):
        owner = getattr(self._callback_context, "operation_owner", None)
        if owner is None:
            owner = _OperationOwner()
            self._callback_context.operation_owner = owner
        elif type(owner) is not _OperationOwner:
            raise JsonSessionAdmissionError(
                "local operation owner is invalid"
            )
        return owner

    def _begin_operation(self, token, *, allowed_global=()):
        try:
            self._require_explicit_public_return_acknowledgement()
            with self._lock:
                self._ensure_active_locked()
                with self._dependency_lock:
                    local_dependency_tokens = getattr(
                        self._callback_context,
                        "dependency_tokens",
                        (),
                    )
                    local_kinds = tuple(
                        self._dependency_tokens[dependency_token]
                        for dependency_token in local_dependency_tokens
                        if dependency_token in self._dependency_tokens
                    )
                    blocked_kinds = local_kinds or tuple(
                        dependency_kind
                        for dependency_kind in self._dependency_tokens.values()
                        if dependency_kind not in allowed_global
                    )
                    if not blocked_kinds:
                        operation_tokens = getattr(
                            self._callback_context,
                            "operation_tokens",
                            (),
                        )
                        token.owner_context = self._local_operation_owner()
                        self._operation_tokens[token] = token.kind
                        self._callback_context.operation_tokens = (
                            operation_tokens + (token,)
                        )
                        return
                if "clock" in local_kinds:
                    self._raise_quarantined_locked(
                        JsonSessionClockError,
                        "session clock became recursive",
                    )
            raise JsonSessionAdmissionError(
                "session operation cannot run from an active dependency "
                "callback"
            )
        except BaseException:
            self._force_end_operation(token)
            raise

    def _end_operation(self, token):
        with self._dependency_lock:
            operation_tokens = getattr(
                self._callback_context,
                "operation_tokens",
                (),
            )
            self._callback_context.operation_tokens = tuple(
                retained
                for retained in operation_tokens
                if retained is not token
            )
            self._operation_tokens.pop(token, None)

    def _force_end_operation(self, token):
        with self._dependency_lock:
            operation_tokens = getattr(
                self._callback_context,
                "operation_tokens",
                (),
            )
            try:
                self._callback_context.operation_tokens = tuple(
                    retained
                    for retained in operation_tokens
                    if retained is not token
                )
            finally:
                self._operation_tokens.pop(token, None)

    def _mark_operation_ambiguity_locked(self, token, reason):
        if (
            token.receive_token is not None
            and self._receive_token is token.receive_token
        ):
            self._receive_token = None
        if self._disposition.state == _SESSION_ACTIVE:
            self._mark_quarantined_locked(reason)
        quarantined = self._disposition.state == _SESSION_QUARANTINED
        self._force_end_operation(token)
        return quarantined

    def _quarantine_operation_ambiguity(self, token, reason):
        quarantined = False
        try:
            with self._lock:
                quarantined = self._mark_operation_ambiguity_locked(
                    token,
                    reason,
                )
        finally:
            if quarantined:
                self._drain_retained_deadline_cancellations()

    def _finish_committed_operation(self, token, result, reason):
        try:
            self._end_operation(token)
            return result
        except BaseException:
            self._quarantine_operation_ambiguity(token, reason)
            raise

    def _finish_public_return(self, token, result):
        acknowledgement = _PublicReturnAcknowledgement(
            self._session_key,
            token,
        )
        with self._dependency_lock:
            if type(result) not in (
                JsonRequestTicket,
                JsonResponseDelivery,
                JsonEventDelivery,
                JsonTelemetryDelivery,
            ) or result._return_acknowledgement is not None:
                raise JsonSessionAdmissionError(
                    "public return value cannot own acknowledgement"
                )
            token.public_result = result
            token.public_acknowledgement = acknowledgement
            object.__setattr__(
                result,
                "_return_acknowledgement",
                acknowledgement,
            )
        return result

    def _acknowledge_public_return_owner(
        self,
        result,
        acknowledgement,
    ):
        token = acknowledgement.operation_token
        with self._dependency_lock:
            if (
                type(token) is not _OperationReservation
                or token.public_result is not result
                or token.public_acknowledgement is not acknowledgement
                or result._return_acknowledgement is not acknowledgement
            ):
                raise JsonSessionAdmissionError(
                    "public return acknowledgement owner is invalid"
                )
            retained = token in self._operation_tokens
            if acknowledgement.committed:
                cleanup_required = retained
            else:
                if not retained:
                    raise JsonSessionAdmissionError(
                        "public return acknowledgement is not retained"
                    )
                acknowledgement.committed = True
                cleanup_required = True
        if not cleanup_required:
            return
        self._force_end_operation(token)

    def _require_explicit_public_return_acknowledgement(
        self,
        *,
        allow_unacknowledged=False,
    ):
        unacknowledged = False
        for token in self._local_operation_tokens():
            acknowledgement = token.public_acknowledgement
            if type(acknowledgement) is not _PublicReturnAcknowledgement:
                continue
            if acknowledgement.committed:
                self._force_end_operation(token)
            else:
                unacknowledged = True
        if unacknowledged and not allow_unacknowledged:
            raise JsonSessionAdmissionError(
                "explicit public return acknowledgement is required"
            )

    def _ensure_operation_context(
        self,
        *,
        allowed_global=(),
        allow_unacknowledged_public_return=False,
    ):
        self._require_explicit_public_return_acknowledgement(
            allow_unacknowledged=(
                allow_unacknowledged_public_return
            ),
        )
        local_kinds = self._local_dependency_kinds()
        if (
            not local_kinds
            and self._local_operation_tokens()
        ):
            return
        blocked_kinds = local_kinds or tuple(
            kind
            for kind in self._dependency_kinds()
            if kind not in allowed_global
        )
        if not blocked_kinds:
            return
        if "clock" in local_kinds:
            with self._lock:
                self._raise_quarantined_locked(
                    JsonSessionClockError,
                    "session clock became recursive",
                )
        raise JsonSessionAdmissionError(
            "session operation cannot run from an active dependency callback"
        )

    def _invoke_deadline_cancellation(self, registration):
        failure = None
        result = None
        with self._dependency_reservation("deadline_cancel"):
            try:
                result = registration.cancel()
            except BaseException as exc:
                failure = exc
        if failure is not None:
            return failure
        if result is not None:
            return JsonSessionDeadlineError(
                "deadline cancellation returned a value"
            )
        return None

    def _handle_deadline_publication_violation(
        self,
        accepted_registration,
        attempted_registration,
        reason,
    ):
        with self._lock:
            if accepted_registration is not None:
                self._retain_deadline_publication_locked(
                    accepted_registration
                )
            if attempted_registration is not None:
                self._retain_deadline_publication_locked(
                    attempted_registration
                )
            if self._disposition.state == _SESSION_ACTIVE:
                self._mark_quarantined_locked(reason)
            quarantined = (
                self._disposition.state == _SESSION_QUARANTINED
            )
        if quarantined:
            self._drain_retained_deadline_cancellations()
        else:
            for registration in (
                accepted_registration,
                attempted_registration,
            ):
                if registration is not None:
                    self._cancel_retained_deadline(registration)

    def _reserve_deadline_registration(self, registration):
        reason = None
        with self._lock:
            if self._disposition.state != _SESSION_ACTIVE:
                reason = (
                    "deadline scheduler published ownership after session termination"
                )
            else:
                canonical = self._deadline_registration_for_owner_locked(
                    registration.owner
                )
                if canonical is not None:
                    reason = (
                        "request deadline scheduler reused a live owner token"
                    )
                else:
                    self._reserved_deadline_registrations.append(registration)
        return reason

    def _release_deadline_reservation_locked(self, registration):
        self._reserved_deadline_registrations = [
            reserved
            for reserved in self._reserved_deadline_registrations
            if reserved is not registration
        ]

    def _retain_deadline_publication_locked(self, registration):
        canonical = self._deadline_registration_for_owner_locked(
            registration.owner
        )
        if canonical is not None and canonical is not registration:
            registration = canonical
        return self._retain_deadline_registration_locked(registration)

    def _complete_deadline_cancellation_locked(self, registration):
        registration.cancelled = True
        registration.cancellation_token = None
        pending = registration.settlement_pending
        if (
            type(pending) is _PendingRequest
            and pending.deadline_registration is registration
        ):
            pending.deadline_registration = None
        registration.settlement_pending = None
        self._release_deadline_reservation_locked(registration)
        self._release_deadline_registration_locked(registration)

    def _retain_deadline_registration_locked(self, registration):
        self._release_deadline_reservation_locked(registration)
        if registration.cancelled:
            self._complete_deadline_cancellation_locked(registration)
            return registration
        for retained in self._retained_deadline_registrations:
            if retained is registration:
                return retained
        self._retained_deadline_registrations.append(registration)
        return registration

    def _deadline_registration_for_owner_locked(
        self,
        owner,
        *,
        ignored_callback_token=None,
    ):
        if not _valid_deadline_owner(owner):
            return None
        for callback_token, active in self._active_deadline_callbacks.items():
            if callback_token is ignored_callback_token:
                continue
            registration = active.registration
            if registration.owner is owner:
                return registration
        for pending in self._pending.values():
            registration = pending.deadline_registration
            if (
                type(registration) is _DeadlineRegistration
                and registration.owner is owner
            ):
                return registration
            if (
                type(registration) is _DeadlineRearmTransfer
                and registration.source_registration is not None
                and registration.source_registration.owner is owner
            ):
                return registration.source_registration
        for registration in self._reserved_deadline_registrations:
            if registration.owner is owner:
                return registration
        for registration in self._retained_deadline_registrations:
            if registration.owner is owner:
                return registration
        return None

    def _release_deadline_registration_locked(self, registration):
        self._retained_deadline_registrations = [
            retained
            for retained in self._retained_deadline_registrations
            if retained is not registration
        ]

    def _begin_deadline_callback(self, registration, callback_token):
        if registration is None:
            return None, True
        with self._lock:
            try:
                self._active_deadline_callbacks[callback_token] = (
                    _ActiveDeadlineCallback(
                        registration,
                        threading.current_thread(),
                    )
                )
                if (
                    registration.cancelled
                    or registration.cancellation_in_progress
                ):
                    return registration, False
                canonical = self._deadline_registration_for_owner_locked(
                    registration.owner,
                    ignored_callback_token=callback_token,
                )
                if canonical is not None and canonical is not registration:
                    return registration, False
                return registration, True
            except BaseException:
                self._active_deadline_callbacks.pop(callback_token, None)
                raise

    def _end_deadline_callback(self, callback_token):
        if callback_token is None:
            return
        with self._lock:
            self._active_deadline_callbacks.pop(callback_token, None)

    def _force_end_deadline_callback(self, callback_token):
        with self._lock:
            self._active_deadline_callbacks.pop(callback_token, None)

    def _end_deadline_cancellation(
        self,
        registration,
        cancellation_token,
    ):
        with self._lock:
            if registration.cancellation_token is cancellation_token:
                registration.cancellation_token = None

    def _force_end_deadline_cancellation(
        self,
        registration,
        cancellation_token,
    ):
        with self._lock:
            if registration.cancellation_token is cancellation_token:
                registration.cancellation_token = None

    @contextmanager
    def _deadline_callback_owner(self, registration):
        canonical = None
        callback_token = object()
        admitted = False
        try:
            canonical, admitted = self._begin_deadline_callback(
                registration,
                callback_token,
            )
            yield canonical, admitted
        finally:
            try: self._end_deadline_callback(callback_token)
            finally: self._force_end_deadline_callback(callback_token)

    def _cancel_retained_deadline(self, registration):
        # Cancellation begins inside the protected lock boundary so an
        # interruption cannot strand cleanup ownership between acquire and try.
        with self._deadline_cleanup_lock: return self._cancel_retained_deadline_owned(registration)

    def _cancel_retained_deadline_owned(self, registration):
        cancellation_token = object()
        try:
            with self._lock:
                retained_registration = next(
                    (
                        owner
                        for owner in self._retained_deadline_registrations
                        if owner is registration
                    ),
                    None,
                )
                owner_active = any(
                    _deadline_registrations_share_owner(
                        active.registration,
                        registration,
                    )
                    for active in self._active_deadline_callbacks.values()
                )
                if retained_registration is not None:
                    registration = retained_registration
                if retained_registration is None:
                    return None
                if registration.cancelled:
                    self._complete_deadline_cancellation_locked(
                        registration
                    )
                    return None
                if owner_active or registration.cancellation_in_progress:
                    return _DEADLINE_CANCELLATION_DEFERRED
                callback_failure_generation = (
                    registration.callback_failure_generation
                )
                registration.cancellation_token = cancellation_token
            failure = self._invoke_deadline_cancellation(registration)
            if failure is None:
                with self._lock:
                    if (
                        registration.callback_failure_generation
                        == callback_failure_generation
                    ):
                        self._complete_deadline_cancellation_locked(
                            registration
                        )
                    else:
                        return _DEADLINE_CANCELLATION_DEFERRED
            return failure
        except BaseException as exc:
            return exc
        finally:
            try:
                self._end_deadline_cancellation(
                    registration,
                    cancellation_token,
                )
            finally:
                self._force_end_deadline_cancellation(
                    registration,
                    cancellation_token,
                )

    def _drain_retained_deadline_cancellations(self, excluded=()):
        with self._lock:
            if self._disposition.state != _SESSION_QUARANTINED:
                return
            active_dependencies = self._dependency_kinds()
            if any(
                kind in ("clock", "deadline_schedule", "deadline_cancel")
                for kind in active_dependencies
            ):
                return
            self._queue_pending_deadline_registrations_locked()
        with self._deadline_cleanup_lock:
            with self._lock:
                blocked_registrations = tuple(
                    active.registration
                    for active
                    in self._active_deadline_callbacks.values()
                ) + tuple(
                    registration
                    for registration in excluded
                    if registration is not None
                )
                registrations = tuple(
                    registration
                    for registration in self._retained_deadline_registrations
                    if not any(
                        _deadline_registrations_share_owner(
                            registration,
                            blocked_registration,
                        )
                        for blocked_registration in blocked_registrations
                    )
                )
            for registration in registrations:
                self._cancel_retained_deadline(registration)

    def _cancel_unadmitted_deadline(
        self,
        registration,
        *,
        require_active=True,
    ):
        # Splitting this line would reopen the gap before cleanup ownership is
        # protected by the exception table.
        try: return self._cancel_unadmitted_deadline_owned(registration, require_active)
        except BaseException:
            with self._lock:
                self._retain_deadline_registration_locked(registration)
                if self._disposition.state == _SESSION_ACTIVE:
                    self._mark_quarantined_locked(
                        "unadmitted request deadline cancellation was interrupted"
                    )
            raise

    def _cancel_unadmitted_deadline_owned(
        self,
        registration,
        require_active,
    ):
        with self._lock:
            self._retain_deadline_registration_locked(registration)
        failure = self._cancel_retained_deadline(registration)
        if failure is _DEADLINE_CANCELLATION_DEFERRED:
            with self._lock:
                if self._disposition.state == _SESSION_ACTIVE:
                    retained_reason = self._mark_quarantined_locked(
                        "unadmitted request deadline cancellation was deferred"
                    )
                elif self._disposition.state == _SESSION_QUARANTINED:
                    retained_reason = self._disposition.quarantine_reason
                else:
                    retained_reason = "session is closed"
            raise JsonSessionDeadlineError(
                "request deadline cancellation remains active: "
                f"{retained_reason}"
            )
        if failure is None:
            with self._lock:
                if require_active:
                    self._ensure_active_locked()
            return
        error = None
        with self._lock:
            if isinstance(failure, Exception):
                if self._disposition.state == _SESSION_ACTIVE:
                    retained_reason = self._mark_quarantined_locked(
                        "unadmitted request deadline cancellation failed"
                    )
                    message = f"JSON session quarantined: {retained_reason}"
                elif self._disposition.state == _SESSION_QUARANTINED:
                    message = (
                        "JSON session quarantined: "
                        f"{self._disposition.quarantine_reason}"
                    )
                else:
                    message = (
                        "request deadline cancellation failed after "
                        "session close"
                    )
                error = JsonSessionDeadlineError(message)
            elif self._disposition.state == _SESSION_ACTIVE:
                self._mark_quarantined_locked(
                    "unadmitted request deadline cancellation was interrupted"
                )
        if isinstance(failure, Exception):
            raise error from failure
        raise failure

    def _sample_clock(self):
        if "clock" in self._local_dependency_kinds():
            with self._lock:
                self._raise_quarantined_locked(
                    JsonSessionClockError,
                    "session clock became recursive",
                )
        current_thread = threading.current_thread()
        with self._lock:
            deadline_callback = any(
                active.thread is current_thread
                for active in self._active_deadline_callbacks.values()
            )
        if deadline_callback:
            return self._sample_clock_from_deadline_callback()
        with self._clock_reservation(blocking=True):
            return self._sample_clock_value()

    @contextmanager
    def _clock_reservation(self, *, blocking):
        token = object()
        try:
            with self._clock_condition:
                while self._clock_owner is not None and blocking:
                    self._clock_condition.wait()
                if self._clock_owner is None:
                    self._clock_owner = token
                admitted = self._clock_owner is token
            if admitted:
                self._begin_active_dependency(token, "clock")
            yield admitted
        finally:
            # Token identity makes cleanup safe even when nonblocking admission
            # failed. Repeated calls close either interruption boundary without
            # releasing another clock sampler.
            try:
                try: self._end_dependency(token)
                finally: self._force_end_dependency(token)
            finally:
                try: self._release_clock_owner(token)
                finally: self._release_clock_owner(token)

    def _release_clock_owner(self, token):
        with self._clock_condition:
            try:
                if self._clock_owner is token:
                    self._clock_owner = None
            finally:
                self._clock_condition.notify_all()

    def _sample_clock_from_deadline_callback(self):
        with self._clock_reservation(blocking=False) as admitted:
            if not admitted:
                with self._lock:
                    self._raise_quarantined_locked(
                        JsonSessionClockError,
                        "request deadline callback cannot wait for the session clock",
                    )
            return self._sample_clock_value()

    def _sample_clock_value(self):
        try:
            raw_value = self._clock()
        except BaseException as exc:
            with self._lock:
                if isinstance(exc, Exception):
                    self._raise_quarantined_locked(
                        JsonSessionClockError,
                        "session clock failed",
                        exc,
                    )
                self._mark_quarantined_locked(
                    "session clock was interrupted"
                )
            raise
        with self._lock:
            self._ensure_active_locked()
            try:
                current = _clock_value(
                    raw_value,
                    "session clock value",
                )
            except JsonSessionAdmissionError as exc:
                self._raise_quarantined_locked(
                    JsonSessionClockError,
                    "session clock failed",
                    exc,
                )
            if current < self._last_clock:
                self._raise_quarantined_locked(
                    JsonSessionClockError,
                    "session clock moved backward",
                )
            self._last_clock = current
            return current

    def _check_deadlines_locked(self, current):
        expired = tuple(
            pending
            for pending in self._pending.values()
            if (
                not self._request_deadline_settled_locked(pending)
                and pending.ticket.deadline <= current
            )
        )
        if not expired:
            return
        first = min(
            expired,
            key=lambda pending: (
                pending.ticket.deadline,
                pending.ticket.request_id,
            ),
        )
        self._raise_ticket_timeout_locked(first.ticket)

    def _raise_ticket_timeout_locked(self, ticket):
        self._raise_quarantined_locked(
            JsonSessionTimeoutError,
            "request deadline expired for "
            f"{ticket.command}#{ticket.request_id}",
        )

    def _check_ticket_deadline_locked(self, ticket, current):
        pending = self._pending.get(ticket.request_id)
        if (
            pending is not None
            and pending.ticket is ticket
            and self._request_deadline_suspended_locked(pending)
        ):
            return
        if ticket.deadline <= current:
            self._raise_ticket_timeout_locked(ticket)

    @staticmethod
    def _request_deadline_settled_locked(pending):
        return pending.terminal is not None or (
            CorrelatedJsonSession
                ._request_deadline_suspended_locked(pending)
        )

    @staticmethod
    def _request_deadline_suspended_locked(pending):
        return (
            pending.accepted is not None
            and pending.contract.deadline_suspended_after_acceptance
        )

    def _revalidate_deadlines(self):
        current = self._sample_clock()
        with self._lock:
            self._ensure_active_locked()
            self._check_deadlines_locked(current)
            return current

    def _retained_request_identifiers_locked(self):
        return set(self._pending)

    def _allocate_request_id_locked(self, retained):
        return self._identifier_allocator.next_available(retained)

    def _contract_for_submission(self, command):
        if type(command) is not str:
            raise JsonSessionAdmissionError("command name must be text")
        contract = self._contracts.get(command)
        if contract is None:
            raise JsonSessionAdmissionError(
                "command is not activated for the bound device session"
            )
        return contract

    def _schedule_request_deadline(
        self,
        ticket,
        transfer,
        handshake,
    ):
        invoked_before_completion = False
        scheduler_result = None
        scheduler_failure = None
        registration = None
        publication_failure = None

        def drive_deadline_callback(source_registration):
            action = handshake.callback_action()
            if action == "duplicate":
                self._reject_duplicate_deadline_callback(
                    source_registration,
                )
                return
            if action != "drive":
                return
            self._drive_deadline(ticket, source_registration)

        def owned_deadline_callback():
            registration = transfer.registration
            with self._deadline_callback_owner(registration) as (
                source_registration,
                admitted,
            ):
                if not admitted:
                    handshake.abort()
                    self._mark_deadline_callback_failure(
                        source_registration,
                        "request deadline callback ran during or after cancellation",
                    )
                    return
                return drive_deadline_callback(source_registration)

        def deadline_callback():
            # A single protected call keeps callback ownership recoverable even
            # when interruption lands before the owner context can be entered.
            try: return owned_deadline_callback()
            except BaseException:
                handshake.abort()
                self._mark_deadline_callback_failure(
                    transfer.registration,
                    "request deadline callback was interrupted",
                )
                raise

        with self._dependency_reservation("deadline_schedule"):
            try:
                try:
                    scheduler_result = self._deadline_scheduler(
                        ticket.deadline,
                        deadline_callback,
                        transfer.publish,
                    )
                except BaseException as exc:
                    handshake.abort()
                    scheduler_failure = exc
            finally:
                try:
                    try:
                        registration, publication_failure = transfer.seal()
                    finally:
                        transfer.force_seal()
                    invoked_before_completion = (
                        handshake.complete_schedule()
                    )
                except BaseException:
                    handshake.abort()
                    raise
        try:
            if (
                scheduler_failure is not None
                and not isinstance(scheduler_failure, Exception)
            ):
                with self._lock:
                    if self._disposition.state == _SESSION_ACTIVE:
                        self._mark_quarantined_locked(
                            "request deadline scheduling was interrupted"
                        )
                raise scheduler_failure
            if isinstance(scheduler_failure, JsonSessionQuarantinedError):
                with self._lock:
                    failure_quarantined = (
                        self._disposition.state == _SESSION_QUARANTINED
                    )
                if failure_quarantined:
                    raise scheduler_failure
            if publication_failure is not None:
                with self._lock:
                    self._raise_quarantined_locked(
                        JsonSessionDeadlineError,
                        publication_failure,
                        scheduler_failure,
                    )
            if scheduler_failure is not None:
                with self._lock:
                    self._ensure_active_locked()
                raise JsonSessionAdmissionError(
                    "request deadline scheduling failed",
                ) from scheduler_failure
            with self._lock:
                if scheduler_result is not None:
                    self._raise_quarantined_locked(
                        JsonSessionDeadlineError,
                        "request deadline scheduler returned a value",
                    )
                if (
                    registration is None
                    or not _valid_deadline_owner(registration.owner)
                ):
                    self._raise_quarantined_locked(
                        JsonSessionDeadlineError,
                        "request deadline scheduler did not publish a valid owner",
                    )
                if not any(
                    reserved is registration
                    for reserved in self._reserved_deadline_registrations
                ):
                    self._raise_quarantined_locked(
                        JsonSessionDeadlineError,
                        "request deadline ownership reservation changed",
                    )
                self._ensure_active_locked()
        except BaseException:
            handshake.abort()
            raise
        return (
            registration,
            invoked_before_completion,
        )

    def _detach_request_deadline_locked(self, pending):
        registration = pending.deadline_registration
        if type(registration) is _DeadlineRearmTransfer:
            return None
        if type(registration) is not _DeadlineRegistration:
            self._raise_quarantined_locked(
                JsonSessionDeadlineError,
                "request has no deadline cancellation owner",
            )
        self._retain_deadline_registration_locked(registration)
        registration.settlement_pending = pending
        return registration

    def _cancel_request_deadline(self, registration):
        failure = self._cancel_retained_deadline(registration)
        if failure is _DEADLINE_CANCELLATION_DEFERRED:
            return
        if failure is None:
            with self._lock:
                self._ensure_active_locked()
            return
        error = None
        with self._lock:
            if isinstance(failure, Exception):
                retained_reason = self._mark_quarantined_locked(
                    "request deadline cancellation failed"
                )
                error = JsonSessionDeadlineError(
                    f"JSON session quarantined: {retained_reason}"
                )
            else:
                self._mark_quarantined_locked(
                    "request deadline cancellation was interrupted"
                )
        if isinstance(failure, Exception):
            raise error from failure
        raise failure

    def _write_outbound_frame(self, frame, deadline, pending):
        with self._dependency_reservation(
            "frame_write",
            require_active=True,
        ):
            with self._lock:
                self._ensure_active_locked()
                if (
                    self._pending.get(pending.ticket.request_id) is not pending
                    or pending.write_admitted
                ):
                    self._raise_quarantined_locked(
                        JsonSessionTransportError,
                        "request write-admission ownership changed",
                    )
                pending.write_admitted = True
            return self._write_frame(frame, deadline)

    def _drive_deadline(self, ticket, source_registration):
        transfer = _DeadlineRegistrationTransfer(
            self._reserve_deadline_registration,
            self._handle_deadline_publication_violation
        )
        handshake = _DeadlineHandshake()
        rearm_transfer = _DeadlineRearmTransfer(source_registration)
        pending = None
        attempted_registration = None
        terminal_rearm = False
        try:
            with self._lock:
                if self._disposition.state != _SESSION_ACTIVE:
                    return
                pending = self._pending.get(ticket.request_id)
                if (
                    pending is None
                    or pending.ticket is not ticket
                    or self._request_deadline_settled_locked(pending)
                    or pending.deadline_registration is not source_registration
                ):
                    return
                pending.deadline_registration = rearm_transfer
            current = self._sample_clock()
            with self._lock:
                self._ensure_active_locked()
                self._check_deadlines_locked(current)
                retained = self._pending.get(ticket.request_id)
                if retained is not pending:
                    return
                if pending.deadline_registration is not rearm_transfer:
                    self._raise_quarantined_locked(
                        JsonSessionDeadlineError,
                        "request deadline rearm ownership changed",
                    )
                if self._request_deadline_settled_locked(pending):
                    pending.deadline_registration = None
                    return
            (
                registration,
                invoked_synchronously,
            ) = self._schedule_request_deadline(
                ticket,
                transfer,
                handshake,
            )
            if invoked_synchronously:
                handshake.abort()
                with self._lock:
                    if registration is not None:
                        self._retain_deadline_registration_locked(
                            registration
                        )
                    self._mark_quarantined_locked(
                        "request deadline rescheduling invoked synchronously"
                    )
                return
            post_schedule = self._sample_clock()
            with self._lock:
                self._ensure_active_locked()
                self._check_deadlines_locked(post_schedule)
                retained = self._pending.get(ticket.request_id)
                if retained is not pending:
                    handshake.abort()
                    attempted_registration = registration
                elif pending.deadline_registration is not rearm_transfer:
                    self._raise_quarantined_locked(
                        JsonSessionDeadlineError,
                        "request deadline rearm ownership changed",
                    )
                elif self._request_deadline_settled_locked(pending):
                    handshake.abort()
                    attempted_registration = registration
                    terminal_rearm = True
                else:
                    if not handshake.activate():
                        self._raise_quarantined_locked(
                            JsonSessionDeadlineError,
                            "request deadline activation was invalidated",
                        )
                    pending.deadline_registration = registration
                    self._release_deadline_reservation_locked(registration)
            if attempted_registration is not None:
                self._cancel_unadmitted_deadline(
                    attempted_registration,
                    require_active=True,
                )
            if terminal_rearm:
                with self._lock:
                    self._ensure_active_locked()
                    retained = self._pending.get(ticket.request_id)
                    if (
                        retained is not pending
                        or not self._request_deadline_settled_locked(pending)
                        or pending.deadline_registration is not rearm_transfer
                    ):
                        self._raise_quarantined_locked(
                            JsonSessionDeadlineError,
                            "request deadline rearm settlement changed",
                        )
                    pending.deadline_registration = None
        except BaseException:
            handshake.abort()
            failed_registration = transfer.registration
            with self._lock:
                if (
                    failed_registration is not None
                    and not failed_registration.cancelled
                ):
                    self._retain_deadline_registration_locked(
                        failed_registration
                    )
                if self._disposition.state == _SESSION_ACTIVE:
                    self._mark_quarantined_locked(
                        "request deadline rescheduling failed"
                    )
            return
        finally:
            with self._lock:
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations(
                    (source_registration,)
                    if source_registration is not None
                    else ()
                )

    def _mark_deadline_callback_failure(self, source_registration, reason):
        quarantined = False
        try:
            with self._lock:
                if source_registration is not None:
                    source_registration.cancelled = False
                    if self._disposition.state == _SESSION_ACTIVE:
                        source_registration.callback_failure_generation += 1
                    self._retain_deadline_registration_locked(
                        source_registration
                    )
                if self._disposition.state == _SESSION_ACTIVE:
                    self._mark_quarantined_locked(reason)
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
        finally:
            if quarantined:
                self._drain_retained_deadline_cancellations(
                    (source_registration,)
                    if source_registration is not None
                    else ()
                )

    def _reject_duplicate_deadline_callback(
        self,
        source_registration,
    ):
        self._mark_deadline_callback_failure(
            source_registration,
            "request deadline callback was invoked more than once",
        )

    def _validated_params(self, contract, params):
        candidate = {} if params is None else params
        with self._dependency_reservation("request_validation"):
            try:
                frozen = freeze_json_object(candidate)
            except JsonProtocolError as exc:
                raise JsonSessionAdmissionError(
                    f"{contract.name} request violates the JSON object boundary"
                ) from exc
            try:
                validation_result = contract.request_validator(frozen)
            except Exception as exc:
                raise JsonSessionAdmissionError(
                    f"{contract.name} request validation failed"
                ) from exc
        if validation_result is not None:
            raise JsonSessionAdmissionError(
                f"{contract.name} request validator returned a value"
            )
        return frozen

    def submit(
        self,
        command,
        params=None,
        *,
        timeout,
        write_admission=None,
        maximum_payload_bytes=JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
    ):
        """Submit one request after optional final write admission.

        The admission callback runs after validation and deadline scheduling,
        outside session state locks under a dependency reservation. The
        callback must remain bounded and return exact ``True``; an exception,
        another return value, or a recursive session operation prevents
        transport I/O. The clock and active state are revalidated after the
        callback before request ownership becomes pending.
        """
        if write_admission is not None and not callable(write_admission):
            raise JsonSessionAdmissionError(
                "request write admission must be callable"
            )
        normalized_payload_limit = _bounded_limit(
            maximum_payload_bytes,
            "request maximum payload bytes",
            JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
        )
        operation_token = _OperationReservation("submit")
        finalization_token = _OperationReservation("submit_finalization")
        inherited_dependency_tokens = self._local_dependency_tokens()
        ambiguity_reason = (
            "request submission did not return its committed ticket"
        )
        ticket = None
        try:
            self._begin_operation(
                operation_token,
                allowed_global=(
                    "request_validation",
                    "response_validation",
                ),
            )
            self._begin_operation(
                finalization_token,
                allowed_global=(
                    "request_validation",
                    "response_validation",
                ),
            )
            ticket = self._submit(
                command,
                params,
                timeout=timeout,
                operation_token=operation_token,
                write_admission=write_admission,
                maximum_payload_bytes=normalized_payload_limit,
            )
            self._finish_committed_operation(
                operation_token,
                ticket,
                ambiguity_reason,
            )
            return self._finish_public_return(
                finalization_token,
                ticket,
            )
        except BaseException:
            try:
                self._force_end_local_dependencies(
                    inherited_dependency_tokens
                )
            finally:
                try:
                    if operation_token.committed:
                        self._quarantine_operation_ambiguity(
                            operation_token,
                            ambiguity_reason,
                        )
                    elif operation_token.admitted:
                        self._quarantine_operation_ambiguity(
                            operation_token,
                            "request submission was interrupted after admission",
                        )
                    else:
                        self._force_end_operation(operation_token)
                finally:
                    self._force_end_operation(finalization_token)
            raise

    def _submit(
        self,
        command,
        params=None,
        *,
        timeout,
        operation_token,
        write_admission,
        maximum_payload_bytes,
    ):
        self._ensure_operation_context()
        with self._lock:
            self._ensure_active_locked()
        contract = self._contract_for_submission(command)
        validated_params = self._validated_params(contract, params)
        normalized_timeout = _positive_timeout(timeout)
        with self._write_lock:
            self._ensure_operation_context()
            pending = None
            deadline_registration = None
            deadline_transfer = _DeadlineRegistrationTransfer(
                self._reserve_deadline_registration,
                self._handle_deadline_publication_violation
            )
            deadline_handshake = None
            admission_failure = None
            attempted_registration = None
            try:
                try:
                    current = self._sample_clock()
                    with self._lock:
                        self._ensure_active_locked()
                        self._check_deadlines_locked(current)
                        retained_request_identifiers = (
                            self._retained_request_identifiers_locked()
                        )
                        if (
                            len(retained_request_identifiers)
                            >= self._maximum_pending_requests
                        ):
                            raise JsonSessionAdmissionError(
                                "session pending-request limit is reached"
                            )
                        deadline = current + normalized_timeout
                        if not math.isfinite(deadline) or deadline <= current:
                            raise JsonSessionAdmissionError(
                                "request deadline is not representably in the future"
                            )
                        request_id = self._allocate_request_id_locked(
                            retained_request_identifiers
                        )
                    try:
                        request = Request(
                            request_id,
                            command,
                            validated_params,
                        )
                        frame = encode_message(request)
                    except JsonProtocolError as exc:
                        raise JsonSessionAdmissionError(
                            f"{command} request violates the JSON envelope"
                        ) from exc
                    if len(frame) - 1 > maximum_payload_bytes:
                        raise JsonSessionAdmissionError(
                            f"{command} request exceeds the negotiated "
                            "JSON payload limit"
                        )
                    write_admission_time = self._sample_clock()
                    with self._lock:
                        self._ensure_active_locked()
                        self._check_deadlines_locked(write_admission_time)
                        if deadline <= write_admission_time:
                            raise JsonSessionAdmissionError(
                                "request deadline expired before write admission"
                            )
                        terminal_acknowledgement = _TerminalAcknowledgement(
                            self._session_key
                        )
                        ticket = JsonRequestTicket(
                            request_id,
                            command,
                            deadline,
                            self._session_key,
                            terminal_acknowledgement,
                        )
                        terminal_acknowledgement.ticket_ref = weakref.ref(ticket)
                        self._issued_terminal_tickets[ticket] = True
                    deadline_handshake = _DeadlineHandshake()
                    (
                        deadline_registration,
                        invoked_synchronously,
                    ) = self._schedule_request_deadline(
                        ticket,
                        deadline_transfer,
                        deadline_handshake,
                    )
                    if invoked_synchronously:
                        deadline_handshake.abort()
                        raise JsonSessionAdmissionError(
                            "request deadline scheduler invoked synchronously"
                        )
                    pre_admission_time = self._sample_clock()
                    with self._lock:
                        self._ensure_active_locked()
                        self._check_deadlines_locked(
                            pre_admission_time
                        )
                        if deadline <= pre_admission_time:
                            raise JsonSessionAdmissionError(
                                "request deadline expired before write admission"
                            )
                        pending = _PendingRequest(
                            ticket,
                            contract,
                            deadline_registration,
                        )
                    if write_admission is not None:
                        with self._dependency_reservation(
                            "write_admission",
                            require_active=True,
                        ):
                            admission_result = write_admission()
                            if admission_result is not True:
                                raise JsonSessionAdmissionError(
                                    "request write admission must return true"
                                )
                    final_write_admission_time = self._sample_clock()
                    with self._lock:
                        self._ensure_active_locked()
                        self._check_deadlines_locked(
                            final_write_admission_time
                        )
                        if deadline <= final_write_admission_time:
                            raise JsonSessionAdmissionError(
                                "request deadline expired before write admission"
                            )
                        if not deadline_handshake.activate():
                            self._raise_quarantined_locked(
                                JsonSessionDeadlineError,
                                "request deadline activation was invalidated",
                            )
                        operation_token.admitted = True
                        self._pending[request_id] = pending
                        self._release_deadline_reservation_locked(
                            deadline_registration
                        )
                        self._identifier_allocator.commit(request_id)
                except BaseException as exc:
                    admission_failure = exc
                    if deadline_handshake is not None:
                        deadline_handshake.abort()
                    if deadline_transfer.registration is not None:
                        deadline_registration = (
                            deadline_transfer.registration
                        )
                        with self._lock:
                            self._retain_deadline_registration_locked(
                                deadline_registration
                            )

                if admission_failure is not None:
                    with self._lock:
                        attached = (
                            pending is not None
                            and self._pending.get(pending.ticket.request_id)
                            is pending
                        )
                        require_active = (
                            self._disposition.state == _SESSION_ACTIVE
                        )
                    if deadline_registration is not None and not attached:
                        attempted_registration = deadline_registration
                        try:
                            self._cancel_unadmitted_deadline(
                                deadline_registration,
                                require_active=require_active,
                            )
                        except BaseException:
                            if isinstance(admission_failure, Exception):
                                raise
                    raise admission_failure

                try:
                    with self._lock:
                        self._ensure_active_locked()
                    written = self._write_outbound_frame(
                        frame,
                        deadline,
                        pending,
                    )
                except BaseException as exc:
                    with self._lock:
                        if isinstance(exc, Exception):
                            self._raise_quarantined_locked(
                                JsonSessionTransportError,
                                "outbound frame write failed",
                                exc,
                            )
                        self._mark_quarantined_locked(
                            "outbound frame write was interrupted"
                        )
                    raise

                completed_at = self._sample_clock()
                with self._lock:
                    if type(written) is not int or written != len(frame):
                        self._raise_quarantined_locked(
                            JsonSessionTransportError,
                            "outbound frame write was incomplete",
                        )
                    self._ensure_active_locked()
                    self._check_deadlines_locked(completed_at)
                    self._check_ticket_deadline_locked(ticket, completed_at)
                    operation_token.committed = True
                return ticket
            except BaseException:
                with self._lock:
                    retained_pending = (
                        pending is not None
                        and self._pending.get(pending.ticket.request_id)
                        is pending
                    )
                    if (
                        self._disposition.state == _SESSION_ACTIVE
                        and retained_pending
                    ):
                        self._mark_quarantined_locked(
                            "request submission was interrupted after admission"
                        )
                raise
            finally:
                with self._lock:
                    quarantined = (
                        self._disposition.state == _SESSION_QUARANTINED
                    )
                if quarantined:
                    self._drain_retained_deadline_cancellations(
                        (attempted_registration,)
                        if attempted_registration is not None
                        else ()
                    )

    def _bind_reader_locked(self):
        reader_thread = threading.current_thread()
        if self._reader_binding is None:
            self._reader_binding = reader_thread
        elif type(self._reader_binding) is _ReaderReleaseToken:
            self._raise_quarantined_locked(
                JsonSessionProtocolError,
                "inbound reader release remained unsettled",
            )
        elif self._reader_binding is not reader_thread:
            self._raise_quarantined_locked(
                JsonSessionProtocolError,
                "inbound frame ownership changed threads",
            )

    def _validate_response(self, pending, response):
        failure = None
        with self._dependency_reservation("response_validation"):
            try:
                validation_result = pending.contract.response_validator(
                    response
                )
            except BaseException as exc:
                failure = exc
        if failure is not None:
            with self._lock:
                if isinstance(failure, Exception):
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        f"{response.cmd} response validation failed",
                        failure,
                    )
                self._mark_quarantined_locked(
                    f"{response.cmd} response validation was interrupted"
                )
            raise failure
        self._revalidate_deadlines()
        with self._lock:
            self._ensure_active_locked()
            if validation_result is not None:
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    f"{response.cmd} response validator returned a value",
                )

    def _pending_for_response_locked(self, response):
        pending = self._pending.get(response.id)
        if pending is None:
            self._raise_quarantined_locked(
                JsonSessionProtocolError,
                "response identifier has no retained request",
            )
        if not pending.write_admitted:
            self._raise_quarantined_locked(
                JsonSessionProtocolError,
                "response arrived before request write admission",
            )
        if pending.ticket.command != response.cmd:
            self._raise_quarantined_locked(
                JsonSessionProtocolError,
                "response command does not match the retained request",
            )
        if pending.terminal is not None:
            self._raise_quarantined_locked(
                JsonSessionProtocolError,
                "request received a response after terminal settlement",
            )
        if response.status == "accepted" and pending.accepted is not None:
            self._raise_quarantined_locked(
                JsonSessionProtocolError,
                "request received duplicate accepted responses",
            )
        if pending.contract.acceptance_required_for_terminal:
            if response.status == "rejected" and pending.accepted is not None:
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    "request was rejected after acceptance",
                )
            if (
                response.status not in ("accepted", "rejected")
                and pending.accepted is None
            ):
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    "request received a terminal response before acceptance",
                )
        return pending

    def _receive_response(self, response):
        with self._lock:
            pending = self._pending_for_response_locked(response)
        self._validate_response(pending, response)
        delivery = JsonResponseDelivery(pending.ticket, response)
        self._revalidate_deadlines()
        with self._lock:
            self._ensure_active_locked()
            retained = self._pending_for_response_locked(response)
            if retained is not pending:
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    "response request ownership changed during validation",
                )
            if response.status == "accepted":
                pending.accepted = response
                registration = (
                    self._detach_request_deadline_locked(pending)
                    if pending.contract.deadline_suspended_after_acceptance
                    else None
                )
            else:
                self._terminal_sequence += 1
                pending.terminal = response
                pending.terminal_sequence = self._terminal_sequence
                registration = (
                    None
                    if (
                        pending.accepted is not None
                        and pending.contract
                            .deadline_suspended_after_acceptance
                    )
                    else self._detach_request_deadline_locked(pending)
                )
        return delivery, registration

    def _receive_event(self, event):
        with self._lock:
            self._ensure_active_locked()
            previous = self._last_event_sequence
            if previous is None:
                contiguous = None
            else:
                expected = (
                    0
                    if previous == JSON_PROTOCOL_MAXIMUM_IDENTIFIER
                    else previous + 1
                )
                if event.seq != expected:
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "controller event sequence is not contiguous",
                    )
                contiguous = True
        delivery = JsonEventDelivery(event, contiguous)
        self._revalidate_deadlines()
        with self._lock:
            self._ensure_active_locked()
            if self._last_event_sequence != previous:
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    "event sequence ownership changed during delivery",
                )
            self._last_event_sequence = event.seq
        return delivery

    def _receive_telemetry(self, telemetry):
        with self._lock:
            self._ensure_active_locked()
            stream_present = telemetry.stream in self._telemetry_sequences
            previous = self._telemetry_sequences.get(telemetry.stream)
            if not stream_present:
                if (
                    len(self._telemetry_sequences)
                    >= self._maximum_telemetry_streams
                ):
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "telemetry stream baseline limit is reached",
                    )
                contiguous = None
            else:
                expected = (
                    0
                    if previous == JSON_PROTOCOL_MAXIMUM_IDENTIFIER
                    else previous + 1
                )
                contiguous = telemetry.seq == expected
        delivery = JsonTelemetryDelivery(telemetry, contiguous)
        self._revalidate_deadlines()
        with self._lock:
            self._ensure_active_locked()
            retained_present = (
                telemetry.stream in self._telemetry_sequences
            )
            retained_previous = self._telemetry_sequences.get(
                telemetry.stream
            )
            if (
                retained_present != stream_present
                or retained_previous != previous
            ):
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    "telemetry sequence ownership changed during delivery",
                )
            self._telemetry_sequences[telemetry.stream] = telemetry.seq
        return delivery

    def receive(self, frame):
        if "frame_write" in self._local_dependency_kinds():
            with self._lock:
                self._raise_quarantined_locked(
                    JsonSessionProtocolError,
                    "session frame writer drove recursive inbound processing",
                )
        operation_token = _OperationReservation("receive")
        finalization_token = _OperationReservation("receive_finalization")
        inherited_dependency_tokens = self._local_dependency_tokens()
        ambiguity_reason = (
            "inbound processing did not return its committed delivery"
        )
        delivery = None
        try:
            self._begin_operation(
                operation_token,
                allowed_global=(
                    "frame_write",
                    "request_validation",
                    "deadline_schedule",
                ),
            )
            self._begin_operation(
                finalization_token,
                allowed_global=(
                    "frame_write",
                    "request_validation",
                    "deadline_schedule",
                ),
            )
            delivery = self._receive(frame, operation_token)
            self._finish_committed_operation(
                operation_token,
                delivery,
                ambiguity_reason,
            )
            return self._finish_public_return(
                finalization_token,
                delivery,
            )
        except BaseException:
            try:
                self._force_end_local_dependencies(
                    inherited_dependency_tokens
                )
            finally:
                try:
                    if operation_token.committed:
                        self._quarantine_operation_ambiguity(
                            operation_token,
                            ambiguity_reason,
                        )
                    elif operation_token.admitted:
                        self._quarantine_operation_ambiguity(
                            operation_token,
                            "inbound frame processing did not return a delivery",
                        )
                    else:
                        self._force_end_operation(operation_token)
                finally:
                    self._force_end_operation(finalization_token)
            raise

    def _receive(self, frame, operation_token):
        receive_token = object()
        inbound_owned = False
        registration = None
        attempted_registration = None
        try:
            self._ensure_operation_context(
                allowed_global=(
                    "frame_write",
                    "request_validation",
                    "deadline_schedule",
                )
            )
            with self._lock:
                self._ensure_active_locked()
                if self._receive_token is not None:
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "inbound frame processing became recursive",
                    )
                operation_token.admitted = True
                operation_token.receive_token = receive_token
                self._receive_token = receive_token
                inbound_owned = True
                self._bind_reader_locked()
            current = self._sample_clock()
            with self._lock:
                self._ensure_active_locked()
                self._check_deadlines_locked(current)

            try:
                inert_frame = _normalize_inbound_frame(frame)
                message = decode_message(inert_frame)
            except JsonProtocolError as exc:
                with self._lock:
                    self._ensure_active_locked()
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "inbound frame violates the JSON envelope",
                        exc,
                    )
            except BaseException as exc:
                with self._lock:
                    if isinstance(exc, Exception):
                        self._ensure_active_locked()
                        self._raise_quarantined_locked(
                            JsonSessionProtocolError,
                            "inbound frame decoder failed",
                            exc,
                        )
                    self._mark_quarantined_locked(
                        "inbound frame decoding was interrupted"
                    )
                raise

            self._revalidate_deadlines()

            if type(message) is Response:
                delivery, registration = self._receive_response(message)
            elif type(message) is Event:
                delivery = self._receive_event(message)
            elif type(message) is Telemetry:
                delivery = self._receive_telemetry(message)
            elif type(message) is ProtocolErrorFrame:
                with self._lock:
                    self._ensure_active_locked()
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "controller returned an uncorrelated protocol error",
                    )
            elif type(message) is Request:
                with self._lock:
                    self._ensure_active_locked()
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "controller returned an inbound request envelope",
                    )
            else:
                with self._lock:
                    self._ensure_active_locked()
                    self._raise_quarantined_locked(
                        JsonSessionProtocolError,
                        "controller returned an unsupported envelope",
                    )

            if registration is not None:
                attempted_registration = registration
                self._cancel_request_deadline(registration)
            delivered_at = self._revalidate_deadlines()
            with self._lock:
                self._ensure_active_locked()
                if type(message) is Response:
                    self._check_ticket_deadline_locked(
                        delivery.ticket,
                        delivered_at,
                    )
                operation_token.committed = True
            return delivery
        except BaseException:
            with self._lock:
                if (
                    inbound_owned
                    and self._disposition.state == _SESSION_ACTIVE
                ):
                    self._mark_quarantined_locked(
                        "inbound frame processing did not return a delivery"
                    )
            raise
        finally:
            with self._lock:
                if self._receive_token is receive_token:
                    self._receive_token = None
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations(
                    (attempted_registration,)
                    if attempted_registration is not None
                    else ()
                )

    def _pending_for_ticket_locked(self, ticket):
        if type(ticket) is not JsonRequestTicket:
            raise JsonSessionAdmissionError("request ticket is invalid")
        pending = self._pending.get(ticket.request_id)
        if (
            ticket._session_key is not self._session_key
            or pending is None
            or pending.ticket is not ticket
        ):
            raise JsonSessionAdmissionError(
                "request ticket does not belong to this retained session"
            )
        return pending

    def acknowledge_public_return(self, result):
        """Release a finalization owner after durable result handoff."""
        self._ensure_operation_context(
            allow_unacknowledged_public_return=True,
        )
        if type(result) not in (
            JsonRequestTicket,
            JsonResponseDelivery,
            JsonEventDelivery,
            JsonTelemetryDelivery,
        ):
            raise JsonSessionAdmissionError(
                "public return acknowledgement value is invalid"
            )
        acknowledgement = result._return_acknowledgement
        if (
            type(acknowledgement) is not _PublicReturnAcknowledgement
            or acknowledgement._session_key is not self._session_key
        ):
            raise JsonSessionAdmissionError(
                "public return value does not belong to this session"
            )
        self._acknowledge_public_return_owner(
            result,
            acknowledgement,
        )

    def recover_public_returns(self):
        """Claim recoverable results without releasing finalization owners."""
        self._ensure_operation_context(
            allow_unacknowledged_public_return=True,
        )
        committed_tokens = []
        results = []
        current_owner = self._local_operation_owner()
        with self._dependency_lock:
            for token in tuple(self._operation_tokens):
                acknowledgement = token.public_acknowledgement
                if acknowledgement is None:
                    continue
                result = token.public_result
                owner_context = token.owner_context
                if (
                    type(acknowledgement)
                    is not _PublicReturnAcknowledgement
                    or acknowledgement._session_key is not self._session_key
                    or acknowledgement.operation_token is not token
                    or type(owner_context) is not _OperationOwner
                    or type(result) not in (
                        JsonRequestTicket,
                        JsonResponseDelivery,
                        JsonEventDelivery,
                        JsonTelemetryDelivery,
                    )
                    or result._return_acknowledgement is not acknowledgement
                ):
                    raise JsonSessionAdmissionError(
                        "retained public return owner is invalid"
                    )
                if acknowledgement.committed:
                    committed_tokens.append(token)
                elif owner_context is current_owner:
                    results.append(result)
        for token in committed_tokens:
            self._force_end_operation(token)
        return tuple(results)

    def abandon_public_returns(self, reason):
        """Quarantine after explicitly abandoning retained result handoffs."""
        normalized_reason = _normalized_reason(reason)
        self._ensure_operation_context(
            allow_unacknowledged_public_return=True,
        )
        quarantine_reason = (
            "public return ownership was abandoned: " + normalized_reason
        )
        abandoned_tokens = []
        committed_tokens = []
        try:
            with self._lock:
                if self._disposition.state == _SESSION_CLOSED:
                    raise JsonSessionClosedError("JSON session is closed")
                with self._dependency_lock:
                    for token in tuple(self._operation_tokens):
                        acknowledgement = token.public_acknowledgement
                        if acknowledgement is None:
                            continue
                        result = token.public_result
                        if (
                            type(acknowledgement)
                            is not _PublicReturnAcknowledgement
                            or acknowledgement._session_key
                            is not self._session_key
                            or acknowledgement.operation_token is not token
                            or type(token.owner_context)
                            is not _OperationOwner
                            or type(result) not in (
                                JsonRequestTicket,
                                JsonResponseDelivery,
                                JsonEventDelivery,
                                JsonTelemetryDelivery,
                            )
                            or result._return_acknowledgement
                            is not acknowledgement
                        ):
                            raise JsonSessionAdmissionError(
                                "retained public return owner is invalid"
                            )
                        if acknowledgement.committed:
                            committed_tokens.append(token)
                        else:
                            abandoned_tokens.append(token)
                    if not abandoned_tokens:
                        for token in committed_tokens:
                            self._operation_tokens.pop(token, None)
                        raise JsonSessionAdmissionError(
                            "session has no public returns to abandon"
                        )
                    if self._disposition.state == _SESSION_ACTIVE:
                        self._mark_quarantined_locked(quarantine_reason)
                    for token in abandoned_tokens + committed_tokens:
                        self._operation_tokens.pop(token, None)
        finally:
            with self._lock:
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations()

    def snapshot(self, ticket):
        try:
            self._ensure_operation_context()
            with self._lock:
                self._ensure_active_locked()
            current = self._sample_clock()
            with self._lock:
                self._ensure_active_locked()
                if self._receive_token is not None:
                    raise JsonSessionAdmissionError(
                        "request snapshot cannot overlap inbound processing"
                    )
                self._check_deadlines_locked(current)
                pending = self._pending_for_ticket_locked(ticket)
                return JsonRequestSnapshot(
                    pending.ticket,
                    pending.accepted,
                    pending.terminal,
                    pending.terminal_sequence,
                )
        finally:
            with self._lock:
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations()

    def _retry_terminal_deadline_cleanup(self, ticket):
        with self._lock:
            self._ensure_active_locked()
            pending = self._pending_for_ticket_locked(ticket)
            registration = pending.deadline_registration
            if (
                pending.terminal is None
                or type(registration) is not _DeadlineRegistration
            ):
                return
            self._retain_deadline_registration_locked(registration)
            registration.settlement_pending = pending
        self._cancel_request_deadline(registration)

    def take_terminal(self, ticket):
        try:
            self._ensure_operation_context()
            with self._lock:
                self._ensure_active_locked()
            self._retry_terminal_deadline_cleanup(ticket)
            current = self._sample_clock()
            with self._lock:
                self._ensure_active_locked()
                if self._receive_token is not None:
                    raise JsonSessionAdmissionError(
                        "terminal read cannot overlap inbound processing"
                    )
                self._check_deadlines_locked(current)
                pending = self._pending_for_ticket_locked(ticket)
                if pending.terminal is None:
                    raise JsonSessionAdmissionError(
                        "request has no terminal response to hand off"
                    )
                if pending.deadline_registration is not None:
                    raise JsonSessionAdmissionError(
                        "terminal read awaits deadline ownership resolution"
                    )
                return pending.terminal
        finally:
            with self._lock:
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations()

    def acknowledge_terminal(self, ticket):
        self._ensure_operation_context()
        with self._lock:
            acknowledgement = (
                self._terminal_acknowledgement_for_ticket_locked(ticket)
            )
            pending = self._pending.get(ticket.request_id)
            if acknowledgement.committed:
                if pending is None:
                    return
                if pending.ticket is not ticket:
                    if (
                        pending.ticket._terminal_acknowledgement
                        is acknowledgement
                    ):
                        raise JsonSessionAdmissionError(
                            "terminal acknowledgement belongs to the retained ticket"
                        )
                    return
                if self._receive_token is not None:
                    raise JsonSessionAdmissionError(
                        "terminal acknowledgement cannot overlap inbound processing"
                    )
                if (
                    pending.terminal is None
                    or pending.deadline_registration is not None
                ):
                    raise JsonSessionAdmissionError(
                        "committed terminal acknowledgement is not releasable"
                    )
                del self._pending[ticket.request_id]
                return
            if self._disposition.state == _SESSION_CLOSED:
                raise JsonSessionClosedError("JSON session is closed")
            if self._receive_token is not None:
                raise JsonSessionAdmissionError(
                    "terminal acknowledgement cannot overlap inbound processing"
                )
            pending = self._pending_for_ticket_locked(ticket)
            if pending.terminal is None:
                raise JsonSessionAdmissionError(
                    "request has no terminal response to acknowledge"
                )
            if pending.deadline_registration is not None:
                raise JsonSessionAdmissionError(
                    "terminal acknowledgement awaits deadline ownership resolution"
                )
            acknowledgement.committed = True
            del self._pending[ticket.request_id]

    def terminal_acknowledgement_complete(self, ticket):
        """Report whether terminal ownership is durably released."""
        with self._lock:
            acknowledgement = (
                self._terminal_acknowledgement_for_ticket_locked(ticket)
            )
            pending = self._pending.get(ticket.request_id)
            return acknowledgement.committed and (
                pending is None or pending.ticket is not ticket
            )

    def _terminal_acknowledgement_for_ticket_locked(self, ticket):
        if (
            type(ticket) is not JsonRequestTicket
            or ticket._session_key is not self._session_key
            or type(ticket._terminal_acknowledgement)
            is not _TerminalAcknowledgement
            or ticket._terminal_acknowledgement._session_key
            is not self._session_key
            or type(ticket._terminal_acknowledgement.ticket_ref)
            is not weakref.ReferenceType
            or ticket._terminal_acknowledgement.ticket_ref() is not ticket
            or ticket not in self._issued_terminal_tickets
        ):
            raise JsonSessionAdmissionError(
                "request ticket does not belong to this retained session"
            )
        return ticket._terminal_acknowledgement

    def expire(self):
        try:
            self._ensure_operation_context()
            with self._lock:
                self._ensure_active_locked()
            current = self._sample_clock()
            with self._lock:
                self._ensure_active_locked()
                if self._receive_token is not None:
                    raise JsonSessionAdmissionError(
                        "manual expiry cannot overlap inbound processing"
                    )
                self._check_deadlines_locked(current)
        finally:
            with self._lock:
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations()

    def retry_deadline_cleanup(self):
        """Retry retained idempotent deadline cancellation owners."""
        self._ensure_operation_context()
        with self._lock:
            retryable_closed = (
                self._disposition.state == _SESSION_CLOSED
                and bool(self._retained_deadline_registrations)
            )
            if (
                self._disposition.state != _SESSION_QUARANTINED
                and not retryable_closed
            ):
                raise JsonSessionAdmissionError(
                    "deadline cleanup retry requires retained terminal owners"
                )
            if self._receive_token is not None:
                raise JsonSessionAdmissionError(
                    "deadline cleanup cannot overlap inbound processing"
                )
            self._queue_pending_deadline_registrations_locked()
        with self._deadline_cleanup_lock:
            with self._lock:
                registrations = tuple(
                    self._retained_deadline_registrations
                )
            for registration in registrations:
                failure = self._cancel_retained_deadline(registration)
                if failure is _DEADLINE_CANCELLATION_DEFERRED:
                    continue
                if (
                    failure is not None
                    and not isinstance(failure, Exception)
                ):
                    raise failure
        with self._lock:
            return len(self._retained_deadline_registrations)

    def quarantine(self, reason):
        normalized = _normalized_reason(reason)
        try:
            self._ensure_operation_context()
            with self._lock:
                self._ensure_active_locked()
                self._raise_quarantined_locked(
                    JsonSessionQuarantinedError,
                    normalized,
                )
        finally:
            with self._lock:
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations()

    def close(self):
        try:
            self._ensure_operation_context()
            with self._lock:
                self._ensure_active_locked()
                with self._dependency_lock:
                    if self._dependency_tokens or self._operation_tokens:
                        raise JsonSessionAdmissionError(
                            "session close cannot overlap an active operation"
                        )
                if any(
                    type(pending.deadline_registration)
                    is _DeadlineRearmTransfer
                    for pending in self._pending.values()
                ):
                    raise JsonSessionAdmissionError(
                        "session close awaits deadline rearm resolution"
                    )
                if self._receive_token is not None:
                    raise JsonSessionAdmissionError(
                        "session close cannot overlap inbound processing"
                    )
                if self._pending:
                    self._raise_quarantined_locked(
                        JsonSessionQuarantinedError,
                        "session closed with retained requests",
                    )
                if self._reserved_deadline_registrations:
                    self._raise_quarantined_locked(
                        JsonSessionQuarantinedError,
                        "session closed with retained deadline reservation",
                    )
                if self._retained_deadline_registrations:
                    self._raise_quarantined_locked(
                        JsonSessionQuarantinedError,
                        "session closed with retained deadline cleanup",
                    )
                self._disposition = _SessionDisposition(_SESSION_CLOSED)
        finally:
            with self._lock:
                quarantined = (
                    self._disposition.state == _SESSION_QUARANTINED
                )
            if quarantined:
                self._drain_retained_deadline_cancellations()
