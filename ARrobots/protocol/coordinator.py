"""Lifecycle boundary for one correlated JSON serial session."""

from collections import deque
import math
import threading
import time

from .messages import JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES
from .session import (
    JSON_SESSION_MAXIMUM_PENDING_REQUESTS,
    JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS,
    CorrelatedJsonSession,
    JsonEventDelivery,
    JsonRequestTicket,
    JsonResponseDelivery,
    JsonSessionAdmissionError,
    JsonSessionClosedError,
    JsonSessionError,
    JsonSessionQuarantinedError,
    JsonTelemetryDelivery,
)
from .transport import (
    JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS,
    JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS,
    JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS,
    JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES,
    JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS,
    JsonSerialTransport,
)


JSON_COORDINATOR_MAXIMUM_DELIVERIES = 128
JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS = 6.0

_SESSION_QUARANTINE_BRIDGE_REASON = (
    "bound JSON serial transport became quarantined"
)
_TRANSPORT_QUARANTINE_BRIDGE_REASON = (
    "bound correlated JSON session became quarantined"
)
_READER_RELEASE_FAILURE_REASON = (
    "JSON reader release was interrupted after ownership mutation"
)
_FRAME_HANDOFF_FAILURE_REASON = (
    "JSON inbound frame handoff did not reach durable coordination"
)


class JsonSerialSessionCoordinatorError(RuntimeError):
    """Base error for JSON session and serial lifecycle coordination."""


class JsonSerialSessionCoordinatorAdmissionError(
    JsonSerialSessionCoordinatorError
):
    """A coordinator argument or operation cannot be admitted."""


class JsonSerialSessionCoordinatorStateError(
    JsonSerialSessionCoordinatorError
):
    """Coordinator lifecycle, capacity, or trust rejects an operation."""


class JsonSerialSessionCoordinatorCloseError(
    JsonSerialSessionCoordinatorError
):
    """Coordinator close completed or stopped with a reported failure."""


class _CoordinatorOperationToken:
    __slots__ = (
        "frame_handoff_obligation",
        "reader_release_mutation_started",
        "reader_release_settled",
    )

    def __init__(self):
        self.frame_handoff_obligation = False
        self.reader_release_mutation_started = False
        self.reader_release_settled = False


def _bounded_delivery_limit(value):
    if (
        type(value) is not int
        or value < 1
        or value > JSON_COORDINATOR_MAXIMUM_DELIVERIES
    ):
        raise JsonSerialSessionCoordinatorAdmissionError(
            "coordinator delivery capacity is invalid"
        )
    return value


def _positive_timeout(value, field_name):
    if type(value) not in (int, float):
        raise JsonSerialSessionCoordinatorAdmissionError(
            f"{field_name} must be a positive finite number"
        )
    try:
        normalized = float(value)
    except (OverflowError, ValueError) as exc:
        raise JsonSerialSessionCoordinatorAdmissionError(
            f"{field_name} must be a positive finite number"
        ) from exc
    if not math.isfinite(normalized) or normalized <= 0:
        raise JsonSerialSessionCoordinatorAdmissionError(
            f"{field_name} must be a positive finite number"
        )
    return normalized


def _bounded_exception_detail(exc):
    try:
        detail = str(exc)
    except BaseException:
        detail = "message unavailable"
    return " ".join(detail.split())[:256]


class SerialWriteCancellationBoundary:
    """Order cancellation against the final serial write-admission check."""

    def __init__(self, context):
        if (not isinstance(context, str) or not context
                or context != context.strip() or "\r" in context or "\n" in context):
            raise TypeError("serial cancellation context must be normalized text")
        self._context = context
        self._event = threading.Event()
        self._lock = threading.Lock()

    def is_set(self):
        cancelled = self._event.is_set()
        if not isinstance(cancelled, bool):
            raise RuntimeError(f"{self._context} cancellation state must be boolean")
        return cancelled

    def cancel(self):
        with self._lock:
            self._event.set()
        return True

    def write_reservation(self):
        return self._lock

    def acquire(self):
        return self._lock.acquire()

    def release(self):
        return self._lock.release()


def close_unowned_serial_port(serial_port):
    """Return any error from closing an unowned serial handle."""
    try:
        serial_port.close()
        if getattr(serial_port, "is_open", None) is not False:
            raise RuntimeError("serial connection did not verify closed")
    except BaseException as error:
        return error
    return None


class JsonSerialSessionCoordinator:
    """Take lifecycle ownership of one already-open serial handle.

    Polling remains caller-owned. Each received delivery enters a bounded
    coordinator queue before the correlation core releases public-return
    ownership. Either layer's quarantine blocks coordinator operations
    immediately. Transport-to-session mirroring occurs when session admission
    permits. A temporarily inadmissible mirror remains visible in
    ``quarantine_reason`` and is retried at later lifecycle boundaries. The
    ``close`` operation blocks new operations, waits only for bounded active
    operations, and verifies serial closure. The session and transport share
    the supplied monotonic clock; custom clocks with coarser ticks must supply
    matching resolution metadata. Close waits use the process monotonic clock
    because condition waits use wall time.
    """

    def __init__(
        self,
        serial_port,
        device,
        contracts,
        *,
        clock=time.monotonic,
        clock_resolution=None,
        deadline_scheduler=None,
        sleeper=time.sleep,
        poll_interval=JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS,
        frame_timeout=JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS,
        write_timeout=JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS,
        read_chunk_bytes=JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES,
        drain_poll_interval=(
            JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS
        ),
        maximum_pending_requests=JSON_SESSION_MAXIMUM_PENDING_REQUESTS,
        maximum_telemetry_streams=JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS,
        delivery_capacity=JSON_COORDINATOR_MAXIMUM_DELIVERIES,
    ):
        try:
            close_serial = getattr(serial_port, "close")
        except Exception as exc:
            raise JsonSerialSessionCoordinatorAdmissionError(
                "serial connection does not expose close ownership"
            ) from exc
        if not callable(close_serial):
            raise JsonSerialSessionCoordinatorAdmissionError(
                "serial connection does not expose close ownership"
            )
        normalized_delivery_capacity = _bounded_delivery_limit(
            delivery_capacity
        )

        transport = JsonSerialTransport(
            serial_port,
            clock=clock,
            sleeper=sleeper,
            poll_interval=poll_interval,
            frame_timeout=frame_timeout,
            write_timeout=write_timeout,
            read_chunk_bytes=read_chunk_bytes,
            drain_poll_interval=drain_poll_interval,
        )
        session_arguments = {
            "clock": clock,
            "clock_resolution": clock_resolution,
            "maximum_pending_requests": maximum_pending_requests,
            "maximum_telemetry_streams": maximum_telemetry_streams,
        }
        if deadline_scheduler is not None:
            session_arguments["deadline_scheduler"] = deadline_scheduler
        session = CorrelatedJsonSession(
            device,
            contracts,
            transport.write_frame,
            **session_arguments,
        )
        self._serial_port = serial_port
        self._close_serial = close_serial
        self._transport = transport
        self._session = session
        self._delivery_capacity = normalized_delivery_capacity
        self._deliveries = deque()
        self._tickets = {}
        self._fault_reason = None
        self._session_quarantine_bridge_failure = None
        self._closing = False
        self._closed = False
        self._terminal_close_error = None
        self._terminal_close_cause = None
        self._session_shutdown_complete = False
        self._serial_shutdown_complete = False
        self._retained_serial_close_failure = None
        self._write_admission_operation = None
        self._reader_release_operation = None
        self._reader_release_cleanup_pending = False
        self._frame_handoff_cleanup_pending = False
        self._active_operations = set()
        self._submit_operations = set()
        self._state_condition = threading.Condition(threading.RLock())
        self._close_lock = threading.Lock()

    @property
    def device(self):
        return self._session.device

    @property
    def quarantined(self):
        with self._state_condition:
            faulted = self._fault_reason is not None
        return (
            faulted
            or self._session.quarantined
            or self._transport.quarantined
        )

    @property
    def quarantine_reason(self):
        with self._state_condition:
            if self._fault_reason is not None:
                if self._session_quarantine_bridge_failure is not None:
                    return (
                        f"{self._fault_reason}; "
                        f"{self._session_quarantine_bridge_failure}"
                    )
                return self._fault_reason
        if self._session.quarantined:
            return self._session.quarantine_reason
        return self._transport.quarantine_reason

    @property
    def closed(self):
        with self._state_condition:
            return self._closed

    @property
    def closing(self):
        """Return whether close owns admission or awaits a close retry."""
        with self._state_condition:
            return self._closing

    @property
    def delivery_count(self):
        with self._state_condition:
            return len(self._deliveries)

    @property
    def pending_tickets(self):
        with self._state_condition:
            return tuple(self._tickets.values())

    @property
    def deadline_cleanup_count(self):
        """Return retained deadline-cancellation owners awaiting retry."""
        return self._session.deadline_cleanup_count

    @property
    def has_unread_input(self):
        """Report whether the JSON framer or serial driver retains input."""
        return (
            self._transport.buffered_input_bytes != 0
            or self._transport.pending_input_bytes != 0
        )

    @property
    def reader_owner(self):
        with self._state_condition:
            self._require_reader_release_idle_locked(
                "reader ownership inspection"
            )
            session_owner = self._session.reader_owner
            transport_owner = self._transport.reader_owner
            if session_owner is not None and transport_owner is None:
                raise JsonSerialSessionCoordinatorStateError(
                    "JSON session reader remains bound without transport "
                    "ownership"
                )
            if (
                session_owner is not None
                and transport_owner is not None
                and session_owner is not transport_owner
            ):
                raise JsonSerialSessionCoordinatorStateError(
                    "JSON session and transport reader ownership differ"
                )
            return transport_owner

    def _record_fault(self, reason):
        with self._state_condition:
            if self._fault_reason is None:
                self._fault_reason = reason

    def _require_write_admission_idle_locked(self, operation):
        owner = self._write_admission_operation
        if owner is None:
            return
        if (
            type(owner) is not _CoordinatorOperationToken
            or owner not in self._active_operations
            or owner not in self._submit_operations
        ):
            raise JsonSerialSessionCoordinatorStateError(
                "coordinator write-admission reservation is invalid"
            )
        raise JsonSerialSessionCoordinatorStateError(
            f"coordinator {operation} rejected during write admission"
        )

    def _require_reader_release_idle_locked(self, operation):
        owner = self._reader_release_operation
        if owner is None:
            return
        if (
            type(owner) is not _CoordinatorOperationToken
            or owner not in self._active_operations
        ):
            raise JsonSerialSessionCoordinatorStateError(
                "coordinator reader-release reservation is invalid"
            )
        raise JsonSerialSessionCoordinatorStateError(
            f"coordinator {operation} rejected during reader release"
        )

    def _require_frame_handoff_idle_locked(self, operation):
        frame_owner = next(
            (
                token
                for token in self._active_operations
                if type(token) is _CoordinatorOperationToken
                and token.frame_handoff_obligation
            ),
            None,
        )
        if frame_owner is None or not self._transport.frame_handoff_pending:
            return
        raise JsonSerialSessionCoordinatorStateError(
            f"coordinator {operation} rejected during inbound frame handoff"
        )

    def _settle_reader_release_failure(self):
        with self._state_condition:
            cleanup_pending = self._reader_release_cleanup_pending
            operation = self._reader_release_operation
            release_in_progress = (
                type(operation) is _CoordinatorOperationToken
                and operation in self._active_operations
                and operation.reader_release_mutation_started
                and not operation.reader_release_settled
                and self._fault_reason is None
            )
            if cleanup_pending and self._fault_reason is None:
                if release_in_progress:
                    return True
                self._fault_reason = _READER_RELEASE_FAILURE_REASON
        if not cleanup_pending:
            return True

        self._transport.quarantine(_READER_RELEASE_FAILURE_REASON)
        if not self._transport.quarantined:
            with self._state_condition:
                self._session_quarantine_bridge_failure = (
                    "reader-release transport quarantine remains pending"
                )
            return False

        if not (self._session.quarantined or self._session.closed):
            try:
                self._session.quarantine(_READER_RELEASE_FAILURE_REASON)
            except (
                JsonSessionClosedError,
                JsonSessionQuarantinedError,
            ):
                pass
            except JsonSessionError as exc:
                detail = _bounded_exception_detail(exc)
                with self._state_condition:
                    self._session_quarantine_bridge_failure = (
                        "session quarantine mirror remains pending after "
                        f"{type(exc).__name__}"
                    )
                    if detail:
                        self._session_quarantine_bridge_failure += (
                            f": {detail}"
                        )
                return False

        if not (self._session.quarantined or self._session.closed):
            with self._state_condition:
                self._session_quarantine_bridge_failure = (
                    "reader-release session quarantine remains pending"
                )
            return False

        with self._state_condition:
            token = self._reader_release_operation
            if (
                type(token) is _CoordinatorOperationToken
                and token.reader_release_mutation_started
            ):
                token.reader_release_settled = True
            self._session_quarantine_bridge_failure = None
            self._reader_release_cleanup_pending = False
        return True

    def _settle_frame_handoff_failure(self):
        with self._state_condition:
            cleanup_pending = self._frame_handoff_cleanup_pending
        if not cleanup_pending:
            return True

        self._transport.quarantine(_FRAME_HANDOFF_FAILURE_REASON)
        if not self._transport.quarantined:
            with self._state_condition:
                self._session_quarantine_bridge_failure = (
                    "frame-handoff transport quarantine remains pending"
                )
            return False

        if not (self._session.quarantined or self._session.closed):
            try:
                recovered = self._session.recover_public_returns()
                if type(recovered) is not tuple:
                    raise JsonSessionAdmissionError(
                        "frame-handoff public-return recovery was invalid"
                    )
                if recovered:
                    self._session.abandon_public_returns(
                        _FRAME_HANDOFF_FAILURE_REASON
                    )
                else:
                    self._session.quarantine(_FRAME_HANDOFF_FAILURE_REASON)
            except (
                JsonSessionClosedError,
                JsonSessionQuarantinedError,
            ):
                pass
            except JsonSessionError as exc:
                detail = _bounded_exception_detail(exc)
                with self._state_condition:
                    self._session_quarantine_bridge_failure = (
                        "session quarantine mirror remains pending after "
                        f"{type(exc).__name__}"
                    )
                    if detail:
                        self._session_quarantine_bridge_failure += (
                            f": {detail}"
                        )
                return False

        if not (self._session.quarantined or self._session.closed):
            with self._state_condition:
                self._session_quarantine_bridge_failure = (
                    "frame-handoff session quarantine remains pending"
                )
            return False

        with self._state_condition:
            self._session_quarantine_bridge_failure = None
            self._frame_handoff_cleanup_pending = False
        return True

    def _coordinate_quarantine(self):
        if not self._settle_reader_release_failure():
            return
        if not self._settle_frame_handoff_failure():
            return
        session_quarantined = self._session.quarantined
        transport_quarantined = self._transport.quarantined
        if not session_quarantined and not transport_quarantined:
            return

        if self._fault_reason is None:
            if session_quarantined:
                self._record_fault(self._session.quarantine_reason)
            else:
                self._record_fault(self._transport.quarantine_reason)

        if transport_quarantined and not (
            session_quarantined or self._session.closed
        ):
            try:
                self._session.quarantine(
                    _SESSION_QUARANTINE_BRIDGE_REASON
                )
            except (
                JsonSessionClosedError,
                JsonSessionQuarantinedError,
            ):
                with self._state_condition:
                    self._session_quarantine_bridge_failure = None
            except JsonSessionError as exc:
                detail = _bounded_exception_detail(exc)
                with self._state_condition:
                    self._session_quarantine_bridge_failure = (
                        "session quarantine mirror remains pending after "
                        f"{type(exc).__name__}"
                    )
                    if detail:
                        self._session_quarantine_bridge_failure += (
                            f": {detail}"
                        )
        elif session_quarantined or self._session.closed:
            with self._state_condition:
                self._session_quarantine_bridge_failure = None
        if session_quarantined and not transport_quarantined:
            self._transport.quarantine(
                _TRANSPORT_QUARANTINE_BRIDGE_REASON
            )

    def _begin_operation(self, token):
        try:
            if type(token) is not _CoordinatorOperationToken:
                raise JsonSerialSessionCoordinatorAdmissionError(
                    "coordinator operation token is invalid"
                )
            with self._state_condition:
                if self._closed:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is closed"
                    )
                if self._closing:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is closing"
                    )
                if self._fault_reason is not None:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is quarantined: "
                        f"{self._fault_reason}"
                    )
                self._require_write_admission_idle_locked("operation")
                self._require_reader_release_idle_locked("operation")
                self._active_operations.add(token)
            self._coordinate_quarantine()
            with self._state_condition:
                if self._fault_reason is not None:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is quarantined: "
                        f"{self._fault_reason}"
                    )
        except BaseException:
            self._force_end_operation(token)
            raise

    def _begin_submit_operation(
        self,
        token,
        reserve_write_admission,
    ):
        try:
            if type(token) is not _CoordinatorOperationToken:
                raise JsonSerialSessionCoordinatorAdmissionError(
                    "coordinator submit token is invalid"
                )
            if type(reserve_write_admission) is not bool:
                raise JsonSerialSessionCoordinatorAdmissionError(
                    "coordinator write-admission selection is invalid"
                )
            with self._state_condition:
                if self._closed:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is closed"
                    )
                if self._closing:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is closing"
                    )
                if self._fault_reason is not None:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is quarantined: "
                        f"{self._fault_reason}"
                    )
                self._require_write_admission_idle_locked("submission")
                self._require_reader_release_idle_locked("submission")
                self._require_frame_handoff_idle_locked("submission")
                if reserve_write_admission and self._active_operations:
                    raise JsonSerialSessionCoordinatorStateError(
                        "write-admitted submission requires an idle coordinator"
                    )
                self._active_operations.add(token)
                self._submit_operations.add(token)
                if reserve_write_admission:
                    self._write_admission_operation = token
            self._coordinate_quarantine()
            with self._state_condition:
                if self._fault_reason is not None:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is quarantined: "
                        f"{self._fault_reason}"
                    )
        except BaseException:
            self._force_end_operation(token)
            raise

    def _begin_cleanup_operation(self, token):
        try:
            if type(token) is not _CoordinatorOperationToken:
                raise JsonSerialSessionCoordinatorAdmissionError(
                    "coordinator cleanup token is invalid"
                )
            with self._state_condition:
                if self._closing and not self._closed:
                    raise JsonSerialSessionCoordinatorStateError(
                        "deadline cleanup cannot overlap coordinator close"
                    )
                self._require_write_admission_idle_locked("deadline cleanup")
                self._require_reader_release_idle_locked("deadline cleanup")
                self._active_operations.add(token)
        except BaseException:
            self._force_end_operation(token)
            raise

    def _begin_reader_release_operation(self, token):
        try:
            if type(token) is not _CoordinatorOperationToken:
                raise JsonSerialSessionCoordinatorAdmissionError(
                    "coordinator reader-release token is invalid"
                )
            with self._state_condition:
                if self._closed:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is closed"
                    )
                if self._closing:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is closing"
                    )
                if self._fault_reason is not None:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is quarantined: "
                        f"{self._fault_reason}"
                    )
                self._require_write_admission_idle_locked("reader release")
                self._require_reader_release_idle_locked("reader release")
                if self._active_operations:
                    raise JsonSerialSessionCoordinatorStateError(
                        "coordinator reader release requires idle operations"
                    )
                self._active_operations.add(token)
                self._reader_release_operation = token
            self._coordinate_quarantine()
            with self._state_condition:
                if self._fault_reason is not None:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON serial session coordinator is quarantined: "
                        f"{self._fault_reason}"
                    )
        except BaseException:
            try: self._force_end_operation(token)
            finally: self._force_end_operation(token)
            raise

    def _mark_reader_release_mutation_started(self, token):
        with self._state_condition:
            if (
                type(token) is not _CoordinatorOperationToken
                or self._reader_release_operation is not token
                or token not in self._active_operations
                or token.reader_release_settled
            ):
                raise JsonSerialSessionCoordinatorStateError(
                    "coordinator reader-release mutation reservation is "
                    "invalid"
                )
            token.reader_release_mutation_started = True
            self._reader_release_cleanup_pending = True

    def _mark_reader_release_settled(self, token):
        with self._state_condition:
            if (
                type(token) is not _CoordinatorOperationToken
                or self._reader_release_operation is not token
                or token not in self._active_operations
            ):
                raise JsonSerialSessionCoordinatorStateError(
                    "coordinator reader-release settlement reservation is "
                    "invalid"
                )
            token.reader_release_settled = True
            self._reader_release_cleanup_pending = False

    def _force_end_operation(self, token):
        frame_handoff_unsettled = False
        if (
            type(token) is _CoordinatorOperationToken
            and token.frame_handoff_obligation
        ):
            frame_handoff_unsettled = self._transport.frame_handoff_pending
        if type(token) is _CoordinatorOperationToken:
            with self._state_condition:
                if (
                    token in self._active_operations
                    and frame_handoff_unsettled
                ):
                    if self._fault_reason is None:
                        self._fault_reason = _FRAME_HANDOFF_FAILURE_REASON
                    self._frame_handoff_cleanup_pending = True
                if self._write_admission_operation is token:
                    self._write_admission_operation = None
                if self._reader_release_operation is token:
                    if token.reader_release_settled:
                        self._reader_release_cleanup_pending = False
                    elif token.reader_release_mutation_started:
                        if self._fault_reason is None:
                            self._fault_reason = (
                                _READER_RELEASE_FAILURE_REASON
                            )
                        self._reader_release_cleanup_pending = True
                    self._reader_release_operation = None
                self._active_operations.discard(token)
                self._submit_operations.discard(token)
                self._state_condition.notify_all()

    def _abandon_public_handoff(self, reason):
        try:
            self._session.abandon_public_returns(reason)
        except BaseException as exc:
            self._record_fault(reason)
            self._transport.quarantine(
                "correlated JSON public-return handoff failed"
            )
            raise JsonSerialSessionCoordinatorStateError(
                "correlated JSON public-return abandonment failed"
            ) from exc

    def _acknowledge_public_handoff(self, result, reason):
        try:
            self._session.acknowledge_public_return(result)
        except BaseException:
            try:
                self._session.acknowledge_public_return(result)
            except BaseException as retry_failure:
                self._abandon_public_handoff(reason)
                raise JsonSerialSessionCoordinatorStateError(
                    reason
                ) from retry_failure
            raise

    def _store_public_handoff(self, result):
        with self._state_condition:
            if type(result) is JsonRequestTicket:
                if result.request_id in self._tickets:
                    raise JsonSerialSessionCoordinatorStateError(
                        "coordinator request identifier collided"
                    )
                self._tickets[result.request_id] = result
                return
            if type(result) in (
                JsonEventDelivery,
                JsonResponseDelivery,
                JsonTelemetryDelivery,
            ):
                if any(
                    retained is result
                    for retained in self._deliveries
                ):
                    raise JsonSerialSessionCoordinatorStateError(
                        "coordinator delivery is already retained"
                    )
                if len(self._deliveries) >= self._delivery_capacity:
                    raise JsonSerialSessionCoordinatorStateError(
                        "coordinator delivery queue became full"
                    )
                self._deliveries.append(result)
                return
        raise JsonSerialSessionCoordinatorStateError(
            "correlated JSON public return has an invalid type"
        )

    def _recover_public_handoffs(self, recovered_settlement=None):
        if recovered_settlement is not None and not callable(
            recovered_settlement
        ):
            raise JsonSerialSessionCoordinatorAdmissionError(
                "recovered handoff settlement must be callable"
            )
        initial_failure = None
        try:
            recovered = self._session.recover_public_returns()
        except BaseException as exc:
            initial_failure = exc
            try:
                recovered = self._session.recover_public_returns()
            except BaseException as retry_failure:
                self._record_fault(
                    "correlated JSON public-return recovery failed"
                )
                self._transport.quarantine(
                    "correlated JSON public-return recovery failed"
                )
                raise JsonSerialSessionCoordinatorStateError(
                    "correlated JSON public-return recovery failed"
                ) from retry_failure
        if type(recovered) is not tuple:
            self._record_fault(
                "correlated JSON public-return recovery was invalid"
            )
            self._transport.quarantine(
                "correlated JSON public-return recovery was invalid"
            )
            raise JsonSerialSessionCoordinatorStateError(
                "correlated JSON public-return recovery was invalid"
            )
        for result in recovered:
            try:
                self._store_public_handoff(result)
            except BaseException:
                self._abandon_public_handoff(
                    "coordinator recovered-return handoff failed"
                )
                raise
            self._acknowledge_public_handoff(
                result,
                "coordinator recovered-return acknowledgement failed",
            )
        if recovered_settlement is not None and recovered:
            if (
                len(recovered) != 1
                or type(recovered[0]) not in (
                    JsonEventDelivery,
                    JsonResponseDelivery,
                    JsonTelemetryDelivery,
                )
            ):
                raise JsonSerialSessionCoordinatorStateError(
                    "recovered inbound frame did not produce one delivery"
                )
            settled = recovered_settlement()
            if settled is not True:
                raise JsonSerialSessionCoordinatorStateError(
                    "recovered inbound frame settlement was not confirmed"
                )
        return initial_failure

    def _raise_operation_failure_after_recovery(
        self,
        operation_failure,
        recovered_settlement=None,
    ):
        try:
            if recovered_settlement is None:
                recovery_failure = self._recover_public_handoffs()
            else:
                recovery_failure = self._recover_public_handoffs(
                    recovered_settlement
                )
        except BaseException as exc:
            recovery_failure = exc
        if recovery_failure is not None:
            self._attach_recovery_failure(
                operation_failure,
                recovery_failure,
            )
        raise operation_failure

    def _attach_recovery_failure(
        self,
        operation_failure,
        recovery_failure,
    ):
        detail = _bounded_exception_detail(recovery_failure)
        note = (
            "coordinator public-return recovery also failed: "
            f"{type(recovery_failure).__name__}"
        )
        if detail:
            note += f": {detail}"
        BaseException.add_note(operation_failure, note)

    def submit(
        self,
        command,
        params=None,
        *,
        timeout,
        write_admission=None,
        maximum_payload_bytes=JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
    ):
        """Submit one request and durably retain the returned ticket.

        ``write_admission`` follows the correlation-core contract: a bounded
        callable invoked outside state locks after validation and deadline
        scheduling. Exact ``True`` admits the request; another return value,
        an exception, an active coordinator operation, or a recursive
        coordinator operation prevents serial transport I/O. The reservation
        remains atomic through outbound commitment, and deadline and
        active-state checks run again after the callback.
        """
        if write_admission is not None and not callable(write_admission):
            raise JsonSerialSessionCoordinatorAdmissionError(
                "coordinator write admission must be callable"
            )
        operation_token = _CoordinatorOperationToken()
        try:
            self._begin_submit_operation(
                operation_token,
                write_admission is not None,
            )
            try:
                ticket = self._session.submit(
                    command,
                    params,
                    timeout=timeout,
                    write_admission=write_admission,
                    maximum_payload_bytes=maximum_payload_bytes,
                )
            except BaseException as operation_failure:
                self._raise_operation_failure_after_recovery(
                    operation_failure
                )
            try:
                self._store_public_handoff(ticket)
            except BaseException:
                self._abandon_public_handoff(
                    "coordinator request ticket handoff failed"
                )
                raise
            self._acknowledge_public_handoff(
                ticket,
                "coordinator request ticket acknowledgement failed",
            )
            return ticket
        finally:
            try:
                self._coordinate_quarantine()
            finally:
                self._force_end_operation(operation_token)

    def _poll_owned(self, operation_token):
        frame = None
        frame_stored = False
        try:
            self._begin_operation(operation_token)
            with self._state_condition:
                if len(self._deliveries) >= self._delivery_capacity:
                    raise JsonSerialSessionCoordinatorStateError(
                        "coordinator delivery queue is full"
                    )
            operation_token.frame_handoff_obligation = True
            frame = self._transport.poll_frame()
            if frame is None:
                operation_token.frame_handoff_obligation = False
                return False
            try:
                delivery = self._session.receive(frame)
            except BaseException as operation_failure:
                self._raise_operation_failure_after_recovery(
                    operation_failure,
                    lambda: self._transport.acknowledge_frame(frame),
                )
            try:
                self._store_public_handoff(delivery)
            except BaseException:
                self._abandon_public_handoff(
                    "coordinator inbound delivery handoff failed"
                )
                raise
            frame_stored = True
            self._acknowledge_public_handoff(
                delivery,
                "coordinator delivery acknowledgement failed",
            )
            frame_acknowledged = self._transport.acknowledge_frame(frame)
            if (
                frame_acknowledged is not True
                or not self._transport.frame_acknowledgement_complete(frame)
            ):
                raise JsonSerialSessionCoordinatorStateError(
                    "transport frame handoff acknowledgement failed"
                )
            operation_token.frame_handoff_obligation = False
            return True
        except BaseException as operation_failure:
            if self._transport.frame_handoff_pending and frame_stored:
                try:
                    frame_acknowledged = self._transport.acknowledge_frame(
                        frame
                    )
                    if frame_acknowledged is not True:
                        raise JsonSerialSessionCoordinatorStateError(
                            "stored frame handoff settlement was not confirmed"
                        )
                except BaseException as settlement_failure:
                    detail = _bounded_exception_detail(settlement_failure)
                    note = (
                        "coordinator frame handoff settlement also failed: "
                        f"{type(settlement_failure).__name__}"
                    )
                    if detail:
                        note += f": {detail}"
                    BaseException.add_note(operation_failure, note)
            handoff_complete = (
                frame is not None
                and self._transport.frame_acknowledgement_complete(frame)
            )
            handoff_unsettled = (
                self._transport.frame_handoff_pending
                or (frame is not None and not handoff_complete)
            )
            if handoff_unsettled:
                if self._session.quarantined or self._transport.quarantined:
                    self._coordinate_quarantine()
                else:
                    self._record_fault(_FRAME_HANDOFF_FAILURE_REASON)
                    self._transport.quarantine(
                        _FRAME_HANDOFF_FAILURE_REASON
                    )
                    self._coordinate_quarantine()
            raise

    def _finish_poll_operation(self, operation_token):
        try:
            self._coordinate_quarantine()
        finally:
            try:
                self._force_end_operation(operation_token)
            finally:
                self._coordinate_quarantine()

    def poll(self):
        """Poll and acknowledge one frame after durable delivery storage."""
        operation_token = _CoordinatorOperationToken()
        try:
            try:
                return self._poll_owned(operation_token)
            finally:
                self._finish_poll_operation(operation_token)
        finally:
            self._finish_poll_operation(operation_token)

    def _release_reader_owned(self, operation_token):
        released = False
        try:
            self._begin_reader_release_operation(operation_token)
            session_owner = self._session.reader_owner
            transport_owner = self._transport.reader_owner
            if session_owner is not None and transport_owner is None:
                raise JsonSerialSessionCoordinatorStateError(
                    "JSON session reader remains bound without transport "
                    "ownership"
                )
            if (
                session_owner is not None
                and transport_owner is not None
                and session_owner is not transport_owner
            ):
                raise JsonSerialSessionCoordinatorStateError(
                    "JSON session and transport reader ownership differ"
                )
            if transport_owner is None:
                self._mark_reader_release_settled(operation_token)
            else:
                if transport_owner is not threading.current_thread():
                    raise JsonSerialSessionCoordinatorAdmissionError(
                        "JSON reader ownership can only be released by its "
                        "current reader"
                    )
                if session_owner is not None:
                    self._session.validate_reader_release()
                self._transport.validate_reader_release()
                self._mark_reader_release_mutation_started(operation_token)
                if session_owner is not None:
                    session_released = self._session.release_reader()
                    if session_released is not True:
                        raise JsonSerialSessionCoordinatorStateError(
                            "JSON session reader release was not confirmed"
                        )
                transport_released = self._transport.release_reader()
                if transport_released is not True:
                    raise JsonSerialSessionCoordinatorStateError(
                        "JSON transport reader release was not confirmed"
                    )
                self._mark_reader_release_settled(operation_token)
                self._coordinate_quarantine()
                released = True
        except BaseException:
            if (
                operation_token.reader_release_mutation_started
                and not operation_token.reader_release_settled
            ):
                session_owner = self._session.reader_owner
                transport_owner = self._transport.reader_owner
                if session_owner is None and transport_owner is None:
                    self._mark_reader_release_settled(operation_token)
                else:
                    self._record_fault(_READER_RELEASE_FAILURE_REASON)
                    self._coordinate_quarantine()
            raise
        return released

    def release_reader(self):
        """Release inbound thread ownership at a complete frame boundary."""
        operation_token = _CoordinatorOperationToken()
        try:
            try:
                return self._release_reader_owned(operation_token)
            finally:
                self._force_end_operation(operation_token)
        finally:
            self._force_end_operation(operation_token)

    def pop_delivery(self):
        """Return the oldest retained delivery without blocking."""
        with self._state_condition:
            self._require_write_admission_idle_locked("delivery retrieval")
            if not self._deliveries:
                return None
            return self._deliveries.popleft()

    def snapshot(self, ticket):
        operation_token = _CoordinatorOperationToken()
        try:
            self._begin_operation(operation_token)
            self._require_retained_ticket(ticket)
            return self._session.snapshot(ticket)
        finally:
            try:
                self._coordinate_quarantine()
            finally:
                self._force_end_operation(operation_token)

    def take_terminal(self, ticket):
        operation_token = _CoordinatorOperationToken()
        try:
            self._begin_operation(operation_token)
            self._require_retained_ticket(ticket)
            return self._session.take_terminal(ticket)
        finally:
            try:
                self._coordinate_quarantine()
            finally:
                self._force_end_operation(operation_token)

    def acknowledge_terminal(self, ticket):
        operation_token = _CoordinatorOperationToken()
        try:
            self._begin_operation(operation_token)
            self._require_retained_ticket(ticket)
            try:
                self._session.acknowledge_terminal(ticket)
            finally:
                if self._session.terminal_acknowledgement_complete(
                    ticket
                ):
                    with self._state_condition:
                        if self._tickets.get(ticket.request_id) is ticket:
                            del self._tickets[ticket.request_id]
        finally:
            try:
                self._coordinate_quarantine()
            finally:
                self._force_end_operation(operation_token)

    def expire(self):
        operation_token = _CoordinatorOperationToken()
        try:
            self._begin_operation(operation_token)
            self._session.expire()
        finally:
            try:
                self._coordinate_quarantine()
            finally:
                self._force_end_operation(operation_token)

    def retry_deadline_cleanup(self):
        """Retry retained session deadline-cancellation ownership."""
        operation_token = _CoordinatorOperationToken()
        try:
            self._begin_cleanup_operation(operation_token)
            return self._session.retry_deadline_cleanup()
        finally:
            try:
                self._coordinate_quarantine()
            finally:
                self._force_end_operation(operation_token)

    def _require_retained_ticket(self, ticket):
        if type(ticket) is not JsonRequestTicket:
            raise JsonSerialSessionCoordinatorAdmissionError(
                "coordinator request ticket is invalid"
            )
        with self._state_condition:
            if self._tickets.get(ticket.request_id) is not ticket:
                raise JsonSerialSessionCoordinatorAdmissionError(
                    "request ticket is not retained by this coordinator"
                )

    def close(
        self,
        *,
        timeout=JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
    ):
        """Close owned serial state."""
        close_timeout = _positive_timeout(timeout, "coordinator close timeout")
        deadline = time.monotonic() + close_timeout
        if not math.isfinite(deadline):
            raise JsonSerialSessionCoordinatorAdmissionError(
                "coordinator close deadline is not representable"
            )

        with self._close_lock:
            return self._close_owned(deadline)

    def _close_owned(self, deadline):
        with self._state_condition:
            if self._closed:
                if self._terminal_close_error is not None:
                    error = JsonSerialSessionCoordinatorCloseError(
                        self._terminal_close_error
                    )
                    if self._terminal_close_cause is not None:
                        raise error from self._terminal_close_cause
                    raise error
                return
            self._require_write_admission_idle_locked("close")
            self._closing = True
            while self._active_operations:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._record_fault(
                        "coordinator close timed out with active operations"
                    )
                    self._transport.quarantine(
                        "coordinator close timed out before serial ownership"
                    )
                    raise JsonSerialSessionCoordinatorCloseError(
                        "coordinator close timed out with active operations"
                    )
                self._state_condition.wait(
                    min(remaining, threading.TIMEOUT_MAX)
                )

        self._coordinate_quarantine()
        with self._state_condition:
            if (
                self._reader_release_cleanup_pending
                or self._frame_handoff_cleanup_pending
            ):
                raise JsonSerialSessionCoordinatorCloseError(
                    "coordinator close awaits ownership quarantine"
                )

        session_failures = []
        if not self._session_shutdown_complete:
            try:
                if self._session.quarantined:
                    session_failures.append(
                        JsonSessionQuarantinedError(
                            "correlated JSON session was already quarantined"
                        )
                    )
                else:
                    self._session.close()
            except Exception as exc:
                session_failures.append(exc)
                if (
                    isinstance(exc, JsonSessionAdmissionError)
                    and not (
                        self._session.closed
                        or self._session.quarantined
                    )
                ):
                    try:
                        self._session.abandon_public_returns(
                            "coordinator close abandoned unresolved "
                            "public-return ownership"
                        )
                    except JsonSessionError as abandonment_failure:
                        session_failures.append(abandonment_failure)
            finally:
                self._session_shutdown_complete = (
                    self._session.closed or self._session.quarantined
                )
                self._coordinate_quarantine()

        serial_failures = []
        if not self._serial_shutdown_complete:
            close_failure = None
            try:
                self._close_serial()
            except Exception as exc:
                close_failure = exc
                serial_failures.append(exc)
            try:
                open_state = self._serial_port.is_open
            except Exception as exc:
                serial_failures.append(exc)
                open_state = None

            if type(open_state) is bool and not open_state:
                self._serial_shutdown_complete = True
                if close_failure is not None:
                    self._retained_serial_close_failure = close_failure
            elif not serial_failures:
                serial_failures.append(
                    RuntimeError(
                        "serial connection did not verify closed"
                    )
                )

        with self._state_condition:
            fully_closed = (
                self._session_shutdown_complete
                and self._serial_shutdown_complete
            )
            if fully_closed:
                self._closed = True
                self._closing = False
                self._state_condition.notify_all()

        retained_serial_failure = self._retained_serial_close_failure
        all_failures = list(session_failures)
        all_failures.extend(serial_failures)
        if (
            retained_serial_failure is not None
            and all(
                failure is not retained_serial_failure
                for failure in all_failures
            )
        ):
            all_failures.append(retained_serial_failure)

        if not fully_closed:
            incomplete_components = []
            if not self._session_shutdown_complete:
                incomplete_components.append("session shutdown")
            if not self._serial_shutdown_complete:
                incomplete_components.append("serial closure")
            failure_reason = (
                "coordinator close remains incomplete: "
                + " and ".join(incomplete_components)
            )
            self._record_fault(failure_reason)
            self._raise_close_failure(failure_reason, all_failures)

        if all_failures:
            if session_failures and retained_serial_failure is not None:
                failure_reason = (
                    "correlated JSON session and serial close reported "
                    "failures after verified closure"
                )
            elif retained_serial_failure is not None:
                failure_reason = (
                    "serial close reported a failure after verified closure"
                )
            else:
                failure_reason = (
                    "correlated JSON session did not close cleanly"
                )
            self._record_fault(failure_reason)
            self._raise_close_failure(
                failure_reason,
                all_failures,
                terminal=True,
            )
        if not (
            self._session.quarantined or self._transport.quarantined
        ):
            with self._state_condition:
                self._fault_reason = None
                self._session_quarantine_bridge_failure = None

    def _raise_close_failure(self, reason, failures, *, terminal=False):
        unique_failures = []
        for failure in failures:
            if all(
                retained is not failure
                for retained in unique_failures
            ):
                unique_failures.append(failure)
        if len(unique_failures) == 1:
            cause = unique_failures[0]
        elif unique_failures:
            cause = ExceptionGroup(
                "coordinator close failures",
                unique_failures,
            )
        else:
            cause = None
        if terminal:
            with self._state_condition:
                self._fault_reason = reason
                self._terminal_close_error = reason
                self._terminal_close_cause = cause
        error = JsonSerialSessionCoordinatorCloseError(reason)
        if cause is not None:
            raise error from cause
        raise error


__all__ = (
    "JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS",
    "JSON_COORDINATOR_MAXIMUM_DELIVERIES",
    "JsonSerialSessionCoordinator",
    "JsonSerialSessionCoordinatorAdmissionError",
    "JsonSerialSessionCoordinatorCloseError",
    "JsonSerialSessionCoordinatorError",
    "JsonSerialSessionCoordinatorStateError",
    "SerialWriteCancellationBoundary",
    "close_unowned_serial_port",
)
