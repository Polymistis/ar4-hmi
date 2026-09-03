"""Bounded JSON-only startup for the main controller."""

from dataclasses import dataclass
import hashlib
import math
import threading
import time

from .coordinator import close_unowned_serial_port
from .main_controller import (
    JsonMainControllerClient,
    JsonMainControllerTerminal,
)
from .messages import Request, encode_message
from .schemas import (
    JSON_MAIN_COMMAND_MANIFEST,
    JsonCommandSchemaError,
    JsonMainHelloResult,
    JsonMainHomeReferenceResult,
    JsonMainPositionResult,
    parse_main_motion_position_result,
    validate_main_config_ext_axis_request,
    validate_main_set_position_request,
    validate_main_update_params_request,
)
from .session import (
    JsonEventDelivery,
    JsonResponseDelivery,
    JsonTelemetryDelivery,
)
from .transport import JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS


_JSON_PHYSICAL_STOP_COMMANDS = frozenset(
    (*JSON_MAIN_COMMAND_MANIFEST, "idle")
)


_JSON_PHYSICAL_STOP_SOURCES = frozenset(
    (
        "emergency_stop_active",
        "emergency_stop_event",
        "emergency_stop_terminal",
    )
)


_JSON_TERMINAL_POSITION_PHYSICAL_STOP_COMMANDS = frozenset(
    (
        "calibrate",
        "jog_tool",
        "move_arc",
        "move_cartesian",
        "move_circle",
        "move_joints",
        "move_linear",
        "move_spline",
        "move_vision",
    )
)

_JSON_CONFIGURATION_FINGERPRINT_DOMAIN = (
    b"ar4-json-main-configuration-v1\0"
)
_JSON_CONFIGURATION_FINGERPRINT_REQUESTS = (
    (1, "update_params"),
    (2, "config_ext_axis"),
)


class JsonMainControllerStartupError(RuntimeError):
    """JSON-only startup cannot establish trusted controller ownership."""

    def __init__(self, message, *, terminal=None, position=None):
        if (
            terminal is not None
            and type(terminal) is not JsonMainControllerTerminal
        ):
            raise TypeError("startup retained terminal is invalid")
        if position is not None and type(position) is not JsonMainPositionResult:
            raise TypeError("startup retained position is invalid")
        if terminal is not None and position is not None:
            raise TypeError("startup error has conflicting retained disposition")
        self.terminal = terminal
        self.position = position
        super().__init__(message)


class JsonMainControllerStartupCleanupError(JsonMainControllerStartupError):
    """JSON startup and serial-owner cleanup both reported failures."""

    def __init__(self, operation_error, cleanup_error, client=None):
        if not isinstance(operation_error, BaseException):
            raise TypeError("startup operation error must be an exception")
        if not isinstance(cleanup_error, BaseException):
            raise TypeError("startup cleanup error must be an exception")
        if client is not None and type(client) is not JsonMainControllerClient:
            raise TypeError("startup cleanup client is invalid")
        operation_detail = _bounded_exception_detail(operation_error)
        cleanup_detail = _bounded_exception_detail(cleanup_error)
        terminal = (
            operation_error.terminal
            if isinstance(operation_error, JsonMainControllerStartupError)
            else None
        )
        position = (
            operation_error.position
            if isinstance(operation_error, JsonMainControllerStartupError)
            else None
        )
        super().__init__(
            "main-controller JSON startup failed and cleanup also failed: "
            f"startup={operation_detail}; cleanup={cleanup_detail}",
            terminal=terminal,
            position=position,
        )
        self.operation_error = operation_error
        self.cleanup_error = cleanup_error
        self.client = client


@dataclass(frozen=True)
class JsonMainControllerPhysicalStop:
    """Validated physical-stop evidence from correlated JSON ownership."""

    command: str
    source: str

    def __post_init__(self):
        if self.command not in _JSON_PHYSICAL_STOP_COMMANDS:
            raise JsonMainControllerStartupError(
                "JSON physical-stop command is invalid"
            )
        if self.source not in _JSON_PHYSICAL_STOP_SOURCES:
            raise JsonMainControllerStartupError(
                "JSON physical-stop source is invalid"
            )


class JsonMainControllerPhysicalStopError(JsonMainControllerStartupError):
    """JSON startup stopped after publishing physical-stop evidence."""

    def __init__(self, physical_stop, position=None):
        if type(physical_stop) is not JsonMainControllerPhysicalStop:
            raise TypeError("JSON startup physical-stop evidence is invalid")
        self.physical_stop = physical_stop
        super().__init__(
            "physical emergency stop reported during main-controller JSON "
            f"{physical_stop.command} ({physical_stop.source})",
            position=position,
        )


class JsonMainControllerPhysicalStopPublicationError(
    JsonMainControllerStartupError
):
    """Physical-stop evidence could not enter the HMI publication path."""

    def __init__(self, physical_stop, publication_error, position=None):
        if type(physical_stop) is not JsonMainControllerPhysicalStop:
            raise TypeError("JSON startup physical-stop evidence is invalid")
        if not isinstance(publication_error, BaseException):
            raise TypeError("physical-stop publication error is invalid")
        self.physical_stop = physical_stop
        self.publication_error = publication_error
        detail = _bounded_exception_detail(publication_error)
        super().__init__(
            "main-controller JSON physical-stop publication failed: "
            f"{detail}",
            position=position,
        )


def _bounded_exception_detail(exc):
    if not isinstance(exc, BaseException):
        raise TypeError("startup error detail requires an exception")
    try:
        detail = str(exc)
    except BaseException:
        detail = ""
    normalized = " ".join(detail.split())[:256]
    return normalized or type(exc).__name__


def retain_main_controller_startup_disposition_error(
    position,
    operation_error,
    context,
):
    """Retain one-shot position metadata across a later startup failure."""
    if type(position) is not JsonMainPositionResult:
        raise TypeError("retained startup position is invalid")
    if not isinstance(operation_error, BaseException):
        raise TypeError("retained startup operation error is invalid")
    if (
        not isinstance(context, str)
        or not context
        or context != context.strip()
        or "\r" in context
        or "\n" in context
    ):
        raise TypeError("retained startup error context is invalid")
    if not isinstance(operation_error, Exception):
        return operation_error
    if not (
        position.speed_limited
        or position.controller_debug
        or position.motion_fault
    ):
        return operation_error
    if isinstance(operation_error, JsonMainControllerStartupError):
        terminal_position = (
            operation_error.terminal.parsed_result
            if operation_error.terminal is not None
            else None
        )
        if (
            operation_error.position is position
            or terminal_position is position
        ):
            return operation_error
        if type(operation_error) is JsonMainControllerPhysicalStopError:
            return JsonMainControllerPhysicalStopError(
                operation_error.physical_stop,
                position,
            )
        if (
            type(operation_error)
            is JsonMainControllerPhysicalStopPublicationError
        ):
            return JsonMainControllerPhysicalStopPublicationError(
                operation_error.physical_stop,
                operation_error.publication_error,
                position,
            )
    detail = _bounded_exception_detail(operation_error)
    return JsonMainControllerStartupError(
        f"{context} after consuming JSON position disposition: "
        f"speed_limited={position.speed_limited!r}; "
        f"controller_debug={position.controller_debug!r}; "
        f"motion_fault={position.motion_fault!r}; operation={detail}",
        position=position,
    )


@dataclass(frozen=True)
class JsonMainControllerStartupResult:
    """Validated identity and position data from JSON-only startup."""

    hello: JsonMainHelloResult
    position: JsonMainPositionResult
    home_reference: JsonMainHomeReferenceResult

    def __post_init__(self):
        if type(self.hello) is not JsonMainHelloResult:
            raise JsonMainControllerStartupError(
                "main-controller JSON startup hello result is invalid"
            )
        if type(self.position) is not JsonMainPositionResult:
            raise JsonMainControllerStartupError(
                "main-controller JSON startup position result is invalid"
            )
        if type(self.home_reference) is not JsonMainHomeReferenceResult:
            raise JsonMainControllerStartupError(
                "main-controller JSON home-reference result is invalid"
            )


def controller_identity_from_json_hello(hello):
    """Convert validated JSON identity data to the HMI identity value."""
    if type(hello) is not JsonMainHelloResult:
        raise TypeError("main-controller JSON hello result is invalid")
    from ARrobots.HMI.joint_motion import ControllerIdentity

    return ControllerIdentity(
        controller_hardware_id=hello.identity.controller_hardware_id,
        driver_model=hello.identity.driver_model,
        firmware_version=hello.firmware.version,
        robot_model=hello.identity.robot_model,
        robot_version=hello.identity.robot_version,
        serial_number=hello.identity.serial_number,
        asset_tag=hello.identity.asset_tag,
        protocol_capabilities=hello.capabilities,
    )


@dataclass(frozen=True)
class JsonMainControllerStartupConfiguration:
    """Immutable semantic configuration for persistent JSON startup."""

    tool_translation_millimeters: tuple
    tool_rotation_degrees: tuple
    motor_directions: tuple
    calibration_directions: tuple
    calibration_switch_active_high: tuple
    positive_joint_limits_degrees: tuple
    negative_joint_limits_degrees: tuple
    steps_per_degree: tuple
    encoder_counts_per_step: tuple
    dh_theta_degrees: tuple
    dh_alpha_degrees: tuple
    dh_d_millimeters: tuple
    dh_a_millimeters: tuple
    external_axis_travel_units: tuple
    external_axis_drive_rotations: tuple
    external_axis_motor_steps: tuple
    robot_joints_millidegrees: tuple
    external_axes_milliunits: tuple

    def __post_init__(self):
        if any(
            type(getattr(self, field_name)) is not tuple
            for field_name in self.__dataclass_fields__
        ):
            raise JsonMainControllerStartupError(
                "main-controller persistent startup configuration is invalid"
            )
        try:
            validate_main_update_params_request(self.update_params)
            validate_main_config_ext_axis_request(self.config_ext_axis)
            validate_main_set_position_request(self.set_position)
        except JsonCommandSchemaError as exc:
            raise JsonMainControllerStartupError(
                "main-controller persistent startup configuration is invalid"
            ) from exc

    @property
    def update_params(self):
        return {
            "calibration_directions": self.calibration_directions,
            "calibration_switch_active_high": (
                self.calibration_switch_active_high
            ),
            "dh_a_millimeters": self.dh_a_millimeters,
            "dh_alpha_degrees": self.dh_alpha_degrees,
            "dh_d_millimeters": self.dh_d_millimeters,
            "dh_theta_degrees": self.dh_theta_degrees,
            "encoder_counts_per_step": self.encoder_counts_per_step,
            "motor_directions": self.motor_directions,
            "negative_joint_limits_degrees": (
                self.negative_joint_limits_degrees
            ),
            "positive_joint_limits_degrees": (
                self.positive_joint_limits_degrees
            ),
            "steps_per_degree": self.steps_per_degree,
            "tool_rotation_degrees": self.tool_rotation_degrees,
            "tool_translation_millimeters": (
                self.tool_translation_millimeters
            ),
        }

    @property
    def config_ext_axis(self):
        return {
            "drive_rotations": self.external_axis_drive_rotations,
            "motor_steps": self.external_axis_motor_steps,
            "travel_units": self.external_axis_travel_units,
        }

    @property
    def set_position(self):
        return {
            "external_axes_milliunits": self.external_axes_milliunits,
            "robot_joints_millidegrees": self.robot_joints_millidegrees,
        }

    @property
    def motion_trace_scale(self):
        return {
            "encoder_counts_per_step": self.encoder_counts_per_step,
            "steps_per_degree": self.steps_per_degree,
        }

    @property
    def configuration_fingerprint(self):
        digest = hashlib.sha256(_JSON_CONFIGURATION_FINGERPRINT_DOMAIN)
        parameters = {
            "update_params": self.update_params,
            "config_ext_axis": self.config_ext_axis,
        }
        for request_id, command in _JSON_CONFIGURATION_FINGERPRINT_REQUESTS:
            digest.update(
                encode_message(Request(request_id, command, parameters[command]))
            )
        return "sha256:" + digest.hexdigest()


@dataclass(frozen=True)
class JsonMainControllerPersistentStartupResult:
    """Validated startup data plus retained persistent-session ownership."""

    startup: JsonMainControllerStartupResult
    client: JsonMainControllerClient
    configuration: JsonMainControllerStartupConfiguration

    def __post_init__(self):
        if type(self.startup) is not JsonMainControllerStartupResult:
            raise JsonMainControllerStartupError(
                "persistent startup data is invalid"
            )
        if type(self.client) is not JsonMainControllerClient:
            raise JsonMainControllerStartupError(
                "persistent startup client is invalid"
            )
        if type(self.configuration) is not JsonMainControllerStartupConfiguration:
            raise JsonMainControllerStartupError(
                "persistent startup configuration is invalid"
            )
        if (
            self.client.closed
            or self.client.closing
            or self.client.quarantined
            or not self.client.session_ready
            or self.client.configuration_sync_required
            or self.client.pending_tickets
            or self.client.delivery_count != 0
            or self.client.deadline_cleanup_count != 0
            or self.client.reader_owner is not None
            or self.client.session_binding != self.startup.hello
        ):
            raise JsonMainControllerStartupError(
                "persistent startup client is not ready"
            )

    @property
    def hello(self):
        return self.startup.hello

    @property
    def configuration_fingerprint(self):
        return self.configuration.configuration_fingerprint

    @property
    def position(self):
        return self.startup.position

    @property
    def home_reference(self):
        return self.startup.home_reference


def _positive_timeout(value, field_name):
    if type(value) not in (int, float):
        raise JsonMainControllerStartupError(f"{field_name} must be numeric")
    try:
        normalized = float(value)
    except OverflowError as exc:
        raise JsonMainControllerStartupError(
            f"{field_name} must be positive and finite"
        ) from exc
    if not math.isfinite(normalized) or normalized <= 0:
        raise JsonMainControllerStartupError(f"{field_name} must be positive")
    return normalized


def _cancellation_requested(cancellation_boundary):
    try:
        requested = cancellation_boundary.is_set()
    except AttributeError as exc:
        raise TypeError(
            "controller startup cancellation boundary lacks is_set"
        ) from exc
    if type(requested) is not bool:
        raise TypeError(
            "controller startup cancellation state must be boolean"
        )
    return requested


class _JsonStartupWriteCommitment:
    """Order one JSON write admission against startup cancellation."""

    def __init__(self, cancellation_boundary):
        self._cancellation_boundary = cancellation_boundary
        self._committed = threading.Event()
        write_reservation = getattr(
            cancellation_boundary,
            "write_reservation",
            None,
        )
        if not callable(write_reservation):
            raise TypeError(
                "controller startup cancellation boundary lacks serial ordering"
            )
        self._write_reservation = write_reservation

    @property
    def committed(self):
        return self._committed.is_set()

    def admit(self):
        reservation = self._write_reservation()
        if (
            not callable(getattr(reservation, "__enter__", None))
            or not callable(getattr(reservation, "__exit__", None))
        ):
            raise TypeError(
                "controller startup write reservation is invalid"
            )
        with reservation:
            if _cancellation_requested(self._cancellation_boundary):
                raise TimeoutError("controller startup cancelled")
            self._committed.set()
            return True


def _submit_with_cancellation_commitment(cancellation_boundary, submit):
    if not callable(submit):
        raise TypeError("JSON startup submission must be callable")
    commitment = _JsonStartupWriteCommitment(cancellation_boundary)
    ticket = submit(commitment.admit)
    if not commitment.committed:
        raise RuntimeError(
            "JSON startup submission returned without write commitment"
        )
    return ticket


def _publish_physical_stop(
    physical_stop_callback,
    command,
    source,
    position=None,
):
    physical_stop = JsonMainControllerPhysicalStop(
        command,
        source,
    )
    try:
        published = physical_stop_callback(physical_stop)
    except BaseException as exc:
        raise JsonMainControllerPhysicalStopPublicationError(
            physical_stop, exc, position
        ) from exc
    if published is not True:
        publication_error = RuntimeError(
            "physical-stop publication callback must return true"
        )
        raise JsonMainControllerPhysicalStopPublicationError(
            physical_stop, publication_error, position
        ) from publication_error
    raise JsonMainControllerPhysicalStopError(physical_stop, position)


def _raise_event_delivery(
    delivery,
    command,
    physical_stop_callback,
):
    event_name = delivery.event.event
    if event_name == "emergency_stop":
        event_data = delivery.event.data
        if (
            len(event_data) != 1
            or frozenset(event_data) != frozenset(("asserted",))
            or event_data.get("asserted") is not True
        ):
            raise JsonMainControllerStartupError(
                "main-controller emergency-stop event data is invalid"
            )
        _publish_physical_stop(
            physical_stop_callback,
            command,
            "emergency_stop_event",
        )
    raise JsonMainControllerStartupError(
        f"unexpected controller event during JSON startup: {event_name}"
    )


def _terminal_failure_error(
    terminal,
    context,
):
    response = terminal.response
    failure = terminal.failure
    if failure is None:
        return JsonMainControllerStartupError(
            f"{context} returned non-completed status {response.status!r}",
            terminal=terminal,
        )
    if failure.code == "emergency_stop_active":
        return JsonMainControllerStartupError(
            "physical-stop rejection bypassed pre-settlement publication",
            terminal=terminal,
        )
    details = dict(failure.details)
    details_text = f"; details={details!r}" if details else ""
    return JsonMainControllerStartupError(
        f"{context} {response.status}: {failure.code}: {failure.message}"
        f"{details_text}",
        terminal=terminal,
    )


def _capture_terminal_physical_stop_error(
    terminal,
    physical_stop_callback,
):
    response = terminal.response
    failure = response.error
    source = None
    position = None
    if failure is not None and failure.code == "emergency_stop_active":
        source = "emergency_stop_active"
    elif (
        response.status == "cancelled"
        and failure is not None
        and failure.code == "emergency_stop"
    ):
        source = "emergency_stop_terminal"
        if response.cmd in _JSON_TERMINAL_POSITION_PHYSICAL_STOP_COMMANDS:
            position = parse_main_motion_position_result(
                failure.details["position"]
            )
    elif (
        response.status == "completed"
        and type(terminal.parsed_result) is JsonMainPositionResult
    ):
        if terminal.parsed_result.motion_fault == "EA":
            source = "emergency_stop_active"
        elif terminal.parsed_result.motion_fault == "EB":
            source = "emergency_stop_event"
    if source is None:
        return None
    try:
        _publish_physical_stop(
            physical_stop_callback,
            response.cmd,
            source,
            position,
        )
    except (
        JsonMainControllerPhysicalStopError,
        JsonMainControllerPhysicalStopPublicationError,
    ) as stop_error:
        if type(terminal.parsed_result) is JsonMainPositionResult:
            if type(stop_error) is JsonMainControllerPhysicalStopError:
                return JsonMainControllerPhysicalStopError(
                    stop_error.physical_stop,
                    terminal.parsed_result,
                )
            return JsonMainControllerPhysicalStopPublicationError(
                stop_error.physical_stop,
                stop_error.publication_error,
                terminal.parsed_result,
            )
        return stop_error
    raise JsonMainControllerStartupError(
        "physical-stop publication returned without a disposition"
    )


def _terminal_settlement_error(terminal, settlement_error, context):
    if (
        terminal.response.cmd == "get_position_disposition"
        and type(terminal.parsed_result) is JsonMainPositionResult
    ):
        retained_error = retain_main_controller_startup_disposition_error(
            terminal.parsed_result,
            settlement_error,
            f"{context} terminal settlement failed",
        )
        if retained_error is not settlement_error:
            return retained_error
        return JsonMainControllerStartupError(
            f"{context} terminal settlement failed: "
            f"{_bounded_exception_detail(settlement_error)}",
            terminal=terminal,
        )
    if terminal.response.cmd == "get_position_disposition":
        terminal_error = _terminal_failure_error(terminal, context)
        return JsonMainControllerStartupError(
            f"{terminal_error}; terminal settlement failed: "
            f"{_bounded_exception_detail(settlement_error)}",
            terminal=terminal,
        )
    return JsonMainControllerStartupError(
        f"{context} terminal settlement failed: "
        f"{_bounded_exception_detail(settlement_error)}",
        terminal=terminal,
    )


def _raise_after_terminal_drain(
    client,
    command,
    physical_stop_callback,
    drain_timeout,
    primary_error,
    cause=None,
):
    try:
        drain_main_controller_input(
            client,
            command,
            physical_stop_callback,
            drain_timeout,
        )
    except (
        JsonMainControllerPhysicalStopError,
        JsonMainControllerPhysicalStopPublicationError,
    ) as stop_error:
        raise stop_error from primary_error
    except BaseException as drain_error:
        raise primary_error from drain_error
    if cause is not None:
        raise primary_error from cause
    raise primary_error


def _await_completed_terminal(
    client,
    ticket,
    context,
    *,
    physical_stop_callback,
    drain_timeout,
    _return_terminal=False,
    accepted_response_allowed=False,
):
    retained_stop_error = None
    while True:
        delivery = client.pop_delivery()
        if delivery is None:
            try:
                client.poll()
            except BaseException:
                if retained_stop_error is not None:
                    raise retained_stop_error
                raise
            continue
        if type(delivery) is JsonResponseDelivery:
            if delivery.ticket is not ticket:
                if retained_stop_error is not None:
                    raise retained_stop_error
                raise JsonMainControllerStartupError(
                    f"{context} received a response for another request"
                )
            if (
                delivery.response.status == "accepted"
                and accepted_response_allowed
            ):
                continue
            try:
                retained_terminal = JsonMainControllerTerminal(delivery.response)
            except BaseException:
                if retained_stop_error is not None:
                    raise retained_stop_error
                raise
            stop_error = retained_stop_error
            if stop_error is None:
                stop_error = _capture_terminal_physical_stop_error(
                    retained_terminal,
                    physical_stop_callback,
                )
            try:
                terminal = client.take_terminal(ticket)
                client.acknowledge_terminal(ticket)
            except BaseException as settlement_error:
                if stop_error is not None:
                    raise stop_error from settlement_error
                primary_error = _terminal_settlement_error(
                    retained_terminal, settlement_error, context
                )
                _raise_after_terminal_drain(
                    client,
                    ticket.command,
                    physical_stop_callback,
                    drain_timeout,
                    primary_error,
                    settlement_error,
                )
            if stop_error is not None:
                raise stop_error
            if terminal.response is not delivery.response:
                ownership_error = JsonMainControllerStartupError(
                    "terminal response ownership changed during settlement",
                    terminal=retained_terminal,
                )
                _raise_after_terminal_drain(
                    client,
                    ticket.command,
                    physical_stop_callback,
                    drain_timeout,
                    ownership_error,
                )
            if _return_terminal:
                return terminal
            if terminal.response.status != "completed":
                _raise_after_terminal_drain(
                    client,
                    ticket.command,
                    physical_stop_callback,
                    drain_timeout,
                    _terminal_failure_error(terminal, context),
                )
            parsed_result = terminal.parsed_result
            if (
                type(parsed_result) is JsonMainPositionResult
                and parsed_result.motion_fault
            ):
                position_error = JsonMainControllerStartupError(
                    "main-controller JSON startup reported motion fault: "
                    + parsed_result.motion_fault,
                    terminal=terminal,
                )
                _raise_after_terminal_drain(
                    client,
                    ticket.command,
                    physical_stop_callback,
                    drain_timeout,
                    position_error,
                )
            return parsed_result
        if type(delivery) is JsonEventDelivery:
            if retained_stop_error is not None:
                raise retained_stop_error
            try:
                _raise_event_delivery(
                    delivery, ticket.command, physical_stop_callback)
            except (JsonMainControllerPhysicalStopError,
                    JsonMainControllerPhysicalStopPublicationError) as error:
                retained_stop_error = error
            continue
        if retained_stop_error is not None:
            raise retained_stop_error
        if type(delivery) is JsonTelemetryDelivery:
            raise JsonMainControllerStartupError(
                "unexpected controller telemetry during JSON startup"
            )
        raise JsonMainControllerStartupError(
            "JSON startup received an unknown delivery type"
        )


def await_main_controller_terminal(
    client, ticket, context, *, physical_stop_callback, drain_timeout,
    accepted_response_allowed=False,
):
    """Return an acknowledged terminal; the caller retains reader ownership."""
    return _await_completed_terminal(
        client, ticket, context, physical_stop_callback=physical_stop_callback,
        drain_timeout=drain_timeout, _return_terminal=True,
        accepted_response_allowed=accepted_response_allowed)


def drain_main_controller_input(
    client,
    command,
    physical_stop_callback,
    drain_timeout,
):
    """Drain owned input while publishing emergency-stop evidence."""
    deadline = time.monotonic() + drain_timeout
    if not math.isfinite(deadline):
        raise JsonMainControllerStartupError(
            "JSON main-controller input-drain deadline is not representable"
        )
    while True:
        delivery = client.pop_delivery()
        if delivery is not None:
            if type(delivery) is JsonEventDelivery:
                _raise_event_delivery(
                    delivery,
                    command,
                    physical_stop_callback,
                )
            if type(delivery) is JsonTelemetryDelivery:
                raise JsonMainControllerStartupError(
                    "unexpected telemetry while draining main-controller JSON"
                )
            if type(delivery) is JsonResponseDelivery:
                raise JsonMainControllerStartupError(
                    "unexpected response while draining main-controller JSON"
                )
            raise JsonMainControllerStartupError(
                "main-controller JSON drain received an unknown delivery type"
            )
        if time.monotonic() >= deadline:
            raise JsonMainControllerStartupError(
                "main-controller JSON input drain timed out"
            )
        polled = client.poll()
        if type(polled) is not bool:
            raise JsonMainControllerStartupError(
                "main-controller JSON input poll returned an invalid disposition"
            )
        if not polled and not client.has_unread_input:
            return


def _submit_startup_request(
    client,
    cancellation_boundary,
    stop_context_command,
    submit,
    physical_stop_callback,
    drain_timeout,
):
    if _cancellation_requested(cancellation_boundary):
        drain_main_controller_input(
            client,
            stop_context_command,
            physical_stop_callback,
            drain_timeout,
        )
        raise TimeoutError("controller startup cancelled")
    try:
        return _submit_with_cancellation_commitment(
            cancellation_boundary,
            submit,
        )
    except TimeoutError:
        if _cancellation_requested(cancellation_boundary):
            drain_main_controller_input(
                client,
                stop_context_command,
                physical_stop_callback,
                drain_timeout,
            )
        raise


def run_main_controller_json_startup(
    serial_port,
    configuration,
    cancellation_boundary,
    *,
    physical_stop_callback,
    request_timeout,
    close_timeout,
):
    """Apply startup configuration while retaining correlated JSON ownership."""

    if type(configuration) is not JsonMainControllerStartupConfiguration:
        raise TypeError("persistent startup configuration is invalid")
    if not callable(physical_stop_callback):
        raise TypeError("physical-stop publication callback must be callable")
    normalized_request_timeout = _positive_timeout(
        request_timeout,
        "JSON startup request timeout",
    )
    normalized_close_timeout = _positive_timeout(
        close_timeout,
        "JSON startup close timeout",
    )
    cancellation_drain_timeout = min(
        normalized_request_timeout,
        JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS,
    )
    if not all(callable(getattr(cancellation_boundary, name, None))
               for name in ("is_set", "write_reservation")):
        raise TypeError("controller startup cancellation boundary is invalid")
    try:
        client = JsonMainControllerClient(serial_port)
    except BaseException as operation_error:
        cleanup_error = close_unowned_serial_port(serial_port)
        if cleanup_error is not None:
            raise JsonMainControllerStartupCleanupError(operation_error, cleanup_error) from cleanup_error
        raise
    position = None
    try:
        if _cancellation_requested(cancellation_boundary):
            raise TimeoutError("controller startup cancelled")
        hello_ticket = _submit_startup_request(
            client,
            cancellation_boundary,
            "hello",
            lambda write_admission: client.request_hello(
                timeout=normalized_request_timeout,
                write_admission=write_admission,
            ),
            physical_stop_callback,
            cancellation_drain_timeout,
        )
        hello = _await_completed_terminal(
            client,
            hello_ticket,
            "main-controller JSON hello",
            physical_stop_callback=physical_stop_callback,
            drain_timeout=cancellation_drain_timeout,
        )
        update_ticket = _submit_startup_request(
            client,
            cancellation_boundary,
            "hello",
            lambda write_admission: client.request_update_params(
                **configuration.update_params,
                timeout=normalized_request_timeout,
                write_admission=write_admission,
            ),
            physical_stop_callback,
            cancellation_drain_timeout,
        )
        _await_completed_terminal(
            client,
            update_ticket,
            "main-controller JSON update-params request",
            physical_stop_callback=physical_stop_callback,
            drain_timeout=cancellation_drain_timeout,
        )

        external_ticket = _submit_startup_request(
            client,
            cancellation_boundary,
            "update_params",
            lambda write_admission: client.request_config_ext_axis(
                **configuration.config_ext_axis,
                timeout=normalized_request_timeout,
                write_admission=write_admission,
            ),
            physical_stop_callback,
            cancellation_drain_timeout,
        )
        _await_completed_terminal(
            client,
            external_ticket,
            "main-controller JSON external-axis configuration request",
            physical_stop_callback=physical_stop_callback,
            drain_timeout=cancellation_drain_timeout,
        )

        set_position_ticket = _submit_startup_request(
            client,
            cancellation_boundary,
            "config_ext_axis",
            lambda write_admission: client.request_set_position(
                **configuration.set_position,
                timeout=normalized_request_timeout,
                write_admission=write_admission,
            ),
            physical_stop_callback,
            cancellation_drain_timeout,
        )
        _await_completed_terminal(
            client,
            set_position_ticket,
            "main-controller JSON set-position request",
            physical_stop_callback=physical_stop_callback,
            drain_timeout=cancellation_drain_timeout,
        )

        position_ticket = _submit_startup_request(
            client,
            cancellation_boundary,
            "set_position",
            lambda write_admission: client.request_position_disposition(
                timeout=normalized_request_timeout,
                write_admission=write_admission,
            ),
            physical_stop_callback,
            cancellation_drain_timeout,
        )
        position = _await_completed_terminal(
            client,
            position_ticket,
            "main-controller JSON position request",
            physical_stop_callback=physical_stop_callback,
            drain_timeout=cancellation_drain_timeout,
        )

        home_reference_ticket = _submit_startup_request(
            client,
            cancellation_boundary,
            "get_position_disposition",
            lambda write_admission: client.request_home_reference(
                timeout=normalized_request_timeout,
                write_admission=write_admission,
            ),
            physical_stop_callback,
            cancellation_drain_timeout,
        )
        home_reference = _await_completed_terminal(
            client,
            home_reference_ticket,
            "main-controller JSON home-reference request",
            physical_stop_callback=physical_stop_callback,
            drain_timeout=cancellation_drain_timeout,
        )
        final_command = "get_home_reference"
        write_reservation = getattr(
            cancellation_boundary,
            "write_reservation",
            None,
        )
        if not callable(write_reservation):
            raise TypeError(
                "controller startup cancellation boundary lacks serial ordering"
            )
        final_reservation = write_reservation()
        if (
            not callable(getattr(final_reservation, "__enter__", None))
            or not callable(getattr(final_reservation, "__exit__", None))
        ):
            raise TypeError(
                "controller startup final ownership reservation is invalid"
            )
        drain_main_controller_input(
            client,
            final_command,
            physical_stop_callback,
            cancellation_drain_timeout,
        )
        with final_reservation:
            cancelled = _cancellation_requested(cancellation_boundary)
            if not cancelled:
                if client.release_reader() is not True:
                    raise JsonMainControllerStartupError(
                        "persistent startup JSON reader release was not "
                        "confirmed"
                    )
                startup = JsonMainControllerStartupResult(
                    hello,
                    position,
                    home_reference,
                )
                persistent_result = JsonMainControllerPersistentStartupResult(
                    startup,
                    client,
                    configuration,
                )
        if cancelled:
            drain_main_controller_input(
                client,
                final_command,
                physical_stop_callback,
                cancellation_drain_timeout,
            )
            raise TimeoutError("controller startup cancelled")
        return persistent_result
    except BaseException as operation_error:
        retained_error = (
            retain_main_controller_startup_disposition_error(
                position,
                operation_error,
                "persistent main-controller JSON startup failed",
            )
            if type(position) is JsonMainPositionResult
            else operation_error
        )
        try:
            client.close(timeout=normalized_close_timeout)
        except BaseException as cleanup_error:
            raise JsonMainControllerStartupCleanupError(
                retained_error,
                cleanup_error,
                client,
            ) from cleanup_error
        if retained_error is operation_error:
            raise
        raise retained_error from operation_error


__all__ = (
    "JsonMainControllerPersistentStartupResult",
    "JsonMainControllerPhysicalStop",
    "JsonMainControllerPhysicalStopError",
    "JsonMainControllerPhysicalStopPublicationError",
    "JsonMainControllerStartupCleanupError",
    "JsonMainControllerStartupConfiguration",
    "JsonMainControllerStartupError",
    "JsonMainControllerStartupResult",
    "await_main_controller_terminal",
    "controller_identity_from_json_hello",
    "drain_main_controller_input",
    "retain_main_controller_startup_disposition_error",
    "run_main_controller_json_startup",
)
