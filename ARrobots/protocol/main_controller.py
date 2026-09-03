"""Device-bound client for the main-controller JSON session."""

from dataclasses import asdict, dataclass, field
from functools import partial
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time

from .catalog import MAIN_CONTROLLER
from .coordinator import (
    JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
    JSON_COORDINATOR_MAXIMUM_DELIVERIES,
    JsonSerialSessionCoordinator,
)
from .messages import JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES, Response
from .schemas import (
    JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS,
    JSON_LIVE_MOTION_LEASE_MINIMUM_MILLISECONDS,
    JSON_MAIN_FIRMWARE_FRAME_RECEIVE_TIMEOUT_SECONDS,
    JSON_MAIN_COMMAND_MANIFEST,
    MAIN_CALIBRATE_COMMAND_CONTRACT,
    MAIN_CONFIG_EXT_AXIS_COMMAND_CONTRACT,
    MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT,
    MAIN_CORRECT_POSITION_COMMAND_CONTRACT,
    MAIN_DELETE_SD_PROGRAM_COMMAND_CONTRACT,
    MAIN_GET_HOME_REFERENCE_COMMAND_CONTRACT,
    MAIN_GET_MOTION_TRACE_COMMAND_CONTRACT,
    MAIN_GET_POSITION_DISPOSITION_COMMAND_CONTRACT,
    MAIN_HELLO_COMMAND_CONTRACT,
    MAIN_READ_ENCODERS_COMMAND_CONTRACT,
    MAIN_SET_ENCODERS_COMMAND_CONTRACT,
    MAIN_TEST_LIMIT_SWITCHES_COMMAND_CONTRACT,
    MAIN_JOG_TOOL_COMMAND_CONTRACT,
    MAIN_LIVE_CART_JOG_COMMAND_CONTRACT,
    MAIN_LIVE_JOINT_JOG_COMMAND_CONTRACT,
    MAIN_LIVE_TOOL_JOG_COMMAND_CONTRACT,
    MAIN_LIST_SD_PROGRAMS_COMMAND_CONTRACT,
    MAIN_MOVE_ARC_COMMAND_CONTRACT,
    MAIN_MOVE_CARTESIAN_COMMAND_CONTRACT,
    MAIN_MOVE_CIRCLE_COMMAND_CONTRACT,
    MAIN_MOVE_JOINTS_COMMAND_CONTRACT,
    MAIN_MOVE_LINEAR_COMMAND_CONTRACT,
    MAIN_MOVE_SPLINE_COMMAND_CONTRACT,
    MAIN_MOVE_VISION_COMMAND_CONTRACT,
    MAIN_MODBUS_READ_COIL_COMMAND_CONTRACT,
    MAIN_MODBUS_READ_DISCRETE_INPUT_COMMAND_CONTRACT,
    MAIN_MODBUS_READ_HOLDING_REGISTER_COMMAND_CONTRACT,
    MAIN_MODBUS_READ_INPUT_REGISTER_COMMAND_CONTRACT,
    MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT,
    MAIN_MODBUS_WRITE_REGISTER_COMMAND_CONTRACT,
    MAIN_PLAY_GCODE_FILE_COMMAND_CONTRACT,
    MAIN_RENEW_LIVE_MOTION_COMMAND_CONTRACT,
    MAIN_SET_POSITION_COMMAND_CONTRACT,
    MAIN_STOP_COMMAND_CONTRACT,
    MAIN_UPDATE_PARAMS_COMMAND_CONTRACT,
    MAIN_WAIT_MODBUS_COIL_COMMAND_CONTRACT,
    MAIN_WAIT_MODBUS_DISCRETE_INPUT_COMMAND_CONTRACT,
    MAIN_WAIT_MODBUS_HOLDING_REGISTER_COMMAND_CONTRACT,
    MAIN_WRITE_GCODE_MOVE_COMMAND_CONTRACT,
    MAIN_ZERO_J7_COMMAND_CONTRACT,
    MAIN_ZERO_J8_COMMAND_CONTRACT,
    MAIN_ZERO_J9_COMMAND_CONTRACT,
    JsonCommandSchemaError,
    JsonMainMotionTracePageResult,
    parse_main_hello_result,
    parse_main_calibration_result,
    parse_main_encoder_counts_result,
    parse_main_home_reference_result,
    parse_main_limit_switches_result,
    parse_main_live_jog_result,
    parse_main_list_sd_programs_result,
    parse_main_move_cartesian_result,
    parse_main_move_joints_result,
    parse_main_motion_position_result,
    parse_main_motion_trace_result,
    parse_main_modbus_read_result,
    parse_main_modbus_wait_result,
    parse_main_delete_sd_program_result,
    parse_main_position_result,
    parse_main_position_correction_result,
    parse_main_position_disposition_result,
    parse_main_renew_live_motion_result,
    parse_main_stop_result,
    parse_main_tool_jog_result,
    validate_main_configuration_fingerprint,
)
from .session import (
    JSON_SESSION_MAXIMUM_PENDING_REQUESTS,
    JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS,
    JsonSessionAdmissionError,
)
from .transport import (
    JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS,
    JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS,
    JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS,
    JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES,
    JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS,
)


def _parse_empty_result(result):
    return None


_MAIN_MODBUS_READ_CONTRACTS = (
    MAIN_MODBUS_READ_HOLDING_REGISTER_COMMAND_CONTRACT,
    MAIN_MODBUS_READ_COIL_COMMAND_CONTRACT,
    MAIN_MODBUS_READ_DISCRETE_INPUT_COMMAND_CONTRACT,
    MAIN_MODBUS_READ_INPUT_REGISTER_COMMAND_CONTRACT,
)
_MAIN_MODBUS_READ_COMMANDS = frozenset(
    contract.name for contract in _MAIN_MODBUS_READ_CONTRACTS
)
_MAIN_MODBUS_WRITE_CONTRACTS = (
    MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT,
    MAIN_MODBUS_WRITE_REGISTER_COMMAND_CONTRACT,
)
_MAIN_MODBUS_WRITE_COMMANDS = frozenset(
    contract.name for contract in _MAIN_MODBUS_WRITE_CONTRACTS
)


_MAIN_COMMANDS = (
    (
        MAIN_HELLO_COMMAND_CONTRACT,
        parse_main_hello_result,
    ),
    (
        MAIN_GET_HOME_REFERENCE_COMMAND_CONTRACT,
        parse_main_home_reference_result,
    ),
    (
        MAIN_GET_MOTION_TRACE_COMMAND_CONTRACT,
        parse_main_motion_trace_result,
    ),
    (
        MAIN_GET_POSITION_DISPOSITION_COMMAND_CONTRACT,
        parse_main_position_disposition_result,
    ),
    (
        MAIN_CORRECT_POSITION_COMMAND_CONTRACT,
        parse_main_position_correction_result,
    ),
    (
        MAIN_TEST_LIMIT_SWITCHES_COMMAND_CONTRACT,
        parse_main_limit_switches_result,
    ),
    (
        MAIN_SET_ENCODERS_COMMAND_CONTRACT,
        _parse_empty_result,
    ),
    (
        MAIN_READ_ENCODERS_COMMAND_CONTRACT,
        parse_main_encoder_counts_result,
    ),
    (
        MAIN_SET_POSITION_COMMAND_CONTRACT,
        _parse_empty_result,
    ),
    (
        MAIN_UPDATE_PARAMS_COMMAND_CONTRACT,
        _parse_empty_result,
    ),
    (
        MAIN_CONFIG_EXT_AXIS_COMMAND_CONTRACT,
        _parse_empty_result,
    ),
    (MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT, _parse_empty_result),
    *((
        contract,
        partial(parse_main_modbus_read_result, command=contract.name),
    ) for contract in _MAIN_MODBUS_READ_CONTRACTS),
    *((
        contract,
        _parse_empty_result,
    ) for contract in _MAIN_MODBUS_WRITE_CONTRACTS),
    (MAIN_ZERO_J7_COMMAND_CONTRACT, parse_main_position_disposition_result),
    (MAIN_ZERO_J8_COMMAND_CONTRACT, parse_main_position_disposition_result),
    (MAIN_ZERO_J9_COMMAND_CONTRACT, parse_main_position_disposition_result),
    (
        MAIN_CALIBRATE_COMMAND_CONTRACT,
        parse_main_calibration_result,
    ),
    (
        MAIN_JOG_TOOL_COMMAND_CONTRACT,
        parse_main_tool_jog_result,
    ),
    (
        MAIN_LIVE_JOINT_JOG_COMMAND_CONTRACT,
        parse_main_live_jog_result,
    ),
    (
        MAIN_LIVE_CART_JOG_COMMAND_CONTRACT,
        parse_main_live_jog_result,
    ),
    (
        MAIN_LIVE_TOOL_JOG_COMMAND_CONTRACT,
        parse_main_live_jog_result,
    ),
    (
        MAIN_MOVE_CARTESIAN_COMMAND_CONTRACT,
        parse_main_move_cartesian_result,
    ),
    (
        MAIN_MOVE_JOINTS_COMMAND_CONTRACT,
        parse_main_move_joints_result,
    ),
    (
        MAIN_MOVE_LINEAR_COMMAND_CONTRACT,
        parse_main_move_cartesian_result,
    ),
    (
        MAIN_MOVE_VISION_COMMAND_CONTRACT,
        parse_main_move_cartesian_result,
    ),
    (
        MAIN_STOP_COMMAND_CONTRACT,
        parse_main_stop_result,
    ),
    (
        MAIN_RENEW_LIVE_MOTION_COMMAND_CONTRACT,
        parse_main_renew_live_motion_result,
    ),
    (
        MAIN_WAIT_MODBUS_COIL_COMMAND_CONTRACT,
        partial(parse_main_modbus_wait_result, command="wait_modbus_coil"),
    ),
    (
        MAIN_WAIT_MODBUS_DISCRETE_INPUT_COMMAND_CONTRACT,
        partial(
            parse_main_modbus_wait_result,
            command="wait_modbus_discrete_input",
        ),
    ),
    (
        MAIN_DELETE_SD_PROGRAM_COMMAND_CONTRACT,
        parse_main_delete_sd_program_result,
    ),
    (
        MAIN_LIST_SD_PROGRAMS_COMMAND_CONTRACT,
        parse_main_list_sd_programs_result,
    ),
    (MAIN_WRITE_GCODE_MOVE_COMMAND_CONTRACT, parse_main_position_result),
    (MAIN_PLAY_GCODE_FILE_COMMAND_CONTRACT, parse_main_move_cartesian_result),
    (
        MAIN_WAIT_MODBUS_HOLDING_REGISTER_COMMAND_CONTRACT,
        partial(
            parse_main_modbus_wait_result,
            command="wait_modbus_holding_register",
        ),
    ),
    (MAIN_MOVE_ARC_COMMAND_CONTRACT, parse_main_move_cartesian_result),
    (MAIN_MOVE_CIRCLE_COMMAND_CONTRACT, parse_main_move_cartesian_result),
    (MAIN_MOVE_SPLINE_COMMAND_CONTRACT, parse_main_move_cartesian_result),
)

_MAIN_COMMANDS_BY_NAME = {
    contract.name: (contract, parser)
    for contract, parser in _MAIN_COMMANDS
}
if (
    len(_MAIN_COMMANDS_BY_NAME) != len(_MAIN_COMMANDS)
    or len(_MAIN_COMMANDS_BY_NAME) != len(JSON_MAIN_COMMAND_MANIFEST)
    or frozenset(_MAIN_COMMANDS_BY_NAME) != frozenset(JSON_MAIN_COMMAND_MANIFEST)
):
    raise RuntimeError("main JSON command registry does not match the manifest")
MAIN_JSON_COMMAND_CONTRACTS = tuple(
    _MAIN_COMMANDS_BY_NAME[command][0]
    for command in JSON_MAIN_COMMAND_MANIFEST
)

_MAIN_LIVE_JOG_COMMANDS = frozenset(
    ("live_joint_jog", "live_cart_jog", "live_tool_jog")
)
_MAIN_LIVE_CONTROL_COMMANDS = frozenset(("renew_live_motion", "stop"))
_MAIN_DIAGNOSTIC_COMMANDS = frozenset((
    "read_encoders", "set_encoders", "test_limit_switches"))
_MAIN_EXTERNAL_AXIS_ZERO_COMMANDS = frozenset(("zero_j7", "zero_j8", "zero_j9"))
_MAIN_BLOCKING_COMMANDS = frozenset((
    "controller_wait",
    "get_motion_trace",
    *_MAIN_MODBUS_READ_COMMANDS,
    *_MAIN_MODBUS_WRITE_COMMANDS,
    "wait_modbus_coil",
    "wait_modbus_discrete_input",
    "wait_modbus_holding_register",
    "delete_sd_program",
    "list_sd_programs",
    "write_gcode_move",
))
_MAIN_RETAINED_MOTION_COMMANDS = frozenset((
    "calibrate",
    "jog_tool",
    "live_cart_jog",
    "live_joint_jog",
    "live_tool_jog",
    "move_arc",
    "move_cartesian",
    "move_circle",
    "move_joints",
    "move_linear",
    "move_spline",
    "move_vision",
    "play_gcode_file",
))
_MAIN_GENERIC_COMMANDS = frozenset((
    "delete_sd_program",
    "list_sd_programs",
    "move_arc",
    "move_cartesian",
    "move_circle",
    "move_linear",
    "move_spline",
    "move_vision",
    "play_gcode_file",
    "wait_modbus_coil",
    "wait_modbus_discrete_input",
    "wait_modbus_holding_register",
    "write_gcode_move",
))
_MAIN_INDETERMINATE_COMMANDS = frozenset((
    *_MAIN_MODBUS_WRITE_COMMANDS,
    "delete_sd_program",
    "write_gcode_move",
))


def _validate_live_control_timeout(
    timeout,
    lease_milliseconds,
    frame_timeout,
):
    complete_frame_bound = max(
        frame_timeout,
        JSON_MAIN_FIRMWARE_FRAME_RECEIVE_TIMEOUT_SECONDS,
    ) if (
        type(frame_timeout) in (int, float)
        and math.isfinite(frame_timeout)
        and frame_timeout > 0
    ) else None
    if (
        type(timeout) not in (int, float)
        or not math.isfinite(timeout)
        or type(lease_milliseconds) is not int
        or lease_milliseconds
            < JSON_LIVE_MOTION_LEASE_MINIMUM_MILLISECONDS
        or lease_milliseconds
            > JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS
        or type(frame_timeout) not in (int, float)
        or not math.isfinite(frame_timeout)
        or frame_timeout <= 0
        or complete_frame_bound is None
        or timeout
            <= lease_milliseconds / 1000.0
                + 2.0 * complete_frame_bound
    ):
        raise JsonSessionAdmissionError(
            "live-motion control timeout must exceed the controller lease "
            "and two complete-frame response bounds"
        )


@dataclass(frozen=True)
class JsonMainControllerTerminal:
    """One validated terminal response and an optional typed success value."""

    response: Response
    parsed_result: object = field(init=False)

    def __init__(self, response):
        if (
            type(response) is not Response
            or not response.terminal
            or response.cmd not in _MAIN_COMMANDS_BY_NAME
        ):
            raise JsonCommandSchemaError(
                "main-controller terminal response is invalid"
            )
        contract, parser = _MAIN_COMMANDS_BY_NAME[response.cmd]
        contract.response_validator(response)
        parsed_result = (
            parser(response.result)
            if response.status == "completed"
            else None
        )
        object.__setattr__(self, "response", response)
        object.__setattr__(self, "parsed_result", parsed_result)

    @property
    def failure(self):
        return self.response.error


@dataclass(frozen=True)
class _JsonMainHandshakeState:
    ticket: object = None
    terminal: object = None
    binding: object = None


@dataclass(frozen=True)
class _JsonMainConfigurationState:
    ticket: object = None
    terminal: object = None
    synchronization_required: bool = False


@dataclass(frozen=True)
class _JsonMainMotionState:
    ticket: object = None
    control_ticket: object = None
    motion_request_id: int = 0
    lease_milliseconds: int = 0
    terminal_order_faulted: bool = False


@dataclass(frozen=True)
class JsonMainMotionTraceReservation:
    generation: int
    configuration_fingerprint: str
    armed_at_ns: int


class JsonMainMotionTraceArm:
    """Own the one-shot admission state for a manual joint trace."""

    def __init__(self):
        self._lock = threading.Lock()
        self._generation = 0
        self._armed = False
        self._armed_at_ns = 0

    @property
    def armed(self):
        with self._lock:
            return self._armed

    def arm(self):
        with self._lock:
            self._generation += 1
            self._armed = True
            self._armed_at_ns = time.time_ns()
            return self._generation

    def cancel(self):
        with self._lock:
            self._armed = False
            self._armed_at_ns = 0

    def reserve(self, configuration_fingerprint):
        validate_main_configuration_fingerprint(
            configuration_fingerprint,
            "motion-trace configuration fingerprint",
        )
        with self._lock:
            if not self._armed:
                return None
            return JsonMainMotionTraceReservation(
                self._generation,
                configuration_fingerprint,
                self._armed_at_ns,
            )

    def consume(self, reservation):
        if type(reservation) is not JsonMainMotionTraceReservation:
            return False
        with self._lock:
            if (
                not self._armed
                or reservation.generation != self._generation
            ):
                return False
            self._armed = False
            self._armed_at_ns = 0
            return True


class JsonMainMotionTraceAssembly:
    """Validate and assemble one immutable paged controller trace."""

    def __init__(
        self,
        *,
        motion_request_id,
        source_session_id,
        configuration_fingerprint,
    ):
        if (
            isinstance(motion_request_id, bool)
            or not isinstance(motion_request_id, int)
            or motion_request_id <= 0
        ):
            raise JsonCommandSchemaError(
                "motion-trace source request identifier is invalid"
            )
        if not isinstance(source_session_id, str) or not source_session_id:
            raise JsonCommandSchemaError(
                "motion-trace source session identifier is invalid"
            )
        validate_main_configuration_fingerprint(
            configuration_fingerprint,
            "motion-trace configuration fingerprint",
        )
        self._motion_request_id = motion_request_id
        self._source_session_id = source_session_id
        self._configuration_fingerprint = configuration_fingerprint
        self._identity = None
        self._records = []
        self._next_page = 0
        self._complete = False

    def accept(self, page):
        if self._complete:
            raise JsonCommandSchemaError(
                "motion-trace assembly already contains every page"
            )
        if type(page) is not JsonMainMotionTracePageResult:
            raise JsonCommandSchemaError(
                "motion-trace assembly requires an available page"
            )
        if (
            page.source_motion_request_id != self._motion_request_id
            or page.source_session_id != self._source_session_id
            or page.configuration_fingerprint
            != self._configuration_fingerprint
        ):
            raise JsonCommandSchemaError(
                "motion-trace page does not match the requested motion"
            )
        identity = (
            page.capture_generation,
            page.configuration_fingerprint,
            page.disposition,
            page.firmware,
            page.page_count,
            page.source_motion_request_id,
            page.source_session_id,
            page.total_records,
        )
        if self._identity is None:
            self._identity = identity
        elif identity != self._identity:
            raise JsonCommandSchemaError(
                "motion-trace page identity changed during assembly"
            )
        if (
            page.page_index != self._next_page
            or page.record_start != len(self._records)
        ):
            raise JsonCommandSchemaError(
                "motion-trace page sequence is discontinuous"
            )
        self._records.extend(page.records)
        self._next_page += 1
        if self._next_page == page.page_count:
            if len(self._records) != page.total_records:
                raise JsonCommandSchemaError(
                    "motion-trace record total changed during assembly"
                )
            self._complete = True
        return self._complete

    def artifact(self, *, controller_identity, motion_parameters, host_times_ns):
        if not self._complete or self._identity is None:
            raise JsonCommandSchemaError(
                "motion-trace artifact requires a complete assembly"
            )
        if type(controller_identity) is not dict or not controller_identity:
            raise JsonCommandSchemaError(
                "motion-trace controller identity is invalid"
            )
        if type(motion_parameters) is not dict or not motion_parameters:
            raise JsonCommandSchemaError(
                "motion-trace motion parameters are invalid"
            )
        required_times = ("armed", "admitted", "terminal", "retrieved")
        timestamp_values = tuple(
            host_times_ns.get(name) for name in required_times
        ) if type(host_times_ns) is dict else ()
        if (
            type(host_times_ns) is not dict
            or frozenset(host_times_ns) != frozenset(required_times)
            or any(
                isinstance(value, bool)
                or not isinstance(value, int)
                or value <= 0
                for value in timestamp_values
            )
            or any(
                earlier > later
                for earlier, later in zip(
                    timestamp_values,
                    timestamp_values[1:],
                )
            )
        ):
            raise JsonCommandSchemaError(
                "motion-trace host timestamps are invalid"
            )
        (
            capture_generation,
            configuration_fingerprint,
            disposition,
            firmware,
            _page_count,
            motion_request_id,
            source_session_id,
            _total_records,
        ) = self._identity
        if (
            motion_parameters.get("trace_configuration_fingerprint")
            != configuration_fingerprint
            or motion_parameters.get("telemetry_enabled") is not False
        ):
            raise JsonCommandSchemaError(
                "motion-trace motion parameters do not select this capture"
            )
        return {
            "artifact_version": 1,
            "capture_generation": capture_generation,
            "configuration_fingerprint": configuration_fingerprint,
            "controller_identity": dict(controller_identity),
            "disposition": asdict(disposition),
            "firmware": asdict(firmware),
            "host_times_ns": dict(host_times_ns),
            "motion_request": {
                "cmd": "move_joints",
                "id": motion_request_id,
                "params": dict(motion_parameters),
            },
            "records": [asdict(record) for record in self._records],
            "source_session_id": source_session_id,
        }


def write_main_motion_trace_artifact(directory, artifact):
    """Atomically publish one validated local trace artifact."""
    if type(artifact) is not dict or artifact.get("artifact_version") != 1:
        raise JsonCommandSchemaError("motion-trace artifact is invalid")
    capture_generation = artifact.get("capture_generation")
    motion_request = artifact.get("motion_request")
    source_session_id = artifact.get("source_session_id")
    if (
        isinstance(capture_generation, bool)
        or not isinstance(capture_generation, int)
        or capture_generation <= 0
        or type(motion_request) is not dict
        or isinstance(motion_request.get("id"), bool)
        or not isinstance(motion_request.get("id"), int)
        or motion_request["id"] <= 0
        or not isinstance(source_session_id, str)
        or len(source_session_id) != 32
        or any(
            character not in "0123456789ABCDEF"
            for character in source_session_id
        )
    ):
        raise JsonCommandSchemaError("motion-trace artifact identity is invalid")
    destination_directory = Path(directory)
    destination_directory.mkdir(parents=True, exist_ok=True)
    destination = destination_directory / (
        f"session-{source_session_id}-motion-{motion_request['id']}-"
        f"capture-{capture_generation}.json"
    )
    if destination.exists():
        raise FileExistsError(f"motion-trace artifact already exists: {destination}")
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=f".{destination.name}.",
            suffix=".partial",
            dir=destination_directory,
            delete=False,
        ) as stream:
            temporary_name = stream.name
            json.dump(
                artifact,
                stream,
                ensure_ascii=True,
                separators=(",", ":"),
                sort_keys=True,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
    return destination


def main_motion_trace_retrieval_eligible(terminal, stop_observed):
    """Return whether a settled traced move can safely enter retrieval."""
    if type(terminal) is not JsonMainControllerTerminal:
        raise JsonCommandSchemaError("motion-trace terminal is invalid")
    if type(stop_observed) is not bool:
        raise JsonCommandSchemaError("motion-trace stop state is invalid")
    if stop_observed:
        return False
    if terminal.response.status == "completed":
        return True
    if terminal.response.status != "failed" or terminal.failure is None:
        return False
    position = terminal.failure.details.get("position")
    if position is None:
        return False
    try:
        parse_main_motion_position_result(position)
    except JsonCommandSchemaError:
        return False
    return True


class JsonMainControllerClientStateError(RuntimeError):
    """Main-controller handshake state rejects a semantic operation."""


class _JsonMainSubmissionReservation:
    """Retain one client submission across lock-free coordinator work."""

    def __init__(
        self,
        owner,
        command,
        motion_ticket=None,
        control_timeout=None,
    ):
        self._owner = owner
        self.command = command
        self.motion_ticket = motion_ticket
        self.control_timeout = control_timeout
        self.maximum_payload_bytes = None

    def __enter__(self):
        try:
            self._owner._begin_submission(self)
            return self
        except BaseException:
            self._owner._force_end_submission(self)
            raise

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            self._owner._end_submission(self)
        finally:
            self._owner._force_end_submission(self)
        return False


class JsonMainControllerClient:
    """Own an already-open serial handle through exact main JSON contracts.

    Construction performs no request transmission. A completed ``hello``
    terminal must be acknowledged before state requests become eligible.
    Polling remains explicit, allowing an application lifecycle owner to
    run serial work outside the Tk thread without introducing callbacks from
    the polling context.
    """

    def __init__(
        self,
        serial_port,
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
        self._coordinator = JsonSerialSessionCoordinator(
            serial_port,
            MAIN_CONTROLLER,
            MAIN_JSON_COMMAND_CONTRACTS,
            clock=clock,
            clock_resolution=clock_resolution,
            deadline_scheduler=deadline_scheduler,
            sleeper=sleeper,
            poll_interval=poll_interval,
            frame_timeout=frame_timeout,
            write_timeout=write_timeout,
            read_chunk_bytes=read_chunk_bytes,
            drain_poll_interval=drain_poll_interval,
            maximum_pending_requests=maximum_pending_requests,
            maximum_telemetry_streams=maximum_telemetry_streams,
            delivery_capacity=delivery_capacity,
        )
        self._state_lock = threading.RLock()
        self._handshake = _JsonMainHandshakeState()
        self._configuration = _JsonMainConfigurationState()
        self._motion = _JsonMainMotionState()
        self._submission_reservation = None
        self._serial_port = serial_port
        self._maximum_pending_requests = maximum_pending_requests
        self._delivery_capacity = delivery_capacity
        self._frame_timeout = frame_timeout

    def _require_submission_idle_locked(self, operation):
        reservation = self._submission_reservation
        if reservation is None:
            return
        if (
            type(reservation) is not _JsonMainSubmissionReservation
            or reservation._owner is not self
            or reservation.command not in _MAIN_COMMANDS_BY_NAME
        ):
            raise JsonMainControllerClientStateError(
                "main-controller submission reservation is invalid"
            )
        raise JsonMainControllerClientStateError(
            f"main-controller {operation} rejected while request submission "
            "is active"
        )

    def _begin_submission(self, reservation):
        try:
            with self._state_lock:
                if (
                    type(reservation) is not _JsonMainSubmissionReservation
                    or reservation._owner is not self
                    or reservation.command not in _MAIN_COMMANDS_BY_NAME
                    or (
                        (reservation.motion_ticket is not None)
                        != (
                            reservation.command
                            in _MAIN_LIVE_CONTROL_COMMANDS
                        )
                    )
                    or (
                        (reservation.control_timeout is not None)
                        != (
                            reservation.command
                            in _MAIN_LIVE_CONTROL_COMMANDS
                        )
                    )
                ):
                    raise JsonMainControllerClientStateError(
                        "main-controller submission reservation is invalid"
                    )
                self._recover_handshake_locked()
                self._recover_configuration_locked()
                self._recover_motion_locked()
                self._require_submission_idle_locked("submission")
                command = reservation.command
                retained_blocking = tuple(
                    ticket
                    for ticket in self._coordinator.pending_tickets
                    if ticket.command in _MAIN_BLOCKING_COMMANDS
                )
                if retained_blocking:
                    raise JsonMainControllerClientStateError(
                        "main-controller JSON blocking request is already "
                        f"pending: {retained_blocking[0].command}"
                    )
                if command == "hello":
                    if self._handshake.binding is not None:
                        raise JsonMainControllerClientStateError(
                            "main-controller JSON session is already established"
                        )
                    if self._handshake.ticket is not None:
                        raise JsonMainControllerClientStateError(
                            "main-controller hello request is already pending"
                        )
                else:
                    if self._handshake.binding is None:
                        raise JsonMainControllerClientStateError(
                            "main-controller JSON session is not established"
                        )
                    if command in _MAIN_LIVE_JOG_COMMANDS and (
                        self._maximum_pending_requests < 2
                        or self._delivery_capacity < 2
                    ):
                        raise JsonMainControllerClientStateError(
                            "main-controller JSON live motion requires "
                            "capacity for paired request ownership"
                        )
                    live_ticket = self._motion.ticket
                    control_ticket = self._motion.control_ticket
                    live_active = (
                        live_ticket is not None
                        and live_ticket.command in _MAIN_LIVE_JOG_COMMANDS
                    )
                    if command in _MAIN_LIVE_CONTROL_COMMANDS:
                        if (
                            not live_active
                            or reservation.motion_ticket is not live_ticket
                            or self._motion.motion_request_id
                            != live_ticket.request_id
                        ):
                            raise JsonMainControllerClientStateError(
                                "main-controller JSON live motion is not "
                                "active"
                            )
                        if control_ticket is not None:
                            raise JsonMainControllerClientStateError(
                                "main-controller JSON live-motion control "
                                "request is "
                                "already pending"
                            )
                        pending_tickets = (
                            self._coordinator.pending_tickets
                        )
                        if (
                            len(pending_tickets) != 1
                            or pending_tickets[0] is not live_ticket
                            or self._coordinator.delivery_count != 0
                        ):
                            raise JsonMainControllerClientStateError(
                                "main-controller JSON live-motion control "
                                "requires drained response deliveries"
                            )
                        snapshot = self._coordinator.snapshot(live_ticket)
                        if (
                            snapshot.accepted is None
                            or snapshot.terminal is not None
                        ):
                            raise JsonMainControllerClientStateError(
                                "main-controller JSON live motion is not "
                                "accepted"
                            )
                        _validate_live_control_timeout(
                            reservation.control_timeout,
                            self._motion.lease_milliseconds,
                            self._frame_timeout,
                        )
                    elif live_active or control_ticket is not None:
                        raise JsonMainControllerClientStateError(
                            "main-controller JSON live motion is active"
                        )
                    if self._configuration.ticket is not None:
                        raise JsonMainControllerClientStateError(
                            "main-controller configuration transition is "
                            "already pending"
                        )
                    if (
                        command in _MAIN_BLOCKING_COMMANDS
                        and (
                            self._coordinator.pending_tickets
                            or self._coordinator.delivery_count != 0
                        )
                    ):
                        raise JsonMainControllerClientStateError(
                            "main-controller blocking request requires drained "
                            "requests and deliveries"
                        )
                    if (
                        command in (
                            "update_params",
                            "config_ext_axis",
                            "correct_position",
                            "set_position",
                            "set_encoders",
                            *_MAIN_EXTERNAL_AXIS_ZERO_COMMANDS,
                        )
                        and (
                            self._coordinator.pending_tickets
                            or self._coordinator.delivery_count != 0
                        )
                    ):
                        raise JsonMainControllerClientStateError(
                            "main-controller configuration transition "
                            "requires drained requests and deliveries"
                        )
                    if command in _MAIN_RETAINED_MOTION_COMMANDS and (
                        self._coordinator.pending_tickets
                        or self._coordinator.delivery_count != 0
                    ):
                        raise JsonMainControllerClientStateError(
                            f"main-controller {command} requires "
                            "drained requests and deliveries"
                        )
                    if (
                        self._configuration.synchronization_required
                        and command not in (
                            "update_params",
                            "config_ext_axis",
                            "set_position",
                        )
                    ):
                        raise JsonMainControllerClientStateError(
                            "main-controller configuration requires "
                            "set-position synchronization"
                        )
                reservation.maximum_payload_bytes = (
                    JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES
                    if command == "hello"
                    else self._handshake.binding.protocol.maximum_payload_bytes
                )
                self._submission_reservation = reservation
                return True
        except BaseException:
            self._force_end_submission(reservation)
            raise

    def _end_submission(self, reservation):
        with self._state_lock:
            if self._submission_reservation is not reservation:
                raise JsonMainControllerClientStateError(
                    "main-controller submission reservation changed"
                )
            self._submission_reservation = None

    def _force_end_submission(self, reservation):
        with self._state_lock:
            if self._submission_reservation is reservation:
                self._submission_reservation = None

    def _pending_tickets_for_command(self, command):
        return tuple(
            ticket
            for ticket in self._coordinator.pending_tickets
            if ticket.command == command
        )

    @property
    def serial_port(self):
        """Return the exact serial handle associated with the coordinator."""
        return self._serial_port

    @property
    def session_binding(self):
        with self._state_lock:
            self._recover_handshake_locked()
            self._recover_configuration_locked()
            self._recover_motion_locked()
            return self._handshake.binding

    @property
    def session_ready(self):
        with self._state_lock:
            self._recover_handshake_locked()
            self._recover_configuration_locked()
            return (
                self._handshake.binding is not None
                and self._submission_reservation is None
                and self._configuration.ticket is None
                and not self._configuration.synchronization_required
            )

    @property
    def configuration_sync_required(self):
        with self._state_lock:
            self._recover_configuration_locked()
            return self._configuration.synchronization_required

    @property
    def quarantined(self):
        return self._coordinator.quarantined

    @property
    def quarantine_reason(self):
        return self._coordinator.quarantine_reason

    @property
    def closed(self):
        return self._coordinator.closed

    @property
    def closing(self):
        return self._coordinator.closing

    @property
    def delivery_count(self):
        return self._coordinator.delivery_count

    @property
    def pending_tickets(self):
        return self._coordinator.pending_tickets

    @property
    def pending_motion_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            return self._motion.ticket

    @property
    def pending_joint_motion_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.ticket
            return (
                ticket
                if ticket is not None and ticket.command == "move_joints"
                else None
            )

    @property
    def pending_cartesian_motion_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.ticket
            return (
                ticket
                if ticket is not None
                and ticket.command == "move_cartesian"
                else None
            )

    @property
    def pending_tool_jog_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.ticket
            return (
                ticket
                if ticket is not None and ticket.command == "jog_tool"
                else None
            )

    @property
    def pending_live_joint_jog_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.ticket
            return (
                ticket
                if ticket is not None
                and ticket.command == "live_joint_jog"
                else None
            )

    @property
    def pending_live_cart_jog_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.ticket
            return (
                ticket
                if ticket is not None
                and ticket.command == "live_cart_jog"
                else None
            )

    @property
    def pending_live_tool_jog_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.ticket
            return (
                ticket
                if ticket is not None
                and ticket.command == "live_tool_jog"
                else None
            )

    @property
    def pending_live_stop_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.control_ticket
            return (
                ticket
                if ticket is not None and ticket.command == "stop"
                else None
            )

    @property
    def pending_live_renewal_ticket(self):
        with self._state_lock:
            self._recover_motion_locked()
            ticket = self._motion.control_ticket
            return (
                ticket
                if ticket is not None
                and ticket.command == "renew_live_motion"
                else None
            )

    @property
    def deadline_cleanup_count(self):
        return self._coordinator.deadline_cleanup_count

    @property
    def has_unread_input(self):
        return self._coordinator.has_unread_input

    @property
    def reader_owner(self):
        return self._coordinator.reader_owner

    def request_hello(self, *, timeout, write_admission=None):
        """Submit the first successful session identity request.

        ``write_admission`` is a bounded pre-I/O callable invoked outside the
        client state lock. Exact ``True`` admits transmission; another value,
        an exception, deadline expiry, or a concurrent client submission
        prevents transmission.
        """
        with _JsonMainSubmissionReservation(self, "hello") as reservation:
            try:
                ticket = self._coordinator.submit(
                    "hello",
                    {},
                    timeout=timeout,
                    write_admission=write_admission,
                    maximum_payload_bytes=(
                        reservation.maximum_payload_bytes
                    ),
                )
                with self._state_lock:
                    self._handshake = _JsonMainHandshakeState(ticket=ticket)
            except BaseException:
                recovered = self._pending_tickets_for_command("hello")
                if len(recovered) == 1:
                    with self._state_lock:
                        self._handshake = _JsonMainHandshakeState(
                            ticket=recovered[0]
                        )
                raise
            return ticket

    def request_controller_wait(
        self,
        seconds,
        *,
        timeout,
        write_admission=None,
    ):
        """Submit one correlated controller wait."""
        with _JsonMainSubmissionReservation(
            self,
            "controller_wait",
        ) as reservation:
            return self._coordinator.submit(
                "controller_wait",
                {"seconds": seconds},
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def request_modbus_read(
        self,
        command,
        *,
        slave_id,
        address,
        timeout,
        write_admission=None,
    ):
        """Submit one correlated scalar Modbus read."""
        if type(command) is not str or command not in _MAIN_MODBUS_READ_COMMANDS:
            raise JsonCommandSchemaError(
                "main-controller JSON Modbus-read command is invalid"
            )
        with _JsonMainSubmissionReservation(self, command) as reservation:
            return self._coordinator.submit(
                command,
                {
                    "slave_id": slave_id,
                    "address": address,
                    "count": 1,
                },
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def request_modbus_write(
        self,
        command,
        *,
        slave_id,
        address,
        value,
        timeout,
        write_admission=None,
    ):
        """Submit one correlated immediate Modbus write."""
        if type(command) is not str or command not in _MAIN_MODBUS_WRITE_COMMANDS:
            raise JsonCommandSchemaError(
                "main-controller JSON Modbus-write command is invalid"
            )
        with _JsonMainSubmissionReservation(self, command) as reservation:
            return self._coordinator.submit(
                command,
                {
                    "slave_id": slave_id,
                    "address": address,
                    "value": value,
                },
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def request_command(
        self,
        command,
        params,
        *,
        timeout,
        write_admission=None,
    ):
        """Submit one active semantic command through centralized metadata."""
        if type(command) is not str or command not in _MAIN_GENERIC_COMMANDS:
            raise JsonCommandSchemaError(
                "main-controller semantic command is not generic"
            )
        if command in _MAIN_RETAINED_MOTION_COMMANDS:
            return self._submit_motion(
                command,
                params,
                timeout=timeout,
                write_admission=write_admission,
            )
        with _JsonMainSubmissionReservation(self, command) as reservation:
            return self._coordinator.submit(
                command,
                params,
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def request_position_disposition(
        self,
        *,
        timeout,
        write_admission=None,
    ):
        """Submit a capability-gated position-disposition request."""
        with _JsonMainSubmissionReservation(
            self,
            "get_position_disposition",
        ) as reservation:
            return self._coordinator.submit(
                "get_position_disposition",
                {},
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def request_home_reference(self, *, timeout, write_admission=None):
        """Submit a read-only J1-J3 home-reference request."""
        with _JsonMainSubmissionReservation(
            self,
            "get_home_reference",
        ) as reservation:
            return self._coordinator.submit(
                "get_home_reference",
                {},
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def _request_diagnostic(self, command, *, timeout, write_admission):
        with _JsonMainSubmissionReservation(self, command) as reservation:
            return self._coordinator.submit(
                command,
                {},
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def request_test_limit_switches(
        self, *, timeout, write_admission=None
    ):
        return self._request_diagnostic(
            "test_limit_switches",
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_read_encoders(self, *, timeout, write_admission=None):
        return self._request_diagnostic(
            "read_encoders",
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_motion_trace(
        self,
        *,
        motion_request_id,
        page_index,
        timeout,
        write_admission=None,
    ):
        """Request one immutable controller-clock trace page."""
        with _JsonMainSubmissionReservation(
            self,
            "get_motion_trace",
        ) as reservation:
            return self._coordinator.submit(
                "get_motion_trace",
                {
                    "motion_request_id": motion_request_id,
                    "page_index": page_index,
                },
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=reservation.maximum_payload_bytes,
            )

    def request_set_encoders(self, *, timeout, write_admission=None):
        return self._request_diagnostic(
            "set_encoders",
            timeout=timeout,
            write_admission=write_admission,
        )

    def _submit_configuration_transition(
        self,
        command,
        params,
        *,
        timeout,
        write_admission,
    ):
        with _JsonMainSubmissionReservation(self, command) as reservation:
            try:
                ticket = self._coordinator.submit(
                    command,
                    params,
                    timeout=timeout,
                    write_admission=write_admission,
                    maximum_payload_bytes=reservation.maximum_payload_bytes,
                )
                with self._state_lock:
                    self._configuration = _JsonMainConfigurationState(
                        ticket=ticket,
                        synchronization_required=(
                            self._configuration.synchronization_required
                        ),
                    )
            except BaseException:
                recovered = self._pending_tickets_for_command(command)
                if len(recovered) == 1:
                    with self._state_lock:
                        self._configuration = _JsonMainConfigurationState(
                            ticket=recovered[0],
                            synchronization_required=(
                                self._configuration.synchronization_required
                            ),
                        )
                raise
            return ticket

    def request_set_position(
        self,
        *,
        robot_joints_millidegrees,
        external_axes_milliunits,
        timeout,
        write_admission=None,
    ):
        """Replace controller step state through fixed-point axis values."""
        return self._submit_configuration_transition(
            "set_position",
            {
                "external_axes_milliunits": external_axes_milliunits,
                "robot_joints_millidegrees": robot_joints_millidegrees,
            },
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_correct_position(self, *, timeout, write_admission=None):
        """Reseed primary controller step state from configured encoders."""
        return self._submit_configuration_transition(
            "correct_position",
            {},
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_update_params(
        self,
        *,
        tool_translation_millimeters,
        tool_rotation_degrees,
        motor_directions,
        calibration_directions,
        calibration_switch_active_high,
        positive_joint_limits_degrees,
        negative_joint_limits_degrees,
        steps_per_degree,
        encoder_counts_per_step,
        dh_theta_degrees,
        dh_alpha_degrees,
        dh_d_millimeters,
        dh_a_millimeters,
        timeout,
        write_admission=None,
    ):
        return self._submit_configuration_transition(
            "update_params",
            {
                "calibration_directions": calibration_directions,
                "calibration_switch_active_high": (
                    calibration_switch_active_high
                ),
                "dh_a_millimeters": dh_a_millimeters,
                "dh_alpha_degrees": dh_alpha_degrees,
                "dh_d_millimeters": dh_d_millimeters,
                "dh_theta_degrees": dh_theta_degrees,
                "encoder_counts_per_step": encoder_counts_per_step,
                "motor_directions": motor_directions,
                "negative_joint_limits_degrees": (
                    negative_joint_limits_degrees
                ),
                "positive_joint_limits_degrees": (
                    positive_joint_limits_degrees
                ),
                "steps_per_degree": steps_per_degree,
                "tool_rotation_degrees": tool_rotation_degrees,
                "tool_translation_millimeters": (
                    tool_translation_millimeters
                ),
            },
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_config_ext_axis(
        self,
        *,
        travel_units,
        drive_rotations,
        motor_steps,
        timeout,
        write_admission=None,
    ):
        return self._submit_configuration_transition(
            "config_ext_axis",
            {
                "drive_rotations": drive_rotations,
                "motor_steps": motor_steps,
                "travel_units": travel_units,
            },
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_zero_external_axis(self, axis, *, timeout, write_admission=None):
        """Submit one correlated external-axis zero transition."""
        if type(axis) is not int or axis not in (7, 8, 9):
            raise JsonCommandSchemaError(
                "external-axis zero requires axis 7, 8, or 9"
            )
        return self._submit_configuration_transition(
            f"zero_j{axis}",
            {},
            timeout=timeout,
            write_admission=write_admission,
        )

    def _submit_motion(
        self,
        command,
        params,
        *,
        timeout,
        write_admission,
        lease_milliseconds=0,
    ):
        with _JsonMainSubmissionReservation(
            self,
            command,
        ) as reservation:
            try:
                ticket = self._coordinator.submit(
                    command,
                    params,
                    timeout=timeout,
                    write_admission=write_admission,
                    maximum_payload_bytes=(
                        reservation.maximum_payload_bytes
                    ),
                )
                with self._state_lock:
                    self._motion = _JsonMainMotionState(
                        ticket=ticket,
                        motion_request_id=ticket.request_id,
                        lease_milliseconds=lease_milliseconds,
                    )
            except BaseException:
                recovered = self._pending_tickets_for_command(command)
                if len(recovered) == 1:
                    with self._state_lock:
                        self._motion = _JsonMainMotionState(
                            ticket=recovered[0],
                            motion_request_id=recovered[0].request_id,
                            lease_milliseconds=lease_milliseconds,
                        )
                raise
            return ticket

    def request_calibrate(
        self,
        *,
        axes,
        offsets,
        timeout,
        write_admission=None,
    ):
        """Submit one selected-axis calibration through the motion owner."""
        return self._submit_motion(
            "calibrate",
            {
                "axes": axes,
                "offsets": offsets,
            },
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_move_joints(
        self,
        *,
        robot_joints_degrees,
        external_axes_units,
        speed_mode,
        speed_value,
        acceleration_percent,
        deceleration_percent,
        ramp_percent,
        wrist_configuration,
        loop_modes,
        telemetry_enabled,
        trace_configuration_fingerprint,
        timeout,
        write_admission=None,
    ):
        return self._submit_motion(
            "move_joints",
            {
                "acceleration_percent": acceleration_percent,
                "deceleration_percent": deceleration_percent,
                "external_axes_units": external_axes_units,
                "loop_modes": loop_modes,
                "ramp_percent": ramp_percent,
                "robot_joints_degrees": robot_joints_degrees,
                "speed_mode": speed_mode,
                "speed_value": speed_value,
                "telemetry_enabled": telemetry_enabled,
                "trace_configuration_fingerprint": (
                    trace_configuration_fingerprint
                ),
                "wrist_configuration": wrist_configuration,
            },
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_jog_tool(
        self,
        *,
        axis,
        direction,
        distance,
        speed_mode,
        speed_value,
        acceleration_percent,
        deceleration_percent,
        ramp_percent,
        wrist_configuration,
        loop_modes,
        timeout,
        write_admission=None,
    ):
        return self._submit_motion(
            "jog_tool",
            {
                "acceleration_percent": acceleration_percent,
                "axis": axis,
                "deceleration_percent": deceleration_percent,
                "direction": direction,
                "distance": distance,
                "loop_modes": loop_modes,
                "ramp_percent": ramp_percent,
                "speed_mode": speed_mode,
                "speed_value": speed_value,
                "wrist_configuration": wrist_configuration,
            },
            timeout=timeout,
            write_admission=write_admission,
        )

    def _request_live_jog(
        self,
        command,
        *,
        axis,
        direction,
        speed_mode,
        speed_value,
        acceleration_percent,
        deceleration_percent,
        ramp_percent,
        wrist_configuration,
        loop_modes,
        telemetry_enabled,
        lease_milliseconds,
        timeout,
        write_admission,
    ):
        return self._submit_motion(
            command,
            {
                "acceleration_percent": acceleration_percent,
                "axis": axis,
                "deceleration_percent": deceleration_percent,
                "direction": direction,
                "lease_milliseconds": lease_milliseconds,
                "loop_modes": loop_modes,
                "ramp_percent": ramp_percent,
                "speed_mode": speed_mode,
                "speed_value": speed_value,
                "telemetry_enabled": telemetry_enabled,
                "wrist_configuration": wrist_configuration,
            },
            timeout=timeout,
            write_admission=write_admission,
            lease_milliseconds=lease_milliseconds,
        )

    def request_live_joint_jog(
        self,
        *,
        axis,
        direction,
        speed_mode,
        speed_value,
        acceleration_percent,
        deceleration_percent,
        ramp_percent,
        wrist_configuration,
        loop_modes,
        telemetry_enabled,
        lease_milliseconds,
        timeout,
        write_admission=None,
    ):
        return self._request_live_jog(
            "live_joint_jog",
            axis=axis,
            direction=direction,
            speed_mode=speed_mode,
            speed_value=speed_value,
            acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent,
            ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration,
            loop_modes=loop_modes,
            telemetry_enabled=telemetry_enabled,
            lease_milliseconds=lease_milliseconds,
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_live_cart_jog(
        self,
        *,
        axis,
        direction,
        speed_mode,
        speed_value,
        acceleration_percent,
        deceleration_percent,
        ramp_percent,
        wrist_configuration,
        loop_modes,
        telemetry_enabled,
        lease_milliseconds,
        timeout,
        write_admission=None,
    ):
        return self._request_live_jog(
            "live_cart_jog",
            axis=axis,
            direction=direction,
            speed_mode=speed_mode,
            speed_value=speed_value,
            acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent,
            ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration,
            loop_modes=loop_modes,
            telemetry_enabled=telemetry_enabled,
            lease_milliseconds=lease_milliseconds,
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_live_tool_jog(
        self,
        *,
        axis,
        direction,
        speed_mode,
        speed_value,
        acceleration_percent,
        deceleration_percent,
        ramp_percent,
        wrist_configuration,
        loop_modes,
        telemetry_enabled,
        lease_milliseconds,
        timeout,
        write_admission=None,
    ):
        return self._request_live_jog(
            "live_tool_jog",
            axis=axis,
            direction=direction,
            speed_mode=speed_mode,
            speed_value=speed_value,
            acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent,
            ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration,
            loop_modes=loop_modes,
            telemetry_enabled=telemetry_enabled,
            lease_milliseconds=lease_milliseconds,
            timeout=timeout,
            write_admission=write_admission,
        )

    def _request_live_motion_control(
        self,
        command,
        motion_ticket,
        *,
        timeout,
        write_admission=None,
    ):
        with _JsonMainSubmissionReservation(
            self,
            command,
            motion_ticket=motion_ticket,
            control_timeout=timeout,
        ) as reservation:
            try:
                ticket = self._coordinator.submit(
                    command,
                    {"motion_id": motion_ticket.request_id},
                    timeout=timeout,
                    write_admission=write_admission,
                    maximum_payload_bytes=(
                        reservation.maximum_payload_bytes
                    ),
                )
                with self._state_lock:
                    self._motion = _JsonMainMotionState(
                        ticket=motion_ticket,
                        control_ticket=ticket,
                        motion_request_id=motion_ticket.request_id,
                        lease_milliseconds=(
                            self._motion.lease_milliseconds
                        ),
                        terminal_order_faulted=(
                            self._motion.terminal_order_faulted
                        ),
                    )
            except BaseException:
                recovered = self._pending_tickets_for_command(command)
                if len(recovered) == 1:
                    with self._state_lock:
                        if self._motion.ticket is motion_ticket:
                            self._motion = _JsonMainMotionState(
                                ticket=motion_ticket,
                                control_ticket=recovered[0],
                                motion_request_id=motion_ticket.request_id,
                                lease_milliseconds=(
                                    self._motion.lease_milliseconds
                                ),
                                terminal_order_faulted=(
                                    self._motion.terminal_order_faulted
                                ),
                            )
                raise
            return ticket

    def request_stop_live_motion(
        self,
        motion_ticket,
        *,
        timeout,
        write_admission=None,
    ):
        return self._request_live_motion_control(
            "stop",
            motion_ticket,
            timeout=timeout,
            write_admission=write_admission,
        )

    def request_renew_live_motion(
        self,
        motion_ticket,
        *,
        timeout,
        write_admission=None,
    ):
        return self._request_live_motion_control(
            "renew_live_motion",
            motion_ticket,
            timeout=timeout,
            write_admission=write_admission,
        )

    def poll(self):
        return self._coordinator.poll()

    def release_reader(self):
        """Release JSON inbound ownership at a complete frame boundary."""
        with self._state_lock:
            self._require_submission_idle_locked("reader release")
        return self._coordinator.release_reader()

    def pop_delivery(self):
        return self._coordinator.pop_delivery()

    def snapshot(self, ticket):
        return self._coordinator.snapshot(ticket)

    def take_terminal(self, ticket):
        """Return a repeatable typed terminal without releasing ownership."""
        with self._state_lock:
            self._require_live_terminal_read_admissible_locked(ticket)
            self._recover_motion_locked()
            self._require_live_terminal_order_locked(ticket)
        response = self._coordinator.take_terminal(ticket)
        terminal = JsonMainControllerTerminal(response)
        with self._state_lock:
            self._require_live_terminal_read_admissible_locked(ticket)
            self._recover_motion_locked()
            self._require_live_terminal_order_locked(ticket)
            state = self._motion
            if (
                ticket is state.control_ticket
                and response.status == "completed"
            ):
                if (
                    state.motion_request_id == 0
                    or terminal.parsed_result.motion_id
                    != state.motion_request_id
                ):
                    raise JsonMainControllerClientStateError(
                        "main-controller JSON live-motion control result "
                        "does not match "
                        "the retained live motion"
                    )
        return terminal

    def acknowledge_terminal(self, ticket):
        with self._state_lock:
            self._require_submission_idle_locked(
                "terminal acknowledgement"
            )
            self._recover_handshake_locked()
            self._recover_configuration_locked()
            self._recover_motion_locked()
            write_ticket = next((
                retained
                for retained in self._coordinator.pending_tickets
                if retained is ticket
                and retained.command in _MAIN_INDETERMINATE_COMMANDS
            ), None)
            if write_ticket is not None:
                terminal = self.take_terminal(write_ticket)
                if terminal.response.status == "failed":
                    raise JsonMainControllerClientStateError(
                        "main-controller side-effect terminal is "
                        "externally indeterminate"
                    )
            if ticket is self._handshake.ticket:
                if self._handshake.terminal is None:
                    hello_terminal = self.take_terminal(ticket)
                    self._handshake = _JsonMainHandshakeState(
                        ticket=ticket,
                        terminal=hello_terminal,
                    )
            if ticket is self._configuration.ticket:
                if self._configuration.terminal is None:
                    configuration_terminal = self.take_terminal(ticket)
                    self._configuration = _JsonMainConfigurationState(
                        ticket=ticket,
                        terminal=configuration_terminal,
                        synchronization_required=(
                            self._configuration.synchronization_required
                        ),
                    )
            if (
                ticket is self._motion.ticket
                or ticket is self._motion.control_ticket
            ):
                self.take_terminal(ticket)
            try:
                self._coordinator.acknowledge_terminal(ticket)
            finally:
                self._recover_handshake_locked()
                self._recover_configuration_locked()
                self._recover_motion_locked()

    def _recover_handshake_locked(self):
        if (
            self._coordinator.quarantined
            or self._coordinator.closing
            or self._coordinator.closed
        ):
            self._handshake = _JsonMainHandshakeState()
            return
        state = self._handshake
        if state.ticket is None or state.terminal is None:
            return
        if any(
            retained is state.ticket
            for retained in self._coordinator.pending_tickets
        ):
            return
        binding = (
            state.terminal.parsed_result
            if state.terminal.response.status == "completed"
            else None
        )
        self._handshake = _JsonMainHandshakeState(binding=binding)

    def _recover_configuration_locked(self):
        if (
            self._coordinator.quarantined
            or self._coordinator.closing
            or self._coordinator.closed
        ):
            self._configuration = _JsonMainConfigurationState()
            return
        state = self._configuration
        if state.ticket is None or state.terminal is None:
            return
        if any(
            retained is state.ticket
            for retained in self._coordinator.pending_tickets
        ):
            return
        synchronization_required = state.synchronization_required
        if state.terminal.response.status == "completed":
            if state.ticket.command in ("update_params", "config_ext_axis"):
                synchronization_required = True
            elif state.ticket.command == "set_position":
                synchronization_required = False
            elif state.ticket.command == "correct_position":
                pass
            elif state.ticket.command in _MAIN_EXTERNAL_AXIS_ZERO_COMMANDS:
                pass
            else:
                raise JsonMainControllerClientStateError(
                    "main-controller configuration transition command "
                    "is invalid"
                )
        self._configuration = _JsonMainConfigurationState(
            synchronization_required=synchronization_required
        )

    def _recover_motion_locked(self):
        if (
            self._coordinator.quarantined
            or self._coordinator.closing
            or self._coordinator.closed
        ):
            self._motion = _JsonMainMotionState()
            return
        state = self._motion
        if state.ticket is None and state.control_ticket is None:
            return
        pending_tickets = self._coordinator.pending_tickets
        ticket = (
            state.ticket
            if any(
                retained is state.ticket
                for retained in pending_tickets
            )
            else None
        )
        control_ticket = (
            state.control_ticket
            if any(
                retained is state.control_ticket
                for retained in pending_tickets
            )
            else None
        )
        if (
            ticket is state.ticket
            and control_ticket is state.control_ticket
        ):
            self._record_live_terminal_order_locked()
            return
        self._motion = _JsonMainMotionState(
            ticket=ticket,
            control_ticket=control_ticket,
            motion_request_id=(
                state.motion_request_id
                if ticket is not None or control_ticket is not None
                else 0
            ),
            lease_milliseconds=(
                state.lease_milliseconds
                if ticket is not None or control_ticket is not None
                else 0
            ),
            terminal_order_faulted=(
                state.terminal_order_faulted
                if ticket is not None or control_ticket is not None
                else False
            ),
        )
        self._record_live_terminal_order_locked()

    def _record_live_terminal_order_locked(self):
        state = self._motion
        if (
            state.ticket is None
            or state.ticket.command not in _MAIN_LIVE_JOG_COMMANDS
            or state.control_ticket is None
        ):
            return
        motion_snapshot = self._coordinator.snapshot(state.ticket)
        control_snapshot = self._coordinator.snapshot(state.control_ticket)
        for name, snapshot in (
            ("motion", motion_snapshot),
            ("control", control_snapshot),
        ):
            terminal_present = snapshot.terminal is not None
            sequence_present = (
                type(snapshot.terminal_sequence) is int
                and snapshot.terminal_sequence > 0
            )
            if terminal_present != sequence_present:
                raise JsonMainControllerClientStateError(
                    "main-controller JSON live-motion "
                    f"{name} terminal ordering metadata is invalid"
                )
        terminal_order_faulted = state.terminal_order_faulted
        if (
            motion_snapshot.terminal is not None
            and (
                control_snapshot.terminal is None
                or motion_snapshot.terminal_sequence
                    <= control_snapshot.terminal_sequence
            )
        ):
            terminal_order_faulted = True
        if terminal_order_faulted == state.terminal_order_faulted:
            return
        self._motion = _JsonMainMotionState(
            ticket=state.ticket,
            control_ticket=state.control_ticket,
            motion_request_id=state.motion_request_id,
            lease_milliseconds=state.lease_milliseconds,
            terminal_order_faulted=terminal_order_faulted,
        )

    def _require_live_terminal_order_locked(self, ticket):
        state = self._motion
        if ticket is state.ticket and state.terminal_order_faulted:
            raise JsonMainControllerClientStateError(
                "main-controller JSON live terminal arrived before "
                "the retained control terminal"
            )

    def _require_live_terminal_read_admissible_locked(self, ticket):
        reservation = self._submission_reservation
        if reservation is None:
            return
        if (
            type(reservation) is not _JsonMainSubmissionReservation
            or reservation._owner is not self
            or reservation.command not in _MAIN_COMMANDS_BY_NAME
        ):
            raise JsonMainControllerClientStateError(
                "main-controller submission reservation is invalid"
            )
        if (
            reservation.command in _MAIN_LIVE_CONTROL_COMMANDS
            and ticket is reservation.motion_ticket
        ):
            raise JsonMainControllerClientStateError(
                "main-controller JSON live terminal read rejected "
                "while control request submission is active"
            )

    def expire(self):
        self._coordinator.expire()

    def retry_deadline_cleanup(self):
        return self._coordinator.retry_deadline_cleanup()

    def close(
        self,
        *,
        timeout=JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
    ):
        return self._coordinator.close(timeout=timeout)


__all__ = (
    "MAIN_JSON_COMMAND_CONTRACTS",
    "JsonMainControllerClient",
    "JsonMainControllerClientStateError",
    "JsonMainControllerTerminal",
    "JsonMainMotionTraceArm",
    "JsonMainMotionTraceAssembly",
    "JsonMainMotionTraceReservation",
    "main_motion_trace_retrieval_eligible",
    "write_main_motion_trace_artifact",
)
