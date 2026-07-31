"""Validated, coalescing joint-motion dispatch for the AR4 HMI.

This module has no GUI or serial dependency.  The desktop application supplies
a blocking request/response callable and consumes completion events on the Tk
event thread.
"""

from dataclasses import dataclass
from contextlib import contextmanager
from collections import deque
from collections.abc import Mapping
from enum import Enum
import json
import math
import re
import struct
import threading
import time
from typing import Callable, Optional, Tuple


JOINT_COUNT = 9
PRIMARY_START_POSITION = (0.0, 0.0, 0.0, 0.0, 45.0, 0.0)
MAX_COMMAND_LENGTH = 4096
MAX_RESPONSE_PAYLOAD_LENGTH = 4096
MAX_RESPONSE_FRAME_LENGTH = MAX_RESPONSE_PAYLOAD_LENGTH + 2
AUXILIARY_BOARD_NONE = "None"
AUXILIARY_BOARD_NANO = "Nano"
AUXILIARY_BOARD_MEGA = "Mega"
AUXILIARY_BOARD_OUTPUT_PINS = {
    AUXILIARY_BOARD_NANO: frozenset(range(8, 14)),
    AUXILIARY_BOARD_MEGA: frozenset(range(28, 54)),
}
AUXILIARY_BOARD_PNEUMATIC_PINS = {
    AUXILIARY_BOARD_NANO: 8,
    AUXILIARY_BOARD_MEGA: 28,
}
AUXILIARY_SERVO_CHANNELS = frozenset(range(7))
AUXILIARY_SERVO_MINIMUM_POSITION = 0
AUXILIARY_SERVO_MAXIMUM_POSITION = 180
CONTROL_POLL_INTERVAL_SECONDS = 0.05
PENDING_CONTROL_FRAME_TIMEOUT_SECONDS = 1.0
RESPONSE_TIMEOUT_SAFETY_SCALE = 1.25
FIRMWARE_MINIMUM_RAMP_PERCENT = 10.0
FIRMWARE_LEGACY_RAMP_NUMERATOR = 200.0
FIRMWARE_DISTRIBUTION_DELAY_MICROSECONDS = 30.0
CONTROLLER_MAXIMUM_RAMP_PERCENT = 100.0
CONTROLLER_FLOAT_MAX = 3.4028234663852886e38
CONTROLLER_SIGNED_INT_MAX = 2147483647
CONTROLLER_MAXIMUM_PULSE_DELAY_MICROSECONDS = 4294967295.0
CONTROLLER_RADIANS_PER_DEGREE = math.pi / 180.0
CONTROLLER_CAPABILITY_GCODE_DIRECTORY_FRAMING_V1 = (
    "GCODE_DIRECTORY_FRAMING_V1"
)
CONTROLLER_CAPABILITY_GCODE_DELETE_IDENTITY_V1 = (
    "GCODE_DELETE_IDENTITY_V1"
)
CONTROLLER_CAPABILITY_GCODE_WRITE_IDENTITY_V1 = (
    "GCODE_WRITE_IDENTITY_V1"
)
CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1 = "JT_WRIST_CONFIG_V1"
CONTROLLER_CAPABILITY_HOME_REFERENCE_V1 = "HOME_REFERENCE_V1"
CONTROLLER_CAPABILITY_HOME_REFERENCE_V2 = "HOME_REFERENCE_V2"
CONTROLLER_CAPABILITY_JOINT_TELEMETRY_V1 = "JOINT_TELEMETRY_V1"
CONTROLLER_CAPABILITY_ESTOP_ADMISSION_V1 = "ESTOP_ADMISSION_V1"
CONTROLLER_ESTOP_ADMISSION_FLAG = "EA"
CONTROLLER_ESTOP_EVENT_FLAG = "EB"
CONTROLLER_PHYSICAL_ESTOP_FLAGS = frozenset(
    (CONTROLLER_ESTOP_ADMISSION_FLAG, CONTROLLER_ESTOP_EVENT_FLAG)
)
JOINT_TELEMETRY_AXIS_COUNT = 6
JOINT_TELEMETRY_PREFIX = "TMA"
JOINT_TELEMETRY_FAMILY_PREFIX = JOINT_TELEMETRY_PREFIX[:2]
JOINT_TELEMETRY_PERIOD_SECONDS = 0.1
JOINT_TELEMETRY_RESPONSE_BUDGET_MARGIN = 2
CONTROLLER_DIRECTORY_SEPARATOR = ","
MAX_CONTROLLER_DIRECTORY_PAYLOAD_BYTES = MAX_RESPONSE_PAYLOAD_LENGTH
MAX_CONTROLLER_IDENTITY_FIELD_LENGTH = 31
MAX_CONTROLLER_CAPABILITY_COUNT = 32
MAX_CONTROLLER_FILENAME_BYTES = 255
CONTROLLER_HARDWARE_ID_LENGTH = 6
CONTROLLER_MEDIA_ID_LENGTH = 32
_FAT_RESERVED_FILENAME_CHARACTERS = frozenset('"*/:<>?\\|')
_CONTROLLER_IDENTITY_FIELD_NAMES = (
    ("DriverModel", "driver_model"),
    ("FirmwareVersion", "firmware_version"),
    ("RobotModel", "robot_model"),
    ("RobotVersion", "robot_version"),
    ("SerialNumber", "serial_number"),
    ("AssetTag", "asset_tag"),
)
_CONTROLLER_IDENTITY_WIRE_FIELDS = frozenset(
    wire_name for wire_name, _ in _CONTROLLER_IDENTITY_FIELD_NAMES
) | {"ControllerHardwareId", "ProtocolCapabilities"}
_SERIAL_QUARANTINE_ATTRIBUTE = "_ar4_transport_quarantine_reason"
_SERIAL_QUARANTINE_LOCK = threading.Lock()
_SERIAL_QUARANTINED_PORTS = {}


class MotionInputError(ValueError):
    """A command input cannot be represented safely by the controller protocol."""


class DeferredJointDispatchOutcome(Enum):
    IDLE = "idle"
    BLOCKED = "blocked"
    DISPATCHED = "dispatched"
    REJECTED = "rejected"


def normalize_auxiliary_board_profile(value, allow_none=False):
    if not isinstance(allow_none, bool):
        raise TypeError("auxiliary-board allow-none flag must be boolean")
    if value is None:
        if allow_none:
            return None
        raise MotionInputError("auxiliary-board profile must be selected")
    if not isinstance(value, str):
        raise MotionInputError("auxiliary-board profile must be text")
    profile = value.strip()
    if profile in ("", AUXILIARY_BOARD_NONE):
        if allow_none:
            return None
        raise MotionInputError("auxiliary-board profile must be selected")
    if profile not in AUXILIARY_BOARD_OUTPUT_PINS:
        raise MotionInputError(
            "auxiliary-board profile must be Nano or Mega"
        )
    return profile


def parse_auxiliary_output_command(command):
    if (
        not isinstance(command, str)
        or not command
        or len(command) > MAX_COMMAND_LENGTH
    ):
        raise MotionInputError("auxiliary output command is invalid")
    match = re.fullmatch(r"(ON|OF)X([0-9]+)\n", command)
    if match is None:
        raise MotionInputError("auxiliary output command is malformed")
    return match.group(1), int(match.group(2))


def validate_auxiliary_output_command(command, board_profile):
    profile = normalize_auxiliary_board_profile(board_profile)
    _, output_pin = parse_auxiliary_output_command(command)
    if output_pin not in AUXILIARY_BOARD_OUTPUT_PINS[profile]:
        raise MotionInputError(
            f"auxiliary output pin {output_pin} is not valid for {profile}"
        )
    return command


def parse_auxiliary_servo_command(command):
    if (
        not isinstance(command, str)
        or not command
        or len(command) > MAX_COMMAND_LENGTH
    ):
        raise MotionInputError("auxiliary servo command is invalid")
    match = re.fullmatch(
        r"SV([0-9]+)P([0-9]+)\n",
        command,
    )
    if match is None:
        raise MotionInputError("auxiliary servo command is malformed")
    channel = int(match.group(1))
    position = int(match.group(2))
    if channel not in AUXILIARY_SERVO_CHANNELS:
        raise MotionInputError(
            f"auxiliary servo channel {channel} is not supported"
        )
    if not (
        AUXILIARY_SERVO_MINIMUM_POSITION
        <= position
        <= AUXILIARY_SERVO_MAXIMUM_POSITION
    ):
        raise MotionInputError(
            "auxiliary servo position must be between "
            f"{AUXILIARY_SERVO_MINIMUM_POSITION} and "
            f"{AUXILIARY_SERVO_MAXIMUM_POSITION}"
        )
    return channel, position


def validate_auxiliary_servo_command(command):
    parse_auxiliary_servo_command(command)
    return command


def auxiliary_pneumatic_output_pin(board_profile):
    profile = normalize_auxiliary_board_profile(board_profile)
    return AUXILIARY_BOARD_PNEUMATIC_PINS[profile]


class LiveMotionScheduleResult(Enum):
    """Non-boolean terminal outcomes from live-motion scheduling admission."""

    CANCELLED = "cancelled"


class SerialActivityRejected(RuntimeError):
    """A legacy serial operation cannot start during shutdown or control injection."""


class _SerialActivityLease:
    """Hold registry ownership across worker and callback thread boundaries."""

    def __init__(self, registry, serial_name, control_injectable):
        self._registry = registry
        self._serial_name = serial_name
        self._control_injectable = control_injectable
        self._lock = threading.Lock()
        self._closed = False

    def close(self):
        with self._lock:
            if self._closed:
                return False
            self._registry.end(
                self._serial_name,
                control_injectable=self._control_injectable,
            )
            self._closed = True
        return True


class SerialActivityRegistry:
    """Track legacy serial ownership for shutdown and injected control writes."""

    CONTROL_INJECT = "inject"
    CONTROL_EXCLUSIVE = "exclusive"

    def __init__(self, serial_names, single_owner_names=()):
        names = self._normalize_name_sequence(serial_names, "serial_names")
        if not names:
            raise MotionInputError("serial_names must not be empty")
        single_owner_names = set(self._normalize_name_sequence(
            single_owner_names,
            "single_owner_names",
        ))
        if single_owner_names.difference(names):
            raise MotionInputError(
                "single_owner_names must be a subset of serial_names"
            )
        self._lock = threading.Lock()
        self._active = {name: 0 for name in names}
        self._noninjectable = {name: 0 for name in names}
        self._control_mode = {name: None for name in names}
        self._single_owner_names = single_owner_names
        self._shutdown = False

    @staticmethod
    def _normalize_name_sequence(serial_names, field_name):
        if isinstance(serial_names, (str, bytes)):
            raise MotionInputError(f"{field_name} must be a sequence of names")
        try:
            names = tuple(serial_names)
        except TypeError as exc:
            raise MotionInputError(
                f"{field_name} must be a sequence of names"
            ) from exc
        if any(
            not isinstance(name, str) or not name.strip() or name != name.strip()
            for name in names
        ):
            raise MotionInputError(
                f"{field_name} must contain non-empty normalized text names"
            )
        if len(set(names)) != len(names):
            raise MotionInputError(f"{field_name} must not contain duplicates")
        return names

    def _require_name(self, serial_name):
        if not isinstance(serial_name, str) or serial_name not in self._active:
            raise MotionInputError(f"unknown serial activity name: {serial_name!r}")
        return serial_name

    def begin(self, serial_name, control_injectable=False):
        name = self._require_name(serial_name)
        if not isinstance(control_injectable, bool):
            raise MotionInputError("control_injectable must be boolean")
        with self._lock:
            if self._shutdown:
                raise SerialActivityRejected(
                    "serial operation rejected during application shutdown"
                )
            if self._control_mode[name] is not None:
                raise SerialActivityRejected(
                    f"serial operation rejected during {name} control dispatch"
                )
            if name in self._single_owner_names and self._active[name] > 0:
                raise SerialActivityRejected(
                    f"serial operation rejected while {name} already has an owner"
                )
            self._active[name] += 1
            if not control_injectable:
                self._noninjectable[name] += 1

    def lease(self, serial_name, control_injectable=False):
        self.begin(serial_name, control_injectable=control_injectable)
        return _SerialActivityLease(
            self,
            serial_name,
            control_injectable,
        )

    def end(self, serial_name, control_injectable=False):
        name = self._require_name(serial_name)
        if not isinstance(control_injectable, bool):
            raise MotionInputError("control_injectable must be boolean")
        with self._lock:
            if self._active[name] <= 0:
                raise RuntimeError(f"serial activity underflow for {name}")
            if not control_injectable and self._noninjectable[name] <= 0:
                raise RuntimeError(
                    f"non-injectable serial activity underflow for {name}"
                )
            self._active[name] -= 1
            if not control_injectable:
                self._noninjectable[name] -= 1

    @contextmanager
    def operations(
        self,
        serial_names,
        control_injectable_names=(),
    ):
        names = self._normalize_name_sequence(serial_names, "serial_names")
        injectable_names = set(self._normalize_name_sequence(
            control_injectable_names,
            "control_injectable_names",
        ))
        unknown_injectable = injectable_names.difference(names)
        if unknown_injectable:
            raise MotionInputError(
                "control_injectable_names must be a subset of serial_names"
            )

        acquired = []
        try:
            for name in names:
                injectable = name in injectable_names
                self.begin(name, control_injectable=injectable)
                acquired.append((name, injectable))
            yield
        finally:
            for name, injectable in reversed(acquired):
                self.end(name, control_injectable=injectable)

    def begin_shutdown(self):
        with self._lock:
            already_started = self._shutdown
            self._shutdown = True
            return not already_started

    def active(self, serial_name):
        name = self._require_name(serial_name)
        with self._lock:
            return self._active[name] > 0

    def idle(self):
        with self._lock:
            return not any(self._active.values()) and not any(
                mode is not None for mode in self._control_mode.values()
            )

    def _reserve_control(self, serial_name, allow_shutdown):
        if not isinstance(allow_shutdown, bool):
            raise MotionInputError("control shutdown admission must be boolean")
        name = self._require_name(serial_name)
        with self._lock:
            if (
                (self._shutdown and not allow_shutdown)
                or self._control_mode[name] is not None
            ):
                return None
            if self._noninjectable[name] > 0:
                return None
            mode = (
                self.CONTROL_INJECT
                if self._active[name] > 0
                else self.CONTROL_EXCLUSIVE
            )
            self._control_mode[name] = mode
            return mode

    def reserve_control(self, serial_name):
        return self._reserve_control(serial_name, False)

    def reserve_emergency_control(self, serial_name):
        """Reserve stop control after normal shutdown admission has closed."""
        return self._reserve_control(serial_name, True)

    def finish_control(self, serial_name, mode):
        name = self._require_name(serial_name)
        if mode not in (self.CONTROL_INJECT, self.CONTROL_EXCLUSIVE):
            raise MotionInputError("unknown serial control mode")
        with self._lock:
            if self._control_mode[name] != mode:
                raise RuntimeError(
                    f"serial control ownership mismatch for {name}"
                )
            self._control_mode[name] = None


class MotionQueueFault(RuntimeError):
    """New motion is blocked until a valid controller position resynchronizes state."""


class MotionTransportBusy(MotionQueueFault):
    """Another command owns the controller transport."""


class ProtocolResponseError(ValueError):
    """A controller response violates the required protocol contract."""


_CONTROLLER_MODBUS_READ_LIMITS = {
    "BA": 65535,
    "BB": 1,
    "BC": 1,
    "BH": 65535,
    "BD": 65535,
}
_CONTROLLER_MODBUS_WRITE_RESPONSES = {
    "BE": "Write Success",
    "BF": "Write Success",
    "SC": "1",
    "SO": "1",
    "WJ": "Done",
    "WK": "Done",
}
_CONTROLLER_MODBUS_REQUEST_OPCODES = frozenset(
    ("BA", "BB", "BC", "BH", "BD", "BE", "BF", "SC", "SO")
)
_CONTROLLER_MODBUS_MAXIMUM_SLAVE_ID = 247
_CONTROLLER_MODBUS_MAXIMUM_ADDRESS = 65535
_CONTROLLER_MODBUS_MAXIMUM_REGISTER_VALUE = 65535
_CONTROLLER_MODBUS_MAXIMUM_REGISTER_READ_QUANTITY = 1
_CONTROLLER_MODBUS_COMMAND_PATTERN = re.compile(
    r"(?P<opcode>BA|BB|BC|BH|BD|BE|BF|SC|SO)"
    r"A(?P<slave>[0-9]+)B(?P<address>[0-9]+)C(?P<value>[0-9]+)\n"
)


def _controller_modbus_integer(value, field_name):
    if not isinstance(field_name, str) or not field_name:
        raise TypeError("controller Modbus field name must be non-empty text")
    if isinstance(value, bool):
        raise MotionInputError(f"controller Modbus {field_name} must be an integer")
    if isinstance(value, int):
        return value
    if (
        not isinstance(value, str)
        or re.fullmatch(r"[0-9]+", value) is None
    ):
        raise MotionInputError(
            f"controller Modbus {field_name} must be unsigned decimal text"
        )
    significant_digits = value.lstrip("0") or "0"
    if len(significant_digits) > len(
        str(_CONTROLLER_MODBUS_MAXIMUM_REGISTER_VALUE)
    ):
        raise MotionInputError(
            f"controller Modbus {field_name} exceeds the protocol range"
        )
    return int(significant_digits)


def build_controller_modbus_command(opcode, slave_id, address, value):
    if (
        not isinstance(opcode, str)
        or opcode not in _CONTROLLER_MODBUS_REQUEST_OPCODES
    ):
        raise MotionInputError("controller Modbus opcode is unsupported")
    slave_id = _controller_modbus_integer(slave_id, "slave ID")
    address = _controller_modbus_integer(address, "address")
    value = _controller_modbus_integer(value, "operation value")

    if not 1 <= slave_id <= _CONTROLLER_MODBUS_MAXIMUM_SLAVE_ID:
        raise MotionInputError("controller Modbus slave ID must be in [1, 247]")
    if not 0 <= address <= _CONTROLLER_MODBUS_MAXIMUM_ADDRESS:
        raise MotionInputError("controller Modbus address must be in [0, 65535]")

    if opcode in ("BB", "BC"):
        if value != 1:
            raise MotionInputError(
                f"controller Modbus {opcode} operation value must be 1"
            )
    elif opcode in ("BA", "BH", "BD"):
        if value != _CONTROLLER_MODBUS_MAXIMUM_REGISTER_READ_QUANTITY:
            raise MotionInputError(
                "controller Modbus register-read quantity must be 1"
            )
    elif opcode in ("BE", "SC"):
        if value not in (0, 1):
            raise MotionInputError(
                "controller Modbus coil value must be 0 or 1"
            )
    elif not 0 <= value <= _CONTROLLER_MODBUS_MAXIMUM_REGISTER_VALUE:
        raise MotionInputError(
            "controller Modbus register value must be in [0, 65535]"
        )

    return f"{opcode}A{slave_id}B{address}C{value}\n"


def validate_controller_modbus_command(command):
    if not isinstance(command, str):
        raise MotionInputError("controller Modbus command must be text")
    match = _CONTROLLER_MODBUS_COMMAND_PATTERN.fullmatch(command)
    if match is None:
        raise MotionInputError("controller Modbus command framing is invalid")
    canonical = build_controller_modbus_command(
        match.group("opcode"),
        match.group("slave"),
        match.group("address"),
        match.group("value"),
    )
    if command != canonical:
        raise MotionInputError("controller Modbus command is not canonical")
    return command


def controller_modbus_command_is_write(command):
    validate_controller_modbus_command(command)
    return command[:2] in ("BE", "BF", "SC", "SO")


def parse_controller_modbus_response(command, response):
    if not isinstance(command, str) or len(command) < 2:
        raise ProtocolResponseError("controller Modbus command is invalid")
    if not isinstance(response, str) or not response:
        raise ProtocolResponseError("controller Modbus response is empty or non-text")

    opcode = command[:2]
    maximum = _CONTROLLER_MODBUS_READ_LIMITS.get(opcode)
    if maximum is not None:
        if re.fullmatch(r"(?:0|[1-9][0-9]*)", response) is None:
            raise ProtocolResponseError(
                f"controller Modbus {opcode} response is not a canonical value"
            )
        value = int(response)
        if value > maximum:
            raise ProtocolResponseError(
                f"controller Modbus {opcode} response exceeds the protocol range"
            )
        return response

    expected = _CONTROLLER_MODBUS_WRITE_RESPONSES.get(opcode)
    if expected is None:
        raise ProtocolResponseError(
            f"controller Modbus opcode {opcode!r} has no response contract"
        )
    if response != expected:
        raise ProtocolResponseError(
            f"controller Modbus {opcode} response is not {expected!r}"
        )
    return response


def classify_controller_modbus_terminal_response(
    command,
    response,
    *,
    paired_with_estop=False,
):
    """Classify a framed Modbus terminal without erasing write uncertainty."""
    validate_controller_modbus_command(command)
    if not isinstance(paired_with_estop, bool):
        raise TypeError("paired Modbus E-stop state must be boolean")
    opcode = command[:2]
    command_is_write = controller_modbus_command_is_write(command)
    if response == "ER":
        if paired_with_estop and command_is_write:
            return "indeterminate"
        return "rejected"
    if response == "Modbus Error":
        if command_is_write:
            return "indeterminate"
        return "rejected"
    if response == "-1" and opcode in ("SC", "SO"):
        return "indeterminate"
    if response == "-2" and opcode in ("SC", "SO"):
        return "rejected"
    parse_controller_modbus_response(command, response)
    return "completed"


@dataclass(frozen=True)
class ControllerIdentity:
    """Validated identity and protocol capabilities reported by a controller."""

    controller_hardware_id: str
    driver_model: str
    firmware_version: str
    robot_model: str
    robot_version: str
    serial_number: str
    asset_tag: str
    protocol_capabilities: tuple


@dataclass(frozen=True)
class PrimaryHomeReference:
    """Controller-reported J1-J3 parking-switch coordinates in degrees."""

    valid: Tuple[bool, bool, bool]
    positions: Tuple[float, float, float]

    def __post_init__(self):
        if (
            not isinstance(self.valid, tuple)
            or len(self.valid) != 3
            or any(not isinstance(value, bool) for value in self.valid)
        ):
            raise MotionInputError(
                "primary home-reference validity must contain three booleans"
            )
        if (
            not isinstance(self.positions, tuple)
            or len(self.positions) != 3
        ):
            raise MotionInputError(
                "primary home-reference positions must contain three values"
            )
        normalized_positions = tuple(
            finite_number(value, f"J{axis} home-reference position")
            for axis, value in enumerate(self.positions, start=1)
        )
        for axis, (valid, position) in enumerate(
            zip(self.valid, normalized_positions),
            start=1,
        ):
            if not valid and position != 0.0:
                raise MotionInputError(
                    f"invalid J{axis} home reference must use a zero position"
                )
        object.__setattr__(self, "positions", normalized_positions)


def validate_controller_hardware_id(value):
    if (
        not isinstance(value, str)
        or re.fullmatch(
            rf"[0-9A-F]{{{CONTROLLER_HARDWARE_ID_LENGTH}}}",
            value,
        )
        is None
    ):
        raise MotionInputError(
            "controller hardware ID must be fixed-width uppercase hexadecimal"
        )
    return value


def validate_controller_media_id(value):
    if (
        not isinstance(value, str)
        or re.fullmatch(
            rf"[0-9A-F]{{{CONTROLLER_MEDIA_ID_LENGTH}}}",
            value,
        )
        is None
    ):
        raise MotionInputError(
            "controller media ID must be fixed-width uppercase hexadecimal"
        )
    return value


def _unique_json_object(pairs):
    result = {}
    for name, value in pairs:
        if not isinstance(name, str) or name in result:
            raise ProtocolResponseError(
                "controller identity response contains a duplicated or invalid field"
            )
        result[name] = value
    return result


def parse_controller_identity_response(response):
    if (
        not isinstance(response, str)
        or not response
        or len(response) > MAX_RESPONSE_PAYLOAD_LENGTH
        or "\n" in response
        or "\r" in response
    ):
        raise ProtocolResponseError("controller identity response is invalid")
    try:
        payload = json.loads(response, object_pairs_hook=_unique_json_object)
    except ProtocolResponseError:
        raise
    except (TypeError, ValueError) as exc:
        raise ProtocolResponseError(
            "controller identity response is not valid JSON"
        ) from exc
    if not isinstance(payload, dict):
        raise ProtocolResponseError("controller identity response must be an object")
    if frozenset(payload) != _CONTROLLER_IDENTITY_WIRE_FIELDS:
        raise ProtocolResponseError(
            "controller identity response fields do not match the protocol schema"
        )

    fields = {}
    for wire_name, field_name in _CONTROLLER_IDENTITY_FIELD_NAMES:
        value = payload.get(wire_name)
        if (
            not isinstance(value, str)
            or not value
            or len(value) > MAX_CONTROLLER_IDENTITY_FIELD_LENGTH
            or not value.isascii()
            or any(ord(character) < 32 or ord(character) == 127 for character in value)
        ):
            raise ProtocolResponseError(
                f"controller identity field {wire_name} is invalid"
            )
        fields[field_name] = value

    try:
        controller_hardware_id = validate_controller_hardware_id(
            payload.get("ControllerHardwareId")
        )
    except MotionInputError as exc:
        raise ProtocolResponseError(
            "controller hardware identity is invalid"
        ) from exc

    capabilities = payload.get("ProtocolCapabilities")
    if (
        not isinstance(capabilities, list)
        or len(capabilities) > MAX_CONTROLLER_CAPABILITY_COUNT
    ):
        raise ProtocolResponseError(
            "controller protocol capabilities must be a bounded list"
        )
    normalized_capabilities = []
    for capability in capabilities:
        if (
            not isinstance(capability, str)
            or re.fullmatch(r"[A-Z][A-Z0-9_]{0,30}", capability) is None
            or capability in normalized_capabilities
        ):
            raise ProtocolResponseError(
                "controller protocol capability is invalid or duplicated"
            )
        normalized_capabilities.append(capability)

    return ControllerIdentity(
        controller_hardware_id=controller_hardware_id,
        **fields,
        protocol_capabilities=tuple(normalized_capabilities),
    )


class SerialTransportQuarantinedError(ConnectionError):
    """Serial framing is uncertain and the connection requires replacement."""


class SerialTransportTimeout(TimeoutError):
    """A response deadline expired and the connection requires replacement."""


class MotionRequestLease:
    """Hold one exclusive logical-motion request across thread boundaries."""

    def __init__(self, registry, name):
        self._registry = registry
        self._name = name
        self._lock = threading.Lock()
        self._closed = False

    @property
    def name(self):
        return self._name

    @property
    def closed(self):
        with self._lock:
            return self._closed

    def close(self):
        with self._lock:
            if self._closed:
                return False
            self._registry._release(self)
            self._closed = True
        return True


class MotionRequestRegistry:
    """Admit one logical motion until every matching result has settled."""

    def __init__(self):
        self._lock = threading.Lock()
        self._active_lease = None

    @staticmethod
    def _normalize_name(name):
        if not isinstance(name, str) or not name.strip() or name != name.strip():
            raise MotionInputError(
                "motion request name must be normalized, non-empty text"
            )
        return name

    def acquire(self, name):
        normalized_name = self._normalize_name(name)
        with self._lock:
            if self._active_lease is not None:
                return None
            lease = MotionRequestLease(
                self,
                normalized_name,
            )
            self._active_lease = lease
            return lease

    @property
    def active(self):
        with self._lock:
            return self._active_lease is not None

    @property
    def active_name(self):
        with self._lock:
            if self._active_lease is None:
                return None
            return self._active_lease.name

    def owns(self, lease):
        if not isinstance(lease, MotionRequestLease):
            return False
        with self._lock:
            return self._active_lease is lease and lease._registry is self

    def _release(self, lease):
        with self._lock:
            if self._active_lease is not lease or lease._registry is not self:
                raise RuntimeError("motion request ownership mismatch")
            self._active_lease = None


class VirtualMotionOperation:
    """Publish one validated terminal result for a virtual-motion request."""

    def __init__(self):
        self._completion_event = threading.Event()
        self._result_lock = threading.Lock()
        self._succeeded = None
        self._error = None

    @property
    def completed(self):
        return self._completion_event.is_set()

    def complete(self, succeeded, error=None):
        if not isinstance(succeeded, bool):
            raise MotionInputError("virtual-motion success state must be boolean")
        if succeeded:
            if error is not None:
                raise MotionInputError(
                    "successful virtual motion must not include an error"
                )
        elif not isinstance(error, str) or not error.strip() or error != error.strip():
            raise MotionInputError(
                "failed virtual motion requires normalized, non-empty error text"
            )
        with self._result_lock:
            if self._completion_event.is_set():
                raise RuntimeError("virtual-motion result was already completed")
            self._succeeded = succeeded
            self._error = error
            self._completion_event.set()
        return True

    def wait(self, timeout=None):
        if timeout is not None:
            timeout = finite_number(timeout, "virtual-motion wait timeout")
            if timeout <= 0:
                raise MotionInputError(
                    "virtual-motion wait timeout must be positive"
                )
        return self._completion_event.wait(timeout)

    def result(self):
        with self._result_lock:
            if not self._completion_event.is_set():
                raise RuntimeError("virtual-motion result is not complete")
            return self._succeeded, self._error


@dataclass(frozen=True)
class DeferredLiveMotionState:
    """Expose one atomic snapshot of deferred live-motion admission."""

    active: object
    pending: object
    generation: int
    attempt_scheduled: bool
    last_error: Optional[str]


class DeferredLiveMotionArbiter:
    """Retry held intent; schedulers must report deferred registration failure."""

    def __init__(
        self,
        schedule,
        start,
        stop,
        retry_delay_ms,
        report_error=None,
    ):
        if not callable(schedule):
            raise MotionInputError("live-motion schedule callback must be callable")
        if not callable(start):
            raise MotionInputError("live-motion start callback must be callable")
        if not callable(stop):
            raise MotionInputError("live-motion stop callback must be callable")
        if (
            isinstance(retry_delay_ms, bool)
            or not isinstance(retry_delay_ms, int)
            or retry_delay_ms <= 0
        ):
            raise MotionInputError("live-motion retry delay must be a positive integer")
        if report_error is not None and not callable(report_error):
            raise MotionInputError(
                "live-motion error-report callback must be callable or None"
            )
        self._schedule_callback = schedule
        self._start_callback = start
        self._stop_callback = stop
        self._report_error_callback = report_error
        self._retry_delay_ms = retry_delay_ms
        self._lock = threading.Lock()
        self._active = None
        self._pending = None
        self._generation = 0
        self._attempt_scheduled = False
        self._last_error = None

    @property
    def active(self):
        with self._lock:
            return self._active

    @property
    def pending(self):
        with self._lock:
            return self._pending

    @property
    def last_error(self):
        with self._lock:
            return self._last_error

    def snapshot(self):
        with self._lock:
            return DeferredLiveMotionState(
                active=self._active,
                pending=self._pending,
                generation=self._generation,
                attempt_scheduled=self._attempt_scheduled,
                last_error=self._last_error,
            )

    def _record_error(self, context, error):
        detail = str(error).strip() or type(error).__name__
        message = f"{context}: {type(error).__name__}: {detail}"
        with self._lock:
            self._last_error = message
        if self._report_error_callback is None:
            return message
        try:
            self._report_error_callback(message)
        except Exception as report_error:
            report_detail = str(report_error).strip() or type(report_error).__name__
            with self._lock:
                self._last_error = (
                    f"{message}; error reporter failed: "
                    f"{type(report_error).__name__}: {report_detail}"
                )
        return message

    def _settle_schedule_failure(self, generation, failure, context):
        cancelled = failure is LiveMotionScheduleResult.CANCELLED
        with self._lock:
            if (
                generation != self._generation
                or not self._attempt_scheduled
            ):
                return False
            self._attempt_scheduled = False
            if cancelled:
                self._pending = None
        if cancelled:
            return True
        if not isinstance(failure, BaseException):
            failure = RuntimeError(
                "schedule failure callback returned an invalid error"
            )
        self._record_error(context, failure)
        return True

    def _request_stop(self, active, generation):
        try:
            stopped = self._stop_callback() is True
        except Exception as exc:
            self._record_error("live-motion stop callback failed", exc)
            return False
        if not stopped:
            self._record_error(
                "live-motion stop callback rejected the request",
                RuntimeError("stop admission returned a non-true result"),
            )
            return False
        with self._lock:
            if generation != self._generation or self._active != active:
                return False
            self._active = None
        return True

    def request(self, desired):
        active_to_stop = None
        schedule_required = False
        delay_ms = 0
        with self._lock:
            if desired is None:
                if self._active is None and self._pending is None:
                    return False
                self._generation += 1
                self._pending = None
                self._attempt_scheduled = False
                generation = self._generation
                active_to_stop = self._active
            elif desired == self._active and self._pending is None:
                return False
            elif desired == self._pending:
                generation = self._generation
                active_to_stop = self._active
                schedule_required = (
                    active_to_stop is None and not self._attempt_scheduled
                )
                delay_ms = self._retry_delay_ms
            else:
                self._generation += 1
                generation = self._generation
                self._pending = desired
                self._attempt_scheduled = False
                active_to_stop = self._active
                schedule_required = active_to_stop is None
                delay_ms = self._retry_delay_ms if active_to_stop is not None else 0

        if active_to_stop is not None:
            if not self._request_stop(active_to_stop, generation):
                return False
            schedule_required = desired is not None
            delay_ms = self._retry_delay_ms
        if schedule_required:
            return self._schedule_attempt(generation, delay_ms)
        return True

    def _schedule_attempt(self, generation, delay_ms):
        with self._lock:
            if generation != self._generation or self._pending is None:
                return False
            if self._attempt_scheduled:
                return True
            self._attempt_scheduled = True

        try:
            scheduled = self._schedule_callback(
                delay_ms,
                lambda: self._attempt(generation),
                lambda failure: self._settle_schedule_failure(
                    generation,
                    failure,
                    "live-motion deferred scheduling failed",
                ),
            )
        except Exception as exc:
            self._settle_schedule_failure(
                generation,
                exc,
                "live-motion scheduling callback failed",
            )
            return False
        if scheduled is LiveMotionScheduleResult.CANCELLED:
            self._settle_schedule_failure(
                generation,
                scheduled,
                "live-motion scheduling cancelled",
            )
            return False
        if scheduled is not True:
            self._settle_schedule_failure(
                generation,
                RuntimeError("schedule admission returned a non-true result"),
                "live-motion scheduling callback rejected the request",
            )
            return False
        return True

    def _attempt(self, generation):
        with self._lock:
            if generation != self._generation or self._pending is None:
                return False
            desired = self._pending
            self._attempt_scheduled = False

        try:
            admitted = self._start_callback(desired) is True
        except Exception as exc:
            self._record_error("live-motion start callback failed", exc)
            admitted = False

        stale_success = False
        retry_required = False
        with self._lock:
            if generation != self._generation or desired != self._pending:
                stale_success = admitted
                if stale_success and self._active is None:
                    self._active = desired
            elif admitted:
                self._active = desired
                self._pending = None
            else:
                retry_required = True

        if stale_success:
            with self._lock:
                current_generation = self._generation
            self._request_stop(desired, current_generation)
        if retry_required:
            self._schedule_attempt(generation, self._retry_delay_ms)
        return admitted and not stale_success


def _serial_command_bytes(command):
    if not isinstance(command, str) or not command.endswith("\n"):
        raise MotionInputError("serial command must be newline-terminated text")
    if not command[:-1] or "\n" in command[:-1] or "\r" in command:
        raise MotionInputError("serial command must contain exactly one trailing line delimiter")
    if len(command) > MAX_COMMAND_LENGTH:
        raise MotionInputError("serial command exceeds the size limit")
    try:
        return command.encode("ascii")
    except UnicodeEncodeError as exc:
        raise MotionInputError("serial command must contain ASCII characters only") from exc


def _require_open_serial_port(serial_port):
    quarantine_reason = _serial_quarantine_reason(serial_port)
    if quarantine_reason is not None:
        raise SerialTransportQuarantinedError(
            "serial connection is quarantined "
            f"({quarantine_reason}); reconnect required"
        )
    if serial_port is None or not getattr(serial_port, "is_open", False):
        raise ConnectionError("controller serial connection is not open")


def _record_serial_quarantine(serial_port, reason):
    if serial_port is None:
        return
    # Retaining failed handles prevents object-id reuse from defeating quarantine.
    with _SERIAL_QUARANTINE_LOCK:
        _SERIAL_QUARANTINED_PORTS[id(serial_port)] = (serial_port, reason)


def _serial_quarantine_reason(serial_port):
    if serial_port is None:
        return None
    with _SERIAL_QUARANTINE_LOCK:
        entry = _SERIAL_QUARANTINED_PORTS.get(id(serial_port))
        if entry is not None and entry[0] is serial_port:
            return entry[1]
    try:
        return getattr(serial_port, _SERIAL_QUARANTINE_ATTRIBUTE, None)
    except Exception:
        return None


def serial_transport_quarantined(serial_port):
    return _serial_quarantine_reason(serial_port) is not None


def quarantine_serial_transport(serial_port, reason):
    """Close and quarantine a transport whose controller state is uncertain."""
    if serial_port is None:
        raise MotionInputError("serial connection is required for quarantine")
    if (
        not isinstance(reason, str)
        or not reason
        or reason != reason.strip()
        or "\r" in reason
        or "\n" in reason
    ):
        raise MotionInputError("serial quarantine reason must be normalized text")
    already_quarantined = serial_transport_quarantined(serial_port)
    if already_quarantined and not getattr(serial_port, "is_open", False):
        return False
    try:
        _raise_quarantined_transport(serial_port, reason)
    except SerialTransportQuarantinedError:
        if getattr(serial_port, "is_open", False):
            raise
        return True


def _raise_quarantined_transport(
    serial_port,
    reason,
    cause=None,
    error_type=SerialTransportQuarantinedError,
):
    _record_serial_quarantine(serial_port, reason)

    mark_error = None
    try:
        setattr(serial_port, _SERIAL_QUARANTINE_ATTRIBUTE, reason)
    except Exception as exc:
        mark_error = exc

    close_error = None
    close = None
    try:
        close = getattr(serial_port, "close", None)
    except Exception as exc:
        close_error = exc
    if close_error is None:
        if not callable(close):
            close_error = TypeError("serial connection does not support close")
        else:
            try:
                close()
                if getattr(serial_port, "is_open", False):
                    close_error = OSError("serial connection remained open")
            except Exception as exc:
                close_error = exc

    details = [reason]
    if close_error is None:
        details.append("serial connection closed; reconnect required")
    else:
        details.append(
            f"serial close failed ({close_error}); "
            "connection retained for cleanup; reconnect required"
        )
    if mark_error is not None:
        details.append(f"quarantine marker failed: {mark_error}")

    error = error_type("; ".join(details))
    error_cause = cause
    if error_cause is None:
        error_cause = close_error
    if error_cause is None:
        error_cause = mark_error
    if error_cause is not None:
        raise error from error_cause
    raise error


def _validate_write_lock(write_lock, parameter_name="write_lock"):
    if not isinstance(parameter_name, str) or not parameter_name:
        raise TypeError("write-lock parameter name must be non-empty text")
    if write_lock is None:
        return
    if not callable(getattr(write_lock, "acquire", None)) or not callable(
        getattr(write_lock, "release", None)
    ):
        raise MotionInputError(
            f"{parameter_name} must satisfy the lock contract"
        )


def _write_serial_bytes(
    serial_port,
    command_bytes,
    write_lock,
    reset_input=None,
    write_admission_check=None,
    write_started_event=None,
    write_boundary_lock=None,
):
    write = getattr(serial_port, "write", None)
    flush = getattr(serial_port, "flush", None)
    if not all(callable(method) for method in (write, flush)):
        raise TypeError("serial connection does not satisfy the line-write contract")
    if write_admission_check is not None and not callable(write_admission_check):
        raise TypeError("serial write admission check must be callable")
    if write_started_event is not None and not callable(
        getattr(write_started_event, "set", None)
    ):
        raise TypeError("serial write-start event must satisfy the event contract")
    _validate_write_lock(write_boundary_lock, "write_boundary_lock")
    if write_boundary_lock is not None and write_boundary_lock is write_lock:
        raise MotionInputError(
            "serial write and write-boundary locks must be distinct"
        )

    acquired = False
    if write_lock is not None:
        acquired = write_lock.acquire()
        if acquired is False:
            raise RuntimeError("serial write lock acquisition failed")
    try:
        if write_admission_check is not None:
            write_admission_check()
        if reset_input is not None:
            reset_input()
        boundary_acquired = False
        if write_boundary_lock is not None:
            boundary_acquired = write_boundary_lock.acquire()
            if boundary_acquired is False:
                raise RuntimeError("serial write-boundary lock acquisition failed")
        try:
            if write_admission_check is not None:
                write_admission_check()
            if write_started_event is not None:
                write_started_event.set()
        finally:
            if write_boundary_lock is not None and boundary_acquired is not False:
                write_boundary_lock.release()
        written = write(command_bytes)
        if written != len(command_bytes):
            raise OSError(
                "serial write was incomplete: "
                f"expected {len(command_bytes)} bytes, wrote {written!r}"
            )
        flush()
    finally:
        if write_lock is not None and acquired is not False:
            write_lock.release()


def _set_serial_timeout(serial_port, timeout):
    try:
        serial_port.timeout = timeout
    except Exception as exc:
        raise TypeError("serial connection does not support timeout updates") from exc


def _raise_quarantined_timeout(serial_port, timeout, stop_error=None):
    details = [f"no controller response within {timeout:g} seconds"]
    if stop_error is not None:
        details.append(f"fail-safe stop write failed: {stop_error}")
    _raise_quarantined_transport(
        serial_port,
        "; ".join(details),
        cause=stop_error,
        error_type=SerialTransportTimeout,
    )


def write_serial_control(
    serial_port,
    command,
    write_lock=None,
    reset_input=False,
    write_started_event=None,
    cancellation_event=None,
    write_boundary_lock=None,
):
    """Write a serialized command without consuming the pending response.

    The optional cancellation lock orders the final admission check against
    write commitment. Cancellation after commitment remains the caller's
    response-ownership responsibility.
    """
    command_bytes = _serial_command_bytes(command)
    _require_open_serial_port(serial_port)
    _validate_write_lock(write_lock)
    if not isinstance(reset_input, bool):
        raise MotionInputError("reset_input must be boolean")
    reset = None
    if reset_input:
        reset = getattr(serial_port, "reset_input_buffer", None)
        if not callable(reset):
            reset = getattr(serial_port, "flushInput", None)
        if not callable(reset):
            raise TypeError("serial connection does not support input-buffer reset")
    if cancellation_event is None:
        if write_boundary_lock is not None:
            raise MotionInputError(
                "write_boundary_lock requires a cancellation_event"
            )
        write_admission_check = None
    else:
        if not callable(getattr(cancellation_event, "is_set", None)):
            raise MotionInputError(
                "cancellation_event must satisfy the event contract"
            )
        if write_started_event is None or write_boundary_lock is None:
            raise MotionInputError(
                "cancellation_event requires write_started_event and "
                "write_boundary_lock"
            )

        def write_admission_check():
            try:
                cancelled = cancellation_event.is_set()
            except Exception as exc:
                raise MotionInputError(
                    "cancellation_event could not be read before transmission"
                ) from exc
            if not isinstance(cancelled, bool):
                raise MotionInputError(
                    "cancellation_event.is_set() must return a boolean"
                )
            if cancelled:
                raise SerialActivityRejected(
                    "serial control write cancelled before transmission"
                )

    try:
        _write_serial_bytes(
            serial_port,
            command_bytes,
            write_lock,
            reset_input=reset,
            write_admission_check=write_admission_check,
            write_started_event=write_started_event,
            write_boundary_lock=write_boundary_lock,
        )
    except SerialActivityRejected:
        raise
    except Exception as exc:
        _raise_quarantined_transport(
            serial_port,
            f"serial control write ended without trusted framing: {exc}",
            cause=exc,
        )
    return True


def _validated_response_set(accepted_responses):
    if accepted_responses is None:
        return None
    if isinstance(accepted_responses, (str, bytes)):
        raise MotionInputError("accepted_responses must be a sequence of text")
    try:
        responses = tuple(accepted_responses)
    except TypeError as exc:
        raise MotionInputError(
            "accepted_responses must be a sequence of text"
        ) from exc
    if not responses:
        raise MotionInputError("accepted_responses must not be empty")
    if any(
        not isinstance(response, str)
        or not response
        or response != response.strip()
        or "\r" in response
        or "\n" in response
        or not response.isascii()
        or len(response) > MAX_RESPONSE_PAYLOAD_LENGTH
        for response in responses
    ):
        raise MotionInputError(
            "accepted_responses must contain normalized non-empty ASCII text"
        )
    if len(set(responses)) != len(responses):
        raise MotionInputError("accepted_responses must not contain duplicates")
    return frozenset(responses)


def decode_serial_response_line(response_bytes, allow_empty=False):
    """Decode exactly one ASCII line without normalizing payload bytes."""
    if not isinstance(allow_empty, bool):
        raise MotionInputError("allow_empty must be boolean")
    if not isinstance(response_bytes, (bytes, bytearray)):
        raise ProtocolResponseError("serial line reader returned a non-bytes response")
    framed = bytes(response_bytes)
    if not framed.endswith(b"\n"):
        raise ProtocolResponseError("controller response is missing the line delimiter")
    if len(framed) > MAX_RESPONSE_FRAME_LENGTH:
        raise ProtocolResponseError("controller response exceeds the size limit")
    payload = framed[:-1]
    if payload.endswith(b"\r"):
        payload = payload[:-1]
    if len(payload) > MAX_RESPONSE_PAYLOAD_LENGTH:
        raise ProtocolResponseError("controller response payload exceeds the size limit")
    if b"\r" in payload or b"\n" in payload:
        raise ProtocolResponseError("controller response contains extra line delimiters")
    try:
        response = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ProtocolResponseError(
            "controller response is not valid ASCII"
        ) from exc
    if response != response.strip():
        raise ProtocolResponseError(
            "controller response contains leading or trailing payload whitespace"
        )
    if not response and not allow_empty:
        raise ProtocolResponseError("controller returned a blank response line")
    return response


def _read_serial_quiet_boundary(serial_port, read, timeout, context):
    quiet_timeout = finite_number(timeout, "serial quiet-boundary timeout")
    if quiet_timeout < CONTROL_POLL_INTERVAL_SECONDS:
        _raise_quarantined_transport(
            serial_port,
            f"{context} quiet-boundary deadline expired",
            error_type=SerialTransportTimeout,
        )
    _set_serial_timeout(serial_port, CONTROL_POLL_INTERVAL_SECONDS)
    trailing = read(1)
    if not isinstance(trailing, (bytes, bytearray)):
        raise ProtocolResponseError(
            f"{context} quiet-boundary reader returned a non-bytes response"
        )
    if trailing:
        raise ProtocolResponseError(f"{context} returned queued trailing data")
    if not getattr(serial_port, "is_open", False):
        raise ConnectionError(f"{context} connection closed during the quiet boundary")


def _read_serial_framed_line(
    serial_port,
    readline,
    read_until,
    deadline,
    timeout,
    initial_bytes=b"",
    allow_empty_terminal_response=False,
):
    response_buffer = bytearray(initial_bytes)
    while True:
        if len(response_buffer) > MAX_RESPONSE_FRAME_LENGTH:
            raise ProtocolResponseError("controller response exceeds the size limit")
        newline_index = response_buffer.find(b"\n")
        if newline_index >= 0:
            if newline_index != len(response_buffer) - 1:
                raise ProtocolResponseError(
                    "controller returned trailing framed data"
                )
            return decode_serial_response_line(
                response_buffer,
                allow_empty=allow_empty_terminal_response,
            )

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            _raise_quarantined_timeout(serial_port, timeout)
        _set_serial_timeout(serial_port, remaining)
        remaining_size = MAX_RESPONSE_FRAME_LENGTH + 1 - len(response_buffer)
        if callable(read_until):
            response_bytes = read_until(b"\n", remaining_size)
        else:
            response_bytes = readline()
        if not isinstance(response_bytes, (bytes, bytearray)):
            raise ProtocolResponseError(
                "serial line reader returned a non-bytes response"
            )
        if not response_bytes:
            _raise_quarantined_timeout(serial_port, timeout)
        response_buffer.extend(response_bytes)


def read_serial_line_response(
    serial_port,
    timeout,
    accepted_responses=None,
    response_deadline=None,
    allow_empty_terminal_response=False,
):
    """Read one bounded response line without taking command-write ownership.

    An absolute monotonic deadline can shorten but cannot extend the timeout.
    An explicitly admitted blank payload must be the command's terminal
    response before the required quiet boundary; non-terminal acknowledgements
    require a continuing response owner. Dynamic terminal responses must be
    validated by the caller before higher-level transport ownership is released.
    Empty terminal admission cannot be combined with accepted_responses.
    """
    if not isinstance(allow_empty_terminal_response, bool):
        raise MotionInputError(
            "allow_empty_terminal_response must be boolean"
        )
    if allow_empty_terminal_response and accepted_responses is not None:
        raise MotionInputError(
            "allow_empty_terminal_response cannot be combined with "
            "accepted_responses"
        )
    timeout = finite_number(timeout, "serial response timeout")
    if timeout <= 0:
        raise MotionInputError("serial response timeout must be positive")
    now = time.monotonic()
    if response_deadline is None:
        deadline = now + timeout
    else:
        deadline = finite_number(response_deadline, "serial response deadline")
        if deadline > now + timeout:
            raise MotionInputError(
                "serial response deadline exceeds its timeout window"
            )
    accepted = _validated_response_set(accepted_responses)
    _require_open_serial_port(serial_port)

    readline = getattr(serial_port, "readline", None)
    read_until = getattr(serial_port, "read_until", None)
    read = getattr(serial_port, "read", None)
    if not (callable(read_until) or callable(readline)) or not callable(read):
        raise TypeError("serial connection does not satisfy the line-read contract")
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError("serial connection does not expose a read timeout") from exc

    operation_error = None
    try:
        response = _read_serial_framed_line(
            serial_port,
            readline,
            read_until,
            deadline,
            timeout,
            allow_empty_terminal_response=allow_empty_terminal_response,
        )
        if accepted is not None and response not in accepted:
            raise ProtocolResponseError(
                f"controller returned an unexpected response: {response!r}"
            )
        _read_serial_quiet_boundary(
            serial_port,
            read,
            deadline - time.monotonic(),
            "controller response",
        )
        return response
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            f"serial response read ended without trusted framing: {exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def read_serial_line_response_with_optional_followup(
    serial_port,
    timeout,
    accepted_responses,
    followup_after_responses,
    accepted_followup_responses,
    control_event=None,
    control_response_timeout_seconds=None,
    control_response_deadline_provider=None,
):
    """Read a primary line and an authorized follow-up under one owner.

    A control deadline provider supplies an existing absolute monotonic deadline.
    The supplied deadline can shorten but cannot extend the configured window.
    """
    timeout = finite_number(timeout, "serial response timeout")
    if timeout <= 0:
        raise MotionInputError("serial response timeout must be positive")
    accepted = _validated_response_set(accepted_responses)
    followup_after = _validated_response_set(followup_after_responses)
    accepted_followups = _validated_response_set(accepted_followup_responses)
    if accepted is None or followup_after is None or accepted_followups is None:
        raise MotionInputError(
            "optional follow-up response contracts must contain response sets"
        )
    if not followup_after.issubset(accepted):
        raise MotionInputError(
            "followup_after_responses must be a subset of accepted_responses"
        )
    if control_event is None:
        if control_response_timeout_seconds is not None:
            raise MotionInputError(
                "control response timeout requires a control_event"
            )
        if control_response_deadline_provider is not None:
            raise MotionInputError(
                "control response deadline provider requires a control_event"
            )
        control_response_timeout = None
    else:
        if not callable(getattr(control_event, "is_set", None)):
            raise MotionInputError("control_event must satisfy the event contract")
        if control_response_timeout_seconds is None:
            raise MotionInputError(
                "control_event requires a control_response_timeout_seconds"
            )
        control_response_timeout = finite_number(
            control_response_timeout_seconds,
            "control_response_timeout_seconds",
        )
        if control_response_timeout <= 0:
            raise MotionInputError(
                "control_response_timeout_seconds must be positive"
            )
        if control_response_deadline_provider is not None and not callable(
            control_response_deadline_provider
        ):
            raise MotionInputError(
                "control_response_deadline_provider must be callable"
            )
    _require_open_serial_port(serial_port)

    readline = getattr(serial_port, "readline", None)
    read_until = getattr(serial_port, "read_until", None)
    read = getattr(serial_port, "read", None)
    if not (callable(read_until) or callable(readline)) or not callable(read):
        raise TypeError("serial connection does not satisfy the line-read contract")
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError("serial connection does not expose a read timeout") from exc

    operation_error = None
    try:
        deadline = time.monotonic() + timeout
        control_deadline = None

        def active_deadline():
            nonlocal control_deadline
            if control_event is not None:
                try:
                    control_requested = control_event.is_set()
                except Exception as exc:
                    raise ProtocolResponseError(
                        f"control_event could not be read: {exc}"
                    ) from exc
                if not isinstance(control_requested, bool):
                    raise ProtocolResponseError(
                        "control_event.is_set() must return a boolean"
                    )
                if control_requested and control_deadline is None:
                    now = time.monotonic()
                    if control_response_deadline_provider is None:
                        control_deadline = now + control_response_timeout
                    else:
                        control_deadline = finite_number(
                            control_response_deadline_provider(),
                            "control response deadline",
                        )
                        if control_deadline > now + control_response_timeout:
                            raise ProtocolResponseError(
                                "control response deadline exceeds its timeout window"
                            )
            if control_deadline is not None:
                if control_response_deadline_provider is not None:
                    return control_deadline, control_response_timeout
                if control_deadline < deadline:
                    return control_deadline, control_response_timeout
            return deadline, timeout

        def read_controlled_line(initial_bytes=b""):
            response_buffer = bytearray(initial_bytes)
            while True:
                if len(response_buffer) > MAX_RESPONSE_FRAME_LENGTH:
                    raise ProtocolResponseError(
                        "controller response exceeds the size limit"
                    )
                newline_index = response_buffer.find(b"\n")
                if newline_index >= 0:
                    if newline_index != len(response_buffer) - 1:
                        raise ProtocolResponseError(
                            "controller returned trailing framed data"
                        )
                    return decode_serial_response_line(response_buffer)

                current_deadline, current_timeout = active_deadline()
                remaining = current_deadline - time.monotonic()
                if remaining <= 0:
                    _raise_quarantined_timeout(serial_port, current_timeout)
                _set_serial_timeout(
                    serial_port,
                    min(CONTROL_POLL_INTERVAL_SECONDS, remaining),
                )
                remaining_size = MAX_RESPONSE_FRAME_LENGTH + 1 - len(response_buffer)
                if callable(read_until):
                    response_bytes = read_until(b"\n", remaining_size)
                else:
                    response_bytes = readline()
                if not isinstance(response_bytes, (bytes, bytearray)):
                    raise ProtocolResponseError(
                        "serial line reader returned a non-bytes response"
                    )
                if not response_bytes:
                    if not getattr(serial_port, "is_open", False):
                        raise ConnectionError(
                            "serial connection closed before a terminal response"
                        )
                    continue
                response_buffer.extend(response_bytes)

        if control_event is None:
            response = _read_serial_framed_line(
                serial_port,
                readline,
                read_until,
                deadline,
                timeout,
            )
        else:
            response = read_controlled_line()
        if response not in accepted:
            raise ProtocolResponseError(
                f"controller returned an unexpected response: {response!r}"
            )

        current_deadline, current_timeout = active_deadline()
        remaining = current_deadline - time.monotonic()
        if remaining < CONTROL_POLL_INTERVAL_SECONDS:
            _raise_quarantined_transport(
                serial_port,
                "controller response follow-up probe deadline expired",
                error_type=SerialTransportTimeout,
            )
        _set_serial_timeout(
            serial_port,
            CONTROL_POLL_INTERVAL_SECONDS,
        )
        leading = read(1)
        if not isinstance(leading, (bytes, bytearray)):
            raise ProtocolResponseError(
                "controller follow-up probe returned a non-bytes response"
            )
        if not leading:
            if not getattr(serial_port, "is_open", False):
                raise ConnectionError(
                    "controller response connection closed during the quiet boundary"
                )
            return response, None
        if response not in followup_after:
            raise ProtocolResponseError(
                "controller returned a follow-up after an ineligible response"
            )

        if control_event is None:
            followup = _read_serial_framed_line(
                serial_port,
                readline,
                read_until,
                deadline,
                timeout,
                initial_bytes=leading,
            )
        else:
            followup = read_controlled_line(initial_bytes=leading)
        if followup not in accepted_followups:
            raise ProtocolResponseError(
                f"controller returned an unexpected follow-up response: {followup!r}"
            )
        current_deadline, _ = active_deadline()
        _read_serial_quiet_boundary(
            serial_port,
            read,
            current_deadline - time.monotonic(),
            "controller follow-up response",
        )
        return response, followup
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            f"serial response read ended without trusted framing: {exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def read_serial_exact_response(serial_port, expected_response, timeout):
    """Read exact unframed bytes followed by a bounded quiet boundary."""
    if (
        not isinstance(expected_response, bytes)
        or not expected_response
        or len(expected_response) > MAX_RESPONSE_PAYLOAD_LENGTH
        or expected_response != expected_response.strip()
        or b"\r" in expected_response
        or b"\n" in expected_response
    ):
        raise MotionInputError(
            "expected_response must contain normalized, non-empty unframed bytes"
        )
    try:
        expected_response.decode("ascii")
    except UnicodeDecodeError as exc:
        raise MotionInputError(
            "expected_response must contain ASCII bytes only"
        ) from exc
    timeout = finite_number(timeout, "serial response timeout")
    if timeout <= 0:
        raise MotionInputError("serial response timeout must be positive")
    _require_open_serial_port(serial_port)
    read = getattr(serial_port, "read", None)
    if not callable(read):
        raise TypeError("serial connection does not satisfy the exact-read contract")
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError("serial connection does not expose a read timeout") from exc

    response = bytearray()
    operation_error = None
    try:
        deadline = time.monotonic() + timeout
        while len(response) < len(expected_response):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                _raise_quarantined_timeout(serial_port, timeout)
            _set_serial_timeout(serial_port, remaining)
            response_bytes = read(len(expected_response) - len(response))
            if not isinstance(response_bytes, (bytes, bytearray)):
                raise ProtocolResponseError(
                    "serial exact reader returned a non-bytes response"
                )
            if not response_bytes:
                _raise_quarantined_timeout(serial_port, timeout)
            response.extend(response_bytes)
        if bytes(response) != expected_response:
            raise ProtocolResponseError(
                "controller returned an unexpected unframed response"
            )
        remaining = deadline - time.monotonic()
        if remaining < CONTROL_POLL_INTERVAL_SECONDS:
            _raise_quarantined_transport(
                serial_port,
                "serial exact response quiet-boundary deadline expired",
                error_type=SerialTransportTimeout,
            )
        _set_serial_timeout(serial_port, CONTROL_POLL_INTERVAL_SECONDS)
        trailing = read(1)
        if not isinstance(trailing, (bytes, bytearray)):
            raise ProtocolResponseError(
                "serial exact reader returned a non-bytes quiet-boundary response"
            )
        if trailing:
            raise ProtocolResponseError(
                "controller returned trailing unframed response data"
            )
        if not getattr(serial_port, "is_open", False):
            raise ConnectionError(
                "serial connection closed before the quiet boundary completed"
            )
        return expected_response.decode("ascii")
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            f"serial exact read ended without trusted framing: {exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def exchange_serial_line_until_cancelled(
    serial_port,
    command,
    cancellation_event,
    write_lock=None,
    poll_interval_seconds=CONTROL_POLL_INTERVAL_SECONDS,
    write_started_event=None,
    write_boundary_lock=None,
    write_cancellation_event=None,
    reset_input=True,
    estop_callback=None,
):
    """Own a framed exchange until terminal data or explicit cancellation.

    A write-commitment event is set after admission and reset checks while the
    optional boundary lock remains held. Callers requiring latched post-commit
    ownership must provide cancellation semantics that observe that event.
    """
    if not callable(getattr(cancellation_event, "is_set", None)):
        raise MotionInputError("cancellation_event must satisfy the event contract")
    try:
        cancelled = cancellation_event.is_set()
    except Exception as exc:
        raise MotionInputError("cancellation_event could not be read") from exc
    if not isinstance(cancelled, bool):
        raise MotionInputError("cancellation_event.is_set() must return a boolean")
    if cancelled:
        raise SerialActivityRejected("serial exchange cancelled before transmission")
    if write_cancellation_event is not None:
        if not callable(getattr(write_cancellation_event, "is_set", None)):
            raise MotionInputError(
                "write_cancellation_event must satisfy the event contract"
            )
        try:
            write_cancelled = write_cancellation_event.is_set()
        except Exception as exc:
            raise MotionInputError(
                "write_cancellation_event could not be read"
            ) from exc
        if not isinstance(write_cancelled, bool):
            raise MotionInputError(
                "write_cancellation_event.is_set() must return a boolean"
            )
        if write_cancelled:
            raise SerialActivityRejected(
                "serial exchange cancelled before transmission"
            )
        if write_started_event is None or write_boundary_lock is None:
            raise MotionInputError(
                "write_cancellation_event requires write_started_event and "
                "write_boundary_lock"
            )
    poll_interval = finite_number(
        poll_interval_seconds,
        "poll_interval_seconds",
    )
    if poll_interval <= 0:
        raise MotionInputError("poll_interval_seconds must be positive")
    if not isinstance(reset_input, bool):
        raise MotionInputError("reset_input must be boolean")
    if estop_callback is not None and not callable(estop_callback):
        raise MotionInputError(
            "serial exchange E-stop callback must be callable or None"
        )
    command_bytes = _serial_command_bytes(command)
    _require_open_serial_port(serial_port)
    _validate_write_lock(write_lock)
    _validate_write_lock(write_boundary_lock, "write_boundary_lock")
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError("serial connection does not expose a read timeout") from exc

    reset_input_method = None
    if reset_input:
        reset_input_method = getattr(
            serial_port,
            "reset_input_buffer",
            None,
        )
        if not callable(reset_input_method):
            reset_input_method = getattr(serial_port, "flushInput", None)
        if not callable(reset_input_method):
            raise TypeError(
                "serial connection does not support input-buffer reset"
            )
    readline = getattr(serial_port, "readline", None)
    read_until = getattr(serial_port, "read_until", None)
    read = getattr(serial_port, "read", None)
    if not (callable(read_until) or callable(readline)) or not callable(read):
        raise TypeError("serial connection does not satisfy the line-exchange contract")

    response = bytearray()
    operation_error = None
    transmitted = False

    def require_write_admission():
        try:
            write_cancelled = cancellation_event.is_set()
        except Exception as exc:
            raise MotionInputError(
                "cancellation_event could not be read before transmission"
            ) from exc
        if not isinstance(write_cancelled, bool):
            raise MotionInputError(
                "cancellation_event.is_set() must return a boolean"
            )
        if write_cancelled:
            raise SerialActivityRejected(
                "serial exchange cancelled before transmission"
            )
        if write_cancellation_event is not None:
            try:
                write_cancelled = write_cancellation_event.is_set()
            except Exception as exc:
                raise MotionInputError(
                    "write_cancellation_event could not be read before "
                    "transmission"
                ) from exc
            if not isinstance(write_cancelled, bool):
                raise MotionInputError(
                    "write_cancellation_event.is_set() must return a boolean"
                )
            if write_cancelled:
                raise SerialActivityRejected(
                    "serial exchange cancelled before transmission"
                )

    def read_followup_frame(initial_bytes=b""):
        frame = bytearray(initial_bytes)
        while True:
            try:
                cancelled = cancellation_event.is_set()
            except Exception as exc:
                raise ProtocolResponseError(
                    f"cancellation_event could not be read: {exc}"
                ) from exc
            if not isinstance(cancelled, bool):
                raise ProtocolResponseError(
                    "cancellation_event.is_set() must return a boolean"
                )
            if cancelled:
                _raise_quarantined_transport(
                    serial_port,
                    "serial exchange cancelled before a terminal response",
                )
            if len(frame) > MAX_RESPONSE_FRAME_LENGTH:
                raise ProtocolResponseError(
                    "controller response exceeds the size limit"
                )
            newline_index = frame.find(b"\n")
            if newline_index >= 0:
                if newline_index != len(frame) - 1:
                    raise ProtocolResponseError(
                        "controller returned trailing framed data"
                    )
                return decode_serial_response_line(frame)

            _set_serial_timeout(serial_port, poll_interval)
            remaining_size = MAX_RESPONSE_FRAME_LENGTH + 1 - len(frame)
            if callable(read_until):
                response_bytes = read_until(b"\n", remaining_size)
            else:
                response_bytes = readline()
            if not isinstance(response_bytes, (bytes, bytearray)):
                raise ProtocolResponseError(
                    "serial line reader returned a non-bytes response"
                )
            if not response_bytes:
                if not getattr(serial_port, "is_open", False):
                    raise ConnectionError(
                        "serial connection closed before a terminal response"
                    )
                continue
            frame.extend(response_bytes)

    try:
        _write_serial_bytes(
            serial_port,
            command_bytes,
            write_lock,
            reset_input=reset_input_method,
            write_admission_check=require_write_admission,
            write_started_event=write_started_event,
            write_boundary_lock=write_boundary_lock,
        )
        transmitted = True
        while True:
            try:
                cancelled = cancellation_event.is_set()
            except Exception as exc:
                raise ProtocolResponseError(
                    f"cancellation_event could not be read: {exc}"
                ) from exc
            if not isinstance(cancelled, bool):
                raise ProtocolResponseError(
                    "cancellation_event.is_set() must return a boolean"
                )
            if cancelled:
                _raise_quarantined_transport(
                    serial_port,
                    "serial exchange cancelled before a terminal response",
                )

            _set_serial_timeout(serial_port, poll_interval)
            remaining_size = MAX_RESPONSE_FRAME_LENGTH + 1 - len(response)
            if callable(read_until):
                response_bytes = read_until(b"\n", remaining_size)
            else:
                response_bytes = readline()
            if not isinstance(response_bytes, (bytes, bytearray)):
                raise ProtocolResponseError(
                    "serial line reader returned a non-bytes response"
                )
            if not response_bytes:
                if not getattr(serial_port, "is_open", False):
                    raise ConnectionError(
                        "serial connection closed before a terminal response"
                    )
                continue
            response.extend(response_bytes)
            if len(response) > MAX_RESPONSE_FRAME_LENGTH:
                raise ProtocolResponseError("controller response exceeds the size limit")
            newline_index = response.find(b"\n")
            if newline_index < 0:
                continue
            if newline_index != len(response) - 1:
                raise ProtocolResponseError(
                    "controller returned trailing framed data"
                )
            decoded = decode_serial_response_line(response)
            if estop_callback is not None:
                response_kind, response_position = (
                    _parse_controller_line_exchange_frame(decoded)
                )
                if response_kind != "terminal":
                    try:
                        published = estop_callback(response_position)
                    except Exception as exc:
                        raise ProtocolResponseError(
                            "serial exchange E-stop publication failed: "
                            f"{exc}"
                        ) from exc
                    if published is not True:
                        raise ProtocolResponseError(
                            "serial exchange E-stop callback did not confirm "
                            "publication"
                        )
                    if response_kind == "estop-admission":
                        _read_serial_quiet_boundary(
                            serial_port,
                            read,
                            max(
                                poll_interval,
                                CONTROL_POLL_INTERVAL_SECONDS,
                            ),
                            "controller response",
                        )
                        return decoded
                    second = read_followup_frame()
                    second_kind, _ = (
                        _parse_controller_line_exchange_frame(second)
                    )
                    if second_kind == "estop-event":
                        raise ProtocolResponseError(
                            "controller returned duplicate physical-stop events"
                        )
                    exchange_response = (
                        parse_controller_line_exchange_frames(
                            (decoded, second)
                        )
                    )
                    _read_serial_quiet_boundary(
                        serial_port,
                        read,
                        max(
                            poll_interval,
                            CONTROL_POLL_INTERVAL_SECONDS,
                        ),
                        "controller response",
                    )
                    return exchange_response.authoritative_response

                _set_serial_timeout(
                    serial_port,
                    max(
                        poll_interval,
                        CONTROL_POLL_INTERVAL_SECONDS,
                    ),
                )
                leading = read(1)
                if not isinstance(leading, (bytes, bytearray)):
                    raise ProtocolResponseError(
                        "controller response follow-up probe returned "
                        "non-bytes data"
                    )
                if not leading:
                    if not getattr(serial_port, "is_open", False):
                        raise ConnectionError(
                            "serial connection closed during the controller "
                            "response quiet boundary"
                        )
                    return decoded
                second = read_followup_frame(initial_bytes=leading)
                second_kind, second_position = (
                    _parse_controller_line_exchange_frame(second)
                )
                if second_kind != "estop-event":
                    raise ProtocolResponseError(
                        "controller returned an invalid post-terminal frame"
                    )
                try:
                    published = estop_callback(second_position)
                except Exception as exc:
                    raise ProtocolResponseError(
                        "serial exchange E-stop publication failed: "
                        f"{exc}"
                    ) from exc
                if published is not True:
                    raise ProtocolResponseError(
                        "serial exchange E-stop callback did not confirm "
                        "publication"
                    )
                exchange_response = parse_controller_line_exchange_frames(
                    (decoded, second)
                )
                _read_serial_quiet_boundary(
                    serial_port,
                    read,
                    max(
                        poll_interval,
                        CONTROL_POLL_INTERVAL_SECONDS,
                    ),
                    "controller response",
                )
                return exchange_response.authoritative_response
            _read_serial_quiet_boundary(
                serial_port,
                read,
                max(poll_interval, CONTROL_POLL_INTERVAL_SECONDS),
                "controller response",
            )
            return decoded
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except (MotionInputError, SerialActivityRejected) as exc:
        operation_error = exc
        if not transmitted:
            raise
        _raise_quarantined_transport(
            serial_port,
            f"serial exchange ended without a trusted response: {exc}",
            cause=exc,
        )
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            f"serial exchange ended without a trusted response: {exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def exchange_serial_line(
    serial_port,
    command,
    response_timeout_seconds,
    write_lock=None,
    control_event=None,
    control_command=None,
    control_ack_timeout_seconds=None,
    control_response_timeout_seconds=None,
    write_started_event=None,
    cancellation_event=None,
    write_boundary_lock=None,
    reset_input=True,
    interim_response_handler=None,
    interim_response_limit=None,
    estop_callback=None,
):
    """Perform a validated newline-delimited exchange on a serial-like object.

    A write-start event is set after admission and any requested input reset,
    immediately before the initial serial write call.
    """
    timeout = finite_number(response_timeout_seconds, "response_timeout_seconds")
    if timeout <= 0:
        raise MotionInputError("response_timeout_seconds must be positive")
    if not isinstance(reset_input, bool):
        raise MotionInputError("reset_input must be boolean")
    if estop_callback is not None and not callable(estop_callback):
        raise MotionInputError(
            "serial exchange E-stop callback must be callable or None"
        )
    if interim_response_handler is not None and not callable(
        interim_response_handler
    ):
        raise MotionInputError(
            "interim_response_handler must be callable or None"
        )
    if interim_response_handler is None:
        if interim_response_limit is not None:
            raise MotionInputError(
                "interim_response_limit requires an interim_response_handler"
            )
    elif (
        isinstance(interim_response_limit, bool)
        or not isinstance(interim_response_limit, int)
        or interim_response_limit <= 0
    ):
        raise MotionInputError(
            "interim_response_limit must be a positive integer"
        )
    command_bytes = _serial_command_bytes(command)
    _require_open_serial_port(serial_port)
    _validate_write_lock(write_lock)
    _validate_write_lock(write_boundary_lock, "write_boundary_lock")
    if cancellation_event is None:
        if write_boundary_lock is not None:
            raise MotionInputError(
                "write_boundary_lock requires a cancellation_event"
            )
    else:
        if not callable(getattr(cancellation_event, "is_set", None)):
            raise MotionInputError(
                "cancellation_event must satisfy the event contract"
            )
        if write_started_event is None or write_boundary_lock is None:
            raise MotionInputError(
                "cancellation_event requires write_started_event and "
                "write_boundary_lock"
            )

    if control_event is None:
        if (
            control_command is not None
            or control_ack_timeout_seconds is not None
            or control_response_timeout_seconds is not None
        ):
            raise MotionInputError(
                "control command and timeouts require a control_event"
            )
        control_bytes = None
        control_ack_timeout = None
        control_response_timeout = None
    else:
        if interim_response_handler is not None:
            raise MotionInputError(
                "interim responses are unsupported during live control"
            )
        if not callable(getattr(control_event, "is_set", None)):
            raise MotionInputError("control_event must satisfy the event contract")
        if control_command is None:
            raise MotionInputError("control_event requires a control_command")
        if control_ack_timeout_seconds is None:
            raise MotionInputError(
                "control_event requires a control_ack_timeout_seconds"
            )
        if control_response_timeout_seconds is None:
            raise MotionInputError(
                "control_event requires a control_response_timeout_seconds"
            )
        control_bytes = _serial_command_bytes(control_command)
        control_ack_timeout = finite_number(
            control_ack_timeout_seconds,
            "control_ack_timeout_seconds",
        )
        if control_ack_timeout <= 0:
            raise MotionInputError(
                "control_ack_timeout_seconds must be positive"
            )
        control_response_timeout = finite_number(
            control_response_timeout_seconds,
            "control_response_timeout_seconds",
        )
        if control_response_timeout <= 0:
            raise MotionInputError(
                "control_response_timeout_seconds must be positive"
            )

    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError("serial connection does not expose a read timeout") from exc

    operation_error = None
    try:
        return _exchange_serial_line_with_timeout(
            serial_port,
            command_bytes,
            timeout,
            write_lock,
            control_event,
            control_bytes,
            control_ack_timeout,
            control_response_timeout,
            write_started_event,
            cancellation_event,
            write_boundary_lock,
            reset_input,
            interim_response_handler,
            interim_response_limit,
            estop_callback,
        )
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except (MotionInputError, SerialActivityRejected) as exc:
        operation_error = exc
        raise
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            f"serial exchange ended without a trusted response: {exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def _exchange_serial_line_with_timeout(
    serial_port,
    command_bytes,
    timeout,
    write_lock,
    control_event,
    control_bytes,
    control_ack_timeout,
    control_response_timeout,
    write_started_event,
    cancellation_event,
    write_boundary_lock,
    reset_input,
    interim_response_handler,
    interim_response_limit,
    estop_callback,
):
    _set_serial_timeout(serial_port, timeout)

    reset_input_method = None
    if reset_input:
        reset_input_method = getattr(
            serial_port,
            "reset_input_buffer",
            None,
        )
        if not callable(reset_input_method):
            reset_input_method = getattr(serial_port, "flushInput", None)
        if not callable(reset_input_method):
            raise TypeError(
                "serial connection does not support input-buffer reset"
            )

    readline = getattr(serial_port, "readline", None)
    read_until = getattr(serial_port, "read_until", None)
    read = getattr(serial_port, "read", None)
    if not (callable(read_until) or callable(readline)) or not callable(read):
        raise TypeError("serial connection does not satisfy the line-exchange contract")

    response_buffer = bytearray()
    interim_response_count = 0
    control_sent = False
    control_attempted = False
    live_acknowledged = False
    initial_write_started = (
        write_started_event
        if write_started_event is not None
        else threading.Event()
    )
    if control_bytes is not None and not callable(
        getattr(initial_write_started, "is_set", None)
    ):
        raise MotionInputError(
            "live serial write-start event must satisfy the event contract"
        )

    def send_control_once():
        nonlocal control_attempted, control_sent
        if control_attempted:
            return
        control_attempted = True
        _write_serial_bytes(serial_port, control_bytes, write_lock)
        control_sent = True

    def control_is_requested():
        try:
            requested = control_event.is_set()
            if not isinstance(requested, bool):
                raise ProtocolResponseError(
                    "control_event.is_set() must return a boolean"
                )
            return requested
        except Exception as exc:
            if not control_attempted:
                try:
                    send_control_once()
                except Exception as stop_exc:
                    raise ProtocolResponseError(
                        f"{exc}; fail-safe stop write failed: {stop_exc}"
                    ) from exc
            raise ProtocolResponseError(
                f"control_event could not be read after live transmission: {exc}"
            ) from exc

    def probe_post_terminal_estop(
        terminal_response,
        followup_deadline,
        followup_timeout,
    ):
        remaining = followup_deadline - time.monotonic()
        if remaining < CONTROL_POLL_INTERVAL_SECONDS:
            _raise_quarantined_transport(
                serial_port,
                "controller response follow-up probe deadline expired",
                error_type=SerialTransportTimeout,
            )
        _set_serial_timeout(
            serial_port,
            CONTROL_POLL_INTERVAL_SECONDS,
        )
        leading = read(1)
        if not isinstance(leading, (bytes, bytearray)):
            raise ProtocolResponseError(
                "controller response follow-up probe returned non-bytes data"
            )
        if not leading:
            if not getattr(serial_port, "is_open", False):
                raise ConnectionError(
                    "serial connection closed during the controller "
                    "response quiet boundary"
                )
            return None
        second = _read_serial_framed_line(
            serial_port,
            readline,
            read_until,
            followup_deadline,
            followup_timeout,
            initial_bytes=leading,
        )
        second_kind, second_position = (
            _parse_controller_line_exchange_frame(second)
        )
        if second_kind != "estop-event":
            raise ProtocolResponseError(
                "controller returned an invalid post-terminal frame"
            )
        try:
            published = estop_callback(second_position)
        except Exception as exc:
            raise ProtocolResponseError(
                "serial exchange E-stop publication failed: "
                f"{exc}"
            ) from exc
        if published is not True:
            raise ProtocolResponseError(
                "serial exchange E-stop callback did not confirm publication"
            )
        exchange_response = parse_controller_line_exchange_frames(
            (terminal_response, second)
        )
        _read_serial_quiet_boundary(
            serial_port,
            read,
            followup_deadline - time.monotonic(),
            "controller response",
        )
        return exchange_response.authoritative_response

    def require_initial_write_admission():
        if cancellation_event is not None:
            try:
                cancelled = cancellation_event.is_set()
            except Exception as exc:
                raise MotionInputError(
                    "cancellation_event could not be read before transmission"
                ) from exc
            if not isinstance(cancelled, bool):
                raise MotionInputError(
                    "cancellation_event.is_set() must return a boolean"
                )
            if cancelled:
                raise SerialActivityRejected(
                    "serial exchange cancelled before transmission"
                )
        if control_bytes is not None:
            try:
                requested = control_event.is_set()
            except Exception as exc:
                raise MotionInputError(
                    "control_event could not be read before live transmission"
                ) from exc
            if not isinstance(requested, bool):
                raise MotionInputError(
                    "control_event.is_set() must return a boolean"
                )
            if requested:
                raise SerialActivityRejected(
                    "live serial exchange stopped before transmission"
                )

    try:
        _write_serial_bytes(
            serial_port,
            command_bytes,
            write_lock,
            reset_input=reset_input_method,
            write_admission_check=require_initial_write_admission,
            write_started_event=(
                initial_write_started
                if control_bytes is not None or write_started_event is not None
                else None
            ),
            write_boundary_lock=write_boundary_lock,
        )
    except Exception as exc:
        if control_bytes is not None and initial_write_started.is_set():
            try:
                send_control_once()
            except Exception as stop_exc:
                raise ProtocolResponseError(
                    f"{exc}; fail-safe stop write failed: {stop_exc}"
                ) from exc
            raise ProtocolResponseError(
                f"live command write failed after transmission started: {exc}"
            ) from exc
        raise

    active_timeout = control_ack_timeout if control_bytes is not None else timeout
    deadline = time.monotonic() + active_timeout

    while True:
        if (
            control_bytes is not None
            and not control_sent
            and control_is_requested()
        ):
            send_control_once()
            if live_acknowledged:
                active_timeout = control_response_timeout
                deadline = time.monotonic() + control_response_timeout

        if deadline is None:
            read_timeout = CONTROL_POLL_INTERVAL_SECONDS
        else:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if control_bytes is not None and not control_sent:
                    try:
                        send_control_once()
                    except Exception as exc:
                        _raise_quarantined_timeout(serial_port, active_timeout, exc)
                    _raise_quarantined_timeout(serial_port, active_timeout)
                _raise_quarantined_timeout(serial_port, active_timeout)
            read_timeout = remaining
            if control_bytes is not None and not control_sent:
                read_timeout = min(read_timeout, CONTROL_POLL_INTERVAL_SECONDS)
        try:
            _set_serial_timeout(serial_port, read_timeout)

            if callable(read_until):
                response_bytes = read_until(
                    b"\n",
                    MAX_RESPONSE_FRAME_LENGTH + 1 - len(response_buffer),
                )
            else:
                response_bytes = readline()
            if not isinstance(response_bytes, (bytes, bytearray)):
                raise ProtocolResponseError(
                    "serial line reader returned a non-bytes response"
                )
            if not response_bytes:
                if not getattr(serial_port, "is_open", False):
                    raise ConnectionError(
                        "serial connection closed during live response ownership"
                    )
                if control_bytes is None:
                    _raise_quarantined_timeout(serial_port, timeout)
                continue

            response_buffer.extend(response_bytes)
            if len(response_buffer) > MAX_RESPONSE_FRAME_LENGTH:
                raise ProtocolResponseError("controller response exceeds the size limit")
            if not response_buffer.endswith(b"\n"):
                continue
            response = decode_serial_response_line(
                response_buffer,
                allow_empty=control_bytes is not None,
            )
            response_buffer.clear()
            if response:
                response_kind = "terminal"
                response_position = None
                if estop_callback is not None:
                    response_kind, response_position = (
                        _parse_controller_line_exchange_frame(response)
                    )
                if response_kind != "terminal":
                    try:
                        published = estop_callback(response_position)
                    except Exception as exc:
                        raise ProtocolResponseError(
                            "serial exchange E-stop publication failed: "
                            f"{exc}"
                        ) from exc
                    if published is not True:
                        raise ProtocolResponseError(
                            "serial exchange E-stop callback did not confirm "
                            "publication"
                        )
                    if response_kind == "estop-admission":
                        quiet_timeout = CONTROL_POLL_INTERVAL_SECONDS
                        if deadline is not None:
                            quiet_timeout = deadline - time.monotonic()
                        _read_serial_quiet_boundary(
                            serial_port,
                            read,
                            quiet_timeout,
                            "controller response",
                        )
                        return response

                    if deadline is None:
                        followup_timeout = control_response_timeout
                        followup_deadline = (
                            time.monotonic() + control_response_timeout
                        )
                    else:
                        followup_timeout = active_timeout
                        followup_deadline = deadline
                    second = _read_serial_framed_line(
                        serial_port,
                        readline,
                        read_until,
                        followup_deadline,
                        followup_timeout,
                    )
                    second_kind, _ = (
                        _parse_controller_line_exchange_frame(second)
                    )
                    if second_kind == "estop-event":
                        raise ProtocolResponseError(
                            "controller returned duplicate physical-stop events"
                        )
                    exchange_response = (
                        parse_controller_line_exchange_frames(
                            (response, second)
                        )
                    )
                    _read_serial_quiet_boundary(
                        serial_port,
                        read,
                        followup_deadline - time.monotonic(),
                        "controller response",
                    )
                    return exchange_response.authoritative_response

                if interim_response_handler is not None:
                    try:
                        consumed = interim_response_handler(response)
                    except Exception as exc:
                        raise ProtocolResponseError(
                            "interim response handler failed after "
                            f"transmission: {exc}"
                        ) from exc
                    if not isinstance(consumed, bool):
                        raise ProtocolResponseError(
                            "interim response handler must return a boolean"
                        )
                    if consumed:
                        interim_response_count += 1
                        if interim_response_count > interim_response_limit:
                            raise ProtocolResponseError(
                                "controller exceeded the interim response limit"
                            )
                        continue
                controller_estop = _is_physical_estop_position_response(response)
                if (
                    control_bytes is not None
                    and live_acknowledged
                    and not control_sent
                    and not controller_estop
                    and estop_callback is not None
                ):
                    followup_timeout = control_response_timeout
                    followup_deadline = (
                        time.monotonic() + followup_timeout
                    )
                    paired_estop = probe_post_terminal_estop(
                        response,
                        followup_deadline,
                        followup_timeout,
                    )
                    if paired_estop is not None:
                        return paired_estop
                if control_bytes is not None and not controller_estop:
                    if not live_acknowledged:
                        if not control_sent:
                            try:
                                send_control_once()
                            except Exception as exc:
                                raise ProtocolResponseError(
                                    "controller returned terminal data before live "
                                    "acknowledgement and the fail-safe stop write "
                                    f"failed: {exc}"
                                ) from exc
                        raise ProtocolResponseError(
                            "controller returned terminal data before live "
                            "acknowledgement"
                        )
                    if not control_sent:
                        try:
                            send_control_once()
                        except Exception as exc:
                            raise ProtocolResponseError(
                                "controller returned terminal data before live stop "
                                f"and the fail-safe stop write failed: {exc}"
                            ) from exc
                        raise ProtocolResponseError(
                            "controller returned terminal data before live stop"
                        )
                if estop_callback is not None:
                    if deadline is None:
                        raise ProtocolResponseError(
                            "controller returned terminal data without a "
                            "bounded response deadline"
                        )
                    paired_estop = probe_post_terminal_estop(
                        response,
                        deadline,
                        active_timeout,
                    )
                    if paired_estop is not None:
                        return paired_estop
                quiet_timeout = CONTROL_POLL_INTERVAL_SECONDS
                if deadline is not None:
                    quiet_timeout = deadline - time.monotonic()
                _read_serial_quiet_boundary(
                    serial_port,
                    read,
                    quiet_timeout,
                    "controller response",
                )
                return response
            if control_bytes is None:
                raise ProtocolResponseError("controller returned a blank response line")
            if live_acknowledged:
                raise ProtocolResponseError(
                    "controller returned a duplicate live acknowledgement"
                )
            live_acknowledged = True
            if control_sent:
                active_timeout = control_response_timeout
                deadline = time.monotonic() + control_response_timeout
            else:
                deadline = None
        except (SerialTransportQuarantinedError, SerialTransportTimeout):
            raise
        except Exception as exc:
            if control_bytes is not None and not control_attempted:
                try:
                    send_control_once()
                except Exception as stop_exc:
                    raise ProtocolResponseError(
                        f"{exc}; fail-safe stop write failed: {stop_exc}"
                    ) from exc
            raise


def finite_number(value, field_name):
    if isinstance(value, bool):
        raise MotionInputError(f"{field_name} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MotionInputError(f"{field_name} must be numeric") from exc
    if not math.isfinite(number):
        raise MotionInputError(f"{field_name} must be finite")
    return number


def controller_number(value, field_name):
    number = finite_number(value, field_name)
    if abs(number) > CONTROLLER_FLOAT_MAX:
        raise MotionInputError(
            f"{field_name} exceeds the controller's finite float range"
        )
    try:
        encoded = struct.unpack(">f", struct.pack(">f", number))[0]
    except (OverflowError, struct.error) as exc:
        raise MotionInputError(
            f"{field_name} cannot be represented by the controller"
        ) from exc
    if not math.isfinite(encoded) or (number != 0 and encoded == 0):
        raise MotionInputError(
            f"{field_name} cannot be represented by the controller"
        )
    return number


def _controller_float(value, field_name):
    number = controller_number(value, field_name)
    try:
        encoded = struct.unpack(">f", struct.pack(">f", number))[0]
    except (OverflowError, struct.error) as exc:
        raise MotionInputError(
            f"{field_name} cannot be represented by the controller"
        ) from exc
    if not math.isfinite(encoded):
        raise MotionInputError(
            f"{field_name} cannot be represented by the controller"
        )
    return encoded


def controller_degree_to_native_radians(value, field_name):
    degrees = _controller_float(value, field_name)
    radians = _controller_float(
        degrees * CONTROLLER_RADIANS_PER_DEGREE,
        f"{field_name} native radians",
    )
    controller_number(
        radians / CONTROLLER_RADIANS_PER_DEGREE,
        f"{field_name} native degrees",
    )
    return radians


def _validate_controller_degree_value(value, field_name):
    controller_degree_to_native_radians(value, field_name)
    return _controller_float(value, field_name)


def _controller_float_product(left, right, field_name):
    return _controller_float(left * right, field_name)


def controller_ratio(numerator, denominator, field_name):
    numerator_float = _controller_float(numerator, f"{field_name} numerator")
    denominator_float = _controller_float(
        denominator,
        f"{field_name} denominator",
    )
    if denominator_float == 0:
        raise MotionInputError(f"{field_name} denominator must be non-zero")
    return _controller_float(
        numerator_float / denominator_float,
        field_name,
    )


def controller_protocol_decimal(value, field_name):
    encoded = _controller_float(value, field_name)
    if encoded == 0:
        return "0"
    return format(encoded, ".46f").rstrip("0").rstrip(".")


def primary_shutdown_position(home_reference):
    if not isinstance(home_reference, PrimaryHomeReference):
        raise MotionInputError(
            "shutdown position requires a controller home reference"
        )
    missing_axes = tuple(
        f"J{axis + 1}"
        for axis in (1, 2)
        if not home_reference.valid[axis]
    )
    if missing_axes:
        raise MotionInputError(
            "shutdown position requires homing "
            + " and ".join(missing_axes)
            + " under the active controller frame"
        )
    target = list(PRIMARY_START_POSITION)
    target[1:3] = home_reference.positions[1:3]
    return tuple(target)


def _finite_tuple(values, expected_length, field_name):
    if isinstance(values, (str, bytes)):
        raise MotionInputError(f"{field_name} must be a numeric sequence")
    try:
        iterator = iter(values)
    except TypeError as exc:
        raise MotionInputError(f"{field_name} must be a numeric sequence") from exc

    normalized = []
    for index in range(expected_length + 1):
        try:
            value = next(iterator)
        except StopIteration:
            break
        normalized.append(finite_number(value, f"{field_name}[{index}]"))
    if len(normalized) != expected_length:
        raise MotionInputError(
            f"{field_name} must contain {expected_length} values; got {len(normalized)}"
        )
    return tuple(normalized)


def _optional_finite_tuple(values, expected_length, field_name):
    if isinstance(values, (str, bytes)):
        raise MotionInputError(
            f"{field_name} must be a numeric-or-empty sequence"
        )
    try:
        iterator = iter(values)
    except TypeError as exc:
        raise MotionInputError(
            f"{field_name} must be a numeric-or-empty sequence"
        ) from exc

    normalized = []
    for index in range(expected_length + 1):
        try:
            value = next(iterator)
        except StopIteration:
            break
        normalized.append(
            None
            if value is None
            else finite_number(value, f"{field_name}[{index}]")
        )
    if len(normalized) != expected_length:
        raise MotionInputError(
            f"{field_name} must contain {expected_length} values; "
            f"got {len(normalized)}"
        )
    if not any(value is not None for value in normalized):
        raise MotionInputError(f"{field_name} must contain at least one target")
    return tuple(normalized)


def _nonnegative_integer_tuple(values, expected_length, field_name):
    if isinstance(values, (str, bytes)):
        raise MotionInputError(f"{field_name} must be an integer sequence")
    try:
        iterator = iter(values)
    except TypeError as exc:
        raise MotionInputError(
            f"{field_name} must be an integer sequence"
        ) from exc

    normalized = []
    for index in range(expected_length + 1):
        try:
            value = next(iterator)
        except StopIteration:
            break
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise MotionInputError(
                f"{field_name}[{index}] must be a non-negative integer"
            )
        normalized.append(value)
    if len(normalized) != expected_length:
        raise MotionInputError(
            f"{field_name} must contain {expected_length} values; "
            f"got {len(normalized)}"
        )
    return tuple(normalized)


def _protocol_number(value):
    return controller_protocol_decimal(value, "protocol value")


@dataclass(frozen=True)
class MotionProfile:
    speed_prefix: str
    speed: float
    acceleration: float
    deceleration: float
    ramp: float
    wrist_config: str
    loop_mode: str

    def __post_init__(self):
        if not isinstance(self.speed_prefix, str):
            raise MotionInputError("speed_prefix must be text")
        if not isinstance(self.wrist_config, str):
            raise MotionInputError("wrist_config must be text")
        if not isinstance(self.loop_mode, str):
            raise MotionInputError("loop_mode must be text")
        if self.speed_prefix not in ("Sp", "Ss"):
            raise MotionInputError("joint speed prefix must be 'Sp' or 'Ss'")

        speed = _controller_float(self.speed, "speed")
        acceleration = _controller_float(self.acceleration, "acceleration")
        deceleration = _controller_float(self.deceleration, "deceleration")
        ramp = _controller_float(self.ramp, "ramp")

        if speed <= 0 or (self.speed_prefix == "Sp" and speed > 100):
            raise MotionInputError("percent speed must be in (0, 100], and seconds must be positive")
        if not 0 < acceleration <= 100:
            raise MotionInputError("acceleration must be in (0, 100]")
        if not 0 < deceleration < 100:
            raise MotionInputError("deceleration must be in (0, 100)")
        if acceleration + deceleration > 100:
            raise MotionInputError(
                "acceleration and deceleration must not overlap"
            )
        if not 0 < ramp <= CONTROLLER_MAXIMUM_RAMP_PERCENT:
            raise MotionInputError(
                "ramp must be in (0, "
                f"{CONTROLLER_MAXIMUM_RAMP_PERCENT:g}]"
            )
        if self.wrist_config not in ("N", "F"):
            raise MotionInputError("wrist_config must be 'N' or 'F'")
        if re.fullmatch(r"[01]{6}", self.loop_mode) is None:
            raise MotionInputError("loop_mode must contain six binary digits")

        object.__setattr__(self, "speed", speed)
        object.__setattr__(self, "acceleration", acceleration)
        object.__setattr__(self, "deceleration", deceleration)
        object.__setattr__(self, "ramp", ramp)

    def protocol_suffix(self):
        return (
            f"{self.speed_prefix}{_protocol_number(self.speed)}"
            f"Ac{_protocol_number(self.acceleration)}"
            f"Dc{_protocol_number(self.deceleration)}"
            f"Rm{_protocol_number(self.ramp)}"
            f"W{self.wrist_config}Lm{self.loop_mode}\n"
        )


@dataclass(frozen=True)
class ControllerJointCalibration:
    negative_limits: Tuple[float, ...]
    positive_limits: Tuple[float, ...]
    steps_per_unit: Tuple[float, ...]

    def __post_init__(self):
        negative_limits = _finite_tuple(
            self.negative_limits,
            JOINT_COUNT,
            "negative_limits",
        )
        positive_limits = _finite_tuple(
            self.positive_limits,
            JOINT_COUNT,
            "positive_limits",
        )
        steps_per_unit = _finite_tuple(
            self.steps_per_unit,
            JOINT_COUNT,
            "steps_per_unit",
        )

        for axis, (negative, positive, scale) in enumerate(
            zip(negative_limits, positive_limits, steps_per_unit),
            start=1,
        ):
            negative_float = _controller_float(
                negative,
                f"J{axis} negative limit",
            )
            positive_float = _controller_float(
                positive,
                f"J{axis} positive limit",
            )
            scale_float = _controller_float(scale, f"J{axis} steps per unit")
            if negative_float < 0 or positive_float < 0 or scale_float <= 0:
                raise MotionInputError(
                    f"J{axis} calibration limits and step scale are out of range"
                )
            travel_float = _controller_float(
                negative_float + positive_float,
                f"J{axis} calibrated travel",
            )
            step_limit = _controller_float_product(
                travel_float,
                scale_float,
                f"J{axis} calibrated step limit",
            )
            if step_limit > CONTROLLER_SIGNED_INT_MAX:
                raise MotionInputError(
                    f"J{axis} calibrated travel exceeds the controller step range"
                )

        object.__setattr__(self, "negative_limits", negative_limits)
        object.__setattr__(self, "positive_limits", positive_limits)
        object.__setattr__(self, "steps_per_unit", steps_per_unit)

    def validate_axis_positions(self, positions):
        if not isinstance(positions, Mapping) or not positions:
            raise MotionInputError(
                "axis positions must be a non-empty mapping"
            )
        normalized = {}
        for axis, position in positions.items():
            if (
                isinstance(axis, bool)
                or not isinstance(axis, int)
                or axis < 1
                or axis > JOINT_COUNT
            ):
                raise MotionInputError("axis position index must be in [1, 9]")
            if axis in normalized:
                raise MotionInputError(f"J{axis} position is duplicated")
            normalized[axis] = finite_number(position, f"J{axis} position")

        for axis, position in normalized.items():
            negative = self.negative_limits[axis - 1]
            positive = self.positive_limits[axis - 1]
            scale = self.steps_per_unit[axis - 1]
            position_float = _controller_float(position, f"J{axis} position")
            negative_float = _controller_float(negative, f"J{axis} negative limit")
            positive_float = _controller_float(positive, f"J{axis} positive limit")
            scale_float = _controller_float(scale, f"J{axis} steps per unit")
            if position_float < -negative_float or position_float > positive_float:
                raise MotionInputError(
                    f"J{axis} position is outside the calibrated limits"
                )
            shifted_position = _controller_float(
                position_float + negative_float,
                f"J{axis} shifted position",
            )
            future_step = _controller_float_product(
                shifted_position,
                scale_float,
                f"J{axis} future step position",
            )
            if future_step < 0 or future_step > CONTROLLER_SIGNED_INT_MAX:
                raise MotionInputError(
                    f"J{axis} position exceeds the controller step range"
                )
        return normalized

    def validate_positions(self, positions):
        normalized = _finite_tuple(positions, JOINT_COUNT, "positions")
        self.validate_axis_positions({
            axis: position
            for axis, position in enumerate(normalized, start=1)
        })
        return normalized


def build_robot_joint_command(positions, profile, calibration):
    if not isinstance(calibration, ControllerJointCalibration):
        raise MotionInputError(
            "calibration must be a ControllerJointCalibration"
        )
    normalized = calibration.validate_positions(positions)
    if not isinstance(profile, MotionProfile):
        raise MotionInputError("profile must be a MotionProfile")
    fields = (
        ("A", normalized[0]),
        ("B", normalized[1]),
        ("C", normalized[2]),
        ("D", normalized[3]),
        ("E", normalized[4]),
        ("F", normalized[5]),
        ("J7", normalized[6]),
        ("J8", normalized[7]),
        ("J9", normalized[8]),
    )
    return "RJ" + "".join(label + _protocol_number(value) for label, value in fields) + profile.protocol_suffix()


def build_virtual_joint_command(positions, profile):
    normalized = _finite_tuple(positions, 6, "positions")
    if not isinstance(profile, MotionProfile):
        raise MotionInputError("profile must be a MotionProfile")
    fields = zip(("A", "B", "C", "D", "E", "F"), normalized)
    return "RJ" + "".join(label + _protocol_number(value) for label, value in fields) + profile.protocol_suffix()


@dataclass(frozen=True)
class JointMove:
    positions: Tuple[float, ...]
    profile: MotionProfile
    calibration: ControllerJointCalibration

    def __post_init__(self):
        if not isinstance(self.calibration, ControllerJointCalibration):
            raise MotionInputError(
                "calibration must be a ControllerJointCalibration"
            )
        object.__setattr__(
            self,
            "positions",
            self.calibration.validate_positions(self.positions),
        )
        if not isinstance(self.profile, MotionProfile):
            raise MotionInputError("profile must be a MotionProfile")

    @property
    def command(self):
        return build_robot_joint_command(
            self.positions,
            self.profile,
            self.calibration,
        )


@dataclass(frozen=True)
class CommandedJointTrajectory:
    """Display-only RJ estimate for firmware that reports terminal position."""

    start_positions: Tuple[float, ...]
    target_positions: Tuple[float, ...]
    estimated_terminal_positions: Tuple[float, ...]
    step_deltas: Tuple[int, ...]
    high_steps: int
    acceleration_steps: float
    cruise_steps: float
    deceleration_steps: float
    average_distribution_delay_microseconds: float
    cruise_delay_microseconds: float
    start_delay_microseconds: float
    end_delay_microseconds: float
    acceleration_duration_seconds: float
    cruise_duration_seconds: float
    deceleration_duration_seconds: float
    duration_seconds: float

    def __post_init__(self):
        start_positions = _finite_tuple(
            self.start_positions,
            JOINT_COUNT,
            "trajectory start positions",
        )
        target_positions = _finite_tuple(
            self.target_positions,
            JOINT_COUNT,
            "trajectory target positions",
        )
        estimated_terminal_positions = _finite_tuple(
            self.estimated_terminal_positions,
            JOINT_COUNT,
            "trajectory estimated terminal positions",
        )
        step_deltas = _nonnegative_integer_tuple(
            self.step_deltas,
            JOINT_COUNT,
            "trajectory step deltas",
        )
        if (
            isinstance(self.high_steps, bool)
            or not isinstance(self.high_steps, int)
            or self.high_steps < 0
            or self.high_steps != max(step_deltas)
        ):
            raise MotionInputError(
                "trajectory coordinated step count is invalid"
            )

        numeric_fields = (
            "acceleration_steps",
            "cruise_steps",
            "deceleration_steps",
            "average_distribution_delay_microseconds",
            "cruise_delay_microseconds",
            "start_delay_microseconds",
            "end_delay_microseconds",
            "acceleration_duration_seconds",
            "cruise_duration_seconds",
            "deceleration_duration_seconds",
            "duration_seconds",
        )
        numeric_values = {}
        for field_name in numeric_fields:
            value = finite_number(
                getattr(self, field_name),
                f"trajectory {field_name}",
            )
            if value < 0:
                raise MotionInputError(
                    f"trajectory {field_name} must be non-negative"
                )
            numeric_values[field_name] = value

        phase_steps = (
            numeric_values["acceleration_steps"]
            + numeric_values["cruise_steps"]
            + numeric_values["deceleration_steps"]
        )
        step_tolerance = max(0.001, self.high_steps * 1e-6)
        if not math.isclose(
            phase_steps,
            self.high_steps,
            rel_tol=0.0,
            abs_tol=step_tolerance,
        ):
            raise MotionInputError(
                "trajectory timing regions do not span the coordinated move"
            )

        phase_duration = (
            numeric_values["acceleration_duration_seconds"]
            + numeric_values["cruise_duration_seconds"]
            + numeric_values["deceleration_duration_seconds"]
        )
        if not math.isclose(
            phase_duration,
            numeric_values["duration_seconds"],
            rel_tol=1e-9,
            abs_tol=1e-12,
        ):
            raise MotionInputError(
                "trajectory phase durations do not match the total duration"
            )
        if self.high_steps == 0:
            if any(numeric_values.values()):
                raise MotionInputError(
                    "zero-step trajectory must have zero timing values"
                )
        elif (
            numeric_values["cruise_delay_microseconds"] <= 0
            or numeric_values["start_delay_microseconds"] <= 0
            or numeric_values["end_delay_microseconds"] <= 0
            or numeric_values["duration_seconds"] <= 0
        ):
            raise MotionInputError(
                "moving trajectory requires positive timing values"
            )

        object.__setattr__(self, "start_positions", start_positions)
        object.__setattr__(self, "target_positions", target_positions)
        object.__setattr__(
            self,
            "estimated_terminal_positions",
            estimated_terminal_positions,
        )
        object.__setattr__(self, "step_deltas", step_deltas)
        for field_name, value in numeric_values.items():
            object.__setattr__(self, field_name, value)

    def progress_at(self, elapsed_seconds):
        elapsed = finite_number(elapsed_seconds, "trajectory elapsed time")
        if elapsed < 0:
            raise MotionInputError(
                "trajectory elapsed time must be non-negative"
            )
        if self.high_steps == 0:
            return 0.0
        if elapsed >= self.duration_seconds:
            return 1.0

        elapsed_microseconds = elapsed * 1_000_000.0
        acceleration_duration = (
            self.acceleration_duration_seconds * 1_000_000.0
        )
        cruise_duration = self.cruise_duration_seconds * 1_000_000.0

        if elapsed_microseconds < acceleration_duration:
            phase_steps = _linear_delay_phase_steps(
                elapsed_microseconds,
                self.acceleration_steps,
                self.start_delay_microseconds,
                self.cruise_delay_microseconds,
            )
            completed_steps = phase_steps
        elif elapsed_microseconds < acceleration_duration + cruise_duration:
            cruise_elapsed = elapsed_microseconds - acceleration_duration
            completed_steps = self.acceleration_steps + (
                cruise_elapsed / self.cruise_delay_microseconds
            )
        else:
            deceleration_elapsed = (
                elapsed_microseconds
                - acceleration_duration
                - cruise_duration
            )
            phase_steps = _linear_delay_phase_steps(
                deceleration_elapsed,
                self.deceleration_steps,
                self.cruise_delay_microseconds,
                self.end_delay_microseconds,
            )
            completed_steps = (
                self.acceleration_steps
                + self.cruise_steps
                + phase_steps
            )

        return min(1.0, max(0.0, completed_steps / self.high_steps))

    def positions_at(self, elapsed_seconds):
        progress = self.progress_at(elapsed_seconds)
        return tuple(
            start + (target - start) * progress
            for start, target in zip(
                self.start_positions,
                self.estimated_terminal_positions,
            )
        )


def _linear_delay_phase_steps(
    elapsed_microseconds,
    phase_steps,
    initial_delay_microseconds,
    final_delay_microseconds,
):
    if phase_steps <= 0 or elapsed_microseconds <= 0:
        return 0.0
    phase_duration = (
        phase_steps
        * (initial_delay_microseconds + final_delay_microseconds)
        * 0.5
    )
    if elapsed_microseconds >= phase_duration:
        return phase_steps

    delay_change = final_delay_microseconds - initial_delay_microseconds
    if delay_change == 0:
        return min(
            phase_steps,
            elapsed_microseconds / initial_delay_microseconds,
        )

    quadratic = delay_change / (2.0 * phase_steps)
    discriminant = (
        initial_delay_microseconds * initial_delay_microseconds
        + 4.0 * quadratic * elapsed_microseconds
    )
    if discriminant < 0:
        raise MotionInputError("trajectory phase timing is not invertible")
    denominator = initial_delay_microseconds + math.sqrt(discriminant)
    if denominator <= 0:
        raise MotionInputError("trajectory phase timing is not invertible")
    return min(
        phase_steps,
        max(0.0, 2.0 * elapsed_microseconds / denominator),
    )


def _controller_calibrated_step(position, calibration, axis):
    position_float = _controller_float(position, f"J{axis} position")
    negative_float = _controller_float(
        calibration.negative_limits[axis - 1],
        f"J{axis} negative limit",
    )
    scale_float = _controller_float(
        calibration.steps_per_unit[axis - 1],
        f"J{axis} steps per unit",
    )
    shifted_position = _controller_float(
        position_float + negative_float,
        f"J{axis} shifted position",
    )
    future_step = _controller_float_product(
        shifted_position,
        scale_float,
        f"J{axis} future step position",
    )
    if future_step < 0 or future_step > CONTROLLER_SIGNED_INT_MAX:
        raise MotionInputError(
            f"J{axis} position exceeds the controller step range"
        )
    return int(future_step)


def _controller_position_from_step(step, calibration, axis):
    scale_float = _controller_float(
        calibration.steps_per_unit[axis - 1],
        f"J{axis} steps per unit",
    )
    negative_float = _controller_float(
        calibration.negative_limits[axis - 1],
        f"J{axis} negative limit",
    )
    zero_step_value = _controller_float_product(
        negative_float,
        scale_float,
        f"J{axis} zero step position",
    )
    if zero_step_value < 0 or zero_step_value > CONTROLLER_SIGNED_INT_MAX:
        raise MotionInputError(
            f"J{axis} zero step exceeds the controller range"
        )
    zero_step = int(zero_step_value)
    relative_step = step - zero_step
    return _controller_float(
        _controller_float(
            relative_step,
            f"J{axis} relative step position",
        )
        / scale_float,
        f"J{axis} estimated terminal position",
    )


def estimate_commanded_joint_trajectory(
    start_positions,
    move,
    minimum_step_delay_microseconds,
):
    """Estimate the coordinated RJ timing envelope without claiming telemetry."""

    if not isinstance(move, JointMove):
        raise MotionInputError("move must be a JointMove")
    start = move.calibration.validate_positions(start_positions)
    target = move.calibration.validate_positions(move.positions)
    minimum_delay = _controller_float(
        minimum_step_delay_microseconds,
        "controller minimum step delay",
    )
    if minimum_delay <= 0:
        raise MotionInputError(
            "controller minimum step delay must be positive"
        )

    start_steps = tuple(
        _controller_calibrated_step(position, move.calibration, axis)
        for axis, position in enumerate(start, start=1)
    )
    target_steps = tuple(
        _controller_calibrated_step(position, move.calibration, axis)
        for axis, position in enumerate(target, start=1)
    )
    step_deltas = tuple(
        abs(target_step - start_step)
        for start_step, target_step in zip(start_steps, target_steps)
    )
    high_steps = max(step_deltas)
    if high_steps == 0:
        return CommandedJointTrajectory(
            start_positions=start,
            target_positions=target,
            estimated_terminal_positions=start,
            step_deltas=step_deltas,
            high_steps=0,
            acceleration_steps=0.0,
            cruise_steps=0.0,
            deceleration_steps=0.0,
            average_distribution_delay_microseconds=0.0,
            cruise_delay_microseconds=0.0,
            start_delay_microseconds=0.0,
            end_delay_microseconds=0.0,
            acceleration_duration_seconds=0.0,
            cruise_duration_seconds=0.0,
            deceleration_duration_seconds=0.0,
            duration_seconds=0.0,
        )

    estimated_terminal_positions = tuple(
        (
            _controller_position_from_step(
                target_step,
                move.calibration,
                axis,
            )
            if step_delta > 0
            else start_position
        )
        for axis, (
            start_position,
            target_step,
            step_delta,
        ) in enumerate(
            zip(start, target_steps, step_deltas),
            start=1,
        )
    )

    high_steps_float = _controller_float(high_steps, "coordinated step count")
    acceleration_ratio = _controller_float(
        move.profile.acceleration / 100.0,
        "acceleration ratio",
    )
    deceleration_ratio = _controller_float(
        move.profile.deceleration / 100.0,
        "deceleration ratio",
    )
    acceleration_steps = _controller_float_product(
        high_steps_float,
        acceleration_ratio,
        "acceleration step count",
    )
    deceleration_steps = _controller_float_product(
        high_steps_float,
        deceleration_ratio,
        "deceleration step count",
    )
    cruise_steps = _controller_float(
        _controller_float(
            high_steps_float - acceleration_steps,
            "pre-deceleration step count",
        )
        - deceleration_steps,
        "cruise step count",
    )
    if cruise_steps < 0:
        raise MotionInputError(
            "controller timing regions overlap after float encoding"
        )

    ramp = max(move.profile.ramp, FIRMWARE_MINIMUM_RAMP_PERCENT)
    ramp_factor = _controller_float(
        ramp / FIRMWARE_MINIMUM_RAMP_PERCENT,
        "motion ramp factor",
    )
    acceleration_weight = _controller_float_product(
        acceleration_steps,
        _controller_float(1.0 + ramp_factor, "acceleration timing factor"),
        "acceleration timing weight",
    )
    deceleration_weight = _controller_float_product(
        deceleration_steps,
        _controller_float(1.0 + ramp_factor, "deceleration timing factor"),
        "deceleration timing weight",
    )
    ramp_weight = _controller_float(
        acceleration_weight + deceleration_weight,
        "ramp timing weight",
    )
    denominator = cruise_steps + ramp_weight * 0.5

    if move.profile.speed_prefix == "Ss":
        target_duration_microseconds = move.profile.speed * 1_000_000.0
        if denominator <= 0:
            cruise_delay = _controller_float(
                target_duration_microseconds / high_steps,
                "seconds-mode cruise delay",
            )
        else:
            cruise_delay = _controller_float(
                target_duration_microseconds / denominator,
                "seconds-mode cruise delay",
            )
        cruise_delay = max(cruise_delay, minimum_delay)
    else:
        speed_ratio = _controller_float(
            move.profile.speed / 100.0,
            "percent speed ratio",
        )
        cruise_delay = _controller_float(
            minimum_delay / speed_ratio,
            "percent-mode cruise delay",
        )

    start_delay = _controller_float_product(
        cruise_delay,
        ramp_factor,
        "motion start delay",
    )
    end_delay = _controller_float_product(
        cruise_delay,
        ramp_factor,
        "motion end delay",
    )
    average_distribution_delay = (
        FIRMWARE_DISTRIBUTION_DELAY_MICROSECONDS
        * sum(step_deltas)
        / high_steps
    )
    effective_delay_floor = minimum_delay + average_distribution_delay
    cruise_delay = max(cruise_delay, effective_delay_floor)
    start_delay = max(start_delay, effective_delay_floor)
    end_delay = max(end_delay, effective_delay_floor)
    for label, delay in (
        ("cruise", cruise_delay),
        ("start", start_delay),
        ("end", end_delay),
    ):
        if (
            delay <= 0
            or delay > CONTROLLER_MAXIMUM_PULSE_DELAY_MICROSECONDS
        ):
            raise MotionInputError(
                f"controller {label} delay is outside the firmware range"
            )

    acceleration_duration = (
        acceleration_steps * (start_delay + cruise_delay) * 0.5
    ) / 1_000_000.0
    cruise_duration = (
        cruise_steps * cruise_delay
    ) / 1_000_000.0
    deceleration_duration = (
        deceleration_steps * (cruise_delay + end_delay) * 0.5
    ) / 1_000_000.0
    duration = (
        acceleration_duration
        + cruise_duration
        + deceleration_duration
    )
    if not math.isfinite(duration) or duration <= 0:
        raise MotionInputError("estimated joint trajectory duration is invalid")

    return CommandedJointTrajectory(
        start_positions=start,
        target_positions=target,
        estimated_terminal_positions=estimated_terminal_positions,
        step_deltas=step_deltas,
        high_steps=high_steps,
        acceleration_steps=acceleration_steps,
        cruise_steps=cruise_steps,
        deceleration_steps=deceleration_steps,
        average_distribution_delay_microseconds=(
            average_distribution_delay
        ),
        cruise_delay_microseconds=cruise_delay,
        start_delay_microseconds=start_delay,
        end_delay_microseconds=end_delay,
        acceleration_duration_seconds=acceleration_duration,
        cruise_duration_seconds=cruise_duration,
        deceleration_duration_seconds=deceleration_duration,
        duration_seconds=duration,
    )


_NUMBER = r"-?(?:\d+(?:\.\d*)?|\.\d+)"
_NONNEGATIVE_NUMBER = r"(?:\d+(?:\.\d*)?|\.\d+)"


def _numeric_fields(*fields):
    return "".join(
        re.escape(marker) + rf"(?P<{name}>{_NUMBER})"
        for marker, name in fields
    )


_ROBOT_JOINT_FIELDS = _numeric_fields(
    ("A", "j1"),
    ("B", "j2"),
    ("C", "j3"),
    ("D", "j4"),
    ("E", "j5"),
    ("F", "j6"),
    ("J7", "j7"),
    ("J8", "j8"),
    ("J9", "j9"),
)
_VIRTUAL_JOINT_FIELDS = _numeric_fields(
    ("A", "j1"),
    ("B", "j2"),
    ("C", "j3"),
    ("D", "j4"),
    ("E", "j5"),
    ("F", "j6"),
)
_CARTESIAN_FIELDS = _numeric_fields(
    ("X", "x"),
    ("Y", "y"),
    ("Z", "z"),
    ("Rz", "rz"),
    ("Ry", "ry"),
    ("Rx", "rx"),
    ("J7", "j7"),
    ("J8", "j8"),
    ("J9", "j9"),
)
_VIRTUAL_CARTESIAN_FIELDS = _numeric_fields(
    ("X", "x"),
    ("Y", "y"),
    ("Z", "z"),
    ("Rz", "rz"),
    ("Ry", "ry"),
    ("Rx", "rx"),
)
_ARC_FIELDS = _numeric_fields(
    ("X", "x"),
    ("Y", "y"),
    ("Z", "z"),
    ("Rz", "rz"),
    ("Ry", "ry"),
    ("Rx", "rx"),
    ("Ex", "ex"),
    ("Ey", "ey"),
    ("Ez", "ez"),
    ("Tr", "tr"),
)
_CIRCLE_FIELDS = _numeric_fields(
    ("Cx", "cx"),
    ("Cy", "cy"),
    ("Cz", "cz"),
    ("Rz", "rz"),
    ("Ry", "ry"),
    ("Rx", "rx"),
    ("Bx", "bx"),
    ("By", "by"),
    ("Bz", "bz"),
    ("Px", "px"),
    ("Py", "py"),
    ("Pz", "pz"),
    ("Tr", "tr"),
)
_VALUE_FIELD = _numeric_fields(("V", "value"))
_DISTANCE_FIELD = rf"(?P<distance>{_NONNEGATIVE_NUMBER})"
_ROUNDING_FIELD = _numeric_fields(("", "rounding"))
_VISION_ROTATION_FIELD = _numeric_fields(("", "vision_rotation"))
_STANDARD_TIMING_PROFILE = re.compile(
    rf"S(?P<mode>[psm])(?P<speed>{_NUMBER})"
    rf"Ac(?P<acceleration>{_NUMBER})"
    rf"Dc(?P<deceleration>{_NUMBER})"
    rf"Rm(?P<ramp>{_NUMBER})(?=Rnd|W)"
)
_LEGACY_JOG_TIMING_PROFILE = re.compile(
    rf"S(?P<mode>[psm])(?P<speed>{_NUMBER})"
    rf"G(?P<acceleration>{_NUMBER})"
    rf"H(?P<deceleration>{_NUMBER})"
    rf"I(?P<ramp>{_NUMBER})(?=W)"
)
_FIELDLESS_COMMANDS = {
    "CP": re.compile(r"^CP\n\Z"),
    "PG": re.compile(r"^PGFn[ -~]+\n\Z"),
    "RP": re.compile(r"^RP\n\Z"),
}
_STANDARD_TIMING_ENVELOPES = {
    "RJ": re.compile(rf"^RJ{_ROBOT_JOINT_FIELDS}(?=S)"),
    **{
        opcode: re.compile(rf"^{opcode}{_CARTESIAN_FIELDS}(?=S)")
        for opcode in ("MG", "MJ", "ML", "MV", "WC", "WG")
    },
    "MA": re.compile(rf"^MA{_ARC_FIELDS}(?=S)"),
    "MC": re.compile(rf"^MC{_CIRCLE_FIELDS}(?=S)"),
    **{
        opcode: re.compile(rf"^{opcode}{_VALUE_FIELD}(?=S)")
        for opcode in ("LC", "LJ", "LT")
    },
}
_VIRTUAL_TIMING_ENVELOPES = {
    "RJ": re.compile(rf"^RJ{_VIRTUAL_JOINT_FIELDS}(?=S)"),
    "MJ": re.compile(rf"^MJ{_VIRTUAL_CARTESIAN_FIELDS}(?=S)"),
    "MV": re.compile(rf"^MV{_VIRTUAL_CARTESIAN_FIELDS}(?=S)"),
}
_LEGACY_TIMING_ENVELOPES = {
    "JT": re.compile(rf"^JT[XYZRPW][01]{_DISTANCE_FIELD}(?=S)"),
}
_STANDARD_TIMING_SUFFIXES = {
    **{
        opcode: re.compile(r"W[NFA]Lm[01]{6}\n\Z")
        for opcode in ("MA", "MC", "MG", "MJ", "RJ")
    },
    "ML": re.compile(
        rf"Rnd{_ROUNDING_FIELD}W[NFA]Lm[01]{{6}}Q0\n\Z"
    ),
    "MV": re.compile(
        rf"W[NFA]Vr{_VISION_ROTATION_FIELD}Lm[01]{{6}}\n\Z"
    ),
    "LJ": re.compile(r"WALm[01]{6}\n\Z"),
    **{
        opcode: re.compile(r"W[NFA]Lm[01]{6}\n\Z")
        for opcode in ("LC", "LT")
    },
    **{
        opcode: re.compile(
            rf"W[NFA]Lm[01]{{6}}"
            rf"Mi[0-9A-F]{{{CONTROLLER_MEDIA_ID_LENGTH}}}"
            rf"Fn[ -~]+\n\Z"
        )
        for opcode in ("WC", "WG")
    },
}
_SERIAL_TIMING_SUFFIXES = {
    **_STANDARD_TIMING_SUFFIXES,
    "RJ": re.compile(r"W[NFA]Lm[01]{6}(?:T1)?\n\Z"),
}
_LIVE_JOG_MAXIMUM_AXES = {
    "LC": 6,
    "LJ": JOINT_COUNT,
    "LT": 6,
}
_LEGACY_TIMING_SUFFIXES = {
    "JT": re.compile(r"W[NFA]Lm[01]{6}\n\Z"),
}
_POSITION_RESPONSE = re.compile(
    rf"^A(?P<j1>{_NUMBER})B(?P<j2>{_NUMBER})C(?P<j3>{_NUMBER})"
    rf"D(?P<j4>{_NUMBER})E(?P<j5>{_NUMBER})F(?P<j6>{_NUMBER})"
    rf"G(?P<x>{_NUMBER})H(?P<y>{_NUMBER})I(?P<z>{_NUMBER})"
    rf"J(?P<rz>{_NUMBER})K(?P<ry>{_NUMBER})L(?P<rx>{_NUMBER})"
    rf"M(?P<speed_violation>[01])N(?P<debug>(?:{_NUMBER})?)"
    rf"O(?P<flag>(?:EA|EB|EC[01]{{6}})?)"
    rf"P(?P<j7>{_NUMBER})Q(?P<j8>{_NUMBER})R(?P<j9>{_NUMBER})$"
)
_PRIMARY_HOME_REFERENCE_V1_RESPONSE = re.compile(
    r"^A(?P<j1_valid>[01])"
    r"B(?P<j1_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"C(?P<j2_valid>[01])"
    r"D(?P<j2_millidegrees>(?:0|-?[1-9][0-9]*))$"
)
_PRIMARY_HOME_REFERENCE_V2_RESPONSE = re.compile(
    r"^A(?P<j1_valid>[01])"
    r"B(?P<j1_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"C(?P<j2_valid>[01])"
    r"D(?P<j2_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"E(?P<j3_valid>[01])"
    r"F(?P<j3_millidegrees>(?:0|-?[1-9][0-9]*))$"
)
_JOINT_TELEMETRY_RESPONSE = re.compile(
    rf"^{re.escape(JOINT_TELEMETRY_PREFIX)}"
    r"(?P<j1_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"B(?P<j2_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"C(?P<j3_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"D(?P<j4_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"E(?P<j5_millidegrees>(?:0|-?[1-9][0-9]*))"
    r"F(?P<j6_millidegrees>(?:0|-?[1-9][0-9]*))$"
)
_JOINT_MOTION_ERROR_RESPONSE = re.compile(r"^(?:ER|EL[01]{9})$")


@dataclass(frozen=True)
class CommandTiming:
    mode: str
    speed: float
    acceleration: float
    deceleration: float
    ramp: float


def _canonicalize_protocol_numbers(command, *matches):
    replacements = []
    encoded_values = {}
    for match in matches:
        for name, value in match.groupdict().items():
            if name == "mode" or value is None:
                continue
            canonical = controller_protocol_decimal(
                value,
                f"command field {name}",
            )
            replacements.append((match.start(name), match.end(name), canonical))
            encoded_values[name] = float(canonical)

    normalized = command
    for start, end, canonical in sorted(replacements, reverse=True):
        normalized = normalized[:start] + canonical + normalized[end:]
    _serial_command_bytes(normalized)
    return normalized, encoded_values


def _validate_motion_angle_fields(command, opcode, encoded_values):
    field_names = []
    if opcode in ("MA", "MC", "MG", "MJ", "ML", "MV", "WC", "WG"):
        field_names.extend(("rz", "ry", "rx"))
    if opcode == "MV":
        field_names.append("vision_rotation")
    if opcode == "JT" and command[2:3] in ("W", "P", "R"):
        field_names.append("distance")

    for field_name in field_names:
        if field_name not in encoded_values:
            raise MotionInputError(
                f"{opcode} command is missing angular field {field_name}"
            )
        _validate_controller_degree_value(
            encoded_values[field_name],
            f"command field {field_name}",
        )


def _validate_command_specific_motion_fields(opcode, mode, encoded_values):
    if opcode == "ML" and encoded_values["rounding"] < 0:
        raise MotionInputError("ML rounding must be non-negative")

    maximum_axis = _LIVE_JOG_MAXIMUM_AXES.get(opcode)
    if maximum_axis is None:
        return
    if mode != "p":
        raise MotionInputError("live-jog speed mode must be Percent")

    vector = encoded_values["value"]
    if not vector.is_integer():
        raise MotionInputError("live-jog vector must be an integer")
    vector = int(vector)
    axis = vector // 10
    direction = vector % 10
    if not 1 <= axis <= maximum_axis or direction not in (0, 1):
        raise MotionInputError("live-jog vector is outside the controller domain")


def _parse_timed_command(
    command,
    opcode,
    envelope,
    timing_pattern,
    suffix_pattern,
    contract_name,
):
    envelope_match = envelope.match(command)
    if envelope_match is None:
        raise MotionInputError(
            f"{contract_name} {opcode} command is missing required fields before timing"
        )
    match = timing_pattern.match(command, envelope_match.end())
    if match is None:
        raise MotionInputError(
            f"{contract_name} motion command has an invalid timing profile"
        )
    suffix_match = (
        None
        if suffix_pattern is None
        else suffix_pattern.match(command, match.end())
    )
    if suffix_match is None:
        raise MotionInputError(
            f"{contract_name} {opcode} command has invalid fields after timing"
        )

    normalized, encoded_values = _canonicalize_protocol_numbers(
        command,
        envelope_match,
        match,
        suffix_match,
    )
    _validate_motion_angle_fields(command, opcode, encoded_values)
    if opcode in ("MA", "MC") and encoded_values.get("tr") != 0.0:
        raise MotionInputError(
            f"{opcode} trajectory rotation is unsupported and must be 0"
        )
    mode = match.group("mode")
    _validate_command_specific_motion_fields(opcode, mode, encoded_values)
    speed = encoded_values["speed"]
    acceleration = encoded_values["acceleration"]
    deceleration = encoded_values["deceleration"]
    ramp = encoded_values["ramp"]
    if speed <= 0:
        raise MotionInputError("command speed must be positive")
    if mode == "p" and speed > 100:
        raise MotionInputError("percent command speed must not exceed 100")
    if not 0 < acceleration <= 100:
        raise MotionInputError("command acceleration must be in (0, 100]")
    if not 0 < deceleration < 100:
        raise MotionInputError("command deceleration must be in (0, 100)")
    if acceleration + deceleration > 100:
        raise MotionInputError(
            "command acceleration and deceleration must not overlap"
        )
    if not 0 < ramp <= CONTROLLER_MAXIMUM_RAMP_PERCENT:
        raise MotionInputError(
            "command ramp must be in (0, "
            f"{CONTROLLER_MAXIMUM_RAMP_PERCENT:g}]"
        )
    return CommandTiming(mode, speed, acceleration, deceleration, ramp), normalized


def _parse_command_contract(command, virtual):
    _serial_command_bytes(command)
    opcode = command[:2]
    if not virtual:
        fieldless = _FIELDLESS_COMMANDS.get(opcode)
        if fieldless is not None:
            if fieldless.fullmatch(command) is None:
                raise MotionInputError(
                    f"serial {opcode} command has an invalid fieldless contract"
                )
            if opcode == "PG":
                validate_controller_filename(command[4:-1], "G-code filename")
            return None, command
    contract_name = "virtual" if virtual else "serial"
    envelopes = (
        _VIRTUAL_TIMING_ENVELOPES
        if virtual
        else _STANDARD_TIMING_ENVELOPES
    )
    envelope = envelopes.get(opcode)
    timing_pattern = _STANDARD_TIMING_PROFILE
    suffix_pattern = (
        _STANDARD_TIMING_SUFFIXES.get(opcode)
        if virtual
        else _SERIAL_TIMING_SUFFIXES.get(opcode)
    )
    if envelope is None:
        envelope = _LEGACY_TIMING_ENVELOPES.get(opcode)
        timing_pattern = _LEGACY_JOG_TIMING_PROFILE
        suffix_pattern = _LEGACY_TIMING_SUFFIXES.get(opcode)
    if envelope is None:
        raise MotionInputError(
            f"{contract_name} command opcode {opcode!r} has no timing contract"
        )
    timing, normalized = _parse_timed_command(
        command,
        opcode,
        envelope,
        timing_pattern,
        suffix_pattern,
        contract_name,
    )
    if not virtual and opcode in ("WC", "WG"):
        filename_index = normalized.find("Fn")
        if filename_index < 0:
            raise MotionInputError(f"serial {opcode} filename marker is missing")
        media_marker_index = (
            filename_index - CONTROLLER_MEDIA_ID_LENGTH - 2
        )
        if (
            media_marker_index < 0
            or normalized[media_marker_index:media_marker_index + 2] != "Mi"
        ):
            raise MotionInputError(
                f"serial {opcode} media identity marker is missing"
            )
        validate_controller_media_id(
            normalized[media_marker_index + 2:filename_index]
        )
        validate_controller_filename(
            normalized[filename_index + 2:-1],
            f"{opcode} filename",
        )
    return timing, normalized


def validate_controller_filename(filename, field_name):
    if (
        not isinstance(filename, str)
        or not filename
        or filename in (".", "..")
        or any(
            character in _FAT_RESERVED_FILENAME_CHARACTERS
            for character in filename
        )
        or CONTROLLER_DIRECTORY_SEPARATOR in filename
        or filename[0] == " "
        or filename[-1] == " "
        or any(ord(character) < 32 or ord(character) > 126 for character in filename)
    ):
        raise MotionInputError(
            f"{field_name} contains a controller-reserved or control character"
        )
    if len(filename.encode("ascii")) > MAX_CONTROLLER_FILENAME_BYTES:
        raise MotionInputError(
            f"{field_name} exceeds {MAX_CONTROLLER_FILENAME_BYTES} encoded bytes"
        )
    return filename


def _serial_axis_targets(command):
    opcode = command[:2]
    envelope = _STANDARD_TIMING_ENVELOPES.get(opcode)
    if envelope is None:
        return {}
    match = envelope.match(command)
    if match is None:
        raise MotionInputError(
            f"serial {opcode} command is missing required axis fields"
        )
    if opcode == "RJ":
        names = tuple(f"j{axis}" for axis in range(1, JOINT_COUNT + 1))
        return {
            axis: match.group(name)
            for axis, name in enumerate(names, start=1)
        }
    if opcode in ("MG", "MJ", "ML", "MV", "WC", "WG"):
        return {
            axis: match.group(f"j{axis}")
            for axis in range(7, JOINT_COUNT + 1)
        }
    return {}


def canonicalize_serial_command(command, calibration=None):
    """Validate and encode every numeric field for controller transmission."""
    normalized = _parse_command_contract(command, virtual=False)[1]
    axis_targets = _serial_axis_targets(normalized)
    if axis_targets:
        if not isinstance(calibration, ControllerJointCalibration):
            raise MotionInputError(
                "target-bearing serial commands require controller calibration"
            )
        calibration.validate_axis_positions(axis_targets)
    return normalized


def request_joint_telemetry(command):
    normalized = _parse_command_contract(command, virtual=False)[1]
    if normalized[:2] != "RJ":
        raise MotionInputError("joint telemetry is available only for RJ commands")
    if normalized.endswith("T1\n"):
        return normalized
    requested = normalized[:-1] + "T1\n"
    return _parse_command_contract(requested, virtual=False)[1]


def joint_telemetry_response_budget(response_timeout_seconds):
    timeout = finite_number(
        response_timeout_seconds,
        "response_timeout_seconds",
    )
    if timeout <= 0:
        raise MotionInputError("response_timeout_seconds must be positive")
    return (
        math.ceil(timeout / JOINT_TELEMETRY_PERIOD_SECONDS)
        + JOINT_TELEMETRY_RESPONSE_BUDGET_MARGIN
    )


def canonicalize_virtual_command(command):
    """Validate and encode every numeric field for simulator parsing."""
    return _parse_command_contract(command, virtual=True)[1]


def parse_command_timing(command):
    """Return a validated controller timing profile, when present."""
    return _parse_command_contract(command, virtual=False)[0]


def parse_virtual_command_timing(command):
    """Return timing from a validated simulator motion command."""
    return _parse_command_contract(command, virtual=True)[0]


def parse_motion_wrist_config(command, virtual=False):
    if not isinstance(virtual, bool):
        raise TypeError("virtual wrist-config flag must be boolean")
    normalized = _parse_command_contract(command, virtual=virtual)[1]
    match = re.search(rf"W([NFA])(?:Vr{_NUMBER})?(?=Lm)", normalized)
    if match is None:
        raise MotionInputError("motion command is missing a wrist configuration")
    return match.group(1)


def parse_command_speed(command):
    """Return the validated controller speed mode and value, when present."""
    timing = parse_command_timing(command)
    if timing is None:
        return None
    return timing.mode, timing.speed


def command_response_timeout(
    command,
    baseline_seconds,
    minimum_ramp_full_scale_seconds,
    millimeter_motion_distance_bound,
    margin_seconds=10.0,
):
    """Derive a finite response deadline from the encoded controller speed."""
    baseline = finite_number(baseline_seconds, "baseline_seconds")
    margin = finite_number(margin_seconds, "margin_seconds")
    if baseline <= 0:
        raise MotionInputError("baseline_seconds must be positive")
    if margin < 0:
        raise MotionInputError("margin_seconds must be non-negative")

    timing = parse_command_timing(command)
    if timing is None:
        if command[:2] == "PG":
            raise MotionInputError(
                "G-code playback requires cancellation-bound response ownership"
            )
        return baseline

    return motion_timing_response_timeout(
        timing,
        baseline,
        minimum_ramp_full_scale_seconds,
        millimeter_motion_distance_bound,
        margin_seconds=margin,
    )


def motion_timing_response_timeout(
    timing,
    baseline_seconds,
    minimum_ramp_full_scale_seconds,
    millimeter_motion_distance_bound,
    margin_seconds=10.0,
):
    """Derive a finite response deadline from validated motion timing."""
    if not isinstance(timing, CommandTiming):
        raise MotionInputError("timing must be a CommandTiming")
    baseline = finite_number(baseline_seconds, "baseline_seconds")
    margin = finite_number(margin_seconds, "margin_seconds")
    if baseline <= 0:
        raise MotionInputError("baseline_seconds must be positive")
    if margin < 0:
        raise MotionInputError("margin_seconds must be non-negative")
    if timing.mode not in ("p", "s", "m"):
        raise MotionInputError("command timing mode is invalid")
    speed = _controller_float(timing.speed, "command speed")
    acceleration = _controller_float(
        timing.acceleration,
        "command acceleration",
    )
    deceleration = _controller_float(
        timing.deceleration,
        "command deceleration",
    )
    ramp = _controller_float(timing.ramp, "command ramp")
    if speed <= 0 or (timing.mode == "p" and speed > 100):
        raise MotionInputError("command timing speed is out of range")
    if not 0 < acceleration <= 100:
        raise MotionInputError("command acceleration must be in (0, 100]")
    if not 0 < deceleration < 100:
        raise MotionInputError("command deceleration must be in (0, 100)")
    if acceleration + deceleration > 100:
        raise MotionInputError(
            "command acceleration and deceleration must not overlap"
        )
    if not 0 < ramp <= CONTROLLER_MAXIMUM_RAMP_PERCENT:
        raise MotionInputError(
            "command ramp must be in (0, "
            f"{CONTROLLER_MAXIMUM_RAMP_PERCENT:g}]"
        )

    full_scale = finite_number(
        minimum_ramp_full_scale_seconds,
        "minimum_ramp_full_scale_seconds",
    )
    if full_scale <= 0:
        raise MotionInputError("minimum_ramp_full_scale_seconds must be positive")
    full_scale *= max(
        1.0,
        ramp / FIRMWARE_MINIMUM_RAMP_PERCENT,
        FIRMWARE_LEGACY_RAMP_NUMERATOR / ramp,
    )

    if timing.mode == "p":
        expected_duration = full_scale * (100.0 / speed)
    elif timing.mode == "s":
        expected_duration = max(full_scale, speed * 2.0)
    else:
        path_bound = finite_number(
            millimeter_motion_distance_bound,
            "millimeter_motion_distance_bound",
        )
        if path_bound <= 0:
            raise MotionInputError("millimeter_motion_distance_bound must be positive")
        expected_duration = max(
            full_scale,
            (path_bound / speed) * 2.0,
        )

    deadline = max(
        baseline,
        expected_duration * RESPONSE_TIMEOUT_SAFETY_SCALE + margin,
    )
    if not math.isfinite(deadline):
        raise MotionInputError("derived response deadline is not finite")
    return deadline


@dataclass(frozen=True)
class PositionResponse:
    raw: str
    joint_text: Tuple[str, ...]
    joints: Tuple[float, ...]
    cartesian_text: Tuple[str, ...]
    cartesian: Tuple[float, ...]
    external_text: Tuple[str, ...]
    external: Tuple[float, ...]
    speed_violation: bool
    debug: str
    flag: str


@dataclass(frozen=True)
class ControllerLineExchangeResponse:
    """Terminal/E-stop frame ownership metadata for a bounded exchange."""

    terminal_response: Optional[str]
    estop_position: Optional[PositionResponse]
    frame_order: str
    admission_position: Optional[PositionResponse] = None

    def __post_init__(self):
        if self.terminal_response is not None:
            if (
                not isinstance(self.terminal_response, str)
                or not self.terminal_response
                or self.terminal_response
                != self.terminal_response.strip()
                or "\r" in self.terminal_response
                or "\n" in self.terminal_response
                or len(self.terminal_response)
                > MAX_RESPONSE_PAYLOAD_LENGTH
            ):
                raise MotionInputError(
                    "controller terminal response is invalid"
                )
            try:
                self.terminal_response.encode("ascii")
            except UnicodeEncodeError as exc:
                raise MotionInputError(
                    "controller terminal response must contain ASCII only"
                ) from exc
        if self.frame_order == "estop-only":
            if (
                self.terminal_response is not None
                or not isinstance(self.estop_position, PositionResponse)
                or self.estop_position.flag != CONTROLLER_ESTOP_EVENT_FLAG
                or self.admission_position is not None
            ):
                raise MotionInputError(
                    "standalone controller E-stop response is invalid"
                )
        elif self.frame_order == "admission-estop":
            if (
                self.terminal_response is not None
                or not isinstance(self.estop_position, PositionResponse)
                or self.estop_position.flag
                != CONTROLLER_ESTOP_ADMISSION_FLAG
                or self.admission_position is not None
            ):
                raise MotionInputError(
                    "controller admission E-stop response is invalid"
                )
        elif self.frame_order == "terminal-only":
            if (
                self.estop_position is not None
                or self.admission_position is not None
            ):
                raise MotionInputError(
                    "terminal-only controller response contains an E-stop"
                )
            if self.terminal_response is None:
                raise MotionInputError(
                    "terminal-only controller response lacks a terminal"
                )
        elif self.frame_order in ("estop-terminal", "terminal-estop"):
            if (
                self.terminal_response is None
                or not isinstance(self.estop_position, PositionResponse)
                or self.estop_position.flag != CONTROLLER_ESTOP_EVENT_FLAG
                or self.admission_position is not None
            ):
                raise MotionInputError(
                    "controller E-stop response is invalid"
                )
        elif self.frame_order == "estop-admission":
            if (
                self.terminal_response is not None
                or not isinstance(self.estop_position, PositionResponse)
                or self.estop_position.flag != CONTROLLER_ESTOP_EVENT_FLAG
                or not isinstance(
                    self.admission_position,
                    PositionResponse,
                )
                or self.admission_position.flag
                != CONTROLLER_ESTOP_ADMISSION_FLAG
            ):
                raise MotionInputError(
                    "controller E-stop admission pair is invalid"
                )
        else:
            raise MotionInputError(
                "controller response frame order is invalid"
            )

    @property
    def authoritative_response(self):
        if self.admission_position is not None:
            return self.admission_position.raw
        if self.estop_position is not None:
            return self.estop_position.raw
        return self.terminal_response


@dataclass(frozen=True)
class ControllerModbusExchangeResponse(ControllerLineExchangeResponse):
    pass


@dataclass(frozen=True)
class JointTelemetry:
    """Actual encoder positions sampled during one requested RJ exchange."""

    raw: str
    joints: Tuple[float, ...]


def parse_joint_telemetry_response(response):
    if (
        not isinstance(response, str)
        or not response
        or response != response.strip()
        or len(response) > MAX_RESPONSE_PAYLOAD_LENGTH
    ):
        raise ProtocolResponseError(
            "controller joint telemetry response is invalid"
        )
    try:
        response.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolResponseError(
            "controller joint telemetry response must contain ASCII only"
        ) from exc
    match = _JOINT_TELEMETRY_RESPONSE.fullmatch(response)
    if match is None:
        raise ProtocolResponseError(
            "controller joint telemetry response has invalid markers or values"
        )
    millidegrees = tuple(
        int(match.group(f"j{axis}_millidegrees"))
        for axis in range(1, JOINT_TELEMETRY_AXIS_COUNT + 1)
    )
    if any(
        value < -CONTROLLER_SIGNED_INT_MAX - 1
        or value > CONTROLLER_SIGNED_INT_MAX
        for value in millidegrees
    ):
        raise ProtocolResponseError(
            "controller joint telemetry response exceeds the signed range"
        )
    return JointTelemetry(
        raw=response,
        joints=tuple(value / 1000.0 for value in millidegrees),
    )


def parse_joint_motion_exchange_response(response):
    if (
        isinstance(response, str)
        and response.startswith(JOINT_TELEMETRY_FAMILY_PREFIX)
    ):
        return parse_joint_telemetry_response(response)
    try:
        parse_position_response(response)
    except ProtocolResponseError as exc:
        if (
            not isinstance(response, str)
            or _JOINT_MOTION_ERROR_RESPONSE.fullmatch(response) is None
        ):
            raise ProtocolResponseError(
                "controller joint-motion exchange returned an invalid frame"
            ) from exc
    return None


def _parse_primary_home_reference_response(
    response,
    response_pattern,
    axis_groups,
):
    if (
        not isinstance(response, str)
        or not response
        or response != response.strip()
        or len(response) > MAX_RESPONSE_PAYLOAD_LENGTH
    ):
        raise ProtocolResponseError(
            "controller home-reference response is invalid"
        )
    try:
        response.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolResponseError(
            "controller home-reference response must contain ASCII only"
        ) from exc
    match = response_pattern.fullmatch(response)
    if match is None:
        raise ProtocolResponseError(
            "controller home-reference response has invalid markers or values"
        )
    millidegrees = tuple(
        int(match.group(position_group))
        for _, position_group in axis_groups
    )
    if any(
        value < -CONTROLLER_SIGNED_INT_MAX - 1
        or value > CONTROLLER_SIGNED_INT_MAX
        for value in millidegrees
    ):
        raise ProtocolResponseError(
            "controller home-reference response exceeds the signed range"
        )
    try:
        missing_axis_count = 3 - len(axis_groups)
        return PrimaryHomeReference(
            valid=(
                tuple(
                    match.group(valid_group) == "1"
                    for valid_group, _ in axis_groups
                )
                + (False,) * missing_axis_count
            ),
            positions=(
                tuple(value / 1000.0 for value in millidegrees)
                + (0.0,) * missing_axis_count
            ),
        )
    except MotionInputError as exc:
        raise ProtocolResponseError(
            f"controller home-reference response is inconsistent: {exc}"
        ) from exc


def parse_primary_home_reference_response(response):
    return _parse_primary_home_reference_response(
        response,
        _PRIMARY_HOME_REFERENCE_V1_RESPONSE,
        (
            ("j1_valid", "j1_millidegrees"),
            ("j2_valid", "j2_millidegrees"),
        ),
    )


def parse_primary_home_reference_v2_response(response):
    return _parse_primary_home_reference_response(
        response,
        _PRIMARY_HOME_REFERENCE_V2_RESPONSE,
        (
            ("j1_valid", "j1_millidegrees"),
            ("j2_valid", "j2_millidegrees"),
            ("j3_valid", "j3_millidegrees"),
        ),
    )


def select_primary_home_reference_capability(protocol_capabilities):
    if isinstance(protocol_capabilities, (str, bytes)):
        raise MotionInputError(
            "controller protocol capabilities must be a sequence"
        )
    try:
        capabilities = tuple(protocol_capabilities)
    except TypeError as exc:
        raise MotionInputError(
            "controller protocol capabilities must be a sequence"
        ) from exc
    if any(
        not isinstance(capability, str) or not capability
        for capability in capabilities
    ):
        raise MotionInputError(
            "controller protocol capabilities contain an invalid value"
        )
    for capability in (
        CONTROLLER_CAPABILITY_HOME_REFERENCE_V2,
        CONTROLLER_CAPABILITY_HOME_REFERENCE_V1,
    ):
        if capability in capabilities:
            return capability
    return None


def primary_home_reference_command(protocol_capabilities):
    capability = select_primary_home_reference_capability(
        protocol_capabilities
    )
    if capability == CONTROLLER_CAPABILITY_HOME_REFERENCE_V2:
        return "H2\n"
    if capability == CONTROLLER_CAPABILITY_HOME_REFERENCE_V1:
        return "HR\n"
    return None


def parse_primary_home_reference_capability_response(
    response,
    protocol_capabilities,
):
    capability = select_primary_home_reference_capability(
        protocol_capabilities
    )
    if capability == CONTROLLER_CAPABILITY_HOME_REFERENCE_V2:
        return parse_primary_home_reference_v2_response(response)
    if capability == CONTROLLER_CAPABILITY_HOME_REFERENCE_V1:
        return parse_primary_home_reference_response(response)
    raise ProtocolResponseError(
        "controller does not advertise a home-reference protocol"
    )


def parse_position_response(response):
    if not isinstance(response, str):
        raise ProtocolResponseError("controller response must be text")
    if response != response.strip():
        raise ProtocolResponseError(
            "controller position response contains leading or trailing whitespace"
        )
    raw = response
    if not raw:
        raise ProtocolResponseError("controller returned an empty response")
    if len(raw) > MAX_RESPONSE_PAYLOAD_LENGTH:
        raise ProtocolResponseError("controller response exceeds the size limit")
    try:
        raw.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolResponseError("controller response must contain ASCII characters only") from exc

    match = _POSITION_RESPONSE.fullmatch(raw)
    if match is None:
        raise ProtocolResponseError("controller position response has invalid markers or values")

    joint_text = tuple(match.group(name) for name in ("j1", "j2", "j3", "j4", "j5", "j6"))
    cartesian_text = tuple(match.group(name) for name in ("x", "y", "z", "rz", "ry", "rx"))
    external_text = tuple(match.group(name) for name in ("j7", "j8", "j9"))

    numeric_text = joint_text + cartesian_text + external_text
    try:
        numeric_values = tuple(
            controller_number(value, "controller position response value")
            for value in numeric_text
        )
    except MotionInputError as exc:
        raise ProtocolResponseError(
            f"controller position response contains an invalid value: {exc}"
        ) from exc
    joints = numeric_values[:6]
    cartesian = numeric_values[6:12]
    external = numeric_values[12:]

    return PositionResponse(
        raw=raw,
        joint_text=joint_text,
        joints=joints,
        cartesian_text=cartesian_text,
        cartesian=cartesian,
        external_text=external_text,
        external=external,
        speed_violation=match.group("speed_violation") == "1",
        debug=match.group("debug"),
        flag=match.group("flag"),
    )


def position_response_is_physical_estop(position):
    return (
        isinstance(position, PositionResponse)
        and position.flag in CONTROLLER_PHYSICAL_ESTOP_FLAGS
    )


def _parse_controller_line_exchange_frame(frame):
    if (
        not isinstance(frame, str)
        or not frame
        or frame != frame.strip()
        or "\r" in frame
        or "\n" in frame
        or len(frame) > MAX_RESPONSE_PAYLOAD_LENGTH
    ):
        raise ProtocolResponseError(
            "controller line exchange frame is invalid"
        )
    try:
        frame.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolResponseError(
            "controller line exchange frame must contain ASCII only"
        ) from exc

    try:
        position = parse_position_response(frame)
    except ProtocolResponseError:
        position = None
    if position is not None:
        if position_response_is_physical_estop(position):
            if position.flag == CONTROLLER_ESTOP_ADMISSION_FLAG:
                return "estop-admission", position
            return "estop-event", position
        return "terminal", None
    return "terminal", None


def _parse_controller_modbus_exchange_frame(command, frame):
    kind, position = _parse_controller_line_exchange_frame(frame)
    if kind != "terminal":
        return kind, position

    try:
        classify_controller_modbus_terminal_response(command, frame)
    except ProtocolResponseError as exc:
        raise ProtocolResponseError(
            "controller Modbus exchange terminal frame is invalid"
        ) from exc
    return "terminal", None


def parse_controller_line_exchange_frames(frames):
    """Validate one terminal and the supported physical-stop frame orders."""
    if not isinstance(frames, (tuple, list)):
        raise ProtocolResponseError(
            "controller line exchange frames must be a sequence"
        )
    frame_sequence = tuple(frames)
    if len(frame_sequence) not in (1, 2):
        raise ProtocolResponseError(
            "controller line exchange must contain one or two frames"
        )

    parsed = tuple(
        _parse_controller_line_exchange_frame(frame)
        for frame in frame_sequence
    )
    kinds = tuple(kind for kind, _ in parsed)
    if len(frame_sequence) == 1:
        if kinds[0] == "estop-event":
            return ControllerLineExchangeResponse(
                None,
                parsed[0][1],
                "estop-only",
            )
        if kinds[0] == "estop-admission":
            return ControllerLineExchangeResponse(
                None,
                parsed[0][1],
                "admission-estop",
            )
        return ControllerLineExchangeResponse(
            frame_sequence[0],
            None,
            "terminal-only",
        )

    if kinds == ("estop-event", "estop-admission"):
        return ControllerLineExchangeResponse(
            None,
            parsed[0][1],
            "estop-admission",
            parsed[1][1],
        )
    if kinds == ("estop-event", "terminal"):
        frame_order = "estop-terminal"
        estop_position = parsed[0][1]
        terminal_response = frame_sequence[1]
    elif kinds == ("terminal", "estop-event"):
        frame_order = "terminal-estop"
        estop_position = parsed[1][1]
        terminal_response = frame_sequence[0]
    else:
        raise ProtocolResponseError(
            "controller line exchange contains duplicate frame roles"
        )
    return ControllerLineExchangeResponse(
        terminal_response,
        estop_position,
        frame_order,
    )


def parse_controller_modbus_exchange_frames(command, frames):
    """Validate a bounded terminal/E-stop frame set under one response owner."""
    validate_controller_modbus_command(command)
    response = parse_controller_line_exchange_frames(frames)
    if response.terminal_response is not None:
        _parse_controller_modbus_exchange_frame(
            command,
            response.terminal_response,
        )
    return ControllerModbusExchangeResponse(
        response.terminal_response,
        response.estop_position,
        response.frame_order,
        response.admission_position,
    )


def read_pending_controller_estop_response(
    serial_port,
    estop_callback,
    timeout=PENDING_CONTROL_FRAME_TIMEOUT_SECONDS,
):
    """Publish and consume a queued E-stop before another controller write."""
    timeout = finite_number(timeout, "pending controller E-stop timeout")
    if timeout < CONTROL_POLL_INTERVAL_SECONDS * 2:
        raise MotionInputError(
            "pending controller E-stop timeout is too short"
        )
    if not callable(estop_callback):
        raise MotionInputError(
            "pending controller E-stop callback must be callable"
        )
    _require_open_serial_port(serial_port)

    readline = getattr(serial_port, "readline", None)
    read_until = getattr(serial_port, "read_until", None)
    read = getattr(serial_port, "read", None)
    if not (callable(read_until) or callable(readline)) or not callable(read):
        raise TypeError(
            "serial connection does not satisfy the pending E-stop "
            "read contract"
        )
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError(
            "serial connection does not expose a read timeout"
        ) from exc

    operation_error = None
    try:
        deadline = time.monotonic() + timeout
        _set_serial_timeout(
            serial_port,
            CONTROL_POLL_INTERVAL_SECONDS,
        )
        leading = read(1)
        if not isinstance(leading, (bytes, bytearray)):
            raise ProtocolResponseError(
                "pending controller E-stop probe returned non-bytes data"
            )
        if not leading:
            if not getattr(serial_port, "is_open", False):
                raise ConnectionError(
                    "controller connection closed during the pending "
                    "E-stop probe"
                )
            return None

        response = _read_serial_framed_line(
            serial_port,
            readline,
            read_until,
            deadline,
            timeout,
            initial_bytes=leading,
        )
        position = parse_position_response(response)
        if not position_response_is_physical_estop(position):
            raise ProtocolResponseError(
                "queued controller frame is not a physical E-stop"
            )
        try:
            published = estop_callback(position)
        except Exception as exc:
            raise ProtocolResponseError(
                f"pending controller E-stop publication failed: {exc}"
            ) from exc
        if published is not True:
            raise ProtocolResponseError(
                "pending controller E-stop callback did not confirm "
                "publication"
            )
        _read_serial_quiet_boundary(
            serial_port,
            read,
            deadline - time.monotonic(),
            "pending controller E-stop response",
        )
        return position
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            "pending controller E-stop read ended without trusted framing: "
            f"{exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def read_controller_line_exchange_response(
    serial_port,
    timeout,
    *,
    estop_callback,
    allow_standalone_estop_event=False,
    interim_response_handler=None,
    interim_response_limit=None,
    accepted_responses=None,
    terminal_validator=None,
    context="controller line response",
):
    """Own one bounded terminal and physical-stop frame exchange."""
    timeout = finite_number(timeout, "controller line response timeout")
    if timeout <= 0:
        raise MotionInputError(
            "controller line response timeout must be positive"
        )
    if not callable(estop_callback):
        raise MotionInputError(
            "controller line E-stop callback must be callable"
        )
    if not isinstance(allow_standalone_estop_event, bool):
        raise MotionInputError(
            "standalone controller E-stop policy must be boolean"
        )
    if interim_response_handler is not None and not callable(
        interim_response_handler
    ):
        raise MotionInputError(
            "controller line interim handler must be callable or None"
        )
    if interim_response_handler is None:
        if interim_response_limit is not None:
            raise MotionInputError(
                "controller line interim limit requires a handler"
            )
    elif (
        isinstance(interim_response_limit, bool)
        or not isinstance(interim_response_limit, int)
        or interim_response_limit <= 0
    ):
        raise MotionInputError(
            "controller line interim limit must be a positive integer"
        )
    if terminal_validator is not None and not callable(terminal_validator):
        raise MotionInputError(
            "controller line terminal validator must be callable or None"
        )
    accepted = _validated_response_set(accepted_responses)
    if (
        not isinstance(context, str)
        or not context
        or context != context.strip()
        or "\r" in context
        or "\n" in context
    ):
        raise MotionInputError(
            "controller line response context must be normalized text"
        )
    _require_open_serial_port(serial_port)

    readline = getattr(serial_port, "readline", None)
    read_until = getattr(serial_port, "read_until", None)
    read = getattr(serial_port, "read", None)
    if not (callable(read_until) or callable(readline)) or not callable(read):
        raise TypeError(
            "serial connection does not satisfy the stop-aware line contract"
        )
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError(
            "serial connection does not expose a read timeout"
        ) from exc

    operation_error = None
    try:
        deadline = time.monotonic() + timeout
        interim_count = 0

        def publish_estop(position):
            try:
                published = estop_callback(position)
            except Exception as exc:
                raise ProtocolResponseError(
                    f"{context} E-stop publication failed: {exc}"
                ) from exc
            if published is not True:
                raise ProtocolResponseError(
                    f"{context} E-stop callback did not confirm publication"
                )

        def read_frame(initial_bytes=b""):
            return _read_serial_framed_line(
                serial_port,
                readline,
                read_until,
                deadline,
                timeout,
                initial_bytes=initial_bytes,
            )

        def read_initial_frame():
            nonlocal interim_count
            while True:
                frame = read_frame()
                frame_kind, _ = _parse_controller_line_exchange_frame(frame)
                if frame_kind != "terminal":
                    return frame
                if interim_response_handler is None:
                    return frame
                try:
                    consumed = interim_response_handler(frame)
                except Exception as exc:
                    raise ProtocolResponseError(
                        f"{context} interim handler failed: {exc}"
                    ) from exc
                if not isinstance(consumed, bool):
                    raise ProtocolResponseError(
                        f"{context} interim handler must return a boolean"
                    )
                if not consumed:
                    return frame
                interim_count += 1
                if interim_count > interim_response_limit:
                    raise ProtocolResponseError(
                        f"{context} exceeded the interim response limit"
                    )

        first = read_initial_frame()
        first_kind, first_position = _parse_controller_line_exchange_frame(
            first
        )
        frames = [first]

        if first_kind == "estop-admission":
            publish_estop(first_position)
            response = parse_controller_line_exchange_frames(frames)
            _read_serial_quiet_boundary(
                serial_port,
                read,
                deadline - time.monotonic(),
                context,
            )
        elif first_kind == "estop-event":
            publish_estop(first_position)
            if allow_standalone_estop_event:
                remaining = deadline - time.monotonic()
                if remaining < CONTROL_POLL_INTERVAL_SECONDS:
                    _raise_quarantined_transport(
                        serial_port,
                        f"{context} E-stop follow-up probe deadline expired",
                        error_type=SerialTransportTimeout,
                    )
                _set_serial_timeout(
                    serial_port,
                    CONTROL_POLL_INTERVAL_SECONDS,
                )
                leading = read(1)
                if not isinstance(leading, (bytes, bytearray)):
                    raise ProtocolResponseError(
                        f"{context} E-stop follow-up probe returned "
                        "non-bytes data"
                    )
                if not leading:
                    if not getattr(serial_port, "is_open", False):
                        raise ConnectionError(
                            f"{context} connection closed during the "
                            "E-stop quiet boundary"
                        )
                    quarantine_serial_transport(
                        serial_port,
                        f"{context} standalone E-stop event has ambiguous "
                        "command-admission framing",
                    )
                    response = parse_controller_line_exchange_frames(frames)
                else:
                    second = read_frame(initial_bytes=leading)
                    second_kind, _ = (
                        _parse_controller_line_exchange_frame(second)
                    )
                    if second_kind == "estop-event":
                        raise ProtocolResponseError(
                            f"{context} returned duplicate physical-stop events"
                        )
                    frames.append(second)
                    response = parse_controller_line_exchange_frames(frames)
                    _read_serial_quiet_boundary(
                        serial_port,
                        read,
                        deadline - time.monotonic(),
                        context,
                    )
            else:
                second = read_frame()
                second_kind, _ = _parse_controller_line_exchange_frame(second)
                if second_kind == "estop-event":
                    raise ProtocolResponseError(
                        f"{context} returned duplicate physical-stop events"
                    )
                frames.append(second)
                response = parse_controller_line_exchange_frames(frames)
                _read_serial_quiet_boundary(
                    serial_port,
                    read,
                    deadline - time.monotonic(),
                    context,
                )
        else:
            remaining = deadline - time.monotonic()
            if remaining < CONTROL_POLL_INTERVAL_SECONDS:
                _raise_quarantined_transport(
                    serial_port,
                    f"{context} follow-up probe deadline expired",
                    error_type=SerialTransportTimeout,
                )
            _set_serial_timeout(
                serial_port,
                CONTROL_POLL_INTERVAL_SECONDS,
            )
            leading = read(1)
            if not isinstance(leading, (bytes, bytearray)):
                raise ProtocolResponseError(
                    f"{context} follow-up probe returned non-bytes data"
                )
            if not leading:
                if not getattr(serial_port, "is_open", False):
                    raise ConnectionError(
                        f"{context} connection closed during the quiet boundary"
                    )
                response = parse_controller_line_exchange_frames(frames)
            else:
                second = read_frame(initial_bytes=leading)
                second_kind, second_position = (
                    _parse_controller_line_exchange_frame(second)
                )
                if second_kind != "estop-event":
                    raise ProtocolResponseError(
                        f"{context} returned an invalid post-terminal frame"
                    )
                publish_estop(second_position)
                frames.append(second)
                response = parse_controller_line_exchange_frames(frames)
                _read_serial_quiet_boundary(
                    serial_port,
                    read,
                    deadline - time.monotonic(),
                    context,
                )

        if (
            response.terminal_response is not None
        ):
            if (
                accepted is not None
                and response.terminal_response not in accepted
            ):
                raise ProtocolResponseError(
                    f"{context} returned an unexpected terminal response"
                )
            if terminal_validator is not None:
                try:
                    terminal_validator(response.terminal_response)
                except Exception as exc:
                    raise ProtocolResponseError(
                        f"{context} terminal validation failed: {exc}"
                    ) from exc
        return response
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            f"{context} ended without trusted framing: {exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def read_controller_exact_exchange_response(
    serial_port,
    expected_response,
    timeout,
    *,
    estop_callback,
    context="controller exact response",
):
    """Own an unframed terminal or a physical-stop framed response."""
    if (
        not isinstance(expected_response, bytes)
        or not expected_response
        or len(expected_response) > MAX_RESPONSE_PAYLOAD_LENGTH
        or expected_response != expected_response.strip()
        or b"\r" in expected_response
        or b"\n" in expected_response
    ):
        raise MotionInputError(
            "expected_response must contain normalized, non-empty "
            "unframed bytes"
        )
    try:
        expected_text = expected_response.decode("ascii")
    except UnicodeDecodeError as exc:
        raise MotionInputError(
            "expected_response must contain ASCII bytes only"
        ) from exc
    if expected_response.startswith(b"A"):
        raise MotionInputError(
            "expected_response conflicts with physical-stop framing"
        )
    timeout = finite_number(timeout, "controller exact response timeout")
    if timeout <= 0:
        raise MotionInputError(
            "controller exact response timeout must be positive"
        )
    if not callable(estop_callback):
        raise MotionInputError(
            "controller exact E-stop callback must be callable"
        )
    if (
        not isinstance(context, str)
        or not context
        or context != context.strip()
        or "\r" in context
        or "\n" in context
    ):
        raise MotionInputError(
            "controller exact response context must be normalized text"
        )
    _require_open_serial_port(serial_port)

    readline = getattr(serial_port, "readline", None)
    read_until = getattr(serial_port, "read_until", None)
    read = getattr(serial_port, "read", None)
    if not (callable(read_until) or callable(readline)) or not callable(read):
        raise TypeError(
            "serial connection does not satisfy the stop-aware exact contract"
        )
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError(
            "serial connection does not expose a read timeout"
        ) from exc

    operation_error = None
    try:
        deadline = time.monotonic() + timeout

        def publish_estop(position):
            try:
                published = estop_callback(position)
            except Exception as exc:
                raise ProtocolResponseError(
                    f"{context} E-stop publication failed: {exc}"
                ) from exc
            if published is not True:
                raise ProtocolResponseError(
                    f"{context} E-stop callback did not confirm publication"
                )

        def read_item():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                _raise_quarantined_timeout(serial_port, timeout)
            _set_serial_timeout(serial_port, remaining)
            leading = read(1)
            if not isinstance(leading, (bytes, bytearray)):
                raise ProtocolResponseError(
                    f"{context} reader returned non-bytes data"
                )
            if not leading:
                _raise_quarantined_timeout(serial_port, timeout)
            if bytes(leading) == b"A":
                frame = _read_serial_framed_line(
                    serial_port,
                    readline,
                    read_until,
                    deadline,
                    timeout,
                    initial_bytes=leading,
                )
                kind, position = _parse_controller_line_exchange_frame(
                    frame
                )
                if kind == "terminal":
                    raise ProtocolResponseError(
                        f"{context} returned an unexpected framed terminal"
                    )
                return "framed", frame, kind, position

            response = bytearray(leading)
            while len(response) < len(expected_response):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    _raise_quarantined_timeout(serial_port, timeout)
                _set_serial_timeout(serial_port, remaining)
                chunk = read(len(expected_response) - len(response))
                if not isinstance(chunk, (bytes, bytearray)):
                    raise ProtocolResponseError(
                        f"{context} reader returned non-bytes data"
                    )
                if not chunk:
                    _raise_quarantined_timeout(serial_port, timeout)
                response.extend(chunk)
            if bytes(response) != expected_response:
                raise ProtocolResponseError(
                    f"{context} returned an unexpected unframed terminal"
                )
            return "terminal", expected_text, "terminal", None

        first_type, first, first_kind, first_position = read_item()
        if first_type == "terminal":
            remaining = deadline - time.monotonic()
            if remaining < CONTROL_POLL_INTERVAL_SECONDS:
                _raise_quarantined_transport(
                    serial_port,
                    f"{context} follow-up probe deadline expired",
                    error_type=SerialTransportTimeout,
                )
            _set_serial_timeout(
                serial_port,
                CONTROL_POLL_INTERVAL_SECONDS,
            )
            leading = read(1)
            if not isinstance(leading, (bytes, bytearray)):
                raise ProtocolResponseError(
                    f"{context} follow-up probe returned non-bytes data"
                )
            if not leading:
                if not getattr(serial_port, "is_open", False):
                    raise ConnectionError(
                        f"{context} connection closed during the quiet boundary"
                    )
                return ControllerLineExchangeResponse(
                    expected_text,
                    None,
                    "terminal-only",
                )
            if bytes(leading) != b"A":
                raise ProtocolResponseError(
                    f"{context} returned trailing unframed data"
                )
            second = _read_serial_framed_line(
                serial_port,
                readline,
                read_until,
                deadline,
                timeout,
                initial_bytes=leading,
            )
            second_kind, second_position = (
                _parse_controller_line_exchange_frame(second)
            )
            if second_kind != "estop-event":
                raise ProtocolResponseError(
                    f"{context} returned an invalid post-terminal frame"
                )
            publish_estop(second_position)
            response = parse_controller_line_exchange_frames(
                (expected_text, second)
            )
        elif first_kind == "estop-admission":
            publish_estop(first_position)
            response = parse_controller_line_exchange_frames((first,))
        else:
            publish_estop(first_position)
            second_type, second, second_kind, _ = read_item()
            if second_kind == "estop-event":
                raise ProtocolResponseError(
                    f"{context} returned duplicate physical-stop events"
                )
            if second_type == "terminal":
                response = parse_controller_line_exchange_frames(
                    (first, expected_text)
                )
            else:
                response = parse_controller_line_exchange_frames(
                    (first, second)
                )

        _read_serial_quiet_boundary(
            serial_port,
            read,
            deadline - time.monotonic(),
            context,
        )
        return response
    except (SerialTransportQuarantinedError, SerialTransportTimeout) as exc:
        operation_error = exc
        raise
    except Exception as exc:
        operation_error = exc
        _raise_quarantined_transport(
            serial_port,
            f"{context} ended without trusted framing: {exc}",
            cause=exc,
        )
    finally:
        try:
            _set_serial_timeout(serial_port, original_timeout)
        except Exception as exc:
            reason = "unable to restore the serial read timeout"
            if operation_error is not None:
                reason = f"{operation_error}; {reason}"
            _raise_quarantined_transport(
                serial_port,
                reason,
                cause=exc,
            )


def read_controller_modbus_exchange_response(
    serial_port,
    command,
    timeout,
    estop_callback,
):
    """Own a complete Modbus terminal response and optional E-stop frame."""
    validate_controller_modbus_command(command)
    if not callable(estop_callback):
        raise MotionInputError(
            "controller Modbus E-stop callback must be callable"
        )
    timeout = finite_number(timeout, "controller Modbus response timeout")
    if timeout <= 0:
        raise MotionInputError(
            "controller Modbus response timeout must be positive"
        )
    response = read_controller_line_exchange_response(
        serial_port,
        timeout,
        estop_callback=estop_callback,
        terminal_validator=lambda terminal: (
            _parse_controller_modbus_exchange_frame(command, terminal)
        ),
        context="controller Modbus response",
    )
    return ControllerModbusExchangeResponse(
        response.terminal_response,
        response.estop_position,
        response.frame_order,
        response.admission_position,
    )


def _is_physical_estop_position_response(response):
    try:
        return position_response_is_physical_estop(
            parse_position_response(response)
        )
    except ProtocolResponseError:
        return False


@dataclass(frozen=True)
class MotionSubmission:
    target: Tuple[float, ...]
    coalesced: bool


@dataclass(frozen=True)
class MotionEvent:
    kind: str
    move: JointMove
    started_at_seconds: Optional[float] = None
    response: Optional[str] = None
    position: Optional[PositionResponse] = None
    telemetry: Optional[JointTelemetry] = None
    error: Optional[str] = None
    pending_discarded: bool = False
    _acknowledgement: Optional[threading.Event] = None

    def acknowledge(self):
        if self._acknowledgement is None:
            return False
        self._acknowledgement.set()
        return True


class DeferredJointAdjustments:
    """Accumulate joint intents until confirmed state permits dispatch."""

    def __init__(self):
        self._lock = threading.Lock()
        self._adjustments = (0.0,) * JOINT_COUNT
        self._targets = (None,) * JOINT_COUNT
        self._profile = None
        self._position_generation = None

    @property
    def pending(self):
        with self._lock:
            return self._pending_locked()

    def add(self, axis, delta, profile, confirmed_position_generation):
        if isinstance(axis, bool) or not isinstance(axis, int) or not 0 <= axis < JOINT_COUNT:
            raise MotionInputError(f"axis must be an integer in [0, {JOINT_COUNT - 1}]")
        normalized_delta = finite_number(delta, "delta")
        if normalized_delta == 0:
            raise MotionInputError("delta must be non-zero")
        if not isinstance(profile, MotionProfile):
            raise MotionInputError("profile must be a MotionProfile")
        self._validate_generation(confirmed_position_generation)

        with self._lock:
            if not self._pending_locked():
                self._position_generation = confirmed_position_generation
            values = list(self._adjustments)
            targets = list(self._targets)
            if targets[axis] is None:
                values[axis] += normalized_delta
            else:
                targets[axis] += normalized_delta
            self._adjustments = tuple(values)
            self._targets = tuple(targets)
            self._profile = profile

            if not self._has_intent_locked():
                self._clear_locked()
            return self._pending_locked()

    def set_target(self, axis, target, profile, confirmed_position_generation):
        if isinstance(axis, bool) or not isinstance(axis, int) or not 0 <= axis < JOINT_COUNT:
            raise MotionInputError(f"axis must be an integer in [0, {JOINT_COUNT - 1}]")
        targets = [None] * JOINT_COUNT
        targets[axis] = target
        return self.set_targets(
            targets,
            profile,
            confirmed_position_generation,
        )

    def set_targets(self, targets, profile, confirmed_position_generation):
        normalized_targets = _optional_finite_tuple(
            targets,
            JOINT_COUNT,
            "targets",
        )
        if not isinstance(profile, MotionProfile):
            raise MotionInputError("profile must be a MotionProfile")
        self._validate_generation(confirmed_position_generation)

        with self._lock:
            if not self._pending_locked():
                self._position_generation = confirmed_position_generation
            adjustments = list(self._adjustments)
            merged_targets = list(self._targets)
            for axis, target in enumerate(normalized_targets):
                if target is None:
                    continue
                adjustments[axis] = 0.0
                merged_targets[axis] = target
            self._adjustments = tuple(adjustments)
            self._targets = tuple(merged_targets)
            self._profile = profile
            return True

    def ready(self, confirmed_position_generation, allow_current_generation=False):
        self._validate_generation(confirmed_position_generation)
        if not isinstance(allow_current_generation, bool):
            raise MotionInputError("allow_current_generation must be boolean")
        with self._lock:
            return self._ready_locked(
                confirmed_position_generation,
                allow_current_generation,
            )

    def consume(
        self,
        actual_positions,
        confirmed_position_generation,
        consumer,
        allow_current_generation=False,
    ):
        actual = _finite_tuple(actual_positions, JOINT_COUNT, "actual_positions")
        self._validate_generation(confirmed_position_generation)
        if not isinstance(allow_current_generation, bool):
            raise MotionInputError("allow_current_generation must be boolean")
        if not callable(consumer):
            raise MotionInputError("consumer must be callable")

        # Keep resolution, dispatcher admission, and clearing under one lock.
        # Producers therefore become accepted only before the submitted snapshot
        # or after that snapshot has been consumed, never inside the clear window.
        with self._lock:
            if not self._ready_locked(
                confirmed_position_generation,
                allow_current_generation,
            ):
                raise MotionQueueFault(
                    "deferred joint intents require a newer position or confirmed transport release"
                )
            resolved = []
            for position, adjustment, target in zip(
                actual,
                self._adjustments,
                self._targets,
            ):
                resolved.append(position + adjustment if target is None else target)
            result = consumer(tuple(resolved), self._profile)
            self._clear_locked()
            return result

    def clear(self):
        with self._lock:
            self._clear_locked()

    def _clear_locked(self):
        self._adjustments = (0.0,) * JOINT_COUNT
        self._targets = (None,) * JOINT_COUNT
        self._profile = None
        self._position_generation = None

    def _pending_locked(self):
        return self._profile is not None

    def _ready_locked(self, confirmed_position_generation, allow_current_generation):
        return (
            self._pending_locked()
            and (
                confirmed_position_generation > self._position_generation
                or (
                    allow_current_generation
                    and confirmed_position_generation == self._position_generation
                )
            )
        )

    def _has_intent_locked(self):
        return any(target is not None for target in self._targets) or any(
            adjustment != 0 for adjustment in self._adjustments
        )

    @staticmethod
    def _validate_generation(generation):
        if isinstance(generation, bool) or not isinstance(generation, int) or generation < 0:
            raise MotionInputError("position generation must be a non-negative integer")


class CoalescingJointDispatcher:
    """Serialize joint moves and retain only the latest pending absolute target."""

    def __init__(
        self,
        exchange: Callable[[str], str],
        calibration_provider: Callable[[], ControllerJointCalibration],
        thread_name="ar4-joint-motion",
        transport_lock=None,
        activity_factory=None,
    ):
        if not callable(exchange):
            raise MotionInputError("exchange must be callable")
        if not callable(calibration_provider):
            raise MotionInputError("calibration_provider must be callable")
        if transport_lock is None:
            transport_lock = threading.Lock()
        if not callable(getattr(transport_lock, "acquire", None)) or not callable(
            getattr(transport_lock, "release", None)
        ):
            raise MotionInputError("transport_lock must satisfy the lock contract")
        if activity_factory is not None and not callable(activity_factory):
            raise MotionInputError("activity_factory must be callable or None")
        self._exchange = exchange
        self._calibration_provider = calibration_provider
        self._thread_name = thread_name
        self._transport_lock = transport_lock
        self._activity_factory = activity_factory
        self._lock = threading.Lock()
        self._events = deque()
        self._latest_telemetry_event = None
        self._next_event_sequence = 0
        self._worker = None
        self._inflight = None
        self._pending = None
        self._desired = None
        self._fault_reason = None
        self._closed = False
        self._transport_reserved = False
        self._activity_lease = None
        self._result_acknowledgement = None

    @property
    def active(self):
        with self._lock:
            return self._worker is not None

    @property
    def fault_reason(self):
        with self._lock:
            return self._fault_reason

    @property
    def pending(self):
        with self._lock:
            return self._pending is not None

    @property
    def desired_target(self):
        with self._lock:
            return self._desired

    def submit_delta(self, axis, delta, actual_positions, profile):
        if isinstance(axis, bool) or not isinstance(axis, int) or not 0 <= axis < JOINT_COUNT:
            raise MotionInputError(f"axis must be an integer in [0, {JOINT_COUNT - 1}]")
        normalized_delta = finite_number(delta, "delta")
        if normalized_delta == 0:
            raise MotionInputError("delta must be non-zero")
        adjustments = [0.0] * JOINT_COUNT
        adjustments[axis] = normalized_delta
        return self.submit_adjustments(adjustments, actual_positions, profile)

    def submit_adjustments(self, adjustments, actual_positions, profile):
        normalized_adjustments = _finite_tuple(
            adjustments,
            JOINT_COUNT,
            "adjustments",
        )
        if not any(adjustment != 0 for adjustment in normalized_adjustments):
            raise MotionInputError("at least one joint adjustment must be non-zero")
        actual = _finite_tuple(actual_positions, JOINT_COUNT, "actual_positions")
        if not isinstance(profile, MotionProfile):
            raise MotionInputError("profile must be a MotionProfile")

        def resolve_target(base):
            return tuple(
                position + adjustment
                for position, adjustment in zip(base, normalized_adjustments)
            )

        return self._submit_resolved_target(actual, profile, resolve_target)

    def submit_target(self, axis, target, actual_positions, profile):
        if isinstance(axis, bool) or not isinstance(axis, int) or not 0 <= axis < JOINT_COUNT:
            raise MotionInputError(f"axis must be an integer in [0, {JOINT_COUNT - 1}]")
        targets = [None] * JOINT_COUNT
        targets[axis] = target
        return self.submit_targets(
            targets,
            actual_positions,
            profile,
        )

    def submit_targets(self, targets, actual_positions, profile):
        normalized_targets = _optional_finite_tuple(
            targets,
            JOINT_COUNT,
            "targets",
        )
        actual = _finite_tuple(actual_positions, JOINT_COUNT, "actual_positions")
        if not isinstance(profile, MotionProfile):
            raise MotionInputError("profile must be a MotionProfile")

        def resolve_target(base):
            values = list(base)
            for axis, target in enumerate(normalized_targets):
                if target is not None:
                    values[axis] = target
            return tuple(values)

        return self._submit_resolved_target(actual, profile, resolve_target)

    def submit_positions(self, target_positions, actual_positions, profile):
        target = _finite_tuple(target_positions, JOINT_COUNT, "target_positions")
        actual = _finite_tuple(actual_positions, JOINT_COUNT, "actual_positions")
        if not isinstance(profile, MotionProfile):
            raise MotionInputError("profile must be a MotionProfile")

        return self._submit_resolved_target(actual, profile, lambda base: target)

    def _submit_resolved_target(self, actual, profile, resolve_target):
        worker_to_start = None
        with self._lock:
            if self._closed:
                raise MotionQueueFault("joint-motion dispatcher is closed")
            if self._fault_reason is not None:
                raise MotionQueueFault(
                    "joint-motion queue requires a valid position resynchronization: "
                    + self._fault_reason
                )

            starting_worker = self._worker is None
            if starting_worker:
                base = self._desired if self._desired is not None else actual
            else:
                base = self._desired
                if base is None:
                    raise MotionQueueFault("joint-motion queue lost desired-state synchronization")

            calibration = self._calibration_provider()
            if not isinstance(calibration, ControllerJointCalibration):
                raise MotionInputError(
                    "calibration_provider must return a ControllerJointCalibration"
                )
            target = calibration.validate_positions(resolve_target(base))
            move = JointMove(target, profile, calibration)
            coalesced = self._inflight is not None or self._pending is not None

            if starting_worker:
                worker_to_start = threading.Thread(
                    target=self._run,
                    name=self._thread_name,
                    daemon=True,
                )
                if not self._transport_lock.acquire(blocking=False):
                    raise MotionTransportBusy("controller transport is busy")
                try:
                    activity_lease = (
                        self._activity_factory()
                        if self._activity_factory is not None
                        else None
                    )
                    if activity_lease is not None and not callable(
                        getattr(activity_lease, "close", None)
                    ):
                        raise MotionInputError(
                            "activity_factory must return a closeable lease or None"
                        )
                except Exception:
                    self._transport_lock.release()
                    raise
                self._transport_reserved = True
                self._activity_lease = activity_lease

            self._desired = target
            self._pending = move
            if starting_worker:
                self._worker = worker_to_start
                # Starting under the admission lock prevents another caller from
                # receiving acceptance for state that startup rollback would clear.
                try:
                    worker_to_start.start()
                except Exception as exc:
                    self._pending = None
                    self._desired = None
                    self._fault_reason = f"worker startup failed: {exc}"
                    try:
                        self._release_transport_locked()
                    finally:
                        self._worker = None
                    raise

        return MotionSubmission(target=target, coalesced=coalesced)

    def _release_transport_locked(self):
        if not self._transport_reserved:
            return False
        self._transport_reserved = False
        activity_lease = self._activity_lease
        self._activity_lease = None
        try:
            self._transport_lock.release()
        finally:
            if activity_lease is not None:
                activity_lease.close()
        return True

    def synchronize(self, positions):
        calibration = self._calibration_provider()
        if not isinstance(calibration, ControllerJointCalibration):
            raise MotionInputError(
                "calibration_provider must return a ControllerJointCalibration"
            )
        normalized = calibration.validate_positions(positions)
        with self._lock:
            if self._closed or self._worker is not None:
                return False
            self._desired = normalized
            self._fault_reason = None
            return True

    def discard_pending_after_completion(self, confirmed_positions):
        calibration = self._calibration_provider()
        if not isinstance(calibration, ControllerJointCalibration):
            raise MotionInputError(
                "calibration_provider must return a ControllerJointCalibration"
            )
        normalized = calibration.validate_positions(confirmed_positions)
        with self._lock:
            if self._closed:
                return False
            if (
                self._worker is None
                or self._inflight is not None
                or self._result_acknowledgement is None
                or self._fault_reason is not None
            ):
                raise MotionQueueFault(
                    "pending targets can be discarded only during successful-result acknowledgement"
                )
            pending_discarded = self._pending is not None
            self._pending = None
            self._desired = normalized
            return pending_discarded

    def invalidate(self, reason):
        if not isinstance(reason, str) or not reason.strip():
            raise MotionInputError("fault reason must be non-empty text")
        with self._lock:
            if self._closed:
                return False
            pending_discarded = self._pending is not None
            self._pending = None
            self._desired = None
            self._fault_reason = reason.strip()
            return pending_discarded

    def close(self):
        acknowledgement = None
        with self._lock:
            self._closed = True
            self._pending = None
            self._desired = None
            self._latest_telemetry_event = None
            acknowledgement = self._result_acknowledgement
            if self._worker is None and self._transport_reserved:
                self._release_transport_locked()
        if acknowledgement is not None:
            acknowledgement.set()

    def drain_events(self, limit=None):
        if limit is not None and (isinstance(limit, bool) or not isinstance(limit, int) or limit < 0):
            raise MotionInputError("event limit must be a non-negative integer or None")
        events = []
        with self._lock:
            while limit is None or len(events) < limit:
                queued_event = self._events[0] if self._events else None
                telemetry_event = self._latest_telemetry_event
                if queued_event is None and telemetry_event is None:
                    break
                if (
                    telemetry_event is not None
                    and (
                        queued_event is None
                        or telemetry_event[0] < queued_event[0]
                    )
                ):
                    self._latest_telemetry_event = None
                    events.append(telemetry_event[1])
                else:
                    events.append(self._events.popleft()[1])
        return events

    def _next_event_record_locked(self, event):
        self._next_event_sequence += 1
        return self._next_event_sequence, event

    def _publish_event(self, event):
        with self._lock:
            self._events.append(
                self._next_event_record_locked(event)
            )

    def publish_telemetry(self, telemetry):
        if not isinstance(telemetry, JointTelemetry):
            raise MotionInputError(
                "joint telemetry event must contain validated telemetry"
            )
        try:
            validated = parse_joint_telemetry_response(telemetry.raw)
        except ProtocolResponseError as exc:
            raise MotionInputError(
                "joint telemetry event must contain validated telemetry"
            ) from exc
        if telemetry != validated:
            raise MotionInputError(
                "joint telemetry event must match its validated wire frame"
            )
        with self._lock:
            if self._closed:
                return False
            move = self._inflight
            if move is None:
                raise MotionQueueFault(
                    "joint telemetry arrived without an in-flight move"
                )
            self._latest_telemetry_event = self._next_event_record_locked(
                MotionEvent(
                    kind="telemetry",
                    move=move,
                    telemetry=validated,
                )
            )
        return True

    def _run(self):
        while True:
            move = None
            with self._lock:
                if self._closed or self._pending is None:
                    self._inflight = None
                    self._release_transport_locked()
                    self._worker = None
                else:
                    move = self._pending
                    self._pending = None
                    self._inflight = move
                    self._latest_telemetry_event = None
            if move is None:
                return

            started_at_seconds = time.monotonic()
            self._publish_event(
                MotionEvent(
                    kind="started",
                    move=move,
                    started_at_seconds=started_at_seconds,
                )
            )
            response = None
            position = None
            try:
                response = self._exchange(move.command)
                if not isinstance(response, str):
                    raise ProtocolResponseError("serial exchange returned a non-text response")
                if response.startswith("E"):
                    raise ProtocolResponseError(f"controller rejected motion: {response}")
                parsed_position = parse_position_response(response)
                move.calibration.validate_positions(
                    parsed_position.joints + parsed_position.external
                )
                position = parsed_position
                if parsed_position.flag:
                    raise ProtocolResponseError(
                        f"controller reported motion fault: {parsed_position.flag}"
                    )
            except Exception as exc:
                acknowledgement = threading.Event()
                with self._lock:
                    pending_discarded = self._pending is not None
                    self._pending = None
                    self._desired = None
                    self._inflight = None
                    if not self._closed:
                        self._fault_reason = str(exc)
                        self._result_acknowledgement = acknowledgement
                    else:
                        acknowledgement.set()
                self._publish_event(
                    MotionEvent(
                        kind="failed",
                        move=move,
                        response=response,
                        position=position,
                        error=str(exc),
                        pending_discarded=pending_discarded,
                        _acknowledgement=acknowledgement,
                    )
                )
                acknowledgement.wait()
                with self._lock:
                    if self._result_acknowledgement is acknowledgement:
                        self._result_acknowledgement = None
                    self._release_transport_locked()
                    self._worker = None
                return

            acknowledgement = threading.Event()
            with self._lock:
                self._inflight = None
                if self._pending is None:
                    self._desired = position.joints + position.external
                if not self._closed:
                    self._result_acknowledgement = acknowledgement
                else:
                    acknowledgement.set()

            self._publish_event(
                MotionEvent(
                    kind="completed",
                    move=move,
                    response=response,
                    position=position,
                    _acknowledgement=acknowledgement,
                )
            )
            acknowledgement.wait()
            with self._lock:
                if self._result_acknowledgement is acknowledgement:
                    self._result_acknowledgement = None
                should_continue = not self._closed and self._pending is not None
                if not should_continue:
                    self._release_transport_locked()
                    self._worker = None
            if not should_continue:
                return
