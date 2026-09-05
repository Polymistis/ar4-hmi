"""Validated, coalescing joint-motion dispatch for the AR4 HMI.

This module has no GUI or serial dependency.  The desktop application supplies
a blocking request/response callable and consumes completion events on the Tk
event thread.
"""

from dataclasses import dataclass
from contextlib import contextmanager
from collections import deque
from collections.abc import Mapping
from decimal import Decimal, InvalidOperation
from enum import Enum
import math
from numbers import Integral, Real
import re
import struct
import threading
import time
from typing import Callable, Optional, Tuple


JOINT_COUNT = 9
PRIMARY_START_POSITION = (0.0, 0.0, 0.0, 0.0, 45.0, 0.0)
MAX_COMMAND_LENGTH = 4096
MAX_RESPONSE_PAYLOAD_LENGTH = 4096
AUXILIARY_BOARD_NONE = "None"
AUXILIARY_BOARD_NANO = "Nano"
AUXILIARY_BOARD_MEGA = "Mega"
AUXILIARY_BOARD_OUTPUT_PINS = {
    AUXILIARY_BOARD_NANO: frozenset(range(8, 14)),
    AUXILIARY_BOARD_MEGA: frozenset(range(28, 54)),
}
AUXILIARY_BOARD_INPUT_PINS = {
    AUXILIARY_BOARD_NANO: frozenset(range(2, 8)),
    AUXILIARY_BOARD_MEGA: frozenset(range(2, 28)),
}
AUXILIARY_BOARD_PNEUMATIC_PINS = {
    AUXILIARY_BOARD_NANO: 8,
    AUXILIARY_BOARD_MEGA: 28,
}
AUXILIARY_SERVO_CHANNELS = frozenset(range(7))
AUXILIARY_BOARD_SERVO_CHANNELS = {
    AUXILIARY_BOARD_NANO: frozenset(range(6)),
    AUXILIARY_BOARD_MEGA: AUXILIARY_SERVO_CHANNELS,
}
AUXILIARY_SERVO_MINIMUM_POSITION = 0
AUXILIARY_SERVO_MAXIMUM_POSITION = 180
AUXILIARY_WAIT_MAXIMUM_SECONDS = 32767
AUXILIARY_CURRENT_MAXIMUM_AMPS = 28.0
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
CONTROLLER_CAPABILITY_JOINT_TELEMETRY_V1 = "JOINT_TELEMETRY_V1"
CONTROLLER_CAPABILITY_ESTOP_ADMISSION_V1 = "ESTOP_ADMISSION_V1"
CONTROLLER_CAPABILITY_CALIBRATION_SWITCH_POLARITY_V1 = (
    "CALIBRATION_SWITCH_POLARITY_V1"
)
CONTROLLER_ESTOP_ADMISSION_FLAG = "EA"
CONTROLLER_ESTOP_EVENT_FLAG = "EB"
CONTROLLER_PHYSICAL_ESTOP_FLAGS = frozenset(
    (CONTROLLER_ESTOP_ADMISSION_FLAG, CONTROLLER_ESTOP_EVENT_FLAG)
)
JOINT_TELEMETRY_AXIS_COUNT = 6
JOINT_TELEMETRY_PERIOD_SECONDS = 0.1
CONTROLLER_DIRECTORY_SEPARATOR = ","
MAX_CONTROLLER_DIRECTORY_PAYLOAD_BYTES = MAX_RESPONSE_PAYLOAD_LENGTH
MAX_CONTROLLER_FILENAME_BYTES = 255
CONTROLLER_HARDWARE_ID_LENGTH = 6
CONTROLLER_MEDIA_ID_LENGTH = 32
_FAT_RESERVED_FILENAME_CHARACTERS = frozenset('"*/:<>?\\|')
_SERIAL_QUARANTINE_ATTRIBUTE = "_ar4_transport_quarantine_reason"
_SERIAL_QUARANTINE_LOCK = threading.Lock()
_SERIAL_QUARANTINED_PORTS = {}


class MotionInputError(ValueError):
    """A command input cannot be represented safely by the controller protocol."""


def encode_calibration_switch_mask(states):
    """Encode normalized J1-J9 active switch states for the controller."""
    if isinstance(states, (str, bytes)):
        raise MotionInputError(
            "calibration switch states must be a nine-value sequence"
        )
    try:
        states = tuple(states)
    except TypeError as exc:
        raise MotionInputError(
            "calibration switch states must be a nine-value sequence"
        ) from exc
    if len(states) != JOINT_COUNT:
        raise MotionInputError(
            "calibration switch states must contain 9 values"
        )

    mask = 0
    for axis, state in enumerate(states, start=1):
        if state not in ("LOW", "HIGH"):
            raise MotionInputError(
                f"J{axis} calibration switch state must be normalized LOW or HIGH"
            )
        if state == "HIGH":
            mask |= 1 << (axis - 1)
    return mask


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


def validate_auxiliary_servo_command(command, board_profile):
    profile = normalize_auxiliary_board_profile(board_profile)
    channel, _ = parse_auxiliary_servo_command(command)
    if channel not in AUXILIARY_BOARD_SERVO_CHANNELS[profile]:
        raise MotionInputError(
            f"auxiliary servo channel {channel} is not valid for {profile}"
        )
    return command


def parse_auxiliary_input_command(command):
    if (
        not isinstance(command, str)
        or not command
        or len(command) > MAX_COMMAND_LENGTH
    ):
        raise MotionInputError("auxiliary input command is invalid")
    match = re.fullmatch(r"JFX([0-9]+)\n", command)
    if match is None:
        raise MotionInputError("auxiliary input command is malformed")
    return int(match.group(1))


def validate_auxiliary_input_command(command, board_profile):
    profile = normalize_auxiliary_board_profile(board_profile)
    input_pin = parse_auxiliary_input_command(command)
    if input_pin not in AUXILIARY_BOARD_INPUT_PINS[profile]:
        raise MotionInputError(
            f"auxiliary input pin {input_pin} is not valid for {profile}"
        )
    return command


def parse_auxiliary_wait_command(command):
    if (
        not isinstance(command, str)
        or not command
        or len(command) > MAX_COMMAND_LENGTH
    ):
        raise MotionInputError("auxiliary wait command is invalid")
    match = re.fullmatch(
        r"WIA([0-9]+)B([01])C([0-9]+)\n",
        command,
    )
    if match is None:
        raise MotionInputError("auxiliary wait command is malformed")
    timeout = int(match.group(3))
    if timeout == 0:
        raise MotionInputError(
            "auxiliary wait timeout must be positive"
        )
    if timeout > AUXILIARY_WAIT_MAXIMUM_SECONDS:
        raise MotionInputError(
            "auxiliary wait timeout exceeds the firmware range"
        )
    return int(match.group(1)), int(match.group(2)), timeout


def validate_auxiliary_wait_command(command, board_profile):
    profile = normalize_auxiliary_board_profile(board_profile)
    input_pin, _, _ = parse_auxiliary_wait_command(command)
    if input_pin not in AUXILIARY_BOARD_INPUT_PINS[profile]:
        raise MotionInputError(
            f"auxiliary input pin {input_pin} is not valid for {profile}"
        )
    return command


def validate_auxiliary_gripper_current_command(command, board_profile):
    normalize_auxiliary_board_profile(board_profile)
    if command != "TG\n":
        raise MotionInputError(
            "auxiliary gripper-current command is malformed"
        )
    return command


def parse_auxiliary_gripper_current_response(command, response):
    if command != "TG\n":
        raise ProtocolResponseError(
            "auxiliary gripper-current response has no matching command"
        )
    if (
        not isinstance(response, str)
        or not response
        or len(response) > 32
        or response != response.strip()
        or not response.isascii()
        or re.fullmatch(
            r"(?:0|[1-9][0-9]*)(?:\.[0-9]{1,3})?",
            response,
        )
        is None
    ):
        raise ProtocolResponseError(
            "auxiliary gripper-current response is malformed"
        )
    current = float(response)
    if not math.isfinite(current) or current > AUXILIARY_CURRENT_MAXIMUM_AMPS:
        raise ProtocolResponseError(
            "auxiliary gripper-current response is outside the sensor range"
        )
    return response


def auxiliary_pneumatic_output_pin(board_profile):
    profile = normalize_auxiliary_board_profile(board_profile)
    return AUXILIARY_BOARD_PNEUMATIC_PINS[profile]


class LiveMotionScheduleResult(Enum):
    """Non-boolean terminal outcomes from live-motion scheduling admission."""

    CANCELLED = "cancelled"


class SerialActivityRejected(RuntimeError):
    """A controller serial operation cannot start during shutdown or control injection."""


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
    """Track controller serial ownership for shutdown and injected control writes."""

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


def parse_controller_modbus_command(command):
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
    return (
        match.group("opcode"),
        int(match.group("slave")),
        int(match.group("address")),
        int(match.group("value")),
    )


def validate_controller_modbus_command(command):
    parse_controller_modbus_command(command)
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
    """Controller-reported J1-J3 parking references in degrees."""

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


def _finite_number_state(value, field_name):
    if isinstance(value, bool):
        raise MotionInputError(f"{field_name} must be numeric")
    try:
        if isinstance(value, Decimal):
            exact_number = value
            if not exact_number.is_finite():
                raise MotionInputError(f"{field_name} must be finite")
            number = float(value)
            exact_nonzero = exact_number != 0
        elif isinstance(value, str):
            token = value.strip()
            if not token:
                raise InvalidOperation
            exact_number = Decimal(token)
            if not exact_number.is_finite():
                raise MotionInputError(f"{field_name} must be finite")
            number = float(value)
            exact_nonzero = exact_number != 0
        elif isinstance(value, Integral):
            number = float(value)
            exact_number = None
            exact_nonzero = value != 0
        elif isinstance(value, Real):
            number = float(value)
            exact_number = None
            exact_nonzero = bool(value != 0)
        else:
            raise TypeError
    except MotionInputError:
        raise
    except OverflowError as exc:
        raise MotionInputError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    except (InvalidOperation, TypeError, ValueError) as exc:
        raise MotionInputError(f"{field_name} must be numeric") from exc
    if not math.isfinite(number):
        if exact_number is not None and exact_number.is_finite():
            raise MotionInputError(
                f"{field_name} is outside the host numeric range"
            )
        raise MotionInputError(f"{field_name} must be finite")
    return number, exact_nonzero


def finite_number(value, field_name):
    number, exact_nonzero = _finite_number_state(value, field_name)
    if exact_nonzero and number == 0:
        raise MotionInputError(
            f"{field_name} is outside the host numeric range"
        )
    return number


def controller_number(value, field_name):
    number, exact_nonzero = _finite_number_state(value, field_name)
    if exact_nonzero and number == 0:
        raise MotionInputError(
            f"{field_name} cannot be represented by the controller"
        )
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


def submit_primary_joint_target(values, submit):
    normalized = _finite_tuple(values, 6, "primary joint target")
    return submit(normalized)


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


def validate_controller_encoder_scale(
    calibration,
    axis,
    encoder_counts_per_step,
    field_name,
):
    if not isinstance(calibration, ControllerJointCalibration):
        raise MotionInputError(
            "encoder scale calibration must be a ControllerJointCalibration"
        )
    if (
        isinstance(axis, bool)
        or not isinstance(axis, int)
        or not 1 <= axis <= 6
    ):
        raise MotionInputError("encoder scale axis must be in [1, 6]")
    scale = _controller_float(encoder_counts_per_step, field_name)
    if scale <= 0.0:
        raise MotionInputError(f"{field_name} must be positive")

    negative = _controller_float(
        calibration.negative_limits[axis - 1],
        f"J{axis} negative limit",
    )
    positive = _controller_float(
        calibration.positive_limits[axis - 1],
        f"J{axis} positive limit",
    )
    steps_per_unit = _controller_float(
        calibration.steps_per_unit[axis - 1],
        f"J{axis} steps per unit",
    )
    travel = _controller_float(
        negative + positive,
        f"J{axis} calibrated travel",
    )
    if travel <= 0.0:
        raise MotionInputError(
            f"J{axis} configured travel must be positive"
        )
    step_limit = int(_controller_float_product(
        travel,
        steps_per_unit,
        f"J{axis} calibrated step limit",
    ))
    maximum_written_count = step_limit * scale
    if (
        not math.isfinite(maximum_written_count)
        or maximum_written_count < 0.0
        or maximum_written_count > CONTROLLER_SIGNED_INT_MAX
    ):
        raise MotionInputError(
            f"{field_name} exceeds the signed encoder counter range"
        )
    return scale


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
class JointExchangeSnapshot:
    """Confirmed start and immutable command inputs for one active RJ exchange."""

    start_positions: Tuple[float, ...]
    target_positions: Tuple[float, ...]
    profile: MotionProfile

    def __post_init__(self):
        object.__setattr__(
            self,
            "start_positions",
            _finite_tuple(self.start_positions, JOINT_COUNT, "start_positions"),
        )
        object.__setattr__(
            self,
            "target_positions",
            _finite_tuple(self.target_positions, JOINT_COUNT, "target_positions"),
        )
        if not isinstance(self.profile, MotionProfile):
            raise MotionInputError("exchange profile must be a MotionProfile")


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
        for opcode in ("MJ", "ML", "MV", "WC")
    },
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
        for opcode in ("MJ", "RJ")
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
    "WC": re.compile(
        rf"W[NFA]Lm[01]{{6}}"
        rf"Mi[0-9A-F]{{{CONTROLLER_MEDIA_ID_LENGTH}}}"
        rf"Fn[ -~]+\n\Z"
    ),
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
@dataclass(frozen=True)
class CommandTiming:
    mode: str
    speed: float
    acceleration: float
    deceleration: float
    ramp: float


@dataclass(frozen=True)
class JointMotionCommand:
    robot_joints_degrees: Tuple[float, float, float, float, float, float]
    external_axes_units: Tuple[float, float, float]
    timing: CommandTiming
    wrist_configuration: str
    loop_modes: Tuple[bool, bool, bool, bool, bool, bool]


@dataclass(frozen=True)
class CartesianMotionCommand:
    translation_millimeters: Tuple[float, float, float]
    orientation_degrees: Tuple[float, float, float]
    external_axes_units: Tuple[float, float, float]
    timing: CommandTiming
    wrist_configuration: str
    loop_modes: Tuple[bool, bool, bool, bool, bool, bool]


@dataclass(frozen=True)
class ToolJogCommand:
    axis: str
    direction: str
    distance: float
    timing: CommandTiming
    wrist_configuration: str
    loop_modes: Tuple[bool, bool, bool, bool, bool, bool]


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
    if len(normalized) > MAX_COMMAND_LENGTH:
        raise MotionInputError("controller command exceeds the size limit")
    return normalized, encoded_values


def _validate_motion_angle_fields(command, opcode, encoded_values):
    field_names = []
    if opcode in ("MJ", "ML", "MV", "WC"):
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
    if not isinstance(command, str) or not command.endswith("\n"):
        raise MotionInputError(
            "controller command must be newline-terminated text"
        )
    if not command[:-1] or "\n" in command[:-1] or "\r" in command:
        raise MotionInputError(
            "controller command must contain exactly one trailing line delimiter"
        )
    if len(command) > MAX_COMMAND_LENGTH:
        raise MotionInputError("controller command exceeds the size limit")
    if not command.isascii():
        raise MotionInputError(
            "controller command must contain ASCII characters only"
        )
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
    if not virtual and opcode == "WC":
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
    if opcode in ("MJ", "ML", "MV", "WC"):
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


def canonicalize_virtual_command(command):
    """Validate and encode every numeric field for simulator parsing."""
    return _parse_command_contract(command, virtual=True)[1]


def parse_command_timing(command):
    """Return a validated controller timing profile, when present."""
    return _parse_command_contract(command, virtual=False)[0]


def parse_joint_motion_command(command, virtual=False):
    if not isinstance(virtual, bool):
        raise TypeError("virtual joint-command flag must be boolean")
    timing, normalized = _parse_command_contract(command, virtual=virtual)
    if timing is None or normalized[:2] != "RJ":
        raise MotionInputError("joint motion requires an RJ command")
    envelope = (
        _VIRTUAL_TIMING_ENVELOPES["RJ"]
        if virtual
        else _STANDARD_TIMING_ENVELOPES["RJ"]
    ).match(normalized)
    if envelope is None:
        raise MotionInputError("joint RJ command is missing target fields")
    loop_match = re.search(r"Lm(?P<loops>[01]{6})(?:T1)?\n\Z", normalized)
    if loop_match is None:
        raise MotionInputError("joint RJ loop modes are invalid")
    return JointMotionCommand(
        robot_joints_degrees=tuple(
            float(envelope.group(f"j{axis}")) for axis in range(1, 7)
        ),
        external_axes_units=(0.0, 0.0, 0.0) if virtual else tuple(
            float(envelope.group(f"j{axis}"))
            for axis in range(7, JOINT_COUNT + 1)
        ),
        timing=timing,
        wrist_configuration=parse_motion_wrist_config(
            normalized,
            virtual=virtual,
        ),
        loop_modes=tuple(
            value == "1" for value in loop_match.group("loops")
        ),
    )


def parse_cartesian_motion_command(command, virtual=False):
    if not isinstance(virtual, bool):
        raise TypeError("virtual Cartesian command flag must be boolean")
    timing, normalized = _parse_command_contract(command, virtual=virtual)
    opcode = normalized[:2]
    allowed_opcodes = ("MJ", "MV") if virtual else ("MJ", "ML", "MV", "WC")
    if timing is None or opcode not in allowed_opcodes:
        raise MotionInputError("Cartesian motion command is unsupported")
    envelope = (
        _VIRTUAL_TIMING_ENVELOPES[opcode]
        if virtual
        else _STANDARD_TIMING_ENVELOPES[opcode]
    ).match(normalized)
    if envelope is None:
        raise MotionInputError(f"Cartesian {opcode} command is missing target fields")
    loop_match = re.search(r"Lm(?P<loops>[01]{6})", normalized)
    if loop_match is None:
        raise MotionInputError(f"Cartesian {opcode} command is missing loop modes")
    loop_text = loop_match.group("loops")
    return CartesianMotionCommand(
        translation_millimeters=tuple(
            float(envelope.group(name)) for name in ("x", "y", "z")
        ),
        orientation_degrees=tuple(
            float(envelope.group(name)) for name in ("rx", "ry", "rz")
        ),
        external_axes_units=(0.0, 0.0, 0.0) if virtual else tuple(
            float(envelope.group(name)) for name in ("j7", "j8", "j9")
        ),
        timing=timing,
        wrist_configuration=parse_motion_wrist_config(
            normalized,
            virtual=virtual,
        ),
        loop_modes=tuple(value == "1" for value in loop_text),
    )


def parse_tool_jog_command(command, virtual=False):
    if not isinstance(virtual, bool):
        raise TypeError("virtual tool-jog command flag must be boolean")
    timing, normalized = _parse_command_contract(command, virtual=virtual)
    if timing is None or normalized[:2] != "JT":
        raise MotionInputError("tool-frame jog requires a JT command")
    envelope = _LEGACY_TIMING_ENVELOPES["JT"].match(normalized)
    if envelope is None:
        raise MotionInputError("tool-frame JT command is missing target fields")
    loop_match = re.search(r"Lm(?P<loops>[01]{6})\n\Z", normalized)
    if loop_match is None:
        raise MotionInputError("tool-frame JT loop modes are invalid")
    axis = {
        "X": "x",
        "Y": "y",
        "Z": "z",
        "W": "rx",
        "P": "ry",
        "R": "rz",
    }.get(normalized[2:3])
    direction = (
        {"1": "positive", "0": "negative"}
        if virtual
        else {"1": "negative", "0": "positive"}
    ).get(normalized[3:4])
    if axis is None or direction is None:
        raise MotionInputError("tool-frame JT direction is invalid")
    return ToolJogCommand(
        axis=axis,
        direction=direction,
        distance=float(envelope.group("distance")),
        timing=timing,
        wrist_configuration=parse_motion_wrist_config(
            normalized,
            virtual=virtual,
        ),
        loop_modes=tuple(
            value == "1" for value in loop_match.group("loops")
        ),
    )


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
class JointMotionExchangeResult:
    response: str
    position: Optional[PositionResponse]
    error: Optional[str] = None
    confirmed_position_unchanged: bool = False

    def __post_init__(self):
        if (
            not isinstance(self.response, str)
            or not self.response
            or self.response != self.response.strip()
            or "\r" in self.response
            or "\n" in self.response
            or len(self.response) > MAX_RESPONSE_PAYLOAD_LENGTH
        ):
            raise MotionInputError(
                "joint-motion exchange response is invalid"
            )
        try:
            self.response.encode("ascii")
        except UnicodeEncodeError as exc:
            raise MotionInputError(
                "joint-motion exchange response must contain ASCII only"
            ) from exc
        if type(self.confirmed_position_unchanged) is not bool:
            raise MotionInputError(
                "joint-motion confirmed-position state is invalid"
            )
        if self.position is not None and (
            not isinstance(self.position, PositionResponse)
            or self.position.raw != self.response
        ):
            raise MotionInputError(
                "joint-motion exchange position does not match the response"
            )
        if self.error is not None and (
            not isinstance(self.error, str)
            or not self.error
            or self.error != self.error.strip()
            or "\r" in self.error
            or "\n" in self.error
        ):
            raise MotionInputError(
                "joint-motion exchange error is invalid"
            )
        if self.confirmed_position_unchanged:
            if self.position is not None or self.error is None:
                raise MotionInputError(
                    "unchanged joint-motion position requires a positionless failure"
                )
        elif self.position is None:
            raise MotionInputError(
                "joint-motion exchange requires a validated position"
            )


@dataclass(frozen=True)
class JointTelemetry:
    """Actual encoder positions sampled during one requested RJ exchange."""

    raw: str
    joints: Tuple[float, ...]


class JointTelemetryMeasurement:
    """Single-threaded bounded aggregate owned by one joint-motion exchange."""

    def __init__(self, clock=time.monotonic_ns):
        if not callable(clock):
            raise MotionInputError("telemetry measurement clock must be callable")
        self._clock = clock
        self._admitted_at_ns = None
        self._terminal_at_ns = None
        self._first_receipt_at_ns = None
        self._last_receipt_at_ns = None
        self._frame_count = 0
        self._interval_count = 0
        self._minimum_interval_ns = None
        self._maximum_interval_ns = None
        self._first_sequence = None
        self._last_sequence = None
        self._first_without_baseline = None
        self._gap_events = 0
        self._canonical_bytes = 0
        self._dispatcher_accepted_frames = 0
        self._dispatcher_rejected_frames = 0

    def _read_time(self, supplied=None):
        if supplied is None:
            try:
                value = self._clock()
            except Exception as exc:
                raise MotionInputError(
                    "telemetry measurement clock failed"
                ) from exc
            invalid_detail = "clock returned invalid time"
        else:
            value = supplied
            invalid_detail = "received an invalid supplied time"
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise MotionInputError(f"telemetry measurement {invalid_detail}")
        return value

    def admit(self, admitted_at_ns=None):
        if self._admitted_at_ns is not None:
            raise MotionInputError("telemetry measurement admission is already recorded")
        self._admitted_at_ns = self._read_time(admitted_at_ns)

    def observe(
        self,
        sequence,
        sequence_contiguous,
        canonical_payload,
        accepted,
        received_at_ns=None,
    ):
        if self._admitted_at_ns is None or self._terminal_at_ns is not None:
            raise MotionInputError("telemetry measurement is outside an active request")
        if (
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence < 0
        ):
            raise MotionInputError("telemetry measurement sequence is invalid")
        if (
            sequence_contiguous is not None
            and type(sequence_contiguous) is not bool
        ):
            raise MotionInputError("telemetry sequence continuity is invalid")
        if self._frame_count and sequence_contiguous is None:
            raise MotionInputError("telemetry sequence baseline changed during a move")
        if (
            not isinstance(canonical_payload, str)
            or not canonical_payload
            or not canonical_payload.isascii()
            or "\r" in canonical_payload
            or "\n" in canonical_payload
        ):
            raise MotionInputError("canonical telemetry payload is invalid")
        payload_bytes = len(canonical_payload) + 1
        if type(accepted) is not bool:
            raise MotionInputError("telemetry dispatcher acceptance is invalid")

        received_at_ns = self._read_time(received_at_ns)
        previous_at_ns = (
            self._admitted_at_ns
            if self._last_receipt_at_ns is None
            else self._last_receipt_at_ns
        )
        if received_at_ns < previous_at_ns:
            raise MotionInputError("telemetry measurement clock moved backwards")
        if not self._frame_count:
            self._first_sequence = sequence
            self._first_without_baseline = sequence_contiguous is None
            self._first_receipt_at_ns = received_at_ns
        else:
            interval_ns = received_at_ns - self._last_receipt_at_ns
            self._interval_count += 1
            self._minimum_interval_ns = (
                interval_ns
                if self._minimum_interval_ns is None
                else min(self._minimum_interval_ns, interval_ns)
            )
            self._maximum_interval_ns = (
                interval_ns
                if self._maximum_interval_ns is None
                else max(self._maximum_interval_ns, interval_ns)
            )
        self._last_sequence = sequence
        self._last_receipt_at_ns = received_at_ns
        self._frame_count += 1
        if sequence_contiguous is False:
            self._gap_events += 1
        self._canonical_bytes += payload_bytes
        if accepted:
            self._dispatcher_accepted_frames += 1
        else:
            self._dispatcher_rejected_frames += 1

    def observe_terminal(self, received_at_ns=None):
        if self._admitted_at_ns is None:
            raise MotionInputError("telemetry measurement lacks request admission")
        if self._terminal_at_ns is not None:
            raise MotionInputError("telemetry terminal is already recorded")
        terminal_at_ns = self._read_time(received_at_ns)
        previous_at_ns = (
            self._admitted_at_ns
            if self._last_receipt_at_ns is None
            else self._last_receipt_at_ns
        )
        if terminal_at_ns < previous_at_ns:
            boundary = (
                "last telemetry receipt"
                if self._last_receipt_at_ns is not None
                else "measurement admission"
            )
            raise MotionInputError(f"telemetry terminal precedes {boundary}")
        self._terminal_at_ns = terminal_at_ns

    def finalize(self, request_id):
        if (
            isinstance(request_id, bool)
            or not isinstance(request_id, int)
            or request_id < 1
        ):
            raise MotionInputError("telemetry measurement request identity is invalid")
        if self._terminal_at_ns is None:
            raise MotionInputError("telemetry measurement lacks terminal settlement")
        receipt_window_ns = (
            self._last_receipt_at_ns - self._first_receipt_at_ns
            if self._frame_count > 1
            else None
        )
        payload_window_ns = (
            self._last_receipt_at_ns - self._admitted_at_ns
            if self._last_receipt_at_ns is not None
            else None
        )
        interval_distribution = (
            {
                "count": self._interval_count,
                "minimum": self._minimum_interval_ns,
                "mean": receipt_window_ns / self._interval_count,
                "maximum": self._maximum_interval_ns,
            }
            if self._interval_count
            else None
        )
        return {
            "request_id": request_id,
            "admitted_at_monotonic_ns": self._admitted_at_ns,
            "terminal_at_monotonic_ns": self._terminal_at_ns,
            "frame_count": self._frame_count,
            "first_sequence": self._first_sequence,
            "last_sequence": self._last_sequence,
            "first_sequence_without_prior_baseline": self._first_without_baseline,
            "sequence_gap_events": self._gap_events,
            "canonical_json_lf_bytes": self._canonical_bytes,
            "first_receipt_at_monotonic_ns": self._first_receipt_at_ns,
            "last_receipt_at_monotonic_ns": self._last_receipt_at_ns,
            "receipt_window_ns": receipt_window_ns,
            "payload_window_ns": payload_window_ns,
            "canonical_json_lf_bytes_per_second": (
                self._canonical_bytes * 1_000_000_000 / payload_window_ns
                if payload_window_ns
                else None
            ),
            "receipt_interval_ns": interval_distribution,
            "final_telemetry_to_terminal_ns": (
                self._terminal_at_ns - self._last_receipt_at_ns
                if self._last_receipt_at_ns is not None
                else None
            ),
            "dispatcher_accepted_frames": self._dispatcher_accepted_frames,
            "dispatcher_rejected_frames": self._dispatcher_rejected_frames,
        }


def ordinary_joint_telemetry_measurement(
    telemetry_enabled,
    trace_requested,
    clock=time.monotonic_ns,
):
    if type(telemetry_enabled) is not bool or type(trace_requested) is not bool:
        raise MotionInputError("telemetry measurement selection is invalid")
    if telemetry_enabled and trace_requested:
        raise MotionInputError("trace capture cannot measure ordinary telemetry")
    return JointTelemetryMeasurement(clock) if telemetry_enabled else None


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


@dataclass(frozen=True)
class MotionSubmission:
    target: Tuple[float, ...]
    coalesced: bool


@dataclass(frozen=True)
class MotionEvent:
    kind: str
    move: Optional[JointMove]
    started_at_seconds: Optional[float] = None
    response: Optional[str] = None
    position: Optional[PositionResponse] = None
    telemetry: Optional[JointTelemetry] = None
    error: Optional[str] = None
    pending_discarded: bool = False
    confirmed_position_unchanged: bool = False
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
        exchange: Callable[[str], JointMotionExchangeResult],
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
        self._inflight_start = None
        self._pending = None
        self._desired = None
        self._confirmed = None
        self._fault_reason = None
        self._closed = False
        self._transport_reserved = False
        self._activity_lease = None
        self._result_acknowledgement = None
        self._transport_release_event_reason = None

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
                self._transport_reserved = True
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
                except Exception as admission_error:
                    try:
                        self._release_transport_locked()
                    except Exception as release_error:
                        try:
                            admission_detail = " ".join(
                                str(admission_error).split()
                            )
                        except Exception:
                            admission_detail = type(admission_error).__name__
                        admission_detail = (
                            admission_detail
                            or type(admission_error).__name__
                        )
                        combined_error = MotionQueueFault(
                            "joint-motion admission failed: "
                            f"{admission_detail}; admission rollback failed: "
                            f"{release_error}"
                        )
                        self._publish_transport_release_fault_locked(
                            combined_error
                        )
                        raise combined_error from release_error
                    raise
                self._activity_lease = activity_lease

            self._desired = target
            if starting_worker:
                self._confirmed = base
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
                    self._confirmed = None
                    self._fault_reason = f"worker startup failed: {exc}"
                    release_error = self._finish_worker_locked()
                    if release_error is not None:
                        raise MotionQueueFault(self._fault_reason) from release_error
                    raise

        return MotionSubmission(target=target, coalesced=coalesced)

    def _release_transport_locked(self):
        if not self._transport_reserved and self._activity_lease is None:
            return False

        release_errors = []
        activity_lease = self._activity_lease
        if activity_lease is not None:
            try:
                activity_lease.close()
            except Exception as exc:
                release_errors.append(("serial activity lease", exc))
            else:
                self._activity_lease = None
        if self._transport_reserved:
            try:
                self._transport_lock.release()
            except Exception as exc:
                release_errors.append(("controller transport lock", exc))
            else:
                self._transport_reserved = False
        if release_errors:
            details = []
            for owner, error in release_errors:
                try:
                    detail = " ".join(str(error).split())
                except Exception:
                    detail = type(error).__name__
                details.append(f"{owner}: {detail or type(error).__name__}")
            raise MotionQueueFault("; ".join(details)) from release_errors[0][1]
        return True

    def _latch_transport_release_fault_locked(self, error):
        try:
            detail = " ".join(str(error).split())
        except Exception:
            detail = type(error).__name__
        detail = detail or type(error).__name__
        release_reason = f"controller ownership release failed: {detail}"
        pending_discarded = self._pending is not None
        self._pending = None
        self._desired = None
        self._confirmed = None
        self._latest_telemetry_event = None
        if self._fault_reason is None:
            self._fault_reason = release_reason
        elif release_reason not in self._fault_reason:
            self._fault_reason = f"{self._fault_reason}; {release_reason}"
        return release_reason, pending_discarded

    def _publish_transport_release_fault_locked(self, error, move=None):
        release_reason, pending_discarded = (
            self._latch_transport_release_fault_locked(error)
        )
        if release_reason != self._transport_release_event_reason:
            self._transport_release_event_reason = release_reason
            self._events.append(
                self._next_event_record_locked(
                    MotionEvent(
                        kind="transport-failed",
                        move=move,
                        error=release_reason,
                        pending_discarded=pending_discarded,
                    )
                )
            )
        return release_reason

    def _finish_worker_locked(self, move=None):
        release_error = None
        try:
            self._release_transport_locked()
        except Exception as exc:
            release_error = exc
            self._publish_transport_release_fault_locked(exc, move)
        self._worker = None
        self._inflight_start = None
        return release_error

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
            if self._transport_reserved or self._activity_lease is not None:
                try:
                    self._release_transport_locked()
                except Exception as exc:
                    self._publish_transport_release_fault_locked(exc)
                    return False
            self._desired = normalized
            self._confirmed = normalized
            self._fault_reason = None
            self._transport_release_event_reason = None
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
            self._confirmed = normalized
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
            self._confirmed = None
            self._fault_reason = reason.strip()
            return pending_discarded

    def close(self):
        acknowledgement = None
        cleanup_complete = False
        with self._lock:
            self._closed = True
            self._pending = None
            self._desired = None
            self._confirmed = None
            self._latest_telemetry_event = None
            acknowledgement = self._result_acknowledgement
            cleanup_complete = self._worker is None
            if cleanup_complete and (
                self._transport_reserved or self._activity_lease is not None
            ):
                cleanup_complete = self._finish_worker_locked() is None
        if acknowledgement is not None:
            acknowledgement.set()
        return cleanup_complete

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

    def publish_position_telemetry(self, raw, joints):
        if (
            not isinstance(raw, str)
            or not raw
            or raw != raw.strip()
            or "\r" in raw
            or "\n" in raw
            or len(raw) > MAX_RESPONSE_PAYLOAD_LENGTH
        ):
            raise MotionInputError(
                "joint telemetry wire evidence is invalid"
            )
        try:
            raw.encode("ascii")
        except UnicodeEncodeError as exc:
            raise MotionInputError(
                "joint telemetry wire evidence must contain ASCII only"
            ) from exc
        validated = JointTelemetry(
            raw=raw,
            joints=_finite_tuple(
                joints,
                JOINT_TELEMETRY_AXIS_COUNT,
                "joint telemetry positions",
            ),
        )
        return self._publish_validated_telemetry(validated)

    def _publish_validated_telemetry(self, telemetry):
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
                    telemetry=telemetry,
                )
            )
        return True

    def current_exchange_snapshot(self, command):
        if not isinstance(command, str):
            raise MotionInputError("exchange command must be text")
        with self._lock:
            move = self._inflight
            start = self._inflight_start
            if move is None or start is None:
                return None
            if command != move.command:
                raise MotionQueueFault(
                    "exchange command does not match the in-flight joint move"
                )
            return JointExchangeSnapshot(start, move.positions, move.profile)

    def _run(self):
        while True:
            move = None
            with self._lock:
                if self._closed or self._pending is None:
                    self._inflight = None
                    self._inflight_start = None
                    self._finish_worker_locked()
                else:
                    move = self._pending
                    self._pending = None
                    self._inflight = move
                    self._inflight_start = self._confirmed
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
            confirmed_position_unchanged = False
            try:
                exchange_result = self._exchange(move.command)
                if isinstance(exchange_result, JointMotionExchangeResult):
                    response = exchange_result.response
                    parsed_position = exchange_result.position
                    exchange_error = exchange_result.error
                    confirmed_position_unchanged = (
                        exchange_result.confirmed_position_unchanged
                    )
                else:
                    raise ProtocolResponseError(
                        "joint exchange returned an invalid result"
                    )
                if confirmed_position_unchanged:
                    raise ProtocolResponseError(exchange_error)
                move.calibration.validate_positions(
                    parsed_position.joints + parsed_position.external
                )
                position = parsed_position
                if exchange_error is not None:
                    raise ProtocolResponseError(exchange_error)
                if parsed_position.flag:
                    raise ProtocolResponseError(
                        f"controller reported motion fault: {parsed_position.flag}"
                    )
            except Exception as exc:
                acknowledgement = threading.Event()
                with self._lock:
                    pending_discarded = self._pending is not None
                    self._pending = None
                    if confirmed_position_unchanged:
                        self._desired = self._confirmed
                    else:
                        self._desired = None
                        self._confirmed = None
                    self._inflight = None
                    self._inflight_start = None
                    if not self._closed:
                        self._fault_reason = (
                            None
                            if confirmed_position_unchanged
                            else str(exc)
                        )
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
                        confirmed_position_unchanged=(
                            confirmed_position_unchanged
                        ),
                        _acknowledgement=acknowledgement,
                    )
                )
                acknowledgement.wait()
                with self._lock:
                    if self._result_acknowledgement is acknowledgement:
                        self._result_acknowledgement = None
                    self._finish_worker_locked(move)
                return

            acknowledgement = threading.Event()
            with self._lock:
                self._inflight = None
                self._inflight_start = None
                self._confirmed = position.joints + position.external
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
                    self._finish_worker_locked(move)
            if not should_continue:
                return
