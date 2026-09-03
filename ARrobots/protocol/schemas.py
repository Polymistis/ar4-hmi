"""Command-domain schemas for the correlated controller protocol."""

from dataclasses import dataclass
from functools import partial
import math
import re
import struct
from types import MappingProxyType

from .catalog import (
    AUXILIARY_COMMANDS,
    AUXILIARY_CONTROLLER,
    MAIN_CONTROLLER,
    MAIN_CONTROLLER_COMMANDS,
)
from .messages import (
    JSON_PROTOCOL_MAXIMUM_IDENTIFIER,
    JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
    JSON_PROTOCOL_VERSION,
    Response,
    Telemetry,
    encode_message,
)
from .session import JsonCommandContract


JSON_PROTOCOL_NAME = "ar4_json"
JSON_SESSION_IDENTIFIER_LENGTH = 32
JSON_HELLO_MAXIMUM_CAPABILITIES = 32
JSON_HELLO_MAXIMUM_TEXT_LENGTH = 31
JSON_POSITION_ROBOT_JOINT_COUNT = 6
JSON_POSITION_EXTERNAL_AXIS_COUNT = 3
JSON_POSITION_CARTESIAN_TRANSLATION_COUNT = 3
JSON_POSITION_CARTESIAN_ORIENTATION_COUNT = 3
JSON_POSITION_SOURCE_CONTROLLER_STEP_STATE = "controller_step_state"
JSON_HOME_REFERENCE_AXIS_COUNT = 3
JSON_MOTION_TRACE_RECORD_CAPACITY = 1024
JSON_MOTION_TRACE_PAGE_RECORDS = 8

_SIGNED_INT32_MINIMUM = -(2 ** 31)
_SIGNED_INT32_MAXIMUM = 2 ** 31 - 1
_FLOAT32_MAXIMUM = 3.4028234663852886e38
_RADIANS_PER_DEGREE = 0.017453292519943295769236907684886

JSON_CAPABILITY_PROTOCOL_V1 = "JSON_PROTOCOL_V1"
JSON_CAPABILITY_REQUEST_CORRELATION_V1 = "REQUEST_CORRELATION_V1"
JSON_CAPABILITY_EVENT_STREAM_V1 = "EVENT_STREAM_V1"
JSON_CONTROLLER_WAIT_MAXIMUM_SECONDS = 2147483
JSON_LIVE_MOTION_LEASE_MINIMUM_MILLISECONDS = 1000
JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS = 5000
JSON_MAIN_FIRMWARE_FRAME_RECEIVE_TIMEOUT_SECONDS = 5.0
_JSON_MAIN_FIRMWARE_IDENTITY = ("AR4 Teensy", "6.7.1-ar4hmi.39", "tracked")
_JSON_AUXILIARY_FIRMWARE_IDENTITIES = {
    "nano": ("AR4 Nano IO", "2.0", "ar4hmi"),
    "mega": ("AR4 Mega IO", "2.0", "ar4hmi"),
}
JSON_AUXILIARY_WAIT_MAXIMUM_SECONDS = 32767
JSON_CONTROLLER_MEDIA_IDENTIFIER_LENGTH = 32
JSON_CONTROLLER_FILENAME_MAXIMUM_BYTES = 255
JSON_REQUIRED_SESSION_CAPABILITIES = frozenset(
    (
        JSON_CAPABILITY_PROTOCOL_V1,
        JSON_CAPABILITY_REQUEST_CORRELATION_V1,
        JSON_CAPABILITY_EVENT_STREAM_V1,
    )
)
JSON_MAIN_COMMAND_MANIFEST = tuple(
    command.name for command in MAIN_CONTROLLER_COMMANDS
)
JSON_AUXILIARY_COMMAND_MANIFEST = tuple(
    command.name for command in AUXILIARY_COMMANDS
)

_CAPABILITY_PATTERN = re.compile(r"[A-Z][A-Z0-9_]{0,30}\Z")
_CONTROLLER_HARDWARE_ID_PATTERN = re.compile(r"[0-9A-F]{6}\Z")
_SESSION_IDENTIFIER_PATTERN = re.compile(
    rf"[0-9A-F]{{{JSON_SESSION_IDENTIFIER_LENGTH}}}\Z"
)
_CONTROLLER_MEDIA_IDENTIFIER_PATTERN = re.compile(
    rf"[0-9A-F]{{{JSON_CONTROLLER_MEDIA_IDENTIFIER_LENGTH}}}\Z"
)
_CONFIGURATION_FINGERPRINT_PATTERN = re.compile(
    r"sha256:[0-9a-f]{64}\Z"
)
_CONTROLLER_FILENAME_RESERVED_CHARACTERS = frozenset('"*/,:<>?\\|')
_POSITION_CONTROLLER_DEBUG_PATTERN = re.compile(
    r"(?:|-?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))\Z"
)
_POSITION_MOTION_FAULT_PATTERN = re.compile(r"(?:|EA|EB|EC[01]{6})\Z")
_POSITION_CONTROLLER_ALARM_PATTERN = re.compile(
    r"(?:ER|EL[01]{6}|EL[01]{9})\Z"
)
_POSITION_CONTROLLER_DEBUG_MAXIMUM_LENGTH = 31

_HELLO_RESULT_FIELDS = frozenset(
    (
        "capabilities",
        "device",
        "firmware",
        "identity",
        "commands",
        "protocol",
        "session_id",
    )
)
_HELLO_FIRMWARE_FIELDS = frozenset(("build", "name", "version"))
_HELLO_IDENTITY_FIELDS = frozenset(
    (
        "asset_tag",
        "controller_hardware_id",
        "driver_model",
        "robot_model",
        "robot_version",
        "serial_number",
    )
)
_HELLO_PROTOCOL_FIELDS = frozenset(
    ("max_payload_bytes", "name", "version")
)
_AUXILIARY_HELLO_RESULT_FIELDS = frozenset(
    ("board", "commands", "device", "firmware", "protocol")
)
_HELLO_FAILED_ERROR_CODES = frozenset(
    ("identity_unavailable", "session_unavailable")
)
_POSITION_RESULT_FIELDS = frozenset(
    (
        "axis_source",
        "cartesian_micrometers",
        "external_axes_milliunits",
        "orientation_millidegrees",
        "robot_joints_millidegrees",
    )
)
_POSITION_DISPOSITION_RESULT_FIELDS = frozenset(
    (
        "axis_source",
        "cartesian_micrometers",
        "controller_debug",
        "external_axes_milliunits",
        "motion_fault",
        "orientation_millidegrees",
        "robot_joints_millidegrees",
        "speed_limited",
    )
)
_POSITION_SNAPSHOT_FIELDS = frozenset(
    (
        "axis_source",
        "cartesian_micrometers",
        "external_axes_milliunits",
        "orientation_millidegrees",
        "robot_joints_millidegrees",
    )
)
_POSITION_FAILED_ERROR_CODES = frozenset(
    ("position_unavailable",)
)
_POSITION_DISPOSITION_FAILED_ERROR_CODES = frozenset(
    ("controller_alarm", "position_unavailable")
)
_POSITION_DISPOSITION_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "session_not_established": MappingProxyType({}),
    }
)
_POSITION_CORRECTION_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "configuration_sync_required": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_POSITION_CORRECTION_ENCODER_FAILURE_FIELDS = frozenset(("axes", "position"))
_HOME_REFERENCE_RESULT_FIELDS = frozenset(
    ("positions_millidegrees", "valid")
)
_HOME_REFERENCE_FAILED_ERROR_CODES = frozenset(
    ("home_reference_unavailable",)
)
_DIAGNOSTIC_COMMANDS = frozenset((
    "read_encoders", "set_encoders", "test_limit_switches"))
_SET_POSITION_REQUEST_FIELDS = frozenset(
    ("external_axes_milliunits", "robot_joints_millidegrees")
)
_UPDATE_PARAMS_REQUEST_FIELDS = frozenset(
    (
        "calibration_directions",
        "calibration_switch_active_high",
        "dh_a_millimeters",
        "dh_alpha_degrees",
        "dh_d_millimeters",
        "dh_theta_degrees",
        "encoder_counts_per_step",
        "motor_directions",
        "negative_joint_limits_degrees",
        "positive_joint_limits_degrees",
        "steps_per_degree",
        "tool_rotation_degrees",
        "tool_translation_millimeters",
    )
)
_CONFIG_EXT_AXIS_REQUEST_FIELDS = frozenset(
    ("drive_rotations", "motor_steps", "travel_units")
)
_EXTERNAL_AXIS_ZERO_COMMANDS = frozenset(
    ("zero_j7", "zero_j8", "zero_j9")
)
_CONTROLLER_WAIT_REQUEST_FIELDS = frozenset(("seconds",))
_MODBUS_READ_REQUEST_FIELDS = frozenset(("address", "count", "slave_id"))
_MODBUS_READ_RESULT_FIELDS = frozenset(("value",))
_MODBUS_WRITE_REQUEST_FIELDS = frozenset(("address", "slave_id", "value"))
_MODBUS_WAIT_REQUEST_FIELDS = frozenset(
    ("address", "expected", "slave_id", "timeout_seconds")
)
_MODBUS_TERMINAL_ERROR_CODES = MappingProxyType(
    {"cancelled": "emergency_stop", "failed": "modbus_error"}
)
_MODBUS_READ_VALUE_MAXIMUMS = MappingProxyType(
    {
        "modbus_read_holding_register": 65535,
        "modbus_read_coil": 1,
        "modbus_read_discrete_input": 1,
        "modbus_read_input_register": 65535,
    }
)
_MODBUS_WRITE_VALUE_MAXIMUMS = MappingProxyType(
    {"modbus_write_coil": 1, "modbus_write_register": 65535}
)
_MODBUS_WAIT_VALUE_MAXIMUMS = MappingProxyType(
    {
        "wait_modbus_coil": 1,
        "wait_modbus_discrete_input": 1,
        "wait_modbus_holding_register": 65535,
    }
)
_CALIBRATE_REQUEST_FIELDS = frozenset(("axes", "offsets"))
_CALIBRATE_RESULT_FIELDS = frozenset(
    ("controller_debug", "position", "speed_limited")
)
_CALIBRATE_POSITION_FAILURE_FIELDS = frozenset(("position",))
_CALIBRATE_STAGE_FAILURE_FIELDS = frozenset(("position", "stage"))
_CALIBRATE_REPRESENTATION_FIELDS = frozenset(("axes",))
_CALIBRATE_FAILURE_STAGES = frozenset(
    ("center_move", "fast_limit_search", "slow_limit_search", "switch_release")
)
_MOVE_JOINTS_REQUEST_FIELDS = frozenset(
    (
        "acceleration_percent",
        "deceleration_percent",
        "external_axes_units",
        "loop_modes",
        "ramp_percent",
        "robot_joints_degrees",
        "speed_mode",
        "speed_value",
        "telemetry_enabled",
        "trace_configuration_fingerprint",
        "wrist_configuration",
    )
)
_MOVE_CARTESIAN_REQUEST_FIELDS = frozenset(
    (
        "acceleration_percent",
        "deceleration_percent",
        "external_axes_units",
        "loop_modes",
        "orientation_degrees",
        "ramp_percent",
        "speed_mode",
        "speed_value",
        "telemetry_enabled",
        "translation_millimeters",
        "wrist_configuration",
    )
)
_MOVE_LINEAR_REQUEST_FIELDS = frozenset(
    ("disable_wrist_rotation", "motion", "rounding_millimeters")
)
_MOVE_VISION_REQUEST_FIELDS = frozenset(
    ("motion", "vision_rotation_degrees")
)
_MOVE_ARC_REQUEST_FIELDS = frozenset(
    ("midpoint_translation_millimeters", "motion")
)
_MOVE_CIRCLE_REQUEST_FIELDS = frozenset(
    (
        "center_translation_millimeters",
        "motion",
        "plane_translation_millimeters",
    )
)
_MOVE_SPLINE_REQUEST_FIELDS = frozenset(("segments",))
_MOVE_SPLINE_SEGMENT_FIELDS = frozenset(
    ("motion", "rounding_millimeters")
)
_STORAGE_TARGET_REQUEST_FIELDS = frozenset(("filename", "media_id"))
_WRITE_GCODE_MOVE_REQUEST_FIELDS = frozenset(
    ("filename", "media_id", "motion")
)
_CONTROLLER_DIRECTORY_RESULT_FIELDS = frozenset(("files", "media_id"))
_DELETE_SD_PROGRAM_RESULT_FIELDS = frozenset(("deleted",))
_AUXILIARY_SERVO_REQUEST_FIELDS = frozenset(("channel", "position"))
_AUXILIARY_INPUT_REQUEST_FIELDS = frozenset(("pin",))
_AUXILIARY_OUTPUT_REQUEST_FIELDS = frozenset(("pin", "state"))
_AUXILIARY_WAIT_REQUEST_FIELDS = frozenset(
    ("pin", "state", "timeout_seconds")
)
_AUXILIARY_INPUT_RESULT_FIELDS = frozenset(("state",))
_AUXILIARY_CURRENT_RESULT_FIELDS = frozenset(("amps",))
_JOG_TOOL_REQUEST_FIELDS = frozenset(
    (
        "acceleration_percent",
        "axis",
        "deceleration_percent",
        "direction",
        "distance",
        "loop_modes",
        "ramp_percent",
        "speed_mode",
        "speed_value",
        "wrist_configuration",
    )
)
_LIVE_JOG_REQUEST_FIELDS = frozenset(
    (
        "acceleration_percent",
        "axis",
        "deceleration_percent",
        "direction",
        "lease_milliseconds",
        "loop_modes",
        "ramp_percent",
        "speed_mode",
        "speed_value",
        "telemetry_enabled",
        "wrist_configuration",
    )
)
_STOP_REQUEST_FIELDS = frozenset(("motion_id",))
_STOP_RESULT_FIELDS = frozenset(("motion_id",))
_RENEW_LIVE_MOTION_REQUEST_FIELDS = frozenset(("motion_id",))
_RENEW_LIVE_MOTION_RESULT_FIELDS = frozenset(("motion_id",))
_MOVE_JOINTS_RESULT_FIELDS = frozenset(
    ("controller_debug", "position", "speed_limited")
)
_MOVE_JOINTS_POSITION_FAILURE_FIELDS = frozenset(("position",))
_MOVE_JOINTS_AXIS_FAILURE_FIELDS = frozenset(("axes", "position"))
_MOVE_JOINTS_LIMIT_REJECTION_FIELDS = frozenset(("axes",))
_MOVE_JOINTS_TELEMETRY_FIELDS = frozenset(
    ("robot_joints_millidegrees",)
)
_MOTION_TRACE_REQUEST_FIELDS = frozenset(
    ("motion_request_id", "page_index")
)
_MOTION_TRACE_NO_CAPTURE_RESULT_FIELDS = frozenset(
    ("capture_state", "source_motion_request_id")
)
_MOTION_TRACE_PAGE_RESULT_FIELDS = frozenset(
    (
        "capture_generation",
        "capture_state",
        "configuration_fingerprint",
        "disposition",
        "firmware",
        "page_count",
        "page_index",
        "record_start",
        "records",
        "source_motion_request_id",
        "source_session_id",
        "total_records",
    )
)
_MOTION_TRACE_DISPOSITION_FIELDS = frozenset(
    (
        "capacity_limited",
        "clock_wrapped",
        "complete",
        "motion_outcome",
        "timing_overrun",
    )
)
_MOTION_TRACE_RECORD_FIELDS = frozenset(
    (
        "commanded_steps",
        "controller_microseconds",
        "encoder_counts",
        "flags",
        "master_index",
        "phase",
        "scheduled_delay_microseconds",
    )
)
_MAIN_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "malformed_frame": MappingProxyType({}),
        "parser_resource_exhausted": MappingProxyType({}),
        "duplicate_field": MappingProxyType({}),
        "nesting_limit_exceeded": MappingProxyType({}),
        "container_limit_exceeded": MappingProxyType({}),
        "invalid_field_name": MappingProxyType({}),
        "invalid_string_value": MappingProxyType({}),
        "invalid_number": MappingProxyType({}),
        "invalid_envelope": MappingProxyType({}),
        "emergency_stop_active": MappingProxyType({}),
        "live_motion_active": MappingProxyType({}),
        "unsupported_version": MappingProxyType({"field": "v"}),
        "unsupported_message_type": MappingProxyType({"field": "type"}),
        "invalid_parameter": MappingProxyType({"field": "params"}),
    }
)
_SET_POSITION_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "position_not_representable": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_CONFIGURATION_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "configuration_not_representable": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_SYNCHRONIZED_SESSION_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "configuration_sync_required": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
        "unsupported_command": MappingProxyType({}),
    }
)
_CALIBRATE_EMPTY_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "configuration_sync_required": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_DIAGNOSTIC_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "configuration_sync_required": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_MOVE_JOINTS_EMPTY_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "configuration_sync_required": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_MOTION_TRACE_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "session_not_established": MappingProxyType({}),
    }
)
_MOVE_JOINTS_POSITION_FAILURE_CODES = frozenset(
    ("emergency_stop", "motion_execution_failed")
)
_MOVE_JOINTS_AXIS_FAILURE_CODES = frozenset(
    ("encoder_collision", "encoder_state_unavailable")
)
_STOP_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "emergency_stop_active": MappingProxyType({}),
        "invalid_parameter": MappingProxyType({"field": "params"}),
        "no_live_motion": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_RENEW_LIVE_MOTION_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        "control_lease_expired": MappingProxyType({}),
        "emergency_stop_active": MappingProxyType({}),
        "invalid_parameter": MappingProxyType({"field": "params"}),
        "no_live_motion": MappingProxyType({}),
        "session_not_established": MappingProxyType({}),
    }
)
_LIVE_MOTION_CONTROL_PARSE_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        code: details
        for code, details in _MAIN_REJECTED_ERROR_DETAILS.items()
        if code != "live_motion_active"
    }
)
_STOP_EXACT_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        **_LIVE_MOTION_CONTROL_PARSE_REJECTED_ERROR_DETAILS,
        **_STOP_REJECTED_ERROR_DETAILS,
    }
)
_RENEW_LIVE_MOTION_EXACT_REJECTED_ERROR_DETAILS = MappingProxyType(
    {
        **_LIVE_MOTION_CONTROL_PARSE_REJECTED_ERROR_DETAILS,
        **_RENEW_LIVE_MOTION_REJECTED_ERROR_DETAILS,
    }
)


class JsonCommandSchemaError(ValueError):
    """A command-domain value violates an exact protocol schema."""


def _require_exact_object(value, fields, field_name):
    if type(value) not in (dict, MappingProxyType):
        raise JsonCommandSchemaError(f"{field_name} must be an object")
    if frozenset(value) != fields:
        raise JsonCommandSchemaError(
            f"{field_name} fields do not match the protocol schema"
        )
    return value


def _require_text(value, field_name):
    if (
        type(value) is not str
        or not value
        or len(value) > JSON_HELLO_MAXIMUM_TEXT_LENGTH
        or not value.isascii()
        or any(
            ord(character) < 0x20 or ord(character) > 0x7E
            for character in value
        )
    ):
        raise JsonCommandSchemaError(f"{field_name} is invalid")
    return value


def _require_bounded_ascii_text(value, field_name, *, maximum_length):
    if (
        type(value) is not str
        or len(value) > maximum_length
        or not value.isascii()
        or any(
            ord(character) < 0x20 or ord(character) > 0x7E
            for character in value
        )
    ):
        raise JsonCommandSchemaError(f"{field_name} is invalid")
    return value


def _require_request_identifier(value, field_name):
    if (
        type(value) is not int
        or value < 1
        or value > JSON_PROTOCOL_MAXIMUM_IDENTIFIER
    ):
        raise JsonCommandSchemaError(f"{field_name} is invalid")
    return value


def _require_unsigned_integer(value, field_name, *, maximum):
    if type(value) is not int or value < 0 or value > maximum:
        raise JsonCommandSchemaError(f"{field_name} is invalid")
    return value


def validate_main_configuration_fingerprint(
    value,
    field_name="configuration fingerprint",
):
    if (
        type(value) is not str
        or _CONFIGURATION_FINGERPRINT_PATTERN.fullmatch(value) is None
    ):
        raise JsonCommandSchemaError(f"{field_name} is invalid")
    return value


def _require_controller_hardware_id(value):
    if (
        type(value) is not str
        or _CONTROLLER_HARDWARE_ID_PATTERN.fullmatch(value) is None
    ):
        raise JsonCommandSchemaError("controller hardware ID is invalid")
    return value


def _require_session_identifier(value):
    if (
        type(value) is not str
        or _SESSION_IDENTIFIER_PATTERN.fullmatch(value) is None
    ):
        raise JsonCommandSchemaError("controller session ID is invalid")
    return value


def _require_capabilities(value):
    if (
        type(value) not in (list, tuple)
        or not value
        or len(value) > JSON_HELLO_MAXIMUM_CAPABILITIES
    ):
        raise JsonCommandSchemaError(
            "controller capabilities must be a bounded array"
        )
    capabilities = []
    for capability in value:
        if (
            type(capability) is not str
            or _CAPABILITY_PATTERN.fullmatch(capability) is None
            or capability in capabilities
        ):
            raise JsonCommandSchemaError(
                "controller capability is invalid or duplicated"
            )
        capabilities.append(capability)
    normalized = tuple(capabilities)
    if frozenset(normalized) != JSON_REQUIRED_SESSION_CAPABILITIES:
        raise JsonCommandSchemaError(
            "controller capabilities do not match JSON protocol v1"
        )
    return normalized


def _require_command_manifest(value, expected, field_name):
    if (
        type(value) not in (list, tuple)
        or tuple(value) != expected
        or len(set(value)) != len(value)
    ):
        raise JsonCommandSchemaError(
            f"{field_name} does not match the active command manifest"
        )
    return tuple(value)


def _require_controller_media_identifier(value):
    if (
        type(value) is not str
        or _CONTROLLER_MEDIA_IDENTIFIER_PATTERN.fullmatch(value) is None
    ):
        raise JsonCommandSchemaError(
            "controller media identifier is invalid"
        )
    return value


def _require_controller_filename(value):
    if (
        type(value) is not str
        or not value
        or value in (".", "..")
        or value[0] == " "
        or value[-1] == " "
        or any(
            character in _CONTROLLER_FILENAME_RESERVED_CHARACTERS
            or ord(character) < 32
            or ord(character) > 126
            for character in value
        )
        or len(value.encode("ascii"))
            > JSON_CONTROLLER_FILENAME_MAXIMUM_BYTES
    ):
        raise JsonCommandSchemaError("controller filename is invalid")
    return value


def _require_signed_int32_tuple(value, length, field_name):
    if type(value) not in (list, tuple) or len(value) != length:
        raise JsonCommandSchemaError(
            f"{field_name} must contain exactly {length} values"
        )
    normalized = []
    for item in value:
        if (
            type(item) is not int
            or item < _SIGNED_INT32_MINIMUM
            or item > _SIGNED_INT32_MAXIMUM
        ):
            raise JsonCommandSchemaError(
                f"{field_name} contains an invalid signed 32-bit value"
            )
        normalized.append(item)
    return tuple(normalized)


def _require_controller_float_tuple(value, length, field_name):
    if type(value) not in (list, tuple) or len(value) != length:
        raise JsonCommandSchemaError(
            f"{field_name} must contain exactly {length} values"
        )
    normalized = []
    for item in value:
        if type(item) not in (int, float):
            raise JsonCommandSchemaError(
                f"{field_name} contains an invalid controller number"
            )
        converted = _controller_float32(item)
        if converted is None:
            raise JsonCommandSchemaError(
                f"{field_name} contains an invalid controller number"
            )
        normalized.append(converted)
    return tuple(normalized)


def _controller_float32(value):
    try:
        parsed = float(value)
        if not math.isfinite(parsed) or abs(parsed) > _FLOAT32_MAXIMUM:
            return None
        converted = struct.unpack("<f", struct.pack("<f", parsed))[0]
    except (OverflowError, struct.error, ValueError):
        return None
    if (
        not math.isfinite(converted)
        or (parsed != 0.0 and converted == 0.0)
    ):
        return None
    return converted


def _require_controller_degree_tuple(value, length, field_name):
    normalized = _require_controller_float_tuple(value, length, field_name)
    for degrees in normalized:
        converted = degrees * _RADIANS_PER_DEGREE
        radians = _controller_float32(converted)
        if (
            radians is None
            or abs(converted) > _FLOAT32_MAXIMUM
            or (degrees != 0.0 and radians == 0.0)
            or not math.isfinite(radians / _RADIANS_PER_DEGREE)
            or abs(radians / _RADIANS_PER_DEGREE) > _FLOAT32_MAXIMUM
        ):
            raise JsonCommandSchemaError(
                f"{field_name} contains an invalid controller angle"
            )
    return normalized


def _controller_axis_calibration_values(
    negative_limit,
    positive_limit,
    steps_per_unit,
    *,
    require_positive_travel=False,
):
    if (
        negative_limit < 0.0
        or positive_limit < 0.0
        or steps_per_unit <= 0.0
    ):
        return None
    travel = _controller_float32(negative_limit + positive_limit)
    if travel is None or (require_positive_travel and travel <= 0.0):
        return None
    step_limit = _controller_float32(travel * steps_per_unit)
    zero_step = _controller_float32(negative_limit * steps_per_unit)
    if not (
        step_limit is not None
        and zero_step is not None
        and 0.0 <= step_limit <= _SIGNED_INT32_MAXIMUM
        and 0.0 <= zero_step <= _SIGNED_INT32_MAXIMUM
        and int(zero_step) <= int(step_limit)
    ):
        return None
    return int(step_limit), int(zero_step)


def _controller_axis_calibration_valid(
    negative_limit,
    positive_limit,
    steps_per_unit,
):
    return (
        _controller_axis_calibration_values(
            negative_limit,
            positive_limit,
            steps_per_unit,
        )
        is not None
    )


def _controller_encoder_calibration_valid(
    step_limit,
    encoder_counts_per_step,
):
    if step_limit < 0 or encoder_counts_per_step <= 0.0:
        return False
    maximum_written_count = step_limit * encoder_counts_per_step
    return (
        math.isfinite(maximum_written_count)
        and 0.0 <= maximum_written_count <= _SIGNED_INT32_MAXIMUM
    )


def _require_binary_integer_tuple(value, length, field_name):
    if type(value) not in (list, tuple) or len(value) != length:
        raise JsonCommandSchemaError(
            f"{field_name} must contain exactly {length} values"
        )
    if any(type(item) is not int or item not in (0, 1) for item in value):
        raise JsonCommandSchemaError(
            f"{field_name} must contain only binary integers"
        )
    return tuple(value)


def _require_boolean_tuple(value, length, field_name):
    if type(value) not in (list, tuple) or len(value) != length:
        raise JsonCommandSchemaError(
            f"{field_name} must contain exactly {length} values"
        )
    if any(type(item) is not bool for item in value):
        raise JsonCommandSchemaError(
            f"{field_name} must contain only booleans"
        )
    return tuple(value)


@dataclass(frozen=True)
class JsonHelloFirmware:
    name: str
    version: str
    build: str

    def __post_init__(self):
        for value, field_name in (
            (self.name, "firmware name"),
            (self.version, "firmware version"),
            (self.build, "firmware build"),
        ):
            _require_text(value, field_name)


@dataclass(frozen=True)
class JsonHelloProtocol:
    name: str
    version: int
    maximum_payload_bytes: int

    def __post_init__(self):
        if self.name != JSON_PROTOCOL_NAME:
            raise JsonCommandSchemaError("controller protocol name is invalid")
        if (
            type(self.version) is not int
            or self.version != JSON_PROTOCOL_VERSION
        ):
            raise JsonCommandSchemaError("controller protocol version is invalid")
        if (
            type(self.maximum_payload_bytes) is not int
            or self.maximum_payload_bytes < 1
            or self.maximum_payload_bytes > JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES
        ):
            raise JsonCommandSchemaError(
                "controller protocol payload limit is invalid"
            )


@dataclass(frozen=True)
class JsonMainControllerIdentity:
    controller_hardware_id: str
    driver_model: str
    robot_model: str
    robot_version: str
    serial_number: str
    asset_tag: str

    def __post_init__(self):
        _require_controller_hardware_id(self.controller_hardware_id)
        for value, field_name in (
            (self.driver_model, "driver model"),
            (self.robot_model, "robot model"),
            (self.robot_version, "robot version"),
            (self.serial_number, "serial number"),
            (self.asset_tag, "asset tag"),
        ):
            _require_text(value, field_name)


@dataclass(frozen=True)
class JsonMainHelloResult:
    firmware: JsonHelloFirmware
    protocol: JsonHelloProtocol
    capabilities: tuple
    session_id: str
    identity: JsonMainControllerIdentity
    commands: tuple
    device: str = MAIN_CONTROLLER

    def __post_init__(self):
        if self.device != MAIN_CONTROLLER:
            raise JsonCommandSchemaError("controller device identity is invalid")
        if type(self.firmware) is not JsonHelloFirmware:
            raise JsonCommandSchemaError("controller firmware identity is invalid")
        if type(self.protocol) is not JsonHelloProtocol:
            raise JsonCommandSchemaError("controller protocol identity is invalid")
        if type(self.identity) is not JsonMainControllerIdentity:
            raise JsonCommandSchemaError("main-controller identity is invalid")
        _require_capabilities(self.capabilities)
        if type(self.capabilities) is not tuple:
            raise JsonCommandSchemaError(
                "controller capabilities must be an immutable tuple"
            )
        _require_session_identifier(self.session_id)
        _require_command_manifest(
            self.commands,
            JSON_MAIN_COMMAND_MANIFEST,
            "main-controller command manifest",
        )
        if type(self.commands) is not tuple:
            raise JsonCommandSchemaError(
                "main-controller command manifest must be immutable"
            )


@dataclass(frozen=True)
class JsonAuxiliaryHelloResult:
    board: str
    firmware: JsonHelloFirmware
    protocol: JsonHelloProtocol
    commands: tuple
    device: str = AUXILIARY_CONTROLLER

    def __post_init__(self):
        if self.device != AUXILIARY_CONTROLLER:
            raise JsonCommandSchemaError(
                "auxiliary-controller device identity is invalid"
            )
        if self.board not in ("nano", "mega"):
            raise JsonCommandSchemaError(
                "auxiliary-controller board identity is invalid"
            )
        if type(self.firmware) is not JsonHelloFirmware:
            raise JsonCommandSchemaError(
                "auxiliary-controller firmware identity is invalid"
            )
        if type(self.protocol) is not JsonHelloProtocol:
            raise JsonCommandSchemaError(
                "auxiliary-controller protocol identity is invalid"
            )
        _require_command_manifest(
            self.commands,
            JSON_AUXILIARY_COMMAND_MANIFEST,
            "auxiliary-controller command manifest",
        )
        if type(self.commands) is not tuple:
            raise JsonCommandSchemaError(
                "auxiliary-controller command manifest must be immutable"
            )


@dataclass(frozen=True)
class JsonScalarResult:
    value: int

    def __post_init__(self):
        if type(self.value) is not int:
            raise JsonCommandSchemaError("scalar result must be an integer")


@dataclass(frozen=True)
class JsonMainDeleteSdProgramResult:
    deleted: bool

    def __post_init__(self):
        if type(self.deleted) is not bool:
            raise JsonCommandSchemaError(
                "delete-SD-program disposition must be boolean"
            )


@dataclass(frozen=True)
class JsonMainSdProgramListResult:
    media_id: str
    files: tuple

    def __post_init__(self):
        _require_controller_media_identifier(self.media_id)
        if type(self.files) is not tuple:
            raise JsonCommandSchemaError(
                "SD-program files must be an immutable tuple"
            )
        normalized = tuple(
            _require_controller_filename(name) for name in self.files
        )
        if (
            len(normalized) != len(set(name.casefold() for name in normalized))
            or tuple(sorted(normalized, key=str.casefold)) != normalized
        ):
            raise JsonCommandSchemaError(
                "SD-program files must be unique and sorted"
            )


@dataclass(frozen=True)
class JsonAuxiliaryInputResult:
    state: bool

    def __post_init__(self):
        if type(self.state) is not bool:
            raise JsonCommandSchemaError(
                "auxiliary input state must be boolean"
            )


@dataclass(frozen=True)
class JsonAuxiliaryCurrentResult:
    amps: float

    def __post_init__(self):
        if (
            type(self.amps) is not float
            or not math.isfinite(self.amps)
            or self.amps < 0.0
            or self.amps > 28.0
        ):
            raise JsonCommandSchemaError(
                "auxiliary gripper current is out of range"
            )


@dataclass(frozen=True)
class JsonMainPositionResult:
    robot_joints_millidegrees: tuple
    external_axes_milliunits: tuple
    cartesian_micrometers: tuple
    orientation_millidegrees: tuple
    axis_source: str = JSON_POSITION_SOURCE_CONTROLLER_STEP_STATE
    speed_limited: bool = False
    controller_debug: str = ""
    motion_fault: str = ""

    def __post_init__(self):
        fields = (
            (
                self.robot_joints_millidegrees,
                JSON_POSITION_ROBOT_JOINT_COUNT,
                "robot joint positions",
            ),
            (
                self.external_axes_milliunits,
                JSON_POSITION_EXTERNAL_AXIS_COUNT,
                "external-axis positions",
            ),
            (
                self.cartesian_micrometers,
                JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
                "Cartesian translation",
            ),
            (
                self.orientation_millidegrees,
                JSON_POSITION_CARTESIAN_ORIENTATION_COUNT,
                "Cartesian orientation",
            ),
        )
        for value, length, field_name in fields:
            if type(value) is not tuple:
                raise JsonCommandSchemaError(
                    f"{field_name} must be an immutable tuple"
                )
            _require_signed_int32_tuple(
                value,
                length,
                field_name,
            )
        if self.axis_source != JSON_POSITION_SOURCE_CONTROLLER_STEP_STATE:
            raise JsonCommandSchemaError("position axis source is invalid")
        if type(self.speed_limited) is not bool:
            raise JsonCommandSchemaError(
                "position speed-limited state must be boolean"
            )
        if (
            type(self.controller_debug) is not str
            or len(self.controller_debug)
            > _POSITION_CONTROLLER_DEBUG_MAXIMUM_LENGTH
            or _POSITION_CONTROLLER_DEBUG_PATTERN.fullmatch(
                self.controller_debug
            ) is None
        ):
            raise JsonCommandSchemaError(
                "position controller debug value is invalid"
            )
        if (
            type(self.motion_fault) is not str
            or _POSITION_MOTION_FAULT_PATTERN.fullmatch(
                self.motion_fault
            ) is None
        ):
            raise JsonCommandSchemaError("position motion fault is invalid")

    @property
    def robot_joints_degrees(self):
        """Return J1-J6 in degrees."""
        return tuple(
            value / 1000.0
            for value in self.robot_joints_millidegrees
        )

    @property
    def external_axes(self):
        """Return J7-J9 in each configured external-axis unit."""
        return tuple(
            value / 1000.0
            for value in self.external_axes_milliunits
        )

    @property
    def cartesian_translation_millimeters(self):
        """Return Cartesian X, Y, and Z in millimeters."""
        return tuple(
            value / 1000.0
            for value in self.cartesian_micrometers
        )

    @property
    def orientation_degrees(self):
        """Return Cartesian orientation in firmware order Rz, Ry, Rx."""
        return tuple(
            value / 1000.0
            for value in self.orientation_millidegrees
        )


@dataclass(frozen=True)
class JsonMainJointMotionResult:
    position: JsonMainPositionResult
    speed_limited: bool
    controller_debug: str

    def __post_init__(self):
        if type(self.position) is not JsonMainPositionResult:
            raise JsonCommandSchemaError(
                "move-joints position result is invalid"
            )
        if type(self.speed_limited) is not bool:
            raise JsonCommandSchemaError(
                "move-joints speed-limited state must be boolean"
            )
        _require_bounded_ascii_text(
            self.controller_debug,
            "move-joints controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        )


@dataclass(frozen=True)
class JsonMainMotionTraceRecord:
    controller_microseconds: int
    master_index: int
    scheduled_delay_microseconds: int
    commanded_steps: tuple
    encoder_counts: tuple
    phase: int
    flags: int

    def __post_init__(self):
        _require_unsigned_integer(
            self.controller_microseconds,
            "motion-trace controller time",
            maximum=0xFFFFFFFF,
        )
        _require_unsigned_integer(
            self.master_index,
            "motion-trace master index",
            maximum=0xFFFFFFFF,
        )
        _require_unsigned_integer(
            self.scheduled_delay_microseconds,
            "motion-trace scheduled delay",
            maximum=0xFFFFFFFF,
        )
        if type(self.commanded_steps) is not tuple:
            raise JsonCommandSchemaError(
                "motion-trace commanded steps must be immutable"
            )
        _require_signed_int32_tuple(
            self.commanded_steps,
            JSON_POSITION_ROBOT_JOINT_COUNT,
            "motion-trace commanded steps",
        )
        if type(self.encoder_counts) is not tuple:
            raise JsonCommandSchemaError(
                "motion-trace encoder counts must be immutable"
            )
        _require_signed_int32_tuple(
            self.encoder_counts,
            JSON_POSITION_ROBOT_JOINT_COUNT,
            "motion-trace encoder counts",
        )
        _require_unsigned_integer(
            self.phase,
            "motion-trace phase",
            maximum=2,
        )
        _require_unsigned_integer(
            self.flags,
            "motion-trace flags",
            maximum=0x03,
        )


@dataclass(frozen=True)
class JsonMainMotionTraceDisposition:
    complete: bool
    capacity_limited: bool
    clock_wrapped: bool
    motion_outcome: str
    timing_overrun: bool

    def __post_init__(self):
        if any(
            type(value) is not bool
            for value in (
                self.complete,
                self.capacity_limited,
                self.clock_wrapped,
                self.timing_overrun,
            )
        ):
            raise JsonCommandSchemaError(
                "motion-trace disposition flags must be boolean"
            )
        if self.motion_outcome not in ("cancelled", "completed", "failed"):
            raise JsonCommandSchemaError(
                "motion-trace motion outcome is invalid"
            )


@dataclass(frozen=True)
class JsonMainMotionTraceNoCaptureResult:
    source_motion_request_id: int
    capture_state: str = "no_capture"

    def __post_init__(self):
        if self.capture_state != "no_capture":
            raise JsonCommandSchemaError(
                "motion-trace empty capture state is invalid"
            )
        _require_request_identifier(
            self.source_motion_request_id,
            "motion-trace source request identifier",
        )


@dataclass(frozen=True)
class JsonMainMotionTracePageResult:
    capture_generation: int
    configuration_fingerprint: str
    disposition: JsonMainMotionTraceDisposition
    firmware: JsonHelloFirmware
    page_count: int
    page_index: int
    record_start: int
    records: tuple
    source_motion_request_id: int
    source_session_id: str
    total_records: int
    capture_state: str = "available"

    def __post_init__(self):
        if self.capture_state != "available":
            raise JsonCommandSchemaError(
                "motion-trace available capture state is invalid"
            )
        _require_request_identifier(
            self.capture_generation,
            "motion-trace capture generation",
        )
        validate_main_configuration_fingerprint(
            self.configuration_fingerprint,
            "motion-trace configuration fingerprint",
        )
        if type(self.disposition) is not JsonMainMotionTraceDisposition:
            raise JsonCommandSchemaError(
                "motion-trace disposition is invalid"
            )
        if (
            type(self.firmware) is not JsonHelloFirmware
            or (
                self.firmware.name,
                self.firmware.version,
                self.firmware.build,
            ) != _JSON_MAIN_FIRMWARE_IDENTITY
        ):
            raise JsonCommandSchemaError(
                "motion-trace firmware identity is invalid"
            )
        _require_unsigned_integer(
            self.total_records,
            "motion-trace total record count",
            maximum=JSON_MOTION_TRACE_RECORD_CAPACITY,
        )
        if self.total_records < 1:
            raise JsonCommandSchemaError(
                "motion-trace total record count is invalid"
            )
        expected_page_count = (
            self.total_records + JSON_MOTION_TRACE_PAGE_RECORDS - 1
        ) // JSON_MOTION_TRACE_PAGE_RECORDS
        if self.page_count != expected_page_count:
            raise JsonCommandSchemaError(
                "motion-trace page count is invalid"
            )
        if (
            type(self.page_index) is not int
            or self.page_index < 0
            or self.page_index >= self.page_count
            or self.record_start
            != self.page_index * JSON_MOTION_TRACE_PAGE_RECORDS
        ):
            raise JsonCommandSchemaError(
                "motion-trace page bounds are invalid"
            )
        expected_records = min(
            JSON_MOTION_TRACE_PAGE_RECORDS,
            self.total_records - self.record_start,
        )
        if (
            type(self.records) is not tuple
            or len(self.records) != expected_records
            or any(
                type(record) is not JsonMainMotionTraceRecord
                for record in self.records
            )
        ):
            raise JsonCommandSchemaError(
                "motion-trace page records are invalid"
            )
        _require_request_identifier(
            self.source_motion_request_id,
            "motion-trace source request identifier",
        )
        _require_session_identifier(self.source_session_id)


@dataclass(frozen=True)
class JsonMainCalibrationResult:
    position: JsonMainPositionResult
    speed_limited: bool
    controller_debug: str

    def __post_init__(self):
        if type(self.position) is not JsonMainPositionResult:
            raise JsonCommandSchemaError(
                "calibration position result is invalid"
            )
        if type(self.speed_limited) is not bool:
            raise JsonCommandSchemaError(
                "calibration speed-limited state must be boolean"
            )
        _require_bounded_ascii_text(
            self.controller_debug,
            "calibration controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        )


@dataclass(frozen=True)
class JsonMainCartesianMotionResult:
    position: JsonMainPositionResult
    speed_limited: bool
    controller_debug: str

    def __post_init__(self):
        if type(self.position) is not JsonMainPositionResult:
            raise JsonCommandSchemaError(
                "Cartesian-motion position result is invalid"
            )
        if type(self.speed_limited) is not bool:
            raise JsonCommandSchemaError(
                "Cartesian-motion speed-limited state must be boolean"
            )
        _require_bounded_ascii_text(
            self.controller_debug,
            "Cartesian-motion controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        )


@dataclass(frozen=True)
class JsonMainToolJogResult:
    position: JsonMainPositionResult
    speed_limited: bool
    controller_debug: str

    def __post_init__(self):
        if type(self.position) is not JsonMainPositionResult:
            raise JsonCommandSchemaError(
                "tool-frame jog position result is invalid"
            )
        if type(self.speed_limited) is not bool:
            raise JsonCommandSchemaError(
                "tool-frame jog speed-limited state must be boolean"
            )
        _require_bounded_ascii_text(
            self.controller_debug,
            "tool-frame jog controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        )


@dataclass(frozen=True)
class JsonMainLiveJogResult:
    position: JsonMainPositionResult
    speed_limited: bool
    controller_debug: str

    def __post_init__(self):
        if type(self.position) is not JsonMainPositionResult:
            raise JsonCommandSchemaError(
                "live-jog position result is invalid"
            )
        if type(self.speed_limited) is not bool:
            raise JsonCommandSchemaError(
                "live-jog speed-limited state must be boolean"
            )
        _require_bounded_ascii_text(
            self.controller_debug,
            "live-jog controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        )


@dataclass(frozen=True)
class JsonMainStopResult:
    motion_id: int

    def __post_init__(self):
        _require_request_identifier(
            self.motion_id,
            "stop result motion identifier",
        )


@dataclass(frozen=True)
class JsonMainRenewLiveMotionResult:
    motion_id: int

    def __post_init__(self):
        _require_request_identifier(
            self.motion_id,
            "live-motion renewal result motion identifier",
        )


@dataclass(frozen=True)
class JsonMainJointTelemetrySample:
    robot_joints_millidegrees: tuple

    def __post_init__(self):
        if type(self.robot_joints_millidegrees) is not tuple:
            raise JsonCommandSchemaError(
                "joint telemetry positions must be an immutable tuple"
            )
        _require_signed_int32_tuple(
            self.robot_joints_millidegrees,
            JSON_POSITION_ROBOT_JOINT_COUNT,
            "joint telemetry positions",
        )

    @property
    def robot_joints_degrees(self):
        return tuple(
            value / 1000.0
            for value in self.robot_joints_millidegrees
        )


@dataclass(frozen=True)
class JsonMainHomeReferenceResult:
    valid: tuple
    positions_millidegrees: tuple

    def __post_init__(self):
        if (
            type(self.valid) is not tuple
            or len(self.valid) != JSON_HOME_REFERENCE_AXIS_COUNT
            or any(type(value) is not bool for value in self.valid)
        ):
            raise JsonCommandSchemaError(
                "home-reference validity must contain three booleans"
            )
        if type(self.positions_millidegrees) is not tuple:
            raise JsonCommandSchemaError(
                "home-reference positions must be an immutable tuple"
            )
        _require_signed_int32_tuple(
            self.positions_millidegrees,
            JSON_HOME_REFERENCE_AXIS_COUNT,
            "home-reference positions",
        )
        if any(
            not valid and position != 0
            for valid, position in zip(
                self.valid,
                self.positions_millidegrees,
            )
        ):
            raise JsonCommandSchemaError(
                "invalid home references must use zero positions"
            )

    @property
    def positions_degrees(self):
        """Return J1-J3 home-reference positions in degrees."""
        return tuple(
            value / 1000.0
            for value in self.positions_millidegrees
        )


def parse_main_hello_result(result):
    """Parse one completed main-controller ``hello`` result."""
    payload = _require_exact_object(
        result,
        _HELLO_RESULT_FIELDS,
        "hello result",
    )
    if payload["device"] != MAIN_CONTROLLER:
        raise JsonCommandSchemaError("controller device identity is invalid")

    firmware_payload = _require_exact_object(
        payload["firmware"],
        _HELLO_FIRMWARE_FIELDS,
        "hello firmware",
    )
    firmware = JsonHelloFirmware(
        name=firmware_payload["name"],
        version=firmware_payload["version"],
        build=firmware_payload["build"],
    )
    if (firmware.name, firmware.version, firmware.build) != (
        _JSON_MAIN_FIRMWARE_IDENTITY
    ):
        raise JsonCommandSchemaError("controller firmware identity is invalid")

    protocol_payload = _require_exact_object(
        payload["protocol"],
        _HELLO_PROTOCOL_FIELDS,
        "hello protocol",
    )
    protocol = JsonHelloProtocol(
        name=protocol_payload["name"],
        version=protocol_payload["version"],
        maximum_payload_bytes=protocol_payload["max_payload_bytes"],
    )

    identity_payload = _require_exact_object(
        payload["identity"],
        _HELLO_IDENTITY_FIELDS,
        "hello identity",
    )
    identity = JsonMainControllerIdentity(
        controller_hardware_id=identity_payload["controller_hardware_id"],
        driver_model=identity_payload["driver_model"],
        robot_model=identity_payload["robot_model"],
        robot_version=identity_payload["robot_version"],
        serial_number=identity_payload["serial_number"],
        asset_tag=identity_payload["asset_tag"],
    )

    return JsonMainHelloResult(
        firmware=firmware,
        protocol=protocol,
        capabilities=_require_capabilities(payload["capabilities"]),
        session_id=_require_session_identifier(payload["session_id"]),
        identity=identity,
        device=payload["device"],
        commands=_require_command_manifest(
            payload["commands"],
            JSON_MAIN_COMMAND_MANIFEST,
            "main-controller command manifest",
        ),
    )


def parse_auxiliary_hello_result(result):
    payload = _require_exact_object(
        result,
        _AUXILIARY_HELLO_RESULT_FIELDS,
        "auxiliary hello result",
    )
    firmware_payload = _require_exact_object(
        payload["firmware"],
        _HELLO_FIRMWARE_FIELDS,
        "auxiliary hello firmware",
    )
    protocol_payload = _require_exact_object(
        payload["protocol"],
        _HELLO_PROTOCOL_FIELDS,
        "auxiliary hello protocol",
    )
    board = payload["board"]
    firmware = JsonHelloFirmware(
        name=firmware_payload["name"],
        version=firmware_payload["version"],
        build=firmware_payload["build"],
    )
    if (firmware.name, firmware.version, firmware.build) != (
        _JSON_AUXILIARY_FIRMWARE_IDENTITIES.get(board)
    ):
        raise JsonCommandSchemaError("auxiliary firmware identity is invalid")
    return JsonAuxiliaryHelloResult(
        board=board,
        firmware=firmware,
        protocol=JsonHelloProtocol(
            name=protocol_payload["name"],
            version=protocol_payload["version"],
            maximum_payload_bytes=protocol_payload["max_payload_bytes"],
        ),
        commands=_require_command_manifest(
            payload["commands"],
            JSON_AUXILIARY_COMMAND_MANIFEST,
            "auxiliary-controller command manifest",
        ),
        device=payload["device"],
    )


def _main_position_result_from_payload(
    payload,
    *,
    speed_limited,
    controller_debug,
    motion_fault,
):
    return JsonMainPositionResult(
        robot_joints_millidegrees=_require_signed_int32_tuple(
            payload["robot_joints_millidegrees"],
            JSON_POSITION_ROBOT_JOINT_COUNT,
            "robot joint positions",
        ),
        external_axes_milliunits=_require_signed_int32_tuple(
            payload["external_axes_milliunits"],
            JSON_POSITION_EXTERNAL_AXIS_COUNT,
            "external-axis positions",
        ),
        cartesian_micrometers=_require_signed_int32_tuple(
            payload["cartesian_micrometers"],
            JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
            "Cartesian translation",
        ),
        orientation_millidegrees=_require_signed_int32_tuple(
            payload["orientation_millidegrees"],
            JSON_POSITION_CARTESIAN_ORIENTATION_COUNT,
            "Cartesian orientation",
        ),
        axis_source=payload["axis_source"],
        speed_limited=speed_limited,
        controller_debug=controller_debug,
        motion_fault=motion_fault,
    )


def parse_main_position_result(result):
    """Parse one completed main-controller position-bearing result."""
    payload = _require_exact_object(
        result,
        _POSITION_RESULT_FIELDS,
        "get-position result",
    )
    return _main_position_result_from_payload(
        payload,
        speed_limited=False,
        controller_debug="",
        motion_fault="",
    )


def parse_main_position_disposition_result(result):
    """Parse one capability-gated position-disposition result."""
    return _parse_main_position_disposition_result(
        result,
        "get-position-disposition result",
    )


def parse_main_position_correction_result(result):
    """Parse one completed position-correction result."""
    return _parse_main_position_disposition_result(
        result,
        "position-correction result",
    )


def _parse_main_position_disposition_result(result, field_name):
    payload = _require_exact_object(
        result,
        _POSITION_DISPOSITION_RESULT_FIELDS,
        field_name,
    )
    return _main_position_result_from_payload(
        payload,
        speed_limited=payload["speed_limited"],
        controller_debug=payload["controller_debug"],
        motion_fault=payload["motion_fault"],
    )


def parse_main_motion_position_result(result):
    """Parse one nested position snapshot from a motion result."""
    payload = _require_exact_object(
        result,
        _POSITION_SNAPSHOT_FIELDS,
        "motion position result",
    )
    return _main_position_result_from_payload(
        payload,
        speed_limited=False,
        controller_debug="",
        motion_fault="",
    )


def parse_main_limit_switches_result(result):
    payload = _require_exact_object(
        result,
        frozenset(("active",)),
        "limit-switch result",
    )
    active = payload["active"]
    if (
        type(active) not in (list, tuple)
        or len(active) != JSON_POSITION_ROBOT_JOINT_COUNT
        or any(type(value) is not bool for value in active)
    ):
        raise JsonCommandSchemaError("limit-switch states are invalid")
    return tuple(active)


def parse_main_encoder_counts_result(result):
    payload = _require_exact_object(
        result,
        frozenset(("counts",)),
        "encoder-count result",
    )
    return _require_signed_int32_tuple(
        payload["counts"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "encoder counts",
    )


def parse_main_move_joints_result(result):
    payload = _require_exact_object(
        result,
        _MOVE_JOINTS_RESULT_FIELDS,
        "move-joints result",
    )
    if type(payload["speed_limited"]) is not bool:
        raise JsonCommandSchemaError(
            "move-joints speed-limited state must be boolean"
        )
    return JsonMainJointMotionResult(
        position=parse_main_motion_position_result(payload["position"]),
        speed_limited=payload["speed_limited"],
        controller_debug=_require_bounded_ascii_text(
            payload["controller_debug"],
            "move-joints controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        ),
    )


def parse_main_motion_trace_result(result):
    if type(result) not in (dict, MappingProxyType):
        raise JsonCommandSchemaError(
            "motion-trace result must be an object"
        )
    capture_state = result.get("capture_state")
    if capture_state == "no_capture":
        payload = _require_exact_object(
            result,
            _MOTION_TRACE_NO_CAPTURE_RESULT_FIELDS,
            "motion-trace empty result",
        )
        return JsonMainMotionTraceNoCaptureResult(
            source_motion_request_id=_require_request_identifier(
                payload["source_motion_request_id"],
                "motion-trace source request identifier",
            ),
        )
    if capture_state != "available":
        raise JsonCommandSchemaError("motion-trace capture state is invalid")
    payload = _require_exact_object(
        result,
        _MOTION_TRACE_PAGE_RESULT_FIELDS,
        "motion-trace page result",
    )
    disposition_payload = _require_exact_object(
        payload["disposition"],
        _MOTION_TRACE_DISPOSITION_FIELDS,
        "motion-trace disposition",
    )
    disposition = JsonMainMotionTraceDisposition(
        complete=disposition_payload["complete"],
        capacity_limited=disposition_payload["capacity_limited"],
        clock_wrapped=disposition_payload["clock_wrapped"],
        motion_outcome=disposition_payload["motion_outcome"],
        timing_overrun=disposition_payload["timing_overrun"],
    )
    firmware_payload = _require_exact_object(
        payload["firmware"],
        _HELLO_FIRMWARE_FIELDS,
        "motion-trace firmware",
    )
    firmware = JsonHelloFirmware(
        name=firmware_payload["name"],
        version=firmware_payload["version"],
        build=firmware_payload["build"],
    )
    records_payload = payload["records"]
    if type(records_payload) not in (list, tuple):
        raise JsonCommandSchemaError(
            "motion-trace records must be an array"
        )
    records = []
    for record_payload in records_payload:
        record = _require_exact_object(
            record_payload,
            _MOTION_TRACE_RECORD_FIELDS,
            "motion-trace record",
        )
        records.append(JsonMainMotionTraceRecord(
            controller_microseconds=record["controller_microseconds"],
            master_index=record["master_index"],
            scheduled_delay_microseconds=(
                record["scheduled_delay_microseconds"]
            ),
            commanded_steps=_require_signed_int32_tuple(
                record["commanded_steps"],
                JSON_POSITION_ROBOT_JOINT_COUNT,
                "motion-trace commanded steps",
            ),
            encoder_counts=_require_signed_int32_tuple(
                record["encoder_counts"],
                JSON_POSITION_ROBOT_JOINT_COUNT,
                "motion-trace encoder counts",
            ),
            phase=record["phase"],
            flags=record["flags"],
        ))
    return JsonMainMotionTracePageResult(
        capture_generation=payload["capture_generation"],
        configuration_fingerprint=payload["configuration_fingerprint"],
        disposition=disposition,
        firmware=firmware,
        page_count=payload["page_count"],
        page_index=payload["page_index"],
        record_start=payload["record_start"],
        records=tuple(records),
        source_motion_request_id=payload["source_motion_request_id"],
        source_session_id=payload["source_session_id"],
        total_records=payload["total_records"],
    )


def parse_main_calibration_result(result):
    payload = _require_exact_object(
        result,
        _CALIBRATE_RESULT_FIELDS,
        "calibration result",
    )
    if type(payload["speed_limited"]) is not bool:
        raise JsonCommandSchemaError(
            "calibration speed-limited state must be boolean"
        )
    return JsonMainCalibrationResult(
        position=parse_main_motion_position_result(payload["position"]),
        speed_limited=payload["speed_limited"],
        controller_debug=_require_bounded_ascii_text(
            payload["controller_debug"],
            "calibration controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        ),
    )


def parse_main_move_cartesian_result(result):
    payload = _require_exact_object(
        result,
        _MOVE_JOINTS_RESULT_FIELDS,
        "Cartesian-motion result",
    )
    if type(payload["speed_limited"]) is not bool:
        raise JsonCommandSchemaError(
            "Cartesian-motion speed-limited state must be boolean"
        )
    return JsonMainCartesianMotionResult(
        position=parse_main_motion_position_result(payload["position"]),
        speed_limited=payload["speed_limited"],
        controller_debug=_require_bounded_ascii_text(
            payload["controller_debug"],
            "Cartesian-motion controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        ),
    )


def parse_main_tool_jog_result(result):
    payload = _require_exact_object(
        result,
        _MOVE_JOINTS_RESULT_FIELDS,
        "tool-frame jog result",
    )
    if type(payload["speed_limited"]) is not bool:
        raise JsonCommandSchemaError(
            "tool-frame jog speed-limited state must be boolean"
        )
    return JsonMainToolJogResult(
        position=parse_main_motion_position_result(payload["position"]),
        speed_limited=payload["speed_limited"],
        controller_debug=_require_bounded_ascii_text(
            payload["controller_debug"],
            "tool-frame jog controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        ),
    )


def parse_main_live_jog_result(result):
    payload = _require_exact_object(
        result,
        _MOVE_JOINTS_RESULT_FIELDS,
        "live-jog result",
    )
    if type(payload["speed_limited"]) is not bool:
        raise JsonCommandSchemaError(
            "live-jog speed-limited state must be boolean"
        )
    return JsonMainLiveJogResult(
        position=parse_main_motion_position_result(payload["position"]),
        speed_limited=payload["speed_limited"],
        controller_debug=_require_bounded_ascii_text(
            payload["controller_debug"],
            "live-jog controller debug value",
            maximum_length=JSON_HELLO_MAXIMUM_TEXT_LENGTH,
        ),
    )


def parse_main_stop_result(result):
    payload = _require_exact_object(
        result,
        _STOP_RESULT_FIELDS,
        "stop result",
    )
    return JsonMainStopResult(
        motion_id=_require_request_identifier(
            payload["motion_id"],
            "stop result motion identifier",
        )
    )


def parse_main_renew_live_motion_result(result):
    payload = _require_exact_object(
        result,
        _RENEW_LIVE_MOTION_RESULT_FIELDS,
        "live-motion renewal result",
    )
    return JsonMainRenewLiveMotionResult(
        motion_id=_require_request_identifier(
            payload["motion_id"],
            "live-motion renewal result motion identifier",
        )
    )


def parse_main_joint_position_telemetry(telemetry):
    if (
        type(telemetry) is not Telemetry
        or telemetry.stream != "joint_position"
    ):
        raise JsonCommandSchemaError(
            "joint-position telemetry envelope is invalid"
        )
    payload = _require_exact_object(
        telemetry.data,
        _MOVE_JOINTS_TELEMETRY_FIELDS,
        "joint-position telemetry data",
    )
    return JsonMainJointTelemetrySample(
        robot_joints_millidegrees=_require_signed_int32_tuple(
            payload["robot_joints_millidegrees"],
            JSON_POSITION_ROBOT_JOINT_COUNT,
            "joint telemetry positions",
        )
    )


def parse_main_home_reference_result(result):
    """Parse one completed main-controller ``get_home_reference`` result."""
    payload = _require_exact_object(
        result,
        _HOME_REFERENCE_RESULT_FIELDS,
        "get-home-reference result",
    )
    valid = payload["valid"]
    if (
        type(valid) not in (list, tuple)
        or len(valid) != JSON_HOME_REFERENCE_AXIS_COUNT
        or any(type(value) is not bool for value in valid)
    ):
        raise JsonCommandSchemaError(
            "home-reference validity must contain three booleans"
        )
    return JsonMainHomeReferenceResult(
        valid=tuple(valid),
        positions_millidegrees=_require_signed_int32_tuple(
            payload["positions_millidegrees"],
            JSON_HOME_REFERENCE_AXIS_COUNT,
            "home-reference positions",
        ),
    )


def validate_main_hello_request(params):
    """Validate the side-effect-free main-controller session probe."""
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError("hello request parameters must be empty")


def _validate_exact_main_rejection(error, response_name, error_details):
    expected_details = error_details.get(error.code)
    if expected_details is None:
        raise JsonCommandSchemaError(
            f"{response_name} rejection code is invalid"
        )
    details = _require_exact_object(
        error.details,
        frozenset(expected_details),
        f"{response_name} rejection details",
    )
    if any(
        details[field] != expected_value
        for field, expected_value in expected_details.items()
    ):
        raise JsonCommandSchemaError(
            f"{response_name} rejection details are invalid"
        )


def _validate_main_rejection(
    error,
    response_name,
    additional_error_details=None,
):
    error_details = dict(_MAIN_REJECTED_ERROR_DETAILS)
    if additional_error_details is not None:
        error_details.update(additional_error_details)
    _validate_exact_main_rejection(error, response_name, error_details)


def _validate_hello_failure(response):
    error = response.error
    if response.status == "rejected":
        _validate_main_rejection(error, "hello")
        return
    if response.status == "failed":
        if error.code not in _HELLO_FAILED_ERROR_CODES:
            raise JsonCommandSchemaError("hello failure code is invalid")
        _require_exact_object(
            error.details,
            frozenset(),
            "hello failure details",
        )
        return
    raise JsonCommandSchemaError("hello response status is invalid")


def validate_main_hello_response(response):
    """Validate one correlated main-controller session-probe response."""
    if type(response) is not Response or response.cmd != "hello":
        raise JsonCommandSchemaError("hello response envelope is invalid")
    if response.status == "completed":
        result = parse_main_hello_result(response.result)
        payload_length = len(encode_message(response)) - 1
        if payload_length > result.protocol.maximum_payload_bytes:
            raise JsonCommandSchemaError(
                "hello response exceeds the advertised payload limit"
            )
        return
    _validate_hello_failure(response)


def validate_main_get_position_disposition_request(params):
    """Validate the capability-gated position-disposition request."""
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError(
            "get-position-disposition request parameters must be empty"
        )


def validate_main_correct_position_request(params):
    """Validate one encoder-backed controller-position correction."""
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError(
            "correct-position request parameters must be empty"
        )


def validate_main_get_home_reference_request(params):
    """Validate the side-effect-free main-controller home-reference request."""
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError(
            "get-home-reference request parameters must be empty"
        )


def validate_main_diagnostic_request(params):
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError(
            "controller diagnostic request parameters must be empty"
        )


def validate_main_set_position_request(params):
    """Validate one fixed-point controller-position replacement."""
    payload = _require_exact_object(
        params,
        _SET_POSITION_REQUEST_FIELDS,
        "set-position request parameters",
    )
    _require_signed_int32_tuple(
        payload["robot_joints_millidegrees"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "set-position robot joints",
    )
    _require_signed_int32_tuple(
        payload["external_axes_milliunits"],
        JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "set-position external axes",
    )


def validate_main_update_params_request(params):
    payload = _require_exact_object(
        params,
        _UPDATE_PARAMS_REQUEST_FIELDS,
        "update-params request parameters",
    )
    _require_controller_float_tuple(
        payload["tool_translation_millimeters"],
        3,
        "tool translation",
    )
    _require_controller_degree_tuple(
        payload["tool_rotation_degrees"],
        3,
        "tool rotation",
    )
    _require_binary_integer_tuple(
        payload["motor_directions"],
        9,
        "motor directions",
    )
    _require_binary_integer_tuple(
        payload["calibration_directions"],
        9,
        "calibration directions",
    )
    _require_boolean_tuple(
        payload["calibration_switch_active_high"],
        9,
        "calibration switch polarity",
    )
    positive_limits = _require_controller_float_tuple(
        payload["positive_joint_limits_degrees"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "positive joint limits",
    )
    negative_limits = _require_controller_float_tuple(
        payload["negative_joint_limits_degrees"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "negative joint limits",
    )
    steps_per_degree = _require_controller_float_tuple(
        payload["steps_per_degree"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "steps per degree",
    )
    encoder_counts = _require_controller_float_tuple(
        payload["encoder_counts_per_step"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "encoder counts per step",
    )
    axis_calibrations = tuple(
        _controller_axis_calibration_values(
            negative_limits[axis],
            positive_limits[axis],
            steps_per_degree[axis],
            require_positive_travel=True,
        )
        for axis in range(JSON_POSITION_ROBOT_JOINT_COUNT)
    )
    if any(calibration is None for calibration in axis_calibrations):
        raise JsonCommandSchemaError(
            "primary-axis calibration cannot be represented"
        )
    if any(
        not _controller_encoder_calibration_valid(
            axis_calibrations[axis][0],
            encoder_counts[axis],
        )
        for axis in range(JSON_POSITION_ROBOT_JOINT_COUNT)
    ):
        raise JsonCommandSchemaError(
            "encoder calibration cannot be represented"
        )
    _require_controller_degree_tuple(
        payload["dh_theta_degrees"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "DH theta",
    )
    _require_controller_degree_tuple(
        payload["dh_alpha_degrees"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "DH alpha",
    )
    _require_controller_float_tuple(
        payload["dh_d_millimeters"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "DH d",
    )
    _require_controller_float_tuple(
        payload["dh_a_millimeters"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "DH a",
    )


def validate_main_config_ext_axis_request(params):
    payload = _require_exact_object(
        params,
        _CONFIG_EXT_AXIS_REQUEST_FIELDS,
        "config-ext-axis request parameters",
    )
    travel_units = _require_controller_float_tuple(
        payload["travel_units"],
        JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "external-axis travel",
    )
    drive_rotations = _require_controller_float_tuple(
        payload["drive_rotations"],
        JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "external-axis drive rotations",
    )
    motor_steps = _require_controller_float_tuple(
        payload["motor_steps"],
        JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "external-axis motor steps",
    )
    for axis in range(JSON_POSITION_EXTERNAL_AXIS_COUNT):
        rotation = drive_rotations[axis]
        steps = motor_steps[axis]
        steps_per_unit = (
            _controller_float32(steps / rotation)
            if rotation > 0.0 and steps > 0.0
            else None
        )
        if (
            travel_units[axis] < 0.0
            or steps_per_unit is None
            or steps_per_unit <= 0.0
            or not _controller_axis_calibration_valid(
                0.0,
                travel_units[axis],
                steps_per_unit,
            )
        ):
            raise JsonCommandSchemaError(
                "external-axis calibration cannot be represented"
            )


def validate_main_external_axis_zero_request(params):
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError(
            "external-axis-zero request parameters must be empty"
        )


def validate_main_controller_wait_request(params):
    payload = _require_exact_object(
        params,
        _CONTROLLER_WAIT_REQUEST_FIELDS,
        "controller-wait request parameters",
    )
    seconds = payload["seconds"]
    _require_controller_float_tuple(
        (seconds,),
        1,
        "controller-wait duration",
    )
    if seconds < 0.0 or seconds > JSON_CONTROLLER_WAIT_MAXIMUM_SECONDS:
        raise JsonCommandSchemaError(
            "controller-wait duration is out of range"
        )


def _validate_modbus_target(payload):
    slave_id = payload["slave_id"]
    if type(slave_id) is not int or not 1 <= slave_id <= 247:
        raise JsonCommandSchemaError("Modbus slave ID is out of range")
    address = payload["address"]
    if type(address) is not int or not 0 <= address <= 65535:
        raise JsonCommandSchemaError("Modbus address is out of range")


def validate_main_modbus_read_request(params):
    payload = _require_exact_object(
        params,
        _MODBUS_READ_REQUEST_FIELDS,
        "Modbus-read request parameters",
    )
    _validate_modbus_target(payload)
    if type(payload["count"]) is not int or payload["count"] != 1:
        raise JsonCommandSchemaError("Modbus read count must equal one")


def parse_main_modbus_read_result(result, *, command):
    maximum_value = (
        _MODBUS_READ_VALUE_MAXIMUMS.get(command)
        if type(command) is str
        else None
    )
    if maximum_value is None:
        raise JsonCommandSchemaError("Modbus-read command is invalid")
    payload = _require_exact_object(
        result,
        _MODBUS_READ_RESULT_FIELDS,
        "Modbus-read result",
    )
    value = payload["value"]
    if type(value) is not int or not 0 <= value <= maximum_value:
        raise JsonCommandSchemaError("Modbus-read value is out of range")
    return value


def parse_main_modbus_wait_result(result, *, command):
    maximum_value = (
        _MODBUS_WAIT_VALUE_MAXIMUMS.get(command)
        if type(command) is str
        else None
    )
    if maximum_value is None:
        raise JsonCommandSchemaError("Modbus-wait command is invalid")
    payload = _require_exact_object(
        result,
        _MODBUS_READ_RESULT_FIELDS,
        "Modbus-wait result",
    )
    value = payload["value"]
    if type(value) is not int or not 0 <= value <= maximum_value:
        raise JsonCommandSchemaError("Modbus-wait value is invalid")
    return JsonScalarResult(value)


def parse_main_delete_sd_program_result(result):
    payload = _require_exact_object(
        result,
        _DELETE_SD_PROGRAM_RESULT_FIELDS,
        "delete-SD-program result",
    )
    if type(payload["deleted"]) is not bool:
        raise JsonCommandSchemaError(
            "delete-SD-program disposition must be boolean"
        )
    return JsonMainDeleteSdProgramResult(payload["deleted"])


def parse_main_list_sd_programs_result(result):
    payload = _require_exact_object(
        result,
        _CONTROLLER_DIRECTORY_RESULT_FIELDS,
        "list-SD-programs result",
    )
    media_id = _require_controller_media_identifier(payload["media_id"])
    files = payload["files"]
    if type(files) not in (list, tuple):
        raise JsonCommandSchemaError(
            "list-SD-programs files must be an array"
        )
    normalized = tuple(_require_controller_filename(name) for name in files)
    if (
        len(normalized) != len(set(name.casefold() for name in normalized))
        or tuple(sorted(normalized, key=str.casefold)) != normalized
    ):
        raise JsonCommandSchemaError(
            "list-SD-programs files must be unique and sorted"
        )
    return JsonMainSdProgramListResult(media_id, normalized)


def validate_main_modbus_write_request(params, *, command):
    maximum_value = (
        _MODBUS_WRITE_VALUE_MAXIMUMS.get(command)
        if type(command) is str
        else None
    )
    if maximum_value is None:
        raise JsonCommandSchemaError("Modbus-write command is invalid")
    payload = _require_exact_object(
        params,
        _MODBUS_WRITE_REQUEST_FIELDS,
        "Modbus-write request parameters",
    )
    _validate_modbus_target(payload)
    value = payload["value"]
    if type(value) is not int or not 0 <= value <= maximum_value:
        raise JsonCommandSchemaError("Modbus-write value is out of range")


def validate_main_modbus_wait_request(params, *, command):
    maximum_value = (
        _MODBUS_WAIT_VALUE_MAXIMUMS.get(command)
        if type(command) is str
        else None
    )
    if maximum_value is None:
        raise JsonCommandSchemaError("Modbus-wait command is invalid")
    payload = _require_exact_object(
        params,
        _MODBUS_WAIT_REQUEST_FIELDS,
        "Modbus-wait request parameters",
    )
    _validate_modbus_target(payload)
    if (
        type(payload["expected"]) is not int
        or not 0 <= payload["expected"] <= maximum_value
    ):
        raise JsonCommandSchemaError("Modbus-wait expected value is invalid")
    timeout = payload["timeout_seconds"]
    if (
        type(timeout) is not int
        or timeout < 1
        or timeout > JSON_CONTROLLER_WAIT_MAXIMUM_SECONDS
    ):
        raise JsonCommandSchemaError("Modbus-wait timeout is out of range")


def validate_main_empty_request(params):
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError("request parameters must be empty")


def validate_main_calibrate_request(params):
    payload = _require_exact_object(
        params,
        _CALIBRATE_REQUEST_FIELDS,
        "calibration request parameters",
    )
    axes = _require_boolean_tuple(
        payload["axes"],
        JSON_POSITION_ROBOT_JOINT_COUNT + JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "calibration axes",
    )
    if not any(axes):
        raise JsonCommandSchemaError(
            "calibration requires at least one selected axis"
        )
    _require_controller_float_tuple(
        payload["offsets"],
        JSON_POSITION_ROBOT_JOINT_COUNT + JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "calibration offsets",
    )


def _validate_main_motion_profile(payload, response_name, speed_modes):
    speed_mode = payload["speed_mode"]
    if speed_mode not in speed_modes:
        raise JsonCommandSchemaError(
            f"{response_name} speed mode is invalid"
        )
    speed = _require_controller_float_tuple(
        (payload["speed_value"],),
        1,
        f"{response_name} speed",
    )[0]
    acceleration = _require_controller_float_tuple(
        (payload["acceleration_percent"],),
        1,
        f"{response_name} acceleration",
    )[0]
    deceleration = _require_controller_float_tuple(
        (payload["deceleration_percent"],),
        1,
        f"{response_name} deceleration",
    )[0]
    ramp = _require_controller_float_tuple(
        (payload["ramp_percent"],),
        1,
        f"{response_name} ramp",
    )[0]
    if speed <= 0.0 or (speed_mode == "percent" and speed > 100.0):
        raise JsonCommandSchemaError(
            f"{response_name} speed is out of range"
        )
    if (
        speed_mode == "seconds"
        and _controller_float32(speed * 1000000.0) is None
    ):
        raise JsonCommandSchemaError(
            f"{response_name} duration cannot be represented"
        )
    combined_ramps = _controller_float32(acceleration + deceleration)
    if (
        acceleration <= 0.0
        or acceleration > 100.0
        or deceleration <= 0.0
        or deceleration >= 100.0
        or combined_ramps is None
        or combined_ramps > 100.0
        or ramp <= 0.0
        or ramp > 100.0
    ):
        raise JsonCommandSchemaError(
            f"{response_name} timing profile is out of range"
        )


def validate_main_move_joints_request(params):
    payload = _require_exact_object(
        params,
        _MOVE_JOINTS_REQUEST_FIELDS,
        "move-joints request parameters",
    )
    _require_controller_float_tuple(
        payload["robot_joints_degrees"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "move-joints robot targets",
    )
    _require_controller_float_tuple(
        payload["external_axes_units"],
        JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "move-joints external-axis targets",
    )
    _validate_main_motion_profile(
        payload,
        "move-joints",
        ("percent", "seconds", "millimeters_per_second"),
    )
    if payload["wrist_configuration"] not in (
        "automatic",
        "near",
        "far",
    ):
        raise JsonCommandSchemaError(
            "move-joints wrist configuration is invalid"
        )
    _require_boolean_tuple(
        payload["loop_modes"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "move-joints loop modes",
    )
    if type(payload["telemetry_enabled"]) is not bool:
        raise JsonCommandSchemaError(
            "move-joints telemetry state must be boolean"
        )
    trace_fingerprint = payload["trace_configuration_fingerprint"]
    if trace_fingerprint is not None:
        validate_main_configuration_fingerprint(
            trace_fingerprint,
            "move-joints trace configuration fingerprint",
        )
        if payload["telemetry_enabled"]:
            raise JsonCommandSchemaError(
                "traced move-joints telemetry must be disabled"
            )


def validate_main_motion_trace_request(params):
    payload = _require_exact_object(
        params,
        _MOTION_TRACE_REQUEST_FIELDS,
        "motion-trace request parameters",
    )
    _require_request_identifier(
        payload["motion_request_id"],
        "motion-trace source request identifier",
    )
    _require_unsigned_integer(
        payload["page_index"],
        "motion-trace page index",
        maximum=JSON_MOTION_TRACE_RECORD_CAPACITY - 1,
    )


def validate_main_move_cartesian_request(params):
    payload = _require_exact_object(
        params,
        _MOVE_CARTESIAN_REQUEST_FIELDS,
        "Cartesian-motion request parameters",
    )
    _require_controller_float_tuple(
        payload["translation_millimeters"],
        JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
        "Cartesian-motion translation target",
    )
    _require_controller_degree_tuple(
        payload["orientation_degrees"],
        JSON_POSITION_CARTESIAN_ORIENTATION_COUNT,
        "Cartesian-motion orientation target",
    )
    _require_controller_float_tuple(
        payload["external_axes_units"],
        JSON_POSITION_EXTERNAL_AXIS_COUNT,
        "Cartesian-motion external-axis targets",
    )
    _validate_main_motion_profile(
        payload,
        "Cartesian-motion",
        ("percent", "seconds", "millimeters_per_second"),
    )
    if payload["wrist_configuration"] not in (
        "automatic",
        "near",
        "far",
    ):
        raise JsonCommandSchemaError(
            "Cartesian-motion wrist configuration is invalid"
        )
    _require_boolean_tuple(
        payload["loop_modes"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "Cartesian-motion loop modes",
    )
    if type(payload["telemetry_enabled"]) is not bool:
        raise JsonCommandSchemaError(
            "Cartesian-motion telemetry state must be boolean"
        )


def validate_main_move_linear_request(params):
    payload = _require_exact_object(
        params,
        _MOVE_LINEAR_REQUEST_FIELDS,
        "linear-motion request parameters",
    )
    validate_main_move_cartesian_request(payload["motion"])
    rounding = _require_controller_float_tuple(
        (payload["rounding_millimeters"],),
        1,
        "linear-motion rounding",
    )[0]
    if rounding != 0.0:
        raise JsonCommandSchemaError(
            "linear-motion rounding must equal zero"
        )
    if payload["disable_wrist_rotation"] is not False:
        raise JsonCommandSchemaError(
            "linear-motion wrist suppression is not implemented"
        )


def validate_main_move_vision_request(params):
    payload = _require_exact_object(
        params,
        _MOVE_VISION_REQUEST_FIELDS,
        "vision-motion request parameters",
    )
    validate_main_move_cartesian_request(payload["motion"])
    _require_controller_degree_tuple(
        (payload["vision_rotation_degrees"],),
        1,
        "vision-motion rotation",
    )


def validate_main_move_arc_request(params):
    payload = _require_exact_object(
        params,
        _MOVE_ARC_REQUEST_FIELDS,
        "arc-motion request parameters",
    )
    validate_main_move_cartesian_request(payload["motion"])
    if payload["motion"]["telemetry_enabled"] is not False:
        raise JsonCommandSchemaError(
            "arc motion cannot enable telemetry"
        )
    _require_controller_float_tuple(
        payload["midpoint_translation_millimeters"],
        JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
        "arc-motion midpoint translation",
    )


def validate_main_move_circle_request(params):
    payload = _require_exact_object(
        params,
        _MOVE_CIRCLE_REQUEST_FIELDS,
        "circle-motion request parameters",
    )
    validate_main_move_cartesian_request(payload["motion"])
    if payload["motion"]["telemetry_enabled"] is not False:
        raise JsonCommandSchemaError(
            "circle motion cannot enable telemetry"
        )
    _require_controller_float_tuple(
        payload["center_translation_millimeters"],
        JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
        "circle-motion center translation",
    )
    _require_controller_float_tuple(
        payload["plane_translation_millimeters"],
        JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
        "circle-motion plane translation",
    )


def validate_main_move_spline_request(params):
    payload = _require_exact_object(
        params,
        _MOVE_SPLINE_REQUEST_FIELDS,
        "spline-motion request parameters",
    )
    segments = payload["segments"]
    if type(segments) not in (list, tuple) or not 1 <= len(segments) <= 6:
        raise JsonCommandSchemaError(
            "spline motion must contain one through six segments"
        )

    translations = []
    roundings = []
    for segment_index, segment in enumerate(segments):
        segment_payload = _require_exact_object(
            segment,
            _MOVE_SPLINE_SEGMENT_FIELDS,
            f"spline-motion segment {segment_index}",
        )
        motion = segment_payload["motion"]
        validate_main_move_cartesian_request(motion)
        if motion["telemetry_enabled"] is not False:
            raise JsonCommandSchemaError(
                "spline motion cannot enable telemetry"
            )
        translations.append(
            _require_controller_float_tuple(
                motion["translation_millimeters"],
                JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
                f"spline-motion segment {segment_index} translation",
            )
        )
        rounding = _require_controller_float_tuple(
            (segment_payload["rounding_millimeters"],),
            1,
            f"spline-motion segment {segment_index} rounding",
        )[0]
        if rounding < 0.0:
            raise JsonCommandSchemaError(
                "spline-motion rounding cannot be negative"
            )
        roundings.append(rounding)

    if roundings[-1] != 0.0:
        raise JsonCommandSchemaError(
            "final spline-motion rounding must equal zero"
        )
    for segment_index, rounding in enumerate(roundings[:-1]):
        if rounding == 0.0:
            continue
        outbound_distance = _controller_float32(math.sqrt(sum(
            (next_value - current_value) ** 2
            for current_value, next_value in zip(
                translations[segment_index],
                translations[segment_index + 1],
            )
        )))
        if outbound_distance is None or outbound_distance <= 0.0:
            raise JsonCommandSchemaError(
                "positive spline rounding requires a nondegenerate "
                "outbound translation leg"
            )
        maximum_rounding = _controller_float32(outbound_distance * 0.45)
        if maximum_rounding is None or rounding > maximum_rounding:
            raise JsonCommandSchemaError(
                "spline rounding exceeds 45 percent of an adjacent leg"
            )
        if segment_index == 0:
            continue
        inbound_distance = _controller_float32(math.sqrt(sum(
            (current_value - previous_value) ** 2
            for previous_value, current_value in zip(
                translations[segment_index - 1],
                translations[segment_index],
            )
        )))
        maximum_rounding = (
            _controller_float32(inbound_distance * 0.45)
            if inbound_distance is not None and inbound_distance > 0.0
            else None
        )
        if maximum_rounding is None:
            raise JsonCommandSchemaError(
                "positive spline rounding requires a nondegenerate "
                "inbound translation leg"
            )
        if rounding > maximum_rounding:
            raise JsonCommandSchemaError(
                "spline rounding exceeds 45 percent of an adjacent leg"
            )


def _validate_storage_target(payload, field_name):
    _require_controller_media_identifier(payload["media_id"])
    _require_controller_filename(payload["filename"])


def validate_main_storage_target_request(params):
    payload = _require_exact_object(
        params,
        _STORAGE_TARGET_REQUEST_FIELDS,
        "storage-target request parameters",
    )
    _validate_storage_target(payload, "storage target")


def validate_main_write_gcode_move_request(params):
    payload = _require_exact_object(
        params,
        _WRITE_GCODE_MOVE_REQUEST_FIELDS,
        "G-code write request parameters",
    )
    _validate_storage_target(payload, "G-code write target")
    validate_main_move_cartesian_request(payload["motion"])
    if payload["motion"]["telemetry_enabled"] is not False:
        raise JsonCommandSchemaError(
            "G-code storage writes cannot enable telemetry"
        )


def validate_main_tool_jog_request(params):
    payload = _require_exact_object(
        params,
        _JOG_TOOL_REQUEST_FIELDS,
        "tool-frame jog request parameters",
    )
    axis = payload["axis"]
    if axis not in ("x", "y", "z", "rx", "ry", "rz"):
        raise JsonCommandSchemaError("tool-frame jog axis is invalid")
    if payload["direction"] not in ("negative", "positive"):
        raise JsonCommandSchemaError("tool-frame jog direction is invalid")
    distance = _require_controller_float_tuple(
        (payload["distance"],),
        1,
        "tool-frame jog distance",
    )[0]
    if distance < 0.0:
        raise JsonCommandSchemaError(
            "tool-frame jog distance is out of range"
        )
    if axis in ("rx", "ry", "rz"):
        _require_controller_degree_tuple(
            (distance,),
            1,
            "tool-frame jog angular distance",
        )
    _validate_main_motion_profile(
        payload,
        "tool-frame jog",
        ("percent", "seconds"),
    )
    if payload["wrist_configuration"] not in (
        "automatic",
        "near",
        "far",
    ):
        raise JsonCommandSchemaError(
            "tool-frame jog wrist configuration is invalid"
        )
    _require_boolean_tuple(
        payload["loop_modes"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        "tool-frame jog loop modes",
    )


def _validate_main_live_jog_request(
    params,
    *,
    command_name,
    axes,
):
    payload = _require_exact_object(
        params,
        _LIVE_JOG_REQUEST_FIELDS,
        f"{command_name} request parameters",
    )
    axis = payload["axis"]
    if axes is None:
        if type(axis) is not int or axis < 1 or axis > 9:
            raise JsonCommandSchemaError(
                f"{command_name} axis is invalid"
            )
    elif axis not in axes:
        raise JsonCommandSchemaError(f"{command_name} axis is invalid")
    if payload["direction"] not in ("negative", "positive"):
        raise JsonCommandSchemaError(
            f"{command_name} direction is invalid"
        )
    _validate_main_motion_profile(
        payload,
        command_name,
        ("percent",),
    )
    if payload["wrist_configuration"] not in (
        "automatic",
        "near",
        "far",
    ):
        raise JsonCommandSchemaError(
            f"{command_name} wrist configuration is invalid"
        )
    _require_boolean_tuple(
        payload["loop_modes"],
        JSON_POSITION_ROBOT_JOINT_COUNT,
        f"{command_name} loop modes",
    )
    if type(payload["telemetry_enabled"]) is not bool:
        raise JsonCommandSchemaError(
            f"{command_name} telemetry state must be boolean"
        )
    lease_milliseconds = payload["lease_milliseconds"]
    if (
        type(lease_milliseconds) is not int
        or lease_milliseconds < JSON_LIVE_MOTION_LEASE_MINIMUM_MILLISECONDS
        or lease_milliseconds > JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS
    ):
        raise JsonCommandSchemaError(
            f"{command_name} control lease is invalid"
        )


def validate_main_live_joint_jog_request(params):
    _validate_main_live_jog_request(
        params,
        command_name="live joint jog",
        axes=None,
    )


def validate_main_live_cart_jog_request(params):
    _validate_main_live_jog_request(
        params,
        command_name="live Cartesian jog",
        axes=("x", "y", "z", "rx", "ry", "rz"),
    )


def validate_main_live_tool_jog_request(params):
    _validate_main_live_jog_request(
        params,
        command_name="live tool jog",
        axes=("x", "y", "z", "rx", "ry", "rz"),
    )


def validate_main_stop_request(params):
    payload = _require_exact_object(
        params,
        _STOP_REQUEST_FIELDS,
        "stop request parameters",
    )
    _require_request_identifier(
        payload["motion_id"],
        "stop motion identifier",
    )


def validate_main_renew_live_motion_request(params):
    payload = _require_exact_object(
        params,
        _RENEW_LIVE_MOTION_REQUEST_FIELDS,
        "live-motion renewal request parameters",
    )
    _require_request_identifier(
        payload["motion_id"],
        "live-motion renewal motion identifier",
    )


def _validate_get_home_reference_failure(response):
    error = response.error
    if response.status == "rejected":
        _validate_main_rejection(error, "get-home-reference")
        return
    if response.status == "failed":
        if error.code not in _HOME_REFERENCE_FAILED_ERROR_CODES:
            raise JsonCommandSchemaError(
                "get-home-reference failure code is invalid"
            )
        _require_exact_object(
            error.details,
            frozenset(),
            "get-home-reference failure details",
        )
        return
    raise JsonCommandSchemaError(
        "get-home-reference response status is invalid"
    )


def validate_main_get_home_reference_response(response):
    """Validate one correlated main-controller home-reference response."""
    if type(response) is not Response or response.cmd != "get_home_reference":
        raise JsonCommandSchemaError(
            "get-home-reference response envelope is invalid"
        )
    if response.status == "completed":
        parse_main_home_reference_result(response.result)
        return
    _validate_get_home_reference_failure(response)


def _validate_get_position_disposition_failure(response):
    error = response.error
    if response.status == "rejected":
        _validate_main_rejection(
            error,
            "get-position-disposition",
            _POSITION_DISPOSITION_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "failed":
        if error.code not in _POSITION_DISPOSITION_FAILED_ERROR_CODES:
            raise JsonCommandSchemaError(
                "get-position-disposition failure code is invalid"
            )
        if error.code == "controller_alarm":
            details = _require_exact_object(
                error.details,
                frozenset(("controller_alarm",)),
                "get-position-disposition controller-alarm details",
            )
            alarm = details["controller_alarm"]
            if (
                type(alarm) is not str
                or _POSITION_CONTROLLER_ALARM_PATTERN.fullmatch(alarm)
                is None
            ):
                raise JsonCommandSchemaError(
                    "get-position-disposition controller alarm is invalid"
                )
        else:
            _require_exact_object(
                error.details,
                frozenset(),
                "get-position-disposition failure details",
            )
        return
    raise JsonCommandSchemaError(
        "get-position-disposition response status is invalid"
    )


def validate_main_get_position_disposition_response(response):
    """Validate one correlated position-disposition response."""
    if (
        type(response) is not Response
        or response.cmd != "get_position_disposition"
    ):
        raise JsonCommandSchemaError(
            "get-position-disposition response envelope is invalid"
        )
    if response.status == "completed":
        parse_main_position_disposition_result(response.result)
        return
    _validate_get_position_disposition_failure(response)


def validate_main_correct_position_response(response):
    """Validate one correlated encoder-backed position correction."""
    if type(response) is not Response or response.cmd != "correct_position":
        raise JsonCommandSchemaError(
            "correct-position response envelope is invalid"
        )
    if response.status == "completed":
        parse_main_position_correction_result(response.result)
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            "correct-position",
            _POSITION_CORRECTION_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "failed":
        error = response.error
        if error.code == "position_unavailable":
            _require_exact_object(
                error.details,
                frozenset(),
                "correct-position position-unavailable details",
            )
            return
        if error.code == "encoder_state_unavailable":
            details = _require_exact_object(
                error.details,
                _POSITION_CORRECTION_ENCODER_FAILURE_FIELDS,
                "correct-position encoder failure details",
            )
            axes = _require_boolean_tuple(
                details["axes"],
                JSON_POSITION_ROBOT_JOINT_COUNT,
                "correct-position encoder failure axes",
            )
            if not any(axes):
                raise JsonCommandSchemaError(
                    "correct-position encoder failure has no affected axis"
                )
            parse_main_motion_position_result(details["position"])
            return
        raise JsonCommandSchemaError(
            "correct-position failure code is invalid"
        )
    raise JsonCommandSchemaError(
        "correct-position response status is invalid"
    )


def validate_main_diagnostic_response(response, *, command):
    if (type(response) is not Response or response.cmd != command
            or command not in _DIAGNOSTIC_COMMANDS):
        raise JsonCommandSchemaError(
            "controller diagnostic response envelope is invalid"
        )
    if response.status == "completed":
        if response.cmd == "test_limit_switches":
            parse_main_limit_switches_result(response.result)
        elif response.cmd == "read_encoders":
            parse_main_encoder_counts_result(response.result)
        else:
            _require_exact_object(
                response.result,
                frozenset(),
                "set-encoders result",
            )
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            "controller diagnostic",
            _DIAGNOSTIC_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "failed":
        if response.error.code != "diagnostic_unavailable":
            raise JsonCommandSchemaError(
                "controller diagnostic failure code is invalid"
            )
        _require_exact_object(
            response.error.details,
            frozenset(),
            "controller diagnostic failure details",
        )
        return
    raise JsonCommandSchemaError(
        "controller diagnostic response status is invalid"
    )


def validate_main_set_position_response(response):
    """Validate one correlated controller-position replacement response."""
    if type(response) is not Response or response.cmd != "set_position":
        raise JsonCommandSchemaError(
            "set-position response envelope is invalid"
        )
    if response.status == "completed":
        _require_exact_object(
            response.result,
            frozenset(),
            "set-position result",
        )
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            "set-position",
            _SET_POSITION_REJECTED_ERROR_DETAILS,
        )
        return
    raise JsonCommandSchemaError(
        "set-position response status is invalid"
    )


def _validate_main_configuration_response(response, command, response_name):
    if type(response) is not Response or response.cmd != command:
        raise JsonCommandSchemaError(
            f"{response_name} response envelope is invalid"
        )
    if response.status == "completed":
        _require_exact_object(
            response.result,
            frozenset(),
            f"{response_name} result",
        )
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            response_name,
            _CONFIGURATION_REJECTED_ERROR_DETAILS,
        )
        return
    raise JsonCommandSchemaError(
        f"{response_name} response status is invalid"
    )


def validate_main_update_params_response(response):
    _validate_main_configuration_response(
        response,
        "update_params",
        "update-params",
    )


def validate_main_config_ext_axis_response(response):
    _validate_main_configuration_response(
        response,
        "config_ext_axis",
        "config-ext-axis",
    )


def validate_main_external_axis_zero_response(response, *, command):
    if (
        type(command) is not str
        or command not in _EXTERNAL_AXIS_ZERO_COMMANDS
        or type(response) is not Response
        or response.cmd != command
    ):
        raise JsonCommandSchemaError(
            "external-axis-zero response envelope is invalid"
        )
    if response.status == "completed":
        _parse_main_position_disposition_result(
            response.result,
            "external-axis-zero result",
        )
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            "external-axis-zero",
            _SYNCHRONIZED_SESSION_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "failed":
        if response.error.code != "position_unavailable":
            raise JsonCommandSchemaError(
                "external-axis-zero failure code is invalid"
            )
        _require_exact_object(
            response.error.details,
            frozenset(),
            "external-axis-zero failure details",
        )
        return
    raise JsonCommandSchemaError(
        "external-axis-zero response status is invalid"
    )


def validate_main_controller_wait_response(response):
    if type(response) is not Response or response.cmd != "controller_wait":
        raise JsonCommandSchemaError(
            "controller-wait response envelope is invalid"
        )
    if response.status == "completed":
        _require_exact_object(
            response.result,
            frozenset(),
            "controller-wait result",
        )
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            "controller-wait",
            _SYNCHRONIZED_SESSION_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "cancelled":
        if response.error.code != "emergency_stop":
            raise JsonCommandSchemaError(
                "controller-wait cancellation code is invalid"
            )
        _require_exact_object(
            response.error.details,
            frozenset(),
            "controller-wait cancellation details",
        )
        return
    raise JsonCommandSchemaError(
        "controller-wait response status is invalid"
    )


def _validate_main_modbus_noncompletion(response, response_name):
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            response_name,
            _SYNCHRONIZED_SESSION_REJECTED_ERROR_DETAILS,
        )
        return
    expected_error_code = _MODBUS_TERMINAL_ERROR_CODES.get(response.status)
    if expected_error_code is not None:
        if response.error.code != expected_error_code:
            raise JsonCommandSchemaError(
                f"{response_name} {response.status} code is invalid"
            )
        _require_exact_object(
            response.error.details,
            frozenset(),
            f"{response_name} {response.status} details",
        )
        return
    raise JsonCommandSchemaError(
        f"{response_name} response status is invalid"
    )


def validate_main_modbus_read_response(response, *, command):
    if (
        type(command) is not str
        or command not in _MODBUS_READ_VALUE_MAXIMUMS
        or type(response) is not Response
        or response.cmd != command
    ):
        raise JsonCommandSchemaError(
            "Modbus-read response envelope is invalid"
        )
    if response.status == "completed":
        parse_main_modbus_read_result(response.result, command=command)
        return
    _validate_main_modbus_noncompletion(response, "Modbus-read")


def validate_main_modbus_write_response(response, *, command):
    if (
        type(command) is not str
        or command not in _MODBUS_WRITE_VALUE_MAXIMUMS
        or type(response) is not Response
        or response.cmd != command
    ):
        raise JsonCommandSchemaError(
            "Modbus-write response envelope is invalid"
        )
    if response.status == "completed":
        _require_exact_object(
            response.result,
            frozenset(),
            "Modbus-write result",
        )
        return
    _validate_main_modbus_noncompletion(response, "Modbus-write")


def validate_main_calibrate_response(response):
    if type(response) is not Response or response.cmd != "calibrate":
        raise JsonCommandSchemaError(
            "calibration response envelope is invalid"
        )
    if response.status == "accepted":
        _require_exact_object(
            response.result,
            frozenset(),
            "calibration accepted result",
        )
        return
    if response.status == "completed":
        parse_main_calibration_result(response.result)
        return

    error = response.error
    if response.status == "rejected":
        if error.code == "calibration_not_representable":
            details = _require_exact_object(
                error.details,
                _CALIBRATE_REPRESENTATION_FIELDS,
                "calibration rejection details",
            )
            axes = _require_boolean_tuple(
                details["axes"],
                JSON_POSITION_ROBOT_JOINT_COUNT
                + JSON_POSITION_EXTERNAL_AXIS_COUNT,
                "calibration rejection axes",
            )
            if not any(axes):
                raise JsonCommandSchemaError(
                    "calibration rejection has no affected axis"
                )
            return
        _validate_main_rejection(
            error,
            "calibration",
            _CALIBRATE_EMPTY_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "cancelled":
        if error.code != "emergency_stop":
            raise JsonCommandSchemaError(
                "calibration cancellation code is invalid"
            )
        details = _require_exact_object(
            error.details,
            _CALIBRATE_POSITION_FAILURE_FIELDS,
            "calibration cancellation details",
        )
        parse_main_motion_position_result(details["position"])
        return
    if response.status == "failed":
        if error.code == "position_unavailable":
            _require_exact_object(
                error.details,
                frozenset(),
                "calibration position-unavailable details",
            )
            return
        if error.code == "calibration_failed":
            details = _require_exact_object(
                error.details,
                _CALIBRATE_STAGE_FAILURE_FIELDS,
                "calibration failure details",
            )
            if (
                type(details["stage"]) is not str
                or details["stage"] not in _CALIBRATE_FAILURE_STAGES
            ):
                raise JsonCommandSchemaError(
                    "calibration failure stage is invalid"
                )
            parse_main_motion_position_result(details["position"])
            return
        raise JsonCommandSchemaError(
            "calibration failure code is invalid"
        )
    raise JsonCommandSchemaError(
        "calibration response status is invalid"
    )


def _validate_motion_position_details(
    details,
    expected_fields,
    *,
    response_name,
    axis_count=None,
):
    payload = _require_exact_object(
        details,
        expected_fields,
        f"{response_name} failure details",
    )
    parse_main_motion_position_result(payload["position"])
    if axis_count is not None:
        _require_boolean_tuple(
            payload["axes"],
            axis_count,
            f"{response_name} failure axes",
        )


def validate_main_move_joints_response(response):
    if type(response) is not Response or response.cmd != "move_joints":
        raise JsonCommandSchemaError(
            "move-joints response envelope is invalid"
        )
    if response.status == "accepted":
        _require_exact_object(
            response.result,
            frozenset(),
            "move-joints accepted result",
        )
        return
    if response.status == "completed":
        parse_main_move_joints_result(response.result)
        return
    error = response.error
    if response.status == "rejected":
        if error.code in (
            "joint_limit_violation",
            "position_not_representable",
        ):
            details = _require_exact_object(
                error.details,
                _MOVE_JOINTS_LIMIT_REJECTION_FIELDS,
                "move-joints rejection details",
            )
            _require_boolean_tuple(
                details["axes"],
                JSON_POSITION_ROBOT_JOINT_COUNT
                + JSON_POSITION_EXTERNAL_AXIS_COUNT,
                "move-joints rejection axes",
            )
            return
        _validate_main_rejection(
            error,
            "move-joints",
            _MOVE_JOINTS_EMPTY_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "cancelled":
        if error.code != "emergency_stop":
            raise JsonCommandSchemaError(
                "move-joints cancellation code is invalid"
            )
        _validate_motion_position_details(
            error.details,
            _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
            response_name="move-joints",
        )
        return
    if response.status == "failed":
        if error.code == "position_unavailable":
            _require_exact_object(
                error.details,
                frozenset(),
                "move-joints position-unavailable details",
            )
            return
        if error.code in _MOVE_JOINTS_POSITION_FAILURE_CODES:
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
                response_name="move-joints",
            )
            return
        if error.code in _MOVE_JOINTS_AXIS_FAILURE_CODES:
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_AXIS_FAILURE_FIELDS,
                response_name="move-joints",
                axis_count=JSON_POSITION_ROBOT_JOINT_COUNT,
            )
            return
        raise JsonCommandSchemaError(
            "move-joints failure code is invalid"
        )
    raise JsonCommandSchemaError(
        "move-joints response status is invalid"
    )


def validate_main_motion_trace_response(response):
    if type(response) is not Response or response.cmd != "get_motion_trace":
        raise JsonCommandSchemaError(
            "motion-trace response envelope is invalid"
        )
    if response.status == "completed":
        parse_main_motion_trace_result(response.result)
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            "motion-trace",
            _MOTION_TRACE_REJECTED_ERROR_DETAILS,
        )
        return
    raise JsonCommandSchemaError(
        "motion-trace response status is invalid"
    )


def _validate_main_cartesian_motion_response(
    response,
    *,
    command_name,
    response_name,
    parse_result,
    accepted_response_allowed=True,
):
    if type(response) is not Response or response.cmd != command_name:
        raise JsonCommandSchemaError(
            f"{response_name} response envelope is invalid"
        )
    if response.status == "accepted":
        if not accepted_response_allowed:
            raise JsonCommandSchemaError(
                f"{response_name} response status is invalid"
            )
        _require_exact_object(
            response.result,
            frozenset(),
            f"{response_name} accepted result",
        )
        return
    if response.status == "completed":
        parse_result(response.result)
        return
    error = response.error
    if response.status == "rejected":
        if error.code in (
            "joint_limit_violation",
            "position_not_representable",
        ):
            details = _require_exact_object(
                error.details,
                _MOVE_JOINTS_LIMIT_REJECTION_FIELDS,
                f"{response_name} rejection details",
            )
            axes = _require_boolean_tuple(
                details["axes"],
                JSON_POSITION_ROBOT_JOINT_COUNT
                + JSON_POSITION_EXTERNAL_AXIS_COUNT,
                f"{response_name} rejection axes",
            )
            if not any(axes) and not (
                command_name == "jog_tool"
                and error.code == "position_not_representable"
            ):
                raise JsonCommandSchemaError(
                    f"{response_name} rejection has no affected axis"
                )
            return
        if error.code == "kinematics_unreachable":
            _require_exact_object(
                error.details,
                frozenset(),
                f"{response_name} kinematics rejection details",
            )
            return
        _validate_main_rejection(
            error,
            response_name,
            _MOVE_JOINTS_EMPTY_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "cancelled":
        if error.code != "emergency_stop":
            raise JsonCommandSchemaError(
                f"{response_name} cancellation code is invalid"
            )
        _validate_motion_position_details(
            error.details,
            _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
            response_name=response_name,
        )
        return
    if response.status == "failed":
        if error.code == "position_unavailable":
            _require_exact_object(
                error.details,
                frozenset(),
                f"{response_name} position-unavailable details",
            )
            return
        if error.code in _MOVE_JOINTS_POSITION_FAILURE_CODES:
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
                response_name=response_name,
            )
            return
        if error.code in _MOVE_JOINTS_AXIS_FAILURE_CODES:
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_AXIS_FAILURE_FIELDS,
                response_name=response_name,
                axis_count=JSON_POSITION_ROBOT_JOINT_COUNT,
            )
            return
        raise JsonCommandSchemaError(
            f"{response_name} failure code is invalid"
        )
    raise JsonCommandSchemaError(
        f"{response_name} response status is invalid"
    )


def validate_main_move_cartesian_response(response):
    _validate_main_cartesian_motion_response(
        response,
        command_name="move_cartesian",
        response_name="Cartesian-motion",
        parse_result=parse_main_move_cartesian_result,
    )


def validate_main_move_linear_response(response):
    _validate_main_cartesian_motion_response(
        response,
        command_name="move_linear",
        response_name="linear motion",
        parse_result=parse_main_move_cartesian_result,
    )


def validate_main_move_vision_response(response):
    _validate_main_cartesian_motion_response(
        response,
        command_name="move_vision",
        response_name="vision motion",
        parse_result=parse_main_move_cartesian_result,
    )


def validate_main_move_arc_response(response):
    _validate_main_cartesian_motion_response(
        response,
        command_name="move_arc",
        response_name="arc motion",
        parse_result=parse_main_move_cartesian_result,
        accepted_response_allowed=False,
    )


def validate_main_move_circle_response(response):
    _validate_main_cartesian_motion_response(
        response,
        command_name="move_circle",
        response_name="circle motion",
        parse_result=parse_main_move_cartesian_result,
        accepted_response_allowed=False,
    )


def validate_main_move_spline_response(response):
    _validate_main_cartesian_motion_response(
        response,
        command_name="move_spline",
        response_name="spline motion",
        parse_result=parse_main_move_cartesian_result,
        accepted_response_allowed=False,
    )


def _validate_main_simple_response(
    response,
    *,
    command,
    parse_result,
    failure_codes=(),
    cancellation_code=None,
):
    if type(response) is not Response or response.cmd != command:
        raise JsonCommandSchemaError(
            f"{command} response envelope is invalid"
        )
    if response.status == "completed":
        parse_result(response.result)
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            command,
            _SYNCHRONIZED_SESSION_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "failed" and response.error.code in failure_codes:
        _require_exact_object(
            response.error.details,
            frozenset(),
            f"{command} failure details",
        )
        return
    if (
        response.status == "cancelled"
        and response.error.code == cancellation_code
    ):
        _require_exact_object(
            response.error.details,
            frozenset(),
            f"{command} cancellation details",
        )
        return
    raise JsonCommandSchemaError(f"{command} response status is invalid")


def validate_main_modbus_wait_response(response, *, command):
    _validate_main_simple_response(
        response,
        command=command,
        parse_result=partial(
            parse_main_modbus_wait_result,
            command=command,
        ),
        failure_codes=("modbus_error", "timeout"),
        cancellation_code="emergency_stop",
    )


def validate_main_delete_sd_program_response(response):
    _validate_main_simple_response(
        response,
        command="delete_sd_program",
        parse_result=parse_main_delete_sd_program_result,
        failure_codes=("media_changed", "storage_error"),
        cancellation_code="emergency_stop",
    )


def validate_main_list_sd_programs_response(response):
    _validate_main_simple_response(
        response,
        command="list_sd_programs",
        parse_result=parse_main_list_sd_programs_result,
        failure_codes=("storage_error",),
        cancellation_code="emergency_stop",
    )


def validate_main_write_gcode_move_response(response):
    if (
        type(response) is Response
        and response.cmd == "write_gcode_move"
        and response.status == "rejected"
    ):
        error = response.error
        if error.code in (
            "joint_limit_violation",
            "position_not_representable",
        ):
            details = _require_exact_object(
                error.details,
                _MOVE_JOINTS_LIMIT_REJECTION_FIELDS,
                "G-code write rejection details",
            )
            _require_boolean_tuple(
                details["axes"],
                JSON_POSITION_ROBOT_JOINT_COUNT
                + JSON_POSITION_EXTERNAL_AXIS_COUNT,
                "G-code write rejection axes",
            )
            return
        if error.code == "kinematics_unreachable":
            _require_exact_object(
                error.details,
                frozenset(),
                "G-code write kinematics rejection details",
            )
            return
    _validate_main_simple_response(
        response,
        command="write_gcode_move",
        parse_result=parse_main_position_result,
        failure_codes=(
            "media_changed",
            "position_unavailable",
            "storage_error",
        ),
        cancellation_code="emergency_stop",
    )


def _validate_main_gcode_playback_response(response, command):
    if (
        type(response) is Response
        and response.cmd == command
        and response.status == "failed"
        and response.error.code in ("media_changed", "storage_error")
    ):
        _require_exact_object(
            response.error.details,
            frozenset(),
            "G-code playback storage failure details",
        )
        return
    _validate_main_cartesian_motion_response(
        response,
        command_name=command,
        response_name="G-code playback",
        parse_result=parse_main_move_cartesian_result,
    )


def validate_main_play_gcode_file_response(response):
    _validate_main_gcode_playback_response(response, "play_gcode_file")


def validate_main_tool_jog_response(response):
    _validate_main_cartesian_motion_response(
        response,
        command_name="jog_tool",
        response_name="tool-frame jog",
        parse_result=parse_main_tool_jog_result,
    )


def _validate_main_live_jog_response(
    response,
    *,
    command_name,
    response_name,
    allow_kinematics_failure,
    allow_positionless_representation,
):
    if type(response) is not Response or response.cmd != command_name:
        raise JsonCommandSchemaError(
            f"{response_name} response envelope is invalid"
        )
    if response.status == "accepted":
        _require_exact_object(
            response.result,
            frozenset(),
            f"{response_name} accepted result",
        )
        return
    if response.status == "completed":
        parse_main_live_jog_result(response.result)
        return
    error = response.error
    if response.status == "rejected":
        _validate_main_rejection(
            error,
            response_name,
            _MOVE_JOINTS_EMPTY_REJECTED_ERROR_DETAILS,
        )
        return
    if response.status == "cancelled":
        if error.code != "emergency_stop":
            raise JsonCommandSchemaError(
                f"{response_name} cancellation code is invalid"
            )
        _validate_motion_position_details(
            error.details,
            _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
            response_name=response_name,
        )
        return
    if response.status == "failed":
        if error.code == "position_unavailable":
            _require_exact_object(
                error.details,
                frozenset(),
                f"{response_name} position-unavailable details",
            )
            return
        if error.code in (
            "joint_limit_reached",
            "position_not_representable",
        ):
            details = _require_exact_object(
                error.details,
                _MOVE_JOINTS_AXIS_FAILURE_FIELDS,
                f"{response_name} failure details",
            )
            parse_main_motion_position_result(details["position"])
            axes = _require_boolean_tuple(
                details["axes"],
                JSON_POSITION_ROBOT_JOINT_COUNT
                + JSON_POSITION_EXTERNAL_AXIS_COUNT,
                f"{response_name} failure axes",
            )
            if not any(axes) and not (
                allow_positionless_representation
                and error.code == "position_not_representable"
            ):
                raise JsonCommandSchemaError(
                    f"{response_name} failure has no affected axis"
                )
            return
        if error.code == "kinematics_unreachable":
            if not allow_kinematics_failure:
                raise JsonCommandSchemaError(
                    f"{response_name} failure code is invalid"
                )
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
                response_name=response_name,
            )
            return
        if error.code == "motion_execution_failed":
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
                response_name=response_name,
            )
            return
        if error.code == "control_lease_expired":
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_POSITION_FAILURE_FIELDS,
                response_name=response_name,
            )
            return
        if error.code in _MOVE_JOINTS_AXIS_FAILURE_CODES:
            _validate_motion_position_details(
                error.details,
                _MOVE_JOINTS_AXIS_FAILURE_FIELDS,
                response_name=response_name,
                axis_count=JSON_POSITION_ROBOT_JOINT_COUNT,
            )
            return
        raise JsonCommandSchemaError(
            f"{response_name} failure code is invalid"
        )
    raise JsonCommandSchemaError(
        f"{response_name} response status is invalid"
    )


def validate_main_live_joint_jog_response(response):
    _validate_main_live_jog_response(
        response,
        command_name="live_joint_jog",
        response_name="live joint jog",
        allow_kinematics_failure=False,
        allow_positionless_representation=False,
    )


def validate_main_live_cart_jog_response(response):
    _validate_main_live_jog_response(
        response,
        command_name="live_cart_jog",
        response_name="live Cartesian jog",
        allow_kinematics_failure=True,
        allow_positionless_representation=False,
    )


def validate_main_live_tool_jog_response(response):
    _validate_main_live_jog_response(
        response,
        command_name="live_tool_jog",
        response_name="live tool jog",
        allow_kinematics_failure=True,
        allow_positionless_representation=True,
    )


def validate_main_stop_response(response):
    if type(response) is not Response or response.cmd != "stop":
        raise JsonCommandSchemaError("stop response envelope is invalid")
    if response.status == "completed":
        parse_main_stop_result(response.result)
        return
    if response.status != "rejected":
        raise JsonCommandSchemaError("stop response status is invalid")
    error = response.error
    if error.code == "live_motion_mismatch":
        details = _require_exact_object(
            error.details,
            frozenset(("active_motion_id",)),
            "stop rejection details",
        )
        _require_request_identifier(
            details["active_motion_id"],
            "stop active-motion identifier",
        )
        return
    _validate_exact_main_rejection(
        error,
        "stop",
        _STOP_EXACT_REJECTED_ERROR_DETAILS,
    )


def validate_main_renew_live_motion_response(response):
    if (
        type(response) is not Response
        or response.cmd != "renew_live_motion"
    ):
        raise JsonCommandSchemaError(
            "live-motion renewal response envelope is invalid"
        )
    if response.status == "completed":
        parse_main_renew_live_motion_result(response.result)
        return
    if response.status != "rejected":
        raise JsonCommandSchemaError(
            "live-motion renewal response status is invalid"
        )
    error = response.error
    if error.code == "live_motion_mismatch":
        details = _require_exact_object(
            error.details,
            frozenset(("active_motion_id",)),
            "live-motion renewal rejection details",
        )
        _require_request_identifier(
            details["active_motion_id"],
            "live-motion renewal active-motion identifier",
        )
        return
    _validate_exact_main_rejection(
        error,
        "live-motion renewal",
        _RENEW_LIVE_MOTION_EXACT_REJECTED_ERROR_DETAILS,
    )


def validate_auxiliary_hello_request(params):
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError(
            "auxiliary hello request parameters must be empty"
        )


def validate_auxiliary_hello_response(response):
    if type(response) is not Response or response.cmd != "hello":
        raise JsonCommandSchemaError(
            "auxiliary hello response envelope is invalid"
        )
    if response.status == "completed":
        result = parse_auxiliary_hello_result(response.result)
        payload_length = len(encode_message(response)) - 1
        if payload_length > result.protocol.maximum_payload_bytes:
            raise JsonCommandSchemaError(
                "auxiliary hello exceeds the advertised payload limit"
            )
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            "auxiliary hello",
            {"busy": MappingProxyType({})},
        )
        return
    raise JsonCommandSchemaError(
        "auxiliary hello response status is invalid"
    )


def validate_auxiliary_servo_request(params):
    payload = _require_exact_object(
        params,
        _AUXILIARY_SERVO_REQUEST_FIELDS,
        "auxiliary servo request parameters",
    )
    if type(payload["channel"]) is not int or not 0 <= payload["channel"] <= 6:
        raise JsonCommandSchemaError("auxiliary servo channel is invalid")
    if type(payload["position"]) is not int or not 0 <= payload["position"] <= 180:
        raise JsonCommandSchemaError("auxiliary servo position is invalid")


def validate_auxiliary_input_read_request(params):
    payload = _require_exact_object(
        params,
        _AUXILIARY_INPUT_REQUEST_FIELDS,
        "auxiliary input-read request parameters",
    )
    if type(payload["pin"]) is not int or not 2 <= payload["pin"] <= 27:
        raise JsonCommandSchemaError("auxiliary input pin is invalid")


def validate_auxiliary_set_output_request(params):
    payload = _require_exact_object(
        params,
        _AUXILIARY_OUTPUT_REQUEST_FIELDS,
        "auxiliary set-output request parameters",
    )
    if type(payload["pin"]) is not int or not 8 <= payload["pin"] <= 53:
        raise JsonCommandSchemaError("auxiliary output pin is invalid")
    if type(payload["state"]) is not bool:
        raise JsonCommandSchemaError("auxiliary output state must be boolean")


def validate_auxiliary_wait_input_request(params):
    payload = _require_exact_object(
        params,
        _AUXILIARY_WAIT_REQUEST_FIELDS,
        "auxiliary wait-input request parameters",
    )
    validate_auxiliary_input_read_request({"pin": payload["pin"]})
    if type(payload["state"]) is not bool:
        raise JsonCommandSchemaError("auxiliary wait state must be boolean")
    timeout = payload["timeout_seconds"]
    if (
        type(timeout) is not int
        or timeout < 1
        or timeout > JSON_AUXILIARY_WAIT_MAXIMUM_SECONDS
    ):
        raise JsonCommandSchemaError("auxiliary wait timeout is invalid")


def validate_auxiliary_empty_request(params):
    if type(params) not in (dict, MappingProxyType) or params:
        raise JsonCommandSchemaError(
            "auxiliary request parameters must be empty"
        )


def parse_auxiliary_input_read_result(result):
    payload = _require_exact_object(
        result,
        _AUXILIARY_INPUT_RESULT_FIELDS,
        "auxiliary input-read result",
    )
    if type(payload["state"]) is not bool:
        raise JsonCommandSchemaError(
            "auxiliary input-read state must be boolean"
        )
    return JsonAuxiliaryInputResult(payload["state"])


def parse_auxiliary_gripper_amps_result(result):
    payload = _require_exact_object(
        result,
        _AUXILIARY_CURRENT_RESULT_FIELDS,
        "auxiliary gripper-current result",
    )
    amps = payload["amps"]
    if (
        type(amps) not in (int, float)
        or not math.isfinite(amps)
        or amps < 0.0
        or amps > 28.0
    ):
        raise JsonCommandSchemaError(
            "auxiliary gripper current is out of range"
        )
    return JsonAuxiliaryCurrentResult(float(amps))


def _validate_auxiliary_response(
    response,
    *,
    command,
    parse_result,
    failure_codes=(),
    cancellation_code=None,
):
    if type(response) is not Response or response.cmd != command:
        raise JsonCommandSchemaError(
            f"auxiliary {command} response envelope is invalid"
        )
    if response.status == "completed":
        parse_result(response.result)
        return
    if response.status == "rejected":
        _validate_main_rejection(
            response.error,
            f"auxiliary {command}",
            {"busy": MappingProxyType({})},
        )
        return
    if response.status == "failed" and response.error.code in failure_codes:
        _require_exact_object(
            response.error.details,
            frozenset(),
            f"auxiliary {command} failure details",
        )
        return
    if (
        response.status == "cancelled"
        and response.error.code == cancellation_code
    ):
        _require_exact_object(
            response.error.details,
            frozenset(),
            f"auxiliary {command} cancellation details",
        )
        return
    raise JsonCommandSchemaError(
        f"auxiliary {command} response status is invalid"
    )


def _empty_result_parser(result):
    return _require_exact_object(result, frozenset(), "empty command result")


def validate_auxiliary_command_response(response, *, command, parse_result):
    if command == "wait_input":
        failure_codes = ("timeout",)
    elif command == "servo":
        failure_codes = ("servo_unavailable",)
    else:
        failure_codes = ()
    cancellation_code = "stop_requested" if command == "wait_input" else None
    _validate_auxiliary_response(
        response,
        command=command,
        parse_result=parse_result,
        failure_codes=failure_codes,
        cancellation_code=cancellation_code,
    )


MAIN_HELLO_COMMAND_CONTRACT = JsonCommandContract(
    "hello",
    validate_main_hello_request,
    validate_main_hello_response,
)

MAIN_GET_POSITION_DISPOSITION_COMMAND_CONTRACT = JsonCommandContract(
    "get_position_disposition",
    validate_main_get_position_disposition_request,
    validate_main_get_position_disposition_response,
)

MAIN_CORRECT_POSITION_COMMAND_CONTRACT = JsonCommandContract(
    "correct_position",
    validate_main_correct_position_request,
    validate_main_correct_position_response,
)

MAIN_GET_HOME_REFERENCE_COMMAND_CONTRACT = JsonCommandContract(
    "get_home_reference",
    validate_main_get_home_reference_request,
    validate_main_get_home_reference_response,
)

MAIN_TEST_LIMIT_SWITCHES_COMMAND_CONTRACT = JsonCommandContract(
    "test_limit_switches",
    validate_main_diagnostic_request,
    partial(validate_main_diagnostic_response, command="test_limit_switches"),
)

MAIN_SET_ENCODERS_COMMAND_CONTRACT = JsonCommandContract(
    "set_encoders",
    validate_main_diagnostic_request,
    partial(validate_main_diagnostic_response, command="set_encoders"),
)

MAIN_READ_ENCODERS_COMMAND_CONTRACT = JsonCommandContract(
    "read_encoders",
    validate_main_diagnostic_request,
    partial(validate_main_diagnostic_response, command="read_encoders"),
)

MAIN_GET_MOTION_TRACE_COMMAND_CONTRACT = JsonCommandContract(
    "get_motion_trace",
    validate_main_motion_trace_request,
    validate_main_motion_trace_response,
)

MAIN_SET_POSITION_COMMAND_CONTRACT = JsonCommandContract(
    "set_position",
    validate_main_set_position_request,
    validate_main_set_position_response,
)

MAIN_UPDATE_PARAMS_COMMAND_CONTRACT = JsonCommandContract(
    "update_params",
    validate_main_update_params_request,
    validate_main_update_params_response,
)

MAIN_CONFIG_EXT_AXIS_COMMAND_CONTRACT = JsonCommandContract(
    "config_ext_axis",
    validate_main_config_ext_axis_request,
    validate_main_config_ext_axis_response,
)

MAIN_ZERO_J7_COMMAND_CONTRACT = JsonCommandContract(
    "zero_j7",
    validate_main_external_axis_zero_request,
    partial(validate_main_external_axis_zero_response, command="zero_j7"),
)

MAIN_ZERO_J8_COMMAND_CONTRACT = JsonCommandContract(
    "zero_j8",
    validate_main_external_axis_zero_request,
    partial(validate_main_external_axis_zero_response, command="zero_j8"),
)

MAIN_ZERO_J9_COMMAND_CONTRACT = JsonCommandContract(
    "zero_j9",
    validate_main_external_axis_zero_request,
    partial(validate_main_external_axis_zero_response, command="zero_j9"),
)

MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT = JsonCommandContract(
    "controller_wait",
    validate_main_controller_wait_request,
    validate_main_controller_wait_response,
)


def _main_modbus_read_contract(command):
    return JsonCommandContract(
        command,
        validate_main_modbus_read_request,
        partial(validate_main_modbus_read_response, command=command),
    )


MAIN_MODBUS_READ_HOLDING_REGISTER_COMMAND_CONTRACT = (
    _main_modbus_read_contract("modbus_read_holding_register")
)
MAIN_MODBUS_READ_COIL_COMMAND_CONTRACT = _main_modbus_read_contract(
    "modbus_read_coil"
)
MAIN_MODBUS_READ_DISCRETE_INPUT_COMMAND_CONTRACT = (
    _main_modbus_read_contract("modbus_read_discrete_input")
)
MAIN_MODBUS_READ_INPUT_REGISTER_COMMAND_CONTRACT = (
    _main_modbus_read_contract("modbus_read_input_register")
)


def _main_modbus_write_contract(command):
    return JsonCommandContract(
        command,
        partial(validate_main_modbus_write_request, command=command),
        partial(validate_main_modbus_write_response, command=command),
    )


MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT = _main_modbus_write_contract(
    "modbus_write_coil"
)
MAIN_MODBUS_WRITE_REGISTER_COMMAND_CONTRACT = _main_modbus_write_contract(
    "modbus_write_register"
)

MAIN_CALIBRATE_COMMAND_CONTRACT = JsonCommandContract(
    "calibrate",
    validate_main_calibrate_request,
    validate_main_calibrate_response,
)

MAIN_MOVE_JOINTS_COMMAND_CONTRACT = JsonCommandContract(
    "move_joints",
    validate_main_move_joints_request,
    validate_main_move_joints_response,
)

MAIN_MOVE_CARTESIAN_COMMAND_CONTRACT = JsonCommandContract(
    "move_cartesian",
    validate_main_move_cartesian_request,
    validate_main_move_cartesian_response,
)

MAIN_MOVE_LINEAR_COMMAND_CONTRACT = JsonCommandContract(
    "move_linear",
    validate_main_move_linear_request,
    validate_main_move_linear_response,
)

MAIN_MOVE_VISION_COMMAND_CONTRACT = JsonCommandContract(
    "move_vision",
    validate_main_move_vision_request,
    validate_main_move_vision_response,
)

MAIN_JOG_TOOL_COMMAND_CONTRACT = JsonCommandContract(
    "jog_tool",
    validate_main_tool_jog_request,
    validate_main_tool_jog_response,
)

MAIN_LIVE_JOINT_JOG_COMMAND_CONTRACT = JsonCommandContract(
    "live_joint_jog",
    validate_main_live_joint_jog_request,
    validate_main_live_joint_jog_response,
    acceptance_required_for_terminal=True,
    deadline_suspended_after_acceptance=True,
)

MAIN_LIVE_CART_JOG_COMMAND_CONTRACT = JsonCommandContract(
    "live_cart_jog",
    validate_main_live_cart_jog_request,
    validate_main_live_cart_jog_response,
    acceptance_required_for_terminal=True,
    deadline_suspended_after_acceptance=True,
)

MAIN_LIVE_TOOL_JOG_COMMAND_CONTRACT = JsonCommandContract(
    "live_tool_jog",
    validate_main_live_tool_jog_request,
    validate_main_live_tool_jog_response,
    acceptance_required_for_terminal=True,
    deadline_suspended_after_acceptance=True,
)

MAIN_STOP_COMMAND_CONTRACT = JsonCommandContract(
    "stop",
    validate_main_stop_request,
    validate_main_stop_response,
)

MAIN_RENEW_LIVE_MOTION_COMMAND_CONTRACT = JsonCommandContract(
    "renew_live_motion",
    validate_main_renew_live_motion_request,
    validate_main_renew_live_motion_response,
)

MAIN_WAIT_MODBUS_COIL_COMMAND_CONTRACT = JsonCommandContract(
    "wait_modbus_coil",
    partial(
        validate_main_modbus_wait_request,
        command="wait_modbus_coil",
    ),
    partial(
        validate_main_modbus_wait_response,
        command="wait_modbus_coil",
    ),
)

MAIN_WAIT_MODBUS_DISCRETE_INPUT_COMMAND_CONTRACT = JsonCommandContract(
    "wait_modbus_discrete_input",
    partial(
        validate_main_modbus_wait_request,
        command="wait_modbus_discrete_input",
    ),
    partial(
        validate_main_modbus_wait_response,
        command="wait_modbus_discrete_input",
    ),
)

MAIN_DELETE_SD_PROGRAM_COMMAND_CONTRACT = JsonCommandContract(
    "delete_sd_program",
    validate_main_storage_target_request,
    validate_main_delete_sd_program_response,
)

MAIN_LIST_SD_PROGRAMS_COMMAND_CONTRACT = JsonCommandContract(
    "list_sd_programs",
    validate_main_empty_request,
    validate_main_list_sd_programs_response,
)

MAIN_WRITE_GCODE_MOVE_COMMAND_CONTRACT = JsonCommandContract(
    "write_gcode_move",
    validate_main_write_gcode_move_request,
    validate_main_write_gcode_move_response,
)

MAIN_PLAY_GCODE_FILE_COMMAND_CONTRACT = JsonCommandContract(
    "play_gcode_file",
    validate_main_storage_target_request,
    validate_main_play_gcode_file_response,
    acceptance_required_for_terminal=True,
    deadline_suspended_after_acceptance=True,
)

MAIN_WAIT_MODBUS_HOLDING_REGISTER_COMMAND_CONTRACT = JsonCommandContract(
    "wait_modbus_holding_register",
    partial(
        validate_main_modbus_wait_request,
        command="wait_modbus_holding_register",
    ),
    partial(
        validate_main_modbus_wait_response,
        command="wait_modbus_holding_register",
    ),
)

MAIN_MOVE_ARC_COMMAND_CONTRACT = JsonCommandContract(
    "move_arc",
    validate_main_move_arc_request,
    validate_main_move_arc_response,
)

MAIN_MOVE_CIRCLE_COMMAND_CONTRACT = JsonCommandContract(
    "move_circle",
    validate_main_move_circle_request,
    validate_main_move_circle_response,
)

MAIN_MOVE_SPLINE_COMMAND_CONTRACT = JsonCommandContract(
    "move_spline",
    validate_main_move_spline_request,
    validate_main_move_spline_response,
)

AUXILIARY_HELLO_COMMAND_CONTRACT = JsonCommandContract(
    "hello",
    validate_auxiliary_hello_request,
    validate_auxiliary_hello_response,
)


def _auxiliary_command_contract(command, request_validator, result_parser):
    return JsonCommandContract(
        command,
        request_validator,
        partial(
            validate_auxiliary_command_response,
            command=command,
            parse_result=result_parser,
        ),
    )


AUXILIARY_SERVO_COMMAND_CONTRACT = _auxiliary_command_contract(
    "servo",
    validate_auxiliary_servo_request,
    _empty_result_parser,
)
AUXILIARY_INPUT_READ_COMMAND_CONTRACT = _auxiliary_command_contract(
    "input_read",
    validate_auxiliary_input_read_request,
    parse_auxiliary_input_read_result,
)
AUXILIARY_SET_OUTPUT_COMMAND_CONTRACT = _auxiliary_command_contract(
    "set_output",
    validate_auxiliary_set_output_request,
    _empty_result_parser,
)
AUXILIARY_WAIT_INPUT_COMMAND_CONTRACT = _auxiliary_command_contract(
    "wait_input",
    validate_auxiliary_wait_input_request,
    _empty_result_parser,
)
AUXILIARY_TEST_GRIPPER_AMPS_COMMAND_CONTRACT = _auxiliary_command_contract(
    "test_gripper_amps",
    validate_auxiliary_empty_request,
    parse_auxiliary_gripper_amps_result,
)
AUXILIARY_STOP_COMMAND_CONTRACT = _auxiliary_command_contract(
    "stop",
    validate_auxiliary_empty_request,
    _empty_result_parser,
)
AUXILIARY_GRIPPER_DETACH_COMMAND_CONTRACT = _auxiliary_command_contract(
    "gripper_detach",
    validate_auxiliary_empty_request,
    _empty_result_parser,
)


__all__ = (
    "AUXILIARY_GRIPPER_DETACH_COMMAND_CONTRACT",
    "AUXILIARY_HELLO_COMMAND_CONTRACT",
    "AUXILIARY_INPUT_READ_COMMAND_CONTRACT",
    "AUXILIARY_SERVO_COMMAND_CONTRACT",
    "AUXILIARY_SET_OUTPUT_COMMAND_CONTRACT",
    "AUXILIARY_STOP_COMMAND_CONTRACT",
    "AUXILIARY_TEST_GRIPPER_AMPS_COMMAND_CONTRACT",
    "AUXILIARY_WAIT_INPUT_COMMAND_CONTRACT",
    "JSON_AUXILIARY_COMMAND_MANIFEST",
    "JSON_AUXILIARY_WAIT_MAXIMUM_SECONDS",
    "JSON_CAPABILITY_EVENT_STREAM_V1",
    "JSON_CAPABILITY_PROTOCOL_V1",
    "JSON_CAPABILITY_REQUEST_CORRELATION_V1",
    "JSON_CONTROLLER_WAIT_MAXIMUM_SECONDS",
    "JSON_HELLO_MAXIMUM_CAPABILITIES",
    "JSON_HELLO_MAXIMUM_TEXT_LENGTH",
    "JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS",
    "JSON_LIVE_MOTION_LEASE_MINIMUM_MILLISECONDS",
    "JSON_MAIN_FIRMWARE_FRAME_RECEIVE_TIMEOUT_SECONDS",
    "JSON_MAIN_COMMAND_MANIFEST",
    "JSON_MOTION_TRACE_PAGE_RECORDS",
    "JSON_MOTION_TRACE_RECORD_CAPACITY",
    "JSON_HOME_REFERENCE_AXIS_COUNT",
    "JSON_POSITION_CARTESIAN_ORIENTATION_COUNT",
    "JSON_POSITION_CARTESIAN_TRANSLATION_COUNT",
    "JSON_POSITION_EXTERNAL_AXIS_COUNT",
    "JSON_POSITION_ROBOT_JOINT_COUNT",
    "JSON_POSITION_SOURCE_CONTROLLER_STEP_STATE",
    "JSON_PROTOCOL_NAME",
    "JSON_REQUIRED_SESSION_CAPABILITIES",
    "JSON_SESSION_IDENTIFIER_LENGTH",
    "MAIN_CALIBRATE_COMMAND_CONTRACT",
    "MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT",
    "MAIN_HELLO_COMMAND_CONTRACT",
    "MAIN_CONFIG_EXT_AXIS_COMMAND_CONTRACT",
    "MAIN_GET_HOME_REFERENCE_COMMAND_CONTRACT",
    "MAIN_GET_MOTION_TRACE_COMMAND_CONTRACT",
    "MAIN_GET_POSITION_DISPOSITION_COMMAND_CONTRACT",
    "MAIN_CORRECT_POSITION_COMMAND_CONTRACT",
    "MAIN_DELETE_SD_PROGRAM_COMMAND_CONTRACT",
    "MAIN_READ_ENCODERS_COMMAND_CONTRACT",
    "MAIN_SET_ENCODERS_COMMAND_CONTRACT",
    "MAIN_TEST_LIMIT_SWITCHES_COMMAND_CONTRACT",
    "MAIN_JOG_TOOL_COMMAND_CONTRACT",
    "MAIN_LIVE_CART_JOG_COMMAND_CONTRACT",
    "MAIN_LIVE_JOINT_JOG_COMMAND_CONTRACT",
    "MAIN_LIVE_TOOL_JOG_COMMAND_CONTRACT",
    "MAIN_MOVE_JOINTS_COMMAND_CONTRACT",
    "MAIN_MOVE_CARTESIAN_COMMAND_CONTRACT",
    "MAIN_MOVE_ARC_COMMAND_CONTRACT",
    "MAIN_MOVE_CIRCLE_COMMAND_CONTRACT",
    "MAIN_MOVE_LINEAR_COMMAND_CONTRACT",
    "MAIN_MOVE_SPLINE_COMMAND_CONTRACT",
    "MAIN_MOVE_VISION_COMMAND_CONTRACT",
    "MAIN_LIST_SD_PROGRAMS_COMMAND_CONTRACT",
    "MAIN_MODBUS_READ_COIL_COMMAND_CONTRACT",
    "MAIN_MODBUS_READ_DISCRETE_INPUT_COMMAND_CONTRACT",
    "MAIN_MODBUS_READ_HOLDING_REGISTER_COMMAND_CONTRACT",
    "MAIN_MODBUS_READ_INPUT_REGISTER_COMMAND_CONTRACT",
    "MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT",
    "MAIN_MODBUS_WRITE_REGISTER_COMMAND_CONTRACT",
    "MAIN_RENEW_LIVE_MOTION_COMMAND_CONTRACT",
    "MAIN_PLAY_GCODE_FILE_COMMAND_CONTRACT",
    "MAIN_SET_POSITION_COMMAND_CONTRACT",
    "MAIN_STOP_COMMAND_CONTRACT",
    "MAIN_UPDATE_PARAMS_COMMAND_CONTRACT",
    "MAIN_WAIT_MODBUS_COIL_COMMAND_CONTRACT",
    "MAIN_WAIT_MODBUS_DISCRETE_INPUT_COMMAND_CONTRACT",
    "MAIN_WAIT_MODBUS_HOLDING_REGISTER_COMMAND_CONTRACT",
    "MAIN_WRITE_GCODE_MOVE_COMMAND_CONTRACT",
    "MAIN_ZERO_J7_COMMAND_CONTRACT",
    "MAIN_ZERO_J8_COMMAND_CONTRACT",
    "MAIN_ZERO_J9_COMMAND_CONTRACT",
    "JsonCommandSchemaError",
    "JsonAuxiliaryHelloResult",
    "JsonAuxiliaryCurrentResult",
    "JsonAuxiliaryInputResult",
    "JsonHelloFirmware",
    "JsonHelloProtocol",
    "JsonMainControllerIdentity",
    "JsonMainDeleteSdProgramResult",
    "JsonMainSdProgramListResult",
    "JsonMainCalibrationResult",
    "JsonMainHelloResult",
    "JsonMainHomeReferenceResult",
    "JsonMainCartesianMotionResult",
    "JsonMainJointMotionResult",
    "JsonMainMotionTraceDisposition",
    "JsonMainMotionTraceNoCaptureResult",
    "JsonMainMotionTracePageResult",
    "JsonMainMotionTraceRecord",
    "JsonMainJointTelemetrySample",
    "JsonMainLiveJogResult",
    "JsonMainRenewLiveMotionResult",
    "JsonMainPositionResult",
    "JsonMainToolJogResult",
    "JsonMainStopResult",
    "JsonScalarResult",
    "parse_main_hello_result",
    "parse_auxiliary_gripper_amps_result",
    "parse_auxiliary_hello_result",
    "parse_auxiliary_input_read_result",
    "parse_main_calibration_result",
    "parse_main_encoder_counts_result",
    "parse_main_home_reference_result",
    "parse_main_limit_switches_result",
    "parse_main_move_cartesian_result",
    "parse_main_joint_position_telemetry",
    "parse_main_live_jog_result",
    "parse_main_renew_live_motion_result",
    "parse_main_move_joints_result",
    "parse_main_motion_trace_result",
    "parse_main_modbus_read_result",
    "parse_main_modbus_wait_result",
    "parse_main_delete_sd_program_result",
    "parse_main_list_sd_programs_result",
    "parse_main_motion_position_result",
    "parse_main_position_result",
    "parse_main_position_disposition_result",
    "parse_main_position_correction_result",
    "parse_main_tool_jog_result",
    "parse_main_stop_result",
    "validate_main_configuration_fingerprint",
    "validate_main_get_position_disposition_request",
    "validate_main_get_position_disposition_response",
    "validate_main_correct_position_request",
    "validate_main_correct_position_response",
    "validate_main_get_home_reference_request",
    "validate_main_get_home_reference_response",
    "validate_main_diagnostic_request",
    "validate_main_diagnostic_response",
    "validate_main_external_axis_zero_request",
    "validate_main_external_axis_zero_response",
    "validate_main_controller_wait_request",
    "validate_main_controller_wait_response",
    "validate_main_hello_request",
    "validate_main_hello_response",
    "validate_main_calibrate_request",
    "validate_main_calibrate_response",
    "validate_main_move_arc_request",
    "validate_main_move_arc_response",
    "validate_main_move_cartesian_request",
    "validate_main_move_cartesian_response",
    "validate_main_move_circle_request",
    "validate_main_move_circle_response",
    "validate_main_move_linear_request",
    "validate_main_move_linear_response",
    "validate_main_move_spline_request",
    "validate_main_move_spline_response",
    "validate_main_move_vision_request",
    "validate_main_move_vision_response",
    "validate_main_move_joints_request",
    "validate_main_move_joints_response",
    "validate_main_motion_trace_request",
    "validate_main_motion_trace_response",
    "validate_main_live_cart_jog_request",
    "validate_main_live_cart_jog_response",
    "validate_main_live_joint_jog_request",
    "validate_main_live_joint_jog_response",
    "validate_main_live_tool_jog_request",
    "validate_main_live_tool_jog_response",
    "validate_main_modbus_read_request",
    "validate_main_modbus_read_response",
    "validate_main_modbus_write_request",
    "validate_main_modbus_write_response",
    "validate_main_modbus_wait_request",
    "validate_main_modbus_wait_response",
    "validate_main_renew_live_motion_request",
    "validate_main_renew_live_motion_response",
    "validate_main_tool_jog_request",
    "validate_main_tool_jog_response",
    "validate_main_stop_request",
    "validate_main_stop_response",
    "validate_main_config_ext_axis_request",
    "validate_main_config_ext_axis_response",
    "validate_main_storage_target_request",
    "validate_main_write_gcode_move_request",
    "validate_main_set_position_request",
    "validate_main_set_position_response",
    "validate_main_update_params_request",
    "validate_main_update_params_response",
)
