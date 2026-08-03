"""Strict controller-motion trace records and hardware-free profile analysis."""

from dataclasses import dataclass
import json
import math
from numbers import Integral, Real
import re
import statistics
import struct
from typing import Optional, Tuple


CONTROLLER_TRACE_SCHEMA = "ar4.controller-trace.v2"
CONTROLLER_TRACE_TIMEBASE = "host-monotonic-offset-seconds"
CONTROLLER_TRACE_TIME_ORIGIN = "immediately-before-rj-write"
CONTROLLER_TRACE_POSITION_UNIT = "degree"
CONTROLLER_TRACE_SOURCE = "JOINT_TELEMETRY_V1"
CONTROLLER_TRACE_AXIS_COUNT = 6
CONTROLLER_TRACE_EXPECTED_SAMPLE_PERIOD_SECONDS = 0.1
CONTROLLER_TRACE_MAXIMUM_BYTES = 16 * 1024 * 1024
CONTROLLER_TRACE_MAXIMUM_SAMPLES = 100_000
CONTROLLER_TRACE_MAXIMUM_LINE_BYTES = 4096
CONTROLLER_TRACE_MAXIMUM_DETAIL_LENGTH = 512
CONTROLLER_TRACE_MINIMUM_ENCODER_MILLIDEGREES = -(2 ** 31)
CONTROLLER_TRACE_MAXIMUM_ENCODER_MILLIDEGREES = 2 ** 31 - 1


class ControllerTraceError(ValueError):
    """Base error for invalid controller traces."""


class ControllerTraceValidationError(ControllerTraceError):
    """A trace value or sequence violates the controller-trace contract."""


class ControllerTraceFormatError(ControllerTraceError):
    """A serialized controller trace is malformed or unsupported."""


class ControllerTraceAnalysisError(ControllerTraceError):
    """Trace metrics cannot be represented safely."""


def _finite_number(value, field_name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ControllerTraceValidationError(
            f"{field_name} must be a finite number"
        )
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ControllerTraceValidationError(
            f"{field_name} must be a finite number"
        ) from exc
    if not math.isfinite(result):
        raise ControllerTraceValidationError(
            f"{field_name} must be a finite number"
        )
    return result


def _positive_number(value, field_name):
    result = _finite_number(value, field_name)
    if result <= 0:
        raise ControllerTraceValidationError(
            f"{field_name} must be positive"
        )
    return result


def _nonnegative_number(value, field_name):
    result = _finite_number(value, field_name)
    if result < 0:
        raise ControllerTraceValidationError(
            f"{field_name} must be non-negative"
        )
    return result


def _fixed_float_tuple(values, expected_length, field_name):
    if isinstance(values, (str, bytes, bytearray)):
        raise ControllerTraceValidationError(
            f"{field_name} must contain {expected_length} numbers"
        )
    try:
        iterator = iter(values)
    except Exception as exc:
        raise ControllerTraceValidationError(
            f"{field_name} must contain {expected_length} numbers"
        ) from exc
    result = []
    for index in range(expected_length + 1):
        try:
            value = next(iterator)
        except StopIteration:
            break
        except Exception as exc:
            raise ControllerTraceValidationError(
                f"{field_name} iteration failed"
            ) from exc
        result.append(_finite_number(value, f"{field_name}[{index}]"))
    if len(result) != expected_length:
        raise ControllerTraceValidationError(
            f"{field_name} must contain {expected_length} numbers"
        )
    return tuple(result)


def _controller_float(value, field_name):
    numeric = _finite_number(value, field_name)
    try:
        packed = struct.pack("<f", numeric)
        controller_value = struct.unpack("<f", packed)[0]
    except (OverflowError, struct.error) as exc:
        raise ControllerTraceValidationError(
            f"{field_name} exceeds the controller numeric range"
        ) from exc
    if not math.isfinite(controller_value):
        raise ControllerTraceValidationError(
            f"{field_name} exceeds the controller numeric range"
        )
    return controller_value


def _controller_float_tuple(values, expected_length, field_name):
    normalized = _fixed_float_tuple(values, expected_length, field_name)
    return tuple(
        _controller_float(value, f"{field_name}[{index}]")
        for index, value in enumerate(normalized)
    )


def _encoder_position_tuple(values):
    positions = _fixed_float_tuple(
        values,
        CONTROLLER_TRACE_AXIS_COUNT,
        "sample encoder_positions",
    )
    normalized = []
    for index, position in enumerate(positions):
        scaled = position * 1000.0
        if not math.isfinite(scaled):
            raise ControllerTraceValidationError(
                f"sample encoder_positions[{index}] exceeds the telemetry range"
            )
        millidegrees = round(scaled)
        if (
            millidegrees < CONTROLLER_TRACE_MINIMUM_ENCODER_MILLIDEGREES
            or millidegrees > CONTROLLER_TRACE_MAXIMUM_ENCODER_MILLIDEGREES
        ):
            raise ControllerTraceValidationError(
                f"sample encoder_positions[{index}] exceeds the telemetry range"
            )
        if not math.isclose(
            scaled,
            millidegrees,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise ControllerTraceValidationError(
                f"sample encoder_positions[{index}] must use millidegree resolution"
            )
        normalized.append(millidegrees / 1000.0)
    return tuple(normalized)


def _bounded_items(values, maximum_count, field_name):
    if isinstance(values, (str, bytes, bytearray)):
        raise ControllerTraceValidationError(f"{field_name} must be a sequence")
    try:
        iterator = iter(values)
    except Exception as exc:
        raise ControllerTraceValidationError(
            f"{field_name} must be a sequence"
        ) from exc
    items = []
    for _ in range(maximum_count + 1):
        try:
            items.append(next(iterator))
        except StopIteration:
            break
        except Exception as exc:
            raise ControllerTraceValidationError(
                f"{field_name} iteration failed"
            ) from exc
    if len(items) > maximum_count:
        raise ControllerTraceValidationError(
            f"{field_name} exceeds the maximum sample count"
        )
    return tuple(items)


def _bounded_ascii_text(value, maximum_length, field_name):
    if (
        not isinstance(value, str)
        or not value
        or len(value) > maximum_length
        or not value.isascii()
        or any(ord(character) < 32 or ord(character) == 127 for character in value)
    ):
        raise ControllerTraceValidationError(
            f"{field_name} must be bounded printable ASCII text"
        )
    return value


@dataclass(frozen=True)
class ControllerMotionProfile:
    """Controller RJ timing inputs preserved with a recorded trace."""

    speed_mode: str
    speed_value: float
    acceleration_percent: float
    deceleration_percent: float
    ramp_percent: float

    def __post_init__(self):
        if (
            not isinstance(self.speed_mode, str)
            or self.speed_mode not in ("p", "s", "m")
        ):
            raise ControllerTraceValidationError(
                "motion-profile speed_mode must be 'p', 's', or 'm'"
            )
        speed = _controller_float(
            self.speed_value,
            "motion-profile speed_value",
        )
        if speed <= 0:
            raise ControllerTraceValidationError(
                "motion-profile speed_value must be positive"
            )
        if self.speed_mode == "p" and speed > 100:
            raise ControllerTraceValidationError(
                "percent motion-profile speed_value must not exceed 100"
            )
        acceleration = _controller_float(
            self.acceleration_percent,
            "motion-profile acceleration_percent",
        )
        deceleration = _controller_float(
            self.deceleration_percent,
            "motion-profile deceleration_percent",
        )
        ramp = _controller_float(
            self.ramp_percent,
            "motion-profile ramp_percent",
        )
        if acceleration <= 0:
            raise ControllerTraceValidationError(
                "motion-profile acceleration_percent must be positive"
            )
        if deceleration <= 0:
            raise ControllerTraceValidationError(
                "motion-profile deceleration_percent must be positive"
            )
        if ramp <= 0:
            raise ControllerTraceValidationError(
                "motion-profile ramp_percent must be positive"
            )
        if acceleration > 100:
            raise ControllerTraceValidationError(
                "motion-profile acceleration_percent must not exceed 100"
            )
        if deceleration >= 100:
            raise ControllerTraceValidationError(
                "motion-profile deceleration_percent must be less than 100"
            )
        if acceleration + deceleration > 100:
            raise ControllerTraceValidationError(
                "motion-profile acceleration and deceleration must not overlap"
            )
        if ramp > 100:
            raise ControllerTraceValidationError(
                "motion-profile ramp_percent must not exceed 100"
            )
        object.__setattr__(self, "speed_value", speed)
        object.__setattr__(self, "acceleration_percent", acceleration)
        object.__setattr__(self, "deceleration_percent", deceleration)
        object.__setattr__(self, "ramp_percent", ramp)


@dataclass(frozen=True)
class ControllerTraceMetadata:
    """Identity and command inputs required to interpret one trace."""

    controller_hardware_id: str
    firmware_version: str
    configuration_fingerprint: str
    start_positions: Tuple[float, ...]
    target_positions: Tuple[float, ...]
    motion_profile: ControllerMotionProfile
    expected_sample_period_seconds: float = (
        CONTROLLER_TRACE_EXPECTED_SAMPLE_PERIOD_SECONDS
    )

    def __post_init__(self):
        if (
            not isinstance(self.controller_hardware_id, str)
            or re.fullmatch(
                r"[0-9A-F]{6}",
                self.controller_hardware_id,
            ) is None
        ):
            raise ControllerTraceValidationError(
                "controller_hardware_id must be six uppercase hexadecimal characters"
            )
        firmware_version = _bounded_ascii_text(
            self.firmware_version,
            64,
            "firmware_version",
        )
        if (
            not isinstance(self.configuration_fingerprint, str)
            or re.fullmatch(
                r"sha256:[0-9a-f]{64}",
                self.configuration_fingerprint,
            ) is None
        ):
            raise ControllerTraceValidationError(
                "configuration_fingerprint must be a lowercase SHA-256 identifier"
            )
        starts = _controller_float_tuple(
            self.start_positions,
            CONTROLLER_TRACE_AXIS_COUNT,
            "start_positions",
        )
        targets = _controller_float_tuple(
            self.target_positions,
            CONTROLLER_TRACE_AXIS_COUNT,
            "target_positions",
        )
        if not isinstance(self.motion_profile, ControllerMotionProfile):
            raise ControllerTraceValidationError(
                "motion_profile must be ControllerMotionProfile"
            )
        expected_period = _positive_number(
            self.expected_sample_period_seconds,
            "expected_sample_period_seconds",
        )
        if expected_period != CONTROLLER_TRACE_EXPECTED_SAMPLE_PERIOD_SECONDS:
            raise ControllerTraceValidationError(
                "expected_sample_period_seconds does not match JOINT_TELEMETRY_V1"
            )
        object.__setattr__(self, "firmware_version", firmware_version)
        object.__setattr__(self, "start_positions", starts)
        object.__setattr__(self, "target_positions", targets)
        object.__setattr__(
            self,
            "expected_sample_period_seconds",
            expected_period,
        )


@dataclass(frozen=True)
class ControllerTraceSample:
    """J1-J6 host-received encoder positions offset from pre-RJ-write time."""

    elapsed_seconds: float
    encoder_positions: Tuple[float, ...]

    def __post_init__(self):
        elapsed = _nonnegative_number(
            self.elapsed_seconds,
            "sample elapsed_seconds",
        )
        positions = _encoder_position_tuple(self.encoder_positions)
        object.__setattr__(self, "elapsed_seconds", elapsed)
        object.__setattr__(self, "encoder_positions", positions)


@dataclass(frozen=True)
class ControllerTraceTerminal:
    """Terminal controller result following the last encoder sample."""

    elapsed_seconds: float
    outcome: str
    reported_positions: Optional[Tuple[float, ...]] = None
    detail: Optional[str] = None

    def __post_init__(self):
        elapsed = _nonnegative_number(
            self.elapsed_seconds,
            "terminal elapsed_seconds",
        )
        if (
            not isinstance(self.outcome, str)
            or self.outcome not in ("completed", "failed", "stopped")
        ):
            raise ControllerTraceValidationError(
                "terminal outcome must be completed, failed, or stopped"
            )
        positions = self.reported_positions
        if positions is not None:
            positions = _controller_float_tuple(
                positions,
                CONTROLLER_TRACE_AXIS_COUNT,
                "terminal reported_positions",
            )
        detail = self.detail
        if self.outcome == "completed":
            if positions is None or detail is not None:
                raise ControllerTraceValidationError(
                    "completed terminal requires reported_positions and no detail"
                )
        else:
            detail = _bounded_ascii_text(
                detail,
                CONTROLLER_TRACE_MAXIMUM_DETAIL_LENGTH,
                "terminal detail",
            )
        object.__setattr__(self, "elapsed_seconds", elapsed)
        object.__setattr__(self, "reported_positions", positions)
        object.__setattr__(self, "detail", detail)


@dataclass(frozen=True)
class ControllerTrace:
    """A complete bounded RJ telemetry exchange trace."""

    metadata: ControllerTraceMetadata
    samples: Tuple[ControllerTraceSample, ...]
    terminal: ControllerTraceTerminal

    def __post_init__(self):
        if not isinstance(self.metadata, ControllerTraceMetadata):
            raise ControllerTraceValidationError(
                "trace metadata must be ControllerTraceMetadata"
            )
        if not isinstance(self.terminal, ControllerTraceTerminal):
            raise ControllerTraceValidationError(
                "trace terminal must be ControllerTraceTerminal"
            )
        samples = _bounded_items(
            self.samples,
            CONTROLLER_TRACE_MAXIMUM_SAMPLES,
            "trace samples",
        )
        previous_elapsed = -1.0
        for sample in samples:
            if not isinstance(sample, ControllerTraceSample):
                raise ControllerTraceValidationError(
                    "trace samples must be ControllerTraceSample values"
                )
            if sample.elapsed_seconds <= previous_elapsed:
                raise ControllerTraceValidationError(
                    "trace sample timestamps must advance strictly"
                )
            if sample.elapsed_seconds > self.terminal.elapsed_seconds:
                raise ControllerTraceValidationError(
                    "trace sample cannot follow the terminal result"
                )
            previous_elapsed = sample.elapsed_seconds
        object.__setattr__(self, "samples", samples)


@dataclass(frozen=True)
class TraceCadenceMetrics:
    sample_count: int
    interval_count: int
    first_sample_latency_seconds: Optional[float]
    last_sample_elapsed_seconds: Optional[float]
    minimum_interval_seconds: Optional[float]
    maximum_interval_seconds: Optional[float]
    mean_interval_seconds: Optional[float]
    median_interval_seconds: Optional[float]
    cadence_gap_count: int
    maximum_gap_multiple: Optional[float]


@dataclass(frozen=True)
class JointTraceMetrics:
    axis: int
    commanded_start_position_degrees: float
    target_position_degrees: float
    commanded_displacement_degrees: float
    first_encoder_position_degrees: Optional[float]
    initial_encoder_error_degrees: Optional[float]
    final_encoder_position_degrees: Optional[float]
    encoder_displacement_degrees: Optional[float]
    final_encoder_error_degrees: Optional[float]
    terminal_reported_error_degrees: Optional[float]
    commanded_direction: int
    overshoot_degrees: Optional[float]
    velocity_sample_count: int
    acceleration_sample_count: int
    jerk_sample_count: int
    peak_absolute_speed_degrees_per_second: Optional[float]
    peak_absolute_speed_at_seconds: Optional[float]
    peak_speed_toward_target_degrees_per_second: Optional[float]
    peak_reverse_speed_degrees_per_second: Optional[float]
    peak_acceleration_toward_target_degrees_per_second_squared: Optional[float]
    peak_deceleration_degrees_per_second_squared: Optional[float]
    peak_absolute_jerk_degrees_per_second_cubed: Optional[float]


@dataclass(frozen=True)
class ControllerTraceAnalysis:
    cadence: TraceCadenceMetrics
    joints: Tuple[JointTraceMetrics, ...]
    terminal_outcome: str
    terminal_elapsed_seconds: float
    profile_analysis_eligible: bool
    blocking_reasons: Tuple[str, ...]
    measurement_notes: Tuple[str, ...]


def _bounded_limit(value, maximum, field_name):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ControllerTraceFormatError(f"{field_name} must be an integer")
    result = int(value)
    if not 1 <= result <= maximum:
        raise ControllerTraceFormatError(
            f"{field_name} must be between 1 and {maximum}"
        )
    return result


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if not isinstance(key, str) or key in result:
            raise ControllerTraceFormatError(
                f"duplicated or invalid JSON key: {key}"
            )
        result[key] = value
    return result


def _reject_json_constant(value):
    raise ControllerTraceFormatError(f"non-finite JSON number: {value}")


def _parse_json_line(line, line_number):
    try:
        parsed = json.loads(
            line,
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
        )
    except ControllerTraceFormatError as exc:
        raise ControllerTraceFormatError(
            f"line {line_number}: {exc}"
        ) from exc
    except (TypeError, ValueError, RecursionError) as exc:
        raise ControllerTraceFormatError(
            f"line {line_number}: invalid JSON"
        ) from exc
    if not isinstance(parsed, dict):
        raise ControllerTraceFormatError(
            f"line {line_number}: record must be a JSON object"
        )
    return parsed


def _trace_lines(payload):
    if not isinstance(payload, (bytes, bytearray)):
        raise ControllerTraceFormatError("trace payload must be bytes")
    payload = bytes(payload)
    if not payload:
        raise ControllerTraceFormatError("trace payload must not be empty")
    if b"\x00" in payload:
        raise ControllerTraceFormatError(
            "trace payload must not contain NUL bytes"
        )
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ControllerTraceFormatError(
            "trace payload must be valid UTF-8"
        ) from exc
    raw_lines = text.split("\n")
    if raw_lines and raw_lines[-1] == "":
        raw_lines.pop()
    lines = []
    for line_number, raw_line in enumerate(raw_lines, start=1):
        if raw_line.endswith("\r"):
            raw_line = raw_line[:-1]
        if not raw_line or "\r" in raw_line:
            raise ControllerTraceFormatError(
                f"line {line_number}: blank or malformed line"
            )
        if len(raw_line.encode("utf-8")) > CONTROLLER_TRACE_MAXIMUM_LINE_BYTES:
            raise ControllerTraceFormatError(
                f"line {line_number}: record exceeds the line limit"
            )
        lines.append(raw_line)
    return tuple(lines)


def _decode_metadata(header):
    expected_keys = {
        "axis_count",
        "configuration_fingerprint",
        "controller_hardware_id",
        "expected_sample_period_seconds",
        "firmware_version",
        "motion_profile",
        "position_unit",
        "schema",
        "source",
        "start_positions",
        "target_positions",
        "timebase",
        "time_origin",
    }
    if set(header) != expected_keys:
        raise ControllerTraceFormatError(
            "line 1: controller-trace header fields do not match the schema"
        )
    if (
        isinstance(header["axis_count"], bool)
        or not isinstance(header["axis_count"], int)
        or header["axis_count"] != CONTROLLER_TRACE_AXIS_COUNT
        or header["position_unit"] != CONTROLLER_TRACE_POSITION_UNIT
        or header["schema"] != CONTROLLER_TRACE_SCHEMA
        or header["source"] != CONTROLLER_TRACE_SOURCE
        or header["timebase"] != CONTROLLER_TRACE_TIMEBASE
        or header["time_origin"] != CONTROLLER_TRACE_TIME_ORIGIN
    ):
        raise ControllerTraceFormatError(
            "line 1: unsupported controller-trace header"
        )
    profile = header["motion_profile"]
    expected_profile_keys = {
        "acceleration_percent",
        "deceleration_percent",
        "ramp_percent",
        "speed_mode",
        "speed_value",
    }
    if not isinstance(profile, dict) or set(profile) != expected_profile_keys:
        raise ControllerTraceFormatError(
            "line 1: motion-profile fields do not match the schema"
        )
    try:
        motion_profile = ControllerMotionProfile(
            speed_mode=profile["speed_mode"],
            speed_value=profile["speed_value"],
            acceleration_percent=profile["acceleration_percent"],
            deceleration_percent=profile["deceleration_percent"],
            ramp_percent=profile["ramp_percent"],
        )
        return ControllerTraceMetadata(
            controller_hardware_id=header["controller_hardware_id"],
            firmware_version=header["firmware_version"],
            configuration_fingerprint=header["configuration_fingerprint"],
            start_positions=header["start_positions"],
            target_positions=header["target_positions"],
            motion_profile=motion_profile,
            expected_sample_period_seconds=(
                header["expected_sample_period_seconds"]
            ),
        )
    except ControllerTraceValidationError as exc:
        raise ControllerTraceFormatError(f"line 1: {exc}") from exc


def decode_controller_trace(
    payload,
    maximum_bytes=CONTROLLER_TRACE_MAXIMUM_BYTES,
    maximum_samples=CONTROLLER_TRACE_MAXIMUM_SAMPLES,
):
    """Decode a complete bounded controller trace from strict UTF-8 JSONL."""

    byte_limit = _bounded_limit(
        maximum_bytes,
        CONTROLLER_TRACE_MAXIMUM_BYTES,
        "maximum_bytes",
    )
    sample_limit = _bounded_limit(
        maximum_samples,
        CONTROLLER_TRACE_MAXIMUM_SAMPLES,
        "maximum_samples",
    )
    if not isinstance(payload, (bytes, bytearray)):
        raise ControllerTraceFormatError("trace payload must be bytes")
    payload = bytes(payload)
    if len(payload) > byte_limit:
        raise ControllerTraceFormatError("trace payload exceeds the byte limit")
    lines = _trace_lines(payload)
    if len(lines) < 2:
        raise ControllerTraceFormatError(
            "trace payload must contain a header and terminal record"
        )
    if len(lines) - 2 > sample_limit:
        raise ControllerTraceFormatError(
            "trace payload exceeds the sample limit"
        )

    metadata = _decode_metadata(_parse_json_line(lines[0], 1))
    samples = []
    expected_sample_keys = {"elapsed_seconds", "encoder_positions", "kind"}
    for line_number, line in enumerate(lines[1:-1], start=2):
        record = _parse_json_line(line, line_number)
        if set(record) != expected_sample_keys or record.get("kind") != "sample":
            raise ControllerTraceFormatError(
                f"line {line_number}: sample fields do not match the schema"
            )
        try:
            samples.append(ControllerTraceSample(
                elapsed_seconds=record["elapsed_seconds"],
                encoder_positions=record["encoder_positions"],
            ))
        except ControllerTraceValidationError as exc:
            raise ControllerTraceFormatError(
                f"line {line_number}: {exc}"
            ) from exc

    terminal_line_number = len(lines)
    terminal_record = _parse_json_line(lines[-1], terminal_line_number)
    expected_terminal_keys = {
        "detail",
        "elapsed_seconds",
        "kind",
        "outcome",
        "reported_positions",
    }
    if (
        set(terminal_record) != expected_terminal_keys
        or terminal_record.get("kind") != "terminal"
    ):
        raise ControllerTraceFormatError(
            f"line {terminal_line_number}: terminal fields do not match the schema"
        )
    try:
        terminal = ControllerTraceTerminal(
            elapsed_seconds=terminal_record["elapsed_seconds"],
            outcome=terminal_record["outcome"],
            reported_positions=terminal_record["reported_positions"],
            detail=terminal_record["detail"],
        )
        return ControllerTrace(metadata, tuple(samples), terminal)
    except ControllerTraceValidationError as exc:
        raise ControllerTraceFormatError(
            f"trace sequence is invalid: {exc}"
        ) from exc


def encode_controller_trace(trace):
    """Encode a controller trace using the canonical JSONL representation."""

    if not isinstance(trace, ControllerTrace):
        raise ControllerTraceFormatError("trace must be ControllerTrace")
    metadata = trace.metadata
    profile = metadata.motion_profile
    records = [{
        "axis_count": CONTROLLER_TRACE_AXIS_COUNT,
        "configuration_fingerprint": metadata.configuration_fingerprint,
        "controller_hardware_id": metadata.controller_hardware_id,
        "expected_sample_period_seconds": (
            metadata.expected_sample_period_seconds
        ),
        "firmware_version": metadata.firmware_version,
        "motion_profile": {
            "acceleration_percent": profile.acceleration_percent,
            "deceleration_percent": profile.deceleration_percent,
            "ramp_percent": profile.ramp_percent,
            "speed_mode": profile.speed_mode,
            "speed_value": profile.speed_value,
        },
        "position_unit": CONTROLLER_TRACE_POSITION_UNIT,
        "schema": CONTROLLER_TRACE_SCHEMA,
        "source": CONTROLLER_TRACE_SOURCE,
        "start_positions": list(metadata.start_positions),
        "target_positions": list(metadata.target_positions),
        "timebase": CONTROLLER_TRACE_TIMEBASE,
        "time_origin": CONTROLLER_TRACE_TIME_ORIGIN,
    }]
    records.extend({
        "elapsed_seconds": sample.elapsed_seconds,
        "encoder_positions": list(sample.encoder_positions),
        "kind": "sample",
    } for sample in trace.samples)
    terminal = trace.terminal
    records.append({
        "detail": terminal.detail,
        "elapsed_seconds": terminal.elapsed_seconds,
        "kind": "terminal",
        "outcome": terminal.outcome,
        "reported_positions": (
            list(terminal.reported_positions)
            if terminal.reported_positions is not None
            else None
        ),
    })
    try:
        lines = [
            json.dumps(
                record,
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            for record in records
        ]
    except (TypeError, ValueError, RecursionError) as exc:
        raise ControllerTraceFormatError(
            "controller trace cannot be encoded as JSON"
        ) from exc
    if any(
        len(line.encode("utf-8")) > CONTROLLER_TRACE_MAXIMUM_LINE_BYTES
        for line in lines
    ):
        raise ControllerTraceFormatError(
            "encoded controller-trace record exceeds the line limit"
        )
    payload = ("\n".join(lines) + "\n").encode("utf-8")
    if len(payload) > CONTROLLER_TRACE_MAXIMUM_BYTES:
        raise ControllerTraceFormatError(
            "encoded controller trace exceeds the byte limit"
        )
    return payload


def _finite_analysis_result(value, field_name):
    if not math.isfinite(value):
        raise ControllerTraceAnalysisError(
            f"{field_name} cannot be represented"
        )
    return value


def _analysis_difference(left, right, field_name):
    return _finite_analysis_result(left - right, field_name)


def _derivative_series(series, field_name):
    derivatives = []
    for (left_time, left_value), (right_time, right_value) in zip(
        series,
        series[1:],
    ):
        interval = right_time - left_time
        if interval <= 0 or not math.isfinite(interval):
            raise ControllerTraceAnalysisError(
                f"{field_name} timestamps must advance strictly"
            )
        timestamp = _finite_analysis_result(
            left_time + interval / 2.0,
            f"{field_name} timestamp",
        )
        derivative = _finite_analysis_result(
            (right_value - left_value) / interval,
            field_name,
        )
        derivatives.append((timestamp, derivative))
    return tuple(derivatives)


def _interval_mean(intervals):
    try:
        result = math.fsum(intervals) / len(intervals)
    except (OverflowError, ZeroDivisionError) as exc:
        raise ControllerTraceAnalysisError(
            "mean telemetry interval cannot be represented"
        ) from exc
    return _finite_analysis_result(result, "mean telemetry interval")


def _peak_absolute(series):
    if not series:
        return None, None
    timestamp, value = max(series, key=lambda item: abs(item[1]))
    return abs(value), timestamp


def _joint_metrics(trace, axis):
    start = trace.metadata.start_positions[axis]
    target = trace.metadata.target_positions[axis]
    commanded_displacement = _analysis_difference(
        target,
        start,
        f"J{axis + 1} commanded displacement",
    )
    direction = (
        1 if commanded_displacement > 0
        else -1 if commanded_displacement < 0
        else 0
    )
    terminal_positions = trace.terminal.reported_positions
    terminal_error = (
        _analysis_difference(
            terminal_positions[axis],
            target,
            f"J{axis + 1} terminal reported error",
        )
        if terminal_positions is not None
        else None
    )
    if not trace.samples:
        return JointTraceMetrics(
            axis=axis + 1,
            commanded_start_position_degrees=start,
            target_position_degrees=target,
            commanded_displacement_degrees=commanded_displacement,
            first_encoder_position_degrees=None,
            initial_encoder_error_degrees=None,
            final_encoder_position_degrees=None,
            encoder_displacement_degrees=None,
            final_encoder_error_degrees=None,
            terminal_reported_error_degrees=terminal_error,
            commanded_direction=direction,
            overshoot_degrees=None,
            velocity_sample_count=0,
            acceleration_sample_count=0,
            jerk_sample_count=0,
            peak_absolute_speed_degrees_per_second=None,
            peak_absolute_speed_at_seconds=None,
            peak_speed_toward_target_degrees_per_second=None,
            peak_reverse_speed_degrees_per_second=None,
            peak_acceleration_toward_target_degrees_per_second_squared=None,
            peak_deceleration_degrees_per_second_squared=None,
            peak_absolute_jerk_degrees_per_second_cubed=None,
        )

    position_series = tuple(
        (sample.elapsed_seconds, sample.encoder_positions[axis])
        for sample in trace.samples
    )
    velocities = _derivative_series(
        position_series,
        f"J{axis + 1} velocity",
    )
    accelerations = _derivative_series(
        velocities,
        f"J{axis + 1} acceleration",
    )
    jerks = _derivative_series(
        accelerations,
        f"J{axis + 1} jerk",
    )
    first = position_series[0][1]
    final = position_series[-1][1]
    peak_speed, peak_speed_time = _peak_absolute(velocities)
    peak_jerk, _ = _peak_absolute(jerks)
    if direction and velocities:
        peak_toward = max(
            (direction * value for _, value in velocities),
        )
        peak_reverse = max(
            (-direction * value for _, value in velocities),
        )
        peak_toward = max(0.0, peak_toward)
        peak_reverse = max(0.0, peak_reverse)
    else:
        peak_toward = None
        peak_reverse = None
    if direction and accelerations:
        peak_acceleration = max(
            (direction * value for _, value in accelerations),
        )
        peak_deceleration = max(
            (-direction * value for _, value in accelerations),
        )
        peak_acceleration = max(0.0, peak_acceleration)
        peak_deceleration = max(0.0, peak_deceleration)
    else:
        peak_acceleration = None
        peak_deceleration = None
    if direction:
        projected_target_errors = tuple(
            _finite_analysis_result(
                direction * _analysis_difference(
                    position,
                    target,
                    f"J{axis + 1} target error",
                ),
                f"J{axis + 1} projected target error",
            )
            for _, position in position_series
        )
        overshoot = max(
            0.0,
            max(projected_target_errors),
        )
    else:
        overshoot = None

    return JointTraceMetrics(
        axis=axis + 1,
        commanded_start_position_degrees=start,
        target_position_degrees=target,
        commanded_displacement_degrees=commanded_displacement,
        first_encoder_position_degrees=first,
        initial_encoder_error_degrees=_analysis_difference(
            first,
            start,
            f"J{axis + 1} initial encoder error",
        ),
        final_encoder_position_degrees=final,
        encoder_displacement_degrees=_analysis_difference(
            final,
            first,
            f"J{axis + 1} encoder displacement",
        ),
        final_encoder_error_degrees=_analysis_difference(
            final,
            target,
            f"J{axis + 1} final encoder error",
        ),
        terminal_reported_error_degrees=terminal_error,
        commanded_direction=direction,
        overshoot_degrees=overshoot,
        velocity_sample_count=len(velocities),
        acceleration_sample_count=len(accelerations),
        jerk_sample_count=len(jerks),
        peak_absolute_speed_degrees_per_second=peak_speed,
        peak_absolute_speed_at_seconds=peak_speed_time,
        peak_speed_toward_target_degrees_per_second=peak_toward,
        peak_reverse_speed_degrees_per_second=peak_reverse,
        peak_acceleration_toward_target_degrees_per_second_squared=(
            peak_acceleration
        ),
        peak_deceleration_degrees_per_second_squared=peak_deceleration,
        peak_absolute_jerk_degrees_per_second_cubed=peak_jerk,
    )


def analyze_controller_trace(trace, cadence_gap_factor=1.5):
    """Derive non-uniform-time motion metrics without claiming live tuning."""

    if not isinstance(trace, ControllerTrace):
        raise ControllerTraceAnalysisError("trace must be ControllerTrace")
    try:
        gap_factor = _positive_number(
            cadence_gap_factor,
            "cadence_gap_factor",
        )
    except ControllerTraceValidationError as exc:
        raise ControllerTraceAnalysisError(str(exc)) from exc
    if gap_factor <= 1.0:
        raise ControllerTraceAnalysisError(
            "cadence_gap_factor must be greater than 1"
        )

    timestamps = tuple(sample.elapsed_seconds for sample in trace.samples)
    intervals = tuple(
        right - left for left, right in zip(timestamps, timestamps[1:])
    )
    expected_period = trace.metadata.expected_sample_period_seconds
    gap_threshold = _finite_analysis_result(
        expected_period * gap_factor,
        "cadence gap threshold",
    )
    gap_count = sum(interval > gap_threshold for interval in intervals)
    maximum_gap_multiple = (
        _finite_analysis_result(
            max(intervals) / expected_period,
            "maximum cadence gap multiple",
        )
        if intervals
        else None
    )
    cadence = TraceCadenceMetrics(
        sample_count=len(timestamps),
        interval_count=len(intervals),
        first_sample_latency_seconds=timestamps[0] if timestamps else None,
        last_sample_elapsed_seconds=timestamps[-1] if timestamps else None,
        minimum_interval_seconds=min(intervals) if intervals else None,
        maximum_interval_seconds=max(intervals) if intervals else None,
        mean_interval_seconds=_interval_mean(intervals) if intervals else None,
        median_interval_seconds=(
            statistics.median(intervals) if intervals else None
        ),
        cadence_gap_count=gap_count,
        maximum_gap_multiple=maximum_gap_multiple,
    )
    joints = tuple(
        _joint_metrics(trace, axis)
        for axis in range(CONTROLLER_TRACE_AXIS_COUNT)
    )

    blocking_reasons = []
    if trace.terminal.outcome != "completed":
        blocking_reasons.append(
            "controller exchange did not complete successfully"
        )
    if len(trace.samples) < 4:
        blocking_reasons.append(
            "at least four encoder samples are required for jerk estimation"
        )
    if not any(joint.commanded_direction for joint in joints):
        blocking_reasons.append(
            "trace contains no commanded J1-J6 displacement"
        )
    if gap_count:
        blocking_reasons.append(
            "telemetry cadence contains one or more sampling gaps"
        )
    initial_coverage_gap = (
        trace.samples
        and trace.samples[0].elapsed_seconds > gap_threshold
    )
    if initial_coverage_gap:
        blocking_reasons.append(
            "encoder sampling began after the initial response window"
        )
    terminal_coverage_gap = (
        trace.samples
        and trace.terminal.elapsed_seconds - trace.samples[-1].elapsed_seconds
        > gap_threshold
    )
    if terminal_coverage_gap:
        blocking_reasons.append(
            "encoder sampling ended before the terminal response window"
        )

    notes = (
        "sample timestamps are host receipt offsets; controller generation timestamps are unavailable",
        "terminal reported positions are controller step-counter state, not encoder samples",
        "finite differences amplify encoder quantization and require repeated-trace confirmation",
    )
    return ControllerTraceAnalysis(
        cadence=cadence,
        joints=joints,
        terminal_outcome=trace.terminal.outcome,
        terminal_elapsed_seconds=trace.terminal.elapsed_seconds,
        profile_analysis_eligible=not blocking_reasons,
        blocking_reasons=tuple(blocking_reasons),
        measurement_notes=notes,
    )
