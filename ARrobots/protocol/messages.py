"""Strict newline-delimited JSON protocol envelopes.

The protocol keeps command payloads independent from serial ownership.  A
single transport owner can therefore correlate replies while routing
controller events and low-priority telemetry through separate channels.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import json
import math
import re
from types import MappingProxyType
from typing import Optional


JSON_PROTOCOL_VERSION = 1
JSON_PROTOCOL_MAXIMUM_FRAME_BYTES = 4096
JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES = JSON_PROTOCOL_MAXIMUM_FRAME_BYTES - 2
JSON_PROTOCOL_MAXIMUM_DEPTH = 8
JSON_PROTOCOL_MAXIMUM_ARRAY_ITEMS = 128
JSON_PROTOCOL_MAXIMUM_OBJECT_FIELDS = 128
JSON_PROTOCOL_MAXIMUM_STRING_LENGTH = 1024
JSON_PROTOCOL_MAXIMUM_ERROR_MESSAGE_LENGTH = 512
JSON_PROTOCOL_MINIMUM_INTEGER = -(2 ** 63)
JSON_PROTOCOL_MAXIMUM_INTEGER = 2 ** 64 - 1
JSON_PROTOCOL_MAXIMUM_IDENTIFIER = 2 ** 32 - 1

_NAME_PATTERN = re.compile(r"[a-z][a-z0-9_]{0,63}\Z")
_FIELD_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,63}\Z")
_RESPONSE_STATUSES = frozenset(
    ("accepted", "completed", "rejected", "cancelled", "failed")
)
_TERMINAL_RESPONSE_STATUSES = frozenset(
    ("completed", "rejected", "cancelled", "failed")
)
_ERROR_RESPONSE_STATUSES = frozenset(("rejected", "cancelled", "failed"))


class JsonProtocolError(ValueError):
    """Base error for an invalid JSON protocol value or frame."""


class JsonProtocolValidationError(JsonProtocolError):
    """A locally constructed message violates the protocol schema."""


class JsonProtocolDecodeError(JsonProtocolError):
    """A received frame is malformed, unsupported, or ambiguous."""


def _printable_ascii(value, field_name, maximum_length, allow_empty=False):
    if type(value) is not str:
        raise JsonProtocolValidationError(f"{field_name} must be text")
    if (not allow_empty and not value) or len(value) > maximum_length:
        qualifier = "non-empty " if not allow_empty else ""
        raise JsonProtocolValidationError(
            f"{field_name} must be {qualifier}text no longer than "
            f"{maximum_length} characters"
        )
    if not value.isascii() or any(
        ord(character) < 32 or ord(character) == 127
        for character in value
    ):
        raise JsonProtocolValidationError(
            f"{field_name} must contain printable ASCII characters"
        )
    return value


def _name(value, field_name):
    normalized = _printable_ascii(value, field_name, 64)
    if _NAME_PATTERN.fullmatch(normalized) is None:
        raise JsonProtocolValidationError(
            f"{field_name} must use lower-case snake-case protocol syntax"
        )
    return normalized


def _identifier(value, field_name, allow_zero=False):
    if type(value) is not int:
        raise JsonProtocolValidationError(
            f"{field_name} must be an unsigned 32-bit integer"
        )
    minimum = 0 if allow_zero else 1
    if value < minimum or value > JSON_PROTOCOL_MAXIMUM_IDENTIFIER:
        raise JsonProtocolValidationError(
            f"{field_name} must be between {minimum} and "
            f"{JSON_PROTOCOL_MAXIMUM_IDENTIFIER}"
        )
    return value


def _freeze_json_value(value, field_name, depth=0):
    if depth > JSON_PROTOCOL_MAXIMUM_DEPTH:
        raise JsonProtocolValidationError(
            f"{field_name} exceeds the maximum JSON nesting depth"
        )
    if value is None or type(value) is bool:
        return value
    if type(value) is int:
        if (
            value < JSON_PROTOCOL_MINIMUM_INTEGER
            or value > JSON_PROTOCOL_MAXIMUM_INTEGER
        ):
            raise JsonProtocolValidationError(
                f"{field_name} exceeds the protocol integer range"
            )
        return value
    if type(value) is float:
        if not math.isfinite(value):
            raise JsonProtocolValidationError(
                f"{field_name} must be a finite number"
            )
        return value
    if type(value) is str:
        return _printable_ascii(
            value,
            field_name,
            JSON_PROTOCOL_MAXIMUM_STRING_LENGTH,
            allow_empty=True,
        )
    if isinstance(value, Mapping):
        try:
            items = []
            for item in value.items():
                if len(items) >= JSON_PROTOCOL_MAXIMUM_OBJECT_FIELDS:
                    raise JsonProtocolValidationError(
                        f"{field_name} contains too many object fields"
                    )
                if type(item) not in (list, tuple) or len(item) != 2:
                    raise JsonProtocolValidationError(
                        f"{field_name} object access failed"
                    )
                key, child = item
                items.append((key, child))
        except JsonProtocolValidationError:
            raise
        except Exception as exc:
            raise JsonProtocolValidationError(
                f"{field_name} object access failed"
            ) from exc
        frozen = {}
        for key, child in items:
            if (
                type(key) is not str
                or _FIELD_PATTERN.fullmatch(key) is None
            ):
                raise JsonProtocolValidationError(
                    f"{field_name} contains an invalid object field name"
                )
            if key in frozen:
                raise JsonProtocolValidationError(
                    f"{field_name} contains a duplicate object field"
                )
            frozen[key] = _freeze_json_value(
                child,
                f"{field_name}.{key}",
                depth + 1,
            )
        return MappingProxyType(frozen)
    if type(value) in (list, tuple):
        if len(value) > JSON_PROTOCOL_MAXIMUM_ARRAY_ITEMS:
            raise JsonProtocolValidationError(
                f"{field_name} contains too many array items"
            )
        return tuple(
            _freeze_json_value(
                child,
                f"{field_name}[{index}]",
                depth + 1,
            )
            for index, child in enumerate(value)
        )
    raise JsonProtocolValidationError(
        f"{field_name} contains a value unsupported by JSON"
    )


def _freeze_object(value, field_name):
    frozen = _freeze_json_value(value, field_name)
    if not isinstance(frozen, Mapping):
        raise JsonProtocolValidationError(f"{field_name} must be an object")
    return frozen


def freeze_json_object(value):
    """Return a deeply immutable snapshot accepted by protocol object fields."""
    return _freeze_object(value, "JSON object")


def _thaw_json_value(value):
    if isinstance(value, Mapping):
        return {
            key: _thaw_json_value(child)
            for key, child in value.items()
        }
    if isinstance(value, tuple):
        return [_thaw_json_value(child) for child in value]
    return value


@dataclass(frozen=True)
class ProtocolFailure:
    code: str
    message: str
    details: Mapping

    def __init__(self, code, message, details=None):
        if details is None:
            details = {}
        object.__setattr__(self, "code", _name(code, "error code"))
        object.__setattr__(
            self,
            "message",
            _printable_ascii(
                message,
                "error message",
                JSON_PROTOCOL_MAXIMUM_ERROR_MESSAGE_LENGTH,
            ),
        )
        object.__setattr__(
            self,
            "details",
            _freeze_object(details, "error details"),
        )

    def to_object(self):
        return {
            "code": self.code,
            "message": self.message,
            "details": _thaw_json_value(self.details),
        }


@dataclass(frozen=True)
class Request:
    id: int
    cmd: str
    params: Mapping

    def __init__(self, request_id, cmd, params=None):
        if params is None:
            params = {}
        object.__setattr__(self, "id", _identifier(request_id, "request id"))
        object.__setattr__(self, "cmd", _name(cmd, "command name"))
        object.__setattr__(
            self,
            "params",
            _freeze_object(params, "request parameters"),
        )

    def to_object(self):
        return {
            "v": JSON_PROTOCOL_VERSION,
            "type": "request",
            "id": self.id,
            "cmd": self.cmd,
            "params": _thaw_json_value(self.params),
        }


@dataclass(frozen=True)
class Response:
    id: int
    cmd: str
    status: str
    result: Optional[Mapping]
    error: Optional[ProtocolFailure]

    def __init__(self, request_id, cmd, status, result=None, error=None):
        normalized_status = _name(status, "response status")
        if normalized_status not in _RESPONSE_STATUSES:
            raise JsonProtocolValidationError(
                "response status is unsupported"
            )
        if normalized_status in ("accepted", "completed"):
            if error is not None:
                raise JsonProtocolValidationError(
                    f"{normalized_status} response cannot contain an error"
                )
            if result is None:
                result = {}
        elif normalized_status in _ERROR_RESPONSE_STATUSES:
            if result is not None:
                raise JsonProtocolValidationError(
                    f"{normalized_status} response cannot contain a result"
                )
            if type(error) is not ProtocolFailure:
                raise JsonProtocolValidationError(
                    f"{normalized_status} response requires a protocol error"
                )
        object.__setattr__(self, "id", _identifier(request_id, "request id"))
        object.__setattr__(self, "cmd", _name(cmd, "command name"))
        object.__setattr__(self, "status", normalized_status)
        object.__setattr__(
            self,
            "result",
            None if result is None else _freeze_object(result, "response result"),
        )
        object.__setattr__(self, "error", error)

    @property
    def terminal(self):
        return self.status in _TERMINAL_RESPONSE_STATUSES

    def to_object(self):
        payload = {
            "v": JSON_PROTOCOL_VERSION,
            "type": "response",
            "id": self.id,
            "cmd": self.cmd,
            "status": self.status,
        }
        if self.result is not None:
            payload["result"] = _thaw_json_value(self.result)
        if self.error is not None:
            payload["error"] = self.error.to_object()
        return payload


@dataclass(frozen=True)
class Event:
    seq: int
    event: str
    data: Mapping

    def __init__(self, sequence, event, data=None):
        if data is None:
            data = {}
        object.__setattr__(
            self,
            "seq",
            _identifier(sequence, "event sequence", allow_zero=True),
        )
        object.__setattr__(self, "event", _name(event, "event name"))
        object.__setattr__(self, "data", _freeze_object(data, "event data"))

    def to_object(self):
        return {
            "v": JSON_PROTOCOL_VERSION,
            "type": "event",
            "seq": self.seq,
            "event": self.event,
            "data": _thaw_json_value(self.data),
        }


@dataclass(frozen=True)
class Telemetry:
    seq: int
    stream: str
    data: Mapping

    def __init__(self, sequence, stream, data=None):
        if data is None:
            data = {}
        object.__setattr__(
            self,
            "seq",
            _identifier(sequence, "telemetry sequence", allow_zero=True),
        )
        object.__setattr__(self, "stream", _name(stream, "telemetry stream"))
        object.__setattr__(
            self,
            "data",
            _freeze_object(data, "telemetry data"),
        )

    def to_object(self):
        return {
            "v": JSON_PROTOCOL_VERSION,
            "type": "telemetry",
            "seq": self.seq,
            "stream": self.stream,
            "data": _thaw_json_value(self.data),
        }


@dataclass(frozen=True)
class ProtocolErrorFrame:
    error: ProtocolFailure

    def __init__(self, error):
        if type(error) is not ProtocolFailure:
            raise JsonProtocolValidationError(
                "protocol-error frame requires a protocol error"
            )
        object.__setattr__(self, "error", error)

    def to_object(self):
        return {
            "v": JSON_PROTOCOL_VERSION,
            "type": "protocol_error",
            "error": self.error.to_object(),
        }


_MESSAGE_TYPES = (Request, Response, Event, Telemetry, ProtocolErrorFrame)


def encode_message(message):
    if type(message) not in _MESSAGE_TYPES:
        raise JsonProtocolValidationError(
            "message must use a supported protocol envelope"
        )
    try:
        payload = json.dumps(
            message.to_object(),
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
    except (TypeError, ValueError, UnicodeError) as exc:
        raise JsonProtocolValidationError(
            "message cannot be encoded as canonical JSON"
        ) from exc
    if not payload or len(payload) > JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES:
        raise JsonProtocolValidationError(
            "encoded message exceeds the protocol payload limit"
        )
    return payload + b"\n"


def _reject_json_constant(value):
    raise JsonProtocolDecodeError(
        f"JSON constant {value} is not supported by the protocol"
    )


def _parse_json_float(value):
    try:
        exact = Decimal(value)
        converted = float(exact)
    except (InvalidOperation, OverflowError, ValueError) as exc:
        raise JsonProtocolDecodeError(
            "JSON number is unsupported by the protocol"
        ) from exc
    if (
        not exact.is_finite()
        or not math.isfinite(converted)
        or (converted == 0.0 and not exact.is_zero())
    ):
        raise JsonProtocolDecodeError(
            "JSON number is unsupported by the protocol"
        )
    return converted


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if not isinstance(key, str) or key in result:
            raise JsonProtocolDecodeError(
                "JSON object contains a duplicate or invalid field"
            )
        result[key] = value
    return result


def _decode_json_frame(frame):
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        raise JsonProtocolDecodeError("protocol frame must be bytes")
    try:
        frame_length = (
            frame.nbytes if isinstance(frame, memoryview) else len(frame)
        )
    except Exception as exc:
        raise JsonProtocolDecodeError("protocol frame is inaccessible") from exc
    if not frame_length or frame_length > JSON_PROTOCOL_MAXIMUM_FRAME_BYTES:
        raise JsonProtocolDecodeError("protocol frame exceeds the size limit")
    try:
        raw = bytes(frame)
    except Exception as exc:
        raise JsonProtocolDecodeError("protocol frame is inaccessible") from exc
    if not raw.endswith(b"\n") or b"\n" in raw[:-1]:
        raise JsonProtocolDecodeError(
            "protocol frame must contain exactly one terminal LF"
        )
    payload = raw[:-1]
    if payload.endswith(b"\r"):
        payload = payload[:-1]
    if not payload or len(payload) > JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES:
        raise JsonProtocolDecodeError("protocol payload is empty or oversized")
    if any(byte < 0x20 or byte > 0x7E for byte in payload):
        raise JsonProtocolDecodeError(
            "protocol payload must contain printable ASCII JSON"
        )
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError as exc:
        raise JsonProtocolDecodeError(
            "protocol payload must contain ASCII JSON"
        ) from exc
    try:
        decoded = json.loads(
            text,
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
            parse_float=_parse_json_float,
        )
    except JsonProtocolDecodeError:
        raise
    except (RecursionError, TypeError, ValueError) as exc:
        raise JsonProtocolDecodeError(
            "protocol payload is not valid JSON"
        ) from exc
    if not isinstance(decoded, dict):
        raise JsonProtocolDecodeError("protocol payload must be a JSON object")
    return decoded


def _require_exact_fields(payload, expected):
    if frozenset(payload) != frozenset(expected):
        raise JsonProtocolDecodeError(
            "protocol message fields do not match the selected envelope"
        )


def _decode_failure(value):
    if not isinstance(value, dict):
        raise JsonProtocolDecodeError("protocol error must be an object")
    _require_exact_fields(value, ("code", "message", "details"))
    return ProtocolFailure(
        value.get("code"),
        value.get("message"),
        value.get("details"),
    )


def decode_message(frame):
    payload = _decode_json_frame(frame)
    version = payload.get("v")
    if type(version) is not int or version != JSON_PROTOCOL_VERSION:
        raise JsonProtocolDecodeError("protocol version is unsupported")
    message_type = payload.get("type")
    try:
        if message_type == "request":
            _require_exact_fields(payload, ("v", "type", "id", "cmd", "params"))
            return Request(payload.get("id"), payload.get("cmd"), payload.get("params"))
        if message_type == "response":
            status = payload.get("status")
            if type(status) is not str:
                raise JsonProtocolDecodeError(
                    "response status must be text"
                )
            expected = {"v", "type", "id", "cmd", "status"}
            if status in ("accepted", "completed"):
                expected.add("result")
                _require_exact_fields(payload, expected)
                return Response(
                    payload.get("id"),
                    payload.get("cmd"),
                    status,
                    result=payload.get("result"),
                )
            if status in _ERROR_RESPONSE_STATUSES:
                expected.add("error")
                _require_exact_fields(payload, expected)
                return Response(
                    payload.get("id"),
                    payload.get("cmd"),
                    status,
                    error=_decode_failure(payload.get("error")),
                )
            raise JsonProtocolDecodeError("response status is unsupported")
        if message_type == "event":
            _require_exact_fields(payload, ("v", "type", "seq", "event", "data"))
            return Event(payload.get("seq"), payload.get("event"), payload.get("data"))
        if message_type == "telemetry":
            _require_exact_fields(payload, ("v", "type", "seq", "stream", "data"))
            return Telemetry(
                payload.get("seq"),
                payload.get("stream"),
                payload.get("data"),
            )
        if message_type == "protocol_error":
            _require_exact_fields(payload, ("v", "type", "error"))
            return ProtocolErrorFrame(_decode_failure(payload.get("error")))
    except JsonProtocolDecodeError:
        raise
    except JsonProtocolValidationError as exc:
        raise JsonProtocolDecodeError(
            "protocol message violates the selected envelope"
        ) from exc
    raise JsonProtocolDecodeError("protocol message type is unsupported")
