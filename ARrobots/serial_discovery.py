"""Passive serial-port inventory and enrolled USB identity resolution."""

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import logging
import re
import unicodedata

from serial.tools import list_ports


logger = logging.getLogger(__name__)
MAIN_CONTROLLER_USB_ID = (0x16C0, 0x0483)
PORT_IDENTITY_NONE = "None"
_FIELDS = (
    "device", "vid", "pid", "serial_number", "location", "manufacturer",
    "product", "interface", "description", "hwid",
)
_PORT_IDENTITY_RE = re.compile(r"usb-v1:[0-9a-f]{64}\Z")


class SerialDiscoveryError(ValueError):
    pass


@dataclass(frozen=True)
class SerialPortRecord:
    device: str
    vid: int | None
    pid: int | None
    serial_number: str | None
    location: str | None
    manufacturer: str | None
    product: str | None
    interface: str | None
    description: str | None
    hwid: str | None

    def __post_init__(self):
        object.__setattr__(self, "device", _text(self.device, "device", required=True))
        for field_name in _FIELDS[3:]:
            value = _text(getattr(self, field_name), field_name)
            object.__setattr__(self, field_name, value)
        for field_name in ("vid", "pid"):
            value = getattr(self, field_name)
            invalid = value is not None and (
                isinstance(value, bool)
                or not isinstance(value, int)
                or not 0 <= value <= 0xFFFF
            )
            if invalid:
                raise SerialDiscoveryError(f"{field_name} must be a 16-bit integer or null")
        if (self.vid is None) != (self.pid is None):
            raise SerialDiscoveryError("vid and pid must both be present or both be null")


@dataclass(frozen=True)
class SerialPortInventory:
    records: tuple[SerialPortRecord, ...]
    conflicting_devices: tuple[str, ...]

    def __post_init__(self):
        records_valid = isinstance(self.records, tuple) and all(
            isinstance(record, SerialPortRecord) for record in self.records
        )
        if not records_valid or not isinstance(self.conflicting_devices, tuple):
            raise SerialDiscoveryError("inventory records must be a record tuple")
        canonical = tuple(sorted(set(self.records), key=_record_key))
        if self.records != canonical:
            raise SerialDiscoveryError("inventory records are not canonical")
        if self.conflicting_devices != _conflicting_devices(self.records):
            raise SerialDiscoveryError("conflicting devices do not match records")


@dataclass(frozen=True)
class SerialPortResolution:
    status: str
    record: SerialPortRecord | None

    def __post_init__(self):
        if self.status not in ("selected", "absent", "unenrolled", "ambiguous"):
            raise SerialDiscoveryError("serial resolution status is invalid")
        expected_type = SerialPortRecord if self.status == "selected" else type(None)
        if not isinstance(self.record, expected_type):
            raise SerialDiscoveryError("serial resolution record is invalid")


def _text(value, field_name, required=False):
    if value is None:
        if required:
            raise SerialDiscoveryError(f"{field_name} must be nonempty text")
        return None
    if not isinstance(value, str):
        raise SerialDiscoveryError(f"{field_name} must be text or null")
    categories = (unicodedata.category(character) for character in value)
    if any(category.startswith("C") or category in ("Zl", "Zp") for category in categories):
        raise SerialDiscoveryError(f"{field_name} contains an unsafe character")
    result = value.strip()
    if required and not result:
        raise SerialDiscoveryError(f"{field_name} must be nonempty text")
    if field_name == "device" and result == PORT_IDENTITY_NONE:
        raise SerialDiscoveryError("device uses the reserved None sentinel")
    return result or None


def _record(source, normalized_device=None):
    values = [getattr(source, "device") if normalized_device is None else normalized_device]
    values.extend(getattr(source, field_name, None) for field_name in _FIELDS[1:])
    return SerialPortRecord(*values)


def _natural_key(value):
    parts = re.split(r"(\d+)", value.casefold())
    return tuple((1, int(part)) if part.isdecimal() else (0, part) for part in parts)


def _record_key(record):
    metadata = (repr(getattr(record, field_name)) for field_name in _FIELDS[1:])
    return _natural_key(record.device), record.device, *metadata


def _conflicting_devices(records):
    counts = Counter(record.device for record in records)
    return tuple(device for device, count in counts.items() if count > 1)


def _clipped(value, limit):
    return value if not value or len(value) <= limit else value[:limit - 3] + "..."


def normalize_serial_ports(records):
    try:
        ordered = tuple(
            sorted({_record(record) for record in records}, key=_record_key)
        )
    except SerialDiscoveryError:
        raise
    except Exception as exc:
        raise SerialDiscoveryError("serial metadata snapshot is invalid") from exc
    return SerialPortInventory(ordered, _conflicting_devices(ordered))


def enumerate_serial_ports():
    records, tainted_devices = [], set()
    try:
        for index, source in enumerate(list_ports.comports()):
            device = None
            try:
                device = _text(getattr(source, "device"), "device", required=True)
                records.append(_record(source, device))
            except Exception:
                if device is not None:
                    tainted_devices.add(device)
                logger.warning("Ignoring invalid serial-port enumeration entry at index %s", index)
    except Exception as exc:
        raise SerialDiscoveryError("serial port enumeration failed") from exc
    return normalize_serial_ports(
        record for record in records if record.device not in tainted_devices)


def validate_port_identity(value):
    if not isinstance(value, str) or _PORT_IDENTITY_RE.fullmatch(value) is None:
        raise SerialDiscoveryError("port identity is not canonical usb-v1 SHA-256 text")
    return value


def port_identity(record):
    if not isinstance(record, SerialPortRecord):
        raise SerialDiscoveryError("port identity requires a normalized record")
    if record.vid is None or record.serial_number is None:
        return None
    values = ["usb-v1", f"{record.vid:04x}", f"{record.pid:04x}", record.serial_number]
    payload = json.dumps(
        values, ensure_ascii=True, separators=(",", ":")
    ).encode("utf-8")
    return "usb-v1:" + hashlib.sha256(payload).hexdigest()


def _inventory(value):
    if not isinstance(value, SerialPortInventory):
        raise SerialDiscoveryError("serial inventory is invalid")
    return value


def resolve_selected_device(inventory, raw_device):
    inventory = _inventory(inventory)
    if raw_device == PORT_IDENTITY_NONE:
        return SerialPortResolution("absent", None)
    device = _text(raw_device, "selected device", required=True)
    matches = tuple(record for record in inventory.records if record.device == device)
    if not matches:
        return SerialPortResolution("absent", None)
    if device in inventory.conflicting_devices or len(matches) != 1:
        return SerialPortResolution("ambiguous", None)
    return SerialPortResolution("selected", matches[0])


def _resolve_identity(inventory, stored_identity, main_controller):
    inventory = _inventory(inventory)
    if stored_identity == PORT_IDENTITY_NONE:
        return SerialPortResolution("unenrolled", None)
    identity = validate_port_identity(stored_identity)
    matches = tuple(
        record
        for record in inventory.records
        if port_identity(record) == identity
    )
    if not matches:
        return SerialPortResolution("absent", None)
    if len(matches) != 1 or matches[0].device in inventory.conflicting_devices:
        return SerialPortResolution("ambiguous", None)
    record = matches[0]
    if main_controller and (record.vid, record.pid) != MAIN_CONTROLLER_USB_ID:
        return SerialPortResolution("unenrolled", None)
    return SerialPortResolution("selected", record)


def resolve_main_identity(inventory, stored_identity):
    return _resolve_identity(inventory, stored_identity, main_controller=True)


def resolve_auxiliary_identity(inventory, stored_identity):
    return _resolve_identity(inventory, stored_identity, main_controller=False)


def selector_entries(inventory):
    inventory = _inventory(inventory)
    entries = []
    records_by_device = {record.device: record for record in inventory.records}
    for device in records_by_device:
        display_device = _clipped(device, 32)
        if device in inventory.conflicting_devices:
            label = f"{display_device} - ambiguous metadata"
        else:
            record = records_by_device[device]
            descriptor = _clipped(
                record.product or record.description or record.manufacturer,
                32,
            )
            serial_number = _clipped(record.serial_number, 32)
            details = [
                "main candidate; role unverified"
                if (record.vid, record.pid) == MAIN_CONTROLLER_USB_ID else None,
                f"USB serial {serial_number}"
                if serial_number else "unverified adapter; manual only",
                f"USB {record.vid:04X}:{record.pid:04X}"
                if record.vid is not None else None,
                descriptor,
                f"location {record.location}" if record.location else None,
            ]
            details = (detail for detail in details if detail)
            label = " - ".join([display_device, *details])
        entries.append((_clipped(label, 160), device))
    return tuple(entries)
