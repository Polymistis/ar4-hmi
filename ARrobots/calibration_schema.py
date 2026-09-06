from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import json
import math
from numbers import Integral, Real
import re

from ARrobots.HMI.joint_motion import (
    AUXILIARY_BOARD_MEGA,
    AUXILIARY_BOARD_NANO,
    AUXILIARY_BOARD_NONE,
    AUXILIARY_BOARD_OUTPUT_PINS,
    AUXILIARY_SERVO_MAXIMUM_POSITION,
    AUXILIARY_SERVO_MINIMUM_POSITION,
    ControllerJointCalibration,
    MotionInputError,
    controller_degree_to_native_radians,
    controller_number,
    controller_ratio,
    validate_controller_encoder_scale,
)


class CalibrationSchemaError(ValueError):
    pass


@dataclass(frozen=True)
class _NumericRule:
    integer: bool = False
    minimum: float | None = None
    minimum_inclusive: bool = True
    maximum: float | None = None


_FINITE = _NumericRule()
_POSITIVE = _NumericRule(minimum=0.0, minimum_inclusive=False)
_NONNEGATIVE = _NumericRule(minimum=0.0)
_BINARY = _NumericRule(integer=True, minimum=0.0, maximum=1.0)
_POSITIVE_INTEGER = _NumericRule(
    integer=True,
    minimum=1.0,
    maximum=2147483647.0,
)
_NONNEGATIVE_INTEGER = _NumericRule(
    integer=True,
    minimum=0.0,
    maximum=2147483647.0,
)
_PERCENT = _NumericRule(minimum=0.0, maximum=100.0)
_VISION_ADJUSTMENT = _NumericRule(minimum=-127.0, maximum=127.0)
_VISION_ZOOM = _NumericRule(minimum=1.0, maximum=50.0)
_PLAIN_DECIMAL = re.compile(r"-?(?:\d+(?:\.\d*)?|\.\d+)\Z")
_LEGACY_PARENTHESIZED_RGB = re.compile(
    r"\(\s*(\d{1,3})\s*,\s*(\d{1,3})\s*,\s*(\d{1,3})\s*\)\Z"
)
_LEGACY_BARE_RGB = re.compile(
    r"(\d{1,3})\s*,\s*(\d{1,3})\s*,\s*(\d{1,3})\Z"
)
_CONTROLLER_PORT_IDENTITY = re.compile(r"usb-v1:[0-9a-f]{64}\Z")

CALIBRATION_SWITCH_STATES = ("LOW", "HIGH")
LEGACY_CALIBRATION_SWITCH_STATE = "HIGH"
CALIBRATION_SWITCH_KEYS = tuple(
    f"J{axis}CalSwitch"
    for axis in range(1, 10)
)
_SERVO_POSITION_KEYS = frozenset(
    f"Servo{channel}{state}"
    for channel in range(4)
    for state in ("off", "on")
)
_DIGITAL_OUTPUT_KEYS = frozenset(
    f"DO{output}{state}"
    for output in range(1, 7)
    for state in ("off", "on")
)
_REQUIRED_AUXILIARY_KEYS = frozenset(
    (
        "DO1off",
        "DO1on",
        "DO2off",
        "DO2on",
        "Servo0off",
        "Servo0on",
        "Servo1off",
        "Servo1on",
    )
)
_GENERAL_RUNTIME_TEXT_KEYS = frozenset(
    (
        "Prog",
        "VisFileLoc",
        "VisProg",
        "curCam",
        "setColor",
        "EOATVisual",
    )
)
_VISION_MAPPING_SENTINELS = {
    "VisOrigXpix": "VisPicOxPEntryField",
    "VisOrigXmm": "VisPicOxMEntryField",
    "VisOrigYpix": "VisPicOyPEntryField",
    "VisOrigYmm": "VisPicOyMEntryField",
    "VisEndXpix": "VisPicXPEntryField",
    "VisEndXmm": "VisPicXMEntryField",
    "VisEndYpix": "VisPicYPEntryField",
    "VisEndYmm": "VisPicYMEntryField",
}
_WRIST_CONFIGURATIONS = frozenset(("N", "F"))
_AUXILIARY_BOARD_PROFILES = frozenset(
    (
        AUXILIARY_BOARD_NONE,
        AUXILIARY_BOARD_NANO,
        AUXILIARY_BOARD_MEGA,
    )
)
_POSITION_TEXT_KEYS = frozenset(
    (
        "J1AngCur",
        "J2AngCur",
        "J3AngCur",
        "J4AngCur",
        "J5AngCur",
        "J6AngCur",
        "J7PosCur",
        "J8PosCur",
        "J9PosCur",
        "XcurPos",
        "YcurPos",
        "ZcurPos",
        "RxcurPos",
        "RycurPos",
        "RzcurPos",
    )
)
_ROTATIONAL_POSITION_KEYS = frozenset(
    ("RxcurPos", "RycurPos", "RzcurPos")
)


def _controller_numeric_schema():
    schema = {
        key: _FINITE
        for key in ("TFx", "TFy", "TFz", "TFrz", "TFry", "TFrx")
    }
    for axis in range(1, 10):
        schema[f"J{axis}MotDir"] = _BINARY
        schema[f"J{axis}CalDir"] = _BINARY
        schema[f"J{axis}calOff"] = _FINITE
        schema[f"J{axis}CalStatVal"] = _BINARY
        schema[f"J{axis}CalStatVal2"] = _BINARY
    for axis in range(1, 7):
        schema[f"J{axis}PosLim"] = _NONNEGATIVE
        schema[f"J{axis}NegLim"] = _NONNEGATIVE
        schema[f"J{axis}StepDeg"] = _POSITIVE
        schema[f"J{axis}DriveMS"] = _POSITIVE_INTEGER
        schema[f"J{axis}EncCPR"] = _POSITIVE_INTEGER
        schema[f"J{axis}OpenLoopVal"] = _BINARY
        for suffix in ("ΘDHpar", "αDHpar", "dDHpar", "aDHpar"):
            schema[f"J{axis}{suffix}"] = _FINITE
    for key in ("J7PosLim", "J8length", "J9length"):
        schema[key] = _NONNEGATIVE
    for key in (
        "J7rotation",
        "J7steps",
        "J8rotation",
        "J8steps",
        "J9rotation",
        "J9steps",
    ):
        schema[key] = _POSITIVE
    schema["J7StepCur"] = _NONNEGATIVE
    return schema


def _runtime_numeric_schema():
    schema = {}
    for key in (
        "autoBGVal",
        "curTheme",
        "DisableWristRotVal",
        "fullRotVal",
        "pick180Val",
        "pickClosestVal",
    ):
        schema[key] = _BINARY
    for key in ("mX1val", "mY1val", "mX2val", "mY2val"):
        schema[key] = _NONNEGATIVE_INTEGER
    for key in ("VisX1Val", "VisY1Val", "VisX2Val", "VisY2Val"):
        schema[key] = _NONNEGATIVE_INTEGER
    for key in (
        "VisRobX1Val",
        "VisRobY1Val",
        "VisRobX2Val",
        "VisRobY2Val",
    ):
        schema[key] = _FINITE
    for key in ("VisBrightVal", "VisContVal"):
        schema[key] = _VISION_ADJUSTMENT
    schema["VisScore"] = _PERCENT
    schema["zoom"] = _VISION_ZOOM
    for element in range(1, 7):
        schema[f"GC_ST_E{element}"] = _FINITE
        schema[f"GC_SToff_E{element}"] = _FINITE
    return schema


_NUMERIC_SCHEMA = {
    **_controller_numeric_schema(),
    **_runtime_numeric_schema(),
}
_REQUIRED_RUNTIME_KEYS = frozenset(
    tuple(_NUMERIC_SCHEMA)
    + tuple(_POSITION_TEXT_KEYS)
    + CALIBRATION_SWITCH_KEYS
    + tuple(_REQUIRED_AUXILIARY_KEYS)
    + tuple(_GENERAL_RUNTIME_TEXT_KEYS)
    + tuple(_VISION_MAPPING_SENTINELS)
    + (
        "GC_ST_WC",
        "VisBacColor",
        "comPort",
        "com2Port",
        "mainControllerPortIdentity",
        "auxiliaryControllerPortIdentity",
        "auxiliaryBoard",
    )
)
_KNOWN_KEYS = frozenset(
    tuple(_NUMERIC_SCHEMA)
    + tuple(_POSITION_TEXT_KEYS)
    + CALIBRATION_SWITCH_KEYS
    + tuple(_SERVO_POSITION_KEYS)
    + tuple(_DIGITAL_OUTPUT_KEYS)
    + tuple(_GENERAL_RUNTIME_TEXT_KEYS)
    + tuple(_VISION_MAPPING_SENTINELS)
    + (
        "GC_ST_WC",
        "VisBacColor",
        "comPort",
        "com2Port",
        "mainControllerPortIdentity",
        "auxiliaryControllerPortIdentity",
        "auxiliaryBoard",
    )
)
_CONTROLLER_NUMBER_KEYS = frozenset(
    (
        "TFx",
        "TFy",
        "TFz",
        "J7PosLim",
        "J7rotation",
        "J7steps",
        "J7StepCur",
        "J8length",
        "J8rotation",
        "J8steps",
        "J9length",
        "J9rotation",
        "J9steps",
        "VisRobX1Val",
        "VisRobY1Val",
        "VisRobX2Val",
        "VisRobY2Val",
    )
    + tuple(f"J{axis}calOff" for axis in range(1, 10))
    + tuple(
        f"J{axis}{suffix}"
        for axis in range(1, 7)
        for suffix in ("PosLim", "NegLim", "StepDeg", "dDHpar", "aDHpar")
    )
    + tuple(f"GC_ST_E{element}" for element in range(1, 4))
    + tuple(f"GC_SToff_E{element}" for element in range(1, 4))
)
_CONTROLLER_ANGLE_KEYS = frozenset(
    ("TFrx", "TFry", "TFrz")
    + tuple(
        f"J{axis}{suffix}"
        for axis in range(1, 7)
        for suffix in ("ΘDHpar", "αDHpar")
    )
    + tuple(f"GC_ST_E{element}" for element in range(4, 7))
    + tuple(f"GC_SToff_E{element}" for element in range(4, 7))
)


def _normalize_numeric(key, value, rule):
    if isinstance(value, bool):
        raise CalibrationSchemaError(f"{key} must be numeric, not boolean")
    try:
        if isinstance(value, Decimal):
            exact_number = value
        elif isinstance(value, Integral):
            exact_number = Decimal(int(value))
        elif isinstance(value, (Real, str)):
            token = str(value).strip()
            if not token:
                raise InvalidOperation
            exact_number = Decimal(token)
        else:
            raise TypeError
    except (InvalidOperation, TypeError, ValueError, OverflowError) as exc:
        raise CalibrationSchemaError(f"{key} must be numeric") from exc
    if not exact_number.is_finite():
        raise CalibrationSchemaError(f"{key} must be finite")
    if (
        rule.integer
        and exact_number != exact_number.to_integral_value()
    ):
        raise CalibrationSchemaError(f"{key} must be an integer")
    minimum = (
        Decimal(str(rule.minimum))
        if rule.minimum is not None
        else None
    )
    if minimum is not None and (
        exact_number < minimum
        or (
            exact_number == minimum
            and not rule.minimum_inclusive
        )
    ):
        comparison = "greater than" if not rule.minimum_inclusive else "at least"
        raise CalibrationSchemaError(
            f"{key} must be {comparison} {rule.minimum:.15g}"
        )
    maximum = (
        Decimal(str(rule.maximum))
        if rule.maximum is not None
        else None
    )
    if maximum is not None and exact_number > maximum:
        raise CalibrationSchemaError(
            f"{key} must be at most {rule.maximum:.15g}"
        )
    if rule.integer:
        return int(exact_number)
    try:
        number = float(exact_number)
    except (OverflowError, ValueError) as exc:
        raise CalibrationSchemaError(
            f"{key} is outside the supported numeric range"
        ) from exc
    if (
        not math.isfinite(number)
        or (exact_number != 0 and number == 0.0)
    ):
        raise CalibrationSchemaError(
            f"{key} is outside the supported numeric range"
        )
    return number


def _controller_contract(callback, *args):
    try:
        return callback(*args)
    except MotionInputError as exc:
        raise CalibrationSchemaError(str(exc)) from exc


def _normalize_position_text(key, value):
    if isinstance(value, bool):
        raise CalibrationSchemaError(f"{key} must be numeric, not boolean")
    if isinstance(value, str):
        token = value.strip()
        if not _PLAIN_DECIMAL.fullmatch(token):
            raise CalibrationSchemaError(
                f"{key} must use plain-decimal command syntax"
            )
        number = _controller_contract(controller_number, token, key)
        if key in _ROTATIONAL_POSITION_KEYS:
            _controller_contract(
                controller_degree_to_native_radians,
                number,
                key,
            )
        return token
    number = _controller_contract(controller_number, value, key)
    if key in _ROTATIONAL_POSITION_KEYS:
        _controller_contract(
            controller_degree_to_native_radians,
            number,
            key,
        )
    text = format(Decimal(str(number)), "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def _normalize_switch_state(key, value):
    if not isinstance(value, str):
        raise CalibrationSchemaError(f"{key} must be HIGH or LOW")
    state = value.strip().upper()
    if state not in CALIBRATION_SWITCH_STATES:
        raise CalibrationSchemaError(f"{key} must be HIGH or LOW")
    return state


def _validate_text_field(key, value):
    if not isinstance(value, str):
        raise CalibrationSchemaError(f"{key} must be text")
    if any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise CalibrationSchemaError(f"{key} contains control characters")
    return value


def _validate_controller_port_identity(key, value):
    if not isinstance(value, str) or (
        value != "None"
        and not _CONTROLLER_PORT_IDENTITY.fullmatch(value)
    ):
        raise CalibrationSchemaError(
            f"{key} must be None or a canonical usb-v1 identity"
        )
    return value


def _normalize_optional_integer_text(key, value, minimum, maximum):
    if value is None or value == "":
        return ""
    if isinstance(value, bool):
        raise CalibrationSchemaError(f"{key} must be an integer or empty")
    if isinstance(value, str):
        token = value.strip()
        if not token:
            return ""
        if not re.fullmatch(r"\d+", token):
            raise CalibrationSchemaError(f"{key} must be an integer or empty")
        number = int(token)
    elif isinstance(value, Integral):
        number = int(value)
    else:
        raise CalibrationSchemaError(f"{key} must be an integer or empty")
    if number < minimum or number > maximum:
        raise CalibrationSchemaError(
            f"{key} must be between {minimum} and {maximum}"
        )
    return str(number)


def normalize_vision_background_color(value):
    """Return a canonical [red, green, blue] byte list."""
    if isinstance(value, str):
        value = _validate_text_field("VisBacColor", value).strip()
        legacy_match = (
            _LEGACY_PARENTHESIZED_RGB.fullmatch(value)
            or _LEGACY_BARE_RGB.fullmatch(value)
        )
        if legacy_match is not None:
            components = tuple(int(component) for component in legacy_match.groups())
        else:
            try:
                components = json.loads(value)
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                raise CalibrationSchemaError(
                    "VisBacColor must contain three RGB integers"
                ) from exc
    else:
        components = value
    if (
        not isinstance(components, (list, tuple))
        or len(components) != 3
    ):
        raise CalibrationSchemaError(
            "VisBacColor must contain three RGB integers"
        )
    normalized = []
    for component in components:
        if isinstance(component, bool) or not isinstance(component, Integral):
            raise CalibrationSchemaError(
                "VisBacColor components must be integers"
            )
        component = int(component)
        if component < 0 or component > 255:
            raise CalibrationSchemaError(
                "VisBacColor components must be between 0 and 255"
            )
        normalized.append(component)
    return normalized


def _infer_legacy_auxiliary_board(calibration_data):
    candidate_boards = set()
    for key in sorted(_DIGITAL_OUTPUT_KEYS):
        if key not in calibration_data:
            continue
        value = calibration_data[key]
        if value is None or (isinstance(value, str) and not value.strip()):
            continue
        normalized = _normalize_optional_integer_text(
            key,
            value,
            0,
            2147483647,
        )
        pin = int(normalized)
        matching_boards = {
            board
            for board, valid_pins in AUXILIARY_BOARD_OUTPUT_PINS.items()
            if pin in valid_pins
        }
        if not matching_boards:
            raise CalibrationSchemaError(
                f"{key} cannot identify a Nano or Mega auxiliary board; "
                "select the board profile before migration"
            )
        candidate_boards.update(matching_boards)
    if len(candidate_boards) > 1:
        raise CalibrationSchemaError(
            "saved digital outputs span Nano and Mega pin ranges; "
            "select the board profile before migration"
        )
    if candidate_boards:
        return candidate_boards.pop()
    return None


def reconcile_auxiliary_output_assignments(calibration_data, board_profile):
    if not isinstance(calibration_data, dict):
        raise CalibrationSchemaError("calibration data must be a JSON object")
    if (
        not isinstance(board_profile, str)
        or board_profile not in _AUXILIARY_BOARD_PROFILES
    ):
        raise CalibrationSchemaError(
            "auxiliaryBoard must be None, Nano, or Mega"
        )

    reconciled = dict(calibration_data)
    valid_pins = AUXILIARY_BOARD_OUTPUT_PINS.get(board_profile, frozenset())
    for key in sorted(_DIGITAL_OUTPUT_KEYS):
        if key not in reconciled:
            continue
        value = reconciled[key]
        if value is None or (isinstance(value, str) and not value.strip()):
            reconciled[key] = ""
            continue
        normalized = _normalize_optional_integer_text(
            key,
            value,
            0,
            2147483647,
        )
        reconciled[key] = (
            normalized
            if int(normalized) in valid_pins
            else ""
        )
    reconciled["auxiliaryBoard"] = board_profile
    return reconciled


def _axis_calibration(axis, negative, positive, scale):
    negative_limits = [0.0] * 9
    positive_limits = [1.0] * 9
    steps_per_unit = [1.0] * 9
    negative_limits[axis - 1] = negative
    positive_limits[axis - 1] = positive
    steps_per_unit[axis - 1] = scale
    return _controller_contract(
        ControllerJointCalibration,
        tuple(negative_limits),
        tuple(positive_limits),
        tuple(steps_per_unit),
    )


def _validate_controller_fields(normalized):
    for key in _CONTROLLER_NUMBER_KEYS:
        if key in normalized:
            _controller_contract(controller_number, normalized[key], key)
    for key in _CONTROLLER_ANGLE_KEYS:
        if key in normalized:
            _controller_contract(
                controller_degree_to_native_radians,
                normalized[key],
                key,
            )
    for axis in range(1, 7):
        drive_key = f"J{axis}DriveMS"
        encoder_key = f"J{axis}EncCPR"
        encoder_multiplier = None
        if drive_key in normalized and encoder_key in normalized:
            encoder_multiplier = _controller_contract(
                controller_ratio,
                normalized[encoder_key],
                normalized[drive_key],
                f"J{axis} encoder multiplier",
            )

        keys = (
            f"J{axis}NegLim",
            f"J{axis}PosLim",
            f"J{axis}StepDeg",
        )
        position_key = f"J{axis}AngCur"
        if position_key in normalized and not all(
            key in normalized for key in keys
        ):
            raise CalibrationSchemaError(
                f"{position_key} requires complete J{axis} calibration"
            )
        if not all(key in normalized for key in keys):
            continue
        negative = normalized[keys[0]]
        positive = normalized[keys[1]]
        if negative + positive <= 0.0:
            raise CalibrationSchemaError(
                f"J{axis} configured travel must be positive"
            )
        calibration = _axis_calibration(
            axis,
            negative,
            positive,
            normalized[keys[2]],
        )
        if encoder_multiplier is not None:
            _controller_contract(
                validate_controller_encoder_scale,
                calibration,
                axis,
                encoder_multiplier,
                f"J{axis} encoder multiplier",
            )
        if position_key in normalized:
            _controller_contract(
                calibration.validate_current_axis_positions,
                {axis: normalized[position_key]},
            )

    for axis, length_key in (
        (7, "J7PosLim"),
        (8, "J8length"),
        (9, "J9length"),
    ):
        keys = (length_key, f"J{axis}rotation", f"J{axis}steps")
        position_key = f"J{axis}PosCur"
        if position_key in normalized and not all(
            key in normalized for key in keys
        ):
            raise CalibrationSchemaError(
                f"{position_key} requires complete J{axis} calibration"
            )
        if not all(key in normalized for key in keys):
            continue
        scale = _controller_contract(
            controller_ratio,
            normalized[keys[2]],
            normalized[keys[1]],
            f"J{axis} steps per unit",
        )
        calibration = _axis_calibration(
            axis,
            0.0,
            normalized[length_key],
            scale,
        )
        if position_key in normalized:
            _controller_contract(
                calibration.validate_current_axis_positions,
                {axis: normalized[position_key]},
            )


def normalize_calibration_data(
    calibration_data,
    *,
    require_runtime_fields=True,
    migrate_legacy_switches=True,
    migrate_legacy_auxiliary_board=True,
):
    if not isinstance(calibration_data, dict):
        raise CalibrationSchemaError("calibration data must be a JSON object")
    if not isinstance(require_runtime_fields, bool):
        raise TypeError("runtime-field validation flag must be boolean")
    if not isinstance(migrate_legacy_switches, bool):
        raise TypeError("switch migration flag must be boolean")
    if not isinstance(migrate_legacy_auxiliary_board, bool):
        raise TypeError("auxiliary-board migration flag must be boolean")
    if not all(isinstance(key, str) for key in calibration_data):
        raise CalibrationSchemaError("calibration keys must be text")
    unknown = sorted(set(calibration_data).difference(_KNOWN_KEYS))
    if unknown:
        raise CalibrationSchemaError(
            "calibration data contains unsupported fields: "
            + ", ".join(unknown)
        )

    normalized = dict(calibration_data)
    if migrate_legacy_switches:
        for key in CALIBRATION_SWITCH_KEYS:
            normalized.setdefault(key, LEGACY_CALIBRATION_SWITCH_STATE)
    if (
        migrate_legacy_auxiliary_board
        and "auxiliaryBoard" not in normalized
    ):
        inferred_board = _infer_legacy_auxiliary_board(normalized)
        if inferred_board is not None:
            normalized["auxiliaryBoard"] = inferred_board
    if require_runtime_fields:
        for key in (
            "comPort",
            "com2Port",
            "mainControllerPortIdentity",
            "auxiliaryControllerPortIdentity",
        ):
            if normalized.get(key) is None:
                normalized[key] = "None"
        normalized.setdefault("auxiliaryBoard", AUXILIARY_BOARD_NONE)
        normalized.setdefault("EOATVisual", "Servo Gripper")

        missing = sorted(_REQUIRED_RUNTIME_KEYS.difference(normalized))
        if missing:
            raise CalibrationSchemaError(
                "calibration data is missing required fields: "
                + ", ".join(missing)
            )

    for key, rule in _NUMERIC_SCHEMA.items():
        if key in normalized:
            normalized[key] = _normalize_numeric(key, normalized[key], rule)
    for key in _POSITION_TEXT_KEYS:
        if key in normalized:
            normalized[key] = _normalize_position_text(key, normalized[key])
    for key in CALIBRATION_SWITCH_KEYS:
        if key in normalized:
            normalized[key] = _normalize_switch_state(key, normalized[key])
    for key in ("comPort", "com2Port"):
        if key in normalized:
            normalized[key] = _validate_text_field(key, normalized[key])
    for key in (
        "mainControllerPortIdentity",
        "auxiliaryControllerPortIdentity",
    ):
        if key in normalized:
            normalized[key] = _validate_controller_port_identity(
                key,
                normalized[key],
            )
    for key in _GENERAL_RUNTIME_TEXT_KEYS:
        if key in normalized:
            normalized[key] = _validate_text_field(key, normalized[key])
    for key, sentinel in _VISION_MAPPING_SENTINELS.items():
        if key not in normalized:
            continue
        if normalized[key] == sentinel:
            continue
        normalized[key] = _normalize_numeric(key, normalized[key], _FINITE)
    if "GC_ST_WC" in normalized:
        wrist_configuration = _validate_text_field(
            "GC_ST_WC",
            normalized["GC_ST_WC"],
        ).strip().upper()
        if wrist_configuration not in _WRIST_CONFIGURATIONS:
            raise CalibrationSchemaError("GC_ST_WC must be N or F")
        normalized["GC_ST_WC"] = wrist_configuration
    if "auxiliaryBoard" in normalized:
        board = _validate_text_field(
            "auxiliaryBoard",
            normalized["auxiliaryBoard"],
        ).strip()
        if board not in _AUXILIARY_BOARD_PROFILES:
            raise CalibrationSchemaError(
                "auxiliaryBoard must be None, Nano, or Mega"
            )
        normalized["auxiliaryBoard"] = board
    else:
        board = None

    for key in _SERVO_POSITION_KEYS:
        if key in normalized:
            normalized[key] = _normalize_optional_integer_text(
                key,
                normalized[key],
                AUXILIARY_SERVO_MINIMUM_POSITION,
                AUXILIARY_SERVO_MAXIMUM_POSITION,
            )
    for key in _DIGITAL_OUTPUT_KEYS:
        if key not in normalized:
            continue
        value = normalized[key]
        if value is None or value == "":
            normalized[key] = ""
            continue
        if board not in AUXILIARY_BOARD_OUTPUT_PINS:
            raise CalibrationSchemaError(
                f"{key} requires a selected Nano or Mega auxiliary board"
            )
        valid_pins = AUXILIARY_BOARD_OUTPUT_PINS[board]
        normalized[key] = _normalize_optional_integer_text(
            key,
            value,
            min(valid_pins),
            max(valid_pins),
        )
        if int(normalized[key]) not in valid_pins:
            raise CalibrationSchemaError(
                f"{key} is not a valid {board} output pin"
            )
    if "VisBacColor" in normalized:
        normalized["VisBacColor"] = normalize_vision_background_color(
            normalized["VisBacColor"]
        )

    _validate_controller_fields(normalized)
    return normalized


def ar4_mk5_calibration_switch_profile():
    return normalize_calibration_data(
        {
            "J1CalSwitch": "LOW",
            "J2CalSwitch": "LOW",
            "J3CalSwitch": "LOW",
            "J4CalSwitch": "HIGH",
            "J5CalSwitch": "HIGH",
            "J6CalSwitch": "HIGH",
        },
        require_runtime_fields=False,
        migrate_legacy_switches=False,
        migrate_legacy_auxiliary_board=False,
    )
