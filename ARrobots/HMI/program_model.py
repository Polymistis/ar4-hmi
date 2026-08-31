"""Import-safe validation helpers for AR4 robot program rows."""

import re

from ARrobots.HMI.joint_motion import MotionInputError


PROGRAM_ROW_COMMAND_PREFIXES = frozenset({
    "Call P", "Run Gc", "Stop P", "Test L", "Test G", "Set En",
    "Read E", "Servo ", "If Inp", "Read C", "If Reg", "If COM",
    "If MBc", "If MBi", "If MBh", "If MBI", "Jump T", "Wait T",
    "Regist", "Positi", "Calibr", "Cal_J1", "Cal_J2", "Cal_J3",
    "Cal_J4", "Cal_J5", "Cal_J6", "Cal_J7", "Cal_J8", "Cal_J9",
    "Tool S", "Move J", "OFF J ", "Move V", "Move P", "OFF PR",
    "Move L", "Move R", "Move A", "Move C", "Start ", "End Sp",
    "Cam On", "Cam Of", "Vis Fi",
})
PROGRAM_ROW_LONG_COMMAND_PREFIXES = frozenset({
    "Wait 5v Inp", "Wait MBcoil", "Wait MBhold", "Wait MBinpu", "Set 5v Outp",
    "Set MBcoil ", "Set MBoutpu",
})
_VIRTUAL_SCENE_ROW_PATTERN = re.compile(r"Virtual (Pick|Place) - ([0-9a-f]{32})\Z")


def decode_program_row_content(row):
    if isinstance(row, (bytes, bytearray)):
        try:
            text = bytes(row).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise MotionInputError("program rows must contain UTF-8 text") from exc
    elif isinstance(row, str):
        text = row
    else:
        raise MotionInputError("program rows must contain encoded or text rows")
    if text.endswith("\r\n"):
        text = text[:-2]
    elif text.endswith("\n"):
        text = text[:-1]
    if "\r" in text or "\n" in text:
        raise MotionInputError("program rows must contain one logical line")
    return text


def program_row_index(rows, expected_row):
    if isinstance(rows, (str, bytes, bytearray)) or not isinstance(
        rows, (tuple, list)
    ):
        raise MotionInputError("program rows must be a sequence")
    if not isinstance(expected_row, str) or any(
        separator in expected_row for separator in ("\r", "\n")
    ):
        raise MotionInputError("program row target must be one text line")
    for index, row in enumerate(rows):
        if decode_program_row_content(row) == expected_row:
            return index
    raise MotionInputError(f"program row {expected_row!r} does not exist")


def normalize_program_tab_number(tab_number):
    if not isinstance(tab_number, str):
        raise MotionInputError("program tab number must be text")
    normalized = tab_number.strip()
    if not normalized or not normalized.isdecimal():
        raise MotionInputError("program tab number must contain decimal digits")
    return normalized


def program_tab_row_index(rows, tab_number):
    normalized = normalize_program_tab_number(tab_number)
    return program_row_index(rows, f"Tab Number {normalized}")


def program_bounded_index(value, label, maximum):
    if isinstance(maximum, bool) or not isinstance(maximum, int) or maximum <= 0:
        raise RuntimeError(f"{label} maximum is invalid")
    if not isinstance(value, str) or re.fullmatch(r"[1-9][0-9]*", value) is None:
        raise MotionInputError(f"{label} must be a positive integer")
    maximum_text = str(maximum)
    if len(value) > len(maximum_text) or (
        len(value) == len(maximum_text) and value > maximum_text
    ):
        raise MotionInputError(f"{label} must be between 1 and {maximum}")
    return int(value)


def parse_virtual_scene_row(command):
    if not isinstance(command, str):
        raise MotionInputError("virtual scene program row must be text")
    match = _VIRTUAL_SCENE_ROW_PATTERN.fullmatch(command)
    if match is None:
        raise MotionInputError("virtual scene row must contain a canonical action and object ID")
    return match.group(1), match.group(2)


def format_virtual_scene_row(action, object_id):
    if not isinstance(action, str) or action not in ("Pick", "Place"):
        raise MotionInputError("virtual scene action must be Pick or Place")
    if not isinstance(object_id, str) or re.fullmatch(r"[0-9a-f]{32}", object_id) is None:
        raise MotionInputError("virtual scene object ID must be 32 lowercase hex characters")
    return f"Virtual {action} - {object_id}"


def program_row_is_supported(command):
    if not isinstance(command, str):
        raise TypeError("robot program command must be text")
    if not command or command.startswith("##"):
        return True
    if command.startswith("Tab Number "):
        tab_number = command[len("Tab Number "):]
        try:
            normalized = normalize_program_tab_number(tab_number)
        except MotionInputError:
            return False
        return tab_number == normalized
    if command.startswith("Virtual "):
        return _VIRTUAL_SCENE_ROW_PATTERN.fullmatch(command) is not None
    return (
        command[:6] in PROGRAM_ROW_COMMAND_PREFIXES
        or command[:11] in PROGRAM_ROW_LONG_COMMAND_PREFIXES
    )


def program_row_error_detail(error):
    try:
        detail = " ".join(str(error).split())
    except Exception:
        return type(error).__name__
    return detail or type(error).__name__
