"""Canonical command inventory for the JSON-only controller protocol."""

from dataclasses import dataclass
import re


MAIN_CONTROLLER = "main_controller"
AUXILIARY_CONTROLLER = "auxiliary_controller"

_NAME_PATTERN = re.compile(r"[a-z][a-z0-9_]{0,63}\Z")
_DEVICES = frozenset((MAIN_CONTROLLER, AUXILIARY_CONTROLLER))


@dataclass(frozen=True)
class CommandDefinition:
    """One active semantic command owned by exactly one controller."""

    name: str
    device: str
    domain: str

    def __post_init__(self):
        for value, field_name in (
            (self.name, "command name"),
            (self.domain, "command domain"),
        ):
            if type(value) is not str or _NAME_PATTERN.fullmatch(value) is None:
                raise ValueError(f"{field_name} is invalid")
        if self.device not in _DEVICES:
            raise ValueError("command device is invalid")


def _main(name, domain):
    return CommandDefinition(name, MAIN_CONTROLLER, domain)


def _auxiliary(name, domain):
    return CommandDefinition(name, AUXILIARY_CONTROLLER, domain)


MAIN_CONTROLLER_COMMANDS = (
    _main("hello", "session"),
    _main("get_home_reference", "state"),
    _main("get_position_disposition", "state"),
    _main("correct_position", "state"),
    _main("set_position", "state"),
    _main("test_limit_switches", "state"),
    _main("set_encoders", "state"),
    _main("read_encoders", "state"),
    _main("get_motion_trace", "diagnostics"),
    _main("update_params", "configuration"),
    _main("config_ext_axis", "configuration"),
    _main("zero_j7", "configuration"),
    _main("zero_j8", "configuration"),
    _main("zero_j9", "configuration"),
    _main("controller_wait", "program"),
    _main("calibrate", "calibration"),
    _main("move_joints", "motion"),
    _main("move_cartesian", "motion"),
    _main("move_linear", "motion"),
    _main("move_vision", "motion"),
    _main("jog_tool", "motion"),
    _main("live_joint_jog", "motion"),
    _main("live_cart_jog", "motion"),
    _main("live_tool_jog", "motion"),
    _main("stop", "motion"),
    _main("renew_live_motion", "motion"),
    _main("modbus_read_holding_register", "modbus"),
    _main("modbus_read_coil", "modbus"),
    _main("modbus_read_discrete_input", "modbus"),
    _main("modbus_read_input_register", "modbus"),
    _main("modbus_write_coil", "modbus"),
    _main("modbus_write_register", "modbus"),
    _main("wait_modbus_coil", "modbus"),
    _main("wait_modbus_discrete_input", "modbus"),
    _main("delete_sd_program", "storage"),
    _main("list_sd_programs", "storage"),
    _main("write_gcode_move", "storage"),
    _main("play_gcode_file", "motion"),
    _main("wait_modbus_holding_register", "modbus"),
    _main("move_arc", "motion"),
    _main("move_circle", "motion"),
    _main("move_spline", "motion"),
)


AUXILIARY_COMMANDS = (
    _auxiliary("hello", "session"),
    _auxiliary("servo", "output"),
    _auxiliary("input_read", "input"),
    _auxiliary("set_output", "output"),
    _auxiliary("wait_input", "input"),
    _auxiliary("test_gripper_amps", "input"),
    _auxiliary("stop", "input"),
    _auxiliary("gripper_detach", "output"),
)


COMMANDS = MAIN_CONTROLLER_COMMANDS + AUXILIARY_COMMANDS


def commands_for_device(device):
    if device == MAIN_CONTROLLER:
        return MAIN_CONTROLLER_COMMANDS
    if device == AUXILIARY_CONTROLLER:
        return AUXILIARY_COMMANDS
    raise ValueError("unknown command device")


__all__ = (
    "AUXILIARY_COMMANDS",
    "AUXILIARY_CONTROLLER",
    "COMMANDS",
    "CommandDefinition",
    "MAIN_CONTROLLER",
    "MAIN_CONTROLLER_COMMANDS",
    "commands_for_device",
)
