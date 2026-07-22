import ast
import copy
from contextlib import contextmanager
from dataclasses import dataclass
from functools import wraps
import io
import json
import math
import os
from pathlib import Path
from queue import Empty, Queue
import re
import tempfile
import threading
import time
from types import SimpleNamespace
from typing import Optional
import unittest

import numpy as np

from ARrobots.HMI.joint_motion import (
    AUXILIARY_BOARD_MEGA,
    AUXILIARY_BOARD_NANO,
    AUXILIARY_BOARD_NONE,
    AUXILIARY_BOARD_OUTPUT_PINS,
    CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1,
    ControllerIdentity,
    ControllerJointCalibration,
    MotionInputError,
    LiveMotionScheduleResult,
    MAX_COMMAND_LENGTH,
    MAX_CONTROLLER_FILENAME_BYTES,
    MAX_RESPONSE_FRAME_LENGTH,
    MAX_RESPONSE_PAYLOAD_LENGTH,
    MotionProfile,
    MotionQueueFault,
    MotionRequestLease,
    MotionRequestRegistry,
    MotionTransportBusy,
    PositionResponse,
    ProtocolResponseError,
    SerialActivityRegistry,
    SerialActivityRejected,
    SerialTransportTimeout,
    VirtualMotionOperation,
    auxiliary_pneumatic_output_pin,
    build_virtual_joint_command,
    canonicalize_serial_command,
    canonicalize_virtual_command,
    command_response_timeout,
    controller_degree_to_native_radians,
    controller_number,
    controller_protocol_decimal,
    controller_ratio,
    decode_serial_response_line,
    exchange_serial_line,
    exchange_serial_line_until_cancelled,
    finite_number,
    motion_timing_response_timeout,
    normalize_auxiliary_board_profile,
    parse_command_timing,
    parse_controller_identity_response,
    parse_controller_modbus_response,
    parse_motion_wrist_config,
    parse_position_response,
    parse_virtual_command_timing,
    quarantine_serial_transport,
    read_serial_exact_response,
    read_serial_line_response,
    read_serial_line_response_with_optional_followup,
    serial_transport_quarantined,
    validate_auxiliary_output_command,
    validate_controller_filename,
    write_serial_control,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
AR4_SOURCE = PROJECT_ROOT / "AR4.py"
CALIBRATION_SOURCE = PROJECT_ROOT / "ARrobots" / "Calibration.py"
NATIVE_KINEMATICS_SOURCE = PROJECT_ROOT / "ARrobots" / "src" / "kinematics.cpp"
NATIVE_WINDOWS_BUILD_SOURCE = (
    PROJECT_ROOT / "ARrobots" / "src" / "build_kinematics.ps1"
)
TEENSY_SOURCE = (
    PROJECT_ROOT
    / "ArduinoSketches"
    / "AR4_teensy41_sketch_v6.7.1"
    / "AR4_teensy41_sketch_v6.7.1.ino"
)
TEENSY_ANGLE_CONVERSION_CONTRACT = TEENSY_SOURCE.with_name(
    "angle_conversion_contract.h"
)
TEENSY_CARTESIAN_POSE_CONTRACT = TEENSY_SOURCE.with_name(
    "cartesian_pose_contract.h"
)
TEENSY_CONTROLLER_DOMAIN_CONTRACT = TEENSY_SOURCE.with_name(
    "controller_domain_contract.h"
)
TEENSY_IDENTITY_CONTRACT = TEENSY_SOURCE.with_name("identity_contract.h")
TEENSY_NUMERIC_PARSE_CONTRACT = TEENSY_SOURCE.with_name(
    "numeric_parse_contract.h"
)
TEENSY_MOTION_COMMAND_PARSE_CONTRACT = TEENSY_SOURCE.with_name(
    "motion_command_parse_contract.h"
)
TEENSY_MOTION_MODE_TRANSACTION = TEENSY_SOURCE.with_name(
    "motion_mode_transaction.h"
)
TEENSY_QUEUE_CONTRACT = TEENSY_SOURCE.with_name("command_queue_contract.h")
TEENSY_DEBUG_CONTRACT = TEENSY_SOURCE.with_name("debug_contract.h")
TEENSY_PERSISTENCE_CONTRACT = TEENSY_SOURCE.with_name("persistence_contract.h")
TEENSY_SERIAL_FRAME_CONTRACT = TEENSY_SOURCE.with_name(
    "serial_frame_contract.h"
)
TEENSY_SPLINE_RESPONSE_CONTRACT = TEENSY_SOURCE.with_name(
    "spline_response_contract.h"
)
TEENSY_TOOL_JOG_CONTRACT = TEENSY_SOURCE.with_name("tool_jog_contract.h")
TEENSY_WRIST_SELECTION_CONTRACT = TEENSY_SOURCE.with_name(
    "wrist_selection_contract.h"
)
VIRTUAL_CARTESIAN_TEST_COMMAND = (
    "MJX1Y2Z3Rz4Ry5Rx6Sp50Ac10Dc20Rm25WNLm000000\n"
)
CONTROLLER_CARTESIAN_TEST_COMMAND = (
    "MJX1Y2Z3Rz4Ry5Rx6J77J88J99Sp50Ac10Dc20Rm25WNLm000000\n"
)
VIRTUAL_TOOL_TEST_COMMAND = "JTX11Sp50G10H20I25WNLm000000\n"
VALID_CONTROLLER_POSITION = parse_position_response(
    "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"
)
VALID_CONTROLLER_IDENTITY_RESPONSE = json.dumps(
    {
        "DriverModel": "Teensy 4.1",
        "FirmwareVersion": "6.7.1-ar4hmi.1",
        "RobotModel": "AR4",
        "RobotVersion": "MK3",
        "SerialNumber": "Unset",
        "AssetTag": "Unset",
        "ProtocolCapabilities": [
            CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1,
        ],
    },
    separators=(",", ":"),
)
SPEED_VIOLATION_CONTROLLER_POSITION = parse_position_response(
    "A1B2C3D4E5F6G1H2I3J4K5L6M1NOP7Q8R9"
)
TEST_CONTROLLER_CALIBRATION = ControllerJointCalibration(
    negative_limits=(100,) * 9,
    positive_limits=(100,) * 9,
    steps_per_unit=(100,) * 9,
)


def canonicalize_main_test_command(command):
    calibration = (
        TEST_CONTROLLER_CALIBRATION
        if isinstance(command, str)
        and command[:2] in ("MG", "MJ", "ML", "MV", "RJ", "WC", "WG")
        else None
    )
    return canonicalize_serial_command(command, calibration)


def completed_virtual_operation(succeeded=True, error="virtual drive failed"):
    operation = VirtualMotionOperation()
    if succeeded:
        operation.complete(True)
    else:
        operation.complete(False, error)
    return operation


def complete_virtual_callback(
    callback,
    request_lease,
    operation,
    timeout,
    controller_succeeded=True,
    deadline=None,
    timed_out=False,
    settlement_callback=None,
):
    succeeded = controller_succeeded is True and isinstance(
        operation,
        VirtualMotionOperation,
    ) and operation.result()[0]
    if settlement_callback is not None:
        succeeded = settlement_callback(succeeded)
    request_lease.close()
    callback(succeeded)


class HmiSourceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(AR4_SOURCE.read_text(encoding="utf-8"), filename=str(AR4_SOURCE))
        cls.module_functions = {
            node.name: node
            for node in cls.tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        cls.module_classes = {
            node.name: node
            for node in cls.tree.body
            if isinstance(node, ast.ClassDef)
        }

    def compile_function(self, name, namespace, *, preserve_decorators=False):
        self.add_motion_request_dependencies(namespace)
        namespace.setdefault("dataclass", dataclass)
        namespace.setdefault("threading", threading)
        namespace.setdefault("contextmanager", contextmanager)
        namespace.setdefault("application_lifecycle_lock", threading.Lock())
        namespace.setdefault("calibration_terminal_owner_lock", threading.Lock())
        namespace.setdefault(
            "calibration_terminal_response_pending",
            threading.Event(),
        )
        namespace.setdefault(
            "calibration_serial_write_committed",
            threading.Event(),
        )
        namespace.setdefault(
            "CALIBRATION_POSITION_KEYS",
            (
                "J1AngCur", "J2AngCur", "J3AngCur",
                "J4AngCur", "J5AngCur", "J6AngCur",
                "XcurPos", "YcurPos", "ZcurPos",
                "RzcurPos", "RycurPos", "RxcurPos",
                "J7PosCur", "J8PosCur", "J9PosCur",
            ),
        )
        namespace.setdefault("_capture_calibration_pose_snapshot", lambda: object())
        namespace.setdefault("SerialActivityRejected", SerialActivityRejected)
        if name == "_require_calibration_terminal_response":
            for class_name in (
                "CalibrationCancellationBoundary",
                "CalibrationWriteCommitment",
            ):
                if class_name not in namespace:
                    namespace[class_name] = self.compile_class(
                        class_name,
                        namespace,
                    )
        if name in (
            "_capture_calibration_pose_snapshot",
            "_restore_calibration_pose_snapshot",
        ) and "CalibrationPoseSnapshot" not in namespace:
            namespace["CalibrationPoseSnapshot"] = self.compile_class(
                "CalibrationPoseSnapshot",
                namespace,
            )
        if (
            name != "_motion_request_rejection_message"
            and "_motion_request_rejection_message" not in namespace
        ):
            self.compile_function("_motion_request_rejection_message", namespace)
        namespace.setdefault("AUXILIARY_BOARD_NONE", AUXILIARY_BOARD_NONE)
        namespace.setdefault(
            "normalize_auxiliary_board_profile",
            normalize_auxiliary_board_profile,
        )
        namespace.setdefault("MotionInputError", MotionInputError)
        namespace.setdefault("math", math)
        namespace.setdefault("IK_POSITION_TOLERANCE_MILLIMETRES", 0.1)
        namespace.setdefault("IK_ROTATION_TOLERANCE_DEGREES", 0.1)
        namespace.setdefault("IK_JOINT_LIMIT_TOLERANCE_DEGREES", 0.001)
        namespace.setdefault("IK_WRIST_SINGULARITY_DEGREES", 2.0)
        if (
            name not in (
                "_clear_auxiliary_board_profile",
                "_motion_request_rejection_message",
            )
            and "_clear_auxiliary_board_profile" not in namespace
        ):
            self.compile_function(
                "_clear_auxiliary_board_profile",
                namespace,
            )
        namespace.setdefault(
            "canonicalize_serial_command",
            canonicalize_serial_command,
        )
        namespace.setdefault(
            "_canonicalize_main_serial_command",
            canonicalize_main_test_command,
        )
        namespace.setdefault(
            "canonicalize_virtual_command",
            canonicalize_virtual_command,
        )
        namespace.setdefault(
            "controller_degree_to_native_radians",
            controller_degree_to_native_radians,
        )
        namespace.setdefault("controller_number", controller_number)
        namespace.setdefault("finite_number", finite_number)
        namespace.setdefault(
            "validate_controller_filename",
            validate_controller_filename,
        )
        namespace.setdefault("LIVE_TOOL_JOG_INCREMENT", 0.25)
        namespace.setdefault("ControllerIdentity", ControllerIdentity)
        namespace.setdefault(
            "CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1",
            CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1,
        )
        namespace.setdefault(
            "parse_controller_identity_response",
            parse_controller_identity_response,
        )
        namespace.setdefault(
            "parse_virtual_command_timing",
            parse_virtual_command_timing,
        )
        namespace.setdefault("parse_command_timing", parse_command_timing)
        namespace.setdefault(
            "parse_motion_wrist_config",
            parse_motion_wrist_config,
        )
        if (
            name in ("live_joint_jog", "live_cartesian_jog", "live_tool_jog")
            and "_parse_live_jog_drive_profile" not in namespace
        ):
            self.compile_function(
                "_parse_live_jog_drive_profile",
                namespace,
            )
        if "kinematics_configuration_ready" not in namespace:
            kinematics_ready = threading.Event()
            kinematics_ready.set()
            namespace["kinematics_configuration_ready"] = kinematics_ready
        namespace.setdefault("VirtualMotionOperation", VirtualMotionOperation)
        namespace.setdefault("PositionResponse", PositionResponse)
        namespace.setdefault("time", time)
        if name in (
            "_read_auxiliary_inactive_stop_response",
            "_run_auxiliary_stop_safe",
            "_try_dispatch_auxiliary_stop",
        ):
            namespace.setdefault(
                "auxiliary_stop_acknowledgement_deadline",
                None,
            )
        if name in ("_try_dispatch_auxiliary_stop", "_request_auxiliary_stop"):
            namespace.setdefault("_auxiliary_stop_not_required", lambda: False)
        if name == "setCom" and "_set_com_admitted" not in namespace:
            self.compile_function("_set_com_admitted", namespace)
        if name in (
            "_read_auxiliary_inactive_stop_response",
            "_run_auxiliary_stop_safe",
        ):
            namespace.setdefault("auxiliary_stop_owner_waiting", False)
            if "_raise_auxiliary_stop_acknowledgement_timeout" not in namespace:
                self.compile_function(
                    "_raise_auxiliary_stop_acknowledgement_timeout",
                    namespace,
                )
            if "_remaining_auxiliary_stop_acknowledgement_time" not in namespace:
                self.compile_function(
                    "_remaining_auxiliary_stop_acknowledgement_time",
                    namespace,
                )
        if name == "_execute_row_auxiliary_command" and (
            "_auxiliary_stop_acknowledgement_deadline_value" not in namespace
        ):
            namespace.setdefault("auxiliary_stop_acknowledgement_deadline", None)
            self.compile_function(
                "_auxiliary_stop_acknowledgement_deadline_value",
                namespace,
            )
        dependencies = {
            "_set_cpp_kinematics_from_values": (
                "_prepare_cpp_kinematics_configuration",
            ),
            "_prepare_controller_startup": (
                "_prepare_cpp_kinematics_configuration",
            ),
            "_prepare_cpp_kinematics_configuration": (
                "_validated_native_kinematics_rotations",
            ),
            "_validated_native_kinematics_rotations": (
                "_validated_native_tool_frame",
            ),
            "_prepare_update_parameters_from_values": (
                "_validated_native_kinematics_rotations",
            ),
            "_active_tool_frame": (
                "_validated_native_tool_frame",
            ),
            "_solve_inverse_kinematics": (
                "_validated_virtual_six_vector",
                "_validated_wrist_config",
                "_validate_inverse_kinematics_result",
            ),
            "_rotation_vector_matrix": (
                "_validated_virtual_six_vector",
            ),
            "_external_cartesian_pose_to_native": (
                "_validated_virtual_six_vector",
            ),
            "_native_cartesian_pose_to_external": (
                "_validated_virtual_six_vector",
            ),
            "_forward_kinematics_display_pose": (
                "_validated_virtual_six_vector",
                "_native_cartesian_pose_to_external",
            ),
            "refresh_gui_from_joint_angles": (
                "_forward_kinematics_display_pose",
            ),
            **{
                function_name: ("_forward_kinematics_display_pose",)
                for function_name in (
                    "XjogNeg",
                    "YjogNeg",
                    "ZjogNeg",
                    "RxjogNeg",
                    "RyjogNeg",
                    "RzjogNeg",
                    "XjogPos",
                    "YjogPos",
                    "ZjogPos",
                    "RxjogPos",
                    "RyjogPos",
                    "RzjogPos",
                )
            },
            "_rotation_error_degrees": (
                "_rotation_vector_matrix",
            ),
            "_validate_inverse_kinematics_result": (
                "_validated_native_ordered_values",
                "_rotation_error_degrees",
            ),
            "_acquire_motion_request": ("_reject_motion_request",),
            "parse_mj_command": ("_external_cartesian_pose_to_native",),
            "mj_command": ("_solve_inverse_kinematics",),
            "mt_command": (
                "_solve_inverse_kinematics",
                "_active_tool_frame",
                "_external_cartesian_pose_to_native",
            ),
            "live_cartesian_jog": (
                "_solve_inverse_kinematics",
                "_external_cartesian_pose_to_native",
            ),
            "live_tool_jog": ("_solve_inverse_kinematics",),
            "LiveCarJog": ("_validated_wrist_config",),
            "LiveToolJog": (
                "_validated_wrist_config",
                "_active_tool_frame",
            ),
            "_prepare_position_command": (
                "_acknowledged_forced_position_target_value",
            ),
            "_prepare_forced_position_command": (
                "_prepare_forced_position_request",
            ),
            "_gcode_playback_command": ("_gcode_storage_filename",),
            "displayPosition": (
                "_clear_acknowledged_forced_position_target",
                "_calibration_pose_widget_groups",
            ),
            "_execute_calibration_command": (
                "_require_calibration_terminal_response",
                "_calibration_result_failure_details",
                "_handle_calibration_result_application_failure",
            ),
            "_run_calibration_stage_safe": (
                "_require_calibration_terminal_response",
            ),
            "_apply_calibration_worker_result": (
                "_calibration_result_failure_details",
                "_handle_calibration_result_application_failure",
            ),
            "_handle_calibration_result_application_failure": (
                "_calibration_result_failure_details",
            ),
        }
        for dependency in dependencies.get(name, ()):
            if dependency not in namespace:
                self.compile_function(
                    dependency,
                    namespace,
                    preserve_decorators=(
                        dependency == "_require_calibration_terminal_response"
                    ),
                )
        if name == "_poll_application_close":
            self.add_shutdown_dependencies(namespace)
        function = copy.deepcopy(self.module_functions[name])
        if not preserve_decorators:
            function.decorator_list = []
        module = ast.Module(body=[function], type_ignores=[])
        compiled = compile(ast.fix_missing_locations(module), str(AR4_SOURCE), "exec")
        exec(compiled, namespace)
        return namespace[name]

    @staticmethod
    def add_motion_request_dependencies(namespace):
        registry = namespace.setdefault(
            "motion_request_registry",
            MotionRequestRegistry(),
        )
        namespace.setdefault("MotionRequestLease", MotionRequestLease)
        namespace.setdefault(
            "application_closing",
            SimpleNamespace(is_set=lambda: False),
        )
        namespace.setdefault("virtual_motion_event_queue", Queue())
        namespace.setdefault("manual_motion_request_state", threading.local())
        namespace.setdefault("motion_request_admission_state", threading.local())
        namespace.setdefault("manual_motion_pose_pending", threading.Event())
        namespace.setdefault(
            "controller_position_resynchronization_required",
            threading.Event(),
        )
        namespace.setdefault("acknowledged_forced_position_lock", threading.Lock())
        namespace.setdefault("acknowledged_forced_position_target", None)
        namespace.setdefault("joint_motion_request_lock", threading.Lock())
        namespace.setdefault("joint_motion_request_lease", None)
        namespace.setdefault("offline_live_jog_motion_lease", None)
        namespace.setdefault(
            "_try_dispatch_controller_correction",
            lambda: False,
        )
        namespace.setdefault(
            "_try_dispatch_deferred_joint_adjustments",
            lambda **kwargs: False,
        )
        namespace.setdefault("_capture_program_motion_pose", lambda: object())
        namespace.setdefault(
            "_reconcile_program_motion_pose",
            lambda snapshot, position, write_started, succeeded: succeeded,
        )

        def acquire(
            name,
            allow_position_recovery=False,
            requires_kinematics=True,
        ):
            readiness = namespace.get("kinematics_configuration_ready")
            if (
                requires_kinematics
                and readiness is not None
                and not readiness.is_set()
            ):
                return None
            if not allow_position_recovery and (
                namespace["manual_motion_pose_pending"].is_set()
                or namespace[
                    "controller_position_resynchronization_required"
                ].is_set()
            ):
                return None
            return registry.acquire(name)

        def finish(request_lease, completion_callback=None, succeeded=None):
            if request_lease.close() is not True:
                raise RuntimeError("motion request was already released")
            if completion_callback is not None:
                completion_callback(succeeded)
            return True

        def reserve_joint():
            existing = namespace.get("joint_motion_request_lease")
            if existing is not None:
                return existing, False
            lease = registry.acquire("Joint target dispatcher")
            if lease is None:
                return None, False
            namespace["joint_motion_request_lease"] = lease
            return lease, True

        def abandon_joint(request_lease):
            namespace["joint_motion_request_lease"] = None
            return request_lease.close()

        namespace.setdefault("_acquire_motion_request", acquire)
        namespace.setdefault("_finish_motion_request", finish)

        def finish_settled(
            completion_callback,
            request_lease,
            succeeded,
            settlement_callback=None,
        ):
            if settlement_callback is not None:
                succeeded = settlement_callback(succeeded)
            return finish(request_lease, completion_callback, succeeded)

        namespace.setdefault(
            "_finish_settled_motion_request",
            finish_settled,
        )
        namespace.setdefault("_reserve_joint_motion_request", reserve_joint)
        namespace.setdefault("_abandon_joint_motion_request", abandon_joint)
        namespace.setdefault(
            "_finish_joint_motion_request_if_idle",
            lambda: False,
        )
        namespace.setdefault(
            "_start_offline_joint_motion",
            lambda command: isinstance(
                namespace["rj_command"](command),
                VirtualMotionOperation,
            ),
        )

    @staticmethod
    def add_shutdown_dependencies(namespace):
        namespace.setdefault("_calibration_shutdown_pending", lambda: False)
        namespace.setdefault("_poll_calibration_events", lambda: None)
        namespace.setdefault("_poll_virtual_motion_events", lambda: None)
        namespace.setdefault("startup_controller_cleanup_lock", threading.Lock())
        namespace.setdefault("startup_controller_cleanup_pending", {})
        namespace.setdefault("_ensure_startup_controller_cleanup", lambda: True)
        namespace.setdefault("startup_auxiliary_cleanup_lock", threading.Lock())
        namespace.setdefault("startup_auxiliary_cleanup_pending", {})
        namespace.setdefault("_ensure_startup_auxiliary_cleanup", lambda: True)
        namespace.setdefault("offline_live_jog_state_lock", threading.Lock())
        namespace.setdefault("offline_live_jog_operation", None)
        namespace.setdefault("_virtual_motion_active", lambda: False)

    def add_save_and_apply_dependencies(self, namespace):
        namespace.setdefault(
            "_preflight_controller_calibration_transport",
            lambda: True,
        )
        namespace.setdefault("threading", threading)
        namespace.setdefault("_validate_controller_pose", lambda values: True)
        namespace["_restore_prewrite_calibration"] = self.compile_function(
            "_restore_prewrite_calibration",
            namespace,
        )

    def add_virtual_completion_timeout(self, namespace):
        namespace.update({
            "parse_virtual_command_timing": parse_virtual_command_timing,
            "motion_timing_response_timeout": motion_timing_response_timeout,
            "_configured_motion_timeout_bounds": lambda: (100.0, 1000.0),
            "VIRTUAL_COMPLETION_BASE_TIMEOUT_SECONDS": 120.0,
            "VIRTUAL_COMPLETION_SAFETY_SCALE": 1.25,
            "VIRTUAL_JOINT_SECONDS_SCALE": 4.5,
            "VIRTUAL_CARTESIAN_SECONDS_SCALE": 4.7,
            "VIRTUAL_TOOL_SECONDS_SCALE": 5.1,
            "SERIAL_RESPONSE_MARGIN_SECONDS": 10.0,
        })
        namespace["_virtual_completion_timeout"] = self.compile_function(
            "_virtual_completion_timeout",
            namespace,
        )

    def compile_class(self, name, namespace):
        class_node = copy.deepcopy(self.module_classes[name])
        module = ast.Module(body=[class_node], type_ignores=[])
        compiled = compile(ast.fix_missing_locations(module), str(AR4_SOURCE), "exec")
        exec(compiled, namespace)
        return namespace[name]

    def compile_nested_function(self, name, namespace):
        matches = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == name
        ]
        self.assertEqual(len(matches), 1, name)
        function = copy.deepcopy(matches[0])
        function.decorator_list = []
        module = ast.Module(body=[function], type_ignores=[])
        compiled = compile(ast.fix_missing_locations(module), str(AR4_SOURCE), "exec")
        exec(compiled, namespace)
        return namespace[name]

    def compile_async_calibration_lifecycle(self, exchange):
        class Entry:
            def __init__(self, value=""):
                self.value = value

            def get(self, *args):
                return self.value

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

        class Label:
            def __init__(self):
                self.text = None
                self.style = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")
                self.style = kwargs.get("style")

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

        class Port:
            is_open = True

        calibration = {}
        for axis in range(1, 10):
            calibration[f"J{axis}CalStatVal"] = Entry("1" if axis <= 3 else "0")
            calibration[f"J{axis}CalStatVal2"] = Entry(
                "1" if 4 <= axis <= 6 else "0"
            )
            calibration[f"J{axis}calOff"] = "0"
        for axis in range(1, 7):
            calibration[f"J{axis}AngCur"] = str(axis)

        state = {
            "activity_registry": SerialActivityRegistry(("ser",)),
            "motion_registry": MotionRequestRegistry(),
            "transport_lock": threading.Lock(),
            "event_queue": Queue(),
            "invalidations": [],
            "captured_pose_snapshots": [],
            "restored_pose_snapshots": [],
            "applied_positions": [],
            "monitor_updates": [],
            "first_label": Label(),
            "second_label": Label(),
        }

        def capture_pose_snapshot():
            snapshot = object()
            state["captured_pose_snapshots"].append(snapshot)
            return snapshot

        def restore_pose_snapshot(snapshot):
            state["restored_pose_snapshots"].append(snapshot)
            return True

        namespace = {
            "dataclass": dataclass,
            "MotionInputError": MotionInputError,
            "MotionRequestLease": MotionRequestLease,
            "SerialActivityRejected": SerialActivityRejected,
            "SerialTransportQuarantinedError": ConnectionError,
            "finite_number": finite_number,
            "calibration_serial_event_queue": state["event_queue"],
            "calibration_operation_lock": threading.Lock(),
            "calibration_operation": None,
            "calibration_next_request_id": 0,
            "calibration_terminal_owner_lock": threading.Lock(),
            "calibration_terminal_response_pending": threading.Event(),
            "calibration_serial_write_committed": threading.Event(),
            "application_lifecycle_lock": threading.Lock(),
            "serial_lock": state["transport_lock"],
            "serial_write_lock": threading.Lock(),
            "serial_activity_registry": state["activity_registry"],
            "motion_request_registry": state["motion_registry"],
            "application_closing": threading.Event(),
            "RUN": {
                "offlineMode": False,
                "ser": Port(),
                "VR_angles": [0.0] * 6,
            },
            "CAL": calibration,
            "cmdSentEntryField": Entry(),
            "cmdRecEntryField": Entry(),
            "almStatusLab": state["first_label"],
            "almStatusLab2": state["second_label"],
            "tab8": SimpleNamespace(ElogView=Entry()),
            "END": "end",
            "pickle": SimpleNamespace(dump=lambda *args: None),
            "open": lambda *args: object(),
            "logger": SimpleNamespace(
                info=lambda *args: None,
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "ErrorHandler": lambda response: None,
            "ProtocolResponseError": ProtocolResponseError,
            "parse_position_response": parse_position_response,
            "_current_controller_joint_calibration": (
                lambda: ControllerJointCalibration(
                    negative_limits=(180.0,) * 9,
                    positive_limits=(180.0,) * 9,
                    steps_per_unit=(1.0,) * 9,
                )
            ),
            "displayPosition": (
                lambda response, parsed=None: state["applied_positions"].append(parsed)
                or parsed
            ),
            "_invalidate_uncertain_controller_calibration": state[
                "invalidations"
            ].append,
            "_capture_calibration_pose_snapshot": capture_pose_snapshot,
            "_restore_calibration_pose_snapshot": restore_pose_snapshot,
            "setStepMonitorsVR": lambda: state["monitor_updates"].append(True),
            "exchange_serial_line_until_cancelled": exchange,
            "serial_transport_quarantined": lambda serial_port: False,
            "threading": threading,
            "Empty": Empty,
            "root": Root(),
        }
        self.add_startup_command_dependencies(namespace)
        namespace["_binary_controller_flag"] = self.compile_function(
            "_binary_controller_flag",
            namespace,
        )
        namespace["_prepare_calibration_command"] = self.compile_function(
            "_prepare_calibration_command",
            namespace,
        )
        namespace["_calibration_available"] = self.compile_function(
            "_calibration_available",
            namespace,
        )
        namespace["_apply_valid_position_response"] = self.compile_function(
            "_apply_valid_position_response",
            namespace,
        )
        namespace["CalibrationStage"] = self.compile_class(
            "CalibrationStage",
            namespace,
        )
        namespace["CalibrationWorkerResult"] = self.compile_class(
            "CalibrationWorkerResult",
            namespace,
        )
        namespace["CalibrationOperation"] = self.compile_class(
            "CalibrationOperation",
            namespace,
        )
        for function_name in (
            "_set_calibration_status",
            "_finish_calibration_operation",
            "_run_calibration_stage_safe",
            "_start_calibration_stage_worker",
            "_claim_calibration_worker_result",
            "_record_calibration_response",
            "_apply_calibration_worker_result",
            "_settle_rejected_calibration_worker_result",
            "_poll_calibration_events",
            "_start_calibration_sequence",
            "startCalRobotAll",
            "_start_single_joint_calibration",
            "startCalRobotJ1",
            "startCalRobotJ2",
            "startCalRobotJ3",
            "startCalRobotJ4",
            "startCalRobotJ5",
            "startCalRobotJ6",
            "startCalRobotJ7",
            "startCalRobotJ8",
            "startCalRobotJ9",
        ):
            namespace[function_name] = self.compile_function(
                function_name,
                namespace,
            )
        return namespace, state

    @staticmethod
    def _valid_update_parameter_values():
        values = {
            "TFx": 0,
            "TFy": 0,
            "TFz": 0,
            "TFrx": 0,
            "TFry": 0,
            "TFrz": 0,
        }
        for axis in range(1, 10):
            values[f"J{axis}MotDir"] = 1
            values[f"J{axis}CalDir"] = 1
        for axis in range(1, 7):
            values.update(
                {
                    f"J{axis}PosLim": 180,
                    f"J{axis}NegLim": 180,
                    f"J{axis}StepDeg": 100,
                    f"J{axis}DriveMS": 16,
                    f"J{axis}EncCPR": 4096,
                    f"J{axis}ΘDHpar": 0,
                    f"J{axis}αDHpar": 0,
                    f"J{axis}dDHpar": 0,
                    f"J{axis}aDHpar": 0,
                }
            )
        return values

    @classmethod
    def _valid_custom_calibration_profile(cls):
        values = cls._valid_update_parameter_values()
        values.update(
            {
                "J7PosLim": 700,
                "J7rotation": 360,
                "J7steps": 36000,
                "J8length": 800,
                "J8rotation": 360,
                "J8steps": 36000,
                "J9length": 900,
                "J9rotation": 360,
                "J9steps": 36000,
            }
        )
        values.update(
            {
                f"J{axis}calOff": axis / 10
                for axis in range(1, 10)
            }
        )
        return values

    def add_startup_command_dependencies(self, namespace):
        namespace.setdefault("MAX_COMMAND_LENGTH", 4096)
        namespace.setdefault("MotionInputError", MotionInputError)
        namespace.setdefault("controller_protocol_decimal", controller_protocol_decimal)
        namespace["_validated_startup_command"] = self.compile_function(
            "_validated_startup_command",
            namespace,
        )
        namespace["_build_startup_numeric_command"] = self.compile_function(
            "_build_startup_numeric_command",
            namespace,
        )

    def add_custom_profile_validation_dependencies(self, namespace):
        namespace.setdefault("ControllerJointCalibration", ControllerJointCalibration)
        namespace.setdefault("MotionInputError", MotionInputError)
        namespace.setdefault("controller_ratio", controller_ratio)
        namespace.setdefault("finite_number", finite_number)
        self.add_startup_command_dependencies(namespace)
        for name in (
            "_binary_controller_flag",
            "_robot_joint_calibration_from_values",
            "_controller_joint_positions_from_values",
            "_controller_joint_calibration_from_values",
            "_validate_controller_pose",
            "_prepare_update_parameters_from_values",
            "_prepare_external_axis_parameters_from_values",
            "_custom_calibration_profile_keys",
            "_custom_calibration_field_values",
            "_prepare_custom_calibration_profile",
        ):
            namespace[name] = self.compile_function(name, namespace)

    def test_module_level_functions_are_not_shadowed(self):
        names = [
            node.name
            for node in self.tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        ]

        self.assertEqual(len(names), len(set(names)))

    def test_execute_row_has_no_bare_returns_that_skip_row_settlement(self):
        bare_returns = [
            node.lineno
            for node in ast.walk(self.module_functions["executeRow"])
            if isinstance(node, ast.Return) and node.value is None
        ]

        self.assertEqual(bare_returns, [])

    def test_vtk_rendering_throttles_when_joint_angles_are_idle(self):
        class RenderWindow:
            def __init__(self):
                self.render_count = 0

            def Render(self):
                self.render_count += 1

        class RootWidget:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

        render_window = RenderWindow()
        root_widget = RootWidget()
        joint_updates = []
        namespace = {
            "RUN": {
                "vtk_running": True,
                "VR_angles": [0.0] * 6,
                "lastRenderedAngles": (0.0,) * 6,
            },
            "finite_number": finite_number,
            "update_joint_angles": lambda: joint_updates.append(True),
        }
        update_vtk = self.compile_function("update_vtk", namespace)

        update_vtk(render_window, root_widget)

        self.assertEqual(render_window.render_count, 0)
        self.assertEqual(joint_updates, [])
        self.assertEqual(root_widget.jobs[0][0], 100)

        namespace["RUN"]["VR_angles"][2] = 4.5
        root_widget.jobs.pop(0)[1]()

        self.assertEqual(render_window.render_count, 1)
        self.assertEqual(joint_updates, [True])
        self.assertEqual(
            namespace["RUN"]["lastRenderedAngles"],
            (0.0, 0.0, 4.5, 0.0, 0.0, 0.0),
        )
        self.assertEqual(root_widget.jobs[0][0], 16)

        namespace["RUN"]["vtk_running"] = False
        root_widget.jobs.pop(0)[1]()

        self.assertEqual(root_widget.jobs, [])

    def test_non_motion_response_timeout_does_not_require_motion_bounds(self):
        bounds_calls = []
        namespace = {
            "parse_command_timing": lambda command: None,
            "MotionInputError": MotionInputError,
            "SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS": 120,
            "_configured_motion_timeout_bounds": lambda: bounds_calls.append(True),
        }
        response_timeout = self.compile_function(
            "_controller_response_timeout",
            namespace,
        )

        self.assertEqual(response_timeout("RP\n"), 120)
        self.assertEqual(bounds_calls, [])

    def test_motion_response_timeout_uses_configured_bounds(self):
        calls = []

        def derive(*args, **kwargs):
            calls.append((args, kwargs))
            return 321

        namespace = {
            "parse_command_timing": lambda command: object(),
            "MotionInputError": MotionInputError,
            "SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS": 120,
            "SERIAL_RESPONSE_MARGIN_SECONDS": 10,
            "_configured_motion_timeout_bounds": lambda: (80, 900),
            "command_response_timeout": derive,
        }
        response_timeout = self.compile_function(
            "_controller_response_timeout",
            namespace,
        )

        command = (
            "RJA1B2C3D4E5F6J70J80J90"
            "Ss200Ac10Dc20Rm25WNLm000000\n"
        )
        self.assertEqual(response_timeout(command), 321)
        self.assertEqual(
            calls,
            [
                (
                    (command, 120, 80, 900),
                    {"margin_seconds": 10},
                )
            ],
        )

    def test_controller_timeout_rejects_ramp_above_firmware_limit(self):
        namespace = {
            "parse_command_timing": parse_command_timing,
            "MotionInputError": MotionInputError,
            "SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS": 120,
            "SERIAL_RESPONSE_MARGIN_SECONDS": 10,
            "_configured_motion_timeout_bounds": lambda: (100, 1000),
            "command_response_timeout": command_response_timeout,
        }
        response_timeout = self.compile_function(
            "_controller_response_timeout",
            namespace,
        )

        prefix = "RJA1B2C3D4E5F6J70J80J90"
        standard_ramp = response_timeout(
            f"{prefix}Sp100Ac10Dc20Rm100WNLm000000\n"
        )

        self.assertEqual(standard_ramp, 1260.0)
        with self.assertRaises(MotionInputError):
            response_timeout(
                f"{prefix}Sp100Ac10Dc20Rm200WNLm000000\n"
            )

    def test_motion_timeout_bounds_follow_controller_configuration(self):
        calibration = {}
        for axis in range(1, 7):
            calibration[f"J{axis}PosLim"] = 10
            calibration[f"J{axis}NegLim"] = 10
            calibration[f"J{axis}StepDeg"] = 100
            calibration[f"J{axis}aDHpar"] = 1
            calibration[f"J{axis}dDHpar"] = 1
        calibration.update(
            {
                "J7PosLim": 10,
                "J8length": 10,
                "J9length": 10,
                "J7rotation": 1,
                "J8rotation": 1,
                "J9rotation": 1,
                "J7steps": 100,
                "J8steps": 100,
                "J9steps": 100,
                "TFx": 0,
                "TFy": 0,
                "TFz": 0,
            }
        )
        namespace = {
            "CAL": calibration,
            "RUN": {"minSpeedDelay": 200},
            "math": math,
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
            "FIRMWARE_AXIS_COUNT": 9,
            "FIRMWARE_DISTRIBUTION_DELAY_MICROSECONDS": 30,
            "FIRMWARE_MAX_MILLIMETERS_PER_SECOND": 192,
        }
        namespace["_configuration_number"] = self.compile_function(
            "_configuration_number",
            namespace,
        )
        namespace["_runtime_number"] = self.compile_function(
            "_runtime_number",
            namespace,
        )
        configured_bounds = self.compile_function(
            "_configured_motion_timeout_bounds",
            namespace,
        )

        full_scale_seconds, millimeter_distance = configured_bounds()

        self.assertEqual(full_scale_seconds, 0.94)
        self.assertAlmostEqual(millimeter_distance, 2.0 * math.pi * 42.0)

    def test_joint_dispatcher_calibration_uses_controller_limits_and_steps(self):
        calibration = {}
        for axis in range(1, 7):
            calibration[f"J{axis}NegLim"] = axis
            calibration[f"J{axis}PosLim"] = axis + 10
            calibration[f"J{axis}StepDeg"] = axis * 100
        calibration.update(
            {
                "J7PosLim": 70,
                "J8length": 80,
                "J9length": 90,
                "J7rotation": 2,
                "J8rotation": 4,
                "J9rotation": 5,
                "J7steps": 200,
                "J8steps": 800,
                "J9steps": 1500,
            }
        )
        namespace = {
            "CAL": calibration,
            "ControllerJointCalibration": ControllerJointCalibration,
            "controller_ratio": controller_ratio,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
        }
        namespace["_controller_joint_calibration_from_values"] = (
            self.compile_function(
                "_controller_joint_calibration_from_values",
                namespace,
            )
        )
        current_calibration = self.compile_function(
            "_current_controller_joint_calibration",
            namespace,
        )

        result = current_calibration()

        self.assertEqual(result.negative_limits, (1, 2, 3, 4, 5, 6, 0, 0, 0))
        self.assertEqual(result.positive_limits, (11, 12, 13, 14, 15, 16, 70, 80, 90))
        self.assertEqual(result.steps_per_unit, (100, 200, 300, 400, 500, 600, 100, 200, 300))

    def test_inverse_kinematics_uses_validated_wrist_configuration(self):
        class Robot:
            def __init__(self):
                self.calls = []

            def SolveInverseKinematicsConfigured(self, target, estimate, wrist):
                self.calls.append((target, estimate, wrist))
                return [1, 2, 3, 4, 5, 6]

            @staticmethod
            def forward_kinematics(joints):
                return [0, 1, 2, *[math.radians(value) for value in (3, 4, 5)]]

            def SolveInverseKinematics(self, target, estimate):
                raise AssertionError("legacy solver was selected")

        robot = Robot()
        namespace = {
            "robot": robot,
            "CAL": {
                **{f"J{axis}PosLim": 180 for axis in range(1, 7)},
                **{f"J{axis}NegLim": 180 for axis in range(1, 7)},
            },
            "finite_number": finite_number,
        }
        solve = self.compile_function("_solve_inverse_kinematics", namespace)

        result = solve(
            np.arange(6),
            (value for value in range(6, 12)),
            " f ",
        )

        self.assertEqual(result, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        self.assertEqual(
            robot.calls,
            [((0.0, 1.0, 2.0, 3.0, 4.0, 5.0),
              (6.0, 7.0, 8.0, 9.0, 10.0, 11.0),
              "F")],
        )

    def test_inverse_kinematics_rejects_legacy_solver_without_calling_it(self):
        calls = []
        namespace = {
            "robot": SimpleNamespace(
                SolveInverseKinematics=(
                    lambda target, estimate: calls.append((target, estimate))
                ),
                forward_kinematics=lambda joints: [0.0] * 6,
            ),
            "CAL": {
                **{f"J{axis}PosLim": 180 for axis in range(1, 7)},
                **{f"J{axis}NegLim": 180 for axis in range(1, 7)},
            },
            "finite_number": finite_number,
        }
        solve = self.compile_function("_solve_inverse_kinematics", namespace)

        with self.assertRaisesRegex(MotionInputError, "required configured solver"):
            solve([10, 0, 0, 0, 0, 0], [1] * 6, "A")
        self.assertEqual(calls, [])

    def test_inverse_kinematics_rejects_invalid_boundary_data(self):
        calls = []
        robot = SimpleNamespace(
            SolveInverseKinematicsConfigured=(
                lambda *args: calls.append(args) or [0.0] * 6
            ),
            SolveInverseKinematics=lambda *args: self.fail(
                "legacy solver was selected"
            ),
            forward_kinematics=lambda joints: [0.0] * 6,
        )
        namespace = {
            "robot": robot,
            "CAL": {
                **{f"J{axis}PosLim": 180 for axis in range(1, 7)},
                **{f"J{axis}NegLim": 180 for axis in range(1, 7)},
            },
            "finite_number": finite_number,
        }
        solve = self.compile_function("_solve_inverse_kinematics", namespace)

        with self.assertRaisesRegex(MotionInputError, "wrist configuration"):
            solve([0] * 6, [0] * 6, "sideways")
        self.assertEqual(calls, [])

        robot.SolveInverseKinematicsConfigured = lambda *args: [0.0] * 5
        with self.assertRaisesRegex(MotionInputError, "result must contain six"):
            solve([0] * 6, [0] * 6, "A")

        robot.SolveInverseKinematicsConfigured = (
            lambda *args: [0.0, 0.0, 0.0, math.inf, 0.0, 0.0]
        )
        with self.assertRaises(MotionInputError):
            solve([0] * 6, [0] * 6, "A")

        robot.SolveInverseKinematicsConfigured = (
            lambda *args: [0.0, 0.0, 0.0, 0.0, 20.0, 0.0]
        )
        with self.assertRaisesRegex(MotionInputError, "N wrist branch"):
            solve([0] * 6, [0] * 6, "N")

        for invalid_result in (
            "000000",
            b"000000",
            {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0},
            {0, 1, 2, 3, 4, 5},
        ):
            with self.subTest(invalid_result=type(invalid_result).__name__):
                robot.SolveInverseKinematicsConfigured = (
                    lambda *args, value=invalid_result: value
                )
                with self.assertRaisesRegex(MotionInputError, "ordered numeric sequence"):
                    solve([0] * 6, [0] * 6, "A")

        robot.SolveInverseKinematicsConfigured = lambda *args: [0.0] * 6
        for invalid_pose in (
            "000000",
            b"000000",
            {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0},
            {0, 1, 2, 3, 4, 5},
        ):
            with self.subTest(invalid_pose=type(invalid_pose).__name__):
                robot.forward_kinematics = lambda joints, value=invalid_pose: value
                with self.assertRaisesRegex(MotionInputError, "ordered numeric sequence"):
                    solve([0] * 6, [0] * 6, "A")
        robot.forward_kinematics = lambda joints: [0.0] * 6

        robot.SolveInverseKinematicsConfigured = "incompatible"
        with self.assertRaisesRegex(MotionInputError, "required configured solver"):
            solve([0] * 6, [0] * 6, "A")

    def test_native_tool_frame_uses_xyz_rx_ry_rz_order(self):
        class Robot:
            def __init__(self):
                self.tool_frames = []
                self.raise_ik = False

            def set_robot_configuration(self, dh, positive, negative, tool):
                self.tool_frames.append(tuple(tool))

            def set_robot_tool_frame(self, *values):
                self.tool_frames.append(values)

            def SolveInverseKinematicsConfigured(self, *args):
                if self.raise_ik:
                    raise RuntimeError("IK failed")
                return None

        class Entry:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        values = self._valid_update_parameter_values()
        values.update(
            {
                "TFx": 1,
                "TFy": 2,
                "TFz": 3,
                "TFrx": 4,
                "TFry": 5,
                "TFrz": 6,
            }
        )
        robot = Robot()
        namespace = {
            "math": math,
            "finite_number": finite_number,
            "robot": robot,
        }
        set_kinematics = self.compile_function(
            "_set_cpp_kinematics_from_values",
            namespace,
        )

        self.assertTrue(set_kinematics(values))
        self.assertEqual(robot.tool_frames, [(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)])

        field_values = {
            key: Entry(value)
            for key, value in (
                ("TFxEntryField", 1),
                ("TFyEntryField", 2),
                ("TFzEntryField", 3),
                ("TFrxEntryField", 4),
                ("TFryEntryField", 5),
                ("TFrzEntryField", 6),
            )
        }
        namespace.update(field_values)
        namespace.update(
            {
                "RUN": {"offlineMode": False, "VR_angles": [0.0] * 6},
                "CAL": {
                    "XcurPos": 0,
                    "YcurPos": 0,
                    "ZcurPos": 0,
                    "RzcurPos": 0,
                    "RycurPos": 0,
                    "RxcurPos": 0,
                    "TFx": 101,
                    "TFy": 102,
                    "TFz": 103,
                    "TFrx": 104,
                    "TFry": 105,
                    "TFrz": 106,
                },
                "parse_mt_command": lambda command: {
                    "offset_vector": (10, 20, 30, 40, 50, 60),
                    "WristConfig": "N",
                },
                "np": SimpleNamespace(array=lambda values, dtype=None: values),
                "ErrorHandler": lambda response: None,
            }
        )
        move_tool = self.compile_function("mt_command", namespace)

        move_tool("JTR11Sp1G1H1I1WNLm000000\n")

        self.assertEqual(
            robot.tool_frames[-2],
            (111.0, 122.0, 133.0, 144.0, 155.0, 166.0),
        )
        self.assertEqual(
            robot.tool_frames[-1],
            (101.0, 102.0, 103.0, 104.0, 105.0, 106.0),
        )

        robot.raise_ik = True
        with self.assertRaisesRegex(RuntimeError, "IK failed"):
            move_tool("JTR11Sp1G1H1I1WNLm000000\n")
        self.assertEqual(
            robot.tool_frames[-1],
            (101.0, 102.0, 103.0, 104.0, 105.0, 106.0),
        )

    def test_tool_roll_parser_uses_validated_timing_and_wrist_fields(self):
        namespace = {
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
            ),
            "re": re,
        }
        parse_tool = self.compile_function("parse_mt_command", namespace)

        result = parse_tool("JTW11Sp50G10H20I25WFLm010101\n")

        self.assertEqual(result["offset_vector"], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        self.assertEqual(result["SpeedType"], "p")
        self.assertEqual(result["Speed"], 50.0)
        self.assertEqual(result["Acc"], 10.0)
        self.assertEqual(result["Dec"], 20.0)
        self.assertEqual(result["Ramp"], 25.0)
        self.assertEqual(result["WristConfig"], "F")

        negative = parse_tool("JTW01Sp50G10H20I25WFLm010101\n")
        self.assertEqual(
            negative["offset_vector"],
            [0.0, 0.0, 0.0, -1.0, 0.0, 0.0],
        )
        self.assertIsNone(parse_tool("JTW1-1Sp50G10H20I25WFLm010101\n"))

    def test_native_configuration_uses_one_atomic_validated_call(self):
        class Robot:
            def __init__(self):
                self.calls = []

            def set_robot_configuration(self, *args):
                self.calls.append(args)

            @staticmethod
            def SolveInverseKinematicsConfigured(*args):
                return None

        values = self._valid_update_parameter_values()
        ready = threading.Event()
        robot = Robot()
        namespace = {
            "robot": robot,
            "kinematics_configuration_ready": ready,
        }
        set_kinematics = self.compile_function(
            "_set_cpp_kinematics_from_values",
            namespace,
        )

        self.assertTrue(set_kinematics(values))
        self.assertTrue(ready.is_set())
        self.assertEqual(len(robot.calls), 1)
        dh, positive, negative, tool = robot.calls[0]
        self.assertEqual(len(dh), 24)
        self.assertEqual(len(positive), 6)
        self.assertEqual(len(negative), 6)
        self.assertEqual(len(tool), 6)

        invalid = dict(values)
        invalid["J3NegLim"] = -1
        with self.assertRaisesRegex(MotionInputError, "non-negative"):
            set_kinematics(invalid)
        self.assertFalse(ready.is_set())
        self.assertEqual(len(robot.calls), 1)

        underflow_theta = dict(values)
        underflow_theta["J1ΘDHpar"] = 1e-44
        with self.assertRaisesRegex(MotionInputError, "theta native radians"):
            set_kinematics(underflow_theta)
        self.assertFalse(ready.is_set())
        self.assertEqual(len(robot.calls), 1)

        underflow_tool_rotation = dict(values)
        underflow_tool_rotation["TFrz"] = 1e-44
        with self.assertRaisesRegex(MotionInputError, "TFrz native radians"):
            set_kinematics(underflow_tool_rotation)
        self.assertFalse(ready.is_set())
        self.assertEqual(len(robot.calls), 1)

        maximum_tool_rotation = dict(values)
        maximum_tool_rotation["TFrz"] = float.fromhex("0x1.fffffep+127")
        with self.assertRaisesRegex(MotionInputError, "TFrz native degrees"):
            set_kinematics(maximum_tool_rotation)
        self.assertFalse(ready.is_set())
        self.assertEqual(len(robot.calls), 1)

        namespace["robot"] = SimpleNamespace()
        with self.assertRaisesRegex(MotionInputError, "required configured API"):
            set_kinematics(values)
        self.assertFalse(ready.is_set())

    def test_native_configuration_rejects_incomplete_apis_before_writes(self):
        for missing_name in (
            "set_robot_configuration",
            "SolveInverseKinematicsConfigured",
        ):
            with self.subTest(missing=missing_name):
                writes = []
                robot = SimpleNamespace(
                    set_dh_parameters_explicit=lambda *values: writes.append("dh"),
                    set_joint_limits=lambda *values: writes.append("limits"),
                    set_robot_tool_frame=lambda *values: writes.append("tool"),
                    get_dh_parameters=lambda: [[0.0] * 4 for _ in range(6)],
                    get_joint_limits=lambda: ([180.0] * 6, [180.0] * 6),
                    get_robot_tool_frame=lambda: [0.0] * 6,
                    set_robot_configuration=lambda *values: writes.append("atomic"),
                    SolveInverseKinematicsConfigured=lambda *values: [0.0] * 6,
                )
                delattr(robot, missing_name)
                ready = threading.Event()
                set_kinematics = self.compile_function(
                    "_set_cpp_kinematics_from_values",
                    {
                        "robot": robot,
                        "kinematics_configuration_ready": ready,
                    },
                )

                with self.assertRaisesRegex(
                    MotionInputError,
                    rf"required configured API:.*{missing_name}",
                ):
                    set_kinematics(self._valid_update_parameter_values())
                self.assertFalse(ready.is_set())
                self.assertEqual(writes, [])

    def test_motion_admission_requires_native_configuration(self):
        ready = threading.Event()
        namespace = {
            "kinematics_configuration_ready": ready,
            "logger": SimpleNamespace(warning=lambda *args: None),
        }
        acquire = self.compile_function("_acquire_motion_request", namespace)

        self.assertIsNone(acquire("Cartesian jog"))
        self.assertEqual(
            namespace["_motion_request_rejection_message"]("fallback"),
            "Cartesian jog not started; native kinematics configuration is unavailable",
        )
        self.assertIsNone(
            acquire("Position recovery", allow_position_recovery=True)
        )
        repair_lease = acquire(
            "Configuration repair",
            requires_kinematics=False,
        )
        self.assertIsInstance(repair_lease, MotionRequestLease)
        repair_lease.close()

        namespace["controller_position_resynchronization_required"].set()
        self.assertIsNone(
            acquire("Non-motion operation", requires_kinematics=False)
        )
        recovery_lease = acquire(
            "Controller startup",
            allow_position_recovery=True,
            requires_kinematics=False,
        )
        self.assertIsInstance(recovery_lease, MotionRequestLease)
        recovery_lease.close()
        namespace["controller_position_resynchronization_required"].clear()

        ready.set()
        lease = acquire("Cartesian jog")
        self.assertIsInstance(lease, MotionRequestLease)
        lease.close()

    def test_synchronous_configuration_repair_bypasses_only_kinematics_gate(self):
        ready = threading.Event()
        namespace = {
            "kinematics_configuration_ready": ready,
            "logger": SimpleNamespace(warning=lambda *args: None),
            "wraps": wraps,
        }
        namespace["_acquire_motion_request"] = self.compile_function(
            "_acquire_motion_request",
            namespace,
        )
        decorator = self.compile_function(
            "_synchronous_motion_request",
            namespace,
        )
        calls = []
        repair = decorator(
            "Configuration repair",
            requires_kinematics=False,
        )(lambda: calls.append("repair") or True)
        motion = decorator("Motion")(lambda: calls.append("motion") or True)

        self.assertTrue(repair())
        self.assertFalse(motion())
        self.assertEqual(calls, ["repair"])

    def test_generic_program_exchange_bypasses_only_kinematics_gate(self):
        ready = threading.Event()
        exchanges = []
        registry = MotionRequestRegistry()
        namespace = {
            "kinematics_configuration_ready": ready,
            "motion_request_registry": registry,
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "wraps": wraps,
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_COMPLETE": "complete",
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "_tracked_serial_operation": (
                lambda *args, **kwargs: lambda callback: callback
            ),
            "_exchange_legacy_main_command": (
                lambda command, **contract: exchanges.append(
                    (command, contract)
                ) or b"Done\n"
            ),
        }
        namespace["_acquire_motion_request"] = self.compile_function(
            "_acquire_motion_request",
            namespace,
        )
        namespace["_synchronous_motion_request"] = self.compile_function(
            "_synchronous_motion_request",
            namespace,
        )
        execute = self.compile_function(
            "_execute_row_main_command",
            namespace,
            preserve_decorators=True,
        )

        self.assertEqual(
            execute("TLX1\n", accepted_responses=(b"Done",)),
            ("complete", b"Done\n"),
        )
        self.assertEqual(
            exchanges,
            [("TLX1\n", {"accepted_responses": (b"Done",)})],
        )
        self.assertFalse(registry.active)

    def test_offline_cartesian_jog_matches_firmware_axis_slots(self):
        base = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        pose_contract = TEENSY_CARTESIAN_POSE_CONTRACT.read_text(
            encoding="utf-8"
        )
        self.assertIn("external_cartesian_pose_to_native", pose_contract)
        self.assertIn("native_cartesian_pose_to_external", pose_contract)
        expected_targets = {
            10: (0.0, 2.0, 3.0, 6.0, 5.0, 4.0),
            11: (2.0, 2.0, 3.0, 6.0, 5.0, 4.0),
            20: (1.0, 1.0, 3.0, 6.0, 5.0, 4.0),
            21: (1.0, 3.0, 3.0, 6.0, 5.0, 4.0),
            30: (1.0, 2.0, 2.0, 6.0, 5.0, 4.0),
            31: (1.0, 2.0, 4.0, 6.0, 5.0, 4.0),
            40: (1.0, 2.0, 3.0, 6.0, 5.0, 3.0),
            41: (1.0, 2.0, 3.0, 6.0, 5.0, 5.0),
            50: (1.0, 2.0, 3.0, 6.0, 4.0, 4.0),
            51: (1.0, 2.0, 3.0, 6.0, 6.0, 4.0),
            60: (1.0, 2.0, 3.0, 5.0, 5.0, 4.0),
            61: (1.0, 2.0, 3.0, 7.0, 5.0, 4.0),
        }
        captured_targets = []
        namespace = {
            "RUN": {
                "offlineMode": True,
                "VR_angles": [0.0] * 6,
                "Alarm": "0",
            },
            "CAL": dict(zip(
                ("XcurPos", "YcurPos", "ZcurPos", "RzcurPos", "RycurPos", "RxcurPos"),
                base,
            )),
            "np": np,
            "_solve_inverse_kinematics": (
                lambda target, estimate, wrist: captured_targets.append(
                    tuple(float(value) for value in target)
                ) or None
            ),
            "_queue_virtual_motion_error": lambda error: None,
            "logger": SimpleNamespace(error=lambda *args: None),
        }
        live_cartesian = self.compile_function("live_cartesian_jog", namespace)

        for vector, expected in expected_targets.items():
            with self.subTest(vector=vector):
                captured_targets.clear()
                self.assertFalse(live_cartesian(
                    f"LCV{vector}Sp10Ac10Dc10Rm10WALm000000\n",
                    threading.Event(),
                ))
                self.assertEqual(captured_targets, [expected])

        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        live_start = firmware.index('if (function == "LC")')
        live_end = firmware.index('if (function == "LJ")', live_start)
        live_branch = firmware[live_start:live_end]
        for vector, index, operator in (
            (10, 0, "-"), (11, 0, "+"),
            (20, 1, "-"), (21, 1, "+"),
            (30, 2, "-"), (31, 2, "+"),
            (40, 3, "-"), (41, 3, "+"),
            (50, 4, "-"), (51, 4, "+"),
            (60, 5, "-"), (61, 5, "+"),
        ):
            self.assertRegex(
                live_branch,
                rf"(?s)Vector == {vector}.*?xyzuvw_In\[{index}\] "
                rf"= xyzuvw_Out\[{index}\] \{operator} JogStepInc",
            )
        self.assertIn(
            "ar4_protocol::external_cartesian_pose_to_native_radians(",
            firmware,
        )
        self.assertIn(
            "ar4_protocol::native_cartesian_pose_to_external(",
            firmware,
        )

    def test_offline_live_workers_forward_encoded_motion_profiles(self):
        expected_profile = ("p", 37.5, 12.5, 23.5, 34.5)
        captured_profiles = {}

        def runtime_namespace(label):
            stop_event = threading.Event()

            def start_segment(event, args):
                captured_profiles[label] = tuple(args[-5:])
                event.set()
                return object()

            runtime = {
                "offlineMode": True,
                "VR_angles": [0.0] * 6,
                "Alarm": "0",
                "KinematicError": 0,
            }
            calibration = {
                "XcurPos": 0.0,
                "YcurPos": 0.0,
                "ZcurPos": 0.0,
                "RzcurPos": 0.0,
                "RycurPos": 0.0,
                "RxcurPos": 0.0,
            }
            namespace = {
                "RUN": runtime,
                "CAL": calibration,
                "np": np,
                "_start_offline_virtual_segment": start_segment,
                "_await_offline_virtual_segment": lambda operation: True,
                "_queue_virtual_motion_error": lambda error: None,
                "logger": SimpleNamespace(error=lambda *args: None),
            }
            for axis in range(1, 7):
                runtime[f"J{axis}axisLimNeg"] = 0.0
                runtime[f"J{axis}StepM"] = 0
                calibration[f"J{axis}StepDeg"] = 1.0
                namespace[f"J{axis}StepLim"] = 100
            return namespace, stop_event

        joint_namespace, joint_stop = runtime_namespace("joint")
        joint_worker = self.compile_function(
            "live_joint_jog",
            joint_namespace,
        )
        self.assertTrue(joint_worker(
            "LJV11Sp37.5Ac12.5Dc23.5Rm34.5WALm000000\n",
            joint_stop,
        ))

        cartesian_namespace, cartesian_stop = runtime_namespace("cartesian")
        cartesian_namespace["_solve_inverse_kinematics"] = (
            lambda target, estimate, wrist: [0.0] * 6
        )
        cartesian_worker = self.compile_function(
            "live_cartesian_jog",
            cartesian_namespace,
        )
        self.assertTrue(cartesian_worker(
            "LCV11Sp37.5Ac12.5Dc23.5Rm34.5WNLm000000\n",
            cartesian_stop,
        ))

        class Robot:
            @staticmethod
            def forward_kinematics(angles):
                return [0.0] * 6

            @staticmethod
            def set_robot_tool_frame(*values):
                return None

        tool_namespace, tool_stop = runtime_namespace("tool")
        tool_namespace.update({
            "robot": Robot(),
            "_solve_inverse_kinematics": (
                lambda target, estimate, wrist: [0.0] * 6
            ),
        })
        tool_worker = self.compile_function(
            "live_tool_jog",
            tool_namespace,
        )
        self.assertTrue(tool_worker(
            "LTV10Sp37.5Ac12.5Dc23.5Rm34.5WNLm000000\n",
            (0.0,) * 6,
            tool_stop,
        ))

        self.assertEqual(
            captured_profiles,
            {
                "joint": expected_profile,
                "cartesian": expected_profile,
                "tool": expected_profile,
            },
        )

        profile_parser = self.compile_function(
            "_parse_live_jog_drive_profile",
            {"parse_command_timing": parse_command_timing},
        )
        for opcode, wrist in (("LJ", "A"), ("LC", "N"), ("LT", "N")):
            for mode in ("s", "m"):
                with self.subTest(opcode=opcode, mode=mode):
                    with self.assertRaises(MotionInputError):
                        profile_parser(
                            f"{opcode}V11S{mode}37.5Ac12.5Dc23.5Rm34.5"
                            f"W{wrist}Lm000000\n",
                            opcode,
                        )

    def test_live_tool_jog_preserves_native_rotation_order(self):
        class Robot:
            def __init__(self):
                self.tool_frames = []

            @staticmethod
            def forward_kinematics(angles):
                return [0.0] * 6

            def set_robot_tool_frame(self, *values):
                self.tool_frames.append(values)

            @staticmethod
            def SolveInverseKinematicsConfigured(*args):
                return None

        class Entry:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        captured_frames = []
        labels = SimpleNamespace(config=lambda **kwargs: None)
        fields = {
            "TFxEntryField": Entry(1),
            "TFyEntryField": Entry(2),
            "TFzEntryField": Entry(3),
            "TFrxEntryField": Entry(4),
            "TFryEntryField": Entry(5),
            "TFrzEntryField": Entry(6),
        }
        namespace = {
            **fields,
            "RUN": {"offlineMode": True},
            "CAL": {
                "TFx": 10,
                "TFy": 20,
                "TFz": 30,
                "TFrx": 40,
                "TFry": 50,
                "TFrz": 60,
            },
            "_live_jog_start_is_blocked": lambda: False,
            "_prepare_live_jog": (
                lambda *args: (40, "Sp", "10", "10", "10", "10", "000000")
            ),
            "LIVE_CARTESIAN_VECTORS": frozenset({40}),
            "finite_number": finite_number,
            "tk": SimpleNamespace(TclError=RuntimeError),
            "start_live_tool_jog_thread": (
                lambda command, frame: captured_frames.append(frame) or True
            ),
            "_dispatch_live_jog": lambda *args: True,
            "almStatusLab": labels,
            "almStatusLab2": labels,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
            ),
        }
        start_live_tool = self.compile_function("LiveToolJog", namespace)

        self.assertTrue(start_live_tool(40))
        self.assertEqual(
            captured_frames,
            [(10.0, 20.0, 30.0, 40.0, 50.0, 60.0)],
        )

        robot = Robot()
        worker_namespace = {
            "RUN": {
                "VR_angles": [0.0] * 6,
                "offlineMode": False,
                "Alarm": "0",
            },
            "robot": robot,
            "finite_number": finite_number,
            "math": math,
            "_queue_virtual_motion_error": lambda error: None,
            "logger": SimpleNamespace(error=lambda *args: None),
        }
        live_worker = self.compile_function("live_tool_jog", worker_namespace)

        original = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
        expected_frames = {
            10: (10.25, 20.0, 30.0, 40.0, 50.0, 60.0),
            11: (9.75, 20.0, 30.0, 40.0, 50.0, 60.0),
            20: (10.0, 20.25, 30.0, 40.0, 50.0, 60.0),
            21: (10.0, 19.75, 30.0, 40.0, 50.0, 60.0),
            30: (10.0, 20.0, 30.25, 40.0, 50.0, 60.0),
            31: (10.0, 20.0, 29.75, 40.0, 50.0, 60.0),
            40: (10.0, 20.0, 30.0, 40.0, 50.0, 60.25),
            41: (10.0, 20.0, 30.0, 40.0, 50.0, 59.75),
            50: (10.0, 20.0, 30.0, 40.0, 50.25, 60.0),
            51: (10.0, 20.0, 30.0, 40.0, 49.75, 60.0),
            60: (10.0, 20.0, 30.0, 40.25, 50.0, 60.0),
            61: (10.0, 20.0, 30.0, 39.75, 50.0, 60.0),
        }
        for vector, expected in expected_frames.items():
            with self.subTest(vector=vector):
                robot.tool_frames.clear()
                self.assertFalse(
                    live_worker(
                        f"LTV{vector}Sp10Ac10Dc10Rm10WALm000000\n",
                        original,
                        threading.Event(),
                    )
                )
                self.assertEqual(robot.tool_frames, [expected, original])

    def test_update_parameters_rejects_late_invalid_input_before_mutation(self):
        values = self._valid_update_parameter_values()
        values["J6aDHpar"] = "invalid"
        calibration = {"sentinel": "unchanged"}
        native_calls = []
        widget_calls = []
        namespace = {
            "CAL": calibration,
            "ControllerJointCalibration": ControllerJointCalibration,
            "MotionInputError": MotionInputError,
            "controller_ratio": controller_ratio,
            "finite_number": finite_number,
            "_collect_update_parameter_values": lambda: dict(values),
            "_set_cpp_kinematics_from_values": (
                lambda prepared: native_calls.append(prepared) or True
            ),
            "_apply_joint_limit_widgets": (
                lambda prepared: widget_calls.append(prepared) or True
            ),
        }
        self.add_startup_command_dependencies(namespace)
        namespace["_binary_controller_flag"] = self.compile_function(
            "_binary_controller_flag",
            namespace,
        )
        namespace["_robot_joint_calibration_from_values"] = self.compile_function(
            "_robot_joint_calibration_from_values",
            namespace,
        )
        namespace["_prepare_update_parameters_from_values"] = (
            self.compile_function(
                "_prepare_update_parameters_from_values",
                namespace,
            )
        )
        namespace["_prepare_update_parameters"] = self.compile_function(
            "_prepare_update_parameters",
            namespace,
        )
        namespace["_apply_update_parameter_values"] = self.compile_function(
            "_apply_update_parameter_values",
            namespace,
        )
        update_parameters = self.compile_function("updateParams", namespace)

        with self.assertRaisesRegex(MotionInputError, "UP field"):
            update_parameters(transmit=False)

        self.assertEqual(calibration, {"sentinel": "unchanged"})
        self.assertEqual(native_calls, [])
        self.assertEqual(widget_calls, [])

    def test_transmitting_calibration_preflights_before_local_mutation(self):
        for function_name, prepare_name, apply_name, transmit_name in (
            (
                "updateParams",
                "_prepare_update_parameters",
                "_apply_update_parameter_values",
                "_transmit_update_parameters",
            ),
            (
                "calExtAxis",
                "_prepare_external_axis_parameters",
                "_apply_external_axis_values",
                "_transmit_external_axis_parameters",
            ),
        ):
            with self.subTest(function=function_name):
                calibration = {"sentinel": "unchanged"}
                calls = []
                namespace = {
                    "CAL": calibration,
                    prepare_name: lambda: ({"sentinel": "new"}, "UPA1\n"),
                    apply_name: lambda values: calls.append("apply") or True,
                    transmit_name: (
                        lambda command, write_started_event: (
                            calls.append("transmit") or True
                        )
                    ),
                    "_preflight_controller_calibration_transport": (
                        lambda: (_ for _ in ()).throw(
                            ConnectionError("controller unavailable")
                        )
                    ),
                    "threading": threading,
                    "logger": SimpleNamespace(
                        error=lambda *args: None,
                        exception=lambda *args: None,
                    ),
                }
                namespace["_apply_single_calibration_transaction"] = (
                    self.compile_function(
                        "_apply_single_calibration_transaction",
                        namespace,
                    )
                )
                function = self.compile_function(function_name, namespace)

                self.assertFalse(function())
                self.assertEqual(calibration, {"sentinel": "unchanged"})
                self.assertEqual(calls, [])

    def test_update_parameters_rejects_nonbinary_direction_flags(self):
        values = self._valid_update_parameter_values()
        namespace = {
            "ControllerJointCalibration": ControllerJointCalibration,
            "MotionInputError": MotionInputError,
            "controller_ratio": controller_ratio,
            "finite_number": finite_number,
            "_collect_update_parameter_values": lambda: dict(values),
        }
        self.add_startup_command_dependencies(namespace)
        namespace["_binary_controller_flag"] = self.compile_function(
            "_binary_controller_flag",
            namespace,
        )
        namespace["_robot_joint_calibration_from_values"] = self.compile_function(
            "_robot_joint_calibration_from_values",
            namespace,
        )
        namespace["_prepare_update_parameters_from_values"] = (
            self.compile_function(
                "_prepare_update_parameters_from_values",
                namespace,
            )
        )
        prepare = self.compile_function("_prepare_update_parameters", namespace)

        for key, invalid in (
            ("J1MotDir", 2),
            ("J5CalDir", -1),
            ("J9MotDir", float("nan")),
        ):
            with self.subTest(key=key, invalid=invalid):
                original = values[key]
                values[key] = invalid
                try:
                    with self.assertRaises(MotionInputError):
                        prepare()
                finally:
                    values[key] = original

    def test_calibration_command_validates_selections_and_offsets(self):
        calibration = {
            f"J{axis}calOff": axis / 10
            for axis in range(1, 10)
        }
        namespace = {
            "CAL": calibration,
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
        }
        self.add_startup_command_dependencies(namespace)
        namespace["_binary_controller_flag"] = self.compile_function(
            "_binary_controller_flag",
            namespace,
        )
        prepare = self.compile_function("_prepare_calibration_command", namespace)

        command = prepare((1, 0, 0, 0, 0, 0, 0, 0, 0))
        self.assertTrue(command.startswith("LLA1B0C0D0E0F0G0H0I0"))
        self.assertTrue(command.endswith("R0.89999997615814208984375\n"))

        for invalid in (2, -1, float("nan"), True):
            with self.subTest(selection=invalid):
                selections = [0] * 9
                selections[4] = invalid
                with self.assertRaises(MotionInputError):
                    prepare(selections)

        calibration["J7calOff"] = float("inf")
        with self.assertRaisesRegex(MotionInputError, "J7 calibration offset"):
            prepare((0,) * 9)

    def test_calibration_field_collection_rejects_nonfinite_values(self):
        class Entry:
            def __init__(self, value="0"):
                self.value = value

            def get(self):
                return self.value

        namespace = {
            "finite_number": finite_number,
            "com1SelectedValue": Entry("COM1"),
            "com2SelectedValue": Entry("COM2"),
            "auxiliaryBoardSelectedValue": Entry(AUXILIARY_BOARD_NANO),
            "J7curAngEntryField": Entry(),
            "J8curAngEntryField": Entry(),
            "J9curAngEntryField": Entry(),
            "visoptions": Entry("camera"),
            "VisBrightSlide": Entry(),
            "VisContrastSlide": Entry(),
            "VisBacColorEntryField": Entry("black"),
            "VisScoreEntryField": Entry(),
            "VisX1PixEntryField": Entry(),
            "VisY1PixEntryField": Entry(),
            "VisX2PixEntryField": Entry(),
            "VisY2PixEntryField": Entry(),
            "VisX1RobEntryField": Entry(),
            "VisY1RobEntryField": Entry(),
            "VisX2RobEntryField": Entry(),
            "VisY2RobEntryField": Entry(),
            "VisZoomSlide": Entry(),
            "RUN": {
                "pick180": Entry(),
                "pickClosest": Entry(),
                "fullRot": Entry(),
                "autoBG": Entry(),
            },
            "_collect_external_axis_values": lambda: {},
        }
        for axis in range(1, 10):
            namespace[f"J{axis}calOffEntryField"] = Entry()
        for axis in range(1, 7):
            namespace[f"J{axis}DriveMSEntryField"] = Entry("16")
            namespace[f"J{axis}EncCPREntryField"] = Entry("4096")
        collect = self.compile_function("_collect_fields_to_calibration", namespace)
        self.assertEqual(
            collect()["auxiliaryBoard"],
            AUXILIARY_BOARD_NANO,
        )

        for field_name, invalid, expected in (
            ("J7curAngEntryField", "nan", "J7 current position"),
            ("J8curAngEntryField", "inf", "J8 current position"),
            ("J9curAngEntryField", "-inf", "J9 current position"),
            ("VisScoreEntryField", "nan", "vision score"),
            ("VisZoomSlide", "inf", "vision zoom"),
            ("J4calOffEntryField", "-inf", "J4 calibration offset"),
        ):
            with self.subTest(field=field_name):
                field = namespace[field_name]
                original = field.value
                field.value = invalid
                try:
                    with self.assertRaisesRegex(MotionInputError, expected):
                        collect()
                finally:
                    field.value = original

    def test_default_calibration_requires_explicit_auxiliary_board_selection(self):
        defaults = json.loads(
            (PROJECT_ROOT / "defaults.json").read_text(encoding="utf-8")
        )

        self.assertEqual(defaults["auxiliaryBoard"], AUXILIARY_BOARD_NONE)

    def test_custom_profile_validation_uses_active_pose_without_mutation(self):
        profile = self._valid_custom_calibration_profile()
        active_calibration = dict(profile)
        for axis in range(1, 7):
            active_calibration[f"J{axis}AngCur"] = 0.0
        for axis in range(7, 10):
            active_calibration[f"J{axis}PosCur"] = 0.0
        namespace = {"CAL": active_calibration}
        self.add_custom_profile_validation_dependencies(namespace)
        prepare = namespace["_prepare_custom_calibration_profile"]

        original = dict(active_calibration)
        prepared = prepare(dict(profile))

        self.assertEqual(active_calibration, original)
        self.assertEqual(prepared["J1DriveMS"], 16)
        self.assertEqual(prepared["J9steps"], 36000.0)

        active_calibration["J1AngCur"] = 20.0
        outside_active_pose = dict(profile)
        outside_active_pose["J1PosLim"] = 10.0
        with self.assertRaises(MotionInputError):
            prepare(outside_active_pose)
        self.assertEqual(active_calibration["J1AngCur"], 20.0)

        missing_external_field = dict(profile)
        del missing_external_field["J9steps"]
        with self.assertRaisesRegex(MotionInputError, "missing J9steps"):
            prepare(missing_external_field)

        largest_accepted_rotation = dict(profile)
        largest_accepted_rotation["TFrz"] = float.fromhex("0x1.fffffcp+127")
        self.assertEqual(
            prepare(largest_accepted_rotation)["TFrz"],
            largest_accepted_rotation["TFrz"],
        )

        for invalid, message in (
            (1e-44, "TFrz native radians"),
            (float.fromhex("0x1.fffffep+127"), "TFrz native degrees"),
        ):
            with self.subTest(rotation=invalid):
                invalid_rotation = dict(profile)
                invalid_rotation["TFrz"] = invalid
                with self.assertRaisesRegex(MotionInputError, message):
                    prepare(invalid_rotation)

    def test_rotation_contract_preflights_startup_and_profile_save(self):
        active_calibration = self._valid_custom_calibration_profile()
        for axis in range(1, 7):
            active_calibration[f"J{axis}AngCur"] = 0.0
        for axis in range(7, 10):
            active_calibration[f"J{axis}PosCur"] = 0.0

        class Robot:
            @staticmethod
            def set_robot_configuration(*args):
                raise AssertionError("startup preflight invoked the native writer")

            @staticmethod
            def SolveInverseKinematicsConfigured(*args):
                return None

        startup_namespace = {
            "CAL": active_calibration,
            "robot": Robot(),
            "com2SelectedValue": SimpleNamespace(get=lambda: "COM2"),
            "auxiliaryBoardSelectedValue": SimpleNamespace(
                get=lambda: AUXILIARY_BOARD_NANO
            ),
            "_prepare_position_command": lambda values: "SPA0\n",
            "ControllerStartupRequest": SimpleNamespace,
        }
        prepare_startup = self.compile_function(
            "_prepare_controller_startup",
            startup_namespace,
        )

        profile_namespace = {"CAL": active_calibration}
        self.add_custom_profile_validation_dependencies(profile_namespace)
        prepare_update = profile_namespace[
            "_prepare_update_parameters_from_values"
        ]

        for invalid, message in (
            (1e-44, "TFrz native radians"),
            (float.fromhex("0x1.fffffep+127"), "TFrz native degrees"),
        ):
            with self.subTest(rotation=invalid):
                update_values = self._valid_update_parameter_values()
                update_values["TFrz"] = invalid
                startup_namespace["_prepare_controller_calibration"] = (
                    lambda values=update_values: (
                        values,
                        "UPA0\n",
                        {},
                        "CEA0\n",
                    )
                )
                with self.assertRaisesRegex(MotionInputError, message):
                    prepare_startup()

                persistence_calls = []
                save_namespace = {
                    "CAL": active_calibration,
                    "_collect_fields_to_calibration": lambda: {},
                    "_prepare_controller_calibration": (
                        lambda values=update_values: (
                            prepare_update(values)[0],
                            "UPA0\n",
                            {},
                            "CEA0\n",
                        )
                    ),
                    "_validate_controller_pose": lambda values: True,
                    "save_calibration": (
                        lambda **kwargs: persistence_calls.append(kwargs) or True
                    ),
                    "logger": SimpleNamespace(exception=lambda *args: None),
                }
                save_namespace["_prepare_custom_calibration_snapshot"] = (
                    self.compile_function(
                        "_prepare_custom_calibration_snapshot",
                        save_namespace,
                    )
                )
                save_profile = self.compile_function(
                    "save_custom_calibration",
                    save_namespace,
                )

                self.assertFalse(save_profile())
                self.assertEqual(persistence_calls, [])

    def test_custom_profile_sync_stages_all_controller_fields_only(self):
        class Entry:
            def __init__(self, value):
                self.value = value
                self.deletions = []
                self.insertions = []

            def delete(self, start, end):
                self.deletions.append((start, end))
                self.value = ""

            def insert(self, index, value):
                self.insertions.append((index, value))
                self.value = value

        binding_function = self.module_functions[
            "_custom_calibration_field_bindings"
        ]
        field_names = {
            node.id
            for node in ast.walk(binding_function)
            if isinstance(node, ast.Name) and node.id.endswith("EntryField")
        }
        active_calibration = {"sentinel": "active"}
        namespace = {
            "CAL": active_calibration,
            "MotionInputError": MotionInputError,
        }
        namespace.update({name: Entry(name) for name in field_names})
        namespace["_custom_calibration_profile_keys"] = self.compile_function(
            "_custom_calibration_profile_keys",
            namespace,
        )
        namespace["_custom_calibration_field_values"] = self.compile_function(
            "_custom_calibration_field_values",
            namespace,
        )
        namespace["_custom_calibration_field_bindings"] = self.compile_function(
            "_custom_calibration_field_bindings",
            namespace,
        )
        synchronize = self.compile_function(
            "sync_calibration_to_fields",
            namespace,
        )

        profile = self._valid_custom_calibration_profile()
        self.assertTrue(synchronize(profile))

        self.assertEqual(active_calibration, {"sentinel": "active"})
        self.assertEqual(namespace["TFxEntryField"].value, str(profile["TFx"]))
        self.assertEqual(
            namespace["axis9stepsEntryField"].value,
            str(profile["J9steps"]),
        )
        self.assertEqual(
            namespace["J9calOffEntryField"].value,
            str(profile["J9calOff"]),
        )
        self.assertTrue(
            all(field.deletions == [(0, "end")] for field in namespace.values()
                if isinstance(field, Entry))
        )
        self.assertTrue(
            all(len(field.insertions) == 1 for field in namespace.values()
                if isinstance(field, Entry))
        )

    def test_custom_profile_callbacks_preserve_active_calibration(self):
        active_calibration = {"sentinel": "active"}
        loaded_profile = {"profile": "loaded"}
        prepared_profile = {"J1DriveMS": 16}
        calls = []
        logger = SimpleNamespace(
            debug=lambda *args: calls.append(("debug", args)),
            error=lambda *args: calls.append(("error", args)),
            exception=lambda *args: calls.append(("exception", args)),
        )
        load_namespace = {
            "CAL": active_calibration,
            "load_calibration": (
                lambda **kwargs: calls.append(("load", kwargs))
                or loaded_profile
            ),
            "_prepare_custom_calibration_profile": (
                lambda values: calls.append(("prepare", values))
                or prepared_profile
            ),
            "sync_calibration_to_fields": (
                lambda values: calls.append(("sync", values)) or True
            ),
            "logger": logger,
        }
        load_custom = self.compile_function(
            "load_custom_calibration",
            load_namespace,
        )

        self.assertTrue(load_custom())
        self.assertEqual(active_calibration, {"sentinel": "active"})
        self.assertIn(
            (
                "load",
                {
                    "calibration_file": "custom.json",
                    "allow_fallback": False,
                },
            ),
            calls,
        )
        self.assertIn(("prepare", loaded_profile), calls)
        self.assertIn(("sync", prepared_profile), calls)

        saved_profile = {"profile": "saved"}
        save_calls = []
        save_namespace = {
            "CAL": active_calibration,
            "_prepare_custom_calibration_snapshot": lambda: saved_profile,
            "save_calibration": (
                lambda **kwargs: save_calls.append(kwargs) or True
            ),
            "logger": logger,
        }
        save_custom = self.compile_function(
            "save_custom_calibration",
            save_namespace,
        )

        self.assertTrue(save_custom())
        self.assertEqual(active_calibration, {"sentinel": "active"})
        self.assertEqual(
            save_calls,
            [{
                "calibration_file": "custom.json",
                "calibration_data": saved_profile,
            }],
        )

    def test_custom_profile_loader_does_not_fall_back_to_defaults(self):
        calibration_tree = ast.parse(
            CALIBRATION_SOURCE.read_text(encoding="utf-8"),
            filename=str(CALIBRATION_SOURCE),
        )
        function = next(
            node
            for node in calibration_tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "load_calibration"
        )
        namespace = {
            "os": SimpleNamespace(
                path=SimpleNamespace(
                    exists=lambda filename: filename == "defaults.json"
                )
            ),
            "json": json,
            "logger": SimpleNamespace(
                debug=lambda *args: None,
                error=lambda *args: None,
                info=lambda *args: None,
            ),
            "convert_calibration": lambda: {"fallback": "legacy"},
            "save_calibration": lambda values: True,
            "open": lambda filename, mode: io.StringIO(
                '{"fallback": "defaults"}'
            ),
        }
        module = ast.Module(body=[copy.deepcopy(function)], type_ignores=[])
        compiled = compile(
            ast.fix_missing_locations(module),
            str(CALIBRATION_SOURCE),
            "exec",
        )
        exec(compiled, namespace)
        load_calibration_file = namespace["load_calibration"]

        self.assertIsNone(
            load_calibration_file(
                calibration_file="custom.json",
                allow_fallback=False,
            )
        )
        self.assertEqual(
            load_calibration_file(calibration_file="custom.json"),
            {"fallback": "defaults"},
        )

        def corrupt_json(source_file):
            raise json.JSONDecodeError("invalid calibration", "{", 1)

        namespace["os"].path.exists = lambda filename: filename == "custom.json"
        namespace["json"] = SimpleNamespace(load=corrupt_json)
        self.assertIsNone(
            load_calibration_file(
                calibration_file="custom.json",
                allow_fallback=False,
            )
        )

        namespace["json"] = SimpleNamespace(load=lambda source_file: [])
        self.assertIsNone(
            load_calibration_file(
                calibration_file="custom.json",
                allow_fallback=False,
            )
        )

    def test_startup_calibration_rejects_loader_failure_before_application(self):
        startup_assignment = next(
            node
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "loaded_calibration"
                for target in node.targets
            )
        )
        self.assertIsInstance(startup_assignment.value, ast.Call)
        self.assertIsInstance(startup_assignment.value.func, ast.Name)
        self.assertEqual(
            startup_assignment.value.func.id,
            "_load_startup_calibration",
        )

        namespace = {"load_calibration": lambda: None}
        load_startup = self.compile_function(
            "_load_startup_calibration",
            namespace,
        )
        with self.assertRaisesRegex(
            RuntimeError,
            "controller motion remains disabled",
        ):
            load_startup()

        calibration_values = {"J1AngCur": 0.0}
        load_startup.__globals__["load_calibration"] = (
            lambda: calibration_values
        )
        self.assertIs(load_startup(), calibration_values)

    def test_save_and_custom_profile_reject_external_pose_before_mutation(self):
        def valid_calibration():
            values = {}
            for axis in range(1, 7):
                values.update(
                    {
                        f"J{axis}AngCur": 0.0,
                        f"J{axis}NegLim": 180.0,
                        f"J{axis}PosLim": 180.0,
                        f"J{axis}StepDeg": 100.0,
                    }
                )
            values.update(
                {
                    "J7PosCur": 0.0,
                    "J8PosCur": 0.0,
                    "J9PosCur": 0.0,
                    "J7PosLim": 10.0,
                    "J8length": 20.0,
                    "J9length": 30.0,
                    "J7rotation": 1.0,
                    "J8rotation": 1.0,
                    "J9rotation": 1.0,
                    "J7steps": 100.0,
                    "J8steps": 100.0,
                    "J9steps": 100.0,
                }
            )
            return values

        for field_name, invalid in (
            ("J7PosCur", float("nan")),
            ("J8PosCur", 21.0),
            ("J9PosCur", -1.0),
        ):
            with self.subTest(field=field_name, invalid=invalid):
                calibration = valid_calibration()
                original = dict(calibration)
                boundary_calls = []
                namespace = {
                    "CAL": calibration,
                    "ControllerJointCalibration": ControllerJointCalibration,
                    "MotionInputError": MotionInputError,
                    "controller_ratio": controller_ratio,
                    "finite_number": finite_number,
                    "_collect_fields_to_calibration": lambda: {
                        field_name: invalid
                    },
                    "_prepare_controller_calibration": lambda: (
                        {},
                        "UPA1\n",
                        {},
                        "CEA1\n",
                    ),
                    "_apply_controller_calibration": (
                        lambda *args: boundary_calls.append("apply") or True
                    ),
                    "_transmit_update_parameters": (
                        lambda *args: boundary_calls.append("update") or True
                    ),
                    "_transmit_external_axis_parameters": (
                        lambda *args: boundary_calls.append("external") or True
                    ),
                    "save_calibration": (
                        lambda *args, **kwargs: boundary_calls.append("save") or True
                    ),
                    "logger": SimpleNamespace(
                        error=lambda *args: None,
                        exception=lambda *args: None,
                    ),
                }
                namespace["_controller_joint_positions_from_values"] = (
                    self.compile_function(
                        "_controller_joint_positions_from_values",
                        namespace,
                    )
                )
                namespace["_controller_joint_calibration_from_values"] = (
                    self.compile_function(
                        "_controller_joint_calibration_from_values",
                        namespace,
                    )
                )
                namespace["_validate_controller_pose"] = self.compile_function(
                    "_validate_controller_pose",
                    namespace,
                )
                self.add_save_and_apply_dependencies(namespace)
                save_and_apply = self.compile_function(
                    "SaveAndApplyCalibration",
                    namespace,
                )
                namespace["_prepare_custom_calibration_snapshot"] = (
                    self.compile_function(
                        "_prepare_custom_calibration_snapshot",
                        namespace,
                    )
                )
                save_custom = self.compile_function(
                    "save_custom_calibration",
                    namespace,
                )

                self.assertFalse(save_and_apply())
                self.assertEqual(calibration, original)
                self.assertEqual(boundary_calls, [])
                self.assertFalse(save_custom())
                self.assertEqual(calibration, original)
                self.assertEqual(boundary_calls, [])

    def test_external_axis_rejects_invalid_rotation_before_mutation(self):
        calibration = self._valid_update_parameter_values()
        calibration.update(
            {
                "J7PosLim": 700,
                "J7rotation": 1,
                "J7steps": 100,
                "J8length": 800,
                "J8rotation": 1,
                "J8steps": 100,
                "J9length": 900,
                "J9rotation": 1,
                "J9steps": 100,
            }
        )
        original = dict(calibration)
        invalid_external = {
            "J7PosLim": 70,
            "J7rotation": 1,
            "J7steps": 100,
            "J8length": 80,
            "J8rotation": 1,
            "J8steps": 100,
            "J9length": 90,
            "J9rotation": 0,
            "J9steps": 100,
        }

        class Widget:
            def __init__(self):
                self.configurations = []

            def config(self, **kwargs):
                self.configurations.append(kwargs)

        widgets = [Widget() for _ in range(9)]
        namespace = {
            "CAL": calibration,
            "ControllerJointCalibration": ControllerJointCalibration,
            "MotionInputError": MotionInputError,
            "controller_ratio": controller_ratio,
            "finite_number": finite_number,
            "_collect_external_axis_values": lambda: dict(invalid_external),
            "HORIZONTAL": "horizontal",
            "J7negLimLab": widgets[0],
            "J8negLimLab": widgets[1],
            "J9negLimLab": widgets[2],
            "J7posLimLab": widgets[3],
            "J8posLimLab": widgets[4],
            "J9posLimLab": widgets[5],
            "J7jogslide": widgets[6],
            "J8jogslide": widgets[7],
            "J9jogslide": widgets[8],
            "J7sliderUpdate": lambda value: None,
            "J8sliderUpdate": lambda value: None,
            "J9sliderUpdate": lambda value: None,
        }
        self.add_startup_command_dependencies(namespace)
        namespace["_controller_joint_calibration_from_values"] = (
            self.compile_function(
                "_controller_joint_calibration_from_values",
                namespace,
            )
        )
        namespace["_prepare_external_axis_parameters_from_values"] = (
            self.compile_function(
                "_prepare_external_axis_parameters_from_values",
                namespace,
            )
        )
        namespace["_prepare_external_axis_parameters"] = self.compile_function(
            "_prepare_external_axis_parameters",
            namespace,
        )
        namespace["_apply_external_axis_values"] = self.compile_function(
            "_apply_external_axis_values",
            namespace,
        )
        calibrate_external = self.compile_function("calExtAxis", namespace)

        with self.assertRaisesRegex(MotionInputError, "J9 rotation"):
            calibrate_external(transmit=False)

        self.assertEqual(calibration, original)
        self.assertTrue(all(not widget.configurations for widget in widgets))

    def test_calibration_transmissions_require_exact_firmware_acknowledgement(self):
        port = SimpleNamespace(is_open=True)
        exchange_calls = []
        write_lock = object()
        helper_namespace = {
            "RUN": {"ser": port},
            "serial_write_lock": write_lock,
            "SERIAL_STARTUP_READ_TIMEOUT_SECONDS": 10,
            "write_serial_control": (
                lambda serial_port, command, write_lock=None, reset_input=False,
                write_started_event=None: (
                    exchange_calls.append(
                        (
                            "write",
                            serial_port,
                            command,
                            write_lock,
                            reset_input,
                            write_started_event,
                        )
                    )
                    or True
                )
            ),
            "read_serial_exact_response": (
                lambda serial_port, expected, timeout: (
                    exchange_calls.append(
                        ("read", serial_port, expected, timeout)
                    )
                    or "Done"
                )
            ),
            "serial_transport_quarantined": lambda serial_port: False,
        }
        exchange_acknowledgement = self.compile_function(
            "_exchange_controller_calibration_acknowledgement",
            helper_namespace,
        )

        self.assertTrue(exchange_acknowledgement("UPA1\n"))
        self.assertEqual(
            exchange_calls,
            [
                ("write", port, "UPA1\n", write_lock, True, None),
                ("read", port, b"Done", 10),
            ],
        )

        calls = []

        def exchange(command, write_started_event=None):
            calls.append((command, write_started_event))
            return True

        namespace = {
            "_exchange_controller_calibration_acknowledgement": exchange,
        }
        transmit_update = self.compile_function(
            "_transmit_update_parameters",
            namespace,
        )
        transmit_external = self.compile_function(
            "_transmit_external_axis_parameters",
            namespace,
        )

        self.assertTrue(transmit_update("UPA1\n"))
        self.assertTrue(transmit_external("CEA2\n"))
        self.assertEqual(
            calls,
            [("UPA1\n", None), ("CEA2\n", None)],
        )

    def test_save_and_apply_does_not_persist_failed_validation(self):
        calibration = {"sentinel": "unchanged"}
        calls = []

        def reject_calibration():
            raise MotionInputError("invalid calibration")

        namespace = {
            "CAL": calibration,
            "MotionInputError": MotionInputError,
            "_collect_fields_to_calibration": lambda: {"sentinel": "new"},
            "_prepare_controller_calibration": reject_calibration,
            "_apply_controller_calibration": (
                lambda *args: calls.append("apply") or True
            ),
            "_transmit_update_parameters": (
                lambda command: calls.append("update") or True
            ),
            "_transmit_external_axis_parameters": (
                lambda command: calls.append("external") or True
            ),
            "save_calibration": (
                lambda prepared: calls.append("save") or True
            ),
            "_restore_controller_calibration": (
                lambda snapshot: calls.append("restore") or True
            ),
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        self.add_save_and_apply_dependencies(namespace)
        save_and_apply = self.compile_function(
            "SaveAndApplyCalibration",
            namespace,
        )

        self.assertFalse(save_and_apply())
        self.assertEqual(calibration, {"sentinel": "unchanged"})
        self.assertEqual(calls, [])

    def test_save_and_apply_rolls_back_prewrite_transmission_failure(self):
        calibration = {"sentinel": "unchanged"}
        calls = []

        def apply_calibration(*args):
            calls.append("apply")
            calibration["sentinel"] = "applied"
            return True

        def restore(snapshot):
            calls.append("restore")
            calibration.clear()
            calibration.update(snapshot)
            return True

        def reject_before_write(command, write_started_event):
            calls.append("update")
            raise SerialActivityRejected("controller unavailable before write")

        namespace = {
            "CAL": calibration,
            "SerialActivityRejected": SerialActivityRejected,
            "_collect_fields_to_calibration": lambda: {"sentinel": "field"},
            "_prepare_controller_calibration": lambda: (
                {"update": 1},
                "UPA1\n",
                {"external": 2},
                "CEA2\n",
            ),
            "_apply_controller_calibration": apply_calibration,
            "_transmit_update_parameters": reject_before_write,
            "_transmit_external_axis_parameters": (
                lambda command, write_started_event: (
                    write_started_event.set()
                    or calls.append("external")
                    or True
                )
            ),
            "save_calibration": (
                lambda prepared: calls.append("save") or True
            ),
            "_restore_controller_calibration": restore,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        self.add_save_and_apply_dependencies(namespace)
        save_and_apply = self.compile_function(
            "SaveAndApplyCalibration",
            namespace,
        )

        self.assertFalse(save_and_apply())
        self.assertEqual(calibration, {"sentinel": "unchanged"})
        self.assertEqual(calls, ["apply", "update", "restore"])

    def test_save_and_apply_retains_state_after_partial_controller_apply(self):
        calibration = {"sentinel": "unchanged"}
        calls = []

        def apply_calibration(*args):
            calls.append("apply")
            calibration["sentinel"] = "applied"
            return True

        def invalidate(reason):
            calls.append(("invalidate", reason))
            return False

        def acknowledge(command, write_started_event):
            write_started_event.set()
            calls.append("update")
            return True

        def reject_external(command, write_started_event):
            write_started_event.set()
            calls.append("external")
            return False

        namespace = {
            "CAL": calibration,
            "_collect_fields_to_calibration": lambda: {"sentinel": "field"},
            "_prepare_controller_calibration": lambda: (
                {"update": 1},
                "UPA1\n",
                {"external": 2},
                "CEA2\n",
            ),
            "_apply_controller_calibration": apply_calibration,
            "_transmit_update_parameters": acknowledge,
            "_transmit_external_axis_parameters": reject_external,
            "_restore_controller_calibration": (
                lambda snapshot: calls.append("restore") or True
            ),
            "_invalidate_uncertain_controller_calibration": invalidate,
            "save_calibration": lambda prepared: calls.append("save") or True,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        self.add_save_and_apply_dependencies(namespace)
        save_and_apply = self.compile_function(
            "SaveAndApplyCalibration",
            namespace,
        )

        self.assertFalse(save_and_apply())
        self.assertEqual(calibration, {"sentinel": "applied"})
        self.assertEqual(calls[:3], ["apply", "update", "external"])
        self.assertEqual(calls[3][0], "invalidate")
        self.assertNotIn("restore", calls)
        self.assertNotIn("save", calls)

    def test_save_and_apply_quarantines_uncertain_first_transmission(self):
        calibration = {"sentinel": "unchanged"}
        calls = []

        def apply_calibration(*args):
            calls.append("apply")
            calibration["sentinel"] = "applied"
            return True

        def fail_transmission(command, write_started_event):
            write_started_event.set()
            calls.append("update")
            raise OSError("acknowledgement lost")

        namespace = {
            "CAL": calibration,
            "_collect_fields_to_calibration": lambda: {"sentinel": "field"},
            "_prepare_controller_calibration": lambda: (
                {"update": 1},
                "UPA1\n",
                {"external": 2},
                "CEA2\n",
            ),
            "_apply_controller_calibration": apply_calibration,
            "_transmit_update_parameters": fail_transmission,
            "_transmit_external_axis_parameters": (
                lambda command, write_started_event: (
                    write_started_event.set()
                    or calls.append("external")
                    or True
                )
            ),
            "_restore_controller_calibration": (
                lambda snapshot: calls.append("restore") or True
            ),
            "_invalidate_uncertain_controller_calibration": (
                lambda reason: calls.append(("invalidate", reason)) or False
            ),
            "save_calibration": lambda prepared: calls.append("save") or True,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        self.add_save_and_apply_dependencies(namespace)
        save_and_apply = self.compile_function(
            "SaveAndApplyCalibration",
            namespace,
        )

        self.assertFalse(save_and_apply())
        self.assertEqual(calibration, {"sentinel": "applied"})
        self.assertEqual(calls[:2], ["apply", "update"])
        self.assertEqual(calls[2][0], "invalidate")
        self.assertNotIn("restore", calls)
        self.assertNotIn("external", calls)
        self.assertNotIn("save", calls)

    def test_save_and_apply_invalidates_state_when_local_rollback_fails(self):
        calibration = {"sentinel": "unchanged"}
        calls = []

        def fail_application(*args):
            calibration["sentinel"] = "partially-applied"
            raise RuntimeError("native application failed")

        namespace = {
            "CAL": calibration,
            "_collect_fields_to_calibration": lambda: {"sentinel": "field"},
            "_prepare_controller_calibration": lambda: (
                {"update": 1},
                "UPA1\n",
                {"external": 2},
                "CEA2\n",
            ),
            "_apply_controller_calibration": fail_application,
            "_restore_controller_calibration": (
                lambda snapshot: (_ for _ in ()).throw(
                    RuntimeError("local rollback failed")
                )
            ),
            "_invalidate_uncertain_controller_calibration": (
                lambda reason: calls.append(("invalidate", reason)) or False
            ),
            "_transmit_update_parameters": (
                lambda command: calls.append("update") or True
            ),
            "_transmit_external_axis_parameters": (
                lambda command: calls.append("external") or True
            ),
            "save_calibration": lambda prepared: calls.append("save") or True,
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        self.add_save_and_apply_dependencies(namespace)
        save_and_apply = self.compile_function(
            "SaveAndApplyCalibration",
            namespace,
        )

        self.assertFalse(save_and_apply())
        self.assertEqual(calls[0][0], "invalidate")
        self.assertIn("local calibration rollback failed", calls[0][1])
        self.assertNotIn("update", calls)
        self.assertNotIn("external", calls)
        self.assertNotIn("save", calls)

    def test_uncertain_calibration_detaches_and_invalidates_controller_state(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.close_count = 0

            def close(self):
                self.close_count += 1
                self.is_open = False

        port = Port()
        resynchronization_required = threading.Event()

        class Dispatcher:
            def __init__(self):
                self.invalidations = []

            def invalidate(self, reason):
                self.invalidations.append(reason)
                return False

        dispatcher = Dispatcher()
        statuses = []
        namespace = {
            "RUN": {"ser": port},
            "quarantine_serial_transport": quarantine_serial_transport,
            "joint_motion_dispatcher": dispatcher,
            "deferred_joint_adjustments": SimpleNamespace(pending=False),
            "_clear_deferred_joint_adjustments": lambda: None,
            "controller_position_resynchronization_required": (
                resynchronization_required
            ),
            "acknowledged_forced_position_lock": threading.Lock(),
            "acknowledged_forced_position_target": (0.0,) * 9,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(
                config=lambda **kwargs: statuses.append(kwargs)
            ),
            "almStatusLab2": SimpleNamespace(
                config=lambda **kwargs: statuses.append(kwargs)
            ),
        }
        namespace["_invalidate_joint_motion_state"] = self.compile_function(
            "_invalidate_joint_motion_state",
            namespace,
        )
        invalidate = self.compile_function(
            "_invalidate_uncertain_controller_calibration",
            namespace,
        )

        self.assertFalse(invalidate("controller calibration acknowledgement lost"))
        self.assertIsNone(namespace["RUN"]["ser"])
        self.assertEqual(port.close_count, 1)
        self.assertTrue(serial_transport_quarantined(port))
        self.assertEqual(
            dispatcher.invalidations,
            ["controller calibration acknowledgement lost"],
        )
        self.assertTrue(resynchronization_required.is_set())
        self.assertEqual(len(statuses), 2)
        self.assertTrue(
            all("reconnection is required" in status["text"] for status in statuses)
        )

    def test_save_and_apply_retains_acknowledged_state_for_persistence_retry(self):
        calibration = {"sentinel": "unchanged"}
        calls = []

        def apply_calibration(*args):
            calls.append("apply")
            calibration["sentinel"] = "applied"
            return True

        def restore(snapshot):
            calls.append("restore")
            calibration.clear()
            calibration.update(snapshot)
            return True

        def acknowledge_update(command, write_started_event):
            write_started_event.set()
            calls.append("update")
            return True

        def acknowledge_external(command, write_started_event):
            write_started_event.set()
            calls.append("external")
            return True

        namespace = {
            "CAL": calibration,
            "SerialActivityRejected": SerialActivityRejected,
            "_collect_fields_to_calibration": lambda: {"sentinel": "field"},
            "_prepare_controller_calibration": lambda: (
                {"update": 1},
                "UPA1\n",
                {"external": 2},
                "CEA2\n",
            ),
            "_apply_controller_calibration": apply_calibration,
            "_transmit_update_parameters": acknowledge_update,
            "_transmit_external_axis_parameters": acknowledge_external,
            "save_calibration": (
                lambda prepared: calls.append("save") or False
            ),
            "_restore_controller_calibration": restore,
            "_retain_calibration_persistence_retry": (
                lambda: calls.append("retry") or True
            ),
            "logger": SimpleNamespace(
                exception=lambda *args: None,
                error=lambda *args: None,
            ),
        }
        self.add_save_and_apply_dependencies(namespace)
        save_and_apply = self.compile_function(
            "SaveAndApplyCalibration",
            namespace,
        )

        self.assertFalse(save_and_apply())
        self.assertEqual(calibration, {"sentinel": "applied"})
        self.assertEqual(
            calls,
            ["apply", "update", "external", "save", "retry"],
        )

    def test_controller_startup_validates_saved_position_before_serialization(self):
        validated_positions = []

        class Calibration:
            @staticmethod
            def validate_positions(positions):
                validated_positions.append(positions)

        namespace = {
            "_current_controller_joint_calibration": lambda: Calibration(),
            "_current_joint_positions": lambda: tuple(range(9)),
            "_build_startup_numeric_command": lambda prefix, fields: (
                prefix + repr(tuple(fields)) + "\n"
            ),
        }
        namespace["_prepare_position_command"] = self.compile_function(
            "_prepare_position_command",
            namespace,
        )
        send_position = self.compile_function("sendPos", namespace)

        command = send_position(transmit=False)

        self.assertEqual(validated_positions, [tuple(range(9))])
        self.assertTrue(command.startswith("SP"))

    def test_controller_startup_stages_position_before_calibration_mutation(self):
        calibration = {"com2Port": "old"}
        apply_calls = []
        preflight_calls = []
        namespace = {
            "CAL": calibration,
            "com2SelectedValue": SimpleNamespace(get=lambda: "COM2"),
            "auxiliaryBoardSelectedValue": SimpleNamespace(
                get=lambda: AUXILIARY_BOARD_NANO
            ),
            "_prepare_controller_calibration": lambda: (
                {"update": 1},
                "UPA1\n",
                {"external": 2},
                "CEA2\n",
            ),
            "_prepare_position_command": (
                lambda values: (_ for _ in ()).throw(
                    MotionInputError("position is outside staged limits")
                )
            ),
            "_prepare_cpp_kinematics_configuration": (
                lambda values: preflight_calls.append(dict(values))
            ),
            "_apply_controller_calibration": (
                lambda *args: apply_calls.append(args) or True
            ),
            "ControllerStartupRequest": SimpleNamespace,
        }
        prepare_startup = self.compile_function(
            "_prepare_controller_startup",
            namespace,
        )

        with self.assertRaisesRegex(MotionInputError, "staged limits"):
            prepare_startup()

        self.assertEqual(calibration, {"com2Port": "old"})
        self.assertEqual(apply_calls, [])
        self.assertEqual(
            preflight_calls,
            [{"com2Port": "old", "update": 1, "external": 2}],
        )

    def test_controller_startup_defers_local_calibration_until_success(self):
        calibration = {"com2Port": "old", "sentinel": "unchanged"}
        preflight_calls = []
        namespace = {
            "CAL": calibration,
            "com2SelectedValue": SimpleNamespace(get=lambda: "COM2"),
            "auxiliaryBoardSelectedValue": SimpleNamespace(
                get=lambda: AUXILIARY_BOARD_NANO
            ),
            "_prepare_controller_calibration": lambda: (
                {"update": 1},
                "UPA1\n",
                {"external": 2},
                "CEA2\n",
            ),
            "_prepare_position_command": lambda values: "SPA1\n",
            "_prepare_cpp_kinematics_configuration": (
                lambda values: preflight_calls.append(dict(values))
            ),
            "ControllerStartupRequest": SimpleNamespace,
        }
        prepare_startup = self.compile_function(
            "_prepare_controller_startup",
            namespace,
        )

        request, update_values, external_values = prepare_startup()

        self.assertEqual(request.auxiliary_port, "COM2")
        self.assertEqual(request.auxiliary_board, AUXILIARY_BOARD_NANO)
        self.assertEqual(update_values, {"update": 1})
        self.assertEqual(external_values, {"external": 2})
        self.assertEqual(
            preflight_calls,
            [
                {
                    "com2Port": "old",
                    "sentinel": "unchanged",
                    "update": 1,
                    "external": 2,
                }
            ],
        )
        self.assertEqual(
            calibration,
            {"com2Port": "old", "sentinel": "unchanged"},
        )

    def test_controller_startup_rejects_returned_pose_before_local_mutation(self):
        calibration = {"com2Port": "old", "sentinel": "unchanged"}
        apply_calls = []
        display_calls = []
        namespace = {
            "CAL": calibration,
            "_controller_joint_calibration_from_values": (
                lambda values: ControllerJointCalibration(
                    negative_limits=(180.0,) * 9,
                    positive_limits=(180.0,) * 9,
                    steps_per_unit=(1.0,) * 9,
                )
            ),
            "_apply_controller_calibration": (
                lambda *args: apply_calls.append(args) or True
            ),
            "displayPosition": (
                lambda *args, **kwargs: display_calls.append((args, kwargs))
                or VALID_CONTROLLER_POSITION
            ),
        }
        apply_startup_result = self.compile_function(
            "_apply_controller_startup_result",
            namespace,
        )
        result = SimpleNamespace(
            position=SimpleNamespace(
                raw="out-of-range",
                joints=(181.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                external=(0.0, 0.0, 0.0),
            ),
            visual_options=(),
        )

        with self.assertRaisesRegex(MotionInputError, "J1 position"):
            apply_startup_result(
                SimpleNamespace(auxiliary_port="COM2"),
                result,
                {"update": 1},
                {"external": 2},
                SimpleNamespace(timeout=5.0),
                None,
            )

        self.assertEqual(
            calibration,
            {"com2Port": "old", "sentinel": "unchanged"},
        )
        self.assertEqual(apply_calls, [])
        self.assertEqual(display_calls, [])

    def test_controller_startup_result_commits_only_after_complete_finalization(self):
        calls = []
        calibration = {
            "com2Port": "old",
            "auxiliaryBoard": "old",
        }
        startup_serial = SimpleNamespace(timeout=2.0)

        class StartupCalibration:
            @staticmethod
            def validate_positions(positions):
                calls.append(("validate", positions))
                return positions

        class ErrorLog:
            def __enter__(self):
                calls.append(("open", "ErrorLog"))
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                calls.append(("close", "ErrorLog"))

        def apply_calibration(update_values, external_values):
            calls.append(("apply", update_values, external_values))
            calibration.update(update_values)
            calibration.update(external_values)
            return True

        def display_position(response, *, parsed):
            calls.append(("display", response, parsed))
            return VALID_CONTROLLER_POSITION

        position = SimpleNamespace(
            raw="startup-position",
            joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            external=(7.0, 8.0, 9.0),
        )
        namespace = {
            "CAL": calibration,
            "_controller_joint_calibration_from_values": (
                lambda values: StartupCalibration()
            ),
            "_apply_controller_calibration": apply_calibration,
            "displayPosition": display_position,
            "ProtocolResponseError": ProtocolResponseError,
            "updateVisOp": lambda options: calls.append(("visual", options)),
            "tab8": SimpleNamespace(
                ElogView=SimpleNamespace(get=lambda *args: ("event",))
            ),
            "END": "end",
            "open": lambda path, mode: ErrorLog(),
            "pickle": SimpleNamespace(
                dump=lambda value, destination: calls.append(
                    ("dump", value, destination)
                )
            ),
            "_invalidate_joint_motion_state": (
                lambda reason: calls.append(("invalidate", reason))
            ),
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        apply_startup_result = self.compile_function(
            "_apply_controller_startup_result",
            namespace,
        )

        self.assertIs(
            apply_startup_result(
                SimpleNamespace(
                    auxiliary_port="COM2",
                    auxiliary_board=AUXILIARY_BOARD_NANO,
                ),
                SimpleNamespace(position=position, visual_options=("arm.jpg",)),
                {"update": 1},
                {"external": 2},
                startup_serial,
                None,
            ),
            VALID_CONTROLLER_POSITION,
        )
        self.assertEqual(calibration["com2Port"], "COM2")
        self.assertEqual(
            calibration["auxiliaryBoard"],
            AUXILIARY_BOARD_NANO,
        )
        self.assertEqual(calibration["update"], 1)
        self.assertEqual(calibration["external"], 2)
        self.assertIsNone(startup_serial.timeout)
        self.assertFalse(any(call[0] == "invalidate" for call in calls))
        self.assertLess(
            next(index for index, call in enumerate(calls) if call[0] == "validate"),
            next(index for index, call in enumerate(calls) if call[0] == "apply"),
        )
        self.assertEqual(
            [call[0] for call in calls if call[0] in ("open", "dump", "close")],
            ["open", "dump", "close"],
        )

    def test_controller_startup_display_rejection_retains_calibration_and_closes(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.timeout = None
                self.close_count = 0

            @staticmethod
            def reset_input_buffer():
                pass

            @staticmethod
            def reset_output_buffer():
                pass

            def close(self):
                self.close_count += 1
                self.is_open = False

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        class StartupResult:
            def __init__(self, position):
                self.position = position
                self.visual_options = ()
                self.auxiliary_serial = None
                self.auxiliary_error = None

        port = Port()
        runtime = {"ser": None}
        calibration = {
            "com2Port": "old",
            "sentinel": "unchanged",
        }
        serial_lock = threading.Lock()
        activity = SerialActivityRegistry(("ser",))
        invalidations = []
        close_calls = []
        cleanup_calls = []
        first_label = Label()
        second_label = Label()
        request = SimpleNamespace(auxiliary_port="COM2")
        result = StartupResult(
            SimpleNamespace(
                raw="startup-position",
                joints=(0.0,) * 6,
                external=(0.0,) * 3,
            )
        )

        def apply_calibration(update_values, external_values):
            calibration.update(update_values)
            calibration.update(external_values)
            return True

        def close_failed(serial_port, activity_lease, request_lease):
            close_calls.append(serial_port)
            serial_port.close()
            if runtime.get("ser") is serial_port:
                runtime["ser"] = None
            activity_lease.close()
            serial_lock.release()
            request_lease.close()
            return True

        def run_startup(
            root,
            startup_request,
            on_finished,
            on_timeout,
            on_abandoned,
            timeout,
        ):
            on_finished(result, False)
            return SimpleNamespace()

        namespace = {
            "application_closing": threading.Event(),
            "serial_lock": serial_lock,
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "logger": SimpleNamespace(
                info=lambda *args: None,
                warning=lambda *args: None,
                error=lambda *args, **kwargs: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "CAL": calibration,
            "RUN": runtime,
            "com1SelectedValue": SimpleNamespace(get=lambda: "COM1"),
            "serial": SimpleNamespace(Serial=lambda **kwargs: port),
            "SERIAL_WRITE_TIMEOUT_SECONDS": 5.0,
            "SERIAL_STARTUP_READ_TIMEOUT_SECONDS": 2.0,
            "time": SimpleNamespace(sleep=lambda seconds: None),
            "_prepare_controller_startup": lambda: (
                request,
                {"sentinel": "staged"},
                {"external": 2},
            ),
            "ControllerStartupResult": StartupResult,
            "startup_with_spinner": run_startup,
            "root": SimpleNamespace(),
            "_controller_joint_calibration_from_values": (
                lambda values: ControllerJointCalibration(
                    negative_limits=(180.0,) * 9,
                    positive_limits=(180.0,) * 9,
                    steps_per_unit=(1.0,) * 9,
                )
            ),
            "_apply_controller_calibration": apply_calibration,
            "displayPosition": lambda *args, **kwargs: None,
            "ProtocolResponseError": ProtocolResponseError,
            "updateVisOp": lambda options: None,
            "tab8": SimpleNamespace(
                ElogView=SimpleNamespace(get=lambda *args: ())
            ),
            "END": "end",
            "pickle": SimpleNamespace(dump=lambda *args: None),
            "open": lambda *args, **kwargs: object(),
            "_invalidate_joint_motion_state": invalidations.append,
            "_request_startup_auxiliary_cleanup": cleanup_calls.append,
            "_poll_failed_controller_close": close_failed,
            "_release_async_main_serial_transport": (
                lambda activity_lease, request_lease: (_ for _ in ()).throw(
                    AssertionError("failed startup must use cleanup ownership")
                )
            ),
        }
        namespace["_apply_controller_startup_result"] = self.compile_function(
            "_apply_controller_startup_result",
            namespace,
        )
        set_com = self.compile_function("setCom", namespace)

        self.assertTrue(set_com())
        self.assertEqual(calibration["sentinel"], "staged")
        self.assertEqual(calibration["external"], 2)
        self.assertEqual(calibration["com2Port"], "old")
        self.assertEqual(
            invalidations,
            [
                "controller startup finalization failed after staged "
                "calibration was applied"
            ],
        )
        self.assertEqual(cleanup_calls, [None])
        self.assertEqual(close_calls, [port])
        self.assertEqual(port.close_count, 1)
        self.assertIsNone(runtime["ser"])
        self.assertFalse(serial_lock.locked())
        self.assertTrue(activity.idle())
        self.assertEqual(
            first_label.text,
            "UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER",
        )
        self.assertEqual(second_label.text, first_label.text)

    def test_motion_timeout_bounds_reject_invalid_runtime_value(self):
        namespace = {
            "RUN": {"minSpeedDelay": "invalid"},
            "math": math,
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
        }
        runtime_number = self.compile_function("_runtime_number", namespace)

        with self.assertRaisesRegex(MotionInputError, "minSpeedDelay"):
            runtime_number("minSpeedDelay")

    def test_motion_timeout_bounds_reject_boolean_configuration(self):
        namespace = {
            "CAL": {"J1PosLim": True},
            "RUN": {"minSpeedDelay": False},
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
        }
        configuration_number = self.compile_function(
            "_configuration_number",
            namespace,
        )
        runtime_number = self.compile_function("_runtime_number", namespace)

        with self.assertRaisesRegex(MotionInputError, "J1PosLim"):
            configuration_number("J1PosLim")
        with self.assertRaisesRegex(MotionInputError, "minSpeedDelay"):
            runtime_number("minSpeedDelay")

    def test_failed_calibration_write_retains_dirty_state_and_retries(self):
        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                job = f"job-{len(self.jobs) + 1}"
                self.jobs.append((job, delay, callback))
                return job

        class Logger:
            def __init__(self):
                self.errors = []
                self.exceptions = []

            def error(self, message):
                self.errors.append(message)

            def exception(self, message):
                self.exceptions.append(message)

        class Closing:
            @staticmethod
            def is_set():
                return False

        outcomes = [False, True]
        root = Root()
        logger = Logger()
        namespace = {
            "_calibration_save_job": "initial-job",
            "_calibration_dirty": True,
            "save_calibration": lambda calibration: outcomes.pop(0),
            "CAL": {"J1AngCur": "1"},
            "logger": logger,
            "application_closing": Closing(),
            "root": root,
            "CALIBRATION_SAVE_DEBOUNCE_MS": 250,
            "RuntimeError": RuntimeError,
            "tk": SimpleNamespace(TclError=RuntimeError),
        }
        write_pending = self.compile_function(
            "_write_pending_calibration",
            namespace,
        )

        self.assertFalse(write_pending())
        self.assertTrue(namespace["_calibration_dirty"])
        self.assertEqual(namespace["_calibration_save_job"], "job-1")
        self.assertEqual(len(root.jobs), 1)
        self.assertEqual(logger.exceptions, [])

        self.assertTrue(write_pending())
        self.assertFalse(namespace["_calibration_dirty"])
        self.assertIsNone(namespace["_calibration_save_job"])

    def test_shutdown_requests_online_and_offline_stop_before_final_flush(self):
        class Flag:
            def __init__(self, initial=False):
                self.value = initial

            def is_set(self):
                return self.value

            def set(self):
                self.value = True

        class Dispatcher:
            def __init__(self):
                self.closed = False
                self.active = True

            def close(self):
                self.closed = True

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

        class Logger:
            def __init__(self):
                self.errors = []

            def error(self, message):
                self.errors.append(message)

        closing = Flag()
        live_pending = Flag(True)
        live_stop = Flag()
        offline_stop = Flag()
        dispatcher = Dispatcher()
        first_label = Label()
        second_label = Label()
        root = Root()
        activity = SerialActivityRegistry(("ser", "ser2"))
        poll_calls = []
        namespace = {
            "application_closing": closing,
            "serial_activity_registry": activity,
            "live_serial_result_pending": live_pending,
            "live_jog_stop_requested": live_stop,
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_stop_event": offline_stop,
            "joint_motion_dispatcher": dispatcher,
            "_poll_application_close": lambda: poll_calls.append(True),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "root": root,
        }
        close_application = self.compile_function("on_closing", namespace)

        self.assertTrue(close_application())
        self.assertTrue(closing.is_set())
        self.assertTrue(live_stop.is_set())
        self.assertTrue(offline_stop.is_set())
        self.assertTrue(dispatcher.closed)
        with self.assertRaises(SerialActivityRejected):
            activity.begin("ser")
        self.assertEqual(poll_calls, [True])
        self.assertEqual(first_label.text, "SHUTDOWN WAITING FOR CONTROLLER")
        self.assertEqual(second_label.text, first_label.text)
        self.assertFalse(close_application())
        self.assertEqual(poll_calls, [True])

    def test_shutdown_waits_for_transport_ownership_before_closing_ports(self):
        class Lock:
            def __init__(self):
                self.results = [False, True, True]
                self.release_count = 0

            def acquire(self, blocking=True):
                self.asserted_nonblocking = blocking is False
                return self.results.pop(0)

            def release(self):
                self.release_count += 1

        class Root:
            def __init__(self):
                self.jobs = []
                self.quit_count = 0
                self.destroy_count = 0

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

            def quit(self):
                self.quit_count += 1

            def destroy(self):
                self.destroy_count += 1

        lock = Lock()
        root = Root()
        activity = SerialActivityRegistry(("ser", "ser2"))
        activity.begin("ser")
        activity.begin_shutdown()
        closed_ports = []
        destroyed_windows = []
        drained_events = []

        def close_port(name):
            closed_ports.append(name)
            return True

        namespace = {
            "serial_lock": lock,
            "auxiliary_serial_lock": threading.Lock(),
            "serial_activity_registry": activity,
            "application_shutdown_started_at": None,
            "time": SimpleNamespace(monotonic=lambda: 0.0),
            "SERIAL_SHUTDOWN_ACTIVITY_GRACE_SECONDS": 1.0,
            "_interrupt_tracked_serial_activity": lambda name: False,
            "SERIAL_SHUTDOWN_POLL_MS": 25,
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
            "root": root,
            "_close_serial_port": close_port,
            "_poll_serial_events": lambda: drained_events.append("serial"),
            "_poll_auxiliary_serial_events": lambda: drained_events.append("auxiliary"),
            "_poll_joint_motion_events": lambda: drained_events.append("joint"),
            "joint_motion_dispatcher": SimpleNamespace(close=lambda: None),
            "_flush_calibration_save": lambda: True,
            "logger": SimpleNamespace(exception=lambda *args: None),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "cv2": SimpleNamespace(
                destroyAllWindows=lambda: destroyed_windows.append(True)
            ),
        }
        poll_close = self.compile_function("_poll_application_close", namespace)

        self.assertFalse(poll_close())
        self.assertEqual(closed_ports, [])
        self.assertEqual(root.destroy_count, 0)
        self.assertEqual(len(root.jobs), 1)
        self.assertEqual(root.jobs[0][0], 25)

        first_retry = root.jobs.pop(0)[1]
        self.assertFalse(first_retry())
        self.assertEqual(closed_ports, [])
        self.assertEqual(root.destroy_count, 0)
        self.assertEqual(len(root.jobs), 1)

        activity.end("ser")
        final_retry = root.jobs.pop(0)[1]
        self.assertTrue(final_retry())
        self.assertTrue(lock.asserted_nonblocking)
        self.assertEqual(closed_ports, ["ser", "ser2"])
        self.assertEqual(
            drained_events,
            [
                "serial", "auxiliary", "joint",
                "serial", "auxiliary", "joint",
                "serial", "auxiliary", "joint",
            ],
        )
        self.assertEqual(lock.release_count, 2)
        self.assertEqual(destroyed_windows, [True])
        self.assertEqual(root.quit_count, 1)
        self.assertEqual(root.destroy_count, 1)

    def test_shutdown_waits_for_retained_cleanup_and_virtual_owners(self):
        class Root:
            def __init__(self):
                self.jobs = []
                self.destroy_count = 0

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

            def quit(self):
                pass

            def destroy(self):
                self.destroy_count += 1

        root = Root()
        auxiliary_pending = {1: object()}
        offline_operation = completed_virtual_operation()
        virtual_active = {"value": False}
        ensures = []
        flushes = []
        closes = []
        namespace = {
            "serial_lock": threading.Lock(),
            "auxiliary_serial_lock": threading.Lock(),
            "serial_activity_registry": SimpleNamespace(idle=lambda: True),
            "startup_controller_cleanup_lock": threading.Lock(),
            "startup_controller_cleanup_pending": {},
            "_ensure_startup_controller_cleanup": lambda: True,
            "startup_auxiliary_cleanup_lock": threading.Lock(),
            "startup_auxiliary_cleanup_pending": auxiliary_pending,
            "_ensure_startup_auxiliary_cleanup": (
                lambda: ensures.append("auxiliary") or True
            ),
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_operation": offline_operation,
            "_virtual_motion_active": lambda: virtual_active["value"],
            "SERIAL_SHUTDOWN_POLL_MS": 25,
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
            "root": root,
            "_close_serial_port": lambda name: closes.append(name) or True,
            "_poll_serial_events": lambda: None,
            "_poll_auxiliary_serial_events": lambda: None,
            "_poll_joint_motion_events": lambda: None,
            "_poll_virtual_motion_events": lambda: None,
            "joint_motion_dispatcher": SimpleNamespace(close=lambda: None),
            "_flush_calibration_save": lambda: flushes.append(True) or True,
            "logger": SimpleNamespace(exception=lambda *args: None),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "cv2": SimpleNamespace(destroyAllWindows=lambda: None),
        }
        poll_close = self.compile_function("_poll_application_close", namespace)

        self.assertFalse(poll_close())
        self.assertEqual(ensures, ["auxiliary"])
        self.assertEqual(flushes, [])
        auxiliary_pending.clear()

        self.assertFalse(root.jobs.pop(0)[1]())
        self.assertEqual(flushes, [])
        namespace["offline_live_jog_operation"] = None
        virtual_active["value"] = True

        self.assertFalse(root.jobs.pop(0)[1]())
        self.assertEqual(flushes, [])
        virtual_active["value"] = False

        self.assertTrue(root.jobs.pop(0)[1]())
        self.assertEqual(flushes, [True])
        self.assertEqual(closes, ["ser", "ser2"])
        self.assertEqual(root.destroy_count, 1)

    def test_shutdown_drains_a_late_legacy_result_before_lock_acquisition(self):
        class SignalingQueue(Queue):
            def __init__(self):
                super().__init__()
                self.started = threading.Event()
                self.terminal = threading.Event()

            def put(self, item, *args, **kwargs):
                super().put(item, *args, **kwargs)
                if item[0] == "started":
                    self.started.set()
                elif item[0] in ("completed", "failed"):
                    self.terminal.set()

        class Widget:
            def delete(self, *args):
                pass

            def insert(self, *args):
                pass

            def config(self, **kwargs):
                pass

        class Root:
            def __init__(self):
                self.jobs = []
                self.quit_count = 0
                self.destroy_count = 0

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

            def quit(self):
                self.quit_count += 1

            def destroy(self):
                self.destroy_count += 1

        class Dispatcher:
            active = False

            def __init__(self, sequence):
                self.sequence = sequence

            def close(self):
                self.sequence.append("dispatcher-close")

        result_queue = SignalingQueue()
        exchange_release = threading.Event()
        closing = threading.Event()
        closing.set()
        transport_lock = threading.Lock()
        transport_lock.acquire()
        auxiliary_lock = threading.Lock()
        activity = SerialActivityRegistry(("ser", "ser2"))
        activity.begin_shutdown()
        legacy_pending = threading.Event()
        legacy_pending.set()
        root = Root()
        sequence = []
        completions = []
        dispatcher = Dispatcher(sequence)

        def exchange(command, control_event=None, write_started_event=None):
            if not exchange_release.wait(2):
                raise TimeoutError("test worker release timed out")
            return "position"

        def apply_result(response):
            sequence.append("result")
            return VALID_CONTROLLER_POSITION if response == "position" else None

        def flush_calibration():
            sequence.append("flush")
            return True

        def close_port(name):
            sequence.append(f"close-{name}")
            return True

        logger = SimpleNamespace(
            error=lambda *args: None,
            exception=lambda *args: None,
        )
        namespace = {
            "dataclass": dataclass,
            "threading": threading,
            "serial_event_queue": result_queue,
            "_exchange_serial_line": exchange,
            "live_jog_stop_requested": threading.Event(),
            "live_serial_result_pending": threading.Event(),
            "legacy_serial_result_pending": legacy_pending,
            "application_closing": closing,
            "serial_lock": transport_lock,
            "auxiliary_serial_lock": auxiliary_lock,
            "serial_activity_registry": activity,
            "application_shutdown_started_at": None,
            "time": SimpleNamespace(monotonic=lambda: 0.0),
            "SERIAL_SHUTDOWN_ACTIVITY_GRACE_SECONDS": 1.0,
            "_interrupt_tracked_serial_activity": lambda name: False,
            "SERIAL_SHUTDOWN_POLL_MS": 25,
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
            "Empty": Empty,
            "cmdSentEntryField": Widget(),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
            "logger": logger,
            "_apply_legacy_serial_response": apply_result,
            "_invalidate_joint_motion_state": lambda reason: None,
            "RUN": {"liveJog": False, "VR_angles": [0.0] * 6},
            "_try_dispatch_deferred_joint_adjustments": lambda **kwargs: False,
            "joint_motion_dispatcher": dispatcher,
            "root": root,
            "_poll_auxiliary_serial_events": lambda: sequence.append(
                "auxiliary"
            ),
            "_poll_joint_motion_events": lambda: sequence.append("joint"),
            "_flush_calibration_save": flush_calibration,
            "_close_serial_port": close_port,
            "cv2": SimpleNamespace(
                destroyAllWindows=lambda: sequence.append("windows")
            ),
        }
        run_worker = self.compile_function("run_send_serial_safe", namespace)
        poll_serial = self.compile_function("_poll_serial_events", namespace)
        namespace["_poll_serial_events"] = poll_serial
        poll_close = self.compile_function("_poll_application_close", namespace)
        worker = threading.Thread(
            target=run_worker,
            args=("RJ\n", False, completions.append),
        )
        worker.start()
        self.assertTrue(result_queue.started.wait(2))

        self.assertFalse(poll_close())
        self.assertTrue(transport_lock.locked())
        self.assertTrue(legacy_pending.is_set())
        self.assertEqual(sequence, ["auxiliary", "joint"])
        self.assertEqual(len(root.jobs), 1)

        exchange_release.set()
        self.assertTrue(result_queue.terminal.wait(2))
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertTrue(transport_lock.locked())

        retry = root.jobs.pop(0)[1]
        self.assertTrue(retry())
        self.assertFalse(transport_lock.locked())
        self.assertFalse(legacy_pending.is_set())
        self.assertEqual(completions, [VALID_CONTROLLER_POSITION])
        self.assertLess(sequence.index("result"), sequence.index("flush"))
        self.assertEqual(
            [entry for entry in sequence if entry.startswith("close-")],
            ["close-ser", "close-ser2"],
        )
        self.assertEqual(root.quit_count, 1)
        self.assertEqual(root.destroy_count, 1)
        self.assertEqual(root.jobs, [])

    def test_shutdown_interrupts_stuck_serial_activity_before_logical_motion_settles(self):
        class Port:
            def __init__(self, activity, logical_motion_active):
                self.activity = activity
                self.logical_motion_active = logical_motion_active
                self.is_open = True
                self.cancel_count = 0
                self.close_count = 0

            def cancel_read(self):
                self.cancel_count += 1

            def close(self):
                self.close_count += 1
                self.is_open = False
                if self.activity.active("ser"):
                    self.activity.end("ser")
                self.logical_motion_active["value"] = False

        class Root:
            def __init__(self):
                self.jobs = []
                self.destroy_count = 0

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

            def quit(self):
                pass

            def destroy(self):
                self.destroy_count += 1

        activity = SerialActivityRegistry(("ser", "ser2"))
        activity.begin("ser")
        activity.begin_shutdown()
        logical_motion_active = {"value": True}
        serial_port = Port(activity, logical_motion_active)
        root = Root()
        namespace = {
            "RUN": {"ser": serial_port, "ser2": None},
            "serial_lock": threading.Lock(),
            "auxiliary_serial_lock": threading.Lock(),
            "serial_activity_registry": activity,
            "application_shutdown_started_at": 0.0,
            "shutdown_serial_cancel_requested": set(),
            "time": SimpleNamespace(monotonic=lambda: 2.0),
            "SERIAL_SHUTDOWN_ACTIVITY_GRACE_SECONDS": 1.0,
            "SERIAL_SHUTDOWN_POLL_MS": 25,
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
            "root": root,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "_poll_serial_events": lambda: None,
            "_poll_auxiliary_serial_events": lambda: None,
            "_poll_joint_motion_events": lambda: None,
            "_virtual_motion_active": (
                lambda: logical_motion_active["value"]
            ),
            "joint_motion_dispatcher": SimpleNamespace(close=lambda: None),
            "_flush_calibration_save": lambda: True,
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "cv2": SimpleNamespace(destroyAllWindows=lambda: None),
        }
        namespace["_close_serial_port"] = self.compile_function(
            "_close_serial_port",
            namespace,
        )
        namespace["_interrupt_tracked_serial_activity"] = self.compile_function(
            "_interrupt_tracked_serial_activity",
            namespace,
        )
        poll_close = self.compile_function("_poll_application_close", namespace)

        self.assertFalse(poll_close())
        self.assertEqual(serial_port.cancel_count, 1)
        self.assertEqual(serial_port.close_count, 0)
        self.assertTrue(activity.active("ser"))
        self.assertEqual(root.jobs[0][0], 25)

        self.assertTrue(root.jobs.pop(0)[1]())
        self.assertEqual(serial_port.close_count, 1)
        self.assertFalse(activity.active("ser"))
        self.assertIsNone(namespace["RUN"]["ser"])
        self.assertEqual(root.destroy_count, 1)

    def test_shutdown_close_helper_retains_a_port_that_did_not_close(self):
        class Port:
            def __init__(self, closes=True, raises=False):
                self.is_open = True
                self.closes = closes
                self.raises = raises

            def close(self):
                if self.raises:
                    raise OSError("close failed")
                if self.closes:
                    self.is_open = False

        class Logger:
            def __init__(self):
                self.errors = []
                self.exceptions = []

            def error(self, *args):
                self.errors.append(args)

            def exception(self, *args):
                self.exceptions.append(args)

        logger = Logger()
        successful = Port()
        namespace = {"RUN": {"ser": successful}, "logger": logger}
        close_port = self.compile_function("_close_serial_port", namespace)

        self.assertTrue(close_port("ser"))
        self.assertIsNone(namespace["RUN"]["ser"])

        stuck = Port(closes=False)
        namespace["RUN"]["ser"] = stuck
        self.assertFalse(close_port("ser"))
        self.assertIs(namespace["RUN"]["ser"], stuck)
        self.assertEqual(len(logger.errors), 1)

        failing = Port(raises=True)
        namespace["RUN"]["ser"] = failing
        self.assertFalse(close_port("ser"))
        self.assertIs(namespace["RUN"]["ser"], failing)
        self.assertEqual(len(logger.exceptions), 1)

    def test_shutdown_retries_a_failed_serial_close(self):
        class Lock:
            def __init__(self):
                self.release_count = 0

            @staticmethod
            def acquire(blocking=True):
                return True

            def release(self):
                self.release_count += 1

        class Root:
            def __init__(self):
                self.jobs = []
                self.destroy_count = 0

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

            def quit(self):
                pass

            def destroy(self):
                self.destroy_count += 1

        attempts = []

        def close_port(name):
            attempts.append(name)
            return not (name == "ser" and attempts.count("ser") == 1)

        lock = Lock()
        auxiliary_lock = Lock()
        root = Root()
        namespace = {
            "serial_lock": lock,
            "auxiliary_serial_lock": auxiliary_lock,
            "serial_activity_registry": SimpleNamespace(idle=lambda: True),
            "SERIAL_SHUTDOWN_POLL_MS": 25,
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
            "root": root,
            "_close_serial_port": close_port,
            "_poll_serial_events": lambda: None,
            "_poll_auxiliary_serial_events": lambda: None,
            "_poll_joint_motion_events": lambda: None,
            "joint_motion_dispatcher": SimpleNamespace(close=lambda: None),
            "_flush_calibration_save": lambda: True,
            "logger": SimpleNamespace(exception=lambda *args: None),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "cv2": SimpleNamespace(destroyAllWindows=lambda: None),
        }
        poll_close = self.compile_function("_poll_application_close", namespace)

        self.assertFalse(poll_close())
        self.assertEqual(root.destroy_count, 0)
        self.assertEqual(len(root.jobs), 1)
        self.assertEqual(root.jobs[0][0], 1000)
        self.assertTrue(root.jobs[0][1]())
        self.assertEqual(root.destroy_count, 1)
        self.assertEqual(lock.release_count, 2)
        self.assertEqual(auxiliary_lock.release_count, 2)
        self.assertEqual(attempts, ["ser", "ser2", "ser", "ser2"])

    def test_failed_connection_retains_transport_until_serial_close(self):
        class Lock:
            def __init__(self):
                self.release_count = 0

            def release(self):
                self.release_count += 1

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

        serial_port = SimpleNamespace(is_open=True)
        outcomes = [False, True]
        close_calls = []
        lock = Lock()
        root = Root()
        activity = SerialActivityRegistry(("ser",))
        activity_lease = activity.lease("ser")
        motion_registry = MotionRequestRegistry()
        request_lease = motion_registry.acquire("Controller connection change")

        def close_port(name, context):
            close_calls.append((name, context))
            return outcomes.pop(0)

        namespace = {
            "RUN": {"ser": serial_port},
            "motion_request_registry": motion_registry,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "serial_lock": lock,
            "_release_async_main_serial_transport": lambda activity, request: (
                lock.release(),
                activity.close(),
                request.close(),
            ),
            "_close_serial_port": close_port,
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "root": root,
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
        }
        namespace["_close_failed_controller_startup"] = self.compile_function(
            "_close_failed_controller_startup",
            namespace,
        )
        poll_close = self.compile_function(
            "_poll_failed_controller_close",
            namespace,
        )

        self.assertFalse(poll_close(serial_port, activity_lease, request_lease))
        self.assertEqual(lock.release_count, 0)
        self.assertEqual(len(root.jobs), 1)
        self.assertEqual(root.jobs[0][0], 1000)

        self.assertTrue(root.jobs[0][1]())
        self.assertEqual(lock.release_count, 1)
        self.assertTrue(activity.idle())
        self.assertFalse(motion_registry.active)
        self.assertEqual(
            close_calls,
            [
                ("ser", "failed controller connection cleanup"),
                ("ser", "failed controller connection cleanup"),
            ],
        )

    def test_failed_connection_survives_close_and_scheduler_failure(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.close_count = 0

            def close(self):
                self.close_count += 1
                if self.close_count == 1:
                    raise OSError("close failed")
                self.is_open = False

        class Root:
            def __init__(self):
                self.after_count = 0

            def after(self, delay, callback):
                self.after_count += 1
                raise RuntimeError("Tk scheduler unavailable")

        serial_port = Port()
        root = Root()
        transport_lock = threading.Lock()
        transport_lock.acquire()
        activity = SerialActivityRegistry(("ser",))
        activity_lease = activity.lease("ser")
        motion_registry = MotionRequestRegistry()
        request_lease = motion_registry.acquire("Controller connection change")
        namespace = {
            "RUN": {"ser": serial_port},
            "motion_request_registry": motion_registry,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "serial_lock": transport_lock,
            "startup_controller_cleanup_lock": threading.Lock(),
            "startup_controller_cleanup_pending": {},
            "startup_controller_cleanup_worker": None,
            "threading": threading,
            "time": time,
            "root": root,
            "SERIAL_SHUTDOWN_RETRY_MS": 1,
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        for name in (
            "_close_serial_port",
            "_release_async_main_serial_transport",
            "_close_failed_controller_startup",
            "_run_startup_controller_cleanup",
            "_ensure_startup_controller_cleanup",
            "_retain_failed_controller_startup",
            "_poll_failed_controller_close",
        ):
            namespace[name] = self.compile_function(name, namespace)

        self.assertFalse(
            namespace["_poll_failed_controller_close"](
                serial_port,
                activity_lease,
                request_lease,
            )
        )

        deadline = time.monotonic() + 2
        while (
            namespace["startup_controller_cleanup_pending"]
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)

        self.assertEqual(namespace["startup_controller_cleanup_pending"], {})
        self.assertEqual(serial_port.close_count, 2)
        self.assertEqual(root.after_count, 1)
        self.assertFalse(transport_lock.locked())
        self.assertTrue(activity.idle())
        self.assertFalse(motion_registry.active)
        self.assertIsNone(namespace["RUN"]["ser"])

    def test_shutdown_retries_final_position_persistence_before_serial_close(self):
        class Lock:
            def __init__(self):
                self.release_count = 0

            @staticmethod
            def acquire(blocking=True):
                return True

            def release(self):
                self.release_count += 1

        class Root:
            def __init__(self):
                self.jobs = []
                self.destroy_count = 0

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

            def quit(self):
                pass

            def destroy(self):
                self.destroy_count += 1

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        outcomes = [False, True]
        closed_ports = []
        root = Root()
        first_label = Label()
        second_label = Label()
        namespace = {
            "serial_lock": Lock(),
            "auxiliary_serial_lock": Lock(),
            "serial_activity_registry": SimpleNamespace(idle=lambda: True),
            "SERIAL_SHUTDOWN_POLL_MS": 25,
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
            "root": root,
            "_close_serial_port": lambda name: closed_ports.append(name) or True,
            "_poll_serial_events": lambda: None,
            "_poll_auxiliary_serial_events": lambda: None,
            "_poll_joint_motion_events": lambda: None,
            "joint_motion_dispatcher": SimpleNamespace(close=lambda: None),
            "_flush_calibration_save": lambda: outcomes.pop(0),
            "logger": SimpleNamespace(exception=lambda *args: None),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "cv2": SimpleNamespace(destroyAllWindows=lambda: None),
        }
        poll_close = self.compile_function("_poll_application_close", namespace)

        self.assertFalse(poll_close())
        self.assertEqual(closed_ports, [])
        self.assertEqual(root.destroy_count, 0)
        self.assertEqual(first_label.text, "SHUTDOWN WAITING FOR CALIBRATION SAVE")
        self.assertEqual(second_label.text, first_label.text)
        self.assertEqual(len(root.jobs), 1)
        self.assertEqual(root.jobs[0][0], 1000)

        self.assertTrue(root.jobs[0][1]())
        self.assertEqual(closed_ports, ["ser", "ser2"])
        self.assertEqual(root.destroy_count, 1)

    def test_shutdown_retains_calibration_terminal_response_ownership(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.cancel_count = 0

            def cancel_read(self):
                self.cancel_count += 1

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

        root = Root()
        first_label = Label()
        second_label = Label()
        main_port = Port()
        auxiliary_port = Port()
        activity = SerialActivityRegistry(("ser", "ser2"))
        activity.begin("ser")
        activity.begin("ser2")
        terminal_pending = threading.Event()
        terminal_pending.set()
        write_committed = threading.Event()
        write_committed.set()
        monotonic_values = iter((0.0, 2.0))
        namespace = {
            "calibration_terminal_response_pending": terminal_pending,
            "calibration_serial_write_committed": write_committed,
            "calibration_operation_lock": threading.Lock(),
            "calibration_operation": None,
            "serial_activity_registry": activity,
            "RUN": {"ser": main_port, "ser2": auxiliary_port},
            "shutdown_serial_cancel_requested": set(),
            "application_shutdown_started_at": None,
            "time": SimpleNamespace(monotonic=lambda: next(monotonic_values)),
            "SERIAL_SHUTDOWN_ACTIVITY_GRACE_SECONDS": 1.0,
            "_poll_serial_events": lambda: None,
            "_poll_calibration_events": lambda: None,
            "_poll_auxiliary_serial_events": lambda: None,
            "_poll_joint_motion_events": lambda: None,
            "_poll_virtual_motion_events": lambda: None,
            "_close_serial_port": lambda *args: self.fail(
                "calibration shutdown closed a tracked port"
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "root": root,
            "SERIAL_SHUTDOWN_POLL_MS": 25,
        }
        namespace["_calibration_shutdown_pending"] = self.compile_function(
            "_calibration_shutdown_pending",
            namespace,
        )
        namespace["_interrupt_tracked_serial_activity"] = self.compile_function(
            "_interrupt_tracked_serial_activity",
            namespace,
        )
        poll_close = self.compile_function("_poll_application_close", namespace)

        self.assertFalse(poll_close())
        self.assertEqual(main_port.cancel_count, 0)
        self.assertEqual(auxiliary_port.cancel_count, 0)
        self.assertEqual(len(root.jobs), 1)

        _, retry = root.jobs.pop(0)
        self.assertFalse(retry())
        self.assertEqual(main_port.cancel_count, 0)
        self.assertEqual(auxiliary_port.cancel_count, 1)
        self.assertEqual(namespace["shutdown_serial_cancel_requested"], {"ser2"})
        self.assertEqual(
            first_label.text,
            "SHUTDOWN WAITING FOR CALIBRATION RESPONSE",
        )
        self.assertEqual(second_label.text, first_label.text)
        self.assertEqual(root.jobs, [(25, poll_close)])

    def test_shutdown_interrupts_prewrite_calibration_activity(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.cancel_count = 0

            def cancel_read(self):
                self.cancel_count += 1

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

        root = Root()
        first_label = Label()
        second_label = Label()
        main_port = Port()
        activity = SerialActivityRegistry(("ser", "ser2"))
        activity.begin("ser")
        terminal_pending = threading.Event()
        terminal_pending.set()
        monotonic_values = iter((0.0, 2.0, 3.0))
        close_calls = []
        namespace = {
            "calibration_terminal_response_pending": terminal_pending,
            "calibration_serial_write_committed": threading.Event(),
            "calibration_operation_lock": threading.Lock(),
            "calibration_operation": None,
            "serial_activity_registry": activity,
            "RUN": {"ser": main_port, "ser2": None},
            "shutdown_serial_cancel_requested": set(),
            "application_shutdown_started_at": None,
            "time": SimpleNamespace(monotonic=lambda: next(monotonic_values)),
            "SERIAL_SHUTDOWN_ACTIVITY_GRACE_SECONDS": 1.0,
            "_poll_serial_events": lambda: None,
            "_poll_calibration_events": lambda: None,
            "_poll_auxiliary_serial_events": lambda: None,
            "_poll_joint_motion_events": lambda: None,
            "_poll_virtual_motion_events": lambda: None,
            "_close_serial_port": lambda *args: close_calls.append(args) or True,
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "root": root,
            "SERIAL_SHUTDOWN_POLL_MS": 25,
        }
        namespace["_calibration_shutdown_pending"] = self.compile_function(
            "_calibration_shutdown_pending",
            namespace,
        )
        namespace["_interrupt_tracked_serial_activity"] = self.compile_function(
            "_interrupt_tracked_serial_activity",
            namespace,
        )
        poll_close = self.compile_function("_poll_application_close", namespace)

        self.assertFalse(poll_close())
        self.assertEqual(main_port.cancel_count, 0)

        _, retry = root.jobs.pop(0)
        self.assertFalse(retry())
        self.assertEqual(main_port.cancel_count, 1)
        self.assertEqual(namespace["shutdown_serial_cancel_requested"], {"ser"})
        self.assertEqual(
            first_label.text,
            "SHUTDOWN WAITING FOR CALIBRATION RESPONSE",
        )
        self.assertEqual(second_label.text, first_label.text)
        self.assertEqual(root.jobs, [(25, poll_close)])

        _, retry = root.jobs.pop(0)
        self.assertFalse(retry())
        self.assertEqual(main_port.cancel_count, 1)
        self.assertEqual(
            close_calls,
            [("ser", "tracked activity shutdown interruption")],
        )
        self.assertEqual(root.jobs, [(25, poll_close)])

    def test_connection_switch_closes_only_inside_transport_reservation(self):
        function = self.module_functions["_set_com_admitted"]
        calls = [node for node in ast.walk(function) if isinstance(node, ast.Call)]
        acquisitions = [
            node
            for node in calls
            if isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "serial_lock"
            and node.func.attr == "acquire"
        ]
        closes = [
            node
            for node in calls
            if isinstance(node.func, ast.Attribute)
            and node.func.attr == "close"
        ]
        protected_releases = [
            node
            for node in ast.walk(function)
            if isinstance(node, ast.Try)
            and any(
                isinstance(child, ast.Call)
                and isinstance(child.func, ast.Name)
                and child.func.id == "_release_async_main_serial_transport"
                for statement in node.finalbody
                for child in ast.walk(statement)
            )
        ]

        self.assertEqual(len(acquisitions), 1)
        self.assertTrue(closes)
        self.assertLess(acquisitions[0].lineno, min(node.lineno for node in closes))
        self.assertEqual(len(protected_releases), 1)

    def test_direct_main_serial_operations_share_dispatcher_transport_reservation(self):
        transport_lock = threading.Lock()
        activity = SerialActivityRegistry(
            ("ser", "ser2"),
            single_owner_names=("ser2",),
        )
        warnings = []
        namespace = {
            "contextmanager": contextmanager,
            "wraps": wraps,
            "main_serial_operation_state": threading.local(),
            "serial_lock": transport_lock,
            "auxiliary_serial_lock": threading.Lock(),
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "logger": SimpleNamespace(
                warning=lambda *args: warnings.append(args)
            ),
        }
        namespace["_reserve_main_serial_operation"] = contextmanager(
            self.compile_function(
                "_reserve_main_serial_operation",
                namespace,
            )
        )
        tracked_operation = self.compile_function(
            "_tracked_serial_operation",
            namespace,
        )
        calls = []

        @tracked_operation("ser")
        def direct_operation():
            calls.append("direct")
            return "complete"

        transport_lock.acquire()
        self.assertFalse(direct_operation())
        self.assertEqual(calls, [])
        self.assertTrue(transport_lock.locked())

        @tracked_operation(
            "ser",
            operation_required=lambda transmit=True: transmit is not False,
        )
        def local_only_operation(transmit=True):
            calls.append("local")
            return "local-complete"

        self.assertEqual(
            local_only_operation(transmit=False),
            "local-complete",
        )
        self.assertEqual(calls, ["local"])
        transport_lock.release()

        @tracked_operation("ser")
        def nested_operation():
            calls.append("nested")
            return "nested-complete"

        @tracked_operation("ser")
        def outer_operation():
            calls.append("outer")
            return nested_operation()

        self.assertEqual(outer_operation(), "nested-complete")
        self.assertEqual(calls, ["local", "outer", "nested"])
        self.assertFalse(transport_lock.locked())
        self.assertTrue(activity.idle())
        self.assertTrue(warnings)

    def test_legacy_main_exchange_resets_before_fast_controller_response(self):
        class SerialPort:
            def __init__(self):
                self.is_open = True
                self.timeout = None
                self.response = bytearray(b"stale\n")
                self.events = []

            def reset_input_buffer(self):
                self.events.append("reset")
                self.response.clear()

            def write(self, command):
                self.events.append(("write", command))
                self.response.extend(b"fast\n")
                return len(command)

            def flush(self):
                self.events.append("flush")

            def read(self, size=1):
                value = bytes(self.response[:size])
                del self.response[:size]
                return value

            def read_until(self, delimiter=b"\n", size=None):
                limit = len(self.response) if size is None else min(
                    size,
                    len(self.response),
                )
                available = bytes(self.response[:limit])
                delimiter_index = available.find(delimiter)
                count = limit if delimiter_index < 0 else delimiter_index + 1
                return self.read(count)

        serial_port = SerialPort()
        namespace = {
            "RUN": {"ser": serial_port},
            "serial_write_lock": threading.Lock(),
            "write_serial_control": write_serial_control,
            "read_serial_line_response": read_serial_line_response,
            "read_serial_exact_response": read_serial_exact_response,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS": 1.0,
        }
        exchange = self.compile_function(
            "_exchange_legacy_main_command",
            namespace,
        )

        self.assertEqual(exchange("TL\n", response_timeout=1.0), "fast")
        self.assertEqual(serial_port.events[:2], ["reset", ("write", b"TL\n")])
        self.assertEqual(serial_port.events.count("reset"), 1)
        self.assertEqual(serial_port.response, b"")

    def test_position_calibration_and_port_mutators_reject_logical_motion(self):
        callback_names = (
            "_run_program_calibration_all",
            "_run_program_calibration_j1",
            "_run_program_calibration_j2",
            "_run_program_calibration_j3",
            "_run_program_calibration_j4",
            "_run_program_calibration_j5",
            "_run_program_calibration_j6",
            "_run_program_calibration_j7",
            "_run_program_calibration_j8",
            "_run_program_calibration_j9",
            "updateParams",
            "calExtAxis",
            "zeroAxis7",
            "zeroAxis8",
            "zeroAxis9",
            "sendPos",
            "CalZeroPos",
            "CalRestPos",
            "SaveAndApplyCalibration",
        )

        def decorator_name(decorator):
            self.assertIsInstance(decorator, ast.Call)
            self.assertIsInstance(decorator.func, ast.Name)
            return decorator.func.id

        for callback_name in callback_names:
            decorators = self.module_functions[callback_name].decorator_list
            self.assertGreaterEqual(len(decorators), 1, callback_name)
            self.assertEqual(
                decorator_name(decorators[0]),
                "_synchronous_motion_request",
                callback_name,
            )
            self.assertEqual(
                decorator_name(decorators[1]),
                "_tracked_serial_operation",
                callback_name,
            )

        execute_row_names = {
            node.id
            for node in ast.walk(self.module_functions["executeRow"])
            if isinstance(node, ast.Name)
        }
        self.assertTrue(set(callback_names[:10]).issubset(execute_row_names))

        set_com_calls = [
            node
            for node in ast.walk(self.module_functions["setCom"])
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_acquire_motion_request"
        ]
        self.assertEqual(len(set_com_calls), 1)
        recovery_keywords = {
            keyword.arg: keyword.value
            for keyword in set_com_calls[0].keywords
        }
        self.assertIsInstance(
            recovery_keywords.get("allow_position_recovery"),
            ast.Constant,
        )
        self.assertIs(
            recovery_keywords["allow_position_recovery"].value,
            True,
        )

        set_com2_decorators = self.module_functions["setCom2"].decorator_list
        self.assertEqual(len(set_com2_decorators), 1)
        self.assertEqual(
            decorator_name(set_com2_decorators[0]),
            "_synchronous_motion_request",
        )

        gcode_start_decorators = self.module_functions[
            "MoveGcodeStartPos"
        ].decorator_list
        self.assertEqual(len(gcode_start_decorators), 1)
        self.assertEqual(
            decorator_name(gcode_start_decorators[0]),
            "_manual_motion_request",
        )

        request_position_decorators = self.module_functions[
            "requestPos"
        ].decorator_list
        self.assertEqual(len(request_position_decorators), 1)
        self.assertEqual(
            decorator_name(request_position_decorators[0]),
            "_tracked_serial_operation",
        )

        namespace = {
            "wraps": wraps,
            "_main_serial_transmit_required": lambda transmit=True: (
                transmit is not False
            ),
        }
        namespace["_synchronous_motion_request"] = self.compile_function(
            "_synchronous_motion_request",
            namespace,
        )
        namespace["_tracked_serial_operation"] = self.compile_function(
            "_tracked_serial_operation",
            namespace,
        )

        callbacks = {}
        for callback_name in callback_names:
            function = copy.deepcopy(self.module_functions[callback_name])
            module = ast.Module(body=[function], type_ignores=[])
            compiled = compile(
                ast.fix_missing_locations(module),
                str(AR4_SOURCE),
                "exec",
            )
            exec(compiled, namespace)
            callbacks[callback_name] = namespace[callback_name]

        active_lease = namespace["motion_request_registry"].acquire(
            "Existing virtual settlement"
        )
        self.assertIsNotNone(active_lease)
        for callback_name, callback in callbacks.items():
            with self.subTest(callback=callback_name):
                self.assertFalse(callback())
                self.assertTrue(
                    namespace["motion_request_registry"].owns(active_lease)
                )
        self.assertTrue(active_lease.close())

    def test_serial_worker_inherits_decorated_main_transport_reservation(self):
        transport_lock = threading.Lock()
        reservation_namespace = {
            "contextmanager": contextmanager,
            "main_serial_operation_state": threading.local(),
            "serial_lock": transport_lock,
            "SerialActivityRejected": SerialActivityRejected,
        }
        reserve = contextmanager(
            self.compile_function(
                "_reserve_main_serial_operation",
                reservation_namespace,
            )
        )
        reservation_namespace["_reserve_main_serial_operation"] = reserve
        transfer = self.compile_function(
            "_transfer_main_serial_reservation",
            reservation_namespace,
        )
        restore = self.compile_function(
            "_restore_main_serial_reservation",
            reservation_namespace,
        )
        started = []

        class Worker:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

            def start(self):
                started.append(self.kwargs)

        pending = threading.Event()
        activity = SerialActivityRegistry(("ser",))
        start_namespace = {
            "application_closing": threading.Event(),
            "controller_correction_requested": threading.Event(),
            "serial_lock": transport_lock,
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "legacy_serial_result_pending": pending,
            "live_jog_stop_requested": threading.Event(),
            "live_serial_result_pending": threading.Event(),
            "threading": SimpleNamespace(Thread=Worker),
            "run_send_serial_safe": lambda *args: None,
            "logger": SimpleNamespace(warning=lambda *args: None),
            "_transfer_main_serial_reservation": transfer,
            "_restore_main_serial_reservation": restore,
        }
        start_serial = self.compile_function(
            "start_send_serial_thread",
            start_namespace,
        )

        with reserve():
            self.assertTrue(start_serial("RP\n"))

        self.assertEqual(len(started), 1)
        self.assertTrue(pending.is_set())
        self.assertTrue(transport_lock.locked())
        self.assertTrue(activity.active("ser"))
        transport_lock.release()
        self.assertTrue(started[0]["args"][3].close())
        self.assertTrue(activity.idle())

        def fail_worker(*args, **kwargs):
            raise RuntimeError("thread construction failed")

        pending.clear()
        start_namespace["threading"] = SimpleNamespace(Thread=fail_worker)
        failing_start = self.compile_function(
            "start_send_serial_thread",
            start_namespace,
        )
        with self.assertRaisesRegex(RuntimeError, "thread construction failed"):
            with reserve():
                failing_start("RP\n")
        self.assertFalse(pending.is_set())
        self.assertFalse(transport_lock.locked())

    def test_windows_xbox_toggles_commit_only_after_auxiliary_results(self):
        assignments = {
            node.targets[0].id: ast.literal_eval(node.value)
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id
            in {
                "XBOX_AUXILIARY_PENDING_KEYS",
                "XBOX_AUXILIARY_FIXED_TOGGLE_COMMANDS",
                "XBOX_AUXILIARY_TOGGLE_RESPONSES",
            }
        }
        pending_keys = assignments["XBOX_AUXILIARY_PENDING_KEYS"]
        fixed_toggle_commands = assignments[
            "XBOX_AUXILIARY_FIXED_TOGGLE_COMMANDS"
        ]
        toggle_responses = assignments["XBOX_AUXILIARY_TOGGLE_RESPONSES"]
        fixed_exchanges = frozenset(
            exchange
            for commands in fixed_toggle_commands.values()
            for exchange in commands.values()
        )

        class Logger:
            def __init__(self):
                self.warnings = []
                self.errors = []
                self.exceptions = []

            def warning(self, *args):
                self.warnings.append(args)

            def error(self, *args):
                self.errors.append(args)

            def exception(self, *args):
                self.exceptions.append(args)

        class Port:
            def __init__(self):
                self.is_open = True
                self.timeout = 11
                self.responses = []
                self.read_sizes = []

            def read(self, size):
                self.read_sizes.append(size)
                if not self.responses:
                    return b""
                response = self.responses[0]
                chunk = response[:size]
                remainder = response[size:]
                if remainder:
                    self.responses[0] = remainder
                else:
                    self.responses.pop(0)
                return chunk

        class ImmediateThread:
            def __init__(self, target, args, daemon):
                self.target = target
                self.args = args
                self.daemon = daemon

            def start(self):
                self.target(*self.args)

        class Label:
            def __init__(self):
                self.configurations = []

            def config(self, **kwargs):
                self.configurations.append(kwargs)

        logger = Logger()
        port = Port()
        runtime = {
            "ser2": port,
            "ser2BoardProfile": (port, AUXILIARY_BOARD_NANO),
            "_grip_closed": False,
            "_pneu_open": False,
            "_grip_pending_request_id": None,
            "_pneu_pending_request_id": None,
        }
        auxiliary_lock = threading.Lock()
        activity = SerialActivityRegistry(
            ("ser", "ser2"),
            single_owner_names=("ser2",),
        )
        decorator_namespace = {
            "wraps": wraps,
            "auxiliary_serial_lock": auxiliary_lock,
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "logger": logger,
        }
        tracked_operation = self.compile_function(
            "_tracked_serial_operation",
            decorator_namespace,
        )
        writes = []
        closes = []
        exchange_namespace = {
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "XBOX_AUXILIARY_FIXED_EXCHANGES": fixed_exchanges,
            "MAX_COMMAND_LENGTH": 4096,
            "re": re,
            "normalize_auxiliary_board_profile": (
                normalize_auxiliary_board_profile
            ),
            "validate_auxiliary_output_command": (
                validate_auxiliary_output_command
            ),
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 5,
            "_write_legacy_auxiliary_command": lambda command: writes.append(
                command
            ),
            "read_serial_exact_response": read_serial_exact_response,
            "_close_serial_port": lambda *args: closes.append(args) or True,
        }
        exchange_namespace["_connected_auxiliary_board_profile"] = (
            self.compile_function(
                "_connected_auxiliary_board_profile",
                exchange_namespace,
            )
        )
        exchange_namespace["_xbox_auxiliary_expected_response"] = (
            self.compile_function(
                "_xbox_auxiliary_expected_response",
                exchange_namespace,
            )
        )
        raw_exchange = self.compile_function(
            "_exchange_xbox_auxiliary_command",
            exchange_namespace,
        )
        exchange = tracked_operation("ser2")(raw_exchange)
        result_queue = Queue()
        worker_namespace = {
            "_exchange_xbox_auxiliary_command": exchange,
            "ProtocolResponseError": ProtocolResponseError,
            "logger": logger,
            "xbox_auxiliary_event_queue": result_queue,
        }
        run_request = self.compile_function(
            "_run_xbox_auxiliary_request",
            worker_namespace,
        )
        toggle_namespace = {
            "RUN": runtime,
            "XBOX_AUXILIARY_PENDING_KEYS": pending_keys,
            "XBOX_AUXILIARY_FIXED_TOGGLE_COMMANDS": fixed_toggle_commands,
            "MotionInputError": MotionInputError,
            "auxiliary_pneumatic_output_pin": auxiliary_pneumatic_output_pin,
            "_connected_auxiliary_board_profile": exchange_namespace[
                "_connected_auxiliary_board_profile"
            ],
            "_xbox_auxiliary_expected_response": exchange_namespace[
                "_xbox_auxiliary_expected_response"
            ],
        }
        toggle_exchange = self.compile_function(
            "_xbox_auxiliary_toggle_exchange",
            toggle_namespace,
        )
        start_namespace = {
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "XBOX_AUXILIARY_PENDING_KEYS": pending_keys,
            "_xbox_auxiliary_toggle_exchange": toggle_exchange,
            "threading": SimpleNamespace(Thread=ImmediateThread),
            "_run_xbox_auxiliary_request": run_request,
            "logger": logger,
        }
        start_request = self.compile_function(
            "_start_xbox_auxiliary_request",
            start_namespace,
        )
        request_namespace = {
            "xbox_auxiliary_next_request_id": 0,
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "XBOX_AUXILIARY_PENDING_KEYS": pending_keys,
            "_start_xbox_auxiliary_request": start_request,
            "logger": logger,
        }
        request_toggle = self.compile_function(
            "_request_xbox_auxiliary_toggle",
            request_namespace,
        )
        first_label = Label()
        second_label = Label()
        closing = threading.Event()
        closing.set()
        poll_namespace = {
            "xbox_auxiliary_event_queue": result_queue,
            "Empty": Empty,
            "XBOX_AUXILIARY_PENDING_KEYS": pending_keys,
            "XBOX_AUXILIARY_TOGGLE_RESPONSES": toggle_responses,
            "RUN": runtime,
            "logger": logger,
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "application_closing": closing,
            "root": SimpleNamespace(after=lambda *args: None),
        }
        poll_results = self.compile_function(
            "_poll_xbox_auxiliary_events",
            poll_namespace,
        )

        class FailingThread:
            def __init__(self, *args, **kwargs):
                raise RuntimeError("thread construction failed")

        start_namespace["threading"] = SimpleNamespace(Thread=FailingThread)
        self.assertFalse(request_toggle("_grip_closed"))
        self.assertFalse(runtime["_grip_closed"])
        self.assertIsNone(runtime["_grip_pending_request_id"])
        start_namespace["threading"] = SimpleNamespace(Thread=ImmediateThread)

        for state_name in ("_grip_closed", "_pneu_open"):
            pending_name = pending_keys[state_name]
            auxiliary_lock.acquire()
            self.assertTrue(request_toggle(state_name))
            self.assertFalse(runtime[state_name])
            self.assertIsInstance(runtime[pending_name], int)
            poll_results()
            self.assertFalse(runtime[state_name])
            self.assertIsNone(runtime[pending_name])
            auxiliary_lock.release()

        nano_toggles = (
            ("_grip_closed", b"Servo Done", "SV0P0\n", True),
            ("_grip_closed", b"Servo Done", "SV0P50\n", False),
            ("_pneu_open", b"Done", "OFX8\n", True),
            ("_pneu_open", b"Done", "ONX8\n", False),
        )
        for state_name, response, command, expected_state in nano_toggles:
            port.responses.append(response)
            self.assertTrue(request_toggle(state_name))
            self.assertNotEqual(runtime[state_name], expected_state)
            write_count = len(writes)
            self.assertFalse(request_toggle(state_name))
            self.assertEqual(len(writes), write_count)
            poll_results()
            self.assertEqual(runtime[state_name], expected_state)
            self.assertEqual(writes[-1], command)
            self.assertEqual(port.timeout, 11)
        self.assertIn(1, port.read_sizes)

        runtime["ser2BoardProfile"] = (port, AUXILIARY_BOARD_MEGA)
        for state_name, response, command, expected_state in (
            ("_pneu_open", b"Done", "OFX28\n", True),
            ("_pneu_open", b"Done", "ONX28\n", False),
        ):
            port.responses.append(response)
            self.assertTrue(request_toggle(state_name))
            poll_results()
            self.assertEqual(runtime[state_name], expected_state)
            self.assertEqual(writes[-1], command)

        runtime["ser2BoardProfile"] = None
        with self.assertRaisesRegex(MotionInputError, "not bound"):
            request_toggle("_pneu_open")
        self.assertIsNone(runtime["_pneu_pending_request_id"])
        runtime["ser2BoardProfile"] = (port, AUXILIARY_BOARD_MEGA)

        port.responses.append(b"Servo Donejunk")
        self.assertTrue(request_toggle("_grip_closed"))
        poll_results()
        self.assertIsNone(runtime["_grip_closed"])
        self.assertTrue(closes)
        self.assertTrue(first_label.configurations)
        self.assertEqual(
            first_label.configurations[-1]["style"],
            "Alarm.TLabel",
        )

        replacement_port = Port()
        runtime["ser2"] = replacement_port
        runtime["ser2BoardProfile"] = (
            replacement_port,
            AUXILIARY_BOARD_MEGA,
        )
        write_count = len(writes)
        with self.assertRaisesRegex(ConnectionError, "connection changed"):
            exchange("SV0P0\n", b"Servo Done", port)
        self.assertEqual(len(writes), write_count)
        self.assertTrue(activity.idle())

    def test_linux_xbox_auxiliary_uses_validated_exchange_result(self):
        fixed_exchanges = frozenset((
            ("SV0P0\n", b"Servo Done"),
        ))
        calls = []

        def exchange(command, expected_response, expected_serial_port=None):
            calls.append((command, expected_response, expected_serial_port))
            return expected_response.decode("ascii")

        for selected_profile, other_profile in (
            (AUXILIARY_BOARD_NANO, AUXILIARY_BOARD_MEGA),
            (AUXILIARY_BOARD_MEGA, AUXILIARY_BOARD_NANO),
        ):
            port = SimpleNamespace(is_open=True)
            runtime = {
                "ser2": port,
                "ser2BoardProfile": (port, selected_profile),
            }
            namespace = {
                "RUN": runtime,
                "XBOX_AUXILIARY_FIXED_EXCHANGES": fixed_exchanges,
                "MAX_COMMAND_LENGTH": 4096,
                "re": re,
                "MotionInputError": MotionInputError,
                "normalize_auxiliary_board_profile": (
                    normalize_auxiliary_board_profile
                ),
                "validate_auxiliary_output_command": (
                    validate_auxiliary_output_command
                ),
                "_exchange_xbox_auxiliary_command": exchange,
            }
            namespace["_connected_auxiliary_board_profile"] = (
                self.compile_function(
                    "_connected_auxiliary_board_profile",
                    namespace,
                )
            )
            namespace["_xbox_auxiliary_expected_response"] = (
                self.compile_function(
                    "_xbox_auxiliary_expected_response",
                    namespace,
                )
            )
            send = self.compile_nested_function(
                "send_xbox_auxiliary",
                namespace,
            )

            self.assertEqual(send("SV0P0\n"), "Servo Done")
            for output_pin in AUXILIARY_BOARD_OUTPUT_PINS[selected_profile]:
                for prefix in ("ON", "OF"):
                    command = f"{prefix}X{output_pin}\n"
                    with self.subTest(
                        selected_profile=selected_profile,
                        command=command,
                    ):
                        self.assertEqual(send(command), "Done")
            call_count = len(calls)
            for prefix in ("ON", "OF"):
                for output_pin in AUXILIARY_BOARD_OUTPUT_PINS[other_profile]:
                    with self.subTest(
                        rejected_profile=selected_profile,
                        prefix=prefix,
                        output_pin=output_pin,
                    ):
                        with self.assertRaises(MotionInputError):
                            send(f"{prefix}X{output_pin}\n")
            self.assertEqual(len(calls), call_count)

        configured_calls = []
        runtime = {"xboxUse": 0}
        polls = []

        def get_gamepad():
            polls.append(True)
            if len(polls) == 1:
                return [SimpleNamespace(code="BTN_SELECT", state=1)]
            runtime["xboxUse"] = 0
            return []

        thread_namespace = {
            "RUN": runtime,
            "application_closing": threading.Event(),
            "get_gamepad": get_gamepad,
            "DO1offEntryField": SimpleNamespace(get=lambda: "42"),
            "DO1onEntryField": SimpleNamespace(get=lambda: "41"),
            "send_xbox_auxiliary": (
                lambda command: configured_calls.append(command) or True
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ChgDis": lambda value: None,
        }
        threadxbox = self.compile_nested_function(
            "threadxbox",
            thread_namespace,
        )
        threadxbox()
        self.assertEqual(configured_calls, ["OFX42\n"])

        configured_calls.clear()
        polls.clear()
        runtime["xboxUse"] = 0
        closing = threading.Event()

        def close_during_poll():
            closing.set()
            return [SimpleNamespace(code="BTN_SELECT", state=1)]

        thread_namespace["application_closing"] = closing
        thread_namespace["get_gamepad"] = close_during_poll
        threadxbox = self.compile_nested_function(
            "threadxbox",
            thread_namespace,
        )
        threadxbox()
        self.assertEqual(configured_calls, [])

        def fail_timeout(
            command,
            expected_response,
            expected_serial_port=None,
        ):
            raise SerialTransportTimeout("auxiliary acknowledgement timed out")

        namespace["_exchange_xbox_auxiliary_command"] = fail_timeout
        with self.assertRaises(SerialTransportTimeout):
            send("ONX28\n")

        def fail_malformed(
            command,
            expected_response,
            expected_serial_port=None,
        ):
            raise ProtocolResponseError("malformed auxiliary acknowledgement")

        namespace["_exchange_xbox_auxiliary_command"] = fail_malformed
        with self.assertRaises(ProtocolResponseError):
            send("SV0P0\n")
        with self.assertRaises(MotionInputError):
            send("BAD\n")
        with self.assertRaises(MotionInputError):
            send("ONX8\n")

    def test_all_auxiliary_output_paths_share_handle_bound_board_validation(self):
        direct_output_functions = {
            f"DO{output_index}{state}"
            for output_index in range(1, 7)
            for state in ("on", "off")
        }
        for function_name in direct_output_functions:
            called_names = {
                node.func.id
                for node in ast.walk(self.module_functions[function_name])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
            }
            self.assertIn("_write_legacy_auxiliary_command", called_names)

        for function_name in (
            "_execute_row_auxiliary_command",
            "_exchange_xbox_auxiliary_command",
        ):
            called_names = {
                node.func.id
                for node in ast.walk(self.module_functions[function_name])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
            }
            self.assertIn("_write_legacy_auxiliary_command", called_names)

        send_xbox_auxiliary = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef)
            and node.name == "send_xbox_auxiliary"
        )
        send_calls = {
            node.func.id
            for node in ast.walk(send_xbox_auxiliary)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
        }
        self.assertIn("_exchange_xbox_auxiliary_command", send_calls)

        class Port:
            def __init__(self):
                self.is_open = True
                self.commands = []

            def reset_input_buffer(self):
                pass

            def write(self, command):
                self.commands.append(command)
                return len(command)

            def flush(self):
                pass

        port = Port()
        runtime = {
            "ser2": port,
            "ser2BoardProfile": (port, AUXILIARY_BOARD_NANO),
        }
        namespace = {
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "normalize_auxiliary_board_profile": (
                normalize_auxiliary_board_profile
            ),
            "validate_auxiliary_output_command": (
                validate_auxiliary_output_command
            ),
            "write_serial_control": write_serial_control,
            "auxiliary_serial_write_lock": threading.Lock(),
            "_clear_auxiliary_board_profile": lambda serial_port=None: True,
        }
        namespace["_connected_auxiliary_board_profile"] = self.compile_function(
            "_connected_auxiliary_board_profile",
            namespace,
        )
        write_output = self.compile_function(
            "_write_legacy_auxiliary_command",
            namespace,
        )

        write_output("ONX8\n")
        self.assertEqual(port.commands, [b"ONX8\n"])
        with self.assertRaises(MotionInputError):
            write_output("ONX28\n")
        self.assertEqual(port.commands, [b"ONX8\n"])

        runtime["ser2BoardProfile"] = (port, AUXILIARY_BOARD_MEGA)
        write_output("OFX53\n")
        self.assertEqual(port.commands[-1], b"OFX53\n")
        with self.assertRaises(MotionInputError):
            write_output("OFX13\n")
        self.assertEqual(port.commands[-1], b"OFX53\n")

        runtime["ser2BoardProfile"] = None
        with self.assertRaisesRegex(MotionInputError, "not bound"):
            write_output("ONX28\n")
        write_output("SV0P0\n")
        self.assertEqual(port.commands[-1], b"SV0P0\n")

    def test_auxiliary_connection_binds_and_clears_one_board_profile(self):
        class Port:
            def __init__(self, name):
                self.name = name
                self.is_open = True
                self.close_count = 0

            def close(self):
                self.close_count += 1
                self.is_open = False

        existing = Port("existing")
        replacements = []

        def open_serial(**kwargs):
            replacement = Port(kwargs["port"])
            replacements.append((replacement, kwargs))
            return replacement

        runtime = {
            "ser2": existing,
            "ser2BoardProfile": (existing, AUXILIARY_BOARD_NANO),
        }
        namespace = {
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "normalize_auxiliary_board_profile": (
                normalize_auxiliary_board_profile
            ),
            "serial": SimpleNamespace(Serial=open_serial),
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 5.0,
            "SERIAL_WRITE_TIMEOUT_SECONDS": 2.0,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        namespace["_bind_auxiliary_board_profile"] = self.compile_function(
            "_bind_auxiliary_board_profile",
            namespace,
        )
        replace = self.compile_function(
            "_replace_auxiliary_serial",
            namespace,
        )
        connected_profile = self.compile_function(
            "_connected_auxiliary_board_profile",
            namespace,
        )
        close_serial = self.compile_function(
            "_close_serial_port",
            namespace,
        )

        with self.assertRaises(MotionInputError):
            replace("COM7", "Unknown")
        self.assertTrue(existing.is_open)
        self.assertEqual(existing.close_count, 0)
        self.assertEqual(replacements, [])

        replacement = replace("COM7", AUXILIARY_BOARD_MEGA)

        self.assertFalse(existing.is_open)
        self.assertEqual(existing.close_count, 1)
        self.assertIs(runtime["ser2"], replacement)
        self.assertEqual(
            runtime["ser2BoardProfile"],
            (replacement, AUXILIARY_BOARD_MEGA),
        )
        self.assertEqual(connected_profile(replacement), AUXILIARY_BOARD_MEGA)
        self.assertEqual(replacements[0][1]["baudrate"], 9600)

        self.assertTrue(close_serial("ser2", "test cleanup"))
        self.assertIsNone(runtime["ser2"])
        self.assertIsNone(runtime["ser2BoardProfile"])

    def test_windows_xbox_buttons_delegate_toggle_state_to_tk(self):
        toggle_contract = {
            "_toggle_servo_gripper": "_grip_closed",
            "_toggle_pneu_gripper": "_pneu_open",
        }
        nested_functions = {
            node.name: node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef)
            and node.name in toggle_contract
        }

        self.assertEqual(set(nested_functions), set(toggle_contract))
        for function_name, state_name in toggle_contract.items():
            tk_calls = [
                node
                for node in ast.walk(nested_functions[function_name])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_tk_call"
            ]
            self.assertEqual(len(tk_calls), 1)
            self.assertIsInstance(tk_calls[0].args[0], ast.Name)
            self.assertEqual(
                tk_calls[0].args[0].id,
                "_request_xbox_auxiliary_toggle",
            )
            self.assertIsInstance(tk_calls[0].args[1], ast.Constant)
            self.assertEqual(tk_calls[0].args[1].value, state_name)
            state_assignments = [
                node
                for node in ast.walk(nested_functions[function_name])
                if isinstance(node, ast.Assign)
                and any(
                    isinstance(target, ast.Subscript)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "RUN"
                    for target in node.targets
                )
            ]
            self.assertEqual(state_assignments, [])

    def test_windows_xbox_teach_reports_tk_scheduler_rejection(self):
        warnings = []
        namespace = {
            "_tk_call": lambda *args: False,
            "logger": SimpleNamespace(
                warning=lambda *args: warnings.append(args)
            ),
        }
        teach_position = self.compile_nested_function(
            "_teach_position",
            namespace,
        )

        teach_position()

        self.assertEqual(len(warnings), 1)
        self.assertIn("teach-position", warnings[0][0])

    def test_windows_xbox_motion_schedule_distinguishes_shutdown(self):
        closing = threading.Event()
        namespace = {
            "application_closing": closing,
            "LiveMotionScheduleResult": LiveMotionScheduleResult,
            "_tk_call": lambda *args: self.fail(
                "shutdown scheduling must not touch Tk"
            ),
        }
        schedule_motion = self.compile_nested_function(
            "_schedule_xbox_motion",
            namespace,
        )

        closing.set()
        self.assertIs(
            schedule_motion(
                0,
                lambda: None,
                lambda failure: self.fail(
                    f"shutdown scheduling reported {failure!r}"
                ),
            ),
            LiveMotionScheduleResult.CANCELLED,
        )

    def test_windows_xbox_motion_reports_deferred_tk_registration_failure(self):
        closing = threading.Event()
        tk_jobs = []
        failures = []

        class Root:
            @staticmethod
            def after(delay_ms, callback):
                raise RuntimeError("Tk registration failed")

        namespace = {
            "application_closing": closing,
            "LiveMotionScheduleResult": LiveMotionScheduleResult,
            "root": Root(),
            "_tk_call": (
                lambda callback, *args: tk_jobs.append((callback, args))
                or True
            ),
        }
        schedule_motion = self.compile_nested_function(
            "_schedule_xbox_motion",
            namespace,
        )

        self.assertTrue(
            schedule_motion(
                60,
                lambda: self.fail("failed registration must not run motion"),
                failures.append,
            )
        )
        self.assertEqual(len(tk_jobs), 1)
        tk_jobs[0][0](*tk_jobs[0][1])

        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], RuntimeError)
        self.assertIn("Tk registration failed", str(failures[0]))

    def test_windows_xbox_motion_reports_shutdown_after_tk_admission(self):
        closing = threading.Event()
        tk_jobs = []
        failures = []
        motion_callbacks = []
        namespace = {
            "application_closing": closing,
            "LiveMotionScheduleResult": LiveMotionScheduleResult,
            "_tk_call": (
                lambda callback, *args: tk_jobs.append((callback, args))
                or True
            ),
        }
        schedule_motion = self.compile_nested_function(
            "_schedule_xbox_motion",
            namespace,
        )

        self.assertTrue(
            schedule_motion(
                60,
                lambda: motion_callbacks.append(True),
                failures.append,
            )
        )
        closing.set()
        tk_jobs[0][0](*tk_jobs[0][1])

        self.assertEqual(
            failures,
            [LiveMotionScheduleResult.CANCELLED],
        )
        self.assertEqual(motion_callbacks, [])

    def test_windows_xbox_switches_delegate_to_admission_arbiters(self):
        expected_arbiters = {
            "_request_switch": "joint_xbox_arbiter",
            "_request_switch_cart": "cartesian_xbox_arbiter",
            "_request_switch_tool": "tool_xbox_arbiter",
        }
        switch_functions = {
            node.name: node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef)
            and node.name in expected_arbiters
        }
        self.assertEqual(set(switch_functions), set(expected_arbiters))
        for function_name, arbiter_name in expected_arbiters.items():
            calls = [
                node
                for node in ast.walk(switch_functions[function_name])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "request"
                and isinstance(node.func.value, ast.Name)
            ]
            self.assertEqual(len(calls), 1, function_name)
            self.assertEqual(calls[0].func.value.id, arbiter_name)

        arbiter_constructions = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeferredLiveMotionArbiter"
        ]
        self.assertEqual(len(arbiter_constructions), 3)

    def test_connection_switch_releases_transport_when_port_selection_fails(self):
        class Selection:
            @staticmethod
            def get():
                raise RuntimeError("selection unavailable")

        class Label:
            def config(self, **kwargs):
                pass

        transport_lock = threading.Lock()
        activity = SerialActivityRegistry(("ser",))
        logged_exceptions = []
        namespace = {
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "serial_lock": transport_lock,
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: logged_exceptions.append(args),
            ),
            "almStatusLab": Label(),
            "almStatusLab2": Label(),
            "CAL": {},
            "RUN": {"ser": None},
            "com1SelectedValue": Selection(),
            "tab8": SimpleNamespace(ElogView=SimpleNamespace(get=lambda *args: ())),
            "pickle": SimpleNamespace(
                dump=lambda *args: (_ for _ in ()).throw(
                    OSError("error-log storage unavailable")
                )
            ),
            "open": lambda *args, **kwargs: object(),
            "END": "end",
        }
        namespace["_release_async_main_serial_transport"] = self.compile_function(
            "_release_async_main_serial_transport",
            namespace,
        )
        set_com = self.compile_function("setCom", namespace)

        self.assertFalse(set_com())
        self.assertTrue(transport_lock.acquire(blocking=False))
        transport_lock.release()
        self.assertTrue(activity.idle())
        self.assertTrue(
            any(
                "persist the controller startup error log" in args[0]
                for args in logged_exceptions
            )
        )

    def test_connection_recovery_holds_full_async_ownership_until_success(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.timeout = None

            @staticmethod
            def reset_input_buffer():
                pass

            @staticmethod
            def reset_output_buffer():
                pass

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        class StartupResult:
            def __init__(self):
                self.position = VALID_CONTROLLER_POSITION
                self.auxiliary_serial = None
                self.auxiliary_error = None

        port = Port()
        callbacks = {}
        transport_lock = threading.Lock()
        activity = SerialActivityRegistry(("ser",))
        motion_registry = MotionRequestRegistry()
        recovery_required = threading.Event()
        recovery_required.set()
        calibration = {"comPort": "COM0"}
        startup_request = SimpleNamespace(auxiliary_port=None)

        def startup_with_spinner(
            root,
            request,
            on_finished,
            on_timeout,
            on_abandoned,
            timeout,
        ):
            callbacks.update(
                finished=on_finished,
                timeout=on_timeout,
                abandoned=on_abandoned,
            )
            return SimpleNamespace()

        def apply_startup(*args):
            self.assertTrue(recovery_required.is_set())
            self.assertTrue(motion_registry.active)
            recovery_required.clear()
            return VALID_CONTROLLER_POSITION

        first_label = Label()
        second_label = Label()
        namespace = {
            "application_closing": threading.Event(),
            "controller_position_resynchronization_required": recovery_required,
            "motion_request_registry": motion_registry,
            "serial_lock": transport_lock,
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "logger": SimpleNamespace(
                info=lambda *args: None,
                warning=lambda *args: None,
                error=lambda *args, **kwargs: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "CAL": calibration,
            "RUN": {"ser": None},
            "com1SelectedValue": SimpleNamespace(get=lambda: " COM7 "),
            "serial": SimpleNamespace(Serial=lambda **kwargs: port),
            "SERIAL_WRITE_TIMEOUT_SECONDS": 5.0,
            "SERIAL_STARTUP_READ_TIMEOUT_SECONDS": 2.0,
            "time": SimpleNamespace(sleep=lambda seconds: None),
            "_prepare_controller_startup": lambda: (startup_request, {}, {}),
            "ControllerStartupResult": StartupResult,
            "startup_with_spinner": startup_with_spinner,
            "root": SimpleNamespace(),
            "_apply_controller_startup_result": apply_startup,
            "_request_startup_auxiliary_cleanup": lambda serial_port: None,
            "_poll_failed_controller_close": lambda *args: self.fail(
                "successful startup must not enter cleanup"
            ),
            "_abandon_failed_controller_startup": lambda *args: self.fail(
                "successful startup must not be abandoned"
            ),
            "tab8": SimpleNamespace(ElogView=SimpleNamespace(get=lambda *args: ())),
            "pickle": SimpleNamespace(dump=lambda *args: None),
            "open": lambda *args, **kwargs: object(),
            "END": "end",
        }
        namespace["_release_async_main_serial_transport"] = self.compile_function(
            "_release_async_main_serial_transport",
            namespace,
        )
        set_com = self.compile_function("setCom", namespace)

        self.assertTrue(set_com())
        self.assertTrue(transport_lock.locked())
        self.assertFalse(activity.idle())
        self.assertTrue(motion_registry.active)
        self.assertEqual(calibration["comPort"], "COM0")
        self.assertIsNone(motion_registry.acquire("competing motion"))

        callbacks["finished"](StartupResult(), False)

        self.assertFalse(transport_lock.locked())
        self.assertTrue(activity.idle())
        self.assertFalse(motion_registry.active)
        self.assertFalse(recovery_required.is_set())
        self.assertEqual(calibration["comPort"], "COM7")
        self.assertEqual(first_label.text, "SYSTEM READY; AUXILIARY CONTROLLER NOT CONFIGURED")
        self.assertEqual(second_label.text, first_label.text)

    def test_auxiliary_connection_wrapper_enforces_logical_and_shutdown_admission(self):
        def build_namespace(*, closing=False, active=False):
            motion_registry = MotionRequestRegistry()
            if active:
                self.assertIsNotNone(motion_registry.acquire("existing motion"))
            closing_event = threading.Event()
            if closing:
                closing_event.set()
            replacement_calls = []
            namespace = {
                "wraps": wraps,
                "motion_request_registry": motion_registry,
                "application_closing": closing_event,
                "auxiliary_serial_lock": threading.Lock(),
                "CAL": {
                    "com2Port": "COM2",
                    "auxiliaryBoard": AUXILIARY_BOARD_MEGA,
                },
                "RUN": {"ser2": None},
                "com2SelectedValue": SimpleNamespace(
                    get=lambda: "COM9",
                    set=lambda value: None,
                ),
                "auxiliaryBoardSelectedValue": SimpleNamespace(
                    get=lambda: AUXILIARY_BOARD_NANO,
                    set=lambda value: None,
                ),
                "_replace_auxiliary_serial": (
                    lambda port, board: replacement_calls.append((port, board))
                ),
                "_close_serial_port": lambda *args: True,
                "logger": SimpleNamespace(
                    info=lambda *args: None,
                    warning=lambda *args: None,
                    error=lambda *args: None,
                    exception=lambda *args: None,
                ),
                "tab8": SimpleNamespace(ElogView=SimpleNamespace(get=lambda *args: ())),
                "pickle": SimpleNamespace(dump=lambda *args: None),
                "open": lambda *args, **kwargs: object(),
                "END": "end",
            }
            namespace["_synchronous_motion_request"] = self.compile_function(
                "_synchronous_motion_request",
                namespace,
            )
            callback = self.compile_function(
                "setCom2",
                namespace,
                preserve_decorators=True,
            )
            return callback, namespace, replacement_calls

        for closing, active in ((True, False), (False, True)):
            with self.subTest(closing=closing, active=active):
                callback, namespace, replacement_calls = build_namespace(
                    closing=closing,
                    active=active,
                )
                self.assertFalse(callback())
                self.assertEqual(replacement_calls, [])
                self.assertFalse(namespace["auxiliary_serial_lock"].locked())
                self.assertEqual(namespace["CAL"]["com2Port"], "COM2")

    def test_auxiliary_connection_transaction_commits_only_after_success(self):
        class Selection:
            def __init__(self, value):
                self.value = value
                self.set_calls = []

            def get(self):
                return self.value

            def set(self, value):
                self.value = value
                self.set_calls.append(value)

        def run_case(replacement_error=None, disable_close=True, disable=False):
            port_selection = Selection("None" if disable else "COM9")
            board_selection = Selection(
                AUXILIARY_BOARD_NONE if disable else AUXILIARY_BOARD_NANO
            )
            calibration = {
                "com2Port": "COM2",
                "auxiliaryBoard": AUXILIARY_BOARD_MEGA,
            }
            runtime = {"ser2": SimpleNamespace(is_open=True) if disable else None}
            replacement_calls = []
            persistence_calls = []

            def replace(port, board):
                replacement_calls.append((port, board))
                if replacement_error is not None:
                    raise replacement_error

            namespace = {
                "wraps": wraps,
                "motion_request_registry": MotionRequestRegistry(),
                "application_closing": threading.Event(),
                "auxiliary_serial_lock": threading.Lock(),
                "CAL": calibration,
                "RUN": runtime,
                "com2SelectedValue": port_selection,
                "auxiliaryBoardSelectedValue": board_selection,
                "_replace_auxiliary_serial": replace,
                "_close_serial_port": lambda *args: disable_close,
                "_retain_calibration_persistence_retry": (
                    lambda: persistence_calls.append(dict(calibration)) or True
                ),
                "logger": SimpleNamespace(
                    info=lambda *args: None,
                    warning=lambda *args: None,
                    error=lambda *args: None,
                    exception=lambda *args: None,
                ),
                "tab8": SimpleNamespace(ElogView=SimpleNamespace(get=lambda *args: ())),
                "pickle": SimpleNamespace(dump=lambda *args: None),
                "open": lambda *args, **kwargs: object(),
                "END": "end",
            }
            namespace["_synchronous_motion_request"] = self.compile_function(
                "_synchronous_motion_request",
                namespace,
            )
            callback = self.compile_function(
                "setCom2",
                namespace,
                preserve_decorators=True,
            )
            result = callback()
            self.assertFalse(namespace["motion_request_registry"].active)
            self.assertFalse(namespace["auxiliary_serial_lock"].locked())
            return (
                result,
                calibration,
                port_selection,
                board_selection,
                replacement_calls,
                persistence_calls,
            )

        success = run_case()
        self.assertTrue(success[0])
        self.assertEqual(success[1]["com2Port"], "COM9")
        self.assertEqual(success[1]["auxiliaryBoard"], AUXILIARY_BOARD_NANO)
        self.assertEqual(
            success[4],
            [("COM9", AUXILIARY_BOARD_NANO)],
        )
        self.assertEqual(
            success[5],
            [{"com2Port": "COM9", "auxiliaryBoard": AUXILIARY_BOARD_NANO}],
        )

        open_failure = run_case(replacement_error=OSError("open failed"))
        self.assertFalse(open_failure[0])
        self.assertEqual(open_failure[1]["com2Port"], "COM2")
        self.assertEqual(open_failure[1]["auxiliaryBoard"], AUXILIARY_BOARD_MEGA)
        self.assertEqual(open_failure[2].set_calls, ["COM2"])
        self.assertEqual(open_failure[3].set_calls, [AUXILIARY_BOARD_MEGA])
        self.assertEqual(open_failure[5], [])

        close_failure = run_case(disable=True, disable_close=False)
        self.assertFalse(close_failure[0])
        self.assertEqual(close_failure[1]["com2Port"], "COM2")
        self.assertEqual(close_failure[1]["auxiliaryBoard"], AUXILIARY_BOARD_MEGA)
        self.assertEqual(close_failure[2].set_calls, ["COM2"])
        self.assertEqual(close_failure[3].set_calls, [AUXILIARY_BOARD_MEGA])
        self.assertEqual(close_failure[5], [])

    def test_auxiliary_connection_persistence_retries_and_restores_on_restart(self):
        class Root:
            def __init__(self):
                self.jobs = {}
                self.next_job = 0

            def after(self, delay, callback):
                self.next_job += 1
                job = f"job-{self.next_job}"
                self.jobs[job] = (delay, callback)
                return job

            def after_cancel(self, job):
                self.jobs.pop(job, None)

            def run_next(self):
                job = next(iter(self.jobs))
                _, callback = self.jobs.pop(job)
                return callback()

        class Selection:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

            def set(self, value):
                self.value = value

        calibration_tree = ast.parse(
            CALIBRATION_SOURCE.read_text(encoding="utf-8"),
            filename=str(CALIBRATION_SOURCE),
        )
        persistence_functions = [
            copy.deepcopy(node)
            for node in calibration_tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name in {"save_calibration", "load_calibration"}
        ]
        persistence_namespace = {
            "os": os,
            "json": json,
            "logger": SimpleNamespace(
                debug=lambda *args: None,
                info=lambda *args: None,
                warning=lambda *args: None,
                error=lambda *args: None,
            ),
            "convert_calibration": lambda: None,
        }
        exec(
            compile(
                ast.fix_missing_locations(
                    ast.Module(body=persistence_functions, type_ignores=[])
                ),
                str(CALIBRATION_SOURCE),
                "exec",
            ),
            persistence_namespace,
        )
        save_file = persistence_namespace["save_calibration"]
        load_file = persistence_namespace["load_calibration"]

        with tempfile.TemporaryDirectory() as directory:
            calibration_file = Path(directory) / "ARconfig.json"
            outcomes = [False, True]
            root = Root()
            calibration = {
                "com2Port": "COM2",
                "auxiliaryBoard": AUXILIARY_BOARD_MEGA,
            }

            def persist(values):
                if outcomes.pop(0) is False:
                    return False
                return save_file(values, str(calibration_file))

            namespace = {
                "_calibration_save_job": None,
                "_calibration_dirty": False,
                "CALIBRATION_SAVE_DEBOUNCE_MS": 250,
                "CAL": calibration,
                "RUN": {"ser2": None},
                "save_calibration": persist,
                "application_closing": threading.Event(),
                "root": root,
                "tk": SimpleNamespace(TclError=RuntimeError),
                "logger": SimpleNamespace(
                    info=lambda *args: None,
                    warning=lambda *args: None,
                    error=lambda *args: None,
                    exception=lambda *args: None,
                ),
                "auxiliary_serial_lock": threading.Lock(),
                "com2SelectedValue": Selection("COM9"),
                "auxiliaryBoardSelectedValue": Selection(AUXILIARY_BOARD_NANO),
                "_replace_auxiliary_serial": lambda *args: None,
                "_close_serial_port": lambda *args: True,
                "tab8": SimpleNamespace(ElogView=SimpleNamespace(get=lambda *args: ())),
                "pickle": SimpleNamespace(dump=lambda *args: None),
                "open": lambda *args, **kwargs: object(),
                "END": "end",
            }
            namespace["_write_pending_calibration"] = self.compile_function(
                "_write_pending_calibration",
                namespace,
            )
            namespace["_schedule_calibration_save"] = self.compile_function(
                "_schedule_calibration_save",
                namespace,
            )
            namespace["_retain_calibration_persistence_retry"] = self.compile_function(
                "_retain_calibration_persistence_retry",
                namespace,
            )
            set_auxiliary = self.compile_function("setCom2", namespace)

            self.assertTrue(set_auxiliary())
            self.assertTrue(namespace["_calibration_dirty"])
            self.assertEqual(len(root.jobs), 1)
            self.assertFalse(root.run_next())
            self.assertTrue(namespace["_calibration_dirty"])
            self.assertEqual(len(root.jobs), 1)
            self.assertTrue(root.run_next())
            self.assertFalse(namespace["_calibration_dirty"])

            restored = load_file(
                str(calibration_file),
                allow_fallback=False,
            )
            self.assertEqual(restored["com2Port"], "COM9")
            self.assertEqual(restored["auxiliaryBoard"], AUXILIARY_BOARD_NANO)

    def test_auxiliary_replacement_retains_open_handle_after_cleanup_failure(self):
        class Replacement:
            def __init__(self):
                self.is_open = True
                self.close_count = 0

            def close(self):
                self.close_count += 1
                raise OSError("close failed")

        replacement = Replacement()
        cleared = []
        runtime = {"ser2": None}
        namespace = {
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "serial": SimpleNamespace(Serial=lambda **kwargs: replacement),
            "SERIAL_WRITE_TIMEOUT_SECONDS": 5.0,
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 5.0,
            "_close_serial_port": lambda *args: True,
            "_bind_auxiliary_board_profile": lambda *args: (_ for _ in ()).throw(
                RuntimeError("binding failed")
            ),
            "_clear_auxiliary_board_profile": cleared.append,
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        replace = self.compile_function("_replace_auxiliary_serial", namespace)

        with self.assertRaisesRegex(
            OSError,
            "replacement remained open",
        ):
            replace("COM9", AUXILIARY_BOARD_NANO)

        self.assertIs(runtime["ser2"], replacement)
        self.assertTrue(replacement.is_open)
        self.assertEqual(replacement.close_count, 1)
        self.assertEqual(cleared, [replacement])

    def test_controller_startup_aborts_and_closes_after_buffer_reset_failure(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.input_reset_count = 0
                self.output_reset_count = 0
                self.close_count = 0

            def reset_input_buffer(self):
                self.input_reset_count += 1

            def reset_output_buffer(self):
                self.output_reset_count += 1
                raise OSError("output reset failed")

            def close(self):
                self.close_count += 1
                self.is_open = False

        class Label:
            def config(self, **kwargs):
                pass

        class Logger:
            def __init__(self):
                self.exceptions = []

            def info(self, *args):
                pass

            def warning(self, *args):
                pass

            def error(self, *args):
                pass

            def exception(self, *args):
                self.exceptions.append(args)

        serial_port = Port()
        transport_lock = threading.Lock()
        activity = SerialActivityRegistry(("ser",))
        logger = Logger()
        runtime = {"ser": None}
        namespace = {
            "application_closing": threading.Event(),
            "serial_lock": transport_lock,
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "logger": logger,
            "almStatusLab": Label(),
            "almStatusLab2": Label(),
            "CAL": {},
            "RUN": runtime,
            "com1SelectedValue": SimpleNamespace(get=lambda: "COM1"),
            "serial": SimpleNamespace(Serial=lambda **kwargs: serial_port),
            "SERIAL_WRITE_TIMEOUT_SECONDS": 5,
            "time": SimpleNamespace(sleep=lambda seconds: None),
            "_prepare_controller_startup": lambda: (_ for _ in ()).throw(
                AssertionError("startup request must not follow reset failure")
            ),
            "tab8": SimpleNamespace(
                ElogView=SimpleNamespace(get=lambda *args: ())
            ),
            "pickle": SimpleNamespace(dump=lambda *args: None),
            "open": lambda *args, **kwargs: object(),
            "END": "end",
            "root": SimpleNamespace(after=lambda *args: None),
            "SERIAL_SHUTDOWN_RETRY_MS": 1000,
        }
        namespace["_close_serial_port"] = self.compile_function(
            "_close_serial_port",
            namespace,
        )
        namespace["_release_async_main_serial_transport"] = self.compile_function(
            "_release_async_main_serial_transport",
            namespace,
        )
        namespace["_close_failed_controller_startup"] = self.compile_function(
            "_close_failed_controller_startup",
            namespace,
        )
        namespace["_poll_failed_controller_close"] = self.compile_function(
            "_poll_failed_controller_close",
            namespace,
        )
        set_com = self.compile_function("setCom", namespace)

        self.assertFalse(set_com())
        self.assertEqual(serial_port.input_reset_count, 1)
        self.assertEqual(serial_port.output_reset_count, 1)
        self.assertEqual(serial_port.close_count, 1)
        self.assertIsNone(runtime["ser"])
        self.assertFalse(transport_lock.locked())
        self.assertTrue(activity.idle())
        self.assertTrue(logger.exceptions)

    def test_thread_construction_failures_release_every_reserved_lock(self):
        def fail_thread(*args, **kwargs):
            raise RuntimeError("thread construction failed")

        drive_lock = threading.Lock()
        drive_namespace = {
            "drive_lock": drive_lock,
            "logger": SimpleNamespace(info=lambda *args: None),
            "threading": SimpleNamespace(Thread=fail_thread),
            "run_driveMotorsJ_safe": lambda *args: None,
        }
        start_drive = self.compile_function(
            "start_driveMotorsJ_thread",
            drive_namespace,
        )
        with self.assertRaisesRegex(RuntimeError, "thread construction failed"):
            start_drive()
        self.assertFalse(drive_lock.locked())

        auxiliary_lock = threading.Lock()
        auxiliary_requested = threading.Event()
        auxiliary_requested.set()
        auxiliary_events = Queue()
        auxiliary_result_event = threading.Event()
        auxiliary_activity = SerialActivityRegistry(("ser2",))
        auxiliary_namespace = {
            "auxiliary_stop_requested": auxiliary_requested,
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_pending_request_id": 1,
            "auxiliary_stop_active_request_id": None,
            "auxiliary_stop_owner_waiting": False,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": auxiliary_result_event,
            "auxiliary_stop_injected_event": threading.Event(),
            "auxiliary_serial_event_queue": auxiliary_events,
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "RUN": {"offlineMode": False},
            "auxiliary_serial_lock": auxiliary_lock,
            "serial_activity_registry": auxiliary_activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "threading": SimpleNamespace(Thread=fail_thread),
            "_run_auxiliary_stop_safe": lambda *args: None,
        }
        dispatch_stop = self.compile_function(
            "_try_dispatch_auxiliary_stop",
            auxiliary_namespace,
        )
        self.assertTrue(dispatch_stop())
        self.assertFalse(auxiliary_lock.locked())
        self.assertFalse(auxiliary_requested.is_set())
        self.assertTrue(auxiliary_activity.idle())
        self.assertIsNone(auxiliary_namespace["auxiliary_stop_active_request_id"])
        self.assertEqual(
            auxiliary_events.get_nowait(),
            (
                "failed",
                1,
                "unable to start auxiliary stop worker: thread construction failed",
            ),
        )

        serial_lock = threading.Lock()
        serial_activity = SerialActivityRegistry(("ser",))
        pending = threading.Event()
        live_pending = threading.Event()
        live_stop = threading.Event()
        serial_namespace = {
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "controller_correction_requested": SimpleNamespace(is_set=lambda: False),
            "serial_lock": serial_lock,
            "serial_activity_registry": serial_activity,
            "SerialActivityRejected": SerialActivityRejected,
            "legacy_serial_result_pending": pending,
            "live_jog_stop_requested": live_stop,
            "live_serial_result_pending": live_pending,
            "threading": SimpleNamespace(Thread=fail_thread),
            "run_send_serial_safe": lambda *args: None,
            "logger": SimpleNamespace(warning=lambda *args: None),
            "_transfer_main_serial_reservation": lambda: False,
            "_restore_main_serial_reservation": lambda: (_ for _ in ()).throw(
                AssertionError("no inherited reservation should require restoration")
            ),
        }
        start_serial = self.compile_function(
            "start_send_serial_thread",
            serial_namespace,
        )
        with self.assertRaisesRegex(RuntimeError, "thread construction failed"):
            start_serial("RP\n", live_jog=True)
        self.assertFalse(serial_lock.locked())
        self.assertFalse(pending.is_set())
        self.assertFalse(live_pending.is_set())
        self.assertFalse(live_stop.is_set())
        self.assertTrue(serial_activity.idle())

        offline_lock = threading.Lock()
        mode_lock = threading.Lock()
        old_stop = threading.Event()
        offline_namespace = {
            "offline_live_jog_lock": offline_lock,
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_stop_event": old_stop,
            "offline_live_jog_operation": None,
            "offline_live_jog_pose_snapshot": None,
            "application_closing": threading.Event(),
            "virtual_motion_event_queue": Queue(),
            "threading": SimpleNamespace(
                Event=threading.Event,
                Thread=fail_thread,
            ),
            "RUN": {"liveJog": False, "VR_angles": [0.0] * 6},
            "MotionInputError": MotionInputError,
            "_validated_virtual_six_vector": (
                lambda values, label: tuple(values)
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        start_offline = self.compile_function(
            "_start_offline_live_jog",
            offline_namespace,
        )
        with self.assertRaisesRegex(RuntimeError, "thread construction failed"):
            start_offline(mode_lock, lambda *args: None, ())
        self.assertFalse(offline_lock.locked())
        self.assertFalse(mode_lock.locked())
        self.assertFalse(offline_namespace["RUN"]["liveJog"])

    def test_every_direct_serial_user_participates_in_shutdown_tracking(self):
        owner_managed = {
            # Profile helpers validate the active auxiliary handle but perform no
            # transport I/O; callers own or delegate the associated operation.
            "_bind_auxiliary_board_profile": {"ser2"},
            "_connected_auxiliary_board_profile": {"ser2"},
            # The failed-start activity lease and serial lock remain reserved
            # until close and cleanup release both ownership layers.
            "_close_failed_controller_startup": {"ser"},
            "_close_startup_auxiliary": {"ser2"},
            # Startup state helpers either delegate auxiliary cleanup or retain
            # the main activity lease transferred by the enclosing admission.
            "_clear_unavailable_startup_auxiliary": {"ser2"},
            "finish_startup": {"ser"},
            "_exchange_auxiliary_line": {"ser2"},
            "_exchange_serial_line": {"ser"},
            # The control reservation remains active until the bounded follow-up
            # read and transport cleanup finish.
            "_raise_auxiliary_stop_acknowledgement_timeout": {"ser2"},
            "_read_auxiliary_inactive_stop_response": {"ser2"},
            "_run_auxiliary_stop_safe": {"ser2"},
            "_replace_auxiliary_serial": {"ser2"},
            "_startup_exchange_response": {"ser"},
            "_exchange_legacy_main_command": {"ser"},
            "_write_legacy_auxiliary_command": {"ser2"},
            # Admission helpers capture connection identity before dispatching
            # work through an operation-owning transport path.
            "_start_xbox_auxiliary_request": {"ser2"},
            "_try_dispatch_controller_correction": {"ser"},
            "send_xbox_auxiliary": {"ser2"},
            # Calibration commands run only inside the tracked public
            # calibration operation that owns the main transport.
            "_execute_calibration_command": {"ser"},
            "_start_calibration_sequence": {"ser"},
            "_invalidate_uncertain_controller_calibration": {"ser"},
            "_preflight_controller_calibration_transport": {"ser"},
            # The Tk callback serializes auxiliary replacement with the
            # transport lock and delegates all close/open work to owned helpers.
            "setCom2": {"ser2"},
            # Main connection admission acquires the activity lease before any
            # handle access and transfers ownership to asynchronous startup.
            "setCom": {"ser"},
            "_set_com_admitted": {"ser"},
        }
        missing = []

        def own_body_nodes(function):
            nodes = []

            class Visitor(ast.NodeVisitor):
                def visit_FunctionDef(self, node):
                    return None

                def visit_AsyncFunctionDef(self, node):
                    return None

                def generic_visit(self, node):
                    nodes.append(node)
                    super().generic_visit(node)

            visitor = Visitor()
            for statement in function.body:
                visitor.visit(statement)
            return nodes

        for function in (
            node
            for node in ast.walk(self.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        ):
            function_nodes = own_body_nodes(function)
            direct_names = {
                node.slice.value
                for node in function_nodes
                if isinstance(node, ast.Subscript)
                and isinstance(node.value, ast.Name)
                and node.value.id == "RUN"
                and isinstance(node.slice, ast.Constant)
                and node.slice.value in {"ser", "ser2"}
            }
            direct_names.update(
                node.args[0].value
                for node in function_nodes
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "RUN"
                and node.func.attr == "get"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value in {"ser", "ser2"}
            )
            if not direct_names:
                continue
            if direct_names.issubset(owner_managed.get(function.name, set())):
                continue

            tracked_names = set()
            for decorator in function.decorator_list:
                if (
                    isinstance(decorator, ast.Call)
                    and isinstance(decorator.func, ast.Name)
                    and decorator.func.id == "_tracked_serial_operation"
                ):
                    tracked_names.update(
                        argument.value
                        for argument in decorator.args
                        if isinstance(argument, ast.Constant)
                        and isinstance(argument.value, str)
                    )

            operation_names = set()
            for context in (
                item.context_expr
                for node in function_nodes
                if isinstance(node, ast.With)
                for item in node.items
            ):
                if (
                    isinstance(context, ast.Call)
                    and isinstance(context.func, ast.Name)
                    and context.func.id == "_tracked_auxiliary_operation"
                ):
                    operation_names.add("ser2")
                    continue
                if not (
                    isinstance(context, ast.Call)
                    and isinstance(context.func, ast.Attribute)
                    and isinstance(context.func.value, ast.Name)
                    and context.func.value.id == "serial_activity_registry"
                    and context.func.attr == "operations"
                ):
                    continue
                operation_names.update(
                    constant.value
                    for argument in context.args
                    for constant in ast.walk(argument)
                    if isinstance(constant, ast.Constant)
                    and isinstance(constant.value, str)
                )
                for keyword in context.keywords:
                    if keyword.arg == "control_injectable_names":
                        operation_names.update(
                            constant.value
                            for constant in ast.walk(keyword.value)
                            if isinstance(constant, ast.Constant)
                            and isinstance(constant.value, str)
                        )

            covered_names = tracked_names | operation_names
            if not direct_names.issubset(covered_names):
                missing.append((function.name, sorted(direct_names - covered_names)))

        self.assertEqual(missing, [])

    def test_async_main_transport_owners_hold_shutdown_activity_leases(self):
        def lease_calls(function_name):
            return [
                node
                for node in ast.walk(self.module_functions[function_name])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "serial_activity_registry"
                and node.func.attr == "lease"
                and len(node.args) == 1
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == "ser"
            ]

        self.assertEqual(len(lease_calls("_set_com_admitted")), 1)
        self.assertEqual(len(lease_calls("start_send_serial_thread")), 1)
        self.assertEqual(len(lease_calls("_start_calibration_sequence")), 1)

        calibration_start = self.module_functions["_start_calibration_sequence"]
        lock_lines = [
            node.lineno
            for node in ast.walk(calibration_start)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "serial_lock"
            and node.func.attr == "acquire"
        ]
        lease_lines = [
            node.lineno
            for node in ast.walk(calibration_start)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "serial_activity_registry"
            and node.func.attr == "lease"
            and any(
                isinstance(argument, ast.Constant) and argument.value == "ser"
                for argument in node.args
            )
        ]
        handle_lines = [
            node.lineno
            for node in ast.walk(calibration_start)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "RUN"
            and node.func.attr == "get"
            and any(
                isinstance(argument, ast.Constant) and argument.value == "ser"
                for argument in node.args
            )
        ]
        self.assertEqual(
            (len(lock_lines), len(lease_lines), len(handle_lines)),
            (1, 1, 1),
        )
        self.assertLess(lock_lines[0], lease_lines[0])
        self.assertLess(lease_lines[0], handle_lines[0])

        dispatcher_calls = [
            node.value
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "joint_motion_dispatcher"
                for target in node.targets
            )
            and isinstance(node.value, ast.Call)
        ]
        self.assertEqual(len(dispatcher_calls), 1)
        activity_factories = [
            keyword.value
            for keyword in dispatcher_calls[0].keywords
            if keyword.arg == "activity_factory"
        ]
        self.assertEqual(len(activity_factories), 1)
        self.assertIsInstance(activity_factories[0], ast.Lambda)
        factory_calls = [
            node
            for node in ast.walk(activity_factories[0])
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "lease"
        ]
        self.assertEqual(len(factory_calls), 1)

    def test_auxiliary_control_injection_is_limited_to_interruptible_wait(self):
        execute_row = self.module_functions["executeRow"]
        injectable_calls = [
            node
            for node in ast.walk(execute_row)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_execute_row_auxiliary_command"
            and any(
                keyword.arg == "control_injectable"
                and isinstance(keyword.value, ast.Constant)
                and keyword.value.value is True
                for keyword in node.keywords
            )
        ]
        wait_branches = [
            node
            for node in ast.walk(execute_row)
            if isinstance(node, ast.If)
            and any(
                isinstance(value, ast.Constant)
                and value.value == "Wait 5v Inp"
                for value in ast.walk(node.test)
            )
        ]

        self.assertEqual(len(injectable_calls), 1)
        self.assertEqual(len(wait_branches), 1)
        self.assertIn(injectable_calls[0], tuple(ast.walk(wait_branches[0])))
        keyword_values = {
            keyword.arg: keyword.value
            for keyword in injectable_calls[0].keywords
        }
        self.assertIn("response_timeout", keyword_values)
        self.assertIn("accepted_responses", keyword_values)
        self.assertIsInstance(keyword_values["response_timeout"], ast.BinOp)
        self.assertIsInstance(keyword_values["accepted_responses"], ast.Name)
        self.assertEqual(
            keyword_values["accepted_responses"].id,
            "AUXILIARY_WAIT_TERMINAL_RESPONSES",
        )

    def test_auxiliary_wait_timeout_matches_firmware_integer_range(self):
        validate_timeout = self.compile_function(
            "_auxiliary_wait_timeout_seconds",
            {
                "MotionInputError": MotionInputError,
                "AUXILIARY_FIRMWARE_SIGNED_INT_MAX": 32767,
                "re": re,
            },
        )

        self.assertEqual(validate_timeout("0"), 0)
        self.assertEqual(validate_timeout(32767), 32767)
        for value in (True, -1, 1.5, "32768", "1e2"):
            with self.subTest(value=value):
                with self.assertRaises(MotionInputError):
                    validate_timeout(value)

    def test_main_wait_contract_matches_teensy_numeric_and_response_types(self):
        namespace = {
            "MotionInputError": MotionInputError,
            "MAIN_FIRMWARE_WAIT_MAX_SECONDS": 2147483,
            "re": re,
            "finite_number": finite_number,
            "controller_protocol_decimal": controller_protocol_decimal,
        }
        validate_modbus = self.compile_function(
            "_main_modbus_wait_timeout_seconds",
            namespace,
        )
        validate_timed = self.compile_function(
            "_main_timed_wait_seconds",
            namespace,
        )

        self.assertEqual(validate_modbus("1"), 1)
        self.assertEqual(validate_modbus("2147483"), 2147483)
        for value in (True, 0, -1, 1.5, "2147484", "1e2"):
            with self.subTest(value=value):
                with self.assertRaises(MotionInputError):
                    validate_modbus(value)
        self.assertEqual(validate_timed("0.5"), (0.5, "0.5"))
        for value in (-1, "2147484", float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(MotionInputError):
                    validate_timed(value)

        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        for opcode, next_opcode in (("WJ", "WK"), ("WK", "SC")):
            branch_start = firmware.index(f'if (function == "{opcode}")')
            branch_end = firmware.index(
                f'if (function == "{next_opcode}")',
                branch_start,
            )
            branch = firmware[branch_start:branch_end]
            self.assertIn('Serial.println("Done")', branch)
            self.assertIn('Serial.println("Modbus Error")', branch)
            self.assertIn('Serial.println("ER")', branch)
        wait_start = firmware.index('if (function == "WT")')
        wait_end = firmware.index(
            'if (function == "ON" || function == "OF")',
            wait_start,
        )
        self.assertIn('Serial.println("WTdone")', firmware[wait_start:wait_end])

    def test_program_modbus_responses_fail_before_row_completion(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        response = {"value": "1"}
        finishes = []
        first_label = Label()
        second_label = Label()
        namespace = {
            "ROW_EXECUTION_COMPLETE": "complete",
            "ROW_EXECUTION_REJECTED": "rejected",
            "ProtocolResponseError": ProtocolResponseError,
            "_execute_row_main_command": (
                lambda command, **contract: ("complete", response["value"])
            ),
            "_finish_execute_row": lambda: finishes.append(True),
            "logger": SimpleNamespace(error=lambda *args: None),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
        }
        execute = self.compile_function(
            "_execute_row_main_response",
            namespace,
        )

        self.assertEqual(
            execute(
                "SCA1B0C1\n",
                response_parser=parse_controller_modbus_response,
            ),
            ("complete", "1"),
        )
        self.assertEqual(finishes, [])

        for rejected in ("ER", "Modbus Error", "-1", "unexpected"):
            with self.subTest(rejected=rejected):
                response["value"] = rejected
                self.assertEqual(
                    execute(
                        "SCA1B0C1\n",
                        response_parser=parse_controller_modbus_response,
                    ),
                    ("rejected", None),
                )
        self.assertEqual(len(finishes), 4)
        self.assertIn("response rejected", first_label.text)
        self.assertEqual(first_label.text, second_label.text)

        execute_row_source = ast.get_source_segment(
            AR4_SOURCE.read_text(encoding="utf-8"),
            self.module_functions["executeRow"],
        )
        self.assertNotIn('response == "Modbus Error"', execute_row_source)
        self.assertNotIn('response == "-1"', execute_row_source)
        self.assertGreaterEqual(
            execute_row_source.count(
                "response_parser=parse_controller_modbus_response"
            ),
            8,
        )

    def test_injectable_auxiliary_command_requires_validated_line_ownership(self):
        execute_auxiliary = self.compile_function(
            "_execute_row_auxiliary_command",
            {
                "finite_number": finite_number,
                "MotionInputError": MotionInputError,
                "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 5,
            },
        )

        with self.assertRaisesRegex(
            MotionInputError,
            "validated line responses",
        ):
            execute_auxiliary("WIA1B1C30\n", control_injectable=True)

    def test_servo_and_output_rows_require_exact_unframed_acknowledgements(self):
        exact_responses = []
        for node in ast.walk(self.module_functions["executeRow"]):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_execute_row_auxiliary_command"
            ):
                for keyword in node.keywords:
                    if (
                        keyword.arg == "expected_response"
                        and isinstance(keyword.value, ast.Constant)
                        and isinstance(keyword.value.value, bytes)
                    ):
                        exact_responses.append(keyword.value.value)

        self.assertCountEqual(exact_responses, (b"Servo Done", b"Done"))

    def test_program_stop_remains_pending_until_auxiliary_terminal_event(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        first_label = Label()
        second_label = Label()
        runtime = {
            "estopActive": False,
            "posOutreach": False,
            "programStopRequestId": None,
        }
        namespace = {
            "_request_auxiliary_stop": lambda: ("pending", 8),
            "tab1": SimpleNamespace(runTrue=1),
            "RUN": runtime,
            "logger": SimpleNamespace(exception=lambda *args: None),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "AUXILIARY_STOP_NOT_REQUIRED": "not-required",
        }
        namespace["_set_program_stop_status"] = self.compile_function(
            "_set_program_stop_status",
            namespace,
        )
        stop_program = self.compile_function("stopProg", namespace)

        stop_program()

        self.assertEqual(runtime["programStopRequestId"], 8)
        self.assertEqual(runtime["programStopState"], "pending")
        self.assertEqual(
            first_label.text,
            "PROGRAM HALT REQUESTED; AUXILIARY STOP PENDING; "
            "ACTIVE MAIN MOTION NOT PREEMPTED",
        )
        self.assertEqual(second_label.text, first_label.text)

    def test_program_launch_rejects_unacknowledged_stop_request(self):
        statuses = []
        namespace = {
            "RUN": {"programStopRequestId": 12},
            "_set_program_stop_status": statuses.append,
        }
        run_program = self.compile_function("runProg", namespace)

        self.assertFalse(run_program())
        self.assertEqual(statuses, ["pending"])

    def test_program_halt_status_never_claims_unconfirmed_motion_stop(self):
        stopped_claim_owners = []
        for function in self.module_functions.values():
            if any(
                isinstance(node, ast.Constant)
                and isinstance(node.value, str)
                and node.value.startswith("PROGRAM STOPPED")
                for node in ast.walk(function)
            ):
                stopped_claim_owners.append(function.name)

        self.assertEqual(stopped_claim_owners, [])

        status_function = self.module_functions["_set_program_stop_status"]
        status_text = " ".join(
            node.value
            for node in ast.walk(status_function)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        )
        self.assertIn("ACTIVE MAIN MOTION NOT PREEMPTED", status_text)

    def test_unconfigured_auxiliary_stop_is_not_required(self):
        namespace = {
            "application_closing": threading.Event(),
            "RUN": {"offlineMode": False, "ser2": None},
            "CAL": {
                "com2Port": "None",
                "auxiliaryBoard": AUXILIARY_BOARD_NONE,
            },
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_active_request_id": None,
            "auxiliary_stop_pending_request_id": None,
            "auxiliary_stop_next_request_id": 0,
            "auxiliary_stop_requested": threading.Event(),
            "AUXILIARY_STOP_NOT_REQUIRED": "not-required",
            "AUXILIARY_STOP_PENDING": "pending",
            "AUXILIARY_STOP_DISPATCHED": "dispatched",
            "_try_dispatch_auxiliary_stop": lambda: self.fail(
                "unconfigured auxiliary stop must not dispatch"
            ),
        }
        namespace["_auxiliary_stop_not_required"] = self.compile_function(
            "_auxiliary_stop_not_required",
            namespace,
        )
        request_stop = self.compile_function(
            "_request_auxiliary_stop",
            namespace,
        )

        self.assertEqual(request_stop(), ("not-required", None))
        self.assertFalse(namespace["auxiliary_stop_requested"].is_set())
        self.assertIsNone(namespace["auxiliary_stop_pending_request_id"])

        namespace["CAL"].update(
            com2Port="COM2",
            auxiliaryBoard=AUXILIARY_BOARD_NANO,
        )
        namespace["_try_dispatch_auxiliary_stop"] = lambda: False
        self.assertEqual(request_stop(), ("pending", 1))
        self.assertTrue(namespace["auxiliary_stop_requested"].is_set())

    def test_auxiliary_stop_request_remains_pending_behind_owned_operation(self):
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2")
        requested = threading.Event()
        namespace = {
            "auxiliary_stop_next_request_id": 0,
            "auxiliary_stop_pending_request_id": None,
            "auxiliary_stop_active_request_id": None,
            "auxiliary_stop_owner_waiting": False,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": threading.Event(),
            "auxiliary_stop_injected_event": threading.Event(),
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_requested": requested,
            "auxiliary_serial_event_queue": Queue(),
            "application_closing": threading.Event(),
            "RUN": {"offlineMode": False},
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "threading": threading,
            "_run_auxiliary_stop_safe": lambda *args: None,
            "AUXILIARY_STOP_NOT_REQUIRED": "not-required",
            "AUXILIARY_STOP_PENDING": "pending",
            "AUXILIARY_STOP_DISPATCHED": "dispatched",
        }
        namespace["_try_dispatch_auxiliary_stop"] = self.compile_function(
            "_try_dispatch_auxiliary_stop",
            namespace,
        )
        request_stop = self.compile_function(
            "_request_auxiliary_stop",
            namespace,
        )

        self.assertEqual(request_stop(), ("pending", 1))
        self.assertTrue(requested.is_set())
        self.assertEqual(namespace["auxiliary_stop_pending_request_id"], 1)
        self.assertIsNone(namespace["auxiliary_stop_active_request_id"])
        self.assertTrue(activity.active("ser2"))
        activity.end("ser2")

    def test_auxiliary_stop_does_not_inject_after_response_ownership_ends(self):
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2", control_injectable=True)
        requested = threading.Event()
        requested.set()
        namespace = {
            "auxiliary_stop_pending_request_id": 5,
            "auxiliary_stop_active_request_id": None,
            "auxiliary_stop_owner_waiting": False,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": threading.Event(),
            "auxiliary_stop_injected_event": threading.Event(),
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_requested": requested,
            "auxiliary_serial_event_queue": Queue(),
            "application_closing": threading.Event(),
            "RUN": {"offlineMode": False},
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "threading": threading,
            "_run_auxiliary_stop_safe": lambda *args: None,
        }
        dispatch_stop = self.compile_function(
            "_try_dispatch_auxiliary_stop",
            namespace,
        )

        self.assertFalse(dispatch_stop())
        self.assertTrue(requested.is_set())
        self.assertEqual(namespace["auxiliary_stop_pending_request_id"], 5)
        self.assertIsNone(namespace["auxiliary_stop_active_request_id"])
        activity.end("ser2", control_injectable=True)
        self.assertTrue(activity.idle())

    def test_program_stop_clears_local_state_when_auxiliary_dispatch_fails(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        first_label = Label()
        second_label = Label()
        tab = SimpleNamespace(runTrue=1)
        namespace = {
            "_request_auxiliary_stop": lambda: (_ for _ in ()).throw(
                RuntimeError("thread construction failed")
            ),
            "tab1": tab,
            "RUN": {"estopActive": False, "posOutreach": False},
            "logger": SimpleNamespace(exception=lambda *args: None),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "AUXILIARY_STOP_NOT_REQUIRED": "not-required",
        }
        namespace["_set_program_stop_status"] = self.compile_function(
            "_set_program_stop_status",
            namespace,
        )
        stop_program = self.compile_function("stopProg", namespace)

        stop_program()

        self.assertEqual(tab.runTrue, 0)
        self.assertEqual(
            first_label.text,
            "PROGRAM SCHEDULING HALTED; AUXILIARY STOP FAILED; "
            "ACTIVE MAIN MOTION NOT PREEMPTED",
        )
        self.assertEqual(second_label.text, first_label.text)

    def test_live_exchange_uses_the_derived_motion_response_timeout(self):
        calls = []

        def exchange(*args, **kwargs):
            calls.append((args, kwargs))
            return "response"

        control_event = object()
        namespace = {
            "exchange_serial_line": exchange,
            "RUN": {"ser": object()},
            "_controller_response_timeout": lambda command: 250000,
            "serial_write_lock": object(),
            "SERIAL_LIVE_ACK_TIMEOUT_SECONDS": 5,
            "serial_transport_quarantined": lambda serial_port: False,
            "SerialTransportQuarantinedError": ConnectionError,
            "SerialTransportTimeout": TimeoutError,
        }
        exchange_line = self.compile_function("_exchange_serial_line", namespace)

        command = "LJV10Sp50Ac10Dc20Rm25WALm000000\n"
        self.assertEqual(exchange_line(command, control_event), "response")
        self.assertEqual(calls[0][0][1], canonicalize_serial_command(command))
        self.assertEqual(
            calls[0][1]["control_ack_timeout_seconds"],
            5,
        )
        self.assertEqual(
            calls[0][1]["control_response_timeout_seconds"],
            250000,
        )
        self.assertEqual(calls[0][0][2], 250000)
        self.assertEqual(calls[0][1]["control_event"], control_event)

    def test_exchange_retains_open_quarantined_main_transport_for_cleanup(self):
        serial_port = SimpleNamespace(is_open=True)
        runtime = {"ser": serial_port}

        def exchange(*args, **kwargs):
            raise OSError("framing uncertain")

        namespace = {
            "exchange_serial_line": exchange,
            "RUN": runtime,
            "_controller_response_timeout": lambda command: 120,
            "serial_write_lock": object(),
            "SERIAL_LIVE_ACK_TIMEOUT_SECONDS": 5,
            "serial_transport_quarantined": lambda value: value is serial_port,
            "SerialTransportQuarantinedError": ConnectionError,
            "SerialTransportTimeout": TimeoutError,
        }
        exchange_line = self.compile_function("_exchange_serial_line", namespace)

        with self.assertRaisesRegex(OSError, "framing uncertain"):
            exchange_line("RP\n")
        self.assertIs(runtime["ser"], serial_port)

    def test_exchange_retains_open_quarantine_exception_for_cleanup(self):
        class QuarantinedError(ConnectionError):
            pass

        class QuarantinedTimeout(TimeoutError):
            pass

        serial_port = SimpleNamespace(is_open=True)
        runtime = {"ser": serial_port}

        def exchange(*args, **kwargs):
            raise QuarantinedError("framing uncertain")

        namespace = {
            "exchange_serial_line": exchange,
            "RUN": runtime,
            "_controller_response_timeout": lambda command: 120,
            "serial_write_lock": object(),
            "SERIAL_LIVE_ACK_TIMEOUT_SECONDS": 5,
            "serial_transport_quarantined": lambda value: False,
            "SerialTransportQuarantinedError": QuarantinedError,
            "SerialTransportTimeout": QuarantinedTimeout,
        }
        exchange_line = self.compile_function("_exchange_serial_line", namespace)

        with self.assertRaisesRegex(QuarantinedError, "framing uncertain"):
            exchange_line("RP\n")
        self.assertIs(runtime["ser"], serial_port)

    def test_display_position_applies_valid_state_and_isolates_faults(self):
        class Widget:
            def __init__(self):
                self.value = None

            def delete(self, *args):
                self.value = None

            def insert(self, index, value):
                self.value = value

            def set(self, value):
                self.value = value

            def config(self, **kwargs):
                self.value = kwargs

        class Dispatcher:
            def __init__(self):
                self.positions = []
                self.accept = True
                self.invalidations = []

            def synchronize(self, position):
                self.positions.append(position)
                return self.accept

            def invalidate(self, reason):
                self.invalidations.append(reason)
                return False

        class Logger:
            def __init__(self):
                self.errors = []
                self.warnings = []

            def error(self, message):
                self.errors.append(message)

            def warning(self, message):
                self.warnings.append(message)

        entry_names = (
            "J1curAngEntryField", "J2curAngEntryField", "J3curAngEntryField",
            "J4curAngEntryField", "J5curAngEntryField", "J6curAngEntryField",
            "XcurEntryField", "YcurEntryField", "ZcurEntryField",
            "RzcurEntryField", "RycurEntryField", "RxcurEntryField",
            "J7curAngEntryField", "J8curAngEntryField", "J9curAngEntryField",
        )
        slider_names = tuple(f"J{axis}jogslide" for axis in range(1, 10))
        widgets = {
            name: Widget()
            for name in entry_names + slider_names
        }
        scheduled = []
        faults = []
        virtual_updates = []
        resynchronization_required = threading.Event()
        dispatcher = Dispatcher()
        namespace = {
            **widgets,
            "CAL": {},
            "RUN": {},
            "confirmed_position_generation": 4,
            "parse_position_response": parse_position_response,
            "PositionResponse": PositionResponse,
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "_current_controller_joint_calibration": lambda: (
                ControllerJointCalibration(
                    negative_limits=(100,) * 9,
                    positive_limits=(100,) * 9,
                    steps_per_unit=(100,) * 9,
                )
            ),
            "cmdRecEntryField": Widget(),
            "manEntryField": Widget(),
            "_schedule_calibration_save": lambda: scheduled.append(True),
            "joint_motion_dispatcher": dispatcher,
            "deferred_joint_adjustments": SimpleNamespace(pending=False),
            "_clear_deferred_joint_adjustments": lambda: None,
            "controller_position_resynchronization_required": (
                resynchronization_required
            ),
            "_try_set_virtual_joint_target": (
                lambda joints: virtual_updates.append(tuple(joints)) or True
            ),
            "ErrorHandler": faults.append,
            "logger": Logger(),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
        }
        namespace["_invalidate_joint_motion_state"] = self.compile_function(
            "_invalidate_joint_motion_state",
            namespace,
        )
        display_position = self.compile_function("displayPosition", namespace)
        response = (
            "A1B2C3D4E5F6G7H8I9J10K11L12"
            "M0N42.5OP13Q14R15"
        )

        self.assertTrue(display_position(response))
        self.assertEqual(
            tuple(namespace["CAL"][key] for key in (
                "J1AngCur", "J2AngCur", "J3AngCur", "J4AngCur", "J5AngCur",
                "J6AngCur", "XcurPos", "YcurPos", "ZcurPos", "RzcurPos",
                "RycurPos", "RxcurPos", "J7PosCur", "J8PosCur", "J9PosCur",
            )),
            tuple(str(value) for value in range(1, 16)),
        )
        self.assertEqual(namespace["RUN"]["WC"], "F")
        self.assertEqual(namespace["confirmed_position_generation"], 5)
        self.assertIsNone(namespace["acknowledged_forced_position_target"])
        self.assertEqual(
            dispatcher.positions,
            [(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 13.0, 14.0, 15.0)],
        )
        self.assertEqual(scheduled, [True])
        self.assertEqual(namespace["manEntryField"].value, "42.5")
        self.assertEqual(
            tuple(widgets[name].value for name in entry_names),
            tuple(str(value) for value in range(1, 16)),
        )
        self.assertEqual(
            tuple(widgets[name].value for name in slider_names),
            ("1", "2", "3", "4", "5", "6", "13", "14", "15"),
        )

        for malformed_response in (
            response.replace("N42.5O", "NdebugO"),
            response.replace("OP13", "OfaultP13"),
            response.replace("OP13", "OEBP0P13"),
        ):
            with self.subTest(malformed_response=malformed_response):
                calibration_before_malformed = dict(namespace["CAL"])
                entries_before_malformed = {
                    name: widgets[name].value
                    for name in entry_names + slider_names
                }
                saves_before_malformed = list(scheduled)
                generation_before_malformed = namespace[
                    "confirmed_position_generation"
                ]

                self.assertFalse(display_position(malformed_response))

                self.assertEqual(namespace["CAL"], calibration_before_malformed)
                self.assertEqual(
                    {
                        name: widgets[name].value
                        for name in entry_names + slider_names
                    },
                    entries_before_malformed,
                )
                self.assertEqual(scheduled, saves_before_malformed)
                self.assertEqual(
                    namespace["confirmed_position_generation"],
                    generation_before_malformed,
                )

        dispatcher.accept = False
        calibration_before_rejection = dict(namespace["CAL"])
        rejected_response = response.replace("A1B", "A99B", 1)
        self.assertFalse(display_position(rejected_response))
        self.assertEqual(namespace["CAL"], calibration_before_rejection)
        self.assertEqual(namespace["confirmed_position_generation"], 5)
        self.assertEqual(scheduled, [True])
        self.assertIn(
            "rejected while joint motion is active",
            namespace["logger"].errors[-1],
        )
        self.assertTrue(resynchronization_required.is_set())
        dispatcher.accept = True
        self.assertTrue(display_position(response))
        self.assertFalse(resynchronization_required.is_set())
        self.assertEqual(
            virtual_updates[-1],
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        )

        calibration_before_limit_failure = dict(namespace["CAL"])
        dispatcher_positions_before_limit_failure = list(dispatcher.positions)
        scheduled_before_limit_failure = list(scheduled)
        out_of_range_response = response.replace("A1B", "A101B", 1)
        self.assertFalse(display_position(out_of_range_response))
        self.assertEqual(namespace["CAL"], calibration_before_limit_failure)
        self.assertEqual(
            dispatcher.positions,
            dispatcher_positions_before_limit_failure,
        )
        self.assertEqual(scheduled, scheduled_before_limit_failure)
        self.assertIn("outside the calibrated limits", namespace["logger"].errors[-1])
        out_of_range_fault = out_of_range_response.replace("O", "OEB", 1)
        self.assertFalse(display_position(out_of_range_fault))
        self.assertEqual(namespace["CAL"], calibration_before_limit_failure)
        self.assertEqual(faults, [])
        self.assertTrue(resynchronization_required.is_set())
        self.assertTrue(display_position(response))
        self.assertFalse(resynchronization_required.is_set())

        fault_response = response.replace("O", "OEC000000", 1)
        generation_before_fault = namespace["confirmed_position_generation"]
        self.assertTrue(display_position(fault_response))
        self.assertEqual(
            namespace["confirmed_position_generation"],
            generation_before_fault,
        )
        self.assertTrue(resynchronization_required.is_set())
        self.assertEqual(
            dispatcher.invalidations[-1],
            "controller reported motion fault: EC000000",
        )
        self.assertEqual(faults, ["EC000000"])
        self.assertTrue(display_position(response))
        self.assertFalse(resynchronization_required.is_set())

        calibration_after_fault = dict(namespace["CAL"])
        self.assertFalse(display_position("malformed response"))
        self.assertEqual(namespace["CAL"], calibration_after_fault)
        self.assertTrue(resynchronization_required.is_set())
        self.assertTrue(display_position(response))
        self.assertFalse(resynchronization_required.is_set())
        self.assertEqual(len(virtual_updates), 4)
        self.assertEqual(namespace["confirmed_position_generation"], 9)
        self.assertEqual(scheduled, [True] * 6)

    def test_execute_row_rejects_unapplied_controller_positions(self):
        class Entry:
            def __init__(self):
                self.value = ""

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

        class ProgramView:
            command = (
                b"Move J X 1 Y 2 Z 3 Rz 4 Ry 5 Rx 6 "
                b"J7 7 J8 8 J9 9 Sp 25 Ac 10 Dc 10 Rm 100 $ N"
            )

            @staticmethod
            def curselection():
                return (0,)

            @staticmethod
            def see(row):
                pass

            def get(self, row):
                return self.command

        class Port:
            def __init__(self, response):
                self.response = response
                self.writes = []

            def write(self, value):
                self.writes.append(value)

            @staticmethod
            def flushInput():
                pass

            def readline(self):
                return self.response

        for response, applies, expected, error_count in (
            (b"EFAIL", True, "rejected", 1),
            (b"malformed", False, "rejected", 0),
            (b"valid", True, "complete", 0),
        ):
            with self.subTest(response=response):
                errors = []
                finishes = []
                invalidations = []
                namespace = {
                    "RUN": {
                        "progRunning": False,
                        "cmdType": None,
                        "cmdTypeLong": None,
                        "offlineMode": False,
                        "moveInProc": 0,
                        "vtk_running": False,
                        "ser": Port(response),
                        "WC": "N",
                    },
                    "CAL": {
                        f"J{axis}OpenLoopVal": SimpleNamespace(get=lambda: 0)
                        for axis in range(1, 7)
                    },
                    "tab1": SimpleNamespace(progView=ProgramView()),
                    "cmdSentEntryField": Entry(),
                    "time": SimpleNamespace(sleep=lambda seconds: None),
                    "_decode_legacy_serial_line": (
                        lambda response_bytes, context: response_bytes.decode("ascii")
                    ),
                    "ErrorHandler": errors.append,
                    "_apply_valid_position_response": (
                        lambda value: VALID_CONTROLLER_POSITION if applies else None
                    ),
                    "_invalidate_joint_motion_state": invalidations.append,
                    "_finish_execute_row": lambda: finishes.append(True),
                    "mj_command": lambda command: completed_virtual_operation(),
                    "ROW_EXECUTION_REJECTED": "rejected",
                    "ROW_EXECUTION_PENDING": "pending",
                    "ROW_EXECUTION_COMPLETE": "complete",
                }
                namespace["_apply_controller_position_response"] = (
                    self.compile_function(
                        "_apply_controller_position_response",
                        namespace,
                    )
                )

                def dispatch_program_command(*args):
                    applied = namespace["_apply_controller_position_response"](
                        response.decode("ascii")
                    )
                    return "complete" if applied else "rejected"

                namespace["_dispatch_program_command"] = dispatch_program_command
                execute = self.compile_function("executeRow", namespace)

                self.assertEqual(execute(), expected)
                self.assertEqual(finishes, [True])
                self.assertEqual(len(errors), error_count)

    def test_execute_row_routes_every_simple_motion_through_one_dispatcher(self):
        class Entry:
            def __init__(self, value="0"):
                self.value = value

            def get(self):
                return self.value

        class ProgramView:
            def __init__(self, command):
                self.command = command.encode("ascii")

            @staticmethod
            def curselection():
                return (0,)

            @staticmethod
            def see(row):
                pass

            def get(self, row):
                return self.command

        cartesian = (
            "X 1 Y 2 Z 3 Rz 4 Ry 5 Rx 6 J7 7 J8 8 J9 9 "
            "Sp 25 Ac 10 Dc 10 Rm 100 $ N"
        )
        cases = (
            (f"Move J [*] {cartesian}", "MJ", "mj"),
            (f"OFF J [ PR: 1 ] [*] {cartesian}", "MJ", "mj"),
            (f"Move V [ PR: 1 ] [*] {cartesian}", "MV", "mv"),
            (
                "Move P [ PR: 1 ] [*] J7 7 J8 8 J9 9 "
                "Sp 25 Ac 10 Dc 10 Rm 100 $ N",
                "MJ",
                "mj",
            ),
            (
                "OFF PR [ PR: 1 ] offs [ *PR: 2 ]  [*] "
                "J7 7 J8 8 J9 9 Sp 25 Ac 10 Dc 10 Rm 100 $ N",
                "MJ",
                "mj",
            ),
            (
                "Move L [*] X 1 Y 2 Z 3 Rz 4 Ry 5 Rx 6 "
                "J7 7 J8 8 J9 9 Sp 25 Ac 10 Dc 10 Rm 100 "
                "Rnd 0 $ N",
                "ML",
                "mj",
            ),
            (
                "Move R [*] J1 1 J2 2 J3 3 J4 4 J5 5 J6 6 "
                "J7 7 J8 8 J9 9 Sp 25 Ac 10 Dc 10 Rm 100 $ N",
                "RJ",
                "rj",
            ),
        )

        for command, expected_opcode, expected_virtual in cases:
            with self.subTest(command=command[:6]):
                dispatches = []
                finishes = []
                mj_dispatch = lambda value: value
                mv_dispatch = lambda value: value
                rj_dispatch = lambda value: value
                namespace = {
                    "RUN": {
                        "progRunning": False,
                        "cmdType": None,
                        "cmdTypeLong": None,
                        "offlineMode": False,
                        "moveInProc": 0,
                        "vtk_running": False,
                        "WC": "N",
                    },
                    "CAL": {
                        **{
                            f"J{axis}OpenLoopVal": SimpleNamespace(get=lambda: 0)
                            for axis in range(1, 7)
                        },
                        "DisableWristRotVal": SimpleNamespace(get=lambda: 0),
                    },
                    "tab1": SimpleNamespace(progView=ProgramView(command)),
                    "VisRetXrobEntryField": Entry("0"),
                    "VisRetYrobEntryField": Entry("0"),
                    "VisRetAngleEntryField": Entry("0"),
                    "mj_command": mj_dispatch,
                    "mv_command": mv_dispatch,
                    "rj_command": rj_dispatch,
                    "_dispatch_program_command": (
                        lambda *args: dispatches.append(args) or "complete"
                    ),
                    "_finish_execute_row": lambda: finishes.append(True),
                    "ROW_EXECUTION_REJECTED": "rejected",
                    "ROW_EXECUTION_PENDING": "pending",
                    "ROW_EXECUTION_COMPLETE": "complete",
                }
                for register in (1, 2):
                    for element in range(1, 7):
                        namespace[
                            f"SP_{register}_E{element}_EntryField"
                        ] = Entry(str(element))

                execute = self.compile_function("executeRow", namespace)
                self.assertEqual(execute(), "complete")
                self.assertEqual(len(dispatches), 1)
                self.assertTrue(dispatches[0][0].startswith(expected_opcode))
                self.assertIs(
                    dispatches[0][1],
                    {
                        "mj": mj_dispatch,
                        "mv": mv_dispatch,
                        "rj": rj_dispatch,
                    }[expected_virtual],
                )
                self.assertTrue(dispatches[0][2].startswith(expected_virtual.upper()))
                self.assertEqual(finishes, [True])

    def test_default_program_motion_holds_owner_and_offline_default_rejects(self):
        class Entry:
            def __init__(self):
                self.value = None

            def delete(self, *args):
                self.value = None

            def insert(self, index, value):
                self.value = value

        def build_dispatch(offline):
            controller_callbacks = []
            completion_results = []
            starts = []
            namespace = {
                "RUN": {"offlineMode": offline, "vtk_running": False},
                "threading": threading,
                "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
                "_controller_response_timeout": lambda command: 1.0,
                "_start_legacy_motion": (
                    lambda command, name, **kwargs: starts.append(command)
                    or controller_callbacks.append(kwargs["completion_callback"])
                    or True
                ),
                "cmdSentEntryField": Entry(),
                "logger": SimpleNamespace(
                    error=lambda *args: None,
                    warning=lambda *args: None,
                    exception=lambda *args: None,
                ),
                "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
                "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
                "ROW_EXECUTION_REJECTED": "rejected",
                "ROW_EXECUTION_PENDING": "pending",
                "ROW_EXECUTION_COMPLETE": "complete",
            }

            def finish_settled(callback, lease, succeeded, settlement_callback):
                succeeded = settlement_callback(succeeded)
                lease.close()
                callback(succeeded)
                return True

            namespace["_finish_settled_motion_request"] = finish_settled
            namespace["_dispatch_program_controller_sequence"] = self.compile_function(
                "_dispatch_program_controller_sequence",
                namespace,
            )
            namespace["_dispatch_program_motion"] = self.compile_function(
                "_dispatch_program_motion",
                namespace,
            )
            namespace["_dispatch_program_command"] = self.compile_function(
                "_dispatch_program_command",
                namespace,
            )
            return namespace, controller_callbacks, completion_results, starts

        online, callbacks, completions, starts = build_dispatch(False)
        result = online["_dispatch_program_command"](
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            lambda command: self.fail("default mode must not start virtual playback"),
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            completions.append,
        )
        self.assertEqual(result, "pending")
        self.assertEqual(starts, [CONTROLLER_CARTESIAN_TEST_COMMAND])
        self.assertTrue(online["motion_request_registry"].active)
        callbacks[0](VALID_CONTROLLER_POSITION)
        self.assertEqual(completions, [True])
        self.assertFalse(online["motion_request_registry"].active)

        offline, callbacks, completions, starts = build_dispatch(True)
        result = offline["_dispatch_program_command"](
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            lambda command: completed_virtual_operation(),
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            completions.append,
        )
        self.assertEqual(result, "rejected")
        self.assertEqual(starts, [])
        self.assertEqual(callbacks, [])
        self.assertEqual(completions, [])
        self.assertFalse(offline["motion_request_registry"].active)

    def test_controller_sequence_drops_stale_pose_after_later_write_failure(self):
        callbacks = []
        write_events = []
        completions = []
        reconciliations = []
        namespace = {
            "RUN": {"offlineMode": False},
            "threading": threading,
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "_controller_response_timeout": lambda command: 1.0,
            "_capture_program_motion_pose": lambda: "saved pose",
            "_start_legacy_motion": (
                lambda command, name, **kwargs: callbacks.append(
                    kwargs["completion_callback"]
                )
                or write_events.append(kwargs["write_started_event"])
                or True
            ),
            "_reconcile_program_motion_pose": (
                lambda snapshot, position, write_started, succeeded: (
                    reconciliations.append(
                        (snapshot, position, write_started.is_set(), succeeded)
                    )
                    or False
                )
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }

        def finish_settled(callback, lease, succeeded, settlement_callback):
            succeeded = settlement_callback(succeeded)
            lease.close()
            callback(succeeded)
            return True

        namespace["_finish_settled_motion_request"] = finish_settled
        dispatch = self.compile_function(
            "_dispatch_program_controller_sequence",
            namespace,
        )

        result = dispatch(
            (
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                CONTROLLER_CARTESIAN_TEST_COMMAND,
            ),
            completions.append,
        )
        self.assertEqual(result, "pending")
        self.assertEqual(len(callbacks), 1)

        write_events[0].set()
        callbacks[0](VALID_CONTROLLER_POSITION)
        self.assertEqual(len(callbacks), 2)
        self.assertIs(write_events[0], write_events[1])
        self.assertFalse(write_events[1].is_set())

        write_events[1].set()
        callbacks[1](None)

        self.assertEqual(
            reconciliations,
            [("saved pose", None, True, False)],
        )
        self.assertEqual(completions, [False])
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_execute_row_rejects_raw_joint_limit_before_worker_start(self):
        class Entry:
            def delete(self, *args):
                pass

            def insert(self, *args):
                pass

        class ProgramView:
            command = (
                b"Move R [*] J1 101 J2 2 J3 3 J4 4 J5 5 J6 6 "
                b"J7 7 J8 8 J9 9 Sp 25 Ac 10 Dc 10 Rm 100 $ N"
            )

            @staticmethod
            def curselection():
                return (0,)

            @staticmethod
            def see(row):
                pass

            def get(self, row):
                return self.command

        starts = []
        finishes = []
        namespace = {
            "RUN": {
                "progRunning": False,
                "cmdType": None,
                "cmdTypeLong": None,
                "offlineMode": False,
                "moveInProc": 0,
                "vtk_running": False,
                "WC": "N",
            },
            "CAL": {
                f"J{axis}OpenLoopVal": SimpleNamespace(get=lambda: 0)
                for axis in range(1, 7)
            },
            "tab1": SimpleNamespace(progView=ProgramView()),
            "cmdSentEntryField": Entry(),
            "rj_command": lambda command: completed_virtual_operation(),
            "threading": threading,
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "_controller_response_timeout": lambda command: 1.0,
            "_start_legacy_motion": lambda *args, **kwargs: starts.append(args) or True,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "_finish_execute_row": lambda: finishes.append(True),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        namespace["_dispatch_program_controller_sequence"] = self.compile_function(
            "_dispatch_program_controller_sequence",
            namespace,
        )
        namespace["_dispatch_program_motion"] = self.compile_function(
            "_dispatch_program_motion",
            namespace,
        )
        namespace["_dispatch_program_command"] = self.compile_function(
            "_dispatch_program_command",
            namespace,
        )
        execute = self.compile_function("executeRow", namespace)

        self.assertEqual(execute(), "rejected")
        self.assertEqual(starts, [])
        self.assertEqual(finishes, [True])
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_main_serial_canonicalizer_uses_active_calibration_for_targets(self):
        calibration = ControllerJointCalibration(
            negative_limits=(10,) * 9,
            positive_limits=(10,) * 9,
            steps_per_unit=(100,) * 9,
        )
        calibration_reads = []
        namespace = {
            "_current_controller_joint_calibration": (
                lambda: calibration_reads.append(True) or calibration
            ),
            "canonicalize_serial_command": canonicalize_serial_command,
        }
        canonicalize_main = self.compile_function(
            "_canonicalize_main_serial_command",
            namespace,
        )

        self.assertEqual(canonicalize_main("RP\n"), "RP\n")
        self.assertEqual(calibration_reads, [])
        with self.assertRaisesRegex(MotionInputError, "J1 position"):
            canonicalize_main(
                "RJA11B2C3D4E5F6J77J88J99"
                "Sp25Ac10Dc10Rm100WNLm000000\n"
            )
        self.assertEqual(calibration_reads, [True])

    def test_compound_and_spline_rejection_preserves_program_selection(self):
        class ProgramView:
            def __init__(self, command):
                self.command = command.encode("ascii")
                self.selection = 4
                self.selection_changes = []

            def curselection(self):
                return (self.selection,)

            @staticmethod
            def see(row):
                pass

            def get(self, row):
                return self.command

            def selection_clear(self, *args):
                self.selection_changes.append(("clear", args))

            def select_set(self, row):
                self.selection = row
                self.selection_changes.append(("select", row))

        commands = (
            "Move A Mid [*] X 1 Y 2 Z 3",
            "Move C Center [*] X 1 Y 2 Z 3",
            "Start Spline",
            "End Spline",
        )
        for command in commands:
            with self.subTest(command=command):
                program_view = ProgramView(command)
                finishes = []
                namespace = {
                    "RUN": {
                        "progRunning": False,
                        "cmdType": None,
                        "cmdTypeLong": None,
                        "offlineMode": False,
                        "moveInProc": 0,
                    },
                    "tab1": SimpleNamespace(progView=program_view),
                    "logger": SimpleNamespace(error=lambda *args: None),
                    "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
                    "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
                    "_finish_execute_row": lambda: finishes.append(True),
                    "ROW_EXECUTION_REJECTED": "rejected",
                    "ROW_EXECUTION_PENDING": "pending",
                    "ROW_EXECUTION_COMPLETE": "complete",
                }
                execute = self.compile_function("executeRow", namespace)

                self.assertEqual(execute(), "rejected")
                self.assertEqual(program_view.selection, 4)
                self.assertEqual(program_view.selection_changes, [])
                self.assertEqual(finishes, [True])

    def test_execute_row_rejects_failed_calibration_result(self):
        class ProgramView:
            @staticmethod
            def curselection():
                return (0,)

            @staticmethod
            def see(row):
                pass

            @staticmethod
            def get(row):
                return b"Cal_J1"

        calibration_calls = []
        finishes = []

        def fail_calibration():
            calibration_calls.append("J1")
            return False

        namespace = {
            "RUN": {
                "progRunning": False,
                "cmdType": None,
                "cmdTypeLong": None,
                "offlineMode": False,
                "moveInProc": 0,
            },
            "tab1": SimpleNamespace(progView=ProgramView()),
            "_run_program_calibration_all": lambda: True,
            "_run_program_calibration_j1": fail_calibration,
            "_run_program_calibration_j2": lambda: True,
            "_run_program_calibration_j3": lambda: True,
            "_run_program_calibration_j4": lambda: True,
            "_run_program_calibration_j5": lambda: True,
            "_run_program_calibration_j6": lambda: True,
            "_run_program_calibration_j7": lambda: True,
            "_run_program_calibration_j8": lambda: True,
            "_run_program_calibration_j9": lambda: True,
            "_finish_execute_row": lambda: finishes.append(True),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        execute = self.compile_function("executeRow", namespace)

        self.assertEqual(execute(), "rejected")
        self.assertEqual(calibration_calls, ["J1"])
        self.assertEqual(finishes, [True])

    def test_auto_calibration_stops_on_unapplied_stage_response(self):
        class Entry:
            def __init__(self, value="0"):
                self.value = value

            def get(self, *args):
                return self.value

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

        class Port:
            def __init__(self, responses):
                self.responses = list(responses)
                self.writes = []
                self.is_open = True
                self.timeout = None

            def write(self, value):
                self.writes.append(value)
                return len(value)

            @staticmethod
            def reset_input_buffer():
                pass

            @staticmethod
            def flush():
                pass

            def readline(self):
                return self.responses.pop(0)

            @staticmethod
            def read(size=1):
                return b""

            def close(self):
                self.is_open = False

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        calibration = {}
        for axis in range(1, 10):
            calibration[f"J{axis}CalStatVal"] = Entry("1")
            calibration[f"J{axis}CalStatVal2"] = Entry("1")
            calibration[f"J{axis}calOff"] = "0"
        for axis in range(1, 7):
            calibration[f"J{axis}AngCur"] = str(axis)

        first_label = Label()
        second_label = Label()
        monitor_updates = []
        applied_results = [None]
        response = (
            b"A1B2C3D4E5F6G7H8I9J10K11L12"
            b"M0N42.5OP13Q14R15\n"
        )
        namespace = {
            "RUN": {
                "offlineMode": False,
                "VR_angles": [0.0] * 6,
                "ser": Port((response,)),
            },
            "CAL": calibration,
            "cmdSentEntryField": Entry(),
            "cmdRecEntryField": Entry(),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "tab8": SimpleNamespace(ElogView=Entry()),
            "END": "end",
            "pickle": SimpleNamespace(dump=lambda *args: None),
            "open": lambda *args: object(),
            "logger": SimpleNamespace(
                info=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "ErrorHandler": lambda response: None,
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "finite_number": finite_number,
            "parse_position_response": parse_position_response,
            "threading": threading,
            "exchange_serial_line_until_cancelled": (
                exchange_serial_line_until_cancelled
            ),
            "application_closing": threading.Event(),
            "serial_write_lock": threading.Lock(),
            "setStepMonitorsVR": lambda: monitor_updates.append(True),
            "_current_controller_joint_calibration": (
                lambda: ControllerJointCalibration(
                    negative_limits=(180.0,) * 9,
                    positive_limits=(180.0,) * 9,
                    steps_per_unit=(1.0,) * 9,
                )
            ),
            "_apply_valid_position_response": (
                lambda response: applied_results.pop(0)
            ),
        }
        self.add_startup_command_dependencies(namespace)
        namespace["_binary_controller_flag"] = self.compile_function(
            "_binary_controller_flag",
            namespace,
        )
        namespace["_prepare_calibration_command"] = self.compile_function(
            "_prepare_calibration_command",
            namespace,
        )
        namespace["_calibration_available"] = self.compile_function(
            "_calibration_available",
            namespace,
        )
        namespace["_record_calibration_response"] = self.compile_function(
            "_record_calibration_response",
            namespace,
        )
        namespace["_execute_calibration_command"] = self.compile_function(
            "_execute_calibration_command",
            namespace,
        )
        calibrate = self.compile_function("_run_program_calibration_all", namespace)

        self.assertFalse(calibrate())
        self.assertEqual(len(namespace["RUN"]["ser"].writes), 1)
        self.assertEqual(monitor_updates, [])
        self.assertEqual(
            first_label.text,
            "Auto Calibration Stage 1 Failed - See Log",
        )
        self.assertEqual(second_label.text, first_label.text)

        namespace["RUN"]["ser"] = Port((response, response))
        applied_results.extend((VALID_CONTROLLER_POSITION, None))
        self.assertFalse(calibrate())
        self.assertEqual(len(namespace["RUN"]["ser"].writes), 2)
        self.assertEqual(monitor_updates, [True])
        self.assertEqual(
            first_label.text,
            "Auto Calibration Stage 2 Failed - See Log",
        )

        for axis in range(1, 10):
            calibration[f"J{axis}CalStatVal2"].value = "0"
        calibration["J8CalStatVal2"].value = "1"
        namespace["RUN"]["ser"] = Port((response, response))
        applied_results.extend(
            (VALID_CONTROLLER_POSITION, VALID_CONTROLLER_POSITION)
        )

        self.assertTrue(calibrate())
        self.assertEqual(len(namespace["RUN"]["ser"].writes), 2)
        self.assertEqual(
            first_label.text,
            "Auto Calibration Stage 2 Successful",
        )

    def test_auto_calibration_quarantines_estop_and_duplicate_framing(self):
        class Entry:
            def __init__(self, value="0"):
                self.value = value

            def get(self, *args):
                return self.value

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

        class Port:
            def __init__(self, response):
                self.response = response
                self.writes = []
                self.is_open = True
                self.timeout = None
                self.close_count = 0

            @staticmethod
            def reset_input_buffer():
                pass

            def write(self, value):
                self.writes.append(value)
                return len(value)

            @staticmethod
            def flush():
                pass

            def readline(self):
                newline_index = self.response.find(b"\n")
                size = len(self.response) if newline_index < 0 else newline_index + 1
                return self.read(size)

            def read(self, size=1):
                response = self.response[:size]
                self.response = self.response[size:]
                return response

            def close(self):
                self.close_count += 1
                self.is_open = False

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        calibration = {}
        for axis in range(1, 10):
            calibration[f"J{axis}CalStatVal"] = Entry("1")
            calibration[f"J{axis}CalStatVal2"] = Entry("1")
            calibration[f"J{axis}calOff"] = "0"

        estop_response = (
            b"A1B2C3D4E5F6G7H8I9J10K11L12"
            b"M0N42.5OEBP13Q14R15\n"
        )
        normal_response = (
            b"A1B2C3D4E5F6G7H8I9J10K11L12"
            b"M0N42.5OP13Q14R15\n"
        )
        port = Port(estop_response + normal_response)
        first_label = Label()
        second_label = Label()
        invalidations = []
        handled_faults = []
        namespace = {
            "RUN": {
                "offlineMode": False,
                "VR_angles": [0.0] * 6,
                "ser": port,
            },
            "CAL": calibration,
            "cmdSentEntryField": Entry(),
            "cmdRecEntryField": Entry(),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "tab8": SimpleNamespace(ElogView=Entry()),
            "END": "end",
            "pickle": SimpleNamespace(dump=lambda *args: None),
            "open": lambda *args: object(),
            "logger": SimpleNamespace(
                info=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "ErrorHandler": handled_faults.append,
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "finite_number": finite_number,
            "parse_position_response": parse_position_response,
            "threading": threading,
            "exchange_serial_line_until_cancelled": (
                exchange_serial_line_until_cancelled
            ),
            "application_closing": threading.Event(),
            "serial_write_lock": threading.Lock(),
            "quarantine_serial_transport": quarantine_serial_transport,
            "_invalidate_joint_motion_state": invalidations.append,
            "setStepMonitorsVR": lambda: None,
            "_current_controller_joint_calibration": (
                lambda: ControllerJointCalibration(
                    negative_limits=(180.0,) * 9,
                    positive_limits=(180.0,) * 9,
                    steps_per_unit=(1.0,) * 9,
                )
            ),
            "_apply_valid_position_response": (
                lambda response: VALID_CONTROLLER_POSITION
            ),
        }
        self.add_startup_command_dependencies(namespace)
        namespace["_binary_controller_flag"] = self.compile_function(
            "_binary_controller_flag",
            namespace,
        )
        namespace["_prepare_calibration_command"] = self.compile_function(
            "_prepare_calibration_command",
            namespace,
        )
        namespace["_calibration_available"] = self.compile_function(
            "_calibration_available",
            namespace,
        )
        namespace["_invalidate_uncertain_controller_calibration"] = (
            self.compile_function(
                "_invalidate_uncertain_controller_calibration",
                namespace,
            )
        )
        namespace["_record_calibration_response"] = self.compile_function(
            "_record_calibration_response",
            namespace,
        )
        namespace["_execute_calibration_command"] = self.compile_function(
            "_execute_calibration_command",
            namespace,
        )
        calibrate = self.compile_function("_run_program_calibration_all", namespace)

        self.assertFalse(calibrate())
        self.assertEqual(len(port.writes), 1)
        self.assertTrue(port.response)
        self.assertEqual(port.close_count, 1)
        self.assertFalse(port.is_open)
        self.assertTrue(serial_transport_quarantined(port))
        self.assertIsNone(namespace["RUN"]["ser"])
        self.assertEqual(
            invalidations,
            ["calibration response failed after controller transmission"],
        )
        self.assertEqual(
            first_label.text,
            "Auto Calibration Stage 1 Failed - See Log",
        )
        self.assertEqual(second_label.text, first_label.text)

        port = Port(estop_response)
        namespace["RUN"]["ser"] = port
        invalidations.clear()

        self.assertFalse(calibrate())
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(port.response, b"")
        self.assertEqual(port.close_count, 1)
        self.assertFalse(port.is_open)
        self.assertTrue(serial_transport_quarantined(port))
        self.assertIsNone(namespace["RUN"]["ser"])
        self.assertEqual(
            invalidations,
            ["calibration command ended with a controller motion fault"],
        )
        self.assertEqual(handled_faults, ["EB"])

        out_of_range_response = (
            b"A181B2C3D4E5F6G7H8I9J10K11L12"
            b"M0N42.5OEBP13Q14R15\n"
        )
        port = Port(out_of_range_response)
        namespace["RUN"]["ser"] = port
        invalidations.clear()
        handled_faults.clear()

        self.assertFalse(calibrate())
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(port.response, b"")
        self.assertEqual(port.close_count, 1)
        self.assertFalse(port.is_open)
        self.assertTrue(serial_transport_quarantined(port))
        self.assertIsNone(namespace["RUN"]["ser"])
        self.assertEqual(
            invalidations,
            [
                "calibration command returned a position outside calibrated "
                "limits"
            ],
        )
        self.assertEqual(handled_faults, [])

    def test_auto_calibration_validates_second_stage_before_first_write(self):
        calibration = {
            f"J{axis}CalStatVal": SimpleNamespace(get=lambda: "1")
            for axis in range(1, 10)
        }
        calibration.update(
            {
                f"J{axis}CalStatVal2": SimpleNamespace(
                    get=(lambda value="2" if axis == 8 else "0": value)
                )
                for axis in range(1, 10)
            }
        )
        executions = []
        namespace = {
            "CAL": calibration,
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
            "_calibration_available": lambda: True,
            "_prepare_calibration_command": lambda selections: (
                "LL" + "".join(str(value) for value in selections) + "\n"
            ),
            "_execute_calibration_command": (
                lambda *args, **kwargs: executions.append(args) or True
            ),
        }
        namespace["_binary_controller_flag"] = self.compile_function(
            "_binary_controller_flag",
            namespace,
        )
        calibrate = self.compile_function("_run_program_calibration_all", namespace)

        with self.assertRaisesRegex(MotionInputError, "J8 second-stage"):
            calibrate()

        self.assertEqual(executions, [])

    def test_calibration_ui_callbacks_dispatch_only_async_workers(self):
        expected_commands = {
            "autoCalBut": "startCalRobotAll",
            "CalJ1But": "startCalRobotJ1",
            "CalJ2But": "startCalRobotJ2",
            "CalJ3But": "startCalRobotJ3",
            "CalJ4But": "startCalRobotJ4",
            "CalJ5But": "startCalRobotJ5",
            "CalJ6But": "startCalRobotJ6",
            "J7calbut": "startCalRobotJ7",
            "J8calbut": "startCalRobotJ8",
            "J9calbut": "startCalRobotJ9",
        }
        actual_commands = {}
        for statement in self.tree.body:
            if (
                not isinstance(statement, ast.Assign)
                or len(statement.targets) != 1
                or not isinstance(statement.targets[0], ast.Name)
                or statement.targets[0].id not in expected_commands
                or not isinstance(statement.value, ast.Call)
            ):
                continue
            command_values = [
                keyword.value
                for keyword in statement.value.keywords
                if keyword.arg == "command"
            ]
            if len(command_values) == 1 and isinstance(command_values[0], ast.Name):
                actual_commands[statement.targets[0].id] = command_values[0].id

        self.assertEqual(actual_commands, expected_commands)

        worker = self.module_functions["_run_calibration_stage_safe"]
        referenced_names = {
            node.id for node in ast.walk(worker) if isinstance(node, ast.Name)
        }
        self.assertFalse(
            referenced_names
            & {
                "root",
                "tab1",
                "cmdSentEntryField",
                "cmdRecEntryField",
                "almStatusLab",
                "almStatusLab2",
            }
        )
        self.assertIn("exchange_serial_line_until_cancelled", referenced_names)

        for callback_name in expected_commands.values():
            callback_names = {
                node.id
                for node in ast.walk(self.module_functions[callback_name])
                if isinstance(node, ast.Name)
            }
            self.assertNotIn(
                "exchange_serial_line_until_cancelled",
                callback_names,
                callback_name,
            )

    def test_calibration_worker_reports_post_write_failure_without_tk_access(self):
        events = Queue()

        def fail_exchange(
            serial_port,
            command,
            control_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            write_started_event.set()
            raise TimeoutError("controller response timed out")

        namespace = {
            "dataclass": dataclass,
            "threading": threading,
            "exchange_serial_line_until_cancelled": fail_exchange,
            "application_closing": threading.Event(),
            "serial_write_lock": threading.Lock(),
            "calibration_serial_event_queue": events,
        }
        namespace["CalibrationWorkerResult"] = self.compile_class(
            "CalibrationWorkerResult",
            namespace,
        )
        worker = self.compile_function("_run_calibration_stage_safe", namespace)
        worker_token = object()

        worker(7, 1, worker_token, object(), "LLA1\n")

        event = events.get_nowait()
        self.assertIsInstance(event, namespace["CalibrationWorkerResult"])
        self.assertIs(event.worker_token, worker_token)
        self.assertEqual(event.request_id, 7)
        self.assertEqual(event.stage_index, 1)
        self.assertEqual(event.event_type, "failed")
        self.assertIsNone(event.response)
        self.assertEqual(event.error, "controller response timed out")
        self.assertTrue(event.write_started)

    def test_calibration_shutdown_wins_before_atomic_write_boundary(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.timeout = None
                self.commands = []

            def reset_input_buffer(self):
                pass

            def write(self, command):
                self.commands.append(command)
                return len(command)

            def flush(self):
                pass

            def read_until(self, delimiter=b"\n", size=None):
                return b""

            def read(self, size=1):
                return b""

        class ClosingBoundaryLock:
            def __init__(self, closing):
                self.closing = closing

            def acquire(self):
                self.closing.set()
                return True

            def release(self):
                pass

        closing = threading.Event()
        namespace = {
            "application_closing": closing,
            "calibration_terminal_owner_lock": threading.Lock(),
            "calibration_terminal_response_pending": threading.Event(),
        }
        require_terminal = self.compile_function(
            "_require_calibration_terminal_response",
            namespace,
            preserve_decorators=True,
        )
        write_commitment = namespace["CalibrationWriteCommitment"](
            namespace["calibration_serial_write_committed"]
        )
        port = Port()

        with self.assertRaisesRegex(
            SerialActivityRejected,
            "cancelled before transmission",
        ):
            with require_terminal(write_commitment) as cancellation_boundary:
                exchange_serial_line_until_cancelled(
                    port,
                    "LLA1\n",
                    cancellation_boundary,
                    write_lock=threading.Lock(),
                    write_boundary_lock=ClosingBoundaryLock(closing),
                    poll_interval_seconds=0.001,
                    write_started_event=write_commitment,
                )

        self.assertEqual(port.commands, [])
        self.assertFalse(write_commitment.is_set())
        self.assertFalse(namespace["calibration_serial_write_committed"].is_set())
        self.assertFalse(namespace["calibration_terminal_response_pending"].is_set())

    def test_calibration_write_boundary_latches_terminal_response(self):
        class Port:
            def __init__(self):
                self.is_open = True
                self.timeout = None
                self.commands = []
                self.response = bytearray(
                    b"A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9\n"
                )

            def reset_input_buffer(self):
                pass

            def write(self, command):
                self.commands.append(command)
                return len(command)

            def flush(self):
                pass

            def read_until(self, delimiter=b"\n", size=None):
                limit = len(self.response) if size is None else min(
                    size,
                    len(self.response),
                )
                available = bytes(self.response[:limit])
                delimiter_index = available.find(delimiter)
                count = limit if delimiter_index < 0 else delimiter_index + 1
                return self.read(count)

            def read(self, size=1):
                response = bytes(self.response[:size])
                del self.response[:size]
                return response

        class ClosingAfterCommitLock:
            def __init__(self, closing):
                self.closing = closing

            def acquire(self):
                return True

            def release(self):
                self.closing.set()

        closing = threading.Event()
        namespace = {
            "application_closing": closing,
            "calibration_terminal_owner_lock": threading.Lock(),
            "calibration_terminal_response_pending": threading.Event(),
        }
        require_terminal = self.compile_function(
            "_require_calibration_terminal_response",
            namespace,
            preserve_decorators=True,
        )
        write_commitment = namespace["CalibrationWriteCommitment"](
            namespace["calibration_serial_write_committed"]
        )
        port = Port()

        with require_terminal(write_commitment) as cancellation_boundary:
            response = exchange_serial_line_until_cancelled(
                port,
                "LLA1\n",
                cancellation_boundary,
                write_lock=threading.Lock(),
                write_boundary_lock=ClosingAfterCommitLock(closing),
                poll_interval_seconds=0.001,
                write_started_event=write_commitment,
            )

        self.assertEqual(response, "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9")
        self.assertEqual(port.commands, [b"LLA1\n"])
        self.assertTrue(closing.is_set())
        self.assertTrue(write_commitment.is_set())
        self.assertTrue(namespace["calibration_serial_write_committed"].is_set())
        self.assertFalse(namespace["calibration_terminal_response_pending"].is_set())

    def test_async_calibration_stages_settle_transport_and_motion_once(self):
        class Entry:
            def __init__(self):
                self.value = ""

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

        class Label:
            def __init__(self):
                self.configurations = []

            def config(self, **kwargs):
                self.configurations.append(kwargs)

        class Port:
            is_open = True

        motion_registry = MotionRequestRegistry()
        request_lease = motion_registry.acquire("Automatic calibration")
        activity_registry = SerialActivityRegistry(("ser",))
        activity_lease = activity_registry.lease("ser")
        transport_lock = threading.Lock()
        self.assertTrue(transport_lock.acquire(blocking=False))
        callbacks = []
        records = []
        dispatches = []
        namespace = {
            "dataclass": dataclass,
            "MotionInputError": MotionInputError,
            "MotionRequestLease": MotionRequestLease,
            "_validated_startup_command": lambda command, prefix: command,
            "calibration_operation_lock": threading.Lock(),
            "calibration_operation": None,
            "serial_lock": transport_lock,
            "motion_request_registry": motion_registry,
            "virtual_motion_event_queue": Queue(),
            "cmdRecEntryField": Entry(),
            "almStatusLab": Label(),
            "almStatusLab2": Label(),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        namespace["CalibrationStage"] = self.compile_class(
            "CalibrationStage",
            namespace,
        )
        namespace["CalibrationWorkerResult"] = self.compile_class(
            "CalibrationWorkerResult",
            namespace,
        )
        namespace["CalibrationOperation"] = self.compile_class(
            "CalibrationOperation",
            namespace,
        )
        namespace["_set_calibration_status"] = self.compile_function(
            "_set_calibration_status",
            namespace,
        )
        namespace["_finish_motion_request"] = self.compile_function(
            "_finish_motion_request",
            namespace,
        )
        namespace["_finish_calibration_operation"] = self.compile_function(
            "_finish_calibration_operation",
            namespace,
        )
        namespace["_claim_calibration_worker_result"] = self.compile_function(
            "_claim_calibration_worker_result",
            namespace,
        )

        def record(response, success, failure, **kwargs):
            records.append((response, success, failure, kwargs))
            return True

        def start_stage(operation):
            dispatches.append(operation.stage_index)
            operation.stage_snapshot = object()
            operation.worker_token = object()
            operation.worker_active = True
            return True

        namespace["_record_calibration_response"] = record
        namespace["_start_calibration_stage_worker"] = start_stage
        apply_result = self.compile_function(
            "_apply_calibration_worker_result",
            namespace,
        )

        stages = (
            namespace["CalibrationStage"]("LLA1\n", "Stage 1 OK", "Stage 1 failed"),
            namespace["CalibrationStage"]("LLA0\n", "Stage 2 OK", "Stage 2 failed"),
        )
        operation = namespace["CalibrationOperation"](
            1,
            "Automatic calibration",
            stages,
            request_lease,
            activity_lease,
            Port(),
            callbacks.append,
        )
        operation.stage_snapshot = object()
        operation.worker_token = object()
        operation.worker_active = True
        namespace["calibration_operation"] = operation

        self.assertTrue(
            apply_result(
                namespace["CalibrationWorkerResult"](
                    operation.worker_token,
                    1,
                    0,
                    "completed",
                    "stage-one",
                    None,
                    True,
                )
            )
        )
        self.assertEqual(dispatches, [1])
        self.assertTrue(transport_lock.locked())
        self.assertTrue(activity_registry.active("ser"))
        self.assertTrue(motion_registry.active)

        self.assertTrue(
            apply_result(
                namespace["CalibrationWorkerResult"](
                    operation.worker_token,
                    1,
                    1,
                    "completed",
                    "stage-two",
                    None,
                    True,
                )
            )
        )
        self.assertFalse(transport_lock.locked())
        self.assertFalse(activity_registry.active("ser"))
        self.assertFalse(motion_registry.active)
        self.assertEqual(callbacks, [True])
        self.assertEqual([record[0] for record in records], ["stage-one", "stage-two"])
        self.assertIsNone(namespace["calibration_operation"])

    def test_async_calibration_failure_runs_complete_production_lifecycle(self):
        exchanges = []

        def failed_exchange(
            serial_port,
            command,
            cancellation_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            self.assertFalse(cancellation_event.is_set())
            self.assertTrue(callable(getattr(write_boundary_lock, "acquire", None)))
            exchanges.append(command)
            write_started_event.set()
            raise TimeoutError("calibration response unavailable")

        namespace, state = self.compile_async_calibration_lifecycle(failed_exchange)
        self.assertTrue(namespace["startCalRobotAll"]())
        event = state["event_queue"].get(timeout=2)
        state["event_queue"].put(event)
        namespace["_poll_calibration_events"]()

        self.assertEqual(len(exchanges), 1)
        self.assertTrue(exchanges[0].startswith("LL"))
        self.assertEqual(len(state["captured_pose_snapshots"]), 1)
        self.assertEqual(state["restored_pose_snapshots"], [])
        self.assertEqual(
            state["invalidations"],
            ["calibration response failed after controller transmission"],
        )
        self.assertFalse(state["transport_lock"].locked())
        self.assertFalse(state["activity_registry"].active("ser"))
        self.assertFalse(state["motion_registry"].active)
        self.assertIsNone(namespace["calibration_operation"])
        self.assertFalse(namespace["calibration_terminal_response_pending"].is_set())
        self.assertEqual(
            state["first_label"].text,
            "Auto Calibration Stage 1 Failed - See Log",
        )
        self.assertEqual(state["second_label"].text, state["first_label"].text)

    def test_async_calibration_success_runs_both_production_stages(self):
        exchanges = []
        response = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"

        def successful_exchange(
            serial_port,
            command,
            cancellation_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            self.assertFalse(cancellation_event.is_set())
            self.assertTrue(callable(getattr(write_boundary_lock, "acquire", None)))
            exchanges.append(command)
            write_started_event.set()
            return response

        namespace, state = self.compile_async_calibration_lifecycle(
            successful_exchange
        )
        self.assertTrue(namespace["startCalRobotAll"]())
        deadline = time.monotonic() + 2
        while state["motion_registry"].active and time.monotonic() < deadline:
            event = state["event_queue"].get(timeout=2)
            state["event_queue"].put(event)
            namespace["_poll_calibration_events"]()

        self.assertEqual(len(exchanges), 2)
        self.assertNotEqual(exchanges[0], exchanges[1])
        self.assertTrue(all(command.startswith("LL") for command in exchanges))
        self.assertEqual(len(state["captured_pose_snapshots"]), 2)
        self.assertEqual(state["restored_pose_snapshots"], [])
        self.assertEqual(len(state["applied_positions"]), 2)
        self.assertEqual(state["invalidations"], [])
        self.assertEqual(len(state["monitor_updates"]), 2)
        self.assertFalse(state["transport_lock"].locked())
        self.assertFalse(state["activity_registry"].active("ser"))
        self.assertFalse(state["motion_registry"].active)
        self.assertIsNone(namespace["calibration_operation"])
        self.assertEqual(
            state["first_label"].text,
            "Auto Calibration Stage 2 Successful",
        )

    def test_completed_calibration_event_requires_write_commitment(self):
        exchanges = []
        response = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"

        def successful_exchange(
            serial_port,
            command,
            cancellation_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            exchanges.append(command)
            write_started_event.set()
            return response

        namespace, state = self.compile_async_calibration_lifecycle(
            successful_exchange
        )
        self.assertTrue(namespace["startCalRobotAll"]())
        event = state["event_queue"].get(timeout=2)
        rejected_event = namespace["CalibrationWorkerResult"](
            event.worker_token,
            event.request_id,
            event.stage_index,
            "completed",
            event.response,
            None,
            False,
        )
        state["event_queue"].put(rejected_event)
        namespace["_poll_calibration_events"]()

        self.assertEqual(len(exchanges), 1)
        self.assertEqual(state["applied_positions"], [])
        self.assertEqual(state["monitor_updates"], [])
        self.assertEqual(
            state["invalidations"],
            [
                "calibration worker result rejected: "
                "calibration worker emitted an invalid success result"
            ],
        )
        self.assertFalse(state["transport_lock"].locked())
        self.assertFalse(state["activity_registry"].active("ser"))
        self.assertFalse(state["motion_registry"].active)
        self.assertIsNone(namespace["calibration_operation"])

    def test_postwrite_calibration_result_failure_quarantines_before_release(self):
        response = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"
        pose_state = {"value": "pre-command"}

        def successful_exchange(
            serial_port,
            command,
            cancellation_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            write_started_event.set()
            return response

        namespace, state = self.compile_async_calibration_lifecycle(
            successful_exchange
        )
        invalidation_observations = []
        restoration_observations = []

        def capture_pose():
            return pose_state["value"]

        def restore_pose(snapshot):
            restoration_observations.append(
                (
                    snapshot,
                    pose_state["value"],
                    state["transport_lock"].locked(),
                    state["activity_registry"].active("ser"),
                    state["motion_registry"].active,
                )
            )
            pose_state["value"] = snapshot
            return True

        def display_position(raw_response, parsed=None):
            pose_state["value"] = "applied-controller-position"
            state["applied_positions"].append(parsed)
            return parsed

        def fail_after_position_application():
            self.assertEqual(pose_state["value"], "applied-controller-position")
            raise RuntimeError("injected post-position refresh failure")

        def invalidate(reason):
            invalidation_observations.append(
                (
                    reason,
                    pose_state["value"],
                    state["transport_lock"].locked(),
                    state["activity_registry"].active("ser"),
                    state["motion_registry"].active,
                )
            )

        namespace["_capture_calibration_pose_snapshot"] = capture_pose
        namespace["_restore_calibration_pose_snapshot"] = restore_pose
        namespace["displayPosition"] = display_position
        namespace["setStepMonitorsVR"] = fail_after_position_application
        namespace["_invalidate_uncertain_controller_calibration"] = invalidate
        self.assertTrue(namespace["startCalRobotAll"]())
        event = state["event_queue"].get(timeout=2)
        self.assertTrue(namespace["calibration_serial_write_committed"].is_set())
        state["event_queue"].put(event)
        namespace["_poll_calibration_events"]()

        self.assertEqual(
            restoration_observations,
            [
                (
                    "pre-command",
                    "applied-controller-position",
                    True,
                    True,
                    True,
                )
            ],
        )
        self.assertEqual(
            invalidation_observations,
            [
                (
                    "calibration result application failed after controller "
                    "transmission: injected post-position refresh failure",
                    "pre-command",
                    True,
                    True,
                    True,
                )
            ],
        )
        self.assertEqual(len(state["applied_positions"]), 1)
        self.assertEqual(pose_state["value"], "pre-command")
        self.assertFalse(state["transport_lock"].locked())
        self.assertFalse(state["activity_registry"].active("ser"))
        self.assertFalse(state["motion_registry"].active)
        self.assertIsNone(namespace["calibration_operation"])
        self.assertFalse(namespace["calibration_serial_write_committed"].is_set())

    def test_synchronous_postwrite_calibration_result_failure_quarantines(self):
        response = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"
        pose_state = {"value": "pre-command"}

        def exchange(
            serial_port,
            command,
            cancellation_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            write_started_event.set()
            return response

        namespace, state = self.compile_async_calibration_lifecycle(exchange)
        invalidations = []
        restorations = []

        def restore_pose(snapshot):
            restorations.append((snapshot, pose_state["value"]))
            pose_state["value"] = snapshot
            return True

        def display_position(raw_response, parsed=None):
            pose_state["value"] = "applied-controller-position"
            state["applied_positions"].append(parsed)
            return parsed

        def fail_after_position_application():
            self.assertEqual(pose_state["value"], "applied-controller-position")
            raise RuntimeError("injected synchronous post-position failure")

        def invalidate(reason):
            invalidations.append(
                (
                    reason,
                    pose_state["value"],
                    namespace["calibration_serial_write_committed"].is_set(),
                )
            )

        namespace["_capture_calibration_pose_snapshot"] = (
            lambda: pose_state["value"]
        )
        namespace["_restore_calibration_pose_snapshot"] = restore_pose
        namespace["displayPosition"] = display_position
        namespace["setStepMonitorsVR"] = fail_after_position_application
        namespace["_invalidate_uncertain_controller_calibration"] = invalidate
        execute = self.compile_function("_execute_calibration_command", namespace)

        self.assertFalse(execute("LLA1\n", "success", "failure"))
        self.assertEqual(
            restorations,
            [("pre-command", "applied-controller-position")],
        )
        self.assertEqual(
            invalidations,
            [
                (
                    "calibration result application failed after controller "
                    "transmission: injected synchronous post-position failure",
                    "pre-command",
                    True,
                )
            ],
        )
        self.assertEqual(len(state["applied_positions"]), 1)
        self.assertEqual(pose_state["value"], "pre-command")
        self.assertFalse(namespace["calibration_serial_write_committed"].is_set())

    def test_calibration_pose_rollback_restores_state_and_replaces_bad_save(self):
        class Widget:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

            def set(self, value):
                self.value = value

        class Root:
            def __init__(self):
                self.cancelled = []
                self.jobs = []

            def after_cancel(self, job):
                self.cancelled.append(job)

            def after(self, delay, callback):
                job = f"repair-job-{len(self.jobs) + 1}"
                self.jobs.append((job, delay, callback))
                return job

        position_keys = (
            "J1AngCur", "J2AngCur", "J3AngCur",
            "J4AngCur", "J5AngCur", "J6AngCur",
            "XcurPos", "YcurPos", "ZcurPos",
            "RzcurPos", "RycurPos", "RxcurPos",
            "J7PosCur", "J8PosCur", "J9PosCur",
        )
        entry_names = (
            "J1curAngEntryField", "J2curAngEntryField", "J3curAngEntryField",
            "J4curAngEntryField", "J5curAngEntryField", "J6curAngEntryField",
            "XcurEntryField", "YcurEntryField", "ZcurEntryField",
            "RzcurEntryField", "RycurEntryField", "RxcurEntryField",
            "J7curAngEntryField", "J8curAngEntryField", "J9curAngEntryField",
        )
        slider_names = tuple(f"J{axis}jogslide" for axis in range(1, 10))
        root = Root()
        acknowledged_target = tuple(float(axis) for axis in range(1, 10))
        namespace = {
            "CALIBRATION_POSITION_KEYS": position_keys,
            "CAL": {
                key: f"saved-position-{index}"
                for index, key in enumerate(position_keys, start=1)
            },
            "RUN": {
                "WC": "N",
                "VR_angles": [float(value) for value in range(1, 7)],
                "StepMonitors": [float(value) for value in range(11, 17)],
                **{
                    f"J{axis}StepM": float(axis + 20)
                    for axis in range(1, 7)
                },
            },
            "confirmed_position_generation": 4,
            "acknowledged_forced_position_lock": threading.Lock(),
            "acknowledged_forced_position_target": acknowledged_target,
            "controller_position_resynchronization_required": threading.Event(),
            "_calibration_dirty": False,
            "_calibration_save_job": None,
            "application_closing": threading.Event(),
            "CALIBRATION_SAVE_DEBOUNCE_MS": 250,
            "_write_pending_calibration": lambda: True,
            "root": root,
            "manEntryField": Widget("saved-debug"),
            "finite_number": finite_number,
        }
        namespace.update({
            name: Widget(f"saved-entry-{index}")
            for index, name in enumerate(entry_names, start=1)
        })
        namespace.update({
            name: Widget(float(index))
            for index, name in enumerate(slider_names, start=1)
        })
        namespace["_acknowledged_forced_position_target_value"] = (
            self.compile_function(
                "_acknowledged_forced_position_target_value",
                namespace,
            )
        )
        namespace["_calibration_pose_widget_groups"] = self.compile_function(
            "_calibration_pose_widget_groups",
            namespace,
        )
        capture = self.compile_function(
            "_capture_calibration_pose_snapshot",
            namespace,
        )
        restore = self.compile_function(
            "_restore_calibration_pose_snapshot",
            namespace,
        )
        snapshot = capture()

        for key in position_keys:
            namespace["CAL"][key] = "uncommitted-position"
        namespace["RUN"]["WC"] = "F"
        namespace["RUN"]["VR_angles"] = [90.0] * 6
        namespace["RUN"]["StepMonitors"] = [900.0] * 6
        for axis in range(1, 7):
            namespace["RUN"][f"J{axis}StepM"] = 900.0
        for name in entry_names + slider_names:
            namespace[name].set("uncommitted-widget")
        namespace["manEntryField"].set("uncommitted-debug")
        namespace["confirmed_position_generation"] = 5
        namespace["acknowledged_forced_position_target"] = None
        namespace["controller_position_resynchronization_required"].set()
        namespace["_calibration_dirty"] = True
        namespace["_calibration_save_job"] = "bad-result-save"

        self.assertTrue(restore(snapshot))
        self.assertEqual(
            tuple(namespace["CAL"][key] for key in position_keys),
            tuple(f"saved-position-{index}" for index in range(1, 16)),
        )
        self.assertEqual(namespace["RUN"]["WC"], "N")
        self.assertEqual(namespace["RUN"]["VR_angles"], list(range(1, 7)))
        self.assertEqual(namespace["RUN"]["StepMonitors"], list(range(11, 17)))
        self.assertEqual(
            tuple(namespace["RUN"][f"J{axis}StepM"] for axis in range(1, 7)),
            tuple(range(21, 27)),
        )
        self.assertEqual(
            tuple(namespace[name].get() for name in entry_names),
            tuple(f"saved-entry-{index}" for index in range(1, 16)),
        )
        self.assertEqual(
            tuple(namespace[name].get() for name in slider_names),
            tuple(float(index) for index in range(1, 10)),
        )
        self.assertEqual(namespace["manEntryField"].get(), "saved-debug")
        self.assertEqual(namespace["confirmed_position_generation"], 4)
        self.assertEqual(
            namespace["acknowledged_forced_position_target"],
            acknowledged_target,
        )
        self.assertFalse(
            namespace["controller_position_resynchronization_required"].is_set()
        )
        self.assertEqual(root.cancelled, ["bad-result-save"])
        self.assertTrue(namespace["_calibration_dirty"])
        self.assertEqual(namespace["_calibration_save_job"], "repair-job-1")
        self.assertEqual(len(root.jobs), 1)

    def test_unowned_malformed_calibration_event_retains_active_ownership(self):
        response = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"
        worker_started = threading.Event()
        worker_release = threading.Event()

        def blocked_exchange(
            serial_port,
            command,
            cancellation_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            write_started_event.set()
            worker_started.set()
            if not worker_release.wait(2):
                raise TimeoutError("test worker release timed out")
            return response

        namespace, state = self.compile_async_calibration_lifecycle(
            blocked_exchange
        )
        try:
            self.assertTrue(namespace["startCalRobotAll"]())
            self.assertTrue(worker_started.wait(2))
            state["event_queue"].put(("malformed",))
            namespace["_poll_calibration_events"]()

            operation = namespace["calibration_operation"]
            self.assertIsNotNone(operation)
            self.assertTrue(operation.worker_active)
            self.assertIsNotNone(operation.worker_token)
            self.assertTrue(state["transport_lock"].locked())
            self.assertTrue(state["activity_registry"].active("ser"))
            self.assertTrue(state["motion_registry"].active)
            self.assertEqual(state["applied_positions"], [])
            self.assertEqual(state["invalidations"], [])
        finally:
            worker_release.set()

        deadline = time.monotonic() + 2
        while state["motion_registry"].active and time.monotonic() < deadline:
            event = state["event_queue"].get(timeout=2)
            state["event_queue"].put(event)
            namespace["_poll_calibration_events"]()

        self.assertEqual(len(state["applied_positions"]), 2)
        self.assertFalse(state["transport_lock"].locked())
        self.assertFalse(state["activity_registry"].active("ser"))
        self.assertFalse(state["motion_registry"].active)
        self.assertIsNone(namespace["calibration_operation"])

    def test_async_single_joint_calibration_uses_one_hot_selection_policy(self):
        exchanges = []
        response = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"

        def successful_exchange(
            serial_port,
            command,
            cancellation_event,
            *,
            write_lock,
            write_boundary_lock,
            write_started_event,
        ):
            self.assertFalse(cancellation_event.is_set())
            self.assertTrue(callable(getattr(write_boundary_lock, "acquire", None)))
            exchanges.append(command)
            write_started_event.set()
            return response

        namespace, state = self.compile_async_calibration_lifecycle(
            successful_exchange
        )
        for joint in range(1, 10):
            with self.subTest(joint=joint):
                monitor_count = len(state["monitor_updates"])
                callback = namespace[f"startCalRobotJ{joint}"]
                self.assertTrue(callback())
                deadline = time.monotonic() + 2
                while state["motion_registry"].active and time.monotonic() < deadline:
                    event = state["event_queue"].get(timeout=2)
                    state["event_queue"].put(event)
                    namespace["_poll_calibration_events"]()

                selection_prefix = "LL" + "".join(
                    f"{label}{int(axis == joint)}"
                    for axis, label in enumerate("ABCDEFGHI", start=1)
                )
                self.assertTrue(exchanges[-1].startswith(selection_prefix + "J"))
                expected_monitor_count = monitor_count + int(joint <= 6)
                self.assertEqual(
                    len(state["monitor_updates"]),
                    expected_monitor_count,
                )
                self.assertFalse(state["transport_lock"].locked())
                self.assertFalse(state["activity_registry"].active("ser"))
                self.assertFalse(state["motion_registry"].active)
                self.assertIsNone(namespace["calibration_operation"])

        self.assertEqual(len(exchanges), 9)

    def test_position_resynchronization_consumers_stop_on_failure(self):
        class Port:
            def __init__(self):
                self.writes = []

            def write(self, value):
                self.writes.append(value)

            @staticmethod
            def flushInput():
                pass

            @staticmethod
            def readline():
                return b"malformed"

            @staticmethod
            def read():
                return b"Done"

        request_namespace = {
            "RUN": {"ser": Port()},
            "_exchange_serial_line": lambda command: "malformed",
            "_apply_controller_position_response": lambda response: False,
        }
        request_position = self.compile_function(
            "requestPos",
            request_namespace,
        )
        self.assertFalse(request_position())

        class Widget:
            def __init__(self):
                self.configurations = []

            def config(self, **kwargs):
                self.configurations.append(kwargs)

        runtime = {"offlineMode": True, "VR_angles": [9.0] * 6}
        monitor_updates = []

        @contextmanager
        def reserve_mode_transport():
            yield

        toggle_namespace = {
            "RUN": runtime,
            "CAL": {f"J{axis}AngCur": str(axis) for axis in range(1, 7)},
            "offline_button": Widget(),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
            "requestPos": lambda: False,
            "setStepMonitorsVR": lambda: monitor_updates.append(True),
            "_mode_change_is_blocked": lambda *args, **kwargs: False,
            "_reserve_main_serial_operation": reserve_mode_transport,
            "SerialActivityRejected": SerialActivityRejected,
        }
        toggle_namespace["_set_offline_mode_status"] = self.compile_function(
            "_set_offline_mode_status",
            toggle_namespace,
        )
        toggle = self.compile_function("toggle_offline_mode", toggle_namespace)
        self.assertFalse(toggle())
        self.assertTrue(runtime["offlineMode"])
        self.assertEqual(runtime["VR_angles"], [9.0] * 6)
        self.assertEqual(monitor_updates, [])
        self.assertEqual(
            toggle_namespace["offline_button"].configurations[-1],
            {"text": "Go Online", "style": "Offline.TButton"},
        )
        for label_name in ("almStatusLab", "almStatusLab2"):
            self.assertEqual(
                toggle_namespace[label_name].configurations[-1],
                {"text": "SYSTEM IN OFFLINE MODE", "style": "Warn.TLabel"},
            )

        for function_name in ("CalZeroPos", "CalRestPos"):
            with self.subTest(function=function_name):
                labels = (Widget(), Widget())
                runtime = {
                    "ser": Port(),
                    "VR_angles": [9.0] * 6,
                }
                forced_updates = []
                namespace = {
                    "RUN": runtime,
                    "CAL": {
                        f"J{axis}AngCur": str(axis)
                        for axis in range(1, 7)
                    },
                    "_force_controller_position": lambda positions: False,
                    "almStatusLab": labels[0],
                    "almStatusLab2": labels[1],
                    "setStepMonitorsVR": (
                        lambda: forced_updates.append(True)
                    ),
                }
                force_position = self.compile_function(
                    function_name,
                    namespace,
                )

                self.assertFalse(force_position())
                self.assertEqual(runtime["VR_angles"], [9.0] * 6)
                self.assertEqual(forced_updates, [])
                self.assertEqual(labels[0].configurations, [])
                self.assertEqual(labels[1].configurations, [])

    def test_set_position_host_exchange_and_firmware_source_contract(self):
        firmware_source = TEENSY_SOURCE.read_text(encoding="utf-8")
        firmware_start = firmware_source.index('if (function == "SP")')
        firmware_end = firmware_source.index(
            '//-----COMMAND ECHO TEST MESSAGE',
            firmware_start,
        )
        firmware_branch = firmware_source[firmware_start:firmware_end]
        for marker in ("'G'", "'H'", "'I'"):
            self.assertIn(f"inData.indexOf({marker})", firmware_branch)
        self.assertIn('Serial.println("Done");', firmware_branch)

        position_response = (
            b"A1B2C3D4E5F6G7H8I9J10K11L12"
            b"M0N42.5OP13Q14R15\n"
        )

        class FirmwarePort:
            def __init__(self):
                self.is_open = True
                self.timeout = None
                self.response = bytearray()
                self.writes = []
                self.discarded = []

            def reset_input_buffer(self):
                if self.response:
                    self.discarded.append(bytes(self.response))
                self.response.clear()

            def flushInput(self):
                self.reset_input_buffer()

            def write(self, value):
                self.writes.append(value)
                if value.startswith(b"SP"):
                    self.response.extend(b"Done\n")
                elif value == b"RP\n":
                    self.response.extend(position_response)
                else:
                    raise AssertionError(f"unexpected controller command: {value!r}")
                return len(value)

            @staticmethod
            def flush():
                pass

            def read(self, size=1):
                value = bytes(self.response[:size])
                del self.response[:size]
                return value

            def read_until(self, delimiter=b"\n", size=None):
                limit = len(self.response) if size is None else min(
                    size,
                    len(self.response),
                )
                available = bytes(self.response[:limit])
                delimiter_index = available.find(delimiter)
                count = (
                    limit
                    if delimiter_index < 0
                    else delimiter_index + len(delimiter)
                )
                return self.read(count)

            def readline(self):
                return self.read_until()

            def close(self):
                self.is_open = False

        def position_namespace(port):
            invalidations = []
            namespace = {
                "RUN": {"ser": port},
                "threading": threading,
                "serial_write_lock": threading.Lock(),
                "write_serial_control": write_serial_control,
                "read_serial_line_response": read_serial_line_response,
                "SERIAL_STARTUP_READ_TIMEOUT_SECONDS": 1.0,
                "ProtocolResponseError": ProtocolResponseError,
                "_invalidate_joint_motion_state": invalidations.append,
                "logger": SimpleNamespace(exception=lambda *args: None),
            }
            self.add_startup_command_dependencies(namespace)
            namespace["_exchange_position_acknowledgement"] = (
                self.compile_function(
                    "_exchange_position_acknowledgement",
                    namespace,
                )
            )
            return namespace, invalidations

        port = FirmwarePort()
        namespace, invalidations = position_namespace(port)
        namespace["_prepare_position_command"] = (
            lambda: "SPA1B2C3D4E5F6G7H8I9\n"
        )
        send_position = self.compile_function("sendPos", namespace)

        self.assertTrue(send_position())
        self.assertEqual(
            port.writes,
            [b"SPA1B2C3D4E5F6G7H8I9\n"],
        )
        self.assertEqual(port.response, b"")
        self.assertEqual(port.discarded, [])
        self.assertEqual(invalidations, [])

        expected_commands = {
            "CalZeroPos": b"SPA0B0C0D0E45F0G7H8I9\n",
            "CalRestPos": b"SPA0B0C-89D0E0F0G7H8I9\n",
        }
        for function_name, expected_command in expected_commands.items():
            with self.subTest(function=function_name):
                port = FirmwarePort()
                namespace, invalidations = position_namespace(port)
                namespace.update(
                    {
                        "CAL": {
                            f"J{axis}AngCur": str(axis)
                            for axis in range(1, 7)
                        },
                        "_current_joint_positions": (
                            lambda: tuple(float(axis) for axis in range(1, 10))
                        ),
                        "_current_controller_joint_calibration": (
                            lambda: ControllerJointCalibration(
                                negative_limits=(180.0,) * 9,
                                positive_limits=(180.0,) * 9,
                                steps_per_unit=(1.0,) * 9,
                            )
                        ),
                        "finite_number": finite_number,
                    }
                )
                namespace["_prepare_forced_position_request"] = (
                    self.compile_function(
                        "_prepare_forced_position_request",
                        namespace,
                    )
                )
                namespace["_prepare_forced_position_command"] = (
                    self.compile_function(
                        "_prepare_forced_position_command",
                        namespace,
                    )
                )
                applied_positions = []

                def exchange_controller(command):
                    return exchange_serial_line(
                        port,
                        command,
                        1.0,
                        write_lock=namespace["serial_write_lock"],
                    )

                namespace["_exchange_serial_line"] = exchange_controller
                namespace["_clear_acknowledged_forced_position_target"] = (
                    self.compile_function(
                        "_clear_acknowledged_forced_position_target",
                        namespace,
                    )
                )

                def apply_position(response):
                    applied_positions.append(response)
                    namespace["_clear_acknowledged_forced_position_target"]()
                    namespace[
                        "controller_position_resynchronization_required"
                    ].clear()
                    return True

                namespace["_apply_controller_position_response"] = apply_position
                namespace["requestPos"] = self.compile_function(
                    "requestPos",
                    namespace,
                )

                class Widget:
                    def __init__(self):
                        self.configurations = []

                    def config(self, **kwargs):
                        self.configurations.append(kwargs)

                monitor_updates = []
                namespace.update(
                    {
                        "almStatusLab": Widget(),
                        "almStatusLab2": Widget(),
                        "tab8": SimpleNamespace(
                            ElogView=SimpleNamespace(get=lambda *args: ())
                        ),
                        "END": "end",
                        "pickle": SimpleNamespace(dump=lambda *args: None),
                        "open": lambda *args, **kwargs: object(),
                        "setStepMonitorsVR": (
                            lambda: monitor_updates.append(True)
                        ),
                        "logger": SimpleNamespace(
                            warning=lambda *args: None,
                            exception=lambda *args: None,
                        ),
                    }
                )
                namespace["_record_acknowledged_forced_position_target"] = (
                    self.compile_function(
                        "_record_acknowledged_forced_position_target",
                        namespace,
                    )
                )
                namespace["_force_controller_position"] = self.compile_function(
                    "_force_controller_position",
                    namespace,
                )
                force_position = self.compile_function(
                    function_name,
                    namespace,
                )

                self.assertTrue(force_position())
                self.assertEqual(
                    port.writes,
                    [expected_command, b"RP\n"],
                )
                self.assertEqual(port.response, b"")
                self.assertEqual(port.discarded, [])
                self.assertEqual(len(invalidations), 1)
                self.assertIn("forced position", invalidations[0])
                self.assertIsNone(
                    namespace["acknowledged_forced_position_target"]
                )
                self.assertFalse(
                    namespace[
                        "controller_position_resynchronization_required"
                    ].is_set()
                )
                self.assertEqual(
                    applied_positions,
                    [position_response[:-1].decode("ascii")],
                )
                self.assertEqual(
                    namespace["RUN"]["VR_angles"],
                    [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                )
                self.assertEqual(monitor_updates, [True])

    def test_forced_position_post_ack_failure_retains_reconnect_target(self):
        target = (0.0, 0.0, -89.0, 0.0, 0.0, 0.0, 7.0, 8.0, 9.0)
        stale_position = tuple(float(axis) for axis in range(1, 10))

        for outcome in (False, RuntimeError("RP failed")):
            with self.subTest(outcome=outcome):
                invalidations = []
                labels = []

                def request_position():
                    if isinstance(outcome, Exception):
                        raise outcome
                    return outcome

                namespace = {
                    "finite_number": finite_number,
                    "_prepare_forced_position_request": (
                        lambda positions: (
                            "SPA0B0C-89D0E0F0G7H8I9\n",
                            target,
                        )
                    ),
                    "_exchange_position_acknowledgement": lambda command: True,
                    "requestPos": request_position,
                    "_invalidate_joint_motion_state": invalidations.append,
                    "logger": SimpleNamespace(exception=lambda *args: None),
                    "almStatusLab": SimpleNamespace(
                        config=lambda **kwargs: labels.append(kwargs)
                    ),
                    "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
                    "_current_joint_positions": lambda: stale_position,
                    "_current_controller_joint_calibration": (
                        lambda: ControllerJointCalibration(
                            negative_limits=(180.0,) * 9,
                            positive_limits=(180.0,) * 9,
                            steps_per_unit=(1.0,) * 9,
                        )
                    ),
                }
                self.add_startup_command_dependencies(namespace)
                namespace["_record_acknowledged_forced_position_target"] = (
                    self.compile_function(
                        "_record_acknowledged_forced_position_target",
                        namespace,
                    )
                )
                force_position = self.compile_function(
                    "_force_controller_position",
                    namespace,
                )
                prepare_reconnect = self.compile_function(
                    "_prepare_position_command",
                    namespace,
                )

                self.assertFalse(force_position((0, 0, -89, 0, 0, 0)))
                self.assertEqual(
                    namespace["acknowledged_forced_position_target"],
                    target,
                )
                self.assertTrue(
                    namespace[
                        "controller_position_resynchronization_required"
                    ].is_set()
                )
                self.assertEqual(len(invalidations), 1)
                self.assertEqual(
                    prepare_reconnect(),
                    "SPA0B0C-89D0E0F0G7H8I9\n",
                )
                self.assertEqual(bool(labels), isinstance(outcome, Exception))

    def test_virtual_gui_refresh_propagates_fk_failure_without_mutation(self):
        runtime = {"VR_angles": [1.0] * 6, "sentinel": "unchanged"}
        calibration = {"XcurPos": "unchanged"}
        step_updates = []
        namespace = {
            "RUN": runtime,
            "CAL": calibration,
            "math": math,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "robot": SimpleNamespace(
                forward_kinematics=lambda joints: (_ for _ in ()).throw(
                    RuntimeError("native FK failed")
                )
            ),
            "setStepMonitorsVR": lambda: step_updates.append(True),
        }
        namespace["_validated_virtual_six_vector"] = self.compile_function(
            "_validated_virtual_six_vector",
            namespace,
        )
        refresh = self.compile_function(
            "refresh_gui_from_joint_angles",
            namespace,
        )

        with self.assertRaisesRegex(
            MotionInputError,
            "Forward kinematics refresh failed: native FK failed",
        ):
            refresh((2.0,) * 6)

        self.assertEqual(runtime, {"VR_angles": [1.0] * 6, "sentinel": "unchanged"})
        self.assertEqual(calibration, {"XcurPos": "unchanged"})
        self.assertEqual(step_updates, [])

        for name in ("driveMotorsJ", "run_driveMotorsJ_safe"):
            refresh_calls = [
                node
                for node in ast.walk(self.module_functions[name])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "refresh_gui_from_joint_angles"
            ]
            self.assertEqual(refresh_calls, [], name)

    def test_virtual_gui_refresh_applies_validated_pose_once(self):
        class Widget:
            def __init__(self):
                self.events = []

            def delete(self, *args):
                self.events.append(("delete", args))

            def insert(self, *args):
                self.events.append(("insert", args))

            def set(self, value):
                self.events.append(("set", value))

        joints = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        runtime = {"VR_angles": [0.0] * 6, "WC": "N"}
        calibration = {}
        step_updates = []
        cartesian_fields = [Widget() for _ in range(6)]
        joint_fields = [Widget() for _ in range(6)]
        sliders = [Widget() for _ in range(6)]
        namespace = {
            "RUN": runtime,
            "CAL": calibration,
            "math": math,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "robot": SimpleNamespace(
                forward_kinematics=lambda values: (
                    10.0,
                    20.0,
                    30.0,
                    math.radians(40.0),
                    math.radians(50.0),
                    math.radians(60.0),
                )
            ),
            "setStepMonitorsVR": (
                lambda: step_updates.append(tuple(runtime["VR_angles"]))
            ),
            "logger": SimpleNamespace(info=lambda *args: None),
        }
        for name, field in zip(
            (
                "XcurEntryField",
                "YcurEntryField",
                "ZcurEntryField",
                "RzcurEntryField",
                "RycurEntryField",
                "RxcurEntryField",
            ),
            cartesian_fields,
        ):
            namespace[name] = field
        for axis, field, slider in zip(
            range(1, 7),
            joint_fields,
            sliders,
        ):
            namespace[f"J{axis}curAngEntryField"] = field
            namespace[f"J{axis}jogslide"] = slider
        namespace["_validated_virtual_six_vector"] = self.compile_function(
            "_validated_virtual_six_vector",
            namespace,
        )
        refresh = self.compile_function(
            "refresh_gui_from_joint_angles",
            namespace,
        )

        self.assertTrue(refresh(joints))

        self.assertEqual(runtime["VR_angles"], list(joints))
        self.assertEqual(runtime["WC"], "F")
        self.assertEqual(step_updates, [joints])
        self.assertEqual(
            tuple(
                calibration[key]
                for key in (
                    "XcurPos",
                    "YcurPos",
                    "ZcurPos",
                    "RzcurPos",
                    "RycurPos",
                    "RxcurPos",
                )
            ),
            ("10.000", "20.000", "30.000", "60.000", "50.000", "40.000"),
        )
        self.assertEqual(
            tuple(calibration[f"J{axis}AngCur"] for axis in range(1, 7)),
            tuple(str(value) for value in joints),
        )
        self.assertTrue(
            all(len(field.events) == 2 for field in cartesian_fields)
        )
        self.assertTrue(all(len(field.events) == 2 for field in joint_fields))
        self.assertTrue(all(len(slider.events) == 1 for slider in sliders))

    def test_mode_transition_rejects_fk_refresh_failure(self):
        class Widget:
            def config(self, **kwargs):
                pass

        @contextmanager
        def reserve_transport():
            yield

        saved_pose = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        runtime = {"offlineMode": False, "VR_angles": list(saved_pose)}
        statuses = []
        step_updates = []
        errors = []
        namespace = {
            "RUN": runtime,
            "CAL": {
                **{f"J{axis}NegLim": "100" for axis in range(1, 7)},
                **{f"J{axis}PosLim": "100" for axis in range(1, 7)},
            },
            "_reserve_main_serial_operation": reserve_transport,
            "_mode_change_is_blocked": lambda *args, **kwargs: False,
            "_set_offline_mode_status": statuses.append,
            "refresh_gui_from_joint_angles": (
                lambda target: (_ for _ in ()).throw(
                    MotionInputError("native FK failed")
                )
            ),
            "setStepMonitorsVR": (
                lambda: step_updates.append(tuple(runtime["VR_angles"]))
            ),
            "SerialActivityRejected": SerialActivityRejected,
            "MotionInputError": MotionInputError,
            "HORIZONTAL": "horizontal",
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda message: errors.append(message),
            ),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
        }
        for axis in range(1, 7):
            namespace[f"J{axis}negLimLab"] = Widget()
            namespace[f"J{axis}posLimLab"] = Widget()
            namespace[f"J{axis}jogslide"] = Widget()
            namespace[f"J{axis}sliderUpdate"] = lambda value: None
        toggle = self.compile_function("toggle_offline_mode", namespace)

        self.assertFalse(toggle())
        self.assertFalse(runtime["offlineMode"])
        self.assertEqual(runtime["VR_angles"], saved_pose)
        self.assertEqual(statuses, [True, False])
        self.assertEqual(step_updates, [tuple(saved_pose)])
        self.assertIn("native FK failed", errors[-1])
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_mode_change_rejects_every_physical_and_virtual_owner(self):
        owner_names = (
            "virtual",
            "controller-live",
            "legacy-serial",
            "dispatcher",
            "serial-lock",
            "controller-correction",
        )
        for owner_name in owner_names:
            with self.subTest(owner=owner_name):
                live_pending = threading.Event()
                legacy_pending = threading.Event()
                serial_lock = threading.Lock()
                dispatcher = SimpleNamespace(active=False)
                virtual_active = {"value": False}
                correction_requested = threading.Event()
                if owner_name == "virtual":
                    virtual_active["value"] = True
                elif owner_name == "controller-live":
                    live_pending.set()
                elif owner_name == "legacy-serial":
                    legacy_pending.set()
                elif owner_name == "dispatcher":
                    dispatcher.active = True
                elif owner_name == "serial-lock":
                    serial_lock.acquire()
                elif owner_name == "controller-correction":
                    correction_requested.set()

                @contextmanager
                def reserve_mode_transport():
                    if not serial_lock.acquire(blocking=False):
                        raise SerialActivityRejected(
                            "controller transport is busy"
                        )
                    try:
                        yield
                    finally:
                        serial_lock.release()

                requests = []
                runtime = {"offlineMode": False, "VR_angles": [1.0] * 6}
                namespace = {
                    "RUN": runtime,
                    "application_closing": threading.Event(),
                    "_virtual_motion_active": (
                        lambda ignored=None: virtual_active["value"]
                    ),
                    "live_serial_result_pending": live_pending,
                    "legacy_serial_result_pending": legacy_pending,
                    "joint_motion_dispatcher": dispatcher,
                    "serial_lock": serial_lock,
                    "controller_correction_requested": correction_requested,
                    "_reserve_main_serial_operation": reserve_mode_transport,
                    "SerialActivityRejected": SerialActivityRejected,
                    "logger": SimpleNamespace(warning=lambda *args: None),
                    "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
                    "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
                    "requestPos": lambda: requests.append(True),
                }
                namespace["_mode_change_is_blocked"] = self.compile_function(
                    "_mode_change_is_blocked",
                    namespace,
                )
                toggle = self.compile_function("toggle_offline_mode", namespace)

                try:
                    self.assertFalse(toggle())
                    self.assertFalse(runtime["offlineMode"])
                    self.assertEqual(requests, [])
                    self.assertEqual(
                        correction_requested.is_set(),
                        owner_name == "controller-correction",
                    )
                finally:
                    if serial_lock.locked():
                        serial_lock.release()

    def test_mode_transition_holds_motion_and_transport_through_commit(self):
        class Widget:
            def config(self, **kwargs):
                pass

        serial_lock = threading.Lock()
        registry = MotionRequestRegistry()
        attempts = []

        @contextmanager
        def reserve_mode_transport():
            if not serial_lock.acquire(blocking=False):
                raise SerialActivityRejected("controller transport is busy")
            try:
                yield
            finally:
                serial_lock.release()

        def request_position():
            def attempt_competing_motion():
                transport_acquired = serial_lock.acquire(blocking=False)
                if transport_acquired:
                    serial_lock.release()
                lease = registry.acquire("Concurrent program motion")
                motion_acquired = lease is not None
                if lease is not None:
                    lease.close()
                attempts.append((transport_acquired, motion_acquired))

            worker = threading.Thread(target=attempt_competing_motion)
            worker.start()
            worker.join(2)
            self.assertFalse(worker.is_alive())
            return True

        runtime = {"offlineMode": True, "VR_angles": [9.0] * 6}
        namespace = {
            "RUN": runtime,
            "CAL": {f"J{axis}AngCur": str(axis) for axis in range(1, 7)},
            "motion_request_registry": registry,
            "serial_lock": serial_lock,
            "application_closing": threading.Event(),
            "controller_correction_requested": threading.Event(),
            "live_serial_result_pending": threading.Event(),
            "legacy_serial_result_pending": threading.Event(),
            "joint_motion_dispatcher": SimpleNamespace(active=False),
            "_virtual_motion_active": lambda ignored=None: False,
            "_reserve_main_serial_operation": reserve_mode_transport,
            "SerialActivityRejected": SerialActivityRejected,
            "requestPos": request_position,
            "offline_button": Widget(),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
            "setStepMonitorsVR": lambda: None,
            "logger": SimpleNamespace(warning=lambda *args: None),
        }
        namespace["_mode_change_is_blocked"] = self.compile_function(
            "_mode_change_is_blocked",
            namespace,
        )
        namespace["_set_offline_mode_status"] = self.compile_function(
            "_set_offline_mode_status",
            namespace,
        )
        toggle = self.compile_function("toggle_offline_mode", namespace)

        self.assertTrue(toggle())
        self.assertFalse(runtime["offlineMode"])
        self.assertEqual(attempts, [(False, False)])
        self.assertFalse(serial_lock.locked())
        self.assertFalse(registry.active)

    def test_mode_transition_acquires_logical_owner_before_transport(self):
        order = []
        lease = object()

        def acquire(name):
            self.assertEqual(name, "Mode transition")
            order.append("logical-acquire")
            return lease

        @contextmanager
        def reserve_transport():
            order.append("transport-enter")
            try:
                yield
            finally:
                order.append("transport-exit")

        class Widget:
            def config(self, **kwargs):
                pass

        namespace = {
            "RUN": {"offlineMode": True, "VR_angles": [9.0] * 6},
            "CAL": {f"J{axis}AngCur": str(axis) for axis in range(1, 7)},
            "_acquire_motion_request": acquire,
            "_finish_motion_request": (
                lambda value: order.append("logical-release") or True
            ),
            "_reserve_main_serial_operation": reserve_transport,
            "_mode_change_is_blocked": lambda *args, **kwargs: False,
            "requestPos": lambda: order.append("request-position") or False,
            "_set_offline_mode_status": lambda offline: None,
            "SerialActivityRejected": SerialActivityRejected,
            "logger": SimpleNamespace(warning=lambda *args: None),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
        }
        toggle = self.compile_function("toggle_offline_mode", namespace)

        self.assertFalse(toggle())
        self.assertEqual(
            order,
            [
                "logical-acquire",
                "transport-enter",
                "request-position",
                "transport-exit",
                "logical-release",
            ],
        )

    def test_offline_controller_correction_remains_queued(self):
        correction_requested = threading.Event()
        correction_requested.set()
        correction_namespace = {
            "controller_correction_requested": correction_requested,
            "controller_correction_state_lock": threading.Lock(),
            "application_closing": threading.Event(),
            "RUN": {"offlineMode": True},
            "serial_lock": threading.Lock(),
        }
        dispatch_correction = self.compile_function(
            "_try_dispatch_controller_correction",
            correction_namespace,
        )

        self.assertFalse(dispatch_correction())
        self.assertTrue(correction_requested.is_set())

    def test_joint_jog_handlers_are_unique_thin_wrappers(self):
        function_counts = {}
        for node in self.tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                function_counts[node.name] = function_counts.get(node.name, 0) + 1

        for axis in range(1, 10):
            for direction in ("Neg", "Pos"):
                name = f"J{axis}jog{direction}"
                self.assertEqual(function_counts.get(name), 1, name)
                function = self.module_functions[name]
                calls = [node for node in ast.walk(function) if isinstance(node, ast.Call)]
                self.assertEqual(len(calls), 1, name)
                self.assertIsInstance(calls[0].func, ast.Name, name)
                self.assertEqual(calls[0].func.id, "_queue_joint_jog", name)

    def test_joint_sliders_submit_absolute_targets(self):
        for axis in range(9):
            name = f"J{axis + 1}sliderExecute"
            function = self.module_functions[name]
            calls = [node for node in ast.walk(function) if isinstance(node, ast.Call)]
            target_calls = [
                node
                for node in calls
                if isinstance(node.func, ast.Name)
                and node.func.id == "_queue_joint_target"
            ]
            self.assertEqual(len(target_calls), 1, name)
            self.assertIsInstance(target_calls[0].args[0], ast.Constant, name)
            self.assertEqual(target_calls[0].args[0].value, axis, name)

    def test_joint_motion_host_routing_uses_semantic_target(self):
        class Label:
            def __init__(self):
                self.events = []

            def config(self, **kwargs):
                self.events.append(kwargs)

        class Logger:
            def __init__(self):
                self.errors = []

            def error(self, message):
                self.errors.append(message)

        class Dispatcher:
            def __init__(self):
                self.calls = []

            def submit_target(self, axis, target, actual, profile):
                self.calls.append(("target", axis, target, actual, profile))
                values = list(actual)
                values[axis] = target
                return SimpleNamespace(target=tuple(values), coalesced=True)

            def submit_delta(self, axis, delta, actual, profile):
                self.calls.append(("delta", axis, delta, actual, profile))
                values = list(actual)
                values[axis] += delta
                return SimpleNamespace(target=tuple(values), coalesced=False)

        class Deferred:
            pending = False

        actual = (0.0,) * 9
        profile = object()
        dispatcher = Dispatcher()
        virtual_targets = []
        first_label = Label()
        second_label = Label()
        logger = Logger()
        namespace = {
            "RUN": {"xboxUse": 1, "offlineMode": False, "VR_angles": [0.0] * 6},
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "controller_correction_requested": threading.Event(),
            "math": math,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "MotionTransportBusy": MotionTransportBusy,
            "_current_joint_motion_profile": lambda: profile,
            "_current_joint_positions": lambda: actual,
            "deferred_joint_adjustments": Deferred(),
            "joint_motion_dispatcher": dispatcher,
            "_defer_joint_target": lambda *args: True,
            "_defer_joint_adjustment": lambda *args: True,
            "_try_set_virtual_joint_target": virtual_targets.append,
            "build_virtual_joint_command": lambda positions, move_profile: "virtual",
            "rj_command": lambda command: None,
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "logger": logger,
        }
        route = self.compile_function("_queue_joint_motion", namespace)

        accepted = route(0, 20, absolute=True)

        self.assertTrue(accepted)
        self.assertEqual(dispatcher.calls, [("target", 0, 20.0, actual, profile)])
        self.assertEqual(virtual_targets, [(20.0,) + (0.0,) * 8])
        self.assertEqual(logger.errors, [])

        self.assertFalse(route(0, True, absolute=True))
        self.assertEqual(dispatcher.calls, [("target", 0, 20.0, actual, profile)])
        self.assertIn("must be numeric", logger.errors[-1])

    def test_cancelled_deferred_delta_retains_busy_status(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        legacy_pending = threading.Event()
        legacy_pending.set()
        first_label = Label()
        second_label = Label()
        namespace = {
            "RUN": {"xboxUse": 1, "offlineMode": False},
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "controller_correction_requested": threading.Event(),
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "MotionTransportBusy": MotionTransportBusy,
            "_current_joint_motion_profile": lambda: object(),
            "deferred_joint_adjustments": SimpleNamespace(pending=True),
            "_defer_joint_target": lambda *args: False,
            "_defer_joint_adjustment": lambda *args: False,
            "legacy_serial_result_pending": legacy_pending,
            "live_serial_result_pending": threading.Event(),
            "serial_lock": SimpleNamespace(locked=lambda: True),
            "joint_motion_dispatcher": SimpleNamespace(active=False),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "logger": SimpleNamespace(error=lambda message: None),
        }
        route = self.compile_function("_queue_joint_motion", namespace)

        self.assertTrue(route(0, -1, absolute=False))
        self.assertEqual(first_label.text, "CONTROLLER MOVE IN PROGRESS")
        self.assertEqual(second_label.text, first_label.text)

    def test_busy_nonlegacy_transport_does_not_create_stranded_intent(self):
        class Dispatcher:
            active = False

            @staticmethod
            def submit_delta(*args):
                raise MotionTransportBusy("busy")

        errors = []
        deferred_calls = []
        namespace = {
            "RUN": {"xboxUse": 1, "offlineMode": False},
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "controller_correction_requested": threading.Event(),
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "MotionTransportBusy": MotionTransportBusy,
            "_current_joint_motion_profile": lambda: object(),
            "_current_joint_positions": lambda: (0.0,) * 9,
            "deferred_joint_adjustments": SimpleNamespace(pending=False),
            "joint_motion_dispatcher": Dispatcher(),
            "_defer_joint_target": lambda *args: deferred_calls.append(args),
            "_defer_joint_adjustment": lambda *args: deferred_calls.append(args),
            "legacy_serial_result_pending": threading.Event(),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "logger": SimpleNamespace(error=errors.append),
        }
        route = self.compile_function("_queue_joint_motion", namespace)

        self.assertFalse(route(0, 1, absolute=False))
        self.assertEqual(deferred_calls, [])
        self.assertIn("outside the legacy motion queue", errors[-1])

    def test_offline_external_axis_is_rejected_with_alarm(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        class Logger:
            def __init__(self):
                self.errors = []

            def error(self, message):
                self.errors.append(message)

        first_label = Label()
        second_label = Label()
        logger = Logger()
        virtual_commands = []
        namespace = {
            "RUN": {"xboxUse": 1, "offlineMode": True, "VR_angles": [0.0] * 6},
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "controller_correction_requested": threading.Event(),
            "math": math,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "MotionTransportBusy": MotionTransportBusy,
            "_current_joint_motion_profile": lambda: object(),
            "build_virtual_joint_command": lambda positions, profile: "virtual",
            "rj_command": virtual_commands.append,
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "logger": logger,
        }
        route = self.compile_function("_queue_joint_motion", namespace)

        accepted = route(6, 1, absolute=False)

        self.assertFalse(accepted)
        self.assertEqual(virtual_commands, [])
        self.assertIn("J7-J9 require a controller", first_label.text)
        self.assertEqual(first_label.text, second_label.text)
        self.assertEqual(logger.errors, [first_label.text])

    def test_offline_virtual_drive_rejection_is_reported(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        errors = []
        virtual_commands = []
        first_label = Label()
        second_label = Label()
        profile = MotionProfile("Sp", 50, 10, 20, 25, "N", "000000")
        runtime = {
            "xboxUse": 1,
            "offlineMode": True,
            "VR_angles": [0.0] * 6,
        }
        namespace = {
            "RUN": runtime,
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "controller_correction_requested": threading.Event(),
            "math": math,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "MotionTransportBusy": MotionTransportBusy,
            "_validated_virtual_six_vector": (
                lambda values, label: tuple(values)
            ),
            "_set_virtual_joint_target": (
                lambda target: runtime.update(VR_angles=list(target))
            ),
            "refresh_gui_from_joint_angles": lambda target: True,
            "_current_joint_motion_profile": lambda: profile,
            "build_virtual_joint_command": build_virtual_joint_command,
            "rj_command": lambda command: virtual_commands.append(command) or False,
            "_complete_program_motion_when_virtual_idle": complete_virtual_callback,
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "logger": SimpleNamespace(error=errors.append),
        }
        self.add_virtual_completion_timeout(namespace)
        namespace["_start_offline_joint_motion"] = self.compile_function(
            "_start_offline_joint_motion",
            namespace,
        )
        route = self.compile_function("_queue_joint_motion", namespace)

        self.assertFalse(route(0, 1, absolute=False))
        self.assertEqual(
            virtual_commands,
            [build_virtual_joint_command((1.0, 0, 0, 0, 0, 0), profile)],
        )
        self.assertIn("offline virtual joint motion did not start", first_label.text)
        self.assertEqual(second_label.text, first_label.text)
        self.assertEqual(errors, [first_label.text])

    def test_offline_joint_terminal_failure_restores_pose_before_release(self):
        jobs = []
        saved_pose = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        partial_pose = (11.0, 12.0, 13.0, 14.0, 15.0, 16.0)
        runtime = {
            "offlineMode": True,
            "VR_angles": list(saved_pose),
        }
        operation = VirtualMotionOperation()
        reconciliations = []
        profile = MotionProfile("Sp", 50, 10, 20, 25, "N", "000000")
        command = build_virtual_joint_command(partial_pose, profile)
        namespace = {
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append(callback)
            ),
            "_validated_virtual_six_vector": (
                lambda values, label: tuple(values)
            ),
            "_set_virtual_joint_target": (
                lambda target: runtime.update(VR_angles=list(target))
            ),
            "rj_command": (
                lambda ignored: runtime.update(VR_angles=list(partial_pose))
                or operation
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        self.add_virtual_completion_timeout(namespace)

        def refresh(target):
            registry = namespace["motion_request_registry"]
            reconciliations.append(
                (registry.active, tuple(target), tuple(runtime["VR_angles"]))
            )
            runtime["VR_angles"] = list(target)
            return True

        namespace["refresh_gui_from_joint_angles"] = refresh
        namespace["_complete_program_motion_when_virtual_idle"] = (
            self.compile_function(
                "_complete_program_motion_when_virtual_idle",
                namespace,
            )
        )
        start = self.compile_function("_start_offline_joint_motion", namespace)

        self.assertTrue(start(command))
        self.assertTrue(namespace["motion_request_registry"].active)
        self.assertEqual(runtime["VR_angles"], list(partial_pose))
        self.assertEqual(len(jobs), 1)

        operation.complete(False, "virtual drive failed after partial update")
        jobs.pop()()

        self.assertEqual(
            reconciliations,
            [(True, saved_pose, partial_pose)],
        )
        self.assertEqual(runtime["VR_angles"], list(saved_pose))
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_joint_result_keeps_pending_virtual_target_then_resynchronizes(self):
        class Dispatcher:
            desired_target = (20.0,) + (0.0,) * 8

        applied = []
        dispatcher = Dispatcher()
        namespace = {
            "joint_motion_dispatcher": dispatcher,
            "_try_set_virtual_joint_target": (
                lambda target: applied.append(target) or True
            ),
        }
        synchronize = self.compile_function("_set_virtual_from_joint_result", namespace)
        position = SimpleNamespace(joints=(10.0,) + (0.0,) * 5)

        self.assertTrue(synchronize(position))
        dispatcher.desired_target = None
        self.assertTrue(synchronize(position))

        self.assertEqual(applied[0], (20.0,) + (0.0,) * 8)
        self.assertEqual(applied[1], position.joints)

    def test_serial_worker_does_not_access_tk_widgets(self):
        worker = self.module_functions["run_send_serial_safe"]
        referenced_names = {
            node.id for node in ast.walk(worker) if isinstance(node, ast.Name)
        }
        forbidden_names = {
            "root",
            "tab1",
            "cmdSentEntryField",
            "cmdRecEntryField",
            "almStatusLab",
            "almStatusLab2",
        }

        self.assertFalse(referenced_names & forbidden_names)

    def test_program_motion_controller_rejection_waits_for_admitted_virtual(self):
        virtual_commands = []
        waits = []
        operation = completed_virtual_operation()

        def start_virtual(command):
            virtual_commands.append(command)
            return operation

        def wait_for_virtual(requested_operation, timeout, deadline=None):
            waits.append((requested_operation, timeout))
            return True

        namespace = {
            "RUN": {"offlineMode": False},
            "_start_legacy_motion": lambda *args, **kwargs: False,
            "_controller_response_timeout": lambda command: 1.0,
            "_complete_program_motion_when_virtual_idle": complete_virtual_callback,
            "_wait_for_virtual_motion_operation": wait_for_virtual,
            "drive_lock": threading.Lock(),
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "threading": threading,
            "logger": SimpleNamespace(error=lambda *args: None),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        self.add_virtual_completion_timeout(namespace)
        dispatch = self.compile_function("_dispatch_program_motion", namespace)
        expected_timeout = namespace["_virtual_completion_timeout"](
            VIRTUAL_CARTESIAN_TEST_COMMAND
        )

        result = dispatch(
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            start_virtual,
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            None,
        )

        self.assertEqual(result, "rejected")
        self.assertEqual(virtual_commands, [VIRTUAL_CARTESIAN_TEST_COMMAND])
        self.assertEqual(waits, [(operation, expected_timeout)])

    def test_program_motion_rejects_physical_virtual_wrist_mismatch(self):
        first_label = SimpleNamespace(text=None)
        second_label = SimpleNamespace(text=None)
        first_label.config = lambda **kwargs: setattr(
            first_label,
            "text",
            kwargs.get("text"),
        )
        second_label.config = lambda **kwargs: setattr(
            second_label,
            "text",
            kwargs.get("text"),
        )
        namespace = {
            "RUN": {"offlineMode": False},
            "_controller_response_timeout": lambda command: self.fail(
                "mismatched wrist commands reached controller timing"
            ),
            "_virtual_completion_timeout": lambda command: self.fail(
                "mismatched wrist commands reached virtual timing"
            ),
            "logger": SimpleNamespace(error=lambda *args: None),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "ROW_EXECUTION_REJECTED": "rejected",
        }
        dispatch = self.compile_function("_dispatch_program_motion", namespace)

        result = dispatch(
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            lambda command: self.fail("mismatched wrist commands were dispatched"),
            VIRTUAL_CARTESIAN_TEST_COMMAND.replace("WN", "WF"),
            None,
        )

        self.assertEqual(result, "rejected")
        self.assertIn("same wrist configuration", first_label.text)
        self.assertEqual(second_label.text, first_label.text)

    def test_program_motion_worker_start_exception_settles_virtual_owner(self):
        def run_dispatch(with_callback):
            waits = []
            completions = []
            first_label = SimpleNamespace(text=None)
            second_label = SimpleNamespace(text=None)
            first_label.config = lambda **kwargs: setattr(
                first_label,
                "text",
                kwargs.get("text"),
            )
            second_label.config = lambda **kwargs: setattr(
                second_label,
                "text",
                kwargs.get("text"),
            )
            operation = completed_virtual_operation()
            namespace = {
                "RUN": {"offlineMode": False},
                "_start_legacy_motion": (
                    lambda *args, **kwargs: (_ for _ in ()).throw(
                        RuntimeError("thread start failed")
                    )
                ),
                "_controller_response_timeout": lambda command: 1.0,
                "_virtual_completion_timeout": lambda command: 1.0,
                "_complete_program_motion_when_virtual_idle": (
                    complete_virtual_callback
                ),
                "_wait_for_virtual_motion_operation": (
                    lambda requested, timeout, deadline=None: (
                        waits.append(requested) or True
                    )
                ),
                "drive_lock": threading.Lock(),
                "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
                "threading": threading,
                "logger": SimpleNamespace(
                    error=lambda *args: None,
                    exception=lambda *args: None,
                ),
                "almStatusLab": first_label,
                "almStatusLab2": second_label,
                "ROW_EXECUTION_REJECTED": "rejected",
                "ROW_EXECUTION_PENDING": "pending",
                "ROW_EXECUTION_COMPLETE": "complete",
            }
            dispatch = self.compile_function(
                "_dispatch_program_motion",
                namespace,
            )
            result = dispatch(
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                lambda command: operation,
                VIRTUAL_CARTESIAN_TEST_COMMAND,
                completions.append if with_callback else None,
            )
            return result, waits, completions, first_label.text, second_label.text

        callback_result = run_dispatch(True)
        self.assertEqual(callback_result[:3], ("pending", [], [False]))
        self.assertIn("worker failed during startup", callback_result[3])
        self.assertEqual(callback_result[4], callback_result[3])

        blocking_result = run_dispatch(False)
        self.assertEqual(blocking_result[0], "rejected")
        self.assertEqual(len(blocking_result[1]), 1)
        self.assertEqual(blocking_result[2], [])
        self.assertIn("worker failed during startup", blocking_result[3])
        self.assertEqual(blocking_result[4], blocking_result[3])

    def test_program_motion_callback_path_never_waits_on_tk(self):
        virtual_commands = []
        waits = []
        controller_callbacks = []
        completion_results = []

        def start_motion(
            command,
            motion_name,
            completion_callback=None,
            request_lease=None,
            write_started_event=None,
        ):
            controller_callbacks.append(completion_callback)
            return True

        def start_virtual(command):
            virtual_commands.append(command)
            return completed_virtual_operation()

        namespace = {
            "RUN": {"offlineMode": False},
            "_start_legacy_motion": start_motion,
            "_controller_response_timeout": lambda command: 1.0,
            "_virtual_completion_timeout": lambda command: 1.0,
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "_complete_program_motion_when_virtual_idle": complete_virtual_callback,
            "_wait_for_virtual_motion_operation": (
                lambda operation, timeout, **kwargs: True
            ),
            "drive_lock": threading.Lock(),
            "threading": threading,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        dispatch = self.compile_function("_dispatch_program_motion", namespace)

        result = dispatch(
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            start_virtual,
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            completion_results.append,
        )

        self.assertEqual(result, "pending")
        self.assertEqual(virtual_commands, [VIRTUAL_CARTESIAN_TEST_COMMAND])
        self.assertEqual(waits, [])
        self.assertEqual(len(controller_callbacks), 1)
        controller_callbacks[0](VALID_CONTROLLER_POSITION)
        self.assertEqual(completion_results, [True])

    def test_controller_first_program_result_retains_virtual_owner(self):
        order = []
        jobs = []
        controller_callbacks = []
        completion_results = []
        operation = VirtualMotionOperation()

        def start_virtual(command):
            order.append("virtual")
            return operation

        namespace = {
            "RUN": {"offlineMode": False},
            "_controller_response_timeout": lambda command: 1.0,
            "_virtual_completion_timeout": lambda command: 1.0,
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append(callback)
            ),
            "threading": threading,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }

        def start_motion(
            command,
            motion_name,
            completion_callback=None,
            request_lease=None,
            write_started_event=None,
        ):
            self.assertTrue(
                namespace["motion_request_registry"].owns(request_lease)
            )
            order.append("physical")
            controller_callbacks.append(completion_callback)
            return True

        namespace["_start_legacy_motion"] = start_motion
        namespace["_complete_program_motion_when_virtual_idle"] = (
            self.compile_function(
                "_complete_program_motion_when_virtual_idle",
                namespace,
            )
        )
        dispatch = self.compile_function("_dispatch_program_motion", namespace)

        result = dispatch(
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            start_virtual,
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            completion_results.append,
        )

        self.assertEqual(result, "pending")
        self.assertEqual(order, ["virtual", "physical"])
        self.assertTrue(namespace["motion_request_registry"].active)
        self.assertIsNone(
            namespace["motion_request_registry"].acquire("unrelated motion")
        )

        controller_callbacks[0](VALID_CONTROLLER_POSITION)
        self.assertTrue(namespace["motion_request_registry"].active)
        self.assertEqual(completion_results, [])
        self.assertEqual(len(jobs), 1)

        operation.complete(True)
        jobs.pop()()

        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertEqual(completion_results, [True])

    def test_online_program_callback_rejects_failed_virtual_admission(self):
        controller_callbacks = []
        completion_results = []

        def start_motion(command, motion_name, completion_callback=None):
            controller_callbacks.append(completion_callback)
            return True

        namespace = {
            "RUN": {"offlineMode": False},
            "_start_legacy_motion": start_motion,
            "_controller_response_timeout": lambda command: 1.0,
            "_virtual_completion_timeout": lambda command: 1.0,
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "_complete_program_motion_when_virtual_idle": complete_virtual_callback,
            "_wait_for_virtual_motion_operation": (
                lambda operation, timeout, **kwargs: True
            ),
            "drive_lock": threading.Lock(),
            "threading": threading,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        dispatch = self.compile_function("_dispatch_program_motion", namespace)

        result = dispatch(
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            lambda command: False,
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            completion_results.append,
        )

        self.assertEqual(result, "rejected")
        self.assertEqual(controller_callbacks, [])
        self.assertEqual(completion_results, [])

    def test_offline_program_motion_callback_defers_without_waiting(self):
        callbacks = []
        waits = []
        namespace = {
            "RUN": {"offlineMode": True},
            "_start_legacy_motion": lambda *args, **kwargs: self.fail(
                "offline motion must not reserve the physical transport"
            ),
            "_controller_response_timeout": lambda command: 1.0,
            "_virtual_completion_timeout": lambda command: 1.0,
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "_complete_program_motion_when_virtual_idle": complete_virtual_callback,
            "_wait_for_virtual_motion_operation": (
                lambda operation, timeout, **kwargs: True
            ),
            "drive_lock": threading.Lock(),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "threading": threading,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        dispatch = self.compile_function("_dispatch_program_motion", namespace)

        result = dispatch(
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            lambda command: completed_virtual_operation(),
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            callbacks.append,
        )

        self.assertEqual(result, "pending")
        self.assertEqual(callbacks, [True])
        self.assertEqual(waits, [])

    def test_offline_program_motion_rejects_failed_virtual_start(self):
        callbacks = []
        waits = []
        namespace = {
            "RUN": {"offlineMode": True},
            "_start_legacy_motion": lambda *args, **kwargs: self.fail(
                "offline motion must not reserve the physical transport"
            ),
            "_controller_response_timeout": lambda command: 1.0,
            "_virtual_completion_timeout": lambda command: 1.0,
            "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.1,
            "_complete_program_motion_when_virtual_idle": complete_virtual_callback,
            "_wait_for_virtual_motion_operation": (
                lambda operation, timeout, **kwargs: True
            ),
            "threading": threading,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        dispatch = self.compile_function("_dispatch_program_motion", namespace)

        result = dispatch(
            CONTROLLER_CARTESIAN_TEST_COMMAND,
            lambda command: False,
            VIRTUAL_CARTESIAN_TEST_COMMAND,
            callbacks.append,
        )

        self.assertEqual(result, "rejected")
        self.assertEqual(callbacks, [])
        self.assertEqual(waits, [])

    def test_program_motion_requires_controller_and_virtual_completion(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        def run_dispatch(controller_succeeded, virtual_finished=True):
            pending_completion = []

            class DeterministicControllerEvent:
                def __init__(self):
                    self.completed = False

                def set(self):
                    self.completed = True

                def wait(self, timeout=None):
                    if controller_succeeded == "late" and timeout is not None:
                        return False
                    if controller_succeeded == "late" and not self.completed:
                        pending_completion.pop()(VALID_CONTROLLER_POSITION)
                    return self.completed

            def start_motion(
                command,
                motion_name,
                completion_callback=None,
                request_lease=None,
                write_started_event=None,
            ):
                if controller_succeeded == "late":
                    pending_completion.append(completion_callback)
                elif controller_succeeded is not None:
                    completion_callback(
                        VALID_CONTROLLER_POSITION
                        if controller_succeeded
                        else None
                    )
                return True

            first_label = Label()
            second_label = Label()
            namespace = {
                "RUN": {"offlineMode": False},
                "_start_legacy_motion": start_motion,
                "_controller_response_timeout": lambda command: 0.01,
                "_virtual_completion_timeout": lambda command: 0.02,
                "_complete_program_motion_when_virtual_idle": complete_virtual_callback,
                "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.01,
                "drive_lock": threading.Lock(),
                "threading": SimpleNamespace(Event=DeterministicControllerEvent),
                "logger": SimpleNamespace(
                    error=lambda *args: None,
                    exception=lambda *args: None,
                ),
                "almStatusLab": first_label,
                "almStatusLab2": second_label,
                "ROW_EXECUTION_REJECTED": "rejected",
                "ROW_EXECUTION_PENDING": "pending",
                "ROW_EXECUTION_COMPLETE": "complete",
                "_wait_for_virtual_motion_operation": (
                    lambda operation, timeout, **kwargs: virtual_finished
                ),
            }
            dispatch = self.compile_function(
                "_dispatch_program_motion",
                namespace,
            )
            result = dispatch(
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                lambda command: completed_virtual_operation(),
                VIRTUAL_CARTESIAN_TEST_COMMAND,
                None,
            )
            return result, first_label.text, second_label.text

        self.assertEqual(run_dispatch(True), ("complete", None, None))
        failed_result = run_dispatch(False)
        self.assertEqual(failed_result[0], "rejected")
        self.assertIn("controller completion failed", failed_result[1])
        self.assertEqual(failed_result[2], failed_result[1])
        missing_result = run_dispatch("late")
        self.assertEqual(missing_result[0], "rejected")
        self.assertIn("controller completion failed", missing_result[1])
        timed_out_result = run_dispatch(True, virtual_finished=False)
        self.assertEqual(timed_out_result[0], "rejected")
        self.assertIn("virtual preview failed or timed out", timed_out_result[1])

    def test_program_pose_reconciliation_covers_failure_matrix(self):
        snapshot_namespace = {
            "dataclass": dataclass,
            "Optional": Optional,
        }
        snapshot_type = self.compile_class(
            "ProgramMotionPoseSnapshot",
            snapshot_namespace,
        )
        saved_controller = tuple(float(value) for value in range(11, 20))
        saved_virtual = saved_controller[:6]

        def run_case(
            controller_position,
            *,
            write_started=False,
            motion_succeeded=False,
            offline=False,
            reject_virtual=False,
        ):
            synchronized = []
            virtual_targets = []
            refreshed_targets = []
            invalidations = []
            resynchronization_required = threading.Event()
            write_boundary = None if offline else threading.Event()
            if write_started:
                write_boundary.set()

            class Dispatcher:
                @staticmethod
                def synchronize(positions):
                    synchronized.append(tuple(positions))
                    return True

            def invalidate(reason):
                invalidations.append(reason)
                resynchronization_required.set()

            def set_virtual(target):
                virtual_targets.append(tuple(target))
                return not reject_virtual

            runtime = {"VR_angles": list(saved_virtual)}

            def refresh(target):
                refreshed_targets.append(tuple(target))
                runtime["VR_angles"] = list(target)
                return True

            namespace = {
                "ProgramMotionPoseSnapshot": snapshot_type,
                "PositionResponse": PositionResponse,
                "MotionInputError": MotionInputError,
                "MotionQueueFault": MotionQueueFault,
                "finite_number": finite_number,
                "RUN": runtime,
                "joint_motion_dispatcher": Dispatcher(),
                "_try_set_virtual_joint_target": set_virtual,
                "refresh_gui_from_joint_angles": refresh,
                "_invalidate_joint_motion_state": invalidate,
                "controller_position_resynchronization_required": (
                    resynchronization_required
                ),
                "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
                "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
                "logger": SimpleNamespace(
                    error=lambda *args: None,
                    exception=lambda *args: None,
                ),
            }
            namespace["_apply_program_motion_pose"] = self.compile_function(
                "_apply_program_motion_pose",
                namespace,
            )
            reconcile = self.compile_function(
                "_reconcile_program_motion_pose",
                namespace,
            )
            snapshot = snapshot_type(
                None if offline else saved_controller,
                saved_virtual,
            )
            result = reconcile(
                snapshot,
                controller_position,
                write_boundary,
                motion_succeeded,
            )
            return SimpleNamespace(
                result=result,
                synchronized=synchronized,
                virtual_targets=virtual_targets,
                refreshed_targets=refreshed_targets,
                invalidations=invalidations,
                resynchronization_required=resynchronization_required.is_set(),
            )

        converged_success = run_case(
            VALID_CONTROLLER_POSITION,
            write_started=True,
            motion_succeeded=True,
        )
        self.assertTrue(converged_success.result)
        self.assertEqual(
            converged_success.synchronized,
            [
                VALID_CONTROLLER_POSITION.joints
                + VALID_CONTROLLER_POSITION.external
            ],
        )
        self.assertEqual(
            converged_success.virtual_targets,
            [VALID_CONTROLLER_POSITION.joints],
        )
        self.assertEqual(converged_success.invalidations, [])
        self.assertFalse(converged_success.resynchronization_required)

        controller_success_virtual_failure = run_case(
            VALID_CONTROLLER_POSITION,
            write_started=True,
            motion_succeeded=False,
        )
        self.assertFalse(controller_success_virtual_failure.result)
        self.assertEqual(
            controller_success_virtual_failure.synchronized,
            [
                VALID_CONTROLLER_POSITION.joints
                + VALID_CONTROLLER_POSITION.external
            ],
        )
        self.assertEqual(
            controller_success_virtual_failure.virtual_targets,
            [VALID_CONTROLLER_POSITION.joints],
        )
        self.assertEqual(controller_success_virtual_failure.invalidations, [])
        self.assertFalse(
            controller_success_virtual_failure.resynchronization_required
        )

        prewrite_rejection = run_case(None, motion_succeeded=False)
        self.assertFalse(prewrite_rejection.result)
        self.assertEqual(prewrite_rejection.synchronized, [saved_controller])
        self.assertEqual(prewrite_rejection.virtual_targets, [saved_virtual])
        self.assertEqual(prewrite_rejection.invalidations, [])
        self.assertFalse(prewrite_rejection.resynchronization_required)

        postwrite_failure = run_case(
            None,
            write_started=True,
            motion_succeeded=False,
        )
        self.assertFalse(postwrite_failure.result)
        self.assertEqual(postwrite_failure.synchronized, [])
        self.assertEqual(postwrite_failure.virtual_targets, [])
        self.assertEqual(len(postwrite_failure.invalidations), 1)
        self.assertTrue(postwrite_failure.resynchronization_required)

        convergence_failure = run_case(
            VALID_CONTROLLER_POSITION,
            write_started=True,
            motion_succeeded=True,
            reject_virtual=True,
        )
        self.assertFalse(convergence_failure.result)
        self.assertEqual(len(convergence_failure.invalidations), 1)
        self.assertTrue(convergence_failure.resynchronization_required)

        offline_failure = run_case(
            None,
            motion_succeeded=False,
            offline=True,
        )
        self.assertFalse(offline_failure.result)
        self.assertEqual(offline_failure.synchronized, [])
        self.assertEqual(offline_failure.virtual_targets, [])
        self.assertEqual(offline_failure.refreshed_targets, [saved_virtual])
        self.assertEqual(offline_failure.invalidations, [])

    def test_program_pose_settlement_precedes_motion_lease_release(self):
        order = []
        namespace = {
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        finish = self.compile_function(
            "_finish_settled_motion_request",
            namespace,
        )
        registry = namespace["motion_request_registry"]
        request_lease = registry.acquire("Program motion")

        def settle(succeeded):
            self.assertTrue(registry.owns(request_lease))
            order.append("pose-reconciled")
            return succeeded

        def complete(succeeded):
            self.assertFalse(registry.active)
            order.append(("row-completed", succeeded))

        finish(complete, request_lease, True, settle)

        self.assertEqual(
            order,
            ["pose-reconciled", ("row-completed", True)],
        )

    def test_virtual_motion_active_covers_every_owner_lock(self):
        locks = {
            "offline_live_jog_lock": threading.Lock(),
            "live_jog_lock": threading.Lock(),
            "live_cartesian_lock": threading.Lock(),
            "live_tool_lock": threading.Lock(),
            "drive_lock": threading.Lock(),
        }
        namespace = {
            **locks,
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_operation": None,
        }
        namespace["_virtual_motion_active"] = self.compile_function(
            "_virtual_motion_active",
            namespace,
        )

        self.assertFalse(namespace["_virtual_motion_active"]())
        for name, lock in locks.items():
            with self.subTest(lock=name):
                lock.acquire()
                try:
                    self.assertTrue(namespace["_virtual_motion_active"]())
                finally:
                    lock.release()

        namespace["offline_live_jog_operation"] = object()
        self.assertTrue(namespace["_virtual_motion_active"]())

    def test_virtual_completion_timeout_scales_long_seconds_motion(self):
        namespace = {}
        self.add_virtual_completion_timeout(namespace)
        commands = (
            "RJA1B2C3D4E5F6Ss30Ac10Dc20Rm25WNLm000000\n",
            (
                "MJX1Y2Z3Rz4Ry5Rx6"
                "Ss30Ac10Dc20Rm25WNLm000000\n"
            ),
            "JTX11Ss30G10H20I25WNLm000000\n",
        )

        for command in commands:
            with self.subTest(command=command[:2]):
                self.assertGreater(
                    namespace["_virtual_completion_timeout"](command),
                    120,
                )

    def test_virtual_cartesian_execution_accepts_supported_speed_modes(self):
        started = []
        forward_inputs = []
        marker = object()
        runtime = {
            "offlineMode": False,
            "xyzuvw_In": np.zeros(6),
            "VR_angles": [0.0] * 6,
            "JangleOut": None,
            "J1axisLimNeg": 0.0,
            "J2axisLimNeg": 0.0,
            "J3axisLimNeg": 0.0,
            "J4axisLimNeg": 0.0,
            "J5axisLimNeg": 0.0,
            "J6axisLimNeg": 0.0,
            "J1StepM": 0,
            "J2StepM": 0,
            "J3StepM": 0,
            "J4StepM": 0,
            "J5StepM": 0,
            "J6StepM": 0,
        }
        namespace = {
            "RUN": runtime,
            "CAL": {
                **{f"J{axis}StepDeg": 1.0 for axis in range(1, 7)},
                **{f"J{axis}PosLim": 100.0 for axis in range(1, 7)},
                **{f"J{axis}NegLim": 100.0 for axis in range(1, 7)},
            },
            "J1StepLim": 100,
            "J2StepLim": 100,
            "J3StepLim": 100,
            "J4StepLim": 100,
            "J5StepLim": 100,
            "J6StepLim": 100,
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
            "canonicalize_virtual_command": canonicalize_virtual_command,
            "math": math,
            "re": re,
            "np": np,
            "robot": SimpleNamespace(
                SolveInverseKinematicsConfigured=(
                    lambda target, seed, wrist: [1, 1, 1, 1, 0, 1]
                ),
                forward_kinematics=lambda seed: (
                    forward_inputs.append(seed)
                    or (
                        [
                            10.0,
                            20.0,
                            30.0,
                            math.radians(4.0),
                            math.radians(5.0),
                            math.radians(6.0),
                        ]
                        if tuple(seed) == (0.0,) * 6
                        else [
                            1.0,
                            2.0,
                            3.0,
                            math.radians(6.0),
                            math.radians(5.0),
                            math.radians(4.0),
                        ]
                    )
                ),
            ),
            "start_driveMotorsJ_thread": (
                lambda *args: started.append(args) or marker
            ),
            "ErrorHandler": lambda response: self.fail(
                f"unexpected virtual error: {response}"
            ),
            "logger": SimpleNamespace(
                info=lambda *args: None,
                error=lambda *args: None,
            ),
        }
        namespace["_validated_virtual_six_vector"] = self.compile_function(
            "_validated_virtual_six_vector",
            namespace,
        )
        namespace["parse_mj_command"] = self.compile_function(
            "parse_mj_command",
            namespace,
        )
        move_cartesian = self.compile_function("mj_command", namespace)

        for mode, speed in (("p", "50"), ("s", "2"), ("m", "25")):
            with self.subTest(mode=mode):
                command = (
                    "MJX1Y2Z3Rz4Ry5Rx6"
                    f"S{mode}{speed}Ac10Dc20Rm25WNLm000000\n"
                )
                self.assertIs(move_cartesian(command), marker)
                self.assertEqual(started[-1][12], mode)
                if mode == "m":
                    self.assertEqual(
                        started[-1][17][:3],
                        (10.0, 20.0, 30.0),
                    )
                    for actual, expected in zip(
                        started[-1][17][3:],
                        (4.0, 5.0, 6.0),
                    ):
                        self.assertAlmostEqual(actual, expected)
                    self.assertEqual(
                        started[-1][18],
                        (1.0, 2.0, 3.0, 6.0, 5.0, 4.0),
                    )

        runtime["VR_angles"] = [0.0] * 5
        started_count = len(started)
        self.assertFalse(
            move_cartesian(
                "MJX1Y2Z3Rz4Ry5Rx6Sm25Ac10Dc20Rm25WNLm000000\n"
            )
        )
        self.assertEqual(len(started), started_count)
        self.assertEqual(len(forward_inputs), 4)

    def test_virtual_cartesian_mm_per_second_drive_uses_real_numpy_endpoints(self):
        sleeps = []
        runtime = {
            "J1StepM": 0,
            "J2StepM": 0,
            "J3StepM": 0,
            "J4StepM": 0,
            "J5StepM": 0,
            "J6StepM": 0,
            "StepMonitors": [0] * 6,
            "VR_angles": [0.0] * 6,
            "stepDeg": [1.0] * 6,
            "negLim": [0.0] * 6,
            "minSpeedDelay": 200,
            "speedViolation": "0",
            "offlineMode": False,
            "liveJog": False,
        }
        namespace = {
            "RUN": runtime,
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
            "math": math,
            "time": SimpleNamespace(sleep=sleeps.append),
            "robot": SimpleNamespace(get_joint_limits=lambda: ((-180,) * 6, (180,) * 6)),
            "live_cartesian_lock": threading.Lock(),
            "live_tool_lock": threading.Lock(),
            "VIRTUAL_CARTESIAN_SECONDS_SCALE": 1.0,
            "VIRTUAL_TOOL_SECONDS_SCALE": 1.0,
            "VIRTUAL_JOINT_SECONDS_SCALE": 1.0,
            "refresh_gui_from_joint_angles": lambda angles: self.fail(
                "online drive must not refresh Tk widgets"
            ),
        }
        namespace["_validated_virtual_six_vector"] = self.compile_function(
            "_validated_virtual_six_vector",
            namespace,
        )
        drive = self.compile_function("driveMotorsJ", namespace)
        args = (
            1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 1,
            "m", 25.0, 0.0, 0.0, 10.0,
        )

        drive(
            *args,
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        )

        self.assertGreater(sleeps[-1], 3.9)
        self.assertEqual(runtime["J1StepM"], 1)
        sleeps.clear()
        with self.assertRaisesRegex(MotionInputError, "timing start"):
            drive(
                *args,
                None,
                np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
        self.assertEqual(sleeps, [])

    def test_asynchronous_completion_waits_for_requested_virtual_result(self):
        jobs = []
        results = []
        operation = VirtualMotionOperation()
        namespace = {
            "time": time,
            "drive_lock": threading.Lock(),
            "VIRTUAL_COMPLETION_POLL_MS": 10,
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append((delay, callback))
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        complete = self.compile_function(
            "_complete_program_motion_when_virtual_idle",
            namespace,
        )
        namespace["_complete_program_motion_when_virtual_idle"] = complete
        request_lease = namespace["motion_request_registry"].acquire("test")

        complete(results.append, request_lease, operation, 1.0)
        self.assertEqual(results, [])
        self.assertEqual(len(jobs), 1)

        operation.complete(False, "drive failed")
        jobs.pop()[1]()
        self.assertEqual(results, [False])

    def test_asynchronous_timeout_uses_only_matching_virtual_settlement(self):
        jobs = []
        results = []
        operation = VirtualMotionOperation()
        drive_lock = threading.Lock()
        clock = [100.0]
        namespace = {
            "time": SimpleNamespace(monotonic=lambda: clock[0]),
            "drive_lock": drive_lock,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append((delay, callback))
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        complete = self.compile_function(
            "_complete_program_motion_when_virtual_idle",
            namespace,
        )
        namespace["_complete_program_motion_when_virtual_idle"] = complete
        request_lease = namespace["motion_request_registry"].acquire("test")

        complete(results.append, request_lease, operation, 0.005)
        clock[0] += 0.01
        jobs.pop()[1]()
        self.assertEqual(results, [])

        drive_lock.acquire()
        operation.complete(True)
        jobs.pop()[1]()
        self.assertEqual(results, [False])
        self.assertFalse(namespace["motion_request_registry"].active)

        drive_lock.release()

    def test_scheduler_fallback_preserves_original_virtual_deadline(self):
        results = []
        event_queue = Queue()
        operation = VirtualMotionOperation()
        namespace = {
            "time": time,
            "threading": threading,
            "drive_lock": threading.Lock(),
            "CONTROL_POLL_INTERVAL_SECONDS": 0.001,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "virtual_motion_event_queue": event_queue,
            "root": SimpleNamespace(
                after=lambda *args: (_ for _ in ()).throw(
                    RuntimeError("scheduler unavailable")
                )
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        for function_name in (
            "_program_completion_fallback_result",
            "_run_program_completion_fallback",
            "_start_program_completion_fallback",
            "_complete_program_motion_when_virtual_idle",
        ):
            namespace[function_name] = self.compile_function(
                function_name,
                namespace,
            )

        deadline = time.monotonic() + 0.01
        threading.Timer(0.03, lambda: operation.complete(True)).start()
        request_lease = namespace["motion_request_registry"].acquire("test")
        namespace["_complete_program_motion_when_virtual_idle"](
            results.append,
            request_lease,
            operation,
            1.0,
            deadline=deadline,
        )

        event_type, payload = event_queue.get(timeout=1)
        callback, queued_lease, succeeded, settlement_callback = payload
        self.assertEqual(event_type, "program-completion")
        self.assertTrue(operation.completed)
        self.assertFalse(succeeded)
        namespace["_finish_settled_motion_request"](
            callback,
            queued_lease,
            succeeded,
            settlement_callback,
        )
        self.assertEqual(results, [False])

    def test_fallback_thread_start_failure_retains_completion_inline(self):
        operation = VirtualMotionOperation()
        drive_lock = threading.Lock()
        drive_lock.acquire()
        results = []

        def fail_thread(*args, **kwargs):
            raise RuntimeError("thread startup failed")

        namespace = {
            "time": time,
            "threading": SimpleNamespace(Thread=fail_thread),
            "drive_lock": drive_lock,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.001,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "virtual_motion_event_queue": Queue(),
            "root": SimpleNamespace(
                after=lambda *args: (_ for _ in ()).throw(
                    RuntimeError("scheduler unavailable")
                )
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        for function_name in (
            "_program_completion_fallback_result",
            "_run_program_completion_fallback",
            "_start_program_completion_fallback",
            "_complete_program_motion_when_virtual_idle",
        ):
            namespace[function_name] = self.compile_function(
                function_name,
                namespace,
            )

        def settle_operation():
            drive_lock.release()
            operation.complete(True)

        threading.Timer(0.02, settle_operation).start()
        request_lease = namespace["motion_request_registry"].acquire("test")
        namespace["_complete_program_motion_when_virtual_idle"](
            results.append,
            request_lease,
            operation,
            1.0,
            deadline=time.monotonic() + 0.2,
        )

        self.assertTrue(operation.completed)
        self.assertFalse(drive_lock.locked())
        self.assertEqual(results, [True])

    def test_synchronous_completion_uses_requested_virtual_result(self):
        errors = []
        namespace = {
            "time": time,
            "drive_lock": threading.Lock(),
            "CONTROL_POLL_INTERVAL_SECONDS": 0.001,
            "logger": SimpleNamespace(error=lambda *args: errors.append(args)),
        }
        wait = self.compile_function(
            "_wait_for_virtual_motion_operation",
            namespace,
        )

        self.assertTrue(wait(completed_virtual_operation(), 0.1))
        self.assertFalse(
            wait(completed_virtual_operation(False, "drive failed"), 0.1)
        )
        delayed = VirtualMotionOperation()
        threading.Timer(0.03, lambda: delayed.complete(True)).start()
        self.assertFalse(wait(delayed, 0.01))
        self.assertEqual(len(errors), 2)

    def test_gcode_playback_uses_the_transferable_serial_worker(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        starts = []
        completion_results = []
        label = Label()

        def start_serial(command, **kwargs):
            starts.append((command, kwargs))
            return True

        namespace = {
            "MotionInputError": MotionInputError,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "GCalmStatusLab": label,
            "RUN": {"estopActive": False, "offlineMode": False},
            "start_send_serial_thread": start_serial,
            "MAX_COMMAND_LENGTH": 4096,
        }
        namespace["_gcode_playback_command"] = self.compile_function(
            "_gcode_playback_command",
            namespace,
        )
        play = self.compile_function("GCplayProg", namespace)

        self.assertTrue(play("demo", completion_results.append))
        self.assertEqual(starts[0][0], "PGFndemo.txt\n")
        self.assertEqual(label.text, "GCODE FILE RUNNING")
        starts[0][1]["completion_callback"](VALID_CONTROLLER_POSITION)
        self.assertEqual(label.text, "GCODE FILE COMPLETE")
        self.assertEqual(completion_results, [True])

        namespace["RUN"]["offlineMode"] = True
        self.assertFalse(play("offline-demo", completion_results.append))
        self.assertEqual(len(starts), 1)
        self.assertEqual(label.text, "G-code playback is unavailable while offline")

        function = self.module_functions["GCplayProg"]
        referenced_names = {
            node.id for node in ast.walk(function) if isinstance(node, ast.Name)
        }
        self.assertNotIn("Thread", referenced_names)
        self.assertEqual(function.decorator_list, [])
        self.assertIn("start_send_serial_thread", referenced_names)

        build_command = namespace["_gcode_playback_command"]
        for filename in (
            "",
            ".",
            "..",
            "folder/demo",
            "folder\\demo",
            "C:demo",
            'bad"name',
            "bad*name",
            "bad<name",
            "bad>name",
            "bad?name",
            "bad|name",
            "bad\tname",
            "é",
        ):
            with self.subTest(filename=filename):
                with self.assertRaises(MotionInputError):
                    build_command(filename)

    def test_gcode_storage_and_feed_boundaries_enforce_controller_units(self):
        namespace = {
            "MotionInputError": MotionInputError,
            "MAX_COMMAND_LENGTH": 4096,
            "finite_number": finite_number,
            "controller_protocol_decimal": controller_protocol_decimal,
        }
        storage_filename = self.compile_function(
            "_gcode_storage_filename",
            namespace,
        )
        convert_feed = self.compile_function(
            "_gcode_feed_rate_mm_per_second",
            namespace,
        )

        self.assertEqual(storage_filename(" demo "), "demo.txt")
        maximum_base = "a" * (MAX_CONTROLLER_FILENAME_BYTES - len(".txt"))
        self.assertEqual(
            storage_filename(maximum_base),
            f"{maximum_base}.txt",
        )
        with self.assertRaises(MotionInputError):
            storage_filename(f"{maximum_base}a")
        for invalid in (
            "",
            ".",
            "..",
            "folder/demo",
            "C:demo",
            'bad"name',
            "bad*name",
            "bad<name",
            "bad>name",
            "bad?name",
            "bad|name",
            "bad\\name",
            "bad\x7fname",
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises(MotionInputError):
                    storage_filename(invalid)
        self.assertEqual(float(convert_feed(60, False)), 1.0)
        self.assertAlmostEqual(float(convert_feed(60, True)), 25.4, places=5)
        with self.assertRaises(MotionInputError):
            convert_feed(0, False)
        with self.assertRaises(MotionInputError):
            convert_feed(60, "metric")

        for function_name in ("GCdelete", "GCconvertProg", "GCexecuteRow"):
            calls = [
                node
                for node in ast.walk(self.module_functions[function_name])
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_gcode_storage_filename"
            ]
            self.assertGreater(len(calls), 0, function_name)

        execute_names = {
            node.id
            for node in ast.walk(self.module_functions["GCexecuteRow"])
            if isinstance(node, ast.Name)
        }
        self.assertFalse({"xVal", "yVal", "zVal"} & execute_names)

    def test_gcode_start_position_uses_manual_owner_and_stays_virtual_offline(self):
        class Entry:
            def __init__(self, value="0"):
                self.value = value
                self.writes = []

            def get(self):
                return self.value

            def delete(self, *args):
                self.writes.append(("delete", args))

            def insert(self, *args):
                self.writes.append(("insert", args))

        captured = []
        sent_entry = Entry()
        mj_dispatch = lambda command: completed_virtual_operation()
        namespace = {
            "wraps": wraps,
            "RUN": {"offlineMode": True, "WC": "N"},
            "CAL": {
                "J7PosCur": 7,
                "J8PosCur": 8,
                "J9PosCur": 9,
                **{
                    f"J{axis}OpenLoopVal": SimpleNamespace(get=lambda: 0)
                    for axis in range(1, 7)
                },
            },
            "GC_ST_E1_EntryField": Entry("1"),
            "GC_ST_E2_EntryField": Entry("2"),
            "GC_ST_E3_EntryField": Entry("3"),
            "GC_ST_E4_EntryField": Entry("4"),
            "GC_ST_E5_EntryField": Entry("5"),
            "GC_ST_E6_EntryField": Entry("6"),
            "GC_SToff_E1_EntryField": Entry("0.1"),
            "GC_SToff_E2_EntryField": Entry("0.2"),
            "GC_SToff_E3_EntryField": Entry("0.3"),
            "GC_SToff_E4_EntryField": Entry("0.4"),
            "GC_SToff_E5_EntryField": Entry("0.5"),
            "GC_SToff_E6_EntryField": Entry("0.6"),
            "GC_ST_WC_EntryField": Entry("N"),
            "cmdSentEntryField": sent_entry,
            "mj_command": mj_dispatch,
            "_start_manual_motion": lambda *args: captured.append(args) or True,
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "logger": SimpleNamespace(warning=lambda *args: None),
        }
        namespace["_manual_motion_request"] = self.compile_function(
            "_manual_motion_request",
            namespace,
        )
        move_to_start = self.compile_function(
            "MoveGcodeStartPos",
            namespace,
            preserve_decorators=True,
        )

        self.assertTrue(move_to_start())
        self.assertEqual(len(captured), 1)
        self.assertIsNone(captured[0][0])
        self.assertEqual(captured[0][1], "G-code start-position motion")
        self.assertIs(captured[0][2], mj_dispatch)
        self.assertTrue(captured[0][3].startswith("MJX1.1Y2.2Z3.3"))
        self.assertNotIn("J7", captured[0][3])
        self.assertEqual(sent_entry.writes, [])
        self.assertFalse(namespace["motion_request_registry"].active)

        function = self.module_functions["MoveGcodeStartPos"]
        attributes = {
            node.attr
            for node in ast.walk(function)
            if isinstance(node, ast.Attribute)
        }
        self.assertFalse(attributes & {"write", "read", "readline", "flushInput"})
        start_calls = [
            node
            for node in ast.walk(function)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_start_manual_motion"
        ]
        self.assertEqual(len(start_calls), 1)

    def test_gcode_playback_uses_cancellation_bound_response_ownership(self):
        serial_port = SimpleNamespace(is_open=True)
        cancellation = threading.Event()
        exchanges = []
        validations = []
        namespace = {
            "RUN": {"ser": serial_port},
            "MotionInputError": MotionInputError,
            "application_closing": cancellation,
            "serial_write_lock": threading.Lock(),
            "parse_command_timing": validations.append,
            "exchange_serial_line_until_cancelled": (
                lambda port, command, event, **kwargs: exchanges.append(
                    (port, command, event, kwargs)
                ) or "position"
            ),
            "_controller_response_timeout": lambda command: self.fail(
                "PG must not receive a fixed response timeout"
            ),
            "exchange_serial_line": lambda *args, **kwargs: self.fail(
                "PG must not use the fixed-deadline exchange"
            ),
            "serial_transport_quarantined": lambda port: False,
        }
        exchange = self.compile_function("_exchange_serial_line", namespace)

        self.assertEqual(exchange("PGFnSampleGcode.txt\n"), "position")
        self.assertEqual(validations, ["PGFnSampleGcode.txt\n"])
        self.assertEqual(len(exchanges), 1)
        self.assertIs(exchanges[0][0], serial_port)
        self.assertIs(exchanges[0][2], cancellation)
        self.assertIs(exchanges[0][3]["write_lock"], namespace["serial_write_lock"])

    def test_firmware_source_contract_missing_gcode_file_recovers_loop(self):
        source = TEENSY_SOURCE.read_text(encoding="utf-8")
        playback_start = source.index('if (function == "PG")')
        playback_end = source.index('if (function == "WG")', playback_start)
        playback = source[playback_start:playback_end]
        missing_start = playback.index("if (!gcFile)")
        missing_end = playback.index("while (gcFile.available()", missing_start)
        missing_file = playback[missing_start:missing_end]

        recovery_statements = [
            line.strip()
            for line in missing_file.splitlines()
            if line.strip()
            and not line.strip().startswith("//")
            and line.strip() not in {"if (!gcFile) {", "}"}
        ]
        self.assertEqual(
            recovery_statements,
            [
                'Serial.println("EG");',
                "consume_current_command();",
                "return;",
            ],
        )
        self.assertNotIn("while (1)", missing_file)

    def test_firmware_stored_playback_stops_after_reported_motion_fault(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        queue_contract = TEENSY_QUEUE_CONTRACT.read_text(encoding="utf-8")
        self.assertIn("enum class MotionCommandStatus", queue_contract)
        self.assertIn("should_emit_generic_motion_error(", queue_contract)
        self.assertIn("should_continue_stored_playback(", queue_contract)

        move_start = firmware.index("MotionCommandStatus moveJ(")
        move_end = firmware.index("const int32_t MODBUS_PARSE_ERROR", move_start)
        move = firmware[move_start:move_end]
        reported_responses = re.findall(
            r'Serial\.println\(Alarm\);\s*Alarm = "0";\s*'
            r"return ar4_protocol::MotionCommandStatus::"
            r"kTerminalFaultReported;",
            move,
        )
        self.assertEqual(len(reported_responses), 2)
        collision_start = move.index("checkEncoders();")
        normal_response = move.index("if (response == true)", collision_start)
        collision = move[collision_start:normal_response]
        self.assertIn("if (TotalCollision > 0)", collision)
        self.assertLess(
            collision.index("sendRobotPos();"),
            collision.index("kTerminalFaultReported"),
        )
        self.assertLess(
            normal_response,
            move.index("kCompleted", normal_response),
        )

        playback_start = firmware.index('if (function == "PG")')
        playback_end = firmware.index(
            'if (function == "WG")',
            playback_start,
        )
        playback = firmware[playback_start:playback_end]
        cartesian_start = playback.index('if (Cmd.substring(0, 1) == "X")')
        cartesian_end = playback.index(
            "int i1 = Cmd.indexOf(',');",
            cartesian_start,
        )
        cartesian = playback[cartesian_start:cartesian_end]
        status_assignment = cartesian.index(
            "const ar4_protocol::MotionCommandStatus status ="
        )
        move_call = cartesian.index(
            "moveJ(Cmd, false, false, true);",
            status_assignment,
        )
        stop_start = cartesian.index(
            "if (!ar4_protocol::should_continue_stored_playback(status))"
        )
        self.assertLess(move_call, stop_start)
        stop_end = cartesian.index("return;", stop_start)
        stop_branch = cartesian[stop_start:stop_end]
        self.assertIn(
            "if (ar4_protocol::should_emit_generic_motion_error(status))",
            stop_branch,
        )
        self.assertIn("gcFile.close();", stop_branch)
        self.assertIn("consume_current_command();", stop_branch)

        for function, next_function, expected_call in (
            ("MJ", "MG", "moveJ(inData, true, false, false);"),
            ("MG", "DG", "moveJ(inData, true, false, true);"),
        ):
            direct_start = firmware.index(f'if (function == "{function}")')
            direct_end = firmware.index(
                f'if (function == "{next_function}")',
                direct_start,
            )
            direct = firmware[direct_start:direct_end]
            status_assignment = direct.index(
                "const ar4_protocol::MotionCommandStatus status ="
            )
            move_call = direct.index(expected_call, status_assignment)
            response_policy = direct.index(
                "if (ar4_protocol::should_emit_generic_motion_error(status))",
                move_call,
            )
            generic_error = direct.index(
                'Serial.println("ER");',
                response_policy,
            )
            self.assertLess(move_call, response_policy)
            self.assertLess(response_policy, generic_error)
            self.assertNotIn("if (!moveJ(", direct)

    def test_tool_jog_wrist_mode_is_paired_with_firmware_parser(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        native_kinematics = NATIVE_KINEMATICS_SOURCE.read_text(encoding="utf-8")
        wrist_contract = TEENSY_WRIST_SELECTION_CONTRACT.read_text(encoding="utf-8")
        motion_contract = TEENSY_MOTION_COMMAND_PARSE_CONTRACT.read_text(
            encoding="utf-8"
        )
        branch_start = firmware.index('if (function == "JT")')
        branch_end = firmware.index('if (function == "MV")', branch_start)
        branch = firmware[branch_start:branch_end]
        self.assertIn("parse_tool_jog_command(inData, commandFields)", branch)
        parse_failure = branch.index("if (!ar4_protocol::parse_tool_jog_command")
        parse_return = branch.index("return;", parse_failure)
        self.assertIn(
            "consume_current_command();",
            branch[parse_failure:parse_return],
        )
        self.assertIn(
            "command.lastIndexOf('W', loop_mode_start - 1)",
            motion_contract,
        )
        self.assertIn(
            "parse_float_spans(command, begins, ends, parsed)",
            motion_contract,
        )
        self.assertIn("!valid_wrist_config(wrist_config)", motion_contract)
        self.assertNotIn("parse_int_spans", motion_contract)

        command = "JTW11Sp50G10H20I25WFLm010101\n"
        firmware_payload = command[2:].strip()
        loop_mode_start = firmware_payload.index("Lm")
        wrist_start = firmware_payload.rfind("W", 0, loop_mode_start)
        ramp_start = firmware_payload.index("I")
        self.assertEqual(
            float(firmware_payload[ramp_start + 1:wrist_start]),
            25.0,
        )
        self.assertEqual(
            firmware_payload[wrist_start + 1:loop_mode_start],
            "F",
        )
        self.assertEqual(firmware_payload[loop_mode_start + 2:], "010101")

        solver_start = firmware.index(
            "void SolveInverseKinematics(char wrist_config)"
        )
        solver_end = firmware.index("template<typename T>", solver_start)
        solver = firmware[solver_start:solver_end]
        self.assertNotIn("WristCon", solver)
        self.assertIn("ar4_protocol::generate_wrist_solutions(", solver)
        self.assertIn("ar4_protocol::select_wrist_solution(", solver)
        self.assertIn(
            "ar4_protocol::generate_wrist_solutions(",
            native_kinematics,
        )
        self.assertIn(
            "ar4_protocol::select_wrist_solution(",
            native_kinematics,
        )
        self.assertIn("if (solVal < 0)", solver)
        rejected_start = solver.index("if (solVal < 0)")
        rejected_end = solver.index(
            "for (int i = 0; i < ROBOT_nDOFs; i++)",
            rejected_start,
        )
        self.assertIn("KinematicError = 1;", solver[rejected_start:rejected_end])
        self.assertIn("return;", solver[rejected_start:rejected_end])
        self.assertIn("wrist_config_valid(wrist_config)", wrist_contract)
        self.assertIn("return -1;", wrist_contract)
        self.assertNotIn("fall back to unfiltered best", solver)

        self.assertIn(
            'const char *FIRMWARE_VERSION = "6.7.1-ar4hmi.1";',
            firmware,
        )
        self.assertIn('"JT_WRIST_CONFIG_V1"', firmware)

        host = AR4_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn('+"I"+ACCramp+"Lm"+LoopMode', host)
        self.assertIn(
            '+"I"+ACCramp+"W"+RUN[\'WC\']+"Lm"+LoopMode',
            host,
        )

    def test_native_windows_build_default_resolves_after_script_start(self):
        source = NATIVE_WINDOWS_BUILD_SOURCE.read_text(encoding="utf-8")
        parameter_end = source.index(")\n\n$ErrorActionPreference")
        self.assertNotIn("$PSScriptRoot", source[:parameter_end])
        source_directory = source.index("$sourceDirectory = $PSScriptRoot")
        default_directory = source.index(
            '$BuildDirectory = Join-Path $sourceDirectory "build-windows-x64"'
        )
        self.assertLess(source_directory, default_directory)

    def test_controller_identity_probe_matches_firmware_opcode(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        identity_contract = TEENSY_IDENTITY_CONTRACT.read_text(encoding="utf-8")
        persistence_contract = TEENSY_PERSISTENCE_CONTRACT.read_text(
            encoding="utf-8"
        )
        host = AR4_SOURCE.read_text(encoding="utf-8")

        self.assertIn('if (function == "HO")', firmware)
        hello_start = firmware.index('if (function == "HO")')
        hello_end = firmware.index('if (function == "RB")', hello_start)
        self.assertNotIn("DEBUG_PRINT", firmware[hello_start:hello_end])
        self.assertIn("ar4_protocol::build_identity_json(", firmware)
        self.assertIn("escape_identity_json", identity_contract)
        self.assertIn("kIdentityFieldMaximumLength = 31", identity_contract)
        self.assertIn("parse_identity_set_command(", identity_contract)
        self.assertIn("count_identity_marker(", identity_contract)
        set_identity_start = firmware.index('if (function == "SR")')
        set_identity_end = firmware.index(
            'if (function == "BA")',
            set_identity_start,
        )
        set_identity = firmware[set_identity_start:set_identity_end]
        self.assertIn("IdentitySetCommandFields commandFields", set_identity)
        self.assertIn("parse_identity_set_command(", set_identity)
        self.assertNotIn('inData.indexOf("[M]")', set_identity)
        self.assertNotIn("inData.substring(", set_identity)
        self.assertIn("consume_current_command();", set_identity)
        validator_start = firmware.index("String validated_identity_field")
        validator_end = firmware.index("void use_default_robot_identity", validator_start)
        self.assertNotIn(".trim()", firmware[validator_start:validator_end])
        self.assertIn(
            "value[kIdentityFieldStorageSize - 1] != '\\0'",
            persistence_contract,
        )
        self.assertIn(
            "memcmp(stored, verified, sizeof(stored)) == 0",
            persistence_contract,
        )
        self.assertIn(
            "kIdentityErasedMarker = 0xFFFFFFFFUL",
            persistence_contract,
        )
        self.assertIn(
            "kIdentityTransactionMarker = 0x41523400UL",
            persistence_contract,
        )
        self.assertIn(
            "kIdentityLegacyMagicNumber = 0x41523401UL",
            persistence_contract,
        )
        self.assertIn(
            "kIdentityMagicNumber = 0x41523402UL",
            persistence_contract,
        )
        self.assertIn(
            "marker == kIdentityErasedMarker",
            persistence_contract,
        )
        self.assertIn(
            "marker != kIdentityMagicNumber",
            persistence_contract,
        )
        self.assertIn("load_legacy_identity_field(", persistence_contract)
        self.assertIn("migrate_legacy_persistence(", persistence_contract)
        setup_start = firmware.index("void setup()")
        setup_end = firmware.index("void loop()", setup_start)
        setup = firmware[setup_start:setup_end]
        self.assertLess(
            setup.index("migrate_legacy_persistence(EEPROM)"),
            setup.index("load_debug_from_eeprom()"),
        )
        migration_failure = setup[
            setup.index("PersistenceMigrationStatus::kFailed"):
            setup.index("load_debug_from_eeprom()")
        ]
        self.assertIn("IdentityRecordStatus::kCorrupt", migration_failure)
        self.assertIn("clear_robot_identity()", migration_failure)
        self.assertIn(
            "kIdentityTransactionMarker\n        )",
            persistence_contract,
        )
        self.assertNotIn(
            "verified[ar4_protocol::kIdentityFieldMaximumLength] = '\\0'",
            firmware,
        )
        self.assertIn("IdentityRecordStatus::kCorrupt", firmware)
        self.assertIn(
            'Serial.println("EEPROM identity record corrupt in load_robot_id")',
            firmware,
        )
        corrupt_branch_start = firmware.index(
            'Serial.println("EEPROM identity record corrupt in load_robot_id")'
        )
        corrupt_branch_end = firmware.index("return;", corrupt_branch_start)
        corrupt_branch = firmware[corrupt_branch_start:corrupt_branch_end]
        self.assertIn("clear_robot_identity()", corrupt_branch)
        self.assertNotIn("use_default_robot_identity()", corrupt_branch)
        self.assertNotIn("identity_field_or_default", firmware)
        handler_start = firmware.index("void handle_hello_command()")
        handler_end = firmware.index(
            "void handle_set_robot_id_command",
            handler_start,
        )
        handler = firmware[handler_start:handler_end]
        self.assertIn("IdentityRecordStatus::kCorrupt", handler)
        self.assertIn('Serial.println("ER")', handler)
        self.assertLess(
            handler.index("IdentityRecordStatus::kCorrupt"),
            handler.index("build_identity_json"),
        )
        persistence_failure_start = firmware.index(
            'Serial.println("Error: Robot identity persistence failed")'
        )
        persistence_failure = firmware[
            firmware.rfind("if (!save_robot_id_to_eeprom", 0, persistence_failure_start):
            persistence_failure_start
        ]
        self.assertIn("IdentityRecordStatus::kCorrupt", persistence_failure)
        self.assertIn("clear_robot_identity()", persistence_failure)
        self.assertIn(
            '_startup_exchange_response("HO\\n", cancel_event)',
            host,
        )
        self.assertNotIn('_startup_exchange_response("HELLO', host)

    def test_control_queries_bypass_the_spline_position_preface(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        contract = TEENSY_SPLINE_RESPONSE_CONTRACT.read_text(encoding="utf-8")
        process_start = firmware.index("void processSerial()")
        process_end = firmware.index("void shiftCMDarray()", process_start)
        process = firmware[process_start:process_end]

        self.assertIn("ar4_protocol::should_emit_spline_preface(", process)
        self.assertIn("procCMDtype.c_str()", process)
        preface_start = process.index("should_emit_spline_preface")
        preface_end = process.index("sendRobotPosSpline();", preface_start)
        self.assertNotIn('procCMDtype == "HO"', process[preface_start:preface_end])
        self.assertIn("opcode[0] == 'M'", contract)
        self.assertIn("opcode[1] == 'S'", contract)
        self.assertIn("opcode[2] == '\\0'", contract)

    def test_firmware_early_exits_consume_and_advance_the_shared_queue(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        queue_contract = TEENSY_QUEUE_CONTRACT.read_text(encoding="utf-8")
        loop_start = firmware.index("void loop()")
        loop_source = firmware[loop_start:]
        for match in re.finditer(r"\breturn;", loop_source):
            preceding = loop_source[max(0, match.start() - 180):match.start()]
            self.assertIn("consume_current_command();", preceding)
        self.assertIn("first = second;", queue_contract)
        self.assertIn("second = third;", queue_contract)
        self.assertIn("if (first.length() == 0)", queue_contract)

    def test_firmware_queue_and_sd_ingress_preserve_non_delimiter_bytes(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        queue_contract = TEENSY_QUEUE_CONTRACT.read_text(encoding="utf-8")
        process_start = firmware.index("void processSerial()")
        process_end = firmware.index("void shiftCMDarray()", process_start)
        loop_start = firmware.index("void loop()")
        first_handler = firmware.index('if (function == "HO")', loop_start)
        playback_start = firmware.index('if (function == "PG")')
        playback_end = firmware.index('if (function == "WG")', playback_start)

        self.assertNotIn(".trim()", firmware[process_start:process_end])
        self.assertNotIn(".trim()", firmware[loop_start:first_handler])
        self.assertNotIn(".trim()", firmware[playback_start:playback_end])
        self.assertGreaterEqual(
            firmware.count("extract_serial_command_payload("),
            4,
        )
        self.assertIn(
            "extract_stored_command_payload(storedRow, Cmd)",
            firmware[playback_start:playback_end],
        )
        self.assertIn(
            "read_stored_command_row(gcFile, storedRow)",
            firmware[playback_start:playback_end],
        )
        self.assertNotIn(
            "readStringUntil(",
            firmware[playback_start:playback_end],
        )
        self.assertIn("frame.charAt(end - 1) != '\\n'", queue_contract)
        self.assertIn("frame.charAt(end - 1) == '\\r'", queue_contract)
        self.assertIn("row.charAt(end - 1) == '\\r'", queue_contract)

    def test_firmware_debug_command_is_validated_and_applied_transactionally(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        debug_contract = TEENSY_DEBUG_CONTRACT.read_text(encoding="utf-8")
        persistence_contract = TEENSY_PERSISTENCE_CONTRACT.read_text(encoding="utf-8")
        branch_start = firmware.index('if (function == "DB")')
        branch_end = firmware.index('if (function == "SR")', branch_start)
        branch = firmware[branch_start:branch_end]
        save_start = firmware.index("bool save_debug_to_eeprom")
        save_end = firmware.index("void load_robot_id_from_eeprom", save_start)
        save_function = firmware[save_start:save_end]

        self.assertIn('#include "debug_contract.h"', firmware)
        self.assertIn('#include "persistence_contract.h"', firmware)
        self.assertIn("parse_debug_command(inData.c_str(), debugCommand)", branch)
        self.assertIn("apply_debug_command(", branch)
        self.assertNotRegex(branch, r"\bDEBUG\s*=")
        self.assertIn("return false;", save_function)
        self.assertIn("return true;", save_function)
        self.assertIn("command.persistence_requested", debug_contract)
        self.assertIn("save_debug_record(EEPROM, value)", save_function)
        self.assertIn("kDebugMagicAddress", persistence_contract)
        self.assertIn("write_eeprom_verified", persistence_contract)
        self.assertLess(
            branch.index("parse_debug_command"),
            branch.index("apply_debug_command"),
        )
        failed_apply = branch.index("if (!ar4_protocol::apply_debug_command")
        done = branch.index('Serial.println("Done")', failed_apply)
        self.assertIn("consume_current_command();", branch[failed_apply:done])
        self.assertIn("return;", branch[failed_apply:done])

    def test_firmware_tool_frame_storage_uses_xyz_rx_ry_rz(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        angle_contract = TEENSY_ANGLE_CONVERSION_CONTRACT.read_text(
            encoding="utf-8"
        )
        tool_contract = TEENSY_TOOL_JOG_CONTRACT.read_text(encoding="utf-8")
        self.assertIn(
            "Robot_Kin_Tool[5] = nativeToolRz;",
            firmware,
        )
        self.assertIn(
            "Robot_Kin_Tool[3] = nativeToolRx;",
            firmware,
        )
        self.assertIn("degrees_to_radians(float degrees", angle_contract)
        self.assertIn("degrees != 0.0f && staged == 0.0f", angle_contract)
        self.assertIn("round_trip_degrees", angle_contract)
        self.assertIn("decode_discrete_tool_offset(", firmware)
        self.assertIn("decode_live_tool_offset(", firmware)
        self.assertIn("if (axis == 'W') return 3;", tool_contract)
        self.assertIn("if (axis == 'R') return 5;", tool_contract)
        self.assertNotIn("void updatejoints()", firmware)

        update_start = firmware.index('if (function == "UP")')
        update_end = firmware.index('if (function == "CE")', update_start)
        update_branch = firmware[update_start:update_end]
        refresh_start = update_branch.index("if (!robot_set_AR())")
        refresh_end = update_branch.index('Serial.print("Done")', refresh_start)
        self.assertIn(
            "consume_current_command();",
            update_branch[refresh_start:refresh_end],
        )
        self.assertIn("return;", update_branch[refresh_start:refresh_end])

        vision_start = firmware.index('if (function == "MV")')
        vision_end = firmware.index('if (function == "ML"', vision_start)
        vision_branch = firmware[vision_start:vision_end]
        self.assertIn("float RXtool = Robot_Kin_Tool[3];", vision_branch)
        self.assertIn(
            "Robot_Kin_Tool[3] = Robot_Kin_Tool[3] "
            "- vision_rotation_radians;",
            vision_branch,
        )
        self.assertIn("Robot_Kin_Tool[3] = RXtool;", vision_branch)

    def test_live_jog_handlers_forward_validated_motion_profiles(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        motion_contract = TEENSY_MOTION_COMMAND_PARSE_CONTRACT.read_text(
            encoding="utf-8"
        )
        self.assertIn("speed_mode != 'p'", motion_contract)

        stop_reader_start = firmware.index(
            "LiveControlFrameStatus read_live_control_frame()"
        )
        stop_reader_end = firmware.index(
            "void send_live_terminal_response(",
            stop_reader_start,
        )
        stop_reader = firmware[stop_reader_start:stop_reader_end]
        self.assertIn("read_serial_frame_byte(inData)", stop_reader)
        self.assertIn("classify_live_control_frame(", stop_reader)
        self.assertIn("inData.c_str()", stop_reader)

        terminal_end = firmware.index("void EstopProg()", stop_reader_end)
        terminal_sender = firmware[stop_reader_end:terminal_end]
        self.assertIn("select_live_terminal_response(", terminal_sender)
        self.assertEqual(terminal_sender.count("sendRobotPos();"), 1)
        self.assertEqual(terminal_sender.count('Serial.println("ER");'), 1)
        self.assertEqual(
            terminal_sender.count("Serial.println(axis_limit_response);"),
            1,
        )

        handler_pairs = (
            ("LC", "LJ", "kCartesian"),
            ("LJ", "LT", "kJoint"),
            ("LT", "JT", "kTool"),
        )
        for opcode, next_opcode, command_kind in handler_pairs:
            branch_start = firmware.index(f'if (function == "{opcode}")')
            branch_end = firmware.index(
                f'if (function == "{next_opcode}")',
                branch_start,
            )
            branch = firmware[branch_start:branch_end]
            with self.subTest(opcode=opcode):
                acknowledgement = branch.index("Serial.println();")
                self.assertLess(
                    branch.index("parse_live_jog_command("),
                    acknowledgement,
                )
                self.assertLess(
                    branch.index('inData = "";'),
                    acknowledgement,
                )
                if opcode == "LT":
                    self.assertLess(
                        branch.index("decode_live_tool_offset("),
                        acknowledgement,
                    )
                self.assertIn(
                    "float ACCspd = commandFields.acceleration;",
                    branch,
                )
                self.assertIn(
                    "float DCCspd = commandFields.deceleration;",
                    branch,
                )
                self.assertIn("float ACCramp = commandFields.ramp;", branch)
                self.assertIn(
                    f"LiveJogCommandKind::{command_kind}",
                    branch,
                )
                self.assertIn("driveMotorsJ(", branch)
                self.assertIn(
                    "liveControlStatus = read_live_control_frame();",
                    branch,
                )
                self.assertEqual(
                    branch.count("send_live_terminal_response("),
                    1,
                )
                self.assertNotIn("Serial.read()", branch)
                self.assertNotRegex(
                    branch,
                    r"float (?:ACCspd|DCCspd|ACCramp)\s*=\s*(?:0|100)(?:\.0f)?;",
                )

    def test_firmware_finite_trajectories_stop_after_first_axis_fault(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        branch_markers = (
            ("ML", 'if (function == "MJ")'),
            ("MC", 'if (function == "MA"'),
            ("MA", None),
        )
        for opcode, end_marker in branch_markers:
            branch_start = firmware.index(f'if (function == "{opcode}"')
            branch_end = (
                len(firmware)
                if end_marker is None
                else firmware.index(end_marker, branch_start)
            )
            branch = firmware[branch_start:branch_end]
            loop_start = branch.index(
                "for (int i = 1; i <= waypoint_count; i++)"
            )
            terminal_start = branch.index(
                "if (KinematicError == 1)",
                loop_start,
            )
            waypoint_loop = branch[loop_start:terminal_start]
            fault_start = waypoint_loop.index('Alarm = "EL"')
            drive_start = waypoint_loop.index("driveMotorsL(", fault_start)
            with self.subTest(opcode=opcode):
                self.assertIn(
                    "break;",
                    waypoint_loop[fault_start:drive_start],
                )
                self.assertNotIn("Serial.println(Alarm);", waypoint_loop)
                terminal = branch[terminal_start:]
                self.assertIn(
                    "else if (TotalAxisFault != 0)",
                    terminal,
                )
                self.assertIn("Serial.println(Alarm);", terminal)

    def test_firmware_serial_ingress_is_bounded_and_recovers_after_overflow(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        frame_contract = TEENSY_SERIAL_FRAME_CONTRACT.read_text(
            encoding="utf-8"
        )

        self.assertIn(
            f"kSerialCommandFrameMaximumLength = {MAX_COMMAND_LENGTH}",
            frame_contract,
        )
        self.assertIn(
            "kStoredCommandRowMaximumLength =",
            frame_contract,
        )
        self.assertIn("append_stored_row_byte(", frame_contract)
        self.assertIn("finish_stored_row(", frame_contract)
        self.assertIn("frame = \"\";", frame_contract)
        self.assertIn("discarding = received != '\\n';", frame_contract)
        self.assertIn('#include "serial_frame_contract.h"', firmware)
        self.assertEqual(firmware.count("Serial.read()"), 1)
        self.assertEqual(
            firmware.count("read_serial_frame_byte(recData)"),
            2,
        )
        reader_start = firmware.index(
            "SerialFrameReadStatus read_serial_frame_byte(String &frame)"
        )
        reader_end = firmware.index("void processSerial()", reader_start)
        reader = firmware[reader_start:reader_end]
        self.assertNotIn('Serial.println("ER")', reader)
        process_end = firmware.index("void shiftCMDarray()", reader_end)
        process_serial = firmware[reader_end:process_end]
        self.assertEqual(
            process_serial.count("SerialFrameReadStatus::kOverflow"),
            2,
        )
        self.assertEqual(process_serial.count('Serial.println("ER")'), 2)
        self.assertEqual(process_serial.count("return;"), 2)

    def test_main_controller_outputs_are_explicitly_unsupported(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        controller_contract = TEENSY_CONTROLLER_DOMAIN_CONTRACT.read_text(
            encoding="utf-8"
        )
        branch_start = firmware.index(
            'if (function == "ON" || function == "OF")'
        )
        branch_end = firmware.index(
            'if (function == "WJ")',
            branch_start,
        )
        branch = firmware[branch_start:branch_end]

        self.assertIn('Serial.println("ER")', branch)
        self.assertIn("consume_current_command();", branch)
        self.assertIn("return;", branch)
        self.assertNotIn("digitalWrite(", branch)
        self.assertNotIn("main_output_pin_is_allowlisted", controller_contract)

    def test_rejected_firmware_motion_preserves_active_mode_state(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        transaction_contract = TEENSY_MOTION_MODE_TRANSACTION.read_text(
            encoding="utf-8"
        )

        constructor_start = transaction_contract.index(
            "MotionModeTransaction("
        )
        destructor_start = transaction_contract.index(
            "~MotionModeTransaction()",
            constructor_start,
        )
        constructor = transaction_contract[
            constructor_start:destructor_start
        ]
        commit_start = transaction_contract.index("void commit()")
        commit_end = transaction_contract.index(
            "bool committed() const",
            commit_start,
        )
        commit = transaction_contract[commit_start:commit_end]
        self.assertNotIn("active_wrist_ = requested_wrist_;", constructor)
        self.assertIn("active_wrist_ = requested_wrist_;", commit)
        self.assertIn(
            "active_loop_modes_[index] = requested_loop_modes_[index];",
            commit,
        )
        self.assertIn("if (!committed_) restore();", transaction_contract)

        self.assertIn('#include "motion_mode_transaction.h"', firmware)
        self.assertIn("int JointLoopModes[ROBOT_nDOFs];", firmware)
        self.assertIn("volatile bool estopActive;", firmware)
        self.assertNotIn("apply_loop_modes(", firmware)
        self.assertNotIn("SolveInverseKinematics();", firmware)
        self.assertEqual(
            firmware.count(
                "MotionModeTransaction<String, ROBOT_nDOFs> motionModes("
            ),
            11,
        )
        self.assertNotIn("motionModes.commit();", firmware)
        self.assertEqual(firmware.count("motionModes->commit();"), 3)

        driver_markers = (
            ("driveMotorsJ", "bool driveMotorsJ(", "//DRIVE MOTORS G"),
            ("driveMotorsG", "bool driveMotorsG(", "//DRIVE MOTORS L"),
            ("driveMotorsL", "bool driveMotorsL(", "//MOVE J"),
        )
        for name, start_marker, end_marker in driver_markers:
            start = firmware.index(start_marker)
            end = firmware.index(end_marker, start)
            driver = firmware[start:end]
            with self.subTest(driver=name):
                self.assertIn(
                    "FirmwareMotionModeTransaction *motionModes",
                    driver,
                )
                self.assertIn("if (estopActive) return false;", driver)
                self.assertLess(
                    driver.index("valid_delay_envelope("),
                    driver.index("motionModes->commit();"),
                )
                self.assertLess(
                    driver.index("motionModes->commit();"),
                    driver.index("digitalWrite("),
                )

        scopes = (
            ("moveJ", "MotionCommandStatus moveJ(", "//COMMUNICATIONS", 2),
            ("LC", 'if (function == "LC"', 'if (function == "LJ"', 1),
            ("LJ", 'if (function == "LJ"', 'if (function == "LT"', 1),
            ("LT", 'if (function == "LT"', 'if (function == "JT"', 1),
            ("JT", 'if (function == "JT"', 'if (function == "MV"', 1),
            ("MV", 'if (function == "MV"', 'if (function == "RJ"', 1),
            ("RJ", 'if (function == "RJ"', 'if (function == "ML"', 1),
            ("ML", 'if (function == "ML"', 'if (function == "MJ"', 1),
            ("WG", 'if (function == "WG"', 'if (function == "MC"', 0),
            ("MC", 'if (function == "MC"', 'if (function == "MA"', 1),
            ("MA", 'if (function == "MA"', None, 1),
        )
        for name, start_marker, end_marker, transaction_arguments in scopes:
            start = firmware.index(start_marker)
            end = (
                len(firmware)
                if end_marker is None
                else firmware.index(end_marker, start)
            )
            scope = firmware[start:end]
            with self.subTest(scope=name):
                self.assertEqual(
                    scope.count(
                        "MotionModeTransaction<String, ROBOT_nDOFs> "
                        "motionModes("
                    ),
                    1,
                )
                self.assertNotIn(
                    "WristCon = String(commandFields.wrist_config);",
                    scope,
                )
                self.assertNotIn("apply_loop_modes(", scope)
                self.assertNotIn("motionModes.commit();", scope)
                self.assertEqual(
                    scope.count("&motionModes"),
                    transaction_arguments,
                )

    def test_unimplemented_motion_options_fail_closed_in_firmware(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        motion_contract = TEENSY_MOTION_COMMAND_PARSE_CONTRACT.read_text(
            encoding="utf-8"
        )
        controller_contract = TEENSY_CONTROLLER_DOMAIN_CONTRACT.read_text(
            encoding="utf-8"
        )

        self.assertIn(
            "command.charAt(disable_wrist_start + 1) != '0'",
            motion_contract,
        )
        self.assertNotIn("disable_wrist_rotation", firmware)
        self.assertNotIn("disable_wrist_rotation", motion_contract)
        self.assertIn("supported_trajectory_rotation(", controller_contract)
        self.assertIn("enum class CartesianMotionCommandKind", motion_contract)
        self.assertIn("enum class LiveJogCommandKind", motion_contract)
        self.assertIn(
            "(requires_rounding ? rounding_start < 0 : rounding_start >= 0)",
            motion_contract,
        )
        self.assertIn(
            "kind == LiveJogCommandKind::kJoint && wrist_config != 'A'",
            motion_contract,
        )

        gcode_constants = {
            node.value
            for node in ast.walk(self.module_functions["GCexecuteRow"])
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertNotIn("Rnd", gcode_constants)

        for opcode, next_opcode in (("MC", "MA"), ("MA", None)):
            branch_start = firmware.index(f'if (function == "{opcode}"')
            branch_end = (
                len(firmware)
                if next_opcode is None
                else firmware.index(
                    f'if (function == "{next_opcode}"',
                    branch_start,
                )
            )
            branch = firmware[branch_start:branch_end]
            with self.subTest(opcode=opcode):
                self.assertIn("supported_trajectory_rotation(", branch)
                self.assertNotIn("trajectoryRotation", branch)

    def test_firmware_numeric_fields_use_strict_text_parsing(self):
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        numeric_contract = TEENSY_NUMERIC_PARSE_CONTRACT.read_text(
            encoding="utf-8"
        )
        motion_contract = TEENSY_MOTION_COMMAND_PARSE_CONTRACT.read_text(
            encoding="utf-8"
        )
        controller_contract = TEENSY_CONTROLLER_DOMAIN_CONTRACT.read_text(
            encoding="utf-8"
        )

        self.assertIn('#include "numeric_parse_contract.h"', firmware)
        self.assertIn('#include "motion_command_parse_contract.h"', firmware)
        self.assertIn('#include "controller_domain_contract.h"', firmware)
        self.assertNotRegex(firmware, r"\.to(?:Float|Int)\(\)")
        self.assertNotRegex(firmware, r"xyzuvw_In\[6\]\s*=")
        self.assertIn("strtof(text, &end)", numeric_contract)
        self.assertIn("strtol(text, &end, 10)", numeric_contract)
        self.assertIn("parsed == 0.0f && nonzero_mantissa", numeric_contract)
        self.assertIn("*end != '\\0'", numeric_contract)

        update_start = firmware.index('if (function == "UP")')
        update_end = firmware.index('if (function == "CE")', update_start)
        update_branch = firmware[update_start:update_end]
        parse_start = update_branch.index("parse_float_marker_fields(")
        first_mutation = update_branch.index("Robot_Kin_Tool[0] =")
        self.assertLess(parse_start, first_mutation)
        self.assertIn("parse_int_marker_fields(", update_branch)
        self.assertIn("consume_current_command();", update_branch[:first_mutation])

        move_j_start = firmware.index("MotionCommandStatus moveJ(")
        move_j_end = firmware.index("//COMMUNICATIONS", move_j_start)
        move_j = firmware[move_j_start:move_j_end]
        self.assertIn("parse_cartesian_move_command(inData, commandFields)", move_j)
        self.assertNotIn("RndStart", move_j)

        jog_start = firmware.index('if (function == "JT")')
        jog_end = firmware.index('if (function == "MV")', jog_start)
        jog_branch = firmware[jog_start:jog_end]
        self.assertIn("parse_tool_jog_command(inData, commandFields)", jog_branch)
        self.assertNotIn("parse_int_spans", jog_branch)

        vision_start = firmware.index('if (function == "MV")')
        vision_end = firmware.index('if (function == "ML"', vision_start)
        vision_branch = firmware[vision_start:vision_end]
        self.assertIn("parse_vision_move_command(inData, commandFields)", vision_branch)
        self.assertNotIn("RndStart", vision_branch)

        linear_start = vision_end
        linear_end = firmware.index('if (function == "MA"', linear_start)
        linear_branch = firmware[linear_start:linear_end]
        self.assertIn(
            '"W" + String(commandFields.wrist_config) + "Lm"',
            linear_branch,
        )
        self.assertNotIn('"WA" + "Lm"', linear_branch)

        self.assertIn("float rounding = 0.0f;", motion_contract)
        self.assertIn("if (rounding_start >= 0)", motion_contract)
        self.assertIn("float parsed[4];", motion_contract)
        self.assertIn("parse_int_span(command, markers[0] + 1", motion_contract)
        self.assertIn("parse_binary_digit_span(", motion_contract)
        self.assertIn("valid_motion_profile(", motion_contract)
        self.assertIn("parse_joint_move_command(", motion_contract)
        self.assertIn("parse_live_jog_command(", motion_contract)
        self.assertIn("parse_linear_move_command(", motion_contract)
        self.assertIn("calibrated_position_to_step(", controller_contract)
        self.assertIn("validate_modbus_request(", controller_contract)
        self.assertIn("timeout_seconds > 0", controller_contract)
        self.assertIn("valid_controller_filename(", controller_contract)
        self.assertIn("fat_reserved_filename_character(", controller_contract)
        self.assertIn("valid_circle_geometry(", controller_contract)
        self.assertIn("valid_arc_geometry(", controller_contract)
        self.assertIn("calculate_ordered_arc_geometry(", controller_contract)
        arc_start = firmware.index('if (function == "MA"')
        arc_branch = firmware[arc_start:]
        self.assertIn("OrderedArcGeometry arc_geometry", arc_branch)
        self.assertIn("&arc_geometry", arc_branch)
        self.assertIn("arc_geometry.radians", arc_branch)
        self.assertNotIn("ABradians = acos", arc_branch)
        self.assertIn("valid_delay_envelope(", controller_contract)
        self.assertIn("pulse_delay_microseconds(", controller_contract)
        self.assertIn("bool driveMotorsJ(", firmware)
        self.assertIn("bool driveMotorsG(", firmware)
        self.assertIn("bool driveMotorsL(", firmware)
        self.assertNotIn(
            "delayMicroseconds(curDelay - disDelayCur)",
            firmware,
        )
        self.assertIn("values_are_binary(stagedDirections)", update_branch)

        calibration_start = firmware.index('if (function == "LL")')
        calibration_end = firmware.index(
            'if (function == "LC")',
            calibration_start,
        )
        calibration_branch = firmware[calibration_start:calibration_end]
        self.assertIn("values_are_binary(requested)", calibration_branch)

        playback_start = firmware.index('if (function == "PG")')
        playback_end = firmware.index('if (function == "WG")', playback_start)
        playback_branch = firmware[playback_start:playback_end]
        self.assertIn("values_are_binary(intFields + 9, 9)", playback_branch)
        self.assertIn("stored_step_target(", playback_branch)

        write_start = firmware.index('if (function == "WG")')
        write_end = firmware.index('if (function == "MC")', write_start)
        write_branch = firmware[write_start:write_end]
        self.assertIn("valid_controller_filename(", write_branch)
        self.assertIn("inverse_solution_to_future_steps(", write_branch)
        self.assertIn("if (writeSD(filename, info))", write_branch)

    def test_tool_jog_directions_match_the_firmware_contract(self):
        discrete_commands = {
            "TXjogNeg": "JTX1",
            "TXjogPos": "JTX0",
            "TYjogNeg": "JTY1",
            "TYjogPos": "JTY0",
            "TZjogNeg": "JTZ1",
            "TZjogPos": "JTZ0",
            "TRxjogNeg": "JTW1",
            "TRxjogPos": "JTW0",
            "TRyjogNeg": "JTP1",
            "TRyjogPos": "JTP0",
            "TRzjogNeg": "JTR1",
            "TRzjogPos": "JTR0",
        }
        for function_name, command_prefix in discrete_commands.items():
            with self.subTest(function=function_name):
                constants = {
                    node.value
                    for node in ast.walk(self.module_functions[function_name])
                    if isinstance(node, ast.Constant)
                    and isinstance(node.value, str)
                }
                self.assertIn(command_prefix, constants)

        live_vectors = {
            "SelTXjogNeg": 10,
            "SelTXjogPos": 11,
            "SelTYjogNeg": 20,
            "SelTYjogPos": 21,
            "SelTZjogNeg": 30,
            "SelTZjogPos": 31,
            "SelTRzjogNeg": 40,
            "SelTRzjogPos": 41,
            "SelTRyjogNeg": 50,
            "SelTRyjogPos": 51,
            "SelTRxjogNeg": 60,
            "SelTRxjogPos": 61,
        }
        for function_name, expected_vector in live_vectors.items():
            with self.subTest(function=function_name):
                calls = [
                    node
                    for node in ast.walk(self.module_functions[function_name])
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "LiveToolJog"
                ]
                self.assertEqual(len(calls), 1)
                self.assertEqual(calls[0].args[0].value, expected_vector)

        host = AR4_SOURCE.read_text(encoding="utf-8")
        self.assertIn("if axis == 'Rz': return 40 if d < 0 else 41", host)
        self.assertIn("if axis == 'Rx': return 60 if d < 0 else 61", host)
        self.assertIn("if axis == 'Tz': return 30 if d < 0 else 31", host)

        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        self.assertIn("ar4_protocol::decode_discrete_tool_offset(", firmware)
        self.assertIn("ar4_protocol::decode_live_tool_offset(", firmware)

    def test_virtual_vision_and_live_tool_offsets_match_firmware(self):
        host = AR4_SOURCE.read_text(encoding="utf-8")
        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        tool_contract = TEENSY_TOOL_JOG_CONTRACT.read_text(encoding="utf-8")

        self.assertIn("LIVE_TOOL_JOG_INCREMENT = 0.25", host)
        self.assertIn("jog_step = LIVE_TOOL_JOG_INCREMENT", host)
        self.assertIn("kLiveToolJogIncrement = 0.25f", tool_contract)
        self.assertIn("ar4_protocol::kLiveToolJogIncrement", firmware)

        move_v_start = host.index('if (RUN[\'cmdType\'] == "Move V")')
        move_v_end = host.index('if (RUN[\'cmdType\'] == "Move P")', move_v_start)
        move_v_branch = host[move_v_start:move_v_end]
        self.assertIn('commandVR = "MV"', move_v_branch)
        self.assertIn('"Vr"+visRot+"Lm"', move_v_branch)
        self.assertIn("mv_command,", move_v_branch)

        vision_command = (
            "MVX1Y2Z3Rz4Ry5Rx6Sp50Ac10Dc20Rm25"
            "WNVr15Lm000000\n"
        )
        vision_parser = self.compile_function("_vision_rotation_degrees", {})
        self.assertEqual(vision_parser(vision_command), 15.0)

        class Robot:
            def __init__(self):
                self.frame = [1.0, 2.0, 3.0, 40.0, 5.0, 6.0]

            def get_robot_tool_frame(self):
                return list(self.frame)

            def set_robot_tool_frame(self, *frame):
                self.frame = list(frame)

        robot = Robot()
        namespace = {
            "contextmanager": contextmanager,
            "robot": robot,
            "_validated_virtual_six_vector": lambda values, label: tuple(values),
        }
        temporary_rotation = self.compile_function(
            "_temporary_vision_tool_rotation",
            namespace,
            preserve_decorators=True,
        )
        with temporary_rotation(15.0):
            self.assertEqual(robot.frame, [1.0, 2.0, 3.0, 25.0, 5.0, 6.0])
        self.assertEqual(robot.frame, [1.0, 2.0, 3.0, 40.0, 5.0, 6.0])

    def test_program_gcode_propagates_completion_and_rejection(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        def build_dispatch(playback):
            first_label = Label()
            second_label = Label()
            namespace = {
                "_gcode_playback_command": lambda filename: f"PGFn{filename}.txt\n",
                "_controller_response_timeout": lambda command: 0.01,
                "SERIAL_EVENT_APPLICATION_MARGIN_SECONDS": 0.01,
                "threading": threading,
                "GCplayProg": playback,
                "logger": SimpleNamespace(
                    error=lambda *args: None,
                    warning=lambda *args: None,
                ),
                "almStatusLab": first_label,
                "almStatusLab2": second_label,
                "ROW_EXECUTION_REJECTED": "rejected",
                "ROW_EXECUTION_PENDING": "pending",
                "ROW_EXECUTION_COMPLETE": "complete",
            }
            return (
                self.compile_function("_dispatch_program_gcode", namespace),
                first_label,
                second_label,
            )

        captured = []

        def asynchronous_playback(filename, completion_callback=None):
            captured.append((filename, completion_callback))
            return True

        dispatch, _, _ = build_dispatch(asynchronous_playback)
        callback_results = []
        self.assertEqual(
            dispatch("demo", callback_results.append),
            "pending",
        )
        captured[0][1](False)
        self.assertEqual(callback_results, [False])

        dispatch, _, _ = build_dispatch(
            lambda filename, completion_callback=None: (
                completion_callback(True) or True
            )
        )
        self.assertEqual(dispatch("demo", None), "complete")

        dispatch, first_label, second_label = build_dispatch(
            lambda filename, completion_callback=None: False
        )
        self.assertEqual(dispatch("demo", None), "rejected")
        self.assertIn("not started", first_label.text)
        self.assertEqual(second_label.text, first_label.text)

    def test_run_gcode_row_returns_the_playback_state(self):
        class Entry:
            def __init__(self, value=""):
                self.value = value

            def get(self):
                return self.value

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

        class ProgramView:
            def __init__(self):
                self.selected = [4]

            def curselection(self):
                return tuple(self.selected)

            def see(self, row):
                pass

            def get(self, row):
                return b"Run Gcode Program - demo"

            def selection_clear(self, *args):
                self.selected = []

            def select_set(self, row):
                self.selected = [row]

        view = ProgramView()
        callback_results = []
        callback = callback_results.append
        dispatches = []
        finishes = []
        namespace = {
            "RUN": {
                "progRunning": False,
                "cmdType": None,
                "cmdTypeLong": None,
                "offlineMode": False,
                "moveInProc": 0,
            },
            "tab1": SimpleNamespace(
                progView=view,
                lastRow=2,
                lastProg="caller.ar4",
            ),
            "ProgEntryField": Entry("main.ar4"),
            "manEntryField": Entry(),
            "END": "end",
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "_dispatch_program_gcode": (
                lambda filename, completion: dispatches.append(
                    (filename, completion)
                ) or "pending"
            ),
            "_finish_execute_row": lambda: finishes.append(True),
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
        }
        execute = self.compile_function("executeRow", namespace)

        self.assertEqual(execute(callback), "pending")
        self.assertEqual(dispatches, [("demo", callback)])
        self.assertEqual(finishes, [])
        self.assertEqual(view.selected, [4])
        self.assertEqual(namespace["tab1"].lastRow, 2)
        self.assertEqual(namespace["tab1"].lastProg, "caller.ar4")

        dispatches[0][1](False)
        self.assertEqual(callback_results, [False])
        self.assertEqual(view.selected, [4])

        namespace["_dispatch_program_gcode"] = lambda *args: "rejected"
        self.assertEqual(execute(callback), "rejected")
        self.assertEqual(finishes, [True])
        self.assertEqual(view.selected, [4])
        self.assertEqual(namespace["tab1"].lastRow, 2)
        self.assertEqual(namespace["tab1"].lastProg, "caller.ar4")

    def test_permanent_deferred_rejection_clears_only_invalid_intent(self):
        class Deferred:
            def __init__(self):
                self.pending = True
                self.clear_count = 0

            def ready(self, *args, **kwargs):
                return True

            def consume(
                self,
                actual_positions,
                confirmed_position_generation,
                consumer,
                allow_current_generation=False,
            ):
                result = consumer((1.0,) * 9, object())
                self.clear()
                return result

            def clear(self):
                self.pending = False
                self.clear_count += 1

        class Dispatcher:
            active = False

            def __init__(self, error):
                self.error = error

            def submit_positions(self, *args):
                raise self.error

        def run_rejection(error):
            deferred = Deferred()
            namespace = {
                "application_closing": SimpleNamespace(is_set=lambda: False),
                "deferred_joint_adjustments": deferred,
                "controller_correction_requested": threading.Event(),
                "legacy_serial_result_pending": threading.Event(),
                "serial_lock": threading.Lock(),
                "joint_motion_dispatcher": Dispatcher(error),
                "confirmed_position_generation": 1,
                "_current_joint_positions": lambda: (0.0,) * 9,
                "_clear_deferred_joint_adjustments": deferred.clear,
                "_try_set_virtual_joint_target": lambda target: True,
                "MotionTransportBusy": MotionTransportBusy,
                "MotionInputError": MotionInputError,
                "MotionQueueFault": MotionQueueFault,
                "logger": SimpleNamespace(error=lambda *args: None),
                "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
                "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            }
            dispatch = self.compile_function(
                "_try_dispatch_deferred_joint_adjustments",
                namespace,
            )
            self.assertFalse(dispatch())
            return deferred

        invalid = run_rejection(MotionInputError("outside configured limits"))
        self.assertFalse(invalid.pending)
        self.assertEqual(invalid.clear_count, 1)

        busy = run_rejection(MotionTransportBusy("transport busy"))
        self.assertTrue(busy.pending)
        self.assertEqual(busy.clear_count, 0)

    def test_gcode_stop_halts_local_scheduling_before_serial_admission(self):
        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        namespace = {
            "tab7": SimpleNamespace(GCrunTrue=1),
            "GCalmStatusLab": Label(),
        }
        stop_gcode = self.compile_function("GCstopProg", namespace)

        self.assertTrue(stop_gcode())
        self.assertEqual(namespace["tab7"].GCrunTrue, 0)
        self.assertIn("SCHEDULING HALTED", namespace["GCalmStatusLab"].text)
        self.assertIn("NOT PREEMPTED", namespace["GCalmStatusLab"].text)

        stop_calls = [
            node
            for node in ast.walk(self.module_functions["GCstopProg"])
            if isinstance(node, ast.Call)
        ]
        self.assertEqual(
            [
                call
                for call in stop_calls
                if isinstance(call.func, ast.Name)
                and call.func.id == "start_send_serial_thread"
            ],
            [],
        )

        stop_function = self.module_functions["GCstopProg"]
        self.assertEqual(stop_function.decorator_list, [])
        execute_function = self.module_functions["executeRow"]
        self.assertEqual(execute_function.decorator_list, [])
        direct_writes = [
            node
            for node in ast.walk(execute_function)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "write"
        ]
        owned_exchanges = [
            node
            for node in ast.walk(execute_function)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_exchange_gcode_row"
        ]
        self.assertEqual(direct_writes, [])
        direct_exchanges = [
            node
            for node in ast.walk(execute_function)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_execute_row_main_response"
        ]
        self.assertGreater(len(direct_exchanges), 0)

        gcode_execute = self.module_functions["GCexecuteRow"]
        owned_exchanges = [
            node
            for node in ast.walk(gcode_execute)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_exchange_gcode_row"
        ]
        self.assertEqual(len(owned_exchanges), 2)
        for function_name in ("_execute_row_main_command", "GCexecuteRow"):
            decorators = self.module_functions[function_name].decorator_list
            self.assertEqual(
                [decorator.func.id for decorator in decorators],
                ["_synchronous_motion_request", "_tracked_serial_operation"],
                function_name,
            )
        generic_request_decorator = self.module_functions[
            "_execute_row_main_command"
        ].decorator_list[0]
        generic_request_keywords = {
            keyword.arg: keyword.value
            for keyword in generic_request_decorator.keywords
        }
        self.assertIn("requires_kinematics", generic_request_keywords)
        self.assertIsInstance(
            generic_request_keywords["requires_kinematics"],
            ast.Constant,
        )
        self.assertIs(
            generic_request_keywords["requires_kinematics"].value,
            False,
        )

        firmware = TEENSY_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn('if (function == "TF")', firmware)
        execute_constants = {
            node.value
            for node in ast.walk(execute_function)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertNotIn("TF", execute_constants)
        self.assertIn(
            "Set tool program rows are unsupported by the Teensy 6.7.1 protocol",
            execute_constants,
        )

    def test_gcode_conversion_rejection_does_not_advance_selection(self):
        class Entry:
            def __init__(self, value=""):
                self.value = value

            def get(self):
                return self.value

            def delete(self, *args):
                self.value = ""

            def insert(self, index, value):
                self.value = value

        class ProgramView:
            def __init__(self):
                self.selection = 1
                self.selection_mutations = []

            @staticmethod
            def index(value):
                return 2

            @staticmethod
            def itemconfig(row, values):
                pass

            def curselection(self):
                return (self.selection,)

            def selection_clear(self, *args):
                self.selection_mutations.append("clear")

            def select_set(self, row):
                self.selection_mutations.append(("select", row))
                self.selection = row

        class SerialPort:
            def __init__(self):
                self.commands = []

            def write(self, command):
                self.commands.append(command)
                return len(command)

            @staticmethod
            def flushInput():
                pass

            @staticmethod
            def readline():
                return b"Done\n"

        class ImmediateThread:
            def __init__(self, target):
                self.target = target

            def start(self):
                self.target()

        program_view = ProgramView()
        serial_port = SerialPort()
        tab = SimpleNamespace(gcodeView=program_view, GCrunTrue=0)
        runtime = {"ser": serial_port, "GCrowinproc": 0}
        namespace = {
            "GcodeProgEntryField": Entry("loaded"),
            "GcodeFilenameField": Entry("output"),
            "cmdSentEntryField": Entry(),
            "RUN": runtime,
            "tab7": tab,
            "END": "end",
            "messagebox": SimpleNamespace(showwarning=lambda *args: None),
            "GCalmStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "GcodCurRowEntryField": Entry(),
            "serial_lock": threading.Lock(),
            "GCexecuteRow": lambda: "rejected",
            "ROW_EXECUTION_PENDING": "pending",
            "ROW_EXECUTION_COMPLETE": "complete",
            "ROW_EXECUTION_REJECTED": "rejected",
            "threading": SimpleNamespace(Thread=ImmediateThread),
            "time": SimpleNamespace(sleep=lambda seconds: None),
            "MotionInputError": MotionInputError,
            "MAX_COMMAND_LENGTH": 4096,
            "logger": SimpleNamespace(error=lambda *args: None),
            "_exchange_legacy_main_command": (
                lambda command: serial_port.commands.append(command.encode()) or "Done"
            ),
        }
        namespace["_gcode_storage_filename"] = self.compile_function(
            "_gcode_storage_filename",
            namespace,
        )
        convert = self.compile_function("GCconvertProg", namespace)

        convert()

        self.assertEqual(serial_port.commands, [b"DGFnoutput.txt\n"])
        self.assertEqual(program_view.selection, 1)
        self.assertEqual(program_view.selection_mutations, [])
        self.assertEqual(tab.GCrunTrue, 0)
        self.assertEqual(runtime["GCrowinproc"], 0)

        stopped_view = ProgramView()
        stopped_tab = SimpleNamespace(gcodeView=stopped_view, GCrunTrue=0)
        stopped_runtime = {"ser": SerialPort(), "GCrowinproc": 0}

        def stop_during_pending_row():
            stopped_tab.GCrunTrue = 0
            return "pending"

        stopped_namespace = dict(namespace)
        stopped_namespace.update(
            {
                "RUN": stopped_runtime,
                "tab7": stopped_tab,
                "GCexecuteRow": stop_during_pending_row,
                "GcodCurRowEntryField": Entry(),
                "cmdSentEntryField": Entry(),
            }
        )
        stopped_convert = self.compile_function(
            "GCconvertProg",
            stopped_namespace,
        )

        stopped_convert()

        self.assertEqual(stopped_view.selection, 1)
        self.assertEqual(stopped_view.selection_mutations, [])
        self.assertEqual(stopped_runtime["GCrowinproc"], 0)

        lock_stopped_view = ProgramView()
        lock_stopped_tab = SimpleNamespace(
            gcodeView=lock_stopped_view,
            GCrunTrue=0,
        )
        lock_stopped_runtime = {"ser": SerialPort(), "GCrowinproc": 0}

        class StopOnLockCheck:
            @staticmethod
            def locked():
                lock_stopped_tab.GCrunTrue = 0
                return True

        lock_stopped_namespace = dict(namespace)
        lock_stopped_namespace.update(
            {
                "RUN": lock_stopped_runtime,
                "tab7": lock_stopped_tab,
                "serial_lock": StopOnLockCheck(),
                "GCexecuteRow": lambda: (_ for _ in ()).throw(
                    AssertionError("stopped conversion must not execute a row")
                ),
                "GcodCurRowEntryField": Entry(),
                "cmdSentEntryField": Entry(),
            }
        )
        lock_stopped_convert = self.compile_function(
            "GCconvertProg",
            lock_stopped_namespace,
        )

        lock_stopped_convert()

        self.assertEqual(lock_stopped_view.selection, 1)
        self.assertEqual(lock_stopped_view.selection_mutations, [])
        self.assertEqual(lock_stopped_runtime["GCrowinproc"], 0)

    def test_gcode_step_rejection_does_not_advance_selection(self):
        program_view = SimpleNamespace(
            curselection=lambda: (3,),
            index=lambda value: 5,
            itemconfig=lambda *args: None,
            selection_clear=lambda *args: (_ for _ in ()).throw(
                AssertionError("rejected row must not clear selection")
            ),
            select_set=lambda *args: (_ for _ in ()).throw(
                AssertionError("rejected row must not advance selection")
            ),
        )
        namespace = {
            "GCalmStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "GCexecuteRow": lambda: "rejected",
            "ROW_EXECUTION_COMPLETE": "complete",
            "tab7": SimpleNamespace(gcodeView=program_view),
        }
        step = self.compile_function("GCstepFwd", namespace)

        self.assertFalse(step())

    def test_gcode_row_exchange_rechecks_local_stop_under_the_write_lock(self):
        serial_port = SimpleNamespace(is_open=True)
        runtime = {"ser": serial_port}
        tab = SimpleNamespace(GCrunTrue=0)
        write_lock = threading.Lock()
        commands = []
        reads = []
        namespace = {
            "_controller_response_timeout": lambda command: 12.0,
            "RUN": runtime,
            "serial_write_lock": write_lock,
            "tab7": tab,
            "write_serial_control": (
                lambda port, command, **kwargs: commands.append(command)
            ),
            "read_serial_line_response": (
                lambda port, timeout: reads.append(timeout) or "position"
            ),
            "serial_transport_quarantined": lambda port: False,
        }
        exchange = self.compile_function("_exchange_gcode_row", namespace)
        command = (
            "WCX1Y2Z3Rz4Ry5Rx6J70J80J90"
            "Sp50Ac10Dc20Rm25WNLm000000Fndemo.txt\n"
        )

        self.assertIsNone(exchange(command))
        self.assertEqual(commands, [])
        self.assertEqual(reads, [])
        self.assertFalse(write_lock.locked())

        tab.GCrunTrue = 1
        self.assertEqual(exchange(command), "position")
        self.assertEqual(commands, [command])
        self.assertEqual(reads, [12.0])
        self.assertFalse(write_lock.locked())

    def test_reverse_step_supplies_an_async_motion_completion(self):
        reverse_step = self.module_functions["stepRev"]
        execute_calls = [
            node
            for node in ast.walk(reverse_step)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "executeRow"
        ]
        self.assertEqual(len(execute_calls), 1)
        completion_keywords = [
            keyword
            for keyword in execute_calls[0].keywords
            if keyword.arg == "motion_complete"
        ]
        self.assertEqual(len(completion_keywords), 1)

        wait_calls = [
            node
            for node in ast.walk(reverse_step)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "wait_until_all_locks_free"
        ]
        self.assertEqual(wait_calls, [])

    def test_manual_cartesian_and_tool_jogs_use_virtual_first_owner(self):
        cartesian = (
            "XjogNeg", "YjogNeg", "ZjogNeg", "RxjogNeg", "RyjogNeg", "RzjogNeg",
            "XjogPos", "YjogPos", "ZjogPos", "RxjogPos", "RyjogPos", "RzjogPos",
        )
        tool = (
            "TXjogNeg", "TYjogNeg", "TZjogNeg", "TRxjogNeg", "TRyjogNeg", "TRzjogNeg",
            "TXjogPos", "TYjogPos", "TZjogPos", "TRxjogPos", "TRyjogPos", "TRzjogPos",
        )
        for name, virtual_dispatch in (
            *((name, "mj_command") for name in cartesian),
            *((name, "mt_command") for name in tool),
        ):
            function = self.module_functions[name]
            manual_calls = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_start_manual_motion"
            ]
            legacy_calls = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_start_legacy_motion"
            ]
            self.assertEqual(len(function.decorator_list), 1, name)
            decorator = function.decorator_list[0]
            self.assertIsInstance(decorator, ast.Call, name)
            self.assertIsInstance(decorator.func, ast.Name, name)
            self.assertEqual(decorator.func.id, "_manual_motion_request", name)
            self.assertEqual(len(manual_calls), 2, name)
            self.assertEqual(legacy_calls, [], name)
            for call in manual_calls:
                self.assertEqual(len(call.args), 4, name)
                self.assertIsInstance(call.args[2], ast.Name, name)
                self.assertEqual(call.args[2].id, virtual_dispatch, name)

        helper = self.module_functions["_start_manual_motion"]
        helper_calls = [
            node
            for node in ast.walk(helper)
            if isinstance(node, ast.Call)
        ]
        virtual_calls = [
            call
            for call in helper_calls
            if isinstance(call.func, ast.Name)
            and call.func.id == "virtual_dispatch"
        ]
        physical_calls = [
            call
            for call in helper_calls
            if isinstance(call.func, ast.Name)
            and call.func.id == "_start_legacy_motion"
        ]
        self.assertEqual(len(virtual_calls), 1)
        self.assertEqual(len(physical_calls), 1)
        self.assertLess(virtual_calls[0].lineno, physical_calls[0].lineno)

    def test_manual_motion_rejects_physical_virtual_wrist_mismatch(self):
        virtual_calls = []
        namespace = {
            "wraps": wraps,
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        start_manual = self.compile_function("_start_manual_motion", namespace)
        manual_guard = self.compile_function("_manual_motion_request", namespace)
        physical = CONTROLLER_CARTESIAN_TEST_COMMAND.replace("WN", "WA")
        guarded = manual_guard("Cartesian jog")(
            lambda: start_manual(
                physical,
                "Cartesian jog",
                lambda command: virtual_calls.append(command),
                VIRTUAL_CARTESIAN_TEST_COMMAND,
            )
        )

        self.assertFalse(guarded())
        self.assertEqual(virtual_calls, [])
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_offline_cartesian_failure_restores_cal_before_owner_release(self):
        class Entry:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        jobs = []
        operation = VirtualMotionOperation()
        saved_pose = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        partial_pose = (21.0, 22.0, 23.0, 24.0, 25.0, 26.0)
        confirmed_cartesian = {
            "XcurPos": "10.000",
            "YcurPos": "20.000",
            "ZcurPos": "30.000",
            "RzcurPos": "0.000",
            "RycurPos": "0.000",
            "RxcurPos": "0.000",
        }
        calibration = {
            **confirmed_cartesian,
            "J7PosCur": 0,
            "J8PosCur": 0,
            "J9PosCur": 0,
            **{
                f"J{axis}OpenLoopVal": SimpleNamespace(get=lambda: 0)
                for axis in range(1, 7)
            },
        }
        runtime = {
            "offlineMode": True,
            "xboxUse": 1,
            "VR_angles": list(saved_pose),
            "WC": "N",
        }
        settlement_ownership = []
        namespace = {
            "wraps": wraps,
            "threading": threading,
            "math": math,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "RUN": runtime,
            "CAL": calibration,
            "robot": SimpleNamespace(
                forward_kinematics=lambda joints: [10.0, 20.0, 30.0, 0.0, 0.0, 0.0]
            ),
            "checkSpeedVals": lambda: None,
            "speedOption": Entry("Percent"),
            "speedEntryField": Entry("50"),
            "ACCspeedField": Entry("10"),
            "DECspeedField": Entry("20"),
            "ACCrampField": Entry("25"),
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append(callback)
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        self.add_virtual_completion_timeout(namespace)

        def set_virtual(target):
            runtime["VR_angles"] = list(target)
            return True

        def refresh(target):
            settlement_ownership.append(
                namespace["motion_request_registry"].active
            )
            runtime["VR_angles"] = list(target)
            calibration.update(confirmed_cartesian)
            return True

        def start_virtual(command):
            runtime["VR_angles"] = list(partial_pose)
            return operation

        namespace["_try_set_virtual_joint_target"] = set_virtual
        namespace["refresh_gui_from_joint_angles"] = refresh
        namespace["mj_command"] = start_virtual
        namespace["_complete_program_motion_when_virtual_idle"] = (
            self.compile_function(
                "_complete_program_motion_when_virtual_idle",
                namespace,
            )
        )
        namespace["_start_manual_motion"] = self.compile_function(
            "_start_manual_motion",
            namespace,
        )
        namespace["_manual_motion_request"] = self.compile_function(
            "_manual_motion_request",
            namespace,
        )
        jog = self.compile_function(
            "XjogNeg",
            namespace,
            preserve_decorators=True,
        )

        self.assertTrue(jog(1.0))
        self.assertTrue(namespace["motion_request_registry"].active)
        self.assertEqual(calibration["XcurPos"], 9.0)
        self.assertEqual(runtime["VR_angles"], list(partial_pose))
        self.assertEqual(len(jobs), 1)

        operation.complete(False, "Cartesian preview failed")
        jobs.pop()()

        self.assertEqual(settlement_ownership, [True])
        self.assertEqual(runtime["VR_angles"], list(saved_pose))
        self.assertEqual(
            {key: calibration[key] for key in confirmed_cartesian},
            confirmed_cartesian,
        )
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_manual_motion_retains_owner_after_controller_first_result(self):
        jobs = []
        order = []
        controller_callbacks = []
        operation = VirtualMotionOperation()
        controller_positions = tuple(float(axis) for axis in range(1, 10))
        namespace = {
            "wraps": wraps,
            "threading": threading,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "RUN": {
                "offlineMode": False,
                "VR_angles": list(controller_positions[:6]),
            },
            "_current_joint_positions": lambda: controller_positions,
            "joint_motion_dispatcher": SimpleNamespace(
                synchronize=lambda positions: True
            ),
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append(callback)
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        self.add_virtual_completion_timeout(namespace)

        def set_virtual(target):
            namespace["RUN"]["VR_angles"] = list(target)
            return True

        namespace["_try_set_virtual_joint_target"] = set_virtual

        def start_physical(
            command,
            motion_name,
            completion_callback=None,
            request_lease=None,
            write_started_event=None,
        ):
            self.assertTrue(
                namespace["motion_request_registry"].owns(request_lease)
            )
            order.append("physical")
            write_started_event.set()
            controller_callbacks.append(completion_callback)
            return True

        namespace["_start_legacy_motion"] = start_physical
        namespace["_complete_program_motion_when_virtual_idle"] = (
            self.compile_function(
                "_complete_program_motion_when_virtual_idle",
                namespace,
            )
        )
        start_manual = self.compile_function("_start_manual_motion", namespace)
        manual_guard = self.compile_function("_manual_motion_request", namespace)

        def start_virtual(command):
            order.append("virtual")
            return operation

        guarded = manual_guard("Cartesian jog")(
            lambda: start_manual(
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                "Cartesian jog",
                start_virtual,
                VIRTUAL_CARTESIAN_TEST_COMMAND,
            )
        )

        self.assertTrue(guarded())
        self.assertEqual(order, ["virtual", "physical"])
        self.assertTrue(namespace["motion_request_registry"].active)

        controller_callbacks[0](VALID_CONTROLLER_POSITION)
        self.assertTrue(namespace["motion_request_registry"].active)
        self.assertEqual(len(jobs), 1)

        operation.complete(True)
        jobs.pop()()
        self.assertFalse(namespace["motion_request_registry"].active)

        rejected = manual_guard("Cartesian jog")(
            lambda: start_manual(
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                "Cartesian jog",
                lambda command: False,
                VIRTUAL_CARTESIAN_TEST_COMMAND,
            )
        )
        self.assertFalse(rejected())
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_manual_speed_violation_preserves_controller_warning(self):
        warning = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
        controller_callbacks = []
        controller_positions = (
            VALID_CONTROLLER_POSITION.joints
            + VALID_CONTROLLER_POSITION.external
        )

        class Label:
            def __init__(self):
                self.text = warning

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        first_label = Label()
        second_label = Label()
        namespace = {
            "wraps": wraps,
            "threading": threading,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "RUN": {
                "offlineMode": False,
                "VR_angles": list(controller_positions[:6]),
            },
            "_current_joint_positions": lambda: controller_positions,
            "joint_motion_dispatcher": SimpleNamespace(
                synchronize=lambda positions: True
            ),
            "_try_set_virtual_joint_target": lambda target: True,
            "_complete_program_motion_when_virtual_idle": (
                complete_virtual_callback
            ),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
        }
        self.add_virtual_completion_timeout(namespace)

        def start_physical(
            command,
            motion_name,
            completion_callback=None,
            request_lease=None,
            write_started_event=None,
        ):
            write_started_event.set()
            controller_callbacks.append(completion_callback)
            return True

        namespace["_start_legacy_motion"] = start_physical
        start_manual = self.compile_function("_start_manual_motion", namespace)
        manual_guard = self.compile_function("_manual_motion_request", namespace)
        guarded = manual_guard("Cartesian jog")(
            lambda: start_manual(
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                "Cartesian jog",
                lambda command: completed_virtual_operation(),
                VIRTUAL_CARTESIAN_TEST_COMMAND,
            )
        )

        self.assertTrue(guarded())
        controller_callbacks[0](SPEED_VIOLATION_CONTROLLER_POSITION)

        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertFalse(namespace["manual_motion_pose_pending"].is_set())
        self.assertEqual(first_label.text, warning)
        self.assertEqual(second_label.text, warning)

    def test_manual_motion_restores_preview_when_controller_never_transmits(self):
        jobs = []
        operation = VirtualMotionOperation()
        controller_positions = tuple(float(axis) for axis in range(1, 10))
        synchronized = []
        namespace = {
            "wraps": wraps,
            "threading": threading,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "RUN": {
                "offlineMode": False,
                "VR_angles": list(controller_positions[:6]),
            },
            "_current_joint_positions": lambda: controller_positions,
            "joint_motion_dispatcher": SimpleNamespace(
                synchronize=lambda positions: synchronized.append(tuple(positions)) or True
            ),
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append(callback)
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        self.add_virtual_completion_timeout(namespace)

        def set_virtual(target):
            namespace["RUN"]["VR_angles"] = list(target)
            return True

        def start_virtual(command):
            namespace["RUN"]["VR_angles"] = [90.0] * 6
            return operation

        def reject_physical(
            command,
            motion_name,
            completion_callback=None,
            request_lease=None,
            write_started_event=None,
        ):
            self.assertFalse(write_started_event.is_set())
            return False

        namespace["_try_set_virtual_joint_target"] = set_virtual
        namespace["_start_legacy_motion"] = reject_physical
        namespace["_complete_program_motion_when_virtual_idle"] = (
            self.compile_function(
                "_complete_program_motion_when_virtual_idle",
                namespace,
            )
        )
        start_manual = self.compile_function("_start_manual_motion", namespace)
        manual_guard = self.compile_function("_manual_motion_request", namespace)
        guarded = manual_guard("Cartesian jog")(
            lambda: start_manual(
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                "Cartesian jog",
                start_virtual,
                VIRTUAL_CARTESIAN_TEST_COMMAND,
            )
        )

        self.assertFalse(guarded())
        self.assertTrue(namespace["motion_request_registry"].active)
        self.assertTrue(namespace["manual_motion_pose_pending"].is_set())
        self.assertEqual(namespace["RUN"]["VR_angles"], [90.0] * 6)

        operation.complete(True)
        jobs.pop()()

        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertFalse(namespace["manual_motion_pose_pending"].is_set())
        self.assertFalse(
            namespace["controller_position_resynchronization_required"].is_set()
        )
        self.assertEqual(
            namespace["RUN"]["VR_angles"],
            list(controller_positions[:6]),
        )
        self.assertEqual(synchronized, [controller_positions])

    def test_manual_motion_blocks_new_motion_after_transmitted_failure(self):
        operation = completed_virtual_operation()
        controller_callbacks = []
        controller_positions = tuple(float(axis) for axis in range(1, 10))
        synchronized = []
        namespace = {
            "wraps": wraps,
            "threading": threading,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "RUN": {
                "offlineMode": False,
                "VR_angles": list(controller_positions[:6]),
            },
            "_current_joint_positions": lambda: controller_positions,
            "joint_motion_dispatcher": SimpleNamespace(
                synchronize=lambda positions: synchronized.append(tuple(positions)) or True
            ),
            "root": SimpleNamespace(after=lambda *args: None),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
            "_try_set_virtual_joint_target": lambda target: self.fail(
                "uncertain transmitted motion must not restore a saved pose"
            ),
        }
        self.add_virtual_completion_timeout(namespace)

        def start_virtual(command):
            namespace["RUN"]["VR_angles"] = [75.0] * 6
            return operation

        def start_physical(
            command,
            motion_name,
            completion_callback=None,
            request_lease=None,
            write_started_event=None,
        ):
            write_started_event.set()
            controller_callbacks.append(completion_callback)
            return True

        namespace["_start_legacy_motion"] = start_physical
        namespace["_complete_program_motion_when_virtual_idle"] = (
            self.compile_function(
                "_complete_program_motion_when_virtual_idle",
                namespace,
            )
        )
        start_manual = self.compile_function("_start_manual_motion", namespace)
        manual_guard = self.compile_function("_manual_motion_request", namespace)
        guarded = manual_guard("Cartesian jog")(
            lambda: start_manual(
                CONTROLLER_CARTESIAN_TEST_COMMAND,
                "Cartesian jog",
                start_virtual,
                VIRTUAL_CARTESIAN_TEST_COMMAND,
            )
        )

        self.assertTrue(guarded())
        controller_callbacks[0](None)

        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertFalse(namespace["manual_motion_pose_pending"].is_set())
        self.assertTrue(
            namespace["controller_position_resynchronization_required"].is_set()
        )
        self.assertEqual(namespace["RUN"]["VR_angles"], [75.0] * 6)
        self.assertEqual(synchronized, [])

        acquire = self.compile_function("_acquire_motion_request", namespace)
        self.assertIsNone(acquire("Joint motion"))
        recovery_lease = acquire(
            "Controller correction",
            allow_position_recovery=True,
        )
        self.assertIsInstance(recovery_lease, MotionRequestLease)
        recovery_lease.close()

    def test_manual_motion_resynchronizes_after_virtual_terminal_failure(self):
        jobs = []
        operation = VirtualMotionOperation()
        controller_callbacks = []
        controller_positions = tuple(float(axis) for axis in range(1, 10))
        synchronized = []
        namespace = {
            "wraps": wraps,
            "threading": threading,
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "MotionQueueFault": MotionQueueFault,
            "VIRTUAL_COMPLETION_POLL_MS": 1,
            "RUN": {
                "offlineMode": False,
                "VR_angles": list(controller_positions[:6]),
            },
            "_current_joint_positions": lambda: controller_positions,
            "joint_motion_dispatcher": SimpleNamespace(
                synchronize=lambda positions: synchronized.append(tuple(positions)) or True
            ),
            "root": SimpleNamespace(
                after=lambda delay, callback: jobs.append(callback)
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        self.add_virtual_completion_timeout(namespace)

        def set_virtual(target):
            namespace["RUN"]["VR_angles"] = list(target)
            return True

        def start_virtual(command):
            namespace["RUN"]["VR_angles"] = [60.0] * 6
            return operation

        def start_physical(
            command,
            motion_name,
            completion_callback=None,
            request_lease=None,
            write_started_event=None,
        ):
            write_started_event.set()
            controller_callbacks.append(completion_callback)
            return True

        namespace["_try_set_virtual_joint_target"] = set_virtual
        namespace["_start_legacy_motion"] = start_physical
        namespace["_complete_program_motion_when_virtual_idle"] = (
            self.compile_function(
                "_complete_program_motion_when_virtual_idle",
                namespace,
            )
        )
        start_manual = self.compile_function("_start_manual_motion", namespace)
        manual_guard = self.compile_function("_manual_motion_request", namespace)
        guarded = manual_guard("Tool-frame jog")(
            lambda: start_manual(
                VIRTUAL_TOOL_TEST_COMMAND,
                "Tool-frame jog",
                start_virtual,
                VIRTUAL_TOOL_TEST_COMMAND,
            )
        )

        self.assertTrue(guarded())
        controller_callbacks[0](VALID_CONTROLLER_POSITION)
        operation.complete(False, "preview failed")
        jobs.pop()()

        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertFalse(namespace["manual_motion_pose_pending"].is_set())
        self.assertFalse(
            namespace["controller_position_resynchronization_required"].is_set()
        )
        self.assertEqual(
            namespace["RUN"]["VR_angles"],
            list(controller_positions[:6]),
        )
        self.assertEqual(synchronized, [controller_positions])

    def test_joint_dispatcher_uses_shared_transport_reservation(self):
        assignments = [
            node
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "joint_motion_dispatcher"
                for target in node.targets
            )
        ]

        self.assertEqual(len(assignments), 1)
        constructor = assignments[0].value
        self.assertIsInstance(constructor, ast.Call)
        self.assertGreaterEqual(len(constructor.args), 2)
        self.assertIsInstance(constructor.args[1], ast.Name)
        self.assertEqual(
            constructor.args[1].id,
            "_current_controller_joint_calibration",
        )
        transport_keywords = [
            keyword
            for keyword in constructor.keywords
            if keyword.arg == "transport_lock"
        ]
        self.assertEqual(len(transport_keywords), 1)
        self.assertIsInstance(transport_keywords[0].value, ast.Name)
        self.assertEqual(transport_keywords[0].value.id, "serial_lock")

    def test_joint_result_poll_acknowledges_transport_owner(self):
        poller = self.module_functions["_poll_joint_motion_events"]
        acknowledgement_calls = [
            node
            for node in ast.walk(poller)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "acknowledge"
        ]

        self.assertEqual(len(acknowledgement_calls), 1)

    def test_joint_completion_reports_ready_or_retained_pending_target(self):
        class Event:
            kind = "completed"
            response = "position"
            position = VALID_CONTROLLER_POSITION

            def __init__(self):
                self.acknowledged = False

            def acknowledge(self):
                self.acknowledged = True

        class Dispatcher:
            def __init__(self):
                self.pending = False
                self.events = []

            def drain_events(self):
                events = self.events
                self.events = []
                return events

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        class Closing:
            @staticmethod
            def is_set():
                return True

        dispatcher = Dispatcher()
        first_label = Label()
        second_label = Label()
        display_calls = []
        namespace = {
            "joint_motion_dispatcher": dispatcher,
            "displayPosition": lambda *args, **kwargs: display_calls.append(
                (args, kwargs)
            ) or VALID_CONTROLLER_POSITION,
            "_set_virtual_from_joint_result": lambda position: True,
            "_try_dispatch_deferred_joint_adjustments": lambda: False,
            "application_closing": Closing(),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
        }
        poll = self.compile_function("_poll_joint_motion_events", namespace)

        ready_event = Event()
        dispatcher.events = [ready_event]
        poll()
        self.assertTrue(ready_event.acknowledged)
        self.assertEqual(first_label.text, "SYSTEM READY")
        self.assertEqual(second_label.text, first_label.text)
        self.assertIs(
            display_calls[-1][1]["synchronize_dispatcher"],
            False,
        )

        queued_event = Event()
        dispatcher.pending = True
        dispatcher.events = [queued_event]
        poll()
        self.assertTrue(queued_event.acknowledged)
        self.assertEqual(first_label.text, "JOINT TARGET QUEUED")

    def test_joint_completion_invalidates_before_failed_virtual_sync_ack(self):
        sequence = []

        class Event:
            kind = "completed"
            response = "position"
            position = VALID_CONTROLLER_POSITION

            @staticmethod
            def acknowledge():
                sequence.append("acknowledged")

        class Dispatcher:
            pending = True

            def __init__(self):
                self.events = [Event()]

            def drain_events(self):
                events = self.events
                self.events = []
                return events

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        dispatcher = Dispatcher()
        first_label = Label()
        second_label = Label()

        def fail_virtual_sync(position):
            first_label.config(text="virtual synchronization failed")
            second_label.config(text="virtual synchronization failed")
            return False

        def invalidate(reason):
            sequence.append(("invalidated", reason))
            dispatcher.pending = False

        namespace = {
            "joint_motion_dispatcher": dispatcher,
            "displayPosition": lambda *args, **kwargs: VALID_CONTROLLER_POSITION,
            "_set_virtual_from_joint_result": fail_virtual_sync,
            "_invalidate_joint_motion_state": invalidate,
            "application_closing": SimpleNamespace(is_set=lambda: True),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
        }
        poll = self.compile_function("_poll_joint_motion_events", namespace)

        poll()

        self.assertEqual(
            sequence,
            [
                (
                    "invalidated",
                    "completed joint motion could not update the virtual model",
                ),
                "acknowledged",
            ],
        )
        self.assertFalse(dispatcher.pending)
        self.assertEqual(first_label.text, "virtual synchronization failed")
        self.assertEqual(second_label.text, first_label.text)

    def test_joint_speed_violation_discards_queued_and_deferred_targets(self):
        warning = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
        sequence = []

        class Event:
            kind = "completed"
            response = SPEED_VIOLATION_CONTROLLER_POSITION.raw
            position = SPEED_VIOLATION_CONTROLLER_POSITION

            @staticmethod
            def acknowledge():
                sequence.append("acknowledged")

        class Dispatcher:
            active = True

            def __init__(self):
                self.pending = True
                self.desired_target = tuple(range(91, 100))
                self.events = [Event()]

            def drain_events(self):
                events = self.events
                self.events = []
                return events

            def discard_pending_after_completion(self, positions):
                sequence.append(("discarded", tuple(positions)))
                discarded = self.pending
                self.pending = False
                self.desired_target = tuple(positions)
                return discarded

        class Label:
            def __init__(self):
                self.text = None

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        dispatcher = Dispatcher()
        first_label = Label()
        second_label = Label()
        deferred = SimpleNamespace(pending=True)
        deferred_attempts = []
        runtime = {
            "VR_angles": list(dispatcher.desired_target[:6]),
            "StepMonitors": None,
        }

        def display_position(*args, **kwargs):
            first_label.config(text=warning)
            second_label.config(text=warning)
            return SPEED_VIOLATION_CONTROLLER_POSITION

        def clear_deferred():
            sequence.append("deferred-cleared")
            deferred.pending = False

        namespace = {
            "RUN": runtime,
            "joint_motion_dispatcher": dispatcher,
            "deferred_joint_adjustments": deferred,
            "displayPosition": display_position,
            "_clear_deferred_joint_adjustments": clear_deferred,
            "_try_dispatch_deferred_joint_adjustments": (
                lambda: deferred_attempts.append(deferred.pending) or False
            ),
            "application_closing": SimpleNamespace(is_set=lambda: True),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "MotionInputError": MotionInputError,
            "finite_number": finite_number,
            "math": math,
            "_invalidate_joint_motion_state": lambda reason: sequence.append(
                ("invalidated", reason)
            ),
            "setStepMonitorsVR": lambda: runtime.__setitem__(
                "StepMonitors",
                tuple(runtime["VR_angles"]),
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: sequence.append("warning"),
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        for function_name in (
            "_set_virtual_joint_target",
            "_try_set_virtual_joint_target",
            "_set_virtual_from_joint_result",
        ):
            namespace[function_name] = self.compile_function(
                function_name,
                namespace,
            )
        poll = self.compile_function("_poll_joint_motion_events", namespace)

        poll()

        expected_positions = (
            SPEED_VIOLATION_CONTROLLER_POSITION.joints
            + SPEED_VIOLATION_CONTROLLER_POSITION.external
        )
        self.assertIn(("discarded", expected_positions), sequence)
        self.assertIn("deferred-cleared", sequence)
        self.assertEqual(sequence[-1], "acknowledged")
        self.assertFalse(dispatcher.pending)
        self.assertFalse(deferred.pending)
        self.assertEqual(
            runtime["VR_angles"],
            list(SPEED_VIOLATION_CONTROLLER_POSITION.joints),
        )
        self.assertEqual(
            runtime["StepMonitors"],
            SPEED_VIOLATION_CONTROLLER_POSITION.joints,
        )
        self.assertEqual(deferred_attempts, [False])
        self.assertEqual(first_label.text, warning)
        self.assertEqual(second_label.text, warning)

    def test_live_completion_clears_stop_state_and_reports_ready(self):
        class Flag:
            def __init__(self, initial=True):
                self.value = initial

            def clear(self):
                self.value = False

            def is_set(self):
                return self.value

            def set(self):
                self.value = True

        class Widget:
            def __init__(self):
                self.value = None

            def delete(self, *args):
                self.value = None

            def insert(self, index, value):
                self.value = value

            def config(self, **kwargs):
                self.value = kwargs.get("text")

        class Closing:
            @staticmethod
            def is_set():
                return True

        class Logger:
            def __init__(self):
                self.exceptions = []

            def exception(self, message):
                self.exceptions.append(message)

        event_queue = Queue()
        event_queue.put(("started", "LJV10\n", None, None, True, None, None))
        event_queue.put(
            (
                "completed",
                "LJV10\n",
                "position",
                None,
                True,
                None,
                None,
            )
        )
        first_label = Widget()
        second_label = Widget()
        legacy_pending = Flag()
        live_pending = Flag()
        live_stop = Flag()
        serial_lock = threading.Lock()
        serial_lock.acquire()
        monitor_updates = []
        deferred_attempts = []
        logger = Logger()
        namespace = {
            "serial_event_queue": event_queue,
            "Empty": Empty,
            "cmdSentEntryField": Widget(),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "logger": logger,
            "_apply_legacy_serial_response": (
                lambda response: VALID_CONTROLLER_POSITION
            ),
            "_invalidate_joint_motion_state": lambda reason: None,
            "RUN": {"liveJog": True},
            "CAL": {
                f"J{axis}AngCur": str(axis)
                for axis in range(1, 7)
            },
            "finite_number": finite_number,
            "setStepMonitorsVR": lambda: monitor_updates.append(True),
            "legacy_serial_result_pending": legacy_pending,
            "live_serial_result_pending": live_pending,
            "live_jog_stop_requested": live_stop,
            "_try_dispatch_deferred_joint_adjustments": lambda **kwargs: deferred_attempts.append(kwargs),
            "serial_lock": serial_lock,
            "joint_motion_dispatcher": SimpleNamespace(active=False),
            "application_closing": Closing(),
        }
        poll = self.compile_function("_poll_serial_events", namespace)

        poll()

        self.assertEqual(first_label.value, "SYSTEM READY")
        self.assertEqual(second_label.value, first_label.value)
        self.assertEqual(namespace["RUN"]["VR_angles"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertFalse(namespace["RUN"]["liveJog"])
        self.assertFalse(legacy_pending.is_set())
        self.assertFalse(live_pending.is_set())
        self.assertFalse(live_stop.is_set())
        self.assertFalse(serial_lock.locked())
        self.assertEqual(monitor_updates, [True])
        self.assertEqual(
            deferred_attempts,
            [{"allow_current_generation": True}],
        )
        self.assertEqual(logger.exceptions, [])

    def test_serial_speed_violation_preserves_warning_for_live_and_generic_results(self):
        warning = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"

        class Widget:
            def __init__(self):
                self.value = None

            def delete(self, *args):
                self.value = None

            def insert(self, index, value):
                self.value = value

            def config(self, **kwargs):
                self.value = kwargs.get("text")

        for live_jog in (False, True):
            with self.subTest(live_jog=live_jog):
                first_label = Widget()
                second_label = Widget()
                serial_lock = threading.Lock()
                serial_lock.acquire()
                event_queue = Queue()
                completion_results = []
                event_queue.put(
                    (
                        "completed",
                        "motion\n",
                        SPEED_VIOLATION_CONTROLLER_POSITION.raw,
                        None,
                        live_jog,
                        completion_results.append,
                        None,
                    )
                )
                deferred = SimpleNamespace(pending=True)
                deferred_attempts = []
                cleared = []
                monitor_updates = []

                def apply_result(response):
                    first_label.config(text=warning)
                    second_label.config(text=warning)
                    return SPEED_VIOLATION_CONTROLLER_POSITION

                def clear_deferred():
                    cleared.append(True)
                    deferred.pending = False

                namespace = {
                    "serial_event_queue": event_queue,
                    "Empty": Empty,
                    "cmdSentEntryField": Widget(),
                    "almStatusLab": first_label,
                    "almStatusLab2": second_label,
                    "logger": SimpleNamespace(
                        warning=lambda *args: None,
                        error=lambda *args: None,
                        exception=lambda *args: None,
                    ),
                    "_apply_legacy_serial_response": apply_result,
                    "_invalidate_joint_motion_state": lambda reason: None,
                    "RUN": {"liveJog": live_jog, "VR_angles": [0.0] * 6},
                    "CAL": {
                        f"J{axis}AngCur": str(axis)
                        for axis in range(1, 7)
                    },
                    "finite_number": finite_number,
                    "setStepMonitorsVR": (
                        lambda: monitor_updates.append(True)
                    ),
                    "legacy_serial_result_pending": threading.Event(),
                    "live_serial_result_pending": threading.Event(),
                    "live_jog_stop_requested": threading.Event(),
                    "serial_lock": serial_lock,
                    "deferred_joint_adjustments": deferred,
                    "_clear_deferred_joint_adjustments": clear_deferred,
                    "_try_dispatch_deferred_joint_adjustments": (
                        lambda **kwargs: deferred_attempts.append(kwargs)
                    ),
                    "joint_motion_dispatcher": SimpleNamespace(active=False),
                    "application_closing": SimpleNamespace(is_set=lambda: True),
                }
                poll = self.compile_function("_poll_serial_events", namespace)

                poll()

                self.assertEqual(
                    completion_results,
                    [SPEED_VIOLATION_CONTROLLER_POSITION],
                )
                self.assertEqual(first_label.value, warning)
                self.assertEqual(second_label.value, warning)
                self.assertEqual(cleared, [True])
                self.assertFalse(deferred.pending)
                self.assertEqual(deferred_attempts, [])
                self.assertFalse(serial_lock.locked())
                self.assertEqual(monitor_updates, [True] if live_jog else [])

    def test_legacy_result_application_precedes_transport_release(self):
        completion_enqueued = threading.Event()

        class SignalingQueue(Queue):
            def put(self, item, *args, **kwargs):
                super().put(item, *args, **kwargs)
                if item[0] in ("completed", "failed"):
                    completion_enqueued.set()

        class Widget:
            def __init__(self):
                self.text = None

            def delete(self, *args):
                pass

            def insert(self, *args):
                pass

            def config(self, **kwargs):
                self.text = kwargs.get("text")

        lock = threading.Lock()
        lock.acquire()
        event_queue = SignalingQueue()
        legacy_pending = threading.Event()
        legacy_pending.set()
        deferred_attempts = []
        completions = []
        application_closing = threading.Event()
        first_label = Widget()
        second_label = Widget()
        namespace = {
            "threading": threading,
            "serial_event_queue": event_queue,
            "_exchange_serial_line": (
                lambda command, control_event=None, write_started_event=None: "position"
            ),
            "live_jog_stop_requested": threading.Event(),
            "live_serial_result_pending": threading.Event(),
            "legacy_serial_result_pending": legacy_pending,
            "application_closing": application_closing,
            "serial_lock": lock,
            "Empty": Empty,
            "cmdSentEntryField": Widget(),
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "_apply_legacy_serial_response": (
                lambda response: VALID_CONTROLLER_POSITION
            ),
            "_invalidate_joint_motion_state": lambda reason: None,
            "RUN": {"liveJog": False},
            "_try_dispatch_deferred_joint_adjustments": lambda **kwargs: deferred_attempts.append(kwargs),
            "joint_motion_dispatcher": SimpleNamespace(active=False),
            "root": SimpleNamespace(after=lambda *args: None),
        }
        run_worker = self.compile_function("run_send_serial_safe", namespace)
        poll_events = self.compile_function("_poll_serial_events", namespace)
        worker = threading.Thread(
            target=run_worker,
            args=("RJ\n", False, completions.append),
        )
        worker.start()
        self.assertTrue(completion_enqueued.wait(2))
        worker.join(2)
        self.assertFalse(worker.is_alive())

        self.assertTrue(lock.locked())
        self.assertEqual(deferred_attempts, [])
        self.assertEqual(completions, [])
        self.assertTrue(legacy_pending.is_set())

        poll_events()

        self.assertFalse(lock.locked())
        self.assertFalse(legacy_pending.is_set())
        self.assertEqual(completions, [VALID_CONTROLLER_POSITION])
        self.assertEqual(first_label.text, "SYSTEM READY")
        self.assertEqual(second_label.text, first_label.text)
        self.assertEqual(
            deferred_attempts,
            [{"allow_current_generation": True}],
        )

    def test_legacy_callbacks_remain_scoped_to_back_to_back_results(self):
        class Widget:
            def delete(self, *args):
                pass

            def insert(self, *args):
                pass

            def config(self, **kwargs):
                pass

        event_queue = Queue()
        transport_lock = threading.Lock()
        pending = threading.Event()
        first_results = []
        second_results = []
        resynchronization_required = threading.Event()

        class Dispatcher:
            active = False

            def __init__(self):
                self.invalidations = []

            def invalidate(self, reason):
                self.invalidations.append(reason)
                return False

        dispatcher = Dispatcher()
        namespace = {
            "serial_event_queue": event_queue,
            "Empty": Empty,
            "cmdSentEntryField": Widget(),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "_apply_legacy_serial_response": (
                lambda response: (
                    VALID_CONTROLLER_POSITION if response == "first" else None
                )
            ),
            "RUN": {"liveJog": False},
            "legacy_serial_result_pending": pending,
            "live_serial_result_pending": threading.Event(),
            "live_jog_stop_requested": threading.Event(),
            "serial_lock": transport_lock,
            "_try_dispatch_deferred_joint_adjustments": lambda **kwargs: False,
            "joint_motion_dispatcher": dispatcher,
            "deferred_joint_adjustments": SimpleNamespace(pending=False),
            "_clear_deferred_joint_adjustments": lambda: None,
            "controller_position_resynchronization_required": (
                resynchronization_required
            ),
            "application_closing": SimpleNamespace(is_set=lambda: True),
        }
        namespace["_invalidate_joint_motion_state"] = self.compile_function(
            "_invalidate_joint_motion_state",
            namespace,
        )
        poll_events = self.compile_function("_poll_serial_events", namespace)

        transport_lock.acquire()
        pending.set()
        event_queue.put(
            (
                "completed",
                "FIRST\n",
                "first",
                None,
                False,
                first_results.append,
                None,
            )
        )
        poll_events()

        self.assertEqual(first_results, [VALID_CONTROLLER_POSITION])
        self.assertEqual(second_results, [])
        self.assertFalse(transport_lock.locked())

        transport_lock.acquire()
        pending.set()
        event_queue.put(
            (
                "failed",
                "SECOND\n",
                None,
                "failed",
                False,
                second_results.append,
                None,
            )
        )
        poll_events()

        self.assertEqual(first_results, [VALID_CONTROLLER_POSITION])
        self.assertEqual(second_results, [None])
        self.assertFalse(transport_lock.locked())
        self.assertTrue(resynchronization_required.is_set())
        self.assertEqual(len(dispatcher.invalidations), 1)
        with self.assertRaises(Empty):
            event_queue.get_nowait()

    def test_controller_correction_settles_through_serial_result_application(self):
        class TerminalQueue(Queue):
            def __init__(self):
                super().__init__()
                self.terminal = threading.Event()

            def put(self, item, *args, **kwargs):
                super().put(item, *args, **kwargs)
                if item[0] in ("completed", "failed"):
                    self.terminal.set()

        class Widget:
            def delete(self, *args):
                pass

            def insert(self, *args):
                pass

            def config(self, **kwargs):
                pass

        class Dispatcher:
            active = False

            def __init__(self):
                self.invalidations = []

            def invalidate(self, reason):
                self.invalidations.append(reason)
                return False

        valid_response = (
            "A1B2C3D4E5F6G7H8I9J10K11L12"
            "M0N42.5OP13Q14R15"
        )
        responses = ["malformed", valid_response]
        result_queue = TerminalQueue()
        transport_lock = threading.Lock()
        correction_requested = threading.Event()
        correction_requested.set()
        resynchronization_required = threading.Event()
        resynchronization_required.set()
        dispatcher = Dispatcher()
        activity = SerialActivityRegistry(("ser",))
        namespace = {
            "threading": threading,
            "serial_event_queue": result_queue,
            "Empty": Empty,
            "serial_lock": transport_lock,
            "serial_activity_registry": activity,
            "SerialActivityRejected": SerialActivityRejected,
            "application_closing": threading.Event(),
            "controller_correction_requested": correction_requested,
            "controller_correction_state_lock": threading.Lock(),
            "controller_position_resynchronization_required": (
                resynchronization_required
            ),
            "RUN": {
                "offlineMode": False,
                "liveJog": False,
                "ser": SimpleNamespace(is_open=True),
            },
            "serial_transport_quarantined": lambda port: False,
            "live_jog_stop_requested": threading.Event(),
            "live_serial_result_pending": threading.Event(),
            "legacy_serial_result_pending": threading.Event(),
            "joint_motion_dispatcher": dispatcher,
            "deferred_joint_adjustments": SimpleNamespace(pending=False),
            "_clear_deferred_joint_adjustments": lambda: None,
            "_exchange_serial_line": lambda *args, **kwargs: responses.pop(0),
            "_transfer_main_serial_reservation": lambda: False,
            "_restore_main_serial_reservation": lambda: None,
            "parse_position_response": parse_position_response,
            "ProtocolResponseError": ProtocolResponseError,
            "cmdSentEntryField": Widget(),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "_try_dispatch_deferred_joint_adjustments": lambda **kwargs: False,
            "root": SimpleNamespace(after=lambda *args: None),
        }
        namespace["_invalidate_joint_motion_state"] = self.compile_function(
            "_invalidate_joint_motion_state",
            namespace,
        )

        def display_position(response, parsed=None, **kwargs):
            if parsed is None:
                namespace["_invalidate_joint_motion_state"](
                    "invalid correction response"
                )
                return None
            self.assertEqual(parsed.raw, valid_response)
            resynchronization_required.clear()
            return parsed

        namespace["displayPosition"] = display_position
        namespace["_apply_legacy_serial_response"] = self.compile_function(
            "_apply_legacy_serial_response",
            namespace,
        )
        namespace["_complete_controller_correction"] = self.compile_function(
            "_complete_controller_correction",
            namespace,
        )
        namespace["run_send_serial_safe"] = self.compile_function(
            "run_send_serial_safe",
            namespace,
        )
        namespace["start_send_serial_thread"] = self.compile_function(
            "start_send_serial_thread",
            namespace,
        )
        namespace["_try_dispatch_controller_correction"] = self.compile_function(
            "_try_dispatch_controller_correction",
            namespace,
        )
        poll_events = self.compile_function("_poll_serial_events", namespace)

        self.assertTrue(namespace["_try_dispatch_controller_correction"]())
        self.assertTrue(result_queue.terminal.wait(2))
        poll_events()
        self.assertTrue(correction_requested.is_set())
        self.assertTrue(resynchronization_required.is_set())
        self.assertFalse(transport_lock.locked())
        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertTrue(activity.idle())

        result_queue.terminal.clear()
        self.assertTrue(namespace["_try_dispatch_controller_correction"]())
        self.assertTrue(result_queue.terminal.wait(2))
        poll_events()
        self.assertFalse(correction_requested.is_set())
        self.assertFalse(resynchronization_required.is_set())
        self.assertFalse(transport_lock.locked())
        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertTrue(activity.idle())

    def test_controller_correction_poll_retains_request_without_usable_transport(self):
        correction_requested = threading.Event()
        correction_requested.set()
        starts = []
        namespace = {
            "controller_correction_requested": correction_requested,
            "controller_correction_state_lock": threading.Lock(),
            "application_closing": threading.Event(),
            "RUN": {"offlineMode": False, "ser": None},
            "serial_lock": threading.Lock(),
            "serial_transport_quarantined": lambda port: getattr(
                port,
                "quarantined",
                False,
            ),
            "start_send_serial_thread": lambda *args, **kwargs: starts.append(
                (args, kwargs)
            ) or True,
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        dispatch = self.compile_function(
            "_try_dispatch_controller_correction",
            namespace,
        )

        cases = (
            None,
            SimpleNamespace(is_open=False, quarantined=False),
            SimpleNamespace(is_open=True, quarantined=True),
        )
        for serial_port in cases:
            with self.subTest(serial_port=serial_port):
                namespace["RUN"]["ser"] = serial_port
                for _ in range(3):
                    self.assertFalse(dispatch())
                self.assertTrue(correction_requested.is_set())
                self.assertFalse(namespace["motion_request_registry"].active)

        self.assertEqual(starts, [])

    def test_fault_recovery_waits_for_dispatcher_acknowledgement(self):
        class Widget:
            def __init__(self):
                self.value = None

            def delete(self, *args):
                self.value = None

            def insert(self, index, value):
                self.value = value

            def config(self, **kwargs):
                self.value = kwargs.get("text")

            @staticmethod
            def get(*args):
                return ()

        class TransportLock:
            def __init__(self):
                self.busy = True

            def locked(self):
                return self.busy

        class Event:
            kind = "failed"
            position = None
            response = "EC100000"
            error = "controller rejected motion"
            pending_discarded = False

            def __init__(self, transport_lock):
                self.transport_lock = transport_lock
                self.acknowledged = False

            def acknowledge(self):
                self.acknowledged = True
                self.transport_lock.busy = False

        class Dispatcher:
            pending = False

            def __init__(self, event):
                self.event = event
                self.invalidations = []

            def drain_events(self):
                event = self.event
                self.event = None
                return [] if event is None else [event]

            def invalidate(self, reason):
                self.invalidations.append(reason)
                return False

        transport_lock = TransportLock()
        event = Event(transport_lock)
        dispatcher = Dispatcher(event)
        correction_requested = threading.Event()
        resynchronization_required = threading.Event()
        controller_starts = []
        auxiliary_stops = []
        cleared_deferred = []
        scheduled = []
        error_log = Widget()
        namespace = {
            "RUN": {
                "offlineMode": False,
                "estopActive": False,
                "posOutreach": False,
                "ser": SimpleNamespace(is_open=True),
            },
            "serial_transport_quarantined": lambda port: False,
            "TRUE": True,
            "END": "end",
            "cmdRecEntryField": Widget(),
            "almStatusLab": Widget(),
            "almStatusLab2": Widget(),
            "GCalmStatusLab": Widget(),
            "tab1": SimpleNamespace(runTrue=1),
            "tab8": SimpleNamespace(ElogView=error_log),
            "logger": SimpleNamespace(
                error=lambda *args: None,
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "pickle": SimpleNamespace(dump=lambda *args: None),
            "open": lambda *args, **kwargs: object(),
            "controller_correction_requested": correction_requested,
            "controller_position_resynchronization_required": (
                resynchronization_required
            ),
            "controller_correction_state_lock": threading.Lock(),
            "serial_lock": transport_lock,
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "_clear_deferred_joint_adjustments": lambda: cleared_deferred.append(True),
            "start_send_serial_thread": lambda command, **kwargs: controller_starts.append(
                (command, kwargs)
            ) or True,
            "_request_auxiliary_stop": lambda: auxiliary_stops.append(True) or True,
            "joint_motion_dispatcher": dispatcher,
            "deferred_joint_adjustments": SimpleNamespace(pending=False),
            "_try_set_virtual_joint_target": lambda target: True,
            "_current_joint_positions": lambda: (0.0,) * 9,
            "_set_virtual_from_joint_result": lambda position: True,
            "_try_dispatch_deferred_joint_adjustments": lambda: False,
            "root": SimpleNamespace(after=lambda *args: scheduled.append(args)),
            "parse_position_response": parse_position_response,
            "ProtocolResponseError": ProtocolResponseError,
        }
        namespace["_invalidate_joint_motion_state"] = self.compile_function(
            "_invalidate_joint_motion_state",
            namespace,
        )

        def display_position(response, parsed=None, **kwargs):
            self.assertIsInstance(parsed, PositionResponse)
            resynchronization_required.clear()
            return parsed

        namespace["displayPosition"] = display_position
        namespace["_apply_legacy_serial_response"] = self.compile_function(
            "_apply_legacy_serial_response",
            namespace,
        )
        namespace["_complete_controller_correction"] = self.compile_function(
            "_complete_controller_correction",
            namespace,
        )
        namespace["_try_dispatch_controller_correction"] = self.compile_function(
            "_try_dispatch_controller_correction",
            namespace,
        )
        namespace["_request_controller_correction"] = self.compile_function(
            "_request_controller_correction",
            namespace,
        )
        namespace["correctPos"] = self.compile_function("correctPos", namespace)
        namespace["stopProg"] = self.compile_function("stopProg", namespace)
        namespace["ErrorHandler"] = self.compile_function("ErrorHandler", namespace)
        poll_events = self.compile_function("_poll_joint_motion_events", namespace)

        poll_events()

        self.assertTrue(event.acknowledged)
        self.assertTrue(correction_requested.is_set())
        self.assertTrue(resynchronization_required.is_set())
        self.assertEqual(controller_starts, [])
        self.assertEqual(auxiliary_stops, [True])
        self.assertTrue(cleared_deferred)
        self.assertEqual(namespace["tab1"].runTrue, 0)

        self.assertTrue(namespace["_try_dispatch_controller_correction"]())
        self.assertEqual(controller_starts[0][0], "CP\n")
        correction_options = controller_starts[0][1]
        self.assertTrue(correction_options["controller_recovery"])
        self.assertTrue(callable(correction_options["completion_callback"]))
        self.assertTrue(correction_requested.is_set())
        self.assertTrue(resynchronization_required.is_set())
        correction_options["completion_callback"](None)
        self.assertTrue(correction_requested.is_set())
        self.assertTrue(resynchronization_required.is_set())
        self.assertFalse(namespace["motion_request_registry"].active)

        self.assertTrue(namespace["_try_dispatch_controller_correction"]())
        response = "A1B2C3D4E5F6G7H8I9J10K11L12M0N42.5OP13Q14R15"
        self.assertTrue(namespace["_apply_legacy_serial_response"](response))
        self.assertFalse(resynchronization_required.is_set())
        controller_starts[1][1]["completion_callback"](
            parse_position_response(response)
        )
        self.assertFalse(correction_requested.is_set())
        self.assertFalse(namespace["motion_request_registry"].active)

    def test_error_recovery_entry_points_have_no_blocking_serial_calls(self):
        forbidden_attributes = {"read", "readline", "write", "flushInput", "sleep"}
        for name in ("ErrorHandler", "correctPos", "stopProg"):
            function = self.module_functions[name]
            called_attributes = {
                node.func.attr
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
            }
            self.assertFalse(called_attributes & forbidden_attributes, name)

    def test_auxiliary_stop_worker_owns_exchange_without_tk_access(self):
        class Lock:
            def __init__(self):
                self.acquire_count = 0
                self.release_count = 0

            def acquire(self):
                self.acquire_count += 1
                return True

            def release(self):
                self.release_count += 1

        for exchange, expected in (
            (
                lambda command: "Nano Inactive Stopped",
                ("completed", 17, "Nano Inactive Stopped"),
            ),
            (
                lambda command: (_ for _ in ()).throw(OSError("auxiliary offline")),
                ("failed", 17, "auxiliary offline"),
            ),
        ):
            with self.subTest(expected=expected[0]):
                event_queue = Queue()
                lock = Lock()
                activity = SerialActivityRegistry(("ser2",))
                control_mode = activity.reserve_control("ser2")
                result_event = threading.Event()
                namespace = {
                    "auxiliary_serial_event_queue": event_queue,
                    "auxiliary_serial_lock": lock,
                    "auxiliary_serial_write_lock": threading.Lock(),
                    "auxiliary_stop_state_lock": threading.Lock(),
                    "auxiliary_stop_active_request_id": 17,
                    "auxiliary_stop_owner_result": None,
                    "auxiliary_stop_owner_result_event": result_event,
                    "auxiliary_stop_injected_event": threading.Event(),
                    "serial_activity_registry": activity,
                    "SerialActivityRegistry": SerialActivityRegistry,
                    "ProtocolResponseError": ProtocolResponseError,
                    "SerialActivityRejected": SerialActivityRejected,
                    "CONTROL_POLL_INTERVAL_SECONDS": 0.01,
                    "AUXILIARY_INACTIVE_STOP_RESPONSE": "Nano Inactive Stopped",
                    "application_closing": threading.Event(),
                    "RUN": {"ser2": object()},
                    "write_serial_control": write_serial_control,
                    "_exchange_auxiliary_line": exchange,
                }
                worker = self.compile_function("_run_auxiliary_stop_safe", namespace)

                worker(17, control_mode)

                self.assertEqual(event_queue.get_nowait(), ("started", 17, "STOP\n"))
                self.assertEqual(event_queue.get_nowait(), expected)
                self.assertEqual(lock.acquire_count, 1)
                self.assertEqual(lock.release_count, 1)
                self.assertTrue(activity.idle())
                self.assertIsNone(namespace["auxiliary_stop_active_request_id"])

        event_queue = Queue()
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2", control_injectable=True)
        control_mode = activity.reserve_control("ser2")
        control_writes = []
        control_written = threading.Event()
        result_event = threading.Event()
        namespace = {
            "auxiliary_serial_event_queue": event_queue,
            "auxiliary_serial_lock": Lock(),
            "auxiliary_serial_write_lock": object(),
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_active_request_id": 23,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": result_event,
            "auxiliary_stop_injected_event": threading.Event(),
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "ProtocolResponseError": ProtocolResponseError,
            "SerialActivityRejected": SerialActivityRejected,
            "SerialTransportTimeout": SerialTransportTimeout,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.01,
            "AUXILIARY_WAIT_NATURAL_RESPONSES": ("Done", "Timeout"),
            "AUXILIARY_INACTIVE_STOP_RESPONSE": "Nano Inactive Stopped",
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.1,
            "application_closing": threading.Event(),
            "time": time,
            "quarantine_serial_transport": lambda port, reason: True,
            "logger": SimpleNamespace(exception=lambda *args: None),
            "RUN": {"ser2": object()},
            "write_serial_control": lambda port, command, write_lock: control_writes.append(
                (port, command, write_lock)
            ) or control_written.set() or True,
            "_exchange_auxiliary_line": lambda command: (_ for _ in ()).throw(
                AssertionError("interruptible wait owner must remain the sole reader")
            ),
            "_read_auxiliary_inactive_stop_response": lambda deadline: (
                "Nano Inactive Stopped"
            ),
        }
        worker = self.compile_function("_run_auxiliary_stop_safe", namespace)

        worker_thread = threading.Thread(target=worker, args=(23, control_mode))
        worker_thread.start()

        self.assertEqual(event_queue.get(timeout=2), ("started", 23, "STOP\n"))
        self.assertTrue(control_written.wait(2))
        with self.assertRaises(Empty):
            event_queue.get(timeout=0.05)
        with namespace["auxiliary_stop_state_lock"]:
            namespace["auxiliary_stop_owner_result"] = (23, True, "Nano Stopped")
            result_event.set()
        worker_thread.join(2)
        self.assertFalse(worker_thread.is_alive())
        self.assertEqual(
            event_queue.get_nowait(),
            ("completed", 23, "Nano Stopped"),
        )
        self.assertEqual(len(control_writes), 1)
        self.assertTrue(activity.active("ser2"))
        activity.end("ser2", control_injectable=True)
        self.assertTrue(activity.idle())

        referenced_names = {
            node.id
            for node in ast.walk(self.module_functions["_run_auxiliary_stop_safe"])
            if isinstance(node, ast.Name)
        }
        self.assertFalse(
            referenced_names
            & {"root", "cmdSentEntryField", "cmdRecEntryField", "almStatusLab"}
        )

    def test_auxiliary_injected_stop_has_single_acknowledgement_deadline(self):
        event_queue = Queue()
        result_event = threading.Event()
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2", control_injectable=True)
        control_mode = activity.reserve_control("ser2")
        serial_port = SimpleNamespace(is_open=True)
        quarantines = []
        namespace = {
            "auxiliary_serial_event_queue": event_queue,
            "auxiliary_serial_lock": threading.Lock(),
            "auxiliary_serial_write_lock": threading.Lock(),
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_active_request_id": 29,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": result_event,
            "auxiliary_stop_injected_event": threading.Event(),
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "SerialActivityRejected": SerialActivityRejected,
            "SerialTransportTimeout": SerialTransportTimeout,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.001,
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.02,
            "application_closing": threading.Event(),
            "RUN": {"ser2": serial_port},
            "write_serial_control": lambda *args, **kwargs: True,
            "quarantine_serial_transport": (
                lambda port, reason: quarantines.append((port, reason)) or True
            ),
            "logger": SimpleNamespace(exception=lambda *args: None),
            "time": time,
        }
        worker = self.compile_function("_run_auxiliary_stop_safe", namespace)

        worker(29, control_mode)

        self.assertEqual(event_queue.get_nowait(), ("started", 29, "STOP\n"))
        terminal = event_queue.get_nowait()
        self.assertEqual(terminal[0:2], ("failed", 29))
        self.assertIn("acknowledgement deadline expired", terminal[2])
        self.assertEqual(
            quarantines,
            [(serial_port, "auxiliary stop acknowledgement deadline expired")],
        )
        self.assertTrue(activity.active("ser2"))
        activity.end("ser2", control_injectable=True)
        self.assertTrue(activity.idle())

    def test_auxiliary_natural_handoff_reuses_post_stop_deadline(self):
        class FakeTime:
            def __init__(self):
                self.values = iter((10.0, 10.09, 10.09, 10.101))

            def monotonic(self):
                return next(self.values)

        class DeadlineLock:
            def __init__(self):
                self.timeouts = []
                self.release_count = 0

            def acquire(self, timeout=None):
                self.timeouts.append(timeout)
                return True

            def release(self):
                self.release_count += 1

        request_id = 30
        event_queue = Queue()
        result_event = threading.Event()
        result_event.set()
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2", control_injectable=True)
        control_mode = activity.reserve_control("ser2")
        serial_port = SimpleNamespace(is_open=True)
        owner_lock = DeadlineLock()
        quarantines = []
        namespace = {
            "auxiliary_serial_event_queue": event_queue,
            "auxiliary_serial_lock": owner_lock,
            "auxiliary_serial_write_lock": threading.Lock(),
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_active_request_id": request_id,
            "auxiliary_stop_owner_result": (request_id, True, "Done"),
            "auxiliary_stop_owner_result_event": result_event,
            "auxiliary_stop_injected_event": threading.Event(),
            "auxiliary_stop_acknowledgement_deadline": None,
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "SerialActivityRejected": SerialActivityRejected,
            "SerialTransportTimeout": SerialTransportTimeout,
            "ProtocolResponseError": ProtocolResponseError,
            "MotionInputError": MotionInputError,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.01,
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.1,
            "AUXILIARY_WAIT_NATURAL_RESPONSES": ("Done", "Timeout"),
            "AUXILIARY_INACTIVE_STOP_RESPONSE": "Nano Inactive Stopped",
            "application_closing": threading.Event(),
            "RUN": {"ser2": serial_port},
            "write_serial_control": lambda *args, **kwargs: True,
            "read_serial_line_response": lambda *args, **kwargs: self.fail(
                "expired shared deadline must prevent a follow-up read"
            ),
            "quarantine_serial_transport": (
                lambda port, reason: quarantines.append((port, reason)) or True
            ),
            "logger": SimpleNamespace(exception=lambda *args: None),
            "finite_number": finite_number,
            "time": FakeTime(),
        }
        namespace["_read_auxiliary_inactive_stop_response"] = (
            self.compile_function(
                "_read_auxiliary_inactive_stop_response",
                namespace,
            )
        )
        worker = self.compile_function("_run_auxiliary_stop_safe", namespace)

        worker(request_id, control_mode)

        self.assertEqual(event_queue.get_nowait(), ("started", request_id, "STOP\n"))
        terminal = event_queue.get_nowait()
        self.assertEqual(terminal[0:2], ("failed", request_id))
        self.assertIn("acknowledgement deadline expired", terminal[2])
        self.assertEqual(len(owner_lock.timeouts), 1)
        self.assertAlmostEqual(owner_lock.timeouts[0], 0.01, places=6)
        self.assertEqual(owner_lock.release_count, 1)
        self.assertEqual(
            quarantines,
            [(serial_port, "auxiliary stop acknowledgement deadline expired")],
        )
        self.assertTrue(activity.active("ser2"))
        activity.end("ser2", control_injectable=True)
        self.assertTrue(activity.idle())

    def test_auxiliary_late_inactive_ack_is_rejected_after_natural_handoff(self):
        class SerialPort:
            def __init__(self, acknowledgement_event):
                self.is_open = True
                self.timeout = 5.0
                self.acknowledgement_event = acknowledgement_event
                self.returned_acknowledgements = 0

            def read_until(self, terminator, size):
                if not self.acknowledgement_event.wait(self.timeout):
                    return b""
                self.returned_acknowledgements += 1
                return b"Nano Inactive Stopped\n"

            def read(self, size=1):
                return b""

            def close(self):
                self.is_open = False

        request_id = 31
        acknowledgement_event = threading.Event()
        stop_written = threading.Event()
        event_queue = Queue()
        result_event = threading.Event()
        owner_lock = threading.Lock()
        owner_lock.acquire()
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2", control_injectable=True)
        control_mode = activity.reserve_control("ser2")
        serial_port = SerialPort(acknowledgement_event)
        namespace = {
            "auxiliary_serial_event_queue": event_queue,
            "auxiliary_serial_lock": owner_lock,
            "auxiliary_serial_write_lock": threading.Lock(),
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_active_request_id": request_id,
            "auxiliary_stop_owner_waiting": True,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": result_event,
            "auxiliary_stop_injected_event": threading.Event(),
            "auxiliary_stop_acknowledgement_deadline": None,
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "SerialActivityRejected": SerialActivityRejected,
            "SerialTransportTimeout": SerialTransportTimeout,
            "ProtocolResponseError": ProtocolResponseError,
            "MotionInputError": MotionInputError,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.01,
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.3,
            "AUXILIARY_WAIT_NATURAL_RESPONSES": ("Done", "Timeout"),
            "AUXILIARY_INACTIVE_STOP_RESPONSE": "Nano Inactive Stopped",
            "application_closing": threading.Event(),
            "RUN": {"ser2": serial_port},
            "write_serial_control": lambda *args, **kwargs: (
                stop_written.set() or True
            ),
            "read_serial_line_response": read_serial_line_response,
            "quarantine_serial_transport": quarantine_serial_transport,
            "logger": SimpleNamespace(exception=lambda *args: None),
            "finite_number": finite_number,
            "time": time,
        }
        namespace["_read_auxiliary_inactive_stop_response"] = (
            self.compile_function(
                "_read_auxiliary_inactive_stop_response",
                namespace,
            )
        )
        worker = self.compile_function("_run_auxiliary_stop_safe", namespace)
        worker_thread = threading.Thread(
            target=worker,
            args=(request_id, control_mode),
        )
        acknowledgement_timer = threading.Timer(
            0.36,
            acknowledgement_event.set,
        )
        acknowledgement_timer.daemon = True

        worker_thread.start()
        self.assertEqual(
            event_queue.get(timeout=2),
            ("started", request_id, "STOP\n"),
        )
        self.assertTrue(stop_written.wait(2))
        acknowledgement_timer.start()
        time.sleep(0.15)
        with namespace["auxiliary_stop_state_lock"]:
            namespace["auxiliary_stop_owner_result"] = (
                request_id,
                True,
                "Done",
            )
            result_event.set()
        owner_lock.release()

        worker_thread.join(2)
        self.assertFalse(worker_thread.is_alive())
        terminal = event_queue.get_nowait()
        self.assertEqual(terminal[0:2], ("failed", request_id))
        self.assertTrue(acknowledgement_event.wait(2))
        acknowledgement_timer.join(2)
        self.assertEqual(serial_port.returned_acknowledgements, 0)
        self.assertFalse(serial_port.is_open)
        self.assertTrue(activity.active("ser2"))
        activity.end("ser2", control_injectable=True)
        self.assertTrue(activity.idle())

    def test_auxiliary_timeout_preserves_deadline_until_owner_exit(self):
        request_id = 32
        event_queue = Queue()
        result_event = threading.Event()
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2", control_injectable=True)
        control_mode = activity.reserve_control("ser2")
        injected_event = threading.Event()
        namespace = {
            "auxiliary_serial_event_queue": event_queue,
            "auxiliary_serial_lock": threading.Lock(),
            "auxiliary_serial_write_lock": threading.Lock(),
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_active_request_id": request_id,
            "auxiliary_stop_pending_request_id": None,
            "auxiliary_stop_requested": threading.Event(),
            "auxiliary_stop_owner_waiting": True,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": result_event,
            "auxiliary_stop_injected_event": injected_event,
            "auxiliary_stop_acknowledgement_deadline": None,
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "SerialActivityRejected": SerialActivityRejected,
            "SerialTransportTimeout": SerialTransportTimeout,
            "ProtocolResponseError": ProtocolResponseError,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.001,
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.001,
            "AUXILIARY_STOP_OWNER_RESPONSES": (
                "Done",
                "Timeout",
                "Nano Stopped",
                "Nano Inactive Stopped",
            ),
            "application_closing": threading.Event(),
            "RUN": {
                "ser2": SimpleNamespace(is_open=True),
                "offlineMode": False,
            },
            "write_serial_control": lambda *args, **kwargs: True,
            "quarantine_serial_transport": lambda *args, **kwargs: (
                (_ for _ in ()).throw(OSError("close failed"))
            ),
            "logger": SimpleNamespace(exception=lambda *args: None),
            "finite_number": finite_number,
            "time": time,
        }
        worker = self.compile_function("_run_auxiliary_stop_safe", namespace)
        publish = self.compile_function(
            "_publish_auxiliary_stop_owner_result",
            namespace,
        )

        worker(request_id, control_mode)

        self.assertEqual(event_queue.get_nowait(), ("started", request_id, "STOP\n"))
        terminal = event_queue.get_nowait()
        self.assertEqual(terminal[0:2], ("failed", request_id))
        self.assertIn("acknowledgement deadline expired", terminal[2])
        self.assertTrue(injected_event.is_set())
        self.assertIsNotNone(
            namespace["auxiliary_stop_acknowledgement_deadline"]
        )

        preserved_deadline = namespace[
            "auxiliary_stop_acknowledgement_deadline"
        ]
        namespace["auxiliary_stop_pending_request_id"] = request_id + 1
        namespace["auxiliary_stop_requested"].set()
        dispatch = self.compile_function(
            "_try_dispatch_auxiliary_stop",
            namespace,
        )
        self.assertFalse(dispatch())
        self.assertEqual(
            namespace["auxiliary_stop_pending_request_id"],
            request_id + 1,
        )
        self.assertEqual(
            namespace["auxiliary_stop_acknowledgement_deadline"],
            preserved_deadline,
        )
        self.assertTrue(injected_event.is_set())

        self.assertFalse(publish(False, "owner transport ended"))
        self.assertFalse(injected_event.is_set())
        self.assertIsNone(namespace["auxiliary_stop_acknowledgement_deadline"])
        self.assertTrue(activity.active("ser2"))
        activity.end("ser2", control_injectable=True)
        self.assertTrue(activity.idle())

    def test_auxiliary_stop_injects_without_competing_with_active_legacy_reader(self):
        legacy_write_started = threading.Event()
        release_legacy_write = threading.Event()

        class SerialPort:
            def __init__(self):
                self.is_open = True
                self.commands = []
                self.flush_count = 0
                self.reset_count = 0

            def reset_input_buffer(self):
                self.reset_count += 1

            def write(self, command):
                self.commands.append(command)
                if command == b"WIA1B1C30\n":
                    legacy_write_started.set()
                    if not release_legacy_write.wait(2):
                        raise TimeoutError("legacy write release timed out")
                return len(command)

            def flush(self):
                self.flush_count += 1

            def close(self):
                self.is_open = False

        event_queue = Queue()
        requested = threading.Event()
        requested.set()
        result_event = threading.Event()
        activity = SerialActivityRegistry(("ser2",))
        activity.begin("ser2", control_injectable=True)
        serial_port = SerialPort()
        namespace = {
            "auxiliary_serial_event_queue": event_queue,
            "auxiliary_serial_lock": threading.Lock(),
            "auxiliary_serial_write_lock": threading.Lock(),
            "auxiliary_stop_requested": requested,
            "auxiliary_stop_state_lock": threading.Lock(),
            "auxiliary_stop_pending_request_id": 31,
            "auxiliary_stop_active_request_id": None,
            "auxiliary_stop_owner_waiting": True,
            "auxiliary_stop_owner_result": None,
            "auxiliary_stop_owner_result_event": result_event,
            "auxiliary_stop_injected_event": threading.Event(),
            "application_closing": SimpleNamespace(is_set=lambda: False),
            "serial_activity_registry": activity,
            "SerialActivityRegistry": SerialActivityRegistry,
            "ProtocolResponseError": ProtocolResponseError,
            "SerialActivityRejected": SerialActivityRejected,
            "SerialTransportTimeout": SerialTransportTimeout,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.01,
            "AUXILIARY_WAIT_TERMINAL_RESPONSES": (
                "Done",
                "Timeout",
                "Nano Stopped",
            ),
            "AUXILIARY_STOP_OWNER_RESPONSES": (
                "Done",
                "Timeout",
                "Nano Stopped",
                "Nano Inactive Stopped",
            ),
            "AUXILIARY_WAIT_NATURAL_RESPONSES": ("Done", "Timeout"),
            "AUXILIARY_INACTIVE_STOP_RESPONSE": "Nano Inactive Stopped",
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.1,
            "time": time,
            "quarantine_serial_transport": lambda port, reason: True,
            "logger": SimpleNamespace(exception=lambda *args: None),
            "RUN": {"ser2": serial_port, "offlineMode": False},
            "write_serial_control": write_serial_control,
            "serial_transport_quarantined": lambda serial_port: False,
            "_exchange_auxiliary_line": lambda command: (_ for _ in ()).throw(
                AssertionError("interruptible wait owner must remain the sole reader")
            ),
            "_read_auxiliary_inactive_stop_response": lambda deadline: (
                "Nano Inactive Stopped"
            ),
            "threading": threading,
        }
        write_legacy = self.compile_function(
            "_write_legacy_auxiliary_command",
            namespace,
        )
        namespace["_run_auxiliary_stop_safe"] = self.compile_function(
            "_run_auxiliary_stop_safe",
            namespace,
        )
        publish_owner_result = self.compile_function(
            "_publish_auxiliary_stop_owner_result",
            namespace,
        )
        dispatch = self.compile_function(
            "_try_dispatch_auxiliary_stop",
            namespace,
        )

        legacy_errors = []

        def run_legacy_write():
            try:
                write_legacy("WIA1B1C30\n")
            except BaseException as exc:
                legacy_errors.append(exc)

        legacy_worker = threading.Thread(target=run_legacy_write)
        legacy_worker.start()
        self.assertTrue(legacy_write_started.wait(2))
        self.assertTrue(dispatch())
        self.assertEqual(event_queue.get(timeout=2), ("started", 31, "STOP\n"))
        with self.assertRaises(Empty):
            event_queue.get(timeout=0.05)
        self.assertEqual(serial_port.commands, [b"WIA1B1C30\n"])

        release_legacy_write.set()
        legacy_worker.join(2)
        self.assertFalse(legacy_worker.is_alive())
        self.assertEqual(legacy_errors, [])
        deadline = time.monotonic() + 2
        while len(serial_port.commands) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(serial_port.commands, [b"WIA1B1C30\n", b"STOP\n"])
        with self.assertRaises(Empty):
            event_queue.get(timeout=0.05)
        self.assertTrue(publish_owner_result(True, "Nano Stopped"))
        self.assertEqual(
            event_queue.get(timeout=2),
            ("completed", 31, "Nano Stopped"),
        )
        self.assertEqual(serial_port.reset_count, 1)
        self.assertEqual(serial_port.flush_count, 2)
        self.assertFalse(requested.is_set())
        self.assertTrue(activity.active("ser2"))
        activity.end("ser2", control_injectable=True)
        deadline = time.monotonic() + 2
        while not activity.idle() and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(activity.idle())

    def test_auxiliary_natural_terminal_before_stop_write_drains_inactive_ack(self):
        class SerialPort:
            def __init__(self, natural_response):
                self.is_open = True
                self.timeout = 5.0
                self.commands = []
                self.response = bytearray()
                self.response_lock = threading.Lock()
                self.natural_response = natural_response
                self.wi_written = threading.Event()
                self.stop_written = threading.Event()
                self.allow_read = threading.Event()
                self.read_threads = set()
                self.reset_count = 0

            def write(self, command):
                self.commands.append(command)
                with self.response_lock:
                    if command.startswith(b"WI"):
                        self.response.extend(
                            f"{self.natural_response}\n".encode("ascii")
                        )
                        self.wi_written.set()
                    elif command == b"STOP\n":
                        self.response.extend(b"Nano Inactive Stopped\n")
                        self.stop_written.set()
                return len(command)

            def reset_input_buffer(self):
                self.reset_count += 1

            def flush(self):
                pass

            def readline(self):
                return self.read_until(
                    b"\n",
                    MAX_RESPONSE_FRAME_LENGTH + 1,
                )

            def read(self, size=1):
                self.read_threads.add(threading.get_ident())
                with self.response_lock:
                    value = bytes(self.response[:size])
                    del self.response[:size]
                return value

            def read_until(self, terminator, size):
                self.read_threads.add(threading.get_ident())
                if not self.allow_read.wait(2):
                    return b""
                with self.response_lock:
                    limit = min(size, len(self.response))
                    index = self.response.find(terminator, 0, limit)
                    count = limit if index < 0 else index + len(terminator)
                    value = bytes(self.response[:count])
                    del self.response[:count]
                return value

            def close(self):
                self.is_open = False

        for request_id, natural_response in ((37, "Done"), (38, "Timeout")):
            with self.subTest(natural_response=natural_response):
                event_queue = Queue()
                write_lock = threading.Lock()
                owner_lock = threading.Lock()
                result_event = threading.Event()
                requested = threading.Event()
                requested.set()
                activity = SerialActivityRegistry(("ser2",))
                serial_port = SerialPort(natural_response)
                logged_errors = []

                @contextmanager
                def tracked_auxiliary_operation(control_injectable=False):
                    self.assertTrue(owner_lock.acquire(blocking=False))
                    activity.begin(
                        "ser2",
                        control_injectable=control_injectable,
                    )
                    try:
                        yield
                    finally:
                        activity.end(
                            "ser2",
                            control_injectable=control_injectable,
                        )
                        owner_lock.release()

                namespace = {
                    "auxiliary_serial_event_queue": event_queue,
                    "auxiliary_serial_lock": owner_lock,
                    "auxiliary_serial_write_lock": write_lock,
                    "auxiliary_stop_requested": requested,
                    "auxiliary_stop_state_lock": threading.Lock(),
                    "auxiliary_stop_pending_request_id": request_id,
                    "auxiliary_stop_active_request_id": None,
                    "auxiliary_stop_owner_waiting": False,
                    "auxiliary_stop_owner_result": None,
                    "auxiliary_stop_owner_result_event": result_event,
                    "auxiliary_stop_injected_event": threading.Event(),
                    "application_closing": threading.Event(),
                    "serial_activity_registry": activity,
                    "SerialActivityRegistry": SerialActivityRegistry,
                    "ProtocolResponseError": ProtocolResponseError,
                    "SerialActivityRejected": SerialActivityRejected,
                    "SerialTransportTimeout": SerialTransportTimeout,
                    "CONTROL_POLL_INTERVAL_SECONDS": 0.01,
                    "AUXILIARY_WAIT_TERMINAL_RESPONSES": (
                        "Done",
                        "Timeout",
                        "Nano Stopped",
                    ),
                    "AUXILIARY_WAIT_NATURAL_RESPONSES": ("Done", "Timeout"),
                    "AUXILIARY_INACTIVE_STOP_RESPONSE": "Nano Inactive Stopped",
                    "AUXILIARY_STOP_OWNER_RESPONSES": (
                        "Done",
                        "Timeout",
                        "Nano Stopped",
                        "Nano Inactive Stopped",
                    ),
                    "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.5,
                    "RUN": {"ser2": serial_port, "offlineMode": False},
                    "write_serial_control": write_serial_control,
                    "read_serial_line_response": read_serial_line_response,
                    "read_serial_line_response_with_optional_followup": (
                        read_serial_line_response_with_optional_followup
                    ),
                    "serial_transport_quarantined": serial_transport_quarantined,
                    "quarantine_serial_transport": quarantine_serial_transport,
                    "finite_number": finite_number,
                    "MotionInputError": MotionInputError,
                    "MAX_RESPONSE_PAYLOAD_LENGTH": (
                        MAX_RESPONSE_PAYLOAD_LENGTH
                    ),
                    "_tracked_auxiliary_operation": tracked_auxiliary_operation,
                    "_write_legacy_auxiliary_command": (
                        lambda command: write_serial_control(
                            serial_port,
                            command,
                            write_lock=write_lock,
                            reset_input=True,
                        )
                    ),
                    "_finish_execute_row": lambda: None,
                    "ROW_EXECUTION_REJECTED": "rejected",
                    "ROW_EXECUTION_COMPLETE": "complete",
                    "logger": SimpleNamespace(
                        warning=lambda *args: None,
                        exception=lambda *args: logged_errors.append(args),
                    ),
                    "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
                    "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
                    "time": time,
                    "threading": threading,
                }
                namespace["_begin_auxiliary_stop_owner_wait"] = (
                    self.compile_function(
                        "_begin_auxiliary_stop_owner_wait",
                        namespace,
                    )
                )
                namespace["_publish_auxiliary_stop_owner_result"] = (
                    self.compile_function(
                        "_publish_auxiliary_stop_owner_result",
                        namespace,
                    )
                )
                namespace["_read_auxiliary_inactive_stop_response"] = (
                    lambda deadline: self.fail(
                        "first response owner must consume the queued inactive ack"
                    )
                )
                namespace["_run_auxiliary_stop_safe"] = self.compile_function(
                    "_run_auxiliary_stop_safe",
                    namespace,
                )
                execute_auxiliary = self.compile_function(
                    "_execute_row_auxiliary_command",
                    namespace,
                )
                dispatch_stop = self.compile_function(
                    "_try_dispatch_auxiliary_stop",
                    namespace,
                )

                owner_results = []
                owner_thread = threading.Thread(
                    target=lambda: owner_results.append(
                        execute_auxiliary(
                            "WIA1B1C30\n",
                            read_line=True,
                            control_injectable=True,
                            response_delay=0,
                            response_timeout=0.5,
                            accepted_responses=(
                                "Done",
                                "Timeout",
                                "Nano Stopped",
                            ),
                        )
                    )
                )
                owner_thread.start()
                self.assertTrue(serial_port.wi_written.wait(2))
                deadline = time.monotonic() + 2
                while (
                    not namespace["auxiliary_stop_owner_waiting"]
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.005)
                self.assertTrue(namespace["auxiliary_stop_owner_waiting"])
                self.assertTrue(dispatch_stop())
                self.assertEqual(
                    event_queue.get(timeout=2),
                    ("started", request_id, "STOP\n"),
                )
                self.assertTrue(serial_port.stop_written.wait(2))
                serial_port.allow_read.set()
                owner_thread.join(2)
                self.assertFalse(owner_thread.is_alive())

                try:
                    stop_result = event_queue.get(timeout=2)
                except Empty:
                    self.fail(
                        "auxiliary stop worker did not finish: "
                        f"owner_results={owner_results!r}, "
                        "owner_waiting="
                        f"{namespace['auxiliary_stop_owner_waiting']!r}, "
                        "owner_result="
                        f"{namespace['auxiliary_stop_owner_result']!r}, "
                        "active_request="
                        f"{namespace['auxiliary_stop_active_request_id']!r}, "
                        f"activity_idle={activity.idle()!r}, "
                        f"logged_errors={logged_errors!r}"
                    )
                self.assertEqual(
                    stop_result,
                    ("completed", request_id, "Nano Inactive Stopped"),
                )
                self.assertEqual(
                    owner_results,
                    [("complete", natural_response)],
                )
                self.assertEqual(
                    serial_port.commands,
                    [b"WIA1B1C30\n", b"STOP\n"],
                )
                self.assertEqual(serial_port.read_threads, {owner_thread.ident})
                self.assertEqual(serial_port.reset_count, 1)
                self.assertTrue(activity.idle())

    def test_auxiliary_followup_without_stop_request_quarantines_transport(self):
        class SerialPort:
            def __init__(self):
                self.is_open = True
                self.timeout = 5.0
                self.response = bytearray(b"Done\nNano Inactive Stopped\n")

            def read(self, size=1):
                value = bytes(self.response[:size])
                del self.response[:size]
                return value

            def read_until(self, terminator, size):
                limit = min(size, len(self.response))
                index = self.response.find(terminator, 0, limit)
                count = limit if index < 0 else index + len(terminator)
                value = bytes(self.response[:count])
                del self.response[:count]
                return value

            def close(self):
                self.is_open = False

        @contextmanager
        def tracked_auxiliary_operation(control_injectable=False):
            yield

        serial_port = SerialPort()
        labels = SimpleNamespace(config=lambda **kwargs: None)
        namespace = {
            "RUN": {"ser2": serial_port},
            "SerialActivityRejected": SerialActivityRejected,
            "ProtocolResponseError": ProtocolResponseError,
            "MotionInputError": MotionInputError,
            "MAX_RESPONSE_PAYLOAD_LENGTH": MAX_RESPONSE_PAYLOAD_LENGTH,
            "SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS": 0.5,
            "AUXILIARY_WAIT_NATURAL_RESPONSES": ("Done", "Timeout"),
            "AUXILIARY_INACTIVE_STOP_RESPONSE": "Nano Inactive Stopped",
            "auxiliary_stop_injected_event": threading.Event(),
            "_tracked_auxiliary_operation": tracked_auxiliary_operation,
            "_write_legacy_auxiliary_command": lambda command: True,
            "_begin_auxiliary_stop_owner_wait": lambda: True,
            "_publish_auxiliary_stop_owner_result": lambda *args: False,
            "read_serial_line_response_with_optional_followup": (
                read_serial_line_response_with_optional_followup
            ),
            "read_serial_line_response": read_serial_line_response,
            "read_serial_exact_response": lambda *args: None,
            "quarantine_serial_transport": quarantine_serial_transport,
            "serial_transport_quarantined": serial_transport_quarantined,
            "finite_number": finite_number,
            "_finish_execute_row": lambda: None,
            "ROW_EXECUTION_REJECTED": "rejected",
            "ROW_EXECUTION_COMPLETE": "complete",
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": labels,
            "almStatusLab2": labels,
            "time": time,
        }
        execute_auxiliary = self.compile_function(
            "_execute_row_auxiliary_command",
            namespace,
        )

        result = execute_auxiliary(
            "WIA1B1C30\n",
            read_line=True,
            control_injectable=True,
            response_delay=0,
            response_timeout=0.5,
            accepted_responses=("Done", "Timeout", "Nano Stopped"),
        )

        self.assertEqual(result, ("rejected", None))
        self.assertIsNone(namespace["RUN"]["ser2"])
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_auxiliary_stop_results_are_applied_on_tk_poll(self):
        class Widget:
            def __init__(self):
                self.value = None

            def delete(self, *args):
                self.value = None

            def insert(self, index, value):
                self.value = value

            def config(self, **kwargs):
                self.value = kwargs.get("text")

        event_queue = Queue()
        event_queue.put(("started", 41, "STOP\n"))
        event_queue.put(("completed", 41, "Nano Inactive Stopped"))
        sent = Widget()
        received = Widget()
        first_label = Widget()
        second_label = Widget()
        recovery_attempts = []
        stop_statuses = []
        runtime = {"programStopRequestId": 41}
        namespace = {
            "auxiliary_serial_event_queue": event_queue,
            "Empty": Empty,
            "cmdSentEntryField": sent,
            "cmdRecEntryField": received,
            "almStatusLab": first_label,
            "almStatusLab2": second_label,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "RUN": runtime,
            "_set_program_stop_status": stop_statuses.append,
            "_try_dispatch_controller_correction": lambda: recovery_attempts.append(
                "controller"
            ),
            "_try_dispatch_auxiliary_stop": lambda: recovery_attempts.append(
                "auxiliary"
            ),
            "_ensure_startup_auxiliary_cleanup": lambda: True,
            "application_closing": SimpleNamespace(is_set=lambda: True),
            "root": SimpleNamespace(after=lambda *args: None),
        }
        poll_events = self.compile_function("_poll_auxiliary_serial_events", namespace)

        poll_events()

        self.assertEqual(sent.value, "STOP\n")
        self.assertEqual(received.value, "Nano Inactive Stopped")
        self.assertEqual(stop_statuses, ["pending", "completed"])
        self.assertIsNone(runtime["programStopRequestId"])
        self.assertEqual(recovery_attempts, ["controller", "auxiliary"])

        runtime["programStopRequestId"] = 42
        event_queue.put(("failed", 42, "auxiliary offline"))
        poll_events()

        self.assertEqual(stop_statuses[-1], "failed")
        self.assertIsNone(runtime["programStopRequestId"])
        self.assertEqual(
            recovery_attempts,
            [
                "controller", "auxiliary",
                "controller", "auxiliary",
            ],
        )

    def test_startup_timeout_is_async_and_waits_for_worker_termination(self):
        class Request:
            pass

        class Spinner:
            def __init__(self):
                self.destroy_count = 0

            def grab_release(self):
                pass

            def destroy(self):
                self.destroy_count += 1

        class Progress:
            def stop(self):
                pass

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))
                return len(self.jobs)

            def after_cancel(self, job):
                pass

        spinner = Spinner()
        root = Root()
        worker_started = threading.Event()
        cancellation_seen = threading.Event()
        finished = []
        timeouts = []
        abandoned = []
        request = Request()

        def startup(startup_request, cancel_event):
            self.assertIs(startup_request, request)
            worker_started.set()
            if not cancel_event.wait(2):
                raise RuntimeError("test did not cancel startup worker")
            cancellation_seen.set()
            return "cancelled"

        namespace = {
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "startup_spinner": lambda *args: (spinner, Progress()),
            "Queue": Queue,
            "Empty": Empty,
            "Thread": threading.Thread,
            "threading": threading,
            "time": time,
            "startup": startup,
            "ControllerStartupRequest": Request,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.005,
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        start = self.compile_function("startup_with_spinner", namespace)

        worker = start(
            root,
            request,
            lambda result, timed_out: finished.append((result, timed_out)),
            lambda: timeouts.append(True),
            lambda result: abandoned.append(result),
            timeout=0.02,
        )
        self.assertTrue(worker_started.wait(2))
        self.assertTrue(worker.is_alive())
        self.assertEqual(finished, [])

        time.sleep(0.03)
        root.jobs.pop(0)[1]()
        self.assertEqual(timeouts, [True])
        self.assertEqual(finished, [])
        self.assertEqual(spinner.destroy_count, 1)

        self.assertTrue(cancellation_seen.wait(2))
        deadline = time.monotonic() + 2
        while not finished and time.monotonic() < deadline:
            if root.jobs:
                root.jobs.pop(0)[1]()
            time.sleep(0.005)
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(finished, [("cancelled", True)])
        self.assertEqual(abandoned, [])
        self.assertEqual(spinner.destroy_count, 1)

    def test_startup_thread_construction_failure_closes_spinner(self):
        class Request:
            pass

        class Spinner:
            def __init__(self):
                self.destroy_count = 0

            def grab_release(self):
                pass

            def destroy(self):
                self.destroy_count += 1

        class Progress:
            def stop(self):
                pass

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append((delay, callback))

        def fail_thread(*args, **kwargs):
            raise RuntimeError("thread construction failed")

        spinner = Spinner()
        root = Root()
        namespace = {
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "startup_spinner": lambda *args: (spinner, Progress()),
            "Queue": Queue,
            "Empty": Empty,
            "Thread": fail_thread,
            "threading": threading,
            "time": time,
            "startup": lambda *args: None,
            "ControllerStartupRequest": Request,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.005,
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        start = self.compile_function("startup_with_spinner", namespace)

        with self.assertRaisesRegex(RuntimeError, "thread construction failed"):
            start(
                root,
                Request(),
                lambda *args: None,
                lambda: None,
                lambda result: None,
            )

        self.assertEqual(spinner.destroy_count, 1)
        self.assertEqual(root.jobs, [])

    def test_delayed_poll_accepts_worker_that_finished_before_deadline(self):
        class Request:
            pass

        class Spinner:
            def grab_release(self):
                pass

            def destroy(self):
                pass

        class Progress:
            def stop(self):
                pass

        class Root:
            def __init__(self):
                self.jobs = []

            def after(self, delay, callback):
                self.jobs.append(callback)
                return len(self.jobs)

            def after_cancel(self, job):
                pass

        root = Root()
        finished = []
        timeouts = []
        abandoned = []
        work_done = threading.Event()
        request = Request()

        def startup(startup_request, cancel_event):
            self.assertIs(startup_request, request)
            work_done.set()
            return "ready"

        namespace = {
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "startup_spinner": lambda *args: (Spinner(), Progress()),
            "Queue": Queue,
            "Empty": Empty,
            "Thread": threading.Thread,
            "threading": threading,
            "time": time,
            "startup": startup,
            "ControllerStartupRequest": Request,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.005,
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        start = self.compile_function("startup_with_spinner", namespace)

        worker = start(
            root,
            request,
            lambda result, timed_out: finished.append((result, timed_out)),
            lambda: timeouts.append(True),
            lambda result: abandoned.append(result),
            timeout=0.02,
        )
        self.assertTrue(work_done.wait(2))
        time.sleep(0.03)
        root.jobs.pop(0)()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(timeouts, [])
        self.assertEqual(finished, [("ready", False)])
        self.assertEqual(abandoned, [])

    def test_startup_scheduler_failure_cancels_without_blocking_tk(self):
        class Request:
            pass

        class Spinner:
            def __init__(self):
                self.destroy_count = 0

            def grab_release(self):
                pass

            def destroy(self):
                self.destroy_count += 1

        class Progress:
            def stop(self):
                pass

        class Root:
            def __init__(self):
                self.job = None
                self.after_count = 0

            def after(self, delay, callback):
                self.after_count += 1
                if self.after_count > 1:
                    raise RuntimeError("scheduler unavailable")
                self.job = callback
                return "job"

            def after_cancel(self, job):
                pass

        spinner = Spinner()
        root = Root()
        finished = []
        timeouts = []
        abandoned = []
        request = Request()

        def startup(startup_request, cancel_event):
            self.assertIs(startup_request, request)
            if not cancel_event.wait(2):
                raise RuntimeError("test did not cancel startup worker")
            return "cancelled"

        namespace = {
            "finite_number": finite_number,
            "MotionInputError": MotionInputError,
            "startup_spinner": lambda *args: (spinner, Progress()),
            "Queue": Queue,
            "Empty": Empty,
            "Thread": threading.Thread,
            "threading": threading,
            "time": time,
            "startup": startup,
            "ControllerStartupRequest": Request,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.005,
            "logger": SimpleNamespace(exception=lambda *args: None),
        }
        start = self.compile_function("startup_with_spinner", namespace)

        worker = start(
            root,
            request,
            lambda result, timed_out: finished.append((result, timed_out)),
            lambda: timeouts.append(True),
            lambda result: abandoned.append(result),
            timeout=10,
        )
        started = time.monotonic()
        root.job()
        self.assertLess(time.monotonic() - started, 0.5)
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(timeouts, [True])
        self.assertEqual(finished, [])
        self.assertEqual(abandoned, ["cancelled"])
        self.assertEqual(spinner.destroy_count, 1)

    def test_startup_worker_sequence_has_no_tk_dependency(self):
        class Request:
            auxiliary_port = "COM2"
            auxiliary_board = AUXILIARY_BOARD_NANO
            update_parameters_command = "UPA1\n"
            external_axis_command = "CEA1\n"
            position_command = "SPA1\n"

        class Result:
            def __init__(
                self,
                position,
                visual_options,
                auxiliary_serial,
                auxiliary_error=None,
            ):
                self.position = position
                self.visual_options = visual_options
                self.auxiliary_serial = auxiliary_serial
                self.auxiliary_error = auxiliary_error

        request = Request()
        auxiliary_serial = object()
        calls = []
        closed = []
        results = []
        errors = []
        main_thread = threading.get_ident()
        raw_position = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"

        def exchange(command, cancel_event, expected_response=None):
            calls.append((threading.get_ident(), command, expected_response))
            if command == "HO\n":
                return VALID_CONTROLLER_IDENTITY_RESPONSE
            if command == "RP\n":
                return raw_position
            return expected_response.decode("ascii").strip()

        namespace = {
            "ControllerStartupRequest": Request,
            "ControllerStartupResult": Result,
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "threading": threading,
            "_connect_startup_auxiliary": (
                lambda port, board_profile: auxiliary_serial
            ),
            "_startup_exchange_response": exchange,
            "parse_position_response": parse_position_response,
            "_startup_visual_options": lambda: ("part.jpg",),
            "_request_startup_auxiliary_cleanup": (
                lambda serial_port: closed.append(serial_port)
            ),
        }
        startup = self.compile_function("startup", namespace)

        def run_startup():
            try:
                results.append(startup(request, threading.Event()))
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=run_startup)
        worker.start()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].position.raw, raw_position)
        self.assertEqual(results[0].visual_options, ("part.jpg",))
        self.assertIsNone(results[0].auxiliary_error)
        self.assertEqual(closed, [])
        self.assertEqual(
            [call[1:] for call in calls],
            [
                ("HO\n", None),
                ("UPA1\n", b"Done"),
                ("CEA1\n", b"Done"),
                ("SPA1\n", b"Done\n"),
                ("RP\n", None),
            ],
        )
        self.assertTrue(all(call[0] != main_thread for call in calls))

        forbidden_names = {
            "setCom2",
            "updateParams",
            "calExtAxis",
            "sendPos",
            "requestPos",
            "updateVisOp",
        }
        referenced_names = {
            node.id
            for node in ast.walk(self.module_functions["startup"])
            if isinstance(node, ast.Name)
        }
        self.assertEqual(referenced_names & forbidden_names, set())

    def test_main_startup_proceeds_without_an_auxiliary_controller(self):
        request_namespace = {
            "dataclass": dataclass,
            "Optional": Optional,
            "MotionInputError": MotionInputError,
            "normalize_auxiliary_board_profile": (
                normalize_auxiliary_board_profile
            ),
            "_validated_startup_command": lambda command, prefix: command,
        }
        request_type = self.compile_class(
            "ControllerStartupRequest",
            request_namespace,
        )
        request = request_type("None", "UPA1\n", "CEA1\n", "SPA1\n")
        self.assertIsNone(request.auxiliary_port)
        self.assertIsNone(request.auxiliary_board)
        profiled_request = request_type(
            "COM2",
            "UPA1\n",
            "CEA1\n",
            "SPA1\n",
            AUXILIARY_BOARD_NANO,
        )
        self.assertEqual(
            profiled_request.auxiliary_board,
            AUXILIARY_BOARD_NANO,
        )
        with self.assertRaises(MotionInputError):
            request_type(
                "COM2",
                "UPA1\n",
                "CEA1\n",
                "SPA1\n",
                "Unknown",
            )

        class Request:
            def __init__(self, auxiliary_port, auxiliary_board=None):
                self.auxiliary_port = auxiliary_port
                self.auxiliary_board = auxiliary_board
                self.update_parameters_command = "UPA1\n"
                self.external_axis_command = "CEA1\n"
                self.position_command = "SPA1\n"

        class Result:
            def __init__(
                self,
                position,
                visual_options,
                auxiliary_serial,
                auxiliary_error=None,
            ):
                self.position = position
                self.visual_options = visual_options
                self.auxiliary_serial = auxiliary_serial
                self.auxiliary_error = auxiliary_error

        raw_position = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"
        for auxiliary_port, auxiliary_board, connector, expected_error in (
            (
                None,
                None,
                lambda port, board_profile: (_ for _ in ()).throw(
                    AssertionError("an unconfigured auxiliary port must not be opened")
                ),
                None,
            ),
            (
                "COM2",
                None,
                lambda port, board_profile: (_ for _ in ()).throw(
                    AssertionError("an unprofiled auxiliary port must not be opened")
                ),
                "auxiliary-board profile must be selected",
            ),
            (
                "COM2",
                AUXILIARY_BOARD_NANO,
                lambda port, board_profile: (_ for _ in ()).throw(
                    OSError("auxiliary offline")
                ),
                "auxiliary offline",
            ),
        ):
            with self.subTest(
                auxiliary_port=auxiliary_port,
                auxiliary_board=auxiliary_board,
            ):
                exchanges = []

                def exchange(command, cancel_event, expected_response=None):
                    exchanges.append((command, expected_response))
                    if command == "HO\n":
                        return VALID_CONTROLLER_IDENTITY_RESPONSE
                    if command == "RP\n":
                        return raw_position
                    return expected_response.decode("ascii").strip()

                namespace = {
                    "ControllerStartupRequest": Request,
                    "ControllerStartupResult": Result,
                    "MotionInputError": MotionInputError,
                    "ProtocolResponseError": ProtocolResponseError,
                    "threading": threading,
                    "_connect_startup_auxiliary": connector,
                    "_startup_exchange_response": exchange,
                    "parse_position_response": parse_position_response,
                    "_startup_visual_options": lambda: (),
                    "_request_startup_auxiliary_cleanup": lambda serial_port: None,
                    "_clear_unavailable_startup_auxiliary": lambda: True,
                }
                startup = self.compile_function("startup", namespace)

                result = startup(
                    Request(auxiliary_port, auxiliary_board),
                    threading.Event(),
                )

                self.assertIsNone(result.auxiliary_serial)
                self.assertEqual(result.auxiliary_error, expected_error)
                self.assertEqual(
                    exchanges,
                    [
                        ("HO\n", None),
                        ("UPA1\n", b"Done"),
                        ("CEA1\n", b"Done"),
                        ("SPA1\n", b"Done\n"),
                        ("RP\n", None),
                    ],
                )

    def test_optional_startup_closes_existing_auxiliary_before_main_exchange(self):
        class Request:
            def __init__(self, auxiliary_port, auxiliary_board=None):
                self.auxiliary_port = auxiliary_port
                self.auxiliary_board = auxiliary_board
                self.update_parameters_command = "UPA1\n"
                self.external_axis_command = "CEA1\n"
                self.position_command = "SPA1\n"

        class Result:
            def __init__(
                self,
                position,
                visual_options,
                auxiliary_serial,
                auxiliary_error=None,
            ):
                self.position = position
                self.visual_options = visual_options
                self.auxiliary_serial = auxiliary_serial
                self.auxiliary_error = auxiliary_error

        raw_position = "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"
        cases = (
            (None, None, False, None),
            (
                "COM2",
                None,
                False,
                "auxiliary-board profile must be selected",
            ),
            ("COM2", AUXILIARY_BOARD_NANO, True, "auxiliary offline"),
        )
        for auxiliary_port, board, connection_fails, expected_error in cases:
            with self.subTest(
                auxiliary_port=auxiliary_port,
                board=board,
            ):
                old_serial = SimpleNamespace(is_open=True)
                runtime = {
                    "ser2": old_serial,
                    "ser2BoardProfile": (
                        old_serial,
                        AUXILIARY_BOARD_NANO,
                    ),
                }
                order = []

                def cleanup(serial_port):
                    self.assertIs(serial_port, old_serial)
                    order.append("auxiliary-closed")
                    serial_port.is_open = False
                    runtime["ser2"] = None
                    runtime["ser2BoardProfile"] = None
                    return True

                def connect(port, board_profile):
                    if not connection_fails:
                        raise AssertionError(
                            "disabled or unprofiled auxiliary must not connect"
                        )
                    raise OSError("auxiliary offline")

                def exchange(command, cancel_event, expected_response=None):
                    if command == "HO\n":
                        self.assertIs(runtime["ser2"], old_serial)
                        self.assertEqual(order, [])
                        order.append(command)
                        return VALID_CONTROLLER_IDENTITY_RESPONSE
                    self.assertIsNone(runtime["ser2"])
                    self.assertEqual(order[:2], ["HO\n", "auxiliary-closed"])
                    order.append(command)
                    if command == "RP\n":
                        return raw_position
                    return expected_response.decode("ascii").strip()

                namespace = {
                    "RUN": runtime,
                    "ControllerStartupRequest": Request,
                    "ControllerStartupResult": Result,
                    "MotionInputError": MotionInputError,
                    "MotionTransportBusy": MotionTransportBusy,
                    "ProtocolResponseError": ProtocolResponseError,
                    "threading": threading,
                    "_connect_startup_auxiliary": connect,
                    "_startup_exchange_response": exchange,
                    "parse_position_response": parse_position_response,
                    "_startup_visual_options": lambda: (),
                    "_request_startup_auxiliary_cleanup": cleanup,
                }
                namespace["_clear_unavailable_startup_auxiliary"] = (
                    self.compile_function(
                        "_clear_unavailable_startup_auxiliary",
                        namespace,
                    )
                )
                startup = self.compile_function("startup", namespace)

                result = startup(
                    Request(auxiliary_port, board),
                    threading.Event(),
                )

                self.assertIsNone(result.auxiliary_serial)
                self.assertEqual(result.auxiliary_error, expected_error)
                self.assertFalse(old_serial.is_open)
                self.assertIsNone(runtime["ser2BoardProfile"])
                self.assertEqual(order[:2], ["HO\n", "auxiliary-closed"])

        retained_serial = SimpleNamespace(is_open=True)
        retained_runtime = {
            "ser2": retained_serial,
            "ser2BoardProfile": (
                retained_serial,
                AUXILIARY_BOARD_NANO,
            ),
        }
        exchanges = []

        def retained_exchange(command, *args, **kwargs):
            exchanges.append(command)
            if command == "HO\n":
                return VALID_CONTROLLER_IDENTITY_RESPONSE
            raise AssertionError("startup must stop after auxiliary cleanup failure")

        retained_namespace = {
            "RUN": retained_runtime,
            "ControllerStartupRequest": Request,
            "ControllerStartupResult": Result,
            "MotionInputError": MotionInputError,
            "MotionTransportBusy": MotionTransportBusy,
            "ProtocolResponseError": ProtocolResponseError,
            "threading": threading,
            "_connect_startup_auxiliary": lambda *args: None,
            "_startup_exchange_response": retained_exchange,
            "parse_position_response": parse_position_response,
            "_startup_visual_options": lambda: (),
            "_request_startup_auxiliary_cleanup": lambda serial_port: False,
        }
        retained_namespace["_clear_unavailable_startup_auxiliary"] = (
            self.compile_function(
                "_clear_unavailable_startup_auxiliary",
                retained_namespace,
            )
        )
        retained_startup = self.compile_function(
            "startup",
            retained_namespace,
        )

        with self.assertRaisesRegex(
            MotionTransportBusy,
            "could not be closed before startup commit",
        ):
            retained_startup(Request(None), threading.Event())

        self.assertEqual(exchanges, ["HO\n"])
        self.assertIs(retained_runtime["ser2"], retained_serial)
        self.assertTrue(retained_serial.is_open)

        changed_serial = SimpleNamespace(is_open=True)
        changed_runtime = {
            "ser2": changed_serial,
            "ser2BoardProfile": (
                changed_serial,
                AUXILIARY_BOARD_NANO,
            ),
        }
        changed_namespace = {
            "RUN": changed_runtime,
            "MotionTransportBusy": MotionTransportBusy,
            "_request_startup_auxiliary_cleanup": lambda serial_port: True,
        }
        clear_changed = self.compile_function(
            "_clear_unavailable_startup_auxiliary",
            changed_namespace,
        )

        with self.assertRaisesRegex(
            MotionTransportBusy,
            "ownership changed during startup cleanup",
        ):
            clear_changed()

        self.assertIs(changed_runtime["ser2"], changed_serial)

    def test_startup_result_normalizes_auxiliary_error_text(self):
        result_type = self.compile_class(
            "ControllerStartupResult",
            {
                "dataclass": dataclass,
                "Optional": Optional,
                "PositionResponse": PositionResponse,
                "ProtocolResponseError": ProtocolResponseError,
                "os": os,
            },
        )
        position = parse_position_response(
            "A1B2C3D4E5F6G1H2I3J4K5L6M0NOP7Q8R9"
        )

        result = result_type(position, (), None, "  auxiliary offline  ")

        self.assertEqual(result.auxiliary_error, "auxiliary offline")
        with self.assertRaisesRegex(
            ProtocolResponseError,
            "cannot contain both",
        ):
            result_type(position, (), object(), "auxiliary offline")

    def test_startup_exchange_consumes_exact_and_line_firmware_responses(self):
        class SerialPort:
            def __init__(self, response):
                self.is_open = True
                self.timeout = 7.5
                self.response = bytearray(response)
                self.commands = []
                self.reset_count = 0

            def reset_input_buffer(self):
                self.reset_count += 1

            def write(self, command):
                self.commands.append(command)
                return len(command)

            def flush(self):
                pass

            def read(self, size):
                chunk = bytes(self.response[:size])
                del self.response[:size]
                return chunk

        namespace = {
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "SerialTransportQuarantinedError": ConnectionError,
            "SerialTransportTimeout": TimeoutError,
            "MAX_COMMAND_LENGTH": 4096,
            "MAX_RESPONSE_FRAME_LENGTH": MAX_RESPONSE_FRAME_LENGTH,
            "MAX_RESPONSE_PAYLOAD_LENGTH": MAX_RESPONSE_PAYLOAD_LENGTH,
            "SERIAL_STARTUP_READ_TIMEOUT_SECONDS": 0.1,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.005,
            "decode_serial_response_line": decode_serial_response_line,
            "serial_transport_quarantined": lambda serial_port: False,
            "serial_write_lock": threading.Lock(),
            "time": time,
        }
        namespace["_validated_startup_command"] = self.compile_function(
            "_validated_startup_command",
            namespace,
        )
        exchange = self.compile_function("_startup_exchange_response", namespace)
        cancel_event = threading.Event()

        exact_serial = SerialPort(b"Done")
        namespace["RUN"] = {"ser": exact_serial}
        self.assertEqual(
            exchange("CEA1\n", cancel_event, expected_response=b"Done"),
            "Done",
        )
        self.assertEqual(exact_serial.commands, [b"CEA1\n"])
        self.assertEqual(exact_serial.timeout, 7.5)

        update_serial = SerialPort(b"Done")
        namespace["RUN"] = {"ser": update_serial}
        self.assertEqual(
            exchange("UPA1\n", cancel_event, expected_response=b"Done"),
            "Done",
        )
        self.assertEqual(update_serial.commands, [b"UPA1\n"])
        self.assertEqual(update_serial.timeout, 7.5)

        trailing_serial = SerialPort(b"Donejunk")
        namespace["RUN"] = {"ser": trailing_serial}
        with self.assertRaisesRegex(
            ProtocolResponseError,
            "trailing unframed data",
        ):
            exchange("CEA1\n", cancel_event, expected_response=b"Done")
        self.assertEqual(trailing_serial.timeout, 7.5)

        framed_serial = SerialPort(b"Done\r\n")
        namespace["RUN"] = {"ser": framed_serial}
        self.assertEqual(
            exchange("SPA1\n", cancel_event, expected_response=b"Done\n"),
            "Done",
        )
        self.assertEqual(framed_serial.commands, [b"SPA1\n"])
        self.assertEqual(framed_serial.timeout, 7.5)

        line_serial = SerialPort(b"A1B2C3\n")
        namespace["RUN"] = {"ser": line_serial}
        self.assertEqual(exchange("RP\n", cancel_event), "A1B2C3")
        self.assertEqual(line_serial.commands, [b"RP\n"])
        self.assertEqual(line_serial.timeout, 7.5)

        class CancelDuringResetSerialPort(SerialPort):
            def reset_input_buffer(self):
                super().reset_input_buffer()
                cancel_event.set()

        cancellation_serial = CancelDuringResetSerialPort(b"Done")
        namespace["RUN"] = {"ser": cancellation_serial}
        with self.assertRaisesRegex(
            TimeoutError,
            "controller startup cancelled",
        ):
            exchange("CEA1\n", cancel_event, expected_response=b"Done")
        self.assertEqual(cancellation_serial.reset_count, 1)
        self.assertEqual(cancellation_serial.commands, [])
        self.assertEqual(cancellation_serial.timeout, 7.5)
        cancel_event.clear()

        maximum_payload = b"x" * MAX_RESPONSE_PAYLOAD_LENGTH
        for delimiter in (b"\n", b"\r\n"):
            with self.subTest(maximum_startup_delimiter=delimiter):
                maximum_serial = SerialPort(maximum_payload + delimiter)
                namespace["RUN"] = {"ser": maximum_serial}
                self.assertEqual(
                    exchange(
                        "RP\n",
                        cancel_event,
                        expected_response=maximum_payload + delimiter,
                    ),
                    maximum_payload.decode("ascii"),
                )
                self.assertEqual(maximum_serial.commands, [b"RP\n"])
                self.assertEqual(maximum_serial.timeout, 7.5)

        oversized_payload = b"x" * (MAX_RESPONSE_PAYLOAD_LENGTH + 1)
        for delimiter in (b"\n", b"\r\n"):
            with self.subTest(oversized_startup_delimiter=delimiter):
                oversized_serial = SerialPort(oversized_payload + delimiter)
                namespace["RUN"] = {"ser": oversized_serial}
                with self.assertRaisesRegex(
                    ProtocolResponseError,
                    "exceeds the size limit",
                ):
                    exchange("RP\n", cancel_event)
                self.assertEqual(oversized_serial.timeout, 7.5)

        for response, expected_response in (
            (b"Done", b"Done"),
            (b"Done\n", b"Done\n"),
        ):
            with self.subTest(expired_quiet_response=response):
                deadline_serial = SerialPort(response)
                exchange.__globals__["RUN"] = {"ser": deadline_serial}
                monotonic_values = iter((100.0, 100.0, 100.099))
                exchange.__globals__["time"] = SimpleNamespace(
                    monotonic=lambda: next(monotonic_values)
                )
                with self.assertRaisesRegex(
                    TimeoutError,
                    "quiet-boundary deadline expired",
                ):
                    exchange(
                        "CEA1\n",
                        cancel_event,
                        expected_response=expected_response,
                    )
                self.assertEqual(deadline_serial.timeout, 7.5)

    def test_startup_numeric_builder_preserves_delimiters_without_exponents(self):
        namespace = {
            "MotionInputError": MotionInputError,
            "MAX_COMMAND_LENGTH": 4096,
            "controller_protocol_decimal": controller_protocol_decimal,
        }
        namespace["_validated_startup_command"] = self.compile_function(
            "_validated_startup_command",
            namespace,
        )
        build = self.compile_function(
            "_build_startup_numeric_command",
            namespace,
        )

        command = build(
            "UP",
            (("A", "1e-5"), ("e", "2e3"), ("+", 3)),
        )

        first = controller_protocol_decimal("1e-5", "first")
        second = controller_protocol_decimal("2e3", "second")
        self.assertEqual(command, f"UPA{first}e{second}+3\n")
        self.assertNotIn("e-", command)

    def test_startup_worker_closes_auxiliary_connection_after_failure(self):
        class Request:
            auxiliary_port = "COM2"
            auxiliary_board = AUXILIARY_BOARD_NANO
            update_parameters_command = "UPA1\n"
            external_axis_command = "CEA1\n"
            position_command = "SPA1\n"

        auxiliary_serial = object()
        closed = []
        exchanges = []

        def exchange(command, *args, **kwargs):
            exchanges.append(command)
            if command == "HO\n":
                return VALID_CONTROLLER_IDENTITY_RESPONSE
            raise TimeoutError("controller startup cancelled")

        namespace = {
            "ControllerStartupRequest": Request,
            "ControllerStartupResult": object,
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "threading": threading,
            "_connect_startup_auxiliary": (
                lambda port, board_profile: auxiliary_serial
            ),
            "_startup_exchange_response": exchange,
            "parse_position_response": parse_position_response,
            "_startup_visual_options": lambda: (),
            "_request_startup_auxiliary_cleanup": (
                lambda serial_port: closed.append(serial_port)
            ),
        }
        startup = self.compile_function("startup", namespace)

        with self.assertRaisesRegex(TimeoutError, "controller startup cancelled"):
            startup(Request(), threading.Event())
        self.assertEqual(exchanges, ["HO\n", "UPA1\n"])
        self.assertEqual(closed, [auxiliary_serial])

    def test_startup_rejects_missing_wrist_capability_before_side_effects(self):
        class Request:
            auxiliary_port = "COM2"
            auxiliary_board = AUXILIARY_BOARD_NANO
            update_parameters_command = "UPA1\n"
            external_axis_command = "CEA1\n"
            position_command = "SPA1\n"

        unsupported_identity = json.loads(VALID_CONTROLLER_IDENTITY_RESPONSE)
        unsupported_identity["ProtocolCapabilities"] = []
        exchanges = []
        connections = []
        cleanup_calls = []

        def exchange(command, *args, **kwargs):
            exchanges.append(command)
            if command != "HO\n":
                raise AssertionError("unsupported firmware must receive no writes")
            return json.dumps(unsupported_identity, separators=(",", ":"))

        namespace = {
            "ControllerStartupRequest": Request,
            "ControllerStartupResult": object,
            "MotionInputError": MotionInputError,
            "ProtocolResponseError": ProtocolResponseError,
            "threading": threading,
            "_connect_startup_auxiliary": (
                lambda *args: connections.append(args)
            ),
            "_startup_exchange_response": exchange,
            "parse_position_response": parse_position_response,
            "_startup_visual_options": lambda: (),
            "_request_startup_auxiliary_cleanup": cleanup_calls.append,
        }
        startup = self.compile_function("startup", namespace)

        with self.assertRaisesRegex(
            ProtocolResponseError,
            "lacks command-local JT wrist configuration",
        ):
            startup(Request(), threading.Event())

        self.assertEqual(exchanges, ["HO\n"])
        self.assertEqual(connections, [])
        self.assertEqual(cleanup_calls, [])

    def test_failed_startup_auxiliary_close_is_retained_and_retried(self):
        class SerialPort:
            def __init__(self):
                self.is_open = True
                self.close_attempts = 0
                self.closed = threading.Event()

            def close(self):
                self.close_attempts += 1
                if self.close_attempts == 1:
                    raise OSError("transient close failure")
                self.is_open = False
                self.closed.set()

        serial_port = SerialPort()
        namespace = {
            "RUN": {"ser2": serial_port},
            "auxiliary_serial_lock": threading.Lock(),
            "startup_auxiliary_cleanup_lock": threading.Lock(),
            "startup_auxiliary_cleanup_pending": {},
            "startup_auxiliary_cleanup_worker": None,
            "threading": threading,
            "time": time,
            "SERIAL_SHUTDOWN_RETRY_MS": 1,
            "logger": SimpleNamespace(
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        namespace["_close_startup_auxiliary"] = self.compile_function(
            "_close_startup_auxiliary",
            namespace,
        )
        namespace["_run_startup_auxiliary_cleanup"] = self.compile_function(
            "_run_startup_auxiliary_cleanup",
            namespace,
        )
        namespace["_ensure_startup_auxiliary_cleanup"] = self.compile_function(
            "_ensure_startup_auxiliary_cleanup",
            namespace,
        )
        request_cleanup = self.compile_function(
            "_request_startup_auxiliary_cleanup",
            namespace,
        )

        self.assertFalse(request_cleanup(serial_port))
        self.assertTrue(serial_port.closed.wait(2))
        deadline = time.monotonic() + 2
        while namespace["startup_auxiliary_cleanup_pending"]:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.001)

        self.assertGreaterEqual(serial_port.close_attempts, 2)
        self.assertIsNone(namespace["RUN"]["ser2"])

        direct_callers = set()
        for name, function in self.module_functions.items():
            if any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_close_startup_auxiliary"
                for node in ast.walk(function)
            ):
                direct_callers.add(name)
        self.assertEqual(
            direct_callers,
            {
                "_request_startup_auxiliary_cleanup",
                "_run_startup_auxiliary_cleanup",
            },
        )

    def test_every_live_jog_disallows_seconds_mode(self):
        for name in ("LiveJointJog", "LiveCarJog", "LiveToolJog"):
            function = self.module_functions[name]
            prepare_calls = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_prepare_live_jog"
            ]
            self.assertEqual(len(prepare_calls), 1, name)
            self.assertEqual(len(prepare_calls[0].args), 3, name)
            self.assertEqual(prepare_calls[0].keywords, [], name)

        live_parameters = self.module_functions["_live_jog_parameters"]
        seconds_prefixes = [
            node
            for node in ast.walk(live_parameters)
            if isinstance(node, ast.Constant) and node.value == "Ss"
        ]
        self.assertEqual(seconds_prefixes, [])

    def test_xbox_watchdog_scheduler_failure_stops_active_motion(self):
        stop_requests = []
        live_stop = threading.Event()
        offline_stop = threading.Event()
        labels = []
        errors = []
        namespace = {
            "RUN": {"_xbox_watchdog_failed": False},
            "live_jog_stop_requested": live_stop,
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_stop_event": offline_stop,
            "_request_switch": lambda value: stop_requests.append(("joint", value)),
            "_request_switch_cart": (
                lambda value: stop_requests.append(("cartesian", value))
            ),
            "_request_switch_tool": (
                lambda value: stop_requests.append(("tool", value))
            ),
            "logger": SimpleNamespace(
                error=lambda *args: errors.append(args),
                exception=lambda *args: errors.append(args),
            ),
            "_lbl": lambda text, style=None: labels.append((text, style)),
        }
        fail_watchdog = self.compile_nested_function(
            "_fail_xbox_watchdog",
            namespace,
        )

        self.assertFalse(fail_watchdog(RuntimeError("Tk unavailable")))
        self.assertTrue(namespace["RUN"]["_xbox_watchdog_failed"])
        self.assertTrue(live_stop.is_set())
        self.assertTrue(offline_stop.is_set())
        self.assertEqual(
            stop_requests,
            [("joint", None), ("cartesian", None), ("tool", None)],
        )
        self.assertEqual(labels, [("XBOX WATCHDOG FAILED", "Alarm.TLabel")])
        self.assertTrue(errors)

        schedule_failures = []
        schedule_namespace = {
            "root": SimpleNamespace(
                after=lambda *args: (_ for _ in ()).throw(
                    RuntimeError("scheduler closed")
                )
            ),
            "WATCHDOG_MS": 200,
            "_watchdog_tick": lambda: None,
            "_fail_xbox_watchdog": (
                lambda error: schedule_failures.append(str(error)) or False
            ),
        }
        schedule_watchdog = self.compile_nested_function(
            "_schedule_watchdog",
            schedule_namespace,
        )
        self.assertFalse(schedule_watchdog())
        self.assertEqual(schedule_failures, ["scheduler closed"])

        initial_failures = []
        poll_namespace = {
            "_find_controller": lambda: 0,
            "_lbl": lambda *args, **kwargs: None,
            "_tk_call": lambda *args, **kwargs: False,
            "_schedule_watchdog": lambda: True,
            "_fail_xbox_watchdog": (
                lambda error: initial_failures.append(str(error)) or False
            ),
        }
        poll_loop = self.compile_nested_function("_poll_loop", poll_namespace)
        poll_loop()
        self.assertEqual(initial_failures, ["initial Tk scheduling failed"])

    def test_speed_fields_reset_overlapping_acceleration_and_deceleration(self):
        class Value:
            def __init__(self, value):
                self.value = str(value)

            def get(self):
                return self.value

            def delete(self, start, end):
                self.value = ""

            def insert(self, start, value):
                self.value = str(value)

        acceleration = Value(60)
        deceleration = Value(41)
        namespace = {
            "ACCrampField": Value(25),
            "ACCspeedField": acceleration,
            "DECspeedField": deceleration,
            "speedEntryField": Value(50),
            "speedOption": Value("Percent"),
        }
        check_speed_values = self.compile_function("checkSpeedVals", namespace)

        check_speed_values()

        self.assertEqual(acceleration.get(), "10")
        self.assertEqual(deceleration.get(), "10")

    def test_live_jog_profile_uses_controller_safe_decimals(self):
        class Value:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        namespace = {
            "CAL": {
                f"J{axis}OpenLoopVal": Value(0)
                for axis in range(1, 7)
            },
            "MotionInputError": MotionInputError,
            "MotionProfile": MotionProfile,
            "ACCrampField": Value(25),
            "ACCspeedField": Value(10),
            "DECspeedField": Value(10),
            "checkSpeedVals": lambda: None,
            "controller_protocol_decimal": controller_protocol_decimal,
            "speedEntryField": Value("1e-5"),
            "speedOption": Value("Percent"),
        }
        live_parameters = self.compile_function(
            "_live_jog_parameters",
            namespace,
        )

        result = live_parameters(25)
        encoded = controller_protocol_decimal("1e-5", "expected value")

        self.assertEqual(result[0], "Sp")
        self.assertEqual(result[1], encoded)
        self.assertNotRegex(result[1], r"[eE+]")

    def test_live_jog_callbacks_do_not_sleep_or_read_serial(self):
        for name in ("LiveJointJog", "LiveCarJog", "LiveToolJog", "StopJog"):
            function = self.module_functions[name]
            calls = [node for node in ast.walk(function) if isinstance(node, ast.Call)]
            blocking_calls = [
                node
                for node in calls
                if (
                    isinstance(node.func, ast.Attribute)
                    and node.func.attr in {"sleep", "read", "readline", "write"}
                )
            ]
            self.assertEqual(blocking_calls, [], name)

    def test_virtual_live_workers_cannot_reactivate_released_jog(self):
        for name in ("live_joint_jog", "live_cartesian_jog", "live_tool_jog"):
            function = self.module_functions[name]
            shared_live_state = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Subscript)
                and isinstance(node.value, ast.Name)
                and node.value.id == "RUN"
                and isinstance(node.slice, ast.Constant)
                and node.slice.value == "liveJog"
            ]
            parameters = [argument.arg for argument in function.args.args]
            self.assertEqual(shared_live_state, [], name)
            self.assertIn("stop_event", parameters, name)

    def test_offline_live_jog_reservation_is_generation_scoped(self):
        first_started = threading.Event()
        second_started = threading.Event()
        starts = []

        def target(label, stop_event):
            starts.append((label, stop_event))
            (first_started if label == "first" else second_started).set()
            stop_event.wait(2)
            return True

        initial_stop = threading.Event()
        initial_stop.set()
        namespace = {
            "threading": threading,
            "offline_live_jog_lock": threading.Lock(),
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_stop_event": initial_stop,
            "offline_live_jog_operation": None,
            "offline_live_jog_pose_snapshot": None,
            "application_closing": threading.Event(),
            "virtual_motion_event_queue": Queue(),
            "RUN": {"liveJog": False, "VR_angles": [0.0] * 6},
            "MotionInputError": MotionInputError,
            "_validated_virtual_six_vector": (
                lambda values, label: tuple(values)
            ),
            "_set_virtual_joint_target": (
                lambda target: namespace["RUN"].update(
                    VR_angles=list(target)
                )
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "refresh_gui_from_joint_angles": lambda angles: True,
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        settle = self.compile_function("_settle_offline_live_jog", namespace)
        start = self.compile_function("_start_offline_live_jog", namespace)
        mode_lock = threading.Lock()

        first_operation = start(mode_lock, target, ("first",))
        self.assertIsInstance(first_operation, VirtualMotionOperation)
        self.assertTrue(first_started.wait(1))
        first_stop = start.__globals__["offline_live_jog_stop_event"]
        self.assertFalse(start(mode_lock, target, ("rejected",)))
        self.assertTrue(namespace["RUN"]["liveJog"])

        first_stop.set()
        deadline = time.monotonic() + 1
        while namespace["offline_live_jog_lock"].locked() and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(namespace["offline_live_jog_lock"].locked())
        self.assertTrue(first_operation.completed)
        self.assertFalse(start(mode_lock, target, ("pending-settlement",)))
        self.assertTrue(namespace["RUN"]["liveJog"])
        self.assertTrue(settle(first_operation))

        second_operation = start(mode_lock, target, ("second",))
        self.assertIsInstance(second_operation, VirtualMotionOperation)
        self.assertTrue(second_started.wait(1))
        second_stop = start.__globals__["offline_live_jog_stop_event"]
        self.assertIsNot(first_stop, second_stop)
        self.assertFalse(second_stop.is_set())
        first_stop.set()
        time.sleep(0.02)
        self.assertTrue(namespace["offline_live_jog_lock"].locked())

        second_stop.set()
        deadline = time.monotonic() + 1
        while namespace["offline_live_jog_lock"].locked() and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(namespace["offline_live_jog_lock"].locked())
        self.assertTrue(settle(second_operation))

        namespace["application_closing"].set()
        self.assertFalse(start(mode_lock, target, ("after-close",)))
        self.assertFalse(namespace["offline_live_jog_lock"].locked())
        self.assertFalse(mode_lock.locked())
        self.assertEqual([label for label, stop_event in starts], ["first", "second"])

    def test_offline_live_refresh_failure_releases_owner_as_failed(self):
        operation = completed_virtual_operation()
        saved_pose = (1.0,) * 6
        current_pose = (2.0,) * 6
        runtime = {"liveJog": True, "VR_angles": list(current_pose)}
        errors = []
        namespace = {
            "RUN": runtime,
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_operation": operation,
            "offline_live_jog_pose_snapshot": saved_pose,
            "_set_virtual_joint_target": (
                lambda target: runtime.update(VR_angles=list(target))
            ),
            "refresh_gui_from_joint_angles": (
                lambda target: (_ for _ in ()).throw(
                    MotionInputError("native FK failed")
                )
            ),
            "logger": SimpleNamespace(
                error=lambda message: errors.append(message),
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        settle = self.compile_function("_settle_offline_live_jog", namespace)
        lease = namespace["motion_request_registry"].acquire("Offline live jog")
        namespace["offline_live_jog_motion_lease"] = lease

        self.assertFalse(settle(operation))

        self.assertEqual(runtime["VR_angles"], list(current_pose))
        self.assertFalse(runtime["liveJog"])
        self.assertFalse(namespace["motion_request_registry"].active)
        self.assertIsNone(namespace["offline_live_jog_operation"])
        self.assertIsNone(namespace["offline_live_jog_pose_snapshot"])
        self.assertIn("GUI settlement failed", errors[-1])

    def test_offline_stop_blocks_segment_admission_and_defers_snapshot(self):
        stop_event = threading.Event()
        state_lock = threading.Lock()
        starts = []
        namespace = {
            "offline_live_jog_state_lock": state_lock,
            "start_driveMotorsJ_thread": lambda *args: starts.append(args),
            "time": time,
            "CONTROL_POLL_INTERVAL_SECONDS": 0.001,
        }
        start_segment = self.compile_function(
            "_start_offline_virtual_segment",
            namespace,
        )
        result = []

        state_lock.acquire()
        worker = threading.Thread(
            target=lambda: result.append(start_segment(stop_event, (1, 2, 3)))
        )
        worker.start()
        stop_event.set()
        state_lock.release()
        worker.join(1)

        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [None])
        self.assertEqual(starts, [])

        operation = VirtualMotionOperation()
        live_stop = threading.Event()
        refreshes = []
        runtime = {
            "offlineMode": True,
            "liveJog": False,
            "VR_angles": [1.0] * 6,
        }
        event_namespace = {
            "RUN": runtime,
            "live_serial_result_pending": threading.Event(),
            "application_closing": threading.Event(),
            "offline_live_jog_state_lock": threading.Lock(),
            "offline_live_jog_operation": operation,
            "offline_live_jog_pose_snapshot": tuple(runtime["VR_angles"]),
            "offline_live_jog_stop_event": live_stop,
            "live_jog_stop_requested": threading.Event(),
            "_set_virtual_joint_target": (
                lambda target: runtime.update(VR_angles=list(target))
            ),
            "refresh_gui_from_joint_angles": (
                lambda target: refreshes.append(list(target)) or True
            ),
            "logger": SimpleNamespace(
                warning=lambda *args: None,
                error=lambda *args: None,
                exception=lambda *args: None,
            ),
            "almStatusLab": SimpleNamespace(config=lambda **kwargs: None),
            "almStatusLab2": SimpleNamespace(config=lambda **kwargs: None),
        }
        stop_jog = self.compile_function("StopJog", event_namespace)
        settle = self.compile_function("_settle_offline_live_jog", event_namespace)
        event_namespace["offline_live_jog_motion_lease"] = event_namespace[
            "motion_request_registry"
        ].acquire("Offline live jog")
        start_blocked = self.compile_function(
            "_live_jog_start_is_blocked",
            event_namespace,
        )
        event_namespace["_live_jog_start_is_blocked"] = start_blocked
        event_namespace["_prepare_live_jog"] = lambda *args: self.fail(
            "an active owner must reject before request parsing"
        )
        live_joint_jog = self.compile_function(
            "LiveJointJog",
            event_namespace,
        )

        self.assertTrue(start_blocked())
        runtime["offlineMode"] = False
        self.assertTrue(start_blocked())
        runtime["offlineMode"] = True
        live_joint_jog("invalid second request")
        self.assertFalse(live_stop.is_set())

        stop_jog(None)

        self.assertTrue(live_stop.is_set())
        self.assertFalse(runtime["liveJog"])
        self.assertEqual(refreshes, [])

        operation.complete(True)
        self.assertTrue(settle(operation))
        self.assertFalse(runtime["liveJog"])
        self.assertEqual(refreshes, [[1.0] * 6])
        self.assertIsNone(event_namespace["offline_live_jog_operation"])

        runtime["offlineMode"] = True
        event_namespace["live_serial_result_pending"].set()
        self.assertTrue(start_blocked())
        event_namespace["live_serial_result_pending"].clear()

    def test_virtual_drive_reservation_precedes_worker_start(self):
        started = threading.Event()
        release = threading.Event()

        def drive(*args):
            started.set()
            release.wait(2)

        namespace = {
            "threading": threading,
            "drive_lock": threading.Lock(),
            "driveMotorsJ": drive,
            "logger": SimpleNamespace(
                info=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        self.compile_function("run_driveMotorsJ_safe", namespace)
        start = self.compile_function("start_driveMotorsJ_thread", namespace)

        operation = start("first")
        self.assertIsInstance(operation, VirtualMotionOperation)
        self.assertTrue(namespace["drive_lock"].locked())
        self.assertTrue(started.wait(1))
        self.assertFalse(start("rejected"))

        release.set()
        deadline = time.monotonic() + 1
        while namespace["drive_lock"].locked() and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(namespace["drive_lock"].locked())
        self.assertTrue(operation.wait(1))
        self.assertEqual(operation.result(), (True, None))

    def test_virtual_drive_failure_publishes_terminal_result(self):
        def drive(*args):
            raise RuntimeError("render update failed")

        namespace = {
            "threading": threading,
            "drive_lock": threading.Lock(),
            "driveMotorsJ": drive,
            "logger": SimpleNamespace(
                info=lambda *args: None,
                exception=lambda *args: None,
            ),
        }
        self.compile_function("run_driveMotorsJ_safe", namespace)
        start = self.compile_function("start_driveMotorsJ_thread", namespace)

        operation = start("failure")

        self.assertTrue(operation.wait(1))
        self.assertFalse(namespace["drive_lock"].locked())
        self.assertEqual(
            operation.result(),
            (False, "RuntimeError: render update failed"),
        )

    def test_virtual_motion_commands_report_drive_start_result(self):
        for name in ("rj_command", "mj_command", "mt_command"):
            function = self.module_functions[name]
            returned_starts = [
                node.value
                for node in ast.walk(function)
                if isinstance(node, ast.Return)
                and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Name)
                and node.value.func.id == "start_driveMotorsJ_thread"
            ]
            failed_starts = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Return)
                and isinstance(node.value, ast.Constant)
                and node.value.value is False
            ]

            self.assertEqual(len(returned_starts), 1, name)
            self.assertTrue(failed_starts, name)

    def test_virtual_live_workers_emit_errors_without_tk_access(self):
        forbidden_names = {
            "ErrorHandler",
            "TFxEntryField",
            "TFyEntryField",
            "TFzEntryField",
            "TFrxEntryField",
            "TFryEntryField",
            "TFrzEntryField",
        }
        for name in ("live_joint_jog", "live_cartesian_jog", "live_tool_jog"):
            function = self.module_functions[name]
            referenced_names = {
                node.id for node in ast.walk(function) if isinstance(node, ast.Name)
            }
            self.assertFalse(referenced_names & forbidden_names, name)

        poller = self.module_functions["_poll_virtual_motion_events"]
        error_handler_calls = [
            node
            for node in ast.walk(poller)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "ErrorHandler"
        ]
        self.assertEqual(len(error_handler_calls), 1)

    def test_live_jog_uses_interruptible_serial_worker(self):
        dispatcher = self.module_functions["_dispatch_live_jog"]
        worker_calls = [
            node
            for node in ast.walk(dispatcher)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "start_send_serial_thread"
        ]
        self.assertEqual(len(worker_calls), 1)
        live_keywords = [
            keyword
            for keyword in worker_calls[0].keywords
            if keyword.arg == "live_jog"
        ]
        self.assertEqual(len(live_keywords), 1)
        self.assertIsInstance(live_keywords[0].value, ast.Constant)
        self.assertIs(live_keywords[0].value.value, True)

        for name in ("LiveJointJog", "LiveCarJog", "LiveToolJog"):
            function = self.module_functions[name]
            dispatch_calls = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_dispatch_live_jog"
            ]
            self.assertEqual(len(dispatch_calls), 1, name)
            live_assignments = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Assign)
                and any(
                    isinstance(target, ast.Subscript)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "RUN"
                    and isinstance(target.slice, ast.Constant)
                    and target.slice.value == "liveJog"
                    for target in node.targets
                )
                and isinstance(node.value, ast.Constant)
                and node.value.value is True
            ]
            self.assertEqual(live_assignments, [], name)

        for name in ("_dispatch_live_jog", "_start_offline_live_jog"):
            function = self.module_functions[name]
            live_assignments = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Assign)
                and any(
                    isinstance(target, ast.Subscript)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "RUN"
                    and isinstance(target.slice, ast.Constant)
                    and target.slice.value == "liveJog"
                    for target in node.targets
                )
                and isinstance(node.value, ast.Constant)
                and node.value.value is True
            ]
            self.assertEqual(len(live_assignments), 1, name)

        stop = self.module_functions["StopJog"]
        stop_requests = [
            node
            for node in ast.walk(stop)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "live_jog_stop_requested"
            and node.func.attr == "set"
        ]
        self.assertEqual(len(stop_requests), 1)

    def test_online_live_jog_does_not_launch_virtual_drive_workers(self):
        pairs = (
            ("LiveJointJog", "start_live_joint_jog_thread"),
            ("LiveCarJog", "start_live_cartesian_jog_thread"),
            ("LiveToolJog", "start_live_tool_jog_thread"),
        )
        for function_name, starter_name in pairs:
            function = self.module_functions[function_name]
            starter_calls = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == starter_name
            ]
            self.assertEqual(len(starter_calls), 1, function_name)

            guarded_calls = []
            for node in ast.walk(function):
                if not isinstance(node, ast.If):
                    continue
                test = node.test
                offline_references = [
                    child
                    for child in ast.walk(test)
                    if isinstance(child, ast.Subscript)
                    and isinstance(child.value, ast.Name)
                    and child.value.id == "RUN"
                    and isinstance(child.slice, ast.Constant)
                    and child.slice.value == "offlineMode"
                ]
                if not offline_references:
                    continue
                guarded_calls.extend(
                    child
                    for child in ast.walk(node)
                    if isinstance(child, ast.Call)
                    and isinstance(child.func, ast.Name)
                    and child.func.id == starter_name
                )
            self.assertEqual(len(guarded_calls), 1, function_name)


if __name__ == "__main__":
    unittest.main()
