import math
import re
import struct
import threading
import time
import unittest
from decimal import Decimal
from unittest.mock import patch

from ARrobots.HMI.joint_motion import (
    AUXILIARY_BOARD_MEGA,
    AUXILIARY_BOARD_NANO,
    AUXILIARY_BOARD_NONE,
    AUXILIARY_BOARD_INPUT_PINS,
    AUXILIARY_BOARD_OUTPUT_PINS,
    AUXILIARY_BOARD_SERVO_CHANNELS,
    AUXILIARY_CURRENT_MAXIMUM_AMPS,
    AUXILIARY_WAIT_MAXIMUM_SECONDS,
    CONTROLLER_CAPABILITY_GCODE_DELETE_IDENTITY_V1,
    CONTROLLER_CAPABILITY_GCODE_DIRECTORY_FRAMING_V1,
    CONTROLLER_CAPABILITY_GCODE_WRITE_IDENTITY_V1,
    CONTROLLER_CAPABILITY_CALIBRATION_SWITCH_POLARITY_V1,
    CONTROLLER_CAPABILITY_ESTOP_ADMISSION_V1,
    CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1,
    CONTROLLER_DIRECTORY_SEPARATOR,
    CONTROLLER_MAXIMUM_RAMP_PERCENT,
    CoalescingJointDispatcher,
    CommandTiming,
    CommandedJointTrajectory,
    ControllerJointCalibration,
    DeferredLiveMotionArbiter,
    LiveMotionScheduleResult,
    MAX_CONTROLLER_DIRECTORY_PAYLOAD_BYTES,
    MAX_RESPONSE_PAYLOAD_LENGTH,
    MAX_CONTROLLER_FILENAME_BYTES,
    DeferredJointAdjustments,
    JointExchangeSnapshot,
    JointMotionCommand,
    JointMotionExchangeResult,
    JointMove,
    JointTelemetry,
    MotionInputError,
    MotionProfile,
    MotionQueueFault,
    MotionRequestLease,
    MotionRequestRegistry,
    MotionTransportBusy,
    PRIMARY_START_POSITION,
    PrimaryHomeReference,
    ProtocolResponseError,
    SerialActivityRegistry,
    SerialActivityRejected,
    VirtualMotionOperation,
    auxiliary_pneumatic_output_pin,
    build_controller_modbus_command,
    build_robot_joint_command,
    canonicalize_serial_command,
    canonicalize_virtual_command,
    classify_controller_modbus_terminal_response,
    command_response_timeout,
    controller_modbus_command_is_write,
    controller_degree_to_native_radians,
    controller_number,
    controller_protocol_decimal,
    encode_calibration_switch_mask,
    estimate_commanded_joint_trajectory,
    finite_number,
    motion_timing_response_timeout,
    normalize_auxiliary_board_profile,
    ordinary_joint_telemetry_measurement,
    parse_auxiliary_output_command,
    parse_auxiliary_gripper_current_response,
    parse_auxiliary_input_command,
    parse_auxiliary_servo_command,
    parse_auxiliary_wait_command,
    parse_cartesian_motion_command,
    parse_command_speed,
    parse_command_timing,
    parse_controller_modbus_response,
    parse_joint_motion_command,
    parse_motion_wrist_config,
    parse_position_response,
    parse_tool_jog_command,
    parse_virtual_command_timing,
    primary_shutdown_position,
    submit_primary_joint_target,
    submit_primary_joint_target_text,
    validate_controller_filename,
    validate_auxiliary_output_command,
    validate_auxiliary_gripper_current_command,
    validate_auxiliary_input_command,
    validate_auxiliary_servo_command,
    validate_auxiliary_wait_command,
    validate_controller_modbus_command,
    validate_controller_encoder_scale,
)


class CalibrationSwitchProtocolTests(unittest.TestCase):
    def test_switch_mask_encodes_j1_as_low_bit(self):
        self.assertEqual(
            encode_calibration_switch_mask(
                (
                    "HIGH",
                    "LOW",
                    "HIGH",
                    "LOW",
                    "LOW",
                    "HIGH",
                    "LOW",
                    "HIGH",
                    "LOW",
                )
            ),
            165,
        )
        self.assertEqual(
            CONTROLLER_CAPABILITY_CALIBRATION_SWITCH_POLARITY_V1,
            "CALIBRATION_SWITCH_POLARITY_V1",
        )
        self.assertEqual(
            encode_calibration_switch_mask(("HIGH",) * 9),
            511,
        )
        self.assertEqual(
            encode_calibration_switch_mask(("LOW",) * 9),
            0,
        )

    def test_switch_mask_rejects_unvalidated_states(self):
        for states in (
            "HIGH",
            ("HIGH",) * 8,
            ("HIGH",) * 8 + ("high",),
            ("HIGH",) * 8 + (1,),
        ):
            with self.subTest(states=states):
                with self.assertRaises(MotionInputError):
                    encode_calibration_switch_mask(states)


TEST_CONTROLLER_MEDIA_ID = "00112233445566778899AABBCCDDEEFF"


def position_response(joints, external=(0, 0, 0), speed_violation=0, debug="", flag=""):
    joint_fields = "".join(
        label + str(value)
        for label, value in zip(("A", "B", "C", "D", "E", "F"), joints)
    )
    cartesian_fields = "G1H2I3J4K5L6"
    external_fields = "P{}Q{}R{}".format(*external)
    return (
        joint_fields
        + cartesian_fields
        + f"M{speed_violation}N{debug}O{flag}"
        + external_fields
    )


def controller_calibration(
    negative_limits=(100,) * 9,
    positive_limits=(100,) * 9,
    steps_per_unit=(100,) * 9,
):
    return ControllerJointCalibration(
        negative_limits=negative_limits,
        positive_limits=positive_limits,
        steps_per_unit=steps_per_unit,
    )


class EncoderScaleContractTests(unittest.TestCase):
    def test_encoder_scale_bounds_signed_counter_conversions(self):
        calibration = controller_calibration()

        for multiplier in (0.3125, 0.5, 1.0):
            with self.subTest(multiplier=multiplier):
                self.assertEqual(
                    validate_controller_encoder_scale(
                        calibration,
                        1,
                        multiplier,
                        "J1 encoder multiplier",
                    ),
                    multiplier,
                )
        with self.assertRaisesRegex(
            MotionInputError,
            "signed encoder counter range",
        ):
            validate_controller_encoder_scale(
                calibration,
                1,
                2147483647.0,
                "J1 encoder multiplier",
            )

    def test_encoder_scale_rejects_invalid_contract_arguments(self):
        calibration = controller_calibration()
        for axis in (False, 0, 7):
            with self.subTest(axis=axis):
                with self.assertRaises(MotionInputError):
                    validate_controller_encoder_scale(
                        calibration,
                        axis,
                        1.0,
                        "encoder multiplier",
                    )
        with self.assertRaises(MotionInputError):
            validate_controller_encoder_scale(
                object(),
                1,
                1.0,
                "encoder multiplier",
            )


def discrete_firmware_joint_duration(
    step_counts,
    profile,
    minimum_delay_microseconds=200.0,
    distribution_delay_microseconds=30.0,
):
    """Independent timing oracle for the active Teensy joint drive loop."""

    steps = tuple(step_counts)
    high_steps = max(steps)
    if high_steps == 0:
        return 0.0

    acceleration_steps = high_steps * (profile.acceleration / 100.0)
    deceleration_steps = high_steps * (profile.deceleration / 100.0)
    cruise_steps = (
        high_steps - acceleration_steps - deceleration_steps
    )
    ramp_factor = max(profile.ramp, 10.0) / 10.0
    denominator = cruise_steps + (
        acceleration_steps * (1.0 + ramp_factor)
        + deceleration_steps * (1.0 + ramp_factor)
    ) * 0.5
    if profile.speed_prefix == "Ss":
        cruise_delay = (
            profile.speed * 1_000_000.0
            / (denominator if denominator > 0 else high_steps)
        )
        cruise_delay = max(
            cruise_delay,
            minimum_delay_microseconds,
        )
    else:
        cruise_delay = minimum_delay_microseconds / (
            profile.speed / 100.0
        )

    start_delay = cruise_delay * ramp_factor
    end_delay = cruise_delay * ramp_factor
    acceleration_increment = (
        (start_delay - cruise_delay) / acceleration_steps
        if acceleration_steps > 0
        else 0.0
    )
    deceleration_increment = (
        (end_delay - cruise_delay) / deceleration_steps
        if deceleration_steps > 0
        else 0.0
    )
    current_delay = start_delay
    current = [0] * 9
    primary_error = [0] * 9
    secondary_error_1 = [0] * 9
    secondary_error_2 = [0] * 9
    elapsed_microseconds = 0.0
    high_step_current = 0

    while any(
        current[axis] < steps[axis]
        for axis in range(9)
    ):
        if high_step_current < acceleration_steps:
            current_delay = max(
                cruise_delay,
                current_delay - acceleration_increment,
            )
        elif high_step_current >= high_steps - deceleration_steps:
            current_delay = min(
                end_delay,
                current_delay + deceleration_increment,
            )
        else:
            current_delay = cruise_delay

        emitted_steps = 0
        for axis, step_count in enumerate(steps):
            if current[axis] >= step_count:
                continue
            primary_period = high_steps // step_count
            leftover_1 = high_steps - step_count * primary_period
            secondary_period_1 = (
                high_steps // leftover_1
                if leftover_1 > 0
                else 0
            )
            leftover_2 = (
                high_steps
                - (
                    step_count * primary_period
                    + (
                        step_count * primary_period
                    ) // secondary_period_1
                )
                if secondary_period_1 > 0
                else 0
            )
            secondary_period_2 = (
                high_steps // leftover_2
                if leftover_2 > 0
                else 0
            )
            if secondary_period_2 == 0:
                secondary_error_2[axis] = 1
            if secondary_error_2[axis] == secondary_period_2:
                secondary_error_2[axis] = 0
                continue
            secondary_error_2[axis] += 1
            if secondary_period_1 == 0:
                secondary_error_1[axis] = 1
            if secondary_error_1[axis] == secondary_period_1:
                secondary_error_1[axis] = 0
                continue
            secondary_error_1[axis] += 1
            primary_error[axis] += 1
            if primary_error[axis] == primary_period:
                current[axis] += 1
                primary_error[axis] = 0
                emitted_steps += 1

        distribution_delay = (
            emitted_steps * distribution_delay_microseconds
        )
        pulse_delay = math.ceil(max(
            minimum_delay_microseconds,
            current_delay - distribution_delay,
        ))
        elapsed_microseconds += distribution_delay + pulse_delay
        high_step_current += 1
        if high_step_current > high_steps * 2:
            raise AssertionError(
                "firmware timing oracle exceeded the coordinated step bound"
            )

    return elapsed_microseconds / 1_000_000.0


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true before timeout")


def collect_events_until_idle(dispatcher, timeout=2.0):
    events = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        new_events = dispatcher.drain_events()
        events.extend(new_events)
        for event in new_events:
            event.acknowledge()
        if not dispatcher.active:
            trailing_events = dispatcher.drain_events()
            events.extend(trailing_events)
            for event in trailing_events:
                event.acknowledge()
            return events
        time.sleep(0.005)
    raise AssertionError("dispatcher did not become idle before timeout")


class AuxiliaryBoardProfileTests(unittest.TestCase):
    def test_normalization_accepts_only_explicit_supported_profiles(self):
        self.assertEqual(
            normalize_auxiliary_board_profile(" Nano "),
            AUXILIARY_BOARD_NANO,
        )
        self.assertEqual(
            normalize_auxiliary_board_profile("Mega"),
            AUXILIARY_BOARD_MEGA,
        )
        for value in (None, "", "None", "  None  "):
            with self.subTest(unconfigured=value):
                self.assertIsNone(
                    normalize_auxiliary_board_profile(value, allow_none=True)
                )
                with self.assertRaises(MotionInputError):
                    normalize_auxiliary_board_profile(value)
        for value in (8, b"Nano", "nano", "Mega 2560", object()):
            with self.subTest(invalid_profile=value):
                with self.assertRaises(MotionInputError):
                    normalize_auxiliary_board_profile(value, allow_none=True)
        with self.assertRaises(TypeError):
            normalize_auxiliary_board_profile("Nano", allow_none=1)

    def test_each_board_accepts_every_configured_output_pin(self):
        for profile, output_pins in AUXILIARY_BOARD_OUTPUT_PINS.items():
            for prefix in ("ON", "OF"):
                for output_pin in output_pins:
                    command = f"{prefix}X{output_pin}\n"
                    with self.subTest(
                        profile=profile,
                        prefix=prefix,
                        output_pin=output_pin,
                    ):
                        self.assertEqual(
                            validate_auxiliary_output_command(command, profile),
                            command,
                        )
                        self.assertEqual(
                            parse_auxiliary_output_command(command),
                            (prefix, output_pin),
                        )
        self.assertEqual(
            parse_auxiliary_output_command("ONX08\n"),
            ("ON", 8),
        )

    def test_each_board_rejects_every_other_board_output_pin(self):
        profile_pairs = (
            (AUXILIARY_BOARD_NANO, AUXILIARY_BOARD_MEGA),
            (AUXILIARY_BOARD_MEGA, AUXILIARY_BOARD_NANO),
        )
        for selected_profile, other_profile in profile_pairs:
            for prefix in ("ON", "OF"):
                for output_pin in AUXILIARY_BOARD_OUTPUT_PINS[other_profile]:
                    with self.subTest(
                        selected_profile=selected_profile,
                        prefix=prefix,
                        output_pin=output_pin,
                    ):
                        with self.assertRaises(MotionInputError):
                            validate_auxiliary_output_command(
                                f"{prefix}X{output_pin}\n",
                                selected_profile,
                            )

    def test_output_validation_rejects_malformed_commands(self):
        malformed_commands = (
            "ONX8",
            "ONX8\r\n",
            " ONX8\n",
            "ONX+8\n",
            "ONX8\nOFX8\n",
            "SV0P0\n",
            "",
            None,
        )
        for command in malformed_commands:
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_output_command(
                        command,
                        AUXILIARY_BOARD_NANO,
                    )

    def test_servo_validation_enforces_each_board_profile(self):
        for profile, channels in AUXILIARY_BOARD_SERVO_CHANNELS.items():
            for channel in channels:
                for position in (0, 1, 90, 179, 180):
                    command = f"SV{channel}P{position}\n"
                    with self.subTest(
                        profile=profile,
                        channel=channel,
                        position=position,
                    ):
                        self.assertEqual(
                            validate_auxiliary_servo_command(
                                command,
                                profile,
                            ),
                            command,
                        )
                        self.assertEqual(
                            parse_auxiliary_servo_command(command),
                            (channel, position),
                        )
        self.assertEqual(
            validate_auxiliary_servo_command(
                "SV6P90\n",
                AUXILIARY_BOARD_MEGA,
            ),
            "SV6P90\n",
        )
        with self.assertRaises(MotionInputError):
            validate_auxiliary_servo_command(
                "SV6P90\n",
                AUXILIARY_BOARD_NANO,
            )
        self.assertEqual(
            parse_auxiliary_servo_command("SV00P090\n"),
            (0, 90),
        )
        self.assertEqual(
            validate_auxiliary_servo_command(
                "SV00P090\n",
                AUXILIARY_BOARD_NANO,
            ),
            "SV00P090\n",
        )

    def test_servo_validation_rejects_malformed_or_out_of_range_commands(self):
        malformed_commands = (
            "SV0P0",
            "SV0P0\r\n",
            " SV0P0\n",
            "SV+0P0\n",
            "SV0P+1\n",
            "SV0P-1\n",
            "SV0P181\n",
            "SV7P0\n",
            "SV0P0\nSV1P0\n",
            "ONX8\n",
            "",
            None,
        )
        for command in malformed_commands:
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_servo_command(
                        command,
                        AUXILIARY_BOARD_MEGA,
                    )

    def test_input_validation_enforces_each_board_profile(self):
        for profile, pins in AUXILIARY_BOARD_INPUT_PINS.items():
            for input_pin in pins:
                command = f"JFX{input_pin}\n"
                with self.subTest(profile=profile, input_pin=input_pin):
                    self.assertEqual(
                        validate_auxiliary_input_command(command, profile),
                        command,
                    )
                    self.assertEqual(
                        parse_auxiliary_input_command(command),
                        input_pin,
                    )
        for input_pin in (0, 1, 8, 27):
            with self.subTest(nano_rejected_pin=input_pin):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_input_command(
                        f"JFX{input_pin}\n",
                        AUXILIARY_BOARD_NANO,
                    )
        for input_pin in (0, 1, 28, 53):
            with self.subTest(mega_rejected_pin=input_pin):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_input_command(
                        f"JFX{input_pin}\n",
                        AUXILIARY_BOARD_MEGA,
                    )
        for command in (
            "JFX2",
            "JFX2\r\n",
            "JFX-1\n",
            "JFX+2\n",
            "JFX2\nJFX3\n",
            "JFX\n",
            "",
            None,
        ):
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_input_command(
                        command,
                        AUXILIARY_BOARD_MEGA,
                    )

    def test_wait_validation_enforces_pin_state_and_timeout_domains(self):
        for profile, input_pin in (
            (AUXILIARY_BOARD_NANO, 2),
            (AUXILIARY_BOARD_MEGA, 27),
        ):
            for state in (0, 1):
                for timeout in (1, AUXILIARY_WAIT_MAXIMUM_SECONDS):
                    command = f"WIA{input_pin}B{state}C{timeout}\n"
                    with self.subTest(
                        profile=profile,
                        state=state,
                        timeout=timeout,
                    ):
                        self.assertEqual(
                            validate_auxiliary_wait_command(command, profile),
                            command,
                        )
                        self.assertEqual(
                            parse_auxiliary_wait_command(command),
                            (input_pin, state, timeout),
                        )
        for command in (
            "WIA2B1C0",
            "WIA2B1C0\r\n",
            "WIA2B1C0\n",
            "WIA2B2C1\n",
            "WIA2B-1C1\n",
            "WIA2B1C-1\n",
            f"WIA2B1C{AUXILIARY_WAIT_MAXIMUM_SECONDS + 1}\n",
            "WIA2B1C1\nWIA2B1C1\n",
            "",
            None,
        ):
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_wait_command(
                        command,
                        AUXILIARY_BOARD_NANO,
                    )
        with self.assertRaises(MotionInputError):
            validate_auxiliary_wait_command(
                "WIA8B1C1\n",
                AUXILIARY_BOARD_NANO,
            )
        for command in ("WIA1B1C1\n", "WIA28B1C1\n"):
            with self.subTest(mega_rejected_wait=command):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_wait_command(
                        command,
                        AUXILIARY_BOARD_MEGA,
                    )

    def test_gripper_current_contract_requires_profile_and_numeric_response(self):
        for profile in (AUXILIARY_BOARD_NANO, AUXILIARY_BOARD_MEGA):
            self.assertEqual(
                validate_auxiliary_gripper_current_command("TG\n", profile),
                "TG\n",
            )
        for command in ("TG", "TG\r\n", " TG\n", "TG\nTG\n", "", None):
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    validate_auxiliary_gripper_current_command(
                        command,
                        AUXILIARY_BOARD_NANO,
                    )
        with self.assertRaises(MotionInputError):
            validate_auxiliary_gripper_current_command(
                "TG\n",
                AUXILIARY_BOARD_NONE,
            )

        for response in ("0", "0.000", "1", "1.25", "27.999", "28.000"):
            with self.subTest(response=response):
                self.assertEqual(
                    parse_auxiliary_gripper_current_response("TG\n", response),
                    response,
                )
        for response in (
            "-0.1",
            "+1",
            ".1",
            "01.0",
            "1.",
            "1.0000",
            str(AUXILIARY_CURRENT_MAXIMUM_AMPS + 0.001),
            "nan",
            "inf",
            " 1.0",
            "1.0 ",
            "",
            None,
        ):
            with self.subTest(response=response):
                with self.assertRaises(ProtocolResponseError):
                    parse_auxiliary_gripper_current_response("TG\n", response)
        with self.assertRaises(ProtocolResponseError):
            parse_auxiliary_gripper_current_response("TM\n", "1.000")

    def test_pneumatic_mapping_uses_a_valid_pin_for_each_board(self):
        expected_pins = {
            AUXILIARY_BOARD_NANO: 8,
            AUXILIARY_BOARD_MEGA: 28,
        }
        for profile, expected_pin in expected_pins.items():
            with self.subTest(profile=profile):
                self.assertEqual(
                    auxiliary_pneumatic_output_pin(profile),
                    expected_pin,
                )
                self.assertIn(
                    expected_pin,
                    AUXILIARY_BOARD_OUTPUT_PINS[profile],
                )
        with self.assertRaises(MotionInputError):
            auxiliary_pneumatic_output_pin(AUXILIARY_BOARD_NONE)


class ReleaseBarrierLock:
    def __init__(self):
        self._lock = threading.Lock()
        self.release_started = threading.Event()
        self.allow_release = threading.Event()

    def acquire(self, blocking=True):
        return self._lock.acquire(blocking)

    def release(self):
        self.release_started.set()
        if not self.allow_release.wait(2):
            raise RuntimeError("test did not allow transport release")
        self._lock.release()

    def locked(self):
        return self._lock.locked()


class VirtualMotionOperationTests(unittest.TestCase):
    def test_success_and_failure_results_are_request_scoped(self):
        successful = VirtualMotionOperation()
        failed = VirtualMotionOperation()

        self.assertFalse(successful.completed)
        with self.assertRaises(RuntimeError):
            successful.result()

        self.assertTrue(successful.complete(True))
        self.assertTrue(failed.complete(False, "virtual drive failed"))

        self.assertTrue(successful.wait(0.1))
        self.assertEqual(successful.result(), (True, None))
        self.assertEqual(failed.result(), (False, "virtual drive failed"))

    def test_terminal_result_contract_rejects_invalid_or_duplicate_data(self):
        invalid_cases = (
            (1, None),
            (True, "unexpected"),
            (False, None),
            (False, ""),
            (False, " padded "),
        )
        for succeeded, error in invalid_cases:
            with self.subTest(succeeded=succeeded, error=error):
                with self.assertRaises(MotionInputError):
                    VirtualMotionOperation().complete(succeeded, error)

        operation = VirtualMotionOperation()
        operation.complete(True)
        with self.assertRaises(RuntimeError):
            operation.complete(True)
        with self.assertRaises(MotionInputError):
            operation.wait(0)


class MotionRequestRegistryTests(unittest.TestCase):
    def test_lease_is_exclusive_and_cross_thread_releasable(self):
        registry = MotionRequestRegistry()
        lease = registry.acquire("program motion")

        self.assertIsInstance(lease, MotionRequestLease)
        self.assertTrue(registry.active)
        self.assertEqual(registry.active_name, "program motion")
        self.assertTrue(registry.owns(lease))
        self.assertIsNone(registry.acquire("manual motion"))

        result = []
        thread = threading.Thread(target=lambda: result.append(lease.close()))
        thread.start()
        thread.join(1)

        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [True])
        self.assertFalse(registry.active)
        self.assertIsNone(registry.active_name)
        self.assertFalse(registry.owns(lease))
        self.assertTrue(lease.closed)
        self.assertFalse(lease.close())
        self.assertIsInstance(registry.acquire("manual motion"), MotionRequestLease)

    def test_invalid_names_and_foreign_leases_are_rejected(self):
        registry = MotionRequestRegistry()
        for invalid in (None, "", " padded ", 1):
            with self.subTest(name=invalid):
                with self.assertRaises(MotionInputError):
                    registry.acquire(invalid)

        lease = registry.acquire("first")
        foreign = MotionRequestRegistry().acquire("foreign")
        self.assertFalse(registry.owns(foreign))
        with self.assertRaises(RuntimeError):
            registry._release(foreign)
        self.assertTrue(registry.owns(lease))


class SerialActivityRegistryTests(unittest.TestCase):
    def test_shutdown_waits_for_existing_activity_and_rejects_new_activity(self):
        registry = SerialActivityRegistry(("ser", "ser2"))

        with registry.operations(("ser",)):
            self.assertTrue(registry.active("ser"))
            self.assertFalse(registry.idle())
            self.assertTrue(registry.begin_shutdown())
            self.assertFalse(registry.begin_shutdown())
            with self.assertRaises(SerialActivityRejected):
                registry.begin("ser2")

        self.assertTrue(registry.idle())

    def test_control_reservation_distinguishes_reader_handoff_and_exclusive_exchange(self):
        registry = SerialActivityRegistry(("ser2",))

        with registry.operations(
            ("ser2",),
            control_injectable_names=("ser2",),
        ):
            mode = registry.reserve_control("ser2")
            self.assertEqual(mode, SerialActivityRegistry.CONTROL_INJECT)
            with self.assertRaises(SerialActivityRejected):
                registry.begin("ser2", control_injectable=True)
            registry.finish_control("ser2", mode)

        with registry.operations(("ser2",)):
            self.assertIsNone(registry.reserve_control("ser2"))

        mode = registry.reserve_control("ser2")
        self.assertEqual(mode, SerialActivityRegistry.CONTROL_EXCLUSIVE)
        self.assertFalse(registry.idle())
        registry.finish_control("ser2", mode)
        self.assertTrue(registry.idle())

    def test_emergency_control_remains_admissible_during_shutdown(self):
        registry = SerialActivityRegistry(("ser2",))
        registry.begin("ser2", control_injectable=True)
        registry.begin_shutdown()

        self.assertIsNone(registry.reserve_control("ser2"))
        mode = registry.reserve_emergency_control("ser2")
        self.assertEqual(mode, SerialActivityRegistry.CONTROL_INJECT)
        registry.finish_control("ser2", mode)
        registry.end("ser2", control_injectable=True)

        mode = registry.reserve_emergency_control("ser2")
        self.assertEqual(mode, SerialActivityRegistry.CONTROL_EXCLUSIVE)
        registry.finish_control("ser2", mode)
        self.assertTrue(registry.idle())

        blocked = SerialActivityRegistry(("ser2",))
        blocked.begin("ser2")
        blocked.begin_shutdown()
        self.assertIsNone(blocked.reserve_emergency_control("ser2"))
        blocked.end("ser2")

    def test_registry_rejects_invalid_names_and_unbalanced_release(self):
        with self.assertRaises(MotionInputError):
            SerialActivityRegistry("ser")
        registry = SerialActivityRegistry(("ser",))
        with self.assertRaises(MotionInputError):
            registry.active("unknown")
        with self.assertRaises(RuntimeError):
            registry.end("ser")
        with self.assertRaises(MotionInputError):
            with registry.operations(
                ("ser",),
                control_injectable_names=("unknown",),
            ):
                pass

    def test_activity_lease_spans_threads_and_closes_once(self):
        registry = SerialActivityRegistry(("ser",))
        lease = registry.lease("ser")
        results = []

        self.assertTrue(registry.active("ser"))
        worker = threading.Thread(target=lambda: results.append(lease.close()))
        worker.start()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(results, [True])
        self.assertTrue(registry.idle())
        self.assertFalse(lease.close())

    def test_activity_lease_close_retries_after_registry_release_failure(self):
        class FailOnceRegistry(SerialActivityRegistry):
            def __init__(self):
                super().__init__(("ser",))
                self.release_attempts = 0

            def end(self, serial_name, control_injectable=False):
                self.release_attempts += 1
                if self.release_attempts == 1:
                    raise RuntimeError("injected registry release failure")
                return super().end(
                    serial_name,
                    control_injectable=control_injectable,
                )

        registry = FailOnceRegistry()
        lease = registry.lease("ser")

        with self.assertRaisesRegex(
            RuntimeError,
            "injected registry release failure",
        ):
            lease.close()
        self.assertTrue(registry.active("ser"))
        self.assertTrue(lease.close())
        self.assertTrue(registry.idle())
        self.assertFalse(lease.close())


class DeferredLiveMotionArbiterTests(unittest.TestCase):
    def test_admission_retries_without_committing_rejected_input(self):
        jobs = []
        starts = []
        admissions = [False, True]

        def schedule(delay_ms, callback, reject_callback):
            jobs.append((delay_ms, callback))
            return True

        def start(desired):
            starts.append(desired)
            return admissions.pop(0)

        arbiter = DeferredLiveMotionArbiter(schedule, start, lambda: True, 60)
        desired = (1, -1)

        self.assertTrue(arbiter.request(desired))
        self.assertIsNone(arbiter.active)
        self.assertEqual(arbiter.pending, desired)
        delay, callback = jobs.pop(0)
        self.assertEqual(delay, 0)
        callback()
        self.assertEqual(starts, [desired])
        self.assertIsNone(arbiter.active)
        self.assertEqual(arbiter.pending, desired)

        delay, callback = jobs.pop(0)
        self.assertEqual(delay, 60)
        callback()
        self.assertEqual(starts, [desired, desired])
        self.assertEqual(arbiter.active, desired)
        self.assertIsNone(arbiter.pending)

    def test_rapid_replacement_keeps_value_and_command_generation_together(self):
        jobs = []
        starts = []

        def schedule(delay_ms, callback, reject_callback):
            jobs.append((delay_ms, callback))
            return True

        arbiter = DeferredLiveMotionArbiter(
            schedule,
            lambda desired: starts.append(desired) is None,
            lambda: True,
            60,
        )
        first = (1, -1)
        second = (6, 1)

        arbiter.request(first)
        first_callback = jobs.pop(0)[1]
        arbiter.request(second)
        second_callback = jobs.pop(0)[1]

        self.assertFalse(first_callback())
        self.assertEqual(starts, [])
        self.assertTrue(second_callback())
        self.assertEqual(starts, [second])
        self.assertEqual(arbiter.active, second)

    def test_cross_arbiter_retry_waits_for_previous_owner_release(self):
        jobs = []
        owner = {"value": None}
        stops = []

        def schedule(delay_ms, callback, reject_callback):
            jobs.append((delay_ms, callback))
            return True

        def stop():
            stops.append(owner["value"])
            owner["value"] = None
            return True

        def start(name):
            if owner["value"] is not None:
                return False
            owner["value"] = name
            return True

        joint = DeferredLiveMotionArbiter(schedule, start, stop, 60)
        tool = DeferredLiveMotionArbiter(schedule, start, stop, 60)
        joint.request("joint")
        jobs.pop(0)[1]()
        self.assertEqual(owner["value"], "joint")

        tool.request("tool")
        jobs.pop(0)[1]()
        self.assertIsNone(tool.active)
        self.assertEqual(tool.pending, "tool")

        joint.request(None)
        self.assertEqual(stops, ["joint"])
        jobs.pop(0)[1]()
        self.assertEqual(owner["value"], "tool")
        self.assertEqual(tool.active, "tool")

    def test_schedule_failure_retains_pending_intent_for_next_poll(self):
        attempts = []
        errors = []

        def schedule(delay_ms, callback, reject_callback):
            attempts.append((delay_ms, callback))
            return len(attempts) > 1

        arbiter = DeferredLiveMotionArbiter(
            schedule,
            lambda desired: True,
            lambda: True,
            60,
            errors.append,
        )

        self.assertFalse(arbiter.request("joint"))
        self.assertEqual(arbiter.pending, "joint")
        self.assertEqual(errors, [arbiter.last_error])
        self.assertIn(
            "live-motion scheduling callback rejected the request",
            arbiter.last_error,
        )
        self.assertTrue(arbiter.request("joint"))
        attempts[-1][1]()
        self.assertEqual(arbiter.active, "joint")

    def test_deferred_schedule_failure_releases_retry_and_reports_error(self):
        jobs = []
        errors = []

        def schedule(delay_ms, callback, reject_callback):
            jobs.append((delay_ms, callback, reject_callback))
            return True

        arbiter = DeferredLiveMotionArbiter(
            schedule,
            lambda desired: True,
            lambda: True,
            60,
            errors.append,
        )

        self.assertTrue(arbiter.request("joint"))
        self.assertTrue(arbiter.snapshot().attempt_scheduled)
        self.assertTrue(jobs[0][2](RuntimeError("Tk registration failed")))
        self.assertFalse(arbiter.snapshot().attempt_scheduled)
        self.assertEqual(arbiter.pending, "joint")
        self.assertEqual(errors, [arbiter.last_error])
        self.assertIn("live-motion deferred scheduling failed", arbiter.last_error)

        self.assertTrue(arbiter.request("joint"))
        self.assertEqual(len(jobs), 2)
        self.assertTrue(jobs[1][1]())
        self.assertEqual(arbiter.active, "joint")

    def test_deferred_schedule_cancellation_clears_pending_without_error(self):
        jobs = []
        errors = []

        def schedule(delay_ms, callback, reject_callback):
            jobs.append((delay_ms, callback, reject_callback))
            return True

        arbiter = DeferredLiveMotionArbiter(
            schedule,
            lambda desired: True,
            lambda: True,
            60,
            errors.append,
        )

        self.assertTrue(arbiter.request("joint"))
        self.assertTrue(arbiter.snapshot().attempt_scheduled)
        self.assertTrue(jobs[0][2](LiveMotionScheduleResult.CANCELLED))
        self.assertFalse(arbiter.snapshot().attempt_scheduled)
        self.assertIsNone(arbiter.pending)
        self.assertIsNone(arbiter.last_error)
        self.assertEqual(errors, [])

    def test_application_close_cancels_pending_schedule_without_error(self):
        errors = []
        arbiter = DeferredLiveMotionArbiter(
            lambda delay_ms, callback, reject_callback: (
                LiveMotionScheduleResult.CANCELLED
            ),
            lambda desired: True,
            lambda: True,
            60,
            errors.append,
        )

        self.assertFalse(arbiter.request("joint"))
        self.assertIsNone(arbiter.pending)
        self.assertIsNone(arbiter.last_error)
        self.assertEqual(errors, [])

    def test_failed_stop_retains_active_owner_and_reports_error(self):
        jobs = []
        errors = []
        stop_results = [False, True]

        def schedule(delay_ms, callback, reject_callback):
            jobs.append((delay_ms, callback))
            return True

        arbiter = DeferredLiveMotionArbiter(
            schedule,
            lambda desired: True,
            lambda: stop_results.pop(0),
            60,
            errors.append,
        )
        arbiter.request("joint")
        jobs.pop(0)[1]()

        self.assertFalse(arbiter.request("tool"))
        self.assertEqual(arbiter.active, "joint")
        self.assertEqual(arbiter.pending, "tool")
        self.assertEqual(errors, [arbiter.last_error])

        self.assertTrue(arbiter.request("tool"))
        delay, callback = jobs.pop(0)
        self.assertEqual(delay, 60)
        self.assertTrue(callback())
        self.assertEqual(arbiter.active, "tool")
        self.assertIsNone(arbiter.pending)


class CommandResponseTimeoutTests(unittest.TestCase):
    def test_modbus_command_builder_matches_firmware_request_domain(self):
        accepted = {
            ("BA", "1", "0", "1"): "BAA1B0C1\n",
            ("BB", 1, 65535, 1): "BBA1B65535C1\n",
            ("BC", 247, 65535, 1): "BCA247B65535C1\n",
            ("BH", 1, 65535, 1): "BHA1B65535C1\n",
            ("BD", 1, 65472, 1): "BDA1B65472C1\n",
            ("BE", 1, 0, 0): "BEA1B0C0\n",
            ("BF", 247, 65535, 65535): "BFA247B65535C65535\n",
            ("SC", 1, 0, 1): "SCA1B0C1\n",
            ("SO", 247, 65535, 65535): "SOA247B65535C65535\n",
        }
        for arguments, expected in accepted.items():
            with self.subTest(arguments=arguments):
                command = build_controller_modbus_command(*arguments)
                self.assertEqual(command, expected)
                self.assertEqual(
                    validate_controller_modbus_command(command),
                    command,
                )
        self.assertEqual(
            build_controller_modbus_command(
                "BF",
                "0" * 10000 + "1",
                "0" * 10000,
                "0" * 10000 + "1",
            ),
            "BFA1B0C1\n",
        )

        rejected = (
            (("BG", 1, 0, 1), "opcode"),
            (("BA", 0, 0, 1), "slave ID"),
            (("BA", 248, 0, 1), "slave ID"),
            (("BA", 1, -1, 1), "address"),
            (("BA", 1, 65536, 1), "address"),
            (("BA", 1, 0, 0), "quantity"),
            (("BA", 1, 0, 2), "quantity"),
            (("BH", 1, 0, 64), "quantity"),
            (("BD", 1, 0, 65), "quantity"),
            (("BA", 1, 0, 65), "quantity"),
            (("BB", 1, 0, 0), "must be 1"),
            (("BC", 1, 0, 2), "must be 1"),
            (("BE", 1, 0, 2), "must be 0 or 1"),
            (("SC", 1, 0, 2), "must be 0 or 1"),
            (("BF", 1, 0, 65536), "register value"),
            (("SO", 1, 0, 65536), "register value"),
            (("BF", " 1", 0, 1), "unsigned decimal"),
            (("BF", "9" * 10000, 0, 1), "protocol range"),
            (("BF", True, 0, 1), "must be an integer"),
        )
        for arguments, message in rejected:
            with self.subTest(arguments=arguments):
                with self.assertRaisesRegex(MotionInputError, message):
                    build_controller_modbus_command(*arguments)

        for command in (
            "BAA01B0C1\n",
            "BAA1B00C1\n",
            "BAA1B0C01\n",
            "BAA1B0C1",
            "BAA1B0C1\r\n",
            "BAA1B0C1\nextra",
        ):
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    validate_controller_modbus_command(command)

    def test_modbus_response_contract_rejects_every_non_success_shape(self):
        accepted = {
            "BA": "65535",
            "BB": "1",
            "BC": "0",
            "BH": "42",
            "BD": "65535",
            "BE": "Write Success",
            "BF": "Write Success",
            "SC": "1",
            "SO": "1",
            "WJ": "Done",
            "WK": "Done",
        }
        for opcode, response in accepted.items():
            with self.subTest(opcode=opcode):
                self.assertEqual(
                    parse_controller_modbus_response(
                        f"{opcode}A1B0C1\n",
                        response,
                    ),
                    response,
                )

        for opcode in accepted:
            for response in ("ER", "Modbus Error", "-1", "", "Done "):
                with self.subTest(opcode=opcode, response=response):
                    with self.assertRaises(ProtocolResponseError):
                        parse_controller_modbus_response(
                            f"{opcode}A1B0C1\n",
                            response,
                        )

        for opcode, response in (("BB", "2"), ("BC", "01"), ("BH", "65536")):
            with self.subTest(opcode=opcode, response=response):
                with self.assertRaises(ProtocolResponseError):
                    parse_controller_modbus_response(
                        f"{opcode}A1B0C1\n",
                        response,
                    )

    def test_modbus_terminal_classification_preserves_write_uncertainty(self):
        for command, response, expected in (
            ("BAA1B0C1\n", "42", "completed"),
            ("BAA1B0C1\n", "ER", "rejected"),
            ("BAA1B0C1\n", "Modbus Error", "rejected"),
            ("BEA1B0C1\n", "ER", "rejected"),
            ("BEA1B0C1\n", "Modbus Error", "indeterminate"),
            ("BFA1B0C1\n", "Write Success", "completed"),
            ("SCA1B0C1\n", "-1", "indeterminate"),
            ("SOA1B0C1\n", "-2", "rejected"),
        ):
            with self.subTest(command=command, response=response):
                self.assertEqual(
                    classify_controller_modbus_terminal_response(
                        command,
                        response,
                    ),
                    expected,
                )
        self.assertFalse(
            controller_modbus_command_is_write("BAA1B0C1\n")
        )
        self.assertTrue(
            controller_modbus_command_is_write("BEA1B0C1\n")
        )
        self.assertTrue(
            controller_modbus_command_is_write("SCA1B0C1\n")
        )

        for opcode in ("BE", "BF"):
            command = f"{opcode}A1B0C1\n"
            with self.subTest(paired_write_opcode=opcode):
                self.assertEqual(
                    classify_controller_modbus_terminal_response(
                        command,
                        "ER",
                        paired_with_estop=True,
                    ),
                    "indeterminate",
                )
        self.assertEqual(
            classify_controller_modbus_terminal_response(
                "BAA1B0C1\n",
                "ER",
                paired_with_estop=True,
            ),
            "rejected",
        )
        with self.assertRaisesRegex(TypeError, "paired Modbus E-stop"):
            classify_controller_modbus_terminal_response(
                "BEA1B0C1\n",
                "ER",
                paired_with_estop=1,
            )

    @staticmethod
    def joint_command(timing):
        return f"RJA1B2C3D4E5F6J70J80J90{timing}WNLm000000\n"

    def test_parses_supported_timing_profiles(self):
        standard = self.joint_command("Ss200Ac10Dc20Rm25")
        legacy_jog = "JTX11Sp50G10H20I25WNLm000000\n"

        self.assertEqual(
            parse_command_timing(standard),
            CommandTiming("s", 200.0, 10.0, 20.0, 25.0),
        )
        self.assertEqual(
            parse_command_timing(legacy_jog),
            CommandTiming("p", 50.0, 10.0, 20.0, 25.0),
        )
        self.assertEqual(
            parse_command_speed(standard),
            ("s", 200.0),
        )
        self.assertEqual(parse_command_speed("RP\n"), None)

    def test_parses_joint_motion_for_the_json_migration_boundary(self):
        command = (
            "RJA1B-2C3D-4E5F-6J71.25J80J9-0.5"
            "Ss1.5Ac10Dc20Rm25WFLm010101\n"
        )

        parsed = parse_joint_motion_command(command)

        self.assertIsInstance(parsed, JointMotionCommand)
        self.assertEqual(
            parsed.robot_joints_degrees,
            (1.0, -2.0, 3.0, -4.0, 5.0, -6.0),
        )
        self.assertEqual(parsed.external_axes_units, (1.25, 0.0, -0.5))
        self.assertEqual(
            parsed.timing,
            CommandTiming("s", 1.5, 10.0, 20.0, 25.0),
        )
        self.assertEqual(parsed.wrist_configuration, "F")
        self.assertEqual(
            parsed.loop_modes,
            (False, True, False, True, False, True),
        )

        telemetry_command = command[:-1] + "T1\n"
        self.assertEqual(
            parse_joint_motion_command(telemetry_command),
            parsed,
        )
        for timing_text, expected_timing in (
            ("Sp50", CommandTiming("p", 50.0, 10.0, 20.0, 25.0)),
            ("Ss1.5", CommandTiming("s", 1.5, 10.0, 20.0, 25.0)),
            ("Sm25", CommandTiming("m", 25.0, 10.0, 20.0, 25.0)),
        ):
            for wrist in ("A", "N", "F"):
                variant = (
                    "RJA1B-2C3D-4E5F-6J71.25J80J9-0.5"
                    f"{timing_text}Ac10Dc20Rm25W{wrist}Lm010101\n"
                )
                with self.subTest(timing=timing_text, wrist=wrist):
                    parsed_variant = parse_joint_motion_command(variant)
                    self.assertEqual(parsed_variant.timing, expected_timing)
                    self.assertEqual(
                        parsed_variant.wrist_configuration,
                        wrist,
                    )

        with self.assertRaises(MotionInputError):
            parse_joint_motion_command("MJ" + command[2:])

    def test_motion_wrist_config_comes_from_validated_command(self):
        controller = self.joint_command("Sp50Ac10Dc20Rm25")
        virtual = "JTX11Sp50G10H20I25WFLm000000\n"
        vision = (
            "MVX1Y2Z3Rz4Ry5Rx6J70J80J90"
            "Sp50Ac10Dc20Rm25WNVr-12.5Lm000000\n"
        )

        self.assertEqual(parse_motion_wrist_config(controller), "N")
        self.assertEqual(
            parse_motion_wrist_config(virtual, virtual=True),
            "F",
        )
        self.assertEqual(parse_motion_wrist_config(vision), "N")
        with self.assertRaises(MotionInputError):
            parse_motion_wrist_config(
                "JTX11Sp50G10H20I25Lm000000\n",
                virtual=True,
            )

    def test_tool_roll_direction_does_not_shadow_wrist_suffix(self):
        command = "JTW11Sp50G10H20I25WFLm000000\n"

        self.assertEqual(
            parse_virtual_command_timing(command),
            CommandTiming("p", 50.0, 10.0, 20.0, 25.0),
        )
        self.assertEqual(
            parse_motion_wrist_config(command, virtual=True),
            "F",
        )

    def test_negative_tool_distance_is_rejected_at_both_host_boundaries(self):
        command = "JTW1-1Sp50G10H20I25WFLm000000\n"

        with self.assertRaises(MotionInputError):
            canonicalize_serial_command(command)
        with self.assertRaises(MotionInputError):
            canonicalize_virtual_command(command)

    def test_motion_angles_reject_controller_radian_underflow(self):
        tiny_angle = controller_protocol_decimal(1e-44, "tiny angle")
        timing = "Sp50Ac10Dc20Rm25"
        cartesian = (
            f"MJX1Y2Z3Rz{tiny_angle}Ry0Rx0J70J80J90"
            f"{timing}WNLm000000\n"
        )
        vision = (
            "MVX1Y2Z3Rz0Ry0Rx0J70J80J90"
            f"{timing}WNVr{tiny_angle}Lm000000\n"
        )
        discrete_tool = (
            f"JTW1{tiny_angle}Sp50G10H20I25WFLm000000\n"
        )
        virtual_cartesian = (
            f"MJX1Y2Z3Rz{tiny_angle}Ry0Rx0"
            f"{timing}WNLm000000\n"
        )

        for command, virtual in (
            (cartesian, False),
            (vision, False),
            (discrete_tool, False),
            (virtual_cartesian, True),
            (discrete_tool, True),
        ):
            with self.subTest(opcode=command[:2], virtual=virtual):
                canonicalize = (
                    canonicalize_virtual_command
                    if virtual
                    else canonicalize_serial_command
                )
                with self.assertRaisesRegex(
                    MotionInputError,
                    "native radians",
                ):
                    canonicalize(command)

        translation = f"JTX1{tiny_angle}Sp50G10H20I25WFLm000000\n"
        self.assertIn(tiny_angle, canonicalize_serial_command(translation))
        self.assertIn(tiny_angle, canonicalize_virtual_command(translation))

    def test_controller_angle_conversion_rejects_native_degree_overflow(self):
        largest_accepted_rotation = float.fromhex("0x1.fffffcp+127")
        maximum_float32 = float.fromhex("0x1.fffffep+127")

        self.assertTrue(
            math.isfinite(
                controller_degree_to_native_radians(
                    largest_accepted_rotation,
                    "test rotation",
                )
            )
        )
        with self.assertRaisesRegex(MotionInputError, "native degrees"):
            controller_degree_to_native_radians(
                maximum_float32,
                "test rotation",
            )

    def test_opcode_schema_ignores_timing_like_filename_payload(self):
        playback = "PGFnSampleGcode.txt\n"
        write_command = (
            "WCX1Y2Z3Rz4Ry5Rx6J70J80J90"
            "Sp50Ac10Dc20Rm25WNLm000000"
            f"Mi{TEST_CONTROLLER_MEDIA_ID}FnSampleGcode.txt\n"
        )

        self.assertIsNone(parse_command_timing(playback))
        self.assertEqual(
            parse_command_timing(write_command),
            CommandTiming("p", 50.0, 10.0, 20.0, 25.0),
        )

    def test_opcode_schemas_accept_supported_motion_envelopes(self):
        timing = "Sp50Ac10Dc20Rm25"
        cartesian = "X1Y2Z3Rz4Ry5Rx6J70J80J90"
        commands = (
            f"RJA1B2C3D4E5F6J70J80J90{timing}WNLm000000\n",
            f"MJ{cartesian}{timing}WNLm000000\n",
            f"MJ{cartesian}{timing}WALm000000\n",
            f"ML{cartesian}{timing}Rnd0WNLm000000Q0\n",
            f"MV{cartesian}{timing}WNVr0Lm000000\n",
            (
                f"WC{cartesian}{timing}WNLm000000"
                f"Mi{TEST_CONTROLLER_MEDIA_ID}Fndemo.txt\n"
            ),
            f"LJV10{timing}WALm000000\n",
            f"LCV20{timing}WALm000000\n",
            f"LTV30{timing}WALm000000\n",
        )
        expected = CommandTiming("p", 50.0, 10.0, 20.0, 25.0)
        for command in commands:
            with self.subTest(command=command[:2]):
                self.assertEqual(parse_command_timing(command), expected)

    def test_storage_write_commands_require_canonical_media_identity(self):
        timing = "Sp50Ac10Dc20Rm25"
        cartesian = "X1Y2Z3Rz4Ry5Rx6J70J80J90"
        calibration = controller_calibration()
        invalid_targets = (
            "Fndemo.txt",
            f"Mi{TEST_CONTROLLER_MEDIA_ID.lower()}Fndemo.txt",
            f"Mi{TEST_CONTROLLER_MEDIA_ID[:-1]}Fndemo.txt",
        )

        for opcode in ("WC",):
            for target in invalid_targets:
                with self.subTest(opcode=opcode, target=target):
                    command = (
                        f"{opcode}{cartesian}{timing}"
                        f"WNLm000000{target}\n"
                    )
                    with self.assertRaisesRegex(
                        MotionInputError,
                        "invalid fields after timing",
                    ):
                        canonicalize_serial_command(
                            command,
                            calibration,
                        )

    def test_live_jog_domains_match_firmware(self):
        timing = "Sp50Ac10Dc20Rm25"
        for opcode, maximum_axis, wrist in (
            ("LC", 6, "N"),
            ("LJ", 9, "A"),
            ("LT", 6, "F"),
        ):
            for vector in (10, 11, maximum_axis * 10, maximum_axis * 10 + 1):
                with self.subTest(opcode=opcode, vector=vector):
                    command = (
                        f"{opcode}V{vector}{timing}W{wrist}Lm000000\n"
                    )
                    self.assertEqual(
                        parse_command_timing(command),
                        CommandTiming("p", 50.0, 10.0, 20.0, 25.0),
                    )

        rejected = (
            "LCV10Ss50Ac10Dc20Rm25WNLm000000\n",
            "LJV10Sm50Ac10Dc20Rm25WALm000000\n",
            "LTV0.1Sp50Ac10Dc20Rm25WFLm000000\n",
            "LCV9Sp50Ac10Dc20Rm25WNLm000000\n",
            "LJV100Sp50Ac10Dc20Rm25WALm000000\n",
            "LTV62Sp50Ac10Dc20Rm25WFLm000000\n",
        )
        for command in rejected:
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    parse_command_timing(command)

    def test_unsupported_motion_fields_fail_closed(self):
        timing = "Sp50Ac10Dc20Rm25"
        cartesian = "X1Y2Z3Rz4Ry5Rx6J70J80J90"

        unsupported_suffixes = (
            f"MJ{cartesian}{timing}Rnd1WNLm000000\n",
            f"ML{cartesian}{timing}Rnd0WNLm000000Q1\n",
            f"MV{cartesian}{timing}Rnd1WNVr0Lm000000\n",
            (
                f"WC{cartesian}{timing}Rnd1WNLm000000"
                f"Mi{TEST_CONTROLLER_MEDIA_ID}Fndemo.txt\n"
            ),
            f"LJV10{timing}WNLm000000\n",
            f"LJV10{timing}WFLm000000\n",
        )
        for command in unsupported_suffixes:
            with self.subTest(command=command[:2]):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "invalid fields after timing",
                ):
                    parse_command_timing(command)
        with self.assertRaisesRegex(MotionInputError, "rounding"):
            parse_command_timing(
                f"ML{cartesian}{timing}Rnd-1WNLm000000Q0\n"
            )

    def test_canonicalizes_every_controller_numeric_field(self):
        command = (
            "RJA0.1B2C3D4E5F6J70J80J90"
            "Sp50Ac10Dc20Rm25WNLm000000\n"
        )

        canonical = canonicalize_serial_command(
            command,
            controller_calibration(),
        )

        expected_j1 = controller_protocol_decimal("0.1", "test value")
        self.assertTrue(canonical.startswith(f"RJA{expected_j1}B2C3"))
        self.assertEqual(parse_command_timing(canonical).speed, 50.0)

    def test_target_bearing_commands_require_active_calibration(self):
        timing = "Sp50Ac10Dc20Rm25"
        commands = (
            f"RJA1B2C3D4E5F6J70J80J90{timing}WNLm000000\n",
            f"MJX1Y2Z3Rz4Ry5Rx6J70J80J90{timing}WNLm000000\n",
        )

        for command in commands:
            with self.subTest(opcode=command[:2]):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "require controller calibration",
                ):
                    canonicalize_serial_command(command)

    def test_target_bearing_commands_validate_calibrated_axis_ranges(self):
        timing = "Sp50Ac10Dc20Rm25"
        calibration = controller_calibration(
            negative_limits=(10,) * 9,
            positive_limits=(10,) * 9,
        )
        valid_joint = f"RJA1B2C3D4E5F6J77J88J99{timing}WNLm000000\n"
        invalid_joint = f"RJA11B2C3D4E5F6J77J88J99{timing}WNLm000000\n"
        invalid_external = (
            f"MJX1Y2Z3Rz4Ry5Rx6J711J88J99{timing}WNLm000000\n"
        )

        self.assertEqual(
            canonicalize_serial_command(valid_joint, calibration),
            valid_joint,
        )
        with self.assertRaisesRegex(MotionInputError, "J1 position"):
            canonicalize_serial_command(invalid_joint, calibration)
        with self.assertRaisesRegex(MotionInputError, "J7 position"):
            canonicalize_serial_command(invalid_external, calibration)

    def test_controller_filenames_reject_reserved_characters(self):
        timing = "Sp50Ac10Dc20Rm25"
        cartesian = "X1Y2Z3Rz4Ry5Rx6J70J80J90"
        calibration = controller_calibration()
        for reserved in f'"*/:<>?\\|{CONTROLLER_DIRECTORY_SEPARATOR}':
            with self.subTest(reserved=reserved):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "controller-reserved",
                ):
                    validate_controller_filename(
                        f"demo{reserved}file.txt",
                        "test filename",
                    )
        commands = (
            "PGFn../demo.txt\n",
            "PGFnC:demo.txt\n",
            (
                f"WC{cartesian}{timing}WNLm000000"
                f"Mi{TEST_CONTROLLER_MEDIA_ID}Fnfolder/demo.txt\n"
            ),
            (
                f"WC{cartesian}{timing}WNLm000000"
                f"Mi{TEST_CONTROLLER_MEDIA_ID}Fnfolder/evilFndemo.txt\n"
            ),
            "PGFndemo,evil.txt\n",
            (
                f"WC{cartesian}{timing}WNLm000000"
                f"Mi{TEST_CONTROLLER_MEDIA_ID}Fndemo,evil.txt\n"
            ),
        )

        for command in commands:
            with self.subTest(opcode=command[:2]):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "controller-reserved",
                ):
                    canonicalize_serial_command(command, calibration)
        for filename in (" demo.txt", "demo.txt "):
            with self.subTest(filename=filename):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "controller-reserved",
                ):
                    validate_controller_filename(filename, "test filename")
        self.assertEqual(
            validate_controller_filename("demo .txt", "test filename"),
            "demo .txt",
        )

    def test_controller_filename_byte_limit_matches_storage_commands(self):
        self.assertEqual(
            MAX_CONTROLLER_DIRECTORY_PAYLOAD_BYTES,
            MAX_RESPONSE_PAYLOAD_LENGTH,
        )
        timing = "Sp50Ac10Dc20Rm25"
        cartesian = "X1Y2Z3Rz4Ry5Rx6J70J80J90"
        calibration = controller_calibration()
        maximum = "a" * MAX_CONTROLLER_FILENAME_BYTES
        oversized = maximum + "a"

        self.assertEqual(
            validate_controller_filename(maximum, "test filename"),
            maximum,
        )
        for command in (
            f"PGFn{maximum}\n",
            (
                f"WC{cartesian}{timing}WNLm000000"
                f"Mi{TEST_CONTROLLER_MEDIA_ID}Fn{maximum}\n"
            ),
        ):
            with self.subTest(accepted=command[:2]):
                self.assertEqual(
                    canonicalize_serial_command(command, calibration),
                    command,
                )

        for command in (
            f"PGFn{oversized}\n",
            (
                f"WC{cartesian}{timing}WNLm000000"
                f"Mi{TEST_CONTROLLER_MEDIA_ID}Fn{oversized}\n"
            ),
        ):
            with self.subTest(rejected=command[:2]):
                with self.assertRaisesRegex(MotionInputError, "encoded bytes"):
                    canonicalize_serial_command(command, calibration)

    def test_rejects_ambiguous_or_unrepresentable_envelope_numbers(self):
        timing = "Sp50Ac10Dc20Rm25"
        overflow = "340282400000000000000000000000000000000"
        underflow = "0.00000000000000000000000000000000000000000000000001"
        commands = (
            "RJA1B2C3D1E1E2F6J70J80J90"
            f"{timing}WNLm000000\n",
            f"RJA{overflow}B2C3D4E5F6J70J80J90{timing}WNLm000000\n",
            f"RJA{underflow}B2C3D4E5F6J70J80J90{timing}WNLm000000\n",
            (
                "MLX1Y2Z3Rz4Ry5Rx6J70J80J90"
                f"{timing}Rnd{overflow}WNLm000000Q0\n"
            ),
            f"JTX1{overflow}{timing}WNLm000000\n",
        )

        for command in commands:
            with self.subTest(command=command[:24]):
                with self.assertRaises(MotionInputError):
                    parse_command_timing(command)

        virtual = f"RJA{overflow}B2C3D4E5F6{timing}WNLm000000\n"
        with self.assertRaises(MotionInputError):
            parse_virtual_command_timing(virtual)

    def test_virtual_timing_separates_simulator_and_controller_envelopes(self):
        timing = "Sp50Ac10Dc20Rm25"
        virtual_commands = (
            f"RJA1B2C3D4E5F6{timing}WNLm000000\n",
            f"MJX1Y2Z3Rz4Ry5Rx6{timing}WALm000000\n",
            "JTX11Sp50G10H20I25WNLm000000\n",
            "JTX01Sp50G10H20I25WNLm000000\n",
        )
        expected = CommandTiming("p", 50.0, 10.0, 20.0, 25.0)

        for command in virtual_commands:
            with self.subTest(command=command[:2]):
                self.assertEqual(parse_virtual_command_timing(command), expected)

        cartesian = parse_cartesian_motion_command(virtual_commands[1], virtual=True)
        self.assertEqual(cartesian.translation_millimeters, (1.0, 2.0, 3.0))
        self.assertEqual(cartesian.orientation_degrees, (6.0, 5.0, 4.0))
        self.assertEqual(cartesian.external_axes_units, (0.0, 0.0, 0.0))
        self.assertEqual(cartesian.timing, expected)
        self.assertEqual(cartesian.wrist_configuration, "A")

        tool_jog = parse_tool_jog_command(virtual_commands[2], virtual=True)
        self.assertEqual(tool_jog.axis, "x")
        self.assertEqual(tool_jog.direction, "positive")
        self.assertEqual(tool_jog.distance, 1.0)
        self.assertEqual(tool_jog.timing, expected)
        self.assertEqual(tool_jog.wrist_configuration, "N")
        self.assertEqual(
            parse_tool_jog_command(virtual_commands[3], virtual=True).direction,
            "negative",
        )

        vision = parse_cartesian_motion_command(
            f"MVX1Y2Z3Rz4Ry5Rx6{timing}WNVr15Lm010101\n",
            virtual=True,
        )
        self.assertEqual(vision.translation_millimeters, (1.0, 2.0, 3.0))
        self.assertEqual(vision.orientation_degrees, (6.0, 5.0, 4.0))
        self.assertEqual(vision.wrist_configuration, "N")
        self.assertEqual(
            vision.loop_modes,
            (False, True, False, True, False, True),
        )
        with self.assertRaises(MotionInputError):
            parse_command_timing(virtual_commands[0])
        with self.assertRaises(MotionInputError):
            parse_command_timing(virtual_commands[1])
        with self.assertRaises(MotionInputError):
            parse_virtual_command_timing(
                f"RJA1B2C3D4E5F6J70J80J90{timing}WNLm000000\n"
            )
        with self.assertRaises(MotionInputError):
            parse_virtual_command_timing(
                f"MJX1Y2Z3Rz4Ry5Rx6J70J80J90{timing}WNLm000000\n"
            )

    def test_motion_opcode_requires_complete_fields_and_timing(self):
        commands = (
            "RJA1B2C3D4E5F6J70J80J90WN\n",
            "RJA1B2C3D4E5F6J70J80Sp50Ac10Dc20Rm25WNLm000000\n",
            "ZZSp50Ac10Dc20Rm25WNLm000000\n",
        )
        for command in commands:
            with self.subTest(command=command):
                with self.assertRaises(MotionInputError):
                    parse_command_timing(command)

    def test_rejects_invalid_timing_fields(self):
        timings = (
            "Sp0Ac10Dc20Rm25",
            "Sp101Ac10Dc20Rm25",
            "Sp50Ac0Dc20Rm25",
            "Sp50Ac10Dc100Rm25",
            "Sp50Ac60Dc41Rm25",
            "Sp50Ac10Dc20Rm0",
            "Sp50Ac10Dc20Rm100.1",
            "SpnanAc10Dc20Rm25",
            "Sx50Ac10Dc20Rm25",
            "Sp50Ac10Dc20",
        )
        for timing in timings:
            with self.subTest(timing=timing):
                with self.assertRaises(MotionInputError):
                    parse_command_timing(self.joint_command(timing))

    def test_typed_and_raw_ramp_contracts_share_firmware_boundaries(self):
        maximum = CONTROLLER_MAXIMUM_RAMP_PERCENT
        command = self.joint_command(f"Sp50Ac10Dc20Rm{maximum:g}")

        self.assertEqual(parse_command_timing(command).ramp, maximum)
        self.assertEqual(
            MotionProfile("Sp", 50, 10, 20, maximum, "N", "000000").ramp,
            maximum,
        )
        for rejected in (0, maximum + 0.00001, 200):
            with self.subTest(rejected=rejected):
                with self.assertRaises(MotionInputError):
                    parse_command_timing(
                        self.joint_command(
                            f"Sp50Ac10Dc20Rm{rejected:.17g}"
                        )
                    )
                with self.assertRaises(MotionInputError):
                    MotionProfile(
                        "Sp",
                        50,
                        10,
                        20,
                        rejected,
                        "N",
                        "000000",
                    )

    def test_rejects_overlapping_acceleration_and_deceleration_regions(self):
        boundary = self.joint_command("Sp50Ac60Dc40Rm25")
        overlapping = self.joint_command("Sp50Ac60Dc40.1Rm25")

        self.assertEqual(
            parse_command_timing(boundary),
            CommandTiming("p", 50.0, 60.0, 40.0, 25.0),
        )
        MotionProfile("Sp", 50, 60, 40, 25, "N", "000000")
        with self.assertRaisesRegex(MotionInputError, "must not overlap"):
            parse_command_timing(overlapping)
        with self.assertRaisesRegex(MotionInputError, "must not overlap"):
            MotionProfile("Sp", 50, 60, 40.1, 25, "N", "000000")

    def test_timing_limits_use_the_controller_float32_values(self):
        with self.assertRaisesRegex(MotionInputError, "deceleration must be"):
            MotionProfile("Sp", 50, 10, 99.999999, 25, "N", "000000")
        with self.assertRaisesRegex(MotionInputError, "deceleration must be"):
            motion_timing_response_timeout(
                CommandTiming("p", 50, 10, 99.999999, 25),
                120,
                100,
                1000,
            )

        profile = MotionProfile(
            "Sp",
            50,
            60.000001,
            40.000001,
            25,
            "N",
            "000000",
        )
        self.assertEqual(profile.acceleration, 60.0)
        self.assertEqual(profile.deceleration, 40.0)
        self.assertIn("Ac60Dc40", profile.protocol_suffix())

        with self.assertRaisesRegex(MotionInputError, "must not overlap"):
            MotionProfile("Sp", 50, 60, 40.000003, 25, "N", "000000")

    def test_rejects_boolean_numeric_values(self):
        for value in (False, True):
            with self.subTest(value=value):
                with self.assertRaises(MotionInputError):
                    finite_number(value, "test value")
                with self.assertRaises(MotionInputError):
                    MotionProfile("Sp", value, 10, 10, 25, "N", "000000")

        positions = [0.0] * 9
        positions[3] = True
        with self.assertRaises(MotionInputError):
            build_robot_joint_command(positions, MotionProfile(
                "Sp", 50, 10, 10, 25, "N", "000000"
            ), controller_calibration())

    def test_rejects_values_outside_the_controller_float_range(self):
        with self.assertRaisesRegex(MotionInputError, "host numeric range"):
            finite_number(10**400, "test value")
        with self.assertRaisesRegex(MotionInputError, "finite float range"):
            controller_number("1e39", "test value")
        with self.assertRaisesRegex(MotionInputError, "finite float range"):
            MotionProfile("Ss", "1e39", 10, 10, 25, "N", "000000")
        with self.assertRaisesRegex(MotionInputError, "finite float range"):
            parse_command_timing(
                self.joint_command(
                    "Ss1000000000000000000000000000000000000000"
                    "Ac10Dc20Rm25"
                )
            )
        with self.assertRaisesRegex(MotionInputError, "represented by the controller"):
            controller_number("1e-50", "test value")

    def test_rejects_exact_nonzero_values_lost_during_host_float_conversion(self):
        values = (
            Decimal("1e-10000"),
            "1e-10000",
            "0." + ("0" * 10000) + "1",
        )
        for value in values:
            with self.subTest(value_type=type(value).__name__):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "represented by the controller",
                ):
                    controller_number(value, "test value")
                with self.assertRaisesRegex(
                    MotionInputError,
                    "host numeric range",
                ):
                    finite_number(value, "test value")

    def test_rejects_exact_finite_values_above_the_host_float_range(self):
        values = (Decimal("1e10000"), "1e10000")
        for value in values:
            with self.subTest(value_type=type(value).__name__):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "host numeric range",
                ):
                    finite_number(value, "test value")
                with self.assertRaisesRegex(
                    MotionInputError,
                    "host numeric range",
                ):
                    controller_number(value, "test value")

    def test_rejects_nonfinite_decimal_values_as_nonfinite(self):
        values = (
            Decimal("NaN"),
            Decimal("sNaN"),
            Decimal("Infinity"),
            "NaN",
            "sNaN",
            "-Infinity",
        )
        for value in values:
            with self.subTest(value=str(value)):
                with self.assertRaisesRegex(MotionInputError, "must be finite"):
                    finite_number(value, "test value")

    def test_controller_decimal_format_cannot_collide_with_field_markers(self):
        values = ("1e-5", "-2.5e10", 3.4028234663852886e38)
        for value in values:
            with self.subTest(value=value):
                encoded = controller_protocol_decimal(value, "test value")
                self.assertNotRegex(encoded, r"[eE+]")
                expected = struct.unpack(">f", struct.pack(">f", float(value)))[0]
                self.assertEqual(float(encoded), expected)

    def test_uses_baseline_for_commands_without_speed(self):
        self.assertEqual(
            command_response_timeout("RP\n", 120, 0, 0),
            120.0,
        )

        with self.assertRaisesRegex(
            MotionInputError,
            "cancellation-bound response ownership",
        ):
            command_response_timeout("PGFnSampleGcode.txt\n", 120, 0, 0)

    def test_scales_seconds_percent_and_millimeter_modes(self):
        cases = (
            ("Ss200Ac10Dc20Rm10", 2510.0),
            ("Sp1Ac10Dc20Rm10", 250010.0),
            ("Sm2Ac10Dc20Rm10", 2510.0),
            ("Sp100Ac10Dc20Rm100", 1260.0),
        )
        for timing, expected in cases:
            with self.subTest(timing=timing):
                self.assertEqual(
                    command_response_timeout(
                        self.joint_command(timing),
                        120,
                        100,
                        1000,
                    ),
                    expected,
                )

        self.assertEqual(
            motion_timing_response_timeout(
                CommandTiming("p", 1.0, 10.0, 20.0, 10.0),
                120,
                100,
                1000,
            ),
            250010.0,
        )
        with self.assertRaises(MotionInputError):
            motion_timing_response_timeout("invalid", 120, 100, 1000)


class PrimaryJointTargetSubmissionTests(unittest.TestCase):
    def test_normalizes_complete_target_and_submits_once(self):
        submissions = []

        def submit(target):
            submissions.append(target)
            return "submitted"

        result = submit_primary_joint_target(
            ("1", "-2.5", 3, 4.25, "5.0", 6),
            submit,
        )

        self.assertEqual(result, "submitted")
        self.assertEqual(
            submissions,
            [(1.0, -2.5, 3.0, 4.25, 5.0, 6.0)],
        )

    def test_rejects_incomplete_target_before_submission(self):
        submissions = []

        with self.assertRaisesRegex(MotionInputError, "must contain 6 values"):
            submit_primary_joint_target(
                (1, 2, 3, 4, 5),
                submissions.append,
            )

        self.assertEqual(submissions, [])

    def test_rejects_invalid_target_before_submission(self):
        submissions = []

        with self.assertRaisesRegex(MotionInputError, "must be finite"):
            submit_primary_joint_target(
                (1, 2, 3, 4, math.nan, 6),
                submissions.append,
            )

        self.assertEqual(submissions, [])

    def test_pasted_target_accepts_supported_formats(self):
        expected = (1.0, -2.5, 3.0, 4.25, 5.0, 6.0)

        for text in ("(1, -2.5, 3, 4.25, 5.0, 6)", "[1 -2.5 3 4.25 5.0 6]"):
            with self.subTest(text=text):
                submissions = []
                result = submit_primary_joint_target_text(
                    text,
                    lambda target: submissions.append(target) or "submitted",
                )

                self.assertEqual(result, "submitted")
                self.assertEqual(submissions, [expected])

    def test_pasted_target_rejects_malformed_text_before_submission(self):
        submissions = []

        for text in ("(1, 2, 3, 4, 5, 6]", "1, 2, 3, , 5, 6", "1 " * 257):
            with self.subTest(text=text):
                with self.assertRaises(MotionInputError):
                    submit_primary_joint_target_text(text, submissions.append)

        self.assertEqual(submissions, [])


class NamedJointPositionTests(unittest.TestCase):
    def test_start_position_matches_post_calibration_pose(self):
        self.assertEqual(
            PRIMARY_START_POSITION,
            (0.0, 0.0, 0.0, 0.0, 45.0, 0.0),
        )

    def test_shutdown_position_uses_controller_parking_references(self):
        reference = PrimaryHomeReference(
            (False, True, True),
            (0.0, -38.2, 52.0),
        )
        target = primary_shutdown_position(reference)

        self.assertEqual(target[0], PRIMARY_START_POSITION[0])
        self.assertAlmostEqual(target[1], -38.2)
        self.assertAlmostEqual(target[2], 52.0)
        self.assertEqual(target[3:], PRIMARY_START_POSITION[3:])

    def test_shutdown_position_requires_both_active_home_references(self):
        with self.assertRaisesRegex(
            MotionInputError,
            "requires homing J2",
        ):
            primary_shutdown_position(
                PrimaryHomeReference(
                    (True, False, True),
                    (163.8, 0.0, 52.0),
                )
            )
        with self.assertRaisesRegex(
            MotionInputError,
            "requires homing J3",
        ):
            primary_shutdown_position(
                PrimaryHomeReference(
                    (True, True, False),
                    (163.8, -38.2, 0.0),
                )
            )
        with self.assertRaisesRegex(
            MotionInputError,
            "requires a controller home reference",
        ):
            primary_shutdown_position(None)

class DeferredJointAdjustmentsTests(unittest.TestCase):
    def setUp(self):
        self.profile = MotionProfile("Sp", 50, 10, 10, 25, "N", "000000")

    @staticmethod
    def consume(deferred, actual, generation, allow_current_generation=False):
        return deferred.consume(
            actual,
            generation,
            lambda target, profile: (target, profile),
            allow_current_generation=allow_current_generation,
        )

    def test_accumulates_axes_until_newer_position_exists(self):
        deferred = DeferredJointAdjustments()

        self.assertTrue(deferred.add(0, 1, self.profile, 4))
        self.assertTrue(deferred.add(1, 2, self.profile, 4))
        self.assertTrue(deferred.add(0, 3, self.profile, 4))
        self.assertFalse(deferred.ready(4))
        self.assertTrue(deferred.ready(5))

        target, profile = self.consume(
            deferred,
            (10, 10, 0, 0, 0, 0, 0, 0, 0),
            5,
        )
        self.assertEqual(target, (14.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertIs(profile, self.profile)

    def test_same_generation_requires_confirmed_transport_release(self):
        deferred = DeferredJointAdjustments()
        deferred.add(0, 3, self.profile, 4)

        self.assertFalse(deferred.ready(4))
        self.assertTrue(
            deferred.ready(4, allow_current_generation=True)
        )
        target, profile = self.consume(
            deferred,
            (10, 0, 0, 0, 0, 0, 0, 0, 0),
            4,
            allow_current_generation=True,
        )

        self.assertEqual(target[0], 13.0)
        self.assertIs(profile, self.profile)
        with self.assertRaises(MotionInputError):
            deferred.ready(4, allow_current_generation=1)

    def test_absolute_target_replaces_prior_delta_and_accepts_later_delta(self):
        deferred = DeferredJointAdjustments()

        deferred.add(0, 3, self.profile, 4)
        deferred.set_target(0, 20, self.profile, 4)
        deferred.add(0, -2, self.profile, 4)

        target, profile = self.consume(
            deferred,
            (100, 0, 0, 0, 0, 0, 0, 0, 0),
            5,
        )
        self.assertEqual(target[0], 18.0)
        self.assertIs(profile, self.profile)

    def test_latest_absolute_target_wins_for_each_axis(self):
        deferred = DeferredJointAdjustments()

        deferred.set_target(0, 10, self.profile, 2)
        deferred.set_target(1, 15, self.profile, 2)
        deferred.set_target(0, 20, self.profile, 2)

        target, _ = self.consume(
            deferred,
            (1, 2, 3, 4, 5, 6, 7, 8, 9),
            3,
        )
        self.assertEqual(target, (20.0, 15.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0))

    def test_multi_axis_target_is_atomic_and_preserves_other_intent(self):
        deferred = DeferredJointAdjustments()
        deferred.add(6, 2, self.profile, 2)

        self.assertTrue(
            deferred.set_targets(
                (0, 0, 0, 0, 45, 0, None, None, None),
                self.profile,
                2,
            )
        )

        target, profile = self.consume(
            deferred,
            (10, 20, 30, 40, 50, 60, 7, 8, 9),
            3,
        )
        self.assertEqual(
            target,
            (0.0, 0.0, 0.0, 0.0, 45.0, 0.0, 9.0, 8.0, 9.0),
        )
        self.assertIs(profile, self.profile)

    def test_multi_axis_target_rejects_empty_or_wrong_length_input(self):
        deferred = DeferredJointAdjustments()

        with self.assertRaisesRegex(MotionInputError, "at least one target"):
            deferred.set_targets(
                (None,) * 9,
                self.profile,
                2,
            )
        with self.assertRaisesRegex(MotionInputError, "contain 9 values"):
            deferred.set_targets(
                (0,) * 8,
                self.profile,
                2,
            )

    def test_concurrent_producers_preserve_every_accepted_axis(self):
        deferred = DeferredJointAdjustments()
        barrier = threading.Barrier(3)
        results = []

        def produce(axis, delta):
            barrier.wait()
            results.append(deferred.add(axis, delta, self.profile, 4))

        producers = [
            threading.Thread(target=produce, args=(0, 1)),
            threading.Thread(target=produce, args=(1, 2)),
        ]
        for producer in producers:
            producer.start()
        barrier.wait()
        for producer in producers:
            producer.join(1)
            self.assertFalse(producer.is_alive())

        target, _ = self.consume(
            deferred,
            (10, 10, 0, 0, 0, 0, 0, 0, 0),
            5,
        )
        self.assertEqual(results, [True, True])
        self.assertEqual(target[:2], (11.0, 12.0))

    def test_input_accepted_after_consume_snapshot_remains_pending(self):
        deferred = DeferredJointAdjustments()
        deferred.add(0, 1, self.profile, 4)
        consumer_entered = threading.Event()
        release_consumer = threading.Event()
        producer_attempted = threading.Event()
        producer_finished = threading.Event()
        consumed = []
        producer_results = []
        failures = []

        def consumer(target, profile):
            consumed.append((target, profile))
            consumer_entered.set()
            if not release_consumer.wait(2):
                raise TimeoutError("test did not release deferred consumer")
            return target

        def consume_first():
            try:
                deferred.consume(
                    (10, 10, 0, 0, 0, 0, 0, 0, 0),
                    5,
                    consumer,
                )
            except Exception as exc:
                failures.append(exc)

        def produce_later():
            producer_attempted.set()
            try:
                producer_results.append(
                    deferred.add(1, 2, self.profile, 5)
                )
            except Exception as exc:
                failures.append(exc)
            finally:
                producer_finished.set()

        consuming_thread = threading.Thread(target=consume_first)
        consuming_thread.start()
        self.assertTrue(consumer_entered.wait(1))
        producer_thread = threading.Thread(target=produce_later)
        producer_thread.start()
        self.assertTrue(producer_attempted.wait(1))
        time.sleep(0.02)
        self.assertFalse(producer_finished.is_set())

        release_consumer.set()
        consuming_thread.join(1)
        producer_thread.join(1)
        self.assertFalse(consuming_thread.is_alive())
        self.assertFalse(producer_thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(consumed[0][0][:2], (11.0, 10.0))
        self.assertEqual(producer_results, [True])
        self.assertTrue(deferred.pending)

        target, _ = self.consume(
            deferred,
            (11, 10, 0, 0, 0, 0, 0, 0, 0),
            6,
        )
        self.assertEqual(target[:2], (11.0, 12.0))

    def test_canceling_adjustments_clears_deferred_target(self):
        deferred = DeferredJointAdjustments()

        deferred.add(0, 1, self.profile, 2)
        self.assertFalse(deferred.add(0, -1, self.profile, 2))
        self.assertFalse(deferred.pending)


class JointMotionProtocolTests(unittest.TestCase):
    def setUp(self):
        self.profile = MotionProfile(
            speed_prefix="Sp",
            speed=50,
            acceleration=10,
            deceleration=10,
            ramp=25,
            wrist_config="N",
            loop_mode="000000",
        )

    def test_builds_complete_nine_axis_command(self):
        command = build_robot_joint_command(
            range(1, 10),
            self.profile,
            controller_calibration(),
        )

        self.assertEqual(
            command,
            "RJA1B2C3D4E5F6J77J88J99Sp50Ac10Dc10Rm25WNLm000000\n",
        )

    def test_motion_fields_use_delimiter_safe_controller_decimals(self):
        profile = MotionProfile(
            speed_prefix="Ss",
            speed="1e-5",
            acceleration=10,
            deceleration=10,
            ramp=25,
            wrist_config="N",
            loop_mode="000000",
        )
        command = build_robot_joint_command(
            ("1e-5", 0, 0, 0, 0, 0, 0, 0, 0),
            profile,
            controller_calibration(),
        )
        encoded = controller_protocol_decimal("1e-5", "expected value")

        self.assertIn(f"A{encoded}B0", command)
        self.assertIn(f"Ss{encoded}Ac10", command)

    def test_rejects_non_finite_command_values(self):
        with self.assertRaises(MotionInputError):
            build_robot_joint_command(
                (0, 0, 0, 0, 0, float("nan"), 0, 0, 0),
                self.profile,
                controller_calibration(),
            )

    def test_rejects_positions_outside_calibrated_limits(self):
        calibration = controller_calibration(
            negative_limits=(10,) * 9,
            positive_limits=(20,) * 9,
        )

        with self.assertRaisesRegex(MotionInputError, "outside the calibrated limits"):
            build_robot_joint_command(
                (21, 0, 0, 0, 0, 0, 0, 0, 0),
                self.profile,
                calibration,
            )

    def test_controller_reports_use_the_step_quantized_endpoint(self):
        calibration = controller_calibration(
            negative_limits=(100, 100, 89) + (100,) * 6,
            positive_limits=(100, 100, 52) + (100,) * 6,
            steps_per_unit=(100, 100, 111.111) + (100,) * 6,
        )
        reported = (0, 0, 52.002, 0, 0, 0, 0, 0, 0)

        with self.assertRaisesRegex(
            MotionInputError,
            "outside the calibrated limits",
        ):
            calibration.validate_positions(reported)
        self.assertEqual(
            calibration.validate_reported_positions(reported),
            reported,
        )
        self.assertEqual(
            calibration.fixed_point_command_positions_from_current(reported),
            (0.0, 0.0, 52.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        with self.assertRaisesRegex(
            MotionInputError,
            "outside the calibrated limits",
        ):
            calibration.validate_reported_positions(
                (0, 0, 52.003, 0, 0, 0, 0, 0, 0)
            )

        decimal_boundary = controller_calibration(
            negative_limits=(89,) * 9,
            positive_limits=(52,) * 9,
            steps_per_unit=(100.01,) * 9,
        )
        self.assertEqual(
            decimal_boundary.validate_reported_positions((52.005,) * 9),
            (52.005,) * 9,
        )
        negative_boundary = controller_calibration(
            negative_limits=(89.0008,) * 9,
            positive_limits=(52,) * 9,
            steps_per_unit=(10000,) * 9,
        )
        self.assertEqual(
            negative_boundary.command_positions_from_report((-89.001,) * 9),
            (-89.0008,) * 9,
        )
        mixed_current = (1.2345, -89.001) + (0,) * 7
        self.assertEqual(
            negative_boundary.command_positions_from_current(mixed_current)[:2],
            (1.2345, -89.0008),
        )
        self.assertEqual(
            negative_boundary.fixed_point_command_positions_from_current(
                (-89.001,) * 9
            ),
            (-89.0,) * 9,
        )
        wide_report = controller_calibration(
            negative_limits=(0,) * 9,
            positive_limits=(3_000_000,) * 9,
            steps_per_unit=(1,) * 9,
        )
        self.assertEqual(
            wide_report.validate_reported_positions((0,) * 9),
            (0.0,) * 9,
        )

    def test_rejects_calibration_that_overflows_controller_step_counters(self):
        with self.assertRaisesRegex(MotionInputError, "controller step range"):
            controller_calibration(
                negative_limits=(0,) * 9,
                positive_limits=(30_000_000,) + (100,) * 8,
                steps_per_unit=(100,) * 9,
            )

    def test_accepts_largest_binary32_step_value_below_signed_limit(self):
        calibration = controller_calibration(
            negative_limits=(0,) * 9,
            positive_limits=(2_147_483_520,) + (100,) * 8,
            steps_per_unit=(1,) * 9,
        )

        command = build_robot_joint_command(
            (2_147_483_520, 0, 0, 0, 0, 0, 0, 0, 0),
            self.profile,
            calibration,
        )

        self.assertTrue(command.startswith("RJA2147483520B0"))

    def test_parses_firmware_position_contract(self):
        raw = position_response(
            (1, -2.5, 3, 4, 5, 6),
            external=(7, 8, 9),
            speed_violation=1,
            debug="42.5",
            flag="EC000000",
        )

        parsed = parse_position_response(raw)

        self.assertEqual(parsed.joints, (1.0, -2.5, 3.0, 4.0, 5.0, 6.0))
        self.assertEqual(parsed.external, (7.0, 8.0, 9.0))
        self.assertTrue(parsed.speed_violation)
        self.assertEqual(parsed.debug, "42.5")
        self.assertEqual(parsed.flag, "EC000000")

    def test_rejects_non_firmware_debug_and_fault_payloads(self):
        raw = position_response((1, 2, 3, 4, 5, 6))
        malformed = (
            raw.replace("NO", "NdebugO"),
            raw.replace("NO", "NOfault"),
            raw.replace("NO", "NOEC01010"),
            raw.replace("NO", "NOEBP0"),
            raw.replace("A1B", "A1P0B"),
        )

        for response in malformed:
            with self.subTest(response=response):
                with self.assertRaises(ProtocolResponseError):
                    parse_position_response(response)

    def test_rejects_missing_or_reordered_response_markers(self):
        with self.assertRaises(ProtocolResponseError):
            parse_position_response("A1C2B3")

    def test_rejects_non_finite_response_values(self):
        raw = position_response((1, 2, 3, 4, 5, 6)).replace("A1B", "A1e999B")

        with self.assertRaises(ProtocolResponseError):
            parse_position_response(raw)

    def test_rejects_position_response_payload_padding(self):
        raw = position_response((1, 2, 3, 4, 5, 6))

        for padded in (f" {raw}", f"{raw} ", f"\t{raw}"):
            with self.subTest(padded=padded):
                with self.assertRaises(ProtocolResponseError):
                    parse_position_response(padded)

    def test_rejects_non_text_profile_fields(self):
        with self.assertRaises(MotionInputError):
            MotionProfile("Sp", 50, 10, 10, 25, "N", None)


class CommandedJointTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.calibration = controller_calibration()
        self.start = (0,) * 9

    def move(self, target, profile):
        return JointMove(target, profile, self.calibration)

    def test_percent_profile_models_synchronized_firmware_envelope(self):
        profile = MotionProfile(
            "Sp",
            50,
            20,
            20,
            20,
            "N",
            "000000",
        )
        trajectory = estimate_commanded_joint_trajectory(
            self.start,
            self.move((10, -5, 0, 0, 0, 0, 0, 0, 0), profile),
            200,
        )

        self.assertIsInstance(trajectory, CommandedJointTrajectory)
        self.assertEqual(
            trajectory.step_deltas,
            (1000, 500, 0, 0, 0, 0, 0, 0, 0),
        )
        self.assertEqual(trajectory.high_steps, 1000)
        self.assertAlmostEqual(
            trajectory.duration_seconds,
            discrete_firmware_joint_duration(
                trajectory.step_deltas,
                profile,
            ),
        )
        self.assertEqual(trajectory.positions_at(0), (0.0,) * 9)
        self.assertEqual(
            trajectory.positions_at(trajectory.duration_seconds / 2)[:2],
            (5.0, -2.5),
        )
        self.assertEqual(
            trajectory.positions_at(trajectory.duration_seconds + 1),
            (10.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_seconds_profile_uses_requested_duration_when_not_clamped(self):
        profile = MotionProfile(
            "Ss",
            2,
            20,
            20,
            20,
            "N",
            "000000",
        )
        trajectory = estimate_commanded_joint_trajectory(
            self.start,
            self.move((10, 0, 0, 0, 0, 0, 0, 0, 0), profile),
            200,
        )

        self.assertAlmostEqual(trajectory.duration_seconds, 2.0, places=6)
        self.assertAlmostEqual(trajectory.positions_at(1.0)[0], 5.0, places=5)

    def test_fast_profile_includes_synchronized_pulse_distribution_cost(self):
        profile = MotionProfile(
            "Sp",
            100,
            20,
            20,
            10,
            "N",
            "000000",
        )
        single_axis = estimate_commanded_joint_trajectory(
            self.start,
            self.move((10, 0, 0, 0, 0, 0, 0, 0, 0), profile),
            200,
        )
        all_axes = estimate_commanded_joint_trajectory(
            self.start,
            self.move((10,) * 9, profile),
            200,
        )

        self.assertEqual(
            single_axis.average_distribution_delay_microseconds,
            30,
        )
        self.assertEqual(
            all_axes.average_distribution_delay_microseconds,
            270,
        )
        self.assertAlmostEqual(
            single_axis.duration_seconds,
            discrete_firmware_joint_duration(
                single_axis.step_deltas,
                profile,
            ),
        )
        self.assertAlmostEqual(
            all_axes.duration_seconds,
            discrete_firmware_joint_duration(
                all_axes.step_deltas,
                profile,
            ),
        )

    def test_average_distribution_model_tracks_discrete_firmware_loop(self):
        profile = MotionProfile(
            "Ss",
            0.25,
            20,
            20,
            20,
            "N",
            "000000",
        )
        trajectory = estimate_commanded_joint_trajectory(
            self.start,
            self.move(
                (10, 3.33, 1.11, 0.37, 0.12, 0.04, 0.01, 0, 0),
                profile,
            ),
            200,
        )
        firmware_duration = discrete_firmware_joint_duration(
            trajectory.step_deltas,
            profile,
        )
        relative_error = abs(
            trajectory.duration_seconds - firmware_duration
        ) / firmware_duration

        self.assertLessEqual(relative_error, 0.03)

    def test_zero_step_delta_holds_confirmed_start_until_terminal_feedback(self):
        profile = MotionProfile(
            "Sp",
            50,
            20,
            20,
            20,
            "N",
            "000000",
        )
        trajectory = estimate_commanded_joint_trajectory(
            self.start,
            self.move((0.009, 0, 0, 0, 0, 0, 0, 0, 0), profile),
            200,
        )

        self.assertEqual(trajectory.high_steps, 0)
        self.assertEqual(trajectory.duration_seconds, 0)
        self.assertEqual(trajectory.positions_at(60), (0.0,) * 9)

    def test_substep_axis_remains_at_confirmed_position_during_other_motion(self):
        profile = MotionProfile(
            "Sp",
            50,
            20,
            20,
            20,
            "N",
            "000000",
        )
        trajectory = estimate_commanded_joint_trajectory(
            self.start,
            self.move((10, 0.009, 0, 0, 0, 0, 0, 0, 0), profile),
            200,
        )

        self.assertEqual(trajectory.target_positions[1], 0.009)
        self.assertEqual(trajectory.step_deltas[1], 0)
        self.assertEqual(trajectory.positions_at(trajectory.duration_seconds)[1], 0)

    def test_terminal_estimate_uses_controller_integer_zero_step(self):
        calibration = controller_calibration(
            negative_limits=(170,) * 9,
            positive_limits=(170,) * 9,
            steps_per_unit=(6400 / 360,) * 9,
        )
        profile = MotionProfile(
            "Sp",
            50,
            20,
            20,
            20,
            "N",
            "000000",
        )
        target = 10.2
        trajectory = estimate_commanded_joint_trajectory(
            self.start,
            JointMove(
                (target, 0, 0, 0, 0, 0, 0, 0, 0),
                profile,
                calibration,
            ),
            200,
        )
        float32 = lambda value: struct.unpack(
            ">f",
            struct.pack(">f", value),
        )[0]
        negative = float32(170)
        scale = float32(6400 / 360)
        zero_step = int(float32(negative * scale))
        target_step = int(
            float32(float32(float32(target) + negative) * scale)
        )
        expected = float32(float32(target_step - zero_step) / scale)

        self.assertEqual(trajectory.estimated_terminal_positions[0], expected)

    def test_estimator_rejects_invalid_timing_boundaries(self):
        profile = MotionProfile(
            "Sp",
            50,
            20,
            20,
            20,
            "N",
            "000000",
        )
        move = self.move((10, 0, 0, 0, 0, 0, 0, 0, 0), profile)

        for invalid_delay in (0, -1, float("nan"), True):
            with self.subTest(invalid_delay=invalid_delay):
                with self.assertRaises(MotionInputError):
                    estimate_commanded_joint_trajectory(
                        self.start,
                        move,
                        invalid_delay,
                    )

        trajectory = estimate_commanded_joint_trajectory(
            self.start,
            move,
            200,
        )
        with self.assertRaisesRegex(MotionInputError, "non-negative"):
            trajectory.positions_at(-0.1)


class JointTelemetryMeasurementTests(unittest.TestCase):
    @staticmethod
    def measurement(*ticks):
        clock = iter(ticks)
        return ordinary_joint_telemetry_measurement(
            True,
            False,
            lambda: next(clock),
        )

    def test_selection_excludes_disabled_and_trace_requests(self):
        self.assertIsNone(
            ordinary_joint_telemetry_measurement(False, False)
        )
        self.assertIsNone(
            ordinary_joint_telemetry_measurement(False, True)
        )
        with self.assertRaisesRegex(MotionInputError, "trace capture"):
            ordinary_joint_telemetry_measurement(True, True)
        for telemetry_enabled, trace_requested in ((1, False), (True, 0)):
            with self.subTest(
                telemetry_enabled=telemetry_enabled,
                trace_requested=trace_requested,
            ):
                with self.assertRaisesRegex(MotionInputError, "selection"):
                    ordinary_joint_telemetry_measurement(
                        telemetry_enabled,
                        trace_requested,
                    )

    def test_empty_move_reports_admission_and_terminal_without_sample_claims(self):
        measurement = self.measurement(100, 160)
        measurement.admit()
        measurement.observe_terminal()

        summary = measurement.finalize(7)

        self.assertEqual(summary["request_id"], 7)
        self.assertEqual(summary["admitted_at_monotonic_ns"], 100)
        self.assertEqual(summary["terminal_at_monotonic_ns"], 160)
        self.assertEqual(summary["frame_count"], 0)
        self.assertEqual(summary["canonical_json_lf_bytes"], 0)
        self.assertIsNone(summary["receipt_interval_ns"])
        self.assertIsNone(summary["final_telemetry_to_terminal_ns"])
        self.assertEqual(summary["dispatcher_accepted_frames"], 0)
        self.assertEqual(summary["dispatcher_rejected_frames"], 0)

    def test_single_sample_preserves_baseline_and_dispatcher_disposition(self):
        measurement = self.measurement(100, 120, 170)
        measurement.admit()
        measurement.observe(4, None, "{}", False)
        measurement.observe_terminal()

        summary = measurement.finalize(8)

        self.assertEqual(summary["first_sequence"], 4)
        self.assertEqual(summary["last_sequence"], 4)
        self.assertTrue(summary["first_sequence_without_prior_baseline"])
        self.assertEqual(summary["canonical_json_lf_bytes"], 3)
        self.assertEqual(summary["first_receipt_at_monotonic_ns"], 120)
        self.assertEqual(summary["last_receipt_at_monotonic_ns"], 120)
        self.assertIsNone(summary["receipt_window_ns"])
        self.assertEqual(summary["payload_window_ns"], 20)
        self.assertEqual(
            summary["canonical_json_lf_bytes_per_second"],
            150_000_000,
        )
        self.assertEqual(summary["final_telemetry_to_terminal_ns"], 50)
        self.assertEqual(summary["dispatcher_accepted_frames"], 0)
        self.assertEqual(summary["dispatcher_rejected_frames"], 1)

    def test_contiguous_and_gap_samples_report_bounded_distribution(self):
        measurement = self.measurement(100, 120, 150, 210, 250)
        measurement.admit()
        measurement.observe(10, True, "{}", False)
        measurement.observe(11, True, "{}", True)
        measurement.observe(13, False, "{}", True)
        measurement.observe_terminal()

        summary = measurement.finalize(9)

        self.assertEqual(summary["frame_count"], 3)
        self.assertEqual(summary["first_sequence"], 10)
        self.assertEqual(summary["last_sequence"], 13)
        self.assertFalse(summary["first_sequence_without_prior_baseline"])
        self.assertEqual(summary["sequence_gap_events"], 1)
        self.assertEqual(summary["receipt_window_ns"], 90)
        self.assertEqual(summary["payload_window_ns"], 110)
        self.assertEqual(
            summary["receipt_interval_ns"],
            {"count": 2, "minimum": 30, "mean": 45, "maximum": 60},
        )
        self.assertEqual(summary["canonical_json_lf_bytes"], 9)
        self.assertEqual(
            summary["canonical_json_lf_bytes_per_second"],
            9 * 1_000_000_000 / 110,
        )
        self.assertEqual(summary["final_telemetry_to_terminal_ns"], 40)
        self.assertEqual(summary["dispatcher_accepted_frames"], 2)
        self.assertEqual(summary["dispatcher_rejected_frames"], 1)

    def test_unsettled_or_out_of_order_measurement_cannot_finalize(self):
        measurement = self.measurement(100, 120)
        with self.assertRaisesRegex(MotionInputError, "active request"):
            measurement.observe(1, None, "{}", True)
        measurement.admit()
        measurement.observe(1, None, "{}", True)
        with self.assertRaisesRegex(MotionInputError, "terminal settlement"):
            measurement.finalize(10)

        backwards = self.measurement(100, 90)
        backwards.admit()
        with self.assertRaisesRegex(MotionInputError, "backwards"):
            backwards.observe(1, None, "{}", True)

    def test_explicit_times_validate_the_production_boundary(self):
        measurement = self.measurement()
        measurement.admit(100)
        measurement.observe(1, None, "{}", True, 120)
        measurement.observe_terminal(150)

        summary = measurement.finalize(11)

        self.assertEqual(summary["admitted_at_monotonic_ns"], 100)
        self.assertEqual(summary["last_receipt_at_monotonic_ns"], 120)
        self.assertEqual(summary["terminal_at_monotonic_ns"], 150)
        for invalid_time in (True, -1, 1.5):
            invalid = self.measurement()
            with self.subTest(invalid_time=invalid_time):
                with self.assertRaisesRegex(MotionInputError, "supplied time"):
                    invalid.admit(invalid_time)


class CoalescingJointDispatcherTests(unittest.TestCase):
    def setUp(self):
        self.profile = MotionProfile("Sp", 50, 10, 10, 25, "N", "000000")
        self.actual = (0,) * 9
        self.calibration = controller_calibration()

    def make_dispatcher(self, exchange, **kwargs):
        def typed_exchange(command):
            result = exchange(command)
            if isinstance(result, str):
                position = parse_position_response(result)
                return JointMotionExchangeResult(result, position)
            return result

        return CoalescingJointDispatcher(
            typed_exchange,
            lambda: self.calibration,
            **kwargs,
        )

    def test_canonical_position_telemetry_preserves_validated_wire_evidence(self):
        dispatcher = None
        raw = '{"data":{"robot_joints_millidegrees":[1,2,3,4,5,6]},"seq":1,"stream":"joint_position","type":"telemetry","v":1}'
        positions = (0.001, 0.002, 0.003, 0.004, 0.005, 0.006)

        def exchange(_command):
            self.assertTrue(
                dispatcher.publish_position_telemetry(raw, positions)
            )
            return position_response((1, 2, 3, 4, 5, 6))

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_positions(
            (1, 2, 3, 4, 5, 6, 0, 0, 0),
            self.actual,
            self.profile,
        )
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in events],
            ["started", "telemetry", "completed"],
        )
        self.assertEqual(events[1].telemetry.raw, raw)
        self.assertEqual(events[1].telemetry.joints, positions)

    def test_canonical_position_telemetry_rejects_invalid_boundaries(self):
        dispatcher = self.make_dispatcher(
            lambda _command: position_response((0,) * 6)
        )
        for raw, positions in (
            ("", (0,) * 6),
            ("{}\n", (0,) * 6),
            ("{}", (0,) * 5),
            ("{}", (0, 0, 0, 0, 0, math.nan)),
        ):
            with self.subTest(raw=raw, positions=positions):
                with self.assertRaises(MotionInputError):
                    dispatcher.publish_position_telemetry(raw, positions)

    def test_typed_exchange_result_preserves_canonical_position(self):
        canonical_raw = '{"cmd":"move_joints","id":2,"result":{},"status":"completed","type":"response","v":1}'
        legacy = parse_position_response(
            position_response(
                (1, 2, 3, 4, 5, 6),
                external=(7, 8, 9),
            )
        )
        canonical_position = legacy.__class__(
            raw=canonical_raw,
            joint_text=legacy.joint_text,
            joints=legacy.joints,
            cartesian_text=legacy.cartesian_text,
            cartesian=legacy.cartesian,
            external_text=legacy.external_text,
            external=legacy.external,
            speed_violation=legacy.speed_violation,
            debug=legacy.debug,
            flag=legacy.flag,
        )
        result = JointMotionExchangeResult(
            canonical_raw,
            canonical_position,
        )
        dispatcher = self.make_dispatcher(lambda _command: result)

        dispatcher.submit_positions(
            (1, 2, 3, 4, 5, 6, 7, 8, 9),
            self.actual,
            self.profile,
        )
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in events],
            ["started", "completed"],
        )
        self.assertEqual(events[-1].response, canonical_raw)
        self.assertIs(events[-1].position, canonical_position)

    def test_exchange_rejects_raw_position_result(self):
        dispatcher = CoalescingJointDispatcher(
            lambda _command: position_response((1, 2, 3, 4, 5, 6)),
            lambda: self.calibration,
        )
        dispatcher.submit_positions(
            (1, 2, 3, 4, 5, 6, 0, 0, 0),
            self.actual,
            self.profile,
        )
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in events],
            ["started", "failed"],
        )
        self.assertIn("invalid result", events[-1].error)

    def test_typed_exchange_failure_retains_authoritative_position(self):
        canonical_raw = '{"cmd":"move_joints","error":{},"id":2,"status":"failed","type":"response","v":1}'
        legacy = parse_position_response(
            position_response(
                (1, 2, 3, 4, 5, 6),
                external=(7, 8, 9),
            )
        )
        canonical_position = legacy.__class__(
            raw=canonical_raw,
            joint_text=legacy.joint_text,
            joints=legacy.joints,
            cartesian_text=legacy.cartesian_text,
            cartesian=legacy.cartesian,
            external_text=legacy.external_text,
            external=legacy.external,
            speed_violation=False,
            debug="",
            flag="",
        )
        result = JointMotionExchangeResult(
            canonical_raw,
            canonical_position,
            "controller failed JSON joint motion",
        )
        dispatcher = self.make_dispatcher(lambda _command: result)

        dispatcher.submit_positions(
            (1, 2, 3, 4, 5, 6, 7, 8, 9),
            self.actual,
            self.profile,
        )
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in events],
            ["started", "failed"],
        )
        self.assertEqual(events[-1].response, canonical_raw)
        self.assertIs(events[-1].position, canonical_position)
        self.assertIn("failed JSON", events[-1].error)

    def test_positionless_rejection_preserves_confirmed_dispatch_state(self):
        self.calibration = controller_calibration(
            negative_limits=(100, 100, 89) + (100,) * 6,
            positive_limits=(100, 100, 52) + (100,) * 6,
            steps_per_unit=(100, 100, 111.111) + (100,) * 6,
        )
        reported = (0, 0, 52.002, 0, 0, 0, 0, 0, 0)
        canonical_raw = '{"cmd":"move_joints","error":{},"id":2,"status":"rejected","type":"response","v":1}'
        rejection = JointMotionExchangeResult(
            canonical_raw,
            None,
            "controller rejected JSON joint motion: joint_limit_violation",
            confirmed_position_unchanged=True,
        )
        calls = []

        def exchange(_command):
            calls.append(True)
            if len(calls) == 1:
                return rejection
            return position_response((2, 3, 4, 5, 6, 7))

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_positions(
            (1, 2, 52, 4, 5, 6, 0, 0, 0),
            reported,
            self.profile,
        )
        rejected_events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in rejected_events],
            ["started", "failed"],
        )
        self.assertTrue(
            rejected_events[-1].confirmed_position_unchanged
        )
        self.assertIsNone(rejected_events[-1].position)
        self.assertIsNone(dispatcher.fault_reason)
        self.assertEqual(
            dispatcher.desired_target,
            (0.0, 0.0, 52.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

        dispatcher.submit_positions(
            (2, 3, 4, 5, 6, 7, 0, 0, 0),
            self.actual,
            self.profile,
        )
        completed_events = collect_events_until_idle(dispatcher)
        self.assertEqual(
            [event.kind for event in completed_events],
            ["started", "completed"],
        )

        exchange_started = threading.Event()
        release_exchange = threading.Event()

        def closing_exchange(_command):
            exchange_started.set()
            if not release_exchange.wait(2):
                raise TimeoutError("test did not release closing exchange")
            return rejection

        closing_dispatcher = self.make_dispatcher(closing_exchange)
        closing_dispatcher.submit_positions(
            (1, 0, 52, 0, 0, 0, 0, 0, 0),
            reported,
            self.profile,
        )
        self.assertTrue(exchange_started.wait(2))
        self.assertFalse(closing_dispatcher.close())
        release_exchange.set()
        closed_events = collect_events_until_idle(closing_dispatcher)
        self.assertEqual(closed_events[-1].kind, "failed")
        self.assertFalse(closing_dispatcher.active)

    def test_typed_exchange_result_validates_raw_position_ownership(self):
        position = parse_position_response(position_response((0,) * 6))
        with self.assertRaises(MotionInputError):
            JointMotionExchangeResult("{}", position)
        with self.assertRaises(MotionInputError):
            JointMotionExchangeResult(position.raw, position, "bad\nerror")
        with self.assertRaises(MotionInputError):
            JointMotionExchangeResult(position.raw, None, "rejected")
        with self.assertRaises(MotionInputError):
            JointMotionExchangeResult(
                position.raw,
                position,
                "rejected",
                confirmed_position_unchanged=True,
            )

    def test_exchange_snapshot_tracks_confirmed_start_across_queued_moves(self):
        dispatcher = None
        first_started = threading.Event()
        release_first = threading.Event()
        commands = []
        snapshots = []

        def exchange(command):
            commands.append(command)
            snapshots.append(dispatcher.current_exchange_snapshot(command))
            if len(commands) == 1:
                first_started.set()
                if not release_first.wait(2):
                    raise TimeoutError("test did not release first exchange")
                return position_response(
                    (1, 2, 3, 4, 5, 6),
                    external=(7, 8, 9),
                )
            return position_response(
                (2, 3, 4, 5, 6, 7),
                external=(7, 8, 9),
            )

        dispatcher = self.make_dispatcher(exchange)
        actual = (0, 0, 0, 0, 0, 0, 7, 8, 9)
        first_target = (1, 2, 3, 4, 5, 6, 7, 8, 9)
        second_target = (2, 3, 4, 5, 6, 7, 7, 8, 9)
        dispatcher.submit_positions(first_target, actual, self.profile)
        self.assertTrue(first_started.wait(2))
        active_snapshot = dispatcher.current_exchange_snapshot(commands[0])
        self.assertIsInstance(active_snapshot, JointExchangeSnapshot)
        with self.assertRaisesRegex(MotionQueueFault, "does not match"):
            dispatcher.current_exchange_snapshot("RJ-invalid\n")
        dispatcher.submit_positions(second_target, actual, self.profile)
        release_first.set()
        collect_events_until_idle(dispatcher)

        self.assertEqual(len(snapshots), 2)
        self.assertEqual(snapshots[0].start_positions, actual)
        self.assertEqual(snapshots[0].target_positions, first_target)
        self.assertEqual(snapshots[1].start_positions, first_target)
        self.assertEqual(snapshots[1].target_positions, second_target)
        self.assertEqual(snapshots[0].profile, self.profile)
        self.assertIsNone(dispatcher.current_exchange_snapshot(commands[-1]))
        with self.assertRaisesRegex(MotionInputError, "must be text"):
            dispatcher.current_exchange_snapshot(None)

    def test_exchange_snapshot_prefers_synchronized_idle_position(self):
        dispatcher = None
        observed = []

        def exchange(command):
            observed.append(dispatcher.current_exchange_snapshot(command))
            return position_response(
                (2, 3, 4, 5, 6, 7),
                external=(7, 8, 9),
            )

        dispatcher = self.make_dispatcher(exchange)
        synchronized = (1, 2, 3, 4, 5, 6, 7, 8, 9)
        stale_actual = (0, 0, 0, 0, 0, 0, 7, 8, 9)
        target = (2, 3, 4, 5, 6, 7, 7, 8, 9)
        self.assertTrue(dispatcher.synchronize(synchronized))

        dispatcher.submit_positions(target, stale_actual, self.profile)
        collect_events_until_idle(dispatcher)

        self.assertEqual(len(observed), 1)
        self.assertEqual(observed[0].start_positions, synchronized)
        self.assertEqual(observed[0].target_positions, target)

    def test_telemetry_events_coalesce_without_delaying_completion(self):
        dispatcher = None
        final_telemetry = None
        publication_finished = threading.Event()

        def exchange(_command):
            nonlocal final_telemetry
            for sample in range(10000):
                raw = f'{{"seq":{sample},"type":"telemetry"}}'
                joints = (sample / 1000.0, 0.002, 0.003, 0.004, 0.005, 0.006)
                final_telemetry = JointTelemetry(
                    raw=raw,
                    joints=joints,
                )
                self.assertTrue(
                    dispatcher.publish_position_telemetry(raw, joints)
                )
            publication_finished.set()
            return position_response((1, 2, 3, 4, 5, 6))

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_positions(
            (1, 2, 3, 4, 5, 6, 0, 0, 0),
            self.actual,
            self.profile,
        )
        self.assertTrue(publication_finished.wait(2))
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in events],
            ["started", "telemetry", "completed"],
        )
        self.assertEqual(events[1].telemetry, final_telemetry)

    def test_telemetry_publication_requires_an_active_validated_exchange(self):
        dispatcher = self.make_dispatcher(
            lambda _command: position_response((0, 0, 0, 0, 0, 0))
        )
        raw = '{"seq":0,"type":"telemetry"}'
        positions = (0.0,) * 6

        with self.assertRaisesRegex(MotionQueueFault, "in-flight"):
            dispatcher.publish_position_telemetry(raw, positions)
        with self.assertRaisesRegex(MotionInputError, "wire evidence"):
            dispatcher.publish_position_telemetry("{}\n", positions)
        with self.assertRaisesRegex(MotionInputError, "positions"):
            dispatcher.publish_position_telemetry(raw, positions[:5])

        dispatcher.close()
        self.assertFalse(
            dispatcher.publish_position_telemetry(raw, positions)
        )

    def test_many_adjustments_become_one_latest_pending_target(self):
        first_started = threading.Event()
        release_first = threading.Event()
        second_completed = threading.Event()
        commands = []

        def exchange(command):
            commands.append(command)
            if len(commands) == 1:
                first_started.set()
                if not release_first.wait(2):
                    raise TimeoutError("test did not release first exchange")
                return position_response((1, 0, 0, 0, 0, 0))
            second_completed.set()
            return position_response((4, 2, -4, 0, 0, 0))

        dispatcher = self.make_dispatcher(exchange)
        first = dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertFalse(first.coalesced)
        self.assertTrue(first_started.wait(2))

        second = dispatcher.submit_delta(1, 2, self.actual, self.profile)
        third = dispatcher.submit_delta(0, 3, self.actual, self.profile)
        fourth = dispatcher.submit_delta(2, -4, self.actual, self.profile)

        self.assertTrue(second.coalesced)
        self.assertTrue(third.coalesced)
        self.assertTrue(fourth.coalesced)
        self.assertEqual(fourth.target, (4.0, 2.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        release_first.set()
        events = collect_events_until_idle(dispatcher)
        self.assertTrue(second_completed.is_set())

        self.assertEqual(len(commands), 2)
        self.assertTrue(commands[0].startswith("RJA1B0C0D0E0F0J70J80J90"))
        self.assertTrue(commands[1].startswith("RJA4B2C-4D0E0F0J70J80J90"))
        self.assertEqual(
            [event.kind for event in events],
            ["started", "completed", "started", "completed"],
        )
        started_events = [
            event for event in events if event.kind == "started"
        ]
        self.assertTrue(
            all(
                isinstance(event.started_at_seconds, float)
                and math.isfinite(event.started_at_seconds)
                for event in started_events
            )
        )
        self.assertLessEqual(
            started_events[0].started_at_seconds,
            started_events[1].started_at_seconds,
        )

    def test_repeated_absolute_targets_replace_instead_of_accumulating(self):
        first_started = threading.Event()
        release_first = threading.Event()
        commands = []

        def exchange(command):
            commands.append(command)
            if len(commands) == 1:
                first_started.set()
                if not release_first.wait(2):
                    raise TimeoutError("test did not release first exchange")
                return position_response((10, 0, 0, 0, 0, 0))
            return position_response((20, 5, 0, 0, 0, 0))

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_target(0, 10, self.actual, self.profile)
        self.assertTrue(first_started.wait(2))

        dispatcher.submit_target(0, 15, self.actual, self.profile)
        dispatcher.submit_target(0, 20, self.actual, self.profile)
        final = dispatcher.submit_target(1, 5, self.actual, self.profile)

        self.assertEqual(final.target, (20.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(dispatcher.pending)
        release_first.set()
        collect_events_until_idle(dispatcher)

        self.assertEqual(len(commands), 2)
        self.assertTrue(commands[0].startswith("RJA10B0C0D0E0F0J70J80J90"))
        self.assertTrue(commands[1].startswith("RJA20B5C0D0E0F0J70J80J90"))
        self.assertFalse(dispatcher.pending)

    def test_delta_applies_to_latest_absolute_target(self):
        first_started = threading.Event()
        release_first = threading.Event()

        def exchange(command):
            if not first_started.is_set():
                first_started.set()
                if not release_first.wait(2):
                    raise TimeoutError("test did not release first exchange")
                return position_response((10, 0, 0, 0, 0, 0))
            return position_response((22, 0, 0, 0, 0, 0))

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_target(0, 10, self.actual, self.profile)
        self.assertTrue(first_started.wait(2))
        dispatcher.submit_target(0, 20, self.actual, self.profile)

        final = dispatcher.submit_delta(0, 2, self.actual, self.profile)

        self.assertEqual(final.target[0], 22.0)
        release_first.set()
        collect_events_until_idle(dispatcher)

    def test_submits_multi_axis_adjustment_as_one_command(self):
        commands = []

        def exchange(command):
            commands.append(command)
            return position_response((1, 2, 3, 0, 0, 0))

        dispatcher = self.make_dispatcher(exchange)
        submission = dispatcher.submit_adjustments(
            (1, 2, 3, 0, 0, 0, 0, 0, 0),
            self.actual,
            self.profile,
        )
        collect_events_until_idle(dispatcher)

        self.assertEqual(submission.target, (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith("RJA1B2C3D0E0F0J70J80J90"))

    def test_submits_partial_multi_axis_target_as_one_command(self):
        commands = []

        def exchange(command):
            commands.append(command)
            return position_response(
                (0, 0, 0, 0, 45, 0),
                external=(7, 8, 9),
            )

        dispatcher = self.make_dispatcher(exchange)
        actual = (10, 20, 30, 40, 50, 60, 7, 8, 9)
        submission = dispatcher.submit_targets(
            (0, 0, 0, 0, 45, 0, None, None, None),
            actual,
            self.profile,
        )
        collect_events_until_idle(dispatcher)

        self.assertEqual(
            submission.target,
            (0.0, 0.0, 0.0, 0.0, 45.0, 0.0, 7.0, 8.0, 9.0),
        )
        self.assertEqual(len(commands), 1)
        self.assertTrue(
            commands[0].startswith("RJA0B0C0D0E45F0J77J88J99")
        )

    def test_uses_latest_confirmed_position_before_tk_consumes_event(self):
        commands = []

        def exchange(command):
            commands.append(command)
            if len(commands) == 1:
                return position_response((5, 0, 0, 0, 0, 0))
            return position_response((6, 0, 0, 0, 0, 0))

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_delta(0, 1, self.actual, self.profile)

        first_events = []
        first_completed = None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and first_completed is None:
            new_events = dispatcher.drain_events()
            first_events.extend(new_events)
            first_completed = next(
                (event for event in new_events if event.kind == "completed"),
                None,
            )
            if first_completed is None:
                time.sleep(0.005)
        self.assertIsNotNone(first_completed)

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        for event in first_events:
            event.acknowledge()
        collect_events_until_idle(dispatcher)

        self.assertEqual(len(commands), 2)
        self.assertTrue(commands[1].startswith("RJA6B0C0D0E0F0J70J80J90"))

    def test_success_acknowledgement_can_discard_pending_target_and_rebase(self):
        self.calibration = controller_calibration(
            negative_limits=(100, 100, 89) + (100,) * 6,
            positive_limits=(100, 100, 52) + (100,) * 6,
            steps_per_unit=(100, 100, 111.111) + (100,) * 6,
        )
        first_started = threading.Event()
        release_first = threading.Event()
        commands = []

        def exchange(command):
            commands.append(command)
            if len(commands) == 1:
                first_started.set()
                if not release_first.wait(2):
                    raise TimeoutError("test did not release first exchange")
                return position_response(
                    (1, 0, 52.002, 0, 0, 0),
                    speed_violation=1,
                )
            return position_response((2, 0, 52.002, 0, 0, 0))

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertTrue(first_started.wait(2))
        dispatcher.submit_delta(1, 2, self.actual, self.profile)
        release_first.set()

        completed = None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and completed is None:
            completed = next(
                (
                    event
                    for event in dispatcher.drain_events()
                    if event.kind == "completed"
                ),
                None,
            )
            if completed is None:
                time.sleep(0.005)
        self.assertIsNotNone(completed)
        self.assertTrue(completed.position.speed_violation)

        confirmed = completed.position.joints + completed.position.external
        self.assertTrue(
            dispatcher.discard_pending_after_completion(confirmed)
        )
        self.assertFalse(dispatcher.pending)
        self.assertEqual(
            dispatcher.desired_target,
            (1.0, 0.0, 52.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        completed.acknowledge()

        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and dispatcher.active:
            time.sleep(0.005)
        self.assertFalse(dispatcher.active)
        self.assertEqual(len(commands), 1)

        followup = dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertEqual(followup.target[0], 2.0)
        collect_events_until_idle(dispatcher)
        self.assertEqual(len(commands), 2)
        self.assertTrue(commands[1].startswith("RJA2B0C52D0E0F0J70J80J90"))

        with self.assertRaises(MotionQueueFault):
            dispatcher.discard_pending_after_completion(confirmed)

    def test_controller_error_discards_pending_motion_and_latches_fault(self):
        first_started = threading.Event()
        release_first = threading.Event()
        commands = []

        def exchange(command):
            commands.append(command)
            first_started.set()
            if not release_first.wait(2):
                raise TimeoutError("test did not release exchange")
            raise ProtocolResponseError("controller rejected JSON joint motion")

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertTrue(first_started.wait(2))
        dispatcher.submit_delta(1, 2, self.actual, self.profile)
        release_first.set()
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(len(commands), 1)
        self.assertEqual(events[-1].kind, "failed")
        self.assertTrue(events[-1].pending_discarded)
        self.assertIn(
            "controller rejected JSON joint motion",
            dispatcher.fault_reason,
        )
        with self.assertRaises(MotionQueueFault):
            dispatcher.submit_delta(2, 1, self.actual, self.profile)

        self.assertTrue(dispatcher.synchronize(self.actual))
        self.assertIsNone(dispatcher.fault_reason)

    def test_malformed_success_response_latches_fault(self):
        dispatcher = self.make_dispatcher(lambda command: "not-a-position")

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(events[-1].kind, "failed")
        self.assertIn("invalid markers or values", events[-1].error)

    def test_out_of_range_success_and_fault_responses_never_mutate_dispatcher_state(self):
        for flag in ("", "EA", "EB"):
            with self.subTest(flag=flag):
                dispatcher = self.make_dispatcher(
                    lambda command, response_flag=flag: position_response(
                        (101, 0, 0, 0, 0, 0),
                        flag=response_flag,
                    )
                )

                dispatcher.submit_delta(0, 1, self.actual, self.profile)
                events = collect_events_until_idle(dispatcher)

                self.assertEqual(events[-1].kind, "failed")
                self.assertIsNone(events[-1].position)
                self.assertIn("outside the calibrated limits", events[-1].error)
                self.assertIsNone(dispatcher.desired_target)
                self.assertIn(
                    "outside the calibrated limits",
                    dispatcher.fault_reason,
                )

    def test_step_quantized_endpoint_response_completes_motion(self):
        self.calibration = controller_calibration(
            negative_limits=(100, 100, 89) + (100,) * 6,
            positive_limits=(100, 100, 52) + (100,) * 6,
            steps_per_unit=(100, 100, 111.111) + (100,) * 6,
        )
        responses = iter((
            position_response((0, 0, 52.002, 0, 0, 0)),
            position_response((1, 0, 52.002, 0, 0, 0)),
        ))
        dispatcher = self.make_dispatcher(lambda _command: next(responses))
        reported = (0, 0, 52.002, 0, 0, 0, 0, 0, 0)

        self.assertTrue(dispatcher.synchronize(reported))
        self.assertEqual(
            dispatcher.desired_target,
            (0.0, 0.0, 52.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        move = JointMove(
            (1, 0, 52, 0, 0, 0, 0, 0, 0),
            self.profile,
            self.calibration,
        )
        trajectory = estimate_commanded_joint_trajectory(reported, move, 30)
        self.assertEqual(trajectory.start_positions, reported)

        dispatcher.submit_positions(
            (0, 0, 52, 0, 0, 0, 0, 0, 0),
            self.actual,
            self.profile,
        )
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(events[-1].kind, "completed")
        self.assertEqual(events[-1].position.joints[2], 52.002)
        self.assertIsNone(dispatcher.fault_reason)

        submission = dispatcher.submit_delta(0, 1, reported, self.profile)
        self.assertEqual(submission.target[2], 52.0)
        followup_events = collect_events_until_idle(dispatcher)
        self.assertEqual(followup_events[-1].kind, "completed")

    def test_position_response_fault_flag_stops_pending_motion(self):
        first_started = threading.Event()
        release_first = threading.Event()
        commands = []

        def exchange(command):
            commands.append(command)
            first_started.set()
            if not release_first.wait(2):
                raise TimeoutError("test did not release exchange")
            return position_response((1, 0, 0, 0, 0, 0), flag="EC100000")

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertTrue(first_started.wait(2))
        dispatcher.submit_delta(1, 2, self.actual, self.profile)
        release_first.set()
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(len(commands), 1)
        self.assertEqual(events[-1].kind, "failed")
        self.assertEqual(events[-1].position.flag, "EC100000")
        self.assertTrue(events[-1].pending_discarded)
        self.assertIn("controller reported motion fault", dispatcher.fault_reason)

    def test_external_invalidation_requires_fresh_position(self):
        dispatcher = self.make_dispatcher(
            lambda command: position_response((0, 0, 0, 0, 0, 0))
        )

        self.assertFalse(dispatcher.invalidate("legacy response was invalid"))
        with self.assertRaises(MotionQueueFault):
            dispatcher.submit_delta(0, 1, self.actual, self.profile)

        self.assertTrue(dispatcher.synchronize(self.actual))
        submission = dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertEqual(submission.target[0], 1.0)
        collect_events_until_idle(dispatcher)

    def test_reserves_shared_transport_before_worker_start(self):
        transport_lock = threading.Lock()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            transport_lock=transport_lock,
        )
        transport_lock.acquire()

        with self.assertRaises(MotionTransportBusy):
            dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertTrue(transport_lock.locked())

        transport_lock.release()
        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertTrue(transport_lock.locked())
        collect_events_until_idle(dispatcher)
        self.assertFalse(transport_lock.locked())

    def test_worker_start_failure_rejects_concurrent_submission_atomically(self):
        transport_lock = threading.Lock()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            transport_lock=transport_lock,
        )
        start_entered = threading.Event()
        contender_attempted = threading.Event()
        contender_finished = threading.Event()
        contender_results = []

        class FailingWorker:
            def __init__(self, *args, **kwargs):
                pass

            def start(self):
                start_entered.set()
                if not contender_attempted.wait(1):
                    raise TimeoutError("concurrent submission did not start")
                time.sleep(0.02)
                if contender_finished.is_set():
                    raise AssertionError(
                        "concurrent submission escaped worker-start admission"
                    )
                raise RuntimeError("synthetic worker startup failure")

        def submit_contender():
            if not start_entered.wait(1):
                contender_results.append(TimeoutError("worker start was not entered"))
                contender_finished.set()
                return
            contender_attempted.set()
            try:
                contender_results.append(
                    dispatcher.submit_delta(1, 2, self.actual, self.profile)
                )
            except Exception as exc:
                contender_results.append(exc)
            finally:
                contender_finished.set()

        contender = threading.Thread(target=submit_contender)
        contender.start()
        with patch(
            "ARrobots.HMI.joint_motion.threading.Thread",
            FailingWorker,
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "synthetic worker startup failure",
            ):
                dispatcher.submit_delta(0, 1, self.actual, self.profile)

        contender.join(1)
        self.assertFalse(contender.is_alive())
        self.assertEqual(len(contender_results), 1)
        self.assertIsInstance(contender_results[0], MotionQueueFault)
        self.assertIn("worker startup failed", str(contender_results[0]))
        self.assertFalse(dispatcher.active)
        self.assertFalse(dispatcher.pending)
        self.assertFalse(transport_lock.locked())

        self.assertTrue(dispatcher.synchronize(self.actual))
        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        collect_events_until_idle(dispatcher)

    def test_activity_rejection_releases_the_shared_transport(self):
        transport_lock = threading.Lock()
        registry = SerialActivityRegistry(("ser",))
        registry.begin_shutdown()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            transport_lock=transport_lock,
            activity_factory=lambda: registry.lease("ser"),
        )

        with self.assertRaises(SerialActivityRejected):
            dispatcher.submit_delta(0, 1, self.actual, self.profile)

        self.assertFalse(transport_lock.locked())
        self.assertFalse(dispatcher.active)

    def test_holds_shared_transport_until_result_is_acknowledged(self):
        transport_lock = threading.Lock()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            transport_lock=transport_lock,
        )

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        completed = None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and completed is None:
            for event in dispatcher.drain_events():
                if event.kind == "completed":
                    completed = event
            if completed is None:
                time.sleep(0.005)

        self.assertIsNotNone(completed)
        self.assertTrue(dispatcher.active)
        self.assertTrue(transport_lock.locked())

        completed.acknowledge()
        wait_until(lambda: not dispatcher.active)
        self.assertFalse(transport_lock.locked())

    def test_holds_activity_lease_until_result_is_acknowledged(self):
        registry = SerialActivityRegistry(("ser",))
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            activity_factory=lambda: registry.lease("ser"),
        )

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        completed = None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and completed is None:
            for event in dispatcher.drain_events():
                if event.kind == "completed":
                    completed = event
            if completed is None:
                time.sleep(0.005)

        self.assertIsNotNone(completed)
        self.assertTrue(registry.active("ser"))
        completed.acknowledge()
        wait_until(lambda: not dispatcher.active)
        self.assertTrue(registry.idle())

    def test_admission_failure_retains_failed_transport_rollback(self):
        class FailOnceReleaseLock:
            def __init__(self):
                self.lock = threading.Lock()
                self.release_attempts = 0

            def acquire(self, blocking=True):
                return self.lock.acquire(blocking)

            def release(self):
                self.release_attempts += 1
                if self.release_attempts == 1:
                    raise RuntimeError("injected transport release failure")
                self.lock.release()

            def locked(self):
                return self.lock.locked()

        def reject_activity_admission():
            raise MotionInputError("injected activity admission failure")

        def return_invalid_activity_lease():
            return object()

        admission_cases = (
            (
                reject_activity_admission,
                "injected activity admission failure",
            ),
            (
                return_invalid_activity_lease,
                "activity_factory must return a closeable lease or None",
            ),
        )
        for activity_factory, admission_detail in admission_cases:
            with self.subTest(admission_detail=admission_detail):
                transport_lock = FailOnceReleaseLock()
                dispatcher = self.make_dispatcher(
                    lambda command: position_response((1, 0, 0, 0, 0, 0)),
                    transport_lock=transport_lock,
                    activity_factory=activity_factory,
                )

                with self.assertRaises(MotionQueueFault) as raised:
                    dispatcher.submit_delta(0, 1, self.actual, self.profile)
                self.assertIn(admission_detail, str(raised.exception))
                self.assertIn(
                    "injected transport release failure",
                    str(raised.exception),
                )
                self.assertTrue(transport_lock.locked())
                self.assertTrue(dispatcher._transport_reserved)
                self.assertIsNone(dispatcher._activity_lease)
                self.assertIn(
                    "ownership release failed",
                    dispatcher.fault_reason,
                )
                events = dispatcher.drain_events()
                self.assertEqual(
                    [event.kind for event in events],
                    ["transport-failed"],
                )
                self.assertIn(admission_detail, events[0].error)
                self.assertIn(
                    "injected transport release failure",
                    events[0].error,
                )

                with self.assertRaisesRegex(
                    MotionQueueFault,
                    "ownership release failed",
                ):
                    dispatcher.submit_delta(0, 1, self.actual, self.profile)

                self.assertTrue(dispatcher.synchronize(self.actual))
                self.assertFalse(transport_lock.locked())
                self.assertFalse(dispatcher._transport_reserved)
                self.assertIsNone(dispatcher.fault_reason)

    def test_activity_release_failure_releases_mutex_and_retries_lease(self):
        class FailOnceRegistry(SerialActivityRegistry):
            def __init__(self):
                super().__init__(("ser",))
                self.release_attempts = 0

            def end(self, serial_name, control_injectable=False):
                self.release_attempts += 1
                if self.release_attempts == 1:
                    raise RuntimeError("injected registry release failure")
                return super().end(
                    serial_name,
                    control_injectable=control_injectable,
                )

        transport_lock = threading.Lock()
        registry = FailOnceRegistry()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            transport_lock=transport_lock,
            activity_factory=lambda: registry.lease("ser"),
        )

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in events],
            ["started", "completed", "transport-failed"],
        )
        self.assertIn(
            "injected registry release failure",
            events[-1].error,
        )
        self.assertFalse(transport_lock.locked())
        self.assertTrue(registry.active("ser"))
        self.assertFalse(dispatcher._transport_reserved)
        self.assertIsNotNone(dispatcher._activity_lease)
        self.assertIn("ownership release failed", dispatcher.fault_reason)
        self.assertIsNone(dispatcher.desired_target)
        self.assertTrue(transport_lock.acquire(blocking=False))
        transport_lock.release()
        with self.assertRaisesRegex(
            MotionQueueFault,
            "ownership release failed",
        ):
            dispatcher.submit_delta(0, 1, self.actual, self.profile)

        with registry.operations(("ser",)):
            self.assertTrue(dispatcher.synchronize(self.actual))
            self.assertTrue(registry.active("ser"))
        self.assertFalse(transport_lock.locked())
        self.assertTrue(registry.idle())
        self.assertFalse(dispatcher._transport_reserved)
        self.assertIsNone(dispatcher._activity_lease)
        self.assertIsNone(dispatcher.fault_reason)

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        recovered_events = collect_events_until_idle(dispatcher)
        self.assertEqual(
            [event.kind for event in recovered_events],
            ["started", "completed"],
        )
        self.assertTrue(registry.idle())

    def test_release_reports_and_recovers_multiple_ownership_failures(self):
        class FailOnceRegistry(SerialActivityRegistry):
            def __init__(self):
                super().__init__(("ser",))
                self.release_attempts = 0

            def end(self, serial_name, control_injectable=False):
                self.release_attempts += 1
                if self.release_attempts == 1:
                    raise RuntimeError("injected registry release failure")
                return super().end(
                    serial_name,
                    control_injectable=control_injectable,
                )

        class FailOnceReleaseLock:
            def __init__(self):
                self.lock = threading.Lock()
                self.release_attempts = 0

            def acquire(self, blocking=True):
                return self.lock.acquire(blocking)

            def release(self):
                self.release_attempts += 1
                if self.release_attempts == 1:
                    raise RuntimeError("injected transport release failure")
                self.lock.release()

            def locked(self):
                return self.lock.locked()

        transport_lock = FailOnceReleaseLock()
        registry = FailOnceRegistry()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            transport_lock=transport_lock,
            activity_factory=lambda: registry.lease("ser"),
        )

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(
            [event.kind for event in events],
            ["started", "completed", "transport-failed"],
        )
        self.assertIn("serial activity lease", events[-1].error)
        self.assertIn("controller transport lock", events[-1].error)
        self.assertTrue(transport_lock.locked())
        self.assertTrue(registry.active("ser"))
        self.assertTrue(dispatcher._transport_reserved)
        self.assertIsNotNone(dispatcher._activity_lease)

        self.assertTrue(dispatcher.synchronize(self.actual))
        self.assertFalse(transport_lock.locked())
        self.assertTrue(registry.idle())
        self.assertFalse(dispatcher._transport_reserved)
        self.assertIsNone(dispatcher._activity_lease)
        self.assertIsNone(dispatcher.fault_reason)

    def test_close_reports_retained_lease_failure_without_raising(self):
        class FailTwiceRegistry(SerialActivityRegistry):
            def __init__(self):
                super().__init__(("ser",))
                self.release_attempts = 0

            def end(self, serial_name, control_injectable=False):
                self.release_attempts += 1
                if self.release_attempts <= 2:
                    raise RuntimeError("injected retained lease failure")
                return super().end(
                    serial_name,
                    control_injectable=control_injectable,
                )

        registry = FailTwiceRegistry()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            activity_factory=lambda: registry.lease("ser"),
        )

        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        initial_events = collect_events_until_idle(dispatcher)
        self.assertEqual(
            [event.kind for event in initial_events],
            ["started", "completed", "transport-failed"],
        )
        self.assertTrue(registry.active("ser"))
        self.assertFalse(dispatcher.closed)

        self.assertFalse(dispatcher.close())
        self.assertTrue(dispatcher.closed)
        close_events = dispatcher.drain_events()
        self.assertEqual(close_events, [])
        self.assertTrue(registry.active("ser"))
        self.assertIsNotNone(dispatcher._activity_lease)

        self.assertTrue(dispatcher.close())
        self.assertTrue(registry.idle())
        self.assertIsNone(dispatcher._activity_lease)
        self.assertFalse(dispatcher.synchronize(self.actual))

    def test_idle_worker_release_failure_publishes_without_a_move(self):
        class FailOnceRegistry(SerialActivityRegistry):
            def __init__(self):
                super().__init__(("ser",))
                self.release_attempts = 0

            def end(self, serial_name, control_injectable=False):
                self.release_attempts += 1
                if self.release_attempts == 1:
                    raise RuntimeError("injected idle release failure")
                return super().end(
                    serial_name,
                    control_injectable=control_injectable,
                )

        class DeferredWorker:
            instance = None

            def __init__(self, target, **_kwargs):
                self.target = target
                type(self).instance = self

            def start(self):
                pass

            def run(self):
                self.target()

        transport_lock = threading.Lock()
        registry = FailOnceRegistry()
        dispatcher = self.make_dispatcher(
            lambda command: position_response((1, 0, 0, 0, 0, 0)),
            transport_lock=transport_lock,
            activity_factory=lambda: registry.lease("ser"),
        )

        with patch(
            "ARrobots.HMI.joint_motion.threading.Thread",
            DeferredWorker,
        ):
            dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertTrue(dispatcher.invalidate("external invalidation"))
        DeferredWorker.instance.run()

        events = dispatcher.drain_events()
        self.assertEqual([event.kind for event in events], ["transport-failed"])
        self.assertIsNone(events[0].move)
        self.assertIn("injected idle release failure", events[0].error)
        self.assertFalse(dispatcher.active)
        self.assertFalse(transport_lock.locked())
        self.assertTrue(registry.active("ser"))
        self.assertIn("ownership release failed", dispatcher.fault_reason)

        self.assertTrue(dispatcher.synchronize(self.actual))
        self.assertTrue(registry.idle())
        self.assertIsNone(dispatcher.fault_reason)

    def test_idle_publication_waits_for_transport_release(self):
        transport_lock = ReleaseBarrierLock()
        command_count = 0
        command_count_lock = threading.Lock()

        def exchange(command):
            nonlocal command_count
            with command_count_lock:
                command_count += 1
                position = command_count
            return position_response((position, 0, 0, 0, 0, 0))

        dispatcher = self.make_dispatcher(
            exchange,
            transport_lock=transport_lock,
        )
        dispatcher.submit_delta(0, 1, self.actual, self.profile)

        completed = None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and completed is None:
            for event in dispatcher.drain_events():
                if event.kind == "completed":
                    completed = event
            if completed is None:
                time.sleep(0.005)
        self.assertIsNotNone(completed)

        completed.acknowledge()
        self.assertTrue(transport_lock.release_started.wait(2))

        result = []
        error = []

        def submit_again():
            try:
                result.append(dispatcher.submit_delta(0, 1, self.actual, self.profile))
            except Exception as exc:
                error.append(exc)

        submitter = threading.Thread(target=submit_again)
        submitter.start()
        time.sleep(0.05)
        self.assertTrue(submitter.is_alive())

        transport_lock.allow_release.set()
        submitter.join(2)
        self.assertFalse(submitter.is_alive())
        self.assertEqual(error, [])
        self.assertEqual(len(result), 1)
        collect_events_until_idle(dispatcher)


if __name__ == "__main__":
    unittest.main()
