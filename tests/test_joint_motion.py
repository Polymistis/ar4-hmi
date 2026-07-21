import json
import math
import re
import struct
import threading
import time
import unittest
from unittest.mock import patch

from ARrobots.HMI.joint_motion import (
    AUXILIARY_BOARD_MEGA,
    AUXILIARY_BOARD_NANO,
    AUXILIARY_BOARD_NONE,
    AUXILIARY_BOARD_OUTPUT_PINS,
    CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1,
    CONTROLLER_MAXIMUM_RAMP_PERCENT,
    CoalescingJointDispatcher,
    CommandTiming,
    ControllerIdentity,
    ControllerJointCalibration,
    DeferredLiveMotionArbiter,
    LiveMotionScheduleResult,
    MAX_RESPONSE_FRAME_LENGTH,
    MAX_RESPONSE_PAYLOAD_LENGTH,
    MAX_CONTROLLER_FILENAME_BYTES,
    DeferredJointAdjustments,
    MotionInputError,
    MotionProfile,
    MotionQueueFault,
    MotionRequestLease,
    MotionRequestRegistry,
    MotionTransportBusy,
    ProtocolResponseError,
    SerialActivityRegistry,
    SerialActivityRejected,
    SerialTransportQuarantinedError,
    SerialTransportTimeout,
    VirtualMotionOperation,
    auxiliary_pneumatic_output_pin,
    build_robot_joint_command,
    canonicalize_serial_command,
    canonicalize_virtual_command,
    command_response_timeout,
    controller_degree_to_native_radians,
    controller_number,
    controller_protocol_decimal,
    decode_serial_response_line,
    exchange_serial_line,
    exchange_serial_line_until_cancelled,
    finite_number,
    motion_timing_response_timeout,
    normalize_auxiliary_board_profile,
    parse_command_speed,
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
    validate_controller_filename,
    validate_auxiliary_output_command,
    write_serial_control,
)


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


class FakeSerial:
    def __init__(self, response=b"ok\n", written=None, timeout=7.5):
        self.is_open = True
        self.response = response
        self.written = written
        self.timeout = timeout
        self.reset_count = 0
        self.flush_count = 0
        self.close_count = 0
        self.commands = []

    def reset_input_buffer(self):
        self.reset_count += 1

    def write(self, command):
        self.commands.append(command)
        return len(command) if self.written is None else self.written

    def flush(self):
        self.flush_count += 1

    def close(self):
        self.close_count += 1
        self.is_open = False

    def readline(self):
        if not isinstance(self.response, (bytes, bytearray)):
            return self.response
        newline_index = self.response.find(b"\n")
        size = len(self.response) if newline_index < 0 else newline_index + 1
        return self.read(size)

    def read(self, size=1):
        response = self.response[:size]
        self.response = self.response[size:]
        return response


class BoundedFakeSerial(FakeSerial):
    def __init__(self, response=b"ok\n", written=None):
        super().__init__(response=response, written=written)
        self.read_until_args = None

    def read_until(self, terminator, size):
        self.read_until_args = (terminator, size)
        terminator_index = self.response.find(terminator, 0, size)
        read_size = size if terminator_index < 0 else terminator_index + len(terminator)
        return self.read(read_size)


class SequenceFakeSerial(FakeSerial):
    def __init__(self, responses):
        super().__init__()
        self.responses = list(responses)

    def readline(self):
        if not self.responses:
            return b""
        return self.responses.pop(0)

    def read(self, size=1):
        return b""


class CloseAfterBlankAcknowledgementSerial(SequenceFakeSerial):
    def readline(self):
        if self.responses:
            return self.responses.pop(0)
        self.is_open = False
        return b""


class PartialThenTimeoutFakeSerial(FakeSerial):
    def __init__(self):
        super().__init__(timeout=6.0)
        self._read_count = 0

    def readline(self):
        self._read_count += 1
        if self._read_count == 1:
            return b"partial"
        time.sleep(self.timeout)
        return b""


class FlushFailingFakeSerial(FakeSerial):
    def flush(self):
        self.flush_count += 1
        raise OSError("flush failed")


class FirstFlushFailingFakeSerial(FakeSerial):
    def flush(self):
        self.flush_count += 1
        if self.flush_count == 1:
            raise OSError("initial flush failed")


class InvalidControlEvent:
    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error

    def is_set(self):
        if self.error is not None:
            raise self.error
        return self.result


class SequenceControlEvent:
    def __init__(self, results):
        self.results = list(results)

    def is_set(self):
        result = self.results.pop(0)
        if isinstance(result, BaseException):
            raise result
        return result


class CloseFailingFakeSerial(FakeSerial):
    def close(self):
        self.close_count += 1
        raise OSError("close failed")


class CloseFailingOnceFakeSerial(FakeSerial):
    def close(self):
        self.close_count += 1
        if self.close_count == 1:
            raise OSError("close failed once")
        self.is_open = False


class StopAfterMotionWriteSerial(SequenceFakeSerial):
    def __init__(self, responses, stop_event):
        super().__init__(responses)
        self.stop_event = stop_event

    def write(self, command):
        written = super().write(command)
        if command.startswith(b"L"):
            self.stop_event.set()
        return written


class MarkerAndCloseFailingFakeSerial(CloseFailingFakeSerial):
    def __setattr__(self, name, value):
        if name == "_ar4_transport_quarantine_reason":
            raise AttributeError("custom attributes are unavailable")
        super().__setattr__(name, value)


class RestoreTimeoutFailingFakeSerial(FakeSerial):
    def __init__(self):
        self._timeout = 7.5
        self.fail_original_timeout = False
        super().__init__(timeout=7.5)
        self.fail_original_timeout = True

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if self.fail_original_timeout and value == 7.5:
            raise OSError("timeout restore failed")
        self._timeout = value


class LiveJogFakeSerial(FakeSerial):
    def __init__(self):
        super().__init__()
        self._read_count = 0
        self.read_waiting = threading.Event()
        self.stop_written = threading.Event()
        self.write_threads = []

    def write(self, command):
        self.write_threads.append(threading.get_ident())
        written = super().write(command)
        if command == b"S\n":
            self.stop_written.set()
        return written

    def readline(self):
        self._read_count += 1
        if self._read_count == 1:
            return b"\r\n"
        self.read_waiting.set()
        if self.stop_written.wait(self.timeout):
            return b"final position\n"
        return b""

    def read(self, size=1):
        return b""


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


class SerialLineExchangeTests(unittest.TestCase):
    def test_strict_line_decoder_preserves_only_protocol_delimiters(self):
        self.assertEqual(decode_serial_response_line(b"Done\n"), "Done")
        self.assertEqual(decode_serial_response_line(b"Done\r\n"), "Done")

        for response in (b" Done\n", b"Done \n", b"Done", b"Done\rX\n"):
            with self.subTest(response=response):
                with self.assertRaises(ProtocolResponseError):
                    decode_serial_response_line(response)

    def test_strict_line_decoder_accepts_maximum_lf_and_crlf_payloads(self):
        payload = b"x" * MAX_RESPONSE_PAYLOAD_LENGTH

        for delimiter in (b"\n", b"\r\n"):
            with self.subTest(delimiter=delimiter):
                self.assertEqual(
                    decode_serial_response_line(payload + delimiter),
                    payload.decode("ascii"),
                )

        oversized = payload + b"x"
        for delimiter in (b"\n", b"\r\n"):
            with self.subTest(oversized_delimiter=delimiter):
                with self.assertRaises(ProtocolResponseError):
                    decode_serial_response_line(oversized + delimiter)

    def test_response_owner_accepts_maximum_lf_and_crlf_payloads(self):
        payload = b"x" * MAX_RESPONSE_PAYLOAD_LENGTH

        for delimiter in (b"\n", b"\r\n"):
            with self.subTest(delimiter=delimiter):
                serial_port = BoundedFakeSerial(response=payload + delimiter)
                response = read_serial_line_response(
                    serial_port,
                    20,
                    accepted_responses=(payload.decode("ascii"),),
                )

                self.assertEqual(response, payload.decode("ascii"))
                self.assertEqual(
                    serial_port.read_until_args,
                    (b"\n", MAX_RESPONSE_FRAME_LENGTH + 1),
                )

    def test_response_owner_reads_validated_terminal_line(self):
        serial_port = FakeSerial(response=b"Nano Stopped\r\n", timeout=7.5)

        response = read_serial_line_response(
            serial_port,
            35,
            accepted_responses=("Done", "Timeout", "Nano Stopped"),
        )

        self.assertEqual(response, "Nano Stopped")
        self.assertEqual(serial_port.timeout, 7.5)
        self.assertTrue(serial_port.is_open)

    def test_response_owner_quarantines_a_queued_second_line(self):
        serial_port = BoundedFakeSerial(response=b"Done\nUnexpected\n")

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "queued trailing data",
        ):
            read_serial_line_response(
                serial_port,
                20,
                accepted_responses=("Done",),
            )

        self.assertEqual(
            serial_port.read_until_args,
            (b"\n", MAX_RESPONSE_FRAME_LENGTH + 1),
        )
        self.assertFalse(serial_port.is_open)

    def test_optional_followup_owner_accepts_authorized_second_line(self):
        serial_port = BoundedFakeSerial(
            response=b"Done\nNano Inactive Stopped\r\n"
        )

        response = read_serial_line_response_with_optional_followup(
            serial_port,
            20,
            accepted_responses=("Done", "Timeout", "Nano Stopped"),
            followup_after_responses=("Done", "Timeout"),
            accepted_followup_responses=("Nano Inactive Stopped",),
        )

        self.assertEqual(response, ("Done", "Nano Inactive Stopped"))
        self.assertTrue(serial_port.is_open)

    def test_optional_followup_owner_accepts_a_quiet_primary_response(self):
        serial_port = BoundedFakeSerial(response=b"Timeout\n")

        response = read_serial_line_response_with_optional_followup(
            serial_port,
            20,
            accepted_responses=("Done", "Timeout", "Nano Stopped"),
            followup_after_responses=("Done", "Timeout"),
            accepted_followup_responses=("Nano Inactive Stopped",),
        )

        self.assertEqual(response, ("Timeout", None))
        self.assertTrue(serial_port.is_open)

    def test_optional_followup_owner_quarantines_invalid_followup_sequences(self):
        responses = (
            b"Nano Stopped\nNano Inactive Stopped\n",
            b"Done\nUnexpected\n",
            b"Done\nNano Inactive Stopped\nUnexpected\n",
        )
        for response in responses:
            with self.subTest(response=response):
                serial_port = BoundedFakeSerial(response=response)
                with self.assertRaises(SerialTransportQuarantinedError):
                    read_serial_line_response_with_optional_followup(
                        serial_port,
                        20,
                        accepted_responses=("Done", "Timeout", "Nano Stopped"),
                        followup_after_responses=("Done", "Timeout"),
                        accepted_followup_responses=("Nano Inactive Stopped",),
                    )
                self.assertFalse(serial_port.is_open)

    def test_optional_followup_owner_rejects_invalid_contracts(self):
        with self.assertRaisesRegex(MotionInputError, "subset"):
            read_serial_line_response_with_optional_followup(
                FakeSerial(response=b"Done\n"),
                20,
                accepted_responses=("Done",),
                followup_after_responses=("Timeout",),
                accepted_followup_responses=("Nano Inactive Stopped",),
            )

        valid_contract = {
            "accepted_responses": ("Done", "Timeout"),
            "followup_after_responses": ("Done", "Timeout"),
            "accepted_followup_responses": ("Nano Inactive Stopped",),
        }
        for field_name in valid_contract:
            with self.subTest(field_name=field_name):
                contract = dict(valid_contract)
                contract[field_name] = None
                serial_port = FakeSerial(response=b"Done\n")
                with self.assertRaisesRegex(MotionInputError, "response sets"):
                    read_serial_line_response_with_optional_followup(
                        serial_port,
                        20,
                        **contract,
                    )
                self.assertTrue(serial_port.is_open)

    def test_optional_followup_owner_uses_independent_control_deadline(self):
        serial_port = SequenceFakeSerial(())
        control_event = threading.Event()
        control_event.set()

        with self.assertRaisesRegex(
            SerialTransportTimeout,
            "within 0.02 seconds",
        ):
            read_serial_line_response_with_optional_followup(
                serial_port,
                32772,
                accepted_responses=("Done", "Timeout", "Nano Stopped"),
                followup_after_responses=("Done", "Timeout"),
                accepted_followup_responses=("Nano Inactive Stopped",),
                control_event=control_event,
                control_response_timeout_seconds=0.02,
            )

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_optional_followup_owner_uses_supplied_control_deadline(self):
        serial_port = SequenceFakeSerial(())
        control_event = threading.Event()
        control_event.set()
        provider_calls = []

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            side_effect=(100.0, 100.08, 100.101),
        ):
            with self.assertRaisesRegex(
                SerialTransportTimeout,
                "within 0.1 seconds",
            ):
                read_serial_line_response_with_optional_followup(
                    serial_port,
                    20,
                    accepted_responses=("Done", "Timeout", "Nano Stopped"),
                    followup_after_responses=("Done", "Timeout"),
                    accepted_followup_responses=("Nano Inactive Stopped",),
                    control_event=control_event,
                    control_response_timeout_seconds=0.1,
                    control_response_deadline_provider=(
                        lambda: provider_calls.append(True) or 100.1
                    ),
                )

        self.assertEqual(provider_calls, [True])
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_supplied_control_deadline_replaces_original_read_deadline(self):
        serial_port = SequenceFakeSerial((b"Done\n",))
        control_event = threading.Event()
        control_event.set()

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            side_effect=(100.0, 100.0, 100.03, 100.04),
        ):
            response = read_serial_line_response_with_optional_followup(
                serial_port,
                0.02,
                accepted_responses=("Done", "Timeout", "Nano Stopped"),
                followup_after_responses=("Done", "Timeout"),
                accepted_followup_responses=("Nano Inactive Stopped",),
                control_event=control_event,
                control_response_timeout_seconds=0.1,
                control_response_deadline_provider=lambda: 100.1,
            )

        self.assertEqual(response, ("Done", None))
        self.assertTrue(serial_port.is_open)

    def test_optional_followup_owner_rejects_extended_control_deadline(self):
        serial_port = SequenceFakeSerial(())
        control_event = threading.Event()
        control_event.set()

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            side_effect=(100.0, 100.0),
        ):
            with self.assertRaisesRegex(
                SerialTransportQuarantinedError,
                "deadline exceeds its timeout window",
            ):
                read_serial_line_response_with_optional_followup(
                    serial_port,
                    20,
                    accepted_responses=("Done", "Timeout", "Nano Stopped"),
                    followup_after_responses=("Done", "Timeout"),
                    accepted_followup_responses=("Nano Inactive Stopped",),
                    control_event=control_event,
                    control_response_timeout_seconds=0.1,
                    control_response_deadline_provider=lambda: 100.101,
                )

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_optional_followup_owner_requires_a_full_quiet_probe(self):
        serial_port = FakeSerial(response=b"Timeout\n", timeout=4.0)

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            side_effect=(100.0, 100.0, 100.49),
        ):
            with self.assertRaisesRegex(
                SerialTransportTimeout,
                "follow-up probe deadline expired",
            ):
                read_serial_line_response_with_optional_followup(
                    serial_port,
                    0.5,
                    accepted_responses=("Done", "Timeout", "Nano Stopped"),
                    followup_after_responses=("Done", "Timeout"),
                    accepted_followup_responses=("Nano Inactive Stopped",),
                )

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_explicit_transport_quarantine_is_idempotent(self):
        serial_port = FakeSerial()

        self.assertTrue(
            quarantine_serial_transport(serial_port, "controller state uncertain")
        )
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))
        self.assertFalse(
            quarantine_serial_transport(serial_port, "controller state uncertain")
        )

        close_failing_port = CloseFailingFakeSerial()
        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "serial close failed",
        ):
            quarantine_serial_transport(
                close_failing_port,
                "controller state uncertain",
            )
        self.assertTrue(close_failing_port.is_open)
        self.assertTrue(serial_transport_quarantined(close_failing_port))

        close_failing_once_port = CloseFailingOnceFakeSerial()
        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "serial close failed",
        ):
            quarantine_serial_transport(
                close_failing_once_port,
                "controller state uncertain",
            )
        self.assertTrue(
            quarantine_serial_transport(
                close_failing_once_port,
                "controller state uncertain",
            )
        )
        self.assertFalse(close_failing_once_port.is_open)
        self.assertEqual(close_failing_once_port.close_count, 2)

        for reason in ("", " padded", "two\nlines"):
            with self.subTest(reason=reason):
                with self.assertRaises(MotionInputError):
                    quarantine_serial_transport(FakeSerial(), reason)

    def test_response_owner_quarantines_empty_terminal_read(self):
        serial_port = FakeSerial(response=b"", timeout=4.0)

        with self.assertRaises(SerialTransportTimeout):
            read_serial_line_response(serial_port, 0.02)

        self.assertEqual(serial_port.timeout, 4.0)
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_response_owner_quarantines_expired_quiet_boundary(self):
        serial_port = FakeSerial(response=b"Done\n", timeout=4.0)

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            side_effect=(100.0, 100.0, 100.49),
        ):
            with self.assertRaisesRegex(
                SerialTransportTimeout,
                "quiet-boundary deadline expired",
            ):
                read_serial_line_response(serial_port, 0.5)

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_response_owner_honors_supplied_absolute_deadline(self):
        serial_port = FakeSerial(response=b"Done\n", timeout=4.0)

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            side_effect=(100.0, 100.0, 100.09),
        ):
            with self.assertRaisesRegex(
                SerialTransportTimeout,
                "quiet-boundary deadline expired",
            ):
                read_serial_line_response(
                    serial_port,
                    0.5,
                    response_deadline=100.1,
                )

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_response_owner_rejects_extended_absolute_deadline(self):
        serial_port = FakeSerial(response=b"Done\n")

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            return_value=100.0,
        ):
            with self.assertRaisesRegex(
                MotionInputError,
                "deadline exceeds its timeout window",
            ):
                read_serial_line_response(
                    serial_port,
                    0.5,
                    response_deadline=100.501,
                )

        self.assertTrue(serial_port.is_open)
        self.assertFalse(serial_transport_quarantined(serial_port))

    def test_response_owner_quarantines_unexpected_terminal_line(self):
        serial_port = FakeSerial(response=b"Unexpected\n", timeout=3.0)

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "unexpected response",
        ):
            read_serial_line_response(
                serial_port,
                20,
                accepted_responses=("Done", "Timeout", "Nano Stopped"),
            )

        self.assertEqual(serial_port.timeout, 3.0)
        self.assertFalse(serial_port.is_open)

    def test_response_owner_rejects_invalid_terminal_contract(self):
        for accepted in ("Done", (), ("Done", "Done"), (" Done",)):
            with self.subTest(accepted=accepted):
                with self.assertRaises(MotionInputError):
                    read_serial_line_response(
                        FakeSerial(response=b"Done\n"),
                        20,
                        accepted_responses=accepted,
                    )

    def test_response_owner_rejects_payload_padding(self):
        for response in (b" Done\n", b"Done \r\n"):
            with self.subTest(response=response):
                serial_port = BoundedFakeSerial(response=response)
                with self.assertRaisesRegex(
                    SerialTransportQuarantinedError,
                    "payload whitespace",
                ):
                    read_serial_line_response(serial_port, 20)
                self.assertFalse(serial_port.is_open)

    def test_exact_response_owner_accepts_complete_unframed_acknowledgement(self):
        class ExactSerial(FakeSerial):
            def read(self, size):
                chunk_size = min(size, 2)
                response = self.response[:chunk_size]
                self.response = self.response[chunk_size:]
                return response

        for expected in (b"Servo Done", b"Done"):
            with self.subTest(expected=expected):
                serial_port = ExactSerial(response=expected, timeout=6.0)
                response = read_serial_exact_response(
                    serial_port,
                    expected,
                    20,
                )

                self.assertEqual(response, expected.decode("ascii"))
                self.assertEqual(serial_port.timeout, 6.0)
                self.assertTrue(serial_port.is_open)

    def test_exact_response_owner_quarantines_empty_and_partial_data(self):
        class ExactSerial(FakeSerial):
            def read(self, size):
                response = self.response[:size]
                self.response = self.response[size:]
                return response

        for response in (b"", b"Servo"):
            with self.subTest(response=response):
                serial_port = ExactSerial(response=response, timeout=6.0)
                with self.assertRaises(SerialTransportTimeout):
                    read_serial_exact_response(
                        serial_port,
                        b"Servo Done",
                        0.02,
                    )
                self.assertFalse(serial_port.is_open)
                self.assertTrue(serial_transport_quarantined(serial_port))

    def test_exact_response_owner_quarantines_unexpected_data(self):
        class ExactSerial(FakeSerial):
            def read(self, size):
                response = self.response[:size]
                self.response = self.response[size:]
                return response

        serial_port = ExactSerial(response=b"Nope")
        with self.assertRaises(SerialTransportQuarantinedError):
            read_serial_exact_response(serial_port, b"Done", 20)

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_exact_response_owner_quarantines_valid_prefix_with_trailing_data(self):
        class ExactSerial(FakeSerial):
            def read(self, size):
                response = self.response[:size]
                self.response = self.response[size:]
                return response

        serial_port = ExactSerial(response=b"Donejunk", timeout=6.0)
        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "trailing unframed response data",
        ):
            read_serial_exact_response(serial_port, b"Done", 20)

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_exact_response_owner_requires_a_full_quiet_boundary(self):
        serial_port = FakeSerial(response=b"Done", timeout=6.0)

        with patch(
            "ARrobots.HMI.joint_motion.time.monotonic",
            side_effect=(100.0, 100.0, 100.49),
        ):
            with self.assertRaisesRegex(
                SerialTransportTimeout,
                "quiet-boundary deadline expired",
            ):
                read_serial_exact_response(serial_port, b"Done", 0.5)

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_cancellation_bound_exchange_ignores_idle_read_intervals(self):
        serial_port = SequenceFakeSerial((b"", b"", b"complete\n"))

        response = exchange_serial_line_until_cancelled(
            serial_port,
            "PGFndemo.txt\n",
            threading.Event(),
            poll_interval_seconds=0.001,
        )

        self.assertEqual(response, "complete")
        self.assertEqual(serial_port.commands, [b"PGFndemo.txt\n"])
        self.assertTrue(serial_port.is_open)

    def test_cancellation_bound_exchange_quarantines_after_transmission(self):
        cancellation = threading.Event()

        class CancellingSerial(FakeSerial):
            def readline(self):
                cancellation.set()
                return b""

        serial_port = CancellingSerial()
        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line_until_cancelled(
                serial_port,
                "PGFndemo.txt\n",
                cancellation,
                poll_interval_seconds=0.001,
            )

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_cancellation_bound_exchange_rechecks_during_write_admission(self):
        cancellation = threading.Event()
        write_started = threading.Event()

        class CancellingResetSerial(FakeSerial):
            def reset_input_buffer(self):
                super().reset_input_buffer()
                cancellation.set()

        serial_port = CancellingResetSerial()
        with self.assertRaisesRegex(
            SerialActivityRejected,
            "cancelled before transmission",
        ):
            exchange_serial_line_until_cancelled(
                serial_port,
                "PGFndemo.txt\n",
                cancellation,
                poll_interval_seconds=0.001,
                write_started_event=write_started,
            )

        self.assertEqual(serial_port.commands, [])
        self.assertFalse(write_started.is_set())
        self.assertTrue(serial_port.is_open)
        self.assertFalse(serial_transport_quarantined(serial_port))

    def test_control_write_preserves_response_ownership(self):
        serial_port = FakeSerial()
        write_lock = threading.Lock()
        write_started = threading.Event()

        self.assertTrue(
            write_serial_control(
                serial_port,
                "STOP\n",
                write_lock=write_lock,
                write_started_event=write_started,
            )
        )

        self.assertEqual(serial_port.commands, [b"STOP\n"])
        self.assertEqual(serial_port.flush_count, 1)
        self.assertEqual(serial_port.reset_count, 0)
        self.assertTrue(serial_port.is_open)
        self.assertTrue(write_started.is_set())

    def test_control_write_resets_before_serialized_legacy_command(self):
        class OrderedSerial(FakeSerial):
            def __init__(self):
                super().__init__()
                self.events = []

            def reset_input_buffer(self):
                self.events.append("reset")
                super().reset_input_buffer()

            def write(self, command):
                self.events.append(("write", command))
                return super().write(command)

            def flush(self):
                self.events.append("flush")
                super().flush()

        serial_port = OrderedSerial()

        self.assertTrue(
            write_serial_control(
                serial_port,
                "SV0P50\n",
                write_lock=threading.Lock(),
                reset_input=True,
            )
        )

        self.assertEqual(
            serial_port.events,
            ["reset", ("write", b"SV0P50\n"), "flush"],
        )
        self.assertEqual(serial_port.reset_count, 1)

    def test_control_write_rejects_non_boolean_reset_flag(self):
        with self.assertRaisesRegex(MotionInputError, "reset_input must be boolean"):
            write_serial_control(FakeSerial(), "STOP\n", reset_input=1)

    def test_performs_one_framed_exchange(self):
        serial_port = FakeSerial(response=b"A response\r\n")

        response = exchange_serial_line(serial_port, "RP\n", 120)

        self.assertEqual(response, "A response")
        self.assertEqual(serial_port.commands, [b"RP\n"])
        self.assertEqual(serial_port.reset_count, 1)
        self.assertEqual(serial_port.flush_count, 1)
        self.assertEqual(serial_port.timeout, 7.5)

    def test_exchange_marks_write_start_before_serial_write(self):
        write_started = threading.Event()

        class ObservingSerial(FakeSerial):
            def __init__(self):
                super().__init__(response=b"Done\n")
                self.write_start_states = []

            def write(self, command):
                self.write_start_states.append(write_started.is_set())
                return super().write(command)

        serial_port = ObservingSerial()

        response = exchange_serial_line(
            serial_port,
            "RP\n",
            120,
            write_started_event=write_started,
        )

        self.assertEqual(response, "Done")
        self.assertEqual(serial_port.write_start_states, [True])
        self.assertTrue(write_started.is_set())

    def test_rejects_incomplete_write(self):
        serial_port = FakeSerial(written=1, timeout=4.0)

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "reconnect required",
        ):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertEqual(serial_port.timeout, 4.0)
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))
        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line(serial_port, "RP\n", 120)

    def test_flush_failure_quarantines_transport(self):
        serial_port = FlushFailingFakeSerial()

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "reconnect required",
        ):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertEqual(serial_port.commands, [b"RP\n"])
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_failed_close_still_poisoned_transport(self):
        serial_port = CloseFailingFakeSerial(written=1)

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "connection retained for cleanup; reconnect required",
        ):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertTrue(serial_port.is_open)
        self.assertEqual(serial_port.close_count, 1)
        self.assertTrue(serial_transport_quarantined(serial_port))
        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line(serial_port, "RP\n", 120)
        self.assertEqual(serial_port.commands, [b"RP\n"])

    def test_failed_marker_and_close_still_poison_transport(self):
        serial_port = MarkerAndCloseFailingFakeSerial(written=1)

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "quarantine marker failed",
        ):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertTrue(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))
        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line(serial_port, "RP\n", 120)
        self.assertEqual(serial_port.commands, [b"RP\n"])

    def test_timeout_restore_failure_quarantines_transport(self):
        serial_port = RestoreTimeoutFailingFakeSerial()

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "unable to restore the serial read timeout",
        ):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_rejects_unframed_or_non_ascii_command(self):
        with self.assertRaises(MotionInputError):
            exchange_serial_line(FakeSerial(), "RP", 120)
        with self.assertRaises(MotionInputError):
            exchange_serial_line(FakeSerial(), "RÉ\n", 120)
        with self.assertRaises(MotionInputError):
            exchange_serial_line(FakeSerial(), "RP\nRJ\n", 120)
        with self.assertRaises(MotionInputError):
            exchange_serial_line(FakeSerial(), "\n", 120)

    def test_empty_read_is_a_timeout(self):
        serial_port = FakeSerial(response=b"", timeout=3.0)

        with self.assertRaisesRegex(TimeoutError, "closed; reconnect required"):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertFalse(serial_port.is_open)
        self.assertEqual(serial_port.close_count, 1)
        self.assertEqual(serial_port.timeout, 3.0)
        with self.assertRaises(ConnectionError):
            exchange_serial_line(serial_port, "RP\n", 120)

    def test_partial_line_times_out_and_quarantines_transport(self):
        serial_port = PartialThenTimeoutFakeSerial()

        with self.assertRaisesRegex(TimeoutError, "closed; reconnect required"):
            exchange_serial_line(serial_port, "RP\n", 0.02)

        self.assertFalse(serial_port.is_open)
        self.assertEqual(serial_port.close_count, 1)
        self.assertEqual(serial_port.timeout, 6.0)

    def test_rejects_non_ascii_response_line(self):
        serial_port = FakeSerial(response=b"bad\xff\n")

        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_oversized_unterminated_response_quarantines_transport(self):
        serial_port = FakeSerial(
            response=b"x" * (MAX_RESPONSE_FRAME_LENGTH + 1)
        )

        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line(serial_port, "RP\n", 120)

        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_bounds_read_until_when_supported(self):
        serial_port = BoundedFakeSerial(response=b"A response\n")

        response = exchange_serial_line(serial_port, "RP\n", 5)

        self.assertEqual(response, "A response")
        self.assertEqual(
            serial_port.read_until_args,
            (b"\n", MAX_RESPONSE_FRAME_LENGTH + 1),
        )

    def test_rejects_blank_response_without_live_control(self):
        serial_port = SequenceFakeSerial((b"\r\n",))

        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line(serial_port, "RP\n", 1)

        self.assertFalse(serial_port.is_open)

    def test_live_terminal_data_before_acknowledgement_is_quarantined(self):
        serial_port = SequenceFakeSerial((b"unexpected\n",))

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "terminal data before live acknowledgement",
        ):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=threading.Event(),
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_live_physical_estop_accepts_one_controller_terminal_position(self):
        response = position_response(
            (1, 2, 3, 4, 5, 6),
            external=(7, 8, 9),
            flag="EB",
        )
        serial_port = SequenceFakeSerial((b"\n", response.encode() + b"\n"))

        received = exchange_serial_line(
            serial_port,
            "LJV10\n",
            1,
            control_event=threading.Event(),
            control_command="S\n",
            control_ack_timeout_seconds=0.5,
            control_response_timeout_seconds=0.5,
        )

        self.assertEqual(received, response)
        self.assertEqual(serial_port.commands, [b"LJV10\n"])
        self.assertTrue(serial_port.is_open)
        self.assertFalse(serial_transport_quarantined(serial_port))

    def test_live_blank_acknowledgement_detects_transport_closure(self):
        serial_port = CloseAfterBlankAcknowledgementSerial((b"\n",))

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "closed during live response ownership",
        ):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=threading.Event(),
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_live_physical_estop_rejects_a_second_terminal_position(self):
        estop_response = position_response(
            (1, 2, 3, 4, 5, 6),
            external=(7, 8, 9),
            flag="EB",
        )
        normal_response = position_response(
            (1, 2, 3, 4, 5, 6),
            external=(7, 8, 9),
        )
        serial_port = FakeSerial(
            response=(
                b"\n"
                + estop_response.encode()
                + b"\n"
                + normal_response.encode()
                + b"\n"
            )
        )

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "queued trailing data",
        ):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=threading.Event(),
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_live_release_before_transmission_sends_no_motion_or_stop(self):
        stop_requested = threading.Event()
        stop_requested.set()
        serial_port = SequenceFakeSerial((b"\n", b"final position\n"))

        with self.assertRaisesRegex(
            SerialActivityRejected,
            "stopped before transmission",
        ):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=stop_requested,
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [])
        self.assertTrue(serial_port.is_open)
        self.assertFalse(serial_transport_quarantined(serial_port))

    def test_live_early_stop_consumes_acknowledgement_before_terminal_data(self):
        stop_requested = threading.Event()
        serial_port = StopAfterMotionWriteSerial(
            (b"\n", b"final position\n"),
            stop_requested,
        )

        response = exchange_serial_line(
            serial_port,
            "LJV10\n",
            1,
            control_event=stop_requested,
            control_command="S\n",
            control_ack_timeout_seconds=0.5,
            control_response_timeout_seconds=0.5,
        )

        self.assertEqual(response, "final position")
        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertTrue(serial_port.is_open)

    def test_live_duplicate_acknowledgement_is_quarantined(self):
        stop_requested = threading.Event()
        serial_port = StopAfterMotionWriteSerial(
            (b"\n", b"\n"),
            stop_requested,
        )

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "duplicate live acknowledgement",
        ):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=stop_requested,
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertFalse(serial_port.is_open)

    def test_live_malformed_frames_attempt_fail_safe_stop(self):
        cases = (
            (object(), "non-bytes"),
            (b"x" * (MAX_RESPONSE_FRAME_LENGTH + 1), "size limit"),
            (b"\xff\n", "not valid ASCII"),
        )
        for response, expected_error in cases:
            with self.subTest(expected_error=expected_error):
                serial_port = FakeSerial(response=response)

                with self.assertRaisesRegex(
                    SerialTransportQuarantinedError,
                    re.escape(expected_error),
                ):
                    exchange_serial_line(
                        serial_port,
                        "LJV10\n",
                        1,
                        control_event=threading.Event(),
                        control_command="S\n",
                        control_ack_timeout_seconds=0.5,
                        control_response_timeout_seconds=0.5,
                    )

                self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
                self.assertFalse(serial_port.is_open)
                self.assertTrue(serial_transport_quarantined(serial_port))

    def test_live_malformed_frame_does_not_duplicate_sent_stop(self):
        stop_requested = threading.Event()
        serial_port = StopAfterMotionWriteSerial((b"\xff\n",), stop_requested)

        with self.assertRaises(SerialTransportQuarantinedError):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=stop_requested,
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])

    def test_live_initial_transmission_failure_attempts_fail_safe_stop(self):
        serial_port = FirstFlushFailingFakeSerial()

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "initial flush failed",
        ):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=threading.Event(),
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertEqual(serial_port.flush_count, 2)

    def test_live_invalid_control_event_rejects_before_transmission(self):
        cases = (
            (
                InvalidControlEvent(error=RuntimeError("event state failed")),
                "event state failed",
            ),
            (
                InvalidControlEvent(result=1),
                "control_event.is_set() must return a boolean",
            ),
        )
        for control_event, expected_error in cases:
            with self.subTest(expected_error=expected_error):
                serial_port = FakeSerial()

                with self.assertRaisesRegex(
                    MotionInputError,
                    "control_event",
                ):
                    exchange_serial_line(
                        serial_port,
                        "LJV10\n",
                        1,
                        control_event=control_event,
                        control_command="S\n",
                        control_ack_timeout_seconds=0.5,
                        control_response_timeout_seconds=0.5,
                    )

                self.assertEqual(serial_port.commands, [])
                self.assertTrue(serial_port.is_open)
                self.assertFalse(serial_transport_quarantined(serial_port))

    def test_live_control_event_failure_after_transmission_quarantines(self):
        control_event = SequenceControlEvent(
            (False, False, MotionInputError("event state failed after write"))
        )
        serial_port = SequenceFakeSerial(())

        with self.assertRaisesRegex(
            SerialTransportQuarantinedError,
            "event state failed after write",
        ):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=control_event,
                control_command="S\n",
                control_ack_timeout_seconds=0.5,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertFalse(serial_port.is_open)
        self.assertTrue(serial_transport_quarantined(serial_port))

    def test_live_control_uses_same_worker_and_response_reader(self):
        serial_port = LiveJogFakeSerial()
        write_lock = threading.Lock()
        stop_requested = threading.Event()
        result = []

        worker = threading.Thread(
            target=lambda: result.append(
                exchange_serial_line(
                    serial_port,
                    "LJV10\n",
                    1,
                    write_lock=write_lock,
                    control_event=stop_requested,
                    control_command="S\n",
                    control_ack_timeout_seconds=0.5,
                    control_response_timeout_seconds=0.5,
                )
            )
        )
        worker.start()
        self.assertTrue(serial_port.read_waiting.wait(1))
        stop_requested.set()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(result, ["final position"])
        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertEqual(len(set(serial_port.write_threads)), 1)
        self.assertEqual(serial_port.reset_count, 1)
        self.assertEqual(serial_port.timeout, 7.5)

    def test_live_acknowledgement_suspends_deadline_until_stop(self):
        serial_port = LiveJogFakeSerial()
        stop_requested = threading.Event()
        result = []
        errors = []

        def exchange():
            try:
                result.append(
                    exchange_serial_line(
                        serial_port,
                        "LJV10\n",
                        0.02,
                        control_event=stop_requested,
                        control_command="S\n",
                        control_ack_timeout_seconds=0.5,
                        control_response_timeout_seconds=0.5,
                    )
                )
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=exchange)
        worker.start()
        try:
            self.assertTrue(serial_port.read_waiting.wait(1))
            time.sleep(0.08)
            self.assertTrue(worker.is_alive())
            self.assertTrue(serial_port.is_open)
        finally:
            stop_requested.set()
            worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(result, ["final position"])
        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])

    def test_live_timeout_writes_fail_safe_stop_without_resetting_input(self):
        serial_port = SequenceFakeSerial(())

        with self.assertRaises(TimeoutError):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                0.02,
                control_event=threading.Event(),
                control_command="S\n",
                control_ack_timeout_seconds=0.02,
                control_response_timeout_seconds=0.5,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertEqual(serial_port.reset_count, 1)
        self.assertFalse(serial_port.is_open)
        self.assertEqual(serial_port.close_count, 1)
        self.assertEqual(serial_port.timeout, 7.5)

    def test_live_stop_honors_the_supplied_response_deadline(self):
        stop_requested = threading.Event()
        serial_port = StopAfterMotionWriteSerial((b"\n",), stop_requested)

        with self.assertRaisesRegex(TimeoutError, "within 0.1 seconds"):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                1,
                control_event=stop_requested,
                control_command="S\n",
                control_ack_timeout_seconds=1,
                control_response_timeout_seconds=0.1,
            )

        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertFalse(serial_port.is_open)

    def test_live_start_ack_uses_short_deadline_independent_of_motion_timeout(self):
        serial_port = SequenceFakeSerial(())
        started = time.monotonic()

        with self.assertRaisesRegex(TimeoutError, "within 0.02 seconds"):
            exchange_serial_line(
                serial_port,
                "LJV10\n",
                10000,
                control_event=threading.Event(),
                control_command="S\n",
                control_ack_timeout_seconds=0.02,
                control_response_timeout_seconds=10000,
            )

        self.assertLess(time.monotonic() - started, 1)
        self.assertEqual(serial_port.commands, [b"LJV10\n", b"S\n"])
        self.assertFalse(serial_port.is_open)

    def test_live_control_requires_a_separate_response_timeout(self):
        with self.assertRaisesRegex(
            MotionInputError,
            "control_ack_timeout_seconds",
        ):
            exchange_serial_line(
                FakeSerial(),
                "LJV10\n",
                120,
                control_event=threading.Event(),
                control_command="S\n",
            )

        with self.assertRaisesRegex(
            MotionInputError,
            "control_response_timeout_seconds",
        ):
            exchange_serial_line(
                FakeSerial(),
                "LJV10\n",
                120,
                control_event=threading.Event(),
                control_command="S\n",
                control_ack_timeout_seconds=5,
            )

        with self.assertRaisesRegex(MotionInputError, "require a control_event"):
            exchange_serial_line(
                FakeSerial(),
                "RP\n",
                120,
                control_response_timeout_seconds=5,
            )


class CommandResponseTimeoutTests(unittest.TestCase):
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
            "Sp50Ac10Dc20Rm25WNLm000000FnSampleGcode.txt\n"
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
            f"MG{cartesian}{timing}WNLm000000\n",
            f"ML{cartesian}{timing}Rnd0WNLm000000Q0\n",
            f"MV{cartesian}{timing}WNVr0Lm000000\n",
            f"WC{cartesian}{timing}WNLm000000Fndemo.txt\n",
            f"WG{cartesian}{timing}WNLm000000Fndemo.txt\n",
            (
                "MAX1Y2Z3Rz4Ry5Rx6Ex7Ey8Ez9Tr0"
                f"{timing}WNLm000000\n"
            ),
            (
                "MCCx1Cy2Cz3Rz4Ry5Rx6Bx7By8Bz9Px10Py11Pz12Tr0"
                f"{timing}WNLm000000\n"
            ),
            f"LJV10{timing}WALm000000\n",
            f"LCV20{timing}WALm000000\n",
            f"LTV30{timing}WALm000000\n",
        )
        expected = CommandTiming("p", 50.0, 10.0, 20.0, 25.0)
        for command in commands:
            with self.subTest(command=command[:2]):
                self.assertEqual(parse_command_timing(command), expected)

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
            f"MG{cartesian}{timing}Rnd1WNLm000000\n",
            f"MJ{cartesian}{timing}Rnd1WNLm000000\n",
            f"ML{cartesian}{timing}Rnd0WNLm000000Q1\n",
            f"MV{cartesian}{timing}Rnd1WNVr0Lm000000\n",
            f"WC{cartesian}{timing}Rnd1WNLm000000Fndemo.txt\n",
            f"WG{cartesian}{timing}Rnd1WNLm000000Fndemo.txt\n",
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
        with self.assertRaisesRegex(MotionInputError, "trajectory rotation"):
            parse_command_timing(
                f"MAX1Y2Z3Rz4Ry5Rx6Ex7Ey8Ez9Tr1{timing}WNLm000000\n"
            )
        with self.assertRaisesRegex(MotionInputError, "trajectory rotation"):
            parse_command_timing(
                "MCCx1Cy2Cz3Rz4Ry5Rx6Bx7By8Bz9Px10Py11Pz12Tr-1"
                f"{timing}WNLm000000\n"
            )
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

    def test_controller_filenames_reject_fat_reserved_characters(self):
        timing = "Sp50Ac10Dc20Rm25"
        cartesian = "X1Y2Z3Rz4Ry5Rx6J70J80J90"
        calibration = controller_calibration()
        for reserved in '"*/:<>?\\|':
            with self.subTest(reserved=reserved):
                with self.assertRaisesRegex(MotionInputError, "FAT-reserved"):
                    validate_controller_filename(
                        f"demo{reserved}file.txt",
                        "test filename",
                    )
        commands = (
            "PGFn../demo.txt\n",
            "PGFnC:demo.txt\n",
            f"WC{cartesian}{timing}WNLm000000Fnfolder/demo.txt\n",
            f"WC{cartesian}{timing}WNLm000000Fnfolder/evilFndemo.txt\n",
            f"WG{cartesian}{timing}WNLm000000Fnfolder\\demo.txt\n",
        )

        for command in commands:
            with self.subTest(opcode=command[:2]):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "FAT-reserved",
                ):
                    canonicalize_serial_command(command, calibration)

    def test_controller_filename_byte_limit_matches_storage_commands(self):
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
            f"WC{cartesian}{timing}WNLm000000Fn{maximum}\n",
            f"WG{cartesian}{timing}WNLm000000Fn{maximum}\n",
        ):
            with self.subTest(accepted=command[:2]):
                self.assertEqual(
                    canonicalize_serial_command(command, calibration),
                    command,
                )

        for command in (
            f"PGFn{oversized}\n",
            f"WC{cartesian}{timing}WNLm000000Fn{oversized}\n",
            f"WG{cartesian}{timing}WNLm000000Fn{oversized}\n",
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
        )
        expected = CommandTiming("p", 50.0, 10.0, 20.0, 25.0)

        for command in virtual_commands:
            with self.subTest(command=command[:2]):
                self.assertEqual(parse_virtual_command_timing(command), expected)
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

    def test_opcode_schemas_reject_missing_required_suffix_fields(self):
        cartesian = "X1Y2Z3Rz4Ry5Rx6J70J80J90"
        timing = "Sp50Ac10Dc20Rm25"

        with self.assertRaises(MotionInputError):
            parse_command_timing(
                f"WG{cartesian}{timing}WNLm000000\n"
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

    def test_parses_controller_identity_and_capabilities(self):
        response = json.dumps(
            {
                "DriverModel": "Teensy 4.1",
                "FirmwareVersion": "6.7.1-ar4hmi.1",
                "RobotModel": 'AR4\\"Model',
                "RobotVersion": "MK3",
                "SerialNumber": "SN\\42",
                "AssetTag": "Unset",
                "ProtocolCapabilities": ["JT_WRIST_CONFIG_V1"],
            },
            separators=(",", ":"),
        )

        identity = parse_controller_identity_response(response)

        self.assertIsInstance(identity, ControllerIdentity)
        self.assertEqual(identity.firmware_version, "6.7.1-ar4hmi.1")
        self.assertEqual(identity.robot_model, 'AR4\\"Model')
        self.assertEqual(identity.serial_number, "SN\\42")
        self.assertIn(
            CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1,
            identity.protocol_capabilities,
        )

    def test_controller_identity_fields_match_firmware_storage_contract(self):
        payload = {
            "DriverModel": "Teensy 4.1",
            "FirmwareVersion": "6.7.1-ar4hmi.1",
            "RobotModel": "A" * 31,
            "RobotVersion": "MK3",
            "SerialNumber": "Unset",
            "AssetTag": "Unset",
            "ProtocolCapabilities": ["JT_WRIST_CONFIG_V1"],
        }
        self.assertEqual(
            parse_controller_identity_response(
                json.dumps(payload, separators=(",", ":"))
            ).robot_model,
            "A" * 31,
        )

        for legacy_printable in (
            " AR4",
            "AR4 ",
            "AR[4",
            "AR]4",
            "AR[M]4",
            "AR[V]4",
            "AR[B]4",
            "AR[S]4",
            "AR[A]4",
        ):
            with self.subTest(legacy_printable=legacy_printable):
                payload["RobotModel"] = legacy_printable
                self.assertEqual(
                    parse_controller_identity_response(
                        json.dumps(payload, separators=(",", ":"))
                    ).robot_model,
                    legacy_printable,
                )

        for invalid in ("", "A" * 32, "AR\n4", "ARé"):
            with self.subTest(invalid=invalid):
                payload["RobotModel"] = invalid
                with self.assertRaises(ProtocolResponseError):
                    parse_controller_identity_response(
                        json.dumps(payload, separators=(",", ":"))
                    )

    def test_rejects_malformed_controller_identity_capabilities(self):
        base = (
            '{"DriverModel":"Teensy 4.1","FirmwareVersion":"version",'
            '"RobotModel":"AR4","RobotVersion":"MK3",'
            '"SerialNumber":"Unset","AssetTag":"Unset",'
            '"ProtocolCapabilities":%s}'
        )
        malformed = (
            "null",
            '"JT_WRIST_CONFIG_V1"',
            '["lowercase"]',
            f'["{"A" * 32}"]',
            '["JT_WRIST_CONFIG_V1","JT_WRIST_CONFIG_V1"]',
        )

        for capabilities in malformed:
            with self.subTest(capabilities=capabilities):
                with self.assertRaises(ProtocolResponseError):
                    parse_controller_identity_response(base % capabilities)

    def test_controller_identity_requires_an_exact_unique_schema(self):
        valid = (
            '{"DriverModel":"Teensy 4.1","FirmwareVersion":"version",'
            '"RobotModel":"AR4","RobotVersion":"MK3",'
            '"SerialNumber":"Unset","AssetTag":"Unset",'
            '"ProtocolCapabilities":["JT_WRIST_CONFIG_V1"]}'
        )
        malformed = (
            valid.replace(',"AssetTag":"Unset"', ''),
            valid[:-1] + ',"Unexpected":true}',
            valid.replace(
                '"RobotModel":"AR4"',
                '"RobotModel":"AR4","RobotModel":"AR5"',
            ),
        )

        for response in malformed:
            with self.subTest(response=response):
                with self.assertRaises(ProtocolResponseError):
                    parse_controller_identity_response(response)

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


class CoalescingJointDispatcherTests(unittest.TestCase):
    def setUp(self):
        self.profile = MotionProfile("Sp", 50, 10, 10, 25, "N", "000000")
        self.actual = (0,) * 9
        self.calibration = controller_calibration()

    def make_dispatcher(self, exchange, **kwargs):
        return CoalescingJointDispatcher(
            exchange,
            lambda: self.calibration,
            **kwargs,
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
                    (1, 0, 0, 0, 0, 0),
                    speed_violation=1,
                )
            return position_response((2, 0, 0, 0, 0, 0))

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
        self.assertEqual(dispatcher.desired_target, confirmed)
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
        self.assertTrue(commands[1].startswith("RJA2B0C0D0E0F0J70J80J90"))

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
            return "EL100000000"

        dispatcher = self.make_dispatcher(exchange)
        dispatcher.submit_delta(0, 1, self.actual, self.profile)
        self.assertTrue(first_started.wait(2))
        dispatcher.submit_delta(1, 2, self.actual, self.profile)
        release_first.set()
        events = collect_events_until_idle(dispatcher)

        self.assertEqual(len(commands), 1)
        self.assertEqual(events[-1].kind, "failed")
        self.assertTrue(events[-1].pending_discarded)
        self.assertIn("controller rejected motion", dispatcher.fault_reason)
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
        for flag in ("", "EB"):
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
