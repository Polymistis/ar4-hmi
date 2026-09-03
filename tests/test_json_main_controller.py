from dataclasses import replace
import json
import threading
import unittest

if __package__:
    from .bounded_temp import BoundedTemporaryDirectory
else:
    from bounded_temp import BoundedTemporaryDirectory

import ARrobots.protocol as protocol
from ARrobots.protocol import (
    JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS,
    JSON_MAIN_FIRMWARE_FRAME_RECEIVE_TIMEOUT_SECONDS,
    MAIN_CORRECT_POSITION_COMMAND_CONTRACT,
    JsonCommandSchemaError,
    JsonMainControllerClient,
    JsonMainControllerClientStateError,
    JsonMainControllerStartupConfiguration,
    JsonMainControllerTerminal,
    JsonMainCalibrationResult,
    JsonMainCartesianMotionResult,
    JsonMainHelloResult,
    JsonMainHomeReferenceResult,
    JsonMainJointMotionResult,
    JsonMainLiveJogResult,
    JsonMainMotionTraceArm,
    JsonMainMotionTraceAssembly,
    JsonMainMotionTracePageResult,
    JsonMainPositionResult,
    JsonMainRenewLiveMotionResult,
    JsonMainToolJogResult,
    JsonMainStopResult,
    JsonResponseDelivery,
    JsonSessionAdmissionError,
    JsonSessionProtocolError,
    JsonSessionTimeoutError,
    ProtocolFailure,
    Request,
    Response,
    encode_message,
    write_main_motion_trace_artifact,
)
class FakeClock:
    def __init__(self, value=10.0):
        self.value = value

    def __call__(self):
        return self.value


class ManualDeadlineScheduler:
    def __init__(self):
        self.registrations = []

    def __call__(self, deadline, callback, publish_owner):
        registration = {
            "active": True,
            "callback": callback,
            "deadline": deadline,
        }

        def cancel():
            registration["active"] = False

        self.registrations.append(registration)
        publish_owner(registration, cancel)


class FakeSerial:
    def __init__(self, reads=()):
        self.is_open = True
        self.timeout = 3.0
        self.write_timeout = 4.0
        self.out_waiting = 0
        self.reads = list(reads)
        self.writes = []
        self.close_calls = 0

    @property
    def in_waiting(self):
        if not self.reads:
            return 0
        return len(self.reads[0])

    def read(self, size):
        if not self.reads:
            return b""
        chunk = self.reads.pop(0)
        if len(chunk) <= size:
            return chunk
        self.reads.insert(0, chunk[size:])
        return chunk[:size]

    def write(self, frame):
        self.writes.append(frame)
        return len(frame)

    def close(self):
        self.close_calls += 1
        self.is_open = False


def sample_main_hello_result():
    return {
        "capabilities": [
            "JSON_PROTOCOL_V1",
            "REQUEST_CORRELATION_V1",
            "EVENT_STREAM_V1",
        ],
        "device": "main_controller",
        "firmware": {
            "build": "tracked",
            "name": "AR4 Teensy",
            "version": "6.7.1-ar4hmi.39",
        },
        "identity": {
            "asset_tag": "Lab",
            "controller_hardware_id": "1705B6",
            "driver_model": "Teensy 4.1",
            "robot_model": "AR4",
            "robot_version": "MK3",
            "serial_number": "SN42",
        },
        "commands": list(protocol.JSON_MAIN_COMMAND_MANIFEST),
        "protocol": {
            "max_payload_bytes": 4094,
            "name": "ar4_json",
            "version": 1,
        },
        "session_id": "00112233445566778899AABBCCDDEEFF",
    }


def sample_main_motion_position_result():
    return {
        "axis_source": "controller_step_state",
        "cartesian_micrometers": [123456, -789000, 0],
        "external_axes_milliunits": [1250, 0, -500],
        "orientation_millidegrees": [90000, -45000, 180000],
        "robot_joints_millidegrees": [
            0,
            -1000,
            170000,
            -170000,
            -2147483648,
            2147483647,
        ],
    }


def sample_main_position_disposition_result():
    result = sample_main_motion_position_result()
    result.update({
        "controller_debug": "",
        "motion_fault": "",
        "speed_limited": False,
    })
    return result


def sample_main_home_reference_result():
    return {
        "positions_millidegrees": [170000, 0, -88000],
        "valid": [True, False, True],
    }


def sample_main_move_joints_params():
    return {
        "acceleration_percent": 10.0,
        "deceleration_percent": 20.0,
        "external_axes_units": (1.25, 0.0, -0.5),
        "loop_modes": (False, True, False, True, False, True),
        "ramp_percent": 25.0,
        "robot_joints_degrees": (0.0, -1.0, 2.0, -3.0, 4.0, -5.0),
        "speed_mode": "percent",
        "speed_value": 50.0,
        "telemetry_enabled": True,
        "trace_configuration_fingerprint": None,
        "wrist_configuration": "near",
    }


def sample_main_move_joints_result():
    return {
        "controller_debug": "",
        "position": sample_main_motion_position_result(),
        "speed_limited": False,
    }


def sample_main_calibration_params():
    return {
        "axes": (True, False, True, False, False, False, False, False, False),
        "offsets": (0.0,) * 9,
    }


def sample_main_calibration_result():
    return sample_main_move_joints_result()


def sample_main_move_cartesian_params():
    return {
        "acceleration_percent": 10.0,
        "deceleration_percent": 20.0,
        "external_axes_units": (1.25, 0.0, -0.5),
        "loop_modes": (False, True, False, True, False, True),
        "orientation_degrees": (10.0, -20.0, 30.0),
        "ramp_percent": 25.0,
        "speed_mode": "millimeters_per_second",
        "speed_value": 50.0,
        "telemetry_enabled": True,
        "translation_millimeters": (100.0, -200.0, 300.0),
        "wrist_configuration": "automatic",
    }


def sample_main_move_cartesian_result():
    return {
        "controller_debug": "",
        "position": sample_main_motion_position_result(),
        "speed_limited": False,
    }


def sample_main_tool_jog_params():
    return {
        "acceleration_percent": 10.0,
        "axis": "rx",
        "deceleration_percent": 20.0,
        "direction": "negative",
        "distance": 1.25,
        "loop_modes": (False, True, False, True, False, True),
        "ramp_percent": 25.0,
        "speed_mode": "percent",
        "speed_value": 50.0,
        "wrist_configuration": "far",
    }


def sample_main_tool_jog_result():
    return {
        "controller_debug": "",
        "position": sample_main_motion_position_result(),
        "speed_limited": False,
    }


def sample_main_live_jog_params(*, axis=2):
    return {
        "acceleration_percent": 10.0,
        "axis": axis,
        "deceleration_percent": 20.0,
        "direction": "positive",
        "lease_milliseconds": (
            JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS
        ),
        "loop_modes": (False, True, False, True, False, True),
        "ramp_percent": 25.0,
        "speed_mode": "percent",
        "speed_value": 50.0,
        "telemetry_enabled": True,
        "wrist_configuration": "near",
    }


def sample_main_live_jog_result():
    return {
        "controller_debug": "",
        "position": sample_main_motion_position_result(),
        "speed_limited": False,
    }


def sample_main_update_params():
    return {
        "calibration_directions": (1, 0, 1, 0, 0, 1, 0, 0, 0),
        "calibration_switch_active_high": (True,) * 9,
        "dh_a_millimeters": (0, 64.2, 305, 0, 0, 0),
        "dh_alpha_degrees": (0, -90, 0, -90, 90, -90),
        "dh_d_millimeters": (169.77, 0, 0, 222.63, 0, 41),
        "dh_theta_degrees": (0, -90, 0, 0, 0, 180),
        "encoder_counts_per_step": (5, 5, 5, 5, 2.5, 5),
        "motor_directions": (0, 1, 1, 1, 1, 1, 1, 1, 1),
        "negative_joint_limits_degrees": (170, 42, 89, 180, 105, 180),
        "positive_joint_limits_degrees": (170, 90, 52, 180, 105, 180),
        "steps_per_degree": (
            88.888,
            111.111,
            111.111,
            99.555,
            43.72,
            44.444,
        ),
        "tool_rotation_degrees": (0, 0, 0),
        "tool_translation_millimeters": (0, 0, 0),
    }


def sample_main_startup_configuration():
    return JsonMainControllerStartupConfiguration(
        **sample_main_update_params(),
        external_axis_travel_units=(3450, 3450, 3450),
        external_axis_drive_rotations=(280, 280, 280),
        external_axis_motor_steps=(4000, 4000, 4000),
        robot_joints_millidegrees=(0,) * 6,
        external_axes_milliunits=(0,) * 3,
    )


def sample_main_motion_trace_result(
    *, motion_request_id, capture_generation=1, page_index=0, total_records=1
):
    record_start = page_index * protocol.JSON_MOTION_TRACE_PAGE_RECORDS
    page_records = min(
        protocol.JSON_MOTION_TRACE_PAGE_RECORDS,
        total_records - record_start,
    )
    return {
        "capture_generation": capture_generation,
        "capture_state": "available",
        "configuration_fingerprint": "sha256:" + "a" * 64,
        "disposition": {
            "capacity_limited": False,
            "clock_wrapped": False,
            "complete": True,
            "motion_outcome": "completed",
            "timing_overrun": False,
        },
        "firmware": sample_main_hello_result()["firmware"],
        "page_count": (
            total_records + protocol.JSON_MOTION_TRACE_PAGE_RECORDS - 1
        ) // protocol.JSON_MOTION_TRACE_PAGE_RECORDS,
        "page_index": page_index,
        "record_start": record_start,
        "records": [{
            "commanded_steps": [0, 1, 2, 3, 4, 5],
            "controller_microseconds": 100 + master_index,
            "encoder_counts": [5, 4, 3, 2, 1, 0],
            "flags": 0,
            "master_index": master_index,
            "phase": 0,
            "scheduled_delay_microseconds": 200,
        } for master_index in range(record_start, record_start + page_records)],
        "source_motion_request_id": motion_request_id,
        "source_session_id": sample_main_hello_result()["session_id"],
        "total_records": total_records,
    }


class JsonMainControllerClientTests(unittest.TestCase):
    def make_client(self, reads=(), **client_options):
        serial_port = FakeSerial(reads)
        clock = FakeClock()
        scheduler = ManualDeadlineScheduler()
        client = JsonMainControllerClient(
            serial_port,
            clock=clock,
            clock_resolution=0.0,
            deadline_scheduler=scheduler,
            **client_options,
        )
        return client, serial_port, clock, scheduler

    def make_controller_wait_client(self, *responses):
        hello = sample_main_hello_result()
        responses = (Response(1, "hello", "completed", hello), *responses)
        return self.make_client(tuple(map(encode_message, responses)))

    def poll_until_delivery(self, client):
        for _attempt in range(32):
            if client.poll():
                return
        self.fail("bounded polling did not produce a complete delivery")

    def establish_session(self, client):
        ticket = client.request_hello(timeout=2.0)
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        self.assertIsInstance(delivery, JsonResponseDelivery)
        self.assertIs(delivery.ticket, ticket)
        terminal = client.take_terminal(ticket)
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.session_ready)
        return terminal

    def test_configuration_fingerprint_is_deterministic_and_scoped(self):
        configuration = sample_main_startup_configuration()
        fingerprint = configuration.configuration_fingerprint

        self.assertEqual(
            fingerprint,
            sample_main_startup_configuration().configuration_fingerprint,
        )
        self.assertRegex(fingerprint, r"\Asha256:[0-9a-f]{64}\Z")
        self.assertNotEqual(
            fingerprint,
            replace(
                configuration,
                motor_directions=(1,) * 9,
            ).configuration_fingerprint,
        )
        self.assertNotEqual(
            fingerprint,
            replace(
                configuration,
                external_axis_motor_steps=(8000, 4000, 4000),
            ).configuration_fingerprint,
        )
        self.assertEqual(
            fingerprint,
            replace(
                configuration,
                robot_joints_millidegrees=(1000,) * 6,
            ).configuration_fingerprint,
        )

    def test_motion_trace_arm_consumes_only_current_reservation(self):
        arm = JsonMainMotionTraceArm()
        fingerprint = (
            sample_main_startup_configuration().configuration_fingerprint
        )

        self.assertIsNone(arm.reserve(fingerprint))
        first_generation = arm.arm()
        reservation = arm.reserve(fingerprint)
        self.assertEqual(reservation.generation, first_generation)
        self.assertEqual(reservation.configuration_fingerprint, fingerprint)
        self.assertTrue(arm.armed)
        self.assertTrue(arm.consume(reservation))
        self.assertFalse(arm.armed)
        self.assertFalse(arm.consume(reservation))

        arm.arm()
        stale_reservation = arm.reserve(fingerprint)
        arm.arm()
        self.assertFalse(arm.consume(stale_reservation))
        self.assertTrue(arm.armed)

    def test_client_starts_idle_without_public_submit(self):
        client, serial_port, _clock, _scheduler = self.make_client()

        self.assertFalse(hasattr(client, "submit"))
        self.assertEqual(serial_port.writes, [])
        self.assertEqual(client.pending_tickets, ())
        self.assertFalse(client.session_ready)
        self.assertIsNone(client.session_binding)
        self.assertFalse(client.quarantined)
        self.assertEqual(client.delivery_count, 0)
        self.assertEqual(client.deadline_cleanup_count, 0)
        with self.assertRaises(JsonSessionAdmissionError):
            client.retry_deadline_cleanup()

    def test_controller_wait_request_ownership_and_terminal_readiness(self):
        responses = (
            Response(2, "get_position_disposition", "completed", sample_main_position_disposition_result()),
            Response(3, "controller_wait", "completed", {}),
            Response(4, "controller_wait", "rejected", error=ProtocolFailure(
                "unsupported_command", "unsupported")),
            Response(5, "controller_wait", "cancelled", error=ProtocolFailure(
                "emergency_stop", "wait interrupted")),
        )
        client, serial_port, *_ = self.make_controller_wait_client(*responses)
        self.establish_session(client)
        position = client.request_position_disposition(timeout=2.0)
        with self.assertRaises(JsonMainControllerClientStateError):
            client.request_controller_wait(1.0, timeout=2.0)
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(position)

        wait = client.request_controller_wait(1.0, timeout=2.0)
        self.assertEqual(serial_port.writes[-1], encode_message(Request(
            3, "controller_wait", {"seconds": 1.0})))
        with self.assertRaises(JsonMainControllerClientStateError):
            client.request_position_disposition(timeout=2.0)
        self.poll_until_delivery(client)
        client.pop_delivery()
        with self.assertRaises(JsonMainControllerClientStateError):
            client.request_position_disposition(timeout=2.0)
        self.assertIsNone(client.take_terminal(wait).parsed_result)
        client.acknowledge_terminal(wait)
        for response in responses[-2:]:
            ticket = client.request_controller_wait(1.0, timeout=2.0)
            self.poll_until_delivery(client)
            client.pop_delivery()
            self.assertEqual(client.take_terminal(ticket).response, response)
            client.acknowledge_terminal(ticket)
            self.assertTrue(client.session_ready)

    def test_modbus_read_requests_are_correlated_and_blocking(self):
        cases = (
            (protocol.MAIN_MODBUS_READ_HOLDING_REGISTER_COMMAND_CONTRACT, 65535),
            (protocol.MAIN_MODBUS_READ_COIL_COMMAND_CONTRACT, 1),
            (protocol.MAIN_MODBUS_READ_DISCRETE_INPUT_COMMAND_CONTRACT, 0),
            (protocol.MAIN_MODBUS_READ_INPUT_REGISTER_COMMAND_CONTRACT, 42),
        )
        self.assertEqual(protocol.parse_main_modbus_read_result(
            {"value": 1}, command=cases[1][0].name), 1)

        def request_read(client, command):
            return client.request_modbus_read(
                command, slave_id=7, address=123, timeout=2.0)

        hello = sample_main_hello_result()
        responses = [
            Response(1, "hello", "completed", hello),
            Response(2, "get_position_disposition", "completed", sample_main_position_disposition_result()),
        ]
        responses.extend(
            Response(request_id, contract.name, "completed", {"value": value})
            for request_id, (contract, value) in enumerate(cases, start=3)
        )
        responses.append(Response(
            7, "modbus_read_coil", "failed",
            error=ProtocolFailure("modbus_error", "bus read failed")))
        client, serial_port, *_ = self.make_client(
            tuple(map(encode_message, responses))
        )
        self.establish_session(client)
        position = client.request_position_disposition(timeout=2.0)
        with self.assertRaises(JsonMainControllerClientStateError):
            request_read(client, cases[0][0].name)
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(position)

        for request_id, (contract, value) in enumerate(cases, start=3):
            ticket = request_read(client, contract.name)
            self.assertEqual(serial_port.writes[-1], encode_message(Request(
                request_id,
                contract.name,
                {"slave_id": 7, "address": 123, "count": 1},
            )))
            with self.assertRaises(JsonMainControllerClientStateError):
                client.request_position_disposition(timeout=2.0)
            self.poll_until_delivery(client)
            client.pop_delivery()
            terminal = client.take_terminal(ticket)
            self.assertEqual(terminal.parsed_result, value)
            with self.assertRaises(JsonMainControllerClientStateError):
                client.request_position_disposition(timeout=2.0)
            client.acknowledge_terminal(ticket)

        ticket = request_read(client, "modbus_read_coil")
        self.poll_until_delivery(client)
        client.pop_delivery()
        self.assertEqual(client.take_terminal(ticket).response.status, "failed")
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.session_ready)

        writes = tuple(serial_port.writes)
        for invalid_command in ("modbus_read_unknown", [], {}):
            with self.assertRaises(JsonCommandSchemaError):
                request_read(client, invalid_command)
        self.assertEqual(tuple(serial_port.writes), writes)
        self.assertTrue(client.session_ready)

    def test_modbus_write_requests_are_correlated_and_fail_closed(self):
        coil = protocol.MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT.name
        reg = protocol.MAIN_MODBUS_WRITE_REGISTER_COMMAND_CONTRACT.name
        params, admission_calls = {"slave_id": 7, "address": 123, "timeout": 2.0}, []
        error, make_client = JsonMainControllerClientStateError, self.make_controller_wait_client
        def submit(client, serial_port, response, value):
            ticket = client.request_modbus_write(
                response.cmd, value=value, **params,
                write_admission=lambda: admission_calls.append(response.cmd) or True)
            expected = Request(response.id, response.cmd, {
                "slave_id": 7, "address": 123, "value": value})
            self.assertEqual(serial_port.writes[-1], encode_message(expected))
            self.poll_until_delivery(client)
            client.pop_delivery()
            self.assertEqual(client.take_terminal(ticket).response, response)
            return ticket

        responses = (
            Response(2, "get_position_disposition", "completed", sample_main_position_disposition_result()),
            Response(3, reg, "rejected", error=ProtocolFailure(
                "unsupported_command", "unsupported")),
            Response(4, coil, "cancelled", error=ProtocolFailure(
                "emergency_stop", "cancelled")),
            Response(5, coil, "completed", {}),
        )
        client, serial_port, *_ = make_client(*responses)
        self.establish_session(client)
        write = client.request_modbus_write
        self.assertRaises(JsonSessionAdmissionError, write, coil, value=1, **params,
                          write_admission=lambda: 1)
        position = client.request_position_disposition(timeout=2.0)
        self.assertRaises(error, write, reg, value=65535, **params)
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(position)
        for response, value in zip(responses[1:], (65535, 1, 1)):
            ticket = submit(client, serial_port, response, value)
            client.acknowledge_terminal(ticket)
        self.assertRaises(JsonCommandSchemaError, write, "", value=0, **params)
        self.assertEqual((client.pending_tickets, len(serial_port.writes)), ((), 5))
        response = Response(2, coil, "failed", error=ProtocolFailure("modbus_error", "failed"))
        client, serial_port, *_ = make_client(response)
        self.establish_session(client)
        ticket = submit(client, serial_port, response, 1)
        self.assertRaises(error, client.acknowledge_terminal, ticket)
        self.assertEqual(client.take_terminal(ticket).response, response)
        self.assertRaises(error, client.request_position_disposition, timeout=2.0)
        self.assertEqual((client.pending_tickets, len(serial_port.writes)), ((ticket,), 2))
        self.assertEqual(admission_calls, [reg, coil, coil, coil])

    def test_generic_playback_suspends_deadline_and_returns_cartesian_result(self):
        params = {"filename": "demo.gcode", "media_id": "0" * 32}
        responses = (
            Response(1, "hello", "completed", sample_main_hello_result()),
            Response(2, "play_gcode_file", "accepted", {}),
            Response(2, "play_gcode_file", "completed", {
                **sample_main_move_cartesian_result(),
                "speed_limited": True,
            }),
        )
        client, serial_port, clock, _scheduler = self.make_client(tuple(map(encode_message, responses)))
        self.establish_session(client)
        ticket = client.request_command("play_gcode_file", params, timeout=2.0)
        self.assertEqual(serial_port.writes[-1], encode_message(Request(2, "play_gcode_file", params)))
        self.poll_until_delivery(client)
        self.assertEqual(client.pop_delivery().response, responses[1])
        clock.value = ticket.deadline
        client.expire()
        self.poll_until_delivery(client)
        terminal = client.take_terminal(ticket)
        self.assertIsInstance(
            terminal.parsed_result, JsonMainCartesianMotionResult)
        self.assertTrue(terminal.parsed_result.speed_limited)
        self.assertEqual(terminal.response, responses[2])
        client.pop_delivery()
        client.acknowledge_terminal(ticket)

    def test_write_admission_rejects_recursive_client_request(self):
        client, serial_port, _clock, _scheduler = self.make_client()
        failures = []
        nested_failures = []
        nested_finished = threading.Event()

        def request_from_callback_thread():
            try:
                client.request_hello(timeout=2.0)
            except BaseException as exc:
                nested_failures.append(exc)
            finally:
                nested_finished.set()

        def request_recursively():
            nested_worker = threading.Thread(
                target=request_from_callback_thread,
                daemon=True,
            )
            nested_worker.start()
            if not nested_finished.wait(timeout=0.5):
                raise RuntimeError(
                    "recursive client request blocked on the state lock"
                )
            nested_worker.join(timeout=0.5)
            if nested_worker.is_alive() or len(nested_failures) != 1:
                raise RuntimeError(
                    "recursive client request did not reject cleanly"
                )
            raise nested_failures[0]

        def request_outer():
            try:
                client.request_hello(
                    timeout=2.0,
                    write_admission=request_recursively,
                )
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=request_outer, daemon=True)
        worker.start()
        worker.join(timeout=0.5)

        self.assertFalse(worker.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(
            failures[0],
            JsonMainControllerClientStateError,
        )
        self.assertIn("submission is active", str(failures[0]))
        self.assertEqual(serial_port.writes, [])
        self.assertEqual(client.pending_tickets, ())
        self.assertFalse(client.quarantined)

    def test_hello_acknowledgement_rejects_until_state_publication(self):
        client, serial_port, _clock, _scheduler = self.make_client(
            (
                encode_message(
                    Response(
                        1,
                        "hello",
                        "completed",
                        sample_main_hello_result(),
                    )
                ),
            )
        )
        coordinator_submit = client._coordinator.submit
        ticket_ready = threading.Event()
        release_return = threading.Event()
        retained_tickets = []
        results = []
        failures = []

        def hold_ticket_return(*args, **kwargs):
            ticket = coordinator_submit(*args, **kwargs)
            retained_tickets.append(ticket)
            ticket_ready.set()
            if not release_return.wait(timeout=1.0):
                raise TimeoutError("test did not release hello ticket return")
            return ticket

        client._coordinator.submit = hold_ticket_return

        def request_hello():
            try:
                results.append(client.request_hello(timeout=2.0))
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=request_hello, daemon=True)
        worker.start()
        try:
            self.assertTrue(ticket_ready.wait(timeout=1.0))
            ticket = retained_tickets[0]
            self.poll_until_delivery(client)
            delivery = client.pop_delivery()
            self.assertIs(delivery.ticket, ticket)
            with self.assertRaisesRegex(
                JsonMainControllerClientStateError,
                "terminal acknowledgement rejected while request submission "
                "is active",
            ):
                client.acknowledge_terminal(ticket)
        finally:
            release_return.set()
        worker.join(timeout=1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(results, retained_tickets)
        client.acknowledge_terminal(retained_tickets[0])
        self.assertTrue(client.session_ready)

    def test_semantic_requests_emit_exact_empty_parameter_frames(self):
        hello_response = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, serial_port, _clock, scheduler = self.make_client(
            (encode_message(hello_response),)
        )

        hello_ticket = client.request_hello(timeout=2.0)
        with self.assertRaises(JsonMainControllerClientStateError):
            client.request_position_disposition(timeout=3.0)
        self.poll_until_delivery(client)
        self.assertFalse(client.session_ready)
        client.acknowledge_terminal(hello_ticket)
        position_ticket = client.request_position_disposition(timeout=3.0)

        self.assertEqual(
            serial_port.writes,
            [
                encode_message(Request(1, "hello", {})),
                encode_message(Request(2, "get_position_disposition", {})),
            ],
        )
        self.assertEqual(client.pending_tickets, (position_ticket,))
        self.assertEqual(hello_ticket.deadline, 12.0)
        self.assertEqual(position_ticket.deadline, 13.0)
        self.assertEqual(
            [item["deadline"] for item in scheduler.registrations],
            [12.0, 13.0],
        )

    def test_completed_position_terminal_is_repeatable_and_typed(self):
        response = Response(
            2,
            "get_position_disposition",
            "completed",
            sample_main_position_disposition_result(),
        )
        hello_response = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (encode_message(hello_response), encode_message(response))
        )
        self.establish_session(client)
        ticket = client.request_position_disposition(timeout=2.0)

        self.poll_until_delivery(client)
        self.assertEqual(client.delivery_count, 1)
        delivery = client.pop_delivery()
        self.assertIsInstance(delivery, JsonResponseDelivery)
        self.assertIs(delivery.ticket, ticket)
        self.assertEqual(delivery.response, response)
        self.assertEqual(client.delivery_count, 0)
        self.assertIsNone(client.pop_delivery())

        snapshot = client.snapshot(ticket)
        self.assertIs(snapshot.ticket, ticket)
        self.assertIsNone(snapshot.accepted)
        self.assertEqual(snapshot.terminal, response)
        first = client.take_terminal(ticket)
        second = client.take_terminal(ticket)
        self.assertIsInstance(first, JsonMainControllerTerminal)
        self.assertEqual(first, second)
        self.assertIsInstance(first.parsed_result, JsonMainPositionResult)
        self.assertEqual(
            first.parsed_result.cartesian_translation_millimeters,
            (123.456, -789.0, 0.0),
        )
        self.assertIsNone(first.failure)
        self.assertEqual(client.pending_tickets, (ticket,))

        binding = client.session_binding
        client.acknowledge_terminal(ticket)
        self.assertEqual(client.pending_tickets, ())
        self.assertTrue(client.session_ready)
        self.assertEqual(client.session_binding, binding)

    def test_position_disposition_is_typed(self):
        hello_result = sample_main_hello_result()
        response = Response(
            2,
            "get_position_disposition",
            "completed",
            sample_main_position_disposition_result(),
        )
        client, serial_port, _clock, _scheduler = self.make_client((
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(response),
        ))
        self.establish_session(client)

        ticket = client.request_position_disposition(timeout=2.0)
        self.poll_until_delivery(client)
        terminal = client.take_terminal(ticket)

        self.assertIsInstance(terminal.parsed_result, JsonMainPositionResult)
        self.assertFalse(terminal.parsed_result.speed_limited)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "get_position_disposition", {})),
        )

    def test_external_axis_zero_requests_are_typed_and_preserve_sync(self):
        rejected = ProtocolFailure("unsupported_command", "unsupported")
        failed = ProtocolFailure("position_unavailable", "unavailable")
        hello = sample_main_hello_result()
        hello_frame = encode_message(Response(1, "hello", "completed", hello))
        cases = (
            (7, "zero_j7", "completed", None), (8, "zero_j8", "completed", None),
            (9, "zero_j9", "completed", None), (7, "zero_j7", "rejected", rejected),
            (7, "zero_j7", "failed", failed),
        )
        for axis, command, status, failure in cases:
            result = sample_main_position_disposition_result() if failure is None else None
            response = Response(2, command, status, result, failure)
            client, serial_port, _clock, _scheduler = self.make_client((hello_frame, encode_message(response)))
            self.establish_session(client)
            ticket = client.request_zero_external_axis(axis, timeout=2.0)
            self.assertEqual(serial_port.writes[-1], encode_message(Request(2, command, {})))
            self.poll_until_delivery(client)
            terminal = client.take_terminal(ticket)
            self.assertIs(type(terminal.parsed_result), JsonMainPositionResult if failure is None else type(None))
            client.acknowledge_terminal(ticket)
            self.assertFalse(client.configuration_sync_required)

        for axis in (True, 6, "7"):
            with self.assertRaises(JsonCommandSchemaError):
                client.request_zero_external_axis(axis, timeout=2.0)

    def test_completed_home_reference_terminal_is_typed(self):
        hello_result = sample_main_hello_result()
        hello_response = Response(
            1,
            "hello",
            "completed",
            hello_result,
        )
        home_response = Response(
            2,
            "get_home_reference",
            "completed",
            sample_main_home_reference_result(),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            (encode_message(hello_response), encode_message(home_response))
        )
        self.establish_session(client)

        ticket = client.request_home_reference(timeout=2.0)
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        self.assertIs(delivery.ticket, ticket)
        terminal = client.take_terminal(ticket)

        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "get_home_reference", {})),
        )
        self.assertIsInstance(
            terminal.parsed_result,
            JsonMainHomeReferenceResult,
        )
        self.assertEqual(
            terminal.parsed_result.positions_degrees,
            (170.0, 0.0, -88.0),
        )
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.session_ready)

    def test_diagnostics_emit_correlated_typed_results(self):
        hello = sample_main_hello_result()
        responses = (
            Response(1, "hello", "completed", hello),
            Response(
                2, "test_limit_switches", "completed",
                {"active": [True, False, True, False, True, False]},
            ),
            Response(3, "set_encoders", "completed", {}),
            Response(
                4, "read_encoders", "completed",
                {"counts": [-1, 0, 1, 1000, -2147483648, 2147483647]},
            ),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            tuple(encode_message(response) for response in responses)
        )
        self.establish_session(client)
        requests = (
            (client.request_test_limit_switches, (True, False) * 3),
            (client.request_set_encoders, None),
            (
                client.request_read_encoders,
                (-1, 0, 1, 1000, -2147483648, 2147483647),
            ),
        )
        for request, expected in requests:
            ticket = request(timeout=2.0)
            self.poll_until_delivery(client)
            self.assertIs(client.pop_delivery().ticket, ticket)
            self.assertEqual(client.take_terminal(ticket).parsed_result, expected)
            client.acknowledge_terminal(ticket)
        self.assertEqual(
            [frame for frame in serial_port.writes[1:]],
            [
                encode_message(Request(2, "test_limit_switches", {})),
                encode_message(Request(3, "set_encoders", {})),
                encode_message(Request(4, "read_encoders", {})),
            ],
        )

    def test_set_position_emits_fixed_point_request_and_typed_terminal(self):
        hello_result = sample_main_hello_result()
        hello_response = Response(1, "hello", "completed", hello_result)
        set_response = Response(2, "set_position", "completed", {})
        client, serial_port, _clock, _scheduler = self.make_client(
            (encode_message(hello_response), encode_message(set_response))
        )
        self.establish_session(client)

        ticket = client.request_set_position(
            robot_joints_millidegrees=(0, -1000, 2000, 0, 45000, 0),
            external_axes_milliunits=(1250, 0, -500),
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        terminal = client.take_terminal(ticket)

        self.assertIs(delivery.ticket, ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(
                Request(
                    2,
                    "set_position",
                    {
                        "external_axes_milliunits": (1250, 0, -500),
                        "robot_joints_millidegrees": (
                            0,
                            -1000,
                            2000,
                            0,
                            45000,
                            0,
                        ),
                    },
                )
            ),
        )
        self.assertIsNone(terminal.parsed_result)
        self.assertIsNone(terminal.failure)
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.session_ready)

    def test_set_position_requires_valid_fixed_point_values(self):
        hello_response = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            (encode_message(hello_response),)
        )
        self.establish_session(client)
        writes_before_request = tuple(serial_port.writes)
        with self.assertRaises(JsonSessionAdmissionError):
            client.request_set_position(
                robot_joints_millidegrees=(0, 0, 0, 0, 0, 0.5),
                external_axes_milliunits=(0,) * 3,
                timeout=2.0,
            )
        self.assertEqual(tuple(serial_port.writes), writes_before_request)
        self.assertEqual(client.pending_tickets, ())

    def test_position_correction_returns_position(self):
        hello = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello)),
            encode_message(Response(
                2,
                "correct_position",
                "completed",
                sample_main_position_disposition_result(),
            )),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)

        ticket = client.request_correct_position(timeout=2.0)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "correct_position", {})),
        )
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        terminal = client.take_terminal(ticket)
        self.assertIs(delivery.ticket, ticket)
        self.assertIsInstance(terminal.parsed_result, JsonMainPositionResult)
        client.acknowledge_terminal(ticket)
        self.assertEqual(client.pending_tickets, ())
        self.assertFalse(client.configuration_sync_required)

    def test_position_correction_invalid_result_names_correction_contract(self):
        invalid = sample_main_position_disposition_result()
        invalid["unexpected"] = False

        with self.assertRaisesRegex(
            JsonCommandSchemaError,
            "position-correction result",
        ):
            MAIN_CORRECT_POSITION_COMMAND_CONTRACT.response_validator(
                Response(2, "correct_position", "completed", invalid)
            )

    def test_calibration_returns_typed_terminal(self):
        params = sample_main_calibration_params()
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "calibrate", "accepted", {})),
            encode_message(
                Response(
                    2,
                    "calibrate",
                    "completed",
                    sample_main_calibration_result(),
                )
            ),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)

        ticket = client.request_calibrate(**params, timeout=2.0)
        self.assertIs(client.pending_motion_ticket, ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "calibrate", params)),
        )
        self.poll_until_delivery(client)
        accepted = client.pop_delivery()
        self.assertIs(accepted.ticket, ticket)
        self.assertEqual(accepted.response.status, "accepted")
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        terminal = client.take_terminal(ticket)
        self.assertIs(delivery.ticket, ticket)
        self.assertIsInstance(terminal.parsed_result, JsonMainCalibrationResult)
        self.assertEqual(
            terminal.parsed_result.position.robot_joints_degrees,
            (0.0, -1.0, 170.0, -170.0, -2147483.648, 2147483.647),
        )
        client.acknowledge_terminal(ticket)
        self.assertIsNone(client.pending_motion_ticket)
        self.assertTrue(client.session_ready)

    def test_calibration_requires_drained_requests_and_deliveries(self):
        params = sample_main_calibration_params()
        hello_result = sample_main_hello_result()
        hello_response = encode_message(
            Response(1, "hello", "completed", hello_result)
        )

        pending_client, pending_serial, _clock, _scheduler = self.make_client(
            (hello_response,)
        )
        self.establish_session(pending_client)
        pending_client.request_position_disposition(timeout=2.0)
        pending_writes = tuple(pending_serial.writes)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "calibrate requires drained requests and deliveries",
        ):
            pending_client.request_calibrate(**params, timeout=2.0)
        self.assertEqual(tuple(pending_serial.writes), pending_writes)

        delivered_client, delivered_serial, _clock, _scheduler = (
            self.make_client(
                (
                    hello_response,
                    encode_message(
                        Response(
                            2,
                            "get_position_disposition",
                            "completed",
                            sample_main_position_disposition_result(),
                        )
                    ),
                )
            )
        )
        self.establish_session(delivered_client)
        delivered_client.request_position_disposition(timeout=2.0)
        self.poll_until_delivery(delivered_client)
        delivered_writes = tuple(delivered_serial.writes)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "calibrate requires drained requests and deliveries",
        ):
            delivered_client.request_calibrate(**params, timeout=2.0)
        self.assertEqual(tuple(delivered_serial.writes), delivered_writes)

    def test_move_joints_emits_exact_request_and_typed_terminal(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(
                Response(
                    2,
                    "move_joints",
                    "completed",
                    sample_main_move_joints_result(),
                )
            ),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        params = sample_main_move_joints_params()

        ticket = client.request_move_joints(**params, timeout=2.0)
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        terminal = client.take_terminal(ticket)

        self.assertIs(delivery.ticket, ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "move_joints", params)),
        )
        self.assertIsInstance(
            terminal.parsed_result,
            JsonMainJointMotionResult,
        )
        self.assertEqual(
            terminal.parsed_result.position.robot_joints_degrees,
            (0.0, -1.0, 170.0, -170.0, -2147483.648, 2147483.647),
        )
        self.assertFalse(terminal.parsed_result.speed_limited)
        self.assertEqual(terminal.parsed_result.controller_debug, "")
        client.acknowledge_terminal(ticket)
        self.assertIsNone(client.pending_joint_motion_ticket)
        self.assertTrue(client.session_ready)

    def test_traced_move_joints_requires_disabled_telemetry(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(
                Response(
                    2,
                    "move_joints",
                    "completed",
                    sample_main_move_joints_result(),
                )
            ),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        params = sample_main_move_joints_params()
        params.update({
            "telemetry_enabled": False,
            "trace_configuration_fingerprint": (
                sample_main_startup_configuration().configuration_fingerprint
            ),
        })

        ticket = client.request_move_joints(**params, timeout=2.0)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "move_joints", params)),
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(ticket)

        writes_before_rejection = tuple(serial_port.writes)
        params["telemetry_enabled"] = True
        with self.assertRaises(JsonSessionAdmissionError):
            client.request_move_joints(**params, timeout=2.0)
        self.assertEqual(tuple(serial_port.writes), writes_before_rejection)

    def test_motion_trace_page_is_correlated_and_typed(self):
        hello_result = sample_main_hello_result()
        source_motion_request_id = 47
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(
                Response(
                    2,
                    "get_motion_trace",
                    "completed",
                    sample_main_motion_trace_result(
                        motion_request_id=source_motion_request_id
                    ),
                )
            ),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)

        ticket = client.request_motion_trace(
            motion_request_id=source_motion_request_id,
            page_index=0,
            timeout=2.0,
        )
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "get_motion_trace", {
                "motion_request_id": source_motion_request_id,
                "page_index": 0,
            })),
        )
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        terminal = client.take_terminal(ticket)

        self.assertIs(delivery.ticket, ticket)
        self.assertIsInstance(
            terminal.parsed_result,
            JsonMainMotionTracePageResult,
        )
        self.assertEqual(
            terminal.parsed_result.source_motion_request_id,
            source_motion_request_id,
        )
        self.assertEqual(
            terminal.parsed_result.records[0].commanded_steps,
            (0, 1, 2, 3, 4, 5),
        )
        no_capture = protocol.parse_main_motion_trace_result({
            "capture_state": "no_capture",
            "source_motion_request_id": source_motion_request_id,
        })
        self.assertIsInstance(
            no_capture,
            protocol.JsonMainMotionTraceNoCaptureResult,
        )
        self.assertEqual(
            no_capture.source_motion_request_id,
            source_motion_request_id,
        )

    def test_motion_trace_retrieval_requires_settled_position_without_stop(self):
        completed = JsonMainControllerTerminal(
            Response(
                1,
                "move_joints",
                "completed",
                sample_main_move_joints_result(),
            )
        )
        failed_with_position = JsonMainControllerTerminal(
            Response(
                2,
                "move_joints",
                "failed",
                error=ProtocolFailure(
                    "motion_execution_failed",
                    "motion failed",
                    {"position": sample_main_motion_position_result()},
                ),
            )
        )
        failed_without_position = JsonMainControllerTerminal(
            Response(
                3,
                "move_joints",
                "failed",
                error=ProtocolFailure(
                    "position_unavailable",
                    "position unavailable",
                ),
            )
        )
        cases = (
            (completed, False, True),
            (failed_with_position, False, True),
            (failed_without_position, False, False),
            (completed, True, False),
        )
        for terminal, stop_observed, expected in cases:
            with self.subTest(
                status=terminal.response.status,
                stop_observed=stop_observed,
            ):
                self.assertIs(
                    protocol.main_motion_trace_retrieval_eligible(
                        terminal,
                        stop_observed,
                    ),
                    expected,
                )

    def test_complete_motion_trace_assembly_writes_one_artifact(self):
        configuration = sample_main_startup_configuration()
        result = sample_main_motion_trace_result(
            motion_request_id=47,
            total_records=2,
        )
        result["configuration_fingerprint"] = (
            configuration.configuration_fingerprint
        )
        result["records"][1]["encoder_counts"][0] = -10
        assembly = JsonMainMotionTraceAssembly(
            motion_request_id=47,
            source_session_id=result["source_session_id"],
            configuration_fingerprint=result["configuration_fingerprint"],
        )
        self.assertTrue(
            assembly.accept(protocol.parse_main_motion_trace_result(result))
        )
        motion_parameters = sample_main_move_joints_params()
        motion_parameters.update({
            "telemetry_enabled": False,
            "trace_configuration_fingerprint": result[
                "configuration_fingerprint"
            ],
        })
        artifact = assembly.artifact(
            controller_identity=sample_main_hello_result()["identity"],
            measurement_scale=configuration.motion_trace_scale,
            motion_parameters=motion_parameters,
            host_times_ns={
                "retrieved": 4,
                "terminal": 3,
                "admitted": 2,
                "armed": 1,
            },
        )
        with self.assertRaises(JsonCommandSchemaError):
            assembly.artifact(
                controller_identity=sample_main_hello_result()["identity"],
                measurement_scale=configuration.motion_trace_scale,
                motion_parameters=motion_parameters,
                host_times_ns={
                    "armed": 2,
                    "admitted": 1,
                    "terminal": 3,
                    "retrieved": 4,
                },
            )
        invalid_scale = dict(configuration.motion_trace_scale)
        invalid_scale["encoder_counts_per_step"] = (0,) * 6
        with self.assertRaises(JsonCommandSchemaError):
            assembly.artifact(
                controller_identity=sample_main_hello_result()["identity"],
                measurement_scale=invalid_scale,
                motion_parameters=motion_parameters,
                host_times_ns={
                    "armed": 1,
                    "admitted": 2,
                    "terminal": 3,
                    "retrieved": 4,
                },
            )
        self.assertEqual(artifact["artifact_version"], 2)
        self.assertEqual(
            artifact["measurement_scale"],
            {
                "encoder_counts_per_step": [5, 5, 5, 5, 2.5, 5],
                "steps_per_degree": list(configuration.steps_per_degree),
            },
        )
        self.assertEqual(artifact["analysis"]["sample_count"], 2)
        self.assertEqual(
            artifact["analysis"]["controller_duration_microseconds"],
            1,
        )
        joint_1 = artifact["analysis"]["joint_following_error"][0]
        self.assertEqual(joint_1["initial_following_error_steps"], 1.0)
        self.assertEqual(joint_1["terminal_following_error_steps"], -2.0)
        self.assertEqual(
            joint_1["maximum_absolute_following_error_steps"],
            2.0,
        )
        self.assertAlmostEqual(
            joint_1["terminal_following_error_degrees"],
            -2.0 / configuration.steps_per_degree[0],
        )

        with BoundedTemporaryDirectory() as directory:
            path = write_main_motion_trace_artifact(directory, artifact)
            stored = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(list(path.parent.iterdir()), [path])
            self.assertEqual(stored, json.loads(json.dumps(artifact)))
            with self.assertRaises(FileExistsError):
                write_main_motion_trace_artifact(directory, artifact)
            self.assertEqual(list(path.parent.iterdir()), [path])

    def test_motion_trace_assembly_rejects_page_discontinuity(self):
        first = protocol.parse_main_motion_trace_result(
            sample_main_motion_trace_result(
                motion_request_id=47,
                total_records=9,
            )
        )
        cases = (
            (
                "generation changed",
                True,
                sample_main_motion_trace_result(
                    motion_request_id=47,
                    capture_generation=2,
                    page_index=1,
                    total_records=9,
                ),
            ),
            (
                "first page skipped",
                False,
                sample_main_motion_trace_result(
                    motion_request_id=47,
                    page_index=1,
                    total_records=9,
                ),
            ),
        )
        for name, accept_first, result in cases:
            with self.subTest(name=name):
                assembly = JsonMainMotionTraceAssembly(
                    motion_request_id=47,
                    source_session_id=result["source_session_id"],
                    configuration_fingerprint=result[
                        "configuration_fingerprint"
                    ],
                )
                if accept_first:
                    self.assertFalse(assembly.accept(first))
                with self.assertRaises(JsonCommandSchemaError):
                    assembly.accept(
                        protocol.parse_main_motion_trace_result(result)
                    )

    def test_interrupted_move_joints_return_retains_recoverable_ticket(self):
        hello_result = sample_main_hello_result()
        client, _serial_port, _clock, _scheduler = self.make_client(
            (
                encode_message(
                    Response(1, "hello", "completed", hello_result)
                ),
                encode_message(
                    Response(
                        2,
                        "move_joints",
                        "completed",
                        sample_main_move_joints_result(),
                    )
                ),
            )
        )
        self.establish_session(client)
        submit = client._coordinator.submit

        def interrupt_after_submission(*args, **kwargs):
            submit(*args, **kwargs)
            raise KeyboardInterrupt()

        client._coordinator.submit = interrupt_after_submission
        with self.assertRaises(KeyboardInterrupt):
            client.request_move_joints(
                **sample_main_move_joints_params(),
                timeout=2.0,
            )

        self.assertEqual(len(client.pending_tickets), 1)
        ticket = client.pending_tickets[0]
        self.assertIs(client.pending_joint_motion_ticket, ticket)
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        self.assertIs(delivery.ticket, ticket)
        client.acknowledge_terminal(ticket)
        self.assertIsNone(client.pending_joint_motion_ticket)
        self.assertTrue(client.session_ready)

    def test_move_joints_requires_session(self):
        client, serial_port, _clock, _scheduler = self.make_client()
        params = sample_main_move_joints_params()

        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "JSON session is not established",
        ):
            client.request_move_joints(**params, timeout=2.0)
        self.assertEqual(serial_port.writes, [])

    def test_move_joints_requires_drained_requests_and_deliveries(self):
        params = sample_main_move_joints_params()
        hello_result = sample_main_hello_result()
        hello_response = encode_message(
            Response(1, "hello", "completed", hello_result)
        )

        pending_client, pending_serial, _clock, _scheduler = self.make_client(
            (hello_response,)
        )
        self.establish_session(pending_client)
        pending_client.request_position_disposition(timeout=2.0)
        pending_writes = tuple(pending_serial.writes)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "move_joints requires drained requests and deliveries",
        ):
            pending_client.request_move_joints(**params, timeout=2.0)
        self.assertEqual(tuple(pending_serial.writes), pending_writes)

        delivered_client, delivered_serial, _clock, _scheduler = (
            self.make_client(
                (
                    hello_response,
                    encode_message(
                        Response(
                            2,
                            "get_position_disposition",
                            "completed",
                            sample_main_position_disposition_result(),
                        )
                    ),
                )
            )
        )
        self.establish_session(delivered_client)
        delivered_client.request_position_disposition(timeout=2.0)
        self.poll_until_delivery(delivered_client)
        delivered_writes = tuple(delivered_serial.writes)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "move_joints requires drained requests and deliveries",
        ):
            delivered_client.request_move_joints(**params, timeout=2.0)
        self.assertEqual(tuple(delivered_serial.writes), delivered_writes)

    def test_move_cartesian_emits_exact_request_and_typed_terminal(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(
                Response(
                    2,
                    "move_cartesian",
                    "completed",
                    sample_main_move_cartesian_result(),
                )
            ),
            *(encode_message(Response(identifier, command, "completed",
                                      sample_main_move_cartesian_result()))
              for identifier, command in enumerate(
                  ("move_arc", "move_circle", "move_spline"), 3)),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        params = sample_main_move_cartesian_params()

        ticket = client.request_command("move_cartesian", params, timeout=2.0)
        self.assertIs(client.pending_motion_ticket, ticket)
        self.assertIs(client.pending_cartesian_motion_ticket, ticket)
        self.assertIsNone(client.pending_joint_motion_ticket)
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        terminal = client.take_terminal(ticket)

        self.assertIs(delivery.ticket, ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "move_cartesian", params)),
        )
        self.assertIsInstance(
            terminal.parsed_result,
            JsonMainCartesianMotionResult,
        )
        self.assertEqual(
            terminal.parsed_result.position.cartesian_translation_millimeters,
            (123.456, -789.0, 0.0),
        )
        client.acknowledge_terminal(ticket)
        self.assertIsNone(client.pending_motion_ticket)
        self.assertTrue(client.session_ready)

        motion = dict(params, telemetry_enabled=False)
        families = (
            ("move_arc", {"motion": motion, "midpoint_translation_millimeters": (0, 1, 0)}),
            ("move_circle", {"motion": motion, "center_translation_millimeters": (0, 0, 0),
                             "plane_translation_millimeters": (0, 2, 0)}),
            ("move_spline", {"segments": ({"motion": motion, "rounding_millimeters": 0.0},)}),
        )
        for identifier, (command, parameters) in enumerate(families, 3):
            ticket = client.request_command(command, parameters, timeout=2.0)
            self.assertIs(client.pending_motion_ticket, ticket)
            self.assertEqual(serial_port.writes[-1], encode_message(Request(identifier, command, parameters)))
            self.poll_until_delivery(client)
            self.assertIs(client.pop_delivery().ticket, ticket)
            terminal = client.take_terminal(ticket)
            self.assertIsInstance(terminal.parsed_result, JsonMainCartesianMotionResult)
            client.acknowledge_terminal(ticket)
        writes_before_rejection = len(serial_port.writes)
        families[-1][1]["segments"][0]["rounding_millimeters"] = 1.0
        with self.assertRaisesRegex(JsonSessionAdmissionError, "move_spline request validation failed"):
            client.request_command("move_spline", families[-1][1], timeout=2.0)
        self.assertEqual(len(serial_port.writes), writes_before_rejection)

    def test_tool_jog_emits_exact_request_and_typed_terminal(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(
                Response(
                    2,
                    "jog_tool",
                    "completed",
                    sample_main_tool_jog_result(),
                )
            ),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        params = sample_main_tool_jog_params()

        ticket = client.request_jog_tool(**params, timeout=2.0)
        self.assertIs(client.pending_motion_ticket, ticket)
        self.assertIs(client.pending_tool_jog_ticket, ticket)
        self.assertIsNone(client.pending_cartesian_motion_ticket)
        self.assertIsNone(client.pending_joint_motion_ticket)
        self.poll_until_delivery(client)
        delivery = client.pop_delivery()
        terminal = client.take_terminal(ticket)

        self.assertIs(delivery.ticket, ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "jog_tool", params)),
        )
        self.assertIsInstance(
            terminal.parsed_result,
            JsonMainToolJogResult,
        )
        self.assertEqual(
            terminal.parsed_result.position.cartesian_translation_millimeters,
            (123.456, -789.0, 0.0),
        )
        client.acknowledge_terminal(ticket)
        self.assertIsNone(client.pending_motion_ticket)
        self.assertTrue(client.session_ready)

    def test_interrupted_tool_jog_retains_recoverable_ticket(self):
        hello_result = sample_main_hello_result()
        client, _serial_port, _clock, _scheduler = self.make_client(
            (
                encode_message(
                    Response(1, "hello", "completed", hello_result)
                ),
                encode_message(
                    Response(
                        2,
                        "jog_tool",
                        "completed",
                        sample_main_tool_jog_result(),
                    )
                ),
            )
        )
        self.establish_session(client)
        submit = client._coordinator.submit

        def interrupt_after_submission(*args, **kwargs):
            submit(*args, **kwargs)
            raise KeyboardInterrupt()

        client._coordinator.submit = interrupt_after_submission
        with self.assertRaises(KeyboardInterrupt):
            client.request_jog_tool(
                **sample_main_tool_jog_params(),
                timeout=2.0,
            )

        ticket = client.pending_tickets[0]
        self.assertIs(client.pending_tool_jog_ticket, ticket)
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(ticket)
        self.assertIsNone(client.pending_motion_ticket)

    def test_live_joint_jog_and_correlated_stop_settle_both_owners(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(3, "stop", "completed", {"motion_id": 2})
            ),
            encode_message(
                Response(
                    2,
                    "live_joint_jog",
                    "completed",
                    sample_main_live_jog_result(),
                )
            ),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        hello_terminal = self.establish_session(client)
        params = sample_main_live_jog_params()

        motion_ticket = client.request_live_joint_jog(
            **params,
            timeout=30.0,
        )
        self.assertIs(client.pending_motion_ticket, motion_ticket)
        self.assertIs(client.pending_live_joint_jog_ticket, motion_ticket)
        self.assertIsNone(client.pending_live_stop_ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "live_joint_jog", params)),
        )

        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "live motion is not accepted",
        ):
            client.request_stop_live_motion(motion_ticket, timeout=16.0)
        self.poll_until_delivery(client)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "requires drained response deliveries",
        ):
            client.request_stop_live_motion(motion_ticket, timeout=16.0)
        accepted_delivery = client.pop_delivery()
        self.assertIs(accepted_delivery.ticket, motion_ticket)
        self.assertEqual(accepted_delivery.response.status, "accepted")

        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "live motion is not active",
        ):
            client.request_stop_live_motion(
                hello_terminal.response,
                timeout=16.0,
            )
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "live motion is active",
        ):
            client.request_position_disposition(timeout=2.0)

        stop_ticket = client.request_stop_live_motion(
            motion_ticket,
            timeout=16.0,
        )
        self.assertIs(client.pending_live_stop_ticket, stop_ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(
                Request(3, "stop", {"motion_id": motion_ticket.request_id})
            ),
        )
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "control request is already pending",
        ):
            client.request_stop_live_motion(motion_ticket, timeout=16.0)

        self.poll_until_delivery(client)
        stop_delivery = client.pop_delivery()
        stop_terminal = client.take_terminal(stop_ticket)
        self.assertIs(stop_delivery.ticket, stop_ticket)
        self.assertIsInstance(stop_terminal.parsed_result, JsonMainStopResult)
        self.assertEqual(stop_terminal.parsed_result.motion_id, 2)
        client.acknowledge_terminal(stop_ticket)
        self.assertIs(client.pending_motion_ticket, motion_ticket)
        self.assertIsNone(client.pending_live_stop_ticket)

        self.poll_until_delivery(client)
        motion_delivery = client.pop_delivery()
        motion_terminal = client.take_terminal(motion_ticket)
        self.assertIs(motion_delivery.ticket, motion_ticket)
        self.assertIsInstance(
            motion_terminal.parsed_result,
            JsonMainLiveJogResult,
        )
        client.acknowledge_terminal(motion_ticket)
        self.assertIsNone(client.pending_motion_ticket)
        self.assertTrue(client.session_ready)

    def test_live_stop_terminal_matches_retained_motion_and_response_order(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(3, "stop", "completed", {"motion_id": 9})
            ),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        stop_ticket = client.request_stop_live_motion(
            motion_ticket,
            timeout=16.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "does not match the retained live motion",
        ):
            client.take_terminal(stop_ticket)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "does not match the retained live motion",
        ):
            client.acknowledge_terminal(stop_ticket)
        self.assertIs(client.pending_live_stop_ticket, stop_ticket)

        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(
                    2,
                    "live_joint_jog",
                    "completed",
                    sample_main_live_jog_result(),
                )
            ),
            encode_message(
                Response(3, "stop", "completed", {"motion_id": 2})
            ),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        stop_ticket = client.request_stop_live_motion(
            motion_ticket,
            timeout=16.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        self.poll_until_delivery(client)
        client.pop_delivery()
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "arrived before the retained control terminal",
        ):
            client.take_terminal(motion_ticket)
        client.acknowledge_terminal(stop_ticket)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "arrived before the retained control terminal",
        ):
            client.take_terminal(motion_ticket)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "arrived before the retained control terminal",
        ):
            client.acknowledge_terminal(motion_ticket)
        self.assertIs(client.pending_motion_ticket, motion_ticket)
        self.assertIsNone(client.pending_live_stop_ticket)

    def test_interrupted_poll_records_live_terminal_order_fault(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(
                    2,
                    "live_joint_jog",
                    "completed",
                    sample_main_live_jog_result(),
                )
            ),
            encode_message(
                Response(3, "stop", "completed", {"motion_id": 2})
            ),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        stop_ticket = client.request_stop_live_motion(
            motion_ticket,
            timeout=16.0,
        )
        poll = client._coordinator.poll

        def interrupt_after_poll():
            for _attempt in range(32):
                if poll():
                    raise KeyboardInterrupt()
            raise AssertionError("bounded polling did not produce a delivery")

        client._coordinator.poll = interrupt_after_poll
        with self.assertRaises(KeyboardInterrupt):
            client.poll()
        client._coordinator.poll = poll
        self.assertIs(client.pop_delivery().ticket, motion_ticket)
        self.poll_until_delivery(client)
        self.assertIs(client.pop_delivery().ticket, stop_ticket)
        client.acknowledge_terminal(stop_ticket)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "arrived before the retained control terminal",
        ):
            client.acknowledge_terminal(motion_ticket)
        self.assertIs(client.pending_motion_ticket, motion_ticket)

    def test_live_terminal_order_survives_control_publication_delay(self):
        hello_result = sample_main_hello_result()
        cases = (
            ("control_first", True),
            ("motion_first", False),
        )
        for response_order, valid_order in cases:
            with self.subTest(response_order=response_order):
                motion_terminal = encode_message(
                    Response(
                        2,
                        "live_joint_jog",
                        "completed",
                        sample_main_live_jog_result(),
                    )
                )
                control_terminal = encode_message(
                    Response(
                        3,
                        "stop",
                        "completed",
                        {"motion_id": 2},
                    )
                )
                ordered_terminals = (
                    (control_terminal, motion_terminal)
                    if valid_order
                    else (motion_terminal, control_terminal)
                )
                responses = (
                    encode_message(
                        Response(1, "hello", "completed", hello_result)
                    ),
                    encode_message(
                        Response(2, "live_joint_jog", "accepted", {})
                    ),
                ) + ordered_terminals
                client, _serial_port, _clock, _scheduler = self.make_client(
                    responses
                )
                self.establish_session(client)
                motion_ticket = client.request_live_joint_jog(
                    **sample_main_live_jog_params(),
                    timeout=30.0,
                )
                self.poll_until_delivery(client)
                client.pop_delivery()
                submit = client._coordinator.submit
                control_ticket_ready = threading.Event()
                release_control_ticket = threading.Event()
                control_tickets = []
                failures = []

                def hold_control_ticket(*args, **kwargs):
                    ticket = submit(*args, **kwargs)
                    control_tickets.append(ticket)
                    control_ticket_ready.set()
                    if not release_control_ticket.wait(timeout=1.0):
                        raise TimeoutError(
                            "test did not release live-control ticket"
                        )
                    return ticket

                client._coordinator.submit = hold_control_ticket

                def request_stop():
                    try:
                        client.request_stop_live_motion(
                            motion_ticket,
                            timeout=16.0,
                        )
                    except BaseException as exc:
                        failures.append(exc)

                worker = threading.Thread(target=request_stop, daemon=True)
                worker.start()
                try:
                    self.assertTrue(control_ticket_ready.wait(timeout=1.0))
                    self.poll_until_delivery(client)
                    first_delivery = client.pop_delivery()
                    if not valid_order:
                        with self.assertRaisesRegex(
                            JsonMainControllerClientStateError,
                            "live terminal read rejected while control "
                            "request submission is active",
                        ):
                            client.take_terminal(motion_ticket)
                finally:
                    release_control_ticket.set()
                worker.join(timeout=1.0)
                self.assertFalse(worker.is_alive())
                self.assertEqual(failures, [])
                stop_ticket = control_tickets[0]
                expected_first_ticket = (
                    stop_ticket if valid_order else motion_ticket
                )
                self.assertIs(first_delivery.ticket, expected_first_ticket)
                self.poll_until_delivery(client)
                second_delivery = client.pop_delivery()
                expected_second_ticket = (
                    motion_ticket if valid_order else stop_ticket
                )
                self.assertIs(second_delivery.ticket, expected_second_ticket)
                client.acknowledge_terminal(stop_ticket)
                if valid_order:
                    client.acknowledge_terminal(motion_ticket)
                    self.assertIsNone(client.pending_motion_ticket)
                else:
                    with self.assertRaisesRegex(
                        JsonMainControllerClientStateError,
                        "arrived before the retained control terminal",
                    ):
                        client.acknowledge_terminal(motion_ticket)
                    self.assertIs(client.pending_motion_ticket, motion_ticket)

    def test_live_terminal_read_rechecks_concurrent_control_admission(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(
                    2,
                    "live_joint_jog",
                    "completed",
                    sample_main_live_jog_result(),
                )
            ),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        submit = client._coordinator.submit
        control_ticket_ready = threading.Event()
        release_control_ticket = threading.Event()
        control_tickets = []
        control_failures = []

        def hold_control_ticket(*args, **kwargs):
            ticket = submit(*args, **kwargs)
            control_tickets.append(ticket)
            control_ticket_ready.set()
            if not release_control_ticket.wait(timeout=1.0):
                raise TimeoutError("test did not release live-control ticket")
            return ticket

        client._coordinator.submit = hold_control_ticket

        def request_stop():
            try:
                client.request_stop_live_motion(
                    motion_ticket,
                    timeout=16.0,
                )
            except BaseException as exc:
                control_failures.append(exc)

        control_worker = threading.Thread(target=request_stop, daemon=True)
        control_worker.start()
        try:
            self.assertTrue(control_ticket_ready.wait(timeout=1.0))
            self.poll_until_delivery(client)
            self.assertIs(client.pop_delivery().ticket, motion_ticket)
            with self.assertRaisesRegex(
                JsonMainControllerClientStateError,
                "live terminal read rejected while control request "
                "submission is active",
            ):
                client.take_terminal(motion_ticket)
        finally:
            release_control_ticket.set()
        control_worker.join(timeout=1.0)
        self.assertFalse(control_worker.is_alive())
        self.assertEqual(control_failures, [])
        self.assertIs(client.pending_live_stop_ticket, control_tickets[0])

    def test_live_renewal_retains_motion_and_reuses_control_slot(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(
                    3,
                    "renew_live_motion",
                    "completed",
                    {"motion_id": 2},
                )
            ),
            encode_message(
                Response(4, "stop", "completed", {"motion_id": 2})
            ),
            encode_message(
                Response(
                    2,
                    "live_joint_jog",
                    "completed",
                    sample_main_live_jog_result(),
                )
            ),
        )
        client, serial_port, _clock, scheduler = self.make_client(responses)
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        self.assertFalse(scheduler.registrations[-1]["active"])

        renewal_ticket = client.request_renew_live_motion(
            motion_ticket,
            timeout=16.0,
        )
        self.assertIs(client.pending_live_renewal_ticket, renewal_ticket)
        self.assertIsNone(client.pending_live_stop_ticket)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(
                Request(
                    3,
                    "renew_live_motion",
                    {"motion_id": motion_ticket.request_id},
                )
            ),
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        renewal_terminal = client.take_terminal(renewal_ticket)
        self.assertIsInstance(
            renewal_terminal.parsed_result,
            JsonMainRenewLiveMotionResult,
        )
        self.assertEqual(renewal_terminal.parsed_result.motion_id, 2)
        client.acknowledge_terminal(renewal_ticket)
        self.assertIsNone(client.pending_live_renewal_ticket)
        self.assertIs(client.pending_motion_ticket, motion_ticket)

        stop_ticket = client.request_stop_live_motion(
            motion_ticket,
            timeout=16.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(stop_ticket)
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(motion_ticket)
        self.assertIsNone(client.pending_motion_ticket)

    def test_live_control_timeout_cannot_precede_lease_settlement(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        writes_before_control = tuple(serial_port.writes)

        for method in (
            client.request_renew_live_motion,
            client.request_stop_live_motion,
        ):
            with self.subTest(method=method.__name__):
                with self.assertRaisesRegex(
                    JsonSessionAdmissionError,
                    "must exceed the controller lease",
                ):
                    method(motion_ticket, timeout=15.0)
                self.assertEqual(tuple(serial_port.writes), writes_before_control)
                self.assertIsNone(client.pending_live_renewal_ticket)
                self.assertIsNone(client.pending_live_stop_ticket)

    def test_live_control_timeout_uses_configured_frame_bound(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            responses,
            frame_timeout=7.0,
        )
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        writes_before_control = tuple(serial_port.writes)

        with self.assertRaisesRegex(
            JsonSessionAdmissionError,
            "must exceed the controller lease",
        ):
            client.request_stop_live_motion(motion_ticket, timeout=19.0)
        self.assertEqual(tuple(serial_port.writes), writes_before_control)
        self.assertIsNone(client.pending_live_stop_ticket)

    def test_live_control_timeout_includes_firmware_frame_bound(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            responses,
            frame_timeout=1.0,
        )
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        writes_before_control = tuple(serial_port.writes)
        minimum_timeout = (
            JSON_LIVE_MOTION_LEASE_MAXIMUM_MILLISECONDS / 1000.0
            + 2.0 * JSON_MAIN_FIRMWARE_FRAME_RECEIVE_TIMEOUT_SECONDS
        )

        with self.assertRaisesRegex(
            JsonSessionAdmissionError,
            "must exceed the controller lease",
        ):
            client.request_stop_live_motion(
                motion_ticket,
                timeout=minimum_timeout,
            )
        self.assertEqual(tuple(serial_port.writes), writes_before_control)
        self.assertIsNone(client.pending_live_stop_ticket)

    def test_live_jog_modes_emit_exact_requests(self):
        cases = (
            (
                "live_joint_jog",
                "request_live_joint_jog",
                "pending_live_joint_jog_ticket",
                2,
            ),
            (
                "live_cart_jog",
                "request_live_cart_jog",
                "pending_live_cart_jog_ticket",
                "x",
            ),
            (
                "live_tool_jog",
                "request_live_tool_jog",
                "pending_live_tool_jog_ticket",
                "rz",
            ),
        )
        for command, method_name, property_name, axis in cases:
            with self.subTest(command=command):
                hello_result = sample_main_hello_result()
                params = sample_main_live_jog_params(axis=axis)
                responses = (
                    encode_message(
                        Response(1, "hello", "completed", hello_result)
                    ),
                    encode_message(
                        Response(2, command, "accepted", {})
                    ),
                    encode_message(
                        Response(
                            2,
                            command,
                            "completed",
                            sample_main_live_jog_result(),
                        )
                    ),
                )
                client, serial_port, _clock, _scheduler = self.make_client(
                    responses
                )
                self.establish_session(client)
                ticket = getattr(client, method_name)(
                    **params,
                    timeout=30.0,
                )
                self.assertIs(getattr(client, property_name), ticket)
                self.assertEqual(
                    serial_port.writes[-1],
                    encode_message(Request(2, command, params)),
                )
                self.poll_until_delivery(client)
                accepted = client.pop_delivery()
                self.assertEqual(accepted.response.status, "accepted")
                self.poll_until_delivery(client)
                completed = client.pop_delivery()
                self.assertEqual(completed.response.status, "completed")
                client.acknowledge_terminal(ticket)
                self.assertIsNone(client.pending_motion_ticket)

    def test_live_jog_requires_paired_request_and_delivery_capacity(self):
        hello_result = sample_main_hello_result()
        hello_frame = encode_message(
            Response(1, "hello", "completed", hello_result)
        )
        for client_options in (
            {"maximum_pending_requests": 1},
            {"delivery_capacity": 1},
        ):
            with self.subTest(client_options=client_options):
                client, serial_port, _clock, _scheduler = self.make_client(
                    (hello_frame,),
                    **client_options,
                )
                self.establish_session(client)
                writes_before_request = tuple(serial_port.writes)

                with self.assertRaisesRegex(
                    JsonMainControllerClientStateError,
                    "requires capacity for paired request ownership",
                ):
                    client.request_live_joint_jog(
                        **sample_main_live_jog_params(),
                        timeout=30.0,
                    )

                self.assertEqual(
                    tuple(serial_port.writes),
                    writes_before_request,
                )
                self.assertEqual(client.pending_tickets, ())

    def test_interrupted_live_jog_submission_retains_motion_owner(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(
                    2,
                    "live_joint_jog",
                    "completed",
                    sample_main_live_jog_result(),
                )
            ),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        submit = client._coordinator.submit

        def interrupt_after_submission(*args, **kwargs):
            submit(*args, **kwargs)
            raise KeyboardInterrupt()

        client._coordinator.submit = interrupt_after_submission
        with self.assertRaises(KeyboardInterrupt):
            client.request_live_joint_jog(
                **sample_main_live_jog_params(),
                timeout=30.0,
            )

        motion_ticket = client.pending_tickets[0]
        self.assertIs(client.pending_live_joint_jog_ticket, motion_ticket)
        self.poll_until_delivery(client)
        accepted = client.pop_delivery()
        self.assertIs(accepted.ticket, motion_ticket)
        self.assertEqual(accepted.response.status, "accepted")
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(motion_ticket)
        self.assertIsNone(client.pending_motion_ticket)

    def test_interrupted_live_stop_submission_retains_both_owners(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "live_joint_jog", "accepted", {})),
            encode_message(
                Response(3, "stop", "completed", {"motion_id": 2})
            ),
            encode_message(
                Response(
                    2,
                    "live_joint_jog",
                    "completed",
                    sample_main_live_jog_result(),
                )
            ),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        motion_ticket = client.request_live_joint_jog(
            **sample_main_live_jog_params(),
            timeout=30.0,
        )
        self.poll_until_delivery(client)
        client.pop_delivery()
        submit = client._coordinator.submit

        def interrupt_after_submission(*args, **kwargs):
            submit(*args, **kwargs)
            raise KeyboardInterrupt()

        client._coordinator.submit = interrupt_after_submission
        with self.assertRaises(KeyboardInterrupt):
            client.request_stop_live_motion(motion_ticket, timeout=16.0)

        self.assertIs(client.pending_live_joint_jog_ticket, motion_ticket)
        stop_ticket = tuple(
            ticket
            for ticket in client.pending_tickets
            if ticket.command == "stop"
        )[0]
        self.assertIs(client.pending_live_stop_ticket, stop_ticket)
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(stop_ticket)
        self.assertIs(client.pending_live_joint_jog_ticket, motion_ticket)
        self.assertIsNone(client.pending_live_stop_ticket)
        self.poll_until_delivery(client)
        client.pop_delivery()
        client.acknowledge_terminal(motion_ticket)
        self.assertIsNone(client.pending_motion_ticket)

    def test_configuration_commands_emit_exact_correlated_requests(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "update_params", "completed", {})),
            encode_message(Response(3, "config_ext_axis", "completed", {})),
            encode_message(Response(4, "set_position", "completed", {})),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        update_params = sample_main_update_params()

        update_ticket = client.request_update_params(
            **update_params,
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        update_delivery = client.pop_delivery()
        update_terminal = client.take_terminal(update_ticket)
        self.assertIs(update_delivery.ticket, update_ticket)
        self.assertIsNone(update_terminal.parsed_result)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(Request(2, "update_params", update_params)),
        )
        client.acknowledge_terminal(update_ticket)
        self.assertTrue(client.configuration_sync_required)
        self.assertFalse(client.session_ready)

        external_params = {
            "drive_rotations": (280, 280, 280),
            "motor_steps": (4000, 4000, 4000),
            "travel_units": (3450, 3450, 3450),
        }
        external_ticket = client.request_config_ext_axis(
            **external_params,
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        external_delivery = client.pop_delivery()
        external_terminal = client.take_terminal(external_ticket)
        self.assertIs(external_delivery.ticket, external_ticket)
        self.assertIsNone(external_terminal.parsed_result)
        self.assertEqual(
            serial_port.writes[-1],
            encode_message(
                Request(3, "config_ext_axis", external_params)
            ),
        )
        client.acknowledge_terminal(external_ticket)
        self.assertTrue(client.configuration_sync_required)
        self.assertFalse(client.session_ready)

        writes_before_blocked_requests = tuple(serial_port.writes)
        for operation in (
            lambda: client.request_position_disposition(timeout=2.0),
            lambda: client.request_home_reference(timeout=2.0),
        ):
            with self.subTest(operation=operation):
                with self.assertRaisesRegex(
                    JsonMainControllerClientStateError,
                    "requires set-position synchronization",
                ):
                    operation()
        self.assertEqual(
            tuple(serial_port.writes),
            writes_before_blocked_requests,
        )

        position_ticket = client.request_set_position(
            robot_joints_millidegrees=(0,) * 6,
            external_axes_milliunits=(0,) * 3,
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        client.acknowledge_terminal(position_ticket)
        self.assertFalse(client.configuration_sync_required)
        self.assertTrue(client.session_ready)

    def test_configuration_transition_requires_drained_client_state(self):
        hello_result = sample_main_hello_result()
        client, serial_port, _clock, _scheduler = self.make_client(
            (
                encode_message(
                    Response(1, "hello", "completed", hello_result)
                ),
            )
        )
        self.establish_session(client)
        position_ticket = client.request_position_disposition(timeout=2.0)
        writes_before_configuration = tuple(serial_port.writes)

        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "requires drained requests and deliveries",
        ):
            client.request_update_params(
                **sample_main_update_params(),
                timeout=2.0,
            )

        self.assertEqual(tuple(serial_port.writes), writes_before_configuration)
        self.assertEqual(client.pending_tickets, (position_ticket,))

    def test_configuration_transition_blocks_new_requests_until_acknowledged(
        self,
    ):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "update_params", "completed", {})),
        )
        client, serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        ticket = client.request_update_params(
            **sample_main_update_params(),
            timeout=2.0,
        )
        writes_after_configuration = tuple(serial_port.writes)

        for operation in (
            lambda: client.request_position_disposition(timeout=2.0),
            lambda: client.request_config_ext_axis(
                travel_units=(3450,) * 3,
                drive_rotations=(280,) * 3,
                motor_steps=(4000,) * 3,
                timeout=2.0,
            ),
            lambda: client.request_set_position(
                robot_joints_millidegrees=(0,) * 6,
                external_axes_milliunits=(0,) * 3,
                timeout=2.0,
            ),
        ):
            with self.subTest(operation=operation):
                with self.assertRaisesRegex(
                    JsonMainControllerClientStateError,
                    "configuration transition is already pending",
                ):
                    operation()
        self.assertEqual(tuple(serial_port.writes), writes_after_configuration)

        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "configuration transition is already pending",
        ):
            client.request_position_disposition(timeout=2.0)
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.configuration_sync_required)

    def test_configuration_submission_reservation_is_not_session_ready(self):
        hello_result = sample_main_hello_result()
        client, _serial_port, _clock, _scheduler = self.make_client(
            (
                encode_message(
                    Response(1, "hello", "completed", hello_result)
                ),
            )
        )
        self.establish_session(client)
        coordinator_submit = client._coordinator.submit
        ticket_ready = threading.Event()
        release_return = threading.Event()
        retained_tickets = []
        failures = []

        def hold_ticket_return(*args, **kwargs):
            ticket = coordinator_submit(*args, **kwargs)
            retained_tickets.append(ticket)
            ticket_ready.set()
            if not release_return.wait(timeout=1.0):
                raise TimeoutError(
                    "test did not release configuration ticket return"
                )
            return ticket

        client._coordinator.submit = hold_ticket_return

        def request_configuration():
            try:
                client.request_update_params(
                    **sample_main_update_params(),
                    timeout=2.0,
                )
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=request_configuration, daemon=True)
        worker.start()
        try:
            self.assertTrue(ticket_ready.wait(timeout=1.0))
            self.assertFalse(client.session_ready)
            with self.assertRaisesRegex(
                JsonMainControllerClientStateError,
                "request submission is active",
            ):
                client.request_position_disposition(timeout=2.0)
        finally:
            release_return.set()
        worker.join(timeout=1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(client.pending_tickets, tuple(retained_tickets))
        self.assertFalse(client.session_ready)

    def test_rejected_configuration_preserves_prior_sync_state(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(
                Response(
                    2,
                    "update_params",
                    "rejected",
                    error=ProtocolFailure(
                        "configuration_not_representable",
                        "configuration rejected",
                    ),
                )
            ),
            encode_message(Response(3, "update_params", "completed", {})),
            encode_message(
                Response(
                    4,
                    "config_ext_axis",
                    "rejected",
                    error=ProtocolFailure(
                        "configuration_not_representable",
                        "configuration rejected",
                    ),
                )
            ),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        ticket = client.request_update_params(
            **sample_main_update_params(),
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        self.assertEqual(client.take_terminal(ticket).response.status, "rejected")
        client.acknowledge_terminal(ticket)

        self.assertFalse(client.configuration_sync_required)
        self.assertTrue(client.session_ready)

        completed_ticket = client.request_update_params(
            **sample_main_update_params(),
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        client.acknowledge_terminal(completed_ticket)
        self.assertTrue(client.configuration_sync_required)

        rejected_ticket = client.request_config_ext_axis(
            travel_units=(3450,) * 3,
            drive_rotations=(280,) * 3,
            motor_steps=(4000,) * 3,
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        client.acknowledge_terminal(rejected_ticket)
        self.assertTrue(client.configuration_sync_required)
        self.assertFalse(client.session_ready)

    def test_rejected_position_preserves_required_configuration_sync(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "update_params", "completed", {})),
            encode_message(
                Response(
                    3,
                    "set_position",
                    "rejected",
                    error=ProtocolFailure(
                        "position_not_representable",
                        "position rejected",
                    ),
                )
            ),
            encode_message(Response(4, "set_position", "completed", {})),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        update_ticket = client.request_update_params(
            **sample_main_update_params(),
            timeout=2.0,
        )
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        client.acknowledge_terminal(update_ticket)

        position_args = {
            "robot_joints_millidegrees": (0,) * 6,
            "external_axes_milliunits": (0,) * 3,
            "timeout": 2.0,
        }
        rejected_ticket = client.request_set_position(**position_args)
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        client.acknowledge_terminal(rejected_ticket)
        self.assertTrue(client.configuration_sync_required)
        self.assertFalse(client.session_ready)
        with self.assertRaisesRegex(
            JsonMainControllerClientStateError,
            "requires set-position synchronization",
        ):
            client.request_position_disposition(timeout=2.0)

        completed_ticket = client.request_set_position(**position_args)
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        client.acknowledge_terminal(completed_ticket)
        self.assertFalse(client.configuration_sync_required)
        self.assertTrue(client.session_ready)

    def test_interrupted_configuration_submission_recovers_ticket(self):
        hello_result = sample_main_hello_result()
        responses = (
            encode_message(Response(1, "hello", "completed", hello_result)),
            encode_message(Response(2, "update_params", "completed", {})),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(responses)
        self.establish_session(client)
        submit = client._coordinator.submit

        def interrupt_after_submission(*args, **kwargs):
            submit(*args, **kwargs)
            raise KeyboardInterrupt()

        client._coordinator.submit = interrupt_after_submission
        try:
            with self.assertRaises(KeyboardInterrupt):
                client.request_update_params(
                    **sample_main_update_params(),
                    timeout=2.0,
                )
        finally:
            client._coordinator.submit = submit

        self.assertEqual(len(client.pending_tickets), 1)
        ticket = client.pending_tickets[0]
        self.poll_until_delivery(client)
        self.assertIsNotNone(client.pop_delivery())
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.configuration_sync_required)
        self.assertFalse(client.session_ready)

    def test_configuration_commands_require_valid_values(self):
        hello_response = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            (encode_message(hello_response),)
        )
        self.establish_session(client)
        writes_before_request = tuple(serial_port.writes)
        with self.assertRaises(JsonSessionAdmissionError):
            client.request_config_ext_axis(
                travel_units=(3450, -1, 3450),
                drive_rotations=(280, 280, 280),
                motor_steps=(4000, 4000, 4000),
                timeout=2.0,
            )
        self.assertEqual(tuple(serial_port.writes), writes_before_request)
        self.assertEqual(client.pending_tickets, ())

    def test_completed_hello_terminal_is_typed(self):
        response = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (encode_message(response),)
        )
        ticket = client.request_hello(timeout=2.0)

        self.poll_until_delivery(client)
        terminal = client.take_terminal(ticket)

        self.assertIsInstance(terminal.parsed_result, JsonMainHelloResult)
        self.assertEqual(
            terminal.parsed_result.identity.controller_hardware_id,
            "1705B6",
        )
        self.assertFalse(client.session_ready)
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.session_ready)
        self.assertEqual(client.session_binding, terminal.parsed_result)

    def test_documented_failure_preserves_structured_error(self):
        failure = ProtocolFailure(
            "position_unavailable",
            "controller position is unavailable",
        )
        response = Response(
            2,
            "get_position_disposition",
            "failed",
            error=failure,
        )
        hello_response = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (encode_message(hello_response), encode_message(response))
        )
        self.establish_session(client)
        ticket = client.request_position_disposition(timeout=2.0)

        self.poll_until_delivery(client)
        terminal = client.take_terminal(ticket)

        self.assertIsNone(terminal.parsed_result)
        self.assertEqual(terminal.failure, failure)

    def test_bound_contract_quarantines_an_invalid_response(self):
        invalid = sample_main_position_disposition_result()
        invalid["fault"] = None
        hello_response = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (
                encode_message(hello_response),
                encode_message(
                    Response(2, "get_position_disposition", "completed", invalid)
                ),
            )
        )
        self.establish_session(client)
        client.request_position_disposition(timeout=2.0)

        with self.assertRaises(JsonSessionProtocolError):
            self.poll_until_delivery(client)

        self.assertTrue(client.quarantined)
        self.assertFalse(client.session_ready)
        self.assertIsNone(client.session_binding)
        self.assertIsNotNone(client.quarantine_reason)

    def test_move_joints_quarantines_an_invalid_completed_result(self):
        hello_result = sample_main_hello_result()
        invalid = sample_main_move_joints_result()
        invalid.pop("position")
        client, _serial_port, _clock, _scheduler = self.make_client(
            (
                encode_message(
                    Response(1, "hello", "completed", hello_result)
                ),
                encode_message(
                    Response(2, "move_joints", "completed", invalid)
                ),
            )
        )
        self.establish_session(client)
        client.request_move_joints(
            **sample_main_move_joints_params(),
            timeout=2.0,
        )

        with self.assertRaises(JsonSessionProtocolError):
            self.poll_until_delivery(client)

        self.assertTrue(client.quarantined)
        self.assertFalse(client.session_ready)
        self.assertIsNone(client.session_binding)

    def test_invalid_timeout_is_rejected_before_transmission(self):
        client, serial_port, _clock, _scheduler = self.make_client()

        with self.assertRaises(JsonSessionAdmissionError):
            client.request_hello(timeout=0)

        self.assertEqual(serial_port.writes, [])
        self.assertEqual(client.pending_tickets, ())

    def test_failed_hello_can_be_acknowledged_and_retried(self):
        failed = Response(
            1,
            "hello",
            "failed",
            error=ProtocolFailure(
                "identity_unavailable",
                "controller identity is unavailable",
            ),
        )
        completed = Response(
            2,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            (encode_message(failed), encode_message(completed))
        )

        first_ticket = client.request_hello(timeout=2.0)
        with self.assertRaises(JsonMainControllerClientStateError):
            client.request_hello(timeout=2.0)
        self.poll_until_delivery(client)
        first_terminal = client.take_terminal(first_ticket)
        self.assertEqual(first_terminal.failure.code, "identity_unavailable")
        client.acknowledge_terminal(first_ticket)
        self.assertFalse(client.session_ready)

        second_ticket = client.request_hello(timeout=2.0)
        self.poll_until_delivery(client)
        client.acknowledge_terminal(second_ticket)

        self.assertTrue(client.session_ready)
        self.assertEqual(
            serial_port.writes,
            [
                encode_message(Request(1, "hello", {})),
                encode_message(Request(2, "hello", {})),
            ],
        )

    def test_rejected_hello_can_be_acknowledged_and_retried(self):
        rejected = Response(
            1,
            "hello",
            "rejected",
            error=ProtocolFailure(
                "invalid_parameter",
                "hello parameters must be empty",
                {"field": "params"},
            ),
        )
        completed = Response(
            2,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (encode_message(rejected), encode_message(completed))
        )

        first_ticket = client.request_hello(timeout=2.0)
        self.poll_until_delivery(client)
        first_terminal = client.take_terminal(first_ticket)
        self.assertEqual(first_terminal.failure.code, "invalid_parameter")
        client.acknowledge_terminal(first_ticket)
        self.assertFalse(client.session_ready)

        second_ticket = client.request_hello(timeout=2.0)
        self.poll_until_delivery(client)
        client.acknowledge_terminal(second_ticket)
        self.assertTrue(client.session_ready)

    def test_expiry_quarantines_and_invalidates_session_binding(self):
        completed = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, clock, _scheduler = self.make_client(
            (encode_message(completed),)
        )
        self.establish_session(client)
        ticket = client.request_position_disposition(timeout=2.0)
        clock.value = ticket.deadline

        with self.assertRaises(JsonSessionTimeoutError):
            client.expire()

        self.assertTrue(client.quarantined)
        self.assertFalse(client.session_ready)
        self.assertIsNone(client.session_binding)

    def test_acknowledgement_interruption_recovers_session_binding(self):
        completed = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (encode_message(completed),)
        )
        ticket = client.request_hello(timeout=2.0)
        self.poll_until_delivery(client)
        acknowledge = client._coordinator.acknowledge_terminal

        def interrupt_after_acknowledgement(retained_ticket):
            acknowledge(retained_ticket)
            raise KeyboardInterrupt()

        client._coordinator.acknowledge_terminal = (
            interrupt_after_acknowledgement
        )
        with self.assertRaises(KeyboardInterrupt):
            client.acknowledge_terminal(ticket)

        self.assertTrue(client.session_ready)
        self.assertEqual(client.pending_tickets, ())
        with self.assertRaises(JsonMainControllerClientStateError):
            client.request_hello(timeout=2.0)

    def test_interrupted_hello_return_retains_recoverable_ticket(self):
        completed = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (encode_message(completed),)
        )
        submit = client._coordinator.submit

        def interrupt_after_submission(*args, **kwargs):
            submit(*args, **kwargs)
            raise KeyboardInterrupt()

        client._coordinator.submit = interrupt_after_submission
        with self.assertRaises(KeyboardInterrupt):
            client.request_hello(timeout=2.0)

        self.assertEqual(len(client.pending_tickets), 1)
        ticket = client.pending_tickets[0]
        self.assertEqual(ticket.command, "hello")
        with self.assertRaises(JsonMainControllerClientStateError):
            client.request_hello(timeout=2.0)

        self.poll_until_delivery(client)
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.session_ready)

    def test_acknowledgement_interruption_before_release_is_retryable(self):
        completed = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, _serial_port, _clock, _scheduler = self.make_client(
            (encode_message(completed),)
        )
        ticket = client.request_hello(timeout=2.0)
        self.poll_until_delivery(client)
        acknowledge = client._coordinator.acknowledge_terminal

        def interrupt_before_acknowledgement(_retained_ticket):
            raise KeyboardInterrupt()

        client._coordinator.acknowledge_terminal = (
            interrupt_before_acknowledgement
        )
        with self.assertRaises(KeyboardInterrupt):
            client.acknowledge_terminal(ticket)

        self.assertFalse(client.session_ready)
        self.assertEqual(client.pending_tickets, (ticket,))
        client._coordinator.acknowledge_terminal = acknowledge
        client.acknowledge_terminal(ticket)
        self.assertTrue(client.session_ready)
        self.assertEqual(client.pending_tickets, ())

    def test_close_verifies_owned_serial_closure(self):
        completed = Response(
            1,
            "hello",
            "completed",
            sample_main_hello_result(),
        )
        client, serial_port, _clock, _scheduler = self.make_client(
            (encode_message(completed),)
        )
        self.establish_session(client)

        client.close(timeout=0.1)

        self.assertTrue(client.closed)
        self.assertFalse(client.closing)
        self.assertFalse(client.session_ready)
        self.assertIsNone(client.session_binding)
        self.assertFalse(serial_port.is_open)
        self.assertEqual(serial_port.close_calls, 1)

    def test_public_terminal_rejects_inconsistent_shapes(self):
        invalid_hello = sample_main_hello_result()
        invalid_hello["unexpected"] = True
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainControllerTerminal(
                Response(1, "hello", "completed", invalid_hello)
            )
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainControllerTerminal(
                Response(
                    2,
                    "get_position_disposition",
                    "failed",
                    error=ProtocolFailure(
                        "internal_error",
                        "unexpected failure",
                    ),
                )
            )
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainControllerTerminal(
                Response(3, "hello", "accepted", {}),
            )


class JsonAuxiliaryControllerClientTests(unittest.TestCase):
    def poll_delivery(self, client):
        for _attempt in range(32):
            if client.poll():
                return client.pop_delivery()
        self.fail("bounded polling did not produce a complete delivery")

    def test_hello_command_and_wait_stop_ordering(self):
        hello_result = {
            "board": "nano",
            "commands": list(protocol.JSON_AUXILIARY_COMMAND_MANIFEST),
            "device": "auxiliary_controller",
            "firmware": {"build": "ar4hmi", "name": "AR4 Nano IO", "version": "2.0"},
            "protocol": {"max_payload_bytes": 384, "name": "ar4_json", "version": 1},
        }
        responses = (
            Response(1, "hello", "completed", hello_result),
            Response(2, "set_output", "completed", {}),
            Response(4, "stop", "completed", {}),
            Response(3, "wait_input", "cancelled", error=ProtocolFailure(
                "stop_requested", "Input wait stopped")),
        )
        serial_port = FakeSerial(tuple(map(encode_message, responses)))
        client = protocol.JsonAuxiliaryControllerClient(
            serial_port, clock=FakeClock(), clock_resolution=0.0,
            deadline_scheduler=ManualDeadlineScheduler())
        hello = client.request_hello(timeout=2.0)
        self.assertIs(self.poll_delivery(client).ticket, hello)
        identity = client.take_terminal(hello).parsed_result
        catalog_names = tuple(
            command.name for command in protocol.AUXILIARY_COMMANDS)
        contract_names = tuple(
            contract.name for contract in protocol.AUXILIARY_JSON_COMMAND_CONTRACTS)
        self.assertEqual(
            (
                identity.commands,
                contract_names,
                len(identity.commands),
                len(contract_names),
            ),
            (catalog_names, catalog_names, 8, 8),
        )
        client.acknowledge_terminal(hello)
        writes_before_rejection = tuple(serial_port.writes)
        invalid = (("servo", {"channel": 6, "position": 90}),
                   ("input_read", {"pin": 8}), ("set_output", {"pin": 28, "state": True}))
        for command, params in invalid:
            with self.subTest(command=command), self.assertRaisesRegex(
                    JsonCommandSchemaError, "bound auxiliary board"):
                client.request_command(command, params, timeout=2.0)
        self.assertEqual(tuple(serial_port.writes), writes_before_rejection)

        output = client.request_command(
            "set_output", {"pin": 8, "state": True}, timeout=2.0)
        expected_frame = b'{"cmd":"set_output","id":2,"params":{"pin":8,"state":true},'
        expected_frame += b'"type":"request","v":1}\n'
        self.assertEqual(serial_port.writes[-1], expected_frame)
        self.poll_delivery(client)
        client.acknowledge_terminal(output)
        wait = client.request_command(
            "wait_input", {"pin": 2, "state": True, "timeout_seconds": 5},
            timeout=7.0)
        with self.assertRaises(protocol.JsonAuxiliaryControllerClientStateError):
            client.request_command("input_read", {"pin": 2}, timeout=2.0)
        stop = client.request_command("stop", {}, timeout=2.0)
        stop_delivery = self.poll_delivery(client)
        self.assertIs(stop_delivery.ticket, stop)
        client.acknowledge_terminal(stop)
        wait_delivery = self.poll_delivery(client)
        self.assertIs(wait_delivery.ticket, wait)
        self.assertEqual(
            client.take_terminal(wait).failure.code, "stop_requested")
        client.acknowledge_terminal(wait)
        self.assertEqual(
            tuple(request.cmd for request in map(
                protocol.decode_message, serial_port.writes)),
            ("hello", "set_output", "wait_input", "stop"))


if __name__ == "__main__":
    unittest.main()
