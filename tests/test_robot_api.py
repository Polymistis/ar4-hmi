import unittest
from unittest import mock

from ARrobots import automation, protocol
from tests import test_json_main_controller as fixtures
from tests.test_json_coordinator import FakeSerial

POSITION = fixtures.sample_main_position_disposition_result()
HOME = fixtures.sample_main_home_reference_result()
STARTUP = (fixtures.sample_main_hello_result(), {}, {}, {}, POSITION, HOME)
COMMANDS = ("hello", "update_params", "config_ext_axis", "set_position",
            "get_position_disposition", "get_home_reference")
CONFIGURATION = protocol.JsonMainControllerStartupConfiguration(
    **fixtures.sample_main_update_params(),
    external_axis_travel_units=(3450, 3450, 3450),
    external_axis_drive_rotations=(280, 280, 280),
    external_axis_motor_steps=(4000, 4000, 4000),
    robot_joints_millidegrees=(0,) * 6,
    external_axes_milliunits=(0,) * 3,
)
AUXILIARY_HELLO = {
    "board": "nano", "commands": list(protocol.JSON_AUXILIARY_COMMAND_MANIFEST),
    "device": "auxiliary_controller",
    "firmware": {"build": "ar4hmi", "name": "AR4 Nano IO", "version": "2.0"},
    "protocol": {"max_payload_bytes": 384, "name": "ar4_json", "version": 1},
}
MOVE_PARAMS = fixtures.sample_main_move_joints_params()
del MOVE_PARAMS["telemetry_enabled"]
MOVE_RESULT = fixtures.sample_main_move_joints_result()
CALIBRATION_PARAMS = fixtures.sample_main_calibration_params()
CALIBRATION_RESULT = fixtures.sample_main_calibration_result()
CARTESIAN_PARAMS = fixtures.sample_main_move_cartesian_params()
del CARTESIAN_PARAMS["telemetry_enabled"]
CARTESIAN_RESULT = fixtures.sample_main_move_cartesian_result()
TOOL_JOG_PARAMS = fixtures.sample_main_tool_jog_params()
TOOL_JOG_RESULT = fixtures.sample_main_tool_jog_result()

class ReactiveSerial(FakeSerial):
    def __init__(self, *outcomes):
        super().__init__()
        self.outcomes = list(outcomes)

    def write(self, frame):
        written = super().write(frame)
        request = protocol.decode_message(frame)
        outcome = self.outcomes.pop(0)
        messages = outcome(request) if callable(outcome) else (() if outcome is None else (
            protocol.Response(request.id, request.cmd, "completed", outcome),))
        self.reads.extend(protocol.encode_message(message) for message in messages)
        return written
def rejected(code, status="rejected"):
    return lambda request: (protocol.Response(
        request.id, request.cmd, status,
        error=protocol.ProtocolFailure(code, "controller rejected request")),)
class RobotApiTests(unittest.TestCase):
    def connect_main(self, *outcomes, startup=STARTUP, error=None, stops=None,
                     boundary=None):
        serial_port = ReactiveSerial(*(startup + outcomes))
        stops = [] if stops is None else stops
        def publish(stop):
            stops.append((stop.command, stop.source, serial_port.is_open))
            return True
        arguments = dict(configuration=CONFIGURATION, physical_stop_callback=publish,
                         request_timeout=0.2)
        if boundary is not None:
            arguments["cancellation_boundary"] = boundary
        if error is not None:
            with self.assertRaises(error) as caught:
                automation.ARRobot.connect(serial_port, **arguments)
            return caught.exception, serial_port, stops
        return automation.ARRobot.connect(serial_port, **arguments), serial_port, stops

    def facade_stop(self, outcome, method="refresh_position", queued=(),
                    source="emergency_stop_event"):
        robot, serial_port, published = self.connect_main(outcome)
        serial_port.reads.extend(protocol.encode_message(item) for item in queued)
        with self.assertRaises(protocol.JsonMainControllerPhysicalStopError):
            getattr(robot, method)(timeout=0.2)
        with self.assertRaises(automation.AutomationStateError):
            getattr(robot, method)(timeout=0.2)
        command = "get_home_reference" if method == "refresh_home_reference" else "get_position_disposition"
        writes = tuple(protocol.decode_message(frame).cmd for frame in serial_port.writes)
        self.assertEqual((published, writes, serial_port.is_open),
                         ([(command, source, True)], COMMANDS + (() if queued else (command,)), False))
        return published

    def test_validated_startup_and_read_command_mapping(self):
        current_position = dict(POSITION, robot_joints_millidegrees=[1000] * 6)
        current_home = dict(HOME, positions_millidegrees=[1000, 0, 3000])
        results = (rejected("session_not_established"), current_position, current_home,
                   {"active": [True, False] * 3},
                   {"counts": [-1, 0, 1, 1000, -2147483648, 2147483647]})
        robot, serial_port, _stops = self.connect_main(*results)
        self.assertEqual(tuple(type(value) for value in (
            robot.identity, robot.startup_position, robot.startup_home_reference)),
            (protocol.JsonMainHelloResult, protocol.JsonMainPositionResult,
             protocol.JsonMainHomeReferenceResult))
        calls = (robot.refresh_position, robot.refresh_home_reference,
                 robot.read_limit_switches, robot.read_encoders)
        with self.assertRaises(automation.AutomationCommandError) as rejected_call:
            robot.refresh_position(timeout=0.2)
        self.assertEqual((rejected_call.exception.controller, rejected_call.exception.command,
                          rejected_call.exception.status, rejected_call.exception.failure.code),
                         (protocol.MAIN_CONTROLLER, COMMANDS[-2], "rejected",
                          "session_not_established"))
        values = tuple(call(timeout=0.2) for call in calls)
        self.assertEqual(tuple(type(value) for value in values),
                         (protocol.JsonMainPositionResult,
                          protocol.JsonMainHomeReferenceResult, tuple, tuple))
        self.assertEqual((values[0].robot_joints_millidegrees,
                          values[1].positions_millidegrees),
                         ((1000,) * 6, (1000, 0, 3000)))
        self.assertEqual(tuple(protocol.decode_message(frame).cmd
                               for frame in serial_port.writes),
                         COMMANDS + (COMMANDS[-2],) + COMMANDS[-2:] +
                         ("test_limit_switches", "read_encoders"))
        robot.close()

    def test_validated_nano_facade_commands_rejections_and_close(self):
        serial_port = ReactiveSerial(
            AUXILIARY_HELLO, rejected("busy"), {"state": True},
            {}, {}, {"amps": 1.25}, {}, rejected("timeout", "failed"), {}, mock.Mock(side_effect=TimeoutError))
        auxiliary = automation.ARAuxiliary.connect(
            serial_port, expected_board="nano", request_timeout=0.2)
        self.assertEqual((type(auxiliary.identity), auxiliary.board_profile),
                         (protocol.JsonAuxiliaryHelloResult, "nano"))
        with self.assertRaises(automation.AutomationCommandError) as caught:
            auxiliary.set_servo(0, 90, timeout=0.2)
        self.assertEqual((caught.exception.controller, caught.exception.command,
                          caught.exception.status, caught.exception.failure.code),
                         (protocol.AUXILIARY_CONTROLLER, "servo", "rejected",
                          "busy"))
        invalid_calls = ((auxiliary.set_servo, (6, 90)),
                         (auxiliary.read_input, (8,)),
                         (auxiliary.set_output, (7, True)))
        for call, arguments in invalid_calls:
            with self.assertRaises(protocol.JsonCommandSchemaError):
                call(*arguments, timeout=0.2)
        for pin, state, timeout in ((8, True, 1), (2, 1, 1), (2, True, 0)):
            with self.assertRaises(protocol.JsonCommandSchemaError): auxiliary.wait_input(pin, state, timeout_seconds=timeout)
        self.assertEqual(auxiliary.read_input(2, timeout=0.2),
                         protocol.JsonAuxiliaryInputResult(True))
        results = (auxiliary.set_servo(5, 180, timeout=0.2),
                   auxiliary.set_output(8, True, timeout=0.2),
                   auxiliary.test_gripper_amps(timeout=0.2),
                   auxiliary.detach_gripper(timeout=0.2))
        self.assertEqual(results, (None, None,
                                   protocol.JsonAuxiliaryCurrentResult(1.25), None))
        # Delete this client seam when host deadlines gain a public observation point.
        client_type = protocol.JsonAuxiliaryControllerClient
        with mock.patch.object(client_type, "request_command", autospec=True,
                               side_effect=client_type.request_command) as captured:
            with self.assertRaises(automation.AutomationCommandError) as caught: auxiliary.wait_input(2, True, timeout_seconds=1)
            self.assertIsNone(auxiliary.wait_input(2, True, timeout_seconds=1))
        expected_timeout = 1 + 2 * protocol.JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS
        self.assertEqual((caught.exception.failure.code, captured.call_args.kwargs["timeout"]), ("timeout", expected_timeout))
        messages = map(protocol.decode_message, serial_port.writes)
        self.assertEqual(tuple((message.cmd, dict(message.params)) for message in messages), (
            ("hello", {}), ("servo", {"channel": 0, "position": 90}),
            ("input_read", {"pin": 2}), ("servo", {"channel": 5, "position": 180}),
            ("set_output", {"pin": 8, "state": True}),
            ("test_gripper_amps", {}), ("gripper_detach", {})) + (
            ("wait_input", {"pin": 2, "state": True, "timeout_seconds": 1}),) * 2)
        with self.assertRaises(automation.AutomationCleanupError): auxiliary.wait_input(2, True, timeout_seconds=1)
        self.assertFalse(serial_port.is_open)
        mega_port = ReactiveSerial(dict(
            AUXILIARY_HELLO, board="mega", firmware=dict(AUXILIARY_HELLO["firmware"], name="AR4 Mega IO")))
        with automation.ARAuxiliary.connect(mega_port, expected_board="mega", request_timeout=0.2) as mega:
            with self.assertRaises(protocol.JsonCommandSchemaError): mega.wait_input(28, True, timeout_seconds=1)
            self.assertEqual(len(mega_port.writes), 1)

    def test_calibration_mapping_validation_rejection_and_reuse(self):
        accepted = lambda request: (
            protocol.Response(request.id, request.cmd, "accepted", {}),
            protocol.Response(request.id, request.cmd, "completed", CALIBRATION_RESULT))
        not_representable = lambda request: (protocol.Response(
            request.id, request.cmd, "rejected", error=protocol.ProtocolFailure(
                "calibration_not_representable", "calibration rejected", {"axes": CALIBRATION_PARAMS["axes"]})),)
        robot, serial_port, published = self.connect_main(
            not_representable, accepted, MOVE_RESULT)
        before = tuple(serial_port.writes)
        with self.assertRaises(protocol.JsonCommandSchemaError):
            robot.calibrate(**dict(CALIBRATION_PARAMS, axes=(False,) * 9), timeout=0.2)
        self.assertEqual(tuple(serial_port.writes), before)
        with self.assertRaises(automation.AutomationCommandError) as caught:
            robot.calibrate(**CALIBRATION_PARAMS, timeout=0.2)
        result = robot.calibrate(**CALIBRATION_PARAMS, timeout=0.2)
        messages = tuple(map(protocol.decode_message, serial_port.writes))
        self.assertEqual(caught.exception.failure.code, "calibration_not_representable")
        self.assertEqual(result, protocol.parse_main_calibration_result(CALIBRATION_RESULT))
        self.assertEqual(tuple((item.cmd, dict(item.params)) for item in messages[-2:]),
                         (("calibrate", CALIBRATION_PARAMS),) * 2)
        self.assertEqual((tuple(item.cmd for item in messages), published),
                         (COMMANDS + ("calibrate",) * 2, []))
        robot.move_joints(**MOVE_PARAMS, timeout=0.2)
        message = protocol.decode_message(serial_port.writes[-1])
        self.assertEqual((message.cmd, dict(message.params)),
                         ("move_joints", dict(MOVE_PARAMS, telemetry_enabled=False)))
        robot.close()

    def test_calibration_timeout_closes_without_stop_claim(self):
        accepted = lambda request: (protocol.Response(
            request.id, request.cmd, "accepted", {}),)
        robot, serial_port, published = self.connect_main(accepted)
        with self.assertRaises(automation.AutomationCleanupError):
            robot.calibrate(**CALIBRATION_PARAMS, timeout=0.01)
        with self.assertRaises(automation.AutomationStateError):
            robot.refresh_position(timeout=0.2)
        commands = tuple(protocol.decode_message(frame).cmd for frame in serial_port.writes)
        self.assertEqual((commands, published, serial_port.is_open),
                         (COMMANDS + ("calibrate",), [], False))

    def test_modbus_facade_wire_mapping(self):
        cases = (
            ("modbus_read_holding_register", dict(slave_id=1, address=10, timeout=0.2), {"value": 42}),
            ("modbus_read_coil", dict(slave_id=2, address=11, timeout=0.2), {"value": 1}),
            ("modbus_read_discrete_input", dict(slave_id=3, address=12, timeout=0.2), {"value": 0}),
            ("modbus_read_input_register", dict(slave_id=4, address=13, timeout=0.2), {"value": 65535}),
            ("modbus_write_coil", dict(slave_id=5, address=14, value=1, timeout=0.2), {}),
            ("modbus_write_register", dict(slave_id=6, address=15, value=123, timeout=0.2), {}),
            ("wait_modbus_coil", dict(slave_id=7, address=16, expected=1, timeout_seconds=1), {"value": 1}),
            ("wait_modbus_discrete_input", dict(slave_id=8, address=17, expected=0, timeout_seconds=2), {"value": 0}),
            ("wait_modbus_holding_register", dict(slave_id=9, address=18, expected=456, timeout_seconds=3), {"value": 456}),
        )
        robot, serial_port, _published = self.connect_main(*(case[2] for case in cases))
        request_command, wait_deadlines = protocol.JsonMainControllerClient.request_command, []
        def observe_request(client, command, params, *, timeout, write_admission=None):
            wait_deadlines.append(timeout)
            return request_command(client, command, params, timeout=timeout, write_admission=write_admission)
        # Delete this client seam when host deadlines gain a public observation point.
        with mock.patch.object(protocol.JsonMainControllerClient,
                               "request_command", observe_request):
            results = tuple(getattr(robot, method)(**kwargs) for method, kwargs, _outcome in cases)
        messages = tuple(map(protocol.decode_message, serial_port.writes[-len(cases):]))
        expected_params = tuple(
            {key: value for key, value in kwargs.items() if key not in ("timeout",)}
            | ({"count": 1} if method.startswith("modbus_read_") else {})
            for method, kwargs, _outcome in cases)
        self.assertEqual(results, (42, 1, 0, 65535, None, None,
                                   protocol.JsonScalarResult(1), protocol.JsonScalarResult(0), protocol.JsonScalarResult(456)))
        self.assertEqual(tuple((item.cmd, dict(item.params)) for item in messages),
                         tuple((case[0], params) for case, params in zip(cases, expected_params)))
        frame = protocol.JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS
        self.assertEqual(wait_deadlines, [1 + 2 * frame, 2 + 2 * frame, 3 + 2 * frame])
        robot.close()

    def test_modbus_failed_write_closes_as_indeterminate(self):
        failed = lambda request: (protocol.Response(
            request.id, request.cmd, "failed", error=protocol.ProtocolFailure("modbus_error", "bus failed")),)
        robot, serial_port, _published = self.connect_main(failed)
        with self.assertRaises(automation.AutomationCleanupError) as caught:
            robot.modbus_write_coil(slave_id=1, address=5, value=1, timeout=0.2)
        self.assertIn("externally indeterminate",
                      str(caught.exception.operation_error.__cause__))
        with self.assertRaises(automation.AutomationStateError):
            robot.modbus_read_coil(slave_id=1, address=5, timeout=0.2)
        self.assertFalse(serial_port.is_open)

    def test_modbus_wait_mismatched_completed_value_closes(self):
        robot, serial_port, _published = self.connect_main({"value": 0})
        with self.assertRaises(protocol.JsonSessionProtocolError):
            robot.wait_modbus_coil(
                slave_id=1, address=6, expected=1, timeout_seconds=1)
        with self.assertRaises(automation.AutomationStateError):
            robot.refresh_position(timeout=0.2)
        self.assertFalse(serial_port.is_open)

    def test_finite_motion_facade_mapping_and_typed_results(self):
        motion = dict(CARTESIAN_PARAMS, telemetry_enabled=False)
        spline = ({"motion": motion, "rounding_millimeters": 0.0},)
        midpoint, center, plane = (50.0, -50.0, 25.0), (0.0, 0.0, 0.0), (0.0, 10.0, 0.0)
        cartesian_result = protocol.parse_main_move_cartesian_result(CARTESIAN_RESULT)
        tool_result = protocol.parse_main_tool_jog_result(TOOL_JOG_RESULT)
        cases = (
            ("move_cartesian", dict(CARTESIAN_PARAMS, timeout=0.2), motion,
             CARTESIAN_RESULT, cartesian_result, True),
            ("move_linear", dict(CARTESIAN_PARAMS, timeout=0.2),
             {"motion": motion, "rounding_millimeters": 0.0,
              "disable_wrist_rotation": False}, CARTESIAN_RESULT, cartesian_result, True),
            ("move_vision", dict(CARTESIAN_PARAMS, vision_rotation_degrees=15.0, timeout=0.2),
             {"motion": motion, "vision_rotation_degrees": 15.0},
             CARTESIAN_RESULT, cartesian_result, True),
            ("jog_tool", dict(TOOL_JOG_PARAMS, timeout=0.2), TOOL_JOG_PARAMS,
             TOOL_JOG_RESULT, tool_result, True),
            ("move_arc", dict(CARTESIAN_PARAMS,
                              midpoint_translation_millimeters=midpoint, timeout=0.2),
             {"motion": motion, "midpoint_translation_millimeters": midpoint},
             CARTESIAN_RESULT, cartesian_result, False),
            ("move_circle", dict(CARTESIAN_PARAMS,
                                 center_translation_millimeters=center,
                                 plane_translation_millimeters=plane, timeout=0.2),
             {"motion": motion, "center_translation_millimeters": center,
              "plane_translation_millimeters": plane},
             CARTESIAN_RESULT, cartesian_result, False),
            ("move_spline", dict(segments=spline, timeout=0.2), {"segments": spline},
             CARTESIAN_RESULT, cartesian_result, False),
        )
        def outcome(payload):
            return lambda request: (
                protocol.Response(request.id, request.cmd, "accepted", {}),
                protocol.Response(request.id, request.cmd, "completed", payload),
            )
        robot, serial_port, _published = self.connect_main(*(
            outcome(payload) if accepts else payload
            for _command, _kwargs, _wire, payload, _expected, accepts in cases
        ))
        results = tuple(getattr(robot, command)(**kwargs)
                        for command, kwargs, _wire, _payload, _expected, _accepts in cases)
        messages = tuple(map(protocol.decode_message, serial_port.writes[-len(cases):]))
        self.assertEqual(tuple((type(value), value) for value in results),
                         tuple((type(case[4]), case[4]) for case in cases))
        self.assertEqual(tuple((message.cmd, dict(message.params)) for message in messages),
                         tuple((case[0], case[2]) for case in cases))
        robot.close()

    def test_finite_motion_validation_and_rejection_reuse(self):
        robot, serial_port, published = self.connect_main(
            rejected("kinematics_unreachable"), CARTESIAN_RESULT)
        before = tuple(serial_port.writes)
        invalid = (
            (robot.move_cartesian,
             dict(CARTESIAN_PARAMS, translation_millimeters=(1.0, 2.0), timeout=0.2)),
            (robot.jog_tool, dict(TOOL_JOG_PARAMS, axis="invalid", timeout=0.2)),
            (robot.move_spline, dict(segments=(), timeout=0.2)),
        )
        for method, kwargs in invalid:
            with self.assertRaises(protocol.JsonCommandSchemaError):
                method(**kwargs)
        self.assertEqual(tuple(serial_port.writes), before)
        with self.assertRaises(automation.AutomationCommandError) as caught:
            robot.move_cartesian(**CARTESIAN_PARAMS, timeout=0.2)
        result = robot.move_cartesian(**CARTESIAN_PARAMS, timeout=0.2)
        commands = tuple(protocol.decode_message(frame).cmd for frame in serial_port.writes)
        self.assertEqual(caught.exception.failure.code, "kinematics_unreachable")
        self.assertEqual(result, protocol.parse_main_move_cartesian_result(CARTESIAN_RESULT))
        self.assertEqual((commands, published), (COMMANDS + ("move_cartesian",) * 2, []))
        robot.close()

    def test_finite_cartesian_timeout_closes_without_stop_or_retry(self):
        accepted = lambda request: (protocol.Response(
            request.id, request.cmd, "accepted", {}),)
        robot, serial_port, published = self.connect_main(accepted)
        with self.assertRaises(automation.AutomationCleanupError):
            robot.move_cartesian(**CARTESIAN_PARAMS, timeout=0.01)
        with self.assertRaises(automation.AutomationStateError):
            robot.move_cartesian(**CARTESIAN_PARAMS, timeout=0.2)
        commands = tuple(protocol.decode_message(frame).cmd for frame in serial_port.writes)
        self.assertEqual((commands, published, serial_port.is_open),
                         (COMMANDS + ("move_cartesian",), [], False))

    def test_terminal_stop_publishes_before_acknowledgement(self):
        cases = (
            ("wait_modbus_coil", dict(slave_id=1, address=7, expected=1, timeout_seconds=1), {}, None),
            ("calibrate", dict(CALIBRATION_PARAMS, timeout=0.2),
             {"position": CALIBRATION_RESULT["position"]},
             protocol.parse_main_calibration_result(CALIBRATION_RESULT).position),
            ("move_joints", dict(MOVE_PARAMS, timeout=0.2), {"position": MOVE_RESULT["position"]},
             protocol.parse_main_move_joints_result(MOVE_RESULT).position),
            ("move_cartesian", dict(CARTESIAN_PARAMS, timeout=0.2),
             {"position": CARTESIAN_RESULT["position"]},
             protocol.parse_main_move_cartesian_result(CARTESIAN_RESULT).position),
        )
        for command, kwargs, details, expected_position in cases:
            event = protocol.Event(1, "emergency_stop", {"asserted": True})
            cancelled = lambda request: (protocol.Response(
                request.id, request.cmd, "cancelled", error=protocol.ProtocolFailure(
                    "emergency_stop", "motion stopped", details)), event)
            robot, serial_port, published = self.connect_main(cancelled)
            acknowledge = protocol.JsonMainControllerClient.acknowledge_terminal
            publication_at_ack = []
            def observe_acknowledgement(client, ticket):
                publication_at_ack.append(tuple(published))
                return acknowledge(client, ticket)
            # No public observation point exposes settlement order; delete this seam
            # when the order becomes observable through an importable public boundary.
            with mock.patch.object(protocol.JsonMainControllerClient,
                                   "acknowledge_terminal", observe_acknowledgement):
                with self.assertRaises(protocol.JsonMainControllerPhysicalStopError) as caught:
                    getattr(robot, command)(**kwargs)
            with self.assertRaises(automation.AutomationStateError):
                getattr(robot, command)(**kwargs)
            commands = tuple(protocol.decode_message(frame).cmd for frame in serial_port.writes)
            publication = [(command, "emergency_stop_terminal", True)]
            self.assertEqual(publication_at_ack, [tuple(publication)])
            self.assertEqual(caught.exception.position, expected_position)
            self.assertEqual((published, commands, serial_port.is_open),
                             (publication, COMMANDS + (command,), False))

    def test_startup_rejection_and_stop_dispositions(self):
        boundary = protocol.SerialWriteCancellationBoundary("test pre-cancel")
        boundary.cancel()
        error, cancelled, _stops = self.connect_main(
            startup=(), error=TimeoutError, boundary=boundary)
        self.assertEqual((str(error), cancelled.writes, cancelled.is_open),
                         ("controller startup cancelled", [], False))
        error, serial_port, _stops = self.connect_main(
            startup=(rejected("identity_unavailable", "failed"),),
            error=protocol.JsonMainControllerStartupError)
        self.assertEqual(error.terminal.failure.code, "identity_unavailable")
        self.assertFalse(serial_port.is_open)
        event = protocol.Event(1, "emergency_stop", {"asserted": True})
        stop_hello = lambda request: (event, protocol.Response(
            request.id, request.cmd, "completed", STARTUP[0]))
        cases = ((STARTUP[:4] + (dict(POSITION, motion_fault="EA"),), "EA", "emergency_stop_active"),
                 (STARTUP[:4] + (dict(POSITION, motion_fault="EB"),), "EB", "emergency_stop_event"),
                 ((stop_hello,) + STARTUP[1:], None, "emergency_stop_event"))
        for startup, fault, source in cases:
            published = []
            _error, stopped, _ = self.connect_main(
                startup=startup, error=protocol.JsonMainControllerPhysicalStopError,
                stops=published)
            expected_command, expected_writes = (("hello", ("hello",)) if fault is None
                                                 else ("get_position_disposition", COMMANDS[:5]))
            writes = tuple(protocol.decode_message(frame).cmd for frame in stopped.writes)
            self.assertEqual((published, writes, stopped.is_open), ([(expected_command, source, True)], expected_writes, False))

    def test_facade_terminal_stop_dispositions(self):
        cases = ((rejected("emergency_stop_active"), "refresh_home_reference",
                  "emergency_stop_active"),
                 (dict(POSITION, motion_fault="EA"), "refresh_position",
                  "emergency_stop_active"),
                 (dict(POSITION, motion_fault="EB"), "refresh_position",
                  "emergency_stop_event"))
        for outcome, method, source in cases:
            self.facade_stop(outcome, method, source=source)

    def test_event_boundaries_and_fatal_deliveries_close(self):
        event = protocol.Event(1, "emergency_stop", {"asserted": True})
        after = lambda request: (protocol.Response(
            request.id, request.cmd, "completed", POSITION), event)
        cases = ((POSITION, (event,)), (after, ()))
        for outcome, queued in cases:
            self.facade_stop(outcome, queued=queued)
        robot, serial_port, published = self.connect_main(lambda _request: (event,))
        with self.assertRaises(automation.AutomationCleanupError):
            robot.refresh_position(timeout=0.01)
        writes = tuple(protocol.decode_message(frame).cmd for frame in serial_port.writes)
        self.assertEqual((published, writes, serial_port.is_open),
                         ([("get_position_disposition", "emergency_stop_event", True)],
                          COMMANDS + ("get_position_disposition",), False))
        fatal = (lambda request: (protocol.Response(
                     request.id, request.cmd, "accepted"),),
                 lambda _request: (protocol.Telemetry(
                     1, "joint_position", {"robot_joints_millidegrees": [0] * 6}),),
                 None)
        for outcome in fatal:
            robot, serial_port, _published = self.connect_main(outcome)
            with self.assertRaises(automation.AutomationCleanupError):
                robot.refresh_position(timeout=0.01)
            self.assertFalse(serial_port.is_open)
