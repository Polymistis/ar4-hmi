import math
import unittest

from ARrobots.protocol import (
    JSON_CAPABILITY_EVENT_STREAM_V1,
    JSON_CAPABILITY_PROTOCOL_V1,
    JSON_CAPABILITY_REQUEST_CORRELATION_V1,
    JSON_HELLO_MAXIMUM_CAPABILITIES,
    JSON_HELLO_MAXIMUM_TEXT_LENGTH,
    JSON_HOME_REFERENCE_AXIS_COUNT,
    JSON_POSITION_CARTESIAN_ORIENTATION_COUNT,
    JSON_POSITION_CARTESIAN_TRANSLATION_COUNT,
    JSON_POSITION_EXTERNAL_AXIS_COUNT,
    JSON_POSITION_ROBOT_JOINT_COUNT,
    JSON_POSITION_SOURCE_CONTROLLER_STEP_STATE,
    JSON_PROTOCOL_MAXIMUM_ERROR_MESSAGE_LENGTH,
    JSON_PROTOCOL_MAXIMUM_IDENTIFIER,
    JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
    JSON_PROTOCOL_MAXIMUM_STRING_LENGTH,
    JSON_PROTOCOL_NAME,
    MAIN_CALIBRATE_COMMAND_CONTRACT,
    MAIN_CORRECT_POSITION_COMMAND_CONTRACT,
    MAIN_GET_POSITION_DISPOSITION_COMMAND_CONTRACT,
    MAIN_GET_HOME_REFERENCE_COMMAND_CONTRACT,
    MAIN_HELLO_COMMAND_CONTRACT,
    MAIN_SET_POSITION_COMMAND_CONTRACT,
    MAIN_CONFIG_EXT_AXIS_COMMAND_CONTRACT,
    MAIN_UPDATE_PARAMS_COMMAND_CONTRACT,
    MAIN_ZERO_J7_COMMAND_CONTRACT,
    MAIN_ZERO_J8_COMMAND_CONTRACT,
    MAIN_ZERO_J9_COMMAND_CONTRACT,
    JsonCommandSchemaError,
    JsonHelloFirmware,
    JsonHelloProtocol,
    JsonMainControllerIdentity,
    JsonMainCalibrationResult,
    JsonMainHelloResult,
    JsonMainHomeReferenceResult,
    JsonMainPositionResult,
    ProtocolErrorFrame,
    ProtocolFailure,
    Response,
    encode_message,
    parse_main_hello_result,
    parse_main_calibration_result,
    parse_main_home_reference_result,
    parse_main_motion_position_result,
    parse_main_position_result,
    parse_main_position_disposition_result,
    validate_main_get_position_disposition_request,
    validate_main_get_position_disposition_response,
    validate_main_get_home_reference_request,
    validate_main_get_home_reference_response,
    validate_main_hello_request,
    validate_main_hello_response,
    validate_main_calibrate_request,
    validate_main_calibrate_response,
    validate_main_correct_position_request,
    validate_main_correct_position_response,
    validate_main_config_ext_axis_request,
    validate_main_config_ext_axis_response,
    validate_main_external_axis_zero_request,
    validate_main_external_axis_zero_response,
    validate_main_set_position_request,
    validate_main_set_position_response,
    validate_main_update_params_request,
    validate_main_update_params_response,
)
import ARrobots.protocol.schemas as protocol_schemas
from ARrobots.protocol.schemas import MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT


GENERIC_MAIN_REJECTION_CASES = (
    ("malformed_frame", {}),
    ("parser_resource_exhausted", {}),
    ("duplicate_field", {}),
    ("nesting_limit_exceeded", {}),
    ("container_limit_exceeded", {}),
    ("invalid_field_name", {}),
    ("invalid_string_value", {}),
    ("invalid_number", {}),
    ("invalid_envelope", {}),
    ("emergency_stop_active", {}),
    ("live_motion_active", {}),
    ("unsupported_version", {"field": "v"}),
    ("unsupported_message_type", {"field": "type"}),
    ("invalid_parameter", {"field": "params"}),
)
MAPPER_ONLY_REJECTION_CASES = (
    ("unknown_command", {"field": "cmd"}),
    ("invalid_request_identifier", {"field": "id"}),
    ("invalid_command_name", {"field": "cmd"}),
)
MAIN_REJECTION_RESPONSE_VALIDATORS = (
    ("hello", validate_main_hello_response),
    ("get_home_reference", validate_main_get_home_reference_response),
    (
        "get_position_disposition",
        validate_main_get_position_disposition_response,
    ),
    ("set_position", validate_main_set_position_response),
    ("update_params", validate_main_update_params_response),
    ("config_ext_axis", validate_main_config_ext_axis_response),
    ("controller_wait", MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT.response_validator),
    (
        "modbus_write_coil",
        protocol_schemas.MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT.response_validator,
    ),
    (
        "modbus_write_register",
        protocol_schemas.MAIN_MODBUS_WRITE_REGISTER_COMMAND_CONTRACT.response_validator,
    ),
    ("calibrate", validate_main_calibrate_response),
)


def sample_main_hello_result():
    return {
        "device": "main_controller",
        "firmware": {
            "name": "AR4 Teensy",
            "version": "6.7.1-ar4hmi.38",
            "build": "tracked",
        },
        "protocol": {
            "name": "ar4_json",
            "version": 1,
            "max_payload_bytes": 4094,
        },
        "capabilities": [
            "JSON_PROTOCOL_V1",
            "REQUEST_CORRELATION_V1",
            "EVENT_STREAM_V1",
        ],
        "commands": list(protocol_schemas.JSON_MAIN_COMMAND_MANIFEST),
        "session_id": "00112233445566778899AABBCCDDEEFF",
        "identity": {
            "controller_hardware_id": "1705B6",
            "driver_model": "Teensy 4.1",
            "robot_model": 'AR4"Model',
            "robot_version": "MK3",
            "serial_number": "SN\\A42",
            "asset_tag": "Lab",
        },
    }


def sample_main_position_result():
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
            -(2 ** 31),
            2 ** 31 - 1,
        ],
    }


def sample_main_position_disposition_result():
    result = sample_main_position_result()
    result.update({
        "controller_debug": "12.5",
        "motion_fault": "EC000001",
        "speed_limited": True,
    })
    return result


def sample_main_calibration_result():
    return {
        "controller_debug": "",
        "position": sample_main_position_result(),
        "speed_limited": False,
    }


def sample_main_home_reference_result():
    return {
        "positions_millidegrees": [170000, 0, -88000],
        "valid": [True, False, True],
    }


class MainRejectedResponseSchemaTests(unittest.TestCase):
    def test_shared_response_validators_accept_generic_pre_dispatch_rejections(self):
        for command, validator in MAIN_REJECTION_RESPONSE_VALIDATORS:
            for error_code, details in GENERIC_MAIN_REJECTION_CASES:
                with self.subTest(command=command, error_code=error_code):
                    response = Response(
                        41,
                        command,
                        "rejected",
                        error=ProtocolFailure(
                            error_code,
                            "request rejected before dispatch",
                            details,
                        ),
                    )
                    self.assertIsNone(validator(response))

    def test_generic_pre_dispatch_rejection_details_are_exact(self):
        for command, validator in MAIN_REJECTION_RESPONSE_VALIDATORS:
            for error_code, details in GENERIC_MAIN_REJECTION_CASES:
                invalid_details = dict(details)
                if invalid_details:
                    invalid_details["field"] = "unexpected"
                else:
                    invalid_details["unexpected"] = "field"
                with self.subTest(command=command, error_code=error_code):
                    response = Response(
                        41,
                        command,
                        "rejected",
                        error=ProtocolFailure(
                            error_code,
                            "request rejected before dispatch",
                            invalid_details,
                        ),
                    )
                    with self.assertRaises(JsonCommandSchemaError):
                        validator(response)

    def test_command_failure_codes_cannot_become_rejections(self):
        cases = (
            (
                "hello",
                "identity_unavailable",
                validate_main_hello_response,
            ),
            (
                "get_home_reference",
                "home_reference_unavailable",
                validate_main_get_home_reference_response,
            ),
            (
                "get_position_disposition",
                "controller_alarm",
                validate_main_get_position_disposition_response,
            ),
            (
                "get_position_disposition",
                "position_unavailable",
                validate_main_get_position_disposition_response,
            ),
        )
        for command, error_code, validator in cases:
            with self.subTest(command=command, error_code=error_code):
                response = Response(
                    41,
                    command,
                    "rejected",
                    error=ProtocolFailure(error_code, "request rejected"),
                )
                with self.assertRaises(JsonCommandSchemaError):
                    validator(response)

    def test_mapper_only_codes_cannot_cross_command_response_boundaries(self):
        for command, validator in MAIN_REJECTION_RESPONSE_VALIDATORS:
            for error_code, details in MAPPER_ONLY_REJECTION_CASES:
                with self.subTest(command=command, error_code=error_code):
                    response = Response(
                        41,
                        command,
                        "rejected",
                        error=ProtocolFailure(
                            error_code,
                            "request rejected before dispatch",
                            details,
                        ),
                    )
                    with self.assertRaises(JsonCommandSchemaError):
                        validator(response)


class MainHelloSchemaTests(unittest.TestCase):
    def test_schema_constants_match_the_wire_contract(self):
        self.assertEqual(JSON_PROTOCOL_NAME, "ar4_json")
        self.assertEqual(JSON_HELLO_MAXIMUM_CAPABILITIES, 32)
        self.assertEqual(JSON_HELLO_MAXIMUM_TEXT_LENGTH, 31)
        self.assertEqual(
            {
                JSON_CAPABILITY_PROTOCOL_V1,
                JSON_CAPABILITY_REQUEST_CORRELATION_V1,
                JSON_CAPABILITY_EVENT_STREAM_V1,
            },
            {
                "JSON_PROTOCOL_V1",
                "REQUEST_CORRELATION_V1",
                "EVENT_STREAM_V1",
            },
        )
        self.assertEqual(MAIN_HELLO_COMMAND_CONTRACT.name, "hello")

    def test_completed_result_parses_to_immutable_typed_identity(self):
        parsed = parse_main_hello_result(sample_main_hello_result())

        self.assertIsInstance(parsed, JsonMainHelloResult)
        self.assertIsInstance(parsed.firmware, JsonHelloFirmware)
        self.assertIsInstance(parsed.protocol, JsonHelloProtocol)
        self.assertIsInstance(parsed.identity, JsonMainControllerIdentity)
        self.assertEqual(parsed.device, "main_controller")
        self.assertEqual(parsed.identity.controller_hardware_id, "1705B6")
        self.assertEqual(parsed.identity.robot_model, 'AR4"Model')
        self.assertEqual(parsed.identity.serial_number, "SN\\A42")
        self.assertEqual(
            parsed.protocol.maximum_payload_bytes,
            4094,
        )
        self.assertIsInstance(parsed.capabilities, tuple)
        self.assertEqual(
            parsed.commands,
            protocol_schemas.JSON_MAIN_COMMAND_MANIFEST,
        )

    def test_request_requires_an_exact_empty_object(self):
        self.assertIsNone(validate_main_hello_request({}))
        for params in (None, [], (), {"probe": True}):
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_hello_request(params)

    def test_completed_response_uses_the_typed_result_schema(self):
        response = Response(
            41,
            "hello",
            "completed",
            sample_main_hello_result(),
        )

        self.assertIsNone(validate_main_hello_response(response))

        for invalid in (
            None,
            Response(
                42,
                "echo",
                "completed",
                sample_main_hello_result(),
            ),
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_hello_response(invalid)

    def test_result_objects_require_exact_fields(self):
        cases = []
        missing = sample_main_hello_result()
        missing.pop("identity")
        cases.append(missing)
        additional = sample_main_hello_result()
        additional["unexpected"] = 1
        cases.append(additional)
        nested_missing = sample_main_hello_result()
        nested_missing["firmware"].pop("build")
        cases.append(nested_missing)
        nested_additional = sample_main_hello_result()
        nested_additional["protocol"]["fallback"] = True
        cases.append(nested_additional)

        for result in cases:
            with self.subTest(result=result):
                with self.assertRaises(JsonCommandSchemaError):
                    parse_main_hello_result(result)

    def test_result_rejects_invalid_fixed_values_and_types(self):
        mutations = (
            ("device", "auxiliary_controller"),
            ("session_id", "0" * 31),
            ("session_id", "g" * 32),
            ("capabilities", "JSON_PROTOCOL_V1"),
            ("commands", ["hello"]),
        )
        for field, value in mutations:
            with self.subTest(field=field, value=value):
                result = sample_main_hello_result()
                result[field] = value
                with self.assertRaises(JsonCommandSchemaError):
                    parse_main_hello_result(result)

        nested_mutations = (
            ("firmware", "name", ""),
            ("firmware", "version", "x" * 32),
            ("firmware", "build", "bad\nvalue"),
            ("protocol", "name", "other"),
            ("protocol", "version", True),
            ("protocol", "max_payload_bytes", 0),
            ("protocol", "max_payload_bytes", 4095),
            ("identity", "controller_hardware_id", "1705b6"),
            ("identity", "driver_model", 4),
            ("identity", "robot_model", ""),
        )
        for group, field, value in nested_mutations:
            with self.subTest(group=group, field=field, value=value):
                result = sample_main_hello_result()
                result[group][field] = value
                with self.assertRaises(JsonCommandSchemaError):
                    parse_main_hello_result(result)

    def test_capabilities_are_bounded_unique_and_complete(self):
        cases = []
        missing = sample_main_hello_result()
        missing["capabilities"].remove("EVENT_STREAM_V1")
        cases.append(missing)
        duplicated = sample_main_hello_result()
        duplicated["capabilities"].append("JSON_PROTOCOL_V1")
        cases.append(duplicated)
        malformed = sample_main_hello_result()
        malformed["capabilities"].append("bad")
        cases.append(malformed)
        oversized = sample_main_hello_result()
        oversized["capabilities"] = [
            "JSON_PROTOCOL_V1",
            "REQUEST_CORRELATION_V1",
            "EVENT_STREAM_V1",
        ] + [f"CAPABILITY_{index}" for index in range(30)]
        cases.append(oversized)

        for result in cases:
            with self.subTest(capabilities=result["capabilities"]):
                with self.assertRaises(JsonCommandSchemaError):
                    parse_main_hello_result(result)

    def test_response_limit_covers_the_received_hello_payload(self):
        result = sample_main_hello_result()
        result["protocol"]["max_payload_bytes"] = 128
        response = Response(41, "hello", "completed", result)

        with self.assertRaisesRegex(
            JsonCommandSchemaError,
            "exceeds the advertised payload limit",
        ):
            validate_main_hello_response(response)

    def test_response_limit_accepts_exact_payload_boundary(self):
        def response_for_limit(maximum_payload_bytes):
            result = sample_main_hello_result()
            result["protocol"]["max_payload_bytes"] = maximum_payload_bytes
            return Response(41, "hello", "completed", result)

        candidate_response = response_for_limit(
            JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES
        )
        candidate_length = len(encode_message(candidate_response)) - 1
        fixed_point_candidate = response_for_limit(candidate_length)
        exact_length = len(encode_message(fixed_point_candidate)) - 1
        exact_response = response_for_limit(exact_length)

        self.assertEqual(len(encode_message(exact_response)) - 1, exact_length)
        self.assertIsNone(validate_main_hello_response(exact_response))
        self.assertGreater(exact_length, 1)
        self.assertEqual(len(str(exact_length)), len(str(exact_length - 1)))
        with self.assertRaisesRegex(
            JsonCommandSchemaError,
            "exceeds the advertised payload limit",
        ):
            validate_main_hello_response(response_for_limit(exact_length - 1))

    def test_only_documented_terminal_failures_are_accepted(self):
        valid = (
            Response(
                1,
                "hello",
                "rejected",
                error=ProtocolFailure(
                    "invalid_parameter",
                    "hello parameters must be empty",
                    {"field": "params"},
                ),
            ),
            Response(
                2,
                "hello",
                "failed",
                error=ProtocolFailure(
                    "identity_unavailable",
                    "controller identity is unavailable",
                ),
            ),
            Response(
                3,
                "hello",
                "failed",
                error=ProtocolFailure(
                    "session_unavailable",
                    "controller session identity is unavailable",
                ),
            ),
        )
        for response in valid:
            with self.subTest(status=response.status, code=response.error.code):
                self.assertIsNone(validate_main_hello_response(response))

        invalid = (
            Response(4, "hello", "accepted", {"queued": True}),
            Response(
                5,
                "hello",
                "cancelled",
                error=ProtocolFailure("stopped", "request stopped"),
            ),
            Response(
                6,
                "hello",
                "rejected",
                error=ProtocolFailure(
                    "invalid_parameter",
                    "wrong field",
                    {"field": "identity"},
                ),
            ),
            Response(
                7,
                "hello",
                "failed",
                error=ProtocolFailure("internal_error", "unexpected failure"),
            ),
        )
        for response in invalid:
            with self.subTest(status=response.status):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_hello_response(response)

    def test_public_dataclasses_reject_invalid_direct_construction(self):
        with self.assertRaises(JsonCommandSchemaError):
            JsonHelloProtocol("ar4_json", True, 4094)
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainControllerIdentity(
                "invalid",
                "Teensy 4.1",
                "AR4",
                "MK3",
                "SN42",
                "Lab",
            )
        valid_firmware = JsonHelloFirmware(
            "AR4 Teensy",
            "6.7.1-ar4hmi.38",
            "tracked",
        )
        valid_protocol = JsonHelloProtocol("ar4_json", 1, 4094)
        valid_identity = JsonMainControllerIdentity(
            "1705B6",
            "Teensy 4.1",
            "AR4",
            "MK3",
            "SN42",
            "Lab",
        )
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainHelloResult(
                valid_firmware,
                valid_protocol,
                [
                    "JSON_PROTOCOL_V1",
                    "REQUEST_CORRELATION_V1",
                    "EVENT_STREAM_V1",
                ],
                "00112233445566778899AABBCCDDEEFF",
                valid_identity,
                protocol_schemas.JSON_MAIN_COMMAND_MANIFEST,
            )
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainHelloResult(
                valid_firmware,
                valid_protocol,
                (
                    "JSON_PROTOCOL_V1",
                    "REQUEST_CORRELATION_V1",
                    "EVENT_STREAM_V1",
                ),
                "00112233445566778899AABBCCDDEEFF",
                valid_identity,
                list(protocol_schemas.JSON_MAIN_COMMAND_MANIFEST),
            )


class MainSetPositionSchemaTests(unittest.TestCase):
    def valid_params(self):
        return {
            "external_axes_milliunits": [1250, 0, -500],
            "robot_joints_millidegrees": [
                0,
                -1000,
                170000,
                -170000,
                -(2 ** 31),
                2 ** 31 - 1,
            ],
        }

    def test_contract_uses_fixed_point_axis_arrays_and_empty_result(self):
        self.assertEqual(
            MAIN_SET_POSITION_COMMAND_CONTRACT.name,
            "set_position",
        )
        self.assertIsNone(
            validate_main_set_position_request(self.valid_params())
        )
        self.assertIsNone(
            validate_main_set_position_response(
                Response(7, "set_position", "completed", {})
            )
        )

    def test_request_rejects_wrong_fields_lengths_and_integer_types(self):
        invalid = (
            None,
            {},
            {
                "robot_joints_millidegrees": [0] * 6,
            },
            {
                "external_axes_milliunits": [0] * 3,
                "robot_joints_millidegrees": [0] * 5,
            },
            {
                "external_axes_milliunits": [0, 0, 0.0],
                "robot_joints_millidegrees": [0] * 6,
            },
            {
                "external_axes_milliunits": [0] * 3,
                "robot_joints_millidegrees": [0, 0, 0, 0, 0, True],
            },
            {
                "external_axes_milliunits": [0] * 3,
                "robot_joints_millidegrees": [
                    0,
                    0,
                    0,
                    0,
                    0,
                    2 ** 31,
                ],
            },
        )
        for params in invalid:
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_set_position_request(params)

    def test_response_accepts_only_documented_terminal_shapes(self):
        for error_code in (
            "session_not_established",
            "position_not_representable",
        ):
            with self.subTest(error_code=error_code):
                self.assertIsNone(
                    validate_main_set_position_response(
                        Response(
                            7,
                            "set_position",
                            "rejected",
                            error=ProtocolFailure(
                                error_code,
                                "set-position rejected",
                            ),
                        )
                    )
                )

        invalid = (
            Response(7, "set_position", "accepted", {}),
            Response(7, "set_position", "completed", {"applied": True}),
            Response(7, "get_position_disposition", "completed", {}),
            Response(
                7,
                "set_position",
                "failed",
                error=ProtocolFailure(
                    "position_not_representable",
                    "set-position failed",
                ),
            ),
            Response(
                7,
                "set_position",
                "rejected",
                error=ProtocolFailure(
                    "position_not_representable",
                    "set-position rejected",
                    {"axis": 1},
                ),
            ),
        )
        for response in invalid:
            with self.subTest(response=response):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_set_position_response(response)


class MainConfigurationSchemaTests(unittest.TestCase):
    def valid_update_params(self):
        return {
            "calibration_directions": [1, 0, 1, 0, 0, 1, 0, 0, 0],
            "calibration_switch_active_high": [True] * 9,
            "dh_a_millimeters": [0, 64.2, 305, 0, 0, 0],
            "dh_alpha_degrees": [0, -90, 0, -90, 90, -90],
            "dh_d_millimeters": [169.77, 0, 0, 222.63, 0, 41],
            "dh_theta_degrees": [0, -90, 0, 0, 0, 180],
            "encoder_counts_per_step": [5, 5, 5, 5, 2.5, 5],
            "motor_directions": [0, 1, 1, 1, 1, 1, 1, 1, 1],
            "negative_joint_limits_degrees": [170, 42, 89, 180, 105, 180],
            "positive_joint_limits_degrees": [170, 90, 52, 180, 105, 180],
            "steps_per_degree": [88.888, 111.111, 111.111, 99.555, 43.72, 44.444],
            "tool_rotation_degrees": [0, 0, 0],
            "tool_translation_millimeters": [0, 0, 0],
        }

    def valid_config_ext_axis(self):
        return {
            "drive_rotations": [280, 280, 280],
            "motor_steps": [4000, 4000, 4000],
            "travel_units": [3450, 3450, 3450],
        }

    def test_contracts_accept_exact_representable_configuration(self):
        self.assertEqual(
            MAIN_UPDATE_PARAMS_COMMAND_CONTRACT.name,
            "update_params",
        )
        self.assertEqual(
            MAIN_CONFIG_EXT_AXIS_COMMAND_CONTRACT.name,
            "config_ext_axis",
        )
        self.assertIsNone(
            validate_main_update_params_request(self.valid_update_params())
        )
        self.assertIsNone(
            validate_main_config_ext_axis_request(
                self.valid_config_ext_axis()
            )
        )

    def test_update_params_rejects_invalid_boundary_values(self):
        invalid = []
        missing = self.valid_update_params()
        missing.pop("dh_a_millimeters")
        invalid.append(missing)
        boolean_number = self.valid_update_params()
        boolean_number["tool_translation_millimeters"] = [True, 0, 0]
        invalid.append(boolean_number)
        invalid_direction = self.valid_update_params()
        invalid_direction["motor_directions"] = [False] + [1] * 8
        invalid.append(invalid_direction)
        invalid_switch = self.valid_update_params()
        invalid_switch["calibration_switch_active_high"] = [1] * 9
        invalid.append(invalid_switch)
        invalid_encoder = self.valid_update_params()
        invalid_encoder["encoder_counts_per_step"] = [5, 5, 5, 5, 0, 5]
        invalid.append(invalid_encoder)
        invalid_limit = self.valid_update_params()
        invalid_limit["positive_joint_limits_degrees"] = [3.0e38] * 6
        invalid_limit["steps_per_degree"] = [3.0e38] * 6
        invalid.append(invalid_limit)
        invalid_angle = self.valid_update_params()
        invalid_angle["dh_theta_degrees"] = [float("nan")] * 6
        invalid.append(invalid_angle)
        underflow = self.valid_update_params()
        underflow["steps_per_degree"] = [1.0e-100] * 6
        invalid.append(underflow)

        for params in invalid:
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_update_params_request(params)

    def test_update_params_bounds_encoder_counter_conversions(self):
        load_max = self.valid_update_params()
        load_max["encoder_counts_per_step"][0] = 0.3125
        self.assertIsNone(validate_main_update_params_request(load_max))

        exact_lower = self.valid_update_params()
        exact_lower["encoder_counts_per_step"][0] = 1.0
        self.assertIsNone(validate_main_update_params_request(exact_lower))

        upper_valid = self.valid_update_params()
        upper_valid["negative_joint_limits_degrees"][0] = 0.0
        upper_valid["positive_joint_limits_degrees"][0] = 1.0
        upper_valid["steps_per_degree"][0] = 1.0
        upper_valid["encoder_counts_per_step"][0] = 2147483520.0
        self.assertIsNone(validate_main_update_params_request(upper_valid))

        upper_invalid = self.valid_update_params()
        upper_invalid["negative_joint_limits_degrees"][0] = 0.0
        upper_invalid["positive_joint_limits_degrees"][0] = 1.0
        upper_invalid["steps_per_degree"][0] = 1.0
        upper_invalid["encoder_counts_per_step"][0] = 2147483648.0
        with self.assertRaises(JsonCommandSchemaError):
            validate_main_update_params_request(upper_invalid)

        large_product = self.valid_update_params()
        large_product["encoder_counts_per_step"][0] = 1.0e30
        with self.assertRaises(JsonCommandSchemaError):
            validate_main_update_params_request(large_product)

    def test_primary_travel_must_be_positive_while_external_zero_is_valid(self):
        primary = self.valid_update_params()
        primary["negative_joint_limits_degrees"][0] = 0.0
        primary["positive_joint_limits_degrees"][0] = 0.0
        with self.assertRaises(JsonCommandSchemaError):
            validate_main_update_params_request(primary)

        external = self.valid_config_ext_axis()
        external["travel_units"][0] = 0.0
        self.assertIsNone(validate_main_config_ext_axis_request(external))

    def test_configuration_float_domain_matches_firmware_boundaries(self):
        float_maximum = 3.4028234663852886e38
        for boundary in (float_maximum, -float_maximum):
            with self.subTest(boundary=boundary):
                params = self.valid_update_params()
                params["tool_translation_millimeters"][0] = boundary
                self.assertIsNone(validate_main_update_params_request(params))

        positive_oversized = math.nextafter(float_maximum, math.inf)
        for oversized in (positive_oversized, -positive_oversized):
            with self.subTest(oversized=oversized):
                params = self.valid_update_params()
                params["tool_translation_millimeters"][0] = oversized
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_update_params_request(params)

        external = self.valid_config_ext_axis()
        external["travel_units"][0] = positive_oversized
        with self.assertRaises(JsonCommandSchemaError):
            validate_main_config_ext_axis_request(external)

    def test_config_ext_axis_rejects_invalid_calibration(self):
        invalid = (
            None,
            {},
            {
                "drive_rotations": [280, 280, 280],
                "motor_steps": [4000, 4000, 4000],
                "travel_units": [3450, 3450],
            },
            {
                "drive_rotations": [280, 0, 280],
                "motor_steps": [4000, 4000, 4000],
                "travel_units": [3450, 3450, 3450],
            },
            {
                "drive_rotations": [280, 280, 280],
                "motor_steps": [4000, 4000, 4000],
                "travel_units": [3450, -1, 3450],
            },
        )
        for params in invalid:
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_config_ext_axis_request(params)

    def test_responses_accept_only_empty_completion_and_named_rejections(self):
        validators = (
            ("update_params", validate_main_update_params_response),
            ("config_ext_axis", validate_main_config_ext_axis_response),
        )
        for command, validator in validators:
            with self.subTest(command=command, status="completed"):
                self.assertIsNone(
                    validator(Response(7, command, "completed", {}))
                )
            for error_code in (
                "configuration_not_representable",
                "session_not_established",
            ):
                with self.subTest(command=command, error_code=error_code):
                    self.assertIsNone(
                        validator(
                            Response(
                                7,
                                command,
                                "rejected",
                                error=ProtocolFailure(
                                    error_code,
                                    "configuration rejected",
                                ),
                            )
                        )
                    )
            for response in (
                Response(7, command, "completed", {"applied": True}),
                Response(7, command, "accepted", {}),
                Response(7, "hello", "completed", {}),
            ):
                with self.subTest(command=command, response=response):
                    with self.assertRaises(JsonCommandSchemaError):
                        validator(response)


class MainCorrectPositionSchemaTests(unittest.TestCase):
    def test_contract_uses_empty_request_and_typed_position_result(self):
        self.assertEqual(
            MAIN_CORRECT_POSITION_COMMAND_CONTRACT.name,
            "correct_position",
        )
        self.assertIsNone(validate_main_correct_position_request({}))
        response = Response(
            17,
            "correct_position",
            "completed",
            sample_main_position_disposition_result(),
        )
        self.assertIsNone(validate_main_correct_position_response(response))

        for params in (None, [], {"enabled": True}):
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_correct_position_request(params)

    def test_failures_require_exact_authoritative_details(self):
        position = sample_main_position_result()
        valid = (
            Response(
                17,
                "correct_position",
                "rejected",
                error=ProtocolFailure(
                    "configuration_sync_required",
                    "configuration required",
                ),
            ),
            Response(
                17,
                "correct_position",
                "failed",
                error=ProtocolFailure(
                    "position_unavailable",
                    "position unavailable",
                ),
            ),
            Response(
                17,
                "correct_position",
                "failed",
                error=ProtocolFailure(
                    "encoder_state_unavailable",
                    "encoder state unavailable",
                    {
                        "axes": [True, False, False, False, False, False],
                        "position": position,
                    },
                ),
            ),
        )
        for response in valid:
            with self.subTest(response=response):
                self.assertIsNone(validate_main_correct_position_response(response))

        invalid = (
            Response(17, "correct_position", "accepted", {}),
            Response(17, "get_position_disposition", "completed", position),
            Response(
                17,
                "correct_position",
                "failed",
                error=ProtocolFailure(
                    "encoder_state_unavailable",
                    "encoder state unavailable",
                    {"axes": [False] * 6, "position": position},
                ),
            ),
            Response(
                17,
                "correct_position",
                "failed",
                error=ProtocolFailure(
                    "encoder_state_unavailable",
                    "encoder state unavailable",
                    {"axes": [True] * 5, "position": position},
                ),
            ),
            Response(
                17,
                "correct_position",
                "failed",
                error=ProtocolFailure(
                    "encoder_state_unavailable",
                    "encoder state unavailable",
                    {
                        "axes": [True, False, False, False, False, False],
                        "position": {},
                    },
                ),
            ),
        )
        for response in invalid:
            with self.subTest(response=response):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_correct_position_response(response)


class MainExternalAxisZeroSchemaTests(unittest.TestCase):
    COMMANDS = (
        ("zero_j7", MAIN_ZERO_J7_COMMAND_CONTRACT),
        ("zero_j8", MAIN_ZERO_J8_COMMAND_CONTRACT),
        ("zero_j9", MAIN_ZERO_J9_COMMAND_CONTRACT),
    )

    def assert_invalid_response(self, response, command="zero_j7"):
        with self.assertRaises(JsonCommandSchemaError):
            validate_main_external_axis_zero_response(response, command=command)

    def test_contracts_use_empty_requests_and_authoritative_positions(self):
        result = sample_main_position_disposition_result()
        for index, (command, contract) in enumerate(self.COMMANDS):
            self.assertEqual(contract.name, command)
            self.assertIsNone(contract.request_validator({}))
            response = Response(18, command, "completed", result)
            self.assertIsNone(contract.response_validator(response))
            sibling = self.COMMANDS[(index + 1) % len(self.COMMANDS)][0]
            for invalid in (
                Response(18, sibling, "completed", result),
                Response(18, command, "accepted", {}),
            ):
                with self.assertRaises(JsonCommandSchemaError):
                    contract.response_validator(invalid)
        for params in (None, [], (), {"axis": 7}):
            with self.subTest(params=params), self.assertRaises(JsonCommandSchemaError):
                validate_main_external_axis_zero_request(params)

    def test_responses_accept_only_documented_exact_terminals(self):
        command = "zero_j7"
        rejections = GENERIC_MAIN_REJECTION_CASES + (
            ("configuration_sync_required", {}),
            ("session_not_established", {}),
            ("unsupported_command", {}),
        )
        for code, details in rejections:
            with self.subTest(code=code):
                failure = ProtocolFailure(code, "request rejected", details)
                response = Response(18, command, "rejected", error=failure)
                self.assertIsNone(
                    validate_main_external_axis_zero_response(
                        response, command=command
                    )
                )
        failure = ProtocolFailure("position_unavailable", "position unavailable")
        response = Response(18, command, "failed", error=failure)
        self.assertIsNone(
            validate_main_external_axis_zero_response(response, command=command)
        )
        for failed_code in ("unsupported_command", "controller_alarm"):
            failure = ProtocolFailure(failed_code, "invalid failure")
            response = Response(18, command, "failed", error=failure)
            with self.subTest(failed_code=failed_code):
                self.assert_invalid_response(response)
        for invalid_command in ("zero_j0", [], {}):
            with self.subTest(command=invalid_command):
                response_command = invalid_command if type(invalid_command) is str else command
                response = Response(18, response_command, "completed", sample_main_position_disposition_result())
                self.assert_invalid_response(response, command=invalid_command)
        with self.assertRaisesRegex(JsonCommandSchemaError, "external-axis-zero result fields do not match"):
            validate_main_external_axis_zero_response(Response(18, command, "completed", {}), command=command)
        exact_empty_codes = (
            "emergency_stop_active", "live_motion_active", "unsupported_command",
            "configuration_sync_required", "session_not_established",
            "position_unavailable",
        )
        for code in exact_empty_codes:
            status = "failed" if code == "position_unavailable" else "rejected"
            failure = ProtocolFailure(
                code, "invalid details", {"unexpected": True}
            )
            response = Response(18, command, status, error=failure)
            with self.subTest(code=code, details="extra"):
                self.assert_invalid_response(response)


class MainControllerWaitSchemaTests(unittest.TestCase):
    def test_contract_accepts_only_the_exact_request_and_terminal_domain(self):
        self.assertEqual(MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT.name, "controller_wait")
        request = MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT.request_validator
        for seconds in (0, 0.125, 2147483):
            with self.subTest(seconds=seconds):
                self.assertIsNone(request({"seconds": seconds}))
        invalid_requests = (
            None,
            {},
            *({"seconds": value} for value in (
                True, math.nan, math.inf, "1", -0.125, 1e-309, 5e-324, 10e-309,
                2147483.0000000005, 2147483.1, 2147484
            )),
            {"seconds": 1, "unexpected": True},
        )
        for params in invalid_requests:
            with self.subTest(params=params), self.assertRaises(JsonCommandSchemaError):
                request(params)
        response = MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT.response_validator
        self.assertIsNone(response(Response(18, "controller_wait", "completed", {})))
        for code in ("configuration_sync_required", "session_not_established",
                     "unsupported_command"):
            failure = ProtocolFailure(code, "request rejected")
            reply = Response(18, "controller_wait", "rejected", error=failure)
            self.assertIsNone(response(reply))
        emergency_stop = ProtocolFailure("emergency_stop", "wait interrupted")
        reply = Response(18, "controller_wait", "cancelled", error=emergency_stop)
        self.assertIsNone(response(reply))
        invalid_responses = (
            ("zero_j7", "completed", {}, None),
            ("controller_wait", "completed", {"seconds": 1}, None),
            ("controller_wait", "accepted", {}, None),
            ("controller_wait", "failed", None, ProtocolFailure(
                "controller_fault", "wait failed")),
            ("controller_wait", "cancelled", None, ProtocolFailure(
                "request_cancelled", "wait cancelled")),
            ("controller_wait", "cancelled", None, ProtocolFailure(
                "emergency_stop", "wait interrupted", {"unexpected": True})),
            ("controller_wait", "rejected", None,
             ProtocolFailure("unsupported_command", "unsupported", {"unexpected": True})),
        )
        for command, status, result, error in invalid_responses:
            with self.subTest(status=status), self.assertRaises(JsonCommandSchemaError):
                response(Response(18, command, status, result, error))


class MainModbusReadSchemaTests(unittest.TestCase):
    COMMANDS = (
        ("modbus_read_holding_register", 65535),
        ("modbus_read_coil", 1),
        ("modbus_read_discrete_input", 1),
        ("modbus_read_input_register", 65535),
    )

    def test_contracts_share_one_exact_request_and_scalar_value_domain(self):
        valid_request = {"slave_id": 1, "address": 0, "count": 1}
        for params in (
            valid_request,
            {"slave_id": 247, "address": 65535, "count": 1},
        ):
            with self.subTest(params=params):
                self.assertIsNone(protocol_schemas.validate_main_modbus_read_request(params))
        invalid_requests = (
            None,
            {},
            *({**valid_request, "slave_id": value} for value in (True, 0, 248)),
            *({**valid_request, "address": value} for value in (True, -1, 65536)),
            *({**valid_request, "count": value} for value in (True, 1.0, 0, 2)),
            {**valid_request, "extra": 0},
        )
        for params in invalid_requests:
            with self.subTest(params=params), self.assertRaises(JsonCommandSchemaError):
                protocol_schemas.validate_main_modbus_read_request(params)

        for index, (command, maximum_value) in enumerate(self.COMMANDS):
            contract = getattr(
                protocol_schemas, f"MAIN_{command.upper()}_COMMAND_CONTRACT"
            )
            self.assertEqual(contract.name, command)
            self.assertIsNone(contract.request_validator(valid_request))
            for value in (0, maximum_value):
                result = {"value": value}
                with self.subTest(command=command, value=value):
                    self.assertEqual(
                        protocol_schemas.parse_main_modbus_read_result(result, command=command),
                        value,
                    )
                    self.assertIsNone(
                        contract.response_validator(
                            Response(18, command, "completed", result)
                        )
                    )
            for invalid_result in (
                {}, {"value": -1}, {"value": maximum_value + 1},
                {"value": True}, {"value": 0, "extra": 0},
            ):
                with self.subTest(command=command, result=invalid_result), self.assertRaises(JsonCommandSchemaError):
                    contract.response_validator(Response(18, command, "completed", invalid_result))
            sibling = self.COMMANDS[(index + 1) % len(self.COMMANDS)][0]
            with self.assertRaises(JsonCommandSchemaError):
                contract.response_validator(Response(18, sibling, "completed", {"value": 0}))

    def test_family_accepts_only_the_shared_terminal_domain(self):
        command = "modbus_read_holding_register"
        validate_response = protocol_schemas.MAIN_MODBUS_READ_HOLDING_REGISTER_COMMAND_CONTRACT.response_validator
        rejections = GENERIC_MAIN_REJECTION_CASES + (
            ("configuration_sync_required", {}),
            ("session_not_established", {}),
            ("unsupported_command", {}),
        )
        for code, details in rejections:
            failure = ProtocolFailure(code, "request rejected", details)
            with self.subTest(status="rejected", code=code):
                self.assertIsNone(
                    validate_response(Response(18, command, "rejected", error=failure))
                )
        for status, code in (
            ("failed", "modbus_error"),
            ("cancelled", "emergency_stop"),
        ):
            failure = ProtocolFailure(code, "request terminated")
            with self.subTest(status=status, code=code):
                self.assertIsNone(
                    validate_response(Response(18, command, status, error=failure))
                )

        invalid_errors = (
            ("failed", "controller_fault", {}),
            ("failed", "modbus_error", {"unexpected": True}),
            ("cancelled", "request_cancelled", {}),
            ("cancelled", "emergency_stop", {"unexpected": True}),
            ("rejected", "unsupported_command", {"unexpected": True}),
        )
        with self.assertRaises(JsonCommandSchemaError):
            validate_response(Response(18, command, "accepted", {}))
        for status, code, details in invalid_errors:
            error = ProtocolFailure(code, "invalid terminal", details)
            with self.subTest(status=status, code=code), self.assertRaises(JsonCommandSchemaError):
                validate_response(Response(18, command, status, error=error))

        valid_result = {"value": 0}
        boundaries = (
            (protocol_schemas.parse_main_modbus_read_result, valid_result),
            (protocol_schemas.validate_main_modbus_read_response, Response(18, command, "completed", valid_result)),
        )
        for validator, value in boundaries:
            for invalid_command in ("modbus_read_unknown", [], {}):
                with self.subTest(validator=validator, command=invalid_command), self.assertRaises(JsonCommandSchemaError):
                    validator(value, command=invalid_command)


class MainModbusWriteSchemaTests(unittest.TestCase):
    COMMANDS = (("modbus_write_coil", 1), ("modbus_write_register", 65535))

    def test_contracts_accept_only_exact_target_and_value_domains(self):
        valid = {"slave_id": 1, "address": 0, "value": 0}
        for command, maximum_value in self.COMMANDS:
            contract = getattr(
                protocol_schemas, f"MAIN_{command.upper()}_COMMAND_CONTRACT"
            )
            self.assertEqual(
                (
                    contract.name, contract.acceptance_required_for_terminal,
                    contract.deadline_suspended_after_acceptance,
                ),
                (command, False, False),
            )
            boundary = {"slave_id": 247, "address": 65535, "value": maximum_value}
            for params in (valid, boundary):
                with self.subTest(command=command, params=params):
                    self.assertIsNone(contract.request_validator(params))

            invalid_requests = (
                None, {}, {**valid, "extra": 0},
                *(
                    {**valid, field: value}
                    for field, values in (
                        ("slave_id", (True, 1.0, 0, 248)),
                        ("address", (True, 0.0, -1, 65536)),
                        ("value", (True, 1.0, -1, maximum_value + 1)),
                    )
                    for value in values
                ),
            )
            for params in invalid_requests:
                with self.subTest(command=command, params=params):
                    self.assertRaises(
                        JsonCommandSchemaError, contract.request_validator, params
                    )
            self.assertIsNone(
                contract.response_validator(Response(18, command, "completed", {}))
            )

        for invalid_command in ("modbus_write_unknown", []):
            with self.assertRaises(JsonCommandSchemaError):
                protocol_schemas.validate_main_modbus_write_request(
                    valid, command=invalid_command)

    def test_family_accepts_only_empty_documented_terminals(self):
        command = "modbus_write_coil"
        contract = protocol_schemas.MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT
        validate_response = contract.response_validator
        valid_terminals = (
            ("rejected", "configuration_sync_required", "request rejected"),
            ("rejected", "session_not_established", "request rejected"),
            ("rejected", "unsupported_command", "request rejected"),
            ("failed", "modbus_error", "write failed"),
            ("cancelled", "emergency_stop", "write stopped"),
        )
        for status, code, message in valid_terminals:
            response = Response(18, command, status, error=ProtocolFailure(code, message))
            with self.subTest(status=status, code=code):
                self.assertIsNone(validate_response(response))

        invalid_terminals = (
            ("completed", {"value": 0}, None, {}),
            ("accepted", {}, None, {}),
            ("failed", None, "controller_fault", {}),
            ("failed", None, "modbus_error", {"unexpected": True}),
            ("cancelled", None, "request_cancelled", {}),
            ("cancelled", None, "emergency_stop", {"unexpected": True}),
            ("rejected", None, "unsupported_command", {"unexpected": True}),
        )
        for status, result, code, details in invalid_terminals:
            error = ProtocolFailure(code, "invalid terminal", details) if code else None
            response = Response(18, command, status, result, error)
            with self.subTest(status=status, error=error):
                self.assertRaises(JsonCommandSchemaError, validate_response, response)
        for response in ({}, Response(18, "modbus_write_register", "completed", {})):
            self.assertRaises(JsonCommandSchemaError, validate_response, response)

        for invalid_command in ("modbus_write_unknown", []):
            with self.assertRaises(JsonCommandSchemaError):
                protocol_schemas.validate_main_modbus_write_response(
                    Response(18, command, "completed", {}),
                    command=invalid_command)


class MainCalibrationSchemaTests(unittest.TestCase):
    def test_request_uses_exact_semantic_axis_arrays(self):
        self.assertEqual(MAIN_CALIBRATE_COMMAND_CONTRACT.name, "calibrate")
        valid = {
            "axes": [True, False, True, False, False, False, False, False, False],
            "offsets": [0.0] * 9,
        }
        self.assertIsNone(validate_main_calibrate_request(valid))

        invalid = (
            {**valid, "axes": [False] * 9},
            {**valid, "axes": [1] + [False] * 8},
            {**valid, "offsets": [0.0] * 8},
            {**valid, "offsets": [math.inf] + [0.0] * 8},
            {**valid, "extra": None},
        )
        for params in invalid:
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_calibrate_request(params)

    def test_completed_response_parses_typed_position_disposition(self):
        result = sample_main_calibration_result()
        self.assertIsNone(
            validate_main_calibrate_response(
                Response(17, "calibrate", "accepted", {})
            )
        )
        response = Response(17, "calibrate", "completed", result)

        self.assertIsNone(validate_main_calibrate_response(response))
        parsed = parse_main_calibration_result(result)
        self.assertIsInstance(parsed, JsonMainCalibrationResult)
        self.assertIsInstance(parsed.position, JsonMainPositionResult)
        self.assertFalse(parsed.speed_limited)
        self.assertEqual(parsed.controller_debug, "")

    def test_terminal_failures_require_exact_typed_details(self):
        position = sample_main_position_result()
        valid = (
            Response(
                17,
                "calibrate",
                "rejected",
                error=ProtocolFailure(
                    "calibration_not_representable",
                    "calibration rejected",
                    {"axes": [True] + [False] * 8},
                ),
            ),
            Response(
                17,
                "calibrate",
                "cancelled",
                error=ProtocolFailure(
                    "emergency_stop",
                    "calibration stopped",
                    {"position": position},
                ),
            ),
            Response(
                17,
                "calibrate",
                "failed",
                error=ProtocolFailure(
                    "calibration_failed",
                    "calibration failed",
                    {"position": position, "stage": "switch_release"},
                ),
            ),
            Response(
                17,
                "calibrate",
                "failed",
                error=ProtocolFailure(
                    "position_unavailable",
                    "position unavailable",
                ),
            ),
        )
        for response in valid:
            with self.subTest(status=response.status, error=response.error.code):
                self.assertIsNone(validate_main_calibrate_response(response))

        invalid = (
            Response(17, "calibrate", "accepted", {"extra": None}),
            Response(
                17, "calibrate", "completed",
                {**sample_main_calibration_result(), "extra": None},
            ),
            Response(
                17, "calibrate", "completed",
                {**sample_main_calibration_result(), "position": {}},
            ),
            Response(
                17, "calibrate", "rejected",
                error=ProtocolFailure(
                    "calibration_not_representable", "rejected",
                    {"axes": [False] * 9},
                ),
            ),
            Response(
                17, "calibrate", "cancelled",
                error=ProtocolFailure("emergency_stop", "stopped"),
            ),
            Response(
                17, "calibrate", "failed",
                error=ProtocolFailure(
                    "calibration_failed", "failed",
                    {"position": position, "stage": "unknown"},
                ),
            ),
            Response(
                17, "calibrate", "failed",
                error=ProtocolFailure(
                    "calibration_failed", "failed",
                    {"position": position, "stage": "center_move", "extra": None},
                ),
            ),
            Response(
                17, "calibrate", "failed",
                error=ProtocolFailure("unknown", "failed"),
            ),
        )
        for response in invalid:
            with self.subTest(status=response.status, error=response.error):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_calibrate_response(response)


class MainHomeReferenceSchemaTests(unittest.TestCase):
    def test_schema_constants_match_the_home_reference_contract(self):
        self.assertEqual(JSON_HOME_REFERENCE_AXIS_COUNT, 3)
        self.assertEqual(
            MAIN_GET_HOME_REFERENCE_COMMAND_CONTRACT.name,
            "get_home_reference",
        )

    def test_completed_result_parses_fixed_point_home_reference(self):
        parsed = parse_main_home_reference_result(
            sample_main_home_reference_result()
        )

        self.assertIsInstance(parsed, JsonMainHomeReferenceResult)
        self.assertEqual(parsed.valid, (True, False, True))
        self.assertEqual(
            parsed.positions_millidegrees,
            (170000, 0, -88000),
        )
        self.assertEqual(parsed.positions_degrees, (170.0, 0.0, -88.0))

    def test_request_requires_an_exact_empty_object(self):
        self.assertIsNone(validate_main_get_home_reference_request({}))
        for params in (None, [], (), {"axis": 1}):
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_get_home_reference_request(params)

    def test_result_rejects_invalid_shapes_and_inconsistent_positions(self):
        invalid_results = (
            {"valid": [True, False, True]},
            {
                "positions_millidegrees": [170000, 0, -88000],
                "valid": [True, False, True],
                "extra": None,
            },
            {
                "positions_millidegrees": [170000, 0],
                "valid": [True, False, True],
            },
            {
                "positions_millidegrees": [170000, 0, -88000],
                "valid": [1, False, True],
            },
            {
                "positions_millidegrees": [170000, 1, -88000],
                "valid": [True, False, True],
            },
            {
                "positions_millidegrees": [2 ** 31, 0, -88000],
                "valid": [True, False, True],
            },
        )
        for result in invalid_results:
            with self.subTest(result=result):
                with self.assertRaises(JsonCommandSchemaError):
                    parse_main_home_reference_result(result)

    def test_completed_and_documented_failure_responses_validate(self):
        self.assertIsNone(
            validate_main_get_home_reference_response(
                Response(
                    44,
                    "get_home_reference",
                    "completed",
                    sample_main_home_reference_result(),
                )
            )
        )
        self.assertIsNone(
            validate_main_get_home_reference_response(
                Response(
                    44,
                    "get_home_reference",
                    "failed",
                    error=ProtocolFailure(
                        "home_reference_unavailable",
                        "controller home reference is unavailable",
                    ),
                )
            )
        )

        invalid = (
            Response(
                44,
                "get_home_reference",
                "failed",
                error=ProtocolFailure("position_unavailable", "wrong domain"),
            ),
            Response(
                44,
                "get_home_reference",
                "accepted",
                {"queued": True},
            ),
            Response(
                44,
                "get_position_disposition",
                "completed",
                sample_main_home_reference_result(),
            ),
        )
        for response in invalid:
            with self.subTest(response=response):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_get_home_reference_response(response)

    def test_public_dataclass_requires_immutable_arrays(self):
        result = sample_main_home_reference_result()
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainHomeReferenceResult(
                result["valid"],
                tuple(result["positions_millidegrees"]),
            )
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainHomeReferenceResult(
                tuple(result["valid"]),
                result["positions_millidegrees"],
            )


class MainPositionSchemaTests(unittest.TestCase):
    def test_schema_constants_match_the_position_contract(self):
        self.assertEqual(JSON_POSITION_ROBOT_JOINT_COUNT, 6)
        self.assertEqual(JSON_POSITION_EXTERNAL_AXIS_COUNT, 3)
        self.assertEqual(JSON_POSITION_CARTESIAN_TRANSLATION_COUNT, 3)
        self.assertEqual(JSON_POSITION_CARTESIAN_ORIENTATION_COUNT, 3)
        self.assertEqual(
            JSON_POSITION_SOURCE_CONTROLLER_STEP_STATE,
            "controller_step_state",
        )
        self.assertEqual(
            MAIN_GET_POSITION_DISPOSITION_COMMAND_CONTRACT.name,
            "get_position_disposition",
        )

    def test_completed_result_parses_fixed_point_position(self):
        parsed = parse_main_position_result(
            sample_main_position_result()
        )

        self.assertIsInstance(parsed, JsonMainPositionResult)
        self.assertEqual(
            parsed.robot_joints_millidegrees,
            (0, -1000, 170000, -170000, -(2 ** 31), 2 ** 31 - 1),
        )
        self.assertEqual(
            parsed.robot_joints_degrees,
            (0.0, -1.0, 170.0, -170.0, -2147483.648, 2147483.647),
        )
        self.assertEqual(parsed.external_axes, (1.25, 0.0, -0.5))
        self.assertEqual(
            parsed.cartesian_translation_millimeters,
            (123.456, -789.0, 0.0),
        )
        self.assertEqual(
            parsed.orientation_degrees,
            (90.0, -45.0, 180.0),
        )
        self.assertEqual(parsed.axis_source, "controller_step_state")
        self.assertFalse(parsed.speed_limited)
        self.assertEqual(parsed.controller_debug, "")
        self.assertEqual(parsed.motion_fault, "")

    def test_disposition_result_parses_one_shot_controller_state(self):
        parsed = parse_main_position_disposition_result(
            sample_main_position_disposition_result()
        )

        self.assertTrue(parsed.speed_limited)
        self.assertEqual(parsed.controller_debug, "12.5")
        self.assertEqual(parsed.motion_fault, "EC000001")
        with self.assertRaises(JsonCommandSchemaError):
            parse_main_position_result(
                sample_main_position_disposition_result()
            )

    def test_motion_snapshot_uses_its_explicit_schema(self):
        snapshot = sample_main_position_result()

        parsed = parse_main_motion_position_result(snapshot)

        self.assertFalse(parsed.speed_limited)
        self.assertEqual(parsed.controller_debug, "")
        self.assertEqual(parsed.motion_fault, "")
        with self.assertRaises(JsonCommandSchemaError):
            parse_main_motion_position_result(
                sample_main_position_disposition_result()
            )

    def test_request_requires_an_exact_empty_object(self):
        self.assertIsNone(validate_main_get_position_disposition_request({}))
        for params in (None, [], (), {"include_encoders": True}):
            with self.subTest(params=params):
                with self.assertRaises(JsonCommandSchemaError):
                    validate_main_get_position_disposition_request(params)

    def test_result_requires_exact_fields_lengths_and_integer_types(self):
        cases = []
        missing = sample_main_position_result()
        missing.pop("axis_source")
        cases.append(missing)
        additional = sample_main_position_result()
        additional["fault"] = None
        cases.append(additional)
        short = sample_main_position_result()
        short["robot_joints_millidegrees"] = [0] * 5
        cases.append(short)
        boolean = sample_main_position_result()
        boolean["external_axes_milliunits"][0] = True
        cases.append(boolean)
        floating = sample_main_position_result()
        floating["cartesian_micrometers"][0] = 1.0
        cases.append(floating)
        overflow = sample_main_position_result()
        overflow["orientation_millidegrees"][0] = 2 ** 31
        cases.append(overflow)
        underflow = sample_main_position_result()
        underflow["external_axes_milliunits"][0] = -(2 ** 31) - 1
        cases.append(underflow)
        wrong_source = sample_main_position_result()
        wrong_source["axis_source"] = "encoders"
        cases.append(wrong_source)
        invalid_speed = sample_main_position_disposition_result()
        invalid_speed["speed_limited"] = 1
        cases.append(invalid_speed)
        invalid_debug = sample_main_position_disposition_result()
        invalid_debug["controller_debug"] = "verbose"
        cases.append(invalid_debug)
        invalid_fault = sample_main_position_disposition_result()
        invalid_fault["motion_fault"] = "EC00001"
        cases.append(invalid_fault)

        for result in cases:
            with self.subTest(result=result):
                with self.assertRaises(JsonCommandSchemaError):
                    (
                        parse_main_position_disposition_result(result)
                        if "speed_limited" in result
                        else parse_main_position_result(result)
                    )

    def test_completed_and_documented_failure_responses_validate(self):
        disposition = Response(
            45,
            "get_position_disposition",
            "completed",
            sample_main_position_disposition_result(),
        )
        alarm = Response(
            46,
            "get_position_disposition",
            "failed",
            error=ProtocolFailure(
                "controller_alarm",
                "controller position is blocked by a motion alarm",
                {"controller_alarm": "EL000001000"},
            ),
        )
        unbound_disposition = Response(
            47,
            "get_position_disposition",
            "rejected",
            error=ProtocolFailure(
                "session_not_established",
                "hello must complete before position disposition",
            ),
        )
        for response, validator in (
            (disposition, validate_main_get_position_disposition_response),
            (alarm, validate_main_get_position_disposition_response),
            (
                unbound_disposition,
                validate_main_get_position_disposition_response,
            ),
        ):
            with self.subTest(status=response.status, cmd=response.cmd):
                self.assertIsNone(
                    validator(response)
                )

    def test_undocumented_position_responses_are_rejected(self):
        invalid_disposition = Response(
            51,
            "get_position_disposition",
            "failed",
            error=ProtocolFailure(
                "controller_alarm",
                "controller position is blocked by a motion alarm",
                {"controller_alarm": "EL001"},
            ),
        )
        with self.assertRaises(JsonCommandSchemaError):
            validate_main_get_position_disposition_response(
                invalid_disposition
            )

    def test_public_position_dataclass_requires_immutable_arrays(self):
        result = sample_main_position_result()
        with self.assertRaises(JsonCommandSchemaError):
            JsonMainPositionResult(
                result["robot_joints_millidegrees"],
                tuple(result["external_axes_milliunits"]),
                tuple(result["cartesian_micrometers"]),
                tuple(result["orientation_millidegrees"]),
            )


if __name__ == "__main__":
    unittest.main()
