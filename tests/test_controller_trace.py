import json
import unittest
from unittest.mock import patch

from ARrobots.controller_trace import (
    CONTROLLER_TRACE_AXIS_COUNT,
    CONTROLLER_TRACE_EXPECTED_SAMPLE_PERIOD_SECONDS,
    CONTROLLER_TRACE_MAXIMUM_ENCODER_MILLIDEGREES,
    CONTROLLER_TRACE_MINIMUM_ENCODER_MILLIDEGREES,
    CONTROLLER_TRACE_SOURCE,
    CONTROLLER_TRACE_MAXIMUM_BYTES,
    CONTROLLER_TRACE_MAXIMUM_LINE_BYTES,
    CONTROLLER_TRACE_MAXIMUM_SAMPLES,
    ControllerMotionProfile,
    ControllerTrace,
    ControllerTraceAnalysisError,
    ControllerTraceFormatError,
    ControllerTraceMetadata,
    ControllerTraceSample,
    ControllerTraceTerminal,
    ControllerTraceValidationError,
    analyze_controller_trace,
    decode_controller_trace,
    encode_controller_trace,
)
from ARrobots.HMI.joint_motion import (
    CONTROLLER_CAPABILITY_JOINT_TELEMETRY_V1,
    CONTROLLER_HARDWARE_ID_LENGTH,
    CONTROLLER_MAXIMUM_RAMP_PERCENT,
    CONTROLLER_SIGNED_INT_MAX,
    JOINT_TELEMETRY_AXIS_COUNT,
    JOINT_TELEMETRY_PERIOD_SECONDS,
    parse_command_timing,
)


FINGERPRINT = "sha256:" + "0" * 64


def motion_profile(**overrides):
    values = {
        "speed_mode": "p",
        "speed_value": 50,
        "acceleration_percent": 10,
        "deceleration_percent": 10,
        "ramp_percent": 25,
    }
    values.update(overrides)
    return ControllerMotionProfile(**values)


def trace_metadata(
    start_positions=(0, 0, 0, 0, 0, 0),
    target_positions=(1, 0, 0, 0, 0, 0),
    **overrides,
):
    values = {
        "controller_hardware_id": "1705B6",
        "firmware_version": "6.7.1-ar4hmi.10",
        "configuration_fingerprint": FINGERPRINT,
        "start_positions": start_positions,
        "target_positions": target_positions,
        "motion_profile": motion_profile(),
        "expected_sample_period_seconds": 0.1,
    }
    values.update(overrides)
    return ControllerTraceMetadata(**values)


def sample(elapsed_seconds, j1, *positions):
    return ControllerTraceSample(
        elapsed_seconds,
        (j1,) + positions + (0,) * (5 - len(positions)),
    )


def completed_trace(samples=(), **metadata_overrides):
    metadata = trace_metadata(**metadata_overrides)
    terminal_elapsed = samples[-1].elapsed_seconds + 0.05 if samples else 0.05
    return ControllerTrace(
        metadata,
        tuple(samples),
        ControllerTraceTerminal(
            terminal_elapsed,
            "completed",
            metadata.target_positions,
        ),
    )


def json_line(value):
    return json.dumps(value, separators=(",", ":"), sort_keys=True)


class ControllerTraceValueTests(unittest.TestCase):
    def test_trace_limits_match_the_joint_telemetry_and_profile_contract(self):
        self.assertEqual(
            CONTROLLER_TRACE_EXPECTED_SAMPLE_PERIOD_SECONDS,
            JOINT_TELEMETRY_PERIOD_SECONDS,
        )
        self.assertEqual(
            CONTROLLER_TRACE_MAXIMUM_ENCODER_MILLIDEGREES,
            CONTROLLER_SIGNED_INT_MAX,
        )
        self.assertEqual(
            CONTROLLER_TRACE_MINIMUM_ENCODER_MILLIDEGREES,
            -CONTROLLER_SIGNED_INT_MAX - 1,
        )
        self.assertEqual(
            CONTROLLER_TRACE_AXIS_COUNT,
            JOINT_TELEMETRY_AXIS_COUNT,
        )
        self.assertEqual(
            CONTROLLER_TRACE_SOURCE,
            CONTROLLER_CAPABILITY_JOINT_TELEMETRY_V1,
        )
        for speed_field, expected_mode in (
            ("Sp50", "p"),
            ("Ss2", "s"),
            ("Sm25", "m"),
        ):
            with self.subTest(speed_field=speed_field):
                timing = parse_command_timing(
                    "RJA1B2C3D4E5F6J70J80J90"
                    f"{speed_field}Ac10Dc20Rm25WNLm000000\n"
                )
                self.assertEqual(timing.mode, expected_mode)
                self.assertEqual(
                    motion_profile(
                        speed_mode=timing.mode,
                    ).speed_mode,
                    expected_mode,
                )
        self.assertEqual(
            len(trace_metadata().controller_hardware_id),
            CONTROLLER_HARDWARE_ID_LENGTH,
        )
        self.assertEqual(
            motion_profile(
                ramp_percent=CONTROLLER_MAXIMUM_RAMP_PERCENT,
            ).ramp_percent,
            CONTROLLER_MAXIMUM_RAMP_PERCENT,
        )

    def test_profile_preserves_valid_controller_inputs(self):
        percent = motion_profile()
        seconds = motion_profile(speed_mode="s", speed_value=120)
        millimeters = motion_profile(speed_mode="m", speed_value=25)

        self.assertEqual(percent.speed_value, 50.0)
        self.assertEqual(seconds.speed_value, 120.0)
        self.assertEqual(millimeters.speed_value, 25.0)

    def test_profile_rejects_invalid_controller_inputs(self):
        failures = (
            {"speed_mode": None},
            {"speed_mode": "percent"},
            {"speed_mode": "seconds"},
            {"speed_mode": "millimeters"},
            {"speed_value": 0},
            {"speed_value": 101},
            {"acceleration_percent": 0},
            {"acceleration_percent": 101},
            {"deceleration_percent": 100},
            {"acceleration_percent": 60, "deceleration_percent": 41},
            {"ramp_percent": 0},
            {"ramp_percent": 101},
            {"speed_value": True},
            {"speed_value": float("inf")},
            {"speed_mode": "s", "speed_value": 1e300},
        )
        for overrides in failures:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ControllerTraceValidationError):
                    motion_profile(**overrides)

    def test_metadata_rejects_identity_configuration_and_target_failures(self):
        failures = (
            {"controller_hardware_id": None},
            {"controller_hardware_id": "1705b6"},
            {"controller_hardware_id": "1705B60"},
            {"firmware_version": ""},
            {"firmware_version": "version\n"},
            {"configuration_fingerprint": None},
            {"configuration_fingerprint": "sha256:wrong"},
            {"start_positions": (0,) * 5},
            {"target_positions": (0,) * 5},
            {"target_positions": (0,) * 7},
            {"target_positions": (0, 0, 0, 0, 0, float("nan"))},
            {"target_positions": (0, 0, 0, 0, 0, 1e300)},
            {"motion_profile": "invalid"},
            {"expected_sample_period_seconds": 0},
            {"expected_sample_period_seconds": 0.2},
        )
        for overrides in failures:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ControllerTraceValidationError):
                    trace_metadata(**overrides)

    def test_terminal_contract_distinguishes_completed_and_failed_results(self):
        completed = ControllerTraceTerminal(
            1,
            "completed",
            (0,) * 6,
        )
        failed = ControllerTraceTerminal(1, "failed", detail="ER")
        stopped = ControllerTraceTerminal(
            1,
            "stopped",
            (0,) * 6,
            "physical stop",
        )

        self.assertIsNone(completed.detail)
        self.assertIsNone(failed.reported_positions)
        self.assertEqual(stopped.detail, "physical stop")
        failures = (
            (1, None, None, "failure"),
            (1, "unknown", None, "failure"),
            (1, "completed", None, None),
            (1, "completed", (0,) * 6, "unexpected"),
            (1, "failed", None, None),
            (1, "stopped", None, "line\nbreak"),
        )
        for elapsed, outcome, positions, detail in failures:
            with self.subTest(outcome=outcome, detail=detail):
                with self.assertRaises(ControllerTraceValidationError):
                    ControllerTraceTerminal(
                        elapsed,
                        outcome,
                        positions,
                        detail,
                    )

    def test_trace_requires_ordered_bounded_samples_before_terminal(self):
        metadata = trace_metadata()
        terminal = ControllerTraceTerminal(0.3, "completed", (0,) * 6)
        failures = (
            (sample(0.2, 0), sample(0.2, 0.1)),
            (sample(0.2, 0), sample(0.1, 0.1)),
            (sample(0.4, 0),),
            ("invalid",),
        )
        for samples in failures:
            with self.subTest(samples=samples):
                with self.assertRaises(ControllerTraceValidationError):
                    ControllerTrace(metadata, samples, terminal)

        with self.assertRaisesRegex(
            ControllerTraceValidationError,
            "maximum sample count",
        ):
            ControllerTrace(
                metadata,
                (
                    sample(index + 1, 0)
                    for index in range(CONTROLLER_TRACE_MAXIMUM_SAMPLES + 1)
                ),
                ControllerTraceTerminal(
                    CONTROLLER_TRACE_MAXIMUM_SAMPLES + 2,
                    "completed",
                    (0,) * 6,
                ),
            )

    def test_encoder_samples_require_signed_millidegree_wire_values(self):
        failures = (
            (0.0001, 0, 0, 0, 0, 0),
            (2147483.648, 0, 0, 0, 0, 0),
            (-2147483.649, 0, 0, 0, 0, 0),
        )
        for positions in failures:
            with self.subTest(positions=positions):
                with self.assertRaises(ControllerTraceValidationError):
                    ControllerTraceSample(0.1, positions)

        boundaries = ControllerTraceSample(
            0.1,
            (-2147483.648, 2147483.647, 0, 0, 0, 0),
        )
        self.assertEqual(boundaries.encoder_positions[0], -2147483.648)
        self.assertEqual(boundaries.encoder_positions[1], 2147483.647)


class ControllerTraceCodecTests(unittest.TestCase):
    def setUp(self):
        self.trace = completed_trace((
            sample(0.1, 0.01),
            sample(0.2, 0.04),
            sample(0.3, 0.09),
            sample(0.4, 0.16),
        ))

    def test_canonical_round_trip_preserves_trace(self):
        payload = encode_controller_trace(self.trace)
        decoded = decode_controller_trace(payload)

        self.assertEqual(decoded, self.trace)
        self.assertEqual(decode_controller_trace(bytearray(payload)), self.trace)
        self.assertTrue(payload.endswith(b"\n"))
        header = json.loads(payload.splitlines()[0])
        self.assertEqual(header["schema"], "ar4.controller-trace.v2")
        self.assertEqual(header["source"], "JOINT_TELEMETRY_V1")
        self.assertEqual(
            header["timebase"],
            "host-monotonic-offset-seconds",
        )
        self.assertEqual(
            header["time_origin"],
            "immediately-before-rj-write",
        )
        self.assertEqual(header["position_unit"], "degree")
        self.assertEqual(
            payload,
            encode_controller_trace(decoded),
        )

    def test_failed_and_stopped_terminal_variants_round_trip(self):
        metadata = trace_metadata()
        terminals = (
            ControllerTraceTerminal(0.1, "failed", detail="ER"),
            ControllerTraceTerminal(
                0.2,
                "stopped",
                (0,) * 6,
                "physical stop",
            ),
        )
        for terminal in terminals:
            with self.subTest(outcome=terminal.outcome):
                trace = ControllerTrace(metadata, (), terminal)
                self.assertEqual(
                    decode_controller_trace(encode_controller_trace(trace)),
                    trace,
                )

    def test_all_rj_speed_modes_round_trip(self):
        for speed_mode in ("p", "s", "m"):
            with self.subTest(speed_mode=speed_mode):
                trace = completed_trace(
                    motion_profile=motion_profile(speed_mode=speed_mode),
                )
                self.assertEqual(
                    decode_controller_trace(encode_controller_trace(trace)),
                    trace,
                )

    def test_decoder_accepts_crlf_without_relaxing_records(self):
        payload = encode_controller_trace(self.trace).replace(b"\n", b"\r\n")
        self.assertEqual(decode_controller_trace(payload), self.trace)

    def test_decoder_rejects_payload_and_json_boundary_failures(self):
        payload = encode_controller_trace(self.trace)
        header, *records = payload.splitlines()
        failures = (
            b"",
            b"\xff",
            payload + b"\x00",
            header + b"\n",
            header + b"\n\n" + b"\n".join(records) + b"\n",
            header + b"\rmalformed\n" + b"\n".join(records) + b"\n",
            b"[]\n{}\n",
            header.replace(b'"schema":', b'"schema":"duplicate","schema":')
            + b"\n"
            + b"\n".join(records)
            + b"\n",
            header.replace(b'"axis_count":6', b'"axis_count":NaN')
            + b"\n"
            + b"\n".join(records)
            + b"\n",
        )
        for invalid in failures:
            with self.subTest(payload=invalid[:80]):
                with self.assertRaises(ControllerTraceFormatError):
                    decode_controller_trace(invalid)
        with self.assertRaises(ControllerTraceFormatError):
            decode_controller_trace("not bytes")

        oversized = header + b"\n" + b" " * (
            CONTROLLER_TRACE_MAXIMUM_LINE_BYTES + 1
        ) + b"\n" + records[-1] + b"\n"
        with self.assertRaisesRegex(ControllerTraceFormatError, "line limit"):
            decode_controller_trace(oversized)

    def test_decoder_rejects_schema_and_record_shape_changes(self):
        payload = encode_controller_trace(self.trace)
        records = [json.loads(line) for line in payload.splitlines()]
        mutations = []

        wrong_schema = [dict(record) for record in records]
        wrong_schema[0]["schema"] = "ar4.controller-trace.v1"
        mutations.append(wrong_schema)
        float_axis_count = [dict(record) for record in records]
        float_axis_count[0]["axis_count"] = 6.0
        mutations.append(float_axis_count)
        missing_header = [dict(record) for record in records]
        del missing_header[0]["firmware_version"]
        mutations.append(missing_header)
        missing_time_origin = [dict(record) for record in records]
        del missing_time_origin[0]["time_origin"]
        mutations.append(missing_time_origin)
        extra_sample = [dict(record) for record in records]
        extra_sample[1]["extra"] = True
        mutations.append(extra_sample)
        wrong_sample_kind = [dict(record) for record in records]
        wrong_sample_kind[1]["kind"] = "encoder"
        mutations.append(wrong_sample_kind)
        wrong_terminal = [dict(record) for record in records]
        wrong_terminal[-1]["kind"] = "sample"
        mutations.append(wrong_terminal)

        for mutated in mutations:
            with self.subTest(mutated=mutated):
                malformed = (
                    "\n".join(json_line(record) for record in mutated) + "\n"
                ).encode("utf-8")
                with self.assertRaises(ControllerTraceFormatError):
                    decode_controller_trace(malformed)

    def test_decoder_rejects_nested_duplicate_profile_key(self):
        payload = encode_controller_trace(self.trace)
        header, *records = payload.splitlines()
        header = header.replace(
            b'"speed_mode":"p"',
            b'"speed_mode":"p","speed_mode":"s"',
        )
        with self.assertRaisesRegex(ControllerTraceFormatError, "duplicated"):
            decode_controller_trace(
                header + b"\n" + b"\n".join(records) + b"\n"
            )

    def test_decoder_rejects_value_sequence_and_limit_failures(self):
        payload = encode_controller_trace(self.trace)
        records = [json.loads(line) for line in payload.splitlines()]
        failures = []
        huge_number = [dict(record) for record in records]
        huge_number[1]["elapsed_seconds"] = 10 ** 1000
        failures.append(huge_number)
        unordered = [dict(record) for record in records]
        unordered[2]["elapsed_seconds"] = unordered[1]["elapsed_seconds"]
        failures.append(unordered)
        after_terminal = [dict(record) for record in records]
        after_terminal[-1]["elapsed_seconds"] = 0.05
        failures.append(after_terminal)
        bad_positions = [dict(record) for record in records]
        bad_positions[1]["encoder_positions"] = [0] * 5
        failures.append(bad_positions)

        for case_index, records_with_failure in enumerate(failures):
            with self.subTest(case_index=case_index):
                malformed = (
                    "\n".join(
                        json_line(record) for record in records_with_failure
                    )
                    + "\n"
                ).encode("utf-8")
                with self.assertRaises(ControllerTraceFormatError):
                    decode_controller_trace(malformed)

        with self.assertRaises(ControllerTraceFormatError):
            decode_controller_trace(payload, maximum_samples=3)
        with self.assertRaises(ControllerTraceFormatError):
            decode_controller_trace(payload, maximum_bytes=len(payload) - 1)
        with self.assertRaises(ControllerTraceFormatError):
            decode_controller_trace(
                payload,
                maximum_bytes=CONTROLLER_TRACE_MAXIMUM_BYTES + 1,
            )
        for invalid_limit in (0, True, 1.5):
            with self.subTest(invalid_limit=invalid_limit):
                with self.assertRaises(ControllerTraceFormatError):
                    decode_controller_trace(
                        payload,
                        maximum_samples=invalid_limit,
                    )

    def test_decoder_wraps_recursive_json_failure(self):
        payload = encode_controller_trace(self.trace)
        with patch(
            "ARrobots.controller_trace.json.loads",
            side_effect=RecursionError("depth"),
        ):
            with self.assertRaisesRegex(
                ControllerTraceFormatError,
                "invalid JSON",
            ):
                decode_controller_trace(payload)

    def test_encoder_rejects_wrong_type(self):
        with self.assertRaises(ControllerTraceFormatError):
            encode_controller_trace("invalid")


class ControllerTraceAnalysisTests(unittest.TestCase):
    def test_unavailable_derivative_metrics_remain_none(self):
        one_sample = analyze_controller_trace(completed_trace((
            sample(0.1, 0.1),
        ))).joints[0]
        self.assertIsNone(one_sample.peak_absolute_speed_degrees_per_second)
        self.assertIsNone(
            one_sample.peak_speed_toward_target_degrees_per_second
        )
        self.assertIsNone(one_sample.peak_reverse_speed_degrees_per_second)
        self.assertIsNone(
            one_sample.peak_acceleration_toward_target_degrees_per_second_squared
        )
        self.assertIsNone(
            one_sample.peak_deceleration_degrees_per_second_squared
        )

        two_samples = analyze_controller_trace(completed_trace((
            sample(0.1, 0.1),
            sample(0.2, 0.2),
        ))).joints[0]
        self.assertIsNotNone(
            two_samples.peak_speed_toward_target_degrees_per_second
        )
        self.assertIsNotNone(
            two_samples.peak_reverse_speed_degrees_per_second
        )
        self.assertIsNone(
            two_samples.peak_acceleration_toward_target_degrees_per_second_squared
        )
        self.assertIsNone(
            two_samples.peak_deceleration_degrees_per_second_squared
        )

    def test_uniform_quadratic_trace_reports_profile_derivatives(self):
        trace = completed_trace((
            sample(0.1, 0.01),
            sample(0.2, 0.04),
            sample(0.3, 0.09),
            sample(0.4, 0.16),
        ))

        analysis = analyze_controller_trace(trace)
        joint = analysis.joints[0]

        self.assertTrue(analysis.profile_analysis_eligible)
        self.assertEqual(analysis.blocking_reasons, ())
        self.assertEqual(analysis.cadence.sample_count, 4)
        self.assertEqual(analysis.cadence.cadence_gap_count, 0)
        self.assertAlmostEqual(analysis.cadence.mean_interval_seconds, 0.1)
        self.assertEqual(joint.commanded_direction, 1)
        self.assertEqual(joint.commanded_start_position_degrees, 0.0)
        self.assertEqual(joint.commanded_displacement_degrees, 1.0)
        self.assertAlmostEqual(joint.initial_encoder_error_degrees, 0.01)
        self.assertAlmostEqual(joint.encoder_displacement_degrees, 0.15)
        self.assertAlmostEqual(joint.final_encoder_error_degrees, -0.84)
        self.assertAlmostEqual(joint.terminal_reported_error_degrees, 0.0)
        self.assertEqual(joint.velocity_sample_count, 3)
        self.assertEqual(joint.acceleration_sample_count, 2)
        self.assertEqual(joint.jerk_sample_count, 1)
        self.assertAlmostEqual(
            joint.peak_absolute_speed_degrees_per_second,
            0.7,
        )
        self.assertAlmostEqual(joint.peak_absolute_speed_at_seconds, 0.35)
        self.assertAlmostEqual(
            joint.peak_acceleration_toward_target_degrees_per_second_squared,
            2.0,
        )
        self.assertAlmostEqual(
            joint.peak_deceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertAlmostEqual(
            joint.peak_absolute_jerk_degrees_per_second_cubed,
            0.0,
        )
        self.assertEqual(joint.overshoot_degrees, 0.0)

    def test_nonuniform_timestamps_do_not_assume_nominal_cadence(self):
        trace = completed_trace(
            (
                sample(0.1, 0.01),
                sample(0.2, 0.04),
                sample(0.4, 0.16),
                sample(0.7, 0.49),
            ),
        )

        analysis = analyze_controller_trace(trace)
        joint = analysis.joints[0]

        self.assertEqual(analysis.cadence.cadence_gap_count, 2)
        self.assertAlmostEqual(
            joint.peak_acceleration_toward_target_degrees_per_second_squared,
            2.0,
        )
        self.assertAlmostEqual(
            joint.peak_absolute_jerk_degrees_per_second_cubed,
            0.0,
        )

    def test_reverse_move_reports_deceleration_reverse_motion_and_overshoot(self):
        trace = completed_trace(
            (
                sample(0.1, 1.0),
                sample(0.2, 0.8),
                sample(0.3, 0.55),
                sample(0.4, 0.45),
            ),
            start_positions=(1, 0, 0, 0, 0, 0),
            target_positions=(0.5, 0, 0, 0, 0, 0),
        )

        joint = analyze_controller_trace(trace).joints[0]

        self.assertEqual(joint.commanded_direction, -1)
        self.assertAlmostEqual(joint.overshoot_degrees, 0.05)
        self.assertGreater(
            joint.peak_deceleration_degrees_per_second_squared,
            0,
        )
        self.assertEqual(joint.peak_reverse_speed_degrees_per_second, 0.0)

    def test_direction_uses_commanded_start_after_first_sample_crosses_target(self):
        trace = completed_trace(
            (
                sample(0.1, 0.2),
                sample(0.2, 0.15),
                sample(0.3, 0.1),
                sample(0.4, 0.1),
            ),
            start_positions=(0, 0, 0, 0, 0, 0),
            target_positions=(0.1, 0, 0, 0, 0, 0),
        )

        joint = analyze_controller_trace(trace).joints[0]

        self.assertEqual(joint.commanded_direction, 1)
        self.assertAlmostEqual(joint.commanded_displacement_degrees, 0.1)
        self.assertAlmostEqual(joint.initial_encoder_error_degrees, 0.2)
        self.assertAlmostEqual(joint.overshoot_degrees, 0.1)

    def test_failed_empty_trace_remains_explicitly_unusable(self):
        trace = ControllerTrace(
            trace_metadata(),
            (),
            ControllerTraceTerminal(0.2, "failed", detail="ER"),
        )

        analysis = analyze_controller_trace(trace)

        self.assertFalse(analysis.profile_analysis_eligible)
        self.assertEqual(analysis.cadence.sample_count, 0)
        self.assertIn(
            "controller exchange did not complete successfully",
            analysis.blocking_reasons,
        )
        self.assertIn(
            "at least four encoder samples are required for jerk estimation",
            analysis.blocking_reasons,
        )
        self.assertIsNone(
            analysis.joints[0].first_encoder_position_degrees
        )
        self.assertEqual(analysis.joints[0].commanded_direction, 1)

    def test_cadence_and_terminal_coverage_gaps_block_tuning(self):
        trace = ControllerTrace(
            trace_metadata(),
            (
                sample(0.1, 0),
                sample(0.2, 0.1),
                sample(0.5, 0.4),
                sample(0.6, 0.5),
            ),
            ControllerTraceTerminal(1.0, "completed", (1, 0, 0, 0, 0, 0)),
        )

        analysis = analyze_controller_trace(trace)

        self.assertFalse(analysis.profile_analysis_eligible)
        self.assertEqual(analysis.cadence.cadence_gap_count, 1)
        self.assertAlmostEqual(analysis.cadence.maximum_gap_multiple, 3.0)
        self.assertIn(
            "telemetry cadence contains one or more sampling gaps",
            analysis.blocking_reasons,
        )
        self.assertIn(
            "encoder sampling ended before the terminal response window",
            analysis.blocking_reasons,
        )

    def test_late_first_sample_blocks_profile_tuning(self):
        trace = ControllerTrace(
            trace_metadata(),
            (
                sample(0.2, 0),
                sample(0.3, 0.1),
                sample(0.4, 0.2),
                sample(0.5, 0.3),
            ),
            ControllerTraceTerminal(
                0.55,
                "completed",
                (1, 0, 0, 0, 0, 0),
            ),
        )

        analysis = analyze_controller_trace(trace)

        self.assertFalse(analysis.profile_analysis_eligible)
        self.assertIn(
            "encoder sampling began after the initial response window",
            analysis.blocking_reasons,
        )

    def test_stationary_trace_and_short_trace_block_tuning(self):
        trace = completed_trace(
            (
                sample(0.1, 0),
                sample(0.2, 0),
                sample(0.3, 0),
            ),
            target_positions=(0, 0, 0, 0, 0, 0),
        )

        analysis = analyze_controller_trace(trace)

        self.assertFalse(analysis.profile_analysis_eligible)
        self.assertIn(
            "trace contains no commanded J1-J6 displacement",
            analysis.blocking_reasons,
        )
        self.assertEqual(analysis.joints[0].commanded_direction, 0)
        self.assertIsNone(analysis.joints[0].overshoot_degrees)

    def test_analysis_rejects_invalid_controls_and_unrepresentable_derivative(self):
        trace = completed_trace((
            sample(0.0, -2_000_000),
            sample(5e-324, 2_000_000),
            sample(0.1, 0),
            sample(0.2, 0),
        ))
        with self.assertRaisesRegex(
            ControllerTraceAnalysisError,
            "cannot be represented",
        ):
            analyze_controller_trace(trace)

        valid = completed_trace((
            sample(0.1, 0),
            sample(0.2, 0.1),
            sample(0.3, 0.2),
            sample(0.4, 0.3),
        ))
        for invalid in (1, 0, True, float("inf"), "1.5"):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ControllerTraceAnalysisError):
                    analyze_controller_trace(valid, invalid)
        with self.assertRaises(ControllerTraceAnalysisError):
            analyze_controller_trace("invalid")


if __name__ == "__main__":
    unittest.main()
