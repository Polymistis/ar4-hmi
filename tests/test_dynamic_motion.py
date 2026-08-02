from dataclasses import FrozenInstanceError, replace
from decimal import Decimal
import json
import math
import unittest
from unittest.mock import patch

from ARrobots.dynamic_motion import (
    OBSERVATION_REPLAY_MAXIMUM_BYTES,
    PREDICTION_TIMESTAMP_TOLERANCE_ULPS,
    AxisStateCovariance,
    ConstantVelocityEstimator,
    ConstantVelocityEstimatorConfig,
    ConstantVelocityPredictor,
    DiagonalCovariance3,
    EstimatorUpdate,
    EstimatorUpdateStatus,
    FutureObservationError,
    MotionEstimate,
    ObservationReplay,
    ObservationValidationError,
    OutOfOrderObservationError,
    PositionObservation,
    PredictionError,
    ReplayFormatError,
    ReplayObservation,
    StaleObservationError,
    StateCovariance3,
    Vector3,
    decode_observation_replay,
    encode_observation_replay,
    prediction_timestamp_tolerance,
)


def observation(
    timestamp,
    position=(0.0, 0.0, 0.0),
    variance=(0.01, 0.01, 0.01),
    frame_id="table",
):
    return PositionObservation(
        timestamp_seconds=timestamp,
        frame_id=frame_id,
        position=Vector3.from_sequence(position, "position"),
        position_variance=DiagonalCovariance3.from_sequence(
            variance,
            "position_variance",
        ),
    )


def estimator_config(**overrides):
    values = {
        "frame_id": "table",
        "maximum_observation_age_seconds": 0.2,
        "minimum_sample_interval_seconds": 0.01,
        "maximum_sample_interval_seconds": 0.5,
        "maximum_future_skew_seconds": 0.005,
    }
    values.update(overrides)
    return ConstantVelocityEstimatorConfig(**values)


def replay_sample(timestamp, position, received_at=None, frame_id="table"):
    return ReplayObservation(
        observation(timestamp, position, frame_id=frame_id),
        timestamp + 0.01 if received_at is None else received_at,
    )


def json_line(value):
    return json.dumps(value, separators=(",", ":"), sort_keys=True)


def replay_payload(records):
    header = {
        "position_unit": "millimeter",
        "schema": "ar4.observation-replay.v1",
        "timebase": "monotonic-seconds",
    }
    return (
        "\n".join(json_line(value) for value in (header, *records))
        + "\n"
    ).encode("utf-8")


def replay_record(
    timestamp,
    position=(0.0, 0.0, 0.0),
    variance=(0.01, 0.01, 0.01),
    received_at=None,
    frame_id="table",
):
    return {
        "frame_id": frame_id,
        "position": list(position),
        "position_variance": list(variance),
        "received_at_seconds": (
            timestamp + 0.01 if received_at is None else received_at
        ),
        "timestamp_seconds": timestamp,
    }


class DynamicMotionValueTests(unittest.TestCase):
    def test_vector_and_covariance_normalize_finite_numeric_values(self):
        vector = Vector3(1, Decimal("2.5"), -3.0)
        covariance = DiagonalCovariance3(-0.0, Decimal("0.25"), 1)

        self.assertEqual(vector.components(), (1.0, 2.5, -3.0))
        self.assertEqual(covariance.components(), (0.0, 0.25, 1.0))
        self.assertEqual(math.copysign(1.0, covariance.x_variance), 1.0)
        with self.assertRaises(FrozenInstanceError):
            vector.x = 4.0

    def test_vector_and_covariance_reject_invalid_domains(self):
        class BrokenIterable:
            def __iter__(self):
                raise RuntimeError("iteration failed")

        invalid_vectors = (
            (True, 0, 0),
            (math.nan, 0, 0),
            (math.inf, 0, 0),
            (Decimal("1e-9999"), 0, 0),
        )
        for values in invalid_vectors:
            with self.subTest(values=values):
                with self.assertRaises(ObservationValidationError):
                    Vector3(*values)

        with self.assertRaisesRegex(
            ObservationValidationError,
            "must contain 3 values",
        ):
            Vector3.from_sequence((1, 2), "position")
        with self.assertRaisesRegex(
            ObservationValidationError,
            "must be non-negative",
        ):
            DiagonalCovariance3(0.0, -0.1, 0.0)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "must be a sequence",
        ):
            Vector3.from_sequence(BrokenIterable(), "position")

    def test_axis_covariance_requires_positive_semidefinite_input(self):
        valid = AxisStateCovariance(4.0, 9.0, -6.0)
        self.assertEqual(valid.position_velocity_covariance, -6.0)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "positive semidefinite",
        ):
            AxisStateCovariance(1.0, 1.0, 1.01)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "positive semidefinite",
        ):
            AxisStateCovariance(0.0, 0.0, 1e-13)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "positive semidefinite",
        ):
            AxisStateCovariance(0.0, 0.0, math.ulp(0.0))

    def test_observation_requires_typed_values_and_bounded_frame_id(self):
        valid = observation(1.0, frame_id="camera/table_1")
        self.assertEqual(valid.frame_id, "camera/table_1")
        for frame_id in ("", " table", "table frame", "x" * 65, 7):
            with self.subTest(frame_id=frame_id):
                with self.assertRaises(ObservationValidationError):
                    observation(1.0, frame_id=frame_id)
        with self.assertRaises(ObservationValidationError):
            PositionObservation(1.0, "table", (0, 0, 0), valid.position_variance)
        with self.assertRaises(ObservationValidationError):
            PositionObservation(1.0, "table", valid.position, (0, 0, 0))


class ConstantVelocityEstimatorTests(unittest.TestCase):
    def test_two_observations_produce_velocity_and_correlated_covariance(self):
        estimator = ConstantVelocityEstimator(estimator_config())
        first = observation(
            1.0,
            (1.0, 2.0, 3.0),
            (0.01, 0.04, 0.09),
        )
        second = observation(
            1.5,
            (2.0, 1.0, 3.5),
            (0.04, 0.09, 0.16),
        )

        baseline = estimator.add_observation(first, 1.01)
        update = estimator.add_observation(second, 1.51)

        self.assertIs(
            baseline.status,
            EstimatorUpdateStatus.BASELINE_ACCEPTED,
        )
        self.assertIsNone(baseline.estimate)
        self.assertIs(
            update.status,
            EstimatorUpdateStatus.ESTIMATE_UPDATED,
        )
        self.assertEqual(update.estimate.velocity, Vector3(2.0, -2.0, 1.0))
        self.assertAlmostEqual(
            update.estimate.covariance.x_axis.velocity_variance,
            0.2,
        )
        self.assertAlmostEqual(
            update.estimate.covariance.x_axis.position_velocity_covariance,
            0.08,
        )
        self.assertIs(estimator.estimate, update.estimate)

    def test_latest_pair_replaces_prior_velocity_estimate(self):
        estimator = ConstantVelocityEstimator(estimator_config())
        estimator.add_observation(observation(1.0, (0, 0, 0)), 1.01)
        estimator.add_observation(observation(1.1, (1, 0, 0)), 1.11)

        update = estimator.add_observation(
            observation(1.2, (1.5, 1.0, 0)),
            1.21,
        )

        self.assertAlmostEqual(update.estimate.velocity.x, 5.0)
        self.assertAlmostEqual(update.estimate.velocity.y, 10.0)
        self.assertEqual(update.estimate.velocity.z, 0.0)
        self.assertAlmostEqual(update.estimate.sample_interval_seconds, 0.1)

    def test_large_observation_gap_resets_the_baseline_explicitly(self):
        estimator = ConstantVelocityEstimator(estimator_config())
        estimator.add_observation(observation(1.0), 1.01)

        update = estimator.add_observation(observation(1.6), 1.61)

        self.assertIs(update.status, EstimatorUpdateStatus.BASELINE_RESET)
        self.assertIsNone(update.estimate)
        self.assertIs(estimator.last_observation, update.observation)
        self.assertIsNone(estimator.estimate)

    def test_rejected_observations_do_not_mutate_estimator_state(self):
        baseline = observation(1.0)
        accepted = observation(1.1, (0.5, 0, 0))
        cases = (
            (
                OutOfOrderObservationError,
                observation(1.1, (1, 0, 0)),
                1.12,
            ),
            (
                ObservationValidationError,
                observation(1.105, (1, 0, 0)),
                1.115,
            ),
            (
                StaleObservationError,
                observation(1.2, (1, 0, 0)),
                1.41,
            ),
            (
                FutureObservationError,
                observation(1.2, (1, 0, 0)),
                1.19,
            ),
            (
                ObservationValidationError,
                observation(1.2, (1, 0, 0), frame_id="camera"),
                1.21,
            ),
            (
                OutOfOrderObservationError,
                observation(1.101, (1, 0, 0)),
                1.10,
            ),
        )
        for error_type, rejected, received_at in cases:
            with self.subTest(error_type=error_type.__name__):
                estimator = ConstantVelocityEstimator(estimator_config())
                estimator.add_observation(baseline, 1.01)
                estimator.add_observation(accepted, 1.11)
                prior_estimate = estimator.estimate
                with self.assertRaises(error_type):
                    estimator.add_observation(rejected, received_at)
                self.assertIs(estimator.last_observation, accepted)
                self.assertEqual(estimator.last_received_at_seconds, 1.11)
                self.assertIs(estimator.estimate, prior_estimate)

    def test_estimator_update_rejects_every_status_payload_mismatch(self):
        current = observation(1.0)
        for status in (
            EstimatorUpdateStatus.BASELINE_ACCEPTED,
            EstimatorUpdateStatus.BASELINE_RESET,
        ):
            with self.subTest(status=status):
                with self.assertRaises(ObservationValidationError):
                    EstimatorUpdate(status, current, object())
        for invalid_estimate in (None, object()):
            with self.subTest(invalid_estimate=invalid_estimate):
                with self.assertRaises(ObservationValidationError):
                    EstimatorUpdate(
                        EstimatorUpdateStatus.ESTIMATE_UPDATED,
                        current,
                        invalid_estimate,
                    )

    def test_future_skew_boundary_and_reset_are_deterministic(self):
        estimator = ConstantVelocityEstimator(estimator_config())
        accepted = observation(1.0)
        update = estimator.add_observation(accepted, 0.995)
        self.assertIs(update.status, EstimatorUpdateStatus.BASELINE_ACCEPTED)

        estimator.reset()

        self.assertIsNone(estimator.last_observation)
        self.assertIsNone(estimator.last_received_at_seconds)
        self.assertIsNone(estimator.estimate)

    def test_estimator_config_rejects_invalid_bounds(self):
        invalid = (
            {"maximum_observation_age_seconds": -0.1},
            {"minimum_sample_interval_seconds": 0.0},
            {"maximum_sample_interval_seconds": 0.0},
            {
                "minimum_sample_interval_seconds": 0.2,
                "maximum_sample_interval_seconds": 0.1,
            },
            {"maximum_future_skew_seconds": -0.1},
        )
        for overrides in invalid:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ObservationValidationError):
                    estimator_config(**overrides)
        config = estimator_config()
        estimator = ConstantVelocityEstimator(config)
        self.assertIs(estimator.config, config)
        with self.assertRaises(ObservationValidationError):
            ConstantVelocityEstimator(object())


class ConstantVelocityPredictorTests(unittest.TestCase):
    def setUp(self):
        estimator = ConstantVelocityEstimator(estimator_config(
            maximum_sample_interval_seconds=1.0,
        ))
        estimator.add_observation(
            observation(1.0, (0, 0, 0), (0.01, 0.01, 0.01)),
            1.01,
        )
        update = estimator.add_observation(
            observation(2.0, (1, -2, 0.5), (0.04, 0.04, 0.04)),
            2.01,
        )
        self.estimate = update.estimate
        self.predictor = ConstantVelocityPredictor(
            maximum_horizon_seconds=1.0,
            process_position_variance_per_second=DiagonalCovariance3(
                0.02,
                0.03,
                0.04,
            ),
        )

    def test_prediction_propagates_position_velocity_and_covariance(self):
        predicted = self.predictor.predict(self.estimate, 2.5)

        self.assertEqual(predicted.position, Vector3(1.5, -3.0, 0.75))
        self.assertEqual(predicted.velocity, self.estimate.velocity)
        self.assertAlmostEqual(
            predicted.covariance.x_axis.position_variance,
            0.1025,
        )
        self.assertAlmostEqual(
            predicted.covariance.x_axis.position_velocity_covariance,
            0.065,
        )
        self.assertAlmostEqual(predicted.horizon_seconds, 0.5)
        self.assertEqual(
            predicted.covariance.position_diagonal().x_variance,
            predicted.covariance.x_axis.position_variance,
        )

    def test_zero_horizon_preserves_estimated_state(self):
        predicted = self.predictor.predict(self.estimate, 2.0)
        self.assertEqual(predicted.position, self.estimate.position)
        self.assertEqual(predicted.velocity, self.estimate.velocity)
        self.assertEqual(predicted.covariance, self.estimate.covariance)

    def test_prediction_rejects_invalid_horizons_and_types(self):
        for target in (1.9, 3.000001, math.nan, True):
            with self.subTest(target=target):
                with self.assertRaises(PredictionError):
                    self.predictor.predict(self.estimate, target)
        with self.assertRaises(PredictionError):
            self.predictor.predict(object(), 2.1)
        with self.assertRaises(PredictionError):
            ConstantVelocityPredictor(
                0.0,
                DiagonalCovariance3(0, 0, 0),
            )
        with self.assertRaises(PredictionError):
            ConstantVelocityPredictor(1.0, object())

        huge_horizon = ConstantVelocityPredictor(
            1e308,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        )
        with self.assertRaisesRegex(
            PredictionError,
            "must not precede",
        ):
            huge_horizon.predict(self.estimate, 1.0)

    def test_timestamp_tolerance_validates_composed_timestamp_scale(self):
        unit_ulp = math.ulp(1.0)
        self.assertEqual(PREDICTION_TIMESTAMP_TOLERANCE_ULPS, 16.0)
        self.assertEqual(
            prediction_timestamp_tolerance(1.0, 2.0, 1.5),
            PREDICTION_TIMESTAMP_TOLERANCE_ULPS * math.ulp(2.0),
        )
        huge_horizon = ConstantVelocityPredictor(
            1e308,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        )
        accepted_estimate = replace(
            self.estimate,
            timestamp_seconds=(
                1.0 + PREDICTION_TIMESTAMP_TOLERANCE_ULPS * unit_ulp
            ),
        )
        rejected_estimate = replace(
            self.estimate,
            timestamp_seconds=(
                1.0
                + (PREDICTION_TIMESTAMP_TOLERANCE_ULPS + 1.0) * unit_ulp
            ),
        )

        accepted = huge_horizon.predict(accepted_estimate, 1.0)

        self.assertEqual(
            accepted.timestamp_seconds,
            accepted_estimate.timestamp_seconds,
        )
        with self.assertRaisesRegex(PredictionError, "must not precede"):
            huge_horizon.predict(rejected_estimate, 1.0)
        for values in (
            (-1.0, 1.0),
            (1.0, True),
            (1.0, 2.0, math.inf),
        ):
            with self.subTest(values=values):
                with self.assertRaises(PredictionError):
                    prediction_timestamp_tolerance(*values)

    def test_prediction_rejects_unrepresentable_projected_state(self):
        zero_axis = AxisStateCovariance(0.0, 0.0, 0.0)
        estimate = MotionEstimate(
            timestamp_seconds=1.0,
            frame_id="table",
            position=Vector3(1e308, 0, 0),
            velocity=Vector3(1e308, 0, 0),
            covariance=StateCovariance3(zero_axis, zero_axis, zero_axis),
            sample_interval_seconds=0.1,
        )

        with self.assertRaisesRegex(PredictionError, "cannot be represented"):
            self.predictor.predict(estimate, 2.0)

        large_axis = AxisStateCovariance(1e308, 1e308, -1e308)
        estimate = MotionEstimate(
            timestamp_seconds=1.0,
            frame_id="table",
            position=Vector3(0, 0, 0),
            velocity=Vector3(0, 0, 0),
            covariance=StateCovariance3(
                large_axis,
                large_axis,
                large_axis,
            ),
            sample_interval_seconds=0.1,
        )
        with self.assertRaisesRegex(PredictionError, "cannot be represented"):
            self.predictor.predict(estimate, 2.0)


class ObservationReplayTests(unittest.TestCase):
    def test_canonical_round_trip_and_replay_produce_deterministic_updates(self):
        replay = ObservationReplay((
            replay_sample(1.0, (0, 0, 0)),
            replay_sample(1.1, (1, 0, 0)),
            replay_sample(1.2, (1.5, 1, 0)),
        ))

        payload = encode_observation_replay(replay)
        decoded = decode_observation_replay(payload)
        updates = decoded.run(estimator_config())

        self.assertEqual(decoded, replay)
        self.assertEqual(decoded.frame_id, "table")
        self.assertTrue(payload.endswith(b"\n"))
        self.assertEqual(
            payload.splitlines()[0],
            (
                b'{"position_unit":"millimeter",'
                b'"schema":"ar4.observation-replay.v1",'
                b'"timebase":"monotonic-seconds"}'
            ),
        )
        self.assertEqual(
            tuple(update.status for update in updates),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertAlmostEqual(updates[-1].estimate.velocity.x, 5.0)
        self.assertAlmostEqual(updates[-1].estimate.velocity.y, 10.0)
        self.assertEqual(updates[-1].estimate.velocity.z, 0.0)

    def test_decoder_accepts_crlf_without_relaxing_record_validation(self):
        payload = replay_payload((replay_record(1.0),))
        decoded = decode_observation_replay(payload.replace(b"\n", b"\r\n"))
        self.assertEqual(decoded.samples[0].observation.timestamp_seconds, 1.0)

    def test_decoder_rejects_payload_and_json_boundary_failures(self):
        header = (
            b'{"position_unit":"millimeter",'
            b'"schema":"ar4.observation-replay.v1",'
            b'"timebase":"monotonic-seconds"}\n'
        )
        failures = (
            b"",
            b"\xff",
            header + b"\x00",
            header,
            header + b"\n",
            header.replace(b"\n", b"\r") + b"x",
            (
                b'{"position_unit":"millimeter",'
                b'"schema":"ar4.observation-replay.v1",'
                b'"schema":"duplicate",'
                b'"timebase":"monotonic-seconds"}\n'
            ),
            (
                b'{"position_unit":"millimeter",'
                b'"schema":"ar4.observation-replay.v1",'
                b'"timebase":NaN}\n'
            ),
            b'[]\n',
        )
        for payload in failures:
            with self.subTest(payload=payload[:40]):
                with self.assertRaises(ReplayFormatError):
                    decode_observation_replay(payload)
        with self.assertRaises(ReplayFormatError):
            decode_observation_replay("not bytes")

        oversized_line = header + b" " * 4097 + b"\n"
        with self.assertRaisesRegex(ReplayFormatError, "line limit"):
            decode_observation_replay(oversized_line)

        parsed_header = {
            "position_unit": "millimeter",
            "schema": "ar4.observation-replay.v1",
            "timebase": "monotonic-seconds",
        }
        with patch(
            "ARrobots.dynamic_motion.json.loads",
            side_effect=(parsed_header, RecursionError("depth")),
        ):
            with self.assertRaisesRegex(ReplayFormatError, "invalid JSON"):
                decode_observation_replay(header + b"{}\n")

    def test_decoder_rejects_schema_and_observation_domain_failures(self):
        invalid_records = []
        missing = replay_record(1.0)
        del missing["frame_id"]
        invalid_records.append(missing)
        extra = replay_record(1.0)
        extra["extra"] = 1
        invalid_records.append(extra)
        invalid_records.extend((
            replay_record(1.0, position=(0, 1)),
            replay_record(1.0, variance=(0, -1, 0)),
            replay_record(1.0, frame_id="table frame"),
            replay_record("1.0", received_at=1.01),
            replay_record(1.0, received_at=True),
        ))
        for record in invalid_records:
            with self.subTest(record=record):
                with self.assertRaises(ReplayFormatError):
                    decode_observation_replay(replay_payload((record,)))

        bad_header = (
            json_line({
                "position_unit": "millimeter",
                "schema": "wrong",
                "timebase": "monotonic-seconds",
            })
            + "\n"
        ).encode("utf-8")
        with self.assertRaises(ReplayFormatError):
            decode_observation_replay(bad_header)
        wrong_unit = (
            json_line({
                "position_unit": "meter",
                "schema": "ar4.observation-replay.v1",
                "timebase": "monotonic-seconds",
            })
            + "\n"
        ).encode("utf-8")
        with self.assertRaises(ReplayFormatError):
            decode_observation_replay(wrong_unit)

    def test_decoder_rejects_sequence_limit_and_order_failures(self):
        failures = (
            (replay_record(1.0), replay_record(1.0)),
            (replay_record(1.0), replay_record(1.1, frame_id="camera")),
            (
                replay_record(1.0, received_at=1.2),
                replay_record(1.1, received_at=1.15),
            ),
        )
        for records in failures:
            with self.subTest(records=records):
                with self.assertRaises(ReplayFormatError):
                    decode_observation_replay(replay_payload(records))

        payload = replay_payload((replay_record(1.0), replay_record(1.1)))
        with self.assertRaises(ReplayFormatError):
            decode_observation_replay(payload, maximum_records=1)
        with self.assertRaises(ReplayFormatError):
            decode_observation_replay(payload, maximum_bytes=len(payload) - 1)
        with self.assertRaises(ReplayFormatError):
            decode_observation_replay(
                payload,
                maximum_bytes=OBSERVATION_REPLAY_MAXIMUM_BYTES + 1,
            )
        for invalid_limit in (0, True, 1.5):
            with self.subTest(invalid_limit=invalid_limit):
                with self.assertRaises(ReplayFormatError):
                    decode_observation_replay(
                        payload,
                        maximum_records=invalid_limit,
                    )

    def test_replay_execution_preserves_estimator_rejection(self):
        replay = ObservationReplay((
            replay_sample(1.0, (0, 0, 0), received_at=1.5),
        ))
        with self.assertRaises(StaleObservationError):
            replay.run(estimator_config())

    def test_replay_preserves_tolerated_future_clock_skew(self):
        replay = ObservationReplay((
            replay_sample(1.0, (0, 0, 0), received_at=0.995),
        ))

        updates = decode_observation_replay(
            encode_observation_replay(replay)
        ).run(estimator_config())

        self.assertIs(
            updates[0].status,
            EstimatorUpdateStatus.BASELINE_ACCEPTED,
        )

    def test_replay_construction_and_encoding_reject_invalid_types(self):
        class BrokenIterator:
            def __iter__(self):
                return self

            def __next__(self):
                raise RuntimeError("iteration failed")

        with self.assertRaises(ObservationValidationError):
            ObservationReplay(())
        with self.assertRaises(ObservationValidationError):
            ObservationReplay((object(),))
        with self.assertRaisesRegex(ObservationValidationError, "iteration failed"):
            ObservationReplay(BrokenIterator())
        with self.assertRaises(ReplayFormatError):
            encode_observation_replay(object())
        with self.assertRaises(ObservationValidationError):
            ObservationReplay((replay_sample(1.0, (0, 0, 0)),)).run(object())


if __name__ == "__main__":
    unittest.main()
