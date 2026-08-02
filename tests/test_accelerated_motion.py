from dataclasses import FrozenInstanceError
import math
import unittest

from ARrobots.dynamic_motion import (
    PREDICTION_TIMESTAMP_TOLERANCE_ULPS,
    AccelerationStateCovariance3,
    AcceleratedMotionEstimate,
    AcceleratedPredictedMotionState,
    AxisAccelerationStateCovariance,
    AxisStateCovariance,
    ConstantAccelerationEstimator,
    ConstantAccelerationPredictor,
    ConstantVelocityEstimatorConfig,
    ConstantVelocityPredictor,
    DiagonalCovariance3,
    EstimatorUpdateStatus,
    MotionEstimate,
    ObservationReplay,
    ObservationValidationError,
    PositionObservation,
    PredictionError,
    ReplayObservation,
    StateCovariance3,
    Vector3,
)
from ARrobots.interception import (
    InterceptCandidateEvaluation,
    InterceptFeasibility,
    InterceptFeasibilityStatus,
    InterceptSelection,
    InterceptSelectionError,
    InterceptSelectionStatus,
    InterceptSelector,
    InterceptSelectorConfig,
    select_acceleration_replay_intercepts,
)


def estimator_config(**overrides):
    values = {
        "frame_id": "table",
        "maximum_observation_age_seconds": 0.2,
        "minimum_sample_interval_seconds": 0.01,
        "maximum_sample_interval_seconds": 3.0,
        "maximum_future_skew_seconds": 0.0,
    }
    values.update(overrides)
    return ConstantVelocityEstimatorConfig(**values)


def observation(
    timestamp,
    position=(0.0, 0.0, 0.0),
    variance=(1.0, 1.0, 1.0),
    frame_id="table",
):
    return PositionObservation(
        timestamp_seconds=timestamp,
        frame_id=frame_id,
        position=Vector3(*position),
        position_variance=DiagonalCovariance3(*variance),
    )


def replay_observation(timestamp, position):
    return ReplayObservation(
        observation(timestamp, position, variance=(0.01, 0.01, 0.01)),
        timestamp,
    )


def acceleration_axis(
    position_variance=1.0,
    velocity_variance=1.0,
    acceleration_variance=1.0,
    position_velocity_covariance=0.0,
    position_acceleration_covariance=0.0,
    velocity_acceleration_covariance=0.0,
):
    return AxisAccelerationStateCovariance(
        position_variance=position_variance,
        velocity_variance=velocity_variance,
        position_velocity_covariance=position_velocity_covariance,
        acceleration_variance=acceleration_variance,
        position_acceleration_covariance=(
            position_acceleration_covariance
        ),
        velocity_acceleration_covariance=(
            velocity_acceleration_covariance
        ),
    )


def acceleration_estimate(
    timestamp=1.0,
    position=(0.0, 0.0, 0.0),
    velocity=(1.0, 0.0, 0.0),
    acceleration=(2.0, 0.0, 0.0),
    covariance=None,
):
    axis = acceleration_axis()
    return AcceleratedMotionEstimate(
        timestamp_seconds=timestamp,
        frame_id="table",
        position=Vector3(*position),
        velocity=Vector3(*velocity),
        covariance=(
            AccelerationStateCovariance3(axis, axis, axis)
            if covariance is None
            else covariance
        ),
        sample_interval_seconds=0.1,
        acceleration=Vector3(*acceleration),
        previous_sample_interval_seconds=0.1,
    )


class VelocityOnlyPredictor:
    maximum_horizon_seconds = 0.5

    def __init__(self):
        self._predictor = ConstantVelocityPredictor(
            self.maximum_horizon_seconds,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        )

    def predict(self, estimate, target_timestamp_seconds):
        base_covariance = StateCovariance3(*(
            AxisStateCovariance(
                axis.position_variance,
                axis.velocity_variance,
                axis.position_velocity_covariance,
            )
            for axis in estimate.covariance.axes()
        ))
        base_estimate = MotionEstimate(
            timestamp_seconds=estimate.timestamp_seconds,
            frame_id=estimate.frame_id,
            position=estimate.position,
            velocity=estimate.velocity,
            covariance=base_covariance,
            sample_interval_seconds=estimate.sample_interval_seconds,
        )
        return self._predictor.predict(
            base_estimate,
            target_timestamp_seconds,
        )


class AccelerationCovarianceTests(unittest.TestCase):
    def test_full_axis_covariance_requires_positive_semidefinite_input(self):
        valid = acceleration_axis(
            position_variance=1.0,
            velocity_variance=4.0,
            acceleration_variance=9.0,
            position_velocity_covariance=2.0,
            position_acceleration_covariance=3.0,
            velocity_acceleration_covariance=6.0,
        )

        self.assertEqual(valid.velocity_acceleration_covariance, 6.0)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "positive semidefinite",
        ):
            acceleration_axis(
                position_acceleration_covariance=1.01,
            )
        with self.assertRaisesRegex(
            ObservationValidationError,
            "positive semidefinite",
        ):
            acceleration_axis(
                position_velocity_covariance=0.9,
                position_acceleration_covariance=0.9,
                velocity_acceleration_covariance=-0.9,
            )
        with self.assertRaisesRegex(
            ObservationValidationError,
            "positive semidefinite",
        ):
            acceleration_axis(
                position_variance=0.0,
                acceleration_variance=0.0,
                position_acceleration_covariance=math.ulp(0.0),
            )
        for overrides in (
            {"acceleration_variance": -1.0},
            {"position_acceleration_covariance": math.inf},
            {"velocity_acceleration_covariance": True},
        ):
            with self.subTest(overrides=overrides):
                with self.assertRaises(ObservationValidationError):
                    acceleration_axis(**overrides)

    def test_three_axis_covariance_requires_acceleration_axes(self):
        axis = acceleration_axis()
        covariance = AccelerationStateCovariance3(axis, axis, axis)

        self.assertEqual(
            covariance.position_diagonal().components(),
            (1.0, 1.0, 1.0),
        )
        with self.assertRaises(ObservationValidationError):
            AccelerationStateCovariance3(
                axis,
                axis,
                AxisStateCovariance(1.0, 1.0, 0.0),
            )
        with self.assertRaises(FrozenInstanceError):
            axis.acceleration_variance = 2.0

    def test_accelerated_states_require_typed_acceleration_covariance(self):
        estimate = acceleration_estimate()
        plain_covariance = StateCovariance3(*(
            AxisStateCovariance(0.0, 0.0, 0.0)
            for _ in range(3)
        ))
        prediction = AcceleratedPredictedMotionState(
            source_timestamp_seconds=estimate.timestamp_seconds,
            timestamp_seconds=estimate.timestamp_seconds,
            frame_id=estimate.frame_id,
            position=estimate.position,
            velocity=estimate.velocity,
            covariance=estimate.covariance,
            acceleration=estimate.acceleration,
        )

        self.assertEqual(prediction.acceleration, estimate.acceleration)
        with self.assertRaises(ObservationValidationError):
            AcceleratedMotionEstimate(
                timestamp_seconds=1.0,
                frame_id="table",
                position=Vector3(0.0, 0.0, 0.0),
                velocity=Vector3(0.0, 0.0, 0.0),
                covariance=plain_covariance,
                sample_interval_seconds=0.1,
                acceleration=Vector3(0.0, 0.0, 0.0),
                previous_sample_interval_seconds=0.1,
            )
        with self.assertRaises(ObservationValidationError):
            AcceleratedMotionEstimate(
                timestamp_seconds=1.0,
                frame_id="table",
                position=estimate.position,
                velocity=estimate.velocity,
                covariance=estimate.covariance,
                sample_interval_seconds=0.1,
                acceleration=(0.0, 0.0, 0.0),
                previous_sample_interval_seconds=0.1,
            )
        with self.assertRaises(ObservationValidationError):
            AcceleratedMotionEstimate(
                timestamp_seconds=1.0,
                frame_id="table",
                position=estimate.position,
                velocity=estimate.velocity,
                covariance=estimate.covariance,
                sample_interval_seconds=0.1,
                acceleration=estimate.acceleration,
                previous_sample_interval_seconds=0.0,
            )
        with self.assertRaises(PredictionError):
            AcceleratedPredictedMotionState(
                source_timestamp_seconds=1.0,
                timestamp_seconds=1.0,
                frame_id="table",
                position=estimate.position,
                velocity=estimate.velocity,
                covariance=plain_covariance,
                acceleration=estimate.acceleration,
            )
        with self.assertRaises(PredictionError):
            AcceleratedPredictedMotionState(
                source_timestamp_seconds=1.0,
                timestamp_seconds=1.0,
                frame_id="table",
                position=Vector3(0.0, 0.0, 0.0),
                velocity=Vector3(0.0, 0.0, 0.0),
                covariance=estimate.covariance,
                acceleration=(0.0, 0.0, 0.0),
            )


class ConstantAccelerationEstimatorTests(unittest.TestCase):
    def test_unequal_intervals_recover_terminal_velocity_and_acceleration(self):
        estimator = ConstantAccelerationEstimator(estimator_config())
        samples = (
            observation(0.0, (0.0, 0.0, 5.0)),
            observation(1.0, (1.0, 1.5, 5.0)),
            observation(3.0, (9.0, 1.5, 5.0)),
        )

        updates = tuple(
            estimator.add_observation(sample, sample.timestamp_seconds)
            for sample in samples
        )
        estimate = updates[-1].estimate

        self.assertEqual(
            tuple(update.status for update in updates),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertEqual(estimate.position.components(), (9.0, 1.5, 5.0))
        for actual, expected in zip(
            estimate.velocity.components(),
            (6.0, -1.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            estimate.acceleration.components(),
            (2.0, -1.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(estimate.previous_sample_interval_seconds, 1.0)
        self.assertEqual(estimate.sample_interval_seconds, 2.0)

    def test_estimator_recovers_translated_quadratic_models(self):
        interval_sets = (
            (0.0, 0.1, 0.35),
            (2.0, 2.4, 3.0),
            (10.0, 10.25, 10.75),
        )
        initial_position = (4.0, -3.0, 8.0)
        initial_velocity = (1.5, -2.0, 0.25)
        acceleration = (0.75, -1.25, 2.5)
        for timestamps in interval_sets:
            with self.subTest(timestamps=timestamps):
                estimator = ConstantAccelerationEstimator(
                    estimator_config()
                )
                for timestamp in timestamps:
                    position = tuple(
                        start
                        + velocity * timestamp
                        + 0.5 * axis_acceleration * timestamp * timestamp
                        for start, velocity, axis_acceleration in zip(
                            initial_position,
                            initial_velocity,
                            acceleration,
                        )
                    )
                    update = estimator.add_observation(
                        observation(timestamp, position),
                        timestamp,
                    )
                for actual, expected in zip(
                    update.estimate.acceleration.components(),
                    acceleration,
                ):
                    self.assertAlmostEqual(actual, expected)
                expected_velocity = tuple(
                    velocity + axis_acceleration * timestamps[-1]
                    for velocity, axis_acceleration in zip(
                        initial_velocity,
                        acceleration,
                    )
                )
                for actual, expected in zip(
                    update.estimate.velocity.components(),
                    expected_velocity,
                ):
                    self.assertAlmostEqual(actual, expected)

    def test_estimator_covariance_matches_three_sample_linear_model(self):
        estimator = ConstantAccelerationEstimator(estimator_config())
        for timestamp, position in (
            (0.0, (0.0, 0.0, 0.0)),
            (1.0, (1.0, 1.0, 1.0)),
        ):
            estimator.add_observation(
                observation(timestamp, position),
                timestamp,
            )

        update = estimator.add_observation(
            observation(3.0, (9.0, 9.0, 9.0)),
            3.0,
        )
        axis = update.estimate.covariance.x_axis

        self.assertAlmostEqual(axis.position_variance, 1.0)
        self.assertAlmostEqual(axis.velocity_variance, 61.0 / 18.0)
        self.assertAlmostEqual(axis.acceleration_variance, 14.0 / 9.0)
        self.assertAlmostEqual(
            axis.position_velocity_covariance,
            5.0 / 6.0,
        )
        self.assertAlmostEqual(
            axis.position_acceleration_covariance,
            1.0 / 3.0,
        )
        self.assertAlmostEqual(
            axis.velocity_acceleration_covariance,
            20.0 / 9.0,
        )

    def test_estimator_keeps_axis_measurement_variance_independent(self):
        estimator = ConstantAccelerationEstimator(estimator_config())
        samples = (
            observation(
                0.0,
                variance=(1.0, 2.0, 3.0),
            ),
            observation(
                1.0,
                (1.0, 1.0, 1.0),
                variance=(4.0, 5.0, 6.0),
            ),
            observation(
                2.0,
                (4.0, 4.0, 4.0),
                variance=(7.0, 8.0, 9.0),
            ),
        )
        for sample in samples[:-1]:
            estimator.add_observation(sample, sample.timestamp_seconds)

        covariance = estimator.add_observation(
            samples[-1],
            samples[-1].timestamp_seconds,
        ).estimate.covariance

        self.assertEqual(
            tuple(axis.position_variance for axis in covariance.axes()),
            (7.0, 8.0, 9.0),
        )
        self.assertEqual(
            tuple(axis.acceleration_variance for axis in covariance.axes()),
            (24.0, 30.0, 36.0),
        )
        self.assertEqual(
            tuple(
                axis.position_acceleration_covariance
                for axis in covariance.axes()
            ),
            (7.0, 8.0, 9.0),
        )

    def test_estimator_rolls_window_and_resets_after_long_gap(self):
        estimator = ConstantAccelerationEstimator(
            estimator_config(maximum_sample_interval_seconds=0.5)
        )
        samples = (
            observation(0.0, (0.0, 0.0, 0.0)),
            observation(0.1, (0.01, 0.0, 0.0)),
            observation(1.0, (1.0, 0.0, 0.0)),
            observation(1.1, (1.21, 0.0, 0.0)),
            observation(1.2, (1.44, 0.0, 0.0)),
            observation(1.3, (1.69, 0.0, 0.0)),
        )

        updates = tuple(
            estimator.add_observation(sample, sample.timestamp_seconds)
            for sample in samples
        )

        self.assertEqual(
            tuple(update.status for update in updates),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                EstimatorUpdateStatus.BASELINE_RESET,
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertAlmostEqual(updates[-1].estimate.velocity.x, 2.6)
        self.assertAlmostEqual(updates[-1].estimate.acceleration.x, 2.0)

    def test_rejected_observation_preserves_estimator_state(self):
        estimator = ConstantAccelerationEstimator(estimator_config())
        first = observation(0.0)
        second = observation(0.1, (0.01, 0.0, 0.0))
        estimator.add_observation(first, 0.0)
        estimator.add_observation(second, 0.1)

        with self.assertRaises(ObservationValidationError):
            estimator.add_observation(
                observation(0.1, (1.0, 0.0, 0.0)),
                0.1,
            )

        self.assertIs(estimator.last_observation, second)
        self.assertEqual(estimator.last_received_at_seconds, 0.1)
        self.assertIsNone(estimator.estimate)
        estimator.reset()
        self.assertIsNone(estimator.last_observation)
        self.assertIsNone(estimator.last_received_at_seconds)
        self.assertIsNone(estimator.estimate)

    def test_estimator_rejects_invalid_config_and_unrepresentable_state(self):
        with self.assertRaises(ObservationValidationError):
            ConstantAccelerationEstimator(object())

        estimator = ConstantAccelerationEstimator(estimator_config())
        estimator.add_observation(
            observation(0.0, (-1e308, 0.0, 0.0)),
            0.0,
        )
        estimator.add_observation(
            observation(1.0, (1e308, 0.0, 0.0)),
            1.0,
        )
        with self.assertRaisesRegex(
            ObservationValidationError,
            "outside the host numeric range",
        ):
            estimator.add_observation(
                observation(2.0, (-1e308, 0.0, 0.0)),
                2.0,
            )
        self.assertEqual(estimator.last_observation.timestamp_seconds, 1.0)

    def test_unrepresentable_update_preserves_published_estimate(self):
        estimator = ConstantAccelerationEstimator(estimator_config())
        samples = (
            observation(0.0, (0.0, 0.0, 0.0)),
            observation(1.0, (1.0, 0.0, 0.0)),
            observation(2.0, (4.0, 0.0, 0.0)),
        )
        for sample in samples:
            estimator.add_observation(sample, sample.timestamp_seconds)
        published_estimate = estimator.estimate

        with self.assertRaisesRegex(
            ObservationValidationError,
            "current interval velocity",
        ):
            estimator.add_observation(
                observation(2.1, (-1e308, 0.0, 0.0)),
                2.1,
            )

        self.assertIs(estimator.estimate, published_estimate)
        self.assertIs(estimator.last_observation, samples[-1])
        self.assertEqual(estimator.last_received_at_seconds, 2.0)

    def test_covariance_coefficient_underflow_preserves_warmup_state(self):
        estimator = ConstantAccelerationEstimator(
            estimator_config(
                minimum_sample_interval_seconds=1e161,
                maximum_sample_interval_seconds=2e162,
            )
        )
        first = observation(0.0)
        second = observation(1e162, (1.0, 0.0, 0.0))
        estimator.add_observation(first, first.timestamp_seconds)
        estimator.add_observation(second, second.timestamp_seconds)

        with self.assertRaisesRegex(
            ObservationValidationError,
            "acceleration coefficient is outside the host numeric range",
        ):
            estimator.add_observation(
                observation(2e162, (2.0, 0.0, 0.0)),
                2e162,
            )

        self.assertIs(estimator.last_observation, second)
        self.assertIsNone(estimator.estimate)

    def test_velocity_underflow_preserves_warmup_state(self):
        estimator = ConstantAccelerationEstimator(estimator_config())
        minimum_float = math.ulp(0.0)
        first = observation(0.0)
        second = observation(2.0, (minimum_float, 0.0, 0.0))
        estimator.add_observation(first, first.timestamp_seconds)
        estimator.add_observation(second, second.timestamp_seconds)

        with self.assertRaisesRegex(
            ObservationValidationError,
            "previous interval velocity is outside the host numeric range",
        ):
            estimator.add_observation(
                observation(4.0, (2.0 * minimum_float, 0.0, 0.0)),
                4.0,
            )

        self.assertIs(estimator.last_observation, second)
        self.assertIsNone(estimator.estimate)


class PredictorModelBoundaryTests(unittest.TestCase):
    def test_constant_velocity_predictor_rejects_accelerated_estimate(self):
        with self.assertRaisesRegex(
            PredictionError,
            "non-accelerated MotionEstimate",
        ):
            ConstantVelocityPredictor(
                2.0,
                DiagonalCovariance3(0.0, 0.0, 0.0),
            ).predict(acceleration_estimate(), 1.1)

    def test_constant_acceleration_predictor_rejects_velocity_only_estimate(self):
        estimate = MotionEstimate(
            timestamp_seconds=1.0,
            frame_id="table",
            position=Vector3(0.0, 0.0, 0.0),
            velocity=Vector3(0.0, 0.0, 0.0),
            covariance=StateCovariance3(*(
                AxisStateCovariance(0.0, 0.0, 0.0)
                for _ in range(3)
            )),
            sample_interval_seconds=0.1,
        )

        with self.assertRaisesRegex(
            PredictionError,
            "estimate must be AcceleratedMotionEstimate",
        ):
            ConstantAccelerationPredictor(
                2.0,
                DiagonalCovariance3(0.0, 0.0, 0.0),
            ).predict(estimate, 1.1)


class ConstantAccelerationPredictorTests(unittest.TestCase):
    def test_prediction_propagates_state_and_full_covariance(self):
        predictor = ConstantAccelerationPredictor(
            2.0,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        )

        prediction = predictor.predict(acceleration_estimate(), 3.0)
        axis = prediction.covariance.x_axis

        self.assertIsInstance(prediction, AcceleratedPredictedMotionState)
        self.assertEqual(prediction.position.components(), (6.0, 0.0, 0.0))
        self.assertEqual(prediction.velocity.components(), (5.0, 0.0, 0.0))
        self.assertEqual(prediction.acceleration.components(), (2.0, 0.0, 0.0))
        self.assertAlmostEqual(axis.position_variance, 9.0)
        self.assertAlmostEqual(axis.velocity_variance, 5.0)
        self.assertAlmostEqual(axis.acceleration_variance, 1.0)
        self.assertAlmostEqual(axis.position_velocity_covariance, 6.0)
        self.assertAlmostEqual(axis.position_acceleration_covariance, 2.0)
        self.assertAlmostEqual(axis.velocity_acceleration_covariance, 2.0)

    def test_white_jerk_process_noise_propagates_every_covariance_term(self):
        predictor = ConstantAccelerationPredictor(
            2.0,
            DiagonalCovariance3(1.0, 1.0, 1.0),
        )

        axis = predictor.predict(
            acceleration_estimate(),
            3.0,
        ).covariance.x_axis

        self.assertAlmostEqual(axis.position_variance, 10.6)
        self.assertAlmostEqual(axis.velocity_variance, 23.0 / 3.0)
        self.assertAlmostEqual(axis.acceleration_variance, 3.0)
        self.assertAlmostEqual(axis.position_velocity_covariance, 8.0)
        self.assertAlmostEqual(
            axis.position_acceleration_covariance,
            10.0 / 3.0,
        )
        self.assertAlmostEqual(axis.velocity_acceleration_covariance, 4.0)

    def test_process_noise_remains_axis_local(self):
        predictor = ConstantAccelerationPredictor(
            2.0,
            DiagonalCovariance3(0.0, 1.0, 2.0),
        )

        covariance = predictor.predict(
            acceleration_estimate(),
            2.0,
        ).covariance

        self.assertEqual(
            tuple(axis.acceleration_variance for axis in covariance.axes()),
            (1.0, 2.0, 3.0),
        )

    def test_state_and_process_covariance_compose_across_horizons(self):
        predictor = ConstantAccelerationPredictor(
            2.0,
            DiagonalCovariance3(0.5, 1.0, 2.0),
        )
        estimate = acceleration_estimate(
            position=(2.0, -1.0, 4.0),
            velocity=(1.0, 2.0, -3.0),
            acceleration=(0.5, -0.25, 1.5),
        )

        direct = predictor.predict(estimate, 3.0)
        first = predictor.predict(estimate, 1.75)
        resumed_estimate = AcceleratedMotionEstimate(
            timestamp_seconds=first.timestamp_seconds,
            frame_id=first.frame_id,
            position=first.position,
            velocity=first.velocity,
            covariance=first.covariance,
            sample_interval_seconds=0.1,
            acceleration=first.acceleration,
            previous_sample_interval_seconds=0.1,
        )
        composed = predictor.predict(resumed_estimate, 3.0)

        for direct_values, composed_values in (
            (direct.position.components(), composed.position.components()),
            (direct.velocity.components(), composed.velocity.components()),
            (
                direct.acceleration.components(),
                composed.acceleration.components(),
            ),
        ):
            for direct_value, composed_value in zip(
                direct_values,
                composed_values,
            ):
                self.assertAlmostEqual(direct_value, composed_value)
        covariance_fields = (
            "position_variance",
            "velocity_variance",
            "position_velocity_covariance",
            "acceleration_variance",
            "position_acceleration_covariance",
            "velocity_acceleration_covariance",
        )
        for direct_axis, composed_axis in zip(
            direct.covariance.axes(),
            composed.covariance.axes(),
        ):
            for field_name in covariance_fields:
                self.assertAlmostEqual(
                    getattr(direct_axis, field_name),
                    getattr(composed_axis, field_name),
                )

    def test_prediction_enforces_input_horizon_and_source_boundaries(self):
        predictor = ConstantAccelerationPredictor(
            2.0,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        )
        estimate = acceleration_estimate()
        unit_ulp = math.ulp(estimate.timestamp_seconds)

        clamped = predictor.predict(
            estimate,
            estimate.timestamp_seconds
            - PREDICTION_TIMESTAMP_TOLERANCE_ULPS * unit_ulp,
        )

        self.assertEqual(clamped.timestamp_seconds, estimate.timestamp_seconds)
        with self.assertRaisesRegex(PredictionError, "must not precede"):
            predictor.predict(
                estimate,
                estimate.timestamp_seconds
                - (PREDICTION_TIMESTAMP_TOLERANCE_ULPS + 1.0) * unit_ulp,
            )
        with self.assertRaisesRegex(PredictionError, "configured maximum"):
            predictor.predict(estimate, 3.1)

    def test_prediction_rejects_invalid_noise_and_numeric_overflow(self):
        with self.assertRaises(PredictionError):
            ConstantAccelerationPredictor(1.0, object())
        for maximum_horizon in (0.0, -1.0, math.inf, True):
            with self.subTest(maximum_horizon=maximum_horizon):
                with self.assertRaises(PredictionError):
                    ConstantAccelerationPredictor(
                        maximum_horizon,
                        DiagonalCovariance3(0.0, 0.0, 0.0),
                    )

        predictor = ConstantAccelerationPredictor(
            2.0,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        )
        estimate = acceleration_estimate(
            position=(1e308, 0.0, 0.0),
            velocity=(1e308, 0.0, 0.0),
            acceleration=(1e308, 0.0, 0.0),
        )
        with self.assertRaisesRegex(PredictionError, "cannot be represented"):
            predictor.predict(estimate, 3.0)
        for target_timestamp in (True, math.inf):
            with self.subTest(target_timestamp=target_timestamp):
                with self.assertRaises(PredictionError):
                    predictor.predict(
                        acceleration_estimate(),
                        target_timestamp,
                    )

    def test_prediction_rejects_state_and_noise_underflow(self):
        minimum_float = math.ulp(0.0)
        zero_noise_predictor = ConstantAccelerationPredictor(
            1.0,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        )
        tiny_velocity_estimate = acceleration_estimate(
            timestamp=0.0,
            velocity=(minimum_float, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
        )

        with self.assertRaisesRegex(
            PredictionError,
            "predicted position term is outside the host numeric range",
        ):
            zero_noise_predictor.predict(tiny_velocity_estimate, 0.5)

        tiny_noise_predictor = ConstantAccelerationPredictor(
            1.0,
            DiagonalCovariance3(minimum_float, 0.0, 0.0),
        )
        with self.assertRaisesRegex(
            PredictionError,
            "predicted position variance term is outside the host numeric "
            "range",
        ):
            tiny_noise_predictor.predict(
                acceleration_estimate(timestamp=0.0),
                0.5,
            )


class AccelerationReplayTests(unittest.TestCase):
    def test_selection_value_rejects_acceleration_blind_candidate(self):
        estimate = acceleration_estimate()
        prediction = VelocityOnlyPredictor().predict(estimate, 1.1)
        candidate = InterceptCandidateEvaluation(
            prediction=prediction,
            maximum_position_standard_deviation_mm=2.0,
            terminal_speed_mm_per_second=1.0,
            feasibility=InterceptFeasibility(
                InterceptFeasibilityStatus.FEASIBLE,
                minimum_arrival_time_seconds=0.0,
                risk_score=0.0,
            ),
            arrival_margin_seconds=0.1,
            rejection_reason=None,
        )

        with self.assertRaisesRegex(
            InterceptSelectionError,
            "does not preserve selection estimate state",
        ):
            InterceptSelection(
                status=InterceptSelectionStatus.SELECTED,
                evaluated_at_seconds=1.0,
                estimate=estimate,
                candidates=(candidate,),
                selected_candidate=candidate,
            )

    def test_replay_owns_acceleration_estimator_and_exposes_warmup(self):
        replay = ObservationReplay((
            replay_observation(0.0, (0.0, 0.0, 0.0)),
            replay_observation(0.1, (0.01, 0.0, 0.0)),
            replay_observation(0.2, (0.04, 0.0, 0.0)),
        ))

        updates = replay.run_constant_acceleration(estimator_config())

        self.assertEqual(
            tuple(update.status for update in updates),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertAlmostEqual(updates[-1].estimate.acceleration.x, 2.0)
        with self.assertRaises(ObservationValidationError):
            replay.run_constant_acceleration(object())

    def test_replay_reselects_intercept_from_acceleration_estimates(self):
        replay = ObservationReplay((
            replay_observation(0.0, (0.0, 0.0, 0.0)),
            replay_observation(0.1, (0.01, 0.0, 0.0)),
            replay_observation(0.2, (0.04, 0.0, 0.0)),
            replay_observation(0.3, (0.09, 0.0, 0.0)),
        ))
        selector = InterceptSelector(
            InterceptSelectorConfig(
                frame_id="table",
                minimum_lead_time_seconds=0.1,
                maximum_lead_time_seconds=0.1,
                candidate_interval_seconds=0.1,
                maximum_estimate_age_seconds=0.2,
                maximum_future_skew_seconds=0.0,
                maximum_position_standard_deviation_mm=10.0,
                maximum_terminal_speed_mm_per_second=10.0,
                minimum_arrival_margin_seconds=0.0,
            ),
            ConstantAccelerationPredictor(
                0.5,
                DiagonalCovariance3(0.0, 0.0, 0.0),
            ),
            lambda prediction, evaluated_at: InterceptFeasibility(
                InterceptFeasibilityStatus.FEASIBLE,
                minimum_arrival_time_seconds=0.0,
                risk_score=0.0,
            ),
        )

        steps = select_acceleration_replay_intercepts(
            replay,
            estimator_config(),
            selector,
        )

        self.assertIsNone(steps[0].selection)
        self.assertIsNone(steps[1].selection)
        self.assertIs(
            steps[2].selection.status,
            InterceptSelectionStatus.SELECTED,
        )
        self.assertIs(
            steps[3].selection.status,
            InterceptSelectionStatus.SELECTED,
        )
        self.assertAlmostEqual(
            steps[2].selection.selected_candidate.prediction.position.x,
            0.09,
        )
        self.assertAlmostEqual(
            steps[3].selection.selected_candidate.prediction.position.x,
            0.16,
        )

    def test_acceleration_replay_rejects_constant_velocity_predictor(self):
        replay = ObservationReplay((
            replay_observation(0.0, (0.0, 0.0, 0.0)),
            replay_observation(0.1, (0.01, 0.0, 0.0)),
            replay_observation(0.2, (0.04, 0.0, 0.0)),
        ))
        feasibility_calls = []
        selector = InterceptSelector(
            InterceptSelectorConfig(
                frame_id="table",
                minimum_lead_time_seconds=0.1,
                maximum_lead_time_seconds=0.1,
                candidate_interval_seconds=0.1,
                maximum_estimate_age_seconds=0.2,
                maximum_future_skew_seconds=0.0,
                maximum_position_standard_deviation_mm=10.0,
                maximum_terminal_speed_mm_per_second=10.0,
                minimum_arrival_margin_seconds=0.0,
            ),
            ConstantVelocityPredictor(
                0.5,
                DiagonalCovariance3(0.0, 0.0, 0.0),
            ),
            lambda prediction, evaluated_at: (
                feasibility_calls.append(prediction)
                or InterceptFeasibility(
                    InterceptFeasibilityStatus.FEASIBLE,
                    minimum_arrival_time_seconds=0.0,
                    risk_score=0.0,
                )
            ),
        )

        with self.assertRaisesRegex(
            InterceptSelectionError,
            "candidate 0 prediction failed",
        ) as raised:
            select_acceleration_replay_intercepts(
                replay,
                estimator_config(),
                selector,
            )

        self.assertIsInstance(raised.exception.__cause__, PredictionError)
        self.assertEqual(
            str(raised.exception.__cause__),
            "estimate must be a non-accelerated MotionEstimate",
        )
        self.assertEqual(feasibility_calls, [])

    def test_acceleration_replay_rejects_custom_velocity_only_output(self):
        replay = ObservationReplay((
            replay_observation(0.0, (0.0, 0.0, 0.0)),
            replay_observation(0.1, (0.01, 0.0, 0.0)),
            replay_observation(0.2, (0.04, 0.0, 0.0)),
        ))
        feasibility_calls = []
        selector = InterceptSelector(
            InterceptSelectorConfig(
                frame_id="table",
                minimum_lead_time_seconds=0.1,
                maximum_lead_time_seconds=0.1,
                candidate_interval_seconds=0.1,
                maximum_estimate_age_seconds=0.2,
                maximum_future_skew_seconds=0.0,
                maximum_position_standard_deviation_mm=10.0,
                maximum_terminal_speed_mm_per_second=10.0,
                minimum_arrival_margin_seconds=0.0,
            ),
            VelocityOnlyPredictor(),
            lambda prediction, evaluated_at: (
                feasibility_calls.append(prediction)
                or InterceptFeasibility(
                    InterceptFeasibilityStatus.FEASIBLE,
                    minimum_arrival_time_seconds=0.0,
                    risk_score=0.0,
                )
            ),
        )

        with self.assertRaisesRegex(
            InterceptSelectionError,
            "prediction output does not preserve acceleration state",
        ):
            select_acceleration_replay_intercepts(
                replay,
                estimator_config(),
                selector,
            )

        self.assertEqual(feasibility_calls, [])


if __name__ == "__main__":
    unittest.main()
