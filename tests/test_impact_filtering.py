from dataclasses import FrozenInstanceError, replace
import math
import unittest

from ARrobots.dynamic_motion import (
    IMPACT_CONFIRMATION_MAXIMUM_OBSERVATIONS,
    ConstantAccelerationPredictor,
    ConstantVelocityEstimatorConfig,
    DiagonalCovariance3,
    EstimatorProcessingError,
    EstimatorUpdate,
    EstimatorUpdateStatus,
    ImpactAwareAccelerationEstimator,
    ImpactAwareEstimatorConfig,
    ObservationReplay,
    ObservationValidationError,
    OutOfOrderObservationError,
    PositionInnovation,
    PositionObservation,
    ReplayObservation,
    Vector3,
)
from ARrobots.interception import (
    InterceptFeasibility,
    InterceptFeasibilityStatus,
    InterceptSelectionError,
    InterceptSelectionStatus,
    InterceptSelector,
    InterceptSelectorConfig,
    select_impact_aware_acceleration_replay_intercepts,
)


def estimator_config(**overrides):
    values = {
        "frame_id": "table",
        "maximum_observation_age_seconds": 0.2,
        "minimum_sample_interval_seconds": 0.01,
        "maximum_sample_interval_seconds": 0.5,
        "maximum_future_skew_seconds": 0.0,
    }
    values.update(overrides)
    return ConstantVelocityEstimatorConfig(**values)


def impact_config(**overrides):
    values = {
        "maximum_axis_standardized_innovation": 4.0,
        "impact_confirmation_observations": 2,
        "process_acceleration_variance_per_second": DiagonalCovariance3(
            0.0,
            0.0,
            0.0,
        ),
    }
    values.update(overrides)
    return ImpactAwareEstimatorConfig(**values)


def observation(
    timestamp,
    position=(0.0, 0.0, 0.0),
    variance=(0.01, 0.01, 0.01),
):
    return PositionObservation(
        timestamp_seconds=timestamp,
        frame_id="table",
        position=Vector3(*position),
        position_variance=DiagonalCovariance3(*variance),
    )


def replay_observation(timestamp, position):
    return ReplayObservation(
        observation(timestamp, position),
        timestamp,
    )


def quadratic_observation(timestamp):
    return observation(timestamp, (timestamp * timestamp, 0.0, 0.0))


def tracked_estimator():
    estimator = ImpactAwareAccelerationEstimator(
        estimator_config(),
        impact_config(),
    )
    updates = tuple(
        estimator.add_observation(sample, sample.timestamp_seconds)
        for sample in (
            quadratic_observation(0.0),
            quadratic_observation(0.1),
            quadratic_observation(0.2),
        )
    )
    return estimator, updates


def selector():
    return InterceptSelector(
        InterceptSelectorConfig(
            frame_id="table",
            minimum_lead_time_seconds=0.1,
            maximum_lead_time_seconds=0.1,
            candidate_interval_seconds=0.1,
            maximum_estimate_age_seconds=0.2,
            maximum_future_skew_seconds=0.0,
            maximum_position_standard_deviation_mm=10.0,
            maximum_terminal_speed_mm_per_second=100.0,
            minimum_arrival_margin_seconds=0.0,
        ),
        ConstantAccelerationPredictor(
            0.5,
            DiagonalCovariance3(0.0, 0.0, 0.0),
        ),
        lambda _prediction, _evaluated_at: InterceptFeasibility(
            InterceptFeasibilityStatus.FEASIBLE,
            minimum_arrival_time_seconds=0.0,
            risk_score=0.0,
        ),
    )


class ImpactFilterValueTests(unittest.TestCase):
    def test_config_normalizes_and_freezes_explicit_gate(self):
        process_noise = DiagonalCovariance3(0, 1, 2)
        config = ImpactAwareEstimatorConfig(
            4,
            3,
            process_noise,
        )

        self.assertEqual(config.maximum_axis_standardized_innovation, 4.0)
        self.assertEqual(config.impact_confirmation_observations, 3)
        self.assertIsNot(
            config.process_acceleration_variance_per_second,
            process_noise,
        )
        with self.assertRaises(FrozenInstanceError):
            config.impact_confirmation_observations = 4

    def test_config_rejects_invalid_gate_confirmation_and_noise(self):
        for threshold in (True, 0.0, -1.0, math.inf):
            with self.subTest(threshold=threshold):
                with self.assertRaises(ObservationValidationError):
                    impact_config(
                        maximum_axis_standardized_innovation=threshold,
                    )
        for count in (
            True,
            1,
            IMPACT_CONFIRMATION_MAXIMUM_OBSERVATIONS + 1,
        ):
            with self.subTest(count=count):
                with self.assertRaises(ObservationValidationError):
                    impact_config(impact_confirmation_observations=count)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "process noise must be built-in",
        ):
            impact_config(
                process_acceleration_variance_per_second=object(),
            )
        tampered_noise = DiagonalCovariance3(0.0, 0.0, 0.0)
        object.__setattr__(tampered_noise, "x_variance", math.nan)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "process noise failed validation",
        ):
            impact_config(
                process_acceleration_variance_per_second=tampered_noise,
            )

    def test_position_innovation_validates_gate_evidence(self):
        innovation = PositionInnovation(
            residual=Vector3(1.0, 0.0, 0.0),
            combined_position_variance=DiagonalCovariance3(1.0, 1.0, 0.0),
            maximum_axis_standardized_innovation=0.5,
            exceeded_axes=(True, False, False),
        )

        self.assertTrue(innovation.exceeded)
        boundary = PositionInnovation(
            residual=Vector3(4.0, 0.0, 0.0),
            combined_position_variance=DiagonalCovariance3(1.0, 1.0, 1.0),
            maximum_axis_standardized_innovation=4.0,
            exceeded_axes=(False, False, False),
        )
        above_boundary = PositionInnovation(
            residual=Vector3(
                4.0 + 8.0 * math.ulp(4.0),
                0.0,
                0.0,
            ),
            combined_position_variance=DiagonalCovariance3(1.0, 1.0, 1.0),
            maximum_axis_standardized_innovation=4.0,
            exceeded_axes=(True, False, False),
        )
        self.assertFalse(boundary.exceeded)
        self.assertTrue(above_boundary.exceeded)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "do not match the gate inputs",
        ):
            replace(innovation, exceeded_axes=(False, False, False))
        invalid_values = (
            {"residual": object()},
            {"combined_position_variance": object()},
            {"maximum_axis_standardized_innovation": 0.0},
            {"exceeded_axes": [True, False, False]},
            {"exceeded_axes": (1, False, False)},
        )
        for replacement_values in invalid_values:
            with self.subTest(replacement_values=replacement_values):
                with self.assertRaises(ObservationValidationError):
                    replace(innovation, **replacement_values)
        tampered_residual = Vector3(0.0, 0.0, 0.0)
        object.__setattr__(tampered_residual, "x", math.nan)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "innovation inputs failed validation",
        ):
            replace(innovation, residual=tampered_residual)

    def test_update_contract_correlates_innovation_disposition(self):
        sample = observation(1.0, (1.0, 0.0, 0.0))
        innovation = PositionInnovation(
            residual=Vector3(1.0, 0.0, 0.0),
            combined_position_variance=DiagonalCovariance3(0.0, 1.0, 1.0),
            maximum_axis_standardized_innovation=4.0,
            exceeded_axes=(True, False, False),
        )

        rejected = EstimatorUpdate(
            EstimatorUpdateStatus.INNOVATION_REJECTED,
            sample,
            innovation=innovation,
            innovation_sequence_count=1,
        )
        reset = EstimatorUpdate(
            EstimatorUpdateStatus.IMPACT_RESET,
            sample,
            innovation=innovation,
            innovation_sequence_count=2,
        )
        non_exceeded = PositionInnovation(
            residual=Vector3(0.0, 0.0, 0.0),
            combined_position_variance=DiagonalCovariance3(
                1.0,
                1.0,
                1.0,
            ),
            maximum_axis_standardized_innovation=4.0,
            exceeded_axes=(False, False, False),
        )
        estimated = tracked_estimator()[1][-1]
        estimated_with_evidence = replace(
            estimated,
            innovation=non_exceeded,
        )
        baseline = EstimatorUpdate(
            EstimatorUpdateStatus.BASELINE_ACCEPTED,
            sample,
        )

        self.assertEqual(rejected.innovation_sequence_count, 1)
        self.assertEqual(reset.innovation_sequence_count, 2)
        self.assertIsNotNone(estimated_with_evidence.innovation)
        invalid_updates = (
            {"innovation": None},
            {"innovation_sequence_count": 0},
            {"estimate": object()},
        )
        for replacement_values in invalid_updates:
            with self.subTest(replacement_values=replacement_values):
                with self.assertRaises(ObservationValidationError):
                    replace(rejected, **replacement_values)
        with self.assertRaisesRegex(
            ObservationValidationError,
            "invalid sequence count",
        ):
            replace(reset, innovation_sequence_count=1)
        invalid_contracts = (
            (
                lambda: replace(estimated, innovation=innovation),
                "requires a non-exceeded innovation",
            ),
            (
                lambda: replace(estimated, innovation_sequence_count=1),
                "cannot carry an innovation sequence",
            ),
            (
                lambda: replace(baseline, innovation=non_exceeded),
                "non-innovation update cannot carry innovation state",
            ),
            (
                lambda: replace(baseline, innovation_sequence_count=1),
                "non-innovation update cannot carry innovation state",
            ),
            (
                lambda: replace(rejected, innovation=non_exceeded),
                "requires an exceeded innovation",
            ),
            (
                lambda: replace(
                    rejected,
                    innovation_sequence_count=(
                        IMPACT_CONFIRMATION_MAXIMUM_OBSERVATIONS
                    ),
                ),
                "reached the confirmation limit",
            ),
            (
                lambda: replace(
                    reset,
                    innovation_sequence_count=(
                        IMPACT_CONFIRMATION_MAXIMUM_OBSERVATIONS + 1
                    ),
                ),
                "exceeds the observation limit",
            ),
        )
        for construct, expected_message in invalid_contracts:
            with self.subTest(expected_message=expected_message):
                with self.assertRaisesRegex(
                    ObservationValidationError,
                    expected_message,
                ):
                    construct()


class ImpactAwareAccelerationEstimatorTests(unittest.TestCase):
    def test_matching_model_updates_without_innovation_rejection(self):
        estimator, updates = tracked_estimator()
        fourth = quadratic_observation(0.3)

        update = estimator.add_observation(fourth, 0.3)

        self.assertEqual(
            tuple(item.status for item in (*updates, update)),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertIsNotNone(update.innovation)
        self.assertFalse(update.innovation.exceeded)
        self.assertIs(estimator.estimate, update.estimate)
        self.assertEqual(estimator.pending_innovation_count, 0)
        self.assertEqual(estimator.impact_generation, 0)

    def test_isolated_innovation_is_rejected_without_model_mutation(self):
        estimator, _updates = tracked_estimator()
        published = estimator.estimate
        model_last = estimator.model_last_observation
        outlier = observation(0.3, (5.0, 0.0, 0.0))

        rejected = estimator.add_observation(outlier, 0.3)

        self.assertIs(
            rejected.status,
            EstimatorUpdateStatus.INNOVATION_REJECTED,
        )
        self.assertEqual(rejected.innovation_sequence_count, 1)
        self.assertTrue(rejected.innovation.exceeded_axes[0])
        self.assertIsNone(estimator.estimate)
        self.assertIs(estimator.model_last_observation, model_last)
        self.assertIs(estimator.last_observation, outlier)

        recovery = estimator.add_observation(
            quadratic_observation(0.4),
            0.4,
        )

        self.assertIs(
            recovery.status,
            EstimatorUpdateStatus.ESTIMATE_UPDATED,
        )
        self.assertIsNot(recovery.estimate, published)
        self.assertFalse(recovery.innovation.exceeded)
        self.assertEqual(estimator.pending_innovation_count, 0)
        self.assertEqual(estimator.impact_generation, 0)

    def test_consecutive_innovations_confirm_impact_and_reacquire(self):
        estimator, _updates = tracked_estimator()

        first = estimator.add_observation(
            observation(0.3, (5.0, 0.0, 0.0)),
            0.3,
        )
        confirmed = estimator.add_observation(
            observation(0.4, (5.1, 0.0, 0.0)),
            0.4,
        )

        self.assertIs(
            first.status,
            EstimatorUpdateStatus.INNOVATION_REJECTED,
        )
        self.assertIs(
            confirmed.status,
            EstimatorUpdateStatus.IMPACT_RESET,
        )
        self.assertEqual(confirmed.innovation_sequence_count, 2)
        self.assertEqual(estimator.pending_innovation_count, 0)
        self.assertEqual(estimator.impact_generation, 1)
        self.assertIsNone(estimator.estimate)
        self.assertIs(estimator.model_last_observation, confirmed.observation)

        warmup = estimator.add_observation(
            observation(0.5, (5.2, 0.0, 0.0)),
            0.5,
        )
        reacquired = estimator.add_observation(
            observation(0.6, (5.3, 0.0, 0.0)),
            0.6,
        )

        self.assertIs(
            warmup.status,
            EstimatorUpdateStatus.WARMUP_ACCEPTED,
        )
        self.assertIs(
            reacquired.status,
            EstimatorUpdateStatus.ESTIMATE_UPDATED,
        )
        self.assertAlmostEqual(reacquired.estimate.velocity.x, 1.0)
        self.assertAlmostEqual(reacquired.estimate.acceleration.x, 0.0)

    def test_confirmation_count_controls_reset_boundary(self):
        estimator = ImpactAwareAccelerationEstimator(
            estimator_config(),
            impact_config(impact_confirmation_observations=3),
        )
        for timestamp in (0.0, 0.1, 0.2):
            estimator.add_observation(
                quadratic_observation(timestamp),
                timestamp,
            )

        updates = tuple(
            estimator.add_observation(
                observation(timestamp, (position, 0.0, 0.0)),
                timestamp,
            )
            for timestamp, position in (
                (0.3, 50.0),
                (0.4, 50.1),
                (0.5, 50.2),
            )
        )

        self.assertEqual(
            tuple(update.status for update in updates),
            (
                EstimatorUpdateStatus.INNOVATION_REJECTED,
                EstimatorUpdateStatus.INNOVATION_REJECTED,
                EstimatorUpdateStatus.IMPACT_RESET,
            ),
        )
        self.assertEqual(
            tuple(update.innovation_sequence_count for update in updates),
            (1, 2, 3),
        )

    def test_long_gap_resets_pending_innovation_without_impact_claim(self):
        estimator, _updates = tracked_estimator()
        estimator.add_observation(
            observation(0.3, (5.0, 0.0, 0.0)),
            0.3,
        )
        after_gap = observation(1.0, (1.0, 0.0, 0.0))

        update = estimator.add_observation(after_gap, 1.0)

        self.assertIs(update.status, EstimatorUpdateStatus.BASELINE_RESET)
        self.assertEqual(estimator.pending_innovation_count, 0)
        self.assertEqual(estimator.impact_generation, 0)
        self.assertIs(estimator.model_last_observation, after_gap)

    def test_rejected_admission_and_numeric_failure_preserve_state(self):
        estimator, _updates = tracked_estimator()
        outlier = observation(0.3, (5.0, 0.0, 0.0))
        estimator.add_observation(outlier, 0.3)
        last_innovation = estimator.last_innovation

        with self.assertRaises(OutOfOrderObservationError):
            estimator.add_observation(
                quadratic_observation(0.25),
                0.3,
            )

        self.assertIs(estimator.last_observation, outlier)
        self.assertIs(estimator.last_innovation, last_innovation)
        self.assertEqual(estimator.pending_innovation_count, 1)

        overflow_estimator = ImpactAwareAccelerationEstimator(
            estimator_config(),
            impact_config(),
        )
        for timestamp in (0.0, 0.1, 0.2):
            overflow_estimator.add_observation(
                observation(timestamp, (1e308, 0.0, 0.0)),
                timestamp,
            )
        with self.assertRaisesRegex(
            EstimatorProcessingError,
            "impact innovation update failed",
        ):
            overflow_estimator.add_observation(
                observation(0.3, (-1e308, 0.0, 0.0)),
                0.3,
            )
        self.assertEqual(
            overflow_estimator.model_last_observation.timestamp_seconds,
            0.2,
        )
        self.assertEqual(overflow_estimator.pending_innovation_count, 0)

    def test_accumulated_model_gap_resets_before_impact_confirmation(self):
        estimator = ImpactAwareAccelerationEstimator(
            estimator_config(maximum_sample_interval_seconds=0.5),
            impact_config(),
        )
        for timestamp in (0.0, 0.1, 0.2):
            estimator.add_observation(
                quadratic_observation(timestamp),
                timestamp,
            )
        first = observation(0.3, (5.0, 0.0, 0.0))
        estimator.add_observation(first, 0.3)
        after_gap = observation(0.79, (5.1, 0.0, 0.0))

        update = estimator.add_observation(after_gap, 0.79)

        self.assertIs(update.status, EstimatorUpdateStatus.BASELINE_RESET)
        self.assertIs(estimator.last_observation, after_gap)
        self.assertIs(estimator.model_last_observation, after_gap)
        self.assertIsNone(estimator.last_innovation)
        self.assertEqual(estimator.pending_innovation_count, 0)
        self.assertEqual(estimator.impact_generation, 0)

        boundary_estimator, _updates = tracked_estimator()
        boundary_estimator.add_observation(
            observation(0.3, (50.0, 0.0, 0.0)),
            0.3,
        )
        boundary_update = boundary_estimator.add_observation(
            observation(0.7, (50.4, 0.0, 0.0)),
            0.7,
        )
        self.assertIs(
            boundary_update.status,
            EstimatorUpdateStatus.IMPACT_RESET,
        )
        self.assertEqual(boundary_estimator.impact_generation, 1)

    def test_failed_confirmed_reset_preserves_preconfirmation_state(self):
        estimator, _updates = tracked_estimator()
        first = observation(0.3, (5.0, 0.0, 0.0))
        estimator.add_observation(first, 0.3)
        model_last = estimator.model_last_observation
        last_innovation = estimator.last_innovation

        def failed_reset(_observation, _received_at):
            raise RuntimeError("baseline replacement failed")

        estimator._replace_baseline = failed_reset
        with self.assertRaisesRegex(
            RuntimeError,
            "baseline replacement failed",
        ):
            estimator.add_observation(
                observation(0.4, (5.1, 0.0, 0.0)),
                0.4,
            )

        self.assertIs(estimator.last_observation, first)
        self.assertIs(estimator.model_last_observation, model_last)
        self.assertIs(estimator.last_innovation, last_innovation)
        self.assertEqual(estimator.pending_innovation_count, 1)
        self.assertEqual(estimator.impact_generation, 0)

    def test_zero_variance_mismatch_exceeds_gate(self):
        estimator = ImpactAwareAccelerationEstimator(
            estimator_config(),
            impact_config(),
        )
        for timestamp in (0.0, 0.1, 0.2):
            estimator.add_observation(
                observation(
                    timestamp,
                    variance=(0.0, 0.0, 0.0),
                ),
                timestamp,
            )

        update = estimator.add_observation(
            observation(
                0.3,
                (0.1, 0.0, 0.0),
                variance=(0.0, 0.0, 0.0),
            ),
            0.3,
        )

        self.assertIs(
            update.status,
            EstimatorUpdateStatus.INNOVATION_REJECTED,
        )
        self.assertEqual(update.innovation.exceeded_axes, (True, False, False))

    def test_constructor_rejects_derived_configs(self):
        class DerivedEstimatorConfig(ConstantVelocityEstimatorConfig):
            pass

        class DerivedImpactConfig(ImpactAwareEstimatorConfig):
            pass

        invalid = (
            (object(), impact_config()),
            (estimator_config(), object()),
            (DerivedEstimatorConfig(**estimator_config().__dict__), impact_config()),
            (
                estimator_config(),
                DerivedImpactConfig(**impact_config().__dict__),
            ),
        )
        for values in invalid:
            with self.subTest(values=values):
                with self.assertRaises(ObservationValidationError):
                    ImpactAwareAccelerationEstimator(*values)
        large_interval = ImpactAwareAccelerationEstimator(
            estimator_config(maximum_sample_interval_seconds=1e308),
            impact_config(),
        )
        self.assertEqual(
            large_interval.config.maximum_sample_interval_seconds,
            1e308,
        )

    def test_reset_clears_tracking_and_impact_state(self):
        estimator, _updates = tracked_estimator()
        estimator.add_observation(
            observation(0.3, (5.0, 0.0, 0.0)),
            0.3,
        )

        estimator.reset()

        self.assertIsNone(estimator.last_observation)
        self.assertIsNone(estimator.model_last_observation)
        self.assertIsNone(estimator.last_received_at_seconds)
        self.assertIsNone(estimator.estimate)
        self.assertIsNone(estimator.last_innovation)
        self.assertEqual(estimator.pending_innovation_count, 0)
        self.assertEqual(estimator.impact_generation, 0)


class ImpactAwareReplayTests(unittest.TestCase):
    def test_replay_exposes_rejection_impact_reset_and_reacquisition(self):
        replay = ObservationReplay((
            replay_observation(0.0, (0.0, 0.0, 0.0)),
            replay_observation(0.1, (0.01, 0.0, 0.0)),
            replay_observation(0.2, (0.04, 0.0, 0.0)),
            replay_observation(0.3, (5.0, 0.0, 0.0)),
            replay_observation(0.4, (5.1, 0.0, 0.0)),
            replay_observation(0.5, (5.2, 0.0, 0.0)),
            replay_observation(0.6, (5.3, 0.0, 0.0)),
        ))

        updates = replay.run_impact_aware_acceleration(
            estimator_config(),
            impact_config(),
        )

        self.assertEqual(
            tuple(update.status for update in updates),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
                EstimatorUpdateStatus.INNOVATION_REJECTED,
                EstimatorUpdateStatus.IMPACT_RESET,
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertEqual(updates[3].innovation_sequence_count, 1)
        self.assertEqual(updates[4].innovation_sequence_count, 2)
        with self.assertRaises(ObservationValidationError):
            replay.run_impact_aware_acceleration(
                object(),
                impact_config(),
            )
        with self.assertRaises(ObservationValidationError):
            replay.run_impact_aware_acceleration(
                estimator_config(),
                object(),
            )

    def test_intercept_replay_invalidates_selection_until_reacquisition(self):
        class DerivedEstimatorConfig(ConstantVelocityEstimatorConfig):
            pass

        replay = ObservationReplay((
            replay_observation(0.0, (0.0, 0.0, 0.0)),
            replay_observation(0.1, (0.01, 0.0, 0.0)),
            replay_observation(0.2, (0.04, 0.0, 0.0)),
            replay_observation(0.3, (5.0, 0.0, 0.0)),
            replay_observation(0.4, (5.1, 0.0, 0.0)),
            replay_observation(0.5, (5.2, 0.0, 0.0)),
            replay_observation(0.6, (5.3, 0.0, 0.0)),
        ))

        steps = select_impact_aware_acceleration_replay_intercepts(
            replay,
            estimator_config(),
            selector(),
            impact_config(),
        )

        self.assertIs(
            steps[2].selection.status,
            InterceptSelectionStatus.SELECTED,
        )
        self.assertIsNone(steps[3].selection)
        self.assertIsNone(steps[4].selection)
        self.assertIsNone(steps[5].selection)
        self.assertIs(
            steps[6].selection.status,
            InterceptSelectionStatus.SELECTED,
        )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "built-in ImpactAwareEstimatorConfig",
        ):
            select_impact_aware_acceleration_replay_intercepts(
                replay,
                estimator_config(),
                selector(),
                object(),
            )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "built-in ConstantVelocityEstimatorConfig",
        ):
            select_impact_aware_acceleration_replay_intercepts(
                replay,
                DerivedEstimatorConfig(**estimator_config().__dict__),
                selector(),
                impact_config(),
            )


if __name__ == "__main__":
    unittest.main()
