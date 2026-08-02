from dataclasses import FrozenInstanceError, replace
from decimal import Decimal
import math
import unittest
from unittest.mock import patch

from ARrobots.dynamic_motion import (
    AxisStateCovariance,
    ConstantVelocityEstimatorConfig,
    ConstantVelocityPredictor,
    DiagonalCovariance3,
    EstimatorUpdate,
    EstimatorUpdateStatus,
    MotionEstimate,
    ObservationReplay,
    PositionObservation,
    ReplayObservation,
    StaleObservationError,
    StateCovariance3,
    Vector3,
)
from ARrobots.interception import (
    INTERCEPT_MAXIMUM_CANDIDATES,
    INTERCEPT_REPLAY_MAXIMUM_CANDIDATE_EVALUATIONS,
    InterceptCandidateEvaluation,
    InterceptFeasibility,
    InterceptFeasibilityStatus,
    InterceptRejectionReason,
    InterceptSelection,
    InterceptSelectionError,
    InterceptSelectionStatus,
    InterceptSelector,
    InterceptSelectorConfig,
    ReplayInterceptStep,
    select_replay_intercepts,
)


def motion_estimate(
    timestamp=1.0,
    position=(0.0, 0.0, 0.0),
    velocity=(1.0, 0.0, 0.0),
    position_variance=(0.01, 0.01, 0.01),
    velocity_variance=(0.0, 0.0, 0.0),
    cross_covariance=(0.0, 0.0, 0.0),
    frame_id="table",
):
    axes = tuple(
        AxisStateCovariance(position_value, velocity_value, cross_value)
        for position_value, velocity_value, cross_value in zip(
            position_variance,
            velocity_variance,
            cross_covariance,
        )
    )
    return MotionEstimate(
        timestamp_seconds=timestamp,
        frame_id=frame_id,
        position=Vector3.from_sequence(position),
        velocity=Vector3.from_sequence(velocity),
        covariance=StateCovariance3(*axes),
        sample_interval_seconds=0.1,
    )


def predictor(maximum_horizon=2.0, process_variance=(0.0, 0.0, 0.0)):
    return ConstantVelocityPredictor(
        maximum_horizon_seconds=maximum_horizon,
        process_position_variance_per_second=DiagonalCovariance3(
            *process_variance
        ),
    )


def selector_config(**overrides):
    values = {
        "frame_id": "table",
        "minimum_lead_time_seconds": 0.2,
        "maximum_lead_time_seconds": 0.8,
        "candidate_interval_seconds": 0.2,
        "maximum_estimate_age_seconds": 0.1,
        "maximum_future_skew_seconds": 0.01,
        "maximum_position_standard_deviation_mm": 10.0,
        "maximum_terminal_speed_mm_per_second": 100.0,
        "minimum_arrival_margin_seconds": 0.05,
    }
    values.update(overrides)
    return InterceptSelectorConfig(**values)


def feasible(minimum_arrival=0.05, risk=0.0):
    return InterceptFeasibility(
        InterceptFeasibilityStatus.FEASIBLE,
        minimum_arrival_time_seconds=minimum_arrival,
        risk_score=risk,
    )


def rejected(status):
    return InterceptFeasibility(status)


def replay_observation(timestamp, position, received_at=None, frame_id="table"):
    return ReplayObservation(
        PositionObservation(
            timestamp_seconds=timestamp,
            frame_id=frame_id,
            position=Vector3.from_sequence(position),
            position_variance=DiagonalCovariance3(0.01, 0.01, 0.01),
        ),
        timestamp + 0.01 if received_at is None else received_at,
    )


def estimator_config(frame_id="table"):
    return ConstantVelocityEstimatorConfig(
        frame_id=frame_id,
        maximum_observation_age_seconds=0.2,
        minimum_sample_interval_seconds=0.01,
        maximum_sample_interval_seconds=0.5,
        maximum_future_skew_seconds=0.01,
    )


class InterceptContractTests(unittest.TestCase):
    def test_feasibility_normalizes_only_complete_feasible_results(self):
        result = InterceptFeasibility(
            InterceptFeasibilityStatus.FEASIBLE,
            Decimal("0.25"),
            Decimal("1.5"),
        )

        self.assertEqual(result.minimum_arrival_time_seconds, 0.25)
        self.assertEqual(result.risk_score, 1.5)
        self.assertEqual(
            rejected(InterceptFeasibilityStatus.UNREACHABLE).status,
            InterceptFeasibilityStatus.UNREACHABLE,
        )
        with self.assertRaises(FrozenInstanceError):
            result.risk_score = 2.0

        invalid = (
            (object(), None, None),
            (InterceptFeasibilityStatus.FEASIBLE, None, 0.0),
            (InterceptFeasibilityStatus.FEASIBLE, 0.1, None),
            (InterceptFeasibilityStatus.FEASIBLE, -0.1, 0.0),
            (InterceptFeasibilityStatus.FEASIBLE, 0.1, math.nan),
            (InterceptFeasibilityStatus.COLLISION, 0.1, None),
            (InterceptFeasibilityStatus.COLLISION, None, 0.0),
        )
        for status, arrival, risk in invalid:
            with self.subTest(status=status, arrival=arrival, risk=risk):
                with self.assertRaises(InterceptSelectionError):
                    InterceptFeasibility(status, arrival, risk)

    def test_config_generates_deterministic_bounded_lead_times(self):
        config = selector_config()

        self.assertEqual(len(config.candidate_lead_times), 4)
        for actual, expected in zip(
            config.candidate_lead_times,
            (0.2, 0.4, 0.6, 0.8),
        ):
            self.assertAlmostEqual(actual, expected)
        with self.assertRaises(FrozenInstanceError):
            config.maximum_lead_time_seconds = 2.0

        invalid_overrides = (
            {"frame_id": "table frame"},
            {"minimum_lead_time_seconds": -0.1},
            {"maximum_lead_time_seconds": -0.1},
            {"candidate_interval_seconds": 0.0},
            {"maximum_estimate_age_seconds": -0.1},
            {"maximum_future_skew_seconds": -0.1},
            {"maximum_position_standard_deviation_mm": -0.1},
            {"maximum_terminal_speed_mm_per_second": -0.1},
            {"minimum_arrival_margin_seconds": -0.1},
            {
                "minimum_lead_time_seconds": 0.5,
                "maximum_lead_time_seconds": 0.4,
            },
            {
                "minimum_lead_time_seconds": 0.005,
                "maximum_future_skew_seconds": 0.01,
            },
            {"candidate_interval_seconds": True},
        )
        for overrides in invalid_overrides:
            with self.subTest(overrides=overrides):
                with self.assertRaises(InterceptSelectionError):
                    selector_config(**overrides)

        with self.assertRaisesRegex(
            InterceptSelectionError,
            "maximum candidate count",
        ):
            selector_config(
                minimum_lead_time_seconds=0.0,
                maximum_lead_time_seconds=float(
                    INTERCEPT_MAXIMUM_CANDIDATES
                ),
                candidate_interval_seconds=1.0,
                maximum_future_skew_seconds=0.0,
            )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "cannot advance",
        ):
            selector_config(
                minimum_lead_time_seconds=1e16,
                maximum_lead_time_seconds=1e16 + 2.0,
                candidate_interval_seconds=1.0,
                maximum_future_skew_seconds=0.0,
            )

    def test_selector_requires_compatible_bounded_dependencies(self):
        config = selector_config()
        valid_predictor = predictor()
        evaluator = lambda prediction, evaluated_at: feasible()
        selector = InterceptSelector(config, valid_predictor, evaluator)

        self.assertIs(selector.config, config)
        self.assertIs(selector.predictor, valid_predictor)
        with self.assertRaises(InterceptSelectionError):
            InterceptSelector(object(), valid_predictor, evaluator)
        with self.assertRaises(InterceptSelectionError):
            InterceptSelector(config, object(), evaluator)
        with self.assertRaises(InterceptSelectionError):
            InterceptSelector(config, valid_predictor, object())
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "does not cover",
        ):
            InterceptSelector(config, predictor(0.89), evaluator)

        huge = selector_config(
            minimum_lead_time_seconds=1e308,
            maximum_lead_time_seconds=1e308,
            candidate_interval_seconds=1e308,
            maximum_estimate_age_seconds=1e308,
            maximum_future_skew_seconds=0.0,
        )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "outside the host range",
        ):
            InterceptSelector(huge, predictor(1e308), evaluator)


class InterceptSelectorTests(unittest.TestCase):
    def test_lowest_risk_candidate_wins_with_earliest_stable_tie_break(self):
        risk_by_timestamp = {
            1.22: 2.0,
            1.42: 1.0,
            1.62: 1.0,
            1.82: 3.0,
        }

        evaluation_timestamps = []

        def evaluate(prediction, evaluated_at):
            evaluation_timestamps.append(evaluated_at)
            return feasible(risk=risk_by_timestamp[round(
                prediction.timestamp_seconds,
                2,
            )])

        selector = InterceptSelector(
            selector_config(),
            predictor(),
            evaluate,
        )
        estimate = motion_estimate()

        first = selector.select(estimate, 1.02)
        second = selector.select(estimate, 1.02)

        self.assertEqual(first, second)
        self.assertIs(first.status, InterceptSelectionStatus.SELECTED)
        self.assertEqual(len(first.candidates), 4)
        self.assertTrue(all(candidate.accepted for candidate in first.candidates))
        self.assertAlmostEqual(
            first.selected_candidate.prediction.timestamp_seconds,
            1.42,
        )
        self.assertEqual(first.selected_candidate.feasibility.risk_score, 1.0)
        self.assertAlmostEqual(first.estimate_age_seconds, 0.02)
        self.assertEqual(evaluation_timestamps, [1.02] * 8)

    def test_uncertainty_and_terminal_speed_reject_before_feasibility(self):
        calls = []

        def evaluate(prediction, evaluated_at):
            calls.append(prediction)
            return feasible()

        uncertain_selector = InterceptSelector(
            selector_config(
                maximum_position_standard_deviation_mm=0.1,
            ),
            predictor(),
            evaluate,
        )
        uncertain_result = uncertain_selector.select(
            motion_estimate(position_variance=(0.04, 0.04, 0.04)),
            1.01,
        )

        self.assertIs(
            uncertain_result.status,
            InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
        )
        self.assertEqual(calls, [])
        self.assertTrue(all(
            candidate.rejection_reason
            is InterceptRejectionReason.UNCERTAINTY_LIMIT
            for candidate in uncertain_result.candidates
        ))

        speed_selector = InterceptSelector(
            selector_config(maximum_terminal_speed_mm_per_second=5.0),
            predictor(),
            evaluate,
        )
        speed_result = speed_selector.select(
            motion_estimate(
                velocity=(6.0, 8.0, 0.0),
                position_variance=(0.0, 0.0, 0.0),
            ),
            1.01,
        )

        self.assertEqual(calls, [])
        self.assertEqual(
            len(speed_result.candidates),
            len(speed_selector.config.candidate_lead_times),
        )
        for candidate, lead_time in zip(
            speed_result.candidates,
            speed_selector.config.candidate_lead_times,
        ):
            self.assertAlmostEqual(
                candidate.prediction.timestamp_seconds,
                1.01 + lead_time,
            )
            self.assertAlmostEqual(
                candidate.terminal_speed_mm_per_second,
                10.0,
            )
        self.assertTrue(all(
            candidate.rejection_reason
            is InterceptRejectionReason.TERMINAL_SPEED_LIMIT
            for candidate in speed_result.candidates
        ))

    def test_terminal_speed_uses_each_predicted_state(self):
        class VaryingVelocityPredictor(ConstantVelocityPredictor):
            def predict(self, estimate, target_timestamp_seconds):
                prediction = super().predict(
                    estimate,
                    target_timestamp_seconds,
                )
                return replace(
                    prediction,
                    velocity=Vector3(
                        10.0 * prediction.horizon_seconds,
                        0.0,
                        0.0,
                    ),
                )

        calls = []

        def evaluate(prediction, evaluated_at):
            calls.append(prediction)
            return feasible()

        selector = InterceptSelector(
            selector_config(
                maximum_terminal_speed_mm_per_second=5.0,
            ),
            VaryingVelocityPredictor(
                maximum_horizon_seconds=2.0,
                process_position_variance_per_second=DiagonalCovariance3(
                    0.0,
                    0.0,
                    0.0,
                ),
            ),
            evaluate,
        )

        result = selector.select(
            motion_estimate(velocity=(0.0, 0.0, 0.0)),
            1.0,
        )

        for candidate, expected_speed in zip(
            result.candidates,
            (2.0, 4.0, 6.0, 8.0),
        ):
            self.assertAlmostEqual(
                candidate.terminal_speed_mm_per_second,
                expected_speed,
            )
        self.assertEqual(
            tuple(candidate.rejection_reason for candidate in result.candidates),
            (
                None,
                None,
                InterceptRejectionReason.TERMINAL_SPEED_LIMIT,
                InterceptRejectionReason.TERMINAL_SPEED_LIMIT,
            ),
        )
        self.assertEqual(len(calls), 2)

    def test_feasibility_rejections_remain_explicit(self):
        expected = {
            InterceptFeasibilityStatus.UNREACHABLE:
                InterceptRejectionReason.UNREACHABLE,
            InterceptFeasibilityStatus.JOINT_LIMIT:
                InterceptRejectionReason.JOINT_LIMIT,
            InterceptFeasibilityStatus.SINGULARITY:
                InterceptRejectionReason.SINGULARITY,
            InterceptFeasibilityStatus.COLLISION:
                InterceptRejectionReason.COLLISION,
            InterceptFeasibilityStatus.ARRIVAL_TIME_UNAVAILABLE:
                InterceptRejectionReason.ARRIVAL_TIME_UNAVAILABLE,
        }
        config = selector_config(
            minimum_lead_time_seconds=0.2,
            maximum_lead_time_seconds=0.2,
        )
        for feasibility_status, reason in expected.items():
            with self.subTest(feasibility_status=feasibility_status):
                selector = InterceptSelector(
                    config,
                    predictor(),
                    lambda prediction, evaluated_at,
                    status=feasibility_status: rejected(status),
                )

                result = selector.select(motion_estimate(), 1.01)

                self.assertIs(
                    result.status,
                    InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                )
                self.assertIs(result.candidates[0].rejection_reason, reason)

    def test_arrival_margin_filters_early_candidates(self):
        selector = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=0.2,
                maximum_lead_time_seconds=0.6,
                candidate_interval_seconds=0.2,
                minimum_arrival_margin_seconds=0.1,
            ),
            predictor(),
            lambda prediction, evaluated_at: feasible(
                minimum_arrival=0.35
            ),
        )

        result = selector.select(motion_estimate(), 1.0)

        self.assertEqual(
            tuple(candidate.rejection_reason for candidate in result.candidates),
            (
                InterceptRejectionReason.INSUFFICIENT_ARRIVAL_MARGIN,
                InterceptRejectionReason.INSUFFICIENT_ARRIVAL_MARGIN,
                None,
            ),
        )
        self.assertAlmostEqual(
            result.selected_candidate.arrival_margin_seconds,
            0.25,
        )

    def test_stale_and_future_estimates_return_without_candidates(self):
        calls = []

        def evaluate(prediction, evaluated_at):
            calls.append(prediction)
            return feasible()

        selector = InterceptSelector(
            selector_config(),
            predictor(),
            evaluate,
        )
        estimate = motion_estimate()

        stale = selector.select(estimate, 1.100001)
        future = selector.select(estimate, 0.989999)

        self.assertIs(stale.status, InterceptSelectionStatus.STALE_ESTIMATE)
        self.assertIs(future.status, InterceptSelectionStatus.FUTURE_ESTIMATE)
        self.assertEqual(stale.candidates, ())
        self.assertEqual(future.candidates, ())
        self.assertIsNone(stale.selected_candidate)
        self.assertEqual(calls, [])

    def test_threshold_boundaries_are_inclusive(self):
        selector = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=0.1,
                maximum_lead_time_seconds=0.1,
                maximum_future_skew_seconds=0.01,
                maximum_position_standard_deviation_mm=0.1,
                maximum_terminal_speed_mm_per_second=5.0,
                minimum_arrival_margin_seconds=0.05,
            ),
            predictor(),
            lambda prediction, evaluated_at: feasible(
                minimum_arrival=0.05
            ),
        )

        result = selector.select(
            motion_estimate(
                velocity=(3.0, 4.0, 0.0),
                position_variance=(0.01, 0.01, 0.01),
            ),
            1.0,
        )

        self.assertIs(result.status, InterceptSelectionStatus.SELECTED)
        self.assertIsNone(result.candidates[0].rejection_reason)

    def test_invalid_runtime_boundaries_fail_closed(self):
        selector = InterceptSelector(
            selector_config(),
            predictor(),
            lambda prediction, evaluated_at: object(),
        )
        estimate = motion_estimate()

        for invalid_estimate, evaluated_at in (
            (object(), 1.0),
            (motion_estimate(frame_id="camera"), 1.0),
            (estimate, True),
            (estimate, math.nan),
        ):
            with self.subTest(
                invalid_estimate=invalid_estimate,
                evaluated_at=evaluated_at,
            ):
                with self.assertRaises(InterceptSelectionError):
                    selector.select(invalid_estimate, evaluated_at)
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "feasibility output is invalid",
        ):
            selector.select(estimate, 1.0)

        def broken_evaluator(prediction, evaluated_at):
            raise RuntimeError("simulation failed")

        broken = InterceptSelector(
            selector_config(),
            predictor(),
            broken_evaluator,
        )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "feasibility evaluation failed",
        ):
            broken.select(estimate, 1.0)

        unrepresentable = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=1.0,
                maximum_lead_time_seconds=1.0,
                maximum_future_skew_seconds=0.0,
            ),
            predictor(),
            lambda prediction, evaluated_at: feasible(),
        )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "prediction failed",
        ):
            unrepresentable.select(
                motion_estimate(
                    position=(1e308, 0.0, 0.0),
                    velocity=(1e308, 0.0, 0.0),
                ),
                1.0,
            )

        excessive_speed = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=0.0,
                maximum_lead_time_seconds=0.0,
                maximum_future_skew_seconds=0.0,
            ),
            predictor(),
            lambda prediction, evaluated_at: feasible(),
        )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "terminal speed is outside",
        ):
            excessive_speed.select(
                motion_estimate(velocity=(1.3e308, 1.3e308, 0.0)),
                1.0,
            )

        overflow = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=1e308,
                maximum_lead_time_seconds=1e308,
                candidate_interval_seconds=1e308,
                maximum_estimate_age_seconds=0.0,
                maximum_future_skew_seconds=0.0,
            ),
            predictor(1e308),
            lambda prediction, evaluated_at: feasible(),
        )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "timestamp is outside the host range",
        ):
            overflow.select(
                motion_estimate(timestamp=1e308),
                1e308,
            )

        indistinguishable = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=0.2,
                maximum_lead_time_seconds=0.4,
                candidate_interval_seconds=0.2,
                maximum_estimate_age_seconds=0.0,
                maximum_future_skew_seconds=0.0,
            ),
            predictor(),
            lambda prediction, evaluated_at: feasible(),
        )
        with self.assertRaisesRegex(
            InterceptSelectionError,
            "timestamps cannot advance",
        ):
            indistinguishable.select(
                motion_estimate(timestamp=1e16),
                1e16,
            )


class InterceptResultValidationTests(unittest.TestCase):
    def setUp(self):
        self.estimate = motion_estimate()
        self.prediction = predictor().predict(self.estimate, 1.2)
        self.feasibility = feasible()
        self.accepted = InterceptCandidateEvaluation(
            prediction=self.prediction,
            maximum_position_standard_deviation_mm=0.1,
            terminal_speed_mm_per_second=1.0,
            feasibility=self.feasibility,
            arrival_margin_seconds=0.15,
            rejection_reason=None,
        )

    def test_candidate_validation_rejects_inconsistent_payloads(self):
        invalid = (
            {"prediction": object()},
            {"maximum_position_standard_deviation_mm": -0.1},
            {"terminal_speed_mm_per_second": math.inf},
            {"arrival_margin_seconds": None},
            {"rejection_reason": InterceptRejectionReason.COLLISION},
            {"rejection_reason": object()},
        )
        for replacements in invalid:
            with self.subTest(replacements=replacements):
                with self.assertRaises(InterceptSelectionError):
                    replace(self.accepted, **replacements)

        threshold_rejection = replace(
            self.accepted,
            feasibility=None,
            arrival_margin_seconds=None,
            rejection_reason=InterceptRejectionReason.UNCERTAINTY_LIMIT,
        )
        self.assertFalse(threshold_rejection.accepted)
        with self.assertRaises(InterceptSelectionError):
            replace(
                threshold_rejection,
                rejection_reason=InterceptRejectionReason.COLLISION,
            )
        with self.assertRaises(InterceptSelectionError):
            replace(threshold_rejection, arrival_margin_seconds=0.1)
        with self.assertRaises(InterceptSelectionError):
            replace(threshold_rejection, feasibility=object())

        collision = InterceptCandidateEvaluation(
            prediction=self.prediction,
            maximum_position_standard_deviation_mm=0.1,
            terminal_speed_mm_per_second=1.0,
            feasibility=rejected(InterceptFeasibilityStatus.COLLISION),
            arrival_margin_seconds=None,
            rejection_reason=InterceptRejectionReason.COLLISION,
        )
        with self.assertRaises(InterceptSelectionError):
            replace(collision, rejection_reason=InterceptRejectionReason.JOINT_LIMIT)
        with self.assertRaises(InterceptSelectionError):
            replace(collision, arrival_margin_seconds=0.1)

    def test_selection_validation_enforces_status_payload_contract(self):
        class BrokenIterator:
            def __iter__(self):
                return self

            def __next__(self):
                raise RuntimeError("iteration failed")

        selected = InterceptSelection(
            InterceptSelectionStatus.SELECTED,
            1.0,
            self.estimate,
            (self.accepted,),
            self.accepted,
        )
        self.assertIs(selected.selected_candidate, self.accepted)

        rejected_candidate = replace(
            self.accepted,
            rejection_reason=(
                InterceptRejectionReason.INSUFFICIENT_ARRIVAL_MARGIN
            ),
        )
        later_candidate = replace(
            self.accepted,
            prediction=predictor().predict(self.estimate, 1.3),
            arrival_margin_seconds=0.25,
        )
        wrong_source_candidate = replace(
            self.accepted,
            prediction=replace(
                self.prediction,
                source_timestamp_seconds=0.9,
            ),
        )
        invalid_constructors = (
            lambda: InterceptSelection(
                object(), 1.0, self.estimate, (), None
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.SELECTED,
                1.0,
                self.estimate,
                (self.accepted,),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.SELECTED,
                1.0,
                self.estimate,
                (self.accepted,),
                replace(self.accepted),
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.SELECTED,
                1.0,
                self.estimate,
                (rejected_candidate,),
                rejected_candidate,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                (),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                (self.accepted,),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.STALE_ESTIMATE,
                1.0,
                self.estimate,
                (rejected_candidate,),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.FUTURE_ESTIMATE,
                1.0,
                object(),
                (),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.SELECTED,
                1.0,
                self.estimate,
                (self.accepted, later_candidate),
                later_candidate,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                (wrong_source_candidate,),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.3,
                self.estimate,
                (rejected_candidate,),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                (rejected_candidate, rejected_candidate),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.STALE_ESTIMATE,
                0.9,
                self.estimate,
                (),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.FUTURE_ESTIMATE,
                1.1,
                self.estimate,
                (),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                (object(),),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                (rejected_candidate,) * (INTERCEPT_MAXIMUM_CANDIDATES + 1),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                "invalid",
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                object(),
                None,
            ),
            lambda: InterceptSelection(
                InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                1.0,
                self.estimate,
                BrokenIterator(),
                None,
            ),
        )
        for constructor in invalid_constructors:
            with self.subTest(constructor=constructor):
                with self.assertRaises(InterceptSelectionError):
                    constructor()


class InterceptReplayTests(unittest.TestCase):
    def test_replay_reselects_after_every_estimate_deterministically(self):
        replay = ObservationReplay((
            replay_observation(1.0, (0.0, 0.0, 0.0)),
            replay_observation(1.1, (1.0, 0.0, 0.0)),
            replay_observation(1.2, (1.5, 1.0, 0.0)),
        ))
        selector = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=0.1,
                maximum_lead_time_seconds=0.3,
                candidate_interval_seconds=0.1,
                maximum_estimate_age_seconds=0.05,
            ),
            predictor(0.4),
            lambda prediction, evaluated_at: feasible(
                minimum_arrival=0.05,
                risk=abs(prediction.position.y),
            ),
        )

        first = select_replay_intercepts(
            replay,
            estimator_config(),
            selector,
        )
        second = select_replay_intercepts(
            replay,
            estimator_config(),
            selector,
        )

        self.assertEqual(first, second)
        self.assertEqual(
            tuple(step.estimator_update.status for step in first),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertIsNone(first[0].selection)
        self.assertTrue(all(
            step.selection.status is InterceptSelectionStatus.SELECTED
            for step in first[1:]
        ))
        self.assertNotEqual(
            first[1].selection.selected_candidate.prediction.position,
            first[2].selection.selected_candidate.prediction.position,
        )
        self.assertEqual(first[2].sample_index, 2)
        self.assertEqual(first[2].received_at_seconds, 1.21)

    def test_replay_baseline_reset_defers_selection_until_next_estimate(self):
        calls = []
        replay = ObservationReplay((
            replay_observation(1.0, (0.0, 0.0, 0.0)),
            replay_observation(1.1, (1.0, 0.0, 0.0)),
            replay_observation(1.7, (2.0, 0.0, 0.0)),
            replay_observation(1.8, (3.0, 0.0, 0.0)),
        ))

        def evaluate(prediction, evaluated_at):
            calls.append((prediction, evaluated_at))
            return feasible()

        selector = InterceptSelector(
            selector_config(),
            predictor(),
            evaluate,
        )

        candidate_count = len(selector.config.candidate_lead_times)
        with patch(
            "ARrobots.interception."
            "INTERCEPT_REPLAY_MAXIMUM_CANDIDATE_EVALUATIONS",
            2 * candidate_count,
        ):
            steps = select_replay_intercepts(
                replay,
                estimator_config(),
                selector,
            )

        self.assertEqual(
            tuple(step.estimator_update.status for step in steps),
            (
                EstimatorUpdateStatus.BASELINE_ACCEPTED,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
                EstimatorUpdateStatus.BASELINE_RESET,
                EstimatorUpdateStatus.ESTIMATE_UPDATED,
            ),
        )
        self.assertIsNone(steps[0].selection)
        self.assertIsNotNone(steps[1].selection)
        self.assertIsNone(steps[2].selection)
        self.assertIsNotNone(steps[3].selection)
        self.assertEqual(
            len(calls),
            2 * candidate_count,
        )

    def test_replay_validates_complete_input_before_feasibility_calls(self):
        calls = []
        replay = ObservationReplay((
            replay_observation(1.0, (0.0, 0.0, 0.0)),
            replay_observation(1.1, (1.0, 0.0, 0.0)),
            replay_observation(
                1.2,
                (2.0, 0.0, 0.0),
                received_at=1.5,
            ),
        ))

        def evaluate(prediction, evaluated_at):
            calls.append(prediction)
            return feasible()

        selector = InterceptSelector(
            selector_config(),
            predictor(),
            evaluate,
        )

        with self.assertRaises(StaleObservationError):
            select_replay_intercepts(
                replay,
                estimator_config(),
                selector,
            )
        self.assertEqual(calls, [])

    def test_replay_bounds_total_candidate_evaluations_before_calls(self):
        calls = []
        candidate_count = INTERCEPT_MAXIMUM_CANDIDATES
        estimate_count = (
            INTERCEPT_REPLAY_MAXIMUM_CANDIDATE_EVALUATIONS
            // candidate_count
            + 1
        )
        replay = ObservationReplay(tuple(
            replay_observation(
                1.0 + index * 0.02,
                (float(index), 0.0, 0.0),
            )
            for index in range(estimate_count + 1)
        ))

        def evaluate(prediction, evaluated_at):
            calls.append(prediction)
            return feasible()

        selector = InterceptSelector(
            selector_config(
                minimum_lead_time_seconds=0.0,
                maximum_lead_time_seconds=float(candidate_count - 1),
                candidate_interval_seconds=1.0,
                maximum_future_skew_seconds=0.0,
            ),
            predictor(float(candidate_count)),
            evaluate,
        )

        with self.assertRaisesRegex(
            InterceptSelectionError,
            "maximum candidate evaluation count",
        ):
            select_replay_intercepts(
                replay,
                estimator_config(),
                selector,
            )
        self.assertEqual(calls, [])

    def test_replay_and_step_boundaries_reject_mismatches(self):
        replay = ObservationReplay((
            replay_observation(1.0, (0.0, 0.0, 0.0)),
            replay_observation(1.1, (1.0, 0.0, 0.0)),
        ))
        selector = InterceptSelector(
            selector_config(),
            predictor(),
            lambda prediction, evaluated_at: feasible(),
        )
        update = replay.run(estimator_config())[0]

        invalid_calls = (
            lambda: select_replay_intercepts(
                object(), estimator_config(), selector
            ),
            lambda: select_replay_intercepts(replay, object(), selector),
            lambda: select_replay_intercepts(
                replay, estimator_config(), object()
            ),
            lambda: select_replay_intercepts(
                replay,
                estimator_config("camera"),
                selector,
            ),
            lambda: select_replay_intercepts(
                replay,
                estimator_config(),
                InterceptSelector(
                    selector_config(frame_id="camera"),
                    predictor(),
                    lambda prediction, evaluated_at: feasible(),
                ),
            ),
            lambda: select_replay_intercepts(
                replay,
                estimator_config("camera"),
                InterceptSelector(
                    selector_config(frame_id="camera"),
                    predictor(),
                    lambda prediction, evaluated_at: feasible(),
                ),
            ),
        )
        for call in invalid_calls:
            with self.subTest(call=call):
                with self.assertRaises(InterceptSelectionError):
                    call()

        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(True, 1.01, update, None)
        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(-1, 1.01, update, None)
        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(0, math.nan, update, None)
        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(0, 1.01, object(), None)
        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(
                0,
                1.01,
                update,
                InterceptSelection(
                    InterceptSelectionStatus.STALE_ESTIMATE,
                    1.01,
                    motion_estimate(),
                    (),
                ),
            )

        estimated_update = replay.run(estimator_config())[1]
        selection = selector.select(estimated_update.estimate, 1.11)
        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(1, 1.11, estimated_update, None)
        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(1, 1.12, estimated_update, selection)
        with self.assertRaises(InterceptSelectionError):
            ReplayInterceptStep(
                1,
                1.11,
                EstimatorUpdate(
                    EstimatorUpdateStatus.ESTIMATE_UPDATED,
                    estimated_update.observation,
                    motion_estimate(),
                ),
                selection,
            )


if __name__ == "__main__":
    unittest.main()
