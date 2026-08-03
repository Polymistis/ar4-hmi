import math
import unittest
from dataclasses import FrozenInstanceError, replace
from decimal import Decimal

from ARrobots.dynamic_motion import (
    ConstantAccelerationPredictor,
    ConstantVelocityEstimatorConfig,
    DiagonalCovariance3,
    EstimatorProcessingError,
    EstimatorUpdateStatus,
    ImpactAwareEstimatorConfig,
    OBSERVATION_REPLAY_MAXIMUM_RECORDS,
    ObservationReplay,
    OutOfOrderObservationError,
    PositionObservation,
    ReplayObservation,
    Vector3,
)
from ARrobots.feedback_replanning import (
    FeedbackReplanEvent,
    FeedbackReplanFault,
    FeedbackReplanFaultPhase,
    FeedbackReplanStatus,
    FeedbackReplanReplayResult,
    FeedbackReplanner,
    FeedbackReplanningError,
    JointReplanTarget,
    run_feedback_replanning_replay,
)
from ARrobots.interception import (
    InterceptFeasibility,
    InterceptFeasibilityStatus,
    InterceptSelector,
    InterceptSelectorConfig,
)
from ARrobots.trajectory_timing import (
    JointBoundaryState,
    JointKinematicLimits,
    SynchronizedJointTrajectory,
    plan_synchronized_rest_to_rest_trajectory,
)


def estimator_config(**overrides):
    values = {
        "frame_id": "table",
        "maximum_observation_age_seconds": 0.2,
        "minimum_sample_interval_seconds": 0.01,
        "maximum_sample_interval_seconds": 0.5,
        "maximum_future_skew_seconds": 0.01,
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


def selector_config(**overrides):
    values = {
        "frame_id": "table",
        "minimum_lead_time_seconds": 0.5,
        "maximum_lead_time_seconds": 0.5,
        "candidate_interval_seconds": 0.1,
        "maximum_estimate_age_seconds": 0.1,
        "maximum_future_skew_seconds": 0.01,
        "maximum_position_standard_deviation_mm": 100.0,
        "maximum_terminal_speed_mm_per_second": 1000.0,
        "minimum_arrival_margin_seconds": 0.05,
    }
    values.update(overrides)
    return InterceptSelectorConfig(**values)


def feasible(_prediction, _evaluated_at):
    return InterceptFeasibility(
        InterceptFeasibilityStatus.FEASIBLE,
        minimum_arrival_time_seconds=0.05,
        risk_score=0.0,
    )


def selector(
    feasibility_evaluator=feasible,
    predictor_horizon=2.0,
    **config_overrides,
):
    predictor = ConstantAccelerationPredictor(
        maximum_horizon_seconds=predictor_horizon,
        process_acceleration_variance_per_second=DiagonalCovariance3(
            0.0,
            0.0,
            0.0,
        ),
    )
    return InterceptSelector(
        selector_config(**config_overrides),
        predictor,
        feasibility_evaluator,
    )


def observation(timestamp, x_position, frame_id="table"):
    return PositionObservation(
        timestamp_seconds=timestamp,
        frame_id=frame_id,
        position=Vector3(x_position, 0.0, 0.0),
        position_variance=DiagonalCovariance3(0.01, 0.01, 0.01),
    )


def replay_sample(
    timestamp,
    x_position,
    received_at=None,
    frame_id="table",
):
    return ReplayObservation(
        observation(timestamp, x_position, frame_id),
        timestamp + 0.01 if received_at is None else received_at,
    )


def joint_limits(
    maximum_velocity=500.0,
    maximum_acceleration=5000.0,
    maximum_jerk=100000.0,
):
    return JointKinematicLimits(
        maximum_velocity,
        maximum_acceleration,
        maximum_jerk,
    )


def stationary_trajectory(limits=None, axis_count=1):
    limits = joint_limits() if limits is None else limits
    return plan_synchronized_rest_to_rest_trajectory(
        (0.0,) * axis_count,
        (0.0,) * axis_count,
        (limits,) * axis_count,
    )


def target_for(selection, positions=None):
    candidate = selection.selected_candidate
    if positions is None:
        positions = (candidate.prediction.position.x,)
    return JointReplanTarget(
        estimate_timestamp_seconds=selection.estimate.timestamp_seconds,
        evaluated_at_seconds=selection.evaluated_at_seconds,
        intercept_timestamp_seconds=candidate.prediction.timestamp_seconds,
        joint_states=tuple(
            JointBoundaryState(position, 0.0, 0.0)
            for position in positions
        ),
    )


def feed_three(replanner, receipts=(1.01, 1.11, 1.21)):
    return tuple(
        replanner.process_observation(
            observation(timestamp, position),
            received_at,
        )
        for timestamp, position, received_at in zip(
            (1.0, 1.1, 1.2),
            (0.0, 1.0, 2.0),
            receipts,
        )
    )


class JointReplanTargetTests(unittest.TestCase):
    def test_normalizes_correlated_finite_target(self):
        target = JointReplanTarget(
            Decimal("1.0"),
            Decimal("1.1"),
            Decimal("1.6"),
            (JointBoundaryState(2, 0, 0),),
        )

        self.assertEqual(target.estimate_timestamp_seconds, 1.0)
        self.assertEqual(target.evaluated_at_seconds, 1.1)
        self.assertEqual(target.intercept_timestamp_seconds, 1.6)
        with self.assertRaises(FrozenInstanceError):
            target.evaluated_at_seconds = 2.0

    def test_rejects_invalid_timestamps_and_joint_sequences(self):
        boundary = JointBoundaryState(0.0, 0.0, 0.0)
        invalid = (
            (
                (True, 1.0, 1.5, (boundary,)),
                "estimate_timestamp_seconds must be numeric",
            ),
            (
                (0.0, math.nan, 1.5, (boundary,)),
                "evaluated_at_seconds must be finite",
            ),
            (
                (0.0, 1.0, 1.0, (boundary,)),
                "intercept timestamp must follow evaluation",
            ),
            (
                (0.0, 1.0, 0.9, (boundary,)),
                "intercept timestamp must follow evaluation",
            ),
            (
                (0.0, 1.0, 1.5, []),
                "joint_states must be a built-in tuple",
            ),
            (
                (0.0, 1.0, 1.5, ()),
                "joint_states must contain at least one state",
            ),
            (
                (0.0, 1.0, 1.5, (boundary,) * 10),
                "joint_states exceeds the supported axis count",
            ),
            (
                (0.0, 1.0, 1.5, (object(),)),
                "joint_states must contain built-in joint boundary states",
            ),
        )
        for values, expected_message in invalid:
            with self.subTest(values=values):
                with self.assertRaisesRegex(
                    FeedbackReplanningError,
                    expected_message,
                ):
                    JointReplanTarget(*values)

    def test_rejects_derived_boundary_before_component_access(self):
        class DerivedBoundary(JointBoundaryState):
            armed = False

            def __getattribute__(self, name):
                if name in {
                    "position_degrees",
                    "velocity_degrees_per_second",
                    "acceleration_degrees_per_second_squared",
                } and object.__getattribute__(self, "armed"):
                    raise AssertionError("derived boundary must not be read")
                return super().__getattribute__(name)

        derived = DerivedBoundary(0.0, 0.0, 0.0)
        object.__setattr__(derived, "armed", True)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "built-in joint boundary states",
        ):
            JointReplanTarget(0.0, 1.0, 1.5, (derived,))


class FeedbackReplannerTests(unittest.TestCase):
    def make_replanner(
        self,
        *,
        resolver=target_for,
        active=None,
        active_started_at=1.0,
        estimator=None,
        intercept_selector=None,
        impact=None,
    ):
        return FeedbackReplanner(
            estimator_config() if estimator is None else estimator,
            selector() if intercept_selector is None else intercept_selector,
            stationary_trajectory() if active is None else active,
            active_started_at,
            resolver,
            impact,
        )

    def test_repeated_updates_replace_from_sampled_desired_state(self):
        resolved_positions = []

        def resolver(selection):
            target = target_for(selection)
            resolved_positions.append(target.joint_states[0].position_degrees)
            return target

        replanner = self.make_replanner(resolver=resolver)
        first_events = feed_three(replanner)

        self.assertEqual(
            tuple(event.status for event in first_events),
            (
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.REPLACED,
            ),
        )
        self.assertEqual(replanner.trajectory_generation, 1)
        first_replacement = replanner.active_trajectory
        first_boundary = first_replacement.states_at(0.1)[0]

        second_event = replanner.process_observation(
            observation(1.3, 3.0),
            1.31,
        )

        self.assertEqual(second_event.status, FeedbackReplanStatus.REPLACED)
        self.assertEqual(second_event.sequence, 3)
        self.assertEqual(second_event.trajectory_generation, 2)
        self.assertIs(
            second_event.replacement_trajectory,
            replanner.active_trajectory,
        )
        second_boundary = replanner.active_trajectory.states_at(0.0)[0]
        self.assertAlmostEqual(
            second_boundary.position_degrees,
            first_boundary.position_degrees,
        )
        self.assertAlmostEqual(
            second_boundary.velocity_degrees_per_second,
            first_boundary.velocity_degrees_per_second,
        )
        self.assertAlmostEqual(
            second_boundary.acceleration_degrees_per_second_squared,
            first_boundary.acceleration_degrees_per_second_squared,
        )
        self.assertEqual(len(resolved_positions), 2)
        self.assertNotEqual(resolved_positions[0], resolved_positions[1])

    def test_impact_filter_invalidates_intercept_until_reacquisition(self):
        replanner = self.make_replanner(impact=impact_config())
        initial_events = tuple(
            replanner.process_observation(
                observation(timestamp, position),
                timestamp + 0.01,
            )
            for timestamp, position in (
                (1.0, 0.0),
                (1.1, 0.01),
                (1.2, 0.04),
            )
        )
        active_after_replacement = replanner.active_trajectory

        rejected = replanner.process_observation(
            observation(1.3, 5.0),
            1.31,
        )
        valid_after_rejection = replanner.active_intercept_valid
        reset = replanner.process_observation(
            observation(1.4, 5.1),
            1.41,
        )
        valid_after_reset = replanner.active_intercept_valid
        warmup = replanner.process_observation(
            observation(1.5, 5.2),
            1.51,
        )
        valid_during_warmup = replanner.active_intercept_valid
        reacquired = replanner.process_observation(
            observation(1.6, 5.3),
            1.61,
        )

        self.assertIs(
            initial_events[-1].status,
            FeedbackReplanStatus.REPLACED,
        )
        self.assertIs(
            rejected.status,
            FeedbackReplanStatus.HOLDING_INNOVATION_REJECTED,
        )
        self.assertIs(
            reset.status,
            FeedbackReplanStatus.HOLDING_AFTER_IMPACT,
        )
        self.assertIs(
            warmup.status,
            FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
        )
        self.assertIs(reacquired.status, FeedbackReplanStatus.REPLACED)
        self.assertIs(
            rejected.estimator_update.status,
            EstimatorUpdateStatus.INNOVATION_REJECTED,
        )
        self.assertIs(
            reset.estimator_update.status,
            EstimatorUpdateStatus.IMPACT_RESET,
        )
        self.assertIs(
            replanner.active_trajectory,
            reacquired.replacement_trajectory,
        )
        self.assertIsNot(
            replanner.active_trajectory,
            active_after_replacement,
        )
        self.assertFalse(valid_after_rejection)
        self.assertFalse(valid_after_reset)
        self.assertFalse(valid_during_warmup)
        self.assertTrue(replanner.active_intercept_valid)
        self.assertEqual(replanner.trajectory_generation, 2)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "does not preserve estimator disposition",
        ):
            replace(
                rejected,
                status=FeedbackReplanStatus.HOLDING_AFTER_IMPACT,
            )

    def test_impact_filter_preserves_accumulated_model_gap_reset(self):
        replanner = self.make_replanner(impact=impact_config())
        self.assertIs(
            feed_three(replanner)[-1].status,
            FeedbackReplanStatus.REPLACED,
        )
        rejected = replanner.process_observation(
            observation(1.3, 50.0),
            1.31,
        )

        reset = replanner.process_observation(
            observation(1.79, 50.1),
            1.80,
        )

        self.assertIs(
            rejected.status,
            FeedbackReplanStatus.HOLDING_INNOVATION_REJECTED,
        )
        self.assertIs(
            reset.status,
            FeedbackReplanStatus.HOLDING_AFTER_ESTIMATOR_RESET,
        )
        self.assertIs(
            reset.estimator_update.status,
            EstimatorUpdateStatus.BASELINE_RESET,
        )
        self.assertFalse(replanner.active_intercept_valid)

    def test_completed_active_trajectory_holds_terminal_state_before_replace(self):
        active = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (1.0,),
            (joint_limits(),),
        )
        replanner = self.make_replanner(
            active=active,
            active_started_at=0.0,
        )

        event = feed_three(replanner)[-1]

        self.assertEqual(event.status, FeedbackReplanStatus.REPLACED)
        self.assertAlmostEqual(
            event.replacement_trajectory.states_at(0.0)[0].position_degrees,
            1.0,
        )

    def test_out_of_order_observation_cannot_replace_newer_plan(self):
        resolver_calls = []

        def resolver(selection):
            resolver_calls.append(selection.estimate.timestamp_seconds)
            return target_for(selection)

        replanner = self.make_replanner(resolver=resolver)
        feed_three(replanner)
        active = replanner.active_trajectory
        generation = replanner.trajectory_generation

        with self.assertRaises(OutOfOrderObservationError):
            replanner.process_observation(
                observation(1.15, 99.0),
                1.22,
            )

        self.assertIs(replanner.active_trajectory, active)
        self.assertEqual(replanner.trajectory_generation, generation)
        self.assertEqual(resolver_calls, [1.2])
        valid = replanner.process_observation(
            observation(1.3, 3.0),
            1.31,
        )
        self.assertEqual(valid.sequence, 3)

    def test_estimator_processing_failure_latches_exact_phase(self):
        replanner = self.make_replanner()
        self.assertIs(
            feed_three(replanner)[-1].status,
            FeedbackReplanStatus.REPLACED,
        )
        active = replanner.active_trajectory
        generation = replanner.trajectory_generation

        def failed_update(_observation, _received_at):
            raise EstimatorProcessingError("estimator numeric failure")

        replanner._estimator.add_observation = failed_update
        event = replanner.process_observation(
            observation(1.3, 3.0),
            1.31,
        )

        self.assertIs(event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            event.fault.phase,
            FeedbackReplanFaultPhase.ESTIMATION,
        )
        self.assertIsNone(event.estimator_update)
        self.assertIsNone(event.selection)
        self.assertIs(replanner.fault, event.fault)
        self.assertIs(replanner.active_trajectory, active)
        self.assertEqual(replanner.trajectory_generation, generation)
        self.assertFalse(replanner.active_intercept_valid)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replanner is faulted",
        ):
            replanner.process_observation(
                observation(1.4, 4.0),
                1.41,
            )

    def test_default_estimator_numeric_failure_latches(self):
        replanner = self.make_replanner()
        first = replanner.process_observation(
            observation(1.0, -1e308),
            1.01,
        )
        second = replanner.process_observation(
            observation(1.1, 1e308),
            1.11,
        )

        faulted = replanner.process_observation(
            observation(1.2, -1e308),
            1.21,
        )

        self.assertIs(
            first.status,
            FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
        )
        self.assertIs(
            second.status,
            FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
        )
        self.assertIs(faulted.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            faulted.fault.phase,
            FeedbackReplanFaultPhase.ESTIMATION,
        )
        self.assertIn("host numeric range", faulted.fault.detail)
        self.assertIs(replanner.fault, faulted.fault)
        self.assertFalse(replanner.active_intercept_valid)

    def test_unexpected_estimator_failure_latches(self):
        replanner = self.make_replanner()

        def failed_update(_observation, _received_at):
            raise RuntimeError("unexpected estimator failure")

        replanner._estimator.add_observation = failed_update
        event = replanner.process_observation(
            observation(1.0, 0.0),
            1.01,
        )

        self.assertIs(event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            event.fault.phase,
            FeedbackReplanFaultPhase.ESTIMATION,
        )
        self.assertIn("unexpected estimator failure", event.fault.detail)
        self.assertIs(replanner.fault, event.fault)

    def test_long_sample_gap_preserves_estimator_reset_disposition(self):
        resolver_calls = []

        def resolver(selection):
            resolver_calls.append(selection.estimate.timestamp_seconds)
            return target_for(selection)

        replanner = self.make_replanner(resolver=resolver)
        feed_three(replanner)
        active = replanner.active_trajectory

        reset = replanner.process_observation(
            observation(2.0, 8.0),
            2.01,
        )
        warmup = replanner.process_observation(
            observation(2.1, 9.0),
            2.11,
        )

        self.assertEqual(
            reset.status,
            FeedbackReplanStatus.HOLDING_AFTER_ESTIMATOR_RESET,
        )
        self.assertEqual(
            reset.estimator_update.status,
            EstimatorUpdateStatus.BASELINE_RESET,
        )
        self.assertEqual(
            warmup.status,
            FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
        )
        self.assertEqual(
            warmup.estimator_update.status,
            EstimatorUpdateStatus.WARMUP_ACCEPTED,
        )
        self.assertIs(replanner.active_trajectory, active)
        self.assertEqual(replanner.trajectory_generation, 1)
        self.assertEqual(resolver_calls, [1.2])

    def test_resolver_receives_deeply_isolated_selection_snapshot(self):
        resolver_prediction = None
        resolver_estimate = None
        selected_timestamp = None
        resolver_candidate_count = None

        def resolver(selection):
            nonlocal resolver_candidate_count
            nonlocal resolver_estimate, resolver_prediction
            nonlocal selected_timestamp
            resolver_candidate_count = len(selection.candidates)
            resolver_estimate = selection.estimate
            resolver_prediction = selection.selected_candidate.prediction
            target = target_for(selection)
            selected_timestamp = target.intercept_timestamp_seconds
            object.__setattr__(
                resolver_estimate,
                "timestamp_seconds",
                resolver_estimate.timestamp_seconds + 1.0,
            )
            object.__setattr__(
                resolver_prediction,
                "timestamp_seconds",
                selected_timestamp + 1.0,
            )
            return target

        event = feed_three(self.make_replanner(
            resolver=resolver,
            intercept_selector=selector(
                maximum_lead_time_seconds=0.7,
                candidate_interval_seconds=0.1,
            ),
        ))[-1]

        self.assertEqual(event.status, FeedbackReplanStatus.REPLACED)
        event_prediction = event.selection.selected_candidate.prediction
        self.assertEqual(resolver_candidate_count, 1)
        self.assertGreater(len(event.selection.candidates), 1)
        self.assertIsNot(resolver_estimate, event.selection.estimate)
        self.assertEqual(event.selection.estimate.timestamp_seconds, 1.2)
        self.assertIsNot(resolver_prediction, event_prediction)
        self.assertEqual(event_prediction.timestamp_seconds, selected_timestamp)

    def test_stale_resolved_target_faults_without_replacing(self):
        initial = stationary_trajectory()

        def stale_target(selection):
            target = target_for(selection)
            return JointReplanTarget(
                estimate_timestamp_seconds=0.0,
                evaluated_at_seconds=target.evaluated_at_seconds,
                intercept_timestamp_seconds=target.intercept_timestamp_seconds,
                joint_states=target.joint_states,
            )

        replanner = self.make_replanner(
            resolver=stale_target,
            active=initial,
        )
        owned_initial = replanner.active_trajectory
        event = feed_three(replanner)[-1]

        self.assertEqual(event.status, FeedbackReplanStatus.FAULTED)
        self.assertEqual(
            event.fault.phase,
            FeedbackReplanFaultPhase.TARGET_RESOLUTION,
        )
        self.assertIn("does not match", event.fault.detail)
        self.assertEqual(replanner.active_trajectory, owned_initial)
        self.assertEqual(replanner.trajectory_generation, 0)
        self.assertIs(replanner.fault, event.fault)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replanner is faulted",
        ):
            replanner.process_observation(observation(1.3, 3.0), 1.31)

    def test_selector_target_and_trajectory_failures_latch_exact_phases(self):
        def selector_failure(_prediction, _evaluated_at):
            raise RuntimeError("feasibility unavailable")

        def target_failure(_selection):
            raise RuntimeError("IK unavailable")

        low_limits = joint_limits(1.0, 1.0, 1.0)

        def infeasible_target(selection):
            return target_for(selection, (100.0,))

        cases = (
            (
                self.make_replanner(
                    intercept_selector=selector(selector_failure),
                ),
                FeedbackReplanFaultPhase.SELECTION,
                "feasibility evaluation failed",
            ),
            (
                self.make_replanner(resolver=target_failure),
                FeedbackReplanFaultPhase.TARGET_RESOLUTION,
                "IK unavailable",
            ),
            (
                self.make_replanner(
                    resolver=infeasible_target,
                    active=stationary_trajectory(low_limits),
                ),
                FeedbackReplanFaultPhase.TRAJECTORY_CONSTRUCTION,
                "limit",
            ),
        )
        for replanner, expected_phase, detail in cases:
            with self.subTest(expected_phase=expected_phase):
                event = feed_three(replanner)[-1]
                self.assertEqual(event.status, FeedbackReplanStatus.FAULTED)
                self.assertEqual(event.fault.phase, expected_phase)
                self.assertIn(detail, event.fault.detail)
                self.assertEqual(replanner.trajectory_generation, 0)

    def test_unhashable_selector_status_faults_without_lookup_leakage(self):
        intercept_selector = selector()
        original_select = intercept_selector.select

        def invalid_select(estimate, evaluated_at):
            selection = original_select(estimate, evaluated_at)
            object.__setattr__(selection, "status", [])
            return selection

        intercept_selector.select = invalid_select
        event = feed_three(self.make_replanner(
            intercept_selector=intercept_selector,
        ))[-1]

        self.assertEqual(event.status, FeedbackReplanStatus.FAULTED)
        self.assertEqual(
            event.fault.phase,
            FeedbackReplanFaultPhase.SELECTION,
        )
        self.assertIn("status", event.fault.detail)

    def test_fault_capture_handles_unprintable_exception(self):
        class UnprintableError(Exception):
            def __str__(self):
                raise RuntimeError("formatting failed")

        def resolver(_selection):
            raise UnprintableError()

        event = feed_three(self.make_replanner(resolver=resolver))[-1]

        self.assertEqual(event.status, FeedbackReplanStatus.FAULTED)
        self.assertEqual(event.fault.detail, "failure detail unavailable")
        self.assertIn("UnprintableError", event.fault.error_type)

    def test_reentrant_resolution_faults_instead_of_overwriting_state(self):
        replanner = None

        def resolver(_selection):
            replanner.process_observation(observation(1.25, 2.5), 1.25)

        replanner = self.make_replanner(resolver=resolver)
        event = feed_three(replanner)[-1]

        self.assertEqual(event.status, FeedbackReplanStatus.FAULTED)
        self.assertEqual(
            event.fault.phase,
            FeedbackReplanFaultPhase.TARGET_RESOLUTION,
        )
        self.assertIn("reentrant", event.fault.detail)
        self.assertEqual(replanner.trajectory_generation, 0)

    def test_derived_target_is_rejected_before_metadata_access(self):
        class DerivedTarget(JointReplanTarget):
            armed = False

            def __getattribute__(self, name):
                if name in {
                    "estimate_timestamp_seconds",
                    "evaluated_at_seconds",
                    "intercept_timestamp_seconds",
                    "joint_states",
                } and object.__getattribute__(self, "armed"):
                    raise AssertionError("derived target must not be read")
                return super().__getattribute__(name)

        def resolver(selection):
            target = target_for(selection)
            derived = DerivedTarget(
                target.estimate_timestamp_seconds,
                target.evaluated_at_seconds,
                target.intercept_timestamp_seconds,
                target.joint_states,
            )
            object.__setattr__(derived, "armed", True)
            return derived

        event = feed_three(self.make_replanner(resolver=resolver))[-1]

        self.assertEqual(event.status, FeedbackReplanStatus.FAULTED)
        self.assertEqual(
            event.fault.phase,
            FeedbackReplanFaultPhase.TARGET_RESOLUTION,
        )
        self.assertIn("built-in JointReplanTarget", event.fault.detail)

    def test_selection_rejections_hold_without_target_resolution(self):
        def rejected(_prediction, _evaluated_at):
            return InterceptFeasibility(
                InterceptFeasibilityStatus.UNREACHABLE
            )

        cases = (
            (
                estimator_config(),
                selector(rejected),
                (1.01, 1.11, 1.21),
                FeedbackReplanStatus.HOLDING_NO_FEASIBLE_CANDIDATE,
            ),
            (
                estimator_config(maximum_observation_age_seconds=0.5),
                selector(maximum_estimate_age_seconds=0.01),
                (1.1, 1.2, 1.3),
                FeedbackReplanStatus.HOLDING_STALE_ESTIMATE,
            ),
            (
                estimator_config(maximum_future_skew_seconds=0.1),
                selector(maximum_future_skew_seconds=0.01),
                (0.95, 1.05, 1.15),
                FeedbackReplanStatus.HOLDING_FUTURE_ESTIMATE,
            ),
        )
        for estimator, intercept_selector, receipts, expected_status in cases:
            resolver_calls = []

            def resolver(selection):
                resolver_calls.append(selection)
                return target_for(selection)

            with self.subTest(expected_status=expected_status):
                replanner = self.make_replanner(
                    resolver=resolver,
                    estimator=estimator,
                    intercept_selector=intercept_selector,
                    active_started_at=0.9,
                )
                event = feed_three(replanner, receipts)[-1]
                self.assertEqual(event.status, expected_status)
                self.assertEqual(resolver_calls, [])
                self.assertEqual(replanner.trajectory_generation, 0)
                self.assertIsNone(event.replacement_trajectory)
                self.assertFalse(replanner.active_intercept_valid)

    def test_selection_holds_invalidate_an_existing_intercept(self):
        permit_candidate = True

        def conditional_feasibility(_prediction, _evaluated_at):
            if permit_candidate:
                return feasible(_prediction, _evaluated_at)
            return InterceptFeasibility(
                InterceptFeasibilityStatus.UNREACHABLE
            )

        no_feasible = self.make_replanner(
            intercept_selector=selector(conditional_feasibility),
        )
        self.assertIs(
            feed_three(no_feasible)[-1].status,
            FeedbackReplanStatus.REPLACED,
        )
        self.assertTrue(no_feasible.active_intercept_valid)
        permit_candidate = False
        no_feasible_event = no_feasible.process_observation(
            observation(1.3, 3.0),
            1.31,
        )

        stale = self.make_replanner(
            estimator=estimator_config(
                maximum_observation_age_seconds=0.5,
            ),
            intercept_selector=selector(
                maximum_estimate_age_seconds=0.01,
            ),
        )
        self.assertIs(
            feed_three(stale)[-1].status,
            FeedbackReplanStatus.REPLACED,
        )
        self.assertTrue(stale.active_intercept_valid)
        stale_event = stale.process_observation(
            observation(1.3, 3.0),
            1.32,
        )

        future = self.make_replanner(
            estimator=estimator_config(
                maximum_future_skew_seconds=0.1,
            ),
            intercept_selector=selector(
                maximum_future_skew_seconds=0.01,
            ),
        )
        self.assertIs(
            feed_three(future)[-1].status,
            FeedbackReplanStatus.REPLACED,
        )
        self.assertTrue(future.active_intercept_valid)
        future_event = future.process_observation(
            observation(1.3, 3.0),
            1.25,
        )

        self.assertIs(
            no_feasible_event.status,
            FeedbackReplanStatus.HOLDING_NO_FEASIBLE_CANDIDATE,
        )
        self.assertFalse(no_feasible.active_intercept_valid)
        self.assertIs(
            stale_event.status,
            FeedbackReplanStatus.HOLDING_STALE_ESTIMATE,
        )
        self.assertFalse(stale.active_intercept_valid)
        self.assertIs(
            future_event.status,
            FeedbackReplanStatus.HOLDING_FUTURE_ESTIMATE,
        )
        self.assertFalse(future.active_intercept_valid)

    def test_fault_invalidates_an_existing_intercept(self):
        resolver_calls = 0

        def resolver(selection):
            nonlocal resolver_calls
            resolver_calls += 1
            if resolver_calls > 1:
                raise RuntimeError("IK unavailable")
            return target_for(selection)

        replanner = self.make_replanner(resolver=resolver)
        self.assertIs(
            feed_three(replanner)[-1].status,
            FeedbackReplanStatus.REPLACED,
        )
        self.assertTrue(replanner.active_intercept_valid)

        event = replanner.process_observation(
            observation(1.3, 3.0),
            1.31,
        )

        self.assertIs(event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            event.fault.phase,
            FeedbackReplanFaultPhase.TARGET_RESOLUTION,
        )
        self.assertFalse(replanner.active_intercept_valid)

    def test_cancel_is_logical_terminal_event_without_trajectory_mutation(self):
        replanner = self.make_replanner()
        events = feed_three(replanner)
        active = replanner.active_trajectory
        self.assertTrue(replanner.active_intercept_valid)

        event = replanner.cancel(1.22)

        self.assertEqual(events[0].sequence, 0)
        self.assertEqual(event.sequence, 3)
        self.assertEqual(event.status, FeedbackReplanStatus.CANCELLED)
        self.assertTrue(replanner.cancelled)
        self.assertFalse(replanner.active_intercept_valid)
        self.assertIs(replanner.active_trajectory, active)
        self.assertEqual(replanner.trajectory_generation, 1)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replanner is cancelled",
        ):
            replanner.process_observation(observation(1.1, 1.0), 1.11)

    def test_event_payload_contract_rejects_inconsistent_dispositions(self):
        replaced = feed_three(self.make_replanner())[-1]
        estimation_fault = FeedbackReplanFault(
            FeedbackReplanFaultPhase.ESTIMATION,
            "ARrobots.dynamic_motion.EstimatorProcessingError",
            "estimation failed",
        )
        selection_fault = FeedbackReplanFault(
            FeedbackReplanFaultPhase.SELECTION,
            "builtins.RuntimeError",
            "selection failed",
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "estimate hold event has inconsistent processing output",
        ):
            replace(
                replaced,
                status=FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
            )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "selection fault requires only an estimated update",
        ):
            replace(
                replaced,
                status=FeedbackReplanStatus.FAULTED,
                replacement_trajectory=None,
                fault=selection_fault,
            )
        estimation_event = FeedbackReplanEvent(
            sequence=0,
            evaluated_at_seconds=1.0,
            status=FeedbackReplanStatus.FAULTED,
            trajectory_generation=0,
            fault=estimation_fault,
        )
        self.assertIs(
            estimation_event.fault.phase,
            FeedbackReplanFaultPhase.ESTIMATION,
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "estimation fault must not carry processing output",
        ):
            replace(
                estimation_event,
                estimator_update=replaced.estimator_update,
            )

        cancelled_replanner = self.make_replanner()
        cancelled = cancelled_replanner.cancel(1.0)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "cancelled event must not carry processing output",
        ):
            replace(
                cancelled,
                estimator_update=replaced.estimator_update,
            )

        reset_replanner = self.make_replanner()
        feed_three(reset_replanner)
        reset = reset_replanner.process_observation(
            observation(2.0, 8.0),
            2.01,
        )
        warmup = reset_replanner.process_observation(
            observation(2.1, 9.0),
            2.11,
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "does not preserve estimator disposition",
        ):
            replace(
                reset,
                status=FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
            )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "does not preserve estimator disposition",
        ):
            replace(
                warmup,
                status=(
                    FeedbackReplanStatus.HOLDING_AFTER_ESTIMATOR_RESET
                ),
            )

    def test_constructor_rejects_boundary_and_active_state_mismatches(self):
        active = stationary_trajectory()

        class DerivedTrajectory(SynchronizedJointTrajectory):
            pass

        derived = DerivedTrajectory(active.axes)
        invalid = (
            (
                (object(), selector(), active, 1.0, target_for),
                "estimator_config must be built-in",
            ),
            (
                (estimator_config(), object(), active, 1.0, target_for),
                "selector must be a built-in",
            ),
            (
                (estimator_config(), selector(), derived, 1.0, target_for),
                "active trajectory must be a synchronized built-in trajectory",
            ),
            (
                (estimator_config(), selector(), active, -1.0, target_for),
                "started_at_seconds must be non-negative",
            ),
            (
                (estimator_config(), selector(), active, 1.0, object()),
                "target_resolver must be callable",
            ),
            (
                (
                    estimator_config(frame_id="camera"),
                    selector(),
                    active,
                    1.0,
                    target_for,
                ),
                "selector and estimator frames must match",
            ),
            (
                (
                    estimator_config(),
                    selector(),
                    active,
                    1.0,
                    target_for,
                    object(),
                ),
                "impact_config must be built-in",
            ),
        )
        for values, expected_message in invalid:
            with self.subTest(values=values):
                with self.assertRaisesRegex(
                    FeedbackReplanningError,
                    expected_message,
                ):
                    FeedbackReplanner(*values)

        tampered = stationary_trajectory()
        object.__setattr__(tampered, "axes", tampered.axes * 10)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "supported axis count",
        ):
            self.make_replanner(active=tampered)

        tampered_limits = stationary_trajectory()
        object.__setattr__(
            tampered_limits.axes[0].limits,
            "maximum_velocity_degrees_per_second",
            math.nan,
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "state failed validation",
        ):
            self.make_replanner(active=tampered_limits)

        tampered_estimator = estimator_config()
        object.__setattr__(
            tampered_estimator,
            "maximum_observation_age_seconds",
            math.nan,
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "config failed validation",
        ):
            self.make_replanner(estimator=tampered_estimator)

        tampered_selector = selector()
        object.__setattr__(
            tampered_selector.config,
            "candidate_lead_times",
            (0.4,),
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "candidate schedule failed validation",
        ):
            self.make_replanner(intercept_selector=tampered_selector)

        tampered_impact = impact_config()
        object.__setattr__(
            tampered_impact,
            "maximum_axis_standardized_innovation",
            math.nan,
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "impact estimator configuration failed validation",
        ):
            self.make_replanner(impact=tampered_impact)

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "positive candidate lead times",
        ):
            self.make_replanner(intercept_selector=selector(
                minimum_lead_time_seconds=0.0,
                maximum_future_skew_seconds=0.0,
            ))


class FeedbackReplanningReplayTests(unittest.TestCase):
    def test_replay_runs_repeated_replacement_to_completion(self):
        replay = ObservationReplay(tuple(
            replay_sample(timestamp, position)
            for timestamp, position in zip(
                (1.0, 1.1, 1.2, 1.3, 1.4),
                (0.0, 1.0, 2.0, 3.0, 4.0),
            )
        ))

        result = run_feedback_replanning_replay(
            replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            target_for,
        )

        self.assertTrue(result.complete)
        self.assertTrue(result.processed_all_samples)
        self.assertFalse(result.faulted)
        self.assertEqual(len(result.events), 5)
        self.assertEqual(result.trajectory_generation, 3)
        self.assertEqual(
            tuple(event.sequence for event in result.events),
            (0, 1, 2, 3, 4),
        )
        self.assertEqual(
            tuple(event.status for event in result.events),
            (
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.REPLACED,
                FeedbackReplanStatus.REPLACED,
                FeedbackReplanStatus.REPLACED,
            ),
        )
        self.assertIs(
            result.active_trajectory,
            result.events[-1].replacement_trajectory,
        )

    def test_impact_replay_invalidates_and_reacquires_intercept(self):
        samples = (
            replay_sample(1.0, 0.0),
            replay_sample(1.1, 0.01),
            replay_sample(1.2, 0.04),
            replay_sample(1.3, 5.0),
            replay_sample(1.4, 5.1),
            replay_sample(1.5, 5.2),
            replay_sample(1.6, 5.3),
        )
        replay = ObservationReplay(samples)

        result = run_feedback_replanning_replay(
            replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            target_for,
            impact_config(),
        )
        invalidated = run_feedback_replanning_replay(
            ObservationReplay(samples[:5]),
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            target_for,
            impact_config(),
        )

        self.assertEqual(
            tuple(event.status for event in result.events),
            (
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.REPLACED,
                FeedbackReplanStatus.HOLDING_INNOVATION_REJECTED,
                FeedbackReplanStatus.HOLDING_AFTER_IMPACT,
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.REPLACED,
            ),
        )
        self.assertTrue(result.active_intercept_valid)
        self.assertEqual(result.trajectory_generation, 2)
        self.assertFalse(invalidated.active_intercept_valid)
        self.assertEqual(invalidated.trajectory_generation, 1)

    def test_replay_exposes_terminal_fault_and_processed_prefix(self):
        replay = ObservationReplay((
            replay_sample(1.0, 0.0),
            replay_sample(1.1, 1.0),
            replay_sample(1.2, 2.0),
            replay_sample(1.3, 3.0),
            # The terminal resolver fault precedes this stale suffix sample.
            replay_sample(1.4, 4.0, received_at=2.0),
        ))
        call_count = 0

        def resolver(selection):
            nonlocal call_count
            call_count += 1
            if call_count == 2:
                raise RuntimeError("second IK failed")
            return target_for(selection)

        result = run_feedback_replanning_replay(
            replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            resolver,
        )

        self.assertFalse(result.complete)
        self.assertFalse(result.processed_all_samples)
        self.assertTrue(result.faulted)
        self.assertFalse(result.active_intercept_valid)
        self.assertEqual(len(result.events), 4)
        self.assertEqual(result.events[-1].status, FeedbackReplanStatus.FAULTED)
        self.assertEqual(result.trajectory_generation, 1)

    def test_last_sample_fault_is_processed_but_not_complete(self):
        replay = ObservationReplay((
            replay_sample(1.0, 0.0),
            replay_sample(1.1, 1.0),
            replay_sample(1.2, 2.0),
        ))

        def resolver(_selection):
            raise RuntimeError("terminal IK failed")

        result = run_feedback_replanning_replay(
            replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            resolver,
        )

        self.assertTrue(result.processed_all_samples)
        self.assertTrue(result.faulted)
        self.assertFalse(result.complete)
        self.assertFalse(result.active_intercept_valid)
        self.assertEqual(len(result.events), len(replay.samples))

    def test_replay_returns_processed_prefix_for_estimation_fault(self):
        replay = ObservationReplay((
            replay_sample(1.0, 1e308),
            replay_sample(1.1, 1e308),
            replay_sample(1.2, 1e308),
            replay_sample(1.3, -1e308),
        ))

        result = run_feedback_replanning_replay(
            replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            lambda selection: target_for(selection, (0.0,)),
            impact_config(),
        )

        self.assertTrue(result.processed_all_samples)
        self.assertTrue(result.faulted)
        self.assertFalse(result.complete)
        self.assertFalse(result.active_intercept_valid)
        self.assertIs(
            result.events[-1].fault.phase,
            FeedbackReplanFaultPhase.ESTIMATION,
        )
        self.assertEqual(
            tuple(event.status for event in result.events),
            (
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.REPLACED,
                FeedbackReplanStatus.FAULTED,
            ),
        )

    def test_replay_validates_preconditions_and_candidate_workload(self):
        camera_replay = ObservationReplay((
            replay_sample(1.0, 0.0, frame_id="camera"),
        ))
        resolver_calls = []

        def resolver(selection):
            resolver_calls.append(selection)
            return target_for(selection)

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replay and estimator frames must match",
        ):
            run_feedback_replanning_replay(
                camera_replay,
                estimator_config(),
                selector(),
                stationary_trajectory(),
                1.0,
                resolver,
            )

        table_replay = ObservationReplay((replay_sample(1.0, 0.0),))
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replay estimator_config must be built-in",
        ):
            run_feedback_replanning_replay(
                table_replay,
                object(),
                selector(),
                stationary_trajectory(),
                1.0,
                resolver,
            )

        unavailable_estimator = estimator_config()
        object.__delattr__(unavailable_estimator, "frame_id")
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replay estimator frame is unavailable",
        ):
            run_feedback_replanning_replay(
                table_replay,
                unavailable_estimator,
                selector(),
                stationary_trajectory(),
                1.0,
                resolver,
            )

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replay selector must be a built-in",
        ):
            run_feedback_replanning_replay(
                table_replay,
                estimator_config(),
                object(),
                stationary_trajectory(),
                1.0,
                resolver,
            )

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replay impact_config must be built-in",
        ):
            run_feedback_replanning_replay(
                table_replay,
                estimator_config(),
                selector(),
                stationary_trajectory(),
                1.0,
                resolver,
                object(),
            )

        unavailable_selector = selector()
        object.__delattr__(unavailable_selector, "_config")
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "candidate schedule is unavailable",
        ):
            run_feedback_replanning_replay(
                table_replay,
                estimator_config(),
                unavailable_selector,
                stationary_trajectory(),
                1.0,
                resolver,
            )

        invalid_schedule_selector = selector()
        object.__setattr__(
            invalid_schedule_selector.config,
            "candidate_lead_times",
            [],
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "candidate schedule is invalid",
        ):
            run_feedback_replanning_replay(
                table_replay,
                estimator_config(),
                invalid_schedule_selector,
                stationary_trajectory(),
                1.0,
                resolver,
            )

        wide_selector = selector(
            predictor_horizon=50.0,
            minimum_lead_time_seconds=0.01,
            maximum_lead_time_seconds=40.96,
            candidate_interval_seconds=0.01,
        )
        self.assertEqual(len(wide_selector.config.candidate_lead_times), 4096)
        workload_replay = ObservationReplay(tuple(
            replay_sample(1.0 + index * 0.1, float(index))
            for index in range(27)
        ))
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "maximum candidate evaluation count",
        ):
            run_feedback_replanning_replay(
                workload_replay,
                estimator_config(),
                wide_selector,
                stationary_trajectory(),
                1.0,
                resolver,
            )
        self.assertEqual(resolver_calls, [])

        # Intervals above the estimator maximum force resets, keeping the
        # admitted-boundary test independent of candidate prediction cost.
        reset_replay = ObservationReplay(tuple(
            replay_sample(1.0 + index * 0.6, float(index))
            for index in range(26)
        ))
        admitted = run_feedback_replanning_replay(
            reset_replay,
            estimator_config(),
            wide_selector,
            stationary_trajectory(),
            1.0,
            resolver,
        )
        self.assertTrue(admitted.complete)
        self.assertEqual(len(admitted.events), 26)
        self.assertEqual(resolver_calls, [])

    def test_replay_result_rejects_every_inconsistent_boundary(self):
        replay = ObservationReplay(tuple(
            replay_sample(timestamp, position)
            for timestamp, position in zip(
                (1.0, 1.1, 1.2, 1.3),
                (0.0, 1.0, 2.0, 3.0),
            )
        ))
        result = run_feedback_replanning_replay(
            replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            target_for,
        )

        invalid_replacements = (
            (
                {"events": list(result.events)},
                "events must be a non-empty built-in tuple",
            ),
            (
                {"events": ()},
                "events must be a non-empty built-in tuple",
            ),
            (
                {"replay_sample_count": 0},
                "event count is invalid",
            ),
            (
                {"replay_sample_count": True},
                "sample count must be an integer",
            ),
            (
                {"replay_sample_count": -1},
                "sample count must be non-negative",
            ),
            (
                {"replay_sample_count": len(result.events) - 1},
                "event count is invalid",
            ),
            (
                {
                    "replay_sample_count":
                        OBSERVATION_REPLAY_MAXIMUM_RECORDS + 1,
                },
                "exceeds the maximum sample count",
            ),
            (
                {
                    "events": (object(),),
                    "replay_sample_count": 1,
                    "trajectory_generation": 0,
                },
                "contains an invalid event",
            ),
            (
                {
                    "events": (
                        replace(result.events[0], sequence=3),
                        *result.events[1:],
                    ),
                },
                "event sequences must be contiguous",
            ),
            (
                {
                    "events": (
                        result.events[0],
                        replace(
                            result.events[1],
                            evaluated_at_seconds=0.5,
                        ),
                        *result.events[2:],
                    ),
                },
                "timestamps must not move backward",
            ),
            (
                {"events": result.events[:2]},
                "partial replay result requires a terminal fault event",
            ),
            (
                {
                    "events": (
                        *result.events[:2],
                        replace(
                            result.events[2],
                            trajectory_generation=2,
                        ),
                        result.events[3],
                    ),
                },
                "event generation sequence is invalid",
            ),
            (
                {"trajectory_generation": True},
                "trajectory_generation must be an integer",
            ),
            (
                {"trajectory_generation": -1},
                "trajectory_generation must be non-negative",
            ),
            (
                {
                    "trajectory_generation":
                        result.trajectory_generation + 1,
                },
                "generation does not match replacement events",
            ),
            (
                {"active_trajectory": object()},
                "active trajectory must be a synchronized built-in trajectory",
            ),
            (
                {"active_trajectory": stationary_trajectory()},
                "active trajectory does not match the last replacement",
            ),
        )
        for replacement_values, expected_message in invalid_replacements:
            with self.subTest(replacement_values=replacement_values):
                with self.assertRaisesRegex(
                    FeedbackReplanningError,
                    expected_message,
                ):
                    replace(result, **replacement_values)

        fault_replay = ObservationReplay((
            replay_sample(1.0, 0.0),
            replay_sample(1.1, 1.0),
            replay_sample(1.2, 2.0),
            replay_sample(1.3, 3.0),
        ))

        def fail_resolution(_selection):
            raise RuntimeError("IK failed")

        faulted_result = run_feedback_replanning_replay(
            fault_replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            fail_resolution,
        )
        extra_hold = replace(
            result.events[0],
            sequence=len(faulted_result.events),
            evaluated_at_seconds=1.31,
        )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "fault event must be terminal",
        ):
            replace(
                faulted_result,
                events=(*faulted_result.events, extra_hold),
            )

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "must not contain cancellation events",
        ):
            FeedbackReplanReplayResult(
                events=(FeedbackReplanEvent(
                    sequence=0,
                    evaluated_at_seconds=1.0,
                    status=FeedbackReplanStatus.CANCELLED,
                    trajectory_generation=0,
                ),),
                replay_sample_count=1,
                active_trajectory=stationary_trajectory(),
                trajectory_generation=0,
            )

    def test_replay_and_fault_value_boundaries_fail_closed(self):
        replay = ObservationReplay((replay_sample(1.0, 0.0),))
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "built-in ObservationReplay",
        ):
            run_feedback_replanning_replay(
                object(),
                estimator_config(),
                selector(),
                stationary_trajectory(),
                1.0,
                target_for,
            )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "fault phase is invalid",
        ):
            FeedbackReplanFault(object(), "RuntimeError", "failure")
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "fault detail must be non-empty text without NUL",
        ):
            FeedbackReplanFault(
                FeedbackReplanFaultPhase.SELECTION,
                "RuntimeError",
                "",
            )
        result = run_feedback_replanning_replay(
            replay,
            estimator_config(),
            selector(),
            stationary_trajectory(),
            1.0,
            target_for,
        )
        self.assertTrue(result.complete)
        self.assertEqual(
            result.events[0].status,
            FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
        )


if __name__ == "__main__":
    unittest.main()
