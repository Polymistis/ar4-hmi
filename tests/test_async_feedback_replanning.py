import math
import unittest
from dataclasses import replace

from ARrobots.dynamic_motion import OutOfOrderObservationError
from ARrobots.feedback_replanning import (
    AsynchronousFeedbackReplanResult,
    AsynchronousFeedbackReplanner,
    FeedbackReplanFaultPhase,
    FeedbackReplanReplayResult,
    FeedbackReplanResolutionRequest,
    FeedbackReplanStatus,
    FeedbackReplanningError,
)
from ARrobots.interception import (
    InterceptCandidateEvaluation,
    InterceptFeasibility,
    InterceptFeasibilityStatus,
)
from ARrobots.trajectory_timing import (
    plan_synchronized_rest_to_rest_trajectory,
)
from tests.test_feedback_replanning import (
    estimator_config,
    feasible,
    joint_limits,
    observation,
    selector,
    stationary_trajectory,
    target_for,
)


def asynchronous_replanner(
    *,
    active=None,
    intercept_selector=None,
    active_started_at=1.0,
):
    return AsynchronousFeedbackReplanner(
        estimator_config(),
        selector() if intercept_selector is None else intercept_selector,
        stationary_trajectory() if active is None else active,
        active_started_at,
    )


def feed_to_request(replanner):
    return tuple(
        replanner.process_observation(
            observation(timestamp, position),
            received_at,
        )
        for timestamp, position, received_at in zip(
            (1.0, 1.1, 1.2),
            (0.0, 1.0, 2.0),
            (1.01, 1.11, 1.21),
        )
    )


class AsynchronousFeedbackReplannerTests(unittest.TestCase):
    def test_selected_observation_dispatches_isolated_request(self):
        replanner = asynchronous_replanner(intercept_selector=selector(
            minimum_lead_time_seconds=0.4,
            maximum_lead_time_seconds=0.6,
            candidate_interval_seconds=0.1,
        ))

        results = feed_to_request(replanner)
        pending = results[-1]
        request = pending.resolution_request

        self.assertEqual(
            tuple(result.event.status for result in results),
            (
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
                FeedbackReplanStatus.AWAITING_TARGET_RESOLUTION,
            ),
        )
        self.assertIsInstance(pending, AsynchronousFeedbackReplanResult)
        self.assertIsInstance(request, FeedbackReplanResolutionRequest)
        self.assertEqual(request.request_sequence, 0)
        self.assertEqual(request.issued_at_seconds, 1.21)
        self.assertEqual(request.trajectory_generation, 0)
        self.assertEqual(request.axis_count, 1)
        self.assertEqual(len(pending.event.selection.candidates), 3)
        self.assertEqual(len(request.selection.candidates), 1)
        self.assertEqual(
            request.selection.selected_candidate,
            pending.event.selection.selected_candidate,
        )
        self.assertFalse(pending.active_intercept_valid)
        self.assertFalse(replanner.active_intercept_valid)
        self.assertEqual(replanner.trajectory_generation, 0)
        self.assertIsNot(
            request.selection,
            replanner.pending_resolution_request.selection,
        )

    def test_derived_selector_candidate_faults_before_metadata_access(self):
        class DerivedCandidate(InterceptCandidateEvaluation):
            armed = False

            def __getattribute__(self, name):
                if name in {
                    "prediction",
                    "maximum_position_standard_deviation_mm",
                    "terminal_speed_mm_per_second",
                    "feasibility",
                    "arrival_margin_seconds",
                    "rejection_reason",
                } and object.__getattribute__(self, "armed"):
                    raise AssertionError("derived candidate must not be read")
                return super().__getattribute__(name)

        intercept_selector = selector()
        original_select = intercept_selector.select

        def derived_select(estimate, evaluated_at):
            selection = original_select(estimate, evaluated_at)
            candidate = selection.selected_candidate
            derived = DerivedCandidate(
                candidate.prediction,
                candidate.maximum_position_standard_deviation_mm,
                candidate.terminal_speed_mm_per_second,
                candidate.feasibility,
                candidate.arrival_margin_seconds,
                candidate.rejection_reason,
            )
            object.__setattr__(derived, "armed", True)
            object.__setattr__(selection, "candidates", (derived,))
            object.__setattr__(selection, "selected_candidate", derived)
            return selection

        intercept_selector.select = derived_select
        result = feed_to_request(asynchronous_replanner(
            intercept_selector=intercept_selector,
        ))[-1]

        self.assertIs(result.event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            result.event.fault.phase,
            FeedbackReplanFaultPhase.SELECTION,
        )
        self.assertIn("non-built-in candidate", result.event.fault.detail)
        self.assertIsNone(result.resolution_request)

    def test_current_completion_replaces_at_completion_time(self):
        active = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (100.0,),
            (joint_limits(),),
        )
        replanner = asynchronous_replanner(active=active)
        pending = feed_to_request(replanner)[-1]
        request = pending.resolution_request
        expected_start = active.states_at(0.25)[0]

        completed = replanner.complete_resolution(
            request.request_sequence,
            target_for(request.selection, (100.0,)),
            1.25,
        )

        self.assertIs(
            completed.event.status,
            FeedbackReplanStatus.REPLACED_AFTER_TARGET_RESOLUTION,
        )
        self.assertEqual(completed.event.sequence, 3)
        self.assertEqual(completed.event.trajectory_generation, 1)
        self.assertEqual(completed.event.resolution_request_sequence, 0)
        self.assertEqual(len(completed.event.selection.candidates), 1)
        self.assertTrue(completed.active_intercept_valid)
        self.assertTrue(replanner.active_intercept_valid)
        self.assertEqual(replanner.active_started_at_seconds, 1.25)
        self.assertEqual(replanner.trajectory_generation, 1)
        self.assertIsNone(replanner.pending_resolution_request)
        replacement_start = replanner.active_trajectory.states_at(0.0)[0]
        self.assertAlmostEqual(
            replacement_start.position_degrees,
            expected_start.position_degrees,
        )
        self.assertAlmostEqual(
            replacement_start.velocity_degrees_per_second,
            expected_start.velocity_degrees_per_second,
        )
        self.assertAlmostEqual(
            replacement_start.acceleration_degrees_per_second_squared,
            expected_start.acceleration_degrees_per_second_squared,
        )

    def test_new_request_supersedes_old_completion_without_target_access(self):
        class PoisonTarget:
            def __getattribute__(self, name):
                raise AssertionError(f"unexpected target access: {name}")

        replanner = asynchronous_replanner()
        first = feed_to_request(replanner)[-1].resolution_request
        second_result = replanner.process_observation(
            observation(1.3, 3.0),
            1.31,
        )
        second = second_result.resolution_request

        superseded = replanner.complete_resolution(0, PoisonTarget(), 1.32)

        self.assertEqual(second.request_sequence, 1)
        self.assertIs(
            superseded.event.status,
            FeedbackReplanStatus.SUPERSEDED_TARGET_RESOLUTION,
        )
        self.assertEqual(
            replanner.pending_resolution_request.request_sequence,
            second.request_sequence,
        )
        self.assertEqual(replanner.trajectory_generation, 0)
        self.assertFalse(superseded.active_intercept_valid)

        accepted = replanner.complete_resolution(
            second.request_sequence,
            target_for(second.selection),
            1.33,
        )
        duplicate = replanner.complete_resolution(0, PoisonTarget(), 1.34)

        self.assertTrue(accepted.active_intercept_valid)
        self.assertIs(
            duplicate.event.status,
            FeedbackReplanStatus.SUPERSEDED_TARGET_RESOLUTION,
        )
        self.assertTrue(duplicate.active_intercept_valid)
        self.assertTrue(replanner.active_intercept_valid)
        self.assertEqual(replanner.trajectory_generation, 1)

    def test_selection_hold_supersedes_pending_request(self):
        permit_candidate = True

        def conditional_feasibility(prediction, evaluated_at):
            if permit_candidate:
                return feasible(prediction, evaluated_at)
            return InterceptFeasibility(
                InterceptFeasibilityStatus.UNREACHABLE
            )

        replanner = asynchronous_replanner(
            intercept_selector=selector(conditional_feasibility),
        )
        request = feed_to_request(replanner)[-1].resolution_request
        permit_candidate = False

        held = replanner.process_observation(
            observation(1.3, 3.0),
            1.31,
        )
        late = replanner.complete_resolution(
            request.request_sequence,
            object(),
            1.32,
        )

        self.assertIs(
            held.event.status,
            FeedbackReplanStatus.HOLDING_NO_FEASIBLE_CANDIDATE,
        )
        self.assertIsNone(held.resolution_request)
        self.assertIsNone(replanner.pending_resolution_request)
        self.assertIs(
            late.event.status,
            FeedbackReplanStatus.SUPERSEDED_TARGET_RESOLUTION,
        )

    def test_rejected_observation_preserves_pending_request(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request

        with self.assertRaises(OutOfOrderObservationError):
            replanner.process_observation(
                observation(1.15, 1.5),
                1.22,
            )

        self.assertEqual(
            replanner.pending_resolution_request.request_sequence,
            request.request_sequence,
        )
        completed = replanner.complete_resolution(
            request.request_sequence,
            target_for(request.selection),
            1.25,
        )
        self.assertTrue(completed.active_intercept_valid)

    def test_expired_completion_is_discarded_without_target_access(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request
        deadline = (
            request.selection.selected_candidate.prediction.timestamp_seconds
        )

        expired = replanner.complete_resolution(
            request.request_sequence,
            object(),
            math.nextafter(deadline, -math.inf),
        )

        self.assertIs(
            expired.event.status,
            FeedbackReplanStatus.EXPIRED_TARGET_RESOLUTION,
        )
        self.assertFalse(expired.active_intercept_valid)
        self.assertIsNone(replanner.pending_resolution_request)
        self.assertIsNone(replanner.fault)
        self.assertEqual(replanner.trajectory_generation, 0)

    def test_failure_at_deadline_expires_without_error_validation(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request
        deadline = (
            request.selection.selected_candidate.prediction.timestamp_seconds
        )

        expired = replanner.fail_resolution(
            request.request_sequence,
            object(),
            deadline,
        )

        self.assertIs(
            expired.event.status,
            FeedbackReplanStatus.EXPIRED_TARGET_RESOLUTION,
        )
        self.assertIsNone(replanner.fault)
        self.assertIsNone(replanner.pending_resolution_request)

    def test_invalid_current_target_latches_target_resolution_fault(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request
        mismatched = replace(
            target_for(request.selection),
            estimate_timestamp_seconds=0.0,
        )

        faulted = replanner.complete_resolution(
            request.request_sequence,
            mismatched,
            1.25,
        )

        self.assertIs(faulted.event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            faulted.event.fault.phase,
            FeedbackReplanFaultPhase.TARGET_RESOLUTION,
        )
        self.assertIn("does not match", faulted.event.fault.detail)
        self.assertEqual(replanner.trajectory_generation, 0)

    def test_current_resolver_failure_latches_and_stale_failure_is_ignored(self):
        replanner = asynchronous_replanner()
        first = feed_to_request(replanner)[-1].resolution_request
        second = replanner.process_observation(
            observation(1.3, 3.0),
            1.31,
        ).resolution_request

        stale = replanner.fail_resolution(0, object(), 1.32)
        failed = replanner.fail_resolution(
            second.request_sequence,
            RuntimeError("IK worker failed"),
            1.33,
        )

        self.assertIs(
            stale.event.status,
            FeedbackReplanStatus.SUPERSEDED_TARGET_RESOLUTION,
        )
        self.assertIs(failed.event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            failed.event.fault.phase,
            FeedbackReplanFaultPhase.TARGET_RESOLUTION,
        )
        self.assertIn("IK worker failed", failed.event.fault.detail)
        self.assertEqual(
            failed.event.resolution_request_sequence,
            second.request_sequence,
        )
        self.assertIs(replanner.fault, failed.event.fault)
        self.assertFalse(failed.active_intercept_valid)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replanner is faulted",
        ):
            replanner.complete_resolution(
                first.request_sequence,
                object(),
                1.34,
            )

    def test_invalid_current_failure_input_preserves_pending_request(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "error must be an Exception",
        ):
            replanner.fail_resolution(
                request.request_sequence,
                object(),
                1.25,
            )

        self.assertEqual(
            replanner.pending_resolution_request.request_sequence,
            request.request_sequence,
        )
        self.assertIsNone(replanner.fault)

    def test_unknown_request_sequence_is_rejected_without_state_mutation(self):
        replanner = asynchronous_replanner()

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "sequence was not issued",
        ):
            replanner.complete_resolution(0, object(), 1.0)

        request = feed_to_request(replanner)[-1].resolution_request
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "must be an integer",
        ):
            replanner.complete_resolution(True, object(), 1.22)
        self.assertEqual(
            replanner.pending_resolution_request.request_sequence,
            request.request_sequence,
        )

    def test_resolution_receipt_time_must_preserve_event_order(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "timestamp precedes active coordinator state",
        ):
            replanner.complete_resolution(
                request.request_sequence,
                object(),
                1.20,
            )

        self.assertEqual(
            replanner.pending_resolution_request.request_sequence,
            request.request_sequence,
        )

    def test_public_request_mutation_does_not_corrupt_pending_state(self):
        replanner = asynchronous_replanner()
        pending = feed_to_request(replanner)[-1]
        request = pending.resolution_request
        object.__setattr__(request, "request_sequence", 99)
        object.__setattr__(
            request.selection,
            "evaluated_at_seconds",
            99.0,
        )
        object.__setattr__(
            pending.event.selection,
            "evaluated_at_seconds",
            88.0,
        )

        fresh = replanner.pending_resolution_request

        self.assertEqual(fresh.request_sequence, 0)
        self.assertEqual(fresh.selection.evaluated_at_seconds, 1.21)
        completed = replanner.complete_resolution(
            fresh.request_sequence,
            target_for(fresh.selection),
            1.25,
        )
        self.assertTrue(completed.active_intercept_valid)

    def test_active_state_corruption_faults_without_replacement(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request
        active = replanner.active_trajectory
        object.__setattr__(active, "axes", active.axes * 2)

        faulted = replanner.complete_resolution(
            request.request_sequence,
            target_for(request.selection),
            1.25,
        )

        self.assertIs(faulted.event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            faulted.event.fault.phase,
            FeedbackReplanFaultPhase.TRAJECTORY_CONSTRUCTION,
        )
        self.assertIn("axis count changed", faulted.event.fault.detail)
        self.assertEqual(replanner.trajectory_generation, 0)

    def test_generation_change_faults_without_target_access(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request
        object.__setattr__(replanner, "_trajectory_generation", 1)

        faulted = replanner.complete_resolution(
            request.request_sequence,
            object(),
            1.25,
        )

        self.assertIs(faulted.event.status, FeedbackReplanStatus.FAULTED)
        self.assertIs(
            faulted.event.fault.phase,
            FeedbackReplanFaultPhase.TRAJECTORY_CONSTRUCTION,
        )
        self.assertIn("generation changed", faulted.event.fault.detail)

    def test_cancel_clears_pending_request_and_is_terminal(self):
        replanner = asynchronous_replanner()
        request = feed_to_request(replanner)[-1].resolution_request

        cancelled = replanner.cancel(1.22)

        self.assertIs(cancelled.event.status, FeedbackReplanStatus.CANCELLED)
        self.assertTrue(replanner.cancelled)
        self.assertFalse(cancelled.active_intercept_valid)
        self.assertIsNone(replanner.pending_resolution_request)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "replanner is cancelled",
        ):
            replanner.complete_resolution(
                request.request_sequence,
                object(),
                1.23,
            )

    def test_async_event_and_result_contracts_reject_inconsistent_payloads(self):
        replanner = asynchronous_replanner()
        pending = feed_to_request(replanner)[-1]

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "pending resolution event has inconsistent output",
        ):
            replace(
                pending.event,
                resolution_request_sequence=None,
            )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "intercept validity is inconsistent",
        ):
            replace(pending, active_intercept_valid=True)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "requires a resolution request",
        ):
            AsynchronousFeedbackReplanResult(
                pending.event,
                False,
            )
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "must not contain asynchronous events",
        ):
            FeedbackReplanReplayResult(
                (pending.event,),
                1,
                replanner.active_trajectory,
                0,
            )

    def test_request_contract_rejects_invalid_axis_and_timestamp(self):
        pending = feed_to_request(asynchronous_replanner())[-1]
        request = pending.resolution_request

        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "axis_count is outside the supported range",
        ):
            replace(request, axis_count=0)
        with self.assertRaisesRegex(
            FeedbackReplanningError,
            "timestamp does not match selection",
        ):
            replace(request, issued_at_seconds=1.22)


if __name__ == "__main__":
    unittest.main()
