from dataclasses import FrozenInstanceError, replace
from decimal import Decimal
import math
import unittest

from ARrobots.trajectory_timing import (
    TRAJECTORY_MAXIMUM_AXES,
    JointBoundaryState,
    JointKinematicLimits,
    JointTrajectoryState,
    JerkLimitedJointTrajectory,
    QuinticJointTrajectory,
    SynchronizedJointTrajectory,
    SynchronizedQuinticTrajectory,
    TrajectoryTimingError,
    minimum_rest_to_rest_joint_trajectory,
    plan_synchronized_quintic_trajectory,
    plan_synchronized_rest_to_rest_trajectory,
    replan_synchronized_quintic_trajectory,
)


def limits(velocity=10.0, acceleration=10.0, jerk=2.0):
    return JointKinematicLimits(velocity, acceleration, jerk)


def boundary(position, velocity=0.0, acceleration=0.0):
    return JointBoundaryState(position, velocity, acceleration)


def assert_quintic_boundary_convergence(test_case, trajectory):
    delta_time = trajectory.duration_seconds * 1e-6
    peak_jerk = trajectory.peak_absolute_jerk_degrees_per_second_cubed
    cases = (
        (
            trajectory.start_state,
            trajectory.state_at(delta_time),
            delta_time,
        ),
        (
            trajectory.target_state,
            trajectory.state_at(trajectory.duration_seconds - delta_time),
            -delta_time,
        ),
    )
    for expected, sample, signed_time in cases:
        expected_position = (
            expected.position_degrees
            + expected.velocity_degrees_per_second * signed_time
            + 0.5
            * expected.acceleration_degrees_per_second_squared
            * signed_time ** 2
        )
        expected_velocity = (
            expected.velocity_degrees_per_second
            + expected.acceleration_degrees_per_second_squared * signed_time
        )
        position_roundoff = 1e-10 * max(
            1.0,
            abs(expected_position),
            abs(sample.position_degrees),
        )
        velocity_roundoff = 1e-10 * max(
            1.0,
            abs(expected_velocity),
            abs(sample.velocity_degrees_per_second),
        )
        acceleration_roundoff = 1e-10 * max(
            1.0,
            abs(expected.acceleration_degrees_per_second_squared),
            abs(sample.acceleration_degrees_per_second_squared),
        )
        test_case.assertLessEqual(
            abs(sample.position_degrees - expected_position),
            peak_jerk * delta_time ** 3 / 6.0 + position_roundoff,
        )
        test_case.assertLessEqual(
            abs(sample.velocity_degrees_per_second - expected_velocity),
            peak_jerk * delta_time ** 2 / 2.0 + velocity_roundoff,
        )
        test_case.assertLessEqual(
            abs(
                sample.acceleration_degrees_per_second_squared
                - expected.acceleration_degrees_per_second_squared
            ),
            peak_jerk * delta_time + acceleration_roundoff,
        )


class DerivedBoundaryState(JointBoundaryState):
    def __getattribute__(self, name):
        if (
            name in (
                "position_degrees",
                "velocity_degrees_per_second",
                "acceleration_degrees_per_second_squared",
            )
            and object.__getattribute__(self, "__dict__").get(
                "_tripwire_armed",
                False,
            )
        ):
            raise AssertionError("derived boundary state was accessed")
        return super().__getattribute__(name)


class DerivedJointKinematicLimits(JointKinematicLimits):
    def __getattribute__(self, name):
        if (
            name.startswith("maximum_")
            and object.__getattribute__(self, "__dict__").get(
                "_tripwire_armed",
                False,
            )
        ):
            raise AssertionError("derived joint limits were accessed")
        return super().__getattribute__(name)


def arm_attribute_tripwire(value):
    object.__setattr__(value, "_tripwire_armed", True)


class JointKinematicLimitsTests(unittest.TestCase):
    def test_limits_normalize_finite_positive_numbers(self):
        value = JointKinematicLimits(
            Decimal("3.5"),
            Decimal("4.5"),
            Decimal("5.5"),
        )

        self.assertEqual(value.maximum_velocity_degrees_per_second, 3.5)
        self.assertEqual(
            value.maximum_acceleration_degrees_per_second_squared,
            4.5,
        )
        self.assertEqual(
            value.maximum_jerk_degrees_per_second_cubed,
            5.5,
        )
        with self.assertRaises(FrozenInstanceError):
            value.maximum_velocity_degrees_per_second = 1.0

    def test_limits_reject_invalid_boundaries(self):
        for invalid in (0.0, -1.0, True, math.inf, math.nan, object()):
            with self.subTest(invalid=invalid):
                with self.assertRaises(TrajectoryTimingError):
                    JointKinematicLimits(invalid, 1.0, 1.0)
                with self.assertRaises(TrajectoryTimingError):
                    JointKinematicLimits(1.0, invalid, 1.0)
                with self.assertRaises(TrajectoryTimingError):
                    JointKinematicLimits(1.0, 1.0, invalid)
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "outside the host numeric range",
        ):
            JointKinematicLimits(Decimal("1e-10000"), 1.0, 1.0)


class MinimumJointTrajectoryTests(unittest.TestCase):
    def test_short_move_uses_triangular_jerk_profile(self):
        trajectory = minimum_rest_to_rest_joint_trajectory(
            0.0,
            0.25,
            limits(),
        )
        expected_jerk_phase = math.cbrt(0.25 / 4.0)

        self.assertAlmostEqual(
            trajectory.jerk_phase_seconds,
            expected_jerk_phase,
        )
        self.assertEqual(trajectory.constant_acceleration_phase_seconds, 0.0)
        self.assertEqual(trajectory.cruise_phase_seconds, 0.0)
        self.assertAlmostEqual(
            trajectory.duration_seconds,
            4.0 * expected_jerk_phase,
        )
        self.assertLess(
            trajectory.peak_acceleration_degrees_per_second_squared,
            trajectory.limits.maximum_acceleration_degrees_per_second_squared,
        )

    def test_medium_move_reaches_acceleration_without_velocity(self):
        trajectory = minimum_rest_to_rest_joint_trajectory(
            2.0,
            3.5,
            limits(velocity=10.0, acceleration=1.0, jerk=2.0),
        )

        self.assertAlmostEqual(trajectory.jerk_phase_seconds, 0.5)
        self.assertAlmostEqual(
            trajectory.constant_acceleration_phase_seconds,
            0.5,
        )
        self.assertEqual(trajectory.cruise_phase_seconds, 0.0)
        self.assertAlmostEqual(trajectory.duration_seconds, 3.0)
        self.assertAlmostEqual(
            trajectory.peak_acceleration_degrees_per_second_squared,
            1.0,
        )
        self.assertAlmostEqual(
            trajectory.peak_velocity_degrees_per_second,
            1.0,
        )

    def test_velocity_limit_before_acceleration_adds_cruise(self):
        trajectory = minimum_rest_to_rest_joint_trajectory(
            0.0,
            3.0,
            limits(velocity=1.0, acceleration=10.0, jerk=4.0),
        )

        self.assertAlmostEqual(trajectory.jerk_phase_seconds, 0.5)
        self.assertEqual(trajectory.constant_acceleration_phase_seconds, 0.0)
        self.assertAlmostEqual(trajectory.cruise_phase_seconds, 2.0)
        self.assertAlmostEqual(trajectory.duration_seconds, 4.0)
        self.assertAlmostEqual(
            trajectory.peak_velocity_degrees_per_second,
            1.0,
        )

    def test_acceleration_and_velocity_limits_add_cruise(self):
        trajectory = minimum_rest_to_rest_joint_trajectory(
            4.0,
            13.0,
            limits(velocity=2.0, acceleration=1.0, jerk=2.0),
        )

        self.assertAlmostEqual(trajectory.jerk_phase_seconds, 0.5)
        self.assertAlmostEqual(
            trajectory.constant_acceleration_phase_seconds,
            1.5,
        )
        self.assertAlmostEqual(trajectory.cruise_phase_seconds, 2.0)
        self.assertAlmostEqual(trajectory.duration_seconds, 7.0)
        self.assertAlmostEqual(
            trajectory.peak_velocity_degrees_per_second,
            2.0,
        )

    def test_threshold_distances_produce_continuous_profiles(self):
        acceleration_limited = limits(
            velocity=2.0,
            acceleration=1.0,
            jerk=2.0,
        )
        at_acceleration = minimum_rest_to_rest_joint_trajectory(
            0.0,
            0.5,
            acceleration_limited,
        )
        at_velocity = minimum_rest_to_rest_joint_trajectory(
            0.0,
            5.0,
            acceleration_limited,
        )
        velocity_limited = minimum_rest_to_rest_joint_trajectory(
            0.0,
            1.0,
            limits(velocity=1.0, acceleration=10.0, jerk=4.0),
        )

        self.assertAlmostEqual(at_acceleration.jerk_phase_seconds, 0.5)
        self.assertEqual(
            at_acceleration.constant_acceleration_phase_seconds,
            0.0,
        )
        self.assertAlmostEqual(at_velocity.cruise_phase_seconds, 0.0)
        self.assertAlmostEqual(
            at_velocity.constant_acceleration_phase_seconds,
            1.5,
        )
        self.assertAlmostEqual(velocity_limited.cruise_phase_seconds, 0.0)
        self.assertAlmostEqual(velocity_limited.jerk_phase_seconds, 0.5)

    def test_negative_and_stationary_moves_preserve_endpoints(self):
        negative = minimum_rest_to_rest_joint_trajectory(
            5.0,
            -4.0,
            limits(velocity=2.0, acceleration=1.0, jerk=2.0),
        )
        stationary = minimum_rest_to_rest_joint_trajectory(
            3.0,
            3.0,
            limits(),
        )

        self.assertEqual(negative.direction, -1.0)
        self.assertEqual(negative.state_at(0.0).position_degrees, 5.0)
        self.assertEqual(
            negative.state_at(negative.duration_seconds).position_degrees,
            -4.0,
        )
        self.assertEqual(stationary.direction, 0.0)
        self.assertEqual(stationary.duration_seconds, 0.0)
        self.assertEqual(stationary.state_at(10.0).position_degrees, 3.0)

    def test_sampling_obeys_limits_and_reaches_exact_terminal_state(self):
        trajectory = minimum_rest_to_rest_joint_trajectory(
            4.0,
            13.0,
            limits(velocity=2.0, acceleration=1.0, jerk=2.0),
        )
        samples = tuple(
            trajectory.state_at(trajectory.duration_seconds * index / 1000)
            for index in range(1001)
        )

        self.assertTrue(all(
            isinstance(sample, JointTrajectoryState)
            for sample in samples
        ))
        self.assertTrue(all(
            4.0 <= sample.position_degrees <= 13.0
            for sample in samples
        ))
        self.assertTrue(all(
            abs(sample.velocity_degrees_per_second)
            <= trajectory.limits.maximum_velocity_degrees_per_second + 1e-12
            for sample in samples
        ))
        self.assertTrue(all(
            abs(sample.acceleration_degrees_per_second_squared)
            <= trajectory.limits.maximum_acceleration_degrees_per_second_squared
            + 1e-12
            for sample in samples
        ))
        self.assertTrue(all(
            abs(sample.jerk_degrees_per_second_cubed)
            <= trajectory.limits.maximum_jerk_degrees_per_second_cubed
            for sample in samples
        ))
        self.assertEqual(samples[-1].position_degrees, 13.0)
        self.assertEqual(samples[-1].velocity_degrees_per_second, 0.0)
        self.assertEqual(
            samples[-1].acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertEqual(samples[-1].jerk_degrees_per_second_cubed, 0.0)

    def test_phase_integrator_matches_closed_form_states(self):
        trajectory = minimum_rest_to_rest_joint_trajectory(
            4.0,
            13.0,
            limits(velocity=2.0, acceleration=1.0, jerk=2.0),
        )

        jerk_boundary = trajectory.state_at(
            trajectory.jerk_phase_seconds
        )
        acceleration_boundary = trajectory.state_at(
            2.0 * trajectory.jerk_phase_seconds
            + trajectory.constant_acceleration_phase_seconds
        )
        midpoint = trajectory.state_at(trajectory.duration_seconds / 2.0)
        deceleration_jerk_boundary = trajectory.state_at(
            trajectory.duration_seconds
            - trajectory.jerk_phase_seconds
            - trajectory.constant_acceleration_phase_seconds
        )
        deceleration_boundary = trajectory.state_at(
            trajectory.duration_seconds - trajectory.jerk_phase_seconds
        )
        terminal_jerk_midpoint = trajectory.state_at(
            trajectory.duration_seconds
            - trajectory.jerk_phase_seconds / 2.0
        )

        self.assertAlmostEqual(
            jerk_boundary.position_degrees,
            4.0 + 2.0 * 0.5 ** 3 / 6.0,
        )
        self.assertAlmostEqual(
            jerk_boundary.velocity_degrees_per_second,
            2.0 * 0.5 ** 2 / 2.0,
        )
        self.assertAlmostEqual(
            jerk_boundary.acceleration_degrees_per_second_squared,
            1.0,
        )
        self.assertAlmostEqual(acceleration_boundary.position_degrees, 6.5)
        self.assertAlmostEqual(
            acceleration_boundary.velocity_degrees_per_second,
            2.0,
        )
        self.assertAlmostEqual(
            acceleration_boundary.acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertAlmostEqual(midpoint.position_degrees, 8.5)
        self.assertAlmostEqual(midpoint.velocity_degrees_per_second, 2.0)
        self.assertAlmostEqual(
            midpoint.acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertAlmostEqual(
            deceleration_jerk_boundary.position_degrees,
            13.0 - (2.0 * 0.5 ** 3 / 6.0 + 0.25 * 1.5 + 1.125),
        )
        self.assertAlmostEqual(
            deceleration_jerk_boundary.velocity_degrees_per_second,
            1.75,
        )
        self.assertAlmostEqual(
            deceleration_jerk_boundary.acceleration_degrees_per_second_squared,
            -1.0,
        )
        self.assertAlmostEqual(
            deceleration_boundary.position_degrees,
            13.0 - 2.0 * 0.5 ** 3 / 6.0,
        )
        self.assertAlmostEqual(
            deceleration_boundary.velocity_degrees_per_second,
            0.25,
        )
        self.assertAlmostEqual(
            deceleration_boundary.acceleration_degrees_per_second_squared,
            -1.0,
        )
        self.assertAlmostEqual(
            terminal_jerk_midpoint.position_degrees,
            13.0 - 2.0 * 0.25 ** 3 / 6.0,
        )
        self.assertAlmostEqual(
            terminal_jerk_midpoint.velocity_degrees_per_second,
            2.0 * 0.25 ** 2 / 2.0,
        )
        self.assertAlmostEqual(
            terminal_jerk_midpoint.acceleration_degrees_per_second_squared,
            -0.5,
        )

    def test_sampling_rejects_invalid_elapsed_time(self):
        trajectory = minimum_rest_to_rest_joint_trajectory(0.0, 1.0, limits())

        for invalid in (-0.1, True, math.nan, math.inf, object()):
            with self.subTest(invalid=invalid):
                with self.assertRaises(TrajectoryTimingError):
                    trajectory.state_at(invalid)

    def test_profile_validation_rejects_malformed_public_values(self):
        valid = minimum_rest_to_rest_joint_trajectory(0.0, 1.0, limits())
        invalid_changes = (
            {"limits": object()},
            {"applied_jerk_degrees_per_second_cubed": 0.0},
            {"applied_jerk_degrees_per_second_cubed": 20.0},
            {"jerk_phase_seconds": 0.0},
            {"target_position_degrees": 2.0},
        )
        for changes in invalid_changes:
            with self.subTest(changes=changes):
                with self.assertRaises(TrajectoryTimingError):
                    replace(valid, **changes)

        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "displacement is outside",
        ):
            minimum_rest_to_rest_joint_trajectory(
                -1e308,
                1e308,
                limits(),
            )


class SynchronizedTrajectoryTests(unittest.TestCase):
    def test_time_scaling_synchronizes_moving_and_stationary_axes(self):
        joint_limits = limits(velocity=2.0, acceleration=1.0, jerk=2.0)
        trajectory = plan_synchronized_rest_to_rest_trajectory(
            (0.0, 0.0, 8.0),
            (9.0, 1.5, 8.0),
            (joint_limits, joint_limits, joint_limits),
        )

        self.assertAlmostEqual(trajectory.duration_seconds, 7.0)
        self.assertAlmostEqual(
            trajectory.axes[1].duration_seconds,
            trajectory.duration_seconds,
        )
        self.assertAlmostEqual(
            trajectory.axes[2].duration_seconds,
            trajectory.duration_seconds,
        )
        self.assertLess(
            trajectory.axes[1].applied_jerk_degrees_per_second_cubed,
            joint_limits.maximum_jerk_degrees_per_second_cubed,
        )
        self.assertEqual(
            trajectory.axes[2].applied_jerk_degrees_per_second_cubed,
            0.0,
        )
        self.assertEqual(
            trajectory.minimum_arrival_time_seconds,
            trajectory.duration_seconds,
        )
        self.assertEqual(
            trajectory.positions_at(trajectory.duration_seconds),
            (9.0, 1.5, 8.0),
        )

    def test_synchronized_samples_remain_bounded_and_deterministic(self):
        axis_limits = (
            limits(velocity=2.0, acceleration=1.0, jerk=2.0),
            limits(velocity=3.0, acceleration=2.0, jerk=4.0),
        )
        trajectory = plan_synchronized_rest_to_rest_trajectory(
            (0.0, 4.0),
            (9.0, -2.0),
            axis_limits,
        )

        first = tuple(
            trajectory.states_at(trajectory.duration_seconds * index / 100)
            for index in range(101)
        )
        second = tuple(
            trajectory.states_at(trajectory.duration_seconds * index / 100)
            for index in range(101)
        )

        self.assertEqual(first, second)
        for samples, axis_limit in zip(zip(*first), axis_limits):
            self.assertTrue(all(
                abs(sample.velocity_degrees_per_second)
                <= axis_limit.maximum_velocity_degrees_per_second + 1e-12
                for sample in samples
            ))
            self.assertTrue(all(
                abs(sample.acceleration_degrees_per_second_squared)
                <= axis_limit.maximum_acceleration_degrees_per_second_squared
                + 1e-12
                for sample in samples
            ))
            self.assertTrue(all(
                abs(sample.jerk_degrees_per_second_cubed)
                <= axis_limit.maximum_jerk_degrees_per_second_cubed
                for sample in samples
            ))

    def test_sequence_boundaries_and_axis_contract_fail_closed(self):
        axis_limit = limits()
        invalid_calls = (
            ((), (), ()),
            ((0.0,), (1.0, 2.0), (axis_limit,)),
            ((0.0,), (1.0,), (object(),)),
            ("0", (1.0,), (axis_limit,)),
        )
        for starts, targets, joint_limits in invalid_calls:
            with self.subTest(starts=starts, targets=targets):
                with self.assertRaises(TrajectoryTimingError):
                    plan_synchronized_rest_to_rest_trajectory(
                        starts,
                        targets,
                        joint_limits,
                    )

        too_many = tuple(0.0 for _ in range(TRAJECTORY_MAXIMUM_AXES + 1))
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "maximum axis count",
        ):
            plan_synchronized_rest_to_rest_trajectory(
                too_many,
                too_many,
                tuple(axis_limit for _ in too_many),
            )

        moving = minimum_rest_to_rest_joint_trajectory(
            0.0,
            1.0,
            axis_limit,
        )
        stationary = minimum_rest_to_rest_joint_trajectory(
            0.0,
            0.0,
            axis_limit,
        )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "durations must match",
        ):
            SynchronizedJointTrajectory((moving, stationary))
        with self.assertRaises(TrajectoryTimingError):
            SynchronizedJointTrajectory((object(),))


class JointBoundaryStateTests(unittest.TestCase):
    def test_boundary_normalizes_finite_values_and_is_immutable(self):
        state = JointBoundaryState(
            Decimal("1.25"),
            Decimal("-2.5"),
            Decimal("3.75"),
        )

        self.assertEqual(state.position_degrees, 1.25)
        self.assertEqual(state.velocity_degrees_per_second, -2.5)
        self.assertEqual(
            state.acceleration_degrees_per_second_squared,
            3.75,
        )
        with self.assertRaises(FrozenInstanceError):
            state.position_degrees = 0.0

    def test_boundary_rejects_invalid_numeric_values(self):
        for invalid in (True, math.inf, math.nan, object()):
            with self.subTest(invalid=invalid):
                with self.assertRaises(TrajectoryTimingError):
                    JointBoundaryState(invalid, 0.0, 0.0)
                with self.assertRaises(TrajectoryTimingError):
                    JointBoundaryState(0.0, invalid, 0.0)
                with self.assertRaises(TrajectoryTimingError):
                    JointBoundaryState(0.0, 0.0, invalid)

        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "outside the host numeric range",
        ):
            JointBoundaryState(Decimal("1e-10000"), 0.0, 0.0)


class QuinticJointTrajectoryTests(unittest.TestCase):
    def test_rest_to_rest_segment_matches_known_quintic(self):
        trajectory = QuinticJointTrajectory(
            boundary(0.0),
            boundary(1.0),
            2.0,
            limits(velocity=2.0, acceleration=2.0, jerk=8.0),
        )

        start = trajectory.state_at(0.0)
        midpoint = trajectory.state_at(1.0)
        target = trajectory.state_at(2.0)

        self.assertEqual(start.position_degrees, 0.0)
        self.assertEqual(start.velocity_degrees_per_second, 0.0)
        self.assertEqual(
            start.acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertAlmostEqual(start.jerk_degrees_per_second_cubed, 7.5)
        self.assertAlmostEqual(midpoint.position_degrees, 0.5)
        self.assertAlmostEqual(midpoint.velocity_degrees_per_second, 0.9375)
        self.assertAlmostEqual(
            midpoint.acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertAlmostEqual(
            midpoint.jerk_degrees_per_second_cubed,
            -3.75,
        )
        self.assertEqual(target.position_degrees, 1.0)
        self.assertEqual(target.velocity_degrees_per_second, 0.0)
        self.assertEqual(
            target.acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertAlmostEqual(target.jerk_degrees_per_second_cubed, 7.5)
        self.assertAlmostEqual(
            trajectory.peak_absolute_velocity_degrees_per_second,
            0.9375,
        )
        self.assertAlmostEqual(
            trajectory.peak_absolute_acceleration_degrees_per_second_squared,
            1.4433756729740645,
        )
        self.assertAlmostEqual(
            trajectory.peak_absolute_jerk_degrees_per_second_cubed,
            7.5,
        )

    def test_arbitrary_boundaries_are_reconstructed_exactly(self):
        start_state = boundary(2.0, velocity=1.0, acceleration=0.5)
        target_state = boundary(5.0, velocity=-0.5, acceleration=-0.25)
        trajectory = QuinticJointTrajectory(
            start_state,
            target_state,
            4.0,
            limits(velocity=20.0, acceleration=20.0, jerk=20.0),
        )

        start = trajectory.state_at(0.0)
        target = trajectory.state_at(trajectory.duration_seconds)

        self.assertEqual(start.position_degrees, start_state.position_degrees)
        self.assertEqual(
            start.velocity_degrees_per_second,
            start_state.velocity_degrees_per_second,
        )
        self.assertEqual(
            start.acceleration_degrees_per_second_squared,
            start_state.acceleration_degrees_per_second_squared,
        )
        self.assertEqual(target.position_degrees, target_state.position_degrees)
        self.assertEqual(
            target.velocity_degrees_per_second,
            target_state.velocity_degrees_per_second,
        )
        self.assertEqual(
            target.acceleration_degrees_per_second_squared,
            target_state.acceleration_degrees_per_second_squared,
        )
        assert_quintic_boundary_convergence(self, trajectory)

    def test_consistent_constant_velocity_boundaries_reduce_exactly(self):
        trajectory = QuinticJointTrajectory(
            boundary(1.0, velocity=2.0),
            boundary(7.0, velocity=2.0),
            3.0,
            limits(velocity=2.0, acceleration=1.0, jerk=1.0),
        )

        midpoint = trajectory.state_at(1.5)

        self.assertEqual(midpoint.position_degrees, 4.0)
        self.assertEqual(midpoint.velocity_degrees_per_second, 2.0)
        self.assertEqual(
            midpoint.acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertEqual(midpoint.jerk_degrees_per_second_cubed, 0.0)
        self.assertEqual(
            trajectory.peak_absolute_velocity_degrees_per_second,
            2.0,
        )
        self.assertEqual(
            trajectory.peak_absolute_acceleration_degrees_per_second_squared,
            0.0,
        )
        self.assertEqual(
            trajectory.peak_absolute_jerk_degrees_per_second_cubed,
            0.0,
        )

    def test_derivative_root_extrema_reject_each_kinematic_limit(self):
        start_state = boundary(0.0)
        target_state = boundary(1.0)
        cases = (
            (
                limits(velocity=1.8, acceleration=10.0, jerk=100.0),
                "velocity limit",
            ),
            (
                limits(velocity=10.0, acceleration=5.7, jerk=100.0),
                "acceleration limit",
            ),
            (
                limits(velocity=10.0, acceleration=10.0, jerk=59.0),
                "jerk limit",
            ),
        )

        for joint_limits, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(TrajectoryTimingError, message):
                    QuinticJointTrajectory(
                        start_state,
                        target_state,
                        1.0,
                        joint_limits,
                    )

    def test_reported_extrema_bound_dense_deterministic_sampling(self):
        trajectory = QuinticJointTrajectory(
            boundary(-2.0, velocity=1.25, acceleration=-0.75),
            boundary(4.0, velocity=-0.5, acceleration=0.25),
            3.0,
            limits(velocity=20.0, acceleration=20.0, jerk=30.0),
        )
        samples = tuple(
            trajectory.state_at(trajectory.duration_seconds * index / 2000)
            for index in range(2001)
        )

        self.assertLessEqual(
            max(abs(sample.velocity_degrees_per_second) for sample in samples),
            trajectory.peak_absolute_velocity_degrees_per_second + 1e-12,
        )
        self.assertLessEqual(
            max(
                abs(sample.acceleration_degrees_per_second_squared)
                for sample in samples
            ),
            trajectory.peak_absolute_acceleration_degrees_per_second_squared
            + 1e-12,
        )
        self.assertLessEqual(
            max(
                abs(sample.jerk_degrees_per_second_cubed)
                for sample in samples
            ),
            trajectory.peak_absolute_jerk_degrees_per_second_cubed + 1e-12,
        )

    def test_sampling_and_public_construction_fail_closed(self):
        trajectory = QuinticJointTrajectory(
            boundary(0.0),
            boundary(1.0),
            2.0,
            limits(velocity=2.0, acceleration=2.0, jerk=8.0),
        )
        for invalid in (-0.1, True, math.inf, math.nan, object()):
            with self.subTest(invalid=invalid):
                with self.assertRaises(TrajectoryTimingError):
                    trajectory.state_at(invalid)
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "exceeds the segment duration",
        ):
            trajectory.state_at(2.1)

        invalid_changes = (
            {"start_state": object()},
            {"target_state": object()},
            {"duration_seconds": 0.0},
            {"limits": object()},
        )
        for changes in invalid_changes:
            with self.subTest(changes=changes):
                with self.assertRaises(TrajectoryTimingError):
                    replace(trajectory, **changes)

    def test_numeric_range_loss_is_rejected(self):
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "duration square is outside",
        ):
            QuinticJointTrajectory(
                boundary(0.0),
                boundary(1.0),
                math.ulp(0.0),
                limits(),
            )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "displacement is outside",
        ):
            QuinticJointTrajectory(
                boundary(-1e308),
                boundary(1e308),
                1.0,
                limits(),
            )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "target velocity cannot be represented",
        ):
            QuinticJointTrajectory(
                boundary(1e12, velocity=1.0, acceleration=1.0),
                boundary(-1e12, velocity=-1.0, acceleration=-1.0),
                0.1,
                limits(velocity=1e20, acceleration=1e20, jerk=1e20),
            )


class SynchronizedQuinticTrajectoryTests(unittest.TestCase):
    def test_planner_builds_fixed_duration_multi_axis_segment(self):
        joint_limits = limits(velocity=20.0, acceleration=20.0, jerk=50.0)
        trajectory = plan_synchronized_quintic_trajectory(
            (
                boundary(0.0, velocity=1.0),
                boundary(2.0),
                boundary(-4.0),
            ),
            (
                boundary(3.0),
                boundary(-1.0, velocity=-0.5),
                boundary(-4.0),
            ),
            3.0,
            (joint_limits, joint_limits, joint_limits),
        )

        self.assertEqual(trajectory.duration_seconds, 3.0)
        self.assertTrue(all(
            axis.duration_seconds == 3.0
            for axis in trajectory.axes
        ))
        self.assertEqual(
            trajectory.positions_at(trajectory.duration_seconds),
            (3.0, -1.0, -4.0),
        )
        for axis in trajectory.axes:
            assert_quintic_boundary_convergence(self, axis)

    def test_replan_preserves_position_velocity_and_acceleration(self):
        joint_limits = limits(velocity=20.0, acceleration=20.0, jerk=50.0)
        active = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (9.0,),
            (joint_limits,),
        )
        replacement_elapsed = active.duration_seconds * 0.4
        active_state = active.states_at(replacement_elapsed)[0]

        replacement = replan_synchronized_quintic_trajectory(
            active,
            replacement_elapsed,
            (boundary(12.0),),
            3.0,
        )
        replacement_start = replacement.states_at(0.0)[0]

        self.assertEqual(
            replacement_start.position_degrees,
            active_state.position_degrees,
        )
        self.assertEqual(
            replacement_start.velocity_degrees_per_second,
            active_state.velocity_degrees_per_second,
        )
        self.assertEqual(
            replacement_start.acceleration_degrees_per_second_squared,
            active_state.acceleration_degrees_per_second_squared,
        )
        self.assertEqual(
            replacement.positions_at(replacement.duration_seconds),
            (12.0,),
        )
        assert_quintic_boundary_convergence(self, replacement.axes[0])

        chained_elapsed = 1.0
        chained_source = replacement.states_at(chained_elapsed)[0]
        chained = replan_synchronized_quintic_trajectory(
            replacement,
            chained_elapsed,
            (boundary(13.0),),
            2.0,
        )
        chained_start = chained.states_at(0.0)[0]
        self.assertEqual(
            (
                chained_start.position_degrees,
                chained_start.velocity_degrees_per_second,
                chained_start.acceleration_degrees_per_second_squared,
            ),
            (
                chained_source.position_degrees,
                chained_source.velocity_degrees_per_second,
                chained_source.acceleration_degrees_per_second_squared,
            ),
        )
        assert_quintic_boundary_convergence(self, chained.axes[0])

    def test_infeasible_replacement_duration_fails_closed(self):
        joint_limits = limits(velocity=20.0, acceleration=20.0, jerk=50.0)
        active = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (9.0,),
            (joint_limits,),
        )

        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "exceeds the acceleration limit",
        ):
            replan_synchronized_quintic_trajectory(
                active,
                active.duration_seconds * 0.4,
                (boundary(12.0),),
                1.0,
            )

    def test_public_planners_reject_invalid_duration_values(self):
        joint_limits = limits(velocity=20.0, acceleration=20.0, jerk=50.0)
        start_state = boundary(0.0)
        target_state = boundary(1.0)
        active = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (1.0,),
            (joint_limits,),
        )

        for invalid in (0.0, -1.0, True, math.inf, math.nan, object()):
            with self.subTest(invalid=invalid):
                with self.assertRaises(TrajectoryTimingError):
                    plan_synchronized_quintic_trajectory(
                        (start_state,),
                        (target_state,),
                        invalid,
                        (joint_limits,),
                    )
                with self.assertRaises(TrajectoryTimingError):
                    replan_synchronized_quintic_trajectory(
                        active,
                        0.0,
                        (target_state,),
                        invalid,
                    )

    def test_trajectory_consumers_reject_derived_value_types_before_access(self):
        joint_limits = limits(velocity=20.0, acceleration=20.0, jerk=50.0)
        derived_boundary = DerivedBoundaryState(0.0, 0.0, 0.0)
        arm_attribute_tripwire(derived_boundary)

        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "start_state must be built-in JointBoundaryState",
        ):
            QuinticJointTrajectory(
                derived_boundary,
                boundary(1.0),
                2.0,
                joint_limits,
            )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "target_state must be built-in JointBoundaryState",
        ):
            QuinticJointTrajectory(
                boundary(0.0),
                derived_boundary,
                2.0,
                joint_limits,
            )

        valid_rest_trajectory = minimum_rest_to_rest_joint_trajectory(
            0.0,
            1.0,
            joint_limits,
        )
        derived_limits = DerivedJointKinematicLimits(20.0, 20.0, 50.0)
        arm_attribute_tripwire(derived_limits)
        with self.assertRaises(TrajectoryTimingError):
            replace(valid_rest_trajectory, limits=derived_limits)
        with self.assertRaises(TrajectoryTimingError):
            minimum_rest_to_rest_joint_trajectory(
                0.0,
                1.0,
                derived_limits,
            )
        with self.assertRaises(TrajectoryTimingError):
            QuinticJointTrajectory(
                boundary(0.0),
                boundary(1.0),
                2.0,
                derived_limits,
            )
        with self.assertRaises(TrajectoryTimingError):
            plan_synchronized_rest_to_rest_trajectory(
                (0.0,),
                (1.0,),
                (derived_limits,),
            )
        with self.assertRaises(TrajectoryTimingError):
            plan_synchronized_quintic_trajectory(
                (boundary(0.0),),
                (boundary(1.0),),
                2.0,
                (derived_limits,),
            )

    def test_sequence_and_active_trajectory_boundaries_fail_closed(self):
        joint_limits = limits(velocity=20.0, acceleration=20.0, jerk=50.0)
        invalid_calls = (
            ((), (), ()),
            ((boundary(0.0),), (boundary(1.0), boundary(2.0)), (joint_limits,)),
            ((boundary(0.0),), (boundary(1.0),), (object(),)),
            ("invalid", (boundary(1.0),), (joint_limits,)),
        )
        for starts, targets, joint_limit_values in invalid_calls:
            with self.subTest(starts=starts, targets=targets):
                with self.assertRaises(TrajectoryTimingError):
                    plan_synchronized_quintic_trajectory(
                        starts,
                        targets,
                        2.0,
                        joint_limit_values,
                    )

        too_many = tuple(
            boundary(0.0)
            for _ in range(TRAJECTORY_MAXIMUM_AXES + 1)
        )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "maximum axis count",
        ):
            plan_synchronized_quintic_trajectory(
                too_many,
                too_many,
                2.0,
                tuple(joint_limits for _ in too_many),
            )

        active = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (1.0,),
            (joint_limits,),
        )
        with self.assertRaises(TrajectoryTimingError):
            replan_synchronized_quintic_trajectory(
                object(),
                0.0,
                (boundary(1.0),),
                2.0,
            )

        class UnhashableType(type):
            __hash__ = None

        class UnhashableActive(metaclass=UnhashableType):
            __slots__ = ()

        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "active trajectory must be a synchronized built-in trajectory",
        ):
            replan_synchronized_quintic_trajectory(
                UnhashableActive(),
                0.0,
                (boundary(1.0),),
                2.0,
            )

        class DerivedSynchronizedTrajectory(SynchronizedJointTrajectory):
            def states_at(self, elapsed_seconds):
                raise AssertionError("derived states_at must not run")

        with self.assertRaises(TrajectoryTimingError):
            replan_synchronized_quintic_trajectory(
                DerivedSynchronizedTrajectory(active.axes),
                0.0,
                (boundary(1.0),),
                2.0,
            )

        class DerivedRestAxis(JerkLimitedJointTrajectory):
            def state_at(self, elapsed_seconds):
                raise AssertionError("derived state_at must not run")

        rest_axis = active.axes[0]
        derived_rest_axis = DerivedRestAxis(
            rest_axis.start_position_degrees,
            rest_axis.target_position_degrees,
            rest_axis.limits,
            rest_axis.jerk_phase_seconds,
            rest_axis.constant_acceleration_phase_seconds,
            rest_axis.cruise_phase_seconds,
            rest_axis.applied_jerk_degrees_per_second_cubed,
        )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "must contain built-in joint trajectories",
        ):
            SynchronizedJointTrajectory((derived_rest_axis,))
        tampered_rest_axes = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (1.0,),
            (joint_limits,),
        )
        object.__setattr__(
            tampered_rest_axes,
            "axes",
            (derived_rest_axis,),
        )
        with self.assertRaises(TrajectoryTimingError) as raised:
            replan_synchronized_quintic_trajectory(
                tampered_rest_axes,
                0.0,
                (boundary(1.0),),
                2.0,
            )
        self.assertEqual(
            str(raised.exception),
            "active trajectory axes must be built-in joint trajectories",
        )

        class DerivedQuinticAxis(QuinticJointTrajectory):
            def state_at(self, elapsed_seconds):
                raise AssertionError("derived state_at must not run")

        quintic_axis = plan_synchronized_quintic_trajectory(
            (boundary(0.0),),
            (boundary(1.0),),
            2.0,
            (joint_limits,),
        ).axes[0]
        derived_quintic_axis = DerivedQuinticAxis(
            quintic_axis.start_state,
            quintic_axis.target_state,
            quintic_axis.duration_seconds,
            quintic_axis.limits,
        )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "must contain built-in quintic trajectories",
        ):
            SynchronizedQuinticTrajectory((derived_quintic_axis,))
        tampered_quintic_axes = plan_synchronized_quintic_trajectory(
            (boundary(0.0),),
            (boundary(1.0),),
            2.0,
            (joint_limits,),
        )
        object.__setattr__(
            tampered_quintic_axes,
            "axes",
            (derived_quintic_axis,),
        )
        with self.assertRaises(TrajectoryTimingError) as raised:
            replan_synchronized_quintic_trajectory(
                tampered_quintic_axes,
                0.0,
                (boundary(1.0),),
                2.0,
            )
        self.assertEqual(
            str(raised.exception),
            "active trajectory axes must be built-in quintic trajectories",
        )

        for tampered_axis_count in (
            plan_synchronized_rest_to_rest_trajectory(
                (0.0,),
                (1.0,),
                (joint_limits,),
            ),
            plan_synchronized_quintic_trajectory(
                (boundary(0.0),),
                (boundary(1.0),),
                2.0,
                (joint_limits,),
            ),
        ):
            with self.subTest(
                bounded_type=type(tampered_axis_count),
            ):
                object.__setattr__(
                    tampered_axis_count,
                    "axes",
                    tampered_axis_count.axes * 10,
                )
                with self.assertRaisesRegex(
                    TrajectoryTimingError,
                    "exceeds the maximum axis count",
                ):
                    replan_synchronized_quintic_trajectory(
                        tampered_axis_count,
                        0.0,
                        (boundary(1.0),),
                        2.0,
                    )

        rest_duration_mismatch = plan_synchronized_rest_to_rest_trajectory(
            (0.0,),
            (1.0,),
            (joint_limits,),
        )
        object.__setattr__(
            rest_duration_mismatch,
            "axes",
            (
                minimum_rest_to_rest_joint_trajectory(
                    0.0,
                    1.0,
                    joint_limits,
                ),
                minimum_rest_to_rest_joint_trajectory(
                    0.0,
                    2.0,
                    joint_limits,
                ),
            ),
        )
        quintic_duration_mismatch = plan_synchronized_quintic_trajectory(
            (boundary(0.0),),
            (boundary(1.0),),
            2.0,
            (joint_limits,),
        )
        object.__setattr__(
            quintic_duration_mismatch,
            "axes",
            (
                QuinticJointTrajectory(
                    boundary(0.0),
                    boundary(1.0),
                    2.0,
                    joint_limits,
                ),
                QuinticJointTrajectory(
                    boundary(0.0),
                    boundary(2.0),
                    3.0,
                    joint_limits,
                ),
            ),
        )
        for tampered_duration in (
            rest_duration_mismatch,
            quintic_duration_mismatch,
        ):
            with self.subTest(
                duration_type=type(tampered_duration),
            ):
                with self.assertRaisesRegex(
                    TrajectoryTimingError,
                    "axis durations must match",
                ):
                    replan_synchronized_quintic_trajectory(
                        tampered_duration,
                        0.0,
                        (boundary(1.0), boundary(2.0)),
                        2.0,
                    )

        tampered_limit_trajectories = (
            plan_synchronized_rest_to_rest_trajectory(
                (0.0,),
                (1.0,),
                (joint_limits,),
            ),
            plan_synchronized_quintic_trajectory(
                (boundary(0.0),),
                (boundary(1.0),),
                2.0,
                (joint_limits,),
            ),
        )
        derived_limits = DerivedJointKinematicLimits(20.0, 20.0, 50.0)
        arm_attribute_tripwire(derived_limits)
        for tampered_limit_trajectory in tampered_limit_trajectories:
            with self.subTest(
                trajectory_type=type(tampered_limit_trajectory),
            ):
                object.__setattr__(
                    tampered_limit_trajectory.axes[0],
                    "limits",
                    derived_limits,
                )
                with self.assertRaises(TrajectoryTimingError) as raised:
                    replan_synchronized_quintic_trajectory(
                        tampered_limit_trajectory,
                        0.0,
                        (boundary(1.0),),
                        2.0,
                    )
                self.assertEqual(
                    str(raised.exception),
                    "active trajectory limits must be built-in joint limits",
                )

        for field_name in ("start_state", "target_state"):
            with self.subTest(field_name=field_name):
                tampered_quintic_trajectory = plan_synchronized_quintic_trajectory(
                    (boundary(0.0),),
                    (boundary(1.0),),
                    2.0,
                    (joint_limits,),
                )
                derived_boundary = DerivedBoundaryState(0.0, 0.0, 0.0)
                arm_attribute_tripwire(derived_boundary)
                object.__setattr__(
                    tampered_quintic_trajectory.axes[0],
                    field_name,
                    derived_boundary,
                )
                with self.assertRaisesRegex(
                    TrajectoryTimingError,
                    "boundaries must be built-in",
                ):
                    replan_synchronized_quintic_trajectory(
                        tampered_quintic_trajectory,
                        0.0,
                        (boundary(1.0),),
                        2.0,
                    )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "exceeds the active trajectory duration",
        ):
            replan_synchronized_quintic_trajectory(
                active,
                active.duration_seconds + 1.0,
                (boundary(1.0),),
                2.0,
            )
        with self.assertRaises(TrajectoryTimingError):
            replan_synchronized_quintic_trajectory(
                active,
                0.0,
                (boundary(1.0), boundary(2.0)),
                2.0,
            )

        first = QuinticJointTrajectory(
            boundary(0.0),
            boundary(1.0),
            2.0,
            joint_limits,
        )
        second = QuinticJointTrajectory(
            boundary(0.0),
            boundary(1.0),
            3.0,
            joint_limits,
        )
        with self.assertRaisesRegex(
            TrajectoryTimingError,
            "durations must match",
        ):
            SynchronizedQuinticTrajectory((first, second))
        with self.assertRaises(TrajectoryTimingError):
            SynchronizedQuinticTrajectory((object(),))


if __name__ == "__main__":
    unittest.main()
