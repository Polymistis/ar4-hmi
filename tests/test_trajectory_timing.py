from dataclasses import FrozenInstanceError, replace
from decimal import Decimal
import math
import unittest

from ARrobots.trajectory_timing import (
    TRAJECTORY_MAXIMUM_AXES,
    JointKinematicLimits,
    JointTrajectoryState,
    SynchronizedJointTrajectory,
    TrajectoryTimingError,
    minimum_rest_to_rest_joint_trajectory,
    plan_synchronized_rest_to_rest_trajectory,
)


def limits(velocity=10.0, acceleration=10.0, jerk=2.0):
    return JointKinematicLimits(velocity, acceleration, jerk)


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


if __name__ == "__main__":
    unittest.main()
