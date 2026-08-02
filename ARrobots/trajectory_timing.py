"""Hardware-free jerk-limited joint trajectory timing and sampling."""

import math
from dataclasses import dataclass
from decimal import Decimal
from numbers import Integral, Real
from typing import Tuple

from ARrobots.dynamic_motion import DynamicMotionError


TRAJECTORY_MAXIMUM_AXES = 9


class TrajectoryTimingError(DynamicMotionError):
    """A trajectory request or represented profile is invalid."""


def _finite_number(value, field_name):
    if isinstance(value, bool):
        raise TrajectoryTimingError(f"{field_name} must be numeric")
    try:
        if isinstance(value, Decimal):
            if not value.is_finite():
                raise TrajectoryTimingError(f"{field_name} must be finite")
            number = float(value)
            if value != 0 and number == 0:
                raise TrajectoryTimingError(
                    f"{field_name} is outside the host numeric range"
                )
        elif isinstance(value, (Integral, Real)):
            number = float(value)
        else:
            raise TypeError
    except TrajectoryTimingError:
        raise
    except OverflowError as exc:
        raise TrajectoryTimingError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    except (TypeError, ValueError) as exc:
        raise TrajectoryTimingError(f"{field_name} must be numeric") from exc
    if not math.isfinite(number):
        raise TrajectoryTimingError(f"{field_name} must be finite")
    return number


def _nonnegative_number(value, field_name):
    number = _finite_number(value, field_name)
    if number < 0:
        raise TrajectoryTimingError(f"{field_name} must be non-negative")
    if number == 0:
        return 0.0
    return number


def _positive_number(value, field_name):
    number = _finite_number(value, field_name)
    if number <= 0:
        raise TrajectoryTimingError(f"{field_name} must be positive")
    return number


def _calculated_number(value, field_name, *, positive=False):
    if not math.isfinite(value) or (positive and value <= 0):
        raise TrajectoryTimingError(
            f"{field_name} is outside the host numeric range"
        )
    if value == 0:
        return 0.0
    return value


def _calculated_sum(values, field_name, *, positive=False):
    try:
        value = math.fsum(values)
    except (OverflowError, ValueError) as exc:
        raise TrajectoryTimingError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    return _calculated_number(value, field_name, positive=positive)


def _comparison_tolerance(*values):
    # Cubic phase reconstruction and time scaling accumulate several rounded
    # products; the wider band is reserved for derived-value comparisons.
    return 128.0 * max(math.ulp(abs(value)) for value in values)


def _bounded_items(values, field_name):
    if isinstance(values, (str, bytes, bytearray)):
        raise TrajectoryTimingError(f"{field_name} must be a sequence")
    try:
        iterator = iter(values)
    except Exception as exc:
        raise TrajectoryTimingError(
            f"{field_name} must be a sequence"
        ) from exc
    items = []
    for _ in range(TRAJECTORY_MAXIMUM_AXES + 1):
        try:
            items.append(next(iterator))
        except StopIteration:
            break
        except Exception as exc:
            raise TrajectoryTimingError(
                f"{field_name} iteration failed"
            ) from exc
    if not items:
        raise TrajectoryTimingError(f"{field_name} must not be empty")
    if len(items) > TRAJECTORY_MAXIMUM_AXES:
        raise TrajectoryTimingError(
            f"{field_name} exceeds the maximum axis count"
        )
    return tuple(items)


def _displacement(start_position, target_position):
    displacement = target_position - start_position
    if not math.isfinite(displacement):
        raise TrajectoryTimingError(
            "joint displacement is outside the host numeric range"
        )
    if displacement == 0:
        return 0.0
    return displacement


@dataclass(frozen=True)
class JointKinematicLimits:
    maximum_velocity_degrees_per_second: float
    maximum_acceleration_degrees_per_second_squared: float
    maximum_jerk_degrees_per_second_cubed: float

    def __post_init__(self):
        for field_name in (
            "maximum_velocity_degrees_per_second",
            "maximum_acceleration_degrees_per_second_squared",
            "maximum_jerk_degrees_per_second_cubed",
        ):
            object.__setattr__(
                self,
                field_name,
                _positive_number(getattr(self, field_name), field_name),
            )


@dataclass(frozen=True)
class JointTrajectoryState:
    elapsed_seconds: float
    position_degrees: float
    velocity_degrees_per_second: float
    acceleration_degrees_per_second_squared: float
    jerk_degrees_per_second_cubed: float

    def __post_init__(self):
        object.__setattr__(
            self,
            "elapsed_seconds",
            _nonnegative_number(self.elapsed_seconds, "state elapsed_seconds"),
        )
        for field_name in (
            "position_degrees",
            "velocity_degrees_per_second",
            "acceleration_degrees_per_second_squared",
            "jerk_degrees_per_second_cubed",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite_number(getattr(self, field_name), f"state {field_name}"),
            )


@dataclass(frozen=True)
class JerkLimitedJointTrajectory:
    """Symmetric rest-to-rest seven-phase S-curve for one joint."""

    start_position_degrees: float
    target_position_degrees: float
    limits: JointKinematicLimits
    jerk_phase_seconds: float
    constant_acceleration_phase_seconds: float
    cruise_phase_seconds: float
    applied_jerk_degrees_per_second_cubed: float

    def __post_init__(self):
        start = _finite_number(
            self.start_position_degrees,
            "trajectory start_position_degrees",
        )
        target = _finite_number(
            self.target_position_degrees,
            "trajectory target_position_degrees",
        )
        if not isinstance(self.limits, JointKinematicLimits):
            raise TrajectoryTimingError(
                "trajectory limits must be JointKinematicLimits"
            )
        jerk_phase = _nonnegative_number(
            self.jerk_phase_seconds,
            "trajectory jerk_phase_seconds",
        )
        acceleration_phase = _nonnegative_number(
            self.constant_acceleration_phase_seconds,
            "trajectory constant_acceleration_phase_seconds",
        )
        cruise_phase = _nonnegative_number(
            self.cruise_phase_seconds,
            "trajectory cruise_phase_seconds",
        )
        applied_jerk = _nonnegative_number(
            self.applied_jerk_degrees_per_second_cubed,
            "trajectory applied_jerk_degrees_per_second_cubed",
        )
        object.__setattr__(self, "start_position_degrees", start)
        object.__setattr__(self, "target_position_degrees", target)
        object.__setattr__(self, "jerk_phase_seconds", jerk_phase)
        object.__setattr__(
            self,
            "constant_acceleration_phase_seconds",
            acceleration_phase,
        )
        object.__setattr__(self, "cruise_phase_seconds", cruise_phase)
        object.__setattr__(
            self,
            "applied_jerk_degrees_per_second_cubed",
            applied_jerk,
        )

        displacement = _displacement(start, target)
        duration = _calculated_sum(
            (
                4.0 * jerk_phase,
                2.0 * acceleration_phase,
                cruise_phase,
            ),
            "trajectory duration",
        )
        if displacement == 0:
            if applied_jerk != 0 or jerk_phase != 0 or acceleration_phase != 0:
                raise TrajectoryTimingError(
                    "stationary trajectory must not contain motion phases"
                )
            return
        if duration <= 0 or applied_jerk <= 0 or jerk_phase <= 0:
            raise TrajectoryTimingError(
                "moving trajectory requires positive jerk and duration"
            )
        jerk_tolerance = _comparison_tolerance(
            applied_jerk,
            self.limits.maximum_jerk_degrees_per_second_cubed,
        )
        if (
            applied_jerk
            > self.limits.maximum_jerk_degrees_per_second_cubed
            + jerk_tolerance
        ):
            raise TrajectoryTimingError("trajectory exceeds the jerk limit")

        peak_acceleration = _calculated_number(
            applied_jerk * jerk_phase,
            "trajectory peak acceleration",
            positive=True,
        )
        acceleration_tolerance = _comparison_tolerance(
            peak_acceleration,
            self.limits.maximum_acceleration_degrees_per_second_squared,
        )
        if (
            peak_acceleration
            > self.limits.maximum_acceleration_degrees_per_second_squared
            + acceleration_tolerance
        ):
            raise TrajectoryTimingError(
                "trajectory exceeds the acceleration limit"
            )

        peak_velocity = _calculated_number(
            peak_acceleration * (jerk_phase + acceleration_phase),
            "trajectory peak velocity",
            positive=True,
        )
        velocity_tolerance = _comparison_tolerance(
            peak_velocity,
            self.limits.maximum_velocity_degrees_per_second,
        )
        if (
            peak_velocity
            > self.limits.maximum_velocity_degrees_per_second
            + velocity_tolerance
        ):
            raise TrajectoryTimingError("trajectory exceeds the velocity limit")

        represented_distance = _calculated_number(
            peak_velocity * _calculated_sum(
                (
                    2.0 * jerk_phase,
                    acceleration_phase,
                    cruise_phase,
                ),
                "trajectory distance phase sum",
                positive=True,
            ),
            "trajectory represented distance",
            positive=True,
        )
        distance = abs(displacement)
        if abs(represented_distance - distance) > _comparison_tolerance(
            represented_distance,
            distance,
        ):
            raise TrajectoryTimingError(
                "trajectory phases do not represent the requested displacement"
            )

    @property
    def displacement_degrees(self):
        return self.target_position_degrees - self.start_position_degrees

    @property
    def direction(self):
        displacement = self.displacement_degrees
        if displacement > 0:
            return 1.0
        if displacement < 0:
            return -1.0
        return 0.0

    @property
    def duration_seconds(self):
        return _calculated_sum(
            (
                4.0 * self.jerk_phase_seconds,
                2.0 * self.constant_acceleration_phase_seconds,
                self.cruise_phase_seconds,
            ),
            "trajectory duration",
        )

    @property
    def peak_acceleration_degrees_per_second_squared(self):
        return (
            self.applied_jerk_degrees_per_second_cubed
            * self.jerk_phase_seconds
        )

    @property
    def peak_velocity_degrees_per_second(self):
        return (
            self.peak_acceleration_degrees_per_second_squared
            * (
                self.jerk_phase_seconds
                + self.constant_acceleration_phase_seconds
            )
        )

    def state_at(self, elapsed_seconds):
        elapsed = _nonnegative_number(
            elapsed_seconds,
            "trajectory elapsed_seconds",
        )
        duration = self.duration_seconds
        if elapsed >= duration:
            return JointTrajectoryState(
                elapsed,
                self.target_position_degrees,
                0.0,
                0.0,
                0.0,
            )
        if self.direction == 0:
            return JointTrajectoryState(
                elapsed,
                self.start_position_degrees,
                0.0,
                0.0,
                0.0,
            )

        jerk = self.applied_jerk_degrees_per_second_cubed
        jerk_phase = self.jerk_phase_seconds
        acceleration_phase = self.constant_acceleration_phase_seconds
        phases = (
            (jerk_phase, jerk),
            (acceleration_phase, 0.0),
            (jerk_phase, -jerk),
            (self.cruise_phase_seconds, 0.0),
            (jerk_phase, -jerk),
            (acceleration_phase, 0.0),
            (jerk_phase, jerk),
        )
        remaining = elapsed
        position = 0.0
        velocity = 0.0
        acceleration = 0.0
        active_jerk = 0.0
        for phase_duration, phase_jerk in phases:
            if phase_duration == 0:
                continue
            step = min(remaining, phase_duration)
            position = _calculated_sum(
                (
                    position,
                    velocity * step,
                    0.5 * acceleration * step * step,
                    phase_jerk * step * step * step / 6.0,
                ),
                "trajectory sampled position",
            )
            velocity = _calculated_sum(
                (
                    velocity,
                    acceleration * step,
                    0.5 * phase_jerk * step * step,
                ),
                "trajectory sampled velocity",
            )
            acceleration = _calculated_sum(
                (
                    acceleration,
                    phase_jerk * step,
                ),
                "trajectory sampled acceleration",
            )
            remaining -= step
            active_jerk = phase_jerk
            if remaining <= 0:
                break

        distance = abs(self.displacement_degrees)
        position_tolerance = _comparison_tolerance(position, distance)
        if position < -position_tolerance or position > distance + position_tolerance:
            raise TrajectoryTimingError(
                "trajectory sample is outside the represented displacement"
            )
        position = min(distance, max(0.0, position))
        direction = self.direction
        return JointTrajectoryState(
            elapsed,
            self.start_position_degrees + direction * position,
            direction * velocity,
            direction * acceleration,
            direction * active_jerk,
        )


@dataclass(frozen=True)
class SynchronizedJointTrajectory:
    axes: Tuple[JerkLimitedJointTrajectory, ...]

    def __post_init__(self):
        axes = _bounded_items(self.axes, "synchronized trajectory axes")
        if any(
            not isinstance(axis, JerkLimitedJointTrajectory)
            for axis in axes
        ):
            raise TrajectoryTimingError(
                "synchronized trajectory axes must contain joint trajectories"
            )
        duration = max(axis.duration_seconds for axis in axes)
        for axis in axes:
            if abs(axis.duration_seconds - duration) > _comparison_tolerance(
                axis.duration_seconds,
                duration,
            ):
                raise TrajectoryTimingError(
                    "synchronized trajectory axis durations must match"
                )
        object.__setattr__(self, "axes", axes)

    @property
    def duration_seconds(self):
        return max(axis.duration_seconds for axis in self.axes)

    @property
    def minimum_arrival_time_seconds(self):
        return self.duration_seconds

    def states_at(self, elapsed_seconds):
        elapsed = _nonnegative_number(
            elapsed_seconds,
            "synchronized trajectory elapsed_seconds",
        )
        return tuple(axis.state_at(elapsed) for axis in self.axes)

    def positions_at(self, elapsed_seconds):
        return tuple(
            state.position_degrees
            for state in self.states_at(elapsed_seconds)
        )


def minimum_rest_to_rest_joint_trajectory(
    start_position_degrees,
    target_position_degrees,
    limits,
):
    """Calculate the minimum symmetric S-curve under supplied joint limits."""
    start = _finite_number(
        start_position_degrees,
        "start_position_degrees",
    )
    target = _finite_number(
        target_position_degrees,
        "target_position_degrees",
    )
    if not isinstance(limits, JointKinematicLimits):
        raise TrajectoryTimingError("limits must be JointKinematicLimits")
    distance = abs(_displacement(start, target))
    if distance == 0:
        return JerkLimitedJointTrajectory(
            start,
            target,
            limits,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    maximum_velocity = limits.maximum_velocity_degrees_per_second
    maximum_acceleration = (
        limits.maximum_acceleration_degrees_per_second_squared
    )
    maximum_jerk = limits.maximum_jerk_degrees_per_second_cubed
    jerk_time_at_acceleration = _calculated_number(
        maximum_acceleration / maximum_jerk,
        "acceleration-limit jerk phase",
        positive=True,
    )
    jerk_time_at_velocity = _calculated_number(
        math.sqrt(maximum_velocity / maximum_jerk),
        "velocity-limit jerk phase",
        positive=True,
    )

    threshold_tolerance = _comparison_tolerance(
        jerk_time_at_acceleration,
        jerk_time_at_velocity,
    )
    if (
        jerk_time_at_velocity
        <= jerk_time_at_acceleration + threshold_tolerance
    ):
        jerk_phase = jerk_time_at_velocity
        acceleration_phase = 0.0
        velocity_distance = _calculated_number(
            maximum_velocity * (2.0 * jerk_phase),
            "velocity-limit displacement",
            positive=True,
        )
        distance_tolerance = _comparison_tolerance(
            distance,
            velocity_distance,
        )
        if distance < velocity_distance - distance_tolerance:
            jerk_phase = _calculated_number(
                math.cbrt((0.5 * distance) / maximum_jerk),
                "triangular jerk phase",
                positive=True,
            )
            cruise_phase = 0.0
        else:
            cruise_phase = _calculated_number(
                max(0.0, (distance - velocity_distance) / maximum_velocity),
                "cruise phase",
            )
    else:
        jerk_phase = jerk_time_at_acceleration
        acceleration_phase_at_velocity = _calculated_number(
            maximum_velocity / maximum_acceleration - jerk_phase,
            "velocity-limit constant-acceleration phase",
        )
        if acceleration_phase_at_velocity < 0:
            if abs(acceleration_phase_at_velocity) > threshold_tolerance:
                raise TrajectoryTimingError(
                    "trajectory limit thresholds are inconsistent"
                )
            acceleration_phase_at_velocity = 0.0
        acceleration_distance = _calculated_number(
            maximum_acceleration * jerk_phase * 2.0 * jerk_phase,
            "acceleration-limit displacement",
            positive=True,
        )
        velocity_distance = _calculated_number(
            maximum_velocity
            * _calculated_sum(
                (
                    2.0 * jerk_phase,
                    acceleration_phase_at_velocity,
                ),
                "velocity-limit phase sum",
                positive=True,
            ),
            "velocity-limit displacement",
            positive=True,
        )
        acceleration_tolerance = _comparison_tolerance(
            distance,
            acceleration_distance,
        )
        velocity_tolerance = _comparison_tolerance(
            distance,
            velocity_distance,
        )
        if distance < acceleration_distance - acceleration_tolerance:
            jerk_phase = _calculated_number(
                math.cbrt((0.5 * distance) / maximum_jerk),
                "triangular jerk phase",
                positive=True,
            )
            acceleration_phase = 0.0
            cruise_phase = 0.0
        elif distance < velocity_distance - velocity_tolerance:
            distance_over_acceleration = _calculated_number(
                distance / maximum_acceleration,
                "acceleration-limited displacement ratio",
                positive=True,
            )
            jerk_time_squared = _calculated_number(
                jerk_phase * jerk_phase,
                "acceleration-limit jerk phase square",
                positive=True,
            )
            numerator = _calculated_number(
                distance_over_acceleration - 2.0 * jerk_time_squared,
                "constant-acceleration phase numerator",
            )
            root = _calculated_number(
                math.hypot(
                    jerk_phase,
                    2.0 * math.sqrt(distance_over_acceleration),
                ),
                "constant-acceleration phase root",
                positive=True,
            )
            denominator = _calculated_number(
                3.0 * jerk_phase + root,
                "constant-acceleration phase denominator",
                positive=True,
            )
            acceleration_phase = _calculated_number(
                max(0.0, 2.0 * (numerator / denominator)),
                "constant-acceleration phase",
            )
            cruise_phase = 0.0
        else:
            acceleration_phase = acceleration_phase_at_velocity
            cruise_phase = _calculated_number(
                max(0.0, (distance - velocity_distance) / maximum_velocity),
                "cruise phase",
            )

    return JerkLimitedJointTrajectory(
        start,
        target,
        limits,
        jerk_phase,
        acceleration_phase,
        cruise_phase,
        maximum_jerk,
    )


def _scale_trajectory_duration(trajectory, duration_seconds):
    if trajectory.duration_seconds == duration_seconds:
        return trajectory
    if trajectory.displacement_degrees == 0:
        return JerkLimitedJointTrajectory(
            trajectory.start_position_degrees,
            trajectory.target_position_degrees,
            trajectory.limits,
            0.0,
            0.0,
            duration_seconds,
            0.0,
        )
    scale = _calculated_number(
        duration_seconds / trajectory.duration_seconds,
        "trajectory synchronization scale",
        positive=True,
    )
    tolerance = _comparison_tolerance(scale, 1.0)
    if scale < 1.0 - tolerance:
        raise TrajectoryTimingError(
            "synchronized duration cannot shorten a minimum trajectory"
        )
    scale = max(1.0, scale)
    applied_jerk = trajectory.applied_jerk_degrees_per_second_cubed
    for _ in range(3):
        applied_jerk = _calculated_number(
            applied_jerk / scale,
            "synchronized trajectory jerk",
            positive=True,
        )
    return JerkLimitedJointTrajectory(
        trajectory.start_position_degrees,
        trajectory.target_position_degrees,
        trajectory.limits,
        _calculated_number(
            trajectory.jerk_phase_seconds * scale,
            "synchronized jerk phase",
            positive=True,
        ),
        _calculated_number(
            trajectory.constant_acceleration_phase_seconds * scale,
            "synchronized constant-acceleration phase",
        ),
        _calculated_number(
            trajectory.cruise_phase_seconds * scale,
            "synchronized cruise phase",
        ),
        applied_jerk,
    )


def plan_synchronized_rest_to_rest_trajectory(
    start_positions_degrees,
    target_positions_degrees,
    limits,
):
    """Synchronize a bounded joint sequence by uniform time scaling."""
    starts = _bounded_items(start_positions_degrees, "start positions")
    targets = _bounded_items(target_positions_degrees, "target positions")
    joint_limits = _bounded_items(limits, "joint limits")
    if len(starts) != len(targets) or len(starts) != len(joint_limits):
        raise TrajectoryTimingError(
            "start positions, target positions, and joint limits must match"
        )
    minimum_trajectories = tuple(
        minimum_rest_to_rest_joint_trajectory(start, target, axis_limits)
        for start, target, axis_limits in zip(starts, targets, joint_limits)
    )
    synchronized_duration = max(
        trajectory.duration_seconds
        for trajectory in minimum_trajectories
    )
    return SynchronizedJointTrajectory(tuple(
        _scale_trajectory_duration(trajectory, synchronized_duration)
        for trajectory in minimum_trajectories
    ))
