"""Hardware-free jerk-limited joint trajectory timing and sampling."""

import math
from dataclasses import dataclass, field
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


def _calculated_product(values, field_name):
    factors = tuple(
        _calculated_number(value, field_name)
        for value in values
    )
    if any(factor == 0 for factor in factors):
        return 0.0
    sign = 1.0
    significand = 1.0
    exponent = 0
    for factor in factors:
        if factor < 0:
            sign = -sign
        factor_significand, factor_exponent = math.frexp(abs(factor))
        significand *= factor_significand
        significand, normalization_exponent = math.frexp(significand)
        exponent += factor_exponent + normalization_exponent
    try:
        product = math.ldexp(sign * significand, exponent)
    except OverflowError as exc:
        raise TrajectoryTimingError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    product = _calculated_number(product, field_name)
    if product == 0:
        raise TrajectoryTimingError(
            f"{field_name} is outside the host numeric range"
        )
    return product


def _calculated_ratio(numerator, denominator, field_name):
    numerator = _calculated_number(numerator, field_name)
    denominator = _calculated_number(denominator, field_name)
    if denominator == 0:
        raise TrajectoryTimingError(f"{field_name} divisor must be nonzero")
    result = _calculated_number(numerator / denominator, field_name)
    if numerator != 0 and result == 0:
        raise TrajectoryTimingError(
            f"{field_name} is outside the host numeric range"
        )
    return result


def _comparison_tolerance(*values):
    # Trajectory integration, polynomial reconstruction, and time scaling
    # accumulate rounded products; duration boundaries also need that band.
    return 128.0 * max(math.ulp(abs(value)) for value in values)


def _polynomial_value(coefficients, argument, field_name):
    value = 0.0
    for coefficient in reversed(coefficients):
        value = _calculated_sum(
            (
                _calculated_product((value, argument), field_name),
                coefficient,
            ),
            field_name,
        )
    return value


def _polynomial_derivative(coefficients, field_name):
    if len(coefficients) <= 1:
        return (0.0,)
    return tuple(
        _calculated_product((index, coefficient), field_name)
        for index, coefficient in enumerate(coefficients[1:], start=1)
    )


def _polynomial_zero_tolerance(coefficients, value):
    return len(coefficients) * _comparison_tolerance(value, *coefficients)


def _append_unit_root(roots, root):
    unit_tolerance = 512.0 * math.ulp(1.0)
    if root < -unit_tolerance or root > 1.0 + unit_tolerance:
        return
    root = min(1.0, max(0.0, root))
    if not any(abs(existing - root) <= unit_tolerance for existing in roots):
        roots.append(root)


def _unit_interval_polynomial_roots(coefficients, field_name):
    coefficients = tuple(
        _calculated_number(coefficient, field_name)
        for coefficient in coefficients
    )
    while len(coefficients) > 1 and coefficients[-1] == 0:
        coefficients = coefficients[:-1]
    degree = len(coefficients) - 1
    if degree <= 0:
        return ()
    if degree == 1:
        root = _calculated_ratio(
            -coefficients[0],
            coefficients[1],
            field_name,
        )
        roots = []
        _append_unit_root(roots, root)
        return tuple(roots)

    derivative = _polynomial_derivative(coefficients, field_name)
    critical_points = _unit_interval_polynomial_roots(
        derivative,
        field_name,
    )
    partition = (0.0, *critical_points, 1.0)
    roots = []
    values = tuple(
        _polynomial_value(coefficients, point, field_name)
        for point in partition
    )
    for point, value in zip(partition, values):
        if abs(value) <= _polynomial_zero_tolerance(coefficients, value):
            _append_unit_root(roots, point)

    for lower, upper, lower_value, upper_value in zip(
        partition,
        partition[1:],
        values,
        values[1:],
    ):
        if lower_value == 0 or upper_value == 0:
            continue
        if (lower_value < 0) == (upper_value < 0):
            continue
        for _ in range(80):
            midpoint = (lower + upper) / 2.0
            if midpoint == lower or midpoint == upper:
                break
            midpoint_value = _polynomial_value(
                coefficients,
                midpoint,
                field_name,
            )
            if midpoint_value == 0:
                lower = midpoint
                upper = midpoint
                break
            if (lower_value < 0) == (midpoint_value < 0):
                lower = midpoint
                lower_value = midpoint_value
            else:
                upper = midpoint
        _append_unit_root(roots, (lower + upper) / 2.0)
    return tuple(sorted(roots))


def _scaled_polynomial_value(
    coefficients,
    argument,
    divisor,
    field_name,
):
    return _calculated_ratio(
        _polynomial_value(coefficients, argument, field_name),
        divisor,
        field_name,
    )


def _peak_absolute_polynomial_value(
    coefficients,
    critical_coefficients,
    divisor,
    field_name,
):
    candidates = (
        0.0,
        *_unit_interval_polynomial_roots(
            critical_coefficients,
            field_name,
        ),
        1.0,
    )
    return max(
        abs(_scaled_polynomial_value(
            coefficients,
            argument,
            divisor,
            field_name,
        ))
        for argument in candidates
    )


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
class JointBoundaryState:
    position_degrees: float
    velocity_degrees_per_second: float
    acceleration_degrees_per_second_squared: float

    def __post_init__(self):
        for field_name in (
            "position_degrees",
            "velocity_degrees_per_second",
            "acceleration_degrees_per_second_squared",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite_number(
                    getattr(self, field_name),
                    f"boundary {field_name}",
                ),
            )


def _quintic_coefficients(start_state, target_state, duration_seconds):
    displacement = _displacement(
        start_state.position_degrees,
        target_state.position_degrees,
    )
    duration_squared = _calculated_product(
        (duration_seconds, duration_seconds),
        "quintic duration square",
    )
    start_velocity_time = _calculated_product(
        (start_state.velocity_degrees_per_second, duration_seconds),
        "quintic start velocity term",
    )
    target_velocity_time = _calculated_product(
        (target_state.velocity_degrees_per_second, duration_seconds),
        "quintic target velocity term",
    )
    start_acceleration_time_squared = _calculated_product(
        (
            start_state.acceleration_degrees_per_second_squared,
            duration_squared,
        ),
        "quintic start acceleration term",
    )
    target_acceleration_time_squared = _calculated_product(
        (
            target_state.acceleration_degrees_per_second_squared,
            duration_squared,
        ),
        "quintic target acceleration term",
    )
    coefficient_0 = start_state.position_degrees
    coefficient_1 = start_velocity_time
    coefficient_2 = _calculated_product(
        (0.5, start_acceleration_time_squared),
        "quintic coefficient 2",
    )
    coefficient_3 = _calculated_sum(
        (
            _calculated_product((10.0, displacement), "quintic coefficient 3"),
            _calculated_product(
                (-6.0, start_velocity_time),
                "quintic coefficient 3",
            ),
            _calculated_product(
                (-4.0, target_velocity_time),
                "quintic coefficient 3",
            ),
            _calculated_product(
                (-1.5, start_acceleration_time_squared),
                "quintic coefficient 3",
            ),
            _calculated_product(
                (0.5, target_acceleration_time_squared),
                "quintic coefficient 3",
            ),
        ),
        "quintic coefficient 3",
    )
    coefficient_4 = _calculated_sum(
        (
            _calculated_product((-15.0, displacement), "quintic coefficient 4"),
            _calculated_product(
                (8.0, start_velocity_time),
                "quintic coefficient 4",
            ),
            _calculated_product(
                (7.0, target_velocity_time),
                "quintic coefficient 4",
            ),
            _calculated_product(
                (1.5, start_acceleration_time_squared),
                "quintic coefficient 4",
            ),
            -target_acceleration_time_squared,
        ),
        "quintic coefficient 4",
    )
    coefficient_5 = _calculated_sum(
        (
            _calculated_product((6.0, displacement), "quintic coefficient 5"),
            _calculated_product(
                (-3.0, start_velocity_time),
                "quintic coefficient 5",
            ),
            _calculated_product(
                (-3.0, target_velocity_time),
                "quintic coefficient 5",
            ),
            _calculated_product(
                (-0.5, start_acceleration_time_squared),
                "quintic coefficient 5",
            ),
            _calculated_product(
                (0.5, target_acceleration_time_squared),
                "quintic coefficient 5",
            ),
        ),
        "quintic coefficient 5",
    )
    return (
        coefficient_0,
        coefficient_1,
        coefficient_2,
        coefficient_3,
        coefficient_4,
        coefficient_5,
    )


def _reconstruction_tolerance(expected, calculated, component_values):
    scale = max(1.0, abs(expected), abs(calculated))
    endpoint_tolerance = 4096.0 * math.ulp(scale)
    roundoff_tolerance = len(component_values) * _comparison_tolerance(
        expected,
        calculated,
        *component_values,
    )
    precision_cap = math.sqrt(math.ulp(1.0)) * scale
    return max(
        endpoint_tolerance,
        min(roundoff_tolerance, precision_cap),
    )


def _validate_reconstructed_boundary(
    expected,
    calculated,
    component_values,
    field_name,
):
    if abs(expected - calculated) > _reconstruction_tolerance(
        expected,
        calculated,
        component_values,
    ):
        raise TrajectoryTimingError(
            f"{field_name} cannot be represented in the host numeric range"
        )


def _scaled_polynomial_components(coefficients, divisor, field_name):
    return tuple(
        _calculated_ratio(coefficient, divisor, field_name)
        for coefficient in coefficients
    )


def _validate_quintic_boundary_reconstruction(
    boundary_state,
    argument,
    position_coefficients,
    velocity_coefficients,
    acceleration_coefficients,
    duration_seconds,
    duration_squared,
    label,
):
    position = _polynomial_value(
        position_coefficients,
        argument,
        f"quintic {label} position",
    )
    velocity = _scaled_polynomial_value(
        velocity_coefficients,
        argument,
        duration_seconds,
        f"quintic {label} velocity",
    )
    acceleration = _scaled_polynomial_value(
        acceleration_coefficients,
        argument,
        duration_squared,
        f"quintic {label} acceleration",
    )
    _validate_reconstructed_boundary(
        boundary_state.position_degrees,
        position,
        position_coefficients,
        f"quintic {label} position",
    )
    _validate_reconstructed_boundary(
        boundary_state.velocity_degrees_per_second,
        velocity,
        _scaled_polynomial_components(
            velocity_coefficients,
            duration_seconds,
            f"quintic {label} velocity component",
        ),
        f"quintic {label} velocity",
    )
    _validate_reconstructed_boundary(
        boundary_state.acceleration_degrees_per_second_squared,
        acceleration,
        _scaled_polynomial_components(
            acceleration_coefficients,
            duration_squared,
            f"quintic {label} acceleration component",
        ),
        f"quintic {label} acceleration",
    )


@dataclass(frozen=True)
class QuinticJointTrajectory:
    """A fixed-duration segment with C2 endpoints and bounded jerk."""

    start_state: JointBoundaryState
    target_state: JointBoundaryState
    duration_seconds: float
    limits: JointKinematicLimits
    _coefficients: Tuple[float, ...] = field(
        init=False,
        repr=False,
        compare=False,
    )
    _velocity_coefficients: Tuple[float, ...] = field(
        init=False,
        repr=False,
        compare=False,
    )
    _acceleration_coefficients: Tuple[float, ...] = field(
        init=False,
        repr=False,
        compare=False,
    )
    _jerk_coefficients: Tuple[float, ...] = field(
        init=False,
        repr=False,
        compare=False,
    )
    _duration_squared: float = field(
        init=False,
        repr=False,
        compare=False,
    )
    _duration_cubed: float = field(
        init=False,
        repr=False,
        compare=False,
    )
    _peak_absolute_velocity: float = field(
        init=False,
        repr=False,
        compare=False,
    )
    _peak_absolute_acceleration: float = field(
        init=False,
        repr=False,
        compare=False,
    )
    _peak_absolute_jerk: float = field(
        init=False,
        repr=False,
        compare=False,
    )

    def __post_init__(self):
        if type(self.start_state) is not JointBoundaryState:
            raise TrajectoryTimingError(
                "quintic start_state must be built-in JointBoundaryState"
            )
        if type(self.target_state) is not JointBoundaryState:
            raise TrajectoryTimingError(
                "quintic target_state must be built-in JointBoundaryState"
            )
        duration = _positive_number(
            self.duration_seconds,
            "quintic duration_seconds",
        )
        if type(self.limits) is not JointKinematicLimits:
            raise TrajectoryTimingError(
                "quintic limits must be built-in JointKinematicLimits"
            )
        object.__setattr__(self, "duration_seconds", duration)

        duration_squared = _calculated_product(
            (duration, duration),
            "quintic duration square",
        )
        duration_cubed = _calculated_product(
            (duration_squared, duration),
            "quintic duration cube",
        )
        coefficients = _quintic_coefficients(
            self.start_state,
            self.target_state,
            duration,
        )
        velocity_coefficients = _polynomial_derivative(
            coefficients,
            "quintic velocity polynomial",
        )
        acceleration_coefficients = _polynomial_derivative(
            velocity_coefficients,
            "quintic acceleration polynomial",
        )
        jerk_coefficients = _polynomial_derivative(
            acceleration_coefficients,
            "quintic jerk polynomial",
        )
        snap_coefficients = _polynomial_derivative(
            jerk_coefficients,
            "quintic snap polynomial",
        )
        _validate_quintic_boundary_reconstruction(
            self.start_state,
            0.0,
            coefficients,
            velocity_coefficients,
            acceleration_coefficients,
            duration,
            duration_squared,
            "start",
        )
        _validate_quintic_boundary_reconstruction(
            self.target_state,
            1.0,
            coefficients,
            velocity_coefficients,
            acceleration_coefficients,
            duration,
            duration_squared,
            "target",
        )

        peak_velocity = _peak_absolute_polynomial_value(
            velocity_coefficients,
            acceleration_coefficients,
            duration,
            "quintic velocity extrema",
        )
        peak_acceleration = _peak_absolute_polynomial_value(
            acceleration_coefficients,
            jerk_coefficients,
            duration_squared,
            "quintic acceleration extrema",
        )
        peak_jerk = _peak_absolute_polynomial_value(
            jerk_coefficients,
            snap_coefficients,
            duration_cubed,
            "quintic jerk extrema",
        )
        limit_values = (
            (
                peak_velocity,
                self.limits.maximum_velocity_degrees_per_second,
                "velocity",
            ),
            (
                peak_acceleration,
                self.limits.maximum_acceleration_degrees_per_second_squared,
                "acceleration",
            ),
            (
                peak_jerk,
                self.limits.maximum_jerk_degrees_per_second_cubed,
                "jerk",
            ),
        )
        for peak, maximum, label in limit_values:
            if peak > maximum + _comparison_tolerance(peak, maximum):
                raise TrajectoryTimingError(
                    f"quintic trajectory exceeds the {label} limit"
                )

        object.__setattr__(self, "_coefficients", coefficients)
        object.__setattr__(
            self,
            "_velocity_coefficients",
            velocity_coefficients,
        )
        object.__setattr__(
            self,
            "_acceleration_coefficients",
            acceleration_coefficients,
        )
        object.__setattr__(self, "_jerk_coefficients", jerk_coefficients)
        object.__setattr__(self, "_duration_squared", duration_squared)
        object.__setattr__(self, "_duration_cubed", duration_cubed)
        object.__setattr__(self, "_peak_absolute_velocity", peak_velocity)
        object.__setattr__(
            self,
            "_peak_absolute_acceleration",
            peak_acceleration,
        )
        object.__setattr__(self, "_peak_absolute_jerk", peak_jerk)

    @property
    def peak_absolute_velocity_degrees_per_second(self):
        return self._peak_absolute_velocity

    @property
    def peak_absolute_acceleration_degrees_per_second_squared(self):
        return self._peak_absolute_acceleration

    @property
    def peak_absolute_jerk_degrees_per_second_cubed(self):
        return self._peak_absolute_jerk

    def state_at(self, elapsed_seconds):
        elapsed = _nonnegative_number(
            elapsed_seconds,
            "quintic elapsed_seconds",
        )
        duration_tolerance = _comparison_tolerance(
            elapsed,
            self.duration_seconds,
        )
        if elapsed > self.duration_seconds + duration_tolerance:
            raise TrajectoryTimingError(
                "quintic elapsed_seconds exceeds the segment duration"
            )
        elapsed = min(elapsed, self.duration_seconds)
        argument = _calculated_ratio(
            elapsed,
            self.duration_seconds,
            "quintic normalized elapsed time",
        )
        position = _polynomial_value(
            self._coefficients,
            argument,
            "quintic sampled position",
        )
        velocity = _scaled_polynomial_value(
            self._velocity_coefficients,
            argument,
            self.duration_seconds,
            "quintic sampled velocity",
        )
        acceleration = _scaled_polynomial_value(
            self._acceleration_coefficients,
            argument,
            self._duration_squared,
            "quintic sampled acceleration",
        )
        jerk = _scaled_polynomial_value(
            self._jerk_coefficients,
            argument,
            self._duration_cubed,
            "quintic sampled jerk",
        )
        if elapsed == 0:
            position = self.start_state.position_degrees
            velocity = self.start_state.velocity_degrees_per_second
            acceleration = (
                self.start_state.acceleration_degrees_per_second_squared
            )
        elif elapsed == self.duration_seconds:
            position = self.target_state.position_degrees
            velocity = self.target_state.velocity_degrees_per_second
            acceleration = (
                self.target_state.acceleration_degrees_per_second_squared
            )
        return JointTrajectoryState(
            elapsed,
            position,
            velocity,
            acceleration,
            jerk,
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
        if type(self.limits) is not JointKinematicLimits:
            raise TrajectoryTimingError(
                "trajectory limits must be built-in JointKinematicLimits"
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
            type(axis) is not JerkLimitedJointTrajectory
            for axis in axes
        ):
            raise TrajectoryTimingError(
                "synchronized trajectory axes must contain built-in joint trajectories"
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


@dataclass(frozen=True)
class SynchronizedQuinticTrajectory:
    axes: Tuple[QuinticJointTrajectory, ...]

    def __post_init__(self):
        axes = _bounded_items(
            self.axes,
            "synchronized quintic trajectory axes",
        )
        if any(
            type(axis) is not QuinticJointTrajectory
            for axis in axes
        ):
            raise TrajectoryTimingError(
                "synchronized quintic axes must contain built-in quintic trajectories"
            )
        duration = max(axis.duration_seconds for axis in axes)
        for axis in axes:
            if abs(axis.duration_seconds - duration) > _comparison_tolerance(
                axis.duration_seconds,
                duration,
            ):
                raise TrajectoryTimingError(
                    "synchronized quintic axis durations must match"
                )
        object.__setattr__(self, "axes", axes)

    @property
    def duration_seconds(self):
        return max(axis.duration_seconds for axis in self.axes)

    def states_at(self, elapsed_seconds):
        elapsed = _nonnegative_number(
            elapsed_seconds,
            "synchronized quintic elapsed_seconds",
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
    if type(limits) is not JointKinematicLimits:
        raise TrajectoryTimingError(
            "limits must be built-in JointKinematicLimits"
        )
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


def plan_synchronized_quintic_trajectory(
    start_states,
    target_states,
    duration_seconds,
    limits,
):
    """Reject rather than alter a caller-selected synchronized duration."""
    starts = _bounded_items(start_states, "quintic start states")
    targets = _bounded_items(target_states, "quintic target states")
    joint_limits = _bounded_items(limits, "quintic joint limits")
    if len(starts) != len(targets) or len(starts) != len(joint_limits):
        raise TrajectoryTimingError(
            "quintic start states, target states, and joint limits must match"
        )
    duration = _positive_number(
        duration_seconds,
        "quintic duration_seconds",
    )
    return SynchronizedQuinticTrajectory(tuple(
        QuinticJointTrajectory(start, target, duration, axis_limits)
        for start, target, axis_limits in zip(
            starts,
            targets,
            joint_limits,
        )
    ))


def replan_synchronized_quintic_trajectory(
    active_trajectory,
    elapsed_seconds,
    target_states,
    duration_seconds,
):
    """Preserve sampled desired C2 state; jerk continuity is not guaranteed."""
    active_trajectory_type = type(active_trajectory)
    if active_trajectory_type is SynchronizedJointTrajectory:
        required_axis_type = JerkLimitedJointTrajectory
    elif active_trajectory_type is SynchronizedQuinticTrajectory:
        required_axis_type = QuinticJointTrajectory
    else:
        raise TrajectoryTimingError(
            "active trajectory must be a synchronized built-in trajectory"
        )
    if any(
        type(axis) is not required_axis_type
        for axis in active_trajectory.axes
    ):
        raise TrajectoryTimingError(
            "active trajectory axes must be built-in joint trajectories"
        )
    if any(
        type(axis.limits) is not JointKinematicLimits
        for axis in active_trajectory.axes
    ):
        raise TrajectoryTimingError(
            "active trajectory limits must be built-in joint limits"
        )
    if required_axis_type is QuinticJointTrajectory and any(
        type(axis.start_state) is not JointBoundaryState
        or type(axis.target_state) is not JointBoundaryState
        for axis in active_trajectory.axes
    ):
        raise TrajectoryTimingError(
            "active trajectory boundaries must be built-in joint states"
        )
    elapsed = _nonnegative_number(
        elapsed_seconds,
        "replan elapsed_seconds",
    )
    duration_tolerance = _comparison_tolerance(
        elapsed,
        active_trajectory.duration_seconds,
    )
    if elapsed > active_trajectory.duration_seconds + duration_tolerance:
        raise TrajectoryTimingError(
            "replan elapsed_seconds exceeds the active trajectory duration"
        )
    elapsed = min(elapsed, active_trajectory.duration_seconds)
    sampled_states = active_trajectory.states_at(elapsed)
    start_states = tuple(
        JointBoundaryState(
            state.position_degrees,
            state.velocity_degrees_per_second,
            state.acceleration_degrees_per_second_squared,
        )
        for state in sampled_states
    )
    joint_limits = tuple(axis.limits for axis in active_trajectory.axes)
    return plan_synchronized_quintic_trajectory(
        start_states,
        target_states,
        duration_seconds,
        joint_limits,
    )
