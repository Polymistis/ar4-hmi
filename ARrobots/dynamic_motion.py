"""Deterministic observation replay, estimation, and prediction contracts."""

import json
import math
import re
from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from numbers import Integral, Real
from typing import Optional, Protocol, Tuple, TypeVar


OBSERVATION_REPLAY_SCHEMA = "ar4.observation-replay.v1"
OBSERVATION_REPLAY_TIMEBASE = "monotonic-seconds"
OBSERVATION_REPLAY_POSITION_UNIT = "millimeter"
OBSERVATION_REPLAY_MAXIMUM_BYTES = 8 * 1024 * 1024
OBSERVATION_REPLAY_MAXIMUM_RECORDS = 100_000
OBSERVATION_REPLAY_MAXIMUM_LINE_BYTES = 4096
# Lead/skew admission can accumulate two four-ULP comparison bands. Reserving
# two additional bands for timestamp composition and output validation keeps
# producer and selector tolerances aligned without letting the advertised
# horizon widen timestamp validity.
PREDICTION_TIMESTAMP_TOLERANCE_ULPS = 16.0
_FRAME_ID_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_.:/-]{0,63}\Z")


class DynamicMotionError(ValueError):
    """Base error for rejected dynamic-motion input."""


class ObservationValidationError(DynamicMotionError):
    """An observation or estimator setting violates the input contract."""


class StaleObservationError(ObservationValidationError):
    """An observation exceeded the configured age bound."""


class FutureObservationError(ObservationValidationError):
    """An observation timestamp exceeded the configured clock-skew bound."""


class OutOfOrderObservationError(ObservationValidationError):
    """A source or receipt timestamp violated the ordering contract."""


class PredictionError(DynamicMotionError):
    """A requested prediction violates the predictor contract."""


class ReplayFormatError(DynamicMotionError):
    """A replay payload, codec setting, or encoded result is invalid."""


def _finite_number(value, field_name):
    if isinstance(value, bool):
        raise ObservationValidationError(f"{field_name} must be numeric")
    try:
        if isinstance(value, Decimal):
            if not value.is_finite():
                raise ObservationValidationError(
                    f"{field_name} must be finite"
                )
            number = float(value)
            if value != 0 and number == 0:
                raise ObservationValidationError(
                    f"{field_name} is outside the host numeric range"
                )
        elif isinstance(value, (Integral, Real)):
            number = float(value)
        else:
            raise TypeError
    except ObservationValidationError:
        raise
    except OverflowError as exc:
        raise ObservationValidationError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    except (TypeError, ValueError) as exc:
        raise ObservationValidationError(
            f"{field_name} must be numeric"
        ) from exc
    if not math.isfinite(number):
        raise ObservationValidationError(f"{field_name} must be finite")
    return number


def _nonnegative_number(value, field_name):
    number = _finite_number(value, field_name)
    if number < 0:
        raise ObservationValidationError(
            f"{field_name} must be non-negative"
        )
    if number == 0:
        return 0.0
    return number


def _positive_number(value, field_name):
    number = _finite_number(value, field_name)
    if number <= 0:
        raise ObservationValidationError(f"{field_name} must be positive")
    return number


def _comparison_tolerance(*values):
    return 4.0 * max(math.ulp(abs(value)) for value in values)


def _prediction_number(value, field_name, *, positive=False):
    try:
        if positive:
            return _positive_number(value, field_name)
        return _nonnegative_number(value, field_name)
    except ObservationValidationError as exc:
        raise PredictionError(str(exc)) from exc


def _prediction_frame_id(value):
    try:
        return _frame_id(value)
    except ObservationValidationError as exc:
        raise PredictionError(str(exc)) from exc


def prediction_timestamp_tolerance(
    source_timestamp_seconds,
    target_timestamp_seconds,
    returned_timestamp_seconds=None,
):
    """Scale the shared ULP budget to the largest timestamp operand."""
    values = [
        _prediction_number(
            source_timestamp_seconds,
            "prediction tolerance source_timestamp_seconds",
        ),
        _prediction_number(
            target_timestamp_seconds,
            "prediction tolerance target_timestamp_seconds",
        ),
    ]
    if returned_timestamp_seconds is not None:
        values.append(_prediction_number(
            returned_timestamp_seconds,
            "prediction tolerance returned_timestamp_seconds",
        ))
    return PREDICTION_TIMESTAMP_TOLERANCE_ULPS * max(
        math.ulp(abs(value)) for value in values
    )


def _fixed_items(values, expected_length, field_name):
    if isinstance(values, (str, bytes, bytearray)):
        raise ObservationValidationError(f"{field_name} must be a sequence")
    try:
        iterator = iter(values)
    except Exception as exc:
        raise ObservationValidationError(
            f"{field_name} must be a sequence"
        ) from exc
    items = []
    for _ in range(expected_length + 1):
        try:
            items.append(next(iterator))
        except StopIteration:
            break
        except Exception as exc:
            raise ObservationValidationError(
                f"{field_name} iteration failed"
            ) from exc
    if len(items) != expected_length:
        raise ObservationValidationError(
            f"{field_name} must contain {expected_length} values"
        )
    return tuple(items)


def _frame_id(value):
    if not isinstance(value, str) or not _FRAME_ID_PATTERN.fullmatch(value):
        raise ObservationValidationError(
            "frame_id must match [A-Za-z][A-Za-z0-9_.:/-]{0,63}"
        )
    return value


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def __post_init__(self):
        object.__setattr__(self, "x", _finite_number(self.x, "vector x"))
        object.__setattr__(self, "y", _finite_number(self.y, "vector y"))
        object.__setattr__(self, "z", _finite_number(self.z, "vector z"))

    @classmethod
    def from_sequence(cls, values, field_name="vector"):
        items = _fixed_items(values, 3, field_name)
        return cls(*(
            _finite_number(value, f"{field_name}[{axis}]")
            for axis, value in enumerate(items)
        ))

    def components(self):
        return self.x, self.y, self.z


@dataclass(frozen=True)
class DiagonalCovariance3:
    x_variance: float
    y_variance: float
    z_variance: float

    def __post_init__(self):
        for field_name in (
            "x_variance",
            "y_variance",
            "z_variance",
        ):
            object.__setattr__(
                self,
                field_name,
                _nonnegative_number(
                    getattr(self, field_name),
                    field_name,
                ),
            )

    @classmethod
    def from_sequence(cls, values, field_name="position_variance"):
        items = _fixed_items(values, 3, field_name)
        return cls(*(
            _nonnegative_number(value, f"{field_name}[{axis}]")
            for axis, value in enumerate(items)
        ))

    def components(self):
        return self.x_variance, self.y_variance, self.z_variance


@dataclass(frozen=True)
class AxisStateCovariance:
    position_variance: float
    velocity_variance: float
    position_velocity_covariance: float

    def __post_init__(self):
        position_variance = _nonnegative_number(
            self.position_variance,
            "position_variance",
        )
        velocity_variance = _nonnegative_number(
            self.velocity_variance,
            "velocity_variance",
        )
        cross_covariance = _finite_number(
            self.position_velocity_covariance,
            "position_velocity_covariance",
        )
        covariance_limit = (
            math.sqrt(position_variance) * math.sqrt(velocity_variance)
        )
        if covariance_limit == 0 and cross_covariance != 0:
            raise ObservationValidationError(
                "axis state covariance must be positive semidefinite"
            )
        tolerance = _comparison_tolerance(
            covariance_limit,
            cross_covariance,
        )
        if abs(cross_covariance) > covariance_limit + tolerance:
            raise ObservationValidationError(
                "axis state covariance must be positive semidefinite"
            )
        object.__setattr__(self, "position_variance", position_variance)
        object.__setattr__(self, "velocity_variance", velocity_variance)
        object.__setattr__(
            self,
            "position_velocity_covariance",
            cross_covariance,
        )


def _bounded_correlation(first_variance, second_variance, covariance):
    covariance_limit = (
        math.sqrt(first_variance) * math.sqrt(second_variance)
    )
    if covariance_limit == 0:
        if covariance != 0:
            raise ObservationValidationError(
                "axis acceleration covariance must be positive semidefinite"
            )
        return 0.0
    tolerance = _comparison_tolerance(covariance_limit, covariance)
    if abs(covariance) > covariance_limit + tolerance:
        raise ObservationValidationError(
            "axis acceleration covariance must be positive semidefinite"
        )
    return max(-1.0, min(1.0, covariance / covariance_limit))


@dataclass(frozen=True)
class AxisAccelerationStateCovariance(AxisStateCovariance):
    acceleration_variance: float
    position_acceleration_covariance: float
    velocity_acceleration_covariance: float

    def __post_init__(self):
        super().__post_init__()
        acceleration_variance = _nonnegative_number(
            self.acceleration_variance,
            "acceleration_variance",
        )
        position_acceleration_covariance = _finite_number(
            self.position_acceleration_covariance,
            "position_acceleration_covariance",
        )
        velocity_acceleration_covariance = _finite_number(
            self.velocity_acceleration_covariance,
            "velocity_acceleration_covariance",
        )
        correlations = (
            _bounded_correlation(
                self.position_variance,
                self.velocity_variance,
                self.position_velocity_covariance,
            ),
            _bounded_correlation(
                self.position_variance,
                acceleration_variance,
                position_acceleration_covariance,
            ),
            _bounded_correlation(
                self.velocity_variance,
                acceleration_variance,
                velocity_acceleration_covariance,
            ),
        )
        position_velocity, position_acceleration, velocity_acceleration = (
            correlations
        )
        determinant_terms = (
            1.0,
            2.0
            * position_velocity
            * position_acceleration
            * velocity_acceleration,
            -(position_velocity * position_velocity),
            -(position_acceleration * position_acceleration),
            -(velocity_acceleration * velocity_acceleration),
        )
        determinant = math.fsum(determinant_terms)
        determinant_tolerance = _comparison_tolerance(*determinant_terms)
        if determinant < -determinant_tolerance:
            raise ObservationValidationError(
                "axis acceleration covariance must be positive semidefinite"
            )
        object.__setattr__(
            self,
            "acceleration_variance",
            acceleration_variance,
        )
        object.__setattr__(
            self,
            "position_acceleration_covariance",
            position_acceleration_covariance,
        )
        object.__setattr__(
            self,
            "velocity_acceleration_covariance",
            velocity_acceleration_covariance,
        )


@dataclass(frozen=True)
class StateCovariance3:
    x_axis: AxisStateCovariance
    y_axis: AxisStateCovariance
    z_axis: AxisStateCovariance

    def __post_init__(self):
        if any(
            not isinstance(axis, AxisStateCovariance)
            for axis in self.axes()
        ):
            raise ObservationValidationError(
                "state covariance must contain three axis covariances"
            )

    def axes(self):
        return self.x_axis, self.y_axis, self.z_axis

    def position_diagonal(self):
        return DiagonalCovariance3(*(
            axis.position_variance for axis in self.axes()
        ))


@dataclass(frozen=True)
class AccelerationStateCovariance3(StateCovariance3):
    x_axis: AxisAccelerationStateCovariance
    y_axis: AxisAccelerationStateCovariance
    z_axis: AxisAccelerationStateCovariance

    def __post_init__(self):
        if any(
            not isinstance(axis, AxisAccelerationStateCovariance)
            for axis in self.axes()
        ):
            raise ObservationValidationError(
                "acceleration state covariance must contain three axis "
                "covariances"
            )


@dataclass(frozen=True)
class PositionObservation:
    """Timestamped millimeter position with diagonal square-millimeter variance."""

    timestamp_seconds: float
    frame_id: str
    position: Vector3
    position_variance: DiagonalCovariance3

    def __post_init__(self):
        object.__setattr__(
            self,
            "timestamp_seconds",
            _nonnegative_number(
                self.timestamp_seconds,
                "observation timestamp_seconds",
            ),
        )
        object.__setattr__(self, "frame_id", _frame_id(self.frame_id))
        if not isinstance(self.position, Vector3):
            raise ObservationValidationError(
                "observation position must be Vector3"
            )
        if not isinstance(self.position_variance, DiagonalCovariance3):
            raise ObservationValidationError(
                "observation position_variance must be DiagonalCovariance3"
            )


@dataclass(frozen=True)
class MotionEstimate:
    timestamp_seconds: float
    frame_id: str
    position: Vector3
    velocity: Vector3
    covariance: StateCovariance3
    sample_interval_seconds: float

    def __post_init__(self):
        object.__setattr__(
            self,
            "timestamp_seconds",
            _nonnegative_number(
                self.timestamp_seconds,
                "estimate timestamp_seconds",
            ),
        )
        object.__setattr__(self, "frame_id", _frame_id(self.frame_id))
        object.__setattr__(
            self,
            "sample_interval_seconds",
            _positive_number(
                self.sample_interval_seconds,
                "estimate sample_interval_seconds",
            ),
        )
        if not isinstance(self.position, Vector3):
            raise ObservationValidationError(
                "estimate position must be Vector3"
            )
        if not isinstance(self.velocity, Vector3):
            raise ObservationValidationError(
                "estimate velocity must be Vector3"
            )
        if not isinstance(self.covariance, StateCovariance3):
            raise ObservationValidationError(
                "estimate covariance must be StateCovariance3"
            )


@dataclass(frozen=True)
class AcceleratedMotionEstimate(MotionEstimate):
    covariance: AccelerationStateCovariance3
    acceleration: Vector3
    previous_sample_interval_seconds: float

    def __post_init__(self):
        super().__post_init__()
        if not isinstance(self.covariance, AccelerationStateCovariance3):
            raise ObservationValidationError(
                "accelerated estimate covariance must be "
                "AccelerationStateCovariance3"
            )
        if not isinstance(self.acceleration, Vector3):
            raise ObservationValidationError(
                "accelerated estimate acceleration must be Vector3"
            )
        object.__setattr__(
            self,
            "previous_sample_interval_seconds",
            _positive_number(
                self.previous_sample_interval_seconds,
                "estimate previous_sample_interval_seconds",
            ),
        )


@dataclass(frozen=True)
class PredictedMotionState:
    source_timestamp_seconds: float
    timestamp_seconds: float
    frame_id: str
    position: Vector3
    velocity: Vector3
    covariance: StateCovariance3

    def __post_init__(self):
        source_timestamp = _prediction_number(
            self.source_timestamp_seconds,
            "prediction source_timestamp_seconds",
        )
        timestamp = _prediction_number(
            self.timestamp_seconds,
            "prediction timestamp_seconds",
        )
        if timestamp < source_timestamp:
            raise PredictionError(
                "prediction timestamp must not precede the source timestamp"
            )
        object.__setattr__(
            self,
            "source_timestamp_seconds",
            source_timestamp,
        )
        object.__setattr__(self, "timestamp_seconds", timestamp)
        object.__setattr__(
            self,
            "frame_id",
            _prediction_frame_id(self.frame_id),
        )
        if not isinstance(self.position, Vector3):
            raise PredictionError("prediction position must be Vector3")
        if not isinstance(self.velocity, Vector3):
            raise PredictionError("prediction velocity must be Vector3")
        if not isinstance(self.covariance, StateCovariance3):
            raise PredictionError(
                "prediction covariance must be StateCovariance3"
            )

    @property
    def horizon_seconds(self):
        return self.timestamp_seconds - self.source_timestamp_seconds


@dataclass(frozen=True)
class AcceleratedPredictedMotionState(PredictedMotionState):
    covariance: AccelerationStateCovariance3
    acceleration: Vector3

    def __post_init__(self):
        super().__post_init__()
        if not isinstance(self.covariance, AccelerationStateCovariance3):
            raise PredictionError(
                "accelerated prediction covariance must be "
                "AccelerationStateCovariance3"
            )
        if not isinstance(self.acceleration, Vector3):
            raise PredictionError(
                "accelerated prediction acceleration must be Vector3"
            )


_EstimateT = TypeVar(
    "_EstimateT",
    bound=MotionEstimate,
    contravariant=True,
)
_PredictionT = TypeVar(
    "_PredictionT",
    bound=PredictedMotionState,
    covariant=True,
)


class MotionPredictor(Protocol[_EstimateT, _PredictionT]):
    """Predictor advertising a positive finite maximum future horizon."""

    @property
    def maximum_horizon_seconds(self) -> float:
        """Return the current upper bound on the supported future horizon."""
        ...

    def predict(
        self,
        estimate: _EstimateT,
        target_timestamp_seconds: float,
    ) -> _PredictionT:
        """Predict at the request, allowing a tolerated clamp to the source."""
        ...


@dataclass(frozen=True)
class ConstantVelocityEstimatorConfig:
    """Observation-admission timing shared by both deterministic estimators."""

    frame_id: str
    maximum_observation_age_seconds: float
    minimum_sample_interval_seconds: float
    maximum_sample_interval_seconds: float
    maximum_future_skew_seconds: float = 0.0

    def __post_init__(self):
        object.__setattr__(self, "frame_id", _frame_id(self.frame_id))
        object.__setattr__(
            self,
            "maximum_observation_age_seconds",
            _nonnegative_number(
                self.maximum_observation_age_seconds,
                "maximum_observation_age_seconds",
            ),
        )
        minimum_interval = _positive_number(
            self.minimum_sample_interval_seconds,
            "minimum_sample_interval_seconds",
        )
        maximum_interval = _positive_number(
            self.maximum_sample_interval_seconds,
            "maximum_sample_interval_seconds",
        )
        if maximum_interval < minimum_interval:
            raise ObservationValidationError(
                "maximum sample interval must not be less than the minimum"
            )
        object.__setattr__(
            self,
            "minimum_sample_interval_seconds",
            minimum_interval,
        )
        object.__setattr__(
            self,
            "maximum_sample_interval_seconds",
            maximum_interval,
        )
        object.__setattr__(
            self,
            "maximum_future_skew_seconds",
            _nonnegative_number(
                self.maximum_future_skew_seconds,
                "maximum_future_skew_seconds",
            ),
        )


class EstimatorUpdateStatus(Enum):
    BASELINE_ACCEPTED = "baseline-accepted"
    WARMUP_ACCEPTED = "warmup-accepted"
    BASELINE_RESET = "baseline-reset"
    ESTIMATE_UPDATED = "estimate-updated"


@dataclass(frozen=True)
class EstimatorUpdate:
    status: EstimatorUpdateStatus
    observation: PositionObservation
    estimate: Optional[MotionEstimate] = None

    def __post_init__(self):
        if not isinstance(self.status, EstimatorUpdateStatus):
            raise ObservationValidationError(
                "estimator update status is invalid"
            )
        if not isinstance(self.observation, PositionObservation):
            raise ObservationValidationError(
                "estimator update observation is invalid"
            )
        requires_estimate = self.status is EstimatorUpdateStatus.ESTIMATE_UPDATED
        if requires_estimate and not isinstance(self.estimate, MotionEstimate):
            raise ObservationValidationError(
                "estimator update result does not match the update status"
            )
        if not requires_estimate and self.estimate is not None:
            raise ObservationValidationError(
                "estimator update result does not match the update status"
            )


def _axis_state_covariance(previous_variance, current_variance, interval):
    velocity_variance = (
        previous_variance / interval + current_variance / interval
    ) / interval
    if (
        (previous_variance > 0 or current_variance > 0)
        and velocity_variance == 0
    ):
        raise ObservationValidationError(
            "velocity variance is outside the host numeric range"
        )
    cross_covariance = current_variance / interval
    if current_variance > 0 and cross_covariance == 0:
        raise ObservationValidationError(
            "position-velocity covariance is outside the host numeric range"
        )
    return AxisStateCovariance(
        position_variance=current_variance,
        velocity_variance=velocity_variance,
        position_velocity_covariance=cross_covariance,
    )


def _motion_estimate(previous, current):
    interval = current.timestamp_seconds - previous.timestamp_seconds
    previous_position = previous.position.components()
    current_position = current.position.components()
    velocity = Vector3(*(
        (current_value - previous_value) / interval
        for previous_value, current_value in zip(
            previous_position,
            current_position,
        )
    ))
    covariance = StateCovariance3(*(
        _axis_state_covariance(
            previous_variance,
            current_variance,
            interval,
        )
        for previous_variance, current_variance in zip(
            previous.position_variance.components(),
            current.position_variance.components(),
        )
    ))
    return MotionEstimate(
        timestamp_seconds=current.timestamp_seconds,
        frame_id=current.frame_id,
        position=current.position,
        velocity=velocity,
        covariance=covariance,
        sample_interval_seconds=interval,
    )


def _validated_estimator_observation(
    config,
    previous,
    previous_receipt,
    observation,
    received_at_seconds,
):
    if not isinstance(observation, PositionObservation):
        raise ObservationValidationError(
            "observation must be PositionObservation"
        )
    if observation.frame_id != config.frame_id:
        raise ObservationValidationError(
            "observation frame_id does not match the estimator frame"
        )
    received_at = _nonnegative_number(
        received_at_seconds,
        "received_at_seconds",
    )
    age = received_at - observation.timestamp_seconds
    age_tolerance = _comparison_tolerance(
        received_at,
        observation.timestamp_seconds,
        config.maximum_observation_age_seconds,
        config.maximum_future_skew_seconds,
    )
    if age < -config.maximum_future_skew_seconds - age_tolerance:
        raise FutureObservationError(
            "observation timestamp exceeds the future-skew bound"
        )
    if age > config.maximum_observation_age_seconds + age_tolerance:
        raise StaleObservationError(
            "observation exceeds the maximum age"
        )
    if previous_receipt is not None:
        receipt_tolerance = _comparison_tolerance(
            received_at,
            previous_receipt,
        )
        if received_at < previous_receipt - receipt_tolerance:
            raise OutOfOrderObservationError(
                "observation receipt timestamps must not move backward"
            )
        if received_at < previous_receipt:
            received_at = previous_receipt
    if previous is None:
        return received_at, EstimatorUpdateStatus.BASELINE_ACCEPTED

    interval = observation.timestamp_seconds - previous.timestamp_seconds
    if interval <= 0:
        raise OutOfOrderObservationError(
            "observation timestamps must advance strictly"
        )
    interval_tolerance = _comparison_tolerance(
        observation.timestamp_seconds,
        previous.timestamp_seconds,
        config.minimum_sample_interval_seconds,
        config.maximum_sample_interval_seconds,
    )
    if interval < config.minimum_sample_interval_seconds - interval_tolerance:
        raise ObservationValidationError(
            "observation interval is below the configured minimum"
        )
    if interval > config.maximum_sample_interval_seconds + interval_tolerance:
        return received_at, EstimatorUpdateStatus.BASELINE_RESET
    return received_at, None


class ConstantVelocityEstimator:
    """Single-owner, bounded two-observation constant-velocity estimator."""

    def __init__(self, config):
        if not isinstance(config, ConstantVelocityEstimatorConfig):
            raise ObservationValidationError(
                "estimator config must be ConstantVelocityEstimatorConfig"
            )
        self._config = config
        self._last_observation = None
        self._last_received_at_seconds = None
        self._estimate = None

    @property
    def config(self):
        return self._config

    @property
    def last_observation(self):
        return self._last_observation

    @property
    def estimate(self):
        return self._estimate

    @property
    def last_received_at_seconds(self):
        return self._last_received_at_seconds

    def reset(self):
        self._last_observation = None
        self._last_received_at_seconds = None
        self._estimate = None

    def add_observation(self, observation, received_at_seconds):
        received_at, baseline_status = _validated_estimator_observation(
            self._config,
            self._last_observation,
            self._last_received_at_seconds,
            observation,
            received_at_seconds,
        )
        if baseline_status is not None:
            self._last_observation = observation
            self._last_received_at_seconds = received_at
            self._estimate = None
            return EstimatorUpdate(
                baseline_status,
                observation,
            )

        estimate = _motion_estimate(self._last_observation, observation)
        self._last_observation = observation
        self._last_received_at_seconds = received_at
        self._estimate = estimate
        return EstimatorUpdate(
            EstimatorUpdateStatus.ESTIMATE_UPDATED,
            observation,
            estimate,
        )


def _finite_sum(values, field_name):
    terms = tuple(_finite_number(value, field_name) for value in values)
    try:
        total = math.fsum(terms)
    except (OverflowError, ValueError) as exc:
        raise ObservationValidationError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    return _finite_number(total, field_name)


def _finite_nonzero_product(values, field_name):
    """Return an exact-zero product and reject nonzero range loss."""

    factors = tuple(_finite_number(value, field_name) for value in values)
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
        raise ObservationValidationError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    product = _finite_number(product, field_name)
    if product == 0:
        raise ObservationValidationError(
            f"{field_name} is outside the host numeric range"
        )
    return product


def _finite_ratio(numerator, denominator, field_name):
    numerator = _finite_number(numerator, field_name)
    denominator = _finite_number(denominator, field_name)
    if denominator == 0:
        raise ObservationValidationError(f"{field_name} divisor must be nonzero")
    result = _finite_number(numerator / denominator, field_name)
    if numerator != 0 and result == 0:
        raise ObservationValidationError(
            f"{field_name} is outside the host numeric range"
        )
    return result


def _linear_variance(coefficients, variances, field_name):
    terms = []
    for coefficient, variance in zip(coefficients, variances):
        term = _finite_nonzero_product(
            (coefficient, coefficient, variance),
            field_name,
        )
        terms.append(term)
    return _finite_sum(terms, field_name)


def _linear_covariance(
    first_coefficients,
    second_coefficients,
    variances,
    field_name,
):
    terms = []
    for first, second, variance in zip(
        first_coefficients,
        second_coefficients,
        variances,
    ):
        term = _finite_nonzero_product(
            (first, second, variance),
            field_name,
        )
        terms.append(term)
    return _finite_sum(terms, field_name)


def _acceleration_estimate_coefficients(previous_interval, current_interval):
    span = _finite_sum(
        (previous_interval, current_interval),
        "acceleration sample span",
    )
    previous_inverse = _finite_number(
        1.0 / previous_interval,
        "previous sample interval reciprocal",
    )
    current_inverse = _finite_number(
        1.0 / current_interval,
        "current sample interval reciprocal",
    )
    acceleration_scale = _finite_number(
        2.0 / span,
        "acceleration sample scale",
    )
    acceleration_coefficients = (
        _finite_nonzero_product(
            (acceleration_scale, previous_inverse),
            "acceleration coefficient",
        ),
        -_finite_nonzero_product(
            (
                acceleration_scale,
                _finite_sum(
                    (previous_inverse, current_inverse),
                    "acceleration reciprocal sum",
                ),
            ),
            "acceleration coefficient",
        ),
        _finite_nonzero_product(
            (acceleration_scale, current_inverse),
            "acceleration coefficient",
        ),
    )
    half_current_interval = _finite_nonzero_product(
        (0.5, current_interval),
        "half current sample interval",
    )
    velocity_coefficients = (
        _finite_nonzero_product(
            (half_current_interval, acceleration_coefficients[0]),
            "velocity coefficient",
        ),
        _finite_sum(
            (
                -current_inverse,
                _finite_nonzero_product(
                    (half_current_interval, acceleration_coefficients[1]),
                    "velocity coefficient",
                ),
            ),
            "velocity coefficient",
        ),
        _finite_sum(
            (
                current_inverse,
                _finite_nonzero_product(
                    (half_current_interval, acceleration_coefficients[2]),
                    "velocity coefficient",
                ),
            ),
            "velocity coefficient",
        ),
    )
    return velocity_coefficients, acceleration_coefficients


def _accelerated_axis_state(
    first_position,
    second_position,
    current_position,
    previous_interval,
    current_interval,
):
    previous_velocity = _finite_ratio(
        _finite_sum(
            (second_position, -first_position),
            "previous position delta",
        ),
        previous_interval,
        "previous interval velocity",
    )
    current_velocity = _finite_ratio(
        _finite_sum(
            (current_position, -second_position),
            "current position delta",
        ),
        current_interval,
        "current interval velocity",
    )
    sample_span = _finite_sum(
        (previous_interval, current_interval),
        "acceleration sample span",
    )
    acceleration = _finite_ratio(
        _finite_nonzero_product(
            (
                2.0,
                _finite_sum(
                    (current_velocity, -previous_velocity),
                    "interval velocity delta",
                ),
            ),
            "estimated acceleration",
        ),
        sample_span,
        "estimated acceleration",
    )
    terminal_velocity = _finite_sum(
        (
            current_velocity,
            _finite_nonzero_product(
                (0.5, acceleration, current_interval),
                "estimated terminal velocity",
            ),
        ),
        "estimated terminal velocity",
    )
    return terminal_velocity, acceleration


def _axis_acceleration_covariance(
    variances,
    velocity_coefficients,
    acceleration_coefficients,
):
    position_coefficients = (0.0, 0.0, 1.0)
    return AxisAccelerationStateCovariance(
        position_variance=variances[2],
        velocity_variance=_linear_variance(
            velocity_coefficients,
            variances,
            "estimated velocity variance",
        ),
        position_velocity_covariance=_linear_covariance(
            position_coefficients,
            velocity_coefficients,
            variances,
            "estimated position-velocity covariance",
        ),
        acceleration_variance=_linear_variance(
            acceleration_coefficients,
            variances,
            "estimated acceleration variance",
        ),
        position_acceleration_covariance=_linear_covariance(
            position_coefficients,
            acceleration_coefficients,
            variances,
            "estimated position-acceleration covariance",
        ),
        velocity_acceleration_covariance=_linear_covariance(
            velocity_coefficients,
            acceleration_coefficients,
            variances,
            "estimated velocity-acceleration covariance",
        ),
    )


def _accelerated_motion_estimate(first, second, current):
    previous_interval = second.timestamp_seconds - first.timestamp_seconds
    current_interval = current.timestamp_seconds - second.timestamp_seconds
    velocity_coefficients, acceleration_coefficients = (
        _acceleration_estimate_coefficients(
            previous_interval,
            current_interval,
        )
    )
    states = tuple(
        _accelerated_axis_state(
            first_position,
            second_position,
            current_position,
            previous_interval,
            current_interval,
        )
        for first_position, second_position, current_position in zip(
            first.position.components(),
            second.position.components(),
            current.position.components(),
        )
    )
    covariance = AccelerationStateCovariance3(*(
        _axis_acceleration_covariance(
            variances,
            velocity_coefficients,
            acceleration_coefficients,
        )
        for variances in zip(
            first.position_variance.components(),
            second.position_variance.components(),
            current.position_variance.components(),
        )
    ))
    return AcceleratedMotionEstimate(
        timestamp_seconds=current.timestamp_seconds,
        frame_id=current.frame_id,
        position=current.position,
        velocity=Vector3(*(state[0] for state in states)),
        covariance=covariance,
        sample_interval_seconds=current_interval,
        acceleration=Vector3(*(state[1] for state in states)),
        previous_sample_interval_seconds=previous_interval,
    )


class ConstantAccelerationEstimator:
    """Single-owner, bounded three-observation acceleration estimator."""

    def __init__(self, config):
        if not isinstance(config, ConstantVelocityEstimatorConfig):
            raise ObservationValidationError(
                "estimator config must be ConstantVelocityEstimatorConfig, "
                "the shared observation-admission config"
            )
        self._config = config
        self._observations = ()
        self._last_received_at_seconds = None
        self._estimate = None

    @property
    def config(self):
        return self._config

    @property
    def last_observation(self):
        if not self._observations:
            return None
        return self._observations[-1]

    @property
    def estimate(self):
        return self._estimate

    @property
    def last_received_at_seconds(self):
        return self._last_received_at_seconds

    def reset(self):
        self._observations = ()
        self._last_received_at_seconds = None
        self._estimate = None

    def add_observation(self, observation, received_at_seconds):
        received_at, baseline_status = _validated_estimator_observation(
            self._config,
            self.last_observation,
            self._last_received_at_seconds,
            observation,
            received_at_seconds,
        )
        if baseline_status is not None:
            self._observations = (observation,)
            self._last_received_at_seconds = received_at
            self._estimate = None
            return EstimatorUpdate(baseline_status, observation)

        candidate_observations = (*self._observations, observation)
        if len(candidate_observations) < 3:
            self._observations = candidate_observations
            self._last_received_at_seconds = received_at
            self._estimate = None
            return EstimatorUpdate(
                EstimatorUpdateStatus.WARMUP_ACCEPTED,
                observation,
            )

        estimate = _accelerated_motion_estimate(*candidate_observations)
        self._observations = candidate_observations[-2:]
        self._last_received_at_seconds = received_at
        self._estimate = estimate
        return EstimatorUpdate(
            EstimatorUpdateStatus.ESTIMATE_UPDATED,
            observation,
            estimate,
        )


def _validated_prediction_request(
    source_timestamp_seconds,
    target_timestamp_seconds,
    maximum_horizon_seconds,
):
    target_timestamp = _prediction_number(
        target_timestamp_seconds,
        "target_timestamp_seconds",
    )
    horizon = target_timestamp - source_timestamp_seconds
    source_tolerance = prediction_timestamp_tolerance(
        source_timestamp_seconds,
        target_timestamp,
    )
    if horizon < -source_tolerance:
        raise PredictionError(
            "target timestamp must not precede the estimate"
        )
    if horizon < 0:
        horizon = 0.0
        target_timestamp = source_timestamp_seconds
    horizon_tolerance = _comparison_tolerance(
        target_timestamp,
        source_timestamp_seconds,
        maximum_horizon_seconds,
    )
    if horizon > maximum_horizon_seconds + horizon_tolerance:
        raise PredictionError(
            "prediction horizon exceeds the configured maximum"
        )
    return target_timestamp, horizon


@dataclass(frozen=True)
class ConstantVelocityPredictor:
    maximum_horizon_seconds: float
    process_position_variance_per_second: DiagonalCovariance3

    def __post_init__(self):
        object.__setattr__(
            self,
            "maximum_horizon_seconds",
            _prediction_number(
                self.maximum_horizon_seconds,
                "maximum_horizon_seconds",
                positive=True,
            ),
        )
        if not isinstance(
            self.process_position_variance_per_second,
            DiagonalCovariance3,
        ):
            raise PredictionError(
                "process noise must be DiagonalCovariance3"
            )

    def predict(self, estimate, target_timestamp_seconds):
        if (
            not isinstance(estimate, MotionEstimate)
            or isinstance(estimate, AcceleratedMotionEstimate)
        ):
            raise PredictionError(
                "estimate must be a non-accelerated MotionEstimate"
            )
        target_timestamp, horizon = _validated_prediction_request(
            estimate.timestamp_seconds,
            target_timestamp_seconds,
            self.maximum_horizon_seconds,
        )

        try:
            position = Vector3(*(
                current + velocity * horizon
                for current, velocity in zip(
                    estimate.position.components(),
                    estimate.velocity.components(),
                )
            ))
            predicted_axes = []
            for axis, process_variance in zip(
                estimate.covariance.axes(),
                self.process_position_variance_per_second.components(),
            ):
                position_variance_terms = tuple(
                    _finite_number(value, "predicted position variance term")
                    for value in (
                        axis.position_variance,
                        2.0 * horizon * axis.position_velocity_covariance,
                        horizon * horizon * axis.velocity_variance,
                        horizon * process_variance,
                    )
                )
                try:
                    position_variance = math.fsum(position_variance_terms)
                except (OverflowError, ValueError) as exc:
                    raise ObservationValidationError(
                        "predicted position variance is outside the host range"
                    ) from exc
                variance_tolerance = _comparison_tolerance(
                    *position_variance_terms
                )
                if position_variance < -variance_tolerance:
                    raise ObservationValidationError(
                        "predicted position variance must be non-negative"
                    )
                position_variance = max(0.0, position_variance)
                try:
                    cross_covariance = math.fsum((
                        axis.position_velocity_covariance,
                        horizon * axis.velocity_variance,
                    ))
                except (OverflowError, ValueError) as exc:
                    raise ObservationValidationError(
                        "predicted cross covariance is outside the host range"
                    ) from exc
                predicted_axes.append(AxisStateCovariance(
                    position_variance=position_variance,
                    velocity_variance=axis.velocity_variance,
                    position_velocity_covariance=cross_covariance,
                ))

            return PredictedMotionState(
                source_timestamp_seconds=estimate.timestamp_seconds,
                timestamp_seconds=target_timestamp,
                frame_id=estimate.frame_id,
                position=position,
                velocity=estimate.velocity,
                covariance=StateCovariance3(*predicted_axes),
            )
        except ObservationValidationError as exc:
            raise PredictionError(
                f"predicted state cannot be represented: {exc}"
            ) from exc


def _nonnegative_covariance_sum(values, field_name):
    terms = tuple(_finite_number(value, field_name) for value in values)
    total = _finite_sum(terms, field_name)
    tolerance = _comparison_tolerance(*terms)
    if total < -tolerance:
        raise ObservationValidationError(
            f"{field_name} must be non-negative"
        )
    return max(0.0, total)


def _predicted_acceleration_covariance(
    axis,
    process_variance,
    horizon,
):
    horizon_squared = _finite_nonzero_product(
        (horizon, horizon),
        "prediction horizon power",
    )
    horizon_cubed = _finite_nonzero_product(
        (horizon_squared, horizon),
        "prediction horizon power",
    )
    horizon_fourth = _finite_nonzero_product(
        (horizon_cubed, horizon),
        "prediction horizon power",
    )
    horizon_fifth = _finite_nonzero_product(
        (horizon_fourth, horizon),
        "prediction horizon power",
    )
    return AxisAccelerationStateCovariance(
        position_variance=_nonnegative_covariance_sum(
            (
                axis.position_variance,
                _finite_nonzero_product(
                    (
                        2.0,
                        horizon,
                        axis.position_velocity_covariance,
                    ),
                    "predicted position variance term",
                ),
                _finite_nonzero_product(
                    (horizon_squared, axis.velocity_variance),
                    "predicted position variance term",
                ),
                _finite_nonzero_product(
                    (
                        horizon_squared,
                        axis.position_acceleration_covariance,
                    ),
                    "predicted position variance term",
                ),
                _finite_nonzero_product(
                    (
                        horizon_cubed,
                        axis.velocity_acceleration_covariance,
                    ),
                    "predicted position variance term",
                ),
                _finite_nonzero_product(
                    (0.25, horizon_fourth, axis.acceleration_variance),
                    "predicted position variance term",
                ),
                _finite_nonzero_product(
                    (process_variance, horizon_fifth, 1.0 / 20.0),
                    "predicted position variance term",
                ),
            ),
            "predicted position variance",
        ),
        velocity_variance=_nonnegative_covariance_sum(
            (
                axis.velocity_variance,
                _finite_nonzero_product(
                    (
                        2.0,
                        horizon,
                        axis.velocity_acceleration_covariance,
                    ),
                    "predicted velocity variance term",
                ),
                _finite_nonzero_product(
                    (horizon_squared, axis.acceleration_variance),
                    "predicted velocity variance term",
                ),
                _finite_nonzero_product(
                    (process_variance, horizon_cubed, 1.0 / 3.0),
                    "predicted velocity variance term",
                ),
            ),
            "predicted velocity variance",
        ),
        position_velocity_covariance=_finite_sum(
            (
                axis.position_velocity_covariance,
                _finite_nonzero_product(
                    (
                        horizon,
                        axis.position_acceleration_covariance,
                    ),
                    "predicted position-velocity covariance term",
                ),
                _finite_nonzero_product(
                    (horizon, axis.velocity_variance),
                    "predicted position-velocity covariance term",
                ),
                _finite_nonzero_product(
                    (
                        1.5,
                        horizon_squared,
                        axis.velocity_acceleration_covariance,
                    ),
                    "predicted position-velocity covariance term",
                ),
                _finite_nonzero_product(
                    (0.5, horizon_cubed, axis.acceleration_variance),
                    "predicted position-velocity covariance term",
                ),
                _finite_nonzero_product(
                    (process_variance, horizon_fourth, 1.0 / 8.0),
                    "predicted position-velocity covariance term",
                ),
            ),
            "predicted position-velocity covariance",
        ),
        acceleration_variance=_nonnegative_covariance_sum(
            (
                axis.acceleration_variance,
                _finite_nonzero_product(
                    (process_variance, horizon),
                    "predicted acceleration variance term",
                ),
            ),
            "predicted acceleration variance",
        ),
        position_acceleration_covariance=_finite_sum(
            (
                axis.position_acceleration_covariance,
                _finite_nonzero_product(
                    (
                        horizon,
                        axis.velocity_acceleration_covariance,
                    ),
                    "predicted position-acceleration covariance term",
                ),
                _finite_nonzero_product(
                    (0.5, horizon_squared, axis.acceleration_variance),
                    "predicted position-acceleration covariance term",
                ),
                _finite_nonzero_product(
                    (process_variance, horizon_cubed, 1.0 / 6.0),
                    "predicted position-acceleration covariance term",
                ),
            ),
            "predicted position-acceleration covariance",
        ),
        velocity_acceleration_covariance=_finite_sum(
            (
                axis.velocity_acceleration_covariance,
                _finite_nonzero_product(
                    (horizon, axis.acceleration_variance),
                    "predicted velocity-acceleration covariance term",
                ),
                _finite_nonzero_product(
                    (process_variance, horizon_squared, 0.5),
                    "predicted velocity-acceleration covariance term",
                ),
            ),
            "predicted velocity-acceleration covariance",
        ),
    )


@dataclass(frozen=True)
class ConstantAccelerationPredictor:
    """Propagate acceleration with white-jerk density in mm^2/s^5."""

    maximum_horizon_seconds: float
    process_acceleration_variance_per_second: DiagonalCovariance3

    def __post_init__(self):
        object.__setattr__(
            self,
            "maximum_horizon_seconds",
            _prediction_number(
                self.maximum_horizon_seconds,
                "maximum_horizon_seconds",
                positive=True,
            ),
        )
        if not isinstance(
            self.process_acceleration_variance_per_second,
            DiagonalCovariance3,
        ):
            raise PredictionError(
                "process noise must be DiagonalCovariance3"
            )

    def predict(self, estimate, target_timestamp_seconds):
        if not isinstance(estimate, AcceleratedMotionEstimate):
            raise PredictionError(
                "estimate must be AcceleratedMotionEstimate"
            )
        target_timestamp, horizon = _validated_prediction_request(
            estimate.timestamp_seconds,
            target_timestamp_seconds,
            self.maximum_horizon_seconds,
        )
        try:
            horizon_squared = _finite_nonzero_product(
                (horizon, horizon),
                "prediction horizon power",
            )
            position = Vector3(*(
                _finite_sum(
                    (
                        current,
                        _finite_nonzero_product(
                            (velocity, horizon),
                            "predicted position term",
                        ),
                        _finite_nonzero_product(
                            (0.5, acceleration, horizon_squared),
                            "predicted position term",
                        ),
                    ),
                    "predicted position",
                )
                for current, velocity, acceleration in zip(
                    estimate.position.components(),
                    estimate.velocity.components(),
                    estimate.acceleration.components(),
                )
            ))
            velocity = Vector3(*(
                _finite_sum(
                    (
                        current,
                        _finite_nonzero_product(
                            (acceleration, horizon),
                            "predicted velocity term",
                        ),
                    ),
                    "predicted velocity",
                )
                for current, acceleration in zip(
                    estimate.velocity.components(),
                    estimate.acceleration.components(),
                )
            ))
            covariance = AccelerationStateCovariance3(*(
                _predicted_acceleration_covariance(
                    axis,
                    process_variance,
                    horizon,
                )
                for axis, process_variance in zip(
                    estimate.covariance.axes(),
                    self.process_acceleration_variance_per_second.components(),
                )
            ))
            return AcceleratedPredictedMotionState(
                source_timestamp_seconds=estimate.timestamp_seconds,
                timestamp_seconds=target_timestamp,
                frame_id=estimate.frame_id,
                position=position,
                velocity=velocity,
                covariance=covariance,
                acceleration=estimate.acceleration,
            )
        except ObservationValidationError as exc:
            raise PredictionError(
                f"predicted state cannot be represented: {exc}"
            ) from exc


@dataclass(frozen=True)
class ReplayObservation:
    observation: PositionObservation
    received_at_seconds: float

    def __post_init__(self):
        if not isinstance(self.observation, PositionObservation):
            raise ObservationValidationError(
                "replay observation must contain PositionObservation"
            )
        received_at = _nonnegative_number(
            self.received_at_seconds,
            "replay received_at_seconds",
        )
        object.__setattr__(self, "received_at_seconds", received_at)


@dataclass(frozen=True)
class ObservationReplay:
    samples: Tuple[ReplayObservation, ...]

    def __post_init__(self):
        samples = _fixed_items_bounded(
            self.samples,
            OBSERVATION_REPLAY_MAXIMUM_RECORDS,
            "replay samples",
        )
        if not samples:
            raise ObservationValidationError(
                "replay must contain at least one observation"
            )
        if any(not isinstance(sample, ReplayObservation) for sample in samples):
            raise ObservationValidationError(
                "replay samples must be ReplayObservation values"
            )
        frame_id = samples[0].observation.frame_id
        previous_timestamp = -1.0
        previous_receipt = -1.0
        for sample in samples:
            timestamp = sample.observation.timestamp_seconds
            if sample.observation.frame_id != frame_id:
                raise ObservationValidationError(
                    "replay observations must use one frame_id"
                )
            if timestamp <= previous_timestamp:
                raise OutOfOrderObservationError(
                    "replay observation timestamps must advance strictly"
                )
            if sample.received_at_seconds < previous_receipt:
                raise OutOfOrderObservationError(
                    "replay receipt timestamps must not move backward"
                )
            previous_timestamp = timestamp
            previous_receipt = sample.received_at_seconds
        object.__setattr__(self, "samples", samples)

    @property
    def frame_id(self):
        return self.samples[0].observation.frame_id

    def _run_estimator(self, estimator):
        return tuple(
            estimator.add_observation(
                sample.observation,
                sample.received_at_seconds,
            )
            for sample in self.samples
        )

    def run(self, estimator_config):
        if not isinstance(
            estimator_config,
            ConstantVelocityEstimatorConfig,
        ):
            raise ObservationValidationError(
                "replay config must be ConstantVelocityEstimatorConfig"
            )
        return self._run_estimator(
            ConstantVelocityEstimator(estimator_config)
        )

    def run_constant_acceleration(self, estimator_config):
        if not isinstance(
            estimator_config,
            ConstantVelocityEstimatorConfig,
        ):
            raise ObservationValidationError(
                "replay config must be ConstantVelocityEstimatorConfig, "
                "the shared observation-admission config"
            )
        return self._run_estimator(
            ConstantAccelerationEstimator(estimator_config)
        )


def _fixed_items_bounded(values, maximum_count, field_name):
    if isinstance(values, (str, bytes, bytearray)):
        raise ObservationValidationError(f"{field_name} must be a sequence")
    try:
        iterator = iter(values)
    except Exception as exc:
        raise ObservationValidationError(
            f"{field_name} must be a sequence"
        ) from exc
    items = []
    for _ in range(maximum_count + 1):
        try:
            items.append(next(iterator))
        except StopIteration:
            break
        except Exception as exc:
            raise ObservationValidationError(
                f"{field_name} iteration failed"
            ) from exc
    if len(items) > maximum_count:
        raise ObservationValidationError(
            f"{field_name} exceeds the maximum record count"
        )
    return tuple(items)


def _bounded_limit(value, maximum, field_name):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ReplayFormatError(f"{field_name} must be an integer")
    value = int(value)
    if not 1 <= value <= maximum:
        raise ReplayFormatError(
            f"{field_name} must be between 1 and {maximum}"
        )
    return value


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ReplayFormatError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value):
    raise ReplayFormatError(f"non-finite JSON number: {value}")


def _parse_json_line(line, line_number):
    try:
        parsed = json.loads(
            line,
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
        )
    except ReplayFormatError as exc:
        raise ReplayFormatError(f"line {line_number}: {exc}") from exc
    except (ValueError, RecursionError) as exc:
        raise ReplayFormatError(
            f"line {line_number}: invalid JSON"
        ) from exc
    if not isinstance(parsed, dict):
        raise ReplayFormatError(
            f"line {line_number}: record must be a JSON object"
        )
    return parsed


def _replay_lines(payload):
    if not isinstance(payload, (bytes, bytearray)):
        raise ReplayFormatError("replay payload must be bytes")
    payload = bytes(payload)
    if not payload:
        raise ReplayFormatError("replay payload must not be empty")
    if b"\x00" in payload:
        raise ReplayFormatError("replay payload must not contain NUL bytes")
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ReplayFormatError("replay payload must be valid UTF-8") from exc
    raw_lines = text.split("\n")
    if raw_lines and raw_lines[-1] == "":
        raw_lines.pop()
    lines = []
    for line_number, raw_line in enumerate(raw_lines, start=1):
        if raw_line.endswith("\r"):
            raw_line = raw_line[:-1]
        if not raw_line or "\r" in raw_line:
            raise ReplayFormatError(
                f"line {line_number}: blank or malformed line"
            )
        if len(raw_line.encode("utf-8")) > OBSERVATION_REPLAY_MAXIMUM_LINE_BYTES:
            raise ReplayFormatError(
                f"line {line_number}: record exceeds the line limit"
            )
        lines.append(raw_line)
    return tuple(lines)


def decode_observation_replay(
    payload,
    maximum_bytes=OBSERVATION_REPLAY_MAXIMUM_BYTES,
    maximum_records=OBSERVATION_REPLAY_MAXIMUM_RECORDS,
):
    """Decode a bounded, strict UTF-8 observation replay payload."""

    byte_limit = _bounded_limit(
        maximum_bytes,
        OBSERVATION_REPLAY_MAXIMUM_BYTES,
        "maximum_bytes",
    )
    record_limit = _bounded_limit(
        maximum_records,
        OBSERVATION_REPLAY_MAXIMUM_RECORDS,
        "maximum_records",
    )
    if not isinstance(payload, (bytes, bytearray)):
        raise ReplayFormatError("replay payload must be bytes")
    if len(payload) > byte_limit:
        raise ReplayFormatError("replay payload exceeds the byte limit")
    lines = _replay_lines(payload)
    if not lines:
        raise ReplayFormatError("replay payload must contain a header")
    if len(lines) - 1 > record_limit:
        raise ReplayFormatError("replay payload exceeds the record limit")

    header = _parse_json_line(lines[0], 1)
    expected_header = {
        "position_unit": OBSERVATION_REPLAY_POSITION_UNIT,
        "schema": OBSERVATION_REPLAY_SCHEMA,
        "timebase": OBSERVATION_REPLAY_TIMEBASE,
    }
    if header != expected_header:
        raise ReplayFormatError("line 1: unsupported replay header")

    samples = []
    expected_keys = {
        "timestamp_seconds",
        "received_at_seconds",
        "frame_id",
        "position",
        "position_variance",
    }
    for line_number, line in enumerate(lines[1:], start=2):
        record = _parse_json_line(line, line_number)
        if set(record) != expected_keys:
            raise ReplayFormatError(
                f"line {line_number}: observation fields do not match the schema"
            )
        try:
            observation = PositionObservation(
                timestamp_seconds=record["timestamp_seconds"],
                frame_id=record["frame_id"],
                position=Vector3.from_sequence(
                    record["position"],
                    "position",
                ),
                position_variance=DiagonalCovariance3.from_sequence(
                    record["position_variance"],
                    "position_variance",
                ),
            )
            samples.append(ReplayObservation(
                observation=observation,
                received_at_seconds=record["received_at_seconds"],
            ))
        except ObservationValidationError as exc:
            raise ReplayFormatError(f"line {line_number}: {exc}") from exc
    try:
        return ObservationReplay(tuple(samples))
    except ObservationValidationError as exc:
        raise ReplayFormatError(f"replay sequence is invalid: {exc}") from exc


def encode_observation_replay(replay):
    """Encode a replay using the canonical versioned JSONL schema."""

    if not isinstance(replay, ObservationReplay):
        raise ReplayFormatError("replay must be ObservationReplay")
    records = [{
        "position_unit": OBSERVATION_REPLAY_POSITION_UNIT,
        "schema": OBSERVATION_REPLAY_SCHEMA,
        "timebase": OBSERVATION_REPLAY_TIMEBASE,
    }]
    for sample in replay.samples:
        observation = sample.observation
        records.append({
            "frame_id": observation.frame_id,
            "position": list(observation.position.components()),
            "position_variance": list(
                observation.position_variance.components()
            ),
            "received_at_seconds": sample.received_at_seconds,
            "timestamp_seconds": observation.timestamp_seconds,
        })
    lines = [
        json.dumps(
            record,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        for record in records
    ]
    if any(
        len(line.encode("utf-8")) > OBSERVATION_REPLAY_MAXIMUM_LINE_BYTES
        for line in lines
    ):
        raise ReplayFormatError("encoded replay record exceeds the line limit")
    payload = ("\n".join(lines) + "\n").encode("utf-8")
    if len(payload) > OBSERVATION_REPLAY_MAXIMUM_BYTES:
        raise ReplayFormatError("encoded replay exceeds the byte limit")
    return payload
