"""Deterministic, hardware-free intercept candidate selection."""

import math
import re
from dataclasses import dataclass, field
from decimal import Decimal
from enum import Enum
from numbers import Integral, Real
from typing import Optional, Tuple

from ARrobots.dynamic_motion import (
    ConstantVelocityEstimatorConfig,
    ConstantVelocityPredictor,
    DynamicMotionError,
    EstimatorUpdate,
    EstimatorUpdateStatus,
    MotionEstimate,
    ObservationReplay,
    PredictedMotionState,
    PredictionError,
)


INTERCEPT_MAXIMUM_CANDIDATES = 4096
INTERCEPT_REPLAY_MAXIMUM_CANDIDATE_EVALUATIONS = 100_000
_FRAME_ID_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_.:/-]{0,63}\Z")


class InterceptSelectionError(DynamicMotionError):
    """Intercept configuration, input, or feasibility output is invalid."""


def _finite_number(value, field_name):
    if isinstance(value, bool):
        raise InterceptSelectionError(f"{field_name} must be numeric")
    try:
        if isinstance(value, Decimal):
            if not value.is_finite():
                raise InterceptSelectionError(f"{field_name} must be finite")
            number = float(value)
            if value != 0 and number == 0:
                raise InterceptSelectionError(
                    f"{field_name} is outside the host numeric range"
                )
        elif isinstance(value, (Integral, Real)):
            number = float(value)
        else:
            raise TypeError
    except InterceptSelectionError:
        raise
    except OverflowError as exc:
        raise InterceptSelectionError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    except (TypeError, ValueError) as exc:
        raise InterceptSelectionError(
            f"{field_name} must be numeric"
        ) from exc
    if not math.isfinite(number):
        raise InterceptSelectionError(f"{field_name} must be finite")
    return number


def _nonnegative_number(value, field_name):
    number = _finite_number(value, field_name)
    if number < 0:
        raise InterceptSelectionError(
            f"{field_name} must be non-negative"
        )
    if number == 0:
        return 0.0
    return number


def _positive_number(value, field_name):
    number = _finite_number(value, field_name)
    if number <= 0:
        raise InterceptSelectionError(f"{field_name} must be positive")
    return number


def _comparison_tolerance(*values):
    return 4.0 * max(math.ulp(abs(value)) for value in values)


def _frame_id(value):
    if not isinstance(value, str) or not _FRAME_ID_PATTERN.fullmatch(value):
        raise InterceptSelectionError(
            "frame_id must match [A-Za-z][A-Za-z0-9_.:/-]{0,63}"
        )
    return value


def _bounded_tuple(values, maximum_count, field_name):
    if isinstance(values, (str, bytes, bytearray)):
        raise InterceptSelectionError(f"{field_name} must be a sequence")
    try:
        iterator = iter(values)
    except Exception as exc:
        raise InterceptSelectionError(
            f"{field_name} must be a sequence"
        ) from exc
    items = []
    for _ in range(maximum_count + 1):
        try:
            items.append(next(iterator))
        except StopIteration:
            break
        except Exception as exc:
            raise InterceptSelectionError(
                f"{field_name} iteration failed"
            ) from exc
    if len(items) > maximum_count:
        raise InterceptSelectionError(
            f"{field_name} exceeds the maximum candidate count"
        )
    return tuple(items)


def _candidate_lead_times(minimum_lead, maximum_lead, interval):
    tolerance = _comparison_tolerance(
        minimum_lead,
        maximum_lead,
        interval,
    )
    values = []
    for index in range(INTERCEPT_MAXIMUM_CANDIDATES):
        lead_time = minimum_lead + index * interval
        if lead_time > maximum_lead + tolerance:
            break
        if lead_time > maximum_lead:
            lead_time = maximum_lead
        if values and lead_time <= values[-1]:
            raise InterceptSelectionError(
                "candidate interval cannot advance in the host numeric range"
            )
        values.append(lead_time)
    next_lead_time = (
        minimum_lead + len(values) * interval
    )
    if next_lead_time <= maximum_lead + tolerance:
        raise InterceptSelectionError(
            "lead-time range exceeds the maximum candidate count"
        )
    return tuple(values)


class InterceptFeasibilityStatus(Enum):
    FEASIBLE = "feasible"
    UNREACHABLE = "unreachable"
    JOINT_LIMIT = "joint-limit"
    SINGULARITY = "singularity"
    COLLISION = "collision"
    ARRIVAL_TIME_UNAVAILABLE = "arrival-time-unavailable"


@dataclass(frozen=True)
class InterceptFeasibility:
    """Validated robot-state feasibility decision.

    Arrival time is a duration from the selector evaluation timestamp. Lower
    application-defined risk scores receive priority.
    """

    status: InterceptFeasibilityStatus
    minimum_arrival_time_seconds: Optional[float] = None
    risk_score: Optional[float] = None

    def __post_init__(self):
        if not isinstance(self.status, InterceptFeasibilityStatus):
            raise InterceptSelectionError("feasibility status is invalid")
        if self.status is InterceptFeasibilityStatus.FEASIBLE:
            if self.minimum_arrival_time_seconds is None:
                raise InterceptSelectionError(
                    "feasible result requires minimum arrival time"
                )
            if self.risk_score is None:
                raise InterceptSelectionError(
                    "feasible result requires a risk score"
                )
            object.__setattr__(
                self,
                "minimum_arrival_time_seconds",
                _nonnegative_number(
                    self.minimum_arrival_time_seconds,
                    "minimum_arrival_time_seconds",
                ),
            )
            object.__setattr__(
                self,
                "risk_score",
                _nonnegative_number(self.risk_score, "risk_score"),
            )
            return
        if (
            self.minimum_arrival_time_seconds is not None
            or self.risk_score is not None
        ):
            raise InterceptSelectionError(
                "rejected feasibility result must not carry timing or risk"
            )


class InterceptRejectionReason(Enum):
    UNCERTAINTY_LIMIT = "uncertainty-limit"
    TERMINAL_SPEED_LIMIT = "terminal-speed-limit"
    UNREACHABLE = "unreachable"
    JOINT_LIMIT = "joint-limit"
    SINGULARITY = "singularity"
    COLLISION = "collision"
    ARRIVAL_TIME_UNAVAILABLE = "arrival-time-unavailable"
    INSUFFICIENT_ARRIVAL_MARGIN = "insufficient-arrival-margin"


_FEASIBILITY_REJECTION_REASONS = {
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


@dataclass(frozen=True)
class InterceptCandidateEvaluation:
    prediction: PredictedMotionState
    maximum_position_standard_deviation_mm: float
    terminal_speed_mm_per_second: float
    feasibility: Optional[InterceptFeasibility]
    arrival_margin_seconds: Optional[float]
    rejection_reason: Optional[InterceptRejectionReason]

    def __post_init__(self):
        if not isinstance(self.prediction, PredictedMotionState):
            raise InterceptSelectionError(
                "candidate prediction must be PredictedMotionState"
            )
        object.__setattr__(
            self,
            "maximum_position_standard_deviation_mm",
            _nonnegative_number(
                self.maximum_position_standard_deviation_mm,
                "maximum_position_standard_deviation_mm",
            ),
        )
        object.__setattr__(
            self,
            "terminal_speed_mm_per_second",
            _nonnegative_number(
                self.terminal_speed_mm_per_second,
                "terminal_speed_mm_per_second",
            ),
        )
        if self.arrival_margin_seconds is not None:
            object.__setattr__(
                self,
                "arrival_margin_seconds",
                _finite_number(
                    self.arrival_margin_seconds,
                    "arrival_margin_seconds",
                ),
            )
        if (
            self.rejection_reason is not None
            and not isinstance(
                self.rejection_reason,
                InterceptRejectionReason,
            )
        ):
            raise InterceptSelectionError(
                "candidate rejection reason is invalid"
            )

        if self.feasibility is None:
            if self.arrival_margin_seconds is not None:
                raise InterceptSelectionError(
                    "unevaluated candidate must not carry arrival margin"
                )
            if self.rejection_reason not in (
                InterceptRejectionReason.UNCERTAINTY_LIMIT,
                InterceptRejectionReason.TERMINAL_SPEED_LIMIT,
            ):
                raise InterceptSelectionError(
                    "unevaluated candidate requires a threshold rejection"
                )
            return
        if not isinstance(self.feasibility, InterceptFeasibility):
            raise InterceptSelectionError(
                "candidate feasibility result is invalid"
            )

        if self.feasibility.status is InterceptFeasibilityStatus.FEASIBLE:
            if self.arrival_margin_seconds is None:
                raise InterceptSelectionError(
                    "feasible candidate requires arrival margin"
                )
            if self.rejection_reason not in (
                None,
                InterceptRejectionReason.INSUFFICIENT_ARRIVAL_MARGIN,
            ):
                raise InterceptSelectionError(
                    "feasible candidate has an inconsistent rejection"
                )
            return

        expected_reason = _FEASIBILITY_REJECTION_REASONS[
            self.feasibility.status
        ]
        if self.arrival_margin_seconds is not None:
            raise InterceptSelectionError(
                "rejected candidate must not carry arrival margin"
            )
        if self.rejection_reason is not expected_reason:
            raise InterceptSelectionError(
                "candidate rejection does not match feasibility status"
            )

    @property
    def accepted(self):
        return self.rejection_reason is None


class InterceptSelectionStatus(Enum):
    SELECTED = "selected"
    NO_FEASIBLE_CANDIDATE = "no-feasible-candidate"
    STALE_ESTIMATE = "stale-estimate"
    FUTURE_ESTIMATE = "future-estimate"


@dataclass(frozen=True)
class InterceptSelection:
    status: InterceptSelectionStatus
    evaluated_at_seconds: float
    estimate: MotionEstimate
    candidates: Tuple[InterceptCandidateEvaluation, ...]
    selected_candidate: Optional[InterceptCandidateEvaluation] = None

    def __post_init__(self):
        if not isinstance(self.status, InterceptSelectionStatus):
            raise InterceptSelectionError("selection status is invalid")
        object.__setattr__(
            self,
            "evaluated_at_seconds",
            _nonnegative_number(
                self.evaluated_at_seconds,
                "evaluated_at_seconds",
            ),
        )
        if not isinstance(self.estimate, MotionEstimate):
            raise InterceptSelectionError(
                "selection estimate must be MotionEstimate"
            )
        candidates = _bounded_tuple(
            self.candidates,
            INTERCEPT_MAXIMUM_CANDIDATES,
            "selection candidates",
        )
        if any(
            not isinstance(candidate, InterceptCandidateEvaluation)
            for candidate in candidates
        ):
            raise InterceptSelectionError(
                "selection candidates contain an invalid value"
            )
        object.__setattr__(self, "candidates", candidates)
        previous_timestamp = None
        for candidate in candidates:
            prediction = candidate.prediction
            if (
                prediction.frame_id != self.estimate.frame_id
                or prediction.source_timestamp_seconds
                != self.estimate.timestamp_seconds
            ):
                raise InterceptSelectionError(
                    "candidate prediction does not match the selection estimate"
                )
            timestamp_tolerance = _comparison_tolerance(
                prediction.timestamp_seconds,
                self.evaluated_at_seconds,
            )
            if (
                prediction.timestamp_seconds
                < self.evaluated_at_seconds - timestamp_tolerance
            ):
                raise InterceptSelectionError(
                    "candidate prediction precedes selection evaluation"
                )
            if (
                previous_timestamp is not None
                and prediction.timestamp_seconds <= previous_timestamp
            ):
                raise InterceptSelectionError(
                    "candidate prediction timestamps must advance strictly"
                )
            previous_timestamp = prediction.timestamp_seconds

        if self.status is InterceptSelectionStatus.SELECTED:
            if self.selected_candidate is None:
                raise InterceptSelectionError(
                    "selected result requires a selected candidate"
                )
            if not any(
                candidate is self.selected_candidate
                for candidate in candidates
            ):
                raise InterceptSelectionError(
                    "selected candidate must belong to the result"
                )
            if not self.selected_candidate.accepted:
                raise InterceptSelectionError(
                    "selected candidate must be accepted"
                )
            accepted = tuple(
                (candidate.feasibility.risk_score, index, candidate)
                for index, candidate in enumerate(candidates)
                if candidate.accepted
            )
            expected = min(
                accepted,
                key=lambda value: (
                    value[0],
                    value[2].prediction.timestamp_seconds,
                    value[1],
                ),
            )[2]
            if self.selected_candidate is not expected:
                raise InterceptSelectionError(
                    "selected candidate does not satisfy deterministic ranking"
                )
            return

        if self.selected_candidate is not None:
            raise InterceptSelectionError(
                "non-selected result must not carry a selected candidate"
            )
        if self.status is InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE:
            if not candidates:
                raise InterceptSelectionError(
                    "no-feasible-candidate result requires candidates"
                )
            if any(candidate.accepted for candidate in candidates):
                raise InterceptSelectionError(
                    "no-feasible-candidate result contains an accepted candidate"
                )
            return
        if candidates:
            raise InterceptSelectionError(
                "stale or future estimate result must not carry candidates"
            )
        if (
            self.status is InterceptSelectionStatus.STALE_ESTIMATE
            and self.evaluated_at_seconds <= self.estimate.timestamp_seconds
        ):
            raise InterceptSelectionError(
                "stale result requires an estimate older than evaluation"
            )
        if (
            self.status is InterceptSelectionStatus.FUTURE_ESTIMATE
            and self.evaluated_at_seconds >= self.estimate.timestamp_seconds
        ):
            raise InterceptSelectionError(
                "future result requires an estimate newer than evaluation"
            )

    @property
    def estimate_age_seconds(self):
        return self.evaluated_at_seconds - self.estimate.timestamp_seconds


@dataclass(frozen=True)
class InterceptSelectorConfig:
    frame_id: str
    minimum_lead_time_seconds: float
    maximum_lead_time_seconds: float
    candidate_interval_seconds: float
    maximum_estimate_age_seconds: float
    maximum_future_skew_seconds: float
    maximum_position_standard_deviation_mm: float
    maximum_terminal_speed_mm_per_second: float
    minimum_arrival_margin_seconds: float
    candidate_lead_times: Tuple[float, ...] = field(
        init=False,
        repr=False,
    )

    def __post_init__(self):
        object.__setattr__(self, "frame_id", _frame_id(self.frame_id))
        minimum_lead = _nonnegative_number(
            self.minimum_lead_time_seconds,
            "minimum_lead_time_seconds",
        )
        maximum_lead = _nonnegative_number(
            self.maximum_lead_time_seconds,
            "maximum_lead_time_seconds",
        )
        interval = _positive_number(
            self.candidate_interval_seconds,
            "candidate_interval_seconds",
        )
        maximum_age = _nonnegative_number(
            self.maximum_estimate_age_seconds,
            "maximum_estimate_age_seconds",
        )
        future_skew = _nonnegative_number(
            self.maximum_future_skew_seconds,
            "maximum_future_skew_seconds",
        )
        maximum_uncertainty = _nonnegative_number(
            self.maximum_position_standard_deviation_mm,
            "maximum_position_standard_deviation_mm",
        )
        maximum_speed = _nonnegative_number(
            self.maximum_terminal_speed_mm_per_second,
            "maximum_terminal_speed_mm_per_second",
        )
        minimum_margin = _nonnegative_number(
            self.minimum_arrival_margin_seconds,
            "minimum_arrival_margin_seconds",
        )
        if maximum_lead < minimum_lead:
            raise InterceptSelectionError(
                "maximum lead time must not be less than the minimum"
            )
        tolerance = _comparison_tolerance(minimum_lead, future_skew)
        if minimum_lead < future_skew - tolerance:
            raise InterceptSelectionError(
                "minimum lead time must cover the maximum future skew"
            )
        lead_times = _candidate_lead_times(
            minimum_lead,
            maximum_lead,
            interval,
        )
        object.__setattr__(
            self,
            "minimum_lead_time_seconds",
            minimum_lead,
        )
        object.__setattr__(
            self,
            "maximum_lead_time_seconds",
            maximum_lead,
        )
        object.__setattr__(
            self,
            "candidate_interval_seconds",
            interval,
        )
        object.__setattr__(
            self,
            "maximum_estimate_age_seconds",
            maximum_age,
        )
        object.__setattr__(
            self,
            "maximum_future_skew_seconds",
            future_skew,
        )
        object.__setattr__(
            self,
            "maximum_position_standard_deviation_mm",
            maximum_uncertainty,
        )
        object.__setattr__(
            self,
            "maximum_terminal_speed_mm_per_second",
            maximum_speed,
        )
        object.__setattr__(
            self,
            "minimum_arrival_margin_seconds",
            minimum_margin,
        )
        object.__setattr__(self, "candidate_lead_times", lead_times)


class InterceptSelector:
    """Select a bounded intercept using a deterministic feasibility callback.

    The callback accepts a predicted state and the selector evaluation
    timestamp, then returns InterceptFeasibility without external mutation.
    """

    def __init__(self, config, predictor, feasibility_evaluator):
        if not isinstance(config, InterceptSelectorConfig):
            raise InterceptSelectionError(
                "selector config must be InterceptSelectorConfig"
            )
        if not isinstance(predictor, ConstantVelocityPredictor):
            raise InterceptSelectionError(
                "selector predictor must be ConstantVelocityPredictor"
            )
        if not callable(feasibility_evaluator):
            raise InterceptSelectionError(
                "feasibility evaluator must be callable"
            )
        try:
            required_horizon = math.fsum((
                config.maximum_estimate_age_seconds,
                config.maximum_lead_time_seconds,
            ))
        except (OverflowError, ValueError) as exc:
            raise InterceptSelectionError(
                "selector prediction horizon is outside the host range"
            ) from exc
        if not math.isfinite(required_horizon):
            raise InterceptSelectionError(
                "selector prediction horizon is outside the host range"
            )
        tolerance = _comparison_tolerance(
            required_horizon,
            predictor.maximum_horizon_seconds,
        )
        if required_horizon > predictor.maximum_horizon_seconds + tolerance:
            raise InterceptSelectionError(
                "predictor horizon does not cover estimate age and lead time"
            )
        self._config = config
        self._predictor = predictor
        self._feasibility_evaluator = feasibility_evaluator

    @property
    def config(self):
        return self._config

    @property
    def predictor(self):
        return self._predictor

    def _empty_result(self, status, estimate, evaluated_at):
        return InterceptSelection(
            status=status,
            evaluated_at_seconds=evaluated_at,
            estimate=estimate,
            candidates=(),
        )

    def _predict(self, estimate, target_timestamp, candidate_index):
        try:
            return self._predictor.predict(estimate, target_timestamp)
        except PredictionError as exc:
            raise InterceptSelectionError(
                f"candidate {candidate_index} prediction failed"
            ) from exc

    def _evaluate_feasibility(
        self,
        prediction,
        evaluated_at_seconds,
        candidate_index,
    ):
        try:
            feasibility = self._feasibility_evaluator(
                prediction,
                evaluated_at_seconds,
            )
        except Exception as exc:
            raise InterceptSelectionError(
                f"candidate {candidate_index} feasibility evaluation failed"
            ) from exc
        if not isinstance(feasibility, InterceptFeasibility):
            raise InterceptSelectionError(
                f"candidate {candidate_index} feasibility output is invalid"
            )
        return feasibility

    def select(self, estimate, evaluated_at_seconds):
        if not isinstance(estimate, MotionEstimate):
            raise InterceptSelectionError("estimate must be MotionEstimate")
        if estimate.frame_id != self._config.frame_id:
            raise InterceptSelectionError(
                "estimate frame_id does not match the selector frame"
            )
        evaluated_at = _nonnegative_number(
            evaluated_at_seconds,
            "evaluated_at_seconds",
        )
        age = evaluated_at - estimate.timestamp_seconds
        age_tolerance = _comparison_tolerance(
            evaluated_at,
            estimate.timestamp_seconds,
            self._config.maximum_estimate_age_seconds,
            self._config.maximum_future_skew_seconds,
        )
        if age > self._config.maximum_estimate_age_seconds + age_tolerance:
            return self._empty_result(
                InterceptSelectionStatus.STALE_ESTIMATE,
                estimate,
                evaluated_at,
            )
        if age < -self._config.maximum_future_skew_seconds - age_tolerance:
            return self._empty_result(
                InterceptSelectionStatus.FUTURE_ESTIMATE,
                estimate,
                evaluated_at,
            )

        candidates = []
        accepted = []
        previous_target_timestamp = None
        for candidate_index, lead_time in enumerate(
            self._config.candidate_lead_times
        ):
            try:
                target_timestamp = math.fsum((evaluated_at, lead_time))
            except (OverflowError, ValueError) as exc:
                raise InterceptSelectionError(
                    f"candidate {candidate_index} timestamp is outside the host range"
                ) from exc
            if not math.isfinite(target_timestamp):
                raise InterceptSelectionError(
                    f"candidate {candidate_index} timestamp is outside the host range"
                )
            if (
                previous_target_timestamp is not None
                and target_timestamp <= previous_target_timestamp
            ):
                raise InterceptSelectionError(
                    "candidate timestamps cannot advance in the host numeric range"
                )
            previous_target_timestamp = target_timestamp
            prediction = self._predict(
                estimate,
                target_timestamp,
                candidate_index,
            )
            maximum_deviation = max(
                math.sqrt(variance)
                for variance in prediction.covariance.position_diagonal().components()
            )
            terminal_speed = math.hypot(*prediction.velocity.components())
            if not math.isfinite(terminal_speed):
                raise InterceptSelectionError(
                    f"candidate {candidate_index} terminal speed is outside "
                    "the host range"
                )

            uncertainty_tolerance = _comparison_tolerance(
                maximum_deviation,
                self._config.maximum_position_standard_deviation_mm,
            )
            if (
                maximum_deviation
                > self._config.maximum_position_standard_deviation_mm
                + uncertainty_tolerance
            ):
                candidates.append(InterceptCandidateEvaluation(
                    prediction=prediction,
                    maximum_position_standard_deviation_mm=maximum_deviation,
                    terminal_speed_mm_per_second=terminal_speed,
                    feasibility=None,
                    arrival_margin_seconds=None,
                    rejection_reason=(
                        InterceptRejectionReason.UNCERTAINTY_LIMIT
                    ),
                ))
                continue

            speed_tolerance = _comparison_tolerance(
                terminal_speed,
                self._config.maximum_terminal_speed_mm_per_second,
            )
            if (
                terminal_speed
                > self._config.maximum_terminal_speed_mm_per_second
                + speed_tolerance
            ):
                candidates.append(InterceptCandidateEvaluation(
                    prediction=prediction,
                    maximum_position_standard_deviation_mm=maximum_deviation,
                    terminal_speed_mm_per_second=terminal_speed,
                    feasibility=None,
                    arrival_margin_seconds=None,
                    rejection_reason=(
                        InterceptRejectionReason.TERMINAL_SPEED_LIMIT
                    ),
                ))
                continue

            feasibility = self._evaluate_feasibility(
                prediction,
                evaluated_at,
                candidate_index,
            )
            if feasibility.status is not InterceptFeasibilityStatus.FEASIBLE:
                candidates.append(InterceptCandidateEvaluation(
                    prediction=prediction,
                    maximum_position_standard_deviation_mm=maximum_deviation,
                    terminal_speed_mm_per_second=terminal_speed,
                    feasibility=feasibility,
                    arrival_margin_seconds=None,
                    rejection_reason=_FEASIBILITY_REJECTION_REASONS[
                        feasibility.status
                    ],
                ))
                continue

            available_time = prediction.timestamp_seconds - evaluated_at
            arrival_margin = (
                available_time - feasibility.minimum_arrival_time_seconds
            )
            margin_tolerance = _comparison_tolerance(
                arrival_margin,
                self._config.minimum_arrival_margin_seconds,
            )
            rejection_reason = None
            if (
                arrival_margin
                < self._config.minimum_arrival_margin_seconds
                - margin_tolerance
            ):
                rejection_reason = (
                    InterceptRejectionReason.INSUFFICIENT_ARRIVAL_MARGIN
                )
            candidate = InterceptCandidateEvaluation(
                prediction=prediction,
                maximum_position_standard_deviation_mm=maximum_deviation,
                terminal_speed_mm_per_second=terminal_speed,
                feasibility=feasibility,
                arrival_margin_seconds=arrival_margin,
                rejection_reason=rejection_reason,
            )
            candidates.append(candidate)
            if candidate.accepted:
                accepted.append((
                    feasibility.risk_score,
                    prediction.timestamp_seconds,
                    candidate_index,
                    candidate,
                ))

        candidate_tuple = tuple(candidates)
        if not accepted:
            return InterceptSelection(
                status=InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE,
                evaluated_at_seconds=evaluated_at,
                estimate=estimate,
                candidates=candidate_tuple,
            )
        selected_candidate = min(accepted, key=lambda value: value[:3])[3]
        return InterceptSelection(
            status=InterceptSelectionStatus.SELECTED,
            evaluated_at_seconds=evaluated_at,
            estimate=estimate,
            candidates=candidate_tuple,
            selected_candidate=selected_candidate,
        )


@dataclass(frozen=True)
class ReplayInterceptStep:
    sample_index: int
    received_at_seconds: float
    estimator_update: EstimatorUpdate
    selection: Optional[InterceptSelection]

    def __post_init__(self):
        if isinstance(self.sample_index, bool) or not isinstance(
            self.sample_index,
            Integral,
        ):
            raise InterceptSelectionError("sample_index must be an integer")
        sample_index = int(self.sample_index)
        if sample_index < 0:
            raise InterceptSelectionError(
                "sample_index must be non-negative"
            )
        object.__setattr__(self, "sample_index", sample_index)
        received_at = _nonnegative_number(
            self.received_at_seconds,
            "received_at_seconds",
        )
        object.__setattr__(self, "received_at_seconds", received_at)
        if not isinstance(self.estimator_update, EstimatorUpdate):
            raise InterceptSelectionError(
                "replay step requires EstimatorUpdate"
            )
        has_estimate = (
            self.estimator_update.status
            is EstimatorUpdateStatus.ESTIMATE_UPDATED
        )
        if has_estimate:
            if not isinstance(self.selection, InterceptSelection):
                raise InterceptSelectionError(
                    "estimated replay step requires intercept selection"
                )
            if self.selection.estimate is not self.estimator_update.estimate:
                raise InterceptSelectionError(
                    "replay selection must use the matching estimate"
                )
            if self.selection.evaluated_at_seconds != received_at:
                raise InterceptSelectionError(
                    "replay selection must use the sample receipt timestamp"
                )
            return
        if self.selection is not None:
            raise InterceptSelectionError(
                "baseline replay step must not carry intercept selection"
            )


def select_replay_intercepts(replay, estimator_config, selector):
    """Re-estimate and reselect after every valid replay observation."""

    if not isinstance(replay, ObservationReplay):
        raise InterceptSelectionError("replay must be ObservationReplay")
    if not isinstance(estimator_config, ConstantVelocityEstimatorConfig):
        raise InterceptSelectionError(
            "estimator_config must be ConstantVelocityEstimatorConfig"
        )
    if not isinstance(selector, InterceptSelector):
        raise InterceptSelectionError("selector must be InterceptSelector")
    if estimator_config.frame_id != selector.config.frame_id:
        raise InterceptSelectionError(
            "estimator and selector frames must match"
        )
    if replay.frame_id != estimator_config.frame_id:
        raise InterceptSelectionError(
            "replay and estimator frames must match"
        )

    updates = replay.run(estimator_config)
    estimated_update_count = sum(
        update.status is EstimatorUpdateStatus.ESTIMATE_UPDATED
        for update in updates
    )
    candidate_evaluation_count = (
        estimated_update_count * len(selector.config.candidate_lead_times)
    )
    if (
        candidate_evaluation_count
        > INTERCEPT_REPLAY_MAXIMUM_CANDIDATE_EVALUATIONS
    ):
        raise InterceptSelectionError(
            "replay exceeds the maximum candidate evaluation count"
        )
    steps = []
    for sample_index, (sample, update) in enumerate(zip(replay.samples, updates)):
        selection = None
        if update.status is EstimatorUpdateStatus.ESTIMATE_UPDATED:
            selection = selector.select(
                update.estimate,
                sample.received_at_seconds,
            )
        steps.append(ReplayInterceptStep(
            sample_index=sample_index,
            received_at_seconds=sample.received_at_seconds,
            estimator_update=update,
            selection=selection,
        ))
    return tuple(steps)
