"""Hardware-free feedback coordination for desired trajectory replacement."""

import copy
import math
from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from numbers import Integral, Real
from typing import Optional, Tuple, Union

from ARrobots.dynamic_motion import (
    ConstantAccelerationEstimator,
    ConstantVelocityEstimatorConfig,
    DynamicMotionError,
    EstimatorUpdate,
    EstimatorUpdateStatus,
    OBSERVATION_REPLAY_MAXIMUM_RECORDS,
    ObservationReplay,
)
from ARrobots.interception import (
    INTERCEPT_REPLAY_MAXIMUM_CANDIDATE_EVALUATIONS,
    InterceptSelection,
    InterceptSelectionStatus,
    InterceptSelector,
    InterceptSelectorConfig,
)
from ARrobots.trajectory_timing import (
    JerkLimitedJointTrajectory,
    JointBoundaryState,
    JointKinematicLimits,
    QuinticJointTrajectory,
    SynchronizedJointTrajectory,
    SynchronizedQuinticTrajectory,
    TRAJECTORY_MAXIMUM_AXES,
    replan_synchronized_quintic_trajectory,
)


REPLAN_FAULT_DETAIL_MAXIMUM_CHARACTERS = 1024
REPLAN_TIMESTAMP_COMPARISON_ULPS = 4.0


class FeedbackReplanningError(DynamicMotionError):
    """A feedback-replanning boundary or coordinator state is invalid."""


def _finite_number(value, field_name):
    if isinstance(value, bool):
        raise FeedbackReplanningError(f"{field_name} must be numeric")
    try:
        if isinstance(value, Decimal):
            if not value.is_finite():
                raise FeedbackReplanningError(
                    f"{field_name} must be finite"
                )
            number = float(value)
            if value != 0 and number == 0:
                raise FeedbackReplanningError(
                    f"{field_name} is outside the host numeric range"
                )
        elif isinstance(value, (Integral, Real)):
            number = float(value)
        else:
            raise TypeError
    except FeedbackReplanningError:
        raise
    except OverflowError as exc:
        raise FeedbackReplanningError(
            f"{field_name} is outside the host numeric range"
        ) from exc
    except (TypeError, ValueError) as exc:
        raise FeedbackReplanningError(
            f"{field_name} must be numeric"
        ) from exc
    if not math.isfinite(number):
        raise FeedbackReplanningError(f"{field_name} must be finite")
    return number


def _nonnegative_number(value, field_name):
    number = _finite_number(value, field_name)
    if number < 0:
        raise FeedbackReplanningError(
            f"{field_name} must be non-negative"
        )
    if number == 0:
        return 0.0
    return number


def _positive_number(value, field_name):
    number = _finite_number(value, field_name)
    if number <= 0:
        raise FeedbackReplanningError(f"{field_name} must be positive")
    return number


def _comparison_tolerance(*values):
    # Event ordering shares the estimator receipt-time comparison boundary.
    return REPLAN_TIMESTAMP_COMPARISON_ULPS * max(
        math.ulp(abs(value)) for value in values
    )


def _bounded_joint_states(values, field_name):
    if type(values) is not tuple:
        raise FeedbackReplanningError(f"{field_name} must be a built-in tuple")
    if not values:
        raise FeedbackReplanningError(
            f"{field_name} must contain at least one state"
        )
    if len(values) > TRAJECTORY_MAXIMUM_AXES:
        raise FeedbackReplanningError(
            f"{field_name} exceeds the supported axis count"
        )
    if any(type(value) is not JointBoundaryState for value in values):
        raise FeedbackReplanningError(
            f"{field_name} must contain built-in joint boundary states"
        )
    return values


def _nonnegative_integer(value, field_name):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise FeedbackReplanningError(f"{field_name} must be an integer")
    integer = int(value)
    if integer < 0:
        raise FeedbackReplanningError(
            f"{field_name} must be non-negative"
        )
    return integer


def _validated_active_trajectory(trajectory):
    trajectory_type = type(trajectory)
    if trajectory_type is SynchronizedJointTrajectory:
        required_axis_type = JerkLimitedJointTrajectory
    elif trajectory_type is SynchronizedQuinticTrajectory:
        required_axis_type = QuinticJointTrajectory
    else:
        raise FeedbackReplanningError(
            "active trajectory must be a synchronized built-in trajectory"
        )
    axes = trajectory.axes
    if type(axes) is not tuple or not axes:
        raise FeedbackReplanningError(
            "active trajectory axes must be a non-empty built-in tuple"
        )
    if len(axes) > TRAJECTORY_MAXIMUM_AXES:
        raise FeedbackReplanningError(
            "active trajectory exceeds the supported axis count"
        )
    if any(type(axis) is not required_axis_type for axis in axes):
        raise FeedbackReplanningError(
            "active trajectory axes have an invalid built-in type"
        )
    if any(type(axis.limits) is not JointKinematicLimits for axis in axes):
        raise FeedbackReplanningError(
            "active trajectory limits must be built-in joint limits"
        )
    if required_axis_type is QuinticJointTrajectory and any(
        type(axis.start_state) is not JointBoundaryState
        or type(axis.target_state) is not JointBoundaryState
        for axis in axes
    ):
        raise FeedbackReplanningError(
            "active trajectory boundaries must be built-in joint states"
        )
    try:
        validated_axes = []
        for axis in axes:
            limits = JointKinematicLimits(
                axis.limits.maximum_velocity_degrees_per_second,
                axis.limits.maximum_acceleration_degrees_per_second_squared,
                axis.limits.maximum_jerk_degrees_per_second_cubed,
            )
            if required_axis_type is JerkLimitedJointTrajectory:
                validated_axes.append(JerkLimitedJointTrajectory(
                    axis.start_position_degrees,
                    axis.target_position_degrees,
                    limits,
                    axis.jerk_phase_seconds,
                    axis.constant_acceleration_phase_seconds,
                    axis.cruise_phase_seconds,
                    axis.applied_jerk_degrees_per_second_cubed,
                ))
            else:
                start_state = JointBoundaryState(
                    axis.start_state.position_degrees,
                    axis.start_state.velocity_degrees_per_second,
                    axis.start_state.acceleration_degrees_per_second_squared,
                )
                target_state = JointBoundaryState(
                    axis.target_state.position_degrees,
                    axis.target_state.velocity_degrees_per_second,
                    axis.target_state.acceleration_degrees_per_second_squared,
                )
                validated_axes.append(QuinticJointTrajectory(
                    start_state,
                    target_state,
                    axis.duration_seconds,
                    limits,
                ))
        if required_axis_type is JerkLimitedJointTrajectory:
            return SynchronizedJointTrajectory(tuple(validated_axes))
        return SynchronizedQuinticTrajectory(tuple(validated_axes))
    except Exception as exc:
        raise FeedbackReplanningError(
            "active trajectory state failed validation"
        ) from exc


def _safe_fault_text(value, fallback):
    try:
        text = str(value)
    except Exception:
        text = fallback
    text = " ".join(text.replace("\x00", "\\0").split())
    if not text:
        text = fallback
    if len(text) > REPLAN_FAULT_DETAIL_MAXIMUM_CHARACTERS:
        text = (
            text[:REPLAN_FAULT_DETAIL_MAXIMUM_CHARACTERS - 3]
            + "..."
        )
    return text


class FeedbackReplanStatus(Enum):
    HOLDING_FOR_ESTIMATE = "holding-for-estimate"
    HOLDING_AFTER_ESTIMATOR_RESET = "holding-after-estimator-reset"
    HOLDING_NO_FEASIBLE_CANDIDATE = "holding-no-feasible-candidate"
    HOLDING_STALE_ESTIMATE = "holding-stale-estimate"
    HOLDING_FUTURE_ESTIMATE = "holding-future-estimate"
    REPLACED = "replaced"
    CANCELLED = "cancelled"
    FAULTED = "faulted"


class FeedbackReplanFaultPhase(Enum):
    SELECTION = "selection"
    TARGET_RESOLUTION = "target-resolution"
    TRAJECTORY_CONSTRUCTION = "trajectory-construction"


@dataclass(frozen=True)
class FeedbackReplanFault:
    phase: FeedbackReplanFaultPhase
    error_type: str
    detail: str

    def __post_init__(self):
        if type(self.phase) is not FeedbackReplanFaultPhase:
            raise FeedbackReplanningError("fault phase is invalid")
        for field_name in ("error_type", "detail"):
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value or "\x00" in value:
                raise FeedbackReplanningError(
                    f"fault {field_name} must be non-empty text without NUL"
                )
            if len(value) > REPLAN_FAULT_DETAIL_MAXIMUM_CHARACTERS:
                raise FeedbackReplanningError(
                    f"fault {field_name} exceeds the character limit"
                )


def _fault_from_exception(phase, error):
    try:
        error_type_value = (
            f"{type(error).__module__}.{type(error).__qualname__}"
        )
    except Exception:
        error_type_value = "builtins.Exception"
    error_type = _safe_fault_text(
        error_type_value,
        "builtins.Exception",
    )
    detail = _safe_fault_text(error, "failure detail unavailable")
    return FeedbackReplanFault(phase, error_type, detail)


@dataclass(frozen=True)
class JointReplanTarget:
    """Joint target correlated to one selector evaluation and prediction."""

    estimate_timestamp_seconds: float
    evaluated_at_seconds: float
    intercept_timestamp_seconds: float
    joint_states: Tuple[JointBoundaryState, ...]

    def __post_init__(self):
        estimate_timestamp = _nonnegative_number(
            self.estimate_timestamp_seconds,
            "target estimate_timestamp_seconds",
        )
        evaluated_at = _nonnegative_number(
            self.evaluated_at_seconds,
            "target evaluated_at_seconds",
        )
        intercept_timestamp = _nonnegative_number(
            self.intercept_timestamp_seconds,
            "target intercept_timestamp_seconds",
        )
        if intercept_timestamp <= evaluated_at:
            raise FeedbackReplanningError(
                "target intercept timestamp must follow evaluation"
            )
        object.__setattr__(
            self,
            "estimate_timestamp_seconds",
            estimate_timestamp,
        )
        object.__setattr__(self, "evaluated_at_seconds", evaluated_at)
        object.__setattr__(
            self,
            "intercept_timestamp_seconds",
            intercept_timestamp,
        )
        object.__setattr__(
            self,
            "joint_states",
            _bounded_joint_states(self.joint_states, "target joint_states"),
        )


def _selection_hold_status(status):
    if status is InterceptSelectionStatus.NO_FEASIBLE_CANDIDATE:
        return FeedbackReplanStatus.HOLDING_NO_FEASIBLE_CANDIDATE
    if status is InterceptSelectionStatus.STALE_ESTIMATE:
        return FeedbackReplanStatus.HOLDING_STALE_ESTIMATE
    if status is InterceptSelectionStatus.FUTURE_ESTIMATE:
        return FeedbackReplanStatus.HOLDING_FUTURE_ESTIMATE
    return None


def _validated_selection(selection, update, evaluated_at):
    if type(selection) is not InterceptSelection:
        raise FeedbackReplanningError(
            "selector returned a non-built-in InterceptSelection"
        )
    if type(selection.candidates) is not tuple:
        raise FeedbackReplanningError(
            "selector returned a non-built-in candidate sequence"
        )
    selection = InterceptSelection(
        selection.status,
        selection.evaluated_at_seconds,
        selection.estimate,
        selection.candidates,
        selection.selected_candidate,
    )
    if (
        update.status is not EstimatorUpdateStatus.ESTIMATE_UPDATED
        or selection.estimate is not update.estimate
        or selection.evaluated_at_seconds != evaluated_at
    ):
        raise FeedbackReplanningError(
            "selector result does not match the current estimator update"
        )
    return selection


def _resolver_selection_snapshot(selection):
    if (
        selection.status is not InterceptSelectionStatus.SELECTED
        or selection.selected_candidate is None
    ):
        raise FeedbackReplanningError(
            "target resolution requires a selected intercept"
        )
    try:
        estimate, selected_candidate = copy.deepcopy((
            selection.estimate,
            selection.selected_candidate,
        ))
        snapshot = InterceptSelection(
            status=InterceptSelectionStatus.SELECTED,
            evaluated_at_seconds=selection.evaluated_at_seconds,
            estimate=estimate,
            candidates=(selected_candidate,),
            selected_candidate=selected_candidate,
        )
    except Exception as exc:
        raise FeedbackReplanningError(
            "selected intercept could not be isolated for target resolution"
        ) from exc
    return snapshot


@dataclass(frozen=True)
class FeedbackReplanEvent:
    sequence: int
    evaluated_at_seconds: float
    status: FeedbackReplanStatus
    trajectory_generation: int
    estimator_update: Optional[EstimatorUpdate] = None
    selection: Optional[InterceptSelection] = None
    replacement_trajectory: Optional[SynchronizedQuinticTrajectory] = None
    fault: Optional[FeedbackReplanFault] = None

    def __post_init__(self):
        object.__setattr__(
            self,
            "sequence",
            _nonnegative_integer(self.sequence, "event sequence"),
        )
        object.__setattr__(
            self,
            "evaluated_at_seconds",
            _nonnegative_number(
                self.evaluated_at_seconds,
                "event evaluated_at_seconds",
            ),
        )
        if type(self.status) is not FeedbackReplanStatus:
            raise FeedbackReplanningError("event status is invalid")
        object.__setattr__(
            self,
            "trajectory_generation",
            _nonnegative_integer(
                self.trajectory_generation,
                "event trajectory_generation",
            ),
        )
        if (
            self.estimator_update is not None
            and type(self.estimator_update) is not EstimatorUpdate
        ):
            raise FeedbackReplanningError(
                "event estimator_update must be a built-in EstimatorUpdate"
            )
        if (
            self.selection is not None
            and type(self.selection) is not InterceptSelection
        ):
            raise FeedbackReplanningError(
                "event selection must be a built-in InterceptSelection"
            )
        if (
            self.replacement_trajectory is not None
            and type(self.replacement_trajectory)
            is not SynchronizedQuinticTrajectory
        ):
            raise FeedbackReplanningError(
                "event replacement must be a built-in quintic trajectory"
            )
        if self.fault is not None and type(self.fault) is not FeedbackReplanFault:
            raise FeedbackReplanningError(
                "event fault must be a built-in FeedbackReplanFault"
            )
        self._validate_contract()

    def _validate_selection_link(self):
        if self.estimator_update is None or self.selection is None:
            raise FeedbackReplanningError(
                "selection event requires an estimator update and selection"
            )
        if (
            self.estimator_update.status
            is not EstimatorUpdateStatus.ESTIMATE_UPDATED
            or self.estimator_update.estimate is not self.selection.estimate
        ):
            raise FeedbackReplanningError(
                "event selection does not match the estimator update"
            )
        if self.selection.evaluated_at_seconds != self.evaluated_at_seconds:
            raise FeedbackReplanningError(
                "event selection timestamp does not match evaluation"
            )

    def _validate_contract(self):
        if self.status is FeedbackReplanStatus.CANCELLED:
            if any(value is not None for value in (
                self.estimator_update,
                self.selection,
                self.replacement_trajectory,
                self.fault,
            )):
                raise FeedbackReplanningError(
                    "cancelled event must not carry processing output"
                )
            return
        if self.status is FeedbackReplanStatus.FAULTED:
            if self.fault is None or self.replacement_trajectory is not None:
                raise FeedbackReplanningError(
                    "faulted event requires only fault-safe trajectory output"
                )
            if self.fault.phase is FeedbackReplanFaultPhase.SELECTION:
                if (
                    self.estimator_update is None
                    or self.estimator_update.status
                    is not EstimatorUpdateStatus.ESTIMATE_UPDATED
                    or self.selection is not None
                ):
                    raise FeedbackReplanningError(
                        "selection fault requires only an estimated update"
                    )
                return
            if self.selection is not None:
                self._validate_selection_link()
                if (
                    self.selection.status
                    is not InterceptSelectionStatus.SELECTED
                ):
                    raise FeedbackReplanningError(
                        "target or trajectory fault requires a selected intercept"
                    )
            else:
                raise FeedbackReplanningError(
                    "target or trajectory fault requires a selection"
                )
            return
        if self.fault is not None:
            raise FeedbackReplanningError(
                "non-faulted event must not carry a fault"
            )
        if self.status in (
            FeedbackReplanStatus.HOLDING_FOR_ESTIMATE,
            FeedbackReplanStatus.HOLDING_AFTER_ESTIMATOR_RESET,
        ):
            if (
                self.estimator_update is None
                or self.estimator_update.status
                is EstimatorUpdateStatus.ESTIMATE_UPDATED
                or self.selection is not None
                or self.replacement_trajectory is not None
            ):
                raise FeedbackReplanningError(
                    "estimate hold event has inconsistent processing output"
                )
            expected_status = FeedbackReplanStatus.HOLDING_FOR_ESTIMATE
            if (
                self.estimator_update.status
                is EstimatorUpdateStatus.BASELINE_RESET
            ):
                expected_status = (
                    FeedbackReplanStatus.HOLDING_AFTER_ESTIMATOR_RESET
                )
            if self.status is not expected_status:
                raise FeedbackReplanningError(
                    "estimate hold event does not preserve reset disposition"
                )
            return
        self._validate_selection_link()
        if self.status is FeedbackReplanStatus.REPLACED:
            if (
                self.selection.status is not InterceptSelectionStatus.SELECTED
                or self.replacement_trajectory is None
            ):
                raise FeedbackReplanningError(
                    "replacement event requires a selected intercept and trajectory"
                )
            return
        expected_status = _selection_hold_status(self.selection.status)
        if (
            expected_status is not self.status
            or self.replacement_trajectory is not None
        ):
            raise FeedbackReplanningError(
                "selection hold event has inconsistent processing output"
            )


@dataclass(frozen=True)
class FeedbackReplanReplayResult:
    events: Tuple[FeedbackReplanEvent, ...]
    replay_sample_count: int
    active_trajectory: Union[
        SynchronizedJointTrajectory,
        SynchronizedQuinticTrajectory,
    ]
    trajectory_generation: int

    def __post_init__(self):
        if type(self.events) is not tuple or not self.events:
            raise FeedbackReplanningError(
                "replay result events must be a non-empty built-in tuple"
            )
        sample_count = _nonnegative_integer(
            self.replay_sample_count,
            "replay result sample count",
        )
        if sample_count == 0 or len(self.events) > sample_count:
            raise FeedbackReplanningError(
                "replay result event count is invalid"
            )
        if sample_count > OBSERVATION_REPLAY_MAXIMUM_RECORDS:
            raise FeedbackReplanningError(
                "replay result exceeds the maximum sample count"
            )
        if any(type(event) is not FeedbackReplanEvent for event in self.events):
            raise FeedbackReplanningError(
                "replay result contains an invalid event"
            )
        previous_evaluated_at = None
        for expected_sequence, event in enumerate(self.events):
            if event.sequence != expected_sequence:
                raise FeedbackReplanningError(
                    "replay event sequences must be contiguous"
                )
            if (
                previous_evaluated_at is not None
                and event.evaluated_at_seconds < previous_evaluated_at
            ):
                raise FeedbackReplanningError(
                    "replay event timestamps must not move backward"
                )
            previous_evaluated_at = event.evaluated_at_seconds
        if any(
            event.status is FeedbackReplanStatus.FAULTED
            for event in self.events[:-1]
        ):
            raise FeedbackReplanningError(
                "replay fault event must be terminal"
            )
        if any(
            event.status is FeedbackReplanStatus.CANCELLED
            for event in self.events
        ):
            raise FeedbackReplanningError(
                "feedback replay must not contain cancellation events"
            )
        if (
            len(self.events) < sample_count
            and self.events[-1].status is not FeedbackReplanStatus.FAULTED
        ):
            raise FeedbackReplanningError(
                "partial replay result requires a terminal fault event"
            )
        generation = _nonnegative_integer(
            self.trajectory_generation,
            "replay result trajectory_generation",
        )
        expected_generation = 0
        last_replacement = None
        for event in self.events:
            if event.status is FeedbackReplanStatus.REPLACED:
                expected_generation += 1
                last_replacement = event.replacement_trajectory
            if event.trajectory_generation != expected_generation:
                raise FeedbackReplanningError(
                    "replay event generation sequence is invalid"
                )
        if expected_generation != generation:
            raise FeedbackReplanningError(
                "replay result generation does not match replacement events"
            )
        validated_active = _validated_active_trajectory(
            self.active_trajectory
        )
        if (
            last_replacement is not None
            and validated_active
            != _validated_active_trajectory(last_replacement)
        ):
            raise FeedbackReplanningError(
                "replay active trajectory does not match the last replacement"
            )
        object.__setattr__(self, "replay_sample_count", sample_count)
        object.__setattr__(self, "trajectory_generation", generation)

    @property
    def processed_all_samples(self):
        return len(self.events) == self.replay_sample_count

    @property
    def faulted(self):
        return self.events[-1].status is FeedbackReplanStatus.FAULTED

    @property
    def complete(self):
        return self.processed_all_samples and not self.faulted


class FeedbackReplanner:
    """Single-owner coordinator; no method transmits a controller command."""

    def __init__(
        self,
        estimator_config,
        selector,
        initial_trajectory,
        initial_trajectory_started_at_seconds,
        target_resolver,
    ):
        if type(estimator_config) is not ConstantVelocityEstimatorConfig:
            raise FeedbackReplanningError(
                "estimator_config must be built-in ConstantVelocityEstimatorConfig"
            )
        if type(selector) is not InterceptSelector:
            raise FeedbackReplanningError(
                "selector must be a built-in InterceptSelector"
            )
        try:
            selector_source_config = selector.config
        except Exception as exc:
            raise FeedbackReplanningError(
                "selector config is unavailable"
            ) from exc
        if type(selector_source_config) is not InterceptSelectorConfig:
            raise FeedbackReplanningError(
                "selector config must be built-in InterceptSelectorConfig"
            )
        try:
            estimator_config = ConstantVelocityEstimatorConfig(
                estimator_config.frame_id,
                estimator_config.maximum_observation_age_seconds,
                estimator_config.minimum_sample_interval_seconds,
                estimator_config.maximum_sample_interval_seconds,
                estimator_config.maximum_future_skew_seconds,
            )
            selector_config = InterceptSelectorConfig(
                selector_source_config.frame_id,
                selector_source_config.minimum_lead_time_seconds,
                selector_source_config.maximum_lead_time_seconds,
                selector_source_config.candidate_interval_seconds,
                selector_source_config.maximum_estimate_age_seconds,
                selector_source_config.maximum_future_skew_seconds,
                selector_source_config.maximum_position_standard_deviation_mm,
                selector_source_config.maximum_terminal_speed_mm_per_second,
                selector_source_config.minimum_arrival_margin_seconds,
            )
        except Exception as exc:
            raise FeedbackReplanningError(
                "estimator or selector config failed validation"
            ) from exc
        if (
            type(selector_source_config.candidate_lead_times) is not tuple
            or selector_source_config.candidate_lead_times
            != selector_config.candidate_lead_times
        ):
            raise FeedbackReplanningError(
                "selector candidate schedule failed validation"
            )
        if selector_config.minimum_lead_time_seconds <= 0:
            raise FeedbackReplanningError(
                "feedback replanning requires positive candidate lead times"
            )
        if selector_config.frame_id != estimator_config.frame_id:
            raise FeedbackReplanningError(
                "selector and estimator frames must match"
            )
        if not callable(target_resolver):
            raise FeedbackReplanningError("target_resolver must be callable")
        active_trajectory = _validated_active_trajectory(initial_trajectory)
        self._estimator = ConstantAccelerationEstimator(estimator_config)
        self._selector = selector
        self._active_trajectory = active_trajectory
        self._active_started_at_seconds = _nonnegative_number(
            initial_trajectory_started_at_seconds,
            "initial trajectory started_at_seconds",
        )
        self._target_resolver = target_resolver
        self._trajectory_generation = 0
        self._next_event_sequence = 0
        self._last_event_at_seconds = None
        self._fault = None
        self._cancelled = False
        self._processing = False

    @property
    def active_trajectory(self):
        return self._active_trajectory

    @property
    def active_started_at_seconds(self):
        return self._active_started_at_seconds

    @property
    def trajectory_generation(self):
        return self._trajectory_generation

    @property
    def fault(self):
        return self._fault

    @property
    def cancelled(self):
        return self._cancelled

    def _require_operational(self):
        if self._fault is not None:
            raise FeedbackReplanningError("feedback replanner is faulted")
        if self._cancelled:
            raise FeedbackReplanningError("feedback replanner is cancelled")
        if self._processing:
            raise FeedbackReplanningError(
                "feedback replanner does not permit reentrant processing"
            )

    def _validated_event_time(self, value):
        evaluated_at = _nonnegative_number(value, "evaluated_at_seconds")
        lower_bound = self._active_started_at_seconds
        if self._last_event_at_seconds is not None:
            lower_bound = max(lower_bound, self._last_event_at_seconds)
        tolerance = _comparison_tolerance(evaluated_at, lower_bound)
        if evaluated_at < lower_bound - tolerance:
            raise FeedbackReplanningError(
                "event timestamp precedes active coordinator state"
            )
        return max(evaluated_at, lower_bound)

    def _event(
        self,
        evaluated_at,
        status,
        update=None,
        selection=None,
        replacement=None,
        fault=None,
        generation=None,
    ):
        if generation is None:
            generation = self._trajectory_generation
        event = FeedbackReplanEvent(
            sequence=self._next_event_sequence,
            evaluated_at_seconds=evaluated_at,
            status=status,
            trajectory_generation=generation,
            estimator_update=update,
            selection=selection,
            replacement_trajectory=replacement,
            fault=fault,
        )
        self._next_event_sequence += 1
        self._last_event_at_seconds = evaluated_at
        return event

    def _fault_event(self, phase, error, evaluated_at, update, selection=None):
        fault = _fault_from_exception(phase, error)
        event = self._event(
            evaluated_at,
            FeedbackReplanStatus.FAULTED,
            update=update,
            selection=selection,
            fault=fault,
        )
        self._fault = fault
        return event

    @staticmethod
    def _validate_target(target, selection, axis_count):
        if type(target) is not JointReplanTarget:
            raise FeedbackReplanningError(
                "target_resolver must return built-in JointReplanTarget"
            )
        target = JointReplanTarget(
            target.estimate_timestamp_seconds,
            target.evaluated_at_seconds,
            target.intercept_timestamp_seconds,
            target.joint_states,
        )
        selected_candidate = selection.selected_candidate
        if (
            target.estimate_timestamp_seconds
            != selection.estimate.timestamp_seconds
            or target.evaluated_at_seconds
            != selection.evaluated_at_seconds
            or target.intercept_timestamp_seconds
            != selected_candidate.prediction.timestamp_seconds
        ):
            raise FeedbackReplanningError(
                "resolved target does not match the selected intercept"
            )
        if len(target.joint_states) != axis_count:
            raise FeedbackReplanningError(
                "resolved target axis count does not match the active trajectory"
            )
        return target

    def _replacement(self, active_trajectory, selection, target):
        active_duration = active_trajectory.duration_seconds
        try:
            elapsed = math.fsum((
                selection.evaluated_at_seconds,
                -self._active_started_at_seconds,
            ))
            duration = math.fsum((
                target.intercept_timestamp_seconds,
                -selection.evaluated_at_seconds,
            ))
        except (OverflowError, ValueError) as exc:
            raise FeedbackReplanningError(
                "replacement timing is outside the host numeric range"
            ) from exc
        elapsed = _nonnegative_number(elapsed, "replacement elapsed_seconds")
        duration = _positive_number(duration, "replacement duration_seconds")
        return replan_synchronized_quintic_trajectory(
            active_trajectory,
            min(elapsed, active_duration),
            target.joint_states,
            duration,
        )

    def process_observation(self, observation, received_at_seconds):
        self._require_operational()
        evaluated_at = self._validated_event_time(received_at_seconds)
        self._processing = True
        try:
            update = self._estimator.add_observation(
                observation,
                evaluated_at,
            )
            if update.status is not EstimatorUpdateStatus.ESTIMATE_UPDATED:
                status = FeedbackReplanStatus.HOLDING_FOR_ESTIMATE
                if update.status is EstimatorUpdateStatus.BASELINE_RESET:
                    status = (
                        FeedbackReplanStatus.HOLDING_AFTER_ESTIMATOR_RESET
                    )
                return self._event(
                    evaluated_at,
                    status,
                    update=update,
                )
            try:
                selection = self._selector.select(
                    update.estimate,
                    evaluated_at,
                )
            except Exception as exc:
                return self._fault_event(
                    FeedbackReplanFaultPhase.SELECTION,
                    exc,
                    evaluated_at,
                    update,
                )
            try:
                selection = _validated_selection(
                    selection,
                    update,
                    evaluated_at,
                )
            except Exception as exc:
                return self._fault_event(
                    FeedbackReplanFaultPhase.SELECTION,
                    exc,
                    evaluated_at,
                    update,
                )
            if selection.status is not InterceptSelectionStatus.SELECTED:
                status = _selection_hold_status(selection.status)
                if status is None:
                    return self._fault_event(
                        FeedbackReplanFaultPhase.SELECTION,
                        FeedbackReplanningError(
                            "selector returned an unsupported status"
                        ),
                        evaluated_at,
                        update,
                    )
                return self._event(
                    evaluated_at,
                    status,
                    update=update,
                    selection=selection,
                )
            try:
                active_trajectory = _validated_active_trajectory(
                    self._active_trajectory
                )
            except Exception as exc:
                return self._fault_event(
                    FeedbackReplanFaultPhase.TRAJECTORY_CONSTRUCTION,
                    exc,
                    evaluated_at,
                    update,
                    selection,
                )
            try:
                resolver_selection = _resolver_selection_snapshot(selection)
                target = self._target_resolver(resolver_selection)
                target = self._validate_target(
                    target,
                    selection,
                    len(active_trajectory.axes),
                )
            except Exception as exc:
                self._active_trajectory = active_trajectory
                return self._fault_event(
                    FeedbackReplanFaultPhase.TARGET_RESOLUTION,
                    exc,
                    evaluated_at,
                    update,
                    selection,
                )
            try:
                replacement = self._replacement(
                    active_trajectory,
                    selection,
                    target,
                )
            except Exception as exc:
                self._active_trajectory = active_trajectory
                return self._fault_event(
                    FeedbackReplanFaultPhase.TRAJECTORY_CONSTRUCTION,
                    exc,
                    evaluated_at,
                    update,
                    selection,
                )
            event = self._event(
                evaluated_at,
                FeedbackReplanStatus.REPLACED,
                update=update,
                selection=selection,
                replacement=replacement,
                generation=self._trajectory_generation + 1,
            )
            self._active_trajectory = replacement
            self._active_started_at_seconds = evaluated_at
            self._trajectory_generation += 1
            return event
        finally:
            self._processing = False

    def cancel(self, evaluated_at_seconds):
        self._require_operational()
        evaluated_at = self._validated_event_time(evaluated_at_seconds)
        event = self._event(
            evaluated_at,
            FeedbackReplanStatus.CANCELLED,
        )
        self._cancelled = True
        return event


def run_feedback_replanning_replay(
    replay,
    estimator_config,
    selector,
    initial_trajectory,
    initial_trajectory_started_at_seconds,
    target_resolver,
):
    """Run one bounded replay without controller or GUI side effects."""
    if type(replay) is not ObservationReplay:
        raise FeedbackReplanningError(
            "replay must be a built-in ObservationReplay"
        )
    if type(estimator_config) is not ConstantVelocityEstimatorConfig:
        raise FeedbackReplanningError(
            "replay estimator_config must be built-in "
            "ConstantVelocityEstimatorConfig"
        )
    try:
        estimator_frame = estimator_config.frame_id
    except Exception as exc:
        raise FeedbackReplanningError(
            "replay estimator frame is unavailable"
        ) from exc
    if replay.frame_id != estimator_frame:
        raise FeedbackReplanningError(
            "replay and estimator frames must match"
        )
    if type(selector) is not InterceptSelector:
        raise FeedbackReplanningError(
            "replay selector must be a built-in InterceptSelector"
        )
    try:
        selector_config = selector.config
        candidate_lead_times = selector_config.candidate_lead_times
    except Exception as exc:
        raise FeedbackReplanningError(
            "replay selector candidate schedule is unavailable"
        ) from exc
    if (
        type(selector_config) is not InterceptSelectorConfig
        or type(candidate_lead_times) is not tuple
        or not candidate_lead_times
    ):
        raise FeedbackReplanningError(
            "replay selector candidate schedule is invalid"
        )
    # The owned acceleration estimator cannot emit an estimate before one
    # baseline and one warmup record. Resets only reduce this count. Deriving
    # the bound from record count avoids an estimator pre-pass, so sample
    # contents beyond a terminal coordinator fault remain uninspected.
    estimate_update_count = max(0, len(replay.samples) - 2)
    candidate_evaluation_count = (
        estimate_update_count * len(candidate_lead_times)
    )
    if (
        candidate_evaluation_count
        > INTERCEPT_REPLAY_MAXIMUM_CANDIDATE_EVALUATIONS
    ):
        raise FeedbackReplanningError(
            "replay exceeds the maximum candidate evaluation count"
        )
    replanner = FeedbackReplanner(
        estimator_config,
        selector,
        initial_trajectory,
        initial_trajectory_started_at_seconds,
        target_resolver,
    )
    events = []
    for sample in replay.samples:
        event = replanner.process_observation(
            sample.observation,
            sample.received_at_seconds,
        )
        events.append(event)
        if event.status is FeedbackReplanStatus.FAULTED:
            break
    return FeedbackReplanReplayResult(
        events=tuple(events),
        replay_sample_count=len(replay.samples),
        active_trajectory=replanner.active_trajectory,
        trajectory_generation=replanner.trajectory_generation,
    )
