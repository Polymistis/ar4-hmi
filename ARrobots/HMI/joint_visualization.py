"""Tk-thread presentation for desired, target, estimated, and actual joints."""

import tkinter as tk
import time

from ARrobots.HMI.joint_motion import (
    JOINT_COUNT,
    JOINT_TELEMETRY_AXIS_COUNT,
    MotionInputError,
    estimate_commanded_joint_trajectory,
    finite_number,
)


ESTIMATED_MARKER_ROLE = "estimated"
ENCODER_MARKER_ROLE = "encoder"
TARGET_MARKER_ROLE = "target"
ESTIMATED_MARKER_COLOR = "#00D7FF"
ESTIMATED_MARKER_OUTLINE_COLOR = "#003B49"
ESTIMATED_MARKER_WIDTH = 3
ESTIMATED_MARKER_HEIGHT_RATIO = 1.0
ESTIMATED_MARKER_MINIMUM_HEIGHT = 8
ENCODER_MARKER_COLOR = "#FFB000"
ENCODER_MARKER_OUTLINE_COLOR = "#5C3A00"
ENCODER_MARKER_WIDTH = 9
ENCODER_MARKER_HEIGHT_RATIO = 0.6
ENCODER_MARKER_MINIMUM_HEIGHT = 8
TARGET_MARKER_COLOR = "#D946EF"
TARGET_MARKER_OUTLINE_COLOR = "#4A044E"
TARGET_MARKER_WIDTH = 13
TARGET_MARKER_HEIGHT_RATIO = 0.25
TARGET_MARKER_MINIMUM_HEIGHT = 4
GHOST_MARKER_MINIMUM_HEIGHT = 12
GHOST_MARKER_MAXIMUM_HEIGHT = 24
_SLIDER_DRAG_ATTRIBUTE = "_ar4_joint_slider_drag_active"
_SLIDER_OVERLAY_BINDINGS_ATTRIBUTE = "_ar4_joint_overlay_bindings_installed"

_MARKER_STYLES = {
    ESTIMATED_MARKER_ROLE: (
        ESTIMATED_MARKER_COLOR,
        ESTIMATED_MARKER_OUTLINE_COLOR,
        ESTIMATED_MARKER_WIDTH,
        ESTIMATED_MARKER_HEIGHT_RATIO,
        ESTIMATED_MARKER_MINIMUM_HEIGHT,
    ),
    ENCODER_MARKER_ROLE: (
        ENCODER_MARKER_COLOR,
        ENCODER_MARKER_OUTLINE_COLOR,
        ENCODER_MARKER_WIDTH,
        ENCODER_MARKER_HEIGHT_RATIO,
        ENCODER_MARKER_MINIMUM_HEIGHT,
    ),
    TARGET_MARKER_ROLE: (
        TARGET_MARKER_COLOR,
        TARGET_MARKER_OUTLINE_COLOR,
        TARGET_MARKER_WIDTH,
        TARGET_MARKER_HEIGHT_RATIO,
        TARGET_MARKER_MINIMUM_HEIGHT,
    ),
}


def _fixed_items(values, expected_length, field_name):
    if isinstance(values, (str, bytes)):
        raise MotionInputError(f"{field_name} must be a sequence")
    try:
        iterator = iter(values)
    except TypeError as exc:
        raise MotionInputError(f"{field_name} must be a sequence") from exc

    items = []
    for _ in range(expected_length + 1):
        try:
            items.append(next(iterator))
        except StopIteration:
            break
    if len(items) != expected_length:
        raise MotionInputError(
            f"{field_name} must contain {expected_length} values"
        )
    return tuple(items)


def _joint_positions(values, field_name):
    items = _fixed_items(values, JOINT_COUNT, field_name)
    return tuple(
        finite_number(value, f"{field_name}[{axis}]")
        for axis, value in enumerate(items, start=1)
    )


def _slider_drag_active(slider):
    active = getattr(slider, _SLIDER_DRAG_ATTRIBUTE, False)
    if not isinstance(active, bool):
        raise MotionInputError(
            "joint slider drag state must be boolean"
        )
    return active


def set_joint_slider_positions(sliders, positions):
    """Preserve active operator drags while updating every idle slider.

    The normalized requested target is returned even when an active slider
    retains the operator-controlled value.
    """

    slider_items = _fixed_items(
        sliders,
        JOINT_COUNT,
        "joint position sliders",
    )
    if any(
        not callable(getattr(slider, "get", None))
        or not callable(getattr(slider, "set", None))
        for slider in slider_items
    ):
        raise TypeError("joint position slider contract is invalid")
    target = _joint_positions(positions, "joint slider positions")
    active = tuple(
        _slider_drag_active(slider)
        for slider in slider_items
    )
    previous = []
    for axis, (slider, drag_active) in enumerate(
        zip(slider_items, active),
        start=1,
    ):
        if drag_active:
            previous.append(None)
            continue
        try:
            previous.append(slider.get())
        except Exception as exc:
            raise RuntimeError(
                f"unable to snapshot J{axis} slider: {exc}"
            ) from exc

    changed = []
    try:
        for axis, (slider, value, drag_active) in enumerate(
            zip(slider_items, target, active),
        ):
            if drag_active:
                continue
            slider.set(value)
            changed.append(axis)
    except Exception as exc:
        rollback_failures = []
        for axis in changed:
            try:
                slider_items[axis].set(previous[axis])
            except Exception as rollback_exc:
                rollback_failures.append(
                    f"J{axis + 1}: {rollback_exc}"
                )
        detail = (
            "; rollback failed for " + ", ".join(rollback_failures)
            if rollback_failures
            else ""
        )
        raise RuntimeError(
            f"unable to update joint slider{detail}"
        ) from exc
    return target


def slider_marker_geometry(
    value,
    lower,
    upper,
    slider_x,
    slider_y,
    slider_width,
    slider_height,
):
    """Return a clamped marker center and height in parent coordinates."""

    marker_value = finite_number(value, "slider marker value")
    lower_bound = finite_number(lower, "slider lower bound")
    upper_bound = finite_number(upper, "slider upper bound")
    x_origin = finite_number(slider_x, "slider x position")
    y_origin = finite_number(slider_y, "slider y position")
    width = finite_number(slider_width, "slider width")
    height = finite_number(slider_height, "slider height")
    if width <= 0 or height <= 0:
        raise MotionInputError("slider geometry must be positive")

    if lower_bound == upper_bound:
        fraction = 0.5
    else:
        fraction = (marker_value - lower_bound) / (
            upper_bound - lower_bound
        )
        fraction = min(1.0, max(0.0, fraction))

    thumb_extent = min(width, height)
    travel = max(0.0, width - thumb_extent)
    marker_x = x_origin + thumb_extent * 0.5 + fraction * travel
    marker_y = y_origin + height * 0.5
    marker_height = min(
        GHOST_MARKER_MAXIMUM_HEIGHT,
        max(GHOST_MARKER_MINIMUM_HEIGHT, height),
    )
    return marker_x, marker_y, marker_height


class GhostSliderMarker:
    """Role-specific overlay marker that leaves the slider thumb visible."""

    def __init__(self, parent, slider, role):
        required_slider_methods = (
            "cget",
            "event_generate",
            "winfo_height",
            "winfo_reqheight",
            "winfo_reqwidth",
            "winfo_rootx",
            "winfo_rooty",
            "winfo_width",
            "winfo_x",
            "winfo_y",
            "bind",
            "bind_all",
        )
        if any(
            not callable(getattr(slider, method_name, None))
            for method_name in required_slider_methods
        ):
            raise TypeError("slider does not satisfy the overlay contract")
        style = _MARKER_STYLES.get(role)
        if style is None:
            raise TypeError(
                "ghost marker role must be 'target', 'estimated', or 'encoder'"
            )
        (
            color,
            outline_color,
            width,
            height_ratio,
            minimum_height,
        ) = style

        self._slider = slider
        self._width = width
        self._height_ratio = height_ratio
        self._minimum_height = minimum_height
        self._placement = None
        self._visible = False
        self._marker = tk.Frame(
            parent,
            background=color,
            borderwidth=0,
            highlightbackground=outline_color,
            highlightthickness=1,
            takefocus=0,
        )
        for sequence in (
            "<ButtonPress-1>",
            "<B1-Motion>",
            "<ButtonRelease-1>",
        ):
            self._marker.bind(
                sequence,
                lambda event, forwarded=sequence: self._forward_pointer(
                    forwarded,
                    event,
                ),
            )
        bindings_installed = getattr(
            self._slider,
            _SLIDER_OVERLAY_BINDINGS_ATTRIBUTE,
            False,
        )
        if not isinstance(bindings_installed, bool):
            raise MotionInputError(
                "joint slider overlay binding state must be boolean"
            )
        if not bindings_installed:
            setattr(self._slider, _SLIDER_DRAG_ATTRIBUTE, False)
            self._slider.bind(
                "<ButtonPress-1>",
                lambda _event: setattr(
                    self._slider,
                    _SLIDER_DRAG_ATTRIBUTE,
                    True,
                ),
                add="+",
            )
            self._slider.bind(
                "<ButtonRelease-1>",
                lambda _event: setattr(
                    self._slider,
                    _SLIDER_DRAG_ATTRIBUTE,
                    False,
                ),
                add="+",
            )
            self._slider.bind_all(
                "<ButtonRelease-1>",
                lambda _event: setattr(
                    self._slider,
                    _SLIDER_DRAG_ATTRIBUTE,
                    False,
                ),
                add="+",
            )
            setattr(
                self._slider,
                _SLIDER_OVERLAY_BINDINGS_ATTRIBUTE,
                True,
            )

    def _forward_pointer(self, sequence, event):
        x = int(event.x_root - self._slider.winfo_rootx())
        y = int(event.y_root - self._slider.winfo_rooty())
        self._slider.event_generate(sequence, x=x, y=y)
        return "break"

    def show(self, value):
        """Show or move the marker and report whether placement changed."""

        width = self._slider.winfo_width()
        height = self._slider.winfo_height()
        if width <= 1:
            width = self._slider.winfo_reqwidth()
        if height <= 1:
            height = self._slider.winfo_reqheight()
        marker_x, marker_y, marker_height = slider_marker_geometry(
            value,
            self._slider.cget("from"),
            self._slider.cget("to"),
            self._slider.winfo_x(),
            self._slider.winfo_y(),
            width,
            height,
        )
        placement = (
            round(marker_x),
            round(marker_y),
            self._width,
            max(
                self._minimum_height,
                round(marker_height * self._height_ratio),
            ),
        )
        if self._visible and placement == self._placement:
            return False
        self._marker.place(
            x=placement[0],
            y=placement[1],
            anchor="center",
            width=placement[2],
            height=placement[3],
        )
        self._marker.lift()
        self._placement = placement
        self._visible = True
        return True

    def hide(self):
        """Hide the marker and report whether visibility changed."""

        if not self._visible:
            return False
        self._marker.place_forget()
        self._visible = False
        return True

    def raise_above(self):
        """Raise a visible marker and report whether a raise was issued."""

        if not self._visible:
            return False
        self._marker.lift()
        return True


class JointMotionVisualization:
    """Keep active targets, estimates, and actual samples distinct."""

    def __init__(
        self,
        sliders,
        target_markers,
        estimated_markers,
        encoder_markers,
        estimated_enabled_provider,
        encoder_enabled_provider,
        actual_source_provider,
        clock=time.monotonic,
    ):
        self._sliders = _fixed_items(
            sliders,
            JOINT_COUNT,
            "joint visualization sliders",
        )
        self._target_markers = _fixed_items(
            target_markers,
            JOINT_COUNT,
            "target joint visualization markers",
        )
        self._estimated_markers = _fixed_items(
            estimated_markers,
            JOINT_COUNT,
            "estimated joint visualization markers",
        )
        self._encoder_markers = _fixed_items(
            encoder_markers,
            JOINT_TELEMETRY_AXIS_COUNT,
            "encoder joint visualization markers",
        )
        if any(
            not callable(getattr(slider, "get", None))
            or not callable(getattr(slider, "set", None))
            for slider in self._sliders
        ):
            raise TypeError("joint visualization slider contract is invalid")
        if any(
            not callable(getattr(marker, "show", None))
            or not callable(getattr(marker, "hide", None))
            or not callable(getattr(marker, "raise_above", None))
            for marker in (
                self._target_markers
                + self._estimated_markers
                + self._encoder_markers
            )
        ):
            raise TypeError("joint visualization marker contract is invalid")
        if not callable(estimated_enabled_provider):
            raise TypeError(
                "estimated visualization enabled provider must be callable"
            )
        if not callable(encoder_enabled_provider):
            raise TypeError(
                "encoder visualization enabled provider must be callable"
            )
        if not callable(actual_source_provider):
            raise TypeError(
                "actual-position source provider must be callable"
            )
        if not callable(clock):
            raise TypeError("joint visualization clock must be callable")

        self._estimated_enabled_provider = estimated_enabled_provider
        self._encoder_enabled_provider = encoder_enabled_provider
        self._actual_source_provider = actual_source_provider
        self._clock = clock
        self._trajectory = None
        self._started_at = None
        self._actual_joint_positions = None
        self._actual_position_source = None
        self._target_markers_visible = False
        self._estimated_markers_visible = False
        self._encoder_markers_visible = False

    @property
    def active(self):
        return self._trajectory is not None

    @staticmethod
    def _enabled(provider, field_name):
        enabled = provider()
        if isinstance(enabled, bool):
            return enabled
        if isinstance(enabled, int) and enabled in (0, 1):
            return enabled == 1
        raise MotionInputError(
            f"{field_name} must contain zero or one"
        )

    def _hide_marker_group(
        self,
        markers,
        visibility_attribute,
        group_name,
    ):
        if not getattr(self, visibility_attribute):
            return True
        failures = []
        for axis, marker in enumerate(markers, start=1):
            try:
                marker.hide()
            except Exception as exc:
                failures.append(f"J{axis}: {exc}")
        if failures:
            raise RuntimeError(
                f"unable to hide {group_name} joint markers: "
                + "; ".join(failures)
            )
        setattr(self, visibility_attribute, False)
        return True

    def _hide_markers(self):
        failures = []
        for markers, visibility_attribute, group_name in (
            (
                self._encoder_markers,
                "_encoder_markers_visible",
                "encoder",
            ),
            (
                self._estimated_markers,
                "_estimated_markers_visible",
                "estimated",
            ),
            (
                self._target_markers,
                "_target_markers_visible",
                "target",
            ),
        ):
            try:
                self._hide_marker_group(
                    markers,
                    visibility_attribute,
                    group_name,
                )
            except Exception as exc:
                failures.append(str(exc))
        if failures:
            raise RuntimeError("; ".join(failures))
        return True

    def _show_marker_group(
        self,
        markers,
        positions,
        visibility_attribute,
    ):
        setattr(self, visibility_attribute, True)
        changed = []
        for marker, position in zip(markers, positions):
            marker_changed = marker.show(position)
            if not isinstance(marker_changed, bool):
                raise RuntimeError(
                    "joint visualization marker show result must be boolean"
                )
            changed.append(marker_changed)
        return tuple(changed)

    def set_desired(self, positions):
        return set_joint_slider_positions(self._sliders, positions)

    @staticmethod
    def _actual_positions(positions):
        values = _fixed_items(
            positions,
            JOINT_TELEMETRY_AXIS_COUNT,
            "actual joint positions",
        )
        return tuple(
            finite_number(value, f"actual joint positions[{axis}]")
            for axis, value in enumerate(values, start=1)
        )

    def observe_actual(self, positions):
        normalized = self._actual_positions(positions)
        if self._trajectory is None:
            return False
        source = self._actual_source_provider()
        if source is None:
            raise RuntimeError(
                "actual-position source is unavailable"
            )
        self._actual_joint_positions = normalized
        self._actual_position_source = source
        return True

    def clear_actual(self):
        self._hide_marker_group(
            self._encoder_markers,
            "_encoder_markers_visible",
            "encoder",
        )
        self._actual_joint_positions = None
        self._actual_position_source = None

    def start(
        self,
        start_positions,
        move,
        minimum_step_delay_microseconds,
        started_at_seconds=None,
    ):
        trajectory = estimate_commanded_joint_trajectory(
            start_positions,
            move,
            minimum_step_delay_microseconds,
        )
        started_at = finite_number(
            (
                self._clock()
                if started_at_seconds is None
                else started_at_seconds
            ),
            "joint visualization start time",
        )
        self._trajectory = trajectory
        self._started_at = started_at
        try:
            self.refresh()
        except Exception as exc:
            self._trajectory = None
            self._started_at = None
            self._actual_joint_positions = None
            self._actual_position_source = None
            try:
                self._hide_markers()
            except Exception as cleanup_exc:
                raise RuntimeError(
                    f"{exc}; marker cleanup failed: {cleanup_exc}"
                ) from exc
            raise
        return trajectory

    def refresh(self):
        try:
            estimated_enabled = self._enabled(
                self._estimated_enabled_provider,
                "estimated joint visualization toggle",
            )
            encoder_enabled = self._enabled(
                self._encoder_enabled_provider,
                "encoder joint visualization toggle",
            )
            actual_available = self._actual_joint_positions is not None
            if actual_available:
                current_source = self._actual_source_provider()
                if (
                    current_source is None
                    or current_source is not self._actual_position_source
                ):
                    self._actual_joint_positions = None
                    self._actual_position_source = None
                    actual_available = False

            if self._trajectory is None:
                self._hide_marker_group(
                    self._target_markers,
                    "_target_markers_visible",
                    "target",
                )
                self._hide_marker_group(
                    self._estimated_markers,
                    "_estimated_markers_visible",
                    "estimated",
                )
                if (
                    encoder_enabled
                    and actual_available
                ):
                    self._show_marker_group(
                        self._encoder_markers,
                        self._actual_joint_positions,
                        "_encoder_markers_visible",
                    )
                else:
                    self._hide_marker_group(
                        self._encoder_markers,
                        "_encoder_markers_visible",
                        "encoder",
                    )
                return None

            now = finite_number(self._clock(), "joint visualization clock")
            elapsed = max(0.0, now - self._started_at)
            estimated_positions = self._trajectory.positions_at(elapsed)
            target_changes = self._show_marker_group(
                self._target_markers,
                self._trajectory.target_positions,
                "_target_markers_visible",
            )
            estimated_changes = (False,) * JOINT_COUNT
            if estimated_enabled:
                estimated_changes = self._show_marker_group(
                    self._estimated_markers,
                    estimated_positions,
                    "_estimated_markers_visible",
                )
            else:
                self._hide_marker_group(
                    self._estimated_markers,
                    "_estimated_markers_visible",
                    "estimated",
                )
            encoder_changes = (False,) * JOINT_TELEMETRY_AXIS_COUNT
            encoder_visible = (
                encoder_enabled
                and actual_available
            )
            if encoder_visible:
                encoder_changes = self._show_marker_group(
                    self._encoder_markers,
                    self._actual_joint_positions,
                    "_encoder_markers_visible",
                )
            else:
                self._hide_marker_group(
                    self._encoder_markers,
                    "_encoder_markers_visible",
                    "encoder",
                )

            for axis, target_changed in enumerate(target_changes):
                estimated_changed = estimated_changes[axis]
                estimated_reasserted = False
                if (
                    target_changed
                    and estimated_enabled
                    and not estimated_changed
                ):
                    raised = self._estimated_markers[axis].raise_above()
                    if not isinstance(raised, bool) or not raised:
                        raise RuntimeError(
                            "visible estimated marker could not "
                            "reassert target layering"
                        )
                    estimated_reasserted = True
                if axis >= JOINT_TELEMETRY_AXIS_COUNT:
                    continue
                encoder_changed = encoder_changes[axis]
                if (
                    encoder_visible
                    and (target_changed or estimated_changed)
                    and (
                        not encoder_changed
                        or estimated_reasserted
                    )
                ):
                    raised = self._encoder_markers[axis].raise_above()
                    if not isinstance(raised, bool) or not raised:
                        raise RuntimeError(
                            "visible encoder marker could not "
                            "reassert topmost layering"
                        )
        except Exception as exc:
            self._trajectory = None
            self._started_at = None
            self._actual_joint_positions = None
            self._actual_position_source = None
            try:
                self._hide_markers()
            except Exception as cleanup_exc:
                raise RuntimeError(
                    f"{exc}; marker cleanup failed: {cleanup_exc}"
                ) from exc
            raise
        return estimated_positions

    def finish(self, preserve_actual=False):
        if not isinstance(preserve_actual, bool):
            raise MotionInputError(
                "actual-position preservation state must be boolean"
            )
        self._trajectory = None
        self._started_at = None
        if not preserve_actual:
            self._actual_joint_positions = None
            self._actual_position_source = None
            return self._hide_markers()
        self.refresh()
        return True
