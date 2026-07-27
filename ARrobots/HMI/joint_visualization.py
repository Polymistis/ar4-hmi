"""Tk-thread presentation for desired and estimated joint positions."""

import tkinter as tk
import time

from ARrobots.HMI.joint_motion import (
    JOINT_COUNT,
    JOINT_TELEMETRY_AXIS_COUNT,
    MotionInputError,
    estimate_commanded_joint_trajectory,
    finite_number,
)


GHOST_MARKER_COLOR = "#00D7FF"
GHOST_MARKER_OUTLINE_COLOR = "#003B49"
GHOST_MARKER_WIDTH = 7
GHOST_MARKER_MINIMUM_HEIGHT = 12
GHOST_MARKER_MAXIMUM_HEIGHT = 24
_SLIDER_DRAG_ATTRIBUTE = "_ar4_joint_slider_drag_active"


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
    """High-contrast overlay marker that leaves the slider thumb visible."""

    def __init__(self, parent, slider, color=GHOST_MARKER_COLOR):
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
        if not isinstance(color, str) or not color:
            raise TypeError("ghost marker color must be non-empty text")

        self._slider = slider
        setattr(self._slider, _SLIDER_DRAG_ATTRIBUTE, False)
        self._placement = None
        self._visible = False
        self._marker = tk.Frame(
            parent,
            background=color,
            borderwidth=0,
            highlightbackground=GHOST_MARKER_OUTLINE_COLOR,
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
            GHOST_MARKER_WIDTH,
            round(marker_height),
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


class JointMotionVisualization:
    """Coordinate target thumbs with actual or estimated in-flight markers."""

    def __init__(
        self,
        sliders,
        markers,
        enabled_provider,
        clock=time.monotonic,
    ):
        self._sliders = _fixed_items(
            sliders,
            JOINT_COUNT,
            "joint visualization sliders",
        )
        self._markers = _fixed_items(
            markers,
            JOINT_COUNT,
            "joint visualization markers",
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
            for marker in self._markers
        ):
            raise TypeError("joint visualization marker contract is invalid")
        if not callable(enabled_provider):
            raise TypeError("joint visualization enabled provider must be callable")
        if not callable(clock):
            raise TypeError("joint visualization clock must be callable")

        self._enabled_provider = enabled_provider
        self._clock = clock
        self._trajectory = None
        self._started_at = None
        self._actual_joint_positions = None
        self._markers_visible = False

    @property
    def active(self):
        return self._trajectory is not None

    def _enabled(self):
        enabled = self._enabled_provider()
        if isinstance(enabled, bool):
            return enabled
        if isinstance(enabled, int) and enabled in (0, 1):
            return enabled == 1
        raise MotionInputError(
            "joint visualization toggle must contain zero or one"
        )

    def _hide_markers(self):
        if not self._markers_visible:
            return True
        failures = []
        for axis, marker in enumerate(self._markers, start=1):
            try:
                marker.hide()
            except Exception as exc:
                failures.append(f"J{axis}: {exc}")
        if failures:
            raise RuntimeError(
                "unable to hide estimated joint markers: "
                + "; ".join(failures)
            )
        self._markers_visible = False
        return True

    def set_desired(self, positions):
        return set_joint_slider_positions(self._sliders, positions)

    def observe_actual(self, positions):
        values = _fixed_items(
            positions,
            JOINT_TELEMETRY_AXIS_COUNT,
            "actual joint positions",
        )
        normalized = tuple(
            finite_number(value, f"actual joint positions[{axis}]")
            for axis, value in enumerate(values, start=1)
        )
        if self._trajectory is None:
            return False
        self._actual_joint_positions = normalized
        return True

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
        self._actual_joint_positions = None
        try:
            self.refresh()
        except Exception as exc:
            self._trajectory = None
            self._started_at = None
            self._actual_joint_positions = None
            try:
                self._hide_markers()
            except Exception as cleanup_exc:
                raise RuntimeError(
                    f"{exc}; marker cleanup failed: {cleanup_exc}"
                ) from exc
            raise
        return trajectory

    def refresh(self):
        if self._trajectory is None:
            self._hide_markers()
            return None
        try:
            enabled = self._enabled()
            if not enabled:
                self._hide_markers()
                return None

            now = finite_number(self._clock(), "joint visualization clock")
            elapsed = max(0.0, now - self._started_at)
            positions = self._trajectory.positions_at(elapsed)
            if self._actual_joint_positions is not None:
                positions = (
                    self._actual_joint_positions
                    + positions[JOINT_TELEMETRY_AXIS_COUNT:]
                )
            self._markers_visible = True
            for marker, position in zip(self._markers, positions):
                marker.show(position)
        except Exception as exc:
            self._trajectory = None
            self._started_at = None
            self._actual_joint_positions = None
            try:
                self._hide_markers()
            except Exception as cleanup_exc:
                raise RuntimeError(
                    f"{exc}; marker cleanup failed: {cleanup_exc}"
                ) from exc
            raise
        return positions

    def finish(self):
        self._trajectory = None
        self._started_at = None
        self._actual_joint_positions = None
        return self._hide_markers()
