from types import SimpleNamespace
import tkinter as tk
import unittest
from unittest.mock import patch

from ARrobots.HMI.joint_motion import (
    ControllerJointCalibration,
    JointMove,
    MotionInputError,
    MotionProfile,
)
from ARrobots.HMI.joint_visualization import (
    GhostSliderMarker,
    JointMotionVisualization,
    set_joint_slider_positions,
    slider_marker_geometry,
)


class FakeSlider:
    def __init__(self, value=0):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FailingSlider(FakeSlider):
    def set(self, value):
        raise RuntimeError("slider write failed")


class FakeMarker:
    def __init__(self, fail_show=False):
        self.fail_show = fail_show
        self.visible = False
        self.value = None
        self.hide_count = 0

    def show(self, value):
        if self.fail_show:
            raise RuntimeError("marker draw failed")
        self.visible = True
        self.value = value

    def hide(self):
        self.visible = False
        self.hide_count += 1


class FakeMarkerFrame:
    def __init__(self, parent, **options):
        self.parent = parent
        self.options = options
        self.bindings = {}
        self.placements = []
        self.lift_count = 0
        self.hide_count = 0

    def bind(self, sequence, callback):
        self.bindings[sequence] = callback

    def place(self, **geometry):
        self.placements.append(geometry)

    def lift(self):
        self.lift_count += 1

    def place_forget(self):
        self.hide_count += 1


class OverlaySlider(FakeSlider):
    def __init__(self, value=0):
        super().__init__(value)
        self.bindings = {}
        self.global_bindings = {}
        self.generated_events = []

    def bind(self, sequence, callback, add=None):
        if add == "+":
            self.bindings.setdefault(sequence, []).append(callback)
        else:
            self.bindings[sequence] = [callback]

    def bind_all(self, sequence, callback, add=None):
        if add == "+":
            self.global_bindings.setdefault(sequence, []).append(callback)
        else:
            self.global_bindings[sequence] = [callback]

    def event_generate(self, sequence, **coordinates):
        self.generated_events.append((sequence, coordinates))
        event = SimpleNamespace(**coordinates)
        for callback in self.bindings.get(sequence, ()):
            callback(event)
        for callback in self.global_bindings.get(sequence, ()):
            callback(event)

    @staticmethod
    def cget(option):
        return {
            "from": "-100.0",
            "to": "100.0",
        }[option]

    @staticmethod
    def winfo_height():
        return 20

    @staticmethod
    def winfo_reqheight():
        return 20

    @staticmethod
    def winfo_reqwidth():
        return 200

    @staticmethod
    def winfo_rootx():
        return 110

    @staticmethod
    def winfo_rooty():
        return 220

    @staticmethod
    def winfo_width():
        return 200

    @staticmethod
    def winfo_x():
        return 10

    @staticmethod
    def winfo_y():
        return 20


def joint_move():
    calibration = ControllerJointCalibration(
        negative_limits=(100,) * 9,
        positive_limits=(100,) * 9,
        steps_per_unit=(100,) * 9,
    )
    profile = MotionProfile(
        "Sp",
        50,
        20,
        20,
        20,
        "N",
        "000000",
    )
    return JointMove(
        (10, -5, 0, 0, 0, 0, 0, 0, 0),
        profile,
        calibration,
    )


class SliderMarkerGeometryTests(unittest.TestCase):
    def test_geometry_tracks_ascending_and_descending_ranges(self):
        lower = slider_marker_geometry(
            -100,
            -100,
            100,
            10,
            20,
            200,
            20,
        )
        middle = slider_marker_geometry(
            0,
            -100,
            100,
            10,
            20,
            200,
            20,
        )
        upper = slider_marker_geometry(
            100,
            -100,
            100,
            10,
            20,
            200,
            20,
        )
        descending_lower = slider_marker_geometry(
            100,
            100,
            -100,
            10,
            20,
            200,
            20,
        )
        descending_upper = slider_marker_geometry(
            -100,
            100,
            -100,
            10,
            20,
            200,
            20,
        )

        self.assertEqual(lower, (20.0, 30.0, 18))
        self.assertEqual(middle, (110.0, 30.0, 18))
        self.assertEqual(upper, (200.0, 30.0, 18))
        self.assertEqual(descending_lower, lower)
        self.assertEqual(descending_upper, upper)

    def test_geometry_clamps_values_and_centers_degenerate_ranges(self):
        below = slider_marker_geometry(
            -200,
            -100,
            100,
            10,
            20,
            200,
            20,
        )
        above = slider_marker_geometry(
            200,
            -100,
            100,
            10,
            20,
            200,
            20,
        )
        degenerate = slider_marker_geometry(
            7,
            0,
            0,
            10,
            20,
            200,
            20,
        )

        self.assertEqual(below[0], 20.0)
        self.assertEqual(above[0], 200.0)
        self.assertEqual(degenerate[0], 110.0)

        for width, height in ((0, 20), (200, 0), (-1, 20)):
            with self.subTest(width=width, height=height):
                with self.assertRaisesRegex(MotionInputError, "positive"):
                    slider_marker_geometry(
                        0,
                        -100,
                        100,
                        10,
                        20,
                        width,
                        height,
                    )


class GhostSliderMarkerTests(unittest.TestCase):
    def test_real_tk_marker_coexists_with_grid_managed_slider(self):
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            message = str(exc).lower()
            if (
                "no display name" in message
                or "couldn't connect to display" in message
            ):
                self.skipTest("Tk display is unavailable")
            raise
        try:
            root.withdraw()
            parent = tk.Frame(root)
            parent.pack()
            slider = tk.Scale(
                parent,
                from_=-100,
                to=100,
                orient=tk.HORIZONTAL,
            )
            slider.grid(row=0, column=0)
            sibling = tk.Button(parent, text="sibling")
            sibling.grid(row=1, column=0)
            marker = GhostSliderMarker(parent, slider)
            root.geometry("+100000+100000")
            root.deiconify()
            root.update()

            self.assertTrue(marker.show(25))
            root.update_idletasks()

            self.assertEqual(slider.winfo_manager(), "grid")
            self.assertEqual(marker._marker.winfo_manager(), "place")
            slider.event_generate(
                "<ButtonPress-1>",
                x=slider.winfo_width() // 2,
                y=slider.winfo_height() // 2,
            )
            root.update()
            self.assertTrue(slider._ar4_joint_slider_drag_active)
            sibling.event_generate("<ButtonRelease-1>", x=1, y=1)
            root.update()
            self.assertFalse(slider._ar4_joint_slider_drag_active)
            self.assertTrue(marker.hide())
            root.update_idletasks()
            self.assertEqual(marker._marker.winfo_manager(), "")
        finally:
            root.destroy()

    def test_overlay_forwards_pointer_and_skips_unchanged_placement(self):
        slider = OverlaySlider()
        with patch(
            "ARrobots.HMI.joint_visualization.tk.Frame",
            FakeMarkerFrame,
        ):
            marker = GhostSliderMarker(object(), slider)

        self.assertTrue(marker.show(0))
        self.assertFalse(marker.show(0))
        self.assertEqual(
            marker._marker.placements,
            [{
                "x": 110,
                "y": 30,
                "anchor": "center",
                "width": 3,
                "height": 18,
            }],
        )
        self.assertEqual(marker._marker.lift_count, 1)

        event = SimpleNamespace(x_root=130, y_root=235)
        result = marker._marker.bindings["<ButtonPress-1>"](event)

        self.assertEqual(result, "break")
        self.assertEqual(
            slider.generated_events[-1],
            ("<ButtonPress-1>", {"x": 20, "y": 15}),
        )
        self.assertTrue(marker.hide())
        self.assertFalse(marker.hide())
        self.assertEqual(marker._marker.hide_count, 1)

    def test_slider_updates_preserve_an_active_operator_drag(self):
        slider = OverlaySlider(5)
        release_states = []
        slider.bind(
            "<ButtonRelease-1>",
            lambda _event: release_states.append(
                slider._ar4_joint_slider_drag_active
            ),
            add="+",
        )
        with patch(
            "ARrobots.HMI.joint_visualization.tk.Frame",
            FakeMarkerFrame,
        ):
            marker = GhostSliderMarker(object(), slider)
        sliders = [slider] + [FakeSlider() for _ in range(8)]
        pointer = SimpleNamespace(x_root=130, y_root=235)
        marker._marker.bindings["<ButtonPress-1>"](pointer)

        set_joint_slider_positions(sliders, (10,) * 9)

        self.assertEqual(slider.value, 5)
        self.assertEqual(
            tuple(item.value for item in sliders[1:]),
            (10.0,) * 8,
        )

        marker._marker.bindings["<ButtonRelease-1>"](pointer)
        self.assertEqual(release_states, [True])
        set_joint_slider_positions(sliders, (20,) * 9)

        self.assertEqual(
            tuple(item.value for item in sliders),
            (20.0,) * 9,
        )

    def test_global_release_clears_a_stale_drag(self):
        slider = OverlaySlider()
        with patch(
            "ARrobots.HMI.joint_visualization.tk.Frame",
            FakeMarkerFrame,
        ):
            marker = GhostSliderMarker(object(), slider)
        pointer = SimpleNamespace(x_root=130, y_root=235)
        marker._marker.bindings["<ButtonPress-1>"](pointer)
        self.assertTrue(slider._ar4_joint_slider_drag_active)

        for callback in slider.global_bindings["<ButtonRelease-1>"]:
            callback(SimpleNamespace())

        self.assertFalse(slider._ar4_joint_slider_drag_active)

    def test_drag_placeholder_keeps_rollback_indices_aligned(self):
        sliders = [FakeSlider(axis) for axis in range(9)]
        sliders[1]._ar4_joint_slider_drag_active = True
        sliders[3] = FailingSlider(3)

        with self.assertRaisesRegex(RuntimeError, "joint slider"):
            set_joint_slider_positions(sliders, (10,) * 9)

        self.assertEqual(
            tuple(slider.value for slider in sliders),
            tuple(range(9)),
        )

    def test_non_boolean_drag_state_is_rejected(self):
        sliders = [FakeSlider() for _ in range(9)]
        sliders[4]._ar4_joint_slider_drag_active = "active"

        with self.assertRaisesRegex(MotionInputError, "must be boolean"):
            set_joint_slider_positions(sliders, (10,) * 9)


class JointMotionVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.sliders = [FakeSlider() for _ in range(9)]
        self.markers = [FakeMarker() for _ in range(9)]
        self.enabled = {"value": 1}
        self.clock = {"value": 10.0}
        self.visualization = JointMotionVisualization(
            self.sliders,
            self.markers,
            lambda: self.enabled["value"],
            clock=lambda: self.clock["value"],
        )

    def test_start_preserves_newer_desired_target_and_updates_estimate(self):
        queued_target = (20, 4, 3, 2, 1, 0, 6, 7, 8)
        self.visualization.set_desired(queued_target)
        trajectory = self.visualization.start(
            (0,) * 9,
            joint_move(),
            200,
        )

        self.assertTrue(self.visualization.active)
        self.assertEqual(
            tuple(slider.value for slider in self.sliders),
            tuple(float(value) for value in queued_target),
        )
        self.assertEqual(
            tuple(marker.value for marker in self.markers),
            (0.0,) * 9,
        )

        self.clock["value"] += trajectory.duration_seconds / 2
        positions = self.visualization.refresh()

        self.assertAlmostEqual(positions[0], 5.0)
        self.assertAlmostEqual(positions[1], -2.5)
        self.assertEqual(
            tuple(slider.value for slider in self.sliders),
            tuple(float(value) for value in queued_target),
        )

    def test_worker_timestamp_accounts_for_tk_poll_delay(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.visualization.finish()
        started_at = self.clock["value"] - trajectory.duration_seconds / 2

        self.visualization.start(
            (0,) * 9,
            joint_move(),
            200,
            started_at_seconds=started_at,
        )

        self.assertAlmostEqual(self.markers[0].value, 5.0)
        self.assertAlmostEqual(self.markers[1].value, -2.5)

    def test_toggle_hides_without_discarding_active_estimate(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.enabled["value"] = 0

        self.assertIsNone(self.visualization.refresh())
        self.assertTrue(self.visualization.active)
        self.assertFalse(any(marker.visible for marker in self.markers))
        hide_counts = tuple(marker.hide_count for marker in self.markers)
        self.assertIsNone(self.visualization.refresh())
        self.assertEqual(
            tuple(marker.hide_count for marker in self.markers),
            hide_counts,
        )

        self.clock["value"] += trajectory.duration_seconds
        self.enabled["value"] = 1
        positions = self.visualization.refresh()

        self.assertEqual(positions, trajectory.target_positions)
        self.assertTrue(all(marker.visible for marker in self.markers))

    def test_finish_hides_markers_without_changing_queued_target(self):
        queued_target = (20, 4, 3, 2, 1, 0, 6, 7, 8)
        self.visualization.set_desired(queued_target)
        self.visualization.start((0,) * 9, joint_move(), 200)

        self.assertTrue(self.visualization.finish())

        self.assertFalse(self.visualization.active)
        self.assertFalse(any(marker.visible for marker in self.markers))
        self.assertEqual(
            tuple(slider.value for slider in self.sliders),
            tuple(float(value) for value in queued_target),
        )

    def test_desired_slider_failure_rolls_back_completed_writes(self):
        sliders = [FakeSlider(axis) for axis in range(9)]
        sliders[2] = FailingSlider(2)
        visualization = JointMotionVisualization(
            sliders,
            [FakeMarker() for _ in range(9)],
            lambda: 1,
        )

        with self.assertRaisesRegex(RuntimeError, "joint slider"):
            visualization.set_desired((10,) * 9)

        self.assertEqual(sliders[0].value, 0)
        self.assertEqual(sliders[1].value, 1)
        self.assertEqual(sliders[3].value, 3)

    def test_marker_failure_hides_every_partial_overlay(self):
        self.markers[2] = FakeMarker(fail_show=True)
        visualization = JointMotionVisualization(
            self.sliders,
            self.markers,
            lambda: 1,
            clock=lambda: self.clock["value"],
        )

        with self.assertRaisesRegex(RuntimeError, "marker draw failed"):
            visualization.start((0,) * 9, joint_move(), 200)

        self.assertFalse(visualization.active)
        self.assertFalse(any(marker.visible for marker in self.markers))

    def test_invalid_toggle_value_hides_markers_and_disables_estimate(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.enabled["value"] = "enabled"

        with self.assertRaisesRegex(MotionInputError, "zero or one"):
            self.visualization.refresh()

        self.assertFalse(self.visualization.active)
        self.assertFalse(any(marker.visible for marker in self.markers))
        self.assertEqual(trajectory.target_positions[0], 10.0)
