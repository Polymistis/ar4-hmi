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
    ENCODER_MARKER_COLOR,
    ENCODER_MARKER_ROLE,
    ENCODER_MARKER_WIDTH,
    ESTIMATED_MARKER_COLOR,
    ESTIMATED_MARKER_ROLE,
    ESTIMATED_MARKER_WIDTH,
    TARGET_MARKER_COLOR,
    TARGET_MARKER_ROLE,
    TARGET_MARKER_WIDTH,
    GhostSliderMarker,
    JointMotionVisualization,
    _SLIDER_DRAG_ATTRIBUTE,
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
        self.raise_count = 0

    def show(self, value):
        if self.fail_show:
            raise RuntimeError("marker draw failed")
        changed = not self.visible or self.value != value
        self.visible = True
        self.value = value
        return changed

    def hide(self):
        changed = self.visible
        self.visible = False
        self.hide_count += 1
        return changed

    def raise_above(self):
        if not self.visible:
            return False
        self.raise_count += 1
        return True


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
    def __init__(self, value=0, *, global_bindings):
        super().__init__(value)
        self.bindings = {}
        self.global_bindings = global_bindings
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

        self.assertEqual(lower, (20.0, 30.0, 20.0))
        self.assertEqual(middle, (110.0, 30.0, 20.0))
        self.assertEqual(upper, (200.0, 30.0, 20.0))
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
    def test_real_tk_marker_layering_geometry_and_routing(self):
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
            self.assertFalse(root.bind_all("<ButtonRelease-1>"))
            estimated_marker = GhostSliderMarker(
                parent,
                slider,
                ESTIMATED_MARKER_ROLE,
            )
            encoder_marker = GhostSliderMarker(
                parent,
                slider,
                ENCODER_MARKER_ROLE,
            )
            target_marker = GhostSliderMarker(
                parent,
                slider,
                TARGET_MARKER_ROLE,
            )
            # Tk drops synthetic pointer events for withdrawn widgets; a
            # one-pixel override-redirect root keeps a mapped event surface
            # without presenting a normal window.
            root.overrideredirect(True)
            root.geometry("1x1+0+0")
            root.deiconify()
            root.update()

            self.assertTrue(target_marker.show(25))
            self.assertTrue(estimated_marker.show(25))
            self.assertTrue(encoder_marker.show(25))
            root.update()

            self.assertEqual(slider.winfo_manager(), "grid")
            self.assertEqual(
                target_marker._marker.winfo_manager(),
                "place",
            )
            self.assertEqual(
                estimated_marker._marker.winfo_manager(),
                "place",
            )
            self.assertEqual(
                encoder_marker._marker.winfo_manager(),
                "place",
            )
            children = parent.winfo_children()
            self.assertGreater(
                children.index(estimated_marker._marker),
                children.index(target_marker._marker),
            )
            self.assertGreater(
                children.index(encoder_marker._marker),
                children.index(estimated_marker._marker),
            )
            self.assertTrue(target_marker.show(50))
            root.update()
            children = parent.winfo_children()
            self.assertGreater(
                children.index(target_marker._marker),
                children.index(encoder_marker._marker),
            )
            self.assertTrue(estimated_marker.raise_above())
            root.update()
            children = parent.winfo_children()
            self.assertGreater(
                children.index(estimated_marker._marker),
                children.index(target_marker._marker),
            )
            self.assertTrue(encoder_marker.raise_above())
            root.update()
            children = parent.winfo_children()
            self.assertGreater(
                children.index(encoder_marker._marker),
                children.index(estimated_marker._marker),
            )
            self.assertTrue(root.bind_all("<ButtonRelease-1>"))
            setattr(slider, _SLIDER_DRAG_ATTRIBUTE, True)
            sibling.event_generate("<ButtonRelease-1>", x=0, y=0)
            root.update()
            self.assertFalse(
                getattr(slider, _SLIDER_DRAG_ATTRIBUTE)
            )

            root.unbind_all("<ButtonRelease-1>")
            forwarded_events = []
            slider.bind(
                "<ButtonPress-1>",
                lambda event: forwarded_events.append(
                    ("press", event.x, event.y)
                ),
                add="+",
            )
            slider.bind(
                "<B1-Motion>",
                lambda event: forwarded_events.append(
                    ("motion", event.x, event.y)
                ),
                add="+",
            )
            slider.bind(
                "<ButtonRelease-1>",
                lambda event: forwarded_events.append(
                    ("release", event.x, event.y)
                ),
                add="+",
            )
            event_x = 1
            event_y = 1
            expected_x = (
                estimated_marker._marker.winfo_rootx()
                + event_x
                - slider.winfo_rootx()
            )
            expected_y = (
                estimated_marker._marker.winfo_rooty()
                + event_y
                - slider.winfo_rooty()
            )
            estimated_marker._marker.event_generate(
                "<ButtonPress-1>",
                x=event_x,
                y=event_y,
            )
            root.update()
            self.assertEqual(
                forwarded_events,
                [("press", expected_x, expected_y)],
            )
            self.assertTrue(
                getattr(slider, _SLIDER_DRAG_ATTRIBUTE)
            )
            motion_x = 2
            motion_y = 2
            expected_motion_x = (
                estimated_marker._marker.winfo_rootx()
                + motion_x
                - slider.winfo_rootx()
            )
            expected_motion_y = (
                estimated_marker._marker.winfo_rooty()
                + motion_y
                - slider.winfo_rooty()
            )
            estimated_marker._marker.event_generate(
                "<B1-Motion>",
                x=motion_x,
                y=motion_y,
            )
            root.update()
            self.assertEqual(
                forwarded_events,
                [
                    ("press", expected_x, expected_y),
                    (
                        "motion",
                        expected_motion_x,
                        expected_motion_y,
                    ),
                ],
            )
            estimated_marker._marker.event_generate(
                "<ButtonRelease-1>",
                x=event_x,
                y=event_y,
            )
            root.update()
            self.assertEqual(
                forwarded_events,
                [
                    ("press", expected_x, expected_y),
                    (
                        "motion",
                        expected_motion_x,
                        expected_motion_y,
                    ),
                    ("release", expected_x, expected_y),
                ],
            )
            self.assertFalse(
                getattr(slider, _SLIDER_DRAG_ATTRIBUTE)
            )
            self.assertTrue(target_marker.hide())
            self.assertTrue(estimated_marker.hide())
            self.assertTrue(encoder_marker.hide())
            root.update_idletasks()
            self.assertEqual(
                target_marker._marker.winfo_manager(),
                "",
            )
            self.assertEqual(
                estimated_marker._marker.winfo_manager(),
                "",
            )
            self.assertEqual(
                encoder_marker._marker.winfo_manager(),
                "",
            )
        finally:
            root.destroy()

    def test_overlay_forwards_pointer_and_skips_unchanged_placement(self):
        slider = OverlaySlider(global_bindings={})
        with patch(
            "ARrobots.HMI.joint_visualization.tk.Frame",
            FakeMarkerFrame,
        ):
            marker = GhostSliderMarker(
                object(),
                slider,
                ESTIMATED_MARKER_ROLE,
            )

        self.assertTrue(marker.show(0))
        self.assertFalse(marker.show(0))
        self.assertEqual(
            marker._marker.placements,
            [{
                "x": 110,
                "y": 30,
                "anchor": "center",
                "width": ESTIMATED_MARKER_WIDTH,
                "height": 20,
            }],
        )
        self.assertEqual(marker._marker.lift_count, 1)
        self.assertEqual(
            marker._marker.options["background"],
            ESTIMATED_MARKER_COLOR,
        )

        event = SimpleNamespace(x_root=130, y_root=235)
        results = [
            marker._marker.bindings[sequence](event)
            for sequence in (
                "<ButtonPress-1>",
                "<B1-Motion>",
                "<ButtonRelease-1>",
            )
        ]

        self.assertEqual(results, ["break", "break", "break"])
        self.assertEqual(
            slider.generated_events[-3:],
            [
                ("<ButtonPress-1>", {"x": 20, "y": 15}),
                ("<B1-Motion>", {"x": 20, "y": 15}),
                ("<ButtonRelease-1>", {"x": 20, "y": 15}),
            ],
        )
        self.assertTrue(marker.hide())
        self.assertFalse(marker.hide())
        self.assertEqual(marker._marker.hide_count, 1)

    def test_marker_roles_are_distinct_and_share_slider_bindings(self):
        global_bindings = {}
        slider = OverlaySlider(global_bindings=global_bindings)
        with patch(
            "ARrobots.HMI.joint_visualization.tk.Frame",
            FakeMarkerFrame,
        ):
            estimated = GhostSliderMarker(
                object(),
                slider,
                ESTIMATED_MARKER_ROLE,
            )
            encoder = GhostSliderMarker(
                object(),
                slider,
                ENCODER_MARKER_ROLE,
            )
            target = GhostSliderMarker(
                object(),
                slider,
                TARGET_MARKER_ROLE,
            )

        self.assertEqual(
            estimated._marker.options["background"],
            ESTIMATED_MARKER_COLOR,
        )
        self.assertEqual(
            encoder._marker.options["background"],
            ENCODER_MARKER_COLOR,
        )
        self.assertEqual(
            target._marker.options["background"],
            TARGET_MARKER_COLOR,
        )
        self.assertTrue(target.show(0))
        self.assertTrue(encoder.show(0))
        self.assertTrue(estimated.show(0))
        self.assertEqual(
            target._marker.placements[-1]["width"],
            TARGET_MARKER_WIDTH,
        )
        self.assertEqual(
            target._marker.placements[-1]["height"],
            5,
        )
        self.assertEqual(
            encoder._marker.placements[-1]["width"],
            ENCODER_MARKER_WIDTH,
        )
        self.assertEqual(
            encoder._marker.placements[-1]["height"],
            12,
        )
        self.assertEqual(
            estimated._marker.placements[-1]["width"],
            ESTIMATED_MARKER_WIDTH,
        )
        self.assertEqual(
            estimated._marker.placements[-1]["height"],
            20,
        )
        self.assertTrue(target.raise_above())
        self.assertTrue(encoder.raise_above())
        self.assertTrue(estimated.raise_above())
        self.assertEqual(target._marker.lift_count, 2)
        self.assertEqual(encoder._marker.lift_count, 2)
        self.assertEqual(estimated._marker.lift_count, 2)
        self.assertEqual(
            len(slider.bindings["<ButtonPress-1>"]),
            1,
        )
        self.assertEqual(
            len(slider.bindings["<ButtonRelease-1>"]),
            1,
        )
        self.assertEqual(
            len(global_bindings["<ButtonRelease-1>"]),
            1,
        )

    def test_slider_updates_preserve_an_active_operator_drag(self):
        slider = OverlaySlider(5, global_bindings={})
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
            marker = GhostSliderMarker(
                object(),
                slider,
                ESTIMATED_MARKER_ROLE,
            )
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
        global_bindings = {}
        slider = OverlaySlider(global_bindings=global_bindings)
        sibling = OverlaySlider(global_bindings=global_bindings)
        with patch(
            "ARrobots.HMI.joint_visualization.tk.Frame",
            FakeMarkerFrame,
        ):
            marker = GhostSliderMarker(
                object(),
                slider,
                ESTIMATED_MARKER_ROLE,
            )
        pointer = SimpleNamespace(x_root=130, y_root=235)
        marker._marker.bindings["<ButtonPress-1>"](pointer)
        self.assertTrue(slider._ar4_joint_slider_drag_active)

        sibling.event_generate("<ButtonRelease-1>", x=0, y=0)

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
        self.target_markers = [FakeMarker() for _ in range(9)]
        self.estimated_markers = [FakeMarker() for _ in range(9)]
        self.encoder_markers = [FakeMarker() for _ in range(6)]
        self.estimated_enabled = {"value": 1}
        self.encoder_enabled = {"value": 1}
        self.actual_source = {"value": object()}
        self.clock = {"value": 10.0}
        self.visualization = JointMotionVisualization(
            self.sliders,
            self.target_markers,
            self.estimated_markers,
            self.encoder_markers,
            lambda: self.estimated_enabled["value"],
            lambda: self.encoder_enabled["value"],
            lambda: self.actual_source["value"],
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
            tuple(marker.value for marker in self.estimated_markers),
            (0.0,) * 9,
        )
        self.assertEqual(
            tuple(marker.value for marker in self.target_markers),
            trajectory.target_positions,
        )
        self.assertTrue(
            all(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

        later_target = (30, 8, 6, 4, 2, 0, 7, 8, 9)
        self.visualization.set_desired(later_target)
        self.clock["value"] += trajectory.duration_seconds / 2
        positions = self.visualization.refresh()

        self.assertAlmostEqual(positions[0], 5.0)
        self.assertAlmostEqual(positions[1], -2.5)
        self.assertEqual(
            tuple(slider.value for slider in self.sliders),
            tuple(float(value) for value in later_target),
        )
        self.assertEqual(
            tuple(marker.value for marker in self.target_markers),
            trajectory.target_positions,
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

        self.assertAlmostEqual(
            self.estimated_markers[0].value,
            5.0,
        )
        self.assertAlmostEqual(
            self.estimated_markers[1].value,
            -2.5,
        )

    def test_encoder_telemetry_remains_distinct_from_estimates(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.clock["value"] += trajectory.duration_seconds / 2
        estimated = trajectory.positions_at(trajectory.duration_seconds / 2)

        self.assertTrue(
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        )
        positions = self.visualization.refresh()

        for observed, expected in zip(positions, estimated):
            self.assertAlmostEqual(observed, expected)
        for marker, expected in zip(
            self.estimated_markers,
            estimated,
        ):
            self.assertAlmostEqual(marker.value, expected)
        self.assertEqual(
            tuple(marker.value for marker in self.encoder_markers),
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        )
        self.assertTrue(
            all(marker.visible for marker in self.encoder_markers)
        )

    def test_actual_marker_reasserts_topmost_layer_after_estimate_moves(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.clock["value"] += trajectory.duration_seconds / 2
        self.assertTrue(
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        )

        self.visualization.refresh()

        self.assertEqual(
            tuple(
                marker.raise_count for marker in self.encoder_markers
            ),
            (0,) * 6,
        )
        self.visualization.refresh()
        self.assertEqual(
            tuple(
                marker.raise_count for marker in self.encoder_markers
            ),
            (0,) * 6,
        )

        self.encoder_enabled["value"] = 0
        self.visualization.refresh()
        self.encoder_enabled["value"] = 1
        self.visualization.refresh()
        self.assertEqual(
            tuple(
                marker.raise_count for marker in self.encoder_markers
            ),
            (0,) * 6,
        )

        self.clock["value"] += trajectory.duration_seconds / 10
        self.visualization.refresh()
        self.assertEqual(
            tuple(
                marker.raise_count for marker in self.encoder_markers
            ),
            (1, 1, 0, 0, 0, 0),
        )

    def test_estimate_reasserts_above_a_restored_target_marker(self):
        self.visualization.start((0,) * 9, joint_move(), 200)
        self.assertTrue(
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        )
        self.assertTrue(self.target_markers[0].hide())

        self.visualization.refresh()

        self.assertTrue(self.target_markers[0].visible)
        self.assertEqual(
            tuple(
                marker.raise_count
                for marker in self.estimated_markers
            ),
            (1, 0, 0, 0, 0, 0, 0, 0, 0),
        )
        self.assertEqual(
            tuple(
                marker.raise_count
                for marker in self.encoder_markers
            ),
            (1, 0, 0, 0, 0, 0),
        )

    def test_actual_telemetry_requires_an_active_six_axis_sample(self):
        self.assertFalse(
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        )
        for positions in (
            (1, 2, 3, 4, 5),
            (1, 2, 3, 4, 5, float("nan")),
        ):
            with self.subTest(positions=positions):
                with self.assertRaises(MotionInputError):
                    self.visualization.observe_actual(positions)

    def test_actual_telemetry_requires_a_trusted_source(self):
        self.visualization.start((0,) * 9, joint_move(), 200)
        self.actual_source["value"] = None

        with self.assertRaisesRegex(RuntimeError, "source is unavailable"):
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))

        self.assertTrue(self.visualization.active)
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

    def test_new_motion_preserves_latest_validated_actual_sample(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.assertTrue(
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        )
        self.assertTrue(self.visualization.finish(True))
        self.clock["value"] += 1

        second = self.visualization.start((0,) * 9, joint_move(), 200)
        self.clock["value"] += second.duration_seconds / 2
        positions = self.visualization.refresh()

        self.assertAlmostEqual(positions[0], 5.0)
        self.assertAlmostEqual(positions[1], -2.5)
        self.assertTrue(
            all(marker.visible for marker in self.encoder_markers)
        )
        self.assertEqual(
            tuple(marker.value for marker in self.encoder_markers),
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        )
        self.assertNotEqual(trajectory.duration_seconds, 0)

    def test_idle_actual_sample_persists_and_respects_toggle(self):
        actual = (1, 2, 3, 4, 5, 6)

        self.visualization.start((0,) * 9, joint_move(), 200)
        self.assertTrue(self.visualization.observe_actual(actual))
        self.assertTrue(self.visualization.finish(True))

        self.assertFalse(self.visualization.active)
        self.assertTrue(
            all(marker.visible for marker in self.encoder_markers)
        )
        self.assertEqual(
            tuple(marker.value for marker in self.encoder_markers),
            tuple(float(value) for value in actual),
        )
        self.assertFalse(
            any(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )

        self.encoder_enabled["value"] = 0
        self.visualization.refresh()
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

        self.encoder_enabled["value"] = 1
        self.visualization.refresh()
        self.assertTrue(
            all(marker.visible for marker in self.encoder_markers)
        )

    def test_actual_sample_clears_on_trust_or_identity_loss(self):
        actual = (1, 2, 3, 4, 5, 6)
        original_source = self.actual_source["value"]
        self.visualization.start((0,) * 9, joint_move(), 200)
        self.visualization.observe_actual(actual)
        self.visualization.finish(True)

        self.actual_source["value"] = None
        self.visualization.refresh()
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

        self.actual_source["value"] = original_source
        self.visualization.refresh()
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

        self.visualization.start((0,) * 9, joint_move(), 200)
        self.visualization.observe_actual(actual)
        self.visualization.finish(True)
        self.actual_source["value"] = object()
        self.visualization.refresh()
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

    def test_finish_can_preserve_or_clear_latest_actual_position(self):
        self.visualization.start((0,) * 9, joint_move(), 200)
        self.visualization.observe_actual((1, 2, 3, 4, 5, 6))

        self.assertTrue(self.visualization.finish(True))
        self.assertTrue(
            all(marker.visible for marker in self.encoder_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )

        self.assertTrue(self.visualization.finish(False))
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

    def test_clear_actual_hides_only_the_obsolete_encoder_sample(self):
        self.visualization.start(
            (0,) * 9,
            joint_move(),
            200,
        )
        self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        self.visualization.refresh()

        self.assertIsNone(self.visualization.clear_actual())
        self.assertTrue(self.visualization.active)
        self.assertFalse(any(marker.visible for marker in self.encoder_markers))
        self.assertTrue(all(marker.visible for marker in self.target_markers))
        self.assertTrue(
            self.visualization.observe_actual((6, 5, 4, 3, 2, 1))
        )
        self.visualization.refresh()
        self.assertTrue(
            all(marker.visible for marker in self.encoder_markers)
        )

    def test_finish_rejects_non_boolean_preservation_before_mutation(self):
        self.visualization.start((0,) * 9, joint_move(), 200)

        with self.assertRaisesRegex(MotionInputError, "must be boolean"):
            self.visualization.finish(1)

        self.assertTrue(self.visualization.active)
        self.assertTrue(
            all(marker.visible for marker in self.target_markers)
        )

    def test_toggle_hides_without_discarding_active_estimate(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.estimated_enabled["value"] = 0

        self.assertIsNotNone(self.visualization.refresh())
        self.assertTrue(self.visualization.active)
        self.assertTrue(
            all(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )
        hide_counts = tuple(
            marker.hide_count for marker in self.estimated_markers
        )
        self.assertIsNotNone(self.visualization.refresh())
        self.assertEqual(
            tuple(
                marker.hide_count
                for marker in self.estimated_markers
            ),
            hide_counts,
        )

        self.clock["value"] += trajectory.duration_seconds
        self.estimated_enabled["value"] = 1
        positions = self.visualization.refresh()

        self.assertEqual(positions, trajectory.target_positions)
        self.assertTrue(
            all(marker.visible for marker in self.estimated_markers)
        )

    def test_marker_channels_can_be_selected_independently(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.clock["value"] += trajectory.duration_seconds / 2
        self.assertTrue(
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        )
        self.visualization.refresh()

        self.encoder_enabled["value"] = 0
        positions = self.visualization.refresh()

        self.assertIsNotNone(positions)
        self.assertTrue(
            all(marker.visible for marker in self.estimated_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

        self.encoder_enabled["value"] = 1
        self.estimated_enabled["value"] = 0
        positions = self.visualization.refresh()

        self.assertIsNotNone(positions)
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )
        self.assertTrue(
            all(marker.visible for marker in self.encoder_markers)
        )
        self.assertEqual(
            tuple(marker.value for marker in self.encoder_markers),
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        )

    def test_finish_hides_markers_without_changing_queued_target(self):
        queued_target = (20, 4, 3, 2, 1, 0, 6, 7, 8)
        self.visualization.set_desired(queued_target)
        self.visualization.start((0,) * 9, joint_move(), 200)

        self.assertTrue(self.visualization.finish())

        self.assertFalse(self.visualization.active)
        self.assertFalse(
            any(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )
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
            [FakeMarker() for _ in range(9)],
            [FakeMarker() for _ in range(6)],
            lambda: 1,
            lambda: 1,
            object,
        )

        with self.assertRaisesRegex(RuntimeError, "joint slider"):
            visualization.set_desired((10,) * 9)

        self.assertEqual(sliders[0].value, 0)
        self.assertEqual(sliders[1].value, 1)
        self.assertEqual(sliders[3].value, 3)

    def test_marker_failure_hides_every_partial_overlay(self):
        self.estimated_markers[2] = FakeMarker(fail_show=True)
        visualization = JointMotionVisualization(
            self.sliders,
            self.target_markers,
            self.estimated_markers,
            self.encoder_markers,
            lambda: 1,
            lambda: 1,
            object,
            clock=lambda: self.clock["value"],
        )

        with self.assertRaisesRegex(RuntimeError, "marker draw failed"):
            visualization.start((0,) * 9, joint_move(), 200)

        self.assertFalse(visualization.active)
        self.assertFalse(
            any(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )

    def test_encoder_failure_hides_every_partial_overlay(self):
        self.visualization.start((0,) * 9, joint_move(), 200)
        self.assertTrue(
            self.visualization.observe_actual((1, 2, 3, 4, 5, 6))
        )
        self.encoder_markers[2].fail_show = True

        with self.assertRaisesRegex(RuntimeError, "marker draw failed"):
            self.visualization.refresh()

        self.assertFalse(self.visualization.active)
        self.assertFalse(
            any(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )
        self.assertEqual(
            tuple(
                marker.hide_count
                for marker in self.encoder_markers
            ),
            (1,) * 6,
        )

    def test_invalid_toggle_value_hides_markers_and_disables_estimate(self):
        trajectory = self.visualization.start((0,) * 9, joint_move(), 200)
        self.estimated_enabled["value"] = "enabled"

        with self.assertRaisesRegex(MotionInputError, "zero or one"):
            self.visualization.refresh()

        self.assertFalse(self.visualization.active)
        self.assertFalse(
            any(marker.visible for marker in self.target_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.estimated_markers)
        )
        self.assertFalse(
            any(marker.visible for marker in self.encoder_markers)
        )
        self.assertEqual(trajectory.target_positions[0], 10.0)
