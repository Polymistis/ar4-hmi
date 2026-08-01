from pathlib import Path
from queue import Queue
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from PIL import Image

if __package__:
    from .bounded_temp import BoundedTemporaryDirectory
else:
    from bounded_temp import BoundedTemporaryDirectory

from ARrobots.HMI.joint_motion import MotionInputError
from ARrobots.HMI.vision_io import (
    MAX_CAMERA_PREVIEW_EVENT_DETAIL,
    VISION_SELECTION_TOO_SMALL_MESSAGE,
    CameraPreviewFrame,
    CameraPreviewLifecycleState,
    CameraPreviewWorker,
    VisionCaptureResult,
    VisionCaptureSettings,
    VisionCoordinateMapping,
    VisionMatchOperationResult,
    VisionMatchOptions,
    VisionMatchResult,
    VisionMatchSettings,
    VisionOperationDrainState,
    VisionOperationWorker,
    VisionSelectionResult,
    VisionSelectionSettings,
    _rotate_vision_template,
    fit_vision_preview_square,
    load_bounded_vision_image,
    normalize_vision_selection_bounds,
    normalize_camera_exception_detail,
    prepare_camera_preview_frame,
    prepare_vision_capture_result,
    prepare_vision_mask_selection_result,
    prepare_vision_match_result,
    prepare_vision_selection_image,
    prepare_vision_template_selection_result,
    select_vision_region,
)


class QueuedCapture:
    def __init__(self):
        self.responses = Queue()
        self.read_threads = []
        self.release_threads = []
        self.release_count = 0
        self.opened = True

    def isOpened(self):
        return self.opened

    def read(self):
        self.read_threads.append(threading.get_ident())
        return self.responses.get(timeout=2)

    def release(self):
        self.release_threads.append(threading.get_ident())
        self.release_count += 1
        self.opened = False


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


class VisionIoTests(unittest.TestCase):
    @staticmethod
    def vision_capture_settings(brightness=0):
        return VisionCaptureSettings(
            brightness=brightness,
            contrast=0,
            zoom_percent=50,
            mask_bounds=(0, 0, 6, 4),
            auto_background=False,
            persist_auto_background=True,
            background_grayscale=17,
            sample_points=(),
        )

    @staticmethod
    def vision_capture_worker(operation, **kwargs):
        return VisionOperationWorker(
            operation,
            VisionCaptureSettings,
            VisionCaptureResult,
            "vision capture",
            "ar4-vision-capture-test",
            **kwargs,
        )

    @staticmethod
    def vision_coordinate_mapping(pixel_extent=48):
        return VisionCoordinateMapping(
            first_pixel_origin=0.0,
            first_robot_origin=0.0,
            second_pixel_origin=0.0,
            second_robot_origin=0.0,
            first_pixel_end=float(pixel_extent),
            first_robot_end=float(pixel_extent * 10),
            second_pixel_end=float(pixel_extent),
            second_robot_end=float(pixel_extent * 10),
        )

    def vision_match_options(self, **overrides):
        values = {
            "template_filename": "template.jpg",
            "minimum_score": 0.99,
            "full_rotation_search": True,
            "pick_closest_180": False,
            "try_closest_out_of_range": False,
            "joint6_positive_limit": 180.0,
            "joint6_negative_limit": 180.0,
            "coordinate_mapping": self.vision_coordinate_mapping(),
        }
        values.update(overrides)
        return VisionMatchOptions(**values)

    def test_vision_capture_processing_validates_and_masks_owned_results(self):
        frame = np.zeros((4, 6, 3), dtype=np.uint8)
        frame[:, :] = (10, 20, 30)
        settings = self.vision_capture_settings()

        result = prepare_vision_capture_result(frame, settings)

        expected_gray = int(
            cv2.cvtColor(
                np.asarray([[[10, 20, 30]]], dtype=np.uint8),
                cv2.COLOR_BGR2GRAY,
            )[0, 0]
        )
        self.assertIsInstance(result, VisionCaptureResult)
        self.assertEqual(result.image.shape, (4, 6))
        self.assertTrue(np.all(result.image[0, :] == 17))
        self.assertTrue(np.all(result.image[:, 0] == 17))
        self.assertTrue(np.all(result.image[1:, 1:] == expected_gray))
        self.assertEqual(result.display_image.shape, (480, 640))
        self.assertFalse(result.image.flags.writeable)
        self.assertFalse(result.display_image.flags.writeable)
        self.assertTrue(result.persist_auto_background)
        self.assertIsNone(result.auto_background_rgb)

        automatic = VisionCaptureSettings(
            brightness=0,
            contrast=0,
            zoom_percent=50,
            mask_bounds=(0, 0, 6, 4),
            auto_background=True,
            persist_auto_background=False,
            background_grayscale=None,
            sample_points=((1, 1), (1, 2), (2, 1)),
        )
        automatic_result = prepare_vision_capture_result(frame, automatic)
        self.assertEqual(
            automatic_result.auto_background_rgb,
            (expected_gray,) * 3,
        )
        self.assertFalse(automatic_result.persist_auto_background)

        with self.assertRaisesRegex(MotionInputError, "enclose an area"):
            VisionCaptureSettings(
                brightness=0,
                contrast=0,
                zoom_percent=50,
                mask_bounds=(1, 0, 1, 4),
                auto_background=False,
                persist_auto_background=True,
                background_grayscale=0,
                sample_points=(),
            )
        with self.assertRaisesRegex(MotionInputError, "enclose an area"):
            VisionCaptureSettings(
                brightness=0,
                contrast=0,
                zoom_percent=50,
                mask_bounds=(1, 0, 2, 4),
                auto_background=False,
                persist_auto_background=True,
                background_grayscale=0,
                sample_points=(),
            )
        with self.assertRaisesRegex(MotionInputError, "outside"):
            prepare_vision_capture_result(
                frame,
                VisionCaptureSettings(
                    brightness=0,
                    contrast=0,
                    zoom_percent=50,
                    mask_bounds=(0, 0, 6, 4),
                    auto_background=True,
                    persist_auto_background=True,
                    background_grayscale=None,
                    sample_points=((4, 0), (1, 1), (2, 2)),
                ),
            )

        oversized = np.lib.stride_tricks.as_strided(
            np.zeros((1, 1, 3), dtype=np.uint8),
            shape=(4097, 4097, 3),
            strides=(0, 0, 0),
        )
        with self.assertRaisesRegex(MotionInputError, "pixel count"):
            prepare_vision_capture_result(oversized, settings)

    def test_vision_selection_processing_uses_validated_owned_results(self):
        frame = np.zeros((12, 16, 3), dtype=np.uint8)
        frame[:, :] = (10, 20, 30)
        capture_settings = VisionCaptureSettings(
            brightness=0,
            contrast=0,
            zoom_percent=50,
            mask_bounds=(0, 0, 16, 12),
            auto_background=False,
            persist_auto_background=True,
            background_grayscale=17,
            sample_points=(),
        )
        mask_settings = VisionSelectionSettings(
            kind="mask",
            capture_settings=capture_settings,
        )

        selection_image = prepare_vision_selection_image(
            frame,
            capture_settings,
        )
        self.assertEqual(selection_image.shape, (12, 16, 3))
        self.assertFalse(selection_image.flags.writeable)
        np.testing.assert_array_equal(
            selection_image[:, :, 0],
            selection_image[:, :, 1],
        )
        self.assertEqual(
            normalize_vision_selection_bounds(
                selection_image,
                15,
                11,
                0,
                0,
            ),
            (3, 3, 12, 8),
        )
        self.assertEqual(
            normalize_vision_selection_bounds(
                selection_image,
                -10,
                -10,
                30,
                30,
            ),
            (0, 0, 16, 12),
        )
        self.assertIsNone(
            normalize_vision_selection_bounds(
                selection_image,
                1,
                1,
                6,
                6,
            )
        )

        mask_result = prepare_vision_mask_selection_result(
            frame,
            mask_settings,
            (3, 3, 12, 8),
        )
        self.assertIsInstance(mask_result, VisionSelectionResult)
        self.assertEqual(mask_result.kind, "mask")
        self.assertEqual(mask_result.mask_bounds, (3, 3, 12, 8))
        self.assertTrue(np.all(mask_result.capture_result.image[0, :] == 17))
        self.assertFalse(mask_result.capture_result.image.flags.writeable)

        template_settings = VisionSelectionSettings(
            kind="template",
            template_filename="part.jpg",
        )
        template_result = prepare_vision_template_selection_result(
            frame,
            template_settings,
            (3, 3, 12, 8),
            ("part.jpg",),
        )
        self.assertEqual(template_result.kind, "template")
        self.assertEqual(template_result.template_image.shape, (5, 9, 3))
        self.assertEqual(template_result.template_preview.shape, (150, 150, 3))
        self.assertFalse(template_result.template_image.flags.writeable)
        self.assertFalse(template_result.template_preview.flags.writeable)

        with self.assertRaisesRegex(MotionInputError, "kind"):
            VisionSelectionSettings(kind="unknown")
        with self.assertRaisesRegex(MotionInputError, "kind"):
            VisionSelectionSettings(kind=[])
        with self.assertRaisesRegex(MotionInputError, "cannot include"):
            VisionSelectionSettings(
                kind="mask",
                capture_settings=capture_settings,
                template_filename="part.jpg",
            )
        with self.assertRaisesRegex(MotionInputError, "cannot include"):
            VisionSelectionSettings(
                kind="template",
                capture_settings=capture_settings,
                template_filename="part.jpg",
            )
        with self.assertRaisesRegex(MotionInputError, "lowercase .jpg"):
            VisionSelectionSettings(
                kind="template",
                template_filename="../part.jpg",
            )
        for filename in ("curImage.jpg", "CON.jpg", "part:name.jpg"):
            with self.subTest(filename=filename):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "invalid or reserved",
                ):
                    VisionSelectionSettings(
                        kind="template",
                        template_filename=filename,
                    )
        with self.assertRaisesRegex(MotionInputError, "options are invalid"):
            VisionSelectionResult(
                kind="mask",
                settings=mask_settings,
                capture_result=mask_result.capture_result,
                mask_bounds=mask_result.mask_bounds,
                visual_options=[],
            )
        with self.assertRaisesRegex(MotionInputError, "template options"):
            VisionSelectionResult(
                kind="mask",
                settings=mask_settings,
                capture_result=mask_result.capture_result,
                mask_bounds=mask_result.mask_bounds,
                visual_options=("part.jpg",),
            )
        with self.assertRaisesRegex(MotionInputError, "contains mask data"):
            VisionSelectionResult(
                kind="template",
                settings=template_settings,
                capture_result=mask_result.capture_result,
                template_filename="part.jpg",
                template_image=template_result.template_image,
                template_preview=template_result.template_preview,
                visual_options=("part.jpg",),
            )
        with self.assertRaisesRegex(MotionInputError, "preview dimensions"):
            VisionSelectionResult(
                kind="template",
                settings=template_settings,
                template_filename="part.jpg",
                template_image=template_result.template_image,
                template_preview=np.zeros((10, 10, 3), dtype=np.uint8),
                visual_options=("part.jpg",),
            )
        with self.assertRaisesRegex(MotionInputError, "result settings"):
            VisionSelectionResult(
                kind="mask",
                settings=template_settings,
                capture_result=mask_result.capture_result,
                mask_bounds=mask_result.mask_bounds,
            )
        with self.assertRaisesRegex(MotionInputError, "result settings"):
            VisionSelectionResult(
                kind="mask",
                settings=object(),
                capture_result=mask_result.capture_result,
                mask_bounds=mask_result.mask_bounds,
            )
        with self.assertRaisesRegex(MotionInputError, "filename does not match"):
            VisionSelectionResult(
                kind="template",
                settings=template_settings,
                template_filename="other.jpg",
                template_image=template_result.template_image,
                template_preview=template_result.template_preview,
                visual_options=("other.jpg",),
            )
        with self.assertRaisesRegex(MotionInputError, "missing"):
            prepare_vision_template_selection_result(
                frame,
                template_settings,
                (3, 3, 12, 8),
                ("other.jpg",),
            )

    def test_interactive_vision_selection_is_cancellable_and_cleans_up(self):
        image = np.zeros((12, 16, 3), dtype=np.uint8)
        callbacks = []
        destroyed = []
        wait_calls = []
        feedback_calls = []
        displayed_images = []

        def register_callback(window_name, callback):
            self.assertEqual(window_name, "AR4 Vision Test")
            callbacks.append(callback)

        def wait_key(delay):
            self.assertGreater(delay, 0)
            wait_calls.append(delay)
            if len(wait_calls) == 1:
                callback = callbacks[0]
                callback(cv2.EVENT_LBUTTONDOWN, 1, 1, 0, None)
                callback(cv2.EVENT_LBUTTONUP, 1, 1, 0, None)
            elif len(wait_calls) == 2:
                callback = callbacks[0]
                callback(cv2.EVENT_LBUTTONDOWN, 1, 1, 0, None)
                callback(cv2.EVENT_LBUTTONUP, 5, 5, 0, None)
            elif len(wait_calls) == 3:
                callback = callbacks[0]
                callback(cv2.EVENT_LBUTTONDOWN, 1, 1, 0, None)
                callback(cv2.EVENT_MOUSEMOVE, 14, 10, 0, None)
                callback(cv2.EVENT_LBUTTONUP, 14, 10, 0, None)
            return -1

        patches = (
            patch("ARrobots.HMI.vision_io.cv2.namedWindow"),
            patch(
                "ARrobots.HMI.vision_io.cv2.setMouseCallback",
                side_effect=register_callback,
            ),
            patch(
                "ARrobots.HMI.vision_io.cv2.imshow",
                side_effect=(
                    lambda name, frame: displayed_images.append(frame)
                ),
            ),
            patch("ARrobots.HMI.vision_io.cv2.rectangle"),
            patch(
                "ARrobots.HMI.vision_io.cv2.putText",
                side_effect=lambda *args: feedback_calls.append(args),
            ),
            patch(
                "ARrobots.HMI.vision_io.cv2.waitKey",
                side_effect=wait_key,
            ),
            patch(
                "ARrobots.HMI.vision_io.cv2.getWindowProperty",
                return_value=1.0,
            ),
            patch(
                "ARrobots.HMI.vision_io.cv2.destroyWindow",
                side_effect=lambda name: destroyed.append(name),
            ),
        )
        with patches[0], patches[1], patches[2], patches[3], patches[4], \
                patches[5], patches[6], patches[7]:
            bounds = select_vision_region(image, "AR4 Vision Test")
        self.assertEqual(bounds, (4, 4, 11, 7))
        self.assertEqual(destroyed, ["AR4 Vision Test"])
        self.assertEqual(len(wait_calls), 3)
        self.assertEqual(len(feedback_calls), 1)
        feedback_args = feedback_calls[0]
        self.assertEqual(feedback_args[1], VISION_SELECTION_TOO_SMALL_MESSAGE)
        self.assertEqual(feedback_args[2], (8, 24))
        self.assertEqual(feedback_args[5], (0, 0, 255))
        self.assertIsNot(feedback_args[0], displayed_images[0])

        cancelled = threading.Event()
        cancelled.set()
        with self.assertRaisesRegex(MotionInputError, "was cancelled"):
            select_vision_region(image, "AR4 Vision Test", cancelled)

        destroyed.clear()
        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            return_value=27,
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=lambda name: destroyed.append(name),
        ):
            with self.assertRaisesRegex(MotionInputError, "was cancelled"):
                select_vision_region(image, "AR4 Vision Test")
        self.assertEqual(destroyed, ["AR4 Vision Test"])

        with self.assertRaisesRegex(MotionInputError, "window name"):
            select_vision_region(image, " invalid")

    def test_interactive_vision_selection_handles_failures_and_interrupts(self):
        image = np.zeros((12, 16, 3), dtype=np.uint8)
        destroyed = []

        with patch(
            "ARrobots.HMI.vision_io.cv2.namedWindow",
            side_effect=RuntimeError("window unavailable"),
        ):
            with self.assertRaisesRegex(
                MotionInputError,
                "window failed: window unavailable",
            ):
                select_vision_region(image, "AR4 Vision Test")

        with patch(
            "ARrobots.HMI.vision_io.cv2.namedWindow",
            side_effect=RuntimeError("n" * 600),
        ):
            with self.assertRaises(MotionInputError) as bounded_window_error:
                select_vision_region(image, "AR4 Vision Test")
        self.assertLessEqual(
            len(str(bounded_window_error.exception)),
            MAX_CAMERA_PREVIEW_EVENT_DETAIL,
        )

        callbacks = []

        def register_callback(window_name, callback):
            callbacks.append(callback)

        def callback_failure(delay):
            self.assertGreater(delay, 0)
            callback = callbacks[0]
            callback(cv2.EVENT_LBUTTONDOWN, "invalid", 1, 0, None)
            callback(cv2.EVENT_LBUTTONUP, 14, 10, 0, None)
            callback(cv2.EVENT_LBUTTONDOWN, 1, "second invalid", 0, None)
            callback(cv2.EVENT_LBUTTONUP, 14, 10, 0, None)
            return -1

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback",
            side_effect=register_callback,
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=callback_failure,
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
        ):
            with self.assertRaisesRegex(MotionInputError, "start X must be numeric"):
                select_vision_region(image, "AR4 Vision Test")

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            return_value=True,
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
        ):
            with self.assertRaisesRegex(MotionInputError, "invalid key state"):
                select_vision_region(image, "AR4 Vision Test")

        selected_callbacks = []

        def select_bounds(delay):
            self.assertGreater(delay, 0)
            callback = selected_callbacks[-1]
            callback(cv2.EVENT_LBUTTONDOWN, 1, 1, 0, None)
            callback(cv2.EVENT_LBUTTONUP, 14, 10, 0, None)
            return -1

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback",
            side_effect=(
                lambda name, callback: selected_callbacks.append(callback)
            ),
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=select_bounds,
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=RuntimeError("destroy unavailable"),
        ):
            with self.assertRaisesRegex(
                MotionInputError,
                "cleanup failed: destroy unavailable",
            ):
                select_vision_region(image, "AR4 Vision Test")

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback",
            side_effect=(
                lambda name, callback: selected_callbacks.append(callback)
            ),
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=select_bounds,
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=RuntimeError("c" * 600),
        ):
            with self.assertRaises(MotionInputError) as cleanup_only_error:
                select_vision_region(image, "AR4 Vision Test")
        cleanup_only_detail = str(cleanup_only_error.exception)
        self.assertEqual(
            len(cleanup_only_detail),
            MAX_CAMERA_PREVIEW_EVENT_DETAIL,
        )
        self.assertIn("c" * 400, cleanup_only_detail)

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=RuntimeError("w" * 400),
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=RuntimeError("destroy unavailable"),
        ):
            with self.assertRaises(MotionInputError) as bounded_error:
                select_vision_region(image, "AR4 Vision Test")
        bounded_detail = str(bounded_error.exception)
        self.assertLessEqual(
            len(bounded_detail),
            MAX_CAMERA_PREVIEW_EVENT_DETAIL,
        )
        self.assertIn("w" * 400, bounded_detail)
        self.assertIn("vision selection window cleanup failed", bounded_detail)

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=RuntimeError("w" * 600),
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=RuntimeError("d" * 600),
        ):
            with self.assertRaises(MotionInputError) as two_long_errors:
                select_vision_region(image, "AR4 Vision Test")
        two_long_details = str(two_long_errors.exception)
        self.assertEqual(
            len(two_long_details),
            MAX_CAMERA_PREVIEW_EVENT_DETAIL,
        )
        self.assertIn("w" * 100, two_long_details)
        self.assertIn("d" * 100, two_long_details)

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=RuntimeError("wait unavailable"),
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=RuntimeError("d" * 600),
        ):
            with self.assertRaises(MotionInputError) as long_cleanup_error:
                select_vision_region(image, "AR4 Vision Test")
        long_cleanup_detail = str(long_cleanup_error.exception)
        self.assertEqual(
            len(long_cleanup_detail),
            MAX_CAMERA_PREVIEW_EVENT_DETAIL,
        )
        self.assertIn("window failed: wait unavailable", long_cleanup_detail)
        self.assertIn("d" * 400, long_cleanup_detail)

        interrupted_cleanup = []

        def fail_interrupted_cleanup(name):
            interrupted_cleanup.append(name)
            raise RuntimeError("interrupt destroy unavailable")

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=KeyboardInterrupt(),
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=fail_interrupted_cleanup,
        ):
            with self.assertRaises(KeyboardInterrupt) as interrupted_error:
                select_vision_region(image, "AR4 Vision Test")
        self.assertEqual(interrupted_cleanup, ["AR4 Vision Test"])
        self.assertIn(
            "vision selection window cleanup failed: interrupt destroy "
            "unavailable",
            normalize_camera_exception_detail(interrupted_error.exception),
        )

        clean_interrupt_cleanup = []
        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=KeyboardInterrupt(),
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=lambda name: clean_interrupt_cleanup.append(name),
        ):
            with self.assertRaises(KeyboardInterrupt) as clean_interrupt:
                select_vision_region(image, "AR4 Vision Test")
        self.assertEqual(clean_interrupt_cleanup, ["AR4 Vision Test"])
        self.assertEqual(
            normalize_camera_exception_detail(clean_interrupt.exception),
            "KeyboardInterrupt",
        )

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            side_effect=RuntimeError("wait unavailable"),
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=RuntimeError("destroy unavailable"),
        ):
            with self.assertRaisesRegex(
                MotionInputError,
                "window failed: wait unavailable; vision selection window "
                "cleanup failed: destroy unavailable",
            ):
                select_vision_region(image, "AR4 Vision Test")

        with patch("ARrobots.HMI.vision_io.cv2.namedWindow"), patch(
            "ARrobots.HMI.vision_io.cv2.setMouseCallback"
        ), patch("ARrobots.HMI.vision_io.cv2.imshow"), patch(
            "ARrobots.HMI.vision_io.cv2.waitKey",
            return_value=-1,
        ), patch(
            "ARrobots.HMI.vision_io.cv2.getWindowProperty",
            return_value=0.0,
        ), patch(
            "ARrobots.HMI.vision_io.cv2.destroyWindow",
            side_effect=lambda name: destroyed.append(name),
        ):
            with self.assertRaisesRegex(MotionInputError, "was closed"):
                select_vision_region(image, "AR4 Vision Test")
        self.assertEqual(destroyed, ["AR4 Vision Test"])

    def test_vision_selection_worker_uses_the_noncoalescing_type_contract(self):
        settings = VisionSelectionSettings(
            kind="template",
            template_filename="part.jpg",
        )
        result = prepare_vision_template_selection_result(
            np.zeros((12, 16, 3), dtype=np.uint8),
            settings,
            (3, 3, 12, 8),
            ("part.jpg",),
        )
        operation_threads = []

        def operation(current, cancellation):
            operation_threads.append(threading.get_ident())
            self.assertIs(current, settings)
            self.assertFalse(cancellation.is_set())
            return result

        caller_thread = threading.get_ident()
        worker = VisionOperationWorker(
            operation,
            VisionSelectionSettings,
            VisionSelectionResult,
            "vision selection",
            "ar4-vision-selection-test",
            coalesce=False,
        )

        submission = worker.submit(settings)
        self.assertTrue(worker.wait_stopped(1))
        event = worker.drain_events()[0]
        self.assertEqual(event.request_id, submission.request_id)
        self.assertIs(event.result, result)
        self.assertEqual(len(operation_threads), 1)
        self.assertNotEqual(operation_threads[0], caller_thread)
        self.assertTrue(worker.close())

        mask_settings = VisionSelectionSettings(
            kind="mask",
            capture_settings=self.vision_capture_settings(),
        )
        mask_result = prepare_vision_mask_selection_result(
            np.zeros((4, 6, 3), dtype=np.uint8),
            mask_settings,
            (0, 0, 6, 4),
        )
        mismatched_worker = VisionOperationWorker(
            lambda current, cancellation: mask_result,
            VisionSelectionSettings,
            VisionSelectionResult,
            "vision selection",
            "ar4-vision-selection-settings-test",
            coalesce=False,
        )
        mismatched = mismatched_worker.submit(settings)
        self.assertTrue(mismatched_worker.wait_stopped(1))
        mismatch_event = mismatched_worker.drain_events()[0]
        self.assertEqual(mismatch_event.request_id, mismatched.request_id)
        self.assertIsNone(mismatch_event.result)
        self.assertIn("settings do not match", mismatch_event.error_detail)
        self.assertTrue(mismatched_worker.close())

    def test_vision_capture_worker_runs_off_caller_and_coalesces_pending(self):
        first_started = threading.Event()
        release_first = threading.Event()
        operation_threads = []

        def operation(settings, cancellation_event):
            operation_threads.append(threading.get_ident())
            if settings.brightness == 1:
                first_started.set()
                self.assertTrue(release_first.wait(1))
            self.assertFalse(cancellation_event.is_set())
            frame = np.full((4, 6, 3), settings.brightness, dtype=np.uint8)
            return prepare_vision_capture_result(frame, settings)

        worker = self.vision_capture_worker(operation)
        caller_thread = threading.get_ident()
        first = worker.submit(self.vision_capture_settings(1))
        self.assertFalse(first.coalesced)
        self.assertTrue(first_started.wait(1))
        second = worker.submit(self.vision_capture_settings(2))
        third = worker.submit(self.vision_capture_settings(3))
        self.assertTrue(second.coalesced)
        self.assertTrue(third.coalesced)
        self.assertEqual(worker.active_request_id, first.request_id)
        self.assertEqual(worker.pending_request_id, third.request_id)
        ownership = worker.drain_events_state()
        self.assertEqual(ownership.events, ())
        self.assertTrue(ownership.active)
        self.assertEqual(ownership.active_request_id, first.request_id)
        self.assertEqual(ownership.pending_request_id, third.request_id)

        release_first.set()
        self.assertTrue(worker.wait_stopped(2))
        terminal = worker.drain_events_state()
        self.assertFalse(terminal.active)
        self.assertIsNone(terminal.active_request_id)
        self.assertIsNone(terminal.pending_request_id)
        events = terminal.events
        self.assertEqual(
            [event.request_id for event in events],
            [first.request_id, third.request_id],
        )
        self.assertTrue(all(event.result is not None for event in events))
        self.assertNotIn(caller_thread, operation_threads)
        self.assertEqual(len(operation_threads), 2)
        self.assertTrue(worker.close())
        self.assertTrue(worker.closed)
        with self.assertRaisesRegex(MotionInputError, "worker is closed"):
            worker.submit(self.vision_capture_settings())

    def test_vision_capture_worker_close_cancels_active_operation(self):
        operation_started = threading.Event()

        def operation(settings, cancellation_event):
            operation_started.set()
            self.assertTrue(cancellation_event.wait(1))
            raise MotionInputError("capture cancelled for shutdown")

        worker = self.vision_capture_worker(operation)
        submission = worker.submit(self.vision_capture_settings())
        self.assertTrue(operation_started.wait(1))
        self.assertFalse(worker.close())
        self.assertTrue(worker.wait_stopped(1))
        event = worker.drain_events()[0]
        self.assertEqual(event.request_id, submission.request_id)
        self.assertIsNone(event.result)
        self.assertIn("cancelled for shutdown", event.error_detail)

    def test_vision_operation_close_settles_request_pending_worker_pickup(self):
        worker_started = threading.Event()
        release_worker = threading.Event()
        operation_calls = []

        class DelayedThread(threading.Thread):
            def run(self):
                worker_started.set()
                if not release_worker.wait(1):
                    return
                super().run()

        worker = self.vision_capture_worker(
            lambda *args: operation_calls.append(args),
            thread_factory=DelayedThread,
        )
        submission = worker.submit(self.vision_capture_settings())
        self.assertTrue(worker_started.wait(1))
        self.assertEqual(worker.pending_request_id, submission.request_id)

        self.assertFalse(worker.close())
        drain_state = worker.drain_events_state()
        self.assertIsInstance(drain_state, VisionOperationDrainState)
        self.assertTrue(drain_state.active)
        self.assertIsNone(drain_state.active_request_id)
        self.assertIsNone(drain_state.pending_request_id)
        events = drain_state.events
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].request_id, submission.request_id)
        self.assertIsNone(events[0].result)
        self.assertEqual(
            events[0].error_detail,
            "vision capture was cancelled during shutdown",
        )
        self.assertIsNone(worker.pending_request_id)

        release_worker.set()
        self.assertTrue(worker.wait_stopped(1))
        self.assertEqual(operation_calls, [])
        self.assertTrue(worker.close())

    def test_vision_operation_drain_state_rejects_invalid_lifecycle(self):
        with self.assertRaisesRegex(MotionInputError, "events must be a tuple"):
            VisionOperationDrainState([], False, None, None)
        with self.assertRaisesRegex(MotionInputError, "must be boolean"):
            VisionOperationDrainState((), 1, None, None)
        with self.assertRaisesRegex(MotionInputError, "request id"):
            VisionOperationDrainState((), True, True, None)
        with self.assertRaisesRegex(MotionInputError, "cannot retain"):
            VisionOperationDrainState((), False, 1, None)

    def test_vision_capture_worker_rolls_back_startup_failures(self):
        settings = self.vision_capture_settings()
        with self.assertRaisesRegex(MotionInputError, "operation"):
            self.vision_capture_worker(None)
        with self.assertRaisesRegex(MotionInputError, "thread factory"):
            self.vision_capture_worker(
                lambda *args: None,
                thread_factory=None,
            )
        with self.assertRaisesRegex(MotionInputError, "type contract"):
            VisionOperationWorker(
                lambda *args: None,
                VisionCaptureSettings,
                VisionMatchOperationResult,
                "vision capture",
                "ar4-vision-invalid-test",
            )

        def failing_factory(**kwargs):
            raise RuntimeError("thread construction unavailable")

        worker = self.vision_capture_worker(
            lambda *args: self.fail("operation must not run"),
            thread_factory=failing_factory,
        )
        with self.assertRaisesRegex(RuntimeError, "construction unavailable"):
            worker.submit(settings)
        self.assertFalse(worker.active)
        self.assertIsNone(worker.pending_request_id)
        self.assertTrue(worker.wait_stopped(0))

        class FailingThread(threading.Thread):
            def start(self):
                raise RuntimeError("thread startup unavailable")

        worker = self.vision_capture_worker(
            lambda *args: self.fail("operation must not run"),
            thread_factory=FailingThread,
        )
        with self.assertRaisesRegex(RuntimeError, "startup unavailable"):
            worker.submit(settings)
        self.assertFalse(worker.active)
        self.assertIsNone(worker.pending_request_id)
        self.assertTrue(worker.wait_stopped(0))

    def test_vision_capture_worker_retires_all_requests_after_terminal_failure(self):
        settings = self.vision_capture_settings()
        result = prepare_vision_capture_result(
            np.zeros((4, 6, 3), dtype=np.uint8),
            settings,
        )
        stringify_started = threading.Event()
        finish_stringify = threading.Event()

        class DelayedTerminalError(RuntimeError):
            def __str__(self):
                stringify_started.set()
                if not finish_stringify.wait(1):
                    return "terminal publication wait timed out"
                return "terminal publication failed"

        worker = self.vision_capture_worker(lambda *args: result)
        append_event = worker._append_event_locked
        fail_next_append = [True]

        def append_with_failure(*args, **kwargs):
            if fail_next_append[0]:
                fail_next_append[0] = False
                raise DelayedTerminalError()
            return append_event(*args, **kwargs)

        worker._append_event_locked = append_with_failure
        first = worker.submit(settings)
        self.assertTrue(stringify_started.wait(1))
        second = worker.submit(settings)
        self.assertTrue(second.coalesced)
        finish_stringify.set()
        self.assertTrue(worker.wait_stopped(1))

        failed_events = worker.drain_events()
        self.assertEqual(
            [event.request_id for event in failed_events],
            [first.request_id, second.request_id],
        )
        self.assertTrue(
            all(
                "terminal publication failed" in event.error_detail
                for event in failed_events
            )
        )
        self.assertFalse(worker.active)
        self.assertIsNone(worker.pending_request_id)

        recovered = worker.submit(settings)
        self.assertTrue(worker.wait_stopped(1))
        recovered_event = worker.drain_events()[0]
        self.assertEqual(recovered_event.request_id, recovered.request_id)
        self.assertIs(recovered_event.result, result)
        self.assertTrue(worker.close())

    def test_vision_match_processing_returns_owned_worker_ready_result(self):
        random = np.random.default_rng(17)
        source = np.full((48, 48), 11, dtype=np.uint8)
        template = random.integers(
            0,
            256,
            size=(8, 10),
            dtype=np.uint8,
        )
        source[12:20, 18:28] = template

        result = prepare_vision_match_result(
            source,
            template,
            11,
            self.vision_match_options(),
        )

        self.assertIsInstance(result, VisionMatchResult)
        self.assertTrue(result.matched)
        self.assertGreaterEqual(result.score, 0.99)
        self.assertEqual(result.angle_degrees, 0.0)
        self.assertEqual(result.pixel_position, (16, 23))
        self.assertEqual(result.robot_position, (160.0, 230.0))
        self.assertEqual(result.annotated_image.shape, (48, 48, 3))
        self.assertEqual(result.display_image.shape, (480, 640, 3))
        self.assertFalse(result.annotated_image.flags.writeable)
        self.assertFalse(result.display_image.flags.writeable)
        self.assertTrue(
            np.any(np.all(result.annotated_image == (0, 255, 0), axis=2))
        )

    def test_vision_match_clamps_both_joint6_limits_when_enabled(self):
        source = np.zeros((48, 48), dtype=np.uint8)
        template = np.zeros((8, 10), dtype=np.uint8)
        positive = (0.95, 120.0, (18, 12), 10, 8)
        with patch(
            "ARrobots.HMI.vision_io._best_narrow_vision_rotation",
            return_value=positive,
        ):
            rejected = prepare_vision_match_result(
                source,
                template,
                0,
                self.vision_match_options(
                    minimum_score=0.9,
                    full_rotation_search=False,
                    joint6_positive_limit=90.0,
                ),
            )
            clamped = prepare_vision_match_result(
                source,
                template,
                0,
                self.vision_match_options(
                    minimum_score=0.9,
                    full_rotation_search=False,
                    try_closest_out_of_range=True,
                    joint6_positive_limit=90.0,
                ),
            )
        self.assertFalse(rejected.matched)
        self.assertTrue(clamped.matched)
        self.assertEqual(clamped.angle_degrees, 90.0)

        negative = (0.95, 240.0, (18, 12), 10, 8)
        with patch(
            "ARrobots.HMI.vision_io._best_narrow_vision_rotation",
            return_value=negative,
        ):
            clamped = prepare_vision_match_result(
                source,
                template,
                0,
                self.vision_match_options(
                    minimum_score=0.9,
                    full_rotation_search=False,
                    try_closest_out_of_range=True,
                    joint6_negative_limit=75.0,
                ),
            )
        self.assertTrue(clamped.matched)
        self.assertEqual(clamped.angle_degrees, -75.0)

    def test_full_rotation_search_checks_each_unique_degree_at_most_once(self):
        source = np.zeros((48, 48), dtype=np.uint8)
        template = np.zeros((8, 10), dtype=np.uint8)
        angles = []

        def candidate(image, current_template, angle, background):
            angles.append(angle)
            score = 0.5 if angle == 211 else 0.1
            return score, float(angle), (0, 0), 10, 8

        with patch(
            "ARrobots.HMI.vision_io._vision_rotation_candidate",
            side_effect=candidate,
        ):
            result = prepare_vision_match_result(
                source,
                template,
                0,
                self.vision_match_options(minimum_score=0.9),
            )

        self.assertFalse(result.matched)
        self.assertEqual(result.score, 0.5)
        self.assertEqual(angles, list(range(360)))
        np.testing.assert_array_equal(
            result.annotated_image[5, 5],
            np.asarray((0, 0, 255), dtype=np.uint8),
        )

    def test_vision_rotation_uses_selected_background_for_new_border_pixels(self):
        template = np.arange(80, dtype=np.uint8).reshape((8, 10))
        with patch(
            "ARrobots.HMI.vision_io.cv2.warpAffine",
            wraps=cv2.warpAffine,
        ) as warp:
            rotated = _rotate_vision_template(template, 45, 173)

        self.assertEqual(rotated.shape, template.shape)
        self.assertEqual(warp.call_count, 1)
        self.assertEqual(warp.call_args.kwargs["borderMode"], cv2.BORDER_CONSTANT)
        self.assertEqual(warp.call_args.kwargs["borderValue"], 173)

    def test_vision_match_worker_rejects_overlap_and_honors_cancellation(self):
        capture_settings = self.vision_capture_settings()
        settings = VisionMatchSettings(
            capture_settings,
            self.vision_match_options(),
        )
        started = threading.Event()
        external_cancellation = threading.Event()

        def operation(match_settings, cancellation_event):
            self.assertIs(match_settings, settings)
            started.set()
            self.assertTrue(cancellation_event.wait(1))
            raise MotionInputError("matching cancelled by test")

        worker = VisionOperationWorker(
            operation,
            VisionMatchSettings,
            VisionMatchOperationResult,
            "vision matching",
            "ar4-vision-match-test",
            coalesce=False,
        )
        submission = worker.submit(settings, external_cancellation)
        self.assertTrue(started.wait(1))
        with self.assertRaisesRegex(MotionInputError, "already active"):
            worker.submit(settings)
        external_cancellation.set()
        self.assertTrue(worker.wait_stopped(1))
        event = worker.drain_events()[0]
        self.assertEqual(event.request_id, submission.request_id)
        self.assertIsNone(event.result)
        self.assertIn("matching cancelled by test", event.error_detail)
        self.assertTrue(worker.close())

    def test_vision_match_boundaries_reject_invalid_inputs(self):
        with self.assertRaisesRegex(MotionInputError, "lowercase .jpg"):
            self.vision_match_options(template_filename="../template.jpg")
        with self.assertRaisesRegex(MotionInputError, "lowercase .jpg"):
            self.vision_match_options(template_filename="..\\template.jpg")
        with self.assertRaisesRegex(MotionInputError, "between 0 and 1"):
            self.vision_match_options(minimum_score=1.01)
        with self.assertRaisesRegex(MotionInputError, "calibration spans"):
            VisionCoordinateMapping(*(0.0 for _ in range(8)))
        with self.assertRaisesRegex(MotionInputError, "must not exceed"):
            prepare_vision_match_result(
                np.zeros((4, 4), dtype=np.uint8),
                np.zeros((5, 4), dtype=np.uint8),
                0,
                self.vision_match_options(),
            )
        cancelled = threading.Event()
        cancelled.set()
        with self.assertRaisesRegex(MotionInputError, "was cancelled"):
            prepare_vision_match_result(
                np.zeros((48, 48), dtype=np.uint8),
                np.zeros((8, 10), dtype=np.uint8),
                0,
                self.vision_match_options(),
                cancelled,
            )

    def test_camera_preview_worker_validates_constructor_contract(self):
        valid_factory = lambda camera_index: None
        constructors = (
            (
                "capture factory",
                lambda: CameraPreviewWorker(None),
                "capture factory",
            ),
            (
                "frame transform",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    frame_transform=None,
                ),
                "frame transform",
            ),
            (
                "warmup",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    warmup_frames=-1,
                ),
                "warmup frame count",
            ),
            (
                "warmup upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    warmup_frames=121,
                ),
                "warmup frame count exceeds",
            ),
            (
                "read limit",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    read_failure_limit=0,
                ),
                "read failure limit must be positive",
            ),
            (
                "read upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    read_failure_limit=121,
                ),
                "read failure count exceeds",
            ),
            (
                "retry",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    retry_seconds=float("inf"),
                ),
                "retry interval",
            ),
            (
                "retry upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    retry_seconds=5.1,
                ),
                "retry interval",
            ),
            (
                "release attempts",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    release_attempts=0,
                ),
                "release attempt count must be positive",
            ),
            (
                "release upper bound",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    release_attempts=11,
                ),
                "release attempt count exceeds",
            ),
            (
                "thread factory",
                lambda: CameraPreviewWorker(
                    valid_factory,
                    thread_factory=None,
                ),
                "thread factory",
            ),
        )
        for name, constructor, expected_detail in constructors:
            with self.subTest(name=name):
                with self.assertRaisesRegex(
                    MotionInputError,
                    expected_detail,
                ):
                    constructor()

    def test_camera_preview_request_and_diagnostic_boundaries(self):
        worker = CameraPreviewWorker(
            lambda camera_source: self.fail(
                "invalid camera source must not reach the factory"
            )
        )
        for source in (
            True,
            -1,
            "",
            " /dev/video0",
            "/dev/video0\n",
            "camera\x00source",
            "x" * 513,
        ):
            with self.subTest(source=source):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "camera source",
                ):
                    worker.request_start(source)
        with self.assertRaisesRegex(
            MotionInputError,
            "replacement option",
        ):
            worker.request_start(0, replace="yes")

        detail = normalize_camera_exception_detail(
            RuntimeError("a" * 511 + " " + "tail")
        )
        self.assertEqual(detail, "a" * 511)
        self.assertEqual(detail, detail.strip())

    def test_camera_capture_once_uses_owned_worker_lifecycle(self):
        capture = QueuedCapture()
        factory_calls = []

        def factory(camera_source):
            factory_calls.append(camera_source)
            return capture

        worker = CameraPreviewWorker(
            factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0.05,
        )
        expected = np.full((2, 3, 3), 19, dtype=np.uint8)
        capture.responses.put((True, expected))
        capture.responses.put((False, None))
        captured = worker.capture_once("/dev/video6", 1)

        np.testing.assert_array_equal(captured, expected)
        self.assertIsNot(captured, expected)
        self.assertEqual(factory_calls, ["/dev/video6"])
        self.assertEqual(capture.release_count, 1)
        self.assertFalse(worker.active)
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["starting", "started", "stopping", "stopped"],
        )

        cancelled = threading.Event()
        cancelled.set()
        with self.assertRaisesRegex(MotionInputError, "was cancelled"):
            worker.capture_once(0, 1, cancelled)
        self.assertEqual(factory_calls, ["/dev/video6"])

    def test_camera_readiness_wait_honors_cancellation(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_source: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        cancelled = threading.Event()
        cancelled.set()

        self.assertFalse(worker.wait_ready(request_id, 1, cancelled))
        self.assertTrue(worker.request_stop(request_id))
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_request_stopped(request_id, 1))

    def test_camera_transition_snapshot_distinguishes_start_from_stop(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_source: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        expected = np.full((2, 3, 3), 23, dtype=np.uint8)
        capture.responses.put((True, expected))

        captured = worker.wait_snapshot_or_stopped(request_id, 1)

        np.testing.assert_array_equal(captured, expected)
        self.assertIsNot(captured, expected)
        self.assertTrue(worker.request_stop(request_id))
        capture.responses.put((False, None))
        self.assertIsNone(worker.wait_snapshot_or_stopped(request_id, 1))
        self.assertTrue(worker.wait_stopped(1))

        cancelled = threading.Event()
        cancelled.set()
        with self.assertRaisesRegex(MotionInputError, "was cancelled"):
            worker.wait_snapshot_or_stopped(request_id, 1, cancelled)

    def test_camera_stop_before_worker_entry_has_terminal_lifecycle(self):
        worker_entry = threading.Event()

        def thread_factory(target, name, daemon):
            def delayed_target():
                worker_entry.wait(1)
                target()

            return threading.Thread(
                target=delayed_target,
                name=name,
                daemon=daemon,
            )

        worker = CameraPreviewWorker(
            lambda camera_source: self.fail(
                "cancelled request must not open a capture"
            ),
            thread_factory=thread_factory,
        )
        request_id = worker.request_start(0)
        self.assertTrue(worker.request_stop(request_id))
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["starting", "stopping", "stopped"],
        )
        worker_entry.set()
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_replacement_retains_undrained_stop_event(self):
        worker_entry = threading.Event()
        capture = QueuedCapture()
        factory_sources = []

        def thread_factory(target, name, daemon):
            def delayed_target():
                worker_entry.wait(1)
                target()

            return threading.Thread(
                target=delayed_target,
                name=name,
                daemon=daemon,
            )

        def capture_factory(camera_source):
            factory_sources.append(camera_source)
            return capture

        worker = CameraPreviewWorker(
            capture_factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
            thread_factory=thread_factory,
        )
        stopped_request = worker.request_start(0)
        self.assertTrue(worker.request_stop(stopped_request))
        replacement_request = worker.request_start(1)
        self.assertEqual(
            [
                (event.kind, event.request_id)
                for event in worker.drain_events()
            ],
            [
                ("stopped", stopped_request),
                ("starting", replacement_request),
            ],
        )

        worker_entry.set()
        self.assertTrue(
            wait_for(
                lambda: (
                    worker.active_request_id == replacement_request
                    and factory_sources == [1]
                )
            )
        )
        self.assertEqual(factory_sources, [1])
        self.assertTrue(worker.request_stop(replacement_request))
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_worker_moves_io_off_caller_and_coalesces_frames(self):
        capture = QueuedCapture()
        factory_threads = []

        def factory(camera_index):
            factory_threads.append((camera_index, threading.get_ident()))
            return capture

        worker = CameraPreviewWorker(
            factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        self.assertFalse(worker.closed)
        caller_thread = threading.get_ident()
        request_id = worker.request_start(2)
        for value in (1, 2, 3):
            capture.responses.put(
                (
                    True,
                    np.full((3, 4, 3), value, dtype=np.uint8),
                )
            )

        def latest_raw_is_three():
            frame = worker.snapshot_raw_frame(request_id)
            return frame is not None and np.all(frame == 3)

        self.assertTrue(wait_for(latest_raw_is_three))
        self.assertEqual(worker.active_request_id, request_id)
        self.assertIsNone(worker.take_latest_frame(request_id + 1))
        preview = worker.take_latest_frame(request_id)
        self.assertIsNotNone(preview)
        self.assertTrue(np.all(preview.image == 3))
        self.assertFalse(preview.image.flags.writeable)
        self.assertEqual(factory_threads[0][0], 2)
        self.assertNotEqual(factory_threads[0][1], caller_thread)
        self.assertTrue(capture.read_threads)
        self.assertNotIn(caller_thread, capture.read_threads)

        events = list(worker.drain_events())
        self.assertEqual(
            [event.kind for event in events],
            ["starting", "started"],
        )
        self.assertEqual(
            [event.sequence for event in events],
            sorted(event.sequence for event in events),
        )

        self.assertTrue(worker.request_stop())
        capture.responses.put((False, None))
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertEqual(capture.release_count, 1)
        self.assertNotIn(caller_thread, capture.release_threads)
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["stopping", "stopped"],
        )

    def test_camera_preview_worker_uses_default_warmup_and_transform(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        for value in range(5):
            capture.responses.put(
                (
                    True,
                    np.full((2, 3, 3), value, dtype=np.uint8),
                )
            )
        final_frame = np.zeros((2, 3, 3), dtype=np.uint8)
        final_frame[:, :] = (10, 20, 30)
        capture.responses.put((True, final_frame))

        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(request_id) is not None
            )
        )
        raw = worker.snapshot_raw_frame(request_id)
        np.testing.assert_array_equal(raw, final_frame)
        preview = worker.take_latest_frame(request_id)
        self.assertEqual(preview.image.shape, (320, 480, 3))
        np.testing.assert_array_equal(preview.image[0, 0], (30, 20, 10))
        self.assertFalse(preview.image.flags.writeable)

        self.assertTrue(worker.request_stop())
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_restart_cannot_be_stranded_during_worker_exit(self):
        first = QueuedCapture()
        second = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: (first, second)[camera_index],
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )

        class ExitBoundaryLock:
            def __init__(self, owner):
                self._lock = threading.Lock()
                self.owner = owner
                self.candidate_releases = 0
                self.boundary = threading.Event()
                self.resume = threading.Event()

            def acquire(self, *args, **kwargs):
                return self._lock.acquire(*args, **kwargs)

            def release(self):
                current = threading.current_thread()
                exit_candidate = (
                    current.name == "ar4-camera-preview"
                    and self.owner._desired is None
                    and self.owner._active_request_id is None
                )
                if exit_candidate:
                    self.candidate_releases += 1
                self._lock.release()
                if exit_candidate and self.candidate_releases == 2:
                    self.boundary.set()
                    if not self.resume.wait(2):
                        raise RuntimeError("exit-boundary test was not resumed")

            def __enter__(self):
                self.acquire()
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                self.release()

        boundary_lock = ExitBoundaryLock(worker)
        worker._lock = boundary_lock
        worker._state_changed = threading.Condition(boundary_lock)
        first_request = worker.request_start(0)
        first.responses.put(
            (True, np.full((2, 2, 3), 1, dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(first_request) is not None
            )
        )
        old_thread = worker._worker

        self.assertTrue(worker.request_stop())
        first.responses.put((False, None))
        self.assertTrue(boundary_lock.boundary.wait(2))
        second_request = worker.request_start(1)
        self.assertIsNot(worker._worker, old_thread)
        self.assertEqual(worker.desired_request_id, second_request)
        boundary_lock.resume.set()
        old_thread.join(1)
        self.assertFalse(old_thread.is_alive())

        second.responses.put(
            (True, np.full((2, 2, 3), 2, dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(second_request) is not None
            )
        )
        self.assertEqual(worker.active_request_id, second_request)
        self.assertTrue(worker.request_stop())
        second.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_close_reports_and_waits_for_live_retirement(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        capture.responses.put(
            (True, np.ones((2, 2, 3), dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(request_id) is not None
            )
        )

        close_state = worker.close_state()
        self.assertIsInstance(close_state, CameraPreviewLifecycleState)
        self.assertTrue(close_state.active)
        self.assertFalse(close_state.stopped)
        self.assertTrue(close_state.closed)
        self.assertIsNone(close_state.fault_reason)
        self.assertFalse(close_state.clean)
        capture.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))
        self.assertFalse(worker.active)
        retired_state = worker.close_state()
        self.assertFalse(retired_state.active)
        self.assertTrue(retired_state.stopped)
        self.assertTrue(retired_state.clean)
        self.assertTrue(worker.close())
        with self.assertRaisesRegex(MotionInputError, "worker is closed"):
            worker.request_start(1)
        with self.assertRaisesRegex(MotionInputError, "wait timeout"):
            worker.wait_stopped(float("nan"))
        with self.assertRaisesRegex(MotionInputError, "wait timeout"):
            worker.wait_stopped(threading.TIMEOUT_MAX + 1)

    def test_camera_preview_worker_replaces_active_request_without_tk_wait(self):
        first = QueuedCapture()
        second = QueuedCapture()
        captures = {4: first, 7: second}
        factory_calls = []

        def factory(camera_index):
            factory_calls.append(camera_index)
            return captures[camera_index]

        worker = CameraPreviewWorker(
            factory,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        first_request = worker.request_start(4)
        first.responses.put(
            (True, np.full((2, 2, 3), 4, dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(first_request) is not None
            )
        )

        second_request = worker.request_start(7)
        self.assertNotEqual(first_request, second_request)
        self.assertEqual(worker.desired_request_id, second_request)
        with self.assertRaisesRegex(
            MotionInputError,
            "ownership changed before idle",
        ):
            worker.wait_request_stopped(
                first_request,
                0,
                require_idle=True,
            )
        first.responses.put((False, None))
        second.responses.put(
            (True, np.full((2, 2, 3), 7, dtype=np.uint8))
        )

        def replacement_is_live():
            frame = worker.snapshot_raw_frame(second_request)
            return frame is not None and np.all(frame == 7)

        self.assertTrue(wait_for(replacement_is_live))
        self.assertEqual(factory_calls, [4, 7])
        self.assertEqual(first.release_count, 1)
        self.assertIsNone(worker.snapshot_raw_frame(first_request))

        self.assertTrue(worker.request_stop())
        second.responses.put((False, None))
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertEqual(second.release_count, 1)
        events = worker.drain_events()
        first_stopped = next(
            index
            for index, event in enumerate(events)
            if event.kind == "stopped"
            and event.request_id == first_request
        )
        second_started = next(
            index
            for index, event in enumerate(events)
            if event.kind == "started"
            and event.request_id == second_request
        )
        self.assertLess(first_stopped, second_started)

    def test_camera_preview_replacement_preserves_undelivered_failure(self):
        class ClosedCapture(QueuedCapture):
            def __init__(self):
                super().__init__()
                self.opened = False

        first = ClosedCapture()
        second = QueuedCapture()
        captures = iter((first, second))
        worker = CameraPreviewWorker(
            lambda camera_index: next(captures),
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        first_request = worker.request_start(0)
        self.assertTrue(worker.wait_stopped(1))

        second_request = worker.request_start(0)
        second.responses.put(
            (True, np.ones((2, 2, 3), dtype=np.uint8))
        )
        self.assertTrue(
            wait_for(
                lambda: worker.snapshot_raw_frame(second_request) is not None
            )
        )
        events = worker.drain_events()
        retained_failure = next(
            event
            for event in events
            if event.kind == "failed"
            and event.request_id == first_request
        )
        self.assertIn("did not open", retained_failure.detail)
        self.assertIn(
            ("started", second_request),
            tuple((event.kind, event.request_id) for event in events),
        )

        self.assertTrue(worker.request_stop())
        second.responses.put((False, None))
        self.assertTrue(worker.wait_stopped(1))

    def test_camera_preview_worker_rejects_invalid_frames_without_poisoning(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        request_id = worker.request_start(0)
        capture.responses.put(
            (True, np.zeros((2, 2), dtype=np.uint8))
        )
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertIsNone(worker.fault_reason)
        self.assertIsNone(worker.snapshot_raw_frame(request_id))
        self.assertEqual(capture.release_count, 1)
        events = worker.drain_events()
        failures = [event for event in events if event.kind == "failed"]
        self.assertEqual(len(failures), 1)
        self.assertIn("three 8-bit channels", failures[0].detail)
        self.assertEqual(events[-1].kind, "stopped")

    def test_camera_preview_worker_exhausts_bounded_read_failures(self):
        capture = QueuedCapture()
        worker = CameraPreviewWorker(
            lambda camera_index: capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        worker.request_start(0)
        for _ in range(3):
            capture.responses.put((False, None))

        self.assertTrue(worker.wait_stopped(1))
        self.assertEqual(capture.release_count, 1)
        failure = next(
            event
            for event in worker.drain_events()
            if event.kind == "failed"
        )
        self.assertIn("consecutive read-failure limit", failure.detail)

    def test_camera_preview_worker_rejects_open_and_read_contract_failures(self):
        class ResultCapture:
            def __init__(self, result, opened=True):
                self.result = result
                self.opened = opened
                self.read_count = 0
                self.release_count = 0

            def isOpened(self):
                return self.opened

            def read(self):
                self.read_count += 1
                return self.result

            def release(self):
                self.release_count += 1
                self.opened = False

        cases = (
            ("closed", ResultCapture((True, None), opened=False), "did not open"),
            ("shape", ResultCapture("invalid"), "read result is invalid"),
            ("status", ResultCapture((1, None)), "read status is invalid"),
            (
                "failed-data",
                ResultCapture(
                    (False, np.zeros((2, 2, 3), dtype=np.uint8))
                ),
                "unexpected frame data",
            ),
        )
        for name, capture, expected_detail in cases:
            with self.subTest(name=name):
                worker = CameraPreviewWorker(
                    lambda camera_index, selected=capture: selected,
                    frame_transform=lambda frame: frame,
                    warmup_frames=0,
                    retry_seconds=0,
                )
                worker.request_start(0)
                self.assertTrue(worker.wait_stopped(1))
                failure = next(
                    event
                    for event in worker.drain_events()
                    if event.kind == "failed"
                )
                self.assertIn(expected_detail, failure.detail)
                self.assertEqual(capture.release_count, 1)
                self.assertEqual(
                    capture.read_count,
                    0 if name == "closed" else 1,
                )

    def test_camera_preview_worker_rejects_missing_capture_operations(self):
        class MissingRead:
            @staticmethod
            def isOpened():
                return True

            @staticmethod
            def release():
                pass

        class MissingRelease:
            @staticmethod
            def isOpened():
                return True

            @staticmethod
            def read():
                return False, None

        class InvalidOpenState:
            @staticmethod
            def isOpened():
                return 1

            @staticmethod
            def read():
                return False, None

            @staticmethod
            def release():
                pass

        cases = (
            (MissingRead(), "no read operation"),
            (MissingRelease(), "no release operation"),
            (InvalidOpenState(), "open state is invalid"),
        )
        for capture, expected_detail in cases:
            with self.subTest(expected_detail=expected_detail):
                worker = CameraPreviewWorker(
                    lambda camera_index, selected=capture: selected,
                    retry_seconds=0,
                )
                worker.request_start(0)
                self.assertTrue(worker.wait_stopped(1))
                failure = next(
                    event
                    for event in worker.drain_events()
                    if event.kind == "failed"
                    and expected_detail in event.detail
                )
                self.assertIn(expected_detail, failure.detail)

    def test_camera_preview_error_details_are_fully_bounded_and_normalized(self):
        prefix = "camera preview failed: "
        long_detail = "x" * (512 - len(prefix) - 1) + " " + "tail"
        worker = CameraPreviewWorker(
            lambda camera_index: (_ for _ in ()).throw(
                RuntimeError(long_detail)
            ),
            retry_seconds=0,
        )
        worker.request_start(0)

        self.assertTrue(worker.wait_stopped(1))
        failure = next(
            event
            for event in worker.drain_events()
            if event.kind == "failed"
        )
        self.assertLessEqual(len(failure.detail), 512)
        self.assertEqual(len(failure.detail), 511)
        self.assertEqual(failure.detail, failure.detail.strip())
        self.assertIsNone(worker.fault_reason)

    def test_camera_preview_worker_latches_unreleased_capture(self):
        class UnreleasedCapture:
            @staticmethod
            def isOpened():
                return True

            @staticmethod
            def read():
                return True, np.zeros((2, 2), dtype=np.uint8)

            @staticmethod
            def release():
                raise RuntimeError("device remains owned")

        worker = CameraPreviewWorker(
            lambda camera_index: UnreleasedCapture(),
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0,
        )
        worker.request_start(0)
        self.assertTrue(wait_for(lambda: not worker.active))
        self.assertIn("camera release failed", worker.fault_reason)
        with self.assertRaisesRegex(
            MotionInputError,
            "requires an application restart",
        ):
            worker.request_start(1)
        self.assertFalse(worker.close())
        failures = [
            event
            for event in worker.drain_events()
            if event.kind == "failed"
        ]
        self.assertEqual(len(failures), 2)
        self.assertIn("device remains owned", failures[-1].detail)

    def test_camera_preview_release_retries_and_latches_persistent_open_state(self):
        class DelayedReleaseCapture:
            def __init__(self, closes_after):
                self.closes_after = closes_after
                self.release_count = 0

            def isOpened(self):
                return self.release_count < self.closes_after

            @staticmethod
            def read():
                return True, np.zeros((2, 2), dtype=np.uint8)

            def release(self):
                self.release_count += 1

        recovering_capture = DelayedReleaseCapture(2)
        recovering_worker = CameraPreviewWorker(
            lambda camera_index: recovering_capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0.25,
        )
        recovering_sleeps = []
        with patch(
            "ARrobots.HMI.vision_io.time.sleep",
            side_effect=recovering_sleeps.append,
        ):
            recovering_worker.request_start(0)
            self.assertTrue(recovering_worker.wait_stopped(1))
        self.assertEqual(recovering_capture.release_count, 2)
        self.assertEqual(recovering_sleeps, [0.25])
        self.assertIsNone(recovering_worker.fault_reason)

        retained_capture = DelayedReleaseCapture(4)
        retained_worker = CameraPreviewWorker(
            lambda camera_index: retained_capture,
            frame_transform=lambda frame: frame,
            warmup_frames=0,
            retry_seconds=0.25,
        )
        retained_sleeps = []
        with patch(
            "ARrobots.HMI.vision_io.time.sleep",
            side_effect=retained_sleeps.append,
        ):
            retained_worker.request_start(0)
            self.assertTrue(retained_worker.wait_stopped(1))
        self.assertEqual(retained_capture.release_count, 3)
        self.assertEqual(retained_sleeps, [0.25, 0.25])
        self.assertIn("device remained open", retained_worker.fault_reason)

    def test_camera_preview_worker_rolls_back_thread_start_failure(self):
        def failing_thread_factory(**kwargs):
            raise RuntimeError("thread construction unavailable")

        creation_worker = CameraPreviewWorker(
            lambda camera_index: self.fail("capture factory must not run"),
            thread_factory=failing_thread_factory,
        )
        with self.assertRaisesRegex(
            RuntimeError,
            "thread construction unavailable",
        ):
            creation_worker.request_start(0)
        self.assertFalse(creation_worker.active)
        self.assertIsNone(creation_worker.desired_request_id)
        self.assertTrue(creation_worker.wait_stopped(0))
        self.assertEqual(
            [event.kind for event in creation_worker.drain_events()],
            ["starting", "failed"],
        )

        invalid_worker = CameraPreviewWorker(
            lambda camera_index: self.fail("capture factory must not run"),
            thread_factory=lambda **kwargs: object(),
        )
        with self.assertRaisesRegex(MotionInputError, "invalid worker"):
            invalid_worker.request_start(0)
        self.assertFalse(invalid_worker.active)
        self.assertIsNone(invalid_worker.desired_request_id)
        self.assertTrue(invalid_worker.wait_stopped(0))

        class FailingThread(threading.Thread):
            def start(self):
                raise RuntimeError("thread unavailable")

        worker = CameraPreviewWorker(
            lambda camera_index: self.fail("capture factory must not run"),
            thread_factory=FailingThread,
        )
        with self.assertRaisesRegex(RuntimeError, "thread unavailable"):
            worker.request_start(0)
        self.assertFalse(worker.active)
        self.assertIsNone(worker.desired_request_id)
        self.assertEqual(
            [event.kind for event in worker.drain_events()],
            ["starting", "failed"],
        )
        self.assertTrue(worker.close())

    def test_camera_preview_frame_conversion_validates_and_converts_bgr(self):
        source = np.zeros((2, 3, 3), dtype=np.uint8)
        source[:, :] = (1, 2, 3)
        preview = prepare_camera_preview_frame(source, 3, 2)

        np.testing.assert_array_equal(preview[0, 0], (3, 2, 1))
        np.testing.assert_array_equal(source[0, 0], (1, 2, 3))
        with self.assertRaisesRegex(MotionInputError, "three 8-bit channels"):
            prepare_camera_preview_frame(
                np.zeros((2, 3), dtype=np.uint8),
                3,
                2,
            )
        with patch(
            "ARrobots.HMI.vision_io.MAX_CAMERA_FRAME_PIXELS",
            3,
        ):
            with self.assertRaisesRegex(MotionInputError, "pixel count"):
                prepare_camera_preview_frame(source, 3, 2)
        first = CameraPreviewFrame(0, 0, preview)
        second = CameraPreviewFrame(0, 0, preview)
        self.assertIsNot(first, second)
        self.assertNotEqual(first, second)

    def test_bounded_loader_decodes_valid_image_from_admitted_bytes(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "template.png"
            source = np.zeros((7, 11, 3), dtype=np.uint8)
            source[2, 3] = (10, 20, 30)
            self.assertTrue(cv2.imwrite(str(image_path), source))

            loaded = load_bounded_vision_image(
                image_path,
                cv2.IMREAD_COLOR,
                "vision template",
            )

        self.assertEqual(loaded.shape, source.shape)
        np.testing.assert_array_equal(loaded[2, 3], source[2, 3])

    def test_bounded_loader_uses_stored_geometry_for_exif_rotated_jpeg(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "rotated.jpg"
            exif = Image.Exif()
            exif[274] = 6
            Image.new("RGB", (3, 5), color=(10, 20, 30)).save(
                image_path,
                exif=exif,
            )

            loaded = load_bounded_vision_image(
                image_path,
                cv2.IMREAD_COLOR,
                "vision template",
            )

        self.assertEqual(loaded.shape, (5, 3, 3))

    def test_bounded_loader_normalizes_pillow_decompression_bomb_errors(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "template.jpg"
            image_path.write_bytes(b"bounded")
            with patch(
                "ARrobots.HMI.vision_io.Image.open",
                side_effect=Image.DecompressionBombError("declared image too large"),
            ):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "could not be decoded",
                ):
                    load_bounded_vision_image(
                        image_path,
                        cv2.IMREAD_COLOR,
                        "vision template",
                    )

    def test_bounded_loader_rejects_unsupported_decode_modes(self):
        with self.assertRaisesRegex(MotionInputError, "decode mode"):
            load_bounded_vision_image(
                "unused.jpg",
                cv2.IMREAD_ANYDEPTH,
                "vision template",
            )

    def test_bounded_loader_normalizes_opencv_decode_errors(self):
        with BoundedTemporaryDirectory() as directory:
            image_path = Path(directory) / "template.png"
            Image.new("RGB", (3, 5), color=(10, 20, 30)).save(image_path)
            with patch(
                "ARrobots.HMI.vision_io.cv2.imdecode",
                side_effect=cv2.error("decode failed"),
            ):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "could not be decoded",
                ):
                    load_bounded_vision_image(
                        image_path,
                        cv2.IMREAD_COLOR,
                        "vision template",
                    )

    def test_bounded_loader_rejects_directory_corruption_and_size_overflow(self):
        with BoundedTemporaryDirectory() as directory:
            root = Path(directory)
            corrupt_path = root / "corrupt.jpg"
            corrupt_path.write_bytes(b"not an image")
            with self.assertRaisesRegex(MotionInputError, "could not be decoded"):
                load_bounded_vision_image(
                    corrupt_path,
                    cv2.IMREAD_COLOR,
                    "vision template",
                )
            with self.assertRaisesRegex(
                MotionInputError,
                "regular file|could not be read",
            ):
                load_bounded_vision_image(
                    root,
                    cv2.IMREAD_COLOR,
                    "vision template",
                )

            oversized_path = root / "oversized.jpg"
            oversized_path.write_bytes(b"12345")
            with patch(
                "ARrobots.HMI.vision_io.MAX_VISION_IMAGE_BYTES",
                4,
            ):
                with self.assertRaisesRegex(
                    MotionInputError,
                    "file-size limit",
                ):
                    load_bounded_vision_image(
                        oversized_path,
                        cv2.IMREAD_COLOR,
                        "vision template",
                    )

    def test_preview_square_preserves_extreme_aspects_with_positive_sides(self):
        wide = np.zeros((1, 8192, 3), dtype=np.uint8)
        tall = np.zeros((8192, 1, 3), dtype=np.uint8)

        wide_preview = fit_vision_preview_square(wide, 150)
        tall_preview = fit_vision_preview_square(tall, 150)

        self.assertEqual(wide_preview.shape, (150, 150, 3))
        self.assertEqual(tall_preview.shape, (150, 150, 3))

    def test_preview_square_rejects_invalid_target_and_image_shape(self):
        with self.assertRaisesRegex(MotionInputError, "preview size"):
            fit_vision_preview_square(np.zeros((1, 1, 3), dtype=np.uint8), 0)
        with self.assertRaisesRegex(MotionInputError, "three 8-bit channels"):
            fit_vision_preview_square(np.zeros((1, 1), dtype=np.uint8), 150)
        with self.assertRaisesRegex(MotionInputError, "8-bit channels"):
            fit_vision_preview_square(
                np.zeros((1, 1, 3), dtype=np.float32),
                150,
            )
        with patch(
            "ARrobots.HMI.vision_io.cv2.resize",
            side_effect=cv2.error("resize unavailable"),
        ):
            with self.assertRaisesRegex(MotionInputError, "preview resize"):
                fit_vision_preview_square(
                    np.zeros((1, 1, 3), dtype=np.uint8),
                    150,
                )


if __name__ == "__main__":
    unittest.main()
