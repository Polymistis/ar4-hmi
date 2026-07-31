from pathlib import Path
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
    fit_vision_preview_square,
    load_bounded_vision_image,
)


class VisionIoTests(unittest.TestCase):
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
        np.testing.assert_array_equal(loaded[2, 3], source[2, 3], strict=False)

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


if __name__ == "__main__":
    unittest.main()
