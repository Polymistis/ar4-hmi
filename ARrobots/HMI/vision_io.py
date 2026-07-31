"""Bounded image loading and preview formatting for the AR4 HMI."""

from io import BytesIO
import os
import stat
import warnings

import cv2
import numpy as np
from PIL import Image, UnidentifiedImageError

from ARrobots.HMI.joint_motion import MotionInputError


MAX_VISION_IMAGE_BYTES = 16 * 1024 * 1024
MAX_VISION_IMAGE_DIMENSION = 8192
MAX_VISION_IMAGE_PIXELS = 16 * 1024 * 1024


def _validated_image_dimensions(width, height, field_name):
    if (
        isinstance(width, bool)
        or isinstance(height, bool)
        or not isinstance(width, int)
        or not isinstance(height, int)
        or width <= 0
        or height <= 0
    ):
        raise MotionInputError(f"{field_name} dimensions are invalid")
    if width > MAX_VISION_IMAGE_DIMENSION or height > MAX_VISION_IMAGE_DIMENSION:
        raise MotionInputError(f"{field_name} dimensions exceed the limit")
    if width * height > MAX_VISION_IMAGE_PIXELS:
        raise MotionInputError(f"{field_name} pixel count exceeds the limit")
    return width, height


def _read_regular_image_bytes(filename, field_name):
    if not isinstance(field_name, str) or not field_name.strip():
        raise TypeError("image field name must be nonempty text")
    try:
        candidate = os.fspath(filename)
    except TypeError as exc:
        raise MotionInputError(f"{field_name} path is invalid") from exc
    if isinstance(candidate, bytes):
        candidate = os.fsdecode(candidate)
    if not isinstance(candidate, str) or not candidate:
        raise MotionInputError(f"{field_name} path is invalid")

    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0)
    no_follow = getattr(os, "O_NOFOLLOW", 0)
    if isinstance(no_follow, int):
        flags |= no_follow
    descriptor = None
    try:
        path_state = os.stat(candidate, follow_symlinks=False)
        if not stat.S_ISREG(path_state.st_mode):
            raise MotionInputError(f"{field_name} must be a regular file")
        if path_state.st_size <= 0:
            raise MotionInputError(f"{field_name} is empty")
        if path_state.st_size > MAX_VISION_IMAGE_BYTES:
            raise MotionInputError(f"{field_name} exceeds the file-size limit")
        descriptor = os.open(candidate, flags)
        file_state = os.fstat(descriptor)
        if not stat.S_ISREG(file_state.st_mode):
            raise MotionInputError(f"{field_name} must be a regular file")
        if (
            file_state.st_dev != path_state.st_dev
            or file_state.st_ino != path_state.st_ino
            or file_state.st_size != path_state.st_size
        ):
            raise MotionInputError(f"{field_name} changed before being read")
        if file_state.st_size <= 0:
            raise MotionInputError(f"{field_name} is empty")
        if file_state.st_size > MAX_VISION_IMAGE_BYTES:
            raise MotionInputError(f"{field_name} exceeds the file-size limit")
        with os.fdopen(descriptor, "rb", closefd=True) as source:
            descriptor = None
            payload = source.read(MAX_VISION_IMAGE_BYTES + 1)
            final_state = os.fstat(source.fileno())
        if len(payload) > MAX_VISION_IMAGE_BYTES:
            raise MotionInputError(f"{field_name} exceeds the file-size limit")
        if (
            not stat.S_ISREG(final_state.st_mode)
            or final_state.st_dev != file_state.st_dev
            or final_state.st_ino != file_state.st_ino
            or final_state.st_size != file_state.st_size
            or len(payload) != file_state.st_size
        ):
            raise MotionInputError(f"{field_name} changed while being read")
        return payload
    except MotionInputError:
        raise
    except OSError as exc:
        raise MotionInputError(f"{field_name} could not be read") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def load_bounded_vision_image(filename, decode_mode, field_name):
    if isinstance(decode_mode, bool) or not isinstance(decode_mode, int):
        raise TypeError("image decode mode must be an integer")
    if decode_mode not in (
        cv2.IMREAD_COLOR,
        cv2.IMREAD_GRAYSCALE,
        cv2.IMREAD_UNCHANGED,
    ):
        raise MotionInputError("image decode mode is unsupported")
    payload = _read_regular_image_bytes(filename, field_name)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", Image.DecompressionBombWarning)
            with Image.open(BytesIO(payload)) as header:
                header_width, header_height = _validated_image_dimensions(
                    header.width,
                    header.height,
                    field_name,
                )
                header.verify()
    except MotionInputError:
        raise
    except (
        Image.DecompressionBombError,
        Image.DecompressionBombWarning,
        OSError,
        SyntaxError,
        UnidentifiedImageError,
    ) as exc:
        raise MotionInputError(f"{field_name} could not be decoded") from exc

    encoded = np.frombuffer(payload, dtype=np.uint8)
    decode_flags = decode_mode
    if decode_mode != cv2.IMREAD_UNCHANGED:
        decode_flags |= cv2.IMREAD_IGNORE_ORIENTATION
    try:
        image = cv2.imdecode(encoded, decode_flags)
    except cv2.error as exc:
        raise MotionInputError(f"{field_name} could not be decoded") from exc
    if image is None:
        raise MotionInputError(f"{field_name} could not be decoded")
    if not isinstance(image, np.ndarray) or image.ndim not in (2, 3):
        raise MotionInputError(f"{field_name} decoded shape is invalid")
    decoded_height, decoded_width = image.shape[:2]
    _validated_image_dimensions(decoded_width, decoded_height, field_name)
    if (decoded_width, decoded_height) != (header_width, header_height):
        raise MotionInputError(f"{field_name} decoded dimensions changed")
    return image


def fit_vision_preview_square(image, target_size):
    if (
        isinstance(target_size, bool)
        or not isinstance(target_size, int)
        or target_size <= 0
        or target_size > MAX_VISION_IMAGE_DIMENSION
    ):
        raise MotionInputError("vision preview size is invalid")
    if (
        not isinstance(image, np.ndarray)
        or image.ndim != 3
        or image.shape[2] != 3
        or image.dtype != np.uint8
    ):
        raise MotionInputError(
            "vision preview image must contain three 8-bit channels"
        )
    height, width = image.shape[:2]
    _validated_image_dimensions(width, height, "vision preview image")

    if width >= height:
        resized_width = target_size
        resized_height = max(1, round(height * target_size / width))
    else:
        resized_height = target_size
        resized_width = max(1, round(width * target_size / height))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA,
    )
    square = np.zeros((target_size, target_size, 3), dtype=np.uint8)
    x_offset = (target_size - resized_width) // 2
    y_offset = (target_size - resized_height) // 2
    square[
        y_offset:y_offset + resized_height,
        x_offset:x_offset + resized_width,
    ] = resized
    return square
