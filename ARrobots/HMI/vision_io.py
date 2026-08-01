"""Bounded image loading and camera-preview support for the AR4 HMI."""

from collections import deque
from dataclasses import dataclass
from io import BytesIO
import math
import os
import stat
import threading
import time

import cv2
import numpy as np
from PIL import Image, UnidentifiedImageError

from ARrobots.HMI.joint_motion import MotionInputError


MAX_VISION_IMAGE_BYTES = 16 * 1024 * 1024
MAX_VISION_IMAGE_DIMENSION = 8192
MAX_VISION_IMAGE_PIXELS = 16 * 1024 * 1024
MAX_CAMERA_FRAME_DIMENSION = 8192
MAX_CAMERA_FRAME_PIXELS = 32 * 1024 * 1024
CAMERA_PREVIEW_WARMUP_FRAMES = 5
MAX_CAMERA_PREVIEW_WARMUP_FRAMES = 120
CAMERA_PREVIEW_READ_FAILURE_LIMIT = 3
MAX_CAMERA_PREVIEW_READ_FAILURE_LIMIT = 120
CAMERA_PREVIEW_RETRY_SECONDS = 0.05
MAX_CAMERA_PREVIEW_RETRY_SECONDS = 5.0
CAMERA_PREVIEW_RELEASE_ATTEMPTS = 3
MAX_CAMERA_PREVIEW_RELEASE_ATTEMPTS = 10
MAX_CAMERA_PREVIEW_EVENT_DETAIL = 512
MAX_CAMERA_PREVIEW_EVENTS = 64
MAX_RETAINED_CAMERA_TERMINAL_EVENTS = 32
MAX_CAMERA_SOURCE_TEXT = 512
CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS = 0.05
MAX_VISION_OPERATION_EVENTS = 64
MAX_VISION_TEMPLATE_FILENAME = 255
VISION_CAPTURE_ADJUSTMENT_MINIMUM = -127
VISION_CAPTURE_ADJUSTMENT_MAXIMUM = 127
VISION_CAPTURE_ZOOM_MINIMUM = 1
VISION_CAPTURE_ZOOM_MAXIMUM = 50
VISION_CAPTURE_DISPLAY_WIDTH = 640
VISION_CAPTURE_DISPLAY_HEIGHT = 480
CAMERA_PREVIEW_EVENT_KINDS = frozenset(
    ("starting", "started", "stopping", "stopped", "failed")
)


@dataclass(frozen=True)
class CameraPreviewEvent:
    sequence: int
    kind: str
    request_id: int
    detail: str | None = None

    def __post_init__(self):
        _validate_nonnegative_integer(self.sequence, "camera event sequence")
        _validate_nonnegative_integer(self.request_id, "camera request id")
        if self.kind not in CAMERA_PREVIEW_EVENT_KINDS:
            raise MotionInputError("camera event kind is invalid")
        if self.detail is not None and (
            not isinstance(self.detail, str)
            or not self.detail
            or " ".join(self.detail.split()) != self.detail
            or self.detail != self.detail.strip()
            or "\r" in self.detail
            or "\n" in self.detail
            or len(self.detail) > MAX_CAMERA_PREVIEW_EVENT_DETAIL
        ):
            raise MotionInputError("camera event detail must be normalized text")


@dataclass(frozen=True, eq=False)
class CameraPreviewFrame:
    sequence: int
    request_id: int
    image: np.ndarray

    def __post_init__(self):
        _validate_nonnegative_integer(self.sequence, "camera frame sequence")
        _validate_nonnegative_integer(self.request_id, "camera request id")
        _validated_camera_frame(self.image, "camera preview frame")


@dataclass(frozen=True)
class CameraPreviewLifecycleState:
    active: bool
    stopped: bool
    closed: bool
    fault_reason: str | None

    def __post_init__(self):
        if not all(
            isinstance(value, bool)
            for value in (self.active, self.stopped, self.closed)
        ):
            raise MotionInputError("camera lifecycle state is invalid")
        if self.active == self.stopped:
            raise MotionInputError("camera lifecycle activity state is invalid")
        if self.fault_reason is not None and (
            not isinstance(self.fault_reason, str)
            or not self.fault_reason
            or self.fault_reason != self.fault_reason.strip()
            or " ".join(self.fault_reason.split()) != self.fault_reason
            or "\r" in self.fault_reason
            or "\n" in self.fault_reason
            or len(self.fault_reason) > MAX_CAMERA_PREVIEW_EVENT_DETAIL
        ):
            raise MotionInputError("camera lifecycle fault state is invalid")

    @property
    def clean(self):
        return self.stopped and self.fault_reason is None


@dataclass(frozen=True)
class VisionCaptureSettings:
    brightness: int
    contrast: int
    zoom_percent: int
    mask_bounds: tuple[int, int, int, int]
    auto_background: bool
    persist_auto_background: bool
    background_grayscale: int | None
    sample_points: tuple[tuple[int, int], ...]

    def __post_init__(self):
        for value, field_name in (
            (self.brightness, "vision brightness"),
            (self.contrast, "vision contrast"),
        ):
            _validate_bounded_integer(
                value,
                field_name,
                VISION_CAPTURE_ADJUSTMENT_MINIMUM,
                VISION_CAPTURE_ADJUSTMENT_MAXIMUM,
            )
        _validate_bounded_integer(
            self.zoom_percent,
            "vision zoom",
            VISION_CAPTURE_ZOOM_MINIMUM,
            VISION_CAPTURE_ZOOM_MAXIMUM,
        )
        if not isinstance(self.mask_bounds, tuple) or len(self.mask_bounds) != 4:
            raise MotionInputError("vision mask bounds are invalid")
        x_minimum, y_minimum, x_maximum, y_maximum = self.mask_bounds
        for value, field_name in zip(
            self.mask_bounds,
            (
                "vision mask minimum X",
                "vision mask minimum Y",
                "vision mask maximum X",
                "vision mask maximum Y",
            ),
        ):
            _validate_bounded_integer(
                value,
                field_name,
                0,
                MAX_CAMERA_FRAME_DIMENSION,
            )
        if x_minimum + 1 >= x_maximum or y_minimum + 1 >= y_maximum:
            raise MotionInputError("vision mask bounds must enclose an area")
        if not isinstance(self.auto_background, bool):
            raise MotionInputError("vision automatic-background state is invalid")
        if not isinstance(self.persist_auto_background, bool):
            raise MotionInputError(
                "vision automatic-background persistence state is invalid"
            )
        if self.auto_background:
            if self.background_grayscale is not None:
                raise MotionInputError(
                    "automatic vision background cannot include a manual value"
                )
            if (
                not isinstance(self.sample_points, tuple)
                or len(self.sample_points) != 3
            ):
                raise MotionInputError(
                    "automatic vision background requires three sample points"
                )
        else:
            _validate_bounded_integer(
                self.background_grayscale,
                "vision background grayscale",
                0,
                255,
            )
            if self.sample_points != ():
                raise MotionInputError(
                    "manual vision background cannot include sample points"
                )
        for index, point in enumerate(self.sample_points, start=1):
            if not isinstance(point, tuple) or len(point) != 2:
                raise MotionInputError(
                    f"vision sample point {index} is invalid"
                )
            for value, coordinate in zip(point, ("first", "second")):
                _validate_bounded_integer(
                    value,
                    f"vision sample point {index} {coordinate} coordinate",
                    0,
                    MAX_CAMERA_FRAME_DIMENSION,
                )


@dataclass(frozen=True, eq=False)
class VisionCaptureResult:
    image: np.ndarray
    display_image: np.ndarray
    zoom_percent: int
    auto_background: bool
    persist_auto_background: bool
    background_grayscale: int
    auto_background_rgb: tuple[int, int, int] | None

    def __post_init__(self):
        _validated_grayscale_image(self.image, "vision capture image")
        _validated_grayscale_image(
            self.display_image,
            "vision capture display image",
        )
        if self.display_image.shape != (
            VISION_CAPTURE_DISPLAY_HEIGHT,
            VISION_CAPTURE_DISPLAY_WIDTH,
        ):
            raise MotionInputError(
                "vision capture display dimensions are invalid"
            )
        _validate_bounded_integer(
            self.zoom_percent,
            "vision zoom",
            VISION_CAPTURE_ZOOM_MINIMUM,
            VISION_CAPTURE_ZOOM_MAXIMUM,
        )
        if not isinstance(self.auto_background, bool):
            raise MotionInputError("vision automatic-background state is invalid")
        if not isinstance(self.persist_auto_background, bool):
            raise MotionInputError(
                "vision automatic-background persistence state is invalid"
            )
        _validate_bounded_integer(
            self.background_grayscale,
            "vision background grayscale",
            0,
            255,
        )
        if self.auto_background:
            _validate_rgb_triplet(
                self.auto_background_rgb,
                "automatic vision background",
            )
        elif self.auto_background_rgb is not None:
            raise MotionInputError(
                "manual vision capture cannot return an automatic background"
            )


@dataclass(frozen=True)
class VisionCoordinateMapping:
    first_pixel_origin: float
    first_robot_origin: float
    second_pixel_origin: float
    second_robot_origin: float
    first_pixel_end: float
    first_robot_end: float
    second_pixel_end: float
    second_robot_end: float

    def __post_init__(self):
        for value, field_name in (
            (self.first_pixel_origin, "vision first-pixel origin"),
            (self.first_robot_origin, "vision first-robot origin"),
            (self.second_pixel_origin, "vision second-pixel origin"),
            (self.second_robot_origin, "vision second-robot origin"),
            (self.first_pixel_end, "vision first-pixel end"),
            (self.first_robot_end, "vision first-robot end"),
            (self.second_pixel_end, "vision second-pixel end"),
            (self.second_robot_end, "vision second-robot end"),
        ):
            _validate_finite_number(value, field_name)
        first_pixel_range = _validate_finite_number(
            self.first_pixel_end - self.first_pixel_origin,
            "vision first-pixel calibration span",
        )
        second_pixel_range = _validate_finite_number(
            self.second_pixel_end - self.second_pixel_origin,
            "vision second-pixel calibration span",
        )
        _validate_finite_number(
            self.first_robot_end - self.first_robot_origin,
            "vision first-robot calibration span",
        )
        _validate_finite_number(
            self.second_robot_end - self.second_robot_origin,
            "vision second-robot calibration span",
        )
        if first_pixel_range == 0 or second_pixel_range == 0:
            raise MotionInputError(
                "vision pixel-to-robot calibration spans must be nonzero"
            )

    def map_position(self, first_pixel, second_pixel):
        _validate_nonnegative_integer(first_pixel, "vision first-pixel result")
        _validate_nonnegative_integer(second_pixel, "vision second-pixel result")
        first_pixel_range = self.first_pixel_end - self.first_pixel_origin
        second_pixel_range = self.second_pixel_end - self.second_pixel_origin
        first_robot = self.first_robot_origin + (
            (first_pixel - self.first_pixel_origin)
            / first_pixel_range
            * (self.first_robot_end - self.first_robot_origin)
        )
        second_robot = self.second_robot_origin + (
            (second_pixel - self.second_pixel_origin)
            / second_pixel_range
            * (self.second_robot_end - self.second_robot_origin)
        )
        return (
            _validate_finite_number(first_robot, "vision first-robot result"),
            _validate_finite_number(second_robot, "vision second-robot result"),
        )


@dataclass(frozen=True)
class VisionMatchOptions:
    template_filename: str
    minimum_score: float
    full_rotation_search: bool
    pick_closest_180: bool
    try_closest_out_of_range: bool
    joint6_positive_limit: float
    joint6_negative_limit: float
    coordinate_mapping: VisionCoordinateMapping

    def __post_init__(self):
        if (
            not isinstance(self.template_filename, str)
            or not self.template_filename
            or self.template_filename != self.template_filename.strip()
            or os.path.basename(self.template_filename) != self.template_filename
            or "/" in self.template_filename
            or "\\" in self.template_filename
            or not self.template_filename.endswith(".jpg")
            or "\x00" in self.template_filename
            or "\r" in self.template_filename
            or "\n" in self.template_filename
            or len(self.template_filename) > MAX_VISION_TEMPLATE_FILENAME
        ):
            raise MotionInputError(
                "vision template must be a lowercase .jpg leaf filename"
            )
        minimum_score = _validate_finite_number(
            self.minimum_score,
            "vision minimum score",
        )
        if minimum_score < 0 or minimum_score > 1:
            raise MotionInputError(
                "vision minimum score must be between 0 and 1"
            )
        for value, field_name in (
            (self.full_rotation_search, "vision full-rotation state"),
            (self.pick_closest_180, "vision closest-180 state"),
            (
                self.try_closest_out_of_range,
                "vision out-of-range fallback state",
            ),
        ):
            if not isinstance(value, bool):
                raise MotionInputError(f"{field_name} must be boolean")
        for value, field_name in (
            (self.joint6_positive_limit, "vision J6 positive limit"),
            (self.joint6_negative_limit, "vision J6 negative limit"),
        ):
            limit = _validate_finite_number(value, field_name)
            if limit < 0:
                raise MotionInputError(f"{field_name} must be non-negative")
        if not isinstance(self.coordinate_mapping, VisionCoordinateMapping):
            raise MotionInputError("vision coordinate mapping is invalid")


@dataclass(frozen=True)
class VisionMatchSettings:
    capture_settings: VisionCaptureSettings
    match_options: VisionMatchOptions

    def __post_init__(self):
        if not isinstance(self.capture_settings, VisionCaptureSettings):
            raise MotionInputError("vision match capture settings are invalid")
        if not isinstance(self.match_options, VisionMatchOptions):
            raise MotionInputError("vision match options are invalid")


@dataclass(frozen=True, eq=False)
class VisionMatchResult:
    matched: bool
    score: float
    angle_degrees: float | None
    pixel_position: tuple[int, int] | None
    robot_position: tuple[float, float] | None
    annotated_image: np.ndarray
    display_image: np.ndarray

    def __post_init__(self):
        if not isinstance(self.matched, bool):
            raise MotionInputError("vision match state is invalid")
        score = _validate_finite_number(self.score, "vision template score")
        if score < -1 or score > 1:
            raise MotionInputError(
                "vision template score must be between -1 and 1"
            )
        _validated_camera_frame(
            self.annotated_image,
            "vision annotated frame",
        )
        _validated_camera_frame(
            self.display_image,
            "vision match display frame",
        )
        if self.display_image.shape != (
            VISION_CAPTURE_DISPLAY_HEIGHT,
            VISION_CAPTURE_DISPLAY_WIDTH,
            3,
        ):
            raise MotionInputError("vision match display dimensions are invalid")
        optional_values = (
            self.angle_degrees,
            self.pixel_position,
            self.robot_position,
        )
        if self.matched != all(value is not None for value in optional_values):
            raise MotionInputError(
                "vision match coordinates are inconsistent with the result"
            )
        if not self.matched:
            return
        angle = _validate_finite_number(
            self.angle_degrees,
            "vision match angle",
        )
        if angle < -180 or angle > 180:
            raise MotionInputError("vision match angle is outside normalization")
        if (
            not isinstance(self.pixel_position, tuple)
            or len(self.pixel_position) != 2
        ):
            raise MotionInputError("vision match pixel position is invalid")
        for value in self.pixel_position:
            _validate_nonnegative_integer(value, "vision match pixel position")
        if (
            not isinstance(self.robot_position, tuple)
            or len(self.robot_position) != 2
        ):
            raise MotionInputError("vision match robot position is invalid")
        for value in self.robot_position:
            _validate_finite_number(value, "vision match robot position")


@dataclass(frozen=True, eq=False)
class VisionMatchOperationResult:
    capture_result: VisionCaptureResult
    match_result: VisionMatchResult
    match_options: VisionMatchOptions

    def __post_init__(self):
        if not isinstance(self.capture_result, VisionCaptureResult):
            raise MotionInputError("vision match capture result is invalid")
        if not isinstance(self.match_result, VisionMatchResult):
            raise MotionInputError("vision match result is invalid")
        if not isinstance(self.match_options, VisionMatchOptions):
            raise MotionInputError("vision match result options are invalid")


@dataclass(frozen=True, eq=False)
class VisionOperationEvent:
    sequence: int
    request_id: int
    result: VisionCaptureResult | VisionMatchOperationResult | None = None
    error_detail: str | None = None

    def __post_init__(self):
        _validate_nonnegative_integer(self.sequence, "vision event sequence")
        _validate_nonnegative_integer(self.request_id, "vision request id")
        if (self.result is None) == (self.error_detail is None):
            raise MotionInputError(
                "vision operation event must contain one terminal outcome"
            )
        if self.result is not None and not isinstance(
            self.result,
            (VisionCaptureResult, VisionMatchOperationResult),
        ):
            raise MotionInputError("vision operation result is invalid")
        if self.error_detail is not None and (
            not isinstance(self.error_detail, str)
            or not self.error_detail
            or self.error_detail != self.error_detail.strip()
            or " ".join(self.error_detail.split()) != self.error_detail
            or "\r" in self.error_detail
            or "\n" in self.error_detail
            or len(self.error_detail) > MAX_CAMERA_PREVIEW_EVENT_DETAIL
        ):
            raise MotionInputError("vision operation error detail is invalid")


@dataclass(frozen=True)
class VisionOperationDrainState:
    events: tuple
    active: bool
    active_request_id: int | None
    pending_request_id: int | None

    def __post_init__(self):
        if not isinstance(self.events, tuple):
            raise MotionInputError("vision drain events must be a tuple")
        if not isinstance(self.active, bool):
            raise MotionInputError("vision drain active state must be boolean")
        for field_name, value in (
            ("vision drain active request id", self.active_request_id),
            ("vision drain pending request id", self.pending_request_id),
        ):
            if value is not None:
                _validate_nonnegative_integer(value, field_name)
        if not self.active and (
            self.active_request_id is not None
            or self.pending_request_id is not None
        ):
            raise MotionInputError(
                "an inactive vision drain cannot retain request ownership"
            )


@dataclass(frozen=True)
class VisionOperationSubmission:
    request_id: int
    coalesced: bool

    def __post_init__(self):
        _validate_nonnegative_integer(self.request_id, "vision request id")
        if not isinstance(self.coalesced, bool):
            raise MotionInputError("vision coalescing state is invalid")


@dataclass(frozen=True)
class _VisionOperationRequest:
    request_id: int
    settings: object
    cancellation_event: object = None

    def __post_init__(self):
        _validate_nonnegative_integer(self.request_id, "vision request id")
        if self.settings is None:
            raise MotionInputError("vision operation settings are invalid")
        if self.cancellation_event is not None:
            _camera_cancellation_requested(self.cancellation_event)


@dataclass(frozen=True)
class _CameraPreviewRequest:
    request_id: int
    camera_source: int | str

    def __post_init__(self):
        _validate_nonnegative_integer(self.request_id, "camera request id")
        _validate_camera_source(self.camera_source)


def _validate_nonnegative_integer(value, field_name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise MotionInputError(f"{field_name} must be a non-negative integer")
    return value


def _validate_bounded_integer(value, field_name, minimum, maximum):
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
        or value > maximum
    ):
        raise MotionInputError(
            f"{field_name} must be an integer between {minimum} and {maximum}"
        )
    return value


def _validate_finite_number(value, field_name):
    if isinstance(value, bool) or not isinstance(
        value,
        (int, float, np.integer, np.floating),
    ):
        raise MotionInputError(f"{field_name} must be numeric")
    number = float(value)
    if not np.isfinite(number):
        raise MotionInputError(f"{field_name} must be finite")
    return number


def _validate_rgb_triplet(value, field_name):
    if not isinstance(value, tuple) or len(value) != 3:
        raise MotionInputError(f"{field_name} must contain three byte values")
    for component in value:
        _validate_bounded_integer(component, field_name, 0, 255)
    return value


def _validate_camera_source(value):
    if isinstance(value, int) and not isinstance(value, bool):
        if value < 0:
            raise MotionInputError(
                "camera source index must be a non-negative integer"
            )
        return value
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or "\r" in value
        or "\n" in value
        or "\x00" in value
        or len(value) > MAX_CAMERA_SOURCE_TEXT
    ):
        raise MotionInputError(
            "camera source must be a normalized device index or name"
        )
    return value


def _validate_camera_wait_timeout(value):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or value < 0
        or value > threading.TIMEOUT_MAX
    ):
        raise MotionInputError(
            "camera wait timeout must be a finite non-negative number"
        )
    return float(value)


def _camera_cancellation_requested(cancellation_event):
    if cancellation_event is None:
        return False
    is_set = getattr(cancellation_event, "is_set", None)
    if not callable(is_set):
        raise MotionInputError(
            "camera cancellation event does not satisfy the event contract"
        )
    cancelled = is_set()
    if not isinstance(cancelled, bool):
        raise MotionInputError("camera cancellation state must be boolean")
    return cancelled


def normalize_camera_exception_detail(error, prefix=""):
    """Return bounded, single-line diagnostic text for camera failures."""

    if (
        not isinstance(prefix, str)
        or prefix != prefix.lstrip()
        or "\r" in prefix
        or "\n" in prefix
    ):
        raise TypeError("camera error prefix must be normalized text")
    try:
        detail = " ".join(str(error).split())
    except Exception:
        detail = type(error).__name__
    detail = detail or type(error).__name__
    bounded = (prefix + detail)[:MAX_CAMERA_PREVIEW_EVENT_DETAIL].rstrip()
    return bounded or type(error).__name__[:MAX_CAMERA_PREVIEW_EVENT_DETAIL]


def _validated_camera_frame(image, field_name):
    if (
        not isinstance(image, np.ndarray)
        or image.ndim != 3
        or image.shape[2] != 3
        or image.dtype != np.uint8
    ):
        raise MotionInputError(
            f"{field_name} must contain three 8-bit channels"
        )
    height, width = image.shape[:2]
    if width <= 0 or height <= 0:
        raise MotionInputError(f"{field_name} dimensions are invalid")
    if (
        width > MAX_CAMERA_FRAME_DIMENSION
        or height > MAX_CAMERA_FRAME_DIMENSION
    ):
        raise MotionInputError(f"{field_name} dimensions exceed the limit")
    if width * height > MAX_CAMERA_FRAME_PIXELS:
        raise MotionInputError(f"{field_name} pixel count exceeds the limit")
    return image


def _validated_grayscale_image(image, field_name):
    if (
        not isinstance(image, np.ndarray)
        or image.ndim != 2
        or image.dtype != np.uint8
    ):
        raise MotionInputError(f"{field_name} must contain 8-bit grayscale data")
    height, width = image.shape
    _validated_image_dimensions(width, height, field_name)
    return image


def copy_camera_capture_frame(image):
    """Validate camera output and detach storage owned by the backend."""

    source = _validated_camera_frame(image, "camera capture frame")
    return np.array(source, dtype=np.uint8, copy=True, order="C")


def prepare_camera_preview_frame(image, width=480, height=320):
    """Convert an untrusted BGR capture into an owned RGB preview frame."""

    _validate_nonnegative_integer(width, "camera preview width")
    _validate_nonnegative_integer(height, "camera preview height")
    if width == 0 or height == 0:
        raise MotionInputError("camera preview dimensions must be positive")
    _validated_image_dimensions(width, height, "camera preview")
    source = _validated_camera_frame(image, "camera capture frame")
    try:
        resized = cv2.resize(
            source,
            (width, height),
            interpolation=(
                cv2.INTER_AREA
                if source.shape[1] >= width and source.shape[0] >= height
                else cv2.INTER_LINEAR
            ),
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    except cv2.error as exc:
        raise MotionInputError("camera preview conversion failed") from exc
    return np.array(
        _validated_camera_frame(rgb, "camera preview frame"),
        dtype=np.uint8,
        copy=True,
        order="C",
    )


def prepare_vision_capture_result(image, settings):
    """Apply one immutable vision-input snapshot away from Tk."""

    source = _validated_camera_frame(image, "vision capture frame")
    _validated_image_dimensions(
        source.shape[1],
        source.shape[0],
        "vision capture frame",
    )
    if not isinstance(settings, VisionCaptureSettings):
        raise MotionInputError("vision capture settings are invalid")
    try:
        adjusted = np.int16(source)
        adjusted = (
            adjusted * (settings.contrast / 127 + 1)
            - settings.contrast
            + settings.brightness
        )
        adjusted = np.uint8(np.clip(adjusted, 0, 255))
        grayscale = cv2.cvtColor(adjusted, cv2.COLOR_BGR2GRAY)
        height, width = grayscale.shape
        row_radius = int(settings.zoom_percent * height / 100)
        column_radius = int(settings.zoom_percent * width / 100)
        row_center = int(height / 2)
        column_center = int(width / 2)
        cropped = grayscale[
            row_center - row_radius:row_center + row_radius,
            column_center - column_radius:column_center + column_radius,
        ]
        if cropped.size == 0:
            raise MotionInputError(
                "vision zoom produced an empty capture region"
            )
        grayscale = cv2.resize(cropped, (width, height))
    except MotionInputError:
        raise
    except (cv2.error, TypeError, ValueError, OverflowError) as exc:
        raise MotionInputError("vision capture conversion failed") from exc

    if settings.auto_background:
        samples = []
        for index, (first, second) in enumerate(
            settings.sample_points,
            start=1,
        ):
            if first >= height or second >= width:
                raise MotionInputError(
                    f"vision sample point {index} is outside the current image"
                )
            samples.append(int(grayscale[first, second]))
        background = int(np.rint(np.mean(np.asarray(samples, dtype=np.float64))))
        auto_background_rgb = (background, background, background)
    else:
        background = settings.background_grayscale
        auto_background_rgb = None

    x_minimum, y_minimum, x_maximum, y_maximum = settings.mask_bounds
    masked = np.full_like(grayscale, background)
    inner_x_minimum = min(max(x_minimum + 1, 0), width)
    inner_x_maximum = min(max(x_maximum, 0), width)
    inner_y_minimum = min(max(y_minimum + 1, 0), height)
    inner_y_maximum = min(max(y_maximum, 0), height)
    if (
        inner_x_minimum < inner_x_maximum
        and inner_y_minimum < inner_y_maximum
    ):
        masked[
            inner_y_minimum:inner_y_maximum,
            inner_x_minimum:inner_x_maximum,
        ] = grayscale[
            inner_y_minimum:inner_y_maximum,
            inner_x_minimum:inner_x_maximum,
        ]
    try:
        display_image = cv2.resize(
            masked,
            (VISION_CAPTURE_DISPLAY_WIDTH, VISION_CAPTURE_DISPLAY_HEIGHT),
            interpolation=cv2.INTER_LINEAR,
        )
    except cv2.error as exc:
        raise MotionInputError("vision display conversion failed") from exc
    captured = np.array(masked, dtype=np.uint8, copy=True, order="C")
    display = np.array(
        display_image,
        dtype=np.uint8,
        copy=True,
        order="C",
    )
    captured.setflags(write=False)
    display.setflags(write=False)
    return VisionCaptureResult(
        image=captured,
        display_image=display,
        zoom_percent=settings.zoom_percent,
        auto_background=settings.auto_background,
        persist_auto_background=settings.persist_auto_background,
        background_grayscale=background,
        auto_background_rgb=auto_background_rgb,
    )


def _raise_if_vision_match_cancelled(cancellation_event):
    if _camera_cancellation_requested(cancellation_event):
        raise MotionInputError("vision matching was cancelled")


def _rotate_vision_template(template, angle, background):
    source = _validated_grayscale_image(template, "vision template")
    angle = _validate_finite_number(angle, "vision template angle")
    _validate_bounded_integer(
        background,
        "vision template background",
        0,
        255,
    )
    image_center = tuple(np.asarray(source.shape[1::-1], dtype=float) / 2)
    try:
        rotation = cv2.getRotationMatrix2D(image_center, -angle, 1.0)
        rotated = cv2.warpAffine(
            source,
            rotation,
            source.shape[1::-1],
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=background,
            flags=cv2.INTER_LINEAR,
        )
    except cv2.error as exc:
        raise MotionInputError("vision template rotation failed") from exc
    return np.array(
        _validated_grayscale_image(rotated, "rotated vision template"),
        dtype=np.uint8,
        copy=True,
        order="C",
    )


def _vision_template_match(image, template):
    source = _validated_grayscale_image(image, "captured vision frame")
    candidate = _validated_grayscale_image(template, "rotated vision template")
    if (
        candidate.shape[0] > source.shape[0]
        or candidate.shape[1] > source.shape[1]
    ):
        raise MotionInputError(
            "vision template must not exceed the captured frame dimensions"
        )
    try:
        scores = cv2.matchTemplate(
            source,
            candidate,
            cv2.TM_CCOEFF_NORMED,
        )
        _, maximum, _, location = cv2.minMaxLoc(scores)
    except cv2.error as exc:
        raise MotionInputError("vision template comparison failed") from exc
    maximum = _validate_finite_number(maximum, "vision template score")
    if maximum < -1 or maximum > 1:
        raise MotionInputError(
            "vision template score must be between -1 and 1"
        )
    if (
        not isinstance(location, tuple)
        or len(location) != 2
        or any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in location
        )
    ):
        raise MotionInputError("vision template location is invalid")
    return maximum, location


def _vision_rotation_candidate(image, template, angle, background):
    rotated = _rotate_vision_template(template, angle, background)
    score, location = _vision_template_match(image, rotated)
    return score, float(angle), location, rotated.shape[1], rotated.shape[0]


def _best_narrow_vision_rotation(
    image,
    template,
    background,
    cancellation_event,
):
    best = None
    for angle in (0.0, 120.0, 240.0):
        _raise_if_vision_match_cancelled(cancellation_event)
        candidate = _vision_rotation_candidate(
            image,
            template,
            angle,
            background,
        )
        if best is None or candidate[0] > best[0]:
            best = candidate

    refinement = 180.0
    while refinement >= 0.9:
        for angle in (best[1] + refinement, best[1] - refinement):
            _raise_if_vision_match_cancelled(cancellation_event)
            candidate = _vision_rotation_candidate(
                image,
                template,
                angle,
                background,
            )
            if candidate[0] > best[0]:
                best = candidate
        refinement /= 2
    return best


def _best_full_vision_rotation(
    image,
    template,
    background,
    minimum_score,
    cancellation_event,
):
    best = None
    for angle in range(360):
        _raise_if_vision_match_cancelled(cancellation_event)
        candidate = _vision_rotation_candidate(
            image,
            template,
            angle,
            background,
        )
        if best is None or candidate[0] > best[0]:
            best = candidate
        if candidate[0] >= minimum_score:
            break
    return best


def _normalized_vision_match_angle(angle, pick_closest_180):
    angle = _validate_finite_number(angle, "vision match angle") % 360
    if angle > 180:
        angle -= 360
    if pick_closest_180:
        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180
    return angle


def _draw_vision_match_axes(image, column, row, angle):
    green = (0, 255, 0)
    dark_green = (0, 128, 0)
    first_end = (
        int(column + 60 * math.cos(math.radians(angle - 90))),
        int(row + 60 * math.sin(math.radians(angle - 90))),
    )
    second_end = (
        int(column + 60 * math.cos(math.radians(angle + 90))),
        int(row + 60 * math.sin(math.radians(angle + 90))),
    )
    third_end = (
        int(column + 30 * math.cos(math.radians(angle))),
        int(row + 30 * math.sin(math.radians(angle))),
    )
    fourth_end = (
        int(column + 30 * math.cos(math.radians(angle + 180))),
        int(row + 30 * math.sin(math.radians(angle + 180))),
    )
    cv2.line(image, (column, row), first_end, green, 3)
    cv2.line(image, (column, row), second_end, green, 3)
    cv2.line(image, (column, row), third_end, green, 3)
    cv2.line(image, (column, row), fourth_end, green, 3)
    tip_start = (
        int(column + 56 * math.cos(math.radians(angle - 90))),
        int(row + 56 * math.sin(math.radians(angle - 90))),
    )
    cv2.line(image, tip_start, first_end, dark_green, 2)
    cv2.circle(image, (column, row), 20, green, 1)


def prepare_vision_match_result(
    image,
    template,
    background,
    options,
    cancellation_event=None,
):
    """Match one captured frame from immutable inputs away from Tk."""

    source = _validated_grayscale_image(image, "captured vision frame")
    candidate = _validated_grayscale_image(template, "vision template")
    if not isinstance(options, VisionMatchOptions):
        raise MotionInputError("vision match options are invalid")
    _validate_bounded_integer(
        background,
        "vision template background",
        0,
        255,
    )
    if (
        candidate.shape[0] > source.shape[0]
        or candidate.shape[1] > source.shape[1]
    ):
        raise MotionInputError(
            "vision template must not exceed the captured frame dimensions"
        )
    _raise_if_vision_match_cancelled(cancellation_event)
    if options.full_rotation_search:
        best = _best_full_vision_rotation(
            source,
            candidate,
            background,
            options.minimum_score,
            cancellation_event,
        )
    else:
        best = _best_narrow_vision_rotation(
            source,
            candidate,
            background,
            cancellation_event,
        )
    _raise_if_vision_match_cancelled(cancellation_event)

    score, angle, location, width, height = best
    matched = score >= options.minimum_score
    angle_degrees = None
    pixel_position = None
    robot_position = None
    try:
        annotated = cv2.cvtColor(source, cv2.COLOR_GRAY2BGR)
        if matched:
            angle = _normalized_vision_match_angle(
                angle,
                options.pick_closest_180,
            )
            if angle > options.joint6_positive_limit:
                if options.try_closest_out_of_range:
                    angle = options.joint6_positive_limit
                else:
                    matched = False
            if angle < -options.joint6_negative_limit:
                if options.try_closest_out_of_range:
                    angle = -options.joint6_negative_limit
                else:
                    matched = False

            column = int(location[0] + width / 2)
            row = int(location[1] + height / 2)
            _draw_vision_match_axes(annotated, column, row, angle)
            if matched:
                angle_degrees = angle
                pixel_position = (row, column)
                robot_position = options.coordinate_mapping.map_position(
                    row,
                    column,
                )
        if not matched:
            cv2.rectangle(
                annotated,
                (5, 5),
                (max(5, source.shape[1] - 5), max(5, source.shape[0] - 5)),
                (0, 0, 255),
                5,
            )
        display_bgr = cv2.resize(
            annotated,
            (VISION_CAPTURE_DISPLAY_WIDTH, VISION_CAPTURE_DISPLAY_HEIGHT),
            interpolation=cv2.INTER_LINEAR,
        )
        display_rgb = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2RGB)
    except MotionInputError:
        raise
    except (cv2.error, TypeError, ValueError, OverflowError) as exc:
        raise MotionInputError("vision match presentation failed") from exc

    annotated_result = np.array(
        annotated,
        dtype=np.uint8,
        copy=True,
        order="C",
    )
    display_result = np.array(
        display_rgb,
        dtype=np.uint8,
        copy=True,
        order="C",
    )
    annotated_result.setflags(write=False)
    display_result.setflags(write=False)
    return VisionMatchResult(
        matched=matched,
        score=score,
        angle_degrees=angle_degrees,
        pixel_position=pixel_position,
        robot_position=robot_position,
        annotated_image=annotated_result,
        display_image=display_result,
    )


class _CombinedVisionCancellation:
    def __init__(self, first, second):
        for event in (first, second):
            _camera_cancellation_requested(event)
        self._first = first
        self._second = second

    def is_set(self):
        return _camera_cancellation_requested(
            self._first
        ) or _camera_cancellation_requested(self._second)

    def wait(self, timeout=None):
        if timeout is not None:
            timeout = _validate_camera_wait_timeout(timeout)
        deadline = None if timeout is None else time.monotonic() + timeout
        while not self.is_set():
            wait_seconds = CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                wait_seconds = min(wait_seconds, remaining)
            self._first.wait(wait_seconds)
        return True


class VisionOperationWorker:
    """Run validated, optionally coalesced vision work away from Tk."""

    def __init__(
        self,
        operation,
        settings_type,
        result_type,
        operation_name,
        thread_name,
        *,
        coalesce=True,
        thread_factory=threading.Thread,
    ):
        if not callable(operation):
            raise MotionInputError("vision operation must be callable")
        if not isinstance(settings_type, type):
            raise MotionInputError("vision settings type is invalid")
        if not isinstance(result_type, type):
            raise MotionInputError("vision result type is invalid")
        if (settings_type, result_type) not in (
            (VisionCaptureSettings, VisionCaptureResult),
            (VisionMatchSettings, VisionMatchOperationResult),
        ):
            raise MotionInputError("vision operation type contract is invalid")
        for value, field_name in (
            (operation_name, "vision operation name"),
            (thread_name, "vision thread name"),
        ):
            if (
                not isinstance(value, str)
                or not value
                or value != value.strip()
                or "\r" in value
                or "\n" in value
                or len(value) > 64
            ):
                raise MotionInputError(f"{field_name} is invalid")
        if not isinstance(coalesce, bool):
            raise MotionInputError("vision coalescing state is invalid")
        if not callable(thread_factory):
            raise MotionInputError("vision thread factory must be callable")
        self._operation = operation
        self._settings_type = settings_type
        self._result_type = result_type
        self._operation_name = operation_name
        self._thread_name = thread_name
        self._coalesce = coalesce
        self._thread_factory = thread_factory
        self._lock = threading.Lock()
        self._events = deque(maxlen=MAX_VISION_OPERATION_EVENTS)
        self._next_event_sequence = 0
        self._next_request_id = 0
        self._pending = None
        self._active_request_id = None
        self._worker = None
        self._worker_stopped = threading.Event()
        self._worker_stopped.set()
        self._close_requested = threading.Event()
        self._closed = False

    @property
    def active(self):
        with self._lock:
            return self._worker is not None

    @property
    def closed(self):
        with self._lock:
            return self._closed

    @property
    def active_request_id(self):
        with self._lock:
            return self._active_request_id

    @property
    def pending_request_id(self):
        with self._lock:
            return None if self._pending is None else self._pending.request_id

    def submit(self, settings, cancellation_event=None):
        if not isinstance(settings, self._settings_type):
            raise MotionInputError(
                f"{self._operation_name} settings are invalid"
            )
        if cancellation_event is not None:
            if _camera_cancellation_requested(cancellation_event):
                raise MotionInputError(
                    f"{self._operation_name} was cancelled"
                )
        with self._lock:
            if self._closed:
                raise MotionInputError(
                    f"{self._operation_name} worker is closed"
                )
            if not self._coalesce and (
                self._worker is not None or self._pending is not None
            ):
                raise MotionInputError(
                    f"{self._operation_name} is already active"
                )
            self._next_request_id += 1
            request = _VisionOperationRequest(
                request_id=self._next_request_id,
                settings=settings,
                cancellation_event=cancellation_event,
            )
            coalesced = self._worker is not None or self._pending is not None
            self._pending = request
            if self._worker is not None:
                return VisionOperationSubmission(request.request_id, coalesced)

            self._worker_stopped.clear()
            try:
                worker = self._thread_factory(
                    target=self._run,
                    name=self._thread_name,
                    daemon=True,
                )
            except Exception as exc:
                self._pending = None
                self._worker_stopped.set()
                detail = normalize_camera_exception_detail(
                    exc,
                    f"{self._operation_name} thread creation failed: ",
                )
                raise RuntimeError(detail) from exc
            if not isinstance(worker, threading.Thread):
                self._pending = None
                self._worker_stopped.set()
                raise MotionInputError(
                    f"{self._operation_name} thread factory returned "
                    "an invalid worker"
                )
            self._worker = worker
            try:
                worker.start()
            except Exception as exc:
                self._worker = None
                self._pending = None
                self._worker_stopped.set()
                detail = normalize_camera_exception_detail(
                    exc,
                    f"{self._operation_name} worker startup failed: ",
                )
                raise RuntimeError(detail) from exc
            return VisionOperationSubmission(request.request_id, coalesced)

    def drain_events(self):
        return self.drain_events_state().events

    def drain_events_state(self):
        """Atomically drain outcomes and snapshot request ownership.

        Consumers treat an absent outcome and absent request ownership as a
        lost terminal event, so both observations must share one lock hold.
        """

        with self._lock:
            events = tuple(self._events)
            state = VisionOperationDrainState(
                events=events,
                active=self._worker is not None,
                active_request_id=self._active_request_id,
                pending_request_id=(
                    None
                    if self._pending is None
                    else self._pending.request_id
                ),
            )
            self._events.clear()
            return state

    def close(self):
        with self._lock:
            self._closed = True
            self._close_requested.set()
            pending = self._pending
            self._pending = None
            if pending is not None:
                self._append_event_locked(
                    pending.request_id,
                    error_detail=(
                        f"{self._operation_name} was cancelled during shutdown"
                    ),
                )
            return self._worker is None

    def wait_stopped(self, timeout=None):
        if timeout is not None:
            timeout = _validate_camera_wait_timeout(timeout)
        return self._worker_stopped.wait(timeout)

    def _append_event_locked(self, request_id, result=None, error_detail=None):
        event = VisionOperationEvent(
            sequence=self._next_event_sequence,
            request_id=request_id,
            result=result,
            error_detail=error_detail,
        )
        self._next_event_sequence += 1
        self._events.append(event)
        return event

    def _operation_cancellation(self, request):
        if request.cancellation_event is None:
            return self._close_requested
        return _CombinedVisionCancellation(
            self._close_requested,
            request.cancellation_event,
        )

    def _run(self):
        current_thread = threading.current_thread()
        try:
            while True:
                with self._lock:
                    request = self._pending
                    self._pending = None
                    if request is None or self._closed:
                        if self._worker is current_thread:
                            self._worker = None
                            self._active_request_id = None
                            self._worker_stopped.set()
                        return
                    self._active_request_id = request.request_id
                result = None
                error_detail = None
                try:
                    result = self._operation(
                        request.settings,
                        self._operation_cancellation(request),
                    )
                    if not isinstance(result, self._result_type):
                        raise MotionInputError(
                            f"{self._operation_name} returned an invalid result"
                        )
                except BaseException as exc:
                    result = None
                    error_detail = normalize_camera_exception_detail(
                        exc,
                        f"{self._operation_name} failed: ",
                    )
                with self._lock:
                    self._append_event_locked(
                        request.request_id,
                        result=result,
                        error_detail=error_detail,
                    )
                    self._active_request_id = None
                    if self._closed:
                        self._pending = None
        except BaseException as exc:
            detail = normalize_camera_exception_detail(
                exc,
                f"{self._operation_name} worker terminated: ",
            )
            with self._lock:
                request_ids = []
                if self._active_request_id is not None:
                    request_ids.append(self._active_request_id)
                if self._pending is not None:
                    request_ids.append(self._pending.request_id)
                self._active_request_id = None
                self._pending = None
                if self._worker is current_thread:
                    self._worker = None
                    self._worker_stopped.set()
                for request_id in dict.fromkeys(request_ids):
                    self._append_event_locked(
                        request_id,
                        error_detail=detail,
                    )
        finally:
            with self._lock:
                if self._worker is current_thread:
                    self._worker = None
                    self._active_request_id = None
                    self._worker_stopped.set()


class CameraPreviewWorker:
    """Own camera acquisition off Tk and retain only the latest frame."""

    def __init__(
        self,
        capture_factory,
        frame_transform=prepare_camera_preview_frame,
        *,
        warmup_frames=CAMERA_PREVIEW_WARMUP_FRAMES,
        read_failure_limit=CAMERA_PREVIEW_READ_FAILURE_LIMIT,
        retry_seconds=CAMERA_PREVIEW_RETRY_SECONDS,
        release_attempts=CAMERA_PREVIEW_RELEASE_ATTEMPTS,
        thread_factory=threading.Thread,
    ):
        if not callable(capture_factory):
            raise MotionInputError("camera capture factory must be callable")
        if not callable(frame_transform):
            raise MotionInputError("camera frame transform must be callable")
        _validate_nonnegative_integer(warmup_frames, "camera warmup frame count")
        if warmup_frames > MAX_CAMERA_PREVIEW_WARMUP_FRAMES:
            raise MotionInputError("camera warmup frame count exceeds the limit")
        _validate_nonnegative_integer(
            read_failure_limit,
            "camera read failure limit",
        )
        if read_failure_limit == 0:
            raise MotionInputError("camera read failure limit must be positive")
        if read_failure_limit > MAX_CAMERA_PREVIEW_READ_FAILURE_LIMIT:
            raise MotionInputError("camera read failure count exceeds the limit")
        if (
            isinstance(retry_seconds, bool)
            or not isinstance(retry_seconds, (int, float))
            or not np.isfinite(retry_seconds)
            or retry_seconds < 0
            or retry_seconds > MAX_CAMERA_PREVIEW_RETRY_SECONDS
        ):
            raise MotionInputError(
                "camera retry interval must be a finite number between zero "
                f"and {MAX_CAMERA_PREVIEW_RETRY_SECONDS} seconds"
            )
        _validate_nonnegative_integer(
            release_attempts,
            "camera release attempt count",
        )
        if release_attempts == 0:
            raise MotionInputError("camera release attempt count must be positive")
        if release_attempts > MAX_CAMERA_PREVIEW_RELEASE_ATTEMPTS:
            raise MotionInputError("camera release attempt count exceeds the limit")
        if not callable(thread_factory):
            raise MotionInputError("camera thread factory must be callable")

        self._capture_factory = capture_factory
        self._frame_transform = frame_transform
        self._warmup_frames = warmup_frames
        self._read_failure_limit = read_failure_limit
        self._retry_seconds = float(retry_seconds)
        self._release_attempts = release_attempts
        self._thread_factory = thread_factory
        self._lock = threading.Lock()
        self._state_changed = threading.Condition(self._lock)
        self._request_changed = threading.Event()
        self._worker_stopped = threading.Event()
        self._worker_stopped.set()
        self._events = deque(maxlen=MAX_CAMERA_PREVIEW_EVENTS)
        self._next_event_sequence = 0
        self._next_frame_sequence = 0
        self._next_request_id = 0
        self._desired = None
        self._active_request_id = None
        self._latest_preview = None
        self._latest_raw_frame = None
        self._worker = None
        self._closed = False
        self._fault_reason = None

    @property
    def active(self):
        with self._lock:
            return self._worker is not None

    @property
    def closed(self):
        with self._lock:
            return self._closed

    @property
    def fault_reason(self):
        with self._lock:
            return self._fault_reason

    @property
    def desired_request_id(self):
        with self._lock:
            return None if self._desired is None else self._desired.request_id

    @property
    def active_request_id(self):
        with self._lock:
            return self._active_request_id

    def request_start(self, camera_source, *, replace=True):
        camera_source = _validate_camera_source(camera_source)
        if not isinstance(replace, bool):
            raise MotionInputError("camera replacement option must be boolean")
        with self._lock:
            if self._closed:
                raise MotionInputError("camera preview worker is closed")
            if self._fault_reason is not None:
                raise MotionInputError(
                    "camera preview worker requires an application restart: "
                    + self._fault_reason
                )
            if not replace and self._worker is not None:
                raise MotionInputError("camera capture worker is busy")
            self._next_request_id += 1
            request = _CameraPreviewRequest(
                request_id=self._next_request_id,
                camera_source=camera_source,
            )
            self._desired = request
            self._latest_preview = None
            self._latest_raw_frame = None
            self._retain_terminal_events_locked()
            self._append_event_locked("starting", request)
            self._request_changed.set()
            self._state_changed.notify_all()
            if self._worker is not None:
                return request.request_id

            self._worker_stopped.clear()
            try:
                worker = self._thread_factory(
                    target=self._run,
                    name="ar4-camera-preview",
                    daemon=True,
                )
            except Exception as exc:
                self._desired = None
                self._worker_stopped.set()
                detail = normalize_camera_exception_detail(
                    exc,
                    "camera thread creation failed: ",
                )
                self._append_event_locked("failed", request, detail)
                self._state_changed.notify_all()
                raise RuntimeError(detail) from exc
            if not isinstance(worker, threading.Thread):
                self._desired = None
                self._worker_stopped.set()
                detail = "camera thread factory returned an invalid worker"
                self._append_event_locked("failed", request, detail)
                self._state_changed.notify_all()
                raise MotionInputError(detail)
            self._worker = worker
            try:
                worker.start()
            except Exception as exc:
                self._worker = None
                self._desired = None
                self._worker_stopped.set()
                detail = normalize_camera_exception_detail(
                    exc,
                    "camera preview worker startup failed: ",
                )
                self._append_event_locked("failed", request, detail)
                self._state_changed.notify_all()
                raise RuntimeError(detail) from exc
            return request.request_id

    def request_stop(self, request_id=None):
        if request_id is not None:
            _validate_nonnegative_integer(request_id, "camera request id")
        with self._lock:
            request = self._desired
            if (
                request is None
                or (
                    request_id is not None
                    and request.request_id != request_id
                )
            ):
                return False
            self._desired = None
            self._latest_preview = None
            self._latest_raw_frame = None
            self._append_event_locked("stopping", request)
            if self._active_request_id != request.request_id:
                self._append_event_locked("stopped", request)
            self._request_changed.set()
            self._state_changed.notify_all()
            return True

    def _close_locked(self):
        self._closed = True
        request = self._desired
        self._desired = None
        self._latest_preview = None
        self._latest_raw_frame = None
        if request is not None:
            self._append_event_locked("stopping", request)
            if self._active_request_id != request.request_id:
                self._append_event_locked("stopped", request)
        self._request_changed.set()
        self._state_changed.notify_all()

    def _lifecycle_state_locked(self):
        return CameraPreviewLifecycleState(
            active=self._worker is not None,
            stopped=self._worker_stopped.is_set(),
            closed=self._closed,
            fault_reason=self._fault_reason,
        )

    def close_state(self):
        """Close acquisition and return one atomic lifecycle snapshot."""

        with self._lock:
            self._close_locked()
            return self._lifecycle_state_locked()

    def close(self):
        return self.close_state().clean

    def wait_stopped(self, timeout=None, cancellation_event=None):
        """Wait for worker retirement from a non-Tk lifecycle owner."""

        if timeout is not None:
            timeout = _validate_camera_wait_timeout(timeout)
        if cancellation_event is None:
            return self._worker_stopped.wait(timeout)
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if _camera_cancellation_requested(cancellation_event):
                return False
            if self._worker_stopped.is_set():
                return True
            wait_seconds = CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                wait_seconds = min(wait_seconds, remaining)
            self._worker_stopped.wait(wait_seconds)

    def wait_ready(self, request_id, timeout, cancellation_event=None):
        """Wait off Tk until a request owns a validated raw frame."""

        _validate_nonnegative_integer(request_id, "camera request id")
        timeout = _validate_camera_wait_timeout(timeout)
        deadline = time.monotonic() + timeout
        with self._state_changed:
            while True:
                if _camera_cancellation_requested(cancellation_event):
                    return False
                if (
                    self._latest_raw_frame is not None
                    and self._latest_raw_frame[0] == request_id
                ):
                    return True
                if self._fault_reason is not None:
                    raise MotionInputError(
                        "camera preview cleanup requires an application "
                        "restart: " + self._fault_reason
                    )
                desired_request_id = (
                    None
                    if self._desired is None
                    else self._desired.request_id
                )
                if (
                    desired_request_id != request_id
                    and self._active_request_id != request_id
                ):
                    raise MotionInputError(
                        "camera preview request ended before readiness"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                if cancellation_event is not None:
                    remaining = min(
                        remaining,
                        CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS,
                    )
                self._state_changed.wait(remaining)

    def wait_snapshot_or_stopped(
        self,
        request_id,
        timeout,
        cancellation_event=None,
    ):
        """Return an owned frame from startup or wait for stopped cleanup."""

        _validate_nonnegative_integer(request_id, "camera request id")
        timeout = _validate_camera_wait_timeout(timeout)
        deadline = time.monotonic() + timeout
        with self._state_changed:
            while True:
                if _camera_cancellation_requested(cancellation_event):
                    raise MotionInputError("camera capture was cancelled")
                if (
                    self._latest_raw_frame is not None
                    and self._latest_raw_frame[0] == request_id
                ):
                    return np.array(
                        self._latest_raw_frame[1],
                        dtype=np.uint8,
                        copy=True,
                        order="C",
                    )
                if self._fault_reason is not None:
                    raise MotionInputError(
                        "camera preview cleanup requires an application "
                        "restart: " + self._fault_reason
                    )
                desired_request_id = (
                    None
                    if self._desired is None
                    else self._desired.request_id
                )
                if (
                    desired_request_id != request_id
                    and self._active_request_id != request_id
                ):
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise MotionInputError(
                        "camera preview transition did not settle before timeout"
                    )
                if cancellation_event is not None:
                    remaining = min(
                        remaining,
                        CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS,
                    )
                self._state_changed.wait(remaining)

    def wait_request_stopped(
        self,
        request_id,
        timeout,
        cancellation_event=None,
        *,
        require_idle=False,
    ):
        """Wait off Tk for request cleanup without joining the worker."""

        _validate_nonnegative_integer(request_id, "camera request id")
        timeout = _validate_camera_wait_timeout(timeout)
        if not isinstance(require_idle, bool):
            raise MotionInputError("camera idle requirement must be boolean")
        deadline = time.monotonic() + timeout
        with self._state_changed:
            while True:
                if _camera_cancellation_requested(cancellation_event):
                    return False
                if self._fault_reason is not None:
                    raise MotionInputError(
                        "camera preview cleanup requires an application "
                        "restart: " + self._fault_reason
                    )
                desired_request_id = (
                    None
                    if self._desired is None
                    else self._desired.request_id
                )
                if (
                    require_idle
                    and desired_request_id is not None
                    and desired_request_id != request_id
                ):
                    raise MotionInputError(
                        "camera request ownership changed before idle"
                    )
                if (
                    desired_request_id != request_id
                    and self._active_request_id != request_id
                ):
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                if cancellation_event is not None:
                    remaining = min(
                        remaining,
                        CAMERA_PREVIEW_CANCELLATION_POLL_SECONDS,
                    )
                self._state_changed.wait(remaining)

    def capture_once(self, camera_source, timeout, cancellation_event=None):
        """Acquire one validated frame through the owned worker lifecycle."""

        timeout = _validate_camera_wait_timeout(timeout)
        if _camera_cancellation_requested(cancellation_event):
            raise MotionInputError("camera capture was cancelled")
        request_id = self.request_start(camera_source, replace=False)
        try:
            ready = self.wait_ready(
                request_id,
                timeout,
                cancellation_event,
            )
            if not isinstance(ready, bool):
                raise RuntimeError(
                    "camera worker returned an invalid readiness result"
                )
            if not ready:
                if _camera_cancellation_requested(cancellation_event):
                    raise MotionInputError("camera capture was cancelled")
                raise MotionInputError(
                    "camera capture did not become ready before timeout"
                )
            frame = self.snapshot_raw_frame(request_id)
            if frame is None:
                raise MotionInputError(
                    "camera capture lost request ownership before snapshot"
                )
            return frame
        finally:
            cleanup_error = None
            try:
                self.request_stop(request_id)
                settled = self.wait_request_stopped(request_id, timeout)
                if not isinstance(settled, bool):
                    raise RuntimeError(
                        "camera worker returned an invalid cleanup result"
                    )
                if not settled:
                    cleanup_error = "camera capture cleanup timed out"
            except Exception as exc:
                cleanup_error = normalize_camera_exception_detail(
                    exc,
                    "camera capture cleanup failed: ",
                )
            if cleanup_error is not None:
                self.close()
                raise MotionInputError(cleanup_error)

    def drain_events(self):
        with self._lock:
            events = tuple(self._events)
            self._events.clear()
            return events

    def take_latest_frame(self, request_id):
        _validate_nonnegative_integer(request_id, "camera request id")
        with self._lock:
            frame = self._latest_preview
            if frame is None or frame.request_id != request_id:
                return None
            self._latest_preview = None
            return frame

    def snapshot_raw_frame(self, request_id):
        _validate_nonnegative_integer(request_id, "camera request id")
        with self._lock:
            if (
                self._latest_raw_frame is None
                or self._latest_raw_frame[0] != request_id
            ):
                return None
            frame = self._latest_raw_frame[1]
        return np.array(frame, dtype=np.uint8, copy=True, order="C")

    def _retain_terminal_events_locked(self):
        retained = tuple(
            event
            for event in self._events
            if event.kind in ("failed", "stopped")
        )[-MAX_RETAINED_CAMERA_TERMINAL_EVENTS:]
        self._events.clear()
        self._events.extend(retained)

    def _append_event_locked(self, kind, request, detail=None):
        event = CameraPreviewEvent(
            sequence=self._next_event_sequence,
            kind=kind,
            request_id=request.request_id,
            detail=detail,
        )
        self._next_event_sequence += 1
        self._events.append(event)
        return event

    def _request_is_current(self, request):
        with self._lock:
            return self._desired == request and not self._closed

    def _wait_for_change(self, request):
        self._request_changed.clear()
        if not self._request_is_current(request):
            return False
        return self._request_changed.wait(self._retry_seconds)

    @staticmethod
    def _validated_capture(capture):
        if not callable(getattr(capture, "read", None)):
            raise MotionInputError("camera capture has no read operation")
        if not callable(getattr(capture, "release", None)):
            raise MotionInputError("camera capture has no release operation")
        opened = getattr(capture, "isOpened", None)
        if callable(opened):
            open_state = opened()
            if not isinstance(open_state, (bool, np.bool_)):
                raise MotionInputError("camera open state is invalid")
            if not bool(open_state):
                raise MotionInputError("camera device did not open")
        return capture

    @staticmethod
    def _read_frame(capture):
        result = capture.read()
        if not isinstance(result, (tuple, list)) or len(result) != 2:
            raise MotionInputError("camera read result is invalid")
        succeeded, frame = result
        if not isinstance(succeeded, (bool, np.bool_)):
            raise MotionInputError("camera read status is invalid")
        if not bool(succeeded):
            if frame is not None:
                raise MotionInputError(
                    "failed camera read returned unexpected frame data"
                )
            return None
        return copy_camera_capture_frame(frame)

    def _release_capture(self, capture):
        last_error = None
        for attempt in range(self._release_attempts):
            try:
                capture.release()
                opened = getattr(capture, "isOpened", None)
                if callable(opened):
                    open_state = opened()
                    if not isinstance(open_state, (bool, np.bool_)):
                        raise MotionInputError("camera open state is invalid")
                    if bool(open_state):
                        raise MotionInputError("camera device remained open")
                return None
            except Exception as exc:
                last_error = exc
                if attempt + 1 < self._release_attempts:
                    time.sleep(self._retry_seconds)
        return normalize_camera_exception_detail(
            last_error,
            "camera release failed: ",
        )

    def _publish_frame(self, request, raw_frame):
        raw_frame.setflags(write=False)
        preview = self._frame_transform(raw_frame)
        preview = np.array(
            _validated_camera_frame(preview, "camera preview frame"),
            dtype=np.uint8,
            copy=True,
            order="C",
        )
        preview.setflags(write=False)
        with self._lock:
            if self._desired != request or self._closed:
                return False
            frame = CameraPreviewFrame(
                sequence=self._next_frame_sequence,
                request_id=request.request_id,
                image=preview,
            )
            self._next_frame_sequence += 1
            first_frame = self._latest_raw_frame is None
            self._latest_raw_frame = (request.request_id, raw_frame)
            self._latest_preview = frame
            if first_frame:
                self._append_event_locked("started", request)
            self._state_changed.notify_all()
            return True

    def _publish_request_failure(self, request, error):
        detail = normalize_camera_exception_detail(
            error,
            "camera preview failed: ",
        )
        with self._lock:
            if self._desired == request:
                self._desired = None
                self._latest_preview = None
                self._latest_raw_frame = None
            self._append_event_locked("failed", request, detail)
            self._request_changed.set()
            self._state_changed.notify_all()
        return detail

    def _run_session(self, request):
        capture = None
        try:
            capture = self._capture_factory(request.camera_source)
            self._validated_capture(capture)
            warmup_remaining = self._warmup_frames
            consecutive_failures = 0
            while self._request_is_current(request):
                frame = self._read_frame(capture)
                if frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= self._read_failure_limit:
                        raise MotionInputError(
                            "camera exceeded the consecutive read-failure limit"
                        )
                    self._wait_for_change(request)
                    continue
                consecutive_failures = 0
                if warmup_remaining:
                    warmup_remaining -= 1
                    continue
                self._publish_frame(request, frame)
        except Exception as exc:
            self._publish_request_failure(request, exc)
        finally:
            release_error = None
            if capture is not None:
                release_error = self._release_capture(capture)
            with self._lock:
                if release_error is not None:
                    affected_request = self._desired or request
                    self._fault_reason = release_error
                    self._desired = None
                    self._latest_preview = None
                    self._latest_raw_frame = None
                    self._append_event_locked(
                        "failed",
                        affected_request,
                        release_error,
                    )
                self._append_event_locked("stopped", request)
                if self._active_request_id == request.request_id:
                    self._active_request_id = None
                self._request_changed.set()
                self._state_changed.notify_all()

    def _run(self):
        current_thread = threading.current_thread()
        try:
            while True:
                with self._lock:
                    request = self._desired
                    if request is None or self._closed:
                        if self._worker is current_thread:
                            self._worker = None
                            self._worker_stopped.set()
                            self._active_request_id = None
                        self._request_changed.set()
                        self._state_changed.notify_all()
                        break
                    self._active_request_id = request.request_id
                    self._state_changed.notify_all()
                self._run_session(request)
        except BaseException as exc:
            detail = normalize_camera_exception_detail(
                exc,
                "camera preview worker terminated: ",
            )
            with self._lock:
                if self._worker is current_thread:
                    request = self._desired
                    self._desired = None
                    self._active_request_id = None
                    self._latest_preview = None
                    self._latest_raw_frame = None
                    self._fault_reason = detail
                    if request is not None:
                        self._append_event_locked("failed", request, detail)
                    self._state_changed.notify_all()
        finally:
            with self._lock:
                if self._worker is current_thread:
                    self._worker = None
                    self._worker_stopped.set()
                    self._active_request_id = None
                self._request_changed.set()
                self._state_changed.notify_all()


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
