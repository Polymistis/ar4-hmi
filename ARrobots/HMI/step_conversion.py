"""Import-safe source and frozen-Windows STEP-to-STL host boundary."""

from dataclasses import dataclass
from operator import attrgetter
import os
import stat
import subprocess
import sys
import tempfile
from threading import Event, Lock
import time

MAX_FILE_BYTES = 64 * 1024 * 1024
CONVERSION_TIMEOUT_SECONDS = 120.0
_fingerprint = attrgetter("st_dev", "st_ino", "st_size", "st_mtime_ns")
_WORKER_MESSAGES = {
    2: "CadQuery could not be loaded by the STEP worker",
    3: "STEP geometry could not be imported",
    4: "STEP geometry could not be converted to STL",
}


class StepConversionError(ValueError):
    """STEP input, dependency, worker, or output is invalid."""


@dataclass(frozen=True)
class StepConversionResult:
    label: str
    stl_payload: bytes


class StepConversionControl:
    """Cancellation and launch arbitration for one conversion."""

    def __init__(self):
        self._cancelled = Event()
        self._launch_lock = Lock()
        self._child_active = Event()

    def cancel(self):
        self._cancelled.set()

    def is_cancelled(self):
        return self._cancelled.is_set()

    def cancel_for_shutdown(self):
        with self._launch_lock:
            self._cancelled.set()
            return self._child_active.is_set()


def _state(path, description):
    try:
        value = os.stat(path, follow_symlinks=False)
    except (OSError, ValueError) as exc:
        raise StepConversionError(f"{description} could not be inspected") from exc
    reparse = getattr(value, "st_file_attributes", 0) & 0x400
    if not stat.S_ISREG(value.st_mode) or reparse:
        raise StepConversionError(f"{description} must be a nonsymlink regular file")
    if value.st_size <= 0 or value.st_size > MAX_FILE_BYTES:
        raise StepConversionError(f"{description} is empty or exceeds the byte limit")
    return value


def _read_regular(path, description):
    expected = _state(path, description)
    try:
        with open(path, "rb") as source:
            opened = os.fstat(source.fileno())
            if _fingerprint(opened) != _fingerprint(expected):
                raise StepConversionError(f"{description} changed before being read")
            payload = source.read(MAX_FILE_BYTES + 1)
            final = os.fstat(source.fileno())
    except OSError as exc:
        raise StepConversionError(f"{description} could not be read") from exc
    if (len(payload) != opened.st_size or _fingerprint(final) != _fingerprint(opened)
            or _fingerprint(_state(path, description)) != _fingerprint(opened)):
        raise StepConversionError(f"{description} changed while being read")
    return payload


def _stop_worker(process):
    try:
        process.terminate()
    except OSError:
        pass
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        try:
            process.kill()
        except OSError:
            pass
        process.wait()


def _output_size(path):
    try:
        value = os.stat(path, follow_symlinks=False)
    except FileNotFoundError:
        return 0
    except (OSError, ValueError) as exc:
        raise StepConversionError("STEP worker output could not be inspected") from exc
    if not stat.S_ISREG(value.st_mode) or getattr(value, "st_file_attributes", 0) & 0x400:
        raise StepConversionError("STEP worker output is not a regular file")
    return value.st_size


def _wait_for_worker(process, output_path, control, deadline_seconds):
    expires = time.monotonic() + deadline_seconds
    while True:
        try:
            cancelled, size = control._cancelled.is_set(), _output_size(output_path)
        except StepConversionError:
            _stop_worker(process)
            raise
        if cancelled or size > MAX_FILE_BYTES:
            _stop_worker(process)
            message = "STEP conversion was cancelled" if cancelled else "STEP worker output exceeds the byte limit"
            raise StepConversionError(message)
        status = process.poll()
        if status is not None:
            return status
        if time.monotonic() >= expires:
            _stop_worker(process)
            raise StepConversionError("STEP conversion timed out")
        time.sleep(0.1)


def convert_step(source_path, *, control=None):
    """Convert one local STEP source to bounded binary STL bytes."""
    frozen = bool(getattr(sys, "frozen", False))
    if frozen and os.name != "nt":
        raise StepConversionError("STEP import is unavailable in frozen non-Windows builds")
    control = StepConversionControl() if control is None else control
    if not isinstance(control, StepConversionControl):
        raise StepConversionError("STEP conversion control is invalid")
    try:
        source_path = os.fspath(source_path)
    except TypeError as exc:
        raise StepConversionError("STEP source path is invalid") from exc
    if not isinstance(source_path, str) or not source_path or "\x00" in source_path:
        raise StepConversionError("STEP source path must be nonempty text")
    source_path = os.path.abspath(os.path.expanduser(source_path))
    if os.path.splitext(source_path)[1].lower() not in (".step", ".stp"):
        raise StepConversionError("STEP source must use a .step or .stp extension")
    if control._cancelled.is_set():
        raise StepConversionError("STEP conversion was cancelled")
    try:
        workspace = tempfile.TemporaryDirectory(prefix="ar4-step-")
        with workspace as directory:
            input_path = os.path.join(directory, "input.step")
            output_path = os.path.join(directory, "output.stl")
            payload = _read_regular(source_path, "STEP source")
            with open(input_path, "xb") as destination:
                if destination.write(payload) != len(payload):
                    raise OSError("incomplete STEP source copy")
            expires = time.monotonic() + CONVERSION_TIMEOUT_SECONDS
            if frozen:
                creationflags = subprocess.CREATE_NO_WINDOW
                working_directory = os.path.dirname(os.path.abspath(sys.executable))
                executable = os.path.join(working_directory, "AR4StepWorker.exe")
                environment = os.environ.copy()
                environment["PYINSTALLER_RESET_ENVIRONMENT"] = "1"
                command = [executable, input_path, output_path]
            else:
                executable = sys.executable
                environment = None
                creationflags = 0
                working_directory = os.path.dirname(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                if os.name == "nt":
                    creationflags = subprocess.CREATE_NO_WINDOW
                    base_executable = getattr(sys, "_base_executable", None) or sys.executable
                    if os.path.normcase(base_executable) != os.path.normcase(sys.executable):
                        executable = base_executable
                        environment = os.environ.copy()
                        environment["__PYVENV_LAUNCHER__"] = sys.executable
                command = [executable, "-m", "ARrobots.HMI.step_worker",
                           input_path, output_path]
            with control._launch_lock:
                if control._cancelled.is_set():
                    raise StepConversionError("STEP conversion was cancelled")
                try:
                    process = subprocess.Popen(
                        command, cwd=working_directory, shell=False, stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        env=environment, creationflags=creationflags)
                except OSError as exc:
                    raise StepConversionError("STEP conversion worker could not be started") from exc
                control._child_active.set()
            try:
                remaining = expires - time.monotonic()
                if remaining <= 0.0:
                    _stop_worker(process)
                    raise StepConversionError("STEP conversion timed out")
                status = _wait_for_worker(process, output_path, control, remaining)
                if status != 0:
                    message = _WORKER_MESSAGES.get(status, "STEP conversion worker crashed")
                    raise StepConversionError(message)
                payload = _read_regular(output_path, "STEP worker output")
                if control._cancelled.is_set():
                    raise StepConversionError("STEP conversion was cancelled")
            finally:
                try:
                    workspace.cleanup()
                finally:
                    control._child_active.clear()
    except OSError as exc:
        raise StepConversionError("STEP temporary workspace could not be managed") from exc
    return StepConversionResult(os.path.splitext(os.path.basename(source_path))[0], payload)
