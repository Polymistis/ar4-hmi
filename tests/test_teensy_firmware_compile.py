import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import unittest

if __package__:
    from .bounded_temp import BoundedTemporaryDirectory
else:
    from bounded_temp import BoundedTemporaryDirectory

PROJECT_ROOT = Path(__file__).resolve().parents[1]
TEENSY_SKETCH = (
    PROJECT_ROOT
    / "ArduinoSketches"
    / "AR4_teensy41_sketch_v6.7.1"
)
PROCESS_TREE_SETTLEMENT_TIMEOUT_SECONDS = 10


def _reported_library_paths(output, library_name, version):
    if not isinstance(output, str):
        raise TypeError("compiler output must be text")
    if not library_name or not version:
        raise ValueError("library name and version must be non-empty")

    verbose_prefix = (
        f"Using library {library_name} at version {version} in folder:"
    )
    table_pattern = re.compile(
        rf"^{re.escape(library_name)}\s+"
        rf"{re.escape(version)}\s+(.+?)\s*$"
    )
    reported_paths = []
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if line.startswith(verbose_prefix):
            raw_path = line[len(verbose_prefix):].strip()
        else:
            match = table_pattern.fullmatch(line)
            if match is None:
                continue
            raw_path = match.group(1).strip()
        if not raw_path:
            raise AssertionError(
                f"{library_name} compiler report omitted the library path"
            )
        reported_paths.append(Path(raw_path).resolve(strict=True))
    return tuple(dict.fromkeys(reported_paths))


class _WindowsProcessJob:
    def __init__(self):
        if os.name != "nt":
            raise OSError("Windows process jobs are unavailable")
        import ctypes
        from ctypes import wintypes

        class IoCounters(ctypes.Structure):
            _fields_ = [
                ("read_operation_count", ctypes.c_ulonglong),
                ("write_operation_count", ctypes.c_ulonglong),
                ("other_operation_count", ctypes.c_ulonglong),
                ("read_transfer_count", ctypes.c_ulonglong),
                ("write_transfer_count", ctypes.c_ulonglong),
                ("other_transfer_count", ctypes.c_ulonglong),
            ]

        class BasicLimitInformation(ctypes.Structure):
            _fields_ = [
                ("per_process_user_time_limit", ctypes.c_longlong),
                ("per_job_user_time_limit", ctypes.c_longlong),
                ("limit_flags", wintypes.DWORD),
                ("minimum_working_set_size", ctypes.c_size_t),
                ("maximum_working_set_size", ctypes.c_size_t),
                ("active_process_limit", wintypes.DWORD),
                ("affinity", ctypes.c_size_t),
                ("priority_class", wintypes.DWORD),
                ("scheduling_class", wintypes.DWORD),
            ]

        class ExtendedLimitInformation(ctypes.Structure):
            _fields_ = [
                ("basic_limit_information", BasicLimitInformation),
                ("io_info", IoCounters),
                ("process_memory_limit", ctypes.c_size_t),
                ("job_memory_limit", ctypes.c_size_t),
                ("peak_process_memory_used", ctypes.c_size_t),
                ("peak_job_memory_used", ctypes.c_size_t),
            ]

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CreateJobObjectW.argtypes = [
            ctypes.c_void_p,
            wintypes.LPCWSTR,
        ]
        kernel32.CreateJobObjectW.restype = wintypes.HANDLE
        kernel32.SetInformationJobObject.argtypes = [
            wintypes.HANDLE,
            ctypes.c_int,
            ctypes.c_void_p,
            wintypes.DWORD,
        ]
        kernel32.SetInformationJobObject.restype = wintypes.BOOL
        kernel32.AssignProcessToJobObject.argtypes = [
            wintypes.HANDLE,
            wintypes.HANDLE,
        ]
        kernel32.AssignProcessToJobObject.restype = wintypes.BOOL
        kernel32.TerminateJobObject.argtypes = [
            wintypes.HANDLE,
            wintypes.UINT,
        ]
        kernel32.TerminateJobObject.restype = wintypes.BOOL
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel32.CloseHandle.restype = wintypes.BOOL

        handle = kernel32.CreateJobObjectW(None, None)
        if not handle:
            raise ctypes.WinError(ctypes.get_last_error())
        limits = ExtendedLimitInformation()
        limits.basic_limit_information.limit_flags = 0x00002000
        if not kernel32.SetInformationJobObject(
            handle,
            9,
            ctypes.byref(limits),
            ctypes.sizeof(limits),
        ):
            error = ctypes.WinError(ctypes.get_last_error())
            kernel32.CloseHandle(handle)
            raise error
        self._ctypes = ctypes
        self._wintypes = wintypes
        self._kernel32 = kernel32
        self._handle = handle

    def assign(self, process):
        if self._handle is None:
            raise RuntimeError("Windows process job is closed")
        process_handle = getattr(process, "_handle", None)
        if process_handle is None:
            raise RuntimeError("Popen process handle is unavailable")
        if not self._kernel32.AssignProcessToJobObject(
            self._handle,
            self._wintypes.HANDLE(process_handle),
        ):
            raise self._ctypes.WinError(self._ctypes.get_last_error())
        return True

    def terminate(self):
        if self._handle is None:
            return False
        if not self._kernel32.TerminateJobObject(self._handle, 1):
            raise self._ctypes.WinError(self._ctypes.get_last_error())
        return True

    def close(self):
        if self._handle is None:
            return False
        handle = self._handle
        self._handle = None
        if not self._kernel32.CloseHandle(handle):
            raise self._ctypes.WinError(self._ctypes.get_last_error())
        return True


def _terminate_process_tree(process, windows_job=None):
    if not isinstance(process, subprocess.Popen):
        raise TypeError("process-tree termination requires a Popen instance")
    if os.name == "nt":
        if not isinstance(windows_job, _WindowsProcessJob):
            raise RuntimeError(
                "Windows process-tree termination requires a job owner"
            )
        windows_job.terminate()
    elif os.name == "posix":
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    else:
        raise OSError(
            f"process-tree termination is unsupported on {os.name!r}"
        )
    try:
        return process.communicate(
            timeout=PROCESS_TREE_SETTLEMENT_TIMEOUT_SECONDS
        )
    except subprocess.TimeoutExpired as exc:
        process.kill()
        for stream in (process.stdout, process.stderr):
            if stream is not None:
                stream.close()
        try:
            process.wait(timeout=PROCESS_TREE_SETTLEMENT_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired as settlement_exc:
            raise RuntimeError(
                "process root did not exit after forced termination"
            ) from settlement_exc
        raise RuntimeError(
            "process tree did not settle after forced termination"
        ) from exc


def _run_bounded_process_tree(command, *, cwd, timeout):
    if (
        not isinstance(command, tuple)
        or not command
        or any(not isinstance(argument, str) for argument in command)
    ):
        raise TypeError("bounded process command must be a nonempty tuple")
    if (
        isinstance(timeout, bool)
        or not isinstance(timeout, (int, float))
        or timeout <= 0
    ):
        raise ValueError("bounded process timeout must be positive")
    process_options = {}
    windows_job = None
    launch_command = command
    if os.name == "nt":
        windows_job = _WindowsProcessJob()
        process_options["creationflags"] = (
            subprocess.CREATE_NEW_PROCESS_GROUP
            | subprocess.CREATE_NO_WINDOW
        )
        launcher_source = (
            "import subprocess,sys;"
            "sys.stdin.read(1);"
            "raise SystemExit(subprocess.call(sys.argv[1:]))"
        )
        launch_command = (
            sys.executable,
            "-c",
            launcher_source,
            *command,
        )
        process_options["stdin"] = subprocess.PIPE
    elif os.name == "posix":
        process_options["start_new_session"] = True
    else:
        raise OSError(
            f"bounded process execution is unsupported on {os.name!r}"
        )
    try:
        try:
            process = subprocess.Popen(
                launch_command,
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                **process_options,
            )
            if windows_job is not None:
                windows_job.assign(process)
                if process.stdin.write("1") != 1:
                    raise OSError(
                        "Windows process launcher admission was incomplete"
                    )
                process.stdin.close()
                process.stdin = None
        except Exception:
            if "process" in locals() and process.poll() is None:
                process.kill()
                try:
                    process.communicate(
                        timeout=PROCESS_TREE_SETTLEMENT_TIMEOUT_SECONDS
                    )
                except subprocess.TimeoutExpired as settlement_exc:
                    for stream in (process.stdout, process.stderr):
                        if stream is not None:
                            stream.close()
                    raise RuntimeError(
                        "failed process launch did not settle"
                    ) from settlement_exc
            raise
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            try:
                stdout, stderr = _terminate_process_tree(
                    process,
                    windows_job,
                )
            except Exception as termination_exc:
                raise RuntimeError(
                    "bounded process timed out and tree cleanup failed"
                ) from termination_exc
            raise subprocess.TimeoutExpired(
                command,
                timeout,
                output=stdout,
                stderr=stderr,
            ) from exc
        return subprocess.CompletedProcess(
            command,
            process.returncode,
            stdout,
            stderr,
        )
    finally:
        if windows_job is not None:
            windows_job.close()


class TeensyFirmwareCompileTests(unittest.TestCase):
    def test_bounded_runner_terminates_descendant_processes(self):
        with BoundedTemporaryDirectory(
            prefix="ar4hmi-process-tree-",
        ) as directory:
            child_pid_path = Path(directory) / "child.pid"
            child_source = (
                "import time\n"
                "print('ready',flush=True)\n"
                "time.sleep(60)\n"
            )
            parent_source = (
                "from pathlib import Path\n"
                "import subprocess,sys,time\n"
                "child=subprocess.Popen([sys.executable,'-c',sys.argv[2]],"
                "stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)\n"
                "ready=child.stdout.readline()\n"
                "if ready != 'ready\\n':\n"
                "    print(f'child readiness failed: {ready!r}',"
                "file=sys.stderr,flush=True)\n"
                "elif child.poll() is not None:\n"
                "    print('child exited after readiness',file=sys.stderr,flush=True)\n"
                "else:\n"
                "    Path(sys.argv[1]).write_text(str(child.pid),encoding='ascii')\n"
                "time.sleep(60)\n"
            )
            command = (
                sys.executable,
                "-c",
                parent_source,
                str(child_pid_path),
                child_source,
            )
            with self.assertRaises(subprocess.TimeoutExpired) as caught:
                _run_bounded_process_tree(
                    command,
                    cwd=PROJECT_ROOT,
                    timeout=5,
                )
            diagnostic = caught.exception.stderr or caught.exception.output or ""
            diagnostic = diagnostic.strip()
            self.assertTrue(
                child_pid_path.is_file(),
                "parent process did not record a ready child identity"
                + (f": {diagnostic}" if diagnostic else ""),
            )
            child_pid = int(child_pid_path.read_text(encoding="ascii"))

            deadline = time.monotonic() + 2
            while self._process_is_active(child_pid):
                if time.monotonic() >= deadline:
                    self.fail(
                        "descendant process survived bounded runner timeout"
                    )
                time.sleep(0.02)

    @staticmethod
    def _process_is_active(process_id):
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes

            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            kernel32.OpenProcess.argtypes = [
                wintypes.DWORD,
                wintypes.BOOL,
                wintypes.DWORD,
            ]
            kernel32.OpenProcess.restype = wintypes.HANDLE
            kernel32.WaitForSingleObject.argtypes = [
                wintypes.HANDLE,
                wintypes.DWORD,
            ]
            kernel32.WaitForSingleObject.restype = wintypes.DWORD
            kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
            kernel32.CloseHandle.restype = wintypes.BOOL
            handle = kernel32.OpenProcess(0x00100000, False, process_id)
            if not handle:
                error_code = ctypes.get_last_error()
                if error_code == 87:
                    return False
                raise ctypes.WinError(error_code)
            try:
                wait_result = kernel32.WaitForSingleObject(handle, 0)
                if wait_result == 0xFFFFFFFF:
                    raise ctypes.WinError(ctypes.get_last_error())
                if wait_result not in (0, 258):
                    raise OSError(
                        f"unexpected process wait result {wait_result}"
                    )
                return wait_result == 258
            finally:
                kernel32.CloseHandle(handle)

        if os.name == "posix":
            try:
                os.kill(process_id, 0)
            except ProcessLookupError:
                return False
            process_status = Path(f"/proc/{process_id}/stat")
            if process_status.is_file():
                fields = process_status.read_text(
                    encoding="ascii"
                ).split()
                if len(fields) >= 3 and fields[2] == "Z":
                    return False
            return True
        raise OSError(
            f"process inspection is unsupported on {os.name!r}"
        )

    def test_tracked_teensy_source_compiles_without_upload(self):
        variable_names = (
            "AR4_ARDUINO_CLI",
            "AR4_ARDUINOJSON_LIBRARY",
            "AR4_TEENSY_BUILD_DIRECTORY",
            "AR4_TEENSY_SPI_LIBRARY",
        )
        values = {
            name: os.environ.get(name)
            for name in variable_names
        }
        missing = tuple(name for name, value in values.items() if not value)
        if missing:
            self.skipTest(
                "firmware compile environment is unavailable: "
                + ", ".join(missing)
            )

        cli = Path(values["AR4_ARDUINO_CLI"]).resolve(strict=True)
        build_parent = Path(
            values["AR4_TEENSY_BUILD_DIRECTORY"]
        ).resolve(strict=True)
        spi_library = Path(
            values["AR4_TEENSY_SPI_LIBRARY"]
        ).resolve(strict=True)
        arduinojson_library = Path(
            values["AR4_ARDUINOJSON_LIBRARY"]
        ).resolve(strict=True)
        self.assertTrue(cli.is_file(), "Arduino CLI path is not a file")
        self.assertTrue(
            build_parent.is_dir(),
            "firmware build parent is not a directory",
        )
        self.assertTrue(
            spi_library.is_dir(),
            "Teensy SPI library path is not a directory",
        )
        self.assertTrue(
            arduinojson_library.is_dir(),
            "ArduinoJson library path is not a directory",
        )
        arduinojson_properties = (
            arduinojson_library / "library.properties"
        )
        self.assertTrue(
            arduinojson_properties.is_file(),
            "ArduinoJson library properties are absent",
        )
        self.assertRegex(
            arduinojson_properties.read_text(encoding="utf-8"),
            r"(?m)^version=7\.4\.3$",
            "ArduinoJson dependency is not pinned to 7.4.3",
        )
        self.assertEqual(
            tuple(part.casefold() for part in spi_library.parts[-6:]),
            (
                "teensy",
                "hardware",
                "avr",
                "1.62.0",
                "libraries",
                "spi",
            ),
            "Teensy SPI path does not select PJRC core 1.62.0",
        )
        self.assertNotEqual(build_parent, PROJECT_ROOT)
        self.assertNotIn(PROJECT_ROOT, build_parent.parents)
        platform_directory = spi_library.parents[1]
        sd_fat_library = (
            platform_directory / "libraries" / "SdFat"
        ).resolve(strict=True)

        with BoundedTemporaryDirectory(
            prefix="ar4hmi-teensy-compile-",
            dir=build_parent,
        ) as build_directory:
            command = (
                str(cli),
                "compile",
                "--no-color",
                "--verbose",
                "--fqbn",
                "teensy:avr:teensy41",
                "--clean",
                "--build-path",
                build_directory,
                "--library",
                str(spi_library),
                "--library",
                str(arduinojson_library),
                str(TEENSY_SKETCH),
            )
            self.assertNotIn("--upload", command)
            result = _run_bounded_process_tree(
                command,
                cwd=PROJECT_ROOT,
                timeout=180,
            )

        self.assertEqual(
            result.returncode,
            0,
            result.stdout + result.stderr,
        )
        output = result.stdout + result.stderr
        self.assertRegex(
            output,
            (
                r"(?m)^(?:"
                r"ModbusMaster\s+2\.0\.1(?:\s|$)"
                r"|Using library ModbusMaster at version 2\.0\.1(?:\s|$)"
                r")"
            ),
            "compile did not select ModbusMaster 2.0.1",
        )
        self.assertEqual(
            set(_reported_library_paths(output, "SPI", "1.0")),
            {spi_library},
            "compile did not select the validated platform SPI library",
        )
        self.assertEqual(
            set(_reported_library_paths(output, "SdFat", "2.1.2")),
            {sd_fat_library},
            "compile did not select the bundled platform SdFat library",
        )
        self.assertEqual(
            set(_reported_library_paths(output, "ArduinoJson", "7.4.3")),
            {arduinojson_library},
            "compile did not select the pinned ArduinoJson library",
        )
        self.assertRegex(
            output,
            (
                r"(?m)^(?:"
                r"teensy:avr\s+1\.62\.0(?:\s|$)"
                r"|Using (?:board|core) .+"
                r"[\\/]teensy[\\/]hardware[\\/]avr[\\/]1\.62\.0\s*$"
                r")"
            ),
            "compile did not select PJRC Teensy core 1.62.0",
        )
