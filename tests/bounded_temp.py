import os
from pathlib import Path
import secrets
import shutil
import tempfile


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_TEMPORARY_PARENT_ENVIRONMENT_VARIABLE = "AR4_TEST_TEMP_DIRECTORY"


def _external_temporary_parent(directory):
    if directory is None:
        directory = os.environ.get(
            _TEMPORARY_PARENT_ENVIRONMENT_VARIABLE
        ) or tempfile.gettempdir()
    raw_directory = os.fspath(directory)
    if not isinstance(raw_directory, str) or "\x00" in raw_directory:
        raise ValueError("temporary-directory parent must be a text path")
    base_directory = os.path.realpath(os.path.abspath(raw_directory))
    if not os.path.isdir(base_directory):
        raise FileNotFoundError(
            "temporary-directory parent does not exist"
        )

    project_root = os.path.realpath(str(_PROJECT_ROOT))
    try:
        inside_project = os.path.normcase(
            os.path.commonpath((base_directory, project_root))
        ) == os.path.normcase(project_root)
    except ValueError:
        inside_project = False
    if inside_project:
        raise ValueError(
            "temporary-directory parent must remain outside the source tree"
        )
    return base_directory


class BoundedTemporaryDirectory:
    _MAXIMUM_ATTEMPTS = 16

    def __init__(self, suffix=None, prefix=None, dir=None):
        directory_mode = 0o700 if os.name == "posix" else 0o777
        suffix = "" if suffix is None else suffix
        prefix = "tmp" if prefix is None else prefix
        for name, value in (("suffix", suffix), ("prefix", prefix)):
            if (
                not isinstance(value, str)
                or "\x00" in value
                or os.path.basename(value) != value
            ):
                raise ValueError(
                    f"temporary-directory {name} must be a path-free string"
                )

        base_directory = _external_temporary_parent(dir)

        self.name = None
        self._active = False
        for _ in range(self._MAXIMUM_ATTEMPTS):
            candidate = os.path.join(
                base_directory,
                f"{prefix}{secrets.token_hex(16)}{suffix}",
            )
            try:
                os.mkdir(candidate, directory_mode)
            except FileExistsError:
                continue
            self.name = candidate
            self._active = True
            break
        if not self._active:
            raise FileExistsError(
                "unable to allocate a unique temporary directory"
            )

    def cleanup(self):
        if not self._active:
            return
        shutil.rmtree(self.name)
        self._active = False

    def __enter__(self):
        if not self._active:
            raise RuntimeError("temporary directory is no longer active")
        return self.name

    def __exit__(self, exc_type, exc_value, traceback):
        self.cleanup()
        return False
