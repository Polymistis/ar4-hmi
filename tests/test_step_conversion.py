import importlib.util
import os
from pathlib import Path
import subprocess
import sys
from threading import Timer
import unittest
from unittest import mock

try:
    from .bounded_temp import BoundedTemporaryDirectory
except ImportError:
    from bounded_temp import BoundedTemporaryDirectory
from ARrobots.HMI.step_conversion import (
    MAX_FILE_BYTES,
    StepConversionControl,
    StepConversionError,
    _wait_for_worker,
    convert_step,
)


_CADQUERY_AVAILABLE = importlib.util.find_spec("cadquery") is not None
_CHILD_EXECUTABLE = getattr(sys, "_base_executable", sys.executable) if os.name == "nt" else sys.executable
_CHILD_ENVIRONMENT = os.environ.copy()
if _CHILD_EXECUTABLE != sys.executable:
    _CHILD_ENVIRONMENT["__PYVENV_LAUNCHER__"] = sys.executable


class StepConversionTests(unittest.TestCase):
    def test_source_dependency_and_output_boundaries(self):
        with BoundedTemporaryDirectory(prefix="ar4-step-boundary-") as directory:
            root = Path(directory)
            wrong_extension = root / "fixture.stl"
            wrong_extension.write_bytes(b"mesh")
            directory_source = root / "directory.step"
            directory_source.mkdir()
            empty = root / "empty.step"
            empty.touch()
            oversized = root / "oversized.stp"
            with oversized.open("wb") as stream:
                stream.truncate(MAX_FILE_BYTES + 1)
            invalid = root / "invalid.STEP"
            invalid.write_bytes(b"not STEP geometry")
            cancelled = StepConversionControl()
            cancelled.cancel()
            worker_message = (
                "STEP geometry could not be imported" if _CADQUERY_AVAILABLE
                else "CadQuery could not be loaded by the STEP worker"
            )
            cases = (
                ("path-type", lambda: convert_step(None), "STEP source path is invalid"),
                ("empty-path", lambda: convert_step(""), "STEP source path must be nonempty text"),
                ("extension", lambda: convert_step(wrong_extension), "STEP source must use a .step or .stp extension"),
                ("directory", lambda: convert_step(directory_source), "STEP source must be a nonsymlink regular file"),
                ("empty-source", lambda: convert_step(empty), "STEP source is empty or exceeds the byte limit"),
                ("oversized-source", lambda: convert_step(oversized), "STEP source is empty or exceeds the byte limit"),
                ("pre-cancelled", lambda: convert_step(invalid, control=cancelled), "STEP conversion was cancelled"),
                ("worker-status", lambda: convert_step(invalid), worker_message),
            )
            for name, operation, message in cases:
                with self.subTest(name=name):
                    with self.assertRaises(StepConversionError) as caught:
                        operation()
                    self.assertEqual(str(caught.exception), message)
            if os.name == "nt":
                with mock.patch.multiple(sys, frozen=True, executable=str(root / "AR4HMI.exe"), create=True):
                    with self.assertRaisesRegex(StepConversionError, "^STEP conversion worker could not be started$"):
                        convert_step(invalid)

    def test_worker_cancel_and_deadline_reap_children(self):
        with BoundedTemporaryDirectory(prefix="ar4-step-process-") as directory:
            root = Path(directory)
            sleeper = root / "sleeper.py"
            sleeper.write_text("import time\ntime.sleep(60)\n", encoding="utf-8")
            cases = (
                ("cancel", 3.0, 0.15, "STEP conversion was cancelled"),
                ("deadline", 0.15, None, "STEP conversion timed out"),
            )
            for name, deadline, cancel_after, message in cases:
                with self.subTest(name=name):
                    control = StepConversionControl()
                    process = subprocess.Popen(
                        [_CHILD_EXECUTABLE, str(sleeper)], stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_CHILD_ENVIRONMENT,
                    )
                    timer = Timer(cancel_after, control.cancel) if cancel_after else None
                    if timer:
                        timer.start()
                    terminal_at_settlement = None
                    try:
                        with self.assertRaisesRegex(StepConversionError, message):
                            _wait_for_worker(process, root / "output.stl", control, deadline)
                        terminal_at_settlement = process.poll()
                    finally:
                        if timer:
                            timer.cancel()
                            timer.join()
                        if process.poll() is None:
                            process.kill()
                        process.wait(timeout=5.0)
                    self.assertIsNotNone(terminal_at_settlement)
                    self.assertEqual(process.poll(), terminal_at_settlement)

    @unittest.skipUnless(_CADQUERY_AVAILABLE, "CadQuery is not installed")
    def test_cadquery_round_trip_preserves_source_and_reloads_scene(self):
        import vtk

        from ARrobots.HMI.cad_scene import PersistentCadScene

        with BoundedTemporaryDirectory(prefix="ar4-step-roundtrip-") as directory:
            root = Path(directory)
            source = root / "known-box.step"
            generator = ("import cadquery,os,sys;"
                         "cadquery.exporters.export(cadquery.Workplane('XY').box(12,23,34),sys.argv[1]);"
                         "os._exit(0)")
            subprocess.run([_CHILD_EXECUTABLE, "-c", generator, str(source)],
                           env=_CHILD_ENVIRONMENT, check=True, timeout=30.0)
            source_before = source.read_bytes()
            result = convert_step(source)
            self.assertEqual(result.label, "known-box")
            self.assertLessEqual(1, len(result.stl_payload))
            self.assertLessEqual(len(result.stl_payload), MAX_FILE_BYTES)
            self.assertEqual(source.read_bytes(), source_before)
            converted = root / "converted.stl"
            converted.write_bytes(result.stl_payload)
            workspace = root / "workspace"
            imported = PersistentCadScene(workspace).import_stl(
                converted, label=result.label
            )
            converted.unlink()
            reloaded = PersistentCadScene(workspace)
            self.assertEqual(reloaded.objects, (imported,))
            renderer = vtk.vtkRenderer()
            reloaded.bind_vtk(renderer, anchors={})
            actors = renderer.GetActors()
            actors.InitTraversal()
            self.assertEqual(actors.GetNumberOfItems(), 1)
            bounds = actors.GetNextActor().GetBounds()
            dimensions = tuple(bounds[index + 1] - bounds[index] for index in (0, 2, 4))
            for actual, expected in zip(dimensions, (12.0, 23.0, 34.0)):
                self.assertAlmostEqual(actual, expected, places=3)
