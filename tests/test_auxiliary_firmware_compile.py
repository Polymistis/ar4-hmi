import os
from pathlib import Path
import re
import unittest

if __package__:
    from .bounded_temp import BoundedTemporaryDirectory
    from .test_teensy_firmware_compile import _run_bounded_process_tree
else:
    from bounded_temp import BoundedTemporaryDirectory
    from test_teensy_firmware_compile import _run_bounded_process_tree


PROJECT_ROOT = Path(__file__).resolve().parents[1]
AUXILIARY_SKETCHES = (
    (
        "Nano",
        "arduino:avr:nano:cpu=atmega328old",
        PROJECT_ROOT / "ArduinoSketches" / "AR4_nano_sketch_v1.5",
    ),
    (
        "Mega",
        "arduino:avr:mega",
        PROJECT_ROOT / "ArduinoSketches" / "AR4_mega_sketch_v1.5",
    ),
)


class AuxiliaryFirmwareCompileTests(unittest.TestCase):
    def test_tracked_auxiliary_sources_compile_without_upload(self):
        cli_value = os.environ.get("AR4_ARDUINO_CLI")
        build_parent_value = os.environ.get(
            "AR4_AUXILIARY_BUILD_DIRECTORY"
        )
        missing = tuple(
            name
            for name, value in (
                ("AR4_ARDUINO_CLI", cli_value),
                (
                    "AR4_AUXILIARY_BUILD_DIRECTORY",
                    build_parent_value,
                ),
            )
            if not value
        )
        if missing:
            self.skipTest(
                "auxiliary firmware compile environment is unavailable: "
                + ", ".join(missing)
            )

        cli = Path(cli_value).resolve(strict=True)
        build_parent = Path(build_parent_value).resolve(strict=True)
        self.assertTrue(cli.is_file(), "Arduino CLI path is not a file")
        self.assertTrue(
            build_parent.is_dir(),
            "auxiliary build parent is not a directory",
        )
        self.assertNotEqual(build_parent, PROJECT_ROOT)
        self.assertNotIn(PROJECT_ROOT, build_parent.parents)

        for board_name, fqbn, sketch in AUXILIARY_SKETCHES:
            with self.subTest(board=board_name):
                with BoundedTemporaryDirectory(
                    prefix=f"ar4hmi-{board_name.casefold()}-compile-",
                    dir=build_parent,
                ) as build_directory:
                    command = (
                        str(cli),
                        "compile",
                        "--no-color",
                        "--verbose",
                        "--fqbn",
                        fqbn,
                        "--clean",
                        "--build-path",
                        build_directory,
                        str(sketch),
                    )
                    self.assertNotIn("--upload", command)
                    result = _run_bounded_process_tree(
                        command,
                        cwd=PROJECT_ROOT,
                        timeout=120,
                    )
                self.assertEqual(
                    result.returncode,
                    0,
                    result.stdout + result.stderr,
                )
                output = result.stdout + result.stderr
                self.assertIn("Sketch uses", output)
                self.assertIn("Global variables use", output)
                self.assertRegex(
                    output,
                    re.compile(
                        r"(?m)^(?:"
                        r"Servo\s+1\.3\.0(?:\s|$)"
                        r"|Using library Servo at version 1\.3\.0(?:\s|$)"
                        r")"
                    ),
                    "compile did not select Servo 1.3.0",
                )
                self.assertRegex(
                    output,
                    re.compile(
                        r"(?m)^(?:"
                        r"arduino:avr\s+1\.8\.8(?:\s|$)"
                        r"|Using platform arduino:avr at version "
                        r"1\.8\.8(?:\s|$)"
                        r")"
                    ),
                    "compile did not select Arduino AVR core 1.8.8",
                )


if __name__ == "__main__":
    unittest.main()
