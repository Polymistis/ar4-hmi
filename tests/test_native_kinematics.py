import os
import math
from importlib.machinery import EXTENSION_SUFFIXES
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import textwrap
import unittest

if __package__:
    from .bounded_temp import BoundedTemporaryDirectory
else:
    from bounded_temp import BoundedTemporaryDirectory
from ARrobots.HMI.joint_motion import (
    MotionInputError,
    parse_command_timing,
    validate_controller_filename,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
HARNESS_SOURCE = PROJECT_ROOT / "tests" / "native" / "kinematics_contract_test.cpp"
LINUX_BUILD_SCRIPT = PROJECT_ROOT / "ARrobots" / "src" / "build_kinematics.sh"


class NativeKinematicsContractTests(unittest.TestCase):
    @staticmethod
    def supported_packaged_runtime():
        return (
            sys.platform == "win32"
            and sys.implementation.name == "cpython"
            and sys.version_info[:2] == (3, 14)
            and platform.machine().lower() in ("amd64", "x86_64")
            and sys.maxsize > 2**32
        )

    @staticmethod
    def configure_native_defaults(native):
        native.robot_data_reset()
        native.set_dh_parameters_explicit(
            *[math.radians(value) for value in (0, -90, 0, 0, 0, 180)],
            *[math.radians(value) for value in (0, -90, 0, -90, 90, -90)],
            0,
            64.2,
            305,
            0,
            0,
            0,
            169.77,
            0,
            0,
            222.63,
            0,
            41,
        )
        native.set_joint_limits(
            [170, 90, 52, 180, 105, 180],
            [170, 42, 89, 180, 105, 180],
        )

    def test_sanitized_native_contract_harness(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable")

        with BoundedTemporaryDirectory() as directory:
            executable = Path(directory) / "kinematics_contract_test"
            compile_result = subprocess.run(
                [
                    compiler,
                    "-std=c++14",
                    "-O1",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-pedantic",
                    "-fsanitize=address,undefined",
                    "-fno-omit-frame-pointer",
                    "-pthread",
                    str(HARNESS_SOURCE),
                    "-o",
                    str(executable),
                ],
                cwd=PROJECT_ROOT,
                capture_output=True,
                text=True,
                timeout=120,
                check=False,
            )
            self.assertEqual(
                compile_result.returncode,
                0,
                compile_result.stdout + compile_result.stderr,
            )

            environment = os.environ.copy()
            environment["ASAN_OPTIONS"] = "abort_on_error=1:detect_leaks=1"
            run_result = subprocess.run(
                [str(executable)],
                cwd=PROJECT_ROOT,
                env=environment,
                capture_output=True,
                text=True,
                timeout=120,
                check=False,
            )
            self.assertEqual(
                run_result.returncode,
                0,
                run_result.stdout + run_result.stderr,
            )

            def firmware_accepts(kind, value):
                probe = subprocess.run(
                    [
                        str(executable),
                        "--protocol-probe",
                        kind,
                        value,
                    ],
                    cwd=PROJECT_ROOT,
                    env=environment,
                    capture_output=True,
                    text=True,
                    timeout=30,
                    check=False,
                )
                self.assertEqual(
                    probe.returncode,
                    0,
                    probe.stdout + probe.stderr,
                )
                self.assertIn(probe.stdout.strip(), ("accepted", "rejected"))
                return probe.stdout.strip() == "accepted"

            for ramp in ("0", "0.1", "25", "100", "100.1", "200"):
                command = (
                    "RJA1B2C3D4E5F6J70J80J90Sp50Ac10Dc20Rm"
                    f"{ramp}WNLm000000\n"
                )
                try:
                    parse_command_timing(command)
                except MotionInputError:
                    host_accepted = False
                else:
                    host_accepted = True
                self.assertEqual(
                    host_accepted,
                    firmware_accepts("ramp", ramp),
                    ramp,
                )

            filenames = (
                "demo.txt",
                "name with spaces.txt",
                ".",
                "..",
                "folder/demo.txt",
                "C:demo.txt",
                'bad"name.txt',
                "bad*name.txt",
                "bad<name.txt",
                "bad>name.txt",
                "bad?name.txt",
                "bad|name.txt",
                "bad\\name.txt",
                "é.txt",
                "a" * 255,
                "a" * 256,
            )
            for filename in filenames:
                try:
                    validate_controller_filename(filename, "test filename")
                except MotionInputError:
                    host_accepted = False
                else:
                    host_accepted = True
                self.assertEqual(
                    host_accepted,
                    firmware_accepts("filename", filename),
                    filename,
                )

    def test_linux_python_binding_source_build_and_import(self):
        if sys.platform != "linux":
            self.skipTest("Linux source-build contract runs only on Linux")
        bash = shutil.which("bash")
        cmake = shutil.which("cmake")
        self.assertIsNotNone(bash, "bash is required for the Linux native build")
        self.assertIsNotNone(cmake, "CMake is required for the Linux native build")

        with BoundedTemporaryDirectory() as directory:
            environment = os.environ.copy()
            environment["PYTHON"] = sys.executable
            environment["AR4_KINEMATICS_BUILD_DIR"] = directory
            build_result = subprocess.run(
                [bash, str(LINUX_BUILD_SCRIPT)],
                cwd=PROJECT_ROOT,
                env=environment,
                capture_output=True,
                text=True,
                timeout=240,
                check=False,
            )
            self.assertEqual(
                build_result.returncode,
                0,
                build_result.stdout + build_result.stderr,
            )

            artifacts = [
                path
                for path in Path(directory).rglob("robot_kinematics*")
                if path.is_file()
                and any(str(path).endswith(suffix) for suffix in EXTENSION_SUFFIXES)
            ]
            self.assertEqual(
                len(artifacts),
                1,
                f"expected one Linux extension artifact, found {artifacts}",
            )
            validation_script = textwrap.dedent(
                """\
                import importlib.util
                import math
                import pathlib
                import sys

                path = pathlib.Path(sys.argv[1])
                spec = importlib.util.spec_from_file_location(
                    "robot_kinematics",
                    path,
                )
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                assert callable(module.set_robot_configuration)
                assert callable(module.SolveInverseKinematicsConfigured)
                module.robot_data_reset()
                module.set_robot_configuration(
                    [0.0] * 24,
                    [180.0] * 6,
                    [180.0] * 6,
                    [0.0] * 6,
                )
                assert module.get_joint_limits() == (
                    [180.0] * 6,
                    [180.0] * 6,
                )
                theta = [
                    math.radians(value)
                    for value in (0, -90, 0, 0, 0, 180)
                ]
                alpha = [
                    math.radians(value)
                    for value in (0, -90, 0, -90, 90, -90)
                ]
                dh = (
                    theta
                    + alpha
                    + [0, 64.2, 305, 0, 0, 0]
                    + [169.77, 0, 0, 222.63, 0, 41]
                )
                module.set_robot_configuration(
                    dh,
                    [170, 90, 52, 180, 105, 180],
                    [170, 42, 89, 180, 105, 180],
                    [0.0] * 6,
                )
                configured_dh = module.get_dh_parameters()
                configured_limits = module.get_joint_limits()
                configured_tool = module.get_robot_tool_frame()
                largest_accepted_rotation = float.fromhex("0x1.fffffcp+127")
                maximum_float32 = float.fromhex("0x1.fffffep+127")

                module.set_robot_tool_frame(
                    0.0,
                    0.0,
                    0.0,
                    largest_accepted_rotation,
                    0.0,
                    0.0,
                )
                assert all(
                    math.isfinite(value)
                    for value in module.get_robot_tool_frame()
                )
                module.set_robot_tool_frame(*configured_tool)

                module.set_robot_configuration(
                    dh,
                    configured_limits[0],
                    configured_limits[1],
                    [0.0, 0.0, 0.0, largest_accepted_rotation, 0.0, 0.0],
                )
                assert all(
                    math.isfinite(value)
                    for value in module.get_robot_tool_frame()
                )
                assert module.get_dh_parameters() == configured_dh
                assert module.get_joint_limits() == configured_limits
                module.set_robot_configuration(
                    dh,
                    configured_limits[0],
                    configured_limits[1],
                    configured_tool,
                )

                for callback in (
                    lambda: module.set_robot_tool_frame(
                        0.0,
                        0.0,
                        0.0,
                        maximum_float32,
                        0.0,
                        0.0,
                    ),
                    lambda: module.set_robot_configuration(
                        dh,
                        configured_limits[0],
                        configured_limits[1],
                        [0.0, 0.0, 0.0, maximum_float32, 0.0, 0.0],
                    ),
                ):
                    try:
                        callback()
                    except ValueError:
                        pass
                    else:
                        raise AssertionError(
                            "maximum finite tool rotation was accepted"
                        )
                    assert module.get_dh_parameters() == configured_dh
                    assert module.get_joint_limits() == configured_limits
                    assert module.get_robot_tool_frame() == configured_tool

                underflow = [1e-300, 0.0, 0.0, 0.0, 0.0, 0.0]
                zeros = [0.0] * 6

                def require_underflow(callback):
                    try:
                        callback()
                    except ValueError as error:
                        assert "underflows" in str(error), error
                    else:
                        raise AssertionError("motion underflow was accepted")

                for callback in (
                    lambda: module.forward_kinematics(underflow),
                    lambda: module.inverse_kinematics(underflow, zeros),
                    lambda: module.inverse_kinematics(zeros, underflow),
                    lambda: module.inverse_kinematics_no_estimate(underflow),
                    lambda: module.SolveInverseKinematics(underflow, zeros),
                    lambda: module.SolveInverseKinematics(zeros, underflow),
                    lambda: module.SolveInverseKinematicsConfigured(
                        underflow,
                        zeros,
                        "A",
                    ),
                    lambda: module.SolveInverseKinematicsConfigured(
                        zeros,
                        underflow,
                        "A",
                    ),
                ):
                    require_underflow(callback)

                joints = [10.0, -20.0, 15.0, 35.0, 30.0, -25.0]
                native_target = list(module.forward_kinematics(joints))
                target = native_target[:3] + [
                    math.degrees(value)
                    for value in native_target[3:]
                ]
                solution = module.SolveInverseKinematicsConfigured(
                    target,
                    joints,
                    "A",
                )
                assert len(solution) == 6
                round_trip = module.forward_kinematics(solution)
                assert max(
                    abs(left - right)
                    for left, right in zip(round_trip, native_target)
                ) < 0.002
                """
            )
            validation = subprocess.run(
                [
                    sys.executable,
                    "-c",
                    validation_script,
                    str(artifacts[0]),
                ],
                cwd=PROJECT_ROOT,
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
            self.assertEqual(
                validation.returncode,
                0,
                validation.stdout + validation.stderr,
            )

    def test_loaded_python_binding_contract(self):
        expected_modules = [
            PROJECT_ROOT / "ARrobots" / f"robot_kinematics{suffix}"
            for suffix in EXTENSION_SUFFIXES
        ]
        try:
            from ARrobots import robot_kinematics as native
        except ImportError as error:
            if self.supported_packaged_runtime() or any(
                path.is_file() for path in expected_modules
            ):
                self.fail(f"compatible native extension failed to import: {error}")
            self.skipTest(f"compatible native extension is unavailable: {error}")

        configured_api = hasattr(native, "SolveInverseKinematicsConfigured")
        if not configured_api:
            if self.supported_packaged_runtime():
                self.fail("supported packaged runtime loaded the legacy native API")
            self.assertFalse(hasattr(native, "set_robot_configuration"))
            return
        self.assertTrue(hasattr(native, "set_robot_configuration"))
        native.robot_data_reset()
        self.assertEqual(native.get_joint_limits(), ([180] * 6, [180] * 6))
        self.configure_native_defaults(native)
        self.assertEqual(
            native.get_joint_limits(),
            ([170, 90, 52, 180, 105, 180], [170, 42, 89, 180, 105, 180]),
        )

        dh_parameters = native.get_dh_parameters()
        with self.assertRaises(ValueError):
            native.set_dh_parameters_explicit(
                *([0.0] * 23),
                math.inf,
            )
        self.assertEqual(native.get_dh_parameters(), dh_parameters)
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.set_dh_parameters_explicit(
                *([0.0] * 23),
                1e-300,
            )
        self.assertEqual(native.get_dh_parameters(), dh_parameters)

        with self.assertRaises(ValueError):
            native.set_joint_limits([170] * 6, [170, 170, -1, 170, 170, 170])
        self.assertEqual(
            native.get_joint_limits(),
            ([170, 90, 52, 180, 105, 180], [170, 42, 89, 180, 105, 180]),
        )

        with self.assertRaises(ValueError):
            native.forward_kinematics([0.0] * 5)
        with self.assertRaises(ValueError):
            native.forward_kinematics([0.0, 0.0, 0.0, math.nan, 0.0, 0.0])
        underflow = [1e-300, 0.0, 0.0, 0.0, 0.0, 0.0]
        zeros = [0.0] * 6
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.forward_kinematics(underflow)
        with self.assertRaisesRegex(ValueError, "native radians"):
            native.forward_kinematics([1e-44, *zeros[1:]])
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.inverse_kinematics(underflow, zeros)
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.inverse_kinematics(zeros, underflow)
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.inverse_kinematics_no_estimate(underflow)
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.SolveInverseKinematics(underflow, zeros)
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.SolveInverseKinematics(zeros, underflow)
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.SolveInverseKinematicsConfigured(underflow, zeros, "A")
        with self.assertRaisesRegex(ValueError, "underflows"):
            native.SolveInverseKinematicsConfigured(zeros, underflow, "A")
        with self.assertRaisesRegex(ValueError, "native radians"):
            native.SolveInverseKinematics(
                [0.0, 0.0, 0.0, 1e-44, 0.0, 0.0],
                zeros,
            )
        with self.assertRaisesRegex(ValueError, "native radians"):
            native.SolveInverseKinematicsConfigured(
                zeros,
                [1e-44, *zeros[1:]],
                "A",
            )
        configured_tool = native.get_robot_tool_frame()
        with self.assertRaises(ValueError):
            native.set_robot_tool_frame(0.0, 0.0, 0.0, 0.0, math.inf, 0.0)
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)
        with self.assertRaisesRegex(ValueError, "native radians"):
            native.set_robot_tool_frame(0.0, 0.0, 0.0, 1e-44, 0.0, 0.0)
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)
        largest_accepted_rotation = float.fromhex("0x1.fffffcp+127")
        maximum_float32 = float.fromhex("0x1.fffffep+127")
        native.set_robot_tool_frame(
            0.0,
            0.0,
            0.0,
            largest_accepted_rotation,
            0.0,
            0.0,
        )
        self.assertTrue(
            all(math.isfinite(value) for value in native.get_robot_tool_frame())
        )
        native.set_robot_tool_frame(*configured_tool)
        with self.assertRaises(ValueError):
            native.set_robot_tool_frame(
                0.0,
                0.0,
                0.0,
                maximum_float32,
                0.0,
                0.0,
            )
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)

        configured_dh = native.get_dh_parameters()
        flattened_dh = [
            configured_dh[joint][field]
            for field in range(4)
            for joint in range(6)
        ]
        configured_limits = native.get_joint_limits()
        native.set_robot_configuration(
            flattened_dh,
            configured_limits[0],
            configured_limits[1],
            configured_tool,
        )
        self.assertEqual(native.get_dh_parameters(), configured_dh)
        self.assertEqual(native.get_joint_limits(), configured_limits)
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)

        native.set_robot_configuration(
            flattened_dh,
            configured_limits[0],
            configured_limits[1],
            [0.0, 0.0, 0.0, largest_accepted_rotation, 0.0, 0.0],
        )
        self.assertEqual(native.get_dh_parameters(), configured_dh)
        self.assertEqual(native.get_joint_limits(), configured_limits)
        self.assertTrue(
            all(math.isfinite(value) for value in native.get_robot_tool_frame())
        )
        native.set_robot_configuration(
            flattened_dh,
            configured_limits[0],
            configured_limits[1],
            configured_tool,
        )

        with self.assertRaises(ValueError):
            native.set_robot_configuration(
                flattened_dh,
                configured_limits[0],
                configured_limits[1],
                [0.0, 0.0, 0.0, maximum_float32, 0.0, 0.0],
            )
        self.assertEqual(native.get_dh_parameters(), configured_dh)
        self.assertEqual(native.get_joint_limits(), configured_limits)
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)

        with self.assertRaisesRegex(ValueError, "underflows"):
            native.set_robot_configuration(
                [*flattened_dh[:-1], 1e-300],
                configured_limits[0],
                configured_limits[1],
                configured_tool,
            )
        self.assertEqual(native.get_dh_parameters(), configured_dh)
        self.assertEqual(native.get_joint_limits(), configured_limits)
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)

        with self.assertRaisesRegex(ValueError, "native radians"):
            native.set_robot_configuration(
                flattened_dh,
                configured_limits[0],
                configured_limits[1],
                [*configured_tool[:3], 1e-44, *configured_tool[4:]],
            )
        self.assertEqual(native.get_dh_parameters(), configured_dh)
        self.assertEqual(native.get_joint_limits(), configured_limits)
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)

        with self.assertRaises(ValueError):
            native.set_robot_configuration(
                flattened_dh,
                configured_limits[0],
                configured_limits[1],
                [*configured_tool[:5], math.inf],
            )
        self.assertEqual(native.get_dh_parameters(), configured_dh)
        self.assertEqual(native.get_joint_limits(), configured_limits)
        self.assertEqual(native.get_robot_tool_frame(), configured_tool)

        singular = [0.0, -30.0, 20.0, 30.0, 0.0, 40.0]
        target = native.forward_kinematics(singular)
        target[3:] = [math.degrees(value) for value in target[3:]]
        for wrist_config in ("A", "F", "N"):
            solution = native.SolveInverseKinematicsConfigured(
                target,
                singular,
                wrist_config,
            )
            self.assertEqual(len(solution), 6)
            for actual, expected in zip(solution, singular):
                self.assertAlmostEqual(actual, expected, places=3)

        multi_turn = [740.0, -20.0, 15.0, 35.0, 30.0, -25.0]
        multi_turn_estimate = [730.0, *multi_turn[1:]]
        native.set_joint_limits(
            [800.0, *configured_limits[0][1:]],
            [800.0, *configured_limits[1][1:]],
        )
        multi_turn_target = native.forward_kinematics(multi_turn)
        multi_turn_target[3:] = [
            math.degrees(value) for value in multi_turn_target[3:]
        ]
        multi_turn_solution = native.SolveInverseKinematicsConfigured(
            multi_turn_target,
            multi_turn_estimate,
            "A",
        )
        self.assertEqual(len(multi_turn_solution), 6)
        self.assertAlmostEqual(multi_turn_solution[0], 740.0, places=3)
        native.set_joint_limits(*configured_limits)

        target_radians = native.forward_kinematics(singular)
        zero_target_radians = native.forward_kinematics([0.0] * 6)
        lower_api_cases = (
            (native.inverse_kinematics(target_radians, singular), target_radians),
            (
                native.inverse_kinematics_no_estimate(zero_target_radians),
                zero_target_radians,
            ),
        )
        for solution, expected_pose in lower_api_cases:
            self.assertEqual(len(solution), 6)
            round_trip = native.forward_kinematics(solution)
            for actual, expected in zip(round_trip, expected_pose):
                self.assertAlmostEqual(actual, expected, places=3)

        with self.assertRaises(ValueError):
            native.SolveInverseKinematicsConfigured(target, singular, "X")
        with self.assertRaises(ValueError):
            native.SolveInverseKinematicsConfigured(
                [math.inf, 0.0, 0.0, 0.0, 0.0, 0.0],
                singular,
                "A",
            )
        self.assertEqual(
            native.SolveInverseKinematicsConfigured(
                [1000000.0, 1000000.0, 1000000.0, 0.0, 0.0, 0.0],
                singular,
                "A",
            ),
            [],
        )


if __name__ == "__main__":
    unittest.main()
