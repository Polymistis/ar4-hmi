import ast
import copy
import ctypes
from decimal import Decimal
import inspect
import json
import os
from pathlib import Path
import pickle
import unittest
from unittest.mock import Mock, patch

if __package__:
    from .bounded_temp import BoundedTemporaryDirectory
else:
    from bounded_temp import BoundedTemporaryDirectory

from ARrobots.Calibration import (
    _LegacyCalibrationValues,
    _durably_replace_json_document,
    _load_json_document,
    _load_legacy_calibration_pickle,
    convert_calibration,
    load_calibration,
    save_calibration,
    snapshot_calibration_values,
)
import ARrobots.Calibration as calibration_module
from ARrobots.calibration_schema import (
    CALIBRATION_SWITCH_KEYS,
    CalibrationSchemaError,
    normalize_calibration_data,
    normalize_vision_background_color,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULTS_PATH = PROJECT_ROOT / "defaults.json"
CALIBRATION_SOURCE = PROJECT_ROOT / "ARrobots" / "Calibration.py"


def _legacy_values_from_defaults(defaults):
    tree = ast.parse(
        CALIBRATION_SOURCE.read_text(encoding="utf-8"),
        filename=str(CALIBRATION_SOURCE),
    )
    conversion = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "convert_calibration"
    )
    indexed_keys = {}
    for node in ast.walk(conversion):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        value = node.value
        if not (
            isinstance(target, ast.Subscript)
            and isinstance(target.value, ast.Name)
            and target.value.id == "CAL"
            and isinstance(value, ast.Call)
            and isinstance(value.func, ast.Attribute)
            and isinstance(value.func.value, ast.Name)
            and value.func.value.id == "calibration"
            and value.func.attr == "get"
            and len(value.args) == 1
        ):
            continue
        indexed_keys[int(ast.literal_eval(value.args[0]))] = ast.literal_eval(
            target.slice
        )
    expected_indices = set(range(195))
    if set(indexed_keys) != expected_indices:
        raise AssertionError("legacy calibration mapping is incomplete")
    return [
        str(defaults[indexed_keys[index]])
        for index in range(195)
    ]


class CalibrationSchemaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.defaults = json.loads(DEFAULTS_PATH.read_text(encoding="utf-8"))

    def test_tracked_defaults_are_canonical_and_machine_neutral(self):
        normalized = normalize_calibration_data(self.defaults)

        self.assertEqual(normalized, self.defaults)
        self.assertEqual(normalized["comPort"], "None")
        self.assertEqual(normalized["com2Port"], "None")
        self.assertEqual(normalized["auxiliaryBoard"], "None")
        self.assertIsInstance(normalized["J1DriveMS"], int)
        self.assertIsInstance(normalized["J1PosLim"], float)
        self.assertIsInstance(normalized["TFx"], float)
        self.assertIsInstance(normalized["VisScore"], float)
        self.assertIsInstance(normalized["GC_ST_E1"], float)
        self.assertIsInstance(normalized["J1AngCur"], str)
        self.assertEqual(normalized["VisBacColor"], [116, 116, 116])
        self.assertTrue(
            all(normalized[key] == "HIGH" for key in CALIBRATION_SWITCH_KEYS)
        )

    def test_legacy_runtime_profile_adds_compatibility_defaults(self):
        legacy = dict(self.defaults)
        for key in CALIBRATION_SWITCH_KEYS:
            del legacy[key]
        del legacy["auxiliaryBoard"]
        for key in (
            "DO3off",
            "DO3on",
            "DO4off",
            "DO4on",
            "DO5off",
            "DO5on",
            "DO6off",
            "DO6on",
            "Servo2off",
            "Servo2on",
            "Servo3off",
            "Servo3on",
        ):
            del legacy[key]
        legacy["com2Port"] = None
        legacy["J1PosLim"] = "170"
        legacy["J1MotDir"] = "0"
        legacy["TFx"] = " 0 "
        legacy["VisScore"] = "85"
        legacy["VisOrigXpix"] = "123.5"
        legacy["GC_ST_E1"] = "324.414"
        legacy["GC_ST_WC"] = " f "
        original = copy.deepcopy(legacy)

        normalized = normalize_calibration_data(legacy)

        self.assertEqual(legacy, original)
        self.assertEqual(normalized["auxiliaryBoard"], "None")
        self.assertEqual(normalized["com2Port"], "None")
        self.assertEqual(normalized["J1PosLim"], 170.0)
        self.assertEqual(normalized["J1MotDir"], 0)
        self.assertEqual(normalized["TFx"], 0.0)
        self.assertEqual(normalized["VisScore"], 85.0)
        self.assertEqual(normalized["VisOrigXpix"], 123.5)
        self.assertEqual(normalized["GC_ST_E1"], 324.414)
        self.assertEqual(normalized["GC_ST_WC"], "F")
        for key in CALIBRATION_SWITCH_KEYS:
            self.assertEqual(normalized[key], "HIGH")

    def test_partial_profile_does_not_gain_runtime_fields(self):
        profile = {
            "J1DriveMS": "800",
            "J1CalSwitch": " low ",
        }

        normalized = normalize_calibration_data(
            profile,
            require_runtime_fields=False,
            migrate_legacy_switches=False,
        )

        self.assertEqual(
            normalized,
            {
                "J1DriveMS": 800,
                "J1CalSwitch": "LOW",
            },
        )

    def test_invalid_numeric_and_enum_values_fail_without_mutation(self):
        cases = (
            ("J1DriveMS", True, "numeric, not boolean"),
            ("J1DriveMS", "800.5", "integer"),
            ("J1DriveMS", 0, "at least 1"),
            ("J1EncCPR", 2147483648, "at most 2147483647"),
            ("J1PosLim", "nan", "finite"),
            ("VisScore", 100.1, "at most 100"),
            ("VisX1Val", -1, "at least 0"),
            ("VisOrigXpix", "not-set", "must be numeric"),
            ("zoom", 0, "at least 1"),
            ("J1CalSwitch", "active", "HIGH or LOW"),
            ("GC_ST_WC", "A", "N or F"),
            ("auxiliaryBoard", "Auto", "None, Nano, or Mega"),
            ("setColor", "blue\nred", "control characters"),
        )
        for key, value, message in cases:
            with self.subTest(key=key, value=value):
                calibration = copy.deepcopy(self.defaults)
                calibration[key] = value
                original = copy.deepcopy(calibration)
                with self.assertRaisesRegex(CalibrationSchemaError, message):
                    normalize_calibration_data(calibration)
                self.assertEqual(calibration, original)

    def test_integer_rules_reject_exact_fractional_and_underflow_edges(self):
        cases = (
            ("J1MotDir", "1.0000000000000000000001"),
            ("J1DriveMS", "800.0000000000000000000001"),
            ("VisX1Val", "-1e-400"),
            ("J1EncCPR", "2147483647.0000001"),
        )
        for key, value in cases:
            with self.subTest(key=key, value=value):
                calibration = copy.deepcopy(self.defaults)
                calibration[key] = value
                with self.assertRaises(CalibrationSchemaError):
                    normalize_calibration_data(calibration)

    def test_json_loader_preserves_decimal_precision_for_schema_validation(self):
        replacements = (
            (
                '"J1DriveMS": 800',
                '"J1DriveMS": 800.0000000000000000000001',
                "J1DriveMS",
            ),
            (
                '"VisX1Val": 0',
                '"VisX1Val": -1e-400',
                "VisX1Val",
            ),
            (
                '"J1EncCPR": 4000',
                '"J1EncCPR": 2147483647.0000001',
                "J1EncCPR",
            ),
        )
        defaults_text = DEFAULTS_PATH.read_text(encoding="utf-8")
        with BoundedTemporaryDirectory(prefix="ar4-exact-json-") as directory:
            for old, new, key in replacements:
                with self.subTest(key=key):
                    calibration_path = Path(directory) / f"{key}.json"
                    calibration_path.write_text(
                        defaults_text.replace(old, new, 1),
                        encoding="utf-8",
                    )
                    loaded = _load_json_document(str(calibration_path))

                    self.assertIsInstance(loaded[key], Decimal)
                    with self.assertRaises(CalibrationSchemaError):
                        normalize_calibration_data(loaded)

    def test_missing_runtime_field_and_zero_joint_travel_are_rejected(self):
        missing = dict(self.defaults)
        del missing["J1DriveMS"]
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "missing required fields: J1DriveMS",
        ):
            normalize_calibration_data(missing)

        missing_text = dict(self.defaults)
        del missing_text["Prog"]
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "missing required fields: Prog",
        ):
            normalize_calibration_data(missing_text)

        no_travel = dict(self.defaults)
        no_travel["J1PosLim"] = 0
        no_travel["J1NegLim"] = 0
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "J1 configured travel must be positive",
        ):
            normalize_calibration_data(no_travel)

    def test_optional_external_axes_accept_zero_travel(self):
        calibration = copy.deepcopy(self.defaults)
        calibration["J7PosLim"] = 0
        calibration["J8length"] = 0
        calibration["J9length"] = 0
        calibration["J7PosCur"] = "0"
        calibration["J8PosCur"] = "0"
        calibration["J9PosCur"] = "0"

        normalized = normalize_calibration_data(calibration)

        self.assertEqual(normalized["J7PosLim"], 0.0)
        self.assertEqual(normalized["J8length"], 0.0)
        self.assertEqual(normalized["J9length"], 0.0)

    def test_controller_domains_and_derived_step_limits_fail_closed(self):
        cases = (
            ("TFx", 1e100),
            ("TFrx", 1e-50),
            ("J1StepDeg", 10_000_000),
        )
        for key, value in cases:
            with self.subTest(key=key):
                calibration = copy.deepcopy(self.defaults)
                calibration[key] = value
                with self.assertRaises(CalibrationSchemaError):
                    normalize_calibration_data(calibration)

        external_ratio = copy.deepcopy(self.defaults)
        external_ratio["J8steps"] = 3e38
        external_ratio["J8rotation"] = 1e-30
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "J8 steps per unit",
        ):
            normalize_calibration_data(external_ratio)

        load_max_encoder_scale = copy.deepcopy(self.defaults)
        load_max_encoder_scale["J1DriveMS"] = 12800
        load_max_encoder_scale["J1EncCPR"] = 4000
        normalized = normalize_calibration_data(load_max_encoder_scale)
        self.assertEqual(normalized["J1DriveMS"], 12800)
        self.assertEqual(normalized["J1EncCPR"], 4000)

        upper_encoder_scale = copy.deepcopy(self.defaults)
        upper_encoder_scale["J1DriveMS"] = 1
        upper_encoder_scale["J1EncCPR"] = 2147483647
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "J1 encoder multiplier",
        ):
            normalize_calibration_data(upper_encoder_scale)

        exact_lower_encoder_scale = copy.deepcopy(self.defaults)
        exact_lower_encoder_scale["J1DriveMS"] = 800
        exact_lower_encoder_scale["J1EncCPR"] = 800
        normalized = normalize_calibration_data(exact_lower_encoder_scale)
        self.assertEqual(normalized["J1DriveMS"], 800)
        self.assertEqual(normalized["J1EncCPR"], 800)

    def test_saved_joint_positions_use_plain_decimal_and_calibrated_limits(self):
        for token in ("1_000", "+1", "1e2"):
            with self.subTest(token=token):
                calibration = copy.deepcopy(self.defaults)
                calibration["J1AngCur"] = token
                with self.assertRaisesRegex(
                    CalibrationSchemaError,
                    "plain-decimal",
                ):
                    normalize_calibration_data(calibration)

        outside = copy.deepcopy(self.defaults)
        outside["J1AngCur"] = "999"
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "outside the calibrated limits",
        ):
            normalize_calibration_data(outside)

        numeric = copy.deepcopy(self.defaults)
        numeric["J1AngCur"] = 1.25
        self.assertEqual(
            normalize_calibration_data(numeric)["J1AngCur"],
            "1.25",
        )

    def test_auxiliary_values_are_optional_bounded_and_board_specific(self):
        calibration = copy.deepcopy(self.defaults)
        calibration["auxiliaryBoard"] = "Nano"
        calibration["Servo2on"] = " 180 "
        calibration["DO1on"] = " 8 "

        normalized = normalize_calibration_data(calibration)

        self.assertEqual(normalized["Servo2on"], "180")
        self.assertEqual(normalized["DO1on"], "8")

        cases = (
            ("Servo0on", "181", "between 0 and 180"),
            ("Servo0on", "1.5", "integer or empty"),
            ("DO1on", "7", "between 8 and 13"),
        )
        for key, value, message in cases:
            with self.subTest(key=key, value=value):
                invalid = copy.deepcopy(calibration)
                invalid[key] = value
                with self.assertRaisesRegex(CalibrationSchemaError, message):
                    normalize_calibration_data(invalid)

        no_board = copy.deepcopy(self.defaults)
        no_board["DO1on"] = "8"
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "requires a selected Nano or Mega",
        ):
            normalize_calibration_data(no_board)

        wrong_board = copy.deepcopy(self.defaults)
        wrong_board["auxiliaryBoard"] = "Mega"
        wrong_board["DO1on"] = "13"
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "between 28 and 53",
        ):
            normalize_calibration_data(wrong_board)

    def test_legacy_auxiliary_board_is_inferred_only_from_one_pin_range(self):
        nano = copy.deepcopy(self.defaults)
        del nano["auxiliaryBoard"]
        nano["DO1on"] = "8"
        self.assertEqual(
            normalize_calibration_data(nano)["auxiliaryBoard"],
            "Nano",
        )

        mega = copy.deepcopy(self.defaults)
        del mega["auxiliaryBoard"]
        mega["DO1on"] = "28"
        self.assertEqual(
            normalize_calibration_data(mega)["auxiliaryBoard"],
            "Mega",
        )

        mixed = copy.deepcopy(self.defaults)
        del mixed["auxiliaryBoard"]
        mixed["DO1on"] = "8"
        mixed["DO2on"] = "28"
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "span Nano and Mega pin ranges",
        ):
            normalize_calibration_data(mixed)

        unknown = copy.deepcopy(self.defaults)
        del unknown["auxiliaryBoard"]
        unknown["DO1on"] = "7"
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "select the board profile before migration",
        ):
            normalize_calibration_data(unknown)

    def test_vision_ranges_and_structured_background_color(self):
        calibration = copy.deepcopy(self.defaults)
        calibration["VisBacColor"] = "(1, 2, 255)"
        normalized = normalize_calibration_data(calibration)
        self.assertEqual(normalized["VisBacColor"], [1, 2, 255])
        self.assertEqual(
            normalize_vision_background_color("[3, 4, 5]"),
            [3, 4, 5],
        )
        self.assertEqual(
            normalize_vision_background_color("6, 7, 8"),
            [6, 7, 8],
        )

        cases = (
            ("VisBrightVal", -128),
            ("VisContVal", 128),
            ("zoom", 50.1),
        )
        for key, value in cases:
            with self.subTest(key=key):
                invalid = copy.deepcopy(self.defaults)
                invalid[key] = value
                with self.assertRaises(CalibrationSchemaError):
                    normalize_calibration_data(invalid)

        for color in (
            "__import__('os').getcwd()",
            [0, 0, 256],
            [0, 0, 1.5],
            [0, 0],
            {"red": 1},
        ):
            with self.subTest(color=color):
                with self.assertRaises(CalibrationSchemaError):
                    normalize_vision_background_color(color)

    def test_unknown_and_compound_fields_are_rejected(self):
        unsupported = copy.deepcopy(self.defaults)
        unsupported["extension"] = {"value": float("inf")}
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "unsupported fields: extension",
        ):
            normalize_calibration_data(unsupported)

        compound_text = copy.deepcopy(self.defaults)
        compound_text["Prog"] = {"path": "Home.ar4"}
        with self.assertRaisesRegex(
            CalibrationSchemaError,
            "Prog must be text",
        ):
            normalize_calibration_data(compound_text)


class CalibrationPersistenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.defaults = json.loads(DEFAULTS_PATH.read_text(encoding="utf-8"))

    def test_live_calibration_snapshot_preserves_binding_identity(self):
        class Binding:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

            def set(self, value):
                self.value = value

        binding = Binding(1)
        calibration = {
            "J1CalStatVal": binding,
            "metadata": {"value": 2},
        }

        snapshot = snapshot_calibration_values(calibration)

        self.assertEqual(
            snapshot,
            {
                "J1CalStatVal": 1,
                "metadata": {"value": 2},
            },
        )
        self.assertIs(calibration["J1CalStatVal"], binding)
        self.assertIsNot(snapshot["metadata"], calibration["metadata"])

    def test_runtime_round_trip_preserves_canonical_types(self):
        with BoundedTemporaryDirectory(prefix="ar4-calibration-") as directory:
            calibration_path = Path(directory) / "ARconfig.json"

            self.assertTrue(
                save_calibration(self.defaults, str(calibration_path))
            )
            raw = json.loads(calibration_path.read_text(encoding="utf-8"))
            self.assertIsInstance(raw["J1DriveMS"], int)
            self.assertIsInstance(raw["J1PosLim"], float)
            self.assertEqual(raw["J1CalSwitch"], "HIGH")

            loaded = load_calibration(
                str(calibration_path),
                allow_fallback=False,
            )
            self.assertEqual(loaded, self.defaults)

    def test_default_fallback_is_validated_and_persisted(self):
        with BoundedTemporaryDirectory(prefix="ar4-default-fallback-") as directory:
            calibration_path = Path(directory) / "ARconfig.json"
            defaults_path = Path(directory) / "defaults.json"
            defaults_path.write_text(
                json.dumps(self.defaults),
                encoding="utf-8",
            )

            loaded = load_calibration(
                str(calibration_path),
                defaults_file=str(defaults_path),
            )

            self.assertEqual(loaded, self.defaults)
            self.assertEqual(
                json.loads(calibration_path.read_text(encoding="utf-8")),
                self.defaults,
            )

    def test_partial_profile_round_trip_stays_partial(self):
        with BoundedTemporaryDirectory(prefix="ar4-profile-") as directory:
            profile_path = Path(directory) / "custom.json"

            self.assertTrue(
                save_calibration(
                    {"J1DriveMS": "800"},
                    str(profile_path),
                    require_runtime_fields=False,
                )
            )
            self.assertEqual(
                json.loads(profile_path.read_text(encoding="utf-8")),
                {"J1DriveMS": 800},
            )
            self.assertEqual(
                load_calibration(
                    str(profile_path),
                    allow_fallback=False,
                    require_runtime_fields=False,
                ),
                {"J1DriveMS": 800},
            )

    def test_runtime_load_migrates_legacy_switch_and_board_fields_in_memory(self):
        with BoundedTemporaryDirectory(prefix="ar4-legacy-config-") as directory:
            calibration_path = Path(directory) / "ARconfig.json"
            legacy = dict(self.defaults)
            for key in CALIBRATION_SWITCH_KEYS:
                del legacy[key]
            del legacy["auxiliaryBoard"]
            calibration_path.write_text(
                json.dumps(legacy),
                encoding="utf-8",
            )

            loaded = load_calibration(
                str(calibration_path),
                allow_fallback=False,
            )

            self.assertEqual(loaded["auxiliaryBoard"], "None")
            for key in CALIBRATION_SWITCH_KEYS:
                self.assertEqual(loaded[key], "HIGH")
            persisted = json.loads(calibration_path.read_text(encoding="utf-8"))
            self.assertNotIn("auxiliaryBoard", persisted)
            self.assertTrue(
                all(key not in persisted for key in CALIBRATION_SWITCH_KEYS)
            )

    def test_duplicate_and_nonfinite_json_fail_closed(self):
        documents = (
            '{"J1DriveMS":800,"J1DriveMS":1600}',
            '{"J1DriveMS":NaN}',
            '{"VisBacColor":[1e999,0,0]}',
        )
        with BoundedTemporaryDirectory(prefix="ar4-invalid-config-") as directory:
            calibration_path = Path(directory) / "custom.json"
            for document in documents:
                with self.subTest(document=document):
                    calibration_path.write_text(document, encoding="utf-8")
                    with self.assertLogs(
                        "ARrobots.Calibration",
                        level="ERROR",
                    ):
                        self.assertIsNone(
                            load_calibration(
                                str(calibration_path),
                                allow_fallback=False,
                                require_runtime_fields=False,
                            )
                        )

    def test_failed_tk_value_read_does_not_write_null(self):
        class UnavailableValue:
            @staticmethod
            def get():
                raise RuntimeError("widget unavailable")

        with BoundedTemporaryDirectory(prefix="ar4-failed-save-") as directory:
            calibration_path = Path(directory) / "custom.json"

            with self.assertLogs("ARrobots.Calibration", level="ERROR"):
                self.assertFalse(
                    save_calibration(
                        {"field": UnavailableValue()},
                        str(calibration_path),
                        require_runtime_fields=False,
                    )
                )
            self.assertFalse(calibration_path.exists())

    def test_mapping_value_is_validated_without_tk_get_dispatch(self):
        with BoundedTemporaryDirectory(prefix="ar4-mapping-save-") as directory:
            calibration_path = Path(directory) / "custom.json"

            with self.assertLogs("ARrobots.Calibration", level="ERROR"):
                self.assertFalse(
                    save_calibration(
                        {"Prog": {"path": "Home.ar4"}},
                        str(calibration_path),
                        require_runtime_fields=False,
                    )
                )

            self.assertFalse(calibration_path.exists())

    def test_short_write_preserves_existing_calibration(self):
        class ShortWriter:
            def __init__(self, descriptor):
                self.descriptor = descriptor

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, traceback):
                os.close(self.descriptor)

            @staticmethod
            def write(document):
                return max(0, len(document) - 1)

            @staticmethod
            def flush():
                raise AssertionError("short writes must not be synchronized")

            def fileno(self):
                return self.descriptor

        def short_fdopen(descriptor, *args, **kwargs):
            return ShortWriter(descriptor)

        with BoundedTemporaryDirectory(prefix="ar4-short-write-") as directory:
            calibration_path = Path(directory) / "custom.json"
            calibration_path.write_text('{"preserved":true}', encoding="utf-8")

            with (
                patch(
                    "ARrobots.Calibration.os.fdopen",
                    side_effect=short_fdopen,
                ),
                self.assertLogs("ARrobots.Calibration", level="ERROR"),
            ):
                self.assertFalse(
                    save_calibration(
                        {"J1DriveMS": "800"},
                        str(calibration_path),
                        require_runtime_fields=False,
                    )
                )

            self.assertEqual(
                calibration_path.read_text(encoding="utf-8"),
                '{"preserved":true}',
            )
            self.assertEqual(
                tuple(Path(directory).glob(".custom.json.*.tmp")),
                (),
            )

    def test_persistence_routes_through_durable_replacement(self):
        with BoundedTemporaryDirectory(prefix="ar4-durable-save-") as directory:
            calibration_path = Path(directory) / "custom.json"

            with patch(
                "ARrobots.Calibration._durably_replace_json_document",
                wraps=_durably_replace_json_document,
            ) as durable_replace:
                self.assertTrue(
                    save_calibration(
                        {"J1DriveMS": "800"},
                        str(calibration_path),
                        require_runtime_fields=False,
                    )
                )

            durable_replace.assert_called_once()
            self.assertEqual(
                json.loads(calibration_path.read_text(encoding="utf-8")),
                {"J1DriveMS": 800},
            )

    def test_durable_replacement_rejects_cross_directory_paths(self):
        with (
            BoundedTemporaryDirectory(prefix="ar4-durable-source-") as source,
            BoundedTemporaryDirectory(prefix="ar4-durable-target-") as target,
        ):
            temporary_path = Path(source) / "temporary.json"
            target_path = Path(target) / "target.json"
            temporary_path.write_text("{}", encoding="utf-8")

            with self.assertRaisesRegex(OSError, "requires one directory"):
                _durably_replace_json_document(
                    str(temporary_path),
                    str(target_path),
                )

    def test_windows_durable_replacement_uses_write_through(self):
        class MoveFile:
            def __init__(self):
                self.calls = []
                self.argtypes = None
                self.restype = None

            def __call__(self, source, target, flags):
                self.calls.append((source, target, flags))
                return 1

        move_file = MoveFile()
        kernel32 = type("Kernel32", (), {"MoveFileExW": move_file})()
        with BoundedTemporaryDirectory(prefix="ar4-windows-durable-") as directory:
            temporary_path = str(Path(directory) / "temporary.json")
            target_path = str(Path(directory) / "target.json")

            with (
                patch.object(calibration_module.os, "name", "nt"),
                patch.object(
                    ctypes,
                    "WinDLL",
                    return_value=kernel32,
                    create=True,
                ),
            ):
                self.assertTrue(
                    _durably_replace_json_document(
                        temporary_path,
                        target_path,
                    )
                )

        self.assertEqual(len(move_file.calls), 1)
        self.assertEqual(move_file.calls[0][0], temporary_path)
        self.assertEqual(move_file.calls[0][1], os.path.abspath(target_path))
        self.assertEqual(
            move_file.calls[0][2],
            0x00000001 | 0x00000008,
        )

    def test_posix_durable_replacement_syncs_directory_metadata(self):
        with BoundedTemporaryDirectory(prefix="ar4-posix-durable-") as directory:
            temporary_path = str(Path(directory) / "temporary.json")
            target_path = str(Path(directory) / "target.json")
            open_directory = Mock(return_value=71)
            replace = Mock()
            synchronize = Mock()
            close = Mock()

            with (
                patch.object(calibration_module.os, "name", "posix"),
                patch.object(
                    calibration_module.os,
                    "O_DIRECTORY",
                    0x010000,
                    create=True,
                ),
                patch.object(
                    calibration_module.os,
                    "O_NOFOLLOW",
                    0x020000,
                    create=True,
                ),
                patch.object(
                    calibration_module.os,
                    "O_CLOEXEC",
                    0x040000,
                    create=True,
                ),
                patch.object(
                    calibration_module.os,
                    "open",
                    open_directory,
                ),
                patch.object(calibration_module.os, "replace", replace),
                patch.object(calibration_module.os, "fsync", synchronize),
                patch.object(calibration_module.os, "close", close),
            ):
                self.assertTrue(
                    _durably_replace_json_document(
                        temporary_path,
                        target_path,
                    )
                )

        open_directory.assert_called_once()
        opened_path, flags = open_directory.call_args.args
        self.assertEqual(opened_path, os.path.abspath(directory))
        self.assertEqual(flags & 0x010000, 0x010000)
        self.assertEqual(flags & 0x020000, 0x020000)
        self.assertEqual(flags & 0x040000, 0x040000)
        replace.assert_called_once_with(
            temporary_path,
            os.path.abspath(target_path),
        )
        synchronize.assert_called_once_with(71)
        close.assert_called_once_with(71)

    def test_failed_atomic_replace_preserves_existing_calibration(self):
        with BoundedTemporaryDirectory(prefix="ar4-failed-replace-") as directory:
            calibration_path = Path(directory) / "custom.json"
            calibration_path.write_text('{"preserved":true}', encoding="utf-8")

            with (
                patch(
                    "ARrobots.Calibration._durably_replace_json_document",
                    side_effect=OSError("replace unavailable"),
                ),
                self.assertLogs("ARrobots.Calibration", level="ERROR"),
            ):
                self.assertFalse(
                    save_calibration(
                        {"J1DriveMS": "800"},
                        str(calibration_path),
                        require_runtime_fields=False,
                    )
                )

            self.assertEqual(
                calibration_path.read_text(encoding="utf-8"),
                '{"preserved":true}',
            )
            self.assertEqual(
                tuple(Path(directory).glob(".custom.json.*.tmp")),
                (),
            )


class LegacyCalibrationConversionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.defaults = json.loads(DEFAULTS_PATH.read_text(encoding="utf-8"))
        cls.legacy_values = _legacy_values_from_defaults(cls.defaults)

    def test_legacy_value_adapter_rejects_invalid_container_and_scalar_types(self):
        valid = list(self.legacy_values)
        cases = (
            (None, "indexed value sequence"),
            ({"0": "value"}, "indexed value sequence"),
            (valid[:-1], "unsupported field layout"),
            (valid + ["extra"], "unsupported field layout"),
            (valid[:10] + [{"nested": True}] + valid[11:], "must be scalar"),
            (valid[:10] + [True] + valid[11:], "must be scalar"),
        )
        for values, message in cases:
            with self.subTest(message=message, value_type=type(values)):
                with self.assertRaisesRegex(
                    CalibrationSchemaError,
                    message,
                ):
                    _LegacyCalibrationValues(values)

    def test_legacy_value_adapter_preserves_valid_scalar_types(self):
        values = list(self.legacy_values)
        values[0] = 12.5
        values[14] = None
        values[151] = 800

        calibration = _LegacyCalibrationValues(values)

        self.assertEqual(calibration.get("0"), 12.5)
        self.assertIsNone(calibration.get("14"))
        self.assertEqual(calibration.get("151"), 800)

    def test_restricted_legacy_loader_rejects_global_construction(self):
        class DangerousValue:
            def __reduce__(self):
                return (eval, ("1 + 1",))

        with BoundedTemporaryDirectory(prefix="ar4-legacy-security-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            legacy_path.write_bytes(pickle.dumps([DangerousValue()]))

            with self.assertRaises(pickle.UnpicklingError):
                _load_legacy_calibration_pickle(str(legacy_path))

    def test_restricted_legacy_loader_rejects_size_and_trailing_data(self):
        with BoundedTemporaryDirectory(prefix="ar4-legacy-bounds-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            legacy_path.write_bytes(
                b"x" * (
                    calibration_module._MAXIMUM_LEGACY_CALIBRATION_BYTES
                    + 1
                )
            )
            with self.assertRaisesRegex(
                CalibrationSchemaError,
                "supported file size",
            ):
                _load_legacy_calibration_pickle(str(legacy_path))

            legacy_path.write_bytes(pickle.dumps([]) + b"trailing")
            with self.assertRaisesRegex(
                CalibrationSchemaError,
                "trailing data",
            ):
                _load_legacy_calibration_pickle(str(legacy_path))

    def test_legacy_conversion_validates_persists_and_backs_up(self):
        with BoundedTemporaryDirectory(prefix="ar4-legacy-convert-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            calibration_path = Path(directory) / "ARconfig.json"
            backup_path = Path(directory) / "ARbot.cal.bak"
            legacy_values = list(self.legacy_values)
            legacy_values[53] = None
            legacy_path.write_bytes(pickle.dumps(legacy_values))

            converted = convert_calibration(
                str(legacy_path),
                str(calibration_path),
                str(backup_path),
            )

            self.assertIsInstance(converted, dict)
            self.assertEqual(converted["VisBacColor"], [116, 116, 116])
            self.assertEqual(converted["auxiliaryBoard"], "None")
            self.assertEqual(converted["com2Port"], "None")
            self.assertTrue(
                all(
                    converted[key] == "HIGH"
                    for key in CALIBRATION_SWITCH_KEYS
                )
            )
            self.assertEqual(
                json.loads(calibration_path.read_text(encoding="utf-8")),
                converted,
            )
            self.assertFalse(legacy_path.exists())
            self.assertTrue(backup_path.exists())

    def test_legacy_conversion_migrates_only_supported_null_fields(self):
        with BoundedTemporaryDirectory(prefix="ar4-legacy-null-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            calibration_path = Path(directory) / "ARconfig.json"
            backup_path = Path(directory) / "ARbot.cal.bak"
            legacy_values = list(self.legacy_values)
            for index in (12, 53, 14, 15, 16, 17, 18, 19, 20, 21):
                legacy_values[index] = None
            legacy_values[133] = 170.0
            legacy_values[151] = 800
            legacy_path.write_bytes(pickle.dumps(legacy_values))

            converted = convert_calibration(
                str(legacy_path),
                str(calibration_path),
                str(backup_path),
            )

            self.assertIsInstance(converted, dict)
            self.assertEqual(converted["comPort"], "None")
            self.assertEqual(converted["com2Port"], "None")
            for key in (
                "Servo0on",
                "Servo0off",
                "Servo1on",
                "Servo1off",
                "DO1on",
                "DO1off",
                "DO2on",
                "DO2off",
            ):
                self.assertEqual(converted[key], "")
            self.assertEqual(converted["J1PosLim"], 170.0)
            self.assertEqual(converted["J1DriveMS"], 800)

    def test_legacy_conversion_rejects_null_in_required_text_field(self):
        with BoundedTemporaryDirectory(prefix="ar4-legacy-null-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            calibration_path = Path(directory) / "ARconfig.json"
            backup_path = Path(directory) / "ARbot.cal.bak"
            legacy_values = list(self.legacy_values)
            legacy_values[13] = None
            legacy_path.write_bytes(pickle.dumps(legacy_values))

            with self.assertLogs("ARrobots.Calibration", level="ERROR"):
                converted = convert_calibration(
                    str(legacy_path),
                    str(calibration_path),
                    str(backup_path),
                )

            self.assertIsNone(converted)
            self.assertTrue(legacy_path.exists())
            self.assertFalse(calibration_path.exists())
            self.assertFalse(backup_path.exists())

    def test_legacy_conversion_infers_auxiliary_board_from_saved_outputs(self):
        with BoundedTemporaryDirectory(prefix="ar4-legacy-board-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            calibration_path = Path(directory) / "ARconfig.json"
            backup_path = Path(directory) / "ARbot.cal.bak"
            legacy_values = list(self.legacy_values)
            legacy_values[18] = "8"
            legacy_path.write_bytes(pickle.dumps(legacy_values))

            converted = convert_calibration(
                str(legacy_path),
                str(calibration_path),
                str(backup_path),
            )

            self.assertIsInstance(converted, dict)
            self.assertEqual(converted["auxiliaryBoard"], "Nano")
            self.assertEqual(converted["DO1on"], "8")

    def test_custom_runtime_path_uses_colocated_legacy_conversion_paths(self):
        with BoundedTemporaryDirectory(prefix="ar4-custom-legacy-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            calibration_path = Path(directory) / "custom-runtime.json"
            backup_path = Path(directory) / "ARbot.cal.bak"
            legacy_values = list(self.legacy_values)
            legacy_values[18] = "8"
            legacy_path.write_bytes(pickle.dumps(legacy_values))

            loaded = load_calibration(
                str(calibration_path),
                defaults_file=str(DEFAULTS_PATH),
            )

            self.assertIsInstance(loaded, dict)
            self.assertEqual(loaded["auxiliaryBoard"], "Nano")
            self.assertTrue(calibration_path.exists())
            self.assertFalse(legacy_path.exists())
            self.assertTrue(backup_path.exists())

    def test_legacy_conversion_default_backup_matches_source_name_case(self):
        parameters = inspect.signature(convert_calibration).parameters
        legacy_default = parameters["legacy_file"].default
        backup_default = parameters["backup_file"].default

        self.assertEqual(legacy_default, "ARbot.cal")
        self.assertEqual(backup_default, f"{legacy_default}.bak")

    def test_backup_failure_does_not_misreport_committed_conversion(self):
        with BoundedTemporaryDirectory(prefix="ar4-legacy-backup-") as directory:
            legacy_path = Path(directory) / "ARbot.cal"
            calibration_path = Path(directory) / "ARconfig.json"
            backup_path = Path(directory) / "ARbot.cal.bak"
            legacy_path.write_bytes(pickle.dumps(self.legacy_values))

            with (
                patch(
                    "ARrobots.Calibration.os.rename",
                    side_effect=OSError("backup unavailable"),
                ),
                self.assertLogs(
                    "ARrobots.Calibration",
                    level="WARNING",
                ),
            ):
                converted = convert_calibration(
                    str(legacy_path),
                    str(calibration_path),
                    str(backup_path),
                )

            self.assertIsInstance(converted, dict)
            self.assertTrue(calibration_path.exists())
            self.assertTrue(legacy_path.exists())


if __name__ == "__main__":
    unittest.main()
