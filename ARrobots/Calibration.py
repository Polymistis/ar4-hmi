import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.NOTSET) # Inherit level from Parent

from collections.abc import Mapping
from decimal import Decimal
import io
import os
import pickle
import json
import tempfile

from ARrobots.calibration_schema import (
    CalibrationSchemaError,
    normalize_calibration_data,
)

_MAXIMUM_LEGACY_CALIBRATION_BYTES = 1024 * 1024
_LEGACY_NULL_DISCONNECTED_FIELDS = frozenset(("comPort", "com2Port"))
_LEGACY_NULL_EMPTY_FIELDS = frozenset(
    (
        "Servo0on",
        "Servo0off",
        "Servo1on",
        "Servo1off",
        "DO1on",
        "DO1off",
        "DO2on",
        "DO2off",
    )
)


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise CalibrationSchemaError(
                f"calibration data contains duplicate field {key}"
            )
        result[key] = value
    return result


def _reject_json_constant(value):
    raise CalibrationSchemaError(
        f"calibration data contains invalid numeric constant {value}"
    )


def _load_json_document(filename):
    with open(filename, 'r', encoding='utf-8') as source:
        return json.load(
            source,
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
            parse_float=Decimal,
        )


def _durably_replace_json_document(temporary_path, target):
    temporary_directory = os.path.dirname(os.path.abspath(temporary_path))
    target = os.path.abspath(target)
    target_directory = os.path.dirname(target)
    if temporary_directory != target_directory:
        raise OSError(
            "calibration durable replacement requires one directory"
        )

    if os.name == "nt":
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.MoveFileExW.argtypes = [
            wintypes.LPCWSTR,
            wintypes.LPCWSTR,
            wintypes.DWORD,
        ]
        kernel32.MoveFileExW.restype = wintypes.BOOL
        if not kernel32.MoveFileExW(
            temporary_path,
            target,
            0x00000001 | 0x00000008,
        ):
            raise ctypes.WinError(ctypes.get_last_error())
        return True

    if os.name != "posix":
        raise OSError(
            f"durable calibration replacement is unsupported on {os.name!r}"
        )
    directory_only = getattr(os, "O_DIRECTORY", None)
    no_follow = getattr(os, "O_NOFOLLOW", None)
    if not isinstance(directory_only, int) or not isinstance(no_follow, int):
        raise OSError(
            "durable calibration replacement requires protected directory opening"
        )
    directory_flags = os.O_RDONLY | directory_only | no_follow
    directory_flags |= getattr(os, "O_CLOEXEC", 0)
    directory_descriptor = os.open(target_directory, directory_flags)
    try:
        os.replace(temporary_path, target)
        os.fsync(directory_descriptor)
    finally:
        os.close(directory_descriptor)
    return True


def _write_json_document(filename, json_data):
    if not isinstance(json_data, str):
        raise TypeError("calibration JSON document must be text")
    target = os.path.abspath(os.fspath(filename))
    directory = os.path.dirname(target)
    if not os.path.isdir(directory):
        raise FileNotFoundError(
            f"calibration directory does not exist: {directory}"
        )
    descriptor = None
    temporary_path = None
    try:
        descriptor, temporary_path = tempfile.mkstemp(
            prefix=f".{os.path.basename(target)}.",
            suffix=".tmp",
            dir=directory,
            text=True,
        )
        with os.fdopen(
            descriptor,
            'w',
            encoding='utf-8',
            newline='\n',
        ) as destination:
            descriptor = None
            written = destination.write(json_data)
            if written != len(json_data):
                raise OSError("calibration JSON write was incomplete")
            destination.flush()
            os.fsync(destination.fileno())
        _durably_replace_json_document(temporary_path, target)
        temporary_path = None
        return True
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary_path is not None:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass
            except OSError as exc:
                logger.error(
                    f"Unable to remove temporary calibration file: {exc}"
                )


def save_calibration(
    calibration_data: dict,
    calibration_file: str="ARconfig.json",
    require_runtime_fields: bool=True,
) -> bool:
    if not isinstance(calibration_data, dict):
        logger.error("Calibration data must be a dictionary")
        return False
    if not isinstance(require_runtime_fields, bool):
        raise TypeError("runtime-field validation flag must be boolean")

    try:
        data_to_save = snapshot_calibration_values(calibration_data)
    except Exception as e:
        logger.error(f"Unable to resolve calibration values: {e}")
        return False

    try:
        data_to_save = normalize_calibration_data(
            data_to_save,
            require_runtime_fields=require_runtime_fields,
            migrate_legacy_switches=require_runtime_fields,
        )
    except (CalibrationSchemaError, TypeError) as e:
        logger.error(f"Calibration data failed schema validation: {e}")
        return False

    try:
        json_data = json.dumps(data_to_save, indent=4, allow_nan=False)
    except Exception as e:
        logger.error(f"Error saving json file as the data is not serializable: {e}")
        return False
    try:
        return _write_json_document(calibration_file, json_data)
    except Exception as e:
        logger.error(f"Error saving calibration file: {e}")
        return False


def snapshot_calibration_values(calibration_data):
    if not isinstance(calibration_data, dict):
        raise CalibrationSchemaError("calibration data must be a dictionary")
    snapshot = {}
    for key, value in calibration_data.items():
        if not isinstance(key, str):
            raise CalibrationSchemaError("calibration keys must be text")
        if isinstance(value, Mapping):
            snapshot[key] = dict(value)
        elif hasattr(value, 'get') and callable(value.get):
            try:
                snapshot[key] = value.get()
            except Exception as exc:
                raise CalibrationSchemaError(
                    f"unable to read calibration variable {key}"
                ) from exc
        else:
            snapshot[key] = value
    return snapshot


def load_calibration(
    calibration_file: str='ARconfig.json',
    defaults_file='defaults.json',
    allow_fallback: bool=True,
    require_runtime_fields: bool=True,
) -> dict | None:
    ''' Load calibration data from JSON or convert from old pickle format '''
    if not isinstance(allow_fallback, bool):
        raise TypeError("calibration fallback flag must be boolean")
    if not isinstance(require_runtime_fields, bool):
        raise TypeError("runtime-field validation flag must be boolean")
    calibration_data = None
    try:
        if os.path.exists(calibration_file):
            logger.debug("JSON config file found, loading")
            calibration_data = _load_json_document(calibration_file)
        elif not allow_fallback:
            logger.error(f"Calibration file not found: {calibration_file}")
            return None
        else:
            calibration_directory = os.path.dirname(
                os.path.abspath(calibration_file)
            )
            legacy_file = os.path.join(calibration_directory, "ARbot.cal")
            backup_file = os.path.join(
                calibration_directory,
                "ARbot.cal.bak",
            )
            if os.path.exists(legacy_file):
                logger.debug(
                    "No JSON config file found, attempting to load %s",
                    legacy_file,
                )
                calibration_data = convert_calibration(
                    legacy_file,
                    calibration_file,
                    backup_file,
                )
            else:
                logger.info("No configuration file found - Loading default settings")
                if os.path.exists(defaults_file):
                    logger.debug("default JSON config file found, loading")
                    calibration_data = _load_json_document(defaults_file)
                else:
                    logger.error(f"Default calibration file not found: {defaults_file}")
        calibration_data = normalize_calibration_data(
            calibration_data,
            require_runtime_fields=require_runtime_fields,
            migrate_legacy_switches=require_runtime_fields,
        )
        if (
            calibration_file != defaults_file
            and not os.path.exists(calibration_file)
            and not save_calibration(
                calibration_data,
                calibration_file,
                require_runtime_fields=require_runtime_fields,
            )
        ):
            logger.error(
                f"Unable to persist default calibration to {calibration_file}"
            )
        return calibration_data
    except Exception as e:
        logger.error(f"Unable to read calibration data: {e}")
        return None


class _RestrictedCalibrationUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        raise pickle.UnpicklingError(
            f"legacy calibration global {module}.{name} is not permitted"
        )


def _load_legacy_calibration_pickle(filename):
    with open(filename, "rb") as source:
        payload = source.read(_MAXIMUM_LEGACY_CALIBRATION_BYTES + 1)
    if len(payload) > _MAXIMUM_LEGACY_CALIBRATION_BYTES:
        raise CalibrationSchemaError(
            "legacy calibration exceeds the supported file size"
        )
    stream = io.BytesIO(payload)
    calibration = _RestrictedCalibrationUnpickler(stream).load()
    if stream.read(1):
        raise CalibrationSchemaError(
            "legacy calibration contains trailing data"
        )
    return calibration


class _LegacyCalibrationValues:
    FIELD_ORDER = (
        'J1AngCur',
        'J2AngCur',
        'J3AngCur',
        'J4AngCur',
        'J5AngCur',
        'J6AngCur',
        'XcurPos',
        'YcurPos',
        'ZcurPos',
        'RxcurPos',
        'RycurPos',
        'RzcurPos',
        'comPort',
        'Prog',
        'Servo0on',
        'Servo0off',
        'Servo1on',
        'Servo1off',
        'DO1on',
        'DO1off',
        'DO2on',
        'DO2off',
        'TFx',
        'TFy',
        'TFz',
        'TFrx',
        'TFry',
        'TFrz',
        'J7PosCur',
        'J8PosCur',
        'J9PosCur',
        'VisFileLoc',
        'VisProg',
        'VisOrigXpix',
        'VisOrigXmm',
        'VisOrigYpix',
        'VisOrigYmm',
        'VisEndXpix',
        'VisEndXmm',
        'VisEndYpix',
        'VisEndYmm',
        'J1calOff',
        'J2calOff',
        'J3calOff',
        'J4calOff',
        'J5calOff',
        'J6calOff',
        'J1OpenLoopVal',
        'J2OpenLoopVal',
        'J3OpenLoopVal',
        'J4OpenLoopVal',
        'J5OpenLoopVal',
        'J6OpenLoopVal',
        'com2Port',
        'curTheme',
        'J1CalStatVal',
        'J2CalStatVal',
        'J3CalStatVal',
        'J4CalStatVal',
        'J5CalStatVal',
        'J6CalStatVal',
        'J7PosLim',
        'J7rotation',
        'J7steps',
        'J7StepCur',
        'J1CalStatVal2',
        'J2CalStatVal2',
        'J3CalStatVal2',
        'J4CalStatVal2',
        'J5CalStatVal2',
        'J6CalStatVal2',
        'VisBrightVal',
        'VisContVal',
        'VisBacColor',
        'VisScore',
        'VisX1Val',
        'VisY1Val',
        'VisX2Val',
        'VisY2Val',
        'VisRobX1Val',
        'VisRobY1Val',
        'VisRobX2Val',
        'VisRobY2Val',
        'zoom',
        'pick180Val',
        'pickClosestVal',
        'curCam',
        'fullRotVal',
        'autoBGVal',
        'mX1val',
        'mY1val',
        'mX2val',
        'mY2val',
        'J8length',
        'J8rotation',
        'J8steps',
        'J9length',
        'J9rotation',
        'J9steps',
        'J7calOff',
        'J8calOff',
        'J9calOff',
        'GC_ST_E1',
        'GC_ST_E2',
        'GC_ST_E3',
        'GC_ST_E4',
        'GC_ST_E5',
        'GC_ST_E6',
        'GC_SToff_E1',
        'GC_SToff_E2',
        'GC_SToff_E3',
        'GC_SToff_E4',
        'GC_SToff_E5',
        'GC_SToff_E6',
        'DisableWristRotVal',
        'J1MotDir',
        'J2MotDir',
        'J3MotDir',
        'J4MotDir',
        'J5MotDir',
        'J6MotDir',
        'J7MotDir',
        'J8MotDir',
        'J9MotDir',
        'J1CalDir',
        'J2CalDir',
        'J3CalDir',
        'J4CalDir',
        'J5CalDir',
        'J6CalDir',
        'J7CalDir',
        'J8CalDir',
        'J9CalDir',
        'J1PosLim',
        'J1NegLim',
        'J2PosLim',
        'J2NegLim',
        'J3PosLim',
        'J3NegLim',
        'J4PosLim',
        'J4NegLim',
        'J5PosLim',
        'J5NegLim',
        'J6PosLim',
        'J6NegLim',
        'J1StepDeg',
        'J2StepDeg',
        'J3StepDeg',
        'J4StepDeg',
        'J5StepDeg',
        'J6StepDeg',
        'J1DriveMS',
        'J2DriveMS',
        'J3DriveMS',
        'J4DriveMS',
        'J5DriveMS',
        'J6DriveMS',
        'J1EncCPR',
        'J2EncCPR',
        'J3EncCPR',
        'J4EncCPR',
        'J5EncCPR',
        'J6EncCPR',
        'J1ΘDHpar',
        'J2ΘDHpar',
        'J3ΘDHpar',
        'J4ΘDHpar',
        'J5ΘDHpar',
        'J6ΘDHpar',
        'J1αDHpar',
        'J2αDHpar',
        'J3αDHpar',
        'J4αDHpar',
        'J5αDHpar',
        'J6αDHpar',
        'J1dDHpar',
        'J2dDHpar',
        'J3dDHpar',
        'J4dDHpar',
        'J5dDHpar',
        'J6dDHpar',
        'J1aDHpar',
        'J2aDHpar',
        'J3aDHpar',
        'J4aDHpar',
        'J5aDHpar',
        'J6aDHpar',
        'GC_ST_WC',
        'J7CalStatVal',
        'J8CalStatVal',
        'J9CalStatVal',
        'J7CalStatVal2',
        'J8CalStatVal2',
        'J9CalStatVal2',
        'setColor',
    )
    FIELD_COUNT = len(FIELD_ORDER)

    def __init__(self, values):
        if not isinstance(values, (list, tuple)):
            raise CalibrationSchemaError(
                "legacy calibration must contain an indexed value sequence"
            )
        if len(values) != self.FIELD_COUNT:
            raise CalibrationSchemaError(
                "legacy calibration contains an unsupported field layout"
            )
        if any(
            not isinstance(value, (str, int, float, type(None)))
            or isinstance(value, bool)
            for value in values
        ):
            raise CalibrationSchemaError(
                "legacy calibration values must be scalar text, numbers, or null"
            )
        self._values = tuple(values)

    def get(self, index):
        if not isinstance(index, str) or not index.isdecimal():
            raise CalibrationSchemaError(
                "legacy calibration index must be decimal text"
            )
        return self._values[int(index)]


def _migrate_legacy_calibration_nulls(calibration_data):
    migrated = dict(calibration_data)
    for key in _LEGACY_NULL_DISCONNECTED_FIELDS:
        if migrated.get(key) is None:
            migrated[key] = "None"
    for key in _LEGACY_NULL_EMPTY_FIELDS:
        if migrated.get(key) is None:
            migrated[key] = ""
    unsupported = sorted(
        key
        for key, value in migrated.items()
        if value is None
    )
    if unsupported:
        raise CalibrationSchemaError(
            "legacy calibration contains unsupported null fields: "
            + ", ".join(unsupported)
        )
    return migrated


def convert_calibration(
    legacy_file="ARbot.cal",
    calibration_file="ARconfig.json",
    backup_file="ARbot.cal.bak",
) -> dict | None:
    ''' Convert old ARbot.cal pickle file to new dictionary format and save as ARconfig.json '''
    if os.path.exists(legacy_file):
        logger.info(f"Converting {legacy_file} to JSON format")
        try:
            pickle_data = _load_legacy_calibration_pickle(legacy_file)
            calibration = _LegacyCalibrationValues(pickle_data)
        except Exception as e:
            logger.error(f"Error loading calibration: {e}")
            return None
    else:
        logger.error(f"No {legacy_file} file found, cannot convert")
        return None
    
    try:
        CAL = {
            field_name: calibration.get(str(index))
            for index, field_name in enumerate(
                _LegacyCalibrationValues.FIELD_ORDER
            )
        }
    except Exception as e:
        logger.error(f"Error converting calibration: {e}")
        return None
    
    try:
        CAL = _migrate_legacy_calibration_nulls(CAL)
        CAL = normalize_calibration_data(
            CAL,
            require_runtime_fields=True,
            migrate_legacy_switches=True,
        )
        if not save_calibration(CAL, calibration_file):
            logger.error("Converted calibration failed to persist")
            return None
    except Exception as e:
        logger.error(f"Error saving new calibration: {e}")
        return None
    try:
        logger.info(f"Backing up {legacy_file} to {backup_file}")
        os.rename(legacy_file, backup_file)
    except Exception as e:
        logger.warning(
            "Converted calibration was committed, but the legacy backup "
            f"rename failed: {e}"
        )
    
    return CAL
