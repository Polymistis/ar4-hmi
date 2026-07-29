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
    FIELD_COUNT = 195

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
    CAL = {}

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
        # The index order is the on-disk legacy format and cannot be inferred.
        CAL['J1AngCur'] = calibration.get("0")
        CAL['J2AngCur'] = calibration.get("1")
        CAL['J3AngCur'] = calibration.get("2")
        CAL['J4AngCur'] = calibration.get("3")
        CAL['J5AngCur'] = calibration.get("4")
        CAL['J6AngCur'] = calibration.get("5")
        CAL['XcurPos'] = calibration.get("6")
        CAL['YcurPos'] = calibration.get("7")
        CAL['ZcurPos'] = calibration.get("8")
        CAL['RxcurPos'] = calibration.get("9")
        CAL['RycurPos'] = calibration.get("10")
        CAL['RzcurPos'] = calibration.get("11")
        CAL['comPort'] = calibration.get("12")
        CAL['Prog'] = calibration.get("13")
        CAL['Servo0on'] = calibration.get("14")
        CAL['Servo0off'] = calibration.get("15")
        CAL['Servo1on'] = calibration.get("16")
        CAL['Servo1off'] = calibration.get("17")
        CAL['DO1on'] = calibration.get("18")
        CAL['DO1off'] = calibration.get("19")
        CAL['DO2on'] = calibration.get("20")
        CAL['DO2off'] = calibration.get("21")
        CAL['TFx'] = calibration.get("22")
        CAL['TFy'] = calibration.get("23")
        CAL['TFz'] = calibration.get("24")
        CAL['TFrx'] = calibration.get("25")
        CAL['TFry'] = calibration.get("26")
        CAL['TFrz'] = calibration.get("27")
        CAL['J7PosCur'] = calibration.get("28")
        CAL['J8PosCur'] = calibration.get("29")
        CAL['J9PosCur'] = calibration.get("30")
        CAL['VisFileLoc'] = calibration.get("31")
        CAL['VisProg'] = calibration.get("32")
        CAL['VisOrigXpix'] = calibration.get("33")
        CAL['VisOrigXmm'] = calibration.get("34")
        CAL['VisOrigYpix'] = calibration.get("35")
        CAL['VisOrigYmm'] = calibration.get("36")
        CAL['VisEndXpix'] = calibration.get("37")
        CAL['VisEndXmm'] = calibration.get("38")
        CAL['VisEndYpix'] = calibration.get("39")
        CAL['VisEndYmm'] = calibration.get("40")
        CAL['J1calOff'] = calibration.get("41")
        CAL['J2calOff'] = calibration.get("42")
        CAL['J3calOff'] = calibration.get("43")
        CAL['J4calOff'] = calibration.get("44")
        CAL['J5calOff'] = calibration.get("45")
        CAL['J6calOff'] = calibration.get("46")
        CAL['J1OpenLoopVal'] = calibration.get("47")
        CAL['J2OpenLoopVal'] = calibration.get("48")
        CAL['J3OpenLoopVal'] = calibration.get("49")
        CAL['J4OpenLoopVal'] = calibration.get("50")
        CAL['J5OpenLoopVal'] = calibration.get("51")
        CAL['J6OpenLoopVal'] = calibration.get("52")
        CAL['com2Port'] = calibration.get("53")
        CAL['curTheme'] = calibration.get("54")
        CAL['J1CalStatVal'] = calibration.get("55")
        CAL['J2CalStatVal'] = calibration.get("56")
        CAL['J3CalStatVal'] = calibration.get("57")
        CAL['J4CalStatVal'] = calibration.get("58")
        CAL['J5CalStatVal'] = calibration.get("59")
        CAL['J6CalStatVal'] = calibration.get("60")
        CAL['J7PosLim'] = calibration.get("61")
        CAL['J7rotation'] = calibration.get("62")
        CAL['J7steps'] = calibration.get("63")
        CAL['J7StepCur'] = calibration.get("64")
        CAL['J1CalStatVal2'] = calibration.get("65")
        CAL['J2CalStatVal2'] = calibration.get("66")
        CAL['J3CalStatVal2'] = calibration.get("67")
        CAL['J4CalStatVal2'] = calibration.get("68")
        CAL['J5CalStatVal2'] = calibration.get("69")
        CAL['J6CalStatVal2'] = calibration.get("70")
        CAL['VisBrightVal'] = calibration.get("71")
        CAL['VisContVal'] = calibration.get("72")
        CAL['VisBacColor'] = calibration.get("73")
        CAL['VisScore'] = calibration.get("74")
        CAL['VisX1Val'] = calibration.get("75")
        CAL['VisY1Val'] = calibration.get("76")
        CAL['VisX2Val'] = calibration.get("77")
        CAL['VisY2Val'] = calibration.get("78")
        CAL['VisRobX1Val'] = calibration.get("79")
        CAL['VisRobY1Val'] = calibration.get("80")
        CAL['VisRobX2Val'] = calibration.get("81")
        CAL['VisRobY2Val'] = calibration.get("82")
        CAL['zoom'] = calibration.get("83")
        CAL['pick180Val'] = calibration.get("84")
        CAL['pickClosestVal'] = calibration.get("85")
        CAL['curCam'] = calibration.get("86")
        CAL['fullRotVal'] = calibration.get("87")
        CAL['autoBGVal'] = calibration.get("88")
        CAL['mX1val'] = calibration.get("89")
        CAL['mY1val'] = calibration.get("90")
        CAL['mX2val'] = calibration.get("91")
        CAL['mY2val'] = calibration.get("92")
        CAL['J8length'] = calibration.get("93")
        CAL['J8rotation'] = calibration.get("94")
        CAL['J8steps'] = calibration.get("95")
        CAL['J9length'] = calibration.get("96")
        CAL['J9rotation'] = calibration.get("97")
        CAL['J9steps'] = calibration.get("98")
        CAL['J7calOff'] = calibration.get("99")
        CAL['J8calOff'] = calibration.get("100")
        CAL['J9calOff'] = calibration.get("101")
        CAL['GC_ST_E1'] = calibration.get("102")
        CAL['GC_ST_E2'] = calibration.get("103")
        CAL['GC_ST_E3'] = calibration.get("104")
        CAL['GC_ST_E4'] = calibration.get("105")
        CAL['GC_ST_E5'] = calibration.get("106")
        CAL['GC_ST_E6'] = calibration.get("107")
        CAL['GC_SToff_E1'] = calibration.get("108")
        CAL['GC_SToff_E2'] = calibration.get("109")
        CAL['GC_SToff_E3'] = calibration.get("110")
        CAL['GC_SToff_E4'] = calibration.get("111")
        CAL['GC_SToff_E5'] = calibration.get("112")
        CAL['GC_SToff_E6'] = calibration.get("113")
        CAL['DisableWristRotVal'] = calibration.get("114")
        CAL['J1MotDir'] = calibration.get("115")
        CAL['J2MotDir'] = calibration.get("116")
        CAL['J3MotDir'] = calibration.get("117")
        CAL['J4MotDir'] = calibration.get("118")
        CAL['J5MotDir'] = calibration.get("119")
        CAL['J6MotDir'] = calibration.get("120")
        CAL['J7MotDir'] = calibration.get("121")
        CAL['J8MotDir'] = calibration.get("122")
        CAL['J9MotDir'] = calibration.get("123")
        CAL['J1CalDir'] = calibration.get("124")
        CAL['J2CalDir'] = calibration.get("125")
        CAL['J3CalDir'] = calibration.get("126")
        CAL['J4CalDir'] = calibration.get("127")
        CAL['J5CalDir'] = calibration.get("128")
        CAL['J6CalDir'] = calibration.get("129")
        CAL['J7CalDir'] = calibration.get("130")
        CAL['J8CalDir'] = calibration.get("131")
        CAL['J9CalDir'] = calibration.get("132")
        CAL['J1PosLim'] = calibration.get("133")
        CAL['J1NegLim'] = calibration.get("134")
        CAL['J2PosLim'] = calibration.get("135")
        CAL['J2NegLim'] = calibration.get("136")
        CAL['J3PosLim'] = calibration.get("137")
        CAL['J3NegLim'] = calibration.get("138")
        CAL['J4PosLim'] = calibration.get("139")
        CAL['J4NegLim'] = calibration.get("140")
        CAL['J5PosLim'] = calibration.get("141")
        CAL['J5NegLim'] = calibration.get("142")
        CAL['J6PosLim'] = calibration.get("143")
        CAL['J6NegLim'] = calibration.get("144")
        CAL['J1StepDeg'] = calibration.get("145")
        CAL['J2StepDeg'] = calibration.get("146")
        CAL['J3StepDeg'] = calibration.get("147")
        CAL['J4StepDeg'] = calibration.get("148")
        CAL['J5StepDeg'] = calibration.get("149")
        CAL['J6StepDeg'] = calibration.get("150")
        CAL['J1DriveMS'] = calibration.get("151")
        CAL['J2DriveMS'] = calibration.get("152")
        CAL['J3DriveMS'] = calibration.get("153")
        CAL['J4DriveMS'] = calibration.get("154")
        CAL['J5DriveMS'] = calibration.get("155")
        CAL['J6DriveMS'] = calibration.get("156")
        CAL['J1EncCPR'] = calibration.get("157")
        CAL['J2EncCPR'] = calibration.get("158")
        CAL['J3EncCPR'] = calibration.get("159")
        CAL['J4EncCPR'] = calibration.get("160")
        CAL['J5EncCPR'] = calibration.get("161")
        CAL['J6EncCPR'] = calibration.get("162")
        CAL['J1ΘDHpar'] = calibration.get("163")
        CAL['J2ΘDHpar'] = calibration.get("164")
        CAL['J3ΘDHpar'] = calibration.get("165")
        CAL['J4ΘDHpar'] = calibration.get("166")
        CAL['J5ΘDHpar'] = calibration.get("167")
        CAL['J6ΘDHpar'] = calibration.get("168")
        CAL['J1αDHpar'] = calibration.get("169")
        CAL['J2αDHpar'] = calibration.get("170")
        CAL['J3αDHpar'] = calibration.get("171")
        CAL['J4αDHpar'] = calibration.get("172")
        CAL['J5αDHpar'] = calibration.get("173")
        CAL['J6αDHpar'] = calibration.get("174")
        CAL['J1dDHpar'] = calibration.get("175")
        CAL['J2dDHpar'] = calibration.get("176")
        CAL['J3dDHpar'] = calibration.get("177")
        CAL['J4dDHpar'] = calibration.get("178")
        CAL['J5dDHpar'] = calibration.get("179")
        CAL['J6dDHpar'] = calibration.get("180")
        CAL['J1aDHpar'] = calibration.get("181")
        CAL['J2aDHpar'] = calibration.get("182")
        CAL['J3aDHpar'] = calibration.get("183")
        CAL['J4aDHpar'] = calibration.get("184")
        CAL['J5aDHpar'] = calibration.get("185")
        CAL['J6aDHpar'] = calibration.get("186")
        CAL['GC_ST_WC'] = calibration.get("187")
        CAL['J7CalStatVal'] = calibration.get("188")
        CAL['J8CalStatVal'] = calibration.get("189")
        CAL['J9CalStatVal'] = calibration.get("190")
        CAL['J7CalStatVal2'] = calibration.get("191")
        CAL['J8CalStatVal2'] = calibration.get("192")
        CAL['J9CalStatVal2'] = calibration.get("193")
        CAL['setColor'] = calibration.get("194")
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
