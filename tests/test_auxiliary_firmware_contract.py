from pathlib import Path
import re
import unittest

from ARrobots.HMI.joint_motion import (
    AUXILIARY_BOARD_INPUT_PINS,
    AUXILIARY_BOARD_MEGA,
    AUXILIARY_BOARD_NANO,
    AUXILIARY_BOARD_OUTPUT_PINS,
    AUXILIARY_BOARD_SERVO_CHANNELS,
    AUXILIARY_WAIT_MAXIMUM_SECONDS,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
NANO_DIRECTORY = (
    PROJECT_ROOT / "ArduinoSketches" / "AR4_nano_sketch_v1.5"
)
MEGA_DIRECTORY = (
    PROJECT_ROOT / "ArduinoSketches" / "AR4_mega_sketch_v1.5"
)
NANO_SKETCH = NANO_DIRECTORY / "AR4_nano_sketch_v1.5.ino"
MEGA_SKETCH = MEGA_DIRECTORY / "AR4_mega_sketch_v1.5.ino"
NANO_PROTOCOL = NANO_DIRECTORY / "auxiliary_protocol_contract.h"
MEGA_PROTOCOL = MEGA_DIRECTORY / "auxiliary_protocol_contract.h"


def source_region(source, start, end):
    start_index = source.index(start)
    end_index = source.index(end, start_index)
    return source[start_index:end_index]


def array_body(source, name):
    match = re.search(
        rf"\b{name}\s*\[[^\]]*\]\s*=\s*\{{(?P<body>.*?)\}};",
        source,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"missing firmware array: {name}")
    return match.group("body")


class AuxiliaryFirmwareContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nano_source = NANO_SKETCH.read_text(encoding="utf-8")
        cls.mega_source = MEGA_SKETCH.read_text(encoding="utf-8")
        cls.protocol_source = NANO_PROTOCOL.read_text(encoding="utf-8")

    def test_board_profiles_match_host_contract(self):
        self.assertEqual(
            AUXILIARY_BOARD_INPUT_PINS[AUXILIARY_BOARD_NANO],
            frozenset(range(2, 8)),
        )
        self.assertEqual(
            AUXILIARY_BOARD_INPUT_PINS[AUXILIARY_BOARD_MEGA],
            frozenset(range(2, 28)),
        )
        self.assertEqual(
            AUXILIARY_BOARD_OUTPUT_PINS[AUXILIARY_BOARD_NANO],
            frozenset(range(8, 14)),
        )
        self.assertEqual(
            AUXILIARY_BOARD_OUTPUT_PINS[AUXILIARY_BOARD_MEGA],
            frozenset(range(28, 54)),
        )
        self.assertEqual(
            AUXILIARY_BOARD_SERVO_CHANNELS[AUXILIARY_BOARD_NANO],
            frozenset(range(6)),
        )
        self.assertEqual(
            AUXILIARY_BOARD_SERVO_CHANNELS[AUXILIARY_BOARD_MEGA],
            frozenset(range(7)),
        )
        self.assertIn(
            f"kMaximumWaitSeconds = {AUXILIARY_WAIT_MAXIMUM_SECONDS}UL",
            self.protocol_source,
        )
        nano_inputs = tuple(
            int(value.strip())
            for value in array_body(self.nano_source, "kInputPins").split(",")
            if value.strip()
        )
        mega_inputs = tuple(
            int(value.strip())
            for value in array_body(self.mega_source, "kInputPins").split(",")
            if value.strip()
        )
        self.assertEqual(nano_inputs, tuple(range(2, 8)))
        self.assertEqual(mega_inputs, tuple(range(2, 28)))

    def test_paired_protocol_headers_are_identical(self):
        self.assertEqual(
            NANO_PROTOCOL.read_bytes(),
            MEGA_PROTOCOL.read_bytes(),
        )
        for source in (self.nano_source, self.mega_source):
            self.assertIn(
                '#include "auxiliary_protocol_contract.h"',
                source,
            )

    def test_protocol_uses_bounded_frames_and_staged_parsing(self):
        for required_text in (
            "char data_[kMaximumCommandLength + 1]",
            "length_ >= kMaximumCommandLength",
            "discarding_ = true",
            "ParsedCommand parsed;",
            "*result = parsed;",
            "digit > maximum",
            "value > (maximum - digit) / 10UL",
            'literalMatches(text, length, "STOP", 4)',
            'literalMatches(text, length, "STOPWI", 6)',
        ):
            self.assertIn(required_text, self.protocol_source)

    def test_sketches_do_not_use_dynamic_serial_strings_or_autonomous_servo_motion(self):
        for source in (self.nano_source, self.mega_source):
            for forbidden_text in (
                "String ",
                "readStringUntil",
                ".toInt(",
                "gripperBackoff",
                "CURRENT_LIMIT",
                "BACKOFF_INTERVAL",
                "servo0.write",
            ):
                self.assertNotIn(forbidden_text, source)
            self.assertEqual(source.count("readCurrentAmps()"), 2)

            setup = source_region(source, "void setup()", "void loop()")
            self.assertNotIn(".attach(", setup)
            self.assertNotIn(".write(", setup)

            write_servo = source_region(
                source,
                "bool writeServo(",
                "void stopWait()",
            )
            self.assertLess(
                write_servo.index("cli()"),
                write_servo.index(".attach("),
            )
            self.assertLess(
                write_servo.index(".attach("),
                write_servo.index(".write(position)"),
            )
            self.assertLess(
                write_servo.index(".write(position)"),
                write_servo.index("SREG = interruptState"),
            )

    def test_nano_excludes_the_unsupported_seventh_servo(self):
        servo_pins = array_body(self.nano_source, "kServoPins")
        self.assertNotIn("A6", servo_pins)
        self.assertIn("kServoCount = 6", self.nano_source)
        self.assertNotIn("servo6", self.nano_source)

    def test_wait_processing_is_nonblocking_and_stop_restricted(self):
        for source in (self.nano_source, self.mega_source):
            service_wait = source_region(
                source,
                "void serviceWait()",
                "bool beginWait(",
            )
            self.assertNotIn("while (", service_wait)
            self.assertNotIn("delay(", service_wait)
            self.assertIn("digitalRead(waitOperation.pin)", service_wait)
            self.assertIn("ar4_auxiliary::updateWait(", service_wait)

            active_wait = source_region(
                source,
                "const ar4_auxiliary::CommandDisposition disposition",
                "switch (command.kind)",
            )
            self.assertIn(
                "ar4_auxiliary::commandDisposition(",
                active_wait,
            )
            self.assertIn(
                "disposition == ar4_auxiliary::kStopActiveWait",
                active_wait,
            )
            self.assertIn(
                "disposition == ar4_auxiliary::kRejectDuringWait",
                active_wait,
            )
            self.assertIn('Serial.println(F("Error"))', active_wait)

    def test_response_framing_matches_host_ownership(self):
        for source in (self.nano_source, self.mega_source):
            self.assertIn('Serial.print(F("Servo Done"))', source)
            self.assertEqual(source.count('Serial.print(F("Done"))'), 2)
            self.assertIn('Serial.println(F("Nano Stopped"))', source)
            self.assertIn(
                'Serial.println(F("Nano Inactive Stopped"))',
                source,
            )

            handle_frame = source_region(
                source,
                "void handleFrame(",
                "void setup()",
            )
            self.assertEqual(
                handle_frame.count("digitalRead(command.pin)"),
                1,
            )
            write_echo = source_region(
                source,
                "void writeEcho(",
                "void handleFrame(",
            )
            self.assertEqual(write_echo.count("Serial.write('\\n')"), 1)
            self.assertNotIn("Serial.println", write_echo)

    def test_mega_initial_high_outputs_are_latched_before_output_enable(self):
        setup = source_region(
            self.mega_source,
            "void setup()",
            "void loop()",
        )
        self.assertLess(
            setup.index("digitalWrite(pin, HIGH)"),
            setup.index("pinMode(kOutputPins[index], OUTPUT)"),
        )


if __name__ == "__main__":
    unittest.main()
