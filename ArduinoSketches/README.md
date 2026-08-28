# Firmware compilation

All tracked controller firmware uses the correlated JSON v1 protocol. Legacy
line commands, protocol sniffing, JSON-to-legacy handoff, and fallback are not
supported.

The matched firmware identities are:

- Teensy 4.1 main controller: `6.7.1-ar4hmi.38`
- Nano auxiliary controller: `2.0`
- Mega auxiliary controller: `2.0`

Compilation never counts as powered-arm verification. Do not add `--upload`
to any command below. Firmware upload, controller writes, output activation,
homing, calibration, and motion remain governed by
[`SAFETY.md`](../SAFETY.md).

## Main controller

The Teensy accepts LF-terminated JSON request envelopes and emits correlated
response, event, and telemetry envelopes. `hello` advertises exactly the
command manifest defined by
[`ARrobots/protocol/catalog.py`](../ARrobots/protocol/catalog.py) and the
session capabilities `JSON_PROTOCOL_V1`,
`REQUEST_CORRELATION_V1`, and `EVENT_STREAM_V1`.

The firmware owns strict framing, request validation, session correlation,
configuration synchronization, motion exclusion, response serialization, and
emergency-stop ordering. The interrupt path latches stop state without writing
serial bytes. Main-loop arbitration finishes an owned response before
publishing the JSON emergency-stop event.

Main commands cover identity and state, configuration, diagnostics,
calibration, joint/Cartesian/linear/vision/arc/circle/spline motion, tool and
live jog, stop, Modbus reads/writes/waits, and SD program
delete/list/write/playback. Arc, circle, and spline requests are atomic,
terminal-only operations with complete motion preflight before output. Exact
request and result shapes are documented in
[`docs/json-protocol-v1.md`](../docs/json-protocol-v1.md) and enforced by the
paired host and firmware schemas.

Dependencies:

- PJRC Teensy core `1.62.0`
- bundled SdFat `2.1.2`
- ModbusMaster `2.0.1`
- ArduinoJson `7.4.3`

```text
arduino-cli core install teensy:avr@1.62.0 --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli lib install ModbusMaster@2.0.1
arduino-cli lib install ArduinoJson@7.4.3
arduino-cli compile --fqbn teensy:avr:teensy41 --clean --build-path <temporary-build-directory> --library <ArduinoJson-library-root> ArduinoSketches/AR4_teensy41_sketch_v6.7.1
```

Arduino library discovery gives sketchbook libraries priority over platform
libraries. An unrelated sketchbook `SPI` library can shadow the Teensy core
implementation. Select the platform library explicitly when required:

```text
arduino-cli compile --fqbn teensy:avr:teensy41 --clean --build-path <temporary-build-directory> --library <Arduino15>/packages/teensy/hardware/avr/1.62.0/libraries/SPI --library <ArduinoJson-library-root> ArduinoSketches/AR4_teensy41_sketch_v6.7.1
```

## Auxiliary controllers

Nano and Mega accept the same bounded JSON v1 envelopes and expose
`hello`, `servo`, `input_read`, `set_output`, `wait_input`,
`test_gripper_amps`, `stop`, and `gripper_detach`.

- Nano: servo channels 0-5, input pins 2-7, output pins 8-13
- Mega: servo channels 0-6, input pins 2-27, output pins 28-53

Servo outputs remain detached through startup. Mega pins 28-35 preload HIGH
before output-mode activation. Input waits are nonblocking and rollover-safe.
An admitted stop completes before a retained wait receives
`cancelled/stop_requested`.
`gripper_detach {}` idempotently detaches servo channel 0. Orderly HMI shutdown
submits one best-effort correlated detach after auxiliary stop and activity
ownership settle; an admitted request with uncertain disposition is not
retried.

Each AVR sketch carries a byte-identical
`auxiliary_protocol_contract.h` because Arduino sketch compilation cannot
include a shared header above the selected sketch directory.

Dependencies:

- Arduino AVR core `1.8.8`
- Servo `1.3.0`

```text
arduino-cli core install arduino:avr@1.8.8
arduino-cli lib install Servo@1.3.0
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old --clean --build-path <temporary-nano-build-directory> ArduinoSketches/AR4_nano_sketch_v1.5
arduino-cli compile --fqbn arduino:avr:mega --clean --build-path <temporary-mega-build-directory> ArduinoSketches/AR4_mega_sketch_v1.5
```

Keep build output outside the tracked source tree. `AR4_ARDUINO_CLI`,
`AR4_TEENSY_BUILD_DIRECTORY`, `AR4_TEENSY_SPI_LIBRARY`,
`AR4_ARDUINOJSON_LIBRARY`, and `AR4_AUXILIARY_BUILD_DIRECTORY` configure
the optional no-upload compile checks.
