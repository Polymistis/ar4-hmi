# Firmware compilation

The commands below compile without uploading. Do not add `--upload`. Any firmware upload requires a separate operator-approved procedure under [`SAFETY.md`](../SAFETY.md), and the resulting record must satisfy the complete live-verification field contract there. Powered testing also requires a cleared workspace, verification of the independent physical stop path, and confirmation of physical driver microsteps or measured motion scale.

The tracked line-oriented Teensy compatibility source identifies version
`6.7.1-ar4hmi.10`, advertises the required `JT_WRIST_CONFIG_V1`,
`GCODE_DIRECTORY_FRAMING_V1`, `GCODE_DELETE_IDENTITY_V1`,
`GCODE_WRITE_IDENTITY_V1`, `ESTOP_ADMISSION_V1`, and
`CALIBRATION_SWITCH_POLARITY_V1` host capabilities plus the optional legacy
`HOME_REFERENCE_V1`, preferred `HOME_REFERENCE_V2`
parking-reference, and `JOINT_TELEMETRY_V1` request-scoped J1-J6 encoder
telemetry contracts, and compiles with Arduino CLI,
PJRC Teensy core 1.62.0, bundled SdFat 2.1.2, and ModbusMaster 2.0.1. `HO`
includes the fixed-width controller hardware identity used to bind storage
requests to the connected Teensy. `RB` performs the Teensy 4.1 target reset
without consulting mutable controller identity fields. Compilation
establishes source and toolchain compatibility only; hardware-free fixtures
cover selected protocol behavior, while correlated JSON parsing and
Cartesian-bound work remain later integration units.

`CALIBRATION_SWITCH_POLARITY_V1` extends `UP` with a J1-low-bit mask describing
the active state of each J1-J9 calibration switch. Existing and migrated
profiles send the all-`HIGH` mask. Calibration search, bounded release, and the
`TL` diagnostic classify each switch against the configured active state.

`JOINT_TELEMETRY_V1` accepts the optional `T1` suffix only on `RJ` commands.
The controller targets ten samples per second, formats signed millidegrees in a
fixed ASCII frame, and drops a sample unless USB capacity remains for both the
telemetry frame and a reserved terminal response. J7-J9 remain host estimates
because the tracked controller has no matching encoder sources. A
loop-scoped response owner brackets every ordinary, admission, and telemetry
terminal writer. The E-stop interrupt records assertion state and pending
output without writing USB serial data. Main-loop code emits pending `EB` only
after the current terminal frame or at an otherwise empty loop boundary.
Telemetry-enabled `RJ` retains its specialized terminal decision across the
drive; an E-stop latches immediately, then terminal framing follows committed
step progress and encoder reconciliation. `EB` identifies the asynchronous
physical-stop event. A stop deferred after telemetry terminal selection
becomes an admission block and emits `EB` immediately after the selected
terminal frame. Command admission is checked atomically before parsing and
again after side-effect-free opcode extraction. A blocked command reserves the
correlated `EA` response against pending `EB` publication. Admission and
loop-response ownership retire together with interrupts disabled, then a
released stop clears only when no newer interrupt generation was recorded. An
asserted or newly reasserted stop continues rejecting commands.
Calibration command `LL` emits `ER` after every failed motion stage. When the
stop interrupt occurs during an owned calibration response, the host consumes
the bounded `ER` terminal and `EB` event pair.
No-upload compilation does not establish encoder accuracy or pulse-timing
behavior.

`GCODE_DIRECTORY_FRAMING_V1` reserves comma as the directory separator,
requires every `.txt` entry to have a reversible controller-command stem, and
caps the complete directory payload at 4096 bytes. Incompatible entries,
directory-buffer allocation failure, directory read failure, and aggregate
overflow return an `EG:` response before any partial listing. Clean
end-of-directory is distinct from an iteration error.
Directory-entry extraction treats the SdFat `getName` result only as a
zero/nonzero read status and derives the filename length from the bounded
terminated buffer, preserving compatibility across boolean and length return
contracts.
`GCODE_DELETE_IDENTITY_V1` prefixes every successful `RG` payload with
`MID:<32-uppercase-hex-CID>|` and requires
`DGMi<same-CID>Fn<filename>`. Directory and delete-lookup traversal revalidate
the CID before a successful payload or absence response, and deletion checks
the CID immediately before and after removal. Delete lookup distinguishes
confirmed absence from directory lookup failure. `RG` terminates with the
identity-prefixed directory payload or a printable `EG:` detail. `DG` returns
definitive `P`, `F`, or `ER` responses; a printable `EG:` detail leaves
host-side deletion reconciliation pending.

`GCODE_WRITE_IDENTITY_V1` requires `WC` and `WG` targets in
`Mi<same-CID>Fn<filename>` form. Every write checks the CID before opening the
file and after closing the flushed file. A media mismatch returns `EG:` without
a position-success frame.

```text
arduino-cli core install teensy:avr@1.62.0 --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli lib install ModbusMaster@2.0.1
arduino-cli compile --fqbn teensy:avr:teensy41 --clean --build-path <temporary-build-directory> ArduinoSketches/AR4_teensy41_sketch_v6.7.1
```

The tracked Nano and Mega compatibility sketches use Arduino AVR core 1.8.8
and Servo 1.3.0. Both sketches enforce complete fixed-buffer commands before
mutation. Nano permits servo channels 0-5, inputs 2-7, and outputs 8-13; Mega
permits servo channels 0-6, inputs 2-27, and outputs 28-53. Servo output remains
detached through startup. Initial attachment keeps AVR interrupts masked until
the admitted `SV` target replaces the Servo library default. `TG` reports
current without autonomous servo correction. `WI` remains nonblocking, and
requires a positive timeout. Each polling pass samples the input before
classifying deadline expiry, and only exact `STOP` or `STOPWI` frames interrupt
an active wait.

Arduino's sketch builder does not expose a header above the selected sketch
directory. Each auxiliary sketch therefore carries a byte-identical
`auxiliary_protocol_contract.h`; the source-contract test rejects divergence.

```text
arduino-cli core install arduino:avr@1.8.8
arduino-cli lib install Servo@1.3.0
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old --clean --build-path <temporary-nano-build-directory> ArduinoSketches/AR4_nano_sketch_v1.5
arduino-cli compile --fqbn arduino:avr:mega --clean --build-path <temporary-mega-build-directory> ArduinoSketches/AR4_mega_sketch_v1.5
```

Arduino library discovery gives sketchbook libraries priority over platform libraries. An unrelated sketchbook `SPI` library can shadow the Teensy core implementation and cause missing `SPISettings` errors in `SdFat`. Preserve the sketchbook and select the platform library explicitly:

```text
arduino-cli compile --fqbn teensy:avr:teensy41 --clean --build-path <temporary-build-directory> --library <Arduino15>/packages/teensy/hardware/avr/1.62.0/libraries/SPI ArduinoSketches/AR4_teensy41_sketch_v6.7.1
```

Keep build output outside the tracked source tree.

Python test fixtures use an external temporary parent. Set
`AR4_TEST_TEMP_DIRECTORY` when the operating-system default temporary directory
is unavailable; the configured directory must already exist outside the source
tree.

`tests/test_teensy_firmware_compile.py` runs the same no-upload compilation
when `AR4_ARDUINO_CLI`, `AR4_TEENSY_BUILD_DIRECTORY`, and
`AR4_TEENSY_SPI_LIBRARY` identify the selected executable, an external
temporary build parent, and the Teensy 1.62.0 platform SPI library. The test
also parses verbose compiler dependency reporting, requires the selected SPI
and SdFat folders under the Teensy 1.62.0 platform, and verifies PJRC Teensy
core 1.62.0 and ModbusMaster 2.0.1. A kill-on-close Windows Job Object or POSIX
process group owns the compiler tree, and the timeout path waits for verified
tree settlement before temporary build cleanup.

`tests/test_auxiliary_firmware_compile.py` runs both AVR no-upload compilations
when `AR4_ARDUINO_CLI` and `AR4_AUXILIARY_BUILD_DIRECTORY` identify the
selected executable and an existing external temporary build parent. The test
requires Arduino AVR core 1.8.8 and Servo 1.3.0 in compiler dependency output.

The dated hardware-free build record, including the current
`6.7.1-ar4hmi.10` result, is recorded in
[`docs/hardware-free-verification-2026-07-19.md`](../docs/hardware-free-verification-2026-07-19.md).
