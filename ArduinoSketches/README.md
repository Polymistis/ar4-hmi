# Firmware compilation

Firmware compilation is hardware-free. Do not add `--upload` without the explicit operator authorization, emergency-stop check, cleared work envelope, firmware identity, configuration profile, and procedure required by `AGENTS.md`.

The tracked line-oriented Teensy compatibility source identifies version `6.7.1-ar4hmi.1`, advertises the required `JT_WRIST_CONFIG_V1` host capability, and compiles with Arduino CLI, PJRC Teensy core 1.62.0, and ModbusMaster 2.0.1. Compilation establishes source and toolchain compatibility only; hardware-free fixtures cover selected protocol behavior, while correlated JSON parsing, Cartesian-bound, and emergency-event work remains a later integration unit.

```text
arduino-cli core install teensy:avr@1.62.0 --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli lib install ModbusMaster@2.0.1
arduino-cli compile --fqbn teensy:avr:teensy41 --clean --build-path <temporary-build-directory> ArduinoSketches/AR4_teensy41_sketch_v6.7.1
```

Arduino library discovery gives sketchbook libraries priority over platform libraries. An unrelated sketchbook `SPI` library can shadow the Teensy core implementation and cause missing `SPISettings` errors in `SdFat`. Preserve the sketchbook and select the platform library explicitly:

```text
arduino-cli compile --fqbn teensy:avr:teensy41 --clean --build-path <temporary-build-directory> --library <Arduino15>/packages/teensy/hardware/avr/1.62.0/libraries/SPI ArduinoSketches/AR4_teensy41_sketch_v6.7.1
```

Keep build output outside the tracked source tree.

The dated hardware-free build result is recorded in [`docs/hardware-free-verification-2026-07-19.md`](../docs/hardware-free-verification-2026-07-19.md).
