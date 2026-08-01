# Drive-off Teensy Firmware Deployment — 2026-07-29

## Verification boundary

This record covers the operator-authorized Teensy 4.1 firmware upload and
read-only controller identity and home-reference queries. Motor power was
operator-confirmed off at authorization. No HMI launch, Nano connection,
calibration, homing, motion command, forced-position write, or
operator-requested auxiliary or motor command formed part of the procedure.
Firmware restart still executed `setup()`, configured the J1-J9 step and
direction pins as outputs, and drove the step pins high. Drive power was off
during that controller-side transition.

The standing project safety context records a cleared work envelope and an
available hardware emergency stop. Neither condition was re-exercised during
this drive-off procedure, and no powered emergency-stop behavior is inferred.

## Manual verification

- Date: `2026-07-29`.
- Operator confirmation: motor power was off when the upload was authorized.
  No power-state change was requested or reported during the procedure. After
  reviewing the query result, the operator acknowledged the invalid J1-J3
  reference state as known and authorized closure of this record.
- Main controller: Teensy 4.1 on `COM13`, Windows device identity
  `USB\VID_16C0&PID_0483&MI_00`.
- Controller hardware identity reported by firmware: `1705B6`.
- Tracked source:
  `ArduinoSketches/AR4_teensy41_sketch_v6.7.1` at Git tree
  `05dca406d98c4c8e0d246cd3517e35bf2dc7cc07`. The tree is reachable from
  the uniquely titled `test: bind shutdown firmware protocol routing` commit,
  dated `2026-07-28`, in both the original and publication histories.
- Self-reported firmware identity: `6.7.1-ar4hmi.5`.
- Auxiliary controller: not connected or queried.
- HMI configuration profile: not loaded.
- Starting pose: not observed; no pose change was commanded. The drive system
  was unpowered, and the `HOME_REFERENCE_V2` response reported every J1-J3
  reference invalid.

Procedure and observed results:

1. Confirmed that the tracked Teensy source had no worktree diff and identified
   `COM13` as the Teensy serial device.
2. Started a clean `teensy:avr:teensy41` build with PJRC core `1.62.0` and
   ModbusMaster `2.0.1`. That build stopped before upload because the generic
   sketchbook `SPI` library shadowed the PJRC Teensy library and lacked the
   required `SPISettings` type.
3. Repeated the clean build with the PJRC `SPI` library selected explicitly.
   Build metadata resolved `SPI`, `SD`, and SdFat from the Teensy `1.62.0`
   platform and ModbusMaster from the installed `2.0.1` library. Arduino CLI
   returned exit code `0`.
4. Uploaded the compiled build to `COM13` through Arduino CLI and Teensy
   Loader. Arduino CLI returned exit code `0`, and `COM13` re-enumerated with
   the same Windows device identity.
5. Opened `COM13` at 9600 baud, sent the read-only `HO\n` query, parsed the
   bounded JSON response, and required firmware version
   `6.7.1-ar4hmi.5` plus a controller hardware identity. The captured
   capability set was checked for the host-required
   `JT_WRIST_CONFIG_V1`, `GCODE_DIRECTORY_FRAMING_V1`,
   `GCODE_DELETE_IDENTITY_V1`, and `GCODE_WRITE_IDENTITY_V1` entries, preferred
   `HOME_REFERENCE_V2`, and M4A7 `JOINT_TELEMETRY_V1`.
6. After capability validation, sent the read-only `H2\n` query and validated
   the complete J1-J3 home-reference frame as `A0B0C0D0E0F0`.
7. Closed the serial port and the Teensy Loader helper. No upload or HMI
   process remained active.

The procedure observed a controller self-report of firmware
`6.7.1-ar4hmi.5`, hardware identity `1705B6`, the required capability
advertisements, and invalid J1-J3 home references. No powered motion,
telemetry cadence, encoder accuracy, or physical output behavior was observed.

## Observed protocol responses

`HO` returned:

```json
{"ControllerHardwareId":"1705B6","DriverModel":"Unset","FirmwareVersion":"6.7.1-ar4hmi.5","RobotModel":"Unset","RobotVersion":"Unset","SerialNumber":"Unset","AssetTag":"Unset","ProtocolCapabilities":["JT_WRIST_CONFIG_V1","GCODE_DIRECTORY_FRAMING_V1","GCODE_DELETE_IDENTITY_V1","GCODE_WRITE_IDENTITY_V1","HOME_REFERENCE_V1","HOME_REFERENCE_V2","JOINT_TELEMETRY_V1"]}
```

`H2` returned:

```text
A0B0C0D0E0F0
```

The `H2` frame reports invalid J1, J2, and J3 home references with zero
placeholder positions. Shutdown Position therefore remains unavailable until
fresh authorized homing establishes valid J2 and J3 references under the
active controller frame.

## Result

The drive-off upload command completed successfully, and read-only
communication passed with Teensy 4.1 hardware identity `1705B6`. After
re-enumeration, the controller advertised expected version
`6.7.1-ar4hmi.5` and the required protocol capabilities. Those observations do
not cryptographically bind the running binary to the cited Git source tree or a
particular local build artifact. No motor, telemetry-cadence, encoder-accuracy,
USB-load, terminal-priority, pulse-timing, homing, or named-position behavior
was exercised. M4A7 powered telemetry verification therefore remains pending.

## Publication-history amendment — 2026-07-31

The original record identified development-history commit
`d845471ce4767c0ec75a7fdf253bd1073c658b7b`. Publication reparented the
unchanged commit sequence onto the official upstream fork point, producing
publication-history commit `63cef1d1119bf8f2c7af6a45aa26898a36c6140a` for
the same uniquely titled change. Both commits contain firmware-directory tree
`05dca406d98c4c8e0d246cd3517e35bf2dc7cc07`, now used above as the durable
source identifier. No hardware observation or procedure was changed by this
provenance amendment.
