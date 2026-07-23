# Drive-off Controller Commissioning — 2026-07-22

## Verification boundary

This record covers authorized firmware upload, non-motion serial communication,
HMI connection, hardware emergency-stop availability, and the
already-deenergized drive state. No calibration, homing, motor-motion, or
operator-requested digital, servo, or gripper command was issued from the HMI.
Nano startup attached servo channels 0-6 and wrote the firmware baseline
gripper position. Drive power remained operator-confirmed off during controller
upload, protocol checks, HMI startup, and subsequent hardware-free development.

## Authorization and safety state

- The operator explicitly authorized Teensy and Nano firmware upload, including the Nano old-bootloader fallback.
- The operator confirmed a clear work envelope before controller access.
- The operator confirmed drive power off before upload and reconfirmed that boundary before source work continued.
- A hardware emergency stop was available before any planned movement. Drive power was already off, so no powered-to-deenergized transition or energized-motion test was performed.

## Controller and configuration identity

- Main controller: Teensy 4.1 running tracked source `ArduinoSketches/AR4_teensy41_sketch_v6.7.1`, firmware identity `6.7.1-ar4hmi.1`, and protocol capability `JT_WRIST_CONFIG_V1`.
- Auxiliary controller: Arduino Nano running tracked source `ArduinoSketches/AR4_nano_sketch_v1.5`, uploaded with the ATmega328P old-bootloader profile after the current-bootloader synchronization sequence failed with `resp=0x00`.
- HMI auxiliary-board profile: `Nano`.
- Starting pose: not physically established. The installed encoders require homing, and startup `SP` restores saved controller step-monitor coordinates without moving or establishing an absolute joint reference. Displayed pre-homing coordinates therefore were not accepted as a physical pose.

## Procedure and observations

1. Compiled the tracked Teensy source for `teensy:avr:teensy41` using PJRC core `1.62.0`, PJRC `SPI`, and ModbusMaster `2.0.1`.
2. Uploaded the compiled Teensy image through Teensy Loader with reboot enabled. Arduino CLI returned exit code `0`.
3. Opened the detected Teensy serial connection at 9600 baud with drive power off, sent `HO\n`, consumed the terminal line, and closed the connection.
4. Observed the controller identity response:

   ```json
   {"DriverModel":"Unset","FirmwareVersion":"6.7.1-ar4hmi.1","RobotModel":"Unset","RobotVersion":"Unset","SerialNumber":"Unset","AssetTag":"Unset","ProtocolCapabilities":["JT_WRIST_CONFIG_V1"]}
   ```

5. Compiled the tracked Nano source with Arduino AVR core `1.8.8` and Servo `1.3.0` for the supported bootloader profiles.
6. Attempted the current-bootloader upload and observed the complete synchronization retry failure with `resp=0x00`. Uploaded with the old-bootloader profile and upload verification enabled; Arduino CLI returned exit code `0`.
7. Opened the Nano serial connection at 9600 baud with drive power off, allowed reset and setup to complete, discarded startup input, sent `TMAR4HMI_VERIFY\n`, consumed the terminal line, and closed the connection. The observed response was `AR4HMI_VERIFY`. Nano setup attached `servo0` through `servo6` on `A0` through `A6` and called `servo0.write(20)` as a reset/startup side effect; no HMI gripper command was sent.
8. Launched `AR4.py` in the isolated CPython 3.12 environment with the detected main controller, auxiliary controller, and `Nano` board profile. Startup reported completed main-controller communication, completed the configured auxiliary path without the auxiliary-open warning, and displayed `SYSTEM READY`. Opening the Nano serial port can reset the board and repeat the servo attachments and baseline gripper write; that repeated physical write was not independently observed.
9. Engaged the hardware emergency stop while drive power was already off. The drive system remained unpowered; no powered interruption behavior was exercised.

Nano firmware also calls `gripperBackoff()` from the main loop and can write
`servo0` autonomously when measured current exceeds `CURRENT_LIMIT`. No
activation of that path was observed during this procedure.

## Result

Drive-off firmware upload and non-motion communication passed for the
identified main and auxiliary controllers. HMI startup reached `SYSTEM READY`.
Hardware emergency-stop availability was recorded without claiming powered
interruption behavior. Nano reset-driven servo attachment and baseline gripper
positioning remain startup side effects rather than operator commands.
Calibration, homing, encoder-reference establishment, motor behavior, motion
speed, coordinated motion, repeatability, payload behavior, and physical pose
remain unverified.
