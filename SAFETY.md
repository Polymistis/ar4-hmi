# AR4HMI Safety and Verification Boundary

## Software-only development

- `AR4.py` is a direct-execution application entry point with no supported import contract. Import attempts fail before GUI construction or saved-controller scheduling. Automated tests must not execute the file. Direct execution automatically schedules the saved-controller connection.
- Hardware-free tests may read source, parse syntax, exercise extracted modules, compile firmware without upload, and use mocked transports.
- Simulation, static analysis, successful compilation, and mocked serial traffic do not establish live-arm behavior.
- Machine-specific runtime calibration remains outside version control. `defaults.json` is the tracked fallback calibration profile and contains saved machine parameters that require validation against connected hardware.

## Hardware side effects

Starting `AR4.py` is explicit operator admission for the saved main-controller connection, validated configuration and position synchronization, configured auxiliary connection, auxiliary-board reset, and firmware-defined startup effects. The main-controller sequence sends no motor-drive command. Opening an auxiliary port can reset the board. The tracked Mega firmware preloads pins 28-35 high before configuring output pins 28-53 as outputs; the tracked Nano firmware configures output pins 8-13 without an explicit startup write. Both firmware builds leave servos detached until an `SV` command supplies a target.

After startup, firmware upload, calibration cycles, homing, operator-commanded output changes, and powered motion require a separate operator-approved procedure. Before powered motion, confirm a cleared work envelope and verify the independent physical stop path under power.

Mechanical self-protection is the primary safety objective. Normal motion must fail closed when homing, calibration, encoder state, direction, joint limits, singularity handling, controller state, or response framing is untrusted.

Physical driver microstep settings and observed motion scale must match the
active profile before accepting a calibration reference or powered-motion
result. Matching host and firmware values alone is insufficient evidence of
physical scale.

## Live verification record

Every live verification record must include:

- date;
- controller and firmware identity;
- configuration profile;
- starting state and pose;
- exact executed procedure and commanded values;
- conservative limits and abort conditions;
- observed results and deviations;
- operator confirmation.

Protocol changes require matching host and firmware updates. Joint limits, stop handling, encoder checks, calibration safeguards, and motion error handling must not be weakened without an explicit requirement and controlled validation plan.
