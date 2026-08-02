# AR4HMI Safety and Verification Boundary

## Software-only development

- `AR4.py` is an executable application entry point, not an importable library. Import attempts fail before GUI construction or saved-controller scheduling; routine analysis and automated tests read or parse the source without executing the file.
- Hardware-free tests may read source, parse syntax, exercise extracted modules, compile firmware without upload, and use mocked transports.
- Simulation, static analysis, successful compilation, and mocked serial traffic do not establish live-arm behavior.
- Machine-specific runtime calibration remains outside version control. `defaults.json` is the tracked fallback calibration profile and contains saved machine parameters that require validation against connected hardware.

## Hardware side effects

Starting `AR4.py` is explicit operator admission for the saved main-controller connection, validated configuration and position synchronization, configured auxiliary connection, auxiliary-board reset, and firmware-defined startup effects. The main-controller sequence sends no motor-drive command. Opening an auxiliary port can reset the board; auxiliary firmware can initialize output pins or servo positions during setup.

After startup, firmware upload, calibration cycles, homing, operator-commanded output changes, and powered motion require a separate operator-approved procedure. Before powered motion, confirm a cleared work envelope and verify the independent physical stop path under power.

Mechanical self-protection is the primary safety objective. Normal motion must fail closed when homing, calibration, encoder state, direction, joint limits, singularity handling, controller state, or response framing is untrusted.

Physical driver microstep settings or measured motion scale must match the active profile before accepting a calibration reference or powered-motion result. Matching host and firmware values alone is insufficient; earlier commissioning exposed a factor-of-two physical driver mismatch despite consistent software settings.

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
