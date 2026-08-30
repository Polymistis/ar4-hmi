# AR4HMI Safety and Verification Boundary

When applicable safety requirements conflict, the stricter requirement governs
until an explicit requirement and controlled validation plan supersede the
conflict.

## Software-only development

- `AR4.py` is a direct-execution application entry point with no supported import contract. Automated tests must not import or execute `AR4.py`. Direct execution automatically connects only a previously enrolled controller resolved unambiguously from passive USB identity; missing, unenrolled, ambiguous, and serial-number-less devices remain disconnected until explicit selection.
- Hardware-free tests may read source, parse syntax, exercise extracted modules, compile firmware without upload, and use mocked transports.
- Automated checks must not open serial ports or issue controller commands.
- Simulation, static analysis, successful compilation, and mocked serial traffic do not establish live-arm behavior.
- Machine-specific runtime calibration remains outside version control. `defaults.json` is the tracked fallback calibration profile and contains saved machine parameters that require validation against connected hardware.

## Hardware side effects

Starting `AR4.py` is explicit operator admission for automatic connection only to previously enrolled main and auxiliary controllers resolved unambiguously from passive USB identity, validated configuration and position synchronization, auxiliary-board reset, and firmware-defined startup effects. Explicit manual port selection is separate admission to open the selected controller and run the same role-validation and startup boundary. The main-controller sequence sends no motor-drive command. Opening an auxiliary port, automatically or manually, can reset the board. The tracked Mega firmware preloads pins 28-35 high before configuring output pins 28-53 as outputs; the tracked Nano firmware configures output pins 8-13 without an explicit startup write. Both firmware profiles leave servos detached until a validated `servo` request supplies a target.

After startup, firmware upload, calibration cycles, homing, operator-commanded output changes, and powered motion require a separate operator-approved procedure. Before powered motion, confirm a cleared work envelope and verify the independent physical stop path under power.

Mechanical self-protection is the primary safety objective. Normal motion must fail closed when homing, calibration, encoder state, direction, joint limits, singularity handling, controller state, or response framing is untrusted.

Recovery and homing paths require explicit bounded semantics. Before broader
motion trials, hardware-validation plans must cover overtravel, self-collision,
stall or encoder-collision response, cable-wrap limits, and direction-sign
correctness. Joint limits, calibration units, saved-pose handling, encoder
modes, and stop behavior require explicit tests before hardware verification.

J1/J2/J3 homing verification must confirm switch release within the lesser of
the configured 10-unit bound and full configured axis travel. Home-reference
availability begins only after successful centered motion.

Before broader arc or circle operation, controlled validation must establish
response deadlines covering the complete commanded traversal. Automatic-wrist
circle validation must establish wrist-branch continuity and repeatability
across the preliminary start move and atomic circle request.

Physical driver microstep settings and observed motion scale must match the
active profile before accepting a calibration reference or powered-motion
result. Matching host and firmware values alone is insufficient evidence of
physical scale.

## Live verification record

Every live verification record must include:

- date;
- deployed host, controller, and firmware identity;
- configuration profile;
- starting state and pose;
- exact executed procedure, commanded values, and expected physical path;
- conservative limits, abort conditions, and recovery steps;
- physical-stop readiness and current work-envelope confirmation;
- observed results and deviations;
- operator confirmation and the next decision supported by the evidence.

Exploratory observations with missing required fields do not qualify as live
verification.
Deviations become tracked requirements or defects before broader operation.

Protocol changes require matching host and firmware updates. Joint limits, stop handling, encoder checks, calibration safeguards, and motion error handling must not be weakened without an explicit requirement and controlled validation plan.
