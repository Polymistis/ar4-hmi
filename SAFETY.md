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
- `Load AR4-MK5 Switch Polarity` stages only the J1-J6 calibration-switch
  selectors. Selection performs no migration, save, apply, native update,
  controller command, or serial access and preserves every other editable
  field. `SaveAndApplyCalibration` remains the separate validation,
  controller-synchronization, and persistence admission.

## Hardware side effects

Starting `AR4.py` is explicit operator admission for automatic connection only to previously enrolled main and auxiliary controllers resolved unambiguously from passive USB identity, validated configuration and position synchronization, auxiliary-board reset, and firmware-defined startup effects. Explicit manual port selection is separate admission to open the selected controller and run the same role-validation and startup boundary. The main-controller sequence sends no motor-drive command. Opening an auxiliary port, automatically or manually, can reset the board. The tracked Mega firmware preloads pins 28-35 high before configuring output pins 28-53 as outputs; the tracked Nano firmware configures output pins 8-13 without an explicit startup write. Both firmware profiles leave servos detached until a validated `servo` request supplies a target.

Passing an already-open serial handle to `ARRobot.connect` or `ARAuxiliary.connect` transfers close ownership to the automation facade at call entry. `ARRobot.connect` performs the normal main-controller startup contract; `ARAuxiliary.connect` performs strict controller-role and board-profile validation. Each side-effecting facade call is separate operator admission for the named command only. Physical output or motion still requires an operator-approved live procedure.

`ARRobot.move_joints` provides bounded synchronous finite main-controller joint
motion and requires a separate operator-approved live procedure for every
invocation. A completed terminal returns `JsonMainJointMotionResult`. The facade
always sends `telemetry_enabled=False` and exposes neither a telemetry switch nor
a finite-motion software stop. The request `timeout` bounds host waiting only;
expiry quarantines and closes the host session, never cancels controller motion,
and leaves robot position unconfirmed because motion can continue. No software
command can stop finite joint motion under this contract, so a verified
independent physical stop and cleared work envelope remain mandatory. A
schema-validated `cancelled/emergency_stop` terminal publishes
`JsonMainControllerPhysicalStop` with source `emergency_stop_terminal` before
terminal acknowledgement, and `JsonMainControllerPhysicalStopError.position`
retains the parsed `JsonMainPositionResult` from the terminal.

`ARRobot.calibrate` provides bounded synchronous selected-axis calibration.
Every live invocation requires a separate operator-approved procedure. One call
sends one validated `calibrate` request containing the exact selected axes
and offsets; no delivered stage alias or implicit stage chaining applies. A
completed terminal returns `JsonMainCalibrationResult`. The request `timeout`
bounds host waiting only; expiry quarantines and closes the host session, never
cancels controller calibration, and leaves robot position unconfirmed because
calibration can continue. The facade exposes no calibration stop command, so a
verified independent physical stop and cleared work envelope remain mandatory.
A schema-validated `cancelled/emergency_stop` terminal
publishes `JsonMainControllerPhysicalStop` with source
`emergency_stop_terminal` before terminal acknowledgement, and
`JsonMainControllerPhysicalStopError.position` retains the parsed
`JsonMainPositionResult` from the terminal.

`ARRobot.move_cartesian`, `ARRobot.move_linear`, `ARRobot.move_vision`,
`ARRobot.jog_tool`, `ARRobot.move_arc`, `ARRobot.move_circle`, and
`ARRobot.move_spline` provide bounded synchronous finite Cartesian and tool
motion. Cartesian-family methods return
`JsonMainCartesianMotionResult`; `ARRobot.jog_tool` returns
`JsonMainToolJogResult`. Facade-generated Cartesian motion always transmits
`telemetry_enabled=False`. `ARRobot.move_linear` additionally fixes
`rounding_millimeters=0.0` and `disable_wrist_rotation=False`. Arc motion starts
from the confirmed current pose, circle motion requires prior arrival at the
declared starting target and inserts no preliminary move, and spline motion is
one atomic one-through-six-segment request. A keyword-only host `timeout` bounds
waiting only; expiry or transport/protocol uncertainty closes the session
without cancelling controller motion or confirming a final pose. No facade
finite-motion method provides a software stop. A schema-validated
`cancelled/emergency_stop` terminal publishes `JsonMainControllerPhysicalStop`
before terminal acknowledgement, and the raised
`JsonMainControllerPhysicalStopError` retains the typed
`JsonMainPositionResult`. Every live invocation requires a separate
operator-approved procedure, a cleared work envelope, and a verified
independent physical stop.

`ARRobot` provides typed scalar Modbus reads, writes, and firmware-owned waits
through the synchronized main-controller session. Firmware-owned waits are
synchronous; callers must select a bounded `timeout_seconds` and call only from
a thread allowed to block until terminal settlement or host timeout. Every live
Modbus invocation requires a separate operator-approved procedure. Scalar reads
always transmit the fixed `count=1` contract. Each wait derives the host deadline
from the requested firmware `timeout_seconds` plus two
`JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS` intervals and returns a
`JsonScalarResult` only after verifying that the completed value equals the
requested `expected` value. A read or wait host timeout closes the session
without inventing a result. An ordinary rejected write terminal leaves the
synchronized session reusable. A `failed` write terminal or host uncertainty
leaves the physical write outcome indeterminate, closes the session fail-closed,
and is never retried. A schema-validated `cancelled/emergency_stop` Modbus
terminal publishes an unpositioned `JsonMainControllerPhysicalStop` with source
`emergency_stop_terminal` before terminal acknowledgement; no later Modbus
write is permitted on that session.

`ARAuxiliary.wait_input` provides a synchronous firmware-owned Nano or Mega
input wait through the role-validated auxiliary session. Call only from a
thread allowed to block until terminal settlement or host timeout. The
board-profile pin, Boolean target state, and bounded integer `timeout_seconds`
are validated before deadline arithmetic or transmission. The host deadline
adds two `JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS` intervals to the firmware
timeout,
and a completed wait returns `None`. A settled controller `failed/timeout`
terminal leaves the synchronized session reusable. Host deadline expiry or
transport or protocol uncertainty closes the session without claiming an input
match. The public synchronous facade exposes no auxiliary stop or concurrent
cancellation call; HMI and protocol stop behavior remains separately owned.

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
