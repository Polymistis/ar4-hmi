# Python automation API

`ARrobots.automation` is the sole supported Python scripting facade for the
JSON-only controllers. The facade is synchronous, typed, and import-safe; no
`AR4.py` import is required. This guide describes the complete public surface.

## Session ownership

`ARRobot.connect` and `ARAuxiliary.connect` accept an already-open serial
handle. Close ownership transfers to the facade at call entry, including
validation or startup failure. Application code must not reuse or close the
handle after transfer. A successful session must be closed explicitly or used
as a context manager. `close_timeout` bounds waiting for active coordinator
operations before shutdown; the owned `serial_port.close()` call is not wrapped
in a deadline and must return.

Main startup requires an exact
`JsonMainControllerStartupConfiguration`, applies the synchronized JSON startup
contract, and validates controller identity. Auxiliary startup validates the
JSON role and the requested `"nano"` or `"mega"` board profile. Neither facade
discovers or opens serial ports.

Only one call can own a session at a time. A concurrent call or `close()` raises
`AutomationStateError` immediately; admission never waits for another facade
call. Input validation runs before serial transmission. No facade call retries
a command.

## Public signatures and results

Names below use result types exported by `ARrobots.protocol`. Every parameter
shown is required unless a default appears.

```text
ARRobot.connect(
    serial_port, *, configuration, physical_stop_callback, request_timeout,
    cancellation_boundary=None, close_timeout=JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
) -> ARRobot
ARRobot.identity -> JsonMainHelloResult
ARRobot.startup_position -> JsonMainPositionResult
ARRobot.startup_home_reference -> JsonMainHomeReferenceResult
ARRobot.refresh_position(*, timeout) -> JsonMainPositionResult
ARRobot.refresh_home_reference(*, timeout) -> JsonMainHomeReferenceResult
ARRobot.read_limit_switches(*, timeout) -> tuple[bool, bool, bool, bool, bool, bool]
ARRobot.read_encoders(*, timeout) -> tuple[int, int, int, int, int, int]
ARRobot.modbus_read_holding_register(*, slave_id, address, timeout) -> int
ARRobot.modbus_read_coil(*, slave_id, address, timeout) -> int
ARRobot.modbus_read_discrete_input(*, slave_id, address, timeout) -> int
ARRobot.modbus_read_input_register(*, slave_id, address, timeout) -> int
ARRobot.modbus_write_coil(*, slave_id, address, value, timeout) -> None
ARRobot.modbus_write_register(*, slave_id, address, value, timeout) -> None
ARRobot.wait_modbus_coil(*, slave_id, address, expected, timeout_seconds) -> JsonScalarResult
ARRobot.wait_modbus_discrete_input(*, slave_id, address, expected, timeout_seconds) -> JsonScalarResult
ARRobot.wait_modbus_holding_register(*, slave_id, address, expected, timeout_seconds) -> JsonScalarResult
ARRobot.calibrate(*, axes, offsets, timeout) -> JsonMainCalibrationResult
ARRobot.move_joints(
    *, robot_joints_degrees, external_axes_units, speed_mode, speed_value, acceleration_percent,
    deceleration_percent, ramp_percent, wrist_configuration, loop_modes, timeout,
) -> JsonMainJointMotionResult
ARRobot.move_cartesian(
    *, translation_millimeters, orientation_degrees, external_axes_units, speed_mode, speed_value,
    acceleration_percent, deceleration_percent, ramp_percent, wrist_configuration, loop_modes, timeout,
) -> JsonMainCartesianMotionResult
ARRobot.move_linear(
    *, translation_millimeters, orientation_degrees, external_axes_units, speed_mode, speed_value,
    acceleration_percent, deceleration_percent, ramp_percent, wrist_configuration, loop_modes, timeout,
) -> JsonMainCartesianMotionResult
ARRobot.move_vision(
    *, translation_millimeters, orientation_degrees, external_axes_units, speed_mode, speed_value,
    acceleration_percent, deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
    vision_rotation_degrees, timeout,
) -> JsonMainCartesianMotionResult
ARRobot.jog_tool(
    *, axis, direction, distance, speed_mode, speed_value, acceleration_percent,
    deceleration_percent, ramp_percent, wrist_configuration, loop_modes, timeout,
) -> JsonMainToolJogResult
ARRobot.move_arc(
    *, translation_millimeters, orientation_degrees, external_axes_units,
    speed_mode, speed_value, acceleration_percent, deceleration_percent,
    ramp_percent, wrist_configuration, loop_modes,
    midpoint_translation_millimeters, timeout,
) -> JsonMainCartesianMotionResult
ARRobot.move_circle(
    *, translation_millimeters, orientation_degrees, external_axes_units,
    speed_mode, speed_value, acceleration_percent, deceleration_percent,
    ramp_percent, wrist_configuration, loop_modes,
    center_translation_millimeters, plane_translation_millimeters, timeout,
) -> JsonMainCartesianMotionResult
ARRobot.move_spline(*, segments, timeout) -> JsonMainCartesianMotionResult
ARRobot.close() -> None

ARAuxiliary.connect(
    serial_port, *, expected_board, request_timeout,
    close_timeout=JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
) -> ARAuxiliary
ARAuxiliary.identity -> JsonAuxiliaryHelloResult
ARAuxiliary.board_profile -> str
ARAuxiliary.read_input(pin, *, timeout) -> JsonAuxiliaryInputResult
ARAuxiliary.set_servo(channel, position, *, timeout) -> None
ARAuxiliary.set_output(pin, state, *, timeout) -> None
ARAuxiliary.wait_input(pin, state, *, timeout_seconds) -> None
ARAuxiliary.test_gripper_amps(*, timeout) -> JsonAuxiliaryCurrentResult
ARAuxiliary.detach_gripper(*, timeout) -> None
ARAuxiliary.close() -> None
```

`with session:` returns the same active facade and closes the session on exit.
Configuration tuples, motion vectors, enum-like strings, pin ranges, Modbus
targets, and numeric bounds are validated by the exact JSON command contracts.
Facade parameter names map directly to the corresponding owners in the
[main-controller command surface](json-protocol-v1.md#main-controller-command-surface)
or [auxiliary-controller command surface](json-protocol-v1.md#auxiliary-controller-command-surface).
`speed_mode`, `wrist_configuration`, `loop_modes`, calibration axes and offset
units, vector units, and spline segment geometry receive no compatibility
conversion or implicit default beyond the forced options documented below.

## Motion contract

Every motion and calibration call is a separate operator admission. Main finite
motion is synchronous: the call returns only after a validated terminal response
or raises. `timeout` bounds host settlement, not controller cancellation. A host
timeout or transport uncertainty closes the session but cannot prove that motion
stopped or establish a final pose.

The facade always forces `telemetry_enabled=False` for joint and Cartesian
motion. `move_linear` also forces `rounding_millimeters=0.0` and
`disable_wrist_rotation=False`. No public facade option overrides those values.
Arc motion begins at the controller-confirmed current pose. Circle motion assumes
prior arrival at the declared start and inserts no preliminary move. One spline
call submits one validated segment collection atomically.

## Timeouts, errors, and recovery

All host timeouts must be positive finite numbers. Main Modbus waits and
`ARAuxiliary.wait_input` derive the host deadline from firmware
`timeout_seconds` plus two complete-frame response bounds. A completed Modbus
wait returns `JsonScalarResult` only when `value` equals `expected`.

- `AutomationStateError` reports inactive, closing, quarantined, or busy use.
- `AutomationCommandError` retains `controller`, `command`, `status`, `failure`,
  and the validated terminal. Reuse is permitted only while the session remains
  active. Ordinary rejections and auxiliary wait timeouts remain reusable. A
  main-controller `emergency_stop_active` rejection closes the session.
- Transport, framing, protocol, or host-timeout uncertainty closes the session.
  A failed main-controller write has an indeterminate physical outcome and must
  never be retried automatically.
- `AutomationCleanupError` retains the operation error, cleanup error, serial
  handle, and the client when client construction succeeded.

After a verified close caused by uncertainty, create a new serial handle and run
the full role-validated `connect` path. If `AutomationCleanupError` is raised,
the retained `serial_port`, together with `client` when non-`None`, identifies
unresolved cleanup ownership. Resolve and verify that handle state before
opening a replacement. Never continue from a presumed controller state.

## Physical-stop evidence and safety

`ARRobot.connect` requires `physical_stop_callback`. The callback receives a
validated `JsonMainControllerPhysicalStop` and must return the exact value
`True`. Emergency-stop evidence is published before terminal acknowledgement.
A publication failure raises `JsonMainControllerPhysicalStopPublicationError`.
A successful publication raises `JsonMainControllerPhysicalStopError`, retains
typed terminal position when available, and closes the facade session.

No finite-motion facade method provides a software stop. Before powered motion,
calibration, controller writes, or output activation, require a cleared work
envelope, a verified independent physical emergency stop, and an approved live
procedure. Static checks, fake transports, and successful imports provide no
live-hardware evidence.

## Definition-only examples

Each `automation-example` marker identifies the immediately following Python
fence for deterministic extraction. Example modules define functions only;
extraction and compilation perform no connection or command. Facade objects and
all command values enter through function arguments.

<!-- automation-example: read-main-state -->
```python
def read_main_state(robot, request):
    return {
        "position": robot.refresh_position(timeout=request["timeout"]),
        "limits": robot.read_limit_switches(timeout=request["timeout"]),
        "encoders": robot.read_encoders(timeout=request["timeout"]),
    }
```

<!-- automation-example: wait-auxiliary-input -->
```python
def wait_auxiliary_input(auxiliary, request):
    return auxiliary.wait_input(
        request["pin"],
        request["state"],
        timeout_seconds=request["timeout_seconds"],
    )
```

## Unsupported compatibility surface

No raw or compatibility API is provided. Audited V7 main names `send`,
`read_line`, `send_json`, `read_json`, `read_response`, `command_raw`,
`command`, `stop`, and `wait_modbus_input` remain unsupported. Use the typed
`ARRobot` methods; `ARRobot.wait_modbus_discrete_input` is the supported discrete
input wait replacement.

Audited V7 auxiliary names `ARNano`, `ARNano.send_json`, `ARNano.read_json`,
`ARNano.read_response`, `ARNano.command`, and `ARNano.stop` remain unsupported.
`ARAuxiliary` is the supported Nano/Mega facade.

No public `ARRobot.live_joint_jog`, `ARRobot.live_cart_jog`,
`ARRobot.live_tool_jog`, `ARRobot.renew_live_motion`, or `ARRobot.stop` facade
exists. Live leases, renewal, and stop remain HMI/protocol-owned functionality,
outside the synchronous scripting facade.
