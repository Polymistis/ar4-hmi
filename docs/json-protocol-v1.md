# AR4 controller JSON protocol v1

Status: JSON-only software contract. Live-controller verification remains
pending under the project safety boundary.

All main-controller and auxiliary-controller serial traffic uses protocol-v1
JSON. `.ar4` rows remain host application syntax and become semantic commands
before JSON encoding. Runtime protocol selection does not exist.

## Authority

The normative host definitions are:

- envelopes and framing limits: `ARrobots/protocol/messages.py`;
- correlation and terminal ownership: `ARrobots/protocol/session.py`,
  `ARrobots/protocol/transport.py`, and `ARrobots/protocol/coordinator.py`;
- canonical command manifests: `ARrobots/protocol/catalog.py` and
  `ARrobots/protocol/schemas.py`;
- exact command parameter, result, status, and error shapes:
  `JsonCommandContract` constants in `ARrobots/protocol/schemas.py`;
- device-bound behavior: `ARrobots.protocol.main_controller` and
  `ARrobots.protocol.auxiliary_controller`; and
- main startup order: `ARrobots/protocol/main_controller_startup.py`.

The Teensy `json_*_contract.h` files and the Nano/Mega
`auxiliary_protocol_contract.h` files mirror those host boundaries. A manifest,
identity, payload-limit, parser, or result-shape mismatch fails closed.

## Wire contract

- One JSON object occupies one serial frame.
- Canonical transmission is printable ASCII JSON followed by LF. Receivers
  also accept one CR immediately before LF.
- A frame contains exactly one terminal LF. Embedded LF, empty payloads,
  non-printable bytes, malformed JSON, duplicate object fields, and trailing
  input are invalid.
- `JSON_PROTOCOL_MAXIMUM_FRAME_BYTES` is 4096 and
  `JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES` is 4094. A peer-advertised lower
  payload limit applies before transmission. Nano and Mega advertise 384;
  canonical LF framing therefore permits a 384-byte auxiliary payload.
- The Teensy partial-frame receive deadline is 5.0 s. Live-control
  timeout calculation uses the greater of that deadline and the configured
  host frame timeout as one complete-frame bound.
- JSON nesting depth is at most 8. Each array and object contains at most 128
  entries. Ordinary strings contain at most 1024 printable ASCII characters;
  error messages contain 1-512.
- Command, status, event, stream, and error-code names contain 1-64 characters
  and match `[a-z][a-z0-9_]*`. Object field names contain 1-64 characters and
  match `[A-Za-z][A-Za-z0-9_]*`.
- Generic integers span `-2^63` through `2^64-1`. Request identifiers span
  `1` through `2^32-1`; event and telemetry sequences span `0` through
  `2^32-1`. Command contracts apply narrower native, unit, and safety bounds.
- Floats must be finite. Decimal fractions that underflow to zero and command
  values that cannot survive required binary32 conversion are invalid.
- Envelope fields are exact. Additional, missing, or duplicate fields are
  rejected at every boundary.

The envelope shapes are:

```text
request:        {"v":1,"type":"request","id":ID,"cmd":NAME,"params":{...}}
response ok:    {"v":1,"type":"response","id":ID,"cmd":NAME,
                 "status":"accepted|completed","result":{...}}
response error: {"v":1,"type":"response","id":ID,"cmd":NAME,
                 "status":"rejected|cancelled|failed",
                 "error":{"code":NAME,"message":TEXT,"details":{...}}}
event:          {"v":1,"type":"event","seq":SEQ,"event":NAME,"data":{...}}
telemetry:      {"v":1,"type":"telemetry","seq":SEQ,"stream":NAME,
                 "data":{...}}
protocol error: {"v":1,"type":"protocol_error",
                 "error":{"code":NAME,"message":TEXT,"details":{...}}}
```

`accepted` is a nonterminal admission response and is used by live-jog
requests. `completed`, `rejected`, `cancelled`, and `failed` are terminal.
Successful responses contain `result` and no `error`; error terminals contain
`error` and no `result`. Command contracts reserve `rejected` for a request
that did not commit the requested operation, `cancelled` for a bounded
interruption with known disposition, and `failed` for execution failure.

A parser failure retaining a valid request identifier and canonical command
returns a correlated `rejected` response. A failure without valid correlation
returns `protocol_error`. Receipt of an uncorrelated protocol error while a
request remains active quarantines the connection.

## Correlation and ownership

- One coordinator owns serial reads and complete-frame assembly for each open
  handle. Reader ownership can move only at a verified complete-frame boundary.
- Request allocation starts at 1, advances modulo `2^32-1`, and skips every
  identifier retained by an admitted request or unsettled cleanup.
- An admitted request owns the identifier until one terminal is retrieved,
  acknowledged, and cleanup settles. A controller emits at most one
  `accepted` response and exactly one terminal response.
- Identifier mismatch, command mismatch, duplicate terminal, terminal before
  required acceptance, missing terminal, malformed frame, sequence error, or
  unexpected envelope quarantines the connection.
- One absolute monotonic deadline governs each request. Expiry initiates
  session-owned cancellation or quarantine; expiry never releases a committed
  identifier by omission.
- Events and telemetry are demultiplexed before command-result parsing. A
  response frame already being written completes before an event frame begins;
  event output cannot split or replace correlated response ownership.
- The main event counter is session-wide, begins at zero, and advances modulo
  `2^32`. The first observed event establishes a baseline; a later gap or
  reordering fails closed. A new main `session_id` resets the baseline.
- Telemetry counters are per stream. Telemetry is low priority and droppable,
  never completes a request, and never establishes confirmed robot position.
- Firmware retains response and event buffers through the complete framed
  write. Partial output, invalid write accounting, ownership conflict, or
  framing loss faults the session.

Physical emergency-stop admission blocks new controller mutation. A validated
`emergency_stop` event or `emergency_stop_active` disposition enters the HMI
physical-stop publication path before fallible terminal acknowledgement.
Active requests retain correlated settlement. Confirmed pose comes only from
an authoritative command terminal or state query, never from telemetry or an
invented substitute value.

## Identity and startup

### Main controller

`hello {}` must be the first request. The exact completed result is owned by
`MAIN_HELLO_COMMAND_CONTRACT` and `parse_main_hello_result` and contains only:

- `device: "main_controller"`;
- `firmware: {"name":"AR4 Teensy","version":"6.7.1-ar4hmi.39",
  "build":"tracked"}`;
- `protocol: {"name":"ar4_json","version":1,
  "max_payload_bytes":4094}`;
- `capabilities`, containing exactly `JSON_PROTOCOL_V1`,
  `REQUEST_CORRELATION_V1`, and `EVENT_STREAM_V1` with no domain capability
  flags;
- `commands`, exactly equal and order-equal to `JSON_MAIN_COMMAND_MANIFEST`;
- `session_id`, exactly 32 uppercase hexadecimal characters and fresh for the
  controller session; and
- `identity`, containing exactly `controller_hardware_id`, `driver_model`,
  `robot_model`, `robot_version`, `serial_number`, and `asset_tag`.

`controller_hardware_id` is exactly six uppercase hexadecimal characters.
Identity and firmware text fields contain 1-31 printable ASCII characters.

`run_main_controller_json_startup` performs this strict order:

1. `hello`
2. `update_params`
3. `config_ext_axis`
4. `set_position`
5. `get_position_disposition`
6. `get_home_reference`
7. drain retained input, release reader ownership, and publish the persistent
   `JsonMainControllerClient`

Each terminal is validated and acknowledged before the next request. Any
identity, capability, manifest, schema, correlation, physical-stop, timeout,
or cleanup failure aborts startup and closes or quarantines the retained
client. Cancellation before final write admission sends no request;
cancellation after commitment preserves terminal and cleanup ownership.

### Auxiliary controller

`hello {}` must precede every semantic auxiliary request. The exact result is
owned by `AUXILIARY_HELLO_COMMAND_CONTRACT` and
`parse_auxiliary_hello_result` and contains only `board`, `commands`, `device`,
`firmware`, and `protocol`.

- Nano: `board: "nano"`, firmware `AR4 Nano IO` / `2.0` / `ar4hmi`.
- Mega: `board: "mega"`, firmware `AR4 Mega IO` / `2.0` / `ar4hmi`.
- Both: `device: "auxiliary_controller"`, protocol `ar4_json` v1,
  `max_payload_bytes: 384`, and an exact `JSON_AUXILIARY_COMMAND_MANIFEST`.

A completed and acknowledged hello binds the board profile and negotiated
payload limit to `JsonAuxiliaryControllerClient`. Auxiliary hello has no
session capability array or session identifier.

## Main-controller command surface

The table lists every active command and the symbol owning the exact parameter,
result, status, and error shapes. `MAIN_JSON_COMMAND_CONTRACTS` binds these
contracts to `JsonMainControllerClient`; `JSON_MAIN_COMMAND_MANIFEST` and the
Teensy `JSON_COMMANDS` array must remain order-identical.

| Commands | Exact schema owner |
| --- | --- |
| `hello` | `MAIN_HELLO_COMMAND_CONTRACT` |
| `get_home_reference` | `MAIN_GET_HOME_REFERENCE_COMMAND_CONTRACT` |
| `get_position_disposition` | `MAIN_GET_POSITION_DISPOSITION_COMMAND_CONTRACT` |
| `correct_position` | `MAIN_CORRECT_POSITION_COMMAND_CONTRACT` |
| `set_position` | `MAIN_SET_POSITION_COMMAND_CONTRACT` |
| `test_limit_switches`, `set_encoders`, `read_encoders` | `MAIN_TEST_LIMIT_SWITCHES_COMMAND_CONTRACT`, `MAIN_SET_ENCODERS_COMMAND_CONTRACT`, `MAIN_READ_ENCODERS_COMMAND_CONTRACT` |
| `get_motion_trace` | `MAIN_GET_MOTION_TRACE_COMMAND_CONTRACT` |
| `update_params`, `config_ext_axis` | `MAIN_UPDATE_PARAMS_COMMAND_CONTRACT`, `MAIN_CONFIG_EXT_AXIS_COMMAND_CONTRACT` |
| `zero_j7`, `zero_j8`, `zero_j9` | `MAIN_ZERO_J7_COMMAND_CONTRACT`, `MAIN_ZERO_J8_COMMAND_CONTRACT`, `MAIN_ZERO_J9_COMMAND_CONTRACT` |
| `controller_wait` | `MAIN_CONTROLLER_WAIT_COMMAND_CONTRACT` |
| `calibrate` | `MAIN_CALIBRATE_COMMAND_CONTRACT` |
| `move_joints` | `MAIN_MOVE_JOINTS_COMMAND_CONTRACT` |
| `move_cartesian`, `move_linear`, `move_vision` | `MAIN_MOVE_CARTESIAN_COMMAND_CONTRACT`, `MAIN_MOVE_LINEAR_COMMAND_CONTRACT`, `MAIN_MOVE_VISION_COMMAND_CONTRACT` |
| `move_arc`, `move_circle`, `move_spline` | `MAIN_MOVE_ARC_COMMAND_CONTRACT`, `MAIN_MOVE_CIRCLE_COMMAND_CONTRACT`, `MAIN_MOVE_SPLINE_COMMAND_CONTRACT` |
| `jog_tool` | `MAIN_JOG_TOOL_COMMAND_CONTRACT` |
| `live_joint_jog`, `live_cart_jog`, `live_tool_jog` | `MAIN_LIVE_JOINT_JOG_COMMAND_CONTRACT`, `MAIN_LIVE_CART_JOG_COMMAND_CONTRACT`, `MAIN_LIVE_TOOL_JOG_COMMAND_CONTRACT` |
| `stop`, `renew_live_motion` | `MAIN_STOP_COMMAND_CONTRACT`, `MAIN_RENEW_LIVE_MOTION_COMMAND_CONTRACT` |
| `modbus_read_holding_register`, `modbus_read_coil`, `modbus_read_discrete_input`, `modbus_read_input_register` | `MAIN_MODBUS_READ_HOLDING_REGISTER_COMMAND_CONTRACT`, `MAIN_MODBUS_READ_COIL_COMMAND_CONTRACT`, `MAIN_MODBUS_READ_DISCRETE_INPUT_COMMAND_CONTRACT`, `MAIN_MODBUS_READ_INPUT_REGISTER_COMMAND_CONTRACT` |
| `modbus_write_coil`, `modbus_write_register` | `MAIN_MODBUS_WRITE_COIL_COMMAND_CONTRACT`, `MAIN_MODBUS_WRITE_REGISTER_COMMAND_CONTRACT` |
| `wait_modbus_coil`, `wait_modbus_discrete_input`, `wait_modbus_holding_register` | `MAIN_WAIT_MODBUS_COIL_COMMAND_CONTRACT`, `MAIN_WAIT_MODBUS_DISCRETE_INPUT_COMMAND_CONTRACT`, `MAIN_WAIT_MODBUS_HOLDING_REGISTER_COMMAND_CONTRACT` |
| `delete_sd_program`, `list_sd_programs`, `write_gcode_move` | `MAIN_DELETE_SD_PROGRAM_COMMAND_CONTRACT`, `MAIN_LIST_SD_PROGRAMS_COMMAND_CONTRACT`, `MAIN_WRITE_GCODE_MOVE_COMMAND_CONTRACT` |
| `play_gcode_file` | `MAIN_PLAY_GCODE_FILE_COMMAND_CONTRACT` |

### Motion and live leases

Motion requests require a completed hello and synchronized configuration.
Only the retained motion owner can accept correlated live control. Motion
success, interruption, and known execution-failure terminals carry typed
position disposition when available; `position_unavailable` prevents pose
manufacture and requires state recovery.

`move_joints` requires `trace_configuration_fingerprint`. Ordinary requests
send `null`. One opted-in manual request sends the synchronized
`sha256:<lowercase hexadecimal>` configuration fingerprint and must set
`telemetry_enabled` to `false`. The fingerprint covers canonical validated
`update_params` and `config_ext_axis` requests; position synchronization and
motion input remain separate evidence.

After an eligible authoritative terminal, `get_motion_trace` accepts exactly
`{"motion_request_id":ID,"page_index":INDEX}`. A missing capture returns
`{"capture_state":"no_capture","source_motion_request_id":ID}`. An available
page identifies the capture generation, source session and motion request,
configuration fingerprint, firmware, disposition, page bounds, and records.
Records contain controller-relative microseconds, master index, scheduled
delay, J1-J6 commanded steps and encoder counts, motion phase, and capture
flags. Every page repeats immutable identity and disposition
metadata. Host assembly accepts exact page order from one generation only.
Physical-stop and untrusted-session paths never retrieve a trace. Only a
complete validated assembly can become one atomically promoted local artifact.

`move_linear` wraps the exact `move_cartesian` parameters under `motion` and
requires `rounding_millimeters: 0` plus `disable_wrist_rotation: false`. The
standalone command rejects nonzero rounding before transmission. The HMI
preserves rounded program paths by compiling contiguous nonzero-rounded
`Move L` rows, terminated by a zero-rounding `Move L` row, into one bounded
`move_spline` request. `Start Spline` through `End Spline` uses the same atomic
request and normalizes the final target to zero rounding because a terminal
target has no outgoing leg. Intermediate rounding values remain unchanged;
the contiguous-row form rejects a span without an authored zero-rounding
terminus. No controller-global cross-request spline state is created. The
active HMI command grammar also rejects a selected wrist-disable flag because
no tracked firmware implements the motion semantic; the setting is never
silently replaced.

`move_arc` has exact parameters
`{"midpoint_translation_millimeters":[X,Y,Z],"motion":MOTION}`. The current
confirmed Cartesian pose is the arc start. `Move A Mid` contributes only the
midpoint translation that selects the traversal; `Move A End` supplies the
complete endpoint `MOTION`. `move_circle` has exact
parameters `{"center_translation_millimeters":[X,Y,Z],"motion":MOTION,
"plane_translation_millimeters":[X,Y,Z]}`. The declared circle start is
`MOTION.translation_millimeters`; HMI program compilation first completes an
ordinary `move_cartesian` to the declared start, then the atomic circle request
verifies exact J1-J9 controller-step equality before any circle output. The
circle radius comes only from the center-to-start vector. The plane point is
not visited and need not lie on the circumference; a finite, non-collinear
center-to-plane vector selects the normal and traversal direction.

`move_spline` has exact parameters
`{"segments":[{"motion":MOTION,"rounding_millimeters":R},...]}`. One through
six segments are permitted. Every rounding value is finite and nonnegative,
the final value equals zero, and every positive value is at most 45 percent of
each adjacent translation leg. Firmware validates the first incoming leg from
the confirmed controller pose and rejects an excessive value before output;
firmware never reduces a transmitted rounding value. Arc, circle, and spline
`MOTION` objects use the exact `move_cartesian` shape with
`telemetry_enabled: false`. Arc, circle, and spline are synchronous and
terminal-only: no `accepted` response is emitted.
Firmware completes geometry, inverse-kinematics, joint-limit, timing, and
trajectory preflight before motor output. The completed result is exact
`{"controller_debug":TEXT,"position":POSITION,"speed_limited":BOOLEAN}`;
`POSITION` contains only `axis_source`, `cartesian_micrometers`,
`external_axes_milliunits`, `orientation_millidegrees`, and
`robot_joints_millidegrees` under the standard controller-step-state position
contract.

`wait_modbus_holding_register` has exact parameters
`{"address":0..65535,"expected":0..65535,"slave_id":1..247,
"timeout_seconds":1..2147483}` and completed result
`{"value":0..65535}`. The returned value must equal the requested value.
`failed/modbus_error`, `failed/timeout`, and `cancelled/emergency_stop` use
exact empty error details. Normal synchronized-session rejections retain the
existing command-specific detail contract and emergency-stop cancellation
retains the existing blocking-request owner.

`play_gcode_file` requires a correlated `accepted` response before SD execution
begins. The host applies a finite pre-acceptance deadline, then suspends that
deadline until the same request identifier receives a completed, cancelled, or
failed terminal. Firmware retains one copied request under the existing response
owner, writes the accepted frame completely, executes the stored program, and
only then stages the terminal; another request cannot enter during playback.
A completed terminal uses the standard Cartesian-motion result and reports
`speed_limited: true` when any completed stored row was speed-limited.

`live_joint_jog`, `live_cart_jog`, and `live_tool_jog` require one `accepted`
response before motion advances. `lease_milliseconds` is an integer from 1000
through 5000. Firmware starts the lease only after the complete accepted frame
is written, checks the lease before direction mutation and coordinated pulse
admission, and stops pulse generation on expiry.

`stop` and `renew_live_motion` use independent request identifiers and exact
parameters `{"motion_id": LIVE_REQUEST_ID}`. A completed control result returns
the same `motion_id`. Stop settles the control request first; the original live
request then returns the authoritative motion terminal. Renewal restarts the
lease. Host control timeouts must exceed the lease plus two complete-frame
bounds, and original-terminal retrieval remains blocked until retained control
settlement completes.

### External-write uncertainty

`modbus_write_coil`, `modbus_write_register`, `delete_sd_program`, and
`write_gcode_move` can change state outside the host.
A pre-admission rejection proves no mutation. After final write admission,
transport loss, timeout, framing failure, cleanup failure, or a `failed`
terminal leaves external state indeterminate. Such a failed terminal remains
inspectable but cannot be acknowledged; later submission stays blocked until
connection cleanup and external-state verification. Automatic retry is
prohibited.

## Auxiliary-controller command surface

| Command | Exact parameters | Completed result | Schema owner |
| --- | --- | --- | --- |
| `hello` | `{}` | `JsonAuxiliaryHelloResult` | `AUXILIARY_HELLO_COMMAND_CONTRACT` |
| `servo` | `{"channel":0..6,"position":0..180}` | `{}` | `AUXILIARY_SERVO_COMMAND_CONTRACT` |
| `input_read` | `{"pin":integer}` | `{"state":boolean}` | `AUXILIARY_INPUT_READ_COMMAND_CONTRACT` |
| `set_output` | `{"pin":integer,"state":boolean}` | `{}` | `AUXILIARY_SET_OUTPUT_COMMAND_CONTRACT` |
| `wait_input` | `{"pin":integer,"state":boolean,"timeout_seconds":1..32767}` | `{}` | `AUXILIARY_WAIT_INPUT_COMMAND_CONTRACT` |
| `test_gripper_amps` | `{}` | `{"amps":0..28}` | `AUXILIARY_TEST_GRIPPER_AMPS_COMMAND_CONTRACT` |
| `stop` | `{}` | `{}` | `AUXILIARY_STOP_COMMAND_CONTRACT` |
| `gripper_detach` | `{}` | `{}` | `AUXILIARY_GRIPPER_DETACH_COMMAND_CONTRACT` |

Board-bound validation narrows shared schema ranges before transmission:

- Nano servo channels are 0-5, input pins are 2-7, and output pins are 8-13.
- Mega servo channels are 0-6, input pins are 2-27, and output pins are 28-53.

Only `stop` can enter while one `wait_input` request is active. A matching
input completes the wait, timeout returns `failed/timeout`, and stop writes a
completed stop terminal before the retained wait returns
`cancelled/stop_requested`. Every other auxiliary request remains blocked
until wait settlement. An injected stop that races a natural wait terminal is
not reported successful until a correlated `stop` request also completes.
`gripper_detach` idempotently detaches servo channel 0 and returns `{}`. An
active `wait_input` produces the standard `rejected/busy` terminal. Orderly HMI
shutdown submits one best-effort correlated detach only after auxiliary work
and stop ownership settle; failure is diagnosed, and uncertainty after write
admission is never retried automatically.

## Verification boundary

Schema validation, static analysis, simulated serial traffic, and no-upload
firmware compilation are hardware-free evidence only. Live-arm verification
requires the separate authorization and recording procedure in `SAFETY.md`.
