# V7 feature coverage

This document records the supported product scope carried forward from the
delivered AR4 HMI 7.0 software. Current source and the linked public contracts define supported behavior.

## Controller coverage

The host and all tracked controllers use correlated JSON directly. Legacy
transport compatibility, raw protocol pass-through, and dual communication
paths are unsupported. The current protocol preserves every delivered
production command family and adds exact identity and command manifests,
bounded framing, request ownership, typed terminals, asynchronous event
handling, transport quarantine, and explicit physical-stop admission.

| Delivered main-controller family | Current route |
| --- | --- |
| `hello` | `hello` with exact controller identity, capabilities, ordered command manifest, payload limit, and session identity |
| `get_position`, `correct_position`, `set_position` | `get_position_disposition`, `correct_position`, `set_position` |
| `test_limit_switches`, `set_encoders`, `read_encoders` | Correlated commands with typed diagnostic results |
| `update_params`, `config_ext_axis`, `zero_j7`, `zero_j8`, `zero_j9` | Typed configuration operations with synchronized admission |
| `wait_time` | Emergency-cancellable `controller_wait` |
| `calibrate`, `calibrate_stage2` | One `calibrate` request for each explicit selected-axis operation |
| `move_joints`, `move_j`, `move_l`, `move_vis`, `jog_t` | `move_joints`, `move_cartesian`, `move_linear`, `move_vision`, `jog_tool` |
| Live jog and host stop | `live_joint_jog`, `live_cart_jog`, `live_tool_jog`, `stop`, `renew_live_motion` |
| `move_a`, `move_c`, `spline_start`, `spline_end` | Atomic `move_arc`, `move_circle`, and `move_spline` |
| Scalar Modbus reads, writes, conditions, and waits | Typed scalar operations, including `wait_modbus_discrete_input` and holding-register and coil waits |
| SD listing, deletion, upload, and playback | `list_sd_programs`, `delete_sd_program`, `write_gcode_move`, `play_gcode_file` |

The tracked Nano and Mega profiles retain `hello`, `servo`, `input_read`,
`set_output`, `wait_input`, `test_gripper_amps`, `stop`, and
`gripper_detach`. Board identity, allowed pins, and response schemas are profile
specific.

## Supported application capabilities

- Manual and programmed joint, Cartesian, linear, vision, tool-frame, arc,
  circle, spline, live, Xbox, and named-position motion use typed JSON owners.
- Single-axis and automatic calibration, controller position correction,
  home-reference handling, encoder telemetry, switch diagnostics, runtime
  parameter updates, external-axis configuration, and J7-J9 zeroing remain
  available.
- `.ar4` editing and execution retain motion, calibration, vision, G-code,
  auxiliary I/O, controller Modbus, branching, waits, and navigation rows.
- `ARRobot` and `ARAuxiliary` provide the supported synchronous scripting
  surface documented in [the Python automation guide](python-automation-api.md).
- `ARrobots.__version__` is the host-product version authority. Controller
  firmware versions, robot revision, protocol version, and Python ABI remain
  independent identities.
- The current native extension retains configured inverse kinematics, wrist
  selection, singularity and range checks, and atomic configuration.
- The calibration schema owns validated profile loading and persistence. The
  AR4-MK5 switch preset stages only the J1-J6 switch-polarity fields; applying
  and saving remain separate actions.
- The persistent CAD scene owns imported STL identity, workspace storage,
  transforms, deletion, parenting, and restart persistence. STEP/STP import is
  optional and uses the isolated worker contract in the
  [Windows packaging guide](../packaging/windows/README.md#step-worker-abi).

## Bundled visual tools

`RobotLinkView` owns the exact V7 link geometry, hierarchy, transforms, colors,
stable `tool_mount`, and bundled-tool actors described by the
[asset provenance contract](v7-asset-provenance.md). Supported bundled visual
selections are `Servo Gripper`, `Welding Torch`, and an explicit clear state.
Only one bundled visual actor can be selected. Imported CAD remains a separate
`PersistentCadScene` owner and can also attach below `tool_mount`.

The `EOATVisual` calibration field persists bundled visual intent. Tracked
defaults and a missing key in an otherwise valid older profile select
`Servo Gripper`. Explicit clear persists as an empty string. An unsupported
well-formed saved name is restored to the Servo default; malformed shared
configuration remains a load failure. Selection and clearing save the profile
before mutating the viewer. Save failure preserves the prior selection and
presentation. Calibration shutdown admission rejects visual-tool changes; an
active calibration save snapshot retains the selected value through the
existing persistence-retry owner.

`launch_vtk_nonblocking()` alone owns initial camera and clipping setup. Live
bundled-tool selection and clearing render the changed private child without
resetting the current camera.

Visual selection, imported CAD, and program workpiece attachment never command
a controller, servo, pneumatic output, or serial transport. Visual geometry is
not physical TCP, payload, collision, reachability, calibration, kinematics, or
motion authority.

## Virtual Pick and Place

Canonical program rows are:

```text
Virtual Pick - <32-lowercase-hex-object-id>
Virtual Place - <32-lowercase-hex-object-id>
```

Labels, filenames, uppercase IDs, alternate spacing, trailing content, and a
literal `ID` token are unsupported. Pick reparents the identified imported
object to `tool_mount`; Place reparents the object to `world`.
`PersistentCadScene.reparent()` preserves world pose and durable parent state.
Multiple imported objects may remain attached. Viewer reopen and application
restart restore durable parents.

Execution requires an active bound viewer and an existing object. Rejection
stops the entire running program without scene mutation, so later physical rows
cannot execute from an invalid visual workpiece state. Successful execution is
visual only and does not enter controller, auxiliary, output, or bundled-tool
paths.

## Deliberate exclusions

The following delivered or archive-only items are not product features:

- raw or legacy Python protocol pass-through;
- debug-only `home_position` and `modbus_query_test` routes;
- producerless `write_gcode_move_precalc` and `play_gcode_file_steps` modes;
- auxiliary `echo`, `test_message`, and duplicate `gripper` aliases;
- silent jog quantization and wording-only dirty-tree changes;
- generated compiler trees, ambiguous native binaries, local logs, machine
  configuration, and sample runtime state;
- additional EOAT archive candidates without selected product behavior.

## Remaining V7 release work

Windows redistribution remains selected V7 modernization work.
Source and frozen recipes, supported Python ABI, profile locks, native module,
and isolated STEP worker are present. Public release remains blocked until the
[third-party redistribution contract](../packaging/windows/THIRD_PARTY_NOTICES.md)
is closed, design-model authority is resolved or restricted assets are removed
from release recipes, a clean landed revision produces matching package builds,
and clean-machine validation succeeds with serial access disabled. Controller
commissioning remains a separate, explicitly authorized hardware procedure.
