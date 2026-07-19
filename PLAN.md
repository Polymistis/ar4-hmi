# AR4HMI Plan

## Objective

Adapt the existing AR4 control stack into a maintainable, hardware-safe system for a defined robotic-arm application. Initial priorities cover HMI responsiveness, maintainability, coordinated motion, and safe interaction during repositioning; hardware and workflow constraints remain incomplete.

## Authority

This file defines project scope, status, acceptance criteria, and architectural decisions. Implementation claims must match the repository. Hardware claims require recorded live-arm evidence.

## Status vocabulary

- `Proposed`: scope described; implementation not started.
- `Implemented`: repository implementation exists; required verification may remain.
- `Tested`: automated checks exercise the stated behavior without attached hardware.
- `Hardware-verified`: an authorized live-arm procedure produced recorded evidence.
- `Blocked`: progress requires a named decision, dependency, permission, or external state change.

## Established baseline

- Commit `63c886f` preserves the imported AR4 control-software baseline.
- Commit `627990a` installs cross-review-gate and the per-clone pre-commit dispatcher.
- `AR4.py` identifies host source version 6.7; tracked Teensy motion firmware identifies version 6.7.1.
- Runtime calibration and machine state remain untracked; `defaults.json` remains the tracked default profile.
- No live-arm command, firmware flash, calibration cycle, or movement was performed during repository setup.

## Safety invariants

- Automated checks must not open serial ports or issue controller commands.
- Host startup must become separable from GUI construction and hardware connection before import-based testing.
- Hardware verification must confirm that the physical emergency-stop path operates independently from desktop GUI state before commanded movement.
- Motion-protocol changes require synchronized host and firmware updates.
- Joint limits, calibration units, saved pose handling, encoder modes, and stop behavior require explicit tests before hardware verification.
- Hardware verification requires a cleared work envelope, verified emergency stop, conservative motion settings, and an operator-approved procedure.

## Milestones

### M0 - Repository and review foundation

Status: `Implemented`

Acceptance criteria:

- Original source preserved in a local Git baseline.
- Cross-review wrappers and per-clone dispatcher installed.
- Gate self-tests pass on the development machine.
- Project safety contract and reviewer hazards are tracked.

### M1 - Application requirement capture

Status: `Blocked`

Recorded priorities:

- Improve HMI responsiveness and maintainability by mapping active call paths, measuring event-loop delays, and removing verified redundant or shadowed functions.
- Keep Tk interaction responsive during repositioning without allowing conflicting controller commands.
- Verify coordinated multi-axis motion from host command through firmware pulse scheduling; distinguish controller-side interpolation from desktop command serialization.
- Keep stop controls and motion status available during movement through explicit command arbitration and main-thread UI updates.
- Complete a host-only HMI pass before motion-planning and interception work.
- Follow with a research and simulation pass for repeatability, online replanning, object tracking, prediction, and interception.

Required inputs:

- Robot generation, controller boards, installed firmware, and axis configuration.
- End effector, payload, tooling, fixtures, and reachable work envelope.
- Intended workflow, operator interaction, program source, and failure recovery.
- Accuracy, repeatability, cycle-time, speed, and duty-cycle targets.
- External I/O, PLC, Modbus, camera, sensor, and gripper integration.
- Deployment operating system, Python runtime, packaging, and maintenance constraints.
- Available guarding, emergency-stop chain, limit switches, encoder configuration, and commissioning process.

Acceptance criteria:

- Target workflow and non-goals are explicit.
- Measurable performance and safety constraints are recorded.
- Hardware and software boundaries are identified.
- Priority order is approved before hardware optimization work begins.

### M2 - Testable HMI transport boundary

Status: `Implemented`

Acceptance criteria:

- Importing testable modules cannot connect to hardware.
- Serial transports support deterministic fakes without changing production semantics.
- Joint-motion validation, framing, and queue logic remain outside GUI construction.

### M2B - Application lifecycle separation

Status: `Proposed`

Acceptance criteria:

- GUI construction, configuration loading, serial connection, and controller synchronization are explicit lifecycle operations.
- Importing the application entry point cannot construct the GUI or schedule a saved controller connection.
- Runtime configuration is schema-validated before use.

### M3 - HMI motion regression foundation

Status: `Tested`

Acceptance criteria:

- Host protocol serialization and response parsing have hardware-free tests.
- Coalescing, deferred input, transport release, timeout, quarantine, and Tk event handoff behavior have hardware-free tests.
- Source-contract tests inspect HMI routing without importing the application entry point.

### M3B - Broader regression foundation

Status: `Proposed`

Acceptance criteria:

- `.ar4` parsing, editing, and execution-state transitions have focused tests.
- Kinematics boundary behavior and calibration conversion have deterministic tests.
- Firmware protocol fixtures cover command compatibility without energizing motors.

### M4 - Purpose-specific optimization

Status: `Blocked`

Host-only HMI optimization is authorized. Hardware tuning, live motion optimization, and application-specific performance claims remain blocked on the missing M1 hardware and measurement inputs. Candidate work must trace to approved measurements such as responsiveness, motion-cycle latency, reliability, maintainability, or operator workflow.

Blocking condition: M1 hardware identity, workflow, safety, and performance inputs are required before milestone-level measurements or hardware optimization can complete.

Acceptance criteria:

- Baseline measurement and target are documented.
- Changes preserve safety invariants and protocol compatibility.
- Automated evidence demonstrates the intended improvement.

### M4A - HMI responsiveness pass

Status: `Implemented`

Acceptance criteria:

- Covered joint and live-jog Tk callbacks perform no blocking serial read, fixed sleep, or motion wait; worker results return through event queues for Tk application.
- Incremental J1-J9 input recorded during an owned move retains semantic delta and absolute-target ordering, then dispatches one consolidated final multi-axis target from confirmed controller state.
- A matching request-scoped owner spans controller-result application and virtual-operation settlement, preventing unrelated motion admission during partial completion.
- Public callbacks that change controller position, calibration, or port selection require logical-motion admission before transport admission. `requestPos` and main-controller replacement remain explicit position-recovery admissions.
- Manual online motion restores the last confirmed virtual pose only when physical transmission never starts. Transmission uncertainty blocks new operator and program motion until a fault-free controller position response resynchronizes the virtual model and joint dispatcher; controller fault correction remains an explicit recovery-only exception.
- Serial and calibration boundary failures reject, quarantine, or retain explicit recovery state without silently advancing confirmed motion or persisted calibration state.
- Deterministic hardware-free checks cover host coalescing, ownership ordering, controller and six-axis simulator timing boundaries, deadline propagation, write-boundary recovery, E-stop response admission, and failure settlement. The missing-file firmware branch has a source-contract check only; firmware compilation and live verification remain pending.

Implemented HMI work:

- Incremental J1-J9 inputs share a validated semantic target dispatcher.
- Main-controller joint commands are serialized while later button deltas and absolute slider targets replace the latest pending multi-axis target. Tracked direct main-controller transactions reserve the same nesting-safe transport admission lock, and asynchronous program motion transfers that reservation to the response worker without opening an interleaving window.
- Joint inputs made during an active legacy move preserve ordered delta and absolute-target semantics, rebase relative changes on the next confirmed controller position, and dispatch a consolidated multi-axis target. Main-thread terminal-result processing keeps transport admission reserved through response application, then releases the matching operation before retrying retained input. Input recorded after a valid response can use that confirmed generation at the release boundary, while unknown controller state blocks dispatch until resynchronization. A transient transport-busy result retains deferred intent; permanent validation or queue rejection clears stale intent instead of retrying indefinitely.
- Deferred joint resolution, dispatcher admission, and consumed-state clearing share one lock. Concurrent input is included in the admitted snapshot or remains pending for the next confirmed generation, and dispatcher worker startup completes or rolls back before another submission can receive acceptance.
- Controller errors, malformed responses, timeouts, and embedded motion faults discard pending motion and require position resynchronization. Automatic calibration uses cancellation-bound single-frame ownership; any post-write framing failure, extra frame, non-position response, or controller fault quarantines the transport, preserves the pre-command local position state, and blocks later calibration stages.
- Owned line exchanges classify controller opcodes before parsing fields. Motion opcodes require complete opcode-specific envelopes, so timing-like payload text cannot replace or duplicate motion timing. Controller-bound `RJ` and `MJ` envelopes require J7-J9, while simulator-bound `RJ` and `MJ` use separate exact J1-J6 and Cartesian envelopes and legacy `JT` retains an explicit simulator contract. Bounded commands validate encoded speed, acceleration, deceleration, and ramp fields, reject acceleration and deceleration regions whose combined share exceeds the move, then derive finite response deadlines from validated timing and configured full-travel bounds. `PG` playback has no duration contract in the current firmware and therefore retains response ownership without a fixed terminal deadline; application shutdown cancels the host exchange and quarantines the transport, without claiming physical-motion preemption. Live-jog start acknowledgement uses `SERIAL_LIVE_ACK_TIMEOUT_SECONDS`; completion after the stop token reuses the command-specific full-travel deadline, while the deadline remains suspended during an acknowledged hold. Live-jog and playback cancellation admission is rechecked under the serial write lock after stale-input reset, so cancellation before transmission writes neither the motion command nor a fail-safe stop token. Any premature terminal data, framing failure, or timeout-state failure after transmission writes the fail-safe stop token where possible, quarantines the main serial transport before ownership release, attempts a verified close, and requires explicit reconnection. A failed close retains the poisoned handle for later cleanup instead of dropping application ownership. Temporary exchange-timeout restoration is attempted across exit paths, and restoration failure triggers the same quarantine.
- Owned framed responses remove only a required LF and an optional preceding CR. Response validation separates the 4096-byte payload limit from the 4098-byte CRLF-capable frame limit, so a maximum-length payload remains valid with either supported delimiter. Payload padding, additional delimiters, non-ASCII data, and queued data outside an operation-specific follow-up contract are protocol failures; terminal readers require a complete bounded quiet interval before releasing ownership and quarantine the transport when the response deadline cannot provide that interval. Host response parsing accepts a validated controller-initiated `EB` position as a physical-E-stop terminal frame. The trusted Teensy v6.7.1 baseline still performs serial response work from `EstopProg` and emits speculative spline acknowledgements from `processSerial`, so an interrupt race can produce multiple terminal frames. No E-stop firmware rewrite is included in M4A. Safe response ownership requires a controller-protocol redesign followed by compilation, simulation, and authorized live-arm verification.
- Serial worker terminal results reach Tk through event queues. The matching transport and shutdown-activity lease remain reserved until Tk applies the operation-local result and releases transport admission. The matching callback then settles any request-scoped motion owner before deferred input is retried. Joint-dispatch and startup leases likewise span worker execution through terminal acknowledgement or connection cleanup. Parsed M1 speed-violation metadata survives serial, manual, startup, calibration, and joint completion; ready or queued status cannot overwrite the warning. Joint M1 completion discards queued and deferred targets, rebases desired state on the confirmed controller position, and leaves later input admissible from that position.
- Controller fault rendering queues recovery instead of performing controller I/O on Tk. Collision correction waits for main-transport release before dispatching `CP\n`. Auxiliary program stop uses request-correlated pending, dispatched, completed, and failed states. Only the firmware's interruptible `WI` wait permits write-only `STOP\n` injection while the wait worker retains sole response ownership. `Nano Stopped` acknowledges interruption directly. Immediately after successful `STOP\n` transmission, one absolute monotonic deadline based on `SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS` replaces the original `WI` deadline. Wait-owner handoff, optional owner-side follow-up, serial-lock acquisition, stop-worker follow-up read, and the required quiet boundary all consume that same deadline. When racing natural `Done` or `Timeout` and `Nano Inactive Stopped` frames are already queued, the wait owner validates both before release and publishes the inactive acknowledgement; when the second frame arrives later, the stop worker consumes the bounded follow-up after ownership release. Missing, unexpected, or additional follow-up data quarantines the auxiliary transport. Program halt status explicitly reports that active main motion is not preempted. Other auxiliary operations retain a pending stop request until an exclusive exchange returns the expected inactive-stop acknowledgement. `WI` timeouts are constrained to the firmware integer range, receive an encoded-timeout-plus-margin read deadline, and accept only documented terminal responses. Servo and output rows consume bounded unframed acknowledgements, require a quiet read boundary after the expected bytes, and quarantine empty, partial, unexpected, or trailing data. Legacy auxiliary commands reset stale input before transmission and share the stop-token write lock, preventing byte interleaving or post-write response erasure. Windows and Linux Xbox gripper paths share the bounded validated exchange owner. Every program, manual, and Xbox `ON`/`OF` command is validated against an explicit Nano or Mega profile bound to the active auxiliary serial handle; an unselected or stale profile blocks output transmission. Nano accepts outputs 8-13, Mega accepts outputs 28-53, and each profile rejects the complete opposing range. Windows pneumatic toggles use profile-valid output 8 for Nano and 28 for Mega. Closing or replacing the handle clears the binding, and existing configurations default to no selected board until the operator chooses a profile. Windows toggle state remains pending until the expected acknowledgement returns through the Tk event queue; transport rejection preserves the last acknowledged state, while an invalid exchange marks state unknown and attempts a verified auxiliary close. Linux performs the same exchange synchronously in the controller worker and changes local grip state only after acknowledgement.
- Live joint, Cartesian, and tool jog callbacks use an interruptible worker-owned request. Button release sets a stop event without sleeping or reading serial data on Tk; the worker writes the firmware stop token and remains the sole response reader. Live modes normalize Seconds and millimetres-per-second selections to Percent because firmware accepts stop input only between segments. Offline live workers reserve a shared request slot before launch, synchronize stop against segment admission, and wait for each request-scoped virtual result. The request-scoped operation remains authoritative until matching Tk settlement, so rejected or invalid later input cannot clear active state and button release still signals the active owner even if the display flag is stale. Tk performs the final virtual-state snapshot only after terminal ownership settlement, so a later press cannot reactivate released work or overlap an unfinished segment. Virtual live-jog errors return through a Tk event queue.
- Windows Xbox joint, Cartesian, and tool intent uses generation-scoped admission arbiters. Held intent remains pending after a busy rejection, active state changes only after the matching Tk live-jog callback accepts the same semantic value, and failed stop or scheduling admission retains pending intent with an explicit diagnostic. A failure from the deferred inner Tk registration clears the scheduled-attempt state before reporting the diagnostic, allowing later held input to retry. Application-closing cancellation before or after Tk admission clears pending scheduling without a false alarm. The Xbox Teach callback also records Tk scheduling rejection. Watchdog scheduler failure disables Windows polling, signals online and offline live-stop events, requests every arbiter to stop, and reports an alarm. Controller polling stops during application shutdown; Linux drops events returned after controller-off or shutdown state becomes active.
- Online joint targets coalesce into the virtual target and confirmed controller responses resynchronize the virtual model. A confirmed joint result that cannot update the virtual model invalidates the pending dispatcher state before response acknowledgement, retains the synchronization warning, and blocks further motion until a later fault-free position response resynchronizes both models. Manual online Cartesian and tool-frame motion captures the last confirmed virtual pose before preview dispatch. A failure before the physical write boundary restores that pose; a failure after transmission starts preserves uncertainty and blocks new operator and program motion until a fault-free controller position response resynchronizes the virtual model and joint dispatcher. Controller fault correction remains an explicit recovery-only exception. Physical success applies the confirmed controller pose even after virtual preview failure. Online live jog no longer launches a competing virtual drive worker; offline J7-J9 input is rejected because the virtual model supports J1-J6 only.
- Simulator `MJ` millimetres-per-second timing captures a validated six-value Cartesian start pose from forward kinematics and a validated target pose before virtual worker admission. Translation distance derives from those explicit endpoints; missing, malformed, or non-finite endpoints reject before virtual motion state changes.
- Online/offline mode changes reject every tracked physical or virtual motion owner. Offline-to-online synchronization validates and applies the controller position before mode, button, status, or virtual-position state commits; failure preserves the offline snapshot.
- Offline manual, joint, program, and live settlement retains request ownership while applying the terminal six-joint pose or restoring the saved pose after failure. FK-derived calibration and widgets refresh within that interval; refresh failure rejects settlement. Virtual drive workers do not perform Tk refresh work.
- Position parsing validates the complete response before state mutation. The accepted frame requires ordered A-R markers, delimiter-safe decimal position fields, the firmware speed bit, an empty or numeric debug field, and a blank, `EB`, or six-bit `EC` fault field; duplicated markers and arbitrary payloads are rejected before calibration, widget, or persistence mutation. Every inbound J1-J9 position, including dispatcher-owned, automatic-calibration, and fault-flagged responses, must fit the applicable calibrated axis limits and signed step-counter range before dispatcher state, `CAL`, position-dependent widgets, persistence state, virtual state, or position-fault rendering can change. External position responses must resynchronize an idle dispatcher before calibration state or position generations advance; dispatcher-owned completion events bypass only that external synchronization. Direct program rows, automatic and single-joint calibration, position requests, offline-to-online synchronization, and forced calibration poses advance success state or copy virtual positions only after a fault-free response is applied. Controller errors, malformed data, embedded motion faults, and dispatcher rejection propagate as operation failure. Every numeric field in a supported raw motion envelope, timing profile, and numeric suffix is captured, binary32 range-checked, and converted to delimiter-safe controller decimal before transmission. Target-bearing raw `RJ`, `MG`, `MJ`, `ML`, `MV`, `WC`, and `WG` envelopes also validate J1-J9 fields present in the command against the active calibrated limits and signed step-counter arithmetic before serialization.
- Calibration persistence is debounced; failed writes retain dirty state and retry during normal operation. Main-controller port switching reserves the shared transport before close or open. Controller startup captures Tk-backed configuration before launch, aborts and closes the connection when serial buffer reset fails, rechecks cancellation after stale-input reset and before every startup write, consumes the firmware's unframed `UP` and `CE` acknowledgements through a bounded quiet boundary, normalizes CRLF on the framed `SP` acknowledgement, performs serial synchronization without worker-thread Tk access, and applies the validated result on Tk through scheduled polling. Normal and forced `SP` senders consume the complete framed `Done` acknowledgement and a quiet boundary; forced-pose synchronization then owns a separate framed `RP` exchange. Unavailable, disabled, or failed auxiliary startup closes and clears any pre-existing auxiliary handle before main-controller synchronization; failed cleanup aborts the startup result, retains the handle, and retries until closure succeeds. Failed main-controller cleanup likewise retains the port, activity lease, and serial reservation under a durable non-Tk retry owner when Tk scheduling is unavailable. Main-controller synchronization proceeds when auxiliary I/O is unconfigured or unavailable and reports the reduced connection state. Timeout dismisses the modal wait and cancels the connection attempt while transport ownership remains reserved until cleanup closes the failed connection. Application shutdown blocks new direct legacy serial operations, requests offline and online live-stop paths, and allows a bounded serial drain interval. Every closing poll applies queued terminal responses and evaluates overdue tracked reads before waiting for logical or virtual motion settlement, so a serial-backed motion owner cannot prevent `cancel_read` and the follow-up transport close. Retained startup-cleanup handles still settle before final calibration persistence. Shutdown persists the resulting confirmed position only after motion settlement and closes remaining serial ports; a failed final write keeps shutdown pending for another save attempt. Successful joint and live events restore an idle status or retain an explicit queued status. Verified shadowed helpers are consolidated, and idle VTK work is reduced.
- Main-controller replacement can enter while position recovery is required, retains logical, transport, and shutdown-activity ownership through asynchronous startup finalization or durable cleanup, and commits `CAL['comPort']` only after a fault-free startup position is applied. Auxiliary replacement or disablement has the same logical and shutdown admission boundary; the selected port and board remain staged until verified close/open and handle binding complete, and failed replacement restores the prior configuration selection. Collision correction remains queued without worker churn while the main transport is absent, closed, or quarantined. Program stop treats explicitly unconfigured auxiliary I/O as not required while configured-but-unavailable hardware remains a diagnostic failure.
- Successful auxiliary port and board-profile changes mark calibration state dirty for debounced persistence; failed persistence remains retryable and a restart restores the committed selection after a successful write. Mode changes and direct program controller exchanges acquire logical-motion ownership before main-transport ownership. Legacy main-controller transactions reset stale input before transmission and consume one bounded framed or exact response without a post-write buffer reset. Timing limits and acceleration/deceleration overlap use the exact binary32 values serialized for the controller. An acknowledged forced `SP` target remains the reconnect synchronization source after a failed or raised `RP` exchange and clears only after a later authoritative fault-free position response.
- Primary, external-axis, startup, and save/apply calibration paths stage and validate complete numeric commands before changing `CAL`, native kinematics, limit widgets, or persisted configuration. Ordinary startup requires a loaded calibration dictionary before application and fails closed with controller motion disabled when loading fails. Save/apply and custom-profile saving validate the complete merged J1-J9 current pose against the staged calibration, including editable J7-J9 positions, before local mutation, controller transmission, or persistence. Custom-profile loading validates every `UP`-owned field, every `CE`-owned field, and all calibration offsets, then validates the active J1-J9 pose against the resulting limits and step scales. Missing, corrupt, or non-object JSON fails the explicit profile load without substituting legacy/default calibration or terminating the application. Validated profile values are staged only in editable fields; active `CAL`, native kinematics, runtime limit labels and sliders, and controller calibration remain unchanged until Save/Apply succeeds. Motor directions, calibration directions, and calibration selections are binary; calibration offsets and saved numeric vision fields are finite. Native tool frames use `(x, y, z, rz, ry, rx)` order at every binding call. Failed validation leaves every destination unchanged. Controller-backed calibration preflights transport admission before local mutation and records the first write boundary; rejection before transmission restores the prior local snapshot. `UP` and `CE` transmission requires the firmware's exact bounded unframed `Done` acknowledgement before persistence. An exception after transmission starts, or failure of `CE` after acknowledged `UP`, retains intended local calibration, invalidates pending joint state, quarantines the main transport, blocks persistence, and requires reconnection. A failed close retains the quarantined handle for later cleanup. Persistence failure after both acknowledgements retains the applied controller-matched state and marks a debounced retry instead of restoring stale local calibration. Startup range-validates the returned J1-J9 pose against the staged combined calibration before local application. Failure after staged calibration application retains controller-matched calibration, leaves the saved auxiliary port unchanged, invalidates motion state, closes the connection, and requires reconnection. Automatic calibration validates and prepares both J1-J6 and J7-J9 stages before the first calibration write.
- Worker-run program motion acquires an exclusive request-scoped motion lease and reserves the matching virtual operation before physical dispatch, so failed virtual admission cannot leave an unowned physical move. The lease spans controller result application, virtual-operation settlement, and pose reconciliation; unrelated manual, live, G-code, offline, and joint-dispatch motion cannot enter during that interval. Controller-worker construction or startup exceptions become failed controller admission and still settle the admitted virtual operation. Before ownership release, a confirmed controller response overwrites the virtual preview, rejection before the physical write boundary restores the saved confirmed pose, offline failure restores the saved virtual pose, and post-write uncertainty or failed convergence activates the controller-position resynchronization block. Row completion waits for the matching applied controller result and virtual terminal result; controller failure, missing completion, failed virtual admission, virtual execution failure, or virtual timeout rejects the row. A missed deadline records failure but retains row ownership until late controller and virtual settlement, preventing a terminal row state from overlapping an active worker. Virtual-operation completion is published only after the matching virtual worker releases the drive lock, so row settlement never waits on a globally reacquirable lock. Virtual deadlines derive from the separate validated simulator envelope, shared timing validation, configured travel bounds, and simulator scaling; controller deadlines continue through the controller-only envelope. Reverse-step program motion combines physical completion with explicit virtual success, registers a Tk completion continuation, and never waits on the acknowledgement-held serial lock. Every simple `executeRow` movement form now enters the same request-scoped dispatcher. Online execution without VTK uses a controller-only sequence owner, while offline execution without an available virtual route rejects before physical transmission. Arc, circle, and spline rows reject without changing program selection because the tracked controller contracts are not safe to transmit. Busy program requests stop before program selection advances; busy manual Cartesian or tool-frame requests return before virtual dispatch. `.ar4` G-code playback transfers the parent row's main-transport reservation to the serial response worker and propagates terminal success or rejection to the row state without changing program navigation before completion. G-code start-position motion uses the manual-motion owner and omits the physical command while offline; playback rejects while offline. G-code conversion retries pending admission, halts on rejection or stop, and advances selection only after an explicit complete row result; manual forward stepping follows the same completion-only rule. G-code stop halts local row scheduling without main-transport admission and explicitly reports that active controller motion is not preempted. The host does not emit `SS\n` because no reachable host `SL` lifecycle establishes firmware spline mode. A missing Teensy SD playback file now emits `EG`, rotates the pending command buffer, and returns to normal command processing.
- G-code playback, deletion, and conversion validate one ASCII path component before constructing firmware commands. Feed values convert from active units per minute to millimetres per second, and imperial coordinates convert from the parsed axis value before start offsets are applied. `Tool Set` rows reject without transmission because the tracked Teensy v6.7.1 protocol has no `TF` handler.
- Virtual program completion preserves the original deadline when Tk scheduling falls back to a background owner. Timeout remains a failed result while ownership waits for the matching request-scoped operation to settle; failure to start the fallback worker retains the same settlement synchronously instead of publishing an early terminal row result.

Detailed findings and boundaries are tracked in `docs/hmi-optimization-audit.md`.

### M4A2 - HMI boundary completion

Status: `Proposed`

Acceptance criteria:

- Main-controller requests and responses share a central transport owner.
- Program execution, Cartesian jog, tool-frame jog, startup, camera, Modbus, calibration, and auxiliary-board paths perform no blocking work in Tk callbacks.
- Worker threads do not read or mutate Tk widgets.
- Discrete motion, program execution, and remaining stop paths have explicit preemption and response ownership; the live-jog path has that ownership now.
- Baseline and post-change event-loop, serial, render, and persistence timings are recorded.

### M4A3 - Delivered 7.0/2.0 baseline audit

Status: `Tested`

Acceptance criteria:

- Archive hashes and entry inventories are recorded before extraction into an isolated comparison directory outside the tracked worktree.
- HMI architecture, dependency and packaging changes, native kinematics boundaries, configuration schemas, host-controller protocols, and Teensy/Mega/Nano firmware changes are compared against the established baseline.
- Current HMI and safety changes are classified as superseded, directly portable, conflicting, or still required.
- Hardware-free checks validate relevant upstream behavior without importing either application entry point or writing to a controller.
- A scored recommendation selects selective integration, porting current changes onto the 7.0 host baseline, or full baseline replacement. No baseline replacement, controller write, calibration, or firmware flash occurs without an explicit follow-on decision.

Audit result:

- `docs/delivered-v7-baseline-audit.md` records the frozen archive intake, provenance, HMI and packaging review, host-controller contract comparison, firmware findings, native-kinematics review, configuration differences, current-change classification, validation limits, and scored recommendation.
- Selective integration into the current hardened baseline scored 4.25/5 and is approved. Porting current work onto the delivered host scored 2.85/5; full replacement scored 1.60/5.
- Delivered source remains isolated audit input. No application entry point or executable was run, and no controller write, calibration, firmware flash, or robot motion occurred.

### M4A4 - Selective 7.0/2.0 integration

Status: `Proposed`

Authorized scope:

- Port native inverse-kinematics validation, wrist-singularity continuity, configuration exposure, and deterministic tests before rebuilding supported native binaries.
- Add schema-validated MK5 calibration-switch polarity and numeric configuration normalization without importing machine-specific ports, limits, poses, or calibration values.
- Define a correlated JSON host-controller contract with one response owner, bounded timeouts, explicit event separation, and paired firmware fixtures before changing active command encoding.
- Correct Teensy JSON validation, Cartesian array bounds, G-code buffer rotation, and emergency-event ownership before compilation, simulation, or hardware consideration.
- Adapt the current semantic coalescer, confirmed-position rebasing, transport quarantine, and Tk result queues to the validated protocol rather than adopting the delivered blocking host transport.
- Port Mega/Nano 2.0 JSON capabilities while preserving explicit board profiles and profile-bound pin validation.
- Keep CAD and EOAT functionality optional and isolated behind validated file, scene, dependency, and lifecycle boundaries.

Acceptance criteria:

- Each integration unit lands independently through the cross-review gate and leaves the tracked baseline coherent.
- Host and firmware protocol changes land together with deterministic compatibility fixtures.
- Native changes expose supported configuration intentionally and pass boundary, singularity, wrapping, and no-solution tests.
- Configuration migration preserves current machine-neutral defaults and rejects malformed, non-finite, or out-of-range values before mutation.
- Firmware sources compile and pass hardware-free protocol checks before any requested flash or live test.
- No worker reads or mutates Tk state, and no newly routed Tk callback waits on serial I/O.
- Optional CAD/EOAT behavior can remain disabled without importing CadQuery or changing application startup.
- Hardware verification follows M5 and cannot be inferred from compilation, simulation, or fake transports.

### M4B - Repeatability and dynamic interception pass

Status: `Proposed`

Research direction:

- Timestamped perception and calibrated frame transforms.
- State estimation with position, velocity, acceleration, and covariance.
- Future-state prediction and reachable intercept selection.
- Global planning plus fast local replanning or visual servo control.
- Synchronized velocity, acceleration, and jerk constrained trajectory generation.
- A non-blocking controller interface supporting replaceable setpoints, state feedback, hold, cancel, watchdog, and fault states.

Acceptance criteria:

- Accuracy, repeatability, latency, speed, payload, workspace, and grasp thresholds are measurable.
- A deterministic simulator and recorded-observation replay path exercise estimator, predictor, feasibility, and replanning behavior.
- Stale data, uncertainty, collision, singularity, joint-limit, controller-timeout, and failed-grasp behavior are explicit.
- Hardware experiments follow M5 and distinguish observed results from simulation.

Research and staged decisions are tracked in `docs/dynamic-motion-research.md`.

### M5 - Controlled hardware validation

Status: `Proposed`

Acceptance criteria:

- Authorized procedure identifies firmware, configuration, start pose, speed limits, expected motion, abort conditions, and recovery steps.
- Physical emergency stop is tested before commanded movement.
- Results distinguish observed hardware behavior from software-only evidence.
- Deviations become tracked requirements or defects before broader operation.

## Architectural decisions

- Preserve the current directory layout until startup and asset-path dependencies are isolated.
- Keep mutable calibration, captured images, error logs, and review artifacts outside version control.
- Treat host commands, firmware parsers, native kinematics bindings, and `.ar4` programs as versioned integration contracts.
- Queue semantic targets, not raw input events. Joint, Cartesian, and tool-frame intents cannot be merged across coordinate spaces without recomputation from confirmed state.
- Keep desktop command coalescing separate from future real-time servo and trajectory-control loops.
- Retain the current hardened baseline and use delivered 7.0/2.0 sources only as isolated selective-integration input under M4A4.
- Route post-bootstrap commits through the role-appropriate cross-review wrapper.
- Route branch integrations through `scripts/codex/auto-merge.ps1`; bare merge into the integration base is prohibited.

## Current implementation boundary

- Incremental J1-J9, absolute slider routing, offline external-axis rejection, and live-jog desktop wiring are statically or behaviorally checked without importing `AR4.py`; dispatcher coalescing, deferred intent ordering, controller and six-axis simulator command schemas, framed responses, optional protocol-authorized follow-up frames, full quiet-boundary acknowledgements, trailing-byte quarantine, retained failed-close ownership, configured Linux and Windows Xbox validation, watchdog scheduler failure, startup acknowledgement variants, startup finalizer rejection, set-position acknowledgement sequencing, live-stop injection and request ownership, pre-write cancellation admission, bounded-command timeout quarantine and restoration, cancellation-bound G-code playback, request-scoped virtual results, late virtual settlement, virtual-worker failures, program-worker startup failure, long Seconds-mode virtual deadlines, direct program and calibration response propagation, calibration transport preflight and write-boundary recovery, binary and finite calibration validation, combined automatic-calibration preflight, native tool-frame order, offline position synchronization, forced calibration-position rejection, completion-only G-code navigation, legacy and dispatcher transport-release ordering, shutdown waits for virtual and retained cleanup owners, nested direct-operation transport admission, asynchronous worker reservation transfer and rollback, direct-operation shutdown tracking, applied program-motion results, G-code local stop admission, post-release collision correction, interruptible auxiliary-wait stop handoff with a single shared post-transmission acknowledgement deadline, optional auxiliary startup, controller-buffer-reset failure handling, combined close-and-scheduler cleanup failure, startup timeout handling, non-preempting program-halt status, and response ownership are tested with deterministic fakes.
- Simple program movement routes now share request-scoped controller ownership, raw target validation, and offline physical-write rejection. The broader typed program state machine, non-motion row migration, Cartesian and tool-frame coalescing, complete main-controller response ownership, application lifecycle separation, and dynamic controller work remain incomplete.
- Tracked Teensy v6.7.1 `MA` and `MC` parsing writes the `Tr` field to `xyzuvw_In[6]` even though `ROBOT_nDOFs` defines a six-element array. Host arc and circle program transmission remains disabled until a paired firmware correction, compilation, simulation, and authorized hardware-validation plan exist. Spline program transmission also remains disabled until one terminal response-owner contract replaces speculative acknowledgements.
- Teensy coordinated pulse scheduling and E-stop response ownership were inspected but not changed. The trusted v6.7.1 E-stop protocol can race speculative spline responses and remains unsafe for claimed single-frame ownership; remediation requires a separate protocol design and hardware-validation plan. The missing-file behavior has a source-contract assertion only; compilation, simulation, and live-arm verification remain pending.
- M4A3 found that delivered 7.0 retains busy-input loss, blocking Tk paths, incomplete response ownership, firmware array-bound defects, and emergency-event races. Selective integration is approved because delivered inverse-kinematics, MK5 calibration-switch, JSON auxiliary-controller, and optional CAD/EOAT changes can be isolated without replacing the hardened HMI baseline.
- No live-arm command, firmware flash, calibration cycle, or movement was performed.
