# AR4HMI Plan

## Objective

Adapt the existing AR4 control stack into a maintainable, hardware-safe system for a defined robotic-arm application. Initial priorities cover HMI responsiveness, maintainability, coordinated motion, and safe interaction during repositioning; hardware and workflow constraints remain incomplete.

## Authority

This file defines project scope, status, acceptance criteria, and architectural decisions. Implementation claims must match the repository. Hardware claims require recorded live-arm evidence.

## Status vocabulary

- `Proposed`: scope described; implementation not started.
- `In progress`: implementation has started; required integration or verification remains.
- `Implemented`: repository implementation exists; required verification may remain.
- `Tested`: automated checks exercise the stated behavior without attached hardware.
- `Hardware-verified`: an authorized live-arm procedure produced recorded evidence.
- `Blocked`: progress requires a named decision, dependency, permission, or external state change.

## Established baseline

- The uniquely titled `chore: import AR4 control software baseline` commit,
  dated `2026-07-15`, preserves the imported control-software baseline across
  both the original and publication histories.
- The uniquely titled `chore: install cross-review gate` commit, dated
  `2026-07-15`, introduced the tracked cross-review gate. Repository-root
  `bootstrap.ps1` installs the per-clone pre-commit dispatcher.
- `AR4.py` identifies host source version 6.7. The imported Teensy baseline identifies version 6.7.1; the current tracked derivative identifies version `6.7.1-ar4hmi.9` and advertises `JT_WRIST_CONFIG_V1`, `GCODE_DIRECTORY_FRAMING_V1`, `GCODE_DELETE_IDENTITY_V1`, `GCODE_WRITE_IDENTITY_V1`, `HOME_REFERENCE_V1`, `HOME_REFERENCE_V2`, `JOINT_TELEMETRY_V1`, and `ESTOP_ADMISSION_V1`. The latest dated deployment record identifies controller version `6.7.1-ar4hmi.5`; `.9` deployment remains unverified.
- Runtime calibration and machine state remain untracked; `defaults.json` remains the tracked default profile.
- No live-arm command, firmware flash, calibration cycle, or movement was performed during repository setup.

## Safety invariants

- Automated checks must not open serial ports or issue controller commands.
- Host startup must become separable from GUI construction and hardware connection before import-based testing.
- Mechanical self-protection is the primary motion-safety objective: ordinary motion must fail closed when homing, calibration, encoder, direction, joint-limit, singularity, or controller state is untrusted; recovery and homing paths require explicit bounded semantics.
- Hardware-validation plans must cover overtravel, self-collision, stall or encoder-collision response, cable-wrap limits, and direction-sign correctness before broader motion trials.
- Hardware verification must confirm that the physical emergency-stop path operates independently from desktop GUI state before commanded movement.
- Motion-protocol changes require synchronized host and firmware updates.
- Joint limits, calibration units, saved pose handling, encoder modes, and stop behavior require explicit tests before hardware verification.
- J1/J2/J3 homing verification must confirm switch release within the lesser of
  the configured ten-unit bound and full configured axis travel, and reference
  availability only after the centered move completes.
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
- Incremental J1-J9 input recorded during an owned move retains semantic delta and absolute-target ordering, then dispatches a consolidated final multi-axis target from confirmed controller state.
- A matching request-scoped owner spans controller-result application and virtual-operation settlement, preventing unrelated motion admission during partial completion.
- Public callbacks that change controller position, calibration, or port selection require logical-motion admission before transport admission. `requestPos` and main-controller replacement remain explicit position-recovery admissions.
- Manual online motion restores the last confirmed virtual pose only when physical transmission never starts. Transmission uncertainty blocks new operator and program motion until a fault-free controller position response resynchronizes the virtual model and joint dispatcher; controller fault correction remains an explicit recovery-only exception.
- Serial and calibration boundary failures reject, quarantine, or retain explicit recovery state without silently advancing confirmed motion or persisted calibration state.
- Deterministic hardware-free checks cover host coalescing, ownership ordering, controller and six-axis simulator timing boundaries, deadline propagation, write-boundary recovery, E-stop response admission, and failure settlement. The missing-file firmware branch has a source-contract check, and the tracked Teensy source compiles for Teensy 4.1 with PJRC core 1.62.0 and ModbusMaster 2.0.1. The dated no-upload build evidence is recorded in [`docs/hardware-free-verification-2026-07-19.md`](docs/hardware-free-verification-2026-07-19.md). Live verification remains pending.

Implemented HMI work:

- Incremental J1-J9 inputs share a validated semantic target dispatcher.
- Main-controller joint commands are serialized while later button deltas and absolute slider targets replace the latest pending multi-axis target. Tracked direct main-controller transactions reserve the same nesting-safe transport admission lock, and asynchronous program motion transfers that reservation to the response worker without opening an interleaving window.
- Joint inputs made during an active legacy move preserve ordered delta and absolute-target semantics, rebase relative changes on the next confirmed controller position, and dispatch a consolidated multi-axis target. Main-thread terminal-result processing keeps transport admission reserved through response application, then releases the matching operation before retrying retained input. Input recorded after a valid response can use that confirmed generation at the release boundary, while unknown controller state blocks dispatch until resynchronization. A transient transport-busy result retains deferred intent; permanent validation or queue rejection clears stale intent instead of retrying indefinitely.
- Deferred joint resolution, dispatcher admission, and consumed-state clearing share synchronized locking. Concurrent input is included in the admitted snapshot or remains pending for the next confirmed generation, and dispatcher worker startup completes or rolls back before another submission can receive acceptance.
- Controller errors, malformed responses, timeouts, and embedded motion faults discard pending motion and require position resynchronization. Automatic calibration requires a terminal controller disposition after a committed write because the line-oriented `LL` protocol has no host abort command. Every failed `LL` motion stage emits `ER`; when a physical stop occurs during the owned response, the host consumes the bounded `ER` terminal and `EB` event pair before releasing calibration transport ownership. Shutdown and calibration write admission share an atomic boundary: shutdown before write commitment rejects transmission, while a committed write latches calibration transport and motion ownership through an applied terminal response or explicit post-write quarantine and verified-close handling. Any post-write framing failure, extra frame, non-position response, controller fault, or result-application failure blocks later calibration stages. Result-application rejection restores the captured pre-command pose and generation, cancels changed debounced persistence, schedules the restored state or retains dirty state after scheduler failure, then quarantines the transport before releasing ownership.
- Owned line exchanges classify controller opcodes before parsing fields. Motion opcodes require complete opcode-specific envelopes, so timing-like payload text cannot replace or duplicate motion timing. Controller-bound `RJ` and `MJ` envelopes require J7-J9, while simulator-bound `RJ` and `MJ` use separate exact J1-J6 and Cartesian envelopes and legacy `JT` retains an explicit simulator contract. Bounded commands validate encoded speed, acceleration, deceleration, and ramp fields, reject acceleration and deceleration regions whose combined share exceeds the move, then derive finite response deadlines from validated timing and configured full-travel bounds. `PG` playback has no duration contract in the current firmware and therefore retains response ownership without a fixed terminal deadline; application shutdown cancels the host exchange and quarantines the transport, without claiming physical-motion preemption. Live-jog start acknowledgement uses `SERIAL_LIVE_ACK_TIMEOUT_SECONDS`; completion after the stop token reuses the command-specific full-travel deadline, while the deadline remains suspended during an acknowledged hold. Live-jog and playback cancellation admission is rechecked under the serial write lock after the bounded pending-stop probe and without discarding unread input, so cancellation before transmission writes neither the motion command nor a fail-safe stop token. Any premature terminal data, framing failure, or timeout-state failure after transmission writes the fail-safe stop token where possible, quarantines the main serial transport before ownership release, attempts a verified close, and requires explicit reconnection. A failed close retains the poisoned handle for later cleanup instead of dropping application ownership. Temporary exchange-timeout restoration is attempted across exit paths, and restoration failure triggers the same quarantine.
- Owned framed responses remove only a required LF and an optional preceding
  CR. Response validation separates the 4096-byte payload limit from the
  4098-byte CRLF-capable frame limit, so a maximum-length payload remains valid
  with either supported delimiter. Payload padding, additional delimiters,
  non-ASCII data, and queued data outside an operation-specific follow-up
  contract are protocol failures; terminal readers require a complete bounded
  quiet interval before releasing ownership and quarantine the transport when
  the response deadline cannot provide that interval. Host response parsing
  distinguishes controller-initiated `EB` physical-stop events from correlated
  `EA` command-admission rejections. The tracked Teensy
  `6.7.1-ar4hmi.9` derivative atomically checks the physical-stop latch and
  input at the queue boundary and again after side-effect-free opcode
  extraction. A loop-scoped response owner brackets every ordinary,
  admission, and telemetry terminal writer. The E-stop interrupt records
  assertion state and pending output without writing USB serial data; main-loop
  code emits pending `EB` only after the current terminal frame or at an empty
  loop boundary. Controller and Modbus wait delays poll the E-stop latch and
  return from the active response scope after consuming the command. A
  pre-terminal interruption emits `ER` before the scoped `EB`; a
  post-terminal interruption publishes `EB` without another terminal. Modbus
  transactions remain bounded by the selected library before the next latch
  poll. Automated coverage and compilation do not establish live response
  latency. A blocked command reserves the correlated `EA` response
  against pending `EB` publication. Admission and loop-response ownership
  retire together with interrupts disabled, and a released latch clears only
  when no newer interrupt generation was recorded; an asserted or newly
  reasserted stop keeps rejecting commands. Telemetry-enabled `RJ` requests retain
  specialized response ownership from before coordinated drive through
  terminal framing. A stop deferred after terminal selection remains an
  admission block and emits an immediate post-terminal `EB`. The telemetry
  joint owner accepts standalone `EB`; `LL` emits an `ER` terminal after every
  E-stop abort, and other active joint and legacy owners require `EB` to pair
  with a terminal or correlated `EA`. The joint dispatcher and shared legacy
  wrapper preserve unread input,
  classify `EA` and `EB` before operation-specific interim callbacks, and own
  ordinary terminals, standalone `EA`, and paired terminal/`EB` orders. An
  acknowledged `LC`, `LJ`, or `LT` exchange derives a finite post-terminal
  probe deadline from the configured control-response bound before rejecting
  an unrequested ordinary terminal. A paired `EB` is published and
  authoritative; an absent or invalid pair triggers fail-safe `S` and
  transport quarantine. An
  immediately available `EA` after `EB` is consumed by the same owner; an
  `EB` without a follow-up byte in the bounded probe is ambiguous, so the
  telemetry joint owner quarantines and verifies closure before releasing
  ownership. While no command owns the main transport, a
  zero-write monitor checks the local queued-byte count and consumes only a
  standalone `EB` under exclusive emergency ownership. The worker publishes
  and quarantines the stop before transferring a result to Tk; Tk releases the
  emergency transport owner before the separately queued stop presentation,
  while shutdown blocks final close until queued or retained presentation
  settles. The monitor remains admissible while shutdown drains; another idle
  frame quarantines and closes the connection. Each accepted physical-stop
  exchange snapshots the complete auxiliary configuration under the shared
  configuration/stop-state lock order while atomically latching shared stop
  state, cancelling queued manual auxiliary work, blocking new manual
  auxiliary admission, and reserving a request-correlated auxiliary `STOP`
  before Tk publication. Auxiliary configuration commits use the same lock
  order and reject while an E-stop, position fault, or stop settlement remains
  active, without making physical-stop latching wait for auxiliary transport
  ownership. A failed physical-stop
  `STOP` attempt restores the same request to pending state; configured stop
  ownership remains admissible during shutdown draining, final serial closure
  waits for pending or active ownership and queued or retained stop
  presentation, and acknowledgement releases that request owner.
  Main-controller pose and status events remain retained for retry when Tk
  rendering or motion invalidation raises, and joint dispatcher fault
  settlement does not apply the same stop again. Main-controller stop events
  from joint, legacy, and G-code storage exchanges carry the source connection
  epoch and confirmed-position generation. A stale prior-session presentation
  is retired with a diagnostic and cannot mutate current controller latches.
  Manual Modbus stop events remain request-correlated. Uncertain framing
  requires reconnection. Host startup requires `ESTOP_ADMISSION_V1`, preventing
  the revised framing contract from running against older firmware.
  Speculative spline acknowledgements from `processSerial` remain a multi-frame
  protocol surface; unmatched combinations quarantine the transport.
  Compilation and hardware-free contract verification cover the revised
  admission path; authorized live-arm verification remains pending.
- Serial worker terminal results reach Tk through event queues. The matching transport and shutdown-activity lease remain reserved until Tk applies the operation-local result and releases transport admission. The matching callback then settles any request-scoped motion owner before deferred input is retried. Joint-dispatch and startup leases likewise span worker execution through terminal acknowledgement or connection cleanup. Parsed M1 speed-violation metadata survives serial, manual, startup, calibration, and joint completion; ready or queued status cannot overwrite the warning. Joint M1 completion discards queued and deferred targets, rebases desired state on the confirmed controller position, and leaves later input admissible from that position.
- Controller fault rendering queues recovery instead of performing controller I/O on Tk. Collision correction waits for main-transport release before dispatching `CP\n`. Auxiliary program stop uses request-correlated pending, dispatched, completed, and failed states. Only the firmware's interruptible `WI` wait permits write-only `STOP\n` injection while the wait worker retains sole response ownership. `Nano Stopped` acknowledges interruption directly. Immediately after successful `STOP\n` transmission, a shared absolute monotonic deadline based on `SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS` replaces the original `WI` deadline. Wait-owner handoff, optional owner-side follow-up, serial-lock acquisition, stop-worker follow-up read, and the required quiet boundary all consume that same deadline. When racing natural `Done` or `Timeout` and `Nano Inactive Stopped` frames are already queued, the wait owner validates both before release and publishes the inactive acknowledgement; when the second frame arrives later, the stop worker consumes the bounded follow-up after ownership release. Missing, unexpected, or additional follow-up data quarantines the auxiliary transport. Program halt status explicitly reports that active main motion is not preempted. Other auxiliary operations retain a pending stop request until an exclusive exchange returns the expected inactive-stop acknowledgement. `WI` timeouts are constrained to the firmware integer range, receive an encoded-timeout-plus-margin read deadline, and accept only documented terminal responses. Servo and output rows consume bounded unframed acknowledgements, require a quiet read boundary after the expected bytes, and quarantine empty, partial, unexpected, or trailing data. Legacy auxiliary commands reset stale input before transmission and share the stop-token write lock, preventing byte interleaving or post-write response erasure. Windows and Linux Xbox gripper paths share the bounded validated exchange owner. Every program, manual, and Xbox `SV`/`ON`/`OF` command requires an explicit Nano or Mega profile bound to the active auxiliary serial handle; an unselected or stale profile blocks transmission. Servo commands enforce channels 0-6 and positions 0-180. Nano accepts outputs 8-13, Mega accepts outputs 28-53, and each profile rejects the complete opposing range. Windows pneumatic toggles use profile-valid output 8 for Nano and 28 for Mega. Closing or replacing the handle clears the binding, and existing configurations default to no selected board until the operator chooses a profile. Windows toggle state remains pending until the expected acknowledgement returns through the Tk event queue; transport rejection preserves the last acknowledged state, while an invalid exchange marks state unknown and attempts a verified auxiliary close. Linux performs the same exchange synchronously in the controller worker and changes local grip state only after acknowledgement.
- Live joint, Cartesian, and tool jog callbacks use an interruptible worker-owned request. Button release sets a stop event without sleeping or reading serial data on Tk; the worker writes the firmware stop token and remains the sole response reader. Live modes normalize Seconds and millimetres-per-second selections to Percent because firmware accepts stop input only between segments. Offline live workers validate the complete command through the shared host parser and forward the encoded speed, acceleration, deceleration, and ramp profile to the virtual drive. Offline live workers reserve a shared request slot before launch, synchronize stop against segment admission, and wait for each request-scoped virtual result. The request-scoped operation remains authoritative until matching Tk settlement, so rejected or invalid later input cannot clear active state and button release still signals the active owner even if the display flag is stale. Tk performs the final virtual-state snapshot only after terminal ownership settlement, so a later press cannot reactivate released work or overlap an unfinished segment. Virtual live-jog errors return through a Tk event queue.
- Windows Xbox joint, Cartesian, and tool intent uses generation-scoped admission arbiters. Held intent remains pending after a busy rejection, active state changes only after the matching Tk live-jog callback accepts the same semantic value, and failed stop or scheduling admission retains pending intent with an explicit diagnostic. A failure from the deferred inner Tk registration clears the scheduled-attempt state before reporting the diagnostic, allowing later held input to retry. Application-closing cancellation before or after Tk admission clears pending scheduling without a false alarm. The Xbox Teach callback also records Tk scheduling rejection. Watchdog scheduler failure disables Windows polling, signals online and offline live-stop events, requests every arbiter to stop, and reports an alarm. Controller polling stops during application shutdown; Linux drops events returned after controller-off or shutdown state becomes active.
- Online joint targets coalesce into the virtual target and confirmed controller responses resynchronize the virtual model. A confirmed joint result that cannot update the virtual model invalidates the pending dispatcher state before response acknowledgement, retains the synchronization warning, and blocks further motion until a later fault-free position response resynchronizes both models. Manual online Cartesian and tool-frame motion captures the last confirmed virtual pose before preview dispatch. A failure before the physical write boundary restores that pose; a failure after transmission starts preserves uncertainty and blocks new operator and program motion until a fault-free controller position response resynchronizes the virtual model and joint dispatcher. Controller fault correction remains an explicit recovery-only exception. Physical success applies the confirmed controller pose even after virtual preview failure. Online live jog no longer launches a competing virtual drive worker; offline J7-J9 input is rejected because the virtual model supports J1-J6 only.
- Simulator `MJ` millimetres-per-second timing captures a validated six-value Cartesian start pose from forward kinematics and a validated target pose before virtual worker admission. Translation distance derives from those explicit endpoints; missing, malformed, or non-finite endpoints reject before virtual motion state changes.
- Online/offline mode changes reject every tracked physical or virtual motion owner. Offline-to-online synchronization validates and applies the controller position before mode, button, status, or virtual-position state commits; failure preserves the offline snapshot.
- Offline manual, joint, program, and live settlement retains request ownership while applying the terminal six-joint pose or restoring the saved pose after failure. FK-derived calibration and widgets refresh within that interval; refresh failure rejects settlement. Virtual drive workers do not perform Tk refresh work.
- Position parsing validates the complete response before state mutation. The accepted frame requires ordered A-R markers, delimiter-safe decimal position fields, the firmware speed bit, an empty or numeric debug field, and a blank, `EA`, `EB`, or six-bit `EC` fault field; duplicated markers and arbitrary payloads are rejected before calibration, widget, or persistence mutation. Every inbound J1-J9 position, including dispatcher-owned, automatic-calibration, and fault-flagged responses, must fit the applicable calibrated axis limits and signed step-counter range before dispatcher state, `CAL`, position-dependent widgets, persistence state, virtual state, or position-fault rendering can change. External position responses must resynchronize an idle dispatcher before calibration state or position generations advance; dispatcher-owned completion events bypass only that external synchronization. Direct program rows, automatic and single-joint calibration, position requests, offline-to-online synchronization, and forced calibration poses advance success state or copy virtual positions only after a fault-free response is applied. Controller errors, malformed data, embedded motion faults, and dispatcher rejection propagate as operation failure. Every numeric field in a supported raw motion envelope, timing profile, and numeric suffix is captured, binary32 range-checked, and converted to delimiter-safe controller decimal before transmission. Target-bearing raw `RJ`, `MG`, `MJ`, `ML`, `MV`, `WC`, and `WG` envelopes also validate J1-J9 fields present in the command against the active calibrated limits and signed step-counter arithmetic before serialization.
- Calibration persistence is debounced; failed writes retain dirty state and retry during normal operation. Main-controller port switching reserves the shared transport before close or open. Controller startup captures Tk-backed configuration before launch, aborts and closes the connection when the initial serial buffer reset fails, and never resets unread input between startup commands. Every unbound startup exchange performs a bounded pending-stop probe, rechecks cancellation before write, and consumes a stop-aware exact or framed response; queued or paired `EA` and `EB` frames latch and publish the stop before startup retry or failure handling. The worker consumes the firmware's unframed `UP` and `CE` acknowledgements through a bounded quiet boundary, normalizes CRLF on the framed `SP` acknowledgement, performs serial synchronization without worker-thread Tk access, and applies the validated result on Tk through scheduled polling. Normal and forced `SP` senders consume the complete framed `Done` acknowledgement and a quiet boundary; forced-pose synchronization then owns a separate framed `RP` exchange. Unavailable, disabled, or failed auxiliary startup closes and clears any pre-existing auxiliary handle before main-controller synchronization; failed cleanup aborts the startup result, retains the handle, and retries until closure succeeds. Failed main-controller cleanup likewise retains the port, activity lease, and serial reservation under a durable non-Tk retry owner when Tk scheduling is unavailable. A stale main-controller identity cleanup request preserves a newer handle-bound identity, emits a stable invariant diagnostic, and is propagated or terminally recorded by every cleanup caller after owned resources are released. Main-controller synchronization proceeds when auxiliary I/O is unconfigured or unavailable and reports the reduced connection state. Timeout dismisses the modal wait and cancels the connection attempt while transport ownership remains reserved until cleanup closes the failed connection. Application shutdown blocks new direct legacy serial operations, requests offline and online live-stop paths, and allows a bounded serial drain interval. Every closing poll applies queued terminal responses and evaluates overdue tracked reads before waiting for logical or virtual motion settlement. Overdue non-calibration readers and pre-write calibration activity receive `cancel_read` and a follow-up transport close; committed calibration instead retains application supervision until an applied terminal controller frame or explicit post-write quarantine and verified-close handling settles the operation. Retained startup-cleanup handles still settle before final calibration persistence. Shutdown persists the resulting confirmed position only after motion settlement and closes remaining serial ports; a failed final write keeps shutdown pending for another save attempt. Successful joint and live events restore an idle status or retain an explicit queued status. Verified shadowed helpers are consolidated, and idle VTK work is reduced.
- Shutdown close readiness also requires the generic main-controller stop
  presentation queue and retained presentation slot to be empty before final
  persistence or serial closure.
- Main-controller replacement can enter while position recovery is required, retains logical, transport, and shutdown-activity ownership through asynchronous startup finalization or durable cleanup, and commits `CAL['comPort']` only after a fault-free startup position is applied. Auxiliary replacement or disablement has the same logical and shutdown admission boundary; the selected port and board remain staged until close/open and handle binding complete. Transport replacement, live calibration, and UI publication share the physical-stop critical section, so a stop admitted before the commit preserves the prior binding and blocks replacement while a later stop observes the verified replacement. Failed replacement restores the prior configuration selection. Any retained auxiliary handle remains stop-required even when persisted configuration is missing or damaged. Auxiliary configuration rejects while an E-stop, position fault, or stop settlement remains active. Collision correction remains queued without worker churn while the main transport is absent, closed, or quarantined. Program stop treats explicitly unconfigured auxiliary I/O as not required while configured-but-unavailable hardware remains a diagnostic failure.
- Successful auxiliary port and board-profile changes mark calibration state dirty for debounced persistence; failed persistence remains retryable and a restart restores the committed selection after a successful write. Mode changes and direct program controller exchanges acquire logical-motion ownership before main-transport ownership. Legacy main-controller transactions perform a bounded pre-write physical-stop probe without resetting unread input, then consume a stop-aware bounded framed or exact response. Timing limits and acceleration/deceleration overlap use the exact binary32 values serialized for the controller. An acknowledged forced `SP` target remains the reconnect synchronization source after a failed or raised `RP` exchange and clears only after a later authoritative fault-free position response.
- Primary, external-axis, startup, and save/apply calibration paths stage and validate complete numeric commands before changing `CAL`, native kinematics, limit widgets, or persisted configuration. Ordinary startup requires a loaded calibration dictionary before application and fails closed with controller motion disabled when loading fails. Save/apply and custom-profile saving validate the complete merged J1-J9 current pose against the staged calibration, including editable J7-J9 positions, before local mutation, controller transmission, or persistence. Custom-profile loading validates every `UP`-owned field, every `CE`-owned field, and all calibration offsets, then validates the active J1-J9 pose against the resulting limits and step scales. Missing, corrupt, or non-object JSON fails the explicit profile load without substituting legacy/default calibration or terminating the application. Validated profile values are staged only in editable fields; active `CAL`, native kinematics, runtime limit labels and sliders, and controller calibration remain unchanged until Save/Apply succeeds. Motor directions, calibration directions, and calibration selections are binary; calibration offsets and saved numeric vision fields are finite. Native tool frames use `(x, y, z, rx, ry, rz)` order at every binding call; display and firmware wire fields remain `(x, y, z, rz, ry, rx)` and are converted at their boundaries. Failed validation leaves every destination unchanged. Controller-backed calibration preflights transport admission before local mutation and records the first write boundary; rejection before transmission restores the prior local snapshot. `UP` and `CE` transmission requires the firmware's exact bounded unframed `Done` acknowledgement before persistence. An exception after transmission starts, or failure of `CE` after acknowledged `UP`, retains intended local calibration, invalidates pending joint state, quarantines the main transport, blocks persistence, and requires reconnection. A failed close retains the quarantined handle for later cleanup. Persistence failure after both acknowledgements retains the applied controller-matched state and marks a debounced retry instead of restoring stale local calibration. Startup range-validates the returned J1-J9 pose against the staged combined calibration before local application. Failure after staged calibration application retains controller-matched calibration, leaves the saved auxiliary port unchanged, invalidates motion state, closes the connection, and requires reconnection. Automatic calibration validates and prepares both J1-J6 and J7-J9 stages before the first calibration write.
- Worker-run program motion acquires an exclusive request-scoped motion lease and reserves the matching virtual operation before physical dispatch, so failed virtual admission cannot leave an unowned physical move. The lease spans controller result application, virtual-operation settlement, and pose reconciliation; unrelated manual, live, G-code, offline, and joint-dispatch motion cannot enter during that interval. Controller-worker construction or startup exceptions become failed controller admission and still settle the admitted virtual operation. Before ownership release, a confirmed controller response overwrites the virtual preview, rejection before the physical write boundary restores the saved confirmed pose, offline failure restores the saved virtual pose, and post-write uncertainty or failed convergence activates the controller-position resynchronization block. Row completion waits for the matching applied controller result and virtual terminal result; controller failure, missing completion, failed virtual admission, virtual execution failure, or virtual timeout rejects the row. A missed deadline records failure but retains row ownership until late controller and virtual settlement, preventing a terminal row state from overlapping an active worker. Virtual-operation completion is published only after the matching virtual worker releases the drive lock, so row settlement never waits on a globally reacquirable lock. Virtual deadlines derive from the separate validated simulator envelope, shared timing validation, configured travel bounds, and simulator scaling; controller deadlines continue through the controller-only envelope. Reverse-step program motion combines physical completion with explicit virtual success, registers a Tk completion continuation, and never waits on the acknowledgement-held serial lock. Every simple `executeRow` movement form now enters the same request-scoped dispatcher. Online execution without VTK uses a controller-only sequence owner, while offline execution without an available virtual route rejects before physical transmission. Arc, circle, and spline rows reject without changing program selection because the tracked controller contracts are not safe to transmit. Busy program requests stop before program selection advances; busy manual Cartesian or tool-frame requests return before virtual dispatch. `.ar4` G-code playback transfers the parent row's main-transport reservation to the serial response worker and propagates terminal success or rejection to the row state without changing program navigation before completion. G-code start-position motion uses the manual-motion owner and omits the physical command while offline; playback rejects while offline. G-code conversion retries pending admission, halts on rejection or stop, and advances selection only after an explicit complete row result; manual forward stepping follows the same completion-only rule. G-code stop halts local row scheduling without main-transport admission and explicitly reports that active controller motion is not preempted. The host does not emit `SS\n` because no reachable host `SL` lifecycle establishes firmware spline mode. A missing Teensy SD playback file now emits `EG`, rotates the pending command buffer, and returns to normal command processing.
- G-code storage, conversion, and program execution share lock-protected
  admission boundary. An operator `RG` or `DG` request blocks Run, Step
  Forward, Step Reverse, and conversion admission through worker completion,
  Tk result application, and transport and motion release. Program ownership
  blocks storage admission before serial or motion acquisition. Conversion
  blocks ordinary logical-motion, mode-change, program-start, storage, and
  controller-replacement admission from the pre-row delete through worker
  settlement; scoped conversion deletion may enter while conversion state is
  active. Storage cleanup stops at the first failed release and
  retains the remaining activity, transport, logical-motion, request, and
  shared-admission ownership under a background retry owner. Conversion
  settlement occurs only after that retained cleanup completes. Native
  kinematics readiness is preflighted before deletion.
  After delete settlement, the conversion worker inherits an exclusive
  logical-motion lease, revalidates stop, shutdown, conversion, and lease
  state at worker entry, and releases the lease before conversion admission
  clears. Final G-code row write admission is ordered against local Stop and
  shutdown through the conversion cancellation lock.
- G-code playback, deletion, and conversion validate an ASCII path component
  before constructing firmware commands. Controller-directory reads and
  deletion use a request-scoped serial worker and correlated Tk result
  application; successful deletion refreshes the listing only after ownership
  release and only when the request-captured local-view generation remains
  current. The local program-path field is read-only, and a completed
  local-file load atomically replaces the path, rows, selection display, and
  view generation. Local loading uses nonblocking descriptor admission before
  accepting only a regular file, reads every source row through
  `MAX_LOCAL_GCODE_SOURCE_LINE_BYTES`, enforces
  `MAX_LOCAL_GCODE_PROGRAM_BYTES` and `MAX_LOCAL_GCODE_PROGRAM_ROWS` before
  accumulation, and accepts only tab or printable ASCII bytes from `0x20`
  through `0x7E` before whitespace canonicalization. Blank and comment-only
  rows are discarded, while repeated actionable rows remain distinct. A
  source with no actionable row rejects before view replacement or
  controller-file deletion. Conversion preserves an explicit row selection
  and starts at row 0 when no selection exists. Stale asynchronous listings
  cannot replace that newer loaded view. Replacement rollback restores rows,
  per-row colors, selection, active and anchor rows, scroll position, program
  path, exact program-path widget state, and current-row display. Operator
  storage and local loading reject during active G-code conversion, while the
  conversion-owned completion callback provides scoped storage admission for
  pre-row deletion. Storage also rejects during program-level ownership, and
  every program mode rejects without execution-state or selection mutation
  while an operator storage request owns asynchronous settlement. `RG` accepts
  only an SD-CID-prefixed directory payload or a printable detailed storage
  error.
  `DG` carries the matching SD CID; `P`, `F`, and `ER` are definitive terminals,
  while a printable detailed error after transmission leaves deletion
  indeterminate. Every unmatched frame quarantines the transport. A
  controller-initiated physical-E-stop
  frame during `RG` or `DG` also quarantines the transport and triggers a
  verified-close attempt because the interrupt handler can resume the storage
  command and emit another frame. A standalone correlated `EA` proves that
  command execution was rejected, clears a pending delete journal, and still
  closes the transport before ordinary result cleanup. A failed close leaves
  the handle quarantined for explicit reconnection. Only a cleanup-release
  failure activates the retained retry owner. Storage never treats a position
  frame as a terminal response. Startup
  binds storage work to the Teensy hardware
  identity. Firmware SD initialization binds the mounted volume to the
  formatted card CID, probes current media before mount reuse, and remounts
  when that identity changes. Mutation requires the expected CID to match both
  the mount binding and current card; directory output carries the mount
  binding only after a final current-card check. Before every delete write, an
  atomic local journal persists the
  request ID, controller ID, SD-card ID, and filename. Every `RG` and `DG`
  worker holds an operating-system operation lease across the controller
  exchange, any authoritative journal reread, and durable settlement,
  preventing a stale
  listing from reconciling an active delete across application processes.
  Default journal state resides in the current account's operating-system
  state directory. The state directory and lock entry reject reparse or
  symbolic links, foreign ownership, non-regular lock entries, and additional
  hard links; POSIX state directories also reject group or other-user write
  access. Journal replacement synchronizes the temporary file and then uses
  `MoveFileExW` with write-through semantics on Windows or synchronizes the
  containing directory after replacement on POSIX.
  Process termination releases the lease while leaving any persisted pending
  delete for explicit orphan recovery. A delete exception or detailed
  controller error after serial commitment remains indeterminate across
  application restarts, blocks another delete, and requires a valid `RG`
  result from the original controller and SD card to determine whether the file
  remains. Journal temporary-file collision retries are bounded, and access
  failures propagate immediately. G-code conversion uses the same asynchronous
  delete owner and completion callback before row processing. Conversion
  admission captures the validated controller filename for the delete and
  every emitted `WC` row; later filename-field edits affect only subsequent
  work. Stop and shutdown
  requests share an atomic cancellation/commit boundary with the delete write:
  pre-commit cancellation clears the pending journal without transmission,
  while post-commit cancellation retains response ownership and suppresses row
  conversion after settlement. Conversion startup refuses shutdown, guards all
  Tk staging before worker transfer, and clears shutdown ownership on every
  failed transfer. Active conversion ownership remains until the row worker
  exits. Only reversible names ending in `.txt` become actionable from
  a directory response, and status text distinguishes actionable programs from
  total card entries. Host-generated names and the paired Teensy filename
  contract reserve comma for directory framing and reject outer spaces.
  Firmware directory traversal distinguishes clean end-of-directory from read
  failure, and delete lookup distinguishes confirmed absence from lookup
  failure. Firmware buffers no more than the shared 4096-byte directory payload
  and emits an explicit error before any listing for an incompatible entry,
  unavailable directory buffer, directory read failure, or aggregate overflow.
  Feed values convert from active units per minute to millimetres per second,
  and imperial coordinates convert from the parsed axis value before start
  offsets are applied. `Tool Set` rows reject without transmission because the
  tracked Teensy `6.7.1-ar4hmi.9` protocol has no `TF` handler.
- Virtual program completion preserves the original deadline when Tk scheduling falls back to a background owner. Timeout remains a failed result while ownership waits for the matching request-scoped operation to settle; failure to start the fallback worker retains the same settlement synchronously instead of publishing an early terminal row result.

- `controller_degree_to_native_radians` defines shared binary32 degree-to-radian and round-trip representability for motion envelopes, `UP` construction, custom-profile validation, startup native preflight, and Teensy conversion. Values rejected by the native public-degree boundary are rejected before controller writes or profile persistence.

Detailed findings and boundaries are tracked in `docs/hmi-optimization-audit.md`.

### M4A2 - HMI boundary completion

Status: `In progress`

Acceptance criteria:

- Main-controller requests and responses share a central transport owner.
- Program execution, Cartesian jog, tool-frame jog, startup, camera, Modbus, calibration, and auxiliary-board paths perform no blocking work in Tk callbacks.
- Worker threads do not read or mutate Tk widgets.
- Discrete motion, program execution, and remaining stop paths have explicit preemption and response ownership; the live-jog path has that ownership now.
- Baseline and post-change event-loop, serial, render, and persistence timings are recorded.
- J1-J6 position fields provide validated exact keyboard targets through the
  existing semantic queue, and asynchronous position refreshes do not overwrite
  active edits.

Implemented portion:

- J1-J6 confirmed-position fields accept exact absolute targets through Enter
  or keypad Enter and route through the existing semantic motion queue.
  Controller and virtual position refreshes preserve an active edit. Invalid
  submissions remain editable, while Escape or focus loss restores the latest
  confirmed position. Pointer focus preserves caret editing, while keyboard
  focus selects the complete value before replacement. Hardware-free
  source-contract coverage passes; live verification requires a later
  controlled HMI relaunch.
- A rejected main-controller replacement restores the last accepted port when
  that port remains in the current menu snapshot, or the `None` menu state
  otherwise, across synchronous rejection, asynchronous startup failure, and
  timeout settlement. Startup-scheduler failure invokes the Tk-owned timeout
  restoration before worker-side cleanup begins.
- Run and Step Forward treat selection state as mandatory program execution
  state. Run initialization, pre-row selection preparation, and completed-row
  advancement use request-scoped worker-to-Tk events; shutdown or cancellation
  declines pending transitions without publishing a false program failure.
  A Run selection-transition failure or completed-row transition error aborts
  row state and requests the program stop path instead of silently replaying a
  completed row; the authoritative stop message owns the status line. A missing
  Step Forward loop marker publishes a setup alarm before requesting that stop
  path, after which the authoritative stop message owns the status line. Worker
  handlers attempt a failure alarm after stop dispatch raises; the alarm is
  admitted only when no authoritative stop reservation was latched. All other
  Step Forward pre-execution setup errors alarm without dispatching a transport
  stop. Step Forward clears the selection at end of program so the existing
  loop or caller-return path handles the next step. Styling and current-row
  display failures remain diagnostic-only after the selection commits.
- Vision template matching and preview loading read one bounded regular-file
  image, validate encoded size and header dimensions before OpenCV decode, and
  verify decoded dimensions against the admitted header. Preview formatting
  preserves a positive resized width and height for extreme aspect ratios, and
  preview failures produce a stable HMI alarm.
- `.ar4` register and position-register rows resolve only through explicit
  widget registries after bounded decimal validation. General registers accept
  the defined 1-16 range; position-register elements accept 1-6. Program text
  cannot become a Python expression, and missing registry bindings fail before
  widget access.
- Direct 5V I/O servo and output buttons validate widget values on Tk and
  serialize exact-response exchanges through a worker. A bounded FIFO retains
  additional manual I/O only behind an active manual exchange; unrelated
  auxiliary transport ownership rejects the sequence instead of deferring
  actuation into a program. Tk event queues apply acknowledgements and
  program-stop status without holding stop-state locks across widget calls,
  and only confirmed settings are persisted. Malformed input, stale
  connections, invalid board pins, response failures, queue saturation, stop
  rejection, and transport contention remain visible. Manual I/O is rejected
  while a request-scoped `.ar4` Run, Step Forward, or Step Reverse owner is
  active, including worker startup and asynchronous reverse completion, and
  while direct row execution remains active. Rejected overlapping program
  admission cannot clear active fault flags, and a pending program-stop request
  blocks new program admission. Setup and worker failures release matching
  program owners, and malformed row failures clear direct row-active state.
  Worker failures publish alarms through the Tk status queue, synchronous
  reverse-motion rejection cannot produce a false ownership error, and
  execution contention or failure status cannot replace an active stop alarm.
  A cancelled asynchronous Step Reverse completion aborts row state and
  releases the matching program owner; only an already-released request is
  treated as stale. Called-program view changes use a request-scoped Tk event
  and acknowledgement, with the cancellation boundary released before widget
  access. Stop admission therefore cannot form a worker/Tk lock cycle, while a
  worker cannot advance until the matching Tk transition settles. Called and
  manually loaded `.ar4` sources require regular files, bounded file, row, and
  line sizes, UTF-8 logical rows without unsupported control characters, and
  the `.ar4` extension. Program-issued calls accept only leaf targets, resolve
  each target beside the current source path, and preserve the exact caller
  source path for return navigation. View replacement rolls back rows, styles,
  selection, scroll position, path, current row, and return state after an
  application failure.
  Program stop and E-stop discard queued requests, and manual status cannot
  replace an active stop alarm. A later manual-panel interaction acknowledges a
  terminal stop-status reservation only when no stop, E-stop, or position fault
  remains active.
- Visible manual Modbus read and write controls build canonical requests on Tk
  against the same slave, address, quantity, coil, and register domains
  enforced by the Teensy firmware. Request-scoped work retains main-transport
  and shutdown-activity ownership through bounded response parsing and Tk
  result application. Manual controller I/O, Run, Step Forward, Step Reverse,
  G-code storage, and G-code conversion use a shared lock-protected admission
  boundary and reject mutual overlap through final cleanup settlement. `BE`
  and `BF` mutation admission also holds the shared stop-state locks and
  rejects while an E-stop latch, position-fault latch, or stop-settlement owner
  remains active.
  Overlapping panel requests reject without blocking Tk,
  while joint input recorded during the exchange remains deferred and is
  retried after trusted ownership release. A bounded pre-write probe consumes a
  queued standalone physical-E-stop frame without resetting unread controller
  input or transmitting the requested command. During an admitted exchange,
  the response owner accepts a terminal frame with an asynchronous E-stop in
  either order. Each validated E-stop exchange snapshots the complete
  auxiliary profile under the shared configuration/stop-state lock order,
  atomically latches shared stop state, cancels queued manual auxiliary work,
  blocks new manual auxiliary admission, and reserves a request-correlated
  auxiliary `STOP` before publishing an independent Tk event. A concurrent
  auxiliary configuration commit therefore either rejects against the latch or
  becomes visible to the stop reservation as one complete snapshot, while
  physical-stop latching remains independent of auxiliary transport
  availability. Failed `STOP` dispatch returns the same request to pending
  state until an inactive-stop acknowledgement settles the owner. The worker
  retains main-response ownership to drain the interrupted terminal frame or
  quarantine uncertain framing. Request correlation prevents a stop event from
  settling a different manual operation. Manual stop presentation remains
  retained ahead of the terminal result when rendering or motion invalidation
  raises. A presentation retained across a fault-free replacement startup is
  retired diagnostically without changing the replacement controller's fault
  latches. The reported position is then applied on Tk, connection closure
  remains required before ownership release, and deferred motion is discarded.
  Correlated `EA` admission rejection proves that the consumed command did not
  execute; an immediately preceding asynchronous `EB` is retained as the
  independently published stop event. Deferred dispatch reports distinct idle,
  blocked, dispatched, and rejected outcomes. Program-stop, E-stop, and
  position-fault reservations block dispatch without consuming the retained
  target, while permanent target rejection retains the alarm instead of
  falling through to `SYSTEM READY`.
  Standalone `ER` responses and read-side `Modbus Error` responses remain
  explicit controller rejections. A `BE` or `BF` `Modbus Error`, a write-side
  `ER` paired with an asynchronous `EB`, or another failure after write
  transmission starts records an indeterminate external-device state, closes
  the connection, discards deferred motion, and requires reconnection plus
  device-state verification. The tracked firmware publishes the result-specific
  `BE` or `BF` terminal before post-transaction E-stop polling, preserving a
  completed, indeterminate, or rejected result when the scoped `EB` follows.
  Multi-register drive-service writes emit the indeterminate `Modbus Error`
  terminal when an E-stop interrupts a partially applied sequence and retain
  failure across every write in the sequence instead of reporting only the
  final register result.
  Failed close, activity-lease release, or transport-lock release retains the
  remaining ownership components and final settlement presentation for later
  Tk polling and blocks shutdown.
  Main-controller replacement rejects while this retained cleanup owner remains
  pending.
  Physical resources may release before widget presentation succeeds, but the
  pending result owner remains durable until final rendering and a post-render
  connection-trust check both succeed.
  Settlement presentation is selected after connection trust and result
  application are evaluated. Connection loss or presentation-application
  failure therefore cannot preserve a normal completion message, and readiness
  requires both a trusted open connection and an idle deferred-dispatch
  outcome. Connection loss after resource release detaches the stale handle,
  clears matching controller identity, invalidates pending joint intent, and
  renders a reconnection alarm without falsely changing confirmed robot
  position.
- Multiple worker-result event queues share a Tk sibling-retry registry. A
  scheduling failure records the registered poll name, and each still-running
  sibling poll retries registered failures without calling Tk from a worker
  thread. This mechanism depends on at least one live Tk poll and does not
  recover when the Tk scheduler or interpreter is unavailable. Application
  shutdown drains every registered queue and clears pending retry state.
- G-code controller-directory reads and file deletion validate and admit work
  on Tk, then perform the bounded serial exchange under a request-scoped worker
  owner. Correlated results update the file list on Tk before transport and
  logical-request release. Every successful directory listing response carries
  the SD-card CID; an empty directory retains that identity prefix. Detailed
  `EG:` responses carry error text instead. Only reversible `.txt`
  entries become actionable, and operation-specific terminal sets quarantine
  unmatched frames.
  A successful deletion starts a silent directory refresh only after the delete
  owner releases and only when the local-view generation remains current. The
  displayed local path is read-only; successful local-file loads atomically
  replace the view and advance that generation. Regular-file validation,
  bounded binary reads, source-byte and row limits, printable-ASCII and tab
  validation, horizontal-whitespace canonicalization, repeated-row
  preservation, and controller command-length validation precede row
  accumulation. Blank and comment-only rows are discarded. A source with no
  actionable row rejects before view replacement or controller-file deletion.
  Rollback restores
  executable Listbox state, displayed
  content, and the exact program-path widget state.
  Operator storage and loading reject during G-code conversion; captured
  conversion completion callbacks provide scoped pre-row delete admission.
  Before a delete write, an atomic local journal records the request,
  controller hardware identity, SD-card identity, and filename. An
  operating-system operation lease serializes every authoritative journal
  reread, controller exchange, comparison, and replacement across application
  processes. A terminated lease holder leaves the durable pending record for
  explicit directory-based orphan recovery.
  Default state uses a private per-user operating-system state directory.
  Lock admission is no-follow, owner checked, regular-file checked, and
  single-link checked. POSIX replacement synchronizes the containing
  directory; Windows replacement uses write-through move semantics. Any
  pre-dispatch lock-admission or pending-journal persistence failure propagates
  before serial commitment. A post-commit delete, journal-settlement, or
  detailed controller failure publishes an indeterminate result, retains
  process-local pending state, and requires a valid directory refresh from the
  original controller and SD card. A restart reloads the atomic journal image;
  any durable pending record continues blocking deletion until that refresh
  confirms file presence or absence. A lease-release failure after a validated
  controller response and durable journal replacement preserves the definitive
  result, records reconciliation state as unavailable, and requires the next
  storage request to reacquire the operation lease and reload the journal
  before admission. Conversion uses the same
  asynchronous delete owner and an atomic cancellation/commit boundary. Stop
  or shutdown before commitment prevents transmission and clears the journal;
  cancellation after commitment retains the terminal response owner and
  prevents row-worker startup after settlement. Tk staging and thread transfer
  are guarded so every pre-worker failure clears shutdown ownership.
  Conversion refuses to start after shutdown begins and remains part of
  shutdown ownership until the conversion worker exits.
  Storage E-stop frames quarantine the transport and trigger a verified-close
  attempt because later command output remains possible. A failed close leaves
  the handle quarantined for explicit reconnection. Storage ownership then
  follows ordinary result cleanup; only a cleanup-release failure activates
  the retained retry owner. Host and firmware filename validation reserve the
  comma directory separator and reject outer spaces.
  Firmware directory iteration distinguishes clean end-of-directory from read
  failure, and delete lookup distinguishes confirmed absence from lookup
  failure. The firmware enforces the shared 4096-byte aggregate payload bound
  and reports incompatible entries, allocation failure, read failure, or
  overflow before emitting a partial listing. SD initialization remounts the
  filesystem after a CID change, and mutation validates the requested CID
  against both the mount binding and current card. Startup requires the paired
  `GCODE_DIRECTORY_FRAMING_V1` and
  `GCODE_DELETE_IDENTITY_V1`, and `GCODE_WRITE_IDENTITY_V1` capabilities
  before any controller mutation.
- Automatic and single-axis calibration buttons prepare validated commands on Tk, launch worker-owned serial exchanges, and apply terminal results on Tk without serial reads or controller waits.
- Multi-stage automatic calibration retains shared motion ownership, main transport reservation, and a shutdown-activity lease until the final successful stage or first failure settles.
- Calibration shutdown cannot cancel an active `LL` read because current Teensy firmware exposes no paired abort command. Pre-write shutdown rejects transmission at the shared lifecycle boundary and interrupts stalled pre-write activity after the normal drain interval. Post-write shutdown remains supervised until an applied terminal controller frame or explicit quarantine and verified-close handling settles the operation, while unrelated auxiliary activity continues through normal shutdown interruption. A later protocol pass must define preemption before claiming immediate calibration cancellation.
- Live camera preview opening, warm-up, capture reads, frame conversion, and
  release run under a request-scoped daemon worker. Tk start and stop callbacks
  only submit replacement or cancellation intent, while shared event polling
  applies the latest coalesced validated frame. Existing vision commands can
  snapshot the latest owned raw preview frame without performing a concurrent
  device read. Replacement discards stale transitional lifecycle chatter while
  terminal stop and failure events remain available, camera cleanup failure
  blocks reuse, and application shutdown supervises worker cleanup for a
  bounded grace period without joining on Tk. Windows and USB source identities
  remain explicit; CSI identities fail closed until a Picamera2 or libcamera
  adapter exists. Hardware camera behavior and timing remain unverified.
  Program `Cam On` and `Cam Off` rows use bounded,
  cancellation-aware waits away from Tk so the next row observes a ready frame
  or a quiescent device. Step Reverse returns camera settlement through the Tk
  event poll, and cancellation retires an unready start request before row
  ownership is released. Preview-off still capture reuses the same validated
  worker lifecycle and backend instead of opening an independent capture path.
  A capture-only worker now snapshots brightness, contrast, zoom, mask, and
  background inputs on Tk; coalesces pending `Snap Image`, `Zero`, and slider-
  release requests to the latest complete settings record; and performs camera
  acquisition, grayscale conversion, zoom, mask application, display resizing,
  and image persistence away from Tk. Request-scoped results return through the
  existing camera poll for Tk-only image and field presentation. A capture
  submitted during preview startup waits for the first owned frame, while a
  capture submitted during preview teardown waits for device quiescence before
  starting a one-shot request. A non-waiting artifact owner rejects overlapping
  capture, mask, template, and matching file access instead of racing the shared
  image files or blocking Tk behind another workflow. Shutdown cancels pending
  capture-only work and
  supervises an active capture under the bounded camera-worker grace period.
  Mask and template drag bounds normalize direction, clamp to the captured
  image, and reject undersized selections before mutating stored mask state;
  callback persistence failures close the OpenCV windows and publish an HMI
  alarm. Successful capture events are presented in worker order, so a failed
  coalesced successor cannot leave the most recent retained successful capture
  hidden behind an older displayed still frame.
  Manual `Snap & Find` now snapshots capture, template, score, rotation,
  joint-limit, and pixel-to-robot calibration inputs on Tk and submits one
  non-coalescing request. A daemon worker owns frame acquisition, captured-image
  persistence, bounded template loading, rotation matching, and annotated-result
  persistence under one artifact lease; the camera event poll performs the
  resulting Tk-only field and image presentation. Matching consumes immutable
  inputs, checks cancellation between rotation candidates, examines each of the
  360 unique integer-degree orientations at most once during full search,
  retains the best sub-threshold result, and applies symmetric J6 fallback
  limits. Failed matches clear stale pixel and robot-coordinate outputs.
  Application shutdown cancels and supervises the matching worker under the
  existing bounded camera-worker grace period. The shared capture-and-match
  worker contract removes an accepted request still pending worker pickup and
  appends a terminal cancellation event during the same locked close
  transition, allowing a waiting program-row owner to settle during shutdown.
  Shutdown also cancels the active program request and directly settles queued
  and registered program-vision owners on Tk, independent of later camera-poll
  delivery. Off-Tk program waits observe request cancellation, and an atomic
  worker drain-and-lifecycle snapshot rejects any registered owner that loses
  active and pending worker ownership without a terminal event.
  Worker results arriving after direct shutdown settlement are discarded by
  request identity instead of being misreported as failed manual matches.
  Program `Vis Find` rows parse
  into validated immutable commands, snapshot capture, matching, and exact
  program-view inputs on Tk, and reuse the same non-coalescing worker for one
  artifact-owned capture-and-match operation. Run and Step Forward wait only on
  their program workers; Step Reverse returns pending and settles through the
  Tk event poll. Result presentation and pass/fail tab selection occur on Tk
  only after request identity, cancellation state, worker ownership, and the
  unchanged program-row snapshot are verified. Failed capture, matching,
  presentation, missing-tab, stale-request, and edited-program paths reject the
  row without selecting a stale destination. `Move V` consumes an immutable
  successful match result instead of parsing Tk result fields, so absent,
  failed, pending, or unsuccessfully presented matches cannot become motion
  commands. The superseded synchronous program capture and matching helpers
  have been removed.
  Hardware camera behavior and timing remain unverified.

Remaining scope includes broader program and G-code row-execution admission,
interactive mask and template workflows, Modbus, auxiliary connection and
device paths, durable event-poll failure handling when the Tk
scheduler or interpreter is unavailable, application-lifecycle, timing, and
calibration-preemption work.

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
- Delivered source remains isolated audit input. During the M4A3
  delivered-source audit, no application entry point or executable was run and
  no controller write, calibration, firmware flash, or robot motion occurred.

### M4A4 - Selective 7.0/2.0 integration

Status: `In progress`

Integration unit status:

- Native inverse kinematics, safe configuration, and command-local wrist propagation: `Tested`.
- Line-oriented strict numeric syntax and shared motion-command grammar: `Tested`.
- Line-oriented auxiliary-controller compatibility hardening: `Tested`.
- Line-oriented command-specific numeric and safety-domain coverage: `In progress`.
- Typed configuration normalization and MK5 calibration-switch polarity: `In progress`.
- Correlated host/Teensy JSON protocol and firmware safety corrections: `Proposed`.
- Hardened HMI transport adoption of the correlated protocol: `Proposed`.
- Mega/Nano 2.0 JSON capability integration: `Proposed`.
- Application lifecycle and broader program/firmware coverage: `Proposed`.
- Optional isolated CAD/EOAT boundaries: `Proposed`.

Authorized scope:

- Port native inverse-kinematics validation, wrist-singularity continuity, configuration exposure, and deterministic tests before rebuilding supported native binaries.
- Add schema-validated MK5 calibration-switch polarity and numeric configuration normalization without importing machine-specific ports, limits, poses, or calibration values.
- Define a correlated JSON host-controller contract with a dedicated response owner, bounded timeouts, explicit event separation, and paired firmware fixtures before changing active command encoding.
- Correct Teensy JSON validation, Cartesian array bounds, G-code buffer rotation, and emergency-event ownership before the correlated JSON protocol unit is compiled, simulated, or considered for hardware use. Independently coherent line-oriented compatibility units retain separate compile requirements.
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

Implemented portions of the active integration unit:

- Runtime and custom-profile JSON now pass through a shared calibration schema
  before application or persistence. Accepted profiles contain only known
  fields, scalar values, and a structured three-component RGB background
  color. Duplicate keys, non-finite numbers, invalid enumerations,
  non-migratable missing runtime fields, unsupported compound values, and
  values outside the field-specific numeric ranges fail before mutation.
  JSON fractions retain decimal precision through integer and range
  validation, preventing fractional integer fields, negative underflow, or
  upper-bound overflow from rounding into accepted values.
  Compatibility loading supplies only the documented switch, disconnected
  port, and inferable auxiliary-board fields. Complete runtime profiles also
  apply the shared controller binary32, degree-to-radian, ratio,
  calibrated-step, and J1-J9 pose-limit contracts. Optional J7-J9 travel may
  remain zero while each external-axis rotation and step scale remains
  positive. Servo positions use the auxiliary-controller range, and nonempty
  digital-output fields require pins valid for the selected Nano or Mega
  profile. Saved position text remains strict plain-decimal text because the
  current program serializer consumes that representation.
  Vision background parsing no longer evaluates profile or program text and
  accepts the prior bare and parenthesized RGB triplets through strict
  non-evaluating compatibility parsers. Serialized background colors use RGB
  order; OpenCV color-mask buffers receive an explicit BGR conversion, image
  capture and matching use grayscale buffers, and canonical RGB background
  values use OpenCV's RGB-to-gray conversion. Vision sample coordinates are
  checked against the current frame
  immediately before pixel access. Capture, mask, and program-insertion
  callbacks expose logged Boolean failures; template loading and result-image
  writing propagate into logged snap or program-row failure results. Failed
  capture or matching cannot silently reuse an older captured image or choose
  a program branch. Program insertion
  validates every reserved vision-row delimiter before changing calibration
  or editor state, atomically replaces a completely encoded program with
  canonical LF line endings, and removes the inserted row when program
  persistence fails. Program branch lookup compares decoded logical row
  content, so LF and CRLF input select the same tab without weakening UTF-8 or
  single-line validation. Save/apply resolves live Tk
  bindings into a separate schema snapshot and validates the complete merged
  calibration before local or controller mutation. Auxiliary connection
  changes use the same snapshot boundary, clear output assignments outside the
  selected board's pin range before replacing the serial connection, and
  preserve existing Tk binding identity when applying normalized values.
  A failed replacement restores the prior validated configuration without
  rewriting the stored port, board, or digital-output assignments. Active
  connection authority remains in the serial handle and its board-profile
  binding; an orphaned replacement that cannot be closed loses that binding
  and cannot authorize output commands.
  A completed connection change retains the staged configuration after a
  later persistence or logging failure. Existing calibration persistence is
  fenced before local mutation. A completed connection is verified without
  rewriting committed local state; recovery targets are normalized and
  re-read across live calibration, digital-output fields, and the relevant
  selectors before persistence becomes dirty. Failed or partial recovery
  disables the output profile and prevents an unverified queued save.
  Pre-existing dirty persistence retains an isolated pre-attempt snapshot as
  the recovery retry target, so the transaction's unverified state cannot
  replace the already queued write. Persistence-fence completion is
  exception-based: a successful no-write settlement returns normally, while a
  failed retry retention enters the existing recovery and reconciliation path.
  Calibration persistence verifies the complete temporary text write,
  synchronizes file contents, and uses platform-specific durable replacement;
  any pre-replacement failure preserves the prior profile.
  Legacy pickle migration bounds the source file, forbids global construction
  and trailing data, validates the indexed scalar format without stringifying
  typed values, maps null ports to the disconnected sentinel, maps null
  mapped-servo and digital-output fields to empty values, and rejects nulls in
  every other legacy field. Migration commits the normalized JSON profile
  before a best-effort backup rename and does not report a backup-only failure
  as conversion failure. Non-default runtime paths
  use colocated legacy input, JSON output, and backup paths. Legacy JSON runtime
  profiles gain the current all-`HIGH` J1-J9
  calibration-switch behavior and disconnected-port sentinels without
  rewriting the source file. A missing auxiliary-board profile is inferred
  only when every configured output belongs to one disjoint Nano or Mega pin
  range; mixed or unknown assignments require an explicit migration choice,
  while profiles without configured outputs remain disconnected.
  Switch polarity is not yet exposed in the HMI or transmitted to the
  controller; paired host and firmware protocol work remains required before
  any `LOW` selection can affect calibration behavior.
- Native inverse kinematics rejects wrong-size, non-finite, and unrepresentable motion inputs, keeps candidate state local, enforces configured joint limits, validates selected-candidate round trips, and shares physical-displacement wrist seed and ranking rules with Teensy firmware through singular and wrap-boundary poses.
- `AR4.py` uses the wrist mode encoded in each validated motion command for virtual inverse kinematics and rejects any physical/virtual mismatch before dispatch. Motion requires the wrist-aware configured solver; legacy solver fallback is rejected before invocation.
- Native configuration stages and validates binary32 values, derived tool-frame radians, and public-unit round trips before atomic binding application. Motion requires the atomic configuration setter and wrist-aware configured solver, so bundled legacy Linux extensions fail closed without mutating native state. Configuration repair, generic non-motion program exchanges, and controller reconnection remain admissible without enabling kinematics-dependent motion.
- The Windows CPython 3.12 x64 module is rebuilt under an ABI-tagged filename from the tracked source with an isolated pinned pybind11 build dependency, explicit Release optimization, warning-clean compilation, and direct binding runtime tests. Linux motion requires a matching extension built from the tracked native source; bundled legacy Linux files are unsupported for motion. Hardware-free build and test evidence is recorded in `docs/hardware-free-verification-2026-07-19.md`.
- Discrete and live Cartesian and tool commands carry the same command-local wrist field through host validation, virtual preview, and Teensy parsing. Tool motion snapshots the active calibration frame rather than editable staged fields. Startup requires the controller's `JT_WRIST_CONFIG_V1`, `GCODE_DIRECTORY_FRAMING_V1`, `GCODE_DELETE_IDENTITY_V1`, `GCODE_WRITE_IDENTITY_V1`, and `ESTOP_ADMISSION_V1` capabilities before auxiliary connection, calibration writes, or position synchronization.
- Controller-bound Cartesian orientation, vision rotation, and rotational tool-jog fields must remain representable after both binary32 degree encoding and binary32 radian conversion. The paired firmware conversion contract rejects nonzero values that collapse to zero before solver or tool-frame mutation. Vision rotation applies to the native Rx tool slot and restores that slot after inverse kinematics; `UP` validates tool and DH rotations before mutation and refreshes the active kinematic cache before acknowledging success.
- The line-oriented Teensy strict parser accepts only complete delimiter-safe plain-decimal fields. Serial preprocessing removes one required line ending, SD preprocessing removes only the line ending already consumed by the SD reader, and neither path removes payload whitespace before command parsing. Shared `JT`, Cartesian, vision, joint, and live-jog grammars reject malformed text, junk prefixes, trailing data, marker collisions, invalid fixed-width fields, non-finite values, binary32 overflow, nonzero binary32 underflow, and integer overflow before parsed output mutation. Every sibling `String.toFloat()` and `String.toInt()` conversion has been replaced by shared strict parsing. Command-specific domain validation now covers motion timing with a shared `(0, 100]` ramp range, calibrated targets, stored step rows, waits, Modbus values, explicit main-controller `ON`/`OF` rejection because no safe GPIO profile exists, controller filenames with a shared 255-byte limit, and nondegenerate path geometry using the complete traversed arc angle. Ordered arc validation and execution share the same staged center, axis, radius, and traversal angle, including midpoint-selected major arcs. Host and firmware live-jog parsing accept only Percent mode and integral axis/direction vectors inside the applicable joint domain; validated speed, acceleration, deceleration, and ramp fields reach every controller and offline handler, and joint live jog accepts only the effective `WA` wrist suffix. All line-oriented serial readers share the host's 4096-byte command-frame ceiling, reject overflow once, discard through the next LF before accepting another frame, and never append an empty-read sentinel to a command buffer. Motion handlers pass the command-local wrist selector directly into inverse kinematics and stage wrist and encoder loop modes until the motor driver completes internal timing, direction, and emergency-stop preflight; rejected, zero-distance, and precompute-only frames preserve the preceding global mode state. Only `ML` accepts the required nonnegative `Rnd` field; other Cartesian motion and storage opcodes reject rounding instead of discarding the field. Unsupported `ML Q1` wrist suppression and nonzero `MA` or `MC Tr` values fail closed in both host and firmware parsing instead of becoming silent no-ops. Host Modbus rows classify command-specific success and response shapes before state advancement. Exhaustive firmware command-domain coverage remains `In progress`.
- Rounded `ML` execution carries the originating command-local wrist selector and loop modes into the derived `MA` frame. Live `LC`, `LJ`, and `LT` control readers accept only exact `S` lines with LF or CRLF termination; overlength or other complete control frames receive one error from the live terminal-response owner, while ordinary command-ingress callers own their separate overflow responses. Host and firmware filename validation rejects the complete FAT-reserved character set, the comma-delimited controller-directory separator, outer spaces, and control or non-ASCII input. Controller-directory framing additionally rejects `.txt` entries without a reversible stem and caps the aggregate payload at 4096 bytes before emission. Controller Modbus polling waits require a positive timeout so every accepted wait performs at least one polling interval.
- The paired line-oriented Nano and Mega compatibility sketches use
  byte-identical self-contained fixed-buffer protocol contracts. Complete
  command parsing validates board-specific servo, input, and output domains,
  wait state and positive timeout, integer overflow, frame length, and
  printable ASCII before any output mutation. Nano excludes unsupported servo
  channel 6, and Mega excludes Serial0 pins 0 and 1 from its input domain.
  Servos remain detached through startup; initial attachment keeps AVR
  interrupts masked until the first admitted position replaces the library
  default. Current telemetry is read-only and cannot trigger an autonomous
  servo correction. `WI` uses a rollover-safe nonblocking state machine,
  samples before classifying deadline expiry, and accepts only exact `STOP` or
  `STOPWI` interruption while active.
  `JF` samples once, and `TM` emits exactly one line delimiter.
  Host legacy transport validates `SV`, `JF`, `WI`, `ON`, `OF`, and `TG`
  against the handle-bound board profile before write. Dynamic gripper-current
  responses require a matching `TG` command plus bounded unsigned
  plain-decimal amperage. Malformed or out-of-range application payloads reject
  the program row after a clean frame without treating the transport as
  corrupt. Arduino CLI no-upload builds pass for Arduino AVR core 1.8.8 and
  Servo 1.3.0, and the sanitized C++ harness exercises the actual parser, frame
  recovery, atomic rejection, wait transitions, rollover behavior, and
  active-wait command admission. These checks establish no live
  auxiliary-controller behavior. Runtime firmware identity negotiation remains
  required before this rewritten compatibility firmware can be distinguished
  from the prior blocking firmware during connection.
- SD-card identity is revalidated after directory and delete-lookup traversal before any successful listing or absence response. Deletion revalidates the admitted CID immediately before and after removal. `WC` and `WG` carry the admitted `Mi<CID>Fn<filename>` target, and firmware revalidates that CID before and after each write. A changed or unreadable card produces a detailed storage error without a success response.
- Modbus `BA`, `BH`, and `BD` reads accept exactly one register because the
  firmware and host response contract carries one scalar value. Other read
  quantities fail before controller transmission. Live `LC` and `LT` handlers
  validate the complete start command before emitting the blank start
  acknowledgement. Finite `ML`, `MC`, and `MA` trajectory loops stop at the
  first joint-limit fault and emit at most one terminal fault frame. SD
  playback accumulates each stored row under the shared command-size boundary
  before parsing and stops after any non-completed Cartesian row without
  duplicating a terminal fault response. Encoder collision sends one
  `EC`-bearing position frame before returning terminal-fault status, so
  stored playback stops before reading another row. Emergency-stop state read
  by motion loops is declared volatile because the interrupt handler writes
  that state asynchronously.
- The paired line-oriented Teensy `6.7.1-ar4hmi.9` compatibility unit advertises `JT_WRIST_CONFIG_V1`, `GCODE_DIRECTORY_FRAMING_V1`, `GCODE_DELETE_IDENTITY_V1`, `GCODE_WRITE_IDENTITY_V1`, `HOME_REFERENCE_V1`, `HOME_REFERENCE_V2`, `JOINT_TELEMETRY_V1`, and `ESTOP_ADMISSION_V1` and compiles for `teensy:avr:teensy41` with PJRC core 1.62.0, bundled SdFat 2.1.2, and ModbusMaster 2.0.1. The compile regression parses the verbose dependency report and requires the active SPI and SdFat folders under the selected Teensy platform; [`docs/hardware-free-verification-2026-07-19.md`](docs/hardware-free-verification-2026-07-19.md) records dated no-upload compatibility results through `.9` and establishes no live-arm behavior or correlated JSON readiness.
- Firmware identity fields share a bounded printable-ASCII storage and response contract with the host, use the same tested JSON producer as the sketch, and load from explicitly terminated EEPROM buffers. The `SR` transport reserves `[M]`, `[V]`, `[B]`, `[S]`, and `[A]`; its shared parser rejects missing, reordered, duplicated, field-embedded, empty, control-byte, and overlength input before persistence, while stored and migrated legacy identity values retain the complete printable range. Erased identity storage, an in-progress transaction, a current committed record, a legacy committed record, and corrupt storage have distinct marker states. Startup migrates valid legacy identity and binary debug data through verified current-schema records, substitutes `Unset` only for erased legacy identity fields, and blocks `HO` after failed migration. The erased debug byte left by the legacy `SR` path migrates to the disabled safe default; every other legacy debug byte outside the binary domain aborts migration before any current-schema write. Interrupted current-schema identity writes reload as corrupt and also block `HO`. `HO` emits one protocol frame regardless of persistent debug or spline state. Debug commands validate the complete grammar before transactional persistence and live-state mutation; complete-buffer EEPROM verification failure suppresses success. Every early `loop()` exit consumes the active command through a tested queue-rotation primitive, so invalid `DB`, `SR`, `JT`, and missing-file `PG` commands cannot wedge later work.
- Native boundary, binary32 underflow, degree-to-radian underflow, strict firmware numeric parsing through the complete shared `JT`, `MJ`, and `MV` handler grammar, serial and SD frame extraction into the shared parser, exact live-control classification and terminal-response selection, strict `SR` identity extraction, paired host/firmware ramp and FAT-reserved filename limits, positive Modbus polling timeouts, rounded `ML` wrist preservation, ordered minor- and major-arc execution geometry, branch, singularity, near-singularity, shared native/firmware wrist generation, cross-seam branch parity, nearest in-range multi-turn normalization, joint-limit, unreachable-target, tool-frame representability, tool-rotation geometry, vision-Rx restoration, immediate firmware kinematic-cache refresh, parallel-determinism, sanitizer, host-routing, command-specific Modbus response classification, generic program admission, Windows loaded-binding, Linux source-build/import, firmware identity, old-`SR` EEPROM migration with erased debug storage, binary legacy debug migration, marker-valid identity corruption, interrupted identity transaction reload, partial-byte EEPROM failure, command-queue, control-query spline isolation, large-finite wrist normalization, transactional debug-command, Cartesian display/wire-to-native ordering, Cartesian direction, firmware wrist-selection rejection, and paired tool-jog signed-TCP-displacement contracts have hardware-free regression coverage.
- G-code directory contract coverage includes exact and overflowing aggregate
  host payloads, reversible and degenerate `.txt` stems, controller and SD-card
  identity binding, cross-process operation-lease exclusion through serial
  settlement, no-follow owner-checked single-link lock and journal admission,
  durable platform-specific replacement, bounded nonblocking journal
  rereading, strict journal schema types, durable restart reconciliation,
  bounded journal, bounded ASCII local-program loading, and
  test-fixture temporary allocation under a validated external parent, prompt
  post-E-stop output,
  operation-specific terminal sets, pre-write and indeterminate post-write
  delete failure, directory reconciliation, real local-load generation
  changes, horizontal-whitespace canonicalization, exact read-only path
  rollback, asynchronous conversion admission, pre-write stop cancellation,
  failed worker-transfer cleanup, write-identity error rendering, and shutdown,
  unterminated final local rows, shared native helper execution, and no-upload
  Teensy compilation. Firmware source-contract checks cover clean
  versus failed directory traversal and case-insensitive delete lookup branch
  ordering. Shared directory-entry name extraction executes against boolean
  and length result contracts; full SD-library traversal remains
  hardware-free source and compile coverage.

### M4A5 - Desired-versus-estimated-and-encoder joint display

Status: `Tested`

Display contract:

- The normal J1-J9 slider thumb represents the active operator input or the
  latest accepted desired target.
- A violet marker represents the endpoint of the coordinated `RJ` command
  currently executing on the controller. Later coalesced input moves the
  normal slider thumb immediately, while the violet marker remains fixed until
  the current command settles and the retained target becomes active.
- A narrow cyan marker represents the commanded coordinated-`RJ` estimate.
  Encoder or terminal position feedback never replaces or recolors this
  marker.
- A separate wide amber marker represents only the latest validated J1-J6
  encoder sample received through request-scoped telemetry. Amber remains
  absent before the first validated telemetry sample, never falls back to
  estimated, startup, terminal, or step-counter data, remains visible while
  idle only while the same open, non-quarantined controller identity remains
  bound, stays above overlapping target and estimate markers, and is never
  presented for J7-J9. Controller trust or identity loss, transport quarantine,
  and a coordinated-joint terminal result without a validated position clear
  the retained amber display because the sample source or controller state is
  unknown.
- Independent controls permit estimate-only, encoder-only, combined, or
  disabled estimate and encoder-sample presentation without changing the
  motion request. The active-target marker remains visible for an executing
  coordinated move.
- A legend identifies the normal thumb as desired input, violet as the active
  move target, cyan as the command estimate, and amber as the latest encoder
  sample.
- The estimate follows the controller's calibrated step conversion,
  synchronized high-step progression, and acceleration, cruise, and
  deceleration timing envelope.
- Controllers without `JOINT_TELEMETRY_V1` expose no encoder-sample marker.
  Startup and terminal position responses continue to update confirmed host
  state and normal sliders but never populate the amber channel.
  Telemetry-capable controllers emit request-scoped interim samples that never
  advance confirmed calibration state.
- Terminal position feedback remains authoritative for confirmed state and
  reconciles every idle normal slider. An actively dragged slider retains
  operator input until pointer release. Faults or unavailable estimates hide
  the marker rather than fabricating live position.
- Cartesian, tool, program, calibration, homing, and indefinite live-jog
  operations remain outside motion tracking until a validated trajectory or
  correlated live-telemetry contract supplies meaningful progress.

Acceptance criteria:

- Estimated and encoder motion markers can be enabled independently without
  affecting motion admission, serial ownership, or controller state.
- J1-J9 desired targets remain interactive while an accepted coordinated move
  owns the serial interface.
- Marker updates run only on the Tk event thread and perform no serial I/O,
  sleeps, persistence-lock acquisition, or worker waits. The source check reads
  an immutable in-memory controller snapshot without taking the controller
  identity mutation lock.
- Deterministic hardware-free tests cover step-derived duration, synchronized
  multi-axis interpolation, terminal clamping, toggle behavior, desired-target
  preservation, fixed active-target presentation during coalesced input,
  persistent same-controller encoder samples, source-loss invalidation,
  active-drag preservation, marker geometry, and deterministic
  target-below-estimate-below-encoder layering.
- Live-arm comparison is recorded separately; hardware-free telemetry fixtures
  do not establish physical encoder accuracy, cadence, or motion-timing impact.

Implemented evidence:

- A validated command-trajectory estimator models controller binary32
  calibration conversion, coordinated step timing, and average
  pulse-distribution overhead without reading or mutating Tk state.
- A Tk-thread visualization owner updates the J1-J9 estimate markers and
  active-target markers plus separate J1-J6 encoder-sample markers from the
  existing joint-motion poll, preserves active pointer drags and coalesced
  desired targets across delayed worker events, retains the active command
  endpoint independently from newer desired input, binds retained samples to
  an open, non-quarantined controller source, reads an atomic source snapshot
  independent of G-code persistence locking, uses the worker's monotonic
  dispatch timestamp, and never substitutes one marker channel for another.
- The complete Windows hardware-free suite passes. Ubuntu verification of the
  expanded target, estimate, and encoder marker unit remains pending. Tracked
  marker tests exercise distinct styling, independent estimate and encoder
  selection, encoder-sample absence before validated telemetry, persistent
  same-source idle display, source-loss and open-quarantine invalidation,
  dedicated identity-lock isolation, complementary marker geometry, fixed
  active targets during queued input, change-driven
  deterministic amber-above-cyan-above-violet layering, shared pointer
  forwarding, drag preservation, redraw suppression, and cleanup without
  opening the application entry point or a serial transport. A mapped real-Tk
  integration test verifies sibling stacking, explicit layer reassertion,
  overlay cleanup from a grid-managed scale, and the global release binding
  shared by sibling widgets on display-capable test hosts; the test skips
  explicitly when no Tk display exists.

### M4A6 - Main-control workspace and named positions

Status: `Tested`

Control contract:

- Joint, Cartesian, and Tool Frame controls occupy separate tabs in the main
  control workspace. J1-J6 use a vertical stack so every joint receives the
  full available slider width.
- Cartesian controls use matching vertical current-position and jog rows.
  Cartesian sliders remain unavailable because the reachable Cartesian set is
  configuration-dependent and cannot be represented by independent fixed axis
  bounds.
- Tool Frame controls use matching vertical relative-jog rows. Absolute sliders
  remain unavailable because tool-frame jog represents signed displacement
  along the moving tool axes rather than an absolute tool-frame position.
- The encoder-sample marker uses a wide amber body while the estimated marker
  uses a narrow bright-cyan body. A shallow violet active-target marker
  distinguishes the executing endpoint from newer desired input. All markers
  retain contrasting outlines, deterministic stacking, pointer forwarding,
  and desired-slider interaction.
- `Start Position` submits the canonical post-calibration J1-J6 target
  `(0, 0, 0, 0, 45, 0)`.
- `Shutdown Position` keeps J1 and J4-J6 at the canonical start values and
  uses the J2/J3 switch coordinates reported by the connected Teensy after
  successful homing under the active controller frame. Each requested
  reference is
  invalidated before calibration motion; the fast search requires debounced
  switch confirmation, backoff stops each axis after a three-millisecond stable
  release and fails when release exceeds the lesser of ten configured axis
  units and full configured axis travel, the slow search must confirm the
  switch again, and the new reference commits only after centered motion
  succeeds. Empty calibration selections fail before any drive call.
  Controller parameter updates and
  forced-position writes
  invalidate both the firmware reference and the matching host binding after
  write commitment. Startup synchronization performs those invalidating
  commands before querying the controller, so J2 and J3 require fresh homing
  before Shutdown Position becomes available. Missing, stale, malformed, or
  out-of-range references reject before motion admission. Every terminal
  calibration disposition refreshes the host reference after the host mirror
  invalidates at `LL` write commitment; a malformed reference response
  quarantines the controller, stops a multi-stage sequence, and requires
  reconnection. The host prefers the `HOME_REFERENCE_V2` `H2` exchange, which
  reports J1-J3. The legacy `HOME_REFERENCE_V1` `HR` exchange remains
  supported for older controllers, but the missing J3 reference leaves
  Shutdown Position unavailable. Post-calibration reference exchange preserves
  pending input so an unexpected queued frame fails the owned protocol boundary
  instead of being discarded. Shutdown Position cannot enter the generic
  deferred queue while an unrelated motion request can invalidate the captured
  controller reference.
- Online named positions enter the semantic joint dispatcher as one partial
  multi-axis absolute target. J7-J9 remain unchanged, an active joint move
  retains only the latest named target, and unrelated owned motion retains the
  complete named target for later dispatch from confirmed controller state.
  Offline mode supports the canonical Start Position through the virtual drive
  and rejects the hardware-specific Shutdown Position.

Acceptance criteria:

- Main-control coordinate tabs and vertical axis ordering have source-contract
  coverage without importing the application entry point.
- Start and shutdown targets are validated against active calibration before
  admission.
- `HOME_REFERENCE_V1` and `HOME_REFERENCE_V2` framing, paired host/firmware
  invalidation, post-homing commit, startup capability discovery, protocol
  preference and fallback, invalid-state synchronization, and host parsing have
  deterministic coverage. The start target remains checked against `Home.ar4`.
- Atomic partial-target submission, deferred-target replacement, external-axis
  preservation, slider-marker geometry, and pointer routing have deterministic
  hardware-free coverage.
- Live-arm verification remains a separate M5 procedure and is not inferred
  from HMI rendering or mocked transport results.

### M4A7 - Low-priority joint encoder telemetry

Status: `In progress`

Protocol contract:

- `JOINT_TELEMETRY_V1` is optional. A connected controller must advertise the
  capability before the host appends `T1` to a coordinated `RJ` command.
  Controllers without the capability retain the estimator-only path.
- `T1` is request-scoped. Telemetry acquisition stops after the owning `RJ`
  exchange, and no other motion opcode can enable the stream. The HMI may
  retain the latest validated sample only while the same open,
  non-quarantined controller identity remains bound.
- The Teensy samples the encoder-backed primary joints at a ten-hertz target
  cadence and emits one bounded ASCII frame containing signed J1-J6
  millidegrees. J7-J9 have no matching encoder source and remain command
  estimates.
- Telemetry formatting uses fixed stack buffers and integer fields. A frame is
  attempted only when the USB transport reports capacity for both the frame and
  a reserved terminal response; unavailable capacity drops the sample without
  retrying or delaying terminal and fault ownership.
- A telemetry-enabled `RJ` claims main-loop response ownership before encoder
  reset and coordinated drive, retaining ownership through terminal framing.
  An E-stop interrupt still latches motion stop immediately but defers serial
  output while that owner is active. The drive commits local step progress,
  reconciles the encoders, and emits the selected terminal response from the
  owning path. `EB` identifies the asynchronous physical-stop event. A stop
  deferred after terminal selection is atomically retained as an admission
  block during ownership commit and emits `EB` immediately after the selected
  terminal frame. Before parsing or output activation, the next queued command
  is consumed and receives a correlated `EA` rejection. A released stop clears
  after that rejection; an asserted stop continues rejecting commands.
  Speculative spline acknowledgements remain outside this contract.
- Host startup treats a correlated `EA` response to identity negotiation as a
  released-latch handshake and performs a bounded `HO` retry on the same serial
  handle; another non-identity response fails startup. Joint-reader quarantine
  detaches a closed main handle and clears matching controller identity before
  dispatcher ownership releases.
- Telemetry work occupies the existing high-pulse wait and the measured work
  duration is deducted from the remaining wait. No telemetry write flushes or
  resets serial state. Hardware verification must still measure the worst-case
  pulse interval because a task exceeding the available wait would extend one
  scheduler interval.
- The host serial owner classifies every received line as telemetry or an
  allowed coordinated-move terminal while preserving the original command
  deadline. Physical-stop `EA` and `EB` frames are classified and published
  before any operation-specific interim callback can inspect the frame. Any
  malformed or unknown frame quarantines the transport before ownership
  release. A response budget derived from that deadline and the negotiated
  cadence quarantines a controller that exceeds the allowed interim volume.
  The existing terminal position frame remains authoritative for confirmed
  pose, collision status, speed-violation state, and queue rebasing.
- While the main controller has no transport owner, the HMI checks only
  pySerial's local queued-byte count on the existing Tk serial-poll cadence.
  Queued data receives exclusive emergency-control ownership and a bounded
  worker read. Ownership remains active through Tk result application, so
  shutdown cannot persist or close ahead of the result. The monitor sends no
  command, adds no Teensy polling load, remains admissible while shutdown
  drains, and accepts only a standalone physical-stop `EB`; any other idle
  frame quarantines and closes the connection. Stop publication precedes
  auxiliary-stop gating and transport quarantine.
- Final shutdown persistence and serial closure remain blocked while a generic
  main-controller stop event is queued or retained after a failed Tk
  presentation.
- Telemetry callbacks use a bounded latest-sample slot, separate from
  ordered lifecycle and terminal events. Tk applies only the latest observed
  sample to the display and performs no serial work.

Acceptance criteria:

- Host parsing, request-marker admission, interim-line demultiplexing,
  malformed-frame quarantine, response-budget enforcement,
  serial-versus-virtual suffix separation, bounded latest-sample coalescing,
  dispatcher ordering, independent encoder-versus-estimate overlays, and
  stale-sample cleanup have deterministic tests. Stop frames cannot be consumed
  by an interim callback, and zero-write idle `EB` monitoring is covered with
  busy-transport, shutdown-admission, invalid-frame quarantine, and ownership
  settlement cases.
- Native sanitized checks cover encoder conversion, atomic validation,
  millidegree rounding, frame bounds, framing, wrap-safe cadence, and
  response-owner state transitions.
- Source-contract checks cover capability negotiation, RJ-only enablement,
  USB backpressure checks before encoder reads and before transmission,
  terminal-capacity reservation, fixed-buffer formatting, pulse-wait
  accounting, monitor commitment before encoder reconciliation, and
  telemetry/E-stop response-owner handoff, including immediate post-terminal
  `EB` publication for a late stop.
- The tracked Teensy source compiles without upload for Teensy 4.1 with the
  pinned toolchain.
- Hardware verification remains pending for observed cadence, encoder accuracy,
  USB load, terminal priority, and pulse-timing behavior across representative
  speeds and simultaneous-axis moves.
- During drive-off deployment on 2026-07-29, Arduino CLI returned exit code
  `0`, and the re-enumerated Teensy 4.1 hardware identity `1705B6`
  self-reported version `6.7.1-ar4hmi.5`. Live `HO` and `H2` responses reported
  the required capabilities and invalid J1-J3 home references without an
  operator-issued motion command. Those responses do not
  cryptographically bind the running binary to the cited source commit or a
  transient build artifact. The dated procedure and observations are recorded
  in `docs/hardware-verification-2026-07-29.md`. Powered M4A7 cadence,
  accuracy, load, priority, and timing observations remain pending.

### M4B - Repeatability and dynamic interception pass

Status: `Proposed`

Research direction:

- Timestamped perception and calibrated frame transforms.
- State estimation with position, velocity, acceleration, and covariance.
- Future-state prediction and reachable intercept selection.
- Global planning plus fast local replanning or visual servo control.
- Synchronized velocity, acceleration, and jerk constrained trajectory generation.
- Measured acceleration and deceleration profile tuning from controller traces
  before changing the current motion curves.
- A non-blocking controller interface supporting replaceable setpoints, state feedback, hold, cancel, watchdog, and fault states.

Acceptance criteria:

- Accuracy, repeatability, latency, speed, payload, workspace, and grasp thresholds are measurable.
- A deterministic simulator and recorded-observation replay path exercise estimator, predictor, feasibility, and replanning behavior.
- Stale data, uncertainty, collision, singularity, joint-limit, controller-timeout, and failed-grasp behavior are explicit.
- Hardware experiments follow M5 and distinguish observed results from simulation.

Research and staged decisions are tracked in `docs/dynamic-motion-research.md`.

### M5 - Controlled hardware validation

Status: `Blocked`

Blocking condition: explicit work-envelope confirmation and powered
verification of the independent hardware stop path are required before another
powered M5 procedure.

Acceptance criteria:

- Authorized procedure identifies firmware, configuration, start pose, speed limits, expected motion, abort conditions, and recovery steps.
- Physical emergency stop is tested before commanded movement.
- Results distinguish observed hardware behavior from software-only evidence.
- Deviations become tracked requirements or defects before broader operation.
- Commissioning verifies physical driver microstep settings or measured motion
  scale against the active profile before accepting a calibration reference;
  matching host and firmware configuration alone is insufficient.

Initial commissioning evidence is recorded in
`docs/hardware-verification-2026-07-22.md` and
`docs/hardware-verification-2026-07-23.md`. The first powered J1 calibration
exposed a factor-of-two driver-microstep mismatch after the software upgrade.
After all six physical driver settings were updated to match the active
per-joint profile, including J5 at `1600` microsteps and the other primary
joints at `800`, operator-observed single-joint calibration and apparent jog
scale passed for J1-J6. A rapid jog sequence also demonstrated responsive HMI
input during motion and simultaneous physical J1-J3 execution, but the
unrecorded input sequence did not verify exact deferred-target coalescing or
controller command count. An approximate absolute-slider return followed
without an accepted final pose. Instrumented accuracy, repeatability, speed
characterization, and fault response remain unverified.

Known M5 deviations:

- Hardware emergency-stop availability was checked by engaging the stop with
  drive power off. No controller E-stop frame, HMI alarm, or powered
  interruption independent of desktop GUI state was recorded before commanded
  motion. A cleared work envelope was confirmed earlier in commissioning, but
  no separate confirmation was recorded immediately before powered checks. The
  powered observations remain exploratory evidence rather than
  acceptance-criterion completion.
- After the initial J1 scale failure made the reference untrusted, subsequent
  diagnostic position and jog commands departed from the fail-closed
  calibration invariant. Motion then stopped until the physical driver
  configuration was corrected and J1 was recalibrated.
- The coordinated-motion procedure changed from scripted absolute-slider
  targets to rapid incremental jog input. The resulting observation established
  responsive input and physical multi-axis execution but did not verify exact
  target coalescing or controller command count.

## Architectural decisions

- Preserve the current directory layout until startup and asset-path dependencies are isolated.
- Keep mutable calibration, captured images, error logs, and review artifacts outside version control.
- Treat host commands, firmware parsers, native kinematics bindings, and `.ar4` programs as versioned integration contracts.
- Queue semantic targets, not raw input events. Joint, Cartesian, and tool-frame intents cannot be merged across coordinate spaces without recomputation from confirmed state.
- Keep desktop command coalescing separate from future real-time servo and trajectory-control loops.
- Retain the current hardened baseline and use delivered 7.0/2.0 sources only as isolated selective-integration input under M4A4.
- Route post-bootstrap commits through the role-appropriate cross-review wrapper. After Claude usage capacity is confirmed exhausted, the explicit first-position Codex-author `-NoClaude` route preserves mandatory fail-closed Codex review and records the fallback locally; missing authentication must be repaired, and reviewer failure never causes automatic substitution.
- Route branch integrations through `scripts/codex/auto-merge.ps1`; bare merge into the integration base is prohibited.

## Current implementation boundary

- Incremental J1-J9, absolute slider routing, offline external-axis rejection, and live-jog desktop wiring are statically or behaviorally checked without importing `AR4.py`; dispatcher coalescing, deferred intent ordering, controller and six-axis simulator command schemas, framed responses, optional protocol-authorized follow-up frames, full quiet-boundary acknowledgements, trailing-byte quarantine, retained failed-close ownership, configured Linux and Windows Xbox validation, watchdog scheduler failure, startup acknowledgement variants, startup finalizer rejection, set-position acknowledgement sequencing, live-stop injection and request ownership, pre-write cancellation admission, bounded-command timeout quarantine and restoration, cancellation-bound G-code playback, nonblocking G-code directory and deletion handoff, request-scoped virtual results, late virtual settlement, virtual-worker failures, program-worker startup failure, long Seconds-mode virtual deadlines, direct program and calibration response propagation, non-blocking calibration button dispatch, terminal-response-required calibration shutdown, calibration transport preflight and write-boundary recovery, binary and finite calibration validation, combined automatic-calibration preflight, native tool-frame order, offline position synchronization, forced calibration-position rejection, completion-only G-code navigation, legacy and dispatcher transport-release ordering, shutdown waits for virtual and retained cleanup owners, nested direct-operation transport admission, asynchronous worker reservation transfer and rollback, direct-operation shutdown tracking, applied program-motion results, G-code local stop admission, post-release collision correction, interruptible auxiliary-wait stop handoff with a shared post-transmission acknowledgement deadline, optional auxiliary startup, controller-buffer-reset failure handling, combined close-and-scheduler cleanup failure, startup timeout handling, non-preempting program-halt status, and response ownership are tested with deterministic fakes.
- Coalesced joint-dispatch cleanup attempts serial-activity lease release before
  transport-lock release and always attempts both components. A failed lease
  remains retained for retry while successful transport-lock release keeps
  controller position recovery admissible. A newly observed unsuccessful
  component discards uncommitted targets, latches a queue fault, and publishes a
  transport-failure event. Worker cleanup publishes before inactivity becomes
  visible, including an idle exit without a move. Close reports incomplete
  cleanup without raising; application shutdown retries cleanup before
  registry-idle gating and continues closing independently when no registry
  ownership remains. An unchanged retained-release failure does not enqueue
  another transport-failure event. Active-worker, retained-fault, invalid-state,
  and cleanup-exception shutdown states each publish one bounded diagnostic.
  Fresh submissions reject under that fault; confirmed-position synchronization
  retries retained ownership before clearing the fault and reports retry failure
  explicitly.
  Initial admission records acquired transport ownership before serial-activity
  setup. A failed admission rollback therefore remains latched and retryable
  through confirmed-position synchronization.
  Position-response rejection distinguishes an active worker, a closed
  dispatcher, and retained ownership. Transport-release events use a dedicated
  ownership-failure alarm rather than reporting a completed move as failed.
- Exact J1-J6 keyboard targets share the absolute semantic queue used by
  sliders. Active text edits survive controller and virtual position refreshes;
  accepted submissions resume confirmed-position display updates, and
  cancelled or abandoned edits restore the latest confirmed position.
- Negotiated `JOINT_TELEMETRY_V1` coordinated-joint exchanges demultiplex
  request-scoped J1-J6 encoder samples into a dedicated amber Tk overlay while
  the cyan J1-J9 command estimate and violet active target remain separate.
  The amber marker may retain the latest encoder sample while idle only while
  the same open, non-quarantined controller identity remains bound; startup and
  terminal step-counter positions never populate that channel. The retained
  sample clears on controller trust, identity, or quarantine loss and when a
  coordinated-joint terminal result cannot validate controller position.
  Source reads use an immutable snapshot without controller or persistence
  locking and cannot wait on G-code persistence. Samples are dropped under
  firmware USB backpressure, never update confirmed host state, and never
  create a J7-J9 encoder marker. Hardware timing and encoder accuracy remain
  unverified.
- Simple program movement routes now share request-scoped controller ownership, raw target validation, and offline physical-write rejection. The broader typed program state machine, non-motion row migration, typed response schemas for remaining program branches, Cartesian and tool-frame coalescing, application lifecycle separation, and dynamic controller work remain incomplete.
- Program-row main-controller exchanges now classify physical-stop position
  frames before command-specific parsing. A reported stop latches host stop
  state, queues Tk position application, invalidates retained joint intent,
  closes the controller connection, and rejects the row before later
  execution. The worker-side physical-stop latch synchronously sets the active
  program cancellation event before Tk presentation. Run and Step modes
  check request-scoped cancellation before row dispatch. A cancelled pending
  Step Reverse callback aborts row state and releases the matching execution
  owner. Called-program sources use bounded regular-file loading and leaf-only
  `.ar4` names. Workers enqueue validated rows for request-scoped Tk
  application and wait for the matching acknowledgement; the cancellation
  boundary orders admission but is released before widget access. Manual
  `.ar4` loading uses the same bounded source contract, and failed view
  replacement restores the captured program-view state. Because program
  navigation retains one return frame, a nested called-program request rejects
  before program-view mutation. Manual top-level loading and new-program
  creation and manual reload clear that frame; explicit cancellation, including
  physical-stop latching, requests a Tk-owned frame clear before later program
  admission, while normal Step Forward completion retains the frame for later
  continuation. Called-program navigation rechecks cancellation after Tk view
  replacement and after replacement rollback, so concurrent stop admission
  cannot restore an abandoned frame. Every `executeRow`
  command branch checks cancellation before local
  effects, while main and auxiliary transports recheck immediately before
  transmission. Run and Step admission rejects an active physical-stop latch,
  position-fault latch, or unsettled stop request without clearing fault state
  or allocating a program owner. `.ar4` `BH` and `BD` register reads use the
  canonical controller builder. Scalar-register rows remain supported; a
  legacy multi-register row is rejected before transmission with an explicit
  scalar-row migration instruction. Only a bound, fault-free startup
  position clears fault latches; pending stop settlement remains blocking.
  Application status updates pass through a stop-aware presenter, so later
  serial, joint, virtual, calibration, program, or auxiliary result polling
  cannot replace a reserved E-stop or stop-settlement alarm.
  Manual, Xbox, and program auxiliary commands share a write-boundary stop
  check under the manual auxiliary state lock. Physical-stop barrier
  publication acquires that same boundary before cancellation and reservation,
  so an ordinary output write commits wholly before the barrier or rejects
  after the barrier.
  Auxiliary profile commits and physical-stop latching share a snapshot
  boundary, and shutdown cannot persist or close while generic stop
  presentation remains queued or retained.
  Request-scoped program cancellation and startup timeout cancellation share
  their final serial write-admission boundary with command commitment. Direct
  controller rows, chained program motion, virtual-preview handoff, and `PG`
  playback therefore reject cancellation before commitment while retaining
  response ownership after a committed write. Every main application-status
  label mutation passes through the stop-aware presenter.
  Program-row acquisition accepts exactly one non-negative selected index and
  performs strict UTF-8, single-line decoding before command classification.
  Selection, size, row-read, and decode failures abort row ownership, publish an
  alarm, and return `ROW_EXECUTION_REJECTED`; no failure is converted into `Stop
  Program` or another executable row. Cosmetic scroll failures are logged and
  do not change row execution. Blank rows, `##` comments, and numeric `Tab
  Number` labels containing the same decimal character class accepted by tab
  lookup are explicit structural no-ops. Tab-label, direct-jump, and conditional-
  jump builders normalize that value and reject invalid input before inserting
  or persisting a row. Every other row must match a currently implemented
  command prefix or is rejected before any command branch runs.
  Legacy `Out On = N` and `Out Off = N` rows have no executable handler and are
  rejected instead of completing as silent no-ops.
  Program `SC` and `SO` rows use the canonical controller builder before
  transmission. The framed exchange owner validates `-1` and `-2` through the
  same command-aware terminal classifier used for final disposition. A `-1`
  or unrecognized post-write terminal records
  indeterminate external-device state, quarantines the main connection,
  invalidates pending motion, stops program advancement, and requires
  reconnection plus device-state verification; `-2` remains a pre-write
  controller rejection.
  Broader non-motion row migration remains incomplete.
- Tracked Teensy `6.7.1-ar4hmi.9` `MA` and `MC` parsing stages `Tr` in a command-local value instead of writing beyond the Cartesian pose array and accepts only the supported zero value. `ML` accepts only `Q0` because wrist-suppression semantics remain unimplemented. `RB` performs the Teensy 4.1 target reset without consulting mutable controller identity fields and emits no terminal frame before reset. Current no-upload toolchain evidence is recorded in [`docs/hardware-free-verification-2026-07-19.md`](docs/hardware-free-verification-2026-07-19.md). Host arc and circle program transmission remains disabled until deterministic trajectory simulation and an authorized hardware-validation plan cover the broader algorithms. Spline program transmission also remains disabled until a terminal response-owner contract replaces speculative acknowledgements.
- Teensy coordinated joint pulse scheduling now deducts bounded telemetry work
  from the existing pulse wait. A telemetry-enabled `RJ` also owns response
  framing across the coordinated drive, so an interrupt latches the stop while
  the drive commits local progress, reconciles encoders, and emits the selected
  terminal response. A loop-scoped owner now brackets every main-loop terminal
  writer, and the E-stop interrupt only latches assertion and pending-response
  state. Main-loop completion emits `EB` at a safe frame boundary, preventing
  ordinary terminal text from being spliced by interrupt-context USB output.
  A stop deferred after telemetry terminal selection becomes an admission
  block during atomic ownership commit and produces an immediate post-terminal
  `EB`. The
  `6.7.1-ar4hmi.9` protocol reports asynchronous physical-stop events as `EB`
  and correlated command-admission rejection as `EA`. Queue-boundary and
  post-parse gates snapshot the stop state with interrupts disabled; the `EA`
  response reserves interrupt framing through atomic loop-response teardown,
  and generation comparison prevents an older response from clearing a newer
  assertion. Joint and legacy exchanges
  preserve unread input, classify stop frames before operation-specific interim
  handlers, and own bounded ordinary-position and `EB`/`EA`/terminal
  orderings. A telemetry exchange with `EB` but no immediately available
  follow-up quarantines and verifies closure before ownership release. Stop
  publication remains mandatory, and Tk pose application is correlated to the
  source connection epoch and confirmed-position generation. Unmatched framing
  is quarantined. A zero-write idle monitor consumes a queued standalone `EB`
  under exclusive emergency ownership, including during shutdown, publishes
  and quarantines the stop before Tk result transfer, then releases transport
  ownership before the separately queued stop presentation. Shutdown still
  blocks final close until that presentation settles, and reconnection remains
  required; no controller polling command is added. Speculative
  spline acknowledgements remain a multi-frame ownership hazard. A
  non-telemetry drive routine can emit an ordinary terminal before the
  safe-boundary `EB`; the live owner probes under its configured response
  bound, publishes the paired `EB`, and treats the stop as authoritative
  before returning or quarantining malformed trailing data. The missing-file
  behavior has a
  source-contract assertion, and the current line-oriented compatibility source
  passes a Teensy 4.1 syntax/toolchain compile. That compile does not establish
  live-arm behavior or satisfy the later correlated JSON safety prerequisite;
  simulation and live-arm verification remain pending.
- M4A3 found that delivered 7.0 retains busy-input loss, blocking Tk paths, incomplete response ownership, firmware array-bound defects, and emergency-event races. Selective integration is approved because delivered inverse-kinematics, MK5 calibration-switch, JSON auxiliary-controller, and optional CAD/EOAT changes can be isolated without replacing the hardened HMI baseline.
- Authorized controller flashing, drive-off communication, HMI startup, and
  exploratory powered single-joint calibration, jog checks, and simultaneous
  J1-J3 motion have been performed and recorded, followed by an approximate
  absolute-slider return without a captured final pose. Known M5 deviations
  prevent those observations from satisfying the controlled-hardware
  acceptance criteria. Broader live-arm behavior remains unverified outside the
  documented observations.
