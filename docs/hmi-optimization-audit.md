# HMI Optimization Audit

## Scope

This audit covers desktop responsiveness, command ownership, incremental jogging, serial response handling, configuration persistence, and virtual-robot rendering. No attached controller or robotic arm was used.

Tk serializes event callbacks on the interpreter thread. Long event handlers block input and redraw processing, while cross-thread widget calls depend on that interpreter event loop. The desktop architecture therefore needs short Tk callbacks, blocking I/O outside the Tk thread, and result application on the Tk thread. See the [Python Tkinter threading model](https://docs.python.org/3/library/tkinter.html#threading-model).

## Implemented HMI work

- `controller_degree_to_native_radians` supplies shared binary32 degree-to-radian and round-trip representability checks for motion envelopes, `UP` construction, custom-profile validation, startup native preflight, and the paired Teensy conversion contract. Native-rejected public-degree boundaries fail before serial writes or profile persistence.
- `ARrobots/HMI/joint_motion.py` owns validated J1-J9 semantic intent state. Dispatcher command transmission is serialized, and ownership remains reserved until Tk applies the result and the shared transport lock has been released. Tracked direct main-controller transactions reserve the same nesting-safe admission lock; asynchronous program motion transfers an existing reservation to the response worker. Button deltas accumulate against the latest desired target, slider positions replace the selected axis target, and changes across axes merge into the latest pending absolute target.
- Joint inputs entered during an active legacy move retain ordered delta and absolute-target semantics. After the legacy response is applied on the Tk thread, relative changes are rebased on the newly confirmed position, later slider targets replace earlier intent for the selected axis, and a consolidated absolute multi-axis target is dispatched. Transport admission remains reserved through matching terminal-result application, then Tk releases the lock and retries retained input. That boundary permits an intent recorded after the response to use the current confirmed generation. Transient transport contention retains deferred intent, while permanent validation or queue rejection clears stale intent. A failed or invalid legacy response leaves the dispatcher faulted until a later valid position resynchronizes state.
- Deferred joint resolution, dispatcher admission, and consumed-state clearing share synchronized locking. Concurrent input enters the admitted snapshot or remains pending for the next confirmed generation, and dispatcher worker startup completes or rolls back before another submission can receive acceptance.
- A valid controller position response advances the queue. Direct program rows, automatic and single-joint calibration, position requests, offline-to-online synchronization, external-axis zeroing, and forced calibration poses now propagate the applied-position result; success labels, later calibration stages, row completion, and virtual-position copies remain blocked after malformed data, a controller error, an embedded motion fault, or dispatcher rejection. Automatic calibration uses a terminal-disposition-required single-frame exchange owner because line-oriented `LL` has no host abort command. Shutdown and the final calibration write-admission check share a lifecycle lock: shutdown before write commitment rejects transmission, while a committed write latches terminal ownership. Post-write shutdown retains supervision until an applied terminal response or explicit quarantine and verified-close handling settles possible calibration motion. A malformed terminal frame, queued second frame, non-position response, controller fault, or result-application failure prevents a later stage from starting. Result-application rejection restores captured pose, widget, generation, and virtual state; cancels changed debounced persistence; schedules the restored state or retains dirty state after scheduler failure; invalidates motion state; and quarantines the transport before ownership release. Controller-bound `RJ` and `MJ` timing schemas require J7-J9, while simulator-bound `RJ` and `MJ` use separate exact J1-J6 and Cartesian envelopes and legacy `JT` retains an explicit simulator contract. Controller and simulator paths capture and binary32-range-check every numeric envelope, timing, and numeric-suffix field; controller-bound commands replace each admitted value with delimiter-safe decimal before transmission. Target-bearing raw `RJ`, `MG`, `MJ`, `ML`, `MV`, `WC`, and `WG` commands additionally validate every included J1-J9 target against the active calibrated limits and signed step-counter range. Exponent notation is rejected because firmware marker parsing cannot distinguish exponent letters from field delimiters. A timeout or timing profile whose acceleration and deceleration regions overlap is rejected before pending motion can advance.
- Serial workers emit operation-local terminal results for Tk-thread application through operation-specific event pollers. Legacy transport admission and the matching shutdown-activity lease remain held until the corresponding main-thread result has been applied; Tk then releases both and invokes the operation-local callback without shared success state. That callback settles any request-scoped motion lease before deferred joint intent is retried. Joint-dispatch and startup leases cover the complete asynchronous ownership interval through acknowledgement or connection cleanup. Main-controller replacement is admitted during position recovery and retains the logical lease alongside the startup transport and activity leases until a fault-free position applies or durable cleanup closes the failed connection.
- Owned framed-response readers remove only a required LF and an optional preceding CR. Validation separates the 4096-byte payload limit from the 4098-byte CRLF-capable frame limit, so a maximum-length payload remains valid with either supported delimiter. Leading or trailing payload whitespace, extra delimiters, non-ASCII bytes, and queued data outside an operation-specific follow-up contract fail the protocol contract and quarantine the affected transport. Terminal ownership requires a complete bounded quiet interval; insufficient remaining response time quarantines the transport instead of treating a shortened read as proof of framing silence. Host parsing accepts a validated controller-initiated `EB` position as a physical-E-stop terminal frame. The active Teensy `6.7.1-ar4hmi.3` derivative retains baseline serial response work from `EstopProg` and speculative spline acknowledgements from `processSerial`, allowing an interrupt race to produce multiple terminal frames. No E-stop firmware rewrite is included in the current pass. Safe single-frame ownership requires a separate controller-protocol design followed by compilation, simulation, and authorized live-arm verification.
- Live joint, Cartesian, and tool jog callbacks no longer sleep or read serial data. The existing worker owns the live command, consumes the firmware's framed blank acknowledgement, observes a thread-safe stop request, writes `S\n`, and consumes the final position response. Firmware polls a byte between live segments and accepts only an exact LF- or CRLF-terminated `S` control frame; an overlength or other complete frame exits through one error response instead of being treated as a successful stop. Initial acknowledgement uses the dedicated `SERIAL_LIVE_ACK_TIMEOUT_SECONDS` watchdog and post-token completion uses the command-specific deadline derived from encoded timing and configured full-travel bounds. Seconds and millimetres-per-second selections are normalized to Percent for live modes because those units do not provide predictable stop-polling cadence. The response deadline remains suspended while the acknowledged control remains held, but every empty read still verifies that the port remains open. Closure or cancelled-read termination enters fail-safe stop and quarantine handling instead of retaining ownership indefinitely. Cancellation admission is rechecked under the serial write lock after stale-input reset; cancellation before transmission writes neither the live command nor `S\n`. Premature terminal data or another framing-uncertain failure after transmission writes the stop token where supported, quarantines the transport, attempts a verified close, and requires an explicit reconnect. Failed physical close retains the poisoned handle under application cleanup ownership. Temporary per-read timeout restoration is attempted across return and exception paths, and restoration failure triggers the same quarantine. pySerial documents timeout-controlled line reads and `close()` changing `is_open` to false in the [short introduction](https://pyserial.readthedocs.io/en/latest/shortintro.html).
- Online joint input updates the coalesced virtual target, and confirmed controller responses resynchronize the virtual model. If a confirmed joint result cannot update the virtual model, the dispatcher and deferred intent are invalidated before result acknowledgement, the warning remains visible, and no pending target starts until a later fault-free position response resynchronizes both models. Manual online Cartesian and tool-frame motion captures the last confirmed virtual pose before preview dispatch. A failure before the physical write boundary restores that pose; a failure after transmission starts preserves uncertainty and blocks new operator and program motion until a fault-free controller position response resynchronizes the virtual model and joint dispatcher. Controller fault correction remains an explicit recovery-only exception. Physical success applies the confirmed controller pose even after virtual preview failure. Independent virtual drive workers run in offline mode and publish request-scoped terminal results. Offline incremental and live J7-J9 input raises a stable alarm because the virtual model contains robot axes J1-J6. Offline live-jog workers reserve a shared request slot before launch, synchronize stop requests against discrete-segment admission, and wait for explicit segment success or failure. The request-scoped operation remains authoritative until matching Tk settlement, so a rejected or invalid later request cannot clear active state and release still signals the active stop owner even when `RUN['liveJog']` is stale. Tk refreshes the final virtual state only after the complete offline request settles, preventing a released worker from observing a later press as permission to resume or updating state after the stop snapshot. Tool motion snapshots and validates the active calibration frame on Tk in native `(x, y, z, rx, ry, rz)` order before worker launch; unsaved editable widget values remain staged, and virtual-motion errors return through a Tk-polled event queue.
- Simulator `MJ` millimetres-per-second timing captures a validated six-value Cartesian start pose from forward kinematics and a validated target pose before virtual worker admission. Translation distance uses those explicit endpoints; missing, malformed, or non-finite endpoints reject before virtual motion state changes.
- `DeferredLiveMotionArbiter` gives Windows Xbox joint, Cartesian, and tool control generation-scoped admission. A busy start retains the latest held semantic value for retry, active state commits only after the matching Tk live-jog callback accepts that value, and failed stop or scheduling admission retains pending intent with an explicit diagnostic. A deferred inner Tk registration failure clears scheduled-attempt state before reporting the diagnostic, allowing later held input to retry. Application-closing cancellation before or after Tk admission clears pending scheduling without a false alarm. The Xbox Teach callback also records Tk scheduling rejection. Windows polling exits on application shutdown. Watchdog scheduling failure disables further Windows polling, signals online and offline live-stop events, requests every arbiter to stop, and records an alarm. Linux checks controller-off and shutdown state after each blocking `get_gamepad` return before processing any returned event.
- `_mode_change_is_blocked` covers controller-live, legacy-serial, dispatcher, transport-lock, virtual-drive, live-mode, and offline-settlement owners. Offline-to-online transition applies a fault-free controller position before changing mode, button, status, or virtual-position state; failed synchronization restores the canonical offline presentation and preserves the virtual snapshot.
- Offline manual, joint, program, and live settlement retains request ownership while applying the terminal six-joint pose or restoring the saved pose after failure. FK-derived calibration and widgets refresh within that interval; refresh failure rejects settlement. Virtual drive workers do not perform Tk refresh work.
- Incremental J1-J9 handlers share `_queue_joint_jog`; shadowing synchronous J7-J9 implementations were removed.
- `displayPosition` validates the complete firmware response contract and all inbound J1-J9 values against current calibrated limits and signed step-counter bounds before updating calibration, widgets, persistence state, or virtual state. The accepted frame requires ordered A-R markers, delimiter-safe decimal fields, the firmware speed bit, an empty or numeric debug field, and a blank, `EB`, or six-bit `EC` fault field. Duplicated reserved markers and arbitrary debug or fault payloads fail before state mutation. Dispatcher responses receive the same validation against the immutable calibration used to serialize the matching move before a completion or fault event can carry a position. External responses must resynchronize an idle joint dispatcher before state mutation or generation advancement; dispatcher-owned completion events suppress only that external synchronization. Position consumers additionally require a fault-free applied result before reporting success or advancing dependent state. Joint serialization rejects values outside the controller's single-precision range, configured axis limits, or signed step-counter range.
- Error rendering no longer performs controller reads on Tk. Collision recovery records a pending controller correction, waits for dispatcher or legacy ownership to release, and sends `CP\n` through the main serial worker. An absent, closed, or quarantined main transport leaves that request queued without starting repeated workers. Program-stop recovery records a request ID and retains pending status until a matching terminal result; explicitly unconfigured auxiliary I/O completes as not required, while configured-but-unavailable hardware remains a diagnostic failure. Status text reports interpreter scheduling separately and warns that active main motion is not preempted. Only the auxiliary firmware's interruptible `WI` wait allows write-only `STOP\n` injection while the wait operation retains sole response ownership. `Nano Stopped` acknowledges the injected stop. Immediately after successful stop-token transmission, one absolute monotonic deadline based on `SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS` replaces the original `WI` deadline. Wait-owner handoff, optional owner-side follow-up, serial-lock acquisition, stop-worker follow-up read, and the required quiet boundary all consume that same deadline. When a natural `Done` or `Timeout` and `Nano Inactive Stopped` are already queued, the wait owner validates both frames before releasing ownership and publishes the inactive-stop acknowledgement to the stop worker. When the inactive acknowledgement arrives later, the stop worker performs the bounded follow-up read after wait ownership releases. Missing, unexpected, or additional follow-up data quarantines the auxiliary transport. Other auxiliary operations retain the stop request until an exclusive bounded exchange returns `Nano Inactive Stopped`. `WI` timeout input is validated against the firmware integer range, line-read ownership uses the encoded duration plus a margin, and empty or unexpected terminal data quarantines the auxiliary transport. Servo and output rows require bounded `Servo Done` and `Done` acknowledgements followed by a quiet read boundary; empty, partial, unexpected, or trailing unframed data quarantines the auxiliary transport. Normal auxiliary commands reset stale input before transmission and share the stop-token write lock, so concurrent writes cannot interleave and a post-write reset cannot erase a stop response. Windows and Linux Xbox gripper paths use the same bounded exchange validation. Every program, manual, and Xbox `ON`/`OF` command is validated against an explicit Nano or Mega profile bound to the active auxiliary serial handle; an unselected or stale profile blocks output transmission. Nano accepts outputs 8-13, Mega accepts outputs 28-53, and each profile rejects the complete opposing range. Windows pneumatic toggles use profile-valid output 8 for Nano and 28 for Mega. Closing or replacing the handle clears the binding, and existing configurations default to no selected board until an operator chooses a profile. Windows requests preserve the last acknowledged toggle state on transport rejection, update state on Tk only after the expected acknowledgement, and mark state unknown after an invalid exchange while attempting a verified auxiliary close. Linux performs the exchange synchronously in the controller worker and updates local grip state only after acknowledgement.
- Position-triggered calibration writes are debounced. Successful auxiliary port or board-profile changes enter the same dirty-state persistence path, so a restart restores the committed selection. A failed write retains dirty state and schedules another attempt while the application remains open. Shutdown blocks new tracked direct serial operations, requests online and offline live-stop paths, and grants active readers a bounded drain interval. Each closing poll drains terminal events and evaluates overdue tracked activity before logical or virtual motion ownership can defer the poll. Overdue readers, including pre-write calibration activity, receive `cancel_read`; a committed main-controller calibration exchange remains supervised because closing that transport cannot preempt Teensy `LL` motion. Shutdown remains pending until an applied calibration terminal frame or explicit post-write quarantine and verified-close handling settles the operation, while overdue auxiliary activity continues through normal cancellation and close handling. Retained controller and auxiliary startup cleanup and every virtual-motion owner still settle before final persistence. The final settled position is flushed before remaining serial ports close; a failed final write keeps shutdown pending for another attempt.
- `_prepare_update_parameters`, `_prepare_external_axis_parameters`, `_prepare_controller_calibration`, `_prepare_calibration_command`, and `_prepare_position_command` stage complete numeric command data before calibration application. Ordinary startup requires a loaded calibration dictionary before application and fails closed with controller motion disabled when loading fails. Save/apply and `_prepare_custom_calibration_snapshot` validate the complete merged J1-J9 current pose against staged limits and step scales, including editable J7-J9 positions, before changing `CAL`, sending controller calibration, or persisting a profile. `_prepare_custom_calibration_profile` validates all `UP`-owned and `CE`-owned values plus calibration offsets, then validates the active J1-J9 pose against the resulting limits and step scales. Missing, corrupt, or non-object JSON fails the explicit profile load without substituting legacy/default calibration or terminating the application. `sync_calibration_to_fields` updates only editable fields, leaving active `CAL`, native kinematics, runtime limit labels and sliders, and controller calibration unchanged until Save/Apply succeeds. Motor directions, calibration directions, and calibration selections are binary; calibration offsets and saved numeric vision fields must be finite. Invalid primary, external-axis, or custom-profile input leaves active state unchanged; `SaveAndApplyCalibration` does not persist failed validation. Controller-backed calibration preflights transport admission before local mutation and records whether transmission started. `UP` and `CE` updates require the firmware's exact bounded unframed `Done` acknowledgement. Rejection before the first controller write restores the prior local snapshot. An exception after transmission starts, or failure of `CE` after acknowledged `UP`, retains the intended local calibration, invalidates pending joint state, quarantines the controller transport, blocks persistence, and requires reconnection because controller state is partial or uncertain. A failed close retains the quarantined handle for later cleanup. Failure to persist after both acknowledgements retains the controller-matched local calibration and marks a debounced persistence retry. Controller startup range-validates the returned J1-J9 pose against the staged combined calibration before local mutation. Failure after staged calibration application retains controller-matched calibration, leaves the saved auxiliary port unchanged, invalidates motion state, closes the connection, and requires reconnection. Automatic calibration validates both primary and external-axis stages before the first write and range-validates fault frames before fault rendering or position-dependent local state changes.
- The owned main-controller line-exchange path has a finite write timeout and command-specific finite response deadlines for bounded commands, derived from speed, acceleration, deceleration, and ramp values after conversion to the controller's binary32 representation, configured full-travel timing, and a safety margin. Teensy `PG` playback has no file-duration contract, so the serial worker retains response ownership until a terminal line or application shutdown instead of treating a fixed threshold as lost framing. Shutdown cancellation quarantines the host transport and attempts a verified close but does not claim physical-motion preemption. Pre-write calibration activity remains interruptible; committed calibration retains ownership until an applied terminal `LL` frame or explicit post-write quarantine and verified-close handling settles the operation. Legacy main-controller exchanges reset stale input before transmission, consume a bounded framed or exact response through one shared owner, share dispatcher transport admission, and prevent serial close during active work; callers remain blocking until queue migration. Public callbacks that change controller position, calibration, or port selection acquire logical-motion admission before transport. `requestPos` and main-controller replacement are explicit position-recovery admissions. Main-controller replacement reserves the shared transport before close/open, commits `CAL['comPort']` only after fault-free startup position application, and retains ownership through failure cleanup. Auxiliary replacement or disablement stages the requested port and board until verified close/open and binding finish; failure restores the prior persisted configuration and UI selection. Controller startup captures Tk-backed command state before launch, treats input or output buffer-reset failure as fatal, rechecks cancellation after stale-input reset and before write admission, and closes the failed connection; a worker attempts the optional auxiliary connection, performs bounded `UP`, `CE`, `SP`, and `RP` main-controller exchanges regardless of auxiliary availability, consumes unframed `UP` and `CE` acknowledgements through a bounded quiet boundary, normalizes CRLF on the framed `SP` acknowledgement, then Tk applies the typed position and visual-option result with the resulting connection status. Normal and forced position senders consume the complete framed `Done` acknowledgement and quiet boundary before a forced-pose path begins a separately owned `RP` exchange. An acknowledged forced target remains the reconnect `SP` source while `RP` fails or raises, and only a later fault-free authoritative position response clears that recovery target. Unavailable, disabled, or failed auxiliary startup closes and clears any pre-existing auxiliary handle before main-controller synchronization; failed cleanup aborts the startup result, retains the handle, and retries close rather than dropping ownership. Failed main-controller cleanup retains the port, activity lease, and serial reservation in a durable non-Tk retry owner, including simultaneous close and Tk-scheduler failure. A startup timeout dismisses the modal spinner and marks the connection attempt cancelled. Failed Tk scheduling no longer joins the worker; worker-side cleanup closes startup transports before releasing ownership. Shutdown closes the joint dispatcher, drains terminal events ahead of final persistence, and waits asynchronously for virtual work, retained cleanup, owned transports, and tracked direct activity before serial close and GUI destruction.
- Successful live completion returns the motion status to ready. Successful joint completion returns to ready when no replacement target remains and retains a queued status when coalesced work remains, but only after the matching virtual-model update succeeds; failure invalidates motion admission before acknowledgement. An M1 speed-violation result retains the controller warning across serial, manual, startup, calibration, and joint settlement. Joint M1 settlement discards queued and deferred targets before result acknowledgement, rebases dispatcher state on the confirmed controller position, and prevents a stale replacement move from starting.
- Worker-run program motion acquires an exclusive request-scoped motion lease
  and reserves the matching virtual operation before physical dispatch, so
  failed virtual admission cannot start an unowned controller move. The lease
  remains active through matching controller-result application,
  virtual-operation settlement, and pose reconciliation, excluding unrelated
  manual, live, G-code, offline, and joint-dispatch motion from the logical
  row. Controller-worker construction or startup exceptions become failed
  controller admission and still settle the admitted virtual operation. Before
  ownership release, a confirmed controller response overwrites the virtual
  preview, rejection before the physical write boundary restores the saved
  confirmed pose, offline failure restores the saved virtual pose, and
  post-write uncertainty or failed convergence activates the
  controller-position resynchronization block. Advancement requires the
  matching applied controller response and virtual terminal success.
  Controller failure, missing completion, failed virtual admission, virtual
  execution failure, or virtual timeout rejects the row. A missed deadline
  records failure but retains row ownership through late controller and virtual
  settlement, preventing a terminal row state from overlapping an active
  worker. Each virtual operation publishes completion only after the matching
  worker releases the drive lock, so row settlement does not depend on a
  globally reacquirable lock. Virtual deadlines derive from the separate
  validated simulator envelope, shared timing validation, configured travel
  bounds, and simulator scaling; controller deadlines continue through the
  controller-only envelope. Reverse-step program motion returns control to Tk
  after dispatch, combines controller success with explicit virtual success,
  and advances selection only from a post-release success callback. Every
  simple `executeRow` movement form uses the same dispatcher. Online operation
  without VTK uses controller-only request ownership; offline operation without
  a virtual route rejects before physical transmission. Busy program motion
  stops without advancing selection, and busy manual Cartesian or tool-frame
  requests return before virtual dispatch. Teensy `MA` and `MC` parsing stages
  `Tr` without writing beyond the six-element Cartesian array, but arc and
  circle program rows remain disabled until deterministic trajectory simulation
  and authorized hardware validation cover the broader algorithms. Spline rows
  remain disabled until speculative acknowledgements are replaced by owned
  terminal-response handling. `Tool Set` rows also reject because the tracked
  Teensy `6.7.1-ar4hmi.3` parser has no `TF` command. Rejection leaves the
  originating program selection unchanged. `.ar4` G-code playback transfers
  the row reservation to the serial worker and returns terminal completion or
  rejection to the row state machine without changing navigation before
  terminal success. G-code start-position motion omits the physical command
  while offline, and G-code playback rejects while offline. Playback, deletion,
  and conversion share ASCII path-component validation. Controller-directory
  reads and deletion use request-scoped workers with Tk result application,
  reject during program-level ownership and reject operator requests during
  active G-code conversion. Shared lock-protected admission excludes
  Run, Step Forward, Step Reverse, and conversion while an operator `RG` or
  `DG` request owns worker execution, Tk result application, transport, and
  logical motion. Program ownership excludes storage before resource
  acquisition. Storage cleanup stops at the first failed release and retains
  remaining activity, transport, logical-motion, request, and admission
  ownership under a background retry owner; conversion settlement waits for
  that retry to complete.
  Conversion-owned completion callbacks provide scoped storage admission for
  pre-row deletion. Program start, storage, and conversion admission are
  atomic. Conversion blocks ordinary logical-motion, mode-change, and
  controller-replacement admission across the delete and row lifecycle. Native
  kinematics readiness is preflighted before controller-file deletion. The row
  worker inherits an exclusive logical-motion lease, revalidates stop,
  shutdown, conversion, and lease state at worker entry, and releases that
  lease before conversion admission clears. Final row-write admission shares
  the conversion cancellation lock with local Stop and shutdown. The
  displayed local path remains read-only, and a completed local-file load
  survives stale results through request-captured view generations. Local
  loading opens POSIX paths
  in nonblocking mode before descriptor-based regular-file validation, so
  selected special files cannot stall the HMI. Bounded binary reads enforce
  source-byte, source-row, and source-line limits before accumulation.
  Retained source bytes are tab or
  printable ASCII `0x20` through `0x7E`, and controller-length-incompatible
  rows are rejected. Blank and comment-only rows are discarded, and a source
  with no actionable row rejects before view replacement or controller-file
  deletion. Supported horizontal whitespace is canonicalized to the
  literal-space row grammar without collapsing repeated actionable rows.
  Conversion starts at row 0 when no selection exists. Full view rollback restores row colors, selection,
  active and anchor rows, scroll
  position, program path, exact program-path widget state, and current-row
  display. Operation-specific `RG` and `DG` terminal sets quarantine unmatched
  frames. Every successful listing carries
  the SD-card CID, while detailed directory errors carry `EG:` text. Every
  destructive command and controller-file write carries the matching CID, and
  startup binds the active transport to the Teensy hardware identity. Firmware
  revalidates the CID after directory and delete-lookup traversal, immediately
  before and after deletion, and before and after each `WC` or `WG` file write.
  A durable
  atomic journal records controller ID, media ID, request ID, and filename
  before each delete write. Every directory and delete worker holds an
  operating-system operation lease across the controller exchange, any
  authoritative journal reread, and durable settlement, preventing a stale
  listing from clearing an active delete across application processes.
  Process termination
  releases the lease while a persisted pending record remains available for
  explicit orphan recovery. Default journal files reside in the current
  account's operating-system state directory. Directory admission rejects
  reparse or symbolic links and foreign ownership; POSIX additionally rejects
  group or other-user write access. Lock and journal admission require a
  current-user-owned direct single-link regular entry. POSIX journal opening
  is nonblocking, so a special-file entry cannot stall state loading.
  Temporary file data is synchronized before
  write-through replacement on Windows or replacement plus containing-directory
  synchronization on POSIX. A committed delete failure or detailed controller
  error remains indeterminate across application restarts, blocks another
  delete, and can be reconciled only from a valid listing returned by the
  original Teensy and SD card. A lease-release failure after a validated
  controller response and durable journal replacement preserves the definitive
  result, marks reconciliation state unavailable in the current process, and
  forces lease reacquisition plus an authoritative journal reload before the
  next storage request. Conversion requests use the same asynchronous
  delete owner before row processing rather than bypassing storage admission
  with a direct `DG` exchange. Conversion Stop and shutdown share an atomic
  cancellation/commit boundary with that delete: cancellation before
  commitment prevents transmission and clears the journal, while cancellation
  after commitment retains terminal-response ownership and suppresses
  row-worker startup after settlement. Conversion startup rejects application
  shutdown, guards Tk staging and worker transfer, and clears shutdown
  ownership on every pre-worker failure. Shutdown retains ownership until an
  active conversion worker exits. Temporary journal-file allocation has a
  bounded collision retry and propagates access failures immediately. Test
  fixtures use a validated external temporary parent with the same bounded
  allocation behavior. Storage
  E-stop frames quarantine the main transport and trigger a verified-close
  attempt because the interrupted firmware handler can later emit another
  frame; close failure leaves the handle quarantined for explicit
  reconnection. Storage ownership then follows ordinary result cleanup; only a
  cleanup-release failure activates the retained retry owner. Only reversible
  `.txt` names become actionable. Detailed storage errors,
  including `WC` media-identity failures during conversion, avoid the
  motion-error renderer, preserve the printable controller detail, and halt
  conversion scheduling.
  Firmware SD initialization binds the mounted volume to the formatted CID,
  probes current media before reuse, and remounts after an identity change.
  Mutation requires the requested CID to match the mount binding and current
  card; directory output carries the mount binding only after a final
  current-card check.
  Firmware directory traversal distinguishes clean
  end-of-directory from read failure, delete lookup distinguishes confirmed
  absence from lookup failure, and incompatible entries, unavailable directory
  buffering, read failure, or aggregate payload beyond 4096 bytes produce an
  error before a partial listing. G-code feed values convert from active units
  per minute to millimetres per second before controller serialization.
  Conversion retries pending admission, stops on rejection or local
  cancellation, clears cross-feature admission on every worker exit, and
  advances navigation only after explicit row completion. Manual forward
  stepping uses the same completion-only rule. Missing Teensy SD files emit
  `EG`, rotate the command buffer, and return to command processing instead of
  halting the firmware loop. G-code stop halts local row scheduling before
  serial admission and reports that active controller motion is not preempted;
  row writes share the control write lock and recheck local stop state under
  that lock. The host does not emit `SS\n` because no reachable host `SL`
  producer establishes firmware spline mode.
- Program-motion completion preserves the original virtual deadline across Tk-scheduler fallback. A missed deadline remains a failed result, but row ownership continues until the matching request-scoped virtual operation settles. If fallback-thread construction fails, the same settlement completes synchronously instead of releasing the row early.
- VTK renders at motion cadence only while virtual joint angles change, then returns to a low-rate idle check. Topmost configuration uses bounded discovery instead of a permanent polling thread.
- Verified shadowed `set_vtk_topmost_delayed` and `create_tool_control` definitions were consolidated.

Automated coverage is hardware-free and never imports `AR4.py`.

## Confirmed remaining blocking and ownership defects

### Main-controller transport

Legacy main-controller callers now use a shared bounded exchange owner instead of resetting input after a write or reading the active handle directly. Those synchronous callers still bypass the result queues and can block the invoking thread. Full HMI correctness requires a central transport service for Teensy commands, with command-specific response schemas and explicit cancellation semantics.

### Program execution

`executeRow` combines program parsing, Tk selection state, command construction, serial I/O, virtual playback, and timing delays. Simple movement forms now share request-scoped dispatch and calibrated raw-target validation, but non-motion branches still use legacy blocking transactions and program workers still touch Tk widgets. Arc, circle, and spline transmission remains deliberately unavailable until paired firmware response contracts are safe. Full correction requires a tested program state machine:

`GCconvertProg` and `GCexecuteRow` remain inside this known worker-thread Tk
exception. Conversion selection, status, and row-color changes require the
typed program state-machine migration before worker-thread widget access can
be removed.

- Parse and validate a program row without Tk or serial dependencies.
- Produce a typed action.
- Dispatch controller work through the central transport owner.
- Apply the result and advance selection on the Tk thread.
- Represent stop, pause, error, and completion as explicit states.

Threading the current function without separating those responsibilities would preserve response races and introduce more cross-thread Tk access.

### Live jog and stop

The implemented live-jog control lane matches the current firmware exception: `LJ`, `LC`, and `LT` poll serial input between short moves and terminate after a newline-delimited token. The Tk callback only sets a stop event; the existing response owner writes the token and reads the resulting position.

Discrete `RJ` motion remains different. The firmware does not run normal serial parsing inside `driveMotorsJ`, so `S\n` cannot replace or interrupt an active discrete target. Pending desktop targets can be discarded, but physical interruption of an in-flight discrete move still depends on the independent emergency-stop path. True cancel or setpoint replacement requires a matching firmware/controller protocol redesign and hardware validation.

### Cartesian and tool-frame incremental jog

Cartesian adjustments still use raw commands that can be rejected while serial work is active. Tool-frame commands represent relative motion in a changing frame. Safe coalescing therefore cannot concatenate raw strings or mix coordinate spaces. A shared intent arbiter must retain the latest target per active coordinate mode and resolve inverse kinematics from the latest confirmed robot state at dispatch time.

### Startup lifecycle

`setCom` now separates Tk-backed request capture and result application from worker-owned controller synchronization. Connection changes reserve the main transport, startup reads are bounded, timeout is an explicit cooperative-cancellation state, and failed scheduling uses non-Tk cleanup. Main serial opening, calibration-field capture, native-kinematics refresh, and some connection logging still occur in the Tk callback. Explicit application lifecycle states and import-safe GUI construction remain required under `PLAN.md` M2B.

### Remaining performance surfaces

- Large optional packages, including VTK, OpenCV, and Matplotlib, load during module startup even when corresponding tabs are unused.
- Camera, program, Modbus, and auxiliary-board paths retain synchronous work. Legacy main-controller exchanges still wait synchronously for controller completion where invoked from Tk callbacks.
- Calibration result application still snapshots and serializes the complete UI log on Tk; persistence remains a measured M4A2 optimization surface.
- Full calibration serialization still reads a broad Tk-backed state dictionary; debouncing reduces frequency but does not reduce payload or provide atomic replacement.
- `moveInProc` contains comparison expressions used where assignments appear intended, but the surrounding state is not consumed consistently enough for a safe isolated edit. The program-state-machine pass must replace that implicit state rather than patch isolated operators.

## HMI completion criteria

- No Tk callback performs a blocking serial read, fixed sleep, motion wait, or long computation.
- No worker thread reads or mutates Tk widgets.
- A central service owns main-controller requests and responses.
- Joint, Cartesian, and tool-frame inputs retain semantic final targets rather than raw event history.
- Stop and fault handling preempt supported motion modes and leave controller state explicit.
- Malformed, late, mismatched, and unsolicited serial data have tested behavior.
- Program execution has deterministic state-transition tests.
- Event-loop delay, command turnaround, queue replacement, render cadence, and configuration-write duration have measured baselines before further tuning.

## Verification boundary

Static compilation and deterministic fake-transport tests establish software behavior only. Live responsiveness, controller timing, motor motion, stop behavior, accuracy, and repeatability remain unverified pending an authorized hardware procedure.
