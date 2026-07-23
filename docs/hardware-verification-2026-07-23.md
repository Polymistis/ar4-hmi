# Powered-Arm Commissioning and Motion Verification — 2026-07-23

## Verification boundary

This record begins with one operator-authorized J1 calibration-scale check and
continues through the resulting driver-setting correction, J1-J6 calibration
and jog checks, a defined pose capture, and a controlled multi-axis HMI
responsiveness check, followed by an approximate absolute-slider return
attempt. No auxiliary output, gripper, or program motion was operator-requested.
Opening the Nano connection during HMI startup can reset the board, attach
`servo0` through `servo6`, and repeat `servo0.write(20)` as a firmware startup
side effect. Physical angles were observed qualitatively rather than measured
with an instrument.

## Authorization, safety state, and deviations

- The operator confirmed a clear work envelope before controller
  commissioning. A separate envelope confirmation immediately before the
  powered checks was not recorded.
- A hardware emergency stop was available and was engaged while drive power
  was off. No controller E-stop frame, HMI alarm, or powered interruption
  independent of desktop GUI state was recorded before commanded motion.
- The powered observations below therefore remain exploratory evidence and do
  not satisfy the complete M5 pre-motion acceptance boundary. Further powered
  M5 verification remains gated on explicit work-envelope confirmation and
  powered verification of the independent hardware stop path.
- After the first J1 calibration exposed an untrusted scale and reference, the
  subsequent approximate `+79`-degree position and nominal `1`-degree jog were
  diagnostic commands outside the fail-closed calibration invariant. Further
  motion stopped after that jog and resumed only after the driver configuration
  was corrected and J1 was recalibrated.

## Controller and configuration identity

- Main controller: Teensy 4.1 on `COM13`, running tracked firmware
  `6.7.1-ar4hmi.1`.
- Auxiliary controller: Arduino Nano on `COM14`; no auxiliary command formed
  part of this check.
- Active primary-axis profile:

  | Joint | Negative limit (degrees) | Positive limit (degrees) | Steps/degree | Drive microsteps |
  | --- | ---: | ---: | ---: | ---: |
  | J1 | `170` | `170` | `88.888` | `800` |
  | J2 | `42` | `90` | `111.111` | `800` |
  | J3 | `89` | `52` | `111.111` | `800` |
  | J4 | `180` | `180` | `99.555` | `800` |
  | J5 | `105` | `105` | `43.72` | `1600` |
  | J6 | `180` | `180` | `44.444` | `800` |

- Active J1 encoder configuration: `4000` counts per revolution.
- Starting pose: not measured. J1 did not have a previously established
  absolute reference.

## Procedure and observations

1. Ran the HMI's J1-only calibration path.
2. J1 reached the calibration reference, then rotated almost one full revolution
   during the commanded return to center instead of moving approximately
   `170` degrees.
3. Moved J1 until the HMI reported approximately `+79` degrees. The resulting
   physical orientation appeared approximately straight forward.
4. Requested a nominal `1`-degree J1 jog. The observed physical motion was
   clearly greater than `1` degree.
5. Stopped further J1 motion pending diagnosis.

## Diagnosis

The active J1 software setting uses `800` drive microsteps and `88.888` steps
per degree. The older J1 setting uses `400` microsteps and `44.4444` steps per
degree. The observed travel is consistent with a physical J1 driver still
configured for `400` microsteps:

- The center-return calculation emits approximately `170 * 88.888 = 15111`
  pulses. A `400`-microstep driver converts that pulse count to approximately
  `340` degrees of J1 travel instead of `170` degrees.
- A nominal `+79`-degree command then produces approximately `158` degrees of
  physical travel, placing a joint displaced near `-170` degrees back near the
  forward orientation.

## Initial J1 result

J1 calibration-scale verification failed. Driver model and physical DIP-switch
settings must be confirmed against the selected HMI profile before calibration
or further motion testing resumes. J2-J6 scale and calibration behavior remain
unverified.

## Driver-setting correction and HMI restart

The operator subsequently opened the current Annin Robotics manual and
confirmed completion of the updated microstep DIP-switch settings on all six
motor drivers. The resulting settings matched the per-joint table above,
including J5 at `1600` and the other primary joints at `800`. The physical
switch positions were not independently observed through the HMI session.

The HMI was relaunched from the isolated CPython 3.12 environment as process
`68168`, and the process remained responsive. No calibration or motion command
formed part of the relaunch. The failed J1 reference remains invalid until a
new authorized J1 calibration completes successfully. Opening `COM14` can reset
the Nano and repeat the firmware's servo attachments and baseline gripper
write; that reset-driven write was not independently observed.

## Post-correction J1 result

After the driver-setting correction and HMI restart, the operator ran J1-only
calibration and then exercised the J1 jog feature. Both calibration and jog
behavior were reported as expected with the active `88.888` steps-per-degree
and `800`-microstep configuration. The exact post-correction jog increment was
not recorded. No instrumented angular, timing, or repeatability measurement was
supplied, so this result does not establish quantified accuracy. J2-J6 remain
unverified.

## Post-correction J2 result

The operator subsequently ran J2-only calibration and then exercised the J2
jog feature. Both calibration and jog behavior were reported as expected with
the active `111.111` steps-per-degree and `800`-microstep configuration. The
exact jog increment was not recorded, and no instrumented angular, timing, or
repeatability measurement was supplied. J3-J6 remain unverified.

## Post-correction J3-J6 results

For each remaining primary joint, the operator ran the HMI's individual
calibration path and then exercised the jog feature to check direction,
calibration travel, and apparent scale. Exact jog increments and physical
angles were not recorded. The operator reported the following results:

| Joint | Active steps/degree | Active microsteps | Operator observation |
| --- | ---: | ---: | --- |
| J3 | `111.111` | `800` | Calibration and jog behavior worked as expected. |
| J4 | `99.555` | `800` | Calibration and jog behavior worked as expected. |
| J5 | `43.72` | `1600` | Calibration and jog behavior worked as expected. |
| J6 | `44.444` | `800` | Calibration and jog behavior worked as expected. |

No instrumented angle, timing, or repeatability measurement was supplied for
these checks. A runtime position snapshot captured afterward reflected ongoing
jog testing, not a defined reference pose, and was therefore rejected as
baseline evidence. A new joint snapshot remained pending until the arm reached
a defined pose.

HMI process `68168` remained responsive after the checks. These observations
verify expected single-joint direction, calibration travel, and apparent jog
scale after the driver update. Simultaneous multi-axis motion, deferred HMI
input, instrumented accuracy, repeatability, speed, and fault response remain
unverified.

## Defined static-pose snapshot

The operator confirmed that the arm had reached the proper static pose.
Two reads of the persisted configuration, captured approximately `1.2` seconds
apart, returned the same joint values and the same configuration write time:

| Joint | Position (degrees) |
| --- | ---: |
| J1 | `+0.011` |
| J2 | `0.000` |
| J3 | `+0.009` |
| J4 | `+0.010` |
| J5 | `+44.991` |
| J6 | `+0.023` |

The second read was captured at `2026-07-23T04:58:29.632-04:00`; the persisted
configuration write time was `2026-07-23T04:57:35.016-04:00`. This
operator-confirmed pose is the baseline for the next controlled motion check.

## Authorized coordinated-motion and queue procedure

The operator authorized the next controlled motion check from the defined
static pose. The HMI motion profile for this procedure is:

- Speed mode: `Seconds`
- Speed value: `5`
- Acceleration region: `15%`
- Deceleration region: `15%`
- Ramp: `80%`

Procedure:

1. Release the J1 absolute slider at `+10` degrees to begin the initial
   five-second J1 move.
2. Before the initial move finishes, release the J2 slider at `+5` degrees,
   the J3 slider at `-5` degrees, and finally the J1 slider at `+5` degrees.
3. Confirm continued HMI interaction and a queued-target status while the first
   controller command remains active.
4. After the first move completes, observe whether J1, J2, and J3 move
   simultaneously toward the consolidated final target
   `(J1=+5, J2=+5, J3=-5)` while J4-J6 retain the baseline targets.
5. Wait for `SYSTEM READY`, then capture the persisted joint pose and the
   operator observations before any return move.

Unexpected direction, contact, abnormal noise, stall, scale error, controller
fault, or HMI lockup ends the procedure without another target. The displayed
state and observed phase are captured before recovery. After a successful
result is recorded, a separate controlled move can return the arm to the
defined static pose.

## Coordinated-motion and queue observations

The executed input sequence deviated from the scripted absolute-slider targets.
The operator could not complete all specified releases inside the available
window and instead entered rapid incremental jog requests across J1, J2, and
J3. The operator observed simultaneous joint motion while continuing to enter
HMI requests. No HMI lockup was reported, and process `68168` remained
responsive after motion settled.

Two post-motion reads returned the same persisted pose:

| Joint | Position (degrees) |
| --- | ---: |
| J1 | `-9.900` |
| J2 | `-1.629` |
| J3 | `-12.339` |
| J4 | `+0.010` |
| J5 | `+44.991` |
| J6 | `+0.023` |

The second read was captured at `2026-07-23T05:04:55.017-04:00`; the
configuration write time remained
`2026-07-23T05:04:12.599-04:00`. The observation verifies non-blocking HMI
interaction and physical multi-axis execution under rapid joint input. The
unrecorded input order and timing do not establish the exact number of
controller commands or prove that every input collapsed into one final
command.

## Post-test return-control observation

The operator attempted to restore the defined static pose with the J1-J6
absolute sliders. Exact numeric slider placement was impractical, and the
displayed J1-J6 position fields provided no action for submitting typed target
values. No exact post-return baseline was accepted. Exact keyboard target entry
was recorded as a separate HMI usability requirement; no additional motion
result is inferred from this observation. The session ended with the arm
operator-reported as approximately near the defined static pose; exact final
joint coordinates and physical pose were not captured.

## Overall result

The driver-scale mismatch was diagnosed and corrected. Operator-observed
individual calibration and jog behavior then passed for J1-J6, and rapid
J1-J3 input demonstrated continued HMI interaction with physical multi-axis
execution. Exact command coalescing, instrumented accuracy, repeatability,
speed, and fault response remain unverified. The recorded pre-motion deviations
prevent these observations from completing M5 controlled-hardware acceptance.
