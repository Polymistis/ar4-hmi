# Dynamic Motion and Interception Research

## Required architecture

Desktop target coalescing improves operator interaction between discrete commands. Object following and interception require a separate feedback-control architecture:

```text
timestamped observations
        |
camera calibration and frame transforms
        |
state estimator: position, velocity, acceleration, covariance
        |
future-state predictor
        |
reachable intercept candidates and grasp poses
        |
IK, collision, joint-limit, timing, and uncertainty filters
        |
global path plus fast local planner / online trajectory generator
        |
high-rate controller with measured joint-state feedback
        |
grasp supervisor, hold/abort logic, and independent emergency stop
```

Pipeline stages must reject stale timestamps, invalid transforms, non-finite values, unreachable poses, and uncertainty beyond an application-defined grasp threshold.

## Current controller limitation

The Teensy `driveMotorsJ` implementation schedules configured-axis pulse work within a coordinated motion loop. Desktop serialization therefore does not imply sequential motor stepping.

Normal command parsing does not run concurrently with that motion loop, and `sendRobotPos` reports the resulting state after the move. The current request/response protocol can support the HMI latest-target queue, but cannot support continuous target replacement or closed-loop visual servoing during a move. Dynamic interception requires a controller redesign with:

- a fixed-period, non-blocking motion update;
- replaceable position or velocity setpoints;
- timestamped joint position and velocity feedback;
- deterministic hold, cancel, watchdog, and fault states;
- bounded command age and sequence checking;
- measured end-to-end latency and control-loop jitter.

A ROS 2 migration is one option, not a current decision. The existing firmware can also be redesigned around equivalent control contracts.

## Planning and control layers

### Repeatable point-to-point motion

Record per-joint position, velocity, acceleration, jerk, and direction-dependent limits. Generate synchronized multi-axis trajectories rather than relying only on a percent-speed scalar. Ruckig provides online multi-DoF trajectory generation with velocity, acceleration, jerk, and synchronization constraints; changed state or target input causes recalculation. The original algorithm is described in [Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](https://arxiv.org/abs/2105.04830), and input validation and synchronization options are documented in the [Ruckig tutorial](https://docs.ruckig.com/tutorial.html).

For planned paths, MoveIt separates geometric planning from time parameterization and supports jerk-limited Ruckig smoothing after time parameterization. The documentation also warns that time-optimal path resampling can deviate from original waypoints and can require additional collision checking. See [MoveIt time parameterization](https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html).

### Reactive local control

MoveIt Servo accepts joint velocity, end-effector velocity, or desired end-effector pose commands and includes optional collision, singularity, limit, and smoothing facilities. That contract matches visual servoing more closely than discrete line commands. See [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html).

MoveIt Hybrid Planning separates a slower global planner from a continuously running local planner that reacts to sensor feedback and can splice trajectory updates, avoid local collisions, or replan. See [MoveIt Hybrid Planning](https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html).

The ROS 2 joint trajectory controller demonstrates a standard execution contract with timestamped waypoints, position feedback, tolerances, trajectory replacement, execution monitoring, and real-time-safe implementation. See the [ros2_control joint trajectory controller](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).

## Tracking and interception

### Baseline estimator

Start with a calibrated, timestamped constant-velocity or constant-acceleration model and explicit covariance. OpenCV supplies standard Kalman `predict` and `correct` operations through [`cv::KalmanFilter`](https://docs.opencv.org/4.x/dd/d6a/classcv_1_1KalmanFilter.html). Rolling friction, table slope, impacts, occlusion, and spin invalidate a single fixed model, so innovation tests and model reset or switching are required.

### Intercept selection

For each future time in a bounded horizon:

1. Predict object state and uncertainty.
2. Generate candidate grasp poses and approach directions.
3. Reject table, fixture, self-collision, singularity, and workspace violations.
4. Solve IK and estimate minimum feasible arrival time under measured joint limits.
5. Require arrival margin, low enough prediction uncertainty, and a valid terminal grasp velocity.
6. Select the lowest-risk feasible candidate, then repeat the calculation after each new observation.

After an impact or unexplained innovation, invalidate the old intercept and return to tracking. A grasp attempt begins only after pose, relative velocity, uncertainty, and gripper timing satisfy explicit thresholds.

Robotic interception literature supports the complete perception-prediction-planning-execution split. A 2024 projectile-interception system combined detection, future-motion prediction, minimum-time planning, obstacle avoidance, and execution under tight timing constraints; see [Preprocessing-based Kinodynamic Motion Planning Framework for Intercepting Projectiles](https://arxiv.org/abs/2401.08022). Work on uneven-object catching likewise combines prediction, intercept calculation, motion planning, and velocity control; see [Neural Motion Prediction for In-flight Uneven Object Catching](https://arxiv.org/abs/2103.08368).

Published success rates from different robots, sensors, end effectors, and objects are not performance estimates for this project.

## Repeatability investigation

Hardware experiments should distinguish accuracy from repeatability and record:

- controller and firmware identity;
- calibration profile and starting pose;
- payload, end effector, cable routing, temperature, and warm-up state;
- commanded trajectory and approach direction;
- measured joint and tool pose from an independent reference;
- backlash, missed-step, encoder, and settling behavior;
- latency from observation through command acceptance and physical response.

Repeat targets from consistent and reversed approach directions to expose hysteresis and backlash. Report distributions and worst observed errors rather than a single successful cycle. Speed, acceleration, jerk, payload, and path shape should be varied only under an approved hardware-validation procedure.

## Staged dynamic-motion work

1. Define measurable repeatability, latency, speed, workspace, payload, and grasp requirements.
2. Instrument timestamped controller state and end-to-end command latency.
3. Build a deterministic simulator and recorded-observation replay path.
4. Compare trajectory-generation options against measured joint constraints.
5. Implement estimator and predictor with covariance and stale-data rejection.
6. Implement reachability and intercept selection in simulation.
7. Add local replanning, collision checking, and terminal visual servoing.
8. Authorize conservative hardware validation only after stop, watchdog, limit, and fault behavior pass software and bench checks.

## Implemented controller-trace analysis foundation

`ARrobots.controller_trace` defines canonical JSONL schema
`ar4.controller-trace.v2` for one complete J1-J6
`JOINT_TELEMETRY_V1` exchange. The header preserves the controller hardware ID,
firmware version, confirmed command-start position, commanded J1-J6 target, RJ
speed and ramp inputs, the nominal telemetry period, and a `sha256:`
configuration fingerprint. The fingerprint
identifies the canonical ASCII `UP` controller-configuration command including
the terminal LF; configuration contents remain outside the trace. The schema
uses the complete existing `CommandTiming.mode` domain: `p`, `s`, and `m`.
The trace header defines the time origin as the host monotonic timestamp
captured immediately before the RJ write. Encoder-sample and terminal
timestamps are host-receipt offsets from that origin. Command direction is
derived from the confirmed start and target positions rather than a
potentially late first encoder sample.

Encoder records contain host-receipt timestamps and J1-J6 positions in degrees.
The terminal record separately identifies success, failure, or stop and stores
controller-reported step-counter positions when available. Terminal positions
are not reclassified as encoder measurements. The decoder bounds payload,
record, and line size; requires strict UTF-8 and exact fields; rejects duplicate
JSON keys and non-finite numbers; and requires strictly increasing sample
timestamps followed by one terminal record.

The hardware-free analyzer derives velocity, acceleration, and jerk through
successive finite differences at the midpoint of each observed interval, so
unequal host-receipt intervals are not treated as uniform samples. Per-joint
results include endpoint error, terminal reported error, direction-aware peak
speed, reverse speed, acceleration, deceleration, overshoot, and absolute jerk.
Cadence statistics identify internal cadence gaps and missing coverage near
command transmission or terminal response. A failed exchange, inadequate
sample count, cadence or boundary gap, or stationary J1-J6 trace makes the
record explicitly ineligible for profile analysis.

The current schema and analyzer create no trace automatically, send no
controller command, and establish no measured motion limit. The firmware emits
best-effort telemetry at a nominal 10 Hz without controller timestamps. Host
receipt jitter and millidegree encoder quantization therefore remain explicit
measurement limitations, and any future tuning decision requires repeated
recorded trials under an approved hardware procedure.

## Implemented deterministic foundation

`ARrobots.dynamic_motion` provides hardware-free constant-velocity and
constant-acceleration object-state estimators behind the same observation and
replay boundary. Each observation carries an explicit coordinate-frame name,
monotonic source timestamp, and diagonal position variance. Measurement errors
are assumed independent across observations and coordinate axes; later camera
calibration and filtered-estimator work must replace that assumption when
correlated measurement uncertainty becomes available.

Estimator configuration bounds observation age, future clock skew, minimum
sample interval, and maximum sample interval. Rejected observations leave the
active baseline and estimate unchanged. A gap beyond the configured maximum
accepts the new observation only as a replacement baseline, making model reset
visible to the caller. The two-observation estimator publishes constant
velocity. The three-observation estimator reports an explicit warmup step after
the second sample, then calculates acceleration and terminal velocity at the
newest timestamp across equal or unequal sample intervals.

The constant-velocity predictor uses a bounded future timestamp, preserves
velocity, propagates position/velocity covariance, and adds configured diagonal
position-process variance per second. The acceleration-aware estimate carries
per-axis position, velocity, and acceleration variance plus all corresponding
cross-covariances. Its predictor propagates that full state covariance and adds
independent continuous white-jerk process noise supplied by the caller. The
process-noise input is acceleration-variance growth in `mm^2/s^5`.
Intercept selection depends on the structural `MotionPredictor` contract rather
than either concrete predictor class. The selector validates the advertised
maximum horizon during construction and before every candidate prediction, so
a mutable predictor cannot silently void the coverage check. Every result's
type, source timestamp, coordinate frame, and requested timestamp within the
shared composed-timestamp tolerance are also validated. Predictor and selector
use that same band for numerically earlier requests and returned timestamps.
Advertised horizon magnitude cannot widen the band, and a request outside the
band fails instead of clamping to the source timestamp.
An accelerated estimate must produce an acceleration-aware predicted state;
the selector rejects a velocity-only result before feasibility evaluation, and
the constant-velocity predictor rejects an accelerated estimate directly.

Recorded input uses canonical JSONL schema `ar4.observation-replay.v1`. The
first record is the exact header:

```json
{"position_unit":"millimeter","schema":"ar4.observation-replay.v1","timebase":"monotonic-seconds"}
```

Each following record contains exactly `timestamp_seconds`,
`received_at_seconds`, `frame_id`, `position`, and `position_variance`.
Positions and variances contain three numeric components in X/Y/Z order. The
position unit is millimeters, velocity is millimeters per second, and position
variance is square millimeters. The codec rejects invalid UTF-8, NUL bytes,
blank or malformed lines, duplicate keys, non-finite values, invalid
dimensions, mixed frames, unordered source or receipt timestamps, and
configured size-limit violations. Replay owns a fresh estimator instance and
therefore cannot partially mutate an external estimator when a recorded
observation is rejected. Canonical encoding emits LF line endings. Decoding
accepts LF or CRLF, and canonical re-encoding normalizes either form to LF;
byte-level hashes therefore identify encoded files rather than logical replay
equivalence.

Current object-model scope stops before calibrated transforms, a physical
impact model, jerk estimation, production reachability, IK, collision checking,
controller setpoint replacement, or grasp supervision. The three-sample
acceleration estimate and innovation gate are deterministic models, not claims
of camera or live-object tracking accuracy. No live-arm result is established
by the deterministic module or associated tests.

## Implemented impact-aware innovation filtering

`ImpactAwareAccelerationEstimator` predicts the last accepted acceleration
estimate to each new observation timestamp, combines predicted and measurement
position variance per axis, and compares each residual with a caller-supplied
maximum standardized innovation. The estimator's caller-supplied maximum sample
interval bounds extrapolation from the accepted model while consecutive
innovations are pending. A larger accumulated model gap produces an ordinary
baseline reset before impact classification. No project default is embedded.
Normalized innovation statistics depend on correctly tuned process and
measurement noise; chi-square behavior alone does not establish correct tuning,
as discussed in
[Kalman Filter Auto-tuning through Enforcing Chi-Squared Normalized Error
Distributions](https://arxiv.org/abs/2306.07225). Innovation-based protection
also prevents a single outlier from directly corrupting the tracked state,
consistent with the bounded-innovation motivation in
[Robust Extended Kalman Filtering for Systems with Measurement
Outliers](https://arxiv.org/abs/1904.00335).

A nonzero residual with zero combined variance is treated as exceeded because
no finite standardized residual exists. Zero measurement and process variance
therefore require an exact model match; realistic covariance is required to
avoid classifying floating-point-level model error as a discontinuity.

One exceeded observation reports `INNOVATION_REJECTED`, leaves the accepted
finite-difference window unchanged, and makes the estimator's public estimate
unavailable. At least two consecutive exceeded observations are required by
configuration. Reaching that count reports `IMPACT_RESET` and transactionally
replaces the model with the confirming observation as a new baseline. The
label identifies a confirmed model discontinuity under the configured gate;
no physical collision classification is claimed. A long model gap uses the
existing baseline-reset disposition instead of claiming impact. A
model-consistent observation after an isolated rejection resumes from the
unchanged model.

Baseline replacement removes the predictive model needed by the innovation
gate. Reacquisition observations are admitted without an innovation comparison
until the acceleration estimator can publish a complete model. Feedback
replanning keeps the intercept invalid until selection and trajectory
replacement succeed. The ungated observation that completes the model can
therefore become part of the replacement model and immediately drive a valid
intercept and replacement trajectory; callers must account for that limitation
when configuring capture quality and covariance.

Impact-aware replay exposes every rejection, reset, warmup, and reacquisition.
`select_impact_aware_acceleration_replay_intercepts` performs no intercept
selection while an innovation sequence is pending or the estimator is warming
after reset. The optional feedback-replanning integration emits distinct
innovation and impact holds, keeps the last desired trajectory only as a
fallback, and marks the active intercept invalid until a new replacement is
constructed. Logical invalidation sends no controller command and does not
claim physical hold, cancellation, or changed arm motion.

## Implemented deterministic intercept selection

`ARrobots.interception` extends the hardware-free foundation through bounded
intercept-time selection. Configuration defines a coordinate frame, estimate
age and future-skew limits, minimum and maximum lead times, a sampling
interval, maximum predicted position standard deviation, maximum terminal
object speed, and minimum arrival margin. Configuration rejects a lead-time
interval that cannot produce strictly advancing samples. Selector construction
and per-candidate validation fail when the predictor cannot cover the permitted
estimate age plus lead time, and selection fails when absolute timestamps
cannot represent strictly advancing candidates.

Each predicted state first passes the uncertainty and terminal-speed limits.
A deterministic injected evaluator then classifies production-feasibility
domains as feasible, unreachable, joint-limited, singular, colliding, or
missing an arrival-time estimate. A feasible result supplies the minimum robot
arrival duration measured from the selector evaluation timestamp and an
application-defined non-negative risk score. The selector compares the largest
per-axis predicted position standard deviation with the uncertainty limit,
enforces the arrival margin, and ranks accepted candidates by risk, predicted
timestamp. The evaluator receives the predicted state and selection timestamp.
Stale and future estimates return explicit non-selected statuses without
invoking the evaluator. Exceptions or malformed evaluator output fail the
selection boundary.

`select_replay_intercepts`, `select_acceleration_replay_intercepts`, and
`select_impact_aware_acceleration_replay_intercepts` validate the complete
recorded estimator sequence and the bounded record-by-candidate workload before
feasibility evaluation, then recompute selection after every applicable
estimate update. Impact-aware acceleration replay retains baseline, warmup,
innovation, and impact-reset steps without invoking feasibility. That behavior
exercises deterministic target reselection after new observations; no geometric
path, controller setpoint, or grasp is generated. The injected evaluator
provides simulation decisions only and does not establish physical
reachability, collision clearance, singularity margin, joint-limit compliance,
or arrival performance.

## Implemented rest-to-rest trajectory timing

`ARrobots.trajectory_timing` provides a hardware-free joint-space timing and
sampling foundation for the feasibility boundary. Every joint receives an
explicit maximum velocity in degrees per second, maximum acceleration in
degrees per second squared, and maximum jerk in degrees per second cubed. No
machine defaults or measured AR4 limits are embedded.

A minimum single-axis move starts and ends at zero velocity and zero
acceleration. The analytical symmetric S-curve contains jerk-up, constant
acceleration, jerk-down, cruise, mirrored deceleration, and terminal jerk-up
phases. Short distances use a triangular jerk profile. Longer distances reach
the acceleration limit, the velocity limit, or both before adding cruise.
Public profile construction revalidates represented displacement and every
kinematic limit.

Multi-axis planning first calculates each minimum profile and selects the
longest duration. Shorter profiles receive uniform time scaling: phase times
grow by the scale, velocity falls by the scale, acceleration falls by the
scale squared, and jerk falls by the scale cubed. Stationary axes retain a
constant position for the common duration. The resulting synchronized duration
is the minimum arrival time under the supplied independent joint limits and
the rest-to-rest assumption.

This duration excludes perception age, transport latency, controller response,
IK, joint-position limits, singularity margin, collision clearance, path
geometry, settling, and gripper timing. Asymmetric directional limits,
continuous controller setpoint replacement, and controller-specific curve
tuning remain future work. The profiles therefore support deterministic
simulation and feasibility plumbing; no physical timing or safe path is
established.

## Implemented arbitrary-state replacement segments

`ARrobots.trajectory_timing` also defines finite joint boundary states with
position, velocity, and acceleration. A fixed-duration quintic segment matches
those values at both endpoints, following the acceleration-continuous
interpolation contract documented by the
[ros2_control joint trajectory controller](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).
Velocity, acceleration, and jerk limits are checked at endpoints and at
interior critical points isolated from derivative-polynomial roots with bounded
bisection. Validation does not depend on a sampling interval, and an infeasible
requested duration is rejected instead of silently changed.

Multi-axis construction uses one explicit duration for up to J1-J9. The local
replacement helper samples an existing built-in desired trajectory at the
replacement instant and uses the sampled position, velocity, and acceleration
as the next segment's start state. Position, velocity, and acceleration are
therefore continuous at that desired-state boundary; jerk may change
discontinuously. Direct construction also accepts an independently measured
start state, but no encoder-feedback adapter is implemented.

The segment is not time-optimal and carries no collision, singularity,
joint-position-limit, absolute timestamp, command-age, sequence, or controller
watchdog contract. No running firmware path accepts these segments or replaces
a setpoint during motion. Deterministic construction and sampling establish a
hardware-free replanning primitive only, not safe or achievable arm motion.

## Implemented feedback-replanning coordinator

`ARrobots.feedback_replanning` connects a constant-acceleration estimator,
caller-supplied intercept selector, correlated joint-target resolution, and
desired-trajectory replacement in a single-owner hardware-free coordinator.
An optional impact-aware estimator configuration adds innovation-rejected and
impact-reset holds. Baseline and warmup updates hold the current desired
trajectory. A long-gap estimator reset has a distinct hold disposition.
No-feasible, stale-estimate, and future-estimate selections produce distinct
hold events without calling the joint-target resolver.

`FeedbackReplanner` passes a selected-only deep snapshot to one synchronous
injected resolver. Resolver output carries the estimate, evaluation, and
intercept timestamps plus one to nine joint boundary states. All timestamps
must match the selected candidate, and the axis count must match a validated
snapshot of the active desired trajectory. The replacement duration comes from
the selected intercept time.
Completed active trajectories hold the terminal desired state before the next
segment starts. A successful replacement advances a monotonic trajectory
generation only after construction succeeds.

`AsynchronousFeedbackReplanner` exposes the same estimator, selector, and
desired-trajectory boundary as a split-phase interface. A selected observation
returns an isolated resolver request containing a monotonic request sequence,
the active trajectory generation, the required axis count, and only the
selected candidate. External code can schedule that request without blocking
observation admission, then submit either a correlated joint target or an
exception through the owning coordinator.

Every accepted observation supersedes earlier pending resolution work. A late
result is reported as superseded before supplied target or error data is
inspected; no trajectory state changes, and any current logical intercept
validity is preserved. A current result at or within the shared timestamp
comparison tolerance of the selected intercept reports
`EXPIRED_TARGET_RESOLUTION` and is discarded. A pre-deadline completion reports
`EXPIRED_TRAJECTORY_WINDOW` without fault when the same target produced a
limit-compliant trajectory at request issuance but no longer does so at result
receipt; a target that was already infeasible at issuance remains a
trajectory-construction fault. Only the current request can replace the desired
trajectory, and the replacement begins from desired state sampled when the
owning coordinator receives the result rather than at the older observation or
worker-completion time. Result receipt timestamps must preserve coordinator
event ordering.
Invalid current output, active-state corruption, or a current resolver failure
remains phase-tagged and latched. Public events and requests are isolated from
the coordinator's internal pending snapshot.

Accepted events receive contiguous sequence numbers. Out-of-order observations
are rejected by the owned estimator before target resolution. Reentrant
processing, stale resolver output, invalid active state, callback failure, and
infeasible trajectory construction cannot replace the last valid trajectory.
Estimator-processing, selection, target-resolution, and
trajectory-construction faults are bounded, phase-tagged, and latched. Logical
cancellation is also terminal. Neither a hold nor cancellation sends a
controller command or claims physical motion has stopped. Estimator holds,
selection holds, pending resolution, expiration dispositions, cancellation,
and faults invalidate the logical active intercept. A superseded result
preserves the current validity state, and a successful replacement restores
validity.

The replay adapter processes the versioned observation replay through the same
coordinator and exposes every processed event. A terminal fault returns an
explicit processed prefix. `processed_all_samples` reports record consumption;
`complete` additionally requires no terminal fault. Neither property claims
physical execution. Replay results require non-decreasing event timestamps,
permit a fault only as the terminal event, and reject logical cancellation
because the replay adapter has no cancellation input. Frame agreement and
candidate-evaluation work are checked before coordinator construction;
workload accounting uses the maximum estimate-bearing update count after the
acceleration estimator's two-record warmup under the same cap as intercept
replay. Replay target resolution remains synchronous injected simulation
logic. The asynchronous coordinator creates no thread, task, executor,
production IK, collision engine, controller feedback adapter, setpoint
transport, or real-time control loop.

## Product constraints requiring later decisions

- Camera count, placement, frame rate, exposure, depth source, and calibration method.
- Ball size, surface, expected speed range, impacts, occlusion, and acceptable miss rate.
- Gripper geometry, close time, contact tolerance, payload, and table clearance.
- Robot encoders, available velocity feedback, controller update rate, and firmware replacement tolerance.
- ROS 2 adoption versus a smaller custom real-time controller interface.
- Licensing constraints: some Ruckig tracking and fully local waypoint capabilities are documented as Pro features, so edition-specific capability must be verified before selection.
