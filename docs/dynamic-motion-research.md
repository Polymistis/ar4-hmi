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

## Implemented deterministic foundation

`ARrobots.dynamic_motion` provides a hardware-free object-state estimator and
replay boundary. The current model uses two ordered position observations to
calculate constant velocity. Each observation carries an explicit
coordinate-frame name, monotonic source timestamp, and diagonal position
variance. Measurement errors are assumed independent across observations and
coordinate axes; later camera calibration and estimator work must replace that
assumption when correlated uncertainty becomes available.

Estimator configuration bounds observation age, future clock skew, minimum
sample interval, and maximum sample interval. Rejected observations leave the
active baseline and estimate unchanged. A gap beyond the configured maximum
accepts the new observation only as a replacement baseline, making model reset
visible to the caller. The predictor uses a bounded future timestamp, preserves
constant velocity, propagates position/velocity covariance, and adds configured
diagonal position-process variance per second.

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

Current scope stops before acceleration estimation, transforms, innovation
tests, impacts, production IK or collision checking, trajectory generation,
controller setpoint replacement, or grasp supervision. No live-arm result is
established by the deterministic module or associated tests.

## Implemented deterministic intercept selection

`ARrobots.interception` extends the hardware-free foundation through bounded
intercept-time selection. Configuration defines a coordinate frame, estimate
age and future-skew limits, minimum and maximum lead times, a sampling
interval, maximum predicted position standard deviation, maximum terminal
object speed, and minimum arrival margin. Configuration rejects a lead-time
interval that cannot produce strictly advancing samples. Selector construction
fails when the predictor cannot cover the permitted estimate age plus lead
time, and selection fails when absolute timestamps cannot represent strictly
advancing candidates.

Each predicted state first passes the uncertainty and terminal-speed limits.
A deterministic injected evaluator then classifies production-feasibility
domains as feasible, unreachable, joint-limited, singular, colliding, or
missing an arrival-time estimate. A feasible result supplies the minimum robot
arrival duration measured from the selector evaluation timestamp and an
application-defined non-negative risk score. The selector compares the largest
per-axis predicted position standard deviation with the uncertainty limit,
enforces the arrival margin, and ranks accepted candidates by risk, predicted
timestamp, and original candidate order. The evaluator receives the predicted
state and selection timestamp. Stale and future estimates return explicit
non-selected statuses without invoking the evaluator. Exceptions or malformed
evaluator output fail the selection boundary.

`select_replay_intercepts` validates the complete recorded estimator sequence
and the bounded record-by-candidate workload before feasibility evaluation,
then recomputes selection after every estimate update. That behavior exercises
deterministic target reselection after new observations; no geometric path,
trajectory, controller setpoint, or grasp is generated. The injected evaluator
provides simulation decisions only and does not establish physical
reachability, collision clearance, singularity margin, joint-limit compliance,
or arrival performance.

## Product constraints requiring later decisions

- Camera count, placement, frame rate, exposure, depth source, and calibration method.
- Ball size, surface, expected speed range, impacts, occlusion, and acceptable miss rate.
- Gripper geometry, close time, contact tolerance, payload, and table clearance.
- Robot encoders, available velocity feedback, controller update rate, and firmware replacement tolerance.
- ROS 2 adoption versus a smaller custom real-time controller interface.
- Licensing constraints: some Ruckig tracking and fully local waypoint capabilities are documented as Pro features, so edition-specific capability must be verified before selection.
