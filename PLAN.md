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

Status: `Proposed`

Recorded priorities:

- Improve HMI responsiveness and maintainability by mapping active call paths, measuring event-loop delays, and removing verified redundant or shadowed functions.
- Keep Tk interaction responsive during repositioning without allowing conflicting controller commands.
- Verify coordinated multi-axis motion from host command through firmware pulse scheduling; distinguish controller-side interpolation from desktop command serialization.
- Keep stop controls and motion status available during movement through explicit command arbitration and main-thread UI updates.

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
- Priority order is approved before optimization work begins.

### M2 - Safe startup and dependency boundaries

Status: `Proposed`

Acceptance criteria:

- GUI construction, configuration loading, serial connection, and controller synchronization are explicit lifecycle operations.
- Importing testable modules cannot connect to hardware.
- Serial transports support deterministic fakes without changing production semantics.
- Runtime configuration is schema-validated before use.

### M3 - Regression test foundation

Status: `Proposed`

Acceptance criteria:

- Host protocol serialization and response parsing have hardware-free tests.
- `.ar4` parsing, editing, and execution-state transitions have focused tests.
- Kinematics boundary behavior and calibration conversion have deterministic tests.
- Firmware protocol fixtures cover command compatibility without energizing motors.

### M4 - Purpose-specific optimization

Status: `Blocked`

Optimization scope remains blocked on M1. Candidate work must trace to approved measurements such as responsiveness, motion-cycle latency, reliability, maintainability, or operator workflow.

Acceptance criteria:

- Baseline measurement and target are documented.
- Changes preserve safety invariants and protocol compatibility.
- Automated evidence demonstrates the intended improvement.

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
- Route every post-bootstrap commit through the role-appropriate cross-review wrapper.
- Route every branch integration through `scripts/codex/auto-merge.ps1`; bare merge into the integration base is prohibited.

## Current setup boundary

Repository setup changes only structure, review infrastructure, documentation, and Git metadata. Robot-control behavior remains unchanged.
