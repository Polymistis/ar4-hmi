# Hardware-free verification - 2026-07-19

Scope: selective 7.0 native-kinematics and Teensy integration. No serial port was opened, no controller command was sent, no firmware was uploaded, and no robot motion was attempted.

## Host regression

Toolchains: CPython 3.14.4 with NumPy 2.3.5 on Ubuntu 26.04 under WSL, plus CPython 3.12.10 on Windows for syntax and packaged-binding checks.

Named environment inputs:

- `AR4_LINUX_PYTHON`: CPython 3.14 executable.
- `AR4_PYTHON_TEST_SITE`: isolated dependency directory outside the repository.

Dependency setup:

```bash
"$AR4_LINUX_PYTHON" -m pip install \
  --target "$AR4_PYTHON_TEST_SITE" \
  numpy==2.3.5 \
  -r ARrobots/src/requirements-build.txt
```

Command:

```bash
PYTHONPATH="$AR4_PYTHON_TEST_SITE" "$AR4_LINUX_PYTHON" -m unittest -v \
  tests.test_hmi_source_contracts \
  tests.test_joint_motion \
  tests.test_native_kinematics
```

Result: the complete hardware-free suite passed. The bundled legacy Linux module produced the expected unsupported-runtime skip, while the current Linux binding was built and imported from source by a separate test in the same run. Temporary calibration fixtures and build outputs remained outside the tracked worktree.

Syntax compilation also succeeded for `AR4.py`, `ARrobots/HMI/joint_motion.py`, and the associated Python test modules without importing `AR4.py`.

The complete suite was rerun on 2026-07-20 after the firmware command-domain and cross-review remediation. The Windows run exercised the packaged CPython 3.12 binding and passed with the expected Linux source-build and GNU sanitizer skips. The Ubuntu run built the Linux binding from tracked source, executed the sanitized native harness, and passed with the expected packaged-Windows-module skip. The Windows runner required a temporary directory with ordinary create and remove access for calibration-persistence fixtures. Command-specific Modbus response classification, the shared host/firmware ramp and filename boundaries, exact firmware ingress preprocessing, and legacy EEPROM migration were included.

Another complete 2026-07-20 run passed on Windows and Ubuntu after live-jog profile forwarding, unsupported motion-option rejection, complete traversed arc-length calculation, and invalid legacy debug-byte handling were added. The Windows run retained the expected Linux source-build and GNU sanitizer skips; the Ubuntu run retained the expected packaged-Windows-module skip.

A 2026-07-20 convergence run passed the complete Ubuntu suite with exact old-`SR` migration for untouched erased debug storage, nearest in-range multi-turn wrist normalization, packaged 740-degree target continuity from a 730-degree estimate, and shared ordered major-arc execution geometry. A targeted Windows source-contract and loaded-binding run passed after rebuilding the packaged CPython 3.12 extension. A complete Windows rerun reached the suite but is not counted as green evidence because the sandbox denied deletion of a temporary calibration fixture and the approval service rejected the required unsandboxed rerun. No controller was opened by either run.

A complete elevated Windows run on 2026-07-20 passed after the live-jog convergence remediation, with the expected Linux-only source-build and sanitizer skips. Hardware-free coverage exercised encoded motion-profile forwarding through every offline live worker, Percent-only live-mode rejection across every firmware live command kind, and guarded live-stop serial reads. The sanitized native harness passed separately under Ubuntu.

A later complete elevated Windows run on 2026-07-20 passed after motion-mode transaction atomicity and command-local wrist routing were added, with the same expected platform-specific skips. An intervening timing-sensitive startup test failure passed in isolation and on the complete rerun; no source change was made for that unrelated transient failure.

The complete elevated Windows suite passed again on 2026-07-20 after cross-review remediation for main-controller output handling, live-jog domain parity, linear-rounding validation, and bounded firmware serial ingress. The ordinary sandbox run was not counted because Windows denied cleanup of an unrelated temporary persistence fixture; the approved rerun completed with only the expected platform-specific skips.

The complete elevated Windows rerun on 2026-07-20 passed after rounded-motion wrist preservation, live control-frame response ownership, positive Modbus polling waits, and complete FAT-reserved filename-character rejection were added. An initial run encountered the known timing-sensitive startup-test boundary; the isolated contract and the complete rerun both passed without a related source change.

The 2026-07-21 complete elevated Windows rerun passed after stored-program playback gained explicit completed, rejected, and terminal-fault-reported outcomes. Source-contract coverage confirms that a reported Cartesian row fault closes playback without emitting a second error response.

## Windows native extension

Toolchain:

- Python 3.12.10 x64
- pybind11 2.13.6
- Visual Studio Build Tools generator `Visual Studio 18 2026`
- MSBuild 18.5.4.18101
- Windows SDK 10.0.26100.0
- CMake 4.2.3-msvc3

Named environment inputs:

- `AR4_WINDOWS_PYTHON`: CPython 3.12 executable with the pinned build dependencies installed.
- `AR4_NATIVE_BUILD_DIRECTORY`: isolated native build directory outside the repository.

Build and installation command:

```powershell
powershell -ExecutionPolicy Bypass -File ARrobots\src\build_kinematics.ps1 -Python $env:AR4_WINDOWS_PYTHON -BuildDirectory $env:AR4_NATIVE_BUILD_DIRECTORY -Install
```

Result: Release compilation succeeded and installed the ABI-tagged `ARrobots/robot_kinematics.cp312-win_amd64.pyd` module.

Loaded-binding verification command:

```powershell
& $env:AR4_WINDOWS_PYTHON -m unittest -v tests.test_native_kinematics
```

Result: loaded-binding checks passed. The sanitized GNU C++ harness check was skipped on the Windows runner because `g++` was unavailable on the Windows path.

The CPython 3.12 extension was rebuilt again on 2026-07-20 after multi-turn normalization changed. The targeted loaded-binding contract reproduced the 740-degree target with a 730-degree estimate under configured 800-degree limits and passed.

## Linux native extension source build

Toolchain:

- Python 3.14.4 x64
- pybind11 2.13.6
- CMake 4.2.3
- GNU C++ 15.2.0

Verification command:

```bash
PYTHONPATH="$AR4_PYTHON_TEST_SITE" "$AR4_LINUX_PYTHON" -m unittest -v \
  tests.test_native_kinematics.NativeKinematicsContractTests.test_linux_python_binding_source_build_and_import
```

Result: the tracked `build_kinematics.sh` produced an ABI-tagged module in an isolated temporary directory. A fresh Python subprocess imported that module, applied the tracked kinematic geometry through the atomic configuration setter, exercised direct and atomic tool-frame representability boundaries, solved a forward-kinematics target through the configured inverse-kinematics entry point, and verified the returned solution by forward round trip. Reset operation and joint-limit readback were also exercised. No Linux binary was installed into the repository.

## Sanitized portable native and firmware contracts

Toolchain: Ubuntu 26.04 under WSL with g++ 15.2.0.

Command:

```bash
g++ -std=c++14 -O1 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer -pthread tests/native/kinematics_contract_test.cpp -o /tmp/ar4_kinematics_contract_test
ASAN_OPTIONS=abort_on_error=1:detect_leaks=1 /tmp/ar4_kinematics_contract_test
```

Result: compilation and execution completed with exit code 0 under AddressSanitizer and UndefinedBehaviorSanitizer. The harness executes native boundary, singularity, shared native/firmware wrist candidate generation, cross-seam physical-displacement parity, tool-frame representability, tool geometry, Cartesian display/wire-to-native ordering, degree-to-radian underflow rejection, signed tool-jog TCP displacement, strict firmware numeric parsing, firmware wrist-selection rejection, firmware identity, command-queue, debug-command, and EEPROM transaction contracts. Numeric fixtures pass canonical `JT`, `MJ`, and `MV` command text through the complete shared handler parser used by the sketch. Coverage includes decimal jog timing, required linear rounding, rounding rejection for non-linear opcodes, wrist configuration, binary loop modes, malformed or trailing fields, non-finite values, exponent form, overflow, nonzero underflow, and transactional rejection. EEPROM fixtures include marker-valid corrupt identity records, interrupted identity transaction reload, injected write failures, and reload behavior.

The 2026-07-20 rerun additionally exercised anchored joint, linear, and live-jog grammars; calibrated target and reference-step conversion; wait and Modbus domains; safe controller filenames; stored-step and interpolated targets; nondegenerate line, circle, and arc inputs; and bounded pulse-delay conversion. Invalid inputs retain staged outputs unchanged.

A later 2026-07-20 rerun passed serial and SD frame extraction through the actual shared motion parser, preserved payload whitespace for strict rejection, compared host and firmware behavior across ramp and filename boundary corpora, and exercised current-schema and legacy EEPROM records. Legacy fixtures cover preserved printable identity values, erased identity fields, preserved debug state, idempotent migration, invalid legacy payloads, and injected migration write failure.

The 2026-07-20 sanitized rerun covering threshold-sensitive quarter arcs, binary legacy-debug validation with no-write assertions, live-jog profile preservation, strict `SR` identity-field extraction, and fail-closed handling for reserved identity markers, unsupported wrist suppression, trajectory rotation, non-linear rounding, and joint-live wrist selection completed with exit code 0 under AddressSanitizer and UndefinedBehaviorSanitizer.

The 2026-07-20 sanitizer rerun for exact old-`SR` erased-debug migration, direct and integrated multi-turn wrist selection, and ordered 210-degree arc center, axis, radius, and traversal-angle contracts passed. The complete Ubuntu suite separately rebuilt and imported the current Linux extension from tracked source.

The 2026-07-20 sanitizer rerun for motion-mode transaction atomicity passed. Rejection preserves the active wrist selector and encoder loop modes, while an accepted commit updates both state groups together and remains idempotent.

The 2026-07-20 sanitizer rerun for the serial-frame accumulator passed with strict warnings enabled. Maximum-length frames complete, oversized unterminated frames clear accumulated storage, discard through the next LF, and accept a later valid frame. Host fixtures also reject live-jog modes, vectors, and linear rounding outside the paired firmware domain.

The 2026-07-20 sanitizer rerun passed exact LF and CRLF live-stop classification, rejection of complete non-stop and overflow control frames, single terminal-response selection, positive Modbus polling waits, rounded `ML` parsing with `WN` and `WF`, and every FAT-reserved filename character. Compilation used strict warnings plus AddressSanitizer and UndefinedBehaviorSanitizer and completed with exit code 0.

The 2026-07-21 sanitizer rerun passed upper-address Modbus register-span boundaries and bounded stored-row accumulation, including maximum-length completion, overflow clearing, invalid-read rejection, and unterminated final-row completion. Compilation used strict warnings plus AddressSanitizer and UndefinedBehaviorSanitizer and completed with exit code 0.

The 2026-07-21 playback-policy sanitizer rerun passed every motion-result outcome: only completed motion advances stored playback, only unreported rejection requests the generic error response, and a reported terminal fault stops playback without duplicate output. Compilation used strict warnings plus AddressSanitizer and UndefinedBehaviorSanitizer and completed with exit code 0.

## Teensy 4.1 firmware

Toolchain:

- Arduino CLI 1.5.1
- Teensy platform 1.62.0
- ModbusMaster 2.0.1

The tracked setup procedure in [`ArduinoSketches/README.md`](../ArduinoSketches/README.md) installs the pinned platform and library. Named environment inputs are `AR4_ARDUINO_CLI`, `AR4_TEENSY_BUILD_DIRECTORY`, and `AR4_TEENSY_SPI_LIBRARY`; named build and library directories remain outside the tracked source tree.

Compilation command:

```powershell
& $env:AR4_ARDUINO_CLI compile --fqbn teensy:avr:teensy41 --clean --build-path $env:AR4_TEENSY_BUILD_DIRECTORY --library $env:AR4_TEENSY_SPI_LIBRARY ArduinoSketches\AR4_teensy41_sketch_v6.7.1
```

Result: compilation succeeded without `--upload` and reported ample flash and RAM headroom. Compilation establishes source and selected-toolchain compatibility only; no protocol behavior, correlated JSON readiness, emergency-event readiness, or live-arm behavior is established by this command.

Compiler warnings originated from upstream ModbusMaster 2.0.1: unused `crc16_update` and a potentially uninitialized `read` local in `ModbusMaster.cpp`. No warning originated from the tracked Teensy sketch or contract headers.

The 2026-07-20 no-upload rerun covering command-local target validation, derived timing envelopes, checked pulse-delay conversion, exact serial and SD ingress preprocessing, SD mutation result handling, classified Modbus wait results, transactional legacy EEPROM migration with strict debug-byte validation, strict `SR` identity parsing, validated live-jog profile forwarding, opcode-specific rounding and wrist policy, unsupported motion-option rejection, and guarded line, circle, and complete arc geometry succeeded with upstream ModbusMaster warnings and no tracked-source warning.

The 2026-07-20 no-upload rerun for shared ordered arc execution on midpoint-selected major paths, safe erased-debug legacy migration, and multi-turn wrist normalization succeeded with ample flash and RAM headroom. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-20 no-upload compile for Percent-only live parsing, matching offline/controller motion profiles, and guarded live-stop serial reads also succeeded with ample flash and RAM headroom. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-20 no-upload compile for command-local wrist selection and driver-preflight-boundary wrist and encoder-loop-mode commits succeeded with ample flash and RAM headroom. Zero-distance, invalid timing or direction, and already-stopped requests return before mode commit. An initial unpinned invocation selected the incompatible user `SPI` library; the documented explicit PJRC Teensy 1.62.0 `SPI` library selection produced the successful result. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-20 no-upload compile after output-command and serial-frame remediation succeeded with ample flash and RAM headroom. Main-controller `ON` and `OF` now return the standard error response without a GPIO mutation path, while the profiled Nano and Mega auxiliary output contracts remain unchanged. Every USB `Serial` line reader uses the shared bounded frame accumulator. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-20 no-upload compile after live terminal-response ownership, rounded-motion wrist propagation, positive Modbus polling waits, and FAT filename validation succeeded with ample flash and RAM headroom. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-21 no-upload compile after Modbus span validation, pre-acknowledgement live-command validation, single-response finite-trajectory faults, volatile emergency-stop polling state, and bounded SD playback rows succeeded with ample flash and RAM headroom. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-21 playback-policy no-upload compile succeeded after `moveJ` gained explicit completed, rejected, and terminal-fault-reported results and PG stopped on every non-completed Cartesian row. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.
