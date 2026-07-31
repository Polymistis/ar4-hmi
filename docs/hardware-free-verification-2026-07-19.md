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

The complete elevated Windows rerun on 2026-07-20 passed after rounded-motion wrist preservation, live control-frame response ownership, positive Modbus polling waits, and complete FAT-reserved filename-character rejection were added. An initial run encountered the known timing-sensitive startup-test boundary; the isolated contract and complete rerun passed without a related source change.

The 2026-07-21 complete elevated Windows rerun passed after stored-program playback gained explicit completed, rejected, and terminal-fault-reported outcomes. Source-contract coverage confirms that a reported Cartesian row fault closes playback without emitting a second error response.

The 2026-07-21 encoder-collision follow-up passed the complete elevated Windows suite after the ordinary sandbox denied access to a temporary persistence fixture. Source-contract coverage binds each reported `moveJ` fault to an emitted response, binds caller-condition polarity to the shared status policy, and confirms that an encoder collision emits the `EC`-bearing position response before stored playback stops.

The 2026-07-23 complete elevated Windows rerun passed after G-code storage
gained operation-specific terminal sets, pre-write and post-write delete
outcomes, directory-based reconciliation, an immutable displayed local path,
complete Listbox rollback, and conversion admission. Real local-file loads
exercise stale-result suppression, including an unterminated final row without
data loss. The startup-timeout fixture now drives its queued poll through a
bounded monotonic deadline instead of assuming that a fixed short sleep always
crosses the deadline. Expected platform-specific skips remained.

The 2026-07-24 bounded Windows regression passed the complete hardware-free
host suite with the expected platform- or environment-specific skips. Coverage
includes
durable G-code delete reconciliation, controller and SD-card identity binding,
a cross-process operation lease spanning serial write and response ownership,
no-follow owner-checked single-link lock and journal admission,
platform-specific durable replacement, exact schema typing, nonblocking
regular-file admission, bounded ASCII local-program loading,
horizontal-whitespace canonicalization, conversion pre-write cancellation,
failed worker-transfer cleanup,
asynchronous conversion admission and shutdown, identity lifecycle cleanup,
exact read-only rollback, and bounded temporary allocation for production and
test fixtures. Local-loader coverage rejects special files without blocking,
rejects ASCII DEL, preserves repeated rows, and confirms row-0 conversion
startup without an explicit selection. Main-controller identity cleanup rejects
a stale serial-handle request with a stable diagnostic, preserves the newer
binding, and propagates the invariant through the covered cleanup call paths.
The covered `WC` conversion paths preserve detailed media-identity errors without
entering the legacy motion-error renderer. A focused HMI source-contract run
also passed. Journal tests reject oversized,
invalid-UTF-8, hard-linked, symbolic-link, and nonregular state entries; the
POSIX FIFO check runs in a bounded child process. Journal lock tests used child
processes to verify exclusion while another process held the lock and while
the production storage worker owned write and read settlement, then verified
production acquisition after release and rejection of linked lock entries on
Windows and Ubuntu 26.04. Windows Job Object and POSIX process
group tests also verified descendant termination at the bounded compile-runner
deadline. Syntax compilation of the host, motion boundary, bounded
temporary-directory helper, and associated test modules completed with exit
code 0 under a short watchdog.

```text
Complete Windows host suite: Ran 461 tests in 7.262s; OK (skipped=3)
Complete Ubuntu host suite: Ran 461 tests in 17.909s; OK (skipped=2)
Focused HMI source contracts: Ran 289 tests in 5.688s; OK
Ubuntu storage and durability contracts: Ran 4 tests in 0.207s; OK
Windows process-tree timeout contract: Ran 1 test in 1.038s; OK
Ubuntu process-tree timeout contract: Ran 1 test in 1.012s; OK
Syntax compilation: exit code 0
```

The 2026-07-26 convergence rerun passed after local G-code sources without an
actionable row were rejected before view replacement or controller-file
deletion. Storage
coverage now injects an operation-lock close-reporting failure along with a
worker-visible release error after a validated controller response plus durable
journal settlement. The definitive conversion callback remains successful,
the cleared journal survives direct rereading, process-local reconciliation
state records the release fault, and the next storage request reacquires the
lease and reloads the journal. The complete obsolete commented firmware
`writeSD` debug block was removed.

The initial Ubuntu aggregate omitted the isolated `pybind11 2.13.6`
`PYTHONPATH` and failed only the Linux native-binding source-build fixture
before compilation. The targeted fixture and complete aggregate passed after
the documented dependency boundary was restored. No source change was made for
that environment failure.

```text
Complete Windows aggregate: Ran 461 tests in 29.959s; OK (skipped=2)
Complete Ubuntu aggregate: Ran 461 tests in 15.857s; OK (skipped=2)
Focused HMI source contracts: Ran 289 tests in 5.450s; OK
Targeted G-code repair contracts: Ran 3 tests in 1.086s; OK
No-upload Teensy compile and timeout contracts: Ran 2 tests in 28.730s; OK
Linux native source-build retry: Ran 1 test in 6.012s; OK
Syntax compilation: exit code 0
```

A later 2026-07-26 convergence rerun covered atomic program/conversion
admission, ordinary motion and controller-replacement rejection throughout
conversion, native-kinematics preflight, inherited row-worker motion ownership,
and pre-delete rejection of blank or comment-only local programs. The Windows
aggregate enabled the no-upload Teensy compile environment. The Ubuntu
aggregate used CPython 3.14.4
with the isolated `pybind11 2.13.6` dependency path. No serial port,
controller command, firmware upload, calibration action, or arm motion
occurred.

```text
Complete Windows aggregate: Ran 461 tests in 29.477s; OK (skipped=2)
Complete Ubuntu aggregate: Ran 461 tests in 15.576s; OK (skipped=2)
Focused HMI source contracts: Ran 289 tests in 5.269s; OK
No-upload Teensy compile and timeout contracts: Ran 2 tests in 23.732s; OK
Syntax compilation: exit code 0
```

A subsequent 2026-07-26 convergence rerun covered the shared lock-protected
admission boundary between asynchronous operator storage, conversion, and
every program execution mode. Active `RG` and `DG` requests reject Run, Step
Forward, Step Reverse, and conversion before program state, selection, or
worker startup changes. Program ownership rejects storage before motion or
serial acquisition. Storage ownership persists through Tk result application
and transport release, while incomplete cleanup retains admission fail-closed.
The Windows aggregate enabled the no-upload Teensy compile environment. The
Ubuntu aggregate used the isolated native-build dependency path. No serial
port, controller command, firmware upload, calibration action, or arm motion
occurred.

```text
Complete Windows aggregate: Ran 461 tests in 38.595s; OK (skipped=2)
Complete Ubuntu aggregate: Ran 461 tests in 19.622s; OK (skipped=2)
Focused HMI source contracts: Ran 289 tests in 7.094s; OK
Targeted storage/program admission contracts: Ran 3 tests in 1.041s; OK
Syntax compilation: exit code 0
```

A post-review 2026-07-26 convergence rerun covered delayed conversion-worker
handoff across local Stop and application shutdown, final G-code row-write
cancellation admission, retryable serial-activity release, and retained
G-code storage cleanup. Result-application and startup failures stop at the
first failed release, preserve conflicting-work admission, retry outside Tk,
and defer conversion settlement until complete cleanup. Shutdown also waits
for retained storage cleanup. The Windows aggregate enabled the no-upload
Teensy compile environment. The Ubuntu aggregate used the isolated
native-build dependency path. No serial port, controller command, firmware
upload, calibration action, or arm motion occurred.

```text
Complete Windows aggregate: Ran 462 tests in 42.670s; OK (skipped=2)
Complete Ubuntu aggregate: Ran 462 tests in 21.011s; OK (skipped=2)
Focused HMI and transport suite: Ran 456 tests in 7.908s; OK
Targeted cleanup and cancellation contracts: Ran 5 tests in 1.101s; OK
Syntax compilation: exit code 0
```

A later 2026-07-26 review-remediation rerun covered SD-volume identity
binding. Firmware initialization now probes the current CID before reusing a
mount, reinitializes the filesystem after an identity change, validates
mutation against both mounted and current media, and emits directory identity
from the mount binding only after a final current-card check. The Windows
aggregate compiled the tracked Teensy source without upload. The Ubuntu
aggregate rebuilt the sanitized native contracts. No serial port, controller
command, firmware upload, calibration action, or arm motion occurred.

```text
Complete Windows aggregate: Ran 462 tests in 42.555s; OK (skipped=2)
Complete Ubuntu aggregate: Ran 462 tests in 20.667s; OK (skipped=2)
Focused HMI and transport suite: Ran 456 tests in 8.682s; OK
No-upload Teensy compile and timeout contracts: Ran 2 tests in 39.132s; OK
Targeted SD mount source contracts: Ran 2 tests in 0.539s; OK
Syntax compilation: exit code 0
```

An earlier sandboxed HMI source-contract invocation is failed evidence. Windows
denied child-Python temporary-file creation while the standard-library
`tempfile` search treated the candidate directory as writable, leaving
`tempfile._mkstemp_inner` in a retry loop for hours before forced
termination. The G-code journal writer and repository test fixtures now use
bounded exclusive-name allocation: access failures propagate immediately and
filename collisions stop after the fixed retry limit. Test fixtures require a
validated parent outside the source tree and accept
`AR4_TEST_TEMP_DIRECTORY` when the operating-system default is unavailable.
Accepted reruns used explicit operating-system process watchdogs.

```text
Terminated process elapsed time: 06:18:40
```

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

The 2026-07-20 sanitizer rerun for motion-mode transaction atomicity passed. Rejection preserves the active wrist selector and encoder loop modes, while an accepted commit updates wrist-selector and encoder-loop state together and remains idempotent.

The 2026-07-20 sanitizer rerun for the serial-frame accumulator passed with strict warnings enabled. Maximum-length frames complete, oversized unterminated frames clear accumulated storage, discard through the next LF, and accept a later valid frame. Host fixtures also reject live-jog modes, vectors, and linear rounding outside the paired firmware domain.

The 2026-07-20 sanitizer rerun passed exact LF and CRLF live-stop classification, rejection of complete non-stop and overflow control frames, single terminal-response selection, positive Modbus polling waits, rounded `ML` parsing with `WN` and `WF`, and every FAT-reserved filename character. Compilation used strict warnings plus AddressSanitizer and UndefinedBehaviorSanitizer and completed with exit code 0.

The 2026-07-21 sanitizer rerun passed upper-address Modbus register-span boundaries and bounded stored-row accumulation, including maximum-length completion, overflow clearing, invalid-read rejection, and unterminated final-row completion. Compilation used strict warnings plus AddressSanitizer and UndefinedBehaviorSanitizer and completed with exit code 0.

The 2026-07-21 playback-policy sanitizer rerun passed every motion-result outcome: only completed motion advances stored playback, only unreported rejection requests the generic error response, and a reported terminal fault stops playback without duplicate output. Compilation used strict warnings plus AddressSanitizer and UndefinedBehaviorSanitizer and completed with exit code 0.

The 2026-07-24 Ubuntu 26.04 sanitizer rerun covered the controller hardware-ID
formatter, SD-media-ID formatter, media-bound delete grammar, revised identity
JSON contract, and case-insensitive controller filename identity. Strict
compilation plus AddressSanitizer and UndefinedBehaviorSanitizer completed
successfully and quickly under a short Linux process watchdog.

```text
Sanitized contract harness: exit code 0; elapsed 6.2s
```

## Teensy 4.1 firmware

Toolchain:

- Arduino CLI 1.5.1
- Teensy platform 1.62.0
- ModbusMaster 2.0.1

The tracked setup procedure in [`ArduinoSketches/README.md`](../ArduinoSketches/README.md) installs the pinned platform and library. Named environment inputs are `AR4_ARDUINO_CLI`, `AR4_TEENSY_BUILD_DIRECTORY`, and `AR4_TEENSY_SPI_LIBRARY`; named build and library directories remain outside the tracked source tree.

Compilation command:

```powershell
& $env:AR4_ARDUINO_CLI compile --verbose --fqbn teensy:avr:teensy41 --clean --build-path $env:AR4_TEENSY_BUILD_DIRECTORY --library $env:AR4_TEENSY_SPI_LIBRARY ArduinoSketches\AR4_teensy41_sketch_v6.7.1
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

The 2026-07-21 encoder-collision no-upload compile succeeded after `moveJ` propagated the reported collision outcome to direct and stored-program callers. Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-23 no-upload compile succeeded after the paired
`GCODE_DIRECTORY_FRAMING_V1` capability, bounded multi-capability identity
producer, directory-separator filename validation, and incompatible SD filename
migration response were added. A later no-upload rerun succeeded after the
directory producer gained the shared 4096-byte aggregate payload bound,
allocation and overflow responses, and reversible `.txt` entry validation. The
environment-gated
`tests/test_teensy_firmware_compile.py` path exercised the tracked sketch with
PJRC Teensy core 1.62.0, bundled SdFat 2.1.2, the platform `SPI` library, and
ModbusMaster 2.0.1.
Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

The 2026-07-24 environment-gated no-upload compile passed within the short
process-tree watchdog after controller hardware identity, SD-card CID framing,
media-bound deletion, fail-closed directory iteration, and error-aware delete
lookup were added. The same pinned Teensy core, bundled SdFat release, platform
`SPI` library, and ModbusMaster release were selected from verbose compiler
dependency output. The regression parses the dependency paths and requires the
active SPI and SdFat folders under the selected Teensy platform.
The `6.7.1-ar4hmi.2` rerun also covered CID revalidation around directory
traversal, lookup, deletion, and `WC` or `WG` file writes, plus the paired
`GCODE_WRITE_IDENTITY_V1` capability.
Directory-entry extraction was then made compatible with SdFat boolean and
length return contracts by deriving the bounded output length from the
terminated buffer. Native behavioral fixtures cover boolean, length, failed,
and unterminated reads, and the no-upload compile pins the active
bundled SdFat 2.1.2 dependency.
The compile runner uses a kill-on-close Windows Job Object or a POSIX process
group, and a separate forced-timeout regression confirmed descendant cleanup.
Upstream ModbusMaster warnings remained; no tracked-source warning was emitted.

```text
No-upload Teensy compile and timeout tests: Ran 2 tests in 21.416s; OK
```

The 2026-07-31 environment-gated rerun compiled the tracked
`6.7.1-ar4hmi.8` Teensy source. Arduino CLI 1.5.1 selected PJRC Teensy core
1.62.0, the platform `SPI` library, bundled SdFat 2.1.2, and ModbusMaster
2.0.1. The bounded test runner used an external build directory, compiled
without `--upload`, verified the selected dependency paths, and exercised
forced-timeout process-tree cleanup. No serial port, controller command,
firmware upload, calibration action, or arm motion occurred.

```text
No-upload Teensy compile and timeout tests: Ran 2 tests in 18.084s; OK
```

## Nano and Mega auxiliary firmware

Toolchain:

- Arduino CLI 1.5.1
- Arduino AVR platform 1.8.8
- Servo 1.3.0

The 2026-07-31 environment-gated no-upload regression compiled the tracked
Nano sketch with `arduino:avr:nano:cpu=atmega328old` and the tracked Mega
sketch with `arduino:avr:mega`. Both builds used external temporary
directories and verified the selected platform and Servo release from verbose
compiler output.

Both compilers returned exit code 0 and reported substantial program-storage
and dynamic-memory headroom.

The paired firmware uses byte-identical fixed-buffer protocol headers because
Arduino's sketch builder rejects the attempted parent-directory header include.
Source-contract coverage rejects header drift, dynamic Arduino `String`
parsing, setup-time servo attachment, autonomous current-driven servo writes,
blocking wait loops, duplicate input sampling, and mismatched response framing.
The Ubuntu 26.04 sanitized C++ harness directly exercised strict command
parsing, board-specific pin and servo domains, integer overflow rejection,
caller-state preservation, frame overflow recovery, positive wait admission,
rollover-safe pending, match, timeout, cancellation, and active-wait command
disposition. Strict compilation plus AddressSanitizer and
UndefinedBehaviorSanitizer completed with exit code 0.

No serial port, controller command, firmware upload, calibration action, or
arm motion occurred.
