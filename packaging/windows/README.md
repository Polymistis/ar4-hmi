# Windows packaging

The tracked recipes build deterministic, flat one-directory Windows packages.
No generated package is cleared for public redistribution. Every generated
manifest records `redistributionApproved=false`; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for open release obligations.

## Supported profiles

Each supported profile requires official standard-GIL CPython 3.14.7 AMD64 and the
`cp314` ABI. The free-threaded `cp314t` ABI and CPython 3.12 are unsupported.
The profiles pin pip 26.2.1, PyInstaller 6.22.2, and
pyinstaller-hooks-contrib 2026.7.

| Profile | Lock and recipe | Output | Profile-specific runtime |
| --- | --- | --- | --- |
| Base | [`requirements-windows-base.lock`](../../requirements-windows-base.lock), [`AR4HMI-base.spec`](AR4HMI-base.spec) | `AR4HMI-base/AR4HMI.exe` | VTK 9.7.0; no CadQuery, OCP, CasADi, NLopt, or STEP worker |
| STEP | [`requirements-windows-step.lock`](../../requirements-windows-step.lock), [`AR4HMI-step.spec`](AR4HMI-step.spec) | `AR4HMI-step/AR4HMI.exe` and sibling `AR4StepWorker.exe` | VTK 9.6.2, CadQuery 2.8.0, cadquery-ocp 7.9.3.1.1, CasADi 3.8.0, and NLopt 2.11.0.post1+ar4hmi.1 |

The profiles use separate PyInstaller analysis graphs. The HMI graph excludes
the STEP worker, CadQuery, and OCP; the STEP collection adds the sibling worker
whose graph owns that native geometry stack. No in-process or ambient-Python
fallback is permitted in a frozen package. Each recipe uses
`contents_directory="."`; an `_internal` payload directory is prohibited.

Every lock row fixes an exact package version, wheel filename, and SHA-256.
Each lock requires wheel-only, hash-checked installation. The STEP lock uses
the project-built Luksan-free NLopt wheel; all other locked artifacts come from
PyPI.

The official `python-3.14.7-amd64.exe` input has SHA-256
`9d9eb2709ef81bf5cd30db3c2096bdbc4ea10087c22e62f27d356b36f6ae9649`.
The packaged native module is
`ARrobots/robot_kinematics.cp314-win_amd64.pyd`, with SHA-256
`6bb0d8cfe8b43317077f942f0ec87f7afaf7424af456c8d230d70985cf81272f`.

## Application-owned package data

The application-data manifest below is copied beside `AR4HMI.exe`.
PyInstaller owns collected dependency data such as Tcl/Tk and the constrained
`ttkbootstrap` asset set.

```text
AR.png                 defaults.json           information.txt
LICENSE.txt             VisBackdrop.png         xbox.png
play-icon.png           stop-icon.png           pp.gif
block.jpg               display setting.jpg     keystone jack.jpg
Link Base-1.STL         Link Base-2.STL         Link Base-3.STL
Link 1-1.STL            Link 1-2.STL            Link 2-1.STL
Link 2-2.STL            Link 2-3.STL            Link 3-1.STL
Link 3-2.STL            Link 4-1.STL            Link 4-2.STL
Link 4-3.STL            Link 5-1.STL            Link 5-2.STL
Link 6-1.STL            Link 6-2.STL            Servo Gripper.STL
Welding Torch.STL
```

Application-owned data uses no wildcard or directory collection. Requirements,
tests, firmware, development tools, `.ar4` examples, unused images, legacy
Linux native modules, and ignored machine state remain outside package output.

## Prerequisites

- Windows PowerShell 5.1 Desktop.
- Git available on `PATH` for source-identity checks.
- Official standard-GIL CPython 3.14.7 AMD64 plus the matching installer file.
- An absolute, empty build root outside the checkout.
- An absolute wheelhouse outside the checkout containing exactly the
  deduplicated union of all supported profile locks.
- No reparse-point ancestor for either external root; build and wheelhouse
  roots must be disjoint.
- Exact `SourceDateEpoch` value `1788115725`.

The build driver rejects unexpected wheelhouse children, missing wheels, hash
mismatches, source distributions, ambient pip configuration, incompatible
Python runtimes, overlapping roots, and nonempty build roots.

## Build the Luksan-free NLopt wheel

The STEP wheelhouse requires
`nlopt-2.11.0.post1+ar4hmi.1-cp314-cp314-win_amd64.whl`, SHA-256
`be580c5695c1afad3fddbf6497978fed5d544a6b9fab26c346bcc689a2c178bc`.
[`nlopt-build-lock.json`](nlopt-build-lock.json) fixes the NLopt and packaging
source commits, CPython installer, SWIG 4.4.1 archive, NumPy 2.5.2, pip 26.2.1,
setuptools 84.0.0, toolchain bytes, source transform, build flags, output
hashes, runtime probes, and source notices.

The builder requires CPython at `C:\Program Files\Python314\python.exe`, Visual
Studio 2026 Build Tools under the standard installation root, MSVC tools
14.50.35717, Windows SDK 10.0.26100.0, and the exact tool identities in the
lock. The frozen input root must contain only lock-named inputs.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File packaging/windows/build-nlopt-wheel.ps1 `
  -InputRoot <absolute-frozen-input-root> `
  -BuildRoot <absolute-empty-external-build-root>
```

The command performs repeated clean passes and validates identical wheels,
native extensions, normalized CMake caches, disabled-Luksan behavior, source
notices, and build-mode notices. Copy the verified `pass-1` wheel into the
external wheelhouse before the package build.

## Build the supported packages

Populate the wheelhouse with every exact wheel named by all supported profile
locks, including the verified custom NLopt wheel. The build itself is offline.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File packaging/windows/build.ps1 `
  -PythonPath <absolute-python-3.14.7-python.exe> `
  -PythonInstallerPath <absolute-python-3.14.7-amd64.exe> `
  -BuildRoot <absolute-empty-external-build-root> `
  -WheelhouseRoot <absolute-external-wheelhouse-root> `
  -SourceDateEpoch 1788115725
```

[`build.ps1`](build.ps1) creates isolated environments, builds each profile in
repeated clean passes, checks installed versions, validates analysis graphs and
package contents, exercises STEP statuses `64` and `3`, and requires
byte-identical package records and manifests between passes. Packages and
manifests appear beneath `pass-1` and `pass-2` inside the selected build root.

## STEP worker ABI

[`step_conversion.py`](../../ARrobots/HMI/step_conversion.py) validates and
privately copies a `.step` or `.stp` source, enforces its declared conversion
deadline and file-size bounds, validates the worker output as a stable regular
file, and removes temporary files. The HMI then passes the returned STL bytes
to [`PersistentCadScene.import_stl()`](../../ARrobots/HMI/cad_scene.py), which
owns geometry validation and durable publication. Frozen mode launches only the
absolute sibling `AR4StepWorker.exe`; no ambient-Python or in-process fallback
exists.

Frozen launch sets `PYINSTALLER_RESET_ENVIRONMENT=1`, uses
`CREATE_NO_WINDOW`, sets the executable directory as `cwd`, and always uses
`shell=False`. Source mode launches `-m ARrobots.HMI.step_worker` through the
active interpreter. On Windows, source mode also uses `CREATE_NO_WINDOW`; when
the active interpreter is a virtual-environment redirector, launch uses
`sys._base_executable` and sets `__PYVENV_LAUNCHER__` to the active interpreter.
Source-mode `cwd` is the repository root and every launch suppresses worker
stdin, stdout, and stderr.

[`step_worker.py`](../../ARrobots/HMI/step_worker.py) accepts exactly an input
path and output path. Stable process statuses are:

| Status | Meaning |
| ---: | --- |
| `0` | CadQuery reported binary STL export complete; host validation still applies. |
| `2` | CadQuery could not load. |
| `3` | STEP geometry could not be imported. |
| `4` | STL conversion failed. |
| `64` | Worker arguments were invalid. |

Any unknown status is a worker crash. Worker stdout, stderr, and exception text
do not enter the HMI error surface.

## Deployment and release validation

Package directories must remain writable. Startup creates `ARconfig.json`,
`ErrorLog`, and `cad-workspace/` beside the executable; additional calibration,
vision, trace, and CAD state can appear later. G-code storage state uses
`%LOCALAPPDATA%/AR4HMI`.

A release candidate requires a clean landed source revision, matching repeated
builds, complete redistribution clearance, and validation on a clean
Windows machine without system Python or serial-device passthrough. Package
construction and software-only launch checks do not constitute controller or
robot verification.

Clean-machine acceptance starts each profile from a restored Windows image,
verifies the external manifest, and launches `AR4HMI.exe` from an unrelated
working directory in a writable package copy. Device tracing must show no
serial-port open attempt; passive Windows device enumeration is allowed. Clean
defaults contain no enrolled controller identity, so an enumerated COM name
cannot authorize startup.

Acceptance covers the application window, Tcl/Tk and ttkbootstrap data,
information and image assets, VTK robot scene, bundled visual-tool selection
and restart persistence, OpenCV, the tagged native kinematics extension, normal
window shutdown, and complete process exit. Manifest-listed files must remain
unchanged; runtime state may create only documented writable-state paths.

The base package must retain STL CAD operation without a STEP worker, CadQuery,
OCP, Numba, LLVM, or TBB payload. STEP acceptance uses the retained
[`StepConversionTests.test_cadquery_round_trip_preserves_source_and_reloads_scene`](../../tests/test_step_conversion.py)
fixture recipe: export `cadquery.Workplane('XY').box(12,23,34)` as
`known-box.step`. Successful conversion must produce the expected 684-byte
binary STL, import through `PersistentCadScene`, and produce VTK bound extents
`(12.0, 23.0, 34.0)` to three decimal places. The STEP package must also
exercise missing arguments as status `64`, malformed STEP as status `3`, host
cancellation and cleanup, and absence of surviving worker processes. Controller
commissioning remains a separate procedure under [`SAFETY.md`](../../SAFETY.md).
