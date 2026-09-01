# Windows third-party redistribution status

This file records required release treatment; this file is not a completed
recipient notice bundle and grants no redistribution permission. Current
package manifests retain `redistributionApproved=false`.

[`build.ps1`](build.ps1) emits `distribution-notices.json` from installed wheel
metadata and declared `License-File` entries. That JSON is an evidence inventory
only. A distributable package still requires copied license texts,
corresponding source and replacement material where applicable, exact native
file ownership, and separate design-file and Microsoft authority.

## Component obligations

| Component | Required recipient treatment | Current status |
| --- | --- | --- |
| AR4HMI software and `robot_kinematics` | Preserve [`LICENSE.txt`](../../LICENSE.txt), attribution, change notice, non-commercial use, and free distribution conditions. | Software terms available; bundled design files remain a separate block. |
| CPython 3.14.7 | Copy the official installer-root `LICENSE.txt`, including incorporated-code terms. | Text available from the locked installer. |
| PyInstaller 6.22.2 bootloader | Preserve exact wheel/source terms and verify applicability of upstream generated-bundle guidance. | Release decision still required. |
| Installed Python distributions | Copy every declared and audited wheel license file for every retained runtime distribution. | Metadata inventory exists; recipient bundle does not. |
| `inputs` 0.5 | Add the canonical release BSD text; the wheel contains no license file. | Supplemental text required. |
| pyserial 3.5 | Add the tagged BSD-3-Clause `LICENSE.txt`; the wheel contains no license file. | Supplemental text required. |
| ttkbootstrap 2.2.2 | Preserve project and Bootstrap-icons MIT texts and resolve the declared `Apache-2.0 OR BSD-2-Clause` alternative. | Wheel texts are incomplete. |
| NumPy 2.5.2 | Preserve every wheel `License-File`, including the platform aggregate and nested license tree. | Copy-ready from the exact wheel. |
| OpenCV-Python 5.0.0.93 | Preserve `cv2/LICENSE.txt` and `cv2/LICENSE-3RD-PARTY.txt`. | Copy-ready from the exact wheel. |
| Pillow 12.3.0 | Preserve the exact wheel license aggregate covering bundled image and font libraries. | Copy-ready from the exact wheel. |
| VTK 9.7.0 and 9.6.2 | Preserve root terms plus every applicable tagged module and bundled-project notice. | Version-specific native ownership and license aggregates remain unresolved. |
| CadQuery 2.8.0 | Preserve the wheel Apache-2.0 text and applicable source notices. | Copy-ready from the exact wheel. |
| cadquery-ocp 7.9.3.1.1 / OCCT 7.9.3 | Preserve OCP Apache-2.0, OCCT LGPL-2.1 plus exception, corresponding modified source, build instructions, and compatible replacement material; resolve every bundled native dependency. | Native provenance and replacement closure remain unresolved. |
| CasADi 3.8.0 / GCC / MinGW | Preserve the full CasADi wheel license tree, LGPL-3.0-or-later corresponding source and replacement mechanics, GCC GPL and Runtime Library Exception, libstdc++ terms, and MinGW terms. | Reproducible source and toolchain closure remain unresolved. |
| NLopt 2.11.0.post1+ar4hmi.1 | Preserve the packaging MIT text, all lock-named upstream notices, and `NLOPT-BUILD-MODE.txt`. | Luksan-free build provenance is reproducible; recipient bundle integration remains open. |
| Microsoft runtime DLLs | Bind every root and wheel-vendored runtime copy to an eligible redistributable source and applicable product-license entitlement. | File identity and signatures do not establish downstream authority. |
| Robot-link, Servo, and Welding Torch models | Obtain written redistribution authority covering every retained design file, or replace/remove restricted files through an approved product change. | [`LICENSE.txt`](../../LICENSE.txt) section 2.2 prohibits design-file redistribution. |

## Locked native source identities

- Base VTK 9.7.0 uses official tag commit
  `23f0a095621e91bbdbeace8451e22b950c8e5f46`; official source archive SHA-256
  `affdb7a15ec34ee0174407f911ab70b646c7af01161818bbab4e1160b7eff720`.
- STEP VTK 9.6.2 uses official tag commit
  `f49a1dbafa60b58ef22f6292ec58370453162192`; official source archive SHA-256
  `aed12cec12a9609179bf66329070266627ca64244a10856a452b2a17ffb04a1d`.
- OCP 7.9.3.1.1 resolves to commit
  `d69b064a3a604ebf245b1f3b14fb54c835a3a571`; ocp-build-system
  `v7.9.3.1.1` resolves to `df2c31c25b8fce57c497895aa514e9c3550c9f02`;
  OCCT tag `V7_9_3` resolves to
  `a016080bf6738d6aeae020badee4e888ad1540a5`.
- CasADi 3.8.0 resolves to commit
  `83b3cec864e42c5b64a07e85d4adf91da71458b1`. GCC 11.2.0 source archive
  SHA-512 is
  `d53a0a966230895c54f01aea38696f818817b505f1e2bfa65e508753fcd01b2aedb4a61434f41f3a2ddbbd9f41384b96153c684ded3f0fa97c82758d9de5c7cf`.
  MinGW-w64 tag `v9.0.0` resolves to
  `acc9b9d9eb63a13d8122cbac4882eb5f4ee2f679`.
- The custom NLopt wheel uses upstream commit
  `88c424d4f458412787df96fcc95218acbca224fd` and nlopt-python commit
  `b4d2871ff46e31b18aa0d45ad861fdfcd43202c3`, with
  `BUILD_SHARED_LIBS=OFF`, `NLOPT_LUKSAN=OFF`, and
  `NLOPT_PYTHON_SABI=OFF`. [`nlopt-build-lock.json`](nlopt-build-lock.json)
  fixes all source, tool, output, probe, and notice identities.

Source tags identify candidate corresponding source but do not prove historical
wheel build provenance. VTK closure requires independent profile-specific
license aggregates and exact non-overlapping native ownership. OCP/OCCT and
CasADi closure requires reproducible replacement builds rather than inference
from version strings.

## VTK closure contract

Only the official tagged VTK sources identified above may close the base and
STEP profiles. Each profile requires a version-specific license aggregate and a
non-overlapping native-owner map derived from the matching `vtk.module`
declarations and bundled-project records. Closure must resolve the Chemistry
MIT text, OpenXR archive treatment, and Holographic Remoting build-only
disposition without treating source identity, historical build attestation,
redistribution terms, or Microsoft entitlement as interchangeable evidence.

The selected OpenXR loader input is the official 1.1.45 archive, SHA-256
`b4806737309ed09ecda0b03eb11d037b6f00d83d0458c8f08f9f38ad6042171f`;
the retained loader DLL has SHA-256
`47d29b2f0e7df7fb367af5ce36fd428df0511f86fab6a279c4971f20237e0d2d`.
Microsoft Holographic Remoting OpenXR 2.9.2 is a build-only input; no
Holographic Remoting SDK runtime DLL is retained.

Validate each frozen archive with `Get-FileHash -Algorithm SHA256 -LiteralPath
<frozen-input>`. Validate each source checkout with `git -C <source-root>
rev-parse HEAD` against the recorded commit. Require `git -C <source-root>
status --porcelain=v1 --untracked-files=all --ignored=matching` to exit
successfully with no output before tracing every proposed owner and applicable
term with `rg -n <declaration> <source-root>`. A later implementation package
must freeze the aggregate and owner-map command inputs, expected output
identities, overlap checks, and negative coverage before source changes.

A source or wheel identity mismatch, owner-map gap or overlap, non-primary
authority, unavailable applicable text, unresolved external license decision,
or attempted package change before closure stops the release unit.

## Native file ownership contract

The release policy must classify every packaged `.exe`, `.dll`, and `.pyd`
under one owner and one disposition. Matching selectors may not overlap.

| Retained path family | Required owner |
| --- | --- |
| `ARrobots/robot_kinematics*.pyd` | AR4HMI source and build contract |
| Root CPython, Tcl/Tk, OpenSSL, libffi, zlib, and `VCRUNTIME140*.dll` files | Locked CPython installer |
| Root `MSVCP140.dll` | Eligible Microsoft redistributable source |
| `numpy/**/*.pyd` and `numpy.libs/*.dll` | Locked NumPy wheel plus applicable Microsoft terms |
| `cv2/cv2.pyd` | Locked OpenCV-Python wheel |
| `PIL/*.pyd` | Locked Pillow wheel |
| `win32/*.pyd` and `pywin32_system32/*` | Locked pywin32 wheel |
| `vtkmodules/*.pyd` and `vtk.libs/*.dll` | Profile-specific locked VTK wheel and version-specific aggregate |
| `OCP/*.pyd` and `cadquery_ocp.libs/*.dll` | Locked cadquery-ocp wheel, OCCT source, and replacement materials |
| `casadi/*.pyd` and `casadi/*.dll` | Locked CasADi wheel, corresponding source, GCC, and MinGW terms |
| Native files under `contourpy/`, `fontTools/`, `kiwisolver/`, and `matplotlib/` | Matching locked STEP-profile wheel |
| `nlopt/*.pyd` | Project-built Luksan-free NLopt wheel and frozen notice set |

The project executables and PyInstaller bootloader require explicit
first-party and build-tool entries. A family description alone cannot approve
an output file; final policy selectors must bind normalized paths and hashes
from the actual package manifests.

## Recipient bundle implementation contract

The remaining release implementation must make
`packaging/windows/notice_bundle.py` the sole recipient-bundle generator and
`packaging/windows/redistribution-lock.json` the sole offline policy owner. The
named generator and lock are planned release inputs and are not present in the
current tree. The lock must record each runtime distribution, copied path and
hash, supplemental
source, profile membership, native selector, source-required obligation, and
Microsoft provenance. A URL alone cannot satisfy a source-required row. The
generator must validate canonical paths, collisions, symlinks, input hashes,
source-required rows, and exact-one-owner native coverage before copying
notices, source, build instructions, and replacement material from an absolute
audited external input root.

The deterministic package output is
`THIRD-PARTY-NOTICES/README.txt`, `inventory.json`, copied license texts,
corresponding source archives, build instructions, and replacement materials.
Generation occurs after payload-only native and denied-artifact checks but
before final package records and manifest hashes. Notice or source filenames
therefore cannot satisfy payload selectors, while every generated file still
enters the final records.

Generated notice and source files must enter the final package records and
manifests. Recipient-bundle manifests revise the current schema `1` and add
`inputs.noticeBundleGeneratorSha256`, computed from the exact repository bytes
of `packaging/windows/notice_bundle.py` used for generation. The same revision
records the redistribution-policy hash, aggregate audited-input hash, generated
bundle hash, and explicit third-party closure status. Repeated clean builds must
verify the generator identity before invocation and reproduce every added field,
the recipient bundle, and package records byte-for-byte while
`diagnostics.redistributionApproved=false` remains unchanged until the separate
release decision.

Implementation stops before source changes while VTK, OCP/OCCT, CasADi,
GCC/MinGW, or Microsoft provenance remains unresolved. Missing canonical
terms, corresponding source, replacement material, unmatched native output,
overlapping selectors, network retrieval during an offline build, or an
unapproved package-layout change also stops the release unit.

## Recipient bundle requirements

A releasable base or STEP package must include a deterministic recipient bundle
covering every retained distribution and native file. Minimum closure requires:

1. Exact copied wheel license files and canonical supplemental texts.
2. Complete corresponding source, patches, build instructions, and compatible
   replacement material for every source-required native component.
3. Exactly one owner and disposition for every packaged `.exe`, `.dll`, and
   `.pyd`.
4. Qualifying Microsoft redistributable acquisition and entitlement records.
5. Written authority for every retained restricted design file, or an approved
   package recipe without restricted files.
6. Repeated offline package passes producing identical recipient bundles and
   package records while `redistributionApproved=false` remains unchanged until
   a separate release decision.

Missing, conflicting, overlapping, or network-fetched closure input fails the
release. Source and notice completion cannot override the separate design-file
restriction.
