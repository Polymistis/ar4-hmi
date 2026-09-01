# V7 visualization asset provenance

## Source and attribution

The robot-link, Servo Gripper, and Welding Torch meshes listed below originate
from the official [Annin Robotics downloads](https://anninrobotics.com/downloads/)
entry `AR4 Version 7.0 (source) Beta`. The migration source archive is
[`AR4-HMI-7.0-source.zip`](https://anninrobotics.com/wp-content/uploads/2026/07/AR4-HMI-7.0-source.zip),
recorded with SHA-256
`ad23601777d438815be1c9f0b9b9fbb577744fab0cfa07b9aef583e97a636591`.

The design geometry is by Chris Annin / Annin Robotics. Copyright and all
design rights remain with Chris Annin. Modified HMI source in this fork uses the
meshes only as built-in robot and tool visualization inputs.

## Contribution-fork scope

Project authorization is limited to official V7 HMI visualization assets
required for HMI improvements intended for contribution to the official
`Annin-Robotics/ar4-hmi` parent. Every retained asset must have an official
source, a verified identity, a product requirement, and Chris Annin / Annin
Robotics attribution. This scope does not authorize a standalone model
collection, unrelated hosting, rebranding, commercial use, or a distributable
application package.

## Shipped asset identities

`ARrobots.HMI.robot_link_view.RobotLinkView` requires the following exact files
at runtime. Each supported Windows PyInstaller specification and
`packaging/windows/build.ps1` carry the same files in the application-data
manifest.

| Asset | Bytes | SHA-256 |
| --- | ---: | --- |
| `Link Base-1.STL` | 470,284 | `31527048758233fba9c4dab6ec800c7fb09a8ba7aa4149914937949c794994de` |
| `Link Base-2.STL` | 3,833,384 | `417e170998f9996ea7debfa66bc6936cf59bc967e894e92177dc4bb448f1cf70` |
| `Link Base-3.STL` | 7,598,384 | `4fea36b41b1dd8022d92319c551faccdcaa84d8c7e909bd81c8e7fcde711b990` |
| `Link 1-1.STL` | 223,984 | `cd52d8354224ccf3345ae5145ab721ac72adad52e6433f2a3adf880e4b8922da` |
| `Link 1-2.STL` | 217,484 | `d296169afd5178b6e5e1df68ec90246e1d887d183d122532e7ea8dd6d28adcaa` |
| `Link 2-1.STL` | 430,984 | `9e559edafbf8e74094bf740c27baf981f443b2626d7351eafc517b2357cb97c0` |
| `Link 2-2.STL` | 584,984 | `dd22181bcd244bc4e97e8a2d8d25105a25db6b0736a775e91256bba4c87f79a0` |
| `Link 2-3.STL` | 119,984 | `8d987a286855c259379c8efea258f7640cd8e622bc0f083dea58968fbc71a906` |
| `Link 3-1.STL` | 592,984 | `0f7ef9393e5763ccdc491365448cc9ce6abd9fa435ae289e4194f1f788da9a66` |
| `Link 3-2.STL` | 68,984 | `97deaa4c95d4bf011245c378e8b664909b3ee902357b1f7a68ae1091a18cd66a` |
| `Link 4-1.STL` | 226,184 | `1b5bc1b1ee2e9d3dcb81acbbaf29939798569d33d712d3dd186eef1b030931a5` |
| `Link 4-2.STL` | 287,984 | `7c18b34bdb0e30f3741ea45e2af0ef95770589eb17cc7a56e2ecf01d458fd998` |
| `Link 4-3.STL` | 820,584 | `de174afc0c58902a344cfa01ad598e5bab4207bf8118f9c04a8208ca43e04dca` |
| `Link 5-1.STL` | 190,884 | `f0f2a8be200f11df10cd1ca3927de7bf7c7c45810c52b9b19e1adecb269c4b22` |
| `Link 5-2.STL` | 54,084 | `ff5e4f22767d84be2a8aed0535fb7bdcfaf741b9f7186b6bac55d73615305361` |
| `Link 6-1.STL` | 42,184 | `69dae9a8e584d6d3a763ab931b54041ad77bd939785a94edb11b127da90bcfcd` |
| `Link 6-2.STL` | 20,484 | `b0d9b2269470286da8b679b3e4a074e298a65fc94519a14bdb8d085051e4261c` |
| `Servo Gripper.STL` | 1,648,384 | `fc5ab1c82ae0696ee2ec5c7cede12d9788cc14c2c162c3015cac042fa581c82b` |
| `Welding Torch.STL` | 6,080,084 | `b78866ae0d5a7135501c8309cc8951d9dc52e1eb67543426788a1a9769f50ecc` |

## Runtime transform contract

`RobotLinkView` owns the built-in mesh hierarchy, fixed transforms, joint
transforms, color roles, `tool_mount`, and selectable bundled-tool actors. The
assembly hierarchy is:

```text
Base-1 -> Base-2 -> Base-3 -> 1-1 -> 1-2 -> 2-1 -> 2-2 -> 2-3
       -> 3-1 -> 3-2 -> 4-1 -> 4-2 -> 4-3 -> 5-1 -> 5-2 -> 6-1 -> 6-2
```

`tool_mount` is attached directly below `Link 6-1.STL`, beside
`Link 6-2.STL`. Fixed operations are applied in the listed order before the
joint rotation:

| Joint part | Ordered fixed operations | Joint operation |
| --- | --- | --- |
| `Link 1-1.STL` | `RotateX(180)`, `RotateZ(-90)`, `Translate(0, 0, -92)` | `RotateZ(-J1)` |
| `Link 2-1.STL` | `RotateZ(-90)`, `RotateX(270)`, `Translate(-64.15, 77.78, 8.87)` | `RotateZ(+J2)` |
| `Link 3-1.STL` | `RotateZ(180)`, `RotateX(180)`, `Translate(0, 305, -27.84)` | `RotateZ(-J3)` |
| `Link 4-1.STL` | `RotateY(90)`, `RotateX(180)`, `Translate(-36.7, 0, -75.94)` | `RotateZ(-J4)` |
| `Link 5-1.STL` | `RotateZ(180)`, `RotateY(90)`, `Translate(147, 0, 44.88)` | `RotateZ(-J5)` |
| `Link 6-1.STL` | `RotateY(90)`, `Translate(43.3, 0, 25)` | `RotateZ(+J6)` |

All other link fixed transforms are identity. Link color roles resolve to
`Silver` for body, `Orange` for main, and `DimGray` for accent.

The supported bundled visual tools are:

| Selection | Asset | Position (mm) | Orientation (degrees) |
| --- | --- | --- | --- |
| `Servo Gripper` | `Servo Gripper.STL` | `(0, 0, 16.5)` | `(0, 0, 90)` |
| `Welding Torch` | `Welding Torch.STL` | `(0, 0, 17)` | `(0, 0, 90)` |

Servo Gripper is the default. `RobotLinkView` validates every supported tool
asset before publication and owns zero or one selected actor below a private
child of `tool_mount`. Unsupported names, missing files, asset-root escape, and
empty geometry fail closed. Tool appearance is color `(0.45, 0.45, 0.45)`,
ambient `0.25`, diffuse `0.75`, specular `0.08`, and specular power `8`.

`PersistentCadScene` separately owns imported CAD, workpiece identity,
transforms, parenting, and persistence. Imported objects may attach through
`tool_mount`, but imported objects never become bundled-tool actors. Visual
geometry supplies no physical TCP, collision, clearance, payload, calibration,
kinematics, motion, or controller authority.

## Runtime acceptance boundary

- Importing `ARrobots.HMI.robot_link_view` must not create application, Tk,
  filesystem, serial, firmware, controller, or hardware side effects.
- Construction resolves every required asset inside the explicit asset root and
  rejects missing files, root escape, non-files, and empty geometry before
  publishing a viewer.
- The descriptor table remains the single owner of filenames, hierarchy,
  colors, fixed transforms, joint indexes, and joint signs.
- Each child has one parent, the rendered assembly has one root, and
  `tool_mount` remains below `Link 6-1.STL` without duplicate actor binding.
- Bundled-tool selection mutates only the private bundled-tool child.
  Imported-scene actors remain under `PersistentCadScene` ownership.
- `launch_vtk_nonblocking()` owns initial camera and clipping setup. Live
  selection and clearing render without a camera reset and preserve the current
  view.
- Asset identity, matrix, geometry, and color checks are software-only. No
  passing render or matrix result establishes physical alignment, collision
  clearance, or live-arm compatibility.

## Licensing scope

[`LICENSE.txt`](../LICENSE.txt) is controlling. Section 2 applies to Annin
Robotics mechanical designs, robot parts, 3D models, CAD files, and print files.
Permitted use is limited to educational, research, and personal non-commercial
use. Section 2.2 prohibits redistribution, republishing, hosting, rebranding,
and commercial use without the required permission.

The contribution-fork scope above is a project-purpose boundary, not an
amendment or waiver of the license and not a grant of downstream redistribution
rights. Public application packages containing these assets remain blocked
until separate authority is obtained or the restricted assets are replaced or
removed from the package. No endorsement by Chris Annin or Annin Robotics is
implied. Licensing questions and permission requests should use the contact
information in `LICENSE.txt`.
