# Persistent CAD scene schema v1

`PersistentCadScene` owns one local visualization scene. The committed manifest is
`cad-workspace/scene.json`; copied meshes are stored below
`cad-workspace/assets/`. The workspace is local runtime state, remains untracked,
and may be replaced with an injected root for tests or another local profile.

## Exact JSON shape

```json
{
  "schema": "ar4-cad-scene",
  "version": 1,
  "objects": [
    {
      "id": "0123456789abcdef0123456789abcdef",
      "label": "Fixture",
      "asset": "assets/0123456789abcdef0123456789abcdef.stl",
      "parent": "world",
      "transform": {
        "x_mm": 0.0,
        "y_mm": 0.0,
        "z_mm": 0.0,
        "rx_deg": 0.0,
        "ry_deg": 0.0,
        "rz_deg": 0.0
      }
    }
  ]
}
```

All shown members are required. Top-level, object, and transform objects reject
unknown or duplicate members. `objects` is an array, `schema` and `version` must
equal the shown literals, and each object ID must be unique.

An ID is exactly 32 lowercase hexadecimal characters. An ID is generated once,
never derived from a label or filename, and never reassigned to another object.
Labels are display text, not identity, and contain at most 255 UTF-8 bytes.

The asset value is exactly `assets/<id>.stl`, where `<id>` equals the containing
object ID. Forward slashes are mandatory. Absolute paths, backslashes, dot
segments, alternate extensions, aliases, and traversal are invalid. Imported
sources are copied into the workspace and are never renamed, edited, or deleted.
Source paths and source filenames are not persistent identity.

## Transform and parent contract

Translation values are millimetres and rotation values are degrees. Every value
must be a finite JSON number; booleans are not numbers. Scale, shear, origin,
matrix, and implicit unit conversion are outside schema v1.

The transform maps object-local coordinates into the selected parent frame. VTK
receives the values through `SetPosition(x_mm, y_mm, z_mm)` and
`SetOrientation(rx_deg, ry_deg, rz_deg)`. VTK specifies the angles as X, Y, and Z
rotations and performs RotateZ, then RotateX, then RotateY. The scene uses VTK's
right-handed Cartesian coordinates; no robot-controller coordinate convention is
implied.

`world` is the fixed visualization-world root and has an identity world matrix.
`tool_mount` is the stable visual flange anchor supplied by the robot viewer and
tracks the rendered sixth-link mount pose. `tool_mount` is not a physical tool
center point. No other parent value is valid in schema v1.

For column-vector matrices, `object_world = parent_world @ object_local`.
Reparenting computes `new_local = inverse(new_parent_world) @ object_world` so the
displayed world pose does not move. A missing anchor, non-invertible matrix, or
non-finite result rejects the complete operation without changing durable or
published state.

An explicit local-pose update may select a parent and parent-local transform in
the same transaction. An omitted parent retains the current parent. Both the
parent and complete transform validate before one manifest replacement and
actor rebind. This operation intentionally differs from reparenting: the given
local pose is authoritative for visualization, so changing the parent may move
the displayed world pose.

## Validation limits

- `scene.json` must be UTF-8 JSON no larger than 1 MiB in encoded form.
- A manifest may contain at most 256 objects.
- Every referenced asset must exist as a non-symlink regular file below the
  injected workspace root.
- Each asset is at most 64 MiB; all referenced assets total at most 256 MiB.
- VTK must read non-empty geometry from every asset before a candidate scene can
  be committed or published.

No read follows a symlink or escapes the workspace. A malformed manifest,
missing asset, invalid transform, duplicate identity, over-limit resource, or VTK
read failure rejects the whole candidate. Loading never silently substitutes an
empty scene or a partial object set.

## Transaction and orphan contract

The validated `scene.json` replacement is the sole durable commit point. A
mutation builds and validates a complete candidate manifest and candidate VTK
objects before changing committed or published state. Manifest writes use a
same-directory temporary file, flush the complete UTF-8 payload, and atomically
replace `scene.json`. The previous committed manifest and published snapshot
remain authoritative until replacement succeeds.

Import exclusively creates `assets/<id>.stl`, writes and flushes the bounded
source copy, then rereads and validates the installed regular file and geometry
before replacing the manifest. Interruption before manifest replacement can
leave only an unreachable partial or complete asset orphan; no committed
manifest references a missing new asset.

Deletion replaces the manifest before removing the now-unreferenced asset.
Interruption or removal failure can likewise leave an unreachable orphan.
Explicit local-pose update, world-pose-preserving reparent, attach, and detach
replace only a complete manifest after validation. Failed operations retain the
prior committed manifest and published snapshot.

Startup performs no orphan deletion. An exact-name `assets/<id>.stl` blob is
scene-reachable only when the validated committed manifest references the same
ID. Any blob left unreachable by interruption or cleanup failure is never
loaded, listed, bound, or otherwise scene-visible and remains local and
untracked below the ignored workspace. Reclamation requires a separately
specified, explicitly invoked maintenance operation; scene loading and ordinary
mutation never infer deletion authority from an unreferenced filename.

Renderer close removes transient actor bindings only. Rebinding derives exactly
one actor per committed object from the same scene owner; persistence does not
depend on viewer lifetime, and reopen cannot create competing actor ownership.

## Safety and asset authority

The scene is visualization only. Scene geometry and transforms provide no
collision, reachability, clearance, payload, kinematics, motion, calibration,
controller, output, or physical TCP authority. No scene operation may issue a
serial command, controller write, motion request, or output activation.

The delivered software license section 2.2 prohibits redistribution of design
files. Delivered or design assets are therefore excluded from the repository,
packages, defaults, examples, and migrations unless separate redistribution
authority is documented. Only operator-selected, locally authorized STL files
may be copied into the ignored local workspace; workspace storage does not grant
redistribution authority.
