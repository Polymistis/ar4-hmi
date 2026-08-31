"""Import-safe ownership for the built-in robot and selectable-tool VTK hierarchy."""

from dataclasses import dataclass
import math
from pathlib import Path

import vtk


@dataclass(frozen=True)
class _Part:
    filename: str
    parent: str | None
    color_role: str
    fixed_operations: tuple = ()
    joint_index: int | None = None
    joint_sign: float = 1.0


@dataclass(frozen=True)
class _BundledTool:
    filename: str
    position: tuple
    orientation: tuple


_PARTS = (
    _Part("Link Base-1.STL", None, "body"),
    _Part("Link Base-2.STL", "Link Base-1.STL", "main"),
    _Part("Link Base-3.STL", "Link Base-2.STL", "accent"),
    _Part("Link 1-1.STL", "Link Base-3.STL", "body", (
        ("RotateX", (180.0,)), ("RotateZ", (-90.0,)),
        ("Translate", (0.0, 0.0, -92.0)),
    ), 0, -1.0),
    _Part("Link 1-2.STL", "Link 1-1.STL", "accent"),
    _Part("Link 2-1.STL", "Link 1-2.STL", "body", (
        ("RotateZ", (-90.0,)), ("RotateX", (270.0,)),
        ("Translate", (-64.15, 77.78, 8.87)),
    ), 1, 1.0),
    _Part("Link 2-2.STL", "Link 2-1.STL", "main"),
    _Part("Link 2-3.STL", "Link 2-2.STL", "accent"),
    _Part("Link 3-1.STL", "Link 2-3.STL", "body", (
        ("RotateZ", (180.0,)), ("RotateX", (180.0,)),
        ("Translate", (0.0, 305.0, -27.84)),
    ), 2, -1.0),
    _Part("Link 3-2.STL", "Link 3-1.STL", "accent"),
    _Part("Link 4-1.STL", "Link 3-2.STL", "body", (
        ("RotateY", (90.0,)), ("RotateX", (180.0,)),
        ("Translate", (-36.7, 0.0, -75.94)),
    ), 3, -1.0),
    _Part("Link 4-2.STL", "Link 4-1.STL", "main"),
    _Part("Link 4-3.STL", "Link 4-2.STL", "accent"),
    _Part("Link 5-1.STL", "Link 4-3.STL", "body", (
        ("RotateZ", (180.0,)), ("RotateY", (90.0,)),
        ("Translate", (147.0, 0.0, 44.88)),
    ), 4, -1.0),
    _Part("Link 5-2.STL", "Link 5-1.STL", "accent"),
    _Part("Link 6-1.STL", "Link 5-2.STL", "body", (
        ("RotateY", (90.0,)), ("Translate", (43.3, 0.0, 25.0)),
    ), 5, 1.0),
    _Part("Link 6-2.STL", "Link 6-1.STL", "accent"),
)
_ROLE_COLORS = {"body": "Silver", "main": "Orange", "accent": "DimGray"}
_TOOL_PARENT = "Link 6-1.STL"
_BUNDLED_TOOLS = {
    "Servo Gripper": _BundledTool(
        "Servo Gripper.STL", (0.0, 0.0, 16.5), (0.0, 0.0, 90.0)
    ),
    "Welding Torch": _BundledTool(
        "Welding Torch.STL", (0.0, 0.0, 17.0), (0.0, 0.0, 90.0)
    ),
}
_BUNDLED_TOOL_OPTIONS = tuple(_BUNDLED_TOOLS)
_ASSET_FILENAMES = (
    tuple(part.filename for part in _PARTS)
    + tuple(tool.filename for tool in _BUNDLED_TOOLS.values())
)


def _resolve_assets(asset_root):
    if asset_root == "":
        raise ValueError("robot visualization asset root is invalid")
    try:
        root = Path(asset_root).expanduser().resolve(strict=True)
    except FileNotFoundError:
        raise
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        raise ValueError("robot visualization asset root is invalid") from exc
    if not root.is_dir():
        raise ValueError("robot visualization asset root must be a directory")

    assets = {}
    for filename in _ASSET_FILENAMES:
        try:
            path = (root / filename).resolve(strict=True)
        except FileNotFoundError as exc:
            raise FileNotFoundError(
                f"robot visualization asset is missing: {filename}"
            ) from exc
        except (OSError, RuntimeError) as exc:
            raise ValueError(
                f"robot visualization asset path is invalid: {filename}"
            ) from exc
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise ValueError(
                f"robot visualization asset escapes the asset root: {filename}"
            ) from exc
        if not path.is_file():
            raise ValueError(f"robot visualization asset must be a file: {filename}")
        assets[filename] = path
    return assets


def _apply_operations(transform, operations):
    for method_name, arguments in operations:
        getattr(transform, method_name)(*arguments)


def _validate_bundled_tool_name(name):
    if not isinstance(name, str) or name not in _BUNDLED_TOOLS:
        choices = ", ".join(_BUNDLED_TOOL_OPTIONS)
        raise ValueError(f"bundled tool selection must be one of: {choices}")
    return name


def _build_bundled_tool_actor(path, tool):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    geometry = reader.GetOutput()
    if (geometry is None or geometry.GetNumberOfPoints() <= 0
            or geometry.GetNumberOfCells() <= 0):
        raise ValueError(
            f"robot visualization asset contains no geometry: {tool.filename}"
        )

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    material = actor.GetProperty()
    material.SetColor(0.45, 0.45, 0.45)
    material.SetAmbient(0.25)
    material.SetDiffuse(0.75)
    material.SetSpecular(0.08)
    material.SetSpecularPower(8.0)
    actor.SetPosition(*tool.position)
    actor.SetOrientation(*tool.orientation)
    return actor


def _finite_angles(values):
    if isinstance(values, (str, bytes)):
        raise ValueError("robot-link joint angles must be a numeric sequence")
    try:
        raw_angles = tuple(values)
    except TypeError as exc:
        raise ValueError("robot-link joint angles must be a numeric sequence") from exc
    if len(raw_angles) != 6 or any(isinstance(value, bool) for value in raw_angles):
        raise ValueError("robot-link joint angles must contain six finite values")
    try:
        angles = tuple(float(value) for value in raw_angles)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("robot-link joint angles must be numeric") from exc
    if not all(math.isfinite(value) for value in angles):
        raise ValueError("robot-link joint angles must contain six finite values")
    return angles


class RobotLinkView:
    """Own the built-in robot geometry, hierarchy, and joint transforms."""

    BUNDLED_TOOL_OPTIONS = _BUNDLED_TOOL_OPTIONS
    DEFAULT_BUNDLED_TOOL = "Servo Gripper"

    def __init__(self, asset_root, selected_bundled_tool=DEFAULT_BUNDLED_TOOL):
        if selected_bundled_tool is not None:
            selected_bundled_tool = _validate_bundled_tool_name(selected_bundled_tool)
        assets = _resolve_assets(asset_root)
        named_colors = vtk.vtkNamedColors()
        actors = {}
        assemblies = {}
        base_transforms = {}
        joint_transforms = {}
        composite_transforms = {}

        for part in _PARTS:
            reader = vtk.vtkSTLReader()
            reader.SetFileName(str(assets[part.filename]))
            reader.Update()
            geometry = reader.GetOutput()
            if (geometry is None or geometry.GetNumberOfPoints() <= 0
                    or geometry.GetNumberOfCells() <= 0):
                raise ValueError(f"robot-link asset contains no geometry: {part.filename}")

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(
                named_colors.GetColor3d(_ROLE_COLORS[part.color_role])
            )

            base_transform = vtk.vtkTransform()
            joint_transform = vtk.vtkTransform()
            composite_transform = vtk.vtkTransform()
            _apply_operations(base_transform, part.fixed_operations)
            composite_transform.Concatenate(base_transform)
            composite_transform.Concatenate(joint_transform)

            assembly = vtk.vtkAssembly()
            assembly.AddPart(actor)
            assembly.SetUserTransform(composite_transform)
            actors[part.filename] = actor
            assemblies[part.filename] = assembly
            base_transforms[part.filename] = base_transform
            joint_transforms[part.filename] = joint_transform
            composite_transforms[part.filename] = composite_transform

        for part in _PARTS:
            if part.parent is not None:
                assemblies[part.parent].AddPart(assemblies[part.filename])
        tool_mount = vtk.vtkAssembly()
        assemblies[_TOOL_PARENT].AddPart(tool_mount)

        bundled_tool_actors = {
            name: _build_bundled_tool_actor(assets[tool.filename], tool)
            for name, tool in _BUNDLED_TOOLS.items()
        }
        bundled_tool_child = vtk.vtkAssembly()
        if selected_bundled_tool is not None:
            bundled_tool_child.AddPart(bundled_tool_actors[selected_bundled_tool])
        tool_mount.AddPart(bundled_tool_child)

        self._named_colors = named_colors
        self._actors = actors
        self._base_transforms = base_transforms
        self._joint_transforms = joint_transforms
        self._composite_transforms = composite_transforms
        self._root = assemblies[_PARTS[0].filename]
        self._tool_mount = tool_mount
        self._bundled_tool_child = bundled_tool_child
        self._bundled_tool_actors = bundled_tool_actors
        self._selected_bundled_tool = selected_bundled_tool

    @property
    def root(self):
        return self._root

    @property
    def tool_mount(self):
        return self._tool_mount

    @property
    def selected_bundled_tool(self):
        return self._selected_bundled_tool

    def select_bundled_tool(self, name):
        name = _validate_bundled_tool_name(name)
        next_actor = self._bundled_tool_actors[name]
        if name == self._selected_bundled_tool:
            return

        if self._selected_bundled_tool is not None:
            current_actor = self._bundled_tool_actors[self._selected_bundled_tool]
            self._bundled_tool_child.RemovePart(current_actor)
        self._bundled_tool_child.AddPart(next_actor)
        self._selected_bundled_tool = name

    def clear_bundled_tool(self):
        if self._selected_bundled_tool is None:
            return
        current_actor = self._bundled_tool_actors[self._selected_bundled_tool]
        self._bundled_tool_child.RemovePart(current_actor)
        self._selected_bundled_tool = None

    def update_joint_angles(self, angles):
        angles = _finite_angles(angles)
        for part in _PARTS:
            if part.joint_index is None:
                continue
            joint_transform = self._joint_transforms[part.filename]
            joint_transform.Identity()
            joint_transform.RotateZ(part.joint_sign * angles[part.joint_index])
            composite_transform = self._composite_transforms[part.filename]
            composite_transform.Identity()
            composite_transform.Concatenate(self._base_transforms[part.filename])
            composite_transform.Concatenate(joint_transform)

    def update_main_color(self, color_name):
        color = self._named_colors.GetColor3d(color_name)
        for part in _PARTS:
            if part.color_role == "main":
                self._actors[part.filename].GetProperty().SetColor(color)

    def tool_mount_world_matrix(self):
        world = vtk.vtkMatrix4x4()
        world.Identity()
        for part in _PARTS:
            product = vtk.vtkMatrix4x4()
            vtk.vtkMatrix4x4.Multiply4x4(
                world, self._composite_transforms[part.filename].GetMatrix(), product
            )
            world = product
            if part.filename == _TOOL_PARENT:
                break
        return world
