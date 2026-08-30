"""Import-safe persistence and VTK ownership for user-supplied STL scenes."""

from collections.abc import Mapping
from dataclasses import dataclass
import json
import math
import os
import re
import secrets
import stat
import tempfile

import vtk


SCHEMA_NAME = "ar4-cad-scene"
SCHEMA_VERSION = 1
MAX_OBJECTS = 256
MAX_JSON_BYTES = 1024 * 1024
MAX_ASSET_BYTES = 64 * 1024 * 1024
MAX_TOTAL_ASSET_BYTES = 256 * 1024 * 1024
MAX_LABEL_BYTES = 255
_ID_PATTERN = re.compile(r"[0-9a-f]{32}\Z")
_PARENTS = frozenset(("world", "tool_mount"))
_TRANSFORM_FIELDS = ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg")


class CadSceneError(ValueError):
    """Scene input, persistence, or view binding is invalid."""


@dataclass(frozen=True)
class CadSceneObject:
    object_id: str
    label: str
    parent: str
    x_mm: float
    y_mm: float
    z_mm: float
    rx_deg: float
    ry_deg: float
    rz_deg: float


def _unique_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise CadSceneError(f"scene contains duplicate field {key!r}")
        value[key] = item
    return value


def _reject_constant(value):
    raise CadSceneError(f"scene contains invalid numeric constant {value}")


def _exact_keys(value, expected, description):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise CadSceneError(f"{description} fields are invalid")


def _finite(value, description):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CadSceneError(f"{description} must be numeric")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise CadSceneError(f"{description} is invalid") from exc
    if not math.isfinite(result):
        raise CadSceneError(f"{description} must be finite")
    return result


def _label(value):
    if not isinstance(value, str):
        raise CadSceneError("scene object label must be text")
    try:
        size = len(value.encode("utf-8"))
    except UnicodeEncodeError as exc:
        raise CadSceneError("scene object label is not valid UTF-8") from exc
    if size > MAX_LABEL_BYTES:
        raise CadSceneError("scene object label exceeds the byte limit")
    return value


def _object_id(value):
    if not isinstance(value, str) or _ID_PATTERN.fullmatch(value) is None:
        raise CadSceneError("scene object ID must be 32 lowercase hex characters")
    return value


def _asset_reference(object_id):
    return f"assets/{object_id}.stl"
def _scene_object(value):
    _exact_keys(value, ("id", "label", "asset", "parent", "transform"), "scene object")
    object_id = _object_id(value["id"])
    if value["asset"] != _asset_reference(object_id):
        raise CadSceneError("scene asset path does not match the object ID")
    if not isinstance(value["parent"], str) or value["parent"] not in _PARENTS:
        raise CadSceneError("scene object parent is unsupported")
    transform = value["transform"]
    _exact_keys(transform, _TRANSFORM_FIELDS, "scene transform")
    numbers = [_finite(transform[name], f"scene transform {name}") for name in _TRANSFORM_FIELDS]
    return CadSceneObject(object_id, _label(value["label"]), value["parent"], *numbers)


def _decode_manifest(payload):
    try:
        document = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_constant,
        )
    except CadSceneError:
        raise
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise CadSceneError("scene manifest is not valid UTF-8 JSON") from exc
    _exact_keys(document, ("schema", "version", "objects"), "scene manifest")
    if document["schema"] != SCHEMA_NAME:
        raise CadSceneError("scene schema is unsupported")
    version = document["version"]
    if type(version) is not int or version != SCHEMA_VERSION:
        raise CadSceneError("scene version is unsupported")
    rows = document["objects"]
    if not isinstance(rows, list) or len(rows) > MAX_OBJECTS:
        raise CadSceneError("scene object array is invalid or over limit")
    objects = tuple(_scene_object(row) for row in rows)
    identifiers = tuple(item.object_id for item in objects)
    if len(identifiers) != len(set(identifiers)):
        raise CadSceneError("scene contains duplicate object IDs or asset paths")
    return objects


def _encode_manifest(objects):
    rows = []
    for item in objects:
        transform = {name: getattr(item, name) for name in _TRANSFORM_FIELDS}
        rows.append({
            "id": item.object_id,
            "label": item.label,
            "asset": _asset_reference(item.object_id),
            "parent": item.parent,
            "transform": transform,
        })
    try:
        payload = json.dumps(
            {"schema": SCHEMA_NAME, "version": SCHEMA_VERSION, "objects": rows},
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8") + b"\n"
    except (TypeError, ValueError, UnicodeEncodeError) as exc:
        raise CadSceneError("scene manifest could not be encoded") from exc
    if len(payload) > MAX_JSON_BYTES:
        raise CadSceneError("scene manifest exceeds the byte limit")
    return payload


def _path(value, description):
    if isinstance(value, bytes):
        raise CadSceneError(f"{description} must be a text path")
    try:
        result = os.fspath(value)
    except TypeError as exc:
        raise CadSceneError(f"{description} is invalid") from exc
    if not isinstance(result, str) or not result or "\x00" in result:
        raise CadSceneError(f"{description} is invalid")
    return result


def _state(path, limit, description):
    try:
        value = os.stat(path, follow_symlinks=False)
    except (OSError, ValueError) as exc:
        raise CadSceneError(f"{description} could not be inspected") from exc
    if not stat.S_ISREG(value.st_mode):
        raise CadSceneError(f"{description} must be a nonsymlink regular file")
    if value.st_size <= 0 or value.st_size > limit:
        raise CadSceneError(f"{description} is empty or exceeds the byte limit")
    return value


def _fingerprint(value):
    return value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns
def _read_regular(path, limit, description):
    expected = _state(path, limit, description)
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = None
    try:
        descriptor = os.open(path, flags)
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode) or _fingerprint(opened) != _fingerprint(expected):
            raise CadSceneError(f"{description} changed before being read")
        with os.fdopen(descriptor, "rb", closefd=True) as source:
            descriptor = None
            payload = source.read(limit + 1)
            final = os.fstat(source.fileno())
    except CadSceneError:
        raise
    except (OSError, ValueError) as exc:
        raise CadSceneError(f"{description} could not be read") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
    if len(payload) != opened.st_size or _fingerprint(final) != _fingerprint(opened):
        raise CadSceneError(f"{description} changed while being read")
    return payload, _fingerprint(final)


def _write_temporary(directory, prefix, payload, *, target=None):
    descriptor = None
    path = None
    try:
        if target is None:
            descriptor, path = tempfile.mkstemp(prefix=prefix, suffix=".tmp", dir=directory)
        else:
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0)
            descriptor = os.open(target, flags, 0o600)
            path = target
        with os.fdopen(descriptor, "wb", closefd=True) as destination:
            descriptor = None
            if destination.write(payload) != len(payload):
                raise OSError("temporary scene write was incomplete")
            destination.flush()
            os.fsync(destination.fileno())
        return path
    except OSError as exc:
        if path is not None:
            try:
                os.unlink(path)
            except OSError:
                pass
        raise CadSceneError("temporary scene file could not be written") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _directory(path, description):
    value = os.stat(path, follow_symlinks=False)
    reparse = getattr(value, "st_file_attributes", 0) & 0x400
    if (not stat.S_ISDIR(value.st_mode) or reparse
            or getattr(os.path, "isjunction", lambda unused: False)(path)):
        raise CadSceneError(f"{description} must be a non-reparse directory")


def _apply_pose(actor, item):
    actor.SetPosition(item.x_mm, item.y_mm, item.z_mm)
    actor.SetOrientation(item.rx_deg, item.ry_deg, item.rz_deg)


def _actor(payload, item, directory):
    temporary = _write_temporary(directory, ".cad-decode-", payload)
    try:
        reader = vtk.vtkSTLReader()
        reader.SetFileName(temporary)
        reader.Update()
        geometry = reader.GetOutput()
        bounds = geometry.GetBounds()
        valid_bounds = bounds is not None and len(bounds) == 6
        valid_bounds = valid_bounds and all(math.isfinite(number) for number in bounds)
        valid_bounds = valid_bounds and all(
            bounds[index] <= bounds[index + 1] for index in (0, 2, 4)
        )
        if geometry.GetNumberOfPoints() <= 0 or geometry.GetNumberOfCells() <= 0:
            raise CadSceneError("scene STL asset contains no geometry")
        if not valid_bounds:
            raise CadSceneError("scene STL asset bounds are invalid")
        owned = vtk.vtkPolyData()
        owned.DeepCopy(geometry)
    finally:
        try:
            os.unlink(temporary)
        except OSError as exc:
            raise CadSceneError("scene decode temporary file could not be removed") from exc
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(owned)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.254, 0.41, 0.882)
    _apply_pose(actor, item)
    return actor


def _pose_matrix(item):
    transform = vtk.vtkTransform()
    transform.Translate(item.x_mm, item.y_mm, item.z_mm)
    transform.RotateZ(item.rz_deg)
    transform.RotateX(item.rx_deg)
    transform.RotateY(item.ry_deg)
    result = vtk.vtkMatrix4x4()
    result.DeepCopy(transform.GetMatrix())
    return result


def _anchor_matrix(value, description):
    if not isinstance(value, vtk.vtkMatrix4x4):
        raise CadSceneError(f"{description} must be vtkMatrix4x4")
    result = vtk.vtkMatrix4x4()
    result.DeepCopy(value)
    values = [result.GetElement(row, column) for row in range(4) for column in range(4)]
    if not all(math.isfinite(number) for number in values):
        raise CadSceneError(f"{description} must be finite")
    if any(abs(result.GetElement(3, column)) > 1e-6 for column in range(3)):
        raise CadSceneError(f"{description} must be affine")
    if abs(result.GetElement(3, 3) - 1.0) > 1e-6 or abs(result.Determinant()) < 1e-12:
        raise CadSceneError(f"{description} must be invertible")
    return result


def _bind_objects(renderer, anchors, objects, actors, add):
    for item in objects:
        actor = actors[item.object_id][0]
        if item.parent == "world":
            action = renderer.AddActor if add else renderer.RemoveActor
        else:
            action = anchors[item.parent].AddPart if add else anchors[item.parent].RemovePart
        action(actor)


class PersistentCadScene:
    def __init__(self, workspace_root):
        raw_root = os.path.abspath(os.path.expanduser(_path(workspace_root, "CAD workspace")))
        try:
            if os.path.lexists(raw_root):
                _directory(raw_root, "CAD workspace")
            else:
                os.makedirs(raw_root)
                _directory(raw_root, "CAD workspace")
            self._root = os.path.realpath(raw_root)
            self._assets = os.path.join(self._root, "assets")
            if os.path.lexists(self._assets):
                _directory(self._assets, "CAD assets")
            else:
                os.mkdir(self._assets)
                _directory(self._assets, "CAD assets")
        except CadSceneError:
            raise
        except OSError as exc:
            raise CadSceneError("CAD workspace could not be initialized") from exc
        self._manifest = os.path.join(self._root, "scene.json")
        self._renderer = None
        self._anchors = {}
        self._objects = self._load_objects() if os.path.lexists(self._manifest) else ()
        self._actors = self._load_actors(self._objects)

    @property
    def objects(self):
        return self._objects

    def is_bound_to(self, renderer):
        return self._renderer is not None and renderer is self._renderer

    def _asset_path(self, object_id):
        return os.path.join(self._assets, f"{object_id}.stl")

    def _load_objects(self):
        payload, _ = _read_regular(self._manifest, MAX_JSON_BYTES, "scene manifest")
        return _decode_manifest(payload)

    def _load_actors(self, objects):
        fingerprints = {}
        total = 0
        for item in objects:
            state = _state(self._asset_path(item.object_id), MAX_ASSET_BYTES, "scene STL asset")
            fingerprint = _fingerprint(state)
            total += fingerprint[2]
            if total > MAX_TOTAL_ASSET_BYTES:
                raise CadSceneError("scene assets exceed the total byte limit")
            fingerprints[item.object_id] = fingerprint
        actors = {}
        for item in objects:
            payload, fingerprint = _read_regular(
                self._asset_path(item.object_id), MAX_ASSET_BYTES, "scene STL asset"
            )
            if fingerprint != fingerprints[item.object_id]:
                raise CadSceneError("scene STL asset changed before decoding")
            actors[item.object_id] = (_actor(payload, item, self._root), fingerprint)
        return actors

    def _validate_inventory(self, objects, actors):
        total = 0
        for item in objects:
            if item.object_id not in actors:
                raise CadSceneError("scene actor ownership is incomplete")
            expected = actors[item.object_id][1]
            actual = _state(self._asset_path(item.object_id), MAX_ASSET_BYTES, "scene STL asset")
            if _fingerprint(actual) != expected:
                raise CadSceneError("scene STL asset changed after validation")
            total += actual.st_size
            if total > MAX_TOTAL_ASSET_BYTES:
                raise CadSceneError("scene assets exceed the total byte limit")

    def _publish(self, objects, actors):
        self._validate_inventory(objects, actors)
        binding = None
        if self._renderer is not None:
            binding = self._renderer, dict(self._anchors)
            if any(item.parent == "tool_mount" for item in objects) and "tool_mount" not in binding[1]:
                raise CadSceneError("bound scene lacks the required tool_mount anchor")
        temporary = _write_temporary(self._root, ".scene.json.", _encode_manifest(objects))
        try:
            os.replace(temporary, self._manifest)
        except (CadSceneError, OSError) as exc:
            try:
                if os.path.lexists(temporary):
                    os.unlink(temporary)
            except OSError as cleanup_error:
                raise CadSceneError("manifest commit and temporary cleanup failed") from exc
            if isinstance(exc, CadSceneError):
                raise
            raise CadSceneError("scene manifest could not be committed") from exc
        previous_objects, previous_actors = self._objects, self._actors
        self._objects, self._actors = objects, actors
        if binding is not None:
            _bind_objects(*binding, previous_objects, previous_actors, False)
        for item in objects:
            _apply_pose(actors[item.object_id][0], item)
        if binding is not None:
            _bind_objects(*binding, objects, actors, True)

    def _find(self, object_id):
        object_id = _object_id(object_id)
        for index, item in enumerate(self._objects):
            if item.object_id == object_id:
                return index, item
        raise CadSceneError("scene object ID was not found")

    def import_stl(self, source_path, *, label):
        if len(self._objects) >= MAX_OBJECTS:
            raise CadSceneError("scene object count exceeds the limit")
        label = _label(label)
        existing = {item.object_id for item in self._objects}
        object_id = next((value for value in (secrets.token_hex(16) for _ in range(16))
                          if value not in existing and not os.path.lexists(self._asset_path(value))), None)
        if object_id is None:
            raise CadSceneError("unable to allocate a unique scene object ID")
        item = CadSceneObject(object_id, label, "world", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        payload, _ = _read_regular(
            _path(source_path, "STL source"), MAX_ASSET_BYTES, "STL source"
        )
        if sum(value[1][2] for value in self._actors.values()) + len(payload) > MAX_TOTAL_ASSET_BYTES:
            raise CadSceneError("scene assets exceed the total byte limit")
        final = self._asset_path(object_id)
        installed = False
        try:
            _write_temporary(self._assets, ".cad-asset-", payload, target=final)
            installed = True
            installed_payload, fingerprint = _read_regular(final, MAX_ASSET_BYTES, "scene STL asset")
            actor = _actor(installed_payload, item, self._root)
            actors = dict(self._actors)
            actors[object_id] = (actor, fingerprint)
            objects = self._objects + (item,)
            self._publish(objects, actors)
        except (CadSceneError, OSError) as operation_error:
            cleanup_error = None
            committed = any(value.object_id == object_id for value in self._objects)
            if installed and not committed and os.path.lexists(final):
                try:
                    os.unlink(final)
                except OSError as exc:
                    cleanup_error = exc
            if cleanup_error is not None:
                message = "STL import failed and staged cleanup also failed"
                raise CadSceneError(message) from operation_error
            if isinstance(operation_error, CadSceneError):
                raise
            raise CadSceneError("STL import failed") from operation_error
        return item

    def update_object(self, object_id, *, label, x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg,
                      parent=None):
        index, current = self._find(object_id)
        if parent is None:
            parent = current.parent
        elif not isinstance(parent, str) or parent not in _PARENTS:
            raise CadSceneError("scene object parent is unsupported")
        values = (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
        numbers = [_finite(value, f"scene transform {name}")
                   for name, value in zip(_TRANSFORM_FIELDS, values)]
        item = CadSceneObject(current.object_id, _label(label), parent, *numbers)
        objects = self._objects[:index] + (item,) + self._objects[index + 1:]
        actors = dict(self._actors)
        self._publish(objects, actors)
        return item

    def delete_object(self, object_id):
        index, current = self._find(object_id)
        objects = self._objects[:index] + self._objects[index + 1:]
        actors = dict(self._actors)
        actors.pop(current.object_id)
        self._publish(objects, actors)
        try:
            os.unlink(self._asset_path(current.object_id))
        except OSError as exc:
            raise CadSceneError("scene deletion committed but asset cleanup failed") from exc

    def reparent(self, object_id, parent, *, anchor_world_matrices):
        index, current = self._find(object_id)
        if not isinstance(parent, str) or parent not in _PARENTS:
            raise CadSceneError("scene object parent is unsupported")
        if parent == current.parent:
            return current
        if not isinstance(anchor_world_matrices, Mapping):
            raise CadSceneError("anchor world matrices must be a mapping")
        identity = vtk.vtkMatrix4x4()
        identity.Identity()

        def parent_matrix(name):
            if name == "world":
                return identity
            if name not in anchor_world_matrices:
                raise CadSceneError(f"world matrix for anchor {name!r} is missing")
            return _anchor_matrix(anchor_world_matrices[name], f"anchor {name!r} matrix")

        world = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(parent_matrix(current.parent), _pose_matrix(current), world)
        inverse = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Invert(parent_matrix(parent), inverse)
        local = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(inverse, world, local)
        transform = vtk.vtkTransform()
        transform.SetMatrix(local)
        values = (*transform.GetPosition(), *transform.GetOrientation())
        values = [_finite(value, "reparented scene transform") for value in values]
        item = CadSceneObject(current.object_id, current.label, parent, *values)
        rebuilt = _pose_matrix(item)
        if any(abs(rebuilt.GetElement(row, column) - local.GetElement(row, column)) > 1e-6
               for row in range(4) for column in range(4)):
            raise CadSceneError("reparented pose cannot be represented as XYZ/RXYZ")
        objects = self._objects[:index] + (item,) + self._objects[index + 1:]
        actors = dict(self._actors)
        self._publish(objects, actors)
        return item

    def bind_vtk(self, renderer, *, anchors):
        if not isinstance(renderer, vtk.vtkRenderer) or not isinstance(anchors, Mapping):
            raise CadSceneError("scene renderer or anchor mapping is invalid")
        anchors = dict(anchors)
        if set(anchors) - {"tool_mount"}:
            raise CadSceneError("scene anchor mapping contains unsupported names")
        if any(not isinstance(value, vtk.vtkAssembly) for value in anchors.values()):
            raise CadSceneError("scene anchors must be vtkAssembly values")
        if any(item.parent == "tool_mount" for item in self._objects) and "tool_mount" not in anchors:
            raise CadSceneError("tool_mount anchor is required by the scene")
        self.unbind_vtk()
        _bind_objects(renderer, anchors, self._objects, self._actors, True)
        self._renderer = renderer
        self._anchors = anchors

    def unbind_vtk(self):
        if self._renderer is None:
            return
        renderer = self._renderer
        anchors = self._anchors
        _bind_objects(renderer, anchors, self._objects, self._actors, False)
        self._renderer = None
        self._anchors = {}
