import json
from pathlib import Path
import unittest
from unittest import mock

import vtk

try:
    from .bounded_temp import BoundedTemporaryDirectory
except ImportError:
    from bounded_temp import BoundedTemporaryDirectory
from ARrobots.HMI.cad_scene import CadSceneError, PersistentCadScene
_OBJECT_ID = "a" * 32
_TRIANGLE = (
    b"solid triangle\nfacet normal 0 -24 20\nouter loop\n"
    b"vertex 1 2 3\nvertex 5 2 3\nvertex 1 7 9\nendloop\nendfacet\nendsolid triangle\n"
)


def _row(**updates):
    fields = ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg")
    transform = dict.fromkeys(fields, 0.0)
    transform.update(updates.pop("transform", {}))
    row = {
        "id": _OBJECT_ID,
        "label": "Fixture",
        "asset": f"assets/{_OBJECT_ID}.stl",
        "parent": "world",
        "transform": transform,
    }
    row.update(updates)
    return row


def _manifest(*rows, **updates):
    document = {"schema": "ar4-cad-scene", "version": 1, "objects": list(rows)}
    document.update(updates)
    return json.dumps(document, separators=(",", ":")).encode("utf-8")
def _actors(renderer):
    actors = renderer.GetActors()
    actors.InitTraversal()
    return tuple(actors.GetNextActor() for _ in range(actors.GetNumberOfItems()))
def _world_matrix(actor, parent=None):
    matrix = actor.GetMatrix()
    if parent is not None:
        combined = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(parent, matrix, combined)
        matrix = combined
    return tuple(matrix.GetElement(row, column) for row in range(4) for column in range(4))


class CadSceneTests(unittest.TestCase):
    def assert_matrix_close(self, left, right):
        for actual, expected in zip(left, right):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_lifecycle_round_trip_preserves_pose_and_source(self):
        with BoundedTemporaryDirectory(prefix="ar4-cad-scene-") as directory:
            root = Path(directory)
            source = root / "source.stl"
            source.write_bytes(_TRIANGLE)
            workspace = root / "workspace"
            renderer = vtk.vtkRenderer()
            tool_mount = vtk.vtkAssembly()
            anchor_transform = vtk.vtkTransform()
            anchor_transform.Translate(100.0, 20.0, -5.0)
            anchor_transform.RotateZ(35.0)
            anchor = anchor_transform.GetMatrix()
            tool_mount.SetUserMatrix(anchor)
            scene = PersistentCadScene(workspace)
            scene.bind_vtk(renderer, anchors={"tool_mount": tool_mount})
            imported = scene.import_stl(source, label="Fixture")
            updated = scene.update_object(
                imported.object_id,
                label="Positioned",
                x_mm=10.0,
                y_mm=-5.0,
                z_mm=2.0,
                rx_deg=10.0,
                ry_deg=20.0,
                rz_deg=30.0,
            )
            self.assertEqual(
                (updated.x_mm, updated.y_mm, updated.z_mm, updated.rx_deg, updated.ry_deg, updated.rz_deg),
                (10.0, -5.0, 2.0, 10.0, 20.0, 30.0))
            actor = _actors(renderer)[0]
            before_attach = _world_matrix(actor)
            attached = scene.reparent(
                imported.object_id, "tool_mount",
                anchor_world_matrices={"tool_mount": anchor})
            self.assertEqual(attached.parent, "tool_mount")
            self.assertEqual(_actors(renderer), ())
            self.assertIs(tool_mount.GetParts().GetItemAsObject(0), actor)
            self.assert_matrix_close(before_attach, _world_matrix(actor, anchor))
            scene.unbind_vtk()
            scene = PersistentCadScene(workspace)
            scene.bind_vtk(renderer, anchors={"tool_mount": tool_mount})
            self.assertEqual(scene.objects[0].object_id, imported.object_id)
            self.assertEqual(scene.objects[0].parent, "tool_mount")
            self.assertEqual(_actors(renderer), ())
            actor = tool_mount.GetParts().GetItemAsObject(0)
            attached_world = _world_matrix(actor, anchor)
            self.assert_matrix_close(before_attach, attached_world)
            detached = scene.reparent(
                imported.object_id, "world",
                anchor_world_matrices={"tool_mount": anchor})
            self.assertEqual(detached.parent, "world")
            self.assertEqual(tool_mount.GetParts().GetNumberOfItems(), 0)
            self.assertEqual(_actors(renderer), (actor,))
            self.assert_matrix_close(attached_world, _world_matrix(actor))
            updated = scene.update_object(imported.object_id, label="Local", parent="tool_mount", x_mm=1.0, y_mm=2.0, z_mm=3.0, rx_deg=4.0, ry_deg=5.0, rz_deg=6.0)
            self.assertEqual((updated, _actors(renderer), tool_mount.GetParts().GetItemAsObject(0)), (type(updated)(imported.object_id, "Local", "tool_mount", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0), (), actor))
            self.assertEqual(PersistentCadScene(workspace).objects, (updated,))
            scene.delete_object(imported.object_id)
            self.assertEqual(scene.objects, ())
            self.assertEqual((_actors(renderer), tool_mount.GetParts().GetNumberOfItems()), ((), 0))
            self.assertFalse((workspace / "assets" / f"{imported.object_id}.stl").exists())
            self.assertEqual(source.read_bytes(), _TRIANGLE)

    def test_malformed_schema_and_file_boundaries_fail_closed(self):
        valid = _manifest(_row())
        cases = (
            ("duplicate-field", b'{"schema":"ar4-cad-scene","schema":"x","version":1,"objects":[]}', None),
            ("unknown-field", _manifest(unexpected=True), None),
            ("nonfinite", _manifest(_row(transform={"x_mm": float("nan")})), None),
            ("duplicate-id", _manifest(_row(), _row()), None),
            ("traversal", _manifest(_row(asset="../outside.stl")), None),
            ("missing-asset", valid, None),
            ("empty-asset", valid, b""),
            ("oversized-asset", valid, 64 * 1024 * 1024 + 1),
            ("oversized-manifest", b" " * (1024 * 1024 + 1), None),
            ("deep-nesting", b"[" * 100_000 + b"]" * 100_000, None),
            ("huge-integer", b"1" * 10_000, None),
        )
        with BoundedTemporaryDirectory(prefix="ar4-cad-invalid-") as directory:
            for name, payload, asset in cases:
                with self.subTest(name=name):
                    workspace = Path(directory) / name
                    assets = workspace / "assets"
                    assets.mkdir(parents=True)
                    asset_path = assets / f"{_OBJECT_ID}.stl"
                    if isinstance(asset, int):
                        with asset_path.open("wb") as stream:
                            stream.truncate(asset)
                    elif asset is not None:
                        asset_path.write_bytes(asset)
                    (workspace / "scene.json").write_bytes(payload)
                    with self.assertRaises(CadSceneError):
                        PersistentCadScene(workspace)

    def test_manifest_replacement_failure_preserves_bound_scene(self):
        with BoundedTemporaryDirectory(prefix="ar4-cad-failure-") as directory:
            root = Path(directory)
            source = root / "source.stl"
            source.write_bytes(_TRIANGLE)
            workspace = root / "workspace"
            scene = PersistentCadScene(workspace)
            imported = scene.import_stl(source, label="Fixture")
            renderer = vtk.vtkRenderer()
            scene.bind_vtk(renderer, anchors={"tool_mount": (tool_mount := vtk.vtkAssembly())})
            manifest_before = (workspace / "scene.json").read_bytes()
            objects_before = scene.objects
            actor_before = _actors(renderer)[0]
            matrix_before = _world_matrix(actor_before)
            assets_before = tuple(
                (path.name, path.read_bytes()) for path in sorted((workspace / "assets").iterdir())
            )
            with mock.patch("ARrobots.HMI.cad_scene.os.replace", side_effect=OSError):
                with self.assertRaises(CadSceneError):
                    scene.update_object(
                        imported.object_id,
                        label="Rejected",
                        x_mm=1.0,
                        y_mm=2.0,
                        z_mm=3.0,
                        rx_deg=4.0,
                        ry_deg=5.0,
                        rz_deg=6.0,
                        parent="tool_mount",
                    )
            self.assertEqual((workspace / "scene.json").read_bytes(), manifest_before)
            self.assertEqual(scene.objects, objects_before)
            self.assertEqual((_actors(renderer), tool_mount.GetParts().GetNumberOfItems()), ((actor_before,), 0))
            self.assert_matrix_close(_world_matrix(actor_before), matrix_before)
            self.assertEqual(
                tuple((path.name, path.read_bytes()) for path in sorted((workspace / "assets").iterdir())),
                assets_before,
            )

    def test_generated_stl_has_nonempty_offscreen_geometry_and_bounds(self):
        with BoundedTemporaryDirectory(prefix="ar4-cad-geometry-") as directory:
            root = Path(directory)
            source = root / "triangle.stl"
            source.write_bytes(_TRIANGLE)
            scene = PersistentCadScene(root / "workspace")
            scene.import_stl(source, label="Triangle")
            renderer = vtk.vtkRenderer()
            window = vtk.vtkRenderWindow()
            window.SetOffScreenRendering(1)
            window.AddRenderer(renderer)
            scene.bind_vtk(renderer, anchors={})
            actor = _actors(renderer)[0]
            geometry = actor.GetMapper().GetInput()
            self.assertEqual(geometry.GetNumberOfPoints(), 3)
            self.assertEqual(geometry.GetNumberOfCells(), 1)
            self.assertEqual(tuple(actor.GetBounds()), (1.0, 5.0, 2.0, 7.0, 3.0, 9.0))
            scene.unbind_vtk()

    def test_repeated_bind_and_unbind_never_duplicates_actors(self):
        with BoundedTemporaryDirectory(prefix="ar4-cad-rebind-") as directory:
            root = Path(directory)
            source = root / "source.stl"
            source.write_bytes(_TRIANGLE)
            scene = PersistentCadScene(root / "workspace")
            scene.import_stl(source, label="Fixture")
            renderer = vtk.vtkRenderer()
            actor = None
            for _ in range(3):
                scene.bind_vtk(renderer, anchors={})
                self.assertTrue(scene.is_bound_to(renderer))
                current = _actors(renderer)
                self.assertEqual(len(current), 1)
                actor = current[0] if actor is None else actor
                self.assertIs(current[0], actor)
                scene.bind_vtk(renderer, anchors={})
                self.assertEqual(_actors(renderer), (actor,))
                scene.unbind_vtk()
                self.assertEqual((scene.is_bound_to(renderer), scene.is_bound_to(None)), (False, False))
                self.assertEqual(_actors(renderer), ())
