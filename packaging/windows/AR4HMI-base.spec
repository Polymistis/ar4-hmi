import json
import os
from pathlib import Path
from PyInstaller.utils.hooks import get_package_paths
ROOT = Path(SPECPATH).resolve().parents[1]
APP_NAMES = sorted("AR.png|defaults.json|information.txt|LICENSE.txt|VisBackdrop.png|xbox.png|play-icon.png|stop-icon.png|pp.gif|block.jpg|display setting.jpg|keystone jack.jpg|Link Base-1.STL|Link Base-2.STL|Link Base-3.STL|Link 1-1.STL|Link 1-2.STL|Link 2-1.STL|Link 2-2.STL|Link 2-3.STL|Link 3-1.STL|Link 3-2.STL|Link 4-1.STL|Link 4-2.STL|Link 4-3.STL|Link 5-1.STL|Link 5-2.STL|Link 6-1.STL|Link 6-2.STL".split("|"))
TTK_ROOT = Path(get_package_paths("ttkbootstrap")[1])
if not isinstance(manifest := json.loads((TTK_ROOT / "assets/elements/manifest.json").read_text(encoding="utf-8")), dict) or not isinstance(images := manifest.get("images"), dict) or not images or any(not isinstance(item, dict) or not isinstance(item.get("file"), str) or Path(item["file"]).name != item["file"] or not item["file"].lower().endswith(".png") for item in images.values()):
    raise ValueError("ttkbootstrap element manifest has an invalid images table")
TTK_NAMES = sorted(["ttkbootstrap/assets/elements/manifest.json", "ttkbootstrap/assets/icons/bootstrap.ttf", "ttkbootstrap/assets/icons/glyphmap.json", "ttkbootstrap/assets/icons/icon_metrics.json", "ttkbootstrap/assets/icons/LICENSE"] + [f"ttkbootstrap/assets/elements/{item['file']}" for item in images.values()])
TTK_DATAS = [(str(TTK_ROOT / Path(name).relative_to("ttkbootstrap")), str(Path(name).parent)) for name in TTK_NAMES]
HMI_EXCLUDES = sorted(["ARrobots.HMI.step_worker", "OCP", "adodbapi", "aiohappyeyeballs", "aiohttp", "aiosignal", "attr", "attrs", "cadquery", "cadquery_ocp", "cadquery_ocp_proxy", "casadi", "ezdxf", "frozenlist", "idna", "llvmlite", "more_itertools", "msgpack", "multidict", "multimethod", "nlopt", "numba", "propcache", "runtype", "scipy", "trame", "trame_client", "trame_common", "trame_components", "trame_server", "trame_vtk", "trame_vuetify", "typing_extensions", "wslink", "yaml", "yarl"])
if not (report_value := os.environ.get("AR4HMI_ANALYSIS_REPORT")) or not (REPORT := Path(report_value)).is_absolute() or not REPORT.parent.is_dir():
    raise ValueError("AR4HMI_ANALYSIS_REPORT must name an absolute file in an existing directory")
hmi = Analysis([str(ROOT / "AR4.py")], pathex=[str(ROOT)], binaries=[], datas=[(str(ROOT / name), ".") for name in APP_NAMES] + TTK_DATAS,
               hiddenimports=[], excludes=HMI_EXCLUDES.copy(), noarchive=False)
hmi.binaries = [row for row in hmi.binaries if not (str(row[0]).replace("\\", "/").lower().rsplit("/", 1)[-1].startswith("opencv_videoio_ffmpeg") and str(row[0]).lower().endswith(".dll"))]

def rows(toc):
    values = [{"name": str(name).replace("\\", "/"), "source": str(source).replace("\\", "/"), "type": str(kind)} for name, source, kind in toc]
    return sorted(values, key=lambda row: (row["name"], row["source"], row["type"]))

def graph(name, script, excludes, analysis):
    return {"name": name, "script": script, "hiddenImports": [], "excludes": excludes,
            "pure": rows(analysis.pure), "binaries": rows(analysis.binaries), "datas": rows(analysis.datas), "missing": rows(analysis.graph.make_missing_toc())}
report = {"schema": 1, "profile": "base", "outputs": ["AR4HMI.exe"],
          "graphs": [graph("hmi", "AR4.py", HMI_EXCLUDES, hmi)],
          "applicationData": APP_NAMES, "ttkData": TTK_NAMES}
REPORT.write_text(json.dumps(report, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8", newline="\n")
hmi_exe = EXE(PYZ(hmi.pure), hmi.scripts, [], exclude_binaries=True, name="AR4HMI", debug=False,
              bootloader_ignore_signals=False, strip=False, upx=False, console=False,
              icon=str(ROOT / "AR.ico"), contents_directory=".")
COLLECT(hmi_exe, hmi.binaries, hmi.datas, strip=False, upx=False, name="AR4HMI-base")
