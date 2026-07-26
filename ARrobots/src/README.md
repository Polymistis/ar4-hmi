# Native kinematics build

Build the extension with the same CPython minor version and architecture used to run the HMI. ABI-tagged filenames are preserved so an incompatible CPython minor version cannot load the module. Install the isolated build dependency first:

```text
python -m pip install -r ARrobots/src/requirements-build.txt
```

Windows x64 with Visual Studio Build Tools:

```text
powershell -ExecutionPolicy Bypass -File ARrobots/src/build_kinematics.ps1 -Python C:\path\to\python.exe -Install
```

Linux requires a current source build because bundled legacy extension files do not expose the atomic configuration and wrist-aware solver contract required by the HMI:

```text
PYTHON=python3 bash ARrobots/src/build_kinematics.sh
```

The Linux artifact remains in the selected build directory. Preserve the ABI-tagged filename when copying a Linux module into `ARrobots`; a loaded module must expose the configured atomic setter `set_robot_configuration` and wrist-aware solver `SolveInverseKinematicsConfigured` before motion admission succeeds.

Run the native contract after installing or copying a module:

```text
python -m unittest -v tests.test_native_kinematics
```

The tests do not import `AR4.py` and do not communicate with a controller.

