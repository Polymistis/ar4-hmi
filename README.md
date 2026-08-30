# AR4 Control Software
**Host source 7.0 — tracked Teensy derivative 6.7.1-ar4hmi.38**

![AR4 Logo](AR.png)

The **Annin Robotics AR4-MK3 Control Software** is the official open-source, non-commercial desktop application for controlling the AR4 six-axis robotic arm.  
It provides real-time joint and Cartesian control, calibration utilities, teach-mode programming, and integration with the AR4-MK3 firmware running on a Teensy 4.1 controller.

---

## 🧭 Project Overview
This repository contains:

- **AR4.py and ARrobots/** – Python-based GUI and modules for robot motion, visualization, and communications.
- **ArduinoSketches/** – Arduino/Teensy firmware and motion-control code.
- **LICENSE.txt** – Annin Robotics Open Source Non-Commercial License.  
- **README.md** – Project information and usage guide.

### Features
- 6-axis robot control interface (Teensy 4.1 based)  
- Live joint & Cartesian jogging  
- Position teach, record, and playback  
- VTK 3D robot visualization  
- Integrated calibration tools  

---

## 🧩 System Requirements
| Component | Recommended |
|------------|-------------|
| **Operating System** | Windows 10/11 ×64; Linux with a current native source build |
| **Python** | 3.12 on Windows; matching local CPython on Linux |
| **Runtime libraries** | Python `tkinter` plus the packages in `requirements.txt` |
| **Native source build** | CMake, a compatible C++ compiler, and `ARrobots/src/requirements-build.txt` |
| **Hardware** | Teensy 4.1 controller + AR4-MK3 robot |
| **Linux** | sudo apt-get install wmctrl |

The repository provides a supported native binary for Windows CPython 3.12 x64. Bundled Linux extension files use the legacy native API and are rejected for motion; build the current tracked native source before running the HMI on Linux.


---

## ⚙️ Setup & Running from Source

Hardware boundaries are documented in
[`SAFETY.md`](SAFETY.md).
Typed scripting uses the [Python automation API](docs/python-automation-api.md).

```bash
# Clone the repository
git clone https://github.com/Polymistis/ar4-hmi.git
cd ar4-hmi

# (Optional) Create a virtual environment
python -m venv venv
venv\Scripts\activate

# Install base dependencies
pip install -r requirements.txt

# Optional source STEP/STP import support
pip install -r requirements-step.txt

# Run the control interface
python AR4.py
```

The base install remains `requirements.txt`. Source STEP/STP import additionally
requires `requirements-step.txt`. STEP/STP support in a frozen build requires bundling the optional STEP dependencies.

Starting the interface explicitly admits the saved main-controller connection,
validated configuration and position synchronization, configured auxiliary
connection, and board-defined startup effects. Confirm the intended controllers
and active profile before launch. The main-controller startup sequence sends no
motor-drive command. Opening a configured auxiliary port can reset the board and
invoke firmware-defined digital-I/O initialization. The tracked Mega firmware
preloads pins 28-35 high before switching output pins to output mode. Both
tracked auxiliary firmware builds leave servos detached until a JSON `servo`
request supplies a target.
Orderly shutdown submits one best-effort correlated `gripper_detach {}` after
auxiliary activity settles. Failure after write admission is diagnosed without
automatic retry.

## 🧠 Troubleshooting
- **Serial connection issues** → Verify the correct COM port and matched JSON-only firmware: Teensy `6.7.1-ar4hmi.38` plus Nano/Mega `2.0`. Startup requires the exact command manifests defined by the [JSON protocol](docs/json-protocol-v1.md); legacy serial commands and fallback are not supported.
- **Motion tracking shows estimates only** → Request-scoped J1-J6 encoder telemetry is selected by JSON `move_joints`. J7-J9 remain estimated because no matching encoder sources are configured.
- **Shutdown Position unavailable** → JSON startup obtains the J1-J3 home reference after configuration. Complete J2 and J3 homing after controller startup or parameter and forced-position updates.
- **Display lag in visualization** → Disable real-time rendering under *Settings → Viewer Options*.

---

## 📜 License
This project is released under the  
**Annin Robotics Open Source Non-Commercial License v1.1 (2025)**.  
You may use and modify the software for personal, educational, or research purposes **only**.  
Commercial use, resale, or redistribution requires written permission.

➡ See the full terms in the [LICENSE.txt](./LICENSE.txt) file.

---

## 🧾 Credits & Contact
Developed by **Chris Annin** – Annin Robotics  
🌐 [https://www.anninrobotics.com](https://www.anninrobotics.com)  
📧 info@anninrobotics.com  

Special thanks to **[Jason Kirk](https://github.com/jason-technology)** for major contributions to the control software architecture and project development.

If you use the AR4 in research, teaching, or projects, please share your work with the community!
