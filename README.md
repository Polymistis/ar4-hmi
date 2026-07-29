# AR4 Control Software
**Host source 6.7 — tracked Teensy derivative 6.7.1-ar4hmi.10**

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
- Optional packaged Windows EXE build  

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

```bash
# Clone the repository
git clone https://github.com/Polymistis/ar4-hmi.git
cd ar4-hmi

# (Optional) Create a virtual environment
python -m venv venv
venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run the control interface
python AR4.py
```

## 🧠 Troubleshooting
- **Serial connection issues** → Verify the correct COM port and a Teensy 4.1 firmware build identifying version `6.7.1-ar4hmi.10` and advertising `JT_WRIST_CONFIG_V1`, `GCODE_DIRECTORY_FRAMING_V1`, `GCODE_DELETE_IDENTITY_V1`, `GCODE_WRITE_IDENTITY_V1`, and `CALIBRATION_SWITCH_POLARITY_V1`.
- **Motion tracking shows estimates only** → `JOINT_TELEMETRY_V1` is optional; the matching firmware adds request-scoped J1-J6 encoder telemetry while J7-J9 remain estimated.
- **Shutdown Position unavailable** → `HOME_REFERENCE_V2` is optional for connection but required for the corrected parking action; complete J2 and J3 homing after controller startup or parameter and forced-position updates. Legacy `HOME_REFERENCE_V1` controllers remain connectable but cannot supply the required J3 switch reference.
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
