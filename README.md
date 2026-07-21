# AR4 Control Software
**Host source 6.7 — tracked Teensy derivative 6.7.1-ar4hmi.1**

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
| **Libraries** | `tkinter`, `ttkbootstrap`, `pyserial`, `vtk`, `numpy`, `pandas`, `pybind11` |
| **Hardware** | Teensy 4.1 controller + AR4-MK3 robot |
| **Linux** | sudo apt-get install wmctrl |

The repository provides a supported native binary for Windows CPython 3.12 x64. Bundled Linux extension files use the legacy native API and are rejected for motion; build the current tracked native source before running the HMI on Linux.


---

## ⚙️ Setup & Running from Source
```bash
# Clone the repository
git clone https://github.com/Annin-Robotics/ar4-hmi.git
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
- **Serial connection issues** → Verify the correct COM port and a Teensy 4.1 firmware build advertising `JT_WRIST_CONFIG_V1`; the tracked compatible build identifies version `6.7.1-ar4hmi.1`.
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
