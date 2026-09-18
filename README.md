# LLM-Powered UAV Agent — Computer Vision Capstone

> **Drone-in-a-Box (DBox)** simulation and automation framework powered by  
> **ArduPilot SITL · Gazebo Harmonic · MAVLink · OpenCV · LLM Task Agents**

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Repository Structure](#3-repository-structure)
4. [Prerequisites](#4-prerequisites)
5. [Installation & Setup](#5-installation--setup)
6. [Running the Simulation](#6-running-the-simulation)
7. [Key Components](#7-key-components)
8. [Custom Parameters (dbox.parm)](#8-custom-parameters-dboxparm)
9. [Development Guide](#9-development-guide)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Project Overview

This capstone project implements an autonomous **Drone-in-a-Box (DBox)** system that combines:

- **ArduPilot** flight control firmware running in Software-In-The-Loop (SITL) mode
- **Gazebo Harmonic** 3D physics and rendering simulation
- **Computer vision** target tracking using OpenCV and Gazebo camera feeds
- **MAVLink** real-time telemetry and command interface
- **LLM-based task agents** for high-level mission planning and decision making

The primary demonstration scenario is an **autonomous drone that visually tracks a moving target** (yellow car roof) using a downward-facing gimbal camera, issuing velocity commands through ArduPilot's GUIDED mode.

---

## 2. System Architecture

```
+-----------------------------------------------------------------------------------+
|                              QGroundControl (GCS)                                 |
|             Operator GUI / Mission Planning / Status Monitoring                    |
+------------------------------------------+----------------------------------------+
                                           | MAVLink (UDP: 14550)
                                           v
+-----------------------------------------------------------------------------------+
|                        ArduPilot SITL (Software In The Loop)                      |
|   - Flight dynamics, EKF3 state estimation, navigation & PID control              |
|   - On-board Lua scripting engine (AP_Scripting)                                  |
|   - MAVProxy routing interface & GCS console                                       |
+-------------------+--------------------------------------+-------------------------+
                    | JSON / Lockstep (UDP: 9002/9003)     | MAVLink (UDP: 14551)
                    v                                      v
+----------------------------------+    +--------------------------------------------------+
|        Gazebo Harmonic           |    |          Companion / Automation Layer             |
|  - 3D physics & visual rendering |    |  - dock_controller.py  -- MAVLink monitor         |
|  - Iris drone + gimbal model     |    |  - target_tracker.py   -- OpenCV vision + GUIDED  |
|  - target_car, runway world      |    |  - test_camera.py      -- Camera stream debug      |
|  - Gazebo Transport topics       |    |  - LLM task agent (planned)                       |
+----------------------------------+    +--------------------------------------------------+
```

**Data flow summary:**

| Link | Protocol | Ports |
|---|---|---|
| Gazebo <-> ArduPilot SITL | JSON over UDP (lockstep) | 9002 / 9003 |
| ArduPilot SITL <-> QGC | MAVLink UDP | 14550 |
| ArduPilot SITL <-> Companion scripts | MAVLink UDP | 14551 |
| Gazebo camera <-> target_tracker.py | Gazebo Transport (gz.transport13) | IPC |

---

## 3. Repository Structure

```
LLM-Powered-UAV-Agent-Computer-Vision/
├── README.md                          <- You are here
│
├── ardupilot-dbox/                    <- ArduPilot SITL fork (vehicle firmware)
│   ├── ArduCopter/                      Flight mode C++ sources
│   ├── libraries/                       Shared ArduPilot C++ libraries
│   │   ├── AC_PID/                      PID controllers
│   │   ├── AC_WPNav/                    Waypoint navigation
│   │   ├── AC_PrecLand/                 Precision landing
│   │   └── AP_Scripting/                Lua scripting engine & bindings
│   ├── scripts/                         On-board Lua scripts (SITL runtime)
│   ├── dbox_control/                    Off-board Python companion scripts
│   │   ├── dock_controller.py           MAVLink status monitor & dock orchestrator
│   │   ├── target_tracker.py            OpenCV visual tracker -> velocity commands
│   │   └── test_camera.py               Camera stream debug tool
│   ├── docs/
│   │   └── 2026-09-16_project_guide.md  Detailed developer navigation guide
│   ├── dbox.parm                        Custom SITL parameter file
│   ├── Tools/autotest/sim_vehicle.py    SITL launcher
│   └── AGENTS.md                        ArduPilot contribution & coding rules
│
└── gz_ws/                             <- Gazebo Harmonic workspace
    └── src/
        └── ardupilot_gazebo/            Official ArduPilot Gazebo plugin
            ├── models/                  3D drone and environment models
            │   ├── iris_with_gimbal/     Iris quad with 3-axis gimbal + camera
            │   ├── iris_with_ardupilot/  Plain Iris model
            │   ├── target_car/           Moving target car (yellow roof)
            │   └── runway/               Runway environment
            ├── worlds/                  SDF world files
            │   ├── iris_runway.sdf       Main simulation world
            │   ├── iris_warehouse.sdf    Warehouse environment
            │   └── gimbal.sdf            Gimbal control demo world
            ├── src/                     C++ Gazebo plugin source
            ├── include/                 Plugin headers
            └── CMakeLists.txt           Build configuration
```

---

## 4. Prerequisites

### Operating System
- **Ubuntu 22.04 LTS (Jammy)** — recommended
- Ubuntu 20.04 minimum (for OpenGL / ogre2 rendering support)
- macOS Big Sur or later (Intel & Apple Silicon) — partially supported

### Required Software

| Software | Version | Purpose |
|---|---|---|
| Gazebo Harmonic | 8.x (LTS) | 3D physics simulation |
| ArduPilot / MAVProxy | Latest stable | SITL flight controller |
| Python | 3.10+ | Companion automation scripts |
| pymavlink | Latest | MAVLink Python bindings |
| OpenCV (cv2) | 4.x | Computer vision |
| NumPy | Latest | Array operations |
| gz-transport13 + gz-msgs10 | Harmonic | Gazebo Python bindings |
| QGroundControl | Latest AppImage | Ground control station GUI |
| CMake | 3.22+ | Building the Gazebo plugin |

---

## 5. Installation & Setup

### 5.1 Clone the Repository

```bash
git clone https://github.com/OtgonbaatarGanbold3/LLM-Powered-UAV-Agent-Computer-Vision.git
cd LLM-Powered-UAV-Agent-Computer-Vision
```

---

### 5.2 Set Up ArduPilot SITL

ArduPilot requires its own Python virtual environment and build toolchain.

```bash
# Install system dependencies
sudo apt update
sudo apt install -y git python3-pip python3-venv python3-dev \
    gcc g++ make libtool libxml2-dev libxslt1-dev \
    python3-matplotlib python3-serial python3-scipy

# Create and activate the ArduPilot virtual environment
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate

# Install MAVProxy and pymavlink
pip install MAVProxy pymavlink

# Build ArduCopter SITL
cd ardupilot-dbox
./waf configure --board sitl
./waf copter
```

> **Note:** The first build takes 5-15 minutes depending on your machine.

---

### 5.3 Build the Gazebo Plugin

First, install Gazebo Harmonic if not already installed:

```bash
# Add the OSRF package repository
sudo apt install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo apt install -y gz-harmonic
```

Install plugin build dependencies:

```bash
sudo apt install -y libgz-sim8-dev rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

Build the plugin:

```bash
cd gz_ws/src/ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

---

### 5.4 Configure Environment Variables

Add the following to your `~/.bashrc` (or `~/.zshrc` on macOS).  
Replace `$HOME/LLM-Powered-UAV-Agent-Computer-Vision` with your actual clone path if different.

```bash
# Append to ~/.bashrc
REPO_ROOT="$HOME/LLM-Powered-UAV-Agent-Computer-Vision"

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/gz_ws/src/ardupilot_gazebo/models:$REPO_ROOT/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"
```

Apply immediately:

```bash
source ~/.bashrc
```

---

### 5.5 Install Python Dependencies

```bash
source ~/venv-ardupilot/bin/activate
pip install pymavlink opencv-python numpy

# Gazebo Python transport bindings (Harmonic):
pip install gz-msgs10 gz-transport13
# If the above fail, use the system packages instead:
# sudo apt install -y python3-gz-msgs10 python3-gz-transport13
```

---

## 6. Running the Simulation

Open **four separate terminal windows**:

### Terminal 1 — Gazebo Harmonic (Physics & Rendering)

```bash
source ~/.bashrc
gz sim -v4 -r iris_runway.sdf
```

The `-v4` flag enables verbose output. The `iris_runway.sdf` world loads the Iris quad with gimbal, the yellow target car, and the runway environment. Wait until Gazebo is fully loaded before starting SITL.

---

### Terminal 2 — ArduPilot SITL

```bash
cd ardupilot-dbox
source ~/venv-ardupilot/bin/activate

./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    --console \
    --map \
    --add-param-file=dbox.parm
```

| Flag | Description |
|---|---|
| `-v ArduCopter` | Select multirotor vehicle type |
| `-f gazebo-iris` | Use the Gazebo-Iris airframe definition |
| `--model JSON` | JSON protocol for Gazebo communication |
| `--console` | Open MAVProxy GCS console window |
| `--map` | Open the moving-map display |
| `--add-param-file=dbox.parm` | Load custom parameters (enables Lua scripting) |

**Arm and take off** from the MAVProxy console:

```
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 10
```

---

### Terminal 3 — QGroundControl (Optional)

```bash
~/qgroundcontrol.sh
```

QGC connects automatically via UDP 14550. Use it for mission planning, parameter tuning, and live video (UDP H.264 on port 5600).

---

### Terminal 4 — Companion / Automation Scripts

**Visual target tracker** (tracks the yellow car, sends velocity commands in GUIDED mode):

```bash
cd ardupilot-dbox
source ~/venv-ardupilot/bin/activate
python3 dbox_control/target_tracker.py
```

**Dock controller** (MAVLink telemetry monitor):

```bash
python3 dbox_control/dock_controller.py --connect udpin:127.0.0.1:14551
```

**Camera debug tool**:

```bash
python3 dbox_control/test_camera.py
```

---

## 7. Key Components

### 7.1 ArduPilot SITL (`ardupilot-dbox/`)

| Path | Purpose |
|---|---|
| `ArduCopter/` | Vehicle C++ sources — flight modes, navigation, parameters |
| `ArduCopter/mode_guided.cpp` | GUIDED mode (used for velocity commands) |
| `libraries/AC_PID/` | PID attitude and rate controllers |
| `libraries/AC_WPNav/` | Waypoint and position navigation |
| `libraries/AC_PrecLand/` | Precision landing (beacon/vision) |
| `libraries/AP_Scripting/` | Lua scripting engine bindings and applet examples |
| `scripts/` | On-board Lua scripts — loaded at SITL boot (git-ignored; use `git add -f`) |
| `Tools/autotest/sim_vehicle.py` | Primary SITL launcher |
| `AGENTS.md` | Contribution rules, commit conventions, code style |

**Rebuild after C++ changes:**

```bash
cd ardupilot-dbox
./waf copter
```

---

### 7.2 Gazebo Harmonic Plugin (`gz_ws/`)

| Path | Purpose |
|---|---|
| `worlds/iris_runway.sdf` | Main world (drone + car + runway) |
| `worlds/iris_warehouse.sdf` | Alternative indoor warehouse world |
| `worlds/gimbal.sdf` | Gimbal and camera streaming demo |
| `models/iris_with_gimbal/` | Iris with 3-axis gimbal and camera sensor |
| `models/iris_with_ardupilot/` | Plain Iris drone model |
| `models/target_car/` | Yellow-roofed moving target car |
| `models/runway/` | Runway environment |

**Gimbal RC control** (from MAVProxy):

| Action | Channel | RC Low | RC High |
|---|---|---|---|
| Roll | RC6 | Roll Left | Roll Right |
| Pitch | RC7 | Pitch Down (nadir) | Pitch Up |
| Yaw | RC8 | Yaw Left | Yaw Right |

Tilt camera toward nadir: `rc 7 1100`

**Rebuild plugin after changes:**

```bash
cd gz_ws/src/ardupilot_gazebo/build
make -j$(nproc)
```

---

### 7.3 Companion Automation (`ardupilot-dbox/dbox_control/`)

#### `target_tracker.py` — Visual Target Tracking

Subscribes to the Gazebo gimbal camera topic, processes each frame with OpenCV to detect the **yellow roof pad** (HSV color segmentation), and sends proportional **body-frame velocity commands** to ArduPilot.

Camera topic:
```
world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Tunable constants at the top of the file:

| Constant | Default | Description |
|---|---|---|
| `KP_FORWARD` | `1.5` | Max forward/backward speed (m/s) |
| `KP_LATERAL` | `1.5` | Max left/right speed (m/s) |
| `KP_YAW` | `0.8` | Max yaw rate (rad/s) |
| `YELLOW_LOWER` | `[20,100,100]` | HSV lower bound for yellow detection |
| `YELLOW_UPPER` | `[40,255,255]` | HSV upper bound for yellow detection |
| `MIN_TARGET_AREA` | `100` | Minimum contour area (pixels) to lock on |

#### `dock_controller.py` — MAVLink Telemetry Monitor

Connects to ArduPilot and prints:
- Flight mode and armed state (`HEARTBEAT`)
- GPS coordinates and altitude (`GLOBAL_POSITION_INT`)
- On-board GCS messages (`STATUSTEXT`)

---

## 8. Custom Parameters (`dbox.parm`)

Loaded at SITL startup via `--add-param-file=dbox.parm`:

| Parameter | Value | Description |
|---|---|---|
| `SCR_ENABLE` | `1` | Enable on-board Lua scripting engine |
| `SCR_VM_I_COUNT` | `200000` | Lua VM instruction limit per scheduler step |
| `SCR_HEAP_SIZE` | `102400` | Heap memory for Lua (bytes) |

Edit `dbox.parm` to persist any parameter changes across SITL restarts.

---

## 9. Development Guide

### On-Board Lua Scripts

1. Place your `.lua` file in `ardupilot-dbox/scripts/`.
2. Ensure `SCR_ENABLE=1` is in `dbox.parm` (already set).
3. Launch SITL — scripts load automatically.
4. View output in the MAVProxy console.

> `scripts/` is in ArduPilot's `.gitignore`. Track explicitly:
> ```bash
> git add -f scripts/my_script.lua
> ```

### Off-Board Python Companion Script

1. Create your script in `ardupilot-dbox/dbox_control/`.
2. Connect via `pymavlink`:
   ```python
   from pymavlink import mavutil
   conn = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
   conn.wait_heartbeat()
   ```

### New Gazebo Model

1. Create `gz_ws/src/ardupilot_gazebo/models/<model_name>/model.config` and `model.sdf`.
2. Reference it in a world SDF file.
3. Rebuild: `cd gz_ws/src/ardupilot_gazebo/build && make -j$(nproc)`.

### Git Workflow

```bash
# Always work on a feature branch
git checkout -b feature/<feature-name>

# Commit message convention (subsystem prefix)
# Copter: implement custom docking landing sequence
# dbox_control: improve yellow target HSV thresholds
# AP_Scripting: expose new vehicle state variables
```

Do not commit directly to `master`. See `ardupilot-dbox/AGENTS.md` for full rules.

---

## 10. Troubleshooting

| Problem | Solution |
|---|---|
| Gazebo can't find plugin | Check `GZ_SIM_SYSTEM_PLUGIN_PATH` includes the `build/` dir. Run `source ~/.bashrc`. |
| SITL fails to connect to Gazebo | Ensure Gazebo is **fully loaded** before starting SITL. Look for `JSON connected` in Gazebo verbose output. |
| `target_tracker.py` shows no image | Verify the camera topic path with `gz topic -l`. Check the world SDF camera sensor is active. |
| MAVLink connection refused | Confirm SITL is running. Check port 14551 availability: `ss -u -a | grep 14551` |
| `pymavlink` not found | Activate the venv: `source ~/venv-ardupilot/bin/activate` |
| Lua scripts not loading | Run `param show SCR_ENABLE` in MAVProxy. Ensure `dbox.parm` is loaded via `--add-param-file`. |
| `waf copter` build fails | Run `./waf distclean` then retry with the venv active. |
| Gazebo rendering issues (black screen) | Requires OpenGL 3.3+. On VMs, enable 3D acceleration or use software rendering. |

For Gazebo-specific issues: [Gazebo troubleshooting docs](https://gazebosim.org/docs/harmonic/troubleshooting)  
For ArduPilot SITL issues: [ArduPilot dev docs](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

---

## Acknowledgements

- [ArduPilot](https://ardupilot.org/) — open-source autopilot firmware
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) — official Gazebo plugin
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) — robotics simulator
- [QGroundControl](http://qgroundcontrol.com/) — ground control station
