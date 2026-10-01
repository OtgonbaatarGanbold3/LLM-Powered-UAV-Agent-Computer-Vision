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
|  - 3D physics & visual rendering |    |  - check_target_motion.py -- world pose check    |
|  - Iris drone + gimbal model     |    |  - drone_cv.py         -- vision-only tracking    |
|  - car + person target paths     |    |  - move_gimbal.py      -- MAVLink camera control  |
|  - Gazebo Transport topics       |    |  - follow_target.py    -- gated GUIDED controller |
+----------------------------------+    +--------------------------------------------------+
```

**Data flow summary:**

| Link | Protocol | Ports |
|---|---|---|
| Gazebo <-> ArduPilot SITL | JSON over UDP (lockstep) | 9002 / 9003 |
| ArduPilot SITL <-> QGC | MAVLink UDP | 14550 |
| ArduPilot SITL <-> Companion scripts | MAVLink UDP | 14551 |
| Gazebo camera <-> drone_cv.py | Gazebo Transport (gz.transport13) | IPC |

---

## 3. Repository Structure

```
LLM-Powered-UAV-Agent-Computer-Vision/
├── README.md                          <- You are here
├── ai_agent/                          <- Computer-vision companion code
│   ├── drone_cv.py                      Marker detector and identity tracker
│   ├── follow_target.py                  Fixed-camera GUIDED target follower
│   ├── move_gimbal.py                   MAVLink gimbal command tool
│   ├── check_target_motion.py           Camera-independent Gazebo pose check
│   ├── test_drone_cv.py                 Perception unit tests
│   ├── test_follow_target.py             Controller and MAVLink unit tests
│   └── test_move_gimbal.py              Gimbal command unit tests
├── config/dbox.parm                   <- Project SITL parameters
├── simulation/                        <- Project Gazebo models, path plugin, and patch
├── setup_simulation.sh                <- Applies the project overlay to Gazebo
├── REAL_WORLD_CV_UPGRADE_PLAN.md      <- Next-phase plan and acceptance criteria
├── ardupilot-dbox/                    <- Pinned upstream ArduPilot submodule
└── gz_ws/src/ardupilot_gazebo/        <- Pinned upstream Gazebo submodule
```

The two submodules are Git references to the official repositories, not copies
of their source in this repository. `setup_simulation.sh` applies only this
project's Gazebo changes to the checked-out upstream version.

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
git clone --recurse-submodules git@github.com:OtgonbaatarGanbold3/LLM-Powered-UAV-Agent-Computer-Vision.git
cd LLM-Powered-UAV-Agent-Computer-Vision
./setup_simulation.sh
```

For a clone made without submodules, run `git submodule update --init --recursive`
before `./setup_simulation.sh`.

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
cd ..
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

Apply the project overlay and build the plugin from the repository root:

```bash
./setup_simulation.sh
cmake -S gz_ws/src/ardupilot_gazebo -B gz_ws/src/ardupilot_gazebo/build-local -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build gz_ws/src/ardupilot_gazebo/build-local -j2
```

---

### 5.4 Configure Environment Variables

The project launchers set Gazebo paths automatically. For direct `gz sim`
commands, add the following to your `~/.bashrc` (or `~/.zshrc` on macOS).
Replace `$HOME/LLM-Powered-UAV-Agent-Computer-Vision` with your actual clone path if different.

```bash
# Append to ~/.bashrc
REPO_ROOT="$HOME/LLM-Powered-UAV-Agent-Computer-Vision"

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/gz_ws/src/ardupilot_gazebo/build-local:$GZ_SIM_SYSTEM_PLUGIN_PATH"
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

For the gimbal milestone, open three terminal windows from the repository root.

### Terminal 1 — Gazebo Harmonic (Physics & Rendering)

```bash
./run_gazebo.sh
```

The launcher builds the local target-path plugin when needed, configures the Gazebo
plugin and model paths, starts `iris_runway.sdf`, and enables the camera image
stream. It is also the one-command moving-target test: the red car with a yellow
roof marker loops north of the drone, while the blue pedestrian with a cyan marker
loops south of it. Both move at 0.75 m/s on deterministic 29.33-second paths. Their
Gazebo model names are `target_car` and `target_person`. SITL is not required when
only checking the target motion; use `./run_gazebo.sh -s` for a headless check. The
targets are dynamic models with gravity disabled; the path plugin commands their
poses so both the drone camera and Gazebo world view receive their motion.

The paths stay on opposite sides of the takeoff point and fit inside the 640 x 480
camera view when the downward camera is centered from the planned 10 m test altitude.
Wait until Gazebo is fully loaded before starting SITL.

---

### Terminal 2 — ArduPilot SITL

```bash
./run_ardupilot.sh
```

The launcher automatically uses `~/venv-ardupilot` when the current Python does not
have the required packages. Set `ARDUPILOT_VENV=/path/to/venv` if your environment is
elsewhere. Run it in an interactive terminal; MAVProxy needs terminal input to keep
its telemetry outputs alive. It also loads the project and gimbal parameter files.
MAVProxy sends telemetry to QGroundControl on UDP 14550 and to companion scripts on UDP 14551. Pass
`--console --map` to the launcher if the MAVProxy graphical tools are wanted.

Before arming, check the disarmed simulator baseline in a third terminal:

```bash
./run_ai_agent.sh preflight
```

This passively checks the MAVLink heartbeat, camera frames, moving targets, and that
the drone stays near its initial Gazebo pose. Run it after each clean startup; it
does not arm or command the vehicle. Use `./run_ardupilot.sh -w` when a repeatable
run needs fresh ArduPilot parameters instead of the saved `eeprom.bin`.

**Arm and take off** from the MAVProxy console:

```
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 10
```

---

### Terminal 3 — Camera, perception, and following

```bash
./run_ai_agent.sh gimbal --pitch -90 --yaw 0
./run_ai_agent.sh track --target car
```

Select the pedestrian without changing source code:

```bash
./run_ai_agent.sh track --target person
```

For a timed test without a window:

```bash
./run_ai_agent.sh track --target car --no-display --duration 30
```

After arming in GUIDED, taking off to 10 m, and pointing the gimbal down, run the
fixed-camera diagnostic tracker (it does not command pursuit) with:

```bash
./run_ai_agent.sh follow --target car
./run_ai_agent.sh follow --target person
```

To start the coordinated search and gimbal lock mode, use:

```bash
./run_ai_agent.sh follow --target car --search
./run_ai_agent.sh follow --target person --search
```

Search mode first holds the aircraft's current local position while the gimbal scans
left-to-right across three pitch rows (-85° to -45° by default). The terminal prints
`vehicle gate: ...` whenever the armed, GUIDED, altitude, heartbeat, local-position,
or heading gate changes. The camera overlay also shows the current gate. If a complete
raster finds no target, a small expanding spiral and circle starts (1.5 m radius,
0.2 m/s) while the gimbal keeps scanning. A detected car marker stops search movement;
the gimbal centers it while the drone holds position. After a stable lock, the drone
follows at a 2.5 m standoff. Pursuit projects the detected image point through the
measured Gazebo camera pose onto the target-height plane. If camera pose or detection
goes stale, pursuit stops. Use `--no-follow-motion` to inspect tracking and gimbal
behavior without target pursuit; the bounded search fallback can still move if no
target is found. Search movement stops after 90 seconds or if the aircraft
exceeds 2.5 m from its recorded search center. Tune the scan with `--scan-yaw-min`,
`--scan-yaw-max`, `--scan-pitch-min`, `--scan-pitch-max`, `--scan-pitch-step`, and
`--scan-rate`; tune the fallback with `--search-radius`, `--search-speed`, and
`--search-timeout`.

Both follow modes require fresh local-position telemetry. Search pursuit stops issuing
movement commands 12 m from where the follower started; restart the follower to reset
this boundary. The default pursuit speed is capped at 1.0 m/s. The camera-pose source
and target-height assumption are specific to this Gazebo model, so real-world pursuit
needs a calibrated camera pose and a range/depth source before flight.

To confirm target models are moving in Gazebo independently of the camera, run this
in another terminal while Gazebo is running:

```bash
./run_ai_agent.sh world-check --target both --duration 5
```

It subscribes to the Gazebo world pose stream and reports the measured world-position
change for each model. `MOVING` confirms the target-path plugin is updating world
poses; `STATIONARY` or `NOT FOUND` points to the Gazebo world/plugin setup rather than
the camera tracker.

Press `s` in the camera window to stop or restart tracking. In search mode this
also stops the gimbal scan and holds the aircraft at its current local position.
Press `q` or Ctrl+C to exit; the search-mode exit path also requests a position
hold when fresh local-position telemetry is available. Gazebo and ArduPilot keep
running. For repeatable headless
fixed-camera runs:

```bash
./run_ai_agent.sh follow --target car --no-display --duration 42
```

This launcher also selects `~/venv-ardupilot` automatically when the current Python
does not contain OpenCV, Gazebo Transport, or pymavlink.

QGroundControl is optional and continues to connect automatically on UDP 14550.

The tracker draws the selected target's bounding box, center, confidence, normalized
image error, camera rate, processing rate, and target-loss count. It has no MAVLink
connection and sends no vehicle or gimbal commands.

Run all standard pitch and yaw positions separately with:

```bash
./run_ai_agent.sh gimbal --sequence
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
| `scripts/` | On-board Lua scripts for local SITL experiments |
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
| `worlds/iris_runway.sdf` | Main world (drone + car + pedestrian + runway) |
| `worlds/iris_warehouse.sdf` | Alternative indoor warehouse world |
| `worlds/gimbal.sdf` | Gimbal and camera streaming demo |
| `models/iris_with_gimbal/` | Iris with 3-axis gimbal and camera sensor |
| `models/iris_with_ardupilot/` | Plain Iris drone model |
| `models/target_car/` | Yellow-roofed moving target car |
| `models/target_person/` | Blue pedestrian target with cyan marker |
| `models/runway/` | Runway environment |
| `src/DeterministicPathPlugin.cc` | Fixed-speed looping target motion |

**Gimbal RC control** (from MAVProxy):

| Action | Channel | RC Low | RC High |
|---|---|---|---|
| Roll | RC6 | Roll Left | Roll Right |
| Pitch | RC7 | Pitch Down (nadir) | Pitch Up |
| Yaw | RC8 | Yaw Left | Yaw Right |

Tilt camera toward nadir: `rc 7 1100`

`./run_gazebo.sh` builds the local deterministic-path plugin when it is missing or
out of date. To rebuild all Gazebo plugins manually:

```bash
cmake -S gz_ws/src/ardupilot_gazebo -B gz_ws/src/ardupilot_gazebo/build-local
cmake --build gz_ws/src/ardupilot_gazebo/build-local -j2
```

---

### 7.3 Computer Vision Agent (`ai_agent/`)

#### `drone_cv.py` — Perception-only Target Tracking

Subscribes to the Gazebo gimbal camera topic and uses HSV color segmentation to
select either the car's yellow roof marker or the pedestrian's cyan marker. It keeps
the last target through five missed frames and reports a loss only after that memory
window expires. This module deliberately has no MAVLink dependency.

Camera topic:
```
world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Useful options:

| Option | Default | Description |
|---|---|---|
| `--target` | `car` | Select `car` or `person` |
| `--max-missed-frames` | `5` | Frames that retain the last target identity |
| `--min-area` | Target-specific | Override the minimum marker contour area |
| `--duration` | `0` | Timed run; zero runs until stopped |
| `--no-display` | off | Process and report metrics without a window |

#### `follow_target.py` — Fixed-Camera Target Following

Uses the marker detector as its perception input and sends velocity-only
`SET_POSITION_TARGET_LOCAL_NED` commands in `MAV_FRAME_BODY_NED` at 10 Hz. It
requires five stable detections before `TRACK`, holds through brief missed frames,
and enters `LOST` with a zero command after 0.4 seconds without a valid detection.
Movement is permitted only while ArduPilot is armed, at least 1 m airborne, and in
GUIDED mode. Vertical velocity and yaw rate remain zero.

The controller applies a center deadband, smoothing, acceleration limiting, and a
2 m/s speed cap. Its camera overlay and headless logs show the state, vehicle gate,
image error, and commanded forward/right velocity.

With `--search`, it also requests local position and heading telemetry, holds the
aircraft stationary during the gimbal raster, and sends a bounded NED position
setpoint only after the raster completes without a detection. It tracks the gimbal
angles it commands. During `TRACK`, it estimates the target's ground offset from altitude,
camera field of view, and gimbal pitch/yaw, then sends limited body-frame velocity
commands. This first ground projection assumes the camera is calibrated and the
target marker height is known; validate and tune it in SITL before relying on the
active-gimbal mode.

## 8. Custom Parameters (`dbox.parm`)

The project-owned [config/dbox.parm](config/dbox.parm) is loaded at SITL startup:

| Parameter | Value | Description |
|---|---|---|
| `SCR_ENABLE` | `1` | Enable on-board Lua scripting engine |
| `SCR_VM_I_COUNT` | `200000` | Lua VM instruction limit per scheduler step |
| `SCR_HEAP_SIZE` | `102400` | Heap memory for Lua (bytes) |

Edit `config/dbox.parm` to persist project parameters across SITL restarts.

---

## 9. Development Guide

### On-Board Lua Scripts

1. Keep project Lua sources outside the upstream ArduPilot checkout, then copy them
   into `ardupilot-dbox/scripts/` for local SITL tests.
2. Ensure `SCR_ENABLE=1` is in `config/dbox.parm` (already set).
3. Launch SITL — scripts load automatically.
4. View output in the MAVProxy console.

The ArduPilot checkout is an upstream submodule; store project Lua sources in this
repository if they need to be shared with teammates.

### Off-Board Python Companion Script

1. Create your script in `ai_agent/`.
2. Connect via `pymavlink`:
   ```python
   from pymavlink import mavutil
   conn = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
   conn.wait_heartbeat()
   ```

### New Gazebo Model

1. Create model files under `simulation/ardupilot_gazebo/models/<model_name>/`.
2. Extend `setup_simulation.sh` to install them into the Gazebo checkout and add
   a world patch if the model needs to be included at startup.
3. Run `./run_gazebo.sh`; it applies the overlay and builds the local plugin.

### Git Workflow

```bash
git checkout -b codex/<feature-name>
git add ai_agent simulation config README.md CV_WORK_PLAN.md REAL_WORLD_CV_UPGRADE_PLAN.md
```

Commit project code in this repository. Upstream ArduPilot and Gazebo changes belong
in their own repositories or in the project overlay.

---

## 10. Troubleshooting

| Problem | Solution |
|---|---|
| Gazebo can't find plugin | Run `./run_gazebo.sh`; it builds and exports both `build-local/` and `build/` plugin paths. |
| Follower stays in `IDLE` | Confirm ArduPilot is armed, above 1 m, and in GUIDED mode. |
| Follower stays in `ACQUIRE` | Point the gimbal down and confirm the selected marker is visible with `track`. |
| SITL fails to connect to Gazebo | Ensure Gazebo is **fully loaded** before starting SITL. Look for `JSON connected` in Gazebo verbose output. |
| Camera tracker shows no image | Verify the camera topic path with `gz topic -l`. Check the world SDF camera sensor is active. |
| MAVLink connection refused | Confirm SITL is running. Check port 14551 availability: `ss -u -a | grep 14551` |
| `pymavlink` not found | Activate the venv: `source ~/venv-ardupilot/bin/activate` |
| Lua scripts not loading | Run `param show SCR_ENABLE` in MAVProxy. Ensure `config/dbox.parm` is loaded by `run_ardupilot.sh`. |
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
