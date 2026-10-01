# Computer Vision Drone-Following Work Plan

## Project goal

Build the computer-vision side of a simulated autonomous drone that can:

1. receive a target description such as `red car` or `walking person`;
2. find and keep the same target in the camera image;
3. command ArduPilot to follow the target safely; and
4. expose a small command/status boundary that the LLM team can connect to later.

The current work does **not** include interpreting general LLM instructions. Until the
LLM integration begins, target descriptions will be supplied through a command-line
argument or another simple local input.

## Ownership boundary

| Area | Computer vision team | LLM team |
|---|---|---|
| Camera and gimbal | Own | Does not control directly |
| Target detection and identity | Own | Supplies target description later |
| Tracking and follow velocity | Own | Starts or stops the behavior later |
| ArduPilot and Gazebo integration | Own for the vision-following path | Consumes status |
| Natural-language mission reasoning | Provides a small interface | Own |

Keep the real-time control loop independent of the LLM. The LLM may select a target
and request tracking, but camera processing and flight commands must continue without
waiting for an LLM response.

## Development rules for Codex

- Complete one work package at a time and verify it before starting the next.
- Reuse the existing Gazebo and ArduPilot models; do not modify ArduPilot core code
  unless the companion-computer interface cannot solve the problem.
- Use Gazebo Transport as the computer-vision camera input. Keep UDP H.264 for
  QGroundControl or optional debugging.
- Use one configurable MAVLink companion endpoint, initially
  `udpin:127.0.0.1:14551`.
- Keep QGroundControl on UDP port 14550.
- Every flight-control feature must have a target-loss behavior and a manual stop.
- Start with a fixed downward camera. Add active gimbal tracking only after drone
  following works with the fixed camera.
- Do not add an LLM framework during the vision milestones.

## Work package 1: Reliable gimbal control

**Status:** Complete — implemented and simulator-tested on September 30, 2026

### Verification completed

- The SITL launcher generated separate MAVProxy outputs for QGroundControl on UDP
  14550 and the companion script on UDP 14551.
- Every standard pitch/yaw command received `MAV_RESULT_ACCEPTED` from ArduPilot.
- Gazebo pose telemetry confirmed pitch movement from 0 to -90 degrees and yaw
  movement in both directions.
- Repeating the -90 degree command after restarting Gazebo and SITL produced the same
  camera-link orientation within normal simulation tolerance.
- The Gazebo Transport camera viewer received live frames after gimbal movement.
- Missing-heartbeat and invalid-angle behavior were tested.

QGroundControl itself was not opened during the automated check; its dedicated 14550
output was confirmed in the generated MAVProxy command.

### Objective

Make pitch and yaw commands move the simulated gimbal consistently through
ArduPilot and MAVLink while the drone remains on the ground.

### Tasks

1. Configure the SITL launch to send a dedicated MAVLink stream to port 14551.
2. Load both the project parameters and Gazebo gimbal parameters:
   - `config/dbox.parm`
   - `gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm`
3. Consolidate gimbal testing in `ai_agent/move_gimbal.py`.
4. Add command-line options for MAVLink endpoint, pitch, and yaw.
5. Add a finite heartbeat timeout and a useful connection failure message.
6. Wait for and report the MAVLink command acknowledgement when available.
7. Confirm the camera image changes with the gimbal angle using the Gazebo Transport
   viewer.
8. Document the exact launch and test commands.

### Acceptance criteria

- QGroundControl remains connected on port 14550 while the gimbal script connects on
  port 14551.
- Commands for pitch `0`, `-45`, and `-90` degrees visibly move the camera.
- Commands for yaw `-45`, `0`, and `45` degrees visibly move the camera.
- Repeating a command after restarting the simulation produces the same orientation.
- A missing MAVLink connection exits with an error instead of waiting forever.
- The test sends no vehicle movement, arming, or takeoff commands.

### Deliberately excluded

- Automatic gimbal search
- Target detection
- Drone movement
- Real-hardware gimbal support

## Work package 2: Deterministic moving targets

**Status:** Complete — updated and simulator-tested on October 1, 2026

### Verification completed

- A project-local Gazebo system advances both targets around their 22 m rectangular
  paths at a fixed simulated speed of 0.75 m/s, giving a 29.33-second loop period.
- A clean simulation ran for more than 181 simulated seconds (over six loops) with
  both targets moving continuously at an exact 0.20 m base height.
- A clean restart reproduced the initial position and fixed-speed motion.
- The exact paths remain within `x = [-4, 2]` and `y = [3, 8]` for the car and
  `y = [-8, -3]` for the pedestrian. The downward camera covers
  approximately 31.15 m by 23.36 m at the planned 10 m test altitude.
- The world loaded both stable model names and both deterministic-path plugins
  without missing-model or missing-plugin errors.

### Objective

Provide repeatable car and pedestrian targets for camera and tracking tests.

### Tasks

1. Repair and validate the existing `target_car` model and its looping trajectory.
2. Add a pedestrian target with a separate deterministic path.
3. Give each target a stable model name and distinct initial appearance.
4. Initially add visible colored markers for simple perception testing.
5. Remove conflicting world configuration, including duplicate spherical coordinates
   and redundant target poses.
6. Keep paths near the takeoff area and prevent targets from intersecting the drone.
7. Add one documented command that launches the test world.

### Acceptance criteria

- The car and pedestrian each complete at least three loops without stopping,
  falling, or leaving the test area.
- Their paths are repeatable after a simulation restart.
- Both targets are visible from the drone at the planned test altitude.
- The world loads without missing-model or plugin errors.

## Work package 3: Perception-only target tracking

**Status:** Complete — implemented and simulator-tested on September 30, 2026

### Verification completed

- The car was detected in all 300 camera frames during a 30-second live run at
  10 Hz, with zero retained frames and zero target losses.
- The pedestrian was directly detected in 296 of 301 frames during a 30-second live
  run. Five brief misses used the identity-memory window, with zero target losses.
- Both live runs covered more than one complete target loop from a 10 m hover with
  the camera pointed downward.
- Synthetic-image tests cover car selection, pedestrian selection, normalized image
  error, short-loss identity retention, loss counting, and malformed camera buffers.
- The tracker imports no MAVLink library and contains no vehicle-command output.

### Objective

Detect and track a selected target in the image without sending flight commands.

### Tasks

1. Consolidate the camera viewer and tracker into the `ai_agent` directory.
2. Receive raw camera frames through Gazebo Transport.
3. Start with deterministic marker detection:
   - yellow marker selects the car;
   - a second marker selects the pedestrian.
4. Draw the target bounding box, center, confidence, and normalized image error.
5. Add a local target option such as `--target car` or `--target person`.
6. Keep the selected target identity through short missed detections.
7. Measure camera rate, processed-frame rate, and target-loss count.

### Acceptance criteria

- Either target can be selected without changing source code.
- The selected target remains identified for a complete movement loop.
- No MAVLink movement commands are sent.
- The viewer closes cleanly and reports camera or format errors clearly.

## Work package 4: Fixed-camera drone following

**Status:** Complete — implemented and simulator-tested on October 1, 2026

### Verification completed

- The car completed a full-loop 42-second follow run with 400 detections in 400
  processed frames, zero target losses, and continuous `TRACK` after acquisition.
  Its center was inside the 10% image error box for 97.0% of tracking frames; mean
  absolute normalized error was `(0.037, 0.038)`.
- The pedestrian completed the same test with 401 detections in 401 frames and zero
  losses. It was centered for 84.3% of tracking frames; mean absolute error was
  `(0.047, 0.085)`.
- Pausing the camera-producing simulation forced `TRACK -> LOST` and a
  `(0.00, 0.00)` body velocity command. The 0.4-second detection timeout plus the
  10 Hz control period bounds this response to 0.5 seconds.
- Ctrl+C stopped the follower and sent three final zero commands. ArduPilot remained
  armed in GUIDED with a healthy heartbeat and Gazebo continued publishing poses.
- Twelve unit tests cover the velocity-only MAVLink mask, zero vertical/yaw-rate
  fields, state acquisition, transient detection misses, stale-target stopping,
  manual stop, vehicle gating, perception, and gimbal commands.

### Objective

Follow the selected marked target with a fixed downward camera using ArduPilot GUIDED
velocity commands.

### Tasks

1. Fix the MAVLink velocity type mask so only the intended velocity fields are active.
2. Implement `IDLE`, `ACQUIRE`, `TRACK`, and `LOST` states.
3. Require several stable detections before entering `TRACK`.
4. Convert horizontal and vertical image error into body-frame X/Y velocity.
5. Keep yaw rate and vertical velocity at zero for the first controller.
6. Add a center deadband, smoothing, acceleration limit, and low maximum speed.
7. Send commands at a steady rate independent of the camera callback.
8. Stop horizontal motion within one second when the target is lost, and immediately
   when tracking is disabled.
9. Gate tracking on armed, airborne, and GUIDED vehicle state.
10. Display current state and commanded velocity in the camera view.

### Acceptance criteria

- From a stable hover, the drone follows the moving car for one complete loop.
- The target center normally stays within 10% of the image width and height from the
  image center.
- The drone commands zero horizontal velocity within one second of losing the target.
- The operator can stop tracking without closing Gazebo or ArduPilot.
- The same behavior works for the marked pedestrian target.

### Coordinated search and active-gimbal extension

**Status:** Implemented; controller tests and headless Gazebo target-motion check
pass. Gimbal servo response and closed-loop flight behavior in armed SITL still need
validation and tuning.

Run with `./run_ai_agent.sh follow --target car --search`. Search mode first holds
the aircraft at its current local position while the gimbal rasters left-to-right
across pitch rows from -85° to -45°. The terminal reports vehicle-gate changes.
Only after a complete scan without a detection does a bounded expanding spiral and
circle begin (1.5 m radius, 0.2 m/s); the gimbal continues scanning during this
fallback. Search motion stops after 90 seconds or at a 2.5 m safety boundary. A
candidate marker stops the aircraft and enters `LOCK`. The gimbal centers the
marker; after five stable detections and ten centered frames, `TRACK` projects the
gimbal ray onto the ground and commands limited body-frame velocity toward the
target. A 0.4-second detection timeout stops horizontal motion and returns to search.

Use `./run_ai_agent.sh world-check --target both --duration 5` to verify target
movement from Gazebo world-pose telemetry without depending on the camera image.

The gimbal raster uses both yaw and pitch. During lock and tracking, gimbal yaw may
move to keep a horizontally displaced target centered. Search mode also requires fresh
`LOCAL_POSITION_NED` and `ATTITUDE` telemetry in addition to the existing armed,
GUIDED, airborne, and heartbeat gates.

The ground projection uses commanded gimbal angles, configured camera field of view,
altitude, and an assumed marker height. Gazebo/SITL validation should confirm angle
signs and servo response, search radius and altitude bounds, lock stability, and
ground-offset following for both targets before this mode is used in a demo.

## Work package 5: Description-based detection

### Objective

Replace colored-marker selection with an open-vocabulary detector that accepts a
short text description.

### Tasks

1. Record representative Gazebo camera clips for repeatable offline tests.
2. Evaluate a small real-time open-vocabulary model such as YOLO-World and compare it
   with Grounding DINO if detection quality is insufficient.
3. Accept a description through `--target-description`, for example:
   - `red car`
   - `walking person`
   - `person wearing blue`
4. Run expensive detection periodically and use lightweight tracking between
   detections.
5. Prevent target switching by combining detection confidence, position continuity,
   and appearance similarity.
6. Keep the controller from work package 4 unchanged; replace only its perception
   input.

### Acceptance criteria

- The operator can choose the car or pedestrian using a text description.
- The detector identifies the requested target in recorded clips and in the live
  simulator.
- The full perception/control loop runs fast enough to maintain stable following on
  the available project computer.
- When several similar objects are present, tracking does not silently switch targets.

## Work package 6: LLM-team integration boundary

### Objective

Expose the proven tracking behavior without coupling flight control to a particular
LLM implementation.

### Proposed command

```json
{
  "command": "track",
  "description": "red car"
}
```

Other commands should initially be limited to:

```json
{"command": "stop"}
{"command": "status"}
```

### Proposed status

```json
{
  "state": "TRACK",
  "description": "red car",
  "target_visible": true,
  "confidence": 0.87
}
```

The transport mechanism—local socket, ROS 2 topic, or another team-selected
interface—should be chosen with the LLM team only when this work package starts.

### Acceptance criteria

- The LLM side can start and stop tracking without importing vision code.
- Invalid descriptions or commands do not produce flight movement.
- Status distinguishes searching, tracking, target lost, stopped, and error states.
- Disconnecting the LLM side does not interrupt an operator stop or target-loss hold.

## Final demonstration definition

1. Gazebo launches with an autonomous car and pedestrian.
2. The drone takes off and holds a safe test altitude under operator supervision.
3. A command selects either target by description.
4. The camera detects the target and the drone follows it.
5. A stop command or target loss makes the drone hold position.
6. QGroundControl continues to provide operator monitoring and manual intervention.

## Next Codex task

Start work package 5 only when requested. Keep the proven work package 4 controller
and safety gates unchanged while replacing its marker-based perception input with a
description-based detector.
