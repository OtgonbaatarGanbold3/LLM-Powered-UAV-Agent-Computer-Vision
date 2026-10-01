# Computer Vision Upgrade Plan

## Current checkpoint (updated 2026-10-01)

The project has simulator-tested MAVLink gimbal commands, deterministic car and
pedestrian paths, color-marker detection, and fixed-camera tracking. The newer
gimbal raster and limited drone search are implemented and have controller tests.
Three clean disarmed launches passed a passive preflight: heartbeat, camera frames,
moving targets, and a stationary drone at its spawn pose. The missing heartbeat was
reproduced when MAVProxy started without an interactive terminal; the launcher now
checks for one and the follower reports a clearer connection error.

Armed search reached `RASTER`, `MOVE`, `LOCK`, and `TRACK`. An earlier moving-car
test exposed a geometry error: the marker stayed centered while the drone traveled
roughly 28 m away from the target area. The current follower composes the measured
Gazebo camera pose from the drone and gimbal transforms and projects the detection
onto the target-height plane. In an 80-frame world-position comparison, the median
car-position error was about 0.09 m. Bounded SITL runs showed the drone following
the car after lock; active pursuit now runs in search mode by default and stops on
stale pose, stale detection, or the 12 m flight-radius boundary. `--no-follow-motion`
keeps the gimbal tracking without target pursuit; bounded search can still move
if no target is found.

The present detector selects the yellow marker beside the red car body or the cyan
marker on the pedestrian. It does not recognize an ordinary car or person from
appearance or a description. The camera ray still uses an assumed target height
and a flat plane; a real vehicle needs calibrated camera pose and measured depth.

## Target capability

An operator supplies a short description such as `red car` or `person wearing blue`.
The vision process finds one matching target, keeps its identity through brief
occlusion, and supplies a timestamped target estimate to the existing flight
controller. The LLM team can later request `track`, `stop`, and `status`; it does
not run the real-time control loop.

## Implementation order

### 1. Make the simulator baseline repeatable

- Start Gazebo and SITL from clean processes three times. Confirm the drone stays at
  its initial pose until the operator arms and commands takeoff.
- Confirm the follower receives a heartbeat on UDP 14551, prints `vehicle gate:
  ready`, and can stop the search and hold position.
- In armed SITL, check actual gimbal pitch and yaw against the raster commands, then
  check the drone remains stationary for a full raster and moves only during the
  fallback search.
- Record the startup log, a camera clip, telemetry, and the reason for any failed
  run. Resolve the heartbeat and unexpected-motion issues before changing the
  perception model.

**Done when:** Three clean runs reach a stable hover and complete a scan, fallback,
target lock, and operator stop without uncommanded movement.

### 2. Build a realistic perception benchmark

- Record camera clips of unmarked cars and people at several heights and view
  angles, with changes in lighting, scale, background, and partial occlusion.
- Add clips with two similar targets and no valid target. Label the desired target
  box and identity. Keep entire scenes in either training or evaluation so nearby
  frames do not leak across the split.
- Measure detection recall, false alarms, identity switches, inference time, and
  end-to-end latency on the computer that will run the agent.

**Done when:** There is a repeatable offline evaluation set and a measured HSV-marker
baseline for comparison.

### 3. Replace marker detection with description-based detection

- Evaluate a small open-vocabulary detector, starting with YOLO-World, on the
  recorded clips using prompts such as `car`, `red car`, and `person`.
- Keep the current perception interface: box, confidence, image error, and
  observation time. Add the selected description without changing MAVLink control.
- If prompt-based inference is too slow or inaccurate for the available hardware,
  use its results to label data and train a compact detector for the demo's limited
  target set. Record accuracy and latency before selecting either path.

**Done when:** `--target-description` selects an unmarked target in recorded clips
and in Gazebo at a measured rate suitable for the 10 Hz control loop.

### 4. Keep the same target and handle uncertainty

- Associate detections across frames using position, motion, and appearance so a
  second similar car does not silently become the target.
- Expose `RASTER`, `MOVE`, `LOCK`, `TRACK`, and `LOST` with confidence and last-seen time.
  On uncertain identity or stale observations, hold position and require a clear
  reacquisition.
- Replay loss, occlusion, and multiple-target clips through the full perception
  pipeline before testing drone motion.

**Done when:** The system follows one selected instance through a full target loop
and reports loss instead of switching identity.

### 5. Calibrate geometry and flight behavior

- Compare commanded gimbal angles with measured gimbal pose. Calibrate camera field
  of view, image-axis signs, and marker/target height assumptions.
- Measure distance or ground offset using an available range source or a validated
  ground-plane estimate. Limit speed, search area, and observation age.
- Repeat the car and pedestrian follow runs in SITL with unmarked targets. Check
  stopping on target loss, telemetry loss, mode change, and manual stop.

**Done when:** The drone tracks both targets without relying on color markers and
holds position on every tested loss path.

### 6. Connect the LLM team and prepare hardware trials

- Expose a small local command boundary: `track(description)`, `stop`, and `status`.
  The vision/control process owns target identity and flight safety gates.
- First run the perception stack on recorded real camera video without flight
  commands. Then test with the vehicle secured and a pilot supervising. Only after
  calibration and operator-stop checks should controlled flight trials begin.

**Done when:** A description starts the proven tracker, status reports its state,
and disconnecting the LLM side does not prevent an operator stop.

## Next implementation task

Finish step 1 by calibrating camera/gimbal geometry and verifying that the drone
moves toward the target in Gazebo world coordinates, then repeat the full armed
sequence three times. The passive startup check is `./run_ai_agent.sh preflight`.
After that, collect the step 2 clips and benchmark YOLO-World before integrating a
detector into the flight loop. Reference: [YOLO-World paper](https://arxiv.org/abs/2401.17270).
