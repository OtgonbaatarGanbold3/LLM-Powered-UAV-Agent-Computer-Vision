#!/usr/bin/env python3
"""Follow a marked simulator target with gated MAVLink body-frame velocity commands."""

import argparse
from dataclasses import dataclass
from enum import Enum
import math
import sys
import threading
import time

import cv2
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil

import drone_cv


CONTROL_RATE_HZ = 10.0
ACQUIRE_FRAMES = 5
DETECTION_TIMEOUT = 0.4
MAX_ACCELERATION = 2.0
MINIMUM_ALTITUDE = 1.0
TELEMETRY_TIMEOUT = 2.5
CAMERA_HORIZONTAL_FOV = 2.0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_VERTICAL_FOV = 2.0 * math.atan(
    math.tan(CAMERA_HORIZONTAL_FOV / 2.0) * CAMERA_HEIGHT / CAMERA_WIDTH
)
CAMERA_POSE_TOPIC = "/world/iris_runway/dynamic_pose/info"
CAMERA_POSE_TIMEOUT = 0.3
MAX_TARGET_RANGE = 25.0


def quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def rotate_vector(q, vector):
    w, x, y, z = q
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def pose_quaternion(pose):
    return pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z


def compose_pose(position, orientation, child):
    translated = rotate_vector(
        orientation, (child.position.x, child.position.y, child.position.z)
    )
    return (
        tuple(position[i] + translated[i] for i in range(3)),
        quaternion_multiply(orientation, pose_quaternion(child)),
    )


def camera_mount_quaternion():
    # SDF camera pose on pitch_link: roll=-1.57, pitch=-1.57, yaw=0.
    roll, pitch = -1.57 / 2.0, -1.57 / 2.0
    return (
        math.cos(roll) * math.cos(pitch),
        math.sin(roll) * math.cos(pitch),
        math.cos(roll) * math.sin(pitch),
        -math.sin(roll) * math.sin(pitch),
    )


CAMERA_MOUNT_QUATERNION = camera_mount_quaternion()


@dataclass(frozen=True)
class CameraPose:
    position: tuple
    orientation: tuple
    observed_at: float


def camera_pose_from_links(vehicle_pose, gimbal_pose, pitch_pose, now):
    position, orientation = compose_pose(
        (vehicle_pose.position.x, vehicle_pose.position.y, vehicle_pose.position.z),
        pose_quaternion(vehicle_pose),
        gimbal_pose,
    )
    position, orientation = compose_pose(position, orientation, pitch_pose)
    return CameraPose(
        position,
        quaternion_multiply(orientation, CAMERA_MOUNT_QUATERNION),
        now,
    )


def target_body_offset(detection, camera_pose, vehicle_yaw, target_height):
    """Intersect the detected image ray with the target-height plane."""
    camera_ray = (
        1.0,
        -detection.error_x * math.tan(CAMERA_HORIZONTAL_FOV / 2.0),
        -detection.error_y * math.tan(CAMERA_VERTICAL_FOV / 2.0),
    )
    world_ray = rotate_vector(camera_pose.orientation, camera_ray)
    if world_ray[2] >= -0.05:
        return None
    distance = (target_height - camera_pose.position[2]) / world_ray[2]
    if distance <= 0 or distance * math.sqrt(sum(v * v for v in world_ray)) > MAX_TARGET_RANGE:
        return None

    # iris_with_gimbal.sdf maps Gazebo +Y to NED north and +X to NED east.
    north, east = distance * world_ray[1], distance * world_ray[0]
    heading = math.radians(vehicle_yaw)
    forward = math.cos(heading) * north + math.sin(heading) * east
    right = -math.sin(heading) * north + math.cos(heading) * east
    return forward, right


class FollowState(Enum):
    IDLE = "IDLE"
    ACQUIRE = "ACQUIRE"
    TRACK = "TRACK"
    LOST = "LOST"


class SearchState(Enum):
    IDLE = "IDLE"
    RASTER = "RASTER"
    MOVE = "MOVE"
    LOCK = "LOCK"
    TRACK = "TRACK"
    LOST = "LOST"


@dataclass(frozen=True)
class VelocityCommand:
    forward: float = 0.0
    right: float = 0.0


@dataclass(frozen=True)
class SearchOutput:
    state: SearchState
    velocity: VelocityCommand
    position_target: tuple = None
    gimbal_pitch: float = -85.0
    gimbal_yaw: float = 0.0
    target_forward: float = 0.0
    target_right: float = 0.0


class SearchController:
    """Coordinate vertical gimbal search, bounded orbit, lock, and follow."""

    def __init__(
        self,
        target,
        max_speed=2.0,
        search_radius=1.5,
        search_speed=0.2,
        search_timeout=90.0,
        scan_pitch_min=-85.0,
        scan_pitch_max=-45.0,
        scan_pitch_step=20.0,
        scan_yaw_min=-60.0,
        scan_yaw_max=60.0,
        scan_rate=30.0,
        follow_motion_enabled=True,
        follow_distance=2.5,
    ):
        self.target = target
        self.max_speed = max_speed
        self.search_radius = search_radius
        self.search_speed = search_speed
        self.search_timeout = search_timeout
        self.scan_pitch_min = scan_pitch_min
        self.scan_pitch_max = scan_pitch_max
        self.scan_pitch_step = scan_pitch_step
        self.scan_yaw_min = scan_yaw_min
        self.scan_yaw_max = scan_yaw_max
        self.scan_rate = scan_rate
        self.follow_motion_enabled = follow_motion_enabled
        self.follow_distance = follow_distance
        self.geometry_error = None
        self.state = SearchState.RASTER
        self.enabled = True
        self.origin = None
        self.motion_started = None
        self.search_motion = True
        self.raster_complete = False
        self.raster_axis = "yaw"
        self.yaw_direction = 1.0
        self.pitch_direction = 1.0
        self.pitch_target = scan_pitch_min
        self.pitch = scan_pitch_min
        self.yaw = scan_yaw_min
        self.last_observation = -1
        self.last_detection = None
        self.stable_frames = 0
        self.centered_frames = 0
        self.command = VelocityCommand()

    def toggle(self):
        self.enabled = not self.enabled
        self.command = VelocityCommand()
        if not self.enabled:
            self.state = SearchState.IDLE
        if self.enabled:
            self.state = SearchState.RASTER
            self.origin = None
            self.motion_started = None
            self.search_motion = True
            self.raster_complete = False
            self.raster_axis = "yaw"
            self.yaw_direction = 1.0
            self.pitch_direction = 1.0
            self.pitch_target = self.scan_pitch_min
            self.pitch = self.scan_pitch_min
            self.yaw = self.scan_yaw_min

    def step(
        self,
        detection,
        observation,
        observation_time,
        now,
        elapsed,
        vehicle_ready,
        position,
        vehicle_yaw,
        altitude,
        camera_pose=None,
    ):
        if not self.enabled or not vehicle_ready:
            self.command = VelocityCommand()
            if self.enabled:
                self.state = SearchState.RASTER
                self.origin = None
                self.motion_started = None
                self.search_motion = True
                self.raster_complete = False
                self.raster_axis = "yaw"
                self.yaw_direction = 1.0
                self.pitch_direction = 1.0
                self.pitch_target = self.scan_pitch_min
                self.pitch = self.scan_pitch_min
                self.yaw = self.scan_yaw_min
            return self._output()

        if position is None or vehicle_yaw is None or altitude is None:
            self.command = VelocityCommand()
            return self._output()

        if self.origin is None:
            self.origin = (position[0], position[1], position[2])

        if detection is not None and observation != self.last_observation:
            self.last_detection = detection
        current_detection = detection if detection is not None else self.last_detection
        fresh = (
            current_detection is not None
            and now - observation_time <= DETECTION_TIMEOUT
        )
        if self.state is SearchState.LOST:
            self.state = (
                SearchState.LOCK
                if fresh
                else SearchState.MOVE
                if self.raster_complete
                else SearchState.RASTER
            )
            self.stable_frames = 0
            self.centered_frames = 0
            if not fresh:
                self.search_motion = True
                self.last_detection = None

        new_observation = observation != self.last_observation
        if self.state in (SearchState.RASTER, SearchState.MOVE):
            if fresh:
                self.state = SearchState.LOCK
                self.last_observation = observation
                self.stable_frames = 1
                self.centered_frames = 0
                self.command = VelocityCommand()
                self._aim_gimbal(current_detection, elapsed)
                return self._output()
            self._scan_gimbal(elapsed)
            if self.raster_complete and self.state is SearchState.RASTER:
                self.state = SearchState.MOVE
                self.motion_started = now
            target = self._orbit_target(now, position)
            self.command = VelocityCommand()
            return self._output(position_target=target)

        if self.state in (SearchState.LOCK, SearchState.TRACK):
            if not fresh:
                self.state = SearchState.LOST
                self.command = VelocityCommand()
                self.stable_frames = 0
                self.centered_frames = 0
                return self._output()

            if new_observation:
                self.last_observation = observation
                if self.state is SearchState.LOCK and detection is not None:
                    self.stable_frames += 1
                if detection is not None:
                    if abs(detection.error_x) <= 0.05 and abs(detection.error_y) <= 0.05:
                        self.centered_frames += 1
                    else:
                        self.centered_frames = 0

            if new_observation and detection is not None:
                self._aim_gimbal(detection, elapsed)
            self.search_motion = False
            if self.state is SearchState.LOCK:
                self.command = VelocityCommand()
                if self.stable_frames >= ACQUIRE_FRAMES and self.centered_frames >= 10:
                    self.state = SearchState.TRACK
                return self._output()

            if not self.follow_motion_enabled:
                self.geometry_error = None
                self.command = VelocityCommand()
                return self._output()

            if camera_pose is None or now - camera_pose.observed_at > CAMERA_POSE_TIMEOUT:
                self.geometry_error = "camera pose unavailable"
                self.command = VelocityCommand()
                return self._output()
            if detection is None or now - observation_time > 0.2:
                self.geometry_error = "target detection stale"
                self.command = VelocityCommand()
                return self._output()

            marker_height = 1.78 if self.target == "person" else 0.405
            offset = target_body_offset(detection, camera_pose, vehicle_yaw, marker_height)
            if offset is None:
                self.geometry_error = "target ray misses ground"
                self.command = VelocityCommand()
                return self._output()
            self.geometry_error = None
            forward, right = offset
            distance = math.hypot(forward, right)
            speed = min(self.max_speed, 0.6 * max(0.0, distance - self.follow_distance))
            desired_forward = speed * forward / distance if distance else 0.0
            desired_right = speed * right / distance if distance else 0.0
            max_change = MAX_ACCELERATION * max(elapsed, 0.0)
            self.command = VelocityCommand(
                self._slew_angle(self.command.forward, desired_forward, max_change),
                self._slew_angle(self.command.right, desired_right, max_change),
            )
            return self._output(
                target_forward=forward,
                target_right=right,
            )

        self.command = VelocityCommand()
        return self._output()

    def _aim_gimbal(self, detection, elapsed):
        pitch_correction = -detection.error_y * math.degrees(CAMERA_VERTICAL_FOV / 2.0)
        yaw_correction = detection.error_x * math.degrees(CAMERA_HORIZONTAL_FOV / 2.0)
        self.pitch = self._slew_angle(
            self.pitch,
            max(-90.0, min(45.0, self.pitch + pitch_correction)),
            20.0 * elapsed,
        )
        self.yaw = self._slew_angle(
            self.yaw,
            max(-160.0, min(160.0, self.yaw + yaw_correction)),
            25.0 * elapsed,
        )

    def _scan_gimbal(self, elapsed):
        step = self.scan_rate * max(elapsed, 0.0)
        if self.raster_axis == "yaw":
            self.yaw += self.yaw_direction * step
            if self.yaw_direction > 0.0 and self.yaw >= self.scan_yaw_max:
                self.yaw = self.scan_yaw_max
                self.yaw_direction = -1.0
            elif self.yaw_direction < 0.0 and self.yaw <= self.scan_yaw_min:
                self.yaw = self.scan_yaw_min
                self.yaw_direction = 1.0
            else:
                return

            if self.pitch >= self.scan_pitch_max:
                self.raster_complete = True
                self.pitch_direction = -1.0
            elif self.pitch <= self.scan_pitch_min:
                self.pitch_direction = 1.0
            self.pitch_target = max(
                self.scan_pitch_min,
                min(
                    self.scan_pitch_max,
                    self.pitch + self.pitch_direction * self.scan_pitch_step,
                ),
            )
            if self.pitch_target != self.pitch:
                self.raster_axis = "pitch"
        else:
            difference = self.pitch_target - self.pitch
            self.pitch += max(-step, min(step, difference))
            if abs(self.pitch_target - self.pitch) < 1e-6:
                self.raster_axis = "yaw"

    def _orbit_target(self, now, position):
        if not self.search_motion:
            return None
        if not self.search_motion or self.motion_started is None:
            return None
        elapsed = now - self.motion_started
        if elapsed >= self.search_timeout:
            self.search_motion = False
            return None
        distance = math.hypot(position[0] - self.origin[0], position[1] - self.origin[1])
        if distance >= self.search_radius + 1.0:
            self.search_motion = False
            return None
        angle = self.search_speed * elapsed / self.search_radius
        radius = min(self.search_radius, self.search_speed * elapsed)
        return (
            self.origin[0] + radius * math.cos(angle),
            self.origin[1] + radius * math.sin(angle),
            self.origin[2],
        )

    @staticmethod
    def _slew_angle(current, target, max_change):
        return current + max(-max_change, min(max_change, target - current))

    def _output(
        self,
        position_target=None,
        target_forward=0.0,
        target_right=0.0,
    ):
        return SearchOutput(
            state=self.state,
            velocity=self.command,
            position_target=position_target,
            gimbal_pitch=self.pitch,
            gimbal_yaw=self.yaw,
            target_forward=target_forward,
            target_right=target_right,
        )


class FollowController:
    def __init__(
        self,
        acquire_frames=ACQUIRE_FRAMES,
        detection_timeout=DETECTION_TIMEOUT,
    ):
        self.acquire_frames = acquire_frames
        self.detection_timeout = detection_timeout
        self.state = FollowState.IDLE
        self.enabled = True
        self.stable_frames = 0
        self.last_observation = -1

    def toggle(self):
        self.enabled = not self.enabled
        if not self.enabled:
            self.state = FollowState.IDLE
            self.stable_frames = 0

    def step(
        self,
        detection,
        observation,
        observation_time,
        now,
        vehicle_ready,
    ):
        if not self.enabled or not vehicle_ready:
            self.last_observation = observation
            self.stable_frames = 0
            self.state = FollowState.IDLE
            return VelocityCommand()

        if self.state is FollowState.IDLE:
            self.state = FollowState.ACQUIRE

        if observation != self.last_observation:
            self.last_observation = observation
            if detection is None:
                if self.state is not FollowState.TRACK:
                    self.stable_frames = 0
            else:
                if self.state is not FollowState.TRACK:
                    self.stable_frames += 1
                    if self.stable_frames >= self.acquire_frames:
                        self.state = FollowState.TRACK

        if now - observation_time > self.detection_timeout:
            self.stable_frames = 0
            self.state = FollowState.LOST

        return VelocityCommand()


class VehicleStatus:
    def __init__(self):
        self.armed = False
        self.mode = "UNKNOWN"
        self.relative_altitude = None
        self.local_position = None
        self.yaw = None
        self.last_heartbeat = 0.0
        self.last_position = 0.0
        self.last_local_position = 0.0
        self.last_attitude = 0.0

    def observe(self, message, now):
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            self.armed = bool(
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.mode = mavutil.mode_string_v10(message)
            self.last_heartbeat = now
        elif message_type == "GLOBAL_POSITION_INT":
            self.relative_altitude = message.relative_alt / 1000.0
            self.last_position = now
        elif message_type == "LOCAL_POSITION_NED":
            self.local_position = (message.x, message.y, message.z)
            self.last_local_position = now
        elif message_type == "ATTITUDE":
            self.yaw = math.degrees(message.yaw)
            self.last_attitude = now

    def drain(self, connection, now):
        while True:
            message = connection.recv_match(blocking=False)
            if message is None:
                return
            self.observe(message, now)

    def readiness(self, now, require_local=False):
        if now - self.last_heartbeat > TELEMETRY_TIMEOUT:
            return False, "heartbeat stale"
        if not self.armed:
            return False, "vehicle disarmed"
        if self.mode != "GUIDED":
            return False, f"mode {self.mode}"
        if self.relative_altitude is None or now - self.last_position > TELEMETRY_TIMEOUT:
            return False, "altitude unavailable"
        if self.relative_altitude < MINIMUM_ALTITUDE:
            return False, f"altitude {self.relative_altitude:.1f} m"
        if require_local and (
            self.local_position is None
            or now - self.last_local_position > TELEMETRY_TIMEOUT
            or self.yaw is None
            or now - self.last_attitude > TELEMETRY_TIMEOUT
        ):
            return False, "local position or heading unavailable"
        return True, "ready"


def velocity_type_mask():
    return (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )


def send_velocity(connection, command):
    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        velocity_type_mask(),
        0.0,
        0.0,
        0.0,
        command.forward,
        command.right,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_local_position(connection, position):
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        position[0],
        position[1],
        position[2],
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_gimbal_angle(connection, pitch, yaw):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,
        pitch,
        0.0,
        yaw,
        0.0,
        0.0,
        0.0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
    )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=drone_cv.MARKERS, default="car")
    parser.add_argument("--topic", default=drone_cv.DEFAULT_CAMERA_TOPIC)
    parser.add_argument(
        "--connect",
        default="udpin:127.0.0.1:14551",
        help="pymavlink connection string (default: %(default)s)",
    )
    parser.add_argument("--max-speed", type=float, default=1.0)
    parser.add_argument(
        "--no-follow-motion",
        action="store_true",
        help="search and point the gimbal without pursuing the target",
    )
    parser.add_argument(
        "--max-flight-radius",
        type=float,
        default=12.0,
        help="stop movement this far from the follower's starting point (default: %(default)s m)",
    )
    parser.add_argument(
        "--search",
        action="store_true",
        help="raster-scan with the gimbal, then move slowly if the target is not found",
    )
    parser.add_argument("--search-radius", type=float, default=1.5)
    parser.add_argument("--search-speed", type=float, default=0.2)
    parser.add_argument("--search-timeout", type=float, default=90.0)
    parser.add_argument("--scan-pitch-min", type=float, default=-85.0)
    parser.add_argument("--scan-pitch-max", type=float, default=-45.0)
    parser.add_argument("--scan-pitch-step", type=float, default=20.0)
    parser.add_argument("--scan-yaw-min", type=float, default=-60.0)
    parser.add_argument("--scan-yaw-max", type=float, default=60.0)
    parser.add_argument("--scan-rate", type=float, default=30.0)
    parser.add_argument("--min-area", type=float)
    parser.add_argument("--frame-timeout", type=float, default=10.0)
    parser.add_argument("--heartbeat-timeout", type=float, default=15.0)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; zero runs until q or Ctrl+C",
    )
    parser.add_argument("--no-display", action="store_true")
    return parser.parse_args()


def validate_args(args):
    if any(
        not math.isfinite(value) or value <= 0
        for value in (args.max_speed, args.max_flight_radius)
    ):
        return "max speed and flight radius must be positive"
    if args.min_area is not None and (not math.isfinite(args.min_area) or args.min_area <= 0):
        return "--min-area must be positive"
    if any(
        not math.isfinite(value) or value <= 0
        for value in (args.frame_timeout, args.heartbeat_timeout)
    ):
        return "timeouts must be positive"
    if not math.isfinite(args.duration) or args.duration < 0:
        return "--duration cannot be negative"
    if any(
        not math.isfinite(value) or value <= 0
        for value in (args.search_radius, args.search_speed, args.search_timeout)
    ):
        return "search radius, speed, and timeout must be positive"
    if any(
        not math.isfinite(value) or value <= 0
        for value in (args.scan_rate, args.scan_pitch_step)
    ):
        return "scan rate and pitch step must be positive"
    if not all(
        math.isfinite(value)
        for value in (args.scan_pitch_min, args.scan_pitch_max, args.scan_yaw_min, args.scan_yaw_max)
    ):
        return "scan limits must be finite"
    if args.scan_pitch_min >= args.scan_pitch_max:
        return "scan pitch limits must be ordered"
    if not -90.0 <= args.scan_pitch_min < args.scan_pitch_max <= 45.0:
        return "scan pitch limits must stay within the simulated gimbal range"
    if not -160.0 <= args.scan_yaw_min < args.scan_yaw_max <= 160.0:
        return "scan yaw limits must be ordered and stay within the simulated gimbal range"
    return None


def draw_follow_overlay(frame, controller, vehicle_status, gate_reason, command):
    color = (0, 255, 0) if controller.state is FollowState.TRACK else (0, 165, 255)
    cv2.putText(
        frame,
        f"control={controller.state.value} enabled={controller.enabled} gate={gate_reason}",
        (10, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
    )
    cv2.putText(
        frame,
        f"cmd forward={command.forward:+.2f} right={command.right:+.2f} "
        f"alt={vehicle_status.relative_altitude or 0.0:.1f}m | s: stop/start",
        (10, 106),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
    )


def draw_search_overlay(frame, output, vehicle_status, gate_reason):
    color = (0, 255, 0) if output.state is SearchState.TRACK else (0, 165, 255)
    cv2.putText(
        frame,
        f"search={output.state.value} gate={gate_reason}",
        (10, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
    )
    motion = (
        f"orbit N/E=({output.position_target[0]:+.1f},"
        f"{output.position_target[1]:+.1f})m"
        if output.position_target is not None
        else f"velocity=({output.velocity.forward:+.2f},"
        f"{output.velocity.right:+.2f})m/s target offset="
        f"({output.target_forward:+.1f},{output.target_right:+.1f})m"
    )
    cv2.putText(
        frame,
        f"gimbal pitch={output.gimbal_pitch:+.1f} yaw={output.gimbal_yaw:+.1f} "
        f"{motion}",
        (10, 106),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        2,
    )
    cv2.putText(
        frame,
        f"alt={vehicle_status.relative_altitude or 0.0:.1f}m | s: stop/start",
        (10, 130),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
    )


def main():
    args = parse_args()
    argument_error = validate_args(args)
    if argument_error:
        print(f"Error: {argument_error}.", file=sys.stderr)
        return 2

    print(f"Connecting to ArduPilot on {args.connect}...", flush=True)
    try:
        connection = mavutil.mavlink_connection(args.connect)
        heartbeat = connection.wait_heartbeat(timeout=args.heartbeat_timeout)
    except (OSError, ValueError) as error:
        print(f"Error: could not open MAVLink connection: {error}", file=sys.stderr)
        return 1
    if heartbeat is None:
        print(
            f"Error: no ArduPilot heartbeat within {args.heartbeat_timeout:g} seconds "
            f"on {args.connect}. Check that ./run_ardupilot.sh is running in an "
            "interactive terminal and MAVProxy reports a connected vehicle.",
            file=sys.stderr,
        )
        connection.close()
        return 1

    vehicle = VehicleStatus()
    vehicle.observe(heartbeat, time.monotonic())
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        5,
        1,
    )
    if args.search:
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            5,
            1,
        )

    marker = drone_cv.MARKERS[args.target]
    frame_condition = threading.Condition()
    first_frame = threading.Event()
    latest_frame = None
    frame_sequence = 0
    camera_frames = 0
    last_camera_frame_at = time.monotonic()
    format_error = []
    camera_pose_lock = threading.Lock()
    latest_camera_pose = None

    def image_callback(message):
        nonlocal latest_frame, frame_sequence, camera_frames, last_camera_frame_at
        try:
            frame_bgr = drone_cv.decode_rgb_image(message)
        except ValueError as error:
            if not format_error:
                format_error.append(str(error))
                first_frame.set()
            return
        with frame_condition:
            latest_frame = frame_bgr
            frame_sequence += 1
            camera_frames += 1
            last_camera_frame_at = time.monotonic()
            frame_condition.notify()
        first_frame.set()

    def pose_callback(message):
        nonlocal latest_camera_pose
        now = time.monotonic()
        poses = {pose.name: pose for pose in message.pose}
        if all(name in poses for name in ("iris_with_gimbal", "gimbal", "pitch_link")):
            camera_pose = camera_pose_from_links(
                poses["iris_with_gimbal"], poses["gimbal"], poses["pitch_link"], now
            )
            with camera_pose_lock:
                latest_camera_pose = camera_pose

    node = Node()
    if not node.subscribe(GzImage, args.topic, image_callback):
        print("Error: Gazebo rejected the camera subscription.", file=sys.stderr)
        connection.close()
        return 1
    if args.search and not node.subscribe(Pose_V, CAMERA_POSE_TOPIC, pose_callback):
        print("Error: Gazebo rejected the camera-pose subscription.", file=sys.stderr)
        connection.close()
        return 1
    if not first_frame.wait(timeout=args.frame_timeout):
        print(
            f"Error: no camera frame received within {args.frame_timeout:g} seconds.",
            file=sys.stderr,
        )
        connection.close()
        return 1
    if format_error:
        print(f"Error: unsupported camera format: {format_error[0]}", file=sys.stderr)
        connection.close()
        return 1

    fixed_controller = FollowController()
    search_controller = SearchController(
        target=args.target,
        max_speed=args.max_speed,
        search_radius=args.search_radius,
        search_speed=args.search_speed,
        search_timeout=args.search_timeout,
        scan_pitch_min=args.scan_pitch_min,
        scan_pitch_max=args.scan_pitch_max,
        scan_pitch_step=args.scan_pitch_step,
        scan_yaw_min=args.scan_yaw_min,
        scan_yaw_max=args.scan_yaw_max,
        scan_rate=args.scan_rate,
        follow_motion_enabled=not args.no_follow_motion,
    )
    controller = search_controller if args.search else fixed_controller
    memory = drone_cv.TargetMemory(max_missed_frames=5)
    latest_detection = None
    latest_detection_time = time.monotonic()
    processed_sequence = 0
    processed_frames = 0
    detected_frames = 0
    tracking_frames = 0
    centered_frames = 0
    error_x_total = 0.0
    error_y_total = 0.0
    command_count = 0
    nonzero_commands = 0
    position_targets = 0
    flight_origin = None
    flight_radius_hit = False
    latest_display = None
    perception_status = "SEARCHING"
    gate_reason = "initializing"
    last_gate_reason = None
    command = VelocityCommand()
    search_output = search_controller._output()
    started_at = time.monotonic()
    previous_control = started_at
    next_control = started_at
    next_report = started_at + 5.0
    exit_code = 0

    action = (
        "Search mode enabled (--search): gimbal raster first, drone movement only if needed"
        if args.search
        else "Search mode disabled: fixed-camera target tracking"
    )
    print(
        f"{action}; target={args.target}, max follow speed={args.max_speed:g} m/s, "
        f"flight radius={args.max_flight_radius:g} m, "
        f"active pursuit={'on' if args.search and not args.no_follow_motion else 'off'}. "
        "Press s to stop/start tracking, q to quit, or Ctrl+C.",
        flush=True,
    )
    try:
        while True:
            now = time.monotonic()
            if args.duration and now - started_at >= args.duration:
                break
            if format_error:
                print(f"Error: unsupported camera format: {format_error[0]}", file=sys.stderr)
                exit_code = 1
                break
            if now - last_camera_frame_at >= args.frame_timeout:
                print(
                    f"Error: camera stream stopped for {args.frame_timeout:g} seconds.",
                    file=sys.stderr,
                )
                exit_code = 1
                break

            with frame_condition:
                if frame_sequence > processed_sequence:
                    frame = latest_frame.copy()
                    processed_sequence = frame_sequence
                else:
                    frame = None

            if frame is not None:
                reference = (
                    search_controller.last_detection
                    if args.search and search_controller.state in (SearchState.LOCK, SearchState.TRACK)
                    else None
                )
                latest_detection = drone_cv.detect_marker(
                    frame, marker, args.min_area, reference=reference
                )
                if latest_detection is not None:
                    latest_detection_time = now
                remembered_detection, perception_status = memory.update(latest_detection)
                processed_frames += 1
                if latest_detection is not None:
                    detected_frames += 1
                elapsed = max(now - started_at, 1e-6)
                drone_cv.draw_tracking_overlay(
                    frame,
                    args.target,
                    marker,
                    remembered_detection,
                    perception_status,
                    camera_frames / elapsed,
                    processed_frames / elapsed,
                    memory.loss_count,
                )
                latest_display = frame

            vehicle.drain(connection, now)
            if now >= next_control:
                ready, gate_reason = vehicle.readiness(now, require_local=True)
                if ready and flight_origin is None:
                    flight_origin = vehicle.local_position[:2]
                if (
                    ready
                    and not flight_radius_hit
                    and math.dist(vehicle.local_position[:2], flight_origin) >= args.max_flight_radius
                ):
                    flight_radius_hit = True
                    if controller.enabled:
                        controller.toggle()
                if flight_radius_hit:
                    ready = False
                    gate_reason = f"flight radius {args.max_flight_radius:g} m reached; restart required"
                if gate_reason != last_gate_reason:
                    print(f"vehicle gate: {gate_reason}", flush=True)
                    last_gate_reason = gate_reason
                previous_state = controller.state
                if args.search:
                    with camera_pose_lock:
                        camera_pose = latest_camera_pose
                    search_output = search_controller.step(
                        latest_detection,
                        processed_sequence,
                        latest_detection_time,
                        now,
                        now - previous_control,
                        ready,
                        vehicle.local_position,
                        vehicle.yaw,
                        vehicle.relative_altitude,
                        camera_pose,
                    )
                    command = search_output.velocity
                    if ready:
                        if search_output.position_target is not None:
                            send_local_position(connection, search_output.position_target)
                            position_targets += 1
                        elif search_output.state is SearchState.TRACK:
                            send_velocity(connection, command)
                        else:
                            send_local_position(connection, vehicle.local_position)
                        send_gimbal_angle(
                            connection,
                            search_output.gimbal_pitch,
                            search_output.gimbal_yaw,
                        )
                    elif vehicle.armed and now - vehicle.last_heartbeat <= TELEMETRY_TIMEOUT:
                        send_velocity(connection, VelocityCommand())
                else:
                    command = fixed_controller.step(
                        latest_detection,
                        processed_sequence,
                        latest_detection_time,
                        now,
                        ready,
                    )
                    if ready or (vehicle.armed and now - vehicle.last_heartbeat <= TELEMETRY_TIMEOUT):
                        send_velocity(connection, command)
                if controller.state is not previous_state:
                    print(
                        f"control {previous_state.value}->{controller.state.value} "
                        f"cmd=({command.forward:+.2f},{command.right:+.2f})",
                        flush=True,
                    )
                command_count += 1
                if command != VelocityCommand():
                    nonzero_commands += 1
                previous_control = now
                next_control += 1.0 / CONTROL_RATE_HZ
                if next_control <= now:
                    next_control = now + 1.0 / CONTROL_RATE_HZ

            if (
                frame is not None
                and latest_detection is not None
                and controller.state is (
                    SearchState.TRACK if args.search else FollowState.TRACK
                )
            ):
                tracking_frames += 1
                error_x_total += abs(latest_detection.error_x)
                error_y_total += abs(latest_detection.error_y)
                if (
                    abs(latest_detection.error_x) <= 0.1
                    and abs(latest_detection.error_y) <= 0.1
                ):
                    centered_frames += 1

            if frame is not None and latest_display is not None:
                if args.search:
                    draw_search_overlay(latest_display, search_output, vehicle, gate_reason)
                else:
                    draw_follow_overlay(latest_display, controller, vehicle, gate_reason, command)
                if not args.no_display:
                    title = (
                        "Drone Search and Follow"
                        if args.search
                        else "Drone Fixed-Camera Follower"
                    )
                    cv2.imshow(title, latest_display)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break
                    if key == ord("s"):
                        controller.toggle()
                        if (
                            args.search
                            and ready
                            and not controller.enabled
                            and vehicle.local_position is not None
                            and time.monotonic() - vehicle.last_local_position
                            <= TELEMETRY_TIMEOUT
                        ):
                            send_local_position(connection, vehicle.local_position)
                        elif not controller.enabled and vehicle.armed:
                            send_velocity(connection, VelocityCommand())

            if args.no_display and now >= next_report:
                error = (
                    "none"
                    if latest_detection is None
                    else f"({latest_detection.error_x:+.2f},{latest_detection.error_y:+.2f})"
                )
                print(
                    f"state={controller.state.value} gate={gate_reason} "
                    f"error={error} cmd=({command.forward:+.2f},{command.right:+.2f})"
                    + (
                        f" gimbal=({search_output.gimbal_pitch:+.1f},"
                        f"{search_output.gimbal_yaw:+.1f})"
                        f" offset=({search_output.target_forward:+.1f},"
                        f"{search_output.target_right:+.1f})"
                        f" geometry={search_controller.geometry_error or 'ready'}"
                        + (
                            f" goal=({search_output.position_target[0]:+.2f},"
                            f"{search_output.position_target[1]:+.2f})"
                            if search_output.position_target is not None
                            else ""
                        )
                        if args.search
                        else ""
                    ),
                    flush=True,
                )
                next_report += 5.0

            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            if (
                args.search
                and vehicle.armed
                and vehicle.local_position is not None
                and time.monotonic() - vehicle.last_local_position <= TELEMETRY_TIMEOUT
            ):
                send_local_position(connection, vehicle.local_position)
            elif vehicle.armed:
                send_velocity(connection, VelocityCommand())
            time.sleep(0.05)
        connection.close()
        if not args.no_display:
            cv2.destroyAllWindows()

    elapsed = max(time.monotonic() - started_at, 1e-6)
    centered_percent = 100.0 * centered_frames / tracking_frames if tracking_frames else 0.0
    mean_error_x = error_x_total / tracking_frames if tracking_frames else 0.0
    mean_error_y = error_y_total / tracking_frames if tracking_frames else 0.0
    print(
        "Follow summary: "
        f"target={args.target} camera_rate={camera_frames / elapsed:.2f}Hz "
        f"processed_rate={processed_frames / elapsed:.2f}Hz "
        f"detections={detected_frames}/{processed_frames} losses={memory.loss_count} "
        f"tracking_frames={tracking_frames} centered={centered_percent:.1f}% "
        f"mean_abs_error=({mean_error_x:.3f},{mean_error_y:.3f}) "
        f"commands={command_count} nonzero_velocity={nonzero_commands} "
        f"position_targets={position_targets}",
        flush=True,
    )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
