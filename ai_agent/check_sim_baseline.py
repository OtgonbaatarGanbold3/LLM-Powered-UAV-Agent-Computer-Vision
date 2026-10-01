#!/usr/bin/env python3
"""Passively check SITL telemetry, camera frames, and Gazebo startup motion."""

import argparse
import math
import threading
import time

from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil

import drone_cv


POSE_TOPIC = "/world/iris_runway/pose/info"
NAMES = ("iris_with_gimbal", "target_car", "target_person")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default="udpin:127.0.0.1:14551")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--max-drift", type=float, default=0.1)
    args = parser.parse_args()
    if args.duration <= 0 or args.max_drift <= 0:
        parser.error("duration and max-drift must be positive")

    connection = mavutil.mavlink_connection(args.connect)
    try:
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            print(f"FAIL: no MAVLink heartbeat on {args.connect}")
            return 1
        print(f"MAVLink: system {connection.target_system}, mode {mavutil.mode_string_v10(heartbeat)}")

        lock = threading.Lock()
        poses = {}
        frames = 0

        def on_pose(message):
            with lock:
                for pose in message.pose:
                    if pose.name not in NAMES:
                        continue
                    point = (pose.position.x, pose.position.y, pose.position.z)
                    record = poses.setdefault(pose.name, [point, 0.0, 0])
                    record[1] = max(record[1], math.dist(record[0], point))
                    record[2] += 1

        def on_image(_message):
            nonlocal frames
            with lock:
                frames += 1

        node = Node()
        if not node.subscribe(Pose_V, POSE_TOPIC, on_pose):
            print(f"FAIL: could not subscribe to {POSE_TOPIC}")
            return 1
        if not node.subscribe(Image, drone_cv.DEFAULT_CAMERA_TOPIC, on_image):
            print("FAIL: could not subscribe to the drone camera")
            return 1

        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        last_heartbeat = time.monotonic()
        deadline = last_heartbeat + args.duration
        while time.monotonic() < deadline:
            message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if message is not None:
                last_heartbeat = time.monotonic()
                armed |= bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        with lock:
            results = {name: (first, drift, count) for name, (first, drift, count) in poses.items()}
            camera_frames = frames
        drone = results.get("iris_with_gimbal")
        ok = not armed and time.monotonic() - last_heartbeat <= 3.0
        if drone is None:
            print("FAIL: drone pose unavailable")
            ok = False
        else:
            start, drift, count = drone
            print(f"drone: start={start} max_drift={drift:.3f}m samples={count}")
            ok &= count >= 2 and drift <= args.max_drift
            ok &= math.dist(start, (0.0, 0.0, 0.195)) <= 0.5
        for name in NAMES[1:]:
            record = results.get(name)
            if record is None:
                print(f"FAIL: {name} pose unavailable")
                ok = False
            else:
                print(f"{name}: max_motion={record[1]:.3f}m samples={record[2]}")
                ok &= record[2] >= 2 and record[1] >= 0.05
        print(f"camera_frames={camera_frames} armed={armed} heartbeat_age={time.monotonic() - last_heartbeat:.1f}s")
        ok &= camera_frames > 0
        print("PASS: disarmed simulator baseline" if ok else "FAIL: simulator baseline")
        return 0 if ok else 1
    finally:
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
