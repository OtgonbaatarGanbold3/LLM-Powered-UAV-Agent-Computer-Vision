#!/usr/bin/env python3
"""Check Gazebo target world poses without using the drone camera."""

import argparse
import math
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


TARGETS = {
    "car": ("target_car", "/world/iris_runway/model/target_car/pose"),
    "person": ("target_person", "/world/iris_runway/model/target_person/pose"),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=("car", "person", "both"), default="both")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    requested = TARGETS.values() if args.target == "both" else (TARGETS[args.target],)
    poses = {}
    lock = threading.Lock()

    node = Node()
    for name, topic in requested:
        def on_pose(message, target_name=name):
            with lock:
                for pose in message.pose:
                    if pose.name == target_name or pose.name.endswith(f"::{target_name}"):
                        position = pose.position
                        sample = (position.x, position.y, position.z)
                        record = poses.setdefault(
                            target_name, {"first": sample, "last": sample}
                        )
                        record["last"] = sample

        if not node.subscribe(Pose_V, topic, on_pose):
            print(f"Gazebo rejected pose subscription: {topic}")
            return 1

    print(f"Checking Gazebo world poses for {args.duration:g}s (camera-independent).")
    time.sleep(args.duration)

    failed = False
    with lock:
        for name, _topic in requested:
            record = poses.get(name)
            if record is None:
                print(f"{name}: NOT FOUND in world pose stream")
                failed = True
                continue
            first, last = record["first"], record["last"]
            distance = math.dist(first, last)
            status = "MOVING" if distance >= 0.05 else "STATIONARY"
            print(
                f"{name}: {status} start=({first[0]:+.2f},{first[1]:+.2f},{first[2]:+.2f}) "
                f"end=({last[0]:+.2f},{last[1]:+.2f},{last[2]:+.2f}) "
                f"delta={distance:.2f}m"
            )
            failed |= status != "MOVING"
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
