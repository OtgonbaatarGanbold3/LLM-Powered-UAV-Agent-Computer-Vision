#!/usr/bin/env python3
"""Point the simulated ArduPilot camera mount through MAVLink."""

import argparse
import sys
import time

from pymavlink import mavutil


PITCH_RANGE = (-90.0, 45.0)
YAW_RANGE = (-160.0, 160.0)
TEST_SEQUENCE = (
    (0.0, 0.0),
    (-45.0, 0.0),
    (-45.0, 45.0),
    (-45.0, -45.0),
    (-90.0, 0.0),
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connect",
        default="udpin:127.0.0.1:14551",
        help="pymavlink connection string (default: %(default)s)",
    )
    parser.add_argument("--pitch", type=float, default=-90.0, help="pitch in degrees")
    parser.add_argument("--yaw", type=float, default=0.0, help="yaw in degrees")
    parser.add_argument(
        "--sequence",
        action="store_true",
        help="run the standard pitch/yaw test sequence",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=2.0,
        help="delay between sequence positions (default: %(default)s)",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for ArduPilot (default: %(default)s)",
    )
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=3.0,
        help="seconds to wait for each command acknowledgement (default: %(default)s)",
    )
    return parser.parse_args()


def check_angle(name, value, limits):
    if not limits[0] <= value <= limits[1]:
        raise ValueError(f"{name} must be between {limits[0]:g} and {limits[1]:g} degrees")


def wait_for_ack(connection, command, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ack = connection.recv_match(
            type="COMMAND_ACK",
            blocking=True,
            timeout=max(0.0, deadline - time.monotonic()),
        )
        if ack is not None and ack.command == command:
            return ack
    return None


def point_gimbal(connection, pitch, yaw, ack_timeout):
    check_angle("pitch", pitch, PITCH_RANGE)
    check_angle("yaw", yaw, YAW_RANGE)
    command = mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL

    print(f"Commanding pitch={pitch:g}°, yaw={yaw:g}°")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        command,
        0,
        pitch,
        0.0,
        yaw,
        0.0,
        0.0,
        0.0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
    )

    ack = wait_for_ack(connection, command, ack_timeout)
    if ack is None:
        print("Warning: command was sent, but no COMMAND_ACK was received.")
        return True

    result = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result)
    result_name = result.name if result else str(ack.result)
    print(f"ArduPilot acknowledgement: {result_name}")
    return ack.result in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    )


def main():
    args = parse_args()
    if args.hold_seconds < 0 or args.heartbeat_timeout <= 0 or args.ack_timeout <= 0:
        print("Timeouts must be positive and --hold-seconds cannot be negative.", file=sys.stderr)
        return 2

    positions = TEST_SEQUENCE if args.sequence else ((args.pitch, args.yaw),)
    try:
        for pitch, yaw in positions:
            check_angle("pitch", pitch, PITCH_RANGE)
            check_angle("yaw", yaw, YAW_RANGE)
    except ValueError as error:
        print(f"Error: {error}", file=sys.stderr)
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
            f"Error: no ArduPilot heartbeat received within {args.heartbeat_timeout:g} seconds.",
            file=sys.stderr,
        )
        connection.close()
        return 1

    print(
        f"Connected to system {connection.target_system}, "
        f"component {connection.target_component}."
    )
    try:
        for index, (pitch, yaw) in enumerate(positions):
            if not point_gimbal(connection, pitch, yaw, args.ack_timeout):
                return 3
            if index + 1 < len(positions):
                time.sleep(args.hold_seconds)
    finally:
        connection.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
