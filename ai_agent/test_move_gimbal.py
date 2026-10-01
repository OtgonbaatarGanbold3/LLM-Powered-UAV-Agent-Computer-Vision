#!/usr/bin/env python3

import types
import unittest

from pymavlink import mavutil

import move_gimbal


class FakeMav:
    def __init__(self):
        self.command = None

    def command_long_send(self, *arguments):
        self.command = arguments


class FakeConnection:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()

    def recv_match(self, **_kwargs):
        return types.SimpleNamespace(
            command=mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )


class MoveGimbalTest(unittest.TestCase):
    def test_point_gimbal_sends_validated_angles(self):
        connection = FakeConnection()

        self.assertTrue(move_gimbal.point_gimbal(connection, -45.0, 30.0, 0.1))
        self.assertEqual(connection.mav.command[4], -45.0)
        self.assertEqual(connection.mav.command[6], 30.0)

    def test_rejects_angle_outside_simulated_mount_range(self):
        with self.assertRaises(ValueError):
            move_gimbal.check_angle("pitch", 46.0, move_gimbal.PITCH_RANGE)


if __name__ == "__main__":
    unittest.main()
