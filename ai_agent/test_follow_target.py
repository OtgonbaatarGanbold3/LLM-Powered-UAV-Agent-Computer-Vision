#!/usr/bin/env python3

import unittest

from pymavlink import mavutil

import drone_cv
import follow_target


class FakeMav:
    def __init__(self):
        self.arguments = None

    def set_position_target_local_ned_send(self, *arguments):
        self.arguments = arguments


class FakeConnection:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()


def detection(error_x=0.3, error_y=-0.4):
    return drone_cv.Detection((1, 2, 3, 4), (2, 3), 30.0, 1.0, error_x, error_y)


class VelocityMessageTest(unittest.TestCase):
    def test_mask_activates_only_velocity_and_zero_yaw_rate(self):
        connection = FakeConnection()

        follow_target.send_velocity(connection, follow_target.VelocityCommand(0.4, -0.2))

        arguments = connection.mav.arguments
        expected_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        self.assertEqual(arguments[3], mavutil.mavlink.MAV_FRAME_BODY_NED)
        self.assertEqual(arguments[4], expected_mask)
        self.assertEqual(arguments[8:11], (0.4, -0.2, 0.0))
        self.assertEqual(arguments[15], 0.0)


class FollowControllerTest(unittest.TestCase):
    def make_controller(self):
        return follow_target.FollowController(
            acquire_frames=2,
            detection_timeout=0.5,
            smoothing=1.0,
            max_acceleration=10.0,
            max_speed=1.0,
        )

    def test_requires_stable_detections_before_tracking(self):
        controller = self.make_controller()

        first = controller.step(detection(), 1, 0.0, 0.0, True, 0.1)
        second = controller.step(detection(), 2, 0.1, 0.1, True, 0.1)

        self.assertEqual(first, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.TRACK)
        self.assertGreater(second.forward, 0.0)
        self.assertGreater(second.right, 0.0)

    def test_brief_miss_holds_track_then_timeout_sends_zero(self):
        controller = self.make_controller()
        controller.step(detection(), 1, 0.0, 0.0, True, 0.1)
        controller.step(detection(), 2, 0.1, 0.1, True, 0.1)

        brief_miss = controller.step(None, 3, 0.1, 0.2, True, 0.1)
        lost = controller.step(None, 4, 0.1, 0.7, True, 0.1)

        self.assertNotEqual(brief_miss, follow_target.VelocityCommand())
        self.assertEqual(lost, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.LOST)

    def test_vehicle_gate_sends_immediate_zero(self):
        controller = self.make_controller()
        controller.step(detection(), 1, 0.0, 0.0, True, 0.1)
        controller.step(detection(), 2, 0.1, 0.1, True, 0.1)

        gated = controller.step(detection(), 3, 0.2, 0.2, False, 0.1)

        self.assertEqual(gated, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.IDLE)

    def test_manual_stop_sends_zero_without_losing_process(self):
        controller = self.make_controller()
        controller.step(detection(), 1, 0.0, 0.0, True, 0.1)
        controller.step(detection(), 2, 0.1, 0.1, True, 0.1)

        controller.toggle()
        stopped = controller.step(detection(), 3, 0.2, 0.2, True, 0.1)

        self.assertFalse(controller.enabled)
        self.assertEqual(stopped, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.IDLE)


class SearchControllerTest(unittest.TestCase):
    def test_raster_holds_position_until_full_gimbal_scan_finishes(self):
        controller = follow_target.SearchController(
            target="car",
            search_radius=1.5,
            search_speed=0.2,
            scan_pitch_min=-20.0,
            scan_pitch_max=20.0,
            scan_pitch_step=20.0,
            scan_yaw_min=-10.0,
            scan_yaw_max=10.0,
            scan_rate=30.0,
        )
        previous_pitch = controller.pitch
        reached_yaw_edge = False
        saw_pitch_step = False

        for tick in range(60):
            now = tick * 0.1
            output = controller.step(
                None, tick, now, now, 0.1, True, (0.0, 0.0, -10.0), 0.0, 10.0
            )
            if output.state is follow_target.SearchState.RASTER:
                self.assertIsNone(output.position_target)
                self.assertEqual(output.velocity, follow_target.VelocityCommand())
                self.assertGreaterEqual(output.gimbal_yaw, -10.0)
                self.assertLessEqual(output.gimbal_yaw, 10.0)
                reached_yaw_edge |= abs(output.gimbal_yaw) == 10.0
                if output.gimbal_pitch != previous_pitch:
                    saw_pitch_step = True
                previous_pitch = output.gimbal_pitch
            else:
                break

        self.assertTrue(reached_yaw_edge)
        self.assertTrue(saw_pitch_step)
        self.assertEqual(output.state, follow_target.SearchState.MOVE)
        self.assertIsNotNone(output.position_target)

    def test_target_detection_interrupts_search_motion_for_lock(self):
        controller = follow_target.SearchController(target="car")

        output = controller.step(
            detection(), 1, 0.0, 0.0, 0.1, True, (0.0, 0.0, -10.0), 0.0, 10.0
        )

        self.assertEqual(output.state, follow_target.SearchState.LOCK)
        self.assertIsNone(output.position_target)
        self.assertEqual(output.velocity, follow_target.VelocityCommand())


class VehicleStatusTest(unittest.TestCase):
    def test_requires_guided_armed_airborne_fresh_telemetry(self):
        status = follow_target.VehicleStatus()
        status.armed = True
        status.mode = "GUIDED"
        status.relative_altitude = 10.0
        status.last_heartbeat = 5.0
        status.last_position = 5.0

        self.assertEqual(status.readiness(5.1), (True, "ready"))
        status.mode = "LOITER"
        self.assertFalse(status.readiness(5.1)[0])


if __name__ == "__main__":
    unittest.main()
