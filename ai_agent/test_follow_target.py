#!/usr/bin/env python3

import math
from types import SimpleNamespace
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
        )

    def test_requires_stable_detections_before_tracking(self):
        controller = self.make_controller()

        first = controller.step(detection(), 1, 0.0, 0.0, True)
        second = controller.step(detection(), 2, 0.1, 0.1, True)

        self.assertEqual(first, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.TRACK)
        self.assertEqual(second, follow_target.VelocityCommand())

    def test_brief_miss_holds_track_then_timeout_sends_zero(self):
        controller = self.make_controller()
        controller.step(detection(), 1, 0.0, 0.0, True)
        controller.step(detection(), 2, 0.1, 0.1, True)

        brief_miss = controller.step(None, 3, 0.1, 0.2, True)
        self.assertEqual(controller.state, follow_target.FollowState.TRACK)
        lost = controller.step(None, 4, 0.1, 0.7, True)

        self.assertEqual(brief_miss, follow_target.VelocityCommand())
        self.assertEqual(lost, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.LOST)

    def test_vehicle_gate_sends_immediate_zero(self):
        controller = self.make_controller()
        controller.step(detection(), 1, 0.0, 0.0, True)
        controller.step(detection(), 2, 0.1, 0.1, True)

        gated = controller.step(detection(), 3, 0.2, 0.2, False)

        self.assertEqual(gated, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.IDLE)

    def test_manual_stop_sends_zero_without_losing_process(self):
        controller = self.make_controller()
        controller.step(detection(), 1, 0.0, 0.0, True)
        controller.step(detection(), 2, 0.1, 0.1, True)

        controller.toggle()
        stopped = controller.step(detection(), 3, 0.2, 0.2, True)

        self.assertFalse(controller.enabled)
        self.assertEqual(stopped, follow_target.VelocityCommand())
        self.assertEqual(controller.state, follow_target.FollowState.IDLE)

    def test_fixed_tracking_without_pursuit_sends_zero(self):
        controller = follow_target.FollowController(acquire_frames=1)

        command = controller.step(detection(), 1, 0.0, 0.0, True)

        self.assertEqual(controller.state, follow_target.FollowState.TRACK)
        self.assertEqual(command, follow_target.VelocityCommand())


class SearchControllerTest(unittest.TestCase):
    def test_gimbal_tracking_respects_accepted_pitch_limit(self):
        controller = follow_target.SearchController(target="car")
        controller._aim_gimbal(detection(0.0, 1.0), 10.0)
        self.assertEqual(controller.pitch, -90.0)

    def test_active_pursuit_uses_camera_pose_and_stops_on_stale_pose(self):
        controller = follow_target.SearchController(target="car", max_speed=1.0)
        controller.state = follow_target.SearchState.TRACK
        camera = follow_target.CameraPose(
            (0.0, 0.0, 10.0),
            (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0),
            0.0,
        )
        result = controller.step(
            detection(0.0, -0.4), 1, 0.0, 0.0, 0.1, True,
            (0.0, 0.0, -10.0), 0.0, 10.0, camera,
        )
        self.assertGreater(result.velocity.right, 0.0)
        self.assertAlmostEqual(result.target_forward, 0.0, delta=0.01)
        self.assertGreater(result.target_right, 4.0)

        stale = controller.step(
            detection(0.0, -0.4), 2, 0.4, 0.4, 0.1, True,
            (0.0, 0.0, -10.0), 0.0, 10.0, camera,
        )
        self.assertEqual(stale.velocity, follow_target.VelocityCommand())
        self.assertEqual(controller.geometry_error, "camera pose unavailable")

    def test_camera_pose_composes_nested_gazebo_frames(self):
        def pose(name, position, orientation):
            return SimpleNamespace(
                name=name,
                position=SimpleNamespace(**dict(zip("xyz", position))),
                orientation=SimpleNamespace(**dict(zip("wxyz", orientation))),
            )

        vehicle = pose("iris_with_gimbal", (2, 3, 10), (1, 0, 0, 0))
        gimbal = pose("gimbal", (0, 0, -0.1), (1, 0, 0, 0))
        pitch = pose("pitch_link", (0, 0, 0.02), (1, 0, 0, 0))
        camera = follow_target.camera_pose_from_links(vehicle, gimbal, pitch, 5.0)

        self.assertEqual(camera.position, (2.0, 3.0, 9.92))
        self.assertEqual(camera.observed_at, 5.0)

    def test_tracking_without_pursuit_keeps_drone_stationary(self):
        controller = follow_target.SearchController(target="car", follow_motion_enabled=False)
        controller.state = follow_target.SearchState.TRACK
        controller.last_detection = detection(0.0, 0.0)

        output = controller.step(
            detection(0.0, 0.0), 1, 0.0, 0.0, 0.1, True,
            (0.0, 0.0, -10.0), 0.0, 10.0,
        )

        self.assertEqual(output.state, follow_target.SearchState.TRACK)
        self.assertEqual(output.velocity, follow_target.VelocityCommand())

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

    def test_manual_stop_disables_search_motion(self):
        controller = follow_target.SearchController(target="car")
        controller.toggle()

        output = controller.step(
            None, 1, 0.0, 0.0, 0.1, True, (0.0, 0.0, -10.0), 0.0, 10.0
        )

        self.assertFalse(controller.enabled)
        self.assertEqual(output.state, follow_target.SearchState.IDLE)
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
