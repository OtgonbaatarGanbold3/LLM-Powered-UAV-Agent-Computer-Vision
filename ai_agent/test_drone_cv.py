#!/usr/bin/env python3

from types import SimpleNamespace
import unittest

import cv2
import numpy as np

import drone_cv


class MarkerDetectionTest(unittest.TestCase):
    def make_frame(self):
        return np.zeros((100, 120, 3), dtype=np.uint8)

    def test_selects_yellow_car_marker_and_reports_normalized_error(self):
        frame = self.make_frame()
        cv2.rectangle(frame, (50, 30), (70, 50), (0, 255, 255), -1)

        detection = drone_cv.detect_marker(frame, drone_cv.MARKERS["car"])

        self.assertIsNotNone(detection)
        self.assertAlmostEqual(detection.error_x, 0.0, places=2)
        self.assertAlmostEqual(detection.error_y, -0.2, places=2)
        self.assertGreater(detection.confidence, 0.9)

    def test_target_selection_ignores_other_marker_color(self):
        frame = self.make_frame()
        cv2.rectangle(frame, (40, 30), (80, 70), (255, 255, 0), -1)

        self.assertIsNone(drone_cv.detect_marker(frame, drone_cv.MARKERS["car"]))
        self.assertIsNotNone(drone_cv.detect_marker(frame, drone_cv.MARKERS["person"]))

    def test_rejects_camera_buffer_with_wrong_size(self):
        message = SimpleNamespace(width=2, height=2, data=b"too short")

        with self.assertRaisesRegex(ValueError, "expected 12 RGB bytes"):
            drone_cv.decode_rgb_image(message)


class TargetMemoryTest(unittest.TestCase):
    def test_retains_identity_for_short_miss_then_counts_one_loss(self):
        detection = drone_cv.Detection((1, 2, 3, 4), (2, 3), 20.0, 0.9, 0.1, -0.2)
        memory = drone_cv.TargetMemory(max_missed_frames=2)

        self.assertEqual(memory.update(detection)[1], "TRACKING")
        retained, status = memory.update(None)
        self.assertEqual(status, "MEMORY")
        self.assertEqual(retained.center, detection.center)
        self.assertEqual(memory.update(None)[1], "MEMORY")
        self.assertEqual(memory.update(None), (None, "LOST"))
        self.assertEqual(memory.loss_count, 1)
        self.assertEqual(memory.update(None), (None, "SEARCHING"))
        self.assertEqual(memory.loss_count, 1)


if __name__ == "__main__":
    unittest.main()
