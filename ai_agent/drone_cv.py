#!/usr/bin/env python3
"""Detect and track a selected simulator target without commanding the drone."""

import argparse
from dataclasses import dataclass, replace
import sys
import threading
import time

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node


DEFAULT_CAMERA_TOPIC = (
    "/world/iris_runway/model/iris_with_gimbal/model/gimbal/"
    "link/pitch_link/sensor/camera/image"
)


@dataclass(frozen=True)
class MarkerSpec:
    label: str
    hsv_lower: tuple
    hsv_upper: tuple
    min_area: float
    draw_color: tuple


@dataclass(frozen=True)
class Detection:
    box: tuple
    center: tuple
    area: float
    confidence: float
    error_x: float
    error_y: float


MARKERS = {
    "car": MarkerSpec("yellow car marker", (18, 90, 90), (42, 255, 255), 25.0, (0, 255, 255)),
    "person": MarkerSpec("cyan person marker", (78, 90, 90), (102, 255, 255), 5.0, (255, 255, 0)),
}
MORPH_KERNEL = np.ones((3, 3), np.uint8)


class TargetMemory:
    """Keep the last detection through a short run of missed frames."""

    def __init__(self, max_missed_frames):
        self.max_missed_frames = max_missed_frames
        self.last_detection = None
        self.missed_frames = 0
        self.loss_count = 0

    def update(self, detection):
        if detection is not None:
            self.last_detection = detection
            self.missed_frames = 0
            return detection, "TRACKING"

        if self.last_detection is not None:
            self.missed_frames += 1
            if self.missed_frames <= self.max_missed_frames:
                confidence = self.last_detection.confidence * (
                    1.0 - self.missed_frames / (self.max_missed_frames + 1.0)
                )
                return replace(self.last_detection, confidence=confidence), "MEMORY"

            self.last_detection = None
            self.loss_count += 1
            return None, "LOST"

        return None, "SEARCHING"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=MARKERS, default="car")
    parser.add_argument("--topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument(
        "--frame-timeout",
        type=float,
        default=10.0,
        help="seconds to wait for camera frames (default: %(default)s)",
    )
    parser.add_argument(
        "--max-missed-frames",
        type=int,
        default=5,
        help="retain the last target for this many missed frames (default: %(default)s)",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        help="override the selected marker's minimum contour area in pixels",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; zero runs until q or Ctrl+C",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="process frames and print metrics without opening a window",
    )
    return parser.parse_args()


def detect_marker(frame_bgr, marker, min_area=None):
    """Return the largest matching colored marker in a BGR image."""
    threshold = marker.min_area if min_area is None else min_area
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(marker.hsv_lower, dtype=np.uint8),
        np.array(marker.hsv_upper, dtype=np.uint8),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < threshold:
        return None

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None

    height, width = frame_bgr.shape[:2]
    center = (
        int(moments["m10"] / moments["m00"]),
        int(moments["m01"] / moments["m00"]),
    )
    box = cv2.boundingRect(contour)
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    solidity = area / hull_area if hull_area else 0.0
    confidence = min(1.0, area / (threshold * 4.0)) * min(1.0, solidity)
    return Detection(
        box=box,
        center=center,
        area=area,
        confidence=confidence,
        error_x=(center[0] - width / 2.0) / (width / 2.0),
        error_y=(center[1] - height / 2.0) / (height / 2.0),
    )


def decode_rgb_image(message):
    expected_size = message.width * message.height * 3
    if message.width <= 0 or message.height <= 0 or len(message.data) != expected_size:
        raise ValueError(
            f"expected {expected_size} RGB bytes for {message.width}x{message.height}, "
            f"received {len(message.data)}"
        )
    frame_rgb = np.frombuffer(message.data, dtype=np.uint8).reshape(
        (message.height, message.width, 3)
    )
    return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)


def draw_tracking_overlay(frame, target, marker, detection, status, camera_hz, process_hz, losses):
    height, width = frame.shape[:2]
    screen_center = (width // 2, height // 2)
    cv2.drawMarker(frame, screen_center, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

    if detection is not None:
        x, y, box_width, box_height = detection.box
        color = marker.draw_color if status == "TRACKING" else (0, 165, 255)
        cv2.rectangle(frame, (x, y), (x + box_width, y + box_height), color, 2)
        cv2.circle(frame, detection.center, 5, color, -1)
        cv2.line(frame, screen_center, detection.center, color, 1)
        cv2.putText(
            frame,
            f"conf={detection.confidence:.2f} err=({detection.error_x:+.2f}, {detection.error_y:+.2f})",
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    cv2.putText(
        frame,
        f"{target.upper()} | {status}",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        marker.draw_color,
        2,
    )
    cv2.putText(
        frame,
        f"camera={camera_hz:.1f} Hz processed={process_hz:.1f} Hz losses={losses}",
        (10, height - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
    )


def validate_args(args):
    if args.frame_timeout <= 0:
        return "--frame-timeout must be positive"
    if args.max_missed_frames < 0:
        return "--max-missed-frames cannot be negative"
    if args.min_area is not None and args.min_area <= 0:
        return "--min-area must be positive"
    if args.duration < 0:
        return "--duration cannot be negative"
    return None


def main():
    args = parse_args()
    argument_error = validate_args(args)
    if argument_error:
        print(f"Error: {argument_error}.", file=sys.stderr)
        return 2

    marker = MARKERS[args.target]
    frame_condition = threading.Condition()
    first_frame = threading.Event()
    latest_frame = None
    frame_sequence = 0
    camera_frames = 0
    format_error = []
    last_camera_frame_at = time.monotonic()

    def image_callback(message):
        nonlocal latest_frame, frame_sequence, camera_frames, last_camera_frame_at
        try:
            frame_bgr = decode_rgb_image(message)
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

    print(f"Target: {args.target} ({marker.label})")
    print(f"Subscribing to Gazebo camera topic:\n  {args.topic}")
    print("Vision-only mode: this process has no MAVLink connection or movement output.")
    node = Node()
    if not node.subscribe(GzImage, args.topic, image_callback):
        print("Error: Gazebo rejected the camera subscription.", file=sys.stderr)
        return 1

    if not first_frame.wait(timeout=args.frame_timeout):
        print(
            f"Error: no camera frame received within {args.frame_timeout:g} seconds.",
            file=sys.stderr,
        )
        return 1
    if format_error:
        print(f"Error: unsupported camera format: {format_error[0]}", file=sys.stderr)
        return 1

    memory = TargetMemory(args.max_missed_frames)
    started_at = time.monotonic()
    last_sequence = 0
    processed_frames = 0
    detected_frames = 0
    retained_frames = 0
    exit_code = 0
    if args.no_display:
        print("Tracker active without a display. Press Ctrl+C to quit.")
    else:
        print("Tracker active. Press q in the camera window or Ctrl+C to quit.")

    try:
        while True:
            now = time.monotonic()
            if args.duration and now - started_at >= args.duration:
                break

            with frame_condition:
                frame_condition.wait_for(
                    lambda: frame_sequence > last_sequence or bool(format_error),
                    timeout=0.1,
                )
                if format_error:
                    print(f"Error: unsupported camera format: {format_error[0]}", file=sys.stderr)
                    exit_code = 1
                    break
                if frame_sequence == last_sequence:
                    if time.monotonic() - last_camera_frame_at >= args.frame_timeout:
                        print(
                            f"Error: camera stream stopped for {args.frame_timeout:g} seconds.",
                            file=sys.stderr,
                        )
                        exit_code = 1
                        break
                    continue
                frame = latest_frame.copy()
                last_sequence = frame_sequence

            raw_detection = detect_marker(frame, marker, args.min_area)
            detection, status = memory.update(raw_detection)
            processed_frames += 1
            if raw_detection is not None:
                detected_frames += 1
            elif status == "MEMORY":
                retained_frames += 1

            elapsed = max(time.monotonic() - started_at, 1e-6)
            camera_hz = camera_frames / elapsed
            process_hz = processed_frames / elapsed
            draw_tracking_overlay(
                frame,
                args.target,
                marker,
                detection,
                status,
                camera_hz,
                process_hz,
                memory.loss_count,
            )

            if not args.no_display:
                cv2.imshow("Drone Perception Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if not args.no_display:
            cv2.destroyAllWindows()

    elapsed = max(time.monotonic() - started_at, 1e-6)
    print(
        "Tracking summary: "
        f"target={args.target} camera_frames={camera_frames} "
        f"camera_rate={camera_frames / elapsed:.2f}Hz "
        f"processed_frames={processed_frames} "
        f"processed_rate={processed_frames / elapsed:.2f}Hz "
        f"detections={detected_frames} retained={retained_frames} "
        f"target_losses={memory.loss_count}"
    )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
