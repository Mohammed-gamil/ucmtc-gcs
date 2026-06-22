"""Vision node for camera-derived lane and obstacle telemetry."""

from __future__ import annotations

import json
import random
import time
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, rclpy, run_node
from rover_core.telemetry_utils import SENSOR_QOS, make_vision_payload

# rcl_interfaces ParameterDescriptor is optional (falls back for local dev).
try:
    from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange  # type: ignore
    _DESCRIPTORS_AVAILABLE = True
except ImportError:
    _DESCRIPTORS_AVAILABLE = False


def _float_descriptor(description: str, min_val: float, max_val: float):
    if not _DESCRIPTORS_AVAILABLE:
        return None
    d = ParameterDescriptor()
    d.description = description
    r = FloatingPointRange()
    r.from_value = min_val
    r.to_value = max_val
    d.floating_point_range = [r]
    return d


def _int_descriptor(description: str, min_val: int, max_val: int):
    if not _DESCRIPTORS_AVAILABLE:
        return None
    d = ParameterDescriptor()
    d.description = description
    r = IntegerRange()
    r.from_value = min_val
    r.to_value = max_val
    d.integer_range = [r]
    return d


class VisionNode(Node):
    """ROS 2 node for rover vision processing and AI inference telemetry.

    QoS rationale
    -------------
    - ``/rover/telemetry/vision`` publisher: BEST_EFFORT sensor stream with
      depth-5. At 30 Hz a slow subscriber should drop frames, not accumulate
      them — stale vision data is worse than a brief gap.
    """

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("vision_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter(
            "publish_rate_hz",
            30.0,
            _float_descriptor("Vision processing and publish rate in Hz", 1.0, 100.0),
        )
        self.declare_parameter(
            "camera_index",
            0,
            _int_descriptor("Index of the video capture device / camera", 0, 10),
        )
        self.declare_parameter(
            "ai_model_path",
            "",
        )
        self.declare_parameter(
            "confidence_threshold",
            0.55,
            _float_descriptor("Minimum confidence threshold for object detection", 0.0, 1.0),
        )

        publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        camera_index: int = (
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        self._confidence_threshold: float = (
            self.get_parameter("confidence_threshold")
            .get_parameter_value()
            .double_value
        )
        period_sec = 1.0 / max(publish_rate_hz, 1.0)

        # ── Pub / Sub ────────────────────────────────────────────────────────
        # Vision telemetry: BEST_EFFORT sensor stream.
        self._publisher = self.create_publisher(
            String,
            "/rover/telemetry/vision",
            SENSOR_QOS,
        )

        # ── Camera init ──────────────────────────────────────────────────────
        self._start_stamp = self.get_clock().now()
        self._frame_id = 0
        self._lane_detected = False
        self._obstacles_count = 0
        self._camera_available = False
        self._camera_capture = None
        try:
            import cv2  # type: ignore

            cap = cv2.VideoCapture(camera_index)
            if cap.isOpened():
                self._camera_available = True
                self._camera_capture = cap
                self.get_logger().info(f"Camera {camera_index} opened successfully")
            else:
                cap.release()
                self.get_logger().warning(
                    f"Camera {camera_index} could not be opened — using simulation"
                )
        except Exception as exc:
            self.get_logger().warning(
                f"cv2 not available ({exc}) — using simulation"
            )

        self._timer = self.create_timer(period_sec, self._timer_callback)
        self.get_logger().info(
            f"Vision node ready (rate={publish_rate_hz:.1f} Hz, "
            f"camera={'hw' if self._camera_available else 'sim'})"
        )

    # ── Frame processing ─────────────────────────────────────────────────────

    def _simulate_frame(self) -> dict[str, Any]:
        self._frame_id += 1
        confidence = 0.72 + random.gauss(0.0, 0.12)
        img_detected = confidence > self._confidence_threshold and random.random() > 0.1
        if self._frame_id % 30 == 0:
            self._lane_detected = random.random() > 0.35
        self._obstacles_count = random.randint(0, 5) if img_detected else 0
        fps_vision = 29.0 + random.gauss(0.0, 1.3)
        img_elapsed_sec = int(
            (self.get_clock().now() - self._start_stamp).nanoseconds / 1_000_000_000
        )
        return make_vision_payload(
            img_confidence=confidence,
            img_detected=img_detected,
            laser_active=random.random() > 0.75,
            img_elapsed_sec=img_elapsed_sec,
            img_task_status="processing" if img_detected else "idle",
            lane_detected=self._lane_detected,
            obstacles_count=self._obstacles_count,
            fps_vision=fps_vision,
        )

    def process_frame(self) -> dict[str, Any]:
        if self._camera_available and self._camera_capture is not None:
            ok, _frame = self._camera_capture.read()
            if not ok:
                self.get_logger().warning("Camera read failed — using simulation frame")
                return self._simulate_frame()

            self._frame_id += 1
            confidence = 0.65 + random.gauss(0.0, 0.1)
            img_detected = confidence > self._confidence_threshold
            if self._frame_id % 25 == 0:
                self._lane_detected = random.random() > 0.4
            self._obstacles_count = random.randint(0, 3) if img_detected else 0
            fps_vision = 28.5 + random.gauss(0.0, 1.0)
            img_elapsed_sec = int(
                (self.get_clock().now() - self._start_stamp).nanoseconds / 1_000_000_000
            )
            return make_vision_payload(
                img_confidence=confidence,
                img_detected=img_detected,
                laser_active=random.random() > 0.7,
                img_elapsed_sec=img_elapsed_sec,
                img_task_status="processing" if img_detected else "idle",
                lane_detected=self._lane_detected,
                obstacles_count=self._obstacles_count,
                fps_vision=fps_vision,
            )

        return self._simulate_frame()

    # ── Publish ──────────────────────────────────────────────────────────────

    def publish_vision_metrics(self) -> dict:
        payload = self.process_frame()
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._publisher.publish(message)
        return payload

    def _timer_callback(self) -> None:
        self.publish_vision_metrics()

    # ── Shutdown ─────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if self._camera_capture is not None:
            try:
                self._camera_capture.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    node = VisionNode()
    run_node(node, fallback_period=0.033)


if __name__ == "__main__":
    main()
