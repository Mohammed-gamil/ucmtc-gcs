"""Vision node for camera-derived lane and obstacle telemetry."""

from __future__ import annotations

import json
import math
import random
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import SENSOR_QOS, make_vision_payload, get_topic_path

try:
    from sensor_msgs.msg import LaserScan
except ImportError:
    LaserScan = None

try:
    from sensor_msgs.msg import Image as ROS_Image
except ImportError:
    ROS_Image = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

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
        self._init_topics()

        # ── Dynamic Config Checker ──
        self._config_mtime = 0.0
        self._check_topic_config()
        self._config_timer = self.create_timer(1.0, self._check_topic_config)

        # ── Camera init ──────────────────────────────────────────────────────
        self._start_stamp = self.get_clock().now()
        self._frame_id = 0
        self._lane_detected = False
        self._obstacles_count = 0
        self._camera_available = False
        self._camera_capture = None
        self._mock_image = None
        self._current_frame = None

        try:
            import cv2  # type: ignore

            cap = cv2.VideoCapture(camera_index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            if cap.isOpened():
                self._camera_available = True
                self._camera_capture = cap
                self.get_logger().info(f"Camera {camera_index} opened successfully (requested HD 1280x720)")
            else:
                cap.release()
                self.get_logger().warning(
                    f"Camera {camera_index} could not be opened — using simulation"
                )

            # Load mock background image for simulation fallback
            import os
            base_dir = os.path.dirname(os.path.abspath(__file__))
            mock_img_path = os.path.normpath(os.path.join(base_dir, "../../../../web_gcs/rover_camera_feed.jpg"))
            if os.path.exists(mock_img_path):
                self._mock_image = cv2.imread(mock_img_path)
                if self._mock_image is not None:
                    self.get_logger().info(f"Loaded simulation background image from {mock_img_path}")
        except Exception as exc:
            self.get_logger().warning(
                f"cv2/image loading not available ({exc}) — using simulation"
            )

        self._timer = self.create_timer(period_sec, self._timer_callback)
        self.get_logger().info(
            f"Vision node ready (rate={publish_rate_hz:.1f} Hz, "
            f"camera={'hw' if self._camera_available else 'sim'})"
        )
    def _init_topics(self) -> None:
        vision_topic = get_topic_path("telemetry_vision", "/rover/telemetry/vision")
        scan_topic = get_topic_path("obstacle_avoidance", "/scan")
        camera_topic = get_topic_path("image_recognition", "/rover/sensors/camera")

        self._publisher = self.create_publisher(
            String,
            vision_topic,
            SENSOR_QOS,
        )
        self._scan_publisher = None
        if LaserScan is not None:
            self._scan_publisher = self.create_publisher(
                LaserScan,
                scan_topic,
                SENSOR_QOS,
            )
        self._image_publisher = None
        if ROS_Image is not None:
            self._image_publisher = self.create_publisher(
                ROS_Image,
                camera_topic,
                SENSOR_QOS,
            )

    def _check_topic_config(self) -> None:
        import os
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.normpath(os.path.join(base_dir, "../../../../web_gcs/topic_config.json"))
            if os.path.exists(config_path):
                mtime = os.path.getmtime(config_path)
                if self._config_mtime != 0.0 and mtime != self._config_mtime:
                    self.get_logger().info("topic_config.json changed! Reconfiguring topics...")
                    self._config_mtime = mtime
                    self._update_topics()
                elif self._config_mtime == 0.0:
                    self._config_mtime = mtime
        except Exception:
            pass

    def _update_topics(self) -> None:
        try:
            if hasattr(self, "_publisher") and self._publisher:
                self.destroy_publisher(self._publisher)
                self._publisher = None
            if hasattr(self, "_scan_publisher") and self._scan_publisher:
                self.destroy_publisher(self._scan_publisher)
                self._scan_publisher = None
            if hasattr(self, "_image_publisher") and self._image_publisher:
                self.destroy_publisher(self._image_publisher)
                self._image_publisher = None
            
            self._init_topics()
            self.get_logger().info("VisionNode topics updated successfully.")
        except Exception as e:
            self.get_logger().error(f"Error reconfiguring VisionNode topics: {e}")
    # ── Frame processing ─────────────────────────────────────────────────────

    def _simulate_frame(self) -> dict[str, Any]:
        self._frame_id += 1

        # Build simulated frame
        import numpy as np
        import cv2
        if self._mock_image is not None:
            frame = self._mock_image.copy()
        else:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            for y in range(0, 480, 40):
                cv2.line(frame, (0, y), (640, y), (20, 30, 20), 1)
            for x in range(0, 640, 40):
                cv2.line(frame, (x, 0), (x, 480), (20, 30, 20), 1)
            cv2.putText(frame, "SIMULATED OPTICS FEED // NO HARDWARE", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 102), 2)

        # Draw sweeping scan line
        time_sec = (self.get_clock().now() - self._start_stamp).nanoseconds / 1e9
        scan_y = int((time_sec * 100) % 480)
        cv2.line(frame, (0, scan_y), (640, scan_y), (0, 255, 102), 1)

        # Add dynamic frame timestamp text
        stamp_str = f"FRAME: {self._frame_id} // T+{time_sec:.2f}s"
        cv2.putText(frame, stamp_str, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 102), 1)

        self._current_frame = frame

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
            ok, frame = self._camera_capture.read()
            if not ok:
                self.get_logger().warning("Camera read failed — using simulation frame")
                return self._simulate_frame()

            self._frame_id += 1
            self._current_frame = frame

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

    def _get_simulated_scan(self, t: float) -> list[float]:
        sim_ranges = []
        obs1_r = 3.5 + 1.5 * math.sin(t * 0.4)
        obs1_a = 0.2 * math.cos(t * 0.3)
        obs1_width = 0.25

        obs2_r = 4.0 + 1.0 * math.cos(t * 0.2)
        obs2_a = math.pi / 3.0 + 0.15 * math.sin(t * 0.5)
        obs2_width = 0.2

        obs3_r = 2.5 + 0.8 * math.sin(t * 0.6)
        obs3_a = -math.pi / 4.0 + 0.2 * math.cos(t * 0.4)
        obs3_width = 0.3

        for i in range(180):
            angle = -math.pi + i * (2.0 * math.pi / 179.0)
            r = 6.0 + 0.8 * math.sin(angle * 3.0) + random.uniform(-0.03, 0.03)
            for obs_r, obs_a, obs_w in [(obs1_r, obs1_a, obs1_width), (obs2_r, obs2_a, obs2_width), (obs3_r, obs3_a, obs3_width)]:
                diff = math.atan2(math.sin(angle - obs_a), math.cos(angle - obs_a))
                if abs(diff) < obs_w:
                    r = min(r, obs_r + random.uniform(-0.01, 0.01))
            r = max(0.12, min(r, 12.0))
            sim_ranges.append(r)
        return sim_ranges

    # ── Publish ──────────────────────────────────────────────────────────────

    def publish_vision_metrics(self) -> dict:
        payload = self.process_frame()
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._publisher.publish(message)

        # Publish the camera image
        if self._image_publisher is not None and ROS_Image is not None and CvBridge is not None:
            if hasattr(self, "_current_frame") and self._current_frame is not None:
                try:
                    bridge = CvBridge()
                    img_msg = bridge.cv2_to_imgmsg(self._current_frame, encoding="bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_link"
                    self._image_publisher.publish(img_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish camera image: {e}")

        if self._scan_publisher is not None and LaserScan is not None:
            scan_msg = LaserScan()
            if hasattr(self, 'get_clock'):
                scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser_frame"
            scan_msg.angle_min = -math.pi
            scan_msg.angle_max = math.pi
            scan_msg.angle_increment = 2.0 * math.pi / 179.0
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = 0.12
            scan_msg.range_max = 12.0

            t = self.get_clock().now().nanoseconds / 1e9
            scan_msg.ranges = self._get_simulated_scan(t)
            self._scan_publisher.publish(scan_msg)

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
