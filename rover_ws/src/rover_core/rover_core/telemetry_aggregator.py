"""Telemetry aggregator that publishes the canonical rover payload."""

from __future__ import annotations

import json
import os
import random
import time
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, rclpy, run_node
from rover_core.telemetry_utils import (
    RELIABLE_QOS,
    SAFETY_HEARTBEAT_QOS,
    SENSOR_QOS,
    clamp,
    decode_json_message,
    make_communication_payload,
    make_jetson_payload,
    make_ros_payload,
    make_navigation_payload,
    make_safety_payload,
    make_vision_payload,
)

# QoS event callbacks require rclpy — fall back gracefully.
try:
    from rclpy.qos_event import SubscriptionEventCallbacks  # type: ignore
    _QOS_EVENTS_AVAILABLE = True
except ImportError:
    _QOS_EVENTS_AVAILABLE = False

# rcl_interfaces ParameterDescriptor is optional (falls back for local dev).
try:
    from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange  # type: ignore
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


class TelemetryAggregatorNode(Node):
    """Merge rover section topics into the canonical ``/rover/telemetry`` payload.

    QoS rationale
    -------------
    - Nav / vision subscribers: SENSOR_QOS (BEST_EFFORT) — must match source publishers.
    - Safety subscriber: SAFETY_HEARTBEAT_QOS (RELIABLE + DEADLINE 500ms + LIFESPAN 1s)
      — must match the SafetyNode publisher. A DEADLINE event fires when no safety
      heartbeat arrives within 500 ms; the aggregator logs a critical error and marks
      safety data as stale so the GCS operator is immediately alerted.
    - Control subscriber: RELIABLE_QOS — matches MotorControlNode publisher.
    - ``/rover/telemetry`` publisher: RELIABLE_QOS so the GCS bridge doesn't miss frames.
    """

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("telemetry_aggregator")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter(
            "publish_rate_hz",
            10.0,
            _float_descriptor("Telemetry aggregator publish rate in Hz", 1.0, 50.0),
        )
        self.declare_parameter(
            "nav_stale_sec",
            1.5,
            _float_descriptor("Staleness timeout for navigation data in seconds", 0.1, 10.0),
        )
        self.declare_parameter(
            "safety_stale_sec",
            1.5,
            _float_descriptor("Staleness timeout for safety data in seconds", 0.1, 10.0),
        )
        self.declare_parameter(
            "vision_stale_sec",
            1.5,
            _float_descriptor("Staleness timeout for vision data in seconds", 0.1, 10.0),
        )
        self.declare_parameter(
            "control_stale_sec",
            2.5,
            _float_descriptor("Staleness timeout for control data in seconds", 0.1, 10.0),
        )

        publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._nav_stale_sec: float = (
            self.get_parameter("nav_stale_sec").get_parameter_value().double_value
        )
        self._safety_stale_sec: float = (
            self.get_parameter("safety_stale_sec").get_parameter_value().double_value
        )
        self._vision_stale_sec: float = (
            self.get_parameter("vision_stale_sec").get_parameter_value().double_value
        )
        self._control_stale_sec: float = (
            self.get_parameter("control_stale_sec").get_parameter_value().double_value
        )
        period_sec = 1.0 / max(publish_rate_hz, 0.1)

        # ── Pub / Sub ────────────────────────────────────────────────────────
        # Merged payload publisher: RELIABLE so the GCS bridge doesn't miss frames.
        self._publisher = self.create_publisher(String, "/rover/telemetry", RELIABLE_QOS)
        # Nav telemetry: BEST_EFFORT — matches SENSOR_QOS nav publisher.
        self._nav_subscription = self.create_subscription(
            String, "/rover/telemetry/nav", self.navigation_callback, SENSOR_QOS
        )
        # Safety telemetry: SAFETY_HEARTBEAT_QOS — must match SafetyNode publisher.
        # DEADLINE event fires when no message arrives within 500 ms.
        self._safety_subscription = self._create_safety_subscription()
        # Vision telemetry: BEST_EFFORT — matches SENSOR_QOS vision publisher.
        self._vision_subscription = self.create_subscription(
            String, "/rover/telemetry/vision", self.vision_callback, SENSOR_QOS
        )
        # Control heartbeats: RELIABLE — matches MotorControlNode publisher.
        self._control_subscription = self.create_subscription(
            String, "/rover/telemetry/control", self.control_callback, RELIABLE_QOS
        )

        # Use node clock (respects use_sim_time) for timing/staleness tracking
        self._start_stamp = self.get_clock().now()
        self._heartbeat_seq = 0
        self._packet_loss_pct = 0.0
        self._navigation: dict[str, Any] = make_navigation_payload(0.0, 0.0, 40.0, -75.0, 0.0, 0, 0.0, "idle")
        self._safety: dict[str, Any] = make_safety_payload(
            "idle", "idle", False, False, False, False, False, False, False, False,
        )
        self._safety_deadline_missed = False
        self._vision: dict[str, Any] = make_vision_payload(0.0, False, False, 0, "idle", False, 0, 0.0)
        self._control: dict[str, Any] = {
            "node_motor_ctrl": False,
            "heartbeat_seq": 0,
            "estop_latched": False,
            "last_command": {},
            "uptime_sec": 0,
            "timestamp_ms": 0,
        }

        self._nav_last_update = self._start_stamp
        self._safety_last_update = self._start_stamp
        self._vision_last_update = self._start_stamp
        self._control_last_update = self._start_stamp

        self._timer = self.create_timer(period_sec, self.timer_callback)
        self.get_logger().info(
            f"Telemetry aggregator ready (rate={publish_rate_hz:.1f} Hz)"
        )

    # ── Safety subscription with deadline callback ───────────────────────────

    def _create_safety_subscription(self):
        """Create /rover/telemetry/safety subscription with DEADLINE event handler."""
        if _QOS_EVENTS_AVAILABLE:
            event_callbacks = SubscriptionEventCallbacks()
            event_callbacks.deadline = self._on_safety_deadline_missed
            return self.create_subscription(
                String,
                "/rover/telemetry/safety",
                self.safety_callback,
                SAFETY_HEARTBEAT_QOS,
                event_callbacks=event_callbacks,
            )
        # Fallback: no QoS event support
        return self.create_subscription(
            String,
            "/rover/telemetry/safety",
            self.safety_callback,
            SAFETY_HEARTBEAT_QOS,
        )

    def _on_safety_deadline_missed(self, event) -> None:
        """Called when safety heartbeat goes silent for 500 ms."""
        self.get_logger().error(
            f"Safety heartbeat DEADLINE MISSED "
            f"(total={getattr(event, 'total_count', '?')}, "
            f"delta={getattr(event, 'total_count_change', '?')}). "
            "Flagging safety state as stale."
        )
        self._safety_deadline_missed = True

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _section_from_message(self, message: Any, section_name: str) -> dict[str, Any]:
        decoded = decode_json_message(message)
        if section_name in decoded and isinstance(decoded[section_name], dict):
            decoded = decoded[section_name]
        return decoded if isinstance(decoded, dict) else {}

    def navigation_callback(self, msg):
        section = self._section_from_message(msg, "Navigation")
        if section:
            self._navigation = section
            self._nav_last_update = self.get_clock().now()

    def safety_callback(self, msg):
        section = self._section_from_message(msg, "Safety")
        if section:
            self._safety = section
            self._safety_last_update = self.get_clock().now()
            if self._safety_deadline_missed:
                self.get_logger().info("Safety heartbeat resumed — clearing deadline flag.")
                self._safety_deadline_missed = False

    def vision_callback(self, msg):
        section = self._section_from_message(msg, "Vision")
        if section:
            self._vision = section
            self._vision_last_update = self.get_clock().now()

    def control_callback(self, msg):
        section = self._section_from_message(msg, "Control")
        if section:
            self._control = section
            self._control_last_update = self.get_clock().now()

    # ── Simulation generators ───────────────────────────────────────────

    def _sample_jetson(self) -> dict[str, Any]:
        uptime_sec = int(
            (self.get_clock().now() - self._start_stamp).nanoseconds / 1_000_000_000
        )
        cpu_pct = clamp(34.0 + random.gauss(0.0, 6.0), 0.0, 100.0)
        gpu_pct = clamp(18.0 + random.gauss(0.0, 4.0), 0.0, 100.0)
        ram_pct = clamp(58.0 + random.gauss(0.0, 3.5), 0.0, 100.0)
        temp_c = 43.0 + random.gauss(0.0, 1.7)

        mem_total_kb = 0
        mem_available_kb = 0
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as meminfo_file:
                for line in meminfo_file:
                    if line.startswith("MemTotal:"):
                        mem_total_kb = int(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        mem_available_kb = int(line.split()[1])
        except OSError:
            mem_total_kb = 0
            mem_available_kb = 0

        if mem_total_kb > 0 and mem_available_kb > 0:
            ram_pct = clamp(((mem_total_kb - mem_available_kb) / mem_total_kb) * 100.0, 0.0, 100.0)

        bat_pct = clamp(98.0 - (uptime_sec * 0.01), 0.0, 100.0)
        bat_voltage = 10.8 + (bat_pct / 100.0) * 1.2

        thermal_path = "/sys/class/thermal/thermal_zone0/temp"
        try:
            with open(thermal_path, "r", encoding="utf-8") as thermal_file:
                temp_c = float(thermal_file.read().strip()) / 1000.0
        except OSError:
            pass

        return make_jetson_payload(cpu_pct, gpu_pct, ram_pct, temp_c, bat_pct, bat_voltage, uptime_sec)

    def _sample_communication(self) -> dict[str, Any]:
        self._heartbeat_seq += 1
        if random.random() > 0.95:
            self._packet_loss_pct = random.uniform(1.0, 6.0)
        else:
            self._packet_loss_pct = max(0.0, self._packet_loss_pct - 0.25)

        rtt_ms = random.randint(15, 35)
        if random.random() > 0.92:
            rtt_ms = random.randint(80, 180)
        channel_rssi = random.randint(-78, -48)
        stream_fps = 28.5 + random.gauss(0.0, 1.5)
        timestamp_ms = int(
            (self.get_clock().now() - self._start_stamp).nanoseconds / 1_000_000
        )
        return make_communication_payload(
            rtt_ms=rtt_ms,
            channel_rssi=channel_rssi,
            stream_fps=stream_fps,
            packet_loss_pct=self._packet_loss_pct,
            heartbeat_seq=self._heartbeat_seq,
            timestamp_ms=timestamp_ms,
        )

    def _sample_ros(self) -> dict[str, Any]:
        now = self.get_clock().now()
        nav_alive = ((now - self._nav_last_update).nanoseconds / 1e9) < self._nav_stale_sec
        safety_alive = (
            ((now - self._safety_last_update).nanoseconds / 1e9) < self._safety_stale_sec
            and not self._safety_deadline_missed
        )
        vision_alive = ((now - self._vision_last_update).nanoseconds / 1e9) < self._vision_stale_sec
        control_alive = ((now - self._control_last_update).nanoseconds / 1e9) < self._control_stale_sec
        rosout_last = "Telemetry aggregator running"
        if not nav_alive:
            rosout_last = "Waiting for nav telemetry"
        elif not safety_alive:
            rosout_last = "Waiting for safety telemetry"
        elif not vision_alive:
            rosout_last = "Waiting for vision telemetry"
        elif not control_alive:
            rosout_last = "Waiting for motor control heartbeat"

        return make_ros_payload(
            node_lane_det=vision_alive,
            node_obs_avoid=safety_alive,
            node_wp_nav=nav_alive,
            node_img_recog=vision_alive,
            node_motor_ctrl=control_alive,
            rosout_last=rosout_last,
        )

    def build_payload(self) -> dict[str, Any]:
        return {
            "Navigation": self._navigation if self._navigation else make_navigation_payload(0.0, 0.0, 40.0, -75.0, 0.0, 0, 0.0, "idle"),
            "Safety": self._safety if self._safety else make_safety_payload(
                "idle",
                "idle",
                False,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
            ),
            "Vision": self._vision if self._vision else make_vision_payload(0.0, False, False, 0, "idle", False, 0, 0.0),
            "Jetson": self._sample_jetson(),
            "Communication": self._sample_communication(),
            "ROS": self._sample_ros(),
        }

    def publish_telemetry(self):
        payload = self.build_payload()
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._publisher.publish(message)
        return payload

    def timer_callback(self):
        return self.publish_telemetry()


def main(args=None):
    node = TelemetryAggregatorNode()
    run_node(node, fallback_period=0.1)


if __name__ == "__main__":
    main()
