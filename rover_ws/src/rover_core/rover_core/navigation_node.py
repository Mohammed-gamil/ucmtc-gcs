"""Navigation node for rover position, heading, and motion telemetry."""

from __future__ import annotations

import json
import math
import random
import time
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, rclpy, run_node
from rover_core.telemetry_utils import (
    COMMAND_QOS,
    SAFETY_HEARTBEAT_QOS,
    SENSOR_QOS,
    decode_json_message,
    make_navigation_payload,
    random_navigation_defaults,
)

# SubscriptionOptions with deadline callback is only available in real rclpy.
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


class NavigationNode(Node):
    """ROS 2 node for rover navigation telemetry and drive command tracking.

    QoS rationale
    -------------
    - ``/rover/commands/motor`` subscriber: COMMAND_QOS — RELIABLE + 200ms
      LIFESPAN so stale drive commands are discarded.
    - ``/rover/telemetry/safety`` subscriber: SAFETY_HEARTBEAT_QOS — must
      match the SafetyNode publisher profile exactly (RELIABLE + DEADLINE +
      LIFESPAN). When no safety message arrives within 500 ms the DEADLINE
      event fires and this node immediately engages a safe stop.
    - ``/rover/telemetry/nav`` publisher: SENSOR_QOS — BEST_EFFORT stream.

    Clock usage
    -----------
    All timestamps use ``self.get_clock().now()`` so the node behaves
    correctly with ``use_sim_time:=true`` during Gazebo/Isaac Sim runs.
    """

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("navigation_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter(
            "publish_rate_hz",
            10.0,
            _float_descriptor("Nav telemetry publish rate in Hz", 1.0, 50.0),
        )
        self.declare_parameter("gps_serial_port", "/dev/ttyUSB0")
        self.declare_parameter("imu_serial_port", "/dev/ttyUSB1")
        self.declare_parameter(
            "stale_data_timeout_sec",
            1.5,
            _float_descriptor("Seconds after which section data is considered stale", 0.1, 10.0),
        )

        publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        period_sec = 1.0 / max(publish_rate_hz, 0.1)

        # ── Pub / Sub ────────────────────────────────────────────────────────
        # Nav telemetry: BEST_EFFORT sensor stream.
        self._publisher = self.create_publisher(
            String,
            "/rover/telemetry/nav",
            SENSOR_QOS,
        )
        # Motor commands: RELIABLE + LIFESPAN.
        self._command_subscription = self.create_subscription(
            String,
            "/rover/commands/motor",
            self._motor_command_callback,
            COMMAND_QOS,
        )
        # Safety state: SAFETY_HEARTBEAT_QOS — must match SafetyNode publisher.
        # A deadline callback fires when no safety message arrives in 500 ms.
        self._safety_subscription = self._create_safety_subscription()

        # ── State ────────────────────────────────────────────────────────────
        self._start_stamp = self.get_clock().now()
        initial_state = random_navigation_defaults()
        self._speed_kmh = initial_state["speed_kmh"]
        self._target_speed_kmh = 0.0
        self._heading_deg = initial_state["heading_deg"]
        self._target_heading_deg = 0.0
        self._pos_lat = initial_state["pos_lat"]
        self._pos_lon = initial_state["pos_lon"]
        self._dist_traveled_m = initial_state["dist_traveled_m"]
        self._wp_current = initial_state["wp_current"]
        self._wp_error_m = initial_state["wp_error_m"]
        self._wp_status = initial_state["wp_status"]
        self._safety_blocked = False
        self._last_command: dict[str, Any] = {}

        self._timer = self.create_timer(period_sec, self._timer_callback)
        self.get_logger().info(
            f"Navigation node ready (rate={publish_rate_hz:.1f} Hz)"
        )

    # ── Safety subscription with deadline callback ───────────────────────────

    def _create_safety_subscription(self):
        """Create /rover/telemetry/safety subscription with DEADLINE event handler.

        When no safety message arrives within 500 ms the rover immediately
        enters a blocked state — this is the safe fallback for a dead safety
        node, network partition, or software hang.
        """
        if _QOS_EVENTS_AVAILABLE:
            event_callbacks = SubscriptionEventCallbacks()
            event_callbacks.deadline = self._on_safety_deadline_missed
            return self.create_subscription(
                String,
                "/rover/telemetry/safety",
                self._safety_callback,
                SAFETY_HEARTBEAT_QOS,
                event_callbacks=event_callbacks,
            )
        # Fallback: no QoS event support — use matching QoS but no callback.
        return self.create_subscription(
            String,
            "/rover/telemetry/safety",
            self._safety_callback,
            SAFETY_HEARTBEAT_QOS,
        )

    def _on_safety_deadline_missed(self, event) -> None:
        """Called when no safety heartbeat arrives within 500 ms.

        Safe action: engage blocked state until safety messages resume.
        This is equivalent to treating a silent safety node as an e-stop.
        """
        self.get_logger().error(
            f"Safety heartbeat DEADLINE MISSED "
            f"(total={getattr(event, 'total_count', '?')}, "
            f"delta={getattr(event, 'total_count_change', '?')}). "
            "Engaging safe-stop until safety messages resume."
        )
        self._safety_blocked = True
        self._target_speed_kmh = 0.0
        self._wp_status = "blocked"

    # ── Command handling ─────────────────────────────────────────────────────

    def _motor_command_callback(self, msg) -> None:
        command = decode_json_message(msg)
        if not command:
            return

        self._last_command = command
        action = str(command.get("action", "")).lower()
        if action == "estop":
            self._target_speed_kmh = 0.0
            self._wp_status = "blocked"
            return
        if action == "stop":
            self._target_speed_kmh = 0.0
            self._wp_status = "idle"
            return
        if action == "resume":
            self._wp_status = "navigating" if self._target_speed_kmh > 0 else "idle"
            return

        speed = command.get("speed_kmh")
        heading = command.get("heading_deg")
        waypoint = command.get("wp_current")
        wp_status = command.get("wp_status")
        if isinstance(speed, (int, float)) and not isinstance(speed, bool):
            self._target_speed_kmh = max(0.0, float(speed))
        if isinstance(heading, (int, float)) and not isinstance(heading, bool):
            self._target_heading_deg = float(heading) % 360.0
        if isinstance(waypoint, int) and not isinstance(waypoint, bool):
            self._wp_current = waypoint
        if isinstance(wp_status, str) and wp_status:
            self._wp_status = wp_status
        elif self._target_speed_kmh > 0.0:
            self._wp_status = "navigating"

    def _safety_callback(self, msg) -> None:
        safety = decode_json_message(msg)
        if not safety:
            return
        # Safety messages arriving means the node is alive — clear deadline flag.
        was_blocked_by_deadline = self._safety_blocked and not (
            safety.get("estop_triggered")
            or safety.get("collision_detected")
            or safety.get("is_blocked")
        )
        self._safety_blocked = bool(
            safety.get("estop_triggered")
            or safety.get("collision_detected")
            or safety.get("is_blocked")
        )
        if was_blocked_by_deadline and not self._safety_blocked:
            self.get_logger().info("Safety heartbeat resumed — releasing deadline block.")
        if self._safety_blocked:
            self._target_speed_kmh = 0.0
            self._wp_status = "blocked"

    # ── Publish ──────────────────────────────────────────────────────────────

    def _publish_nav_data(self) -> dict:
        if self._safety_blocked:
            self._target_speed_kmh = 0.0

        self._speed_kmh += (self._target_speed_kmh - self._speed_kmh) * 0.25
        self._heading_deg += (
            (self._target_heading_deg - self._heading_deg + 540.0) % 360.0 - 180.0
        ) * 0.15
        self._heading_deg %= 360.0

        if self._target_speed_kmh <= 0.0 and not self._safety_blocked:
            self._speed_kmh = max(0.0, self._speed_kmh - 0.1)

        # Use node clock for elapsed-time-based calculations so sim time works.
        now_stamp = self.get_clock().now()
        elapsed_since_start = (
            (now_stamp - self._start_stamp).nanoseconds / 1_000_000_000
        )

        # dt is nominally 1/publish_rate_hz; use a fixed 100ms to avoid drift.
        dt = 0.1
        distance_increment = (self._speed_kmh / 3.6) * dt
        heading_rad = math.radians(self._heading_deg)
        self._pos_lat += (distance_increment / 111000.0) * math.cos(heading_rad)
        self._pos_lon += (distance_increment / 111000.0) * math.sin(heading_rad)
        self._dist_traveled_m += distance_increment
        self._wp_error_m = max(
            0.0,
            abs(math.sin(elapsed_since_start / 4.0)) * (1.0 + random.random()),
        )
        if self._speed_kmh <= 0.1:
            self._wp_status = "idle" if not self._safety_blocked else "blocked"
        elif self._wp_status == "idle":
            self._wp_status = "navigating"

        payload = make_navigation_payload(
            speed_kmh=self._speed_kmh,
            heading_deg=self._heading_deg,
            pos_lat=self._pos_lat,
            pos_lon=self._pos_lon,
            dist_traveled_m=self._dist_traveled_m,
            wp_current=self._wp_current,
            wp_error_m=self._wp_error_m,
            wp_status=self._wp_status,
        )
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._publisher.publish(message)
        return payload

    def _timer_callback(self) -> None:
        self._publish_nav_data()

    # ── Backwards-compat aliases used in tests ────────────────────────────────
    def motor_command_callback(self, msg):
        return self._motor_command_callback(msg)

    def safety_callback(self, msg):
        return self._safety_callback(msg)

    def publish_nav_data(self):
        return self._publish_nav_data()

    def timer_callback(self):
        return self._timer_callback()


def main(args=None):
    node = NavigationNode()
    run_node(node, fallback_period=0.1)


if __name__ == "__main__":
    main()
