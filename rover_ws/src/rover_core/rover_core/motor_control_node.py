"""Motor control node for rover command intake and heartbeat publishing."""

from __future__ import annotations

import json
import time
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, rclpy, run_node
from rover_core.telemetry_utils import (
    COMMAND_QOS,
    RELIABLE_QOS,
    decode_json_message,
)

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


class MotorControlNode(Node):
    """Command-handling node that tracks motor state and publishes heartbeats.

    QoS rationale
    -------------
    - ``/rover/commands/motor`` subscriber: RELIABLE so no command is silently
      dropped on a congested link.
    - ``/rover/telemetry/control`` publisher: RELIABLE with depth-10 so the
      telemetry aggregator always receives heartbeats even under brief load.
    """

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("motor_control_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter(
            "publish_rate_hz",
            5.0,
            _float_descriptor("Motor heartbeat publish rate in Hz", 1.0, 50.0),
        )
        self.declare_parameter(
            "max_speed_kmh",
            15.0,
            _float_descriptor("Maximum speed limit in km/h", 0.0, 50.0),
        )

        publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._max_speed_kmh: float = (
            self.get_parameter("max_speed_kmh").get_parameter_value().double_value
        )
        period_sec = 1.0 / max(publish_rate_hz, 0.1)

        # ── Pub / Sub ────────────────────────────────────────────────────────
        # Command subscriber: RELIABLE — must not drop any motor command.
        self._command_subscription = self.create_subscription(
            String,
            "/rover/commands/motor",
            self._motor_command_callback,
            COMMAND_QOS,
        )
        # Heartbeat publisher: RELIABLE diagnostics stream.
        self._heartbeat_publisher = self.create_publisher(
            String,
            "/rover/telemetry/control",
            RELIABLE_QOS,
        )

        # ── State ────────────────────────────────────────────────────────────
        # Use node clock (respects use_sim_time) for all timing.
        self._start_stamp = self.get_clock().now()
        self._heartbeat_seq = 0
        self._last_command: dict[str, Any] = {}
        self._estop_latched = False

        self._timer = self.create_timer(period_sec, self._timer_callback)
        self.get_logger().info(
            f"Motor control node ready (rate={publish_rate_hz:.1f} Hz, "
            f"max_speed={self._max_speed_kmh:.1f} km/h)"
        )

    # ── Command handling ─────────────────────────────────────────────────────

    def _motor_command_callback(self, msg) -> None:
        command = decode_json_message(msg)
        if not command:
            return
        self._last_command = command
        action = str(command.get("action", "")).lower()
        if action == "estop":
            self._estop_latched = True
            self.get_logger().warning("Motor control received emergency stop")
        elif action == "resume":
            self._estop_latched = False
            self.get_logger().info("Motor control resumed")
        elif action == "stop":
            self.get_logger().info("Motor control stopping rover")
        elif action == "drive":
            self.get_logger().debug("Motor control accepted drive command")

    # ── Timer ────────────────────────────────────────────────────────────────

    def _timer_callback(self) -> None:
        self._heartbeat_seq += 1
        now_stamp = self.get_clock().now()
        uptime_sec = int(
            (now_stamp - self._start_stamp).nanoseconds / 1_000_000_000
        )
        timestamp_ms = int(now_stamp.nanoseconds / 1_000_000)

        payload = {
            "node_motor_ctrl": True,
            "heartbeat_seq": self._heartbeat_seq,
            "estop_latched": self._estop_latched,
            "last_command": self._last_command,
            "uptime_sec": uptime_sec,
            "timestamp_ms": timestamp_ms,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._heartbeat_publisher.publish(message)
        return payload

    # ── Public aliases ─────────────────────────────────────────────────────────────

    def timer_callback(self):
        """Public alias for backwards-compatibility and tests."""
        return self._timer_callback()

    # ── Shutdown safety ──────────────────────────────────────────────────────

    def _send_zero_command(self) -> None:
        """Publish a zero-velocity stop before the node exits (anti-pattern #8)."""
        try:
            stop_payload = {"action": "stop", "speed_kmh": 0.0}
            msg = String()
            msg.data = json.dumps(stop_payload, separators=(",", ":"))
            self._heartbeat_publisher.publish(msg)
        except Exception:
            pass

    def destroy_node(self) -> None:
        self._send_zero_command()
        super().destroy_node()


def main(args=None):
    node = MotorControlNode()
    run_node(node, fallback_period=0.2)


if __name__ == "__main__":
    main()
