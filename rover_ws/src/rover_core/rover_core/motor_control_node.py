"""Motor control node for rover command intake and heartbeat publishing."""

from __future__ import annotations

import json
import math
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import (
    COMMAND_QOS,
    RELIABLE_QOS,
    Twist,
    _STD_MSGS_AVAILABLE,
    decode_json_message,
    get_topic_path,
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
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self._init_topics()

        # ── Dynamic Config Checker ──
        self._config_mtime = 0.0
        self._check_topic_config()
        self._config_timer = self.create_timer(1.0, self._check_topic_config)

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
    def _init_topics(self) -> None:
        motor_topic = get_topic_path("motor_control", "/rover/commands/motor")
        control_topic = get_topic_path("telemetry_control", "/rover/telemetry/control")
        
        # Command velocity topic parameter override
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        if cmd_vel_topic == "/cmd_vel":
            cmd_vel_topic = get_topic_path("cmd_vel_echo", "/cmd_vel")

        self._command_subscription = self.create_subscription(
            String,
            motor_topic,
            self._motor_command_callback,
            COMMAND_QOS,
        )
        self._heartbeat_publisher = self.create_publisher(
            String,
            control_topic,
            RELIABLE_QOS,
        )
        self._cmd_vel_publisher = None
        if _STD_MSGS_AVAILABLE:
            self._cmd_vel_publisher = self.create_publisher(
                Twist, cmd_vel_topic, COMMAND_QOS
            )
            self.get_logger().info(f"{cmd_vel_topic} publisher created (geometry_msgs/Twist)")

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
            if hasattr(self, "_command_subscription") and self._command_subscription:
                self.destroy_subscription(self._command_subscription)
                self._command_subscription = None
            if hasattr(self, "_heartbeat_publisher") and self._heartbeat_publisher:
                self.destroy_publisher(self._heartbeat_publisher)
                self._heartbeat_publisher = None
            if hasattr(self, "_cmd_vel_publisher") and self._cmd_vel_publisher:
                self.destroy_publisher(self._cmd_vel_publisher)
                self._cmd_vel_publisher = None
            
            self._init_topics()
            self.get_logger().info("MotorControlNode topics updated successfully.")
        except Exception as e:
            self.get_logger().error(f"Error reconfiguring MotorControlNode topics: {e}")
    # ── Command handling ─────────────────────────────────────────────────────

    def _motor_command_callback(self, msg) -> None:
        command = decode_json_message(msg)
        if not command:
            return
        action = str(command.get("action", "")).lower()
        if action == "drive":
            speed = float(command.get("speed_kmh", 0.0))
            speed = min(speed, self._max_speed_kmh)
            command["speed_kmh"] = speed
            self.get_logger().debug("Motor control accepted drive command")
            # Publish standard /cmd_vel: convert km/h → m/s on linear.x
            # heading_deg is mapped to angular.z (positive = turn left, rad/s)
            if self._cmd_vel_publisher is not None:
                twist = Twist()
                heading_deg = float(command.get("heading_deg", 0.0))
                if heading_deg > 180.0:
                    heading_deg -= 360.0
                twist.linear.x = round(speed / 3.6, 4)         # km/h → m/s
                # Map heading offset to angular.z (simple proportional, capped ±1 rad/s)
                angular_z = max(-1.0, min(1.0, math.radians(heading_deg) * 0.3))
                twist.angular.z = round(angular_z, 4)
                self._cmd_vel_publisher.publish(twist)
        elif action == "estop":
            self._estop_latched = True
            self.get_logger().warning("Motor control received emergency stop")
            self._publish_zero_cmd_vel()
        elif action == "resume":
            self._estop_latched = False
            self.get_logger().info("Motor control resumed")
        elif action == "stop":
            self.get_logger().info("Motor control stopping rover")
            self._publish_zero_cmd_vel()
        self._last_command = command

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

    def _publish_zero_cmd_vel(self) -> None:
        """Publish a zero-velocity Twist on /cmd_vel (safe command on stop/estop)."""
        if self._cmd_vel_publisher is not None:
            try:
                self._cmd_vel_publisher.publish(Twist())
            except Exception:
                pass

    def _send_zero_command(self) -> None:
        """Publish a zero-velocity stop before the node exits (anti-pattern #8)."""
        self._publish_zero_cmd_vel()
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
