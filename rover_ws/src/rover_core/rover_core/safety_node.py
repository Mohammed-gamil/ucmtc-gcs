"""Safety node for emergency stop, collision detection, and rover blocking."""

from __future__ import annotations

import json
import random
import time
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, rclpy, run_node
from rover_core.telemetry_utils import (
    COMMAND_QOS,
    SAFETY_HEARTBEAT_QOS,
    decode_json_message,
    make_safety_payload,
)

# rclpy.parameter imports are optional — ParameterDescriptor is available only
# on real ROS 2. Fall back gracefully for local dev / unit tests.
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


class SafetyNode(Node):
    """ROS 2 node for rover safety monitoring and command override logic.

    QoS rationale
    -------------
    - ``/rover/commands/motor`` subscriber: RELIABLE — must never miss an
      e-stop command.
    - ``/rover/telemetry/safety`` publisher: SAFETY_HEARTBEAT_QOS —
      RELIABLE with a 500 ms DEADLINE and 1 s LIFESPAN.

      DEADLINE: fires a ``RequestedDeadlineMissed`` event on any subscriber
      that has not received a message within 500 ms, allowing navigation and
      the aggregator to engage a safe fallback mode. Also fires on the
      publisher side (``OfferedDeadlineMissed``) if the timer is somehow
      delayed, alerting operators.

      LIFESPAN: messages older than 1 s are discarded before delivery so
      subscribers cannot act on stale safety state after a comm recovery.

    Clock usage
    -----------
    All timestamps use ``self.get_clock().now()`` (rclpy-compatible) instead
    of ``time.time()`` so the node works correctly with ``use_sim_time:=true``
    when running in Gazebo or Isaac Sim.
    """

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("safety_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter(
            "publish_rate_hz",
            20.0,
            _float_descriptor("Safety check and publish rate in Hz", 10.0, 100.0),
        )
        self.declare_parameter(
            "estop_gpio_pin",
            17,
            _int_descriptor("BCM GPIO pin number for hardware e-stop button", 0, 27),
        )
        self.declare_parameter(
            "collision_threshold_m",
            0.3,
            _float_descriptor("Obstacle distance (m) that triggers a collision event", 0.1, 2.0),
        )
        self.declare_parameter(
            "safety_deadline_ms",
            500,
            _int_descriptor("DEADLINE QoS for /rover/telemetry/safety (ms)", 50, 2000),
        )

        publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._collision_threshold_m: float = (
            self.get_parameter("collision_threshold_m").get_parameter_value().double_value
        )
        period_sec = 1.0 / max(publish_rate_hz, 1.0)

        # ── Pub / Sub ────────────────────────────────────────────────────────
        # Safety telemetry: SAFETY_HEARTBEAT_QOS — RELIABLE + DEADLINE(500ms) + LIFESPAN(1s)
        self._publisher = self.create_publisher(
            String,
            "/rover/telemetry/safety",
            SAFETY_HEARTBEAT_QOS,
        )
        # Motor commands: RELIABLE + LIFESPAN(200ms) — never miss an e-stop,
        # discard stale commands.
        self._command_subscription = self.create_subscription(
            String,
            "/rover/commands/motor",
            self._motor_command_callback,
            COMMAND_QOS,
        )

        # ── State ────────────────────────────────────────────────────────────
        # Use node clock (respects use_sim_time) for all timing.
        self._start_stamp = self.get_clock().now()
        self._mode = "idle"
        self._light_state = "idle"
        self._estop_mech_armed = False
        self._estop_wire_armed = False
        self._estop_triggered = False
        self._is_blocked = False
        self._collision_detected = False
        self._border_crossed = False
        self._border_partial = False
        self._obstacle_touched = False
        # Wall-clock used for collision timers (independent of sim time)
        self._collision_until: float = 0.0
        self._last_command: dict[str, Any] = {}

        self._timer = self.create_timer(period_sec, self._timer_callback)
        self.get_logger().info(
            f"Safety node ready (rate={publish_rate_hz:.1f} Hz, "
            f"DEADLINE=500ms, LIFESPAN=1s)"
        )

    # ── Hardware / sensor simulation ─────────────────────────────────────────

    def _check_estop_hardware(self) -> dict[str, Any]:
        now_wall = time.monotonic()
        self._collision_detected = now_wall < self._collision_until

        speed_val = 0.0
        if self._last_command.get("action") == "drive":
            try:
                raw_speed = self._last_command.get("speed_kmh", 0.0)
                if raw_speed is not None:
                    speed_val = float(raw_speed)
            except (ValueError, TypeError):
                speed_val = 0.0

        if not self._estop_triggered and not self._collision_detected:
            if speed_val > 6.0:
                if random.random() < 0.01:
                    self._collision_detected = True
                    self._collision_until = now_wall + 1.0
            elif random.random() < 0.002:
                self._collision_detected = True
                self._collision_until = now_wall + 0.75

        self._obstacle_touched = self._collision_detected and random.random() > 0.5
        self._border_partial = self._collision_detected and random.random() > 0.7
        self._border_crossed = self._collision_detected and random.random() > 0.9

        if self._estop_triggered:
            self._mode = "estop"
            self._light_state = "red"
            self._is_blocked = True
        elif self._collision_detected:
            self._mode = "collision"
            self._light_state = "red"
            self._is_blocked = True
        elif self._last_command.get("action") in {"drive", "resume"}:
            self._mode = "monitoring"
            self._light_state = "green"
            self._is_blocked = False
        else:
            self._mode = "idle"
            self._light_state = "idle"
            self._is_blocked = False

        return {
            "estop_button_pressed": self._estop_triggered,
            "estop_wire_broken": self._estop_wire_armed and self._estop_triggered,
            "collision_front": self._collision_detected,
            "collision_rear": False,
            "collision_left": False,
            "collision_right": False,
            "hardware_fault": self._estop_triggered or self._collision_detected,
            "fault_message": (
                "estop" if self._estop_triggered
                else ("collision" if self._collision_detected else "ok")
            ),
        }

    # ── Command handling ─────────────────────────────────────────────────────

    def _motor_command_callback(self, msg) -> None:
        command = decode_json_message(msg)
        if not command:
            return

        self._last_command = command
        action = str(command.get("action", "")).lower()
        if action == "estop":
            self._estop_triggered = True
            self._collision_until = max(self._collision_until, time.monotonic())
            self.get_logger().warning("Emergency stop latched from command")
        elif action == "resume":
            if not self._collision_detected:
                self._estop_triggered = False
                self._is_blocked = False
                self.get_logger().info("Safety latch cleared")
        elif action == "stop":
            self._is_blocked = True
        elif action == "drive":
            self._mode = "monitoring"

    # ── Publish ──────────────────────────────────────────────────────────────

    def _publish_safety_state(self) -> dict:
        self._check_estop_hardware()
        # Compute uptime using node clock (honours use_sim_time).
        now_stamp = self.get_clock().now()
        uptime_sec = int(
            (now_stamp - self._start_stamp).nanoseconds / 1_000_000_000
        )
        payload = make_safety_payload(
            mode=self._mode,
            light_state=self._light_state,
            estop_mech_armed=self._estop_mech_armed,
            estop_wire_armed=self._estop_wire_armed,
            estop_triggered=self._estop_triggered,
            is_blocked=self._is_blocked,
            collision_detected=self._collision_detected,
            border_crossed=self._border_crossed,
            border_partial=self._border_partial,
            obstacle_touched=self._obstacle_touched,
        )
        # Embed uptime in payload for diagnostics.
        payload["uptime_sec"] = uptime_sec
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._publisher.publish(message)
        return payload

    def _timer_callback(self) -> None:
        self._publish_safety_state()

    # ── Shutdown safety ──────────────────────────────────────────────────────

    def _send_zero_command(self) -> None:
        """Publish an e-stop safety state before exit so the rover halts."""
        try:
            stop_payload = make_safety_payload(
                mode="estop",
                light_state="red",
                estop_mech_armed=False,
                estop_wire_armed=False,
                estop_triggered=True,
                is_blocked=True,
                collision_detected=False,
                border_crossed=False,
                border_partial=False,
                obstacle_touched=False,
            )
            msg = String()
            msg.data = json.dumps(stop_payload, separators=(",", ":"))
            self._publisher.publish(msg)
        except Exception:
            pass

    def destroy_node(self) -> None:
        self._send_zero_command()
        super().destroy_node()

    # ── Backwards-compat aliases used in tests ────────────────────────────────
    def check_estop_hardware(self):
        return self._check_estop_hardware()

    def publish_safety_state(self):
        return self._publish_safety_state()

    def motor_command_callback(self, msg):
        return self._motor_command_callback(msg)

    def timer_callback(self):
        return self._timer_callback()


def main(args=None):
    node = SafetyNode()
    run_node(node, fallback_period=0.05)


if __name__ == "__main__":
    main()
