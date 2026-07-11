"""Motor control node for rover command intake and heartbeat publishing."""

from __future__ import annotations

import json
import math
import queue
import threading
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import (
    COMMAND_QOS,
    RELIABLE_QOS,
    SENSOR_QOS,
    SAFETY_HEARTBEAT_QOS,
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
        self.declare_parameter("esp32_serial_port", "/dev/ttyUSB2")
        self.declare_parameter("esp32_baud_rate", 115200)
        self.declare_parameter("enable_esp32", True)
        self.declare_parameter("use_simulation", True)

        publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._max_speed_kmh: float = (
            self.get_parameter("max_speed_kmh").get_parameter_value().double_value
        )
        self._esp32_port = self.get_parameter("esp32_serial_port").get_parameter_value().string_value
        self._esp32_baud = self.get_parameter("esp32_baud_rate").get_parameter_value().integer_value
        self._enable_esp32 = self.get_parameter("enable_esp32").get_parameter_value().bool_value
        self._use_simulation = self.get_parameter("use_simulation").get_parameter_value().bool_value
        if self._use_simulation:
            self._enable_esp32 = False
        period_sec = 1.0 / max(publish_rate_hz, 0.1)

        # ── Serial Port to ESP32 ─────────────────────────────────────────────
        self._serial_conn = None
        if self._enable_esp32:
            try:
                import serial
                self._serial_conn = serial.Serial(
                    port=self._esp32_port,
                    baudrate=self._esp32_baud,
                    timeout=0.1,
                    write_timeout=0.1
                )
                self.get_logger().info(f"Connected to ESP32 on {self._esp32_port} at {self._esp32_baud} baud.")
            except Exception as e:
                self.get_logger().warning(f"Could not open ESP32 serial port {self._esp32_port}: {e}. Running in simulation/fallback mode.")

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
        self._current_heading_deg = 0.0
        self._safety_blocked = False
        self._active_drive_command = False
        self._drive_speed_kmh = 0.0
        self._drive_heading_deg = 0.0

        self._running = True
        self._serial_queue = queue.Queue(maxsize=10)
        self._serial_thread = None
        if self._serial_conn:
            self._serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
            self._serial_thread.start()

        self._timer = self.create_timer(period_sec, self._timer_callback)
        self.get_logger().info(
            f"Motor control node ready (rate={publish_rate_hz:.1f} Hz, "
            f"max_speed={self._max_speed_kmh:.1f} km/h)"
        )
    def _init_topics(self) -> None:
        motor_topic = get_topic_path("motor_control", "/rover/commands/motor")
        control_topic = get_topic_path("telemetry_control", "/rover/telemetry/control")
        nav_topic = get_topic_path("telemetry_nav", "/rover/telemetry/nav")
        safety_topic = get_topic_path("telemetry_safety", "/rover/telemetry/safety")
        
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
        self._nav_subscription = self.create_subscription(
            String,
            nav_topic,
            self._nav_callback,
            SENSOR_QOS,
        )
        self._safety_subscription = self.create_subscription(
            String,
            safety_topic,
            self._safety_callback,
            SAFETY_HEARTBEAT_QOS,
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
            if hasattr(self, "_nav_subscription") and self._nav_subscription:
                self.destroy_subscription(self._nav_subscription)
                self._nav_subscription = None
            if hasattr(self, "_safety_subscription") and self._safety_subscription:
                self.destroy_subscription(self._safety_subscription)
                self._safety_subscription = None
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

    def _serial_worker(self) -> None:
        while self._running:
            try:
                cmd = self._serial_queue.get(timeout=0.05)
                if self._serial_conn and self._serial_conn.is_open:
                    try:
                        self._serial_conn.write(cmd)
                        self._serial_conn.flush()
                    except Exception as e:
                        self.get_logger().error(f"Serial write error in worker: {e}")
            except queue.Empty:
                continue

    def _queue_serial_write(self, data: bytes) -> None:
        if not self._serial_conn:
            return
        if data == b"X\n":
            while not self._serial_queue.empty():
                try:
                    self._serial_queue.get_nowait()
                except Exception:
                    break
        try:
            self._serial_queue.put_nowait(data)
        except queue.Full:
            try:
                self._serial_queue.get_nowait()
                self._serial_queue.put_nowait(data)
            except Exception:
                pass

    def _nav_callback(self, msg) -> None:
        nav_data = decode_json_message(msg)
        if nav_data:
            self._current_heading_deg = float(nav_data.get("heading_deg", 0.0))

    def _safety_callback(self, msg) -> None:
        safety_data = decode_json_message(msg)
        if safety_data:
            self._safety_blocked = bool(
                safety_data.get("estop_triggered")
                or safety_data.get("collision_detected")
                or safety_data.get("is_blocked")
            )
            if self._safety_blocked:
                self._send_serial_estop()

    def _send_serial_estop(self) -> None:
        self._queue_serial_write(b"X\n")

    def _pwm_to_rover_char(self, left_pwm: int, right_pwm: int) -> str:
        threshold = 30
        if abs(left_pwm) < threshold and abs(right_pwm) < threshold:
            return 'X'
        if left_pwm >= threshold and right_pwm >= threshold:
            ratio = left_pwm / max(right_pwm, 1)
            if 0.6 <= ratio <= 1.4:
                return 'W'
            elif ratio < 0.6:
                return 'Q'
            else:
                return 'E'
        elif left_pwm <= -threshold and right_pwm <= -threshold:
            ratio = abs(left_pwm) / max(abs(right_pwm), 1)
            if 0.6 <= ratio <= 1.4:
                return 'S'
            elif ratio < 0.6:
                return 'Z'
            else:
                return 'C'
        elif left_pwm <= -threshold and right_pwm >= threshold:
            return 'A'
        elif left_pwm >= threshold and right_pwm <= -threshold:
            return 'D'
        return 'X'

    def _publish_drive_command(self) -> None:
        if self._cmd_vel_publisher is None:
            return
        target_heading = self._drive_heading_deg
        error_deg = (target_heading - self._current_heading_deg + 540.0) % 360.0 - 180.0

        twist = Twist()
        twist.linear.x = round(self._drive_speed_kmh / 3.6, 4)
        angular_z = max(-1.0, min(1.0, -math.radians(error_deg) * 0.4))
        twist.angular.z = round(angular_z, 4)
        self._cmd_vel_publisher.publish(twist)

        max_speed_mps = self._max_speed_kmh / 3.6
        if max_speed_mps <= 0:
            return
        left_mps = twist.linear.x - twist.angular.z * 0.25
        right_mps = twist.linear.x + twist.angular.z * 0.25

        max_req = max(abs(left_mps), abs(right_mps))
        if max_req > max_speed_mps:
            scale = max_speed_mps / max_req
            left_mps *= scale
            right_mps *= scale

        left_pwm = int((left_mps / max_speed_mps) * 255)
        right_pwm = int((right_mps / max_speed_mps) * 255)

        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        if self._serial_conn and self._serial_conn.is_open and not self._safety_blocked:
            rover_char = self._pwm_to_rover_char(left_pwm, right_pwm)
            self._queue_serial_write(f"{rover_char}\n".encode("utf-8"))

    def _motor_command_callback(self, msg) -> None:
        command = decode_json_message(msg)
        if not command:
            return
        action = str(command.get("action", "")).lower()
        if action == "drive":
            speed = float(command.get("speed_kmh", 0.0))
            speed = min(speed, self._max_speed_kmh)
            self._drive_speed_kmh = speed
            self._drive_heading_deg = float(command.get("heading_deg", 0.0))
            self._active_drive_command = True
            command["speed_kmh"] = speed
            self.get_logger().debug("Motor control accepted drive command")
            self._publish_drive_command()
        elif action == "estop":
            self._active_drive_command = False
            self._estop_latched = True
            self.get_logger().warning("Motor control received emergency stop")
            self._publish_zero_cmd_vel()
            self._send_serial_estop()
        elif action == "resume":
            self._estop_latched = False
            self.get_logger().info("Motor control resumed")
            if self._active_drive_command and self._serial_conn and self._serial_conn.is_open:
                self._publish_drive_command()
        elif action == "stop":
            self._active_drive_command = False
            self.get_logger().info("Motor control stopping rover")
            self._publish_zero_cmd_vel()
            self._send_serial_estop()
        self._last_command = command

    # ── Timer ────────────────────────────────────────────────────────────────

    def _timer_callback(self) -> None:
        self._heartbeat_seq += 1
        now_stamp = self.get_clock().now()
        uptime_sec = int(
            (now_stamp - self._start_stamp).nanoseconds / 1_000_000_000
        )
        timestamp_ms = int(now_stamp.nanoseconds / 1_000_000)

        if self._active_drive_command and not self._safety_blocked:
            self._publish_drive_command()

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
        self._running = False
        if self._serial_thread:
            self._serial_thread.join(timeout=1.0)
        if self._serial_conn and self._serial_conn.is_open:
            try:
                self._serial_conn.write(b"X\n")
                self._serial_conn.flush()
                self._serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    node = MotorControlNode()
    run_node(node, fallback_period=0.2)


if __name__ == "__main__":
    main()
