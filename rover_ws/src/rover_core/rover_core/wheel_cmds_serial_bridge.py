#!/usr/bin/env python3
"""Node C: Serial bridge between ROS2 /cmd_vel and the ESP32 firmware.

Subscribes to:
- /cmd_vel (geometry_msgs/Twist)
- /rover/hud (std_msgs/String - JSON for armed status and drift active)
- /rover/telemetry/safety (std_msgs/String - JSON for collision/estop state)

Publishes:
- /rover/telemetry/control (std_msgs/String - JSON heartbeat)
"""

from __future__ import annotations

import json
import queue
import threading
import time
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import COMMAND_QOS, SENSOR_QOS, RELIABLE_QOS, now_ms

try:
    from geometry_msgs.msg import Twist
except ImportError:
    Twist = None


class WheelCmdsSerialBridge(Node):
    """Bridges Twist commands directly to ESP32 serial port at 50 Hz, with E-stops & heartbeats."""

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("wheel_cmds_serial_bridge")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("esp32_serial_port", "/dev/ttyUSB2")
        self.declare_parameter("esp32_baud_rate", 115200)
        self.declare_parameter("enable_esp32", True)
        self.declare_parameter("use_simulation", True)
        self.declare_parameter("control_rate_hz", 50.0)

        # Get values
        self._esp32_port = self.get_parameter("esp32_serial_port").value
        self._esp32_baud = self.get_parameter("esp32_baud_rate").value
        self._enable_esp32 = self.get_parameter("enable_esp32").value
        self._use_simulation = self.get_parameter("use_simulation").value
        self._control_rate_hz = self.get_parameter("control_rate_hz").value

        if self._use_simulation:
            self._enable_esp32 = False

        # ── Serial Port Setup ────────────────────────────────────────────────
        self._serial_conn = None
        if self._enable_esp32:
            try:
                import serial
                self._serial_conn = serial.Serial(
                    port=self._esp32_port,
                    baudrate=self._esp32_baud,
                    timeout=0.05,
                    write_timeout=0.05
                )
                self.get_logger().info(f"Connected to ESP32 on {self._esp32_port} at {self._esp32_baud} baud.")
            except Exception as e:
                self.get_logger().warning(
                    f"Could not open ESP32 serial port {self._esp32_port}: {e}. "
                    "Running in simulation/fallback mode."
                )
                self._enable_esp32 = False

        # ── Threaded Serial Worker ───────────────────────────────────────────
        self._running = True
        self._serial_queue = queue.Queue(maxsize=10)
        self._serial_thread = None
        if self._serial_conn:
            self._serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
            self._serial_thread.start()

        # ── State variables ──────────────────────────────────────────────────
        self._last_linear_x = 0.0
        self._last_angular_z = 0.0
        self._armed = False
        self._last_armed_state = False
        self._drift_active = False
        self._safety_blocked = False
        self._need_reset = False
        self._heartbeat_seq = 0
        self._start_stamp = self.get_clock().now()

        # ── Pub/Sub ──────────────────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist if Twist else Any,
            "/cmd_vel",
            self.cmd_vel_callback,
            COMMAND_QOS
        )

        self.hud_sub = self.create_subscription(
            String,
            "/rover/hud",
            self.hud_callback,
            RELIABLE_QOS
        )

        self.safety_sub = self.create_subscription(
            String,
            "/rover/telemetry/safety",
            self.safety_callback,
            RELIABLE_QOS
        )

        self.control_pub = self.create_publisher(
            String,
            "/rover/telemetry/control",
            RELIABLE_QOS
        )

        # ── Timers ───────────────────────────────────────────────────────────
        # Main control loop to send serial packets at 50 Hz
        period_sec = 1.0 / self._control_rate_hz
        self.control_timer = self.create_timer(period_sec, self.control_loop)

        # Heartbeat loop at 5 Hz (standard control telemetry rate)
        self.heartbeat_timer = self.create_timer(0.2, self.publish_heartbeat)

        self.get_logger().info("WheelCmdsSerialBridge initialized.")

    def cmd_vel_callback(self, msg: Any) -> None:
        """Receive latest cmd_vel linear and angular inputs."""
        self._last_linear_x = float(msg.linear.x)
        self._last_angular_z = float(msg.angular.z)

    def hud_callback(self, msg: Any) -> None:
        """Receive HUD updates to track arm status and drift active state."""
        try:
            data = json.loads(msg.data)
            armed = bool(data.get("armed", False))
            self._drift_active = bool(data.get("drift_active", False))
            if armed and not self._last_armed_state:
                # Arming transition: trigger serial reset
                self._need_reset = True
                self.get_logger().info("NFS Serial Bridge: Arming detected, queueing ESP32 reset.")
            elif not armed and self._last_armed_state:
                self.get_logger().info("NFS Serial Bridge: Disarming detected.")
            self._last_armed_state = armed
            self._armed = armed
        except Exception:
            pass

    def safety_callback(self, msg: Any) -> None:
        """Receive safety node updates to track E-stop/blocked transitions."""
        try:
            data = json.loads(msg.data)
            blocked = bool(
                data.get("estop_triggered") or
                data.get("collision_detected") or
                data.get("is_blocked")
            )
            if not blocked and self._safety_blocked:
                # Transition from safety block to clear: queue serial reset
                self._need_reset = True
                self.get_logger().info("NFS Serial Bridge: Safety cleared, queueing ESP32 reset.")
            self._safety_blocked = blocked
        except Exception:
            pass

    def _serial_worker(self) -> None:
        """Background thread writing data to ESP32 serial port."""
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
        """Safely push data to the serial queue, dropping old elements on full."""
        if not self._serial_conn:
            return
        try:
            self._serial_queue.put_nowait(data)
        except queue.Full:
            try:
                self._serial_queue.get_nowait()
                self._serial_queue.put_nowait(data)
            except Exception:
                pass

    def control_loop(self) -> None:
        """Primary 50 Hz control loop. Formats and writes framed packets."""
        # 1. Handle Safety E-stop / Disarmed states
        if not self._armed or self._safety_blocked:
            self._queue_serial_write(b"E\n")  # Fallback backward-compatible single character
            self._queue_serial_write(b"<ESTOP>\n")
            return

        # 2. Check if a Reset frame is required (arm/safety release)
        if self._need_reset:
            self._queue_serial_write(b"R\n")  # Fallback backward-compatible
            self._queue_serial_write(b"<RESET>\n")
            self._need_reset = False

        # 3. Form and send the Twist linear and angular packet: <linear_x, angular_z, drift_active, checksum>
        linear_x_int = int(self._last_linear_x * 1000)
        angular_z_int = int(self._last_angular_z * 1000)
        drift_active_int = 1 if self._drift_active else 0
        checksum = (linear_x_int + angular_z_int + drift_active_int) & 0xFF

        packet = f"<{linear_x_int},{angular_z_int},{drift_active_int},{checksum}>\n"
        self._queue_serial_write(packet.encode("utf-8"))

    def publish_heartbeat(self) -> None:
        """Publish compatibility control heartbeat to satisfy telemetry aggregator."""
        self._heartbeat_seq += 1
        now_stamp = self.get_clock().now()
        uptime_sec = int(
            (now_stamp - self._start_stamp).nanoseconds / 1_000_000_000
        )
        timestamp_ms = int(now_stamp.nanoseconds / 1_000_000)

        # Mirror standard MotorControlNode telemetry layout
        payload = {
            "node_motor_ctrl": True,
            "heartbeat_seq": self._heartbeat_seq,
            "estop_latched": self._safety_blocked,
            "last_command": {
                "action": "drive" if self._armed else "stop",
                "linear_x": self._last_linear_x,
                "angular_z": self._last_angular_z,
                "drift_active": self._drift_active,
            },
            "uptime_sec": uptime_sec,
            "timestamp_ms": timestamp_ms,
            "esp32_connected": self._serial_conn is not None and self._serial_conn.is_open,
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.control_pub.publish(msg)

    def shutdown(self) -> None:
        """Properly close serial thread during exit."""
        self._running = False
        if self._serial_conn:
            try:
                self._serial_conn.close()
            except Exception:
                pass


def main(args=None):
    node = WheelCmdsSerialBridge()
    try:
        run_node(node, fallback_period=0.02)
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
