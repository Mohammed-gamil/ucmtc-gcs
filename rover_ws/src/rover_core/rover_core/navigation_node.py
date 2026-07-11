"""Navigation node for rover position, heading, and motion telemetry."""

from __future__ import annotations

import json
import math
import random
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import (
    COMMAND_QOS,
    SAFETY_HEARTBEAT_QOS,
    SENSOR_QOS,
    decode_json_message,
    make_navigation_payload,
    random_navigation_defaults,
    get_topic_path,
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
        self.declare_parameter("use_simulation", True)

        publish_rate_hz: float = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self.use_simulation = self.get_parameter("use_simulation").get_parameter_value().bool_value
        self._gps_port = self.get_parameter("gps_serial_port").get_parameter_value().string_value
        self._imu_port = self.get_parameter("imu_serial_port").get_parameter_value().string_value
        period_sec = 1.0 / max(publish_rate_hz, 0.1)

        # ── Serial Connections ───────────────────────────────────────────────
        self._gps_conn = None
        self._imu_conn = None
        if not self.use_simulation:
            try:
                import serial
                self._gps_conn = serial.Serial(self._gps_port, 9600, timeout=0.1)
                self.get_logger().info(f"Opened GPS serial port: {self._gps_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to open GPS serial port {self._gps_port}: {e}")
            try:
                import serial
                self._imu_conn = serial.Serial(self._imu_port, 115200, timeout=0.1)
                self.get_logger().info(f"Opened IMU serial port: {self._imu_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to open IMU serial port {self._imu_port}: {e}")

        # ── Pub / Sub ────────────────────────────────────────────────────────
        self._init_topics()

        # ── Dynamic Config Checker ──
        self._config_mtime = 0.0
        self._check_topic_config()
        self._config_timer = self.create_timer(1.0, self._check_topic_config)

        # ── State ────────────────────────────────────────────────────────────
        self._start_stamp = self.get_clock().now()
        initial_state = random_navigation_defaults()
        self._speed_kmh = initial_state["speed_kmh"]
        self._target_speed_kmh = 0.0
        self._saved_speed_kmh = 0.0
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

    def _init_topics(self) -> None:
        nav_topic = get_topic_path("telemetry_nav", "/rover/telemetry/nav")
        motor_topic = get_topic_path("motor_control", "/rover/commands/motor")
        safety_topic = get_topic_path("telemetry_safety", "/rover/telemetry/safety")

        self._publisher = self.create_publisher(
            String,
            nav_topic,
            SENSOR_QOS,
        )
        self._command_subscription = self.create_subscription(
            String,
            motor_topic,
            self._motor_command_callback,
            COMMAND_QOS,
        )
        self._safety_subscription = self._create_safety_subscription(safety_topic)

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
            if hasattr(self, "_command_subscription") and self._command_subscription:
                self.destroy_subscription(self._command_subscription)
                self._command_subscription = None
            if hasattr(self, "_safety_subscription") and self._safety_subscription:
                self.destroy_subscription(self._safety_subscription)
                self._safety_subscription = None
            
            self._init_topics()
            self.get_logger().info("NavigationNode topics updated successfully.")
        except Exception as e:
            self.get_logger().error(f"Error reconfiguring NavigationNode topics: {e}")

    # ── Safety subscription with deadline callback ───────────────────────────

    def _create_safety_subscription(self, safety_topic: str):
        """Create safety subscription with DEADLINE event handler."""
        if _QOS_EVENTS_AVAILABLE:
            event_callbacks = SubscriptionEventCallbacks()
            event_callbacks.deadline = self._on_safety_deadline_missed
            return self.create_subscription(
                String,
                safety_topic,
                self._safety_callback,
                SAFETY_HEARTBEAT_QOS,
                event_callbacks=event_callbacks,
            )
        # Fallback: no QoS event support — use matching QoS but no callback.
        return self.create_subscription(
            String,
            safety_topic,
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
            self._saved_speed_kmh = self._target_speed_kmh
            self._target_speed_kmh = 0.0
            self._wp_status = "blocked"
            return
        if action == "stop":
            self._saved_speed_kmh = self._target_speed_kmh
            self._target_speed_kmh = 0.0
            self._wp_status = "idle"
            return
        if action == "resume":
            self._target_speed_kmh = self._saved_speed_kmh
            self._wp_status = "navigating" if self._target_speed_kmh > 0 else "idle"
            return

        if action not in ("", "drive"):
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

    def _read_sensors(self) -> None:
        if self.use_simulation:
            return

        # Read GPS (NMEA format)
        if self._gps_conn and self._gps_conn.is_open:
            try:
                import pynmea2
                while self._gps_conn.in_waiting:
                    line = self._gps_conn.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('$GPRMC') or line.startswith('$GPGGA'):
                        try:
                            msg = pynmea2.parse(line)
                            if hasattr(msg, 'latitude') and msg.latitude:
                                self._pos_lat = float(msg.latitude)
                            if hasattr(msg, 'longitude') and msg.longitude:
                                self._pos_lon = float(msg.longitude)
                            if hasattr(msg, 'spd_over_grnd') and msg.spd_over_grnd is not None:
                                self._speed_kmh = float(msg.spd_over_grnd) * 1.852  # knots to km/h
                        except Exception:
                            pass
            except Exception as e:
                self.get_logger().error(f"Error reading GPS serial: {e}")

        # Read IMU (assuming ASCII orientation data: "heading_deg,pitch,roll")
        if self._imu_conn and self._imu_conn.is_open:
            try:
                while self._imu_conn.in_waiting:
                    line = self._imu_conn.readline().decode('utf-8', errors='ignore').strip()
                    parts = line.split(',')
                    if parts and len(parts) >= 1:
                        try:
                            self._heading_deg = float(parts[0]) % 360.0
                        except ValueError:
                            pass
            except Exception as e:
                self.get_logger().error(f"Error reading IMU serial: {e}")

    # ── Publish ──────────────────────────────────────────────────────────────

    def _publish_nav_data(self) -> dict:
        if self._safety_blocked:
            self._target_speed_kmh = 0.0

        if self.use_simulation:
            self._speed_kmh += (self._target_speed_kmh - self._speed_kmh) * 0.25
            self._heading_deg += (
                (self._target_heading_deg - self._heading_deg + 540.0) % 360.0 - 180.0
            ) * 0.15
            self._heading_deg %= 360.0

            if self._target_speed_kmh <= 0.0 and not self._safety_blocked:
                self._speed_kmh = max(0.0, self._speed_kmh - 0.1)

            # dt is nominally 1/publish_rate_hz; use a fixed 100ms to avoid drift.
            dt = 0.1
            distance_increment = (self._speed_kmh / 3.6) * dt
            heading_rad = math.radians(self._heading_deg)
            meters_per_deg_lat = 111320.0
            meters_per_deg_lon = 111320.0 * math.cos(math.radians(self._pos_lat))
            self._pos_lat += (distance_increment / meters_per_deg_lat) * math.cos(heading_rad)
            self._pos_lon += (distance_increment / meters_per_deg_lon) * math.sin(heading_rad)
            self._dist_traveled_m += distance_increment
        else:
            self._read_sensors()
            if self._safety_blocked:
                self._speed_kmh = 0.0

        # Use node clock for elapsed-time-based calculations so sim time works.
        now_stamp = self.get_clock().now()
        elapsed_since_start = (
            (now_stamp - self._start_stamp).nanoseconds / 1_000_000_000
        )
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

    def destroy_node(self) -> None:
        if self._gps_conn and self._gps_conn.is_open:
            try:
                self._gps_conn.close()
            except Exception:
                pass
        if self._imu_conn and self._imu_conn.is_open:
            try:
                self._imu_conn.close()
            except Exception:
                pass
        super().destroy_node()

    def timer_callback(self):
        return self._timer_callback()


def main(args=None):
    node = NavigationNode()
    run_node(node, fallback_period=0.1)


if __name__ == "__main__":
    main()
