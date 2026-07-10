"""Telemetry aggregator that publishes the canonical rover payload."""

from __future__ import annotations

import json
import random
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import (
    RELIABLE_QOS,
    SAFETY_HEARTBEAT_QOS,
    SENSOR_QOS,
    _STD_MSGS_AVAILABLE,
    BatteryState,
    Imu,
    LaserScan,
    Log,
    NavSatFix,
    Odometry,
    TFMessage,
    Twist,
    clamp,
    decode_json_message,
    make_battery_payload,
    make_communication_payload,
    make_gps_payload,
    make_imu_payload,
    make_jetson_payload,
    make_navigation_payload,
    make_odom_payload,
    make_ros_payload,
    make_rosout_payload,
    make_safety_payload,
    make_scan_payload,
    make_tf_summary_payload,
    make_vision_payload,
    get_topic_path,
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
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self._init_topics()

        # ── Dynamic Config Checker ──
        self._config_mtime = 0.0
        self._check_topic_config()
        self._config_timer = self.create_timer(1.0, self._check_topic_config)
        if _STD_MSGS_AVAILABLE:
            self.get_logger().info(
                "Standard ROS 2 topic subscriptions active: "
                "/scan /imu /fix /odom /battery_state /cmd_vel /tf /tf_static /rosout"
            )
        else:
            self.get_logger().warning(
                "Standard ROS 2 message types (sensor_msgs, geometry_msgs, nav_msgs, tf2_msgs) "
                "not available — standard topic subscriptions disabled."
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

        # ── Standard-sensor state (initialised to None → «not received yet») ──
        self._imu_msg: Any = None
        self._scan_msg: Any = None
        self._gps_msg: Any = None
        self._odom_msg: Any = None
        self._battery_msg: Any = None
        self._cmd_vel_linear_x: float = 0.0   # last received /cmd_vel
        self._cmd_vel_linear_y: float = 0.0
        self._cmd_vel_angular_z: float = 0.0
        self._tf_msg: Any = None
        self._tf_static_msg: Any = None
        self._rosout_msg: Any = None

        self._nav_last_update = self._start_stamp
        self._safety_last_update = self._start_stamp
        self._vision_last_update = self._start_stamp
        self._control_last_update = self._start_stamp

        self._timer = self.create_timer(period_sec, self.timer_callback)
        self.get_logger().info(
            f"Telemetry aggregator ready (rate={publish_rate_hz:.1f} Hz)"
        )

    def _init_topics(self) -> None:
        telem_topic = get_topic_path("telemetry_aggregator", "/rover/telemetry")
        nav_topic = get_topic_path("telemetry_nav", "/rover/telemetry/nav")
        safety_topic = get_topic_path("telemetry_safety", "/rover/telemetry/safety")
        vision_topic = get_topic_path("telemetry_vision", "/rover/telemetry/vision")
        control_topic = get_topic_path("telemetry_control", "/rover/telemetry/control")

        self._publisher = self.create_publisher(String, telem_topic, RELIABLE_QOS)
        
        self._nav_subscription = self.create_subscription(
            String, nav_topic, self.navigation_callback, SENSOR_QOS
        )
        self._safety_subscription = self._create_safety_subscription(safety_topic)
        self._vision_subscription = self.create_subscription(
            String, vision_topic, self.vision_callback, SENSOR_QOS
        )
        self._control_subscription = self.create_subscription(
            String, control_topic, self.control_callback, RELIABLE_QOS
        )

        if _STD_MSGS_AVAILABLE:
            scan_topic = get_topic_path("obstacle_avoidance", "/scan")
            imu_topic = get_topic_path("imu_accel", "/imu")
            gps_topic = get_topic_path("gps_fix", "/fix")
            odom_topic = get_topic_path("odom_coord", "/odom")
            battery_topic = get_topic_path("battery_state", "/battery_state")
            
            cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
            if cmd_vel_topic == "/cmd_vel":
                cmd_vel_topic = get_topic_path("cmd_vel_echo", "/cmd_vel")

            self._scan_sub = self.create_subscription(
                LaserScan, scan_topic, self._scan_callback, SENSOR_QOS
            )
            self._imu_sub = self.create_subscription(
                Imu, imu_topic, self._imu_callback, SENSOR_QOS
            )
            self._gps_sub = self.create_subscription(
                NavSatFix, gps_topic, self._gps_callback, SENSOR_QOS
            )
            self._odom_sub = self.create_subscription(
                Odometry, odom_topic, self._odom_callback, RELIABLE_QOS
            )
            self._battery_sub = self.create_subscription(
                BatteryState, battery_topic, self._battery_callback, SENSOR_QOS
            )
            self._cmd_vel_sub = self.create_subscription(
                Twist, cmd_vel_topic, self._cmd_vel_callback, RELIABLE_QOS
            )
            
            self._tf_sub = self.create_subscription(
                TFMessage, "/tf", self._tf_callback, SENSOR_QOS
            )
            try:
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
                tf_static_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )
            except Exception:
                tf_static_qos = RELIABLE_QOS
            self._tf_static_sub = self.create_subscription(
                TFMessage, "/tf_static", self._tf_static_callback, tf_static_qos
            )
            self._rosout_sub = self.create_subscription(
                Log, "/rosout", self._rosout_callback, RELIABLE_QOS
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
            if hasattr(self, "_nav_subscription") and self._nav_subscription:
                self.destroy_subscription(self._nav_subscription)
                self._nav_subscription = None
            if hasattr(self, "_safety_subscription") and self._safety_subscription:
                self.destroy_subscription(self._safety_subscription)
                self._safety_subscription = None
            if hasattr(self, "_vision_subscription") and self._vision_subscription:
                self.destroy_subscription(self._vision_subscription)
                self._vision_subscription = None
            if hasattr(self, "_control_subscription") and self._control_subscription:
                self.destroy_subscription(self._control_subscription)
                self._control_subscription = None
            
            if hasattr(self, "_scan_sub") and self._scan_sub:
                self.destroy_subscription(self._scan_sub)
                self._scan_sub = None
            if hasattr(self, "_imu_sub") and self._imu_sub:
                self.destroy_subscription(self._imu_sub)
                self._imu_sub = None
            if hasattr(self, "_gps_sub") and self._gps_sub:
                self.destroy_subscription(self._gps_sub)
                self._gps_sub = None
            if hasattr(self, "_odom_sub") and self._odom_sub:
                self.destroy_subscription(self._odom_sub)
                self._odom_sub = None
            if hasattr(self, "_battery_sub") and self._battery_sub:
                self.destroy_subscription(self._battery_sub)
                self._battery_sub = None
            if hasattr(self, "_cmd_vel_sub") and self._cmd_vel_sub:
                self.destroy_subscription(self._cmd_vel_sub)
                self._cmd_vel_sub = None
            if hasattr(self, "_tf_sub") and self._tf_sub:
                self.destroy_subscription(self._tf_sub)
                self._tf_sub = None
            if hasattr(self, "_tf_static_sub") and self._tf_static_sub:
                self.destroy_subscription(self._tf_static_sub)
                self._tf_static_sub = None
            if hasattr(self, "_rosout_sub") and self._rosout_sub:
                self.destroy_subscription(self._rosout_sub)
                self._rosout_sub = None
            
            self._init_topics()
            self.get_logger().info("TelemetryAggregator topics updated successfully.")
        except Exception as e:
            self.get_logger().error(f"Error reconfiguring TelemetryAggregator topics: {e}")

    # ── Safety subscription with deadline callback ───────────────────────────

    def _create_safety_subscription(self, safety_topic: str):
        """Create safety subscription with DEADLINE event handler."""
        if _QOS_EVENTS_AVAILABLE:
            event_callbacks = SubscriptionEventCallbacks()
            event_callbacks.deadline = self._on_safety_deadline_missed
            return self.create_subscription(
                String,
                safety_topic,
                self.safety_callback,
                SAFETY_HEARTBEAT_QOS,
                event_callbacks=event_callbacks,
            )
        # Fallback: no QoS event support
        return self.create_subscription(
            String,
            safety_topic,
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

    # ── Standard ROS 2 topic callbacks ──────────────────────────────────────

    def _imu_callback(self, msg) -> None:
        """Cache latest sensor_msgs/Imu message from /imu."""
        self._imu_msg = msg

    def _scan_callback(self, msg) -> None:
        """Cache latest sensor_msgs/LaserScan message from /scan."""
        self._scan_msg = msg

    def _gps_callback(self, msg) -> None:
        """Cache latest sensor_msgs/NavSatFix message from /fix."""
        self._gps_msg = msg

    def _odom_callback(self, msg) -> None:
        """Cache latest nav_msgs/Odometry message from /odom."""
        self._odom_msg = msg

    def _battery_callback(self, msg) -> None:
        """Cache latest sensor_msgs/BatteryState message from /battery_state."""
        self._battery_msg = msg

    def _cmd_vel_callback(self, msg) -> None:
        """Echo the last /cmd_vel Twist so the GCS can display what was commanded."""
        self._cmd_vel_linear_x = msg.linear.x
        self._cmd_vel_linear_y = msg.linear.y
        self._cmd_vel_angular_z = msg.angular.z

    def _tf_callback(self, msg) -> None:
        """Cache latest /tf TFMessage for frame-tree summary."""
        self._tf_msg = msg

    def _tf_static_callback(self, msg) -> None:
        """Cache latest /tf_static TFMessage (TRANSIENT_LOCAL — received once)."""
        self._tf_static_msg = msg

    def _rosout_callback(self, msg) -> None:
        """Cache latest /rosout log entry for the GCS log panel."""
        self._rosout_msg = msg

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
        payload: dict[str, Any] = {
            "Navigation": self._navigation if self._navigation else make_navigation_payload(0.0, 0.0, 40.0, -75.0, 0.0, 0, 0.0, "idle"),
            "Safety": self._safety if self._safety else make_safety_payload(
                "idle", "idle", False, False, False, False, False, False, False, False,
            ),
            "Vision": self._vision if self._vision else make_vision_payload(0.0, False, False, 0, "idle", False, 0, 0.0),
            "Jetson": self._sample_jetson(),
            "Communication": self._sample_communication(),
            "ROS": self._sample_ros(),
        }

        # ── Standard ROS 2 sensor sections ────────────────────────────────────
        # These are only present when the corresponding topics are publishing;
        # each payload builder returns {"available": False} when the last msg is None.
        if _STD_MSGS_AVAILABLE:
            payload["Sensors"] = {
                "imu": make_imu_payload(self._imu_msg),
                "scan": make_scan_payload(self._scan_msg),
            }
            payload["GPS"] = make_gps_payload(self._gps_msg)
            payload["Odom"] = make_odom_payload(self._odom_msg)
            payload["Battery"] = make_battery_payload(self._battery_msg)
            payload["CmdVelEcho"] = {
                "linear_x": round(self._cmd_vel_linear_x, 4),
                "linear_y": round(self._cmd_vel_linear_y, 4),
                "angular_z": round(self._cmd_vel_angular_z, 6),
            }
            payload["TF"] = {
                "dynamic": make_tf_summary_payload(self._tf_msg),
                "static": make_tf_summary_payload(self._tf_static_msg),
            }
            payload["Rosout"] = make_rosout_payload(self._rosout_msg)

        return payload

    def publish_telemetry(self):
        payload = self.build_payload()
        message = String()
        
        try:
            import numpy as np
            class NumpyEncoder(json.JSONEncoder):
                def default(self, obj):
                    if isinstance(obj, np.bool_):
                        return bool(obj)
                    if isinstance(obj, np.integer):
                        return int(obj)
                    if isinstance(obj, np.floating):
                        return float(obj)
                    if isinstance(obj, np.ndarray):
                        return obj.tolist()
                    return super().default(obj)
            json_str = json.dumps(payload, separators=(",", ":"), cls=NumpyEncoder)
        except ImportError:
            json_str = json.dumps(payload, separators=(",", ":"))

        message.data = json_str
        self._publisher.publish(message)
        return payload

    def timer_callback(self):
        return self.publish_telemetry()


def main(args=None):
    node = TelemetryAggregatorNode()
    run_node(node, fallback_period=0.1)


if __name__ == "__main__":
    main()
