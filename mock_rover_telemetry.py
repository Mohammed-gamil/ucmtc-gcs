#!/usr/bin/env python3
"""Mock rover telemetry publisher."""

from __future__ import annotations

import json
import math
import random
import time
from typing import Any

try:
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from std_msgs.msg import String as _String
    from sensor_msgs.msg import LaserScan as _LaserScan
    from std_msgs.msg import Float32 as _Float32

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    _rclpy = None
    _LaserScan = None
    _Float32 = None

    class _String:
        def __init__(self, data: str = ""):
            self.data = data

    class _Logger:
        def __init__(self, name: str):
            self.name = name

        def info(self, message: str):
            print(f"[INFO] [{self.name}] {message}")

        def warning(self, message: str):
            print(f"[WARN] [{self.name}] {message}")

        def error(self, message: str):
            print(f"[ERROR] [{self.name}] {message}")

        def debug(self, message: str):
            print(f"[DEBUG] [{self.name}] {message}")

    class _Publisher:
        def __init__(self, topic_name: str):
            self.topic_name = topic_name
            self.last_message = None

        def publish(self, message: _String):
            self.last_message = message

    class _Node:
        def __init__(self, name: str):
            self._name = name
            self._logger = _Logger(name)
            self._timers: list[dict[str, Any]] = []

        def get_logger(self):
            return self._logger

        def create_publisher(self, message_type, topic_name: str, qos_profile):
            return _Publisher(topic_name)

        def create_timer(self, period_sec: float, callback):
            timer = {
                "period_sec": period_sec,
                "callback": callback,
                "next_fire": time.monotonic() + period_sec,
            }
            self._timers.append(timer)
            return timer

        def destroy_node(self):
            self._timers.clear()

        def _spin_once(self):
            now = time.monotonic()
            for timer in self._timers:
                if now >= timer["next_fire"]:
                    timer["next_fire"] = now + timer["period_sec"]
                    timer["callback"]()

    _NodeType = _Node
else:
    _NodeType = _Node

Node = _NodeType
String = _String
Float32 = _Float32


def _ros_initialized() -> bool:
    return bool(ROS_AVAILABLE and hasattr(_rclpy, "ok") and _rclpy.ok())


def _ros_shutdown():
    if _ros_initialized():
        _rclpy.shutdown()


def run_node(node, fallback_period: float = 0.01):
    if ROS_AVAILABLE:
        initialized_here = False
        if not _ros_initialized():
            _rclpy.init()
            initialized_here = True
        try:
            _rclpy.spin(node)
        finally:
            if initialized_here or getattr(node, "_owns_ros_context", False):
                _ros_shutdown()
        return

    try:
        while True:
            node._spin_once()
            time.sleep(fallback_period)
    except KeyboardInterrupt:
        return None


def get_topic_path(topic_key: str, default_path: str) -> str:
    """Resolve the topic path from topic_config.json, falling back to default_path."""
    import os
    import json

    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(base_dir, "web_gcs/topic_config.json")
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = json.load(f)
                if topic_key in config and "path" in config[topic_key]:
                    path = config[topic_key]["path"]
                    if path:
                        return path
    except Exception:
        pass
    return default_path


class MockRoverTelemetryPublisher(Node):
    """Mock ROS2 node that publishes simulated rover telemetry at 100 Hz."""

    def __init__(self):
        self._owns_ros_context = False
        if ROS_AVAILABLE and not _ros_initialized():
            _rclpy.init()
            self._owns_ros_context = True
        super().__init__("mock_rover_telemetry")

        self._init_topics()
        
        # ── Dynamic Config Checker ──
        self._config_mtime = 0.0
        self._check_topic_config()
        self._config_timer = self.create_timer(1.0, self._check_topic_config)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def _init_topics(self) -> None:
        telem_topic = get_topic_path("telemetry_aggregator", "/rover/telemetry")
        scan_topic = get_topic_path("obstacle_avoidance", "/scan")
        mission_topic = get_topic_path("mission_phase", "/mission_phase")
        arm_topic = get_topic_path("arm_status", "/arm_status")
        speed_limit_topic = get_topic_path("speed_limit", "/speed_limit")
        
        self.publisher_ = self.create_publisher(String, telem_topic, 10)
        self.mission_pub_ = self.create_publisher(String, mission_topic, 10)
        self.arm_pub_ = self.create_publisher(String, arm_topic, 10)
        
        self.speed_limit_pub_ = None
        if ROS_AVAILABLE and Float32 is not None:
            self.speed_limit_pub_ = self.create_publisher(Float32, speed_limit_topic, 10)
        
        self.scan_pub = None
        if ROS_AVAILABLE and _LaserScan is not None:
            try:
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
                scan_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=5
                )
            except Exception:
                scan_qos = 10
            self.scan_pub = self.create_publisher(_LaserScan, scan_topic, scan_qos)
            self.get_logger().info(f"Mock {scan_topic} publisher initialized (sensor_msgs/LaserScan, BEST_EFFORT)")

    def _check_topic_config(self) -> None:
        import os
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(base_dir, "web_gcs/topic_config.json")
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
            if hasattr(self, "publisher_") and self.publisher_:
                self.destroy_publisher(self.publisher_)
                self.publisher_ = None
            if hasattr(self, "mission_pub_") and self.mission_pub_:
                self.destroy_publisher(self.mission_pub_)
                self.mission_pub_ = None
            if hasattr(self, "arm_pub_") and self.arm_pub_:
                self.destroy_publisher(self.arm_pub_)
                self.arm_pub_ = None
            if hasattr(self, "speed_limit_pub_") and self.speed_limit_pub_:
                self.destroy_publisher(self.speed_limit_pub_)
                self.speed_limit_pub_ = None
            if hasattr(self, "scan_pub") and self.scan_pub:
                self.destroy_publisher(self.scan_pub)
                self.scan_pub = None
            
            self._init_topics()
            self.get_logger().info("MockRoverTelemetryPublisher topics updated successfully.")
        except Exception as e:
            self.get_logger().error(f"Error reconfiguring MockRoverTelemetryPublisher topics: {e}")

        self.tick_count = 0
        self.start_time = time.time()
        self.speed_kmh = 0.0
        self.heading_deg = 0.0
        self.pos_lat = 40.0 + random.uniform(-0.001, 0.001)
        self.pos_lon = -75.0 + random.uniform(-0.001, 0.001)
        self.dist_traveled_m = 0.0
        self.wp_current = 0
        self.wp_status = "idle"
        self.wp_error_m = 0.0
        self.collision_active = False
        self.collision_counter = 0
        self.battery_pct = 100.0
        self.cpu_base = 65.0
        self.packet_loss = 0.0
        self.heartbeat_seq = 0
        self.vision_fps = 30.0
        self.lane_detected_state = False
        self.obstacles_count = 0
        
        self.mission_phase_state = "IDLE"
        self.arm_state = "STOWED"
        self.speed_limit_val = 15.0
        
        self.get_logger().info("Mock Rover Telemetry Publisher initialized (100 Hz)")

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

    def timer_callback(self):
        self.tick_count += 1
        telemetry = self.generate_telemetry()
        message = String()
        message.data = json.dumps(telemetry, separators=(",", ":"))
        self.publisher_.publish(message)
        
        if self.tick_count % 100 == 0:
            mission_msg = String()
            mission_msg.data = self.mission_phase_state
            self.mission_pub_.publish(mission_msg)
            
            arm_msg = String()
            arm_msg.data = self.arm_state
            self.arm_pub_.publish(arm_msg)
            
            if self.speed_limit_pub_ is not None and Float32 is not None:
                limit_msg = Float32()
                limit_msg.data = float(self.speed_limit_val)
                self.speed_limit_pub_.publish(limit_msg)

        if self.scan_pub is not None and _LaserScan is not None:
            if self.tick_count % 10 == 0:
                scan_msg = _LaserScan()
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
                
                t = time.time()
                scan_msg.ranges = self._get_simulated_scan(t)
                self.scan_pub.publish(scan_msg)

        if self.tick_count % 100 == 0:
            self.get_logger().debug(
                f'Published telemetry #{self.tick_count}: '
                f'Speed={telemetry["Navigation"]["speed_kmh"]:.2f} km/h, '
                f'Heading={telemetry["Navigation"]["heading_deg"]:.1f} deg, '
                f'Battery={telemetry["Jetson"]["bat_pct"]:.1f}%'
            )

    def generate_telemetry(self) -> dict[str, Any]:
        self.speed_kmh = max(0.0, min(15.0, 5.0 + 8.0 * (0.5 + 0.5 * math.sin(self.tick_count / 50.0)) + random.gauss(0, 0.5)))
        self.heading_deg = (self.heading_deg + random.uniform(0.5, 1.5)) % 360.0

        distance_increment = (self.speed_kmh / 3.6) * 0.01
        heading_rad = math.radians(self.heading_deg)
        self.pos_lat += (distance_increment / 111000.0) * math.cos(heading_rad)
        self.pos_lon += (distance_increment / 111000.0) * math.sin(heading_rad)
        self.dist_traveled_m += distance_increment

        if self.tick_count % 300 == 0:
            self.wp_current = (self.wp_current + 1) % 10
            self.wp_status = random.choice(["idle", "navigating", "reached"])
        self.wp_error_m = max(0.0, random.gauss(0.5, 0.2))

        if self.tick_count % 1000 == 0:
            self.mission_phase_state = random.choice(["MAPPING", "AUTONOMY", "TELEOP", "IDLE", "RETURN_TO_BASE"])
            self.arm_state = random.choice(["STOWED", "DEPLOYING", "ACTIVE", "RETRACTING", "ERROR"])
            self.speed_limit_val = random.choice([2.5, 5.0, 10.0, 15.0])

        if self.tick_count % 500 == 0:
            self.collision_active = True
            self.collision_counter = 0
        if self.collision_active:
            self.collision_counter += 1
            if self.collision_counter > 20:
                self.collision_active = False

        self.vision_fps = 28.0 + random.gauss(0, 1.0)
        img_confidence = random.gauss(0.7, 0.1) if random.random() > 0.3 else 0.0
        img_detected = img_confidence > 0.5
        if self.tick_count % 200 == 0:
            self.lane_detected_state = random.random() > 0.5
        self.obstacles_count = random.randint(0, 5) if img_detected else 0

        cpu_jitter = random.gauss(0, 3)
        if random.random() > 0.95:
            cpu_jitter += random.uniform(10, 20)
        self.cpu_base = max(55, min(85, self.cpu_base + cpu_jitter * 0.1))
        cpu_pct = max(0.0, min(100.0, self.cpu_base + random.gauss(0, 2)))
        gpu_pct = max(0.0, min(100.0, cpu_pct * 0.6 + random.gauss(0, 3)))
        ram_pct = max(0.0, min(100.0, 65.0 + (self.tick_count / 10000.0) * 5 + random.gauss(0, 2)))
        temp_c = 35.0 + (cpu_pct / 100.0) * 40.0 + random.gauss(0, 0.5)
        self.battery_pct = max(0.0, min(100.0, self.battery_pct - 0.001))
        bat_voltage = 10.5 + (self.battery_pct / 100.0) * 1.5

        if self.tick_count % 200 == 0:
            self.packet_loss = random.uniform(2, 8)
        else:
            self.packet_loss = max(0.0, self.packet_loss - 0.5)
        rtt_ms = random.randint(100, 200) if self.tick_count % 200 < 10 else random.randint(15, 30)

        self.heartbeat_seq = (self.heartbeat_seq + 1) % 65536
        timestamp_ms = int((time.time() - self.start_time) * 1000)

        # Generate simulated ranges
        t = time.time()
        raw_ranges = self._get_simulated_scan(t)
        rounded_ranges = [round(r, 2) for r in raw_ranges]
        
        # Calculate forward range (closest in front -30 deg to +30 deg, which is index 75 to 105)
        fwd_ranges = [r for r in raw_ranges[75:105] if r is not None]
        fwd_range = min(fwd_ranges) if fwd_ranges else 12.0

        return {
            "Navigation": {
                "speed_kmh": round(self.speed_kmh, 2),
                "heading_deg": round(self.heading_deg, 1),
                "pos_lat": round(self.pos_lat, 6),
                "pos_lon": round(self.pos_lon, 6),
                "dist_traveled_m": round(self.dist_traveled_m, 1),
                "wp_current": self.wp_current,
                "wp_error_m": round(self.wp_error_m, 2),
                "wp_status": self.wp_status,
            },
            "Safety": {
                "mode": "monitoring" if self.collision_active else "idle",
                "light_state": "red" if self.collision_active else "green",
                "estop_mech_armed": False,
                "estop_wire_armed": False,
                "estop_triggered": False,
                "is_blocked": False,
                "collision_detected": self.collision_active,
                "border_crossed": False,
                "border_partial": False,
                "obstacle_touched": False,
            },
            "Vision": {
                "img_confidence": round(max(0.0, min(1.0, img_confidence)), 2),
                "img_detected": img_detected,
                "laser_active": random.random() > 0.8,
                "img_elapsed_sec": int(self.tick_count / 100),
                "img_task_status": "processing" if img_detected else "idle",
                "lane_detected": self.lane_detected_state,
                "obstacles_count": self.obstacles_count,
                "fps_vision": round(self.vision_fps, 1),
            },
            "Jetson": {
                "cpu_pct": round(cpu_pct, 1),
                "gpu_pct": round(gpu_pct, 1),
                "ram_pct": round(ram_pct, 1),
                "temp_c": round(temp_c, 1),
                "bat_pct": round(self.battery_pct, 1),
                "bat_voltage": round(bat_voltage, 2),
                "uptime_sec": int(time.time() - self.start_time),
            },
            "Communication": {
                "rtt_ms": rtt_ms,
                "channel_rssi": random.randint(-80, -50),
                "stream_fps": round(max(0.0, 28.0 + random.gauss(0, 2)), 1),
                "packet_loss_pct": round(self.packet_loss, 1),
                "heartbeat_seq": self.heartbeat_seq,
                "timestamp_ms": timestamp_ms,
            },
            "ROS": {
                "node_lane_det": random.random() > 0.001,
                "node_obs_avoid": random.random() > 0.001,
                "node_wp_nav": random.random() > 0.001,
                "node_img_recog": random.random() > 0.001,
                "node_motor_ctrl": random.random() > 0.001,
                "esp32_connected": True,
                "rosout_last": "Mock node running smoothly.",
            },
            "Sensors": {
                "imu": {
                    "available": False
                },
                "scan": {
                    "available": True,
                    "frame_id": "laser_frame",
                    "angle_min_rad": -3.1416,
                    "angle_max_rad": 3.1416,
                    "range_min_m": 0.12,
                    "range_max_m": 12.0,
                    "num_points": 180,
                    "num_valid": 180,
                    "forward_range_m": round(fwd_range, 3),
                    "min_range_m": min(rounded_ranges),
                    "max_range_m": max(rounded_ranges),
                    "ranges": rounded_ranges
                }
            }
        }


def main(args=None):
    node = MockRoverTelemetryPublisher()
    run_node(node, fallback_period=0.01)


if __name__ == "__main__":
    main()
