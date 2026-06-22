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

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    _rclpy = None

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


class MockRoverTelemetryPublisher(Node):
    """Mock ROS2 node that publishes simulated rover telemetry at 100 Hz."""

    def __init__(self):
        self._owns_ros_context = False
        if ROS_AVAILABLE and not _ros_initialized():
            _rclpy.init()
            self._owns_ros_context = True
        super().__init__("mock_rover_telemetry")

        self.publisher_ = self.create_publisher(String, "/rover/telemetry", 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

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
        self.get_logger().info("Mock Rover Telemetry Publisher initialized (100 Hz)")

    def timer_callback(self):
        self.tick_count += 1
        telemetry = self.generate_telemetry()
        message = String()
        message.data = json.dumps(telemetry, separators=(",", ":"))
        self.publisher_.publish(message)
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
                "rosout_last": "Mock node running smoothly.",
            },
        }


def main(args=None):
    node = MockRoverTelemetryPublisher()
    run_node(node, fallback_period=0.01)


if __name__ == "__main__":
    main()
