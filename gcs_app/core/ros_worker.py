"""ROS-to-Qt bridge for rover telemetry and operator commands."""

from __future__ import annotations

import json
import math
import random
import threading
import time
from queue import Empty, Queue
from typing import Any

from gcs_app.core.data_models import TelemetryPayload
from gcs_app.qt_compat import QThread

try:  # pragma: no cover - real rover runtime only.
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - used in the workspace.
    ROS_AVAILABLE = False

    class String:
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

    class _FallbackPublisher:
        def __init__(self, topic_name: str):
            self.topic_name = topic_name
            self.last_message: String | None = None

        def publish(self, message: String):
            self.last_message = message

    class Node:
        def __init__(self, name: str):
            self._name = name
            self._logger = _Logger(name)
            self._timers: list[dict[str, Any]] = []
            self._subscriptions: list[dict[str, Any]] = []

        def get_logger(self):
            return self._logger

        def create_publisher(self, message_type, topic_name: str, qos_profile):
            return _FallbackPublisher(topic_name)

        def create_subscription(self, message_type, topic_name: str, callback, qos_profile):
            subscription = {"topic_name": topic_name, "callback": callback}
            self._subscriptions.append(subscription)
            return subscription

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
            self._subscriptions.clear()

        def _spin_once(self):
            now = time.monotonic()
            for timer in self._timers:
                if now >= timer["next_fire"]:
                    timer["next_fire"] = now + timer["period_sec"]
                    timer["callback"]()

    class _RclpyStub:
        @staticmethod
        def init(args=None):
            return None

        @staticmethod
        def shutdown():
            return None

        @staticmethod
        def spin(node):
            try:
                while True:
                    node._spin_once()
                    time.sleep(0.05)
            except KeyboardInterrupt:
                return None

        @staticmethod
        def spin_once(node, timeout_sec: float | None = None):
            node._spin_once()
            if timeout_sec:
                time.sleep(min(timeout_sec, 0.05))

    rclpy = _RclpyStub()


def _extract_message_data(message: Any) -> str:
    if hasattr(message, "data"):
        return message.data
    if isinstance(message, str):
        return message
    raise ValueError("Unsupported telemetry message type")


class TelemetryReceiverNode(Node):
    """ROS2 receiver node responsible for telemetry ingress from `/rover/telemetry`."""

    def __init__(self, data_lock: threading.Lock, shared_buffer: dict[str, Any]):
        super().__init__("gcs_telemetry_receiver")
        self._data_lock = data_lock
        self._shared_buffer = shared_buffer
        self._telemetry_subscription = self.create_subscription(
            String,
            "/rover/telemetry",
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg: String):
        try:
            payload = json.loads(_extract_message_data(msg))
            telemetry = TelemetryPayload.from_dict(payload)
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self.get_logger().warning(f"Dropped invalid telemetry frame: {exc}")
            return

        with self._data_lock:
            self._shared_buffer["telemetry"] = telemetry
            self._shared_buffer["last_error"] = None
            self._shared_buffer["connected"] = True
            self._shared_buffer["last_update"] = time.time()


class ROSWorker(QThread):
    """Background telemetry bridge used by the Qt UI."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._data_lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()
        self._shared_buffer: dict[str, Any] = {
            "telemetry": None,
            "connected": False,
            "last_error": None,
            "last_update": None,
            "last_command": None,
        }
        self._command_queue: Queue[dict[str, Any]] = Queue()
        self._ros_node: TelemetryReceiverNode | None = None
        self._command_publisher = None
        self._sim_start = time.time()
        self._sim_tick = 0
        self._sim_speed_kmh = 0.0
        self._sim_heading_deg = 0.0
        self._sim_lat = 40.0
        self._sim_lon = -75.0
        self._sim_distance_m = 0.0
        self._sim_battery_pct = 100.0
        self._sim_heartbeat = 0

    def run(self):
        try:
            if ROS_AVAILABLE:
                import rclpy
                if not rclpy.ok():
                    rclpy.init(args=None)
                self._ros_node = TelemetryReceiverNode(self._data_lock, self._shared_buffer)
                self._command_publisher = self._ros_node.create_publisher(String, "/rover/commands/motor", 10)
                with self._data_lock:
                    self._shared_buffer["connected"] = False
                while self._running.is_set():
                    rclpy.spin_once(self._ros_node, timeout_sec=0.05)
                    self._flush_command_queue()
            else:
                while self._running.is_set():
                    self._flush_command_queue()
                    telemetry = self._build_fallback_telemetry()
                    with self._data_lock:
                        self._shared_buffer["telemetry"] = telemetry
                        self._shared_buffer["connected"] = True
                        self._shared_buffer["last_error"] = None
                        self._shared_buffer["last_update"] = time.time()
                    time.sleep(0.1)
        except Exception as exc:
            with self._data_lock:
                self._shared_buffer["last_error"] = str(exc)
                self._shared_buffer["connected"] = False
        finally:
            if self._ros_node is not None:
                self._ros_node.destroy_node()
                self._ros_node = None
            if ROS_AVAILABLE:
                rclpy.shutdown()

    def _flush_command_queue(self):
        latest_command: dict[str, Any] | None = None
        while True:
            try:
                latest_command = self._command_queue.get_nowait()
            except Empty:
                break

        if latest_command is None:
            return

        with self._data_lock:
            self._shared_buffer["last_command"] = latest_command

        if self._command_publisher is not None:
            message = String()
            message.data = json.dumps(latest_command, separators=(",", ":"))
            self._command_publisher.publish(message)

    def _build_fallback_telemetry(self) -> TelemetryPayload:
        self._sim_tick += 1
        self._sim_speed_kmh = min(12.0, max(0.0, self._sim_speed_kmh + random.uniform(-0.4, 0.6)))
        self._sim_heading_deg = (self._sim_heading_deg + 1.2) % 360.0
        distance_increment = (self._sim_speed_kmh / 3.6) * 0.1
        heading_rad = math.radians(self._sim_heading_deg)
        self._sim_lat += (distance_increment / 111000.0) * math.cos(heading_rad)
        self._sim_lon += (distance_increment / 111000.0) * math.sin(heading_rad)
        self._sim_distance_m += distance_increment
        self._sim_battery_pct = max(0.0, self._sim_battery_pct - 0.001)
        self._sim_heartbeat += 1

        with self._data_lock:
            command = self._shared_buffer.get("last_command")

        estop_triggered = bool(command and command.get("action") == "estop")
        if command and command.get("action") in {"stop", "estop"}:
            self._sim_speed_kmh = 0.0

        telemetry = {
            "Navigation": {
                "speed_kmh": round(self._sim_speed_kmh, 2),
                "heading_deg": round(self._sim_heading_deg, 1),
                "pos_lat": round(self._sim_lat, 6),
                "pos_lon": round(self._sim_lon, 6),
                "dist_traveled_m": round(self._sim_distance_m, 1),
                "wp_current": int(self._sim_tick / 50) % 10,
                "wp_error_m": round(abs(math.sin(self._sim_tick / 20.0)) * 1.5, 2),
                "wp_status": "idle" if self._sim_speed_kmh == 0.0 else "navigating",
            },
            "Safety": {
                "mode": "estop" if estop_triggered else "monitoring",
                "light_state": "red" if estop_triggered else "green",
                "estop_mech_armed": False,
                "estop_wire_armed": False,
                "estop_triggered": estop_triggered,
                "is_blocked": estop_triggered,
                "collision_detected": False,
                "border_crossed": False,
                "border_partial": False,
                "obstacle_touched": False,
            },
            "Vision": {
                "img_confidence": round(max(0.0, min(1.0, random.gauss(0.7, 0.15))), 2),
                "img_detected": random.random() > 0.35,
                "laser_active": random.random() > 0.75,
                "img_elapsed_sec": int(time.time() - self._sim_start),
                "img_task_status": "tracking",
                "lane_detected": random.random() > 0.4,
                "obstacles_count": random.randint(0, 4),
                "fps_vision": round(max(0.0, random.gauss(29.5, 1.4)), 1),
            },
            "Jetson": {
                "cpu_pct": round(max(0.0, min(100.0, 38.0 + random.gauss(0, 6))), 1),
                "gpu_pct": round(max(0.0, min(100.0, 20.0 + random.gauss(0, 4))), 1),
                "ram_pct": round(max(0.0, min(100.0, 61.0 + random.gauss(0, 3))), 1),
                "temp_c": round(max(0.0, 42.0 + random.gauss(0, 1.5)), 1),
                "bat_pct": round(self._sim_battery_pct, 1),
                "bat_voltage": round(10.8 + (self._sim_battery_pct / 100.0) * 1.2, 2),
                "uptime_sec": int(time.time() - self._sim_start),
            },
            "Communication": {
                "rtt_ms": random.randint(18, 45),
                "channel_rssi": random.randint(-76, -48),
                "stream_fps": round(max(0.0, random.gauss(29.0, 1.5)), 1),
                "packet_loss_pct": round(max(0.0, random.gauss(0.4, 0.3)), 1),
                "heartbeat_seq": self._sim_heartbeat,
                "timestamp_ms": int((time.time() - self._sim_start) * 1000),
            },
            "ROS": {
                "node_lane_det": True,
                "node_obs_avoid": True,
                "node_wp_nav": True,
                "node_img_recog": True,
                "node_motor_ctrl": True,
                "rosout_last": "Fallback simulator active.",
            },
        }
        return TelemetryPayload.from_dict(telemetry)

    def send_motor_command(self, command: dict[str, Any]):
        if not isinstance(command, dict):
            raise TypeError("command must be a dictionary")
        normalized = dict(command)
        normalized.setdefault("source", "gcs")
        normalized.setdefault("timestamp_ms", int(time.time() * 1000))
        self._command_queue.put(normalized)

    def get_latest_telemetry(self) -> TelemetryPayload | None:
        with self._data_lock:
            telemetry = self._shared_buffer.get("telemetry")
        return telemetry if isinstance(telemetry, TelemetryPayload) else None

    def get_last_error(self) -> str | None:
        with self._data_lock:
            return self._shared_buffer.get("last_error")

    def is_connected(self) -> bool:
        with self._data_lock:
            last_update = self._shared_buffer.get("last_update")
            if last_update is None:
                return False
            if time.time() - last_update > 2.0:
                return False
            return bool(self._shared_buffer.get("connected"))

    def stop(self):
        self._running.clear()
        if hasattr(self, "isRunning"):
            try:
                running = self.isRunning()
            except Exception:
                running = False
        else:
            running = False
        if running:
            self.wait(1000)


RosWorker = ROSWorker


__all__ = ["ROSWorker", "RosWorker", "TelemetryReceiverNode"]
