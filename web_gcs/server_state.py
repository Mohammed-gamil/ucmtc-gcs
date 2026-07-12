import json
import os
import sys
import threading
import time
from typing import Any

# Ensure correct path loading
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
ROVER_SRC_DIR = os.path.join(WORKSPACE_DIR, "rover_ws", "src", "rover_core")
if ROVER_SRC_DIR not in sys.path:
    sys.path.insert(0, ROVER_SRC_DIR)

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rover_core.telemetry_utils import COMMAND_QOS, RELIABLE_QOS
    ROS_AVAILABLE = True
except ImportError:
    # Use fallback compatibility module
    try:
        from rover_core.ros_compat import Node, String, rclpy, ROS_AVAILABLE
        from rover_core.telemetry_utils import COMMAND_QOS, RELIABLE_QOS
    except ImportError:
        ROS_AVAILABLE = False
        COMMAND_QOS = 1   # type: ignore[assignment]
        RELIABLE_QOS = 10 # type: ignore[assignment]
        class String:
            def __init__(self, data: str = ""):
                self.data = data
        class Node:
            def __init__(self, name: str):
                self._name = name
            def create_publisher(self, message_type, topic, qos):
                class Pub:
                    def publish(self, msg): pass
                return Pub()
            def create_subscription(self, message_type, topic, callback, qos):
                return None
            def get_logger(self):
                class Logger:
                    def info(self, msg): print("[INFO]", msg)
                    def warning(self, msg): print("[WARN]", msg)
                    def error(self, msg): print("[ERROR]", msg)
                return Logger()

# Global Connector import
from gcs_app.core.global_connector import get_global_connector, shutdown_global_connector

STATIC_DIR = os.path.join(WORKSPACE_DIR, "web_gcs")
PORT = 8082

# Shared memory buffer for telemetry and command handling
data_lock = threading.Lock()
telemetry_state: dict[str, Any] = {
    "latest": None,
    "last_update": 0.0,
    "camera_frame": None,
    "camera_frame_raw": None,
    "last_camera_update": 0.0,
}
command_publisher = None
cmd_vel_publisher = None
sim_estop_triggered = False
_bridge_node = None

def _get_bridge_node():
    global _bridge_node
    return _bridge_node

DEFAULT_TOPICS = {
    "motor_control": {"label": "Motor Control", "path": "/rover/commands/motor"},
    "lane_detection": {"label": "Lane Detection", "path": "/rover/sensors/lane"},
    "obstacle_avoidance": {"label": "Obstacle Avoidance", "path": "/scan"},
    "waypoint_navigator": {"label": "Waypoint Navigator", "path": "/rover/commands/nav"},
    "image_recognition": {"label": "Image Recognition", "path": "/rgb/image_raw/compressed"},
    "telemetry_aggregator": {"label": "Telemetry Aggregator", "path": "/rover/telemetry"},
    "telemetry_nav": {"label": "Telemetry Nav", "path": "/rover/telemetry/nav"},
    "telemetry_safety": {"label": "Telemetry Safety", "path": "/rover/telemetry/safety"},
    "telemetry_control": {"label": "Telemetry Control", "path": "/rover/telemetry/control"},
    "telemetry_vision": {"label": "Telemetry Vision", "path": "/rover/telemetry/vision"},
    "imu_accel": {"label": "IMU ACCEL", "path": "/imu"},
    "odom_coord": {"label": "ODOM COORD", "path": "/odom"},
    "cmd_vel_echo": {"label": "CMD_VEL ECHO", "path": "/cmd_vel"},
    "gps_fix": {"label": "GPS /FIX", "path": "/gps"},
    "battery_state": {"label": "BATTERY STATE", "path": "/battery_state"},
    "mission_phase": {"label": "Mission Phase", "path": "/mission_phase"},
    "arm_status": {"label": "Arm Status", "path": "/arm_status"},
    "speed_limit": {"label": "Speed Limit", "path": "/speed_limit"},
    "rover_hud": {"label": "Rover Racer HUD", "path": "/rover/hud"},
    "wheel_cmds": {"label": "Wheel Commands", "path": "/wheel_cmds"},
}

topic_config = dict(DEFAULT_TOPICS)
TOPIC_CONFIG_FILE = os.path.join(STATIC_DIR, "topic_config.json")
NETWORK_CONFIG_FILE = os.path.join(STATIC_DIR, "network_config.json")

def load_topic_config():
    global topic_config
    if os.path.exists(TOPIC_CONFIG_FILE):
        try:
            with open(TOPIC_CONFIG_FILE, "r") as f:
                loaded = json.load(f)
                for k, v in DEFAULT_TOPICS.items():
                    if k in loaded and isinstance(loaded[k], dict):
                        topic_config[k] = {
                            "label": loaded[k].get("label", v["label"]),
                            "path": loaded[k].get("path", v["path"])
                        }
                    else:
                        topic_config[k] = dict(v)
        except Exception as e:
            print(f"[WARN] Failed to load topic config: {e}")
            topic_config = dict(DEFAULT_TOPICS)
    else:
        topic_config = dict(DEFAULT_TOPICS)

def save_topic_config():
    try:
        with open(TOPIC_CONFIG_FILE, "w") as f:
            json.dump(topic_config, f, indent=2)
    except Exception as e:
        print(f"[WARN] Failed to save topic config: {e}")

def load_network_config():
    config = {}
    if os.path.exists(NETWORK_CONFIG_FILE):
        try:
            with open(NETWORK_CONFIG_FILE, "r") as f:
                config = json.load(f)
        except Exception as e:
            print(f"[WARN] Failed to load network config: {e}")
            
    domain = str(config.get("domain", "32"))
    discovery = str(config.get("discovery", "SUBNET"))
    peer_id = str(config.get("peer_id", "gcs-operator"))
    local_ip = str(config.get("local_ip", "127.0.0.1"))
    
    os.environ["ROS_DOMAIN_ID"] = domain
    if discovery == "LOCALHOST":
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    elif discovery == "SUBNET":
        os.environ["ROS_LOCALHOST_ONLY"] = "0"
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
    elif discovery == "OFF":
        os.environ["ROS_LOCALHOST_ONLY"] = "0"
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "OFF"
    else:
        os.environ.pop("ROS_LOCALHOST_ONLY", None)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SYSTEM_DEFAULT"
    
    get_global_connector(
        local_peer_id=peer_id,
        ros_domain_id=int(domain),
        enable_discovery=(discovery != "OFF")
    )
