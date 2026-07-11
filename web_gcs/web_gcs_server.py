#!/usr/bin/env python3
"""
Web GCS Server Bridge with Global Connector integration.

Acts as a ROS2 subscriber/publisher node and runs a threaded HTTP server
to serve the HTML/CSS/JS frontend dashboard and stream telemetry via SSE.
Includes endpoints for the Global Connector peer management.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
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
    "last_camera_update": 0.0,
}
command_publisher = None
cmd_vel_publisher = None
sim_estop_triggered = False
_bridge_node = None

def _get_bridge_node():
    return _bridge_node

# Flask & SocketIO Imports
from flask import Flask, request, jsonify, Response, abort
import queue

# Load metadata-driven custom modules
from web_gcs.topic_registry import get_registry, update_topic_data, load_registry
from web_gcs.websocket import socketio, init_websocket, start_websocket_worker
from web_gcs.api import api_bp

# Define Flask application
app = Flask(__name__, static_folder=STATIC_DIR, static_url_path="")
app.register_blueprint(api_bp)
init_websocket(app)

import math
import web_gcs.topic_registry as tr

def my_telemetry_update_hook(name, data, timestamp):
    with data_lock:
        if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
            telemetry_state["latest"] = {
                "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}
            }
        latest = telemetry_state["latest"]
        telemetry_state["last_update"] = timestamp or time.time()
        
        if name == "/battery_state":
            pct = float(data.get("percentage", 0.0))
            if pct <= 1.0:
                pct *= 100.0
            volts = float(data.get("voltage", 0.0))
            latest["Battery"] = {
                "available": True,
                "voltage": volts,
                "percentage": pct,
            }
            jetson = latest.setdefault("Jetson", {})
            jetson["bat_pct"] = pct
            jetson["bat_voltage"] = volts
            
        elif name in ("/gps", "/fix", "/gps/raw"):
            latest["GPS"] = {
                "available": True,
                "latitude": float(data.get("latitude", 0.0)),
                "longitude": float(data.get("longitude", 0.0)),
                "altitude": float(data.get("altitude", 0.0)),
            }
            
        elif name in ("/imu", "/compass_imu"):
            sensors = latest.setdefault("Sensors", {})
            sensors["imu"] = {
                "available": True,
                "accel_x": float(data.get("linear_acceleration", {}).get("x", 0.0)),
                "accel_y": float(data.get("linear_acceleration", {}).get("y", 0.0)),
                "accel_z": float(data.get("linear_acceleration", {}).get("z", 0.0)),
                "gyro_x": float(data.get("angular_velocity", {}).get("x", 0.0)),
                "gyro_y": float(data.get("angular_velocity", {}).get("y", 0.0)),
                "gyro_z": float(data.get("angular_velocity", {}).get("z", 0.0)),
            }
            ori = data.get("orientation", {})
            ox = float(ori.get("x", 0.0))
            oy = float(ori.get("y", 0.0))
            oz = float(ori.get("z", 0.0))
            ow = float(ori.get("w", 1.0))
            siny_cosp = 2.0 * (ow * oz + ox * oy)
            cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            latest.setdefault("Navigation", {})["heading_deg"] = math.degrees(yaw_rad) % 360.0

        elif name in ("/odom", "/odometry"):
            pose = data.get("pose", {}).get("pose", {})
            position = pose.get("position", {})
            orientation = pose.get("orientation", {})
            twist = data.get("twist", {}).get("twist", {})
            linear = twist.get("linear", {})
            angular = twist.get("angular", {})
            vx = float(linear.get("x", 0.0))
            vy = float(linear.get("y", 0.0))
            speed_kmh = math.sqrt(vx*vx + vy*vy) * 3.6
            latest["Odom"] = {
                "available": True,
                "pos_x": float(position.get("x", 0.0)),
                "pos_y": float(position.get("y", 0.0)),
                "pos_z": float(position.get("z", 0.0)),
                "ori_x": float(orientation.get("x", 0.0)),
                "ori_y": float(orientation.get("y", 0.0)),
                "ori_z": float(orientation.get("z", 0.0)),
                "ori_w": float(orientation.get("w", 1.0)),
                "speed_kmh": float(speed_kmh),
                "twist_linear_x": float(vx),
                "twist_angular_z": float(angular.get("z", 0.0)),
            }
            nav = latest.setdefault("Navigation", {})
            nav["speed_kmh"] = speed_kmh
            ox = float(orientation.get("x", 0.0))
            oy = float(orientation.get("y", 0.0))
            oz = float(orientation.get("z", 0.0))
            ow = float(orientation.get("w", 1.0))
            siny_cosp = 2.0 * (ow * oz + ox * oy)
            cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            nav["heading_deg"] = math.degrees(yaw_rad) % 360.0

        elif name in ("/cmd_vel", "/cmd_vel_teleop", "/cmd_vel_nav", "/cmd_vel_int"):
            linear = data.get("linear", {})
            angular = data.get("angular", {})
            latest["CmdVelEcho"] = {
                "linear_x": float(linear.get("x", 0.0)),
                "linear_y": float(linear.get("y", 0.0)),
                "angular_z": float(angular.get("z", 0.0)),
            }
            
        elif name == "/rover/telemetry":
            latest.update(data)

        elif name == "/scan":
            try:
                raw_ranges = data.get("ranges", [])
                range_min = float(data.get("range_min", 0.1))
                range_max = float(data.get("range_max", 12.0))
                angle_min = float(data.get("angle_min", -math.pi))
                angle_max = float(data.get("angle_max", math.pi))
                angle_inc = float(data.get("angle_increment", 0.0118))
                
                valid_ranges = [r for r in raw_ranges if r is not None and range_min <= r <= range_max]
                fwd_idx = int((-angle_min) / angle_inc) if angle_inc > 0 else 0
                fwd_idx = max(0, min(fwd_idx, len(raw_ranges) - 1))
                fwd_range = raw_ranges[fwd_idx] if (raw_ranges and fwd_idx < len(raw_ranges) and raw_ranges[fwd_idx] is not None) else float("inf")
                
                n = len(raw_ranges)
                max_points = 180
                if n > max_points:
                    step = max(1, n // max_points)
                    downsampled = [raw_ranges[i] for i in range(0, n, step)][:max_points]
                else:
                    downsampled = raw_ranges
                    
                formatted_ranges = [
                    round(r, 2) if (r is not None and range_min <= r <= range_max and math.isfinite(r)) else None
                    for r in downsampled
                ]
                
                scan_payload = {
                    "available": True,
                    "frame_id": data.get("header", {}).get("frame_id", "laser_frame"),
                    "angle_min_rad": round(angle_min, 4),
                    "angle_max_rad": round(angle_max, 4),
                    "range_min_m": round(range_min, 3),
                    "range_max_m": round(range_max, 3),
                    "num_points": len(raw_ranges),
                    "num_valid": len(valid_ranges),
                    "forward_range_m": round(fwd_range, 3) if (fwd_range != float("inf") and math.isfinite(fwd_range)) else None,
                    "min_range_m": round(min(valid_ranges), 3) if valid_ranges else None,
                    "max_range_m": round(max(valid_ranges), 3) if valid_ranges else None,
                    "ranges": formatted_ranges,
                }
                
                sensors = latest.setdefault("Sensors", {})
                sensors["scan"] = scan_payload
            except Exception as e:
                print(f"[WARN] Failed to process scan telemetry payload: {e}")

tr.TELEMETRY_UPDATE_HOOK = my_telemetry_update_hook

def my_drive_command_hook(command):
    action = command.get("action")
    
    global sim_estop_triggered
    if action == "estop":
        sim_estop_triggered = True
    elif action == "resume":
        sim_estop_triggered = False

    if action == "drive":
        speed = float(command.get("speed_kmh", 0.0))
        heading = float(command.get("heading_deg", 0.0))
        throttle = float(command.get("throttle_pct", 0.0))
        
        if not (0.0 <= speed <= 15.0):
            raise ValueError("Speed out of bounds [0, 15]")
        if not (0.0 <= heading <= 360.0):
            raise ValueError("Heading out of bounds [0, 360]")
        if not (0.0 <= throttle <= 1.0):
            raise ValueError("Throttle out of bounds [0, 1]")

        if cmd_vel_publisher:
            try:
                from geometry_msgs.msg import Twist
                twist = Twist()
                
                # Closed-loop proportional control using latest telemetry heading
                current_heading = 0.0
                latest = telemetry_state.get("latest")
                if latest and isinstance(latest, dict):
                    nav = latest.get("Navigation")
                    if nav and isinstance(nav, dict):
                        current_heading = float(nav.get("heading_deg", 0.0))

                error_deg = (heading - current_heading + 540.0) % 360.0 - 180.0
                twist.linear.x = round(speed / 3.6, 4)
                angular_z = max(-1.0, min(1.0, -math.radians(error_deg) * 0.4))
                twist.angular.z = round(angular_z, 4)
                cmd_vel_publisher.publish(twist)
            except Exception as e:
                print(f"[WARN] Failed to publish Twist on /cmd_vel: {e}")

    elif action in ("estop", "stop"):
        if cmd_vel_publisher:
            try:
                from geometry_msgs.msg import Twist
                cmd_vel_publisher.publish(Twist())
            except Exception as e:
                pass

    if command_publisher:
        msg = String()
        msg.data = json.dumps(command, separators=(",", ":"))
        command_publisher.publish(msg)

from web_gcs import websocket as ws
ws.DRIVE_COMMAND_HOOK = my_drive_command_hook

@app.route("/")
@app.route("/index.html")
def serve_index():
    return app.send_static_file("index.html")

@app.route("/events")
def events():
    def event_generator():
        last_sent_time = 0.0
        while True:
            with data_lock:
                latest = telemetry_state["latest"]
                last_update = telemetry_state["last_update"]
            
            # Only push if there's new data or every 1 second to keep connection alive
            if latest and (last_update > last_sent_time or time.time() - last_sent_time > 1.0):
                connector = get_global_connector()
                peers_snapshot = connector.get_peers_snapshot()

                payload = json.dumps({
                    "telemetry": latest,
                    "connected": (time.time() - last_update) < 2.0,
                    "peers": peers_snapshot,
                    "peer_count": connector.peer_count,
                    "peers_connected": connector.connected_count,
                })
                yield f"data: {payload}\n\n"
                last_sent_time = time.time()
            time.sleep(0.05)

    return Response(event_generator(), mimetype="text/event-stream")

@app.route("/api/peers", methods=["GET"])
def get_peers():
    connector = get_global_connector()
    return jsonify({
        "peers": connector.get_peers_snapshot(),
        "peer_count": connector.peer_count,
        "connected_count": connector.connected_count,
        "local_peer_id": connector._local_peer_id,
    })

@app.route("/api/peers/<peer_id>/telemetry", methods=["GET"])
def get_peer_telemetry(peer_id):
    connector = get_global_connector()
    telemetry = connector.get_peer_telemetry(peer_id)
    if telemetry:
        return jsonify({"peer_id": peer_id, "telemetry": telemetry})
    else:
        return jsonify({"error": f"No telemetry for peer '{peer_id}'"}), 404

@app.route("/api/peers/telemetry/all", methods=["GET"])
def get_all_peers_telemetry():
    connector = get_global_connector()
    merged = connector.get_merged_telemetry()
    return jsonify(merged)

@app.route("/api/connector/status", methods=["GET"])
def get_connector_status():
    connector = get_global_connector()
    return jsonify({
        "local_peer_id": connector._local_peer_id,
        "peer_count": connector.peer_count,
        "connected_count": connector.connected_count,
        "running": connector._running.is_set(),
    })

@app.route("/api/setup/interfaces", methods=["GET"])
def get_setup_interfaces():
    ips = []
    try:
        import subprocess
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"], capture_output=True, text=True)
        for line in out.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 4:
                name = parts[1]
                ip = parts[3].split("/")[0]
                ips.append({"interface": name, "ip": ip})
    except Exception:
        pass
    if not ips:
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ips.append({"interface": "primary", "ip": s.getsockname()[0]})
            s.close()
        except Exception:
            ips.append({"interface": "localhost", "ip": "127.0.0.1"})
    return jsonify({"interfaces": ips})

@app.route("/api/config/topics", methods=["GET"])
def get_api_config_topics():
    return jsonify(topic_config)

@app.route("/api/config/topics", methods=["POST"])
def post_api_config_topics():
    try:
        data = request.get_json(force=True)
        for k, v in DEFAULT_TOPICS.items():
            if k in data and isinstance(data[k], dict):
                topic_config[k] = {
                    "label": str(data[k].get("label", v["label"])),
                    "path": str(data[k].get("path", v["path"]))
                }
        save_topic_config()
        node = _get_bridge_node()
        if node is not None and hasattr(node, "update_topics"):
            node.update_topics()
        return jsonify({"status": "success", "message": "Topic configuration saved & updated"})
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 400

@app.route("/api/config/topics/reset", methods=["POST"])
def post_api_config_topics_reset():
    try:
        topic_config.clear()
        topic_config.update(DEFAULT_TOPICS)
        if os.path.exists(TOPIC_CONFIG_FILE):
            try:
                os.remove(TOPIC_CONFIG_FILE)
            except Exception:
                pass
        node = _get_bridge_node()
        if node is not None and hasattr(node, "update_topics"):
            node.update_topics()
        return jsonify({"status": "success", "message": "Topic configuration reset to default"})
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 400

@app.route("/command", methods=["POST"])
def post_command():
    try:
        command = request.get_json(force=True)
        my_drive_command_hook(command)
        return jsonify({"status": "success", "message": "Command dispatched"})
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 400

@app.route("/api/peers", methods=["POST"])
def post_peers():
    try:
        peer_data = request.get_json(force=True)
        peer_id = peer_data.get("peer_id", "")
        ip = peer_data.get("ip_address", "")
        port = int(peer_data.get("port", 8090))
        role = peer_data.get("role", "rover")
        team_name = peer_data.get("team_name", "")
        ros_domain_id = int(peer_data.get("ros_domain_id", 0))

        if not peer_id or not ip:
            raise ValueError("peer_id and ip_address are required")

        connector = get_global_connector()
        connector.add_manual_peer(
            peer_id=peer_id,
            ip=ip,
            port=port,
            role=role,
            team_name=team_name,
            ros_domain_id=ros_domain_id,
        )
        return jsonify({
            "status": "success",
            "message": f"Peer '{peer_id}' added @ {ip}:{port}",
        })
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 400

@app.route("/api/test-suite/start", methods=["POST"])
def post_test_suite_start():
    try:
        import shutil
        import subprocess

        def spawn_terminal(title: str, cmd_str: str) -> bool:
            if shutil.which("gnome-terminal"):
                try:
                    subprocess.Popen(["gnome-terminal", f"--title={title}", "--", "bash", "-c", cmd_str])
                    return True
                except Exception:
                    pass
            if shutil.which("x-terminal-emulator"):
                try:
                    subprocess.Popen(["x-terminal-emulator", "-e", "bash", "-c", cmd_str])
                    return True
                except Exception:
                    pass
            if shutil.which("xterm"):
                try:
                    subprocess.Popen(["xterm", "-title", title, "-e", "bash", "-c", cmd_str])
                    return True
                except Exception:
                    pass
            return False

        t1_cmd = "source /opt/ros/humble/setup.bash && cd /home/medochi/GS/rover_ws && colcon build --packages-select rover_core && source install/setup.bash && ros2 launch rover_core rover_bringup.launch.py; exec bash"
        t2_cmd = "source /opt/ros/humble/setup.bash && source /home/medochi/GS/rover_ws/install/setup.bash && ros2 topic echo /rover/commands/motor; exec bash"

        s1 = spawn_terminal("ROS2 Rover Bringup (Simulation)", t1_cmd)
        s2 = spawn_terminal("ROS2 Motor Commands Monitor", t2_cmd)

        if s1 and s2:
            return jsonify({
                "status": "success",
                "message": "Spawned two test terminals (Bringup + Motor Command echo)!"
            })
        else:
            return jsonify({
                "status": "error",
                "message": "Failed to spawn terminals. Make sure gnome-terminal, xterm, or x-terminal-emulator is installed and running in a graphical session."
            }), 500
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 500

@app.route("/api/test-suite/stop", methods=["POST"])
def post_test_suite_stop():
    try:
        import subprocess
        subprocess.run(["pkill", "-f", "rover_bringup.launch.py"])
        subprocess.run(["pkill", "-f", "ros2 launch"])
        subprocess.run(["pkill", "-f", "motor_control_node"])
        subprocess.run(["pkill", "-f", "navigation_node"])
        subprocess.run(["pkill", "-f", "safety_node"])
        subprocess.run(["pkill", "-f", "vision_node"])
        subprocess.run(["pkill", "-f", "telemetry_aggregator"])
        return jsonify({
            "status": "success",
            "message": "Terminated simulated rover bringup and topic monitor instances."
        })
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 500

@app.route("/api/test-suite/run", methods=["POST"])
def post_test_suite_run():
    try:
        import subprocess
        cmd = ["/home/medochi/GS/.venv/bin/python", "-m", "unittest", "tests/test_gcs_rover.py"]
        res = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd="/home/medochi/GS"
        )
        logs = res.stdout + "\n" + res.stderr
        success = res.returncode == 0
        return jsonify({
            "status": "success" if success else "failure",
            "message": "Test suite executed successfully!" if success else "Test suite failed!",
            "logs": logs.strip(),
        })
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 500

@app.route("/api/test-suite/ui-run", methods=["POST"])
def post_test_suite_ui_run():
    try:
        import subprocess
        cmd = ["/home/medochi/GS/.venv/bin/python", "-m", "unittest", "tests.test_gcs_rover.TestWebGCSUI"]
        res = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd="/home/medochi/GS"
        )
        logs = res.stdout + "\n" + res.stderr
        success = res.returncode == 0
        return jsonify({
            "status": "success" if success else "failure",
            "message": "UI test suite executed successfully!" if success else "UI test suite failed!",
            "logs": logs.strip(),
        })
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 500

@app.route("/api/setup/ping", methods=["POST"])
def post_setup_ping():
    try:
        params = request.get_json(force=True)
        host = params.get("host", "127.0.0.1")
        
        import socket
        try:
            socket.gethostbyname(host)
        except Exception:
            raise ValueError(f"Invalid hostname or IP address format: '{host}'")

        import subprocess
        res = subprocess.run(
            ["ping", "-c", "3", "-W", "1", host],
            capture_output=True,
            text=True
        )
        logs = res.stdout + "\n" + res.stderr
        success = res.returncode == 0
        return jsonify({
            "status": "success" if success else "failure",
            "logs": logs.strip(),
        })
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 400

@app.route("/api/peers/<peer_id>", methods=["DELETE"])
def delete_peer(peer_id):
    connector = get_global_connector()
    removed = connector.remove_peer(peer_id)
    if removed:
        return jsonify({"status": "success", "message": f"Peer '{peer_id}' removed"})
    else:
        return jsonify({"status": "error", "message": f"Peer '{peer_id}' not found"}), 404

@app.route("/api/ros2/topics", methods=["GET"])
def get_api_ros2_topics():
    topics = []
    topics_json_path = os.path.join(STATIC_DIR, "topics.json")
    if os.path.exists(topics_json_path):
        try:
            with open(topics_json_path, "r") as f:
                raw_topics = json.load(f)
            for t in raw_topics:
                t_name = t.get("name")
                t_type = t.get("type")
                types = [x.strip() for x in t_type.split("\n") if x.strip()] if t_type else ["unknown"]
                topics.append({"name": t_name, "types": types})
        except Exception:
            pass
    return jsonify({"topics": topics})

@app.route("/api/config/network", methods=["GET"])
def get_api_config_network():
    if os.path.exists(NETWORK_CONFIG_FILE):
        try:
            with open(NETWORK_CONFIG_FILE, "r") as f:
                config = json.load(f)
                return jsonify(config)
        except Exception:
            pass
    return jsonify({
        "domain": "0",
        "discovery": "SUBNET",
        "peer_id": "gcs-operator",
        "local_ip": "127.0.0.1"
    })

@app.route("/api/config/network", methods=["POST"])
def post_api_config_network():
    try:
        data = request.get_json(force=True)
        config = {
            "domain": str(data.get("domain", "0")),
            "discovery": str(data.get("discovery", "SUBNET")),
            "peer_id": str(data.get("peer_id", "gcs-operator")),
            "local_ip": str(data.get("local_ip", "127.0.0.1"))
        }
        with open(NETWORK_CONFIG_FILE, "w") as f:
            json.dump(config, f, indent=2)
        load_network_config()
        return jsonify({"status": "success", "message": "Network configuration saved & applied"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

@app.route("/api/ssh/status", methods=["GET"])
def get_api_ssh_status():
    return jsonify({
        "connected": False,
        "tunnel_active": False,
        "bringup_active": False,
        "host": "",
        "user": ""
    })

@app.route("/api/ssh/tunnel/start", methods=["POST"])
def post_api_ssh_tunnel_start():
    return jsonify({"status": "error", "message": "SSH tunnel feature not configured"}), 400

@app.route("/api/ssh/tunnel/stop", methods=["POST"])
def post_api_ssh_tunnel_stop():
    return jsonify({"status": "success", "message": "SSH tunnel stopped"})

@app.route("/api/ssh/launch-bringup", methods=["POST"])
def post_api_ssh_launch_bringup():
    return jsonify({"status": "error", "message": "SSH remote bringup not configured"}), 400

@app.route("/api/ssh/stop-bringup", methods=["POST"])
def post_api_ssh_stop_bringup():
    return jsonify({"status": "success", "message": "Remote bringup stopped"})

@app.route("/api/ssh/execute", methods=["POST"])
def post_api_ssh_execute():
    return jsonify({"status": "error", "message": "SSH command execution not configured"}), 400


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
}

topic_config = dict(DEFAULT_TOPICS)
TOPIC_CONFIG_FILE = os.path.join(STATIC_DIR, "topic_config.json")

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

NETWORK_CONFIG_FILE = os.path.join(STATIC_DIR, "network_config.json")

def load_network_config():
    if os.path.exists(NETWORK_CONFIG_FILE):
        try:
            with open(NETWORK_CONFIG_FILE, "r") as f:
                config = json.load(f)
                domain = str(config.get("domain", "0"))
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
        except Exception as e:
            print(f"[WARN] Failed to load network config: {e}")



class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """
    [LEGACY COMPATIBILITY] Threaded HTTP server for integration testing.
    NOTE: Preserved solely for backward-compatibility with `TestWebGCSUI`
    in `tests/test_gcs_rover.py`. Not used during normal GCS runtime.
    """
    daemon_threads = True


class GCSWebHandler(BaseHTTPRequestHandler):
    """
    [LEGACY COMPATIBILITY] HTTP request handler for integration testing.
    NOTE: Preserved solely for backward-compatibility with `TestWebGCSUI`
    in `tests/test_gcs_rover.py`. Not used during normal GCS runtime.
    """

    def log_message(self, format, *args):
        # Suppress request spam logs to keep stdout clean
        pass

    def send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_cors_headers()
        self.end_headers()

    def do_GET(self):
        if self.path == "/api/config/topics":
            self._send_json(200, json.dumps(topic_config))
            return

        if self.path == "/api/ros2/topics":
            topics = []
            topics_json_path = os.path.join(STATIC_DIR, "topics.json")
            if os.path.exists(topics_json_path):
                try:
                    with open(topics_json_path, "r") as f:
                        raw_topics = json.load(f)
                    for t in raw_topics:
                        t_name = t.get("name")
                        t_type = t.get("type")
                        types = [x.strip() for x in t_type.split("\n") if x.strip()] if t_type else ["unknown"]
                        topics.append({"name": t_name, "types": types})
                except Exception as e:
                    pass
            self._send_json(200, json.dumps({"topics": topics}))
            return

        if self.path == "/events":
            # Server-Sent Events stream for real-time telemetry
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_cors_headers()
            self.end_headers()

            last_sent_time = 0.0
            try:
                while True:
                    with data_lock:
                        latest = telemetry_state["latest"]
                        last_update = telemetry_state["last_update"]

                    # Only push if there's new data or every 1 second to keep connection alive
                    if latest and (last_update > last_sent_time or time.time() - last_sent_time > 1.0):
                        # Include peer data from global connector
                        connector = get_global_connector()
                        peers_snapshot = connector.get_peers_snapshot()

                        payload = json.dumps({
                            "telemetry": latest,
                            "connected": (time.time() - last_update) < 2.0,
                            "peers": peers_snapshot,
                            "peer_count": connector.peer_count,
                            "peers_connected": connector.connected_count,
                            "camera_frame": telemetry_state["camera_frame"],
                        })
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        last_sent_time = time.time()

                    time.sleep(0.05)
            except (ConnectionResetError, BrokenPipeError):
                # Client disconnected
                return
            return

        # ── Global Connector API endpoints ──
        if self.path == "/api/peers":
            connector = get_global_connector()
            response = json.dumps({
                "peers": connector.get_peers_snapshot(),
                "peer_count": connector.peer_count,
                "connected_count": connector.connected_count,
                "local_peer_id": connector._local_peer_id,
            })
            self._send_json(200, response)
            return

        if self.path.startswith("/api/peers/") and "/telemetry" in self.path:
            peer_id = self.path.split("/api/peers/")[1].split("/telemetry")[0]
            connector = get_global_connector()
            telemetry = connector.get_peer_telemetry(peer_id)
            if telemetry:
                self._send_json(200, json.dumps({"peer_id": peer_id, "telemetry": telemetry}))
            else:
                self._send_json(404, json.dumps({"error": f"No telemetry for peer '{peer_id}'"}))
            return

        if self.path == "/api/peers/telemetry/all":
            connector = get_global_connector()
            merged = connector.get_merged_telemetry()
            self._send_json(200, json.dumps(merged, default=str))
            return

        if self.path == "/api/connector/status":
            connector = get_global_connector()
            self._send_json(200, json.dumps({
                "local_peer_id": connector._local_peer_id,
                "peer_count": connector.peer_count,
                "connected_count": connector.connected_count,
                "running": connector._running.is_set(),
            }))
            return

        if self.path == "/api/setup/interfaces":
            ips = []
            try:
                import subprocess
                out = subprocess.run(["ip", "-o", "-4", "addr", "show"], capture_output=True, text=True)
                for line in out.stdout.splitlines():
                    parts = line.split()
                    if len(parts) >= 4:
                        name = parts[1]
                        ip = parts[3].split("/")[0]
                        ips.append({"interface": name, "ip": ip})
            except Exception:
                pass
            if not ips:
                try:
                    import socket
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    ips.append({"interface": "primary", "ip": s.getsockname()[0]})
                    s.close()
                except Exception:
                    ips.append({"interface": "localhost", "ip": "127.0.0.1"})
            self._send_json(200, json.dumps({"interfaces": ips}))
            return

        if self.path == "/api/topics/img/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_cors_headers()
            self.end_headers()

            try:
                import base64
                last_frame_b64 = None
                while True:
                    with data_lock:
                        frame_b64 = telemetry_state["camera_frame"]
                    
                    if frame_b64 and frame_b64 != last_frame_b64:
                        try:
                            jpeg_bytes = base64.b64decode(frame_b64)
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpeg_bytes)}\r\n\r\n".encode("utf-8"))
                            self.wfile.write(jpeg_bytes)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                            last_frame_b64 = frame_b64
                        except Exception:
                            pass
                    
                    time.sleep(0.04)
            except (ConnectionResetError, BrokenPipeError):
                return
            except Exception:
                pass
            return

        # Serve static assets
        if self.path in ("/", "/index.html"):
            filename = "index.html"
            content_type = "text/html"
        elif self.path == "/styles.css":
            filename = "styles.css"
            content_type = "text/css"
        elif self.path == "/app.js":
            filename = "app.js"
            content_type = "application/javascript"
        else:
            self.send_error(404, "File Not Found")
            return

        filepath = os.path.join(STATIC_DIR, filename)
        if os.path.exists(filepath):
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_cors_headers()
            self.end_headers()
            with open(filepath, "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404, "File Not Found")

    def do_POST(self):
        if self.path == "/api/config/topics":
            content_length = int(self.headers.get("Content-Length", 0))
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode("utf-8"))
                for k, v in DEFAULT_TOPICS.items():
                    if k in data and isinstance(data[k], dict):
                        topic_config[k] = {
                            "label": str(data[k].get("label", v["label"])),
                            "path": str(data[k].get("path", v["path"]))
                        }
                save_topic_config()
                node = _get_bridge_node()
                if node is not None and hasattr(node, "update_topics"):
                    node.update_topics()
                self._send_json(200, json.dumps({"status": "success", "message": "Topic configuration saved & updated"}))
            except Exception as exc:
                self._send_json(400, json.dumps({"status": "error", "message": str(exc)}))
            return

        if self.path == "/api/config/topics/reset":
            try:
                topic_config.clear()
                topic_config.update(DEFAULT_TOPICS)
                if os.path.exists(TOPIC_CONFIG_FILE):
                    try:
                        os.remove(TOPIC_CONFIG_FILE)
                    except Exception:
                        pass
                node = _get_bridge_node()
                if node is not None and hasattr(node, "update_topics"):
                    node.update_topics()
                self._send_json(200, json.dumps({"status": "success", "message": "Topic configuration reset to default"}))
            except Exception as exc:
                self._send_json(400, json.dumps({"status": "error", "message": str(exc)}))
            return

        if self.path == "/command":
            content_length = int(self.headers.get("Content-Length", 0))
            post_data = self.rfile.read(content_length)
            try:
                command = json.loads(post_data.decode("utf-8"))
                
                # Enforce bounds checks on the backend for safety
                action = command.get("action")
                
                # Update local simulation E-Stop state
                global sim_estop_triggered
                if action == "estop":
                    sim_estop_triggered = True
                elif action == "resume":
                    sim_estop_triggered = False

                if action == "drive":
                    speed = float(command.get("speed_kmh", 0.0))
                    heading = float(command.get("heading_deg", 0.0))
                    throttle = float(command.get("throttle_pct", 0.0))
                    
                    if not (0.0 <= speed <= 15.0):
                        raise ValueError("Speed out of bounds [0, 15]")
                    if not (0.0 <= heading <= 360.0):
                        raise ValueError("Heading out of bounds [0, 360]")
                    if not (0.0 <= throttle <= 1.0):
                        raise ValueError("Throttle out of bounds [0, 1]")

                # Publish command
                if command_publisher:
                    msg = String()
                    msg.data = json.dumps(command, separators=(",", ":"))
                    command_publisher.publish(msg)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_cors_headers()
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success", "message": "Command dispatched"}).encode("utf-8"))
            except Exception as exc:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.send_cors_headers()
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(exc)}).encode("utf-8"))

        # ── Global Connector: Add manual peer ──
        elif self.path == "/api/peers":
            content_length = int(self.headers.get("Content-Length", 0))
            post_data = self.rfile.read(content_length)
            try:
                peer_data = json.loads(post_data.decode("utf-8"))
                peer_id = peer_data.get("peer_id", "")
                ip = peer_data.get("ip_address", "")
                port = int(peer_data.get("port", 8090))
                role = peer_data.get("role", "rover")
                team_name = peer_data.get("team_name", "")
                ros_domain_id = int(peer_data.get("ros_domain_id", 0))

                if not peer_id or not ip:
                    raise ValueError("peer_id and ip_address are required")

                connector = get_global_connector()
                connector.add_manual_peer(
                    peer_id=peer_id,
                    ip=ip,
                    port=port,
                    role=role,
                    team_name=team_name,
                    ros_domain_id=ros_domain_id,
                )
                self._send_json(200, json.dumps({
                    "status": "success",
                    "message": f"Peer '{peer_id}' added @ {ip}:{port}",
                }))
            except Exception as exc:
                self._send_json(400, json.dumps({"status": "error", "message": str(exc)}))

        # ── Test Suite: Start ──
        elif self.path == "/api/test-suite/start":
            try:
                import shutil
                import subprocess

                # Helper to spawn terminal
                def spawn_terminal(title: str, cmd_str: str) -> bool:
                    # Try gnome-terminal
                    if shutil.which("gnome-terminal"):
                        try:
                            subprocess.Popen(["gnome-terminal", f"--title={title}", "--", "bash", "-c", cmd_str])
                            return True
                        except Exception:
                            pass
                    # Try x-terminal-emulator
                    if shutil.which("x-terminal-emulator"):
                        try:
                            subprocess.Popen(["x-terminal-emulator", "-e", "bash", "-c", cmd_str])
                            return True
                        except Exception:
                            pass
                    # Try xterm
                    if shutil.which("xterm"):
                        try:
                            subprocess.Popen(["xterm", "-title", title, "-e", "bash", "-c", cmd_str])
                            return True
                        except Exception:
                            pass
                    return False

                # Terminal 1: Build the workspace and run the ROS2 Rover Stack
                t1_cmd = "source /opt/ros/humble/setup.bash && cd /home/medochi/GS/rover_ws && colcon build --packages-select rover_core && source install/setup.bash && ros2 launch rover_core rover_bringup.launch.py; exec bash"
                
                # Terminal 2: Monitor motor command topic
                t2_cmd = "source /opt/ros/humble/setup.bash && source /home/medochi/GS/rover_ws/install/setup.bash && ros2 topic echo /rover/commands/motor; exec bash"

                # Attempt to spawn both
                s1 = spawn_terminal("ROS2 Rover Bringup (Simulation)", t1_cmd)
                s2 = spawn_terminal("ROS2 Motor Commands Monitor", t2_cmd)

                if s1 and s2:
                    self._send_json(200, json.dumps({
                        "status": "success",
                        "message": "Spawned two test terminals (Bringup + Motor Command echo)!"
                    }))
                else:
                    self._send_json(500, json.dumps({
                        "status": "error",
                        "message": "Failed to spawn terminals. Make sure gnome-terminal, xterm, or x-terminal-emulator is installed and running in a graphical session."
                    }))
            except Exception as exc:
                self._send_json(500, json.dumps({"status": "error", "message": str(exc)}))
        # ── Test Suite: Stop ──
        elif self.path == "/api/test-suite/stop":
            try:
                import subprocess
                # Terminate any active ros2 launch or nodes spawned by the simulator
                subprocess.run(["pkill", "-f", "rover_bringup.launch.py"])
                subprocess.run(["pkill", "-f", "ros2 launch"])
                subprocess.run(["pkill", "-f", "motor_control_node"])
                subprocess.run(["pkill", "-f", "navigation_node"])
                subprocess.run(["pkill", "-f", "safety_node"])
                subprocess.run(["pkill", "-f", "vision_node"])
                subprocess.run(["pkill", "-f", "telemetry_aggregator"])
                self._send_json(200, json.dumps({
                    "status": "success",
                    "message": "Terminated simulated rover bringup and topic monitor instances."
                }))
            except Exception as exc:
                self._send_json(500, json.dumps({"status": "error", "message": str(exc)}))

        # ── Test Suite: Run Unittests ──
        elif self.path == "/api/test-suite/run":
            try:
                import subprocess
                cmd = ["/home/medochi/GS/.venv/bin/python", "-m", "unittest", "tests/test_gcs_rover.py"]
                res = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    cwd="/home/medochi/GS"
                )
                logs = res.stdout + "\n" + res.stderr
                success = res.returncode == 0
                self._send_json(200, json.dumps({
                    "status": "success" if success else "failure",
                    "message": "Test suite executed successfully!" if success else "Test suite failed!",
                    "logs": logs.strip(),
                }))
            except Exception as exc:
                self._send_json(500, json.dumps({"status": "error", "message": str(exc)}))

        # ── Test Suite: Run UI Tests ──
        elif self.path == "/api/test-suite/ui-run":
            try:
                import subprocess
                cmd = ["/home/medochi/GS/.venv/bin/python", "-m", "unittest", "tests.test_gcs_rover.TestWebGCSUI"]
                res = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    cwd="/home/medochi/GS"
                )
                logs = res.stdout + "\n" + res.stderr
                success = res.returncode == 0
                self._send_json(200, json.dumps({
                    "status": "success" if success else "failure",
                    "message": "UI test suite executed successfully!" if success else "UI test suite failed!",
                    "logs": logs.strip(),
                }))
            except Exception as exc:
                self._send_json(500, json.dumps({"status": "error", "message": str(exc)}))

        # ── Setup: Ping Rover Host ──
        elif self.path == "/api/setup/ping":
            content_length = int(self.headers.get("Content-Length", 0))
            post_data = self.rfile.read(content_length)
            try:
                params = json.loads(post_data.decode("utf-8"))
                host = params.get("host", "127.0.0.1")
                
                # Input sanitization to prevent command injection
                import socket
                try:
                    # Resolves host / checks if IP format is valid
                    socket.gethostbyname(host)
                except Exception:
                    raise ValueError(f"Invalid hostname or IP address format: '{host}'")

                import subprocess
                res = subprocess.run(
                    ["ping", "-c", "3", "-W", "1", host],
                    capture_output=True,
                    text=True
                )
                logs = res.stdout + "\n" + res.stderr
                success = res.returncode == 0
                self._send_json(200, json.dumps({
                    "status": "success" if success else "failure",
                    "logs": logs.strip(),
                }))
            except Exception as exc:
                self._send_json(400, json.dumps({"status": "error", "message": str(exc)}))



    def do_DELETE(self):
        # ── Global Connector: Remove peer ──
        if self.path.startswith("/api/peers/"):
            peer_id = self.path.split("/api/peers/")[1]
            connector = get_global_connector()
            removed = connector.remove_peer(peer_id)
            if removed:
                self._send_json(200, json.dumps({"status": "success", "message": f"Peer '{peer_id}' removed"}))
            else:
                self._send_json(404, json.dumps({"status": "error", "message": f"Peer '{peer_id}' not found"}))

    def _send_json(self, code: int, body: str) -> None:
        """Helper to send a JSON response."""
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_cors_headers()
        self.end_headers()
        self.wfile.write(body.encode("utf-8"))


class WebGCSBridgeNode(Node):
    """ROS 2 Node that bridges network topics to the HTTP SSE stream."""

    def __init__(self):
        super().__init__("web_gcs_bridge")
        global command_publisher, cmd_vel_publisher
        command_publisher = self.create_publisher(
            String, "/rover/commands/motor", COMMAND_QOS
        )
        try:
            from geometry_msgs.msg import Twist
            cmd_vel_publisher = self.create_publisher(
                Twist, "/cmd_vel", COMMAND_QOS
            )
            self.get_logger().info("/cmd_vel publisher created (geometry_msgs/Twist)")
        except ImportError:
            cmd_vel_publisher = None
            self.get_logger().warning("geometry_msgs not available, /cmd_vel publisher skipped")
        self.subscription = self.create_subscription(
            String,
            "/rover/telemetry",
            self.telemetry_callback,
            RELIABLE_QOS,
        )
        self.get_logger().info("Web GCS Bridge subscriber/publisher initialized.")

    def telemetry_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            with data_lock:
                telemetry_state["latest"] = payload
                telemetry_state["last_update"] = time.time()
            connector = get_global_connector()
            connector.set_local_telemetry(payload)
        except Exception as e:
            self.get_logger().warning(f"Failed to parse incoming telemetry: {e}")

    def update_topics(self):
        pass

    def destroy_subscriptions(self):
        pass

    def camera_callback(self, msg):
        pass

    def camera_raw_callback(self, msg):
        pass

    def scan_callback(self, msg):
        pass

    def mission_callback(self, msg):
        pass

    def arm_callback(self, msg):
        pass

    def speed_limit_callback(self, msg):
        pass

    def imu_callback(self, msg):
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}
                    }
                latest = telemetry_state["latest"]
                if "Sensors" not in latest:
                    latest["Sensors"] = {}
                latest["Sensors"]["imu"] = {
                    "available": True,
                    "accel_x": float(msg.linear_acceleration.x),
                    "accel_y": float(msg.linear_acceleration.y),
                    "accel_z": float(msg.linear_acceleration.z),
                    "gyro_x": float(msg.angular_velocity.x),
                    "gyro_y": float(msg.angular_velocity.y),
                    "gyro_z": float(msg.angular_velocity.z),
                }
                telemetry_state["last_update"] = time.time()
        except Exception:
            pass

    def odom_callback(self, msg):
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}
                    }
                latest = telemetry_state["latest"]
                if "Odom" not in latest:
                    latest["Odom"] = {}
                import math
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                speed_kmh = math.sqrt(vx*vx + vy*vy) * 3.6
                latest["Odom"] = {
                    "available": True,
                    "pos_x": float(msg.pose.pose.position.x),
                    "pos_y": float(msg.pose.pose.position.y),
                    "pos_z": float(msg.pose.pose.position.z),
                    "ori_x": float(msg.pose.pose.orientation.x),
                    "ori_y": float(msg.pose.pose.orientation.y),
                    "ori_z": float(msg.pose.pose.orientation.z),
                    "ori_w": float(msg.pose.pose.orientation.w),
                    "speed_kmh": float(speed_kmh),
                    "twist_linear_x": float(vx),
                    "twist_angular_z": float(msg.twist.twist.angular.z),
                }
                telemetry_state["last_update"] = time.time()
        except Exception:
            pass

    def gps_callback(self, msg):
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}
                    }
                latest = telemetry_state["latest"]
                if "GPS" not in latest:
                    latest["GPS"] = {}
                latest["GPS"] = {
                    "available": True,
                    "latitude": float(msg.latitude),
                    "longitude": float(msg.longitude),
                    "altitude": float(msg.altitude),
                }
                telemetry_state["last_update"] = time.time()
        except Exception:
            pass

    def battery_callback(self, msg):
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}
                    }
                latest = telemetry_state["latest"]
                if "Battery" not in latest:
                    latest["Battery"] = {}
                percentage = float(msg.percentage)
                if percentage <= 1.0:
                    percentage *= 100.0
                latest["Battery"] = {
                    "available": True,
                    "voltage": float(msg.voltage),
                    "percentage": percentage,
                }
                if "Jetson" not in latest:
                    latest["Jetson"] = {}
                latest["Jetson"]["bat_pct"] = percentage
                latest["Jetson"]["bat_voltage"] = float(msg.voltage)
                telemetry_state["last_update"] = time.time()
        except Exception:
            pass

    def cmd_vel_echo_callback(self, msg):
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}
                    }
                latest = telemetry_state["latest"]
                if "CmdVelEcho" not in latest:
                    latest["CmdVelEcho"] = {}
                latest["CmdVelEcho"] = {
                    "linear_x": float(msg.linear.x),
                    "angular_z": float(msg.angular.z),
                    "linear_y": float(msg.linear.y),
                }
                telemetry_state["last_update"] = time.time()
        except Exception:
            pass


def camera_frame_process_worker():
    import numpy as np
    import cv2
    import base64
    import time
    from web_gcs.topic_registry import get_registry

    # Lazy imports for ROS types
    Time = None
    try:
        from rclpy.time import Time
    except ImportError:
        pass

    last_update_time = 0.0
    frame_count = 0
    fps = 0.0
    last_fps_calc_time = None

    while True:
        try:
            registry = get_registry()
            cam_topic = topic_config.get("image_recognition", {}).get("path", "/rgb/image_raw/compressed")
            if cam_topic in registry and registry[cam_topic]["latest_raw"] is not None:
                msg = registry[cam_topic]["latest_raw"]
                last_update = registry[cam_topic]["last_update"]
                arrival_time = registry[cam_topic].get("arrival_time")
                
                if last_update > last_update_time:
                    # Let's initialize last_fps_calc_time if needed
                    if last_fps_calc_time is None:
                        if arrival_time:
                            last_fps_calc_time = arrival_time
                        elif Time:
                            last_fps_calc_time = Time(nanoseconds=int(last_update * 1e9))
                        else:
                            last_fps_calc_time = last_update

                    frame_count += 1
                    
                    # Calculate real-time FPS
                    if arrival_time and Time and hasattr(arrival_time, 'nanoseconds'):
                        elapsed_duration = arrival_time - last_fps_calc_time
                        elapsed_seconds = elapsed_duration.nanoseconds / 1e9
                        if elapsed_seconds >= 1.0:
                            fps = frame_count / elapsed_seconds
                            frame_count = 0
                            last_fps_calc_time = arrival_time
                    else:
                        elapsed_seconds = last_update - last_fps_calc_time
                        if elapsed_seconds >= 1.0:
                            fps = frame_count / elapsed_seconds
                            frame_count = 0
                            last_fps_calc_time = last_update

                    # Calculate latency
                    latency_ms = 0.0
                    if arrival_time and Time and hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                        try:
                            msg_generation_time = Time.from_msg(msg.header.stamp)
                            latency_duration = arrival_time - msg_generation_time
                            latency_ms = latency_duration.nanoseconds / 1e6
                        except Exception:
                            pass
                    elif hasattr(msg, 'header') and hasattr(msg.header, 'stamp') and hasattr(msg.header.stamp, 'sec'):
                        # Fallback simple latency calc
                        msg_gen = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                        latency_ms = max(0.0, (last_update - msg_gen) * 1000.0)

                    cv_image = None
                    if hasattr(msg, "data"):
                        try:
                            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        except Exception:
                            pass
                    
                    if cv_image is not None:
                        # Draw instrumentation dashboard overlay
                        size_kb = len(msg.data) / 1024.0
                        fps_text = f"FPS: {fps:.1f}"
                        latency_text = f"Latency: {latency_ms:.1f} ms"
                        size_text = f"Size: {size_kb:.1f} KB"
                        
                        # Draw overlay
                        cv2.rectangle(cv_image, (10, 10), (260, 95), (0, 0, 0), -1)
                        cv2.putText(cv_image, fps_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.putText(cv_image, latency_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(cv_image, size_text, (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)
                        
                        success, encoded_img = cv2.imencode('.jpg', cv_image)
                        if success:
                            jpeg_bytes = encoded_img.tobytes()
                            b64_str = base64.b64encode(jpeg_bytes).decode('utf-8')
                            
                            registry[cam_topic]["latest_instrumented"] = jpeg_bytes
                            
                            with data_lock:
                                telemetry_state["camera_frame"] = b64_str
                                telemetry_state["last_camera_update"] = last_update
                    else:
                        # Fallback for uncompressed images or non-decodable formats
                        if hasattr(msg, "encoding") and hasattr(msg, "height") and hasattr(msg, "width"):
                            height = msg.height
                            width = msg.width
                            if msg.encoding in ("bgr8", "rgb8"):
                                np_arr_raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                                if msg.encoding == "rgb8":
                                    np_arr_raw = cv2.cvtColor(np_arr_raw, cv2.COLOR_RGB2BGR)
                                
                                success, encoded_img = cv2.imencode('.jpg', np_arr_raw)
                                if success:
                                    jpeg_bytes = encoded_img.tobytes()
                                    b64_str = base64.b64encode(jpeg_bytes).decode('utf-8')
                                    with data_lock:
                                        telemetry_state["camera_frame"] = b64_str
                                        telemetry_state["last_camera_update"] = last_update
                            elif msg.encoding == "jpeg":
                                b64_str = base64.b64encode(msg.data).decode('utf-8')
                                with data_lock:
                                    telemetry_state["camera_frame"] = b64_str
                                    telemetry_state["last_camera_update"] = last_update
                        else:
                            try:
                                data_bytes = bytes(msg.data)
                            except Exception:
                                data_bytes = msg.data
                            b64_str = base64.b64encode(data_bytes).decode('utf-8')
                            with data_lock:
                                telemetry_state["camera_frame"] = b64_str
                                telemetry_state["last_camera_update"] = last_update
                    
                    last_update_time = last_update
        except Exception:
            pass
        time.sleep(0.04)


def run_http_server():
    """Runs the Flask-SocketIO development server at runtime."""
    print(f"[HTTP] Web dashboard serving via Flask-SocketIO at http://localhost:{PORT}")
    try:
        socketio.run(app, host="0.0.0.0", port=PORT, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass


def main(args=None):
    load_registry()
    load_network_config()
    load_topic_config()
    connector = get_global_connector()
    connector.start()

    # Start background camera frame processing worker thread
    t_cam = threading.Thread(target=camera_frame_process_worker, daemon=True)
    t_cam.start()

    # Initialize queue for bridge -> websockets hand-off
    out_queue = queue.Queue()
    start_websocket_worker(out_queue)

    if ROS_AVAILABLE:
        if not rclpy.ok():
            rclpy.init(args=args)
        node = WebGCSBridgeNode()
        global _bridge_node
        _bridge_node = node

        # Start the metadata-driven RosBridge subscriber worker thread
        from web_gcs.ros_subscriber import RosBridge
        ros_bridge = RosBridge(node, out_queue)
        ros_bridge.start()

        try:
            run_http_server()
        except KeyboardInterrupt:
            pass
        finally:
            _bridge_node = None
            try:
                node.destroy_node()
            except Exception: pass
            try:
                rclpy.shutdown()
            except Exception: pass
            shutdown_global_connector()
    else:
        sim_mode = os.environ.get("GCS_SIM_MODE") == "1" or os.environ.get("GCS_SIM_MODE") == "true" or "--sim" in sys.argv
        if not sim_mode:
            print("[INFO] ROS2 not available. Running in standby mode (no simulation data).")
            try:
                run_http_server()
            except KeyboardInterrupt:
                pass
            finally:
                shutdown_global_connector()
        else:
            print("[WARN] ROS2 not available. Running in Fallback Simulation thread.")
            
            # Start simulation thread to periodically feed the registry and compatibility buffers
            def sim_thread_func():
                import random
                import math
                sim_start = time.time()
                tick = 0
                speed = 0.0
                heading = 0.0
                lat = 40.0
                lon = -75.0
                dist = 0.0

                try:
                    with open(os.path.join(STATIC_DIR, "rover_camera_feed.jpg"), "rb") as f:
                        sim_img_bytes = f.read()
                except Exception:
                    sim_img_bytes = b""

                class MockCompressedImage:
                    def __init__(self, data_bytes):
                        self.data = data_bytes

                while True:
                    tick += 1
                    t = time.time()
                    global sim_estop_triggered
                    if sim_estop_triggered:
                        speed = 0.0
                    else:
                        speed = max(0.0, min(12.0, speed + random.uniform(-0.5, 0.7)))
                    heading = (heading + 1.5) % 360.0
                    dist_inc = (speed / 3.6) * 0.1
                    lat += (dist_inc / 111000.0) * math.cos(math.radians(heading))
                    lon += (dist_inc / 111000.0) * math.sin(math.radians(heading))
                    dist += dist_inc

                    # Simulate ranges
                    raw_ranges = [round(max(0.12, min(6.0 + 0.8 * math.sin(i * 0.05), 12.0)), 2) for i in range(180)]
                    bat_pct = round(max(0.0, 100.0 - (tick * 0.005)), 1)
                    bat_volts = round(11.8 - (tick * 0.0005), 2)

                    # 1. Update topic registry (metadata-driven)
                    update_topic_data("/battery_state", {
                        "voltage": bat_volts,
                        "percentage": bat_pct / 100.0
                    }, t)
                    
                    update_topic_data("/odom", {
                        "pose": {
                            "pose": {
                                "position": {"x": dist, "y": 0.0, "z": 0.0},
                                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                            }
                        },
                        "twist": {
                            "twist": {
                                "linear": {"x": speed / 3.6, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                            }
                        }
                    }, t)
                    
                    update_topic_data("/scan", {
                        "ranges": raw_ranges,
                        "range_min": 0.12,
                        "range_max": 12.0
                    }, t)
                    
                    update_topic_data("/gps", {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": 10.0
                    }, t)
                    
                    update_topic_data("/imu", {
                        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
                        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }, t)

                    cam_topic = topic_config.get("image_recognition", {}).get("path", "/rgb/image_raw/compressed")
                    update_topic_data(cam_topic, {
                        "format": "jpeg"
                    }, t)
                    registry = get_registry()
                    if cam_topic in registry:
                        registry[cam_topic]["latest_raw"] = MockCompressedImage(sim_img_bytes)

                    # 2. Update compatibility states (for SSE / events API tests)
                    payload = {
                        "Navigation": {
                            "speed_kmh": round(speed, 2),
                            "heading_deg": round(heading, 1),
                            "pos_lat": round(lat, 6),
                            "pos_lon": round(lon, 6),
                            "dist_traveled_m": round(dist, 1),
                            "wp_current": int(tick / 40) % 10,
                            "wp_error_m": round(abs(math.sin(tick / 15.0)) * 2.0, 2),
                            "wp_status": "idle" if speed <= 0.0 else "navigating"
                        },
                        "Safety": {
                            "mode": "estop" if sim_estop_triggered else ("monitoring" if speed > 0.0 else "idle"),
                            "light_state": "red" if sim_estop_triggered else ("green" if speed > 0.0 else "idle"),
                            "estop_mech_armed": False,
                            "estop_wire_armed": False,
                            "estop_triggered": sim_estop_triggered,
                            "is_blocked": sim_estop_triggered,
                            "collision_detected": False,
                            "border_crossed": False,
                            "border_partial": False,
                            "obstacle_touched": False
                        },
                        "Vision": {
                            "img_confidence": round(random.uniform(0.6, 0.95), 2),
                            "img_detected": True,
                            "laser_active": random.random() > 0.8,
                            "img_elapsed_sec": int(time.time() - sim_start),
                            "img_task_status": "processing",
                            "lane_detected": random.random() > 0.3,
                            "obstacles_count": random.randint(0, 3),
                            "fps_vision": round(29.0 + random.uniform(-1.0, 1.0), 1)
                        },
                        "Jetson": {
                            "cpu_pct": round(42.0 + random.uniform(-10, 10), 1),
                            "gpu_pct": round(25.0 + random.uniform(-8, 8), 1),
                            "ram_pct": 62.4,
                            "temp_c": round(45.0 + (speed * 1.2) + random.uniform(-0.5, 0.5), 1),
                            "bat_pct": bat_pct,
                            "bat_voltage": bat_volts,
                            "uptime_sec": int(time.time() - sim_start)
                        },
                        "Communication": {
                            "rtt_ms": random.randint(18, 35),
                            "channel_rssi": random.randint(-74, -50),
                            "stream_fps": round(29.5 + random.uniform(-0.5, 0.5), 1),
                            "packet_loss_pct": round(max(0.0, random.uniform(-0.2, 0.8)), 1),
                            "heartbeat_seq": tick,
                            "timestamp_ms": int((time.time() - sim_start) * 1000)
                        },
                        "ROS": {
                            "node_lane_det": True,
                            "node_obs_avoid": True,
                            "node_wp_nav": True,
                            "node_img_recog": True,
                            "node_motor_ctrl": True,
                            "esp32_connected": True,
                            "rosout_last": "Local simulation running successfully."
                        },
                        "Sensors": {
                            "imu": {"available": False},
                            "scan": {
                                "available": True,
                                "frame_id": "laser_frame",
                                "angle_min_rad": -3.1416,
                                "angle_max_rad": 3.1416,
                                "range_min_m": 0.12,
                                "range_max_m": 12.0,
                                "num_points": 180,
                                "num_valid": 180,
                                "forward_range_m": 12.0,
                                "min_range_m": 0.12,
                                "max_range_m": 12.0,
                                "ranges": raw_ranges
                            }
                        }
                    }
                    with data_lock:
                        telemetry_state["latest"] = payload
                        telemetry_state["last_update"] = t

                    connector.set_local_telemetry(payload)
                    connector.relay_telemetry_to_peers()

                    time.sleep(0.1)

            sim_thread = threading.Thread(target=sim_thread_func, daemon=True)
            sim_thread.start()

            try:
                run_http_server()
            except KeyboardInterrupt:
                pass
            finally:
                shutdown_global_connector()


if __name__ == "__main__":
    main()
