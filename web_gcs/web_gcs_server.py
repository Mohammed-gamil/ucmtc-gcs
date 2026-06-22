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
}
command_publisher = None
sim_estop_triggered = False


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Threaded HTTP server to handle multiple connection streams without blocking."""
    daemon_threads = True


class GCSWebHandler(BaseHTTPRequestHandler):
    """HTTP request handler for GCS Web assets, SSE streams, POST commands, and Global Connector API."""

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
    """ROS 2 Node that bridges network topics to the HTTP SSE stream.

    QoS rationale
    -------------
    - ``/rover/commands/motor`` publisher: COMMAND_QOS (RELIABLE/depth-1) —
      commands must not be dropped and only the latest matters.
    - ``/rover/telemetry`` subscriber: RELIABLE_QOS — must match the
      TelemetryAggregatorNode publisher which uses RELIABLE_QOS.
    """

    def __init__(self):
        super().__init__("web_gcs_bridge")
        global command_publisher
        # Command publisher: RELIABLE — e-stop and drive commands must arrive.
        command_publisher = self.create_publisher(
            String, "/rover/commands/motor", COMMAND_QOS
        )
        # Telemetry subscriber: RELIABLE — matches aggregator RELIABLE_QOS publisher.
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

            # Feed telemetry to the global connector for relay to peers
            connector = get_global_connector()
            connector.set_local_telemetry(payload)
        except Exception as e:
            self.get_logger().warning(f"Failed to parse incoming telemetry: {e}")


def run_http_server():
    server = ThreadedHTTPServer(("0.0.0.0", PORT), GCSWebHandler)
    print(f"[HTTP] Web dashboard serving at http://localhost:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main(args=None):
    # Start Global Connector
    connector = get_global_connector()
    connector.start()

    # Start HTTP server in a daemon thread
    server_thread = threading.Thread(target=run_http_server, daemon=True)
    server_thread.start()

    if ROS_AVAILABLE:
        if not rclpy.ok():
            rclpy.init(args=args)
        node = WebGCSBridgeNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, Exception):
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            shutdown_global_connector()
    else:
        print("[WARN] ROS2 not available. Running in HTTP web-server fallback simulation mode.")
        # Local mock updater for standalone running
        sim_start = time.time()
        import random
        import math
        
        tick = 0
        speed = 0.0
        heading = 0.0
        lat = 40.0
        lon = -75.0
        dist = 0.0
        
        try:
            while True:
                tick += 1
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
                        "bat_pct": round(max(0.0, 100.0 - (tick * 0.005)), 1),
                        "bat_voltage": round(11.8 - (tick * 0.0005), 2),
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
                        "rosout_last": "Local simulation running successfully."
                    }
                }
                
                with data_lock:
                    telemetry_state["latest"] = payload
                    telemetry_state["last_update"] = time.time()

                # Feed to global connector for relay
                connector.set_local_telemetry(payload)
                connector.relay_telemetry_to_peers()

                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            shutdown_global_connector()


if __name__ == "__main__":
    main()
