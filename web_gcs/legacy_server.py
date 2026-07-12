import json
import os
import time
import math
import subprocess
import shutil
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

from web_gcs.server_state import (
    STATIC_DIR, PORT, data_lock, telemetry_state,
    command_publisher, cmd_vel_publisher, _get_bridge_node,
    topic_config, DEFAULT_TOPICS, TOPIC_CONFIG_FILE,
    get_global_connector
)

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
        import web_gcs.server_state as ss
        if self.path == "/api/config/topics":
            self._send_json(200, json.dumps(ss.topic_config))
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
                except Exception:
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
                    
                    time.sleep(0.005)
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
        import web_gcs.server_state as ss
        if self.path == "/api/config/topics":
            content_length = int(self.headers.get("Content-Length", 0))
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode("utf-8"))
                for k, v in DEFAULT_TOPICS.items():
                    if k in data and isinstance(data[k], dict):
                        ss.topic_config[k] = {
                            "label": str(data[k].get("label", v["label"])),
                            "path": str(data[k].get("path", v["path"]))
                        }
                ss.save_topic_config()
                node = _get_bridge_node()
                if node is not None and hasattr(node, "update_topics"):
                    node.update_topics()
                self._send_json(200, json.dumps({"status": "success", "message": "Topic configuration saved & updated"}))
            except Exception as exc:
                self._send_json(400, json.dumps({"status": "error", "message": str(exc)}))
            return

        if self.path == "/api/config/topics/reset":
            try:
                ss.topic_config.clear()
                ss.topic_config.update(DEFAULT_TOPICS)
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
                if action == "estop":
                    ss.sim_estop_triggered = True
                elif action == "resume":
                    ss.sim_estop_triggered = False

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
                if ss.command_publisher:
                    msg = ss.String()
                    msg.data = json.dumps(command, separators=(",", ":"))
                    ss.command_publisher.publish(msg)

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
                ros_domain_id = int(peer_data.get("ros_domain_id", 32))

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
                # Helper to spawn terminal
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
                cmd = ["/home/medochi/GS/.venv/bin/python", "-m", "unittest", "tests/test_gcs_rover.py"]
                res = subprocess.run(cmd, capture_output=True, text=True, cwd="/home/medochi/GS")
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
                cmd = ["/home/medochi/GS/.venv/bin/python", "-m", "unittest", "tests.test_gcs_rover.TestWebGCSUI"]
                res = subprocess.run(cmd, capture_output=True, text=True, cwd="/home/medochi/GS")
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
                try:
                    # Resolves host / checks if IP format is valid
                    socket.gethostbyname(host)
                except Exception:
                    raise ValueError(f"Invalid hostname or IP address format: '{host}'")

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
