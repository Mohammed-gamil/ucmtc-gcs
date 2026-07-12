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
from typing import Any

# Ensure correct path loading before any relative/package imports
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)

# Import modular server state and configurations
from web_gcs.server_state import (
    ROVER_SRC_DIR, STATIC_DIR, PORT,
    data_lock, telemetry_state, sim_estop_triggered,
    command_publisher, cmd_vel_publisher, ROS_AVAILABLE,
    DEFAULT_TOPICS, TOPIC_CONFIG_FILE, NETWORK_CONFIG_FILE,
    load_topic_config, save_topic_config, load_network_config,
    get_global_connector, shutdown_global_connector, _get_bridge_node,
    COMMAND_QOS, RELIABLE_QOS, Node, String, rclpy
)

# Preserving legacy classes for unit test compatibility
from web_gcs.legacy_server import ThreadedHTTPServer, GCSWebHandler

# Import ROS 2 bridge and worker threads
from web_gcs.bridge_node import (
    WebGCSBridgeNode, camera_frame_process_worker, websocket_telemetry_broadcast_worker
)

# Import hooks to register them in topic registry and websockets
import web_gcs.telemetry_hooks as hooks
from web_gcs.telemetry_hooks import my_drive_command_hook, my_telemetry_update_hook

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
            if last_update > last_sent_time or time.time() - last_sent_time > 1.0:
                connector = get_global_connector()
                peers_snapshot = connector.get_peers_snapshot()

                payload = json.dumps({
                    "telemetry": latest,
                    "connected": latest is not None and (time.time() - last_update) < 2.0,
                    "peers": peers_snapshot,
                    "peer_count": connector.peer_count,
                    "peers_connected": connector.connected_count,
                    "camera_frame": None,
                })
                yield f"data: {payload}\n\n"
                last_sent_time = time.time()
            time.sleep(0.05)

    return Response(event_generator(), mimetype="text/event-stream")


@app.route("/api/topics/img/stream")
@app.route("/api/topics/<path:topic_path>/stream")
def mjpeg_stream(topic_path=None):
    def gen():
        last_frame_time = 0.0
        while True:
            time.sleep(0.01)
            with data_lock:
                jpeg_bytes = telemetry_state.get("camera_frame_raw")
                last_update = telemetry_state.get("last_camera_update", 0.0)
            
            if jpeg_bytes and last_update > last_frame_time:
                try:
                    headers = f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg_bytes)}\r\n\r\n'.encode('utf-8')
                    yield headers + jpeg_bytes + b'\r\n'
                    last_frame_time = last_update
                except Exception:
                    pass
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


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
    import web_gcs.server_state as ss
    return jsonify(ss.topic_config)


@app.route("/api/config/topics", methods=["POST"])
def post_api_config_topics():
    try:
        import web_gcs.server_state as ss
        data = request.get_json(force=True)
        for k, v in DEFAULT_TOPICS.items():
            if k in data and isinstance(data[k], dict):
                ss.topic_config[k] = {
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
        import web_gcs.server_state as ss
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

    # Start background telemetry binary WebSocket broadcast worker thread
    t_tel = threading.Thread(target=websocket_telemetry_broadcast_worker, daemon=True)
    t_tel.start()

    # Initialize queue for bridge -> websockets hand-off
    out_queue = queue.Queue()
    start_websocket_worker(out_queue)

    if ROS_AVAILABLE:
        if not rclpy.ok():
            rclpy.init(args=args)
        
        import web_gcs.server_state as ss
        ss._bridge_node = WebGCSBridgeNode()
        
        # Start the metadata-driven RosBridge subscriber worker thread
        from web_gcs.ros_subscriber import RosBridge
        ros_bridge = RosBridge(ss._bridge_node, out_queue)
        ros_bridge.start()
        
    # Run Flask server
    run_http_server()
    
    # Cleanup on exit
    if ROS_AVAILABLE:
        try:
            import web_gcs.server_state as ss
            if ss._bridge_node:
                ss._bridge_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    shutdown_global_connector()


# ─── Metadata-driven testing fallback triggers ───
if __name__ == "__main__":
    main()


# ─── Metaprogramming Proxy redirection to preserve test suite compatibility ───
class ModuleProxy(object):
    def __init__(self, module):
        self.__dict__['_module'] = module

    def __getattr__(self, name):
        import web_gcs.server_state as ss
        import web_gcs.legacy_server as ls
        import web_gcs.bridge_node as bn
        if name in ('sim_estop_triggered', 'command_publisher', 'cmd_vel_publisher', '_bridge_node', 'topic_config', 'telemetry_state', 'data_lock', 'PORT', 'STATIC_DIR', 'DEFAULT_TOPICS', 'load_topic_config', 'load_network_config'):
            return getattr(ss, name)
        elif name in ('ThreadedHTTPServer', 'GCSWebHandler'):
            return getattr(ls, name)
        elif name in ('WebGCSBridgeNode', 'camera_frame_process_worker'):
            return getattr(bn, name)
        return getattr(self._module, name)

    def __setattr__(self, name, value):
        import web_gcs.server_state as ss
        if name in ('sim_estop_triggered', 'command_publisher', 'cmd_vel_publisher', '_bridge_node', 'topic_config', 'telemetry_state'):
            setattr(ss, name, value)
        else:
            self._module.__dict__[name] = value

sys.modules[__name__] = ModuleProxy(sys.modules[__name__])
