import os
import sys
import json
import math
import time
import threading
import base64

from web_gcs.server_state import (
    Node, String, COMMAND_QOS, RELIABLE_QOS, ROS_AVAILABLE,
    data_lock, telemetry_state, get_global_connector, topic_config
)

class WebGCSBridgeNode(Node):
    """ROS 2 Node that bridges network topics to the HTTP SSE stream."""

    def __init__(self):
        super().__init__("web_gcs_bridge")
        import web_gcs.server_state as ss
        ss.command_publisher = self.create_publisher(
            String, "/rover/commands/motor", COMMAND_QOS
        )
        try:
            from geometry_msgs.msg import Twist
            ss.cmd_vel_publisher = self.create_publisher(
                Twist, "/cmd_vel", COMMAND_QOS
            )
            self.get_logger().info("/cmd_vel publisher created (geometry_msgs/Twist)")
        except ImportError:
            ss.cmd_vel_publisher = None
            self.get_logger().warning("geometry_msgs not available, /cmd_vel publisher skipped")
        
        self.subscription = self.create_subscription(
            String,
            "/rover/telemetry",
            self.telemetry_callback,
            RELIABLE_QOS,
        )
        self._mission_sub = self.create_subscription(
            String, "/mission_phase", self.mission_callback, RELIABLE_QOS
        )
        try:
            from std_msgs.msg import Bool
            self._arm_sub = self.create_subscription(
                Bool, "/arm_status", self.arm_callback, RELIABLE_QOS
            )
        except Exception:
            self._arm_sub = self.create_subscription(
                String, "/arm_status", self.arm_callback, RELIABLE_QOS
            )
        
        self._speed_limit_sub = None
        try:
            from nav2_msgs.msg import SpeedLimit
            self._speed_limit_sub = self.create_subscription(
                SpeedLimit, "/speed_limit", self.speed_limit_callback, RELIABLE_QOS
            )
        except Exception:
            try:
                from std_msgs.msg import Float32
                self._speed_limit_sub = self.create_subscription(
                    Float32, "/speed_limit", self.speed_limit_callback, RELIABLE_QOS
                )
            except Exception:
                pass
        self.get_logger().info("Web GCS Bridge subscriber/publisher initialized.")

    def telemetry_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            with data_lock:
                old = telemetry_state.get("latest")
                if old and isinstance(old, dict):
                    if "Navigation" in old and isinstance(old["Navigation"], dict):
                        nav = payload.setdefault("Navigation", {})
                        if "mission_phase" in old["Navigation"]:
                            nav["mission_phase"] = old["Navigation"]["mission_phase"]
                        if "speed_limit" in old["Navigation"]:
                            nav["speed_limit"] = old["Navigation"]["speed_limit"]
                    if "Safety" in old and isinstance(old["Safety"], dict):
                        safety = payload.setdefault("Safety", {})
                        if "arm_status" in old["Safety"]:
                            safety["arm_status"] = old["Safety"]["arm_status"]
                
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
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}, "Navigation": {}, "Safety": {}
                    }
                latest = telemetry_state["latest"]
                if "Navigation" not in latest:
                    latest["Navigation"] = {}
                latest["Navigation"]["mission_phase"] = str(msg.data)
                telemetry_state["last_update"] = time.time()
        except Exception:
            pass

    def arm_callback(self, msg):
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}, "Navigation": {}, "Safety": {}
                    }
                latest = telemetry_state["latest"]
                if "Safety" not in latest:
                    latest["Safety"] = {}
                if hasattr(msg, "data"):
                    latest["Safety"]["arm_status"] = str(msg.data)
                else:
                    latest["Safety"]["arm_status"] = str(msg)
                telemetry_state["last_update"] = time.time()
        except Exception:
            pass

    def speed_limit_callback(self, msg):
        try:
            with data_lock:
                if telemetry_state["latest"] is None or not isinstance(telemetry_state["latest"], dict):
                    telemetry_state["latest"] = {
                        "Sensors": {}, "Odom": {}, "CmdVelEcho": {}, "GPS": {}, "Battery": {}, "Jetson": {}, "ROS": {}, "Navigation": {}, "Safety": {}
                    }
                latest = telemetry_state["latest"]
                if "Navigation" not in latest:
                    latest["Navigation"] = {}
                if hasattr(msg, "speed_limit"):
                    latest["Navigation"]["speed_limit"] = float(msg.speed_limit)
                elif hasattr(msg, "data"):
                    latest["Navigation"]["speed_limit"] = float(msg.data)
                telemetry_state["last_update"] = time.time()
        except Exception:
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
    from web_gcs.topic_registry import get_registry

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
            import web_gcs.server_state as ss
            cam_topic = ss.topic_config.get("image_recognition", {}).get("path", "/rgb/image_raw/compressed")
            if cam_topic in registry and registry[cam_topic]["latest_raw"] is not None:
                msg = registry[cam_topic]["latest_raw"]
                last_update = registry[cam_topic]["last_update"]
                arrival_time = registry[cam_topic].get("arrival_time")
                
                if last_update > last_update_time:
                    if last_fps_calc_time is None:
                        if arrival_time:
                            last_fps_calc_time = arrival_time
                        elif Time:
                            last_fps_calc_time = Time(nanoseconds=int(last_update * 1e9))
                        else:
                            last_fps_calc_time = last_update

                    frame_count += 1
                    
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

                    latency_ms = 0.0
                    if arrival_time and Time and hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                        try:
                            msg_generation_time = Time.from_msg(msg.header.stamp)
                            latency_duration = arrival_time - msg_generation_time
                            latency_ms = latency_duration.nanoseconds / 1e6
                        except Exception:
                            pass
                    elif hasattr(msg, 'header') and hasattr(msg.header, 'stamp') and hasattr(msg.header.stamp, 'sec'):
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
                        size_kb = len(msg.data) / 1024.0
                        fps_text = f"FPS: {fps:.1f}"
                        latency_text = f"Latency: {latency_ms:.1f} ms"
                        size_text = f"Size: {size_kb:.1f} KB"
                        
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
                                telemetry_state["camera_frame_raw"] = jpeg_bytes
                                telemetry_state["last_camera_update"] = last_update
                    else:
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
                                        telemetry_state["camera_frame_raw"] = jpeg_bytes
                                        telemetry_state["last_camera_update"] = last_update
                            elif msg.encoding == "jpeg":
                                jpeg_bytes = bytes(msg.data)
                                b64_str = base64.b64encode(jpeg_bytes).decode('utf-8')
                                with data_lock:
                                    telemetry_state["camera_frame"] = b64_str
                                    telemetry_state["camera_frame_raw"] = jpeg_bytes
                                    telemetry_state["last_camera_update"] = last_update
                        else:
                            try:
                                data_bytes = bytes(msg.data)
                            except Exception:
                                data_bytes = msg.data
                            b64_str = base64.b64encode(data_bytes).decode('utf-8')
                            with data_lock:
                                telemetry_state["camera_frame"] = b64_str
                                telemetry_state["camera_frame_raw"] = data_bytes
                                telemetry_state["last_camera_update"] = last_update
                    
                    last_update_time = last_update
        except Exception:
            pass
        time.sleep(0.002)


def websocket_telemetry_broadcast_worker():
    """Polls telemetry_state and broadcasts updates to clients via binary MessagePack WebSockets."""
    import msgpack
    from web_gcs.websocket import socketio
    
    last_sent_time = 0.0
    while True:
        time.sleep(0.05) # 20 Hz
        with data_lock:
            latest = telemetry_state["latest"]
            last_update = telemetry_state["last_update"]
        
        if latest and last_update > last_sent_time:
            try:
                connector = get_global_connector()
                peers_snapshot = connector.get_peers_snapshot()
                sim_mode = (not ROS_AVAILABLE or os.environ.get("GCS_SIM_MODE") == "1" or os.environ.get("GCS_SIM_MODE") == "true" or "--sim" in sys.argv)
                
                payload = {
                    "telemetry": latest,
                    "connected": latest is not None and (time.time() - last_update) < 2.0,
                    "peers": peers_snapshot,
                    "peer_count": connector.peer_count,
                    "peers_connected": connector.connected_count,
                    "simulation_mode": sim_mode,
                }
                binary_payload = msgpack.packb(payload)
                socketio.emit("telemetry_binary_update", binary_payload)
                last_sent_time = last_update
            except Exception:
                pass
