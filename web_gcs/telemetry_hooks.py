import json
import math
import time
from web_gcs.server_state import (
    data_lock, telemetry_state, sim_estop_triggered, ROS_AVAILABLE,
    cmd_vel_publisher, command_publisher, String, get_global_connector
)
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
            mission_phase = latest.get("Navigation", {}).get("mission_phase")
            speed_limit = latest.get("Navigation", {}).get("speed_limit")
            arm_status = latest.get("Safety", {}).get("arm_status")
            
            latest.update(data)
            
            if mission_phase is not None:
                latest.setdefault("Navigation", {})["mission_phase"] = mission_phase
            if speed_limit is not None:
                latest.setdefault("Navigation", {})["speed_limit"] = speed_limit
            if arm_status is not None:
                latest.setdefault("Safety", {})["arm_status"] = arm_status

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
    print(f"[WS COMMAND] Received: {command}")
    
    global sim_estop_triggered
    import web_gcs.server_state as ss
    if action == "estop":
        ss.sim_estop_triggered = True
    elif action == "resume":
        ss.sim_estop_triggered = False

    if action == "drive":
        try:
            speed = float(command.get("speed_kmh", 0.0))
            heading = float(command.get("heading_deg", 0.0))
            throttle = float(command.get("throttle_pct", 0.0))
        except (TypeError, ValueError) as e:
            print(f"[WS COMMAND ERROR] Parsing error: {e} in {command}")
            raise

        if not (0.0 <= speed <= 15.0):
            print(f"[WS COMMAND ERROR] Speed out of bounds: {speed}")
            raise ValueError("Speed out of bounds [0, 15]")
        if not (0.0 <= heading <= 360.0):
            print(f"[WS COMMAND ERROR] Heading out of bounds: {heading}")
            raise ValueError("Heading out of bounds [0, 360]")
        if not (0.0 <= throttle <= 1.0):
            print(f"[WS COMMAND ERROR] Throttle out of bounds: {throttle}")
            raise ValueError("Throttle out of bounds [0, 1]")

        if ss.cmd_vel_publisher:
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
                ss.cmd_vel_publisher.publish(twist)
            except Exception as e:
                print(f"[WARN] Failed to publish Twist on /cmd_vel: {e}")

    elif action in ("estop", "stop"):
        if ss.cmd_vel_publisher:
            try:
                from geometry_msgs.msg import Twist
                ss.cmd_vel_publisher.publish(Twist())
            except Exception as e:
                pass

    if ss.command_publisher:
        msg = String()
        msg.data = json.dumps(command, separators=(",", ":"))
        ss.command_publisher.publish(msg)

from web_gcs import websocket as ws
ws.DRIVE_COMMAND_HOOK = my_drive_command_hook
