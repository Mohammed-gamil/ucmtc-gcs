"""Shared telemetry helpers for rover nodes.

Also defines canonical QoS profiles used across the rover stack so
every node imports a single source-of-truth rather than bare integers.
"""

from __future__ import annotations

import json
import math
import random
import time
from typing import Any

# ---------------------------------------------------------------------------
# QoS constants — import these instead of using bare integers
# ---------------------------------------------------------------------------
try:
    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        DurabilityPolicy,
        HistoryPolicy,
        Duration,
    )

    # High-frequency sensor / telemetry streams: drop stale messages, never
    # block the publisher waiting for slow subscribers.
    SENSOR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )

    # Safety heartbeat — RELIABLE with 500 ms DEADLINE and 1 s LIFESPAN.
    # DEADLINE: fires a QoS event when no safety message arrives in 500 ms,
    #   letting subscribers take a safe action (e.g. engage e-stop).
    # LIFESPAN: discards safety messages older than 1 s before delivery so
    #   subscribers never act on stale safety state.
    # Use this on BOTH the publisher and the subscriber for the deadline event
    # to fire on both sides.
    SAFETY_HEARTBEAT_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        deadline=Duration(seconds=0, nanoseconds=500_000_000),  # 500 ms
        lifespan=Duration(seconds=1, nanoseconds=0),            # 1 s
    )

    # Drive commands: RELIABLE with LIFESPAN so commands older than 200 ms
    # are discarded before delivery. A stale drive command (e.g. after a
    # network hiccup) must never be acted upon.
    COMMAND_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        lifespan=Duration(seconds=0, nanoseconds=200_000_000),  # 200 ms
    )

    # Diagnostics / heartbeats: reliable delivery, small buffer.
    RELIABLE_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

except ImportError:
    # Fallback when rclpy is not installed (unit tests, local dev).
    SENSOR_QOS = 5             # type: ignore[assignment]
    SAFETY_HEARTBEAT_QOS = 1   # type: ignore[assignment]
    COMMAND_QOS = 1            # type: ignore[assignment]
    RELIABLE_QOS = 10          # type: ignore[assignment]


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def now_ms() -> int:
    return int(time.time() * 1000)


def decode_json_message(message: Any) -> dict[str, Any]:
    """Safely decode a ROS String message or raw string to a dict.

    Returns an empty dict on any error — callers must check for empty
    return rather than catching exceptions.
    """
    if hasattr(message, "data"):
        raw_message = message.data
    elif isinstance(message, str):
        raw_message = message
    elif isinstance(message, dict):
        return dict(message)
    else:
        return {}

    if not raw_message:
        return {}
    try:
        decoded = json.loads(raw_message)
    except json.JSONDecodeError:
        return {}
    return decoded if isinstance(decoded, dict) else {}


def make_navigation_payload(
    speed_kmh: float,
    heading_deg: float,
    pos_lat: float,
    pos_lon: float,
    dist_traveled_m: float,
    wp_current: int,
    wp_error_m: float,
    wp_status: str,
) -> dict[str, Any]:
    return {
        "speed_kmh": round(speed_kmh, 2),
        "heading_deg": round(heading_deg, 1),
        "pos_lat": round(pos_lat, 6),
        "pos_lon": round(pos_lon, 6),
        "dist_traveled_m": round(dist_traveled_m, 1),
        "wp_current": int(wp_current),
        "wp_error_m": round(wp_error_m, 2),
        "wp_status": wp_status,
    }


def make_safety_payload(
    mode: str,
    light_state: str,
    estop_mech_armed: bool,
    estop_wire_armed: bool,
    estop_triggered: bool,
    is_blocked: bool,
    collision_detected: bool,
    border_crossed: bool,
    border_partial: bool,
    obstacle_touched: bool,
) -> dict[str, Any]:
    return {
        "mode": mode,
        "light_state": light_state,
        "estop_mech_armed": estop_mech_armed,
        "estop_wire_armed": estop_wire_armed,
        "estop_triggered": estop_triggered,
        "is_blocked": is_blocked,
        "collision_detected": collision_detected,
        "border_crossed": border_crossed,
        "border_partial": border_partial,
        "obstacle_touched": obstacle_touched,
    }


def make_vision_payload(
    img_confidence: float,
    img_detected: bool,
    laser_active: bool,
    img_elapsed_sec: int,
    img_task_status: str,
    lane_detected: bool,
    obstacles_count: int,
    fps_vision: float,
) -> dict[str, Any]:
    return {
        "img_confidence": round(clamp(img_confidence, 0.0, 1.0), 2),
        "img_detected": img_detected,
        "laser_active": laser_active,
        "img_elapsed_sec": int(img_elapsed_sec),
        "img_task_status": img_task_status,
        "lane_detected": lane_detected,
        "obstacles_count": int(obstacles_count),
        "fps_vision": round(fps_vision, 1),
    }


def make_jetson_payload(
    cpu_pct: float,
    gpu_pct: float,
    ram_pct: float,
    temp_c: float,
    bat_pct: float,
    bat_voltage: float,
    uptime_sec: int,
) -> dict[str, Any]:
    return {
        "cpu_pct": round(clamp(cpu_pct, 0.0, 100.0), 1),
        "gpu_pct": round(clamp(gpu_pct, 0.0, 100.0), 1),
        "ram_pct": round(clamp(ram_pct, 0.0, 100.0), 1),
        "temp_c": round(temp_c, 1),
        "bat_pct": round(clamp(bat_pct, 0.0, 100.0), 1),
        "bat_voltage": round(bat_voltage, 2),
        "uptime_sec": int(uptime_sec),
    }


def make_communication_payload(
    rtt_ms: int,
    channel_rssi: int,
    stream_fps: float,
    packet_loss_pct: float,
    heartbeat_seq: int,
    timestamp_ms: int,
) -> dict[str, Any]:
    return {
        "rtt_ms": int(rtt_ms),
        "channel_rssi": int(channel_rssi),
        "stream_fps": round(max(0.0, stream_fps), 1),
        "packet_loss_pct": round(clamp(packet_loss_pct, 0.0, 100.0), 1),
        "heartbeat_seq": int(heartbeat_seq),
        "timestamp_ms": int(timestamp_ms),
    }


def make_ros_payload(
    node_lane_det: bool,
    node_obs_avoid: bool,
    node_wp_nav: bool,
    node_img_recog: bool,
    node_motor_ctrl: bool,
    rosout_last: str,
    esp32_connected: bool = False,
) -> dict[str, Any]:
    return {
        "node_lane_det": node_lane_det,
        "node_obs_avoid": node_obs_avoid,
        "node_wp_nav": node_wp_nav,
        "node_img_recog": node_img_recog,
        "node_motor_ctrl": node_motor_ctrl,
        "esp32_connected": esp32_connected,
        "rosout_last": rosout_last,
    }


def random_navigation_defaults() -> dict[str, Any]:
    return {
        "speed_kmh": 0.0,
        "heading_deg": 0.0,
        "pos_lat": 40.0 + random.uniform(-0.001, 0.001),
        "pos_lon": -75.0 + random.uniform(-0.001, 0.001),
        "dist_traveled_m": 0.0,
        "wp_current": 0,
        "wp_error_m": 0.0,
        "wp_status": "idle",
    }


# ---------------------------------------------------------------------------
# Standard ROS 2 message-type imports (optional — file stays importable without ROS)
# ---------------------------------------------------------------------------
try:
    from geometry_msgs.msg import Twist                   # /cmd_vel
    from sensor_msgs.msg import (
        LaserScan,                                        # /scan
        Imu,                                              # /imu
        BatteryState,                                     # /battery_state
        NavSatFix,                                        # /fix (GPS)
    )
    from nav_msgs.msg import Odometry                     # /odom
    from tf2_msgs.msg import TFMessage                    # /tf, /tf_static
    from rcl_interfaces.msg import Log                    # /rosout
    _STD_MSGS_AVAILABLE = True
except ImportError:
    Twist = None          # type: ignore[misc,assignment]
    LaserScan = None      # type: ignore[misc,assignment]
    Imu = None            # type: ignore[misc,assignment]
    BatteryState = None   # type: ignore[misc,assignment]
    NavSatFix = None      # type: ignore[misc,assignment]
    Odometry = None       # type: ignore[misc,assignment]
    TFMessage = None      # type: ignore[misc,assignment]
    Log = None            # type: ignore[misc,assignment]
    _STD_MSGS_AVAILABLE = False


# ---------------------------------------------------------------------------
# Standard-topic payload builders — convert native ROS messages → GCS JSON dicts
# ---------------------------------------------------------------------------

def make_imu_payload(msg: Any) -> "dict[str, Any]":
    """Convert a sensor_msgs/Imu message to a flat GCS-friendly dict.

    Angular velocity in rad/s; linear acceleration in m/s² (includes gravity per REP-103).
    Orientation as (x, y, z, w) quaternion.  covariance[0] == -1.0 signals «not provided».
    """
    if msg is None:
        return {"available": False}
    return {
        "available": True,
        "frame_id": msg.header.frame_id,
        "orientation_x": round(msg.orientation.x, 6),
        "orientation_y": round(msg.orientation.y, 6),
        "orientation_z": round(msg.orientation.z, 6),
        "orientation_w": round(msg.orientation.w, 6),
        "orientation_cov_ok": msg.orientation_covariance[0] >= 0.0,
        "angular_velocity_x": round(msg.angular_velocity.x, 6),    # rad/s
        "angular_velocity_y": round(msg.angular_velocity.y, 6),
        "angular_velocity_z": round(msg.angular_velocity.z, 6),
        "angular_velocity_cov_ok": msg.angular_velocity_covariance[0] >= 0.0,
        "linear_acceleration_x": round(msg.linear_acceleration.x, 4),  # m/s² incl. gravity
        "linear_acceleration_y": round(msg.linear_acceleration.y, 4),
        "linear_acceleration_z": round(msg.linear_acceleration.z, 4),
        "linear_acceleration_cov_ok": msg.linear_acceleration_covariance[0] >= 0.0,
        # Direct fallback keys to match GCS direct callbacks
        "accel_x": round(msg.linear_acceleration.x, 4),
        "accel_y": round(msg.linear_acceleration.y, 4),
        "accel_z": round(msg.linear_acceleration.z, 4),
        "gyro_x": round(msg.angular_velocity.x, 6),
        "gyro_y": round(msg.angular_velocity.y, 6),
        "gyro_z": round(msg.angular_velocity.z, 6),
    }


def make_scan_payload(msg: Any) -> "dict[str, Any]":
    """Convert a sensor_msgs/LaserScan to a GCS-friendly summary dict.

    Full ranges array is summarised (forward distance, valid count, min/max)
    to keep the SSE payload small.  Invalid rays use Infinity / NaN — never 0.
    """
    if msg is None:
        return {"available": False}
    ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
    fwd_idx = (
        int((-msg.angle_min) / msg.angle_increment)
        if msg.angle_increment > 0 else 0
    )
    fwd_idx = max(0, min(fwd_idx, len(msg.ranges) - 1))
    fwd_range = msg.ranges[fwd_idx] if msg.ranges else float("inf")

    raw_ranges = msg.ranges
    n = len(raw_ranges)
    max_points = 180
    if n > max_points:
        step = max(1, n // max_points)
        downsampled = [raw_ranges[i] for i in range(0, n, step)][:max_points]
    else:
        downsampled = raw_ranges

    formatted_ranges = [
        round(r, 2) if (msg.range_min <= r <= msg.range_max and math.isfinite(r)) else None
        for r in downsampled
    ]

    return {
        "available": True,
        "frame_id": msg.header.frame_id,
        "angle_min_rad": round(msg.angle_min, 4),
        "angle_max_rad": round(msg.angle_max, 4),
        "range_min_m": round(msg.range_min, 3),
        "range_max_m": round(msg.range_max, 3),
        "num_points": len(msg.ranges),
        "num_valid": len(ranges),
        "forward_range_m": round(fwd_range, 3) if fwd_range != float("inf") else None,
        "min_range_m": round(min(ranges), 3) if ranges else None,
        "max_range_m": round(max(ranges), 3) if ranges else None,
        "ranges": formatted_ranges,
    }


def make_gps_payload(msg: Any) -> "dict[str, Any]":
    """Convert a sensor_msgs/NavSatFix to a GCS-friendly dict.

    Latitude/longitude in decimal degrees; altitude in meters (WGS-84 ellipsoid).
    fix_status: -1=no fix, 0=fix, 1=SBAS, 2=GBAS.
    """
    if msg is None:
        return {"available": False}
    status = int(msg.status.status) if hasattr(msg, "status") else -1
    cov = list(msg.position_covariance) if len(msg.position_covariance) > 0 else []
    return {
        "available": True,
        "frame_id": msg.header.frame_id,
        "latitude": round(msg.latitude, 8),     # decimal degrees
        "longitude": round(msg.longitude, 8),
        "altitude_m": round(msg.altitude, 3),   # meters above WGS-84 ellipsoid
        "altitude": round(msg.altitude, 3),      # Direct fallback key
        "fix_status": status,                   # -1=no fix, 0=fix, 1=sbas, 2=gbas
        "has_fix": status >= 0,
        "position_covariance_type": int(msg.position_covariance_type),
        # Diagonal: lat_var, lon_var, alt_var (row-major 3×3 at indices 0,4,8)
        "position_cov_lat": round(cov[0], 6) if len(cov) > 0 else None,
        "position_cov_lon": round(cov[4], 6) if len(cov) > 4 else None,
        "position_cov_alt": round(cov[8], 6) if len(cov) > 8 else None,
    }


def make_odom_payload(msg: Any) -> "dict[str, Any]":
    """Convert a nav_msgs/Odometry to a GCS-friendly dict.

    pose.pose is robot pose in header.frame_id (usually 'odom').
    twist.twist is body-frame velocity (linear.x = forward speed m/s).
    """
    if msg is None:
        return {"available": False}
    p = msg.pose.pose
    t = msg.twist.twist
    import math
    vx = t.linear.x
    vy = t.linear.y
    speed_kmh = math.sqrt(vx*vx + vy*vy) * 3.6
    return {
        "available": True,
        "frame_id": msg.header.frame_id,           # fixed frame, usually 'odom'
        "child_frame_id": msg.child_frame_id,       # body frame, usually 'base_link'
        "pos_x": round(p.position.x, 4),           # meters
        "pos_y": round(p.position.y, 4),
        "pos_z": round(p.position.z, 4),
        "orient_x": round(p.orientation.x, 6),
        "orient_y": round(p.orientation.y, 6),
        "orient_z": round(p.orientation.z, 6),
        "orient_w": round(p.orientation.w, 6),
        "linear_x": round(t.linear.x, 4),          # m/s forward (body frame)
        "linear_y": round(t.linear.y, 4),
        "linear_z": round(t.linear.z, 4),
        "angular_z": round(t.angular.z, 6),         # rad/s yaw-rate (body frame)
        # Direct fallback keys to match GCS direct callbacks
        "ori_x": round(p.orientation.x, 6),
        "ori_y": round(p.orientation.y, 6),
        "ori_z": round(p.orientation.z, 6),
        "ori_w": round(p.orientation.w, 6),
        "speed_kmh": round(speed_kmh, 2),
        "twist_linear_x": round(vx, 4),
        "twist_angular_z": round(t.angular.z, 6),
    }


def make_battery_payload(msg: Any) -> "dict[str, Any]":
    """Convert a sensor_msgs/BatteryState to a GCS-friendly dict.

    voltage in Volts; current in Amperes (positive = charging); percentage 0–100%.
    NaN fields are reported as None.
    """
    if msg is None:
        return {"available": False}

    def _safe(v: float) -> "float | None":
        return round(v, 3) if v == v else None  # NaN check

    pct = _safe(msg.percentage)
    return {
        "available": True,
        "voltage_v": _safe(msg.voltage),
        "voltage": _safe(msg.voltage),           # Direct fallback key
        "current_a": _safe(msg.current),
        "charge_ah": _safe(msg.charge),
        "capacity_ah": _safe(msg.capacity),
        "percentage": round(pct * 100.0, 1) if pct is not None else None,  # → 0–100%
        "power_supply_status": int(msg.power_supply_status),
        # 0=UNKNOWN,1=CHARGING,2=DISCHARGING,3=NOT_CHARGING,4=FULL
        "power_supply_health": int(msg.power_supply_health),
        "present": bool(msg.present),
    }


def make_tf_summary_payload(msg: Any) -> "dict[str, Any]":
    """Return a compact summary of a TFMessage for the GCS dashboard."""
    if msg is None:
        return {"available": False, "transforms": []}
    transforms = [
        {"parent": tf.header.frame_id, "child": tf.child_frame_id}
        for tf in msg.transforms
    ]
    return {"available": True, "count": len(transforms), "transforms": transforms}


def make_rosout_payload(msg: Any) -> "dict[str, Any]":
    """Convert the latest rcl_interfaces/Log entry to a GCS-friendly dict."""
    if msg is None:
        return {"available": False, "msg": "", "level": 0, "name": ""}
    return {
        "available": True,
        "level": int(msg.level),   # 10=DEBUG,20=INFO,30=WARN,40=ERROR,50=FATAL
        "name": msg.name,
        "msg": msg.msg,
        "file": msg.file,
        "function": msg.function,
        "line": int(msg.line),
    }


def get_cmd_vel_topic_name() -> str:
    """Resolve the cmd_vel topic name from topic_config.json, falling back to '/cmd_vel'."""
    import os
    import json

    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.normpath(os.path.join(base_dir, "../../../../web_gcs/topic_config.json"))
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = json.load(f)
                if "cmd_vel_echo" in config and "path" in config["cmd_vel_echo"]:
                    path = config["cmd_vel_echo"]["path"]
                    if path:
                        return path
    except Exception:
        pass
    return "/cmd_vel"


def get_topic_path(topic_key: str, default_path: str) -> str:
    """Resolve the topic path from topic_config.json, falling back to default_path."""
    import os
    import json

    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.normpath(os.path.join(base_dir, "../../../../web_gcs/topic_config.json"))
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

