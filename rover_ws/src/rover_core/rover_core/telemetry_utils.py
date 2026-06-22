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
) -> dict[str, Any]:
    return {
        "node_lane_det": node_lane_det,
        "node_obs_avoid": node_obs_avoid,
        "node_wp_nav": node_wp_nav,
        "node_img_recog": node_img_recog,
        "node_motor_ctrl": node_motor_ctrl,
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
