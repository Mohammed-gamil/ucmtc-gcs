
"""Strict telemetry dataclasses and validation for the rover payload."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _ensure_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must be an object")
    return value


def _coerce_field(section: str, field_name: str, value: Any, expected_type: str) -> Any:
    field_path = f"{section}.{field_name}"
    if expected_type == "number":
        if not _is_number(value):
            raise ValueError(f"{field_path} must be a number")
        return float(value)
    if expected_type == "integer":
        if not _is_integer(value):
            raise ValueError(f"{field_path} must be an integer")
        return int(value)
    if expected_type == "boolean":
        if not isinstance(value, bool):
            raise ValueError(f"{field_path} must be a boolean")
        return value
    if expected_type == "string":
        if not isinstance(value, str):
            raise ValueError(f"{field_path} must be a string")
        return value
    raise ValueError(f"Unsupported expected type for {field_path}: {expected_type}")


def _validate_section(section: str, value: Any, schema: dict[str, str]) -> dict[str, Any]:
    section_data = _ensure_mapping(value, section)
    expected_keys = set(schema)
    actual_keys = set(section_data)
    if actual_keys != expected_keys:
        missing = sorted(expected_keys - actual_keys)
        extra = sorted(actual_keys - expected_keys)
        problems: list[str] = []
        if missing:
            problems.append(f"missing keys: {', '.join(missing)}")
        if extra:
            problems.append(f"unexpected keys: {', '.join(extra)}")
        raise ValueError(f"{section} has invalid keys ({'; '.join(problems)})")

    return {
        field_name: _coerce_field(section, field_name, section_data[field_name], field_type)
        for field_name, field_type in schema.items()
    }


_NAVIGATION_SCHEMA = {
    "speed_kmh": "number",
    "heading_deg": "number",
    "pos_lat": "number",
    "pos_lon": "number",
    "dist_traveled_m": "number",
    "wp_current": "integer",
    "wp_error_m": "number",
    "wp_status": "string",
}

_SAFETY_SCHEMA = {
    "mode": "string",
    "light_state": "string",
    "estop_mech_armed": "boolean",
    "estop_wire_armed": "boolean",
    "estop_triggered": "boolean",
    "is_blocked": "boolean",
    "collision_detected": "boolean",
    "border_crossed": "boolean",
    "border_partial": "boolean",
    "obstacle_touched": "boolean",
}

_VISION_SCHEMA = {
    "img_confidence": "number",
    "img_detected": "boolean",
    "laser_active": "boolean",
    "img_elapsed_sec": "integer",
    "img_task_status": "string",
    "lane_detected": "boolean",
    "obstacles_count": "integer",
    "fps_vision": "number",
}

_JETSON_SCHEMA = {
    "cpu_pct": "number",
    "gpu_pct": "number",
    "ram_pct": "number",
    "temp_c": "number",
    "bat_pct": "number",
    "bat_voltage": "number",
    "uptime_sec": "integer",
}

_COMMUNICATION_SCHEMA = {
    "rtt_ms": "integer",
    "channel_rssi": "integer",
    "stream_fps": "number",
    "packet_loss_pct": "number",
    "heartbeat_seq": "integer",
    "timestamp_ms": "integer",
}

_ROS_SCHEMA = {
    "node_lane_det": "boolean",
    "node_obs_avoid": "boolean",
    "node_wp_nav": "boolean",
    "node_img_recog": "boolean",
    "node_motor_ctrl": "boolean",
    "esp32_connected": "boolean",
    "rosout_last": "string",
}


@dataclass
class NavigationData:
    """Navigation section contract for rover kinematics and waypoint state at 100Hz."""

    speed_kmh: float
    heading_deg: float
    pos_lat: float
    pos_lon: float
    dist_traveled_m: float
    wp_current: int
    wp_error_m: float
    wp_status: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SafetyData:
    """Safety section contract for e-stop, collision, and boundary conditions consumed by the UI alert layer."""

    mode: str
    light_state: str
    estop_mech_armed: bool
    estop_wire_armed: bool
    estop_triggered: bool
    is_blocked: bool
    collision_detected: bool
    border_crossed: bool
    border_partial: bool
    obstacle_touched: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class VisionData:
    """Vision section contract for confidence, detection state, and perception timing/throughput metrics."""

    img_confidence: float
    img_detected: bool
    laser_active: bool
    img_elapsed_sec: int
    img_task_status: str
    lane_detected: bool
    obstacles_count: int
    fps_vision: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class JetsonData:
    """Jetson section contract for onboard compute health (CPU/GPU/RAM/temperature/battery/uptime)."""

    cpu_pct: float
    gpu_pct: float
    ram_pct: float
    temp_c: float
    bat_pct: float
    bat_voltage: float
    uptime_sec: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class CommunicationData:
    """Communication section contract for link quality, RTT, packet loss, stream FPS, and heartbeat timestamping."""

    rtt_ms: int
    channel_rssi: int
    stream_fps: float
    packet_loss_pct: float
    heartbeat_seq: int
    timestamp_ms: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class RosData:
    """ROS section contract for distributed node liveliness booleans and latest rosout status text."""

    node_lane_det: bool
    node_obs_avoid: bool
    node_wp_nav: bool
    node_img_recog: bool
    node_motor_ctrl: bool
    esp32_connected: bool
    rosout_last: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TelemetryPayload:
    """Master payload contract aggregating all telemetry domains consumed by the PyQt6 GCS render layer."""

    navigation: NavigationData
    safety: SafetyData
    vision: VisionData
    jetson: JetsonData
    communication: CommunicationData
    ros: RosData

    def to_dict(self) -> dict[str, Any]:
        return {
            "Navigation": self.navigation.to_dict(),
            "Safety": self.safety.to_dict(),
            "Vision": self.vision.to_dict(),
            "Jetson": self.jetson.to_dict(),
            "Communication": self.communication.to_dict(),
            "ROS": self.ros.to_dict(),
        }

    @classmethod
    def empty(cls) -> "TelemetryPayload":
        return cls(
            navigation=NavigationData(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, "idle"),
            safety=SafetyData("idle", "idle", False, False, False, False, False, False, False, False),
            vision=VisionData(0.0, False, False, 0, "idle", False, 0, 0.0),
            jetson=JetsonData(0.0, 0.0, 0.0, 0.0, 100.0, 12.0, 0),
            communication=CommunicationData(0, 0, 0.0, 0.0, 0, 0),
            ros=RosData(False, False, False, False, False, False, ""),
        )

    @classmethod
    def from_dict(cls, data: dict) -> "TelemetryPayload":
        """
        Build a TelemetryPayload from raw JSON-decoded data.

        Contract:
        - Receives a nested dictionary decoded from `std_msgs/String` JSON payloads on `/rover/telemetry`.
        - Performs strict schema validation for all required top-level domains and nested keys.
        - Enforces expected scalar types (float/int/bool/str) before data can cross into UI-facing code.
        - Raises `ValueError` when fields are missing or types are invalid to prevent latent UI crashes,
          formatting exceptions, and unsafe fallbacks in the Qt event loop.
        """
        payload = _ensure_mapping(data, "telemetry payload")

        required_top_level = {"Navigation", "Safety", "Vision", "Jetson", "Communication", "ROS"}
        allowed_optional = {"Sensors", "GPS", "Odom", "Battery", "CmdVelEcho", "TF", "Rosout"}
        actual_top_level = set(payload)
        
        missing = sorted(required_top_level - actual_top_level)
        extra = sorted(actual_top_level - required_top_level - allowed_optional)
        if missing or extra:
            problems: list[str] = []
            if missing:
                problems.append(f"missing keys: {', '.join(missing)}")
            if extra:
                problems.append(f"unexpected keys: {', '.join(extra)}")
            raise ValueError(f"telemetry payload has invalid keys ({'; '.join(problems)})")

        navigation = NavigationData(**_validate_section("Navigation", payload["Navigation"], _NAVIGATION_SCHEMA))
        safety = SafetyData(**_validate_section("Safety", payload["Safety"], _SAFETY_SCHEMA))
        vision = VisionData(**_validate_section("Vision", payload["Vision"], _VISION_SCHEMA))
        jetson = JetsonData(**_validate_section("Jetson", payload["Jetson"], _JETSON_SCHEMA))
        communication = CommunicationData(
            **_validate_section("Communication", payload["Communication"], _COMMUNICATION_SCHEMA)
        )
        ros = RosData(**_validate_section("ROS", payload["ROS"], _ROS_SCHEMA))

        return cls(
            navigation=navigation,
            safety=safety,
            vision=vision,
            jetson=jetson,
            communication=communication,
            ros=ros,
        )


__all__ = [
    "CommunicationData",
    "JetsonData",
    "NavigationData",
    "RosData",
    "SafetyData",
    "TelemetryPayload",
    "VisionData",
]

