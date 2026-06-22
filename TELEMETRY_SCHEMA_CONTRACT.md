# UCMTC Telemetry Schema Contract (Rover → GCS)

This document defines the **canonical telemetry payload contract** sent over ROS2 DDS (as JSON inside `std_msgs/String`) from rover to GCS.

## Canonical schema artifact

- Machine-readable JSON Schema: `telemetry_payload.schema.json`

Use this schema as the source of truth for both publisher and receiver validation.

## Top-level payload shape

The payload MUST be a JSON object with exactly these keys:

- `Navigation`
- `Safety`
- `Vision`
- `Jetson`
- `Communication`
- `ROS`

No extra top-level keys are allowed.

## Type contract

### Navigation
- `speed_kmh`: number
- `heading_deg`: number
- `pos_lat`: number
- `pos_lon`: number
- `dist_traveled_m`: number
- `wp_current`: integer
- `wp_error_m`: number
- `wp_status`: string

### Safety
- `mode`: string
- `light_state`: string
- `estop_mech_armed`: boolean
- `estop_wire_armed`: boolean
- `estop_triggered`: boolean
- `is_blocked`: boolean
- `collision_detected`: boolean
- `border_crossed`: boolean
- `border_partial`: boolean
- `obstacle_touched`: boolean

### Vision
- `img_confidence`: number
- `img_detected`: boolean
- `laser_active`: boolean
- `img_elapsed_sec`: integer
- `img_task_status`: string
- `lane_detected`: boolean
- `obstacles_count`: integer
- `fps_vision`: number

### Jetson
- `cpu_pct`: number
- `gpu_pct`: number
- `ram_pct`: number
- `temp_c`: number
- `bat_pct`: number
- `bat_voltage`: number
- `uptime_sec`: integer

### Communication
- `rtt_ms`: integer
- `channel_rssi`: integer
- `stream_fps`: number
- `packet_loss_pct`: number
- `heartbeat_seq`: integer
- `timestamp_ms`: integer

### ROS
- `node_lane_det`: boolean
- `node_obs_avoid`: boolean
- `node_wp_nav`: boolean
- `node_img_recog`: boolean
- `node_motor_ctrl`: boolean
- `rosout_last`: string

## Operational rules

1. Reject malformed payloads at ingestion (receiver thread), before any UI update.
2. Use strict validation (required keys + exact primitive types).
3. Do not use silent fallback dictionaries (`.get(..., {})`) for required objects.
4. Keep UI rendering decoupled from ingest rate (e.g., 100Hz ingest, 30Hz render timer).
5. Keep video transport separate (WebRTC), and include only telemetry metrics in JSON.

## Notes on JSON correctness

- JSON does **not** support comments (`// ...`).
- Domain blocks like `Navigation`, `Safety`, etc. must be nested objects.
- Example payloads should be validated against `telemetry_payload.schema.json` in CI.
