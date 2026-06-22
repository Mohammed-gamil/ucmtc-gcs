# UCMTC Rover ROS2 Workspace - Complete Structure Summary

## Project Overview
This is the hardware payload workspace (`rover_ws`) for the UCMTC rover running on a Jetson Nano.
It now contains navigation, safety, vision, motor control, and telemetry aggregation nodes.
Together they ingest hardware data, process it, and publish the canonical JSON telemetry payload
over DDS to a remote Ground Control Station on the same WiFi network.

---

## Directory Structure

```
rover_ws/
└── src/
    └── rover_core/                    # Main ROS2 package
        ├── launch/
        │   └── rover_bringup.launch.py
        ├── config/
        │   └── mediamtx.yml
        ├── rover_core/                # Python module
        │   ├── __init__.py
        │   ├── navigation_node.py
        │   ├── safety_node.py
        │   ├── vision_node.py
        │   ├── motor_control_node.py
        │   ├── telemetry_aggregator.py
        │   ├── ros_compat.py
        │   └── telemetry_utils.py
        ├── resource/
        │   └── rover_core             # ROS2 resource marker
        ├── package.xml
        └── setup.py
```

---

## Core Node Specifications

### 1. Navigation Node (navigation_node.py)
**Purpose**: GPS/IMU positioning and movement telemetry

**Published Topics**:
- `/rover/telemetry/nav` - Navigation telemetry

**Key Methods**:
- `__init__()` - Initialize publisher, serial connections (GPS/IMU), and 10 Hz timer
- `read_sensors()` - Read GPS (NMEA) and IMU (accel/gyro/mag) from serial ports
- `publish_nav_data()` - Format and publish telemetry with lat/lon, heading, speed, altitude
- `timer_callback()` - 10 Hz orchestrator

**Published Payload Structure**:
```json
{
  "timestamp": <float>,
  "latitude": <float>,
  "longitude": <float>,
  "heading": <float>,
  "speed": <float>,
  "altitude": <float>
}
```

---

### 2. Safety Node (safety_node.py)
**Purpose**: Emergency stop, collision detection, and motor override

**Published Topics**:
- `/rover/telemetry/safety` - Safety state and flags

**Subscribed Topics**:
- `/rover/commands/motor` - Motor control commands (can be overridden if unsafe)

**Key Methods**:
- `__init__()` - Initialize publisher, motor subscriber, GPIO interface, 20 Hz timer
- `check_estop_hardware()` - Poll GPIO for e-stop button, wire breaks, collision sensors
- `publish_safety_state()` - Aggregate safety data and publish state
- `motor_command_callback()` - Subscribe to motor commands; can block unsafe commands
- `timer_callback()` - 20 Hz safety-critical orchestrator

**Published Payload Structure**:
```json
{
  "timestamp": <float>,
  "estop_active": <bool>,
  "collision_detected": <bool>,
  "motor_override": <bool>,
  "safety_state": <"SAFE" | "ESTOP" | "COLLISION">
}
```

---

### 3. Vision Node (vision_node.py)
**Purpose**: Camera capture, AI inference (lane detection, obstacle detection), and vision metrics

**Published Topics**:
- `/rover/telemetry/vision` - Vision processing metrics

**Subscribed Topics**:
- `/rover/camera/image_raw` (optional - alternative to direct cv2.VideoCapture)

**Key Methods**:
- `__init__()` - Initialize publisher, camera interface (cv2 or ROS image), AI model loader, ~30 Hz timer
- `process_frame()` - Capture frame, preprocess, run AI model, calculate confidence and metrics
- `publish_vision_metrics()` - Format and publish vision telemetry
- `camera_image_callback()` - Optional ROS image topic subscriber callback
- `timer_callback()` - ~30 Hz (0.033s) frame processing orchestrator

**Published Payload Structure**:
```json
{
  "timestamp": <float>,
  "frame_id": <int>,
  "confidence": <float>,
  "lanes_detected": <int>,
  "obstacle_distance": <float>,
  "processing_time_ms": <float>
}
```

### 4. Motor Control Node (motor_control_node.py)
**Purpose**: Accept rover drive commands and publish a small heartbeat/control status payload

**Subscribed Topics**:
- `/rover/commands/motor` - Motor control commands from the GCS

**Published Topics**:
- `/rover/telemetry/control` - Motor control heartbeat and last command snapshot

**Published Payload Structure**:
```json
{
  "node_motor_ctrl": <bool>,
  "heartbeat_seq": <int>,
  "estop_latched": <bool>,
  "last_command": <object>,
  "uptime_sec": <int>,
  "timestamp_ms": <int>
}
```

### 5. Telemetry Aggregator (telemetry_aggregator.py)
**Purpose**: Merge the rover section topics into the canonical payload consumed by the GCS

**Subscribed Topics**:
- `/rover/telemetry/nav`
- `/rover/telemetry/safety`
- `/rover/telemetry/vision`
- `/rover/telemetry/control`

**Published Topics**:
- `/rover/telemetry` - Canonical combined telemetry payload for the WiFi GCS link

**Published Payload Structure**:
```json
{
  "Navigation": { ... },
  "Safety": { ... },
  "Vision": { ... },
  "Jetson": { ... },
  "Communication": { ... },
  "ROS": { ... }
}
```

---

## Build System Files

### setup.py
- Standard Python setuptools configuration
- Defines package metadata and console script entry points for all rover nodes
- Maps console script entry points:
  - `navigation_node` → `rover_core.navigation_node:main`
  - `safety_node` → `rover_core.safety_node:main`
  - `vision_node` → `rover_core.vision_node:main`
  - `motor_control_node` → `rover_core.motor_control_node:main`
  - `telemetry_aggregator` → `rover_core.telemetry_aggregator:main`

### package.xml
- ROS2 package metadata (format 3)
- Declares dependencies: rclpy, sensor_msgs, std_msgs, geometry_msgs, launch, launch_ros
- License: MIT

### rover_bringup.launch.py
- ROS2 Python launch file
- `generate_launch_description()` function
- Instantiates navigation, safety, vision, motor control, and telemetry aggregator nodes with namespace 'rover'
- Ready for parameter configuration and topic remapping

---

## Configuration Files

### mediamtx.yml
- MediaMTX server configuration for video streaming
- RTMP port: 1935
- HLS port: 8080
- HTTP API port: 9997
- Configured for low-latency rover camera streaming

---

## Implementation Notes

### No Internal Logic
The original scaffold has been replaced with working simulation-friendly logic. Hardware-specific
sensor and inference hooks can still be expanded as rover sensors and cameras are attached.

### Extensive Docstrings
Each function includes comprehensive docstrings explaining:
- What it is responsible for
- What inputs/sensors it reads
- What data structures it returns
- What it publishes to DDS
- Exact topic names and payload formats

### Ready for ROS2 Build
To build this workspace:
```bash
cd rover_ws
colcon build --packages-select rover_core
```

To run individual nodes:
```bash
ros2 run rover_core navigation_node
ros2 run rover_core safety_node
ros2 run rover_core vision_node
ros2 run rover_core motor_control_node
ros2 run rover_core telemetry_aggregator
```

To launch the rover stack together:
```bash
ros2 launch rover_core rover_bringup.launch.py
```

---

## Total File Count
- **Python files**: 10 (5 rover nodes + 2 shared helpers + 1 setup + 1 launch + 1 init)
- **Configuration files**: 2 (package.xml + mediamtx.yml)
- **Resource files**: 1 (ROS2 resource marker)
- **Total lines of boilerplate**: updated from the original scaffold

All files follow PEP 8 conventions and include detailed docstrings for every class and method.
