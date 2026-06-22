# Quick Reference: UCMTC Rover ROS2 Nodes

## Build & Run Commands

### Build the workspace
```bash
cd rover_ws
colcon build --packages-select rover_core
```

### Source the setup script
```bash
source rover_ws/install/setup.bash
```

### Run individual nodes
```bash
ros2 run rover_core navigation_node
ros2 run rover_core safety_node
ros2 run rover_core vision_node
ros2 run rover_core motor_control_node
ros2 run rover_core telemetry_aggregator
```

### Launch the rover stack together
```bash
ros2 launch rover_core rover_bringup.launch.py
```

### Monitor published topics
```bash
ros2 topic list
ros2 topic echo /rover/telemetry/nav
ros2 topic echo /rover/telemetry/safety
ros2 topic echo /rover/telemetry/vision
ros2 topic echo /rover/telemetry/control
ros2 topic echo /rover/telemetry
```

---

## Node Publishing Schedule

| Node | Topic | Rate | Payload Fields |
|------|-------|------|----------------|
| **Navigation** | `/rover/telemetry/nav` | 10 Hz (0.1s) | timestamp, latitude, longitude, heading, speed, altitude |
| **Safety** | `/rover/telemetry/safety` | 20 Hz (0.05s) | timestamp, estop_active, collision_detected, motor_override, safety_state |
| **Vision** | `/rover/telemetry/vision` | ~30 Hz (0.033s) | timestamp, frame_id, confidence, lanes_detected, obstacle_distance, processing_time_ms |
| **Motor Control** | `/rover/telemetry/control` | 5 Hz (0.2s) | node_motor_ctrl, heartbeat_seq, estop_latched, last_command |
| **Telemetry Aggregator** | `/rover/telemetry` | 10 Hz (0.1s) | canonical Navigation, Safety, Vision, Jetson, Communication, ROS payload |

---

## Implementation Checklist

### Navigation Node
- [ ] Wire GPS serial input on rover hardware
- [ ] Wire IMU serial input on rover hardware
- [ ] Tune odometry and heading smoothing against field data
- [ ] Verify `/rover/telemetry/nav` on the GCS

### Safety Node
- [ ] Wire GPIO inputs for e-stop and collision sensors
- [ ] Tune debounce thresholds and collision timing
- [ ] Validate command override behavior on the rover
- [ ] Verify `/rover/telemetry/safety` on the GCS

### Vision Node
- [ ] Wire the rover camera into the capture path
- [ ] Replace simulated inference with the onboard model
- [ ] Tune lane/obstacle thresholds and publish rate
- [ ] Verify `/rover/telemetry/vision` on the GCS

### Launch Configuration
- [x] Implement `generate_launch_description()` in `rover_bringup.launch.py`
- [x] Add Node action objects for all rover nodes
- [x] Configure namespace 'rover' for topic prefix
- [ ] Verify `ros2 launch rover_core rover_bringup.launch.py` on the Jetson

---

## Key Architecture Details

### Topic Naming Convention
- All rover topics use prefix: `/rover/`
- Telemetry topics: `/rover/telemetry/<subsystem>`
- Command topics: `/rover/commands/<subsystem>`
- Canonical GCS topic: `/rover/telemetry`
- Primary drive command topic: `/rover/commands/motor`

### DDS Middleware
- Default ROS2 DDS implementation will automatically handle network communication
- Telemetry payloads are published as standard ROS2 messages
- Remote GCS subscribes to these topics over network

### Safety-Critical Design
- SafetyNode runs at 20 Hz (twice as fast as others) for critical monitoring
- Can override motor commands if safety conditions violated
- E-stop is hardware-based (GPIO) for fail-safe operation

### Video Streaming
- MediaMTX server (separate process) handles video relay
- Vision node captures frames and processes them locally
- Video stream available at RTMP/HLS endpoints for GCS display

---

## File Locations Reference

```
/home/medochi/GS/
├── rover_ws/
│   └── src/
│       └── rover_core/
│           ├── rover_core/
│           │   ├── navigation_node.py        ← GPS/IMU telemetry
│           │   ├── safety_node.py            ← E-stop & collision detection
│           │   ├── vision_node.py            ← Camera & AI inference
│           │   ├── motor_control_node.py     ← Drive command heartbeat
│           │   ├── telemetry_aggregator.py   ← Canonical payload merger
│           │   ├── ros_compat.py             ← ROS fallback helpers
│           │   └── telemetry_utils.py       ← Shared telemetry helpers
│           ├── launch/
│           │   └── rover_bringup.launch.py  ← Launch all nodes
│           ├── config/
│           │   └── mediamtx.yml          ← Video streaming config
│           ├── setup.py                  ← Build configuration
│           └── package.xml               ← Package metadata
├── ROVER_STRUCTURE_SUMMARY.md            ← Full documentation
└── QUICK_REFERENCE.md                    ← This file
```

---

## Debugging Tips

### Check if nodes are running
```bash
ros2 node list
```

### Monitor CPU and memory
```bash
ros2 system-state
```

### Check message queue depths
```bash
ros2 topic info /rover/telemetry/nav --verbose
```

### Record rosbag for playback
```bash
ros2 bag record -a  # Record all topics
ros2 bag play rosbag2_2024_04_12  # Playback recording
```

### View ROS2 graph
```bash
rqt_graph
```

---

## Next Steps

1. Connect the rover hardware inputs for navigation, safety, and vision
2. Tune the live rover parameters for your field environment
3. Deploy to Jetson Nano and verify the launch stack
4. Connect the Ground Control Station over WiFi
5. Stress test the telemetry and control path under motion
