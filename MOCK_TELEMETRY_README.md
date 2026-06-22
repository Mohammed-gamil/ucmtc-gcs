# Mock Rover Telemetry Publisher

## Overview

`mock_rover_telemetry.py` is a standalone ROS2 node that publishes simulated rover telemetry at **100 Hz** to the `/rover/telemetry` topic. It's designed to stress-test your PyQt6 Ground Control Station before actual rover hardware is available.

The node publishes JSON-serialized telemetry payloads that perfectly match the rover's real telemetry schema, with realistic simulated sensor data including:
- Smooth navigation tracking (speed, heading, GPS position)
- System monitoring (CPU, GPU, RAM, battery, temperature)
- Network performance metrics (RTT, packet loss, RSSI)
- Safety state transitions (collision detection events)
- Vision processing metrics (confidence, lane detection, obstacles)
- ROS node health status

---

## Features

### Realistic Data Simulation

1. **Navigation** - Smooth random walk
   - Speed fluctuates smoothly between 0-15 km/h using sinusoidal variation
   - Heading rotates continuously (0-360°, wrapping)
   - GPS position increments based on speed and heading
   - Distance traveled accumulates
   - Waypoint system with status changes every 3 seconds

2. **Jetson System Metrics**
   - CPU: Hovers 55-85% with occasional spikes (5% chance)
   - GPU: Correlates with CPU (~60% of CPU load)
   - RAM: Slowly increases over time (simulating memory leak)
   - Temperature: Correlates with CPU load
   - Battery: Continuous drain (~0.1% per second)
   - Uptime: Real elapsed time since startup

3. **Network Simulation**
   - RTT: Normally 15-30 ms, occasional spikes to 100-200 ms
   - Packet Loss: Mostly 0%, occasional spikes to 2-8%
   - Channel RSSI: -50 to -80 dBm (typical WiFi range)
   - Stream FPS: ~30 FPS with jitter
   - Heartbeat sequence counter

4. **Safety Events**
   - Collision detection triggers every ~5 seconds
   - Collision state persists for ~200ms (realistic sensor settling time)
   - Light state changes (red when collision detected)

5. **Vision Processing**
   - Image confidence: Gaussian around 0.7 when detected
   - Lane detection: Random state changes every 2 seconds
   - Obstacle count: 0-5 obstacles when detected
   - Vision FPS: ~30 FPS with realistic jitter

6. **ROS Node Health**
   - Simulates occasional node failures (1% chance per node per second)
   - Last rosout message for debugging

---

## Installation & Setup

### Prerequisites

1. **ROS2 Installed** - Any recent distro (Humble, Iron, Jazzy)
   ```bash
   # Example for Humble on Ubuntu 22.04
   sudo apt install ros-humble-desktop
   ```

2. **Python Environment**
   - Python 3.8 or higher
   - `rclpy` (included with ROS2)
   - Standard library modules (json, math, random, time)

3. **Source ROS2 Environment**
   ```bash
   source /opt/ros/humble/setup.bash
   # or your corresponding ROS2 distro
   ```

### Setup Steps

1. **Place the script in your workspace**
   ```bash
   cp mock_rover_telemetry.py ~/GS/
   chmod +x ~/GS/mock_rover_telemetry.py
   ```

2. **Ensure script is executable**
   ```bash
   chmod +x ~/GS/mock_rover_telemetry.py
   ```

3. **Verify ROS2 is working**
   ```bash
   ros2 node list  # Should show any running nodes
   ```

---

## Usage

### Run the Mock Publisher

```bash
# Method 1: Direct execution
python3 ~/GS/mock_rover_telemetry.py

# Method 2: If executable
~/GS/mock_rover_telemetry.py

# Method 3: Using ROS2 run (if in colcon workspace)
ros2 run your_package mock_rover_telemetry
```

### Monitor Published Topics

In a separate terminal (with ROS2 sourced):

```bash
# List all topics
ros2 topic list

# Monitor the telemetry topic
ros2 topic echo /rover/telemetry

# Get message rate info
ros2 topic hz /rover/telemetry

# Get payload size
ros2 topic bw /rover/telemetry
```

### Monitor with rqt

```bash
# Launch rqt GUI
rqt

# Select Plugins > Topics > Topic Monitor
# Subscribe to /rover/telemetry
```

---

## Publishing Details

### Topic Information

- **Topic Name**: `/rover/telemetry`
- **Message Type**: `std_msgs/String`
- **Publishing Rate**: 100 Hz (10 ms intervals)
- **Expected Bandwidth**: ~500 KB/s

### JSON Payload Schema

```json
{
  "Navigation": {
    "speed_kmh": 0.0,           // Current speed (0-15 km/h)
    "heading_deg": 0.0,         // Heading (0-360°)
    "pos_lat": 0.0,             // Latitude in decimal degrees
    "pos_lon": 0.0,             // Longitude in decimal degrees
    "dist_traveled_m": 0.0,     // Cumulative distance (meters)
    "wp_current": 0,            // Current waypoint index
    "wp_error_m": 0.0,          // Distance error to waypoint (meters)
    "wp_status": "idle"         // Waypoint status (idle/navigating/reached)
  },
  "Safety": {
    "mode": "idle",             // Safety mode (idle/monitoring)
    "light_state": "idle",      // Light indicator (idle/green/red)
    "estop_mech_armed": false,  // Mechanical e-stop armed
    "estop_wire_armed": false,  // Wire break e-stop armed
    "estop_triggered": false,   // E-stop activated
    "is_blocked": false,        // Rover blocked/stalled
    "collision_detected": false,// Collision detected (flashes periodically)
    "border_crossed": false,    // Boundary crossed
    "border_partial": false,    // Partially crossed boundary
    "obstacle_touched": false   // Obstacle contact detected
  },
  "Vision": {
    "img_confidence": 0.0,      // AI confidence (0.0-1.0)
    "img_detected": false,      // Object detected in frame
    "laser_active": false,      // Laser rangefinder active
    "img_elapsed_sec": 0,       // Processing elapsed time
    "img_task_status": "idle",  // Task status (idle/processing)
    "lane_detected": false,     // Lane markings detected
    "obstacles_count": 0,       // Number of detected obstacles
    "fps_vision": 0.0           // Vision processing FPS
  },
  "Jetson": {
    "cpu_pct": 0.0,             // CPU usage (0-100%)
    "gpu_pct": 0.0,             // GPU usage (0-100%)
    "ram_pct": 0.0,             // RAM usage (0-100%)
    "temp_c": 0.0,              // Temperature (°C)
    "bat_pct": 100.0,           // Battery percentage (0-100%)
    "bat_voltage": 12.0,        // Battery voltage (V)
    "uptime_sec": 0             // System uptime (seconds)
  },
  "Communication": {
    "rtt_ms": 0,                // Round-trip time (milliseconds)
    "channel_rssi": 0,          // Signal strength (dBm, -80 to -50)
    "stream_fps": 0.0,          // Video stream FPS
    "packet_loss_pct": 0.0,     // Packet loss percentage
    "heartbeat_seq": 0,         // Heartbeat sequence number
    "timestamp_ms": 0           // Timestamp in milliseconds
  },
  "ROS": {
    "node_lane_det": true,      // Lane detection node alive
    "node_obs_avoid": true,     // Obstacle avoidance node alive
    "node_wp_nav": true,        // Waypoint navigation node alive
    "node_img_recog": true,     // Image recognition node alive
    "node_motor_ctrl": true,    // Motor control node alive
    "rosout_last": "..."        // Last log message
  }
}
```

---

## Testing Your GCS

### Stress Testing Checklist

- [ ] **Fast Publishing Rate**: Verify GCS handles 100 Hz updates without lag
- [ ] **Thread Safety**: Check that telemetry parsing doesn't block UI thread
- [ ] **JSON Parsing**: Ensure JSON deserialization is fast and doesn't crash
- [ ] **Collision Flashing**: Visually confirm red alert when collision_detected toggles
- [ ] **Battery Drain**: Verify battery percentage decreases over time
- [ ] **GPS Movement**: Confirm latitude/longitude values update smoothly
- [ ] **Network Spikes**: Observe RTT jumps to 100+ ms and verify UI shows warning
- [ ] **Node Failures**: Check that node status indicators toggle occasionally
- [ ] **CPU/Memory Trending**: Verify graphs track increasing RAM over time
- [ ] **Message Queue**: Monitor `/rover/telemetry` topic doesn't back up

### Performance Monitoring

```bash
# Monitor publication rate and latency
watch -n 1 'ros2 topic hz /rover/telemetry'

# Check message size
ros2 topic bw /rover/telemetry

# Monitor with full diagnostics
ros2 doctor
```

---

## Customization

### Adjust Publishing Rate

Edit line in `timer_callback()`:
```python
timer_period = 0.01  # Change to 0.05 for 20 Hz, 0.02 for 50 Hz, etc.
```

### Modify Data Ranges

Edit the simulation parameters in `generate_telemetry()`:

```python
# Navigation
base_speed = 5.0 + 8.0 * ...  # Adjust min/max speed

# Battery drain rate
self.battery_pct -= 0.001  # Increase number to drain faster

# Collision frequency
if self.tick_count % 500 == 0:  # Change 500 to increase/decrease frequency

# CPU baseline
self.cpu_base = max(55, min(85, ...))  # Adjust CPU range
```

### Add More Simulation Features

- Implement circular path navigation (heading slowly rotates)
- Add GPS drift simulation
- Simulate temperature spike when CPU spikes
- Add correlation between speed and CPU load
- Introduce message dropouts for network simulation

---

## Troubleshooting

### Node won't start

**Problem**: `ModuleNotFoundError: No module named 'rclpy'`

**Solution**: Source ROS2 environment
```bash
source /opt/ros/humble/setup.bash
python3 mock_rover_telemetry.py
```

### No messages published

**Problem**: Topic `/rover/telemetry` shows no activity

**Solution**: Check if middleware is configured
```bash
echo $ROS_DISTRO
ros2 doctor
```

### High CPU usage

**Problem**: Script consuming 50%+ CPU

**Solution**: This is normal at 100 Hz with JSON serialization. Reduce rate if needed:
```python
timer_period = 0.02  # 50 Hz instead of 100 Hz
```

### GCS missing messages

**Problem**: GCS not receiving all published telemetry

**Solution**: Increase publisher queue depth in script:
```python
self.publisher_ = self.create_publisher(String, '/rover/telemetry', 100)  # Increase from 10
```

---

## Performance Expectations

### System Impact

- **CPU Usage**: 5-15% on modern systems (depends on JSON parsing overhead)
- **Memory Usage**: ~50-100 MB for the node
- **Network Bandwidth**: ~500 KB/s for JSON strings at 100 Hz
- **Latency**: <5 ms between publish and subscription receive

### Example Output

```
[INFO] [1712973450.123456789] [mock_rover_telemetry]: Mock Rover Telemetry Publisher initialized (100 Hz)
[DEBUG] [1712973451.234567890] [mock_rover_telemetry]: Published telemetry #100: Speed=5.32 km/h, Heading=12.3°, Battery=99.9%
[DEBUG] [1712973452.345678901] [mock_rover_telemetry]: Published telemetry #200: Speed=7.81 km/h, Heading=25.6°, Battery=99.8%
[DEBUG] [1712973453.456789012] [mock_rover_telemetry]: Published telemetry #300: Speed=4.12 km/h, Heading=38.9°, Battery=99.7%
```

---

## Integration with PyQt6 GCS

### Example Subscriber Code

```python
from rclpy.subscription import Subscription
from std_msgs.msg import String

class RoverTelemetrySubscriber:
    def __init__(self):
        self.subscription: Subscription = node.create_subscription(
            String,
            '/rover/telemetry',
            self.telemetry_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1)  # Latest-only
        )
    
    def telemetry_callback(self, msg: String):
        """Parse incoming telemetry JSON."""
        try:
            telemetry = json.loads(msg.data)
            # Update UI with telemetry data
            self.update_dashboard(telemetry)
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse telemetry: {e}")
```

### Threading Considerations

- Wrap subscriber in `QThread` to avoid blocking UI
- Use signals/slots to communicate between ROS thread and Qt main thread
- Implement circular buffer to decouple publishing rate from UI update rate

---

## License & Attribution

This mock publisher is provided as-is for testing and development purposes.
Feel free to modify and extend for your specific needs.
