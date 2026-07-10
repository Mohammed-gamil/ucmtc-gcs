# UCMTC Ground Control Station — Complete Documentation

> **Version 2.5** · WiFi DDS Telemetry Link · Global Team Mesh  
> Last updated: June 2026

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Project Structure](#2-project-structure)
3. [Prerequisites](#3-prerequisites)
4. [Quick Start Guide](#4-quick-start-guide)
5. [Running the Web Dashboard](#5-running-the-web-dashboard)
6. [Running the ROS2 Rover Stack](#6-running-the-ros2-rover-stack)
7. [Connecting to Your Team's ROS System](#7-connecting-to-your-teams-ros-system)
8. [Global Connector — Team Mesh](#8-global-connector--team-mesh)
9. [Dashboard Guide](#9-dashboard-guide)
10. [REST API Reference](#10-rest-api-reference)
11. [Telemetry Schema](#11-telemetry-schema)
12. [Sending Commands](#12-sending-commands)
13. [Testing](#13-testing)
14. [Troubleshooting](#14-troubleshooting)
15. [Environment Variables](#15-environment-variables)
16. [Network Ports Reference](#16-network-ports-reference)

---

## 1. System Overview

The UCMTC GCS is a **real-time ground control station** for rover operations at university robotics challenges. It provides:

- **Live telemetry dashboard** — Navigation, safety, vision, compute health, and link quality
- **Operator command console** — Drive, stop, resume, and emergency stop controls
- **Global team mesh** — Connect to other team members' ROS systems over WiFi
- **ROS2 integration** — Full subscriber/publisher bridge with `rclpy`
- **Fallback simulation** — Runs without ROS2 installed using built-in mock telemetry

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        UCMTC GCS v2.5                          │
│                                                                 │
│  ┌──────────────┐    ┌──────────────────┐   ┌───────────────┐  │
│  │  Web Browser  │◄──│  HTTP Server     │──►│  ROS2 Bridge  │  │
│  │  Dashboard    │   │  :8082           │   │  Node         │  │
│  │  (HTML/JS)    │   │  SSE Stream      │   │  /rover/*     │  │
│  └──────────────┘    └────────┬─────────┘   └───────┬───────┘  │
│                               │                      │          │
│                     ┌─────────▼──────────┐          │          │
│                     │ Global Connector    │          │          │
│                     │ UDP :9876 discover  │          │          │
│                     │ TCP :8090 data      │          │          │
│                     └─────────┬──────────┘          │          │
│                               │                      │          │
└───────────────────────────────┼──────────────────────┼──────────┘
                                │                      │
                    ┌───────────▼───────┐    ┌────────▼────────┐
                    │  Team Peers       │    │  Rover Hardware  │
                    │  (rovers, drones, │    │  Nav / Safety /  │
                    │   base stations)  │    │  Vision / Motor  │
                    └───────────────────┘    └─────────────────┘
```

---

## 2. Project Structure

```
GS/
├── web_gcs/                          # Web dashboard (frontend + server)
│   ├── web_gcs_server.py             # HTTP/SSE server with ROS2 bridge
│   ├── index.html                    # Dashboard HTML
│   ├── styles.css                    # Sci-fi neon theme CSS
│   └── app.js                        # Real-time telemetry renderer
│
├── gcs_app/                          # Core GCS Python application
│   ├── core/
│   │   ├── data_models.py            # Strict telemetry dataclasses
│   │   ├── ros_worker.py             # ROS-to-Qt telemetry bridge
│   │   └── global_connector.py       # Team mesh network connector
│   ├── ui/                           # Desktop Qt UI (optional)
│   └── main.py                       # Desktop entry point
│
├── rover_ws/                         # ROS2 workspace
│   └── src/rover_core/               # ROS2 package
│       ├── rover_core/
│       │   ├── navigation_node.py    # GPS/IMU telemetry node
│       │   ├── safety_node.py        # E-stop and collision node
│       │   ├── vision_node.py        # Camera AI inference node
│       │   ├── motor_control_node.py # Drive command heartbeat node
│       │   ├── telemetry_aggregator.py # Canonical payload merger
│       │   ├── telemetry_utils.py    # Payload builder helpers
│       │   └── ros_compat.py         # ROS2/fallback compatibility
│       ├── launch/
│       │   └── rover_bringup.launch.py
│       ├── setup.py
│       └── package.xml
│
├── tests/
│   └── test_gcs_rover.py            # 29 unit tests
│
├── telemetry_payload.schema.json     # JSON Schema for telemetry
├── mock_rover_telemetry.py           # Standalone mock telemetry script
└── DOCS.md                           # ← This file
```

---

## 3. Prerequisites

### Minimum Requirements
- **Python 3.10+**
- **Modern web browser** (Chrome, Firefox, Edge)
- **Network access** (WiFi or Ethernet for team connections)

### Optional (for full ROS2 mode)
- **ROS2 Humble** (or later) installed
- **colcon** build tools
- **rclpy** and **std_msgs** packages

> **Note:** The system runs in **fallback simulation mode** when ROS2 is not installed. All dashboard features work with simulated telemetry.

---

## 4. Quick Start Guide

### Fastest way to see the dashboard running:

```bash
# 1. Navigate to the project
cd ~/GS

# 2. Start the web server (works without ROS2)
python web_gcs/web_gcs_server.py

# 3. Open your browser
# → http://localhost:8082
```

That's it! The dashboard will show simulated rover telemetry immediately.

---

## 5. Running the Web Dashboard

### Standard Start (Simulation Mode)

```bash
cd ~/GS
python web_gcs/web_gcs_server.py
```

This starts:
- **HTTP server** on port `8082` — serves the dashboard
- **SSE stream** on `/events` — pushes real-time telemetry
- **Global Connector** — UDP discovery on `9876`, TCP listener on `8090`
- **Mock telemetry** — simulated rover data if ROS2 is unavailable

### With ROS2 (Real Rover Mode)

```bash
# Terminal 1: Source ROS2 and launch rover nodes
source /opt/ros/humble/setup.bash
cd ~/GS/rover_ws
colcon build --packages-select rover_core
source install/setup.bash
ros2 launch rover_core rover_bringup.launch.py

# Terminal 2: Start the web dashboard
source /opt/ros/humble/setup.bash
cd ~/GS
python web_gcs/web_gcs_server.py
```

The dashboard will subscribe to `/rover/telemetry` and show real data.

### Accessing the Dashboard

Open any browser and go to:
```
http://localhost:8082
```

To access from another device on the same network:
```
http://<your-ip-address>:8082
```

Find your IP with: `hostname -I` or `ip addr show`

---

## 6. Running the ROS2 Rover Stack

### Build the Workspace

```bash
cd ~/GS/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_core
source install/setup.bash
```

### Launch All Nodes

```bash
ros2 launch rover_core rover_bringup.launch.py
```

This starts 5 nodes:

| Node | Topic Published | Function |
|------|----------------|----------|
| `navigation_node` | `/rover/telemetry/nav` | GPS position, speed, heading, waypoints |
| `safety_node` | `/rover/telemetry/safety` | E-stop states, collision, geofence |
| `vision_node` | `/rover/telemetry/vision` | AI confidence, lane detection, obstacles |
| `motor_control_node` | `/rover/telemetry/control` | Drive command heartbeat |
| `telemetry_aggregator` | `/rover/telemetry` | Merges all above into canonical payload |

### Run Individual Nodes

```bash
ros2 run rover_core navigation_node
ros2 run rover_core safety_node
ros2 run rover_core vision_node
ros2 run rover_core motor_control_node
ros2 run rover_core telemetry_aggregator
```

### Monitor Topics

```bash
# Watch the merged telemetry stream
ros2 topic echo /rover/telemetry

# Watch a specific section
ros2 topic echo /rover/telemetry/nav

# Check active topics
ros2 topic list

# Check node health
ros2 node list
```

---

## 7. Connecting to Your Team's ROS System

There are **three ways** to connect your GCS to a teammate's ROS system:

### Method A: Same ROS Domain (Easiest)

If both your GCS and your team's rover are on the **same WiFi network**, ROS2 DDS auto-discovers them by default:

```bash
# On BOTH machines, use the same domain ID (default is 0)
export ROS_DOMAIN_ID=0

# On your GCS machine
python web_gcs/web_gcs_server.py

# On the team's rover machine
ros2 launch rover_core rover_bringup.launch.py
```

The dashboard will automatically receive telemetry via DDS multicast.

### Method B: Global Connector — Manual Peer (via Dashboard UI)

1. Start the GCS: `python web_gcs/web_gcs_server.py`
2. Open `http://localhost:8082`
3. Scroll to the **"Global Team Mesh"** panel
4. Fill in the form:
   - **Peer ID**: Any unique label (e.g., `rover-alpha`, `drone-2`)
   - **IP Address**: Your teammate's IP (e.g., `192.168.1.50`)
   - **Port**: Their connector port (default `8090`)
   - **Role**: Select `Rover`, `Drone`, `Base Station`, etc.
   - **Team Name**: Optional label
5. Click **CONNECT PEER**

The peer will appear in the live peer list below the form.

### Method C: Global Connector — Programmatic (Python Script)

```python
from gcs_app.core.global_connector import get_global_connector

# Get the singleton connector
connector = get_global_connector()
connector.start()

# Add a team member's system
connector.add_manual_peer(
    peer_id="team-rover",
    ip="192.168.1.50",
    port=8090,
    role="rover",
    team_name="UCMTC-Alpha"
)

# Check connected peers
print(connector.get_peers_snapshot())

# Get their telemetry
telemetry = connector.get_peer_telemetry("team-rover")
print(telemetry)
```

### Method D: Auto-Discovery (Both Running Connector)

If both systems run the Global Connector, they discover each other automatically via UDP broadcast:

```bash
# On Machine A (your GCS)
python web_gcs/web_gcs_server.py

# On Machine B (team's system) — just needs to run ANY script that starts the connector
python -c "
from gcs_app.core.global_connector import get_global_connector
c = get_global_connector(role='rover', team_name='TeamB')
c.start()
import time
while True: time.sleep(1)
"
```

Both will discover each other within 2-4 seconds on the same WiFi/LAN subnet.

### Method E: SSH Tunneling (Connecting Remote Peers Across Internet/Subnets)

If the rover and GCS are on different networks (e.g., across the internet, behind firewalls/NATs, or on restricted university WiFi), you can tunnel the TCP data stream over SSH:

1. **Establish the SSH Tunnel**:
   Forward local port `8091` to the remote rover's Global Connector TCP listener port `8090` (assumes SSH key authentication is configured):
   ```bash
   ssh -N -L 8091:localhost:8090 user@remote-rover-ip
   ```
   *(Alternatively, toggle **`SSH Tunnel to Rover`** in the TUI after configuring `GCS_SSH_HOST` / `GCS_SSH_USER` environment variables).*

2. **Register the Tunnel Peer**:
   On your local GCS dashboard (`http://localhost:8082`), go to the **Global Team Mesh** panel and add the peer manually:
   - **Peer ID**: `remote-rover` (or any unique label)
   - **IP Address**: `127.0.0.1` (points to the local end of the tunnel)
   - **Port**: `8091` (the local forwarded port)
   - **Role**: `Rover`
   - Click **CONNECT PEER**.

The local GCS will establish a secure connection through the SSH tunnel, and remote telemetry data will start flowing.

---

## 8. Global Connector — Team Mesh

### How It Works

The Global Connector runs 4 background threads:

| Thread | Function |
|--------|----------|
| **UDP Discovery Listener** | Listens on port `9876` for peer announcements |
| **UDP Announcer** | Broadcasts "I'm here" packets every 2 seconds |
| **TCP Listener** | Accepts incoming data connections on port `8090` |
| **Maintenance Loop** | Checks peer health, marks stale/dead peers |

### Peer Connection Lifecycle

```
DISCOVERED → CONNECTING → CONNECTED → (data flowing)
                              ↓
                            STALE → DISCONNECTED
                              ↓
                            ERROR (reconnect attempt)
```

### Peer Data Protocol

Peers exchange newline-delimited JSON over TCP:

**Handshake (first message):**
```json
{"type": "handshake", "peer_id": "rover-1", "role": "rover", "team_name": "UCMTC"}
```

**Telemetry (continuous):**
```json
{"type": "telemetry", "peer_id": "rover-1", "timestamp": 1718964396.5, "data": { ... }}
```

### Configuration via Environment Variables

```bash
# Change the discovery broadcast port (default: 9876)
export GCS_DISCOVERY_PORT=9999

# Then start the server
python web_gcs/web_gcs_server.py
```

---

## 9. Dashboard Guide

The dashboard is divided into a **header**, **alerts bar**, and **3-column grid**:

### Header Bar
| Indicator | Description |
|-----------|-------------|
| **TELEMETRY** | Connection status to the rover (green=online, red=offline) |
| **SAFETY MODE** | Current safety state (monitoring/idle/alert) |
| **TEAM MESH** | Number of connected team peers |
| **RTT LATENCY** | Round-trip time to the rover in milliseconds |
| **BATTERY** | Rover battery percentage with visual bar |

### Column 1 — Navigation & Commands
- **Compass** — Live heading dial with cardinal direction
- **Kinematics** — Speed, odometry, GPS position, waypoint status
- **Command Console** — Speed/heading sliders, throttle, waypoint selector
- **Action Buttons** — DRIVE, STOP, RESUME
- **Emergency Stop** — Large red E-STOP button

### Column 2 — Compute & Network
- **Compute Health** — CPU, GPU, RAM bars with temperature and uptime
- **Real-time Chart** — Live CPU%, temperature, and packet loss graph
- **ROS Nodes** — Green/red status dots for each ROS node
- **Team Mesh Panel** — Add peers, view connected peers with live status

### Column 3 — Safety & Vision
- **Safety Diagnostics** — E-stop states, collision, geofence status
- **Computer Vision** — AI confidence dial, lane detection, obstacle count
- **Link Quality** — RSSI signal strength, packet loss, video FPS

---

## 10. REST API Reference

All endpoints support CORS and return JSON.

### Telemetry Stream

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/events` | `GET` | SSE stream of real-time telemetry + peer data |

**SSE payload format:**
```json
{
  "telemetry": { "Navigation": {...}, "Safety": {...}, ... },
  "connected": true,
  "peers": [{"peer_id": "rover-1", "status": "connected", ...}],
  "peer_count": 2,
  "peers_connected": 1
}
```

### Commands

| Endpoint | Method | Body | Description |
|----------|--------|------|-------------|
| `/command` | `POST` | `{"action": "drive", "speed_kmh": 5.0, ...}` | Send motor command |

**Valid actions:** `drive`, `stop`, `resume`, `estop`

### Peer Management

| Endpoint | Method | Body | Description |
|----------|--------|------|-------------|
| `/api/peers` | `GET` | — | List all peers |
| `/api/peers` | `POST` | `{"peer_id": "...", "ip_address": "...", "port": 8090, "role": "rover"}` | Add manual peer |
| `/api/peers/{id}` | `DELETE` | — | Remove peer |
| `/api/peers/{id}/telemetry` | `GET` | — | Get peer's latest telemetry |
| `/api/peers/telemetry/all` | `GET` | — | Get merged telemetry from all sources |
| `/api/connector/status` | `GET` | — | Connector health status |

### Example: Add a Peer via cURL

```bash
curl -X POST http://localhost:8082/api/peers \
  -H "Content-Type: application/json" \
  -d '{
    "peer_id": "rover-alpha",
    "ip_address": "192.168.1.50",
    "port": 8090,
    "role": "rover",
    "team_name": "UCMTC"
  }'
```

### Example: List All Peers

```bash
curl http://localhost:8082/api/peers
```

### Example: Send a Drive Command

```bash
curl -X POST http://localhost:8082/command \
  -H "Content-Type: application/json" \
  -d '{
    "action": "drive",
    "speed_kmh": 5.0,
    "heading_deg": 180.0,
    "throttle_pct": 0.5,
    "wp_current": 3,
    "source": "curl",
    "timestamp_ms": 1718964396000
  }'
```

### Example: Emergency Stop via cURL

```bash
curl -X POST http://localhost:8082/command \
  -H "Content-Type: application/json" \
  -d '{"action": "estop", "speed_kmh": 0, "throttle_pct": 0, "estop_triggered": true}'
```

---

## 11. Telemetry Schema

The canonical telemetry payload has 6 top-level sections. Full JSON Schema is in `telemetry_payload.schema.json`.

```json
{
  "Navigation": {
    "speed_kmh": 4.5,
    "heading_deg": 120.5,
    "pos_lat": 40.012345,
    "pos_lon": -75.012345,
    "dist_traveled_m": 12.4,
    "wp_current": 2,
    "wp_error_m": 0.5,
    "wp_status": "navigating"
  },
  "Safety": {
    "mode": "monitoring",
    "light_state": "green",
    "estop_mech_armed": false,
    "estop_wire_armed": false,
    "estop_triggered": false,
    "is_blocked": false,
    "collision_detected": false,
    "border_crossed": false,
    "border_partial": false,
    "obstacle_touched": false
  },
  "Vision": {
    "img_confidence": 0.85,
    "img_detected": true,
    "laser_active": false,
    "img_elapsed_sec": 120,
    "img_task_status": "tracking",
    "lane_detected": true,
    "obstacles_count": 2,
    "fps_vision": 29.5
  },
  "Jetson": {
    "cpu_pct": 45.2,
    "gpu_pct": 20.0,
    "ram_pct": 60.5,
    "temp_c": 50.4,
    "bat_pct": 95.0,
    "bat_voltage": 11.8,
    "uptime_sec": 300
  },
  "Communication": {
    "rtt_ms": 25,
    "channel_rssi": -60,
    "stream_fps": 29.5,
    "packet_loss_pct": 0.1,
    "heartbeat_seq": 102,
    "timestamp_ms": 300000
  },
  "ROS": {
    "node_lane_det": true,
    "node_obs_avoid": true,
    "node_wp_nav": true,
    "node_img_recog": true,
    "node_motor_ctrl": true,
    "rosout_last": "All systems nominal"
  }
}
```

---

## 12. Sending Commands

### Command Format

Commands are JSON objects with an `action` field:

| Action | Parameters | Description |
|--------|-----------|-------------|
| `drive` | `speed_kmh`, `heading_deg`, `throttle_pct`, `wp_current` | Start driving |
| `stop` | — | Gentle stop (coast to halt) |
| `resume` | — | Resume from stop |
| `estop` | — | Emergency stop (immediate halt, latches) |

### Safety Bounds (enforced server-side)

| Parameter | Min | Max |
|-----------|-----|-----|
| `speed_kmh` | 0.0 | 15.0 |
| `heading_deg` | 0.0 | 360.0 |
| `throttle_pct` | 0.0 | 1.0 |

Commands outside these bounds are rejected with a `400` error.

### ROS Topic

Commands are published to: `/rover/commands/motor` as `std_msgs/String` JSON.

---

## 13. Testing

### Run All Tests

```bash
cd ~/GS
python -m unittest tests.test_gcs_rover -v
```

### Expected Output

```
29 tests in 0.5s — OK

Test breakdown:
  TestTelemetryDataModels ........... 6 tests
  TestROSWorker ..................... 2 tests
  TestRoverNodesCompatibility ....... 5 tests
  TestWebGCSServer .................. 2 tests
  TestGlobalConnector .............. 14 tests
```

### What's Tested

- ✅ Telemetry schema parsing and validation
- ✅ Missing/extra key detection
- ✅ Type coercion enforcement
- ✅ ROS worker fallback telemetry generation
- ✅ Command queue processing
- ✅ All 5 ROS nodes (navigation, safety, vision, motor, aggregator)
- ✅ E-stop latching behavior
- ✅ Web server configuration
- ✅ Command bounds validation
- ✅ Global connector peer management
- ✅ UDP announce packet build/parse
- ✅ Peer telemetry storage and retrieval
- ✅ Merged telemetry structure

---

## 14. Troubleshooting

### Dashboard shows "OFFLINE"

| Cause | Fix |
|-------|-----|
| Server not running | Start with `python web_gcs/web_gcs_server.py` |
| Wrong URL | Use `http://localhost:8082` (not https) |
| Port in use | Check with `lsof -i :8082` and kill the conflicting process |

### No telemetry data (blank values)

| Cause | Fix |
|-------|-----|
| ROS2 not sourced | Run `source /opt/ros/humble/setup.bash` before starting |
| Nodes not launched | Run `ros2 launch rover_core rover_bringup.launch.py` |
| No ROS2 installed | This is fine — the server runs in simulation mode automatically |

### Peer won't connect

| Cause | Fix |
|-------|-----|
| Wrong IP | Verify with `ping <peer-ip>` |
| Firewall blocking | Open ports 8090 (TCP) and 9876 (UDP) |
| Peer not running connector | They need to run a script with `GlobalConnector.start()` |
| Different subnet | Auto-discovery only works on the same LAN broadcast domain |

### Port already in use

```bash
# Find what's using the port
lsof -i :8082   # HTTP dashboard
lsof -i :8090   # TCP connector
lsof -i :9876   # UDP discovery

# Kill it
kill -9 <PID>
```

### ROS2 build errors

```bash
cd ~/GS/rover_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_core
source install/setup.bash
```

---

## 15. Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `GCS_DISCOVERY_PORT` | `9876` | UDP port for peer auto-discovery |
| `ROS_DOMAIN_ID` | `0` | ROS2 DDS domain (set same on all machines) |

---

## 16. Network Ports Reference

| Port | Protocol | Service | Direction |
|------|----------|---------|-----------|
| `8082` | TCP/HTTP | Web dashboard + SSE + REST API | Inbound |
| `8090` | TCP | Global Connector data stream | Inbound + Outbound |
| `9876` | UDP | Peer auto-discovery broadcast | Inbound + Outbound |
| `7400-7500` | UDP | ROS2 DDS (auto-assigned) | Bidirectional |

### Firewall Rules (if needed)

```bash
# Allow dashboard access
sudo ufw allow 8082/tcp

# Allow team mesh connections
sudo ufw allow 8090/tcp
sudo ufw allow 9876/udp

# Allow ROS2 DDS
sudo ufw allow 7400:7500/udp
```

---

## Quick Reference Card

```
┌─────────────────────────────────────────────┐
│          UCMTC GCS — Quick Start            │
├─────────────────────────────────────────────┤
│                                             │
│  Start dashboard:                           │
│    python web_gcs/web_gcs_server.py         │
│                                             │
│  Open browser:                              │
│    http://localhost:8082                     │
│                                             │
│  Run tests:                                 │
│    python -m unittest tests.test_gcs_rover  │
│                                             │
│  Add team peer (API):                       │
│    curl -X POST localhost:8082/api/peers \   │
│      -H "Content-Type: application/json" \  │
│      -d '{"peer_id":"r1",                   │
│           "ip_address":"192.168.1.50",      │
│           "port":8090, "role":"rover"}'     │
│                                             │
│  Emergency stop (API):                      │
│    curl -X POST localhost:8082/command \     │
│      -H "Content-Type: application/json" \  │
│      -d '{"action":"estop"}'                │
│                                             │
│  Launch ROS2 nodes:                         │
│    ros2 launch rover_core                   │
│         rover_bringup.launch.py             │
│                                             │
└─────────────────────────────────────────────┘
```
