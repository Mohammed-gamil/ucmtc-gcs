# UCMTC Tactical Ground Control Station (GCS) — Run Guide

This document outlines the startup, execution, and testing procedures for the UCMTC Ground Control Station (GCS) and ROS 2 rover simulation stack.

---

## 🗺️ Execution Modes Overview

The GCS can be run in two primary modes:
1. **Standalone Simulation Mode (No ROS 2 Required)**: Employs an internal, physics-based software-in-the-loop generator to feed the dashboard and team mesh.
2. **Full ROS 2 Mode (Humble LTS)**: Sourced ROS 2 nodes publishing and subscribing to active DDS topics over the network, utilizing strict QoS profiles.

---

## ⚡ The Quickest Way: Interactive TUI Dashboard (Zero-Touch Standby Bringup)

We have included an automated Terminal User Interface (TUI) to orchestrate and manage all GCS processes. **It performs a zero-touch build, source, and server launch sequence automatically on startup, opening in a quiet standby state.**

### Start the TUI
To boot the GCS server and log monitor in a single action, run:
```bash
./gcs_tui_runner.py
```
*(If execution permission is needed: `chmod +x gcs_tui_runner.py`)*

### What the TUI Automated Startup Does:
1. **Environment Check**: Scans the system for a local ROS 2 Humble installation.
2. **Auto-Build (ROS 2 Mode)**: If ROS 2 is found but the workspace setup file is missing, it runs `colcon build --symlink-install` and sources it automatically.
3. **Standby Server Launch**:
   * **ROS 2 Mode**: Launches the ROS 2 Web GCS Server Bridge in the background. It does **not** launch the local simulator nodes by default, meaning the GUI will start in a clean standby disconnected state until you connect a real rover or launch the bringup stack.
   * **Simulation Fallback Mode**: Launches the Web GCS Server in a quiet standby mode (`--no-sim`), keeping all telemetry numbers at zero/static.
4. **Logs & Control Terminal**: Boots into the curses dashboard, showing the active GCS Server as `[RUNNING]` and streaming its logs.

### TUI Hotkeys and Controls:
* **`[Arrow Up / Arrow Down]`**: Navigate between project components.
* **`[Enter / Space]`**: Toggle individual components on or off. You can start the simulation nodes by selecting `ROS 2 Rover Bringup` or `Mock Telemetry Pub` and pressing Space.
* **`[PgUp / PgDn]`**: Scroll through the active console logs of the selected process in the right-hand panel.
* **`[O]`**: **Open GUI Browser** (Pings the default browser to open/reload the Web GCS dashboard).
* **`[K]`**: Panic-kill all active background processes.
* **`[Q] or [ESC]`**: Safely terminate all processes and exit the TUI runner.

---

## 1. Standalone Simulation Mode (No ROS 2)

If you are running the project on a base station/machine without a local ROS 2 installation:

```bash
# 1. Source the python virtual environment
source .venv/bin/activate

# 2. Run the HTTP SSE server in fallback mode
python web_gcs/web_gcs_server.py
```

### Dashboard Access
Once the Web Backend is running, open your web browser and navigate to:
* **[http://localhost:8082](http://localhost:8082)**

To access the console from another laptop or device connected to the same WiFi:
* Check your machine's IP address: `hostname -I` or `ip addr show`
* Open browser at: `http://<YOUR-IP-ADDRESS>:8082`

---

## 2. Full ROS 2 Humble Mode (Real/Simulation Bringup)

In this mode, all 5 telemetry nodes compile, initialize, and stream data over the ROS 2 DDS middleware.

### Step 1: Sourcing & Building the Workspace
Ensure ROS 2 Humble is installed. Open a terminal and run:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to ROS 2 Workspace
cd rover_ws

# Build using symlink-install (keeps python nodes active without rebuilds)
colcon build --symlink-install

# Source the newly built workspace overlay
source install/setup.bash
```

### Step 2: Bring Up Rover Node Stack
Using the built bringup launch file (which automatically loads environment parameters and sets up QoS configurations):
```bash
ros2 launch rover_core rover_bringup.launch.py
```

### Step 3: Run GCS Server Bridge (Web Backend & GUI host)
Open a new terminal session, source the environment, and start the application:
```bash
source /opt/ros/humble/setup.bash
source rover_ws/install/setup.bash
.venv/bin/python web_gcs/web_gcs_server.py
```
Open **[http://localhost:8082](http://localhost:8082)** to access the GUI dashboard.

---

## 🔗 SSH Telemetry Tunneling (Connecting Remote Rovers)

If the GCS and the rover are on different networks (restricted WiFi or across the internet), you can tunnel the Global Connector TCP data stream over SSH.

### Step 1: Configure Environment (for TUI)
Before launching the TUI runner, export your SSH parameters:
```bash
export GCS_SSH_HOST="203.0.113.5"    # Remote Rover IP
export GCS_SSH_USER="ubuntu"         # Remote Rover SSH Username
export GCS_SSH_LOCAL_PORT="8091"     # Port forwarded on local GCS (default: 8091)
export GCS_SSH_REMOTE_PORT="8090"    # Peer data port on Rover (default: 8090)
```
*Note: Key-based SSH authentication must be set up beforehand so the background process can authenticate without a password prompt.*

### Step 2: Launch the Tunnel
Start the TUI runner:
```bash
./gcs_tui_runner.py
```
Select **`SSH Tunnel to Rover`** in the menu and press **`[Space/Enter]`**. If it connects successfully, the status badge will update to `[RUNNING]`. If it fails, check `ssh_tunnel.log` in the right log panel.

*(Alternatively, run the tunnel manually: `ssh -N -L 8091:localhost:8090 ubuntu@203.0.113.5`)*

### Step 3: Register the Peer on GCS Dashboard
1. Open the Web GCS dashboard: `http://localhost:8082`
2. Scroll to the **Global Team Mesh** panel.
3. Register the peer manually:
   * **Peer ID**: `remote-rover`
   * **IP Address**: `127.0.0.1` (points to the local end of the SSH tunnel)
   * **Port**: `8091` (the local forwarded port)
   * **Role**: `Rover`
4. Click **CONNECT PEER**. The GCS will now securely stream telemetry from the remote rover over SSH.

---

## 🧪 Simulation Tools & Test Verification

### Telemetry Mock Publisher (Alternative Sim)
To test ROS 2 node subscriptions without running the full rover node loop, you can run a standalone telemetry simulator:
```bash
source /opt/ros/humble/setup.bash
.venv/bin/python mock_rover_telemetry.py
```

### Run Python Unit/Integration Tests
To verify all GCS endpoints, mesh socket connections, and backend boundary safety parameter validations, run the test suite:
```bash
.venv/bin/python -m unittest tests/test_gcs_rover.py -v
```

---

## 🔌 Network Ports Reference

The GCS uses specific communication channels:

| Port | Protocol | Usage | Description |
|---|---|---|---|
| **`8082`** | TCP/HTTP | Web Dashboard | Hosts the HTML console page (the Web GUI) and the SSE (`/events`) stream. |
| **`8090`** | TCP | Global Mesh Link | Dedicated connection port for receiving telemetry updates from team peers. |
| **`8091`** | TCP | SSH Telemetry Forwarding | Default local port for SSH tunnels linking remote peer streams. |
| **`9876`** | UDP | Peer Discovery | Dynamic broadcast channel for auto-discovering mesh units on the subnet. |

---

## 🛡️ Troubleshooting & Failsafes

### Port Conflicts
If you receive a `BindError` or "Port already in use":
* Run `lsof -i :8082` or `lsof -i :8090` to find the blocking PID.
* Kill the process using: `kill -9 <PID>`

### Orphaned Nodes
If ROS 2 nodes remain active in the background after closing processes:
```bash
pkill -f rover_bringup.launch.py
pkill -f telemetry_aggregator
pkill -f navigation_node
pkill -f safety_node
pkill -f vision_node
pkill -f motor_control_node
pkill -f web_gcs_server.py
```
*(Note: Running processes through the TUI automatically prevents this by using custom Unix Process Groups).*
