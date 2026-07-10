#!/usr/bin/env bash

# ANSI Color Codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}🧹 Clearing Ground Station & ROS 2 environment cleanly...${NC}"

# 1. Kill active Python web server or telemetry runners
echo "Stopping GCS web servers and simulators..."
pkill -f gcs_gui_runner.py || true
pkill -f web_gcs_server.py || true
pkill -f mock_rover_telemetry.py || true
pkill -f live_publisher.py || true

# 2. Free up GCS bound ports
echo "Releasing bound TCP ports..."
fuser -k 8082/tcp &>/dev/null || true
fuser -k 8090/tcp &>/dev/null || true

# 3. Kill all ROS 2 nodes and launch systems
echo "Stopping ROS 2 nodes and launchers..."
pkill -f _node || true
pkill -f ros2 || true
pkill -f rover_bringup.launch.py || true

# Kill specific package nodes
pkill -f motor_control_node || true
pkill -f navigation_node || true
pkill -f safety_node || true
pkill -f vision_node || true
pkill -f telemetry_aggregator || true

# 4. Stop ROS 2 DDS Daemon to clear cache
echo "Stopping ROS 2 background daemon..."
if command -v ros2 &> /dev/null; then
    # Source ROS environment to run the daemon command if needed
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    ros2 daemon stop &>/dev/null || true
fi

# 5. Final check
echo -e "${GREEN}✓ All GCS and ROS 2 processes terminated successfully!${NC}"
echo -e "${YELLOW}🚀 You can now start the Ground Station fresh using:${NC}"
echo -e "   ./gcs_gui_runner.py"
