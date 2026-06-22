#!/bin/bash
# quick_start_mock_telemetry.sh
# Quick start script for mock rover telemetry testing

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║     UCMTC Mock Rover Telemetry - Quick Start Guide            ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 environment not sourced!"
    echo ""
    echo "Please source your ROS2 environment first:"
    echo "  source /opt/ros/humble/setup.bash"
    echo ""
    echo "Then run this script again."
    exit 1
fi

echo "✓ ROS2 Distro: $ROS_DISTRO"
echo ""

# Check Python version
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
echo "✓ Python Version: $PYTHON_VERSION"
echo ""

# Check if rclpy is available
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "❌ Error: rclpy not found!"
    echo "Please ensure ROS2 is properly installed and sourced."
    exit 1
fi

echo "✓ rclpy available"
echo ""

# Confirm mock_rover_telemetry.py exists
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MOCK_SCRIPT="$SCRIPT_DIR/mock_rover_telemetry.py"

if [ ! -f "$MOCK_SCRIPT" ]; then
    echo "❌ Error: mock_rover_telemetry.py not found at $MOCK_SCRIPT"
    exit 1
fi

echo "✓ mock_rover_telemetry.py found"
echo ""

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                   STARTUP OPTIONS                              ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "1. Start mock telemetry publisher (foreground)"
echo "2. Start mock telemetry publisher (background)"
echo "3. Start mock telemetry + monitor topic"
echo "4. Start mock telemetry + launch rqt_graph"
echo "5. Just show the code"
echo "0. Exit"
echo ""

read -p "Select option (0-5): " choice

case $choice in
    1)
        echo ""
        echo "Starting mock telemetry publisher..."
        echo "Press Ctrl+C to stop."
        echo ""
        python3 "$MOCK_SCRIPT"
        ;;
    2)
        echo ""
        echo "Starting mock telemetry publisher in background..."
        python3 "$MOCK_SCRIPT" > /tmp/mock_telemetry.log 2>&1 &
        PID=$!
        echo "✓ Running with PID: $PID"
        echo "To stop: kill $PID"
        echo "To view logs: tail -f /tmp/mock_telemetry.log"
        echo ""
        ;;
    3)
        echo ""
        echo "Starting mock telemetry publisher..."
        echo ""
        python3 "$MOCK_SCRIPT" > /tmp/mock_telemetry.log 2>&1 &
        PUBLISHER_PID=$!
        sleep 2
        echo "✓ Publisher started (PID: $PUBLISHER_PID)"
        echo ""
        echo "Monitoring telemetry topic (Ctrl+C to stop)..."
        echo ""
        ros2 topic echo /rover/telemetry
        kill $PUBLISHER_PID 2>/dev/null
        ;;
    4)
        echo ""
        echo "Starting mock telemetry publisher..."
        python3 "$MOCK_SCRIPT" > /tmp/mock_telemetry.log 2>&1 &
        PUBLISHER_PID=$!
        sleep 2
        echo "✓ Publisher started (PID: $PUBLISHER_PID)"
        echo ""
        echo "Launching rqt_graph..."
        rqt_graph &
        RQT_PID=$!
        wait $RQT_PID
        kill $PUBLISHER_PID 2>/dev/null
        ;;
    5)
        echo ""
        head -50 "$MOCK_SCRIPT"
        echo ""
        echo "... (use 'less $MOCK_SCRIPT' to see full file)"
        ;;
    0)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid option!"
        exit 1
        ;;
esac

echo ""
