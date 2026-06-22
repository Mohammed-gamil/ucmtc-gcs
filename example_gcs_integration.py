#!/usr/bin/env python3
"""
Example PyQt6 GCS Integration with Mock Rover Telemetry

This example demonstrates how to integrate the mock rover telemetry publisher
with a PyQt6 Ground Control Station UI. It shows best practices for:
1. Threading (keeping ROS callbacks off the Qt main thread)
2. Signal/slot communication between ROS and UI
3. JSON parsing and error handling
4. Real-time data display updates

This is NOT a complete GCS - just the telemetry subscription logic.
"""

import json
import sys
import threading
from typing import Optional

from gcs_app.qt_compat import QMainWindow, QThread, QTextEdit, QVBoxLayout, QWidget, QApplication, pyqtSignal


class RoverTelemetryReceiver(QThread):
    """
    QThread that subscribes to rover telemetry and emits signals to update the UI.
    
    This runs the ROS event loop in a separate thread to avoid blocking Qt's main thread.
    """
    
    # Signals emitted to update UI
    telemetry_received = pyqtSignal(dict)  # Full telemetry dict
    connection_status = pyqtSignal(bool)   # True when connected, False on error
    error_occurred = pyqtSignal(str)       # Error message
    
    def __init__(self):
        """Initialize the telemetry receiver thread."""
        super().__init__()
        self.running = True
        self.rclpy = None
        self.node = None
        
    def run(self):
        """
        Thread execution - runs the ROS event loop.
        
        This method blocks until stop() is called.
        """
        try:
            # Lazy import to allow running without ROS in some environments
            import rclpy
            from std_msgs.msg import String
            
            self.rclpy = rclpy
            
            # Initialize ROS2
            rclpy.init()
            self.node = rclpy.create_node('gcs_telemetry_receiver')
            from rclpy.qos import QoSProfile
            
            # Create subscription with latest-only QoS (drop old messages)
            self.node.create_subscription(
                String,
                '/rover/telemetry',
                self.telemetry_callback,
                qos_profile=QoSProfile(depth=1)
            )
            
            self.connection_status.emit(True)
            self.node.get_logger().info('Connected to /rover/telemetry')
            
            # Spin (block here, processing callbacks)
            while self.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
        except ImportError:
            self.error_occurred.emit('ROS2 not available - ensure environment is sourced')
            self.connection_status.emit(False)
        except Exception as e:
            self.error_occurred.emit(f'ROS2 connection failed: {str(e)}')
            self.connection_status.emit(False)
        finally:
            if self.node:
                self.node.destroy_node()
            if self.rclpy:
                self.rclpy.shutdown()
    
    def telemetry_callback(self, msg):
        """
        Callback executed when telemetry message is received.
        
        Parses JSON and emits signal to update UI.
        """
        try:
            telemetry = json.loads(msg.data)
            self.telemetry_received.emit(telemetry)
        except json.JSONDecodeError as e:
            self.error_occurred.emit(f'Failed to parse telemetry JSON: {str(e)}')
    
    def stop(self):
        """Signal the thread to stop and wait for it to exit."""
        self.running = False
        self.wait()


class GCSWindow(QMainWindow):
    """
    Simple PyQt6 window demonstrating telemetry display.
    
    In a real application, this would have:
    - Multiple gauges and status indicators
    - Real-time graphs
    - Control panels
    - Video display
    """
    
    def __init__(self):
        """Initialize the GCS window."""
        super().__init__()
        self.setWindowTitle('UCMTC Rover GCS - Mock Telemetry Demo')
        self.setGeometry(100, 100, 1200, 800)
        
        # Text display for telemetry (simple example)
        self.text_display = QTextEdit()
        self.text_display.setReadOnly(True)
        self.text_display.setFont(self.font())
        
        layout = QVBoxLayout()
        layout.addWidget(self.text_display)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)
        
        # Start telemetry receiver thread
        self.telemetry_receiver = RoverTelemetryReceiver()
        self.telemetry_receiver.telemetry_received.connect(self.on_telemetry_received)
        self.telemetry_receiver.connection_status.connect(self.on_connection_status)
        self.telemetry_receiver.error_occurred.connect(self.on_error)
        self.telemetry_receiver.start()
        
        # Statistics
        self.message_count = 0
        self.last_speed = 0.0
        self.last_battery = 100.0
    
    def on_telemetry_received(self, telemetry: dict):
        """
        Slot called when telemetry is received.
        
        This executes in the Qt main thread (thread-safe).
        """
        self.message_count += 1
        
        try:
            # Extract key data for display
            nav = telemetry.get('Navigation', {})
            safety = telemetry.get('Safety', {})
            jetson = telemetry.get('Jetson', {})
            comms = telemetry.get('Communication', {})
            
            self.last_speed = nav.get('speed_kmh', 0.0)
            self.last_battery = jetson.get('bat_pct', 100.0)
            
            # Format display text
            display = f"""
╔════════════════════════════════════════════════════════════════╗
║           UCMTC ROVER TELEMETRY - Message #{self.message_count}              ║
╚════════════════════════════════════════════════════════════════╝

NAVIGATION:
  Speed:           {nav.get('speed_kmh', 0):.2f} km/h
  Heading:         {nav.get('heading_deg', 0):.1f}°
  Position:        {nav.get('pos_lat', 0):.6f}, {nav.get('pos_lon', 0):.6f}
  Distance:        {nav.get('dist_traveled_m', 0):.1f} m
  Waypoint:        {nav.get('wp_current', 0)} ({nav.get('wp_status', 'idle')})

SAFETY:
  Mode:            {safety.get('mode', 'unknown')}
  Light:           {safety.get('light_state', 'unknown')}
  Collision:       {'🔴 COLLISION DETECTED!' if safety.get('collision_detected') else '✓ Clear'}
  E-Stop:          {'⚠️  ARMED' if safety.get('estop_mech_armed') else '✓ Disarmed'}

JETSON:
  CPU:             {jetson.get('cpu_pct', 0):.1f}%
  GPU:             {jetson.get('gpu_pct', 0):.1f}%
  RAM:             {jetson.get('ram_pct', 0):.1f}%
  Temperature:     {jetson.get('temp_c', 0):.1f}°C
  Battery:         {jetson.get('bat_pct', 0):.1f}% ({jetson.get('bat_voltage', 0):.2f}V)
  Uptime:          {jetson.get('uptime_sec', 0)}s

COMMUNICATION:
  RTT:             {comms.get('rtt_ms', 0)} ms
  Signal:          {comms.get('channel_rssi', 0)} dBm
  Packet Loss:     {comms.get('packet_loss_pct', 0):.1f}%
  Heartbeat:       #{comms.get('heartbeat_seq', 0)}

════════════════════════════════════════════════════════════════
Last updated: {comms.get('timestamp_ms', 0)} ms
            """
            
            self.text_display.setText(display)
            
        except Exception as e:
            self.text_display.setText(f'Error parsing telemetry: {str(e)}\n\nRaw: {json.dumps(telemetry, indent=2)}')
    
    def on_connection_status(self, connected: bool):
        """Handle connection status changes."""
        status = 'Connected ✓' if connected else 'Disconnected ✗'
        self.statusBar().showMessage(f'ROS Status: {status}')
    
    def on_error(self, error_msg: str):
        """Handle errors."""
        self.statusBar().showMessage(f'Error: {error_msg}')
    
    def closeEvent(self, event):
        """Cleanup when window is closed."""
        self.telemetry_receiver.stop()
        event.accept()


def main():
    """Entry point for the example GCS application."""
    try:
        app = QApplication(sys.argv)
        window = GCSWindow()
        window.show()
        sys.exit(app.exec())
    except ImportError as e:
        print(f'Error: PyQt6 not installed. Install with: pip install PyQt6')
        sys.exit(1)


if __name__ == '__main__':
    main()
