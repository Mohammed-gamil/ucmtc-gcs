"""Main window for the UCMTC Ground Control Station."""

from __future__ import annotations

import time
from typing import Any

from gcs_app.core.data_models import TelemetryPayload
from gcs_app.core.ros_worker import ROSWorker
from gcs_app.qt_compat import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QTimer,
    QVBoxLayout,
    QWidget,
)
from gcs_app.ui.atoms.status_led import StatusLED
from gcs_app.ui.molecules.telemetry_card import TelemetryCard
from gcs_app.ui.organisms.navigation_panel import NavigationPanel
from gcs_app.ui.organisms.safety_panel import SafetyPanel
from gcs_app.ui.organisms.video_dashboard import VideoDashboard


class MainWindow(QMainWindow):
    """Page: Main window holding all organisms and UI sections."""

    def __init__(self, parent: Any = None):
        super().__init__(parent)
        self.setWindowTitle("UCMTC Rover GCS")
        self.resize(1600, 960)

        self.worker = ROSWorker()
        self.worker.start()

        self.connection_led = StatusLED("Telemetry", "offline")
        self.safety_led = StatusLED("Safety", "idle")
        self.command_led = StatusLED("Command", "idle")
        self.error_led = StatusLED("Errors", "clear")

        self.navigation_card = NavigationPanel()
        self.safety_card = SafetyPanel()
        self.vision_card = VideoDashboard()
        self.jetson_card = TelemetryCard(
            "Jetson",
            fields=["cpu_pct", "gpu_pct", "ram_pct", "temp_c", "bat_pct", "bat_voltage", "uptime_sec"],
        )
        self.communication_card = TelemetryCard(
            "Communication",
            fields=["rtt_ms", "channel_rssi", "stream_fps", "packet_loss_pct", "heartbeat_seq", "timestamp_ms"],
        )
        self.ros_card = TelemetryCard(
            "ROS",
            fields=[
                "node_lane_det",
                "node_obs_avoid",
                "node_wp_nav",
                "node_img_recog",
                "node_motor_ctrl",
                "esp32_connected",
                "rosout_last",
            ],
        )

        self.speed_input = QLineEdit("5.0")
        self.heading_input = QLineEdit("0.0")
        self.throttle_input = QLineEdit("0.35")
        self.waypoint_input = QLineEdit("0")
        self.command_status_label = QLabel("No command sent yet")

        self.drive_button = QPushButton("Drive")
        self.drive_button.setObjectName("DriveButton")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("StopButton")
        self.estop_button = QPushButton("E-Stop")
        self.estop_button.setObjectName("EstopButton")
        self.resume_button = QPushButton("Resume")
        self.resume_button.setObjectName("ResumeButton")

        self.drive_button.clicked.connect(self.send_drive_command)
        self.stop_button.clicked.connect(self.send_stop_command)
        self.estop_button.clicked.connect(self.send_estop_command)
        self.resume_button.clicked.connect(self.send_resume_command)

        # Create alert banner with a default healthy state
        self.alert_banner = QLabel(" 🟢 SYSTEM STATUS: Operating normally - All metrics within safe parameters")
        self.alert_banner.setObjectName("AlertBanner")
        self.alert_banner.setStyleSheet(
            "background-color: #064e3b; color: #34d399; border: 1px solid #047857; border-radius: 8px; padding: 10px; font-weight: bold; font-size: 13px;"
        )

        self._build_ui()

        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.update_ui)
        self.render_timer.start(100)

        self.setStyleSheet(
            """
            QMainWindow {
                background: #0f172a;
                color: #e2e8f0;
            }
            QLabel {
                color: #e2e8f0;
                font-family: 'Segoe UI', Arial, Helvetica, sans-serif;
            }
            QLabel#CardTitle {
                font-weight: bold;
                font-size: 14px;
                color: #38bdf8;
                margin-bottom: 6px;
            }
            QPushButton {
                background-color: #1e293b;
                color: #f1f5f9;
                border: 1px solid #475569;
                border-radius: 8px;
                padding: 8px 16px;
                font-weight: bold;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QPushButton:hover {
                background-color: #334155;
            }
            QPushButton#DriveButton {
                background-color: #047857;
                border: 1px solid #059669;
                color: white;
            }
            QPushButton#DriveButton:hover {
                background-color: #059669;
            }
            QPushButton#StopButton {
                background-color: #b45309;
                border: 1px solid #d97706;
                color: white;
            }
            QPushButton#StopButton:hover {
                background-color: #d97706;
            }
            QPushButton#ResumeButton {
                background-color: #1d4ed8;
                border: 1px solid #2563eb;
                color: white;
            }
            QPushButton#ResumeButton:hover {
                background-color: #2563eb;
            }
            QPushButton#EstopButton {
                background-color: #b91c1c;
                border: 1px solid #dc2626;
                color: white;
                font-size: 13px;
            }
            QPushButton#EstopButton:hover {
                background-color: #dc2626;
            }
            QLineEdit {
                background-color: #111827;
                color: #e5e7eb;
                border: 1px solid #334155;
                border-radius: 8px;
                padding: 6px 8px;
            }
            QLineEdit:focus {
                border: 1px solid #38bdf8;
            }
            """
        )

    def _build_ui(self):
        root = QWidget()
        root_layout = QVBoxLayout()
        root_layout.setContentsMargins(16, 16, 16, 16)
        root_layout.setSpacing(12)

        header_layout = QHBoxLayout()
        title_block = QVBoxLayout()
        title_label = QLabel("UCMTC Rover GCS")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #38bdf8;")
        subtitle_label = QLabel("Telemetry and control over WiFi DDS")
        subtitle_label.setStyleSheet("font-size: 12px; color: #94a3b8;")
        title_block.addWidget(title_label)
        title_block.addWidget(subtitle_label)
        header_layout.addLayout(title_block)
        header_layout.addStretch(1)
        header_layout.addWidget(self.connection_led)
        header_layout.addWidget(self.safety_led)
        header_layout.addWidget(self.command_led)
        header_layout.addWidget(self.error_led)
        root_layout.addLayout(header_layout)

        # Insert Alert Banner below header
        root_layout.addWidget(self.alert_banner)

        cards_grid = QGridLayout()
        cards_grid.setSpacing(12)
        cards_grid.addWidget(self.navigation_card, 0, 0)
        cards_grid.addWidget(self.safety_card, 0, 1)
        cards_grid.addWidget(self.vision_card, 1, 0)
        cards_grid.addWidget(self.jetson_card, 1, 1)
        cards_grid.addWidget(self.communication_card, 2, 0)
        cards_grid.addWidget(self.ros_card, 2, 1)
        root_layout.addLayout(cards_grid)

        control_panel = QWidget()
        control_layout = QVBoxLayout()
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.setSpacing(8)

        control_title = QLabel("Control")
        control_layout.addWidget(control_title)

        field_grid = QGridLayout()
        field_grid.setSpacing(8)
        field_grid.addWidget(QLabel("Speed km/h"), 0, 0)
        field_grid.addWidget(self.speed_input, 0, 1)
        field_grid.addWidget(QLabel("Heading deg"), 0, 2)
        field_grid.addWidget(self.heading_input, 0, 3)
        field_grid.addWidget(QLabel("Throttle"), 1, 0)
        field_grid.addWidget(self.throttle_input, 1, 1)
        field_grid.addWidget(QLabel("Waypoint"), 1, 2)
        field_grid.addWidget(self.waypoint_input, 1, 3)
        control_layout.addLayout(field_grid)

        button_row = QHBoxLayout()
        button_row.addWidget(self.drive_button)
        button_row.addWidget(self.stop_button)
        button_row.addWidget(self.resume_button)
        button_row.addWidget(self.estop_button)
        button_row.addStretch(1)
        control_layout.addLayout(button_row)

        control_layout.addWidget(self.command_status_label)
        control_panel.setLayout(control_layout)
        root_layout.addWidget(control_panel)

        root.setLayout(root_layout)
        self.setCentralWidget(root)

    def _read_float(self, widget: QLineEdit, default: float) -> float:
        text = widget.text().strip()
        if not text:
            return default
        return float(text)

    def _read_int(self, widget: QLineEdit, default: int) -> int:
        text = widget.text().strip()
        if not text:
            return default
        return int(float(text))

    def _send_command(self, command: dict[str, Any], status_text: str):
        try:
            self.worker.send_motor_command(command)
        except Exception as exc:
            self.command_status_label.setText(f"Command error: {exc}")
            self.command_led.set_state("error", "Command")
            self.statusBar().showMessage(f"Command error: {exc}")
            return

        self.command_status_label.setText(status_text)
        self.command_led.set_state(command.get("action", "sent"), "Command")
        self.statusBar().showMessage(status_text)

    def send_drive_command(self):
        try:
            speed_kmh = self._read_float(self.speed_input, 0.0)
            heading_deg = self._read_float(self.heading_input, 0.0)
            throttle_pct = self._read_float(self.throttle_input, 0.35)
            waypoint = self._read_int(self.waypoint_input, 0)
        except ValueError as exc:
            self.command_status_label.setText(f"Invalid control value: {exc}")
            self.command_led.set_state("error", "Command")
            self.statusBar().showMessage(f"Invalid control value: {exc}")
            return

        # Operator error validations
        if speed_kmh < 0.0 or speed_kmh > 15.0:
            msg = f"Rejected: Speed {speed_kmh} out of bounds [0.0, 15.0]!"
            self.command_status_label.setText(msg)
            self.statusBar().showMessage(msg)
            return
        if heading_deg < 0.0 or heading_deg > 360.0:
            msg = f"Rejected: Heading {heading_deg} out of bounds [0.0, 360.0]!"
            self.command_status_label.setText(msg)
            self.statusBar().showMessage(msg)
            return
        if throttle_pct < 0.0 or throttle_pct > 1.0:
            msg = f"Rejected: Throttle {throttle_pct} out of bounds [0.0, 1.0]!"
            self.command_status_label.setText(msg)
            self.statusBar().showMessage(msg)
            return

        command = {
            "action": "drive",
            "speed_kmh": speed_kmh,
            "heading_deg": heading_deg,
            "throttle_pct": throttle_pct,
            "wp_current": waypoint,
            "source": "gcs",
            "timestamp_ms": int(time.time() * 1000),
        }
        self._send_command(command, f"Drive command sent: {speed_kmh:.1f} km/h @ {heading_deg:.1f} deg")

    def send_stop_command(self):
        command = {
            "action": "stop",
            "speed_kmh": 0.0,
            "heading_deg": self._read_float(self.heading_input, 0.0),
            "throttle_pct": 0.0,
            "source": "gcs",
            "timestamp_ms": int(time.time() * 1000),
        }
        self._send_command(command, "Stop command sent")

    def send_estop_command(self):
        command = {
            "action": "estop",
            "speed_kmh": 0.0,
            "heading_deg": self._read_float(self.heading_input, 0.0),
            "throttle_pct": 0.0,
            "estop_triggered": True,
            "source": "gcs",
            "timestamp_ms": int(time.time() * 1000),
        }
        self._send_command(command, "Emergency stop sent")

    def send_resume_command(self):
        command = {
            "action": "resume",
            "source": "gcs",
            "timestamp_ms": int(time.time() * 1000),
        }
        self._send_command(command, "Resume command sent")

    def _get_cardinal_direction(self, deg: float) -> str:
        deg = deg % 360.0
        directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        idx = int((deg + 11.25) / 22.5) % 16
        return directions[idx]

    def update_ui(self):
        """Render latest telemetry snapshot at a capped rate."""
        data = self.worker.get_latest_telemetry()
        is_connected = self.worker.is_connected()
        
        # Subsystem alerts check
        warnings = []
        if not is_connected:
            warnings.append("COMMUNICATION OFFLINE (No Heartbeat)")
        
        if data is not None:
            # Check safety states
            if data.safety.estop_triggered:
                warnings.append("EMERGENCY STOP TRIGGERED")
            if data.safety.collision_detected:
                warnings.append("COLLISION DETECTED")
            if data.safety.border_crossed:
                warnings.append("BORDER BREACHED")
            
            # Check Jetson temperature
            if data.jetson.temp_c > 75.0:
                warnings.append(f"HIGH JETSON TEMP ({data.jetson.temp_c}°C)")
            # Check Jetson battery
            if data.jetson.bat_pct < 15.0:
                warnings.append(f"LOW BATTERY ({data.jetson.bat_pct}%)")
            
            # Check Communication packet loss or RSSI
            if data.communication.packet_loss_pct > 8.0:
                warnings.append(f"HIGH PACKET LOSS ({data.communication.packet_loss_pct}%)")
            if data.communication.rtt_ms > 150:
                warnings.append(f"HIGH LATENCY ({data.communication.rtt_ms} ms)")
            
            # Check ROS Node Health
            dead_nodes = []
            if not data.ros.node_lane_det:
                dead_nodes.append("Vision/Lane")
            if not data.ros.node_obs_avoid:
                dead_nodes.append("Safety")
            if not data.ros.node_wp_nav:
                dead_nodes.append("Nav")
            if not data.ros.node_motor_ctrl:
                dead_nodes.append("Motor")
            if dead_nodes:
                warnings.append(f"ROS NODE FAULT ({', '.join(dead_nodes)})")
        
        if warnings:
            alert_text = " ⚠️ ALERT: " + " | ".join(warnings)
            self.alert_banner.setText(alert_text)
            self.alert_banner.setStyleSheet(
                "background-color: #7f1d1d; color: #fca5a5; border: 1px solid #b91c1c; border-radius: 8px; padding: 10px; font-weight: bold; font-size: 13px;"
            )
        else:
            self.alert_banner.setText(" 🟢 SYSTEM STATUS: Operating normally - All metrics within safe parameters")
            self.alert_banner.setStyleSheet(
                "background-color: #064e3b; color: #34d399; border: 1px solid #047857; border-radius: 8px; padding: 10px; font-weight: bold; font-size: 13px;"
            )

        if data is None:
            if is_connected:
                self.statusBar().showMessage("Connected to rover telemetry, waiting for first frame.")
                self.connection_led.set_state("connected", "Telemetry")
            else:
                self.statusBar().showMessage("Waiting for rover telemetry.")
                self.connection_led.set_state("offline", "Telemetry")
            last_error = self.worker.get_last_error()
            if last_error:
                self.error_led.set_state("error", "Errors")
                self.command_status_label.setText(last_error)
            else:
                self.error_led.set_state("clear", "Errors")
            return

        self._update_cards(data)
        self.connection_led.set_state("connected" if is_connected else "offline", "Telemetry")
        if data.safety.estop_triggered or data.safety.collision_detected:
            self.safety_led.set_state("alert", "Safety")
        else:
            self.safety_led.set_state(data.safety.mode, "Safety")

        last_error = self.worker.get_last_error()
        if last_error:
            self.error_led.set_state("error", "Errors")
        else:
            self.error_led.set_state("clear", "Errors")

        self.statusBar().showMessage(
            f"Speed {data.navigation.speed_kmh:.1f} km/h | Battery {data.jetson.bat_pct:.1f}% | RTT {data.communication.rtt_ms} ms"
        )

    def _update_cards(self, data: TelemetryPayload):
        cardinal = self._get_cardinal_direction(data.navigation.heading_deg)
        nav_dict = {
            "speed_kmh": f"{data.navigation.speed_kmh:.2f} km/h",
            "heading_deg": f"{data.navigation.heading_deg:.1f}° ({cardinal})",
            "pos_lat": f"{data.navigation.pos_lat:.6f}",
            "pos_lon": f"{data.navigation.pos_lon:.6f}",
            "dist_traveled_m": f"{data.navigation.dist_traveled_m:.1f} m",
            "wp_current": f"Waypoint {data.navigation.wp_current}",
            "wp_error_m": f"{data.navigation.wp_error_m:.2f} m",
            "wp_status": data.navigation.wp_status.upper(),
        }
        self.navigation_card.set_values(nav_dict)

        safety_dict = {
            "mode": data.safety.mode.upper(),
            "light_state": data.safety.light_state.upper(),
            "estop_mech_armed": "ARMED" if data.safety.estop_mech_armed else "DISARMED",
            "estop_wire_armed": "ARMED" if data.safety.estop_wire_armed else "DISARMED",
            "estop_triggered": "⚠️ TRIGGERED" if data.safety.estop_triggered else "CLEAR",
            "is_blocked": "🛑 BLOCKED" if data.safety.is_blocked else "CLEAR",
            "collision_detected": "💥 COLLISION!" if data.safety.collision_detected else "CLEAR",
            "border_crossed": "🚫 BREACHED" if data.safety.border_crossed else "CLEAR",
            "border_partial": "🔲 PARTIAL" if data.safety.border_partial else "CLEAR",
            "obstacle_touched": "⚠️ TOUCHED" if data.safety.obstacle_touched else "CLEAR",
        }
        self.safety_card.set_values(safety_dict)

        vision_dict = {
            "img_confidence": f"{data.vision.img_confidence * 100:.0f} %",
            "img_detected": "YES" if data.vision.img_detected else "NO",
            "laser_active": "ACTIVE" if data.vision.laser_active else "INACTIVE",
            "img_elapsed_sec": f"{data.vision.img_elapsed_sec} sec",
            "img_task_status": data.vision.img_task_status.upper(),
            "lane_detected": "YES" if data.vision.lane_detected else "NO",
            "obstacles_count": f"{data.vision.obstacles_count} detected",
            "fps_vision": f"{data.vision.fps_vision:.1f} FPS",
        }
        self.vision_card.set_values(vision_dict)

        jetson_dict = {
            "cpu_pct": f"{data.jetson.cpu_pct:.1f} %",
            "gpu_pct": f"{data.jetson.gpu_pct:.1f} %",
            "ram_pct": f"{data.jetson.ram_pct:.1f} %",
            "temp_c": f"{data.jetson.temp_c:.1f} °C",
            "bat_pct": f"{data.jetson.bat_pct:.1f} %",
            "bat_voltage": f"{data.jetson.bat_voltage:.2f} V",
            "uptime_sec": f"{data.jetson.uptime_sec} sec",
        }
        self.jetson_card.set_values(jetson_dict)

        comm_dict = {
            "rtt_ms": f"{data.communication.rtt_ms} ms",
            "channel_rssi": f"{data.communication.channel_rssi} dBm",
            "stream_fps": f"{data.communication.stream_fps:.1f} FPS",
            "packet_loss_pct": f"{data.communication.packet_loss_pct:.1f} %",
            "heartbeat_seq": f"#{data.communication.heartbeat_seq}",
            "timestamp_ms": f"{data.communication.timestamp_ms} ms",
        }
        self.communication_card.set_values(comm_dict)

        ros_dict = {
            "node_lane_det": "🟢 RUNNING" if data.ros.node_lane_det else "🔴 FAULT",
            "node_obs_avoid": "🟢 RUNNING" if data.ros.node_obs_avoid else "🔴 FAULT",
            "node_wp_nav": "🟢 RUNNING" if data.ros.node_wp_nav else "🔴 FAULT",
            "node_img_recog": "🟢 RUNNING" if data.ros.node_img_recog else "🔴 FAULT",
            "node_motor_ctrl": "🟢 RUNNING" if data.ros.node_motor_ctrl else "🔴 FAULT",
            "esp32_connected": "🟢 CONNECTED" if getattr(data.ros, "esp32_connected", False) else "🔴 DISCONNECTED",
            "rosout_last": data.ros.rosout_last,
        }
        self.ros_card.set_values(ros_dict)

    def closeEvent(self, event):
        """Stop timer and worker to avoid hanging ROS threads on exit."""
        self.render_timer.stop()
        self.worker.stop()
        if hasattr(event, "accept"):
            event.accept()

